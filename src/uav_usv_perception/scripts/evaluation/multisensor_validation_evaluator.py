#!/usr/bin/env python3
"""Evaluate GT, LiDAR, camera, and fused tracks in Shadow Mode."""

from collections import deque
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _position(tracked):
    point = tracked.pose.pose.position
    return (float(point.x), float(point.y), float(point.z))


def _velocity(tracked):
    vector = tracked.twist.twist.linear
    return (float(vector.x), float(vector.y), float(vector.z))


def _distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _mean(values):
    return sum(values) / len(values) if values else None


def _source_name(mask):
    names = []
    for bit, name in (
        (TrackedObject.SOURCE_LIDAR, 'LIDAR'),
        (TrackedObject.SOURCE_CAMERA, 'CAMERA'),
        (TrackedObject.SOURCE_AIS, 'AIS'),
        (TrackedObject.SOURCE_FUSED, 'FUSED'),
    ):
        if int(mask) & int(bit):
            names.append(name)
    return '+'.join(names) if names else 'GROUND_TRUTH'


class SourceStatistics:
    def __init__(self, window_size):
        self.detections = deque(maxlen=window_size)
        self.position_errors = deque(maxlen=window_size)
        self.velocity_errors = deque(maxlen=window_size)
        self.timestamp_deltas = deque(maxlen=window_size)
        self.confidences = deque(maxlen=window_size)
        self.last_track_id = ''
        self.id_switches = 0
        self.matched_samples = 0

    def add(self, truth, tracked):
        self.detections.append(1.0 if tracked is not None else 0.0)
        if tracked is None:
            return
        self.position_errors.append(
            _distance(_position(truth), _position(tracked))
        )
        self.velocity_errors.append(
            _distance(_velocity(truth), _velocity(tracked))
        )
        truth_stamp = _stamp_seconds(truth.last_update)
        track_stamp = _stamp_seconds(tracked.last_update)
        if truth_stamp > 0.0 and track_stamp > 0.0:
            self.timestamp_deltas.append(
                abs(track_stamp - truth_stamp) * 1000.0
            )
        self.confidences.append(float(tracked.confidence))
        if self.last_track_id and tracked.track_id != self.last_track_id:
            self.id_switches += 1
        self.last_track_id = tracked.track_id
        self.matched_samples += 1

    def summary(self):
        continuity = 1.0
        if self.matched_samples > 1:
            continuity = max(
                0.0,
                1.0 - self.id_switches / float(self.matched_samples - 1),
            )
        return {
            'samples': len(self.detections),
            'matched_samples': self.matched_samples,
            'detection_rate': _mean(self.detections),
            'mean_position_error_m': _mean(self.position_errors),
            'mean_velocity_error_mps': _mean(self.velocity_errors),
            'mean_timestamp_delta_ms': _mean(self.timestamp_deltas),
            'mean_confidence': _mean(self.confidences),
            'id_switches': self.id_switches,
            'id_continuity': continuity,
            'last_track_id': self.last_track_id,
        }


class MultisensorValidationEvaluator(Node):
    SOURCES = ('ground_truth', 'lv_dot', 'uav_camera', 'fusion')

    def __init__(self):
        super().__init__('multisensor_validation_evaluator')
        defaults = {
            'ground_truth': '/perception/ground_truth/tracks',
            'lv_dot': '/perception/lv_dot/observations',
            'uav_camera': '/perception/uav_01/observations',
            'fusion': '/perception/fused/tracks',
        }
        for name, topic in defaults.items():
            self.declare_parameter(name + '_topic', topic)
        self.declare_parameter(
            'metrics_topic', '/perception/multisensor/metrics'
        )
        self.declare_parameter('target_id', 'target_vessel')
        self.declare_parameter('association_distance_m', 12.0)
        self.declare_parameter('max_timestamp_delta_seconds', 0.5)
        self.declare_parameter('online_timeout_seconds', 1.5)
        self.declare_parameter('window_size', 1000)
        self.declare_parameter('evaluation_rate_hz', 10.0)

        self.topics = {
            name: str(self.get_parameter(name + '_topic').value)
            for name in self.SOURCES
        }
        self.target_id = str(self.get_parameter('target_id').value)
        self.association_distance = max(
            0.1, float(self.get_parameter('association_distance_m').value)
        )
        self.max_timestamp_delta = max(
            0.01,
            float(
                self.get_parameter('max_timestamp_delta_seconds').value
            ),
        )
        self.online_timeout = max(
            0.1,
            float(self.get_parameter('online_timeout_seconds').value),
        )
        window_size = max(10, int(self.get_parameter('window_size').value))
        rate = max(
            1.0, float(self.get_parameter('evaluation_rate_hz').value)
        )
        metrics_topic = str(self.get_parameter('metrics_topic').value)

        self.messages = {name: None for name in self.SOURCES}
        self.arrivals = {name: 0.0 for name in self.SOURCES}
        self.statistics = {
            name: SourceStatistics(window_size)
            for name in ('lv_dot', 'uav_camera', 'fusion')
        }
        self.associations = deque(maxlen=window_size)
        self.last_truth_stamp = None
        self.publisher = self.create_publisher(String, metrics_topic, 10)
        for name, topic in self.topics.items():
            self.create_subscription(
                TrackedObjectArray,
                topic,
                lambda message, source=name: self._on_tracks(
                    source, message
                ),
                10,
            )
        self.create_timer(1.0 / rate, self._evaluate)
        self.get_logger().info(
            'Multisensor Shadow evaluator -> %s' % metrics_topic
        )

    def _on_tracks(self, source, message):
        self.messages[source] = message
        self.arrivals[source] = time.monotonic()

    def _online(self, source, now):
        return (
            self.messages[source] is not None
            and now - self.arrivals[source] <= self.online_timeout
        )

    def _truth(self):
        message = self.messages['ground_truth']
        if message is None:
            return None
        for tracked in message.objects:
            if tracked.track_id == self.target_id:
                return tracked
        return message.objects[0] if message.objects else None

    def _match(self, source, truth):
        message = self.messages[source]
        if message is None or truth is None:
            return None
        truth_stamp = _stamp_seconds(truth.last_update)
        truth_position = _position(truth)
        candidates = []
        for tracked in message.objects:
            stamp = _stamp_seconds(tracked.last_update)
            if stamp <= 0.0:
                stamp = _stamp_seconds(message.header.stamp)
            delta = abs(stamp - truth_stamp)
            if truth_stamp > 0.0 and delta > self.max_timestamp_delta:
                continue
            distance = _distance(truth_position, _position(tracked))
            if distance <= self.association_distance:
                candidates.append((delta, distance, tracked))
        if not candidates:
            return None
        preferred_ids = []
        if source == 'fusion':
            preferred_ids.append(self.target_id)
        last_id = self.statistics[source].last_track_id
        if last_id:
            preferred_ids.append(last_id)
        for track_id in preferred_ids:
            selected = [
                item for item in candidates if item[2].track_id == track_id
            ]
            if selected:
                return min(selected, key=lambda item: item[:2])[2]
        return min(candidates, key=lambda item: item[:2])[2]

    @staticmethod
    def _row(tracked, truth, online):
        if tracked is None:
            return {
                'online': online,
                'track_id': '',
                'position': None,
                'velocity': None,
                'source': 'UNKNOWN',
                'source_mask': 0,
                'confidence': None,
                'position_error_m': None,
                'velocity_error_mps': None,
                'timestamp': None,
                'timestamp_delta_ms': None,
            }
        stamp = _stamp_seconds(tracked.last_update)
        truth_stamp = _stamp_seconds(truth.last_update)
        return {
            'online': online,
            'track_id': tracked.track_id,
            'position': list(_position(tracked)),
            'velocity': list(_velocity(tracked)),
            'source': _source_name(tracked.source_mask),
            'source_mask': int(tracked.source_mask),
            'confidence': float(tracked.confidence),
            'position_error_m': _distance(
                _position(truth), _position(tracked)
            ),
            'velocity_error_mps': _distance(
                _velocity(truth), _velocity(tracked)
            ),
            'timestamp': stamp,
            'timestamp_delta_ms': (
                abs(stamp - truth_stamp) * 1000.0
                if stamp > 0.0 and truth_stamp > 0.0 else None
            ),
        }

    def _evaluate(self):
        now = time.monotonic()
        truth = self._truth() if self._online('ground_truth', now) else None
        matches = {
            name: self._match(name, truth)
            for name in ('lv_dot', 'uav_camera', 'fusion')
        } if truth is not None else {
            'lv_dot': None, 'uav_camera': None, 'fusion': None
        }
        truth_stamp = _stamp_seconds(truth.last_update) if truth else 0.0
        if (
            truth is not None
            and truth_stamp > 0.0
            and truth_stamp != self.last_truth_stamp
        ):
            self.last_truth_stamp = truth_stamp
            for name, tracked in matches.items():
                self.statistics[name].add(truth, tracked)
            fusion = matches['fusion']
            mask = int(fusion.source_mask) if fusion is not None else 0
            self.associations.append(bool(
                mask & TrackedObject.SOURCE_LIDAR
                and mask & TrackedObject.SOURCE_CAMERA
                and mask & TrackedObject.SOURCE_FUSED
            ))

        if truth is None:
            truth_row = self._row(
                None, None, self._online('ground_truth', now)
            )
        else:
            truth_row = self._row(truth, truth, True)
        sources = {'ground_truth': truth_row}
        for name, tracked in matches.items():
            sources[name] = self._row(
                tracked, truth, self._online(name, now)
            ) if truth is not None else self._row(
                None, None, self._online(name, now)
            )

        fusion = matches['fusion']
        fusion_mask = int(fusion.source_mask) if fusion is not None else 0
        payload = {
            'mode': 'multisensor_shadow_validation',
            'target_id': self.target_id,
            'control_source': 'ground_truth',
            'control_output_published': False,
            'sources': sources,
            'association': {
                'ground_truth_track_id': (
                    truth.track_id if truth is not None else ''
                ),
                'lv_dot_track_id': (
                    matches['lv_dot'].track_id
                    if matches['lv_dot'] is not None else ''
                ),
                'uav_camera_track_id': (
                    matches['uav_camera'].track_id
                    if matches['uav_camera'] is not None else ''
                ),
                'fusion_track_id': (
                    fusion.track_id if fusion is not None else ''
                ),
                'fusion_source_mask': fusion_mask,
                'fusion_sources': _source_name(fusion_mask),
                'all_sources_associated': bool(
                    fusion_mask & TrackedObject.SOURCE_LIDAR
                    and fusion_mask & TrackedObject.SOURCE_CAMERA
                    and fusion_mask & TrackedObject.SOURCE_FUSED
                ),
                'association_rate': _mean(self.associations),
            },
            'summary': {
                name: statistics.summary()
                for name, statistics in self.statistics.items()
            },
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=True, sort_keys=True)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MultisensorValidationEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
