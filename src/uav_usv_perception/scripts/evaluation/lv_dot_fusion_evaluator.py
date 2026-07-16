#!/usr/bin/env python3
"""Evaluate ground truth, LV-DOT observations, and fused Shadow tracks."""

from collections import deque
from dataclasses import dataclass, field
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


def _source_name(source_mask):
    names = []
    for bit, name in (
        (TrackedObject.SOURCE_LIDAR, 'LIDAR'),
        (TrackedObject.SOURCE_CAMERA, 'CAMERA'),
        (TrackedObject.SOURCE_AIS, 'AIS'),
        (TrackedObject.SOURCE_FUSED, 'FUSED'),
    ):
        if int(source_mask) & int(bit):
            names.append(name)
    return '+'.join(names) if names else 'UNKNOWN'


def _classification_name(classification):
    return {
        TrackedObject.CLASS_UNKNOWN: 'UNKNOWN',
        TrackedObject.CLASS_VESSEL: 'VESSEL',
        TrackedObject.CLASS_BUOY: 'BUOY',
        TrackedObject.CLASS_DEBRIS: 'DEBRIS',
        TrackedObject.CLASS_LANDMARK: 'LANDMARK',
    }.get(int(classification), 'UNKNOWN')


@dataclass
class SourceState:
    message: TrackedObjectArray = None
    arrival: float = 0.0


@dataclass
class RollingMetrics:
    detections: deque = field(default_factory=deque)
    position_errors: deque = field(default_factory=deque)
    velocity_errors: deque = field(default_factory=deque)
    latencies_ms: deque = field(default_factory=deque)
    confidences: deque = field(default_factory=deque)
    last_track_id: str = ''
    id_switches: int = 0
    matched_samples: int = 0

    @classmethod
    def with_window(cls, size):
        return cls(
            detections=deque(maxlen=size),
            position_errors=deque(maxlen=size),
            velocity_errors=deque(maxlen=size),
            latencies_ms=deque(maxlen=size),
            confidences=deque(maxlen=size),
        )

    def add(self, matched, position_error=None, velocity_error=None,
            latency_ms=None, confidence=None):
        self.detections.append(1.0 if matched is not None else 0.0)
        if matched is None:
            return
        self.position_errors.append(float(position_error))
        self.velocity_errors.append(float(velocity_error))
        if latency_ms is not None:
            self.latencies_ms.append(float(latency_ms))
        self.confidences.append(float(confidence))
        if self.last_track_id and matched.track_id != self.last_track_id:
            self.id_switches += 1
        self.last_track_id = matched.track_id
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
            'mean_latency_ms': _mean(self.latencies_ms),
            'mean_confidence': _mean(self.confidences),
            'id_switches': self.id_switches,
            'id_continuity': continuity,
            'last_track_id': self.last_track_id,
        }


class LvDotFusionEvaluator(Node):
    def __init__(self):
        super().__init__('lv_dot_fusion_evaluator')
        self.declare_parameter(
            'ground_truth_topic', '/perception/ground_truth/tracks'
        )
        self.declare_parameter(
            'observation_topic', '/perception/lv_dot/observations'
        )
        self.declare_parameter(
            'fusion_topic', '/perception/lv_dot/fused_tracks'
        )
        self.declare_parameter(
            'metrics_topic', '/perception/lv_dot/fusion_metrics'
        )
        self.declare_parameter('target_id', 'target_vessel')
        self.declare_parameter('association_distance_m', 12.0)
        self.declare_parameter('max_timestamp_delta_seconds', 0.5)
        self.declare_parameter('online_timeout_seconds', 1.5)
        self.declare_parameter('window_size', 1000)
        self.declare_parameter('evaluation_rate_hz', 10.0)

        self.topics = {
            'ground_truth': str(
                self.get_parameter('ground_truth_topic').value
            ),
            'lv_dot': str(self.get_parameter('observation_topic').value),
            'fusion': str(self.get_parameter('fusion_topic').value),
        }
        metrics_topic = str(self.get_parameter('metrics_topic').value)
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

        self.states = {name: SourceState() for name in self.topics}
        self.metrics = {
            name: RollingMetrics.with_window(window_size)
            for name in ('lv_dot', 'fusion')
        }
        self.last_evaluated_stamp = None
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
            'Shadow fusion evaluator GT=%s LV-DOT=%s Fusion=%s -> %s'
            % (
                self.topics['ground_truth'],
                self.topics['lv_dot'],
                self.topics['fusion'],
                metrics_topic,
            )
        )

    def _on_tracks(self, source, message):
        self.states[source].message = message
        self.states[source].arrival = time.monotonic()

    def _online(self, source, now_monotonic):
        state = self.states[source]
        return (
            state.message is not None
            and now_monotonic - state.arrival <= self.online_timeout
        )

    def _truth_target(self):
        message = self.states['ground_truth'].message
        if message is None:
            return None
        for tracked in message.objects:
            if tracked.track_id == self.target_id:
                return tracked
        return message.objects[0] if message.objects else None

    def _nearest(self, source, truth):
        message = self.states[source].message
        if message is None or truth is None:
            return None
        truth_stamp = _stamp_seconds(truth.last_update)
        if truth_stamp <= 0.0:
            truth_stamp = _stamp_seconds(
                self.states['ground_truth'].message.header.stamp
            )
        truth_position = _position(truth)
        candidates = []
        for tracked in message.objects:
            stamp = _stamp_seconds(tracked.last_update)
            if stamp <= 0.0:
                stamp = _stamp_seconds(message.header.stamp)
            if (
                truth_stamp > 0.0
                and stamp > 0.0
                and abs(stamp - truth_stamp) > self.max_timestamp_delta
            ):
                continue
            distance = _distance(truth_position, _position(tracked))
            if distance <= self.association_distance:
                candidates.append((abs(stamp - truth_stamp), distance, tracked))
        if not candidates:
            return None
        preferred_ids = []
        if source == 'fusion':
            preferred_ids.append(self.target_id)
        last_track_id = self.metrics[source].last_track_id
        if last_track_id:
            preferred_ids.append(last_track_id)
        for preferred_id in preferred_ids:
            preferred = [
                item for item in candidates
                if item[2].track_id == preferred_id
            ]
            if preferred:
                return min(preferred, key=lambda item: item[:2])[2]
        return min(candidates, key=lambda item: item[:2])[2]

    def _latency_ms(self, tracked, truth):
        tracked_stamp = _stamp_seconds(tracked.last_update)
        truth_stamp = _stamp_seconds(truth.last_update)
        if tracked_stamp <= 0.0:
            return None
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        transport_age = now_seconds - tracked_stamp
        if 0.0 <= transport_age <= 60.0:
            return transport_age * 1000.0
        if truth_stamp > 0.0:
            return abs(tracked_stamp - truth_stamp) * 1000.0
        return None

    def _source_row(self, name, tracked, truth, online):
        if tracked is None:
            return {
                'online': online,
                'track_id': '',
                'position': None,
                'velocity': None,
                'source': 'UNKNOWN',
                'source_mask': 0,
                'classification': 'UNKNOWN',
                'confidence': None,
                'position_error_m': None,
                'velocity_error_mps': None,
                'latency_ms': None,
            }
        return {
            'online': online,
            'track_id': tracked.track_id,
            'position': list(_position(tracked)),
            'velocity': list(_velocity(tracked)),
            'source': _source_name(tracked.source_mask),
            'source_mask': int(tracked.source_mask),
            'classification': _classification_name(
                tracked.classification
            ),
            'confidence': float(tracked.confidence),
            'position_error_m': (
                0.0 if name == 'ground_truth' else _distance(
                    _position(truth), _position(tracked)
                )
            ),
            'velocity_error_mps': (
                0.0 if name == 'ground_truth' else _distance(
                    _velocity(truth), _velocity(tracked)
                )
            ),
            'latency_ms': (
                0.0 if name == 'ground_truth'
                else self._latency_ms(tracked, truth)
            ),
        }

    def _evaluate(self):
        now_monotonic = time.monotonic()
        truth_online = self._online('ground_truth', now_monotonic)
        truth = self._truth_target() if truth_online else None
        matched = {
            'lv_dot': self._nearest('lv_dot', truth),
            'fusion': self._nearest('fusion', truth),
        } if truth is not None else {'lv_dot': None, 'fusion': None}

        truth_stamp = _stamp_seconds(truth.last_update) if truth else 0.0
        if truth is not None and truth_stamp <= 0.0:
            truth_stamp = _stamp_seconds(
                self.states['ground_truth'].message.header.stamp
            )
        is_new_sample = (
            truth is not None
            and truth_stamp > 0.0
            and truth_stamp != self.last_evaluated_stamp
        )
        if is_new_sample:
            self.last_evaluated_stamp = truth_stamp
            for name in ('lv_dot', 'fusion'):
                tracked = matched[name]
                self.metrics[name].add(
                    tracked,
                    position_error=(
                        _distance(_position(truth), _position(tracked))
                        if tracked is not None else None
                    ),
                    velocity_error=(
                        _distance(_velocity(truth), _velocity(tracked))
                        if tracked is not None else None
                    ),
                    latency_ms=(
                        self._latency_ms(tracked, truth)
                        if tracked is not None else None
                    ),
                    confidence=(
                        tracked.confidence if tracked is not None else None
                    ),
                )

        sources = {
            'ground_truth': self._source_row(
                'ground_truth', truth, truth, truth_online
            ) if truth is not None else self._source_row(
                'ground_truth', None, None, truth_online
            ),
            'lv_dot': self._source_row(
                'lv_dot', matched['lv_dot'], truth,
                self._online('lv_dot', now_monotonic),
            ),
            'fusion': self._source_row(
                'fusion', matched['fusion'], truth,
                self._online('fusion', now_monotonic),
            ),
        }
        payload = {
            'mode': 'shadow_fusion_validation',
            'target_id': self.target_id,
            'control_source': 'ground_truth',
            'control_output_published': False,
            'sources': sources,
            'summary': {
                name: self.metrics[name].summary()
                for name in ('lv_dot', 'fusion')
            },
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=True, sort_keys=True)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LvDotFusionEvaluator()
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
