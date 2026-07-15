#!/usr/bin/env python3
"""Compare LV-DOT observations with Gazebo ground truth in Shadow Mode."""

from collections import deque
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from uav_usv_interfaces.msg import TrackedObjectArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _position(tracked):
    point = tracked.pose.pose.position
    return (float(point.x), float(point.y), float(point.z))


def _velocity(tracked):
    vector = tracked.twist.twist.linear
    return (float(vector.x), float(vector.y), float(vector.z))


class LvDotShadowEvaluator(Node):
    def __init__(self):
        super().__init__('lv_dot_shadow_evaluator')
        self.declare_parameter(
            'ground_truth_topic', '/perception/ground_truth/tracks'
        )
        self.declare_parameter(
            'observation_topic', '/perception/lv_dot/observations'
        )
        self.declare_parameter(
            'metrics_topic', '/perception/lv_dot/shadow_metrics'
        )
        self.declare_parameter('target_id', 'target_vessel')
        self.declare_parameter('association_distance_m', 20.0)
        self.declare_parameter('online_timeout_seconds', 1.5)
        self.declare_parameter('window_size', 100)
        self.declare_parameter('evaluation_rate_hz', 10.0)
        self.declare_parameter(
            'lidar_bbox_topic', '/lv_dot/diagnostics/lidar_bboxes'
        )
        self.declare_parameter(
            'filtered_bbox_topic', '/lv_dot/diagnostics/filtered_bboxes'
        )
        self.declare_parameter(
            'tracked_bbox_topic', '/lv_dot/diagnostics/tracked_bboxes'
        )

        ground_truth_topic = str(
            self.get_parameter('ground_truth_topic').value
        )
        observation_topic = str(
            self.get_parameter('observation_topic').value
        )
        metrics_topic = str(self.get_parameter('metrics_topic').value)
        self.target_id = str(self.get_parameter('target_id').value)
        self.association_distance = max(
            0.1,
            float(self.get_parameter('association_distance_m').value),
        )
        self.online_timeout = max(
            0.1,
            float(self.get_parameter('online_timeout_seconds').value),
        )
        window_size = max(5, int(self.get_parameter('window_size').value))
        rate = max(
            1.0, float(self.get_parameter('evaluation_rate_hz').value)
        )

        self.publisher = self.create_publisher(String, metrics_topic, 10)
        self.create_subscription(
            TrackedObjectArray,
            ground_truth_topic,
            self._on_ground_truth,
            10,
        )
        self.create_subscription(
            TrackedObjectArray,
            observation_topic,
            self._on_observations,
            10,
        )
        diagnostic_topics = {
            'lidar_bbox_count': str(
                self.get_parameter('lidar_bbox_topic').value
            ),
            'filtered_bbox_count': str(
                self.get_parameter('filtered_bbox_topic').value
            ),
            'tracked_bbox_count': str(
                self.get_parameter('tracked_bbox_topic').value
            ),
        }
        self.pipeline_counts = {key: 0 for key in diagnostic_topics}
        self.pipeline_arrivals = {key: 0.0 for key in diagnostic_topics}
        self.diagnostic_subscriptions = [
            self.create_subscription(
                MarkerArray,
                topic,
                self._diagnostic_callback(key),
                10,
            )
            for key, topic in diagnostic_topics.items()
        ]
        self.ground_truth = None
        self.observations = None
        self.ground_truth_arrival = 0.0
        self.observation_arrival = 0.0
        self.detections = deque(maxlen=window_size)
        self.position_errors = deque(maxlen=window_size)
        self.velocity_errors = deque(maxlen=window_size)
        self.latencies = deque(maxlen=window_size)
        self.detection_arrivals = deque(maxlen=max(50, window_size * 2))
        self.last_track_id = ''
        self.id_switches = 0
        self.matched_samples = 0
        self.create_timer(1.0 / rate, self._evaluate)
        self.get_logger().info(
            'LV-DOT Shadow evaluator %s vs %s -> %s'
            % (observation_topic, ground_truth_topic, metrics_topic)
        )

    def _on_ground_truth(self, message):
        self.ground_truth = message
        self.ground_truth_arrival = time.monotonic()

    def _on_observations(self, message):
        self.observations = message
        self.observation_arrival = time.monotonic()
        if message.objects:
            self.detection_arrivals.append(self.observation_arrival)

    def _diagnostic_callback(self, key):
        def callback(message):
            self.pipeline_counts[key] = sum(
                marker.action in (Marker.ADD, Marker.MODIFY)
                for marker in message.markers
            )
            self.pipeline_arrivals[key] = time.monotonic()
        return callback

    def _ground_truth_target(self):
        if self.ground_truth is None:
            return None
        for tracked in self.ground_truth.objects:
            if tracked.track_id == self.target_id:
                return tracked
        return self.ground_truth.objects[0] if self.ground_truth.objects else None

    def _nearest_observation(self, truth):
        if self.observations is None:
            return None
        truth_position = _position(truth)
        if self.last_track_id:
            for tracked in self.observations.objects:
                if (
                    tracked.track_id == self.last_track_id
                    and _distance(truth_position, _position(tracked))
                    <= self.association_distance
                ):
                    return tracked
        selected = None
        selected_distance = self.association_distance
        for tracked in self.observations.objects:
            distance = _distance(truth_position, _position(tracked))
            if distance <= selected_distance:
                selected = tracked
                selected_distance = distance
        return selected

    @staticmethod
    def _mean(values):
        return sum(values) / len(values) if values else None

    def _evaluate(self):
        now_monotonic = time.monotonic()
        while (
            self.detection_arrivals
            and now_monotonic - self.detection_arrivals[0] > 5.0
        ):
            self.detection_arrivals.popleft()
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        truth_online = (
            self.ground_truth is not None
            and now_monotonic - self.ground_truth_arrival
            <= self.online_timeout
        )
        lv_dot_online = (
            self.observations is not None
            and now_monotonic - self.observation_arrival
            <= self.online_timeout
        )
        truth = self._ground_truth_target() if truth_online else None
        matched = self._nearest_observation(truth) if truth is not None else None

        if truth is not None:
            detected = matched is not None and lv_dot_online
            self.detections.append(1.0 if detected else 0.0)
            if detected:
                self.position_errors.append(
                    _distance(_position(truth), _position(matched))
                )
                self.velocity_errors.append(
                    _distance(_velocity(truth), _velocity(matched))
                )
                stamp = _stamp_seconds(matched.last_update)
                if stamp <= 0.0 and self.observations is not None:
                    stamp = _stamp_seconds(self.observations.header.stamp)
                if stamp > 0.0:
                    self.latencies.append(max(0.0, now_seconds - stamp))
                if self.last_track_id and matched.track_id != self.last_track_id:
                    self.id_switches += 1
                self.last_track_id = matched.track_id
                self.matched_samples += 1

        stability = 1.0
        if self.matched_samples > 1:
            stability = max(
                0.0,
                1.0 - self.id_switches / float(self.matched_samples - 1),
            )
        metrics = {
            'mode': 'shadow',
            'ground_truth_online': truth_online,
            'lv_dot_online': lv_dot_online,
            'target_id': self.target_id,
            'matched_track_id': matched.track_id if matched else '',
            'position_error_m': self._mean(self.position_errors),
            'velocity_error_mps': self._mean(self.velocity_errors),
            'detection_rate': self._mean(self.detections),
            'track_stability': stability,
            'id_switches': self.id_switches,
            'latency_ms': (
                self._mean(self.latencies) * 1000.0
                if self.latencies else None
            ),
            'window_samples': len(self.detections),
            'observation_count': (
                len(self.observations.objects)
                if self.observations is not None and lv_dot_online else 0
            ),
            'detection_frequency_hz': (
                (len(self.detection_arrivals) - 1)
                / max(
                    1e-6,
                    self.detection_arrivals[-1]
                    - self.detection_arrivals[0],
                )
                if len(self.detection_arrivals) >= 2 else 0.0
            ),
        }
        for key, count in self.pipeline_counts.items():
            online = (
                now_monotonic - self.pipeline_arrivals[key]
                <= self.online_timeout
            ) if self.pipeline_arrivals[key] > 0.0 else False
            metrics[key] = count if online else 0
            metrics[key.replace('_count', '_online')] = online
        message = String()
        message.data = json.dumps(metrics, ensure_ascii=True, sort_keys=True)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LvDotShadowEvaluator()
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
