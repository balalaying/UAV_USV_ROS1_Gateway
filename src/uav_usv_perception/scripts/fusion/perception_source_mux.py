#!/usr/bin/env python3
"""Select one perception source without changing the mission interface."""

from copy import deepcopy
import json
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from uav_usv_interfaces.msg import TrackedObjectArray


VALID_SOURCES = ('ground_truth', 'sensor', 'hybrid')


class PerceptionSourceMux(Node):
    def __init__(self):
        super().__init__('perception_source_mux')
        self.declare_parameter('perception_source', 'ground_truth')
        self.declare_parameter(
            'ground_truth_topic', '/perception/ground_truth/tracks'
        )
        self.declare_parameter(
            'sensor_topic', '/perception/fused/tracks'
        )
        self.declare_parameter(
            'hybrid_topic', '/perception/hybrid/tracks'
        )
        self.declare_parameter(
            'output_topic', '/fleet/perception/targets'
        )
        self.declare_parameter(
            'status_topic', '/perception/source_status'
        )
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('source_timeout_seconds', 1.0)
        self.declare_parameter('frame_id', 'map')

        self.source = self._validated_source(
            self.get_parameter('perception_source').value
        )
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.timeout = max(
            0.1, float(
                self.get_parameter('source_timeout_seconds').value
            )
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self.topics = {
            'ground_truth': str(
                self.get_parameter('ground_truth_topic').value
            ),
            'sensor': str(self.get_parameter('sensor_topic').value),
            'hybrid': str(self.get_parameter('hybrid_topic').value),
        }
        self.latest = {source: None for source in VALID_SOURCES}
        self.received_at = {source: 0.0 for source in VALID_SOURCES}

        self.publisher = self.create_publisher(
            TrackedObjectArray, self.output_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10
        )
        for source, topic in self.topics.items():
            self.create_subscription(
                TrackedObjectArray,
                topic,
                lambda message, source_name=source: self._on_source(
                    source_name, message
                ),
                10,
            )
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            'Perception source=%s -> %s' % (self.source, self.output_topic)
        )

    @staticmethod
    def _validated_source(value):
        source = str(value).strip().lower()
        if source not in VALID_SOURCES:
            raise ValueError(
                'perception_source must be one of: '
                + ', '.join(VALID_SOURCES)
            )
        return source

    def _on_parameters(self, parameters):
        requested = self.source
        for parameter in parameters:
            if parameter.name == 'perception_source':
                try:
                    requested = self._validated_source(parameter.value)
                except ValueError as error:
                    return SetParametersResult(
                        successful=False, reason=str(error)
                    )
        if requested != self.source:
            self.get_logger().info(
                'Perception source %s -> %s' % (self.source, requested)
            )
            self.source = requested
        return SetParametersResult(successful=True)

    def _on_source(self, source, message):
        self.latest[source] = message
        self.received_at[source] = time.monotonic()

    def _publish(self):
        now_monotonic = time.monotonic()
        message = self.latest[self.source]
        age = now_monotonic - self.received_at[self.source]
        online = message is not None and age <= self.timeout
        if online:
            output = deepcopy(message)
        else:
            output = TrackedObjectArray()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = self.frame_id
        self.publisher.publish(output)

        status = String()
        status.data = json.dumps({
            'source': self.source,
            'topic': self.topics[self.source],
            'online': online,
            'age_seconds': round(age, 3) if message is not None else None,
            'track_count': len(output.objects),
        }, separators=(',', ':'))
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSourceMux()
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
