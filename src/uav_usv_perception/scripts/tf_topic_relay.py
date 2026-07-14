#!/usr/bin/env python3
"""Relay a namespaced dynamic TF stream to the global /tf topic."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfTopicRelay(Node):
    def __init__(self):
        super().__init__('tf_topic_relay')
        self.declare_parameter('input_topic', '/usv_01/tf')
        self.declare_parameter('output_topic', '/tf')
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        qos = QoSProfile(depth=100)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.publisher = self.create_publisher(TFMessage, output_topic, qos)
        self.create_subscription(
            TFMessage, input_topic, self.publisher.publish, qos
        )
        self.get_logger().info(
            'TF relay: %s -> %s' % (input_topic, output_topic)
        )


def main(args=None):
    rclpy.init(args=args)
    node = TfTopicRelay()
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
