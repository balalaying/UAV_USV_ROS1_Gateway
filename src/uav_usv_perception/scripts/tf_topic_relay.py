#!/usr/bin/env python3
"""Relay a namespaced dynamic TF stream to the global /tf topic."""

import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.executors import ExternalShutdownException
from uav_usv_ros1_compat.node import Node
from uav_usv_ros1_compat.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_usv_ros1_compat.time_fields import time_is_zero
from tf2_msgs.msg import TFMessage


class TfTopicRelay(Node):
    def __init__(self):
        super().__init__('tf_topic_relay')
        self.declare_parameter('input_topic', '/usv_01/tf')
        self.declare_parameter('output_topic', '/tf')
        self.declare_parameter('repair_zero_stamps', True)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        qos = QoSProfile(depth=100)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.publisher = self.create_publisher(TFMessage, output_topic, qos)
        self.repair_zero_stamps = bool(
            self.get_parameter('repair_zero_stamps').value
        )
        self.repaired_transforms = 0
        self.create_subscription(
            TFMessage, input_topic, self._relay, qos
        )
        self.get_logger().info(
            'TF relay: %s -> %s' % (input_topic, output_topic)
        )

    def _relay(self, message):
        if self.repair_zero_stamps:
            stamp = self.get_clock().now().to_msg()
            for transform in message.transforms:
                header_stamp = transform.header.stamp
                if time_is_zero(header_stamp):
                    transform.header.stamp = stamp
                    self.repaired_transforms += 1
        self.publisher.publish(message)


def main(args=None):
    ros1.init(args=args)
    node = TfTopicRelay()
    try:
        ros1.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ros1.ok():
            ros1.shutdown()


if __name__ == '__main__':
    main()
