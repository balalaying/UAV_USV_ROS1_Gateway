#!/usr/bin/env python3
"""Expose one fleet vehicle pose as the standard input expected by LV-DOT."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from uav_usv_interfaces.msg import VehicleState


class LvDotPoseAdapter(Node):
    def __init__(self):
        super().__init__('lv_dot_pose_adapter')
        self.declare_parameter('vehicle_id', 'usv_01')
        self.declare_parameter('input_topic', '/fleet/state')
        self.declare_parameter(
            'output_topic', '/perception/lv_dot/usv_01/pose'
        )
        self.declare_parameter('frame_id', 'map')
        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publisher = self.create_publisher(
            PoseStamped, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleState,
            input_topic,
            self._on_state,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'LV-DOT pose adapter %s[%s] -> %s'
            % (input_topic, self.vehicle_id, output_topic)
        )

    def _on_state(self, message):
        if message.vehicle_id != self.vehicle_id or not message.online:
            return
        output = PoseStamped()
        output.header = message.header
        output.header.frame_id = message.header.frame_id or self.frame_id
        output.pose = message.pose
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LvDotPoseAdapter()
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
