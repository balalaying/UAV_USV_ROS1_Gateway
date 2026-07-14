#!/usr/bin/env python3
"""Move the standalone Mid-360 demo USV without touching fleet control."""

from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Mid360DemoMotion(Node):
    def __init__(self):
        super().__init__('mid360_demo_motion')
        self.declare_parameter('command_topic', '/model/usv_01/cmd_vel')
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('angular_speed', 0.035)
        self.declare_parameter('publish_rate', 10.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value)
        )
        command_topic = str(self.get_parameter('command_topic').value)

        self.gz_node = GzTransportNode()
        self.command_pub = self.gz_node.advertise(command_topic, Twist)
        self.create_timer(1.0 / publish_rate, self._publish_command)
        self.get_logger().info(
            'Mid-360 demo motion: %s linear=%.2f m/s angular=%.3f rad/s'
            % (command_topic, self.linear_speed, self.angular_speed)
        )

    def _publish_command(self):
        command = Twist()
        command.linear.x = self.linear_speed
        command.angular.z = self.angular_speed
        self.command_pub.publish(command)

    def destroy_node(self):
        command = Twist()
        self.command_pub.publish(command)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mid360DemoMotion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
