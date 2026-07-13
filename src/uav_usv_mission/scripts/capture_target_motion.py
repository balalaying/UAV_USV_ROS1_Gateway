#!/usr/bin/env python3
"""Deterministic Gazebo target motion, including a sudden-turn test case."""

import time

from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class CaptureTargetMotion(Node):
    def __init__(self):
        super().__init__('capture_target_motion')
        self.declare_parameter(
            'command_topic', '/model/enemy_target/cmd_vel'
        )
        self.declare_parameter('speed', 1.2)
        self.declare_parameter('nominal_turn_rate', 0.01)
        self.declare_parameter('enable_sudden_turn', True)
        self.declare_parameter('sudden_turn_time', 55.0)
        self.declare_parameter('sudden_turn_duration', 8.0)
        self.declare_parameter('sudden_turn_rate', 0.24)
        self.declare_parameter('publish_rate', 10.0)

        self.speed = float(self.get_parameter('speed').value)
        self.nominal_turn_rate = float(
            self.get_parameter('nominal_turn_rate').value
        )
        self.enable_sudden_turn = bool(
            self.get_parameter('enable_sudden_turn').value
        )
        self.sudden_turn_time = float(
            self.get_parameter('sudden_turn_time').value
        )
        self.sudden_turn_duration = float(
            self.get_parameter('sudden_turn_duration').value
        )
        self.sudden_turn_rate = float(
            self.get_parameter('sudden_turn_rate').value
        )
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value)
        )

        self.gz_node = GzTransportNode()
        self.publisher = self.gz_node.advertise(
            str(self.get_parameter('command_topic').value), Twist
        )
        self.status_pub = self.create_publisher(
            String, '/capture/target_motion_status', 10
        )
        self.started_at = time.monotonic()
        self.last_mode = ''
        self.create_timer(1.0 / publish_rate, self._publish)

    def _publish(self):
        elapsed = time.monotonic() - self.started_at
        in_sudden_turn = (
            self.enable_sudden_turn
            and self.sudden_turn_time <= elapsed
            < self.sudden_turn_time + self.sudden_turn_duration
        )
        turn_rate = (
            self.sudden_turn_rate
            if in_sudden_turn else self.nominal_turn_rate
        )
        command = Twist()
        command.linear.x = self.speed
        command.angular.z = turn_rate
        self.publisher.publish(command)

        mode = 'SUDDEN_TURN' if in_sudden_turn else 'NOMINAL'
        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().warning(
                'Target motion mode %s: speed=%.2f turn_rate=%.3f'
                % (mode, self.speed, turn_rate)
            )
        self.status_pub.publish(String(
            data='mode=%s elapsed=%.1f speed=%.2f turn_rate=%.3f'
            % (mode, elapsed, self.speed, turn_rate)
        ))


def main(args=None):
    rclpy.init(args=args)
    node = CaptureTargetMotion()
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
