#!/usr/bin/env python3
"""Deterministic target motion profiles for LV-DOT tuning bags."""

import json
import math
import time

from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def motion_command(
    profile,
    elapsed,
    constant_speed,
    constant_leg_seconds,
    turn_speed,
    turn_radius,
    acceleration_initial,
    acceleration,
    acceleration_max,
):
    """Return body-frame speed and yaw rate for a tuning profile."""
    if profile == 'constant':
        leg = int(elapsed / max(1.0, constant_leg_seconds))
        direction = 1.0 if leg % 2 == 0 else -1.0
        return direction * constant_speed, 0.0
    if profile == 'turn':
        return turn_speed, turn_speed / max(1.0, turn_radius)
    speed = min(
        acceleration_max,
        acceleration_initial + acceleration * elapsed,
    )
    return speed, speed / max(1.0, turn_radius)


class LvDotTargetMotion(Node):
    def __init__(self):
        super().__init__('lv_dot_target_motion')
        self.declare_parameter(
            'command_topic', '/model/target_vessel/cmd_vel'
        )
        self.declare_parameter('profile', 'constant')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('constant_speed_mps', 0.45)
        self.declare_parameter('constant_leg_seconds', 12.0)
        self.declare_parameter('turn_speed_mps', 0.80)
        self.declare_parameter('turn_radius_m', 2.0)
        self.declare_parameter('acceleration_initial_mps', 0.15)
        self.declare_parameter('acceleration_mps2', 0.08)
        self.declare_parameter('acceleration_max_mps', 0.85)
        self.declare_parameter(
            'status_topic', '/lv_dot/tuning/target_motion'
        )

        self.profile = str(
            self.get_parameter('profile').value
        ).strip().lower()
        if self.profile not in ('constant', 'turn', 'acceleration'):
            raise ValueError(
                'profile must be constant, turn, or acceleration'
            )
        rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self.constant_speed = max(
            0.0, float(self.get_parameter('constant_speed_mps').value)
        )
        self.constant_leg_seconds = max(
            1.0, float(self.get_parameter('constant_leg_seconds').value)
        )
        self.turn_speed = max(
            0.0, float(self.get_parameter('turn_speed_mps').value)
        )
        self.turn_radius = max(
            1.0, float(self.get_parameter('turn_radius_m').value)
        )
        self.acceleration_initial = max(
            0.0,
            float(self.get_parameter('acceleration_initial_mps').value),
        )
        self.acceleration = max(
            0.0, float(self.get_parameter('acceleration_mps2').value)
        )
        self.acceleration_max = max(
            self.acceleration_initial,
            float(self.get_parameter('acceleration_max_mps').value),
        )

        self.gz_node = GzTransportNode()
        topic = str(self.get_parameter('command_topic').value)
        self.publisher = self.gz_node.advertise(topic, Twist)
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )
        self.started_at = time.monotonic()
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'LV-DOT target profile=%s topic=%s' % (self.profile, topic)
        )

    def _command(self, elapsed):
        return motion_command(
            self.profile,
            elapsed,
            self.constant_speed,
            self.constant_leg_seconds,
            self.turn_speed,
            self.turn_radius,
            self.acceleration_initial,
            self.acceleration,
            self.acceleration_max,
        )

    def _publish(self):
        elapsed = time.monotonic() - self.started_at
        speed, turn_rate = self._command(elapsed)
        command = Twist()
        command.linear.x = speed
        command.angular.z = turn_rate
        self.publisher.publish(command)

        status = {
            'profile': self.profile,
            'elapsed_s': round(elapsed, 3),
            'speed_mps': round(speed, 4),
            'turn_rate_rps': round(turn_rate, 4),
            'turn_radius_m': (
                round(abs(speed / turn_rate), 3)
                if not math.isclose(turn_rate, 0.0) else None
            ),
        }
        self.status_publisher.publish(String(
            data=json.dumps(status, sort_keys=True)
        ))


def main(args=None):
    rclpy.init(args=args)
    node = LvDotTargetMotion()
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
