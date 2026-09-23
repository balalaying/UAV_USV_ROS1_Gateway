#!/usr/bin/env python3

import math
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rospy


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ColregsScenarioController:
    """Drives standard encounter vessels through the Gazebo Twist API."""

    def __init__(self):
        self.pose_topic = rospy.get_param(
            '~pose_topic',
            '/world/default/pose/info',
        )
        self.ownship_name = rospy.get_param(
            '~ownship_name',
            'landing_boat',
        )
        self.target_name = rospy.get_param(
            '~target_name',
            'target_vessel',
        )
        self.ownship_cmd_topic = rospy.get_param(
            '~ownship_cmd_topic',
            '/model/simple_boat/cmd_vel',
        )
        self.target_cmd_topic = rospy.get_param(
            '~target_cmd_topic',
            '/model/target_vessel/cmd_vel',
        )

        self.auto_ownship = bool(
            rospy.get_param('~auto_ownship', True)
        )
        self.ownship_speed = float(
            rospy.get_param('~ownship_speed', 1.0)
        )
        self.ownship_heading = float(
            rospy.get_param('~ownship_heading', 0.0)
        )
        self.target_speed = float(
            rospy.get_param('~target_speed', 1.0)
        )
        self.target_heading = float(
            rospy.get_param('~target_heading', math.pi)
        )
        self.heading_kp = float(
            rospy.get_param('~heading_kp', 1.8)
        )
        self.max_turn_rate = float(
            rospy.get_param('~max_turn_rate', 0.8)
        )
        self.pose_timeout = float(
            rospy.get_param('~pose_timeout', 1.0)
        )
        self.scenario_name = rospy.get_param(
            '~scenario_name',
            'head_on',
        )

        control_rate = max(
            1.0,
            float(rospy.get_param('~control_rate', 10.0)),
        )

        self.gz_node = GzTransportNode()

        self.ownship_pub = self.gz_node.advertise(
            self.ownship_cmd_topic,
            Twist,
        )
        self.target_pub = self.gz_node.advertise(
            self.target_cmd_topic,
            Twist,
        )

        self.gz_node.subscribe(
            Pose_V,
            self.pose_topic,
            self._on_pose,
        )

        self.poses = {}
        self.last_pose_time = 0.0

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / control_rate),
            self._on_timer,
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            'COLREGs scenario "%s": ownship auto=%s speed=%.2f '
            'heading=%.1f deg, target speed=%.2f heading=%.1f deg.',
            self.scenario_name,
            self.auto_ownship,
            self.ownship_speed,
            math.degrees(self.ownship_heading),
            self.target_speed,
            math.degrees(self.target_heading),
        )

    def shutdown(self):
        if self.auto_ownship:
            self._publish(self.ownship_pub, 0.0, 0.0)

        self._publish(self.target_pub, 0.0, 0.0)

    def _on_pose(self, msg):
        for pose in msg.pose:
            if pose.name in (self.ownship_name, self.target_name):
                self.poses[pose.name] = pose

        self.last_pose_time = time.monotonic()

    def _heading_command(self, name, speed, heading):
        pose = self.poses.get(name)

        if pose is None:
            return 0.0, 0.0

        yaw_error = wrap_pi(
            heading - yaw_from_quaternion(pose.orientation)
        )

        turn_rate = clamp(
            self.heading_kp * yaw_error,
            -self.max_turn_rate,
            self.max_turn_rate,
        )

        speed_scale = clamp(
            1.0 - abs(yaw_error) / math.pi,
            0.3,
            1.0,
        )

        return speed * speed_scale, turn_rate

    def _on_timer(self, _event):
        if time.monotonic() - self.last_pose_time > self.pose_timeout:
            return

        target_speed, target_turn = self._heading_command(
            self.target_name,
            self.target_speed,
            self.target_heading,
        )

        self._publish(
            self.target_pub,
            target_speed,
            target_turn,
        )

        if self.auto_ownship:
            own_speed, own_turn = self._heading_command(
                self.ownship_name,
                self.ownship_speed,
                self.ownship_heading,
            )

            self._publish(
                self.ownship_pub,
                own_speed,
                own_turn,
            )

    @staticmethod
    def _publish(publisher, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        publisher.publish(msg)


def main():
    rospy.init_node('colregs_scenario_controller')

    ColregsScenarioController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
