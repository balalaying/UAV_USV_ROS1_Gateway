#!/usr/bin/env python3

import sys
import termios
import tty

from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rospy


HELP_TEXT = """
Keyboard boat control

  W/S : increase/decrease forward speed
  A/D : turn left/right
  X   : reverse direction
  Space : stop
  Q or Ctrl-C : quit
"""


class KeyboardBoatControl:

    def __init__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_boat_control must be run in a terminal')

        self.topic = rospy.get_param('~topic', '/model/simple_boat/cmd_vel')
        self.speed_step = rospy.get_param('~speed_step', 0.2)
        self.turn_step = rospy.get_param('~turn_step', 0.15)
        self.max_speed = rospy.get_param('~max_speed', 2.0)
        self.max_turn = rospy.get_param('~max_turn', 1.2)

        self.gz_node = GzTransportNode()
        self.publisher = self.gz_node.advertise(self.topic, Twist)

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        rospy.loginfo(HELP_TEXT)
        rospy.loginfo('Publishing Gazebo Twist commands on %s', self.topic)

    def shutdown(self):
        self.publish_cmd(0.0, 0.0)

        if hasattr(self, 'old_terminal_settings'):
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.old_terminal_settings,
            )

    def run(self):
        self.publish_cmd(self.linear_x, self.angular_z)

        while not rospy.is_shutdown():
            self.handle_key(sys.stdin.read(1).lower())

    def handle_key(self, key):
        old_linear_x = self.linear_x
        old_angular_z = self.angular_z

        if key == 'w':
            self.linear_x += self.speed_step
        elif key == 's':
            self.linear_x -= self.speed_step
        elif key == 'a':
            self.angular_z += self.turn_step
        elif key == 'd':
            self.angular_z -= self.turn_step
        elif key == 'x':
            self.linear_x = -self.linear_x
        elif key == ' ':
            self.linear_x = 0.0
            self.angular_z = 0.0
        elif key == 'q' or key == '\x03':
            raise KeyboardInterrupt

        self.linear_x = self.clamp(
            self.linear_x,
            -self.max_speed,
            self.max_speed,
        )
        self.angular_z = self.clamp(
            self.angular_z,
            -self.max_turn,
            self.max_turn,
        )

        changed = (
            self.linear_x != old_linear_x
            or self.angular_z != old_angular_z
        )

        if changed:
            self.publish_cmd(self.linear_x, self.angular_z)
            rospy.loginfo(
                'linear_x=%.2f, angular_z=%.2f',
                self.linear_x,
                self.angular_z,
            )

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        if not self.publisher.publish(msg):
            rospy.logwarn('Failed to publish boat command')


def main():
    rospy.init_node('keyboard_boat_control')

    node = KeyboardBoatControl()
    rospy.on_shutdown(node.shutdown)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
