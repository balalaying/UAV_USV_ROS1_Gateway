#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from nav_msgs.msg import Odometry
import rospy
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MaritimeTfPublisher:
    """Publishes world truth as a consistent map/odom/base TF for test scenes."""

    def __init__(self):
        self.pose_topic = rospy.get_param(
            '~pose_topic',
            '/world/default/pose/info',
        )
        self.map_frame_id = rospy.get_param(
            '~map_frame_id',
            'map',
        )
        self.odom_frame_id = rospy.get_param(
            '~odom_frame_id',
            'odom',
        )
        self.ownship_name = rospy.get_param(
            '~ownship_name',
            'landing_boat',
        )
        self.ownship_frame_id = rospy.get_param(
            '~ownship_frame_id',
            'landing_boat/base_link',
        )
        self.target_name = rospy.get_param(
            '~target_name',
            'target_vessel',
        )
        self.target_frame_id = rospy.get_param(
            '~target_frame_id',
            'target_vessel/base_link',
        )
        self.odom_topic = rospy.get_param(
            '~odom_topic',
            '/maritime/ownship/odom',
        )

        self.gz_node = GzTransportNode()
        self.gz_node.subscribe(
            Pose_V,
            self.pose_topic,
            self._on_pose,
        )

        self.tf_broadcaster = TransformBroadcaster()

        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=20,
        )

        self.previous_pose = None
        self.previous_time = None

        rospy.loginfo(
            'Publishing test-scene TF: map -> odom -> %s and map -> %s.',
            self.ownship_frame_id,
            self.target_frame_id,
        )

    def _on_pose(self, msg):
        ownship = None
        target = None

        for pose in msg.pose:
            if pose.name == self.ownship_name:
                ownship = pose
            elif pose.name == self.target_name:
                target = pose

        if ownship is None:
            return

        now = rospy.Time.now()
        stamp = now

        transforms = [
            self._identity_map_to_odom(stamp)
        ]

        transforms.append(
            self._pose_transform(
                stamp,
                self.odom_frame_id,
                self.ownship_frame_id,
                ownship,
            )
        )

        if target is not None:
            transforms.append(
                self._pose_transform(
                    stamp,
                    self.map_frame_id,
                    self.target_frame_id,
                    target,
                )
            )

        self.tf_broadcaster.sendTransform(transforms)
        self._publish_odom(ownship, now, stamp)

    def _identity_map_to_odom(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame_id
        transform.child_frame_id = self.odom_frame_id
        transform.transform.rotation.w = 1.0
        return transform

    @staticmethod
    def _pose_transform(stamp, parent, child, pose):
        transform = TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z

        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z
        transform.transform.rotation.w = pose.orientation.w

        return transform

    def _publish_odom(self, pose, now, stamp):
        vx = 0.0
        wz = 0.0

        if self.previous_pose is not None and self.previous_time is not None:
            dt = (now - self.previous_time).to_sec()

            if dt > 1e-4:
                dx = pose.position.x - self.previous_pose.position.x
                dy = pose.position.y - self.previous_pose.position.y

                yaw = yaw_from_quaternion(pose.orientation)
                previous_yaw = yaw_from_quaternion(
                    self.previous_pose.orientation
                )

                vx = (
                    math.cos(yaw) * dx
                    + math.sin(yaw) * dy
                ) / dt

                yaw_delta = math.atan2(
                    math.sin(yaw - previous_yaw),
                    math.cos(yaw - previous_yaw),
                )

                wz = yaw_delta / dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.ownship_frame_id

        odom.pose.pose.position.x = pose.position.x
        odom.pose.pose.position.y = pose.position.y
        odom.pose.pose.position.z = pose.position.z

        odom.pose.pose.orientation.x = pose.orientation.x
        odom.pose.pose.orientation.y = pose.orientation.y
        odom.pose.pose.orientation.z = pose.orientation.z
        odom.pose.pose.orientation.w = pose.orientation.w

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        self.previous_pose = pose
        self.previous_time = now


def main():
    rospy.init_node('maritime_tf_publisher')

    MaritimeTfPublisher()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
