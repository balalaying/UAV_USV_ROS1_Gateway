#!/usr/bin/env python3
"""Publish Gazebo fleet poses as ROS odometry and authoritative map TF."""

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from nav_msgs.msg import Odometry
import rospy
from tf2_ros import TransformBroadcaster


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FleetPoseTfPublisher:
    """Direct Gazebo truth -> ROS TF/odom adapter for fleet demo worlds."""

    def __init__(self):
        self.pose_topic = rospy.get_param(
            '~pose_topic',
            '/world/heterogeneous_332/pose/info',
        )
        self.map_frame_id = rospy.get_param(
            '~map_frame_id',
            'map',
        )
        self.vehicle_ids = [
            str(item)
            for item in rospy.get_param(
                '~vehicle_ids',
                ['usv_01', 'usv_02', 'usv_03'],
            )
        ]
        self.target_ids = [
            str(item)
            for item in rospy.get_param(
                '~target_ids',
                ['enemy_ship', 'friendly_ship'],
            )
        ]
        self.odom_template = rospy.get_param(
            '~odom_topic_template',
            '/{vehicle_id}/odom',
        )

        publish_rate = max(
            1.0,
            float(rospy.get_param('~publish_rate_hz', 30.0)),
        )
        self.log_period = max(
            1.0,
            float(rospy.get_param('~log_period_seconds', 5.0)),
        )

        self.gz_node = GzTransportNode()
        self.tf_broadcaster = TransformBroadcaster()

        self.odom_publishers = {
            vehicle_id: rospy.Publisher(
                self.odom_template.format(vehicle_id=vehicle_id),
                Odometry,
                queue_size=20,
            )
            for vehicle_id in self.vehicle_ids
        }

        self.lock = threading.Lock()
        self.poses = {}
        self.pose_received_at = {}
        self.previous = {}

        self.callback_count = 0
        self.publish_count = 0
        self.last_log = time.monotonic()
        self.last_error_log = 0.0

        if not self.gz_node.subscribe(
            Pose_V,
            self.pose_topic,
            self._on_pose_v,
        ):
            raise RuntimeError(
                'failed to subscribe Gazebo topic ' + self.pose_topic
            )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate),
            self._publish,
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            'Fleet pose TF: %s -> map TF for vehicles=%s targets=%s',
            self.pose_topic,
            ','.join(self.vehicle_ids),
            ','.join(self.target_ids),
        )

    def _on_pose_v(self, msg):
        try:
            tracked = set(self.vehicle_ids) | set(self.target_ids)

            poses = {
                pose.name: pose
                for pose in msg.pose
                if pose.name in tracked
            }

            if not poses:
                return

            with self.lock:
                received_at = time.monotonic()
                self.poses.update(poses)

                for name in poses:
                    self.pose_received_at[name] = received_at

                self.callback_count += 1

        except Exception as error:
            now = time.monotonic()

            if now - self.last_error_log > 2.0:
                self.last_error_log = now
                rospy.logwarn(
                    'Gazebo pose callback failed: %s',
                    error,
                )

    @staticmethod
    def _pose_to_transform(stamp, parent, child, pose):
        transform = TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = float(
            pose.position.x
        )
        transform.transform.translation.y = float(
            pose.position.y
        )
        transform.transform.translation.z = float(
            pose.position.z
        )

        transform.transform.rotation.x = float(
            pose.orientation.x
        )
        transform.transform.rotation.y = float(
            pose.orientation.y
        )
        transform.transform.rotation.z = float(
            pose.orientation.z
        )
        transform.transform.rotation.w = float(
            pose.orientation.w
        )

        return transform

    def _odom_message(self, vehicle_id, pose, stamp, now):
        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = vehicle_id + '/odom'
        odom.child_frame_id = vehicle_id + '/base_link'

        odom.pose.pose.position.x = float(pose.position.x)
        odom.pose.pose.position.y = float(pose.position.y)
        odom.pose.pose.position.z = float(pose.position.z)

        odom.pose.pose.orientation.x = float(
            pose.orientation.x
        )
        odom.pose.pose.orientation.y = float(
            pose.orientation.y
        )
        odom.pose.pose.orientation.z = float(
            pose.orientation.z
        )
        odom.pose.pose.orientation.w = float(
            pose.orientation.w
        )

        previous = self.previous.get(vehicle_id)

        if previous is not None:
            previous_pose, previous_time, previous_yaw = previous

            dt = (now - previous_time).to_sec()

            if dt > 1e-4:
                yaw = _yaw_from_quaternion(
                    pose.orientation
                )

                dx = (
                    float(pose.position.x)
                    - float(previous_pose.position.x)
                )
                dy = (
                    float(pose.position.y)
                    - float(previous_pose.position.y)
                )

                odom.twist.twist.linear.x = (
                    math.cos(yaw) * dx
                    + math.sin(yaw) * dy
                ) / dt

                yaw_delta = math.atan2(
                    math.sin(yaw - previous_yaw),
                    math.cos(yaw - previous_yaw),
                )

                odom.twist.twist.angular.z = (
                    yaw_delta / dt
                )

        self.previous[vehicle_id] = (
            pose,
            now,
            _yaw_from_quaternion(pose.orientation),
        )

        return odom

    def _publish(self, _event):
        with self.lock:
            poses = dict(self.poses)
            pose_received_at = dict(
                self.pose_received_at
            )
            callback_count = self.callback_count

        if not poses:
            return

        now = rospy.Time.now()
        stamp = now

        transforms = []

        for vehicle_id in self.vehicle_ids:
            pose = poses.get(vehicle_id)

            if pose is None:
                continue

            base_frame = vehicle_id + '/base_link'

            transforms.append(
                self._pose_to_transform(
                    stamp,
                    self.map_frame_id,
                    base_frame,
                    pose,
                )
            )

            self.odom_publishers[vehicle_id].publish(
                self._odom_message(
                    vehicle_id,
                    pose,
                    stamp,
                    now,
                )
            )

        for target_id in self.target_ids:
            pose = poses.get(target_id)

            if pose is None:
                continue

            transforms.append(
                self._pose_to_transform(
                    stamp,
                    self.map_frame_id,
                    target_id + '/base_link',
                    pose,
                )
            )

        if transforms:
            self.tf_broadcaster.sendTransform(
                transforms
            )
            self.publish_count += 1

        monotonic_now = time.monotonic()

        if monotonic_now - self.last_log >= self.log_period:
            self.last_log = monotonic_now

            rospy.loginfo(
                'Fleet pose TF active: callbacks=%d publishes=%d seen=%s',
                callback_count,
                self.publish_count,
                ','.join(sorted(pose_received_at)),
            )

    def shutdown(self):
        try:
            self.gz_node.unsubscribe(
                self.pose_topic
            )
        except Exception:
            pass


def main():
    rospy.init_node('fleet_pose_tf_publisher')

    FleetPoseTfPublisher()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
