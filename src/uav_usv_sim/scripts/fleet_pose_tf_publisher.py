#!/usr/bin/env python3
"""Publish Gazebo fleet poses as ROS odometry and authoritative map TF."""

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FleetPoseTfPublisher(Node):
    """Direct Gazebo truth -> ROS TF/odom adapter for fleet demo worlds.

    The global /tf tree intentionally uses direct map -> */base_link edges for
    every vehicle and target. Local Nav2 odom chains can still exist inside
    each USV namespace, but they are not the fleet-level authority. Keeping the
    fleet display tree flat avoids mixing map-frame perception with per-robot
    odom frames in Qt, RViz and future WebGL clients.
    """

    def __init__(self):
        super().__init__('fleet_pose_tf_publisher')
        self.declare_parameter('pose_topic', '/world/heterogeneous_332/pose/info')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('vehicle_ids', ['usv_01', 'usv_02', 'usv_03'])
        self.declare_parameter('target_ids', ['enemy_ship', 'friendly_ship'])
        self.declare_parameter('odom_topic_template', '/{vehicle_id}/odom')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('log_period_seconds', 5.0)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.map_frame_id = str(self.get_parameter('map_frame_id').value)
        self.vehicle_ids = [
            str(item) for item in self.get_parameter('vehicle_ids').value
        ]
        self.target_ids = [
            str(item) for item in self.get_parameter('target_ids').value
        ]
        self.odom_template = str(
            self.get_parameter('odom_topic_template').value
        )
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self.log_period = max(
            1.0, float(self.get_parameter('log_period_seconds').value)
        )

        self.gz_node = GzTransportNode()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publishers = {
            vehicle_id: self.create_publisher(
                Odometry,
                self.odom_template.format(vehicle_id=vehicle_id),
                20,
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

        if not self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v):
            raise RuntimeError('failed to subscribe Gazebo topic ' + self.pose_topic)

        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            'Fleet pose TF: %s -> map TF for vehicles=%s targets=%s'
            % (
                self.pose_topic,
                ','.join(self.vehicle_ids),
                ','.join(self.target_ids),
            )
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
        except Exception as error:  # noqa: BLE001 - guard Gazebo callback thread.
            now = time.monotonic()
            if now - self.last_error_log > 2.0:
                self.last_error_log = now
                self.get_logger().warning(
                    'Gazebo pose callback failed: %s' % error
                )

    def _pose_to_transform(self, stamp, parent, child, pose):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(pose.position.x)
        transform.transform.translation.y = float(pose.position.y)
        transform.transform.translation.z = float(pose.position.z)
        transform.transform.rotation.x = float(pose.orientation.x)
        transform.transform.rotation.y = float(pose.orientation.y)
        transform.transform.rotation.z = float(pose.orientation.z)
        transform.transform.rotation.w = float(pose.orientation.w)
        return transform

    def _odom_message(self, vehicle_id, pose, stamp, now):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = vehicle_id + '/odom'
        odom.child_frame_id = vehicle_id + '/base_link'
        odom.pose.pose.position.x = float(pose.position.x)
        odom.pose.pose.position.y = float(pose.position.y)
        odom.pose.pose.position.z = float(pose.position.z)
        odom.pose.pose.orientation.x = float(pose.orientation.x)
        odom.pose.pose.orientation.y = float(pose.orientation.y)
        odom.pose.pose.orientation.z = float(pose.orientation.z)
        odom.pose.pose.orientation.w = float(pose.orientation.w)

        previous = self.previous.get(vehicle_id)
        if previous is not None:
            previous_pose, previous_time, previous_yaw = previous
            dt = (now - previous_time).nanoseconds * 1e-9
            if dt > 1e-4:
                yaw = _yaw_from_quaternion(pose.orientation)
                dx = float(pose.position.x) - float(previous_pose.position.x)
                dy = float(pose.position.y) - float(previous_pose.position.y)
                odom.twist.twist.linear.x = (
                    math.cos(yaw) * dx + math.sin(yaw) * dy
                ) / dt
                yaw_delta = math.atan2(
                    math.sin(yaw - previous_yaw),
                    math.cos(yaw - previous_yaw),
                )
                odom.twist.twist.angular.z = yaw_delta / dt
        self.previous[vehicle_id] = (
            pose,
            now,
            _yaw_from_quaternion(pose.orientation),
        )
        return odom

    def _publish(self):
        with self.lock:
            poses = dict(self.poses)
            pose_received_at = dict(self.pose_received_at)
            callback_count = self.callback_count
        if not poses:
            return

        now = self.get_clock().now()
        stamp = now.to_msg()
        transforms = []
        for vehicle_id in self.vehicle_ids:
            pose = poses.get(vehicle_id)
            if pose is None:
                continue
            base_frame = vehicle_id + '/base_link'
            transforms.append(
                self._pose_to_transform(
                    stamp, self.map_frame_id, base_frame, pose
                )
            )
            self.odom_publishers[vehicle_id].publish(
                self._odom_message(vehicle_id, pose, stamp, now)
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
            self.tf_broadcaster.sendTransform(transforms)
            self.publish_count += 1

        monotonic_now = time.monotonic()
        if monotonic_now - self.last_log >= self.log_period:
            self.last_log = monotonic_now
            self.get_logger().info(
                'Fleet pose TF active: callbacks=%d publishes=%d seen=%s'
                % (
                    callback_count,
                    self.publish_count,
                    ','.join(sorted(pose_received_at)),
                )
            )

    def destroy_node(self):
        self.gz_node.unsubscribe(self.pose_topic)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FleetPoseTfPublisher()
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
