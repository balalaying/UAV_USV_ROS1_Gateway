#!/usr/bin/env python3
"""Publish read-only fleet UAV and camera TF from Gazebo truth poses."""

import math
import threading

from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class UavCameraTf(Node):
    def __init__(self):
        super().__init__('uav_camera_tf')
        self.declare_parameter(
            'vehicle_ids', ['uav_01', 'uav_02', 'uav_03', 'uav_04']
        )
        self.declare_parameter(
            'pose_topic', '/world/fleet_dynamic_capture/pose/info'
        )
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('mount_x', 0.0)
        self.declare_parameter('mount_y', 0.0)
        self.declare_parameter('mount_z', 0.0)
        self.declare_parameter('mount_roll', 0.0)
        self.declare_parameter('mount_pitch', 1.5707)
        self.declare_parameter('mount_yaw', 0.0)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.vehicle_ids = [
            str(value) for value in self.get_parameter('vehicle_ids').value
        ]
        self.map_frame = str(self.get_parameter('map_frame_id').value)
        self.gz_node = GzNode()
        self.pose_lock = threading.Lock()
        self.poses = {}
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_mounts()
        pose_topic = str(self.get_parameter('pose_topic').value)
        if not self.gz_node.subscribe(Pose_V, pose_topic, self._on_pose):
            raise RuntimeError('Unable to subscribe to ' + pose_topic)
        rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self.create_timer(1.0 / rate, self._publish_dynamic)
        self.get_logger().info(
            'UAV camera TF: map -> */base_link -> */camera_link'
        )

    def _publish_mounts(self):
        xyz = (
            float(self.get_parameter('mount_x').value),
            float(self.get_parameter('mount_y').value),
            float(self.get_parameter('mount_z').value),
        )
        quaternion = quaternion_from_rpy(
            float(self.get_parameter('mount_roll').value),
            float(self.get_parameter('mount_pitch').value),
            float(self.get_parameter('mount_yaw').value),
        )
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for vehicle_id in self.vehicle_ids:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = vehicle_id + '/base_link'
            transform.child_frame_id = vehicle_id + '/camera_link'
            transform.transform.translation.x = xyz[0]
            transform.transform.translation.y = xyz[1]
            transform.transform.translation.z = xyz[2]
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            transforms.append(transform)
        self.static_broadcaster.sendTransform(transforms)

    def _on_pose(self, msg):
        updates = {}
        wanted = set(self.vehicle_ids)
        for pose in msg.pose:
            if pose.name not in wanted:
                continue
            updates[pose.name] = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
        if updates:
            with self.pose_lock:
                self.poses.update(updates)

    def _publish_dynamic(self):
        with self.pose_lock:
            poses = dict(self.poses)
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for vehicle_id in self.vehicle_ids:
            pose = poses.get(vehicle_id)
            if pose is None:
                continue
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = vehicle_id + '/base_link'
            transform.transform.translation.x = pose[0]
            transform.transform.translation.y = pose[1]
            transform.transform.translation.z = pose[2]
            transform.transform.rotation.x = pose[3]
            transform.transform.rotation.y = pose[4]
            transform.transform.rotation.z = pose[5]
            transform.transform.rotation.w = pose[6]
            transforms.append(transform)
        if transforms:
            self.dynamic_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = UavCameraTf()
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
