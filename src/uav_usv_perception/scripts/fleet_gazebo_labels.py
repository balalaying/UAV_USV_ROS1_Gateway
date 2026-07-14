#!/usr/bin/env python3
"""Publish presentation-only Gazebo labels for the capture fleet."""

import math
import threading

from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FleetGazeboLabels(Node):
    COLORS = {
        'uav_01': (0.10, 0.50, 1.00, 1.0),
        'uav_02': (0.95, 0.35, 0.12, 1.0),
        'uav_03': (0.15, 0.78, 0.32, 1.0),
        'uav_04': (0.78, 0.28, 0.90, 1.0),
        'usv_01': (1.00, 0.82, 0.12, 1.0),
        'usv_02': (1.00, 0.65, 0.08, 1.0),
        'enemy_target': (1.00, 0.12, 0.12, 1.0),
        'mid360': (0.12, 0.95, 0.95, 1.0),
    }

    def __init__(self):
        super().__init__('fleet_gazebo_labels')
        self.declare_parameter(
            'pose_topic', '/world/fleet_dynamic_capture/pose/info'
        )
        self.declare_parameter('mid360_vehicle_id', 'usv_01')
        self.declare_parameter('mid360_mount_x', 0.9075)
        self.declare_parameter('mid360_mount_z', 1.5625)
        self.declare_parameter('show_mid360', True)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.mid360_vehicle_id = str(
            self.get_parameter('mid360_vehicle_id').value
        )
        self.mid360_mount_x = float(
            self.get_parameter('mid360_mount_x').value
        )
        self.mid360_mount_z = float(
            self.get_parameter('mid360_mount_z').value
        )
        self.show_mid360 = bool(self.get_parameter('show_mid360').value)
        rate = max(1.0, float(
            self.get_parameter('publish_rate_hz').value
        ))

        self.gz_node = GzNode()
        self.marker_pub = self.gz_node.advertise('/marker', Marker)
        self.pose_lock = threading.Lock()
        self.poses = {}
        if not self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose):
            raise RuntimeError('failed to subscribe to ' + self.pose_topic)
        self.create_timer(1.0 / rate, self._publish)

    def _on_pose(self, msg):
        wanted = {
            'uav_01', 'uav_02', 'uav_03', 'uav_04',
            'usv_01', 'usv_02', 'enemy_target',
        }
        updates = {}
        for pose in msg.pose:
            if pose.name not in wanted:
                continue
            updates[pose.name] = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
        if updates:
            with self.pose_lock:
                self.poses.update(updates)

    @staticmethod
    def _label(entity):
        if entity == 'enemy_target':
            return 'TARGET'
        return entity.replace('_', '-').upper()

    def _text_marker(self, marker_id, namespace, text, xyz, color, scale):
        marker = Marker()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD_MODIFY
        marker.type = Marker.TEXT
        marker.visibility = Marker.ALL
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        for material in (marker.material.ambient, marker.material.diffuse):
            material.r, material.g, material.b, material.a = color
        marker.text = text
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

    def _publish(self):
        with self.pose_lock:
            poses = dict(self.poses)
        entities = (
            'uav_01', 'uav_02', 'uav_03', 'uav_04',
            'usv_01', 'usv_02', 'enemy_target',
        )
        for index, entity in enumerate(entities):
            pose = poses.get(entity)
            if pose is None:
                continue
            z_offset = 4.2 if entity.startswith('uav_') else 3.2
            self._text_marker(
                index + 1,
                'fleet_entity_labels',
                self._label(entity),
                (pose[0], pose[1], pose[2] + z_offset),
                self.COLORS[entity],
                1.5,
            )

        carrier = (
            poses.get(self.mid360_vehicle_id) if self.show_mid360 else None
        )
        if carrier is None:
            return
        yaw = 2.0 * math.atan2(carrier[3], carrier[4])
        sensor_x = carrier[0] + math.cos(yaw) * self.mid360_mount_x
        sensor_y = carrier[1] + math.sin(yaw) * self.mid360_mount_x
        self._text_marker(
            100,
            'fleet_sensor_labels',
            'MID-360',
            (
                sensor_x,
                sensor_y,
                carrier[2] + self.mid360_mount_z + 1.3,
            ),
            self.COLORS['mid360'],
            1.15,
        )


def main(args=None):
    rclpy.init(args=args)
    node = FleetGazeboLabels()
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
