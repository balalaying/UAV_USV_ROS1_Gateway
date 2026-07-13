#!/usr/bin/env python3
"""Render capture topics as RViz markers without making task decisions."""

import json
import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class CaptureVisualizer(Node):
    COLORS = (
        (0.10, 0.90, 0.35),
        (0.10, 0.70, 1.00),
        (0.10, 0.45, 1.00),
        (1.00, 0.72, 0.10),
    )

    def __init__(self):
        super().__init__('capture_visualizer')
        self.target = None
        self.states = {}
        self.prediction = None
        self.assignment_points = None
        self.assignment_metadata = {}
        self.capture_center = (0.0, 0.0)
        self.capture_radius = 18.0
        self.capture_state = 'SEARCH'
        self.status = 'starting'
        self.target_status = {}
        self.publisher = self.create_publisher(
            MarkerArray, '/capture/markers', 10
        )
        self.create_subscription(
            TrackedObjectArray,
            '/fleet/perception/targets',
            self._on_targets,
            10,
        )
        self.create_subscription(
            VehicleState,
            '/fleet/state',
            self._on_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Path,
            '/capture/target_prediction',
            lambda msg: setattr(self, 'prediction', msg),
            10,
        )
        self.create_subscription(
            PoseArray,
            '/capture/assignment_points',
            lambda msg: setattr(self, 'assignment_points', msg),
            10,
        )
        self.create_subscription(
            String, '/capture/roles', self._on_roles, 10
        )
        self.create_subscription(
            String,
            '/capture/state',
            lambda msg: setattr(self, 'capture_state', msg.data),
            10,
        )
        self.create_subscription(
            String,
            '/capture/status',
            lambda msg: setattr(self, 'status', msg.data),
            10,
        )
        self.create_subscription(
            String,
            '/capture/target_status',
            self._on_target_status,
            10,
        )
        self.create_timer(0.2, self._publish)

    def _on_targets(self, msg):
        self.target = msg.objects[0] if msg.objects else None

    def _on_state(self, msg):
        if msg.vehicle_id:
            self.states[msg.vehicle_id] = msg

    def _on_roles(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.capture_state = str(payload.get('state', self.capture_state))
        self.capture_radius = float(
            payload.get('capture_radius', self.capture_radius)
        )
        center = payload.get('center', self.capture_center)
        if isinstance(center, list) and len(center) >= 2:
            self.capture_center = (float(center[0]), float(center[1]))
        self.assignment_metadata = {
            str(item.get('vehicle_id')): item
            for item in payload.get('assignments', [])
            if item.get('vehicle_id')
        }

    def _on_target_status(self, msg):
        try:
            self.target_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.target_status = {}

    @staticmethod
    def _marker(marker_id, marker_type, namespace, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    @staticmethod
    def _set_color(marker, color, alpha=1.0):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = alpha

    def _append_target(self, result, stamp):
        if self.target is None:
            return
        target = self._marker(1, Marker.CUBE, 'target', stamp)
        target.pose = self.target.pose.pose
        target.scale.x, target.scale.y, target.scale.z = 7.0, 2.6, 2.0
        self._set_color(target, (0.92, 0.16, 0.12), 0.95)
        result.markers.append(target)

        label = self._marker(2, Marker.TEXT_VIEW_FACING, 'target', stamp)
        label.pose.position = self.target.pose.pose.position
        label.pose.position.z += 5.0
        label.scale.z = 2.2
        self._set_color(label, (0.12, 0.12, 0.12))
        speed = float(self.target_status.get('speed', 0.0))
        tracking_text = (
            'TRACKED' if self.target_status.get('tracked', False) else 'STALE'
        )
        label.text = '%s\n%s | %.1f m/s' % (
            self.target.track_id, tracking_text, speed
        )
        result.markers.append(label)

    def _append_prediction(self, result, stamp):
        if self.prediction is None or not self.prediction.poses:
            return
        line = self._marker(10, Marker.LINE_STRIP, 'prediction', stamp)
        line.scale.x = 0.65
        self._set_color(line, (1.0, 0.35, 0.05), 0.95)
        for pose in self.prediction.poses:
            line.points.append(Point(
                x=pose.pose.position.x,
                y=pose.pose.position.y,
                z=pose.pose.position.z,
            ))
        result.markers.append(line)

        endpoint = self._marker(11, Marker.SPHERE, 'prediction', stamp)
        endpoint.pose = self.prediction.poses[-1].pose
        endpoint.scale.x = endpoint.scale.y = endpoint.scale.z = 2.5
        self._set_color(endpoint, (1.0, 0.35, 0.05), 0.9)
        result.markers.append(endpoint)

    def _append_capture_radius(self, result, stamp):
        if not self.assignment_metadata:
            return
        circle = self._marker(20, Marker.LINE_STRIP, 'capture_radius', stamp)
        circle.scale.x = 0.55
        self._set_color(circle, (0.65, 0.10, 0.90), 0.9)
        segments = 72
        for index in range(segments + 1):
            angle = 2.0 * math.pi * index / segments
            circle.points.append(Point(
                x=self.capture_center[0] + self.capture_radius * math.cos(angle),
                y=self.capture_center[1] + self.capture_radius * math.sin(angle),
                z=1.1,
            ))
        result.markers.append(circle)

    def _append_assignments(self, result, stamp):
        if self.assignment_points is None:
            return
        ordered = sorted(
            self.assignment_metadata.items(),
            key=lambda item: int(item[1].get('index', 0)),
        )
        for color_index, (vehicle_id, metadata) in enumerate(ordered):
            point_index = int(metadata.get('index', -1))
            if not 0 <= point_index < len(self.assignment_points.poses):
                continue
            pose = self.assignment_points.poses[point_index]
            color = self.COLORS[color_index % len(self.COLORS)]
            sphere = self._marker(
                30 + color_index * 3, Marker.SPHERE, 'assignments', stamp
            )
            sphere.pose = pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 3.5
            self._set_color(sphere, color, 0.9)
            result.markers.append(sphere)

            label = self._marker(
                31 + color_index * 3,
                Marker.TEXT_VIEW_FACING,
                'assignments',
                stamp,
            )
            label.pose.position = pose.position
            label.pose.position.z += 3.0
            label.scale.z = 1.6
            self._set_color(label, (0.08, 0.08, 0.08))
            label.text = '%s\n%s' % (vehicle_id, metadata.get('role', ''))
            result.markers.append(label)

            state = self.states.get(vehicle_id)
            if state is not None:
                line = self._marker(
                    32 + color_index * 3,
                    Marker.LINE_LIST,
                    'assignment_lines',
                    stamp,
                )
                line.scale.x = 0.4
                self._set_color(line, color, 0.85)
                line.points = [
                    Point(
                        x=state.pose.position.x,
                        y=state.pose.position.y,
                        z=state.pose.position.z,
                    ),
                    Point(x=pose.position.x, y=pose.position.y, z=pose.position.z),
                ]
                result.markers.append(line)

    def _append_vehicles(self, result, stamp):
        for index, vehicle_id in enumerate(sorted(self.states)):
            state = self.states[vehicle_id]
            color = self.COLORS[index % len(self.COLORS)]
            marker = self._marker(
                100 + index * 2, Marker.SPHERE, 'vehicles', stamp
            )
            marker.pose = state.pose
            marker.scale.x = marker.scale.y = marker.scale.z = 2.6
            self._set_color(marker, color)
            result.markers.append(marker)

            role = self.assignment_metadata.get(vehicle_id, {}).get(
                'role', 'unassigned'
            )
            label = self._marker(
                101 + index * 2,
                Marker.TEXT_VIEW_FACING,
                'vehicle_roles',
                stamp,
            )
            label.pose.position = state.pose.position
            label.pose.position.z += 4.0
            label.scale.z = 1.7
            self._set_color(label, (0.08, 0.08, 0.08))
            label.text = '%s | %s' % (vehicle_id, role)
            result.markers.append(label)

    def _append_status(self, result, stamp):
        status = self._marker(200, Marker.TEXT_VIEW_FACING, 'status', stamp)
        status.pose.position.x = -5.0
        status.pose.position.y = -38.0
        status.pose.position.z = 8.0
        status.scale.z = 2.0
        self._set_color(status, (0.08, 0.08, 0.08))
        status.text = '%s\n%s' % (self.capture_state, self.status)
        result.markers.append(status)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        result = MarkerArray()
        self._append_target(result, stamp)
        self._append_prediction(result, stamp)
        self._append_capture_radius(result, stamp)
        self._append_assignments(result, stamp)
        self._append_vehicles(result, stamp)
        self._append_status(result, stamp)
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureVisualizer()
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
