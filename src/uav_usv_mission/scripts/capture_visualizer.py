#!/usr/bin/env python3
"""RViz markers for the minimal dynamic capture closed loop."""

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
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
    def __init__(self):
        super().__init__('capture_visualizer')
        self.target = None
        self.states = {}
        self.uav_point = None
        self.usv_point = None
        self.status = 'starting'
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
            PoseStamped,
            '/capture/uav_observation_point',
            lambda msg: setattr(self, 'uav_point', msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/capture/usv_intercept_point',
            lambda msg: setattr(self, 'usv_point', msg),
            10,
        )
        self.create_subscription(
            String,
            '/capture/status',
            lambda msg: setattr(self, 'status', msg.data),
            10,
        )
        self.create_timer(0.2, self._publish)

    def _on_targets(self, msg):
        self.target = msg.objects[0] if msg.objects else None

    def _on_state(self, msg):
        if msg.vehicle_id in ('uav_01', 'usv_01'):
            self.states[msg.vehicle_id] = msg

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

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        result = MarkerArray()
        if self.target is not None:
            target = self._marker(1, Marker.CUBE, 'target', stamp)
            target.pose = self.target.pose.pose
            target.scale.x, target.scale.y, target.scale.z = 7.0, 2.6, 2.0
            target.color.r, target.color.g, target.color.b, target.color.a = (
                0.92, 0.16, 0.12, 0.95
            )
            result.markers.append(target)
            label = self._marker(2, Marker.TEXT_VIEW_FACING, 'target', stamp)
            label.pose.position = self.target.pose.pose.position
            label.pose.position.z += 5.0
            label.scale.z = 2.2
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = 'target_vessel'
            result.markers.append(label)

        points = (
            ('uav observation', self.uav_point, 0.1, 0.85, 1.0, 10),
            ('usv intercept', self.usv_point, 0.1, 0.45, 1.0, 20),
        )
        for label_text, point, red, green, blue, marker_id in points:
            if point is None:
                continue
            sphere = self._marker(marker_id, Marker.SPHERE, 'assignments', stamp)
            sphere.pose = point.pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 3.5
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (
                red, green, blue, 0.9
            )
            result.markers.append(sphere)
            label = self._marker(
                marker_id + 1, Marker.TEXT_VIEW_FACING, 'assignments', stamp
            )
            label.pose.position = point.pose.position
            label.pose.position.z += 3.0
            label.scale.z = 1.8
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = label_text
            result.markers.append(label)

        for index, vehicle_id in enumerate(('uav_01', 'usv_01')):
            state = self.states.get(vehicle_id)
            if state is None:
                continue
            marker = self._marker(30 + index, Marker.SPHERE, 'vehicles', stamp)
            marker.pose = state.pose
            marker.scale.x = marker.scale.y = marker.scale.z = 2.6
            if vehicle_id == 'uav_01':
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.9, 0.35
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.45, 1.0
            marker.color.a = 1.0
            result.markers.append(marker)

        if self.target is not None:
            line = self._marker(50, Marker.LINE_LIST, 'assignments', stamp)
            line.scale.x = 0.45
            line.color.r, line.color.g, line.color.b, line.color.a = (
                1.0, 0.75, 0.1, 0.9
            )
            origin = self.target.pose.pose.position
            for point in (self.uav_point, self.usv_point):
                if point is None:
                    continue
                line.points.extend([
                    Point(x=origin.x, y=origin.y, z=origin.z + 1.0),
                    Point(
                        x=point.pose.position.x,
                        y=point.pose.position.y,
                        z=point.pose.position.z,
                    ),
                ])
            result.markers.append(line)

        status = self._marker(60, Marker.TEXT_VIEW_FACING, 'status', stamp)
        status.pose.position.x = -5.0
        status.pose.position.y = -38.0
        status.pose.position.z = 8.0
        status.scale.z = 2.0
        status.color.r = status.color.g = status.color.b = status.color.a = 1.0
        status.text = self.status
        result.markers.append(status)
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
