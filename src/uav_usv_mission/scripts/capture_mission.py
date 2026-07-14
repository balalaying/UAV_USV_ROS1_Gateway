#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


@dataclass
class CaptureTarget:
    track_id: str
    label: str
    color: str
    x: float
    y: float
    z: float = 0.0


class CaptureMission(Node):
    """UAV perception and UAV-USV cooperative capture mission mock-up."""

    COLORS = {
        'red': (0.92, 0.08, 0.05),
        'green': (0.05, 0.75, 0.20),
        'blue': (0.05, 0.25, 0.95),
        'yellow': (1.0, 0.82, 0.05),
        'purple': (0.58, 0.18, 0.86),
        'cyan': (0.05, 0.85, 0.92),
    }

    def __init__(self):
        super().__init__('capture_mission')
        self.declare_parameter('topic_namespace', '')
        self.declare_parameter('uav_id', 'uav_01')
        self.declare_parameter('usv_ids', 'usv_01,usv_02,usv_03,usv_04')
        self.declare_parameter('uav_ids', 'uav_01,uav_02,uav_03,uav_04')
        self.declare_parameter('capture_radius', 38.0)
        self.declare_parameter('uav_capture_altitude', 32.0)
        self.declare_parameter('patrol_radius', 260.0)
        self.declare_parameter('patrol_period', 95.0)
        self.declare_parameter('detect_range', 360.0)
        self.declare_parameter(
            'targets',
            (
                'red_lighthouse:red:-260:-120;'
                'green_lighthouse:green:250:-210;'
                'blue_lighthouse:blue:320:170;'
                'yellow_lighthouse:yellow:-330:230;'
                'purple_lighthouse:purple:35:310;'
                'cyan_lighthouse:cyan:0:-360'
            ),
        )

        self.uav_id = str(self.get_parameter('uav_id').value)
        self.usv_ids = [
            item.strip()
            for item in str(self.get_parameter('usv_ids').value).split(',')
            if item.strip()
        ]
        self.uav_ids = [
            item.strip()
            for item in str(self.get_parameter('uav_ids').value).split(',')
            if item.strip()
        ]
        self.targets = self._parse_targets(
            str(self.get_parameter('targets').value)
        )
        self.leases = {}
        self.active_target = None
        self.capture_dispatched = False
        self.start_time = time.monotonic()
        namespace = str(
            self.get_parameter('topic_namespace').value
        ).strip('/')
        self._topic = (
            (lambda name: '/%s%s' % (namespace, name))
            if namespace
            else (lambda name: name)
        )

        self.target_pub = self.create_publisher(
            TrackedObjectArray, self._topic('/fleet/perception/targets'), 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, self._topic('/fleet/capture/markers'), 10
        )
        self.status_pub = self.create_publisher(
            String, self._topic('/fleet/capture/status'), 10
        )
        self.command_pub = self.create_publisher(
            FleetCommand, self._topic('/fleet/command'), 20
        )
        self.selected_target_pub = self.create_publisher(
            PoseStamped,
            self._topic('/fleet/base/selected_target'),
            10,
        )

        self.create_subscription(
            ControlLease,
            self._topic('/fleet/control_lease'),
            self._on_lease,
            20,
        )
        self.create_subscription(
            String,
            self._topic('/fleet/base/operator_action'),
            self._on_operator_action,
            20,
        )
        self.create_timer(0.2, self._publish_perception)
        self.create_timer(0.2, self._publish_markers)
        self.create_timer(0.5, self._dispatch_pending_capture)
        self.status_pub.publish(String(data='mode=idle target=none'))
        self.get_logger().info(
            'Capture mission online: perception=%s targets=%d usv=%s'
            % (self.uav_id, len(self.targets), ','.join(self.usv_ids))
        )

    def _parse_targets(self, spec):
        targets = []
        for entry in spec.split(';'):
            entry = entry.strip()
            if not entry:
                continue
            parts = entry.split(':')
            if len(parts) != 4:
                raise ValueError(
                    'target entry must be id:color:x:y, got %s' % entry
                )
            track_id, color, x, y = parts
            targets.append(
                CaptureTarget(
                    track_id=track_id,
                    label=track_id.replace('_', ' '),
                    color=color,
                    x=float(x),
                    y=float(y),
                )
            )
        return targets

    def _on_lease(self, msg):
        if not msg.revoked:
            self.leases[msg.vehicle_id] = msg
            self._dispatch_pending_capture()

    def _on_operator_action(self, msg):
        action = msg.data.strip()
        upper = action.upper()
        if upper.startswith('CAPTURE:'):
            track_id = action.split(':', 1)[1].strip()
            self._start_capture(track_id)
        elif upper == 'CANCEL_CAPTURE':
            self.active_target = None
            self.capture_dispatched = False
            self.status_pub.publish(String(data='mode=idle target=none'))
            self.get_logger().info('Capture mission canceled')

    def _start_capture(self, track_id):
        target = next(
            (item for item in self.targets if item.track_id == track_id),
            None,
        )
        if target is None:
            self.get_logger().warn('Unknown capture target: %s' % track_id)
            return
        self.active_target = target
        self.capture_dispatched = False
        self._publish_selected_target(target)
        self.status_pub.publish(
            String(
                data=(
                    'mode=capture target=%s x=%.1f y=%.1f radius=%.1f'
                    % (
                        target.track_id,
                        target.x,
                        target.y,
                        float(self.get_parameter('capture_radius').value),
                    )
                )
            )
        )
        self.get_logger().info(
            'Capture started for %s at (%.1f, %.1f)'
            % (target.track_id, target.x, target.y)
        )
        self._dispatch_pending_capture()

    def _dispatch_pending_capture(self):
        if self.active_target is None or self.capture_dispatched:
            return
        required = self.usv_ids + self.uav_ids
        if any(vehicle_id not in self.leases for vehicle_id in required):
            return
        self._send_capture_commands(self.active_target)
        self.capture_dispatched = True
        self.status_pub.publish(
            String(
                data='mode=capture target=%s dispatched=true'
                % self.active_target.track_id
            )
        )

    def _publish_selected_target(self, target):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = target.x
        msg.pose.position.y = target.y
        msg.pose.position.z = target.z
        msg.pose.orientation.w = 1.0
        self.selected_target_pub.publish(msg)

    def _send_capture_commands(self, target):
        radius = float(self.get_parameter('capture_radius').value)
        usv_count = max(1, len(self.usv_ids))
        for index, usv_id in enumerate(self.usv_ids):
            angle = 2.0 * math.pi * index / usv_count
            self._send_command(
                usv_id,
                FleetCommand.COMMAND_NAVIGATE,
                target.x + math.cos(angle) * radius,
                target.y + math.sin(angle) * radius,
                0.0,
                [radius, float(index)],
            )
        altitude = float(self.get_parameter('uav_capture_altitude').value)
        for index, uav_id in enumerate(self.uav_ids):
            offset = 8.0 * index
            self._send_command(
                uav_id,
                FleetCommand.COMMAND_NAVIGATE,
                target.x + offset,
                target.y - offset,
                altitude + 4.0 * index,
                [radius, float(index)],
            )

    def _send_command(self, vehicle_id, command_type, x, y, z, parameters):
        lease = self.leases.get(vehicle_id) or self.leases.get('*')
        if lease is None:
            self.get_logger().warn(
                'No base-station lease for %s; capture command skipped'
                % vehicle_id
            )
            return
        msg = FleetCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.command_id = '%s-capture-%d' % (
            vehicle_id,
            int(time.monotonic() * 1000) % 1000000,
        )
        msg.vehicle_id = vehicle_id
        msg.lease_id = lease.lease_id
        msg.command_type = command_type
        msg.priority = 210
        msg.target_pose.position.x = x
        msg.target_pose.position.y = y
        msg.target_pose.position.z = z
        msg.target_pose.orientation.w = 1.0
        msg.parameters = [float(value) for value in parameters]
        self.command_pub.publish(msg)

    def _uav_patrol_pose(self):
        elapsed = time.monotonic() - self.start_time
        radius = float(self.get_parameter('patrol_radius').value)
        period = max(5.0, float(self.get_parameter('patrol_period').value))
        angle = 2.0 * math.pi * (elapsed % period) / period
        return radius * math.cos(angle), radius * math.sin(angle), angle

    def _visible_targets(self):
        uav_x, uav_y, _angle = self._uav_patrol_pose()
        detect_range = float(self.get_parameter('detect_range').value)
        visible = []
        for target in self.targets:
            distance = math.hypot(target.x - uav_x, target.y - uav_y)
            if distance <= detect_range:
                visible.append((target, distance))
        visible.sort(key=lambda item: item[1])
        return visible

    def _publish_perception(self):
        now = self.get_clock().now().to_msg()
        array = TrackedObjectArray()
        array.header.stamp = now
        array.header.frame_id = 'map'
        for target, distance in self._visible_targets():
            obj = TrackedObject()
            obj.track_id = target.track_id
            obj.first_seen = now
            obj.last_update = now
            obj.source_mask = TrackedObject.SOURCE_CAMERA
            obj.classification = TrackedObject.CLASS_LANDMARK
            obj.pose.pose.position.x = target.x
            obj.pose.pose.position.y = target.y
            obj.pose.pose.position.z = target.z
            obj.pose.pose.orientation.w = 1.0
            obj.dimensions.x = 4.0
            obj.dimensions.y = 4.0
            obj.dimensions.z = 12.0
            obj.confidence = float(max(0.50, min(0.98, 1.0 - distance / 900.0)))
            array.objects.append(obj)
        self.target_pub.publish(array)

    def _publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0
        for target in self.targets:
            color = self.COLORS.get(target.color, (1.0, 1.0, 1.0))
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'map'
            marker.ns = 'capture_targets'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = target.x
            marker.pose.position.y = target.y
            marker.pose.position.z = 6.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 5.0
            marker.scale.y = 5.0
            marker.scale.z = 12.0
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.88
            marker.lifetime = Duration(sec=0, nanosec=0)
            markers.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = 'capture_target_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = target.x
            label.pose.position.y = target.y
            label.pose.position.z = 15.5
            label.pose.orientation.w = 1.0
            label.scale.z = 6.0
            label.color.r = 0.05
            label.color.g = 0.05
            label.color.b = 0.05
            label.color.a = 1.0
            label.text = target.track_id
            markers.markers.append(label)

        uav_x, uav_y, angle = self._uav_patrol_pose()
        uav = Marker()
        uav.header.stamp = stamp
        uav.header.frame_id = 'map'
        uav.ns = 'capture_uav_patrol'
        uav.id = marker_id
        marker_id += 1
        uav.type = Marker.ARROW
        uav.action = Marker.ADD
        uav.pose.position.x = uav_x
        uav.pose.position.y = uav_y
        uav.pose.position.z = float(self.get_parameter('uav_capture_altitude').value)
        uav.pose.orientation.z = math.sin(angle * 0.5)
        uav.pose.orientation.w = math.cos(angle * 0.5)
        uav.scale.x = 14.0
        uav.scale.y = 4.0
        uav.scale.z = 4.0
        uav.color.r = 0.1
        uav.color.g = 0.45
        uav.color.b = 1.0
        uav.color.a = 0.95
        markers.markers.append(uav)

        if self.active_target is not None:
            self._append_capture_markers(markers, stamp, marker_id)
        self.marker_pub.publish(markers)

    def _append_capture_markers(self, markers, stamp, marker_id):
        target = self.active_target
        radius = float(self.get_parameter('capture_radius').value)
        ring = Marker()
        ring.header.stamp = stamp
        ring.header.frame_id = 'map'
        ring.ns = 'capture_ring'
        ring.id = marker_id
        marker_id += 1
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.pose.position.x = target.x
        ring.pose.position.y = target.y
        ring.pose.position.z = 0.08
        ring.pose.orientation.w = 1.0
        ring.scale.x = radius * 2.0
        ring.scale.y = radius * 2.0
        ring.scale.z = 0.08
        ring.color.r = 0.0
        ring.color.g = 0.65
        ring.color.b = 1.0
        ring.color.a = 0.22
        markers.markers.append(ring)

        for index, usv_id in enumerate(self.usv_ids):
            angle = 2.0 * math.pi * index / max(1, len(self.usv_ids))
            point = Marker()
            point.header = ring.header
            point.ns = 'capture_slots'
            point.id = marker_id
            marker_id += 1
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose.position.x = target.x + math.cos(angle) * radius
            point.pose.position.y = target.y + math.sin(angle) * radius
            point.pose.position.z = 1.0
            point.pose.orientation.w = 1.0
            point.scale.x = 5.5
            point.scale.y = 5.5
            point.scale.z = 2.5
            point.color.r = 0.0
            point.color.g = 0.85
            point.color.b = 1.0
            point.color.a = 0.9
            markers.markers.append(point)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureMission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
