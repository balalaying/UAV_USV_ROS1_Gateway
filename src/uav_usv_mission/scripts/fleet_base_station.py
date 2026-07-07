#!/usr/bin/env python3
import math
import time
import uuid

import cv2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import SensorStatus
from uav_usv_interfaces.msg import VehicleState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class SensorTracker:
    def __init__(self, vehicle_id, sensor_id, topic, message_type):
        self.vehicle_id = vehicle_id
        self.sensor_id = sensor_id
        self.topic = topic
        self.message_type = message_type
        self.last_received = 0.0
        self.rate_hz = 0.0
        self.total_messages = 0
        self.total_bytes = 0

    def update(self, size):
        now = time.monotonic()
        if self.last_received > 0.0:
            instant_rate = 1.0 / max(1e-3, now - self.last_received)
            if self.rate_hz <= 0.0:
                self.rate_hz = instant_rate
            else:
                self.rate_hz = 0.8 * self.rate_hz + 0.2 * instant_rate
        self.last_received = now
        self.total_messages += 1
        self.total_bytes += max(0, int(size))


class FleetBaseStation(Node):
    """Fleet command authority and visible sensor-data termination point."""

    def __init__(self):
        super().__init__('fleet_base_station')
        self.declare_parameter('owner_id', 'shore_base_station')
        self.declare_parameter('uav_id', 'uav_01')
        self.declare_parameter('usv_id', 'usv_01')
        self.declare_parameter('uav_ids', 'uav_01,uav_02,uav_03')
        self.declare_parameter('usv_ids', 'usv_01,usv_02,usv_03')
        self.declare_parameter('auto_demo', True)
        self.declare_parameter('target_x', 24.0)
        self.declare_parameter('target_y', 8.0)
        self.declare_parameter('uav_altitude', 16.0)
        self.declare_parameter('lease_duration', 5.0)

        self.owner_id = self.get_parameter('owner_id').value
        self.uav_ids = self._parse_id_list(
            self.get_parameter('uav_ids').value
        )
        self.usv_ids = self._parse_id_list(
            self.get_parameter('usv_ids').value
        )
        self.uav_id = self.get_parameter('uav_id').value
        self.usv_id = self.get_parameter('usv_id').value
        if self.uav_id not in self.uav_ids:
            self.uav_ids.insert(0, self.uav_id)
        if self.usv_id not in self.usv_ids:
            self.usv_ids.insert(0, self.usv_id)
        self.auto_demo = bool(self.get_parameter('auto_demo').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.uav_altitude = float(
            self.get_parameter('uav_altitude').value
        )
        self.lease_duration = float(
            self.get_parameter('lease_duration').value
        )
        self.lease_id = uuid.uuid4().hex

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        base_sensor_qos = QoSProfile(depth=1)
        base_sensor_qos.reliability = ReliabilityPolicy.RELIABLE
        base_sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.lease_pub = self.create_publisher(
            ControlLease, '/fleet/control_lease', lease_qos
        )
        self.command_pub = self.create_publisher(
            FleetCommand, '/fleet/command', 20
        )
        self.sensor_status_pub = self.create_publisher(
            SensorStatus, '/fleet/sensor_status', 20
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/fleet/base/markers', marker_qos
        )
        self.mosaic_pub = self.create_publisher(
            Image, '/fleet/base/camera_mosaic', base_sensor_qos
        )
        self.scan_pub = self.create_publisher(
            LaserScan, '/fleet/base/usv_scan', base_sensor_qos
        )

        self.create_subscription(
            VehicleState, '/fleet/state', self._on_state, sensor_qos
        )
        self.create_subscription(
            CommandAck, '/fleet/command_ack', self._on_ack, 20
        )
        self.create_subscription(
            PoseStamped,
            '/fleet/base/operator_goal',
            self._on_operator_goal,
            10,
        )
        self.create_subscription(
            String,
            '/fleet/base/operator_action',
            self._on_operator_action,
            10,
        )

        self.trackers = {}
        self.images = {}
        self.vehicle_states = {}
        self.command_status = {}
        self.command_counter = 0
        self.lease_publish_count = 0
        self.demo_ready_time = time.monotonic() + 4.0
        self.demo_stage = 'waiting'
        self.takeoff_command_id = ''

        for uav_id in self.uav_ids:
            self._add_image_sensor(
                uav_id,
                'down_camera',
                '/fleet/uplink/%s/camera' % uav_id,
                self._make_image_callback(uav_id),
                sensor_qos,
            )
        for usv_id in self.usv_ids:
            self._add_image_sensor(
                usv_id,
                'front_camera',
                '/fleet/uplink/%s/camera' % usv_id,
                self._make_image_callback(usv_id),
                sensor_qos,
            )
            scan_topic = '/fleet/uplink/%s/scan' % usv_id
            self.trackers[scan_topic] = SensorTracker(
                usv_id,
                'front_lidar',
                scan_topic,
                'sensor_msgs/LaserScan',
            )
            self.create_subscription(
                LaserScan,
                scan_topic,
                self._make_scan_callback(usv_id),
                sensor_qos,
            )
            odom_topic = '/fleet/uplink/%s/odom' % usv_id
            self.trackers[odom_topic] = SensorTracker(
                usv_id, 'navigation', odom_topic, 'nav_msgs/Odometry'
            )
            self.create_subscription(
                Odometry,
                odom_topic,
                self._make_odom_callback(usv_id),
                sensor_qos,
            )

        self.create_timer(1.0, self._publish_leases)
        self.create_timer(1.0, self._publish_sensor_status)
        self.create_timer(0.5, self._publish_markers)
        self.create_timer(1.0 / 15.0, self._publish_camera_mosaic)
        self.create_timer(0.5, self._advance_demo)
        self.get_logger().info(
            'Base station %s online; lease=%s, vehicles=%s/%s, target=(%.1f, %.1f)'
            % (
                self.owner_id,
                self.lease_id[:8],
                ','.join(self.usv_ids),
                ','.join(self.uav_ids),
                self.target_x,
                self.target_y,
            )
        )

    @staticmethod
    def _parse_id_list(value):
        ids = []
        for item in str(value).split(','):
            item = item.strip()
            if item and item not in ids:
                ids.append(item)
        return ids

    def _add_image_sensor(
        self, vehicle_id, sensor_id, topic, callback, qos
    ):
        self.trackers[topic] = SensorTracker(
            vehicle_id, sensor_id, topic, 'sensor_msgs/Image'
        )
        self.create_subscription(Image, topic, callback, qos)

    def _make_image_callback(self, vehicle_id):
        def callback(msg):
            topic = '/fleet/uplink/%s/camera' % vehicle_id
            self.trackers[topic].update(len(msg.data))
            self.images[vehicle_id] = msg

        return callback

    def _make_scan_callback(self, vehicle_id):
        def callback(msg):
            topic = '/fleet/uplink/%s/scan' % vehicle_id
            self.trackers[topic].update(len(msg.ranges) * 4)
            if vehicle_id == self.usv_id:
                self.scan_pub.publish(msg)

        return callback

    def _make_odom_callback(self, vehicle_id):
        def callback(_msg):
            topic = '/fleet/uplink/%s/odom' % vehicle_id
            self.trackers[topic].update(256)

        return callback

    def _on_state(self, msg):
        self.vehicle_states[msg.vehicle_id] = (msg, time.monotonic())

    def _on_ack(self, msg):
        self.command_status[msg.command_id] = msg
        self.get_logger().info(
            'ACK %s from %s: status=%d progress=%.0f%% %s'
            % (
                msg.command_id,
                msg.vehicle_id,
                msg.status,
                msg.progress * 100.0,
                msg.message,
            )
        )

    def _on_operator_goal(self, msg):
        self.target_x = float(msg.pose.position.x)
        self.target_y = float(msg.pose.position.y)
        altitude = float(msg.pose.position.z)
        if altitude <= 0.0:
            altitude = self.uav_altitude
        self.get_logger().info(
            'OPERATOR cooperative goal: x=%.1f y=%.1f z=%.1f'
            % (self.target_x, self.target_y, altitude)
        )
        for usv_id in self.usv_ids:
            self._send_command(
                usv_id,
                FleetCommand.COMMAND_NAVIGATE,
                target=(self.target_x, self.target_y, 0.0),
            )
        for uav_id in self.uav_ids:
            self._send_command(
                uav_id,
                FleetCommand.COMMAND_NAVIGATE,
                target=(self.target_x, self.target_y, altitude),
            )

    def _on_operator_action(self, msg):
        action = msg.data.strip().upper()
        if action == 'TAKEOFF':
            for uav_id in self.uav_ids:
                state_entry = self.vehicle_states.get(uav_id)
                if state_entry is not None and state_entry[0].armed:
                    self.get_logger().warn(
                        'Ignoring TAKEOFF for %s: UAV is already armed'
                        % uav_id
                    )
                    continue
                self._send_command(
                    uav_id,
                    FleetCommand.COMMAND_TAKEOFF,
                    parameters=[self.uav_altitude],
                )
        elif action == 'HOLD_ALL':
            for vehicle_id in self.uav_ids + self.usv_ids:
                self._send_command(vehicle_id, FleetCommand.COMMAND_HOLD)
        elif action == 'EMERGENCY_STOP':
            for vehicle_id in self.uav_ids + self.usv_ids:
                self._send_command(
                    vehicle_id, FleetCommand.COMMAND_EMERGENCY_STOP
                )
        else:
            self.get_logger().warn('Unknown operator action: %s' % action)

    def _future_stamp(self, seconds):
        nanoseconds = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        stamp = self.get_clock().now().to_msg()
        stamp.sec = nanoseconds // 1000000000
        stamp.nanosec = nanoseconds % 1000000000
        return stamp

    def _publish_leases(self):
        for vehicle_id in self.uav_ids + self.usv_ids:
            lease = ControlLease()
            lease.header.stamp = self.get_clock().now().to_msg()
            lease.vehicle_id = vehicle_id
            lease.lease_id = self.lease_id
            lease.owner_id = self.owner_id
            lease.priority = 200
            lease.valid_until = self._future_stamp(self.lease_duration)
            lease.revoked = False
            self.lease_pub.publish(lease)
        self.lease_publish_count += 1

    def _publish_sensor_status(self):
        now = time.monotonic()
        summary = []
        for tracker in self.trackers.values():
            age = (
                now - tracker.last_received
                if tracker.last_received > 0.0
                else float('inf')
            )
            status = SensorStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.vehicle_id = tracker.vehicle_id
            status.sensor_id = tracker.sensor_id
            status.uplink_topic = tracker.topic
            status.message_type = tracker.message_type
            status.measured_rate_hz = float(tracker.rate_hz)
            status.age_seconds = float(min(age, 9999.0))
            status.total_messages = tracker.total_messages
            status.total_bytes = tracker.total_bytes
            status.healthy = age < 2.0 and tracker.rate_hz > 0.2
            self.sensor_status_pub.publish(status)
            summary.append(
                '%s/%s=%s %.1fHz'
                % (
                    tracker.vehicle_id,
                    tracker.sensor_id,
                    'OK' if status.healthy else 'WAIT',
                    tracker.rate_hz,
                )
            )
        self.get_logger().info(
            'SENSOR UPLINK: ' + ' | '.join(summary),
            throttle_duration_sec=5.0,
        )

    @staticmethod
    def _decode_image(msg):
        channels = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
        }.get(msg.encoding.lower())
        if channels is None:
            raise ValueError('unsupported encoding %s' % msg.encoding)
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step
        )
        image = rows[:, :msg.width * channels].reshape(
            msg.height, msg.width, channels
        )
        encoding = msg.encoding.lower()
        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image.copy()

    def _camera_panel(self, msg, title, tracker):
        width, height = 480, 360
        if msg is None:
            panel = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(
                panel,
                'WAITING FOR SENSOR UPLINK',
                (55, 190),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            panel = cv2.resize(
                self._decode_image(msg),
                (width, height),
                interpolation=cv2.INTER_AREA,
            )
        cv2.rectangle(panel, (0, 0), (width, 48), (0, 0, 0), -1)
        cv2.putText(
            panel,
            '%s  %.1f FPS' % (title, tracker.rate_hz),
            (14, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (70, 255, 90),
            2,
            cv2.LINE_AA,
        )
        return panel

    def _publish_camera_mosaic(self):
        if self.mosaic_pub.get_subscription_count() == 0:
            return
        panels = []
        try:
            for usv_id in self.usv_ids:
                tracker = self.trackers[
                    '/fleet/uplink/%s/camera' % usv_id
                ]
                panels.append(
                    self._camera_panel(
                        self.images.get(usv_id),
                        '%s FRONT CAMERA' % usv_id.upper(),
                        tracker,
                    )
                )
            for uav_id in self.uav_ids:
                tracker = self.trackers[
                    '/fleet/uplink/%s/camera' % uav_id
                ]
                panels.append(
                    self._camera_panel(
                        self.images.get(uav_id),
                        '%s DOWN CAMERA' % uav_id.upper(),
                        tracker,
                    )
                )
            if not panels:
                return
            while len(panels) % 3:
                panels.append(np.zeros_like(panels[0]))
            rows = [
                np.hstack(panels[index:index + 3])
                for index in range(0, len(panels), 3)
            ]
            mosaic = np.vstack(rows)
        except Exception as exc:
            self.get_logger().warn(
                'Unable to build base camera mosaic: %s' % exc,
                throttle_duration_sec=5.0,
            )
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = mosaic.shape[0]
        msg.width = mosaic.shape[1]
        msg.encoding = 'bgr8'
        msg.step = mosaic.shape[1] * 3
        msg.data = mosaic.tobytes()
        self.mosaic_pub.publish(msg)

    def _send_command(
        self, vehicle_id, command_type, target=None, parameters=None
    ):
        self.command_counter += 1
        command = FleetCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'map'
        command.command_id = '%s-%03d' % (
            vehicle_id,
            self.command_counter,
        )
        command.vehicle_id = vehicle_id
        command.lease_id = self.lease_id
        command.command_type = command_type
        command.priority = 200
        command.expires_at = self._future_stamp(30.0)
        if target is not None:
            command.target_pose.position.x = float(target[0])
            command.target_pose.position.y = float(target[1])
            command.target_pose.position.z = float(target[2])
            command.target_pose.orientation.w = 1.0
        command.parameters = list(parameters or [])
        self.command_pub.publish(command)
        self.get_logger().info(
            'COMMAND %s -> %s type=%d'
            % (
                command.command_id,
                command.vehicle_id,
                command.command_type,
            )
        )
        return command.command_id

    def _vehicle_online(self, vehicle_id):
        entry = self.vehicle_states.get(vehicle_id)
        return (
            entry is not None
            and entry[0].online
            and time.monotonic() - entry[1] < 2.0
        )

    def _advance_demo(self):
        if not self.auto_demo:
            return
        if self.demo_stage == 'waiting':
            if (
                self.lease_publish_count < 2
                or time.monotonic() < self.demo_ready_time
            ):
                return
            if not (
                self._vehicle_online(self.uav_id)
                and self._vehicle_online(self.usv_id)
            ):
                return
            self.takeoff_command_id = self._send_command(
                self.uav_id,
                FleetCommand.COMMAND_TAKEOFF,
                parameters=[self.uav_altitude],
            )
            for usv_id in self.usv_ids:
                self._send_command(
                    usv_id,
                    FleetCommand.COMMAND_NAVIGATE,
                    target=(self.target_x, self.target_y, 0.0),
                )
            for uav_id in self.uav_ids[1:]:
                self._send_command(
                    uav_id,
                    FleetCommand.COMMAND_TAKEOFF,
                    parameters=[self.uav_altitude],
                )
            self.demo_stage = 'taking_off'
        elif self.demo_stage == 'taking_off':
            ack = self.command_status.get(self.takeoff_command_id)
            if ack is None:
                return
            if ack.status == CommandAck.STATUS_REJECTED:
                self.takeoff_command_id = self._send_command(
                    self.uav_id,
                    FleetCommand.COMMAND_TAKEOFF,
                    parameters=[self.uav_altitude],
                )
                return
            if ack.status != CommandAck.STATUS_SUCCEEDED:
                return
            self._send_command(
                self.uav_id,
                FleetCommand.COMMAND_NAVIGATE,
                target=(
                    self.target_x,
                    self.target_y,
                    self.uav_altitude,
                ),
            )
            for uav_id in self.uav_ids[1:]:
                self._send_command(
                    uav_id,
                    FleetCommand.COMMAND_NAVIGATE,
                    target=(
                        self.target_x,
                        self.target_y,
                        self.uav_altitude,
                    ),
                )
            self.demo_stage = 'cooperative_navigation'

    def _publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        for vehicle_id, (state, _) in self.vehicle_states.items():
            body = Marker()
            body.header.stamp = stamp
            body.header.frame_id = 'map'
            body.ns = 'fleet_vehicles'
            body.id = marker_id
            marker_id += 1
            body.type = (
                Marker.SPHERE
                if state.vehicle_type == VehicleState.TYPE_UAV
                else Marker.CUBE
            )
            body.action = Marker.ADD
            body.pose = state.pose
            body.scale.x = 2.2
            body.scale.y = 1.5
            body.scale.z = 0.8
            body.color.r = 0.1
            body.color.g = 0.7 if state.online else 0.1
            body.color.b = 1.0
            body.color.a = 0.95
            markers.markers.append(body)

            text = Marker()
            text.header = body.header
            text.ns = 'fleet_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = state.pose.position.x
            text.pose.position.y = state.pose.position.y
            text.pose.position.z = state.pose.position.z + 2.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.9
            text.color.r = 0.05
            text.color.g = 0.05
            text.color.b = 0.05
            text.color.a = 1.0
            text.text = '%s | %s' % (vehicle_id, state.status_text)
            markers.markers.append(text)

        target = Marker()
        target.header.stamp = stamp
        target.header.frame_id = 'map'
        target.ns = 'base_target'
        target.id = marker_id
        marker_id += 1
        target.type = Marker.CYLINDER
        target.action = Marker.ADD
        target.pose.position.x = self.target_x
        target.pose.position.y = self.target_y
        target.pose.position.z = 0.3
        target.pose.orientation.w = 1.0
        target.scale.x = 2.5
        target.scale.y = 2.5
        target.scale.z = 0.6
        target.color.r = 1.0
        target.color.g = 0.15
        target.color.b = 0.05
        target.color.a = 0.95
        target.lifetime = Duration(seconds=0.0).to_msg()
        markers.markers.append(target)

        sensor_lines = []
        now = time.monotonic()
        for tracker in self.trackers.values():
            age = now - tracker.last_received if tracker.last_received else 9999
            sensor_lines.append(
                '%s/%s %s %.1fHz'
                % (
                    tracker.vehicle_id,
                    tracker.sensor_id,
                    'OK' if age < 2.0 else 'OFFLINE',
                    tracker.rate_hz,
                )
            )
        panel = Marker()
        panel.header = target.header
        panel.ns = 'base_sensor_status'
        panel.id = marker_id
        panel.type = Marker.TEXT_VIEW_FACING
        panel.action = Marker.ADD
        panel.pose.position.x = self.target_x
        panel.pose.position.y = self.target_y
        panel.pose.position.z = 5.0
        panel.pose.orientation.w = 1.0
        panel.scale.z = 0.65
        panel.color.r = 0.05
        panel.color.g = 0.05
        panel.color.b = 0.05
        panel.color.a = 1.0
        panel.text = 'BASE SENSOR UPLINK\n' + '\n'.join(sensor_lines)
        markers.markers.append(panel)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FleetBaseStation()
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
