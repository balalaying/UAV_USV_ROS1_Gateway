#!/usr/bin/env python3
import math
import threading
import time
import uuid

import cv2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw
from PIL import ImageFont
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
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

    VEHICLE_DISPLAY_NAMES = {
        'usv_01': '我方船一号（蓝色）',
        'usv_02': '我方船二号（绿色）',
        'usv_03': '我方船三号（青色）',
        'uav_01': '我方无人机一号',
        'uav_02': '我方无人机二号',
        'uav_03': '我方无人机三号',
    }

    def __init__(self):
        super().__init__('fleet_base_station')
        self.declare_parameter('topic_namespace', '')
        self.declare_parameter('owner_id', 'shore_base_station')
        self.declare_parameter('uav_id', 'uav_01')
        self.declare_parameter('usv_id', 'usv_01')
        self.declare_parameter('uav_ids', 'uav_01,uav_02,uav_03')
        self.declare_parameter('usv_ids', 'usv_01,usv_02,usv_03')
        self.declare_parameter('auto_demo', True)
        self.declare_parameter('monitor_only', False)
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
        self.monitor_only = bool(self.get_parameter('monitor_only').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.uav_altitude = float(
            self.get_parameter('uav_altitude').value
        )
        self.lease_duration = float(
            self.get_parameter('lease_duration').value
        )
        self.lease_id = uuid.uuid4().hex

        self.topic_namespace = str(
            self.get_parameter('topic_namespace').value
        ).strip('/')

        def topic(name):
            if not self.topic_namespace:
                return name
            return '/%s%s' % (self.topic_namespace, name)

        self._topic = topic

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
        mosaic_qos = QoSProfile(depth=1)
        mosaic_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        mosaic_qos.durability = DurabilityPolicy.VOLATILE

        self.lease_pub = self.create_publisher(
            ControlLease, self._topic('/fleet/control_lease'), lease_qos
        )
        self.command_pub = self.create_publisher(
            FleetCommand, self._topic('/fleet/command'), 20
        )
        self.sensor_status_pub = self.create_publisher(
            SensorStatus, self._topic('/fleet/sensor_status'), 20
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, self._topic('/fleet/base/markers'), marker_qos
        )
        self.mosaic_pub = self.create_publisher(
            Image, self._topic('/fleet/base/camera_mosaic'), mosaic_qos
        )
        self.scan_pub = self.create_publisher(
            LaserScan, self._topic('/fleet/base/usv_scan'), base_sensor_qos
        )

        self.create_subscription(
            VehicleState,
            self._topic('/fleet/state'),
            self._on_state,
            sensor_qos,
        )
        self.create_subscription(
            CommandAck, self._topic('/fleet/command_ack'), self._on_ack, 20
        )
        self.create_subscription(
            PoseStamped,
            self._topic('/fleet/base/operator_goal'),
            self._on_operator_goal,
            10,
        )
        self.create_subscription(
            String,
            self._topic('/fleet/base/operator_action'),
            self._on_operator_action,
            10,
        )

        self.trackers = {}
        self.images = {}
        self.camera_frames = {}
        self.camera_frame_versions = {}
        self.decoded_camera_versions = {}
        self.camera_frame_version = 0
        self.last_mosaic_version = -1
        self.title_font = self._load_title_font()
        self.camera_lock = threading.Lock()
        self.vehicle_states = {}
        self.command_status = {}
        self.command_counter = 0
        self.lease_publish_count = 0
        self.demo_ready_time = time.monotonic() + 4.0
        self.demo_stage = 'waiting'
        self.takeoff_command_id = ''
        self.image_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        radar_topic = self._topic('/fleet/base/radar/scan')
        self.trackers[radar_topic] = SensorTracker(
            self.owner_id,
            'base_radar',
            radar_topic,
            'sensor_msgs/LaserScan',
        )
        self.create_subscription(
            LaserScan,
            radar_topic,
            self._on_base_radar_scan,
            sensor_qos,
        )

        for uav_id in self.uav_ids:
            camera_topic = self._topic(
                '/fleet/uplink/%s/camera/image_raw' % uav_id
            )
            self._add_image_sensor(
                uav_id,
                'uav_camera',
                camera_topic,
                self._make_image_callback(uav_id, camera_topic),
                sensor_qos,
                self.image_callback_group,
            )
        for usv_id in self.usv_ids:
            camera_topic = self._topic(
                '/fleet/uplink/%s/camera' % usv_id
            )
            self._add_image_sensor(
                usv_id,
                'front_camera',
                camera_topic,
                self._make_image_callback(usv_id, camera_topic),
                sensor_qos,
                self.image_callback_group,
            )
            scan_topic = self._topic('/fleet/uplink/%s/scan' % usv_id)
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
            odom_topic = self._topic('/fleet/uplink/%s/odom' % usv_id)
            self.trackers[odom_topic] = SensorTracker(
                usv_id, 'navigation', odom_topic, 'nav_msgs/Odometry'
            )
            self.create_subscription(
                Odometry,
                odom_topic,
                self._make_odom_callback(usv_id),
                sensor_qos,
            )

        if not self.monitor_only:
            self.create_timer(
                1.0,
                self._publish_leases,
                callback_group=self.timer_callback_group,
            )
        self.create_timer(
            1.0,
            self._publish_sensor_status,
            callback_group=self.timer_callback_group,
        )
        self.create_timer(
            0.5, self._publish_markers, callback_group=self.timer_callback_group
        )
        self.create_timer(
            1.0 / 15.0,
            self._publish_camera_mosaic,
            callback_group=self.timer_callback_group,
        )
        if not self.monitor_only:
            self.create_timer(
                0.5,
                self._advance_demo,
                callback_group=self.timer_callback_group,
            )
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
        self, vehicle_id, sensor_id, topic, callback, qos, callback_group
    ):
        self.trackers[topic] = SensorTracker(
            vehicle_id, sensor_id, topic, 'sensor_msgs/Image'
        )
        self.create_subscription(
            Image, topic, callback, qos, callback_group=callback_group
        )

    def _make_image_callback(self, vehicle_id, topic):
        def callback(msg):
            self.trackers[topic].update(len(msg.data))
            with self.camera_lock:
                self.images[vehicle_id] = msg
                self.camera_frame_versions[vehicle_id] = (
                    self.camera_frame_versions.get(vehicle_id, 0) + 1
                )
                self.camera_frame_version += 1

        return callback

    def _decode_latest_camera_frames(self):
        with self.camera_lock:
            pending = [
                (vehicle_id, msg, self.camera_frame_versions[vehicle_id])
                for vehicle_id, msg in self.images.items()
                if self.decoded_camera_versions.get(vehicle_id) !=
                self.camera_frame_versions.get(vehicle_id)
            ]
        for vehicle_id, msg, version in pending:
            try:
                frame = self._decode_image(msg)
                if frame.shape[:2] != (135, 240):
                    frame = cv2.resize(
                        frame,
                        (240, 135),
                        interpolation=cv2.INTER_AREA,
                    )
            except Exception as exc:
                self.get_logger().warn(
                    'Unable to decode camera frame for %s: %s'
                    % (vehicle_id, exc),
                    throttle_duration_sec=5.0,
                )
                continue
            self.camera_frames[vehicle_id] = frame
            self.decoded_camera_versions[vehicle_id] = version

    def _make_scan_callback(self, vehicle_id):
        def callback(msg):
            topic = self._topic('/fleet/uplink/%s/scan' % vehicle_id)
            self.trackers[topic].update(len(msg.ranges) * 4)
            if vehicle_id == self.usv_id:
                self.scan_pub.publish(msg)

        return callback

    def _on_base_radar_scan(self, msg):
        topic = self._topic('/fleet/base/radar/scan')
        self.trackers[topic].update(len(msg.ranges) * 4)

    def _make_odom_callback(self, vehicle_id):
        def callback(_msg):
            topic = self._topic('/fleet/uplink/%s/odom' % vehicle_id)
            self.trackers[topic].update(256)

        return callback

    def _on_state(self, msg):
        self.vehicle_states[msg.vehicle_id] = (msg, time.monotonic())

    def _on_ack(self, msg):
        self.command_status[msg.command_id] = msg
        if self.monitor_only:
            return
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
        if self.monitor_only:
            self.get_logger().info(
                'Monitor-only console observed operator goal; no command sent'
            )
            return
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
        if self.monitor_only:
            self.get_logger().info(
                'Monitor-only console routed operator action without taking '
                'control lease: %s' % action
            )
            return
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
        elif action.startswith('CAPTURE:') or action == 'CANCEL_CAPTURE':
            self.get_logger().info(
                'Capture action routed to cooperative mission: %s' % action
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
            # UAV camera health is published by uav_camera_adapter with the
            # source timestamp, frame, latency, and TF result.
            if tracker.sensor_id == 'uav_camera':
                continue
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
            status.frame_id = ''
            status.measured_rate_hz = float(tracker.rate_hz)
            status.age_seconds = float(min(age, 9999.0))
            status.latency_seconds = 0.0
            status.processing_time_ms = 0.0
            status.point_count = 0
            status.total_messages = tracker.total_messages
            status.total_bytes = tracker.total_bytes
            status.dropped_messages = 0
            status.healthy = age < 2.0 and tracker.rate_hz > 0.2
            status.timed_out = age >= 2.0
            status.tf_target_frame = ''
            status.tf_available = True
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

    @staticmethod
    def _load_title_font():
        for path in (
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        ):
            try:
                return ImageFont.truetype(path, 11)
            except OSError:
                continue
        return ImageFont.load_default()

    def _draw_panel_title(self, panel, title, rate_hz):
        text = '%s  %.1f FPS' % (title, rate_hz)
        try:
            rgb = cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)
            image = PilImage.fromarray(rgb)
            draw = ImageDraw.Draw(image)
            draw.text((6, 3), text, font=self.title_font,
                      fill=(70, 255, 90))
            panel[:, :] = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        except Exception:
            cv2.putText(
                panel,
                text.encode('ascii', errors='ignore').decode('ascii'),
                (6, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.26,
                (70, 255, 90),
                1,
                cv2.LINE_AA,
            )

    def _vehicle_display_name(self, vehicle_id):
        return self.VEHICLE_DISPLAY_NAMES.get(vehicle_id, vehicle_id)

    def _camera_panel(self, frame, title, tracker):
        width, height = 240, 135
        if frame is None:
            panel = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(
                panel,
                'WAITING FOR SENSOR UPLINK',
                (18, 74),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.36,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:
            panel = frame.copy()
        cv2.rectangle(panel, (0, 0), (width, 22), (0, 0, 0), -1)
        self._draw_panel_title(panel, title, tracker.rate_hz)
        if '主感知' in title or title.startswith('PRIMARY PERCEPTION'):
            cv2.rectangle(
                panel, (1, 1), (width - 2, height - 2),
                (0, 210, 255), 3,
            )
        return panel

    def _publish_camera_mosaic(self):
        if self.mosaic_pub.get_subscription_count() == 0:
            return
        with self.camera_lock:
            frame_version = self.camera_frame_version
        if frame_version == self.last_mosaic_version:
            return
        self._decode_latest_camera_frames()
        panels = []
        try:
            for usv_id in self.usv_ids:
                tracker = self.trackers[
                    self._topic('/fleet/uplink/%s/camera' % usv_id)
                ]
                panels.append(
                    self._camera_panel(
                        self.camera_frames.get(usv_id),
                        '%s 前视相机' % self._vehicle_display_name(usv_id),
                        tracker,
                    )
                )
            for uav_id in self.uav_ids:
                tracker = self.trackers[
                    self._topic(
                        '/fleet/uplink/%s/camera/image_raw' % uav_id
                    )
                ]
                title = '%s 下视相机' % self._vehicle_display_name(uav_id)
                if uav_id == 'uav_01':
                    title = '%s 主感知相机（PX4）' % (
                        self._vehicle_display_name(uav_id)
                    )
                panels.append(
                    self._camera_panel(
                        self.camera_frames.get(uav_id),
                        title,
                        tracker,
                    )
                )
            if not panels:
                return
            columns = 4
            while len(panels) % columns:
                panels.append(np.zeros_like(panels[0]))
            rows = [
                np.hstack(panels[index:index + columns])
                for index in range(0, len(panels), columns)
            ]
            mosaic = np.vstack(rows)
        except Exception as exc:
            self.get_logger().warn(
                'Unable to build base camera mosaic: %s' % exc,
                throttle_duration_sec=5.0,
            )
            return
        msg = Image()
        with self.camera_lock:
            primary_frame = self.images.get(self.uav_id)
        msg.header.stamp = (
            primary_frame.header.stamp
            if primary_frame is not None
            else self.get_clock().now().to_msg()
        )
        msg.header.frame_id = 'map'
        msg.height = mosaic.shape[0]
        msg.width = mosaic.shape[1]
        msg.encoding = 'bgr8'
        msg.step = mosaic.shape[1] * 3
        msg.data = mosaic.tobytes()
        self.mosaic_pub.publish(msg)
        self.last_mosaic_version = frame_version

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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
