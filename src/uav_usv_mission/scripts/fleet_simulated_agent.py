#!/usr/bin/env python3
import math
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from nav_msgs.msg import Odometry
import numpy as np
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import VehicleState


class SimVehicle:
    def __init__(self, vehicle_id, vehicle_type, model_name, x, y, z):
        self.vehicle_id = vehicle_id
        self.vehicle_type = vehicle_type
        self.model_name = model_name
        self.default_pose = (float(x), float(y), float(z))
        self.pose = None
        self.last_pose_time = 0.0
        self.lease = None
        self.active_command_id = ''
        self.status_text = 'simulated fleet member online'
        self.mode = 'SIM-UAV' if vehicle_type == 'uav' else 'SIM-USV'
        self.target = None


class FleetSimulatedAgent(Node):
    """Publishes fleet data for visual auxiliary UAV/USV members."""

    def __init__(self):
        super().__init__('fleet_simulated_agent')
        self.declare_parameter('topic_namespace', '')
        self.declare_parameter(
            'vehicles',
            (
                'usv_02:usv:landing_boat_02:-18:15:0.6;'
                'uav_02:uav:fleet_uav_02:-18:15:9.5;'
                'usv_03:usv:landing_boat_03:-22:-16:0.6;'
                'uav_03:uav:fleet_uav_03:-22:-16:10.5'
            ),
        )
        self.declare_parameter('pose_topic', '/world/default/pose/info')
        self.declare_parameter('image_rate', 5.0)
        self.declare_parameter('scan_rate', 5.0)
        self.declare_parameter('publish_images', True)

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.vehicles = self._parse_vehicles(
            self.get_parameter('vehicles').value
        )
        namespace = str(
            self.get_parameter('topic_namespace').value
        ).strip('/')
        self._topic = (
            (lambda name: '/%s%s' % (namespace, name))
            if namespace
            else (lambda name: name)
        )
        self.state_pub = self.create_publisher(
            VehicleState, self._topic('/fleet/state'), sensor_qos
        )
        self.ack_pub = self.create_publisher(
            CommandAck, self._topic('/fleet/command_ack'), 20
        )
        self.image_pubs = {}
        self.scan_pubs = {}
        self.odom_pubs = {}
        for vehicle in self.vehicles.values():
            prefix = self._topic('/fleet/uplink/%s' % vehicle.vehicle_id)
            self.image_pubs[vehicle.vehicle_id] = self.create_publisher(
                Image, prefix + '/camera', sensor_qos
            )
            if vehicle.vehicle_type == 'usv':
                self.scan_pubs[vehicle.vehicle_id] = self.create_publisher(
                    LaserScan, prefix + '/scan', sensor_qos
                )
                self.odom_pubs[vehicle.vehicle_id] = self.create_publisher(
                    Odometry, prefix + '/odom', sensor_qos
                )

        self.create_subscription(
            ControlLease,
            self._topic('/fleet/control_lease'),
            self._on_lease,
            lease_qos,
        )
        self.create_subscription(
            FleetCommand,
            self._topic('/fleet/command'),
            self._on_command,
            20,
        )

        self.lock = threading.Lock()
        self.sensor_callback_group = ReentrantCallbackGroup()
        self.gz_node = GzTransportNode()
        self.pose_topic = self.get_parameter('pose_topic').value
        self.publish_images = bool(self.get_parameter('publish_images').value)
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)

        self.start_time = time.monotonic()
        self.create_timer(0.2, self._publish_states)
        if self.publish_images:
            self.create_timer(
                1.0 / max(0.5, float(self.get_parameter('image_rate').value)),
                self._publish_images,
                callback_group=self.sensor_callback_group,
            )
        self.create_timer(
            1.0 / max(0.5, float(self.get_parameter('scan_rate').value)),
            self._publish_usv_sensors,
            callback_group=self.sensor_callback_group,
        )
        self.get_logger().info(
            'Simulated fleet agent online for: %s'
            % ', '.join(self.vehicles.keys())
        )

    def destroy_node(self):
        self.gz_node.unsubscribe(self.pose_topic)
        super().destroy_node()

    @staticmethod
    def _parse_vehicles(spec):
        vehicles = {}
        for entry in str(spec).split(';'):
            entry = entry.strip()
            if not entry:
                continue
            parts = entry.split(':')
            if len(parts) != 6:
                raise ValueError(
                    'vehicle entry must be id:type:model:x:y:z, got %s'
                    % entry
                )
            vehicle_id, vehicle_type, model_name, x, y, z = parts
            vehicle_type = vehicle_type.lower()
            if vehicle_type not in ('uav', 'usv'):
                raise ValueError('vehicle type must be uav or usv')
            vehicles[vehicle_id] = SimVehicle(
                vehicle_id, vehicle_type, model_name, x, y, z
            )
        return vehicles

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose_v(self, msg):
        now = time.monotonic()
        with self.lock:
            by_model = {v.model_name: v for v in self.vehicles.values()}
            for pose in msg.pose:
                vehicle = by_model.get(pose.name)
                if vehicle is not None:
                    vehicle.pose = pose
                    vehicle.last_pose_time = now

    def _on_lease(self, msg):
        with self.lock:
            for vehicle in self.vehicles.values():
                if msg.vehicle_id in (vehicle.vehicle_id, '*'):
                    vehicle.lease = msg

    def _lease_is_valid(self, vehicle, lease_id):
        return (
            vehicle.lease is not None
            and not vehicle.lease.revoked
            and vehicle.lease.lease_id == lease_id
            and self._stamp_seconds(vehicle.lease.valid_until)
            > self._now_seconds()
        )

    def _on_command(self, msg):
        with self.lock:
            selected = [
                vehicle
                for vehicle in self.vehicles.values()
                if msg.vehicle_id in (vehicle.vehicle_id, '*')
            ]
            for vehicle in selected:
                if msg.command_type != FleetCommand.COMMAND_EMERGENCY_STOP:
                    if not self._lease_is_valid(vehicle, msg.lease_id):
                        self._ack(
                            vehicle.vehicle_id,
                            msg.command_id,
                            CommandAck.STATUS_REJECTED,
                            'invalid or expired control lease',
                        )
                        continue
                vehicle.active_command_id = msg.command_id
                if msg.command_type == FleetCommand.COMMAND_NAVIGATE:
                    vehicle.target = (
                        msg.target_pose.position.x,
                        msg.target_pose.position.y,
                        msg.target_pose.position.z,
                    )
                    vehicle.status_text = 'received cooperative target'
                    self._ack(
                        vehicle.vehicle_id,
                        msg.command_id,
                        CommandAck.STATUS_ACCEPTED,
                        'simulated vehicle accepted target',
                    )
                    self._ack(
                        vehicle.vehicle_id,
                        msg.command_id,
                        CommandAck.STATUS_EXECUTING,
                        vehicle.status_text,
                        0.25,
                    )
                elif msg.command_type == FleetCommand.COMMAND_TAKEOFF:
                    if vehicle.vehicle_type == 'uav':
                        vehicle.status_text = 'simulated UAV airborne'
                        self._ack(
                            vehicle.vehicle_id,
                            msg.command_id,
                            CommandAck.STATUS_SUCCEEDED,
                            vehicle.status_text,
                            1.0,
                        )
                        vehicle.active_command_id = ''
                    else:
                        self._ack(
                            vehicle.vehicle_id,
                            msg.command_id,
                            CommandAck.STATUS_REJECTED,
                            'takeoff ignored by simulated USV',
                        )
                elif msg.command_type == FleetCommand.COMMAND_HOLD:
                    vehicle.status_text = 'holding simulated position'
                    vehicle.target = None
                    self._ack(
                        vehicle.vehicle_id,
                        msg.command_id,
                        CommandAck.STATUS_SUCCEEDED,
                        vehicle.status_text,
                        1.0,
                    )
                    vehicle.active_command_id = ''
                elif msg.command_type == FleetCommand.COMMAND_EMERGENCY_STOP:
                    vehicle.status_text = 'simulated emergency stop'
                    vehicle.target = None
                    self._ack(
                        vehicle.vehicle_id,
                        msg.command_id,
                        CommandAck.STATUS_SUCCEEDED,
                        vehicle.status_text,
                        1.0,
                    )
                    vehicle.active_command_id = ''

    def _ack(self, vehicle_id, command_id, status, message, progress=0.0):
        ack = CommandAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.vehicle_id = vehicle_id
        ack.command_id = command_id
        ack.status = status
        ack.progress = float(progress)
        ack.message = message
        self.ack_pub.publish(ack)

    def _vehicle_pose(self, vehicle):
        if vehicle.pose is not None:
            return (
                vehicle.pose.position.x,
                vehicle.pose.position.y,
                vehicle.pose.position.z,
                vehicle.pose.orientation.x,
                vehicle.pose.orientation.y,
                vehicle.pose.orientation.z,
                vehicle.pose.orientation.w,
            )
        x, y, z = vehicle.default_pose
        return (x, y, z, 0.0, 0.0, 0.0, 1.0)

    def _publish_states(self):
        with self.lock:
            vehicles = list(self.vehicles.values())
        now = time.monotonic()
        for vehicle in vehicles:
            pose = self._vehicle_pose(vehicle)
            state = VehicleState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = 'map'
            state.vehicle_id = vehicle.vehicle_id
            state.vehicle_type = (
                VehicleState.TYPE_UAV
                if vehicle.vehicle_type == 'uav'
                else VehicleState.TYPE_USV
            )
            state.online = (
                vehicle.pose is not None
                or now - self.start_time > 0.5
            )
            state.armed = vehicle.vehicle_type == 'uav'
            state.mode = vehicle.mode
            state.pose.position.x = pose[0]
            state.pose.position.y = pose[1]
            state.pose.position.z = pose[2]
            state.pose.orientation.x = pose[3]
            state.pose.orientation.y = pose[4]
            state.pose.orientation.z = pose[5]
            state.pose.orientation.w = pose[6]
            state.battery_percent = 92.0
            state.active_command_id = vehicle.active_command_id
            state.status_text = vehicle.status_text
            self.state_pub.publish(state)

    def _publish_usv_sensors(self):
        with self.lock:
            vehicles = [
                vehicle
                for vehicle in self.vehicles.values()
                if vehicle.vehicle_type == 'usv'
            ]
        for vehicle in vehicles:
            pose = self._vehicle_pose(vehicle)
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'map'
            odom.child_frame_id = vehicle.vehicle_id + '/base_link'
            odom.pose.pose.position.x = pose[0]
            odom.pose.pose.position.y = pose[1]
            odom.pose.pose.position.z = pose[2]
            odom.pose.pose.orientation.x = pose[3]
            odom.pose.pose.orientation.y = pose[4]
            odom.pose.pose.orientation.z = pose[5]
            odom.pose.pose.orientation.w = pose[6]
            self.odom_pubs[vehicle.vehicle_id].publish(odom)

            scan = LaserScan()
            scan.header.stamp = odom.header.stamp
            scan.header.frame_id = vehicle.vehicle_id + '/front_lidar'
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = math.radians(1.0)
            scan.time_increment = 0.0
            scan.scan_time = 0.2
            scan.range_min = 0.4
            scan.range_max = 55.0
            count = int(
                round((scan.angle_max - scan.angle_min) / scan.angle_increment)
            )
            phase = time.monotonic() * 0.2 + hash(vehicle.vehicle_id) % 7
            ranges = []
            for index in range(count):
                angle = scan.angle_min + index * scan.angle_increment
                value = 35.0 + 7.0 * math.sin(angle * 3.0 + phase)
                if abs(math.sin(angle * 2.0 + phase)) > 0.96:
                    value = 12.0 + 4.0 * math.sin(phase)
                ranges.append(float(max(scan.range_min, min(scan.range_max, value))))
            scan.ranges = ranges
            self.scan_pubs[vehicle.vehicle_id].publish(scan)

    def _publish_images(self):
        with self.lock:
            vehicles = list(self.vehicles.values())
        for vehicle in vehicles:
            image = self._make_camera_image(vehicle)
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = vehicle.vehicle_id + '/camera'
            msg.height = image.shape[0]
            msg.width = image.shape[1]
            msg.encoding = 'bgr8'
            msg.step = image.shape[1] * 3
            msg.data = image.tobytes()
            self.image_pubs[vehicle.vehicle_id].publish(msg)

    def _make_camera_image(self, vehicle):
        width, height = 360, 240
        pose = self._vehicle_pose(vehicle)
        base_color = (
            (80, 130, 210)
            if vehicle.vehicle_id.endswith('02')
            else (40, 140, 230)
        )
        if vehicle.vehicle_type == 'usv':
            base_color = (
                (180, 95, 30)
                if vehicle.vehicle_id.endswith('03')
                else (170, 90, 35)
            )
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:] = (42, 84, 112)
        horizon = 86 if vehicle.vehicle_type == 'uav' else 118
        image[:horizon, :] = (130, 170, 205)
        cv2.rectangle(image, (0, horizon), (width, height), (70, 115, 150), -1)
        for offset in range(-80, 420, 80):
            x0 = int((offset + time.monotonic() * 15) % 440 - 80)
            cv2.line(
                image,
                (x0, horizon + 18),
                (x0 + 160, height),
                (95, 145, 170),
                1,
                cv2.LINE_AA,
            )
        center = (width // 2, height // 2 + 26)
        if vehicle.vehicle_type == 'uav':
            cv2.circle(image, center, 26, base_color, -1, cv2.LINE_AA)
            cv2.line(image, (center[0] - 44, center[1]), (center[0] + 44, center[1]), (25, 25, 25), 5)
            cv2.line(image, (center[0], center[1] - 44), (center[0], center[1] + 44), (25, 25, 25), 5)
        else:
            pts = np.array(
                [
                    (center[0] - 70, center[1] + 28),
                    (center[0] + 74, center[1] + 28),
                    (center[0] + 48, center[1] - 20),
                    (center[0] - 56, center[1] - 20),
                ],
                dtype=np.int32,
            )
            cv2.fillPoly(image, [pts], base_color, cv2.LINE_AA)
            cv2.rectangle(
                image,
                (center[0] - 28, center[1] - 54),
                (center[0] + 32, center[1] - 18),
                (210, 220, 215),
                -1,
            )
        cv2.rectangle(image, (0, 0), (width, 40), (8, 14, 20), -1)
        cv2.putText(
            image,
            '%s  x=%.1f y=%.1f z=%.1f'
            % (vehicle.vehicle_id.upper(), pose[0], pose[1], pose[2]),
            (12, 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (230, 245, 245),
            1,
            cv2.LINE_AA,
        )
        return image


def main(args=None):
    rclpy.init(args=args)
    node = FleetSimulatedAgent()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
