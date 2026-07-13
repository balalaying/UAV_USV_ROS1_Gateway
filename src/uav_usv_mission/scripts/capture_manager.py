#!/usr/bin/env python3
"""Generate and dispatch one-UAV/one-USV dynamic capture assignments."""

import math
import time
import uuid

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState


class CaptureManager(Node):
    def __init__(self):
        super().__init__('capture_manager')
        self.declare_parameter('uav_id', 'uav_01')
        self.declare_parameter('usv_id', 'usv_01')
        self.declare_parameter('target_id', 'target_vessel')
        self.declare_parameter('takeoff_altitude', 18.0)
        self.declare_parameter('uav_home_z', 2.1)
        self.declare_parameter('observation_altitude', 22.0)
        self.declare_parameter('observation_offset', 12.0)
        self.declare_parameter('uav_prediction_time', 2.5)
        self.declare_parameter('usv_prediction_time', 9.0)
        self.declare_parameter('command_period', 5.0)
        self.declare_parameter('command_move_threshold', 3.0)

        self.uav_id = str(self.get_parameter('uav_id').value)
        self.usv_id = str(self.get_parameter('usv_id').value)
        self.target_id = str(self.get_parameter('target_id').value)
        self.takeoff_altitude = float(
            self.get_parameter('takeoff_altitude').value
        )
        self.uav_home_z = float(self.get_parameter('uav_home_z').value)
        self.observation_altitude = float(
            self.get_parameter('observation_altitude').value
        )
        self.observation_offset = float(
            self.get_parameter('observation_offset').value
        )
        self.uav_prediction_time = float(
            self.get_parameter('uav_prediction_time').value
        )
        self.usv_prediction_time = float(
            self.get_parameter('usv_prediction_time').value
        )
        self.command_period = float(
            self.get_parameter('command_period').value
        )
        self.command_move_threshold = float(
            self.get_parameter('command_move_threshold').value
        )

        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.lease_pub = self.create_publisher(
            ControlLease, '/fleet/control_lease', lease_qos
        )
        self.command_pub = self.create_publisher(
            FleetCommand, '/fleet/command', 20
        )
        self.uav_point_pub = self.create_publisher(
            PoseStamped, '/capture/uav_observation_point', 10
        )
        self.usv_point_pub = self.create_publisher(
            PoseStamped, '/capture/usv_intercept_point', 10
        )
        self.status_pub = self.create_publisher(
            String, '/capture/status', 10
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
            self._on_vehicle_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CommandAck, '/fleet/command_ack', self._on_ack, 20
        )

        self.lease_id = 'capture-' + uuid.uuid4().hex[:12]
        self.target = None
        self.vehicle_states = {}
        self.phase = 'WAITING_FOR_LINKS'
        self.takeoff_command_id = ''
        self.last_takeoff_attempt = 0.0
        self.last_command_time = 0.0
        self.last_points = {'uav': None, 'usv': None}
        self.latest_acks = {}
        self.create_timer(0.5, self._publish_lease)
        self.create_timer(0.2, self._update)
        self.get_logger().info(
            'Capture manager ready: %s + %s -> %s'
            % (self.uav_id, self.usv_id, self.target_id)
        )

    def _on_targets(self, msg):
        self.target = next(
            (obj for obj in msg.objects if obj.track_id == self.target_id),
            None,
        )

    def _on_vehicle_state(self, msg):
        if msg.vehicle_id in (self.uav_id, self.usv_id):
            self.vehicle_states[msg.vehicle_id] = msg

    def _on_ack(self, msg):
        if msg.vehicle_id not in (self.uav_id, self.usv_id):
            return
        self.latest_acks[msg.command_id] = msg
        self.get_logger().info(
            'ACK %s %s status=%d: %s'
            % (msg.vehicle_id, msg.command_id, msg.status, msg.message)
        )
        if msg.command_id != self.takeoff_command_id:
            return
        if msg.status == CommandAck.STATUS_SUCCEEDED:
            self.phase = 'CAPTURE_ACTIVE'
        elif msg.status in (
            CommandAck.STATUS_REJECTED,
            CommandAck.STATUS_FAILED,
            CommandAck.STATUS_CANCELED,
        ):
            self.takeoff_command_id = ''
            self.phase = 'WAITING_FOR_PX4_READY'

    def _publish_lease(self):
        msg = ControlLease()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vehicle_id = '*'
        msg.lease_id = self.lease_id
        msg.owner_id = 'capture_manager'
        msg.priority = 100
        valid_until = self.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
        msg.valid_until = valid_until.to_msg()
        msg.revoked = False
        self.lease_pub.publish(msg)

    def _new_command(self, vehicle_id, command_type, lifetime=8.0):
        msg = FleetCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.command_id = '%s-%s' % (vehicle_id, uuid.uuid4().hex[:10])
        msg.vehicle_id = vehicle_id
        msg.lease_id = self.lease_id
        msg.command_type = command_type
        msg.priority = 100
        expires = self.get_clock().now() + rclpy.duration.Duration(seconds=lifetime)
        msg.expires_at = expires.to_msg()
        msg.target_pose.orientation.w = 1.0
        return msg

    def _send_takeoff(self):
        msg = self._new_command(
            self.uav_id, FleetCommand.COMMAND_TAKEOFF, lifetime=15.0
        )
        msg.parameters = [self.takeoff_altitude]
        self.takeoff_command_id = msg.command_id
        self.last_takeoff_attempt = time.monotonic()
        self.phase = 'TAKEOFF_REQUESTED'
        self.command_pub.publish(msg)
        self.get_logger().info('Sent PX4 takeoff command ' + msg.command_id)

    def _capture_points(self):
        pose = self.target.pose.pose.position
        velocity = self.target.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        if speed > 0.05:
            unit_x = velocity.x / speed
            unit_y = velocity.y / speed
        else:
            unit_x, unit_y = 1.0, 0.0
        perpendicular_x, perpendicular_y = -unit_y, unit_x

        uav = PoseStamped()
        uav.header.stamp = self.get_clock().now().to_msg()
        uav.header.frame_id = 'map'
        uav.pose.position.x = (
            pose.x + velocity.x * self.uav_prediction_time
            + perpendicular_x * self.observation_offset
        )
        uav.pose.position.y = (
            pose.y + velocity.y * self.uav_prediction_time
            + perpendicular_y * self.observation_offset
        )
        uav.pose.position.z = self.observation_altitude
        uav.pose.orientation.w = 1.0

        usv = PoseStamped()
        usv.header = uav.header
        usv.pose.position.x = pose.x + velocity.x * self.usv_prediction_time
        usv.pose.position.y = pose.y + velocity.y * self.usv_prediction_time
        usv.pose.position.z = 0.55
        usv.pose.orientation.w = 1.0
        return uav, usv

    def _point_moved(self, name, point):
        previous = self.last_points[name]
        if previous is None:
            return True
        return math.hypot(
            point.pose.position.x - previous[0],
            point.pose.position.y - previous[1],
        ) >= self.command_move_threshold

    def _send_navigation(self, vehicle_id, point):
        msg = self._new_command(
            vehicle_id, FleetCommand.COMMAND_NAVIGATE
        )
        msg.target_pose = point.pose
        self.command_pub.publish(msg)

    def _update(self):
        uav_state = self.vehicle_states.get(self.uav_id)
        usv_state = self.vehicle_states.get(self.usv_id)
        links_ready = bool(
            self.target is not None
            and uav_state is not None and uav_state.online
            and usv_state is not None and usv_state.online
        )
        if not links_ready:
            self.phase = 'WAITING_FOR_LINKS'
            self._publish_status('waiting for target, PX4 DDS, and USV odometry')
            return

        if (
            uav_state.armed
            and uav_state.mode == 'PX4/HOLD'
            and uav_state.pose.position.z >= (
                self.uav_home_z + self.takeoff_altitude - 1.0
            )
        ):
            self.phase = 'CAPTURE_ACTIVE'

        if self.phase in ('WAITING_FOR_LINKS', 'WAITING_FOR_PX4_READY'):
            if time.monotonic() - self.last_takeoff_attempt >= 2.0:
                self._send_takeoff()
            return
        if self.phase == 'TAKEOFF_REQUESTED':
            if time.monotonic() - self.last_takeoff_attempt > 20.0:
                self.takeoff_command_id = ''
                self.phase = 'WAITING_FOR_PX4_READY'
            self._publish_status('PX4 takeoff in progress')
            return
        if self.phase != 'CAPTURE_ACTIVE':
            return

        uav_point, usv_point = self._capture_points()
        self.uav_point_pub.publish(uav_point)
        self.usv_point_pub.publish(usv_point)
        now = time.monotonic()
        period_elapsed = now - self.last_command_time >= self.command_period
        if period_elapsed:
            if self._point_moved('uav', uav_point):
                self._send_navigation(self.uav_id, uav_point)
                self.last_points['uav'] = (
                    uav_point.pose.position.x, uav_point.pose.position.y
                )
            if self._point_moved('usv', usv_point):
                self._send_navigation(self.usv_id, usv_point)
                self.last_points['usv'] = (
                    usv_point.pose.position.x, usv_point.pose.position.y
                )
            self.last_command_time = now
        self._publish_status(
            'capture active: target predicted and assignments dispatched through agents'
        )

    def _publish_status(self, text):
        msg = String()
        msg.data = self.phase + ': ' + text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureManager()
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
