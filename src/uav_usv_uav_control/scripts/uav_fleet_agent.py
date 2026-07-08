#!/usr/bin/env python3
import math
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from pymavlink import mavutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import VehicleState


class UavFleetAgent(Node):
    """UAV edge agent: camera uplink and lease-protected PX4 offboard tasks."""

    def __init__(self):
        super().__init__('uav_fleet_agent')
        self.declare_parameter('vehicle_id', 'uav_01')
        self.declare_parameter('drone_name', 'x500_mono_cam_down_0')
        self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14540')
        self.declare_parameter('camera_topic', '/uav/down_camera/image')
        self.declare_parameter('pose_topic', '/world/default/pose/info')
        self.declare_parameter(
            'deck_release_topic', '/model/x500_0/release_from_deck'
        )
        self.declare_parameter('takeoff_speed', 1.2)
        self.declare_parameter('navigate_speed', 3.0)

        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.drone_name = self.get_parameter('drone_name').value
        self.mavlink_url = self.get_parameter('mavlink_url').value
        self.takeoff_speed = float(self.get_parameter('takeoff_speed').value)
        self.navigate_speed = float(
            self.get_parameter('navigate_speed').value
        )
        prefix = '/fleet/uplink/%s' % self.vehicle_id

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(
            VehicleState, '/fleet/state', sensor_qos
        )
        self.ack_pub = self.create_publisher(
            CommandAck, '/fleet/command_ack', 20
        )
        self.camera_uplink_pub = self.create_publisher(
            Image, prefix + '/camera', sensor_qos
        )
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.camera_uplink_pub.publish,
            sensor_qos,
        )
        self.create_subscription(
            ControlLease, '/fleet/control_lease', self._on_lease, lease_qos
        )
        self.create_subscription(
            FleetCommand, '/fleet/command', self._on_command, 20
        )

        self.gz_node = GzTransportNode()
        self.pose_topic = self.get_parameter('pose_topic').value
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)
        self.release_pub = self.gz_node.advertise(
            self.get_parameter('deck_release_topic').value,
            Boolean,
        )

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.lease = None
        self.pending_command = None
        self.drone_pose = None
        self.local_position = None
        self.heartbeat = None
        self.mav = None
        self.target_system = 1
        self.target_component = 1
        self.position_setpoint = None
        self.operation = None
        self.operation_goal = None
        self.operation_start_distance = 1.0
        self.operation_phase = ''
        self.phase_started = 0.0
        self.active_command_id = ''
        self.status_text = 'connecting to PX4'
        self.offboard_active = False
        self.last_gcs_heartbeat = 0.0
        self.last_update = time.monotonic()

        self.worker = threading.Thread(target=self._mavlink_loop, daemon=True)
        self.worker.start()
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info(
            'UAV fleet agent %s ready; uplink=%s/*'
            % (self.vehicle_id, prefix)
        )

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=3.0)
        self.gz_node.unsubscribe(self.pose_topic)
        super().destroy_node()

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lease_is_valid(self, lease_id):
        with self.lock:
            lease = self.lease
        return (
            lease is not None
            and not lease.revoked
            and lease.lease_id == lease_id
            and self._stamp_seconds(lease.valid_until) > self._now_seconds()
        )

    def _on_pose_v(self, msg):
        for pose in msg.pose:
            if pose.name == self.drone_name:
                with self.lock:
                    self.drone_pose = pose
                return

    def _on_lease(self, msg):
        if msg.vehicle_id in (self.vehicle_id, '*'):
            with self.lock:
                self.lease = msg

    def _ack(self, command_id, status, message, progress=0.0):
        ack = CommandAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.command_id = command_id
        ack.vehicle_id = self.vehicle_id
        ack.status = status
        ack.progress = float(progress)
        ack.message = message
        self.ack_pub.publish(ack)

    def _on_command(self, msg):
        if msg.vehicle_id not in (self.vehicle_id, '*'):
            return
        if self._stamp_seconds(msg.expires_at) <= self._now_seconds():
            self._ack(
                msg.command_id,
                CommandAck.STATUS_REJECTED,
                'command expired',
            )
            return
        if (
            msg.command_type != FleetCommand.COMMAND_EMERGENCY_STOP
            and not self._lease_is_valid(msg.lease_id)
        ):
            self._ack(
                msg.command_id,
                CommandAck.STATUS_REJECTED,
                'invalid or expired control lease',
            )
            return
        with self.lock:
            self.pending_command = msg
        self._ack(
            msg.command_id,
            CommandAck.STATUS_ACCEPTED,
            'UAV command accepted',
        )

    def _publish_state(self):
        with self.lock:
            pose = self.drone_pose
            heartbeat = self.heartbeat
            command_id = self.active_command_id
            status_text = self.status_text
            connected = self.mav is not None and self.local_position is not None

        state = VehicleState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.vehicle_id = self.vehicle_id
        state.vehicle_type = VehicleState.TYPE_UAV
        state.online = connected
        state.armed = bool(
            heartbeat is not None
            and heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        state.mode = 'OFFBOARD' if self.offboard_active else 'PX4'
        if pose is not None:
            state.pose.position.x = pose.position.x
            state.pose.position.y = pose.position.y
            state.pose.position.z = pose.position.z
            state.pose.orientation.x = pose.orientation.x
            state.pose.orientation.y = pose.orientation.y
            state.pose.orientation.z = pose.orientation.z
            state.pose.orientation.w = pose.orientation.w
        state.battery_percent = -1.0
        state.active_command_id = command_id
        state.status_text = status_text
        self.state_pub.publish(state)

    def _mavlink_loop(self):
        try:
            self.mav = mavutil.mavlink_connection(
                self.mavlink_url,
                autoreconnect=True,
                source_system=253,
                source_component=0,
            )
            heartbeat = self.mav.wait_heartbeat(timeout=120.0)
            if heartbeat is None:
                raise RuntimeError('PX4 heartbeat timeout')
            self.target_system = self.mav.target_system
            self.target_component = self.mav.target_component
            with self.lock:
                self.status_text = 'PX4 connected; waiting for base command'

            while not self.stop_event.is_set() and rclpy.ok():
                now = time.monotonic()
                self._receive_mavlink()
                self._send_gcs_heartbeat(now)
                self._consume_pending_command(now)
                self._update_operation(now)
                if self.offboard_active and self.position_setpoint is not None:
                    self._send_setpoint(*self.position_setpoint)
                time.sleep(0.02)
        except Exception as exc:
            with self.lock:
                self.status_text = 'PX4 agent error: %s' % exc
            self.get_logger().error(str(exc))

    def _receive_mavlink(self):
        msg = self.mav.recv_match(blocking=False)
        while msg is not None:
            msg_type = msg.get_type()
            with self.lock:
                if msg_type == 'LOCAL_POSITION_NED':
                    self.local_position = msg
                elif msg_type == 'HEARTBEAT':
                    self.heartbeat = msg
            msg = self.mav.recv_match(blocking=False)

    def _send_gcs_heartbeat(self, now):
        if now - self.last_gcs_heartbeat < 1.0:
            return
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        self.last_gcs_heartbeat = now

    def _consume_pending_command(self, now):
        with self.lock:
            command = self.pending_command
            self.pending_command = None
        if command is None:
            return

        self.active_command_id = command.command_id
        if command.command_type == FleetCommand.COMMAND_TAKEOFF:
            altitude = (
                float(command.parameters[0])
                if command.parameters
                else 12.0
            )
            self.operation = 'takeoff'
            self.operation_goal = altitude
            self.operation_phase = 'wait_pose'
            self.phase_started = now
            self.status_text = 'preparing PX4 offboard takeoff'
        elif command.command_type == FleetCommand.COMMAND_NAVIGATE:
            self.operation = 'navigate'
            self.operation_goal = (
                command.target_pose.position.x,
                command.target_pose.position.y,
                command.target_pose.position.z,
            )
            self.operation_phase = 'moving'
            self.phase_started = now
            self.status_text = 'flying to base-station target'
            self._ack(
                command.command_id,
                CommandAck.STATUS_EXECUTING,
                self.status_text,
                0.05,
            )
        elif command.command_type in (
            FleetCommand.COMMAND_HOLD,
            FleetCommand.COMMAND_EMERGENCY_STOP,
        ):
            if self.local_position is not None:
                self.position_setpoint = [
                    self.local_position.x,
                    self.local_position.y,
                    self.local_position.z,
                ]
            self.operation = 'hold'
            self.status_text = 'holding current position'
            self._ack(
                command.command_id,
                CommandAck.STATUS_SUCCEEDED,
                self.status_text,
                1.0,
            )
            self.active_command_id = ''
        else:
            self._ack(
                command.command_id,
                CommandAck.STATUS_REJECTED,
                'command is not supported by UAV agent',
            )
            self.active_command_id = ''

    def _update_operation(self, now):
        if self.operation == 'takeoff':
            self._update_takeoff(now)
        elif self.operation == 'navigate':
            self._update_navigation(now)

    def _update_takeoff(self, now):
        if self.local_position is None or self.drone_pose is None:
            return
        if self.operation_phase == 'wait_pose':
            self.position_setpoint = [
                self.local_position.x,
                self.local_position.y,
                self.local_position.z,
            ]
            self.operation_phase = 'prestream'
            self.phase_started = now
            self._ack(
                self.active_command_id,
                CommandAck.STATUS_EXECUTING,
                'streaming initial PX4 setpoints',
                0.1,
            )
            return
        if self.operation_phase == 'prestream':
            if now - self.phase_started < 3.0:
                self._send_setpoint(*self.position_setpoint)
                return
            base_mode, custom_mode, sub_mode = mavutil.px4_map['OFFBOARD']
            self.mav.set_mode(base_mode, custom_mode, sub_mode)
            self.mav.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            )
            self.offboard_active = True
            self.operation_phase = 'spool'
            self.phase_started = now
            self.position_setpoint[2] -= 0.8
            self.status_text = 'armed; building lift on shoreline platform'
            return
        if self.operation_phase == 'spool':
            if now - self.phase_started < 2.0:
                return
            release = Boolean()
            release.data = True
            self.release_pub.publish(release)
            self.operation_goal = (
                self.position_setpoint[0],
                self.position_setpoint[1],
                self.local_position.z - float(self.operation_goal),
            )
            self.operation_phase = 'climb'
            self.phase_started = now
            self.status_text = 'taking off under base-station command'
            return
        if self.operation_phase == 'climb':
            self.position_setpoint = self._move_toward(
                self.position_setpoint,
                self.operation_goal,
                self.takeoff_speed,
                now,
            )
            if self.local_position.z <= self.operation_goal[2] + 0.7:
                command_id = self.active_command_id
                self.operation = 'hold'
                self.active_command_id = ''
                self.status_text = 'takeoff completed; holding'
                self._ack(
                    command_id,
                    CommandAck.STATUS_SUCCEEDED,
                    self.status_text,
                    1.0,
                )

    def _update_navigation(self, now):
        if self.local_position is None or self.drone_pose is None:
            return
        goal = self._gazebo_to_local(*self.operation_goal)
        if self.position_setpoint is None:
            self.position_setpoint = [
                self.local_position.x,
                self.local_position.y,
                self.local_position.z,
            ]
        self.position_setpoint = self._move_toward(
            self.position_setpoint,
            goal,
            self.navigate_speed,
            now,
        )
        distance = math.sqrt(
            (self.drone_pose.position.x - self.operation_goal[0]) ** 2
            + (self.drone_pose.position.y - self.operation_goal[1]) ** 2
            + (self.drone_pose.position.z - self.operation_goal[2]) ** 2
        )
        if distance < 2.0:
            command_id = self.active_command_id
            self.operation = 'hold'
            self.active_command_id = ''
            self.status_text = 'navigation target reached; holding'
            self._ack(
                command_id,
                CommandAck.STATUS_SUCCEEDED,
                self.status_text,
                1.0,
            )

    def _move_toward(self, start, goal, speed, now):
        dt = max(0.001, min(0.1, now - self.last_update))
        self.last_update = now
        delta = [goal[index] - start[index] for index in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        step = speed * dt
        if distance <= step:
            return list(goal)
        scale = step / max(distance, 1e-6)
        return [
            start[index] + delta[index] * scale
            for index in range(3)
        ]

    def _gazebo_to_local(self, x_gz, y_gz, z_up):
        return (
            self.local_position.x + y_gz - self.drone_pose.position.y,
            self.local_position.y + x_gz - self.drone_pose.position.x,
            self.local_position.z - z_up + self.drone_pose.position.z,
        )

    def _send_setpoint(self, x, y, z_down, yaw=0.0):
        mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self.mav.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            float(x),
            float(y),
            float(z_down),
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            float(yaw),
            0.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = UavFleetAgent()
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
