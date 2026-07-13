#!/usr/bin/env python3
"""Translate fleet commands into PX4 Offboard setpoints over uXRCE-DDS."""

import math
import time

import rclpy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import VehicleState


class UavDdsFleetAgent(Node):
    """Single-vehicle edge agent for a PX4 SITL instance."""

    def __init__(self):
        super().__init__('uav_dds_fleet_agent')
        self.declare_parameter('vehicle_id', 'uav_01')
        self.declare_parameter('px4_namespace', '/uav_01')
        self.declare_parameter('px4_system_id', 1)
        self.declare_parameter('home_x', -30.0)
        self.declare_parameter('home_y', -24.0)
        self.declare_parameter('home_z', 2.1)
        self.declare_parameter('takeoff_tolerance', 0.6)
        self.declare_parameter('navigation_tolerance', 1.8)
        self.declare_parameter('prestream_seconds', 1.5)
        self.declare_parameter('arming_timeout', 10.0)

        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.namespace = str(self.get_parameter('px4_namespace').value).rstrip('/')
        self.system_id = int(self.get_parameter('px4_system_id').value)
        self.home = (
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_z').value),
        )
        self.takeoff_tolerance = float(
            self.get_parameter('takeoff_tolerance').value
        )
        self.navigation_tolerance = float(
            self.get_parameter('navigation_tolerance').value
        )
        self.prestream_seconds = float(
            self.get_parameter('prestream_seconds').value
        )
        self.arming_timeout = float(
            self.get_parameter('arming_timeout').value
        )

        px4_qos = QoSProfile(depth=1)
        px4_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        px4_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            self.namespace + '/fmu/in/offboard_control_mode',
            px4_qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            self.namespace + '/fmu/in/trajectory_setpoint',
            px4_qos,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            self.namespace + '/fmu/in/vehicle_command',
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.namespace + '/fmu/out/vehicle_local_position_v1',
            self._on_local_position,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            self.namespace + '/fmu/out/vehicle_status_v4',
            self._on_vehicle_status,
            px4_qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            self.namespace + '/fmu/out/vehicle_command_ack_v1',
            self._on_px4_command_ack,
            px4_qos,
        )
        self.create_subscription(
            ControlLease,
            '/fleet/control_lease',
            self._on_lease,
            lease_qos,
        )
        self.create_subscription(
            FleetCommand,
            '/fleet/command',
            self._on_fleet_command,
            20,
        )
        self.state_pub = self.create_publisher(VehicleState, '/fleet/state', 20)
        self.ack_pub = self.create_publisher(
            CommandAck, '/fleet/command_ack', 20
        )

        self.local = None
        self.status = None
        self.lease = None
        self.setpoint = None
        self.operation = 'waiting_dds'
        self.command_id = ''
        self.operation_started = 0.0
        self.last_arm_request = 0.0
        self.last_progress_ack = 0.0
        self.arm_rejected = False
        self.create_timer(0.05, self._update_offboard)
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info(
            'PX4 DDS agent ready: vehicle=%s namespace=%s system_id=%d'
            % (self.vehicle_id, self.namespace, self.system_id)
        )

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def _on_local_position(self, msg):
        self.local = msg
        if self.setpoint is None and msg.xy_valid and msg.z_valid:
            self.setpoint = [float(msg.x), float(msg.y), float(msg.z)]
            self.operation = 'idle'

    def _on_vehicle_status(self, msg):
        self.status = msg

    def _on_lease(self, msg):
        if msg.vehicle_id in (self.vehicle_id, '*'):
            self.lease = msg

    def _lease_is_valid(self, lease_id):
        return (
            self.lease is not None
            and not self.lease.revoked
            and self.lease.lease_id == lease_id
            and self._stamp_seconds(self.lease.valid_until) > self._now_seconds()
        )

    def _publish_ack(self, command_id, status, message, progress=0.0):
        if not command_id:
            return
        msg = CommandAck()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_id = command_id
        msg.vehicle_id = self.vehicle_id
        msg.status = status
        msg.progress = float(progress)
        msg.message = message
        self.ack_pub.publish(msg)

    def _reject(self, msg, reason):
        self._publish_ack(
            msg.command_id, CommandAck.STATUS_REJECTED, reason
        )

    def _on_fleet_command(self, msg):
        if msg.vehicle_id not in (self.vehicle_id, '*'):
            return
        if self._stamp_seconds(msg.expires_at) <= self._now_seconds():
            self._reject(msg, 'command expired')
            return
        if (
            msg.command_type != FleetCommand.COMMAND_EMERGENCY_STOP
            and not self._lease_is_valid(msg.lease_id)
        ):
            self._reject(msg, 'invalid or expired control lease')
            return
        if self.local is None or not self.local.xy_valid or not self.local.z_valid:
            self._reject(msg, 'PX4 local position is not valid')
            return

        if msg.command_type == FleetCommand.COMMAND_TAKEOFF:
            if self.status is None or not self.status.pre_flight_checks_pass:
                self._reject(msg, 'PX4 preflight checks have not passed')
                return
            altitude = float(msg.parameters[0]) if msg.parameters else 15.0
            self.setpoint = [self.local.x, self.local.y, self.local.z]
            self.operation = 'prestream_takeoff'
            self.operation_started = time.monotonic()
            self.last_arm_request = 0.0
            self.arm_rejected = False
            self.command_id = msg.command_id
            self._publish_ack(
                msg.command_id,
                CommandAck.STATUS_ACCEPTED,
                'takeoff accepted; prestreaming Offboard setpoints',
                0.05,
            )
            self.takeoff_target_z = float(self.local.z) - max(1.0, altitude)
        elif msg.command_type == FleetCommand.COMMAND_NAVIGATE:
            if self.status is None or self.status.arming_state != (
                VehicleStatus.ARMING_STATE_ARMED
            ):
                self._reject(msg, 'PX4 must be armed before navigation')
                return
            self.setpoint = self._world_to_ned(
                msg.target_pose.position.x,
                msg.target_pose.position.y,
                msg.target_pose.position.z,
            )
            self.operation = 'navigate'
            self.operation_started = time.monotonic()
            self.command_id = msg.command_id
            self._publish_ack(
                msg.command_id,
                CommandAck.STATUS_ACCEPTED,
                'world target converted to PX4 local NED setpoint',
                0.05,
            )
        elif msg.command_type in (
            FleetCommand.COMMAND_HOLD,
            FleetCommand.COMMAND_EMERGENCY_STOP,
        ):
            self.setpoint = [self.local.x, self.local.y, self.local.z]
            self.operation = 'hold'
            self.command_id = ''
            self._publish_ack(
                msg.command_id,
                CommandAck.STATUS_SUCCEEDED,
                'PX4 position hold active',
                1.0,
            )
        else:
            self._reject(msg, 'command is not supported by PX4 agent')

    def _world_to_ned(self, world_x, world_y, world_z):
        return [
            float(world_y) - self.home[1],
            float(world_x) - self.home[0],
            -(float(world_z) - self.home[2]),
        ]

    def _world_pose(self):
        if self.local is None:
            return self.home
        return (
            self.home[0] + float(self.local.y),
            self.home[1] + float(self.local.x),
            self.home[2] - float(self.local.z),
        )

    def _send_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.command = command
        for name, value in params.items():
            setattr(msg, name, float(value))
        msg.target_system = self.system_id
        msg.target_component = 1
        msg.source_system = 250
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _on_px4_command_ack(self, msg):
        if msg.command not in (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        ):
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return
        if (
            msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            and self.operation in ('prestream_takeoff', 'arming_takeoff')
        ):
            self.arm_rejected = True
            self.get_logger().warn(
                'PX4 rejected arm request: result=%d param2=%d'
                % (msg.result, msg.result_param2)
            )

    def _publish_offboard_setpoint(self):
        timestamp = self._timestamp_us()
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self.offboard_pub.publish(mode)

        trajectory = TrajectorySetpoint()
        trajectory.timestamp = timestamp
        trajectory.position = [float(value) for value in self.setpoint]
        trajectory.velocity = [math.nan, math.nan, math.nan]
        trajectory.acceleration = [math.nan, math.nan, math.nan]
        trajectory.jerk = [math.nan, math.nan, math.nan]
        trajectory.yaw = math.nan
        trajectory.yawspeed = math.nan
        self.setpoint_pub.publish(trajectory)

    def _update_offboard(self):
        if self.setpoint is None:
            return
        self._publish_offboard_setpoint()
        now = time.monotonic()

        if self.operation == 'prestream_takeoff':
            if now - self.operation_started < self.prestream_seconds:
                return
            self.operation = 'arming_takeoff'

        if self.operation == 'arming_takeoff':
            if self.status is not None and (
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.setpoint[2] = self.takeoff_target_z
                self.operation = 'takeoff'
                self.operation_started = now
                self._publish_ack(
                    self.command_id,
                    CommandAck.STATUS_EXECUTING,
                    'PX4 armed and entered OFFBOARD; climbing',
                    0.35,
                )
                return
            if now - self.operation_started > self.arming_timeout:
                reason = 'PX4 arm/OFFBOARD timeout'
                if self.arm_rejected:
                    reason += ' after PX4 rejected arming'
                command_id = self.command_id
                self.command_id = ''
                self.operation = 'hold'
                self._publish_ack(
                    command_id, CommandAck.STATUS_FAILED, reason
                )
                return
            if now - self.last_arm_request >= 0.5:
                self.last_arm_request = now
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )

        if self.operation == 'takeoff' and self.local is not None:
            if abs(float(self.local.z) - self.setpoint[2]) <= self.takeoff_tolerance:
                command_id = self.command_id
                self.command_id = ''
                self.operation = 'hold'
                self._publish_ack(
                    command_id,
                    CommandAck.STATUS_SUCCEEDED,
                    'PX4 takeoff completed',
                    1.0,
                )
        elif self.operation == 'navigate' and self.local is not None:
            distance = math.sqrt(sum(
                (float(current) - float(target)) ** 2
                for current, target in zip(
                    (self.local.x, self.local.y, self.local.z), self.setpoint
                )
            ))
            if distance <= self.navigation_tolerance:
                command_id = self.command_id
                self.command_id = ''
                self.operation = 'hold'
                self._publish_ack(
                    command_id,
                    CommandAck.STATUS_SUCCEEDED,
                    'PX4 navigation target reached',
                    1.0,
                )
            elif now - self.last_progress_ack >= 1.0:
                self.last_progress_ack = now
                self._publish_ack(
                    self.command_id,
                    CommandAck.STATUS_EXECUTING,
                    'PX4 flying to dynamic observation point',
                    max(0.05, min(0.95, 1.0 / (1.0 + distance))),
                )

    def _publish_state(self):
        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.vehicle_id = self.vehicle_id
        msg.vehicle_type = VehicleState.TYPE_UAV
        msg.online = self.local is not None and self.status is not None
        msg.armed = bool(
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        msg.mode = 'PX4/' + self.operation.upper()
        x, y, z = self._world_pose()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        if self.local is not None:
            msg.twist.linear.x = float(self.local.vy)
            msg.twist.linear.y = float(self.local.vx)
            msg.twist.linear.z = -float(self.local.vz)
        msg.battery_percent = -1.0
        msg.active_command_id = self.command_id
        if self.status is None:
            msg.status_text = 'waiting for PX4 uXRCE-DDS'
        elif not self.status.pre_flight_checks_pass:
            msg.status_text = 'PX4 DDS online; preflight checks pending'
        else:
            msg.status_text = 'PX4 DDS online; preflight checks passed'
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UavDdsFleetAgent()
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
