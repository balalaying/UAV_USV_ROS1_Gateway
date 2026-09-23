#!/usr/bin/env python3
"""ROS 1 fleet gateway with legacy JSON telemetry and Protobuf v1 control."""

import json
import math
from pathlib import Path
import socket
import subprocess
import threading
import time
import uuid

from uav_usv_ros1_compat.packages import get_package_share_directory
import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.executors import ExternalShutdownException
from uav_usv_ros1_compat.node import Node
from uav_usv_ros1_compat.qos import DurabilityPolicy
from uav_usv_ros1_compat.qos import QoSProfile
from uav_usv_ros1_compat.qos import ReliabilityPolicy
from uav_usv_ros1_compat.qos import qos_profile_sensor_data
from std_msgs.msg import String
from uav_usv_interfaces.msg import CaptureState
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import SensorStatus
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState

from .fleet_registry import FleetRegistry
from .health_monitor import HealthMonitor
from .http_server import FleetHttpServer
from .message_converter import mission_from_ros
from .message_converter import sensor_from_ros
from .message_converter import target_from_ros
from .message_converter import vehicle_from_ros
from .protocol import ProtocolEncoder
from .protocol_v1 import V1ProtocolEncoder
from . import uav_usv_gateway_v1_pb2 as gateway_v1_pb
from .rate_limiter import LatestValueStore
from .websocket_server import FleetWebSocketServer


class ControlCommandValidationError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


def _required_control_parameter(parameters, name, value_field):
    if name not in parameters:
        raise ControlCommandValidationError(
            'MISSING_PARAMETER',
            'Missing required parameter: %s' % name,
        )
    value = parameters[name]
    actual_field = value.WhichOneof('value')
    if actual_field != value_field:
        raise ControlCommandValidationError(
            'INVALID_PARAMETER_TYPE',
            '%s must use %s.' % (name, value_field),
        )
    return getattr(value, value_field)


def depart_target_from_control(command):
    """Return validated map/Gazebo ENU x/y for a USV_DEPART command."""
    frame_id = _required_control_parameter(
        command.parameters, 'frame_id', 'string_value')
    if frame_id != 'map':
        raise ControlCommandValidationError(
            'INVALID_TARGET_FRAME',
            'USV_DEPART frame_id must equal map.',
        )

    target_x = float(_required_control_parameter(
        command.parameters, 'target_x_m', 'double_value'))
    target_y = float(_required_control_parameter(
        command.parameters, 'target_y_m', 'double_value'))
    if not math.isfinite(target_x):
        raise ControlCommandValidationError(
            'INVALID_TARGET_COORDINATE',
            'USV_DEPART target_x_m must be finite.',
        )
    if not math.isfinite(target_y):
        raise ControlCommandValidationError(
            'INVALID_TARGET_COORDINATE',
            'USV_DEPART target_y_m must be finite.',
        )
    return target_x, target_y


def device_command_specs():
    """Device-command mapping kept separate for validation and regression tests."""
    return {
        'USV_HOLD': {
            'prefix': 'usv_',
            'command_type': FleetCommand.COMMAND_HOLD,
            'parameters': [],
        },
        'USV_STOP': {
            'prefix': 'usv_',
            'command_type': FleetCommand.COMMAND_HOLD,
            'parameters': [],
        },
        'USV_EMERGENCY_STOP': {
            'prefix': 'usv_',
            'command_type': FleetCommand.COMMAND_EMERGENCY_STOP,
            'parameters': [],
        },
        'USV_DEPART': {
            'prefix': 'usv_',
            'command_type': FleetCommand.COMMAND_NAVIGATE,
            'parameters': [],
        },
        'USV_RETURN': {
            'prefix': 'usv_',
            'command_type': FleetCommand.COMMAND_RETURN,
            'parameters': [],
        },
        'UAV_TAKEOFF': {
            'prefix': 'uav_',
            'command_type': FleetCommand.COMMAND_TAKEOFF,
            'parameters': [15.0],
        },
        'UAV_HOVER': {
            'prefix': 'uav_',
            'command_type': FleetCommand.COMMAND_HOLD,
            'parameters': [],
        },
        'UAV_LAND': {
            'prefix': 'uav_',
            'command_type': FleetCommand.COMMAND_LAND,
            'parameters': [0.0],
        },
        'UAV_RETURN': {
            'prefix': 'uav_',
            'command_type': FleetCommand.COMMAND_RETURN,
            'parameters': [],
        },
        'UAV_EMERGENCY_LAND': {
            'prefix': 'uav_',
            'command_type': FleetCommand.COMMAND_EMERGENCY_LAND,
            'parameters': [],
            'emergency': True,
        },
    }


def lan_addresses():
    addresses = set()
    try:
        output = subprocess.check_output(
            ['hostname', '-I'], text=True, timeout=1.0)
        for address in output.split():
            if ':' not in address and not address.startswith('127.'):
                addresses.add(address)
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None):
            address = item[4][0]
            if ':' not in address and not address.startswith('127.'):
                addresses.add(address)
    except OSError:
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(('8.8.8.8', 80))
        address = probe.getsockname()[0]
        if not address.startswith('127.'):
            addresses.add(address)
        probe.close()
    except OSError:
        pass
    return sorted(addresses)


def communication_profile(
    enable_world_model_publish,
    enable_world_model_summary_publish,
    include_world_model_in_snapshot,
    world_model_publish_rate_hz,
    world_model_summary_publish_rate_hz,
    world_model_topic='/fleet/world_model',
    world_model_summary_topic='/fleet/world_model_summary',
):
    """Summarize gateway bandwidth posture for shore clients."""
    full_enabled = bool(enable_world_model_publish)
    summary_enabled = bool(enable_world_model_summary_publish)
    include_full_snapshot = bool(include_world_model_in_snapshot)
    if full_enabled and include_full_snapshot:
        mode = 'local_full'
    elif summary_enabled and not full_enabled and not include_full_snapshot:
        mode = 'remote_summary'
    else:
        mode = 'custom'
    return {
        'mode': mode,
        'primary_data_source': 'fleet_world_model',
        'world_model_topic': str(world_model_topic),
        'world_model_summary_topic': str(world_model_summary_topic),
        'publish_world_model': full_enabled,
        'publish_world_model_summary': summary_enabled,
        'include_world_model_in_snapshot': include_full_snapshot,
        'world_model_publish_rate_hz': float(world_model_publish_rate_hz),
        'world_model_summary_publish_rate_hz': float(
            world_model_summary_publish_rate_hz
        ),
        'low_bandwidth_recommended': mode == 'remote_summary',
    }


class FleetGatewayNode(Node):
    def __init__(self):
        super().__init__('fleet_gateway')
        self._declare_parameters()
        self.gateway_name = str(self.get_parameter('gateway_name').value)
        self.protocol = ProtocolEncoder(source=self.gateway_name)
        self.boot_id = uuid.uuid4().hex
        self.protocol_v1 = V1ProtocolEncoder(
            source=self.gateway_name,
            stream_epoch=self.boot_id,
        )
        self._vehicle_v1_received_at = {}
        self._vehicle_v1_source_sequence = {}
        self._vehicle_v1_status = {}
        self._v1_command_lock = threading.Lock()
        self._active_v1_command = None
        self._active_v1_mission = None
        self._last_v1_mission_context = None

        # Device-control bridge state is intentionally separate from mission
        # command state. Gateway consumes an existing base-station lease;
        # it never publishes or creates a competing ControlLease.
        self._v1_device_lock = threading.Lock()
        self._v1_device_leases = {}
        self._v1_device_commands = {}

        self.health = HealthMonitor()
        self.registry = FleetRegistry(
            stale_timeout=self._float('stale_timeout_sec'),
            remove_timeout=self._float('remove_timeout_sec'),
            auto_remove=bool(self.get_parameter(
                'auto_remove_offline_vehicle').value),
        )
        self.latest_vehicles = LatestValueStore()
        self.latest_sensors = LatestValueStore()
        self.latest_sensor_streams = LatestValueStore()
        self.latest_world_model = {}
        self.latest_world_model_summary = {}
        self.latest_base_station_state = {}
        self.recent_base_station_events = []
        self.latest_algorithm_status = {}
        self.target_frame = ''
        self.perception_source = 'fleet_world_model'
        self.websocket = None
        self.http = None
        self._push_stop = threading.Event()
        self._push_thread = None
        self._gateway_subscriptions = []
        self.algorithm_action_pub = self.create_publisher(
            String,
            str(self.get_parameter('algorithm_action_topic').value),
            10,
        )
        self.fleet_command_pub = self.create_publisher(
            FleetCommand,
            str(self.get_parameter('fleet_command_topic').value),
            20,
        )
        self._create_subscriptions()
        self._start_transports()
        self._start_push_loop()
        self._print_startup()

    def _hello_v1(self):
        hello = gateway_v1_pb.GatewayHello()
        hello.instance_id = self.gateway_name
        hello.supported_versions.append('1.0')
        hello.runtime_modes.extend([
            gateway_v1_pb.SIMULATION,
            gateway_v1_pb.REAL,
        ])
        hello.capabilities.extend([
            'gateway.heartbeat',
            'telemetry.pose_batch',
            'device.status',
            'mission.status',
            'control.command',
            'control.ack',
            'control.feedback',
            'control.result',
        ])
        hello.binary_telemetry = True
        hello.boot_id = self.boot_id
        return hello

    def _heartbeat_v1(self):
        heartbeat = gateway_v1_pb.GatewayHeartbeat()
        heartbeat.instance_id = self.gateway_name
        heartbeat.boot_id = self.boot_id
        heartbeat.uptime_ms = int(
            max(0.0, time.monotonic() - self.health.started_at) * 1000
        )
        return heartbeat

    def _handle_command_v1(self, payload):
        request = gateway_v1_pb.GatewayEnvelope()
        try:
            request.ParseFromString(payload)
        except Exception:
            return None

        if (
            request.message_type != 'control.command'
            or request.WhichOneof('body') != 'control_command'
        ):
            return None

        command = request.control_command
        command_name = command.command.strip().upper()

        action_map = {
            'MISSION.START': 'CAPTURE',
            'MISSION.PAUSE': 'PAUSE',
            'MISSION.RESUME': 'RESUME',
            'MISSION.CANCEL': 'STOP',
        }

        device_specs = device_command_specs()

        ack = gateway_v1_pb.ControlAck()
        ack.command_id = command.command_id

        with self._v1_command_lock:
            active_mission = dict(self._active_v1_mission or {})

        mission_id = str(
            request.mission_id
            or active_mission.get('mission_id')
            or ''
        )
        run_id = str(
            request.run_id
            or active_mission.get('run_id')
            or ''
        )

        mission_context = {
            'command_id': command.command_id,
            'client_request_id': command.client_request_id,
            'command': command_name,
            'mission_id': mission_id,
            'run_id': run_id,
            'started_at': time.time(),
        }

        action = action_map.get(command_name)

        if not command.command_id:
            ack.status = gateway_v1_pb.REJECTED
            ack.code = 'INVALID_COMMAND_ID'
            ack.message = 'commandId must not be empty.'
            ack.retryable = False

        elif (
            command.HasField('deadline_at')
            and (
                float(command.deadline_at.seconds)
                + float(command.deadline_at.nanos) / 1e9
            ) <= time.time()
        ):
            ack.status = gateway_v1_pb.REJECTED
            ack.code = 'COMMAND_EXPIRED'
            ack.message = 'Command deadline has expired.'
            ack.retryable = False

        elif command_name in device_specs:
            spec = device_specs[command_name]
            depart_target = None
            depart_error = None
            if command_name == 'USV_DEPART':
                try:
                    depart_target = depart_target_from_control(command)
                except ControlCommandValidationError as error:
                    depart_error = error
            device_codes = [
                str(value).strip().lower().replace('-', '_')
                for value in command.target.device_codes
                if str(value).strip()
            ]

            if command.target.scope != gateway_v1_pb.DEVICE:
                ack.status = gateway_v1_pb.REJECTED
                ack.code = 'INVALID_TARGET_SCOPE'
                ack.message = (
                    '%s requires target.scope=DEVICE.'
                    % command_name
                )
                ack.retryable = False

            elif len(device_codes) != 1:
                ack.status = gateway_v1_pb.REJECTED
                ack.code = 'INVALID_DEVICE_COUNT'
                ack.message = (
                    '%s requires exactly one deviceCode.'
                    % command_name
                )
                ack.retryable = False

            elif not device_codes[0].startswith(spec['prefix']):
                ack.status = gateway_v1_pb.REJECTED
                ack.code = 'INVALID_DEVICE_TYPE'
                ack.message = (
                    '%s target must identify one %s.'
                    % (
                        command_name,
                        spec['prefix'].rstrip('_').upper(),
                    )
                )
                ack.retryable = False

            elif depart_error is not None:
                ack.status = gateway_v1_pb.REJECTED
                ack.code = depart_error.code
                ack.message = str(depart_error)
                ack.retryable = False

            else:
                device_code = device_codes[0]
                vehicles = {
                    str(item.get('id')): item
                    for item in self.registry.vehicles()
                }
                vehicle = vehicles.get(device_code)

                if vehicle is None:
                    ack.status = gateway_v1_pb.REJECTED
                    ack.code = 'UNKNOWN_DEVICE'
                    ack.message = (
                        'Unknown deviceCode: %s' % device_code
                    )
                    ack.retryable = False

                elif not bool(vehicle.get('online')):
                    ack.status = gateway_v1_pb.REJECTED
                    ack.code = 'DEVICE_OFFLINE'
                    ack.message = (
                        'Device is offline: %s' % device_code
                    )
                    ack.retryable = True

                else:
                    emergency = bool(spec.get('emergency')) or (
                        spec['command_type']
                        == FleetCommand.COMMAND_EMERGENCY_STOP
                    )
                    lease = (
                        None
                        if emergency
                        else self._valid_base_station_lease_v1(
                            device_code
                        )
                    )

                    if not emergency and lease is None:
                        ack.status = gateway_v1_pb.REJECTED
                        ack.code = 'NO_VALID_CONTROL_LEASE'
                        ack.message = (
                            'No valid shore base-station lease for %s.'
                            % device_code
                        )
                        ack.retryable = True

                    else:
                        ros_command_id = (
                            'gateway-%s-%s'
                            % (
                                device_code,
                                uuid.uuid4().hex[:12],
                            )
                        )

                        ros_command = FleetCommand()
                        ros_command.header.stamp = (
                            self.get_clock().now().to_msg()
                        )
                        ros_command.header.frame_id = 'map'
                        ros_command.command_id = ros_command_id
                        ros_command.vehicle_id = device_code
                        ros_command.lease_id = (
                            ''
                            if lease is None
                            else str(lease.lease_id)
                        )
                        ros_command.command_type = spec['command_type']
                        ros_command.priority = (
                            200
                            if lease is None
                            else int(lease.priority)
                        )
                        ros_command.expires_at = (
                            self._future_stamp_v1(
                                self._float(
                                    'device_command_timeout_sec'
                                )
                            )
                        )
                        ros_command.target_pose.orientation.w = 1.0
                        if depart_target is not None:
                            ros_command.target_pose.position.x = (
                                depart_target[0]
                            )
                            ros_command.target_pose.position.y = (
                                depart_target[1]
                            )
                        ros_command.parameters = list(
                            spec['parameters']
                        )

                        device_context = {
                            'command_id': command.command_id,
                            'client_request_id': (
                                command.client_request_id
                            ),
                            'command': command_name,
                            'mission_id': mission_id,
                            'run_id': run_id,
                            'started_at': time.time(),
                            'ros_command_id': ros_command_id,
                            'device_code': device_code,
                            'command_type': int(
                                spec['command_type']
                            ),
                        }

                        with self._v1_device_lock:
                            duplicate = any(
                                item.get('command_id')
                                == command.command_id
                                for item in (
                                    self._v1_device_commands.values()
                                )
                            )

                            if not duplicate:
                                self._v1_device_commands[
                                    ros_command_id
                                ] = dict(device_context)

                        if duplicate:
                            ack.status = gateway_v1_pb.REJECTED
                            ack.code = 'DUPLICATE_COMMAND_ID'
                            ack.message = (
                                'Device commandId is already active.'
                            )
                            ack.retryable = False
                        else:
                            self._schedule_device_command_v1(
                                ros_command
                            )

                            ack.status = gateway_v1_pb.ACCEPTED
                            ack.code = 'ACCEPTED'
                            ack.message = (
                                '%s accepted and scheduled for '
                                '/fleet/command.'
                                % command_name
                            )
                            ack.retryable = False

                            self.get_logger().info(
                                'v1 device command %s prepared '
                                'platform_command_id=%s '
                                'ros_command_id=%s device=%s '
                                'lease=%s'
                                % (
                                    command_name,
                                    command.command_id,
                                    ros_command_id,
                                    device_code,
                                    ros_command.lease_id,
                                )
                            )

        elif (
            command_name != 'MISSION.START'
            and command_name in action_map
            and not active_mission
        ):
            ack.status = gateway_v1_pb.REJECTED
            ack.code = 'NO_ACTIVE_MISSION'
            ack.message = (
                '%s requires an active mission.' % command_name
            )
            ack.retryable = False

        elif action is None:
            ack.status = gateway_v1_pb.REJECTED
            ack.code = 'UNSUPPORTED_COMMAND'
            ack.message = (
                'Unsupported command: %s' % command.command
            )
            ack.retryable = False

        else:
            try:
                self.algorithm_action_pub.publish(
                    String(data=action)
                )
            except Exception as error:
                ack.status = gateway_v1_pb.REJECTED
                ack.code = 'ROS_PUBLISH_FAILED'
                ack.message = (
                    'Failed to publish algorithm action: %s'
                    % error
                )
                ack.retryable = True
                self.get_logger().error(ack.message)
            else:
                with self._v1_command_lock:
                    self._active_v1_command = dict(
                        mission_context
                    )

                    if command_name == 'MISSION.START':
                        self._active_v1_mission = dict(
                            mission_context
                        )
                        self._last_v1_mission_context = None

                ack.status = gateway_v1_pb.ACCEPTED
                ack.code = 'ACCEPTED'
                ack.message = (
                    '%s accepted and routed to '
                    '/fleet/algorithm/action.'
                    % command_name
                )
                ack.retryable = False

                self.get_logger().info(
                    'v1 mission command %s -> ROS1 '
                    'algorithm action %s command_id=%s '
                    'mission_id=%s run_id=%s'
                    % (
                        command_name,
                        action,
                        command.command_id,
                        mission_id,
                        run_id,
                    )
                )

        response = self.protocol_v1.envelope(
            'control.ack',
            'control.command.%s' % command.command_id,
            ack,
        )
        response.mission_id = mission_id
        response.run_id = run_id
        response.correlation_id = command.command_id
        return response.SerializeToString()

    def _send_control_feedback_v1(
        self, context, phase, message, progress=0.0
    ):
        if self.websocket is None or not self.websocket.running:
            return

        feedback = gateway_v1_pb.ControlFeedback()
        feedback.command_id = str(context.get('command_id') or '')
        feedback.status = gateway_v1_pb.EXECUTING
        feedback.progress = float(progress)
        feedback.phase = str(phase)
        feedback.message = str(message)

        envelope = self.protocol_v1.envelope(
            'control.feedback',
            'control.command.%s' % feedback.command_id,
            feedback,
        )
        envelope.mission_id = str(context.get('mission_id') or '')
        envelope.run_id = str(context.get('run_id') or '')
        envelope.correlation_id = feedback.command_id

        self.websocket.broadcast_path(
            '/uav_usv/v1',
            envelope.SerializeToString(),
            priority=2,
            opcode=0x2,
        )

    def _send_control_result_v1(
        self, context, status, code, message
    ):
        if self.websocket is None or not self.websocket.running:
            return

        result = gateway_v1_pb.ControlResult()
        result.command_id = str(context.get('command_id') or '')
        result.status = status
        result.code = str(code)
        result.message = str(message)

        started = float(context.get('started_at') or time.time())
        completed = time.time()

        started_sec = int(started)
        result.started_at.seconds = started_sec
        result.started_at.nanos = int(
            max(0.0, started - started_sec) * 1000000000
        )

        completed_sec = int(completed)
        result.completed_at.seconds = completed_sec
        result.completed_at.nanos = int(
            max(0.0, completed - completed_sec) * 1000000000
        )

        envelope = self.protocol_v1.envelope(
            'control.result',
            'control.command.%s' % result.command_id,
            result,
        )
        envelope.mission_id = str(context.get('mission_id') or '')
        envelope.run_id = str(context.get('run_id') or '')
        envelope.correlation_id = result.command_id

        self.websocket.broadcast_path(
            '/uav_usv/v1',
            envelope.SerializeToString(),
            priority=2,
            opcode=0x2,
        )


    def _future_stamp_v1(self, seconds):
        now = self.get_clock().now()
        nanoseconds = (
            int(now.nanoseconds)
            + int(float(seconds) * 1000000000)
        )
        stamp = now.to_msg()
        stamp.secs = nanoseconds // 1000000000
        stamp.nsecs = nanoseconds % 1000000000
        return stamp

    @staticmethod
    def _stamp_nanoseconds_v1(stamp):
        return (
            int(getattr(stamp, 'secs', 0)) * 1000000000
            + int(getattr(stamp, 'nsecs', 0))
        )

    def _on_control_lease_v1(self, message):
        owner = str(
            self.get_parameter(
                'base_station_lease_owner'
            ).value
        )

        if str(message.owner_id) != owner:
            return

        vehicle_id = str(message.vehicle_id).strip()
        if not vehicle_id:
            return

        with self._v1_device_lock:
            current = self._v1_device_leases.get(vehicle_id)

            if bool(message.revoked):
                if (
                    current is not None
                    and str(current.lease_id)
                    == str(message.lease_id)
                ):
                    self._v1_device_leases.pop(
                        vehicle_id, None
                    )
            else:
                self._v1_device_leases[vehicle_id] = message

    def _valid_base_station_lease_v1(self, device_code):
        owner = str(
            self.get_parameter(
                'base_station_lease_owner'
            ).value
        )
        now_ns = int(self.get_clock().now().nanoseconds)

        with self._v1_device_lock:
            candidates = [
                self._v1_device_leases.get(device_code),
                self._v1_device_leases.get('*'),
            ]

        for lease in candidates:
            if lease is None:
                continue
            if str(lease.owner_id) != owner:
                continue
            if bool(lease.revoked):
                continue
            if str(lease.vehicle_id) not in (
                device_code,
                '*',
            ):
                continue
            if not str(lease.lease_id):
                continue
            if (
                self._stamp_nanoseconds_v1(
                    lease.valid_until
                )
                <= now_ns
            ):
                continue
            return lease

        return None

    def _schedule_device_command_v1(self, ros_command):
        timer = threading.Timer(
            0.05,
            self._dispatch_device_command_v1,
            args=(ros_command,),
        )
        timer.daemon = True
        timer.start()

    def _dispatch_device_command_v1(self, ros_command):
        with self._v1_device_lock:
            context = dict(
                self._v1_device_commands.get(
                    ros_command.command_id
                )
                or {}
            )

        if not context:
            return

        try:
            self.fleet_command_pub.publish(ros_command)
        except Exception as error:
            with self._v1_device_lock:
                self._v1_device_commands.pop(
                    ros_command.command_id,
                    None,
                )

            self.get_logger().error(
                'Failed to publish FleetCommand %s: %s'
                % (
                    ros_command.command_id,
                    error,
                )
            )
            self._send_control_result_v1(
                context,
                gateway_v1_pb.FAILED,
                'ROS_PUBLISH_FAILED',
                'Failed to publish /fleet/command: %s'
                % error,
            )
            return

        self.get_logger().info(
            'v1 ROS1 FleetCommand dispatched '
            'ros_command_id=%s device=%s type=%d'
            % (
                ros_command.command_id,
                ros_command.vehicle_id,
                int(ros_command.command_type),
            )
        )

    def _on_command_ack_v1(self, message):
        ros_command_id = str(message.command_id)

        with self._v1_device_lock:
            context = dict(
                self._v1_device_commands.get(
                    ros_command_id
                )
                or {}
            )

        if not context:
            return

        expected_device = str(
            context.get('device_code') or ''
        )
        ack_device = str(message.vehicle_id or '')

        if (
            ack_device
            and expected_device
            and ack_device != expected_device
        ):
            self.get_logger().warning(
                'Ignoring CommandAck device mismatch '
                'command_id=%s expected=%s got=%s'
                % (
                    ros_command_id,
                    expected_device,
                    ack_device,
                )
            )
            return

        status = int(message.status)
        progress = max(
            0.0,
            min(1.0, float(message.progress)),
        )
        message_text = str(message.message or '')

        feedback_phases = {
            CommandAck.STATUS_RECEIVED: 'ROS_RECEIVED',
            CommandAck.STATUS_ACCEPTED: 'ROS_ACCEPTED',
            CommandAck.STATUS_EXECUTING: 'ROS_EXECUTING',
        }

        if status in feedback_phases:
            self._send_control_feedback_v1(
                context,
                feedback_phases[status],
                message_text,
                progress,
            )
            return

        terminal = {
            CommandAck.STATUS_SUCCEEDED: (
                gateway_v1_pb.SUCCEEDED,
                'ROS_SUCCEEDED',
            ),
            CommandAck.STATUS_REJECTED: (
                gateway_v1_pb.REJECTED,
                'ROS_REJECTED',
            ),
            CommandAck.STATUS_FAILED: (
                gateway_v1_pb.FAILED,
                'ROS_FAILED',
            ),
            CommandAck.STATUS_CANCELED: (
                gateway_v1_pb.CANCELLED,
                'ROS_CANCELED',
            ),
        }

        result_spec = terminal.get(status)
        if result_spec is None:
            self.get_logger().warning(
                'Ignoring unknown CommandAck status=%s '
                'command_id=%s'
                % (
                    status,
                    ros_command_id,
                )
            )
            return

        with self._v1_device_lock:
            self._v1_device_commands.pop(
                ros_command_id,
                None,
            )

        result_status, result_code = result_spec
        self._send_control_result_v1(
            context,
            result_status,
            result_code,
            message_text,
        )

    def _process_algorithm_status_v1(self):
        status = self._mission_status_v1()
        if status is None:
            return

        state = str(status.state or 'IDLE').upper()
        phase = str(status.phase or '')
        progress = float(status.progress)

        operation_result = None
        mission_result = None
        mission_feedback = None

        with self._v1_command_lock:
            active_command = (
                dict(self._active_v1_command)
                if self._active_v1_command is not None
                else {}
            )
            mission = (
                dict(self._active_v1_mission)
                if self._active_v1_mission is not None
                else {}
            )

            command_name = str(active_command.get('command') or '')

            if command_name == 'MISSION.PAUSE' and state == 'PAUSED':
                operation_result = (
                    active_command,
                    gateway_v1_pb.SUCCEEDED,
                    'MISSION_PAUSED',
                    'Mission pause completed.',
                )
                self._active_v1_command = None

            elif (
                command_name == 'MISSION.RESUME'
                and state in ('WAITING', 'RUNNING', 'SUCCESS')
            ):
                operation_result = (
                    active_command,
                    gateway_v1_pb.SUCCEEDED,
                    'MISSION_RESUMED',
                    'Mission resume completed.',
                )
                self._active_v1_command = None

            elif command_name == 'MISSION.CANCEL' and state == 'IDLE':
                operation_result = (
                    active_command,
                    gateway_v1_pb.SUCCEEDED,
                    'MISSION_CANCEL_COMPLETED',
                    'Mission cancel completed.',
                )
                self._active_v1_command = None

                if mission:
                    mission_result = (
                        mission,
                        gateway_v1_pb.CANCELLED,
                        'MISSION_CANCELLED',
                        'Mission cancelled.',
                    )
                    self._last_v1_mission_context = dict(mission)
                    self._active_v1_mission = None

            elif (
                command_name in (
                    'MISSION.PAUSE',
                    'MISSION.RESUME',
                    'MISSION.CANCEL',
                )
                and state == 'FAILED'
            ):
                operation_result = (
                    active_command,
                    gateway_v1_pb.FAILED,
                    'ALGORITHM_FAILED',
                    'Algorithm failed while applying %s.' % command_name,
                )
                self._active_v1_command = None

            if mission and mission_result is None:
                if state == 'SUCCESS':
                    mission_result = (
                        mission,
                        gateway_v1_pb.SUCCEEDED,
                        'MISSION_SUCCEEDED',
                        'Mission completed successfully.',
                    )
                    self._last_v1_mission_context = dict(mission)
                    self._active_v1_mission = None

                    if (
                        self._active_v1_command is not None
                        and self._active_v1_command.get('command_id')
                        == mission.get('command_id')
                    ):
                        self._active_v1_command = None

                elif state == 'FAILED':
                    mission_result = (
                        mission,
                        gateway_v1_pb.FAILED,
                        'MISSION_FAILED',
                        'Mission failed.',
                    )
                    self._last_v1_mission_context = dict(mission)
                    self._active_v1_mission = None

                    if (
                        self._active_v1_command is not None
                        and self._active_v1_command.get('command_id')
                        == mission.get('command_id')
                    ):
                        self._active_v1_command = None

                elif state in ('WAITING', 'RUNNING', 'PAUSED'):
                    feedback_key = '%s|%s' % (state, phase)
                    stored = self._active_v1_mission

                    if (
                        stored is not None
                        and stored.get('last_feedback_key') != feedback_key
                    ):
                        stored['last_feedback_key'] = feedback_key
                        stored['seen_non_idle'] = True
                        mission_feedback = dict(stored)

                elif (
                    state == 'IDLE'
                    and bool(mission.get('seen_non_idle'))
                    and command_name != 'MISSION.CANCEL'
                ):
                    mission_result = (
                        mission,
                        gateway_v1_pb.CANCELLED,
                        'MISSION_STOPPED',
                        'Mission stopped before successful completion.',
                    )
                    self._last_v1_mission_context = dict(mission)
                    self._active_v1_mission = None

                    if (
                        self._active_v1_command is not None
                        and self._active_v1_command.get('command_id')
                        == mission.get('command_id')
                    ):
                        self._active_v1_command = None

        if mission_feedback is not None:
            self._send_control_feedback_v1(
                mission_feedback,
                phase,
                'Mission state: %s.' % state,
                progress,
            )

        if operation_result is not None:
            self._send_control_result_v1(*operation_result)

        if mission_result is not None:
            self._send_control_result_v1(*mission_result)


    def _declare_parameters(self):
        defaults = {
            'gateway_name': 'uav_usv_fleet_gateway',
            'use_sim_time': False,
            'fleet_id': 'demo',
            'bind_address': '0.0.0.0',
            'enable_websocket': True,
            'websocket_port': 8765,
            'websocket_path': '/ws',
            'enable_http_server': True,
            'http_port': 8080,
            'vehicle_publish_rate_hz': 10.0,
            'target_publish_rate_hz': 10.0,
            'snapshot_rate_hz': 1.0,
            'diagnostics_rate_hz': 1.0,
            'sensor_publish_rate_hz': 1.0,
            'sensor_stream_publish_rate_hz': 20.0,
            'enable_pose_batch_v1': True,
            'pose_batch_v1_rate_hz': 20.0,
            'enable_device_status_v1': True,
            'device_status_v1_rate_hz': 5.0,
            'enable_mission_status_v1': True,
            'mission_status_v1_rate_hz': 1.0,
            'algorithm_status_topic': '/fleet/algorithm/status',
            'algorithm_action_topic': '/fleet/algorithm/action',
            'enable_world_model_publish': True,
            'world_model_publish_rate_hz': 2.0,
            'enable_world_model_summary_publish': True,
            'world_model_summary_publish_rate_hz': 1.0,
            'include_world_model_in_snapshot': True,
            'stale_timeout_sec': 3.0,
            'remove_timeout_sec': 30.0,
            'max_clients': 8,
            'client_queue_size': 100,
            'auto_remove_offline_vehicle': False,
            'enable_mqtt': False,
            'world_model_topic': '/fleet/world_model',
            'world_model_summary_topic': '/fleet/world_model_summary',
            'enable_base_station_service': True,
            'base_station_state_topic': '/base_station/state',
            'base_station_events_topic': '/base_station/events',
            'enable_legacy_topic_fallback': False,
            'vehicle_state_topics': ['/fleet/state'],
            'perception_targets_topic': '/fleet/perception/targets',
            'sensor_status_topic': '/fleet/sensor_status',
            'sensor_stream_topic': '/fleet/gateway/sensor_stream',
            'mission_state_topic': '/capture/state',
            'perception_source_status_topic': '/perception/source_status',
        }
        defaults.update({
            'fleet_command_topic': '/fleet/command',
            'command_ack_topic': '/fleet/command_ack',
            'control_lease_topic': '/fleet/control_lease',
            'base_station_lease_owner': 'shore_base_station',
            'device_command_timeout_sec': 30.0,
        })

        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _float(self, name):
        return float(self.get_parameter(name).value)

    def _create_subscriptions(self):
        world_model_topic = str(self.get_parameter(
            'world_model_topic').value)
        world_model_summary_topic = str(self.get_parameter(
            'world_model_summary_topic').value)
        use_base_station_service = bool(self.get_parameter(
            'enable_base_station_service').value)
        if use_base_station_service:
            self._gateway_subscriptions.extend([
                self.create_subscription(
                    String,
                    str(self.get_parameter('base_station_state_topic').value),
                    self._on_base_station_state,
                    10,
                ),
                self.create_subscription(
                    String,
                    str(self.get_parameter('base_station_events_topic').value),
                    self._on_base_station_event,
                    50,
                ),
            ])
        else:
            self._gateway_subscriptions.extend([
                self.create_subscription(
                    String, world_model_topic, self._on_world_model, 10),
                self.create_subscription(
                    String, world_model_summary_topic,
                    self._on_world_model_summary, 10),
            ])
        vehicle_topics = list(self.get_parameter(
            'vehicle_state_topics').value)
        targets_topic = str(self.get_parameter(
            'perception_targets_topic').value)
        sensor_topic = str(self.get_parameter('sensor_status_topic').value)
        mission_topic = str(self.get_parameter('mission_state_topic').value)
        source_topic = str(self.get_parameter(
            'perception_source_status_topic').value)
        sensor_stream_topic = str(
            self.get_parameter('sensor_stream_topic').value)
        self._gateway_subscriptions.append(self.create_subscription(
            String, sensor_stream_topic, self._on_sensor_stream, 10))
        if bool(self.get_parameter('enable_mission_status_v1').value):
            self._gateway_subscriptions.append(self.create_subscription(
                String,
                str(self.get_parameter('algorithm_status_topic').value),
                self._on_algorithm_status,
                10,
            ))

        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._gateway_subscriptions.extend([
            self.create_subscription(
                ControlLease,
                str(self.get_parameter('control_lease_topic').value),
                self._on_control_lease_v1,
                lease_qos,
            ),
            self.create_subscription(
                CommandAck,
                str(self.get_parameter('command_ack_topic').value),
                self._on_command_ack_v1,
                50,
            ),
        ])

        legacy_fallback = bool(self.get_parameter(
            'enable_legacy_topic_fallback').value)
        pose_batch_v1 = bool(self.get_parameter('enable_pose_batch_v1').value)
        if legacy_fallback or pose_batch_v1:
            for topic in vehicle_topics:
                self._gateway_subscriptions.append(self.create_subscription(
                    VehicleState, str(topic), self._on_vehicle,
                    qos_profile_sensor_data))
        if legacy_fallback:
            self._gateway_subscriptions.extend([
                self.create_subscription(
                    TrackedObjectArray, targets_topic, self._on_targets, 10),
                self.create_subscription(
                    SensorStatus, sensor_topic, self._on_sensor,
                    qos_profile_sensor_data),
                self.create_subscription(
                    CaptureState, mission_topic, self._on_mission, 10),
                self.create_subscription(String, source_topic,
                                         self._on_source_status, 10),
            ])
        self.topic_summary = {
            'base_station_service_enabled': use_base_station_service,
            'base_station_state': str(self.get_parameter(
                'base_station_state_topic').value),
            'base_station_events': str(self.get_parameter(
                'base_station_events_topic').value),
            'primary_world_model': world_model_topic,
            'world_model_summary': world_model_summary_topic,
            'legacy_fallback_enabled': bool(self.get_parameter(
                'enable_legacy_topic_fallback').value),
            'legacy_vehicle_state': vehicle_topics,
            'legacy_perception_targets': targets_topic,
            'legacy_sensor_status': sensor_topic,
            'sensor_stream': sensor_stream_topic,
            'algorithm_status': str(self.get_parameter(
                'algorithm_status_topic').value),
            'algorithm_action': str(self.get_parameter(
                'algorithm_action_topic').value),
            'fleet_command': str(self.get_parameter(
                'fleet_command_topic').value),
            'command_ack': str(self.get_parameter(
                'command_ack_topic').value),
            'control_lease': str(self.get_parameter(
                'control_lease_topic').value),
            'base_station_lease_owner': str(self.get_parameter(
                'base_station_lease_owner').value),
            'legacy_mission_state': mission_topic,
            'legacy_perception_source_status': source_topic,
        }

    def _start_transports(self):
        bind = str(self.get_parameter('bind_address').value)
        if bool(self.get_parameter('enable_websocket').value):
            try:
                self.websocket = FleetWebSocketServer(
                    host=bind,
                    port=int(self.get_parameter('websocket_port').value),
                    path=str(self.get_parameter('websocket_path').value),
                    protocol=self.protocol,
                    hello_factory=self._hello,
                    snapshot_factory=self._snapshot,
                    queue_size=int(self.get_parameter(
                        'client_queue_size').value),
                    max_clients=int(self.get_parameter('max_clients').value),
                    sent_callback=lambda count: self.health.increment(
                        'sent_messages', count),
                    drop_callback=lambda count: self.health.increment(
                        'dropped_messages', count),
                    protocol_v1=self.protocol_v1,
                    hello_factory_v1=self._hello_v1,
                    command_handler_v1=self._handle_command_v1,
                )
                self.websocket.start()
                self.health.websocket_running = True
            except OSError as error:
                self.websocket = None
                self.health.websocket_running = False
                self.get_logger().error(
                    'WebSocket disabled after startup failure: %s' % error)
        if bool(self.get_parameter('enable_http_server').value):
            try:
                share = Path(get_package_share_directory(
                    'uav_usv_fleet_gateway'))
                self.http = FleetHttpServer(
                    bind, int(self.get_parameter('http_port').value),
                    share / 'web', self._http_health)
                self.http.start()
            except (OSError, FileNotFoundError) as error:
                self.http = None
                self.get_logger().error(
                    'HTTP server disabled after startup failure: %s' % error)

    def _start_push_loop(self):
        """Run WebSocket publication on wall time, independent of /clock."""
        self._push_stop.clear()
        self._push_thread = threading.Thread(
            target=self._push_loop,
            name='fleet-gateway-publisher',
            daemon=True,
        )
        self._push_thread.start()

    def _push_loop(self):
        tasks = {
            'vehicle': [
                1.0 / max(0.1, self._float('vehicle_publish_rate_hz')),
                self.broadcast_vehicle_state,
            ],
            'targets': [
                1.0 / max(0.1, self._float('target_publish_rate_hz')),
                self.broadcast_targets,
            ],
            'sensors': [
                1.0 / max(0.1, self._float('sensor_publish_rate_hz')),
                self._broadcast_sensor_status,
            ],
            'sensor_streams': [
                1.0 / max(
                    0.1, self._float('sensor_stream_publish_rate_hz')),
                self._broadcast_sensor_streams,
            ],
            'world_model': [
                1.0 / max(0.1, self._float('world_model_publish_rate_hz')),
                self.broadcast_world_model,
            ],
            'world_model_summary': [
                1.0 / max(
                    0.1,
                    self._float('world_model_summary_publish_rate_hz'),
                ),
                self.broadcast_world_model_summary,
            ],
            'snapshot': [
                1.0 / max(0.1, self._float('snapshot_rate_hz')),
                self.broadcast_snapshot,
            ],
            'base_station_snapshot': [
                1.0 / max(0.1, self._float('snapshot_rate_hz')),
                self.broadcast_base_station_snapshot,
            ],
            'diagnostics': [
                1.0 / max(0.1, self._float('diagnostics_rate_hz')),
                self._broadcast_diagnostics,
            ],
            'heartbeat_v1': [
                1.0,
                self._broadcast_heartbeat_v1,
            ],
            'pose_batch_v1': [
                1.0 / max(0.1, self._float('pose_batch_v1_rate_hz')),
                self._broadcast_pose_batch_v1,
            ],
            'device_status_v1': [
                1.0 / max(
                    0.1, self._float('device_status_v1_rate_hz')),
                self._broadcast_device_status_v1,
            ],
            'mission_status_v1': [
                1.0 / max(0.1, self._float('mission_status_v1_rate_hz')),
                self._broadcast_mission_status_v1,
            ],
        }
        now = time.monotonic()
        deadlines = {name: now for name in tasks}
        while not self._push_stop.is_set():
            now = time.monotonic()
            for name, (interval, callback) in tasks.items():
                if now < deadlines[name]:
                    continue
                try:
                    callback()
                except Exception as error:  # Keep other streams alive.
                    self.get_logger().error(
                        'Gateway push %s failed: %s' % (name, error))
                deadlines[name] = now + interval
            next_deadline = min(deadlines.values())
            self._push_stop.wait(max(
                0.001, min(0.02, next_deadline - time.monotonic())))

    def _on_vehicle(self, message):
        now = time.monotonic()
        model = vehicle_from_ros(message, now)
        self._vehicle_v1_received_at[model.id] = now
        self._vehicle_v1_source_sequence[model.id] = int(
            getattr(message.header, 'seq', 0) or 0)
        self.registry.update_vehicle(model)
        self.latest_vehicles.update(model.id, model.public())

        self._vehicle_v1_status[model.id] = {
            'vehicle_type': int(
                getattr(message, 'vehicle_type', 0) or 0),
            'online': bool(getattr(message, 'online', False)),
            'armed': bool(getattr(message, 'armed', False)),
            'mode': str(getattr(message, 'mode', '') or ''),
            'battery_percent': float(
                getattr(message, 'battery_percent', 0.0) or 0.0),
            'active_command_id': str(
                getattr(message, 'active_command_id', '') or ''),
            'status_text': str(
                getattr(message, 'status_text', '') or ''),
        }

        self.health.increment('received_messages')

    def _on_targets(self, message):
        now = time.monotonic()
        models = [target_from_ros(
            item, message.header, now, formal_source='source_mux')
            for item in message.objects]
        self.registry.update_targets(models)
        self.target_frame = str(message.header.frame_id)
        self.health.increment('received_messages')

    def _on_sensor(self, message):
        model = sensor_from_ros(message, time.monotonic())
        self.registry.update_sensor(model)
        self.latest_sensors.update(
            (model.vehicle_id, model.sensor_id), model.public())
        self.health.increment('received_messages')

    def _on_sensor_stream(self, message):
        """Cache the latest compressed frame without blocking a ROS callback."""
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.health.increment('dropped_messages')
            return
        if not isinstance(payload, dict):
            self.health.increment('dropped_messages')
            return
        message_type = str(payload.pop('message_type', 'sensor_stream'))
        stream_id = str(payload.get('stream_id') or payload.get(
            'vehicle_id') or message_type)
        self.latest_sensor_streams.update(
            '%s:%s' % (message_type, stream_id),
            {'message_type': message_type, 'data': payload},
        )
        self.health.increment('received_messages')

    def _on_algorithm_status(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Invalid algorithm status JSON')
            return
        if not isinstance(status, dict) or status.get(
                'schema_version') != 'uav_usv_algorithm_status.v1':
            self.get_logger().warning(
                'Ignoring unsupported algorithm status payload')
            return
        self.latest_algorithm_status = dict(status)
        self.health.increment('received_messages')
        self._process_algorithm_status_v1()

    def _on_mission(self, message):
        self.registry.update_mission(mission_from_ros(message))
        self.health.increment('received_messages')

    def _on_source_status(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            status = {
                'source': 'not_available',
                'raw_status': str(message.data),
            }
        self.registry.update_source_status(status)
        if status.get('source'):
            self.perception_source = str(status['source'])
        self.health.increment('received_messages')

    def _on_world_model(self, message):
        try:
            model = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Invalid Fleet World Model JSON')
            return
        if not isinstance(model, dict):
            return
        now = time.monotonic()
        self.registry.update_world_model(model, now)
        self.latest_world_model = model
        perception = model.get('perception') or {}
        self.perception_source = str(
            perception.get('primary_source') or 'fleet_world_model'
        )
        self.target_frame = str(model.get('map_frame') or 'map')
        self.health.increment('received_messages')

    def _on_base_station_state(self, message):
        """Cache the Base Station contract without recomputing situation."""
        try:
            state = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Invalid Base Station state JSON')
            return
        if not isinstance(state, dict) or state.get(
                'schema_version') != 'base_station_service.v1':
            self.get_logger().warning('Ignoring unsupported Base Station state')
            return
        now = time.monotonic()
        self.latest_base_station_state = state
        # Preserve legacy payloads as a view of the same immutable snapshot.
        self.registry.update_world_model(state, now)
        self.latest_world_model = state
        perception = state.get('perception') or {}
        self.perception_source = str(
            perception.get('primary_source') or 'base_station_service'
        )
        self.target_frame = str(state.get('map_frame') or 'map')
        self.health.increment('received_messages')

    def _on_base_station_event(self, message):
        try:
            event = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Invalid Base Station event JSON')
            return
        if not isinstance(event, dict) or event.get(
                'schema_version') != 'base_station_event.v1':
            return
        self.recent_base_station_events.append(event)
        del self.recent_base_station_events[:-100]
        self._broadcast('base_station_event', event, priority=2)
        self.health.increment('received_messages')

    def _on_world_model_summary(self, message):
        try:
            summary = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(summary, dict):
            self.registry.update_world_model_summary(summary)
            self.latest_world_model_summary = summary
            self.health.increment('received_messages')

    def _broadcast(self, message_type, data, priority=0):
        if self.websocket is None or not self.websocket.running:
            return
        self.websocket.broadcast(
            self.protocol.dumps(message_type, data), priority=priority)

    def broadcast_vehicle_state(self):
        """Broadcast the latest state of every registered vehicle at 10 Hz."""
        for value in self.registry.vehicles():
            alert = value.get('stale') or not value.get('online')
            priority = 2 if alert else 0
            self._broadcast('vehicle_state', value, priority=priority)

    def broadcast_targets(self):
        """Broadcast the latest formal perception target set at 10 Hz."""
        self._broadcast('perception_targets', {
            'frame_id': self.target_frame,
            'source': 'fleet_world_model',
            'selected_source': self.perception_source,
            'targets': self.registry.targets(),
        })

    def _broadcast_sensor_status(self):
        for value in self.registry.sensors():
            priority = 2 if not value.get('online') else 1
            self._broadcast('sensor_status', value, priority=priority)

    def _broadcast_sensor_streams(self):
        for item in self.latest_sensor_streams.pop_dirty():
            self._broadcast(
                item['message_type'], item['data'], priority=0)

    def broadcast_world_model(self):
        if not bool(self.get_parameter('enable_world_model_publish').value):
            return
        model = self.registry.world_model()
        if model:
            self._broadcast('fleet_world_model', model, priority=1)

    def broadcast_world_model_summary(self):
        if not bool(self.get_parameter(
                'enable_world_model_summary_publish').value):
            return
        summary = self.registry.world_model_summary()
        if summary:
            self._broadcast('fleet_world_model_summary', summary, priority=1)

    def broadcast_snapshot(self):
        """Broadcast a complete recovery snapshot at 1 Hz."""
        self._broadcast('fleet_snapshot', self._snapshot(), priority=1)

    def broadcast_base_station_snapshot(self):
        """Broadcast the authoritative shore-state payload at 1 Hz."""
        if self.latest_base_station_state:
            self._broadcast(
                'base_station_snapshot', self.latest_base_station_state,
                priority=1,
            )


    def _mission_status_v1(self):
        source = self.latest_algorithm_status
        if not isinstance(source, dict) or not source:
            return None

        phase = str(source.get('phase') or 'idle')
        phase_upper = phase.upper()
        mode = str(source.get('mode') or 'idle').lower()
        active = bool(source.get('active', False))
        last_error = str(source.get('last_error') or '')
        diagnostics = source.get('diagnostics') or {}
        if not isinstance(diagnostics, dict):
            diagnostics = {}
        captured = bool(diagnostics.get('captured', False))

        if phase_upper == 'ALGORITHM_ERROR' or last_error:
            state = 'FAILED'
            progress = 0.0
        elif phase_upper == 'CAPTURED_GUARDING' and captured:
            state = 'SUCCESS'
            progress = 1.0
        elif phase.lower() == 'paused':
            state = 'PAUSED'
            progress = 0.0
        elif phase_upper in (
            'WAITING_FOR_VEHICLES',
            'WAITING_FOR_ENEMY_ENTITY',
            'WAITING_FOR_FRIENDLY_ENTITY',
        ):
            state = 'WAITING'
            progress = 0.0
        elif active and phase_upper == 'GB_SFLA_CS_INTERCEPT':
            state = 'RUNNING'
            progress = 0.0
        elif active and mode != 'idle':
            state = 'RUNNING'
            progress = 0.0
        else:
            state = 'IDLE'
            progress = 0.0

        status = gateway_v1_pb.MissionStatus()
        status.state = state
        status.phase = phase
        status.progress = progress

        with self._v1_command_lock:
            mission_context = dict(
                self._active_v1_mission
                or self._last_v1_mission_context
                or {}
            )
            command_context = dict(self._active_v1_command or {})

        status.mission_id = str(mission_context.get('mission_id') or '')
        status.run_id = str(mission_context.get('run_id') or '')
        status.active_command_id = str(
            command_context.get('command_id')
            or mission_context.get('command_id')
            or ''
        )
        return status

    def _broadcast_mission_status_v1(self):
        if not bool(self.get_parameter('enable_mission_status_v1').value):
            return
        if self.websocket is None or not self.websocket.running:
            return
        status = self._mission_status_v1()
        if status is None:
            return

        envelope = self.protocol_v1.envelope(
            'mission.status',
            'mission.status',
            status,
        )
        envelope.mission_id = status.mission_id
        envelope.run_id = status.run_id
        envelope.correlation_id = status.active_command_id
        self.websocket.broadcast_path(
            '/uav_usv/v1',
            envelope.SerializeToString(),
            priority=2,
            opcode=0x2,
        )

    @staticmethod
    def _set_proto_timestamp(timestamp, value):
        try:
            seconds_value = float(value)
        except (TypeError, ValueError):
            return
        if not math.isfinite(seconds_value) or seconds_value <= 0.0:
            return
        whole = int(seconds_value)
        nanos = int(round((seconds_value - whole) * 1_000_000_000.0))
        if nanos >= 1_000_000_000:
            whole += 1
            nanos -= 1_000_000_000
        timestamp.seconds = whole
        timestamp.nanos = max(0, nanos)

    @staticmethod
    def _valid_xyz(value):
        if not isinstance(value, dict):
            return False
        try:
            return all(
                value.get(axis) is not None
                and math.isfinite(float(value.get(axis)))
                for axis in ('x', 'y', 'z')
            )
        except (TypeError, ValueError):
            return False

    def _pose_batch_v1(self):
        batch = gateway_v1_pb.PoseBatch()
        batch.snapshot_mode = gateway_v1_pb.LATEST_STATE
        batch.snapshot_time.GetCurrentTime()

        threshold_ms = int(self.registry.stale_timeout * 1000.0)
        batch.freshness_threshold_ms = threshold_ms
        expected_codes = (
            'uav_01', 'uav_02', 'uav_03',
            'usv_01', 'usv_02', 'usv_03',
        )
        batch.expected_device_codes.extend(expected_codes)

        values = {
            str(item.get('id')): item
            for item in self.latest_vehicles.values()
            if isinstance(item, dict) and item.get('id')
        }
        missing_codes = [code for code in expected_codes if code not in values]
        batch.missing_device_codes.extend(missing_codes)

        stale_codes = []
        now = time.monotonic()
        for code in expected_codes:
            item = values.get(code)
            if item is None:
                continue

            received_at = self._vehicle_v1_received_at.get(code)
            age_ms = (
                int(max(0.0, now - received_at) * 1000.0)
                if received_at is not None else threshold_ms + 1
            )
            fresh = (
                age_ms <= threshold_ms
                and bool(item.get('online', False))
                and not bool(item.get('stale', False))
            )
            if not fresh:
                stale_codes.append(code)

            sample = batch.vehicles.add()
            sample.device_code = code
            self._set_proto_timestamp(
                sample.source_timestamp, item.get('last_update'))
            sample.source_sequence = int(
                self._vehicle_v1_source_sequence.get(code, 0))
            sample.age_ms = max(0, age_ms)
            sample.fresh = fresh

            position = item.get('position') or {}
            sample.position_valid = self._valid_xyz(position)
            if sample.position_valid:
                sample.local_position_enu_m.x = float(position['x'])
                sample.local_position_enu_m.y = float(position['y'])
                sample.local_position_enu_m.z = float(position['z'])

            orientation = item.get('orientation') or {}
            quaternion_keys = ('qx', 'qy', 'qz', 'qw')
            if all(orientation.get(key) is not None for key in quaternion_keys):
                try:
                    sample.orientation.x = float(orientation['qx'])
                    sample.orientation.y = float(orientation['qy'])
                    sample.orientation.z = float(orientation['qz'])
                    sample.orientation.w = float(orientation['qw'])
                except (TypeError, ValueError):
                    pass

            velocity = item.get('linear_velocity') or {}
            if self._valid_xyz(velocity):
                sample.linear_velocity_mps.x = float(velocity['x'])
                sample.linear_velocity_mps.y = float(velocity['y'])
                sample.linear_velocity_mps.z = float(velocity['z'])

            yaw = orientation.get('yaw')
            try:
                if yaw is not None and math.isfinite(float(yaw)):
                    sample.heading_deg = (
                        math.degrees(float(yaw)) + 360.0
                    ) % 360.0
            except (TypeError, ValueError):
                pass

        batch.stale_device_codes.extend(stale_codes)
        batch.complete = not missing_codes and not stale_codes
        return batch

    def _device_status_v1(self, device_code):
        item = self._vehicle_v1_status.get(device_code)
        if not isinstance(item, dict):
            return None

        status = gateway_v1_pb.DeviceStatus()
        status.device_code = str(device_code)

        received_at = self._vehicle_v1_received_at.get(device_code)
        age = (
            max(0.0, time.monotonic() - received_at)
            if received_at is not None else float('inf')
        )

        # Do not keep presenting an old cached ROS state as ONLINE.
        # Backend requires a fresh device.status for UAV safety gating.
        freshness_limit = min(
            float(self.registry.stale_timeout), 2.0
        )
        fresh = (
            age <= freshness_limit
            and bool(item.get('online', False))
        )

        status.connection_state = (
            'ONLINE' if fresh else 'OFFLINE'
        )

        status.control_mode = str(item.get('mode') or '')
        status.armed = bool(item.get('armed', False))

        active_command_id = str(
            item.get('active_command_id') or ''
        )
        status.active_command_id = active_command_id
        status.operation_state = (
            'ACTIVE' if active_command_id else 'READY'
        )

        try:
            battery = float(item.get('battery_percent', 0.0))
            if math.isfinite(battery):
                status.battery_percent = battery
        except (TypeError, ValueError):
            pass

        status.health = 'OK' if fresh else 'STALE'

        vehicle_type = int(
            item.get('vehicle_type', 0) or 0
        )
        is_uav = (
            vehicle_type == 1
            or str(device_code).startswith('uav_')
        )

        if is_uav:
            if not fresh:
                status.flight_state = 'UNKNOWN'
            elif status.armed:
                status.flight_state = 'AIRBORNE'
            else:
                status.flight_state = 'GROUNDED'
        else:
            status.flight_state = 'NOT_APPLICABLE'

        return status

    def _broadcast_device_status_v1(self):
        if not bool(
                self.get_parameter(
                    'enable_device_status_v1').value):
            return

        if self.websocket is None or not self.websocket.running:
            return

        device_codes = (
            'uav_01', 'uav_02', 'uav_03',
            'usv_01', 'usv_02', 'usv_03',
        )

        for device_code in device_codes:
            status = self._device_status_v1(device_code)
            if status is None:
                continue

            payload = self.protocol_v1.dumps(
                'device.status',
                'fleet.device_status.%s' % device_code,
                status,
            )
            self.websocket.broadcast_path(
                '/uav_usv/v1',
                payload,
                priority=1,
                opcode=0x2,
            )

    def _broadcast_pose_batch_v1(self):
        if not bool(self.get_parameter('enable_pose_batch_v1').value):
            return
        if self.websocket is None or not self.websocket.running:
            return
        payload = self.protocol_v1.dumps(
            'telemetry.pose_batch',
            'fleet.pose',
            self._pose_batch_v1(),
        )
        self.websocket.broadcast_path(
            '/uav_usv/v1', payload, priority=1, opcode=0x2)

    def _broadcast_heartbeat_v1(self):
        if self.websocket is None or not self.websocket.running:
            return
        payload = self.protocol_v1.dumps(
            'gateway.heartbeat',
            'gateway.lifecycle',
            self._heartbeat_v1(),
        )
        self.websocket.broadcast_path(
            '/uav_usv/v1', payload, priority=2, opcode=0x2)

    def _broadcast_diagnostics(self):
        self._broadcast(
            'gateway_diagnostics', self._diagnostics(), priority=1)

    def _diagnostics(self):
        vehicles = self.registry.vehicles()
        targets = self.registry.targets()
        clients = self.websocket.client_count if self.websocket else 0
        diagnostics = self.health.snapshot(clients, vehicles, targets)
        diagnostics['communication_profile'] = self._communication_profile()
        diagnostics['base_station_state_available'] = bool(
            self.latest_base_station_state)
        diagnostics['base_station_event_cache_size'] = len(
            self.recent_base_station_events)
        return diagnostics

    def _snapshot(self):
        snapshot = self.registry.snapshot(
            self._diagnostics(),
            include_world_model=bool(
                self.get_parameter('include_world_model_in_snapshot').value
            ),
        )
        snapshot['base_station_state'] = self.latest_base_station_state
        snapshot['recent_base_station_events'] = list(
            self.recent_base_station_events)
        return snapshot

    def _communication_profile(self):
        return communication_profile(
            self.get_parameter('enable_world_model_publish').value,
            self.get_parameter('enable_world_model_summary_publish').value,
            self.get_parameter('include_world_model_in_snapshot').value,
            self.get_parameter('world_model_publish_rate_hz').value,
            self.get_parameter('world_model_summary_publish_rate_hz').value,
            self.get_parameter('world_model_topic').value,
            self.get_parameter('world_model_summary_topic').value,
        )

    def _hello(self):
        return {
            'gateway_name': self.gateway_name,
            'fleet_id': str(self.get_parameter('fleet_id').value),
            'protocol_version': '1.0',
            'primary_data_source': (
                'base_station_service'
                if bool(self.get_parameter(
                    'enable_base_station_service').value)
                else 'fleet_world_model'
            ),
            'communication_profile': self._communication_profile(),
            'command_interface': {
                # Legacy JSON /ws remains intentionally read-only.
                'enabled': False,
                'transport': 'legacy_json_websocket',
                'future_ros_output': '/fleet/command',
                'future_ack_input': '/fleet/command_ack',
                'accepted_commands': [
                    'submit_task',
                    'cancel_task',
                    'emergency_stop',
                ],
                'note': (
                    'Legacy /ws is read-only. Use Protobuf v1 '
                    '/uav_usv/v1 for bidirectional control.'
                ),
                'protobuf_v1_control': {
                    'enabled': True,
                    'websocket_path': '/uav_usv/v1',
                    'ros_output': str(self.get_parameter(
                        'fleet_command_topic').value),
                    'ack_input': str(self.get_parameter(
                        'command_ack_topic').value),
                    'lease_input': str(self.get_parameter(
                        'control_lease_topic').value),
                },
            },
            'use_sim_time': bool(self.get_parameter('use_sim_time').value),
            'websocket_path': str(
                self.get_parameter('websocket_path').value),
            'read_only': True,
        }

    def _http_health(self):
        diagnostics = self._diagnostics()
        return {
            'status': 'ok',
            'websocket': bool(
                self.websocket is not None and self.websocket.running),
            'ros_node': True,
            'clients': diagnostics['connected_clients'],
        }

    def _print_startup(self):
        bind = str(self.get_parameter('bind_address').value)
        addresses = (
            [bind] if bind not in ('0.0.0.0', '::') else lan_addresses()
        )
        self.get_logger().info(
            'Fleet Gateway started: legacy /ws read-only; '
            'Protobuf v1 /uav_usv/v1 bidirectional control enabled'
        )
        self.get_logger().info('use_sim_time=%s' % bool(
            self.get_parameter('use_sim_time').value))
        self.get_logger().info('Subscriptions: %s' % json.dumps(
            self.topic_summary, ensure_ascii=False))
        if not addresses:
            self.get_logger().info(
                'Run `hostname -I` to find the LAN address.')
            addresses = ['<LAN-IP>']
        for address in addresses:
            if self.http:
                self.get_logger().info(
                    'HTTP: http://%s:%d' % (address, self.http.port))
            if self.websocket:
                self.get_logger().info(
                    'WebSocket: ws://%s:%d%s' % (
                        address, self.websocket.port, self.websocket.path))

    def destroy_node(self):
        self._push_stop.set()
        if self._push_thread is not None:
            self._push_thread.join(timeout=2.0)
            self._push_thread = None
        if self.http is not None:
            self.http.stop()
        if self.websocket is not None:
            self.websocket.stop()
        self.health.websocket_running = False
        try:
            return super().destroy_node()
        except KeyboardInterrupt:
            return None


def main(args=None):
    ros1.init(args=args)
    node = FleetGatewayNode()
    try:
        ros1.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ros1.ok():
            ros1.shutdown()


if __name__ == '__main__':
    main()
