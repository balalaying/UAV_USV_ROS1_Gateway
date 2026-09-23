#!/usr/bin/env python3
"""State machine and dynamic dispatcher for scalable fleet capture."""

import json
import math
import time
import uuid

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.parameter_interfaces import SetParametersResult
from uav_usv_ros1_compat.executors import ExternalShutdownException
from uav_usv_ros1_compat.node import Node
from uav_usv_ros1_compat.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_usv_ros1_compat.qos import qos_profile_sensor_data
from std_msgs.msg import String
from uav_usv_interfaces.msg import CaptureAssignment as CaptureAssignmentMsg
from uav_usv_interfaces.msg import CaptureAssignmentArray
from uav_usv_interfaces.msg import CaptureState
from uav_usv_interfaces.msg import CaptureTargetStatus
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState
from uav_usv_mission.capture_planner import CapturePlanner
from uav_usv_mission.capture_planner import VehicleKinematics
from uav_usv_mission.target_predictor import TargetPredictor
from uav_usv_mission.target_predictor import TargetState


class CaptureManager(Node):
    SEARCH = 'SEARCH'
    TRACKING = 'TRACKING'
    APPROACHING = 'APPROACHING'
    ENCIRCLING = 'ENCIRCLING'
    HOLDING = 'HOLDING'
    SUCCESS = 'SUCCESS'
    FAILED = 'FAILED'

    STATE_CODES = {
        SEARCH: CaptureState.STATE_SEARCH,
        TRACKING: CaptureState.STATE_TRACKING,
        APPROACHING: CaptureState.STATE_APPROACHING,
        ENCIRCLING: CaptureState.STATE_ENCIRCLING,
        HOLDING: CaptureState.STATE_HOLDING,
        SUCCESS: CaptureState.STATE_SUCCESS,
        FAILED: CaptureState.STATE_FAILED,
    }

    def __init__(self):
        super().__init__('capture_manager')
        self.declare_parameter('uav_id', 'uav_01')
        self.declare_parameter('uav_ids', [''])
        self.declare_parameter('usv_id', 'usv_01')
        self.declare_parameter('usv_ids', [''])
        self.declare_parameter('target_id', 'target_vessel')
        self.declare_parameter('takeoff_altitude', 18.0)
        self.declare_parameter('uav_home_z', 1.35)
        self.declare_parameter('observation_altitude', 22.0)
        self.declare_parameter('capture_radius', 18.0)
        self.declare_parameter('prediction_horizon', 12.0)
        self.declare_parameter('prediction_step', 1.0)
        self.declare_parameter('uav_prediction_time', 2.5)
        self.declare_parameter('usv_prediction_time', 9.0)
        self.declare_parameter('command_period', 5.0)
        self.declare_parameter('command_move_threshold', 3.0)
        self.declare_parameter('target_timeout', 2.0)
        self.declare_parameter('vehicle_state_timeout', 2.0)
        self.declare_parameter('fleet_ready_timeout', 35.0)
        self.declare_parameter('tracking_confirmations', 3)
        self.declare_parameter('encircle_tolerance', 28.0)
        self.declare_parameter('holding_tolerance', 16.0)
        self.declare_parameter('success_duration', 5.0)
        self.declare_parameter('max_takeoff_attempts', 3)
        self.declare_parameter('command_failure_threshold', 3)
        self.declare_parameter('minimum_uavs', 1)
        self.declare_parameter('minimum_usvs', 1)
        self.declare_parameter('excluded_vehicle_ids', [''])
        self.declare_parameter('auto_start', True)

        legacy_uav = str(self.get_parameter('uav_id').value)
        configured_uavs = self._string_list('uav_ids')
        legacy_usv = str(self.get_parameter('usv_id').value)
        configured_usvs = self._string_list('usv_ids')
        self.uav_ids = configured_uavs or [legacy_uav]
        self.usv_ids = configured_usvs or [legacy_usv]
        self.vehicle_ids = self.uav_ids + self.usv_ids
        if len(set(self.vehicle_ids)) != len(self.vehicle_ids):
            raise ValueError('configured vehicle IDs must be unique')

        self.target_id = str(self.get_parameter('target_id').value)
        self.takeoff_altitude = self._float_parameter('takeoff_altitude')
        self.uav_home_z = self._float_parameter('uav_home_z')
        self.command_period = self._float_parameter('command_period')
        self.command_move_threshold = self._float_parameter(
            'command_move_threshold'
        )
        self.target_timeout = self._float_parameter('target_timeout')
        self.vehicle_state_timeout = self._float_parameter(
            'vehicle_state_timeout'
        )
        self.fleet_ready_timeout = self._float_parameter(
            'fleet_ready_timeout'
        )
        self.tracking_confirmations = int(
            self.get_parameter('tracking_confirmations').value
        )
        self.encircle_tolerance = self._float_parameter(
            'encircle_tolerance'
        )
        self.holding_tolerance = self._float_parameter('holding_tolerance')
        self.success_duration = self._float_parameter('success_duration')
        self.max_takeoff_attempts = int(
            self.get_parameter('max_takeoff_attempts').value
        )
        self.command_failure_threshold = int(
            self.get_parameter('command_failure_threshold').value
        )
        self.minimum_uavs = int(self.get_parameter('minimum_uavs').value)
        self.minimum_usvs = int(self.get_parameter('minimum_usvs').value)
        self.excluded_vehicle_ids = set(
            self._string_list('excluded_vehicle_ids')
        )
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.add_on_set_parameters_callback(self._on_parameters)

        self.predictor = TargetPredictor(
            horizon=self._float_parameter('prediction_horizon'),
            step=self._float_parameter('prediction_step'),
        )
        self.planner = CapturePlanner(
            capture_radius=self._float_parameter('capture_radius'),
            observation_altitude=self._float_parameter(
                'observation_altitude'
            ),
            uav_prediction_time=self._float_parameter(
                'uav_prediction_time'
            ),
            usv_prediction_time=self._float_parameter(
                'usv_prediction_time'
            ),
        )

        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.lease_pub = self.create_publisher(
            ControlLease, '/fleet/control_lease', lease_qos
        )
        self.command_pub = self.create_publisher(
            FleetCommand, '/fleet/command', 60
        )
        self.uav_point_pub = self.create_publisher(
            PoseStamped, '/capture/uav_observation_point', 10
        )
        self.usv_point_pub = self.create_publisher(
            PoseStamped, '/capture/usv_intercept_point', 10
        )
        self.assignments_pub = self.create_publisher(
            PoseArray, '/capture/assignment_points', 10
        )
        self.prediction_pub = self.create_publisher(
            Path, '/capture/target_prediction', 10
        )
        self.roles_pub = self.create_publisher(
            CaptureAssignmentArray, '/capture/roles', 10
        )
        self.state_pub = self.create_publisher(
            CaptureState, '/capture/state', 10
        )
        self.target_status_pub = self.create_publisher(
            CaptureTargetStatus, '/capture/target_status', 10
        )
        self.roles_json_pub = self.create_publisher(
            String, '/capture/roles_json', 10
        )
        self.state_text_pub = self.create_publisher(
            String, '/capture/state_text', 10
        )
        self.target_json_pub = self.create_publisher(
            String, '/capture/target_status_json', 10
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
            CommandAck, '/fleet/command_ack', self._on_ack, 60
        )
        self.create_subscription(
            String,
            '/fleet/base/operator_action',
            self._on_operator_action,
            10,
        )

        self.started_at = time.monotonic()
        self.lease_id = 'capture-' + uuid.uuid4().hex[:12]
        self.target = None
        self.last_target_rx = 0.0
        self.target_confirmations = 0
        self.vehicle_states = {}
        self.last_vehicle_rx = {}
        self.quarantined_vehicles = {}
        self.state = self.SEARCH
        self.state_reason = 'waiting for target track'
        self.state_entered = time.monotonic()
        self.holding_since = None
        self.mission_started = False
        self.start_requested = self.auto_start
        self.paused = False
        self.takeoff_commands = {}
        self.takeoff_attempts = {
            vehicle_id: 0 for vehicle_id in self.uav_ids
        }
        self.last_takeoff_attempt = {
            vehicle_id: 0.0 for vehicle_id in self.uav_ids
        }
        self.airborne_uavs = set()
        self.command_failures = {
            vehicle_id: 0 for vehicle_id in self.vehicle_ids
        }
        self.last_command_time = 0.0
        self.last_points = {}
        self.current_prediction = []
        self.current_plan = None
        self.previous_roles = {}
        self.assignment_signature = ()
        self.allocation_generation = 0
        self.active_uav_ids = []
        self.active_usv_ids = []
        self.create_timer(0.5, self._publish_lease)
        self.create_timer(0.2, self._update)
        self.get_logger().info(
            'Capture manager ready: UAVs=%s USVs=%s target=%s auto_start=%s'
            % (
                ','.join(self.uav_ids), ','.join(self.usv_ids),
                self.target_id, self.auto_start,
            )
        )

    def _string_list(self, name):
        return [
            str(value) for value in self.get_parameter(name).value
            if str(value)
        ]

    def _float_parameter(self, name):
        return float(self.get_parameter(name).value)

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'excluded_vehicle_ids':
                requested = {str(value) for value in parameter.value if str(value)}
                unknown = requested.difference(self.vehicle_ids)
                if unknown:
                    return SetParametersResult(
                        successful=False,
                        reason='unknown vehicle IDs: ' + ','.join(sorted(unknown)),
                    )
                self.excluded_vehicle_ids = requested
                self.get_logger().warning(
                    'Runtime vehicle exclusions: %s'
                    % (','.join(sorted(requested)) or 'none')
                )
        return SetParametersResult(successful=True)

    @staticmethod
    def _yaw(orientation):
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _set_state(self, new_state, reason):
        self.state_reason = reason
        if self.state == new_state:
            return
        self.get_logger().info(
            'Capture state %s -> %s: %s' % (self.state, new_state, reason)
        )
        self.state = new_state
        self.state_entered = time.monotonic()
        if new_state != self.HOLDING:
            self.holding_since = None

    def _on_targets(self, msg):
        target = next(
            (obj for obj in msg.objects if obj.track_id == self.target_id),
            None,
        )
        if target is None:
            return
        self.target = target
        self.last_target_rx = time.monotonic()
        self.target_confirmations += 1
        if self.state == self.SEARCH:
            self._set_state(self.TRACKING, 'target track acquired')

    def _on_vehicle_state(self, msg):
        if msg.vehicle_id in self.vehicle_ids:
            self.vehicle_states[msg.vehicle_id] = msg
            self.last_vehicle_rx[msg.vehicle_id] = time.monotonic()

    def _on_ack(self, msg):
        if msg.vehicle_id not in self.vehicle_ids:
            return
        if msg.status != CommandAck.STATUS_EXECUTING:
            self.get_logger().info(
                'ACK %s %s status=%d: %s'
                % (msg.vehicle_id, msg.command_id, msg.status, msg.message)
            )
        expected_takeoff = self.takeoff_commands.get(msg.vehicle_id)
        if msg.command_id == expected_takeoff:
            if msg.status == CommandAck.STATUS_SUCCEEDED:
                self.airborne_uavs.add(msg.vehicle_id)
                self.takeoff_commands.pop(msg.vehicle_id, None)
            elif msg.status in (
                CommandAck.STATUS_REJECTED,
                CommandAck.STATUS_FAILED,
                CommandAck.STATUS_CANCELED,
            ):
                self.takeoff_commands.pop(msg.vehicle_id, None)

        if msg.status in (
            CommandAck.STATUS_REJECTED,
            CommandAck.STATUS_FAILED,
        ):
            self.command_failures[msg.vehicle_id] += 1
            if (
                self.command_failures[msg.vehicle_id]
                >= self.command_failure_threshold
            ):
                self.quarantined_vehicles[msg.vehicle_id] = (
                    'repeated command failure: ' + msg.message
                )
                self.get_logger().error(
                    'Quarantined %s after command failures'
                    % msg.vehicle_id
                )
        elif msg.status == CommandAck.STATUS_SUCCEEDED:
            self.command_failures[msg.vehicle_id] = 0

    def _on_operator_action(self, msg):
        action = msg.data.strip()
        upper = action.upper()
        if upper.startswith('CAPTURE:'):
            requested_target = action.split(':', 1)[1].strip()
            if requested_target and requested_target != self.target_id:
                self.get_logger().warning(
                    'Ignoring capture request for unknown target %s'
                    % requested_target
                )
                return
            self.start_requested = True
            self.paused = False
            self.started_at = time.monotonic()
            self._set_state(self.TRACKING, 'operator approved capture')
            self.get_logger().warning(
                'Operator started capture; ArduPilot takeoff is now enabled'
            )
        elif upper == 'HOLD_ALL':
            self.paused = True
            self._hold_active_fleet()
            self._set_state(self.TRACKING, 'capture paused by operator')
        elif upper == 'CANCEL_CAPTURE':
            self.start_requested = False
            self.paused = False
            self.mission_started = False
            self._hold_active_fleet()
            self.last_points.clear()
            self._set_state(self.TRACKING, 'waiting for operator start')

    def _hold_active_fleet(self):
        for vehicle_id in self.active_uav_ids + self.active_usv_ids:
            self.command_pub.publish(self._new_command(
                vehicle_id, FleetCommand.COMMAND_HOLD
            ))

    def _publish_lease(self):
        msg = ControlLease()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vehicle_id = '*'
        msg.lease_id = self.lease_id
        msg.owner_id = 'capture_manager'
        msg.priority = 100
        valid_until = self.get_clock().now() + ros1.duration.Duration(
            seconds=2.0
        )
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
        expires = self.get_clock().now() + ros1.duration.Duration(
            seconds=lifetime
        )
        msg.expires_at = expires.to_msg()
        msg.target_pose.orientation.w = 1.0
        return msg

    def _send_takeoff(self, vehicle_id):
        msg = self._new_command(
            vehicle_id, FleetCommand.COMMAND_TAKEOFF, lifetime=15.0
        )
        msg.parameters = [self.takeoff_altitude]
        self.takeoff_commands[vehicle_id] = msg.command_id
        self.takeoff_attempts[vehicle_id] += 1
        self.last_takeoff_attempt[vehicle_id] = time.monotonic()
        self.command_pub.publish(msg)
        self.get_logger().info(
            'Sent ArduPilot takeoff command %s to %s'
            % (msg.command_id, vehicle_id)
        )

    def _vehicle_available(self, vehicle_id, now):
        if vehicle_id in self.excluded_vehicle_ids:
            return False
        if vehicle_id in self.quarantined_vehicles:
            return False
        state = self.vehicle_states.get(vehicle_id)
        received = self.last_vehicle_rx.get(vehicle_id, 0.0)
        return (
            state is not None
            and state.online
            and now - received <= self.vehicle_state_timeout
        )

    def _refresh_active_fleet(self, now):
        self.active_uav_ids = [
            vehicle_id for vehicle_id in self.uav_ids
            if self._vehicle_available(vehicle_id, now)
        ]
        self.active_usv_ids = [
            vehicle_id for vehicle_id in self.usv_ids
            if self._vehicle_available(vehicle_id, now)
        ]

    def _refresh_airborne_states(self):
        minimum_z = self.uav_home_z + self.takeoff_altitude - 1.0
        for vehicle_id in self.active_uav_ids:
            state = self.vehicle_states.get(vehicle_id)
            if (
                state is not None
                and state.armed
                and state.pose.position.z >= minimum_z
                and state.mode in ('GUIDED', 'BRAKE', 'LOITER')
            ):
                self.airborne_uavs.add(vehicle_id)

    def _target_state(self):
        pose = self.target.pose.pose
        twist = self.target.twist.twist
        return TargetState(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            vx=float(twist.linear.x),
            vy=float(twist.linear.y),
            vz=float(twist.linear.z),
            yaw=self._yaw(pose.orientation),
            yaw_rate=float(twist.angular.z),
        )

    def _planner_vehicles(self):
        result = []
        for vehicle_id in self.active_uav_ids + self.active_usv_ids:
            state = self.vehicle_states[vehicle_id]
            result.append(VehicleKinematics(
                vehicle_id=vehicle_id,
                vehicle_type=int(state.vehicle_type),
                x=float(state.pose.position.x),
                y=float(state.pose.position.y),
                z=float(state.pose.position.z),
                vx=float(state.twist.linear.x),
                vy=float(state.twist.linear.y),
                yaw=self._yaw(state.pose.orientation),
            ))
        return result

    def _assignment_pose(self, assignment):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = assignment.x
        pose.pose.position.y = assignment.y
        pose.pose.position.z = assignment.z
        pose.pose.orientation.w = 1.0
        return pose

    def _inactive_reason(self, vehicle_id, now):
        if vehicle_id in self.excluded_vehicle_ids:
            return 'excluded by runtime parameter'
        if vehicle_id in self.quarantined_vehicles:
            return self.quarantined_vehicles[vehicle_id]
        if vehicle_id not in self.vehicle_states:
            return 'no VehicleState received'
        if now - self.last_vehicle_rx.get(vehicle_id, 0.0) > self.vehicle_state_timeout:
            return 'VehicleState timeout'
        if not self.vehicle_states[vehicle_id].online:
            return 'vehicle reports offline'
        return 'not assigned'

    def _update_allocation_generation(self):
        signature = tuple(sorted(
            (vehicle_id, assignment.role)
            for vehicle_id, assignment in self.current_plan.assignments.items()
        ))
        if signature != self.assignment_signature:
            self.assignment_signature = signature
            self.allocation_generation += 1
            self.previous_roles = {
                vehicle_id: assignment.role
                for vehicle_id, assignment in self.current_plan.assignments.items()
            }
            self.last_points = {
                key: value for key, value in self.last_points.items()
                if key in self.current_plan.assignments
            }
            if self.mission_started:
                self._set_state(
                    self.APPROACHING,
                    'fleet availability changed; assignments regenerated',
                )
            self.get_logger().warning(
                'Allocation generation %d: %s'
                % (
                    self.allocation_generation,
                    ', '.join('%s=%s' % pair for pair in signature),
                )
            )

    def _publish_plan(self, target_state):
        now_msg = self.get_clock().now().to_msg()
        prediction = Path()
        prediction.header.stamp = now_msg
        prediction.header.frame_id = 'map'
        for point in self.current_prediction:
            pose = PoseStamped()
            pose.header = prediction.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z + 0.8
            pose.pose.orientation.z = math.sin(point.yaw * 0.5)
            pose.pose.orientation.w = math.cos(point.yaw * 0.5)
            prediction.poses.append(pose)
        self.prediction_pub.publish(prediction)

        pose_array = PoseArray()
        pose_array.header = prediction.header
        roles = CaptureAssignmentArray()
        roles.header = prediction.header
        roles.target_id = self.target_id
        roles.capture_center.x = self.current_plan.center_x
        roles.capture_center.y = self.current_plan.center_y
        roles.capture_center.z = target_state.z
        roles.capture_radius = float(self.current_plan.capture_radius)
        roles.generation = self.allocation_generation

        first_uav = True
        first_usv = True
        active_metadata = []
        for vehicle_id in self.vehicle_ids:
            assignment = self.current_plan.assignments.get(vehicle_id)
            item = CaptureAssignmentMsg()
            item.vehicle_id = vehicle_id
            if assignment is None:
                state = self.vehicle_states.get(vehicle_id)
                item.vehicle_type = (
                    int(state.vehicle_type) if state is not None
                    else (
                        VehicleState.TYPE_UAV
                        if vehicle_id in self.uav_ids
                        else VehicleState.TYPE_USV
                    )
                )
                item.role_type = CaptureAssignmentMsg.ROLE_UNASSIGNED
                item.role_name = 'unassigned'
                item.active = False
                item.status = self._inactive_reason(vehicle_id, time.monotonic())
                roles.assignments.append(item)
                continue

            point = self._assignment_pose(assignment)
            pose_array.poses.append(point.pose)
            item.vehicle_type = assignment.vehicle_type
            item.role_type = assignment.role_type
            item.role_name = assignment.role
            item.target_pose = point.pose
            item.assignment_cost = float(assignment.cost)
            item.active = True
            item.status = 'assigned'
            roles.assignments.append(item)
            active_metadata.append({
                'index': len(pose_array.poses) - 1,
                'vehicle_id': vehicle_id,
                'role': assignment.role,
                'cost': round(assignment.cost, 3),
            })
            if assignment.vehicle_type == VehicleState.TYPE_UAV and first_uav:
                self.uav_point_pub.publish(point)
                first_uav = False
            if assignment.vehicle_type == VehicleState.TYPE_USV and first_usv:
                self.usv_point_pub.publish(point)
                first_usv = False
        self.assignments_pub.publish(pose_array)
        self.roles_pub.publish(roles)

        roles_json = String()
        roles_json.data = json.dumps({
            'state': self.state,
            'target_id': self.target_id,
            'capture_radius': self.current_plan.capture_radius,
            'center': [self.current_plan.center_x, self.current_plan.center_y],
            'generation': self.allocation_generation,
            'assignments': active_metadata,
        }, separators=(',', ':'))
        self.roles_json_pub.publish(roles_json)

        target_status = CaptureTargetStatus()
        target_status.header = prediction.header
        target_status.track_id = self.target_id
        target_status.tracked = True
        target_status.confirmations = self.target_confirmations
        target_status.pose = self.target.pose.pose
        target_status.twist = self.target.twist.twist
        target_status.speed_mps = float(target_state.speed)
        target_status.turn_rate_rps = float(target_state.yaw_rate)
        target_status.track_age_s = float(
            max(0.0, time.monotonic() - self.last_target_rx)
        )
        target_status.prediction_model = TargetPredictor.MODEL_NAME
        self.target_status_pub.publish(target_status)

        target_json = String()
        target_json.data = json.dumps({
            'track_id': self.target_id,
            'tracked': True,
            'confirmations': self.target_confirmations,
            'position': [target_state.x, target_state.y, target_state.z],
            'velocity': [target_state.vx, target_state.vy, target_state.vz],
            'speed': target_state.speed,
            'turn_rate': target_state.yaw_rate,
            'prediction_model': TargetPredictor.MODEL_NAME,
        }, separators=(',', ':'))
        self.target_json_pub.publish(target_json)

    def _point_moved(self, vehicle_id, assignment):
        previous = self.last_points.get(vehicle_id)
        if previous is None:
            return True
        return math.hypot(
            assignment.x - previous[0], assignment.y - previous[1]
        ) >= self.command_move_threshold

    def _dispatch_plan(self):
        now = time.monotonic()
        if now - self.last_command_time < self.command_period:
            return
        for vehicle_id, assignment in self.current_plan.assignments.items():
            if not self._point_moved(vehicle_id, assignment):
                continue
            msg = self._new_command(
                vehicle_id, FleetCommand.COMMAND_NAVIGATE
            )
            msg.target_pose.position.x = assignment.x
            msg.target_pose.position.y = assignment.y
            msg.target_pose.position.z = assignment.z
            self.command_pub.publish(msg)
            self.last_points[vehicle_id] = (assignment.x, assignment.y)
        self.last_command_time = now

    def _maximum_assignment_error(self):
        errors = []
        for vehicle_id, assignment in self.current_plan.assignments.items():
            state = self.vehicle_states.get(vehicle_id)
            if state is None:
                return math.inf
            errors.append(math.sqrt(
                (state.pose.position.x - assignment.x) ** 2
                + (state.pose.position.y - assignment.y) ** 2
                + (state.pose.position.z - assignment.z) ** 2
            ))
        return max(errors, default=math.inf)

    def _update_capture_state(self, maximum_error):
        now = time.monotonic()
        if self.state == self.APPROACHING:
            if maximum_error <= self.encircle_tolerance:
                self._set_state(
                    self.ENCIRCLING, 'all active vehicles entered capture area'
                )
        if self.state == self.ENCIRCLING:
            if maximum_error <= self.holding_tolerance:
                self._set_state(
                    self.HOLDING, 'all active roles reached assigned sectors'
                )
                self.holding_since = now
        elif self.state == self.HOLDING:
            if maximum_error > self.encircle_tolerance:
                self._set_state(self.ENCIRCLING, 'formation error increased')
            elif self.holding_since is None:
                self.holding_since = now
            elif now - self.holding_since >= self.success_duration:
                self._set_state(
                    self.SUCCESS, 'capture geometry held continuously'
                )

    def _publish_state(self, status_text):
        msg = CaptureState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.state = self.STATE_CODES[self.state]
        msg.state_name = self.state
        msg.target_id = self.target_id
        msg.reason = self.state_reason
        msg.configured_uavs = len(self.uav_ids)
        msg.configured_usvs = len(self.usv_ids)
        msg.active_uavs = len(self.active_uav_ids)
        msg.active_usvs = len(self.active_usv_ids)
        msg.allocation_generation = self.allocation_generation
        msg.degraded = (
            len(self.active_uav_ids) < len(self.uav_ids)
            or len(self.active_usv_ids) < len(self.usv_ids)
        )
        self.state_pub.publish(msg)

        state_text = String()
        state_text.data = self.state
        self.state_text_pub.publish(state_text)
        status = String()
        status.data = self.state + ': ' + status_text
        self.status_pub.publish(status)

    def _manage_takeoff(self, now):
        self._refresh_airborne_states()
        for vehicle_id in self.active_uav_ids:
            if vehicle_id in self.airborne_uavs:
                continue
            if self.takeoff_attempts[vehicle_id] >= self.max_takeoff_attempts:
                self.quarantined_vehicles[vehicle_id] = (
                    'ArduPilot takeoff retries exceeded'
                )
                continue
            command_pending = vehicle_id in self.takeoff_commands
            timed_out = now - self.last_takeoff_attempt[vehicle_id] > 22.0
            if command_pending and not timed_out:
                continue
            if command_pending:
                self.takeoff_commands.pop(vehicle_id, None)
            if now - self.last_takeoff_attempt[vehicle_id] >= 2.0:
                self._send_takeoff(vehicle_id)

    def _fleet_has_minimum(self):
        airborne_active = [
            item for item in self.active_uav_ids
            if item in self.airborne_uavs
        ]
        return (
            len(airborne_active) >= self.minimum_uavs
            and len(self.active_usv_ids) >= self.minimum_usvs
        )

    def _fleet_ready_to_start(self, now):
        all_uavs = (
            len(self.active_uav_ids) == len(self.uav_ids)
            and all(item in self.airborne_uavs for item in self.uav_ids)
        )
        all_usvs = len(self.active_usv_ids) == len(self.usv_ids)
        if all_uavs and all_usvs:
            return True
        return (
            now - self.started_at >= self.fleet_ready_timeout
            and self._fleet_has_minimum()
        )

    def _update(self):
        now = time.monotonic()
        self._refresh_active_fleet(now)
        target_fresh = (
            self.target is not None
            and now - self.last_target_rx <= self.target_timeout
        )
        if not target_fresh:
            self._set_state(self.SEARCH, 'waiting for a fresh target track')
            self._publish_state('waiting for target track')
            return

        if self.target_confirmations < self.tracking_confirmations:
            self._set_state(self.TRACKING, 'confirming target track')
            self._publish_state('confirming target track')
            return

        if not self.start_requested:
            self._set_state(self.TRACKING, 'waiting for operator start')
            self._publish_state(
                'target confirmed; press Start Capture in the Qt console'
            )
            return

        if self.paused:
            self._set_state(self.TRACKING, 'capture paused by operator')
            self._publish_state('fleet holding; press Continue in Qt')
            return

        self._manage_takeoff(now)
        self._refresh_active_fleet(now)
        self._refresh_airborne_states()
        if not self.mission_started:
            if not self._fleet_ready_to_start(now):
                self._set_state(self.TRACKING, 'waiting for configured fleet')
                self._publish_state(
                    'airborne UAVs %d/%d, online USVs %d/%d'
                    % (
                        len(self.airborne_uavs.intersection(self.active_uav_ids)),
                        len(self.uav_ids),
                        len(self.active_usv_ids),
                        len(self.usv_ids),
                    )
                )
                return
            self.mission_started = True

        if not self._fleet_has_minimum():
            self._set_state(
                self.TRACKING, 'insufficient active vehicles for capture'
            )
            self._publish_state('waiting for replacement vehicles')
            return

        active_airborne = [
            item for item in self.active_uav_ids
            if item in self.airborne_uavs
        ]
        self.active_uav_ids = active_airborne
        target_state = self._target_state()
        self.current_prediction = self.predictor.predict(target_state)
        self.current_plan = self.planner.plan(
            target_state,
            self.current_prediction,
            self._planner_vehicles(),
            self.previous_roles,
        )
        self._update_allocation_generation()
        if self.state in (self.SEARCH, self.TRACKING):
            self._set_state(
                self.APPROACHING, 'active fleet ready; plan dispatched'
            )
        self._publish_plan(target_state)
        self._dispatch_plan()
        maximum_error = self._maximum_assignment_error()
        self._update_capture_state(maximum_error)
        self._publish_state(
            'CTRV target tracked; max assignment error %.1f m'
            % maximum_error
        )


def main(args=None):
    ros1.init(args=args)
    node = CaptureManager()
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
