#!/usr/bin/env python3
"""State machine and task dispatcher for scalable dynamic capture."""

import json
import math
import time
import uuid

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
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
from uav_usv_mission.capture_planner import CapturePlanner
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
        self.declare_parameter('tracking_confirmations', 3)
        self.declare_parameter('encircle_tolerance', 28.0)
        self.declare_parameter('holding_tolerance', 16.0)
        self.declare_parameter('success_duration', 5.0)
        self.declare_parameter('max_takeoff_attempts', 3)

        legacy_uav = str(self.get_parameter('uav_id').value)
        configured_uavs = [
            str(value) for value in self.get_parameter('uav_ids').value
            if str(value)
        ]
        legacy_usv = str(self.get_parameter('usv_id').value)
        configured_usvs = [
            str(value) for value in self.get_parameter('usv_ids').value
            if str(value)
        ]
        self.uav_ids = configured_uavs or [legacy_uav]
        self.usv_ids = configured_usvs or [legacy_usv]
        self.vehicle_ids = self.uav_ids + self.usv_ids
        self.target_id = str(self.get_parameter('target_id').value)
        self.takeoff_altitude = float(
            self.get_parameter('takeoff_altitude').value
        )
        self.uav_home_z = float(self.get_parameter('uav_home_z').value)
        self.command_period = float(
            self.get_parameter('command_period').value
        )
        self.command_move_threshold = float(
            self.get_parameter('command_move_threshold').value
        )
        self.target_timeout = float(
            self.get_parameter('target_timeout').value
        )
        self.tracking_confirmations = int(
            self.get_parameter('tracking_confirmations').value
        )
        self.encircle_tolerance = float(
            self.get_parameter('encircle_tolerance').value
        )
        self.holding_tolerance = float(
            self.get_parameter('holding_tolerance').value
        )
        self.success_duration = float(
            self.get_parameter('success_duration').value
        )
        self.max_takeoff_attempts = int(
            self.get_parameter('max_takeoff_attempts').value
        )

        self.predictor = TargetPredictor(
            horizon=float(self.get_parameter('prediction_horizon').value),
            step=float(self.get_parameter('prediction_step').value),
        )
        self.planner = CapturePlanner(
            capture_radius=float(self.get_parameter('capture_radius').value),
            observation_altitude=float(
                self.get_parameter('observation_altitude').value
            ),
            uav_prediction_time=float(
                self.get_parameter('uav_prediction_time').value
            ),
            usv_prediction_time=float(
                self.get_parameter('usv_prediction_time').value
            ),
        )

        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.lease_pub = self.create_publisher(
            ControlLease, '/fleet/control_lease', lease_qos
        )
        self.command_pub = self.create_publisher(
            FleetCommand, '/fleet/command', 30
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
            String, '/capture/roles', 10
        )
        self.state_pub = self.create_publisher(
            String, '/capture/state', 10
        )
        self.target_status_pub = self.create_publisher(
            String, '/capture/target_status', 10
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
            CommandAck, '/fleet/command_ack', self._on_ack, 30
        )

        self.lease_id = 'capture-' + uuid.uuid4().hex[:12]
        self.target = None
        self.last_target_rx = 0.0
        self.target_confirmations = 0
        self.vehicle_states = {}
        self.state = self.SEARCH
        self.state_entered = time.monotonic()
        self.holding_since = None
        self.takeoff_commands = {}
        self.takeoff_attempts = {vehicle_id: 0 for vehicle_id in self.uav_ids}
        self.last_takeoff_attempt = {
            vehicle_id: 0.0 for vehicle_id in self.uav_ids
        }
        self.airborne_uavs = set()
        self.command_vehicle = {}
        self.command_failures = {vehicle_id: 0 for vehicle_id in self.vehicle_ids}
        self.last_command_time = 0.0
        self.last_points = {}
        self.current_prediction = []
        self.current_plan = None
        self.create_timer(0.5, self._publish_lease)
        self.create_timer(0.2, self._update)
        self.get_logger().info(
            'Capture manager ready: UAVs=%s USVs=%s target=%s'
            % (','.join(self.uav_ids), ','.join(self.usv_ids), self.target_id)
        )

    def _set_state(self, new_state, reason):
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
            if self.command_failures[msg.vehicle_id] >= 5:
                self._set_state(
                    self.FAILED,
                    '%s reported repeated command failures' % msg.vehicle_id,
                )
        elif msg.status in (
            CommandAck.STATUS_ACCEPTED,
            CommandAck.STATUS_SUCCEEDED,
        ):
            self.command_failures[msg.vehicle_id] = 0

    def _publish_lease(self):
        msg = ControlLease()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vehicle_id = '*'
        msg.lease_id = self.lease_id
        msg.owner_id = 'capture_manager'
        msg.priority = 100
        valid_until = self.get_clock().now() + rclpy.duration.Duration(
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
        expires = self.get_clock().now() + rclpy.duration.Duration(
            seconds=lifetime
        )
        msg.expires_at = expires.to_msg()
        msg.target_pose.orientation.w = 1.0
        self.command_vehicle[msg.command_id] = vehicle_id
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
            'Sent PX4 takeoff command %s to %s'
            % (msg.command_id, vehicle_id)
        )

    def _refresh_airborne_states(self):
        minimum_z = self.uav_home_z + self.takeoff_altitude - 1.0
        for vehicle_id in self.uav_ids:
            state = self.vehicle_states.get(vehicle_id)
            if (
                state is not None
                and state.armed
                and state.pose.position.z >= minimum_z
                and state.mode in ('PX4/HOLD', 'PX4/NAVIGATE')
            ):
                self.airborne_uavs.add(vehicle_id)

    def _target_state(self):
        pose = self.target.pose.pose.position
        velocity = self.target.twist.twist.linear
        return TargetState(
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z),
            vx=float(velocity.x),
            vy=float(velocity.y),
            vz=float(velocity.z),
        )

    def _assignment_pose(self, assignment):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = assignment.x
        pose.pose.position.y = assignment.y
        pose.pose.position.z = assignment.z
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_plan(self, target_state):
        now = self.get_clock().now().to_msg()
        prediction = Path()
        prediction.header.stamp = now
        prediction.header.frame_id = 'map'
        for point in self.current_prediction:
            pose = PoseStamped()
            pose.header = prediction.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z + 0.8
            pose.pose.orientation.w = 1.0
            prediction.poses.append(pose)
        self.prediction_pub.publish(prediction)

        assignments = PoseArray()
        assignments.header = prediction.header
        metadata = []
        for index, vehicle_id in enumerate(self.vehicle_ids):
            assignment = self.current_plan.assignments.get(vehicle_id)
            if assignment is None:
                continue
            point = self._assignment_pose(assignment)
            assignments.poses.append(point.pose)
            metadata.append({
                'index': len(assignments.poses) - 1,
                'vehicle_id': vehicle_id,
                'role': assignment.role,
            })
            if vehicle_id == self.uav_ids[0]:
                self.uav_point_pub.publish(point)
            if vehicle_id == self.usv_ids[0]:
                self.usv_point_pub.publish(point)
        self.assignments_pub.publish(assignments)

        roles = String()
        roles.data = json.dumps({
            'state': self.state,
            'target_id': self.target_id,
            'capture_radius': self.current_plan.capture_radius,
            'center': [
                self.current_plan.center_x,
                self.current_plan.center_y,
            ],
            'assignments': metadata,
        }, separators=(',', ':'))
        self.roles_pub.publish(roles)

        target_status = String()
        target_status.data = json.dumps({
            'track_id': self.target_id,
            'tracked': True,
            'confirmations': self.target_confirmations,
            'position': [target_state.x, target_state.y, target_state.z],
            'velocity': [target_state.vx, target_state.vy, target_state.vz],
            'speed': math.hypot(target_state.vx, target_state.vy),
        }, separators=(',', ':'))
        self.target_status_pub.publish(target_status)

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
                self._set_state(self.ENCIRCLING, 'all vehicles entered capture area')
        if self.state == self.ENCIRCLING:
            if maximum_error <= self.holding_tolerance:
                self._set_state(self.HOLDING, 'all roles reached assigned sectors')
                self.holding_since = now
        elif self.state == self.HOLDING:
            if maximum_error > self.encircle_tolerance:
                self._set_state(self.ENCIRCLING, 'formation error increased')
            elif self.holding_since is None:
                self.holding_since = now
            elif now - self.holding_since >= self.success_duration:
                self._set_state(self.SUCCESS, 'capture geometry held continuously')

    def _publish_status(self, text):
        state = String()
        state.data = self.state
        self.state_pub.publish(state)
        status = String()
        status.data = self.state + ': ' + text
        self.status_pub.publish(status)

    def _update(self):
        now = time.monotonic()
        target_fresh = (
            self.target is not None
            and now - self.last_target_rx <= self.target_timeout
        )
        if not target_fresh:
            if self.state != self.FAILED:
                self._set_state(self.SEARCH, 'waiting for a fresh target track')
            self._publish_status('searching for target')
            return

        if self.state == self.FAILED:
            self._publish_status('manual reset required after repeated failures')
            return

        links_ready = all(
            vehicle_id in self.vehicle_states
            and self.vehicle_states[vehicle_id].online
            for vehicle_id in self.vehicle_ids
        )
        if (
            self.target_confirmations < self.tracking_confirmations
            or not links_ready
        ):
            self._set_state(self.TRACKING, 'validating target and vehicle links')
            self._publish_status('tracking target and waiting for all agents')
            return

        self._refresh_airborne_states()
        for vehicle_id in self.uav_ids:
            if vehicle_id in self.airborne_uavs:
                continue
            if self.takeoff_attempts[vehicle_id] >= self.max_takeoff_attempts:
                self._set_state(
                    self.FAILED, '%s exceeded takeoff retries' % vehicle_id
                )
                self._publish_status('PX4 takeoff failed')
                return
            command_pending = vehicle_id in self.takeoff_commands
            timed_out = (
                now - self.last_takeoff_attempt[vehicle_id] > 22.0
            )
            if command_pending and not timed_out:
                continue
            if command_pending:
                self.takeoff_commands.pop(vehicle_id, None)
            if now - self.last_takeoff_attempt[vehicle_id] >= 2.0:
                self._send_takeoff(vehicle_id)

        if len(self.airborne_uavs) != len(self.uav_ids):
            self._set_state(self.TRACKING, 'PX4 takeoff in progress')
            self._publish_status(
                'airborne UAVs %d/%d'
                % (len(self.airborne_uavs), len(self.uav_ids))
            )
            return

        target_state = self._target_state()
        self.current_prediction = self.predictor.predict(target_state)
        self.current_plan = self.planner.plan(
            target_state,
            self.current_prediction,
            self.uav_ids,
            self.usv_ids,
        )
        if self.state in (self.SEARCH, self.TRACKING):
            self._set_state(self.APPROACHING, 'all agents ready; plan dispatched')
        self._publish_plan(target_state)
        self._dispatch_plan()
        maximum_error = self._maximum_assignment_error()
        self._update_capture_state(maximum_error)
        self._publish_status(
            'target tracked; max assignment error %.1f m' % maximum_error
        )


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
