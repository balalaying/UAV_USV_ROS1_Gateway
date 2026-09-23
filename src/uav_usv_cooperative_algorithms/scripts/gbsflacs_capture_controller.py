#!/usr/bin/env python3
"""Live ROS 1 adapter for the supplied 3 UAV + 3 USV GBSFLACS planner.

The planner consumes live fleet/world-model state and only publishes bounded
COMMAND_NAVIGATE requests.  Arming, takeoff, leases, MAVLink and Gazebo motion
remain the responsibility of the existing base station and vehicle agents.
"""

import json
import math
import threading
import time

import numpy as np
import rospy
from std_msgs.msg import String
from uav_usv_interfaces.msg import CommandAck, ControlLease, FleetCommand, VehicleState

from uav_usv_cooperative_algorithms import gbsflacs_reference as core


TERMINAL_ACKS = {
    CommandAck.STATUS_SUCCEEDED,
    CommandAck.STATUS_REJECTED,
    CommandAck.STATUS_FAILED,
    CommandAck.STATUS_CANCELED,
}


class LiveEnvironment:
    """Small state contract required by the supplied GBSFLACSAlgorithm."""

    def __init__(self):
        self.agents = np.empty((0, 8), dtype=float)
        self.targets = np.empty((0, 6), dtype=float)
        self.permanently_captured = set()
        self.guarding_agents = {}
        self.previous_assignments = {}
        self.ball_tree = None
        self.all_balls = {}

    def _get_guarding_agent_ids(self):
        result = set()
        for identifiers in self.guarding_agents.values():
            result.update(int(identifier) for identifier in identifiers)
        return result


def _position(record):
    try:
        value = record['pose']['position']
        return np.asarray([
            float(value['x']), float(value['y']), float(value.get('z', 0.0))
        ], dtype=float)
    except (KeyError, TypeError, ValueError):
        return None


class GbsflacsCaptureController:
    def __init__(self):
        self.lock = threading.RLock()
        self.uav_ids = list(rospy.get_param('~uav_ids', [
            'uav_01', 'uav_02', 'uav_03'
        ]))
        self.usv_ids = list(rospy.get_param('~usv_ids', [
            'usv_01', 'usv_02', 'usv_03'
        ]))
        self.vehicle_ids = self.uav_ids + self.usv_ids
        if len(self.uav_ids) != 3 or len(self.usv_ids) != 3:
            raise ValueError('this integration requires exactly 3 UAV and 3 USV')

        self.target_id = str(rospy.get_param('~target_id', 'enemy_ship'))
        self.control_rate = max(0.2, min(2.0, float(
            rospy.get_param('~control_rate', 1.0))))
        self.state_timeout = max(0.5, float(rospy.get_param('~state_timeout', 3.0)))
        self.world_timeout = max(0.5, float(rospy.get_param('~world_timeout', 3.0)))
        self.capture_radius = max(5.0, float(rospy.get_param('~capture_radius', 50.0)))
        self.minimum_agents = max(1, min(6, int(
            rospy.get_param('~minimum_capture_agents', 5))))
        self.uav_step = max(2.0, min(9.0, float(
            rospy.get_param('~uav_command_step', 8.0))))
        self.usv_step = max(2.0, float(rospy.get_param('~usv_command_step', 12.0)))
        self.uav_min_altitude = float(rospy.get_param('~uav_min_altitude', 3.0))
        self.uav_max_altitude = float(rospy.get_param('~uav_max_altitude', 45.0))
        self.command_timeout = max(
            10.0, float(rospy.get_param('~command_timeout', 120.0)))
        self.seed = int(rospy.get_param('~seed', 42))
        self.active = bool(rospy.get_param('~auto_start', False))

        self.states = {}
        self.leases = {}
        self.acks = {}
        self.inflight = {}
        self.inflight_started = {}
        self.world = None
        self.world_received = 0.0
        self.counter = 0
        self.last_error = ''
        self.phase = 'WAITING_FOR_START' if not self.active else 'STARTING'
        self.env = None
        self.planner = None
        self.last_goals = {}
        self.last_assignments = {}

        self.command_pub = rospy.Publisher('/fleet/command', FleetCommand, queue_size=30)
        self.status_pub = rospy.Publisher(
            '/fleet/gbsflacs/status', String, queue_size=5, latch=True)
        self.algorithm_status_pub = rospy.Publisher(
            '/fleet/algorithm/status', String, queue_size=10, latch=True)
        self.plan_pub = rospy.Publisher(
            '/fleet/gbsflacs/plan', String, queue_size=5, latch=True)
        rospy.Subscriber('/fleet/state', VehicleState, self._on_state, queue_size=60)
        rospy.Subscriber('/fleet/control_lease', ControlLease, self._on_lease, queue_size=30)
        rospy.Subscriber('/fleet/command_ack', CommandAck, self._on_ack, queue_size=60)
        rospy.Subscriber('/fleet/world_model', String, self._on_world, queue_size=5)
        rospy.Subscriber('/fleet/algorithm/action', String, self._on_action, queue_size=10)
        rospy.Subscriber('/fleet/base/operator_action', String, self._on_action, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._tick)
        rospy.loginfo(
            'GBSFLACS capture adapter ready: active=%s rate=%.1fHz target=%s',
            self.active, self.control_rate, self.target_id)

    def _on_state(self, message):
        if message.vehicle_id in self.vehicle_ids:
            with self.lock:
                self.states[message.vehicle_id] = (message, time.monotonic())

    def _on_lease(self, message):
        if message.vehicle_id in self.vehicle_ids or message.vehicle_id == '*':
            with self.lock:
                self.leases[message.vehicle_id] = message

    def _on_ack(self, message):
        if not message.command_id.startswith('gbsflacs-'):
            return
        with self.lock:
            self.acks[message.command_id] = message
            pending = self.inflight.get(message.vehicle_id)
            if pending == message.command_id and message.status in TERMINAL_ACKS:
                del self.inflight[message.vehicle_id]
                self.inflight_started.pop(message.vehicle_id, None)
                if message.status != CommandAck.STATUS_SUCCEEDED:
                    self.last_error = '%s: %s' % (message.vehicle_id, message.message)
                    self.active = False
                    self.phase = 'COMMAND_FAILED'

    def _on_world(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            with self.lock:
                self.world = payload
                self.world_received = time.monotonic()

    def _on_action(self, message):
        action = str(message.data).strip()
        upper = action.upper()
        if upper.startswith('CAPTURE:'):
            requested = action.split(':', 1)[1].strip()
            if requested:
                self.target_id = requested
            self._start()
        elif upper in {'CAPTURE', 'START_CAPTURE', 'START_GBSFLACS', 'GBSFLACS'}:
            self._start()
        elif upper in {'CANCEL_CAPTURE', 'STOP_GBSFLACS', 'STOP_ALGORITHM', 'STOP'}:
            with self.lock:
                self.active = False
                self.phase = 'STOPPED'
            rospy.loginfo('GBSFLACS stopped; no new navigation commands will be sent')

    def _start(self):
        with self.lock:
            self.active = True
            self.phase = 'STARTING'
            self.last_error = ''
            self.env = None
            self.planner = None
            self.inflight.clear()
            self.inflight_started.clear()
        rospy.loginfo('GBSFLACS requested for target %s', self.target_id)

    def _fresh_states(self):
        now = time.monotonic()
        with self.lock:
            snapshot = dict(self.states)
        return {
            vehicle_id: value[0] for vehicle_id, value in snapshot.items()
            if now - value[1] <= self.state_timeout and value[0].online
        }

    def _target(self):
        with self.lock:
            world = self.world
            received = self.world_received
        if world is None or time.monotonic() - received > self.world_timeout:
            return None
        for collection in ('entities', 'targets'):
            for record in world.get(collection, []) or []:
                identifier = record.get('id') or record.get('uuid') or record.get('target_id')
                if identifier == self.target_id:
                    position = _position(record)
                    if position is not None:
                        velocity = (record.get('velocity', {}) or {}).get('linear', {}) or {}
                        vx = float(velocity.get('x', 0.0))
                        vy = float(velocity.get('y', 0.0))
                        return position, math.hypot(vx, vy), math.atan2(vy, vx)
        return None

    def _numeric_id(self, vehicle_id):
        if vehicle_id in self.uav_ids:
            return 1000 + self.uav_ids.index(vehicle_id)
        return 2000 + self.usv_ids.index(vehicle_id)

    def _vehicle_id(self, numeric_id):
        numeric_id = int(numeric_id)
        if 1000 <= numeric_id < 1003:
            return self.uav_ids[numeric_id - 1000]
        if 2000 <= numeric_id < 2003:
            return self.usv_ids[numeric_id - 2000]
        return None

    def _agent_array(self, states):
        rows = []
        for vehicle_id in self.vehicle_ids:
            state = states[vehicle_id]
            p = state.pose.position
            v = state.twist.linear
            rows.append([
                p.x, p.y, p.z,
                math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
                0.0, 0.0,
                0.0 if vehicle_id in self.uav_ids else 1.0,
                float(self._numeric_id(vehicle_id)),
            ])
        return np.asarray(rows, dtype=float)

    def _ensure_planner(self):
        if self.planner is not None:
            return
        core.SEED = self.seed
        core.ARENA_SIZE_XY = 600.0
        core.ARENA_SIZE_Z = 100.0
        core.CAPTURE_RADIUS = self.capture_radius
        core.MIN_CAPTURE_AGENTS = self.minimum_agents
        core.V_MAX_UAV = 3.0
        core.V_MAX_USV = 0.5
        # The live world model already tracks target motion.  Predicting it a
        # second time inside the offline model would double-count movement.
        core.TARGET_IS_STATIC = 1
        core.TARGET_SPEED = 0.0
        core.USV_Z = 0.0
        core.TARGET_Z = 0.0
        self.env = LiveEnvironment()
        self.planner = core.GBSFLACSAlgorithm(self.env, 'GB-SFLA-CS-ROS1')

    def _valid_lease(self, vehicle_id):
        with self.lock:
            lease = self.leases.get(vehicle_id) or self.leases.get('*')
        if lease is None or lease.revoked or lease.valid_until <= rospy.Time.now():
            return None
        return lease

    @staticmethod
    def _bounded_goal(state, final_goal, maximum_step):
        current = np.asarray([
            state.pose.position.x, state.pose.position.y, state.pose.position.z
        ], dtype=float)
        delta = np.asarray(final_goal, dtype=float) - current
        distance = float(np.linalg.norm(delta))
        if distance <= maximum_step:
            return np.asarray(final_goal, dtype=float)
        return current + delta * (maximum_step / distance)

    def _send_navigate(self, vehicle_id, state, final_goal):
        with self.lock:
            if vehicle_id in self.inflight or state.active_command_id:
                return False
        lease = self._valid_lease(vehicle_id)
        if lease is None:
            return False
        maximum = self.uav_step if vehicle_id in self.uav_ids else self.usv_step
        goal = self._bounded_goal(state, final_goal, maximum)
        self.counter += 1
        message = FleetCommand()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'map'
        message.command_id = 'gbsflacs-%s-%06d' % (vehicle_id, self.counter)
        message.vehicle_id = vehicle_id
        message.lease_id = lease.lease_id
        message.command_type = FleetCommand.COMMAND_NAVIGATE
        message.priority = 200
        message.expires_at = rospy.Time.now() + rospy.Duration(10.0)
        message.target_pose.position.x = float(goal[0])
        message.target_pose.position.y = float(goal[1])
        message.target_pose.position.z = float(goal[2])
        message.target_pose.orientation.w = 1.0
        with self.lock:
            self.inflight[vehicle_id] = message.command_id
            self.inflight_started[vehicle_id] = time.monotonic()
        self.command_pub.publish(message)
        return True

    def _build_algorithm_status(self, phase, states, diagnostics=None):
        source_phase = str(phase or 'WAITING_FOR_START').upper()
        standard_phase = source_phase
        standard_active = bool(self.active)
        standard_error = str(self.last_error or '')
        standard_diagnostics = dict(diagnostics or {})
        standard_diagnostics.update({
            'source': 'gbsflacs',
            'source_phase': source_phase,
            'target_id': self.target_id,
            'captured': source_phase == 'CAPTURED',
            'inflight': dict(self.inflight),
            'online': sorted(states),
        })

        if source_phase in {'WAITING_FOR_START', 'STOPPED'}:
            standard_phase = 'IDLE'
            standard_active = False
        elif source_phase in {
                'STARTING',
                'WAITING_FOR_VEHICLES',
                'WAITING_FOR_AIRBORNE_UAVS'}:
            # Gateway only recognizes specific waiting phase names.  Preserve
            # the more precise GBSFLACS reason in diagnostics.
            standard_phase = 'WAITING_FOR_VEHICLES'
            if source_phase == 'WAITING_FOR_AIRBORNE_UAVS':
                standard_diagnostics['reason'] = 'waiting_for_airborne_uavs'
                standard_diagnostics['uavs_not_ready'] = list(
                    standard_diagnostics.get('not_ready', []))
        elif source_phase in {'WAITING_FOR_WORLD_MODEL', 'WAITING_FOR_TARGET'}:
            standard_phase = 'WAITING_FOR_ENEMY_ENTITY'
            standard_diagnostics['reason'] = (
                'waiting_for_world_model'
                if source_phase == 'WAITING_FOR_WORLD_MODEL'
                else 'waiting_for_target'
            )
        elif source_phase == 'GB_SFLA_CS_CAPTURE':
            standard_phase = 'GB_SFLA_CS_INTERCEPT'
        elif source_phase == 'CAPTURED':
            standard_phase = 'CAPTURED_GUARDING'
            standard_active = False
        elif source_phase in {
                'COMMAND_FAILED', 'COMMAND_TIMEOUT', 'ALGORITHM_ERROR'}:
            standard_active = False
            if not standard_error:
                standard_error = 'GBSFLACS entered %s' % source_phase

        return {
            'schema_version': 'uav_usv_algorithm_status.v1',
            'timestamp': rospy.Time.now().to_sec(),
            'mode': 'capture',
            'active': standard_active,
            'phase': standard_phase,
            'publish_commands': True,
            'vehicle_count': len(states),
            'required_vehicle_count': len(self.vehicle_ids),
            'target_id': self.target_id,
            'last_error': standard_error,
            'diagnostics': standard_diagnostics,
        }

    def _publish(self, phase, states, diagnostics=None):
        status = {
            'schema_version': 'uav_usv_gbsflacs_status.v1',
            'stamp': rospy.Time.now().to_sec(),
            'active': self.active,
            'phase': phase,
            'target_id': self.target_id,
            'online': sorted(states),
            'inflight': dict(self.inflight),
            'last_error': self.last_error,
            'diagnostics': diagnostics or {},
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        algorithm_status = self._build_algorithm_status(
            phase, states, diagnostics)
        self.algorithm_status_pub.publish(String(
            data=json.dumps(algorithm_status, ensure_ascii=False)))

    def _tick(self, _event):
        states = self._fresh_states()
        if not self.active:
            self._publish(self.phase, states)
            return
        missing = sorted(set(self.vehicle_ids) - set(states))
        if missing:
            self.phase = 'WAITING_FOR_VEHICLES'
            self._publish(self.phase, states, {'missing': missing})
            return
        now = time.monotonic()
        with self.lock:
            timed_out = [
                vehicle_id for vehicle_id, started in self.inflight_started.items()
                if now - started > self.command_timeout
            ]
            if timed_out:
                self.active = False
                self.phase = 'COMMAND_TIMEOUT'
                self.last_error = 'terminal ACK timeout: ' + ','.join(sorted(timed_out))
        if timed_out:
            rospy.logerr('GBSFLACS stopped: %s', self.last_error)
            self._publish(self.phase, states, {'timed_out': sorted(timed_out)})
            return
        unsafe_uavs = [identifier for identifier in self.uav_ids if (
            not states[identifier].armed
            or states[identifier].pose.position.z < self.uav_min_altitude
        )]
        if unsafe_uavs:
            self.phase = 'WAITING_FOR_AIRBORNE_UAVS'
            self._publish(self.phase, states, {'not_ready': unsafe_uavs})
            return
        target = self._target()
        if target is None:
            self.phase = 'WAITING_FOR_TARGET'
            self._publish(self.phase, states)
            return

        try:
            self._ensure_planner()
            target_position, speed, heading = target
            core.TARGET_Z = float(target_position[2])
            self.env.agents = self._agent_array(states)
            self.env.targets = np.asarray([[
                target_position[0], target_position[1], target_position[2],
                speed, heading, 3000.0,
            ]], dtype=float)
            assignments = self.planner.step()
            goals = {}
            for numeric_id, waypoint in self.planner.desired_waypoints.items():
                vehicle_id = self._vehicle_id(numeric_id)
                if vehicle_id is None:
                    continue
                goal = np.asarray(waypoint, dtype=float).copy()
                if vehicle_id in self.uav_ids:
                    goal[2] = float(np.clip(
                        goal[2], self.uav_min_altitude, self.uav_max_altitude))
                else:
                    goal[2] = 0.0
                goals[vehicle_id] = goal

            distances = np.linalg.norm(
                self.env.agents[:, :3] - target_position[:3], axis=1)
            in_range = int(np.count_nonzero(distances <= self.capture_radius))
            captured = in_range >= self.minimum_agents
            self.last_goals = goals
            self.last_assignments = {
                self._vehicle_id(agent_id): self.target_id
                for agent_id in assignments
                if self._vehicle_id(agent_id) is not None
            }
            if captured:
                self.env.permanently_captured.add(0)
                self.active = False
                self.phase = 'CAPTURED'
            else:
                self.phase = 'GB_SFLA_CS_CAPTURE'
                for vehicle_id, goal in goals.items():
                    self._send_navigate(vehicle_id, states[vehicle_id], goal)

            plan = {
                'schema_version': 'uav_usv_gbsflacs_plan.v1',
                'stamp': rospy.Time.now().to_sec(),
                'target_id': self.target_id,
                'assignments': self.last_assignments,
                'goals': {key: [float(x) for x in value] for key, value in goals.items()},
                'objective': float(self.planner.last_objective),
            }
            self.plan_pub.publish(String(data=json.dumps(plan, ensure_ascii=False)))
            self._publish(self.phase, states, {
                'capture_radius_m': self.capture_radius,
                'minimum_capture_agents': self.minimum_agents,
                'agents_in_range': in_range,
                'granular_balls': len(self.planner.leaf_balls),
                'objective': float(self.planner.last_objective),
            })
        except Exception as exc:
            self.active = False
            self.phase = 'ALGORITHM_ERROR'
            self.last_error = '%s: %s' % (type(exc).__name__, exc)
            rospy.logerr('GBSFLACS stopped after error: %s', self.last_error)
            self._publish(self.phase, states)


def main():
    rospy.init_node('gbsflacs_capture_controller')
    GbsflacsCaptureController()
    rospy.spin()


if __name__ == '__main__':
    main()
