#!/usr/bin/env python3
"""Run the supplied capture or escort algorithm against live 332 simulation state.

The node is deliberately above the vehicle-control layer.  It reads the existing
ROS 1 world model and VehicleState streams, then emits the repository's existing
FleetCommand message.  No camera, radar, lidar, PointCloud2, LaserScan, Image, or
MAVLink transport contract is modified.
"""

import json
import itertools
import math
import threading
import time
from dataclasses import dataclass

import numpy as np
import rospy
from std_msgs.msg import String
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import VehicleState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Keep the planner and escort state machine from the exact merged 3-D program
# supplied for this integration.  In particular, this preserves its shared
# hysteretic GB-SFLA-CS controller and UAV/USV alternating capture slots.
from uav_usv_cooperative_algorithms import combined_reference as algorithm_core

escort_core = algorithm_core
capture_core = algorithm_core


VALID_MODES = {'idle', 'capture', 'escort'}


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_segment_distance_2d(point, start, end):
    """Distance used by the reference algorithm's safe-segment checks."""
    point = np.asarray(point, dtype=float)[:2]
    start = np.asarray(start, dtype=float)[:2]
    end = np.asarray(end, dtype=float)[:2]
    segment = end - start
    denominator = float(np.dot(segment, segment))
    if denominator <= 1e-12:
        return float(np.linalg.norm(point - start))
    parameter = float(np.clip(np.dot(point - start, segment) / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + parameter * segment)))


def pose_position(record):
    pose = record.get('pose', {}) if isinstance(record, dict) else {}
    position = pose.get('position', {}) if isinstance(pose, dict) else {}
    try:
        return np.array(
            [float(position['x']), float(position['y']), float(position.get('z', 0.0))],
            dtype=float,
        )
    except (KeyError, TypeError, ValueError):
        return None


class LiveCaptureEnvironment:
    """Minimum real-state environment required by the supplied GB-SFLA-CS core."""

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


@dataclass
class CommandMemory:
    command_type: int
    goal: tuple
    sent_at: float


class CooperativeAlgorithmController:
    def __init__(self):
        self.lock = threading.RLock()
        self.uav_ids = list(rospy.get_param(
            '~uav_ids', ['uav_01', 'uav_02', 'uav_03']
        ))
        self.usv_ids = list(rospy.get_param(
            '~usv_ids', ['usv_01', 'usv_02', 'usv_03']
        ))
        self.vehicle_ids = self.uav_ids + self.usv_ids
        self.enemy_entity_id = str(rospy.get_param('~enemy_entity_id', 'enemy_ship'))
        self.friendly_entity_id = str(
            rospy.get_param('~friendly_entity_id', 'friendly_ship')
        )

        requested_mode = str(rospy.get_param('~mode', 'idle')).strip().lower()
        if requested_mode not in VALID_MODES:
            raise ValueError('mode must be idle, capture, or escort')
        self.mode = requested_mode
        self.active = bool(rospy.get_param('~auto_start', False)) and self.mode != 'idle'
        self.publish_commands = bool(rospy.get_param('~publish_commands', True))
        self.control_rate = max(0.2, float(rospy.get_param('~control_rate', 1.0)))
        self.command_refresh = max(
            0.5, float(rospy.get_param('~command_refresh_seconds', 2.0))
        )
        self.command_change_threshold = max(
            0.1, float(rospy.get_param('~command_change_threshold', 1.5))
        )
        self.state_timeout = max(0.5, float(rospy.get_param('~state_timeout', 2.5)))
        self.world_model_timeout = max(
            0.5, float(rospy.get_param('~world_model_timeout', 2.5))
        )
        self.seed = int(rospy.get_param('~seed', 42))

        self.uav_takeoff_altitude = float(
            rospy.get_param('~uav_takeoff_altitude', 12.0)
        )
        self.uav_ready_altitude = float(rospy.get_param('~uav_ready_altitude', 25.0))
        self.uav_capture_altitude = float(
            rospy.get_param('~uav_capture_altitude', 30.0)
        )
        self.uav_min_operating_altitude = float(
            rospy.get_param('~uav_min_operating_altitude', 12.0)
        )
        self.uav_max_operating_altitude = float(
            rospy.get_param('~uav_max_operating_altitude', 45.0)
        )
        self.uav_escort_altitude = float(
            rospy.get_param('~uav_escort_altitude', 30.0)
        )
        self.uav_max_speed = float(rospy.get_param('~uav_max_speed', 3.0))
        self.usv_max_speed = float(rospy.get_param('~usv_max_speed', 2.0))
        self.coordinated_batch_refresh = max(
            2.0,
            float(rospy.get_param('~coordinated_batch_refresh_seconds', 5.0)),
        )
        self.coordinated_min_move = max(
            0.0, float(rospy.get_param('~coordinated_min_move_m', 3.0))
        )
        self.auto_capture_after_guard = bool(
            rospy.get_param('~auto_capture_after_guard', True)
        )
        self.escort_hold_seconds = max(
            0.0, float(rospy.get_param('~escort_hold_seconds', 20.0))
        )
        self.guard_hold_seconds = max(
            0.0, float(rospy.get_param('~guard_hold_seconds', 20.0))
        )
        self.escort_formation_tolerance = max(
            0.5, float(rospy.get_param('~escort_formation_tolerance', 4.0))
        )
        self.final_formation_spacing = max(
            4.0, float(rospy.get_param('~final_formation_spacing', 12.0))
        )
        self.final_formation_tolerance = max(
            0.5, float(rospy.get_param('~final_formation_tolerance', 2.0))
        )

        # The supplied escort program uses inverse-distance repulsion and a
        # protected-circle bypass before moving a platform. Apply the same
        # ideas here at the FleetCommand waypoint boundary: the live adapter
        # deliberately does not call the reference simulator's pose updater.
        self.collision_avoidance_enabled = bool(
            rospy.get_param('~collision_avoidance_enabled', True)
        )
        self.uav_collision_clearance = max(
            1.0, float(rospy.get_param('~uav_collision_clearance', 6.0))
        )
        self.usv_collision_clearance = max(
            1.0, float(rospy.get_param('~usv_collision_clearance', 12.0))
        )
        self.uav_repulsion_range = max(
            self.uav_collision_clearance,
            float(rospy.get_param('~uav_repulsion_range', 15.0)),
        )
        self.usv_repulsion_range = max(
            self.usv_collision_clearance,
            float(rospy.get_param('~usv_repulsion_range', 30.0)),
        )
        self.collision_repulsion_gain = max(
            0.0, float(rospy.get_param('~collision_repulsion_gain', 100.0))
        )
        self.friendly_avoid_radius = max(
            0.0, float(rospy.get_param('~friendly_avoid_radius', 15.0))
        )
        self.enemy_avoid_radius = max(
            0.0, float(rospy.get_param('~enemy_avoid_radius', 10.0))
        )
        self.safety_circle_margin = max(
            0.5, float(rospy.get_param('~safety_circle_margin', 2.0))
        )
        self.capture_radius = float(rospy.get_param('~capture_radius', 50.0))
        self.capture_guard_radius = float(
            rospy.get_param('~capture_guard_radius', 18.0)
        )
        self.capture_min_agents = int(rospy.get_param('~capture_min_agents', 5))
        self.capture_target_static = bool(
            rospy.get_param('~capture_target_static_for_planner', True)
        )

        self.escort_sensor_radius = float(
            rospy.get_param('~escort_sensor_radius', 120.0)
        )
        self.escort_ring_radius = float(
            rospy.get_param('~escort_ring_radius', 30.0)
        )
        self.escort_guard_arc_radius = float(
            rospy.get_param('~escort_guard_arc_radius', 35.0)
        )
        self.escort_support_radius = float(
            rospy.get_param('~escort_support_radius', 30.0)
        )
        self.escort_blocker_ratio = float(
            rospy.get_param('~escort_blocker_ratio', 0.38)
        )
        self.escort_blocker_min = float(
            rospy.get_param('~escort_blocker_min', 15.0)
        )
        self.escort_blocker_max = float(
            rospy.get_param('~escort_blocker_max', 35.0)
        )
        self.escort_reserve_count = int(
            rospy.get_param('~escort_reserve_count', 0)
        )

        self.vehicle_states = {}
        self.leases = {}
        self.world_model = None
        self.world_model_received = 0.0
        self.command_counter = 0
        self.command_memory = {}
        self.last_batch_goals = {}
        self.last_batch_phase = ''
        self.last_batch_sent_at = 0.0
        self.batch_counter = 0
        self.airborne_ready = set()
        self.last_goals = {}
        self.last_assignments = {}
        self.last_phase = 'idle'
        self.last_error = ''
        self.capture_env = None
        self.capture_algorithm = None
        self.capture_hold_goals = {}
        self.escort_sim = None
        self.escort_sequence_stage = 'escort'
        self.escort_ready_since = None
        self.guard_ready_since = None
        self.command_pub = rospy.Publisher(
            '/fleet/command', FleetCommand, queue_size=60
        )
        self.status_pub = rospy.Publisher(
            '/fleet/algorithm/status', String, queue_size=10, latch=True
        )
        self.assignment_pub = rospy.Publisher(
            '/fleet/algorithm/assignments', String, queue_size=10, latch=True
        )
        self.marker_pub = rospy.Publisher(
            '/fleet/algorithm/markers', MarkerArray, queue_size=5
        )

        rospy.Subscriber(
            '/fleet/state', VehicleState, self._on_vehicle_state, queue_size=60
        )
        rospy.Subscriber(
            '/fleet/control_lease', ControlLease, self._on_lease, queue_size=30
        )
        rospy.Subscriber(
            '/fleet/world_model', String, self._on_world_model, queue_size=5
        )
        rospy.Subscriber(
            '/fleet/algorithm/action', String, self._on_action, queue_size=10
        )
        # Existing Qt capture buttons already publish here.  Reusing the action
        # contract avoids introducing a second operator-control protocol.
        rospy.Subscriber(
            '/fleet/base/operator_action', String, self._on_action, queue_size=20
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self._on_timer
        )
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            'Cooperative algorithm controller ready: mode=%s active=%s commands=%s',
            self.mode,
            self.active,
            self.publish_commands,
        )

    def _on_vehicle_state(self, message):
        with self.lock:
            self.vehicle_states[message.vehicle_id] = (message, time.monotonic())

    def _on_lease(self, message):
        if message.vehicle_id in self.vehicle_ids or message.vehicle_id == '*':
            with self.lock:
                self.leases[message.vehicle_id] = message

    def _on_world_model(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as exc:
            rospy.logwarn_throttle(5.0, 'Invalid /fleet/world_model JSON: %s' % exc)
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            self.world_model = payload
            self.world_model_received = time.monotonic()

    def _on_action(self, message):
        action = str(message.data).strip().upper()
        if action.startswith('CAPTURE:') or action in {'START_CAPTURE', 'CAPTURE'}:
            self._activate('capture')
        elif action.startswith('ESCORT:') or action in {
            'START_ESCORT', 'ESCORT', 'GUARD', 'START_GUARD'
        }:
            self._activate('escort')
        elif action in {'HOLD_ALL', 'PAUSE_ALGORITHM', 'PAUSE'}:
            with self.lock:
                self.active = False
                self.last_phase = 'paused'
            self._hold_all('algorithm paused')
        elif action in {
            'CANCEL_CAPTURE', 'CANCEL_ESCORT', 'STOP_ALGORITHM', 'STOP'
        }:
            with self.lock:
                self.mode = 'idle'
                self.active = False
                self.last_phase = 'idle'
            self._hold_all('algorithm stopped')
        elif action in {'RESUME_ALGORITHM', 'RESUME'}:
            with self.lock:
                if self.mode != 'idle':
                    self.active = True

    def _activate(self, mode):
        with self.lock:
            changed = self.mode != mode
            self.mode = mode
            self.active = True
            self.last_error = ''
            if changed:
                self.capture_env = None
                self.capture_algorithm = None
                self.capture_hold_goals = {}
                self.escort_sim = None
                self.command_memory.clear()
                self.last_batch_goals.clear()
                self.last_batch_phase = ''
                self.last_batch_sent_at = 0.0
            if mode == 'escort':
                self.escort_sequence_stage = 'escort'
                self.escort_ready_since = None
                self.guard_ready_since = None
        rospy.loginfo('Cooperative algorithm activated: %s', mode)

    def _fresh_states(self):
        now = time.monotonic()
        with self.lock:
            entries = dict(self.vehicle_states)
        result = {}
        for vehicle_id in self.vehicle_ids:
            entry = entries.get(vehicle_id)
            if entry is None:
                continue
            state, received_at = entry
            if now - received_at <= self.state_timeout and state.online:
                result[vehicle_id] = state
        return result

    def _entity(self, entity_id):
        with self.lock:
            model = self.world_model
            received_at = self.world_model_received
        if model is None or time.monotonic() - received_at > self.world_model_timeout:
            return None
        for collection_name in ('entities', 'targets'):
            for item in model.get(collection_name, []) or []:
                identifier = item.get('id') or item.get('uuid') or item.get('target_id')
                if identifier == entity_id:
                    position = pose_position(item)
                    if position is not None:
                        return item, position
        perception = model.get('perception', {}) or {}
        for source_name in ('fused_targets', 'ground_truth'):
            source = perception.get(source_name, {}) or {}
            for item in source.get('objects', []) or []:
                identifier = item.get('id') or item.get('uuid')
                if identifier == entity_id:
                    position = pose_position(item)
                    if position is not None:
                        return item, position
        return None

    @staticmethod
    def _entity_velocity(record):
        linear = (record.get('velocity', {}) or {}).get('linear', {}) or {}
        try:
            vx = float(linear.get('x', 0.0))
            vy = float(linear.get('y', 0.0))
            speed = math.hypot(vx, vy)
            heading = math.atan2(vy, vx) if speed > 1e-6 else 0.0
            return speed, heading
        except (TypeError, ValueError):
            return 0.0, 0.0

    def _numeric_agent_id(self, vehicle_id):
        if vehicle_id in self.uav_ids:
            return 1000 + self.uav_ids.index(vehicle_id)
        return 2000 + self.usv_ids.index(vehicle_id)

    def _vehicle_id_from_numeric(self, numeric_id):
        numeric_id = int(numeric_id)
        if 1000 <= numeric_id < 1000 + len(self.uav_ids):
            return self.uav_ids[numeric_id - 1000]
        if 2000 <= numeric_id < 2000 + len(self.usv_ids):
            return self.usv_ids[numeric_id - 2000]
        return None

    def _build_agent_array(self, states):
        rows = []
        for vehicle_id in self.vehicle_ids:
            state = states[vehicle_id]
            position = state.pose.position
            linear = state.twist.linear
            speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
            rows.append([
                float(position.x),
                float(position.y),
                float(position.z),
                float(speed),
                float(yaw_from_quaternion(state.pose.orientation)),
                0.0,
                0.0 if vehicle_id in self.uav_ids else 1.0,
                float(self._numeric_agent_id(vehicle_id)),
            ])
        return np.asarray(rows, dtype=float)

    def _ensure_capture_algorithm(self):
        if self.capture_algorithm is not None:
            return
        capture_core.SEED = self.seed
        capture_core.ARENA_SIZE_XY = 600.0
        capture_core.ARENA_SIZE_Z = 200.0
        capture_core.CAPTURE_RADIUS = self.capture_radius
        capture_core.GBSFLACS_CAPTURE_RADIUS = self.capture_radius
        capture_core.SFLA_ACTIVATION_DISTANCE = 2.0 * self.capture_radius
        capture_core.SFLA_DEACTIVATION_DISTANCE = (
            1.2 * capture_core.SFLA_ACTIVATION_DISTANCE
        )
        capture_core.MIN_CAPTURE_AGENTS = self.capture_min_agents
        capture_core.V_MAX_UAV = self.uav_max_speed
        capture_core.V_MAX_USV = self.usv_max_speed
        capture_core.TARGET_IS_STATIC = 1 if self.capture_target_static else 0
        capture_core.TARGET_SPEED = 0.0
        capture_core.TARGET_RUN_NUM = max(3.0 * self.capture_radius, 100.0)
        capture_core.USV_Z = 0.0
        capture_core.TARGET_Z = 0.0
        capture_core.UAV_ALTITUDE_MIN = self.uav_min_operating_altitude
        capture_core.UAV_ALTITUDE_MAX = self.uav_max_operating_altitude
        capture_core.VIEW_Z_MAX = self.uav_max_operating_altitude
        self.capture_env = LiveCaptureEnvironment()
        self.capture_algorithm = capture_core.GBSFLACSAlgorithm(
            self.capture_env, 'GB-SFLA-CS-LIVE'
        )

    def _capture_goals(self, states, enemy_record, enemy_position):
        self._ensure_capture_algorithm()
        speed, heading = self._entity_velocity(enemy_record)
        self.capture_env.agents = self._build_agent_array(states)
        self.capture_env.targets = np.asarray([[
            enemy_position[0], enemy_position[1], enemy_position[2],
            speed, heading, 3000.0,
        ]], dtype=float)
        capture_core.TARGET_Z = float(enemy_position[2])

        already_captured = 0 in self.capture_env.permanently_captured
        assignments = self.capture_algorithm.step() if not already_captured else {}
        goals = {}
        numeric_assignments = dict(assignments)
        if already_captured:
            goals.update(self._final_formation_goals(enemy_position))
            numeric_assignments.update({
                self._numeric_agent_id(vehicle_id): 0 for vehicle_id in goals
            })

        for numeric_id, waypoint in self.capture_algorithm.desired_waypoints.items():
            vehicle_id = self._vehicle_id_from_numeric(numeric_id)
            if vehicle_id is None or vehicle_id in goals:
                continue
            goal = np.asarray(waypoint, dtype=float).copy()
            if vehicle_id in self.uav_ids:
                goal[2] = float(np.clip(
                    goal[2], self.uav_min_operating_altitude,
                    self.uav_max_operating_altitude,
                ))
            else:
                goal[2] = 0.0
            goals[vehicle_id] = goal

        # The supplied algorithm uses a true geometric enclosure: enough
        # platforms must be inside the 3-D radius and their largest uncovered
        # horizontal bearing interval may not exceed 180 degrees.
        if not already_captured:
            offsets = self.capture_env.agents[:, :3] - enemy_position[:3]
            distances = np.linalg.norm(offsets, axis=1)
            in_range = np.where(distances <= self.capture_radius + 1e-9)[0]
            max_gap = 2.0 * math.pi
            if len(in_range) >= 2:
                angles = np.sort(np.mod(
                    np.arctan2(offsets[in_range, 1], offsets[in_range, 0]),
                    2.0 * math.pi,
                ))
                max_gap = float(np.max(np.diff(np.r_[angles, angles[0] + 2.0 * math.pi])))
            if (
                len(in_range) >= self.capture_min_agents
                and max_gap <= capture_core.MAX_CAPTURE_ANGULAR_GAP + 1e-9
            ):
                guards = {
                    int(self.capture_env.agents[index, 7]) for index in in_range
                }
                self.capture_env.guarding_agents[0] = guards
                self.capture_env.permanently_captured.add(0)
                self.capture_hold_goals = {
                    vehicle_id: np.asarray(goal, dtype=float).copy()
                    for vehicle_id, goal in self._final_formation_goals(
                        enemy_position
                    ).items()
                }
                goals = self._final_formation_goals(enemy_position)
                already_captured = True
                rospy.loginfo(
                    'Enemy geometrically captured; guard agents=%s max_gap_deg=%.1f',
                    sorted(guards), math.degrees(max_gap),
                )

        assignments_by_name = {
            self._vehicle_id_from_numeric(agent_id): self.enemy_entity_id
            for agent_id in numeric_assignments
            if self._vehicle_id_from_numeric(agent_id) is not None
        }
        formation_errors = {
            vehicle_id: float(np.linalg.norm(
                np.asarray([
                    states[vehicle_id].pose.position.x,
                    states[vehicle_id].pose.position.y,
                    states[vehicle_id].pose.position.z,
                ]) - goal
            ))
            for vehicle_id, goal in goals.items()
        }
        formation_ready = bool(
            already_captured and len(formation_errors) == len(self.vehicle_ids)
            and max(formation_errors.values()) <= self.final_formation_tolerance
        )
        phase = (
            'FINAL_RECTANGLE_READY' if formation_ready
            else 'FINAL_RECTANGLE_FORMING' if already_captured
            else 'GB_SFLA_CS_INTERCEPT'
        )
        diagnostics = {
            'objective': float(self.capture_algorithm.last_objective)
            if np.isfinite(self.capture_algorithm.last_objective) else None,
            'granular_ball_count': len(self.capture_algorithm.leaf_balls),
            'captured': already_captured,
            'capture_radius_m': self.capture_radius,
            'minimum_capture_agents': self.capture_min_agents,
            'final_formation': 'rectangle_2x3',
            'formation_ready': formation_ready,
            'formation_errors_m': formation_errors,
        }
        return goals, assignments_by_name, phase, diagnostics

    def _final_formation_goals(self, enemy_position):
        """Return a visible 2x3 rectangular formation centered on the enemy."""
        center = np.asarray(enemy_position, dtype=float)
        spacing = self.final_formation_spacing
        goals = {}
        for column, vehicle_id in enumerate(self.uav_ids):
            goals[vehicle_id] = np.asarray([
                center[0] + (column - 1) * spacing,
                center[1] + 0.5 * spacing,
                self.uav_capture_altitude,
            ], dtype=float)
        for column, vehicle_id in enumerate(self.usv_ids):
            goals[vehicle_id] = np.asarray([
                center[0] + (column - 1) * spacing,
                center[1] - 0.5 * spacing,
                0.0,
            ], dtype=float)
        return goals

    def _ensure_escort_sim(self):
        if self.escort_sim is not None:
            return
        # The reference visualizer uses a small demonstration canvas.  Expand
        # only its planning bounds before construction; live Gazebo coordinates
        # remain unchanged.
        escort_core.WORLD_X_MIN = -1000.0
        escort_core.WORLD_X_MAX = 1000.0
        escort_core.WORLD_Y_MIN = -1000.0
        escort_core.WORLD_Y_MAX = 1000.0
        self.escort_sim = escort_core.EscortGuardSimulator(
            sensor_radius=self.escort_sensor_radius,
            seed=self.seed,
            num_uav=len(self.uav_ids),
            num_usv=len(self.usv_ids),
            escort_reserve_count=self.escort_reserve_count,
            ring_radius=self.escort_ring_radius,
            guard_arc_radius=self.escort_guard_arc_radius,
            support_guard_radius=self.escort_support_radius,
            blocker_ratio=self.escort_blocker_ratio,
            blocker_r_min=self.escort_blocker_min,
            blocker_r_max=self.escort_blocker_max,
            core_arrival_tolerance=3.0,
            wing_arrival_tolerance=4.0,
            own_target_avoid_radius=8.0,
            safe_distance=3.0,
            avoid_distance=10.0,
        )
        self.escort_sim.world_x_min = -1000.0
        self.escort_sim.world_x_max = 1000.0
        self.escort_sim.world_y_min = -1000.0
        self.escort_sim.world_y_max = 1000.0
        # The merged reference scene intentionally starts without an enemy
        # until the N-key action.  Live Gazebo already owns that enemy, so
        # initialize one task here and overwrite it from world-model truth.
        self.escort_sim.reset()

    def _escort_platform_vehicle_map(self):
        result = {}
        for index, platform in enumerate(self.escort_sim.platforms):
            prefix = platform.identifier[:1]
            number = int(platform.identifier[1:]) - 1
            if prefix == 'U' and 0 <= number < len(self.uav_ids):
                result[index] = self.uav_ids[number]
            elif prefix == 'S' and 0 <= number < len(self.usv_ids):
                result[index] = self.usv_ids[number]
        return result

    def _escort_goals(self, states, enemy_position, friendly_position):
        self._ensure_escort_sim()
        sim = self.escort_sim
        sim.frame += 1
        sim.own_position = friendly_position.copy()
        sim.own_position[2] = 0.0
        sim.own_goal = sim.own_position.copy()
        index_to_vehicle = self._escort_platform_vehicle_map()
        for index, vehicle_id in index_to_vehicle.items():
            state = states[vehicle_id]
            sim.platforms[index].position = np.array([
                state.pose.position.x,
                state.pose.position.y,
                state.pose.position.z if vehicle_id in self.uav_ids else 0.0,
            ], dtype=float)
            sim.platforms[index].altitude = float(
                sim.platforms[index].position[2]
            )

        task = sim.threats[0]
        task.position = enemy_position.copy()
        task.position[2] = 0.0
        if task.state == 'waiting':
            task.state = 'approaching'
        distance = float(np.linalg.norm(task.position[:2] - sim.own_position[:2]))
        detection_changed = False
        if (
            self.escort_sequence_stage == 'guard'
            and task.state == 'approaching'
            and distance <= sim.sensor_radius
        ):
            task.state = 'detected'
            task.detected_frame = sim.frame
            detection_changed = True
        if detection_changed:
            sim._replan_detected_guards()
        else:
            sim._synchronize_guard_plan()
        sim._refresh_all_blockers()

        platform_goals = sim._desired_non_core_goals()
        for detected_task in sim.detected_threats:
            if detected_task.core_guard_index is not None:
                platform_goals[detected_task.core_guard_index] = (
                    detected_task.blocker_point.copy()
                )
            if (
                detected_task.state in {'detected', 'forming'}
                and sim.formation_ready(detected_task.threat_id)
            ):
                detected_task.state = 'orbiting'

        goals = {}
        assignments = {}
        roles = {}
        for index, vehicle_id in index_to_vehicle.items():
            platform = sim.platforms[index]
            goal_xy = platform_goals.get(index, platform.position)
            if vehicle_id in self.uav_ids:
                z = float(np.clip(
                    goal_xy[2], self.uav_min_operating_altitude,
                    self.uav_max_operating_altitude,
                ))
            else:
                z = 0.0
            goals[vehicle_id] = np.array([goal_xy[0], goal_xy[1], z], dtype=float)
            roles[vehicle_id] = platform.role
            assignments[vehicle_id] = (
                self.enemy_entity_id
                if platform.assigned_threat_id is not None
                else self.friendly_entity_id
            )

        now = time.monotonic()
        formation_errors = {}
        for vehicle_id, goal in goals.items():
            state = states[vehicle_id]
            current = np.array([
                state.pose.position.x,
                state.pose.position.y,
                state.pose.position.z if vehicle_id in self.uav_ids else 0.0,
            ], dtype=float)
            formation_errors[vehicle_id] = float(np.linalg.norm(current - goal))

        escort_ready = bool(
            len(formation_errors) == len(self.vehicle_ids)
            and max(formation_errors.values()) <= self.escort_formation_tolerance
        )
        escort_elapsed = 0.0
        guard_elapsed = 0.0
        guard_ready = False
        guard_hold_complete = False

        if self.escort_sequence_stage == 'escort':
            self.guard_ready_since = None
            if escort_ready:
                if self.escort_ready_since is None:
                    self.escort_ready_since = now
                    rospy.loginfo(
                        'Escort formation ready; holding for %.1f seconds',
                        self.escort_hold_seconds,
                    )
                escort_elapsed = now - self.escort_ready_since
                phase = 'ESCORT_HOLD'
                if escort_elapsed >= self.escort_hold_seconds:
                    self.escort_sequence_stage = 'guard'
                    self.escort_ready_since = None
                    phase = 'GUARD_FORMING'
                    rospy.loginfo(
                        'Escort hold complete; starting guard formation'
                    )
            else:
                self.escort_ready_since = None
                phase = 'ESCORT_FORMING'
        else:
            self.escort_ready_since = None
            guard_ready = bool(
                sim.detected_threats
                and task.state == 'orbiting'
                and sim.formation_ready(task.threat_id)
            )
            if guard_ready:
                if self.guard_ready_since is None:
                    self.guard_ready_since = now
                    rospy.loginfo(
                        'Guard formation ready; holding for %.1f seconds',
                        self.guard_hold_seconds,
                    )
                guard_elapsed = now - self.guard_ready_since
                guard_hold_complete = guard_elapsed >= self.guard_hold_seconds
                phase = 'GUARD_HOLD'
            else:
                self.guard_ready_since = None
                phase = 'GUARD_FORMING'
        diagnostics = {
            'enemy_distance_to_friendly_m': distance,
            'sensor_radius_m': sim.sensor_radius,
            'threat_detected': bool(sim.detected_threats),
            'roles': roles,
            'blocker_point': task.blocker_point.tolist()
            if sim.detected_threats else None,
            'core_ready': sim.core_guard_arrived(task.threat_id)
            if sim.detected_threats else False,
            'wing_ready_ratio': sim.wing_arrival_ratio(task.threat_id)
            if sim.detected_threats else 0.0,
            'sequence_stage': self.escort_sequence_stage,
            'formation_errors_m': formation_errors,
            'escort_formation_ready': escort_ready,
            'escort_hold_seconds': self.escort_hold_seconds,
            'escort_hold_elapsed': min(escort_elapsed, self.escort_hold_seconds),
            'guard_formation_ready': guard_ready,
            'guard_hold_seconds': self.guard_hold_seconds,
            'guard_hold_elapsed': min(guard_elapsed, self.guard_hold_seconds),
            'guard_hold_complete': guard_hold_complete,
        }
        return goals, assignments, phase, diagnostics

    def _valid_lease(self, vehicle_id):
        with self.lock:
            lease = self.leases.get(vehicle_id) or self.leases.get('*')
        if lease is None or lease.revoked or lease.valid_until <= rospy.Time.now():
            return None
        return lease

    def _send_command(self, vehicle_id, command_type, goal=None, parameters=None, force=False):
        if not self.publish_commands:
            return False
        lease = self._valid_lease(vehicle_id)
        if command_type != FleetCommand.COMMAND_EMERGENCY_STOP and lease is None:
            rospy.logwarn_throttle(
                5.0, 'Waiting for a valid base-station control lease for %s' % vehicle_id
            )
            return False

        now = time.monotonic()
        goal_tuple = tuple(float(value) for value in (goal or (0.0, 0.0, 0.0)))
        previous = self.command_memory.get(vehicle_id)
        if previous is not None and not force:
            age = now - previous.sent_at
            goal_change = math.sqrt(sum(
                (a - b) ** 2 for a, b in zip(goal_tuple, previous.goal)
            ))
            if (
                previous.command_type == command_type
                and age < self.command_refresh
                and goal_change < self.command_change_threshold
            ):
                return False

        self.command_counter += 1
        message = FleetCommand()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'map'
        message.command_id = 'algorithm-%s-%s-%06d' % (
            self.mode, vehicle_id, self.command_counter
        )
        message.vehicle_id = vehicle_id
        message.lease_id = lease.lease_id if lease is not None else ''
        message.command_type = command_type
        message.priority = 200
        message.expires_at = rospy.Time.now() + rospy.Duration(10.0)
        message.target_pose.position.x = goal_tuple[0]
        message.target_pose.position.y = goal_tuple[1]
        message.target_pose.position.z = goal_tuple[2]
        message.target_pose.orientation.w = 1.0
        message.parameters = list(parameters or [])
        self.command_pub.publish(message)
        self.command_memory[vehicle_id] = CommandMemory(
            command_type=command_type, goal=goal_tuple, sent_at=now
        )
        return True

    def _vehicle_group(self, vehicle_id):
        return self.uav_ids if vehicle_id in self.uav_ids else self.usv_ids

    def _repulsion_adjustment(self, vehicle_id, states):
        """Inverse-distance repulsion from the supplied escort algorithm."""
        state = states[vehicle_id]
        current = np.array([state.pose.position.x, state.pose.position.y], dtype=float)
        interaction_range = (
            self.uav_repulsion_range
            if vehicle_id in self.uav_ids else self.usv_repulsion_range
        )
        result = np.zeros(2, dtype=float)
        group = self._vehicle_group(vehicle_id)
        own_index = group.index(vehicle_id)
        for other_id in group:
            if other_id == vehicle_id or other_id not in states:
                continue
            other = states[other_id].pose.position
            difference = current - np.array([other.x, other.y], dtype=float)
            distance = float(np.linalg.norm(difference))
            if distance >= interaction_range:
                continue
            if distance <= 1e-6:
                # Split coincident starts deterministically rather than
                # producing the zero force used by the offline simulation.
                other_index = group.index(other_id)
                angle = (own_index - other_index) * (math.pi / 2.0)
                direction = np.array([math.cos(angle), math.sin(angle)])
                distance = 0.1
            else:
                direction = difference / distance
            result += (
                self.collision_repulsion_gain
                * (1.0 / distance - 1.0 / interaction_range)
                * direction
            )
        return result

    @staticmethod
    def _current_position(state):
        return np.asarray([
            state.pose.position.x,
            state.pose.position.y,
            state.pose.position.z,
        ], dtype=float)

    def _minimum_group_distance(self, states, identifiers):
        distances = []
        for index, first_id in enumerate(identifiers):
            if first_id not in states:
                continue
            first = self._current_position(states[first_id])[:2]
            for second_id in identifiers[index + 1:]:
                if second_id in states:
                    second = self._current_position(states[second_id])[:2]
                    distances.append(float(np.linalg.norm(first - second)))
        return min(distances) if distances else None

    @staticmethod
    def _synchronized_path_distance(first_start, first_end, second_start, second_end):
        """Closest separation when two direct paths start and finish together."""
        first_start = np.asarray(first_start, dtype=float)[:2]
        first_end = np.asarray(first_end, dtype=float)[:2]
        second_start = np.asarray(second_start, dtype=float)[:2]
        second_end = np.asarray(second_end, dtype=float)[:2]
        relative_start = first_start - second_start
        relative_motion = (
            (first_end - first_start) - (second_end - second_start)
        )
        denominator = float(np.dot(relative_motion, relative_motion))
        if denominator <= 1e-12:
            progress = 0.0
        else:
            progress = float(np.clip(
                -np.dot(relative_start, relative_motion) / denominator,
                0.0,
                1.0,
            ))
        return float(np.linalg.norm(
            relative_start + progress * relative_motion
        ))

    def _ring_slots(self, states, identifiers, center, altitude_values, radius):
        """Create separated final slots; these are not route segments."""
        current_center = np.mean([
            self._current_position(states[vehicle_id])[:2]
            for vehicle_id in identifiers
        ], axis=0)
        direction = current_center - center[:2]
        base_angle = (
            math.atan2(direction[1], direction[0])
            if float(np.linalg.norm(direction)) > 1e-6 else 0.0
        )
        slots = []
        count = len(identifiers)
        for index in range(count):
            angle = base_angle + 2.0 * math.pi * index / count
            slots.append(np.asarray([
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
                altitude_values[index],
            ], dtype=float))
        return slots

    def _best_direct_assignment(
        self, states, identifiers, slots, clearance, obstacles
    ):
        """Assign complete goals while penalizing crossing direct trajectories."""
        starts = {
            vehicle_id: self._current_position(states[vehicle_id])
            for vehicle_id in identifiers
        }
        best = None
        for order in itertools.permutations(range(len(slots))):
            candidate = {
                vehicle_id: np.asarray(slots[slot_index], dtype=float).copy()
                for vehicle_id, slot_index in zip(identifiers, order)
            }
            distances = {
                vehicle_id: float(np.linalg.norm(
                    candidate[vehicle_id] - starts[vehicle_id]
                ))
                for vehicle_id in identifiers
            }
            cost = sum(distances.values())
            minimum_separation = float('inf')
            for first_index, first_id in enumerate(identifiers):
                for second_id in identifiers[first_index + 1:]:
                    separation = self._synchronized_path_distance(
                        starts[first_id], candidate[first_id],
                        starts[second_id], candidate[second_id],
                    )
                    minimum_separation = min(minimum_separation, separation)
                    if separation < clearance:
                        cost += 10000.0 * (clearance - separation + 1.0)
            if max(distances.values(), default=0.0) > 2.0 * self.coordinated_min_move:
                for distance in distances.values():
                    if distance < self.coordinated_min_move:
                        cost += 2000.0 * (
                            self.coordinated_min_move - distance + 1.0
                        )
            for center, radius in obstacles:
                for vehicle_id in identifiers:
                    route_clearance = point_segment_distance_2d(
                        center, starts[vehicle_id], candidate[vehicle_id]
                    )
                    # A vehicle already inside a protected circle must be able
                    # to leave it; otherwise prefer a path that never enters.
                    start_clearance = float(np.linalg.norm(
                        starts[vehicle_id][:2] - np.asarray(center)[:2]
                    ))
                    if route_clearance < radius and start_clearance >= radius:
                        cost += 10000.0 * (radius - route_clearance + 1.0)
            score = (cost, candidate, minimum_separation, distances)
            if best is None or score[0] < best[0]:
                best = score
        return best

    def _coordinate_group_goals(
        self, states, identifiers, goals, clearance, obstacles, diagnostics
    ):
        """Return collision-checked direct goals for one vehicle class."""
        slots = [np.asarray(goals[vehicle_id], dtype=float) for vehicle_id in identifiers]
        center = np.mean(slots, axis=0)
        altitude_values = [float(slot[2]) for slot in slots]
        endpoint_separations = [
            float(np.linalg.norm(slots[first][:2] - slots[second][:2]))
            for first in range(len(slots))
            for second in range(first + 1, len(slots))
        ]
        minimum_endpoint = min(endpoint_separations, default=float('inf'))
        spread = minimum_endpoint < clearance
        identity = {
            vehicle_id: slot.copy()
            for vehicle_id, slot in zip(identifiers, slots)
        }
        identity_separations = [
            self._synchronized_path_distance(
                self._current_position(states[first_id]), identity[first_id],
                self._current_position(states[second_id]), identity[second_id],
            )
            for first_index, first_id in enumerate(identifiers)
            for second_id in identifiers[first_index + 1:]
        ]
        identity_clearance = min(
            identity_separations, default=float('inf')
        )
        identity_obstacle_safe = True
        for obstacle_center, obstacle_radius in obstacles:
            for vehicle_id in identifiers:
                start = self._current_position(states[vehicle_id])
                start_clearance = float(np.linalg.norm(
                    start[:2] - np.asarray(obstacle_center)[:2]
                ))
                route_clearance = point_segment_distance_2d(
                    obstacle_center, start, identity[vehicle_id]
                )
                if (
                    start_clearance >= obstacle_radius
                    and route_clearance < obstacle_radius
                ):
                    identity_obstacle_safe = False
                    break
            if not identity_obstacle_safe:
                break

        # Keep the algorithm's role-to-vehicle mapping whenever its original
        # direct trajectories are already safe. This lets its formation-ready
        # checks observe the exact goals they generated.
        if (
            not spread
            and identity_clearance >= clearance
            and identity_obstacle_safe
        ):
            diagnostics['predicted_direct_path_clearance_m'][
                'uav' if identifiers[0] in self.uav_ids else 'usv'
            ] = None if not np.isfinite(identity_clearance) else identity_clearance
            return identity

        if spread:
            radius = max(
                self.capture_guard_radius,
                1.25 * clearance /
                max(2.0 * math.sin(math.pi / len(identifiers)), 1e-6),
            )
            slots = self._ring_slots(
                states, identifiers, center, altitude_values, radius
            )

        best = self._best_direct_assignment(
            states, identifiers, slots, clearance, obstacles
        )
        # If even the best assignment brings simultaneous direct tracks too
        # close, widen the final formation instead of introducing waypoints.
        expansion = 0
        while best[2] < clearance and expansion < 3:
            expansion += 1
            radius = max(
                self.capture_guard_radius,
                clearance,
                max(float(np.linalg.norm(slot[:2] - center[:2])) for slot in slots),
            ) * 1.5
            slots = self._ring_slots(
                states, identifiers, center, altitude_values, radius
            )
            best = self._best_direct_assignment(
                states, identifiers, slots, clearance, obstacles
            )
            spread = True

        if spread:
            diagnostics['expanded_goal_groups'].append({
                'vehicles': list(identifiers),
                'expansions': expansion,
            })
        diagnostics['predicted_direct_path_clearance_m'][
            'uav' if identifiers[0] in self.uav_ids else 'usv'
        ] = None if not np.isfinite(best[2]) else best[2]
        return best[1]

    def _prepare_coordinated_goals(
        self, states, goals, enemy_position, friendly_position, diagnostics
    ):
        prepared = {
            vehicle_id: np.asarray(goals[vehicle_id], dtype=float).copy()
            for vehicle_id in self.vehicle_ids
        }
        if self.collision_avoidance_enabled:
            for vehicle_id in self.vehicle_ids:
                repulsion = self._repulsion_adjustment(vehicle_id, states)
                if float(np.linalg.norm(repulsion)) > 1e-6:
                    prepared[vehicle_id][:2] += repulsion
                    diagnostics['repulsion_adjusted'].append(vehicle_id)

            # Surface targets may not finish inside either protected vessel.
            for vehicle_id in self.usv_ids:
                for label, center, radius in (
                    ('friendly', friendly_position, self.friendly_avoid_radius),
                    ('enemy', enemy_position, self.enemy_avoid_radius),
                ):
                    if center is None or radius <= 0.0:
                        continue
                    offset = prepared[vehicle_id][:2] - np.asarray(center)[:2]
                    distance = float(np.linalg.norm(offset))
                    safe_radius = radius + self.safety_circle_margin
                    if distance < safe_radius:
                        if distance <= 1e-6:
                            index = self.usv_ids.index(vehicle_id)
                            angle = 2.0 * math.pi * index / len(self.usv_ids)
                            offset = np.array([math.cos(angle), math.sin(angle)])
                        else:
                            offset /= distance
                        prepared[vehicle_id][:2] = (
                            np.asarray(center)[:2] + safe_radius * offset
                        )
                        diagnostics['protected_goal_adjustments'].append(
                            '%s:%s' % (vehicle_id, label)
                        )

            uav_goals = self._coordinate_group_goals(
                states, self.uav_ids, prepared,
                self.uav_collision_clearance, [], diagnostics,
            )
            usv_obstacles = [
                (np.asarray(center)[:2], radius + self.safety_circle_margin)
                for center, radius in (
                    (friendly_position, self.friendly_avoid_radius),
                    (enemy_position, self.enemy_avoid_radius),
                )
                if center is not None and radius > 0.0
            ]
            usv_goals = self._coordinate_group_goals(
                states, self.usv_ids, prepared,
                self.usv_collision_clearance, usv_obstacles, diagnostics,
            )
            prepared.update(uav_goals)
            prepared.update(usv_goals)
        return prepared

    def _dispatch_goals(
        self, states, goals, enemy_position=None, friendly_position=None,
        phase=''
    ):
        diagnostics = {
            'enabled': self.collision_avoidance_enabled,
            'waiting_vehicles': [],
            'repulsion_adjusted': [],
            'protected_goal_adjustments': [],
            'expanded_goal_groups': [],
            'predicted_direct_path_clearance_m': {},
            'batch_state': 'IDLE',
            'batch_id': self.batch_counter,
            'batch_sent_vehicles': [],
            'minimum_uav_distance_m': self._minimum_group_distance(
                states, self.uav_ids
            ),
            'minimum_usv_distance_m': self._minimum_group_distance(
                states, self.usv_ids
            ),
        }

        # TAKEOFF is a synchronized preparation stage. Surface vehicles do not
        # leave early while one of the three aircraft is still climbing.
        for vehicle_id in self.uav_ids:
            state = states[vehicle_id]
            if not state.armed:
                self.airborne_ready.discard(vehicle_id)
            elif state.pose.position.z >= self.uav_ready_altitude:
                self.airborne_ready.add(vehicle_id)
        if len(self.airborne_ready) != len(self.uav_ids):
            diagnostics['batch_state'] = 'WAITING_FOR_ALL_UAVS_AIRBORNE'
            for vehicle_id in self.uav_ids:
                state = states[vehicle_id]
                if not state.armed and not state.active_command_id:
                    if self._send_command(
                        vehicle_id,
                        FleetCommand.COMMAND_TAKEOFF,
                        parameters=[self.uav_takeoff_altitude],
                    ):
                        diagnostics['batch_sent_vehicles'].append(vehicle_id)
            return diagnostics

        active_vehicles = [
            vehicle_id for vehicle_id in self.vehicle_ids
            if states[vehicle_id].active_command_id
        ]
        if active_vehicles:
            diagnostics['batch_state'] = 'WAITING_FOR_BATCH_COMPLETION'
            diagnostics['waiting_vehicles'] = active_vehicles
            return diagnostics

        missing_goals = [
            vehicle_id for vehicle_id in self.vehicle_ids
            if vehicle_id not in goals
        ]
        if missing_goals:
            diagnostics['batch_state'] = 'WAITING_FOR_COMPLETE_GOAL_SET'
            diagnostics['waiting_vehicles'] = missing_goals
            return diagnostics

        missing_leases = [
            vehicle_id for vehicle_id in self.vehicle_ids
            if self._valid_lease(vehicle_id) is None
        ]
        if missing_leases:
            diagnostics['batch_state'] = 'WAITING_FOR_ALL_CONTROL_LEASES'
            diagnostics['waiting_vehicles'] = missing_leases
            return diagnostics

        coordinated_goals = self._prepare_coordinated_goals(
            states, goals, enemy_position, friendly_position, diagnostics
        )
        goal_change = max([
            float(np.linalg.norm(
                coordinated_goals[vehicle_id]
                - np.asarray(self.last_batch_goals.get(
                    vehicle_id, coordinated_goals[vehicle_id]
                ), dtype=float)
            ))
            for vehicle_id in self.vehicle_ids
        ] or [0.0])
        phase_changed = phase != self.last_batch_phase
        age = time.monotonic() - self.last_batch_sent_at
        if (
            self.last_batch_goals
            and not phase_changed
            and goal_change < self.command_change_threshold
            and age < self.coordinated_batch_refresh
        ):
            diagnostics['batch_state'] = 'BATCH_GOALS_STABLE'
            return diagnostics

        self.batch_counter += 1
        diagnostics['batch_id'] = self.batch_counter
        diagnostics['batch_state'] = 'BATCH_DISPATCHED'
        for vehicle_id in self.vehicle_ids:
            if self._send_command(
                vehicle_id,
                FleetCommand.COMMAND_NAVIGATE,
                goal=tuple(coordinated_goals[vehicle_id]),
                force=True,
            ):
                diagnostics['batch_sent_vehicles'].append(vehicle_id)
        self.last_batch_goals = {
            vehicle_id: goal.copy()
            for vehicle_id, goal in coordinated_goals.items()
        }
        self.last_batch_phase = phase
        self.last_batch_sent_at = time.monotonic()
        return diagnostics

    def _hold_all(self, reason):
        rospy.loginfo('%s; sending HOLD to the six vehicle agents', reason)
        for vehicle_id in self.vehicle_ids:
            self._send_command(
                vehicle_id, FleetCommand.COMMAND_HOLD, force=True
            )

    def _publish_markers(self, states, goals, enemy_position=None, friendly_position=None):
        markers = MarkerArray()
        now = rospy.Time.now()
        marker_id = 0
        for vehicle_id, goal in sorted(goals.items()):
            goal = np.asarray(goal, dtype=float)
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = 'map'
            marker.ns = 'algorithm_goals'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(goal[0])
            marker.pose.position.y = float(goal[1])
            marker.pose.position.z = float(goal[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 2.5
            if vehicle_id in self.uav_ids:
                marker.color.r, marker.color.g, marker.color.b = 0.10, 0.55, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.85, 0.65
            marker.color.a = 0.9
            marker.lifetime = rospy.Duration(2.5)
            markers.markers.append(marker)

            state = states.get(vehicle_id)
            if state is not None:
                line = Marker()
                line.header.stamp = now
                line.header.frame_id = 'map'
                line.ns = 'algorithm_assignment_lines'
                line.id = marker_id
                marker_id += 1
                line.type = Marker.LINE_LIST
                line.action = Marker.ADD
                line.scale.x = 0.45
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    marker.color.r, marker.color.g, marker.color.b, 0.7
                )
                start = state.pose.position
                from geometry_msgs.msg import Point
                p0 = Point(x=start.x, y=start.y, z=start.z)
                p1 = Point(x=float(goal[0]), y=float(goal[1]), z=float(goal[2]))
                line.points = [p0, p1]
                line.lifetime = rospy.Duration(2.5)
                markers.markers.append(line)
        self.marker_pub.publish(markers)

    def _publish_json(self, publisher, payload):
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        publisher.publish(message)

    def _publish_status(self, states, phase, diagnostics=None):
        payload = {
            'schema_version': 'uav_usv_algorithm_status.v1',
            'timestamp': rospy.Time.now().to_sec(),
            'mode': self.mode,
            'active': self.active,
            'phase': phase,
            'publish_commands': self.publish_commands,
            'vehicle_count': len(states),
            'required_vehicle_count': len(self.vehicle_ids),
            'last_error': self.last_error,
            'diagnostics': diagnostics or {},
        }
        self._publish_json(self.status_pub, payload)

    def _on_timer(self, _event):
        with self.lock:
            mode = self.mode
            active = self.active
        states = self._fresh_states()
        if not active or mode == 'idle':
            self._publish_status(states, self.last_phase)
            return
        if len(states) != len(self.vehicle_ids):
            missing = sorted(set(self.vehicle_ids) - set(states))
            self.last_phase = 'WAITING_FOR_VEHICLES'
            self._publish_status(states, self.last_phase, {'missing': missing})
            return

        enemy = self._entity(self.enemy_entity_id)
        if enemy is None:
            self.last_phase = 'WAITING_FOR_ENEMY_ENTITY'
            self._publish_status(states, self.last_phase)
            return
        enemy_record, enemy_position = enemy
        friendly = self._entity(self.friendly_entity_id)

        try:
            if mode == 'capture':
                goals, assignments, phase, diagnostics = self._capture_goals(
                    states, enemy_record, enemy_position
                )
                friendly_position = friendly[1] if friendly is not None else None
            elif mode == 'escort':
                if friendly is None:
                    self.last_phase = 'WAITING_FOR_FRIENDLY_ENTITY'
                    self._publish_status(states, self.last_phase)
                    return
                _, friendly_position = friendly
                goals, assignments, phase, diagnostics = self._escort_goals(
                    states, enemy_position, friendly_position
                )
            else:
                return
            self.last_error = ''
            self.last_phase = phase
            self.last_goals = goals
            self.last_assignments = assignments
            diagnostics['collision_avoidance'] = self._dispatch_goals(
                states, goals, enemy_position, friendly_position, phase=phase
            )
            self._publish_markers(
                states, goals, enemy_position, friendly_position
            )
            self._publish_json(self.assignment_pub, {
                'schema_version': 'uav_usv_algorithm_assignments.v1',
                'timestamp': rospy.Time.now().to_sec(),
                'mode': mode,
                'phase': phase,
                'assignments': assignments,
                'goals': {
                    vehicle_id: {
                        'x': float(goal[0]),
                        'y': float(goal[1]),
                        'z': float(goal[2]),
                    }
                    for vehicle_id, goal in goals.items()
                },
            })
            self._publish_status(states, phase, diagnostics)
            if (
                mode == 'escort'
                and self.auto_capture_after_guard
                and diagnostics.get('guard_hold_complete', False)
            ):
                rospy.loginfo(
                    'Guard hold complete; switching to GBSFLACS capture'
                )
                self._activate('capture')
        except Exception as exc:
            self.last_error = '%s: %s' % (type(exc).__name__, exc)
            self.last_phase = 'ALGORITHM_ERROR'
            rospy.logerr_throttle(2.0, 'Algorithm controller error: %s' % self.last_error)
            self._publish_status(states, self.last_phase)

    def shutdown(self):
        try:
            self.timer.shutdown()
            self._hold_all('algorithm controller shutdown')
        except Exception:
            pass


def main():
    rospy.init_node('cooperative_algorithm_controller')
    CooperativeAlgorithmController()
    rospy.spin()


if __name__ == '__main__':
    main()
