#!/usr/bin/env python3
import math
import random
import time
from collections import deque
from dataclasses import dataclass
from itertools import permutations

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker as GzMarker
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


@dataclass
class Vessel:
    name: str
    x: float
    y: float
    yaw: float
    target_x: float
    target_y: float
    state: str
    speed: float = 0.0
    linear: float = 0.0
    angular: float = 0.0


class CooperativeResponseMission(Node):
    """Single-world patrol, defense, retreat tracking and capture demo."""

    def __init__(self):
        super().__init__('cooperative_response_mission')
        self.declare_parameter('world_name', 'cooperative_response_sim')
        self.declare_parameter('base_x', -120.0)
        self.declare_parameter('base_y', -120.0)
        self.declare_parameter('seed', 23)
        self.declare_parameter('defend_radius', 90.0)
        self.declare_parameter('trigger_radius', 230.0)
        self.declare_parameter('own_patrol_speed', 8.0)
        self.declare_parameter('own_guard_speed', 16.0)
        self.declare_parameter('enemy_speed', 6.0)
        self.declare_parameter('retreat_speed', 9.0)
        self.declare_parameter('capture_radius', 62.0)
        self.declare_parameter('capture_start_radius', 125.0)
        self.declare_parameter('capture_contract_rate', 1.0)
        self.declare_parameter('capture_prediction_time', 3.0)
        self.declare_parameter('block_distance', 30.0)
        self.declare_parameter('guard_stop_distance', 34.0)
        self.declare_parameter('enemy_guard_stop_distance', 24.0)
        self.declare_parameter('intercept_stop_distance', 24.0)
        self.declare_parameter('guard_spacing', 45.0)
        self.declare_parameter('outcome_delay', 7.0)
        self.declare_parameter('auto_capture', False)
        self.declare_parameter('base_avoid_radius', 62.0)
        self.declare_parameter('own_avoid_radius', 58.0)
        self.declare_parameter('collision_stop_radius', 30.0)
        self.declare_parameter('collision_lookahead', 3.0)
        self.declare_parameter('radar_avoid_radius', 58.0)
        self.declare_parameter('patrol_waypoint_lead', 0.85)
        self.declare_parameter('gazebo_tactical_markers', False)
        self.declare_parameter('px4_uav_enabled', False)
        self.declare_parameter('px4_uav_model', 'x500_mono_cam_down_0')

        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.random = random.Random(int(self.get_parameter('seed').value))
        self.start_time = time.monotonic()
        self.guard_start_time = None
        self.outcome_time = None
        self.retreat_enemy = None
        self.capture_active = False
        self.capture_started = None
        self.capture_phase = 'idle'
        self.capture_ring_radius = float(
            self.get_parameter('capture_start_radius').value
        )
        self.capture_complete = False
        self.capture_slots = {}
        self.teleported = False
        self.teleport_index = 0
        self.last_uav_pose_update = 0.0
        self.uav_x = self.base_x
        self.uav_y = self.base_y
        self.uav_z = 48.0
        self.px4_uav_enabled = bool(
            self.get_parameter('px4_uav_enabled').value
        )
        self.px4_uav_model = str(
            self.get_parameter('px4_uav_model').value
        )
        self.base_radar_points = []
        self.base_radar_stamp = 0.0
        self.patrol_lanes = {}

        self.own_boats = self._make_own_boats()
        self.enemy_boats = self._make_enemy_boats()
        self.enemy_spawn = {
            enemy.name: (enemy.x, enemy.y, enemy.yaw)
            for enemy in self.enemy_boats
        }
        self.gz_node = GzTransportNode()
        self.pose_topic = '/world/%s/pose/info' % str(
            self.get_parameter('world_name').value
        )
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose)
        self.cmd_pubs = {
            vessel.name: self.gz_node.advertise(
                '/model/%s/cmd_vel' % vessel.name, Twist
            )
            for vessel in self.own_boats + self.enemy_boats
        }
        self.gz_marker_pub = self.gz_node.advertise('/marker', GzMarker)
        self.trails = {
            vessel.name: deque(maxlen=100)
            for vessel in self.own_boats + self.enemy_boats
        }
        self.last_trail_update = 0.0
        self.last_gz_marker_update = 0.0

        self.own_pub = self.create_publisher(
            PoseArray, '/defense/own_ships', 10
        )
        self.enemy_pub = self.create_publisher(
            PoseArray, '/defense/enemy_ships', 10
        )
        self.defense_status_pub = self.create_publisher(
            String, '/defense/status', 10
        )
        self.capture_status_pub = self.create_publisher(
            String, '/fleet/capture/status', 10
        )
        self.target_pub = self.create_publisher(
            TrackedObjectArray, '/fleet/perception/targets', 10
        )
        self.selected_target_pub = self.create_publisher(
            PoseStamped, '/fleet/base/selected_target', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/mission/markers', 10
        )
        self.create_subscription(
            String,
            '/fleet/base/operator_action',
            self._on_operator_action,
            20,
        )
        self.create_subscription(
            LaserScan,
            '/fleet/base/radar/scan',
            self._on_base_radar,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._update)
        self.create_timer(0.2, self._publish)
        self.create_timer(0.8, self._try_initial_teleport)
        self.get_logger().info(
            'Cooperative response mission online: patrol -> defense -> capture'
        )

    def _make_own_boats(self):
        offsets = [(-120, -72), (-115, -24), (-115, 24), (-120, 72)]
        return [
            Vessel(
                'own_%02d' % (index + 1),
                self.base_x + dx,
                self.base_y + dy,
                0.0,
                self.base_x + dx,
                self.base_y + dy,
                'patrol',
            )
            for index, (dx, dy) in enumerate(offsets)
        ]

    def _on_base_radar(self, msg):
        points = {}
        step = max(1, len(msg.ranges) // 360)
        for index in range(0, len(msg.ranges), step):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            x = self.base_x + 8.0 + distance * math.cos(angle)
            y = self.base_y + distance * math.sin(angle)
            key = (round(x / 6.0), round(y / 6.0))
            points.setdefault(key, (x, y))
        self.base_radar_points = list(points.values())
        self.base_radar_stamp = time.monotonic()

    def _make_enemy_boats(self):
        boats = []
        used = []
        for index in range(4):
            for _attempt in range(100):
                radius = self.random.uniform(390.0, 560.0)
                angle = self.random.uniform(-math.pi, math.pi)
                x = self.base_x + radius * math.cos(angle)
                y = self.base_y + radius * math.sin(angle)
                if all(math.hypot(x - px, y - py) > 110.0 for px, py in used):
                    used.append((x, y))
                    break
            yaw = math.atan2(self.base_y - y, self.base_x - x)
            boats.append(
                Vessel(
                    'enemy_%02d' % (index + 1),
                    x,
                    y,
                    yaw,
                    self.base_x,
                    self.base_y,
                    'attack',
                )
            )
        return boats

    def _try_initial_teleport(self):
        if self.teleported or time.monotonic() - self.start_time < 5.0:
            return
        world = str(self.get_parameter('world_name').value)
        enemy = self.enemy_boats[self.teleport_index]
        spawn_x, spawn_y, spawn_yaw = self.enemy_spawn[enemy.name]
        success = self._set_model_pose(
            world, enemy.name, spawn_x, spawn_y, 1.65, spawn_yaw
        )
        if not success:
            return
        self.teleport_index += 1
        if self.teleport_index >= len(self.enemy_boats):
            self.teleported = True
            self.get_logger().info('Enemy fleet randomized for this run')

    def _on_pose(self, msg):
        vessels = {
            vessel.name: vessel
            for vessel in self.own_boats + self.enemy_boats
        }
        for pose in msg.pose:
            if self.px4_uav_enabled and pose.name == self.px4_uav_model:
                self.uav_x = pose.position.x
                self.uav_y = pose.position.y
                self.uav_z = pose.position.z
                continue
            vessel = vessels.get(pose.name)
            if vessel is None:
                continue
            vessel.x = pose.position.x
            vessel.y = pose.position.y
            vessel.yaw = self._yaw(pose.orientation.z, pose.orientation.w)

    def _on_operator_action(self, msg):
        action = msg.data.strip()
        upper = action.upper()
        if upper.startswith('CAPTURE:'):
            target = action.split(':', 1)[1].strip()
            if (
                self.retreat_enemy is not None
                and target.lower() == self.retreat_enemy.name.lower()
            ):
                self.capture_active = True
                self.capture_complete = False
                self.capture_started = time.monotonic()
                self.capture_phase = 'approach'
                self.capture_slots.clear()
                self.get_logger().info('Capture approved for %s' % target)
        elif upper == 'CANCEL_CAPTURE':
            self.capture_active = False

    def _update(self):
        now = time.monotonic()
        defend_radius = float(self.get_parameter('defend_radius').value)
        trigger_radius = float(self.get_parameter('trigger_radius').value)
        threats = [
            enemy for enemy in self.enemy_boats
            if enemy.state in ('attack', 'blocked')
            and self._distance_to_base(enemy) <= trigger_radius
        ]

        if self.retreat_enemy is None:
            for enemy in self.enemy_boats:
                if enemy.state == 'attack':
                    enemy.target_x = self.base_x
                    enemy.target_y = self.base_y
                    enemy.speed = float(self.get_parameter('enemy_speed').value)
            if threats:
                if self.guard_start_time is None:
                    self.guard_start_time = now
                self._assign_guard_points(threats, defend_radius)
                for own in self.own_boats:
                    own.state = 'guard'
                    own.speed = float(self.get_parameter('own_guard_speed').value)
                self._mark_blocked(threats)
                blocked = [enemy for enemy in threats if enemy.state == 'blocked']
                delay = float(self.get_parameter('outcome_delay').value)
                if blocked and now - self.guard_start_time >= delay:
                    self._resolve_defense(blocked[0])
            else:
                self._patrol_own_boats()
        elif self.capture_active:
            self._update_capture()
        else:
            self._update_retreat()
            self._patrol_own_boats()

        self._update_uav(now)
        self._update_trails(now)
        if (
            bool(self.get_parameter('gazebo_tactical_markers').value)
            and now - self.last_gz_marker_update >= 0.5
        ):
            self.last_gz_marker_update = now
            self._publish_gazebo_markers()
        for vessel in self.own_boats + self.enemy_boats:
            self._drive(vessel)

    def _patrol_own_boats(self):
        lead = float(self.get_parameter('patrol_waypoint_lead').value)
        lane_radii = (150.0, 185.0, 220.0, 255.0)
        for index, own in enumerate(self.own_boats):
            entering_patrol = own.state != 'patrol'
            own.state = 'patrol'
            own.speed = float(self.get_parameter('own_patrol_speed').value)
            direction = 1.0 if index % 2 == 0 else -1.0
            reached = math.hypot(
                own.x - own.target_x, own.y - own.target_y
            ) < 25.0
            if entering_patrol or reached or index not in self.patrol_lanes:
                current = math.atan2(
                    own.y - self.base_y, own.x - self.base_x
                )
                angle = current + direction * lead
                radius = lane_radii[index]
                own.target_x = self.base_x + radius * math.cos(angle)
                own.target_y = self.base_y + radius * math.sin(angle)
                self.patrol_lanes[index] = angle

    def _assign_guard_points(self, threats, radius):
        ordered = sorted(threats, key=self._distance_to_base)
        spacing = float(self.get_parameter('guard_spacing').value)
        angular_spacing = spacing / max(radius, 1.0)
        for index, own in enumerate(self.own_boats):
            enemy = ordered[index % len(ordered)]
            angle = math.atan2(enemy.y - self.base_y, enemy.x - self.base_x)
            spread = (index // len(ordered)) * angular_spacing
            if index % 2:
                spread = -spread
            own.target_x = self.base_x + radius * math.cos(angle + spread)
            own.target_y = self.base_y + radius * math.sin(angle + spread)

    def _mark_blocked(self, threats):
        threshold = float(
            self.get_parameter('enemy_guard_stop_distance').value
        )
        for enemy in threats:
            nearest = min(
                math.hypot(enemy.x - own.x, enemy.y - own.y)
                for own in self.own_boats
            )
            if nearest <= threshold:
                enemy.state = 'blocked'
                enemy.speed = 0.0
                enemy.target_x = enemy.x
                enemy.target_y = enemy.y

    def _resolve_defense(self, retreat_enemy):
        self.retreat_enemy = retreat_enemy
        self.retreat_enemy.state = 'retreating'
        for enemy in self.enemy_boats:
            if enemy is self.retreat_enemy:
                continue
            enemy.state = 'sunk'
            enemy.speed = 0.0
            enemy.target_x = enemy.x
            enemy.target_y = enemy.y
        self.outcome_time = time.monotonic()
        if bool(self.get_parameter('auto_capture').value):
            self.capture_active = True
        self.get_logger().info(
            '%s retreating; remaining enemies simulated sunk'
            % self.retreat_enemy.name
        )

    def _update_retreat(self):
        enemy = self.retreat_enemy
        dx = enemy.x - self.base_x
        dy = enemy.y - self.base_y
        length = max(1.0, math.hypot(dx, dy))
        tangent = 36.0 * math.sin((time.monotonic() - self.outcome_time) * 0.18)
        enemy.target_x = self.base_x + dx / length * 650.0 - dy / length * tangent
        enemy.target_y = self.base_y + dy / length * 650.0 + dx / length * tangent
        enemy.speed = float(self.get_parameter('retreat_speed').value)

    def _update_capture(self):
        enemy = self.retreat_enemy
        self._update_retreat()
        final_radius = float(self.get_parameter('capture_radius').value)
        start_radius = float(
            self.get_parameter('capture_start_radius').value
        )
        if self.capture_started is None:
            self.capture_started = time.monotonic()
        elapsed = time.monotonic() - self.capture_started
        if self.capture_phase == 'approach':
            radius = start_radius
        else:
            radius = max(
                final_radius,
                start_radius - elapsed * float(
                    self.get_parameter('capture_contract_rate').value
                ),
            )
            if radius <= final_radius + 0.1:
                self.capture_phase = 'hold'
        self.capture_ring_radius = radius
        prediction_time = float(
            self.get_parameter('capture_prediction_time').value
        )
        center_x = enemy.x + enemy.linear * math.cos(enemy.yaw) * prediction_time
        center_y = enemy.y + enemy.linear * math.sin(enemy.yaw) * prediction_time
        self._assign_capture_slots(center_x, center_y, enemy.yaw, radius)
        arrived = 0
        formed = 0
        for own in self.own_boats:
            slot = self.capture_slots[own.name]
            angle = (
                enemy.yaw + math.pi
                + 2.0 * math.pi * slot / len(self.own_boats)
                + 0.12 * math.sin(elapsed * 0.18)
            )
            own.target_x = center_x + radius * math.cos(angle)
            own.target_y = center_y + radius * math.sin(angle)
            own.state = 'capture'
            base_speed = float(self.get_parameter('own_guard_speed').value)
            own.speed = base_speed if self.capture_phase == 'approach' else base_speed * 0.72
            if self.capture_phase == 'hold':
                own.speed = base_speed * 0.42
            stop_distance = float(
                self.get_parameter('intercept_stop_distance').value
            )
            slot_error = math.hypot(
                own.x - own.target_x, own.y - own.target_y
            )
            if slot_error < 52.0:
                formed += 1
            if slot_error < stop_distance:
                arrived += 1
        if self.capture_phase == 'approach' and formed >= 3:
            self.capture_phase = 'contract'
            self.capture_started = time.monotonic()
        if arrived == len(self.own_boats):
            enemy.state = 'captured'
            enemy.speed = 0.0
            self.capture_active = False
            self.capture_complete = True
            self.capture_phase = 'complete'

    def _assign_capture_slots(self, center_x, center_y, enemy_yaw, radius):
        if len(self.capture_slots) == len(self.own_boats):
            return
        slots = []
        for index in range(len(self.own_boats)):
            angle = (
                enemy_yaw + math.pi
                + 2.0 * math.pi * index / len(self.own_boats)
            )
            slots.append((
                center_x + radius * math.cos(angle),
                center_y + radius * math.sin(angle),
            ))
        best_assignment = None
        best_cost = float('inf')
        for assignment in permutations(range(len(slots))):
            cost = sum(
                math.hypot(
                    own.x - slots[slot][0], own.y - slots[slot][1]
                )
                for own, slot in zip(self.own_boats, assignment)
            )
            if cost < best_cost:
                best_cost = cost
                best_assignment = assignment
        self.capture_slots = {
            own.name: slot
            for own, slot in zip(self.own_boats, best_assignment)
        }
        self.get_logger().info(
            'Capture slots locked: %s'
            % ', '.join(
                '%s->%d' % (name, slot)
                for name, slot in sorted(self.capture_slots.items())
            )
        )

    def _update_trails(self, now):
        if now - self.last_trail_update < 0.5:
            return
        self.last_trail_update = now
        for vessel in self.own_boats + self.enemy_boats:
            trail = self.trails[vessel.name]
            if not trail or math.hypot(
                vessel.x - trail[-1][0], vessel.y - trail[-1][1]
            ) >= 1.5:
                trail.append((vessel.x, vessel.y, 0.8))

    @staticmethod
    def _set_gz_color(marker, color):
        for material in (marker.material.diffuse, marker.material.ambient):
            material.r, material.g, material.b, material.a = color

    def _gz_line(self, marker_id, namespace, points, color, width=1.0):
        if len(points) < 2:
            return
        marker = GzMarker()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = GzMarker.ADD_MODIFY
        marker.type = GzMarker.LINE_STRIP
        marker.visibility = GzMarker.ALL
        marker.scale.x = width
        marker.pose.orientation.w = 1.0
        self._set_gz_color(marker, color)
        for x, y, z in points:
            point = marker.point.add()
            point.x, point.y, point.z = float(x), float(y), float(z)
        self.gz_marker_pub.publish(marker)

    def _gz_circle(self, marker_id, namespace, x, y, radius, color, z=0.7):
        points = []
        for index in range(97):
            angle = 2.0 * math.pi * index / 96.0
            points.append((
                x + radius * math.cos(angle),
                y + radius * math.sin(angle),
                z,
            ))
        self._gz_line(marker_id, namespace, points, color, 1.5)

    def _publish_gazebo_markers(self):
        self._gz_circle(
            1, 'mission_warning', self.base_x, self.base_y,
            float(self.get_parameter('trigger_radius').value),
            (1.0, 0.35, 0.05, 0.75),
        )
        self._gz_circle(
            2, 'mission_defense', self.base_x, self.base_y,
            float(self.get_parameter('defend_radius').value),
            (0.05, 0.45, 1.0, 0.8),
        )
        for index, vessel in enumerate(self.own_boats):
            self._gz_line(
                20 + index, 'command_links',
                [(self.base_x, self.base_y, 2.0), (vessel.x, vessel.y, 2.0)],
                (0.1, 0.75, 1.0, 0.38), 0.5,
            )
        for index, vessel in enumerate(self.own_boats + self.enemy_boats):
            color = (
                (0.1, 0.65, 1.0, 0.75)
                if vessel.name.startswith('own')
                else (1.0, 0.18, 0.12, 0.72)
            )
            self._gz_line(
                40 + index, 'vessel_trails', list(self.trails[vessel.name]),
                color, 1.0,
            )
        if self.retreat_enemy is not None:
            radius = (
                self.capture_ring_radius
                if self.capture_active
                else float(self.get_parameter('capture_start_radius').value)
            )
            self._gz_circle(
                10, 'capture_ring', self.retreat_enemy.x,
                self.retreat_enemy.y, radius,
                (1.0, 0.05, 0.05, 0.9), 1.2,
            )

    def _update_uav(self, now):
        if self.px4_uav_enabled:
            return
        if self.retreat_enemy is not None:
            alpha = 0.15
            self.uav_x += alpha * (self.retreat_enemy.x - self.uav_x)
            self.uav_y += alpha * (self.retreat_enemy.y - self.uav_y)
        else:
            angle = (now - self.start_time) * 0.075
            self.uav_x = self.base_x + 300.0 * math.cos(angle)
            self.uav_y = self.base_y + 220.0 * math.sin(angle)
        if now - self.last_uav_pose_update >= 0.2:
            self.last_uav_pose_update = now
            self._set_model_pose(
                str(self.get_parameter('world_name').value),
                'uav_01', self.uav_x, self.uav_y, self.uav_z, 0.0,
                timeout=40,
            )

    def _set_model_pose(self, world, name, x, y, z, yaw, timeout=250):
        request = GzPose()
        request.name = name
        request.position.x = float(x)
        request.position.y = float(y)
        request.position.z = float(z)
        request.orientation.z = math.sin(yaw * 0.5)
        request.orientation.w = math.cos(yaw * 0.5)
        executed, response = self.gz_node.request(
            '/world/%s/set_pose' % world,
            request,
            GzPose,
            Boolean,
            timeout,
        )
        return bool(executed and response.data)

    def _drive(self, vessel):
        if vessel.state in ('blocked', 'sunk', 'captured'):
            desired_linear = 0.0
            desired_angular = 0.0
        else:
            dx = vessel.target_x - vessel.x
            dy = vessel.target_y - vessel.y
            if vessel.name.startswith('own'):
                dx, dy = self._apply_avoidance(vessel, dx, dy)
            distance = math.hypot(dx, dy)
            desired = math.atan2(dy, dx)
            error = self._normalize(desired - vessel.yaw)
            desired_angular = max(-1.4, min(1.4, error * 1.9))
            desired_linear = vessel.speed * max(0.0, math.cos(error))
            if vessel.name.startswith('own'):
                speed_scale, emergency_turn = self._collision_speed_scale(vessel)
                desired_linear *= speed_scale
                desired_angular += emergency_turn
                desired_angular = max(-1.4, min(1.4, desired_angular))
            stop_distance = 8.0
            if vessel.state == 'guard':
                stop_distance = float(
                    self.get_parameter('guard_stop_distance').value
                )
            if distance < stop_distance:
                desired_linear = 0.0
        vessel.linear += max(-0.7, min(0.7, desired_linear - vessel.linear))
        vessel.angular += max(-0.12, min(0.12, desired_angular - vessel.angular))
        command = Twist()
        command.linear.x = float(vessel.linear)
        command.angular.z = float(vessel.angular)
        self.cmd_pubs[vessel.name].publish(command)

    def _collision_speed_scale(self, vessel):
        slow_radius = float(self.get_parameter('own_avoid_radius').value)
        stop_radius = float(
            self.get_parameter('collision_stop_radius').value
        )
        lookahead = float(
            self.get_parameter('collision_lookahead').value
        )
        own_vx = vessel.linear * math.cos(vessel.yaw)
        own_vy = vessel.linear * math.sin(vessel.yaw)
        scale = 1.0
        turn = 0.0
        for other in self.own_boats:
            if other is vessel:
                continue
            relative_x = other.x - vessel.x
            relative_y = other.y - vessel.y
            distance = math.hypot(relative_x, relative_y)
            other_vx = other.linear * math.cos(other.yaw)
            other_vy = other.linear * math.sin(other.yaw)
            future_x = relative_x + (other_vx - own_vx) * lookahead
            future_y = relative_y + (other_vy - own_vy) * lookahead
            future_distance = math.hypot(future_x, future_y)
            risk_distance = min(distance, future_distance)
            if risk_distance >= slow_radius:
                continue
            if distance <= stop_radius:
                scale = 0.0
                cross = (
                    math.cos(vessel.yaw) * relative_y
                    - math.sin(vessel.yaw) * relative_x
                )
                side = -1.0 if cross >= 0.0 else 1.0
                if vessel.name > other.name:
                    side *= -1.0
                turn += side * 1.4
                continue
            local_scale = max(
                0.0,
                min(1.0, (risk_distance - stop_radius) /
                    max(1.0, slow_radius - stop_radius)),
            )
            # The lexicographically larger vessel yields, preventing both
            # boats from repeatedly choosing opposite avoidance directions.
            if vessel.name > other.name:
                scale = min(scale, local_scale)
            else:
                scale = min(scale, max(0.3, local_scale))
            cross = math.cos(vessel.yaw) * relative_y - math.sin(vessel.yaw) * relative_x
            side = -1.0 if cross >= 0.0 else 1.0
            if vessel.name > other.name:
                side *= -1.0
            turn += side * (slow_radius - risk_distance) / slow_radius * 1.1
        return scale, max(-0.9, min(0.9, turn))

    def _apply_avoidance(self, vessel, dx, dy):
        base_dx = vessel.x - self.base_x
        base_dy = vessel.y - self.base_y
        base_distance = max(1e-3, math.hypot(base_dx, base_dy))
        avoid_radius = float(self.get_parameter('base_avoid_radius').value)
        if base_distance < avoid_radius:
            strength = 3.0 * (avoid_radius - base_distance + 12.0)
            dx += base_dx / base_distance * strength
            dy += base_dy / base_distance * strength
        own_radius = float(self.get_parameter('own_avoid_radius').value)
        for other in self.own_boats:
            if other is vessel:
                continue
            other_dx = vessel.x - other.x
            other_dy = vessel.y - other.y
            distance = math.hypot(other_dx, other_dy)
            if 1e-3 < distance < own_radius:
                lookahead = float(
                    self.get_parameter('collision_lookahead').value
                )
                predicted_x = other.x + other.linear * math.cos(other.yaw) * lookahead
                predicted_y = other.y + other.linear * math.sin(other.yaw) * lookahead
                other_dx = vessel.x - predicted_x
                other_dy = vessel.y - predicted_y
                predicted_distance = max(1e-3, math.hypot(other_dx, other_dy))
                strength = 2.8 * (own_radius - min(distance, predicted_distance))
                dx += other_dx / predicted_distance * strength
                dy += other_dy / predicted_distance * strength
        if time.monotonic() - self.base_radar_stamp < 1.0:
            radar_radius = float(
                self.get_parameter('radar_avoid_radius').value
            )
            heading_x = math.cos(vessel.yaw)
            heading_y = math.sin(vessel.yaw)
            for obstacle_x, obstacle_y in self.base_radar_points:
                if any(
                    math.hypot(obstacle_x - known.x, obstacle_y - known.y)
                    < 22.0
                    for known in self.own_boats + self.enemy_boats
                ):
                    continue
                away_x = vessel.x - obstacle_x
                away_y = vessel.y - obstacle_y
                distance = math.hypot(away_x, away_y)
                if distance < 14.0 or distance >= radar_radius:
                    continue
                forward = -away_x * heading_x - away_y * heading_y
                if forward < -8.0:
                    continue
                strength = 1.4 * (radar_radius - distance)
                dx += away_x / distance * strength
                dy += away_y / distance * strength
                side = 1.0 if vessel.name in ('own_01', 'own_03') else -1.0
                dx += -away_y / distance * strength * 0.45 * side
                dy += away_x / distance * strength * 0.45 * side
        return dx, dy

    def _segment_hits_base(self, x1, y1, x2, y2, radius):
        vx = x2 - x1
        vy = y2 - y1
        length_sq = vx * vx + vy * vy
        if length_sq < 1e-6:
            return self._distance_to_xy_base(x1, y1) < radius
        t = ((self.base_x - x1) * vx + (self.base_y - y1) * vy) / length_sq
        t = max(0.0, min(1.0, t))
        closest_x = x1 + t * vx
        closest_y = y1 + t * vy
        return self._distance_to_xy_base(closest_x, closest_y) < radius

    def _distance_to_xy_base(self, x, y):
        return math.hypot(x - self.base_x, y - self.base_y)

    def _publish(self):
        self._publish_pose_arrays()
        self._publish_status()
        self._publish_perception()
        self._publish_markers()

    def _publish_pose_arrays(self):
        stamp = self.get_clock().now().to_msg()
        own = PoseArray()
        own.header.stamp = stamp
        own.header.frame_id = 'map'
        enemy = PoseArray()
        enemy.header = own.header
        own.poses = [self._pose(vessel) for vessel in self.own_boats]
        enemy.poses = [self._pose(vessel) for vessel in self.enemy_boats]
        self.own_pub.publish(own)
        self.enemy_pub.publish(enemy)

    def _publish_status(self):
        threats = sum(
            1 for enemy in self.enemy_boats
            if self._distance_to_base(enemy)
            <= float(self.get_parameter('trigger_radius').value)
            and enemy.state in ('attack', 'blocked')
        )
        blocked = sum(enemy.state == 'blocked' for enemy in self.enemy_boats)
        if self.capture_complete:
            mode = 'complete'
        elif self.capture_active:
            mode = 'capture'
        elif self.retreat_enemy is not None:
            mode = 'tracking'
        elif threats:
            mode = 'guard'
        else:
            mode = 'patrol'
        enemy_states = ','.join(
            '%s:%s:%.1f' % (enemy.name, enemy.state, self._distance_to_base(enemy))
            for enemy in self.enemy_boats
        )
        status = (
            'mode=%s threats=%d blocked=%d defend_radius=%.1f '
            'trigger_radius=%.1f base_safety_radius=30.0 '
            'own_guard_speed=%.1f enemy_speed=%.1f guard_stop_distance=%.1f '
            'enemy_guard_stop_distance=%.1f intercept_stop_distance=%.1f '
            'guard_spacing=%.1f base=%.1f:%.1f enemy_states=%s'
            % (
                mode, threats, blocked,
                float(self.get_parameter('defend_radius').value),
                float(self.get_parameter('trigger_radius').value),
                float(self.get_parameter('own_guard_speed').value),
                float(self.get_parameter('enemy_speed').value),
                float(self.get_parameter('guard_stop_distance').value),
                float(self.get_parameter('enemy_guard_stop_distance').value),
                float(self.get_parameter('intercept_stop_distance').value),
                float(self.get_parameter('guard_spacing').value),
                self.base_x, self.base_y, enemy_states,
            )
        )
        status += ' capture_ring_radius=%.1f capture_phase=%s' % (
            self.capture_ring_radius, self.capture_phase
        )
        self.defense_status_pub.publish(String(data=status))
        target = self.retreat_enemy.name if self.retreat_enemy else 'none'
        self.capture_status_pub.publish(
            String(data='mode=%s target=%s' % (mode, target))
        )

    def _publish_perception(self):
        msg = TrackedObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        if self.retreat_enemy is not None and self.retreat_enemy.state in (
            'retreating', 'captured'
        ):
            enemy = self.retreat_enemy
            distance = math.hypot(enemy.x - self.uav_x, enemy.y - self.uav_y)
            if distance <= 520.0:
                track = TrackedObject()
                track.track_id = enemy.name
                track.first_seen = msg.header.stamp
                track.last_update = msg.header.stamp
                track.source_mask = TrackedObject.SOURCE_CAMERA
                track.classification = TrackedObject.CLASS_VESSEL
                track.pose.pose.position.x = enemy.x
                track.pose.pose.position.y = enemy.y
                track.pose.pose.orientation.w = 1.0
                track.twist.twist.linear.x = enemy.linear * math.cos(enemy.yaw)
                track.twist.twist.linear.y = enemy.linear * math.sin(enemy.yaw)
                track.dimensions.x = 24.0
                track.dimensions.y = 9.0
                track.dimensions.z = 8.0
                track.confidence = max(0.62, min(0.98, 1.0 - distance / 1200.0))
                msg.objects.append(track)
                selected = PoseStamped()
                selected.header = msg.header
                selected.pose = track.pose.pose
                self.selected_target_pub.publish(selected)
        self.target_pub.publish(msg)

    def _publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0
        for radius, color, ns in (
            (float(self.get_parameter('trigger_radius').value), (1.0, 0.45, 0.0), 'warning'),
            (float(self.get_parameter('defend_radius').value), (0.0, 0.45, 1.0), 'defense'),
        ):
            ring = Marker()
            ring.header.stamp = stamp
            ring.header.frame_id = 'map'
            ring.ns = ns
            ring.id = marker_id
            marker_id += 1
            ring.type = Marker.LINE_STRIP
            ring.action = Marker.ADD
            ring.scale.x = 2.2
            ring.color.r, ring.color.g, ring.color.b = color
            ring.color.a = 0.9
            ring.pose.orientation.w = 1.0
            for index in range(65):
                angle = 2.0 * math.pi * index / 64.0
                point = Point()
                point.x = self.base_x + radius * math.cos(angle)
                point.y = self.base_y + radius * math.sin(angle)
                point.z = 1.0
                ring.points.append(point)
            markers.markers.append(ring)
        for vessel in self.own_boats + self.enemy_boats:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'map'
            marker.ns = 'vessels'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self._pose(vessel)
            marker.pose.position.z = 2.0
            marker.scale.x = 20.0
            marker.scale.y = 8.0
            marker.scale.z = 5.0
            if vessel.name.startswith('own'):
                marker.color.b, marker.color.g = 1.0, 0.35
            elif vessel.state == 'retreating':
                marker.color.r, marker.color.g = 1.0, 0.65
            else:
                marker.color.r = 1.0
            marker.color.a = 0.95
            markers.markers.append(marker)
        uav = Marker()
        uav.header.stamp = stamp
        uav.header.frame_id = 'map'
        uav.ns = 'main_uav'
        uav.id = marker_id
        uav.type = Marker.SPHERE
        uav.action = Marker.ADD
        uav.pose.position.x = self.uav_x
        uav.pose.position.y = self.uav_y
        uav.pose.position.z = self.uav_z
        uav.pose.orientation.w = 1.0
        uav.scale.x = uav.scale.y = uav.scale.z = 12.0
        uav.color.b, uav.color.g, uav.color.a = 1.0, 0.6, 1.0
        uav.lifetime = Duration(sec=0, nanosec=300000000)
        markers.markers.append(uav)
        self.marker_pub.publish(markers)

    @staticmethod
    def _pose(vessel):
        pose = Pose()
        pose.position.x = vessel.x
        pose.position.y = vessel.y
        pose.orientation.z = math.sin(vessel.yaw * 0.5)
        pose.orientation.w = math.cos(vessel.yaw * 0.5)
        return pose

    def _distance_to_base(self, vessel):
        return math.hypot(vessel.x - self.base_x, vessel.y - self.base_y)

    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw(z, w):
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def main(args=None):
    rclpy.init(args=args)
    node = CooperativeResponseMission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
