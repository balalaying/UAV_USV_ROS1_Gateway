#!/usr/bin/env python3
import math
import random
from contextlib import suppress
from dataclasses import dataclass

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from gz.msgs10.marker_pb2 import Marker as GzMarker
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker as RvizMarker
from visualization_msgs.msg import MarkerArray


@dataclass
class Boat:
    name: str
    x: float
    y: float
    yaw: float
    target_x: float
    target_y: float
    state: str
    command_speed: float = 0.0
    smooth_linear: float = 0.0
    smooth_angular: float = 0.0


class DefenseDemo(Node):
    """Gazebo maritime defense algorithm demo."""

    def __init__(self):
        super().__init__('defense_demo')
        self.declare_parameter('topic_namespace', '')
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('own_count', 4)
        self.declare_parameter('enemy_count', 4)
        self.declare_parameter('defend_radius', 75.0)
        self.declare_parameter('trigger_radius', 190.0)
        self.declare_parameter('base_safety_radius', 18.0)
        self.declare_parameter('own_patrol_speed', 7.0)
        self.declare_parameter('own_guard_speed', 15.0)
        self.declare_parameter('enemy_speed', 4.5)
        self.declare_parameter('enemy_evasion_radius', 48.0)
        self.declare_parameter('enemy_evasion_gain', 36.0)
        self.declare_parameter('base_avoid_radius', 34.0)
        self.declare_parameter('base_avoid_gain', 2.8)
        self.declare_parameter('base_hard_keepout_radius', 16.0)
        self.declare_parameter('guard_spacing', 28.0)
        self.declare_parameter('guard_lead_distance', 12.0)
        self.declare_parameter('guard_target_alpha', 0.12)
        self.declare_parameter('guard_stop_distance', 20.0)
        self.declare_parameter('enemy_guard_stop_distance', 22.0)
        self.declare_parameter('intercept_stop_distance', 18.0)
        self.declare_parameter('own_avoid_radius', 30.0)
        self.declare_parameter('own_avoid_gain', 2.4)
        self.declare_parameter('own_yield_radius', 45.0)
        self.declare_parameter('own_brake_radius', 22.0)
        self.declare_parameter('linear_accel_limit', 8.0)
        self.declare_parameter('angular_accel_limit', 2.8)
        self.declare_parameter('guard_switch_penalty', 120.0)
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('rviz_marker_rate', 20.0)
        self.declare_parameter('gazebo_marker_rate', 5.0)
        self.declare_parameter('seed', 7)
        self.declare_parameter('pose_topic', '/world/defense/pose/info')

        random.seed(int(self.get_parameter('seed').value))
        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.own_boats = self._make_own_boats(
            int(self.get_parameter('own_count').value)
        )
        self.enemy_boats = self._make_enemy_boats(
            int(self.get_parameter('enemy_count').value)
        )
        self.last_time = self.get_clock().now()
        self.last_rviz_marker_time = self.get_clock().now()
        self.last_gazebo_marker_time = self.get_clock().now()
        self.guard_assignments = {}
        namespace = str(
            self.get_parameter('topic_namespace').value
        ).strip('/')
        self._topic = (
            (lambda name: '/%s%s' % (namespace, name))
            if namespace
            else (lambda name: name)
        )

        self.gz_node = GzTransportNode()
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.cmd_pubs = {}
        for boat in self.own_boats + self.enemy_boats:
            self.cmd_pubs[boat.name] = self.gz_node.advertise(
                '/model/%s/cmd_vel' % boat.name,
                Twist,
            )
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)
        self.marker_pub = self.gz_node.advertise('/marker', GzMarker)

        self.own_pose_pub = self.create_publisher(
            PoseArray, self._topic('/defense/own_ships'), 10
        )
        self.enemy_pose_pub = self.create_publisher(
            PoseArray, self._topic('/defense/enemy_ships'), 10
        )
        self.status_pub = self.create_publisher(
            String, self._topic('/defense/status'), 10
        )
        self.rviz_marker_pub = self.create_publisher(
            MarkerArray, self._topic('/defense/rviz_markers'), 10
        )

        update_rate = max(1.0, float(self.get_parameter('update_rate').value))
        self.create_timer(1.0 / update_rate, self._update)
        self.get_logger().info(
            'Gazebo defense demo started: own=%d enemy=%d'
            % (len(self.own_boats), len(self.enemy_boats))
        )

    def _make_own_boats(self, count):
        starts = [(-70.0, -42.0), (-66.0, -14.0), (-66.0, 16.0), (-70.0, 44.0)]
        boats = []
        for index in range(max(3, min(4, count))):
            offset_x, offset_y = starts[index]
            x = self.base_x + offset_x
            y = self.base_y + offset_y
            boats.append(
                Boat(
                    name='own_%02d' % (index + 1),
                    x=x,
                    y=y,
                    yaw=0.0,
                    target_x=x,
                    target_y=y,
                    state='patrol',
                )
            )
        return boats

    def _make_enemy_boats(self, count):
        starts = [
            (360.0, -160.0),
            (390.0, -60.0),
            (390.0, 65.0),
            (360.0, 165.0),
        ]
        boats = []
        for index in range(max(3, min(4, count))):
            offset_x, offset_y = starts[index]
            x = self.base_x + offset_x
            y = self.base_y + offset_y
            boats.append(
                Boat(
                    name='enemy_%02d' % (index + 1),
                    x=x,
                    y=y,
                    yaw=math.atan2(self.base_y - y, self.base_x - x),
                    target_x=self.base_x,
                    target_y=self.base_y,
                    state='attack',
                )
            )
        return boats

    def _update(self):
        now = self.get_clock().now()
        dt = max(
            0.001,
            min(0.2, (now - self.last_time).nanoseconds * 1e-9),
        )
        self.last_time = now

        defend_radius = max(
            5.0, float(self.get_parameter('defend_radius').value)
        )
        trigger_radius = max(
            defend_radius + 5.0,
            float(self.get_parameter('trigger_radius').value),
        )
        base_safety_radius = max(
            12.0, float(self.get_parameter('base_safety_radius').value)
        )
        enemy_speed = max(0.0, float(self.get_parameter('enemy_speed').value))
        enemy_evasion_radius = max(
            0.0, float(self.get_parameter('enemy_evasion_radius').value)
        )
        enemy_evasion_gain = max(
            0.0, float(self.get_parameter('enemy_evasion_gain').value)
        )
        base_avoid_radius = max(
            base_safety_radius + 8.0,
            float(self.get_parameter('base_avoid_radius').value),
        )
        base_avoid_gain = max(
            0.0, float(self.get_parameter('base_avoid_gain').value)
        )
        base_hard_keepout_radius = max(
            base_safety_radius + 2.0,
            float(self.get_parameter('base_hard_keepout_radius').value),
        )
        own_patrol_speed = max(
            0.0, float(self.get_parameter('own_patrol_speed').value)
        )
        own_guard_speed = max(
            0.0, float(self.get_parameter('own_guard_speed').value)
        )
        guard_spacing = max(0.0, float(self.get_parameter('guard_spacing').value))
        guard_lead_distance = max(
            0.0, float(self.get_parameter('guard_lead_distance').value)
        )
        guard_target_alpha = max(
            0.02,
            min(1.0, float(self.get_parameter('guard_target_alpha').value)),
        )
        guard_stop_distance = max(
            1.0, float(self.get_parameter('guard_stop_distance').value)
        )
        enemy_guard_stop_distance = max(
            1.0, float(self.get_parameter('enemy_guard_stop_distance').value)
        )
        intercept_stop_distance = max(
            1.0,
            float(self.get_parameter('intercept_stop_distance').value),
        )
        own_avoid_radius = max(
            5.0, float(self.get_parameter('own_avoid_radius').value)
        )
        own_avoid_gain = max(
            0.0, float(self.get_parameter('own_avoid_gain').value)
        )
        own_yield_radius = max(
            own_avoid_radius,
            float(self.get_parameter('own_yield_radius').value),
        )
        own_brake_radius = max(
            4.0, float(self.get_parameter('own_brake_radius').value)
        )

        for enemy in self.enemy_boats:
            enemy.command_speed = enemy_speed
            self._update_enemy_attack_target(
                enemy,
                base_safety_radius,
                enemy_evasion_radius,
                enemy_evasion_gain,
            )

        threats = [
            enemy
            for enemy in self.enemy_boats
            if self._distance(enemy.x, enemy.y, self.base_x, self.base_y)
            <= trigger_radius
        ]
        threats.sort(
            key=lambda boat: self._distance(
                boat.x, boat.y, self.base_x, self.base_y
            )
        )

        if threats:
            self._assign_guard_targets(
                threats,
                defend_radius,
                base_safety_radius,
                guard_spacing,
                guard_lead_distance,
                guard_target_alpha,
            )
            for boat in self.own_boats:
                boat.state = 'guard'
                boat.command_speed = own_guard_speed
            self._stop_blocked_enemies(
                threats,
                defend_radius,
                base_safety_radius,
                guard_lead_distance,
                guard_stop_distance,
                enemy_guard_stop_distance,
                intercept_stop_distance,
            )
        else:
            self.guard_assignments.clear()
            for boat in self.own_boats:
                boat.state = 'patrol'
                if self._distance(boat.x, boat.y, boat.target_x, boat.target_y) < 4.0:
                    boat.target_x, boat.target_y = self._random_patrol_target(
                        base_avoid_radius
                    )
                boat.command_speed = own_patrol_speed

        self._send_all_velocity(
            dt,
            own_avoid_radius,
            own_avoid_gain,
            own_yield_radius,
            own_brake_radius,
            base_avoid_radius,
            base_avoid_gain,
            base_hard_keepout_radius,
        )
        self._publish_pose_arrays()
        self._publish_status(
            threats,
            defend_radius,
            trigger_radius,
            base_safety_radius,
        )
        self._publish_gazebo_markers_throttled(
            threats,
            defend_radius,
            trigger_radius,
            base_safety_radius,
            guard_lead_distance,
        )
        self._publish_rviz_markers_throttled(
            threats,
            defend_radius,
            trigger_radius,
            base_safety_radius,
            guard_lead_distance,
        )

    def _update_enemy_attack_target(
        self,
        enemy,
        base_safety_radius,
        evasion_radius,
        evasion_gain,
    ):
        enemy.state = 'attack'
        distance_to_base = self._distance(enemy.x, enemy.y, self.base_x, self.base_y)
        from_base_x = enemy.x - self.base_x
        from_base_y = enemy.y - self.base_y
        from_base_length = max(1e-6, math.hypot(from_base_x, from_base_y))
        away_x = from_base_x / from_base_length
        away_y = from_base_y / from_base_length

        if distance_to_base <= base_safety_radius:
            side = -1.0 if enemy.name.endswith(('1', '3')) else 1.0
            tangent_x = -away_y * side
            tangent_y = away_x * side
            enemy.target_x = (
                self.base_x
                + away_x * (base_safety_radius + 26.0)
                + tangent_x * 38.0
            )
            enemy.target_y = (
                self.base_y
                + away_y * (base_safety_radius + 26.0)
                + tangent_y * 38.0
            )
            enemy.state = 'base_hold'
            enemy.command_speed *= 0.65
            return

        to_base_x = self.base_x - enemy.x
        to_base_y = self.base_y - enemy.y
        to_base_length = max(1e-6, math.hypot(to_base_x, to_base_y))
        forward_x = to_base_x / to_base_length
        forward_y = to_base_y / to_base_length
        tangent_x = -forward_y
        tangent_y = forward_x
        lateral_shift = 0.0

        for own in self.own_boats:
            own_dx = own.x - enemy.x
            own_dy = own.y - enemy.y
            own_distance = math.hypot(own_dx, own_dy)
            if own_distance < 1e-6 or own_distance >= evasion_radius:
                continue
            ahead = (own_dx * forward_x + own_dy * forward_y) > -8.0
            if not ahead:
                continue
            cross = forward_x * own_dy - forward_y * own_dx
            side = -1.0 if cross >= 0.0 else 1.0
            strength = (evasion_radius - own_distance) / evasion_radius
            lateral_shift += side * strength * evasion_gain

        if abs(lateral_shift) > 1.0:
            enemy.state = 'evade'
        enemy.target_x = self.base_x + tangent_x * lateral_shift
        enemy.target_y = self.base_y + tangent_y * lateral_shift

    def _assign_guard_targets(
        self,
        threats,
        defend_radius,
        base_safety_radius,
        guard_spacing,
        guard_lead_distance,
        guard_target_alpha,
    ):
        guard_points = []
        slots_per_threat = max(1, math.ceil(len(self.own_boats) / len(threats)))
        for threat in threats:
            guard_x, guard_y = self._guard_point_for_enemy(
                threat,
                defend_radius,
                base_safety_radius,
                guard_lead_distance,
            )
            direction_x = threat.x - self.base_x
            direction_y = threat.y - self.base_y
            length = max(1e-6, math.hypot(direction_x, direction_y))
            unit_x = direction_x / length
            unit_y = direction_y / length
            tangent_x = -unit_y
            tangent_y = unit_x
            for slot_index in range(slots_per_threat):
                offset = (slot_index - (slots_per_threat - 1) * 0.5) * guard_spacing
                slot_name = '%s_slot_%02d' % (threat.name, slot_index)
                guard_points.append(
                    (
                        guard_x + tangent_x * offset,
                        guard_y + tangent_y * offset,
                        threat,
                        slot_name,
                    )
                )

        assignments = self._assign_stable_boats(self.own_boats, guard_points)
        for boat, target_x, target_y, _threat, _slot_name in assignments:
            self._set_guard_target(boat, target_x, target_y, guard_target_alpha)

    def _assign_stable_boats(self, boats, guard_points):
        switch_penalty = max(
            0.0, float(self.get_parameter('guard_switch_penalty').value)
        )
        points_by_slot = {
            slot_name: (guard_x, guard_y, threat, slot_name)
            for guard_x, guard_y, threat, slot_name in guard_points
        }
        active_slots = set(points_by_slot)
        self.guard_assignments = {
            boat_name: slot_name
            for boat_name, slot_name in self.guard_assignments.items()
            if slot_name in active_slots
        }

        assignments = []
        assigned_slots = set()
        unassigned_boats = []
        for boat in boats:
            slot_name = self.guard_assignments.get(boat.name)
            if slot_name in points_by_slot:
                guard_x, guard_y, threat, slot_name = points_by_slot[slot_name]
                assignments.append((boat, guard_x, guard_y, threat, slot_name))
                assigned_slots.add(slot_name)
            else:
                unassigned_boats.append(boat)

        for boat in unassigned_boats:
            candidates = [
                point for point in guard_points if point[3] not in assigned_slots
            ]
            if not candidates:
                candidates = guard_points
            guard_x, guard_y, threat, slot_name = min(
                candidates,
                key=lambda point: (
                    self._distance(boat.x, boat.y, point[0], point[1])
                    + (
                        0.0
                        if self.guard_assignments.get(boat.name) == point[3]
                        else switch_penalty
                    )
                ),
            )
            self.guard_assignments[boat.name] = slot_name
            assigned_slots.add(slot_name)
            assignments.append((boat, guard_x, guard_y, threat, slot_name))
        return assignments

    def _set_guard_target(self, boat, target_x, target_y, alpha):
        if boat.state != 'guard':
            boat.target_x = target_x
            boat.target_y = target_y
            return
        boat.target_x += (target_x - boat.target_x) * alpha
        boat.target_y += (target_y - boat.target_y) * alpha

    def _guard_point_for_enemy(
        self,
        enemy,
        defend_radius,
        base_safety_radius=18.0,
        guard_lead_distance=0.0,
    ):
        threat_x, threat_y = self._predicted_enemy_point(
            enemy,
            guard_lead_distance,
        )
        direction_x = threat_x - self.base_x
        direction_y = threat_y - self.base_y
        length = max(1e-6, math.hypot(direction_x, direction_y))
        unit_x = direction_x / length
        unit_y = direction_y / length
        radius = defend_radius
        if length < defend_radius:
            radius = max(base_safety_radius + 10.0, length - 8.0)
        return (
            self.base_x + unit_x * radius,
            self.base_y + unit_y * radius,
        )

    def _predicted_enemy_point(self, enemy, lead_distance):
        if lead_distance <= 0.0:
            return enemy.x, enemy.y
        to_target_x = enemy.target_x - enemy.x
        to_target_y = enemy.target_y - enemy.y
        length = math.hypot(to_target_x, to_target_y)
        if length < 1e-6:
            return enemy.x, enemy.y
        lead = min(lead_distance, length)
        return (
            enemy.x + to_target_x / length * lead,
            enemy.y + to_target_y / length * lead,
        )

    def _stop_blocked_enemies(
        self,
        threats,
        defend_radius,
        base_safety_radius,
        guard_lead_distance,
        stop_distance,
        enemy_guard_stop_distance,
        intercept_stop_distance,
    ):
        for enemy in threats:
            guard_x, guard_y = self._guard_point_for_enemy(
                enemy,
                defend_radius,
                base_safety_radius,
                guard_lead_distance,
            )
            guarded = False
            for own in self.own_boats:
                if own.state != 'guard':
                    continue
                near_guard_point = (
                    self._distance(own.x, own.y, guard_x, guard_y)
                    <= stop_distance
                )
                enemy_near_guard_point = (
                    self._distance(enemy.x, enemy.y, guard_x, guard_y)
                    <= enemy_guard_stop_distance
                )
                close_to_defender = (
                    self._distance(own.x, own.y, enemy.x, enemy.y)
                    <= intercept_stop_distance
                )
                if (near_guard_point and enemy_near_guard_point) or close_to_defender:
                    guarded = True
                    break
            if guarded:
                enemy.state = 'blocked'
                enemy.target_x = enemy.x
                enemy.target_y = enemy.y
                enemy.command_speed = 0.0

    def _random_patrol_target(self, base_avoid_radius):
        for _ in range(40):
            x = random.uniform(-95.0, -18.0)
            y = random.uniform(-85.0, 85.0)
            if self._distance(x, y, self.base_x, self.base_y) > base_avoid_radius:
                return x, y
        return -85.0, random.choice([-1.0, 1.0]) * 70.0

    @staticmethod
    def _distance(x0, y0, x1, y1):
        return math.hypot(x1 - x0, y1 - y0)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_pose_v(self, msg):
        boats = {boat.name: boat for boat in self.own_boats + self.enemy_boats}
        for pose in msg.pose:
            boat = boats.get(pose.name)
            if boat is None:
                continue
            boat.x = pose.position.x
            boat.y = pose.position.y
            boat.yaw = self._yaw_from_quaternion(pose.orientation)

    def _send_all_velocity(
        self,
        dt,
        own_avoid_radius,
        own_avoid_gain,
        own_yield_radius,
        own_brake_radius,
        base_avoid_radius,
        base_avoid_gain,
        base_hard_keepout_radius,
    ):
        for boat_index, boat in enumerate(self.own_boats + self.enemy_boats):
            twist = Twist()
            dx = boat.target_x - boat.x
            dy = boat.target_y - boat.y
            base_push_x, base_push_y, inside_keepout = self._base_avoidance_vector(
                boat,
                base_avoid_radius,
                base_avoid_gain,
                base_hard_keepout_radius,
            )
            if inside_keepout:
                dx = base_push_x
                dy = base_push_y
            else:
                dx += base_push_x
                dy += base_push_y
            if boat in self.own_boats:
                avoid_x, avoid_y = self._own_avoidance_vector(
                    boat, own_avoid_radius, own_avoid_gain
                )
                dx += avoid_x
                dy += avoid_y
                own_index = boat_index
            else:
                own_index = -1
            desired_yaw = math.atan2(dy, dx)
            yaw_error = self._normalize_angle(desired_yaw - boat.yaw)
            distance = math.hypot(dx, dy)
            heading_scale = max(0.18, math.cos(yaw_error))
            speed = (
                0.0
                if distance < 0.8
                else min(boat.command_speed, distance * 0.55) * heading_scale
            )
            if boat in self.own_boats:
                speed *= self._own_yield_scale(
                    boat,
                    own_index,
                    desired_yaw,
                    own_yield_radius,
                    own_brake_radius,
                )
            angular = float(max(-1.6, min(1.6, yaw_error * 2.2)))
            linear_step = (
                max(0.1, float(self.get_parameter('linear_accel_limit').value))
                * dt
            )
            angular_step = (
                max(0.1, float(self.get_parameter('angular_accel_limit').value))
                * dt
            )
            boat.smooth_linear = self._slew(
                boat.smooth_linear, float(speed), linear_step
            )
            boat.smooth_angular = self._slew(
                boat.smooth_angular, angular, angular_step
            )
            twist.linear.x = float(boat.smooth_linear)
            twist.angular.z = float(boat.smooth_angular)
            self.cmd_pubs[boat.name].publish(twist)

    @staticmethod
    def _slew(current, target, max_step):
        if target > current + max_step:
            return current + max_step
        if target < current - max_step:
            return current - max_step
        return target

    def _base_avoidance_vector(
        self,
        boat,
        avoid_radius,
        avoid_gain,
        hard_keepout_radius,
    ):
        dx = boat.x - self.base_x
        dy = boat.y - self.base_y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return -avoid_radius, 0.0, True
        if distance >= avoid_radius:
            return 0.0, 0.0, False

        unit_x = dx / distance
        unit_y = dy / distance
        strength = (avoid_radius - distance) / avoid_radius
        push = strength * avoid_radius * avoid_gain
        if distance <= hard_keepout_radius:
            push = max(push, avoid_radius)
        return unit_x * push, unit_y * push, distance <= hard_keepout_radius

    def _own_avoidance_vector(self, boat, avoid_radius, avoid_gain):
        push_x = 0.0
        push_y = 0.0
        for other in self.own_boats:
            if other.name == boat.name:
                continue
            dx = boat.x - other.x
            dy = boat.y - other.y
            distance = math.hypot(dx, dy)
            if distance < 1e-6 or distance >= avoid_radius:
                continue
            strength = (avoid_radius - distance) / avoid_radius
            push_x += dx / distance * strength * avoid_radius * avoid_gain
            push_y += dy / distance * strength * avoid_radius * avoid_gain
        return push_x, push_y

    def _own_yield_scale(
        self,
        boat,
        own_index,
        desired_yaw,
        yield_radius,
        brake_radius,
    ):
        scale = 1.0
        forward_x = math.cos(desired_yaw)
        forward_y = math.sin(desired_yaw)
        for other_index, other in enumerate(self.own_boats):
            if other.name == boat.name:
                continue
            dx = other.x - boat.x
            dy = other.y - boat.y
            distance = math.hypot(dx, dy)
            if distance >= yield_radius:
                continue

            front_dot = (dx * forward_x + dy * forward_y) / max(distance, 1e-6)
            crossing_risk = front_dot > -0.25
            lower_priority = own_index > other_index
            if distance <= brake_radius and crossing_risk:
                scale = min(scale, 0.0 if lower_priority else 0.25)
            elif crossing_risk and lower_priority:
                soft_scale = max(
                    0.18,
                    (distance - brake_radius) / max(
                        1.0, yield_radius - brake_radius
                    ),
                )
                scale = min(scale, soft_scale)
        return scale

    def _publish_pose_arrays(self):
        own = PoseArray()
        enemy = PoseArray()
        own.header.stamp = self.get_clock().now().to_msg()
        own.header.frame_id = 'map'
        enemy.header = own.header
        for boat in self.own_boats:
            own.poses.append(self._boat_pose(boat))
        for boat in self.enemy_boats:
            enemy.poses.append(self._boat_pose(boat))
        self.own_pose_pub.publish(own)
        self.enemy_pose_pub.publish(enemy)

    def _boat_pose(self, boat):
        pose = Pose()
        pose.position.x = boat.x
        pose.position.y = boat.y
        pose.position.z = 0.0
        qz, qw = math.sin(boat.yaw * 0.5), math.cos(boat.yaw * 0.5)
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _publish_status(
        self,
        threats,
        defend_radius,
        trigger_radius,
        base_safety_radius,
    ):
        msg = String()
        blocked = sum(1 for enemy in self.enemy_boats if enemy.state == 'blocked')
        own_targets = ','.join(
            '%s:%.1f:%.1f:%s'
            % (boat.name, boat.target_x, boat.target_y, boat.state)
            for boat in self.own_boats
        )
        enemy_states = ','.join(
            '%s:%s:%.1f'
            % (
                enemy.name,
                enemy.state,
                self._distance(enemy.x, enemy.y, self.base_x, self.base_y),
            )
            for enemy in self.enemy_boats
        )
        msg.data = (
            'mode=%s threats=%d blocked=%d defend_radius=%.1f '
            'trigger_radius=%.1f base_safety_radius=%.1f '
            'own_guard_speed=%.1f enemy_speed=%.1f guard_stop_distance=%.1f '
            'enemy_guard_stop_distance=%.1f intercept_stop_distance=%.1f '
            'guard_spacing=%.1f '
            'base=%.1f:%.1f own_targets=%s enemy_states=%s'
            % (
                'guard' if threats else 'patrol',
                len(threats),
                blocked,
                defend_radius,
                trigger_radius,
                base_safety_radius,
                float(self.get_parameter('own_guard_speed').value),
                float(self.get_parameter('enemy_speed').value),
                float(self.get_parameter('guard_stop_distance').value),
                float(self.get_parameter('enemy_guard_stop_distance').value),
                float(self.get_parameter('intercept_stop_distance').value),
                float(self.get_parameter('guard_spacing').value),
                self.base_x,
                self.base_y,
                own_targets,
                enemy_states,
            )
        )
        self.status_pub.publish(msg)

    def _publish_gazebo_markers_throttled(
        self,
        threats,
        defend_radius,
        trigger_radius,
        base_safety_radius,
        guard_lead_distance,
    ):
        rate = max(1.0, float(self.get_parameter('gazebo_marker_rate').value))
        now = self.get_clock().now()
        elapsed = (now - self.last_gazebo_marker_time).nanoseconds * 1e-9
        if elapsed < 1.0 / rate:
            return
        self.last_gazebo_marker_time = now
        self._publish_gazebo_markers(
            threats,
            defend_radius,
            trigger_radius,
            base_safety_radius,
            guard_lead_distance,
        )

    def _publish_gazebo_markers(
        self,
        threats,
        defend_radius,
        trigger_radius,
        base_safety_radius,
        guard_lead_distance,
    ):
        self._marker_delete_all()
        self._marker_disc(
            1,
            'warning_area',
            trigger_radius,
            0.06,
            (1.0, 0.55, 0.0, 0.14),
        )
        self._marker_disc(
            2,
            'guard_area',
            defend_radius,
            0.12,
            (0.0, 0.45, 1.0, 0.18),
        )
        self._marker_disc(
            5,
            'base_safety_area',
            base_safety_radius,
            0.18,
            (1.0, 0.0, 0.0, 0.16),
        )
        self._marker_circle(
            3,
            'defend_radius',
            defend_radius,
            (0.0, 0.55, 1.0, 1.0),
            width=2.8,
            z=3.0,
        )
        self._marker_circle(
            4,
            'trigger_radius',
            trigger_radius,
            (1.0, 0.25, 0.0, 1.0),
            width=3.5,
            z=4.0,
        )
        self._marker_circle(
            6,
            'base_safety_radius',
            base_safety_radius,
            (1.0, 0.0, 0.0, 1.0),
            width=2.8,
            z=3.6,
        )
        next_id = 10
        for enemy in self.enemy_boats:
            guard_x, guard_y = self._guard_point_for_enemy(
                enemy,
                defend_radius,
                base_safety_radius,
                guard_lead_distance,
            )
            self._marker_line(
                next_id,
                'enemy_to_base',
                [(enemy.x, enemy.y, 0.8), (self.base_x, self.base_y, 0.8)],
                (1.0, 0.1, 0.0, 1.0 if enemy.state != 'blocked' else 0.35),
            )
            next_id += 1
            self._marker_sphere(
                next_id,
                'intercept_points',
                guard_x,
                guard_y,
                2.0,
                (0.0, 1.0, 1.0, 1.0),
                scale=7.0,
            )
            next_id += 1
            if enemy.state == 'blocked':
                self._marker_text(
                    next_id,
                    'blocked_label',
                    enemy.x,
                    enemy.y,
                    7.0,
                    'STOP',
                    (1.0, 0.0, 0.0, 1.0),
                    4.8,
                )
                next_id += 1
        for boat in self.own_boats:
            self._marker_sphere(
                next_id,
                'guard_target',
                boat.target_x,
                boat.target_y,
                1.0,
                (0.0, 0.9, 1.0, 0.95 if boat.state == 'guard' else 0.3),
                scale=6.0,
            )
            next_id += 1
            if boat.state == 'guard':
                self._marker_line(
                    next_id,
                    'own_to_target',
                    [(boat.x, boat.y, 0.7), (boat.target_x, boat.target_y, 0.7)],
                    (0.0, 0.8, 1.0, 0.85),
                )
                next_id += 1
        mode = 'GUARD' if threats else 'PATROL'
        blocked = sum(1 for enemy in self.enemy_boats if enemy.state == 'blocked')
        self._marker_text(
            100,
            'status',
            -150.0,
            125.0,
            8.0,
            '%s | threats=%d | blocked=%d | defend=%.0fm | warning=%.0fm | safe=%.0fm'
            % (
                mode,
                len(threats),
                blocked,
                defend_radius,
                trigger_radius,
                base_safety_radius,
            ),
        )

    def _publish_rviz_markers_throttled(
        self,
        threats,
        defend_radius,
        trigger_radius,
        base_safety_radius,
        guard_lead_distance,
    ):
        rate = max(1.0, float(self.get_parameter('rviz_marker_rate').value))
        now = self.get_clock().now()
        elapsed = (now - self.last_rviz_marker_time).nanoseconds * 1e-9
        if elapsed < 1.0 / rate:
            return
        self.last_rviz_marker_time = now
        self._publish_rviz_markers(
            threats,
            defend_radius,
            trigger_radius,
            base_safety_radius,
            guard_lead_distance,
        )

    def _base_marker(self, marker_id, ns, marker_type):
        marker = GzMarker()
        marker.action = GzMarker.ADD_MODIFY
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.visibility = GzMarker.ALL
        marker.pose.orientation.w = 1.0
        return marker

    def _set_color(self, marker, color):
        marker.material.diffuse.r = float(color[0])
        marker.material.diffuse.g = float(color[1])
        marker.material.diffuse.b = float(color[2])
        marker.material.diffuse.a = float(color[3])
        marker.material.ambient.r = float(color[0])
        marker.material.ambient.g = float(color[1])
        marker.material.ambient.b = float(color[2])
        marker.material.ambient.a = float(color[3])

    def _marker_delete_all(self):
        marker = GzMarker()
        marker.action = GzMarker.DELETE_ALL
        self.marker_pub.publish(marker)

    def _marker_circle(self, marker_id, ns, radius, color, width=1.0, z=0.4):
        marker = self._base_marker(marker_id, ns, GzMarker.LINE_STRIP)
        marker.scale.x = width
        self._set_color(marker, color)
        for index in range(145):
            angle = index * 2.0 * math.pi / 144.0
            point = marker.point.add()
            point.x = self.base_x + radius * math.cos(angle)
            point.y = self.base_y + radius * math.sin(angle)
            point.z = z
        self.marker_pub.publish(marker)

    def _marker_disc(self, marker_id, ns, radius, height, color):
        marker = self._base_marker(marker_id, ns, GzMarker.CYLINDER)
        marker.pose.position.x = self.base_x
        marker.pose.position.y = self.base_y
        marker.pose.position.z = 0.08
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = height
        self._set_color(marker, color)
        self.marker_pub.publish(marker)

    def _marker_line(self, marker_id, ns, points, color):
        marker = self._base_marker(marker_id, ns, GzMarker.LINE_STRIP)
        marker.scale.x = 1.4
        self._set_color(marker, color)
        for x, y, z in points:
            point = marker.point.add()
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
        self.marker_pub.publish(marker)

    def _marker_sphere(self, marker_id, ns, x, y, z, color, scale=4.0):
        marker = self._base_marker(marker_id, ns, GzMarker.SPHERE)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = max(1.0, scale * 0.35)
        self._set_color(marker, color)
        self.marker_pub.publish(marker)

    def _marker_text(
        self,
        marker_id,
        ns,
        x,
        y,
        z,
        text,
        color=(0.0, 0.0, 0.0, 1.0),
        scale=5.0,
    ):
        marker = self._base_marker(marker_id, ns, GzMarker.TEXT)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.z = scale
        self._set_color(marker, color)
        marker.text = text
        self.marker_pub.publish(marker)

    def _publish_rviz_markers(
        self,
        threats,
        defend_radius,
        trigger_radius,
        base_safety_radius,
        guard_lead_distance,
    ):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        marker_id = self._rviz_area(
            markers,
            stamp,
            marker_id,
            'warning_area',
            trigger_radius,
            (1.0, 0.45, 0.0, 0.18),
        )
        marker_id = self._rviz_area(
            markers,
            stamp,
            marker_id,
            'guard_area',
            defend_radius,
            (0.0, 0.45, 1.0, 0.22),
        )
        marker_id = self._rviz_area(
            markers,
            stamp,
            marker_id,
            'base_safety_area',
            base_safety_radius,
            (1.0, 0.0, 0.0, 0.18),
        )
        marker_id = self._rviz_circle(
            markers,
            stamp,
            marker_id,
            'warning_line',
            trigger_radius,
            (1.0, 0.12, 0.0, 1.0),
            1.8,
        )
        marker_id = self._rviz_circle(
            markers,
            stamp,
            marker_id,
            'guard_line',
            defend_radius,
            (0.0, 0.55, 1.0, 1.0),
            1.5,
        )
        marker_id = self._rviz_circle(
            markers,
            stamp,
            marker_id,
            'base_safety_line',
            base_safety_radius,
            (1.0, 0.0, 0.0, 1.0),
            1.4,
        )
        marker_id = self._rviz_base(markers, stamp, marker_id)

        for enemy in self.enemy_boats:
            guard_x, guard_y = self._guard_point_for_enemy(
                enemy,
                defend_radius,
                base_safety_radius,
                guard_lead_distance,
            )
            marker_id = self._rviz_ship(
                markers,
                stamp,
                marker_id,
                enemy,
                (1.0, 0.05, 0.02, 1.0),
                'enemy',
            )
            marker_id = self._rviz_line(
                markers,
                stamp,
                marker_id,
                'attack_lines',
                [(enemy.x, enemy.y, 0.7), (self.base_x, self.base_y, 0.7)],
                (1.0, 0.05, 0.0, 0.9 if enemy.state != 'blocked' else 0.3),
                0.6,
            )
            marker_id = self._rviz_sphere(
                markers,
                stamp,
                marker_id,
                'intercept_points',
                guard_x,
                guard_y,
                1.2,
                5.0,
                (0.0, 0.95, 1.0, 1.0),
            )
            if enemy.state == 'blocked':
                marker_id = self._rviz_text(
                    markers,
                    stamp,
                    marker_id,
                    'blocked',
                    enemy.x,
                    enemy.y,
                    6.0,
                    'STOP',
                    (1.0, 0.0, 0.0, 1.0),
                    5.0,
                )

        for boat in self.own_boats:
            marker_id = self._rviz_ship(
                markers,
                stamp,
                marker_id,
                boat,
                (0.05, 0.35, 1.0, 1.0),
                'own',
            )
            marker_id = self._rviz_sphere(
                markers,
                stamp,
                marker_id,
                'own_targets',
                boat.target_x,
                boat.target_y,
                1.1,
                4.0,
                (0.0, 0.8, 1.0, 0.9),
            )
            if boat.state == 'guard':
                marker_id = self._rviz_line(
                    markers,
                    stamp,
                    marker_id,
                    'own_to_target',
                    [(boat.x, boat.y, 0.8), (boat.target_x, boat.target_y, 0.8)],
                    (0.0, 0.7, 1.0, 0.85),
                    0.35,
                )

        blocked = sum(1 for enemy in self.enemy_boats if enemy.state == 'blocked')
        self._rviz_text(
            markers,
            stamp,
            marker_id,
            'status',
            -150.0,
            130.0,
            10.0,
            'Defense Demo | threats=%d | blocked=%d | guard %.0fm | warning %.0fm | safe %.0fm'
            % (
                len(threats),
                blocked,
                defend_radius,
                trigger_radius,
                base_safety_radius,
            ),
            (0.0, 0.0, 0.0, 1.0),
            6.0,
        )
        self.rviz_marker_pub.publish(markers)

    def _rviz_marker(self, stamp, marker_id, ns, marker_type):
        marker = RvizMarker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = RvizMarker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.nanosec = 250000000
        return marker

    @staticmethod
    def _rviz_color(marker, color):
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

    def _rviz_area(self, markers, stamp, marker_id, ns, radius, color):
        marker = self._rviz_marker(stamp, marker_id, ns, RvizMarker.CYLINDER)
        marker.pose.position.x = self.base_x
        marker.pose.position.y = self.base_y
        marker.pose.position.z = -0.02
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = 0.04
        self._rviz_color(marker, color)
        markers.markers.append(marker)
        return marker_id + 1

    def _rviz_circle(self, markers, stamp, marker_id, ns, radius, color, width):
        marker = self._rviz_marker(stamp, marker_id, ns, RvizMarker.LINE_STRIP)
        marker.scale.x = width
        self._rviz_color(marker, color)
        for index in range(145):
            angle = index * 2.0 * math.pi / 144.0
            point = Point()
            point.x = self.base_x + radius * math.cos(angle)
            point.y = self.base_y + radius * math.sin(angle)
            point.z = 0.8
            marker.points.append(point)
        markers.markers.append(marker)
        return marker_id + 1

    def _rviz_base(self, markers, stamp, marker_id):
        marker = self._rviz_marker(stamp, marker_id, 'base', RvizMarker.CYLINDER)
        marker.pose.position.x = self.base_x
        marker.pose.position.y = self.base_y
        marker.pose.position.z = 1.0
        marker.scale.x = 16.0
        marker.scale.y = 16.0
        marker.scale.z = 2.0
        self._rviz_color(marker, (0.0, 0.8, 0.25, 1.0))
        markers.markers.append(marker)
        return marker_id + 1

    def _rviz_ship(self, markers, stamp, marker_id, boat, color, ns):
        marker = self._rviz_marker(stamp, marker_id, ns, RvizMarker.CUBE)
        marker.pose.position.x = boat.x
        marker.pose.position.y = boat.y
        marker.pose.position.z = 0.8
        marker.pose.orientation.z = math.sin(boat.yaw * 0.5)
        marker.pose.orientation.w = math.cos(boat.yaw * 0.5)
        marker.scale.x = 8.0
        marker.scale.y = 3.0
        marker.scale.z = 1.4
        self._rviz_color(marker, color)
        markers.markers.append(marker)
        marker_id += 1
        return self._rviz_text(
            markers,
            stamp,
            marker_id,
            ns + '_label',
            boat.x,
            boat.y,
            4.5,
            '%s %s' % (boat.name, boat.state),
            (0.0, 0.0, 0.0, 1.0),
            3.0,
        )

    def _rviz_line(self, markers, stamp, marker_id, ns, points, color, width):
        marker = self._rviz_marker(stamp, marker_id, ns, RvizMarker.LINE_STRIP)
        marker.scale.x = width
        self._rviz_color(marker, color)
        for x, y, z in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            marker.points.append(point)
        markers.markers.append(marker)
        return marker_id + 1

    def _rviz_sphere(self, markers, stamp, marker_id, ns, x, y, z, scale, color):
        marker = self._rviz_marker(stamp, marker_id, ns, RvizMarker.SPHERE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = max(1.0, scale * 0.35)
        self._rviz_color(marker, color)
        markers.markers.append(marker)
        return marker_id + 1

    def _rviz_text(self, markers, stamp, marker_id, ns, x, y, z, text, color, scale):
        marker = self._rviz_marker(
            stamp, marker_id, ns, RvizMarker.TEXT_VIEW_FACING
        )
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.z = scale
        marker.text = text
        self._rviz_color(marker, color)
        markers.markers.append(marker)
        return marker_id + 1


def main(args=None):
    rclpy.init(args=args)
    node = DefenseDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with suppress(Exception, KeyboardInterrupt):
            node.destroy_node()
        with suppress(Exception, KeyboardInterrupt):
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
