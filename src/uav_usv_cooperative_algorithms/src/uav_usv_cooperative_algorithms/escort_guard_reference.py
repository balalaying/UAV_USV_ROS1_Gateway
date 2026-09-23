"""单目标随机方向出现与随机角度环绕的 UAV/USV 护航守卫仿真。

运行示例：

    python 护航守卫727+3+3+1_Python39.py
    python 护航守卫727+3+3+1_Python39.py --sensor-radius 12
    python 护航守卫727+3+3+1_Python39.py --reserve-count 2

程序只考虑一个敌方目标。目标从随机方向逐步接近，进入感知范围后触发
守卫编队；编队形成后，目标沿自身轨道按随机角度段进行顺时针或逆时针
环绕，并在每段结束时随机决定是否切换方向。
"""
from __future__ import annotations
import argparse
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/uav_usv_matplotlib")

# ---------------------------------------------------------------------------
# 用户可直接修改的编队参数
# ---------------------------------------------------------------------------
NUM_UAV = 3
NUM_USV = 3
ESCORT_RESERVE_COUNT = 0 # 允许设置为 0 或 2

WORLD_X_MIN = -80.0
WORLD_X_MAX = 80.0
WORLD_Y_MIN = -50.0
WORLD_Y_MAX = 50.0
OWN_START_X = -52.0
OWN_START_Y = 0.0

LOCAL_UAV_SIZE = 220.0
LOCAL_USV_SIZE = 220.0
GLOBAL_UAV_SIZE = 28.0
GLOBAL_USV_SIZE = 34.0
GLOBAL_ENEMY_SCALE = 0.45

SUPPORT_GUARD_RADIUS = 4.2
SUPPORT_ARC_HALF_ANGLE_DEG = 65.0
SUPPORT_ARC_REAR_OFFSET_DEG = 180.0

DEFAULT_SENSOR_RADIUS = 12.0

RANDOM_ORBIT_MIN_ANGLE_DEG = 25.0
RANDOM_ORBIT_MAX_ANGLE_DEG = 120.0
RANDOM_ORBIT_MIN_SPEED = 0.012
RANDOM_ORBIT_MAX_SPEED = 0.024
RANDOM_ORBIT_REVERSE_PROBABILITY = 0.65

WING_ARRIVAL_TOLERANCE = 0.35
WING_READY_RATIO = 0.80

EPS = 1e-12
DETECTED_STATES = {"detected", "forming", "orbiting"}
STATE_LABELS = {
    "waiting": "等待出现",
    "approaching": "接近中",
    "detected": "已感知",
    "forming": "守卫编队中",
    "orbiting": "环绕机动",
}


def normalize(v: np.ndarray, fallback: Optional[np.ndarray] = None) -> np.ndarray:
    arr = np.asarray(v, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm <= EPS:
        if fallback is None:
            return np.zeros_like(arr)
        fb = np.asarray(fallback, dtype=float)
        fb_norm = float(np.linalg.norm(fb))
        return fb / fb_norm if fb_norm > EPS else np.zeros_like(arr)
    return arr / norm


def rotate90(v: np.ndarray) -> np.ndarray:
    arr = np.asarray(v, dtype=float)
    return np.array([-arr[1], arr[0]], dtype=float)


def wrapped_angle_distance(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def compute_blocker_point(
    own: np.ndarray,
    enemy: np.ndarray,
    ratio: float = 0.38,
    r_min: float = 2.2,
    r_max: float = 4.0,
    fallback_direction: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, float]:
    """计算严格位于敌我连线内部的动态核心阻断点。"""
    if not (0.0 < ratio < 1.0):
        raise ValueError("ratio must be in (0, 1)")
    if not (0.0 <= r_min <= r_max):
        raise ValueError("expected 0 <= r_min <= r_max")
    own = np.asarray(own, dtype=float)
    enemy = np.asarray(enemy, dtype=float)
    delta = enemy - own
    distance = float(np.linalg.norm(delta))
    if distance <= EPS:
        delta = normalize(
            np.array([1.0, 0.0]) if fallback_direction is None else fallback_direction,
            np.array([1.0, 0.0]),
        )
        distance = 1.0
    requested = float(np.clip(ratio * distance, r_min, r_max))
    radius = min(max(distance * 1e-9, requested), distance * (1.0 - 1e-9))
    t = radius / distance
    return own + t * delta, float(t)


@dataclass
class Platform:
    identifier: str
    kind: str
    position: np.ndarray
    max_speed: float
    gain: float
    role: str = "escort"
    goal: Optional[np.ndarray] = None
    assigned_threat_id: Optional[int] = None


@dataclass
class ThreatTask:
    threat_id: int
    spawn_frame: int
    spawn_angle: float
    spawn_radius: float
    position: np.ndarray
    state: str = "waiting"
    current_speed_limit: float = 0.0
    detected_frame: Optional[int] = None
    blocker_point: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    blocker_t: float = math.nan
    guard_quota: int = 0
    core_guard_index: Optional[int] = None
    wing_guard_indices: List[int] = field(default_factory=list)
    wing_slot_by_index: Dict[int, int] = field(default_factory=dict)
    core_motion_state: str = "idle"
    core_dispatch_origin: Optional[np.ndarray] = None
    core_dispatch_initial_distance: float = 0.0
    core_trajectory: List[np.ndarray] = field(default_factory=list)
    controlled_radius: float = 0.0
    controlled_angle: float = 0.0
    orbit_direction: int = 1
    orbit_segment_angle: float = 0.0
    orbit_segment_remaining: float = 0.0
    orbit_angular_speed_current: float = 0.0
    orbit_segment_count: int = 0
    orbit_direction_changes: int = 0


class EscortGuardSimulator:
    """单个移动威胁的感知、编队和随机环绕持续盯防控制器。"""

    def __init__(
        self,
        *,
        sensor_radius: float = DEFAULT_SENSOR_RADIUS,
        seed: int = 42,
        num_uav: int = NUM_UAV,
        num_usv: int = NUM_USV,
        escort_reserve_count: int = ESCORT_RESERVE_COUNT,
        ring_radius: float = 4.0,
        guard_arc_radius: float = 5.2,
        support_guard_radius: float = SUPPORT_GUARD_RADIUS,
        support_arc_half_angle_deg: float = SUPPORT_ARC_HALF_ANGLE_DEG,
        support_arc_rear_offset_deg: float = SUPPORT_ARC_REAR_OFFSET_DEG,
        guard_arc_half_angle_deg: float = 32.0,
        max_guard_arc_half_angle_deg: float = 75.0,
        minimum_guard_spacing: float = 1.05,
        blocker_ratio: float = 0.38,
        blocker_r_min: float = 2.2,
        blocker_r_max: float = 4.0,
        core_arrival_tolerance: float = 0.24,
        wing_arrival_tolerance: float = WING_ARRIVAL_TOLERANCE,
        wing_ready_ratio: float = WING_READY_RATIO,
        enemy_approach_speed: float = 0.10,
        enemy_forming_speed: float = 0.14,
        enemy_controlled_speed: float = 0.30,
        enemy_min_radius: float = 6.6,
        random_orbit_min_angle_deg: float = RANDOM_ORBIT_MIN_ANGLE_DEG,
        random_orbit_max_angle_deg: float = RANDOM_ORBIT_MAX_ANGLE_DEG,
        random_orbit_min_speed: float = RANDOM_ORBIT_MIN_SPEED,
        random_orbit_max_speed: float = RANDOM_ORBIT_MAX_SPEED,
        random_orbit_reverse_probability: float = RANDOM_ORBIT_REVERSE_PROBABILITY,
        avoidance_mode: str = "auto",
        avoid_distance: float = 3.8,
        forward_shift: float = 2.2,
        own_max_speed: float = 0.075,
        own_gain: float = 0.08,
        cruise_speed: float = 0.035,
        safe_distance: float = 0.75,
        repulsion_gain: float = 0.025,
        own_target_avoid_radius: float = 2.0,
        core_trail_length: int = 240,
        dt: float = 1.0,
    ) -> None:
        platform_count = int(num_uav) + int(num_usv)
        if num_uav < 0 or num_usv < 0 or platform_count < 6:
            raise ValueError("The UAV/USV total must be greater than 6")
        if escort_reserve_count not in {0, 2}:
            raise ValueError("escort_reserve_count must be 0 or 2")
        if support_guard_radius <= own_target_avoid_radius:
            raise ValueError("support_guard_radius must exceed the own-target safety radius")
        if not (0.0 <= support_arc_half_angle_deg < 180.0):
            raise ValueError("support_arc_half_angle_deg must be in [0, 180)")
        if sensor_radius <= max(guard_arc_radius, support_guard_radius, own_target_avoid_radius):
            raise ValueError("sensor_radius must exceed guard and safety radii")
            raise ValueError("sensor_radius is too small for two separated guard tracks")
        if not (0.0 < wing_ready_ratio <= 1.0):
            raise ValueError("wing_ready_ratio must be in (0, 1]")
        if not (0.0 < random_orbit_min_angle_deg <= random_orbit_max_angle_deg <= 360.0):
            raise ValueError("random orbit angles must satisfy 0 < min <= max <= 360")
        if not (0.0 < random_orbit_min_speed <= random_orbit_max_speed):
            raise ValueError("random orbit speeds must satisfy 0 < min <= max")
        if not (0.0 <= random_orbit_reverse_probability <= 1.0):
            raise ValueError("random_orbit_reverse_probability must be in [0, 1]")
            raise ValueError("invalid stagger delay range")

        self.seed = int(seed)
        self.reset_count = 0
        self.rng = np.random.default_rng(self.seed)
        self.num_uav = int(num_uav)
        self.num_usv = int(num_usv)
        self.enemy_count = 1
        self.spawn_mode = "single_random_direction"
        self.enemy_motion = "random_segment_orbit"
        self.sensor_radius = float(sensor_radius)
        self.escort_reserve_count = int(escort_reserve_count)
        self.ring_radius = float(ring_radius)
        self.guard_arc_radius = float(guard_arc_radius)
        self.support_guard_radius = float(support_guard_radius)
        self.support_arc_half_angle = math.radians(float(support_arc_half_angle_deg))
        self.support_arc_rear_offset = math.radians(float(support_arc_rear_offset_deg))
        self.guard_arc_half_angle = math.radians(float(guard_arc_half_angle_deg))
        self.max_guard_arc_half_angle = math.radians(float(max_guard_arc_half_angle_deg))
        self.minimum_guard_spacing = float(minimum_guard_spacing)
        self.blocker_ratio = float(blocker_ratio)
        self.blocker_r_min = float(blocker_r_min)
        self.blocker_r_max = float(blocker_r_max)
        self.core_arrival_tolerance = float(core_arrival_tolerance)
        self.wing_arrival_tolerance = float(wing_arrival_tolerance)
        self.wing_ready_ratio = float(wing_ready_ratio)
        self.enemy_approach_speed = float(enemy_approach_speed)
        self.enemy_forming_speed = float(enemy_forming_speed)
        self.enemy_controlled_speed = float(enemy_controlled_speed)
        self.enemy_min_radius = float(enemy_min_radius)
        self.random_orbit_min_angle = math.radians(float(random_orbit_min_angle_deg))
        self.random_orbit_max_angle = math.radians(float(random_orbit_max_angle_deg))
        self.random_orbit_min_speed = float(random_orbit_min_speed)
        self.random_orbit_max_speed = float(random_orbit_max_speed)
        self.random_orbit_reverse_probability = float(random_orbit_reverse_probability)
        self.avoidance_mode = avoidance_mode
        self.avoid_distance = float(avoid_distance)
        self.forward_shift = float(forward_shift)
        self.own_max_speed = float(own_max_speed)
        self.own_gain = float(own_gain)
        self.cruise_speed = float(cruise_speed)
        self.safe_distance = float(safe_distance)
        self.repulsion_gain = float(repulsion_gain)
        self.own_target_avoid_radius = float(own_target_avoid_radius)
        self._own_target_route_margin = max(0.08, 0.03 * self.own_target_avoid_radius)
        self.core_trail_length = int(core_trail_length)
        self.dt = float(dt)
        self.world_x_min = float(WORLD_X_MIN)
        self.world_x_max = float(WORLD_X_MAX)
        self.world_y_min = float(WORLD_Y_MIN)
        self.world_y_max = float(WORLD_Y_MAX)
        self.initial_own_position = np.array([OWN_START_X, OWN_START_Y], dtype=float)
        if not (
            self.world_x_min + self.ring_radius < self.initial_own_position[0] < self.world_x_max - self.ring_radius
            and self.world_y_min + self.ring_radius < self.initial_own_position[1] < self.world_y_max - self.ring_radius
        ):
            raise ValueError("The initial escort position must leave room for the escort ring")
            raise ValueError("sensor_radius is too small for two separated enemy tracks")

        self.forward = np.array([1.0, 0.0], dtype=float)
        self.own_position = self.initial_own_position.copy()
        self.own_goal = self.own_position.copy()
        self.avoid_direction = np.zeros(2, dtype=float)
        self.frame = 0
        self.paused = False
        self.phase = "正常护航"
        self.last_message = "敌方目标尚未进入感知范围"

        self.platforms = self._create_mixed_ring()
        self.threats = self._create_threat_schedule()
        self.reserve_guard_indices: List[int] = []
        self._reserve_slot_offsets: Dict[int, np.ndarray] = {}
        self._reserve_slot_by_index: Dict[int, int] = {}
        self.support_guard_indices: List[int] = []
        self._support_slot_by_index: Dict[int, int] = {}
        self._last_detected_ids: Tuple[int, ...] = ()
        self._own_target_bypass_side: Dict[int, int] = {}

    @property
    def max_targets(self) -> int:
        return 1

    @property
    def threat_count(self) -> int:
        return len(self.threats)

    @property
    def spawned_threats(self) -> List[ThreatTask]:
        return [task for task in self.threats if task.state != "waiting"]

    @property
    def detected_threats(self) -> List[ThreatTask]:
        return [task for task in self.threats if task.state in DETECTED_STATES]

    @property
    def threat_active(self) -> bool:
        return bool(self.detected_threats)

    @property
    def forward_guard_indices(self) -> List[int]:
        result: List[int] = []
        for task in self.detected_threats:
            if task.core_guard_index is not None:
                result.append(task.core_guard_index)
            result.extend(task.wing_guard_indices)
        return result

    def get_threat(self, threat_id: int) -> ThreatTask:
        for task in self.threats:
            if task.threat_id == threat_id:
                return task
        raise KeyError(f"Unknown threat id {threat_id}")

    def _create_mixed_ring(self) -> List[Platform]:
        kinds: List[str] = []
        uav_left, usv_left = self.num_uav, self.num_usv
        prefer_uav = True
        while uav_left + usv_left:
            if (prefer_uav and uav_left > 0) or usv_left == 0:
                kinds.append("UAV")
                uav_left -= 1
            else:
                kinds.append("USV")
                usv_left -= 1
            prefer_uav = not prefer_uav
        rotation = float(self.rng.uniform(-0.10, 0.10))
        angles = np.linspace(0.0, 2.0 * math.pi, len(kinds), endpoint=False) + rotation
        result: List[Platform] = []
        uav_no = usv_no = 0
        for angle, kind in zip(angles, kinds):
            position = self.own_position + self.ring_radius * np.array(
                [math.cos(angle), math.sin(angle)], dtype=float
            )
            if kind == "UAV":
                uav_no += 1
                result.append(Platform(f"U{uav_no}", kind, position, 0.28, 0.42))
            else:
                usv_no += 1
                result.append(Platform(f"S{usv_no}", kind, position, 0.15, 0.32))
        return result

    def _create_threat_schedule(self) -> List[ThreatTask]:
        """Create exactly one target at a uniformly random bearing."""
        angle = float(self.rng.uniform(0.0, 2.0 * math.pi))
        radius = float(self.rng.uniform(self.sensor_radius + 6.0, self.sensor_radius + 12.0))
        position = self.own_position + radius * np.array(
            [math.cos(angle), math.sin(angle)], dtype=float
        )
        return [
            ThreatTask(
                threat_id=1,
                spawn_frame=0,
                spawn_angle=angle,
                spawn_radius=radius,
                position=position,
            )
        ]

    def _start_random_orbit_segment(self, task: ThreatTask) -> None:
        """Choose the next random angular segment and clockwise/counterclockwise direction."""
        previous_direction = task.orbit_direction
        if task.orbit_segment_count == 0:
            new_direction = int(self.rng.choice(np.array([-1, 1], dtype=int)))
        elif float(self.rng.random()) < self.random_orbit_reverse_probability:
            new_direction = -previous_direction
        else:
            new_direction = previous_direction
        if task.orbit_segment_count > 0 and new_direction != previous_direction:
            task.orbit_direction_changes += 1
        task.orbit_direction = new_direction
        task.orbit_segment_angle = float(
            self.rng.uniform(self.random_orbit_min_angle, self.random_orbit_max_angle)
        )
        task.orbit_segment_remaining = task.orbit_segment_angle
        task.orbit_angular_speed_current = float(
            self.rng.uniform(self.random_orbit_min_speed, self.random_orbit_max_speed)
        )
        task.orbit_segment_count += 1

    def reset(self) -> None:
        self.reset_count += 1
        self.rng = np.random.default_rng(self.seed + self.reset_count)
        self.own_position = self.initial_own_position.copy()
        self.own_goal = self.own_position.copy()
        self.avoid_direction = np.zeros(2, dtype=float)
        self.frame = 0
        self.paused = False
        self.phase = "正常护航"
        self.last_message = "场景已重置"
        self.platforms = self._create_mixed_ring()
        self.threats = self._create_threat_schedule()
        self.reserve_guard_indices = []
        self._reserve_slot_offsets = {}
        self._reserve_slot_by_index = {}
        self.support_guard_indices = []
        self._support_slot_by_index = {}
        self._last_detected_ids = ()
        self._own_target_bypass_side = {}

    def toggle_pause(self) -> bool:
        self.paused = not self.paused
        return self.paused

    def _spawn_due_threats(self) -> None:
        for task in self.threats:
            if task.state == "waiting" and self.frame >= task.spawn_frame:
                task.position = self.own_position + task.spawn_radius * np.array(
                    [math.cos(task.spawn_angle), math.sin(task.spawn_angle)], dtype=float
                )
                task.state = "approaching"
                task.current_speed_limit = self.enemy_approach_speed
                self.last_message = f"敌方目标 T{task.threat_id} 已从随机方向出现"

    def _move_point_toward(
        self, current: np.ndarray, desired: np.ndarray, speed_limit: float
    ) -> np.ndarray:
        delta = np.asarray(desired, dtype=float) - np.asarray(current, dtype=float)
        distance = float(np.linalg.norm(delta))
        max_step = max(0.0, float(speed_limit)) * self.dt
        if distance <= max_step + EPS:
            return np.asarray(desired, dtype=float).copy()
        return np.asarray(current, dtype=float) + delta * (max_step / (distance + EPS))

    def _move_enemy(self, task: ThreatTask) -> None:
        if task.state == "waiting":
            task.current_speed_limit = 0.0
            return
        relative = task.position - self.own_position
        distance = float(np.linalg.norm(relative))
        direction = normalize(relative, np.array([1.0, 0.0]))
        if task.state == "approaching":
            task.current_speed_limit = self.enemy_approach_speed
            desired = self.own_position
            task.position = self._move_point_toward(task.position, desired, task.current_speed_limit)
            task.position = self._clip_position_to_world(task.position, margin=0.2)
            return
        if task.state in {"detected", "forming"}:
            task.current_speed_limit = self.enemy_forming_speed
            # 每个敌方目标只接近到自己的最终受控轨道。目标到达自身轨道后立即停止径向逼近；在守卫队形尚未完成时，
            # 仅维持当前方位和指定半径，不再继续靠近高价值目标。
            target_radius = self._controlled_track_radius(task)
            desired = self.own_position + direction * target_radius
            task.position = self._move_point_toward(
                task.position, desired, task.current_speed_limit
            )
            task.position = self._clip_position_to_world(task.position, margin=0.2)
            return
        task.current_speed_limit = self.enemy_controlled_speed
        if task.state == "orbiting":
            if task.orbit_segment_remaining <= EPS:
                self._start_random_orbit_segment(task)
            angular_step = min(
                task.orbit_segment_remaining,
                task.orbit_angular_speed_current * self.dt,
            )
            task.controlled_angle += task.orbit_direction * angular_step
            task.orbit_segment_remaining = max(
                0.0, task.orbit_segment_remaining - angular_step
            )
            desired = self.own_position + task.controlled_radius * np.array(
                [math.cos(task.controlled_angle), math.sin(task.controlled_angle)], dtype=float
            )
            task.position = self._move_point_toward(
                task.position, desired, task.current_speed_limit
            )
            task.position = self._clip_position_to_world(task.position, margin=0.2)
            if task.orbit_segment_remaining <= EPS:
                self._start_random_orbit_segment(task)
            return

    def _detect_new_threats(self) -> bool:
        changed = False
        for task in self.threats:
            if task.state != "approaching":
                continue
            distance = float(np.linalg.norm(task.position - self.own_position))
            if distance <= self.sensor_radius + EPS:
                task.state = "detected"
                task.detected_frame = self.frame
                changed = True
                self.last_message = f"感知到敌方目标 T{task.threat_id}，触发守卫机制"
        return changed

    def _threat_geometry(self, task: ThreatTask) -> Tuple[np.ndarray, np.ndarray, float]:
        delta = task.position - self.own_position
        distance = float(np.linalg.norm(delta))
        return delta, normalize(delta, self.forward), distance

    def _refresh_task_blocker(self, task: ThreatTask) -> None:
        task.blocker_point, task.blocker_t = compute_blocker_point(
            self.own_position,
            task.position,
            ratio=self.blocker_ratio,
            r_min=self.blocker_r_min,
            r_max=self.blocker_r_max,
            fallback_direction=self.forward,
        )

    def _refresh_all_blockers(self) -> None:
        for task in self.detected_threats:
            self._refresh_task_blocker(task)

    @staticmethod
    def _minimum_cost_assignment(cost_matrix: np.ndarray) -> List[int]:
        """使用 O(n^3) 匈牙利算法完成矩形最小代价唯一分配。"""
        costs = np.asarray(cost_matrix, dtype=float)
        if costs.ndim != 2:
            raise ValueError("cost_matrix must be two-dimensional")
        rows, columns = costs.shape
        if rows == 0:
            return []
        if rows > columns:
            raise ValueError("there must be at least as many candidates as slots")
        if not np.all(np.isfinite(costs)):
            raise ValueError("cost_matrix must contain only finite values")

        # 经典势函数形式的匈牙利算法；数组使用 1-based 索引以保持公式清晰。
        u = np.zeros(rows + 1, dtype=float)
        v = np.zeros(columns + 1, dtype=float)
        matched_row = np.zeros(columns + 1, dtype=int)
        predecessor = np.zeros(columns + 1, dtype=int)

        for row in range(1, rows + 1):
            matched_row[0] = row
            min_value = np.full(columns + 1, math.inf, dtype=float)
            used = np.zeros(columns + 1, dtype=bool)
            column0 = 0
            while True:
                used[column0] = True
                active_row = matched_row[column0]
                delta = math.inf
                column1 = 0
                for column in range(1, columns + 1):
                    if used[column]:
                        continue
                    reduced = costs[active_row - 1, column - 1] - u[active_row] - v[column]
                    if reduced < min_value[column] - EPS:
                        min_value[column] = reduced
                        predecessor[column] = column0
                    if min_value[column] < delta - EPS:
                        delta = min_value[column]
                        column1 = column
                for column in range(columns + 1):
                    if used[column]:
                        u[matched_row[column]] += delta
                        v[column] -= delta
                    else:
                        min_value[column] -= delta
                column0 = column1
                if matched_row[column0] == 0:
                    break
            while True:
                column1 = predecessor[column0]
                matched_row[column0] = matched_row[column1]
                column0 = column1
                if column0 == 0:
                    break

        assignment = [-1] * rows
        for column in range(1, columns + 1):
            row = matched_row[column]
            if row != 0:
                assignment[row - 1] = column - 1
        if any(column < 0 for column in assignment):
            raise RuntimeError("minimum-cost assignment is incomplete")
        return assignment

    def guard_quota_per_detected_target(self) -> Dict[int, int]:
        tasks = self.detected_threats
        if not tasks:
            return {}
        available = len(self.platforms) - self.escort_reserve_count
        direct_quota = max(1, available // 2)
        return {tasks[0].threat_id: direct_quota}

    def _select_reserve_guards(self) -> List[int]:
        tasks = self.detected_threats
        if not tasks:
            return []
        # 保留对当前威胁平均到达时间最长的平台，尽量不占用最适合拦截的成员。
        scores = []
        for index, platform in enumerate(self.platforms):
            costs = [
                np.linalg.norm(platform.position - task.blocker_point)
                / (platform.max_speed + EPS)
                for task in tasks
            ]
            scores.append((min(costs), index))
        return [index for _, index in sorted(scores, reverse=True)[: self.escort_reserve_count]]

    def _guard_arc_geometry(self, task: ThreatTask) -> Tuple[float, float]:
        """Return the single forward guard arc used for the only threat."""
        wing_count = max(0, task.guard_quota - 1)
        if wing_count <= 1:
            return self.guard_arc_radius, 0.0
        half_angle = min(
            self.max_guard_arc_half_angle,
            max(self.guard_arc_half_angle, math.radians(8.0) * (wing_count - 1)),
        )
        angular_step = 2.0 * half_angle / max(wing_count - 1, 1)
        radius = self.guard_arc_radius
        if self.minimum_guard_spacing > EPS:
            radius = max(
                radius,
                self.minimum_guard_spacing
                / (2.0 * max(math.sin(angular_step / 2.0), 1e-6)),
            )
        return radius, half_angle

    def wing_goals(self, task_or_id: Union[ThreatTask, int]) -> List[np.ndarray]:
        task = task_or_id if isinstance(task_or_id, ThreatTask) else self.get_threat(task_or_id)
        wing_count = max(0, task.guard_quota - 1)
        if wing_count == 0:
            return []
        _, threat_dir, _ = self._threat_geometry(task)
        lateral = rotate90(threat_dir)
        radius, half_angle = self._guard_arc_geometry(task)
        angles: Iterable[float]
        if wing_count == 1:
            angles = [0.0]
        else:
            angles = np.linspace(-half_angle, half_angle, wing_count)
        return [
            self.own_position
            + radius * (math.cos(phi) * threat_dir + math.sin(phi) * lateral)
            for phi in angles
        ]

    def support_goals(
        self, task_or_id: Union[ThreatTask, int], count: Optional[int] = None
    ) -> List[np.ndarray]:
        """Return a rear support arc that rotates with one detected threat."""
        task = task_or_id if isinstance(task_or_id, ThreatTask) else self.get_threat(task_or_id)
        support_count = len(self.support_guard_indices) if count is None else int(count)
        if support_count <= 0:
            return []
        _, threat_dir, _ = self._threat_geometry(task)
        center_dir = np.array(
            [
                math.cos(self.support_arc_rear_offset) * threat_dir[0]
                - math.sin(self.support_arc_rear_offset) * threat_dir[1],
                math.sin(self.support_arc_rear_offset) * threat_dir[0]
                + math.cos(self.support_arc_rear_offset) * threat_dir[1],
            ],
            dtype=float,
        )
        lateral = rotate90(center_dir)
        if support_count == 1:
            angles: Iterable[float] = [0.0]
        else:
            angles = np.linspace(
                -self.support_arc_half_angle,
                self.support_arc_half_angle,
                support_count,
            )
        return [
            self.own_position
            + self.support_guard_radius
            * (math.cos(phi) * center_dir + math.sin(phi) * lateral)
            for phi in angles
        ]

    def _current_reserve_goal_offsets(self) -> List[np.ndarray]:
        if not self.reserve_guard_indices:
            return []
        threat_dir = self.composite_threat_direction()
        rear = -threat_dir
        lateral = rotate90(rear)
        if len(self.reserve_guard_indices) == 1:
            return [self.ring_radius * rear]
        return [
            self.ring_radius * normalize(rear + 0.55 * lateral),
            self.ring_radius * normalize(rear - 0.55 * lateral),
        ]

    def _current_reserve_goals(self) -> Dict[int, np.ndarray]:
        offsets = self._current_reserve_goal_offsets()
        result: Dict[int, np.ndarray] = {}
        for index, slot in self._reserve_slot_by_index.items():
            if slot < len(offsets):
                result[index] = self.own_position + offsets[slot]
        return result

    def _assign_reserve_slots(self) -> None:
        self._reserve_slot_offsets = {}
        self._reserve_slot_by_index = {}
        if not self.reserve_guard_indices:
            return
        offsets = self._current_reserve_goal_offsets()
        candidates = self.reserve_guard_indices
        cost = np.zeros((len(offsets), len(candidates)), dtype=float)
        for row, offset in enumerate(offsets):
            goal = self.own_position + offset
            for col, index in enumerate(candidates):
                cost[row, col] = np.linalg.norm(self.platforms[index].position - goal)
        assignment = self._minimum_cost_assignment(cost)
        for slot, column in enumerate(assignment):
            index = candidates[column]
            self._reserve_slot_by_index[index] = slot
            self._reserve_slot_offsets[index] = offsets[slot]

    def _begin_core_dispatch(self, task: ThreatTask) -> None:
        if task.core_guard_index is None:
            task.core_motion_state = "idle"
            return
        core = self.platforms[task.core_guard_index]
        task.core_dispatch_origin = core.position.copy()
        task.core_dispatch_initial_distance = float(np.linalg.norm(core.position - task.blocker_point))
        task.core_trajectory = [core.position.copy()]
        task.core_motion_state = "moving"

    def _replan_detected_guards(self) -> None:
        self._own_target_bypass_side = {}
        for platform in self.platforms:
            platform.role = "escort"
            platform.assigned_threat_id = None
        tasks = self.detected_threats
        for task in self.threats:
            if task not in tasks:
                task.guard_quota = 0
                task.core_guard_index = None
                task.wing_guard_indices = []
                task.wing_slot_by_index = {}
        if not tasks:
            self.reserve_guard_indices = []
            self._reserve_slot_offsets = {}
            self._reserve_slot_by_index = {}
            self.support_guard_indices = []
            self._support_slot_by_index = {}
            self._last_detected_ids = ()
            return

        self._refresh_all_blockers()
        quotas = self.guard_quota_per_detected_target()
        for task in tasks:
            task.guard_quota = quotas[task.threat_id]
            task.core_guard_index = None
            task.wing_guard_indices = []
            task.wing_slot_by_index = {}

        self.reserve_guard_indices = self._select_reserve_guards()
        reserve_set = set(self.reserve_guard_indices)
        candidates = [i for i in range(len(self.platforms)) if i not in reserve_set]

        core_cost = np.zeros((len(tasks), len(candidates)), dtype=float)
        for row, task in enumerate(tasks):
            for col, index in enumerate(candidates):
                platform = self.platforms[index]
                core_cost[row, col] = np.linalg.norm(platform.position - task.blocker_point) / (
                    platform.max_speed + EPS
                )
        core_columns = self._minimum_cost_assignment(core_cost)
        used = set(reserve_set)
        for task, column in zip(tasks, core_columns):
            index = candidates[column]
            task.core_guard_index = index
            used.add(index)

        slot_records: List[Tuple[ThreatTask, int, np.ndarray]] = []
        for task in tasks:
            for slot, goal in enumerate(self.wing_goals(task)):
                slot_records.append((task, slot, goal))
        wing_candidates = [i for i in range(len(self.platforms)) if i not in used]
        if slot_records:
            wing_cost = np.zeros((len(slot_records), len(wing_candidates)), dtype=float)
            for row, (_, _, goal) in enumerate(slot_records):
                for col, index in enumerate(wing_candidates):
                    wing_cost[row, col] = np.linalg.norm(self.platforms[index].position - goal)
            wing_columns = self._minimum_cost_assignment(wing_cost)
            for record, column in zip(slot_records, wing_columns):
                task, slot, _ = record
                index = wing_candidates[column]
                task.wing_guard_indices.append(index)
                task.wing_slot_by_index[index] = slot
                used.add(index)

        self.support_guard_indices = []
        self._support_slot_by_index = {}
        if len(tasks) == 1:
            task = tasks[0]
            support_candidates = [i for i in range(len(self.platforms)) if i not in used]
            support_goals = self.support_goals(task, count=len(support_candidates))
            if support_candidates:
                support_cost = np.zeros((len(support_goals), len(support_candidates)), dtype=float)
                for row, goal in enumerate(support_goals):
                    for col, index in enumerate(support_candidates):
                        support_cost[row, col] = np.linalg.norm(
                            self.platforms[index].position - goal
                        )
                support_columns = self._minimum_cost_assignment(support_cost)
                for slot, column in enumerate(support_columns):
                    index = support_candidates[column]
                    self.support_guard_indices.append(index)
                    self._support_slot_by_index[index] = slot
                    used.add(index)

        for index in self.reserve_guard_indices:
            reserve = self.platforms[index]
            reserve.role = "reserve"
            reserve.assigned_threat_id = tasks[0].threat_id
        for index in self.support_guard_indices:
            support = self.platforms[index]
            support.role = "support"
            support.assigned_threat_id = tasks[0].threat_id
        for task in tasks:
            assert task.core_guard_index is not None
            core = self.platforms[task.core_guard_index]
            core.role = "core"
            core.assigned_threat_id = task.threat_id
            for index in task.wing_guard_indices:
                wing = self.platforms[index]
                wing.role = "wing"
                wing.assigned_threat_id = task.threat_id
            self._begin_core_dispatch(task)
            if task.state == "detected":
                task.state = "forming"
        self._assign_reserve_slots()
        self._last_detected_ids = tuple(sorted(task.threat_id for task in tasks))

    def _synchronize_guard_plan(self) -> None:
        ids = tuple(sorted(task.threat_id for task in self.detected_threats))
        if ids != self._last_detected_ids:
            self._replan_detected_guards()

    def composite_threat_direction(self) -> np.ndarray:
        tasks = self.detected_threats
        if not tasks:
            return normalize(self.forward, np.array([1.0, 0.0]))
        combined = np.zeros(2, dtype=float)
        nearest_direction = normalize(self.forward)
        nearest_distance = math.inf
        for task in tasks:
            _, direction, distance = self._threat_geometry(task)
            combined += direction / max(distance, 1.0) ** 2
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_direction = direction
        return normalize(combined, nearest_direction)

    def _resolve_avoid_direction(self) -> np.ndarray:
        left = rotate90(normalize(self.forward, np.array([1.0, 0.0])))
        if self.avoidance_mode == "left":
            return left
        if self.avoidance_mode == "right":
            return -left
        lateral = float(np.dot(self.composite_threat_direction(), left))
        if lateral > 0.05:
            return -left
        if lateral < -0.05:
            return left
        return -left

    def _truncate_own_step_for_platforms(
        self, start: np.ndarray, end: np.ndarray
    ) -> np.ndarray:
        """连续缩短我方目标航步，避免移动中心主动穿入任一平台。"""
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        direction = end - start
        a = float(np.dot(direction, direction))
        if a <= EPS:
            return start.copy()
        allowed = 1.0
        radius = self.own_target_avoid_radius
        for platform in self.platforms:
            relative = start - platform.position
            # 若当前已贴近边界，只允许沿远离该平台的方向移动。
            start_distance = float(np.linalg.norm(relative))
            if start_distance < radius - 1e-9:
                allowed = 0.0
                continue
            closest_parameter = float(
                np.clip(-np.dot(relative, direction) / a, 0.0, 1.0)
            )
            closest = relative + closest_parameter * direction
            if float(np.linalg.norm(closest)) >= radius - 1e-10:
                continue
            b = 2.0 * float(np.dot(relative, direction))
            c = float(np.dot(relative, relative) - radius**2)
            discriminant = max(0.0, b * b - 4.0 * a * c)
            root = math.sqrt(discriminant)
            roots = sorted(
                value for value in (
                    (-b - root) / (2.0 * a),
                    (-b + root) / (2.0 * a),
                )
                if 0.0 <= value <= 1.0
            )
            if roots:
                allowed = min(allowed, max(0.0, roots[0] - 1e-8))
        return start + allowed * direction

    def _clip_position_to_world(self, position: np.ndarray, margin: float = 0.0) -> np.ndarray:
        lower = np.array([self.world_x_min + margin, self.world_y_min + margin], dtype=float)
        upper = np.array([self.world_x_max - margin, self.world_y_max - margin], dtype=float)
        if np.any(lower > upper):
            lower = np.minimum(lower, upper)
            upper = np.maximum(lower, upper)
        return np.clip(np.asarray(position, dtype=float), lower, upper)

    def _move_own_target(self) -> None:
        if not self.detected_threats:
            velocity = normalize(self.forward) * self.cruise_speed
            self.avoid_direction = np.zeros(2, dtype=float)
        else:
            self.avoid_direction = self._resolve_avoid_direction()
            desired_velocity = (
                normalize(self.forward) * self.forward_shift
                + self.avoid_direction * self.avoid_distance
            )
            velocity = self.own_gain * desired_velocity
        speed = float(np.linalg.norm(velocity))
        if speed > self.own_max_speed:
            velocity *= self.own_max_speed / (speed + EPS)
        proposed = self.own_position + velocity * self.dt
        self.own_position = self._truncate_own_step_for_platforms(
            self.own_position, proposed
        )
        self.own_position = self._clip_position_to_world(self.own_position, margin=0.8)
        self.own_goal = self.own_position + velocity

    def _normal_ring_goals(self) -> Dict[int, np.ndarray]:
        count = len(self.platforms)
        return {
            index: self.own_position
            + self.ring_radius * np.array([math.cos(phi), math.sin(phi)], dtype=float)
            for index, phi in enumerate(np.linspace(0.0, 2.0 * math.pi, count, endpoint=False))
        }

    def _desired_non_core_goals(self) -> Dict[int, np.ndarray]:
        if not self.detected_threats:
            return self._normal_ring_goals()

        # 未被选为核心/弧线/预留成员的平台继续执行正常环形护航，
        # 而不是停在原地。随后用守卫弧和贴身护航目标覆盖对应成员。
        result = self._normal_ring_goals()
        for core_index in self._core_guard_indices():
            result.pop(core_index, None)
        for task in self.detected_threats:
            goals = self.wing_goals(task)
            for index, slot in task.wing_slot_by_index.items():
                if slot < len(goals):
                    result[index] = goals[slot]

        if len(self.detected_threats) == 1 and self.support_guard_indices:
            task = self.detected_threats[0]
            support_goals = self.support_goals(task)
            for index, slot in self._support_slot_by_index.items():
                if slot < len(support_goals):
                    result[index] = support_goals[slot]

        result.update(self._current_reserve_goals())
        return result

    def _core_guard_indices(self) -> set[int]:
        return {
            task.core_guard_index
            for task in self.detected_threats
            if task.core_guard_index is not None
        }

    def _repulsion_velocity(self, index: int) -> np.ndarray:
        if index in self._core_guard_indices():
            return np.zeros(2, dtype=float)
        current = self.platforms[index].position
        repulsion = np.zeros(2, dtype=float)
        for other_index, other in enumerate(self.platforms):
            if other_index == index:
                continue
            diff = current - other.position
            distance = float(np.linalg.norm(diff))
            if EPS < distance < self.safe_distance:
                repulsion += (
                    self.repulsion_gain
                    * (1.0 / distance - 1.0 / self.safe_distance)
                    * diff
                    / distance
                )
        return repulsion

    @property
    def own_target_route_radius(self) -> float:
        return self.own_target_avoid_radius + self._own_target_route_margin

    def _segment_intersects_own_target_circle(
        self, start: np.ndarray, end: np.ndarray, radius: Optional[float] = None
    ) -> bool:
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        protected_radius = self.own_target_avoid_radius if radius is None else float(radius)
        segment = end - start
        denominator = float(np.dot(segment, segment))
        if denominator <= EPS:
            return float(np.linalg.norm(start - self.own_position)) < protected_radius - EPS
        parameter = float(
            np.clip(np.dot(self.own_position - start, segment) / denominator, 0.0, 1.0)
        )
        closest = start + parameter * segment
        return float(np.linalg.norm(closest - self.own_position)) < protected_radius - EPS

    @staticmethod
    def _directed_arc_delta(start_angle: float, end_angle: float, side: int) -> float:
        if side > 0:
            return float((end_angle - start_angle) % (2.0 * math.pi))
        return float((start_angle - end_angle) % (2.0 * math.pi))

    def _bypass_path_length(self, current: np.ndarray, goal: np.ndarray, side: int) -> float:
        center = self.own_position
        radius = self.own_target_route_radius
        p = np.asarray(current, dtype=float) - center
        g = np.asarray(goal, dtype=float) - center
        pd = max(float(np.linalg.norm(p)), radius)
        gd = max(float(np.linalg.norm(g)), radius)
        pa = math.atan2(p[1], p[0])
        ga = math.atan2(g[1], g[0])
        p_alpha = math.acos(float(np.clip(radius / pd, 0.0, 1.0)))
        g_alpha = math.acos(float(np.clip(radius / gd, 0.0, 1.0)))
        start_tangent = pa + side * p_alpha
        end_tangent = ga - side * g_alpha
        arc = self._directed_arc_delta(start_tangent, end_tangent, side)
        return (
            math.sqrt(max(0.0, pd * pd - radius * radius))
            + radius * arc
            + math.sqrt(max(0.0, gd * gd - radius * radius))
        )

    def _choose_bypass_side(self, index: int, current: np.ndarray, goal: np.ndarray) -> int:
        existing = self._own_target_bypass_side.get(index)
        if existing in {-1, 1}:
            return existing
        ccw = self._bypass_path_length(current, goal, 1)
        cw = self._bypass_path_length(current, goal, -1)
        side = (1 if index % 2 == 0 else -1) if abs(ccw - cw) <= 1e-9 else (1 if ccw < cw else -1)
        self._own_target_bypass_side[index] = side
        return side

    def _truncate_step_before_safety_circle(self, start: np.ndarray, end: np.ndarray) -> np.ndarray:
        if not self._segment_intersects_own_target_circle(start, end):
            return np.asarray(end, dtype=float)
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        direction = end - start
        a = float(np.dot(direction, direction))
        if a <= EPS:
            return start.copy()
        relative = start - self.own_position
        b = 2.0 * float(np.dot(relative, direction))
        c = float(np.dot(relative, relative) - self.own_target_avoid_radius**2)
        disc = max(0.0, b * b - 4.0 * a * c)
        root = math.sqrt(disc)
        roots = [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
        valid = [value for value in roots if 0.0 <= value <= 1.0]
        if not valid:
            return start.copy()
        return start + max(0.0, min(valid) - 1e-8) * direction

    def _safe_route_waypoint(
        self, index: int, current: np.ndarray, goal: np.ndarray, max_step: float
    ) -> np.ndarray:
        """用稳定的切向引导连续绕过随船移动的安全圆。"""
        center = self.own_position
        route_radius = self.own_target_route_radius
        current = np.asarray(current, dtype=float)
        goal = np.asarray(goal, dtype=float)
        current_relative = current - center
        goal_relative = goal - center
        current_distance = float(np.linalg.norm(current_relative))
        goal_distance = float(np.linalg.norm(goal_relative))

        if goal_distance < route_radius:
            goal_relative = normalize(goal_relative, current_relative) * route_radius
            goal = center + goal_relative

        # 直线路径已经安全时立即恢复直接追踪，并清除旧绕行方向。
        if (
            current_distance >= route_radius - 1e-8
            and not self._segment_intersects_own_target_circle(
                current, goal, route_radius
            )
        ):
            self._own_target_bypass_side.pop(index, None)
            return goal

        side = self._choose_bypass_side(index, current, goal)
        radial = normalize(current_relative, -goal_relative)
        tangent = side * rotate90(radial)
        radial_error = current_distance - route_radius

        # 距离安全圆较远时一边靠近一边绕行；贴近圆周后以切向运动为主，
        # 并用径向反馈抵消我方目标移动造成的内外漂移。
        if radial_error > max(0.25, 1.5 * max_step):
            steering = tangent - 0.85 * radial
        else:
            correction = float(
                np.clip(-2.5 * radial_error / max(max_step, 1e-6), -1.4, 1.8)
            )
            steering = tangent + correction * radial
        return current + normalize(steering, tangent) * max_step

    def _move_platform_safely(
        self, index: int, goal: np.ndarray, *, include_repulsion: bool = False
    ) -> None:
        platform = self.platforms[index]
        current = platform.position.copy()
        max_step = platform.max_speed * self.dt
        waypoint = self._safe_route_waypoint(index, current, np.asarray(goal, dtype=float), max_step)
        velocity = platform.gain * (waypoint - current)
        if include_repulsion and index not in self._own_target_bypass_side:
            velocity += self._repulsion_velocity(index)
        speed = float(np.linalg.norm(velocity))
        if speed > platform.max_speed:
            velocity *= platform.max_speed / (speed + EPS)
        proposed = current + velocity * self.dt
        proposed = self._truncate_step_before_safety_circle(current, proposed)
        platform.position = self._clip_position_to_world(proposed, margin=0.2)
        platform.goal = np.asarray(goal, dtype=float).copy()

    def _move_platforms(self) -> None:
        cores = self._core_guard_indices()
        goals = self._desired_non_core_goals()
        for index, platform in enumerate(self.platforms):
            if index in cores:
                continue
            self._move_platform_safely(index, goals.get(index, platform.position), include_repulsion=True)
        for task in self.detected_threats:
            if task.core_guard_index is None:
                continue
            self._move_platform_safely(task.core_guard_index, task.blocker_point)
            core = self.platforms[task.core_guard_index]
            core.role = "core"
            core.assigned_threat_id = task.threat_id
            task.core_trajectory.append(core.position.copy())
            if len(task.core_trajectory) > self.core_trail_length:
                task.core_trajectory = task.core_trajectory[-self.core_trail_length :]
            task.core_motion_state = "holding" if self.core_guard_arrived(task.threat_id) else "moving"

    def core_remaining_distance(self, threat_id: int) -> float:
        task = self.get_threat(threat_id)
        if task.core_guard_index is None:
            return math.inf
        return float(np.linalg.norm(self.platforms[task.core_guard_index].position - task.blocker_point))

    def core_guard_arrived(self, threat_id: int) -> bool:
        task = self.get_threat(threat_id)
        if task.core_guard_index is None:
            return False
        return self.core_remaining_distance(threat_id) <= self.core_arrival_tolerance

    def wing_arrival_ratio(self, threat_id: int) -> float:
        task = self.get_threat(threat_id)
        goals = self.wing_goals(task)
        if not task.wing_guard_indices:
            return 1.0
        arrived = 0
        for index in task.wing_guard_indices:
            slot = task.wing_slot_by_index[index]
            if slot < len(goals):
                error = float(np.linalg.norm(self.platforms[index].position - goals[slot]))
                if error <= self.wing_arrival_tolerance:
                    arrived += 1
        return arrived / len(task.wing_guard_indices)

    def formation_ready(self, threat_id: int) -> bool:
        task = self.get_threat(threat_id)
        return bool(
            task.state in {"detected", "forming", "orbiting"}
            and self.core_guard_arrived(threat_id)
            and self.wing_arrival_ratio(threat_id) + EPS >= self.wing_ready_ratio
        )

    def _controlled_track_radius(self, task: ThreatTask) -> float:
        """Return the single controlled orbit radius."""
        return float(
            max(
                self.enemy_min_radius,
                min(self.enemy_min_radius + 1.4, self.sensor_radius - 0.6),
            )
        )

    def controlled_track_error(self, task_or_id: Union[ThreatTask, int]) -> float:
        """Return the radial error from the threat's own controlled track."""
        task = task_or_id if isinstance(task_or_id, ThreatTask) else self.get_threat(task_or_id)
        distance = float(np.linalg.norm(task.position - self.own_position))
        return abs(distance - self._controlled_track_radius(task))

    def target_on_controlled_track(
        self, task_or_id: Union[ThreatTask, int], tolerance: Optional[float] = None
    ) -> bool:
        """Return whether a target has reached its assigned radial track."""
        tol = (
            max(
                1e-8,
                self.own_max_speed * self.dt
                + self.enemy_forming_speed * self.dt * 0.05,
            )
            if tolerance is None
            else float(tolerance)
        )
        return self.controlled_track_error(task_or_id) <= tol + EPS

    def _enter_controlled_motion(self, task: ThreatTask) -> None:
        relative = task.position - self.own_position
        task.controlled_radius = self._controlled_track_radius(task)
        task.controlled_angle = math.atan2(relative[1], relative[0])
        task.state = "orbiting"
        task.orbit_segment_count = 0
        task.orbit_direction_changes = 0
        self._start_random_orbit_segment(task)
        direction_label = "逆时针" if task.orbit_direction > 0 else "顺时针"
        self.last_message = (
            f"T1 守卫队形形成，开始随机分段环绕；当前方向：{direction_label}"
        )

    def _update_control_transitions(self) -> None:
        for task in self.detected_threats:
            if (
                task.state in {"detected", "forming"}
                and self.formation_ready(task.threat_id)
                and self.target_on_controlled_track(task)
            ):
                self._enter_controlled_motion(task)

    def threat_bearing_label(self, task: ThreatTask) -> str:
        _, direction, _ = self._threat_geometry(task)
        left = rotate90(normalize(self.forward))
        angle = math.degrees(math.atan2(float(np.dot(direction, left)), float(np.dot(direction, self.forward))))
        if angle < 0.0:
            angle += 360.0
        labels = ("正前方", "左前方", "正左方", "左后方", "正后方", "右后方", "正右方", "右前方")
        sector = int(((angle + 22.5) % 360.0) // 45.0)
        return f"{labels[sector]}（{angle:.1f}°）"

    def step(self) -> None:
        if self.paused:
            return
        self.frame += 1
        self._spawn_due_threats()

        # 先让敌方目标按当前状态运动，再执行感知判定。
        for task in self.threats:
            self._move_enemy(task)
        detection_changed = self._detect_new_threats()
        if detection_changed:
            self._replan_detected_guards()
        else:
            self._synchronize_guard_plan()

        self._move_own_target()
        self._refresh_all_blockers()
        self._move_platforms()
        self._update_control_transitions()

        detected = len(self.detected_threats)
        approaching = sum(task.state == "approaching" for task in self.threats)
        controlled = sum(task.state == "orbiting" for task in self.threats)
        if detected == 0:
            self.phase = f"正常护航：{approaching} 个远距离目标正在接近"
        elif controlled == detected:
            self.phase = f"持续动态盯防：{controlled} 个目标处于受控机动"
        else:
            self.phase = f"守卫编队形成中：已感知 {detected} 个目标"

    def status(self) -> Dict[str, object]:
        records = []
        for task in self.threats:
            distance = float(np.linalg.norm(task.position - self.own_position))
            records.append(
                {
                    "threat_id": task.threat_id,
                    "state": task.state,
                    "state_label": STATE_LABELS[task.state],
                    "spawn_frame": task.spawn_frame,
                    "position": task.position.copy(),
                    "distance": distance,
                    "detected": task.state in DETECTED_STATES,
                    "motion": "随机角度分段环绕",
                    "orbit_direction": ("逆时针" if task.orbit_direction > 0 else "顺时针"),
                    "orbit_segment_angle_deg": math.degrees(task.orbit_segment_angle),
                    "orbit_segment_remaining_deg": math.degrees(task.orbit_segment_remaining),
                    "orbit_segment_count": task.orbit_segment_count,
                    "orbit_direction_changes": task.orbit_direction_changes,
                    "bearing": self.threat_bearing_label(task) if task.state != "waiting" else "未出现",
                    "guard_quota": task.guard_quota,
                    "guard_track_radius": (
                        self._guard_arc_geometry(task)[0]
                        if task.state in DETECTED_STATES else 0.0
                    ),
                    "core_guard": (
                        self.platforms[task.core_guard_index].identifier
                        if task.core_guard_index is not None else None
                    ),
                    "wing_ready_ratio": self.wing_arrival_ratio(task.threat_id) if task.state in DETECTED_STATES else 0.0,
                    "core_ready": self.core_guard_arrived(task.threat_id) if task.state in DETECTED_STATES else False,
                }
            )
        return {
            "frame": self.frame,
            "phase": self.phase,
            "message": self.last_message,
            "paused": self.paused,
            "sensor_radius": self.sensor_radius,
            "enemy_count": self.enemy_count,
            "spawn_mode": self.spawn_mode,
            "detected_count": len(self.detected_threats),
            "reserve_count": len(self.reserve_guard_indices),
            "support_count": len(self.support_guard_indices),
            "normal_escort_count": max(
                0,
                len(self.platforms)
                - len(self.forward_guard_indices)
                - len(self.support_guard_indices)
                - len(self.reserve_guard_indices),
            ),
            "uav_count": self.num_uav,
            "usv_count": self.num_usv,
            "threats": records,
        }


class EscortGuardVisualizer:
    TARGET_COLORS = ("#e53935",)

    def __init__(self, simulator: EscortGuardSimulator) -> None:
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

        self.plt = plt
        self.Circle = Circle
        self.Polygon = Polygon
        self.FancyArrowPatch = FancyArrowPatch
        self.FancyBboxPatch = FancyBboxPatch
        self.Rectangle = Rectangle
        self.sim = simulator
        self.animation = None
        self.min_view_half_width = 19.0
        self.min_view_half_height = 12.0
        self.view_half_width = self.min_view_half_width
        self.view_half_height = self.min_view_half_height
        self.view_padding = 2.5
        self.view_shrink_smoothing = 0.12
        self.view_aspect_ratio = self.min_view_half_width / self.min_view_half_height

        preferred_fonts = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Arial Unicode MS", "DejaVu Sans"]
        noto_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")
        if noto_path.exists():
            try:
                font_manager.fontManager.addfont(str(noto_path))
                preferred_fonts.insert(0, font_manager.FontProperties(fname=str(noto_path)).get_name())
            except (OSError, RuntimeError, ValueError):
                pass
        plt.rcParams["font.sans-serif"] = preferred_fonts
        plt.rcParams["axes.unicode_minus"] = False

        self.fig, (self.local_ax, self.global_ax) = plt.subplots(1, 2, figsize=(18, 8.5))
        self.ax = self.local_ax  # backward-compatible alias for the tracked local view
        self.fig.subplots_adjust(wspace=0.08, left=0.05, right=0.98, top=0.92, bottom=0.08)

        self.local_view = self._create_view_bundle(
            self.local_ax, "局部视图（以我方守卫目标为中心）", local=True
        )
        self.global_view = self._create_view_bundle(
            self.global_ax, "全局视图（固定长方形画布）", local=False
        )
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self._refresh_artists()

    def _create_view_bundle(self, ax, title: str, *, local: bool) -> Dict[str, object]:
        ax.set_aspect("equal", adjustable="box")
        ax.set_facecolor("#dff4f7")
        ax.grid(alpha=0.25, linestyle="--")
        ax.set_xlabel("X / 海里（示意）")
        ax.set_ylabel("Y / 海里（示意）")
        ax.set_title(title, fontsize=13, weight="bold")

        boundary = self.Rectangle(
            (self.sim.world_x_min, self.sim.world_y_min),
            self.sim.world_x_max - self.sim.world_x_min,
            self.sim.world_y_max - self.sim.world_y_min,
            fill=False, linewidth=2.0, linestyle="-", edgecolor="#455a64", alpha=0.85, zorder=0
        )
        ax.add_patch(boundary)

        uav_size = LOCAL_UAV_SIZE if local else GLOBAL_UAV_SIZE
        usv_size = LOCAL_USV_SIZE if local else GLOBAL_USV_SIZE
        enemy_scale = 1.0 if local else GLOBAL_ENEMY_SCALE
        own_scale = 1.0 if local else 0.42
        label_fontsize = 7.8 if local else 4.8

        sensor_circle = self.Circle(
            tuple(self.sim.own_position), self.sim.sensor_radius,
            fill=False, linestyle="--", linewidth=1.8, edgecolor="#1976d2", alpha=0.65,
            label="圆形感知范围", zorder=1,
        )
        ax.add_patch(sensor_circle)
        uav_scatter = ax.scatter([], [], marker="o", s=uav_size, zorder=5, label="UAV")
        usv_scatter = ax.scatter([], [], marker="s", s=usv_size, zorder=5, label="USV")
        own_patch = self.FancyBboxPatch(
            (-1.6 * own_scale, -0.55 * own_scale), 3.2 * own_scale, 1.1 * own_scale,
            boxstyle="round,pad=0.18,rounding_size=0.18",
            facecolor="#4472c4", edgecolor="#203864", linewidth=1.8, zorder=6,
        )
        ax.add_patch(own_patch)
        own_text = ax.text(0.0, 0.0, "我方高价值护航目标", ha="center", va="center", color="white", weight="bold", fontsize=9.5 if local else 5.0, zorder=7)

        enemy_patches, enemy_texts, threat_lines, guard_arc_lines, blocker_crosses = [], [], [], [], []
        support_arc_line, = ax.plot(
            [], [], "-.", color="#1565c0", linewidth=1.8, alpha=0.78,
            zorder=2, label="动态支援弧",
        )
        for slot in range(self.sim.max_targets):
            color = self.TARGET_COLORS[slot]
            patch = self.Polygon(np.zeros((3, 2)), closed=True, facecolor=color, edgecolor="#5d0000", linewidth=1.5 if local else 0.8, zorder=6)
            ax.add_patch(patch)
            text = ax.text(0.0, 0.0, "", ha="center", va="center", color="white", weight="bold", fontsize=8 if local else 4.5, zorder=7)
            line, = ax.plot([], [], "--", color=color, linewidth=1.5, alpha=0.8, zorder=2, label="敌我连线" if slot == 0 else None)
            arc, = ax.plot([], [], color=color, linewidth=2.0, alpha=0.75, zorder=2, label="防护弧" if slot == 0 else None)
            cross, = ax.plot([], [], marker="x", markersize=10, markeredgewidth=2.2, color=color, zorder=8, label="阻断点" if slot == 0 else None)
            enemy_patches.append(patch)
            enemy_texts.append(text)
            threat_lines.append(line)
            guard_arc_lines.append(arc)
            blocker_crosses.append(cross)

        heading_arrow = self.FancyArrowPatch((0, 0), (2.5, 0), arrowstyle="-|>", mutation_scale=18, linewidth=2.0, color="#1f4e79", zorder=4)
        ax.add_patch(heading_arrow)
        avoid_arrow = self.FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=16, linewidth=2.0, linestyle="--", color="#00695c", zorder=4)
        ax.add_patch(avoid_arrow)
        platform_labels = [ax.text(0, 0, "", ha="center", va="bottom", fontsize=label_fontsize, zorder=9) for _ in self.sim.platforms]
        status_text = ax.text(
            0.985, 0.985, "", transform=ax.transAxes, ha="right", va="top", fontsize=8.8,
            bbox=dict(facecolor="white", alpha=0.9, edgecolor="#6c757d", boxstyle="round,pad=0.45"), zorder=12,
        )
        help_text = ax.text(
            0.015, 0.015,
            "Space：暂停/继续   N：按当前参数重置   Esc：关闭\n局部图跟随我方目标；全局图显示固定长方形画布",
            transform=ax.transAxes, ha="left", va="bottom", fontsize=8.5,
            bbox=dict(facecolor="white", alpha=0.84, edgecolor="#90a4ae"), zorder=12,
        )
        ax.legend(loc="lower right", framealpha=0.9, fontsize=8)
        return {
            "ax": ax, "boundary": boundary, "sensor_circle": sensor_circle,
            "local": local, "enemy_scale": enemy_scale, "own_scale": own_scale,
            "uav_scatter": uav_scatter, "usv_scatter": usv_scatter,
            "own_patch": own_patch, "own_text": own_text,
            "enemy_patches": enemy_patches, "enemy_texts": enemy_texts,
            "threat_lines": threat_lines, "guard_arc_lines": guard_arc_lines,
            "blocker_crosses": blocker_crosses, "support_arc_line": support_arc_line,
            "heading_arrow": heading_arrow, "avoid_arrow": avoid_arrow,
            "platform_labels": platform_labels, "status_text": status_text, "help_text": help_text,
        }

    def _scene_points_for_view(self) -> np.ndarray:
        points: List[np.ndarray] = [self.sim.own_position.copy()]
        points.extend(platform.position.copy() for platform in self.sim.platforms)
        for task in self.sim.spawned_threats:
            points.append(task.position.copy())
            if task.state in DETECTED_STATES:
                points.append(task.blocker_point.copy())
        return np.asarray(points, dtype=float)

    def _required_view_half_extents(self) -> Tuple[float, float]:
        points = self._scene_points_for_view()
        relative = points - self.sim.own_position
        max_x = float(np.max(np.abs(relative[:, 0]))) if len(relative) else 0.0
        max_y = float(np.max(np.abs(relative[:, 1]))) if len(relative) else 0.0
        width = max(self.min_view_half_width, max_x + self.view_padding)
        height = max(self.min_view_half_height, max_y + self.view_padding)
        if width / height < self.view_aspect_ratio:
            width = height * self.view_aspect_ratio
        else:
            height = width / self.view_aspect_ratio
        return width, height

    def _center_local_view_on_own_target(self) -> None:
        required_width, required_height = self._required_view_half_extents()
        if required_width >= self.view_half_width:
            self.view_half_width = required_width
        else:
            self.view_half_width += self.view_shrink_smoothing * (required_width - self.view_half_width)
            self.view_half_width = max(required_width, self.view_half_width)
        self.view_half_height = self.view_half_width / self.view_aspect_ratio
        if self.view_half_height < required_height:
            self.view_half_height = required_height
            self.view_half_width = required_height * self.view_aspect_ratio
        x, y = self.sim.own_position
        self.local_ax.set_xlim(x - self.view_half_width, x + self.view_half_width)
        self.local_ax.set_ylim(y - self.view_half_height, y + self.view_half_height)

    def _set_global_view(self) -> None:
        self.global_ax.set_xlim(self.sim.world_x_min, self.sim.world_x_max)
        self.global_ax.set_ylim(self.sim.world_y_min, self.sim.world_y_max)

    @staticmethod
    def _role_style(platform: Platform) -> Tuple[str, str, float]:
        if platform.role == "core":
            return "#ffbf00", "#b00020", 3.0
        if platform.role == "wing":
            return "#ffd966", "#8a6d00", 1.8
        if platform.role == "support":
            return "#90caf9", "#1565c0", 1.8
        if platform.role == "reserve":
            return "#a5d6a7", "#2e7d32", 1.8
        return "#86c56f", "#275d38", 1.4

    def _update_platform_scatter(self, kind: str, artist, *, local: bool) -> None:
        entries = [p for p in self.sim.platforms if p.kind == kind]
        if not entries:
            artist.set_offsets(np.empty((0, 2)))
            return
        artist.set_offsets(np.array([p.position for p in entries]))
        styles = [self._role_style(p) for p in entries]
        artist.set_facecolors([item[0] for item in styles])
        artist.set_edgecolors([item[1] for item in styles])
        line_scale = 1.0 if local else 0.55
        artist.set_linewidths([item[2] * line_scale for item in styles])

    def _arc_polyline(self, task: ThreatTask) -> np.ndarray:
        if task.state not in DETECTED_STATES or task.guard_quota <= 1:
            return np.empty((0, 2))
        _, direction, _ = self.sim._threat_geometry(task)
        lateral = rotate90(direction)
        radius, half_angle = self.sim._guard_arc_geometry(task)
        phis = np.linspace(-half_angle, half_angle, 90)
        return np.array([
            self.sim.own_position + radius * (math.cos(phi) * direction + math.sin(phi) * lateral)
            for phi in phis
        ])

    def _support_arc_polyline(self) -> np.ndarray:
        if len(self.sim.detected_threats) != 1 or not self.sim.support_guard_indices:
            return np.empty((0, 2))
        task = self.sim.detected_threats[0]
        _, threat_dir, _ = self.sim._threat_geometry(task)
        center_dir = np.array(
            [
                math.cos(self.sim.support_arc_rear_offset) * threat_dir[0]
                - math.sin(self.sim.support_arc_rear_offset) * threat_dir[1],
                math.sin(self.sim.support_arc_rear_offset) * threat_dir[0]
                + math.cos(self.sim.support_arc_rear_offset) * threat_dir[1],
            ],
            dtype=float,
        )
        lateral = rotate90(center_dir)
        phis = np.linspace(
            -self.sim.support_arc_half_angle,
            self.sim.support_arc_half_angle,
            90,
        )
        return np.array([
            self.sim.own_position
            + self.sim.support_guard_radius
            * (math.cos(phi) * center_dir + math.sin(phi) * lateral)
            for phi in phis
        ])

    def _hide_target_slot(self, view: Dict[str, object], slot: int) -> None:
        view["enemy_patches"][slot].set_visible(False)
        view["enemy_texts"][slot].set_visible(False)
        view["threat_lines"][slot].set_data([], [])
        view["guard_arc_lines"][slot].set_data([], [])
        view["blocker_crosses"][slot].set_data([], [])

    def _render_view(self, view: Dict[str, object], *, local: bool) -> List[object]:
        ax = view["ax"]
        self._update_platform_scatter("UAV", view["uav_scatter"], local=local)
        self._update_platform_scatter("USV", view["usv_scatter"], local=local)
        for text, platform in zip(view["platform_labels"], self.sim.platforms):
            label_offset = 0.34 if local else 0.16
            text.set_position(platform.position + np.array([0.0, label_offset]))
            suffix = f"/T{platform.assigned_threat_id}" if platform.assigned_threat_id else ""
            text.set_text(platform.identifier + suffix)
            text.set_weight("bold" if platform.role == "core" else "normal")
            text.set_color("#8b0000" if platform.role == "core" else "#1f2933")

        own_x, own_y = self.sim.own_position
        own_scale = float(view["own_scale"])
        view["own_patch"].set_x(own_x - 1.6 * own_scale)
        view["own_patch"].set_y(own_y - 0.55 * own_scale)
        view["own_text"].set_position((own_x, own_y))
        view["sensor_circle"].center = (own_x, own_y)
        view["sensor_circle"].set_radius(self.sim.sensor_radius)

        for slot in range(self.sim.max_targets):
            if slot >= len(self.sim.threats) or self.sim.threats[slot].state == "waiting":
                self._hide_target_slot(view, slot)
                continue
            task = self.sim.threats[slot]
            x, y = task.position
            enemy_scale = float(view["enemy_scale"])
            view["enemy_patches"][slot].set_xy(np.array([
                [x, y + 0.60 * enemy_scale],
                [x - 0.52 * enemy_scale, y - 0.40 * enemy_scale],
                [x + 0.52 * enemy_scale, y - 0.40 * enemy_scale],
            ]))
            view["enemy_patches"][slot].set_visible(True)
            view["enemy_texts"][slot].set_position((x, y - 0.05))
            view["enemy_texts"][slot].set_text(f"T{task.threat_id}\n{STATE_LABELS[task.state]}")
            view["enemy_texts"][slot].set_visible(True)
            if task.state in DETECTED_STATES:
                view["threat_lines"][slot].set_data([own_x, x], [own_y, y])
                view["blocker_crosses"][slot].set_data([task.blocker_point[0]], [task.blocker_point[1]])
                arc = self._arc_polyline(task)
                if len(arc):
                    view["guard_arc_lines"][slot].set_data(arc[:, 0], arc[:, 1])
                else:
                    view["guard_arc_lines"][slot].set_data([], [])
            else:
                view["threat_lines"][slot].set_data([], [])
                view["blocker_crosses"][slot].set_data([], [])
                view["guard_arc_lines"][slot].set_data([], [])

        support_arc = self._support_arc_polyline()
        if len(support_arc):
            view["support_arc_line"].set_data(support_arc[:, 0], support_arc[:, 1])
        else:
            view["support_arc_line"].set_data([], [])

        heading_start = self.sim.own_position + np.array([0.0, -1.2])
        view["heading_arrow"].set_positions(heading_start, heading_start + normalize(self.sim.forward) * 2.2)
        if self.sim.detected_threats:
            avoid_start = self.sim.own_position + np.array([0.0, 1.2])
            view["avoid_arrow"].set_positions(avoid_start, avoid_start + self.sim.avoid_direction * 2.2)
            view["avoid_arrow"].set_visible(True)
        else:
            view["avoid_arrow"].set_visible(False)

        status = self.sim.status()
        reserve_mode = "保留2个守卫" if self.sim.escort_reserve_count == 2 else "不保留贴身守卫"
        lines = [
            f"帧：{status['frame']}｜UAV/USV：{status['uav_count']}/{status['usv_count']}",
            f"敌方数量：1｜出现：随机方向",
            f"感知半径：{status['sensor_radius']:.1f}｜已感知：{status['detected_count']}",
            f"守卫模式：{reserve_mode}｜直接守卫：{sum(item['guard_quota'] for item in status['threats'])}",
            f"动态支援：{status['support_count']}｜普通护航：{status['normal_escort_count']}",
            f"阶段：{status['phase']}",
        ]
        for item in status["threats"]:
            if item["state"] == "waiting":
                lines.append(f"T{item['threat_id']} 等待至第 {item['spawn_frame']} 帧")
            else:
                ready = f"核心{'√' if item['core_ready'] else '×'}/弧线{item['wing_ready_ratio']*100:.0f}%" if item["detected"] else "未触发守卫"
                if item["state"] == "orbiting":
                    motion = (
                        f"{item['orbit_direction']}｜本段{item['orbit_segment_angle_deg']:.0f}°"
                        f"｜余{item['orbit_segment_remaining_deg']:.0f}°"
                    )
                else:
                    motion = item["motion"]
                lines.append(
                    f"T{item['threat_id']} {item['state_label']}｜距{item['distance']:.2f}｜{motion}｜守卫{item['guard_quota']}｜{ready}"
                )
        lines.append("已暂停" if status["paused"] else "运行中")
        if local:
            view["status_text"].set_text("\n".join(lines))
            ax.set_title("局部视图（以我方守卫目标为中心）", fontsize=13, weight="bold")
        else:
            view["status_text"].set_text(
                "固定全局矩形画布\n"
                f"X范围：[{self.sim.world_x_min:.0f}, {self.sim.world_x_max:.0f}]\n"
                f"Y范围：[{self.sim.world_y_min:.0f}, {self.sim.world_y_max:.0f}]\n"
                + reserve_mode + "\n单目标随机分段环绕"
            )
            ax.set_title("全局视图（固定长方形画布）", fontsize=13, weight="bold")

        artists: List[object] = [
            view["boundary"], view["sensor_circle"], view["uav_scatter"], view["usv_scatter"],
            view["own_patch"], view["own_text"], view["heading_arrow"], view["avoid_arrow"],
            view["status_text"], view["help_text"], view["support_arc_line"],
        ]
        artists.extend(view["enemy_patches"])
        artists.extend(view["enemy_texts"])
        artists.extend(view["threat_lines"])
        artists.extend(view["guard_arc_lines"])
        artists.extend(view["blocker_crosses"])
        artists.extend(view["platform_labels"])
        return artists

    def _refresh_artists(self) -> Tuple[object, ...]:
        self._center_local_view_on_own_target()
        self._set_global_view()
        local_artists = self._render_view(self.local_view, local=True)
        global_artists = self._render_view(self.global_view, local=False)
        self.fig.suptitle("单目标随机环绕与 UAV/USV 自适应护航守卫仿真", fontsize=15, weight="bold")
        return tuple(local_artists + global_artists)

    def update(self, _frame: int) -> Tuple[object, ...]:
        self.sim.step()
        return self._refresh_artists()

    def on_key(self, event) -> None:
        key = (event.key or "").lower()
        if key in {" ", "space"}:
            self.sim.toggle_pause()
        elif key == "n":
            self.sim.reset()
        elif key in {"escape", "esc"}:
            self.plt.close(self.fig)
            return
        else:
            return
        self._refresh_artists()
        self.fig.canvas.draw_idle()

    def show(self, interval_ms: int = 55) -> None:
        from matplotlib.animation import FuncAnimation
        self.animation = FuncAnimation(self.fig, self.update, interval=interval_ms, blit=False, cache_frame_data=False)
        self.plt.show()

    def save_snapshot(self, path: Union[str, Path], dpi: int = 150) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        self._refresh_artists()
        self.fig.savefig(output, dpi=dpi, bbox_inches="tight")
        return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-radius", type=float, default=DEFAULT_SENSOR_RADIUS)
    parser.add_argument("--reserve-count", type=int, choices=(0, 2), default=ESCORT_RESERVE_COUNT, help="是否保留 2 个贴身守卫：0 表示不保留，2 表示保留")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--headless-frames", type=int, default=0, metavar="N")
    parser.add_argument("--save-snapshot", type=str, default=None, metavar="PATH")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.headless_frames < 0:
        raise SystemExit("--headless-frames must be non-negative")
    if args.headless_frames or args.save_snapshot:
        import matplotlib
        matplotlib.use("Agg")
    sim = EscortGuardSimulator(
        sensor_radius=args.sensor_radius,
        seed=args.seed,
        num_uav=NUM_UAV,
        num_usv=NUM_USV,
        escort_reserve_count=args.reserve_count,
    )
    for _ in range(args.headless_frames):
        sim.step()
    if args.save_snapshot:
        visualizer = EscortGuardVisualizer(sim)
        output = visualizer.save_snapshot(args.save_snapshot)
        print(f"Snapshot saved: {output.resolve()}")
        visualizer.plt.close(visualizer.fig)
    if args.headless_frames:
        status = sim.status()
        print(
            "Headless dynamic-threat run completed: "
            f"frames={status['frame']}, enemies={status['enemy_count']}, "
            f"detected={status['detected_count']}, phase={status['phase']}, reserve={args.reserve_count}"
        )
        return 0
    print(
        f"UAV={NUM_UAV}, USV={NUM_USV}; 单目标随机方向出现；"
        f"随机环绕角度={RANDOM_ORBIT_MIN_ANGLE_DEG:.0f}°~{RANDOM_ORBIT_MAX_ANGLE_DEG:.0f}°；"
        f"感知半径={args.sensor_radius}; reserve_count={args.reserve_count}; "
        f"全局画布=[{WORLD_X_MIN},{WORLD_X_MAX}]x[{WORLD_Y_MIN},{WORLD_Y_MAX}]."
    )
    EscortGuardVisualizer(sim).show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
