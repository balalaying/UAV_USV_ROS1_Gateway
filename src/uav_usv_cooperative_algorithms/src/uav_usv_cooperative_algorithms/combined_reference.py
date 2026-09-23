from __future__ import annotations

"""护航守卫与 GBSFLACS 三维单目标围捕仿真。

运行示例：

    python 护航守卫_GBSFLACS_三维单目标_Python39.py
    python 护航守卫_GBSFLACS_三维单目标_Python39.py --sensor-radius 12
    python 护航守卫_GBSFLACS_三维单目标_Python39.py --reserve-count 2

依赖：numpy、scipy、scikit-learn、matplotlib。

初始化时没有敌方目标。按 N 随机生成一个目标；目标进入感知范围后先
执行护航守卫逻辑；按 J 后切换到 GBSFLACS 三维围捕。围捕期间 UAV 可
动态升降，USV、敌方目标和我方高价值目标严格保持在 z=0。按 N 只重置
敌方目标及围捕运行时，不重置我方目标或平台当前位置。近距离六个环位
按 UAV—USV 严格交替；达到捕获条件后，平台沿最后航点完成交叉队形。
"""
import argparse
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from sklearn.cluster import KMeans

# ---------------------------------------------------------------------------
# 用户可直接修改的编队参数
# ---------------------------------------------------------------------------
NUM_UAV = 3
NUM_USV = 3
ESCORT_RESERVE_COUNT = 0  # 允许设置为 0 或 2

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

UAV_ALTITUDE_MIN = 0.0
UAV_ALTITUDE_MAX = 2.0
VIEW_Z_MIN = 0.0
VIEW_Z_MAX = 10.0

RANDOM_ORBIT_MIN_ANGLE_DEG = 25.0
RANDOM_ORBIT_MAX_ANGLE_DEG = 120.0
RANDOM_ORBIT_MIN_SPEED = 0.012
RANDOM_ORBIT_MAX_SPEED = 0.024
RANDOM_ORBIT_REVERSE_PROBABILITY = 0.65

WING_ARRIVAL_TOLERANCE = 0.35
WING_READY_RATIO = 0.80

DIS_GLOBAL = 0   # 是否显示俯视图


EPS = 1e-12
DETECTED_STATES = {"detected", "forming", "orbiting"}
STATE_LABELS = {
    "waiting": "等待出现",
    "approaching": "接近中",
    "detected": "已感知",
    "forming": "守卫编队中",
    "orbiting": "环绕机动",
    "captured": "GBSFLACS 围捕成功",
}

# ---------------------------------------------------------------------------
# GBSFLACS 物理尺度适配（算法流程保持原脚本实现）
# ---------------------------------------------------------------------------
SEED = 1234
ARENA_SIZE_XY = max(WORLD_X_MAX - WORLD_X_MIN, WORLD_Y_MAX - WORLD_Y_MIN)
CAPTURE_RADIUS = 2.6
GBSFLACS_CAPTURE_RADIUS = CAPTURE_RADIUS
UAV_COUNT = NUM_UAV
USV_COUNT = NUM_USV
TARGET_COUNT = 1
TARGET_IS_STATIC = 0
TARGET_RUN_NUM = 30.0
MIN_CAPTURE_AGENTS = UAV_COUNT + USV_COUNT - 1
# --- 围捕判定模式 ---
# 'radius'：快速接近式捕获，仅要求捕获半径内人数达到阈值；
# 'geometric'：真正几何围捕，同时要求最大角度空缺不超过阈值。
CAPTURE_MODE = "geometric"
MAX_CAPTURE_ANGULAR_GAP = np.pi
USE_TARGET_PREDICTION = False
SFLA_ACTIVATION_DISTANCE = 2.0 * CAPTURE_RADIUS
USE_BASE_ENCIRCLEMENT_CONTROLLER = True
BASE_RING_RADIUS_RATIO = 0.65
BASE_RING_UAV_HEIGHT_RATIO = 0.45
MAX_EXTRA_AGENTS_PER_TARGET = 2
CS_STAGNATION_LIMIT = 2
CS_COOLDOWN = 3
MAX_ASSIGNMENT_SWITCH_RATIO = 0.15
SWITCH_MIN_ETA_GAIN = 0.25
SFLA_DEACTIVATION_DISTANCE = 1.20 * SFLA_ACTIVATION_DISTANCE
SFLA_EARLY_STOP_TOL = 1e-4
SFLA_EARLY_STOP_PATIENCE = 2
CS_QUALITY_TRIGGER = 0.45
QTH = 0.6
ALPHA = 0.4
BETA = 0.4
GAMMA = 0.2
V_MAX_UAV = 0.28
V_MAX_USV = 0.15
TARGET_SPEED = 0.30
USV_Z = 0.0
TARGET_Z = 0.0
CS_PA = 0.25
CS_BETA = 1.5


def horizontal(value: np.ndarray) -> np.ndarray:
    """Return the horizontal x/y components of a 2-D or 3-D vector."""
    arr = np.asarray(value, dtype=float)
    if arr.shape[0] < 2:
        raise ValueError("a position/vector must contain at least x and y")
    return arr[:2].copy()


def point3(value: np.ndarray, altitude: float = 0.0) -> np.ndarray:
    """Build a three-dimensional position from horizontal coordinates."""
    xy = horizontal(value)
    return np.array([xy[0], xy[1], float(altitude)], dtype=float)


def with_altitude(value: np.ndarray, altitude: float) -> np.ndarray:
    """Copy a position and force its z coordinate to the requested altitude."""
    return point3(value, altitude)


def horizontal_distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(horizontal(a) - horizontal(b)))


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
    """计算严格位于敌我水平连线内部、海拔为 0 的动态核心阻断点。"""
    if not (0.0 < ratio < 1.0):
        raise ValueError("ratio must be in (0, 1)")
    if not (0.0 <= r_min <= r_max):
        raise ValueError("expected 0 <= r_min <= r_max")
    own_xy = horizontal(own)
    enemy_xy = horizontal(enemy)
    delta = enemy_xy - own_xy
    distance = float(np.linalg.norm(delta))
    if distance <= EPS:
        delta = normalize(
            np.array([1.0, 0.0]) if fallback_direction is None else horizontal(fallback_direction),
            np.array([1.0, 0.0]),
        )
        distance = 1.0
    requested = float(np.clip(ratio * distance, r_min, r_max))
    radius = min(max(distance * 1e-9, requested), distance * (1.0 - 1e-9))
    t = radius / distance
    return point3(own_xy + t * delta, 0.0), float(t)


@dataclass
class Platform:
    identifier: str
    kind: str
    position: np.ndarray
    max_speed: float
    gain: float
    altitude: float
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
    blocker_point: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
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


class BaseAlgorithm(ABC):
    """所有对比算法的基类"""

    def __init__(self, env, name="Base"):
        self.env = env
        self.name = name
        self.metrics = {
            "capture_time": [],
            "total_distance": 0,
            "recluster_count": 0,
            "reconfiguration_count": 0,
            "energy_consumption": 0,
            "target_load_balance": 0.0,
            "success_rate": 0
        }

    @abstractmethod
    # @abstractmethod
    def step(self, agents, targets):
        """返回：{agent_id: target_id}"""
        pass

    def reset_metrics(self):
        """重置性能指标"""
        self.metrics = {
            "capture_time": [],
            "total_distance": 0,
            "recluster_count": 0,
            "reconfiguration_count": 0,
            "energy_consumption": 0,
            "target_load_balance": 0.0,
            "success_rate": 0
        }

    def _active_agents(self):
        """返回未被锁定为守卫的智能体，避免守卫继续参与分组和任务分配。"""
        guarding_ids = set()
        if hasattr(self.env, "_get_guarding_agent_ids"):
            guarding_ids = set(self.env._get_guarding_agent_ids())
        elif hasattr(self.env, "guarding_agents"):
            for guard_ids in self.env.guarding_agents.values():
                guarding_ids.update(int(agent_id) for agent_id in guard_ids)

        return [
            agent for agent in self.env.agents
            if int(agent[7]) not in guarding_ids
        ]


def calculate_target_load_balance(assignments, active_target_ids, active_agent_ids=None):
    """Return the coefficient of variation of active-target assignment loads."""
    active_target_ids = list(active_target_ids)
    if not active_target_ids:
        return 0.0

    if active_agent_ids is None:
        active_agent_ids = set(assignments)
    else:
        active_agent_ids = set(active_agent_ids)

    target_to_index = {target_id: index for index, target_id in enumerate(active_target_ids)}
    loads = np.zeros(len(active_target_ids), dtype=float)
    for agent_id, target_id in assignments.items():
        if agent_id in active_agent_ids and target_id in target_to_index:
            loads[target_to_index[target_id]] += 1

    mean_load = np.mean(loads)
    if mean_load <= 1e-12:
        return 0.0
    return float(np.std(loads) / mean_load)


def count_reconfigurations(previous_assignments, current_assignments, active_agent_ids=None):
    """Count agents whose effective target assignment changed after initialization."""
    if not previous_assignments:
        return 0

    if active_agent_ids is None:
        active_agent_ids = set(current_assignments)
    else:
        active_agent_ids = set(active_agent_ids)

    return sum(
        agent_id in previous_assignments
        and previous_assignments[agent_id] != target_id
        for agent_id, target_id in current_assignments.items()
        if agent_id in active_agent_ids
    )


def calculate_success_rate(captured_target_ids, total_targets):
    """Return the fraction of targets captured in the current run."""
    if total_targets <= 0:
        return 0.0
    return len(captured_target_ids) / total_targets


# ==================== 算法1：你的GB-SFLA（基于现有代码重构） ====================
class GBSFLACSAlgorithm(BaseAlgorithm):
    """有效的粒球-SFLA-CS混合围捕算法。

    设计原则：
    1. 粒球仅保存由 ``env.agents`` 同步得到的真实状态；
    2. SFLA 优化可执行围捕航点，不修改真实坐标或粒球观测；
    3. CS 优化完整分配/结构候选，只有综合代价下降时才接受；
    4. 目标分配先满足最低围捕容量，再优化距离、负载与重配置。
    """

    class GranularBallNode:
        def __init__(self, ball_id, agent_ids=None, parent=None):
            self.ball_id = int(ball_id)
            self.agent_ids = list(agent_ids or [])
            self.parent = parent
            self.children = []
            self.children_quality = []
            self.is_leaf = True
            self.is_empty = False
            self.points = np.empty((0, 3), dtype=float)
            self.center = np.zeros(3, dtype=float)
            self.radius = 0.0
            self.density = 0.0
            self.quality = 0.0

        def sync_from_agent_map(self, agent_map):
            self.agent_ids = [
                int(agent_id) for agent_id in self.agent_ids
                if int(agent_id) in agent_map
            ]
            if not self.agent_ids:
                self.points = np.empty((0, 3), dtype=float)
                self.center = np.zeros(3, dtype=float)
                self.radius = 0.0
                self.density = 0.0
                self.is_empty = True
                return

            self.points = np.asarray(
                [agent_map[agent_id][:3] for agent_id in self.agent_ids],
                dtype=float,
            )
            self.center = np.mean(self.points, axis=0)
            if len(self.points) > 1:
                self.radius = float(np.max(np.linalg.norm(self.points - self.center, axis=1)))
            else:
                self.radius = 0.0
            volume = (4.0 / 3.0) * np.pi * max(self.radius, 1e-6) ** 3
            self.density = float(len(self.points) / (volume + 1e-12))
            self.is_empty = False

        def update_features(self):
            """兼容环境绘图/同步代码；只根据当前 points 更新几何属性。"""
            points = np.asarray(self.points, dtype=float)
            if points.size == 0:
                self.points = np.empty((0, 3), dtype=float)
                self.center = np.zeros(3, dtype=float)
                self.radius = 0.0
                self.density = 0.0
                self.is_empty = True
                return
            self.points = points.reshape((-1, 3))
            self.center = np.mean(self.points, axis=0)
            self.radius = float(np.max(np.linalg.norm(self.points - self.center, axis=1))) if len(self.points) > 1 else 0.0
            volume = (4.0 / 3.0) * np.pi * max(self.radius, 1e-6) ** 3
            self.density = float(len(self.points) / (volume + 1e-12))
            self.is_empty = False

    def __init__(self, env, name="GB-SFLA-CS", use_sfla=True, use_cs=True):
        super().__init__(env, name)
        # 四个粒球消融版本共享同一主流程，仅通过这两个开关移除模块。
        self.use_sfla = bool(use_sfla)
        self.use_cs = bool(use_cs)
        self.QTH = 0.60
        self.split_tolerance = 0.035
        self.merge_tolerance = 0.025
        self.accept_tolerance = 1e-9

        self.sfla_population_size = 8
        self.sfla_memeplex_count = 2
        self.sfla_local_iterations = 3

        self.cs_nests = 6
        self.cs_iterations = 2
        self.cs_pa = CS_PA
        self.cs_beta = CS_BETA

        self.weights = {
            "uncovered": 100.0,
            "completion": 30.0,
            "angular": 20.0,
            "makespan": 10.0,
            "distance": 4.0,
            "balance": 3.0,
            "reconfiguration": 2.0,
            "energy": 1.0,
            "group": 2.0,
        }

        self.rng = np.random.default_rng(int(getattr(env, "seed", SEED)) + 701)
        self.next_ball_id = 0
        self.leaf_balls = []
        self.ball_tree = None
        self.all_balls = {}
        self.desired_waypoints = {}
        self.last_assignments = {}
        self.last_objective = np.inf
        self._agent_map_cache = None
        self._target_reference_cache = {}
        self._speed_cache = {}
        # SFLA 跨时间步热启动与槽位连续性状态。
        self.previous_sfla_plans = {}
        self.previous_slot_waypoints = {}
        # 共享近距离围捕激活状态。保留 sfla_active_targets 作为兼容别名。
        self.encirclement_active_targets = set()
        self.sfla_active_targets = self.encirclement_active_targets
        self.sfla_slot_continuity_weight = 0.35
        self.sfla_early_stop_tolerance = SFLA_EARLY_STOP_TOL
        self.sfla_early_stop_patience = SFLA_EARLY_STOP_PATIENCE
        # CS 事件触发状态。
        self.cs_stagnation_steps = 0
        self.cs_cooldown_remaining = 0
        self.last_cs_priority = None
        self.last_active_target_signature = None
        self.last_cs_event = None
        self.metrics['use_sfla'] = int(self.use_sfla)
        self.metrics['use_cs'] = int(self.use_cs)
        self.metrics['sfla_waypoint_steps'] = 0
        self.metrics['cs_structure_accepts'] = 0
        self.metrics['cs_assignment_accepts'] = 0
        self.metrics['cs_trigger_count'] = 0
        self.metrics['cs_cooldown_skips'] = 0
        self.metrics['assignment_makespan'] = 0.0
        self.metrics['assignment_completion_gap'] = 0.0
        self.metrics['assignment_angular_gap'] = 0.0
        self.metrics['switch_budget_used'] = 0
        self.metrics['released_redundant_agents'] = 0
        self.metrics['cs_event_target_change'] = 0
        self.metrics['cs_event_constraint'] = 0
        self.metrics['cs_event_stagnation'] = 0
        self.metrics['cs_event_quality'] = 0
        self.metrics['sfla_warm_start_hits'] = 0
        self.metrics['sfla_early_stops'] = 0
        self.metrics['sfla_activation_count'] = 0
        self.metrics['base_encirclement_activation_count'] = 0
        self.metrics['base_encirclement_waypoint_steps'] = 0

    # ------------------------------------------------------------------
    # 基础状态和粒球几何
    # ------------------------------------------------------------------
    def _get_next_id(self):
        value = self.next_ball_id
        self.next_ball_id += 1
        return value

    def _active_target_ids(self):
        return [
            target_id for target_id in range(len(self.env.targets))
            if target_id not in self.env.permanently_captured
        ]

    def _begin_step_cache(self):
        active_agents = BaseAlgorithm._active_agents(self)
        self._agent_map_cache = {int(agent[7]): agent for agent in active_agents}
        self._target_reference_cache = {}
        self._speed_cache = {
            int(agent[7]): float(V_MAX_UAV if int(agent[6]) == 0 else V_MAX_USV)
            for agent in active_agents
        }

    def _active_agent_map(self):
        if self._agent_map_cache is not None:
            return self._agent_map_cache
        return {int(agent[7]): agent for agent in BaseAlgorithm._active_agents(self)}

    def _max_speed(self, agent_id):
        agent_id = int(agent_id)
        if agent_id in self._speed_cache:
            return self._speed_cache[agent_id]
        agent = self._active_agent_map().get(agent_id)
        if agent is None:
            for candidate in self.env.agents:
                if int(candidate[7]) == int(agent_id):
                    agent = candidate
                    break
        if agent is None:
            return float(V_MAX_UAV)
        return float(V_MAX_UAV if int(agent[6]) == 0 else V_MAX_USV)

    def _new_ball(self, agent_ids, parent=None):
        ball = self.GranularBallNode(self._get_next_id(), agent_ids, parent=parent)
        ball.sync_from_agent_map(self._active_agent_map())
        ball.quality = self._calculate_cluster_quality(ball)
        return ball

    def _clone_partition(self, balls):
        return [self._new_ball(list(ball.agent_ids)) for ball in balls if ball.agent_ids]

    def _refresh_tree(self):
        self.all_balls = {ball.ball_id: ball for ball in self.leaf_balls}
        if not self.leaf_balls:
            self.ball_tree = None
        elif len(self.leaf_balls) == 1:
            self.ball_tree = self.leaf_balls[0]
            self.ball_tree.parent = None
        else:
            root_ids = [agent_id for ball in self.leaf_balls for agent_id in ball.agent_ids]
            root = self.GranularBallNode(-1, root_ids)
            root.sync_from_agent_map(self._active_agent_map())
            root.is_leaf = False
            root.children = self.leaf_balls
            root.children_quality = [float(ball.quality) for ball in self.leaf_balls]
            for ball in self.leaf_balls:
                ball.parent = root
                ball.is_leaf = True
            self.ball_tree = root
            self.all_balls[root.ball_id] = root
        self.env.ball_tree = self.ball_tree

    def _sync_real_balls(self):
        """用真实智能体位置更新粒球成员和几何，优化器不得改写真实状态。"""
        agent_map = self._active_agent_map()
        active_ids = sorted(agent_map)
        if not active_ids:
            self.leaf_balls = []
            self._refresh_tree()
            return

        if not self.leaf_balls:
            self.leaf_balls = [self._new_ball(active_ids)]
            self._refresh_tree()
            return

        # 先用上一步成员同步中心，再按最近真实中心重新归属。
        valid_balls = []
        for ball in self.leaf_balls:
            ball.agent_ids = [agent_id for agent_id in ball.agent_ids if agent_id in agent_map]
            ball.sync_from_agent_map(agent_map)
            if ball.agent_ids:
                valid_balls.append(ball)
        self.leaf_balls = valid_balls or [self._new_ball(active_ids)]

        centers = np.asarray([ball.center for ball in self.leaf_balls], dtype=float)
        memberships = {ball.ball_id: [] for ball in self.leaf_balls}
        for agent_id in active_ids:
            position = np.asarray(agent_map[agent_id][:3], dtype=float)
            nearest_index = int(np.argmin(np.linalg.norm(centers - position, axis=1)))
            memberships[self.leaf_balls[nearest_index].ball_id].append(agent_id)

        synced = []
        for ball in self.leaf_balls:
            ball.agent_ids = memberships[ball.ball_id]
            ball.sync_from_agent_map(agent_map)
            if ball.agent_ids:
                ball.quality = self._calculate_cluster_quality(ball)
                synced.append(ball)
        self.leaf_balls = synced or [self._new_ball(active_ids)]
        self._refresh_tree()

    def _calculate_cluster_quality(self, ball):
        n = len(ball.agent_ids)
        if n == 0:
            return 0.0
        if n == 1:
            return 0.25

        points = np.asarray(ball.points, dtype=float)
        avg_distance = float(np.mean(np.linalg.norm(points - ball.center, axis=1)))
        tightness = 1.0 / (1.0 + avg_distance / (ARENA_SIZE_XY * 0.1 + 1e-12))

        ideal_min = max(1, int(MIN_CAPTURE_AGENTS))
        ideal_max = ideal_min + 3
        if n < ideal_min:
            scale = n / ideal_min
        elif n <= ideal_max:
            scale = 1.0
        else:
            scale = max(0.35, 1.0 - 0.05 * (n - ideal_max))

        agent_map = self._active_agent_map()
        uav_count = sum(int(agent_map[agent_id][6]) == 0 for agent_id in ball.agent_ids if agent_id in agent_map)
        usv_count = n - uav_count
        if uav_count == 0 or usv_count == 0:
            heterogeneity = 0.5
        else:
            heterogeneity = 4.0 * uav_count * usv_count / (n * n)

        quality = ALPHA * tightness + BETA * scale + GAMMA * heterogeneity
        return float(np.clip(quality, 0.0, 1.0))

    def _ball_loss(self, ball):
        n = len(ball.agent_ids)
        min_agents = max(1, int(MIN_CAPTURE_AGENTS))
        shortage = max(0, min_agents - n) / min_agents
        oversize = max(0, n - (min_agents + 3)) / max(min_agents, 1)
        return float((1.0 - ball.quality) + 0.35 * shortage + 0.05 * oversize)

    def _partition_loss(self, balls, active_target_count):
        valid = [ball for ball in balls if ball.agent_ids]
        if not valid:
            return float("inf")
        total_agents = sum(len(ball.agent_ids) for ball in valid)
        weighted = sum(len(ball.agent_ids) * self._ball_loss(ball) for ball in valid) / max(total_agents, 1)
        feasible_group_count = max(1, min(active_target_count, total_agents // max(MIN_CAPTURE_AGENTS, 1)))
        count_gap = abs(len(valid) - feasible_group_count) / feasible_group_count
        undersized = sum(len(ball.agent_ids) < MIN_CAPTURE_AGENTS for ball in valid) / len(valid)
        return float(weighted + 0.12 * count_gap + 0.20 * undersized)

    # ------------------------------------------------------------------
    # 粒球分裂与合并：均在真实成员 ID 分区上进行
    # ------------------------------------------------------------------
    def _split_candidate(self, ball):
        minimum_child = max(2, int(MIN_CAPTURE_AGENTS))
        if len(ball.agent_ids) < 2 * minimum_child:
            return None
        if len(np.unique(np.round(ball.points, 8), axis=0)) < 2:
            return None

        labels = KMeans(n_clusters=2, random_state=SEED, n_init=10).fit_predict(ball.points)
        ids = np.asarray(ball.agent_ids, dtype=int)
        children_ids = [ids[labels == label].tolist() for label in (0, 1)]
        if any(len(group) < minimum_child for group in children_ids):
            return None
        return [self._new_ball(group) for group in children_ids]

    def _try_deterministic_split(self, ball, active_target_count):
        children = self._split_candidate(ball)
        if children is None:
            return None
        parent_loss = self._ball_loss(ball)
        child_loss = sum(len(child.agent_ids) * self._ball_loss(child) for child in children) / len(ball.agent_ids)
        capacity_pressure = len(self.leaf_balls) < active_target_count
        if child_loss + self.split_tolerance < parent_loss:
            return children
        if capacity_pressure and child_loss <= parent_loss + self.split_tolerance:
            return children
        return None

    def _try_merge_pair(self, balls, active_target_count):
        if len(balls) <= 1:
            return None
        candidates = []
        for i in range(len(balls)):
            for j in range(i + 1, len(balls)):
                a, b = balls[i], balls[j]
                distance = float(np.linalg.norm(a.center - b.center))
                candidates.append((distance, i, j))
        candidates.sort()

        current_loss = self._partition_loss(balls, active_target_count)
        for _, i, j in candidates:
            a, b = balls[i], balls[j]
            if len(a.agent_ids) >= MIN_CAPTURE_AGENTS and len(b.agent_ids) >= MIN_CAPTURE_AGENTS:
                continue
            merged = self._new_ball(a.agent_ids + b.agent_ids)
            proposal = [ball for index, ball in enumerate(balls) if index not in (i, j)] + [merged]
            proposal_loss = self._partition_loss(proposal, active_target_count)
            if proposal_loss + self.merge_tolerance < current_loss:
                return proposal
            if len(balls) > active_target_count and proposal_loss <= current_loss + self.merge_tolerance:
                return proposal
        return None

    def _adapt_structure(self, active_target_count):
        operations = 0
        max_operations = max(1, active_target_count)
        while operations < max_operations:
            candidates = sorted(
                self.leaf_balls,
                key=lambda ball: (ball.quality, -len(ball.agent_ids)),
            )
            accepted = False
            for ball in candidates:
                if ball.quality >= self.QTH and len(self.leaf_balls) >= active_target_count:
                    continue
                children = self._try_deterministic_split(ball, active_target_count)
                if children is None:
                    continue
                self.leaf_balls = [candidate for candidate in self.leaf_balls if candidate is not ball] + children
                self.metrics["recluster_count"] += 1
                operations += 1
                accepted = True
                break
            if not accepted or len(self.leaf_balls) >= active_target_count:
                break

        merged = self._try_merge_pair(self.leaf_balls, active_target_count)
        if merged is not None:
            self.leaf_balls = merged
            self.metrics["recluster_count"] += 1
        for ball in self.leaf_balls:
            ball.quality = self._calculate_cluster_quality(ball)
        self._refresh_tree()

    # ------------------------------------------------------------------
    # 目标参考位置、代价与容量优先分配
    # ------------------------------------------------------------------
    def _target_reference_position(self, target_id):
        """返回统一目标参考位置；预测开关独立于 SFLA/CS 消融开关。"""
        target_id = int(target_id)
        if target_id in self._target_reference_cache:
            return self._target_reference_cache[target_id].copy()
        target = self.env.targets[target_id]
        position = np.asarray(target[:3], dtype=float).copy()

        # 静态目标或关闭预测时，所有消融算法均使用完全相同的当前位置。
        if TARGET_IS_STATIC != 0 or not USE_TARGET_PREDICTION:
            position[2] = TARGET_Z
            self._target_reference_cache[target_id] = position.copy()
            return position

        nearby = []
        for agent in self.env.agents:
            distance_xy = float(np.linalg.norm(np.asarray(agent[:2]) - position[:2]))
            if distance_xy < TARGET_RUN_NUM:
                nearby.append(np.asarray(agent[:2], dtype=float))
        if nearby:
            mean_xy = np.mean(np.asarray(nearby), axis=0)
            direction = position[:2] - mean_xy
            norm = float(np.linalg.norm(direction))
            if norm > 1e-12:
                position[:2] += TARGET_SPEED * direction / norm
        else:
            speed = float(target[3]) if len(target) > 3 else float(TARGET_SPEED)
            theta = float(target[4]) if len(target) > 4 else 0.0
            position[:2] += speed * np.array([np.cos(theta), np.sin(theta)])
        position[2] = TARGET_Z
        self._target_reference_cache[target_id] = position.copy()
        return position

    def _arrival_cost(self, agent_id, target_id):
        agent = self._active_agent_map()[int(agent_id)]
        target_position = self._target_reference_position(int(target_id))
        return float(np.linalg.norm(np.asarray(agent[:3]) - target_position) / max(self._max_speed(agent_id), 1e-9))

    def _agent_to_ball(self, balls=None):
        mapping = {}
        for ball in (balls if balls is not None else self.leaf_balls):
            for agent_id in ball.agent_ids:
                mapping[int(agent_id)] = ball.ball_id
        return mapping

    def _capacity_requirements(self, active_ids, active_target_ids):
        requirements = {target_id: 0 for target_id in active_target_ids}
        if not active_target_ids:
            return requirements
        minimum = max(1, int(MIN_CAPTURE_AGENTS))
        if len(active_ids) >= minimum * len(active_target_ids):
            return {target_id: minimum for target_id in active_target_ids}

        base, remainder = divmod(len(active_ids), len(active_target_ids))
        for index, target_id in enumerate(active_target_ids):
            requirements[target_id] = base + (1 if index < remainder else 0)
        return requirements

    def _capacity_bounds(self, active_ids, active_target_ids):
        """返回各目标最低需求和可行冗余上限。

        当总智能体数量超过 ``最低需求 + MAX_EXTRA`` 的总容量时，按目标
        顺序均匀扩展上限，只允许不可避免的最小溢出。
        """
        requirements = self._capacity_requirements(active_ids, active_target_ids)
        upper = {
            int(target_id): int(requirements[int(target_id)] + max(0, MAX_EXTRA_AGENTS_PER_TARGET))
            for target_id in active_target_ids
        }
        shortage = max(0, len(active_ids) - sum(upper.values()))
        target_ids = [int(target_id) for target_id in active_target_ids]
        index = 0
        while shortage > 0 and target_ids:
            upper[target_ids[index % len(target_ids)]] += 1
            shortage -= 1
            index += 1
        return requirements, upper

    def _target_completion_priority(self, target_id, active_ids, requirements):
        """Return a lexicographic completion priority for one active target.

        Targets needing fewer additional in-range agents are handled first.  The
        required-member ETA breaks ties, so the allocator can finish targets
        that are already close without abandoning minimum coverage of others.
        """
        target_id = int(target_id)
        requirement = max(1, int(requirements.get(target_id, MIN_CAPTURE_AGENTS)))
        agent_map = self._active_agent_map()
        target_pos = self._target_reference_position(target_id)
        distances = []
        arrivals = []
        in_range_vectors = []
        for agent_id in active_ids:
            agent = agent_map[int(agent_id)]
            delta = np.asarray(agent[:3], dtype=float) - target_pos
            distance = float(np.linalg.norm(delta))
            distances.append(distance)
            arrivals.append(self._arrival_cost(agent_id, target_id))
            if distance <= CAPTURE_RADIUS:
                in_range_vectors.append(delta[:2])

        in_range_count = len(in_range_vectors)
        remaining = max(0, requirement - in_range_count)
        arrivals.sort()
        kth_eta = arrivals[min(requirement - 1, len(arrivals) - 1)] if arrivals else float('inf')

        angular_deficit = 1.0
        if CAPTURE_MODE == 'radius':
            angular_deficit = 0.0
        elif in_range_count >= requirement and len(in_range_vectors) >= 2:
            vectors = np.asarray(in_range_vectors, dtype=float)
            valid = np.linalg.norm(vectors, axis=1) > 1e-9
            vectors = vectors[valid]
            if len(vectors) >= 2:
                angles = np.sort(np.mod(np.arctan2(vectors[:, 1], vectors[:, 0]), 2 * np.pi))
                max_gap = float(np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi])))
                angular_deficit = max(0.0, max_gap - MAX_CAPTURE_ANGULAR_GAP) / (2 * np.pi)
        return (float(remaining), float(angular_deficit), float(kth_eta), target_id)

    def _agent_in_capture_range(self, agent_id, target_id):
        agent = self._active_agent_map()[int(agent_id)]
        target = self._target_reference_position(int(target_id))
        return bool(np.linalg.norm(np.asarray(agent[:3], dtype=float) - target) <= CAPTURE_RADIUS)

    def _capacity_aware_assignment(self, active_target_ids, balls=None):
        active_target_ids = [int(target_id) for target_id in active_target_ids]
        agent_map = self._active_agent_map()
        active_ids = sorted(agent_map)
        if not active_ids or not active_target_ids:
            return {}

        balls = balls if balls is not None else self.leaf_balls
        agent_to_ball = self._agent_to_ball(balls)
        previous = self.last_assignments or getattr(self.env, "previous_assignments", {}) or {}
        requirements, upper_bounds = self._capacity_bounds(active_ids, active_target_ids)

        assignments = {}
        loads = {target_id: 0 for target_id in active_target_ids}
        unassigned = set(active_ids)
        ball_targets = {}

        # Completion-first order: finish targets already close to capture, while
        # still assigning every active target its minimum required team.
        target_order = sorted(
            active_target_ids,
            key=lambda target_id: self._target_completion_priority(
                target_id, active_ids, requirements
            ),
        )
        target_rank = {target_id: rank for rank, target_id in enumerate(target_order)}

        for target_id in target_order:
            for _ in range(requirements[target_id]):
                if not unassigned:
                    break
                best_agent = None
                best_cost = float("inf")
                for agent_id in unassigned:
                    cost = self._arrival_cost(agent_id, target_id)
                    if self._agent_in_capture_range(agent_id, target_id):
                        cost -= 0.75
                    if agent_id in previous:
                        cost += -0.45 if int(previous[agent_id]) == target_id else 0.45
                    ball_id = agent_to_ball.get(agent_id)
                    if ball_id in ball_targets:
                        cost += -0.25 if ball_targets[ball_id] == target_id else 0.55
                    if cost < best_cost:
                        best_cost = cost
                        best_agent = agent_id
                assignments[best_agent] = target_id
                unassigned.remove(best_agent)
                loads[target_id] += 1
                ball_id = agent_to_ball.get(best_agent)
                if ball_id is not None:
                    ball_targets.setdefault(ball_id, target_id)

        mean_requirement = max(1.0, len(active_ids) / len(active_target_ids))
        rank_scale = max(len(target_order) - 1, 1)
        for agent_id in sorted(unassigned):
            available_targets = [
                target_id for target_id in active_target_ids
                if loads[target_id] < upper_bounds[target_id]
            ]
            candidate_targets = available_targets or active_target_ids
            best_target = None
            best_cost = float("inf")
            ball_id = agent_to_ball.get(agent_id)
            for target_id in candidate_targets:
                cost = self._arrival_cost(agent_id, target_id)
                cost += 0.25 * (loads[target_id] / mean_requirement)
                cost += 0.12 * (target_rank[target_id] / rank_scale)
                if self._agent_in_capture_range(agent_id, target_id):
                    cost -= 0.50
                if agent_id in previous and int(previous[agent_id]) != target_id:
                    cost += 0.55
                if ball_id in ball_targets and ball_targets[ball_id] != target_id:
                    cost += 0.50
                if cost < best_cost:
                    best_cost = cost
                    best_target = target_id
            assignments[agent_id] = int(best_target)
            loads[int(best_target)] += 1
            if ball_id is not None:
                ball_targets.setdefault(ball_id, int(best_target))

        return self._repair_assignment_capacity(assignments, active_target_ids)

    def _repair_assignment_capacity(self, assignments, active_target_ids=None):
        active_target_ids = (
            self._active_target_ids() if active_target_ids is None
            else [int(target_id) for target_id in active_target_ids]
        )
        agent_map = self._active_agent_map()
        active_ids = sorted(agent_map)
        if not active_target_ids or not active_ids:
            return {}

        active_set = set(active_target_ids)
        repaired = {}
        for agent_id in active_ids:
            target_id = assignments.get(agent_id)
            if target_id in active_set:
                repaired[agent_id] = int(target_id)
            else:
                repaired[agent_id] = min(
                    active_target_ids,
                    key=lambda candidate: self._arrival_cost(agent_id, candidate),
                )

        requirements, upper_bounds = self._capacity_bounds(active_ids, active_target_ids)
        loads = {target_id: 0 for target_id in active_target_ids}
        for target_id in repaired.values():
            loads[target_id] += 1

        # 先修复最低围捕容量缺口。
        for deficient_target in active_target_ids:
            while loads[deficient_target] < requirements[deficient_target]:
                donors = [
                    target_id for target_id in active_target_ids
                    if loads[target_id] > requirements[target_id]
                ]
                if not donors:
                    break
                move_options = []
                for agent_id, source_target in repaired.items():
                    if source_target not in donors:
                        continue
                    increase = self._arrival_cost(agent_id, deficient_target) - self._arrival_cost(agent_id, source_target)
                    if self.last_assignments.get(agent_id) == deficient_target:
                        increase -= 0.25
                    move_options.append((increase, agent_id, source_target))
                if not move_options:
                    break
                _, agent_id, source_target = min(move_options)
                repaired[agent_id] = deficient_target
                loads[source_target] -= 1
                loads[deficient_target] += 1

        # 在总容量可行时修复过度冗余，减少几十个成员集中到同一目标。
        while True:
            overloaded = [target_id for target_id in active_target_ids if loads[target_id] > upper_bounds[target_id]]
            receivers = [target_id for target_id in active_target_ids if loads[target_id] < upper_bounds[target_id]]
            if not overloaded or not receivers:
                break
            move_options = []
            for source_target in overloaded:
                if loads[source_target] <= requirements[source_target]:
                    continue
                for agent_id, assigned_target in repaired.items():
                    if assigned_target != source_target:
                        continue
                    for destination in receivers:
                        increase = self._arrival_cost(agent_id, destination) - self._arrival_cost(agent_id, source_target)
                        if self.last_assignments.get(agent_id) == destination:
                            increase -= 0.25
                        move_options.append((increase, agent_id, source_target, destination))
            if not move_options:
                break
            _, agent_id, source_target, destination = min(move_options)
            repaired[agent_id] = destination
            loads[source_target] -= 1
            loads[destination] += 1
        return repaired

    def _stabilize_assignment(self, candidate, active_target_ids):
        """Limit nonessential target switches while preserving capacity feasibility."""
        active_target_ids = [int(target_id) for target_id in active_target_ids]
        candidate = self._repair_assignment_capacity(candidate, active_target_ids)
        active_ids = sorted(self._active_agent_map())
        previous = {
            int(agent_id): int(target_id)
            for agent_id, target_id in (self.last_assignments or {}).items()
            if int(agent_id) in active_ids and int(target_id) in active_target_ids
        }
        if not previous:
            self.metrics['switch_budget_used'] = 0
            return candidate

        stabilized = {
            agent_id: previous.get(agent_id, int(candidate[agent_id]))
            for agent_id in active_ids
        }
        requirements, upper_bounds = self._capacity_bounds(active_ids, active_target_ids)
        loads = {target_id: 0 for target_id in active_target_ids}
        for target_id in stabilized.values():
            loads[target_id] += 1

        changes = []
        for agent_id in active_ids:
            old_target = stabilized[agent_id]
            new_target = int(candidate[agent_id])
            if old_target == new_target:
                continue
            eta_gain = self._arrival_cost(agent_id, old_target) - self._arrival_cost(agent_id, new_target)
            completion_bonus = (
                0.35 if self._agent_in_capture_range(agent_id, new_target) else 0.0
            )
            changes.append((eta_gain + completion_bonus, agent_id, old_target, new_target))

        # Mandatory moves first: repair deficits and overloaded old targets.
        mandatory = set()
        for destination in active_target_ids:
            while loads[destination] < requirements[destination]:
                options = [
                    item for item in changes
                    if item[3] == destination
                    and item[1] not in mandatory
                    and loads[item[2]] > requirements[item[2]]
                ]
                if not options:
                    break
                chosen = max(options)
                _, agent_id, source, target = chosen
                stabilized[agent_id] = target
                loads[source] -= 1
                loads[target] += 1
                mandatory.add(agent_id)

        for source in active_target_ids:
            while loads[source] > upper_bounds[source]:
                options = [
                    item for item in changes
                    if item[2] == source
                    and item[1] not in mandatory
                    and loads[item[3]] < upper_bounds[item[3]]
                ]
                if not options:
                    break
                chosen = max(options)
                _, agent_id, old_target, new_target = chosen
                stabilized[agent_id] = new_target
                loads[old_target] -= 1
                loads[new_target] += 1
                mandatory.add(agent_id)

        switch_budget = int(np.ceil(MAX_ASSIGNMENT_SWITCH_RATIO * len(active_ids)))
        optional_used = 0
        for gain, agent_id, source, destination in sorted(changes, reverse=True):
            if agent_id in mandatory or optional_used >= switch_budget:
                continue
            if gain < SWITCH_MIN_ETA_GAIN:
                continue
            if loads[source] <= requirements[source] or loads[destination] >= upper_bounds[destination]:
                continue
            stabilized[agent_id] = destination
            loads[source] -= 1
            loads[destination] += 1
            optional_used += 1

        stabilized = self._repair_assignment_capacity(stabilized, active_target_ids)
        switches = sum(
            agent_id in previous and int(stabilized[agent_id]) != int(previous[agent_id])
            for agent_id in active_ids
        )
        self.metrics['switch_budget_used'] = int(switches)
        return stabilized

    def _assignment_components(self, assignments, balls=None):
        """Evaluate capacity, current completion, geometry and predicted finish time."""
        active_target_ids = self._active_target_ids()
        active_agent_map = self._active_agent_map()
        active_ids = sorted(active_agent_map)
        invalid = {
            "uncovered": float("inf"), "completion": float("inf"),
            "angular": float("inf"), "makespan": float("inf"),
            "distance": float("inf"), "balance": float("inf"),
            "reconfiguration": float("inf"), "energy": float("inf"),
            "group": float("inf"),
        }
        if not active_target_ids:
            return {key: 0.0 for key in invalid}
        if set(assignments) != set(active_ids):
            return invalid
        if any(target_id not in active_target_ids for target_id in assignments.values()):
            return invalid

        requirements = self._capacity_requirements(active_ids, active_target_ids)
        loads = {target_id: 0 for target_id in active_target_ids}
        arrivals_by_target = {target_id: [] for target_id in active_target_ids}
        in_range_by_target = {target_id: [] for target_id in active_target_ids}
        arrival_times = []
        energy_terms = []
        for agent_id, target_id in assignments.items():
            target_id = int(target_id)
            loads[target_id] += 1
            arrival = self._arrival_cost(agent_id, target_id)
            arrivals_by_target[target_id].append(arrival)
            arrival_times.append(arrival)
            agent = active_agent_map[agent_id]
            target = self._target_reference_position(target_id)
            delta = np.asarray(agent[:3], dtype=float) - target
            distance = float(np.linalg.norm(delta))
            if distance <= CAPTURE_RADIUS:
                in_range_by_target[target_id].append(int(agent_id))
            energy_terms.append((distance / max(ARENA_SIZE_XY, 1)) ** 2)

        uncovered = float(sum(max(0, requirements[t] - loads[t]) for t in active_target_ids))
        completion = float(sum(max(0, requirements[t] - len(in_range_by_target[t])) for t in active_target_ids))
        angular = 0.0
        if CAPTURE_MODE == 'geometric':
            for target_id in active_target_ids:
                ids = in_range_by_target[target_id]
                if len(ids) < requirements[target_id]:
                    angular += 1.0
                    continue
                vectors = np.asarray([
                    active_agent_map[agent_id][:2] - self._target_reference_position(target_id)[:2]
                    for agent_id in ids
                ], dtype=float)
                valid = np.linalg.norm(vectors, axis=1) > 1e-9
                vectors = vectors[valid]
                if len(vectors) < 2:
                    angular += 1.0
                    continue
                angles = np.sort(np.mod(np.arctan2(vectors[:, 1], vectors[:, 0]), 2 * np.pi))
                gap = float(np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi])))
                angular += max(0.0, gap - MAX_CAPTURE_ANGULAR_GAP) / (2 * np.pi)

        horizon = max(ARENA_SIZE_XY / max(min(V_MAX_UAV, V_MAX_USV), 1), 1e-9)
        completion_times = []
        for target_id in active_target_ids:
            requirement = max(1, int(requirements[target_id]))
            arrivals = sorted(arrivals_by_target[target_id])
            if len(arrivals) >= requirement:
                completion_times.append(arrivals[requirement - 1])
            else:
                completion_times.append(horizon + float(requirement - len(arrivals)))
        makespan = float(max(completion_times, default=0.0) / horizon)
        distance_norm = float(np.mean(arrival_times) / horizon) if arrival_times else 0.0
        load_values = np.asarray(list(loads.values()), dtype=float)
        balance = float(np.std(load_values) / (np.mean(load_values) + 1e-12))
        previous = self.last_assignments or getattr(self.env, "previous_assignments", {}) or {}
        reconfiguration = sum(
            agent_id in previous and int(previous[agent_id]) != int(target_id)
            for agent_id, target_id in assignments.items()
        ) / max(len(assignments), 1)
        energy = float(np.mean(energy_terms)) if energy_terms else 0.0
        valid_balls = [ball for ball in (balls if balls is not None else self.leaf_balls) if ball.agent_ids]
        group_loss = float(np.mean([1.0 - ball.quality for ball in valid_balls])) if valid_balls else 1.0
        return {
            "uncovered": uncovered,
            "completion": completion,
            "angular": float(angular),
            "makespan": makespan,
            "distance": distance_norm,
            "balance": balance,
            "reconfiguration": float(reconfiguration),
            "energy": energy,
            "group": group_loss,
        }

    def _assignment_priority(self, assignments, balls=None):
        """Lexicographic objective: feasibility, real completion, geometry, finish time."""
        c = self._assignment_components(assignments, balls)
        return (
            c["uncovered"],
            c["completion"],
            c["angular"],
            c["makespan"],
            c["distance"],
            c["reconfiguration"],
            c["balance"],
            c["energy"],
            c["group"],
        )

    def _is_priority_better(self, candidate_priority, incumbent_priority):
        """按字典序比较，避免次要指标改善牺牲任务可行性或 makespan。"""
        for candidate, incumbent in zip(candidate_priority, incumbent_priority):
            candidate = float(candidate)
            incumbent = float(incumbent)
            if candidate + self.accept_tolerance < incumbent:
                return True
            if incumbent + self.accept_tolerance < candidate:
                return False
        return False

    def _composite_cost(self, assignments, balls=None):
        """保留标量值用于日志；CS接受决策使用 `_assignment_priority`。"""
        components = self._assignment_components(assignments, balls)
        if not all(np.isfinite(value) for value in components.values()):
            return float("inf")
        return float(sum(self.weights[key] * components[key] for key in self.weights))

    def _accept_assignment_if_better(self, incumbent, candidate, balls=None):
        incumbent_priority = self._assignment_priority(incumbent, balls)
        candidate_priority = self._assignment_priority(candidate, balls)
        if self._is_priority_better(candidate_priority, incumbent_priority):
            return True, dict(candidate), candidate_priority
        return False, dict(incumbent), incumbent_priority

    def _should_run_cs(self, current_priority, active_target_ids=None):
        """Run CS only for explicit state events, not on every simulation step."""
        if not self.use_cs:
            return False

        explicit_target_event = active_target_ids is not None
        if active_target_ids is None:
            active_target_ids = self._active_target_ids()
        else:
            active_target_ids = [int(value) for value in active_target_ids]

        signature = tuple(sorted(active_target_ids))
        target_changed = (
            explicit_target_event
            and self.last_active_target_signature is not None
            and signature != self.last_active_target_signature
        )
        if explicit_target_event:
            self.last_active_target_signature = signature

        has_constraint_gap = bool(
            current_priority and float(current_priority[0]) > self.accept_tolerance
        )
        poor_structure = any(
            ball.agent_ids and float(ball.quality) < CS_QUALITY_TRIGGER
            for ball in self.leaf_balls
        )
        stagnated = self.cs_stagnation_steps >= CS_STAGNATION_LIMIT
        initial_step = not bool(self.last_assignments)

        # Cooldown blocks ordinary quality/stagnation/constraint events.  A real
        # active-target set change bypasses cooldown because the old assignment
        # is no longer the same task instance.
        if self.cs_cooldown_remaining > 0 and not target_changed and not initial_step:
            self.metrics['cs_cooldown_skips'] += 1
            return False

        events = []
        if target_changed:
            self.metrics['cs_event_target_change'] += 1
            events.append('target_change')
        if has_constraint_gap:
            self.metrics['cs_event_constraint'] += 1
            events.append('constraint')
        if stagnated:
            self.metrics['cs_event_stagnation'] += 1
            events.append('stagnation')
        if poor_structure:
            self.metrics['cs_event_quality'] += 1
            events.append('quality')
        if initial_step:
            events.append('initial')
        self.last_cs_event = '+'.join(events) if events else None
        return bool(initial_step or target_changed or has_constraint_gap or stagnated or poor_structure)

    def _update_cs_progress(self, current_priority):
        if self.last_cs_priority is None or self._is_priority_better(current_priority, self.last_cs_priority):
            self.cs_stagnation_steps = 0
        else:
            self.cs_stagnation_steps += 1
        self.last_cs_priority = tuple(float(value) for value in current_priority)

    def _levy_flight(self, beta=None):
        beta = float(self.cs_beta if beta is None else beta)
        sigma_u = (
            math.gamma(1 + beta) * math.sin(math.pi * beta / 2)
            / (math.gamma((1 + beta) / 2) * beta * 2 ** ((beta - 1) / 2))
        ) ** (1 / beta)
        u = self.rng.normal(0, sigma_u)
        v = self.rng.normal(0, 1)
        return float(u / (abs(v) ** (1 / beta) + 1e-12))

    def _accept_if_improved(self, incumbent, incumbent_cost, candidate, candidate_cost):
        if candidate_cost + self.accept_tolerance < incumbent_cost:
            return True, dict(candidate), float(candidate_cost)
        return False, dict(incumbent), float(incumbent_cost)

    def _mutate_assignment(self, assignment, active_target_ids, balls=None):
        candidate = dict(assignment)
        active_target_ids = [int(target_id) for target_id in active_target_ids]
        if len(active_target_ids) <= 1 or not candidate:
            return candidate

        balls = balls if balls is not None else self.leaf_balls
        levy = abs(self._levy_flight())
        mutation_fraction = min(0.40, max(1.0 / len(candidate), 0.04 * levy))
        mutation_count = max(1, int(np.ceil(mutation_fraction * len(candidate))))

        selected_ids = []
        if balls and self.rng.random() < 0.65:
            ball = balls[int(self.rng.integers(0, len(balls)))]
            selected_ids.extend(ball.agent_ids)
        remaining = [agent_id for agent_id in candidate if agent_id not in selected_ids]
        if len(selected_ids) < mutation_count and remaining:
            extra_count = min(mutation_count - len(selected_ids), len(remaining))
            selected_ids.extend(self.rng.choice(remaining, size=extra_count, replace=False).tolist())

        for agent_id in selected_ids[:max(mutation_count, len(selected_ids))]:
            current_target = candidate[agent_id]
            alternatives = [target_id for target_id in active_target_ids if target_id != current_target]
            if not alternatives:
                continue
            if self.rng.random() < 0.65:
                candidate[agent_id] = min(alternatives, key=lambda target_id: self._arrival_cost(agent_id, target_id))
            else:
                candidate[agent_id] = int(self.rng.choice(alternatives))
        return self._repair_assignment_capacity(candidate, active_target_ids)

    def _try_cs_structure_improvement(self, assignment, active_target_ids):
        incumbent_balls = self.leaf_balls
        incumbent_assignment = dict(assignment)
        incumbent_priority = self._assignment_priority(incumbent_assignment, incumbent_balls)

        for _ in range(max(2, self.cs_iterations)):
            proposal = self._clone_partition(incumbent_balls)
            if not proposal:
                break
            if self.rng.random() < 0.65:
                splittable = [ball for ball in proposal if len(ball.agent_ids) >= 2 * max(2, MIN_CAPTURE_AGENTS)]
                if not splittable:
                    continue
                chosen = min(splittable, key=lambda ball: ball.quality)
                children = self._split_candidate(chosen)
                if children is None:
                    continue
                proposal = [ball for ball in proposal if ball is not chosen] + children
            else:
                merged = self._try_merge_pair(proposal, len(active_target_ids))
                if merged is None:
                    continue
                proposal = merged

            proposal_assignment = self._capacity_aware_assignment(active_target_ids, proposal)
            proposal_priority = self._assignment_priority(proposal_assignment, proposal)
            if self._is_priority_better(proposal_priority, incumbent_priority):
                incumbent_balls = proposal
                incumbent_assignment = dict(proposal_assignment)
                incumbent_priority = proposal_priority
                self.metrics["recluster_count"] += 1
                self.metrics['cs_structure_accepts'] += 1

        self.leaf_balls = incumbent_balls
        self._refresh_tree()
        return incumbent_assignment, self._composite_cost(incumbent_assignment, incumbent_balls)

    def _try_cs_assignment_improvement(self, assignment, active_target_ids):
        incumbent = dict(assignment)
        incumbent_priority = self._assignment_priority(incumbent, self.leaf_balls)
        nests = [dict(incumbent)]
        for _ in range(self.cs_nests - 1):
            nests.append(self._mutate_assignment(incumbent, active_target_ids, self.leaf_balls))

        for _ in range(self.cs_iterations):
            updated = []
            for nest in nests:
                candidate = self._mutate_assignment(nest, active_target_ids, self.leaf_balls)
                accepted, chosen, _ = self._accept_assignment_if_better(nest, candidate, self.leaf_balls)
                updated.append(chosen if accepted else dict(nest))
            nests = updated

            priorities = [self._assignment_priority(nest, self.leaf_balls) for nest in nests]
            abandon_count = max(1, int(np.ceil(self.cs_pa * len(nests))))
            worst_indices = sorted(range(len(nests)), key=lambda index: priorities[index], reverse=True)[:abandon_count]
            for index in worst_indices:
                if self.rng.random() < self.cs_pa:
                    nests[index] = self._mutate_assignment(incumbent, active_target_ids, self.leaf_balls)

            best_nest = min(nests, key=lambda nest: self._assignment_priority(nest, self.leaf_balls))
            best_priority = self._assignment_priority(best_nest, self.leaf_balls)
            if self._is_priority_better(best_priority, incumbent_priority):
                incumbent = dict(best_nest)
                incumbent_priority = best_priority
                self.metrics['cs_assignment_accepts'] += 1
        return incumbent, self._composite_cost(incumbent, self.leaf_balls)

    def _plan_bounds_clip(self, params):
        clipped = np.asarray(params, dtype=float).copy()
        clipped[0] = clipped[0] % (2 * np.pi)
        clipped[1] = np.clip(clipped[1], 0.45, 0.72)
        clipped[2] = np.clip(clipped[2], 0.25, 0.75)
        return clipped

    def _encirclement_should_activate(self, target_id, critical_distance):
        """共享基础围捕控制器的近距离激活/退出滞回。"""
        target_id = int(target_id)
        if CAPTURE_MODE != 'geometric' or not USE_BASE_ENCIRCLEMENT_CONTROLLER:
            self.encirclement_active_targets.discard(target_id)
            return False

        was_active = target_id in self.encirclement_active_targets
        threshold = SFLA_DEACTIVATION_DISTANCE if was_active else SFLA_ACTIVATION_DISTANCE
        active = float(critical_distance) <= float(threshold)
        if active and not was_active:
            self.encirclement_active_targets.add(target_id)
            self.metrics['base_encirclement_activation_count'] += 1
            if self.use_sfla:
                self.metrics['sfla_activation_count'] += 1
        elif not active:
            self.encirclement_active_targets.discard(target_id)
        return active

    def _sfla_should_activate(self, target_id, critical_distance):
        """兼容旧接口；SFLA与基础控制器共享相同的近距离激活状态。"""
        return self._encirclement_should_activate(target_id, critical_distance)

    def _base_plan_parameters(self, agent_ids, target_id):
        """构造确定性的基础环形方案 ``[phase, radius_ratio, height_ratio]``。"""
        agent_ids = [int(agent_id) for agent_id in agent_ids]
        target_id = int(target_id)
        target_pos = self._target_reference_position(target_id)
        agent_map = self._active_agent_map()
        positions = np.asarray(
            [np.asarray(agent_map[agent_id][:3], dtype=float) for agent_id in agent_ids],
            dtype=float,
        )
        if len(positions):
            centroid = np.mean(positions[:, :2], axis=0)
            direction = centroid - target_pos[:2]
            if float(np.linalg.norm(direction)) > 1e-9:
                phase = float(np.mod(np.arctan2(direction[1], direction[0]), 2 * np.pi))
            else:
                phase = float(self.previous_sfla_plans.get(target_id, np.array([0.0]))[0])
        else:
            phase = 0.0
        return self._plan_bounds_clip(np.array([
            phase,
            BASE_RING_RADIUS_RATIO,
            BASE_RING_UAV_HEIGHT_RATIO,
        ], dtype=float))

    def _build_base_encirclement_waypoints(self, assignments, store_history=True):
        """为所有 GB 消融算法生成共享的远追击/近环形基础航点。"""
        active_agent_ids = {int(agent_id) for agent_id in assignments}
        active_target_ids = {int(target_id) for target_id in assignments.values()}
        self.previous_slot_waypoints = {
            int(agent_id): np.asarray(waypoint, dtype=float)
            for agent_id, waypoint in self.previous_slot_waypoints.items()
            if int(agent_id) in active_agent_ids
        }
        self.encirclement_active_targets.intersection_update(active_target_ids)

        grouped = {}
        for agent_id, target_id in assignments.items():
            grouped.setdefault(int(target_id), []).append(int(agent_id))

        waypoints = {}
        agent_map = self._active_agent_map()
        for target_id, agent_ids in grouped.items():
            target_ref = self._target_reference_position(target_id)
            distances = sorted(
                float(np.linalg.norm(np.asarray(agent_map[agent_id][:3]) - target_ref))
                for agent_id in agent_ids
            )
            required = min(max(1, int(MIN_CAPTURE_AGENTS)), len(distances))
            critical_distance = distances[required - 1] if distances else float('inf')

            if not self._encirclement_should_activate(target_id, critical_distance):
                for agent_id in agent_ids:
                    waypoint = np.asarray(target_ref, dtype=float).copy()
                    if int(agent_map[agent_id][6]) == 1:
                        waypoint[2] = USV_Z
                    waypoints[int(agent_id)] = waypoint
                continue

            base_params = self._base_plan_parameters(agent_ids, target_id)
            waypoints.update(
                self._plan_to_waypoints(
                    agent_ids, target_id, base_params, store_history=False
                )
            )
            self.metrics['base_encirclement_waypoint_steps'] += 1

        if store_history:
            for agent_id, waypoint in waypoints.items():
                self.previous_slot_waypoints[int(agent_id)] = np.asarray(waypoint, dtype=float).copy()
        return waypoints

    def _plan_to_waypoints(self, agent_ids, target_id, params, store_history=False):
        agent_ids = [int(agent_id) for agent_id in agent_ids]
        if len(agent_ids) != 6:
            raise ValueError(
                "alternating encirclement requires exactly 3 UAV and 3 USV"
            )
        if len(set(agent_ids)) != 6:
            raise ValueError(
                "alternating encirclement requires six unique platform IDs"
            )
        agent_map = self._active_agent_map()
        unknown_ids = sorted(
            agent_id for agent_id in agent_ids if agent_id not in agent_map
        )
        if unknown_ids:
            unknown_text = ", ".join(str(agent_id) for agent_id in unknown_ids)
            raise ValueError(
                f"alternating encirclement received unknown platform IDs: "
                f"{unknown_text}"
            )

        params = self._plan_bounds_clip(params)
        target_pos = self._target_reference_position(target_id)
        ring_radius = float(params[1] * CAPTURE_RADIUS)
        uav_ids = [
            agent_id for agent_id in agent_ids
            if int(agent_map[agent_id][6]) == 0
        ]
        usv_ids = [
            agent_id for agent_id in agent_ids
            if int(agent_map[agent_id][6]) == 1
        ]
        if len(uav_ids) != 3 or len(usv_ids) != 3:
            raise ValueError(
                "alternating encirclement requires exactly 3 UAV and 3 USV"
            )

        # 偶数槽位为 UAV，奇数槽位为 USV；SFLA 只优化整个环的
        # 相位、半径和 UAV 高度，不再允许全局匹配改变平台类型顺序。
        angles = params[0] + 2.0 * np.pi * np.arange(6) / 6.0
        max_vertical = np.sqrt(
            max(CAPTURE_RADIUS ** 2 - ring_radius ** 2, 0.0)
        )
        uav_z = target_pos[2] + float(params[2] * 0.80 * max_vertical)
        slot_altitudes = np.where(np.arange(6) % 2 == 0, uav_z, USV_Z)
        slots = np.column_stack([
            target_pos[0] + ring_radius * np.cos(angles),
            target_pos[1] + ring_radius * np.sin(angles),
            slot_altitudes,
        ])

        waypoints = {}
        for typed_agent_ids, slot_indices in (
            (uav_ids, np.array([0, 2, 4], dtype=int)),
            (usv_ids, np.array([1, 3, 5], dtype=int)),
        ):
            candidate_slots = slots[slot_indices]
            cost_matrix = np.zeros((3, 3), dtype=float)
            for row, agent_id in enumerate(typed_agent_ids):
                agent = agent_map[agent_id]
                arrival_cost = (
                    np.linalg.norm(
                        candidate_slots - np.asarray(agent[:3]), axis=1
                    )
                    / max(self._max_speed(agent_id), 1e-9)
                )
                previous_slot = self.previous_slot_waypoints.get(agent_id)
                if previous_slot is not None:
                    continuity_cost = (
                        np.linalg.norm(
                            candidate_slots - np.asarray(previous_slot), axis=1
                        )
                        / max(CAPTURE_RADIUS, 1e-9)
                    )
                    arrival_cost = (
                        arrival_cost
                        + self.sfla_slot_continuity_weight * continuity_cost
                    )
                cost_matrix[row] = arrival_cost

            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for row, col in zip(row_ind, col_ind):
                agent_id = typed_agent_ids[int(row)]
                waypoint = np.asarray(
                    candidate_slots[int(col)], dtype=float
                ).copy()
                delta = waypoint - target_pos
                distance = float(np.linalg.norm(delta))
                if distance > CAPTURE_RADIUS:
                    waypoint = target_pos + delta * (
                        (CAPTURE_RADIUS * 0.98) / max(distance, 1e-12)
                    )
                if int(agent_map[agent_id][6]) == 1:
                    waypoint[2] = USV_Z
                waypoints[agent_id] = waypoint

        if store_history:
            for agent_id, waypoint in waypoints.items():
                self.previous_slot_waypoints[int(agent_id)] = np.asarray(waypoint, dtype=float).copy()
        return waypoints

    def _local_plan_cost(self, agent_ids, target_id, params):
        waypoints = self._plan_to_waypoints(agent_ids, target_id, params, store_history=False)
        if not waypoints:
            return float("inf")
        agent_map = self._active_agent_map()
        target_pos = self._target_reference_position(target_id)
        arrivals = []
        angles = []
        radii = []
        positions = []
        continuity_terms = []
        for agent_id, waypoint in waypoints.items():
            arrivals.append(np.linalg.norm(waypoint - np.asarray(agent_map[agent_id][:3])) / max(self._max_speed(agent_id), 1e-9))
            vector = waypoint[:2] - target_pos[:2]
            angles.append(float(np.mod(np.arctan2(vector[1], vector[0]), 2 * np.pi)))
            radii.append(float(np.linalg.norm(waypoint - target_pos)))
            positions.append(waypoint)
            previous_slot = self.previous_slot_waypoints.get(int(agent_id))
            if previous_slot is not None:
                continuity_terms.append(float(np.linalg.norm(waypoint - previous_slot) / max(CAPTURE_RADIUS, 1e-9)))

        sorted_angles = np.sort(np.asarray(angles))
        gaps = np.diff(np.r_[sorted_angles, sorted_angles[0] + 2 * np.pi]) if len(sorted_angles) > 1 else np.array([2 * np.pi])
        ideal_gap = 2 * np.pi / max(len(sorted_angles), 1)
        angular_error = float(np.max(np.abs(gaps - ideal_gap)) / (2 * np.pi))
        radius_error = float(np.mean(np.abs(np.asarray(radii) - np.mean(radii))) / max(CAPTURE_RADIUS, 1))

        collision_penalty = 0.0
        positions = np.asarray(positions)
        if len(positions) > 1:
            pairwise = cdist(positions, positions)
            pairwise += np.eye(len(positions)) * 1e9
            minimum_spacing = CAPTURE_RADIUS * 0.25
            collision_penalty = float(np.mean(np.maximum(0.0, minimum_spacing - pairwise) / minimum_spacing))

        arrival_norm = float(np.mean(arrivals) / max(ARENA_SIZE_XY / V_MAX_UAV, 1e-9))
        slot_continuity = float(np.mean(continuity_terms)) if continuity_terms else 0.0
        return float(
            4.0 * arrival_norm
            + 2.0 * angular_error
            + radius_error
            + 2.0 * collision_penalty
            + self.sfla_slot_continuity_weight * slot_continuity
        )

    def _sfla_target_waypoints(self, agent_ids, target_id, base_params=None):
        if not agent_ids:
            return {}
        target_id = int(target_id)
        base_candidate = self._plan_bounds_clip(
            self._base_plan_parameters(agent_ids, target_id)
            if base_params is None else np.asarray(base_params, dtype=float)
        )
        has_warm_start = target_id in self.previous_sfla_plans
        if has_warm_start:
            self.metrics['sfla_warm_start_hits'] += 1

        # 基础方案始终作为一只青蛙进入种群，确保 SFLA 在同一控制基线上优化。
        population = [base_candidate.copy()]
        if has_warm_start and self.sfla_population_size > 1:
            warm_start = self._plan_bounds_clip(
                np.asarray(self.previous_sfla_plans[target_id], dtype=float)
            )
            if not np.allclose(warm_start, base_candidate):
                population.append(warm_start)
        while len(population) < self.sfla_population_size:
            params = np.array([
                self.rng.uniform(0, 2 * np.pi),
                self.rng.uniform(0.45, 0.72),
                self.rng.uniform(0.25, 0.75),
            ])
            population.append(self._plan_bounds_clip(params))

        best_seen = float('inf')
        no_improvement = 0
        for _ in range(self.sfla_local_iterations):
            costs = np.asarray([
                self._local_plan_cost(agent_ids, target_id, params)
                for params in population
            ])
            order = np.argsort(costs)
            current_best = float(costs[int(order[0])])
            if best_seen - current_best > self.sfla_early_stop_tolerance:
                best_seen = current_best
                no_improvement = 0
            else:
                no_improvement += 1
                if no_improvement >= self.sfla_early_stop_patience:
                    self.metrics['sfla_early_stops'] += 1
                    break

            global_best = population[int(order[0])].copy()
            memeplexes = [
                order[index::self.sfla_memeplex_count].tolist()
                for index in range(self.sfla_memeplex_count)
            ]
            for memeplex in memeplexes:
                if len(memeplex) < 2:
                    continue
                local_best_index = min(memeplex, key=lambda index: costs[index])
                worst_index = max(memeplex, key=lambda index: costs[index])
                worst = population[worst_index]
                local_best = population[local_best_index]

                candidate = self._plan_bounds_clip(
                    worst + self.rng.random(3) * (local_best - worst)
                )
                candidate_cost = self._local_plan_cost(agent_ids, target_id, candidate)
                if candidate_cost >= costs[worst_index]:
                    candidate = self._plan_bounds_clip(
                        worst + self.rng.random(3) * (global_best - worst)
                    )
                    candidate_cost = self._local_plan_cost(agent_ids, target_id, candidate)
                if candidate_cost >= costs[worst_index]:
                    candidate = self._plan_bounds_clip(np.array([
                        self.rng.uniform(0, 2 * np.pi),
                        self.rng.uniform(0.45, 0.72),
                        self.rng.uniform(0.25, 0.75),
                    ]))
                    candidate_cost = self._local_plan_cost(agent_ids, target_id, candidate)
                if candidate_cost < costs[worst_index]:
                    population[worst_index] = candidate

        best = min(
            population,
            key=lambda params: self._local_plan_cost(agent_ids, target_id, params),
        )
        self.previous_sfla_plans[target_id] = np.asarray(best, dtype=float).copy()
        return self._plan_to_waypoints(agent_ids, target_id, best, store_history=True)

    def _build_sfla_waypoints(self, assignments, base_waypoints=None):
        """在共享基础航点上，仅对已进入近距离的目标执行 SFLA 优化。"""
        if not self.use_sfla:
            return dict(base_waypoints or {})

        active_agent_ids = {int(agent_id) for agent_id in assignments}
        active_target_ids = {int(target_id) for target_id in assignments.values()}
        self.previous_slot_waypoints = {
            int(agent_id): np.asarray(waypoint, dtype=float)
            for agent_id, waypoint in self.previous_slot_waypoints.items()
            if int(agent_id) in active_agent_ids
        }
        self.previous_sfla_plans = {
            int(target_id): np.asarray(plan, dtype=float)
            for target_id, plan in self.previous_sfla_plans.items()
            if int(target_id) in active_target_ids
        }
        self.encirclement_active_targets.intersection_update(active_target_ids)

        grouped = {}
        for agent_id, target_id in assignments.items():
            grouped.setdefault(int(target_id), []).append(int(agent_id))

        waypoints = dict(base_waypoints or {})
        agent_map = self._active_agent_map()
        for target_id, agent_ids in grouped.items():
            target_ref = self._target_reference_position(target_id)
            distances = sorted(
                float(np.linalg.norm(np.asarray(agent_map[agent_id][:3]) - target_ref))
                for agent_id in agent_ids
            )
            required = min(max(1, int(MIN_CAPTURE_AGENTS)), len(distances))
            critical_distance = distances[required - 1] if distances else float('inf')

            if not self._encirclement_should_activate(target_id, critical_distance):
                continue

            base_params = self._base_plan_parameters(agent_ids, target_id)
            waypoints.update(
                self._sfla_target_waypoints(
                    agent_ids, target_id, base_params=base_params
                )
            )
            self.metrics['sfla_waypoint_steps'] += 1

        for agent_id, waypoint in waypoints.items():
            self.previous_slot_waypoints[int(agent_id)] = np.asarray(waypoint, dtype=float).copy()
        return waypoints

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def _get_all_leaves(self, node):
        if node is None:
            return []
        if node.is_leaf:
            return [node]
        leaves = []
        for child in node.children:
            leaves.extend(self._get_all_leaves(child))
        return leaves

    def step(self):
        self._begin_step_cache()
        active_target_ids = self._active_target_ids()
        if not active_target_ids or not self._active_agent_map():
            self.desired_waypoints = {}
            return {}

        if self.cs_cooldown_remaining > 0:
            self.cs_cooldown_remaining -= 1

        self._sync_real_balls()
        self._adapt_structure(len(active_target_ids))

        assignment = self._capacity_aware_assignment(active_target_ids, self.leaf_balls)
        base_priority = self._assignment_priority(assignment, self.leaf_balls)

        if self._should_run_cs(base_priority, active_target_ids):
            self.metrics['cs_trigger_count'] += 1
            assignment, _ = self._try_cs_structure_improvement(assignment, active_target_ids)
            assignment, _ = self._try_cs_assignment_improvement(assignment, active_target_ids)
            self.cs_cooldown_remaining = CS_COOLDOWN

        assignment = self._repair_assignment_capacity(assignment, active_target_ids)
        assignment = self._stabilize_assignment(assignment, active_target_ids)
        final_priority = self._assignment_priority(assignment, self.leaf_balls)
        final_components = self._assignment_components(assignment, self.leaf_balls)
        objective = self._composite_cost(assignment, self.leaf_balls)
        self._update_cs_progress(final_priority)

        base_waypoints = self._build_base_encirclement_waypoints(
            assignment, store_history=not self.use_sfla
        )
        self.desired_waypoints = (
            self._build_sfla_waypoints(assignment, base_waypoints=base_waypoints)
            if self.use_sfla
            else base_waypoints
        )
        self.last_assignments = dict(assignment)
        self.last_objective = float(objective)
        self.metrics['assignment_makespan'] = float(final_components['makespan'])
        self.metrics['assignment_completion_gap'] = float(final_components['completion'])
        self.metrics['assignment_angular_gap'] = float(final_components['angular'])
        self._refresh_tree()

        if bool(getattr(self.env, "gbsflacs_verbose", False)):
            print(
                f"   Assignments ({self.name}, SFLA={self.use_sfla}, CS={self.use_cs}): "
                f"{assignment}; balls={len(self.leaf_balls)}, objective={self.last_objective:.4f}, "
                f"makespan={final_components['makespan']:.4f}, "
                f"completion_gap={final_components['completion']:.1f}"
            )
        return assignment


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
        if int(num_uav) != NUM_UAV or int(num_usv) != NUM_USV:
            raise ValueError("GBSFLACS merged scene requires exactly 3 UAV and 3 USV")
        if num_uav < 0 or num_usv < 0 or platform_count < 6:
            raise ValueError("The UAV/USV total must be at least 6")
        if escort_reserve_count not in {0, 2}:
            raise ValueError("escort_reserve_count must be 0 or 2")
        if support_guard_radius <= own_target_avoid_radius:
            raise ValueError("support_guard_radius must exceed the own-target safety radius")
        if not (0.0 <= support_arc_half_angle_deg < 180.0):
            raise ValueError("support_arc_half_angle_deg must be in [0, 180)")
        if sensor_radius <= max(guard_arc_radius, support_guard_radius, own_target_avoid_radius):
            raise ValueError("sensor_radius must exceed guard and safety radii")
        if not (0.0 < wing_ready_ratio <= 1.0):
            raise ValueError("wing_ready_ratio must be in (0, 1]")
        if not (0.0 < random_orbit_min_angle_deg <= random_orbit_max_angle_deg <= 360.0):
            raise ValueError("random orbit angles must satisfy 0 < min <= max <= 360")
        if not (0.0 < random_orbit_min_speed <= random_orbit_max_speed):
            raise ValueError("random orbit speeds must satisfy 0 < min <= max")
        if not (0.0 <= random_orbit_reverse_probability <= 1.0):
            raise ValueError("random_orbit_reverse_probability must be in [0, 1]")

        self.seed = int(seed)
        self.reset_count = 0
        self.rng = np.random.default_rng(self.seed)
        self.num_uav = int(num_uav)
        self.num_usv = int(num_usv)
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
        self.initial_own_position = np.array([OWN_START_X, OWN_START_Y, 0.0], dtype=float)
        if not (
            self.world_x_min + self.ring_radius < self.initial_own_position[0] < self.world_x_max - self.ring_radius
            and self.world_y_min + self.ring_radius < self.initial_own_position[1] < self.world_y_max - self.ring_radius
        ):
            raise ValueError("The initial escort position must leave room for the escort ring")

        self.forward = np.array([1.0, 0.0], dtype=float)
        self.own_position = self.initial_own_position.copy()
        self.own_goal = self.own_position.copy()
        self.avoid_direction = np.zeros(2, dtype=float)
        self.frame = 0
        self.paused = False
        self.phase = "正常护航"
        # self.last_message = "敌方目标尚未进入感知范围"

        self.last_message = "当前没有敌方目标，请按 N 键随机生成"

        self.platforms = self._create_mixed_ring()
        # self.threats = self._create_threat_schedule()

        # 初始化时没有敌方目标，按 N 键后再随机生成
        self.threats: List[ThreatTask] = []

        self.reserve_guard_indices: List[int] = []
        self._reserve_slot_offsets: Dict[int, np.ndarray] = {}
        self._reserve_slot_by_index: Dict[int, int] = {}
        self.support_guard_indices: List[int] = []
        self._support_slot_by_index: Dict[int, int] = {}
        self._last_detected_ids: Tuple[int, ...] = ()
        self._own_target_bypass_side: Dict[int, int] = {}
        self.gbsflacs_active = False
        self.capture_success = False
        self.gbsflacs_verbose = False
        self.permanently_captured: set = set()
        self.guarding_agents: Dict[int, set] = {}
        self.previous_assignments: Dict[int, int] = {}
        self.ball_tree = None
        self.agents = np.empty((0, 8), dtype=float)
        self.targets = np.empty((0, 6), dtype=float)
        self.last_gbsflacs_assignments: Dict[int, int] = {}
        self.gbsflacs_algorithm = GBSFLACSAlgorithm(self)
        self._sync_gbsflacs_environment()

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
            horizontal_position = horizontal(self.own_position) + self.ring_radius * np.array(
                [math.cos(angle), math.sin(angle)], dtype=float
            )
            if kind == "UAV":
                uav_no += 1
                altitude = float(self.rng.uniform(UAV_ALTITUDE_MIN, UAV_ALTITUDE_MAX))
                position = point3(horizontal_position, altitude)
                result.append(Platform(f"U{uav_no}", kind, position, 0.28, 0.42, altitude))
            else:
                usv_no += 1
                altitude = 0.0
                position = point3(horizontal_position, altitude)
                result.append(Platform(f"S{usv_no}", kind, position, 0.15, 0.32, altitude))
        return result

    def _create_threat_schedule(self) -> List[ThreatTask]:
        """Create exactly one target at a uniformly random bearing."""
        angle = float(self.rng.uniform(0.0, 2.0 * math.pi))
        radius = float(self.rng.uniform(self.sensor_radius + 4.0, self.sensor_radius + 8.0))
        position = point3(
            horizontal(self.own_position)
            + radius * np.array([math.cos(angle), math.sin(angle)], dtype=float),
            0.0,
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
        """按 N 生成新目标；保留我方目标、平台位置和 UAV 当前高度。"""
        self.reset_count += 1
        self.gbsflacs_active = False
        self.capture_success = False
        for platform in self.platforms:
            platform.role = "escort"
            platform.goal = platform.position.copy()
            platform.assigned_threat_id = None
            if platform.kind == "UAV":
                platform.altitude = float(platform.position[2])
            else:
                platform.altitude = 0.0
                platform.position[2] = 0.0

        self.reserve_guard_indices = []
        self._reserve_slot_offsets = {}
        self._reserve_slot_by_index = {}
        self.support_guard_indices = []
        self._support_slot_by_index = {}
        self._last_detected_ids = ()
        self._own_target_bypass_side = {}
        self.avoid_direction = np.zeros(2, dtype=float)

        angle = float(self.rng.uniform(0.0, 2.0 * math.pi))
        radius = float(self.rng.uniform(self.sensor_radius + 4.0, self.sensor_radius + 8.0))
        position = point3(
            horizontal(self.own_position)
            + radius * np.array([math.cos(angle), math.sin(angle)], dtype=float),
            0.0,
        )
        self.threats = [
            ThreatTask(
                threat_id=1,
                spawn_frame=self.frame,
                spawn_angle=angle,
                spawn_radius=radius,
                position=position,
                state="approaching",
                current_speed_limit=self.enemy_approach_speed,
            )
        ]
        self.phase = "正常护航"
        self.last_message = "已随机生成敌方目标；我方编队位置与 UAV 当前高度保持不变"
        self._reset_gbsflacs_runtime()

    def toggle_pause(self) -> bool:
        self.paused = not self.paused
        return self.paused

    def _reset_gbsflacs_runtime(self) -> None:
        """Clear every target-specific GBSFLACS state for a newly spawned target."""
        self.permanently_captured = set()
        self.guarding_agents = {}
        self.previous_assignments = {}
        self.ball_tree = None
        self.last_gbsflacs_assignments = {}
        self.gbsflacs_algorithm = GBSFLACSAlgorithm(self)
        self._sync_gbsflacs_environment()

    def _get_guarding_agent_ids(self) -> set:
        """The merged single-target scene releases no platform before completion."""
        result = set()
        for agent_ids in self.guarding_agents.values():
            result.update(int(agent_id) for agent_id in agent_ids)
        return result

    def _sync_gbsflacs_environment(self) -> None:
        """Expose the escort scene through the array contract used by GBSFLACS."""
        rows = []
        for index, platform in enumerate(self.platforms):
            rows.append(
                [
                    float(platform.position[0]),
                    float(platform.position[1]),
                    float(platform.position[2]),
                    0.0,
                    0.0,
                    0.0,
                    0.0 if platform.kind == "UAV" else 1.0,
                    float(index),
                ]
            )
        self.agents = np.asarray(rows, dtype=float).reshape((-1, 8))

        target_rows = []
        for task in self.threats:
            if task.state == "waiting":
                continue
            target_rows.append(
                [
                    float(task.position[0]),
                    float(task.position[1]),
                    0.0,
                    float(task.current_speed_limit),
                    float(task.controlled_angle),
                    float(task.threat_id),
                ]
            )
        self.targets = np.asarray(target_rows, dtype=float).reshape((-1, 6))

    def _move_platform_to_gbsflacs_waypoint(
        self, index: int, waypoint: np.ndarray
    ) -> None:
        """Move one platform toward a GBSFLACS waypoint in full three dimensions."""
        platform = self.platforms[index]
        desired = np.asarray(waypoint, dtype=float).copy()
        if desired.shape != (3,) or not np.all(np.isfinite(desired)):
            desired = platform.position.copy()
        if platform.kind == "USV":
            desired[2] = 0.0
        else:
            desired[2] = float(np.clip(desired[2], UAV_ALTITUDE_MIN, VIEW_Z_MAX))

        delta = desired - platform.position
        distance = float(np.linalg.norm(delta))
        max_step = platform.max_speed * self.dt
        if distance > EPS:
            platform.position = platform.position + delta * (min(distance, max_step) / distance)
        platform.position = self._clip_position_to_world(platform.position, margin=0.2)
        platform.position[2] = (
            0.0
            if platform.kind == "USV"
            else float(np.clip(platform.position[2], UAV_ALTITUDE_MIN, VIEW_Z_MAX))
        )
        platform.goal = desired.copy()
        platform.role = "capture"
        platform.assigned_threat_id = self.threats[0].threat_id

    def _step_gbsflacs_capture(self) -> None:
        """Run one GBSFLACS decision and apply its three-dimensional waypoints."""
        self._sync_gbsflacs_environment()
        assignments = self.gbsflacs_algorithm.step()
        self.last_gbsflacs_assignments = {
            int(agent_id): int(target_id)
            for agent_id, target_id in assignments.items()
        }
        waypoints = self.gbsflacs_algorithm.desired_waypoints
        for index, platform in enumerate(self.platforms):
            waypoint = waypoints.get(index, platform.position)
            self._move_platform_to_gbsflacs_waypoint(index, waypoint)
        self.previous_assignments = self.last_gbsflacs_assignments.copy()
        self._sync_gbsflacs_environment()

    def _maximum_capture_angular_gap(
        self, platform_indices: Sequence[int], target_position: np.ndarray
    ) -> float:
        """Return the largest uncovered XY bearing interval around the target."""
        indices = [int(index) for index in platform_indices]
        if len(indices) < 2:
            return float(2.0 * np.pi)
        target_xy = horizontal(target_position)
        vectors = np.asarray(
            [horizontal(self.platforms[index].position) - target_xy for index in indices],
            dtype=float,
        )
        vectors = vectors[np.linalg.norm(vectors, axis=1) > 1e-9]
        if len(vectors) < 2:
            return float(2.0 * np.pi)
        angles = np.sort(
            np.mod(np.arctan2(vectors[:, 1], vectors[:, 0]), 2.0 * np.pi)
        )
        gaps = np.diff(np.r_[angles, angles[0] + 2.0 * np.pi])
        return float(np.max(gaps))

    def _capture_condition_met(self) -> bool:
        """Check the shared three-dimensional radius and XY coverage criterion."""
        if not self.threats or self.threats[0].state in {
            "waiting",
            "approaching",
            "captured",
        }:
            return False
        target_position = self.threats[0].position
        distances = np.asarray(
            [
                np.linalg.norm(
                    np.asarray(platform.position, dtype=float) - target_position
                )
                for platform in self.platforms
            ],
            dtype=float,
        )
        in_range = np.where(distances <= GBSFLACS_CAPTURE_RADIUS + EPS)[0]
        if len(in_range) < MIN_CAPTURE_AGENTS:
            return False
        gap = self._maximum_capture_angular_gap(in_range, target_position)
        return bool(gap <= MAX_CAPTURE_ANGULAR_GAP + EPS)

    def _update_gbsflacs_capture_state(self) -> bool:
        """Commit a successful capture exactly once and stop the target."""
        if not self.threats:
            return False
        task = self.threats[0]
        if task.state == "captured" or not self._capture_condition_met():
            return False

        task.state = "captured"
        task.current_speed_limit = 0.0
        task.orbit_angular_speed_current = 0.0
        task.orbit_segment_remaining = 0.0
        task.position[2] = 0.0
        self.gbsflacs_active = False
        self.capture_success = True
        self.permanently_captured.add(0)
        self.phase = "GBSFLACS 三维围捕成功"
        self.last_message = (
            f"GBSFLACS 围捕成功：目标 T{task.threat_id} 已被 "
            f"{MIN_CAPTURE_AGENTS} 个以上平台完成三维几何包围"
        )
        print(self.last_message)
        self._sync_gbsflacs_environment()
        return True

    def start_gbsflacs_capture(self) -> bool:
        """Validate J-key preconditions and enter the GBSFLACS capture phase."""
        if not self.threats:
            self.last_message = "当前没有敌方目标，请先按 N 键随机生成"
            return False
        if self.gbsflacs_active:
            self.last_message = "GBSFLACS 三维围捕正在进行，请勿重复启动"
            return False
        task = self.threats[0]
        if task.state == "captured":
            self.last_message = "该目标已经由 GBSFLACS 围捕成功"
            return False
        if task.state not in DETECTED_STATES:
            self.last_message = "目标尚未进入感知范围，暂不能启动 GBSFLACS 围捕"
            return False
        self.gbsflacs_active = True
        self.capture_success = False
        for platform in self.platforms:
            platform.role = "capture"
            platform.assigned_threat_id = task.threat_id
        self._sync_gbsflacs_environment()
        self.phase = "GBSFLACS 三维围捕中"
        self.last_message = "GBSFLACS 三维围捕已启动"
        return True

    def _spawn_due_threats(self) -> None:
        for task in self.threats:
            if task.state == "waiting" and self.frame >= task.spawn_frame:
                task.position = point3(
                    horizontal(self.own_position)
                    + task.spawn_radius
                    * np.array([math.cos(task.spawn_angle), math.sin(task.spawn_angle)], dtype=float),
                    0.0,
                )
                task.state = "approaching"
                task.current_speed_limit = self.enemy_approach_speed
                self.last_message = f"敌方目标 T{task.threat_id} 已从随机方向出现"

    def _move_point_toward(
        self, current: np.ndarray, desired: np.ndarray, speed_limit: float
    ) -> np.ndarray:
        current_arr = np.asarray(current, dtype=float)
        desired_arr = np.asarray(desired, dtype=float)
        delta = horizontal(desired_arr) - horizontal(current_arr)
        distance = float(np.linalg.norm(delta))
        max_step = max(0.0, float(speed_limit)) * self.dt
        result = current_arr.copy()
        if distance <= max_step + EPS:
            result[:2] = horizontal(desired_arr)
        else:
            result[:2] = horizontal(current_arr) + delta * (max_step / (distance + EPS))
        result[2] = current_arr[2] if current_arr.shape[0] >= 3 else 0.0
        return result

    def _move_enemy(self, task: ThreatTask) -> None:
        if task.state == "waiting":
            task.current_speed_limit = 0.0
            task.position[2] = 0.0
            return
        if task.state == "captured":
            task.current_speed_limit = 0.0
            task.position[2] = 0.0
            return
        relative = horizontal(task.position) - horizontal(self.own_position)
        direction = normalize(relative, np.array([1.0, 0.0]))
        if task.state == "approaching":
            task.current_speed_limit = self.enemy_approach_speed
            desired = with_altitude(self.own_position, 0.0)
            task.position = self._move_point_toward(task.position, desired, task.current_speed_limit)
            task.position = self._clip_position_to_world(task.position, margin=0.2)
            task.position[2] = 0.0
            return
        if task.state in {"detected", "forming"}:
            task.current_speed_limit = self.enemy_forming_speed
            target_radius = self._controlled_track_radius(task)
            desired = point3(horizontal(self.own_position) + direction * target_radius, 0.0)
            task.position = self._move_point_toward(task.position, desired, task.current_speed_limit)
            task.position = self._clip_position_to_world(task.position, margin=0.2)
            task.position[2] = 0.0
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
            task.orbit_segment_remaining = max(0.0, task.orbit_segment_remaining - angular_step)
            desired = point3(
                horizontal(self.own_position)
                + task.controlled_radius
                * np.array([math.cos(task.controlled_angle), math.sin(task.controlled_angle)], dtype=float),
                0.0,
            )
            task.position = self._move_point_toward(task.position, desired, task.current_speed_limit)
            task.position = self._clip_position_to_world(task.position, margin=0.2)
            task.position[2] = 0.0
            if task.orbit_segment_remaining <= EPS:
                self._start_random_orbit_segment(task)
            return

    def _detect_new_threats(self) -> bool:
        changed = False
        for task in self.threats:
            if task.state != "approaching":
                continue
            distance = horizontal_distance(task.position, self.own_position)
            if distance <= self.sensor_radius + EPS:
                task.state = "detected"
                task.detected_frame = self.frame
                changed = True
                self.last_message = f"感知到敌方目标 T{task.threat_id}，触发守卫机制"
        return changed

    def _threat_geometry(self, task: ThreatTask) -> Tuple[np.ndarray, np.ndarray, float]:
        delta = horizontal(task.position) - horizontal(self.own_position)
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
        scores = []
        for index, platform in enumerate(self.platforms):
            costs = [
                horizontal_distance(platform.position, task.blocker_point)
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
            point3(
                horizontal(self.own_position)
                + radius * (math.cos(phi) * threat_dir + math.sin(phi) * lateral),
                0.0,
            )
            for phi in angles
        ]

    def support_goals(
        self, task_or_id: Union[ThreatTask, int], count: Optional[int] = None
    ) -> List[np.ndarray]:
        """Return a rear support arc on the sea plane that rotates with the threat."""
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
        angles: Iterable[float]
        if support_count == 1:
            angles = [0.0]
        else:
            angles = np.linspace(-self.support_arc_half_angle, self.support_arc_half_angle, support_count)
        return [
            point3(
                horizontal(self.own_position)
                + self.support_guard_radius
                * (math.cos(phi) * center_dir + math.sin(phi) * lateral),
                0.0,
            )
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
                platform = self.platforms[index]
                result[index] = point3(horizontal(self.own_position) + offsets[slot], platform.altitude)
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
            goal = point3(horizontal(self.own_position) + offset, 0.0)
            for col, index in enumerate(candidates):
                cost[row, col] = horizontal_distance(self.platforms[index].position, goal)
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
        task.core_dispatch_initial_distance = horizontal_distance(core.position, task.blocker_point)
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
                core_cost[row, col] = horizontal_distance(platform.position, task.blocker_point) / (
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
                    wing_cost[row, col] = horizontal_distance(self.platforms[index].position, goal)
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
                        support_cost[row, col] = horizontal_distance(self.platforms[index].position, goal)
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
        """连续缩短我方目标水平航步，避免移动中心主动穿入任一平台。"""
        start_xy = horizontal(start)
        end_xy = horizontal(end)
        direction = end_xy - start_xy
        a = float(np.dot(direction, direction))
        if a <= EPS:
            return point3(start_xy, 0.0)
        allowed = 1.0
        radius = self.own_target_avoid_radius
        for platform in self.platforms:
            relative = start_xy - horizontal(platform.position)
            start_distance = float(np.linalg.norm(relative))
            if start_distance < radius - 1e-9:
                allowed = 0.0
                continue
            closest_parameter = float(np.clip(-np.dot(relative, direction) / a, 0.0, 1.0))
            closest = relative + closest_parameter * direction
            if float(np.linalg.norm(closest)) >= radius - 1e-10:
                continue
            b = 2.0 * float(np.dot(relative, direction))
            c = float(np.dot(relative, relative) - radius**2)
            discriminant = max(0.0, b * b - 4.0 * a * c)
            root = math.sqrt(discriminant)
            roots = sorted(
                value
                for value in ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))
                if 0.0 <= value <= 1.0
            )
            if roots:
                allowed = min(allowed, max(0.0, roots[0] - 1e-8))
        return point3(start_xy + allowed * direction, 0.0)

    def _clip_position_to_world(self, position: np.ndarray, margin: float = 0.0) -> np.ndarray:
        result = np.asarray(position, dtype=float).copy()
        result[0] = float(np.clip(result[0], self.world_x_min + margin, self.world_x_max - margin))
        result[1] = float(np.clip(result[1], self.world_y_min + margin, self.world_y_max - margin))
        if result.shape[0] < 3:
            result = point3(result, 0.0)
        return result

    def _move_own_target(self) -> None:
        if not self.detected_threats:
            velocity = normalize(self.forward) * self.cruise_speed
            self.avoid_direction = np.zeros(2, dtype=float)
        else:
            self.avoid_direction = self._resolve_avoid_direction()
            desired_velocity = normalize(self.forward) * self.forward_shift + self.avoid_direction * self.avoid_distance
            velocity = self.own_gain * desired_velocity
        speed = float(np.linalg.norm(velocity))
        if speed > self.own_max_speed:
            velocity *= self.own_max_speed / (speed + EPS)
        proposed = point3(horizontal(self.own_position) + velocity * self.dt, 0.0)
        self.own_position = self._truncate_own_step_for_platforms(self.own_position, proposed)
        self.own_position = self._clip_position_to_world(self.own_position, margin=0.8)
        self.own_position[2] = 0.0
        self.own_goal = point3(horizontal(self.own_position) + velocity, 0.0)

    def _normal_ring_goals(self) -> Dict[int, np.ndarray]:
        count = len(self.platforms)
        result: Dict[int, np.ndarray] = {}
        for index, phi in enumerate(np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)):
            platform = self.platforms[index]
            result[index] = point3(
                horizontal(self.own_position)
                + self.ring_radius * np.array([math.cos(phi), math.sin(phi)], dtype=float),
                platform.altitude,
            )
        return result

    def _desired_non_core_goals(self) -> Dict[int, np.ndarray]:
        if not self.detected_threats:
            return self._normal_ring_goals()

        result = self._normal_ring_goals()
        for core_index in self._core_guard_indices():
            result.pop(core_index, None)
        for task in self.detected_threats:
            goals = self.wing_goals(task)
            for index, slot in task.wing_slot_by_index.items():
                if slot < len(goals):
                    result[index] = with_altitude(goals[slot], self.platforms[index].altitude)

        if len(self.detected_threats) == 1 and self.support_guard_indices:
            task = self.detected_threats[0]
            support_goals = self.support_goals(task)
            for index, slot in self._support_slot_by_index.items():
                if slot < len(support_goals):
                    result[index] = with_altitude(support_goals[slot], self.platforms[index].altitude)

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
        current = horizontal(self.platforms[index].position)
        repulsion = np.zeros(2, dtype=float)
        for other_index, other in enumerate(self.platforms):
            if other_index == index:
                continue
            diff = current - horizontal(other.position)
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
        start_xy = horizontal(start)
        end_xy = horizontal(end)
        center = horizontal(self.own_position)
        protected_radius = self.own_target_avoid_radius if radius is None else float(radius)
        segment = end_xy - start_xy
        denominator = float(np.dot(segment, segment))
        if denominator <= EPS:
            return float(np.linalg.norm(start_xy - center)) < protected_radius - EPS
        parameter = float(np.clip(np.dot(center - start_xy, segment) / denominator, 0.0, 1.0))
        closest = start_xy + parameter * segment
        return float(np.linalg.norm(closest - center)) < protected_radius - EPS

    @staticmethod
    def _directed_arc_delta(start_angle: float, end_angle: float, side: int) -> float:
        if side > 0:
            return float((end_angle - start_angle) % (2.0 * math.pi))
        return float((start_angle - end_angle) % (2.0 * math.pi))

    def _bypass_path_length(self, current: np.ndarray, goal: np.ndarray, side: int) -> float:
        center = horizontal(self.own_position)
        radius = self.own_target_route_radius
        p = horizontal(current) - center
        g = horizontal(goal) - center
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
            return np.asarray(end, dtype=float).copy()
        altitude = float(np.asarray(start, dtype=float)[2])
        start_xy = horizontal(start)
        end_xy = horizontal(end)
        direction = end_xy - start_xy
        a = float(np.dot(direction, direction))
        if a <= EPS:
            return point3(start_xy, altitude)
        relative = start_xy - horizontal(self.own_position)
        b = 2.0 * float(np.dot(relative, direction))
        c = float(np.dot(relative, relative) - self.own_target_avoid_radius**2)
        disc = max(0.0, b * b - 4.0 * a * c)
        root = math.sqrt(disc)
        roots = [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
        valid = [value for value in roots if 0.0 <= value <= 1.0]
        if not valid:
            return point3(start_xy, altitude)
        return point3(start_xy + max(0.0, min(valid) - 1e-8) * direction, altitude)

    def _safe_route_waypoint(
        self, index: int, current: np.ndarray, goal: np.ndarray, max_step: float
    ) -> np.ndarray:
        """在水平面绕过安全圆，同时保留平台固定高度。"""
        altitude = self.platforms[index].altitude
        center = horizontal(self.own_position)
        route_radius = self.own_target_route_radius
        current_xy = horizontal(current)
        goal_xy = horizontal(goal)
        current_relative = current_xy - center
        goal_relative = goal_xy - center
        current_distance = float(np.linalg.norm(current_relative))
        goal_distance = float(np.linalg.norm(goal_relative))

        if goal_distance < route_radius:
            goal_relative = normalize(goal_relative, current_relative) * route_radius
            goal_xy = center + goal_relative

        goal3 = point3(goal_xy, altitude)
        if current_distance >= route_radius - 1e-8 and not self._segment_intersects_own_target_circle(current, goal3, route_radius):
            self._own_target_bypass_side.pop(index, None)
            return goal3

        side = self._choose_bypass_side(index, current, goal3)
        radial = normalize(current_relative, -goal_relative)
        tangent = side * rotate90(radial)
        radial_error = current_distance - route_radius
        if radial_error > max(0.25, 1.5 * max_step):
            steering = tangent - 0.85 * radial
        else:
            correction = float(np.clip(-2.5 * radial_error / max(max_step, 1e-6), -1.4, 1.8))
            steering = tangent + correction * radial
        return point3(current_xy + normalize(steering, tangent) * max_step, altitude)

    def _move_platform_safely(
        self, index: int, goal: np.ndarray, *, include_repulsion: bool = False
    ) -> None:
        platform = self.platforms[index]
        current = platform.position.copy()
        goal3 = with_altitude(goal, platform.altitude)
        max_step = platform.max_speed * self.dt
        waypoint = self._safe_route_waypoint(index, current, goal3, max_step)
        velocity = platform.gain * (horizontal(waypoint) - horizontal(current))
        if include_repulsion and index not in self._own_target_bypass_side:
            velocity += self._repulsion_velocity(index)
        speed = float(np.linalg.norm(velocity))
        if speed > platform.max_speed:
            velocity *= platform.max_speed / (speed + EPS)
        proposed = point3(horizontal(current) + velocity * self.dt, platform.altitude)
        proposed = self._truncate_step_before_safety_circle(current, proposed)
        platform.position = self._clip_position_to_world(proposed, margin=0.2)
        platform.position[2] = platform.altitude if platform.kind == "UAV" else 0.0
        platform.goal = goal3.copy()

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
            core_index = task.core_guard_index
            core_goal = with_altitude(task.blocker_point, self.platforms[core_index].altitude)
            self._move_platform_safely(core_index, core_goal)
            core = self.platforms[core_index]
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
        return horizontal_distance(self.platforms[task.core_guard_index].position, task.blocker_point)

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
                error = horizontal_distance(self.platforms[index].position, goals[slot])
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
        task = task_or_id if isinstance(task_or_id, ThreatTask) else self.get_threat(task_or_id)
        distance = horizontal_distance(task.position, self.own_position)
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
        relative = horizontal(task.position) - horizontal(self.own_position)
        task.controlled_radius = self._controlled_track_radius(task)
        task.controlled_angle = math.atan2(relative[1], relative[0])
        task.state = "orbiting"
        task.orbit_segment_count = 0
        task.orbit_direction_changes = 0
        self._start_random_orbit_segment(task)
        direction_label = "逆时针" if task.orbit_direction > 0 else "顺时针"
        self.last_message = f"T1 守卫队形形成，开始随机分段环绕；当前方向：{direction_label}"

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
        if self.capture_success:
            for task in self.threats:
                task.current_speed_limit = 0.0
                task.position[2] = 0.0
            # 捕获判定仍沿用“至少 5 个平台 + 最大角隙”条件；成功后只
            # 继续执行成功帧已经确定的最后一组交替航点，使尚未到位的
            # 平台完成 UAV—USV 交叉队形。目标保持静止，航点到达后平台
            # 自然保持不动，不再运行 GB、CS 或 SFLA 优化。
            final_waypoints = self.gbsflacs_algorithm.desired_waypoints
            for index in range(len(self.platforms)):
                waypoint = final_waypoints.get(index)
                if waypoint is not None:
                    self._move_platform_to_gbsflacs_waypoint(index, waypoint)
            self._move_own_target()
            self._sync_gbsflacs_environment()
            self.phase = "GBSFLACS 三维围捕成功"
            return
        self._spawn_due_threats()

        # 先让敌方目标按当前状态运动，再执行感知判定。
        for task in self.threats:
            self._move_enemy(task)

        if self.gbsflacs_active:
            self._move_own_target()
            self._refresh_all_blockers()
            self._step_gbsflacs_capture()
            if not self._update_gbsflacs_capture_state():
                self.phase = "GBSFLACS 三维围捕中"
            return

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
        self._sync_gbsflacs_environment()

    def status(self) -> Dict[str, object]:
        records = []
        for task in self.threats:
            distance = horizontal_distance(task.position, self.own_position)
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
        capture_in_range = 0
        capture_max_gap_deg = 360.0
        if self.threats:
            target_position = self.threats[0].position
            distances = np.asarray(
                [
                    np.linalg.norm(platform.position - target_position)
                    for platform in self.platforms
                ],
                dtype=float,
            )
            capture_indices = np.where(
                distances <= GBSFLACS_CAPTURE_RADIUS + EPS
            )[0]
            capture_in_range = int(len(capture_indices))
            capture_max_gap_deg = math.degrees(
                self._maximum_capture_angular_gap(capture_indices, target_position)
            )
        uav_altitudes = [
            float(platform.position[2])
            for platform in self.platforms
            if platform.kind == "UAV"
        ]
        return {
            "frame": self.frame,
            "phase": self.phase,
            "message": self.last_message,
            "paused": self.paused,
            "sensor_radius": self.sensor_radius,
            "enemy_count": len(self.threats),
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
            "gbsflacs_active": self.gbsflacs_active,
            "capture_success": self.capture_success,
            "capture_radius": GBSFLACS_CAPTURE_RADIUS,
            "minimum_capture_agents": MIN_CAPTURE_AGENTS,
            "capture_in_range": capture_in_range,
            "capture_max_gap_deg": capture_max_gap_deg,
            "granular_ball_count": len(self.gbsflacs_algorithm.leaf_balls),
            "cs_trigger_count": int(
                self.gbsflacs_algorithm.metrics.get("cs_trigger_count", 0)
            ),
            "sfla_waypoint_steps": int(
                self.gbsflacs_algorithm.metrics.get("sfla_waypoint_steps", 0)
            ),
            "uav_altitude_min_current": min(uav_altitudes, default=0.0),
            "uav_altitude_max_current": max(uav_altitudes, default=0.0),
            "threats": records,
        }
