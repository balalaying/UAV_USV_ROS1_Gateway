import os

os.environ['OMP_NUM_THREADS'] = '1'
os.environ.setdefault('MPLCONFIGDIR', '/tmp/uav_usv_matplotlib')
import matplotlib

# The ROS adapter is headless.  Keeping plotting imports available preserves
# the supplied reference program while avoiding a second GUI event loop.
matplotlib.use('Agg')
# import skfuzzy as fuzz
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
import numpy as np
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt

# from matplotlib.patches import FancyArrowPatch
# from matplotlib.widgets import AxesWidget
# from matplotlib.transforms import Affine2D
# from matplotlib import transforms
# from mpl_toolkits.mplot3d import Axes3D
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import random
# import math
# from sklearn.neighbors import NearestNeighbors
#---------------------------------------------------------------
# from algorithms.gb_sfla import GBSFLAAlgorithm
# from algorithms.standard_sfla import StandardSFLAAlgorithm
# from algorithms.pso import PSOAlgorithm
# from algorithms.ga import GAAlgorithm


# 设置随机种子（任意整数，如 42）
SEED = 42
random.seed(SEED)
np.random.seed(SEED)

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'KaiTi']
plt.rcParams['axes.unicode_minus'] = False

# ==================== 1. 全局参数配置 (升级 + SFLA/CS参数) ====================
ARENA_SIZE_XY = 3000
ARENA_SIZE_Z = ARENA_SIZE_XY
OBSTACLES = []

# ---智能体初始化方式控制 ---
INIT_RANDOMSTATE = 0  # 1: 随机分布, 0: 均匀分布
plt1 = 1
# --- 智能体与目标参数 ---
CAPTURE_RADIUS = 100
UAV_COUNT = 3
USV_COUNT = 3
TARGET_COUNT = 1
TARGET_IS_STATIC = 0
TARGET_RUN_NUM = 300
# MIN_CAPTURE_AGENTS = int(np.sqrt(UAV_COUNT + USV_COUNT)-1)
MIN_CAPTURE_AGENTS = (UAV_COUNT + USV_COUNT) // TARGET_COUNT -1
print("UAV_COUNT,USV_COUNT,TARGET_COUNT,MIN_CAPTURE_AGENTS", UAV_COUNT, USV_COUNT, TARGET_COUNT, MIN_CAPTURE_AGENTS)
# --- 粒球计算参数 ---
QTH = 0.6
RADIUS_CTRL = 1.0
ALPHA = 0.4
BETA = 0.4
GAMMA = 0.2

# --- 速度与动力学 ---
V_MAX_UAV = ARENA_SIZE_XY // 20
V_MAX_USV = int(V_MAX_UAV * 0.7)
TARGET_SPEED = V_MAX_UAV // 2
print("V_MAX_UAV,V_MAX_USV,TARGET_SPEED",V_MAX_UAV,V_MAX_USV,TARGET_SPEED)
V_MIN = 0
OMEGA_MAX = np.pi / 4

# --- 3D专用变量 ---
USV_Z = 0
TARGET_Z = 0
UAV_Z_MIN, UAV_Z_MAX = 0, ARENA_SIZE_Z

# ==================== 新增：SFLA 与 CS 算法参数 ====================
# SFLA 参数 (用于粒球内部微调)
SFLA_MEME_COUNT = 2  # 模因组数量
SFLA_MAX_STEP = 50  # 青蛙最大跳跃步长 (对应无人机速度限制)
SFLA_ITER_LOCAL = 5  # 局部迭代次数

# CS 参数 (用于粒球间重组)
CS_PA = 0.25  # 发现概率 (控制探索强度)
CS_BETA = 1.5  # 莱维飞行指数

# ==================== 新增：算法对比框架 ====================
import math
from abc import ABC, abstractmethod


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

    def __init__(self, env, name="GB-SFLA-CS"):
        super().__init__(env, name)
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
            "distance": 4.0,
            "balance": 3.0,
            "reconfiguration": 2.0,
            "energy": 1.0,
            "group": 2.0,
        }

        self.rng = np.random.default_rng(SEED + 701)
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
        """预测目标在本步逃逸后的参考位置，供分配和航点规划使用。"""
        target_id = int(target_id)
        if target_id in self._target_reference_cache:
            return self._target_reference_cache[target_id].copy()
        target = self.env.targets[target_id]
        position = np.asarray(target[:3], dtype=float).copy()
        if TARGET_IS_STATIC != 0:
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

    def _capacity_aware_assignment(self, active_target_ids, balls=None):
        active_target_ids = [int(target_id) for target_id in active_target_ids]
        agent_map = self._active_agent_map()
        active_ids = sorted(agent_map)
        if not active_ids or not active_target_ids:
            return {}

        balls = balls if balls is not None else self.leaf_balls
        agent_to_ball = self._agent_to_ball(balls)
        previous = self.last_assignments or getattr(self.env, "previous_assignments", {}) or {}
        requirements = self._capacity_requirements(active_ids, active_target_ids)

        assignments = {}
        loads = {target_id: 0 for target_id in active_target_ids}
        unassigned = set(active_ids)
        ball_targets = {}

        # 优先处理“最近可用智能体也较远”的困难目标，避免被容易目标抢占资源。
        target_order = sorted(
            active_target_ids,
            key=lambda target_id: min(self._arrival_cost(agent_id, target_id) for agent_id in active_ids),
            reverse=True,
        )

        for target_id in target_order:
            for _ in range(requirements[target_id]):
                if not unassigned:
                    break
                best_agent = None
                best_cost = float("inf")
                for agent_id in unassigned:
                    cost = self._arrival_cost(agent_id, target_id)
                    if agent_id in previous:
                        cost += -0.35 if int(previous[agent_id]) == target_id else 0.35
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
        for agent_id in sorted(unassigned):
            best_target = None
            best_cost = float("inf")
            ball_id = agent_to_ball.get(agent_id)
            for target_id in active_target_ids:
                cost = self._arrival_cost(agent_id, target_id)
                cost += 0.30 * (loads[target_id] / mean_requirement)
                if agent_id in previous and int(previous[agent_id]) != target_id:
                    cost += 0.40
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

        requirements = self._capacity_requirements(active_ids, active_target_ids)
        loads = {target_id: 0 for target_id in active_target_ids}
        for target_id in repaired.values():
            loads[target_id] += 1

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
        return repaired

    def _composite_cost(self, assignments, balls=None):
        active_target_ids = self._active_target_ids()
        active_ids = sorted(self._active_agent_map())
        if not active_target_ids:
            return 0.0
        if set(assignments) != set(active_ids):
            return float("inf")
        if any(target_id not in active_target_ids for target_id in assignments.values()):
            return float("inf")

        loads = {target_id: 0 for target_id in active_target_ids}
        arrival_times = []
        energy_terms = []
        for agent_id, target_id in assignments.items():
            loads[target_id] += 1
            agent = self._active_agent_map()[agent_id]
            distance = float(np.linalg.norm(np.asarray(agent[:3]) - self._target_reference_position(target_id)))
            arrival_times.append(distance / max(self._max_speed(agent_id), 1e-9))
            energy_terms.append((distance / max(ARENA_SIZE_XY, 1)) ** 2)

        uncovered = sum(max(0, MIN_CAPTURE_AGENTS - loads[target_id]) for target_id in active_target_ids)
        distance_norm = float(np.mean(arrival_times) / max(ARENA_SIZE_XY / V_MAX_UAV, 1e-9)) if arrival_times else 0.0
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

        return float(
            self.weights["uncovered"] * uncovered
            + self.weights["distance"] * distance_norm
            + self.weights["balance"] * balance
            + self.weights["reconfiguration"] * reconfiguration
            + self.weights["energy"] * energy
            + self.weights["group"] * group_loss
        )

    # ------------------------------------------------------------------
    # CS：完整分配/结构候选，仅改进时接受
    # ------------------------------------------------------------------
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
        incumbent_cost = self._composite_cost(incumbent_assignment, incumbent_balls)

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
            proposal_cost = self._composite_cost(proposal_assignment, proposal)
            accepted, chosen_assignment, chosen_cost = self._accept_if_improved(
                incumbent_assignment,
                incumbent_cost,
                proposal_assignment,
                proposal_cost,
            )
            if accepted:
                incumbent_balls = proposal
                incumbent_assignment = chosen_assignment
                incumbent_cost = chosen_cost
                self.metrics["recluster_count"] += 1

        self.leaf_balls = incumbent_balls
        self._refresh_tree()
        return incumbent_assignment, incumbent_cost

    def _try_cs_assignment_improvement(self, assignment, active_target_ids):
        incumbent = dict(assignment)
        incumbent_cost = self._composite_cost(incumbent, self.leaf_balls)
        nests = [dict(incumbent)]
        for _ in range(self.cs_nests - 1):
            nests.append(self._mutate_assignment(incumbent, active_target_ids, self.leaf_balls))

        for _ in range(self.cs_iterations):
            updated = []
            for nest in nests:
                candidate = self._mutate_assignment(nest, active_target_ids, self.leaf_balls)
                nest_cost = self._composite_cost(nest, self.leaf_balls)
                candidate_cost = self._composite_cost(candidate, self.leaf_balls)
                _, chosen, _ = self._accept_if_improved(nest, nest_cost, candidate, candidate_cost)
                updated.append(chosen)
            nests = updated

            fitness_costs = np.asarray([self._composite_cost(nest, self.leaf_balls) for nest in nests])
            abandon_count = max(1, int(np.ceil(self.cs_pa * len(nests))))
            for index in np.argsort(fitness_costs)[-abandon_count:]:
                if self.rng.random() < self.cs_pa:
                    nests[int(index)] = self._mutate_assignment(incumbent, active_target_ids, self.leaf_balls)

            best_nest = min(nests, key=lambda nest: self._composite_cost(nest, self.leaf_balls))
            best_cost = self._composite_cost(best_nest, self.leaf_balls)
            _, incumbent, incumbent_cost = self._accept_if_improved(
                incumbent,
                incumbent_cost,
                best_nest,
                best_cost,
            )
        return incumbent, incumbent_cost

    # ------------------------------------------------------------------
    # SFLA：青蛙为局部围捕航点方案
    # ------------------------------------------------------------------
    def _plan_bounds_clip(self, params):
        clipped = np.asarray(params, dtype=float).copy()
        clipped[0] = clipped[0] % (2 * np.pi)
        clipped[1] = np.clip(clipped[1], 0.45, 0.72)
        clipped[2] = np.clip(clipped[2], 0.25, 0.75)
        return clipped

    def _plan_to_waypoints(self, agent_ids, target_id, params):
        agent_ids = [int(agent_id) for agent_id in agent_ids]
        if not agent_ids:
            return {}
        params = self._plan_bounds_clip(params)
        target_pos = self._target_reference_position(target_id)
        ring_radius = float(params[1] * CAPTURE_RADIUS)
        angles = params[0] + 2 * np.pi * np.arange(len(agent_ids)) / len(agent_ids)
        agent_map = self._active_agent_map()

        slots_by_agent = {}
        cost_matrix = np.zeros((len(agent_ids), len(agent_ids)), dtype=float)
        for row, agent_id in enumerate(agent_ids):
            agent = agent_map[agent_id]
            agent_type = int(agent[6])
            max_vertical = np.sqrt(max(CAPTURE_RADIUS ** 2 - ring_radius ** 2, 0.0))
            z_offset = 0.0 if agent_type == 1 else float(params[2] * 0.80 * max_vertical)
            slots = np.column_stack([
                target_pos[0] + ring_radius * np.cos(angles),
                target_pos[1] + ring_radius * np.sin(angles),
                np.full(len(angles), USV_Z if agent_type == 1 else target_pos[2] + z_offset),
            ])
            slots_by_agent[agent_id] = slots
            cost_matrix[row] = np.linalg.norm(slots - np.asarray(agent[:3]), axis=1) / max(self._max_speed(agent_id), 1e-9)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        waypoints = {}
        for row, col in zip(row_ind, col_ind):
            agent_id = agent_ids[int(row)]
            waypoint = np.asarray(slots_by_agent[agent_id][int(col)], dtype=float)
            # 最终物理安全检查：航点必须位于目标三维捕获球内。
            delta = waypoint - target_pos
            distance = float(np.linalg.norm(delta))
            if distance > CAPTURE_RADIUS:
                waypoint = target_pos + delta * ((CAPTURE_RADIUS * 0.98) / max(distance, 1e-12))
            if int(agent_map[agent_id][6]) == 1:
                waypoint[2] = USV_Z
            waypoints[agent_id] = waypoint
        return waypoints

    def _local_plan_cost(self, agent_ids, target_id, params):
        waypoints = self._plan_to_waypoints(agent_ids, target_id, params)
        if not waypoints:
            return float("inf")
        agent_map = self._active_agent_map()
        target_pos = self._target_reference_position(target_id)
        arrivals = []
        angles = []
        radii = []
        positions = []
        for agent_id, waypoint in waypoints.items():
            arrivals.append(np.linalg.norm(waypoint - np.asarray(agent_map[agent_id][:3])) / max(self._max_speed(agent_id), 1e-9))
            vector = waypoint[:2] - target_pos[:2]
            angles.append(float(np.mod(np.arctan2(vector[1], vector[0]), 2 * np.pi)))
            radii.append(float(np.linalg.norm(waypoint - target_pos)))
            positions.append(waypoint)

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
        return float(4.0 * arrival_norm + 2.0 * angular_error + radius_error + 2.0 * collision_penalty)

    def _sfla_target_waypoints(self, agent_ids, target_id):
        if not agent_ids:
            return {}
        population = []
        for index in range(self.sfla_population_size):
            if index == 0:
                params = np.array([0.0, 0.55, 0.45], dtype=float)
            else:
                params = np.array([
                    self.rng.uniform(0, 2 * np.pi),
                    self.rng.uniform(0.45, 0.72),
                    self.rng.uniform(0.25, 0.75),
                ])
            population.append(self._plan_bounds_clip(params))

        for _ in range(self.sfla_local_iterations):
            costs = np.asarray([self._local_plan_cost(agent_ids, target_id, params) for params in population])
            order = np.argsort(costs)
            global_best = population[int(order[0])].copy()
            memeplexes = [order[index::self.sfla_memeplex_count].tolist() for index in range(self.sfla_memeplex_count)]
            for memeplex in memeplexes:
                if len(memeplex) < 2:
                    continue
                local_best_index = min(memeplex, key=lambda index: costs[index])
                worst_index = max(memeplex, key=lambda index: costs[index])
                worst = population[worst_index]
                local_best = population[local_best_index]

                candidate = self._plan_bounds_clip(worst + self.rng.random(3) * (local_best - worst))
                candidate_cost = self._local_plan_cost(agent_ids, target_id, candidate)
                if candidate_cost >= costs[worst_index]:
                    candidate = self._plan_bounds_clip(worst + self.rng.random(3) * (global_best - worst))
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

        best = min(population, key=lambda params: self._local_plan_cost(agent_ids, target_id, params))
        return self._plan_to_waypoints(agent_ids, target_id, best)

    def _build_sfla_waypoints(self, assignments):
        grouped = {}
        for agent_id, target_id in assignments.items():
            grouped.setdefault(int(target_id), []).append(int(agent_id))
        waypoints = {}
        for target_id, agent_ids in grouped.items():
            waypoints.update(self._sfla_target_waypoints(agent_ids, target_id))
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

        # 1. GB：真实状态同步与自适应结构优化。
        self._sync_real_balls()
        self._adapt_structure(len(active_target_ids))

        # 2. 容量优先初始分配。
        assignment = self._capacity_aware_assignment(active_target_ids, self.leaf_balls)

        # 3. CS：先搜索结构候选，再搜索完整分配候选；均采用改进接受准则。
        assignment, _ = self._try_cs_structure_improvement(assignment, active_target_ids)
        assignment, objective = self._try_cs_assignment_improvement(assignment, active_target_ids)
        assignment = self._repair_assignment_capacity(assignment, active_target_ids)
        objective = self._composite_cost(assignment, self.leaf_balls)

        # 4. SFLA：为最终分配生成真实可执行的围捕航点。
        self.desired_waypoints = self._build_sfla_waypoints(assignment)
        self.last_assignments = dict(assignment)
        self.last_objective = float(objective)
        self._refresh_tree()

        print(
            f"   Assignments (effective GB-SFLA-CS): {assignment}; "
            f"balls={len(self.leaf_balls)}, objective={self.last_objective:.4f}"
        )
        return assignment



# ==================== 2. 图片加载工具函数 ====================
def load_image(path, size=0.15):
    if not os.path.exists(path):
        print(f"⚠️ 警告: 找不到图片 {path}")
        return None
    try:
        img = plt.imread(path)
        return OffsetImage(img, zoom=size)
    except Exception as e:
        print(f"⚠️ 警告: 无法读取图片 {path}: {e}")
        return None


# ==================== 3. 数据结构：粒球节点 ====================
class GranularBallNode:
    def __init__(self, ball_id, points, parent=None):
        self.ball_id = ball_id
        self.points = np.array(points)
        self.agent_ids = []  #  新增：存储智能体ID
        self.parent = parent
        self.children = []
        self.children_quality = []  # ✅ 新增：存储子节点的质量值
        self.is_empty = False  # ✅ 新增：标记是否为空节点

        # 物理属性
        self.center = np.mean(points, axis=0) if len(points) > 0 else np.zeros(3)
        self.radius = np.max(cdist([self.center], points)) if len(points) > 0 else 0

        # 质量属性 (用于CS算法判断)
        self.quality = 0.0
        self.is_leaf = True

    def split(self, next_id_func, env_instance=None):
        n_points = len(self.points)
        if n_points < 4:
            return False

        kmeans = KMeans(n_clusters=2, random_state=42, n_init=10)
        labels = kmeans.fit_predict(self.points)

        unique_labels, counts = np.unique(labels, return_counts=True)
        if np.any(counts < 2):
            return False

        new_children = []
        for i in range(2):
            child_points = self.points[labels == i]
            if len(child_points) < 2:
                return False
            child_node = GranularBallNode(ball_id=next_id_func(), points=child_points, parent=self)
            new_children.append(child_node)

        # ✅ 关键修复：检查子节点是否有智能体
        children_with_agents = []
        for child in new_children:
            # 检查子节点是否有智能体
            if hasattr(child, 'agent_ids') and child.agent_ids:
                children_with_agents.append(child)
            elif len(child.points) > 0:
                # 如果子节点有点但没有agent_ids，给它分配空的agent_ids
                child.agent_ids = []
                children_with_agents.append(child)

        # ✅ 如果只有一个子节点有智能体，父节点直接等于这个子节点
        if len(children_with_agents) == 1:
            only_child = children_with_agents[0]
            # 将父节点的属性设置为子节点的属性
            self.points = only_child.points
            self.agent_ids = only_child.agent_ids
            self.center = only_child.center
            self.radius = only_child.radius
            self.children = []  # 清空子节点
            self.is_leaf = True  # 变回叶子节点
            self.children_quality = []  # 清空子节点质量
            print(f"🔄 Ball-{self.ball_id} 吸收唯一有智能体的子节点")
            return True

        # ✅ 如果两个子节点都没有智能体，不进行分裂
        if len(children_with_agents) == 0:
            print(f"⚠️ Ball-{self.ball_id} 分裂失败：两个子节点都没有智能体")
            return False

        # ✅ 正常情况：两个子节点都有智能体
        self.children = new_children
        self.is_leaf = False

        # ✅ 存储子节点的质量值
        self.children_quality = []
        if env_instance is not None:
            for child in self.children:
                # 计算子节点的质量
                child.quality = env_instance._calculate_cluster_quality(child)
                self.children_quality.append(child.quality)

        return True


# ==================== 4. 环境主类 (SwarmEnv3D) ====================
class SwarmEnv3D:
    def __init__(self):
        self.guarding_agents = {}  # 键：目标ID → 值：守卫智能体ID集合
        self.reset()
        # -----------------------------------------------------------------------------
        self.algorithm = None
        self.algorithm_name = "GB-SFLA-CS"
        self.set_algorithm(self.algorithm_name)

    def set_algorithm(self, algorithm_name):
        self.algorithm_name = algorithm_name
        if algorithm_name == "GB-SFLA-CS":
            self.algorithm = GBSFLACSAlgorithm(self, "GB-SFLA-CS")
        else:
            raise ValueError(f"Unknown algorithm: {algorithm_name}")

    def step_with_algorithm(self):
        """使用当前算法执行一步"""
        if self.current_algorithm is None:
            self.set_algorithm(self.algorithm_name)

        # 执行算法获取分配方案
        assignments = self.current_algorithm.step()

        # 原有的移动逻辑...
        # 使用assignments来移动智能体

        return self.current_algorithm.metrics

    # -----------------------------------------------------------------------------

    def _calculate_repulsion_force(self, agent_pos, all_agents_pos, obstacles_pos=None):
        repulsion_force = np.zeros(3)
        for other_pos in all_agents_pos:
            if np.linalg.norm(agent_pos - other_pos) < 1e-5: continue
            dist_vec = agent_pos - other_pos
            dist = np.linalg.norm(dist_vec)
            if dist < self.repulsion_range:
                if dist == 0: dist = 0.01
                direction = dist_vec / dist
                force_magnitude = self.repulsion_gain * (1.0 / dist)
                repulsion_force += direction * force_magnitude
        return repulsion_force

    def reset(self):
        self.time_step = 0
        self.permanently_captured = set()
        self.total_travel_distance = 0.0
        self.target_load_balance_history = []
        self.reconfiguration_count = 0
        self.previous_assignments = {}
        self.guarding_agents = {}
        self.target_captors = {}
        self.repulsion_range = 50
        self.repulsion_gain  = 100
        random.seed(SEED)
        np.random.seed(SEED)
        # --- 初始化智能体 ---
        self.agents = []

        # 根据 INIT_RANDOMSTATE 决定初始化方式
        if INIT_RANDOMSTATE == 1:
            # ===== 原有随机分布方式 =====
            print("🎲 使用随机分布初始化智能体")

            # UAV 随机分布
            for i in range(UAV_COUNT):
                pos_x = random.uniform(ARENA_SIZE_XY * 0.1, ARENA_SIZE_XY * 0.9)
                pos_y = random.uniform(ARENA_SIZE_XY * 0.1, ARENA_SIZE_XY * 0.9)
                pos_z = random.uniform(UAV_Z_MIN, UAV_Z_MAX * 0.5)
                vel = random.uniform(V_MIN, V_MAX_UAV)
                theta = random.uniform(0, 2 * np.pi)
                phi = random.uniform(-np.pi / 2, np.pi / 2)
                self.agents.append([pos_x, pos_y, pos_z, vel, theta, phi, 0, i + 1000])

            # USV 随机分布
            for i in range(USV_COUNT):
                pos_x = random.uniform(ARENA_SIZE_XY * 0.05, ARENA_SIZE_XY * 0.95)
                pos_y = random.uniform(ARENA_SIZE_XY * 0.05, ARENA_SIZE_XY * 0.95)
                pos_z = USV_Z
                vel = random.uniform(V_MIN, V_MAX_USV)
                theta = random.uniform(0, 2 * np.pi)
                phi = 0
                self.agents.append([pos_x, pos_y, pos_z, vel, theta, phi, 1, i + 2000])

        else:
            # ===== 新增：均匀分布方式 =====
            print("📐 使用均匀分布初始化智能体")

            # 计算网格布局
            total_agents = UAV_COUNT + USV_COUNT

            # 计算网格行列数（尽量接近正方形）
            grid_cols = int(np.ceil(np.sqrt(total_agents)))
            grid_rows = int(np.ceil(total_agents / grid_cols))

            # 计算每个网格的大小
            cell_width = ARENA_SIZE_XY / grid_cols
            cell_height = ARENA_SIZE_XY / grid_rows

            # 添加小随机扰动避免完全对齐（可选）
            jitter = 0.1  # 扰动幅度（相对于单元格大小）

            # 初始化所有智能体位置
            agent_positions = []
            agent_id = 0

            # 生成网格点
            for row in range(grid_rows):
                for col in range(grid_cols):
                    if agent_id >= total_agents:
                        break

                    # 计算基础位置（网格中心）
                    base_x = col * cell_width + cell_width / 2
                    base_y = row * cell_height + cell_height / 2

                    # 添加小随机扰动
                    jitter_x = random.uniform(-cell_width * jitter, cell_width * jitter)
                    jitter_y = random.uniform(-cell_height * jitter, cell_height * jitter)

                    pos_x = base_x + jitter_x
                    pos_y = base_y + jitter_y

                    # 确保在边界内
                    pos_x = max(cell_width * 0.1, min(pos_x, ARENA_SIZE_XY - cell_width * 0.1))
                    pos_y = max(cell_height * 0.1, min(pos_y, ARENA_SIZE_XY - cell_height * 0.1))

                    agent_positions.append([pos_x, pos_y, agent_id])
                    agent_id += 1

            # 分配 UAV（前 UAV_COUNT 个位置给 UAV）
            # for i in range(UAV_COUNT + USV_COUNT):
            for i in range(UAV_COUNT):
                if i < len(agent_positions):
                    pos_x, pos_y, _ = agent_positions[i]
                    # UAV 在高度上均匀分布
                    pos_z = random.uniform(UAV_Z_MIN, UAV_Z_MAX)
                    vel = random.uniform(V_MIN, V_MAX_UAV)
                    theta = random.uniform(0, 2 * np.pi)
                    phi = random.uniform(-np.pi / 2, np.pi / 2)
                    self.agents.append([pos_x, pos_y, pos_z, vel, theta, phi, 0, i + 1000])

            # 分配 USV（剩余位置给 USV）
            usv_start_id = UAV_COUNT
            for i in range(USV_COUNT):
                if usv_start_id + i < len(agent_positions):
                    pos_x, pos_y, _ = agent_positions[usv_start_id + i]
                    pos_z = USV_Z
                    vel = random.uniform(V_MIN, V_MAX_USV)
                    theta = random.uniform(0, 2 * np.pi)
                    phi = 0
                    self.agents.append([pos_x, pos_y, pos_z, vel, theta, phi, 1, i + 2000])

        self.agents = np.array(self.agents, dtype=float)
        self._validate_unique_agent_ids()
        self.agent_index_by_id = {
            int(agent[7]): index
            for index, agent in enumerate(self.agents)
        }
        print("self.agents",self.agents)
        # --- 初始化目标（保持不变）---
        self.targets = []
        tar_num = 0
        for i in range(TARGET_COUNT):
            if tar_num:
                pos_x = random.uniform(ARENA_SIZE_XY * -0.2, ARENA_SIZE_XY * 1.25)
                pos_y = random.uniform(ARENA_SIZE_XY * -0.2, ARENA_SIZE_XY * 1.25)
            else:
                pos_x = random.uniform(ARENA_SIZE_XY * 0.2, ARENA_SIZE_XY * 0.8)
                pos_y = random.uniform(ARENA_SIZE_XY * 0.2, ARENA_SIZE_XY * 0.8)
            pos_z = TARGET_Z
            vel = random.uniform(TARGET_SPEED * 0.5, TARGET_SPEED)
            theta = random.uniform(0, 2 * np.pi)
            self.targets.append([pos_x, pos_y, pos_z, vel, theta, i + 3000])
        self.targets = np.array(self.targets)
        print('targets',self.targets)
        # --- 初始化粒球树（保持不变）---
        self.all_balls = {}
        self.next_ball_id = 0
        root_ball = GranularBallNode(self.next_ball_id, [], parent=None)
        self.ball_tree = root_ball
        self.all_balls[self.next_ball_id] = root_ball
        self.next_ball_id += 1


    def _get_guarding_agent_ids(self):
        """Return agents already locked into post-capture guard duties."""
        guarding_ids = set()
        for guard_ids in self.guarding_agents.values():
            guarding_ids.update(int(agent_id) for agent_id in guard_ids)
        return guarding_ids

    def _get_active_agents(self):
        """Return non-guarding agents used by GB grouping and task allocation."""
        guarding_ids = self._get_guarding_agent_ids()
        return [
            agent for agent in self.agents
            if int(agent[7]) not in guarding_ids
        ]

    def _validate_unique_agent_ids(self):
        """Fail fast if UAV/USV IDs overlap, because assignments use IDs as dict keys."""
        ids = [int(agent[7]) for agent in self.agents]
        if len(ids) != len(set(ids)):
            duplicates = sorted({agent_id for agent_id in ids if ids.count(agent_id) > 1})
            raise ValueError(f"Duplicate agent IDs detected: {duplicates}")

    def _sanitize_assignments(self, assignments):
        """Ensure every non-guarding agent is directed to an active target."""
        active_target_ids = [
            target_id for target_id in range(len(self.targets))
            if target_id not in self.permanently_captured
        ]
        if not active_target_ids:
            return {}

        sanitized = dict(assignments)
        guarding_ids = self._get_guarding_agent_ids()
        for agent in self.agents:
            agent_id = int(agent[7])
            if agent_id in guarding_ids:
                continue

            target_id = sanitized.get(agent_id)
            if target_id not in active_target_ids:
                distances = [
                    np.linalg.norm(agent[:3] - self.targets[candidate_id, :3])
                    for candidate_id in active_target_ids
                ]
                sanitized[agent_id] = active_target_ids[int(np.argmin(distances))]
        return sanitized

    def _record_assignment_metrics(self, assignments):
        """Record load balance and assignment changes under one shared definition."""
        active_target_ids = [
            target_id for target_id in range(len(self.targets))
            if target_id not in self.permanently_captured
        ]
        guarding_ids = self._get_guarding_agent_ids()
        active_agent_ids = {
            int(agent[7]) for agent in self.agents
            if int(agent[7]) not in guarding_ids
        }
        effective_assignments = {
            agent_id: target_id
            for agent_id, target_id in assignments.items()
            if agent_id in active_agent_ids and target_id in active_target_ids
        }

        load_balance = calculate_target_load_balance(
            effective_assignments,
            active_target_ids,
            active_agent_ids,
        )
        self.target_load_balance_history.append(load_balance)
        self.reconfiguration_count += count_reconfigurations(
            self.previous_assignments,
            effective_assignments,
            active_agent_ids,
        )
        self.previous_assignments = effective_assignments.copy()

        self.algorithm.metrics["reconfiguration_count"] = self.reconfiguration_count
        self.algorithm.metrics["target_load_balance"] = load_balance


    def _sfla_local_search(self, node):
        """简化的SFLA局部搜索"""
        if len(node.points) < 2:
            return

        # 简单的向中心移动
        center = node.center
        new_positions = []
        for point in node.points:
            direction = center - point
            dist = np.linalg.norm(direction)
            if dist > 1e-5:
                direction = direction / dist
            new_pos = point + direction * min(dist * 0.5, 50)
            new_positions.append(new_pos)

        node.points = np.array(new_positions)
        node.center = np.mean(node.points, axis=0)



    # ==================== 核心算法：自适应重组 (融合 SFLA + CS) ====================
    def _adaptive_recluster(self):
        """
        混合策略主流程：
        1. SFLA: 所有叶子节点内部进行微调。
        2. CS: 质量差的叶子节点进行全局重组。
        """
        leaf_nodes = self._get_all_leaves(self.ball_tree)

        # 清理空节点
        valid_leaf_nodes = [node for node in leaf_nodes if len(node.points) > 0]
        if not valid_leaf_nodes:
            print("⚠️ 警告: 没有有效叶子节点，重建树结构")
            self._rebuild_tree_from_agents()
            leaf_nodes = self._get_all_leaves(self.ball_tree)
            valid_leaf_nodes = leaf_nodes

        # --- 阶段 1: SFLA 局部开发 ---
        for node in valid_leaf_nodes:
            if len(node.points) > 0:
                self._sfla_local_search(node)

        # --- 阶段 2: 质量评估与 CS 全局探索 ---
        split_candidates = []
        for node in valid_leaf_nodes:
            # 重新计算质量 (因为SFLA可能改变了位置)
            node.quality = self._calculate_cluster_quality(node)
            if node.quality < QTH and len(node.points) >= 4:
                split_candidates.append(node)

        # 执行 CS 重组
        for node in split_candidates:
            self._cs_global_restructure(node)

        # 重新分配智能体到更新后的树结构
        self._assign_agents_to_leaves()

        # ✅ 新增：更新所有非叶节点的子节点质量
        self._update_parent_nodes_quality(self.ball_tree)

    def _update_parent_nodes_quality(self, node):
        """递归更新所有非叶节点的子节点质量"""
        if node is None:
            return

        if not node.is_leaf and node.children:
            # 计算子节点的质量
            node.children_quality = []
            for child in node.children:
                child.quality = self._calculate_cluster_quality(child)
                node.children_quality.append(child.quality)

        # 递归处理子节点
        for child in node.children:
            self._update_parent_nodes_quality(child)

    # ==================== 辅助函数 ====================
    def _assign_agents_to_leaves(self):
        """分配智能体到叶子节点"""
        if self.ball_tree is None:
            print("⚠️ Warning: ball_tree is None")
            return

        leaf_nodes = self._get_all_leaves(self.ball_tree)
        if not leaf_nodes or len(self.env.agents) == 0:
            print(f"⚠️ Warning: No leaf nodes ({len(leaf_nodes)}) or no agents ({len(self.env.agents)})")
            return

        # print(f"🔍 Assigning {len(self.env.agents)} agents to {len(leaf_nodes)} leaf nodes")

        # 清空所有叶子节点的points和agent_ids
        for node in leaf_nodes:
            node.points = []
            node.agent_ids = []

        # 分配智能体
        for agent in self.env.agents:
            agent_pos = agent[:3]
            agent_id = int(agent[7])

            # 找到最近的叶子节点
            min_dist = float('inf')
            best_node = None
            for node in leaf_nodes:
                dist = np.linalg.norm(agent_pos - node.center)
                if dist < min_dist:
                    min_dist = dist
                    best_node = node

            if best_node is not None:
                best_node.points.append(agent_pos)
                best_node.agent_ids.append(agent_id)
            else:
                print(f"⚠️ Warning: No leaf node found for agent {agent_id}")

        # 更新节点特征
        for node in leaf_nodes:
            if len(node.points) > 0:
                node.update_features()
                # print(f"   Node {node.ball_id}: {len(node.agent_ids)} agents, center={node.center}")

    def _cleanup_empty_nodes(self, valid_leaf_nodes):
        """清理树中的空节点"""
        # 获取所有节点
        all_nodes = self._get_all_nodes(self.ball_tree)

        # 标记要删除的空节点
        nodes_to_remove = []
        for node in all_nodes:
            if hasattr(node, 'is_empty') and node.is_empty:
                nodes_to_remove.append(node)

        # 从树中移除空节点
        for node in nodes_to_remove:
            if node.parent:
                # 从父节点的子节点列表中移除
                if node in node.parent.children:
                    node.parent.children.remove(node)
                # 如果父节点现在只有一个子节点，将父节点替换为子节点
                if len(node.parent.children) == 1:
                    child = node.parent.children[0]
                    child.parent = node.parent.parent
                    if node.parent.parent:
                        # 更新祖父节点的子节点列表
                        for i, grandchild in enumerate(node.parent.parent.children):
                            if grandchild == node.parent:
                                node.parent.parent.children[i] = child
                                break
                    else:
                        # 如果父节点是根节点，更新根节点
                        self.ball_tree = child
            elif node == self.ball_tree:
                # 如果根节点是空的，用第一个有效节点替换
                if valid_leaf_nodes:
                    self.ball_tree = valid_leaf_nodes[0]
                    self.ball_tree.parent = None

        # 重新分配智能体到新的树结构
        # self._assign_agents_to_valid_leaves(valid_leaf_nodes)

    def _assign_agents_to_leaves1(self):
        leaf_nodes = self._get_all_leaves(self.ball_tree)
        if not leaf_nodes or len(self.agents) == 0:
            return

        # 清空所有叶子节点的 points 和 agent_ids
        for node in leaf_nodes:
            node.points = []
            node.agent_ids = []

        # 遍历每个智能体，分配到最近的叶子节点
        for agent in self._get_active_agents():
            agent_pos = agent[:3]
            agent_id = int(agent[7])
            best_node = min(leaf_nodes, key=lambda x: np.linalg.norm(agent_pos - x.center))
            best_node.points.append(agent_pos)
            best_node.agent_ids.append(agent_id)

        # 更新节点几何属性
        for node in leaf_nodes:
            if len(node.points) > 0:
                node.points = np.array(node.points)
                node.center = np.mean(node.points, axis=0)
                node.radius = np.max(cdist([node.center], node.points)) if len(node.points) > 1 else 0
            else:
                node.is_empty = True

        # 清理空节点（但不重新分配）
        self._cleanup_empty_nodes(leaf_nodes)

    def _rebuild_tree_from_agents(self):
        """当所有节点都为空时，从未承担守卫任务的智能体重建树。"""
        active_agents = self._get_active_agents()
        all_points = np.asarray([agent[:3] for agent in active_agents], dtype=float)
        self.ball_tree = GranularBallNode(self._get_next_id(), all_points)
        self.all_balls = {self.ball_tree.ball_id: self.ball_tree}

        self.ball_tree.agent_ids = [int(agent[7]) for agent in active_agents]

    def _get_all_nodes(self, node):
        """获取树中的所有节点"""
        if node is None:
            return []
        nodes = [node]
        for child in node.children:
            nodes.extend(self._get_all_nodes(child))
        return nodes



    def _get_all_leaves(self, node):
        if node is None:
            return []
        if node.is_leaf:
            return [node]
        leaves = []
        for child in node.children:
            leaves.extend(self._get_all_leaves(child))
        return leaves

    def _get_next_id(self):
        id_val = self.next_ball_id
        self.next_ball_id += 1
        return id_val

    def _calculate_cluster_quality(self, node):
        points = node.points
        n = len(points)
        if n == 0:
            return 0
        elif n == 1:
            return 0.5

        center = node.center
        distances = cdist(points, [center])
        avg_distance = np.mean(distances)
        internal_tightness = 1 / (1 + avg_distance / (ARENA_SIZE_XY * 0.1))

        ideal_min = MIN_CAPTURE_AGENTS
        ideal_max = MIN_CAPTURE_AGENTS + 3
        if n < ideal_min:
            scale_factor = n / ideal_min
        elif n <= ideal_max:
            scale_factor = 1.0
        else:
            scale_factor = max(0.5, 1.0 - (n - ideal_max) * 0.05)

        quality = (0.6 * internal_tightness + 0.4 * scale_factor)
        return quality

    def _calculate_fixed_circle_position(self, target_pos, agent_id, guard_ids, agent_type):
        """
        计算智能体在固定圆圈上的位置
        :param target_pos: 目标位置
        :param agent_id: 智能体ID
        :param guard_ids: 守卫智能体ID列表
        :param agent_type: 智能体类型 (0: UAV, 1: USV)
        :return: 期望位置
        """
        if agent_id not in guard_ids:
            return None

        idx = guard_ids.index(agent_id)
        n = len(guard_ids)

        # 圆圈半径 = CAPTURE_RADIUS/4 (因为直径是CAPTURE_RADIUS/2)
        circle_radius = CAPTURE_RADIUS / 4

        # 计算每个智能体的角度
        angle = 2 * np.pi * idx / n

        # 计算圆圈上的位置
        desired_x = target_pos[0] + circle_radius * np.cos(angle)
        desired_y = target_pos[1] + circle_radius * np.sin(angle)

        # 根据智能体类型设置高度。UAV 的三维守卫位置必须仍处于捕获半径内。
        if agent_type == 0:  # UAV
            max_vertical_offset = np.sqrt(max(CAPTURE_RADIUS ** 2 - circle_radius ** 2, 0.0))
            desired_z = target_pos[2] + min(200.0, 0.9 * max_vertical_offset)
        else:  # USV
            desired_z = target_pos[2]  # USV和目标同高

        return np.array([desired_x, desired_y, desired_z])

    def _update_capture_state(self):
        """在智能体和目标完成本步运动后检测围捕，并锁定守卫智能体。"""
        current_locked = set()
        self.target_captors = {}

        for target_id in range(len(self.targets)):
            target_pos = self.targets[target_id, :3]
            dists = np.linalg.norm(self.agents[:, :3] - target_pos, axis=1)
            in_range_indices = np.where(dists <= CAPTURE_RADIUS)[0]
            captor_ids = {
                int(self.agents[index, 7])
                for index in in_range_indices
            }
            self.target_captors[target_id] = captor_ids

            if len(captor_ids) >= MIN_CAPTURE_AGENTS:
                current_locked.add(target_id)
                if target_id not in self.guarding_agents:
                    # 选择距离目标最近的 MIN_CAPTURE_AGENTS 个智能体作为守卫，结果稳定且可复现。
                    sorted_indices = in_range_indices[np.argsort(dists[in_range_indices])]
                    guard_ids = {
                        int(self.agents[index, 7])
                        for index in sorted_indices[:MIN_CAPTURE_AGENTS]
                    }
                    self.guarding_agents[target_id] = guard_ids
                    print(f"\n🎯 目标 {target_id} 已被围捕成功！")
                    print(f"   参与围捕的智能体ID: {sorted(captor_ids)}")
                    print(f"   保留守卫智能体ID: {sorted(guard_ids)}")
                    print(f"   释放智能体ID: {sorted(captor_ids - guard_ids)}")

        self.permanently_captured.update(current_locked)

    def _resolve_algorithm_waypoint(self, agent_id, target_id, fallback_position):
        """仅为有效 GBSFLACS 使用其可执行航点，其他算法保持原有直追目标逻辑。"""
        fallback = np.asarray(fallback_position, dtype=float)
        if self.algorithm_name != "GB-SFLA-CS":
            return fallback
        waypoints = getattr(self.algorithm, "desired_waypoints", {}) or {}
        waypoint = waypoints.get(int(agent_id))
        if waypoint is None:
            return fallback
        waypoint = np.asarray(waypoint, dtype=float)
        if waypoint.shape != (3,) or not np.all(np.isfinite(waypoint)):
            return fallback
        if int(target_id) in getattr(self, "permanently_captured", set()):
            return fallback
        return waypoint

    # ==================== 主循环逻辑 ====================
    def step(self):
        total_energy = 0
        # 1. 使用算法进行分组和任务分配。围捕判定在本步运动完成后执行。
        assignments = self.algorithm.step()  # 现在是 {智能体ID: 目标ID}
        assignments = self._sanitize_assignments(assignments)
        self._record_assignment_metrics(assignments)
        # 2. 获取分组节点（用于可视化同步）
        leaf_nodes = []
        if hasattr(self.algorithm, 'clusters_for_drawing'):
            leaf_nodes = self.algorithm.clusters_for_drawing
        elif hasattr(self, 'ball_tree') and self.ball_tree is not None:
            leaf_nodes = self._get_all_leaves(self.ball_tree)

        # 3. 执行移动
        for i in range(len(self.agents)):
            agent = self.agents[i]
            agent_pos = agent[:3]
            agent_id = int(agent[7])
            agent_type = int(agent[6])
            max_v = V_MAX_UAV if agent_type == 0 else V_MAX_USV

            # 检查是否为守卫智能体
            is_guarding = False
            guard_target_id = None
            for target_id, guards in self.guarding_agents.items():
                if agent_id in guards:
                    is_guarding = True
                    guard_target_id = target_id
                    break

            if is_guarding:
                # 执行守卫动作
                target_pos = self.targets[guard_target_id, :3]
                guard_ids = list(self.guarding_agents[guard_target_id])
                desired_pos = self._calculate_fixed_circle_position(target_pos, agent_id, guard_ids, agent_type)
                if desired_pos is None:
                    desired_pos = agent_pos
            else:
                # 非守卫智能体，根据算法分配移动
                if agent_id in assignments:
                    target_id = assignments[agent_id]
                    target_pos = self.targets[target_id, :3]
                    desired_pos = self._resolve_algorithm_waypoint(
                        agent_id, target_id, target_pos
                    )
                else:
                    # 没有分配到目标，保持原地
                    desired_pos = agent_pos

            # 动力学更新
            dx = desired_pos[0] - agent[0]
            dy = desired_pos[1] - agent[1]
            dz = desired_pos[2] - agent[2]
            distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            if distance < 1e-5: continue

            desired_theta = np.arctan2(dy, dx)
            desired_phi = np.arctan2(dz, np.sqrt(dx ** 2 + dy ** 2))
            new_v = min(distance, max_v)

            new_x = agent[0] + new_v * np.cos(desired_phi) * np.cos(desired_theta)
            new_y = agent[1] + new_v * np.cos(desired_phi) * np.sin(desired_theta)
            new_z = agent[2] + new_v * np.sin(desired_phi)
            if agent_type == 1: new_z = USV_Z

            self.agents[i, 0] = new_x
            self.agents[i, 1] = new_y
            self.agents[i, 2] = new_z
            self.agents[i, 3] = new_v
            self.agents[i, 4] = desired_theta
            self.agents[i, 5] = desired_phi
            self.total_travel_distance += new_v
            total_energy += new_v ** 2

        if TARGET_IS_STATIC == 0:
            self._update_targets()

        # 4. 在本步智能体/目标运动完成后立即判定围捕，避免捕获时间滞后一拍。
        self._update_capture_state()
        self.time_step += 1
        self.algorithm.metrics["total_distance"] = self.total_travel_distance
        self.algorithm.metrics["energy_consumption"] += total_energy

        # 同步粒球树（仅GB-SFLA需要）
        if self.algorithm_name in ["GB-SFLA", "GB-SFLA-CS", "GB-CS", "GB-only"]:
            self._sync_tree_with_agents()

        self.current_clusters_for_drawing = leaf_nodes

        return total_energy

    def _update_targets(self):
        """目标逃跑逻辑；随机种子仅在 reset() 时设置，随后连续推进随机流。"""
        for j in range(len(self.targets)):
            if j in self.permanently_captured:
                self.targets[j, 3] = 0
                continue
            target = self.targets[j]
            tx, ty, tz = target[0], target[1], TARGET_Z
            nearby_agents = []
            for agent in self.agents:
                dist_xy = np.sqrt((agent[0] - tx) ** 2 + (agent[1] - ty) ** 2)
                if dist_xy < TARGET_RUN_NUM:
                    nearby_agents.append((agent[0], agent[1], dist_xy))
            if nearby_agents:
                avg_x = sum([a[0] for a in nearby_agents]) / len(nearby_agents)
                avg_y = sum([a[1] for a in nearby_agents]) / len(nearby_agents)
                escape_dx = tx - avg_x
                escape_dy = ty - avg_y
                norm = np.sqrt(escape_dx ** 2 + escape_dy ** 2)
                if norm > 1e-5:
                    escape_dx = escape_dx / norm * TARGET_SPEED
                    escape_dy = escape_dy / norm * TARGET_SPEED
                else:
                    escape_dx, escape_dy = 0, 0
                target[0] += escape_dx
                target[1] += escape_dy
                target[3] = np.sqrt(escape_dx ** 2 + escape_dy ** 2)
                target[4] = np.arctan2(escape_dy, escape_dx)
            else:
                target[3] = TARGET_SPEED
                target[4] += random.uniform(-0.1, 0.1)
                target[0] += target[3] * np.cos(target[4])
                target[1] += target[3] * np.sin(target[4])
                target[2] = TARGET_Z

        # ==================== 新增：SFLA 与 CS 核心算法模块 ====================

    def _sync_tree_geometry_only(self):
        """只同步几何属性，不重新分配智能体ID"""
        leaf_nodes = self._get_all_leaves(self.ball_tree)
        if not leaf_nodes:
            return

        # 只更新几何属性，不清空agent_ids
        for node in leaf_nodes:
            if len(node.points) > 0:
                node.points = np.array(node.points)
                node.center = np.mean(node.points, axis=0)
                distances = cdist([node.center], node.points)
                node.radius = np.max(distances) if len(distances) > 0 else 0
            else:
                node.points = np.empty((0, 3))
                node.radius = 1.0

    def _assign_and_sync_agents(self):
        """统一的分配和同步函数"""
        leaf_nodes = self._get_all_leaves(self.ball_tree)
        if not leaf_nodes or len(self.agents) == 0:
            return

        # 清空所有叶子节点的 points 和 agent_ids
        for node in leaf_nodes:
            node.points = []
            node.agent_ids = []

        # 遍历每个智能体，分配到最近的叶子节点
        for agent in self._get_active_agents():
            agent_pos = agent[:3]
            agent_id = int(agent[7])
            best_node = min(leaf_nodes, key=lambda x: np.linalg.norm(agent_pos - x.center))
            best_node.points.append(agent_pos)
            best_node.agent_ids.append(agent_id)

        # 更新节点几何属性
        for node in leaf_nodes:
            if node.points:
                node.points = np.array(node.points)
                node.center = np.mean(node.points, axis=0)
                distances = cdist([node.center], node.points)
                node.radius = np.max(distances) if len(distances) > 0 else 0
            else:
                node.is_empty = True

        # 清理空节点
        self._cleanup_empty_nodes(leaf_nodes)

    def _sync_tree_with_agents(self):
        leaf_nodes = self._get_all_leaves(self.ball_tree)
        if not leaf_nodes:
            return

        # 1. 同时清空 points 和 agent_ids
        for node in leaf_nodes:
            node.points = []
            node.agent_ids = []  # 关键修复：清空 agent_ids

        # 2. 遍历所有智能体，同时分配坐标和ID
        for agent in self._get_active_agents():
            agent_pos = agent[:3]
            agent_id = int(agent[7])  # 提取智能体ID
            best_node = None
            best_dist = float('inf')
            for node in leaf_nodes:
                dist = np.linalg.norm(agent_pos - node.center)
                if dist < best_dist:
                    best_dist = dist
                    best_node = node
            if best_node is not None:
                best_node.points.append(agent_pos)
                # ✅ 添加去重检查：如果这个ID已经在这个节点中，就不要再添加
                if agent_id not in best_node.agent_ids:
                    best_node.agent_ids.append(agent_id)

        # 3. 更新几何属性
        for node in leaf_nodes:
            if node.points:
                node.points = np.array(node.points)
                node.center = np.mean(node.points, axis=0)
                distances = cdist([node.center], node.points)
                node.radius = np.max(distances) if len(distances) > 0 else 0
            else:
                node.points = np.empty((0, 3))
                node.radius = 1.0

    def print_final_tree_structure(self):
        """
        仿真结束时，打印完整的粒球树结构和包含的智能体ID
        修复：直接使用节点存储的agent_ids，不再依赖坐标匹配
        """
        print("\n" + "=" * 60)
        print("📊 仿真结束：最终粒球树结构与成员报告")
        print("=" * 60)

        # ✅ 新增：输出围捕总结
        print(f"\n🏆 围捕总结:")
        print(f"   总目标数: {TARGET_COUNT}")
        print(f"   成功围捕目标数: {len(self.permanently_captured)}")
        print(f"   剩余未围捕目标数: {TARGET_COUNT - len(self.permanently_captured)}")

        if self.guarding_agents:
            print(f"\n🛡️ 守卫智能体分配:")
            for target_id, guard_ids in self.guarding_agents.items():
                print(f"   目标 {target_id}: 守卫智能体 {sorted(list(guard_ids))}")

        print("\n" + "-" * 60)

        def _recursive_print(node, depth=0):
            indent = "  " * depth
            ball_id = node.ball_id
            point_count = len(node.points)
            quality = getattr(node, 'quality', 'N/A')
            node_type = "叶子节点(Leaf)" if node.is_leaf else "非叶节点(Internal)"

            # ✅ 显示子节点质量
            children_quality_str = ""
            if not node.is_leaf and hasattr(node, 'children_quality') and node.children_quality:
                children_quality_str = f", 子节点质量: {[round(q, 3) for q in node.children_quality]}"

            print(
                f"{indent}📂 Ball-{ball_id} | {node_type} | 数量: {point_count} | 质量: {quality:.3f}{children_quality_str}")

            # 直接使用存储的agent_ids，无需坐标匹配
            if node.is_leaf and hasattr(node, 'agent_ids') and node.agent_ids:
                id_str = ", ".join(map(str, sorted(node.agent_ids)))
                print(f"{indent}   🆔 包含智能体ID: [{id_str}]")
            elif node.is_leaf:
                print(f"{indent}   🆔 包含智能体ID: []")

            for child in node.children:
                _recursive_print(child, depth + 1)

        _recursive_print(self.ball_tree)


# ==================== 7. 绘图与主函数 (最终整合版) ====================

def draw_view(ax, agents, targets, img_uav, img_usv, elev, azim, title, env):
    """优化后的绘图函数：三视图 + 图片显示 + 粒球树可视化"""
    ax.clear()

    # ================= 核心设置 =================
    ax.set_box_aspect([1, 1, 1])  # 强制 1:1:1 比例
    xy_edge = 0.5
    ax.set_xlim(-ARENA_SIZE_XY * xy_edge, ARENA_SIZE_XY * (1 + xy_edge))
    ax.set_ylim(-ARENA_SIZE_XY * xy_edge, ARENA_SIZE_XY * (1 + xy_edge))
    ax.set_zlim(-ARENA_SIZE_XY * xy_edge, ARENA_SIZE_XY * (1 + xy_edge))
    ax.set_title(title, fontsize=12, pad=10)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=elev, azim=azim)
    ax.grid(True, alpha=0.3)

    # 1. 绘制地面实心平面
    plane_size = ARENA_SIZE_XY * (1 + xy_edge)
    xx = np.array([[0 - ARENA_SIZE_XY * xy_edge, 0 - ARENA_SIZE_XY * xy_edge],
                   [plane_size, plane_size]])
    yy = np.array([[0 - ARENA_SIZE_XY * xy_edge, plane_size],
                   [0 - ARENA_SIZE_XY * xy_edge, plane_size]])
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, color='lightblue', alpha=0.15, zorder=0, linewidth=0)

    # 2. 绘制粒球 (基于二叉树的叶子节点)
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'brown', 'cyan', 'magenta']

    # 关键修复：直接从 env.agents 获取最新坐标构建虚拟球 (后备方案)
    # 如果基于树的绘制失效，这个后备方案能保证至少有一个球显示
    if not hasattr(env, 'current_clusters_for_drawing') or len(env.current_clusters_for_drawing) == 0:
        print("⚠️ 警告: 未检测到有效粒球，绘制全局包围球...")
        # 绘制一个包含所有智能体的大球作为 fallback
        all_pts = np.array([a[:3] for a in agents])
        center = np.mean(all_pts, axis=0)
        radius = np.max(np.sqrt(np.sum((all_pts - center) ** 2, axis=1))) if len(all_pts) > 0 else 100

        # 绘制 fallback 球
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='gray', alpha=0.1, linewidth=0, shade=False)
    else:
        # 正常绘制逻辑
        for idx, node in enumerate(env.current_clusters_for_drawing):
            color = colors[idx % len(colors)]

            # 双重保险：检查 node.points 是否有效
            if not hasattr(node, 'points') or len(node.points) == 0:
                continue

            try:
                center = node.center
                radius = node.radius

                # A. 绘制中心点
                # ax.scatter(center[0], center[1], center[2], c=color, s=80, marker='o', edgecolors='black', zorder=5)

                # B. 绘制粒球框 (3D 球面)
                # 注意：这里加了 try-catch 防止 center 或 radius 是 nan
                try:
                    u = np.linspace(0, 2 * np.pi, 15)
                    v = np.linspace(0, np.pi, 15)
                    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
                    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
                    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
                    ax.plot_surface(x, y, z, color=color, alpha=0.05, linewidth=0, shade=False, zorder=1)
                except Exception as e:
                    print(f"绘制球体失败: {e}")
                    continue

                # C. 绘制连线 (体现整体性)
                # 修复：确保 points 是 numpy 数组且格式正确

                points_array = np.array(node.points)
                # 如果半径太小或者没有点，跳过绘制球面，只画连线
                if radius < 1e-5 or len(points_array) == 0:
                    continue
                if points_array.size > 0:
                    for point in points_array:
                        if len(point) >= 3:
                            ax.plot([point[0], center[0]],
                                    [point[1], center[1]],
                                    [point[2], center[2]],
                                    color=color, alpha=0.4, linewidth=1, zorder=5)
            except Exception as e:
                print(f"处理节点 {idx} 时出错: {e}")
                continue

    # 3. 绘制智能体 (图片模式)
    for agent in agents:
        x, y, z = agent[:3]
        agent_type = int(agent[6])  # 0: UAV, 1: USV

        # 选择图片
        current_img = img_uav if agent_type == 0 else img_usv

        if current_img:
            # 将 3D 坐标投影到 2D 屏幕坐标
            x2d, y2d, _ = proj3d.proj_transform(x, y, z, ax.get_proj())
            # 创建 AnnotationBbox
            ab = AnnotationBbox(current_img, (x2d, y2d), xycoords='data', frameon=False, zorder=10)
            ax.add_artist(ab)
        else:
            # 如果没有图片，回退到几何图形
            color = 'cyan' if agent_type == 0 else 'limegreen'
            marker = '^' if agent_type == 0 else 's'
            ax.scatter(x, y, z, c=color, marker=marker, s=100, edgecolors='black', zorder=10)

    # 4. 绘制目标
    if len(targets) > 0:
        for i, target in enumerate(targets):
            tx, ty, tz = target[:3]
            # 被捕获变绿色，否则红色
            color = 'green' if i in env.permanently_captured else 'red'

            # ------------------- 替换：绘制三维捕获球（仅未被捕获的目标） -------------------
            # if i not in env.permanently_captured:
            # 1. 生成球体参数（θ：方位角0~2π，φ：极角0~π）
            theta = np.linspace(0, 2 * np.pi, 30)  # 方位角采样点（控制圆周细分）
            phi = np.linspace(0, np.pi, 20)  # 极角采样点（控制上下半球细分）
            theta_grid, phi_grid = np.meshgrid(theta, phi)  # 生成网格

            # 2. 球体参数方程（中心(tx,ty,tz)，半径CAPTURE_RADIUS）
            x = tx + CAPTURE_RADIUS * np.sin(phi_grid) * np.cos(theta_grid)
            y = ty + CAPTURE_RADIUS * np.sin(phi_grid) * np.sin(theta_grid)
            z = tz + CAPTURE_RADIUS * np.cos(phi_grid)

            # 3. 绘制3D球体表面（保持原视觉风格：红、半透、低层级）
            ax.plot_surface(
                x, y, z,
                color=color,  # 原圈的颜色
                alpha=0.2,  # 原圈的透明度
                rstride=1,  # 行步长（控制曲面精细度）
                cstride=1,  # 列步长（控制曲面精细度）
                zorder=10  # 原圈的层级（确保不被其他元素遮挡）
            )

            # ------------------- 保留：绘制目标本体 -------------------
            ax.scatter(tx, ty, tz, c=color, s=2, marker='.', linewidth=2, zorder=11)



# ==================== 修改后的主函数 ====================
def main2():
    """运行多个算法并可视化对比"""
    algorithms = ["GB-SFLA-CS"]

    results = {}

    # 创建图形窗口用于对比显示
    plt.ion()
    n_cols = 4
    n_rows = int(np.ceil(len(algorithms) / n_cols))
    fig_compare = plt.figure(figsize=(5 * n_cols, 6 * n_rows))

    # 为每个算法创建子图
    axes = []
    for i, algo in enumerate(algorithms):
        ax = fig_compare.add_subplot(n_rows, n_cols, i + 1, projection='3d')
        axes.append(ax)

    # 加载图片
    img_uav = load_image('./figs/UAV3.png', size=0.3)
    img_usv = load_image('./figs/USV3.png', size=0.3)

    # 运行每个算法
    for idx, algo_name in enumerate(algorithms):
        print(f"\n{'=' * 60}")
        print(f"Running {algo_name}...")
        print(f"{'=' * 60}")

        # 初始化环境
        env = SwarmEnv3D()
        env.set_algorithm(algo_name)

        # 获取当前子图
        ax = axes[idx]
        ax.clear()

        # 设置子图标题
        ax.set_title(f"{algo_name}\nStep: 0", fontsize=10)
        ax.set_xlim(0, ARENA_SIZE_XY)
        ax.set_ylim(0, ARENA_SIZE_XY)
        ax.set_zlim(0, ARENA_SIZE_Z)

        # 运行仿真
        max_steps = 100
        capture_history = []

        for step in range(max_steps):
            # 执行一步
            total_energy = env.step()

            # 每10步更新一次可视化
            if step % 1 == 0:
                print(f"  Step {step}: {len(env.permanently_captured)}/{len(env.targets)} targets captured")

                # 清除子图
                ax.clear()

                # 绘制粒球
                if hasattr(env, 'current_clusters_for_drawing'):
                    for node in env.current_clusters_for_drawing:
                        if hasattr(node, 'center') and hasattr(node, 'radius'):
                            center = node.center
                            radius = node.radius

                            # 绘制球体
                            u = np.linspace(0, 2 * np.pi, 10)
                            v = np.linspace(0, np.pi, 10)
                            x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
                            y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
                            z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
                            ax.plot_surface(x, y, z, alpha=0.1, color='blue')

                # 绘制智能体
                for agent in env.agents:
                    x, y, z = agent[:3]
                    agent_type = int(agent[6])
                    color = 'cyan' if agent_type == 0 else 'limegreen'
                    marker = '^' if agent_type == 0 else 's'
                    ax.scatter(x, y, z, c=color, marker=marker, s=10)

                # 绘制目标
                for i, target in enumerate(env.targets):
                    tx, ty, tz = target[:3]
                    color = 'green' if i in env.permanently_captured else 'red'
                    ax.scatter(tx, ty, tz, c=color, s=10, marker='o')

                    # 绘制捕获范围
                    if i not in env.permanently_captured:
                        u = np.linspace(0, 2 * np.pi, 20)
                        v = np.linspace(0, np.pi, 20)
                        x = tx + CAPTURE_RADIUS * np.outer(np.cos(u), np.sin(v))
                        y = ty + CAPTURE_RADIUS * np.outer(np.sin(u), np.sin(v))
                        z = tz + CAPTURE_RADIUS * np.outer(np.ones(np.size(u)), np.cos(v))
                        ax.plot_surface(x, y, z, alpha=0.1, color=color)

                # 更新标题
                ax.set_title(f"{algo_name}\nStep: {step}, Captured: {len(env.permanently_captured)}/{len(env.targets)}",
                             fontsize=10)
                ax.set_xlim(0, ARENA_SIZE_XY)
                ax.set_ylim(0, ARENA_SIZE_XY)
                ax.set_zlim(0, ARENA_SIZE_Z)

                # 刷新图形
                fig_compare.canvas.draw()
                fig_compare.canvas.flush_events()
                plt.pause(0.01)

            # 记录捕获历史
            capture_history.append(len(env.permanently_captured))

            # 检查是否完成
            if len(env.permanently_captured) >= len(env.targets):
                print(f"✅ {algo_name} completed in {step + 1} steps!")
                break

        # 记录结果
        results[algo_name] = {
            "capture_time": step + 1,
            "total_travel_distance": env.total_travel_distance,
            "target_load_balance": (
                float(np.mean(env.target_load_balance_history))
                if env.target_load_balance_history else 0.0
            ),
            "reconfiguration_count": env.reconfiguration_count,
            "recluster_count": env.algorithm.metrics["recluster_count"],
            "success_rate": calculate_success_rate(env.permanently_captured, len(env.targets)),
            "capture_history": capture_history,
            "target_load_balance_history": env.target_load_balance_history.copy()
        }

        print(f"📊 Results for {algo_name}:")
        print(f"   Capture Time: {results[algo_name]['capture_time']} steps")
        print(f"   Total Travel Distance: {results[algo_name]['total_travel_distance']:.2f}")
        print(f"   Target Load Balance (mean CV): {results[algo_name]['target_load_balance']:.4f}")
        print(f"   Reconfiguration Count: {results[algo_name]['reconfiguration_count']}")
        print(f"   Recluster Count: {results[algo_name]['recluster_count']}")
        print(f"   Success Rate (Captured Targets/Total): {results[algo_name]['success_rate'] * 100:.1f}%")
    print('----------------------------\n','results',results)
    # 创建性能对比图表
    create_performance_comparison(results, algorithms)

    plt.ioff()
    plt.show()


def create_performance_comparison(results, algorithms):
    """创建性能对比图表"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 15))
    colors = ['blue', 'green', 'red', 'orange', 'purple', 'brown', 'cyan', 'magenta']


    def annotate_bars(axis, bars, formatter):
        for bar in bars:
            height = bar.get_height()
            axis.text(
                bar.get_x() + bar.get_width() / 2.,
                height,
                formatter(height),
                ha='center',
                va='bottom',
            )

    # 1. 捕获时间对比
    ax1 = axes[0, 0]
    capture_times = [results[algo]["capture_time"] for algo in algorithms]
    bars1 = ax1.bar(algorithms, capture_times, color=colors[:len(algorithms)])
    ax1.set_title('Capture Time Comparison')
    ax1.set_ylabel('Steps')
    ax1.tick_params(axis='x', rotation=45)
    annotate_bars(ax1, bars1, lambda value: f'{int(value)}')

    # 2. 统一定义的重配置次数对比
    ax2 = axes[0, 1]
    reconfiguration_counts = [results[algo]["reconfiguration_count"] for algo in algorithms]
    bars2 = ax2.bar(algorithms, reconfiguration_counts, color=colors[:len(algorithms)])
    ax2.set_title('Reconfiguration Count Comparison')
    ax2.set_ylabel('Count')
    ax2.tick_params(axis='x', rotation=45)
    annotate_bars(ax2, bars2, lambda value: f'{int(value)}')

    # 3. 成功率对比
    ax3 = axes[1, 0]
    success_rates = [results[algo]["success_rate"] * 100 for algo in algorithms]
    bars3 = ax3.bar(algorithms, success_rates, color=colors[:len(algorithms)])
    ax3.set_title('Success Rate Comparison')
    ax3.set_ylabel('Captured Targets / Total Targets (%)')
    ax3.set_ylim(0, 110)
    ax3.tick_params(axis='x', rotation=45)
    annotate_bars(ax3, bars3, lambda value: f'{value:.1f}%')

    # 4. 总航程对比
    ax4 = axes[1, 1]
    total_travel_distances = [results[algo]["total_travel_distance"] for algo in algorithms]
    bars4 = ax4.bar(algorithms, total_travel_distances, color=colors[:len(algorithms)])
    ax4.set_title('Total Travel Distance Comparison')
    ax4.set_ylabel('Distance')
    ax4.tick_params(axis='x', rotation=45)
    annotate_bars(ax4, bars4, lambda value: f'{value:.0f}')

    # 5. 活跃目标负载均衡对比
    ax5 = axes[2, 0]
    target_load_balances = [results[algo]["target_load_balance"] for algo in algorithms]
    bars5 = ax5.bar(algorithms, target_load_balances, color=colors[:len(algorithms)])
    ax5.set_title('Target Load Balance Comparison')
    ax5.set_ylabel('Mean Load CV (Lower is Better)')
    ax5.tick_params(axis='x', rotation=45)
    annotate_bars(ax5, bars5, lambda value: f'{value:.3f}')

    # 6. 捕获进度曲线
    ax6 = axes[2, 1]
    for algo in algorithms:
        history = results[algo]["capture_history"]
        ax6.plot(range(len(history)), history, label=algo, linewidth=2)
    ax6.set_title('Capture Progress Over Time')
    ax6.set_xlabel('Steps')
    ax6.set_ylabel('Number of Captured Targets')
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()

    from datetime import datetime
    now = datetime.now()
    month = now.month  # 月 (1-12)
    day = now.day  # 日 (1-31)
    hour = now.hour  # 小时 (0-23)
    minute = now.minute  # 分钟 (0-59)

    plt.savefig('./result_png/algorithm_comparison' +
                str(INIT_RANDOMSTATE) +
                str(UAV_COUNT) +
                str(USV_COUNT) +
                str(TARGET_COUNT) +
                str(month) +
                str(day) +
                str(hour) +
                str(minute) +
                '.png',
                dpi=150,
                bbox_inches='tight')
    print("\n📊 Performance comparison chart saved as 'algorithm_comparison  "+
          str(INIT_RANDOMSTATE) +
          str(UAV_COUNT) +
          str(USV_COUNT) +
          str(TARGET_COUNT)
          )


# ==================== 修改后的主函数入口 ====================
if __name__ == "__main__":

    print("🚀 Running algorithm comparison with visualization...")
    main2()
