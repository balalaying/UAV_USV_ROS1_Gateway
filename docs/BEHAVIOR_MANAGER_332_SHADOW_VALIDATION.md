# Behavior Manager 332 Shadow验证

验证日期：2026-07-27

## 验证范围

本次验证只检查完整332世界中的 Fleet World Model 输入、行为判断、角色建议
和控制隔离。为降低无关启动负载，未启动 PX4、DDS、RViz 和 Mid-360 RGL。

启动命令：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  start_px4:=false \
  start_dds_agent:=false \
  start_rviz:=false \
  enable_mid360:=false \
  enable_shadow_behavior_manager:=true
```

测试持续约2分钟。

## World Model结果

在线读取 `/fleet/world_model_summary`：

```text
schema_version: fleet_world_model.summary.v1
map_frame: map
uav_count: 3
usv_count: 3
entity_count: 2
target_count: 1
threat_count: 1
primary_source: ground_truth
mission_state: TRACKING
behavior_state: DEFENSE
tf_edge_count: 14
```

对象覆盖：

```text
UAV: uav_01, uav_02, uav_03
USV: usv_01, usv_02, usv_03
任务实体: friendly_ship, enemy_ship
```

UAV未启动PX4，所以World Model中的UAV来自`map -> uav_xx/base_link`
TF占位状态，`control_ready=false`。三艘USV agent在线，因此
`usv_control_ready=3`。

## 行为结果

在线威胁：

```text
target_id: enemy_ship
threat_level: HIGH
threat_score: 约0.66
distance_to_friendly_ship: 约70.5m
```

Shadow结果：

```text
behavior: DEFENSE
reason: hostile_target_inside_defense_policy
shadow_mode: true
control_enabled: false
publishes_fleet_command: false
```

动态角色：

```text
uav_01 -> threat_observer
uav_02 -> defense_overwatch_01
uav_03 -> defense_overwatch_02
usv_02 -> primary_interceptor
usv_01 -> defense_screen_01
usv_03 -> defense_screen_02
```

主拦截USV不是固定编号。策略根据World Model中的实时位置，选择离
`enemy_ship`最近的平台，本次为`usv_02`。

所有建议均包含：

```text
frame_id: map
control_command: null
```

## 发现并修复的问题

首次运行时，`capture_manager`在`auto_start=false`状态下获得目标轨迹后，
会从`SEARCH`进入被动`TRACKING`。Shadow策略原先把`TRACKING`解释为围捕
已开始，导致：

```text
mission_state: TRACKING
behavior_state: CAPTURE
```

实际此时操作者尚未批准围捕。修复后只有以下真正执行阶段才映射为
`CAPTURE`：

```text
APPROACHING
ENCIRCLING
HOLDING
```

被动`TRACKING`继续交给威胁策略判断，本场景正确得到`DEFENSE`。

新增单元测试：

```text
test_passive_capture_tracking_does_not_claim_capture_execution
```

策略测试结果：

```text
6 passed
```

## 频率和稳定性

```text
/fleet/world_model: 5.000 Hz
/fleet/behavior/shadow_state: 2.000 Hz
World Model age: 通常小于0.2s
transition_pending: false
节点崩溃: 0
```

`fleet_behavior_manager`运行拓扑：

```text
Subscriber:
  /fleet/world_model

Publisher:
  /fleet/behavior/shadow_state
```

它没有订阅原始Camera、PointCloud或单平台状态，也没有发布
`/fleet/command`和`/fleet/control_lease`。

在`auto_start=false`期间监听`/fleet/command`5秒，没有收到任务命令。
`/fleet/command`唯一发布者仍为原有`capture_manager`。

## 已知问题

1. 本次没有启动PX4，因此没有验证三架UAV的真实`control_ready`状态；
2. 本次关闭Mid-360，只验证了行为层和统一世界模型，不代表完整感知回归；
3. 三艘USV的`scan_raw`出现ROS 2可靠性QoS不兼容警告，可能影响Nav2局部
   障碍物输入；该问题不影响本次Shadow结果，但正式避障验收前需要修复；
4. 当前行为结果仍是JSON调试接口，真实控制启用前需要任务请求鉴权、
   ControlLease仲裁和安全约束；
5. 当前没有执行操作者批准后的真实`CAPTURE`切换，避免在本次控制隔离
   测试中触发PX4/Nav2任务。

## 结论

332世界下的Shadow Behavior Manager已证明：

- 只依赖Fleet World Model；
- 能看到完整3 UAV + 3 USV + 2任务实体；
- 能根据实时威胁和位置动态分配角色；
- 被动跟踪不会误报为已执行围捕；
- 行为状态成功回写到Fleet World Model；
- 不改变现有控制链。

这只是行为决策Shadow验收，不代表护航、防御、围捕真实控制闭环已经完成。
