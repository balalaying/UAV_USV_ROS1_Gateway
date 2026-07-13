# 舰队动态围捕验证记录

验证日期：2026-07-13

## 范围

本阶段将双 UAV 围捕扩展为可配置的多载具任务层，并实际验证：

- 4 架 PX4 UAV：`uav_01` 至 `uav_04`
- 2 艘 Nav2 USV：`usv_01`、`usv_02`
- 1 艘动态目标船：`enemy_target`
- UAV 和 USV 均只接收 `FleetCommand`，任务节点不直接控制 Gazebo 或 PX4

冻结接口及校验值见 `CAPTURE_INTERFACE_FREEZE_V1.md`。

## 数据链路

```text
Gazebo enemy_target pose
  -> target_tracker
  -> /perception/tracked_objects (TrackedObjectArray)
  -> capture_manager
       -> target_predictor (CTRV)
       -> capture_planner (按类型构造槽位 + 匈牙利分配)
       -> /fleet/command (FleetCommand)
  -> UAV/USV agent
       -> PX4 Offboard / Nav2 NavigateToPose
       -> /fleet/vehicle_state (VehicleState)
       -> /fleet/command_ack (CommandAck)
  -> capture_manager 状态推进和故障重分配
```

结构化观察接口：

- `/capture/state`：`uav_usv_interfaces/msg/CaptureState`
- `/capture/roles`：`uav_usv_interfaces/msg/CaptureAssignmentArray`
- `/capture/target_status`：`uav_usv_interfaces/msg/CaptureTargetStatus`
- `/capture/markers`：RViz 动态围捕 Marker

旧 Qt 迁移期间保留 `_text`、`_json` 后缀的兼容话题，Qt 本阶段未重构。

## 启动

```bash
cd <你的工作区>/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py
```

USV 不可达测试：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  simulate_usv_02_unreachable:=true
```

关闭目标突然转向：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  enable_sudden_turn:=false
```

## Topic 验证

```bash
ros2 topic info /capture/state -v
ros2 topic echo /capture/state --once
ros2 topic echo /capture/roles --once
ros2 topic echo /capture/target_status --once
ros2 topic echo /fleet/vehicle_state
ros2 topic echo /fleet/command_ack
ros2 topic hz /capture/markers
```

## 运行结果

### 正常运行

- 四个 PX4 instance 均完成 DDS 连接、解锁、Offboard 起飞和位置跟踪。
- 两个 Nav2 实例均进入 active，USV 实际在 Gazebo 中运动。
- `capture_manager` 从 `SEARCH` 推进到 `SUCCESS`。
- 最终状态：`active_uavs=4`、`active_usvs=2`、`degraded=false`。
- 两艘 USV 的观测位移分别约为 `(-16, 2) -> (59.02, 39.12)` 和
  `(-16, 14) -> (77.42, 26.26)`。
- 日志目录：
  `/home/dji/.ros/log/2026-07-13-21-19-11-746106-dji-Legion-R7000-AHP9-73505`

### UAV 掉线

运行中终止 `uav_04_dds_agent` 后，状态更新为：

- `active_uavs=3`
- `active_usvs=2`
- `degraded=true`
- 分配代数由 2 更新为 3

`uav_04` 被移出任务，另外三架 UAV 获得互不重复的空中槽位，两艘 USV
继续保持水面截获角色。

### USV 不可达

以 `simulate_usv_02_unreachable:=true` 启动后，连续失败 ACK 触发隔离：

- `usv_02` 被标记为不可用并取消分配
- `active_uavs=4`
- `active_usvs=1`
- `degraded=true`
- `usv_01` 保持水面截获角色

日志目录：
`/home/dji/.ros/log/2026-07-13-21-25-20-085438-dji-Legion-R7000-AHP9-77064`

### 目标突然转向

动态目标节点在设定时刻将角速度切换至 `0.24 rad/s`，随后恢复正常航行。
`target_tracker` 估计航向和转弯率，CTRV 预测轨迹发生弯曲，分配器根据新预测点
持续更新围捕槽位；任务没有退回 Gazebo 直接控制模式。

## 已知问题

- 六载具同时启动对 CPU 和显卡负载较高，因此 launch 对 Nav2 和 PX4 采用错峰启动。
- Ctrl+C 集中关闭时，ROS 2 Humble 的 Python Nav2 ActionClient 偶尔在反馈对象拆除阶段
  报 `TypeError`；该问题发生在退出阶段，运行中的命令和反馈不受影响。
- 当前故障隔离属于任务进程生命周期内的临时隔离，尚未实现持久健康数据库。
- CTRV 输入来自 Gazebo 目标真值跟踪，后续接真实感知时需要补充测量协方差和滤波器。
