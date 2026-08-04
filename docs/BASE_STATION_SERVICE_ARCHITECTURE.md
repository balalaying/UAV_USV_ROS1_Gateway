# Base Station Service 架构

## 目的

`uav_usv_base_station` 提供一个只读的岸基服务层。它位于 Fleet World
Model 与 Qt、WebGL、远程面板、任务规划器、日志和回放系统之间，避免每个
客户端自行订阅、缓存和解释 ROS 2 数据。

```text
Fleet World Model (/fleet/world_model)
                 |
                 v
       Base Station Service (read-only)
          |                    |
          |                    +-- /base_station/events
          v
       /base_station/state
          |
          +-- Qt client (future service-only migration)
          +-- WebSocket / WebGL gateway
          +-- Remote Dashboard
          +-- Mission Planner (read-only situation input)
          +-- Log / replay recorder
```

服务不订阅原始相机、点云、LV-DOT 调试、PX4 或控制话题；不会发布
`FleetCommand`，也不改写 `/fleet/world_model`。因此它不能改变围捕、导航或
飞控行为。

## 模块关系

| 模块 | 职责 | 边界 |
| --- | --- | --- |
| `fleet_world_model` | 生成权威舰队状态 | 仍是唯一上游真值接口 |
| `base_station_service` | 缓存、事件、显示轨迹、客户端快照 | 只读，不回写上游 |
| Qt / WebGL | 展示与交互客户端 | 后续从服务读状态，不读底层传感器 |
| Gateway | 传输适配器 | 后续转发`/base_station/state`，不承担状态计算 |

## ROS 接口

| 方向 | Topic | 类型 | 默认频率 | 用途 |
| --- | --- | --- | --- | --- |
| 输入 | `/fleet/world_model` | `std_msgs/msg/String` | 上游约5 Hz | 原始权威世界模型 JSON |
| 输出 | `/base_station/state` | `std_msgs/msg/String` | 5 Hz | 面向所有客户端的状态快照 |
| 输出 | `/base_station/events` | `std_msgs/msg/String` | 事件驱动 | 目标、威胁、任务、在线状态变化 |

`/base_station/state` 使用 `base_station_service.v1` JSON。它保留 World Model
中的 `fleet`、`entities`、`targets`、`predictions`、`threats`、`mission`、
`perception`、`sensors`、`communication`、`tf`、`environment` 和 `health`，
并新增服务自身字段：

- `base_station`：岸基 ID、`map`坐标位置、通信状态、已连接/在线载具、任务状态；
- `target_history`：每个目标的显示用位置、航向、速度、时间历史；
- `recent_events`：最近事件列表；
- `source`：上游 schema 和只读标记。

这不是对 World Model schema 的改动，而是独立的下游服务契约。

## 缓存与时间

1. 收到一个 World Model JSON 后，服务深拷贝保存，绝不修改原对象。
2. 目标历史优先使用目标自身时间戳，缺失时使用 `world_time`，最后才使用本机
   接收时刻。默认每个目标保存 120 个变化点，仅服务进程内存保存。
3. `/base_station/state` 使用 transient-local QoS；新客户端连接时可立即获得最近
   一份完整态势，不必等待下一帧。
4. 当前状态同时保留 `source_world_time` 与 `received_at_wall_time`，以便客户端
   区分仿真时间、源数据时间和岸基接收时间。

## 事件机制

事件只表示上游状态的变化，服务不会产生控制决策：

- `target_appeared` / `target_lost`
- `threat_changed`
- `mission_changed`
- `vehicle_online` / `vehicle_offline`
- `sensor_online` / `sensor_offline`

每个事件采用 `base_station_event.v1`，包含 `event_type`、`severity`、`timestamp`
和 `payload`。事件也会保留在状态快照的 `recent_events` 内，便于重连客户端恢复
最近上下文。

## Base Station 对象

Base Station 属于服务内部对象，不添加 TF，也不改写 World Model：

```json
{
  "id": "base_station",
  "frame_id": "map",
  "position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "communication_status": "LOCAL_SIMULATION",
  "connected_vehicle_ids": ["uav_01", "usv_01"],
  "online_vehicle_ids": ["uav_01", "usv_01"],
  "mission_status": "SEARCH"
}
```

默认配置在`uav_usv_base_station/config/base_station.yaml`：332场景使用岛屿起飞
平台上方的`shore_command_base`，其`map`位置为`(-35.0, -190.0, 17.5)`。此对象
为未来远程岸基、基站故障诊断和多基站切换提供稳定入口。

## 启动

完整演示默认启动服务：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

如需关闭：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py \
  enable_base_station_service:=false
```

也可单独启动：

```bash
ros2 launch uav_usv_base_station base_station_service.launch.py
```

验证：

```bash
ros2 topic echo /base_station/state --once
ros2 topic echo /base_station/events
ros2 topic hz /base_station/state
```

## WebGL 与公网部署预留

未来通信适配应固定为：

```text
/base_station/state + /base_station/events
      -> communication gateway
      -> authenticated WebSocket / MQTT / HTTPS
      -> WebGL / remote dashboard
```

公网部署时，Gateway 只负责认证、限流、断线重连、压缩与协议转换；所有状态缓存、
事件和轨迹都仍由 Base Station Service 负责。这样 4G/5G、VPN、云中继或异地
岸基不会改变感知、控制和 World Model 的 ROS 2 主线。

## 兼容性

本阶段未修改以下模块或接口：

- LV-DOT、Camera-LiDAR Fusion、Fleet Perception Fusion；
- Fleet World Model 与 `/fleet/world_model`；
- PX4、Nav2、UAV/USV agent；
- `capture_manager`、`FleetCommand`、TF；
- 现有 Qt 页面。

Qt 当前仍可作为已有 World Model 客户端使用。后续只需把它的一个输入替换为
`/base_station/state`，无需再触碰传感器或控制链路。
