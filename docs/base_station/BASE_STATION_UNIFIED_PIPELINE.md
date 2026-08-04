# 岸基统一态势数据闭环

## 目标

本模块是只读的岸基数据边界：

```text
传感器 -> 感知 -> 舰队感知融合 -> Fleet World Model
       -> Base Station Service -> Qt / Gateway / 后续 WebGL
```

它不修改 LV-DOT、相机-激光融合、`perception_source_mux`、PX4、Nav2、
FleetCommand、`capture_manager` 或舰队的 `map` TF 体系。

## Base Station 配置

`uav_usv_base_station/config/base_station.yaml` 是岸基参考配置的唯一来源。
默认以 `map` 原点作为基站：

```yaml
base_station_id: base_station
map_frame: map
base_station_x: 0.0
base_station_y: 0.0
base_station_z: 0.0
base_station_yaw: 0.0
radar_display_range_m: 300.0
```

部署时可通过已有 ROS 参数改为 `heterogeneous_332.sdf` 中
`shore_command_base` 指挥站房屋的位置，不需要修改 Qt 或 Gateway 代码。
Base Station 只是 `map` 中的参考实体，不会创建或替代新的世界坐标系。

## 状态契约

`/base_station/state` 使用 `std_msgs/String` 承载 JSON，schema 为
`base_station_service.v1`。其内容包括：

- `timestamp`、`source_world_time`、`received_at_wall_time`、`map_frame`；
- `base_station`：配置、通信状态、已连接/在线载具和任务状态；
- `fleet`、`entities`、`targets`、`predictions`、`threats`、`target_history`；
- `perception`、`sensors`、`communication`、`health`、`recent_events`。

每个客户端目标都包含 `target_id`、`map` 坐标位置、速度、航向、尺寸、类别、
身份、置信度、来源、`source_mask`、最后更新时间、数据年龄、
`is_ground_truth_fallback` 和 `is_real_fusion`。因此显示端不会把控制用的
ground-truth 回退目标错误标成真实传感器融合结果。

## 岸基雷达坐标

所有实体的真实位置仍使用 `map` 坐标。雷达页面只在显示时计算相对岸基量：

```text
dx = target.map.x - base_station.map.x
dy = target.map.y - base_station.map.y
distance = sqrt(dx^2 + dy^2)
bearing = normalize_degrees(atan2(dy, dx) - base_station.yaw)
```

约定：`map` 的北向是 `+Y`，东向是 `+X`；Qt 雷达图据此绘制 N/E/S/W。
目标详情中的 `0°` 指向配置的基站航向，同时保留绝对 `map` 坐标、相对距离和
相对方位角。

## Qt 数据流

当 `use_base_station_service:=true` 时，Fleet Situation View 只订阅：

- `/base_station/state`
- `/base_station/events`

该业务态势页不会订阅 World Model、融合目标、TF、点云、相机或 LV-DOT 调试话题。
Perception Monitor 仍作为独立的底层传感器调试页面保留。

态势雷达支持网格、距离环、覆盖范围、历史轨迹、预测轨迹、威胁圈和标签。点击目标后
可查看 `map` 坐标、岸基相对距离/方位、置信度及来源/回退状态。

## Gateway 与 Web 契约

Gateway 默认订阅 `/base_station/state` 与 `/base_station/events`，只缓存和转发
服务输出，不重新计算目标历史、威胁、在线状态或基站位置。

稳定的只读 WebSocket 消息：

| 消息类型 | 内容 |
|---|---|
| `base_station_snapshot` | 每秒一次，直接封装 `base_station_service.v1` 快照。 |
| `base_station_event` | 事件发生时立即发送，直接封装 `base_station_event.v1`。 |
| `gateway_diagnostics` | Gateway 健康状态、客户端数量和缓存可用性。 |
| `command_ack_reserved` | 预留给未来只读命令回执展示。 |

`fleet_snapshot` 与 World Model 消息仍作为兼容接口保留。后续 WebGL 应优先使用
`base_station_snapshot`，不应解析 ROS TF。

## 事件规则

Base Station Service 默认缓存最近 100 条事件。事件类型包括：

- `target_appeared`、`target_lost`；
- `threat_changed`；
- `vehicle_online`、`vehicle_offline`；
- `sensor_online`、`sensor_offline`；
- `mission_changed`。

每条事件都有时间戳、实体 ID，且在 payload 中保存旧值/新值。同一状态连续不变时
不会重复产生事件。

## 安全边界

Base Station Service、Qt 舰队态势页和 Gateway 均为只读模块，不发布 FleetCommand、
Nav2 目标、PX4 Topic 或感知 Observation。`map` 仍是唯一舰队世界坐标系，
`perception_source=ground_truth` 仍为默认 Shadow 模式控制来源。
