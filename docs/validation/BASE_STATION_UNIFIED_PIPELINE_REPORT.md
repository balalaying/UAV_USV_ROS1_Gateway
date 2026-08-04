# 岸基统一态势数据闭环验证报告

## 修改文件

- `src/uav_usv_base_station/config/base_station.yaml`
- `src/uav_usv_base_station/launch/base_station_service.launch.py`
- `src/uav_usv_base_station/uav_usv_base_station/base_station_service_node.py`
- `src/uav_usv_base_station/uav_usv_base_station/service_cache.py`
- `src/uav_usv_mission/scripts/fleet_base_station_gui.py`
- `src/uav_usv_bringup/launch/dynamic_capture_console.launch.py`
- `src/uav_usv_fleet_gateway/config/fleet_gateway.yaml`
- `src/uav_usv_fleet_gateway/launch/remote_summary_gateway.launch.py`
- `src/uav_usv_fleet_gateway/uav_usv_fleet_gateway/gateway_node.py`

## 新增文件

- `docs/base_station/BASE_STATION_UNIFIED_PIPELINE.md`
- 本验证报告

## 实现结果

- Base Station 默认以 `map` 原点为参考，位置可由配置或启动参数修改。
- `/base_station/state` 以 5 Hz 输出只读状态快照。
- Qt 的 Fleet Situation View 在服务模式下只读取 Base Station State/Event；
  Perception Monitor 仍用于底层传感器调试。
- Gateway 默认读取 Base Station Service，发送 `base_station_snapshot` 以及
  实时 `base_station_event`；设置 `enable_base_station_service:=false` 后仍可
  使用旧的 Fleet World Model 兼容输入。

## 验证矩阵

| 项目 | 结果 | 方法 |
|---|---|---|
| Python 语法 | 通过 | 编译修改的 Python 与 launch 文件。 |
| Base Station 契约 | 通过 | 检查 `map` frame、目标来源、回退标记和事件实体 ID。 |
| 基站原点 | 通过 | 默认配置为 `(0, 0, 0)`、yaw 为 `0`。 |
| 非原点参考 | 通过（契约测试） | 缓存正确接收 `(10, -5, 3)` 和 yaw `0.5`，客户端无坐标硬编码。 |
| 332 主入口 | 通过（控制安全模式） | 关闭 PX4/DDS、Qt、LV-DOT、相机-激光融合后启动服务与舰队主链。 |
| World Model / Service 频率 | 通过 | `/fleet/world_model` 为 5.01 Hz；`/base_station/state` 为 4.99-5.00 Hz。 |
| Qt / Gateway | 通过 | Qt 订阅 `/base_station/state` 而不订阅 `/fleet/world_model`；Gateway 成功收到服务状态并通过 WebSocket 输出 `base_station_snapshot`。 |
| 事件 | 通过（缓存契约测试） | 目标、威胁、任务、载具和传感器状态变化均产生去重事件，且含实体 ID、旧值/新值。 |

## 已知限制

本阶段不下发网页命令、不修改行为管理器，也不将 `perception_source` 从
`ground_truth` 切换到真实感知。完整 PX4 飞行不属于只读态势链验证范围。

Base Station 的首次事件为易失消息，晚于事件启动的 `ros2 topic echo` 不会回放该条消息。
持久事件历史会保存在每个 `/base_station/state.recent_events` 快照中；Gateway 会将后续
实时事件立即转发为 `base_station_event`。
