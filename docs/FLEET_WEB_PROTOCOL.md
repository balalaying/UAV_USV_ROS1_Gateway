# 舰队 Web 数据协议

## 当前定位

本阶段不开发正式 WebGL 网页，只冻结 ROS2 到网页端的数据接口。

未来网页端的主数据源是：

```text
/fleet/world_model
  -> uav_usv_fleet_gateway
  -> WebSocket
  -> WebGL岸基态势平台
```

网页端不应直接订阅相机、点云、LV-DOT debug、Gazebo pose 或单个控制节点话题。相机和点云属于传感器层，三维态势平台只接收已经转换到 `map` 坐标系的舰队世界模型、目标、载具、实体、任务和健康摘要。

## 连接

网关默认提供：

- HTTP 页面：`http://<ROS2电脑局域网IP>:8080`
- WebSocket：`ws://<ROS2电脑局域网IP>:8765/ws`
- 协议版本：`1.0`
- 数据方向：只读，网关只向浏览器发布监控数据

查询电脑地址：

```bash
hostname -I
```

只启动网关接口：

```bash
ros2 launch uav_usv_fleet_gateway mobile_fleet_demo.launch.py
```

常用接口参数：

```bash
ros2 launch uav_usv_fleet_gateway mobile_fleet_demo.launch.py \
  websocket_port:=8765 \
  http_port:=8080 \
  world_model_topic:=/fleet/world_model \
  world_model_summary_topic:=/fleet/world_model_summary \
  enable_world_model_publish:=true \
  world_model_publish_rate_hz:=2.0 \
  enable_world_model_summary_publish:=true \
  world_model_summary_publish_rate_hz:=1.0 \
  include_world_model_in_snapshot:=true \
  enable_legacy_topic_fallback:=false
```

`enable_legacy_topic_fallback` 仅用于旧系统兼容。面向最终岸基平台时应保持 `false`，避免网页端形成第二套状态源。

远距离/低带宽链路建议使用摘要优先模式：

```bash
ros2 launch uav_usv_fleet_gateway remote_summary_gateway.launch.py \
  world_model_summary_publish_rate_hz:=1.0
```

这个模式仍会保留兼容的载具、目标、传感器和诊断消息，但不会周期性发送完整 `fleet_world_model`，`fleet_snapshot.world_model` 也为空对象。它适合公网、4G/5G、卫星或不稳定链路。

## 通用信封

所有服务端文本消息均使用以下结构：

```json
{
  "schema_version": "1.0",
  "message_type": "fleet_snapshot",
  "timestamp": 1784692800.0,
  "sequence": 42,
  "source": "uav_usv_fleet_gateway",
  "data": {}
}
```

- `timestamp` 是网关生成信封的 Unix 秒数。
- `sequence` 是网关进程内全局递增序号，可用于发现丢包或乱序。
- ROS 原消息时间戳不会被信封时间覆盖，保存在具体对象的 `last_update`、`timestamp` 或 `stamp` 中。
- 所有未知、上游未提供或用负数表示不可用的值转换为 JSON `null`，不得用零伪造。
- 坐标数据保留原消息的 `frame_id`。当前正式载具和目标通常位于 `map`。

## 首次连接流程

1. 浏览器连接 `/ws`。
2. 网关发送 `gateway_hello`。
3. 网关立即发送 `fleet_snapshot`。
4. 后续自动发送增量状态和 1 Hz 完整快照，不需要客户端轮询。
5. 断线重连后重复上述流程，前端应按 ID 覆盖旧数据。

推荐重连间隔从 2 秒开始，并设置上限；当前测试页固定使用 2 秒。

默认实时频率：

- 每个已注册载具的 `vehicle_state`：10 Hz
- `perception_targets`：10 Hz
- `fleet_world_model`：2 Hz
- `fleet_world_model_summary`：1 Hz
- `fleet_snapshot`：1 Hz
- `gateway_diagnostics`：1 Hz
- `sensor_status`：1 Hz

频率由网关墙钟推送线程控制，不依赖 `/clock`。客户端应按 ID 覆盖本地对象，不应把每条状态追加成新的载具卡片。

## 消息类型

### gateway_hello

包含 `gateway_name`、`fleet_id`、`protocol_version`、`primary_data_source`、`communication_profile`、`use_sim_time`、`websocket_path` 和 `read_only`。客户端应检查 `read_only=true`。

当前 `primary_data_source` 固定为 `fleet_world_model`。

`communication_profile.mode` 用于区分通信档位：

- `local_full`：本地调试/内网演示，推完整 `fleet_world_model`。
- `remote_summary`：远距离低带宽链路，优先推 `fleet_world_model_summary`，不推完整 world model。
- `custom`：用户手动组合了开关和频率。

### fleet_snapshot

`data` 包含：

- `vehicles`：当前注册载具完整列表。
- `targets`：当前正式感知出口的目标列表。
- `sensors`：传感器健康摘要，不含图像和点云。
- `entities`：非控制载具但需要显示的任务实体，例如 `friendly_ship`、`enemy_ship`。
- `world_model`：最近一次完整舰队世界模型原文，schema 为 `fleet_world_model.v1`。
- `mission`：任务状态与当前感知源状态。
- `gateway`：网关诊断。

### fleet_world_model

完整舰队世界模型，直接转发 `/fleet/world_model` 的 JSON 内容。正式 WebGL 端优先使用这个消息构建三维态势，包括：

- `fleet.uav/usv/unknown`：所有无人机、无人船和未知载具。
- `entities`：保护船、敌船、指挥站等任务实体。
- `targets`：统一感知目标。
- `sensors`：传感器健康和频率摘要。
- `tf`：用于显示的 `map` 坐标系关系摘要。
- `mission`：任务状态。
- `perception`：当前感知源和融合状态。

兼容消息 `vehicle_state`、`perception_targets`、`sensor_status` 仍会保留，但默认由 `fleet_world_model` 派生，不再作为网页端的主入口。

### fleet_world_model_summary

轻量世界模型摘要，直接转发 `/fleet/world_model_summary` 的 JSON 内容。它是未来远距离公网/4G/5G/卫星链路的优先消息，因为它只包含统计、状态和健康摘要，不携带完整 TF、原始点云或视频。

典型字段：

- `schema_version`：固定为 `fleet_world_model.summary.v1`
- `world_model_schema_version`：对应完整模型版本，当前为 `fleet_world_model.v1`
- `uav_count`
- `usv_count`
- `entity_count`
- `target_count`
- `sensor_count`
- `mission_state`
- `primary_source`
- `tf_edge_count`

岸基平台可以用它做连接健康、任务状态、数据量估算和低带宽兜底显示；需要完整三维态势时再消费 `fleet_world_model`。

本地全量与远程摘要模式的启动方式、带宽边界和未来任务接口约定见
`docs/GATEWAY_DEPLOYMENT_INTERFACE.md`。

### vehicle_state

关键字段：

| 字段 | 含义 | 可为 null |
| --- | --- | --- |
| `id` | 载具唯一 ID，如 `uav_01` | 否 |
| `type` | `UAV`、`USV` 或 `UNKNOWN` | 否 |
| `namespace` | 由载具 ID 得到的逻辑命名空间 | 否 |
| `online` / `stale` | 上游在线状态与网关超时状态 | 否 |
| `position` | 原 `frame_id` 下的米制位置 | 各分量可用时否 |
| `orientation` | 欧拉角与原始四元数，弧度 | 是 |
| `linear_velocity` / `angular_velocity` | m/s 与 rad/s | 是 |
| `speed` | 三维线速度模长，m/s | 是 |
| `battery.percentage` | 百分比；当前代理多为不可用 | 是 |
| `battery.voltage` | 当前消息没有该字段 | 是，当前恒为 null |
| `mode` / `armed` | 模式与解锁状态 | 是 |
| `state_source` | 数据来源，常见为 `vehicle_state` 或 `tf_only` | 否 |

### perception_targets

`data.source` 表示该兼容消息由 `fleet_world_model` 派生，`selected_source` 表示世界模型内部当前采用的目标来源，例如 `ground_truth`、`fused_targets` 或未来 `sensor`。每个目标包含：

- `track_id`、`class_name`、`confidence`
- `timestamp`、`stamp.sec`、`stamp.nanosec`、`frame_id`
- `position`、`velocity`
- `bbox.length/width/height/yaw`
- `sensor_source`：例如 `lidar`、`camera`、`fusion`

LV-DOT Shadow 话题不会冒充正式控制目标。本 Demo 默认订阅 `/fleet/world_model`，只在 `enable_legacy_topic_fallback=true` 时才回退订阅旧的 mux 和状态话题。

### sensor_status

仅发布 `vehicle_id`、`sensor_id`、类型、在线状态、频率、消息时间、frame、状态、延迟、点数和丢包摘要。完整图像、点云不经过此 WebSocket。

### gateway_diagnostics

包含客户端数、注册/在线/陈旧载具数、目标数、ROS 接收数、WebSocket 发送数、队列丢弃数、运行时间、WebSocket 状态和 `communication_profile`。前端应优先用 `communication_profile.mode` 决定是否等待完整 `fleet_world_model`。

### pong / error

客户端可发送：

```json
{"command":"ping"}
```

或：

```json
{"command":"request_snapshot"}
```

其他未知命令返回 `unsupported_command`。当前不存在解锁、导航、参数、脚本或文件接口。

## 未来任务命令接口占位

本阶段网关仍然是只读接口，不发布 `/fleet/command`，不申请 `/fleet/control_lease`，不改变 PX4/Nav2/capture_manager。为了让未来 WebGL 岸基平台可以先按稳定格式开发，WebSocket 已经识别以下命令：

- `submit_task`
- `cancel_task`
- `emergency_stop`

在命令接口未启用前，服务器返回：

```json
{
  "schema_version": "1.0",
  "message_type": "command_response",
  "timestamp": 1784692800.0,
  "sequence": 43,
  "source": "uav_usv_fleet_gateway",
  "data": {
    "code": "command_interface_disabled",
    "message": "Mission commands are recognized but disabled in this read-only gateway.",
    "accepted_commands": ["submit_task", "cancel_task", "emergency_stop"]
  }
}
```

未来启用控制时，链路必须是：

```text
WebGL任务请求
  -> uav_usv_fleet_gateway
  -> Mission/Behavior Manager
  -> /fleet/control_lease
  -> /fleet/command
  -> UAV/USV agent
  -> PX4 / Nav2
  -> /fleet/command_ack
  -> /fleet/world_model
  -> WebGL
```

禁止 WebGL 或 Gateway 直接发布 PX4 topic、Nav2 action、Gazebo `cmd_vel` 或单个传感器话题。

建议的 `submit_task` 请求格式：

```json
{
  "command": "submit_task",
  "request_id": "web-20260727-0001",
  "operator_id": "shore_operator",
  "task": {
    "task_id": "task-capture-enemy-001",
    "task_type": "capture",
    "target_id": "enemy_ship",
    "priority": 120,
    "constraints": {
      "allowed_vehicles": ["uav_01", "uav_02", "usv_01", "usv_02"],
      "max_duration_sec": 300.0
    }
  }
}
```

建议的 `cancel_task` 请求格式：

```json
{
  "command": "cancel_task",
  "request_id": "web-20260727-0002",
  "operator_id": "shore_operator",
  "task_id": "task-capture-enemy-001"
}
```

建议的 `emergency_stop` 请求格式：

```json
{
  "command": "emergency_stop",
  "request_id": "web-20260727-0003",
  "operator_id": "shore_operator",
  "scope": {
    "vehicle_ids": ["usv_01"],
    "all": false
  }
}
```

这些字段是 Web 到任务层的意图表达，不等同于 `FleetCommand.msg`。真正转换到 `FleetCommand` 时必须由服务端任务/行为管理器生成 `command_id`、`lease_id`、`expires_at` 和具体 `target_pose`。

## JavaScript 最小接入

```javascript
const ws = new WebSocket("ws://192.168.1.100:8765/ws");

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  switch (message.message_type) {
    case "fleet_world_model":
      updateWorldModel(message.data);
      break;
    case "fleet_snapshot":
      updateFleet(message.data);
      break;
    case "vehicle_state":
      updateVehicle(message.data);
      break;
    case "perception_targets":
      updateTargets(message.data.targets);
      break;
    case "gateway_diagnostics":
      updateDiagnostics(message.data);
      break;
  }
};
```

Vue 中建议在 Pinia/store 内以 `fleet_world_model` 作为唯一主状态，按 `vehicle.id`、`entity.id` 和 `target.id` 保存字典，组件只读取 store；不要把 WebSocket 放进每张卡片。Three.js 中可直接用 `pose.position.x/y/z`，绕 Z 轴可从 `pose.orientation` 换算 yaw；正式网页应假设输入已是 `map`，不在前端做 TF 查询。

正式 WebGL 页面建议优先使用：

- `fleet.uav` / `fleet.usv`：绘制无人机和无人船。
- `entities`：绘制保护船、敌船、基地、任务实体。
- `targets`：绘制目标框、身份、速度箭头和轨迹。
- `sensors`：显示平台传感器在线状态。
- `mission`：显示当前任务状态。
- `perception`：显示感知来源和 Shadow 状态。

正式 WebGL 页面不应使用：

- 原始相机图像。
- 原始 PointCloud2。
- `/tf` 或 `/tf_static`。
- Gazebo 内部 pose。
- LV-DOT debug topic。

## 网页替换与兼容策略

正式前端成员只需替换：

```text
src/uav_usv_fleet_gateway/web/
```

保留根路径入口或同步修改 HTTP 静态入口即可。ROS 订阅、注册表、协议和 WebSocket 服务不需要改变。

`schema_version` 遵循主次版本策略：新增可选字段提升次版本；删除字段、改类型或改语义提升主版本。客户端应忽略未知字段，并把允许为 `null` 的字段作为正常状态处理。
