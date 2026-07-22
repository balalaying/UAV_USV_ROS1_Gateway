# 舰队 Web 数据协议

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
- `fleet_snapshot`：1 Hz
- `gateway_diagnostics`：1 Hz
- `sensor_status`：1 Hz

频率由网关墙钟推送线程控制，不依赖 `/clock`。客户端应按 ID 覆盖本地对象，不应把每条状态追加成新的载具卡片。

## 消息类型

### gateway_hello

包含 `gateway_name`、`fleet_id`、`protocol_version`、`use_sim_time`、`websocket_path` 和 `read_only`。客户端应检查 `read_only=true`。

### fleet_snapshot

`data` 包含：

- `vehicles`：当前注册载具完整列表。
- `targets`：当前正式感知出口的目标列表。
- `sensors`：传感器健康摘要，不含图像和点云。
- `mission`：任务状态与当前感知源状态。
- `gateway`：网关诊断。

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

### perception_targets

`data.source` 固定标识正式出口 `source_mux`，`selected_source` 表示 mux 当前选择。每个目标包含：

- `track_id`、`class_name`、`confidence`
- `timestamp`、`stamp.sec`、`stamp.nanosec`、`frame_id`
- `position`、`velocity`
- `bbox.length/width/height/yaw`
- `sensor_source`：例如 `lidar`、`camera`、`fusion`

LV-DOT Shadow 话题不会冒充 `/fleet/perception/targets`。本 Demo 只订阅 mux 正式出口。

### sensor_status

仅发布 `vehicle_id`、`sensor_id`、类型、在线状态、频率、消息时间、frame、状态、延迟、点数和丢包摘要。完整图像、点云不经过此 WebSocket。

### gateway_diagnostics

包含客户端数、注册/在线/陈旧载具数、目标数、ROS 接收数、WebSocket 发送数、队列丢弃数、运行时间和 WebSocket 状态。

### pong / error

客户端可发送：

```json
{"command":"ping"}
```

或：

```json
{"command":"request_snapshot"}
```

其他命令返回 `unsupported_command`。不存在解锁、导航、任务、参数、脚本或文件接口。

## JavaScript 最小接入

```javascript
const ws = new WebSocket("ws://192.168.1.100:8765/ws");

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  switch (message.message_type) {
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

Vue 中建议在 Pinia/store 内按 `vehicle.id` 和 `target.track_id` 保存字典，组件只读取 store；不要把 WebSocket 放进每张卡片。Three.js 中可直接用 `position.x/y/z`，绕 Z 轴使用 `orientation.yaw`；必须先确认 `frame_id` 与场景坐标一致。

## 网页替换与兼容策略

正式前端成员只需替换：

```text
src/uav_usv_fleet_gateway/web/
```

保留根路径入口或同步修改 HTTP 静态入口即可。ROS 订阅、注册表、协议和 WebSocket 服务不需要改变。

`schema_version` 遵循主次版本策略：新增可选字段提升次版本；删除字段、改类型或改语义提升主版本。客户端应忽略未知字段，并把允许为 `null` 的字段作为正常状态处理。
