# Fleet Gateway 部署与接口约定

## 当前边界

当前阶段不实现正式 WebGL 页面，只冻结机器人端到岸基端的数据接口。

唯一权威数据链为：

```text
标准化感知与载具状态
        |
        v
/fleet/world_model
        |
        v
uav_usv_fleet_gateway
        |
        v
WebSocket JSON
        |
        v
未来岸基服务器 / WebGL
```

Gateway 不订阅原始相机、PointCloud2、Gazebo pose 或 LV-DOT debug
话题。远端也不需要 ROS 2 DDS，不需要解析 TF；所有正式位置已经转换到
`map`。

## 两种部署模式

### 本地全量模式

适用于同一台电脑或局域网内的调试显示：

```bash
ros2 launch uav_usv_fleet_gateway mobile_fleet_demo.launch.py \
  bind_address:=0.0.0.0 \
  websocket_port:=8765
```

该模式发送：

- `fleet_world_model`：完整世界模型，默认 2 Hz；
- `fleet_world_model_summary`：摘要，默认 1 Hz；
- `vehicle_state`：从世界模型派生的平台状态，默认 10 Hz；
- `perception_targets`：从世界模型派生的目标状态，默认 10 Hz；
- `sensor_status`：传感器健康状态，默认 1 Hz；
- `fleet_snapshot`：恢复快照，默认 1 Hz；
- `gateway_diagnostics`：Gateway诊断，默认 1 Hz。

### 远程摘要模式

适用于未来 4G、5G、卫星或公网岸基链路：

```bash
ros2 launch uav_usv_fleet_gateway remote_summary_gateway.launch.py \
  bind_address:=0.0.0.0 \
  websocket_port:=8765 \
  world_model_summary_publish_rate_hz:=1.0
```

该模式具有以下强制限制：

- 不发送完整 `fleet_world_model`；
- `fleet_snapshot.world_model` 为空；
- 发送 `fleet_world_model_summary`；
- 继续发送从世界模型派生的平台、目标和告警状态；
- 不启动内置 HTTP 测试页面；
- 不启用旧零散 ROS topic 回退；
- 不传输原始视频、点云或完整 TF。

这不是最终公网安全方案。真实部署时仍需在 Gateway 外增加 TLS、身份认证、
授权、审计和链路重连策略。

## WebSocket连接

默认地址：

```text
ws://<机器人或岸基服务器IP>:8765/ws
```

每条服务器消息统一使用以下信封：

```json
{
  "schema_version": "1.0",
  "message_type": "fleet_world_model_summary",
  "timestamp": 1785128400.0,
  "sequence": 84,
  "source": "uav_usv_fleet_gateway",
  "data": {}
}
```

字段约定：

| 字段 | 含义 |
| --- | --- |
| `schema_version` | Gateway传输协议版本 |
| `message_type` | 消息类型 |
| `timestamp` | Gateway发送时的Unix秒 |
| `sequence` | 单个Gateway进程内递增序号 |
| `source` | Gateway实例名 |
| `data` | 对应消息负载 |

`sequence` 可用于发现岸基接收端丢包，但重启 Gateway 后会重新计数。

## 首次连接

WebSocket握手完成后，服务端依次主动发送：

1. `gateway_hello`
2. `fleet_snapshot`

`gateway_hello.data.communication_profile.mode` 用于识别当前链路模式：

| 值 | 含义 |
| --- | --- |
| `local_full` | 发送完整世界模型，快照也包含完整模型 |
| `remote_summary` | 不发送完整模型，快照不含完整模型 |
| `custom` | 用户自定义的混合配置 |

未来前端应先读取该字段，再决定是否等待完整世界模型。

## 正式消息来源

| WebSocket消息 | ROS 2权威来源 | 用途 |
| --- | --- | --- |
| `fleet_world_model` | `/fleet/world_model` | 本地完整态势 |
| `fleet_world_model_summary` | `/fleet/world_model_summary` | 远程低带宽摘要 |
| `vehicle_state` | `/fleet/world_model`中的`fleet` | 平台位置、速度、模式、在线状态 |
| `perception_targets` | `/fleet/world_model`中的`targets` | 世界坐标目标 |
| `sensor_status` | `/fleet/world_model`中的`sensors` | 传感器在线、频率、延迟 |
| `fleet_snapshot` | Gateway缓存 | 新客户端恢复当前状态 |
| `gateway_diagnostics` | Gateway内部统计 | 客户端、丢包、链路模式 |

默认 `enable_legacy_topic_fallback=false`，因此这些 WebSocket 消息不会从旧的
零散 ROS topic 构造。只有排查兼容问题时才允许临时打开回退模式。

## Schema版本

完整世界模型：

```text
fleet_world_model.v1
```

轻量摘要：

```text
fleet_world_model.summary.v1
```

摘要同时携带：

```json
{
  "world_model_schema_version": "fleet_world_model.v1"
}
```

这使岸基端能分别判断传输摘要和完整世界模型是否兼容。

完整示例：

- `src/uav_usv_fleet_gateway/web/protocol-example.json`
- `src/uav_usv_fleet_gateway/web/world-model-summary-example.json`
- `src/uav_usv_fleet_gateway/web/command-request-example.json`

## 未来任务指令接口

当前 Gateway 为只读模式。协议已经保留下列命令名称：

- `submit_task`
- `cancel_task`
- `emergency_stop`

当前发送这些命令会收到：

```json
{
  "message_type": "command_response",
  "data": {
    "code": "command_interface_disabled"
  }
}
```

后续启用控制时，必须经过岸基鉴权、ControlLease、Mission/Behavior Manager 和
`FleetCommand`，不能让 WebSocket 直接发布 PX4、Nav2 或 Gazebo 控制话题。

## 验证命令

确认世界模型存在：

```bash
ros2 topic hz /fleet/world_model
ros2 topic hz /fleet/world_model_summary
```

检查Gateway参数：

```bash
ros2 param get /fleet_gateway enable_world_model_publish
ros2 param get /fleet_gateway include_world_model_in_snapshot
ros2 param get /fleet_gateway enable_legacy_topic_fallback
```

远程摘要模式的预期值：

```text
enable_world_model_publish: false
include_world_model_in_snapshot: false
enable_legacy_topic_fallback: false
```

接口测试：

```bash
python3 -m pytest -q src/uav_usv_fleet_gateway/test
```

## 后续WebGL接入规则

未来 WebGL 只消费 Gateway 的版本化 JSON：

- 平台、实体、目标的坐标都必须是 `map`；
- 前端不查询TF；
- 前端不直接连接ROS 2；
- 前端不根据原始传感器重新计算目标；
- 前端状态以 `fleet_world_model` 或远程摘要流为准；
- 任务按钮只能调用经过鉴权的任务接口，不能直接控制载具。
