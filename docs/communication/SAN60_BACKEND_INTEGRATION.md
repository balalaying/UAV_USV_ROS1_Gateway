# SAN60 后端联调说明

## 1. 范围与当前链路

当前已实现的 JSON 实时链路为：

```text
/san60/spectrum
  -> SensorStreamAdapter（约 100 Hz 限流到约 10 Hz）
  -> /fleet/gateway/sensor_stream
  -> FleetGatewayNode
  -> LatestValueStore
  -> WebSocket /ws
  -> Backend
```

本说明只描述 Gateway 的 JSON WebSocket `/ws`。当前本地默认地址为：

```text
ws://<gateway-ip>:8765/ws
```

本机验证地址为：

```text
ws://127.0.0.1:8765/ws
```

正式部署地址由后端/部署配置决定。

连接成功后 Gateway 会先发送 `gateway_hello` 和 `fleet_snapshot`，之后还会发送
其他舰队消息。客户端必须按顶层 `message_type` 筛选 SAN60 数据。

## 2. 消息类型与数据源标识

```ini
message_type = spectrum_frame
vehicle_id = uav_01
sensor_id = san60
stream_id = uav_01_san60
```

上述数据源标识由 Gateway launch 参数配置。后端必须按收到的字段处理，不应写死
只支持 `uav_01`。未来增加 SAN60 或调整载机时，可以使用新的 `vehicle_id` 和
`stream_id`，无需改变 envelope。

## 3. WebSocket `spectrum_frame` 示例

当前实际 Gateway envelope 如下。为避免在文档中展开 1641 个数值，示例
`powers_dbm` 只列出三个点；线上该字段是包含 1641 个 JSON number 的数组，
不会包含省略号字符串。

```json
{
  "schema_version": "1.0",
  "message_type": "spectrum_frame",
  "timestamp": 1790228140.25,
  "sequence": 12345,
  "source": "uav_usv_fleet_gateway",
  "data": {
    "vehicle_id": "uav_01",
    "sensor_id": "san60",
    "stream_id": "uav_01_san60",
    "type": "spectrum",
    "captured_at": 1790228140.14739,
    "start_hz": 2399948790.93684,
    "stop_hz": 2500046445.56842,
    "bin_hz": 61035.1552631579,
    "rbw_hz": 100000,
    "ref_level_dbm": 0,
    "temperature_c": 39.07,
    "peak_hz": 2461289121.97632,
    "peak_dbm": -56.7138519287109,
    "powers_dbm": [-91.25, -90.88, -89.74],
    "sequence": 38423
  }
}
```

真实层级注意事项：

- `message_type` 位于 envelope 顶层；
- SAN60 数据和数据源标识直接位于 `data`；
- 没有额外的 `payload` 层；
- `timestamp` 是 Gateway 生成 envelope 时的 Unix 秒；
- `captured_at` 来自 SAN60 原始帧；
- Adapter 不修改 `powers_dbm`，不生成完整 frequency 数组。

## 4. 两个 sequence 的含义

### `envelope.sequence`（顶层 `sequence`）

Gateway 进程内的全局消息序号。所有通过该 JSON WebSocket encoder 的消息共同
递增，不只包含 SAN60；Gateway 重启后重新计数。它用于观察 Gateway WebSocket
消息流，不等于 SAN60 帧号。

### `data.sequence`（`data` 内层 `sequence`）

SAN60 原始采集序号，由采集端生成并由 Adapter 原样保留。SAN60 ROS 输入约
100 Hz，而 Adapter 主动只取约 10 Hz，因此后端可能看到：

```text
100, 110, 120, 130, ...
```

这属于主动降采样，不等于网络丢包。后端不得仅凭 `data.sequence` 跳号判定
Gateway 丢包，也不得重新生成或覆盖该序号。

## 5. 频谱坐标恢复

不传输完整 frequency 数组。对于 `powers_dbm` 中第 `i` 个点：

```text
frequency[i] = start_hz + i * bin_hz
i = 0 ... len(powers_dbm) - 1
```

显示为 MHz：

```text
frequency_mhz[i] = frequency[i] / 1e6
```

Y 轴：

```text
powers_dbm[i]
```

Y 轴单位为 `dBm`。由于浮点步长和设备端边界定义可能存在舍入差异，显示端应以
`start_hz + i * bin_hz` 为准，不应通过 `stop_hz` 再平均生成步长。

## 6. Backend 推荐处理

收到 `message_type == "spectrum_frame"` 后：

1. 确认 `data` 是对象，并校验 `data.stream_id`；
2. 确认 `data.powers_dbm` 是非空数组；
3. 使用 `vehicle_id + stream_id` 建立或查找传感器实例；
4. 每个 stream 只缓存 latest frame，新帧覆盖旧帧；
5. 不建立无界历史队列；
6. 不重新采样、压缩或修改 `powers_dbm`；
7. 不重新计算或覆盖 SAN60 原始 `data.sequence`；
8. 可将完整 `data` 或完整 envelope 原样转交 Unity；
9. Unity 实时显示建议限制在 10–20 Hz，并始终消费最新帧；
10. 如需历史频谱，应单独设计有容量/保留期的数据存储，不得让实时链路无限积压。

建议缓存键使用结构化组合，例如：

```text
(vehicle_id, stream_id)
```

不要只以 `sensor_id` 建键，因为未来可能有多架载机各自使用 `san60`。

## 7. Unity 首版最低字段

必须转交：

```text
vehicle_id
sensor_id
stream_id
start_hz
stop_hz
bin_hz
rbw_hz
temperature_c
peak_hz
peak_dbm
powers_dbm
sequence
```

建议同时保留但首版显示非必需：

```text
captured_at
ref_level_dbm
```

Unity 首版只需恢复频率轴并显示实时曲线、频段、RBW、温度和峰值；无需 Start、
Stop、参数下发、Marker、Waterfall、IQ 或历史保存。

## 8. 本地 WebSocket 验证客户端

仓库工具：

```text
tools/test_san60_gateway_ws.py
```

它复用仓库现有的无第三方依赖 `WebSocketClient`，只打印
`message_type == "spectrum_frame"` 的摘要，不展开 `powers_dbm`。

```bash
cd /home/liu/uav_usv_ros1_ws/src/UAV_USV
python3 tools/test_san60_gateway_ws.py \
  --url ws://127.0.0.1:8765/ws \
  --count 10
```

持续观察直到 `Ctrl-C`：

```bash
python3 tools/test_san60_gateway_ws.py \
  --url ws://127.0.0.1:8765/ws
```

典型摘要：

```text
spectrum_frame gateway_seq=1234 san60_seq=880 vehicle=uav_01 sensor=san60 stream=uav_01_san60
  band=2399.949-2500.046 MHz bin=61.035 kHz RBW=100.000 kHz
  Temp=39.1 C Peak=2461.289 MHz / -56.7 dBm points=1641
```

## 9. 后端联调验收 checklist

```text
[ ] 后端可以连接/接收 Gateway WebSocket 数据
[ ] 能识别 spectrum_frame
[ ] vehicle_id 正确
[ ] sensor_id 正确
[ ] stream_id 正确
[ ] powers_dbm 是数值数组且数量为 1641
[ ] start_hz / stop_hz / bin_hz 正确
[ ] peak_hz / peak_dbm 正确
[ ] temperature_c 正确
[ ] 后端按 vehicle_id + stream_id 缓存 latest frame
[ ] 后端没有建立无界频谱队列
[ ] 不因 SAN60 data.sequence 主动跳号误报丢包
[ ] Gateway 顶层 sequence 与 SAN60 内层 sequence 未混用
[ ] Unity 可以收到 spectrum_frame
[ ] Unity 能按 start_hz + i * bin_hz 恢复 1641 点频率轴
[ ] Unity 实时频谱显示稳定（建议 10-20 Hz）
[ ] 六路相机同时运行正常
[ ] 长时间运行无持续内存增长或队列积压
```

## 10. 联调期间本地监控命令

ROS 输入和 Adapter 聚合输出频率：

```bash
source /opt/ros/noetic/setup.bash
source /home/liu/uav_usv_ros1_ws/devel/setup.bash
rostopic hz /san60/spectrum
```

```bash
rostopic hz /fleet/gateway/sensor_stream
```

注意：`/fleet/gateway/sensor_stream` 同时承载 SAN60、相机和其他传感器帧，
`rostopic hz` 显示的是聚合频率，不是 SAN60 单流频率。

只确认 ROS 聚合 topic 中出现 `spectrum_frame`，不打印 1641 点数组：

```bash
rostopic echo /fleet/gateway/sensor_stream \
  | grep --line-buffered -o '"message_type":"spectrum_frame"'
```

建议使用前述 WebSocket 客户端观察 SAN60 专用摘要和真实 Gateway sequence。

节点、进程和监听端口：

```bash
rosnode list | grep -E 'fleet_gateway|fleet_sensor_stream_adapter'
pgrep -af 'fleet_gateway|fleet_sensor_stream_adapter|spectrum_receiver|harogic_sender'
ss -ltnp | grep ':8765'
```

资源监控：

```bash
free -h
nvidia-smi
top
```

## 11. 当前职责边界

我方当前已完成：

```text
SAN60 ROS Topic
  -> Adapter 约 10 Hz latest-only 转发
  -> Gateway latest cache
  -> JSON WebSocket /ws 输出 spectrum_frame
```

Backend 当前需要完成：

```text
连接 JSON /ws
  -> 识别 spectrum_frame
  -> 按 vehicle_id + stream_id 缓存 latest frame
  -> 转交 Unity
```

Unity 当前需要完成：

```text
消费 latest spectrum_frame
  -> 恢复 X 轴
  -> 10-20 Hz 实时显示
```

### Protobuf `/uav_usv/v1` 边界

当前 `uav_usv_gateway_v1.proto` 的 `GatewayEnvelope.oneof body` 没有 SAN60
Spectrum 类型。因此，如果正式 Backend 使用二进制 Protobuf WebSocket
`/uav_usv/v1`，而不是本说明中的 JSON `/ws`，则当前接口不能直接承载 SAN60。

届时需要前后端共同冻结并新增 Spectrum protobuf 消息、字段编号、编码/解码和兼容
策略。当前阶段不修改 Protobuf，也不把 JSON `spectrum_frame` 假定为已经存在的
Protobuf 能力。
