# 手机舰队数据测试岸基站报告

## 目标与架构

该 Demo 将 ROS 2 舰队世界模型转换为稳定 JSON，通过局域网 WebSocket 发送给手机浏览器。当前严格只读，不提供任何控制入口。

```text
Fleet World Model
  /fleet/world_model
  /fleet/world_model_summary
  -> Gateway FWM Adapter（世界模型转换）
  -> FleetRegistry（统一内部模型）
  -> ProtocolEncoder（JSON v1.0）
  -> WebSocket（有界客户端队列）
  -> 原生 HTML/CSS/JS 手机测试页 / 未来WebGL岸基态势平台
```

HTTP 线程、WebSocket asyncio 事件循环、墙钟推送线程和 ROS executor 相互独立。ROS 回调只更新最新值缓存，不处理网页布局，也不等待网络发送。推送线程通过 `call_soon_threadsafe` 将消息交给 asyncio loop，再进入每个客户端独立的异步有界优先队列。

周期推送使用墙钟而不是 ROS 仿真时钟，因此 `use_sim_time=true` 时，即使 Gazebo 暂停、`/clock` 尚未出现或临时中断，手机网页的快照、诊断和缓存状态仍会刷新。

## 仓库审计

### 舰队世界模型

- 主话题：`/fleet/world_model`
- 摘要话题：`/fleet/world_model_summary`
- 消息类型：`std_msgs/String` JSON
- 完整模型schema：`fleet_world_model.v1`
- 摘要schema：`fleet_world_model.summary.v1`
- 坐标主框架：`map`
- 载具：`fleet.uav`、`fleet.usv`、`fleet.unknown`
- 任务实体：`entities`，例如 `friendly_ship`、`enemy_ship`
- 目标：`targets`
- 传感器健康：`sensors`
- TF摘要：`tf`
- 任务与感知：`mission`、`perception`

网关默认只订阅世界模型，不再直接把 `/fleet/state`、`/fleet/perception/targets`、`/fleet/sensor_status` 当作网页主入口。旧话题只作为 `enable_legacy_topic_fallback=true` 时的兼容回退。

### 兼容输出

为兼容当前手机测试页，网关仍从世界模型派生以下 WebSocket 消息：

- `vehicle_state`：10 Hz，来自 `fleet.uav/usv/unknown`
- `perception_targets`：10 Hz，来自 `targets`
- `sensor_status`：1 Hz，来自 `sensors`
- `fleet_snapshot`：1 Hz，包含 `vehicles`、`targets`、`sensors`、`entities` 和完整 `world_model`
- `fleet_world_model`：2 Hz，完整世界模型
- `fleet_world_model_summary`：1 Hz，低带宽摘要，优先用于远距离链路健康和兜底态势
- `gateway_diagnostics`：1 Hz

未来 WebGL 正式平台应优先消费 `fleet_world_model`，只把兼容消息作为快速 UI 或调试入口。

远距离链路可关闭完整世界模型推送：

```bash
ros2 launch uav_usv_fleet_gateway remote_summary_gateway.launch.py
```

此时 WebSocket 仍推送 `fleet_world_model_summary`、载具、目标、传感器和诊断摘要，但不周期性推送完整 `fleet_world_model`。这是未来 4G/5G/卫星链路的推荐起点。

`gateway_hello` 和 `gateway_diagnostics` 都包含 `communication_profile`：

- `local_full`：完整世界模型模式；
- `remote_summary`：远距离摘要模式；
- `custom`：手动组合参数。

前端或岸基服务器可以用该字段决定是否等待完整 `fleet_world_model`。

### 现有网络代码

编码前仓库中没有通用 HTTP、WebSocket 或 MQTT 舰队网关，也没有可复用的手机网页。运行环境未安装 `websockets`/`aiohttp` Python 包，所以本实现使用 Python 标准库完成静态 HTTP 和受限 RFC6455 文本服务，不增加 Broker 或 pip 依赖。

## 实际订阅

默认配置位于 `src/uav_usv_fleet_gateway/config/fleet_gateway.yaml`：

| 数据 | 话题 | ROS 类型 |
| --- | --- | --- |
| 舰队世界模型 | `/fleet/world_model` | `std_msgs/String` JSON |
| 舰队世界模型摘要 | `/fleet/world_model_summary` | `std_msgs/String` JSON |
| 旧载具状态回退 | `/fleet/state` | `VehicleState` |
| 旧正式感知目标回退 | `/fleet/perception/targets` | `TrackedObjectArray` |
| 旧传感器摘要回退 | `/fleet/sensor_status` | `SensorStatus` |
| 旧围捕任务回退 | `/capture/state` | `CaptureState` |
| 旧感知源状态回退 | `/perception/source_status` | `std_msgs/String` JSON |

全部话题和服务端口均可通过 YAML/launch 参数修改，核心代码没有固定载具数量或固定载具 ID。旧话题默认不订阅。

## 构建与启动

```bash
cd /home/dji/UAV_USV
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_usv_fleet_gateway --symlink-install
source install/setup.bash
ros2 launch uav_usv_fleet_gateway mobile_fleet_demo.launch.py use_sim_time:=true
```

该 launch 只启动只读网关。完整仿真应另一个终端先启动原主线。网关停止或网页断开不会影响仿真和控制节点。

## 手机访问

查询电脑地址：

```bash
hostname -I
```

本机本次检测到的主要局域网地址为 `10.17.116.15`，因此本次测试地址为：

```text
http://10.17.116.15:8080
ws://10.17.116.15:8765/ws
```

地址会随网络变化，演示时以启动日志和 `hostname -I` 为准。

同 WiFi：电脑和手机连接同一路由器，手机直接打开 HTTP 地址。手机热点：手机开启热点，电脑连接热点后，在手机浏览器访问电脑获得的热点网段地址。手机无需 ROS 2、MQTT 或固定 IP。

检查防火墙：

```bash
sudo ufw status
sudo ufw allow 8080/tcp
sudo ufw allow 8765/tcp
```

程序不会自动修改防火墙。

## 网页功能

- 自动推断当前网页主机并生成 WebSocket 地址
- 连接、断开、自动重连、请求完整快照
- 无需点击按钮即可接收 10 Hz 载具状态、10 Hz 目标状态和 1 Hz 完整快照
- 自动接收 2 Hz 完整 `fleet_world_model`，供未来 WebGL 使用
- 按消息类型显示最近 5 秒接收 Hz、累计数量和最后更新时间
- 按 ID 更新 UAV/USV 卡片和目标卡片，不重复堆积
- Canvas 俯视图：UAV 三角形、USV 船形、目标框、航向、速度向量、拖动与缩放
- 传感器和网关健康摘要
- 最近 JSON 的暂停、恢复、清空和折叠
- 320 px 以上响应式 CSS，无 Vue/React/Three.js/npm 依赖

## 验证记录

已实测：

- Python 语法检查通过。
- `colcon build --packages-select uav_usv_fleet_gateway` 通过。
- 18 个自动测试通过，覆盖协议序号/null、姿态转换、注册表更新/超时、多目标、WebSocket 首帧/快照/ping/非法命令/断开、空闲连接、双客户端顺序、异步队列新旧位置替换与告警保护、HTTP 根路径与 health、手机页面依赖检查。
- 服务监听 `0.0.0.0:8080` 和 `0.0.0.0:8765`。
- HTTP `/` 返回网页，`/health` 返回正常 JSON。
- WebSocket 返回 `101 Switching Protocols`，随后按序发送 hello 和 snapshot。
- 实际 ROS 2 消息注入验证：`uav_01`、`usv_01`、`enemy_target`、`mid360` 均进入首次快照。
- `request_snapshot`、`ping`、非法 JSON 和未知只读命令处理通过。
- Ctrl+C 清理通过（修复了与 rclpy 内部订阅列表同名的问题）。
- 按用户要求执行 3 分钟稳定性测试：单客户端持续收到 2820 条消息，其中完整快照 181、载具状态 1361、目标状态 900、传感器状态 180、诊断 180、pong 17；sequence 无倒退，连接未中断。
- 稳定性测试结束时进程 RSS 约 61.9 MB、CPU 约 3.1%，HTTP/WebSocket 保持正常；客户端断开后计数恢复为 0，节点继续运行。
- 停止 `usv_01` 输入超过 3 秒后，快照正确显示 `online=false, stale=true`，同时 `uav_01` 保持在线。
- 实时推送回归：在 `use_sim_time=true` 且 `/clock` 发布者为 0 的条件下，不发送 `request_snapshot`，2 秒内收到 vehicle_state 21 条、perception_targets 20 条、fleet_snapshot 3 条和 diagnostics 2 条。
- 动态更新回归：同一 WebSocket 长连接中，将 `uav_01.position.x` 从 10 更新为 50，浏览器侧自动依次收到 `[10.0, 50.0]`，无需请求快照或刷新页面。

未实测：

- 尚未用真实 Android/iPhone 设备访问；本机没有可用 Chromium，因此未完成浏览器设备模拟截图。
- 手机关闭屏幕、切换 WiFi/蜂窝网络后的恢复尚未用真机测试。
- 原需求中的 30 分钟运行未执行；用户已允许缩短，实际完成的是 3 分钟连续稳定性测试。
- MQTT 仅保留 `enable_mqtt=false` 参数，未实现传输。
- 当前不会制作正式 WebGL 页面；本阶段只冻结 ROS2 到网页端的数据接口。
- 网关不会转发原始视频或点云，未来网页三维态势只接收世界坐标目标和健康摘要。
- 不发送视频或点云，这是本阶段刻意限制。

## 当前限制与下一步

- 标准库 WebSocket 支持本 Demo 需要的文本、ping/pong 和关闭流程，不提供压缩扩展。
- HTTPS/WSS 需要后续反向代理或 TLS 终止，本包当前直接提供 HTTP/WS。
- 载具电压在当前 `VehicleState` 不存在；电池百分比为负时按不可用处理。
- 下一步如需 MQTT，建议在协议层之后增加并行 transport，复用完全相同的信封；不要让 MQTT 消息进入 ROS 回调或让 Broker 成为 WebSocket 的强依赖。
