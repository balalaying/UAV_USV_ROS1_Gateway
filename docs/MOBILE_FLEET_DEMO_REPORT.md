# 手机舰队数据测试岸基站报告

## 目标与架构

该 Demo 将现有 ROS 2 舰队状态转换为稳定 JSON，通过局域网 WebSocket 发送给手机浏览器。当前严格只读，不提供任何控制入口。

```text
ROS 2 Topics
  -> ROS Adapter（消息转换）
  -> FleetRegistry（统一内部模型）
  -> ProtocolEncoder（JSON v1.0）
  -> WebSocket（有界客户端队列）
  -> 原生 HTML/CSS/JS 手机测试页
```

HTTP 线程、WebSocket 线程和 ROS executor 相互独立。ROS 回调只更新最新值缓存，不处理网页布局，也不等待网络发送。

## 仓库审计

### 载具状态

- 通用消息：`uav_usv_interfaces/msg/VehicleState`
- 实际话题：`/fleet/state`
- 发布者：`uav_fleet_agent.py`、`uav_dds_fleet_agent.py`、`usv_fleet_agent.py` 和仿真 agent
- ID 规则：`uav_01...`、`usv_01...`，消息内 `vehicle_id` 是权威 ID
- UAV/USV 类型：`VehicleState.TYPE_UAV` / `TYPE_USV`
- 位姿：`VehicleState.pose`，主线 frame 为 `map`
- 速度：`VehicleState.twist`
- 姿态：`VehicleState.pose.orientation`；网关保留四元数并计算欧拉角
- 模式/解锁：`mode`、`armed`
- 电池：仅有 `battery_percent`；当前多个 agent 使用 `-1.0` 表示不可用，网关输出 `null`
- 电压：消息不存在，输出 `null`
- namespace：状态集中发布在全局 `/fleet/state`，逻辑 namespace 由 `vehicle_id` 表示为 `/<vehicle_id>`

### 感知与任务

- perception fusion 输出：`/perception/fused/tracks`
- source mux 正式输出：`/fleet/perception/targets`
- source mux 状态：`/perception/source_status`
- LV-DOT ROS2 track：`/perception/lv_dot_ros2/tracks`
- LV-DOT dynamic：`/perception/lv_dot_ros2/dynamic_tracks`
- LV-DOT 标准观察：`/perception/lv_dot/observations`
- 统一目标消息：`TrackedObjectArray`，对象含 track ID、分类、来源 mask、位姿/速度协方差、尺寸和置信度
- 传感器健康：`/fleet/sensor_status`，类型为 `SensorStatus`
- 围捕任务：`/capture/state`，类型为 `CaptureState`
- 角色/目标诊断还包括 `/capture/roles`、`/capture/target_status` 和文本兼容话题

网关优先订阅 source mux 的正式输出，不订阅 LV-DOT Shadow 作为正式目标，因此不会混淆真值、Shadow 和融合结果。`perception_source` 的默认值未被修改，仍为 `ground_truth`。

### 现有网络代码

编码前仓库中没有通用 HTTP、WebSocket 或 MQTT 舰队网关，也没有可复用的手机网页。运行环境未安装 `websockets`/`aiohttp` Python 包，所以本实现使用 Python 标准库完成静态 HTTP 和受限 RFC6455 文本服务，不增加 Broker 或 pip 依赖。

## 实际订阅

默认配置位于 `src/uav_usv_fleet_gateway/config/fleet_gateway.yaml`：

| 数据 | 话题 | ROS 类型 |
| --- | --- | --- |
| 载具状态 | `/fleet/state` | `VehicleState` |
| 正式感知目标 | `/fleet/perception/targets` | `TrackedObjectArray` |
| 传感器摘要 | `/fleet/sensor_status` | `SensorStatus` |
| 围捕任务 | `/capture/state` | `CaptureState` |
| 感知源状态 | `/perception/source_status` | `std_msgs/String` JSON |

全部话题和服务端口均可通过 YAML/launch 参数修改，核心代码没有固定载具数量或固定载具 ID。

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
- 按 ID 更新 UAV/USV 卡片和目标卡片，不重复堆积
- Canvas 俯视图：UAV 三角形、USV 船形、目标框、航向、速度向量、拖动与缩放
- 传感器和网关健康摘要
- 最近 JSON 的暂停、恢复、清空和折叠
- 320 px 以上响应式 CSS，无 Vue/React/Three.js/npm 依赖

## 验证记录

已实测：

- Python 语法检查通过。
- `colcon build --packages-select uav_usv_fleet_gateway` 通过。
- 16 个自动测试通过，覆盖协议序号/null、姿态转换、注册表更新/超时、多目标、WebSocket 首帧/快照/ping/非法命令/断开、空闲连接、双客户端顺序、队列上限、HTTP 根路径与 health、手机页面依赖检查。
- 服务监听 `0.0.0.0:8080` 和 `0.0.0.0:8765`。
- HTTP `/` 返回网页，`/health` 返回正常 JSON。
- WebSocket 返回 `101 Switching Protocols`，随后按序发送 hello 和 snapshot。
- 实际 ROS 2 消息注入验证：`uav_01`、`usv_01`、`enemy_target`、`mid360` 均进入首次快照。
- `request_snapshot`、`ping`、非法 JSON 和未知只读命令处理通过。
- Ctrl+C 清理通过（修复了与 rclpy 内部订阅列表同名的问题）。
- 按用户要求执行 3 分钟稳定性测试：单客户端持续收到 2820 条消息，其中完整快照 181、载具状态 1361、目标状态 900、传感器状态 180、诊断 180、pong 17；sequence 无倒退，连接未中断。
- 稳定性测试结束时进程 RSS 约 61.9 MB、CPU 约 3.1%，HTTP/WebSocket 保持正常；客户端断开后计数恢复为 0，节点继续运行。
- 停止 `usv_01` 输入超过 3 秒后，快照正确显示 `online=false, stale=true`，同时 `uav_01` 保持在线。

未实测：

- 尚未用真实 Android/iPhone 设备访问；本机没有可用 Chromium，因此未完成浏览器设备模拟截图。
- 手机关闭屏幕、切换 WiFi/蜂窝网络后的恢复尚未用真机测试。
- 原需求中的 30 分钟运行未执行；用户已允许缩短，实际完成的是 3 分钟连续稳定性测试。
- MQTT 仅保留 `enable_mqtt=false` 参数，未实现传输。
- 不发送视频或点云，这是本阶段刻意限制。

## 当前限制与下一步

- 标准库 WebSocket 支持本 Demo 需要的文本、ping/pong 和关闭流程，不提供压缩扩展。
- HTTPS/WSS 需要后续反向代理或 TLS 终止，本包当前直接提供 HTTP/WS。
- 载具电压在当前 `VehicleState` 不存在；电池百分比为负时按不可用处理。
- 下一步如需 MQTT，建议在协议层之后增加并行 transport，复用完全相同的信封；不要让 MQTT 消息进入 ROS 回调或让 Broker 成为 WebSocket 的强依赖。
