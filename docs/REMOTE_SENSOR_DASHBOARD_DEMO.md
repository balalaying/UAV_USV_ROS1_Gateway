# 远程传感器岸基站 Demo

## 目标

该 Demo 在不改变既有舰队协议的前提下增加相机、map 点云和
Camera-LiDAR 融合框显示。浏览器不连接 ROS 2，也不连接仿真主机的局域网
地址。

```text
Camera / Mid-360 / Fusion / Fleet World Model
                    |
                    v
fleet_sensor_stream_adapter
                    |
                    v
fleet_gateway (127.0.0.1，仅本机)
                    |
             主动出站 WebSocket
                    |
                    v
remote_relay (公网服务器角色)
                    |
                    v
浏览器 WebSocket + HTML5 Canvas
```

本机演示时，中继运行在 `127.0.0.1:9765`，用于验证进程和接口边界。它不使用
主机局域网 IP。真实跨网时只需把中继进程部署到公网服务器，并把
`relay_url` 改为公网 `wss://` 地址。

## 启动

先启动统一 332 主世界和实时感知链，关闭 Qt 以避免重复点云投影：

```bash
cd <YOUR_UAV_USV_WORKSPACE>
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py \
  enable_console:=false \
  start_rviz:=false
```

第二个终端启动远端架构模拟：

```bash
cd <YOUR_UAV_USV_WORKSPACE>
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_usv_fleet_gateway remote_sensor_dashboard.launch.py
```

`use_sim_time` 必须与主世界一致。当前
`fleet_dynamic_capture_live_perception.launch.py` 默认使用墙钟，因此远端
Dashboard 也默认 `false`；若主世界显式使用仿真时钟，两边都传
`use_sim_time:=true`。

浏览器打开：

```text
http://127.0.0.1:9080
```

页面连接的是中继端：

```text
ws://127.0.0.1:9765/ws
```

而不是 ROS 2 网关的 `ws://127.0.0.1:8765/ws`。

## 真实异网部署

公网服务器运行中继：

```bash
ros2 run uav_usv_fleet_gateway fleet_remote_relay \
  --bind-address 0.0.0.0 \
  --websocket-port 9765 \
  --http-address 0.0.0.0 \
  --http-port 9080 \
  --web-root <INSTALLED_SHARE>/uav_usv_fleet_gateway/web \
  --token <SHARED_TOKEN>
```

仿真机或真实载具端只建立出站连接：

```bash
ros2 launch uav_usv_fleet_gateway remote_sensor_dashboard.launch.py \
  start_local_relay:=false \
  relay_url:='wss://<PUBLIC_HOST>/uplink?token=<SHARED_TOKEN>'
```

公网部署应由 Nginx 或 Caddy 终止 TLS，并把 `/ws` 和 `/uplink` 转发到
`127.0.0.1:9765`。网页端填写 `wss://<PUBLIC_HOST>/ws`。中继地址变更不会改变
ROS 2 topic、内部模型或 WebSocket 信封。

## 协议兼容性

原消息保持不变：

- `gateway_hello`
- `fleet_snapshot`
- `vehicle_state`
- `perception_targets`
- `sensor_status`
- `gateway_diagnostics`

新增向后兼容消息：

| `message_type` | 内容 | 默认频率 |
| --- | --- | --- |
| `camera_frame` | JPEG Base64、frame、时间戳、分辨率 | 8 Hz/流 |
| `pointcloud_frame` | map 坐标 XYZ 平铺数组 | 8 Hz/流 |
| `fusion_debug` | Camera-LiDAR Marker 几何 | 10 Hz/流 |

所有消息继续使用协议 v1.0 信封。旧客户端会忽略未知 `message_type`，新客户端
可以同时处理原状态消息和传感器扩展。

## ROS 2 输入

三艘 USV：

- `/perception/usv_XX/mid360/points_filtered`
- `/perception/usv_XX/camera/detections/image`
- `/perception/usv_XX/camera_lidar/fused_bboxes`

三架 UAV：

- `/fleet/uplink/uav_XX/camera/image_raw`

点云先经过 `qt_pointcloud_projection_node.py`，按消息时间戳查询 TF 并转换到
`map`，再进入网页。网页不处理传感器坐标和 TF。

投影节点在等待历史 TF 时保留当前待处理帧，不让更新更快的新帧持续覆盖它。
这保证动态载具的点云、融合框和 Fleet World Model 使用同一时刻的 `map` 关系。

## 页面能力

- 三艘 USV 感知源切换；
- 六路相机流切换；
- 俯视/斜俯视 map 点云；
- Camera-LiDAR 融合三维框；
- UAV、USV、敌方目标和速度方向；
- 滚轮缩放、拖动平移、双击恢复；
- 载具在线、模式、位置、速度；
- WebSocket 消息频率、丢包和链路状态。

## 当前边界

- 只读显示，不发布 `FleetCommand`；
- 不修改 `capture_manager`、PX4、Nav2 或 source mux；
- JPEG/Base64 是演示方案，正式高码率视频建议改为 WebRTC；
- 公网服务器、域名和 TLS 证书需要部署方提供，本仓库保留相同上行接口。
