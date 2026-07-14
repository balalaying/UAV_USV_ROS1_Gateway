# UAV 相机接口

本文冻结舰队主线中 4 架 UAV 的相机数据接口，供后续 UAV 图像、USV Mid-360
点云和 LV-DOT 适配使用。本阶段只建立数据、标定、TF 和健康状态链路，不运行目标
检测，也不改变围捕任务的真值输入。

审计与完整回归日期：2026-07-14。

## 数据链路

```text
Gazebo camera sensor
  -> gz_sensor_bridge（兼容入口）
  -> /fleet/uplink/uav_01/camera
  -> uav_camera_adapter（不解码、不重编码）
  +-> /fleet/uplink/uav_01/camera/image_raw
  +-> /fleet/uplink/uav_01/camera/camera_info
  +-> /fleet/sensor_status

Gazebo model pose
  -> uav_camera_tf（只读）
  -> map -> uav_01/base_link -> uav_01/camera_link
```

`uav_02`、`uav_03` 和 `uav_04` 使用同样的命名规则。旧图像 topic 保留，现有
节点不会因本次接口升级失效；新算法和 Qt 基站应订阅标准 `image_raw` topic。

## 审计结果

4 架 UAV 均由 `fleet_dynamic_capture.sdf` 中的
`x500_low_latency_cam_down` 模型创建，Gazebo entity name 与 vehicle ID 相同。
相机安装在固定 `camera_link` 上，安装姿态相对 `base_link` 绕 Y 轴旋转
`1.5707 rad`，因此相机光轴朝向水面。

| 项目 | 当前实际值 |
| --- | --- |
| Gazebo sensor | `camera`，类型 `camera` |
| 水平视场角 | `1.74 rad` |
| SDF 分辨率 | `240 x 135` |
| ROS 图像格式 | `rgb8` |
| 单帧数据 | `97,200 bytes` |
| SDF 更新率 | `15 Hz` |
| 完整舰队实测 | `8.4-8.7 Hz` |
| Frame | `<vehicle_id>/camera_link` |
| 时间戳 | ROS 2 接收时钟，适配器原样保留 |

主线使用 `use_sim_time=false`。Gazebo 原始图像的仿真时间与 PX4、Mid-360 当前的
ROS 墙钟不是同一时间域，因此传感器桥在 ROS 入口写入节点时间；适配器不会再次
修改时间戳。这样图像和当前 Mid-360 接口可在同一 ROS 时间域内进行后续同步。

## Topic

以 `uav_01` 为例：

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/fleet/uplink/uav_01/camera` | `sensor_msgs/msg/Image` | 兼容输入，不建议新算法直接依赖 |
| `/fleet/uplink/uav_01/camera_info_raw` | `sensor_msgs/msg/CameraInfo` | Gazebo 标定的内部适配输入 |
| `/fleet/uplink/uav_01/camera/image_raw` | `sensor_msgs/msg/Image` | LV-DOT/融合模块使用的标准图像出口 |
| `/fleet/uplink/uav_01/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 与 `image_raw` 使用相同 header 的真实仿真内参 |
| `/fleet/sensor_status` | `uav_usv_interfaces/msg/SensorStatus` | 频率、延迟、像素数、丢帧估计、超时和 TF 健康 |

Gazebo 提供的当前理想针孔内参为：

```text
K = [101.238062, 0,          120.0,
     0,          101.238075, 67.5,
     0,          0,          1]

D = [0, 0, 0, 0, 0]
distortion_model = plumb_bob
```

`uav_camera_adapter` 使用 Gazebo 的 `CameraInfo`，没有按视场角重新猜测内参。
每次发布 `camera_info` 时只把 header 对齐到对应图像，标定矩阵保持原值。

## TF

每架 UAV 的 frame 结构如下：

```text
map
  -> uav_01/base_link
    -> uav_01/camera_link
```

- `map -> uav_01/base_link` 由 `uav_camera_tf` 读取 Gazebo entity pose 后发布。
- `uav_01/base_link -> uav_01/camera_link` 是静态安装变换。
- TF 节点只读取位姿，不发布 Gazebo 速度、PX4 setpoint 或 `FleetCommand`。
- `SensorStatus.tf_available` 使用 TF2 实际检查 `map -> camera_link`，没有修改图像
  坐标来掩盖 TF 错误。

验证：

```bash
ros2 run tf2_ros tf2_echo map uav_01/camera_link
```

完整围捕运行时实测相机位置从起飞台移动到约
`[14.6, 90.4, 25.8] m`，证明 camera frame 会随 PX4 UAV 运动，而不是固定在世界原点。

## 启动参数

主线默认开启相机标准适配：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py
```

相关参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `enable_uav_camera_adapter` | `true` | 启动标准图像、CameraInfo、TF 和 SensorStatus |
| `uav_camera_rate` | `15.0` | 相机桥最大转发率及健康状态期望频率，Hz |

仅调试旧兼容入口时可关闭：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  enable_uav_camera_adapter:=false
```

关闭后旧 `/fleet/uplink/<uav>/camera` 仍存在，但标准 topic、UAV 相机 TF 和 Qt 中的
UAV 视频不会发布。

## Qt 与 RViz

Qt 的 UAV 视频入口已改为标准 `image_raw`，视频仍由 ROS 工作线程接收，Qt 不运行
检测算法。相机健康行使用 `sensor_id=uav_camera`，显示：

- ONLINE/超时；
- 实测频率；
- 延迟；
- 最近更新时间；
- frame 和 TF 状态。

基站四路 UAV 与两路 USV 视频拼接实测约 `15 Hz`，Qt 启动后无缺失 topic 时不会
阻塞。RViz 配置 `fleet_dynamic_capture_mid360.rviz` 已加入：

- `UAV-01 Camera` Image display；
- 4 架 UAV 的 `base_link` 与 `camera_link` TF 显示。

## 2026-07-14 完整回归

启动完整 `4 UAV + 2 USV + enemy_target + Mid-360` 主线后，通过
`CAPTURE:enemy_target` 启动任务，结果如下：

- 4 个 PX4 DDS agent 均上线；
- 4 架 UAV 均解锁、进入 Offboard、完成起飞和动态位置跟踪；
- 2 艘 USV 均接受 Nav2 动态任务点；
- 围捕状态经过 `APPROACHING -> ENCIRCLING -> HOLDING -> SUCCESS`；
- 4 路相机 `healthy=true`、`tf_available=true`；
- Mid-360 同时保持约 `9.8 Hz`，围捕控制链未修改。

| 指标 | 实测值 |
| --- | ---: |
| UAV-01/02/03/04 相机 | `8.42-8.66 Hz` |
| 适配层延迟 | `6.16-6.73 ms` |
| 单路标准图像有效载荷 | `0.826-0.836 MB/s` |
| 四路图像有效载荷 | 约 `3.33 MB/s` |
| 四路图像 + Mid-360 有效载荷 | `3.737 MB/s` |
| `uav_camera_adapter` | 约 `10.8% CPU`，`73 MB RSS` |
| Gazebo real-time factor | `1.0003` |
| GPU | 约 `9%`，显存 `1542/8188 MiB` |

无损检查使用相同时间戳配对兼容输入与标准输出，编码、尺寸、frame 和 SHA-256
数据摘要完全一致。

## 验证命令

```bash
ros2 topic list -t | grep '/camera/'
ros2 topic hz /fleet/uplink/uav_01/camera/image_raw
ros2 topic echo /fleet/uplink/uav_01/camera/camera_info --once
ros2 topic echo /fleet/sensor_status --once
ros2 run tf2_ros tf2_echo map uav_01/camera_link

# 已运行主世界后打开 Qt
ros2 launch uav_usv_bringup dynamic_capture_console.launch.py
```

## 已知限制

1. SDF 设置为 15 Hz，但完整舰队同时运行 6 路相机、Mid-360、4 个 PX4 和 2 套
   Nav2 时，UAV 相机实际约 8.5 Hz。接口稳定，但后续视觉算法应按消息时间戳同步，
   不应假设固定帧间隔。
2. 主 launch 中 `prepare_large_x500.py` 打印的 `320x180@20` 针对 PX4 自带
   `mono_cam`；当前世界使用独立的 `low_latency_down_camera`，实测仍是
   `240x135@15`。后续若提高分辨率，应单独进行负载回归，不能只依据启动日志。
3. 当前 `map -> UAV base_link` 使用 Gazebo 真值，适合仿真接口验证。接入真实定位或
   SLAM 后，应由统一定位模块接管该变换，标准相机 topic 无需改变。
