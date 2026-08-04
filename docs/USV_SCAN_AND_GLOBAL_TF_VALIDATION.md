# USV扫描与全局TF验证

## 问题

332主世界中，Gazebo传感器桥以 `BEST_EFFORT` QoS发布
`/usv_xx/scan_raw`，而 `boat_nav2_interface` 原来使用默认
`RELIABLE` QoS订阅，两端不兼容。

此外，旧过滤逻辑在船体横滚或俯仰超过 `0.025 rad` 时丢弃整帧。
海浪会频繁触发该条件，使Nav2使用的 `/usv_xx/scan` 输出不连续。

舰队全局 `/tf` 原来只有 `map -> */base_link` 和相机安装关系。
`front_lidar` 只存在于每艘USV的私有Nav2 TF topic中，Qt、RViz和FWM
无法从 `map` 查询雷达坐标。

## 修改

1. `boat_nav2_interface` 使用 `qos_profile_sensor_data` 订阅原始扫描。
2. 不再按整船倾角丢弃扫描帧，继续使用每个扫描端点的世界高度过滤海浪。
3. 全局 `/tf_static` 增加：
   - `usv_xx/base_link -> usv_xx/front_lidar`
   - `uav_xx/base_link -> uav_xx/camera_link`
4. FWM区分动态TF和静态TF，静态边永久有效。
5. Nav2私有 `/usv_xx/tf` 保持不变，控制链未修改。

## 在线结果

332场景、PX4关闭、Mid-360关闭时，10秒同时计数：

| Topic | 频率 |
| --- | ---: |
| `/usv_01/scan_raw` | 11.29 Hz |
| `/usv_01/scan` | 11.29 Hz |
| `/usv_02/scan_raw` | 11.39 Hz |
| `/usv_02/scan` | 11.39 Hz |
| `/usv_03/scan_raw` | 11.40 Hz |
| `/usv_03/scan` | 11.74 Hz |

仿真实时率变化会影响绝对频率，但原始和过滤后的消息数量保持一一对应。
三组发布者和订阅者均为 `BEST_EFFORT`，日志中没有QoS不兼容告警。

全局TF实测可查询：

```text
map -> usv_01/base_link -> front_lidar/camera_link/depth_camera_link
map -> usv_02/base_link -> front_lidar/camera_link/depth_camera_link
map -> usv_03/base_link -> front_lidar/camera_link/depth_camera_link
map -> uav_01/base_link -> camera_link
map -> uav_02/base_link -> camera_link
map -> uav_03/base_link -> camera_link
map -> enemy_ship/base_link
map -> friendly_ship/base_link
```

## 验证命令

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  start_px4:=false start_dds_agent:=false start_rviz:=false \
  enable_mid360:=false

ros2 topic info /usv_01/scan_raw --verbose
ros2 run tf2_ros tf2_echo map usv_01/front_lidar
ros2 topic echo /fleet/world_model --once
```

网页端当前不启动；`/fleet/world_model` 和
`/fleet/world_model_summary` 保留为未来远程Gateway接口。
