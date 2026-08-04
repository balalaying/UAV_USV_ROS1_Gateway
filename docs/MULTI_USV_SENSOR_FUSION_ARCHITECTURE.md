# 三无人船统一传感器与舰队感知架构

## 1. 目标与边界

332主世界中的 `USV_01`、`USV_02`、`USV_03` 现在使用相同的感知结构：

```text
USV
├── RGB camera
├── RGB-D depth camera
└── Livox Mid-360 (RGL)
```

本次修改只扩展仿真传感器、TF、Observation流水线、舰队感知出口和Qt调试选择器。
以下控制链没有修改：

- PX4 UAV控制；
- USV Nav2和USV agent；
- `capture_manager`；
- `FleetCommand`；
- `perception_source_mux`默认来源。

因此真实感知仍处于Shadow模式，不会接管围捕控制。

## 2. 数据流

```text
                 USV_01                         USV_02                         USV_03
        RGB / Depth / Mid360           RGB / Depth / Mid360           RGB / Depth / Mid360
                  |                              |                              |
                  +------------ Gazebo sensor bridge / RGL bridge ------------+
                                                 |
                       /fleet/uplink/usv_xx/{camera,depth,mid360}
                                                 |
                 +-------------------------------+-------------------------------+
                 |                               |                               |
        mid360_preprocessor             mid360_preprocessor             mid360_preprocessor
                 |                               |                               |
        LV-DOT + Camera-LiDAR           LV-DOT + Camera-LiDAR           LV-DOT + Camera-LiDAR
                 |                               |                               |
       /perception/usv_01/             /perception/usv_02/             /perception/usv_03/
            observations                    observations                    observations
                 +-------------------------------+-------------------------------+
                                                 |
                                   fleet_usv_perception_fusion
                                                 |
                                 /fleet/perception/usv_tracks
                                                 |
                                  fleet_global_perception_fusion
                                                 |
                               /fleet/perception/fused_targets
                                                 |
                                 /perception/fused/tracks
                                     (兼容别名，仅显示)
                                                 |
                                  Fleet World Model
                                                 |
                          /fleet/world_model, /fleet/world_model_summary
```

`TrackedObjectArray`仍是传感器无关的统一Observation格式。任务层不需要知道目标来自
哪一艘船、相机还是雷达。

后续网页岸基平台不直接订阅相机、点云或LV-DOT debug topic。网页只消费
`/fleet/world_model` 和必要的轻量摘要/命令接口；所有传感器、目标和载具状态必须先进入
舰队级 `map` 坐标和FWM。

## 3. Namespace与节点实例

载具控制节点继续位于：

| 载具 | ROS namespace | Gazebo entity |
|---|---|---|
| USV_01 | `/usv_01` | `usv_01`，控制模型名 `own_01` |
| USV_02 | `/usv_02` | `usv_02`，控制模型名 `own_02` |
| USV_03 | `/usv_03` | `usv_03`，控制模型名 `own_03` |

感知节点使用舰队级语义路径，避免把跨载具融合隐藏在控制namespace中：

| 功能 | USV_01示例 | 其余实例规则 |
|---|---|---|
| LV-DOT | `/perception/lv_dot/usv_01/lv_dot_detector_node` | 替换vehicle ID |
| Camera detector | `/usv_01_camera_detection` | 替换vehicle ID |
| Camera-LiDAR | `/usv_01_camera_lidar_association` | 替换vehicle ID |
| Observation adapter | `/usv_01_lv_dot_observation_adapter` | 替换vehicle ID |

源码和算法中不再依赖固定的 `usv_01` 输入topic。Launch为每个实例显式传入topic和frame。

## 4. Topic接口

每艘USV均提供下表接口，其中 `{id}` 为 `usv_01`、`usv_02` 或 `usv_03`：

| Topic | 类型 | 用途 |
|---|---|---|
| `/fleet/uplink/{id}/camera/image_raw` | `sensor_msgs/Image` | 标准RGB图像 |
| `/fleet/uplink/{id}/camera/camera_info` | `sensor_msgs/CameraInfo` | RGB内参 |
| `/fleet/uplink/{id}/depth/image_raw` | `sensor_msgs/Image` | 深度图，`32FC1`，单位米 |
| `/fleet/uplink/{id}/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度相机内参 |
| `/fleet/uplink/{id}/mid360/points` | `sensor_msgs/PointCloud2` | Mid-360原始点云 |
| `/perception/{id}/mid360/points_filtered` | `sensor_msgs/PointCloud2` | 预处理点云 |
| `/perception/lv_dot/{id}/tracks` | `TrackedObjectArray` | LV-DOT轨迹 |
| `/perception/lv_dot/{id}/dynamic_tracks` | `TrackedObjectArray` | 动态轨迹 |
| `/perception/{id}/observations` | `TrackedObjectArray` | 相机-雷达标准Observation |
| `/fleet/perception/usv_tracks` | `TrackedObjectArray` | 三艘USV融合轨迹 |
| `/fleet/perception/fused_targets` | `TrackedObjectArray` | 舰队级统一目标 |

`/perception/fused/tracks`是Qt和旧调试工具使用的兼容别名。正式后续WebSocket网关应订阅
`/fleet/perception/fused_targets`。

## 5. TF体系

所有传感器结果最终进入 `map`：

```text
map
└── usv_xx/base_link
    ├── usv_xx/camera_link
    ├── usv_xx/depth_camera_link
    ├── usv_xx/mid360_link
    └── usv_xx/front_lidar
```

- 舰队级全局 `/tf` 使用 Gazebo 真值直接发布 `map -> */base_link`；
- `/usv_xx/tf` 中的局部 `map -> usv_xx/odom -> usv_xx/base_link` 仅供Nav2/USV局部控制使用，不再转发进全局 `/tf`；
- 相机和Mid-360安装外参由静态TF发布；
- 点云、检测框和Observation按消息时间戳转换到 `map`；
- Qt只显示map坐标，不处理camera/body/lidar局部坐标。

当前仿真安装外参：

| Frame | 相对 `base_link` 的平移 |
|---|---|
| `camera_link` | `(3.24, 0.0, 1.55)` m |
| `depth_camera_link` | `(3.24, 0.0, 1.55)` m |
| `mid360_link` | `(0.9075, 0.0, 1.5625)` m |

## 6. 模型与运行时注入

三个船体SDF都声明了相同规格的深度相机：

- 240 x 135；
- `R_FLOAT32`；
- 15 Hz配置频率；
- 0.2到80 m量程；
- 不改变质量、惯量、碰撞和控制插件。

Mid-360仍采用已验证的RGL运行时注入方案。`prepare_fleet_mid360.py`一次处理三个
vehicle ID，并为每艘船生成独立模型副本、topic和frame。这样不会复制或替换USV控制模型。

## 7. Qt选择器

Qt `Perception Monitor`页面增加“USV感知源”选择器：

- 我方船一号（蓝色）；
- 我方船二号（绿色）；
- 我方船三号（青色）。

每艘船都有独立线程安全缓存，包含：

- 原始和过滤点云；
- LV-DOT框、Track和Dynamic Track；
- Camera-LiDAR调试层；
- `base_link`和`mid360_link` TF；
- 对应融合相机画面及状态。

切换下拉框只更换显示缓存，不改变ROS算法、topic发布或任务控制。

## 8. 启动与验证

完整实时感知主线：

```bash
cd ~/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_usv_bringup \
  fleet_dynamic_capture_live_perception.launch.py
```

不启动PX4和Qt的传感器回归：

```bash
ros2 launch uav_usv_bringup \
  fleet_dynamic_capture_live_perception.launch.py \
  start_px4:=false start_dds_agent:=false \
  start_rviz:=false enable_console:=false
```

检查三路数据：

```bash
ros2 topic hz /fleet/uplink/usv_01/mid360/points
ros2 topic hz /fleet/uplink/usv_02/mid360/points
ros2 topic hz /fleet/uplink/usv_03/mid360/points
ros2 topic echo /fleet/uplink/usv_03/depth/camera_info \
  --once --field header.frame_id
ros2 run tf2_ros tf2_echo map usv_03/mid360_link
ros2 topic echo /fleet/perception/fused_targets --once --field header
```

## 9. 2026-07-23在线回归结果

在RTX 4060 Laptop GPU主机、关闭PX4和Qt的完整332感知场景中：

| 项目 | 实测结果 |
|---|---|
| 三路Mid-360 | 均持续发布，稳定后约12到15 Hz |
| 原始点数 | 每帧约2393、2678、2595点，随场景变化 |
| 三路RGB | 约10到12 Hz |
| 三路Depth | 约9到12 Hz，编码 `32FC1` |
| 深度frame | 三路均为对应 `{id}/depth_camera_link` |
| TF | `map -> usv_03/base_link -> usv_03/mid360_link`连续随船变化 |
| 舰队USV轨迹 | `/fleet/perception/usv_tracks`持续发布map header |
| 舰队融合目标 | `/fleet/perception/fused_targets`持续发布map header |
| FWM舰队实体 | 轻量模式下3 UAV、3 USV均进入 `/fleet/world_model`，UAV为 `state_source=tf_only` |
| FWM任务实体 | `friendly_ship`、`enemy_ship`进入 `entities` |
| RGL GPU占用 | 约13%，显存约1989 MiB（整场景总量） |
| Gazebo server | 约176% CPU，RSS约1.46 GiB |
| 传感器桥 | 约91% CPU，RSS约197 MiB |
| 单个LV-DOT实例 | 约5到7% CPU，RSS约55 MiB |

测试中没有修改或切换 `perception_source_mux`，任务输入仍为ground truth。

## 10. 已知问题

1. 完整三船传感器显著增加Gazebo和桥接负载。后续网页阶段应只上传目标和降采样数据，
   不应直接推送三路原始点云。
2. 现有Nav2二维 `scan_raw` 有历史QoS警告，本次未修改USV控制链。
3. 仿真偶尔暂停/恢复时，LV-DOT会丢弃一帧非单调时间戳，随后可继续运行。
4. 一次Ctrl+C关闭测试中，某个Gazebo点云桥在transport析构阶段返回 `-11`；运行期间三路
   点云均稳定。该问题属于关闭顺序，不影响在线数据，但后续应单独修复桥析构。
5. 本阶段只建立RGB-D数据接口，深度图尚未进入目标检测或融合算法。后续算法必须先走
   Shadow验证，不能直接进入控制。
