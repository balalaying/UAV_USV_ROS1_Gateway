# 雷达与相机融合适配审计报告

日期：2026-07-31
范围：当前332主世界、`fleet_dynamic_capture_live_perception.launch.py`、传感器桥、LV-DOT、相机-雷达融合、舰队融合、Fleet World Model与Qt显示。
方法：静态审计当前SDF、launch、节点源码和已有在线验证记录；审计时没有Gazebo/ROS运行进程，因此本文不把配置值写成当日在线实测值。

## 1. 结论摘要

当前系统已经具备三艘USV的**独立完整传感器链路**，并且已经具备在`map`坐标下做**目标级三船融合**的节点和主启动入口：

```text
USV_01 / USV_02 / USV_03
  Mid-360 + RGB camera + RGB-D depth camera
             |
        各自预处理、LV-DOT、相机-雷达几何关联
             |
      /perception/usv_xx/observations  (map)
             |
  fleet_usv_perception_fusion          (跨船目标级关联)
             |
      /fleet/perception/usv_tracks     (map)
             |
  fleet_global_perception_fusion       (规范化/稳定输出)
             |
      /fleet/perception/fused_targets  (map)
             |
      Fleet World Model -> /fleet/world_model
```

需要特别区分：

- 已完成的是**对象/轨迹级融合**，不是三路原始`PointCloud2`的拼接、配准或联合建图。
- Camera-LiDAR使用相机检测引导的点云ROI、时间戳TF和投影匹配，属于真实几何关联；并非仅在Qt中把两层图形叠加。
- Depth Camera已在三船模型、桥接与TF中存在，但当前不参与Camera-LiDAR的深度估计、目标检测或Fleet Fusion。
- 所有感知与World Model仍为Shadow侧链。`perception_source`默认仍为`ground_truth`，不进入`capture_manager`控制输入。

## 2. 传感器层

### 2.1 三艘USV的传感器清单

| 载具 | Mid-360 | RGB Camera | RGB-D Depth Camera | 结论 |
|---|---|---|---|---|
| `usv_01` | 有，RGL运行时注入 | 有 | 有 | 完整接口 |
| `usv_02` | 有，RGL运行时注入 | 有 | 有 | 完整接口 |
| `usv_03` | 有，RGL运行时注入 | 有 | 有 | 完整接口 |

船体包装模型`sim332_usv_{blue,green,cyan}`复用`defense_own_0x_boat`。RGB与Depth传感器定义在后者；Mid-360不直接写入源SDF，而由`prepare_fleet_mid360.py`复制运行时模型并追加RGL插件、`mid360_link`语义frame和可视外壳。这种方式没有修改船体的质量、碰撞、惯量或控制插件。

### 2.2 标准ROS接口

`{id}`表示`usv_01`、`usv_02`或`usv_03`。

| 数据 | Gazebo来源 | ROS标准Topic | 消息 | `frame_id` | 频率 |
|---|---|---|---|---|---|
| RGB原始图 | `/defense/own_0x/front_camera` | `/fleet/uplink/{id}/camera/image_raw` | `sensor_msgs/Image` | `{id}/camera_link` | SDF 30Hz，桥默认限至15Hz |
| RGB内参 | 桥按RGB尺寸/FOV合成 | `/fleet/uplink/{id}/camera/camera_info` | `sensor_msgs/CameraInfo` | `{id}/camera_link` | 随图像发布 |
| 深度图 | `/defense/own_0x/depth_camera` | `/fleet/uplink/{id}/depth/image_raw` | `sensor_msgs/Image`，`32FC1` | `{id}/depth_camera_link` | SDF/桥为15Hz上限 |
| 深度内参 | 桥按深度图合成 | `/fleet/uplink/{id}/depth/camera_info` | `sensor_msgs/CameraInfo` | `{id}/depth_camera_link` | 随深度图发布 |
| Mid-360原始点云 | RGL topic | `/fleet/uplink/{id}/mid360/points` | `sensor_msgs/PointCloud2` | `{id}/mid360_link` | 主启动默认20Hz |
| Mid-360过滤点云 | `mid360_preprocessor.py` | `/perception/{id}/mid360/points_filtered` | `sensor_msgs/PointCloud2` | 保留原`mid360_link` | 跟随输入 |

已有`MULTI_USV_SENSOR_FUSION_ARCHITECTURE.md`记录过一次完整场景在线回归：三路Mid-360约12--15Hz，三路RGB约10--12Hz，深度约9--12Hz。该数据为历史验证记录，不是本次审计的实时测量。

### 2.3 深度相机的实际状态

深度相机的SDF、桥接、标准topic与静态TF均已存在；当前仓库中没有节点订阅`/fleet/uplink/{id}/depth/image_raw`进行检测、尺度恢复、ROI深度约束或融合。因此深度相机目前是**已接入但未被感知算法消费的预留输入**。

## 3. TF架构与坐标流

### 3.1 当前舰队级TF树

`fleet_pose_tf_publisher.py`以Gazebo真值在全局`/tf`中发布直接边：

```text
map
├── usv_01/base_link
│   ├── usv_01/camera_link               (static)
│   ├── usv_01/depth_camera_link         (static)
│   ├── usv_01/front_lidar               (static)
│   └── usv_01/mid360_link               (static)
├── usv_02/base_link
│   └── 同上
├── usv_03/base_link
│   └── 同上
├── uav_01/base_link -> uav_01/camera_link
├── uav_02/base_link -> uav_02/camera_link
├── uav_03/base_link -> uav_03/camera_link
├── friendly_ship/base_link
└── enemy_ship/base_link
```

当前没有名为`base_station`的TF frame。世界中存在`shore_command_base`模型，但它没有被当前`fleet_pose_tf_publisher`列为`target_ids`，因此不会形成`map -> base_station`或`map -> shore_command_base/base_link`的权威TF边。

每艘USV的局部Nav2链路仍可在`/usv_xx/tf`中存在`{id}/odom -> {id}/base_link`，但已被刻意隔离，避免与全局`map -> {id}/base_link`并存于同一全局树。

### 3.2 传感器安装外参

| 子frame | 相对`{id}/base_link`平移m | 发布方式 |
|---|---:|---|
| `{id}/camera_link` | `(3.24, 0.0, 1.55)` | `static_transform_publisher` |
| `{id}/depth_camera_link` | `(3.24, 0.0, 1.55)` | `static_transform_publisher` |
| `{id}/front_lidar` | `(0.9075, 0.0, 1.5625)` | `static_transform_publisher` |
| `{id}/mid360_link` | `(0.9075, 0.0, 1.5625)` | `static_transform_publisher` |

RGB SDF中没有显式`gz_frame_id`，而桥接节点在ROS消息头中赋值为`{id}/camera_link`；其物理`<pose>`与上述静态外参相同。Depth SDF拥有对应`gz_frame_id`，桥也显式写入相同frame。

### 3.3 是否已转换到统一坐标

- 原始点云和过滤点云：**没有变换点坐标**，仍在各自`{id}/mid360_link`局部坐标；这是正确的传感器数据保留方式。
- LV-DOT：每帧以`PointCloud2.header.stamp`查询`map <- {id}/mid360_link`，在进入算法前转换到`map`，输出Track/BBox为`map`。
- Vision-guided ROI：按点云时刻查询`camera <- lidar`与`map <- camera`，将ROI点、3D框和Observation发布为`map`。
- Camera-LiDAR association：按历史时间戳查询`camera <- map`与`map <- camera`；输出数组header固定为`map`。
- Fleet Fusion：若输入不是`map`会按消息时间戳做TF转换；主线输入理论上已是`map`。

因此，三雷达数据没有先融合成一个全球点云，但其检测结果在融合前已统一到`map`。

## 4. Mid-360处理链

每艘船都实例化相同链路：

```text
RGL Livox Mid-360
  -> /fleet/uplink/{id}/mid360/rgl_points        (Gazebo)
  -> gz_pointcloud_bridge.py
  -> /fleet/uplink/{id}/mid360/points            (PointCloud2)
  -> mid360_preprocessor.py
  -> /perception/{id}/mid360/points_filtered
  -> lv_dot_detector_node (LifecycleNode)
  -> /perception/lv_dot/{id}/diagnostics/lidar_bboxes
  -> /perception/lv_dot/{id}/tracks
  -> /perception/lv_dot/{id}/dynamic_tracks
  -> lv_dot_observation_adapter.py
  -> /perception/{id}/lidar/observations
```

预处理执行NaN/Inf过滤、距离裁剪、Z裁剪、船体自身裁剪与体素降采样。LV-DOT ROS2版本完成DBSCAN、3D BBox、关联、Kalman跟踪和动态分类。主线适配器从`dynamic_tracks`输出标准`TrackedObjectArray`。

注意：Camera-LiDAR链直接读取LV-DOT的`tracks`和`lidar_bboxes`，其最终`/perception/{id}/observations`并不是简单地把`lidar/observations`再发布一次。

## 5. Camera-LiDAR Fusion审计

### 5.1 当前融合类型

当前不是C“仅可视化叠加”。实际实现以**A：Camera检测 + LiDAR几何定位**为主，同时保留B“目标级回退与汇合”：

```text
RGB Image -> simulation_marker Camera Detection -> Detection2D
                                               |
Filtered Mid-360 -> LV-DOT BBox/Track --------+-->
  Vision-guided LiDAR ROI: 点云投影到图像、ROI选点、局部聚类、3D框
                                               |
  Camera-LiDAR Association: 投影IoU/像素门限、Track邻近、身份信息
                                               |
             /perception/{id}/observations (map)
```

`vision_guided_lidar_roi_node.py`使用：

1. CameraInfo内参；
2. 点云消息时刻的`camera <- mid360`外参；
3. 同时刻的`map <- camera`外参；
4. `sync_slop_seconds=0.20`的相机检测匹配；
5. 相机ROI内的点云筛选、局部DBSCAN、3D框平滑与连续帧确认。

`camera_lidar_association_node.py`还会把LV-DOT 3D框投影回相机，依据IoU、像素中心距离、`pixel_gate`和`minimum_association_score`关联。其输出明确携带：

- `sensor_source=camera+lidar`、`SOURCE_CAMERA|SOURCE_LIDAR|SOURCE_FUSED`；
- `sensor_source=lidar`的LiDAR-only回退；
- `sensor_source=camera`的Camera-only伪深度回退。

### 5.2 外参是否参与计算

参与。两个融合节点均创建TF2 buffer，并按消息时间戳查询`camera_link`、`mid360_link`与`map`之间变换；没有使用固定矩阵写死外参。

### 5.3 当前限制

- `camera_detector_backend=simulation_marker`：相机检测语义来自仿真标记后端，不是训练后的真实视觉检测器。
- `enable_global_lidar_fallback=true`：ROI或同步失败时可以出现LiDAR-only或Camera-only目标；这些候选不应被误读为已完成Camera+LiDAR融合。
- Camera-only位置由已知船宽与2D框宽推深度，精度明显低于ROI-LiDAR 3D框。
- Depth图尚未用于上述几何，因此它不能纠正Camera-only伪深度，也没有参与三维框。

## 6. 三船融合审计

### 已完成

主启动为三个`vehicle_id`循环启动：LV-DOT、LV-DOT observation adapter、Camera-LiDAR fusion。随后：

```text
/perception/usv_01/observations
/perception/usv_02/observations
/perception/usv_03/observations
             |
             v
fleet_usv_perception_fusion
             |
/fleet/perception/usv_tracks
             |
fleet_global_perception_fusion
             |
/fleet/perception/fused_targets
```

第一层`perception_fusion_node.py`按时间窗、XY距离、置信度、协方差、语义和身份信息，将跨topic候选组成同一Track；位置/速度/尺寸使用置信度加权，协方差包含离散度，`source_mask`合并多个传感器来源。第二层以单一`usv_tracks`流生成稳定的舰队规范出口及兼容别名`/perception/fused/tracks`。

### 未完成或不应误判为已完成

1. 未做多雷达原始点云拼接、点云地图、ICP/NDT配准或统一占据栅格。
2. 第二层只有一个输入topic，它不是新的“三船并行融合”计算；跨船关联发生在第一层。
3. 主线设置`aggregation_wait_seconds=0.0`、`observation_history_seconds=0.0`，优先低延迟；这不等同于严格三船同时到帧的批同步。
4. 此次审计未启动场景，无法证明当前时刻三艘船均正发布、同一目标均被观测、并最终合成一个非空Track。已有文档记录的是此前在线回归。

## 7. Fleet World Model审计

`fleet_world_model_node.py`订阅：

| 输入 | 用途 |
|---|---|
| `/fleet/state` | UAV/USV状态；缺少状态时可由`map -> */base_link`生成TF只读占位 |
| `/fleet/sensor_status` | 传感器健康与频率 |
| `/fleet/perception/usv_tracks` | 第一层三船融合结果 |
| `/fleet/perception/fused_targets` | 舰队规范目标，优先作为`targets` |
| `/fleet/perception/targets` | 仍用于控制主线的ground truth回退 |
| `/tf`、`/tf_static` | TF边和静态安装关系 |

输出：

| Topic | 类型 | 内容 |
|---|---|---|
| `/fleet/world_model` | `std_msgs/String` JSON | `fleet_world_model.v1`完整态势 |
| `/fleet/world_model_summary` | `std_msgs/String` JSON | 轻量摘要 |

完整模型能表达：三个USV状态、`usv_tracks`、`fused_targets`、目标位置/速度/尺寸/类别/身份/置信度/来源、传感器状态、map-frame TF边和预测/威胁字段。

但有两个边界：

- SensorStatus目前确定来自Mid360预处理器和统一相机适配器；Depth桥本身没有独立SensorStatus，因此World Model不能单独报告“depth camera健康”。
- 当`/fleet/perception/fused_targets`为空时，World Model的顶层`targets`会回退`/fleet/perception/targets`，而该控制源当前默认是ground truth。网页或Qt必须同时看`perception.primary_source`，不能把顶层目标一概当成真实雷达融合结果。

## 8. Qt显示现状

当前Qt属于**A + C**的组合：

- **A，单船传感器切换**：`Perception Monitor`可选择USV_01/02/03，各自具有独立缓存、相机画面、原始/过滤点云、LV-DOT、Camera-LiDAR调试图层和`base_link`/`mid360_link` TF。
- **C，显示舰队融合目标**：同一画布可订阅`/fleet/perception/fused_targets`并显示Fusion Target；业务状态页则优先读`/fleet/world_model`。

它不是将三船原始点云合成一张统一雷达图的“全局点云态势”。Qt的三船选择器只切换可视化缓存，不改变算法、控制或topic。

## 9. 主要问题与风险

1. RGB与Depth内参由桥按固定FOV合成；与真实相机标定不同，未来实机需替换为真实CameraInfo与外参标定流程。
2. RGB的ROS frame由桥赋值而非SDF的`gz_frame_id`显式声明；当前物理pose与静态TF一致，但后续改模型时存在漂移风险。
3. Camera-only和LiDAR-only回退默认开启，Qt中会出现未融合候选；它们不能证明相机与雷达空间一致。
4. Depth数据未被消费，当前“RGB-D”仅是传感器可用性，不是RGB-D融合能力。
5. 三船融合按目标级定位结果关联，不处理遮挡、多视角点云配准与严格同步；远距离或相邻目标可能发生错误合并/拆分。
6. `base_station`全局frame缺失；若岸基实体需要成为坐标参考、任务约束或网页可视化对象，应在后续单独定义，不应临时复用`shore_command_base`名称。
7. Shadow模式保持安全，但也意味着当前真实感知不会驱动围捕；World Model中需始终可见`primary_source`。

## 10. 下一阶段建议（不在本次实施）

1. 做一次三船在线验收：分别确认三路点云、RGB、Depth、LV-DOT、`/perception/{id}/observations`和`/fleet/perception/fused_targets`的频率、frame与非空数据。
2. 新增独立Depth SensorStatus，并决定深度图是用于相机测距、ROI约束还是仅保留给未来算法。
3. 在固定目标船场景中记录Camera投影、ROI点、最终3D框，量化三船的`map`误差及跨船关联结果。
4. 为岸基设施正式发布`map -> base_station`静态TF，并在World Model中明确其实体ID。
5. 在不改变Shadow安全边界的前提下，为Fleet Fusion增加按来源/载具的贡献统计，便于网页端说明一个目标由哪艘船、哪类传感器支持。

## 11. 审计涉及的主要文件

- `src/uav_usv_bringup/launch/fleet_dynamic_capture.launch.py`
- `src/uav_usv_bringup/launch/fleet_dynamic_capture_live_perception.launch.py`
- `src/uav_usv_gazebo/tools/prepare_fleet_mid360.py`
- `src/uav_usv_gazebo/models/defense_own_0{1,2,3}_boat/model.sdf`
- `src/uav_usv_mission/scripts/gz_sensor_bridge.py`
- `src/uav_usv_sim/scripts/fleet_pose_tf_publisher.py`
- `src/uav_usv_perception/scripts/mid360_preprocessor.py`
- `src/uav_usv_lv_dot/src/detector_node.cpp`
- `src/uav_usv_perception/scripts/fusion/vision_guided_lidar_roi_node.py`
- `src/uav_usv_perception/scripts/fusion/camera_lidar_association_node.py`
- `src/uav_usv_perception/scripts/fusion/perception_fusion_node.py`
- `src/uav_usv_mission/scripts/fleet_world_model_node.py`
- `src/uav_usv_mission/scripts/fleet_base_station_gui.py`
