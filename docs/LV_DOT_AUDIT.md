# LV-DOT 接入审计与适配设计

本文评估 [Zhefan-Xu/LV-DOT](https://github.com/Zhefan-Xu/LV-DOT) 是否适合接入
UAV_USV 的 ROS 2 Humble 感知层，并给出不影响现有围捕主线的最小验证方案。

审计日期：2026-07-14。

上游审计基线：

```text
repository: https://github.com/Zhefan-Xu/LV-DOT
branch:     main
commit:     449bf2c960a26b067b235d82f6e0aac65fc05a6b
commit date: 2025-04-24
license:    MIT
paper:      arXiv:2502.20607
```

本阶段只审计、设计和制定测试计划。没有导入或修改 LV-DOT 源码，没有修改
`capture_manager`、`FleetCommand`、PX4、Nav2 或围捕算法。

## 结论

**LV-DOT 可以作为后续感知算法的候选核心，但不能直接放入当前主线运行。**

原因不是 Mid-360 的 `PointCloud2` 格式，而是系统边界存在三项实质差异：

1. 上游是 ROS 1 Melodic/Noetic 的 catkin 包，当前项目是 Ubuntu 22.04 + ROS 2
   Humble；上游没有 ROS 2 分支、ament 构建或 ROS 2 launch。
2. 上游完整视觉链路使用 **RGB-D** 图像，当前 UAV 标准接口只有 RGB
   `image_raw` 和 `CameraInfo`，没有深度图。
3. 上游论文及代码假设 LiDAR、RGB-D 相机和里程计属于**同一刚体载体**，使用一份
   robot pose 和固定 `body_to_camera`、`body_to_lidar` 外参。本项目应保持这个原理：
   主 LV-DOT 使用 `usv_01` 上的 Mid-360、船头相机和船体里程计；`uav_01` 相机只
   提供独立的远距离辅助观测，不参与船载传感器的固定外参计算。

因此建议分两层使用：

- 船载主感知层：在同一艘 USV 上运行 LV-DOT 的 LiDAR、视觉、融合和跟踪；
- UAV 辅助层：无人机独立产生远距离语义观测；
- 舰队层：在 `map` 坐标系按时间戳、协方差和目标关联融合船载航迹与 UAV 观测。

这属于“分布式观测融合”，不是把跨载具原始数据伪装成上游的同机融合模式。

## 1. 上游仓库审计

### 1.1 ROS 与构建系统

| 项目 | 上游实际情况 | 当前工程影响 |
| --- | --- | --- |
| ROS 版本 | ROS 1 Melodic/Noetic | 不能在 Humble 工作区直接 `colcon build` |
| Ubuntu | 18.04/20.04 | 当前主机是 Ubuntu 22.04 |
| 构建 | catkin，`catkin_make` | 需要隔离复现或移植到 ament |
| C++ | C++14，`-O3` | ROS 2 端可继续使用 |
| 主包 | `onboard_detector` | 目前只有一个 catkin package |
| 测试 | 未提供自动化测试 | 移植时必须补接口和回放测试 |
| Release | 未发布二进制 release | 应固定 Git commit，不跟随 `main` 漂移 |

上游 README 的原始编译流程为：

```bash
sudo apt install ros-noetic-vision-msgs
pip install ultralytics

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/LV-DOT.git
cd ..
catkin_make
```

源码实际还使用了 `sensor_msgs`、`geometry_msgs`、`nav_msgs`、
`visualization_msgs`、PCL、Eigen、OpenCV、cv_bridge、message_filters 和
image_transport。上游 `package.xml` 没有完整声明这些依赖，因此仅执行 README 中的
两条安装命令不一定能在干净环境中编译。

### 1.2 CUDA、GPU 与模型

LV-DOT 的 C++ 几何检测、DBSCAN、融合和 Kalman 跟踪不要求 CUDA。CUDA 只用于
Python YOLOv11 颜色检测加速：

```python
device = "cuda" if torch.cuda.is_available() else "cpu"
```

因此：

- CUDA 不是编译 LV-DOT 的硬依赖；
- 没有 CUDA 时 YOLO 会回退 CPU，但完整实时性能不保证；
- 上游没有固定 PyTorch、Ultralytics、CUDA 或驱动版本；
- 上游随仓库提供 `yolo11n.pt`，同时保留两个旧 `.pth` 权重文件。

上游论文在 Jetson Orin NX 上给出的平均模块耗时为：

| 模块 | 平均耗时 |
| --- | ---: |
| LiDAR detection | 8.34 ms |
| Visual depth detection | 10.22 ms |
| Visual color detection | 34.15 ms |
| LiDAR-visual fusion | 0.40 ms |
| Tracking and identification | 2.29 ms |

颜色检测约接近 30 Hz，但这是论文硬件和室内数据上的结果，不是当前舰队世界的保证值。
配置中的 `time_step=0.033` 也只是 30 Hz 定时器目标，不代表实际推理频率。

当前开发机审计结果：

```text
OS:       Ubuntu 22.04.5
ROS:      Humble only，未安装 ROS Noetic
GPU:      NVIDIA GeForce RTX 4060 Laptop GPU，8 GB
driver:   535.309.01
nvcc:     未安装
PyTorch:  2.12.0+cu130
CUDA usable by PyTorch: false
PCL dev:  当前 pkg-config 未找到 pcl_common
```

当前 PyTorch 报告显卡驱动与其 CUDA 13.0 构建不兼容。开始推理验证前必须选择一个
与驱动兼容的 PyTorch wheel，或升级 NVIDIA 驱动；不能把 `nvidia-smi` 能看到 GPU
误认为 PyTorch 已能使用 GPU。

### 1.3 上游输入

| 输入 | ROS 1 类型 | 默认 topic | 代码要求 |
| --- | --- | --- | --- |
| LiDAR | `sensor_msgs/PointCloud2` | `/pointcloud` | PCL `PointXYZ`，至少需要 x/y/z |
| 深度图 | `sensor_msgs/Image` | `/camera/depth/image_rect_raw` | `16UC1`；`32FC1` 会按 scale 转为 `16UC1` |
| RGB 图 | `sensor_msgs/Image` | `/camera/color/image_raw` | YOLO 强制转为 `bgr8` |
| Pose | `geometry_msgs/PoseStamped` | `/mavros/local_position/pose` | 与深度图、LiDAR分别 ApproximateTime 同步 |
| Odom | `nav_msgs/Odometry` | `/mavros/local_position/odom` | 可替代 Pose |
| YOLO box | `vision_msgs/Detection2DArray` | `yolo_detector/detected_bounding_boxes` | 由仓库内 Python 节点产生 |

上游 README 中的颜色 topic 与 YAML 默认值有一处差异：README 写
`/camera/color/image_rect_raw`，当前 YAML 和 Python 节点实际使用
`/camera/color/image_raw`。接入时应以参数或 remap 为准，不依赖 README 文本。

### 1.4 CameraInfo 与标定

上游**不订阅** `sensor_msgs/CameraInfo`。以下值直接写在 YAML：

- depth `fx, fy, cx, cy`；
- color `fx, fy, cx, cy`；
- image width/height；
- depth scale；
- `body_to_camera_depth` 4x4 矩阵；
- `body_to_camera_color` 4x4 矩阵；
- `body_to_lidar` 4x4 矩阵。

当前项目已经发布标准 `CameraInfo`，所以未来适配器应从消息读取标定并做一致性检查，
不应复制一套容易失效的手写内参。若为了复现原版必须生成 YAML，也应由标定导出工具
一次性生成，而不是在 launch 中猜测焦距。

### 1.5 TF 与时间同步

上游没有使用 tf2 查询传感器在消息时刻的姿态。它使用：

```text
world_T_sensor = world_T_body(pose/odom) * body_T_sensor(YAML)
```

并把多数点云和 Marker 输出的 `frame_id` 固定写成 `map`。这对同一载体上的固定传感器
成立，但对 `uav_01/camera_link` 与 `usv_01/mid360_link` 不成立。

上游同步关系是：

```text
depth Image    + body Pose/Odom  -> ApproximateTime
LiDAR cloud    + body Pose/Odom  -> ApproximateTime
RGB Image      -> 独立 subscriber
YOLO boxes     -> 独立 subscriber
```

RGB、YOLO box、LiDAR 和深度结果没有放进同一个 message_filters 同步器。当前项目的
相机约 8.5 Hz、Mid-360 约 10 Hz，不能继续假设固定 `dt=0.033 s`；ROS 2 移植必须
使用消息时间戳计算跟踪步长，并在每条观测的时间戳查询 TF。

### 1.6 上游输出与跟踪接口

上游没有发布可直接替代 `TrackedObjectArray` 的结构化 track topic。主要输出包括：

- 多组 `visualization_msgs/MarkerArray`：LiDAR、视觉、过滤、跟踪、动态框和轨迹；
- 多组 `sensor_msgs/PointCloud2`：过滤点云、聚类、动态点云；
- YOLO 检测图和 `vision_msgs/Detection2DArray`；
- `onboard_detector/get_dynamic_obstacles` ROS 1 service。

服务请求与响应为：

```text
request:
  geometry_msgs/Point current_position
  float64 range

response:
  geometry_msgs/Vector3[] position
  geometry_msgs/Vector3[] velocity
  geometry_msgs/Vector3[] size
```

服务返回的是 `dynamicBBoxes_`，按与请求位置的距离排序。它没有 header、时间戳、
稳定 track ID、置信度、类别或协方差；目标顺序还会随距离变化。因此不能把数组下标
当成 track ID。Marker 也只适合显示，不应成为任务层数据接口。

上游跟踪内部采用特征关联和常加速度 Kalman filter，状态包含位置、速度和加速度；
动态分类结合 YOLO 结果、估计速度和逐点位移检查。未来 ROS 2 移植应从内部 track
直接发布结构化消息，而不是长期轮询这个有损 service。

### 1.7 检测类别与海事适配风险

仓库当前 YOLO 代码只接受：

```python
target_classes = ["person"]
```

所以即使 COCO 权重包含 `boat`，当前节点也不会把船作为动态颜色目标输出。仿真船的
视角、纹理和尺度还存在域差异，不能默认通用权重可可靠识别 `enemy_target`。

默认几何参数也针对室内人员：

- `target_object_size=[0.5, 0.5, 1.5]`；
- `max_object_size=[3.0, 3.0, 2.0]`；
- LiDAR 本地范围在源码中初始化为约 `10 x 10 x 5 m`；
- 深度最大范围默认 5 m；
- 地面/屋顶过滤采用室内高度假设。

当前演示船模型已经放大，默认最大尺寸可能直接过滤船体；海浪点、水平面、船体自身点
也与室内地面不同。海事数据验证前需要参数化范围、高度裁剪、聚类尺度和目标尺寸，
但这些属于后续算法移植，不在本阶段修改。

### 1.8 多实例能力

上游没有提供多实例编排。理论上可以把每套 detector 放进独立进程和 namespace，但
当前实现存在以下限制：

- 参数前缀固定为 `onboard_detector/...`；
- launch 将 YAML 加载到绝对 namespace `/onboard_detector`；
- Python YOLO 输入 topic 是绝对硬编码；
- 节点名、service 名和相对输出名默认相同；
- 每实例会重复加载一份 YOLO 模型，占用独立 GPU 显存；
- 没有生命周期、资源调度或多相机批处理。

因此“可以启动多个进程”不等于“已支持舰队多实例”。第一轮只允许一个 LV-DOT
实例；多载具扩展应先完成 namespace、参数、模型共享和资源上限设计。

## 2. 与当前 UAV_USV 接口对照

### 2.1 已满足的接口

| 当前接口 | 状态 | 与 LV-DOT 的关系 |
| --- | --- | --- |
| `/fleet/uplink/usv_01/mid360/points` | 已完成 | 标准 PointCloud2，可作为原始 LiDAR 输入 |
| `/perception/usv_01/points_filtered` | 已完成 | 已去 NaN/Inf、裁距、自身裁剪和体素降采样 |
| `/fleet/uplink/usv_01/camera` | 已完成 | 船头 RGB 原始图像，可作为船载颜色检测入口 |
| `/fleet/uplink/uav_01/camera/image_raw` | 已完成 | UAV 远距离辅助语义观测入口 |
| `/fleet/uplink/uav_01/camera/camera_info` | 已完成 | UAV 辅助观测可读取真实内参 |
| `map -> usv_01/base_link -> mid360_link` | 已完成 | LiDAR 观测可变换到 map |
| `map -> uav_01/base_link -> camera_link` | 已完成 | 图像观测可使用消息时刻相机位姿 |
| `/fleet/sensor_status` | 已完成 | 可继续报告适配器健康状态 |
| `TrackedObjectArray` | 已完成 | 最终统一输出，无需新增核心消息 |

### 2.2 尚不满足的接口

| 缺口 | 影响 |
| --- | --- |
| USV 船头相机只有 RGB，没有 depth image | 原版视觉深度检测和完整 combined mode 不能运行 |
| USV 相机没有标准 CameraInfo 和完整 camera TF | 不能可靠生成船载固定外参和投影矩阵 |
| 只有 ROS 2 Humble | 上游 catkin/ROS 1 节点不能原生启动 |
| 当前相机分辨率 240x135、约 8.5 Hz | 小型远距离船只的 YOLO 检测风险高 |
| 上游只筛选 person | 不会输出 enemy vessel 的颜色检测结果 |
| 上游 service 丢失 track 元数据 | 不能无损映射到 `TrackedObjectArray` |
| 当前 PyTorch CUDA 不可用 | YOLO 会回退 CPU，可能影响完整舰队实时性 |

### 2.3 PointCloud2 兼容性

当前过滤点云包含 `x/y/z/intensity` 浮点字段。上游通过 PCL 读取为 `PointXYZ`，会忽略
额外 intensity 字段，因此基本格式兼容。LV-DOT 不依赖 Livox `CustomMsg`、逐点时间
或 IMU，这一点适合当前标准 `PointCloud2` 出口。

但必须保持：

- `header.stamp` 是有效 ROS 时间；
- `header.frame_id=usv_01/mid360_link`；
- 在消息时间查询 `map <- mid360_link`；
- 算法内部只变换一次，不能把已经位于 map 的点云再次应用传感器外参。

## 3. 目标适配架构

### 3.1 原版同载体架构

原版 LV-DOT 的数据关系是：

```text
same robot body pose
  +-- fixed body_T_rgbd
  |     +-- RGB image -> YOLO 2D dynamic boxes
  |     `-- Depth image -> 3D visual boxes
  `-- fixed body_T_lidar
        `-- PointCloud2 -> 3D LiDAR boxes

3D visual boxes + 3D LiDAR boxes + 2D color boxes
  -> detection fusion
  -> association + Kalman tracking + dynamic classification
```

### 3.2 船载主感知 + UAV 辅助感知架构

建议在 `usv_01` 保持原版同载体融合原理，再把 UAV 作为上层辅助源：

```text
usv_01/base_link + usv_01 odometry
  +-- fixed base_T_mid360
  |     `-- Mid-360 -> mid360_preprocessor -> filtered cloud
  `-- fixed base_T_usv_camera
        `-- USV RGB-D camera -> RGB + depth + CameraInfo

co-located USV sensors
  -> ROS 2 LV-DOT core
  -> /perception/usv_01/lvdot_tracks (TrackedObjectArray)
                                      \
                                       -> fleet_track_fusion
                                      /     -> /perception/lvdot/tracks
uav_01 RGB camera                     /      -> TrackedObjectArray
  -> uav_camera_adapter
  -> maritime semantic detector
  -> image-to-map geolocation
  -> /perception/uav_01/observations

/perception/ground_truth/tracks ----\
/perception/lvdot/tracks ------------> perception_source_mux
/perception/hybrid/tracks -----------/       |
                                              v
                                /fleet/perception/targets
                                              |
                                      capture_manager
```

重要边界：

- `LV-DOT adapter` 属于 `uav_usv_perception`；
- 船载相机、Mid-360 和 USV odometry 使用同一个 `usv_01/base_link` 与固定安装外参；
- UAV 辅助节点不进入 LV-DOT 原始点云/图像融合内部；
- `perception_source_mux` 只选来源，不生成控制命令；
- `capture_manager` 仍只订阅 `/fleet/perception/targets`；
- 感知节点不得发布 `FleetCommand`、PX4 topic、Nav2 action 或 Gazebo cmd_vel；
- 真值和传感器输出必须先在不同 topic 并行运行，验证后才能切换 mux。

### 3.3 船载原始融合与 UAV 辅助融合的边界

船载 Mid-360 与船载相机固定安装在 `usv_01/base_link`，可以保持上游原理：

```text
T_camera_mid360
  = inverse(T_base_camera) * T_base_mid360
```

这是固定标定值，可以用于点云到图像投影和 LiDAR-visual detection fusion。船体运动时
只需用 USV 在消息时刻的 `map -> base_link` 位姿把检测结果变换到世界坐标。

UAV 辅助相机则不能进入这份固定外参。`uav_01` 和 `usv_01` 的相对位姿随时间变化：

```text
T_camera_lidar(t)
  = inverse(T_map_camera(t)) * T_map_lidar(t)
```

它不是 YAML 中的一份常量。若把当前时刻的动态变换冒充固定外参，异步消息、网络延迟
和载具运动会使投影框错位。正确做法是：

1. 每个传感器按自己的 `header.stamp` 查询 TF；
2. 先在各自载体上产生带协方差的目标观测；
3. 把观测变换到 `map`；
4. 使用时间窗、空间门限、类别和速度做航迹级关联；
5. 在世界坐标进行异步融合。

因此本方案明确分工：

- USV 内部：原始数据级 LV-DOT 融合；
- UAV 与 USV 之间：目标/航迹级辅助融合；
- UAV 观测失效时：船载 LV-DOT 仍可独立运行；
- 船载目标暂时被遮挡时：UAV 观测可延长航迹或提示搜索区域。

## 4. 统一感知输出

不新增核心消息。最终输出继续使用：

```text
uav_usv_interfaces/msg/TrackedObjectArray
```

建议候选输出 topic：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `/perception/uav_01/observations` | `TrackedObjectArray` | UAV 视觉观测/航迹 |
| `/perception/usv_01/observations` | `TrackedObjectArray` | USV LiDAR 观测/航迹 |
| `/perception/lvdot/tracks` | `TrackedObjectArray` | LV-DOT/分布式融合候选输出 |
| `/perception/ground_truth/tracks` | `TrackedObjectArray` | Gazebo 真值候选输入 |
| `/fleet/perception/targets` | `TrackedObjectArray` | mux 后的冻结任务接口 |

字段映射原则：

| `TrackedObject` 字段 | 来源/规则 |
| --- | --- |
| `header.frame_id` | 固定 `map` |
| `track_id`、`uuid` | 由跟踪器稳定生成，不能使用数组下标 |
| `first_seen`、`last_update` | 观测时间，不使用回调到达时间替代 |
| `source_mask` | LiDAR=`1`、Camera=`2`、融合=`8`，可按位组合 |
| `classification` | 船只为 `CLASS_VESSEL`；未知时不得强行标船 |
| `pose`、`twist` | map 坐标及对应协方差 |
| `dimensions` | 3D bbox；未知时给出显式不确定性策略 |
| `confidence` | 检测、关联和航迹置信度综合值 |

上游 service 不足以提供上述全部字段。短期审计 wrapper 可以用于验证位置、速度和尺寸，
但生产适配必须从内部跟踪对象发布稳定 ID、时间戳和协方差。

## 5. ROS 2 移植边界

推荐建立独立 fork，保留上游 commit 作为基线，不把 ROS 1 源码直接复制进当前 mission
包。后续实现时至少需要：

1. `catkin` 改为 `ament_cmake`；
2. `roscpp/rospy` 改为 `rclcpp/rclpy`；
3. ROS 1 launch XML 改为 ROS 2 launch；
4. 更新 `vision_msgs/Detection2DArray` 的 ROS 2 字段访问；
5. 更新 message_filters、cv_bridge、PCL 转换接口；
6. 参数改为节点私有 ROS 2 parameters，禁止绝对硬编码；
7. 使用 tf2 按消息时间获取传感器位姿；
8. 使用实际时间戳计算 Kalman `dt`；
9. 增加 `TrackedObjectArray` publisher；
10. 增加 lifecycle/健康状态、超时、输入频率和处理耗时；
11. 把 indoor/person 参数改成可配置 marine profile；
12. 增加 rosbag2 回放和确定性接口测试。

原型期可以用 ROS 1 Noetic 容器和 ros1_bridge 做上游基线复现，但不建议把 bridge 作为
最终主线：当前主机没有 Noetic，消息和 service 需要双侧构建，GPU、GUI、时间和 TF
调试边界也会变复杂。

## 6. 依赖安装方案

### 6.1 上游基线复现环境

使用 Ubuntu 20.04 + ROS Noetic 容器或独立环境，固定上游 commit。完整依赖至少包括：

```text
ROS Noetic:
  roscpp rospy sensor_msgs geometry_msgs nav_msgs visualization_msgs
  cv_bridge message_filters image_transport vision_msgs
  pcl_ros pcl_conversions message_generation message_runtime

System:
  build-essential cmake libpcl-dev libeigen3-dev

Python:
  numpy opencv-python torch ultralytics
```

验证顺序：

```bash
catkin_make
roslaunch onboard_detector run_detector.launch
rosservice call /onboard_detector/get_dynamic_obstacles ...
```

依赖应写入锁定文件或容器镜像；不要在当前 ROS 2 Python 环境直接执行未固定版本的
`pip install --upgrade`。

### 6.2 ROS 2 Humble 原生适配环境

建议依赖：

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-message-filters \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  ros-humble-tf2-ros \
  ros-humble-vision-msgs \
  libeigen3-dev libpcl-dev
```

YOLO 放入单独 Python venv 或容器，先验证：

```bash
python3 -c 'import torch; print(torch.cuda.is_available())'
python3 -c 'from ultralytics import YOLO; print("ultralytics ready")'
```

只有第一条输出 `True` 后，才记录 GPU 推理性能。版本选择应依据当前 NVIDIA 驱动兼容
矩阵，不应继续使用现在无法初始化 CUDA 的 `torch 2.12.0+cu130` 环境做性能验收。

## 7. 最小验证场景

场景只使用：

```text
uav_01 + usv_01 + enemy_target
```

不启动其余 3 架 UAV 和第 2 艘 USV，以便把算法问题与舰队负载分开。

### Gate 0：冻结主线基准

```text
perception_source=ground_truth
```

记录围捕状态、目标轨迹、PX4/USV 状态和 `/fleet/perception/targets` 摘要。此结果是所有
后续测试的回退基线。

### Gate 1：上游可复现性

在隔离的 ROS 1 Noetic 环境播放上游 corridor rosbag，验证原 commit 能编译、产生
动态 box、点云和 service 输出。此步骤不连接 UAV_USV。

通过标准：上游自己的数据集可运行；否则先解决依赖，不能把错误归因于本项目接口。

### Gate 2：USV LiDAR-only

```text
/perception/usv_01/points_filtered
  + usv_01 pose/TF
  -> ROS 2 LV-DOT LiDAR branch
  -> /perception/usv_01/observations
```

验证 `enemy_target` 的 3D位置、速度、尺寸、稳定 track ID 和消息时间戳。先不接
`/fleet/perception/targets`。

### Gate 3：USV 同载体视觉入口

```text
/fleet/uplink/usv_01/camera/image_raw
  + /fleet/uplink/usv_01/camera/depth/image_raw
  + USV camera_info
  + fixed usv_01/base_link -> camera_link TF
  -> LV-DOT visual depth + color branch
```

当前船头只有 RGB `camera`。本 Gate 开始前需要把船载传感器升级为 RGB-D，或明确只
验证 `LiDAR + RGB color` 的裁剪版本。若没有 depth，不能把结果称为完整原版 LV-DOT
combined mode。

通过标准：船载 RGB、depth、CameraInfo、Mid-360 与 USV odometry 同一时间域；固定
外参投影正确；目标框能识别船只；空检测不会导致节点崩溃。

### Gate 4：UAV 辅助视觉入口

```text
/fleet/uplink/uav_01/camera/image_raw
  + UAV camera_info
  + map -> uav_01/camera_link
  -> maritime semantic detector
  -> image-to-map geolocation
  -> /perception/uav_01/observations
```

UAV 负责远距离发现、类别和搜索区域提示，不直接向 LV-DOT 提供“固定外参相机”。单目
图像的 3D 定位需要水面平面求交、目标尺寸先验或其他测距信息，并在消息中体现较大的
位置协方差。

### Gate 5：船载航迹与 UAV 辅助观测融合

只有船载 LV-DOT 和 UAV 辅助入口各自通过后，才在 `map` 中关联两类观测。测试：

- 同一目标的时间差门限；
- UAV/USV 运动时关联不跳变；
- 单个传感器超时后降级为另一来源；
- track ID 不因观测顺序改变；
- 输出协方差随单源/融合状态变化。

输出先进入 `/perception/lvdot/tracks`，与真值并排显示和记录误差。

### Gate 6：Qt 与只读演示

Qt 订阅轻量 `TrackedObjectArray` 和 `SensorStatus`：

- 显示 target ID、来源、位置、速度、置信度和更新时间；
- 不在 Qt 主线程处理点云或运行 YOLO；
- topic 缺失、空目标和算法退出时界面不崩溃。

### Gate 7：来源切换回归

通过 mux 切换：

```text
ground_truth -> /fleet/perception/targets
lvdot        -> /fleet/perception/targets
hybrid       -> /fleet/perception/targets
```

先切回 `ground_truth` 证明原围捕行为完全恢复，再在人工确认后测试 `lvdot`。来源切换不
允许修改 `capture_manager` 的消息类型或控制输出。

## 8. 验证命令模板

未来实现完成后使用以下命令验证；本阶段没有创建这些节点：

```bash
# 输入
ros2 topic hz /fleet/uplink/uav_01/camera/image_raw
ros2 topic echo /fleet/uplink/uav_01/camera/camera_info --once
ros2 topic hz /perception/usv_01/points_filtered

# TF
ros2 run tf2_ros tf2_echo map uav_01/camera_link
ros2 run tf2_ros tf2_echo map usv_01/mid360_link

# 候选感知输出
ros2 topic hz /perception/lvdot/tracks
ros2 topic echo /perception/lvdot/tracks --once

# 冻结任务接口
ros2 topic info /fleet/perception/targets -v
ros2 topic echo /fleet/perception/targets --once
```

需要记录：输入频率、端到端延迟、track 数量、ID 连续性、位置/速度 RMSE、CPU、GPU、
显存、ROS 带宽和 Gazebo real-time factor。

## 9. 不影响现有围捕的证明

本阶段仓库修改范围只有本文件。当前控制链仍为：

```text
target_tracker
  -> /fleet/perception/targets (TrackedObjectArray)
  -> capture_manager
  -> FleetCommand
  -> UAV/USV agents
  -> PX4/Nav2
```

当前 `fleet_dynamic_capture.launch.py` 的 `perception_source` 明确仍是预留参数，默认
`ground_truth`；`capture_manager` 继续订阅 `/fleet/perception/targets`。本次没有新增
publisher、subscriber、service、action、launch process 或运行时依赖，因此不会改变
4 UAV + 2 USV 围捕、PX4 Offboard、Nav2、Mid-360 或 Qt 的现有行为。

未来接入也必须满足：

1. LV-DOT 先以 shadow mode 发布候选 topic；
2. 与真值对比通过后再由 mux 选择来源；
3. mux 的默认值始终为 `ground_truth`；
4. 感知节点不获得控制 lease，不发布 `FleetCommand`；
5. 任何算法异常都可通过停掉适配节点并切回真值恢复主线。

## 10. 风险与决策

| 风险 | 严重度 | 决策 |
| --- | --- | --- |
| ROS 1 到 ROS 2 移植量 | 高 | 先隔离复现，再建立 ROS 2 fork |
| USV RGB 缺少 depth | 高 | 船载补 RGB-D，或明确验证 LiDAR+RGB 裁剪模式 |
| UAV 辅助视角无固定外参 | 高 | 仅做世界坐标航迹级融合，不进入船载原始融合 |
| 默认只检测 person | 高 | 建立海事类别/仿真数据测试，配置 boat/target 类 |
| 室内参数过滤大型船 | 高 | 建立 marine profile，按模型尺寸和海浪重新标定 |
| 上游 service 丢 track 元数据 | 中 | ROS 2 端从内部 tracker 发布 TrackedObjectArray |
| CUDA 当前不可用 | 中 | 修复驱动/PyTorch 兼容后再测实时性 |
| 低分辨率下视 UAV 图像 | 中 | 先量化召回率，再决定分辨率和相机姿态 |
| 多实例显存和命名冲突 | 中 | 第一阶段只跑单实例，后续做 namespace/资源预算 |
| 上游无自动测试 | 中 | 用 rosbag2、接口契约和真值误差回归补齐 |

## 11. 阶段验收判断

| 问题 | 结论 |
| --- | --- |
| LV-DOT 能否直接在当前 ROS 2 Humble 编译？ | 不能，需要 ROS 2 移植或 ROS 1 隔离环境 |
| 当前 Mid-360 点云能否供其 LiDAR 分支使用？ | 格式可用，需要 ROS 2/PCL 适配和 TF 时间校验 |
| 当前船载 RGB 能否进入 YOLO？ | 可以，但需标准 CameraInfo/TF 和船只类别 |
| 当前船载相机能否运行原版视觉 3D 检测？ | 不能，现有 sensor 缺少 depth image |
| 船载 RGB-D 与船载 Mid-360 能否保持原版固定外参融合？ | 可以，这是推荐主链路 |
| UAV RGB 应怎样辅助？ | 独立定位后在目标/航迹层融合，不进入船载固定外参 |
| 最终能否输出现有 `TrackedObjectArray`？ | 可以，无需新增核心消息 |
| capture_manager 是否需要理解感知来源？ | 不需要，继续只接收冻结 topic |
| 是否适合成为未来多载具感知核心？ | 有条件适合；需 ROS 2、海事目标和分布式融合改造 |

## 参考资料

- [LV-DOT GitHub 仓库](https://github.com/Zhefan-Xu/LV-DOT)
- [LV-DOT 论文，arXiv:2502.20607](https://arxiv.org/abs/2502.20607)
- [当前 UAV 相机接口](UAV_CAMERA_INTERFACE.md)
- [当前 Mid-360 接口](SENSOR_INTERFACE.md)
- [围捕接口冻结说明](CAPTURE_INTERFACE_FREEZE_V1.md)
