# LV-DOT ROS 2 原生迁移计划

本文给出 LV-DOT 从 ROS 1 Noetic 隔离环境迁移到 UAV_USV ROS 2 Humble
主线的工程方案。本文只做源码边界分析、接口设计、阶段计划和回滚设计，**没有开始
ROS 2 迁移编码，也没有删除现有 ROS 1 Shadow Mode**。

审计日期：2026-07-15。

上游冻结基线：

```text
repository:  https://github.com/Zhefan-Xu/LV-DOT
commit:      449bf2c960a26b067b235d82f6e0aac65fc05a6b
commit date: 2025-04-24
license:     MIT
```

当前 UAV_USV 安全边界保持不变：

- `perception_source=ground_truth`；
- `capture_manager`、`FleetCommand`、PX4、Nav2 和 UAV/USV agent 未修改；
- ROS 1 容器版只作为旁路检测器；
- `/fleet/perception/targets` 仍是任务层唯一目标输入；
- ROS 2 原生版达到等价验收前，不删除 TCP 桥和 ROS 1 镜像。

## 1. 迁移结论

**建议迁移，但只建议立即开始 Phase 1，不建议现在全面重写或切换控制源。**

推荐采用“方案 B：ROS 2 原生分层迁移”：

```text
uav_usv_lv_dot_core
  纯 C++ 算法库，不依赖 ROS 消息、ROS 时间、TF 或发布器

uav_usv_lv_dot_ros2
  ROS 2 生命周期节点，负责订阅、参数、TF、时间、诊断和消息转换
```

原因如下：

1. 上游 2720 行的 `dynamicDetector.cpp` 同时承担通信、定时调度、坐标变换、
   聚类、关联、Kalman 跟踪、动态判断和可视化，直接把 `ros::` 替换成
   `rclcpp::` 只能得到“能编译的 ROS1 结构”，不能解决固定步长、线程安全、
   多实例和结构化输出问题。
2. DBSCAN、Kalman、LiDAR 聚类和大部分几何关联本身与 ROS 无关，适合逐段抽取并
   用同一批 rosbag2 做等价回放。
3. 当前真实仿真验收已建立 ROS 1 基准：三种目标运动均持续产生动态框，检测率
   100%，平均位置误差约 1.05--1.25 m。现在具备迁移对照条件。
4. 当前容器链路已经可用，因此没有必要一次性冒险替换。

迁移完成前的正式架构仍为：

```text
ROS 2 PointCloud2 / TF
  -> TCP ingress
  -> ROS 1 LV-DOT container
  -> TCP egress
  -> lv_dot_adapter
  -> /perception/lv_dot/observations
```

目标架构为：

```text
ROS 2 PointCloud2 / CameraInfo / TF
  -> lv_dot_ros2::DetectorNode
  -> lv_dot_core
  -> TrackedObjectArray
  -> /perception/lv_dot/observations
```

## 2. 上游源码边界图

### 2.1 当前 ROS 1 结构

```text
src/detector_node.cpp
  `-- ros::init + ros::spin
       `-- dynamicDetector                         [ROS/算法混合，2720+438 行]
            +-- ROS 1 参数、subscriber、publisher、service、timer
            +-- message_filters ApproximateTime
            +-- Pose/Odom + 手写外参 -> map
            +-- depth projection / UV detection
            +-- LiDAR passthrough / DBSCAN / bbox
            +-- LiDAR-visual bbox filtering
            +-- association / Kalman / dynamic voting
            `-- Marker、PointCloud 和 service 输出

独立 Python 进程
  yolov11_detector_node.py
    `-- yolov11_detector.py
         +-- rospy Subscriber/Publisher/Timer
         `-- Ultralytics YOLO 推理
```

### 2.2 建议拆分后的边界

```text
uav_usv_lv_dot_ros2
  DetectorNode (LifecycleNode)
    +-- 参数声明与校验
    +-- PointCloud2 / Image / CameraInfo 订阅
    +-- tf2 时间戳查询
    +-- PCL/OpenCV 与 core 数据转换
    +-- TrackedObjectArray 正式输出
    +-- bbox/pointcloud/stats 诊断输出
    `-- 健康状态与输入超时
              |
              v
uav_usv_lv_dot_core
  +-- types: FrameInput, Detection, Track, DetectorResult
  +-- preprocessing
  +-- dbscan / lidar clustering
  +-- association
  +-- kalman tracking
  +-- dynamic classification
  `-- optional visual fusion interface
```

原则是 core 接收普通 C++ 数据和显式的 `stamp_seconds`、`dt`、传感器位姿，
返回结构化检测结果；core 不创建节点、不查询参数、不发布 Marker，也不调用
`now()`。

## 3. 文件级迁移清单

上游本次审计的 C++、Python 主体约 5625 行，其中 `dynamicDetector.cpp` 一项占
2720 行，是主要风险来源。

| 上游文件 | 当前职责和耦合 | 迁移决定 | 目标位置 |
| --- | --- | --- | --- |
| `src/detector_node.cpp` | `ros::init`、NodeHandle、`ros::spin` | 不复用；建立 ROS 2 生命周期入口 | `uav_usv_lv_dot_ros2/src/detector_node.cpp` |
| `dynamicDetector.h/.cpp` | 通信、定时器、TF假设、检测、跟踪、动态判断、发布和 service 全部混合 | 不整文件机械移植；按阶段抽取纯算法方法 | 两个新包分别承接 core 与 node |
| `dbscan.h/.cpp` | 只依赖 STL 和数学，纯算法 | 基本原样移植，清理宏和 `using namespace`，补单元测试 | `uav_usv_lv_dot_core/src/dbscan.cpp` |
| `kalmanFilter.h/.cpp` | 只依赖 Eigen，纯算法 | 复用公式；接口增加显式 `dt` 和初始化状态检查 | `uav_usv_lv_dot_core/src/kalman_filter.cpp` |
| `lidarDetector.h/.cpp` | PCL、Eigen、DBSCAN；头文件有不必要的 `ros/ros.h` | 去掉 ROS include，改成输入 cloud、输出 cluster/bbox 的纯类 | `uav_usv_lv_dot_core/src/lidar_detector.cpp` |
| `uvDetector.h/.cpp` | OpenCV、Eigen、Kalman 和 `box3D`；主要算法无 ROS | 作为后续可选视觉深度模块迁移；LiDAR 第一阶段不阻塞于它 | `uav_usv_lv_dot_core/src/uv_detector.cpp` |
| `utils.h` | `box3D` 与几何算法混在 ROS1 geometry/tf2 转换中 | 拆为纯 `types.hpp`/`geometry.hpp`；消息转换放 ROS2 包 | core + ros2 conversion |
| `fakeDetector.h/.cpp` | ROS1假目标、Marker和固定 0.033 s 定时器 | 不迁移；现有 Gazebo 真值与 rosbag 已覆盖用途 | 无 |
| `src/fake_detector_node.cpp` | fakeDetector ROS1入口 | 不迁移 | 无 |
| `cfg/detector_param.yaml` | 绝对 topic、固定外参、固定 `time_step` | 转为 ROS2 参数 YAML；topic 用 remap，外参优先用 TF | `uav_usv_lv_dot_ros2/config/*.yaml` |
| `launch/run_detector.launch` | ROS1 XML、绝对 `/onboard_detector` 参数空间 | 重写 Python launch，全部支持 namespace/remap | `uav_usv_lv_dot_ros2/launch/*.launch.py` |
| `srv/GetDynamicObstacles.srv` | 返回无 header/ID/confidence 的位置速度数组 | 第一阶段不迁移；正式接口使用 `TrackedObjectArray` | 无；如有查询需求另行评审 |
| `rviz/*.rviz` | ROS1 Marker/点云显示配置 | 只借鉴显示项，改成 ROS2 诊断 topic | ROS2 包 rviz 配置 |
| `yolov11_detector.py` | rospy、CvBridge、绝对相机 topic、三个 30 Hz timer、只接受 `person` | 算法调用可参考；ROS外壳重写，海事类别和模型单独验证 | 可选 `uav_usv_lv_dot_vision` |
| `yolov11_detector_node.py` | rospy入口 | 不复用 | rclpy/rclcpp 新入口 |
| `yolo_detector.py` | 旧 ShuffleNet/PyTorch 检测实现与 rospy 混合 | 不进入 LiDAR 首轮迁移；保留上游对照 | 可选后续插件 |
| `scripts/.../models`、`weights` | 模型代码和权重 | 代码、权重分别核对许可证与来源后再引入 | 外部模型目录或下载脚本 |
| `CMakeLists.txt` | catkin、message_generation、PCL | 重写为 ament_cmake；core 与 ROS2 节点独立 target | 两个新 CMake |
| `package.xml` | ROS1依赖声明不完整 | 新建 format 3 清单并完整声明依赖 | 两个新 package.xml |

### 3.1 可直接复用的算法

- DBSCAN 聚类；
- PCL 点云聚类后的中心、尺寸和 PCA 几何计算；
- bbox IOU 和特征生成；
- 历史框关联与线性预测思路；
- Kalman 状态模型和测量更新；
- 动态速度阈值、投票和连续性判断；
- UV 深度图检测算法，可在 LiDAR 路径稳定后迁移。

### 3.2 必须重写的通信外壳

- ROS 1 subscriber/publisher/service/timer；
- `message_filters` 中 Pose/Odom 与传感器同步方式；
- 参数读取和绝对 topic；
- PCL/ROS 消息转换；
- TF 和消息时间管理；
- Marker、PointCloud2 与 `TrackedObjectArray` 发布；
- Python YOLO 的 rospy 外壳；
- 节点启动、命名空间、生命周期和健康状态。

## 4. 三种迁移方案比较

| 维度 | A：ROS2薄封装 | B：ROS2原生分层迁移 | C：继续ROS1容器 |
| --- | --- | --- | --- |
| 做法 | 机械替换 ROS API，保留巨型类 | 抽取纯算法 core，重建 ROS2 node | 保留当前 TCP + Noetic |
| 初期工作量 | 中 | 高 | 低 |
| 回归风险 | 高，定时和共享状态问题会被保留 | 可按模块和数据包逐阶段控制 | 当前已知风险最低 |
| 可测试性 | 低 | 高，可对 core 做确定性测试 | 算法内部难做自动化单测 |
| 时间/TF正确性 | 仍需大量补丁 | 从接口层统一设计 | 保留跨系统时钟和桥延迟 |
| 多实例 | 进程可复制，但硬编码多 | namespace、参数和资源原生支持 | 端口、容器、显存管理复杂 |
| 性能 | 少一层桥，但巨型节点仍难优化 | 可减少复制并按阶段测量 | 有序列化和TCP开销 |
| 维护成本 | 高 | 中，边界清楚 | 高，长期维护双ROS环境 |
| 适合作为最终主线 | 不推荐 | **推荐** | 不推荐，仅保留基线 |

明确推荐方案 B。方案 C 在迁移期间保留，承担回归基准和故障回退；方案 A 不作为
中间成果，因为它会消耗迁移成本，却不建立稳定的算法边界。

## 5. ROS 2 节点与数据流设计

### 5.1 单实例

```text
/perception/usv_01/points_filtered  [PointCloud2]
                    |
                    v
 /usv_01/lv_dot/detector            [LifecycleNode]
   +-- tf2: map <- usv_01/mid360_link @ cloud stamp
   +-- cloud conversion
   +-- lv_dot_core::process_lidar(frame)
   +-- track conversion
   +-- health/statistics
   |
   +--> /perception/lv_dot/observations       [TrackedObjectArray, official]
   +--> diagnostics/lidar_bboxes              [MarkerArray]
   +--> diagnostics/tracked_bboxes            [MarkerArray]
   +--> diagnostics/dynamic_bboxes            [MarkerArray]
   +--> diagnostics/points                    [PointCloud2, optional]
   `--> diagnostics/statistics                [diagnostic_msgs/DiagnosticArray]
```

`MarkerArray` 只用于显示和对照，不再通过 Marker 反推任务目标。

### 5.2 与现有感知层连接

```text
ROS1 baseline / ROS2 native candidate
                  |
                  v
/perception/lv_dot/observations
                  |
                  v
perception_fusion_node
                  |
                  v
perception_source_mux  [仍配置 ground_truth]
                  |
                  v
/fleet/perception/targets
                  |
                  v
capture_manager
```

迁移验证阶段，ROS1版与ROS2版必须发布到不同候选 topic，避免互相覆盖：

```text
/perception/lv_dot_ros1/observations
/perception/lv_dot_ros2/observations
```

等价验收后，launch 再将选中的后端 remap 到冻结接口
`/perception/lv_dot/observations`。

## 6. Topic、QoS 和参数设计

### 6.1 Topic

所有节点内部 topic 名使用相对名称，舰队级出口由 launch remap；禁止在源码中写死
`usv_01`、`/onboard_detector`、相机、点云或 service 的绝对名称。

| 方向 | 相对接口 | 类型 | 建议 QoS |
| --- | --- | --- | --- |
| 输入 | `points` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS，depth 5 |
| 可选输入 | `camera/image_raw` | `sensor_msgs/msg/Image` | SensorDataQoS |
| 可选输入 | `camera/camera_info` | `sensor_msgs/msg/CameraInfo` | reliable + transient local 或系统现有 QoS |
| 可选输入 | `vision/observations` | `vision_msgs/msg/Detection2DArray` | SensorDataQoS |
| 正式输出 | `observations` | `uav_usv_interfaces/msg/TrackedObjectArray` | reliable，depth 10 |
| 诊断 | `diagnostics/lidar_bboxes` | `visualization_msgs/msg/MarkerArray` | best effort，depth 1 |
| 诊断 | `diagnostics/tracked_bboxes` | `MarkerArray` | best effort，depth 1 |
| 诊断 | `diagnostics/dynamic_bboxes` | `MarkerArray` | best effort，depth 1 |
| 诊断 | `diagnostics/points` | `PointCloud2` | SensorDataQoS，可关闭 |
| 健康 | `diagnostics/statistics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable，depth 10 |

正式 track 输出规则：

- `header.stamp` 使用产生该估计的最新传感器时间；
- `header.frame_id` 默认为 `map`；
- `track_id` 由 core 中的稳定轨迹 ID 产生，不使用数组下标或 Marker ID；
- `source_mask` 至少置 `SOURCE_LIDAR`，视觉融合后按位增加 `SOURCE_CAMERA`；
- 海事目标默认 `CLASS_VESSEL`，无法确认时用 `CLASS_UNKNOWN`；
- `pose`、`twist`、尺寸、置信度和协方差由跟踪器或配置明确填充；
- 不能用全零协方差暗示“完全确定”。

### 6.2 参数分组

建议参数均由 ROS2 YAML 声明并校验：

```yaml
frames:
  output: map
  tf_timeout_s: 0.10

input:
  max_age_s: 0.50
  queue_depth: 5

lidar:
  local_range: [60.0, 60.0, 8.0]
  ground_height: -0.40
  roof_height: 8.0
  dbscan_epsilon: 2.4
  dbscan_min_points: 6

tracking:
  max_match_range: 8.0
  history_size: 20
  dynamic_velocity_threshold: 0.12
  dynamic_vote_threshold: 0.35
  force_dynamic_frames: 3
  min_dt_s: 0.02
  max_dt_s: 0.50

output:
  publish_diagnostics: true
  publish_debug_clouds: false
```

数值示例来自当前海事调优方向，最终值以 `LV_DOT_TUNING_REPORT.md` 和回放测试为准；
不得无记录地复制成多载具通用默认值。

## 7. 时间、TF 与执行模型

### 7.1 时间戳驱动

上游使用固定 `dt=0.033`，并用五个 timer 分别执行检测、跟踪、分类和可视化。
ROS 2 版必须改为传感器时间戳驱动：

1. 收到点云后读取 `msg.header.stamp`；
2. 计算 `dt = current_stamp - previous_accepted_stamp`；
3. 非递增时间戳直接丢弃并计数；
4. `dt` 超过配置上限时重置或扩大协方差，不假装只过去了 0.033 s；
5. Kalman 状态转移矩阵每帧按实际 `dt` 更新；
6. 诊断 timer 只发布健康状态，不驱动算法步骤。

ROS 时间用于消息和 TF，`std::chrono::steady_clock` 只用于处理耗时和墙钟超时。
节点统一支持 `use_sim_time`。

### 7.2 TF 规则

每帧按点云时间查询：

```text
map <- usv_01/mid360_link @ pointcloud.header.stamp
```

规则：

- 输入点云保留原始 `frame_id`；
- node 在进入 core 前只做一次坐标变换；
- core 明确知道输入点是在 sensor frame 还是 output frame；
- 输出统一到参数 `frames.output`，默认 `map`；
- TF 查询失败时不使用最新 TF 冒充历史 TF；该帧丢弃并上报计数；
- 相机观测使用各自图像时间戳查询相机 TF；
- 不再要求单独的 Pose/Odom topic 与 PointCloud2 做 ApproximateTime 同步。

### 7.3 线程安全

上游默认 `ros::spin()` 近似串行处理，共享大量 bbox/history 容器。ROS 2 第一版必须
使用单线程 executor 或 mutually-exclusive callback group，避免因为改成多线程而引入
数据竞争。只有 core 变成显式输入/输出、无共享发布状态后，才评估并行预处理和推理。

## 8. 多实例与命名空间

单个二进制必须支持：

```text
/usv_01/lv_dot/detector
/usv_02/lv_dot/detector
...
```

每个实例由 launch 提供：

- `vehicle_id`；
- node namespace；
- 点云、相机和输出 remap；
- output frame；
- 同一份只读算法参数；
- 可选的每载具传感器覆盖参数。

推荐每艘 USV 一个 detector 实例。LiDAR core 不使用 GPU，实例间共享二进制和 YAML；
如果后续开启 YOLO，模型加载应放到独立视觉进程或支持推理服务/批处理，不能默认每艘船
无限复制一份 GPU 模型。

生命周期建议：

- `unconfigured`：未加载参数和模型；
- `inactive`：已配置但不订阅高频传感器；
- `active`：接收数据并发布观测；
- `finalized`：释放 PCL、OpenCV 和模型资源。

首轮多实例验收只做两艘 USV，不直接扩大到完整舰队；每增加一个实例都记录 CPU、
内存、点云带宽和处理频率。

## 9. 分阶段实施计划

### Phase 0：冻结对照基线

修改文件：不新增算法代码，只固定文档、镜像 tag、参数和数据包清单。

构建目标：无。

测试：确认三份在线 bag 和六次回放结果可读，ROS1 镜像 commit 为
`449bf2c...`。

通过标准：`LV_DOT_TUNING_REPORT.md` 中的指标可重复，source mux 为 ground truth。

回滚：不需要；这是只读基线。

### Phase 1：建立 ROS 2 包和最小 PointCloud2 输入

预计修改：

```text
src/uav_usv_lv_dot_core/{CMakeLists.txt,package.xml,include,src,test}
src/uav_usv_lv_dot_ros2/{CMakeLists.txt,package.xml,include,src,config,launch,test}
```

构建目标：core 空算法接口、生命周期 detector node、PointCloud2 转换、按时间戳 TF
查询、输入频率和 TF 诊断；不输出动态目标。

测试：使用 `constant_final_20260715` 回放，检查每条点云只处理一次、TF 成功率、
frame 和时间戳。

通过标准：连续运行不崩溃；输入频率接近 10 Hz；TF 成功率达到 99%；无固定车辆名和
绝对 topic。

回滚：launch 参数 `lv_dot_backend:=ros1_container`，不影响现有容器。

### Phase 2：移植 LiDAR 聚类和诊断输出

预计修改：core 中 `dbscan`、`lidar_detector`、类型和几何测试；ROS2 node 增加
`lidar_bboxes` 与 debug cloud。

构建目标：复现点云裁剪、DBSCAN、cluster 和 bbox，不含跟踪。

测试：三份 bag 对比每帧聚类框数量、中心和尺寸。

通过标准：平均聚类框数量差异不超过 5%；关联后 bbox 中心均值差异不超过 0.20 m；
无海浪点导致的系统性爆框。

回滚：只关闭 ROS2 candidate node，ROS1不变。

### Phase 3：移植关联、Kalman 和动态判定

预计修改：core 中 association、history、Kalman、dynamic classifier；补不同 `dt`、
丢帧、转弯和加速单元测试。

构建目标：输出内部稳定 `Track`，并发布 tracked/dynamic 诊断框。

测试：三类 bag 与 ROS1 逐级对比 `tracked_bboxes`、`dynamic_bboxes`、位置和速度。

通过标准：三种场景动态框持续非空；检测率满足当前工程门槛；ID切换不多于ROS1基线
加1；平均位置差异不超过0.25 m；输出不低于5 Hz。

回滚：保留 Phase 2 聚类节点用于定位问题，正式候选仍使用ROS1输出。

### Phase 4：直接发布 TrackedObjectArray

预计修改：ROS2 message conversion、协方差策略、track ID 和 classification 映射。

构建目标：直接发布 `/perception/lv_dot_ros2/observations`，不经过 Marker adapter。

测试：运行现有 `lv_dot_shadow_evaluator`，并验证 fusion 可以订阅候选 topic。

通过标准：字段完整、时间单调、frame=`map`、track ID稳定、无NaN；正式任务输入仍为
ground truth。

回滚：停止 ROS2 observations，保留 ROS1 adapter。

### Phase 5：ROS1/ROS2同包回放等价验证

预计修改：增加双后端比较工具和报告，不改控制节点。

构建目标：同一数据包分别运行 ROS1 容器版和 ROS2 原生版。

测试：见第10节。

通过标准：阶段框、检测率、误差、ID、延迟和资源达到约定容差；两次回放可重复。

回滚：保持 `lv_dot_backend=ros1_container`。

### Phase 6：接入 source mux 的 sensor 候选源

预计修改：仅 launch/remap/配置，原则上不修改 mux 核心。

构建目标：允许人工选择 ROS2 observation 作为 sensor 候选，但默认仍 ground truth。

测试：最小 `uav_01 + usv_01 + target_vessel` 场景先做 Shadow，再由人工显式切换。

通过标准：切换前后无topic冲突；失去sensor输入可回退；控制节点不订阅debug Marker。

回滚：`perception_source:=ground_truth`。

### Phase 7：多实例验证

预计修改：namespace launch、两实例配置和资源统计工具。

构建目标：`usv_01`、`usv_02` 同时运行独立 detector。

测试：不同点云和TF不串线，输出track来源可区分，融合层可关联同一目标。

通过标准：无硬编码、无共享可写状态、无topic覆盖；频率和资源在预算内。

回滚：只启用 `usv_01` 单实例。

## 10. ROS1/ROS2 等价测试策略

必须使用任务一生成的同一批原始数据，不录入旧的 LV-DOT 最终输出作为算法输入：

| 场景 | 数据包 | 时长 |
| --- | --- | ---: |
| 匀速 | `bags/lv_dot_acceptance/constant_final_20260715` | 106.87 s |
| 转弯 | `bags/lv_dot_acceptance/turn_final_20260715` | 103.79 s |
| 加速 | `bags/lv_dot_acceptance/acceleration_final_20260715` | 185.59 s |

每个后端、每个场景至少回放两次。比较方法：

1. 使用时间窗和最近邻关联 bbox/track，不比较数组顺序；
2. track ID 可先归一化再比较，因为不同实现的 ID 数值本身没有语义；
3. 对比 `lidar_bboxes -> tracked_bboxes -> dynamic_bboxes -> observations` 每一级；
4. 记录聚类框数量、动态框数量、位置、速度、轨迹连续性和输出频率；
5. 记录 CPU、RSS、处理耗时、输入排队和 TF 失败；
6. 使用相同 rosbag 回放速率、相同预热窗口和相同参数。

ROS 1 当前对照值：

| 场景 | 检测率 | 平均位置误差 | 平均速度误差 | ID切换 | 输出频率 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 匀速 | 100% | 1.055 m | 0.196 m/s | 0 | 10.0 Hz |
| 转弯 | 100% | 1.235 m | 0.413 m/s | 1 | 10.0 Hz |
| 加速 | 100% | 1.253 m | 0.431 m/s | 2 | 10.0 Hz |

建议迁移等价门槛：

- 三种场景均持续产生非空动态框；
- 检测率不低于 ROS1 基线 5 个百分点；
- 相对 ROS1 匹配轨迹的平均位置差异不超过 0.25 m；
- 平均速度差异不超过 0.20 m/s；
- 单场景 ID切换不比ROS1多1次以上；
- 输出频率不低于5 Hz，目标仍为10 Hz；
- ROS2处理延迟不高于ROS1容器链路的120%；
- 两次回放的检测率差异不超过1个百分点；
- 无崩溃、NaN、时间倒退或TF静默回退。

如果算法因实际 `dt` 修正而与ROS1数值不同，应同时比较各自相对ground truth的误差；
不能为了逐值复刻ROS1而保留已确认的时间错误。

## 11. 依赖清单

### core 包

- C++17；
- Eigen3；
- PCL common/filters；
- OpenCV core/imgproc（仅视觉阶段）；
- GoogleTest；
- 不依赖 ROS、CUDA、Qt 或 Gazebo。

### ROS2 包

- `rclcpp`、`rclcpp_lifecycle`；
- `sensor_msgs`、`geometry_msgs`、`visualization_msgs`；
- `diagnostic_msgs`；
- `tf2_ros`、`tf2_eigen`、`pcl_conversions`；
- `cv_bridge`、`image_transport`、`vision_msgs`（视觉阶段）；
- `uav_usv_interfaces`；
- `ament_cmake`、`ament_cmake_gtest`。

### 可选视觉后端

- Python 3、PyTorch、Ultralytics、OpenCV；
- 与宿主驱动匹配的 CUDA 运行时；
- 经许可核对的权重文件。

LiDAR core 不应因未安装 CUDA 而无法构建。视觉后端必须是可选依赖。

## 12. 风险清单与缓解

| 风险 | 影响 | 缓解措施 |
| --- | --- | --- |
| `dynamicDetector` 单类过大 | 很难确认行为等价 | 按检测、关联、跟踪、分类分阶段抽取 |
| 固定 `dt=0.033` | 10 Hz点云速度估计错误 | 使用消息时间差并补可变dt测试 |
| 五个timer共享状态 | ROS2多线程下产生竞态 | 首版单线程/互斥callback group |
| 输出大量 `now()` 时间戳 | 破坏回放和时序分析 | 输出继承传感器时间，处理耗时另记 |
| 输出frame硬编码`map` | 可能重复变换或漂移 | 每帧TF查询，core显式记录输入frame |
| 本地范围和室内尺寸假设 | 海事目标被过滤 | 所有范围参数化并使用现有三类bag回归 |
| Marker/service缺少track元数据 | 任务层错误关联 | core直接输出`TrackedObjectArray` |
| Python YOLO只筛`person` | 无法识别船 | 视觉阶段单独训练/配置海事类别 |
| RGB/YOLO/LiDAR不同步 | 错误融合 | 观测带时间戳，设置最大时间差和过期策略 |
| 多实例重复加载GPU模型 | 显存耗尽 | 视觉服务共享/批处理，LiDAR实例独立 |
| 上游无自动化测试 | 重构易回归 | 对纯core增加单测并用同一bag双后端对比 |
| 上游依赖声明不完整 | 干净环境构建失败 | 新package.xml完整声明，CI从空环境构建 |
| 权重来源与许可证不清 | 分发风险 | 源码MIT与模型权重分别审计，不直接提交未知权重 |
| ROS1/ROS2两套输出互相覆盖 | 评估数据污染 | 使用`lv_dot_ros1`/`lv_dot_ros2`独立topic |
| 仿真时间暂停/跳变 | 状态发散 | 检测非单调时间并安全重置track |

## 13. 预计修改文件

开始编码后预计新增：

```text
src/uav_usv_lv_dot_core/
  CMakeLists.txt
  package.xml
  include/uav_usv_lv_dot_core/{types,dbscan,lidar_detector,tracker}.hpp
  src/{dbscan,lidar_detector,tracker}.cpp
  test/{test_dbscan,test_variable_dt,test_dynamic_classifier}.cpp

src/uav_usv_lv_dot_ros2/
  CMakeLists.txt
  package.xml
  include/uav_usv_lv_dot_ros2/{detector_node,message_conversion}.hpp
  src/{detector_node,message_conversion,main}.cpp
  config/lv_dot_maritime.yaml
  launch/lv_dot_ros2.launch.py
  test/{test_tf_timestamp,test_message_conversion}.cpp

tools/
  compare_lv_dot_backends.py

docs/
  LV_DOT_ROS2_PORT_RESULTS.md
```

可能调整但不应重写：

- `src/uav_usv_perception/launch/lv_dot_shadow.launch.py`：增加后端选择或候选remap；
- `src/uav_usv_perception/launch/perception_layer.launch.py`：只接候选输出配置；
- `src/uav_usv_perception/scripts/evaluation/lv_dot_shadow_evaluator.py`：支持双后端比较；
- 顶层 README/依赖文档：增加构建与回滚命令。

明确不应修改：

- `capture_manager`；
- FleetCommand、VehicleState、CommandAck、ControlLease 消息；
- PX4/USV agent；
- Nav2控制；
- source mux 默认值；
- 当前 ROS1 容器资产和任务一数据包。

## 14. 切换与回滚设计

建议 launch 增加显式参数：

```text
lv_dot_backend:=ros1_container   # 迁移期间默认
lv_dot_backend:=ros2_native      # 仅测试时人工选择
```

后端只决定谁提供 `/perception/lv_dot/observations`，不决定任务控制源。
`perception_source` 是另一层独立开关，迁移阶段继续保持：

```text
perception_source:=ground_truth
```

发生以下任一情况立即回滚到 ROS1 基线：

- 动态框连续2秒为空；
- TF失败率超过1%；
- 时间戳倒退或出现NaN；
- 输出频率低于5 Hz；
- 轨迹ID频繁切换；
- 节点崩溃或资源持续增长；
- ROS2候选影响控制主线。

回滚不删除数据和代码，只停止 ROS2 candidate，重新启动 ROS1 container；source mux
始终保持 ground truth，因此回滚不应改变围捕行为。

## 15. 是否立即开始编码

结论是：**可以开始 Phase 1，但不能立即进行全面迁移，也不能切换到sensor控制。**

开始 Phase 1 的前置条件已经满足：

- 上游 commit 已冻结；
- 三类真实仿真场景已经验收；
- 数据包和量化基线已经生成；
- ROS1 Shadow Mode可以作为回退；
- `TrackedObjectArray` 正式输出契约已经冻结。

Phase 1 完成并通过“PointCloud2 + 时间戳TF + namespace”验收后，必须停下来审查一次，
再决定是否进入 Phase 2。不得把包骨架可以编译视为算法迁移完成。
