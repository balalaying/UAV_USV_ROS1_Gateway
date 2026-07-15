# LV-DOT ROS 2原生迁移Phase 2报告

日期：2026-07-15。

## 1. 阶段结论

Phase 2已完成LV-DOT LiDAR聚类链路的ROS 2原生迁移：

```text
PointCloud2
  -> 精确时间戳TF
  -> ROS无关PointCloudFrame
  -> 点云预处理
  -> DBSCAN
  -> ClusterResult
  -> 轴对齐3D bbox
  -> diagnostics/lidar_bboxes (MarkerArray)
```

三份指定数据均完成1倍速全量回放：

- `constant_final_20260715`；
- `turn_final_20260715`；
- `acceleration_final_20260715`。

没有迁移或启用：

- Kalman；
- tracking；
- dynamic voting；
- vision；
- fusion；
- 感知结果控制接管。

正式观测出口仍发布完整但空的`TrackedObjectArray`。三份回放分别检查了
1060、1026和1847个观测消息，非空消息数量全部为0。

安全边界保持：

```text
perception_source=ground_truth
```

本阶段没有修改：

- `capture_manager`；
- `FleetCommand`；
- PX4；
- Nav2；
- UAV/USV agent；
- `perception_source_mux`默认逻辑；
- ROS 1 LV-DOT Docker基线。

## 2. 迁移来源与许可

算法来源：

```text
https://github.com/Zhefan-Xu/LV-DOT
commit: 449bf2c960a26b067b235d82f6e0aac65fc05a6b
```

提取边界：

| ROS 1文件 | Phase 2对应实现 |
| --- | --- |
| `onboard_detector/dbscan.h/.cpp` | `uav_usv_lv_dot_core/dbscan.hpp/.cpp` |
| `onboard_detector/lidarDetector.h/.cpp` | `uav_usv_lv_dot_core/lidar_clusterer.hpp/.cpp` |
| `dynamicDetector::lidarPoseCB`中的LiDAR预处理 | `DetectorCore::preprocess` |
| `dynamicDetector::lidarDetect`中的大框过滤 | `LidarClusterer::cluster` |

LV-DOT原项目采用MIT许可。本仓库在core包中保留：

```text
src/uav_usv_lv_dot_core/LICENSE.LV-DOT
```

package manifest标记为`Apache-2.0 AND MIT`，迁移源文件也记录了上游commit。

### 2.1 修改文件

core：

```text
src/uav_usv_lv_dot_core/CMakeLists.txt
src/uav_usv_lv_dot_core/package.xml
src/uav_usv_lv_dot_core/LICENSE.LV-DOT
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/types.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/detector_core.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/dbscan.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/lidar_clusterer.hpp
src/uav_usv_lv_dot_core/src/detector_core.cpp
src/uav_usv_lv_dot_core/src/dbscan.cpp
src/uav_usv_lv_dot_core/src/lidar_clusterer.cpp
src/uav_usv_lv_dot_core/test/test_detector_core.cpp
```

ROS 2外壳：

```text
src/uav_usv_lv_dot_ros2/CMakeLists.txt
src/uav_usv_lv_dot_ros2/package.xml
src/uav_usv_lv_dot_ros2/config/lv_dot_phase1.yaml
src/uav_usv_lv_dot_ros2/include/uav_usv_lv_dot_ros2/detector_node.hpp
src/uav_usv_lv_dot_ros2/include/uav_usv_lv_dot_ros2/cluster_marker_conversion.hpp
src/uav_usv_lv_dot_ros2/src/detector_node.cpp
src/uav_usv_lv_dot_ros2/src/cluster_marker_conversion.cpp
src/uav_usv_lv_dot_ros2/test/test_phase1_conversions.cpp
```

验证和文档：

```text
tools/lv_dot/run_phase2_replay.sh
tools/lv_dot/sample_phase2_resources.py
tools/lv_dot/compare_phase2_clusters.py
docs/LV_DOT_ROS2_PHASE2_REPORT.md
```

## 3. 软件边界

### 3.1 uav_usv_lv_dot_core

新增核心类型：

- `LidarCluster`：cluster id、质心、三轴尺寸、点数、时间戳；
- `ClusteringStatistics`：输入、有效、聚类和噪声点数，以及预处理/聚类耗时；
- `DbscanPoint`和`Dbscan`：ROS无关DBSCAN；
- `LidarClusterer`：按cluster生成质心、AABB和点数。

core仍不依赖：

- `rclcpp`；
- ROS message；
- TF；
- parameter server；
- PCL；
- PX4、Nav2、Gazebo或Qt。

core输入、输出为：

```text
PointCloudFrame -> DetectionResult
```

`DetectionResult`本阶段同时包含：

```text
lidar_clusters       # 仅供ROS2诊断外壳使用
clustering_statistics
tracks               # 本阶段保持空
```

### 3.2 uav_usv_lv_dot_ros2

ROS 2 LifecycleNode继续负责：

- `PointCloud2`转换；
- 根据点云时间戳查询`map <- sensor_frame`；
- 实际`dt`；
- 参数读取；
- 诊断；
- Marker转换；
- 空`TrackedObjectArray`标准出口。

新增相对topic：

| Topic | 类型 | QoS | 用途 |
| --- | --- | --- | --- |
| `diagnostics/lidar_bboxes` | `visualization_msgs/msg/MarkerArray` | best effort, depth 5 | 仅RViz和对比诊断 |

默认namespace后完整topic为：

```text
/perception/lv_dot_ros2/diagnostics/lidar_bboxes
```

每帧MarkerArray先发布`DELETEALL`，再发布每个cluster的`LINE_LIST`真3D框。
Marker的`text`只携带测试元数据：

```text
cluster_id
points
preprocess_ms
cluster_ms
```

Marker不是正式感知接口，不能进入capture manager。

## 4. 算法行为

### 4.1 输入契约保护

节点默认订阅已经验收的：

```text
/perception/usv_01/points_filtered
```

core重复执行有限值、距离、传感器Z范围、自身裁剪和0.04 m体素检查。这些步骤对
标准`points_filtered`输入是幂等保护，也允许未来用同一core安全处理未预处理输入。

### 4.2 LV-DOT内部预处理

预处理顺序与ROS 1行为保持一致：

1. 传感器坐标系局部X/Y裁剪；
2. 按水平距离执行高斯概率采样；
3. 使用该帧精确TF变换到`map`；
4. 在`map`坐标执行地面/顶部Z裁剪；
5. 点数超过阈值时执行渐进体素降采样。

ROS 2版本使用每帧确定性`mt19937`种子，保证同一bag重复运行可复现。ROS 1版本
使用全局`rand()`，而且可视化函数中的`srand(cluster_id)`会改变后续随机状态，
因此两者不追求逐点bit一致，只要求cluster统计和几何结果接近。

### 4.3 DBSCAN语义

上游实现计算：

```text
dx^2 + dy^2 + dz^2 <= epsilon
```

即`epsilon`直接与距离平方比较。Phase 2原样保留该行为，没有把`0.65`解释成
普通欧氏半径。其等效欧氏半径约为：

```text
sqrt(0.65) = 0.806 m
```

`min_points=3`包含查询点自身，与上游一致。

### 4.4 3D bbox

每个cluster计算：

- 算术质心；
- X/Y/Z最小值和最大值；
- 轴对齐尺寸；
- cluster点数；
- 输入点云时间戳。

超过`[30, 15, 12] m`的框按上游逻辑丢弃。

## 5. 参数对应

所有默认值来自`LV_DOT_TUNING_REPORT.md`和当时挂载的
`containers/lv_dot_noetic/detector_param.yaml`，本阶段没有重新调参。

| ROS 2参数 | 值 | 来源/作用 |
| --- | ---: | --- |
| `input_min_range` | 0.5 m | Mid-360调优预处理 |
| `input_max_range` | 20.0 m | 调优场景有效范围 |
| `input_min_z` | -1.75 m | 传感器坐标海浪裁剪 |
| `input_max_z` | 4.0 m | 传感器坐标上限 |
| `input_voxel_size` | 0.04 m | 调优点云体素 |
| `local_range_x` | 10.0 m | 上游固定局部范围 |
| `local_range_y` | 10.0 m | 上游固定局部范围 |
| `ground_height` | 0.22 m | map坐标海面过滤 |
| `roof_height` | 6.0 m | map坐标顶部过滤 |
| `downsample_threshold` | 12000 | 上游自适应降采样阈值 |
| `adaptive_voxel_initial_size` | 0.1 m | 上游初始体素 |
| `gaussian_downsample_sigma` | 16.0 | 调优后的高斯sigma |
| `lidar_dbscan_min_points` | 3 | 调优后的LiDAR DBSCAN参数 |
| `lidar_dbscan_epsilon` | 0.65 | 调优值，按距离平方阈值解释 |
| `maximum_object_size` | `[30, 15, 12] m` | 上游大框过滤 |

参数位于：

```text
src/uav_usv_lv_dot_ros2/config/lv_dot_phase1.yaml
```

文件名保留Phase 1时期名称以免破坏现有launch引用，内容已增加Phase 2参数。

## 6. ROS 1/ROS 2逐帧对比方法

工具：

```text
tools/lv_dot/run_phase2_replay.sh
tools/lv_dot/compare_phase2_clusters.py
tools/lv_dot/sample_phase2_resources.py
```

每份bag以1倍速只回放：

```text
/perception/usv_01/points_filtered
/tf
/tf_static
```

没有重放旧的ROS 2结果。新节点重新计算并记录：

```text
/perception/usv_01/points_filtered
/perception/lv_dot_ros2/diagnostics/lidar_bboxes
/perception/lv_dot_ros2/diagnostics
/perception/lv_dot_ros2/observations
```

### 6.1 ROS 1时间限制

上游`publish3dBox()`没有填写Marker header stamp，因此记录的ROS 1框时间戳为0。
对比工具采用因果关联：每个ROS 1 Marker只关联它发布前最近到达的点云，禁止
匹配未来点云。

### 6.2 ROS 1 Z信息限制

上游Marker将框底部固定画在`map z=0`，并丢失真实bbox的Z中心和最小Z。因此：

- X/Y中心和X/Y尺寸是主要等价指标；
- 报告中的3D display误差只能评价旧Marker显示，不代表core真实Z误差；
- ROS 2 Marker保留真实Z中心和尺寸，不继续复制旧可视化缺陷。

### 6.3 ROS 1点数限制

ROS 1 `lidar_bboxes` Marker不包含cluster点数。工具只能计算一个代理上界：

1. 将同帧输入点云按USV姿态和Mid-360外参变换到map；
2. 执行局部范围和map Z裁剪；
3. 统计旧bbox内的输入点数；
4. 与ROS 2 Marker中的真实cluster点数比较。

该代理发生在ROS 1随机高斯采样之前，因此相对差异偏大，不能被误读为聚类失败。

## 7. 三份bag量化结果

### 7.1 几何和cluster数量

| 场景 | 比较帧 | ROS1平均簇 | ROS2平均簇 | 数量MAD | 数量完全一致率 | XY中心均值/P95 | XY尺寸均值/P95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 匀速 | 1059 | 1.312 | 1.310 | 0.081 | 92.16% | 0.055/0.105 m | 0.080/0.231 m |
| 转弯 | 1025 | 1.505 | 1.483 | 0.126 | 87.80% | 0.087/0.200 m | 0.163/0.673 m |
| 加速 | 1846 | 1.499 | 1.480 | 0.112 | 89.11% | 0.071/0.165 m | 0.171/0.704 m |

极少数错误配对产生了3.26到5.12 m的最大中心误差：这些帧正好存在双方簇数量
不同，贪心最近邻将额外杂簇配到目标簇。P95仍不超过0.20 m，更能表示正常帧行为。

### 7.2 点数代理

| 场景 | 真实ROS2点数与ROS1代理上界的平均绝对差 | 平均相对差 |
| --- | ---: | ---: |
| 匀速 | 17.31点 | 67.52% |
| 转弯 | 12.89点 | 89.60% |
| 加速 | 12.29点 | 77.40% |

相对差偏大的原因是：目标远端簇本身点数较少、ROS 1实际点数未发布、代理没有
执行不可复现的全局`rand()`采样。该项用于记录接口缺口，不作为Phase 2否决指标。

### 7.3 处理时间和资源

| 场景 | ROS2聚类均值/最大 | ROS1输入到Marker均值 | ROS2输入到Marker均值 | CPU均值/最大 | RSS均值/最大 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 匀速 | 1.414/4.000 ms | 52.31 ms | 13.13 ms | 3.68/4.2% | 31.21/31.27 MiB |
| 转弯 | 1.505/4.227 ms | 24.05 ms | 17.31 ms | 4.46/5.1% | 31.00/31.07 MiB |
| 加速 | 1.474/4.695 ms | 50.79 ms | 17.65 ms | 4.82/5.2% | 31.04/31.07 MiB |

ROS 1没有发布纯聚类耗时，表中ROS 1值是同一bag内“输入记录时间到Marker记录
时间”的端到端延迟，包含ROS1定时器、桥和算法；ROS2同时给出core纯聚类耗时和
同口径端到端延迟。两者含义在表中明确区分。

## 8. 稳定性和TF结果

| 场景 | 输入 | 接受 | TF成功/失败 | 畸形点云 | 非单调时间 | 输出频率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 匀速 | 1069 | 1060 | 1060/9 | 0 | 0 | 10 Hz |
| 转弯 | 1026 | 1026 | 1026/0 | 0 | 0 | 10 Hz |
| 加速 | 1847 | 1847 | 1847/0 | 0 | 0 | 10 Hz |

匀速bag的`map` TF约在点云开始0.9 s后才出现，前9帧按设计丢弃。没有使用最新TF、
固定外参或修改点云坐标掩盖失败。三个节点进程均正常退出，日志无ERROR、FATAL、
异常或崩溃。

回放结果保存在仓库外：

```text
/var/tmp/UAV_USV_lv_dot_phase2/
```

大小：匀速11 MiB、转弯11 MiB、加速20 MiB。

## 9. 构建和测试

```bash
source /opt/ros/humble/setup.bash
cd <YOUR_UAV_USV_WORKSPACE>

colcon build --packages-select \
  uav_usv_lv_dot_core uav_usv_lv_dot_ros2 \
  --symlink-install

source install/setup.bash
colcon test --packages-select \
  uav_usv_lv_dot_core uav_usv_lv_dot_ros2
colcon test-result --verbose
```

结果：

```text
uav_usv_lv_dot_core: 6 tests passed
uav_usv_lv_dot_ros2: 4 tests passed
0 failures
```

测试覆盖：

- 未configure时拒绝处理；
- frame和空正式观测契约；
- reset；
- 上游DBSCAN距离平方语义；
- 质心、尺寸、点数和噪声；
- 有限值/Z过滤、TF变换和聚类；
- PointCloud2字段转换和错误布局；
- 完整Track消息转换仍可用；
- 3D Marker、时间戳和点数元数据。

namespace实测：

```text
/phase2_alt/lv_dot_detector_node
/phase2_alt/observations
/phase2_alt/diagnostics
/phase2_alt/diagnostics/lidar_bboxes
```

节点成功执行：

```text
unconfigured -> inactive -> active
```

源码中没有硬编码`usv_01`或绝对输入topic；默认launch仅通过参数和remap形成实例。

## 10. 重复测试命令

示例：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=67

tools/lv_dot/run_phase2_replay.sh \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase2/constant_ros2 \
  1.0

tools/lv_dot/compare_phase2_clusters.py \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase2/constant_ros2 \
  --resource-log \
    /var/tmp/UAV_USV_lv_dot_phase2/constant_ros2_logs/resources.jsonl \
  --json-output \
    /var/tmp/UAV_USV_lv_dot_phase2/constant_comparison.json
```

直接启动节点：

```bash
ros2 launch uav_usv_lv_dot_ros2 lv_dot_ros2.launch.py \
  vehicle_id:=usv_01 \
  points_topic:=/perception/usv_01/points_filtered \
  output_frame:=map \
  use_sim_time:=true
```

RViz添加`MarkerArray`并选择：

```text
/perception/lv_dot_ros2/diagnostics/lidar_bboxes
```

Fixed Frame使用`map`。

## 11. 已知问题

1. 上游DBSCAN是O(N^2)全点扫描。本阶段优先保持算法语义，没有改为空间索引；当前
   约数百点、10 Hz数据下性能充足，未来高密度点云需要独立性能优化和等价性复测。
2. ROS 1全局随机数状态不可从bag恢复，ROS2使用确定性采样，因此不承诺逐点、逐框
   bit一致。
3. ROS 1 Marker没有时间戳、真实Z中心和cluster点数，导致三项对比只能使用因果
   关联、XY主指标和点数代理。
4. Phase 2没有track id连续性。每帧cluster id只表示该帧DBSCAN序号。
5. `/perception/lv_dot_ros2/observations`仍为空，这是安全要求，不是缺陷。
6. 目前没有把Marker加入主线RViz配置，避免诊断迁移影响现有演示布局；topic可独立
   添加显示。

## 12. Phase 2验收判断

验收项：

- ROS2稳定输出LiDAR cluster：通过；
- 三种指定bag无崩溃：通过；
- cluster数量接近ROS1：通过，平均簇数差约0.002到0.022；
- 中心误差可接受：通过，XY均值0.055到0.087 m，P95不超过0.20 m；
- 参数来自当前调优结果：通过；
- CPU和内存稳定：通过；
- source mux保持ground truth：通过；
- 没有迁移Phase 3算法：通过。

结论：Phase 2通过工程验收。

## 13. 下一阶段边界

Phase 3再迁移：

```text
tracking
  -> Kalman
  -> dynamic classification
```

Phase 3开始前继续保留ROS 1 Docker基线和当前三份bag，不删除Phase 2诊断输出，
不把source mux切到sensor。本提交到Phase 2为止，不自动开始Phase 3。
