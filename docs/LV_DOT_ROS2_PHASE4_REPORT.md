# LV-DOT ROS 2原生迁移Phase 4报告

日期：2026-07-16。

## 1. 阶段结论

Phase 4完成了ROS 2原生动态目标分类旁路：

```text
PointCloud2
  -> Phase 2点云预处理和DBSCAN
  -> Phase 3一对一关联和Kalman跟踪
  -> 速度门限
  -> 历史位移和方向投票
  -> 连续帧一致性
  -> 3/12动态保持
  -> /perception/lv_dot_ros2/dynamic_tracks
```

指定三份运动bag和一份可复现静态bag均以1倍速完成回放。结果为：

- 匀速、转弯、加速场景动态检测率分别为99.11%、99.18%、97.67%；
- 最长连续丢失分别为0.40 s、0.70 s、1.10 s；
- 静态600帧中动态误检为0；
- 四个场景均持续发布约10 Hz的`dynamic_tracks`数组；
- ROS 2节点没有崩溃，CPU和RSS保持稳定；
- 正式`observations`输出在全部场景中始终为空；
- `perception_source_mux`默认值仍为`ground_truth`；
- 围捕控制、PX4、Nav2和agent没有修改。

本阶段没有接入：

- `/fleet/perception/targets`；
- source mux；
- perception fusion；
- LV-DOT视觉模块；
- capture manager或任何行为控制。

## 2. 修改文件

### 2.1 纯算法core

```text
src/uav_usv_lv_dot_core/CMakeLists.txt
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/types.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/dynamic_classifier.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/detector_core.hpp
src/uav_usv_lv_dot_core/src/dynamic_classifier.cpp
src/uav_usv_lv_dot_core/src/detector_core.cpp
src/uav_usv_lv_dot_core/test/test_detector_core.cpp
```

### 2.2 ROS 2外壳

```text
src/uav_usv_lv_dot_ros2/config/lv_dot_phase1.yaml
src/uav_usv_lv_dot_ros2/include/uav_usv_lv_dot_ros2/detector_node.hpp
src/uav_usv_lv_dot_ros2/src/detector_node.cpp
src/uav_usv_lv_dot_ros2/test/test_phase1_conversions.cpp
```

配置文件保留历史名称`lv_dot_phase1.yaml`，避免破坏Phase 1至Phase 3的启动接口；
内容已经累积到Phase 4。

### 2.3 验证工具

```text
tools/lv_dot/create_static_environment_bag.py
tools/lv_dot/run_phase4_replay.sh
tools/lv_dot/compare_phase4_dynamic.py
```

大体积回放结果保存在`/var/tmp/UAV_USV_lv_dot_phase4`，没有提交进Git。

## 3. 算法边界

### 3.1 Core接口

`uav_usv_lv_dot_core`仍然不依赖：

- `rclcpp`；
- ROS message；
- TF2；
- parameter server；
- PX4、Nav2、Gazebo或Qt。

动态分类接口为：

```text
vector<TrackEstimate> + timestamp
  -> DynamicClassifier
  -> vector<DynamicTrackEstimate>
  -> vector<TrackEstimate> dynamic_tracks
  -> DynamicClassificationStatistics
```

`DynamicTrackEstimate`包含：

- `track_id`；
- 完整`TrackEstimate`；
- `dynamic_probability`；
- `is_dynamic`；
- `motion_state`；
- `motion_history`；
- 动态置信度。

状态为：

```text
STATIC
  -> MOVING_CANDIDATE
  -> CONFIRMED_DYNAMIC
```

`LOST`轨迹可以由Phase 3继续短时预测，但不会发布到`dynamic_tracks`。

### 3.2 上游对应关系

迁移基线：

```text
https://github.com/Zhefan-Xu/LV-DOT
commit: 449bf2c960a26b067b235d82f6e0aac65fc05a6b
```

对应关系：

| ROS 1 LV-DOT行为 | ROS 2 Phase 4实现 |
| --- | --- |
| `classificationCB`速度判断 | KF水平速度与净位移速度门限 |
| 当前点和历史点速度投票 | 历史轨迹分段位移和净运动方向投票 |
| `dynamicVotingThreshold` | `dynamic_voting_threshold` |
| `dynamicConsistencyThreshold` | 连续candidate/dynamic计数 |
| `forceDynaFrames/forceDynaCheckRange` | 最近12帧中3帧dynamic后保持 |
| ROS 1动态bbox | ROS 2 `TrackedObjectArray dynamic_tracks` |

## 4. 点级投票到轨迹级投票的适配

ROS 1的`dynamicDetector`在分类回调中仍能访问每个聚类的点，因此能够比较当前点和历史点，
再按bbox速度方向进行点级投票。Phase 4冻结的输入是`TrackEstimate`，不包含聚类点，不能在
不破坏接口的前提下逐点复现。

ROS 2保留相同判定要素，并做了以下接口适配：

1. 使用最近12帧首尾位置计算净位移速度；
2. 按`frame_skip=2`比较分段位移；
3. 分段速度高于门限且与净位移同向时计一票；
4. 投票率、KF速度和净位移速度同时通过后进入candidate；
5. 连续2帧candidate后确认dynamic；
6. 最近12帧已有3帧dynamic时沿用ROS 1的强制动态保持。

采用完整窗口的原因来自实际静态回放：固定点云经过上游概率降采样后，bbox中心仍存在厘米级
抖动，单帧速度偶尔超过0.05 m/s。第一次实现只看短时track位移，导致493/600帧静态误判。
加入历史净位移和方向连续性后，复验结果为0/600。该变化没有修改调优阈值，但动态目标初次
确认会增加约1.0至1.8秒延迟。

## 5. 参数

参数全部来自`LV_DOT_TUNING_REPORT.md`中的最终结果，没有为Phase 4重新调值：

| 参数 | 值 | 作用 |
| --- | ---: | --- |
| `frame_skip` | 2 | 历史分段投票间隔 |
| `dynamic_velocity_threshold` | 0.05 m/s | KF速度和净位移速度门限 |
| `dynamic_voting_threshold` | 0.15 | 最低运动投票率 |
| `frames_force_dynamic` | 3 | 强制保持所需动态帧数 |
| `frames_force_dynamic_check_range` | 12 | 强制保持和稳定运动窗口 |
| `dynamic_consistency_threshold` | 2 | candidate连续确认帧数 |
| `dynamic_history_size` | 100 | 每条track保留的运动历史上限 |

配置合法性由core检查：历史长度必须覆盖跳帧、一致性和12帧保持窗口。

## 6. ROS 2接口

默认namespace为`/perception/lv_dot_ros2`。

| Topic | 类型 | Phase 4行为 |
| --- | --- | --- |
| `points` | `sensor_msgs/msg/PointCloud2` | 相对输入，SensorDataQoS |
| `tracks` | `TrackedObjectArray` | Phase 3全部track，保持不变 |
| `dynamic_tracks` | `TrackedObjectArray` | 只包含`CONFIRMED_DYNAMIC` track |
| `observations` | `TrackedObjectArray` | 继续发布空数组，隔离控制主线 |
| `diagnostics/lidar_bboxes` | `MarkerArray` | Phase 2聚类诊断，保持不变 |
| `diagnostics` | `DiagnosticArray` | 输入、TF、聚类、跟踪和动态分类统计 |

完整新topic为：

```text
/perception/lv_dot_ros2/dynamic_tracks
```

它没有remap到：

```text
/perception/lv_dot/observations
/fleet/perception/targets
```

## 7. Dynamic Diagnostics

新增字段：

- `dynamic_total_tracks`；
- `dynamic_candidates`；
- `dynamic_confirmed`；
- `dynamic_static_count`；
- `dynamic_unclassified_count`；
- `dynamic_ratio`；
- `dynamic_average_velocity_mps`；
- `dynamic_classification_latency_ms`；
- `dynamic_average_classification_latency_ms`；
- `dynamic_confirmed_count_total`。

这些字段只用于健康诊断，不作为正式目标接口。

## 8. 测试数据和命令

### 8.1 指定运动bag

```text
bags/lv_dot_acceptance/constant_final_20260715
bags/lv_dot_acceptance/turn_final_20260715
bags/lv_dot_acceptance/acceleration_final_20260715
```

### 8.2 静态bag

静态场景由`constant_final_20260715`中已验收的真实Mid-360过滤点云生成：

- 选择开始后15秒的一帧真实点云；
- 固定USV/UAV pose和TF；
- 将目标真值速度置零；
- 以10 Hz重复60秒；
- 不包含旧LV-DOT输出。

复现命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/lv_dot/create_static_environment_bag.py \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase4/static_environment
```

单场景Phase 4回放：

```bash
tools/lv_dot/run_phase4_replay.sh \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase4/constant_ros2 1.0
```

对比：

```bash
python3 tools/lv_dot/compare_phase4_dynamic.py \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase4/constant_ros2 \
  --resource-log \
    /var/tmp/UAV_USV_lv_dot_phase4/constant_ros2_logs/resources.jsonl
```

构建和单元测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  uav_usv_lv_dot_core uav_usv_lv_dot_ros2 --symlink-install
source install/setup.bash
colcon test --packages-select \
  uav_usv_lv_dot_core uav_usv_lv_dot_ros2
colcon test-result --verbose
```

结果：36 tests，0 errors，0 failures，0 skipped。

## 9. ROS 1 / ROS 2动态结果对比

评估规则：

- 3.0 m真值关联门限；
- 指标预热5秒；
- 目标ID选择使用0.5 m迟滞，另行记录严格最近点切换；
- 误检是门限外的动态object；
- 确认延迟从bag首个真值帧开始计算。

| 场景 | 实现 | 检测率 | 确认延迟 | 最长丢失 | 误检object率 | 持续ID切换 | 严格最近点切换 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 匀速 | ROS 1 | 100.00% | 0.00 s | 0.00 s | 3.75% | 0 | 0 |
| 匀速 | ROS 2 | 99.11% | 1.20 s | 0.40 s | 0.00% | 1 | 1 |
| 转弯 | ROS 1 | 100.00% | 0.00 s | 0.00 s | 4.07% | 5 | 9 |
| 转弯 | ROS 2 | 99.18% | 1.00 s | 0.70 s | 0.20% | 1 | 17 |
| 加速 | ROS 1 | 100.00% | 0.00 s | 0.00 s | 2.40% | 4 | 12 |
| 加速 | ROS 2 | 97.67% | 1.80 s | 1.10 s | 0.54% | 4 | 32 |
| 静态 | ROS 2 | 0个动态目标 | 不适用 | 不适用 | 0.00% | 0 | 0 |

ROS 1运动bag在录制开始时已经完成track初始化，因此其确认延迟显示为0，不代表ROS 1从冷
启动到确认不需要时间。当前Docker daemon中没有保留ROS 1镜像，静态bag的ROS 1重算结果
未获得；静态误检结论只标记为ROS 2已实测，不伪造ROS 1结果。三种运动场景仍使用原验收
bag中真实记录的ROS 1结果作为基线。

## 10. 性能

| 场景 | 输入帧 | 输出频率 | 分类平均耗时 | 整体平均处理 | CPU均值 | RSS均值 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 匀速 | 1069 | 10.00 Hz | 0.023 ms | 13.264 ms | 4.50% | 31.78 MiB |
| 转弯 | 1026 | 10.00 Hz | 0.025 ms | 17.206 ms | 4.55% | 31.73 MiB |
| 加速 | 1847 | 10.00 Hz | 0.026 ms | 17.508 ms | 4.79% | 31.77 MiB |
| 静态 | 600 | 10.00 Hz | 0.022 ms | 1.869 ms | 3.36% | 31.20 MiB |

匀速bag最开始先到PointCloud2、后到`map` TF，节点按设计丢弃9帧，最终TF成功率
99.158%。其他三个场景TF成功率为100%。没有使用最新TF掩盖该启动顺序。

## 11. 安全边界验证

代码检查结果：

```text
capture_manager                     未修改
FleetCommand                        未修改
PX4 agent                           未修改
USV agent / Nav2                    未修改
perception_source_mux               未修改
perception_source默认值             ground_truth
/fleet/perception/targets           无新增发布者
```

bag检查结果：

```text
constant     observations非空消息: 0 / 1060
turn         observations非空消息: 0 / 1026
acceleration observations非空消息: 0 / 1847
static       observations非空消息: 0 / 600
```

因此Phase 4结果仍然只是Shadow候选输出，不会接管围捕。

## 12. 已知问题

1. `TrackedObject`当前没有`dynamic_probability`和`motion_state`字段；两者保留在core内部，
   ROS正式旁路消息只携带动态置信度和track数据。
2. ROS 2接口无法逐点复现ROS 1投票，使用轨迹历史适配后换取更低静态误检，但增加约1至
   1.8秒冷启动确认延迟。
3. 加速场景严格最近点选择有32次变化，说明目标附近多cluster仍会互相竞争；使用持续ID迟滞
   后实际任务级切换为4次。该问题属于Phase 3关联质量，不在Phase 4改变跟踪算法。
4. 当前只验证单个USV实例；没有接入source mux，也没有进行全舰队资源测试。
5. ROS 1静态重算镜像当前不在本机Docker数据目录，未将该项标记为已测试通过。

## 13. 下一阶段

Phase 5应继续保持Shadow安全边界，先验证：

1. `dynamic_tracks`到感知融合适配层的结构兼容性；
2. 多源时间戳、TF和track ID策略；
3. ROS 2与ROS 1同bag的正式输出等价性；
4. 在人工切换前验证sensor候选源，不自动修改source mux默认值。

本阶段到此停止，不提前进入Phase 5。
