# LV-DOT ROS 2原生迁移Phase 3报告

日期：2026-07-15。

## 1. 阶段结论

Phase 3完成了ROS 2原生LiDAR跟踪链路：

```text
PointCloud2
  -> 点云预处理
  -> DBSCAN / LidarCluster
  -> 预测位置和尺寸门控
  -> 加权特征关联
  -> 一对一匹配
  -> 真实dt恒加速度Kalman
  -> NEW / CONFIRMED / LOST / REMOVED
  -> /perception/lv_dot_ros2/tracks
```

三份指定bag均以1倍速完成全量回放：

- `constant_final_20260715`；
- `turn_final_20260715`；
- `acceleration_final_20260715`。

结果满足本阶段工程验收：

- ROS 2持续输出非空`TrackedObjectArray`；
- 目标track在三类运动中持续存在；
- 速度由真实消息时间差估计，不使用固定`0.033 s`；
- 三份bag均无节点崩溃；
- ROS 2位置和速度均值误差不差于ROS 1基线；
- 正式`observations`安全出口仍为空；
- `perception_source_mux`、围捕和控制主线未修改。

当前安全边界仍为：

```text
perception_source=ground_truth
```

本阶段没有迁移：

- dynamic classification；
- dynamic voting；
- vision；
- 多源fusion；
- source mux切换；
- 围捕控制接管。

## 2. 修改文件

### 2.1 纯算法core

```text
src/uav_usv_lv_dot_core/CMakeLists.txt
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/types.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/kalman_filter.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/multi_object_tracker.hpp
src/uav_usv_lv_dot_core/include/uav_usv_lv_dot_core/detector_core.hpp
src/uav_usv_lv_dot_core/src/kalman_filter.cpp
src/uav_usv_lv_dot_core/src/multi_object_tracker.cpp
src/uav_usv_lv_dot_core/src/detector_core.cpp
src/uav_usv_lv_dot_core/test/test_detector_core.cpp
```

### 2.2 ROS 2外壳

```text
src/uav_usv_lv_dot_ros2/config/lv_dot_phase1.yaml
src/uav_usv_lv_dot_ros2/include/uav_usv_lv_dot_ros2/detector_node.hpp
src/uav_usv_lv_dot_ros2/src/detector_node.cpp
```

配置文件沿用历史名称`lv_dot_phase1.yaml`，避免破坏Phase 1/2启动接口；内容已累积到
Phase 3。

### 2.3 验证与文档

```text
tools/lv_dot/run_phase3_replay.sh
tools/lv_dot/compare_phase3_tracks.py
docs/LV_DOT_ROS2_PHASE3_REPORT.md
```

## 3. 算法边界

### 3.1 ROS无关边界

`uav_usv_lv_dot_core`仍不依赖：

- `rclcpp`；
- ROS message；
- TF2；
- parameter server；
- PX4、Nav2、Gazebo或Qt。

跟踪器接口为：

```text
vector<LidarCluster> + timestamp + sensor_position
  -> MultiObjectTracker
  -> vector<TrackEstimate> + TrackingStatistics
```

ROS 2节点只负责：

- `PointCloud2`转换；
- 按点云时间戳查询TF；
- 参数读取；
- ROS消息发布；
- Diagnostics。

### 3.2 上游来源

迁移基线：

```text
https://github.com/Zhefan-Xu/LV-DOT
commit: 449bf2c960a26b067b235d82f6e0aac65fc05a6b
```

对应关系：

| ROS 1实现 | ROS 2 core |
| --- | --- |
| `dynamicDetector::boxAssociation*` | `MultiObjectTracker::Impl::update` |
| `dynamicDetector::findBestMatch` | 预测距离/尺寸门控和特征相似度候选 |
| `kalmanFilter.h/.cpp` | `ConstantAccelerationKalman` |
| `kalmanFilterMatrixAcc` | 按真实dt构建每轴三状态转移矩阵 |
| `getKalmanObservationAcc` | 按真实历史时间跨度计算速度和加速度观测 |
| `kalmanFilterAndUpdateHist` | 匹配更新、尺寸历史和生命周期 |

## 4. 关联设计

### 4.1 保持的ROS 1行为

每个cluster使用9维特征：

```text
[relative_x, relative_y, relative_z,
 size_x, size_y, size_z,
 point_center_x, point_center_y, point_center_z]
```

最终调优权重：

```text
[3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0.0, 0.0, 0.0]
```

门控保持为：

```text
predicted XY distance < 2.0 m
max(XY size) difference < 8.0 m
```

候选分数仍为：

```text
cos(previous_feature, current_feature)
+ cos(predicted_feature, current_feature)
```

### 4.2 必要的工程扩展

ROS 1对每个当前框独立选择上一帧框，多个当前框可能同时复制同一条历史；同时，漏检时
会清除历史，也没有真正持久的track ID。Phase 3在不改变门控和相似度公式的前提下增加：

1. 候选按相似度排序；
2. 一条旧track和一个新检测只能匹配一次；
3. 未匹配检测创建新track；
4. 未匹配track进入`LOST`并短时预测；
5. 超过漏检上限后进入`REMOVED`并释放。

这部分是满足稳定ID和生命周期要求的ROS 2工程扩展。在线诊断中的
`tracking_id_switch_count`是“一对一关联所有权冲突”的保守代理；真正的目标ID切换在
bag对比工具中使用真值目标单独统计。

## 5. Kalman设计

### 5.1 状态

上游6状态：

```text
[x, y, vx, vy, ax, ay]
```

实现中拆成两个互相独立、数学等价的三状态滤波器：

```text
X轴: [x, vx, ax]
Y轴: [y, vy, ay]
```

Z轴保持上游行为，直接使用本帧LiDAR bbox中心，不进行Kalman估计。

### 5.2 真实时间转移

每帧使用消息时间戳差`dt`：

```text
A(dt) =
[1, dt, 0.5*dt^2]
[0,  1,       dt]
[0,  0,        1]
```

禁止固定时间步。速度和加速度观测使用最多3帧历史，并以实际纳秒时间差作为分母。

### 5.3 噪声参数

参数完全来自已验收调优结果：

```text
initial P:       0.25
Q position:      0.01
Q velocity:      0.05
Q acceleration:  0.05
R position:      0.04
R velocity:      0.30
R acceleration:  0.60
```

更新过程保持上游的全状态观测和`P=(I-KH)P`行为。

## 6. Track生命周期

状态转换：

```text
new detection
  -> NEW
  -> 3 hits -> CONFIRMED
  -> unmatched -> LOST
  -> reacquired -> NEW/CONFIRMED
  -> missed_count > 5 -> REMOVED
```

`TrackEstimate`内部包含：

- `track_id`和确定性UUID；
- position、velocity和acceleration；
- pose/twist covariance；
- dimensions；
- age和missed count；
- confidence；
- lifecycle。

现有`TrackedObject`消息没有lifecycle字段，因此生命周期通过内部状态和Diagnostics统计；
`REMOVED`只在删除统计中出现，不发布已删除目标。

## 7. ROS 2接口

默认namespace为`/perception/lv_dot_ros2`。

| Topic | 类型 | 本阶段行为 |
| --- | --- | --- |
| `points` | `sensor_msgs/msg/PointCloud2` | 相对输入，SensorDataQoS |
| `tracks` | `uav_usv_interfaces/msg/TrackedObjectArray` | Phase 3非空track |
| `observations` | `uav_usv_interfaces/msg/TrackedObjectArray` | 继续发布空数组，隔离控制主线 |
| `diagnostics/lidar_bboxes` | `visualization_msgs/msg/MarkerArray` | Phase 2聚类诊断 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 输入、TF、聚类和跟踪健康 |

完整新topic：

```text
/perception/lv_dot_ros2/tracks
```

没有将`tracks` remap到：

```text
/fleet/perception/targets
/perception/lv_dot/observations
```

因此不会进入`capture_manager`。

## 8. Diagnostics

新增当前帧和累计统计：

- detection、matched、created、removed；
- active、confirmed、lost；
- 关联成功率；
- 平均匹配距离；
- ID冲突代理计数；
- 平均目标速度；
- 当前和平均Kalman更新时间。

三份实测最后一帧Kalman平均更新时间分别约为：

| 场景 | Kalman平均耗时 |
| --- | ---: |
| constant | 0.008 ms |
| turn | 0.010 ms |
| acceleration | 0.011 ms |

## 9. ROS 1 / ROS 2实测对比

评估规则：

- 预热5秒；
- 以`target_vessel`真值为目标；
- 3.0 m目标门限；
- 若上一track ID仍在门限内，且误差不比最近track大0.5 m以上，则优先保持原ID；
- 同时记录不带该保持规则的严格最近点切换数。

ROS 1的velocity Marker没有持久ID，`marker.id`只是帧内索引，因此ROS 1的ID切换只作为
可视化基线代理，不能解释为真正的目标身份。

### 9.1 目标结果

| 场景 | 实现 | 检测率 | 位置均值 / P95 | 速度均值 / P95 | 持久ID切换 | 严格最近点切换 | 输出频率 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| constant | ROS 1 | 100% | 0.503 / 0.689 m | 0.309 / 1.258 m/s | 21 | 119 | 10.000 Hz |
| constant | ROS 2 | 100% | 0.503 / 0.671 m | 0.201 / 0.502 m/s | 1 | 39 | 10.000 Hz |
| turn | ROS 1 | 100% | 0.787 / 1.356 m | 0.577 / 2.093 m/s | 53 | 171 | 10.000 Hz |
| turn | ROS 2 | 100% | 0.709 / 1.121 m | 0.422 / 0.802 m/s | 22 | 109 | 10.000 Hz |
| acceleration | ROS 1 | 100% | 0.793 / 1.407 m | 0.572 / 1.917 m/s | 117 | 313 | 10.000 Hz |
| acceleration | ROS 2 | 100% | 0.719 / 1.148 m | 0.448 / 1.048 m/s | 33 | 167 | 10.000 Hz |

三种场景最长目标丢失时间均为`0.0 s`。

### 9.2 ROS 1 / ROS 2直接差异

| 场景 | 目标位置差均值 / P95 | 目标速度差均值 / P95 |
| --- | ---: | ---: |
| constant | 0.098 / 0.448 m | 0.249 / 1.275 m/s |
| turn | 0.280 / 0.939 m | 0.533 / 2.144 m/s |
| acceleration | 0.288 / 0.949 m | 0.532 / 2.241 m/s |

ROS 2平均track数量高于ROS 1，因为Phase 3按要求保留短时`LOST`轨迹，而ROS 1在漏检时
立即丢弃历史。这是生命周期语义差异，不是cluster数量回归；Phase 2的cluster对比保持
不变。

## 10. 性能与稳定性

| 场景 | 输入/接受帧 | CPU均值 / 最大 | RSS均值 / 最大 | 输入到track延迟均值 / 最大 | 结果 |
| --- | ---: | ---: | ---: | ---: | --- |
| constant | 1069 / 1060 | 3.30% / 3.80% | 32.03 / 32.09 MB | 12.87 / 23.28 ms | 通过 |
| turn | 1026 / 1026 | 3.93% / 4.40% | 31.79 / 31.86 MB | 17.27 / 25.80 ms | 通过 |
| acceleration | 1847 / 1847 | 4.26% / 4.60% | 31.86 / 31.91 MB | 17.33 / 26.01 ms | 通过 |

`constant`开头有9帧因bag启动时`map` TF尚未出现而按设计丢弃，之后1060帧全部处理；
`turn`和`acceleration`的TF失败数均为0。三次节点均干净退出，无非单调时间戳、无
malformed cloud、无崩溃。

回放输出：

```text
/var/tmp/UAV_USV_lv_dot_phase3/constant_ros2
/var/tmp/UAV_USV_lv_dot_phase3/turn_ros2
/var/tmp/UAV_USV_lv_dot_phase3/acceleration_ros2
```

分析结果：

```text
/var/tmp/UAV_USV_lv_dot_phase3/constant_result.json
/var/tmp/UAV_USV_lv_dot_phase3/turn_result.json
/var/tmp/UAV_USV_lv_dot_phase3/acceleration_result.json
```

## 11. 安全边界证明

三份ROS 2回放中的`observations`统计：

| 场景 | 消息数 | 非空消息数 |
| --- | ---: | ---: |
| constant | 1060 | 0 |
| turn | 1026 | 0 |
| acceleration | 1847 | 0 |

本阶段diff中没有以下文件：

- `capture_manager`；
- `FleetCommand`消息；
- PX4 agent；
- USV/Nav2 agent；
- `perception_source_mux`；
- source mux默认参数。

ROS 1 Docker版未删除，仍作为后续Phase 4对照基线。

## 12. 构建与测试命令

```bash
colcon build --packages-select \
  uav_usv_lv_dot_core uav_usv_lv_dot_ros2 \
  --symlink-install --event-handlers console_direct+

colcon test --packages-select \
  uav_usv_lv_dot_core uav_usv_lv_dot_ros2 \
  --event-handlers console_direct+ \
  --return-code-on-test-failure
```

单元测试覆盖：

- core未配置保护；
- DBSCAN和bbox回归；
- cluster后创建`NEW` track；
- 三次命中转`CONFIRMED`；
- 漏检转`LOST`；
- 超过漏检阈值后`REMOVED`；
- track ID连续；
- 两个检测不能占用同一旧track；
- ROS消息完整字段转换。

全量回放示例：

```bash
tools/lv_dot/run_phase3_replay.sh \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase3/constant_ros2 1.0

python3 tools/lv_dot/compare_phase3_tracks.py \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase3/constant_ros2 \
  --resource-log \
  /var/tmp/UAV_USV_lv_dot_phase3/constant_ros2_logs/resources.jsonl
```

## 13. 已知问题

1. 当前没有dynamic classification，所有LiDAR cluster都会成为候选track；海事目标可能由
   多个局部cluster表示，因此“每帧只选最近中心”的苛刻评估会产生额外ID跳变。
2. ROS 1 Marker没有持久ID，无法进行严格的一一ID等价比较；本报告同时提供真值锚定误差
   和帧内Marker代理切换数。
3. 生命周期字段尚未进入现有`TrackedObject`消息。为了不破坏冻结消息，本阶段只通过core
   和Diagnostics暴露状态。
4. `LOST`保留5帧使ROS 2平均track数量高于ROS 1。这是按Phase 3要求引入的故障宽限。
5. 本阶段不判断动静态，`tracks`不能作为控制输入；`observations`继续保持空。

## 14. 下一阶段

Phase 4只应迁移：

- dynamic classification；
- dynamic voting；
- 连续性判定；
- 动态track诊断。

在Phase 4与ROS 1同bag对比通过前，不应把`tracks`或动态结果接入source mux，也不应改变
`perception_source=ground_truth`。
