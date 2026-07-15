# LV-DOT ROS 2 原生迁移 Phase 1 报告

日期：2026-07-15。

本阶段建立 ROS 2 Humble 原生 LV-DOT 工程框架，只实现：

- 纯 C++ core 接口；
- ROS 2 LifecycleNode；
- `PointCloud2` 输入转换；
- 按消息时间戳查询 TF；
- 实际 `dt` 计算；
- 空 `TrackedObjectArray` 标准出口；
- 运行诊断和稳定性测试。

本阶段**没有迁移 DBSCAN、聚类、跟踪、Kalman 或动态判断算法**，没有修改
LV-DOT 参数，没有启动 Phase 2。

安全边界保持不变：

```text
perception_source=ground_truth
```

以下控制主线均未修改：

- `capture_manager`；
- `FleetCommand`；
- PX4；
- Nav2；
- UAV/USV agent；
- source mux 默认逻辑；
- ROS 1 LV-DOT Docker Shadow Mode。

## 1. 软件包结构

### 1.1 uav_usv_lv_dot_core

```text
src/uav_usv_lv_dot_core/
├── CMakeLists.txt
├── package.xml
├── include/uav_usv_lv_dot_core/
│   ├── detector_core.hpp
│   └── types.hpp
├── src/
│   └── detector_core.cpp
└── test/
    └── test_detector_core.cpp
```

该包是纯算法库边界：

- 不包含 `rclcpp`；
- 不包含 ROS message；
- 不包含 TF；
- 不读取 parameter server；
- 不依赖 PX4、Nav2、Gazebo 或 Qt。

当前定义：

- `PointXYZI`：ROS消息转换后的基础点；
- `RigidTransform`：不依赖TF消息的平移与四元数；
- `FrameContext`：时间戳、实际`dt`、输入/输出frame和该帧变换；
- `PointCloudFrame`：core输入；
- `TrackEstimate`：未来完整轨迹输出契约；
- `DetectionResult`：core输出；
- `DetectorCore`：Phase 1空处理接口。

`DetectorCore::process()` 当前只验证生命周期、保存时间/frame契约并返回空track，
没有复制上游算法代码。这样可以在Phase 2逐步迁移算法，而不改变ROS2外壳。

### 1.2 uav_usv_lv_dot_ros2

```text
src/uav_usv_lv_dot_ros2/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── lv_dot_phase1.yaml
├── include/uav_usv_lv_dot_ros2/
│   ├── detector_node.hpp
│   ├── message_conversion.hpp
│   └── pointcloud_conversion.hpp
├── launch/
│   └── lv_dot_ros2.launch.py
├── src/
│   ├── detector_node.cpp
│   ├── main.cpp
│   ├── message_conversion.cpp
│   └── pointcloud_conversion.cpp
└── test/
    └── test_phase1_conversions.cpp
```

该包负责：

- LifecycleNode状态转换；
- SensorDataQoS点云订阅；
- `PointCloud2`到core类型的转换；
- 精确时间戳TF查询；
- 计算逐帧`dt`；
- core结果到`TrackedObjectArray`的完整字段转换；
- 空观测发布；
- DiagnosticArray发布。

## 2. 节点架构

节点名：

```text
lv_dot_detector_node
```

类型：

```text
rclcpp_lifecycle::LifecycleNode
```

数据流：

```text
remapped PointCloud2
        |
        v
relative topic: points                 SensorDataQoS
        |
        +--> 检查frame_id和时间单调性
        |
        +--> lookupTransform(
        |      output_frame,
        |      cloud.header.frame_id,
        |      cloud.header.stamp)
        |
        +--> PointCloud2 -> PointCloudFrame
        |
        +--> DetectorCore::process()           Phase 1返回空tracks
        |
        +--> observations                      TrackedObjectArray
        |
        `--> diagnostics                       DiagnosticArray
```

默认完整节点名：

```text
/perception/lv_dot_ros2/lv_dot_detector_node
```

launch自动执行：

```text
unconfigured
  -> configure
inactive
  -> activate
active
```

也可以使用：

```bash
ros2 launch uav_usv_lv_dot_ros2 lv_dot_ros2.launch.py autostart:=false
ros2 lifecycle set /perception/lv_dot_ros2/lv_dot_detector_node configure
ros2 lifecycle set /perception/lv_dot_ros2/lv_dot_detector_node activate
```

节点使用`SingleThreadedExecutor`。这是有意设计：上游算法迁移前先保持串行处理，
避免未来把上游共享历史状态放进多线程回调时产生数据竞争。

## 3. Topic设计

源码只使用相对topic：

| 相对topic | 类型 | 方向 | QoS |
| --- | --- | --- | --- |
| `points` | `sensor_msgs/msg/PointCloud2` | 输入 | SensorDataQoS |
| `observations` | `uav_usv_interfaces/msg/TrackedObjectArray` | 输出 | reliable, depth 10 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 输出 | reliable, depth 10 |

默认launch映射后：

| 完整topic | 说明 |
| --- | --- |
| `/perception/usv_01/points_filtered` | 已验收Mid-360预处理点云 |
| `/perception/lv_dot_ros2/observations` | ROS2原生候选观测，不接管控制 |
| `/perception/lv_dot_ros2/diagnostics` | Phase 1运行诊断 |

输出观测目前为：

```yaml
header:
  stamp: <PointCloud2.header.stamp>
  frame_id: map
objects: []
```

虽然Phase 1没有track，message conversion已经覆盖未来对象的：

- `uuid`和`track_id`；
- first/last时间；
- source和class；
- position和velocity；
- pose/twist covariance；
- dimensions；
- confidence；
- MMSI。

没有通过Marker或数组下标构造正式轨迹。

## 4. TF设计

每帧使用：

```text
target: output_frame，默认map
source: PointCloud2.header.frame_id
time:   PointCloud2.header.stamp
```

当前数据实际查询：

```text
map <- usv_01/mid360_link @ cloud stamp
```

规则：

1. 不使用固定外参；
2. 不调用time zero查询“最新TF”；
3. TF失败时丢弃该帧；
4. TF失败计入`tf_failure_count`；
5. 成功变换被转换成core的`RigidTransform`并放入该帧`FrameContext`；
6. Phase 1不执行算法坐标变换，防止提前引入Phase 2语义。

constant bag回放开始时，TF树还没有收到`map`变换，前9帧按设计被丢弃。TF树建立后
没有继续增加失败。完整1倍速回放最终结果：

```text
input_count:       1069
accepted_count:    1060
tf_failure_count:  9
tf_success_rate:   99.1581%
```

这9帧没有通过“使用最新TF”或修改点云坐标进行掩盖。

## 5. 时间设计

不使用上游固定的`0.033 s`。

每个通过TF检查的点云计算：

```text
dt = current_cloud_stamp - previous_accepted_cloud_stamp
```

行为：

- 第一帧`dt=0`；
- 后续帧使用实际消息时间差；
- 时间戳相同或倒退时丢弃；
- 记录`nonmonotonic_stamp_count`；
- 诊断发布`last_dt_seconds`和`average_dt_seconds`；
- 输出观测继承输入点云时间戳；
- `steady_clock`只用于测量处理耗时；
- 支持`use_sim_time`。

1倍速完整回放：

```text
average_dt_seconds:       0.100000
input_rate_hz:            10.000
nonmonotonic_stamp_count: 0
```

rosbag的`/clock`可能在传感器消息送达后才前进几毫秒。输入延迟因此被限制为不小于0，
避免显示没有物理意义的负延迟；原始消息时间戳没有被修改。

## 6. 诊断设计

`/perception/lv_dot_ros2/diagnostics`每秒输出：

- lifecycle节点状态；
- vehicle ID；
- output frame；
- 输入与有效帧计数；
- 点云时间戳频率；
- TF成功数、成功率和失败数；
- 畸形点云数；
- 非单调时间戳数；
- last/average dt；
- last/average处理耗时；
- last/average输入延迟；
- 最近点数。

状态规则：

- active但尚未收到点云：WARN；
- TF累计成功率低于99%：WARN；
- 有数据且TF累计成功率至少99%：OK。

## 7. Launch参数

启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_usv_lv_dot_ros2 lv_dot_ros2.launch.py \
  vehicle_id:=usv_01 \
  points_topic:=/perception/usv_01/points_filtered \
  output_frame:=map \
  use_sim_time:=true
```

参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `vehicle_id` | `usv_01` | 诊断硬件标识，不参与topic硬编码 |
| `points_topic` | 根据vehicle_id生成 | remap到相对`points` |
| `output_frame` | `map` | 精确时间戳TF目标frame |
| `use_sim_time` | `true` | 使用bag/Gazebo仿真时间 |
| `node_namespace` | `/perception/lv_dot_ros2` | 控制所有相对输出topic |
| `autostart` | `true` | 自动configure和activate |

C++源码中没有出现`usv_01`和绝对输入topic。

namespace切换实测：

```bash
ros2 launch uav_usv_lv_dot_ros2 lv_dot_ros2.launch.py \
  autostart:=false \
  use_sim_time:=false \
  node_namespace:=/phase1_alt \
  vehicle_id:=test_vehicle \
  points_topic:=/phase1/input
```

实际生成：

```text
/phase1_alt/lv_dot_detector_node
/phase1_alt/observations
/phase1_alt/diagnostics
```

手动状态转换`unconfigured -> inactive -> active`全部成功。

## 8. 构建与单元测试

构建命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select uav_usv_lv_dot_core uav_usv_lv_dot_ros2
```

结果：两个包构建成功，无编译警告。

测试命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test \
  --packages-select uav_usv_lv_dot_core uav_usv_lv_dot_ros2
colcon test-result --verbose
```

结果：

```text
uav_usv_lv_dot_core: 3 tests, 0 errors, 0 failures
uav_usv_lv_dot_ros2: 3 tests, 0 errors, 0 failures
```

覆盖：

- core未配置时拒绝处理；
- Phase 1 core保持frame契约并返回空track；
- core reset；
- PointCloud2 x/y/z/intensity转换；
- 缺少必需字段的畸形PointCloud2拒绝；
- 完整TrackEstimate到TrackedObject字段转换。

## 9. constant bag在线验证

使用已有验收数据：

```text
bags/lv_dot_acceptance/constant_final_20260715
duration: 106.869 s
```

只回放原始输入，不回放旧LV-DOT算法输出：

```bash
ros2 bag play bags/lv_dot_acceptance/constant_final_20260715 \
  --clock \
  --topics \
    /perception/usv_01/points_filtered \
    /tf \
    /tf_static
```

实测：

| 指标 | 结果 |
| --- | ---: |
| 输入帧 | 1069 |
| 有效帧 | 1060 |
| 启动预热TF失败 | 9 |
| 最终TF成功率 | 99.1581% |
| 输出观测频率 | 10.000 Hz |
| 平均dt | 0.100000 s |
| 非单调时间戳 | 0 |
| 畸形点云 | 0 |
| 平均处理耗时 | 11.107 ms |
| 平均输入延迟 | 2.809 ms |
| 节点退出 | 无 |

1倍速资源采样113次：

| 指标 | 结果 |
| --- | ---: |
| CPU均值 | 1.146% |
| CPU峰值 | 1.700% |
| RSS均值 | 29.998 MiB |
| RSS最小 | 29.066 MiB |
| RSS最大 | 30.137 MiB |
| RSS末值 | 30.137 MiB |

## 10. 30分钟稳定性测试

为了避免`--loop`令同一bag时间戳倒退，使用同一constant bag以0.05倍速连续回放，
而不是循环旧时间轴：

```bash
ros2 bag play bags/lv_dot_acceptance/constant_final_20260715 \
  --rate 0.05 \
  --clock \
  --topics \
    /perception/usv_01/points_filtered \
    /tf \
    /tf_static
```

说明：0.05倍只改变消息的墙钟送达速度，点云header时间戳仍保持约0.1秒间隔，所以
节点计算的传感器频率和`dt`仍是10 Hz与0.1秒。

实测节点存活：

```text
31 min 10 s
```

资源采样器实际持续超过1800秒，获得359个5秒周期样本；首个样本到最后一个样本的
时间跨度为1797秒，退出前节点进程总存活1870秒。

最终诊断：

| 指标 | 结果 |
| --- | ---: |
| 输入帧 | 908 |
| 有效帧 | 899 |
| TF失败 | 9，全部位于启动预热 |
| TF成功率 | 99.0088% |
| 平均dt | 0.100002 s |
| 时间戳频率 | 10.000 Hz |
| 非单调时间戳 | 0 |
| 畸形点云 | 0 |
| 崩溃/重启 | 0 |

长期资源：

| 指标 | 结果 |
| --- | ---: |
| CPU均值 | 0.687% |
| CPU峰值 | 0.700% |
| RSS均值 | 30.269 MiB |
| RSS最小 | 29.066 MiB |
| RSS最大 | 30.457 MiB |
| RSS首值 | 29.066 MiB |
| RSS末值 | 30.344 MiB |
| 首尾变化 | +1.277 MiB |

RSS增长发生在启动和首次处理不同大小点云的初始化阶段，约8分钟后稳定在
30.344 MiB并保持到测试结束，没有持续线性增长。

慢速回放使TF消息和点云的墙钟间隔被放大，平均处理耗时为164.590 ms，主要是
`lookupTransform`等待对应时刻TF。该值不代表10 Hz实际负载；1倍速结果11.107 ms
才是本阶段实时性能参考。

## 11. 与ROS1版本的差异

| 项目 | ROS1 Shadow版本 | ROS2 Phase 1 |
| --- | --- | --- |
| 运行环境 | Noetic Docker + TCP桥 | Humble原生进程 |
| 节点结构 | 普通roscpp节点 | LifecycleNode |
| core边界 | 通信和算法混在dynamicDetector | 独立无ROS C++库 |
| 输入topic | 参数中包含绝对默认topic | 源码相对`points`，launch remap |
| TF | Pose/Odom乘手写外参 | 按点云时间查询tf2 |
| TF失败 | 没有统一计数 | 丢帧并诊断 |
| 时间步长 | 固定0.033秒 | 相邻有效消息实际时间差 |
| 正式输出 | Marker/service再由adapter转换 | 原生TrackedObjectArray接口 |
| 生命周期 | 无 | configure/activate/deactivate/cleanup |
| 多实例准备 | 存在绝对namespace和topic | namespace、vehicle和remap分离 |
| 当前算法 | 完整LV-DOT | 空core，尚未进入Phase 2 |

ROS2 Phase 1不是ROS1算法替代品，二者当前不能比较检测率。ROS1 Docker仍是算法
基准和回滚路径。

## 12. 已知问题

1. bag启动时TF树尚未建立，前9个点云被丢弃；这是显式安全行为。未来可在launch
   侧先播放TF或增加激活前TF就绪检查，但不能改用最新TF掩盖。
2. Phase 1只把查询到的变换放进core frame context，尚未执行算法坐标转换；
   Phase 2应明确只变换一次。
3. 当前空观测会按每个有效点云发布。它是候选topic，不连接source mux控制源。
4. 处理时延诊断包含TF等待；Phase 2应增加“TF等待、转换、聚类”分阶段耗时。
5. 尚未加入ROS2参数动态更新和Lifecycle manager，这些不是Phase 1验收范围。

## 13. Phase 2建议

下一阶段只迁移LiDAR聚类和诊断输出：

1. 将上游`dbscan`迁入`uav_usv_lv_dot_core`；
2. 将`lidarDetector`去除ROS include后迁入core；
3. core接收`PointCloudFrame`，输出cluster和bbox；
4. 增加纯算法单元测试；
5. 发布仅用于诊断的LiDAR bbox和debug cloud；
6. 使用同一三份bag与ROS1逐帧比较聚类数量、中心和尺寸；
7. 继续保持`perception_source=ground_truth`；
8. 不迁移跟踪、Kalman和动态判断，直到Phase 2单独验收。

本报告完成后停止，不自动进入Phase 2。
