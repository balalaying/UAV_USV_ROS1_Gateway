# LV-DOT ROS2 Phase 5报告

日期：2026-07-16

状态：**已实测完成，保持Shadow Mode，未接管围捕控制。**

## 1. 阶段目标

Phase 5将ROS2原生LV-DOT的`dynamic_tracks`转换为冻结的标准感知契约，验证其与
现有`perception_fusion`的时间、坐标、ID和置信度兼容性。本阶段没有切换
`perception_source_mux`，没有发布`/fleet/perception/targets`。

## 2. 实际数据流

```text
/perception/lv_dot_ros2/dynamic_tracks
  -> lv_dot_observation_adapter
  -> /perception/lv_dot/observations
                         \
                          +-> perception_fusion
/perception/ground_truth/tracks
                         /
  -> /perception/lv_dot/fused_tracks
  -> lv_dot_fusion_evaluator
  -> /perception/lv_dot/fusion_metrics
  -> Qt Perception Monitor
```

验证launch只启动适配器、现有融合器和评估器，不启动source mux。

## 3. 新增模块

### 3.1 lv_dot_observation_adapter

输入：

```text
/perception/lv_dot_ros2/dynamic_tracks
uav_usv_interfaces/msg/TrackedObjectArray
```

输出：

```text
/perception/lv_dot/observations
uav_usv_interfaces/msg/TrackedObjectArray
```

处理规则：

1. 保留数组header、对象时间戳、位置、速度、尺寸和协方差；
2. 保留稳定`track_id`，缺失ID的对象不进入标准接口；
3. 确保`source_mask`包含`SOURCE_LIDAR`；
4. confidence限制在`[0, 1]`；
5. 缺失UUID时根据track ID确定性生成；
6. 不把“运动”错误解释成“船”，未知classification继续保持UNKNOWN；
7. 不修改输入消息。

旁路状态：

```text
/perception/lv_dot/observation_status
std_msgs/msg/String (JSON)
```

状态中报告输入/输出数量、丢弃数量、frame、timestamp和V1兼容映射。

### 3.2 lv_dot_fusion_evaluator

评估器只读三个航迹topic，按时间门限和空间门限匹配`target_vessel`，输出：

- 当前三源位置、速度、track ID、来源、classification和confidence；
- 位置误差、速度误差、时间戳差；
- 检测率、ID切换次数和ID连续性；
- 明确写出`control_source=ground_truth`和
  `control_output_published=false`。

### 3.3 独立验证launch

```bash
ros2 launch uav_usv_perception lv_dot_fusion_validation.launch.py
```

可配置：

```text
use_sim_time
dynamic_tracks_topic
observation_topic
ground_truth_topic
fused_topic
metrics_topic
target_id
```

## 4. 消息兼容方案

现有`TrackedObject`已经包含：

- `track_id`和UUID；
- `source_mask`；
- `classification`；
- `first_seen`和`last_update`；
- pose/twist及其6x6 covariance；
- dimensions和confidence。

因此本阶段**没有修改任何msg定义**。缺少的内部字段按下表兼容：

| LV-DOT内部字段 | V1正式表达 | 兼容说明 |
| --- | --- | --- |
| dynamic_probability | `confidence` | Phase 4已经输出track confidence与动态概率组合值 |
| motion_state | 不写入V1 | dynamic_tracks输入天然表示`CONFIRMED_DYNAMIC` |
| sensor_source | `source_mask` | 适配后至少含`SOURCE_LIDAR` |
| covariance | pose/twist covariance | 原样保留 |

如以后需要同时发布STATIC、MOVING_CANDIDATE和CONFIRMED_DYNAMIC，应创建版本化
`TrackedObjectV2`或独立状态数组，不重新解释V1字段。

## 5. 融合行为

复用了现有`perception_fusion_node.py`，没有重写算法。验证参数：

```text
target_frame=map
association_distance=12.0 m
sync_slop=0.5 s
preferred_track_ids=[target_vessel]（融合器原有默认值）
```

融合器执行：时间窗过滤、按消息时间戳TF变换、同源互斥最近邻关联、置信度加权、
协方差传播以及稳定ID维护。多源成功关联时`source_mask`增加`SOURCE_FUSED`。

## 6. 测试方法

继续使用既有数据，不创建新Gazebo场景：

```text
bags/lv_dot_acceptance/constant_final_20260715
bags/lv_dot_acceptance/turn_final_20260715
bags/lv_dot_acceptance/acceleration_final_20260715
```

Phase 4的ROS2动态轨迹来自：

```text
/var/tmp/UAV_USV_lv_dot_phase4/{constant,turn,acceleration}_ros2
```

为避免两个bag的record time不同，使用消息header stamp合成验证输入：

```bash
python3 tools/lv_dot/create_phase5_fusion_input.py \
  <ground_truth_bag> <phase4_bag> <phase5_input_bag>
```

然后重新运行Phase 5链路：

```bash
tools/lv_dot/run_phase5_replay.sh \
  <phase5_input_bag> <phase5_output_bag> 3.0
```

最后独立分析新输出：

```bash
python3 tools/lv_dot/analyze_phase5_fusion.py \
  <phase5_output_bag> --output <result.json>
```

输出路径：

```text
/var/tmp/UAV_USV_lv_dot_phase5/constant_output
/var/tmp/UAV_USV_lv_dot_phase5/turn_output
/var/tmp/UAV_USV_lv_dot_phase5/acceleration_output
```

三份回放日志均未出现ERROR、Traceback、进程死亡或节点崩溃。

## 7. 量化结果

下表中的“时间差”是观测与真值消息时间戳差，不冒充端到端处理延迟。

| 场景 | 来源 | 检测率 | 平均位置误差 | P95位置误差 | 平均速度误差 | ID切换 | 平均时间差 | 多源标志率 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 匀速 | LV-DOT | 99.34% | 1.397 m | 1.606 m | 0.212 m/s | 1 | 32.63 ms | 0% |
| 匀速 | Fusion | 100% | 0.674 m | 0.792 m | 0.094 m/s | 0 | 0.47 ms | 98.02% |
| 转弯 | LV-DOT | 99.41% | 1.457 m | 1.839 m | 0.370 m/s | 1 | 48.51 ms | 0% |
| 转弯 | Fusion | 100% | 0.849 m | 1.162 m | 0.213 m/s | 0 | 32.92 ms | 93.65% |
| 加速 | LV-DOT | 98.97% | 1.469 m | 1.865 m | 0.408 m/s | 4 | 29.70 ms | 0% |
| 加速 | Fusion | 100% | 0.710 m | 0.941 m | 0.186 m/s | 0 | 24.04 ms | 96.98% |

融合后的平均位置误差相对LV-DOT单源降低约42%至52%。三类场景中融合目标ID均为
`target_vessel`且没有切换。未出现多源标志的少量样本来自动态轨迹冷启动或短暂失配，
此时融合器只保留真值轨迹，不伪造传感器贡献。

## 8. Qt显示

在现有`Perception Monitor`页增加三行对比表：

```text
Ground Truth
LV-DOT
Fusion
```

每行显示track ID、三维位置、三维速度、source mask解释、confidence、位置误差、速度
误差、时间延迟和在线状态。Qt只订阅轻量JSON
`/perception/lv_dot/fusion_metrics`，不在GUI线程处理点云或高频航迹。

使用`QT_QPA_PLATFORM=offscreen`完成6秒启动测试，窗口正常运行；topic缺失时保持
“等待数据”，不会崩溃。

## 9. 构建和测试结果

```bash
colcon build --symlink-install \
  --packages-select uav_usv_perception uav_usv_mission

colcon test --packages-select uav_usv_perception
colcon test-result --verbose
```

结果：

```text
2 packages built
39 tests, 0 errors, 0 failures, 0 skipped
```

新增适配器测试验证了source mask、confidence边界、时间戳、UUID、covariance保留和
缺失track ID丢弃行为。

## 10. 安全边界证明

本阶段未修改：

```text
capture_manager
FleetCommand
PX4
Nav2
UAV/USV agent控制接口
perception_source_mux
uav_usv_interfaces/msg
```

保持：

```text
perception_source=ground_truth
```

Phase 5输出bag只含ground truth、dynamic tracks、标准observations、fused tracks、
adapter status和fusion metrics；不存在`/fleet/perception/targets`。

## 11. 修改文件

```text
src/uav_usv_perception/scripts/adapters/lv_dot_observation_adapter.py
src/uav_usv_perception/scripts/evaluation/lv_dot_fusion_evaluator.py
src/uav_usv_perception/launch/lv_dot_fusion_validation.launch.py
src/uav_usv_perception/test/test_lv_dot_phase5_adapter.py
src/uav_usv_perception/CMakeLists.txt
src/uav_usv_perception/docs/interfaces/TRACKED_OBJECT_CONTRACT.md
src/uav_usv_mission/scripts/fleet_base_station_gui.py
tools/lv_dot/create_phase5_fusion_input.py
tools/lv_dot/run_phase5_replay.sh
tools/lv_dot/analyze_phase5_fusion.py
docs/LV_DOT_ROS2_PHASE5_REPORT.md
```

## 12. 已知问题和下一阶段

1. 融合结果含ground truth，因此这里只证明接口、关联和融合行为，不能把融合误差当作
   纯传感器性能；
2. 加速场景LV-DOT仍有4次ID切换，问题来自Phase 3关联，不在Phase 5改跟踪器；
3. `dynamic_probability`和`motion_state`仍是算法内部状态，V1接口只传递其兼容表达；
4. 本阶段没有启动source mux的sensor模式，也没有进行任务控制试验。

下一阶段应先在独立验证中移除ground truth融合输入，加入UAV视觉观测或其他真实候选源，
通过稳定性门槛后再人工评审是否允许sensor模式。Phase 5不会自动进行该切换。
