# 多源感知层架构

本文定义 UAV_USV 的独立感知层。感知层把 Gazebo 真值、UAV 视觉、USV Mid-360 和
未来 LV-DOT 后端统一为 `TrackedObjectArray`，任务层只看到冻结接口
`/fleet/perception/targets`。

## 边界

```text
Gazebo target pose --------------------> ground_truth_adapter
                                                |
                                                v
                              /perception/ground_truth/tracks

UAV camera backend ---> /perception/uav_01/observations --\
                                                            +-> sensor fusion
USV LiDAR backend ---> /perception/usv_01/observations ----/         |
                                                                      v
                                                   /perception/fused/tracks

sensor observations + optional ground truth ---> hybrid fusion
                                                        |
                                                        v
                                          /perception/hybrid/tracks

 ground_truth / sensor / hybrid ---> perception_source_mux
                                           |
                                           v
                              /fleet/perception/targets
                                           |
                                           v
                                    capture_manager
```

感知层不发布 `FleetCommand`，不调用 PX4 topic，不发送 Nav2 action，也不发布 Gazebo
`cmd_vel`。`capture_manager`、PX4 agent、USV agent 和控制 lease 不属于本包。

## 目录

```text
uav_usv_perception/
  scripts/adapters/ground_truth_adapter.py
  scripts/tracking/track_association.py
  scripts/fusion/perception_fusion_node.py
  scripts/fusion/perception_source_mux.py
  docs/interfaces/TRACKED_OBJECT_CONTRACT.md
  launch/perception_layer.launch.py
  test/test_track_association.py
```

现有 Mid-360 与 UAV camera adapter 保持在 `scripts/`，后续可以机械移动目录，但本阶段
不为目录美观改动已验证的可执行文件路径。

## 节点

### ground_truth_adapter

读取 Gazebo `Pose_V`，估计目标速度和偏航角速度，发布结构化真值观测。它不再直接
占用任务接口。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `pose_topic` | `/world/minimal_dynamic_capture/pose/info` | Gazebo entity pose |
| `entity_name` | 空 | 单目标便捷参数，非空时覆盖 entity_names |
| `entity_names` | `[target_vessel]` | 支持多个真值实体 |
| `output_topic` | `/perception/ground_truth/tracks` | 真值候选出口 |
| `publish_rate_hz` | `10.0` | 发布频率 |
| `timeout_seconds` | `1.0` | entity pose 超时 |
| `velocity_alpha` | `0.35` | 差分速度低通系数 |

真值 `source_mask=SOURCE_UNKNOWN`，避免把仿真真值伪装成相机或 LiDAR。

### perception_fusion_node

一个实例可订阅任意数量 `TrackedObjectArray`。第一版完成：

- 按 `header.frame_id` 和消息时间查询 TF；
- 把 pose、twist 和 covariance 转换到 `map`；
- 按时间窗和 XY 最近邻组成观测组；
- 保留已有稳定 ID，必要时生成 `fused_NNNN`；
- 置信度使用 `1 - product(1-confidence)` 融合；
- 多 topic 命中时增加 `SOURCE_FUSED`；
- 对位置、速度做可配置平滑；
- 超时删除航迹，等待期间对 confidence 衰减。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_topics` | UAV/USV observations | 任意数量输入 topic |
| `input_topics_csv` | 空 | launch/CLI 使用的逗号分隔入口，非空时覆盖数组 |
| `output_topic` | `/perception/fused/tracks` | 融合出口 |
| `target_frame` | `map` | 统一坐标系 |
| `sync_slop_seconds` | `0.35` | 跨来源时间关联窗 |
| `association_distance` | `8.0` | XY 最近邻门限，m |
| `max_input_age_seconds` | `1.0` | 输入消息接收超时 |
| `track_timeout_seconds` | `2.0` | 航迹删除时间 |
| `smoothing_alpha` | `0.75` | 新观测权重 |

launch 启动两个实例：

- `sensor_perception_fusion`：只订阅 UAV/USV 观测；
- `hybrid_perception_fusion`：额外订阅 ground truth。

这避免在 mux 内重复实现关联算法。

### perception_source_mux

mux 是 `/fleet/perception/targets` 的唯一感知发布者。支持动态参数：

```bash
ros2 param set /perception_source_mux perception_source ground_truth
ros2 param set /perception_source_mux perception_source sensor
ros2 param set /perception_source_mux perception_source hybrid
```

无效值会被拒绝。选中来源超时后发布空数组，不偷偷回退到真值，防止演示时把传感器
故障误判为算法仍然有效。状态发布到 `/perception/source_status`，内容包含来源、topic、
在线状态、数据年龄和 track 数量。

## Topic

| Topic | 类型 | 发布者 | 订阅者 |
| --- | --- | --- | --- |
| `/perception/ground_truth/tracks` | `TrackedObjectArray` | ground truth adapter | hybrid fusion、mux |
| `/perception/uav_01/observations` | `TrackedObjectArray` | UAV 后端插件 | sensor/hybrid fusion |
| `/perception/usv_01/observations` | `TrackedObjectArray` | USV 后端插件 | sensor/hybrid fusion |
| `/perception/fused/tracks` | `TrackedObjectArray` | sensor fusion | mux |
| `/perception/hybrid/tracks` | `TrackedObjectArray` | hybrid fusion | mux |
| `/fleet/perception/targets` | `TrackedObjectArray` | source mux | capture manager/Qt |
| `/perception/source_status` | `std_msgs/String` JSON | source mux | Qt/诊断 |

相机、CameraInfo、Mid-360 点云和 SensorStatus 是后端输入，不直接进入 fusion：

```text
/fleet/uplink/uav_xx/camera/image_raw
/fleet/uplink/uav_xx/camera/camera_info
/perception/usv_xx/points_filtered
/fleet/sensor_status
```

未来的 LV-DOT adapter 只需把自身结果转换为
`/perception/usv_xx/observations`，fusion、mux 和 capture manager 均无需修改。

## 启动

独立启动感知层：

```bash
ros2 launch uav_usv_perception perception_layer.launch.py \
  pose_topic:=/world/minimal_dynamic_capture/pose/info \
  target_entity:=target_vessel \
  perception_source:=ground_truth
```

最小围捕已包含该 launch：

```bash
ros2 launch uav_usv_bringup minimal_dynamic_capture.launch.py \
  perception_source:=ground_truth
```

传感器 shadow mode：保持任务使用真值，同时检查候选融合结果。

```bash
ros2 topic echo /perception/fused/tracks
ros2 topic echo /fleet/perception/targets
```

## 最小验收

```bash
ros2 topic echo /perception/ground_truth/tracks --once
ros2 topic echo /perception/fused/tracks --once
ros2 topic echo /fleet/perception/targets --once
ros2 topic echo /perception/source_status --once
```

切换到 `sensor` 前，UAV/USV 后端必须实际发布观测。最小场景只有一个任务目标，后端
应保持 `track_id=target_vessel`，因为现有 capture manager 明确按该 ID 选择目标。
未来多目标系统应新增任务目标选择器，不应让 mux 根据数组顺序重命名目标。

## 已知边界

1. 第一版使用最近邻，不处理长时间遮挡、航迹分裂/合并和 JPDA。
2. `SOURCE_UNKNOWN` 的真值参与 hybrid 时会设置 `SOURCE_FUSED`，但不会冒充物理传感器。
3. 输入 covariance 会旋转并加上来源间位置离散度，但尚未实现完整信息滤波。
4. sensor mode 不自动回退 ground truth，这是故障可见性设计。
5. 任务目标 ID 选择仍属于任务配置，不属于 fusion。
