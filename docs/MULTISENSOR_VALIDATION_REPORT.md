# UAV视觉Observation与多源融合验证报告

日期：2026-07-16

## 1. 阶段结论

本阶段已经完成UAV视觉Observation接口、传感器无关Observation契约、
Ground Truth + LV-DOT + UAV Camera三源融合、Qt Sensor Layer和离线重复验证。

系统仍处于Shadow Mode：

- `perception_source`保持`ground_truth`；
- 本阶段launch不启动`perception_source_mux`；
- `/fleet/perception/targets`和围捕控制链没有被本阶段输出接管；
- `capture_manager`、FleetCommand、PX4、Nav2和UAV/USV agent没有修改。

UAV视觉第一阶段使用Ground Truth代理产生Observation。它验证了图像触发、
CameraInfo、图像时间戳TF、视锥判断、统一消息和融合链路，但不代表已经实现真实
图像目标检测。

## 2. 数据链路

```text
UAV image_raw + CameraInfo + image-stamped TF
                         + Ground Truth proxy
                                  |
                                  v
                   uav_visual_observation_node
                                  |
                   /perception/uav_01/observations
                                  |
                                  +-------------------+
                                                      |
Mid-360 -> LV-DOT ROS2 -> dynamic_tracks              |
                                  |                   |
                                  v                   |
                     lv_dot_observation_adapter       |
                                  |                   |
                    /perception/lv_dot/observations   |
                                  |                   |
                                  +---------+---------+
                                            |
Ground Truth -------------------------------+
                                            v
                                  perception_fusion
                                            |
                           /perception/fused/tracks
                                            |
                     multisensor_validation_evaluator
                                            |
                    /perception/multisensor/metrics
                                            |
                                      Qt Sensor Layer

Control path (unchanged):
ground_truth -> source_mux -> /fleet/perception/targets -> capture_manager
```

融合节点使用短时观测历史，以最慢来源的最新时间戳作为水位线，从每个来源选择
时间最接近的一帧。只有三个来源在时间和空间上均可关联时，Shadow验证launch才
更新融合轨迹。该同步策略仅在`observation_history_seconds > 0`时启用；默认
值为`0`，因此原有融合行为保持不变。

## 3. 新增模块

| 模块 | 设计职责 |
| --- | --- |
| `uav_visual_observation_node.py` | 由图像触发，检查CameraInfo、精确时间TF、距离和视锥，输出CAMERA来源Observation |
| `multisensor_validation_evaluator.py` | 在线比较GT、LV-DOT、UAV Camera和Fusion，发布轻量JSON指标 |
| `multisensor_fusion_validation.launch.py` | 只启动Observation adapter、Shadow fusion和评估器，不启动source mux |
| `create_multisensor_validation_input.py` | 按消息时间戳合并原始传感器bag与LV-DOT结果，并保留`/tf_static` QoS |
| `run_multisensor_validation_replay.sh` | 启动Shadow链、回放输入并录制所有中间输出 |
| `analyze_multisensor_validation.py` | 离线计算检测率、误差、ID连续性、时间差和来源完整率 |

## 4. 标准Topic

| Topic | 类型 | Frame | 用途 |
| --- | --- | --- | --- |
| `/fleet/uplink/uav_01/camera/image_raw` | `sensor_msgs/Image` | UAV camera frame | 视觉Observation触发源 |
| `/fleet/uplink/uav_01/camera/camera_info` | `sensor_msgs/CameraInfo` | UAV camera frame | 相机内参 |
| `/perception/ground_truth/tracks` | `TrackedObjectArray` | `map` | Shadow参考真值 |
| `/perception/lv_dot/dynamic_tracks` | `TrackedObjectArray` | `map` | LV-DOT ROS2动态轨迹 |
| `/perception/lv_dot/observations` | `TrackedObjectArray` | `map` | 标准LiDAR Observation |
| `/perception/uav_01/observations` | `TrackedObjectArray` | `map` | 标准Camera Observation |
| `/perception/fused/tracks` | `TrackedObjectArray` | `map` | 多源融合候选轨迹 |
| `/perception/multisensor/metrics` | `std_msgs/String` JSON | N/A | Qt轻量监控指标 |

## 5. Observation统一语义

LV-DOT和UAV Camera都使用`TrackedObjectArray`，不向正式Observation添加像素框、
点云数量等传感器私有字段。

| 字段 | 统一语义 |
| --- | --- |
| `uuid` | 来源内稳定track ID生成的稳定UUID |
| `track_id` | 来源内部轨迹ID，融合后为稳定目标ID |
| `pose/twist` | `header.frame_id`坐标系下的位置和速度 |
| `covariance` | 来源估计不确定度 |
| `source_mask` | LiDAR=`1`，Camera=`2`，融合结果附加Fused=`8` |
| `confidence` | `[0,1]`来源置信度 |
| `classification` | 传感器无关目标类别 |
| `last_update` | 产生该观测的传感器测量时间 |

完整契约见：
`src/uav_usv_perception/docs/interfaces/TRACKED_OBJECT_CONTRACT.md`。

## 6. 启动与重复验证

构建：

```bash
cd <your_uav_usv_workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_usv_perception --symlink-install
source install/setup.bash
```

在线Shadow链：

```bash
ros2 launch uav_usv_perception \
  multisensor_fusion_validation.launch.py
```

离线重复验证示例：

```bash
tools/perception/create_multisensor_validation_input.py \
  bags/lv_dot_acceptance/constant_final_20260715 \
  /var/tmp/UAV_USV_lv_dot_phase4/constant_ros2 \
  /var/tmp/UAV_USV_multisensor/constant_input

tools/perception/run_multisensor_validation_replay.sh \
  /var/tmp/UAV_USV_multisensor/constant_input \
  /var/tmp/UAV_USV_multisensor/constant_output 3.0

tools/perception/analyze_multisensor_validation.py \
  /var/tmp/UAV_USV_multisensor/constant_output
```

## 7. 实测结果

同一套参数分别回放匀速、转弯和加速场景。所有Fusion输出都只维护
`target_vessel`一个目标，且`source_mask=11`，即LiDAR、Camera和Fused位全部有效。

| 场景 | UAV检测率 | Fusion检测率 | Fusion平均位置误差 | Fusion平均速度误差 | Fusion ID切换 | 完整来源率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 匀速 | 99.15% | 98.30% | 0.496 m | 0.086 m/s | 0 | 100% |
| 转弯 | 99.32% | 98.44% | 0.554 m | 0.146 m/s | 0 | 100% |
| 加速 | 99.57% | 98.81% | 0.548 m | 0.157 m/s | 0 | 100% |

加速场景中LV-DOT来源自身发生4次ID切换，但融合轨迹仍保持`target_vessel`且
ID切换为0，说明跨来源关联没有把同一敌船拆成多个Fusion Target。

回放输出位置：

- `/var/tmp/UAV_USV_multisensor/constant_output_v5`
- `/var/tmp/UAV_USV_multisensor/turn_output`
- `/var/tmp/UAV_USV_multisensor/acceleration_output`

这些目录是本机测试产物，不提交到Git。

## 8. Qt验证

Qt保留原窗口框架，只扩展现有Perception Monitor：

- Sensor Layer提供Ground Truth、LV-DOT、UAV Camera、Fusion四个开关；
- 每层显示track ID、位置、速度、来源、置信度、误差、时间差和更新时间；
- 显示`LV-DOT track + Camera track -> Fusion track`关联关系；
- Qt仅订阅轻量`/perception/multisensor/metrics`，不在主线程处理图像或点云。

无显示器启动和ROS topic注入测试均通过。测试中Camera行显示`camera_track`，关联
关系显示`LIDAR+CAMERA+FUSED`且关联率为100%。

## 9. 测试汇总

```text
colcon build uav_usv_perception: PASS
colcon test uav_usv_perception: 49 tests, 0 failures
ament_flake8: PASS
shellcheck: PASS
git diff --check: PASS
Qt offscreen startup: PASS
Qt multisensor topic smoke test: PASS
constant/turn/acceleration bag replay: PASS
```

## 10. 已知限制与下一步边界

1. UAV Observation仍是真值代理，真实视觉检测器尚未实现。
2. Camera Observation的误差是代理噪声模型结果，不能作为真实模型精度引用。
3. 当前只验证单敌船和单UAV视觉来源，尚未验证多相机间的数据关联。
4. Fusion仍是Shadow候选源，未接管`/fleet/perception/targets`。
5. 本阶段完成后应停止；Behavior Manager（Escort/Defense/Capture）需要单独立项，
   并在真实视觉检测验证通过后再讨论source mux切换。
