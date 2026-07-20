# 视觉主导 USV 感知实施报告

## 1. 实现范围

完成 Camera 身份检测、类型安全元数据、身份多帧滤波、Camera ROI 点云提取、
局部 DBSCAN、鲁棒 3D BBox、LV-DOT Track 兼容、LiDAR fallback、Fusion 身份保持、
Qt 着色/诊断、Launch/YAML、仿真 visual 身份板和自动化测试。

未修改 PX4、Nav2、FleetCommand、capture_manager、vehicle agents 或 source mux 默认值。

## 2. 主要文件

新增：

- `uav_usv_interfaces/msg/AffiliatedDetection2D*.msg`；
- `uav_usv_perception/scripts/vision_guided/vision_guided_core.py`；
- `scripts/adapters/usv_camera_detection_node.py`；
- `scripts/fusion/vision_guided_lidar_roi_node.py`；
- `scripts/fusion/camera_lidar_association_node.py`；
- `config/vision_guided_usv_perception.yaml`；
- `launch/camera_lidar_fusion.launch.py`；
- `test/test_vision_guided_perception.py`。

修改：`TrackedObject.msg`、接口/感知 CMake、`perception_fusion_node.py`、
`tf_topic_relay.py`、四个 Gazebo 船模型 visual、Qt 绘图与 GUI、两个 bringup launch。

## 3. 消息兼容

所有字段追加在消息末尾，不重排旧 enum；ROS 2 Python/C++ 消息已重新生成。
未填写身份的旧节点默认 UNKNOWN。关联质量通过 `association_score` 和
`bbox_point_count` 直接进入 Qt，不依赖自由文本解析。

## 4. 实测环境与启动

- ROS 2 Humble；Gazebo Harmonic；2026-07-18；
- `start_px4=false start_dds_agent=false start_rviz=false`，完整舰队世界；
- 为可重复传感器测试，在相机前方临时生成红/蓝静态方体，位置分别为
  `(9,2,1)` 与 `(9,5,1)`；它们不写入世界文件且不参与控制；
- LV-DOT lifecycle：`active [3]`。

## 5. 实时结果

5 秒独立计数：

| 数据 | 频率 |
|---|---:|
| USV Camera | 9.34 Hz |
| Mid-360 filtered cloud | 15.69 Hz |
| Camera-LiDAR observations | 15.49 Hz |

120 帧连续样本：

| 指标 | FRIENDLY | HOSTILE |
|---|---:|---:|
| 有效样本 | 120/120 | 120/120 |
| 稳定 ID 数 | 1 | 1 |
| 平均位置误差 | 1.218 m | 1.203 m |
| P95 位置误差 | 1.259 m | 1.221 m |
| X/Y 中心抖动 | 0.018/0.027 m | 0.012/0.034 m |
| 长宽高抖动 | 0.044/0.014/0.065 m | 0.064/0.033/0.089 m |
| 平均关联分数 | 0.521 | 0.494 |
| 平均框点数 | 17.7 | 14.9 |

身份准确率为 240/240，身份切换 0，Track ID 切换 0，TF 失败 0。
输出去重后实测 16.84 Hz，120/120 帧同时包含两个 Camera+LiDAR 目标。

## 6. 状态与延迟

最终状态快照：`fused_count=2`、`camera_only_count=0`、`lidar_only_count=0`、
`control_connected=false`、`perception_source=ground_truth`。
Camera 单帧处理约 6.4 ms，关联节点约 0.72 ms；ROI 状态快照为 37.7 ms，
同步误差约 41.4 ms。ROI 处理为 Python/Numpy 实现，是当前主要 CPU 消耗。

## 7. 资源与带宽

同一时刻进程采样：ROI 72.4% 单核、约 80.6 MB RSS；Camera 9.5%、101.7 MB；
Association 17.7%、75.3 MB；Mid360 preprocessor 13.0%、73.1 MB。
GPU 利用率 9%，显存 1486 MB；Gazebo real-time factor 0.99997。
过滤点云约 0.78-0.86 MB/s，融合 Observation 约 30-38 KB/s。

## 8. 旧链路对比

旧链路以全局 Marker/BBox 回调驱动，实测曾出现 1 个与视觉目标无关的全局
LiDAR candidate，并在全局 Marker 为空时错误发布空数组。新链路在同一双目标
采样中 false fused bbox 为 0、valid recall 为 240/240、match rate 为 100%。
ROI 外水面和岸边点不会进入主要融合框；原全局链仍保留为 fallback。

这不是大规模统计结论。可重复的算法验证由 15 个视觉引导单测覆盖，严格海况、
夜间和遮挡仍需专门 rosbag 做长期评估。

## 9. 场景覆盖

| 场景 | 结果 |
|---|---|
| Friendly + Hostile 同视场 | Gazebo 120/120 双目标通过 |
| 相近目标一对一 | 可用点 mask 单测 + 双目标实测通过 |
| Camera 离开/丢帧 | 身份 hold/timeout 单测通过，LV-DOT fallback 保留 |
| 水面噪声 | 水面门限与 ROI 外排除生效 |
| Camera-only | 大协方差输出实测/代码路径通过 |
| LiDAR-only | UNKNOWN 强制规则与 fallback 保留 |
| Unknown/无目标 | 颜色噪声与空检测单测通过 |
| Qt | offscreen 运行 12 秒无崩溃；未完成 30 分钟长期测试 |

## 10. 构建与测试

- 7 个相关包构建成功；只有 rosidl 的 CMake CMP0148 开发警告；
- `colcon test` 最终汇总：83 tests，0 errors，0 failures，0 skipped；
- Python `py_compile`、4 个 SDF `xmllint`、YAML 解析、`git diff --check` 均通过；
- Qt offscreen 启动正常，仅有 offscreen 平台的 `propagateSizeHints` 提示。

## 11. TF 与安全

`map -> usv_01/base_link -> camera_link/mid360_link` 可按消息时间查询。
实测 Camera 位姿约 `[-12.680,2.018,2.159]`，Mid360 约
`[-14.989,1.961,2.035]`，均随 USV 运动。

`/fleet/perception/targets` 唯一发布者仍是 `target_tracker`；订阅者为原
`capture_manager` 与 visualizer。新节点没有发布 FleetCommand 或底层控制 Topic。

## 12. 已知限制

1. 当前语义后端识别仿真身份板，不是通用真实船舶检测器；
2. LiDAR 只看到目标迎面表面，中心沿视线存在约 1.2 m 系统偏差；已用保守厚度补全，
   但不是完整物体重建；
3. ROI Python 节点占用一个 CPU 核的约 72%，后续可迁移 C++/PCL 或批量投影；
4. ROS daemon 偶有发现延迟，重启 daemon 可恢复 CLI 检查；
5. 快速重复启动 Gazebo 曾出现一次世界服务无响应，彻底清理 GZ 进程后恢复；
6. Qt 只完成 12 秒自动无界面回归，长期稳定性需继续跑 30 分钟。

## 13. 未来 YOLO 替换

实现 `DetectorBackend.detect(image)`，输出同一 `VisualCandidate`，并把 backend 注册到
节点构造器即可。后端必须继续输出 class 与 affiliation；ROI、消息、Fusion、Qt 无需改动。
