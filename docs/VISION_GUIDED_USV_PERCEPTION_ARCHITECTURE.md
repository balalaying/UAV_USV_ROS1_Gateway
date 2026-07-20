# 视觉主导 USV 感知架构

## 1. 调整目标

本次只调整 USV 感知子主线。控制与任务主线仍为：

```text
/fleet/perception/targets (ground truth)
  -> capture_manager -> FleetCommand -> vehicle agents -> PX4 / Nav2
```

新链路运行在 Shadow Mode，不发布控制命令，也不发布
`/fleet/perception/targets`。

## 2. 调整前后

调整前：

```text
Mid-360 -> 全场预处理 -> 全局 DBSCAN/BBox/Track
                              + Camera 后置语义关联
```

调整后：

```text
USV Camera -> Detection2D + class + affiliation
                      |
Mid-360 filtered -----+-> 历史 TF 投影 -> Camera ROI
                          -> 深度过滤 -> 局部 DBSCAN
                          -> 鲁棒 PCA BBox -> 跳变/尺寸平滑
                          -> 关联 LV-DOT Track
                          -> Camera+LiDAR TrackedObjectArray
                          -> perception_fusion -> Qt

Mid-360 -> 原 LV-DOT 全局链 -> LiDAR-only fallback
```

Camera 回答“是什么、属于哪一方”；LiDAR 回答“在哪里和多大”；
LV-DOT 提供 Track、速度、动态状态与视觉失效保底。

## 3. 节点职责

| 节点 | 职责 | 是否连接控制 |
|---|---|---|
| `usv_01_camera_detection` | 仿真身份板检测、2D 框、类别、阵营、多帧确认 | 否 |
| `usv_01_vision_guided_lidar_roi` | 历史 TF、投影、ROI 点云、局部聚类、稳定 3D 框 | 否 |
| `usv_01_camera_lidar_association` | 兼容旧输出、复用 LV-DOT Track、保留全局 fallback | 否 |
| `perception_fusion` | 合并多源位置、速度、source mask 与身份 | 否 |
| `tf_topic_relay` | 将 USV TF 接入全局 `/tf`，仅修复零时间戳 | 否 |
| `fleet_base_station_gui` | 感知结果、诊断和相机画面显示 | 否 |

## 4. Topic 数据流

| Topic | 类型 | 说明 |
|---|---|---|
| `/fleet/uplink/usv_01/camera/image_raw` | `sensor_msgs/Image` | 原始船载相机 |
| `/fleet/uplink/usv_01/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/perception/usv_01/camera/detections` | `vision_msgs/Detection2DArray` | 兼容 2D 检测 |
| `/perception/usv_01/camera/affiliated_detections` | `AffiliatedDetection2DArray` | 类型安全的身份元数据 |
| `/perception/usv_01/camera/detections/image` | `sensor_msgs/Image` | 标注图像 |
| `/perception/usv_01/mid360/points_filtered` | `sensor_msgs/PointCloud2` | 标准过滤点云 |
| `/perception/lv_dot_ros2/tracks` | `TrackedObjectArray` | LV-DOT Track 候选 |
| `/perception/usv_01/vision_guided/roi_cloud` | `PointCloud2` | 被 Camera ROI 接受的点 |
| `/perception/usv_01/vision_guided/roi_clusters` | `MarkerArray` | 局部簇调试显示 |
| `/perception/usv_01/vision_guided/roi_bboxes` | `MarkerArray` | 稳定 3D 框 |
| `/perception/usv_01/vision_guided/observations` | `TrackedObjectArray` | Camera 主导原始结果 |
| `/perception/usv_01/camera_lidar/observations` | `TrackedObjectArray` | 兼容融合出口 |
| `.../camera/detection_status` | `std_msgs/String` JSON | Camera 统计 |
| `.../vision_guided/status` | `std_msgs/String` JSON | ROI/TF/耗时统计 |
| `.../camera_lidar/status` | `std_msgs/String` JSON | 三类输出统计 |

## 5. 消息语义

`TrackedObject` 保留原字段并在末尾兼容增加：

- `AFFILIATION_UNKNOWN/FRIENDLY/HOSTILE/NEUTRAL`；
- `affiliation`、`affiliation_confidence`；
- `bbox_point_count`、`association_score`。

`class_name=vessel` 表示目标类别，`affiliation=HOSTILE` 表示阵营，二者不混用。
旧发布者未赋值时自然得到 `UNKNOWN/0.0`。LiDAR-only 强制为 UNKNOWN。

## 6. Camera 检测

`DetectorBackend` 隔离检测器实现。当前
`SimulationMarkerDetectorBackend` 使用 HSV 多颜色分量、形态学开闭运算、
面积/尺寸/长宽比/图像高度过滤，并对重叠连通域去重。蓝、红、绿局部身份板
分别映射 FRIENDLY、HOSTILE、NEUTRAL；无可靠标识为 UNKNOWN。
`FutureYoloDetectorBackend` 只定义替换边界，不引入模型或大型依赖。

## 7. 身份时间滤波

每个视觉临时 ID 保存最近 N 帧投票、确认身份、置信度、最后更新时间和切换原因。
默认 3 帧确认、FRIENDLY/HOSTILE 间 4 帧才切换、1 秒保持、2 秒超时降为 UNKNOWN。
LiDAR-only 只有在完全相同 Track ID 上才能短时继承身份；新 Track 不继承。

## 8. Camera ROI 到 LiDAR

1. 按点云时间戳寻找最近的 Detection 与 CameraInfo；
2. 严格查询该时间戳的 `camera <- lidar`、`map <- lidar` TF；
3. 去 NaN/Inf、相机后方、距离外和水面点；
4. 投影至图像并按扩展 ROI 取点；
5. 百分位与 MAD 深度过滤；
6. ROI 内空间哈希 DBSCAN；
7. 一对一占用点索引，避免同簇重复认领；
8. 选择点数/预测距离最优的簇；
9. 转到 map 后生成 BBox。

TF 失败时丢弃该帧，不使用最新 TF 掩盖错误。`tf_topic_relay` 只把上游明确为
零的时间戳修复为接收时系统时间；非零时间戳原样保留。

## 9. 稳定 3D BBox

BBox 使用 5%/95% trimmed extrema、XY PCA 朝向和中位中心，不使用单纯 min/max。
单侧可见表面通过可配置的 `minimum_occluded_extent` 补足保守厚度。EMA 平滑中心、
尺寸和 yaw；超过 `maximum_bbox_jump` 的单帧测量沿用上一稳定框并降权，
新位置连续 3 帧一致才接受，兼顾抗串扰和真实运动。

## 10. 三类降级

- Camera+LiDAR：米制位置、小协方差、类别和阵营、`SOURCE_FUSED`；
- Camera-only：单目估距、大协方差 25、保留类别和阵营；
- LiDAR-only：原 LV-DOT 几何/速度，阵营 UNKNOWN；同 Track 可短时身份保持。

## 11. Fusion 与 Qt

Fusion 从非 Ground Truth 的已知身份中选择最高身份置信度，UNKNOWN 不覆盖短时
稳定身份，Ground Truth 不给真实感知补身份。Qt 支持 Source 与 Affiliation 两种着色，
标签显示 Track、类别、阵营、置信度、来源、速度、动态状态、关联分数和点数。
ROS 回调只更新线程安全缓存，Qt 定时器负责绘制。

## 12. 参数与边界

参数集中在 `config/vision_guided_usv_perception.yaml`。关键值：ROI 扩展 14 px、
最大 ROI 点数 600、局部 DBSCAN `eps=0.85`、BBox EMA 0.45、跳变门限 0.75 m、
持续跳变确认 3 帧。Launch 默认开启视觉链与身份模式，但始终是 Shadow Mode。
