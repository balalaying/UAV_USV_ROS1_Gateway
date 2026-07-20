# Qt LV-DOT全过程感知可视化报告

## 1. 实现范围

本次修改只增加被动显示链路，不修改LV-DOT、感知融合、任务管理和载具控制算法。

原先功能重叠的 `Perception Monitor` 与 `LV-DOT Perception Debug` 已合并为一个
`Perception Monitor` 页面。页面保留原有黑色三维感知画布、斜俯视/正俯视切换、
UAV-01相机画中画和右侧实时状态，同时增加LV-DOT各处理阶段的独立图层。

系统仍处于Shadow Mode，`perception_source` 保持 `ground_truth`。Qt不会发布
`/fleet/perception/targets`，也不会向控制链发送感知调试结果。

## 2. 数据流

```text
Mid-360原始PointCloud2
  -> qt_pointcloud_projection_node
  -> /perception/visualization/usv_01/topdown_points
  -> Qt raw cloud layer

mid360_preprocessor
  -> /perception/usv_01/mid360/points_filtered
  -> lv_dot_debug_visualization_node
  -> /perception/lv_dot/debug/cloud
  -> qt_pointcloud_projection_node
  -> /perception/lv_dot/debug/cloud_filtered_map
  -> Qt filtered cloud layer

LV-DOT bbox / tracks / dynamic_tracks
  -> lv_dot_debug_visualization_node
  -> debug clusters / bboxes / tracks / dynamic
  -> Qt layer cache

perception_fusion
  -> /perception/fused/tracks
  -> Qt fusion layer

/tf + /tf_static
  -> map -> usv_01/base_link -> usv_01/mid360_link
  -> Qt vessel/Mid-360 TF layer
```

ROS回调线程只写入线程安全的“最新帧”缓存。Qt线程以40毫秒定时器读取快照并绘制，
不会在ROS回调中操作控件，也不会积压历史点云。

## 3. Debug Topic

| Topic | 类型 | Qt显示内容 |
|---|---|---|
| `/perception/visualization/usv_01/topdown_points` | `sensor_msgs/PointCloud2` | Mid-360原始点云 |
| `/perception/lv_dot/debug/cloud` | `sensor_msgs/PointCloud2` | LV-DOT过滤点云调试出口 |
| `/perception/lv_dot/debug/cloud_filtered_map` | `sensor_msgs/PointCloud2` | map坐标过滤点云 |
| `/perception/lv_dot/debug/clusters` | `visualization_msgs/MarkerArray` | DBSCAN聚类中心 |
| `/perception/lv_dot/debug/bboxes` | `visualization_msgs/MarkerArray` | 三维框 |
| `/perception/lv_dot/debug/tracks` | `TrackedObjectArray` | Track、轨迹和速度 |
| `/perception/lv_dot/debug/dynamic` | `TrackedObjectArray` | 动态目标状态 |
| `/perception/fused/tracks` | `TrackedObjectArray` | 融合目标 |
| `/perception/lv_dot/debug/status` | `std_msgs/String` | 各级频率和数量统计 |

`/perception/lv_dot/debug/cloud` 没有复制原始高带宽点云，而是复用LV-DOT实际输入，
避免为显示额外占用一份DDS带宽。

## 4. 页面与图层

合并后的 `Perception Monitor` 支持：

- Mid-360原始点云；
- 过滤后点云；
- DBSCAN Clusters；
- 3D Bounding Box；
- Tracks、轨迹和速度；
- Dynamic状态；
- Fusion Target；
- `usv_01/base_link` 与 `usv_01/mid360_link` TF；
- Track标签和Map网格；
- UAV-01相机画中画；
- 斜俯视3D与Map正俯视2D切换。

点云采用有限时间窗口累积和显示点数上限。默认原始点云累积最近6帧，过滤点云累积
最近4帧，体素尺寸分别为0.015米和0.03米，画布每层最多显示60000点。该设置只提高
显示密度，不修改LV-DOT实际接收的单帧点云。关闭某一图层只停止Qt绘制，不改变
对应ROS算法。

## 5. 启动方式

实时Gazebo传感器演示：

```bash
cd <你的UAV_USV工作区>
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_usv_bringup \
  fleet_dynamic_capture_live_perception.launch.py
```

只验证感知显示、不启动PX4和RViz：

```bash
ros2 launch uav_usv_bringup \
  fleet_dynamic_capture_live_perception.launch.py \
  start_px4:=false start_dds_agent:=false start_rviz:=false
```

可选参数：

```bash
enable_lv_dot_debug:=true
enable_pointcloud_projection:=true
topdown_point_rate:=20.0
topdown_max_points:=80000
```

## 6. 实测结果

2026-07-18在完整Gazebo舰队世界、关闭PX4与RViz的显示验证中：

| 项目 | 结果 |
|---|---|
| 原始点云投影 | 约18.7至19.3 Hz |
| 过滤点云投影 | 约18.6至19.1 Hz |
| 单帧Mid-360输入 | 约2550至2730点 |
| 密集原始显示 | 约16684点（最近6帧） |
| 密集过滤显示 | 约10383点（最近4帧） |
| UAV-01相机输入 | 稳态约9.6 Hz |
| `map -> usv_01/mid360_link` | 连续可查询，随船体运动 |
| Qt ROS线程 | 未阻塞，GUI进程持续在线 |
| 自动控制连接 | `false`，保持Shadow安全边界 |
| 单元测试 | 5项通过 |

当前测试位置没有目标聚类时，Cluster/BBox/Track数量正确显示为0；显示层不会伪造目标。

## 7. 主要文件

- `uav_usv_perception/scripts/visualization/lv_dot_debug_visualization_node.py`：
  统一LV-DOT调试话题和轻量统计。
- `uav_usv_mission/uav_usv_mission/lv_dot_debug_visualization.py`：
  线程安全模型和PyQtGraph OpenGL绘制组件。
- `uav_usv_mission/scripts/fleet_base_station_gui.py`：
  合并页面、ROS订阅、相机画中画、TF查询和状态显示。
- `uav_usv_bringup/launch/dynamic_capture_console.launch.py`：
  调试节点、点云投影节点和Qt启动参数。

## 8. 已知问题

- 显示帧率仍受同时显示的点数、Gazebo渲染和相机数量影响；演示时可先关闭“原始点云”，
  只保留过滤点云和目标框。
- 目标不在Mid-360量程或没有形成有效聚类时，不会显示3D框，这是感知输入状态而非Qt故障。
- 当前只显示 `usv_01` 的Mid-360 TF；后续多雷达实例应通过参数增加frame，而不是硬编码复制页面。
