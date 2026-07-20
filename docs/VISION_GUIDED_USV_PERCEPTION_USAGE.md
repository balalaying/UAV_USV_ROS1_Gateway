# 视觉主导 USV 感知使用说明

## 1. 构建

```bash
cd <你的工作区>/UAV_USV
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  uav_usv_interfaces uav_usv_gazebo uav_usv_lv_dot_core \
  uav_usv_lv_dot_ros2 uav_usv_perception uav_usv_mission \
  uav_usv_bringup --allow-overriding uav_usv_interfaces
source install/setup.bash
```

## 2. 推荐启动

完整 Shadow 感知，不启动 PX4、DDS 和 RViz：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py \
  start_px4:=false start_dds_agent:=false start_rviz:=false
```

完整演示与 Qt：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

默认参数已经是：

```text
enable_vision_guided_perception=true
enable_global_lidar_fallback=true
enable_affiliation_filter=true
enable_affiliation_qt_mode=true
camera_detector_backend=simulation_marker
vision_guided_shadow_mode=true
```

## 3. Qt 使用

在 Perception Monitor 中选择：

- `Sensor Source`：LiDAR 黄色、Camera 蓝色、Camera+LiDAR 绿色；
- `Affiliation`：FRIENDLY 蓝/青、HOSTILE 红、NEUTRAL 灰/绿、UNKNOWN 黄。

右侧可查看相机检测率、三类输出数、ROI 点数、处理延迟、TF 失败和身份切换。
图层开关只控制显示，不会改变算法或任务控制。

## 4. 关键检查

```bash
ros2 lifecycle get /perception/lv_dot_ros2/lv_dot_detector_node
ros2 topic hz /perception/usv_01/mid360/points_filtered
ros2 topic hz /perception/usv_01/vision_guided/observations
ros2 topic echo /perception/usv_01/camera/detection_status --once
ros2 topic echo /perception/usv_01/vision_guided/status --once
ros2 topic echo /perception/usv_01/camera_lidar/status --once
ros2 run tf2_ros tf2_echo map usv_01/camera_link
ros2 run tf2_ros tf2_echo map usv_01/mid360_link
ros2 topic info -v /fleet/perception/targets
```

正确状态应包含：`control_connected=false`、`perception_source=ground_truth`、
LV-DOT `active`，且最终目标 Topic 发布者仍为 `target_tracker`。

## 5. 参数调整

统一配置文件：

```text
src/uav_usv_perception/config/vision_guided_usv_perception.yaml
```

- 漏点多：增大 `roi_expand_pixels`，降低 `minimum_roi_points`；
- 水面误点：提高 `water_surface_margin`；
- 目标簇断裂：增大 `local_cluster_epsilon` 或降低 `local_cluster_min_samples`；
- 框抖动：降低 `bbox_smoothing_alpha` 或 `maximum_bbox_jump`；
- 合法快速运动被拒：增大 `maximum_bbox_jump`，不要先放大全局 DBSCAN；
- CPU 高：降低 `maximum_roi_points`；
- 身份抖动：提高 `affiliation_switch_frames`；
- 短时遮挡身份丢失：增大 `affiliation_hold_seconds`。

每次修改后重新启动节点即可，`--symlink-install` 下不必复制配置。

## 6. Detector Backend

当前可运行后端是：

```bash
camera_detector_backend:=simulation_marker
```

`FutureYoloDetectorBackend` 是接口边界，尚未安装权重和推理依赖；选择未实现后端会明确
报错而不是静默伪造结果。接入 YOLO 时保持 `VisualCandidate` 输出契约即可。

## 7. 常见故障

- `TF failure` 增长：确认 `/usv_01/tf` 和 `/tf` 有非零时间戳，不能用 latest TF 代替；
- Camera 有框但 ROI 空：检查 CameraInfo、目标是否处于 Mid360 视场、Z 门限和同步误差；
- 兼容 observations 为空：确认视觉引导 observations 在线；关联节点现在由视觉回调触发；
- Qt 无数据：确认 Topic QoS 为 SensorData，GUI 只缓存最新帧；
- Gazebo 显示但传感器不发布：先彻底结束旧 GZ 进程再重启；
- ROS CLI 看不到节点：执行 `ros2 daemon stop && ros2 daemon start`。

清理命令：

```bash
./tools/clean_capture_processes.sh
```
