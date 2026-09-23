# uav_usv_perception

ROS1 Noetic 感知与跟踪包，提供 Gazebo 真值适配、相机/激光雷达观测、
Mid-360 点云预处理、融合、目标跟踪和 Qt 可视化数据。

主仿真会按 `enable_mid360` 参数自动装配点云链路。独立启动感知层：

```bash
cd ~/uav_usv_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav_usv_perception perception_layer.launch perception_source:=ground_truth
```

主要接口：

- `/fleet/perception/targets`：统一目标输出。
- `/fleet/sensor_status`：传感器在线率、延迟和健康状态。
- `/fleet/uplink/usv_01/mid360/points`：USV 1 原始点云。
- `/perception/usv_01/mid360/points_filtered`：预处理点云。

本包已移除旧 ROS2 LV-DOT shadow/Docker 转发链。现有 LV-DOT ROS1 实现位于
`uav_usv_lv_dot` 和 `uav_usv_lv_dot_core`，不需要跨 ROS 版本转发。
