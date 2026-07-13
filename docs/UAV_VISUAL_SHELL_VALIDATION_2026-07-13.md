# UAV 放大显示层验证记录

日期：2026-07-13

## 目标

在不修改 PX4 动力学模型的前提下，将四架无人机在 Gazebo 和 RViz 中放大显示，保留完整的多 PX4 围捕闭环。

## 实现

- 真实实体仍为 `uav_01` 至 `uav_04`，质量、惯量、碰撞、电机、传感器和 PX4 接口未修改。
- 显示实体为 `uav_01_large_visual` 至 `uav_04_large_visual`。
- 显示实体复用 PX4 `x500_base` 的原版机架、电机和桨叶 mesh，只包含 visual。
- `VisualPoseFollower` 从真实 x500 的 canonical link 恢复模型世界位姿，再同步显示实体。
- 显示节点逐个隐藏真实 x500 机架和旋翼 link 的小尺寸 visual，连续三轮确认后才停止重试；不删除真实模型或 link。
- 默认显示比例为 `uav_visual_scale:=12.0`，可以通过 launch 参数修改。
- Gazebo 显示 UAV 编号、颜色和角色；RViz 同步显示载具比例、角色、任务点和速度方向。

## 修改文件

- `src/uav_usv_gazebo/plugins/VisualPoseFollower.cc`
- `src/uav_usv_gazebo/models/large_uav_visual_shell/model.sdf`
- `src/uav_usv_gazebo/models/large_uav_visual_shell/model.config`
- `src/uav_usv_gazebo/CMakeLists.txt`
- `src/uav_usv_mission/scripts/uav_visual_shell_spawner.py`
- `src/uav_usv_mission/scripts/capture_visualizer.py`
- `src/uav_usv_mission/CMakeLists.txt`
- `src/uav_usv_bringup/launch/fleet_dynamic_capture.launch.py`

## 启动命令

```bash
cd <你的UAV_USV工作区>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  uav_visual_scale:=12.0
```

临时恢复六倍显示：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  uav_visual_scale:=6.0
```

## 验证命令

```bash
colcon build --packages-select \
  uav_usv_gazebo uav_usv_mission uav_usv_bringup --symlink-install

ros2 param get /uav_visual_shell_spawner uav_visual_scale
ros2 topic info /fleet/command -v
ros2 topic echo --once /capture/roles
```

## 运行结果

- 三个包构建成功，Python 语法检查和 SDF 校验通过。
- 四个 DDS agent 分别使用 system ID 1、2、3、4。
- 四架 UAV 均成功解锁、起飞并进入 `PX4/NAVIGATE` 或到点后的 `PX4/HOLD`。
- `/fleet/command` 保持 `FleetCommand`，一个任务发布者和六个 agent 订阅者。
- 四架 UAV 获得不同 `air_observer` 槽位，两艘 USV 保持 `surface_interceptor` 角色。
- 显示外壳与真实 UAV 同帧位置误差约为 0.0015 至 0.0033 米。
- 真实小模型 visual 已隐藏，Gazebo 只显示十二倍原版 x500 外壳。
- 运行日志目录：`/home/dji/.ros/log/2026-07-13-22-43-28-133419-dji-Legion-R7000-AHP9-109037`。

## 已知问题

- 放大外壳的桨叶是展示用静态 visual，不模拟旋翼转速；真实旋翼动力学仍由隐藏的 PX4 实体执行。
- PX4 自带 SDF 会输出 `gz_frame_id` 兼容性警告，但不影响 DDS、OFFBOARD 或位置控制。
- 原版 mesh 来自 PX4 的 `x500_base`，因此运行时仍需正确设置 PX4 模型资源路径。
