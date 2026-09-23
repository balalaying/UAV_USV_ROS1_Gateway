# uav_usv_bringup

ROS1 Noetic 集成启动包。当前主环境为 Gazebo Harmonic + ArduPilot SITL，
不使用 PX4、ROS2 或 `colcon`。

## 3 UAV + 3 USV 主入口

```bash
cd ~/uav_usv_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch \
  start_ardupilot:=true start_algorithms:=false
```

默认行为：启动 Gazebo 世界、6 个 ArduPilot SITL、6 个 Fleet agent 和 Qt；
协同算法默认关闭。关闭 Qt 可传 `start_gui:=false`，关闭 Mid-360 可传
`enable_mid360:=false`。

算法演示入口：

```bash
roslaunch uav_usv_bringup gbsflacs_332_sim.launch
```

该入口会自动执行护航、守卫和围捕流程。基础载具调试时不要使用它。

## 关键脚本

- `scripts/start_ardupilot_332.sh`：启动 3 个 ArduCopter 和 3 个 ArduRover。
- `scripts/stop_heterogeneous_332.sh`：关闭本项目相关 ROS、Gazebo、Qt 和 SITL 进程。
- `ROS1_332_QT_GUIDE.md`：详细启动和排障说明。

所有飞行器/船舶模型必须保持 `<lock_step>0</lock_step>`。
