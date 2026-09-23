# uav_usv_gazebo

ROS1 UAV/USV 项目的 Gazebo Harmonic 世界、模型和系统插件包。

当前 3+3 主场景是 `worlds/heterogeneous_332.sdf`，包含 3 架 ArduPilot
UAV、3 艘 ArduPilot USV、友船、敌船和岸基环境。插件包括电机桥、Rover
桥、船体运动和可视实体跟随等实现。

通常不要单独启动世界，应通过 bringup 包启动完整控制链：

```bash
cd ~/uav_usv_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch \
  start_ardupilot:=true start_algorithms:=false
```

只检查 Gazebo 世界时可运行：

```bash
rosrun uav_usv_gazebo run_gz_world.sh heterogeneous_332.sdf
```

本包使用仓库内 `third_party/ardupilot_gazebo` 和系统 Gazebo Harmonic，
不需要 PX4 资源同步。所有 ArduPilot 模型必须保持
`<lock_step>0</lock_step>`。
