# uav_usv_sim

ROS1 Noetic 仿真辅助包，提供舰队位姿 TF、AIS/COLREGs 测试节点、键盘船舶
控制以及 move_base 兼容接口。完整的 3 UAV + 3 USV 仿真由
`uav_usv_bringup` 统一启动。

构建：

```bash
cd ~/uav_usv_ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

当前可用的独立导航入口：

```bash
roslaunch uav_usv_sim boat_move_base_navigation.launch
```

当前工程不再维护 ROS2/Nav2/PX4 启动链；载具运动应通过统一的
`/fleet/command` 接口和 ArduPilot agent 完成。
