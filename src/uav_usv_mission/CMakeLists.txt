cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_mission)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_msgs
  uav_usv_interfaces
  uav_usv_ros1_compat
  visualization_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
  std_srvs tf2_ros tf2_msgs uav_usv_interfaces uav_usv_ros1_compat visualization_msgs)

catkin_install_python(PROGRAMS
  scripts/fleet_base_station.py
  scripts/fleet_base_station_gui.py
  scripts/gz_sensor_bridge.py
  scripts/gz_entity_truth_bridge.py
  scripts/capture_gazebo_camera_follow.py
  scripts/capture_manager.py
  scripts/capture_mission.py
  scripts/capture_target_motion.py
  scripts/capture_visualizer.py
  scripts/cooperative_response_mission.py
  scripts/defense_demo.py
  scripts/fleet_behavior_manager.py
  scripts/fleet_simulated_agent.py
  scripts/fleet_world_model_node.py
  scripts/target_tracker.py
  scripts/uav_visual_shell_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
