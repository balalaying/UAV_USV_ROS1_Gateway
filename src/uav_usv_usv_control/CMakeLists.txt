cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_usv_control)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs uav_usv_interfaces uav_usv_ros1_compat)
catkin_package(CATKIN_DEPENDS rospy sensor_msgs uav_usv_interfaces uav_usv_ros1_compat)
catkin_install_python(PROGRAMS scripts/usv_gz_fleet_agent.py scripts/usv_fleet_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
