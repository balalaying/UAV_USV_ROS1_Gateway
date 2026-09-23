cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_cooperative_algorithms)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  uav_usv_interfaces
  visualization_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  rospy
  std_msgs
  uav_usv_interfaces
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/cooperative_algorithm_controller.py
  scripts/gbsflacs_capture_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/check_algorithms.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
