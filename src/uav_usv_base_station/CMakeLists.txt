cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_base_station)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs uav_usv_ros1_compat)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy std_msgs uav_usv_ros1_compat)

catkin_install_python(PROGRAMS scripts/base_station_service
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
