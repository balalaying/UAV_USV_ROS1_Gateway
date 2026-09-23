cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_ros1_compat)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
