cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_localization)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs sensor_msgs tf2_ros)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs tf2_ros)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
