cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_navigation)

find_package(catkin REQUIRED COMPONENTS move_base uav_usv_interfaces uav_usv_usv_control)

catkin_package(CATKIN_DEPENDS move_base uav_usv_interfaces uav_usv_usv_control)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
