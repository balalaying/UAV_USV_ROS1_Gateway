cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_colregs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs rospy uav_usv_interfaces)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs rospy uav_usv_interfaces)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
