cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_tests)

find_package(catkin REQUIRED COMPONENTS uav_usv_interfaces)

catkin_package(CATKIN_DEPENDS uav_usv_interfaces)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
