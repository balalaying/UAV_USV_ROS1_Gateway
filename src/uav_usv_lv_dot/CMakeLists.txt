cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_lv_dot)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  uav_usv_interfaces
  uav_usv_lv_dot_core
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs roscpp sensor_msgs tf2 tf2_ros
    uav_usv_interfaces uav_usv_lv_dot_core visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_node
  src/cluster_marker_conversion.cpp
  src/detector_node.cpp
  src/message_conversion.cpp
  src/pointcloud_conversion.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(lv_dot_detector_node src/main.cpp)
add_dependencies(lv_dot_detector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lv_dot_detector_node
  ${PROJECT_NAME}_node ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node lv_dot_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
