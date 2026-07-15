cmake_minimum_required(VERSION 3.16)
project(uav_usv_lv_dot_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uav_usv_interfaces REQUIRED)
find_package(uav_usv_lv_dot_core REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_node SHARED
  src/cluster_marker_conversion.cpp
  src/detector_node.cpp
  src/message_conversion.cpp
  src/pointcloud_conversion.cpp
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  uav_usv_interfaces
  uav_usv_lv_dot_core
  visualization_msgs
)
target_link_libraries(
  ${PROJECT_NAME}_node
  uav_usv_lv_dot_core::uav_usv_lv_dot_core
)

add_executable(lv_dot_detector_node src/main.cpp)
target_link_libraries(lv_dot_detector_node ${PROJECT_NAME}_node)
ament_target_dependencies(lv_dot_detector_node rclcpp rclcpp_lifecycle)

install(
  TARGETS ${PROJECT_NAME}_node lv_dot_detector_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_phase1_conversions
    test/test_phase1_conversions.cpp
  )
  target_link_libraries(test_phase1_conversions ${PROJECT_NAME}_node)
  ament_target_dependencies(
    test_phase1_conversions sensor_msgs uav_usv_interfaces visualization_msgs)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  diagnostic_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2_ros
  uav_usv_interfaces
  uav_usv_lv_dot_core
  visualization_msgs
)
ament_package()
