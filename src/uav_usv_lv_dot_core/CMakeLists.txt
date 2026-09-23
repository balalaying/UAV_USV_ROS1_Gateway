cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_lv_dot_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dbscan.cpp
  src/detector_core.cpp
  src/dynamic_classifier.cpp
  src/kalman_filter.cpp
  src/lidar_clusterer.cpp
  src/multi_object_tracker.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES LICENSE.LV-DOT DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_detector_core test/test_detector_core.cpp)
  target_link_libraries(test_detector_core ${PROJECT_NAME})
endif()
