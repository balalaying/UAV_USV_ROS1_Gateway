cmake_minimum_required(VERSION 3.16)
project(uav_usv_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_SIM REQUIRED gz-sim8)
pkg_check_modules(GZ_PLUGIN REQUIRED gz-plugin2)
pkg_check_modules(GZ_MATH REQUIRED gz-math7)
pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)

catkin_package()

function(add_gz_system_plugin target source)
  add_library(${target} SHARED ${source})
  target_include_directories(${target} PRIVATE
    ${GZ_SIM_INCLUDE_DIRS}
    ${GZ_PLUGIN_INCLUDE_DIRS}
    ${GZ_MATH_INCLUDE_DIRS}
    ${GZ_TRANSPORT_INCLUDE_DIRS}
    ${GZ_MSGS_INCLUDE_DIRS}
  )
  target_link_directories(${target} PRIVATE
    ${GZ_SIM_LIBRARY_DIRS}
    ${GZ_PLUGIN_LIBRARY_DIRS}
    ${GZ_MATH_LIBRARY_DIRS}
    ${GZ_TRANSPORT_LIBRARY_DIRS}
    ${GZ_MSGS_LIBRARY_DIRS}
  )
  target_link_libraries(${target} PRIVATE
    ${GZ_SIM_LIBRARIES}
    ${GZ_PLUGIN_LIBRARIES}
    ${GZ_MATH_LIBRARIES}
    ${GZ_TRANSPORT_LIBRARIES}
    ${GZ_MSGS_LIBRARIES}
  )
endfunction()


add_executable(gz_laserscan_bridge
  src/gz_laserscan_bridge.cpp
)

target_include_directories(gz_laserscan_bridge PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
)

target_link_directories(gz_laserscan_bridge PRIVATE
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
)

target_link_libraries(gz_laserscan_bridge PRIVATE
  ${catkin_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)

install(TARGETS gz_laserscan_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/keyboard_boat_control.py
  scripts/rviz_goal_boat_control.py
  scripts/boat_nav2_interface.py
  scripts/colregs_scenario_controller.py
  scripts/ais_simulator.py
  scripts/maritime_tf_publisher.py
  scripts/fleet_pose_tf_publisher.py
  scripts/nav_goal_marker_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  tools/build_coastline_assets.py
  tools/run_gz_world.sh
  tools/prepare_coastline.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  worlds
  models
  launch
  rviz
  config
  maps
  docs
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
