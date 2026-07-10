cmake_minimum_required(VERSION 3.16)
project(uav_usv_sim)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_SIM REQUIRED gz-sim8)
pkg_check_modules(GZ_PLUGIN REQUIRED gz-plugin2)
pkg_check_modules(GZ_MATH REQUIRED gz-math7)
pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)

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

add_gz_system_plugin(BoatWaveFollower plugins/BoatWaveFollower.cc)
add_gz_system_plugin(DroneDeckFollower plugins/DroneDeckFollower.cc)

install(TARGETS
  BoatWaveFollower
  DroneDeckFollower
  LIBRARY DESTINATION lib/${PROJECT_NAME}/plugins
)

install(PROGRAMS
  scripts/keyboard_boat_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_boat_control
)

install(PROGRAMS
  scripts/cooperative_lighthouse_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cooperative_lighthouse_mission
)

install(PROGRAMS
  scripts/rviz_goal_boat_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rviz_goal_boat_control
)

install(PROGRAMS
  scripts/boat_nav2_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME boat_nav2_interface
)

install(PROGRAMS
  scripts/colregs_scenario_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME colregs_scenario_controller
)

install(PROGRAMS
  scripts/ais_simulator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ais_simulator
)

install(PROGRAMS
  scripts/maritime_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME maritime_tf_publisher
)

install(PROGRAMS
  scripts/nav_goal_marker_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav_goal_marker_relay
)

install(PROGRAMS
  scripts/uav_buoy_visual_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME uav_buoy_visual_mission
)

install(PROGRAMS
  tools/build_coastline_assets.py
  tools/prepare_large_x500.py
  tools/run_gz_world.sh
  tools/prepare_coastline.sh
  tools/sync_to_px4.sh
  DESTINATION lib/${PROJECT_NAME}
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
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
