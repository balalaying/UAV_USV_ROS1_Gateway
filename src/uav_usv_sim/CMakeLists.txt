cmake_minimum_required(VERSION 3.16)
project(uav_usv_sim)

find_package(ament_cmake REQUIRED)

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

install(DIRECTORY
  launch
  rviz
  config
  docs
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
