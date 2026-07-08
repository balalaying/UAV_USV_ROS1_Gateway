cmake_minimum_required(VERSION 3.16)
project(uav_usv_mission)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/fleet_base_station.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fleet_base_station
)

install(PROGRAMS
  scripts/fleet_base_station_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fleet_base_station_gui
)

install(PROGRAMS
  scripts/fleet_simulated_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fleet_simulated_agent
)

install(PROGRAMS
  scripts/defense_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME defense_demo
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
