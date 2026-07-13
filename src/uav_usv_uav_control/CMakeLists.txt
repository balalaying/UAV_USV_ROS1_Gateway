cmake_minimum_required(VERSION 3.16)
project(uav_usv_uav_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/uav_fleet_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME uav_fleet_agent
)

install(PROGRAMS
  scripts/uav_dds_fleet_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME uav_dds_fleet_agent
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
