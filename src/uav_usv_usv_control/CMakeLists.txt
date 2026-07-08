cmake_minimum_required(VERSION 3.16)
project(uav_usv_usv_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/usv_fleet_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME usv_fleet_agent
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
