cmake_minimum_required(VERSION 3.16)
project(uav_usv_mission)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

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

install(PROGRAMS
  scripts/target_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_tracker
)

install(PROGRAMS
  scripts/capture_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME capture_manager
)

install(PROGRAMS
  scripts/capture_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME capture_visualizer
)

install(PROGRAMS
  scripts/capture_target_motion.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME capture_target_motion
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    capture_algorithms
    test/test_capture_algorithms.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
