cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/start_ardupilot_332.sh
  scripts/check_332_stack.sh
  scripts/stop_332_stack.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md ROS1_332_QT_GUIDE.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
