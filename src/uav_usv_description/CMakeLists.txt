cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_description)

find_package(catkin REQUIRED)

catkin_package()

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
