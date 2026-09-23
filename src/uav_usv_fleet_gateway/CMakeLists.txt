cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_fleet_gateway)

find_package(catkin REQUIRED COMPONENTS
  rospy sensor_msgs std_msgs uav_usv_interfaces uav_usv_ros1_compat
  visualization_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  rospy sensor_msgs std_msgs uav_usv_interfaces uav_usv_ros1_compat
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/fleet_gateway
  scripts/fleet_sensor_stream_adapter
  scripts/fleet_remote_uplink
  scripts/fleet_remote_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch web proto
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
