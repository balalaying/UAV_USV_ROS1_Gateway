cmake_minimum_required(VERSION 3.16)
project(uav_usv_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/gz_pointcloud_bridge.py
  scripts/fleet_gazebo_labels.py
  scripts/mid360_demo_motion.py
  scripts/mid360_preprocessor.py
  scripts/tf_topic_relay.py
  scripts/uav_camera_adapter.py
  scripts/uav_camera_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
