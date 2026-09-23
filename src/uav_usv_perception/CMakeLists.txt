cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_perception)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  rospy
  rosgraph_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_msgs
  uav_usv_interfaces
  uav_usv_ros1_compat
  vision_msgs
  visualization_msgs
)

catkin_package(CATKIN_DEPENDS
  geographic_msgs geometry_msgs rospy rosgraph_msgs sensor_msgs std_msgs
  std_srvs tf2_ros tf2_msgs uav_usv_interfaces uav_usv_ros1_compat
  vision_msgs visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/gz_pointcloud_bridge.py
  scripts/fleet_gazebo_labels.py
  scripts/mid360_demo_motion.py
  scripts/mid360_preprocessor.py
  scripts/lv_dot_target_motion.py
  scripts/tf_topic_relay.py
  scripts/uav_camera_adapter.py
  scripts/uav_camera_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  scripts/adapters/ground_truth_adapter.py
  scripts/adapters/lv_dot_observation_adapter.py
  scripts/adapters/uav_visual_observation_node.py
  scripts/adapters/usv_camera_detection_node.py
  scripts/evaluation/lv_dot_fusion_evaluator.py
  scripts/evaluation/multisensor_validation_evaluator.py
  scripts/fusion/perception_fusion_node.py
  scripts/fusion/perception_source_mux.py
  scripts/fusion/camera_lidar_association_node.py
  scripts/fusion/vision_guided_lidar_roi_node.py
  scripts/visualization/qt_pointcloud_projection_node.py
  scripts/visualization/lv_dot_debug_visualization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/tracking/track_association.py
  scripts/vision_guided/vision_guided_core.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config docs launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
