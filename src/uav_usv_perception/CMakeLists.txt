cmake_minimum_required(VERSION 3.16)
project(uav_usv_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
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

install(PROGRAMS
  scripts/adapters/ground_truth_adapter.py
  scripts/adapters/lv_dot_adapter.py
  scripts/adapters/lv_dot_egress_relay.py
  scripts/adapters/lv_dot_ingress_relay.py
  scripts/adapters/lv_dot_observation_adapter.py
  scripts/adapters/lv_dot_pose_adapter.py
  scripts/evaluation/lv_dot_fusion_evaluator.py
  scripts/evaluation/lv_dot_shadow_evaluator.py
  scripts/fusion/perception_fusion_node.py
  scripts/fusion/perception_source_mux.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/tracking/track_association.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY docs launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_track_association
    test/test_track_association.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts/tracking
  )
  ament_add_pytest_test(
    test_lv_dot_tuning_helpers
    test/test_lv_dot_tuning_helpers.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(
    test_lv_dot_phase5_adapter
    test/test_lv_dot_phase5_adapter.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts/adapters
  )
endif()

ament_package()
