#!/usr/bin/env bash
set -euo pipefail

echo "Stopping heterogeneous_332 ROS and Gazebo processes..."
rosnode kill \
  /fleet_base_station_gui \
  /usv_01_qt_pointcloud_projection \
  /usv_01_mid360_preprocessor /usv_02_mid360_preprocessor /usv_03_mid360_preprocessor \
  /usv_01_mid360_gz_bridge /usv_02_mid360_gz_bridge /usv_03_mid360_gz_bridge \
  /usv_01_mid360_tf /usv_02_mid360_tf /usv_03_mid360_tf \
  /fleet_pose_tf_publisher \
  /base_station_service \
  /fleet_world_model \
  /fleet_base_station \
  /cooperative_algorithm_controller \
  /gz_entity_truth_bridge \
  /gz_sensor_bridge \
  /uav_01_agent /uav_02_agent /uav_03_agent \
  /usv_01_agent /usv_02_agent /usv_03_agent \
  /ardupilot_332_instances /heterogeneous_332_gazebo \
  2>/dev/null || true
pkill -TERM -f '[a]rducopter' 2>/dev/null || true
pkill -TERM -f '[a]rdurover' 2>/dev/null || true
pkill -TERM -f '[s]im_vehicle.py.*uav_usv_ardupilot_332' 2>/dev/null || true
pkill -TERM -f '[g]z sim' 2>/dev/null || true
pkill -TERM -f '[r]oslaunch.*heterogeneous_332_qt_ros1.launch' 2>/dev/null || true
pkill -TERM -f '[r]oslaunch.*gbsflacs_332_sim.launch' 2>/dev/null || true
pkill -TERM -f '[f]leet_base_station_gui.py' 2>/dev/null || true
pkill -TERM -f '[c]ooperative_algorithm_controller.py' 2>/dev/null || true
pkill -TERM -f '[u]av_fleet_agent.py' 2>/dev/null || true
pkill -TERM -f '[u]sv_gz_fleet_agent.py' 2>/dev/null || true
sleep 3
pkill -KILL -f '[a]rducopter' 2>/dev/null || true
pkill -KILL -f '[a]rdurover' 2>/dev/null || true
pkill -KILL -f '[g]z sim' 2>/dev/null || true
pkill -KILL -f '[r]oslaunch.*gbsflacs_332_sim.launch' 2>/dev/null || true
pkill -KILL -f '[f]leet_base_station_gui.py' 2>/dev/null || true
pkill -KILL -f '[c]ooperative_algorithm_controller.py' 2>/dev/null || true
pkill -KILL -f '[u]av_fleet_agent.py' 2>/dev/null || true
pkill -KILL -f '[u]sv_gz_fleet_agent.py' 2>/dev/null || true

echo "Stopped ArduPilot SITL, ROS 1 launch, and Gazebo processes. Logs were retained."
