#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 BAG_DIRECTORY [RATE]" >&2
  exit 2
fi

BAG_DIRECTORY="$1"
RATE="${2:-1.0}"

if [[ ! -f "${BAG_DIRECTORY}/metadata.yaml" ]]; then
  echo "Not a rosbag2 directory: ${BAG_DIRECTORY}" >&2
  exit 2
fi

# Replay inputs only. Recorded LV-DOT outputs and old metrics are deliberately
# excluded so every replay measures the currently mounted detector parameters.
exec ros2 bag play "${BAG_DIRECTORY}" --rate "${RATE}" --topics \
  /fleet/uplink/uav_01/camera/image_raw \
  /fleet/uplink/uav_01/camera/camera_info \
  /perception/usv_01/points_filtered \
  /perception/lv_dot/uav_01/pose \
  /perception/lv_dot/usv_01/pose \
  /perception/ground_truth/tracks \
  /lv_dot/tuning/target_motion \
  /tf \
  /tf_static
