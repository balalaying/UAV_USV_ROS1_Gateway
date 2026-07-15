#!/usr/bin/env bash
set -euo pipefail

OUTPUT="${1:-/tmp/uav_usv_bags/lv_dot_$(date +%Y%m%d_%H%M%S)}"

if [[ -e "${OUTPUT}" ]]; then
  echo "Bag output already exists: ${OUTPUT}" >&2
  exit 2
fi

mkdir -p "$(dirname "${OUTPUT}")"

exec ros2 bag record --storage sqlite3 --output "${OUTPUT}" \
  /fleet/uplink/uav_01/camera/image_raw \
  /fleet/uplink/uav_01/camera/camera_info \
  /fleet/uplink/usv_01/mid360/points \
  /perception/usv_01/points_filtered \
  /perception/lv_dot/uav_01/pose \
  /perception/lv_dot/usv_01/pose \
  /perception/ground_truth/tracks \
  /perception/lv_dot/observations \
  /perception/lv_dot/shadow_metrics \
  /lv_dot/onboard_detector/dynamic_bboxes \
  /lv_dot/onboard_detector/velocity_visualizaton \
  /lv_dot/diagnostics/lidar_bboxes \
  /lv_dot/diagnostics/filtered_bboxes \
  /lv_dot/diagnostics/tracked_bboxes \
  /lv_dot/tuning/target_motion \
  /fleet/sensor_status \
  /tf \
  /tf_static
