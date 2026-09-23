#!/usr/bin/env bash
set -euo pipefail

live_topic() {
  local topic="$1"
  if timeout 6 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "LIVE ${topic}"
  else
    echo "MISS ${topic}"
    return 1
  fi
}

echo "===== 核心节点 ====="
rosnode list | grep -E '^/(fleet|base_station|gz_|uav_0|usv_0)|mid360|pointcloud' | sort || true

echo
echo "===== 核心数据流（收到真实消息才算 LIVE） ====="
status=0
for topic in \
  /fleet/state \
  /fleet/base/camera_mosaic \
  /fleet/base/radar/scan \
  /fleet/world_model \
  /base_station/state; do
  live_topic "${topic}" || status=1
done
for id in usv_01 usv_02 usv_03; do
  live_topic "/fleet/uplink/${id}/mid360/points" || status=1
  live_topic "/perception/${id}/mid360/points_filtered" || status=1
done
live_topic /perception/visualization/usv_01/topdown_points || status=1

echo
echo "===== Mid-360 点云摘要 ====="
python3 - <<'PY'
import rospy
from sensor_msgs.msg import PointCloud2
rospy.init_node('check_mid360_once', anonymous=True, disable_signals=True)
for vehicle_id in ('usv_01', 'usv_02', 'usv_03'):
    topic = '/fleet/uplink/%s/mid360/points' % vehicle_id
    try:
        msg = rospy.wait_for_message(topic, PointCloud2, timeout=5.0)
        print('%s: points=%d frame=%s fields=%s' % (
            vehicle_id, msg.width * msg.height, msg.header.frame_id,
            ','.join(field.name for field in msg.fields)))
    except Exception as error:
        print('%s: NO DATA (%s)' % (vehicle_id, error))
PY

echo
echo "===== 舰队摘要 ====="
python3 - <<'PY'
import json
import rospy
from std_msgs.msg import String
rospy.init_node('check_fleet_once', anonymous=True, disable_signals=True)
try:
    msg = rospy.wait_for_message('/base_station/state', String, timeout=5.0)
    data = json.loads(msg.data)
    fleet = data.get('fleet', {})
    uav = fleet.get('uav', [])
    usv = fleet.get('usv', [])
    online = [item.get('id') for item in uav + usv if item.get('online')]
    print('UAV=%d USV=%d online=%d ids=%s' % (
        len(uav), len(usv), len(online), ','.join(online)))
except Exception as error:
    print('NO BASE STATE:', error)
PY

echo
echo "===== 最新仓库模型策略 ====="
gazebo_share="$(rospack find uav_usv_gazebo)"
repo_root="$(cd "${gazebo_share}/../.." && pwd)"
"${gazebo_share}/tools/verify_repo_models.py" \
  --world "${gazebo_share}/worlds/heterogeneous_332.sdf" \
  --model-root "${gazebo_share}/models"

echo
echo "===== ArduPilot SITL ====="
ardupilot_root="${ARDUPILOT_DIR:-${repo_root}/third_party/ardupilot}"
for path in \
  "${ardupilot_root}/build/sitl/bin/arducopter" \
  "${ardupilot_root}/build/sitl/bin/ardurover" \
  "${repo_root}/devel/lib/uav_usv_gazebo/plugins/libArduPilotPlugin.so" \
  "${repo_root}/devel/lib/uav_usv_gazebo/plugins/libArduPilotMotorBridge.so" \
  "${repo_root}/devel/lib/uav_usv_gazebo/plugins/libArduPilotRoverBridge.so"; do
  [[ -f "${path}" ]] && echo "OK   ${path}" || { echo "MISS ${path}"; status=1; }
done
pgrep -af 'arducopter|ardurover' || { echo "MISS running ArduPilot SITL"; status=1; }

echo
echo "===== RGL / 磁盘保护 ====="
rgl_root="${UAV_USV_RGL_ROOT:-/var/tmp/RGLGazeboPlugin_v0.2.0_focal_custom}"
for path in \
  "${rgl_root}/install/RGLServerPlugin/libRGLServerPluginInstance.so" \
  "${rgl_root}/install/RGLServerPlugin/libRGLServerPluginManager.so" \
  "${rgl_root}/lidar_patterns/LivoxMid360.mat3x4f"; do
  [[ -f "${path}" ]] && echo "OK   ${path}" || echo "MISS ${path}"
done
log_bytes="$(find /tmp -maxdepth 1 -type f -name 'uav_usv_ardupilot_*.log' -printf '%s\n' 2>/dev/null | awk '{sum += $1} END {print sum + 0}')"
echo "ArduPilot launcher log bytes: ${log_bytes}"
df -h / | tail -n 1

exit "${status}"
