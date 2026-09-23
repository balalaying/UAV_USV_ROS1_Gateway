#!/usr/bin/env bash
set -uo pipefail

echo "===== 双算法节点 ====="
if rosnode list 2>/dev/null | grep -qx '/cooperative_algorithm_controller'; then
  echo "OK /cooperative_algorithm_controller"
else
  echo "MISS /cooperative_algorithm_controller"
fi

echo "===== 输入接口 ====="
for topic in /fleet/state /fleet/world_model /fleet/control_lease; do
  if rostopic type "${topic}" >/dev/null 2>&1; then
    echo "OK ${topic} ($(rostopic type "${topic}"))"
  else
    echo "MISS ${topic}"
  fi
done

echo "===== 输出接口 ====="
for topic in /fleet/algorithm/status /fleet/algorithm/assignments /fleet/algorithm/markers; do
  if rostopic type "${topic}" >/dev/null 2>&1; then
    echo "OK ${topic} ($(rostopic type "${topic}"))"
  else
    echo "MISS ${topic}"
  fi
done

echo "===== 当前算法状态 ====="
python3 -c 'import json,rospy; from std_msgs.msg import String; rospy.init_node("algorithm_check",anonymous=True,disable_signals=True); data=json.loads(rospy.wait_for_message("/fleet/algorithm/status",String,timeout=4).data); print("mode=%s active=%s phase=%s vehicles=%s/%s error=%s" % (data.get("mode"),data.get("active"),data.get("phase"),data.get("vehicle_count"),data.get("required_vehicle_count"),data.get("last_error") or "none"))' 2>/dev/null \
  || echo "状态消息暂不可用"
