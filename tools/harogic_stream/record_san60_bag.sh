#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
TARGET_MIB="${1:-500}"
OUTPUT_BASE="${2:-$SCRIPT_DIR/bags/san60_spectrum_$(date +%Y%m%d_%H%M%S)}"
TOPIC="/san60/spectrum"

if ! [[ "$TARGET_MIB" =~ ^[1-9][0-9]*$ ]]; then
  echo "target size must be a positive integer in MiB" >&2
  exit 2
fi

case "$OUTPUT_BASE" in
  *.bag) OUTPUT_BASE="${OUTPUT_BASE%.bag}" ;;
esac

mkdir -p "$(dirname "$OUTPUT_BASE")"
source /opt/ros/noetic/setup.bash
if [[ -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
  source "$WORKSPACE_DIR/devel/setup.bash"
fi

CORE_PID=""
PUBLISHER_PID=""
BAG_PID=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$PUBLISHER_PID" ]] && kill -0 "$PUBLISHER_PID" 2>/dev/null; then
    kill -INT "$PUBLISHER_PID" 2>/dev/null || true
    wait "$PUBLISHER_PID" 2>/dev/null || true
  fi
  if [[ -n "$CORE_PID" ]] && kill -0 "$CORE_PID" 2>/dev/null; then
    kill -INT "$CORE_PID" 2>/dev/null || true
    wait "$CORE_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

if ! rosnode list >/dev/null 2>&1; then
  roscore >"$OUTPUT_BASE.roscore.log" 2>&1 &
  CORE_PID=$!
  for _ in $(seq 1 50); do
    rosnode list >/dev/null 2>&1 && break
    sleep 0.2
  done
fi

python3 "$SCRIPT_DIR/ros_spectrum_publisher.py" _fps:=100.0 >"$OUTPUT_BASE.publisher.log" 2>&1 &
PUBLISHER_PID=$!

for _ in $(seq 1 100); do
  if rostopic type "$TOPIC" 2>/dev/null | grep -qx 'std_msgs/String'; then
    break
  fi
  if ! kill -0 "$PUBLISHER_PID" 2>/dev/null; then
    echo "SAN-60 publisher exited before advertising $TOPIC" >&2
    exit 1
  fi
  sleep 0.1
done

if ! rostopic type "$TOPIC" 2>/dev/null | grep -qx 'std_msgs/String'; then
  echo "topic $TOPIC was not advertised" >&2
  exit 1
fi

rosbag record --buffsize=256 -O "$OUTPUT_BASE" "$TOPIC" >"$OUTPUT_BASE.rosbag.log" 2>&1 &
BAG_PID=$!
ACTIVE_BAG="$OUTPUT_BASE.bag.active"
FINAL_BAG="$OUTPUT_BASE.bag"
TARGET_BYTES=$((TARGET_MIB * 1024 * 1024))

echo "Recording $TOPIC to $FINAL_BAG"
echo "Target size: approximately $TARGET_MIB MiB"

while kill -0 "$BAG_PID" 2>/dev/null; do
  CURRENT_BYTES=0
  if [[ -f "$ACTIVE_BAG" ]]; then
    CURRENT_BYTES=$(stat -c %s "$ACTIVE_BAG")
  fi
  printf '\rRecorded: %d / %d MiB' "$((CURRENT_BYTES / 1024 / 1024))" "$TARGET_MIB"
  if (( CURRENT_BYTES >= TARGET_BYTES )); then
    break
  fi
  sleep 2
done
printf '\n'

if kill -0 "$BAG_PID" 2>/dev/null; then
  kill -INT "$BAG_PID"
fi
wait "$BAG_PID" || true
BAG_PID=""

if [[ ! -f "$FINAL_BAG" ]]; then
  echo "rosbag did not produce $FINAL_BAG" >&2
  exit 1
fi

kill -INT "$PUBLISHER_PID" 2>/dev/null || true
wait "$PUBLISHER_PID" 2>/dev/null || true
PUBLISHER_PID=""

echo "Finished: $FINAL_BAG"
du -h "$FINAL_BAG"
rosbag info "$FINAL_BAG"
