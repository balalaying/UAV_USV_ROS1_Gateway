#!/usr/bin/env bash
set -euo pipefail

GAZEBO_SHARE_DIR="$(rospack find uav_usv_gazebo)"
BRINGUP_SHARE_DIR="$(rospack find uav_usv_bringup)"
REPO_ROOT="$(cd "${GAZEBO_SHARE_DIR}/../.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${REPO_ROOT}/third_party/ardupilot}"
SIM_VEHICLE="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
COPTER_BIN="${ARDUPILOT_DIR}/build/sitl/bin/arducopter"
ROVER_BIN="${ARDUPILOT_DIR}/build/sitl/bin/ardurover"
RUNTIME_ROOT="${UAV_USV_ARDUPILOT_RUNTIME_ROOT:-/tmp/uav_usv_ardupilot_332}"

for required in "${SIM_VEHICLE}" "${COPTER_BIN}" "${ROVER_BIN}"; do
  if [[ ! -x "${required}" ]]; then
    echo "ArduPilot SITL component missing: ${required}" >&2
    echo "Build it with: cd ${ARDUPILOT_DIR} && ./waf configure --board sitl && ./waf copter rover" >&2
    exit 2
  fi
done

echo "Waiting for Gazebo world heterogeneous_332..."
world_ready=0
gz_has_service_command=0
if gz --commands 2>/dev/null | grep -qx "service"; then
  gz_has_service_command=1
fi
for _attempt in $(seq 1 180); do
  if [[ "${gz_has_service_command}" -eq 1 ]]; then
    if gz service -l 2>/dev/null \
      | grep -qx "/world/heterogeneous_332/control"; then
      world_ready=1
      break
    fi
  else
    # Some source-built Gazebo Harmonic installations expose only the
    # ``model`` and ``sim`` CLI verbs even though transport itself is fully
    # functional.  Waiting exclusively on ``gz service`` would therefore
    # time out and prevent every SITL instance from starting.  A successful
    # model query proves that the world transport endpoint is responsive;
    # require all six controlled models so their ArduPilot plugins are ready.
    model_list="$(timeout 5 gz model --list 2>/dev/null || true)"
    model_count=0
    for model_name in uav_01 uav_02 uav_03 usv_01 usv_02 usv_03; do
      if grep -Eq "^[[:space:]]*-[[:space:]]+${model_name}[[:space:]]*$" \
        <<<"${model_list}"; then
        model_count=$((model_count + 1))
      fi
    done
    if [[ "${model_count}" -eq 6 ]]; then
      world_ready=1
      break
    fi
  fi
  sleep 1
done
if [[ "${world_ready}" -ne 1 ]]; then
  if [[ "${gz_has_service_command}" -eq 1 ]]; then
    echo "Gazebo world service did not appear within 180 seconds" >&2
  else
    echo "Gazebo did not report all six controlled models within 180 seconds" >&2
  fi
  exit 3
fi

mkdir -p "${RUNTIME_ROOT}"
PROCESS_GROUP_FILE="${RUNTIME_ROOT}/.sitl_process_groups"

# Recover from an interrupted roslaunch (terminal close, power loss, or
# SIGKILL).  sim_vehicle may have exited while its terminal helper and SITL
# child remain alive, so retain and clean the session/process-group ids.
if [[ -f "${PROCESS_GROUP_FILE}" ]]; then
  while IFS= read -r stale_pgid; do
    [[ "${stale_pgid}" =~ ^[0-9]+$ ]] || continue
    stale_args="$(ps -eo pgid=,args= | awk -v pgid="${stale_pgid}" '$1 == pgid { $1=""; print }')"
    if grep -Eq 'uav_usv_ardupilot_332|build/sitl/bin/ardu(copter|rover)' \
      <<<"${stale_args}"; then
      echo "Stopping stale ArduPilot SITL process group ${stale_pgid}"
      kill -TERM -- "-${stale_pgid}" 2>/dev/null || true
      sleep 0.2
      kill -KILL -- "-${stale_pgid}" 2>/dev/null || true
    fi
  done <"${PROCESS_GROUP_FILE}"
fi
: >"${PROCESS_GROUP_FILE}"

pids=()
cleanup() {
  trap - EXIT INT TERM
  local pid
  for pid in "${pids[@]:-}"; do
    # Each sim_vehicle instance runs in its own session.  Killing the process
    # group also reaches the terminal helper and SITL binary after
    # sim_vehicle.py has handed them off (they may already be re-parented).
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  sleep 0.5
  for pid in "${pids[@]:-}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  : >"${PROCESS_GROUP_FILE}"
}
trap cleanup EXIT INT TERM

start_instance() {
  local instance="$1"
  local vehicle="$2"
  local frame="$3"
  local identity="$4"
  local vehicle_dir="${RUNTIME_ROOT}/${identity}"
  local terminal_tmp_dir="${vehicle_dir}/terminal_tmp"
  local log_file="/tmp/uav_usv_ardupilot_${identity}.log"
  local -a vehicle_options=()
  if [[ "${vehicle}" == "Rover" ]]; then
    vehicle_options+=(
      "--add-param-file=${BRINGUP_SHARE_DIR}/config/rover_10ms.parm"
    )
  fi
  mkdir -p "${vehicle_dir}" "${terminal_tmp_dir}"
  (
    cd "${ARDUPILOT_DIR}"
    # ArduPilot's terminal helper names its temporary launcher with only
    # second-level precision. Give every concurrent SITL instance a private
    # TMPDIR so their launcher files cannot overwrite or concatenate each
    # other. Use bash in-process by default to avoid opening six terminals.
    export TMPDIR="${terminal_tmp_dir}"
    export SITL_RITW_TERMINAL="${SITL_RITW_TERMINAL:-bash}"
    exec setsid "${SIM_VEHICLE}" -N --no-mavproxy --auto-sysid \
      -I "${instance}" -v "${vehicle}" -f "${frame}" --model JSON \
      --use-dir "${vehicle_dir}" "${vehicle_options[@]}"
  ) >"${log_file}" 2>&1 &
  pids+=("$!")
  echo "$!" >>"${PROCESS_GROUP_FILE}"
  echo "${identity}: ${vehicle}/${frame}, JSON-FDM=$((9002 + instance * 10)), MAVLink TCP=$((5760 + instance * 10)), log=${log_file}"
}

echo "Gazebo ready; starting 3 ArduCopter and 3 ArduRover instances"
start_instance 0 ArduCopter gazebo-iris uav_01
start_instance 1 ArduCopter gazebo-iris uav_02
start_instance 2 ArduCopter gazebo-iris uav_03
start_instance 3 Rover gazebo-rover usv_01
start_instance 4 Rover gazebo-rover usv_02
start_instance 5 Rover gazebo-rover usv_03
wait
