#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./tools/setup_px4.sh [options]

Options:
  --dir PATH                 PX4 target directory.
                             Default: third_party/PX4-Autopilot
  --repo URL                 PX4 git repository.
                             Default: https://github.com/PX4/PX4-Autopilot.git
  --branch NAME              Checkout a PX4 branch or tag after cloning.
                             Default: leave repository default branch.
  --install-system-deps      Run PX4 Tools/setup/ubuntu.sh after clone/update.
                             This may ask for sudo password.
  --skip-submodules          Do not update PX4 git submodules.
  -h, --help                 Show this help.

Examples:
  ./tools/setup_px4.sh
  ./tools/setup_px4.sh --branch release/1.15
  ./tools/setup_px4.sh --install-system-deps

After setup:
  export PX4_DIR="$(pwd)/third_party/PX4-Autopilot"
  ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py px4_dir:="$PX4_DIR"
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${WORKSPACE_DIR}/third_party/PX4-Autopilot"
PX4_REPO="https://github.com/PX4/PX4-Autopilot.git"
PX4_BRANCH=""
INSTALL_SYSTEM_DEPS=0
UPDATE_SUBMODULES=1

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dir)
      PX4_DIR="$(realpath -m "$2")"
      shift 2
      ;;
    --repo)
      PX4_REPO="$2"
      shift 2
      ;;
    --branch)
      PX4_BRANCH="$2"
      shift 2
      ;;
    --install-system-deps)
      INSTALL_SYSTEM_DEPS=1
      shift
      ;;
    --skip-submodules)
      UPDATE_SUBMODULES=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

need_cmd git
need_cmd realpath

mkdir -p "$(dirname "${PX4_DIR}")"

if [ -d "${PX4_DIR}/.git" ]; then
  echo "PX4 repository already exists: ${PX4_DIR}"
  git -C "${PX4_DIR}" fetch --tags --prune
else
  echo "Cloning PX4 into: ${PX4_DIR}"
  git clone "${PX4_REPO}" "${PX4_DIR}"
fi

if [ -n "${PX4_BRANCH}" ]; then
  echo "Checking out PX4 branch/tag: ${PX4_BRANCH}"
  git -C "${PX4_DIR}" checkout "${PX4_BRANCH}"
fi

if [ "${UPDATE_SUBMODULES}" -eq 1 ]; then
  echo "Updating PX4 submodules..."
  git -C "${PX4_DIR}" submodule update --init --recursive
fi

if [ "${INSTALL_SYSTEM_DEPS}" -eq 1 ]; then
  echo "Running PX4 Ubuntu dependency setup. This may ask for sudo password."
  bash "${PX4_DIR}/Tools/setup/ubuntu.sh"
else
  cat <<EOF

PX4 source is ready.

System dependencies were not installed automatically.
If this machine has never built PX4 before, run:
  ./tools/setup_px4.sh --install-system-deps

Use this PX4 path for the current shell:
  export PX4_DIR="${PX4_DIR}"

Then launch the existing simulation exactly as before:
  ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py px4_dir:="\$PX4_DIR"

EOF
fi
