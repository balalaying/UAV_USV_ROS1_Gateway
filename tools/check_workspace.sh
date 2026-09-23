#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPOSITORY_ROOT}/../.." && pwd)"

set +u
source /opt/ros/noetic/setup.bash
set -u

find "${REPOSITORY_ROOT}/src" -type f -name '*.py' -print0 \
  | xargs -0 -r python3 -m py_compile

find "${REPOSITORY_ROOT}/src" -type f \
  \( -name 'package.xml' -o -name '*.sdf' -o -name '*.urdf' -o -name '*.xacro' \) \
  -print0 | xargs -0 -r xmllint --noout

cd "${WORKSPACE_ROOT}"
catkin_make
catkin_make run_tests
catkin_test_results --verbose "${WORKSPACE_ROOT}/build/test_results"

echo "Workspace checks passed."
