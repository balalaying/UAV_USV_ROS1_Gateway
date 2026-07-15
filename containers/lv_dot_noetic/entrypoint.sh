#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/lv_dot_ws/devel/setup.bash
exec "$@"
