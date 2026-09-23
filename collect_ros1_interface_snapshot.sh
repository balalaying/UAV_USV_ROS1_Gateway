#!/usr/bin/env bash

desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null)"
[ -n "$desktop_dir" ] || desktop_dir="$HOME/Desktop"

snapshot_dir="$desktop_dir/ROS1_INTERFACE_SNAPSHOT_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$snapshot_dir"

echo "===== 开始采集 ROS1 接口快照 ====="
echo "输出目录: $snapshot_dir"

{
    echo "recorded_at=$(date -Is)"
    echo "hostname=$(hostname)"
    echo "user=$(whoami)"
    echo "pwd=$(pwd)"
    echo "ROS_DISTRO=${ROS_DISTRO:-UNKNOWN}"

    echo
    echo "===== ROS环境变量 ====="
    env | grep '^ROS_' | sort

    echo
    echo "===== 工作空间 ====="
    echo "HOME=$HOME"
    echo "current_directory=$(pwd)"
} > "$snapshot_dir/system_and_ros_env.txt"

rosnode list | sort \
    > "$snapshot_dir/rosnode_list.txt" 2>&1

rostopic list -v \
    > "$snapshot_dir/rostopic_list_verbose.txt" 2>&1

rostopic list \
    > "$snapshot_dir/rostopic_list.txt" 2>&1

{
    for topic in $(rostopic list 2>/dev/null); do
        type=$(rostopic type "$topic" 2>/dev/null)
        echo "$topic    $type"
    done
} > "$snapshot_dir/topic_types.txt"

rosservice list | sort \
    > "$snapshot_dir/rosservice_list.txt" 2>&1

{
    for srv in $(rosservice list 2>/dev/null); do
        type=$(rosservice type "$srv" 2>/dev/null)
        echo "$srv    $type"
    done
} > "$snapshot_dir/service_types.txt"

rosparam list | sort \
    > "$snapshot_dir/rosparam_list.txt" 2>&1

ps -eo pid,args | grep -E \
'roscore|rosmaster|roslaunch|rosrun|sim_vehicle|arducopter|ardurover|mavros|gazebo|gzserver|gzclient' | \
grep -v grep \
    > "$snapshot_dir/relevant_processes.txt"

{
    echo "===== ROS_PACKAGE_PATH ====="
    echo "$ROS_PACKAGE_PATH"

    echo
    echo "===== 相关 ROS 包 ====="

    rospack list 2>/dev/null | grep -Ei \
    'uav|usv|mavros|ardupilot|gazebo|control|planner|mission|fleet|agent|gateway'
} > "$snapshot_dir/relevant_ros_packages.txt"

mkdir -p "$snapshot_dir/node_info"

for node in $(rosnode list 2>/dev/null); do
    safe_name=$(echo "$node" | sed 's#/#_#g')

    rosnode info "$node" \
        > "$snapshot_dir/node_info/${safe_name}.txt" 2>&1
done

{
    echo "===== MESSAGE TYPES ====="

    for topic in $(rostopic list 2>/dev/null); do
        rostopic type "$topic" 2>/dev/null
    done | sort -u

    echo
    echo "===== SERVICE TYPES ====="

    for srv in $(rosservice list 2>/dev/null); do
        rosservice type "$srv" 2>/dev/null
    done | sort -u
} > "$snapshot_dir/interface_type_summary.txt"

echo
echo "===== ROS1 接口快照完成 ====="
ls -lh "$snapshot_dir"

echo
echo "SNAPSHOT_DIRECTORY=$snapshot_dir"
