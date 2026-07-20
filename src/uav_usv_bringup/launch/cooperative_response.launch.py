import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _script(package, source_name, installed_name):
    source_root = os.path.abspath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', '..'
    ))
    source = os.path.join(source_root, package, 'scripts', source_name)
    if os.path.exists(source):
        return source
    return os.path.abspath(os.path.join(
        get_package_share_directory(package), '..', '..', 'lib', package,
        installed_name,
    ))


def generate_launch_description():
    bringup = get_package_share_directory('uav_usv_bringup')
    gui_config = os.path.join(bringup, 'config', 'gazebo_white_gui.config')
    rviz_config = os.path.join(bringup, 'rviz', 'defense.rviz')
    base_station = _script(
        'uav_usv_mission', 'fleet_base_station.py', 'fleet_base_station'
    )
    sensor_agent = _script(
        'uav_usv_mission', 'fleet_simulated_agent.py', 'fleet_simulated_agent'
    )
    mission = _script(
        'uav_usv_mission', 'cooperative_response_mission.py',
        'cooperative_response_mission'
    )
    dds_agent = _script(
        'uav_usv_uav_control', 'uav_dds_fleet_agent.py',
        'uav_dds_fleet_agent'
    )
    qt_gui = _script(
        'uav_usv_mission', 'fleet_base_station_gui.py',
        'fleet_base_station_gui'
    )
    ids = 'uav_01,uav_02,uav_03,uav_04'
    usv_ids = 'usv_01,usv_02,usv_03,usv_04'
    vehicles = (
        'usv_01:usv:own_01:-260:-230:1.1;'
        'usv_02:usv:own_02:-250:-160:1.1;'
        'usv_03:usv:own_03:-250:-80:1.1;'
        'usv_04:usv:own_04:-260:-10:1.1;'
        ''
    )
    px4_models = os.path.expanduser(
        os.environ.get(
            'PX4_GZ_MODEL_PATH',
            '~/PX4-Autopilot/Tools/simulation/gz/models',
        )
    )
    px4_dir = os.path.expanduser(
        os.environ.get('PX4_DIR', '~/PX4-Autopilot')
    )
    px4_ros_ws = os.path.expanduser(
        os.environ.get('PX4_ROS_WS', '~/Desktop/Px4_ros')
    )
    px4_plugins = os.path.join(
        px4_dir, 'build', 'px4_sitl_default', 'src', 'modules',
        'simulation', 'gz_plugins',
    )
    px4_prepare_cmd = (
        'ros2 run uav_usv_sim prepare_large_x500.py '
        '--px4-dir "%s" --scale 1.0 '
        '--camera-width 320 --camera-height 180 --camera-rate 20.0'
    ) % px4_dir
    px4_poses = (
        '-131,-234,6.2,0,0,0',
        '-109,-234,6.2,0,0,0',
        '-131,-216,6.2,0,0,0',
        '-109,-216,6.2,0,0,0',
    )
    px4_binary = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_etc = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'etc')
    px4_instances = []
    for index, pose in enumerate(px4_poses):
        instance_dir = os.path.join(
            px4_dir, 'build', 'px4_sitl_default', 'instance_%d' % index
        )
        px4_instances.append(
            'mkdir -p "{work}"; cd "{work}"; '
            'PX4_SIM_MODEL=gz_x500_mono_cam_down '
            'PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=cooperative_response_sim '
            'PX4_UXRCE_DDS_NS="uav_{vehicle:02d}" '
            'PX4_GZ_MODEL_POSE="{pose}" '
            '"{binary}" -i {index} -d "{etc}"'
            .format(
                work=instance_dir, pose=pose, binary=px4_binary,
                index=index, vehicle=index + 1, etc=px4_etc,
            )
        )
    px4_cmd = (
        px4_prepare_cmd + ' && { ('
        + ') & ('.join(px4_instances)
        + ') & wait; }'
    )
    gazebo_cmd = (
        'ros2 run uav_usv_sim prepare_large_x500.py '
        '--px4-dir "%s" --scale 1.0 '
        '--camera-width 320 --camera-height 180 --camera-rate 20.0 && '
        'export GZ_SIM_RESOURCE_PATH="%s:${GZ_SIM_RESOURCE_PATH:-}"; '
        'export GZ_SIM_SYSTEM_PLUGIN_PATH="%s:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"; '
        'export GZ_SIM_ARGS="-r --gui-config %s"; '
        'ros2 run uav_usv_gazebo run_gz_world.sh cooperative_response_sim'
    ) % (px4_dir, px4_models, px4_plugins, gui_config)

    return LaunchDescription([
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_qt', default_value='true'),
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('start_dds', default_value='true'),
        ExecuteProcess(
            cmd=['bash', '-c', gazebo_cmd], output='screen',
            condition=IfCondition(LaunchConfiguration('start_gazebo')),
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='cooperative_map_tf', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        ),
        ExecuteProcess(
            cmd=[
                'rviz2', '-d', rviz_config, '--ros-args',
                '-r', '__node:=cooperative_response_rviz',
                '-r', '/defense/rviz_markers:=/mission/markers',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
        ExecuteProcess(cmd=[
            'python3', mission, '--ros-args',
            '-p', 'world_name:=cooperative_response_sim',
            '-p', 'trigger_radius:=230.0', '-p', 'defend_radius:=90.0',
            '-p', 'base_avoid_radius:=62.0',
            '-p', 'px4_uav_enabled:=true',
            '-p', 'px4_uav_model:=x500_mono_cam_down_0',
            '-p', 'gazebo_tactical_markers:=false',
        ], output='screen'),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_dds')),
        ),
        ExecuteProcess(cmd=[
            'python3', base_station, '--ros-args',
            '-r', '__node:=cooperative_base_station',
            '-p', ['uav_ids:=', ids], '-p', ['usv_ids:=', usv_ids],
            '-p', 'auto_demo:=false',
        ], output='screen'),
        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=['bash', '-c', px4_cmd], output='screen',
                condition=IfCondition(LaunchConfiguration('start_px4')),
            )],
        ),
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source "%s/install/setup.bash" && python3 "%s"'
                % (px4_ros_ws, dds_agent),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_dds')),
        ),
        ExecuteProcess(cmd=[
            'python3', sensor_agent, '--ros-args',
            '-r', '__node:=cooperative_sensor_agent',
            '-p', ['vehicles:=', vehicles],
            '-p', 'pose_topic:=/world/cooperative_response_sim/pose/info',
            '-p', 'image_rate:=20.0', '-p', 'scan_rate:=10.0',
            '-p', 'publish_images:=false',
        ], output='screen'),
        Node(
            package='uav_usv_mission', executable='gz_sensor_bridge',
            name='cooperative_gz_sensor_bridge', output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'python3', qt_gui, '--ros-args',
                '-r', '__node:=cooperative_base_station_qt',
                '-p', 'defense_node_name:=/cooperative_response_mission',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_qt')),
        ),
    ])
