import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _script(source_root, package, script_name, installed_name):
    source = os.path.join(source_root, package, 'scripts', script_name)
    if os.path.exists(source):
        return source
    installed = os.path.join(
        get_package_share_directory(package),
        '..',
        '..',
        'lib',
        package,
        installed_name,
    )
    return os.path.abspath(installed)


def _camera_bridges(_context):
    bridges = []
    usv_pairs = [
        ('own_01', 'usv_01'),
        ('own_02', 'usv_02'),
        ('own_03', 'usv_03'),
        ('own_04', 'usv_04'),
    ]
    for model_name, vehicle_id in usv_pairs:
        gz_topic = '/defense/%s/front_camera' % model_name
        bridges.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='%s_capture_camera_bridge' % vehicle_id,
                output='screen',
                arguments=[
                    gz_topic + '@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
                remappings=[
                    (gz_topic, '/fleet/uplink/%s/camera' % vehicle_id),
                ],
            )
        )
    for index in range(1, 5):
        vehicle_id = 'uav_%02d' % index
        topic = '/fleet/uplink/%s/camera' % vehicle_id
        bridges.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='%s_capture_camera_bridge' % vehicle_id,
                output='screen',
                arguments=[
                    topic + '@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
            )
        )
    return bridges


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_usv_bringup')
    source_package_dir = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
    )
    source_root = os.path.abspath(os.path.join(source_package_dir, '..'))
    rviz_config = os.path.join(source_package_dir, 'rviz', 'capture.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(bringup_share, 'rviz', 'capture.rviz')
    gazebo_gui_config = os.path.join(
        source_package_dir,
        'config',
        'gazebo_white_gui.config',
    )
    if not os.path.exists(gazebo_gui_config):
        gazebo_gui_config = os.path.join(
            bringup_share,
            'config',
            'gazebo_white_gui.config',
        )

    mission_package = 'uav_usv_mission'
    base_station_script = _script(
        source_root, mission_package, 'fleet_base_station.py', 'fleet_base_station'
    )
    sim_agent_script = _script(
        source_root, mission_package, 'fleet_simulated_agent.py', 'fleet_simulated_agent'
    )
    qt_script = _script(
        source_root, mission_package, 'fleet_base_station_gui.py', 'fleet_base_station_gui'
    )
    capture_script = _script(
        source_root, mission_package, 'capture_mission.py', 'capture_mission'
    )

    uav_ids = 'uav_01,uav_02,uav_03,uav_04'
    usv_ids = 'usv_01,usv_02,usv_03,usv_04'
    vehicles = (
        'usv_01:usv:own_01:-190:-162:0.6;'
        'usv_02:usv:own_02:-186:-134:0.6;'
        'usv_03:usv:own_03:-186:-104:0.6;'
        'usv_04:usv:own_04:-190:-76:0.6;'
        'uav_01:uav:uav_01:-120:-120:2.9;'
        'uav_02:uav:uav_02:-120:-120:6.4;'
        'uav_03:uav:uav_03:-120:-120:9.9;'
        'uav_04:uav:uav_04:-120:-120:13.4'
    )
    px4_dir = os.path.expanduser(
        os.environ.get('PX4_DIR', '~/PX4-Autopilot')
    )
    px4_models = os.path.join(
        px4_dir, 'Tools', 'simulation', 'gz', 'models'
    )
    gazebo_cmd = (
        'export GZ_SIM_RESOURCE_PATH="%s:${GZ_SIM_RESOURCE_PATH:-}"; '
        'export GZ_SIM_ARGS="-r --gui-config %s"; '
        'ros2 run uav_usv_gazebo run_gz_world.sh capture_ocean_sim'
    ) % (px4_models, gazebo_gui_config)

    return LaunchDescription(
        [
            DeclareLaunchArgument('start_gazebo', default_value='true'),
            DeclareLaunchArgument('start_rviz', default_value='true'),
            DeclareLaunchArgument('start_qt', default_value='true'),
            DeclareLaunchArgument('start_base_station', default_value='true'),
            DeclareLaunchArgument('start_sensor_agent', default_value='true'),
            DeclareLaunchArgument(
                'start_camera_bridge',
                default_value='true',
                description=(
                    'Enable only when Gazebo camera topics are available. '
                    'The standalone demo uses the fleet sensor agent.'
                ),
            ),
            DeclareLaunchArgument('start_capture_mission', default_value='true'),
            DeclareLaunchArgument(
                'start_px4',
                default_value='false',
                description=(
                    'Reserved PX4 hook for uav_01. Keep false for the current '
                    'standalone capture Gazebo world.'
                ),
            ),
            ExecuteProcess(
                cmd=['bash', '-c', gazebo_cmd],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_gazebo')),
            ),
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config, '--ros-args', '-r', '__node:=capture_rviz'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    base_station_script,
                    '--ros-args',
                    '-r',
                    '__node:=capture_base_station',
                    '-p',
                    ['uav_ids:=', uav_ids],
                    '-p',
                    ['usv_ids:=', usv_ids],
                    '-p',
                    'uav_id:=uav_01',
                    '-p',
                    'usv_id:=usv_01',
                    '-p',
                    'auto_demo:=false',
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_base_station')),
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    sim_agent_script,
                    '--ros-args',
                    '-r',
                    '__node:=capture_sensor_agent',
                    '-p',
                    ['vehicles:=', vehicles],
                    '-p',
                    'pose_topic:=/world/capture_ocean_sim/pose/info',
                    '-p',
                    'image_rate:=15.0',
                    '-p',
                    'scan_rate:=10.0',
                    '-p',
                    'publish_images:=false',
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_sensor_agent')),
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    capture_script,
                    '--ros-args',
                    '-r',
                    '__node:=capture_mission',
                    '-p',
                    ['uav_ids:=', uav_ids],
                    '-p',
                    ['usv_ids:=', usv_ids],
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_capture_mission')),
            ),
            OpaqueFunction(
                function=_camera_bridges,
                condition=IfCondition(LaunchConfiguration('start_camera_bridge')),
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    qt_script,
                    '--ros-args',
                    '-r',
                    '__node:=capture_qt_base_station',
                    '-p',
                    'defense_node_name:=/defense_sim_demo',
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_qt')),
            ),
            LogInfo(
                msg=(
                    'PX4 hook reserved for uav_01. Current capture demo uses '
                    'the base-station command interface; start_px4 is kept as '
                    'an integration switch for the next step.'
                ),
                condition=IfCondition(LaunchConfiguration('start_px4')),
            ),
        ]
    )
