import os
import shlex

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString


UAV_CONFIG = (
    ('uav_01', 1, -45.0, -34.0),
    ('uav_02', 2, -35.0, -34.0),
    ('uav_03', 3, -25.0, -34.0),
    ('uav_04', 4, -15.0, -34.0),
)
USV_IDS = ('usv_01', 'usv_02')
WORLD_NAME = 'fleet_dynamic_capture'


def _px4_command(px4_dir, px4_rcs, instance):
    return [
        'bash',
        '-c',
        [
            'set -e; PX4_ROOT=', px4_dir,
            '; BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"; '
            'ETC="$PX4_ROOT/build/px4_sitl_default/etc"; '
            'WORK=/var/tmp/UAV_USV_fleet_capture/px4_instance_%d; '
            'mkdir -p "$WORK"; '
            'rm -f "$WORK/parameters.bson" "$WORK/parameters_backup.bson"; '
            'exec "$BIN" -d -i %d -w "$WORK" -s '
            % (instance, instance),
            shlex.quote(px4_rcs),
            ' "$ETC"',
        ],
    ]


def _dds_agent_command(
    px4_ros_ws, executable, vehicle_id, system_id, home_x, home_y
):
    return [
        'bash',
        '-c',
        [
            'set -e; source ', px4_ros_ws,
            '/install/setup.bash; exec ', shlex.quote(executable),
            ' --ros-args -r __node:=%s_dds_agent '
            '-p use_sim_time:=false -p vehicle_id:=%s '
            '-p px4_namespace:=/%s -p px4_system_id:=%d '
            '-p home_x:=%.3f -p home_y:=%.3f -p home_z:=1.35'
            % (
                vehicle_id,
                vehicle_id,
                vehicle_id,
                system_id,
                home_x,
                home_y,
            ),
        ],
    ]


def _nav_params(source_file, vehicle_id):
    return ReplaceString(
        source_file=source_file,
        replacements={
            'landing_boat/base_link': vehicle_id + '/base_link',
            'global_frame: odom': 'global_frame: ' + vehicle_id + '/odom',
            'odom_topic: /odom': 'odom_topic: odom',
            'topic: /boat/scan': 'topic: scan',
        },
    )


def _boat_interface(vehicle_id, use_sim_time, nav_params):
    return Node(
        package='uav_usv_sim',
        executable='boat_nav2_interface',
        namespace=vehicle_id,
        name='boat_nav2_interface',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[
            nav_params,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'boat_name': vehicle_id,
                'base_frame_id': vehicle_id + '/base_link',
                'odom_frame_id': vehicle_id + '/odom',
                'lidar_frame_id': vehicle_id + '/front_lidar',
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'model_pose_topic': '/unused/%s/pose' % vehicle_id,
                'boat_cmd_topic': '/model/%s/cmd_vel' % vehicle_id,
                'cmd_vel_topic': 'cmd_vel',
                'odom_topic': 'odom',
                'map_topic': 'map',
                'scan_topic': 'scan_raw',
                'filtered_scan_topic': 'scan',
                'scan_range_topic': 'scan_range',
                'marker_topic': 'reference_markers',
                'publish_empty_map': True,
                'map_width': 500.0,
                'map_height': 500.0,
                'enable_lidar_safety': False,
            },
        ],
    )


def _usv_agent(vehicle_id, use_sim_time, unreachable=False):
    parameters = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'vehicle_id': vehicle_id,
        'odom_topic': '/%s/odom' % vehicle_id,
        'camera_topic': '/%s/camera' % vehicle_id,
        'scan_topic': '/%s/scan' % vehicle_id,
        'navigate_action': '/%s/navigate_to_pose' % vehicle_id,
        'emergency_cmd_topic': '/model/%s/cmd_vel' % vehicle_id,
    }
    if unreachable is not False:
        parameters['simulate_unreachable'] = ParameterValue(
            unreachable, value_type=bool
        )
    return Node(
        package='uav_usv_usv_control',
        executable='usv_fleet_agent',
        name=vehicle_id + '_agent',
        output='screen',
        parameters=[parameters],
    )


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_usv_bringup')
    gazebo_share = get_package_share_directory('uav_usv_gazebo')
    gazebo_prefix = get_package_prefix('uav_usv_gazebo')
    sim_share = get_package_share_directory('uav_usv_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    uav_prefix = get_package_prefix('uav_usv_uav_control')

    px4_dir = LaunchConfiguration('px4_dir')
    px4_ros_ws = LaunchConfiguration('px4_ros_ws')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_gazebo = LaunchConfiguration('start_gazebo')
    start_rviz = LaunchConfiguration('start_rviz')
    start_px4 = LaunchConfiguration('start_px4')
    start_dds_agent = LaunchConfiguration('start_dds_agent')
    enable_sudden_turn = LaunchConfiguration('enable_sudden_turn')
    sudden_turn_time = LaunchConfiguration('sudden_turn_time')
    simulate_usv_02_unreachable = LaunchConfiguration(
        'simulate_usv_02_unreachable'
    )
    uav_visual_scale = LaunchConfiguration('uav_visual_scale')

    px4_dir_default = os.path.expanduser(
        os.environ.get('PX4_DIR', '~/PX4-Autopilot')
    )
    px4_ros_default = os.path.expanduser(
        os.environ.get('PX4_ROS_WS', '~/Desktop/Px4_ros')
    )
    px4_models = os.path.join(
        px4_dir_default, 'Tools', 'simulation', 'gz', 'models'
    )
    px4_plugins = os.path.join(
        px4_dir_default,
        'build',
        'px4_sitl_default',
        'src',
        'modules',
        'simulation',
        'gz_plugins',
    )
    gazebo_plugins = os.path.join(
        gazebo_prefix, 'lib', 'uav_usv_gazebo', 'plugins'
    )
    run_world = os.path.join(
        gazebo_prefix, 'lib', 'uav_usv_gazebo', 'run_gz_world.sh'
    )
    world = os.path.join(
        gazebo_share, 'worlds', 'fleet_dynamic_capture.sdf'
    )
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'minimal_dynamic_capture.rviz'
    )
    px4_rcs = os.path.join(
        bringup_share, 'config', 'px4_minimal_capture.rcS'
    )
    nav_params = os.path.join(sim_share, 'config', 'boat_nav2_params.yaml')
    navigation_launch = os.path.join(
        nav2_share, 'launch', 'navigation_launch.py'
    )
    uav_agent = os.path.join(
        uav_prefix,
        'lib',
        'uav_usv_uav_control',
        'uav_dds_fleet_agent',
    )

    gazebo_env = {
        'GZ_SIM_RESOURCE_PATH': (
            gazebo_share + '/models:' + px4_models + ':'
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': (
            gazebo_plugins + ':' + px4_plugins + ':'
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ),
        'GZ_SIM_ARGS': '-r',
    }

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=EnvironmentVariable(
                'PX4_DIR', default_value=px4_dir_default
            ),
        ),
        DeclareLaunchArgument(
            'px4_ros_ws',
            default_value=EnvironmentVariable(
                'PX4_ROS_WS', default_value=px4_ros_default
            ),
        ),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('start_dds_agent', default_value='true'),
        DeclareLaunchArgument('enable_sudden_turn', default_value='true'),
        DeclareLaunchArgument('sudden_turn_time', default_value='55.0'),
        DeclareLaunchArgument('uav_visual_scale', default_value='6.0'),
        DeclareLaunchArgument(
            'simulate_usv_02_unreachable', default_value='false'
        ),
        ExecuteProcess(
            cmd=[run_world, world],
            output='screen',
            additional_env=gazebo_env,
            condition=IfCondition(start_gazebo),
        ),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='log',
            condition=IfCondition(start_dds_agent),
        ),
    ]

    for usv_index, vehicle_id in enumerate(USV_IDS):
        actions.append(_boat_interface(vehicle_id, use_sim_time, nav_params))
        configured_nav_params = _nav_params(nav_params, vehicle_id)
        actions.append(TimerAction(
            period=2.0 + 3.0 * usv_index,
            actions=[GroupAction(actions=[
                PushRosNamespace(vehicle_id),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch),
                    launch_arguments={
                        'namespace': vehicle_id,
                        'use_sim_time': use_sim_time,
                        'params_file': configured_nav_params,
                        'autostart': 'true',
                        'use_composition': 'False',
                        'log_level': 'warn',
                    }.items(),
                ),
            ])],
        ))
    actions.append(_usv_agent('usv_01', use_sim_time))
    actions.append(_usv_agent(
        'usv_02', use_sim_time, simulate_usv_02_unreachable
    ))

    for instance, (vehicle_id, system_id, home_x, home_y) in enumerate(
        UAV_CONFIG
    ):
        actions.append(ExecuteProcess(
            cmd=_dds_agent_command(
                px4_ros_ws,
                uav_agent,
                vehicle_id,
                system_id,
                home_x,
                home_y,
            ),
            output='screen',
            condition=IfCondition(start_dds_agent),
        ))
        px4_environment = dict(gazebo_env)
        px4_environment.update({
            'PX4_SIM_MODEL': 'gz_x500',
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': WORLD_NAME,
            'PX4_GZ_MODEL_NAME': vehicle_id,
            'PX4_UXRCE_DDS_NS': vehicle_id,
        })
        actions.append(TimerAction(
            period=8.0 + 2.0 * instance,
            actions=[ExecuteProcess(
                cmd=_px4_command(px4_dir, px4_rcs, instance),
                output='screen',
                additional_env=px4_environment,
                condition=IfCondition(start_px4),
            )],
        ))

    actions.extend([
        Node(
            package='uav_usv_mission',
            executable='target_tracker',
            name='target_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'track_id': 'enemy_target',
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='capture_target_motion',
            name='capture_target_motion',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'command_topic': '/model/target_vessel/cmd_vel',
                'enable_sudden_turn': ParameterValue(
                    enable_sudden_turn, value_type=bool
                ),
                'sudden_turn_time': ParameterValue(
                    sudden_turn_time, value_type=float
                ),
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='capture_manager',
            name='capture_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'uav_ids': [item[0] for item in UAV_CONFIG],
                'usv_ids': list(USV_IDS),
                'target_id': 'enemy_target',
                'uav_home_z': 1.35,
                'takeoff_altitude': 18.0,
                'observation_altitude': 24.0,
                'capture_radius': 28.0,
                'prediction_horizon': 16.0,
                'command_period': 4.0,
                'command_failure_threshold': 2,
                'encircle_tolerance': 42.0,
                'holding_tolerance': 24.0,
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='uav_visual_shell_spawner',
            name='uav_visual_shell_spawner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'uav_ids': [item[0] for item in UAV_CONFIG],
                'world_name': WORLD_NAME,
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'uav_visual_scale': ParameterValue(
                    uav_visual_scale, value_type=float
                ),
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='capture_visualizer',
            name='capture_visualizer',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'uav_visual_scale': ParameterValue(
                    uav_visual_scale, value_type=float
                ),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='fleet_dynamic_capture_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(start_rviz),
        ),
    ])
    return LaunchDescription(actions)
