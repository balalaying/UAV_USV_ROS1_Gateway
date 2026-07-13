import os
import shlex

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _px4_command(px4_dir, px4_rcs, instance):
    return [
        'bash',
        '-c',
        [
            'set -e; PX4_ROOT=', px4_dir,
            '; BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"; '
            'ETC="$PX4_ROOT/build/px4_sitl_default/etc"; '
            'WORK=/var/tmp/UAV_USV_dual_capture/px4_instance_%d; '
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
        gazebo_share, 'worlds', 'dual_uav_dynamic_capture.sdf'
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

    configured_nav_params = RewrittenYaml(
        source_file=nav_params,
        root_key='',
        param_rewrites={
            'use_sim_time': 'false',
            'robot_base_frame': 'usv_01/base_link',
        },
        convert_types=True,
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

    px4_environments = []
    for vehicle_id in ('uav_01', 'uav_02'):
        environment = dict(gazebo_env)
        environment.update({
            'PX4_SIM_MODEL': 'gz_x500',
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'dual_uav_dynamic_capture',
            'PX4_GZ_MODEL_NAME': vehicle_id,
            'PX4_UXRCE_DDS_NS': vehicle_id,
        })
        px4_environments.append(environment)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='ROS wall time; PX4 still uses Gazebo lockstep time.',
        ),
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
        Node(
            package='uav_usv_sim',
            executable='boat_nav2_interface',
            name='dual_capture_boat_interface',
            output='screen',
            parameters=[
                nav_params,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                    'boat_name': 'usv_01',
                    'base_frame_id': 'usv_01/base_link',
                    'lidar_frame_id': 'landing_boat/hull/front_lidar',
                    'pose_topic': (
                        '/world/dual_uav_dynamic_capture/pose/info'
                    ),
                    'model_pose_topic': '/boat/pose',
                    'boat_cmd_topic': '/model/simple_boat/cmd_vel',
                    'scan_topic': '/boat/scan_raw',
                    'filtered_scan_topic': '/boat/scan',
                    'publish_empty_map': True,
                    'map_width': 400.0,
                    'map_height': 400.0,
                    'enable_lidar_safety': False,
                },
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_nav_params,
                    'autostart': 'true',
                    'use_composition': 'False',
                    'log_level': 'info',
                }.items(),
            )],
        ),
        Node(
            package='uav_usv_usv_control',
            executable='usv_fleet_agent',
            name='usv_01_agent',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'vehicle_id': 'usv_01',
                'odom_topic': '/odom',
                'navigate_action': '/navigate_to_pose',
                'scan_topic': '/boat/scan',
            }],
        ),
        ExecuteProcess(
            cmd=_dds_agent_command(
                px4_ros_ws, uav_agent, 'uav_01', 1, -35.0, -28.0
            ),
            output='screen',
            condition=IfCondition(start_dds_agent),
        ),
        ExecuteProcess(
            cmd=_dds_agent_command(
                px4_ros_ws, uav_agent, 'uav_02', 2, -25.0, -28.0
            ),
            output='screen',
            condition=IfCondition(start_dds_agent),
        ),
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=_px4_command(px4_dir, px4_rcs, 0),
                    output='screen',
                    additional_env=px4_environments[0],
                    condition=IfCondition(start_px4),
                ),
                ExecuteProcess(
                    cmd=_px4_command(px4_dir, px4_rcs, 1),
                    output='screen',
                    additional_env=px4_environments[1],
                    condition=IfCondition(start_px4),
                ),
            ],
        ),
        Node(
            package='uav_usv_mission',
            executable='target_tracker',
            name='target_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'pose_topic': '/world/dual_uav_dynamic_capture/pose/info',
                'track_id': 'enemy_target',
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='capture_manager',
            name='capture_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'uav_ids': ['uav_01', 'uav_02'],
                'usv_ids': ['usv_01'],
                'target_id': 'enemy_target',
                'uav_home_z': 1.35,
                'takeoff_altitude': 18.0,
                'observation_altitude': 22.0,
                'capture_radius': 18.0,
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='capture_visualizer',
            name='capture_visualizer',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='dual_uav_dynamic_capture_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(start_rviz),
        ),
    ])
