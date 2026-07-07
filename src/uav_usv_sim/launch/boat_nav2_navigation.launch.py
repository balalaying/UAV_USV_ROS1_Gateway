import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def bridge_action(name, argument, remappings=None):
    try:
        get_package_prefix('ros_gz_bridge')
    except PackageNotFoundError:
        return LogInfo(msg='ros_gz_bridge is not installed; bridge skipped.')

    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=name,
        output='screen',
        arguments=[argument],
        remappings=remappings or [],
    )


def generate_launch_description():
    package_share = get_package_share_directory('uav_usv_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(
        package_share,
        'config',
        'boat_nav2_params.yaml',
    )
    default_rviz_config = os.path.join(
        package_share,
        'rviz',
        'boat_nav2_navigation.rviz',
    )
    default_map = os.path.join(
        package_share,
        'maps',
        'sydney_coastline.yaml',
    )
    navigation_launch = os.path.join(
        nav2_bringup_share,
        'launch',
        'navigation_launch.py',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation time.',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Nav2 parameters for the boat.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='false',
                description='Start RViz with Nav2 tools.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_rviz_config,
                description='RViz config file.',
            ),
            DeclareLaunchArgument(
                'map',
                default_value=default_map,
                description='Static Sydney coastline occupancy map.',
            ),
            DeclareLaunchArgument(
                'scan_topic',
                default_value='/boat/scan',
                description='Tilt-filtered Boat LaserScan topic for Nav2.',
            ),
            DeclareLaunchArgument(
                'raw_scan_topic',
                default_value='/boat/scan_raw',
                description='Raw Boat LaserScan topic from Gazebo.',
            ),
            DeclareLaunchArgument(
                'boat_cmd_topic',
                default_value='/model/simple_boat/cmd_vel',
                description='Gazebo boat cmd_vel topic.',
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='/cmd_vel',
                description='Nav2 velocity command topic after smoothing.',
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='true',
                description='Automatically activate Nav2 lifecycle nodes.',
            ),

            bridge_action(
                'simulation_clock_bridge',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ),
            bridge_action(
                'boat_lidar_bridge',
                '/boat/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                remappings=[
                    ('/boat/scan', LaunchConfiguration('raw_scan_topic')),
                ],
            ),

            Node(
                package='uav_usv_sim',
                executable='boat_nav2_interface',
                name='boat_nav2_interface',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                        'scan_topic': LaunchConfiguration('raw_scan_topic'),
                        'filtered_scan_topic': LaunchConfiguration('scan_topic'),
                        'boat_cmd_topic': LaunchConfiguration('boat_cmd_topic'),
                        'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                        'publish_empty_map': False,
                    }
                ],
            ),

            Node(
                package='uav_usv_sim',
                executable='nav_goal_marker_relay',
                name='nav_goal_marker_relay',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                        'yaml_filename': LaunchConfiguration('map'),
                    }
                ],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                        'autostart': LaunchConfiguration('autostart'),
                        'node_names': ['map_server'],
                    }
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': LaunchConfiguration('params_file'),
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': 'False',
                    'log_level': 'info',
                }.items(),
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='boat_nav2_rviz',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')],
                parameters=[
                    {
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
        ]
    )
