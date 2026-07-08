from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_usv_bringup')
    demo_launch = (
        bringup_share + '/launch/Qt_cooperation.launch.py'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation time.',
            ),
            DeclareLaunchArgument(
                'start_nav2',
                default_value='true',
                description='Start the main USV Nav2 stack.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='false',
                description='Open RViz together with the Qt base station.',
            ),
            DeclareLaunchArgument(
                'start_gui',
                default_value='true',
                description='Open the Qt base-station console.',
            ),
            DeclareLaunchArgument(
                'auto_demo',
                default_value='true',
                description='Automatically run the cooperative demo.',
            ),
            DeclareLaunchArgument(
                'target_x',
                default_value='24.0',
                description='Base-station target X in map frame.',
            ),
            DeclareLaunchArgument(
                'target_y',
                default_value='8.0',
                description='Base-station target Y in map frame.',
            ),
            DeclareLaunchArgument(
                'uav_altitude',
                default_value='16.0',
                description='UAV cooperative flight altitude.',
            ),
            DeclareLaunchArgument(
                'start_simulated_fleet',
                default_value='true',
                description='Show auxiliary fleet members 02 and 03.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'start_nav2': LaunchConfiguration('start_nav2'),
                    'start_rviz': LaunchConfiguration('start_rviz'),
                    'start_gui': LaunchConfiguration('start_gui'),
                    'auto_demo': LaunchConfiguration('auto_demo'),
                    'target_x': LaunchConfiguration('target_x'),
                    'target_y': LaunchConfiguration('target_y'),
                    'uav_altitude': LaunchConfiguration('uav_altitude'),
                    'start_simulated_fleet': LaunchConfiguration(
                        'start_simulated_fleet'
                    ),
                }.items(),
            )
        ]
    )
