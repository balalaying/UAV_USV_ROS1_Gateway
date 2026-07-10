from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation time.',
            ),
            DeclareLaunchArgument(
                'uav_model_name',
                default_value='x500_mono_cam_down_0',
                description='Gazebo entity name of the camera-equipped UAV.',
            ),
            DeclareLaunchArgument(
                'mavlink_url',
                default_value='udp:127.0.0.1:14540',
                description='PX4 MAVLink endpoint.',
            ),
            DeclareLaunchArgument(
                'takeoff_altitude',
                default_value='16.0',
                description='UAV visual patrol altitude in metres.',
            ),
            DeclareLaunchArgument(
                'patrol_speed',
                default_value='3.0',
                description='UAV patrol speed in m/s.',
            ),
            DeclareLaunchArgument(
                'target_speed',
                default_value='3.2',
                description='UAV speed after target detection in m/s.',
            ),
            DeclareLaunchArgument(
                'boat_standoff_distance',
                default_value='7.0',
                description='Safe goal distance from the buoy.',
            ),
            DeclareLaunchArgument(
                'image_processing_rate',
                default_value='24.0',
                description='Detection and camera mosaic rate in FPS.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Start RViz with the UAV camera mosaic.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('uav_usv_sim'),
                            'launch',
                            'uav_buoy_cooperative_navigation.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'uav_model_name': LaunchConfiguration('uav_model_name'),
                    'mavlink_url': LaunchConfiguration('mavlink_url'),
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'patrol_speed': LaunchConfiguration('patrol_speed'),
                    'target_speed': LaunchConfiguration('target_speed'),
                    'boat_standoff_distance': LaunchConfiguration(
                        'boat_standoff_distance'
                    ),
                    'image_processing_rate': LaunchConfiguration(
                        'image_processing_rate'
                    ),
                    'start_nav2': 'false',
                    'start_rviz': LaunchConfiguration('start_rviz'),
                }.items(),
            ),
        ]
    )
