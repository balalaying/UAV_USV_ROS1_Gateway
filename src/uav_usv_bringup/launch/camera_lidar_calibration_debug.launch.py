"""Fixed-target Camera-Mid360 calibration with passive Qt visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    live_launch = PathJoinSubstitution([
        FindPackageShare('uav_usv_bringup'),
        'launch',
        'fleet_dynamic_capture_live_perception.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_console', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(live_launch),
            launch_arguments={
                'start_gazebo': LaunchConfiguration('start_gazebo'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'start_px4': 'false',
                'start_dds_agent': 'false',
                'enable_console': LaunchConfiguration('start_console'),
                'enable_mid360': 'true',
                'enable_lv_dot': 'true',
                'enable_camera_lidar_fusion': 'true',
                'enable_vision_guided_perception': 'true',
                'target_speed': '0.0',
                'target_nominal_turn_rate': '0.0',
                'enable_sudden_turn': 'false',
            }.items(),
        ),
    ])
