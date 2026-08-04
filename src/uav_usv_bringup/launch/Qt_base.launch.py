"""Attach the USV_01 live perception chain and Qt to a running 332 core."""

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
        DeclareLaunchArgument('enable_console', default_value='true'),
        DeclareLaunchArgument('enable_lv_dot', default_value='true'),
        DeclareLaunchArgument(
            'enable_camera_lidar_fusion', default_value='true'
        ),
        DeclareLaunchArgument('topdown_point_rate', default_value='8.0'),
        DeclareLaunchArgument('topdown_max_points', default_value='6000'),
        DeclareLaunchArgument('topdown_voxel_size', default_value='0.16'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(live_launch),
            launch_arguments={
                'start_fleet_core': 'false',
                'enable_console': LaunchConfiguration('enable_console'),
                'enable_lv_dot': LaunchConfiguration('enable_lv_dot'),
                'enable_camera_lidar_fusion': LaunchConfiguration(
                    'enable_camera_lidar_fusion'
                ),
                'enable_secondary_usv_perception': 'false',
                'enable_perception_topdown': 'false',
                'enable_pointcloud_projection': 'false',
                'enable_debug_filtered_projection': 'false',
                'perception_start_delay': '0.5',
                'console_start_delay': '3.0',
                'mid360_vehicle_ids': 'usv_01',
                'topdown_point_rate': LaunchConfiguration(
                    'topdown_point_rate'
                ),
                'topdown_max_points': LaunchConfiguration(
                    'topdown_max_points'
                ),
                'topdown_voxel_size': LaunchConfiguration(
                    'topdown_voxel_size'
                ),
                'topdown_persistence_frames': '1',
            }.items(),
        ),
    ])
