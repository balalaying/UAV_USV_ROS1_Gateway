"""Start the 332 simulator, vehicle control and mission core.

Run this first for a responsive demonstration startup.  The live perception
and Qt client are intentionally started by the companion perception launch.
The canonical all-in-one launch remains fully compatible.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    core_launch = PathJoinSubstitution([
        FindPackageShare('uav_usv_bringup'),
        'launch',
        'fleet_dynamic_capture.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('start_dds_agent', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('enable_mid360', default_value='true'),
        DeclareLaunchArgument('mid360_vehicle_ids', default_value='usv_01'),
        DeclareLaunchArgument('mid360_update_rate', default_value='12.0'),
        DeclareLaunchArgument('mid360_range', default_value='70.0'),
        DeclareLaunchArgument('uav_model_scale', default_value='12.0'),
        DeclareLaunchArgument('usv_model_scale', default_value='2.0'),
        DeclareLaunchArgument('uav_camera_rate', default_value='15.0'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=EnvironmentVariable(
                'PX4_DIR',
                default_value=os.path.expanduser('~/PX4-Autopilot'),
            ),
        ),
        DeclareLaunchArgument(
            'px4_ros_ws',
            default_value=EnvironmentVariable(
                'PX4_ROS_WS',
                default_value=os.path.expanduser('~/Desktop/Px4_ros'),
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'start_gazebo': 'true',
                'start_px4': LaunchConfiguration('start_px4'),
                'start_dds_agent': LaunchConfiguration('start_dds_agent'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'enable_mid360': LaunchConfiguration('enable_mid360'),
                'mid360_vehicle_ids': LaunchConfiguration(
                    'mid360_vehicle_ids'
                ),
                'mid360_update_rate': LaunchConfiguration(
                    'mid360_update_rate'
                ),
                'mid360_range': LaunchConfiguration('mid360_range'),
                'mid360_visualize': 'false',
                'uav_model_scale': LaunchConfiguration('uav_model_scale'),
                'usv_model_scale': LaunchConfiguration('usv_model_scale'),
                'uav_camera_rate': LaunchConfiguration('uav_camera_rate'),
                'px4_dir': LaunchConfiguration('px4_dir'),
                'px4_ros_ws': LaunchConfiguration('px4_ros_ws'),
            }.items(),
        ),
    ])
