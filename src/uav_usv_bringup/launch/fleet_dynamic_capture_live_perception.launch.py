"""Run the fleet capture scenario with the live perception display chain."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_lv_dot = LaunchConfiguration('enable_lv_dot')
    enable_console = LaunchConfiguration('enable_console')
    enable_camera_lidar_fusion = LaunchConfiguration(
        'enable_camera_lidar_fusion'
    )
    enable_vision_guided_perception = LaunchConfiguration(
        'enable_vision_guided_perception'
    )

    fleet_launch = PathJoinSubstitution([
        FindPackageShare('uav_usv_bringup'),
        'launch',
        'fleet_dynamic_capture.launch.py',
    ])
    lv_dot_launch = PathJoinSubstitution([
        FindPackageShare('uav_usv_lv_dot_ros2'),
        'launch',
        'lv_dot_ros2.launch.py',
    ])
    console_launch = PathJoinSubstitution([
        FindPackageShare('uav_usv_bringup'),
        'launch',
        'dynamic_capture_console.launch.py',
    ])
    camera_lidar_launch = PathJoinSubstitution([
        FindPackageShare('uav_usv_perception'),
        'launch',
        'camera_lidar_fusion.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('start_dds_agent', default_value='true'),
        DeclareLaunchArgument('target_speed', default_value='1.2'),
        DeclareLaunchArgument(
            'target_nominal_turn_rate', default_value='0.01'
        ),
        DeclareLaunchArgument('enable_sudden_turn', default_value='true'),
        DeclareLaunchArgument('enable_lv_dot', default_value='true'),
        DeclareLaunchArgument('enable_console', default_value='true'),
        DeclareLaunchArgument(
            'enable_camera_lidar_fusion', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_vision_guided_perception', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_global_lidar_fallback', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_affiliation_filter', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_affiliation_qt_mode', default_value='true'
        ),
        DeclareLaunchArgument(
            'camera_detector_backend', default_value='simulation_marker'
        ),
        DeclareLaunchArgument(
            'vision_guided_shadow_mode', default_value='true'
        ),
        DeclareLaunchArgument('enable_mid360', default_value='true'),
        DeclareLaunchArgument('mid360_update_rate', default_value='20.0'),
        DeclareLaunchArgument('mid360_range', default_value='70.0'),
        DeclareLaunchArgument('mid360_voxel_size', default_value='0.12'),
        DeclareLaunchArgument('uav_model_scale', default_value='6.0'),
        DeclareLaunchArgument('uav_camera_rate', default_value='30.0'),
        DeclareLaunchArgument('topdown_point_rate', default_value='20.0'),
        DeclareLaunchArgument('lv_dot_input_max_range', default_value='70.0'),
        DeclareLaunchArgument('lv_dot_local_range_x', default_value='50.0'),
        DeclareLaunchArgument('lv_dot_local_range_y', default_value='20.0'),
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
        DeclareLaunchArgument(
            'rgl_install',
            default_value='/var/tmp/RGLGazeboPlugin/install',
        ),
        DeclareLaunchArgument(
            'rgl_patterns',
            default_value='/var/tmp/RGLGazeboPlugin/lidar_patterns',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fleet_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_gazebo': LaunchConfiguration('start_gazebo'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'start_px4': LaunchConfiguration('start_px4'),
                'start_dds_agent': LaunchConfiguration('start_dds_agent'),
                'target_speed': LaunchConfiguration('target_speed'),
                'target_nominal_turn_rate': LaunchConfiguration(
                    'target_nominal_turn_rate'
                ),
                'enable_sudden_turn': LaunchConfiguration(
                    'enable_sudden_turn'
                ),
                'enable_mid360': LaunchConfiguration('enable_mid360'),
                'mid360_update_rate': LaunchConfiguration(
                    'mid360_update_rate'
                ),
                'mid360_range': LaunchConfiguration('mid360_range'),
                'mid360_voxel_size': LaunchConfiguration(
                    'mid360_voxel_size'
                ),
                'mid360_visualize': 'false',
                'uav_model_scale': LaunchConfiguration('uav_model_scale'),
                'uav_camera_rate': LaunchConfiguration('uav_camera_rate'),
                'px4_dir': LaunchConfiguration('px4_dir'),
                'px4_ros_ws': LaunchConfiguration('px4_ros_ws'),
                'rgl_install': LaunchConfiguration('rgl_install'),
                'rgl_patterns': LaunchConfiguration('rgl_patterns'),
            }.items(),
        ),
        Node(
            package='uav_usv_perception',
            executable='uav_camera_adapter.py',
            name='fleet_live_uav_camera_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'vehicle_ids': [
                    'uav_01', 'uav_02', 'uav_03', 'uav_04', 'usv_01'
                ],
                'expected_rate_hz': LaunchConfiguration('uav_camera_rate'),
            }],
        ),
        TimerAction(
            period=3.0,
            condition=IfCondition(enable_lv_dot),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lv_dot_launch),
                launch_arguments={
                    'vehicle_id': 'usv_01',
                    'points_topic': (
                        '/perception/usv_01/mid360/points_filtered'
                    ),
                    'output_frame': 'map',
                    'node_namespace': '/perception/lv_dot_ros2',
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                    'input_max_range': LaunchConfiguration(
                        'lv_dot_input_max_range'
                    ),
                    'local_range_x': LaunchConfiguration(
                        'lv_dot_local_range_x'
                    ),
                    'local_range_y': LaunchConfiguration(
                        'lv_dot_local_range_y'
                    ),
                }.items(),
            )],
        ),
        TimerAction(
            period=4.0,
            condition=IfCondition(enable_camera_lidar_fusion),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_lidar_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'image_topic': (
                        '/fleet/uplink/usv_01/camera/image_raw'
                    ),
                    'camera_info_topic': (
                        '/fleet/uplink/usv_01/camera/camera_info'
                    ),
                    'enable_vision_guided_perception': (
                        enable_vision_guided_perception
                    ),
                    'enable_global_lidar_fallback': LaunchConfiguration(
                        'enable_global_lidar_fallback'
                    ),
                    'enable_affiliation_filter': LaunchConfiguration(
                        'enable_affiliation_filter'
                    ),
                    'camera_detector_backend': LaunchConfiguration(
                        'camera_detector_backend'
                    ),
                    'vision_guided_shadow_mode': LaunchConfiguration(
                        'vision_guided_shadow_mode'
                    ),
                }.items(),
            )],
        ),
        TimerAction(
            period=5.0,
            condition=IfCondition(enable_console),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(console_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'demo_mode': 'true',
                    'enable_perception_topdown': 'true',
                    'enable_pointcloud_projection': 'true',
                    'topdown_points_input_topic': (
                        '/fleet/uplink/usv_01/mid360/points'
                    ),
                    'topdown_lidar_bboxes_topic': (
                        '/perception/lv_dot_ros2/diagnostics/lidar_bboxes'
                    ),
                    'topdown_camera_topic': (
                        '/perception/usv_01/camera/detections/image'
                    ),
                    'topdown_point_rate': LaunchConfiguration(
                        'topdown_point_rate'
                    ),
                    'enable_affiliation_qt_mode': LaunchConfiguration(
                        'enable_affiliation_qt_mode'
                    ),
                }.items(),
            )],
        ),
    ])
