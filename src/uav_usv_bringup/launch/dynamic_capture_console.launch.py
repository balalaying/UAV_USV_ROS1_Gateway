"""Start the passive Qt console for the running fleet capture scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    demo_mode = LaunchConfiguration('demo_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_perception_topdown = LaunchConfiguration(
        'enable_perception_topdown'
    )
    enable_pointcloud_projection = LaunchConfiguration(
        'enable_pointcloud_projection'
    )
    enable_lv_dot_debug = LaunchConfiguration('enable_lv_dot_debug')
    enable_affiliation_qt_mode = LaunchConfiguration(
        'enable_affiliation_qt_mode'
    )
    return LaunchDescription([
        DeclareLaunchArgument('demo_mode', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'enable_perception_topdown', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_pointcloud_projection', default_value='true'
        ),
        DeclareLaunchArgument('enable_lv_dot_debug', default_value='true'),
        DeclareLaunchArgument(
            'enable_affiliation_qt_mode', default_value='true'
        ),
        DeclareLaunchArgument(
            'topdown_points_input_topic',
            default_value='/fleet/uplink/usv_01/mid360/points',
        ),
        DeclareLaunchArgument(
            'topdown_camera_topic',
            default_value='/fleet/uplink/uav_01/camera/image_raw',
        ),
        DeclareLaunchArgument(
            'topdown_lidar_bboxes_topic',
            default_value=(
                '/perception/lv_dot_ros2/diagnostics/lidar_bboxes'
            ),
        ),
        DeclareLaunchArgument('topdown_point_rate', default_value='10.0'),
        DeclareLaunchArgument('topdown_max_points', default_value='80000'),
        DeclareLaunchArgument('topdown_voxel_size', default_value='0.015'),
        DeclareLaunchArgument(
            'topdown_persistence_frames', default_value='6'
        ),
        DeclareLaunchArgument('topdown_min_z', default_value='-1.0'),
        DeclareLaunchArgument('topdown_max_z', default_value='8.0'),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_debug_visualization_node.py',
            name='lv_dot_debug_visualization',
            output='screen',
            condition=IfCondition(enable_lv_dot_debug),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='qt_pointcloud_projection_node.py',
            name='qt_pointcloud_projection',
            output='screen',
            condition=IfCondition(enable_pointcloud_projection),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'input_topic': LaunchConfiguration(
                    'topdown_points_input_topic'
                ),
                'pointcloud_display_rate_hz': ParameterValue(
                    LaunchConfiguration('topdown_point_rate'),
                    value_type=float,
                ),
                'pointcloud_max_points': ParameterValue(
                    LaunchConfiguration('topdown_max_points'),
                    value_type=int,
                ),
                'pointcloud_voxel_size': ParameterValue(
                    LaunchConfiguration('topdown_voxel_size'),
                    value_type=float,
                ),
                'pointcloud_min_z': ParameterValue(
                    LaunchConfiguration('topdown_min_z'),
                    value_type=float,
                ),
                'pointcloud_max_z': ParameterValue(
                    LaunchConfiguration('topdown_max_z'),
                    value_type=float,
                ),
                'pointcloud_persistence_frames': ParameterValue(
                    LaunchConfiguration('topdown_persistence_frames'),
                    value_type=int,
                ),
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='qt_pointcloud_projection_node.py',
            name='lv_dot_debug_filtered_projection',
            output='screen',
            condition=IfCondition(enable_lv_dot_debug),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'input_topic': '/perception/lv_dot/debug/cloud',
                'output_topic': (
                    '/perception/lv_dot/debug/cloud_filtered_map'
                ),
                'status_topic': (
                    '/perception/lv_dot/debug/cloud_filtered_status'
                ),
                'output_frame': 'map',
                'pointcloud_display_rate_hz': 20.0,
                'pointcloud_max_points': 60000,
                'pointcloud_voxel_size': 0.03,
                'pointcloud_min_z': -1.0,
                'pointcloud_max_z': 8.0,
                'pointcloud_persistence_frames': 4,
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='fleet_base_station',
            name='dynamic_capture_sensor_hub',
            output='screen',
            parameters=[{
                'auto_demo': False,
                'monitor_only': True,
                'owner_id': 'dynamic_capture_console',
                'uav_id': 'uav_01',
                'usv_id': 'usv_01',
                'uav_ids': 'uav_01,uav_02,uav_03',
                'usv_ids': 'usv_01,usv_02,usv_03',
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='fleet_base_station_gui',
            name='dynamic_capture_console',
            output='screen',
            parameters=[{
                'demo_mode': ParameterValue(demo_mode, value_type=bool),
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'enable_perception_topdown': ParameterValue(
                    enable_perception_topdown, value_type=bool
                ),
                'enable_lv_dot_debug': ParameterValue(
                    enable_lv_dot_debug, value_type=bool
                ),
                'enable_affiliation_qt_mode': ParameterValue(
                    enable_affiliation_qt_mode, value_type=bool
                ),
                'topdown_camera_topic': LaunchConfiguration(
                    'topdown_camera_topic'
                ),
                'topdown_lidar_bboxes_topic': LaunchConfiguration(
                    'topdown_lidar_bboxes_topic'
                ),
                'capture_namespace': '',
                'defense_namespace': '',
            }],
        ),
    ])
