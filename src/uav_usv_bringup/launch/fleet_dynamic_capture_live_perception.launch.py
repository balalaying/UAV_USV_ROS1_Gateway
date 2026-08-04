"""Run the fleet capture scenario with the live perception display chain.

REFRACTOR_METADATA
STATUS: primary
PURPOSE: The only complete 332 integrated entry (Gazebo, PX4, perception,
         Fleet World Model, Base Station Service, and clients).
CANONICAL_SUCCESSOR: self
SAFE_TO_RUN_WITH_MAIN_LAUNCH: yes (this is the main launch)
"""

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
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


USV_IDS = ('usv_01', 'usv_02', 'usv_03')


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
    enable_base_station_service = LaunchConfiguration(
        'enable_base_station_service'
    )
    start_fleet_core = LaunchConfiguration('start_fleet_core')
    enable_secondary_usv_perception = LaunchConfiguration(
        'enable_secondary_usv_perception'
    )
    perception_start_delay = LaunchConfiguration('perception_start_delay')
    console_start_delay = LaunchConfiguration('console_start_delay')

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
    base_station_config = PathJoinSubstitution([
        FindPackageShare('uav_usv_base_station'), 'config',
        'base_station.yaml',
    ])

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'start_fleet_core', default_value='true',
            description=(
                'Start Gazebo, PX4, Nav2 and mission core. Set false when '
                'using the split perception/UI launch.'
            ),
        ),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('start_dds_agent', default_value='true'),
        DeclareLaunchArgument('enable_px4_camera_follow', default_value='true'),
        DeclareLaunchArgument('camera_follow_target', default_value='uav_01'),
        DeclareLaunchArgument('camera_follow_offset_x', default_value='-42.0'),
        DeclareLaunchArgument('camera_follow_offset_y', default_value='-42.0'),
        DeclareLaunchArgument('camera_follow_offset_z', default_value='28.0'),
        DeclareLaunchArgument('camera_follow_delay', default_value='10.0'),
        DeclareLaunchArgument('target_speed', default_value='1.2'),
        DeclareLaunchArgument(
            'target_nominal_turn_rate', default_value='0.01'
        ),
        DeclareLaunchArgument('enable_sudden_turn', default_value='true'),
        DeclareLaunchArgument('enable_lv_dot', default_value='true'),
        DeclareLaunchArgument(
            'enable_secondary_usv_perception', default_value='false',
            description=(
                'Enable Mid-360 LV-DOT/fusion processing for USV_02/03. '
                'The live demonstration defaults to USV_01 only.'
            ),
        ),
        DeclareLaunchArgument(
            'perception_start_delay', default_value='18.0'
        ),
        DeclareLaunchArgument('console_start_delay', default_value='22.0'),
        DeclareLaunchArgument('enable_console', default_value='true'),
        DeclareLaunchArgument(
            'enable_camera_lidar_fusion', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_vision_guided_perception', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_base_station_service', default_value='true'
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
        DeclareLaunchArgument(
            'mid360_vehicle_ids', default_value='usv_01',
            description='Comma-separated USVs carrying active demo Mid-360s.',
        ),
        DeclareLaunchArgument('mid360_update_rate', default_value='12.0'),
        DeclareLaunchArgument('mid360_range', default_value='70.0'),
        DeclareLaunchArgument('mid360_voxel_size', default_value='0.12'),
        DeclareLaunchArgument('uav_model_scale', default_value='12.0'),
        DeclareLaunchArgument('usv_model_scale', default_value='2.0'),
        DeclareLaunchArgument('uav_camera_rate', default_value='15.0'),
        DeclareLaunchArgument('topdown_point_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'enable_perception_topdown', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_pointcloud_projection', default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_debug_filtered_projection', default_value='true'
        ),
        DeclareLaunchArgument('topdown_max_points', default_value='12000'),
        DeclareLaunchArgument('topdown_voxel_size', default_value='0.10'),
        DeclareLaunchArgument(
            'topdown_persistence_frames', default_value='1'
        ),
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
            condition=IfCondition(start_fleet_core),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_gazebo': LaunchConfiguration('start_gazebo'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'start_px4': LaunchConfiguration('start_px4'),
                'start_dds_agent': LaunchConfiguration('start_dds_agent'),
                'enable_px4_camera_follow': LaunchConfiguration(
                    'enable_px4_camera_follow'
                ),
                'camera_follow_target': LaunchConfiguration(
                    'camera_follow_target'
                ),
                'camera_follow_offset_x': LaunchConfiguration(
                    'camera_follow_offset_x'
                ),
                'camera_follow_offset_y': LaunchConfiguration(
                    'camera_follow_offset_y'
                ),
                'camera_follow_offset_z': LaunchConfiguration(
                    'camera_follow_offset_z'
                ),
                'camera_follow_delay': LaunchConfiguration(
                    'camera_follow_delay'
                ),
                'target_speed': LaunchConfiguration('target_speed'),
                'target_nominal_turn_rate': LaunchConfiguration(
                    'target_nominal_turn_rate'
                ),
                'enable_sudden_turn': LaunchConfiguration(
                    'enable_sudden_turn'
                ),
                'enable_mid360': LaunchConfiguration('enable_mid360'),
                'mid360_vehicle_ids': LaunchConfiguration(
                    'mid360_vehicle_ids'
                ),
                'mid360_update_rate': LaunchConfiguration(
                    'mid360_update_rate'
                ),
                'mid360_range': LaunchConfiguration('mid360_range'),
                'mid360_voxel_size': LaunchConfiguration(
                    'mid360_voxel_size'
                ),
                'mid360_visualize': 'false',
                'uav_model_scale': LaunchConfiguration('uav_model_scale'),
                'usv_model_scale': LaunchConfiguration('usv_model_scale'),
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
                    'uav_01', 'uav_02', 'uav_03',
                    'usv_01', 'usv_02', 'usv_03',
                ],
                'expected_rate_hz': LaunchConfiguration('uav_camera_rate'),
            }],
        ),
        Node(
            package='uav_usv_base_station',
            executable='base_station_service',
            name='base_station_service',
            output='screen',
            condition=IfCondition(enable_base_station_service),
            parameters=[base_station_config, {
                'use_sim_time': use_sim_time,
                'world_model_topic': '/fleet/world_model',
                'state_topic': '/base_station/state',
                'events_topic': '/base_station/events',
                'publish_rate_hz': 5.0,
                'target_history_length': 120,
                'map_frame': 'map',
                'communication_status': 'LOCAL_SIMULATION',
            }],
        ),
        TimerAction(
            period=console_start_delay,
            condition=IfCondition(enable_console),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(console_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'demo_mode': 'true',
                    'fleet_world_model_only': 'true',
                    'use_base_station_service': 'true',
                    'enable_perception_topdown': LaunchConfiguration(
                        'enable_perception_topdown'
                    ),
                    'enable_pointcloud_projection': LaunchConfiguration(
                        'enable_pointcloud_projection'
                    ),
                    'enable_debug_filtered_projection': LaunchConfiguration(
                        'enable_debug_filtered_projection'
                    ),
                    'topdown_points_input_topic': (
                        '/fleet/uplink/usv_01/mid360/points'
                    ),
                    'topdown_lidar_bboxes_topic': (
                        '/perception/lv_dot/usv_01/'
                        'diagnostics/lidar_bboxes'
                    ),
                    'topdown_camera_topic': (
                        '/perception/usv_01/camera/detections/image'
                    ),
                    'topdown_point_rate': LaunchConfiguration(
                        'topdown_point_rate'
                    ),
                    'topdown_max_points': LaunchConfiguration(
                        'topdown_max_points'
                    ),
                    'topdown_voxel_size': LaunchConfiguration(
                        'topdown_voxel_size'
                    ),
                    'topdown_persistence_frames': LaunchConfiguration(
                        'topdown_persistence_frames'
                    ),
                    'enable_affiliation_qt_mode': LaunchConfiguration(
                        'enable_affiliation_qt_mode'
                    ),
                }.items(),
            )],
        ),
    ]

    for index, vehicle_id in enumerate(USV_IDS):
        lv_dot_namespace = '/perception/lv_dot/' + vehicle_id
        vehicle_perception = '/perception/' + vehicle_id
        if index == 0:
            lv_dot_condition = IfCondition(enable_lv_dot)
            camera_lidar_condition = IfCondition(
                enable_camera_lidar_fusion
            )
        else:
            lv_dot_condition = IfCondition(PythonExpression([
                "'", enable_lv_dot, "'.lower() == 'true' and '",
                enable_secondary_usv_perception,
                "'.lower() == 'true'",
            ]))
            camera_lidar_condition = IfCondition(PythonExpression([
                "'", enable_camera_lidar_fusion,
                "'.lower() == 'true' and '",
                enable_secondary_usv_perception,
                "'.lower() == 'true'",
            ]))
        actions.append(TimerAction(
            period=PythonExpression([
                perception_start_delay, ' + ', str(0.5 * index)
            ]),
            condition=lv_dot_condition,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lv_dot_launch),
                launch_arguments={
                    'vehicle_id': vehicle_id,
                    'points_topic': (
                        vehicle_perception
                        + '/mid360/points_filtered'
                    ),
                    'output_frame': 'map',
                    'node_namespace': lv_dot_namespace,
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
        ))
        actions.append(Node(
            package='uav_usv_perception',
            executable='lv_dot_observation_adapter.py',
            name=vehicle_id + '_lv_dot_observation_adapter',
            output='screen',
            condition=lv_dot_condition,
            parameters=[{
                'input_topic': lv_dot_namespace + '/dynamic_tracks',
                'output_topic': (
                    vehicle_perception + '/lidar/observations'
                ),
                'status_topic': (
                    vehicle_perception + '/lidar/observation_status'
                ),
            }],
        ))
        actions.append(TimerAction(
            period=PythonExpression([
                perception_start_delay, ' + ', str(2.0 + 0.5 * index)
            ]),
            condition=camera_lidar_condition,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_lidar_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'vehicle_id': vehicle_id,
                    'image_topic': (
                        '/fleet/uplink/%s/camera/image_raw' % vehicle_id
                    ),
                    'camera_info_topic': (
                        '/fleet/uplink/%s/camera/camera_info' % vehicle_id
                    ),
                    'detections_topic': (
                        vehicle_perception + '/camera/detections'
                    ),
                    'affiliated_detections_topic': (
                        vehicle_perception
                        + '/camera/affiliated_detections'
                    ),
                    'points_topic': (
                        vehicle_perception
                        + '/mid360/points_filtered'
                    ),
                    'lidar_bboxes_topic': (
                        lv_dot_namespace + '/diagnostics/lidar_bboxes'
                    ),
                    'lidar_tracks_topic': (
                        lv_dot_namespace + '/tracks'
                    ),
                    'output_topic': (
                        vehicle_perception + '/observations'
                    ),
                    'camera_frame': vehicle_id + '/camera_link',
                    'debug_image_topic': (
                        vehicle_perception
                        + '/camera/detections/image'
                    ),
                    'camera_status_topic': (
                        vehicle_perception
                        + '/camera/detection_status'
                    ),
                    'vision_observations_topic': (
                        vehicle_perception
                        + '/vision_guided/observations'
                    ),
                    'vision_roi_cloud_topic': (
                        vehicle_perception + '/vision_guided/roi_cloud'
                    ),
                    'vision_roi_bboxes_topic': (
                        vehicle_perception + '/vision_guided/roi_bboxes'
                    ),
                    'vision_camera_projection_topic': (
                        vehicle_perception
                        + '/vision_guided/camera_projection'
                    ),
                    'vision_status_topic': (
                        vehicle_perception + '/vision_guided/status'
                    ),
                    'lidar_only_markers_topic': (
                        vehicle_perception
                        + '/camera_lidar/lidar_only_bboxes'
                    ),
                    'camera_only_markers_topic': (
                        vehicle_perception
                        + '/camera_lidar/camera_only_bboxes'
                    ),
                    'fused_markers_topic': (
                        vehicle_perception
                        + '/camera_lidar/fused_bboxes'
                    ),
                    'association_status_topic': (
                        vehicle_perception + '/camera_lidar/status'
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
        ))

    usv_observation_topics = [
        '/perception/%s/observations' % vehicle_id
        for vehicle_id in USV_IDS
    ]
    actions.extend([
        Node(
            package='uav_usv_perception',
            executable='perception_fusion_node.py',
            name='fleet_usv_perception_fusion',
            output='screen',
            parameters=[{
                'input_topics': usv_observation_topics,
                'output_topic': '/fleet/perception/usv_tracks',
                'target_frame': 'map',
                'aggregation_wait_seconds': 0.0,
                'observation_history_seconds': 0.0,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='perception_fusion_node.py',
            name='fleet_global_perception_fusion',
            output='screen',
            parameters=[{
                'input_topics': ['/fleet/perception/usv_tracks'],
                'output_topic': '/fleet/perception/fused_targets',
                'output_alias_topics': ['/perception/fused/tracks'],
                'target_frame': 'map',
                'aggregation_wait_seconds': 0.0,
                'observation_history_seconds': 0.0,
            }],
        ),
    ])
    return LaunchDescription(actions)
