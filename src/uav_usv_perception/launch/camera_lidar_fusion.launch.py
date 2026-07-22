"""Start the passive USV camera and Mid-360 association pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = PathJoinSubstitution([
        FindPackageShare('uav_usv_perception'), 'config',
        'vision_guided_usv_perception.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
            'camera_detector_backend', default_value='simulation_marker'
        ),
        DeclareLaunchArgument(
            'vision_guided_shadow_mode', default_value='true'
        ),
        DeclareLaunchArgument('vehicle_id', default_value='usv_01'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/fleet/uplink/usv_01/camera/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/fleet/uplink/usv_01/camera/camera_info',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/perception/usv_01/camera/detections',
        ),
        DeclareLaunchArgument(
            'affiliated_detections_topic',
            default_value='/perception/usv_01/camera/affiliated_detections',
        ),
        DeclareLaunchArgument(
            'points_topic',
            default_value='/perception/usv_01/mid360/points_filtered',
        ),
        DeclareLaunchArgument(
            'lidar_bboxes_topic',
            default_value=(
                '/perception/lv_dot_ros2/diagnostics/lidar_bboxes'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_tracks_topic',
            default_value='/perception/lv_dot_ros2/tracks',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value=(
                '/perception/usv_01/camera_lidar/observations'
            ),
        ),
        DeclareLaunchArgument('camera_frame', default_value='usv_01/camera_link'),
        DeclareLaunchArgument('output_frame', default_value='map'),
        DeclareLaunchArgument('sync_slop_seconds', default_value='0.20'),
        DeclareLaunchArgument('camera_max_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('camera_min_pixels', default_value='2'),
        DeclareLaunchArgument(
            'camera_maximum_center_y_ratio', default_value='0.88'
        ),
        DeclareLaunchArgument(
            'minimum_association_score', default_value='0.08'
        ),
        DeclareLaunchArgument('association_pixel_gate', default_value='80.0'),
        DeclareLaunchArgument(
            'minimum_lidar_xy_extent', default_value='0.20'
        ),
        Node(
            package='uav_usv_perception',
            executable='usv_camera_detection_node.py',
            name='usv_01_camera_detection',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'image_topic': LaunchConfiguration('image_topic'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'affiliated_detections_topic': LaunchConfiguration(
                    'affiliated_detections_topic'
                ),
                'detector_backend': LaunchConfiguration(
                    'camera_detector_backend'
                ),
                'enable_affiliation_filter': ParameterValue(
                    LaunchConfiguration('enable_affiliation_filter'),
                    value_type=bool,
                ),
                'debug_image_topic': (
                    '/perception/usv_01/camera/detections/image'
                ),
                'max_rate_hz': ParameterValue(
                    LaunchConfiguration('camera_max_rate_hz'),
                    value_type=float,
                ),
                'min_pixels': ParameterValue(
                    LaunchConfiguration('camera_min_pixels'),
                    value_type=int,
                ),
                'maximum_center_y_ratio': ParameterValue(
                    LaunchConfiguration('camera_maximum_center_y_ratio'),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='vision_guided_lidar_roi_node.py',
            name='usv_01_vision_guided_lidar_roi',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('enable_vision_guided_perception')
            ),
            parameters=[config, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'detections_topic': LaunchConfiguration(
                    'affiliated_detections_topic'
                ),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'points_topic': LaunchConfiguration('points_topic'),
                'tracks_topic': LaunchConfiguration('lidar_tracks_topic'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'output_frame': LaunchConfiguration('output_frame'),
                'shadow_mode': ParameterValue(
                    LaunchConfiguration('vision_guided_shadow_mode'),
                    value_type=bool,
                ),
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='camera_lidar_association_node.py',
            name='usv_01_camera_lidar_association',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'camera_detections_topic': LaunchConfiguration(
                    'detections_topic'
                ),
                'affiliated_detections_topic': LaunchConfiguration(
                    'affiliated_detections_topic'
                ),
                'vision_guided_observations_topic': (
                    '/perception/usv_01/vision_guided/observations'
                ),
                'camera_info_topic': LaunchConfiguration(
                    'camera_info_topic'
                ),
                'lidar_bboxes_topic': LaunchConfiguration(
                    'lidar_bboxes_topic'
                ),
                'lidar_tracks_topic': LaunchConfiguration(
                    'lidar_tracks_topic'
                ),
                'output_topic': LaunchConfiguration('output_topic'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'output_frame': LaunchConfiguration('output_frame'),
                'sync_slop_seconds': ParameterValue(
                    LaunchConfiguration('sync_slop_seconds'), value_type=float
                ),
                'minimum_association_score': ParameterValue(
                    LaunchConfiguration('minimum_association_score'),
                    value_type=float,
                ),
                'pixel_gate': ParameterValue(
                    LaunchConfiguration('association_pixel_gate'),
                    value_type=float,
                ),
                'minimum_lidar_xy_extent': ParameterValue(
                    LaunchConfiguration('minimum_lidar_xy_extent'),
                    value_type=float,
                ),
                'enable_global_lidar_fallback': ParameterValue(
                    LaunchConfiguration('enable_global_lidar_fallback'),
                    value_type=bool,
                ),
            }],
        ),
    ])
