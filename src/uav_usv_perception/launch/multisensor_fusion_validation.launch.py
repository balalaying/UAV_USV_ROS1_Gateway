from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    vehicle_id = LaunchConfiguration('vehicle_id')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    ground_truth_topic = LaunchConfiguration('ground_truth_topic')
    dynamic_tracks_topic = LaunchConfiguration('dynamic_tracks_topic')
    lv_dot_topic = LaunchConfiguration('lv_dot_observation_topic')
    uav_topic = LaunchConfiguration('uav_observation_topic')
    fused_topic = LaunchConfiguration('fused_topic')
    metrics_topic = LaunchConfiguration('metrics_topic')
    target_id = LaunchConfiguration('target_id')
    require_in_fov = LaunchConfiguration('require_in_fov')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('vehicle_id', default_value='uav_01'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/fleet/uplink/uav_01/camera/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/fleet/uplink/uav_01/camera/camera_info',
        ),
        DeclareLaunchArgument(
            'ground_truth_topic',
            default_value='/perception/ground_truth/tracks',
        ),
        DeclareLaunchArgument(
            'dynamic_tracks_topic',
            default_value='/perception/lv_dot_ros2/dynamic_tracks',
        ),
        DeclareLaunchArgument(
            'lv_dot_observation_topic',
            default_value='/perception/lv_dot/observations',
        ),
        DeclareLaunchArgument(
            'uav_observation_topic',
            default_value='/perception/uav_01/observations',
        ),
        DeclareLaunchArgument(
            'fused_topic', default_value='/perception/fused/tracks'
        ),
        DeclareLaunchArgument(
            'metrics_topic',
            default_value='/perception/multisensor/metrics',
        ),
        DeclareLaunchArgument('target_id', default_value='target_vessel'),
        DeclareLaunchArgument('require_in_fov', default_value='true'),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_observation_adapter.py',
            name='lv_dot_observation_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'input_topic': dynamic_tracks_topic,
                'output_topic': lv_dot_topic,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='uav_visual_observation_node.py',
            name='uav_visual_observation_node',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'vehicle_id': vehicle_id,
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'ground_truth_topic': ground_truth_topic,
                'output_topic': uav_topic,
                'target_id': target_id,
                'require_in_fov': ParameterValue(
                    require_in_fov, value_type=bool
                ),
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='perception_fusion_node.py',
            name='multisensor_shadow_fusion',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'input_topics_csv': [
                    ground_truth_topic, ',', lv_dot_topic, ',', uav_topic
                ],
                'output_topic': fused_topic,
                'target_frame': 'map',
                'association_distance': 12.0,
                'sync_slop_seconds': 0.5,
                'aggregation_wait_seconds': 0.20,
                'observation_history_seconds': 1.0,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='multisensor_validation_evaluator.py',
            name='multisensor_validation_evaluator',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'ground_truth_topic': ground_truth_topic,
                'lv_dot_topic': lv_dot_topic,
                'uav_camera_topic': uav_topic,
                'fusion_topic': fused_topic,
                'metrics_topic': metrics_topic,
                'target_id': target_id,
            }],
        ),
    ])
