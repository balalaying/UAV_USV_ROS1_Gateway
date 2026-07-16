from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    dynamic_tracks_topic = LaunchConfiguration('dynamic_tracks_topic')
    observation_topic = LaunchConfiguration('observation_topic')
    ground_truth_topic = LaunchConfiguration('ground_truth_topic')
    fused_topic = LaunchConfiguration('fused_topic')
    metrics_topic = LaunchConfiguration('metrics_topic')
    target_id = LaunchConfiguration('target_id')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'dynamic_tracks_topic',
            default_value='/perception/lv_dot_ros2/dynamic_tracks',
        ),
        DeclareLaunchArgument(
            'observation_topic',
            default_value='/perception/lv_dot/observations',
        ),
        DeclareLaunchArgument(
            'ground_truth_topic',
            default_value='/perception/ground_truth/tracks',
        ),
        DeclareLaunchArgument(
            'fused_topic',
            default_value='/perception/lv_dot/fused_tracks',
        ),
        DeclareLaunchArgument(
            'metrics_topic',
            default_value='/perception/lv_dot/fusion_metrics',
        ),
        DeclareLaunchArgument('target_id', default_value='target_vessel'),
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
                'output_topic': observation_topic,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='perception_fusion_node.py',
            name='lv_dot_shadow_fusion',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'input_topics_csv': [
                    ground_truth_topic, ',', observation_topic
                ],
                'output_topic': fused_topic,
                'target_frame': 'map',
                'association_distance': 12.0,
                'sync_slop_seconds': 0.5,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_fusion_evaluator.py',
            name='lv_dot_fusion_evaluator',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'ground_truth_topic': ground_truth_topic,
                'observation_topic': observation_topic,
                'fusion_topic': fused_topic,
                'metrics_topic': metrics_topic,
                'target_id': target_id,
            }],
        ),
    ])
