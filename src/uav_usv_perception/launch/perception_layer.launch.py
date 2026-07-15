from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_ground_truth = LaunchConfiguration('start_ground_truth_adapter')
    pose_topic = LaunchConfiguration('pose_topic')
    target_entity = LaunchConfiguration('target_entity')
    source = LaunchConfiguration('perception_source')
    uav_topic = LaunchConfiguration('uav_observation_topic')
    usv_topic = LaunchConfiguration('usv_observation_topic')
    lv_dot_topic = LaunchConfiguration('lv_dot_observation_topic')

    ground_truth_topic = '/perception/ground_truth/tracks'
    sensor_topic = '/perception/fused/tracks'
    hybrid_topic = '/perception/hybrid/tracks'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'start_ground_truth_adapter', default_value='true'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/world/minimal_dynamic_capture/pose/info',
        ),
        DeclareLaunchArgument('target_entity', default_value='target_vessel'),
        DeclareLaunchArgument(
            'perception_source', default_value='ground_truth'
        ),
        DeclareLaunchArgument(
            'uav_observation_topic',
            default_value='/perception/uav_01/observations',
        ),
        DeclareLaunchArgument(
            'usv_observation_topic',
            default_value='/perception/usv_01/observations',
        ),
        DeclareLaunchArgument(
            'lv_dot_observation_topic',
            default_value='/perception/lv_dot/observations',
        ),

        Node(
            package='uav_usv_perception',
            executable='ground_truth_adapter.py',
            name='ground_truth_adapter',
            output='screen',
            condition=IfCondition(start_ground_truth),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'pose_topic': pose_topic,
                'entity_name': target_entity,
                'output_topic': ground_truth_topic,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='perception_fusion_node.py',
            name='sensor_perception_fusion',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'input_topics_csv': [
                    uav_topic, ',', usv_topic, ',', lv_dot_topic
                ],
                'output_topic': sensor_topic,
                'target_frame': 'map',
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='perception_fusion_node.py',
            name='hybrid_perception_fusion',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'input_topics_csv': [
                    uav_topic, ',', usv_topic, ',', lv_dot_topic, ',',
                    ground_truth_topic
                ],
                'output_topic': hybrid_topic,
                'target_frame': 'map',
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='perception_source_mux.py',
            name='perception_source_mux',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'perception_source': source,
                'ground_truth_topic': ground_truth_topic,
                'sensor_topic': sensor_topic,
                'hybrid_topic': hybrid_topic,
                'output_topic': '/fleet/perception/targets',
            }],
        ),
    ])
