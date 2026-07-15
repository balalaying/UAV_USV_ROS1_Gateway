from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_adapter = LaunchConfiguration('start_lv_dot_adapter')
    start_ingress = LaunchConfiguration('start_lv_dot_ingress')
    start_egress = LaunchConfiguration('start_lv_dot_egress')
    start_pose_adapter = LaunchConfiguration('start_lv_dot_pose_adapter')
    start_evaluator = LaunchConfiguration('start_shadow_evaluator')
    bbox_topic = LaunchConfiguration('lv_dot_bbox_topic')
    velocity_topic = LaunchConfiguration('lv_dot_velocity_topic')
    observation_topic = LaunchConfiguration('lv_dot_observation_topic')
    ground_truth_topic = LaunchConfiguration('ground_truth_topic')
    metrics_topic = LaunchConfiguration('shadow_metrics_topic')
    target_id = LaunchConfiguration('target_id')
    ingress_port = LaunchConfiguration('lv_dot_ingress_port')
    egress_port = LaunchConfiguration('lv_dot_egress_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lv_dot_adapter', default_value='true'),
        DeclareLaunchArgument('start_lv_dot_ingress', default_value='true'),
        DeclareLaunchArgument('start_lv_dot_egress', default_value='true'),
        DeclareLaunchArgument(
            'start_lv_dot_pose_adapter', default_value='true'
        ),
        DeclareLaunchArgument('start_shadow_evaluator', default_value='true'),
        DeclareLaunchArgument(
            'lv_dot_bbox_topic',
            default_value='/lv_dot/onboard_detector/dynamic_bboxes',
        ),
        DeclareLaunchArgument(
            'lv_dot_velocity_topic',
            default_value=(
                '/lv_dot/onboard_detector/velocity_visualizaton'
            ),
        ),
        DeclareLaunchArgument(
            'lv_dot_observation_topic',
            default_value='/perception/lv_dot/observations',
        ),
        DeclareLaunchArgument(
            'ground_truth_topic',
            default_value='/perception/ground_truth/tracks',
        ),
        DeclareLaunchArgument(
            'shadow_metrics_topic',
            default_value='/perception/lv_dot/shadow_metrics',
        ),
        DeclareLaunchArgument('target_id', default_value='target_vessel'),
        DeclareLaunchArgument('lv_dot_ingress_port', default_value='19090'),
        DeclareLaunchArgument('lv_dot_egress_port', default_value='19091'),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_egress_relay.py',
            name='lv_dot_egress_relay',
            output='screen',
            condition=IfCondition(start_egress),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'port': ParameterValue(egress_port, value_type=int),
                'bbox_topic': bbox_topic,
                'velocity_topic': velocity_topic,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_ingress_relay.py',
            name='lv_dot_ingress_relay',
            output='screen',
            condition=IfCondition(start_ingress),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'pointcloud_topic': '/perception/usv_01/points_filtered',
                'pose_topic': '/perception/lv_dot/usv_01/pose',
                'image_topic': (
                    '/fleet/uplink/uav_01/camera/image_raw'
                ),
                'camera_info_topic': (
                    '/fleet/uplink/uav_01/camera/camera_info'
                ),
                'port': ParameterValue(ingress_port, value_type=int),
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_pose_adapter.py',
            name='lv_dot_pose_adapter',
            output='screen',
            condition=IfCondition(start_pose_adapter),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'vehicle_id': 'usv_01',
                'input_topic': '/fleet/state',
                'output_topic': '/perception/lv_dot/usv_01/pose',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_adapter.py',
            name='lv_dot_adapter',
            output='screen',
            condition=IfCondition(start_adapter),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'bbox_topic': bbox_topic,
                'velocity_topic': velocity_topic,
                'output_topic': observation_topic,
                'target_frame': 'map',
                # The isolated backend currently runs the native LiDAR path.
                # Set to 11 only after camera fusion is genuinely enabled.
                'source_mask': 1,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_shadow_evaluator.py',
            name='lv_dot_shadow_evaluator',
            output='screen',
            condition=IfCondition(start_evaluator),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'ground_truth_topic': ground_truth_topic,
                'observation_topic': observation_topic,
                'metrics_topic': metrics_topic,
                'target_id': target_id,
            }],
        ),
    ])
