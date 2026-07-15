from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    vehicle_id = LaunchConfiguration('vehicle_id')
    points_topic = LaunchConfiguration('points_topic')
    output_frame = LaunchConfiguration('output_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    node_namespace = LaunchConfiguration('node_namespace')
    autostart = LaunchConfiguration('autostart')

    detector = LifecycleNode(
        package='uav_usv_lv_dot_ros2',
        executable='lv_dot_detector_node',
        name='lv_dot_detector_node',
        namespace=node_namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('uav_usv_lv_dot_ros2'),
                'config',
                'lv_dot_phase1.yaml',
            ]),
            {
                'vehicle_id': vehicle_id,
                'output_frame': output_frame,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[('points', points_topic)],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(detector),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(detector),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument('vehicle_id', default_value='usv_01'),
        DeclareLaunchArgument(
            'points_topic',
            default_value=['/perception/', vehicle_id, '/points_filtered'],
        ),
        DeclareLaunchArgument('output_frame', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'node_namespace', default_value='/perception/lv_dot_ros2'
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        detector,
        RegisterEventHandler(
            OnProcessStart(
                target_action=detector,
                on_start=[configure],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=detector,
                goal_state='inactive',
                entities=[activate],
            )
        ),
    ])
