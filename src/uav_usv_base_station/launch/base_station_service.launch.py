"""Launch the read-only Base Station Service."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('uav_usv_base_station'), 'config',
        'base_station.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_model_topic', default_value='/fleet/world_model'
        ),
        DeclareLaunchArgument('state_topic', default_value='/base_station/state'),
        DeclareLaunchArgument('events_topic', default_value='/base_station/events'),
        DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('target_history_length', default_value='120'),
        DeclareLaunchArgument('event_history_length', default_value='100'),
        Node(
            package='uav_usv_base_station',
            executable='base_station_service',
            name='base_station_service',
            output='screen',
            parameters=[config_file, {
                'world_model_topic': LaunchConfiguration('world_model_topic'),
                'state_topic': LaunchConfiguration('state_topic'),
                'events_topic': LaunchConfiguration('events_topic'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'target_history_length': LaunchConfiguration(
                    'target_history_length'
                ),
                'event_history_length': LaunchConfiguration(
                    'event_history_length'
                ),
            }],
        ),
    ])
