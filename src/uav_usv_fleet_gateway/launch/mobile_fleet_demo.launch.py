from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('uav_usv_fleet_gateway')
    config = os.path.join(share, 'config', 'fleet_gateway.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bind_address = LaunchConfiguration('bind_address')
    websocket_port = LaunchConfiguration('websocket_port')
    http_port = LaunchConfiguration('http_port')
    enable_websocket = LaunchConfiguration('enable_websocket')
    enable_http_server = LaunchConfiguration('enable_http_server')
    world_model_topic = LaunchConfiguration('world_model_topic')
    world_model_summary_topic = LaunchConfiguration('world_model_summary_topic')
    enable_world_model_publish = LaunchConfiguration(
        'enable_world_model_publish')
    world_model_publish_rate_hz = LaunchConfiguration(
        'world_model_publish_rate_hz')
    enable_world_model_summary_publish = LaunchConfiguration(
        'enable_world_model_summary_publish')
    world_model_summary_publish_rate_hz = LaunchConfiguration(
        'world_model_summary_publish_rate_hz')
    include_world_model_in_snapshot = LaunchConfiguration(
        'include_world_model_in_snapshot')
    enable_legacy_topic_fallback = LaunchConfiguration(
        'enable_legacy_topic_fallback')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('websocket_port', default_value='8765'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('enable_websocket', default_value='true'),
        DeclareLaunchArgument('enable_http_server', default_value='true'),
        DeclareLaunchArgument(
            'world_model_topic', default_value='/fleet/world_model'),
        DeclareLaunchArgument(
            'world_model_summary_topic',
            default_value='/fleet/world_model_summary'),
        DeclareLaunchArgument(
            'enable_world_model_publish', default_value='true'),
        DeclareLaunchArgument(
            'world_model_publish_rate_hz', default_value='2.0'),
        DeclareLaunchArgument(
            'enable_world_model_summary_publish', default_value='true'),
        DeclareLaunchArgument(
            'world_model_summary_publish_rate_hz', default_value='1.0'),
        DeclareLaunchArgument(
            'include_world_model_in_snapshot', default_value='true'),
        DeclareLaunchArgument(
            'enable_legacy_topic_fallback', default_value='false'),
        Node(
            package='uav_usv_fleet_gateway',
            executable='fleet_gateway',
            name='fleet_gateway',
            output='screen',
            parameters=[config, {
                'use_sim_time': use_sim_time,
                'bind_address': bind_address,
                'websocket_port': websocket_port,
                'http_port': http_port,
                'enable_websocket': enable_websocket,
                'enable_http_server': enable_http_server,
                'world_model_topic': world_model_topic,
                'world_model_summary_topic': world_model_summary_topic,
                'enable_world_model_publish': enable_world_model_publish,
                'world_model_publish_rate_hz': world_model_publish_rate_hz,
                'enable_world_model_summary_publish': (
                    enable_world_model_summary_publish
                ),
                'world_model_summary_publish_rate_hz': (
                    world_model_summary_publish_rate_hz
                ),
                'include_world_model_in_snapshot': (
                    include_world_model_in_snapshot
                ),
                'enable_legacy_topic_fallback': enable_legacy_topic_fallback,
            }],
        ),
    ])
