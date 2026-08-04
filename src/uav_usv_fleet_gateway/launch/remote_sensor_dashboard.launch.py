from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('uav_usv_fleet_gateway')
    config = os.path.join(share, 'config', 'fleet_gateway.yaml')
    web_root = os.path.join(share, 'web')
    use_sim_time = LaunchConfiguration('use_sim_time')
    relay_host = LaunchConfiguration('relay_host')
    relay_ws_port = LaunchConfiguration('relay_websocket_port')
    relay_http_port = LaunchConfiguration('relay_http_port')
    relay_token = LaunchConfiguration('relay_token')
    relay_url = LaunchConfiguration('relay_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Must match the main simulation; the 332 live world uses '
                'wall time by default.'
            ),
        ),
        DeclareLaunchArgument('start_local_relay', default_value='true'),
        DeclareLaunchArgument('relay_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('relay_websocket_port', default_value='9765'),
        DeclareLaunchArgument('relay_http_port', default_value='9080'),
        DeclareLaunchArgument('relay_token', default_value=''),
        DeclareLaunchArgument(
            'relay_url', default_value='ws://127.0.0.1:9765/uplink'),
        DeclareLaunchArgument('camera_rate_hz', default_value='8.0'),
        DeclareLaunchArgument('pointcloud_rate_hz', default_value='8.0'),
        DeclareLaunchArgument('pointcloud_max_points', default_value='7000'),
        # Keep the full useful Mid-360 elevation band.  Sea-surface cleanup is
        # handled by the perception preprocessor; this web-only projection must
        # not remove low hull returns from the target vessel.
        DeclareLaunchArgument('pointcloud_display_min_z', default_value='-1.0'),
        DeclareLaunchArgument('pointcloud_display_max_range', default_value='45.0'),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('start_local_relay')),
            cmd=[
                'python3', '-m',
                'uav_usv_fleet_gateway.remote_relay',
                '--bind-address', relay_host,
                '--websocket-port', relay_ws_port,
                '--http-address', relay_host,
                '--http-port', relay_http_port,
                '--web-root', web_root,
                '--token', relay_token,
            ],
            output='screen',
        ),
        Node(
            package='uav_usv_fleet_gateway',
            executable='fleet_gateway',
            name='remote_demo_fleet_gateway',
            output='screen',
            parameters=[config, {
                'use_sim_time': use_sim_time,
                'bind_address': '127.0.0.1',
                'websocket_port': 8765,
                'enable_http_server': False,
                'sensor_stream_publish_rate_hz': 20.0,
            }],
        ),
        Node(
            package='uav_usv_fleet_gateway',
            executable='fleet_sensor_stream_adapter',
            name='remote_demo_sensor_stream_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_rate_hz': LaunchConfiguration('camera_rate_hz'),
                'pointcloud_rate_hz': LaunchConfiguration(
                    'pointcloud_rate_hz'),
                'pointcloud_max_points': LaunchConfiguration(
                    'pointcloud_max_points'),
            }],
        ),
        *[
            Node(
                package='uav_usv_perception',
                executable='qt_pointcloud_projection_node.py',
                name=vehicle_id + '_web_pointcloud_projection',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic': (
                        '/perception/%s/mid360/points_filtered' % vehicle_id
                    ),
                    'output_topic': (
                        '/perception/visualization/%s/topdown_points'
                        % vehicle_id
                    ),
                    'status_topic': (
                        '/perception/visualization/%s/web_topdown_status'
                        % vehicle_id
                    ),
                    'output_frame': 'map',
                    'pointcloud_display_rate_hz': LaunchConfiguration(
                        'pointcloud_rate_hz'),
                    'pointcloud_max_points': LaunchConfiguration(
                        'pointcloud_max_points'),
                    # Web-only cleanup. The LV-DOT input remains untouched.
                    'pointcloud_min_z': LaunchConfiguration(
                        'pointcloud_display_min_z'),
                    'pointcloud_max_range': LaunchConfiguration(
                        'pointcloud_display_max_range'),
                    'pointcloud_voxel_size': 0.25,
                    'pointcloud_persistence_frames': 1,
                }],
            )
            for vehicle_id in ('usv_01', 'usv_02', 'usv_03')
        ],
        TimerAction(
            period=1.5,
            actions=[ExecuteProcess(
                cmd=[
                    'python3', '-m',
                    'uav_usv_fleet_gateway.remote_uplink',
                    '--source-url', 'ws://127.0.0.1:8765/ws',
                    '--relay-url', relay_url,
                ],
                output='screen',
            )],
        ),
    ])
