"""Start the passive Qt console for the running fleet capture scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    demo_mode = LaunchConfiguration('demo_mode')
    return LaunchDescription([
        DeclareLaunchArgument('demo_mode', default_value='true'),
        Node(
            package='uav_usv_mission',
            executable='fleet_base_station',
            name='dynamic_capture_sensor_hub',
            output='screen',
            parameters=[{
                'auto_demo': False,
                'monitor_only': True,
                'owner_id': 'dynamic_capture_console',
                'uav_id': 'uav_01',
                'usv_id': 'usv_01',
                'uav_ids': 'uav_01,uav_02,uav_03,uav_04',
                'usv_ids': 'usv_01,usv_02',
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='fleet_base_station_gui',
            name='dynamic_capture_console',
            output='screen',
            parameters=[{
                'demo_mode': ParameterValue(demo_mode, value_type=bool),
                'capture_namespace': '',
                'defense_namespace': '',
            }],
        ),
    ])
