import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def camera_bridge_setup(context):
    model_name = LaunchConfiguration('uav_model_name').perform(context)
    prefix = (
        '/world/default/model/%s/link/camera_link/sensor/camera'
        % model_name
    )
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='base_uav_camera_bridge',
            output='screen',
            arguments=[
                prefix + '/image@sensor_msgs/msg/Image[gz.msgs.Image',
                (
                    prefix
                    + '/camera_info@sensor_msgs/msg/CameraInfo'
                    + '[gz.msgs.CameraInfo'
                ),
            ],
            remappings=[
                (prefix + '/image', '/uav/down_camera/image'),
                (prefix + '/camera_info', '/uav/down_camera/camera_info'),
            ],
        )
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_usv_bringup')
    sim_share = get_package_share_directory('uav_usv_sim')
    nav_launch = os.path.join(
        sim_share,
        'launch',
        'boat_nav2_navigation.launch.py',
    )
    rviz_config = os.path.join(
        bringup_share,
        'rviz',
        'Qt_base_station.rviz',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation time.',
            ),
            DeclareLaunchArgument(
                'start_nav2',
                default_value='true',
                description='Start the boat Nav2 stack.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='false',
                description='Open the fleet base-station RViz view.',
            ),
            DeclareLaunchArgument(
                'start_gui',
                default_value='true',
                description='Open the Qt fleet base-station console.',
            ),
            DeclareLaunchArgument(
                'start_simulated_fleet',
                default_value='false',
                description='Publish auxiliary fleet members 02 and 03.',
            ),
            DeclareLaunchArgument(
                'auto_demo',
                default_value='true',
                description='Automatically command UAV and USV to target.',
            ),
            DeclareLaunchArgument(
                'target_x',
                default_value='24.0',
                description='Cooperative target X in map frame.',
            ),
            DeclareLaunchArgument(
                'target_y',
                default_value='8.0',
                description='Cooperative target Y in map frame.',
            ),
            DeclareLaunchArgument(
                'uav_altitude',
                default_value='16.0',
                description='UAV takeoff and target altitude.',
            ),
            DeclareLaunchArgument(
                'uav_model_name',
                default_value='x500_mono_cam_down_0',
                description='Gazebo UAV entity name.',
            ),
            DeclareLaunchArgument(
                'mavlink_url',
                default_value='udp:127.0.0.1:14540',
                description='PX4 MAVLink endpoint.',
            ),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'start_rviz': 'false',
                        }.items(),
                    )
                ],
                scoped=True,
                condition=IfCondition(LaunchConfiguration('start_nav2')),
            ),
            OpaqueFunction(function=camera_bridge_setup),
            Node(
                package='uav_usv_uav_control',
                executable='uav_fleet_agent',
                name='uav_fleet_agent',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        'drone_name': LaunchConfiguration('uav_model_name'),
                        'mavlink_url': LaunchConfiguration('mavlink_url'),
                    }
                ],
            ),
            Node(
                package='uav_usv_usv_control',
                executable='usv_fleet_agent',
                name='usv_fleet_agent',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package='uav_usv_mission',
                executable='fleet_base_station',
                name='fleet_base_station',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        'auto_demo': ParameterValue(
                            LaunchConfiguration('auto_demo'),
                            value_type=bool,
                        ),
                        'target_x': ParameterValue(
                            LaunchConfiguration('target_x'),
                            value_type=float,
                        ),
                        'target_y': ParameterValue(
                            LaunchConfiguration('target_y'),
                            value_type=float,
                        ),
                        'uav_altitude': ParameterValue(
                            LaunchConfiguration('uav_altitude'),
                            value_type=float,
                        ),
                        'uav_ids': 'uav_01,uav_02,uav_03',
                        'usv_ids': 'usv_01,usv_02,usv_03',
                    }
                ],
            ),
            Node(
                package='uav_usv_mission',
                executable='fleet_simulated_agent',
                name='fleet_simulated_agent',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
                condition=IfCondition(
                    LaunchConfiguration('start_simulated_fleet')
                ),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='fleet_base_station_rviz',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
            Node(
                package='uav_usv_mission',
                executable='fleet_base_station_gui',
                name='fleet_base_station_gui',
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_gui')),
            ),
        ]
    )
