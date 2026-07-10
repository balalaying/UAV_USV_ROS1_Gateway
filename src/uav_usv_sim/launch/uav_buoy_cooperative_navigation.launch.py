import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            name='uav_down_camera_bridge',
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
    package_share = get_package_share_directory('uav_usv_sim')
    nav_launch = os.path.join(
        package_share,
        'launch',
        'boat_nav2_navigation.launch.py',
    )
    rviz_config = os.path.join(
        package_share,
        'rviz',
        'uav_buoy_cooperative_navigation.rviz',
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
                'uav_model_name',
                default_value='x500_mono_cam_down_0',
                description='Gazebo entity name of the camera-equipped UAV.',
            ),
            DeclareLaunchArgument(
                'mavlink_url',
                default_value='udp:127.0.0.1:14540',
                description='PX4 MAVLink endpoint.',
            ),
            DeclareLaunchArgument(
                'takeoff_altitude',
                default_value='16.0',
                description='UAV visual patrol altitude in metres.',
            ),
            DeclareLaunchArgument(
                'patrol_speed',
                default_value='3.0',
                description='UAV patrol speed in m/s.',
            ),
            DeclareLaunchArgument(
                'target_speed',
                default_value='3.2',
                description='UAV speed after target detection in m/s.',
            ),
            DeclareLaunchArgument(
                'boat_standoff_distance',
                default_value='7.0',
                description='Safe Nav2 goal distance from the buoy.',
            ),
            DeclareLaunchArgument(
                'image_processing_rate',
                default_value='24.0',
                description='Detection and camera mosaic rate in FPS.',
            ),
            DeclareLaunchArgument(
                'start_nav2',
                default_value='true',
                description='Start the USV Nav2 stack for cooperative tracking.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Start RViz with maps, Nav2, and camera mosaic.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'start_rviz': 'false',
                }.items(),
                condition=IfCondition(LaunchConfiguration('start_nav2')),
            ),
            OpaqueFunction(function=camera_bridge_setup),
            Node(
                package='uav_usv_sim',
                executable='uav_buoy_visual_mission',
                name='uav_buoy_visual_mission',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                        'mavlink_url': LaunchConfiguration('mavlink_url'),
                        'drone_name': LaunchConfiguration('uav_model_name'),
                        'takeoff_altitude': ParameterValue(
                            LaunchConfiguration('takeoff_altitude'),
                            value_type=float,
                        ),
                        'patrol_speed': ParameterValue(
                            LaunchConfiguration('patrol_speed'),
                            value_type=float,
                        ),
                        'target_speed': ParameterValue(
                            LaunchConfiguration('target_speed'),
                            value_type=float,
                        ),
                        'boat_standoff_distance': ParameterValue(
                            LaunchConfiguration('boat_standoff_distance'),
                            value_type=float,
                        ),
                        'image_processing_rate': ParameterValue(
                            LaunchConfiguration('image_processing_rate'),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='uav_usv_visual_rviz',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
        ]
    )
