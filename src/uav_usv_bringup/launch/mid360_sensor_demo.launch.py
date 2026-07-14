import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rgl_actions(context):
    rgl_root = LaunchConfiguration('rgl_install').perform(context)
    rgl_patterns_dir = LaunchConfiguration('rgl_patterns').perform(context)
    rgl_plugin_dir = os.path.join(rgl_root, 'RGLServerPlugin')
    rgl_available = (
        os.path.exists(os.path.join(
            rgl_plugin_dir, 'libRGLServerPluginManager.so'
        ))
        and os.path.exists(os.path.join(
            rgl_plugin_dir, 'libRGLServerPluginInstance.so'
        ))
        and os.path.exists(os.path.join(
            rgl_patterns_dir, 'LivoxMid360.mat3x4f'
        ))
    )
    if not rgl_available:
        raise RuntimeError(
            'RGL Mid-360 libraries or LivoxMid360.mat3x4f were not found. '
            'Build RobotecAI/RGLGazeboPlugin and check rgl_install and '
            'rgl_patterns.'
        )
    gazebo_environment = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': (
            rgl_plugin_dir
            + ':'
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ),
        'RGL_PATTERNS_DIR': rgl_patterns_dir,
        'LD_LIBRARY_PATH': (
            rgl_plugin_dir
            + ':'
            + os.environ.get('LD_LIBRARY_PATH', '')
        ),
    }

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'uav_usv_gazebo', 'run_gz_world.sh',
                'mid360_sensor_demo',
            ],
            output='screen',
            additional_env=gazebo_environment,
            condition=IfCondition(LaunchConfiguration('start_gazebo')),
        ),
        Node(
            package='uav_usv_perception',
            executable='gz_pointcloud_bridge.py',
            name='mid360_pointcloud_bridge',
            output='screen',
            parameters=[{
                'gz_topic': '/usv_01/mid360_rgl/points',
                'ros_topic': '/usv_01/mid360/points',
                'frame_id': 'usv_01/mid360_link',
                'gz_clock_topic': '/clock',
            }],
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_usv_bringup')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'mid360_sensor_demo.rviz'
    )

    start_rviz = LaunchConfiguration('start_rviz')
    move_usv = LaunchConfiguration('move_usv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_gazebo',
            default_value='true',
            description='Start the standalone Mid-360 Gazebo world.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz with PointCloud2 and TF displays.',
        ),
        DeclareLaunchArgument(
            'move_usv',
            default_value='true',
            description='Move the demo USV on a slow arc.',
        ),
        DeclareLaunchArgument(
            'rgl_install',
            default_value='/var/tmp/RGLGazeboPlugin/install',
            description='RGLGazeboPlugin installation prefix.',
        ),
        DeclareLaunchArgument(
            'rgl_patterns',
            default_value='/var/tmp/RGLGazeboPlugin/lidar_patterns',
            description='Directory containing LivoxMid360.mat3x4f.',
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.8',
            description='Standalone demo USV speed in m/s.',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.035',
            description='Standalone demo USV yaw rate in rad/s.',
        ),
        OpaqueFunction(function=_rgl_actions),
        Node(
            package='uav_usv_sim',
            executable='maritime_tf_publisher',
            name='mid360_demo_tf',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'pose_topic': '/world/mid360_sensor_demo/pose/info',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'ownship_name': 'usv_01',
                'ownship_frame_id': 'usv_01/base_link',
                'target_name': '__none__',
                'target_frame_id': '__none__/base_link',
                'odom_topic': '/usv_01/odom',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mid360_mount_tf',
            output='screen',
            arguments=[
                '--x', '0.65', '--y', '0.0', '--z', '2.18',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'usv_01/base_link',
                '--child-frame-id', 'usv_01/mid360_link',
            ],
        ),
        Node(
            package='uav_usv_perception',
            executable='mid360_demo_motion.py',
            name='mid360_demo_motion',
            output='screen',
            parameters=[{
                'command_topic': '/model/usv_01/cmd_vel',
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
            }],
            condition=IfCondition(move_usv),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mid360_sensor_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(start_rviz),
        ),
    ])
