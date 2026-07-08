import os
import shlex

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def _boat_camera_bridge_action():
    try:
        get_package_prefix('ros_gz_bridge')
    except PackageNotFoundError:
        return LogInfo(
            msg=(
                'ros_gz_bridge is not installed; boat camera will only publish '
                'Gazebo topics. Install the matching ros-gz bridge package to '
                'view it in RViz.'
            )
        )

    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='boat_front_camera_bridge',
        output='screen',
        arguments=[
            '/boat/front_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/boat/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
    )


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _px4_dir_error(px4_dir):
    if not px4_dir or px4_dir == '/path/to/PX4-Autopilot' or px4_dir.startswith('/path/'):
        return (
            'PX4 path is not configured. Please pass your local PX4-Autopilot '
            'directory, for example: ros2 launch uav_usv_sim '
            'uav_usv_px4_sim.launch.py px4_dir:=/your/path/PX4-Autopilot'
        )

    if not os.path.isdir(px4_dir):
        return (
            f'PX4 directory does not exist: {px4_dir}. Please pass px4_dir:=... '
            'or export PX4_DIR to your local PX4-Autopilot path.'
        )

    if not os.path.isfile(os.path.join(px4_dir, 'Makefile')):
        return (
            f'PX4 directory does not look like a PX4-Autopilot source tree: '
            f'{px4_dir}. Missing Makefile.'
        )

    return None


def _launch_setup(context, *args, **kwargs):
    px4_dir = os.path.expanduser(LaunchConfiguration('px4_dir').perform(context))
    px4_model = LaunchConfiguration('px4_model').perform(context)
    model_pose = LaunchConfiguration('model_pose').perform(context)
    start_rviz = _as_bool(LaunchConfiguration('start_rviz').perform(context))
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    fuel_cache_path = os.path.expanduser(
        LaunchConfiguration('fuel_cache_path').perform(context)
    )
    asset_root = os.path.expanduser(
        LaunchConfiguration('asset_root').perform(context)
    )

    px4_error = _px4_dir_error(px4_dir)
    if px4_error is not None:
        return [LogInfo(msg=px4_error)]

    sync_script = os.path.join(
        get_package_prefix('uav_usv_sim'),
        'lib',
        'uav_usv_sim',
        'sync_to_px4.sh',
    )
    command = (
        f'{shlex.quote(sync_script)} && '
        f'cd {shlex.quote(px4_dir)} && '
        f'make px4_sitl {shlex.quote(px4_model)}'
    )

    actions = [
        ExecuteProcess(
            cmd=['bash', '-c', command],
            output='screen',
            additional_env={
                'PX4_DIR': px4_dir,
                'PX4_GZ_WORLD': 'default',
                'PX4_GZ_MODEL_POSE': model_pose,
                'GZ_IP': '127.0.0.1',
                'GZ_FUEL_CACHE_PATH': fuel_cache_path,
                'UAV_USV_ASSET_ROOT': asset_root,
                'GZ_SIM_RESOURCE_PATH': (
                    asset_root + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
                ),
                'HOME': os.environ.get('HOME', ''),
            },
        ),
        _boat_camera_bridge_action(),
    ]
    if start_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='uav_usv_rviz',
                output='screen',
                arguments=['-d', rviz_config],
            )
        )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory('uav_usv_sim')
    default_rviz_config = os.path.join(
        package_share,
        'rviz',
        'boat_front_camera.rviz',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'px4_dir',
                default_value=EnvironmentVariable(
                    'PX4_DIR',
                    default_value='/home/dji/PX4-Autopilot',
                ),
                description='PX4-Autopilot source directory.',
            ),
            DeclareLaunchArgument(
                'px4_model',
                default_value='gz_x500_mono_cam_down',
                description='PX4 Gazebo model target with a downward camera.',
            ),
            DeclareLaunchArgument(
                'model_pose',
                default_value='-16,0,3.55,0,0,0',
                description='PX4_GZ_MODEL_POSE used to spawn x500 on the shoreline helipad.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='false',
                description='Start RViz with the boat front camera display.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_rviz_config,
                description='RViz config used when start_rviz is true.',
            ),
            DeclareLaunchArgument(
                'fuel_cache_path',
                default_value='/var/tmp/UAV_USV_gz_fuel',
                description='Gazebo Fuel cache kept outside /home.',
            ),
            DeclareLaunchArgument(
                'asset_root',
                default_value='/var/tmp/UAV_USV_assets',
                description='Generated external simulation asset directory.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
