import os
import shlex
import shutil

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
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


def _keyboard_terminal_command(command):
    terminal = (
        shutil.which('x-terminal-emulator')
        or shutil.which('gnome-terminal')
        or shutil.which('konsole')
        or shutil.which('xfce4-terminal')
        or shutil.which('xterm')
    )

    if terminal is None:
        return None

    shell_command = f'{command}; exec bash'
    name = os.path.basename(terminal)

    if name == 'gnome-terminal':
        return [terminal, '--', 'bash', '-lc', shell_command]

    if name == 'konsole':
        return [terminal, '-e', 'bash', '-lc', shell_command]

    if name == 'xfce4-terminal':
        return [terminal, '--command', f'bash -lc {shlex.quote(shell_command)}']

    return [terminal, '-e', 'bash', '-lc', shell_command]


def _launch_setup(context, *args, **kwargs):
    package_name = 'uav_usv_sim'
    package_share = get_package_share_directory(package_name)
    package_prefix = get_package_prefix(package_name)
    plugin_dir = os.path.join(package_prefix, 'lib', package_name, 'plugins')
    models_dir = os.path.join(package_share, 'models')
    world_path = LaunchConfiguration('world').perform(context)
    gz_args = shlex.split(LaunchConfiguration('gz_args').perform(context))
    start_keyboard = _as_bool(LaunchConfiguration('start_keyboard').perform(context))
    keyboard_terminal = _as_bool(
        LaunchConfiguration('keyboard_terminal').perform(context)
    )
    keyboard_topic = LaunchConfiguration('keyboard_topic').perform(context)
    start_rviz = _as_bool(LaunchConfiguration('start_rviz').perform(context))
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    fuel_cache_path = os.path.expanduser(
        LaunchConfiguration('fuel_cache_path').perform(context)
    )
    asset_root = os.path.expanduser(
        LaunchConfiguration('asset_root').perform(context)
    )
    workspace_prefix = os.path.dirname(package_prefix)
    workspace_setup = os.path.join(workspace_prefix, 'setup.bash')
    prepare_script = os.path.join(
        package_prefix,
        'lib',
        package_name,
        'prepare_coastline.sh',
    )
    gz_command = ' '.join(
        shlex.quote(value) for value in ['gz', 'sim', *gz_args, world_path]
    )

    actions = [
        ExecuteProcess(
            cmd=[
                'bash',
                '-c',
                f'{shlex.quote(prepare_script)} && {gz_command}',
            ],
            output='screen',
            additional_env={
                'GZ_FUEL_CACHE_PATH': fuel_cache_path,
                'UAV_USV_ASSET_ROOT': asset_root,
                'GZ_SIM_RESOURCE_PATH': (
                    asset_root + ':' + models_dir + ':'
                    + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
                ),
                'GZ_SIM_SYSTEM_PLUGIN_PATH': (
                    plugin_dir + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
                ),
            },
        ),
        _boat_camera_bridge_action(),
    ]

    if not start_keyboard:
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

    keyboard_command = (
        'source /opt/ros/humble/setup.bash && '
        f'source {shlex.quote(workspace_setup)} && '
        'ros2 run uav_usv_sim keyboard_boat_control '
        f'--ros-args -p topic:={shlex.quote(keyboard_topic)}'
    )

    if keyboard_terminal:
        terminal_cmd = _keyboard_terminal_command(keyboard_command)
        if terminal_cmd is not None:
            actions.append(
                ExecuteProcess(
                    cmd=terminal_cmd,
                    output='screen',
                )
            )
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

        actions.append(
            LogInfo(
                msg='No terminal emulator found; running keyboard control in launch process.'
            )
        )

    actions.append(
        ExecuteProcess(
            cmd=shlex.split(keyboard_command),
            output='screen',
            emulate_tty=True,
        )
    )
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
    default_world = os.path.join(package_share, 'worlds', 'default.sdf')
    default_rviz_config = os.path.join(
        package_share,
        'rviz',
        'boat_front_camera.rviz',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=default_world,
                description='Gazebo world file to load.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value='-r',
                description='Extra arguments passed to gz sim.',
            ),
            DeclareLaunchArgument(
                'start_keyboard',
                default_value='true',
                description='Start keyboard boat control with the world.',
            ),
            DeclareLaunchArgument(
                'keyboard_terminal',
                default_value='true',
                description='Run keyboard control in a separate terminal window.',
            ),
            DeclareLaunchArgument(
                'keyboard_topic',
                default_value='/model/simple_boat/cmd_vel',
                description='Gazebo Twist topic used by the boat velocity controller.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
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
