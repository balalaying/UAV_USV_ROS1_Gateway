import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory('uav_usv_gazebo')
    bringup_share = get_package_share_directory('uav_usv_bringup')
    gazebo_prefix = get_package_prefix('uav_usv_gazebo')
    run_world = os.path.join(
        gazebo_prefix, 'lib', 'uav_usv_gazebo', 'run_gz_world.sh'
    )
    world = os.path.join(
        gazebo_share, 'worlds', 'heterogeneous_332.sdf'
    )
    px4_dir = LaunchConfiguration('px4_dir')
    gui_args = LaunchConfiguration('gz_args')
    gui_config = os.path.join(
        bringup_share, 'config', 'gazebo_white_gui.config'
    )

    resource_path = [
        gazebo_share,
        '/models:',
        px4_dir,
        '/Tools/simulation/gz/models',
        ':',
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    ]
    plugin_path = [
        gazebo_prefix,
        '/lib/uav_usv_gazebo/plugins:',
        px4_dir,
        '/build/px4_sitl_default/src/modules/simulation/gz_plugins:',
        EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=EnvironmentVariable(
                'PX4_DIR', default_value=os.path.expanduser('~/PX4-Autopilot')
            ),
            description='PX4 source tree used only to resolve the x500 model.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r --gui-config ' + gui_config,
            description='Arguments passed through GZ_SIM_ARGS.',
        ),
        ExecuteProcess(
            cmd=[run_world, world],
            output='screen',
            additional_env={
                'GZ_SIM_ARGS': gui_args,
                'GZ_SIM_RESOURCE_PATH': resource_path,
                'GZ_SIM_SYSTEM_PLUGIN_PATH': plugin_path,
            },
        ),
    ])
