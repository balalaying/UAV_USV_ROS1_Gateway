import os
import shlex
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _clock_bridge_action():
    try:
        get_package_prefix('ros_gz_bridge')
    except PackageNotFoundError:
        return LogInfo(msg='ros_gz_bridge is unavailable; /clock bridge skipped.')
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='colregs_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _load_scenario(package_share, scenario_name):
    config_path = os.path.join(
        package_share,
        'config',
        'colregs_scenarios.yaml',
    )
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    scenarios = config.get('scenarios', {})
    if scenario_name not in scenarios:
        raise RuntimeError(
            'Unknown scenario "%s". Available: %s'
            % (scenario_name, ', '.join(sorted(scenarios)))
        )
    return scenarios[scenario_name], config.get('ais', {})


def _generate_world(package_share, scenario_name, scenario):
    source_world = os.path.join(package_share, 'worlds', 'default.sdf')
    with open(source_world, encoding='utf-8') as stream:
        world_text = stream.read()

    pose = ' '.join(str(value) for value in scenario['target_pose'])
    target_include = """
    <include>
      <uri>model://target_vessel</uri>
      <name>target_vessel</name>
      <pose>%s</pose>
    </include>
""" % pose
    world_text = world_text.replace(
        '  </world>',
        target_include + '  </world>',
        1,
    )
    output_path = os.path.join(
        tempfile.gettempdir(),
        'uav_usv_colregs_%s_%d.sdf' % (scenario_name, os.getpid()),
    )
    with open(output_path, 'w', encoding='utf-8') as stream:
        stream.write(world_text)
    return output_path


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('uav_usv_sim')
    package_prefix = get_package_prefix('uav_usv_sim')
    scenario_name = LaunchConfiguration('scenario').perform(context)
    auto_ownship = _as_bool(
        LaunchConfiguration('auto_ownship').perform(context)
    )
    start_rviz = _as_bool(LaunchConfiguration('start_rviz').perform(context))
    scenario, ais = _load_scenario(package_share, scenario_name)
    generated_world = _generate_world(
        package_share,
        scenario_name,
        scenario,
    )

    models_dir = os.path.join(package_share, 'models')
    plugin_dir = os.path.join(
        package_prefix,
        'lib',
        'uav_usv_sim',
        'plugins',
    )
    urdf_file = os.path.join(
        package_share,
        'urdf',
        'landing_boat.urdf.xacro',
    )
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    actions = [
        LogInfo(
            msg='Starting COLREGs scenario "%s": %s'
            % (scenario_name, scenario['description'])
        ),
        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                *shlex.split(LaunchConfiguration('gz_args').perform(context)),
                generated_world,
            ],
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': (
                    models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
                ),
                'GZ_SIM_SYSTEM_PLUGIN_PATH': (
                    plugin_dir
                    + ':'
                    + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
                ),
            },
        ),
        _clock_bridge_action(),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='landing_boat_robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'frame_prefix': 'landing_boat/',
                    'robot_description': ParameterValue(
                        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
                        value_type=str,
                    ),
                }
            ],
        ),
        Node(
            package='uav_usv_sim',
            executable='maritime_tf_publisher',
            name='maritime_tf_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='uav_usv_sim',
            executable='colregs_scenario_controller',
            name='colregs_scenario_controller',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'scenario_name': scenario_name,
                    'auto_ownship': auto_ownship,
                    'ownship_speed': float(scenario['ownship_speed']),
                    'ownship_heading': float(scenario['ownship_heading']),
                    'target_speed': float(scenario['target_speed']),
                    'target_heading': float(scenario['target_heading']),
                }
            ],
        ),
        Node(
            package='uav_usv_sim',
            executable='ais_simulator',
            name='ais_simulator',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'mmsi': int(ais.get('mmsi', 413000001)),
                    'vessel_name': str(
                        ais.get('vessel_name', 'SIM_TARGET_01')
                    ),
                    'update_period': float(ais.get('update_period', 2.0)),
                    'latency': float(ais.get('latency', 0.5)),
                    'dropout_probability': float(
                        ais.get('dropout_probability', 0.03)
                    ),
                    'position_noise_std': float(
                        ais.get('position_noise_std', 0.8)
                    ),
                    'speed_noise_std': float(
                        ais.get('speed_noise_std', 0.05)
                    ),
                    'heading_noise_std': float(
                        ais.get('heading_noise_std', 0.015)
                    ),
                }
            ],
        ),
    ]
    if start_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='colregs_test_rviz',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
            )
        )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory('uav_usv_sim')
    default_rviz = os.path.join(
        package_share,
        'rviz',
        'colregs_test_scenario.rviz',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'scenario',
                default_value='head_on',
                description=(
                    'Standard encounter: head_on, crossing_starboard, '
                    'or overtaking.'
                ),
            ),
            DeclareLaunchArgument(
                'auto_ownship',
                default_value='true',
                description=(
                    'Drive ownship with its existing Gazebo Twist interface. '
                    'Set false to control it with keyboard or another node.'
                ),
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Start RViz with TF and AIS track displays.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_rviz,
                description='RViz configuration for standard test scenes.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value='-r',
                description='Extra arguments passed to gz sim.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
