import os
import shlex

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString


UAV_CONFIG = (
    ('uav_01', 1, -86.86, -222.43, 19.75),
    ('uav_02', 2, -75.00, -215.00, 19.75),
    ('uav_03', 3, -63.14, -207.57, 19.75),
)
USV_CONFIG = (
    ('usv_01', 'own_01'),
    ('usv_02', 'own_02'),
    ('usv_03', 'own_03'),
)
USV_IDS = tuple(item[0] for item in USV_CONFIG)
WORLD_NAME = 'heterogeneous_332'
TARGET_ID = 'enemy_ship'


def _launch_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        '1', 'true', 'yes', 'on'
    )


def _fleet_runtime_actions(
    context,
    gazebo_share,
    gazebo_plugins,
    run_world,
    prepare_x500,
    prepare_mid360,
    world,
    gazebo_gui_config,
    standard_rviz_config,
    mid360_rviz_config,
):
    start_gazebo = _launch_bool(context, 'start_gazebo')
    start_rviz = _launch_bool(context, 'start_rviz')
    enable_mid360 = _launch_bool(context, 'enable_mid360')
    mid360_visualize = _launch_bool(context, 'mid360_visualize')
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    uav_model_scale = LaunchConfiguration('uav_model_scale').perform(context)
    uav_camera_rate = LaunchConfiguration('uav_camera_rate').perform(context)
    vehicle_id = LaunchConfiguration('mid360_vehicle_id').perform(context)
    ros_topic = LaunchConfiguration('mid360_topic').perform(context)
    update_rate = float(
        LaunchConfiguration('mid360_update_rate').perform(context)
    )
    max_range = float(
        LaunchConfiguration('mid360_range').perform(context)
    )
    min_range = float(
        LaunchConfiguration('mid360_min_range').perform(context)
    )
    voxel_size = float(
        LaunchConfiguration('mid360_voxel_size').perform(context)
    )
    rgl_root = LaunchConfiguration('rgl_install').perform(context)
    rgl_patterns = LaunchConfiguration('rgl_patterns').perform(context)
    rgl_plugin_dir = os.path.join(rgl_root, 'RGLServerPlugin')
    frame_id = vehicle_id + '/mid360_link'
    raw_topic = '/fleet/uplink/%s/mid360/rgl_points' % vehicle_id
    filtered_topic = '/perception/%s/mid360/points_filtered' % vehicle_id
    preview_topic = '/perception/%s/mid360/preview' % vehicle_id

    px4_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    px4_plugins = os.path.join(
        px4_dir,
        'build',
        'px4_sitl_default',
        'src',
        'modules',
        'simulation',
        'gz_plugins',
    )
    base_environment = {
        'GZ_SIM_RESOURCE_PATH': (
            gazebo_share + '/models:' + px4_models + ':'
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': (
            gazebo_plugins + ':' + px4_plugins + ':'
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ),
        'GZ_SIM_ARGS': '-r --gui-config ' + gazebo_gui_config,
    }
    actions = []
    if start_gazebo:
        prepare_uav = (
            '%s --px4-dir %s --scale %s '
            '--camera-width 320 --camera-height 180 --camera-rate %s'
            % (
                shlex.quote(prepare_x500),
                shlex.quote(px4_dir),
                shlex.quote(uav_model_scale),
                shlex.quote(uav_camera_rate),
            )
        )
        selected_world = world
        environment = dict(base_environment)
        if enable_mid360:
            required = (
                os.path.join(
                    rgl_plugin_dir, 'libRGLServerPluginManager.so'
                ),
                os.path.join(
                    rgl_plugin_dir, 'libRGLServerPluginInstance.so'
                ),
                os.path.join(rgl_patterns, 'LivoxMid360.mat3x4f'),
            )
            missing = [path for path in required if not os.path.isfile(path)]
            if missing:
                raise RuntimeError(
                    'RGL Mid-360 dependency missing: ' + ', '.join(missing)
                )
            output_root = '/var/tmp/UAV_USV_fleet_mid360'
            selected_world = os.path.join(
                output_root, 'worlds', os.path.basename(world)
            )
            prepare_sensor = (
                '%s --world %s --models-dir %s --output-root %s '
                '--vehicle-id %s --raw-topic %s --frame-id %s '
                '--update-rate %.6f --min-range %.6f --max-range %.6f'
                % (
                    shlex.quote(prepare_mid360),
                    shlex.quote(world),
                    shlex.quote(os.path.join(gazebo_share, 'models')),
                    shlex.quote(output_root),
                    shlex.quote(vehicle_id),
                    shlex.quote(raw_topic),
                    shlex.quote(frame_id),
                    update_rate,
                    min_range,
                    max_range,
                )
            )
            command = (
                'set -e; rm -rf %s; %s; %s; exec %s %s'
                % (
                    shlex.quote(output_root),
                    prepare_sensor,
                    prepare_uav,
                    shlex.quote(run_world),
                    shlex.quote(selected_world),
                )
            )
            environment['GZ_SIM_RESOURCE_PATH'] = (
                os.path.join(output_root, 'models') + ':'
                + environment['GZ_SIM_RESOURCE_PATH']
            )
            environment['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
                rgl_plugin_dir + ':'
                + environment['GZ_SIM_SYSTEM_PLUGIN_PATH']
            )
            environment['LD_LIBRARY_PATH'] = (
                rgl_plugin_dir + ':'
                + os.environ.get('LD_LIBRARY_PATH', '')
            )
            environment['RGL_PATTERNS_DIR'] = rgl_patterns
        else:
            command = (
                'set -e; %s; exec %s %s'
                % (
                    prepare_uav,
                    shlex.quote(run_world),
                    shlex.quote(selected_world),
                )
            )
        actions.append(ExecuteProcess(
            cmd=['bash', '-c', command],
            output='screen',
            additional_env=environment,
        ))

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fleet_usv_01_camera_mount_tf',
        output='screen',
        arguments=[
            '--x', '3.24', '--y', '0.0', '--z', '1.55',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'usv_01/base_link',
            '--child-frame-id', 'usv_01/camera_link',
        ],
    ))

    if enable_mid360:
        actions.extend([
            Node(
                package='uav_usv_perception',
                executable='gz_pointcloud_bridge.py',
                name='fleet_mid360_pointcloud_bridge',
                output='screen',
                parameters=[{
                    'gz_topic': raw_topic,
                    'ros_topic': ros_topic,
                    'frame_id': frame_id,
                    'publish_clock': False,
                    'stamp_mode': 'node',
                }],
            ),
            Node(
                package='uav_usv_perception',
                executable='mid360_preprocessor.py',
                name='fleet_mid360_preprocessor',
                output='screen',
                parameters=[{
                    'input_topic': ros_topic,
                    'output_topic': filtered_topic,
                    'preview_topic': preview_topic,
                    'vehicle_id': vehicle_id,
                    'frame_id': frame_id,
                    'expected_rate_hz': update_rate,
                    'min_range': min_range,
                    'max_range': max_range,
                    'voxel_size': voxel_size,
                    'preview_enabled': mid360_visualize,
                }],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fleet_mid360_mount_tf',
                output='screen',
                arguments=[
                    '--x', '0.9075', '--y', '0.0', '--z', '1.5625',
                    '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                    '--frame-id', vehicle_id + '/base_link',
                    '--child-frame-id', frame_id,
                ],
            ),
            Node(
                package='uav_usv_perception',
                executable='tf_topic_relay.py',
                name='fleet_mid360_tf_relay',
                output='screen',
                parameters=[{
                    'input_topic': '/%s/tf' % vehicle_id,
                    'output_topic': '/tf',
                }],
            ),
        ])

    if start_rviz:
        rviz_config = (
            mid360_rviz_config
            if enable_mid360 and mid360_visualize
            else standard_rviz_config
        )
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='fleet_dynamic_capture_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
        ))
    return actions


def _px4_command(px4_dir, px4_rcs, instance):
    return [
        'bash',
        '-c',
        [
            'set -e; PX4_ROOT=', px4_dir,
            '; BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"; '
            'ETC="$PX4_ROOT/build/px4_sitl_default/etc"; '
            'WORK=/var/tmp/UAV_USV_fleet_capture/px4_instance_%d; '
            'mkdir -p "$WORK"; '
            'rm -f "$WORK/parameters.bson" "$WORK/parameters_backup.bson"; '
            'exec "$BIN" -d -i %d -w "$WORK" -s '
            % (instance, instance),
            shlex.quote(px4_rcs),
            ' "$ETC"',
        ],
    ]


def _dds_agent_command(
    px4_ros_ws, executable, vehicle_id, system_id, home_x, home_y, home_z
):
    return [
        'bash',
        '-c',
        [
            'set -e; source ', px4_ros_ws,
            '/install/setup.bash; exec ', shlex.quote(executable),
            ' --ros-args -r __node:=%s_dds_agent '
            '-p use_sim_time:=false -p vehicle_id:=%s '
            '-p px4_namespace:=/%s -p px4_system_id:=%d '
            '-p home_x:=%.3f -p home_y:=%.3f -p home_z:=%.3f'
            % (
                vehicle_id,
                vehicle_id,
                vehicle_id,
                system_id,
                home_x,
                home_y,
                home_z,
            ),
        ],
    ]


def _nav_params(source_file, vehicle_id):
    return ReplaceString(
        source_file=source_file,
        replacements={
            'landing_boat/base_link': vehicle_id + '/base_link',
            'global_frame: odom': 'global_frame: ' + vehicle_id + '/odom',
            'odom_topic: /odom': 'odom_topic: odom',
            'topic: /boat/scan': 'topic: scan',
        },
    )


def _boat_interface(vehicle_id, model_control_name, use_sim_time, nav_params):
    return Node(
        package='uav_usv_sim',
        executable='boat_nav2_interface',
        namespace=vehicle_id,
        name='boat_nav2_interface',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[
            nav_params,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'boat_name': vehicle_id,
                'base_frame_id': vehicle_id + '/base_link',
                'odom_frame_id': vehicle_id + '/odom',
                'lidar_frame_id': vehicle_id + '/front_lidar',
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'model_pose_topic': '/unused/%s/pose' % vehicle_id,
                'boat_cmd_topic': '/model/%s/cmd_vel' % model_control_name,
                'cmd_vel_topic': 'cmd_vel',
                'odom_topic': 'odom',
                'map_topic': 'map',
                'scan_topic': 'scan_raw',
                'filtered_scan_topic': 'scan',
                'scan_range_topic': 'scan_range',
                'marker_topic': 'reference_markers',
                'publish_empty_map': True,
                'map_width': 1050.0,
                'map_height': 900.0,
                'enable_lidar_safety': False,
            },
        ],
    )


def _usv_agent(
    vehicle_id, model_control_name, use_sim_time, unreachable=False
):
    parameters = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'vehicle_id': vehicle_id,
        'odom_topic': '/%s/odom' % vehicle_id,
        'camera_topic': '/%s/camera' % vehicle_id,
        'scan_topic': '/%s/scan' % vehicle_id,
        'navigate_action': '/%s/navigate_to_pose' % vehicle_id,
        'emergency_cmd_topic': '/model/%s/cmd_vel' % model_control_name,
    }
    if unreachable is not False:
        parameters['simulate_unreachable'] = ParameterValue(
            unreachable, value_type=bool
        )
    return Node(
        package='uav_usv_usv_control',
        executable='usv_fleet_agent',
        name=vehicle_id + '_agent',
        output='screen',
        parameters=[parameters],
    )


def _sensor_bridges():
    return [Node(
        package='uav_usv_mission',
        executable='gz_sensor_bridge',
        name='fleet_capture_sensor_bridge',
        output='screen',
        parameters=[{
            'world_name': WORLD_NAME,
            'uav_ids': [item[0] for item in UAV_CONFIG],
            'uav_model_names': [item[0] for item in UAV_CONFIG],
            'usv_ids': list(USV_IDS),
            'usv_source_names': [item[1] for item in USV_CONFIG],
            'bridge_usv_scans': True,
            'bridge_base_radar': False,
        }],
    )]


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_usv_bringup')
    gazebo_share = get_package_share_directory('uav_usv_gazebo')
    gazebo_prefix = get_package_prefix('uav_usv_gazebo')
    sim_share = get_package_share_directory('uav_usv_sim')
    sim_prefix = get_package_prefix('uav_usv_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    uav_prefix = get_package_prefix('uav_usv_uav_control')

    px4_dir = LaunchConfiguration('px4_dir')
    px4_ros_ws = LaunchConfiguration('px4_ros_ws')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_px4 = LaunchConfiguration('start_px4')
    start_dds_agent = LaunchConfiguration('start_dds_agent')
    enable_sudden_turn = LaunchConfiguration('enable_sudden_turn')
    sudden_turn_time = LaunchConfiguration('sudden_turn_time')
    target_speed = LaunchConfiguration('target_speed')
    target_nominal_turn_rate = LaunchConfiguration(
        'target_nominal_turn_rate'
    )
    simulate_usv_02_unreachable = LaunchConfiguration(
        'simulate_usv_02_unreachable'
    )
    uav_model_scale = LaunchConfiguration('uav_model_scale')

    px4_dir_default = os.path.expanduser(
        os.environ.get('PX4_DIR', '~/PX4-Autopilot')
    )
    px4_ros_default = os.path.expanduser(
        os.environ.get('PX4_ROS_WS', '~/Desktop/Px4_ros')
    )
    px4_models = os.path.join(
        px4_dir_default, 'Tools', 'simulation', 'gz', 'models'
    )
    px4_plugins = os.path.join(
        px4_dir_default,
        'build',
        'px4_sitl_default',
        'src',
        'modules',
        'simulation',
        'gz_plugins',
    )
    gazebo_plugins = os.path.join(
        gazebo_prefix, 'lib', 'uav_usv_gazebo', 'plugins'
    )
    run_world = os.path.join(
        gazebo_prefix, 'lib', 'uav_usv_gazebo', 'run_gz_world.sh'
    )
    prepare_x500 = os.path.join(
        sim_prefix, 'lib', 'uav_usv_sim', 'prepare_large_x500.py'
    )
    prepare_mid360 = os.path.join(
        gazebo_prefix, 'lib', 'uav_usv_gazebo', 'prepare_fleet_mid360.py'
    )
    world = os.path.join(
        gazebo_share, 'worlds', 'heterogeneous_332.sdf'
    )
    standard_rviz_config = os.path.join(
        bringup_share, 'rviz', 'minimal_dynamic_capture.rviz'
    )
    mid360_rviz_config = os.path.join(
        bringup_share, 'rviz', 'fleet_dynamic_capture_mid360.rviz'
    )
    gazebo_gui_config = os.path.join(
        bringup_share, 'config', 'gazebo_white_gui.config'
    )
    px4_rcs = os.path.join(
        bringup_share, 'config', 'px4_minimal_capture.rcS'
    )
    nav_params = os.path.join(sim_share, 'config', 'boat_nav2_params.yaml')
    navigation_launch = os.path.join(
        nav2_share, 'launch', 'navigation_launch.py'
    )
    uav_agent = os.path.join(
        uav_prefix,
        'lib',
        'uav_usv_uav_control',
        'uav_dds_fleet_agent',
    )

    gazebo_env = {
        'GZ_SIM_RESOURCE_PATH': (
            gazebo_share + '/models:' + px4_models + ':'
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': (
            gazebo_plugins + ':' + px4_plugins + ':'
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ),
        'GZ_SIM_ARGS': '-r --gui-config ' + gazebo_gui_config,
    }

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=EnvironmentVariable(
                'PX4_DIR', default_value=px4_dir_default
            ),
        ),
        DeclareLaunchArgument(
            'px4_ros_ws',
            default_value=EnvironmentVariable(
                'PX4_ROS_WS', default_value=px4_ros_default
            ),
        ),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('start_dds_agent', default_value='true'),
        DeclareLaunchArgument('enable_sudden_turn', default_value='true'),
        DeclareLaunchArgument('sudden_turn_time', default_value='55.0'),
        DeclareLaunchArgument('target_speed', default_value='1.2'),
        DeclareLaunchArgument(
            'target_nominal_turn_rate', default_value='0.01'
        ),
        DeclareLaunchArgument('uav_model_scale', default_value='6.0'),
        DeclareLaunchArgument('uav_camera_rate', default_value='20.0'),
        DeclareLaunchArgument('enable_mid360', default_value='true'),
        DeclareLaunchArgument('mid360_vehicle_id', default_value='usv_01'),
        DeclareLaunchArgument(
            'mid360_topic',
            default_value='/fleet/uplink/usv_01/mid360/points',
        ),
        DeclareLaunchArgument('mid360_update_rate', default_value='10.0'),
        DeclareLaunchArgument('mid360_visualize', default_value='true'),
        DeclareLaunchArgument('mid360_min_range', default_value='0.5'),
        DeclareLaunchArgument('mid360_range', default_value='70.0'),
        DeclareLaunchArgument('mid360_voxel_size', default_value='0.12'),
        DeclareLaunchArgument(
            'rgl_install',
            default_value='/var/tmp/RGLGazeboPlugin/install',
        ),
        DeclareLaunchArgument(
            'rgl_patterns',
            default_value='/var/tmp/RGLGazeboPlugin/lidar_patterns',
        ),
        DeclareLaunchArgument(
            'simulate_usv_02_unreachable', default_value='false'
        ),
        OpaqueFunction(
            function=_fleet_runtime_actions,
            kwargs={
                'gazebo_share': gazebo_share,
                'gazebo_plugins': gazebo_plugins,
                'run_world': run_world,
                'prepare_x500': prepare_x500,
                'prepare_mid360': prepare_mid360,
                'world': world,
                'gazebo_gui_config': gazebo_gui_config,
                'standard_rviz_config': standard_rviz_config,
                'mid360_rviz_config': mid360_rviz_config,
            },
        ),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='log',
            condition=IfCondition(start_dds_agent),
        ),
    ]

    actions.extend(_sensor_bridges())

    for usv_index, (vehicle_id, model_control_name) in enumerate(USV_CONFIG):
        actions.append(_boat_interface(
            vehicle_id, model_control_name, use_sim_time, nav_params
        ))
        configured_nav_params = _nav_params(nav_params, vehicle_id)
        actions.append(TimerAction(
            period=2.0 + 3.0 * usv_index,
            actions=[GroupAction(actions=[
                PushRosNamespace(vehicle_id),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch),
                    launch_arguments={
                        'namespace': vehicle_id,
                        'use_sim_time': use_sim_time,
                        'params_file': configured_nav_params,
                        'autostart': 'true',
                        'use_composition': 'False',
                        'log_level': 'warn',
                    }.items(),
                ),
            ])],
        ))
    for vehicle_id, model_control_name in USV_CONFIG:
        unreachable = (
            simulate_usv_02_unreachable
            if vehicle_id == 'usv_02' else False
        )
        actions.append(_usv_agent(
            vehicle_id, model_control_name, use_sim_time, unreachable
        ))

    for instance, (
        vehicle_id, system_id, home_x, home_y, home_z
    ) in enumerate(
        UAV_CONFIG
    ):
        actions.append(ExecuteProcess(
            cmd=_dds_agent_command(
                px4_ros_ws,
                uav_agent,
                vehicle_id,
                system_id,
                home_x,
                home_y,
                home_z,
            ),
            output='screen',
            condition=IfCondition(start_dds_agent),
        ))
        px4_environment = dict(gazebo_env)
        px4_environment.update({
            'PX4_SIM_MODEL': 'gz_x500_mono_cam_down',
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': WORLD_NAME,
            'PX4_GZ_MODEL_NAME': vehicle_id,
            'PX4_UXRCE_DDS_NS': vehicle_id,
        })
        actions.append(TimerAction(
            period=8.0 + 2.0 * instance,
            actions=[ExecuteProcess(
                cmd=_px4_command(px4_dir, px4_rcs, instance),
                output='screen',
                additional_env=px4_environment,
                condition=IfCondition(start_px4),
            )],
        ))

    actions.extend([
        Node(
            package='uav_usv_mission',
            executable='target_tracker',
            name='target_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'track_id': TARGET_ID,
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='capture_manager',
            name='capture_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'uav_ids': [item[0] for item in UAV_CONFIG],
                'usv_ids': list(USV_IDS),
                'target_id': TARGET_ID,
                'uav_home_z': 19.75,
                'takeoff_altitude': 18.0,
                'observation_altitude': 42.0,
                'capture_radius': 28.0,
                'prediction_horizon': 16.0,
                'command_period': 4.0,
                'command_failure_threshold': 2,
                'encircle_tolerance': 42.0,
                'holding_tolerance': 24.0,
                'auto_start': False,
            }],
        ),
        Node(
            package='uav_usv_mission',
            executable='capture_visualizer',
            name='capture_visualizer',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'uav_visual_scale': ParameterValue(
                    uav_model_scale, value_type=float
                ),
            }],
        ),
    ])
    return LaunchDescription(actions)
