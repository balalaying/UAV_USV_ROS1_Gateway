import os
import shlex

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORLD_NAME = 'lv_dot_tuning'


def _runtime(context, paths):
    world = paths['world']
    output_root = '/tmp/UAV_USV_lv_dot_tuning'
    selected_world = os.path.join(
        output_root, 'worlds', os.path.basename(world)
    )
    raw_topic = '/fleet/uplink/usv_01/mid360/rgl_points'
    frame_id = 'usv_01/mid360_link'
    update_rate = float(
        LaunchConfiguration('mid360_update_rate').perform(context)
    )
    min_range = float(
        LaunchConfiguration('mid360_min_range').perform(context)
    )
    max_range = float(
        LaunchConfiguration('mid360_range').perform(context)
    )
    voxel_size = float(
        LaunchConfiguration('mid360_voxel_size').perform(context)
    )
    min_z = float(LaunchConfiguration('mid360_min_z').perform(context))
    max_z = float(LaunchConfiguration('mid360_max_z').perform(context))

    required = (
        os.path.join(
            paths['rgl_plugin_dir'], 'libRGLServerPluginManager.so'
        ),
        os.path.join(
            paths['rgl_plugin_dir'], 'libRGLServerPluginInstance.so'
        ),
        os.path.join(paths['rgl_patterns'], 'LivoxMid360.mat3x4f'),
    )
    missing = [item for item in required if not os.path.isfile(item)]
    if missing:
        raise RuntimeError(
            'RGL Mid-360 dependency missing: ' + ', '.join(missing)
        )

    prepare_sensor = (
        '%s --world %s --models-dir %s --output-root %s '
        '--vehicle-id usv_01 --raw-topic %s --frame-id %s '
        '--update-rate %.6f --min-range %.6f --max-range %.6f'
        % (
            shlex.quote(paths['prepare_mid360']),
            shlex.quote(world),
            shlex.quote(paths['models_dir']),
            shlex.quote(output_root),
            shlex.quote(raw_topic),
            shlex.quote(frame_id),
            update_rate,
            min_range,
            max_range,
        )
    )
    command = (
        'set -e; rm -rf %s; %s; exec %s %s'
        % (
            shlex.quote(output_root),
            prepare_sensor,
            shlex.quote(paths['run_world']),
            shlex.quote(selected_world),
        )
    )
    environment = {
        'GZ_SIM_RESOURCE_PATH': (
            os.path.join(output_root, 'models') + ':'
            + paths['models_dir'] + ':' + paths['px4_models'] + ':'
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': (
            paths['rgl_plugin_dir'] + ':' + paths['gazebo_plugins'] + ':'
            + paths['px4_plugins'] + ':'
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ),
        'LD_LIBRARY_PATH': (
            paths['rgl_plugin_dir'] + ':'
            + os.environ.get('LD_LIBRARY_PATH', '')
        ),
        'RGL_PATTERNS_DIR': paths['rgl_patterns'],
        'GZ_SIM_ARGS': '-r',
    }

    actions = []
    if LaunchConfiguration('start_gazebo').perform(
        context
    ).lower() in ('true', '1', 'yes', 'on'):
        actions.append(ExecuteProcess(
            cmd=['bash', '-c', command],
            output='screen',
            additional_env=environment,
        ))

    actions.extend([
        Node(
            package='uav_usv_perception',
            executable='gz_pointcloud_bridge.py',
            name='lv_dot_tuning_mid360_bridge',
            output='screen',
            parameters=[{
                'gz_topic': raw_topic,
                'ros_topic': '/fleet/uplink/usv_01/mid360/points',
                'frame_id': frame_id,
                'publish_clock': False,
                'stamp_mode': 'node',
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='mid360_preprocessor.py',
            name='lv_dot_tuning_mid360_preprocessor',
            output='screen',
            parameters=[{
                'input_topic': '/fleet/uplink/usv_01/mid360/points',
                'output_topic': '/perception/usv_01/points_filtered',
                'preview_topic': '/perception/usv_01/mid360/preview',
                'vehicle_id': 'usv_01',
                'frame_id': frame_id,
                'tf_target_frame': 'map',
                'expected_rate_hz': update_rate,
                'min_range': min_range,
                'max_range': max_range,
                'voxel_size': voxel_size,
                'min_z': min_z,
                'max_z': max_z,
                'preview_enabled': True,
            }],
        ),
    ])
    return actions


def generate_launch_description():
    gazebo_share = get_package_share_directory('uav_usv_gazebo')
    gazebo_prefix = get_package_prefix('uav_usv_gazebo')
    perception_share = get_package_share_directory('uav_usv_perception')
    px4_dir = os.path.expanduser(
        os.environ.get('PX4_DIR', '~/PX4-Autopilot')
    )
    rgl_root = os.environ.get(
        'RGL_INSTALL', '/var/tmp/RGLGazeboPlugin/install'
    )
    rgl_patterns = os.environ.get(
        'RGL_PATTERNS', '/var/tmp/RGLGazeboPlugin/lidar_patterns'
    )
    paths = {
        'world': os.path.join(
            gazebo_share, 'worlds', 'lv_dot_tuning.sdf'
        ),
        'models_dir': os.path.join(gazebo_share, 'models'),
        'run_world': os.path.join(
            gazebo_prefix, 'lib', 'uav_usv_gazebo', 'run_gz_world.sh'
        ),
        'prepare_mid360': os.path.join(
            gazebo_prefix,
            'lib',
            'uav_usv_gazebo',
            'prepare_fleet_mid360.py',
        ),
        'px4_dir': px4_dir,
        'px4_models': os.path.join(
            px4_dir, 'Tools', 'simulation', 'gz', 'models'
        ),
        'px4_plugins': os.path.join(
            px4_dir,
            'build',
            'px4_sitl_default',
            'src',
            'modules',
            'simulation',
            'gz_plugins',
        ),
        'gazebo_plugins': os.path.join(
            gazebo_prefix, 'lib', 'uav_usv_gazebo', 'plugins'
        ),
        'rgl_plugin_dir': os.path.join(rgl_root, 'RGLServerPlugin'),
        'rgl_patterns': rgl_patterns,
    }
    use_sim_time = LaunchConfiguration('use_sim_time')
    profile = LaunchConfiguration('target_profile')
    perception_layer = os.path.join(
        perception_share, 'launch', 'perception_layer.launch.py'
    )
    shadow_launch = os.path.join(
        perception_share, 'launch', 'lv_dot_shadow.launch.py'
    )
    rviz_config = os.path.join(
        perception_share, 'rviz', 'lv_dot_tuning.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_qt', default_value='false'),
        DeclareLaunchArgument(
            'target_profile',
            default_value='constant',
            description='constant, turn, or acceleration',
        ),
        DeclareLaunchArgument('mid360_update_rate', default_value='10.0'),
        DeclareLaunchArgument('mid360_min_range', default_value='0.5'),
        DeclareLaunchArgument('mid360_range', default_value='20.0'),
        DeclareLaunchArgument('mid360_voxel_size', default_value='0.04'),
        DeclareLaunchArgument(
            'mid360_min_z',
            default_value='-1.75',
            description='Minimum sensor-frame Z retained for LV-DOT',
        ),
        DeclareLaunchArgument(
            'mid360_max_z',
            default_value='4.0',
            description='Maximum sensor-frame Z retained for LV-DOT',
        ),
        OpaqueFunction(function=_runtime, args=[paths]),
        Node(
            package='uav_usv_mission',
            executable='gz_sensor_bridge',
            name='lv_dot_tuning_camera_bridge',
            output='screen',
            parameters=[{
                'world_name': WORLD_NAME,
                'uav_ids': ['uav_01'],
                'uav_model_names': ['uav_01'],
                'usv_ids': ['unused_usv'],
                'usv_source_names': ['unused_usv'],
                'bridge_usv_scans': False,
                'bridge_base_radar': False,
                'camera_max_rate': 10.0,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='uav_camera_adapter.py',
            name='lv_dot_tuning_camera_adapter',
            output='screen',
            parameters=[{
                'vehicle_ids': ['uav_01'],
                'expected_rate_hz': 10.0,
                'tf_target_frame': 'map',
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='uav_camera_tf.py',
            name='lv_dot_tuning_camera_tf',
            output='screen',
            parameters=[{
                'vehicle_ids': ['uav_01'],
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
            }],
        ),
        Node(
            package='uav_usv_sim',
            executable='boat_nav2_interface',
            namespace='usv_01',
            name='lv_dot_tuning_boat_interface',
            output='screen',
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[{
                'use_sim_time': False,
                'boat_name': 'usv_01',
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'model_pose_topic': '/unused/usv_01/pose',
                'boat_cmd_topic': '/model/simple_boat/cmd_vel',
                'cmd_vel_topic': 'cmd_vel',
                'odom_topic': 'odom',
                'map_topic': 'map',
                'scan_topic': 'scan_raw',
                'filtered_scan_topic': 'scan',
                'marker_topic': 'reference_markers',
                'map_frame_id': 'map',
                'odom_frame_id': 'usv_01/odom',
                'base_frame_id': 'usv_01/base_link',
                'lidar_frame_id': 'usv_01/front_lidar',
                'publish_empty_map': True,
                'enable_lidar_safety': False,
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='tf_topic_relay.py',
            name='lv_dot_tuning_usv_tf_relay',
            output='screen',
            parameters=[{
                'input_topic': '/usv_01/tf',
                'output_topic': '/tf',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lv_dot_tuning_mid360_mount',
            output='screen',
            arguments=[
                '--x', '0.9075', '--y', '0.0', '--z', '1.5625',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'usv_01/base_link',
                '--child-frame-id', 'usv_01/mid360_link',
            ],
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_target_motion.py',
            name='lv_dot_tuning_target_motion',
            output='screen',
            parameters=[{'profile': profile}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_layer),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'target_entity': 'target_vessel',
                # Safety boundary: tuning never controls from sensor output.
                'perception_source': 'ground_truth',
            }.items(),
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_pose_adapter.py',
            name='lv_dot_tuning_uav_pose_adapter',
            output='screen',
            parameters=[{
                'input_mode': 'gazebo',
                'gazebo_pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'gazebo_entity_name': 'uav_01',
                'output_topic': '/perception/lv_dot/uav_01/pose',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='uav_usv_perception',
            executable='lv_dot_pose_adapter.py',
            name='lv_dot_tuning_pose_adapter',
            output='screen',
            parameters=[{
                'input_mode': 'gazebo',
                'gazebo_pose_topic': '/world/%s/pose/info' % WORLD_NAME,
                'gazebo_entity_name': 'usv_01',
                'output_topic': '/perception/lv_dot/usv_01/pose',
                'frame_id': 'map',
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(shadow_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_lv_dot_pose_adapter': 'false',
                'target_id': 'target_vessel',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lv_dot_tuning_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
        ExecuteProcess(
            cmd=[
                os.path.join(
                    get_package_prefix('uav_usv_mission'),
                    'lib',
                    'uav_usv_mission',
                    'fleet_base_station_gui',
                ),
                '--ros-args', '-p', 'demo_mode:=true',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_qt')),
        ),
    ])
