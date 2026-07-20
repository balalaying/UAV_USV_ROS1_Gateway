import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _script(source_root, package, script_name, installed_name):
    source = os.path.join(source_root, package, 'scripts', script_name)
    if os.path.exists(source):
        return source
    installed = os.path.join(
        get_package_share_directory(package),
        '..',
        '..',
        'lib',
        package,
        installed_name,
    )
    return os.path.abspath(installed)


def _camera_bridges(partition, namespace):
    bridges = []
    for index in range(1, 5):
        vehicle_id = 'uav_%02d' % index
        topic = '/fleet/uplink/%s/camera' % vehicle_id
        bridges.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='%s_%s_camera_bridge' % (namespace, vehicle_id),
                output='screen',
                additional_env={'GZ_PARTITION': partition},
                arguments=[
                    topic + '@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
                remappings=[
                    (topic, '/%s/fleet/uplink/%s/camera'
                     % (namespace, vehicle_id)),
                ],
            )
        )
    for index in range(1, 5):
        model_name = 'own_%02d' % index
        vehicle_id = 'usv_%02d' % index
        topic = '/defense/%s/front_camera' % model_name
        bridges.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='%s_%s_camera_bridge' % (namespace, vehicle_id),
                output='screen',
                additional_env={'GZ_PARTITION': partition},
                arguments=[
                    topic + '@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
                remappings=[
                    (topic, '/%s/fleet/uplink/%s/camera'
                     % (namespace, vehicle_id)),
                ],
            )
        )
    return bridges


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_usv_bringup')
    gazebo_share = get_package_share_directory('uav_usv_gazebo')
    mission_share = get_package_share_directory('uav_usv_mission')
    source_root = os.path.abspath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', '..'
    ))

    capture_world = os.path.join(
        gazebo_share, 'worlds', 'capture_ocean_sim.sdf'
    )
    defense_world = os.path.join(
        gazebo_share, 'worlds', 'open_ocean_defense_sim.sdf'
    )
    gui_config = os.path.join(
        bringup_share, 'config', 'gazebo_white_gui.config'
    )
    capture_rviz = os.path.join(bringup_share, 'rviz', 'capture.rviz')
    defense_rviz = os.path.join(bringup_share, 'rviz', 'defense.rviz')
    px4_dir = os.path.expanduser(
        os.environ.get('PX4_DIR', '~/PX4-Autopilot')
    )
    px4_models = os.path.join(
        px4_dir, 'Tools', 'simulation', 'gz', 'models'
    )

    base_station = _script(
        source_root, 'uav_usv_mission',
        'fleet_base_station.py', 'fleet_base_station'
    )
    sensor_agent = _script(
        source_root, 'uav_usv_mission',
        'fleet_simulated_agent.py', 'fleet_simulated_agent'
    )
    capture_mission = _script(
        source_root, 'uav_usv_mission',
        'capture_mission.py', 'capture_mission'
    )
    defense_demo = _script(
        source_root, 'uav_usv_mission',
        'defense_demo.py', 'defense_demo'
    )
    gui = _script(
        source_root, 'uav_usv_mission',
        'fleet_base_station_gui.py', 'fleet_base_station_gui'
    )

    capture_vehicles = (
        'usv_01:usv:own_01:-190:-162:0.6;'
        'usv_02:usv:own_02:-186:-134:0.6;'
        'usv_03:usv:own_03:-186:-104:0.6;'
        'usv_04:usv:own_04:-190:-76:0.6;'
        'uav_01:uav:uav_01:-120:-120:2.9;'
        'uav_02:uav:uav_02:-120:-120:6.4;'
        'uav_03:uav:uav_03:-120:-120:9.9;'
        'uav_04:uav:uav_04:-120:-120:13.4'
    )
    defense_vehicles = capture_vehicles

    capture_gazebo = (
        'export GZ_PARTITION=capture_demo; '
        'export GZ_SIM_RESOURCE_PATH="%s:${GZ_SIM_RESOURCE_PATH:-}"; '
        'export GZ_SIM_ARGS="-r --gui-config %s"; '
        'ros2 run uav_usv_gazebo run_gz_world.sh capture_ocean_sim'
    ) % (px4_models, gui_config)
    defense_gazebo = (
        'export GZ_PARTITION=defense_demo; '
        'export GZ_SIM_RESOURCE_PATH="%s:${GZ_SIM_RESOURCE_PATH:-}"; '
        'export GZ_SIM_ARGS="-r --gui-config %s"; '
        'ros2 run uav_usv_gazebo run_gz_world.sh open_ocean_defense_sim'
    ) % (px4_models, gui_config)

    capture_ids = 'uav_01,uav_02,uav_03,uav_04'
    usv_ids = 'usv_01,usv_02,usv_03,usv_04'

    return LaunchDescription([
        DeclareLaunchArgument('start_capture_gazebo', default_value='true'),
        DeclareLaunchArgument('start_defense_gazebo', default_value='true'),
        DeclareLaunchArgument('start_capture_rviz', default_value='true'),
        DeclareLaunchArgument('start_defense_rviz', default_value='true'),
        DeclareLaunchArgument('start_qt', default_value='true'),
        ExecuteProcess(
            cmd=['bash', '-c', capture_gazebo],
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_capture_gazebo')
            ),
        ),
        ExecuteProcess(
            cmd=['bash', '-c', defense_gazebo],
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_defense_gazebo')
            ),
        ),
        ExecuteProcess(
            cmd=[
                'rviz2', '-d', capture_rviz, '--ros-args',
                '-r', '__node:=capture_rviz',
                '-r', '/fleet/capture/markers:=/capture/fleet/capture/markers',
                '-r', '/fleet/base/selected_target:=/capture/fleet/base/selected_target',
                '-r', '/fleet/base/markers:=/capture/fleet/base/markers',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_capture_rviz')),
        ),
        ExecuteProcess(
            cmd=[
                'rviz2', '-d', defense_rviz, '--ros-args',
                '-r', '__node:=defense_rviz',
                '-r', '/defense/own_ships:=/defense/defense/own_ships',
                '-r', '/defense/enemy_ships:=/defense/defense/enemy_ships',
                '-r', '/defense/rviz_markers:=/defense/defense/rviz_markers',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_defense_rviz')),
        ),
        ExecuteProcess(
            cmd=[
                'python3', base_station, '--ros-args',
                '-r', '__node:=capture_base_station',
                '-p', 'topic_namespace:=capture',
                '-p', ['uav_ids:=', capture_ids],
                '-p', ['usv_ids:=', usv_ids],
                '-p', 'auto_demo:=false',
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'python3', sensor_agent, '--ros-args',
                '-r', '__node:=capture_sensor_agent',
                '-p', 'topic_namespace:=capture',
                '-p', ['vehicles:=', capture_vehicles],
                '-p', 'pose_topic:=/world/capture_ocean_sim/pose/info',
                '-p', 'image_rate:=15.0',
                '-p', 'scan_rate:=10.0',
                '-p', 'publish_images:=false',
            ],
            output='screen',
            additional_env={'GZ_PARTITION': 'capture_demo'},
        ),
        ExecuteProcess(
            cmd=[
                'python3', capture_mission, '--ros-args',
                '-r', '__node:=capture_mission',
                '-p', 'topic_namespace:=capture',
                '-p', ['uav_ids:=', capture_ids],
                '-p', ['usv_ids:=', usv_ids],
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'python3', defense_demo, '--ros-args',
                '-r', '__node:=defense_sim_demo',
                '-p', 'topic_namespace:=defense',
                '-p', 'pose_topic:=/world/open_ocean_defense_sim/pose/info',
                '-p', 'defend_radius:=75.0',
                '-p', 'trigger_radius:=190.0',
                '-p', 'enemy_speed:=4.5',
                '-p', 'own_guard_speed:=15.0',
            ],
            output='screen',
            additional_env={'GZ_PARTITION': 'defense_demo'},
        ),
        ExecuteProcess(
            cmd=[
                'python3', base_station, '--ros-args',
                '-r', '__node:=defense_base_station',
                '-p', 'topic_namespace:=defense',
                '-p', ['uav_ids:=', capture_ids],
                '-p', ['usv_ids:=', usv_ids],
                '-p', 'auto_demo:=false',
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'python3', sensor_agent, '--ros-args',
                '-r', '__node:=defense_sensor_agent',
                '-p', 'topic_namespace:=defense',
                '-p', ['vehicles:=', defense_vehicles],
                '-p', 'pose_topic:=/world/open_ocean_defense_sim/pose/info',
                '-p', 'image_rate:=15.0',
                '-p', 'scan_rate:=10.0',
                '-p', 'publish_images:=false',
            ],
            output='screen',
            additional_env={'GZ_PARTITION': 'defense_demo'},
        ),
        *_camera_bridges('capture_demo', 'capture'),
        *_camera_bridges('defense_demo', 'defense'),
        ExecuteProcess(
            cmd=[
                'python3', gui, '--ros-args',
                '-r', '__node:=unified_base_station_gui',
                '-p', 'capture_namespace:=capture',
                '-p', 'defense_namespace:=defense',
                '-p', 'defense_node_name:=/defense_sim_demo',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_qt')),
        ),
    ])
