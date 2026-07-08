import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory('uav_usv_gazebo')
    bringup_share = get_package_share_directory('uav_usv_bringup')
    source_package_dir = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
    )
    source_root = os.path.abspath(os.path.join(source_package_dir, '..'))
    source_world = os.path.join(
        source_root,
        'uav_usv_gazebo',
        'worlds',
        'defense.sdf',
    )
    installed_world = os.path.join(gazebo_share, 'worlds', 'defense.sdf')
    world = source_world if os.path.exists(source_world) else installed_world
    source_rviz_config = os.path.join(
        source_package_dir,
        'rviz',
        'defense.rviz',
    )
    installed_rviz_config = os.path.join(bringup_share, 'rviz', 'defense.rviz')
    rviz_config = (
        source_rviz_config
        if os.path.exists(source_rviz_config)
        else installed_rviz_config
    )
    source_mission_script = os.path.join(
        source_root,
        'uav_usv_mission',
        'scripts',
        'defense_demo.py',
    )
    installed_mission_script = os.path.join(
        get_package_share_directory('uav_usv_mission'),
        '..',
        '..',
        'lib',
        'uav_usv_mission',
        'defense_demo',
    )
    mission_script = (
        source_mission_script
        if os.path.exists(source_mission_script)
        else os.path.abspath(installed_mission_script)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_gazebo',
                default_value='true',
                description='Start Gazebo with the defense demo world.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Start RViz with the defense visualization.',
            ),
            DeclareLaunchArgument(
                'defend_radius',
                default_value='75.0',
                description='Guard circle radius around the base.',
            ),
            DeclareLaunchArgument(
                'trigger_radius',
                default_value='190.0',
                description='Enemy distance that triggers guard behavior.',
            ),
            DeclareLaunchArgument(
                'base_safety_radius',
                default_value='18.0',
                description='Enemy ships turn away before entering this base radius.',
            ),
            DeclareLaunchArgument(
                'enemy_speed',
                default_value='3.5',
                description='Enemy ship attack speed.',
            ),
            DeclareLaunchArgument(
                'enemy_evasion_radius',
                default_value='48.0',
                description='Enemy ships try to go around defenders inside this radius.',
            ),
            DeclareLaunchArgument(
                'enemy_evasion_gain',
                default_value='36.0',
                description='Enemy lateral target offset when evading defenders.',
            ),
            DeclareLaunchArgument(
                'base_avoid_radius',
                default_value='34.0',
                description='All ships start avoiding the base inside this radius.',
            ),
            DeclareLaunchArgument(
                'base_avoid_gain',
                default_value='2.8',
                description='Repulsive gain used to keep ships away from the base.',
            ),
            DeclareLaunchArgument(
                'base_hard_keepout_radius',
                default_value='16.0',
                description='Ships inside this radius prioritize leaving the base area.',
            ),
            DeclareLaunchArgument(
                'own_patrol_speed',
                default_value='7.0',
                description='Own ship random patrol speed before threats arrive.',
            ),
            DeclareLaunchArgument(
                'own_guard_speed',
                default_value='15.0',
                description='Own ship speed when moving to guard points.',
            ),
            DeclareLaunchArgument(
                'guard_spacing',
                default_value='28.0',
                description='Spacing between own ships assigned to the same threat.',
            ),
            DeclareLaunchArgument(
                'guard_lead_distance',
                default_value='24.0',
                description='Predictive lead distance used to place guard points.',
            ),
            DeclareLaunchArgument(
                'guard_target_alpha',
                default_value='0.22',
                description='Low-pass factor for moving guard target points.',
            ),
            DeclareLaunchArgument(
                'intercept_stop_distance',
                default_value='26.0',
                description='Enemy ship stops when a defender is this close.',
            ),
            DeclareLaunchArgument(
                'own_avoid_radius',
                default_value='30.0',
                description='Own ships start repelling each other inside this radius.',
            ),
            DeclareLaunchArgument(
                'own_yield_radius',
                default_value='45.0',
                description='Lower-priority own ships slow down inside this radius.',
            ),
            DeclareLaunchArgument(
                'own_brake_radius',
                default_value='22.0',
                description='Lower-priority own ships stop inside this high-risk radius.',
            ),
            DeclareLaunchArgument(
                'rviz_marker_rate',
                default_value='20.0',
                description='RViz marker publishing rate.',
            ),
            DeclareLaunchArgument(
                'gazebo_marker_rate',
                default_value='5.0',
                description='Gazebo visual marker publishing rate.',
            ),
            ExecuteProcess(
                cmd=['gz', 'sim', '-r', world],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_gazebo')),
            ),
            ExecuteProcess(
                cmd=[
                    'rviz2',
                    '-d',
                    rviz_config,
                    '--ros-args',
                    '-r',
                    '__node:=defense_rviz',
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
            ExecuteProcess(
                cmd=[
                    mission_script,
                    '--ros-args',
                    '-r',
                    '__node:=defense_demo',
                    '-p',
                    ['defend_radius:=', LaunchConfiguration('defend_radius')],
                    '-p',
                    ['trigger_radius:=', LaunchConfiguration('trigger_radius')],
                    '-p',
                    [
                        'base_safety_radius:=',
                        LaunchConfiguration('base_safety_radius'),
                    ],
                    '-p',
                    ['enemy_speed:=', LaunchConfiguration('enemy_speed')],
                    '-p',
                    [
                        'enemy_evasion_radius:=',
                        LaunchConfiguration('enemy_evasion_radius'),
                    ],
                    '-p',
                    [
                        'enemy_evasion_gain:=',
                        LaunchConfiguration('enemy_evasion_gain'),
                    ],
                    '-p',
                    [
                        'base_avoid_radius:=',
                        LaunchConfiguration('base_avoid_radius'),
                    ],
                    '-p',
                    ['base_avoid_gain:=', LaunchConfiguration('base_avoid_gain')],
                    '-p',
                    [
                        'base_hard_keepout_radius:=',
                        LaunchConfiguration('base_hard_keepout_radius'),
                    ],
                    '-p',
                    ['own_patrol_speed:=', LaunchConfiguration('own_patrol_speed')],
                    '-p',
                    ['own_guard_speed:=', LaunchConfiguration('own_guard_speed')],
                    '-p',
                    ['guard_spacing:=', LaunchConfiguration('guard_spacing')],
                    '-p',
                    [
                        'guard_lead_distance:=',
                        LaunchConfiguration('guard_lead_distance'),
                    ],
                    '-p',
                    [
                        'guard_target_alpha:=',
                        LaunchConfiguration('guard_target_alpha'),
                    ],
                    '-p',
                    [
                        'intercept_stop_distance:=',
                        LaunchConfiguration('intercept_stop_distance'),
                    ],
                    '-p',
                    ['own_avoid_radius:=', LaunchConfiguration('own_avoid_radius')],
                    '-p',
                    ['own_yield_radius:=', LaunchConfiguration('own_yield_radius')],
                    '-p',
                    ['own_brake_radius:=', LaunchConfiguration('own_brake_radius')],
                    '-p',
                    ['rviz_marker_rate:=', LaunchConfiguration('rviz_marker_rate')],
                    '-p',
                    [
                        'gazebo_marker_rate:=',
                        LaunchConfiguration('gazebo_marker_rate'),
                    ],
                ],
                output='screen',
            ),
        ]
    )
