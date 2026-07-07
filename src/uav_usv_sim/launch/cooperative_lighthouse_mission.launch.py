from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mavlink_url', default_value='udp:127.0.0.1:14540'),
        DeclareLaunchArgument('heartbeat_timeout', default_value='180.0'),
        DeclareLaunchArgument('target_x', default_value='35.0'),
        DeclareLaunchArgument('target_y', default_value='18.0'),
        DeclareLaunchArgument('target_radius', default_value='5.0'),
        DeclareLaunchArgument('takeoff_altitude', default_value='8.0'),
        DeclareLaunchArgument('takeoff_climb_rate', default_value='0.8'),
        DeclareLaunchArgument('cruise_altitude', default_value='8.0'),
        DeclareLaunchArgument('drone_cruise_speed', default_value='2.0'),
        DeclareLaunchArgument('drone_deck_approach_speed', default_value='1.2'),
        DeclareLaunchArgument('drone_depart_delay', default_value='10.0'),
        DeclareLaunchArgument('deck_hover_altitude', default_value='4.0'),
        DeclareLaunchArgument('deck_lock_radius', default_value='0.25'),
        DeclareLaunchArgument('deck_align_radius', default_value='0.8'),
        DeclareLaunchArgument('deck_settle_time', default_value='1.5'),
        DeclareLaunchArgument('deck_descent_rate', default_value='0.35'),
        DeclareLaunchArgument('deck_touchdown_tolerance', default_value='0.12'),
        DeclareLaunchArgument('deck_touchdown_hold_time', default_value='0.8'),
        DeclareLaunchArgument('deck_offset_x', default_value='-1.518'),
        DeclareLaunchArgument('deck_offset_y', default_value='0.0'),
        DeclareLaunchArgument('deck_offset_z', default_value='0.56'),
        DeclareLaunchArgument('deck_land_altitude', default_value='0.1'),
        DeclareLaunchArgument('boat_speed', default_value='1.1'),
    ]

    mission_node = Node(
        package='uav_usv_sim',
        executable='cooperative_lighthouse_mission',
        name='cooperative_lighthouse_mission',
        output='screen',
        parameters=[
            {
                'mavlink_url': LaunchConfiguration('mavlink_url'),
                'heartbeat_timeout': ParameterValue(
                    LaunchConfiguration('heartbeat_timeout'),
                    value_type=float,
                ),
                'target_x': ParameterValue(
                    LaunchConfiguration('target_x'),
                    value_type=float,
                ),
                'target_y': ParameterValue(
                    LaunchConfiguration('target_y'),
                    value_type=float,
                ),
                'target_radius': ParameterValue(
                    LaunchConfiguration('target_radius'),
                    value_type=float,
                ),
                'takeoff_altitude': ParameterValue(
                    LaunchConfiguration('takeoff_altitude'),
                    value_type=float,
                ),
                'takeoff_climb_rate': ParameterValue(
                    LaunchConfiguration('takeoff_climb_rate'),
                    value_type=float,
                ),
                'cruise_altitude': ParameterValue(
                    LaunchConfiguration('cruise_altitude'),
                    value_type=float,
                ),
                'drone_cruise_speed': ParameterValue(
                    LaunchConfiguration('drone_cruise_speed'),
                    value_type=float,
                ),
                'drone_deck_approach_speed': ParameterValue(
                    LaunchConfiguration('drone_deck_approach_speed'),
                    value_type=float,
                ),
                'drone_depart_delay': ParameterValue(
                    LaunchConfiguration('drone_depart_delay'),
                    value_type=float,
                ),
                'deck_hover_altitude': ParameterValue(
                    LaunchConfiguration('deck_hover_altitude'),
                    value_type=float,
                ),
                'deck_lock_radius': ParameterValue(
                    LaunchConfiguration('deck_lock_radius'),
                    value_type=float,
                ),
                'deck_align_radius': ParameterValue(
                    LaunchConfiguration('deck_align_radius'),
                    value_type=float,
                ),
                'deck_settle_time': ParameterValue(
                    LaunchConfiguration('deck_settle_time'),
                    value_type=float,
                ),
                'deck_descent_rate': ParameterValue(
                    LaunchConfiguration('deck_descent_rate'),
                    value_type=float,
                ),
                'deck_touchdown_tolerance': ParameterValue(
                    LaunchConfiguration('deck_touchdown_tolerance'),
                    value_type=float,
                ),
                'deck_touchdown_hold_time': ParameterValue(
                    LaunchConfiguration('deck_touchdown_hold_time'),
                    value_type=float,
                ),
                'deck_offset_x': ParameterValue(
                    LaunchConfiguration('deck_offset_x'),
                    value_type=float,
                ),
                'deck_offset_y': ParameterValue(
                    LaunchConfiguration('deck_offset_y'),
                    value_type=float,
                ),
                'deck_offset_z': ParameterValue(
                    LaunchConfiguration('deck_offset_z'),
                    value_type=float,
                ),
                'deck_land_altitude': ParameterValue(
                    LaunchConfiguration('deck_land_altitude'),
                    value_type=float,
                ),
                'boat_speed': ParameterValue(
                    LaunchConfiguration('boat_speed'),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription([*args, mission_node])
