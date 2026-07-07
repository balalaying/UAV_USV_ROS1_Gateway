#!/usr/bin/env python3
import math
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CooperativeLighthouseMission(Node):

    def __init__(self):
        super().__init__('cooperative_lighthouse_mission')

        self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14540')
        self.declare_parameter('heartbeat_timeout', 180.0)
        self.declare_parameter('boat_cmd_topic', '/model/simple_boat/cmd_vel')
        self.declare_parameter(
            'deck_release_topic',
            '/model/x500_0/release_from_deck',
        )
        self.declare_parameter('pose_topic', '/world/default/pose/info')
        self.declare_parameter('boat_name', 'landing_boat')
        self.declare_parameter('drone_name', 'x500_mono_cam_down_0')
        self.declare_parameter('target_x', 35.0)
        self.declare_parameter('target_y', 18.0)
        self.declare_parameter('target_radius', 5.0)
        self.declare_parameter('takeoff_altitude', 8.0)
        self.declare_parameter('takeoff_climb_rate', 0.8)
        self.declare_parameter('cruise_altitude', 8.0)
        self.declare_parameter('drone_cruise_speed', 2.0)
        self.declare_parameter('drone_deck_approach_speed', 1.2)
        self.declare_parameter('drone_depart_delay', 10.0)
        self.declare_parameter('deck_hover_altitude', 4.0)
        self.declare_parameter('deck_lock_radius', 0.25)
        self.declare_parameter('deck_align_radius', 0.8)
        self.declare_parameter('deck_settle_time', 1.5)
        self.declare_parameter('deck_descent_rate', 0.35)
        self.declare_parameter('deck_touchdown_tolerance', 0.12)
        self.declare_parameter('deck_touchdown_hold_time', 0.8)
        self.declare_parameter('deck_offset_x', -1.518)
        self.declare_parameter('deck_offset_y', 0.0)
        self.declare_parameter('deck_offset_z', 0.56)
        self.declare_parameter('boat_speed', 1.1)
        self.declare_parameter('boat_turn_gain', 1.4)
        self.declare_parameter('boat_max_turn', 0.8)
        self.declare_parameter('deck_land_altitude', 0.1)

        self.mavlink_url = self.get_parameter('mavlink_url').value
        self.heartbeat_timeout = float(self.get_parameter('heartbeat_timeout').value)
        self.boat_cmd_topic = self.get_parameter('boat_cmd_topic').value
        self.deck_release_topic = self.get_parameter('deck_release_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.boat_name = self.get_parameter('boat_name').value
        self.drone_name = self.get_parameter('drone_name').value
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_radius = float(self.get_parameter('target_radius').value)
        self.takeoff_altitude = float(self.get_parameter('takeoff_altitude').value)
        self.takeoff_climb_rate = float(
            self.get_parameter('takeoff_climb_rate').value
        )
        self.cruise_altitude = float(self.get_parameter('cruise_altitude').value)
        self.drone_cruise_speed = float(
            self.get_parameter('drone_cruise_speed').value
        )
        self.drone_deck_approach_speed = float(
            self.get_parameter('drone_deck_approach_speed').value
        )
        self.drone_depart_delay = float(
            self.get_parameter('drone_depart_delay').value
        )
        self.deck_hover_altitude = float(
            self.get_parameter('deck_hover_altitude').value
        )
        self.deck_lock_radius = float(self.get_parameter('deck_lock_radius').value)
        self.deck_align_radius = float(self.get_parameter('deck_align_radius').value)
        self.deck_settle_time = float(self.get_parameter('deck_settle_time').value)
        self.deck_descent_rate = float(self.get_parameter('deck_descent_rate').value)
        self.deck_touchdown_tolerance = float(
            self.get_parameter('deck_touchdown_tolerance').value
        )
        self.deck_touchdown_hold_time = float(
            self.get_parameter('deck_touchdown_hold_time').value
        )
        self.deck_offset_x = float(self.get_parameter('deck_offset_x').value)
        self.deck_offset_y = float(self.get_parameter('deck_offset_y').value)
        self.deck_offset_z = float(self.get_parameter('deck_offset_z').value)
        self.boat_speed = float(self.get_parameter('boat_speed').value)
        self.boat_turn_gain = float(self.get_parameter('boat_turn_gain').value)
        self.boat_max_turn = float(self.get_parameter('boat_max_turn').value)
        self.deck_land_altitude = float(self.get_parameter('deck_land_altitude').value)

        self.gz_node = GzTransportNode()
        self.boat_pub = self.gz_node.advertise(self.boat_cmd_topic, Twist)
        self.release_pub = self.gz_node.advertise(self.deck_release_topic, Boolean)
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)

        self.boat_pose = None
        self.drone_pose = None
        self.local_position = None
        self.vehicle_armed = False
        self.stop_event = threading.Event()
        self.mav = None
        self.target_system = 1
        self.target_component = 1
        self.drone_sp = None
        self.local_origin_gz = None
        self.land_requested = False
        self.land_service = self.create_service(
            Trigger,
            'land_on_deck',
            self._on_land_request,
        )

        self.get_logger().info(
            'Mission target lighthouse: x=%.1f y=%.1f, drone_depart_delay=%.1f s'
            % (self.target_x, self.target_y, self.drone_depart_delay)
        )

    def destroy_node(self):
        self.stop_event.set()
        self.publish_boat_cmd(0.0, 0.0)
        super().destroy_node()

    def _on_pose_v(self, msg):
        for pose in msg.pose:
            if pose.name == self.boat_name:
                self.boat_pose = pose
            elif (
                pose.name == self.drone_name
                or (
                    self.drone_pose is None
                    and pose.name in ('x500_0', 'x500_mono_cam_0')
                )
            ):
                self.drone_pose = pose

    def _on_land_request(self, request, response):
        del request
        self.land_requested = True
        response.success = True
        response.message = 'Landing command accepted.'
        self.get_logger().info('Landing command received')
        return response

    def publish_boat_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.boat_pub.publish(msg)

    def release_drone_from_deck(self):
        msg = Boolean()
        msg.data = True
        for _ in range(4):
            self.release_pub.publish(msg)
            self.pump_mavlink(0.02)
        self.get_logger().info('Released drone from deck follower constraint')

    def lock_drone_to_deck(self):
        msg = Boolean()
        msg.data = False
        for _ in range(10):
            self.release_pub.publish(msg)
            self.pump_mavlink(0.02)
        self.get_logger().info('Locked drone back to deck follower constraint')

    def move_setpoint_toward(self, goal_x, goal_y, goal_z, max_speed, dt):
        if self.drone_sp is None:
            if self.local_position is None:
                self.drone_sp = [goal_x, goal_y, goal_z]
            else:
                self.drone_sp = [
                    self.local_position.x,
                    self.local_position.y,
                    self.local_position.z,
                ]

        dx = goal_x - self.drone_sp[0]
        dy = goal_y - self.drone_sp[1]
        dz = goal_z - self.drone_sp[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        max_step = max(0.02, max_speed * dt)

        if distance <= max_step or distance < 1e-6:
            self.drone_sp = [goal_x, goal_y, goal_z]
        else:
            scale = max_step / distance
            self.drone_sp[0] += dx * scale
            self.drone_sp[1] += dy * scale
            self.drone_sp[2] += dz * scale

        return self.drone_sp

    def wait_for_gazebo_drone_pose(self, timeout=30.0):
        start = time.monotonic()
        while rclpy.ok() and not self.stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.drone_pose is not None:
                return
            if time.monotonic() - start > timeout:
                raise RuntimeError('Timed out waiting for Gazebo drone pose')
            time.sleep(0.05)

    def calibrate_local_gazebo_frame(self):
        self.wait_for_local_position()
        self.wait_for_gazebo_drone_pose()
        self.local_origin_gz = (
            self.drone_pose.position.x - self.local_position.y,
            self.drone_pose.position.y - self.local_position.x,
            self.drone_pose.position.z + self.local_position.z,
        )
        self.get_logger().info(
            'Gazebo/PX4 frame origin: x=%.2f y=%.2f z=%.2f'
            % self.local_origin_gz
        )

    def gazebo_to_local(self, x_gz, y_gz, z_up_gz):
        if self.local_position is not None and self.drone_pose is not None:
            return (
                self.local_position.x + (y_gz - self.drone_pose.position.y),
                self.local_position.y + (x_gz - self.drone_pose.position.x),
                self.local_position.z - (z_up_gz - self.drone_pose.position.z),
            )

        if self.local_origin_gz is None:
            raise RuntimeError('Gazebo/PX4 frame is not calibrated')
        origin_x, origin_y, origin_z = self.local_origin_gz
        return y_gz - origin_y, x_gz - origin_x, origin_z - z_up_gz

    def local_to_gazebo(self, x_local, y_local, z_down):
        if self.local_position is not None and self.drone_pose is not None:
            return (
                self.drone_pose.position.x + (y_local - self.local_position.y),
                self.drone_pose.position.y + (x_local - self.local_position.x),
                self.drone_pose.position.z - (z_down - self.local_position.z),
            )

        if self.local_origin_gz is None:
            raise RuntimeError('Gazebo/PX4 frame is not calibrated')
        origin_x, origin_y, origin_z = self.local_origin_gz
        return y_local + origin_x, x_local + origin_y, origin_z - z_down

    def gazebo_drone_error(self, target_x, target_y, target_z_up):
        if self.drone_pose is None:
            return float('inf'), float('inf'), float('inf')

        horizontal = math.hypot(
            self.drone_pose.position.x - target_x,
            self.drone_pose.position.y - target_y,
        )
        vertical = abs(self.drone_pose.position.z - target_z_up)
        return math.hypot(horizontal, vertical), horizontal, vertical

    def connect_px4(self):
        self.get_logger().info(f'Waiting for PX4 MAVLink on {self.mavlink_url}')
        self.mav = mavutil.mavlink_connection(
            self.mavlink_url,
            autoreconnect=True,
            source_system=255,
            source_component=0,
        )
        heartbeat = self.mav.wait_heartbeat(timeout=self.heartbeat_timeout)
        if heartbeat is None:
            raise RuntimeError('Timed out waiting for PX4 heartbeat')
        self.target_system = self.mav.target_system
        self.target_component = self.mav.target_component
        self.get_logger().info(
            f'PX4 heartbeat received: sys={self.target_system}, comp={self.target_component}'
        )

    def command_long(self, command, params):
        values = list(params) + [0.0] * (7 - len(params))
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,
            *values[:7],
        )

    def set_param(self, name, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        self.mav.mav.param_set_send(
            self.target_system,
            self.target_component,
            name.encode('ascii'),
            float(value),
            param_type,
        )
        self.get_logger().info(f'Set PX4 parameter {name}={value}')

    def configure_px4_for_sitl_mission(self):
        # No QGroundControl is running in this automated demo, and SITL may not
        # report a power module. Disable those arming blockers for simulation.
        self.set_param('NAV_DLL_ACT', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.set_param('CBRK_SUPPLY_CHK', 894281, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.pump_mavlink(1.0)

    def set_mode(self, mode):
        if mode not in mavutil.px4_map:
            raise RuntimeError(f'PX4 mode {mode} is not available in pymavlink px4_map')
        base_mode, custom_mode, custom_sub_mode = mavutil.px4_map[mode]
        self.mav.set_mode(base_mode, custom_mode, custom_sub_mode)
        self.get_logger().info(f'Requested PX4 mode {mode}')

    def arm(self):
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1.0])
        self.get_logger().info('Requested arm')

    def disarm(self):
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [0.0])
        self.get_logger().info('Requested disarm')

    def force_disarm(self):
        self.command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, 21196.0],
        )
        self.get_logger().info('Requested force disarm')

    def send_local_position_setpoint(self, x, y, z_down, yaw=0.0):
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self.mav.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(x),
            float(y),
            float(z_down),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(yaw),
            0.0,
        )

    def pump_mavlink(self, duration):
        end_time = time.monotonic() + duration
        while rclpy.ok() and not self.stop_event.is_set() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.0)
            msg = self.mav.recv_match(blocking=False)
            while msg is not None:
                msg_type = msg.get_type()
                if msg_type == 'LOCAL_POSITION_NED':
                    self.local_position = msg
                elif msg_type == 'HEARTBEAT':
                    self.vehicle_armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                msg = self.mav.recv_match(blocking=False)
            time.sleep(0.02)

    def deck_target_pose(self):
        if self.boat_pose is None:
            return self.target_x, self.target_y, self.deck_hover_altitude, 0.0

        yaw = yaw_from_quaternion(self.boat_pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        deck_x = (
            self.boat_pose.position.x
            + cos_yaw * self.deck_offset_x
            - sin_yaw * self.deck_offset_y
        )
        deck_y = (
            self.boat_pose.position.y
            + sin_yaw * self.deck_offset_x
            + cos_yaw * self.deck_offset_y
        )
        deck_z = self.boat_pose.position.z + self.deck_offset_z
        hover_z_up = deck_z + self.deck_hover_altitude
        return deck_x, deck_y, hover_z_up, yaw

    def wait_for_local_position(self, timeout=30.0):
        start = time.monotonic()
        while rclpy.ok() and not self.stop_event.is_set():
            self.pump_mavlink(0.1)
            if self.local_position is not None:
                return
            if time.monotonic() - start > timeout:
                raise RuntimeError('Timed out waiting for PX4 LOCAL_POSITION_NED')

    def prepare_offboard_takeoff(self):
        self.calibrate_local_gazebo_frame()
        start_x = self.local_position.x
        start_y = self.local_position.y
        start_z = self.local_position.z
        takeoff_z = -abs(self.takeoff_altitude)
        self.drone_sp = [start_x, start_y, start_z]

        self.get_logger().info('Streaming initial deck-hold offboard setpoints')
        for _ in range(80):
            self.send_local_position_setpoint(start_x, start_y, start_z)
            self.pump_mavlink(0.05)

        self.set_mode('OFFBOARD')
        self.release_drone_from_deck()
        self.arm()

        self.get_logger().info(
            'Climbing smoothly to %.1f m at %.1f m/s'
            % (self.takeoff_altitude, self.takeoff_climb_rate)
        )
        start = time.monotonic()
        last_step = start
        while rclpy.ok() and not self.stop_event.is_set():
            now = time.monotonic()
            dt = now - last_step
            last_step = now
            sp_x, sp_y, sp_z = self.move_setpoint_toward(
                start_x,
                start_y,
                takeoff_z,
                self.takeoff_climb_rate,
                dt,
            )
            self.send_local_position_setpoint(sp_x, sp_y, sp_z)
            self.pump_mavlink(0.05)
            if self.local_position and self.local_position.z <= takeoff_z + 0.6:
                return start_x, start_y
            if time.monotonic() - start > 45.0:
                raise RuntimeError('Timed out during offboard takeoff')

    def boat_distance_to_target(self):
        if self.boat_pose is None:
            return float('inf')
        dx = self.target_x - self.boat_pose.position.x
        dy = self.target_y - self.boat_pose.position.y
        return math.hypot(dx, dy)

    def lighthouse_standoff_point(self, reference_x, reference_y):
        dx = reference_x - self.target_x
        dy = reference_y - self.target_y
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            dx = -1.0
            dy = 0.0
            distance = 1.0

        radius = max(self.target_radius, 0.5)
        return (
            self.target_x + radius * dx / distance,
            self.target_y + radius * dy / distance,
        )

    def update_boat_to_target(self):
        if self.boat_pose is None:
            self.publish_boat_cmd(0.0, 0.0)
            return False

        dx = self.target_x - self.boat_pose.position.x
        dy = self.target_y - self.boat_pose.position.y
        distance = math.hypot(dx, dy)
        if distance <= self.target_radius:
            self.publish_boat_cmd(0.0, 0.0)
            return True

        desired_yaw = math.atan2(dy, dx)
        yaw = yaw_from_quaternion(self.boat_pose.orientation)
        yaw_error = wrap_pi(desired_yaw - yaw)
        turn = clamp(
            self.boat_turn_gain * yaw_error,
            -self.boat_max_turn,
            self.boat_max_turn,
        )
        speed_scale = clamp(1.0 - abs(yaw_error) / math.pi, 0.25, 1.0)
        self.publish_boat_cmd(self.boat_speed * speed_scale, turn)
        return False

    def move_boat_then_drone_to_lighthouse(self, start_x, start_y):
        self.get_logger().info(
            'Boat moving toward lighthouse; drone holding above the boat first'
        )
        hold_z = -abs(self.takeoff_altitude)
        start_gz_x, start_gz_y, _ = self.local_to_gazebo(start_x, start_y, hold_z)
        drone_target_gz_x, drone_target_gz_y = self.lighthouse_standoff_point(
            start_gz_x,
            start_gz_y,
        )
        lighthouse_x, lighthouse_y, lighthouse_z = self.gazebo_to_local(
            drone_target_gz_x,
            drone_target_gz_y,
            self.cruise_altitude,
        )
        self.get_logger().info(
            'Drone lighthouse standoff target: x=%.1f y=%.1f radius=%.1f m'
            % (drone_target_gz_x, drone_target_gz_y, self.target_radius)
        )
        start = time.monotonic()
        drone_depart_time = start + self.drone_depart_delay
        last_log = 0.0
        last_step = start

        while rclpy.ok() and not self.stop_event.is_set():
            boat_arrived = self.update_boat_to_target()

            now = time.monotonic()
            dt = now - last_step
            last_step = now
            if now < drone_depart_time:
                sp_x, sp_y, sp_z = self.move_setpoint_toward(
                    start_x,
                    start_y,
                    hold_z,
                    self.drone_deck_approach_speed,
                    dt,
                )
                drone_goal = 'holding'
            else:
                sp_x, sp_y, sp_z = self.move_setpoint_toward(
                    lighthouse_x,
                    lighthouse_y,
                    lighthouse_z,
                    self.drone_cruise_speed,
                    dt,
                )
                drone_goal = 'lighthouse'

            self.send_local_position_setpoint(sp_x, sp_y, sp_z)
            self.pump_mavlink(0.05)

            if now - last_log > 5.0:
                self.get_logger().info(
                    'Distance to lighthouse: boat=%.1f m, drone=%s, drone_goal=%s'
                    % (
                        self.boat_distance_to_target(),
                        'unknown'
                        if self.drone_pose is None
                        else '%.1f m'
                        % math.hypot(
                            self.target_x - self.drone_pose.position.x,
                            self.target_y - self.drone_pose.position.y,
                        ),
                        drone_goal,
                    )
                )
                last_log = now

            if boat_arrived:
                self.publish_boat_cmd(0.0, 0.0)
                self.get_logger().info(
                    'Boat reached lighthouse vicinity; drone switching to deck hover'
                )
                return

            if now - start > 180.0:
                self.publish_boat_cmd(0.0, 0.0)
                raise RuntimeError('Timed out moving to lighthouse')

    def land_on_realtime_deck_and_lock(self):
        self.get_logger().info('Landing directly on the real-time boat landing pad')
        last_log = 0.0
        last_step = time.monotonic()
        start = last_step
        phase = 'approach'
        settled_since = None
        touchdown_since = None

        while rclpy.ok() and not self.stop_event.is_set():
            deck_x, deck_y, hover_z_up, deck_yaw = self.deck_target_pose()
            deck_surface_z_up = hover_z_up - self.deck_hover_altitude
            land_z_up = deck_surface_z_up + self.deck_land_altitude
            land_x, land_y, land_z_down = self.gazebo_to_local(
                deck_x,
                deck_y,
                land_z_up,
            )
            hover_x, hover_y, hover_z_down = self.gazebo_to_local(
                deck_x,
                deck_y,
                hover_z_up,
            )
            hover_actual_error, hover_actual_h, hover_actual_v = (
                self.gazebo_drone_error(deck_x, deck_y, hover_z_up)
            )
            now = time.monotonic()
            dt = now - last_step
            last_step = now

            if self.drone_sp is None:
                if self.local_position is not None:
                    self.drone_sp = [
                        self.local_position.x,
                        self.local_position.y,
                        self.local_position.z,
                    ]
                else:
                    self.drone_sp = [hover_x, hover_y, hover_z_down]

            if phase == 'approach':
                sp_x, sp_y, sp_z = self.move_setpoint_toward(
                    hover_x,
                    hover_y,
                    hover_z_down,
                    self.drone_deck_approach_speed,
                    dt,
                )
                if hover_actual_error <= self.deck_align_radius:
                    phase = 'settle'
                    settled_since = now
                    self.get_logger().info('Aligned above landing pad; settling before descent')

            elif phase == 'settle':
                self.drone_sp = [hover_x, hover_y, hover_z_down]
                sp_x, sp_y, sp_z = self.drone_sp
                if settled_since is not None and now - settled_since >= self.deck_settle_time:
                    phase = 'descent'
                    self.get_logger().info(
                        'Beginning vertical descent at %.2f m/s'
                        % self.deck_descent_rate
                    )

            else:
                # During the final descent, keep x/y locked to the moving deck
                # and lower z only by a small step each cycle. The last meter is
                # deliberately slower so the deck lock does not look like a pull.
                current_z = self.drone_sp[2]
                _, _, actual_vertical_to_land = self.gazebo_drone_error(
                    deck_x,
                    deck_y,
                    land_z_up,
                )
                final_scale = clamp(actual_vertical_to_land / 1.0, 0.35, 1.0)
                z_step = max(0.004, self.deck_descent_rate * final_scale * dt)
                next_z = min(current_z + z_step, land_z_down)
                self.drone_sp = [land_x, land_y, next_z]
                sp_x, sp_y, sp_z = self.drone_sp

            self.send_local_position_setpoint(sp_x, sp_y, sp_z, deck_yaw)
            self.pump_mavlink(0.05)
            setpoint_error = math.sqrt((sp_x - land_x) ** 2
                                       + (sp_y - land_y) ** 2
                                       + (sp_z - land_z_down) ** 2)
            setpoint_horizontal_error = math.hypot(sp_x - land_x, sp_y - land_y)
            actual_error, actual_horizontal_error, actual_vertical_error = (
                self.gazebo_drone_error(deck_x, deck_y, land_z_up)
            )

            if now - last_log > 5.0:
                self.get_logger().info(
                    'Landing phase=%s: target=(%.1f, %.1f, %.1f), setpoint=(%.1f, %.1f, %.1f), hover_h=%.2f, actual_h=%.2f, actual_v=%.2f'
                    % (
                        phase,
                        deck_x,
                        deck_y,
                        land_z_up,
                        *self.local_to_gazebo(sp_x, sp_y, sp_z),
                        hover_actual_h,
                        actual_horizontal_error,
                        actual_vertical_error,
                    )
                )
                last_log = now

            touchdown_ready = (
                phase == 'descent'
                and setpoint_error <= self.deck_lock_radius
                and setpoint_horizontal_error <= self.deck_lock_radius
                and actual_horizontal_error <= self.deck_touchdown_tolerance
                and actual_vertical_error <= self.deck_touchdown_tolerance
            )
            if touchdown_ready:
                if touchdown_since is None:
                    touchdown_since = now
                    self.get_logger().info(
                        'Touchdown position reached; holding before deck lock'
                    )
            else:
                touchdown_since = None

            if (
                touchdown_since is not None
                and now - touchdown_since >= self.deck_touchdown_hold_time
            ):
                self.get_logger().info('Reached landing pad vicinity; relocking deck follower')
                self.lock_drone_to_deck()
                self.force_disarm()
                self.pump_mavlink(1.0)
                return

            if now - start > 90.0:
                raise RuntimeError('Timed out landing on the moving deck')

    def land_on_deck_now(self):
        self.get_logger().info('Landing command active: switching PX4 to LAND')
        self.set_mode('LAND')
        land_end = time.monotonic() + 60.0
        while rclpy.ok() and not self.stop_event.is_set() and time.monotonic() < land_end:
            self.pump_mavlink(0.1)
            if self.local_position is not None and self.local_position.z > -0.25:
                self.get_logger().info('PX4 LAND descent reached deck height')
                return

    def run(self):
        self.connect_px4()
        self.configure_px4_for_sitl_mission()
        start_x, start_y = self.prepare_offboard_takeoff()
        self.move_boat_then_drone_to_lighthouse(start_x, start_y)
        self.land_on_realtime_deck_and_lock()
        self.publish_boat_cmd(0.0, 0.0)
        self.get_logger().info('Cooperative lighthouse mission complete')


def main(args=None):
    rclpy.init(args=args)
    node = CooperativeLighthouseMission()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
