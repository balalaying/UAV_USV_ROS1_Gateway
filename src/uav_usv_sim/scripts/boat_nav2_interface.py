#!/usr/bin/env python3
import copy
import math
import time

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist as RosTwist
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.transport13 import Node as GzTransportNode
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def roll_pitch_from_quaternion(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
    return roll, math.asin(sinp)


def clamp(value, lower, upper):
    return max(lower, min(value, upper))

class VelocityPid:
    """PID correction around a velocity feed-forward command."""

    def __init__(self, kp, ki, kd, integral_limit, derivative_alpha):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.derivative_alpha = clamp(derivative_alpha, 0.0, 1.0)
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return self.kp * error

        self.integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )

        raw_derivative = 0.0
        if self.previous_error is not None:
            raw_derivative = (error - self.previous_error) / dt
        self.previous_error = error
        self.filtered_derivative += self.derivative_alpha * (
            raw_derivative - self.filtered_derivative
        )

        return (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_derivative
        )


class BoatNav2Interface:
    """Adapts the Gazebo boat simulation to the ROS interfaces Nav2 expects."""

    def __init__(self):








































        # Zero keeps the established Nav2 behavior.  A positive value is a
        # runtime ceiling used by the base-station speed-control page.



















        self.pose_topic = rospy.get_param('~pose_topic', '/world/default/pose/info')
        self.model_pose_topic = rospy.get_param('~model_pose_topic', '/boat/pose')
        self.boat_cmd_topic = rospy.get_param('~boat_cmd_topic', '/model/simple_boat/cmd_vel')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.scan_topic = rospy.get_param('~scan_topic', '/boat/scan_raw')
        self.filtered_scan_topic = rospy.get_param('~filtered_scan_topic', '/boat/scan')
        self.scan_range_topic = rospy.get_param('~scan_range_topic', '/boat/scan_range')
        self.boat_name = rospy.get_param('~boat_name', 'landing_boat')
        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'landing_boat/base_link')
        self.lidar_frame_id = rospy.get_param('~lidar_frame_id', 'landing_boat/hull/front_lidar')
        self.lidar_offset_x = float(rospy.get_param('~lidar_offset_x', 0.9075))
        self.lidar_offset_y = float(rospy.get_param('~lidar_offset_y', 0.0))
        self.lidar_offset_z = float(rospy.get_param('~lidar_offset_z', 1.5625))
        self.map_resolution = float(rospy.get_param('~map_resolution', 0.5))
        self.map_width = float(rospy.get_param('~map_width', 240.0))
        self.map_height = float(rospy.get_param('~map_height', 180.0))
        self.map_publish_period = float(
            rospy.get_param('~map_publish_period', 2.0)
        )
        self.publish_empty_map = bool(
            rospy.get_param('~publish_empty_map', True)
        )
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.8))
        self.marker_topic = rospy.get_param('~marker_topic', '/boat/nav2_reference_markers')
        self.control_frequency = max(
            1.0,
            float(rospy.get_param('~control_frequency', 20.0)),
        )
        self.enable_velocity_pid = bool(
            rospy.get_param('~enable_velocity_pid', True)
        )
        derivative_filter_alpha = float(
            rospy.get_param('~derivative_filter_alpha', 0.2)
        )
        self.velocity_measurement_alpha = clamp(
            float(rospy.get_param('~velocity_measurement_alpha', 0.3)),
            0.01,
            1.0,
        )
        self.linear_setpoint_alpha = clamp(
            float(rospy.get_param('~linear_setpoint_alpha', 0.45)),
            0.01,
            1.0,
        )
        self.angular_setpoint_alpha = clamp(
            float(rospy.get_param('~angular_setpoint_alpha', 0.6)),
            0.01,
            1.0,
        )
        self.max_linear_output = max(
            0.0,
            float(rospy.get_param('~max_linear_output', 2.8)),
        )
        self.speed_limit_mps = max(
            0.0,
            float(rospy.get_param('~speed_limit_mps', 0.0)),
        )
        self.speed_multiplier = max(
            0.25, float(rospy.get_param('~speed_multiplier', 1.0))
        )
        self.min_linear_output = min(
            0.0,
            float(rospy.get_param('~min_linear_output', -0.65)),
        )
        self.max_angular_output = max(
            0.0,
            float(rospy.get_param('~max_angular_output', 2.2)),
        )
        self.command_deadband = max(
            0.0,
            float(rospy.get_param('~command_deadband', 0.01)),
        )
        self.enable_lidar_safety = bool(
            rospy.get_param('~enable_lidar_safety', True)
        )
        self.safety_slow_distance = max(
            0.1,
            float(rospy.get_param('~safety_slow_distance', 9.0)),
        )
        self.safety_stop_distance = clamp(
            float(rospy.get_param('~safety_stop_distance', 3.8)),
            0.1,
            self.safety_slow_distance,
        )
        self.safety_escape_distance = clamp(
            float(rospy.get_param('~safety_escape_distance', 2.2)),
            0.1,
            self.safety_stop_distance,
        )
        self.safety_release_distance = max(
            self.safety_stop_distance,
            float(rospy.get_param('~safety_release_distance', 4.8)),
        )
        self.safety_reverse_speed = clamp(
            float(rospy.get_param('~safety_reverse_speed', -0.42)),
            self.min_linear_output,
            0.0,
        )
        self.safety_turn_rate = clamp(
            abs(float(rospy.get_param('~safety_turn_rate', 0.9))),
            0.1,
            self.max_angular_output,
        )
        self.safety_blocked_timeout = max(
            0.1,
            float(rospy.get_param('~safety_blocked_timeout', 1.5)),
        )
        self.safety_min_escape_duration = max(
            0.1,
            float(rospy.get_param('~safety_min_escape_duration', 2.5)),
        )
        self.safety_max_escape_duration = max(
            self.safety_min_escape_duration,
            float(rospy.get_param('~safety_max_escape_duration', 6.0)),
        )
        self.max_scan_tilt = max(
            0.0,
            float(rospy.get_param('~max_scan_tilt', 0.025)),
        )
        self.filter_wave_points = bool(
            rospy.get_param('~filter_wave_points', True)
        )
        self.min_obstacle_world_z = float(
            rospy.get_param('~min_obstacle_world_z', 0.9)
        )
        self.linear_pid = VelocityPid(
            float(rospy.get_param('~linear_kp', 0.18)),
            float(rospy.get_param('~linear_ki', 0.03)),
            float(rospy.get_param('~linear_kd', 0.01)),
            float(rospy.get_param('~linear_integral_limit', 0.5)),
            derivative_filter_alpha,
        )
        self.angular_pid = VelocityPid(
            float(rospy.get_param('~angular_kp', 0.28)),
            float(rospy.get_param('~angular_ki', 0.04)),
            float(rospy.get_param('~angular_kd', 0.015)),
            float(rospy.get_param('~angular_integral_limit', 0.8)),
            derivative_filter_alpha,
        )

        self.gz_node = GzTransportNode()
        self.gz_cmd_pub = self.gz_node.advertise(self.boat_cmd_topic, GzTwist)

        self.map_pub = None
        if self.publish_empty_map:
            self.map_pub = rospy.Publisher(
                self.map_topic,
                OccupancyGrid,
                queue_size=1,
                latch=True,
            )
        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=20,
        )
        self.scan_pub = rospy.Publisher(
            self.filtered_scan_topic,
            LaserScan,
            queue_size=10,
        )
        self.scan_range_pub = rospy.Publisher(
            self.scan_range_topic,
            LaserScan,
            queue_size=1,
        )
        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.tf_broadcaster = TransformBroadcaster()

        self.cmd_sub = rospy.Subscriber(
            self.cmd_vel_topic,
            RosTwist,
            self._on_cmd_vel,
            queue_size=10,
        )
        self.scan_sub = rospy.Subscriber(
            self.scan_topic,
            LaserScan,
            self._on_scan,
            queue_size=10,
        )

        self.boat_pose = None
        self.previous_pose = None
        self.previous_pose_time = None
        self.last_cmd_time = time.monotonic()
        self.last_marker_time = 0.0
        self.target_cmd = (0.0, 0.0)
        self.filtered_setpoint = [0.0, 0.0]
        self.filtered_velocity = [0.0, 0.0]
        self.last_output_cmd = (0.0, 0.0)
        self.last_control_time = time.monotonic()
        self.command_timed_out = False
        self.forward_clearance = math.inf
        self.left_clearance = math.inf
        self.right_clearance = math.inf
        self.safety_escape_active = False
        self.safety_turn_direction = 1.0
        self.safety_blocked_duration = 0.0
        self.safety_escape_started = 0.0
        self.empty_map = (
            self.make_empty_map() if self.publish_empty_map else None
        )

        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)
        self.gz_node.subscribe(Pose, self.model_pose_topic, self._on_model_pose)

        self.map_timer = None
        if self.publish_empty_map:
            self.map_timer = rospy.Timer(
                rospy.Duration(self.map_publish_period),
                self.publish_map,
            )
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_frequency),
            self.update_velocity_control,
        )
        self.marker_timer = rospy.Timer(
            rospy.Duration(1.0),
            self.publish_reference_markers,
        )

        if self.publish_empty_map:
            self.publish_map()
        rospy.loginfo(
            'Nav2 interface ready: %s -> PID -> %s, odom=%s, '
            'empty_map=%s, scan=%s -> %s, pose=%s, velocity_pid=%s.'
            % (
                self.cmd_vel_topic,
                self.boat_cmd_topic,
                self.odom_topic,
                self.publish_empty_map,
                self.scan_topic,
                self.filtered_scan_topic,
                self.model_pose_topic,
                self.enable_velocity_pid,
            )
        )

    def shutdown(self):
        try:
            self.gz_node.unsubscribe(self.pose_topic)
            self.gz_node.unsubscribe(self.model_pose_topic)
        except Exception:
            pass
        self.publish_gz_cmd(0.0, 0.0)

    def _on_pose_v(self, msg):
        for pose in msg.pose:
            if pose.name != self.boat_name:
                continue
            self.process_pose(pose)
            return

    def _on_model_pose(self, msg):
        self.process_pose(msg)

    def process_pose(self, pose):
        if not not rospy.is_shutdown():
            return
        now = rospy.Time.now()
        stamp = now
        dt = None
        if self.previous_pose_time is not None:
            dt = (now - self.previous_pose_time).to_sec()

        vx = 0.0
        wz = 0.0
        if self.previous_pose is not None and dt and dt > 1e-4:
            dx = pose.position.x - self.previous_pose.position.x
            dy = pose.position.y - self.previous_pose.position.y
            yaw = yaw_from_quaternion(pose.orientation)
            prev_yaw = yaw_from_quaternion(self.previous_pose.orientation)
            vx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt
            yaw_delta = math.atan2(
                math.sin(yaw - prev_yaw),
                math.cos(yaw - prev_yaw),
            )
            wz = yaw_delta / dt

        self.boat_pose = pose
        self.previous_pose = pose
        self.previous_pose_time = now
        alpha = self.velocity_measurement_alpha
        self.filtered_velocity[0] += alpha * (vx - self.filtered_velocity[0])
        self.filtered_velocity[1] += alpha * (wz - self.filtered_velocity[1])
        self.publish_tf_and_odom(pose, stamp, vx, wz)

    def _on_cmd_vel(self, msg):
        self.last_cmd_time = time.monotonic()
        self.command_timed_out = False
        max_forward = self.max_linear_output * self.speed_multiplier
        if self.speed_limit_mps > 0.0:
            max_forward = min(max_forward, self.speed_limit_mps)
        self.target_cmd = (
            clamp(
                float(msg.linear.x) * self.speed_multiplier,
                self.min_linear_output,
                max_forward,
            ),
            clamp(
                float(msg.angular.z),
                -self.max_angular_output,
                self.max_angular_output,
            ),
        )

    def _on_scan(self, msg):
        filtered_msg = copy.deepcopy(msg)
        filtered_ranges = list(msg.ranges)
        if self.filter_wave_points and self.boat_pose is not None:
            for index, distance in enumerate(filtered_ranges):
                if (
                    not math.isfinite(distance)
                    or distance < msg.range_min
                    or distance > msg.range_max
                ):
                    continue
                angle = msg.angle_min + index * msg.angle_increment
                if (
                    self.scan_endpoint_world_z(distance, angle)
                    < self.min_obstacle_world_z
                ):
                    filtered_ranges[index] = math.inf
        filtered_msg.ranges = filtered_ranges

        sectors = {'front': [], 'left': [], 'right': []}
        for index, distance in enumerate(filtered_ranges):
            if (
                not math.isfinite(distance)
                or distance < msg.range_min
                or distance > msg.range_max
            ):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) <= 0.42:
                sectors['front'].append(distance)
            if 0.25 <= angle <= 1.35:
                sectors['left'].append(distance)
            if -1.35 <= angle <= -0.25:
                sectors['right'].append(distance)

        self.forward_clearance = self.robust_sector_min(sectors['front'])
        self.left_clearance = self.robust_sector_min(sectors['left'])
        self.right_clearance = self.robust_sector_min(sectors['right'])
        self.scan_pub.publish(filtered_msg)

        scan_range = LaserScan()
        scan_range.header = msg.header
        scan_range.angle_min = msg.angle_min
        scan_range.angle_max = msg.angle_max
        scan_range.angle_increment = msg.angle_increment
        scan_range.time_increment = msg.time_increment
        scan_range.scan_time = msg.scan_time
        scan_range.range_min = msg.range_min
        scan_range.range_max = msg.range_max
        scan_range.ranges = [msg.range_max] * len(msg.ranges)
        scan_range.intensities = [0.0] * len(msg.ranges)
        self.scan_range_pub.publish(scan_range)

    def scan_endpoint_world_z(self, distance, angle):
        q = self.boat_pose.orientation
        local_x = self.lidar_offset_x + distance * math.cos(angle)
        local_y = self.lidar_offset_y + distance * math.sin(angle)
        local_z = self.lidar_offset_z
        rotation_z_x = 2.0 * (q.x * q.z - q.w * q.y)
        rotation_z_y = 2.0 * (q.y * q.z + q.w * q.x)
        rotation_z_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        return (
            self.boat_pose.position.z
            + rotation_z_x * local_x
            + rotation_z_y * local_y
            + rotation_z_z * local_z
        )

    @staticmethod
    def robust_sector_min(values):
        if not values:
            return math.inf
        values.sort()
        return values[min(2, len(values) - 1)]

    def start_safety_escape(self, reason):
        self.safety_escape_active = True
        self.safety_escape_started = time.monotonic()
        self.safety_blocked_duration = 0.0
        self.safety_turn_direction = (
            1.0 if self.left_clearance >= self.right_clearance else -1.0
        )
        self.reset_velocity_control()
        rospy.logwarn(
            'Lidar safety escape (%s): obstacle %.2f m ahead, reversing %s.'
            % (
                reason,
                self.forward_clearance,
                'left' if self.safety_turn_direction > 0.0 else 'right',
            )
        )

    def apply_lidar_safety(self, linear_target, angular_target, dt):
        if not self.enable_lidar_safety:
            return linear_target, angular_target

        if self.safety_escape_active:
            elapsed = time.monotonic() - self.safety_escape_started
            can_release = (
                elapsed >= self.safety_min_escape_duration
                and self.forward_clearance >= self.safety_release_distance
            )
            if can_release or elapsed >= self.safety_max_escape_duration:
                self.safety_escape_active = False
                rospy.loginfo('Lidar safety escape completed.')
            else:
                return (
                    self.safety_reverse_speed,
                    self.safety_turn_direction * self.safety_turn_rate,
                )

        if (
            linear_target >= 0.0
            and self.forward_clearance < self.safety_escape_distance
        ):
            self.start_safety_escape('distance')
            return (
                self.safety_reverse_speed,
                self.safety_turn_direction * self.safety_turn_rate,
            )

        near_shore_and_still = (
            self.forward_clearance < self.safety_stop_distance
            and abs(self.filtered_velocity[0]) < 0.08
            and abs(self.filtered_velocity[1]) < 0.12
        )
        if near_shore_and_still:
            self.safety_blocked_duration += dt
        else:
            self.safety_blocked_duration = 0.0
        if self.safety_blocked_duration >= self.safety_blocked_timeout:
            self.start_safety_escape('blocked')
            return (
                self.safety_reverse_speed,
                self.safety_turn_direction * self.safety_turn_rate,
            )

        if linear_target > 0.0:
            if self.forward_clearance < self.safety_stop_distance:
                linear_target = 0.0
                if abs(angular_target) < self.safety_turn_rate:
                    direction = (
                        1.0
                        if self.left_clearance >= self.right_clearance
                        else -1.0
                    )
                    angular_target = direction * self.safety_turn_rate
            elif self.forward_clearance < self.safety_slow_distance:
                span = self.safety_slow_distance - self.safety_stop_distance
                scale = (
                    self.forward_clearance - self.safety_stop_distance
                ) / max(span, 1e-6)
                linear_target *= clamp(scale, 0.0, 1.0)

        return linear_target, angular_target

    def publish_gz_cmd(self, linear_x, angular_z):
        msg = GzTwist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.gz_cmd_pub.publish(msg)

    def update_velocity_control(self, _event=None):
        now = time.monotonic()
        dt = clamp(now - self.last_control_time, 1e-3, 0.2)
        self.last_control_time = now

        if now - self.last_cmd_time > self.cmd_timeout:
            if not self.safety_escape_active and not self.command_timed_out:
                self.command_timed_out = True
                self.reset_velocity_control()
                self.publish_gz_cmd(0.0, 0.0)
            if not self.safety_escape_active:
                return
            self.target_cmd = (0.0, 0.0)

        linear_target, angular_target = self.target_cmd
        linear_target, angular_target = self.apply_lidar_safety(
            linear_target,
            angular_target,
            dt,
        )
        self.filtered_setpoint[0] += self.linear_setpoint_alpha * (
            linear_target - self.filtered_setpoint[0]
        )
        self.filtered_setpoint[1] += self.angular_setpoint_alpha * (
            angular_target - self.filtered_setpoint[1]
        )

        if self.enable_velocity_pid and self.boat_pose is not None:
            linear_error = (
                self.filtered_setpoint[0] - self.filtered_velocity[0]
            )
            angular_error = (
                self.filtered_setpoint[1] - self.filtered_velocity[1]
            )
            linear_output = (
                self.filtered_setpoint[0]
                + self.linear_pid.update(linear_error, dt)
            )
            angular_output = (
                self.filtered_setpoint[1]
                + self.angular_pid.update(angular_error, dt)
            )
        else:
            linear_output = self.filtered_setpoint[0]
            angular_output = self.filtered_setpoint[1]

        linear_output = clamp(
            linear_output,
            self.min_linear_output,
            self.max_linear_output,
        )
        angular_output = clamp(
            angular_output,
            -self.max_angular_output,
            self.max_angular_output,
        )
        if (
            abs(linear_target) <= self.command_deadband
            and abs(self.filtered_setpoint[0]) <= self.command_deadband
        ):
            linear_output = 0.0
            self.linear_pid.reset()
        if (
            abs(angular_target) <= self.command_deadband
            and abs(self.filtered_setpoint[1]) <= self.command_deadband
        ):
            angular_output = 0.0
            self.angular_pid.reset()

        self.last_output_cmd = (linear_output, angular_output)
        self.publish_gz_cmd(linear_output, angular_output)

    def reset_velocity_control(self):
        self.target_cmd = (0.0, 0.0)
        self.filtered_setpoint = [0.0, 0.0]
        self.last_output_cmd = (0.0, 0.0)
        self.linear_pid.reset()
        self.angular_pid.reset()

    def publish_tf_and_odom(self, pose, stamp, vx, wz):
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = self.map_frame_id
        map_to_odom.child_frame_id = self.odom_frame_id
        map_to_odom.transform.rotation.w = 1.0

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame_id
        odom_to_base.child_frame_id = self.base_frame_id
        odom_to_base.transform.translation.x = pose.position.x
        odom_to_base.transform.translation.y = pose.position.y
        odom_to_base.transform.translation.z = pose.position.z
        odom_to_base.transform.rotation.x = pose.orientation.x
        odom_to_base.transform.rotation.y = pose.orientation.y
        odom_to_base.transform.rotation.z = pose.orientation.z
        odom_to_base.transform.rotation.w = pose.orientation.w

        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = stamp
        base_to_lidar.header.frame_id = self.base_frame_id
        base_to_lidar.child_frame_id = self.lidar_frame_id
        base_to_lidar.transform.translation.x = self.lidar_offset_x
        base_to_lidar.transform.translation.y = self.lidar_offset_y
        base_to_lidar.transform.translation.z = self.lidar_offset_z
        base_to_lidar.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(
            [map_to_odom, odom_to_base, base_to_lidar]
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = pose.position.x
        odom.pose.pose.position.y = pose.position.y
        odom.pose.pose.position.z = pose.position.z
        odom.pose.pose.orientation.x = pose.orientation.x
        odom.pose.pose.orientation.y = pose.orientation.y
        odom.pose.pose.orientation.z = pose.orientation.z
        odom.pose.pose.orientation.w = pose.orientation.w
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def make_empty_map(self):
        width = max(1, int(round(self.map_width / self.map_resolution)))
        height = max(1, int(round(self.map_height / self.map_resolution)))
        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame_id
        grid.info.resolution = self.map_resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = -0.5 * width * self.map_resolution
        grid.info.origin.position.y = -0.5 * height * self.map_resolution
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (width * height)
        self.mark_static_landmarks(grid)
        return grid

    def mark_static_landmarks(self, grid):
        landmarks = [
            (35.0, 18.0, 2.8),
            (-42.0, 44.0, 1.8),
            (34.0, -56.0, 1.8),
            (78.0, 28.0, 1.8),
        ]
        for x, y, radius in landmarks:
            self.mark_occupied_circle(grid, x, y, radius)

    def mark_occupied_circle(self, grid, world_x, world_y, radius):
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        center_x = int((world_x - origin_x) / resolution)
        center_y = int((world_y - origin_y) / resolution)
        radius_cells = max(1, int(math.ceil(radius / resolution)))

        for cy in range(center_y - radius_cells, center_y + radius_cells + 1):
            if cy < 0 or cy >= grid.info.height:
                continue
            for cx in range(center_x - radius_cells, center_x + radius_cells + 1):
                if cx < 0 or cx >= grid.info.width:
                    continue
                dx = (cx - center_x) * resolution
                dy = (cy - center_y) * resolution
                if math.hypot(dx, dy) <= radius:
                    grid.data[cy * grid.info.width + cx] = 100

    def publish_map(self, _event=None):
        if self.map_pub is None or self.empty_map is None:
            return
        self.empty_map.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.empty_map)

    def publish_reference_markers(self, _event=None):
        now = time.monotonic()
        if now - self.last_marker_time < 0.9:
            return
        self.last_marker_time = now
        stamp = rospy.Time.now()
        markers = MarkerArray()

        markers.markers.append(
            self.make_marker(1, 'navigation_area', Marker.CUBE, 0.0, 0.0, -0.03,
                             240.0, 180.0, 0.02, 0.02, 0.18, 0.32, 0.18, stamp)
        )
        markers.markers.append(
            self.make_marker(2, 'static_obstacle_footprints', Marker.CYLINDER,
                             35.0, 18.0, 0.18, 6.0, 6.0, 0.35,
                             1.0, 0.82, 0.1, 0.95, stamp)
        )
        markers.markers.append(
            self.make_marker(3, 'static_obstacle_volume', Marker.CYLINDER,
                             35.0, 18.0, 5.0, 4.4, 4.4, 10.0,
                             0.95, 0.86, 0.32, 0.55, stamp)
        )
        markers.markers.append(
            self.make_text(4, 'labels', 'Lighthouse', 35.0, 18.0, 11.0, 1.8, stamp)
        )
        for idx, (x, y, label) in enumerate(
            [(-42.0, 44.0, 'Buoy A'), (34.0, -56.0, 'Buoy B'), (78.0, 28.0, 'Buoy C')]
        ):
            marker_id = 10 + idx * 3
            markers.markers.append(
                self.make_marker(marker_id, 'static_obstacle_footprints',
                                 Marker.CYLINDER, x, y, 0.16, 4.0, 4.0, 0.32,
                                 1.0, 0.12, 0.08, 0.95, stamp)
            )
            markers.markers.append(
                self.make_marker(marker_id + 1, 'static_obstacle_volume',
                                 Marker.CYLINDER, x, y, 2.0, 3.0, 3.0, 4.0,
                                 0.92, 0.18, 0.12, 0.55, stamp)
            )
            markers.markers.append(
                self.make_text(marker_id + 2, 'labels', label, x, y, 5.0, 1.4, stamp)
            )
        self.marker_pub.publish(markers)

    def make_marker(self, marker_id, namespace, marker_type, x, y, z,
                    sx, sy, sz, r, g, b, a, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(sx)
        marker.scale.y = float(sy)
        marker.scale.z = float(sz)
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(a)
        marker.lifetime = rospy.Duration(0.0)
        return marker

    def make_text(self, marker_id, namespace, text, x, y, z, scale, stamp):
        marker = self.make_marker(
            marker_id, namespace, Marker.TEXT_VIEW_FACING, x, y, z,
            scale, scale, scale, 1.0, 1.0, 1.0, 0.95, stamp)
        marker.text = text
        return marker


def main():
    rospy.init_node('boat_move_base_interface')

    node = BoatNav2Interface()
    rospy.on_shutdown(node.shutdown)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
