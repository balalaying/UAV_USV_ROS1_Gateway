#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


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


def rotate_xy(x, y, yaw):
    return (
        math.cos(yaw) * x - math.sin(yaw) * y,
        math.sin(yaw) * x + math.cos(yaw) * y,
    )


class RvizGoalBoatControl(Node):

    def __init__(self):
        super().__init__('rviz_goal_boat_control')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('pose_topic', '/world/default/pose/info')
        self.declare_parameter('boat_cmd_topic', '/model/simple_boat/cmd_vel')
        self.declare_parameter('scan_topic', '/boat/scan')
        self.declare_parameter('scan_range_topic', '/boat/scan_range')
        self.declare_parameter('boat_name', 'landing_boat')
        self.declare_parameter('accepted_goal_frames', ['map', 'world', 'default'])
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('arrival_radius', 0.8)
        self.declare_parameter('slow_radius', 5.0)
        self.declare_parameter('max_speed', 2.6)
        self.declare_parameter('min_speed', 0.35)
        self.declare_parameter('turn_gain', 1.5)
        self.declare_parameter('max_turn', 1.8)
        self.declare_parameter('heading_slowdown_yaw', 1.0)
        self.declare_parameter('stale_pose_timeout', 1.0)
        self.declare_parameter('marker_topic', '/boat/navigation_markers')
        self.declare_parameter('enable_avoidance', True)
        self.declare_parameter('obstacle_slow_distance', 18.0)
        self.declare_parameter('obstacle_stop_distance', 4.0)
        self.declare_parameter('obstacle_turn_gain', 2.8)
        self.declare_parameter('obstacle_clear_distance', 21.0)
        self.declare_parameter('avoidance_hold_time', 3.0)
        self.declare_parameter('avoidance_filter_alpha', 0.35)
        self.declare_parameter('avoidance_min_speed', 0.55)
        self.declare_parameter('scan_stale_timeout', 1.0)
        self.declare_parameter('lidar_offset_x', 0.9075)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', 1.5625)
        self.declare_parameter('boat_frame_id', 'landing_boat/base_link')
        self.declare_parameter('lidar_frame_id', 'landing_boat/hull/front_lidar')

        self.goal_topic = self.get_parameter('goal_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.boat_cmd_topic = self.get_parameter('boat_cmd_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_range_topic = self.get_parameter('scan_range_topic').value
        self.boat_name = self.get_parameter('boat_name').value
        self.accepted_goal_frames = set(
            self.get_parameter('accepted_goal_frames').value
        )
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius').value)
        self.slow_radius = float(self.get_parameter('slow_radius').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.heading_slowdown_yaw = float(
            self.get_parameter('heading_slowdown_yaw').value
        )
        self.stale_pose_timeout = float(
            self.get_parameter('stale_pose_timeout').value
        )
        self.marker_topic = self.get_parameter('marker_topic').value
        self.enable_avoidance = bool(self.get_parameter('enable_avoidance').value)
        self.obstacle_slow_distance = float(
            self.get_parameter('obstacle_slow_distance').value
        )
        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value
        )
        self.obstacle_turn_gain = float(
            self.get_parameter('obstacle_turn_gain').value
        )
        self.obstacle_clear_distance = float(
            self.get_parameter('obstacle_clear_distance').value
        )
        self.avoidance_hold_time = float(
            self.get_parameter('avoidance_hold_time').value
        )
        self.avoidance_filter_alpha = float(
            self.get_parameter('avoidance_filter_alpha').value
        )
        self.avoidance_min_speed = float(
            self.get_parameter('avoidance_min_speed').value
        )
        self.scan_stale_timeout = float(
            self.get_parameter('scan_stale_timeout').value
        )
        self.lidar_offset_x = float(self.get_parameter('lidar_offset_x').value)
        self.lidar_offset_y = float(self.get_parameter('lidar_offset_y').value)
        self.lidar_offset_z = float(self.get_parameter('lidar_offset_z').value)
        self.boat_frame_id = self.get_parameter('boat_frame_id').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value

        self.gz_node = GzTransportNode()
        self.boat_pub = self.gz_node.advertise(self.boat_cmd_topic, Twist)
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)
        self.scan_range_pub = self.create_publisher(
            LaserScan,
            self.scan_range_topic,
            1,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self._on_goal,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            10,
        )
        self.timer = self.create_timer(1.0 / self.control_rate, self._on_timer)

        self.boat_pose = None
        self.last_pose_time = 0.0
        self.last_scan_time = 0.0
        self.scan = None
        self.goal_xy = None
        self.last_log_time = 0.0
        self.avoidance_direction = 0.0
        self.avoidance_last_active_time = 0.0
        self.avoidance_turn_state = 0.0
        self.angular_cmd_state = 0.0
        self.arrived = False

        self.get_logger().info(
            'Waiting for RViz goals on %s. Use 2D Goal Pose in frame map/world.'
            % self.goal_topic
        )
        self.get_logger().info(
            'Reading boat pose from %s and publishing Gazebo Twist to %s.'
            % (self.pose_topic, self.boat_cmd_topic)
        )
        self.get_logger().info(
            'Publishing RViz navigation reference markers on %s.'
            % self.marker_topic
        )
        self.get_logger().info(
            'Local obstacle avoidance is %s; reading LaserScan from %s.'
            % ('enabled' if self.enable_avoidance else 'disabled', self.scan_topic)
        )

    def destroy_node(self):
        self.publish_cmd(0.0, 0.0)
        super().destroy_node()

    def _on_pose_v(self, msg):
        for pose in msg.pose:
            if pose.name == self.boat_name:
                self.boat_pose = pose
                self.last_pose_time = time.monotonic()
                self.publish_lidar_tf()
                return

    def _on_goal(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')
        if frame_id and frame_id not in self.accepted_goal_frames:
            self.get_logger().warn(
                'Goal frame "%s" is not in %s; treating coordinates as Gazebo world XY.'
                % (frame_id, sorted(self.accepted_goal_frames))
            )

        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.arrived = False
        self.get_logger().info(
            'New boat goal: x=%.2f y=%.2f frame=%s'
            % (self.goal_xy[0], self.goal_xy[1], frame_id or '<empty>')
        )

    def _on_scan(self, msg):
        self.scan = msg
        self.last_scan_time = time.monotonic()
        self.publish_scan_range(msg)

    def _on_timer(self):
        self.publish_markers()
        self.publish_lidar_tf()

        if self.goal_xy is None:
            self.publish_cmd(0.0, 0.0)
            return

        if self.boat_pose is None:
            self.throttled_log('Waiting for Gazebo boat pose...')
            self.publish_cmd(0.0, 0.0)
            return

        if time.monotonic() - self.last_pose_time > self.stale_pose_timeout:
            self.throttled_log('Boat pose is stale; stopping.')
            self.publish_cmd(0.0, 0.0)
            return

        boat_x = self.boat_pose.position.x
        boat_y = self.boat_pose.position.y
        goal_x, goal_y = self.goal_xy
        dx = goal_x - boat_x
        dy = goal_y - boat_y
        distance = math.hypot(dx, dy)

        if distance <= self.arrival_radius:
            self.publish_cmd(0.0, 0.0)
            if not self.arrived:
                self.get_logger().info(
                    'Arrived at RViz goal: distance=%.2f m' % distance
                )
                self.arrived = True
            return

        yaw = yaw_from_quaternion(self.boat_pose.orientation)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = wrap_pi(desired_yaw - yaw)

        speed_scale = clamp(distance / max(self.slow_radius, 0.01), 0.0, 1.0)
        linear_x = self.max_speed * speed_scale
        linear_x = max(self.min_speed, linear_x)

        heading_scale = clamp(
            1.0 - abs(yaw_error) / max(self.heading_slowdown_yaw, 0.01),
            0.15,
            1.0,
        )
        linear_x *= heading_scale
        angular_z = clamp(
            self.turn_gain * yaw_error,
            -self.max_turn,
            self.max_turn,
        )
        linear_x, angular_z, avoidance_text = self.apply_obstacle_avoidance(
            linear_x,
            angular_z,
        )
        angular_z = self.filter_angular_command(angular_z)

        self.publish_cmd(linear_x, angular_z)
        suffix = ''
        if avoidance_text:
            suffix = ' avoidance=' + avoidance_text
        self.throttled_log(
            'Tracking goal: boat=(%.1f, %.1f) goal=(%.1f, %.1f) '
            'distance=%.1f yaw_error=%.2f cmd=(%.2f, %.2f)%s'
            % (
                boat_x,
                boat_y,
                goal_x,
                goal_y,
                distance,
                yaw_error,
                linear_x,
                angular_z,
                suffix,
            )
        )

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.boat_pub.publish(msg)

    def publish_scan_range(self, scan):
        range_scan = LaserScan()
        range_scan.header = scan.header
        range_scan.angle_min = scan.angle_min
        range_scan.angle_max = scan.angle_max
        range_scan.angle_increment = scan.angle_increment
        range_scan.time_increment = scan.time_increment
        range_scan.scan_time = scan.scan_time
        range_scan.range_min = scan.range_min
        range_scan.range_max = scan.range_max
        range_scan.ranges = [scan.range_max] * len(scan.ranges)
        range_scan.intensities = [0.0] * len(scan.ranges)
        self.scan_range_pub.publish(range_scan)

    def apply_obstacle_avoidance(self, linear_x, angular_z):
        if not self.enable_avoidance or self.scan is None:
            return linear_x, angular_z, ''

        if time.monotonic() - self.last_scan_time > self.scan_stale_timeout:
            return linear_x, angular_z, ''

        sectors = self.scan_sectors()
        if sectors is None:
            return linear_x, angular_z, ''

        front_min = sectors['front']
        left_min = sectors['left']
        right_min = sectors['right']
        if not math.isfinite(front_min):
            self.decay_avoidance_state()
            return linear_x, angular_z, ''

        now = time.monotonic()
        obstacle_active = front_min <= self.obstacle_slow_distance
        recently_active = now - self.avoidance_last_active_time <= self.avoidance_hold_time
        clear_enough = front_min > self.obstacle_clear_distance

        if not obstacle_active and (not recently_active or clear_enough):
            self.avoidance_direction = 0.0
            self.decay_avoidance_state()
            return linear_x, angular_z, ''

        if obstacle_active:
            self.avoidance_last_active_time = now
            proposed_direction = self.choose_avoidance_direction(left_min, right_min)
            if self.avoidance_direction == 0.0:
                self.avoidance_direction = proposed_direction

        clearance_span = max(
            self.obstacle_slow_distance - self.obstacle_stop_distance,
            0.01,
        )
        danger = clamp(
            (self.obstacle_slow_distance - front_min) / clearance_span,
            0.0,
            1.0,
        )
        speed_scale = clamp(
            (front_min - self.obstacle_stop_distance) / clearance_span,
            0.0,
            1.0,
        )
        linear_x *= speed_scale

        if recently_active and not obstacle_active:
            danger = max(danger, 0.25)

        target_turn = self.avoidance_direction * self.max_turn * clamp(
            0.35 + 0.65 * danger,
            0.35,
            1.0,
        )
        alpha = clamp(self.avoidance_filter_alpha, 0.01, 1.0)
        self.avoidance_turn_state = (
            (1.0 - alpha) * self.avoidance_turn_state
            + alpha * target_turn
        )

        if front_min <= self.obstacle_stop_distance:
            linear_x = max(min(linear_x, self.avoidance_min_speed), 0.25)
            angular_z = clamp(self.avoidance_turn_state, -self.max_turn, self.max_turn)
        else:
            linear_x = max(linear_x, self.avoidance_min_speed)
            angular_z = clamp(
                self.avoidance_turn_state,
                -self.max_turn,
                self.max_turn,
            )

        return (
            linear_x,
            angular_z,
            'front=%.1f left=%.1f right=%.1f'
            % (front_min, left_min, right_min),
        )

    def choose_avoidance_direction(self, left_min, right_min):
        left_clearance = left_min if math.isfinite(left_min) else self.obstacle_clear_distance
        right_clearance = right_min if math.isfinite(right_min) else self.obstacle_clear_distance
        if left_clearance < right_clearance:
            return -1.0
        return 1.0

    def decay_avoidance_state(self):
        self.avoidance_turn_state *= 0.75
        if abs(self.avoidance_turn_state) < 0.02:
            self.avoidance_turn_state = 0.0

    def filter_angular_command(self, angular_z):
        alpha = clamp(self.avoidance_filter_alpha, 0.01, 1.0)
        self.angular_cmd_state = (
            (1.0 - alpha) * self.angular_cmd_state + alpha * angular_z
        )
        return clamp(self.angular_cmd_state, -self.max_turn, self.max_turn)

    def scan_sectors(self):
        if self.scan is None:
            return None

        front = []
        left = []
        right = []
        angle = self.scan.angle_min
        for value in self.scan.ranges:
            valid = (
                math.isfinite(value)
                and value >= self.scan.range_min
                and value <= self.scan.range_max
            )
            if valid:
                abs_angle = abs(angle)
                if abs_angle <= math.radians(35.0):
                    front.append(value)
                if math.radians(15.0) <= angle <= math.radians(85.0):
                    left.append(value)
                if math.radians(-85.0) <= angle <= math.radians(-15.0):
                    right.append(value)
            angle += self.scan.angle_increment

        return {
            'front': min(front) if front else math.inf,
            'left': min(left) if left else math.inf,
            'right': min(right) if right else math.inf,
        }

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        markers.markers.append(
            self.make_marker(
                0,
                'navigation_area',
                Marker.CUBE,
                18.0,
                0.0,
                -0.04,
                170.0,
                130.0,
                0.02,
                0.02,
                0.18,
                0.32,
                0.22,
                stamp,
            )
        )
        markers.markers.append(
            self.make_marker(
                1,
                'lighthouse',
                Marker.CYLINDER,
                35.0,
                18.0,
                5.0,
                4.4,
                4.4,
                10.0,
                0.95,
                0.86,
                0.32,
                0.9,
                stamp,
            )
        )
        markers.markers.append(
            self.make_text_marker(
                2,
                'lighthouse_label',
                'Lighthouse',
                35.0,
                18.0,
                10.8,
                1.6,
                stamp,
            )
        )

        buoy_positions = [
            (-42.0, 44.0, 'Buoy A'),
            (34.0, -56.0, 'Buoy B'),
            (78.0, 28.0, 'Buoy C'),
        ]
        marker_id = 10
        for x, y, label in buoy_positions:
            markers.markers.append(
                self.make_marker(
                    marker_id,
                    'buoys',
                    Marker.CYLINDER,
                    x,
                    y,
                    2.0,
                    3.0,
                    3.0,
                    4.0,
                    0.92,
                    0.18,
                    0.12,
                    0.85,
                    stamp,
                )
            )
            markers.markers.append(
                self.make_text_marker(
                    marker_id + 1,
                    'buoy_labels',
                    label,
                    x,
                    y,
                    4.8,
                    1.3,
                    stamp,
                )
            )
            marker_id += 2

        if self.boat_pose is not None:
            boat_marker = self.make_marker(
                30,
                'boat',
                Marker.ARROW,
                self.boat_pose.position.x,
                self.boat_pose.position.y,
                0.8,
                4.0,
                1.2,
                1.2,
                0.05,
                0.75,
                0.95,
                1.0,
                stamp,
            )
            boat_marker.pose.orientation.x = self.boat_pose.orientation.x
            boat_marker.pose.orientation.y = self.boat_pose.orientation.y
            boat_marker.pose.orientation.z = self.boat_pose.orientation.z
            boat_marker.pose.orientation.w = self.boat_pose.orientation.w
            markers.markers.append(boat_marker)
            markers.markers.append(
                self.make_text_marker(
                    31,
                    'boat_label',
                    'Boat',
                    self.boat_pose.position.x,
                    self.boat_pose.position.y,
                    3.0,
                    1.2,
                    stamp,
                )
            )

        if self.goal_xy is not None:
            goal_x, goal_y = self.goal_xy
            markers.markers.append(
                self.make_marker(
                    40,
                    'goal',
                    Marker.SPHERE,
                    goal_x,
                    goal_y,
                    0.8,
                    2.2,
                    2.2,
                    2.2,
                    0.1,
                    1.0,
                    0.25,
                    1.0,
                    stamp,
                )
            )
            markers.markers.append(
                self.make_text_marker(
                    41,
                    'goal_label',
                    'Goal',
                    goal_x,
                    goal_y,
                    3.0,
                    1.4,
                    stamp,
                )
            )

        self.marker_pub.publish(markers)

    def publish_lidar_tf(self):
        if self.boat_pose is None:
            return

        stamp = self.get_clock().now().to_msg()
        boat_tf = TransformStamped()
        boat_tf.header.stamp = stamp
        boat_tf.header.frame_id = 'map'
        boat_tf.child_frame_id = self.boat_frame_id
        boat_tf.transform.translation.x = self.boat_pose.position.x
        boat_tf.transform.translation.y = self.boat_pose.position.y
        boat_tf.transform.translation.z = self.boat_pose.position.z
        boat_tf.transform.rotation.x = self.boat_pose.orientation.x
        boat_tf.transform.rotation.y = self.boat_pose.orientation.y
        boat_tf.transform.rotation.z = self.boat_pose.orientation.z
        boat_tf.transform.rotation.w = self.boat_pose.orientation.w

        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = stamp
        lidar_tf.header.frame_id = self.boat_frame_id
        lidar_tf.child_frame_id = self.lidar_frame_id
        lidar_tf.transform.translation.x = self.lidar_offset_x
        lidar_tf.transform.translation.y = self.lidar_offset_y
        lidar_tf.transform.translation.z = self.lidar_offset_z
        lidar_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform([boat_tf, lidar_tf])

    def make_marker(
        self,
        marker_id,
        namespace,
        marker_type,
        x,
        y,
        z,
        scale_x,
        scale_y,
        scale_z,
        red,
        green,
        blue,
        alpha,
        stamp,
    ):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale_x)
        marker.scale.y = float(scale_y)
        marker.scale.z = float(scale_z)
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)
        return marker

    def make_text_marker(self, marker_id, namespace, text, x, y, z, scale, stamp):
        marker = self.make_marker(
            marker_id,
            namespace,
            Marker.TEXT_VIEW_FACING,
            x,
            y,
            z,
            scale,
            scale,
            scale,
            1.0,
            1.0,
            1.0,
            0.95,
            stamp,
        )
        marker.text = text
        return marker

    def throttled_log(self, text, period=2.0):
        now = time.monotonic()
        if now - self.last_log_time >= period:
            self.get_logger().info(text)
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = RvizGoalBoatControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
