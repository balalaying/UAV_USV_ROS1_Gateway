#!/usr/bin/env python3
import math
import threading
import time

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from pymavlink import mavutil
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class UavBuoyVisualMission(Node):
    """Searches for a buoy from the UAV camera and coordinates UAV-USV pursuit."""

    def __init__(self):
        super().__init__('uav_buoy_visual_mission')

        self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14540')
        self.declare_parameter('heartbeat_timeout', 180.0)
        self.declare_parameter('pose_topic', '/world/default/pose/info')
        self.declare_parameter('camera_topic', '/uav/down_camera/image')
        self.declare_parameter('boat_camera_topic', '/boat/front_camera')
        self.declare_parameter(
            'detection_image_topic',
            '/uav/down_camera/detections',
        )
        self.declare_parameter(
            'camera_mosaic_topic',
            '/uav_usv/camera_mosaic',
        )
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('target_topic', '/uav/detected_target')
        self.declare_parameter('marker_topic', '/uav/visual_target_marker')
        self.declare_parameter('boat_name', 'landing_boat')
        self.declare_parameter('drone_name', 'x500_mono_cam_down_0')
        self.declare_parameter(
            'deck_release_topic',
            '/model/x500_0/release_from_deck',
        )
        self.declare_parameter('takeoff_altitude', 16.0)
        self.declare_parameter('takeoff_climb_rate', 1.0)
        self.declare_parameter('patrol_speed', 3.0)
        self.declare_parameter('target_speed', 3.2)
        self.declare_parameter('boat_standoff_distance', 7.0)
        self.declare_parameter('detection_max_distance', 14.0)
        self.declare_parameter('detection_confirm_frames', 5)
        self.declare_parameter('minimum_red_pixels', 20)
        self.declare_parameter('image_processing_rate', 24.0)
        self.declare_parameter('mosaic_panel_width', 480)
        self.declare_parameter('mosaic_panel_height', 360)
        self.declare_parameter(
            'patrol_waypoints',
            [
                20.0, 10.0,
                45.0, 20.0,
                70.0, 28.0,
                45.0, -18.0,
                34.0, -50.0,
                -18.0, 18.0,
                -40.0, 42.0,
            ],
        )

        self.mavlink_url = self.get_parameter('mavlink_url').value
        self.heartbeat_timeout = float(
            self.get_parameter('heartbeat_timeout').value
        )
        self.pose_topic = self.get_parameter('pose_topic').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.boat_camera_topic = self.get_parameter(
            'boat_camera_topic'
        ).value
        self.detection_image_topic = self.get_parameter(
            'detection_image_topic'
        ).value
        self.camera_mosaic_topic = self.get_parameter(
            'camera_mosaic_topic'
        ).value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.boat_name = self.get_parameter('boat_name').value
        self.drone_name = self.get_parameter('drone_name').value
        self.deck_release_topic = self.get_parameter(
            'deck_release_topic'
        ).value
        self.takeoff_altitude = float(
            self.get_parameter('takeoff_altitude').value
        )
        self.takeoff_climb_rate = float(
            self.get_parameter('takeoff_climb_rate').value
        )
        self.patrol_speed = float(self.get_parameter('patrol_speed').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.boat_standoff_distance = float(
            self.get_parameter('boat_standoff_distance').value
        )
        self.detection_max_distance = float(
            self.get_parameter('detection_max_distance').value
        )
        self.detection_confirm_frames = int(
            self.get_parameter('detection_confirm_frames').value
        )
        self.minimum_red_pixels = int(
            self.get_parameter('minimum_red_pixels').value
        )
        self.image_processing_period = 1.0 / max(
            1.0,
            float(self.get_parameter('image_processing_rate').value),
        )
        self.mosaic_panel_width = max(
            160,
            int(self.get_parameter('mosaic_panel_width').value),
        )
        self.mosaic_panel_height = max(
            120,
            int(self.get_parameter('mosaic_panel_height').value),
        )
        flat_waypoints = [
            float(value)
            for value in self.get_parameter('patrol_waypoints').value
        ]
        self.patrol_waypoints = list(
            zip(flat_waypoints[0::2], flat_waypoints[1::2])
        )

        self.gz_node = GzTransportNode()
        self.release_pub = self.gz_node.advertise(
            self.deck_release_topic,
            Boolean,
        )

        target_qos = QoSProfile(depth=1)
        target_qos.reliability = ReliabilityPolicy.RELIABLE
        target_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.goal_topic,
            10,
        )
        self.target_pub = self.create_publisher(
            PoseStamped,
            self.target_topic,
            target_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            target_qos,
        )
        self.detection_image_pub = self.create_publisher(
            Image,
            self.detection_image_topic,
            image_qos,
        )
        self.camera_mosaic_pub = self.create_publisher(
            Image,
            self.camera_mosaic_topic,
            image_qos,
        )
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self._on_image,
            image_qos,
        )
        self.boat_camera_sub = self.create_subscription(
            Image,
            self.boat_camera_topic,
            self._on_boat_image,
            image_qos,
        )

        self.stop_event = threading.Event()
        self.mav = None
        self.target_system = 1
        self.target_component = 1
        self.local_position = None
        self.local_origin_gz = None
        self.last_gcs_heartbeat = 0.0
        self.drone_sp = None
        self.drone_pose = None
        self.boat_pose = None
        self.buoy_poses = {}
        self.detected_target = None
        self.cooperative_boat_goal = None
        self.boat_image = None
        self.target_confirm_count = 0
        self.last_detection_log = 0.0
        self.last_uav_image_process = 0.0
        self.last_boat_image_process = 0.0
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)

        self.get_logger().info(
            'Visual patrol ready: camera=%s, waypoints=%d'
            % (self.camera_topic, len(self.patrol_waypoints))
        )

    def destroy_node(self):
        self.stop_event.set()
        self.gz_node.unsubscribe(self.pose_topic)
        super().destroy_node()

    def _on_pose_v(self, msg):
        for pose in msg.pose:
            if pose.name == self.boat_name:
                self.boat_pose = pose
            elif pose.name == self.drone_name:
                self.drone_pose = pose
            elif pose.name.startswith('medium_buoy_'):
                self.buoy_poses[pose.name] = pose

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self.last_uav_image_process < self.image_processing_period:
            return
        self.last_uav_image_process = now
        try:
            image = self.decode_image(msg)
        except Exception as exc:
            self.get_logger().warn(
                'Unable to decode UAV camera image: %s' % exc,
                throttle_duration_sec=5.0,
            )
            return
        image = cv2.resize(image, (640, 480))

        red_mask = cv2.inRange(image, (0, 0, 145), (125, 125, 255))
        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )
        contours, _ = cv2.findContours(
            red_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        center_x = 0.5 * image.shape[1]
        center_y = 0.5 * image.shape[0]
        center_limit = 0.32 * min(image.shape[:2])
        selected = None
        selected_distance = float('inf')
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.minimum_red_pixels:
                continue
            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1e-6:
                continue
            contour_x = moments['m10'] / moments['m00']
            contour_y = moments['m01'] / moments['m00']
            center_distance = math.hypot(
                contour_x - center_x,
                contour_y - center_y,
            )
            if (
                center_distance <= center_limit
                and center_distance < selected_distance
            ):
                selected = contour
                selected_distance = center_distance
        area = cv2.contourArea(selected) if selected is not None else 0.0
        candidate = self._nearest_visible_buoy()

        confirmed_in_frame = (
            selected is not None and candidate is not None
        )
        if confirmed_in_frame:
            self.target_confirm_count += 1
            x, y, width, height = cv2.boundingRect(selected)
            cv2.rectangle(
                image,
                (x, y),
                (x + width, y + height),
                (0, 255, 255),
                3,
            )
            cv2.putText(
                image,
                'BUOY %.0f px' % area,
                (x, max(28, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            if (
                self.detected_target is None
                and self.target_confirm_count >= self.detection_confirm_frames
            ):
                self.detected_target = candidate
                self.get_logger().info(
                    'Camera confirmed buoy %s at world x=%.2f y=%.2f'
                    % (
                        candidate[0],
                        candidate[1],
                        candidate[2],
                    )
                )
        else:
            self.target_confirm_count = max(0, self.target_confirm_count - 1)

        status = (
            'TARGET LOCKED'
            if self.detected_target is not None
            else 'PATROLLING - RED PIXELS %.0f' % area
        )
        cv2.putText(
            image,
            status,
            (20, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (40, 255, 40) if self.detected_target else (255, 220, 40),
            2,
            cv2.LINE_AA,
        )
        if self.detection_image_pub.get_subscription_count() > 0:
            annotated = self.make_image_message(image, msg.header)
            self.detection_image_pub.publish(annotated)
        if self.camera_mosaic_pub.get_subscription_count() > 0:
            self.publish_camera_mosaic(image, msg.header)

    def _on_boat_image(self, msg):
        now = time.monotonic()
        if now - self.last_boat_image_process < self.image_processing_period:
            return
        self.last_boat_image_process = now
        try:
            self.boat_image = self.decode_image(msg)
        except Exception as exc:
            self.get_logger().warn(
                'Unable to decode boat camera image: %s' % exc,
                throttle_duration_sec=5.0,
            )

    def publish_camera_mosaic(self, uav_image, header):
        panel_height = self.mosaic_panel_height
        panel_width = self.mosaic_panel_width
        if self.boat_image is None:
            boat_panel = np.zeros(
                (panel_height, panel_width, 3),
                dtype=np.uint8,
            )
            cv2.putText(
                boat_panel,
                'WAITING FOR BOAT CAMERA',
                (80, 245),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            boat_panel = cv2.resize(
                self.boat_image,
                (panel_width, panel_height),
                interpolation=cv2.INTER_AREA,
            )
        uav_panel = cv2.resize(
            uav_image,
            (panel_width, panel_height),
            interpolation=cv2.INTER_AREA,
        )
        cv2.rectangle(boat_panel, (0, 0), (panel_width, 42), (0, 0, 0), -1)
        cv2.rectangle(uav_panel, (0, 0), (panel_width, 42), (0, 0, 0), -1)
        cv2.putText(
            boat_panel,
            'USV FRONT CAMERA',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            uav_panel,
            'UAV DOWN CAMERA + DETECTION',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        mosaic = np.hstack((boat_panel, uav_panel))
        message = self.make_image_message(mosaic, header)
        self.camera_mosaic_pub.publish(message)

    @staticmethod
    def make_image_message(image, header):
        message = Image()
        message.header = header
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = image.shape[1] * 3
        message.data = image.tobytes()
        return message

    @staticmethod
    def decode_image(msg):
        channels_by_encoding = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
        }
        channels = channels_by_encoding.get(msg.encoding.lower())
        if channels is None:
            raise ValueError('unsupported image encoding %s' % msg.encoding)
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.step,
        )
        image = rows[:, :msg.width * channels].reshape(
            msg.height,
            msg.width,
            channels,
        )
        encoding = msg.encoding.lower()
        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image.copy()

    def _nearest_visible_buoy(self):
        if self.drone_pose is None:
            return None
        nearest = None
        nearest_distance = float('inf')
        for name, pose in self.buoy_poses.items():
            distance = math.hypot(
                pose.position.x - self.drone_pose.position.x,
                pose.position.y - self.drone_pose.position.y,
            )
            if (
                distance <= self.detection_max_distance
                and distance < nearest_distance
            ):
                nearest = (name, pose.position.x, pose.position.y)
                nearest_distance = distance
        return nearest

    def connect_px4(self):
        self.get_logger().info('Waiting for PX4 MAVLink on %s' % self.mavlink_url)
        self.mav = mavutil.mavlink_connection(
            self.mavlink_url,
            autoreconnect=True,
            source_system=254,
            source_component=0,
        )
        heartbeat = self.mav.wait_heartbeat(timeout=self.heartbeat_timeout)
        if heartbeat is None:
            raise RuntimeError('Timed out waiting for PX4 heartbeat')
        self.target_system = self.mav.target_system
        self.target_component = self.mav.target_component

    def pump(self, duration):
        end = time.monotonic() + duration
        while (
            rclpy.ok()
            and not self.stop_event.is_set()
            and time.monotonic() < end
        ):
            now = time.monotonic()
            if now - self.last_gcs_heartbeat >= 1.0:
                self.mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                self.last_gcs_heartbeat = now
            msg = self.mav.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == 'LOCAL_POSITION_NED':
                    self.local_position = msg
                msg = self.mav.recv_match(blocking=False)
            time.sleep(0.02)

    def set_param(self, name, value, param_type):
        self.mav.mav.param_set_send(
            self.target_system,
            self.target_component,
            name.encode('ascii'),
            float(value),
            param_type,
        )

    def configure_px4(self):
        self.set_param(
            'NAV_DLL_ACT',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        self.set_param(
            'CBRK_SUPPLY_CHK',
            894281,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        self.pump(1.0)

    def send_setpoint(self, x, y, z_down, yaw=0.0):
        mask = (
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
            mask,
            float(x),
            float(y),
            float(z_down),
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            float(yaw),
            0.0,
        )

    def move_setpoint_toward(self, goal, speed, dt):
        if self.drone_sp is None:
            self.drone_sp = [
                self.local_position.x,
                self.local_position.y,
                self.local_position.z,
            ]
        delta = [goal[index] - self.drone_sp[index] for index in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        step = max(0.02, speed * dt)
        if distance <= step:
            self.drone_sp = list(goal)
        elif distance > 1e-6:
            scale = step / distance
            self.drone_sp = [
                self.drone_sp[index] + delta[index] * scale
                for index in range(3)
            ]
        return self.drone_sp

    def wait_for_pose_sources(self, timeout=40.0):
        start = time.monotonic()
        while rclpy.ok() and not self.stop_event.is_set():
            self.pump(0.1)
            if self.local_position is not None and self.drone_pose is not None:
                self.local_origin_gz = (
                    self.drone_pose.position.x - self.local_position.y,
                    self.drone_pose.position.y - self.local_position.x,
                    self.drone_pose.position.z + self.local_position.z,
                )
                return
            if time.monotonic() - start > timeout:
                raise RuntimeError('Timed out waiting for PX4/Gazebo UAV pose')

    def gazebo_to_local(self, x_gz, y_gz, z_up):
        if self.local_position is not None and self.drone_pose is not None:
            return (
                self.local_position.x + y_gz - self.drone_pose.position.y,
                self.local_position.y + x_gz - self.drone_pose.position.x,
                self.local_position.z - z_up + self.drone_pose.position.z,
            )
        origin_x, origin_y, origin_z = self.local_origin_gz
        return y_gz - origin_y, x_gz - origin_x, origin_z - z_up

    def release_from_deck(self):
        release = Boolean()
        release.data = True
        for _ in range(5):
            self.release_pub.publish(release)
            self.pump(0.05)

    def takeoff(self):
        self.wait_for_pose_sources()
        start = [
            self.local_position.x,
            self.local_position.y,
            self.local_position.z,
        ]
        self.drone_sp = list(start)
        for _ in range(80):
            self.send_setpoint(*start)
            self.pump(0.05)

        base_mode, custom_mode, sub_mode = mavutil.px4_map['OFFBOARD']
        self.mav.set_mode(base_mode, custom_mode, sub_mode)
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )

        goal = (start[0], start[1], -abs(self.takeoff_altitude))
        spool_setpoint = (start[0], start[1], start[2] - 0.8)
        self.get_logger().info(
            'Armed on the fixed shoreline helipad; building lift before climb'
        )
        for _ in range(40):
            self.send_setpoint(*spool_setpoint)
            self.pump(0.05)
        self.release_from_deck()
        self.get_logger().info(
            'Taking off to %.1f m for visual patrol' % self.takeoff_altitude
        )
        last = time.monotonic()
        deadline = last + 50.0
        while rclpy.ok() and not self.stop_event.is_set():
            now = time.monotonic()
            setpoint = self.move_setpoint_toward(
                goal,
                self.takeoff_climb_rate,
                now - last,
            )
            last = now
            self.send_setpoint(*setpoint)
            self.pump(0.05)
            if self.local_position.z <= goal[2] + 0.7:
                return
            if now > deadline:
                raise RuntimeError('Timed out during UAV takeoff')

    def patrol_until_detection(self):
        if not self.patrol_waypoints:
            raise RuntimeError('No patrol waypoints configured')
        index = 0
        last = time.monotonic()
        while rclpy.ok() and not self.stop_event.is_set():
            if self.detected_target is not None:
                return
            target_x, target_y = self.patrol_waypoints[index]
            local_goal = self.gazebo_to_local(
                target_x,
                target_y,
                self.takeoff_altitude,
            )
            now = time.monotonic()
            setpoint = self.move_setpoint_toward(
                local_goal,
                self.patrol_speed,
                now - last,
            )
            last = now
            self.send_setpoint(*setpoint)
            self.pump(0.05)

            if self.drone_pose is not None:
                distance = math.hypot(
                    target_x - self.drone_pose.position.x,
                    target_y - self.drone_pose.position.y,
                )
                if distance < 2.0:
                    index = (index + 1) % len(self.patrol_waypoints)
                    self.get_logger().info(
                        'Patrol waypoint reached; next=%d/%d'
                        % (index + 1, len(self.patrol_waypoints))
                    )

    def boat_standoff_goal(self, target_x, target_y):
        if self.boat_pose is None:
            return target_x - self.boat_standoff_distance, target_y
        dx = self.boat_pose.position.x - target_x
        dy = self.boat_pose.position.y - target_y
        distance = max(0.01, math.hypot(dx, dy))
        return (
            target_x + self.boat_standoff_distance * dx / distance,
            target_y + self.boat_standoff_distance * dy / distance,
        )

    def publish_cooperative_target(self):
        name, target_x, target_y = self.detected_target
        stamp = self.get_clock().now().to_msg()

        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = 'map'
        target.pose.position.x = target_x
        target.pose.position.y = target_y
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)

        if self.cooperative_boat_goal is None:
            self.cooperative_boat_goal = self.boat_standoff_goal(
                target_x,
                target_y,
            )
        goal_x, goal_y = self.cooperative_boat_goal
        yaw = math.atan2(target_y - goal_y, target_x - goal_x)
        boat_goal = PoseStamped()
        boat_goal.header = target.header
        boat_goal.pose.position.x = goal_x
        boat_goal.pose.position.y = goal_y
        boat_goal.pose.orientation.z = math.sin(0.5 * yaw)
        boat_goal.pose.orientation.w = math.cos(0.5 * yaw)
        self.goal_pub.publish(boat_goal)
        self.publish_target_marker(target, name, goal_x, goal_y)
        self.get_logger().info(
            'Sent visual target to boat Nav2: buoy=(%.2f, %.2f), '
            'safe_goal=(%.2f, %.2f)'
            % (target_x, target_y, goal_x, goal_y)
        )
        return goal_x, goal_y

    def publish_target_marker(self, target, name, goal_x, goal_y):
        markers = MarkerArray()
        buoy = Marker()
        buoy.header = target.header
        buoy.ns = 'uav_visual_target'
        buoy.id = 0
        buoy.type = Marker.SPHERE
        buoy.action = Marker.ADD
        buoy.pose = target.pose
        buoy.pose.position.z = 3.0
        buoy.scale.x = 2.4
        buoy.scale.y = 2.4
        buoy.scale.z = 2.4
        buoy.color.r = 1.0
        buoy.color.g = 0.1
        buoy.color.b = 0.05
        buoy.color.a = 0.95
        buoy.lifetime = Duration(seconds=0.0).to_msg()
        markers.markers.append(buoy)

        label = Marker()
        label.header = target.header
        label.ns = 'uav_visual_target'
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = target.pose.position.x
        label.pose.position.y = target.pose.position.y
        label.pose.position.z = 6.0
        label.pose.orientation.w = 1.0
        label.scale.z = 1.3
        label.color.r = 0.9
        label.color.g = 0.05
        label.color.b = 0.05
        label.color.a = 1.0
        label.text = 'UAV detected: %s' % name
        label.lifetime = Duration(seconds=0.0).to_msg()
        markers.markers.append(label)

        line = Marker()
        line.header = target.header
        line.ns = 'uav_visual_target'
        line.id = 2
        line.type = Marker.ARROW
        line.action = Marker.ADD
        line.points = []
        from geometry_msgs.msg import Point
        start = Point()
        start.x = goal_x
        start.y = goal_y
        start.z = 0.5
        end = Point()
        end.x = target.pose.position.x
        end.y = target.pose.position.y
        end.z = 0.5
        line.points.extend([start, end])
        line.scale.x = 0.35
        line.scale.y = 0.65
        line.scale.z = 0.8
        line.color.r = 1.0
        line.color.g = 0.65
        line.color.b = 0.0
        line.color.a = 1.0
        line.lifetime = Duration(seconds=0.0).to_msg()
        markers.markers.append(line)
        self.marker_pub.publish(markers)

    def pursue_target(self, boat_goal):
        _, target_x, target_y = self.detected_target
        last = time.monotonic()
        while rclpy.ok() and not self.stop_event.is_set():
            local_goal = self.gazebo_to_local(
                target_x,
                target_y,
                self.takeoff_altitude,
            )
            now = time.monotonic()
            setpoint = self.move_setpoint_toward(
                local_goal,
                self.target_speed,
                now - last,
            )
            last = now
            self.send_setpoint(*setpoint)
            self.pump(0.05)

            boat_distance = float('inf')
            if self.boat_pose is not None:
                boat_distance = math.hypot(
                    self.boat_pose.position.x - boat_goal[0],
                    self.boat_pose.position.y - boat_goal[1],
                )
            drone_distance = float('inf')
            if self.drone_pose is not None:
                drone_distance = math.hypot(
                    self.drone_pose.position.x - target_x,
                    self.drone_pose.position.y - target_y,
                )
            if boat_distance < 3.0 and drone_distance < 2.0:
                self.get_logger().info(
                    'UAV and USV reached the visual buoy target'
                )
                break

        while rclpy.ok() and not self.stop_event.is_set():
            local_goal = self.gazebo_to_local(
                target_x,
                target_y,
                self.takeoff_altitude,
            )
            self.drone_sp = list(local_goal)
            self.send_setpoint(*local_goal)
            self.pump(0.05)

    def run(self):
        self.connect_px4()
        self.configure_px4()
        self.takeoff()
        self.get_logger().info('UAV airborne; beginning visual buoy patrol')
        self.patrol_until_detection()
        boat_goal = self.publish_cooperative_target()
        self.pursue_target(boat_goal)


def main(args=None):
    rclpy.init(args=args)
    node = UavBuoyVisualMission()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
