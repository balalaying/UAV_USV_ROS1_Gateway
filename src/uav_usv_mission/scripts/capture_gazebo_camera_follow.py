#!/usr/bin/env python3
"""Keep the Gazebo GUI on the active capture scene without controlling it."""

import math
import threading

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.executors import ExternalShutdownException
from uav_usv_ros1_compat.node import Node

from uav_usv_interfaces.msg import CaptureState


ACTIVE_STATES = {
    CaptureState.STATE_TRACKING,
    CaptureState.STATE_APPROACHING,
    CaptureState.STATE_ENCIRCLING,
    CaptureState.STATE_HOLDING,
}


def _look_at_quaternion(position, target):
    """Return a level Gazebo GUI camera quaternion aimed at *target*.

    Gazebo's GUI camera uses its local +X axis as the view direction.  Using
    a zero roll is deliberate: the sea horizon must remain level while the
    director camera moves between capture phases.
    """
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    dz = target[2] - position[2]
    horizontal = math.hypot(dx, dy)
    if horizontal < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, horizontal)
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    return (
        -math.sin(half_pitch) * math.sin(half_yaw),
        math.sin(half_pitch) * math.cos(half_yaw),
        math.cos(half_pitch) * math.sin(half_yaw),
        math.cos(half_pitch) * math.cos(half_yaw),
    )


class CaptureGazeboCameraFollow(Node):
    """A display-only director camera for the Gazebo capture demonstration."""

    def __init__(self):
        super().__init__('capture_gazebo_camera_follow')
        self.declare_parameter('capture_state_topic', '/capture/state')
        self.declare_parameter(
            'world_pose_topic', '/world/heterogeneous_332/pose/info'
        )
        self.declare_parameter('target_id', 'enemy_ship')
        self.declare_parameter('gui_service', '/gui/move_to/pose')
        self.declare_parameter('update_rate_hz', 1.25)
        self.declare_parameter('minimum_distance', 95.0)
        self.declare_parameter('maximum_distance', 115.0)
        self.declare_parameter('horizontal_offset_x', -0.55)
        self.declare_parameter('horizontal_offset_y', -0.60)
        self.declare_parameter('vertical_offset', 0.62)
        self.declare_parameter('max_follow_vehicles', 4)
        self.declare_parameter('follow_uav_id', 'uav_01')
        self.declare_parameter('uav_follow_distance', 34.0)
        self.declare_parameter('uav_follow_height', 16.0)
        self.declare_parameter('uav_lookahead', 12.0)
        self.declare_parameter(
            'vehicle_ids',
            ['uav_01', 'uav_02', 'uav_03', 'usv_01', 'usv_02', 'usv_03'],
        )

        self.target_id = str(self.get_parameter('target_id').value)
        self.gui_service = str(self.get_parameter('gui_service').value)
        self.minimum_distance = float(self.get_parameter('minimum_distance').value)
        self.maximum_distance = float(self.get_parameter('maximum_distance').value)
        self.horizontal_offset_x = float(self.get_parameter('horizontal_offset_x').value)
        self.horizontal_offset_y = float(self.get_parameter('horizontal_offset_y').value)
        self.vertical_offset = float(self.get_parameter('vertical_offset').value)
        self.max_follow_vehicles = max(1, int(self.get_parameter('max_follow_vehicles').value))
        self.follow_uav_id = str(self.get_parameter('follow_uav_id').value)
        self.uav_follow_distance = max(
            5.0, float(self.get_parameter('uav_follow_distance').value))
        self.uav_follow_height = max(
            2.0, float(self.get_parameter('uav_follow_height').value))
        self.uav_lookahead = max(
            0.0, float(self.get_parameter('uav_lookahead').value))
        self.vehicle_ids = set(
            str(value) for value in self.get_parameter('vehicle_ids').value
        )
        rate = max(0.25, float(self.get_parameter('update_rate_hz').value))

        self.lock = threading.Lock()
        self.capture_state = CaptureState.STATE_SEARCH
        self.target_position = None
        self.vehicles = {}
        self.follow_uav_pose = None
        self.last_camera_position = None
        self.last_service_warning = None
        self.gz_node = GzTransportNode()
        self.create_subscription(
            CaptureState, str(self.get_parameter('capture_state_topic').value),
            self._on_capture_state, 10)
        world_pose_topic = str(self.get_parameter('world_pose_topic').value)
        if not self.gz_node.subscribe(Pose_V, world_pose_topic, self._on_gazebo_pose):
            raise RuntimeError('failed to subscribe Gazebo topic ' + world_pose_topic)
        self.create_timer(1.0 / rate, self._update_camera)
        self.get_logger().info(
            'Gazebo capture camera director ready; it only moves the GUI view.')

    def _on_capture_state(self, msg):
        with self.lock:
            self.capture_state = msg.state

    def _on_gazebo_pose(self, msg):
        poses = {item.name: item for item in msg.pose}
        target = poses.get(self.target_id)
        if target is None:
            return
        with self.lock:
            self.target_position = (
                float(target.position.x), float(target.position.y),
                float(target.position.z))
            self.vehicles = {
                vehicle_id: (
                    float(poses[vehicle_id].position.x),
                    float(poses[vehicle_id].position.y),
                    float(poses[vehicle_id].position.z))
                for vehicle_id in self.vehicle_ids if vehicle_id in poses
            }
            follow_uav = poses.get(self.follow_uav_id)
            if follow_uav is not None:
                orientation = follow_uav.orientation
                yaw = math.atan2(
                    2.0 * (orientation.w * orientation.z
                           + orientation.x * orientation.y),
                    1.0 - 2.0 * (orientation.y * orientation.y
                           + orientation.z * orientation.z),
                )
                self.follow_uav_pose = (
                    float(follow_uav.position.x),
                    float(follow_uav.position.y),
                    float(follow_uav.position.z), yaw,
                )

    def _update_camera(self):
        with self.lock:
            if self.capture_state not in ACTIVE_STATES:
                return
            target = self.target_position
            vehicles = list(self.vehicles.values())
            follow_uav = self.follow_uav_pose
        if target is None or not vehicles:
            return
        if follow_uav is not None:
            uav_x, uav_y, uav_z, yaw = follow_uav
            forward_x, forward_y = math.cos(yaw), math.sin(yaw)
            # Third-person chase view: the observer stays behind and above
            # UAV-01, while the look-ahead keeps the pursuit direction visible.
            focus = (
                uav_x + forward_x * self.uav_lookahead,
                uav_y + forward_y * self.uav_lookahead,
                uav_z + 1.5,
            )
            position = (
                uav_x - forward_x * self.uav_follow_distance,
                uav_y - forward_y * self.uav_follow_distance,
                uav_z + self.uav_follow_height,
            )
            if (self.last_camera_position is not None
                    and math.dist(position, self.last_camera_position) < 1.0):
                return
            self._move_gui_camera(position, focus)
            return
        vehicles.sort(key=lambda item: math.hypot(item[0] - target[0], item[1] - target[1]))
        vehicles = vehicles[:self.max_follow_vehicles]
        vehicle_center = tuple(
            sum(item[index] for item in vehicles) / len(vehicles)
            for index in range(3))
        # Keep the target prominent while retaining the closest interceptors.
        focus = tuple(0.62 * target[index] + 0.38 * vehicle_center[index]
                      for index in range(3))
        focus = (focus[0], focus[1], max(0.0, focus[2]))
        spread = max(math.hypot(item[0] - focus[0], item[1] - focus[1])
                     for item in vehicles + [target])
        distance = min(self.maximum_distance,
                       max(self.minimum_distance, 1.35 * spread + 40.0))
        position = (
            focus[0] + self.horizontal_offset_x * distance,
            focus[1] + self.horizontal_offset_y * distance,
            focus[2] + self.vertical_offset * distance)
        if self.last_camera_position is not None and math.dist(position, self.last_camera_position) < 3.0:
            return
        self._move_gui_camera(position, focus)

    def _move_gui_camera(self, position, focus):
        quaternion = _look_at_quaternion(position, focus)
        request = GUICamera()
        request.pose.position.x, request.pose.position.y, request.pose.position.z = position
        (request.pose.orientation.x, request.pose.orientation.y,
         request.pose.orientation.z, request.pose.orientation.w) = quaternion
        try:
            success, response = self.gz_node.request(
                self.gui_service, request, GUICamera, Boolean, 150)
        except Exception as error:
            self._warn_service_unavailable(str(error))
            return
        if not success or not response.data:
            self._warn_service_unavailable('Gazebo GUI did not accept camera pose')
            return
        self.last_camera_position = position

    def _warn_service_unavailable(self, detail):
        now = self.get_clock().now().nanoseconds
        if self.last_service_warning is None or now - self.last_service_warning > 10_000_000_000:
            self.get_logger().warning(
                'Waiting for Gazebo GUI camera service %s: %s'
                % (self.gui_service, detail))
            self.last_service_warning = now


def main(args=None):
    ros1.init(args=args)
    node = CaptureGazeboCameraFollow()
    try:
        ros1.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ros1.ok():
            ros1.shutdown()


if __name__ == '__main__':
    main()
