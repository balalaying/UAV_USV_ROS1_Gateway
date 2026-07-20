#!/usr/bin/env python3
"""Generate a standard camera observation for multisensor interface tests."""

from collections import deque
from copy import deepcopy
import hashlib
import json
import math
import random
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _stable_uuid(track_id):
    value = bytearray(hashlib.sha256(track_id.encode('utf-8')).digest()[:16])
    value[6] = (value[6] & 0x0F) | 0x50
    value[8] = (value[8] & 0x3F) | 0x80
    return list(value)


def _rotation_matrix(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


def transform_point(point, transform):
    rotation = _rotation_matrix(transform.rotation)
    translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ])
    transformed = rotation @ np.array(point, dtype=float) + translation
    return tuple(float(value) for value in transformed)


def project_camera_point(point, camera_info, optical_axis='x'):
    """Project a camera-frame point, supporting Gazebo +X or ROS +Z axes."""
    x, y, z = (float(value) for value in point)
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if fx <= 0.0 or fy <= 0.0:
        return None
    if optical_axis == 'x':
        depth = x
        horizontal = -y
        vertical = -z
    elif optical_axis == 'z':
        depth = z
        horizontal = x
        vertical = y
    else:
        raise ValueError("optical_axis must be 'x' or 'z'")
    if depth <= 1e-6:
        return None
    return (
        cx + fx * horizontal / depth,
        cy + fy * vertical / depth,
        depth,
    )


def make_camera_observation(
    truth,
    stamp,
    vehicle_id,
    position_noise,
    velocity_noise,
    position_variance,
    velocity_variance,
    confidence,
    random_source,
    first_seen,
):
    tracked = deepcopy(truth)
    tracked.track_id = '%s_camera_%s' % (vehicle_id, truth.track_id)
    tracked.uuid.uuid = _stable_uuid(tracked.track_id)
    tracked.first_seen = deepcopy(first_seen)
    tracked.last_update = deepcopy(stamp)
    tracked.source_mask = TrackedObject.SOURCE_CAMERA
    tracked.class_name = 'vessel'
    tracked.class_confidence = min(1.0, max(0.0, float(confidence)))
    tracked.sensor_source = 'camera'
    tracked.mmsi = 0
    for axis in ('x', 'y', 'z'):
        position = getattr(tracked.pose.pose.position, axis)
        setattr(
            tracked.pose.pose.position,
            axis,
            float(position) + random_source.gauss(0.0, position_noise),
        )
        velocity = getattr(tracked.twist.twist.linear, axis)
        setattr(
            tracked.twist.twist.linear,
            axis,
            float(velocity) + random_source.gauss(0.0, velocity_noise),
        )
    tracked.pose.covariance = [0.0] * 36
    tracked.twist.covariance = [0.0] * 36
    for index in (0, 7, 14):
        tracked.pose.covariance[index] = position_variance
        tracked.twist.covariance[index] = velocity_variance
    tracked.confidence = min(1.0, max(0.0, float(confidence)))
    return tracked


class UavVisualObservationNode(Node):
    def __init__(self):
        super().__init__('uav_visual_observation_node')
        self.declare_parameter('vehicle_id', 'uav_01')
        self.declare_parameter(
            'image_topic', '/fleet/uplink/uav_01/camera/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic',
            '/fleet/uplink/uav_01/camera/camera_info',
        )
        self.declare_parameter(
            'ground_truth_topic', '/perception/ground_truth/tracks'
        )
        self.declare_parameter(
            'output_topic', '/perception/uav_01/observations'
        )
        self.declare_parameter(
            'status_topic', '/perception/uav_01/observation_status'
        )
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('target_id', 'target_vessel')
        self.declare_parameter('optical_axis', 'x')
        self.declare_parameter('require_in_fov', True)
        self.declare_parameter('min_range_m', 1.0)
        self.declare_parameter('max_range_m', 180.0)
        self.declare_parameter('pixel_margin', 8.0)
        self.declare_parameter('ground_truth_tolerance_seconds', 0.5)
        self.declare_parameter('tf_wait_seconds', 0.5)
        self.declare_parameter('tf_lookup_timeout_seconds', 0.02)
        self.declare_parameter('position_noise_stddev', 0.35)
        self.declare_parameter('velocity_noise_stddev', 0.10)
        self.declare_parameter('position_variance', 0.25)
        self.declare_parameter('velocity_variance', 0.04)
        self.declare_parameter('confidence', 0.82)
        self.declare_parameter('noise_seed', 1001)

        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.truth_topic = str(
            self.get_parameter('ground_truth_topic').value
        )
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        configured_camera_frame = str(
            self.get_parameter('camera_frame').value
        )
        self.camera_frame = configured_camera_frame or (
            self.vehicle_id + '/camera_link'
        )
        self.target_id = str(self.get_parameter('target_id').value)
        self.optical_axis = str(self.get_parameter('optical_axis').value)
        self.require_in_fov = bool(
            self.get_parameter('require_in_fov').value
        )
        self.min_range = max(
            0.0, float(self.get_parameter('min_range_m').value)
        )
        self.max_range = max(
            self.min_range,
            float(self.get_parameter('max_range_m').value),
        )
        self.pixel_margin = max(
            0.0, float(self.get_parameter('pixel_margin').value)
        )
        self.truth_tolerance = max(
            0.01,
            float(
                self.get_parameter(
                    'ground_truth_tolerance_seconds'
                ).value
            ),
        )
        self.tf_wait = max(
            0.01, float(self.get_parameter('tf_wait_seconds').value)
        )
        self.tf_timeout = max(
            0.0,
            float(self.get_parameter('tf_lookup_timeout_seconds').value),
        )
        self.position_noise = max(
            0.0,
            float(self.get_parameter('position_noise_stddev').value),
        )
        self.velocity_noise = max(
            0.0,
            float(self.get_parameter('velocity_noise_stddev').value),
        )
        self.position_variance = max(
            0.0, float(self.get_parameter('position_variance').value)
        )
        self.velocity_variance = max(
            0.0, float(self.get_parameter('velocity_variance').value)
        )
        self.confidence = min(
            1.0, max(0.0, float(
                self.get_parameter('confidence').value
            ))
        )
        self.random = random.Random(
            int(self.get_parameter('noise_seed').value)
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray, self.output_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.truth_history = deque(maxlen=100)
        self.pending_images = deque(maxlen=200)
        self.first_seen = {}
        self.received_images = 0
        self.published_arrays = 0
        self.published_observations = 0
        self.dropped = {
            'camera_info': 0,
            'ground_truth': 0,
            'tf': 0,
            'range': 0,
            'fov': 0,
        }
        self.last_status = {
            'online': False,
            'tf_available': False,
            'visible': False,
            'pixel': None,
            'range_m': None,
            'frame_id': self.camera_frame,
            'last_update': None,
        }

        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            self.info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackedObjectArray, self.truth_topic, self._on_truth, 10
        )
        self.create_timer(0.02, self._process_pending)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            'UAV visual observation %s + %s + TF -> %s'
            % (self.image_topic, self.truth_topic, self.output_topic)
        )

    def _on_camera_info(self, message):
        self.camera_info = message
        if message.header.frame_id:
            self.camera_frame = message.header.frame_id

    def _on_truth(self, message):
        self.truth_history.append(message)

    def _on_image(self, message):
        self.received_images += 1
        self.pending_images.append((time.monotonic(), message))
        self.last_status.update({
            'online': True,
            'frame_id': message.header.frame_id or self.camera_frame,
            'last_update': {
                'sec': int(message.header.stamp.sec),
                'nanosec': int(message.header.stamp.nanosec),
            },
        })

    def _nearest_truth(self, stamp):
        image_time = _stamp_seconds(stamp)
        selected = None
        selected_delta = self.truth_tolerance
        for message in self.truth_history:
            message_time = _stamp_seconds(message.header.stamp)
            delta = abs(message_time - image_time)
            if delta > selected_delta:
                continue
            tracked = next(
                (
                    item for item in message.objects
                    if item.track_id == self.target_id
                ),
                None,
            )
            if tracked is not None:
                selected = (message, tracked)
                selected_delta = delta
        return selected

    def _lookup_camera_transform(self, source_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                self.camera_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None

    def _publish_empty(self, image, reason):
        output = TrackedObjectArray()
        output.header.stamp = image.header.stamp
        output.header.frame_id = self.output_frame
        self.publisher.publish(output)
        self.published_arrays += 1
        self.dropped[reason] += 1

    def _process_image(self, image):
        if self.camera_info is None:
            return False, 'camera_info'
        selected = self._nearest_truth(image.header.stamp)
        if selected is None:
            return False, 'ground_truth'
        truth_message, truth = selected
        source_frame = truth_message.header.frame_id or self.output_frame
        if source_frame != self.output_frame:
            return False, 'ground_truth'
        transform = self._lookup_camera_transform(
            source_frame, image.header.stamp
        )
        if transform is None:
            return False, 'tf'

        truth_point = truth.pose.pose.position
        camera_point = transform_point(
            (truth_point.x, truth_point.y, truth_point.z), transform
        )
        projected = project_camera_point(
            camera_point, self.camera_info, self.optical_axis
        )
        self.last_status['tf_available'] = True
        if projected is None:
            self.last_status['visible'] = False
            return True, 'range'
        u, v, depth = projected
        distance = math.sqrt(sum(value * value for value in camera_point))
        self.last_status['pixel'] = [u, v]
        self.last_status['range_m'] = distance
        if distance < self.min_range or distance > self.max_range:
            self.last_status['visible'] = False
            return True, 'range'
        in_fov = (
            -self.pixel_margin <= u < self.camera_info.width + self.pixel_margin
            and -self.pixel_margin <= v < self.camera_info.height + self.pixel_margin
        )
        if self.require_in_fov and not in_fov:
            self.last_status['visible'] = False
            return True, 'fov'

        first_seen = self.first_seen.setdefault(
            truth.track_id, deepcopy(image.header.stamp)
        )
        observation = make_camera_observation(
            truth=truth,
            stamp=image.header.stamp,
            vehicle_id=self.vehicle_id,
            position_noise=self.position_noise,
            velocity_noise=self.velocity_noise,
            position_variance=self.position_variance,
            velocity_variance=self.velocity_variance,
            confidence=self.confidence,
            random_source=self.random,
            first_seen=first_seen,
        )
        output = TrackedObjectArray()
        output.header.stamp = image.header.stamp
        output.header.frame_id = self.output_frame
        output.objects.append(observation)
        self.publisher.publish(output)
        self.published_arrays += 1
        self.published_observations += 1
        self.last_status.update({
            'online': True,
            'visible': True,
            'frame_id': self.camera_frame,
            'last_update': {
                'sec': int(image.header.stamp.sec),
                'nanosec': int(image.header.stamp.nanosec),
            },
        })
        return True, None

    def _process_pending(self):
        while self.pending_images:
            received_at, image = self.pending_images[0]
            completed, reason = self._process_image(image)
            if not completed and time.monotonic() - received_at < self.tf_wait:
                return
            self.pending_images.popleft()
            if reason is not None:
                self._publish_empty(image, reason)

    def _publish_status(self):
        payload = {
            'mode': 'ground_truth_proxy',
            'vehicle_id': self.vehicle_id,
            'sensor_source': 'CAMERA',
            'image_topic': self.image_topic,
            'camera_info_topic': self.info_topic,
            'ground_truth_topic': self.truth_topic,
            'output_topic': self.output_topic,
            'received_images': self.received_images,
            'published_arrays': self.published_arrays,
            'published_observations': self.published_observations,
            'dropped': dict(self.dropped),
        }
        payload.update(self.last_status)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=True, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = UavVisualObservationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
