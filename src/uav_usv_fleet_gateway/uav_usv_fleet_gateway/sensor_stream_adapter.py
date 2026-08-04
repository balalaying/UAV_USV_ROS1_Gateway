#!/usr/bin/env python3
"""Compress ROS sensor data into transport-neutral gateway payloads."""

import base64
from collections import defaultdict
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def decode_image(message):
    """Return a BGR numpy image for common Gazebo encodings."""
    encoding = str(message.encoding).lower()
    channels = {
        'mono8': 1,
        'rgb8': 3,
        'bgr8': 3,
        'rgba8': 4,
        'bgra8': 4,
    }.get(encoding)
    if channels is None:
        raise ValueError('unsupported image encoding %s' % encoding)
    row_values = int(message.step)
    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected = int(message.height) * row_values
    if raw.size < expected:
        raise ValueError('truncated image data')
    rows = raw[:expected].reshape((int(message.height), row_values))
    pixels = rows[:, :int(message.width) * channels]
    image = pixels.reshape((int(message.height), int(message.width), channels))
    if encoding == 'mono8':
        return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
    if encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def encode_camera_payload(message, vehicle_id, stream_id, quality, max_width):
    image = decode_image(message)
    if max_width > 0 and image.shape[1] > max_width:
        ratio = float(max_width) / float(image.shape[1])
        image = cv2.resize(
            image, (max_width, max(1, round(image.shape[0] * ratio))),
            interpolation=cv2.INTER_AREA,
        )
    ok, encoded = cv2.imencode(
        '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return {
        'message_type': 'camera_frame',
        'stream_id': stream_id,
        'vehicle_id': vehicle_id,
        'frame_id': str(message.header.frame_id),
        'timestamp': stamp_seconds(message.header.stamp),
        'encoding': 'image/jpeg',
        'width': int(image.shape[1]),
        'height': int(image.shape[0]),
        'data_base64': base64.b64encode(encoded).decode('ascii'),
    }


def cloud_payload(message, vehicle_id, stream_id, maximum):
    values = point_cloud2.read_points_numpy(
        message, field_names=['x', 'y', 'z'], skip_nans=True)
    points = np.asarray(values, dtype=np.float32).reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    if maximum > 0 and len(points) > maximum:
        indices = np.linspace(0, len(points) - 1, maximum, dtype=np.int64)
        points = points[indices]
    return {
        'message_type': 'pointcloud_frame',
        'stream_id': stream_id,
        'vehicle_id': vehicle_id,
        'frame_id': str(message.header.frame_id),
        'timestamp': stamp_seconds(message.header.stamp),
        'point_count': int(len(points)),
        'xyz': points.reshape(-1).round(3).tolist(),
    }


def _pose(marker):
    pose = marker.pose
    return {
        'position': {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
        },
        'orientation': {
            'x': float(pose.orientation.x),
            'y': float(pose.orientation.y),
            'z': float(pose.orientation.z),
            'w': float(pose.orientation.w),
        },
    }


def marker_payload(message, vehicle_id, stream_id):
    markers = []
    frame_id = 'map'
    timestamp = 0.0
    for marker in message.markers:
        if marker.action in (Marker.DELETE, Marker.DELETEALL):
            continue
        frame_id = str(marker.header.frame_id or frame_id)
        timestamp = max(timestamp, stamp_seconds(marker.header.stamp))
        markers.append({
            'id': int(marker.id),
            'namespace': str(marker.ns),
            'type': int(marker.type),
            'pose': _pose(marker),
            'scale': {
                'x': float(marker.scale.x),
                'y': float(marker.scale.y),
                'z': float(marker.scale.z),
            },
            'points': [
                [round(float(point.x), 3), round(float(point.y), 3),
                 round(float(point.z), 3)]
                for point in marker.points
            ],
            'text': str(marker.text),
        })
    return {
        'message_type': 'fusion_debug',
        'stream_id': stream_id,
        'vehicle_id': vehicle_id,
        'frame_id': frame_id,
        'timestamp': timestamp,
        'box_count': len(markers),
        'boxes': markers,
    }


def parse_stream_spec(spec):
    parts = str(spec).split('|', 2)
    if len(parts) != 3 or not all(parts):
        raise ValueError(
            'stream spec must be vehicle_id|stream_id|topic: %s' % spec)
    return tuple(parts)


class SensorStreamAdapter(Node):
    def __init__(self):
        super().__init__('fleet_sensor_stream_adapter')
        self.declare_parameter('output_topic', '/fleet/gateway/sensor_stream')
        self.declare_parameter('camera_rate_hz', 8.0)
        self.declare_parameter('pointcloud_rate_hz', 8.0)
        self.declare_parameter('bbox_rate_hz', 10.0)
        self.declare_parameter('jpeg_quality', 72)
        self.declare_parameter('camera_max_width', 720)
        self.declare_parameter('pointcloud_max_points', 12000)
        self.declare_parameter('camera_streams', [
            'usv_01|usv_01_front|/perception/usv_01/camera/detections/image',
            'usv_02|usv_02_front|/perception/usv_02/camera/detections/image',
            'usv_03|usv_03_front|/perception/usv_03/camera/detections/image',
            'uav_01|uav_01_down|/fleet/uplink/uav_01/camera/image_raw',
            'uav_02|uav_02_down|/fleet/uplink/uav_02/camera/image_raw',
            'uav_03|uav_03_down|/fleet/uplink/uav_03/camera/image_raw',
        ])
        self.declare_parameter('pointcloud_streams', [
            'usv_01|usv_01_mid360|/perception/visualization/usv_01/topdown_points',
            'usv_02|usv_02_mid360|/perception/visualization/usv_02/topdown_points',
            'usv_03|usv_03_mid360|/perception/visualization/usv_03/topdown_points',
        ])
        self.declare_parameter('bbox_streams', [
            'usv_01|usv_01_fusion|/perception/usv_01/camera_lidar/fused_bboxes',
            'usv_02|usv_02_fusion|/perception/usv_02/camera_lidar/fused_bboxes',
            'usv_03|usv_03_fusion|/perception/usv_03/camera_lidar/fused_bboxes',
        ])
        self.publisher = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), 10)
        self.last_publish = defaultdict(float)
        self.rates = {
            'camera': max(0.1, float(
                self.get_parameter('camera_rate_hz').value)),
            'pointcloud': max(0.1, float(
                self.get_parameter('pointcloud_rate_hz').value)),
            'bbox': max(0.1, float(
                self.get_parameter('bbox_rate_hz').value)),
        }
        self.jpeg_quality = min(95, max(
            25, int(self.get_parameter('jpeg_quality').value)))
        self.camera_max_width = max(
            0, int(self.get_parameter('camera_max_width').value))
        self.pointcloud_max_points = max(
            100, int(self.get_parameter('pointcloud_max_points').value))
        self._stream_subscriptions = []
        self._create_streams()

    def _ready(self, kind, stream_id):
        key = '%s:%s' % (kind, stream_id)
        now = time.monotonic()
        if now - self.last_publish[key] < 1.0 / self.rates[kind]:
            return False
        self.last_publish[key] = now
        return True

    def _publish(self, payload):
        self.publisher.publish(String(
            data=json.dumps(payload, separators=(',', ':'))))

    def _create_streams(self):
        for spec in self.get_parameter('camera_streams').value:
            vehicle_id, stream_id, topic = parse_stream_spec(spec)
            self._stream_subscriptions.append(self.create_subscription(
                Image, topic,
                lambda msg, v=vehicle_id, s=stream_id:
                self._camera(msg, v, s),
                qos_profile_sensor_data))
        for spec in self.get_parameter('pointcloud_streams').value:
            vehicle_id, stream_id, topic = parse_stream_spec(spec)
            self._stream_subscriptions.append(self.create_subscription(
                PointCloud2, topic,
                lambda msg, v=vehicle_id, s=stream_id:
                self._cloud(msg, v, s),
                qos_profile_sensor_data))
        for spec in self.get_parameter('bbox_streams').value:
            vehicle_id, stream_id, topic = parse_stream_spec(spec)
            self._stream_subscriptions.append(self.create_subscription(
                MarkerArray, topic,
                lambda msg, v=vehicle_id, s=stream_id:
                self._markers(msg, v, s),
                qos_profile_sensor_data))
        self.get_logger().info(
            'Sensor web streams: %d cameras, %d clouds, %d bbox topics'
            % (
                len(self.get_parameter('camera_streams').value),
                len(self.get_parameter('pointcloud_streams').value),
                len(self.get_parameter('bbox_streams').value),
            ))

    def _camera(self, message, vehicle_id, stream_id):
        if not self._ready('camera', stream_id):
            return
        try:
            self._publish(encode_camera_payload(
                message, vehicle_id, stream_id,
                self.jpeg_quality, self.camera_max_width))
        except (ValueError, cv2.error) as error:
            self.get_logger().warning('Camera stream failed: %s' % error)

    def _cloud(self, message, vehicle_id, stream_id):
        if not self._ready('pointcloud', stream_id):
            return
        try:
            self._publish(cloud_payload(
                message, vehicle_id, stream_id, self.pointcloud_max_points))
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.get_logger().warning('Point cloud stream failed: %s' % error)

    def _markers(self, message, vehicle_id, stream_id):
        if self._ready('bbox', stream_id):
            self._publish(marker_payload(message, vehicle_id, stream_id))


def main(args=None):
    rclpy.init(args=args)
    node = SensorStreamAdapter()
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
