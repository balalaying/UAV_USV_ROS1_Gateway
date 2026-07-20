#!/usr/bin/env python3
"""Prepare a bounded map-frame XYZ cloud for the Qt perception display."""

from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def xyz_array_to_pointcloud(header, points):
    """Pack an Nx3 float array without constructing Python point objects."""
    xyz = np.ascontiguousarray(points, dtype='<f4').reshape((-1, 3))
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = int(xyz.shape[0])
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = xyz.tobytes(order='C')
    message.is_dense = bool(np.isfinite(xyz).all())
    return message


def quaternion_rotation_matrix(quaternion):
    """Return a 3x3 rotation matrix from a geometry quaternion."""
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z),
         2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),
         1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),
         2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def project_points(
    points,
    rotation,
    translation,
    min_z,
    max_z,
    voxel_size,
    max_points,
):
    """Transform, height-filter and deterministically thin XYZ points."""
    values = np.asarray(points, dtype=np.float64)
    if values.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    values = values.reshape((-1, 3))
    values = values[np.isfinite(values).all(axis=1)]
    if values.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    transformed = values @ np.asarray(rotation).T
    transformed += np.asarray(translation, dtype=np.float64)
    transformed = transformed[
        (transformed[:, 2] >= float(min_z))
        & (transformed[:, 2] <= float(max_z))
    ]
    if transformed.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    if voxel_size > 0.0:
        cells = np.floor(transformed[:, :2] / float(voxel_size)).astype(
            np.int64
        )
        _, indices = np.unique(cells, axis=0, return_index=True)
        transformed = transformed[np.sort(indices)]

    if max_points > 0 and transformed.shape[0] > max_points:
        indices = np.linspace(
            0, transformed.shape[0] - 1, max_points, dtype=np.int64
        )
        transformed = transformed[indices]
    return transformed.astype(np.float32, copy=False)


class QtPointCloudProjectionNode(Node):
    def __init__(self):
        super().__init__('qt_pointcloud_projection_node')
        self.declare_parameter(
            'input_topic', '/perception/usv_01/points_filtered'
        )
        self.declare_parameter(
            'output_topic',
            '/perception/visualization/usv_01/topdown_points',
        )
        self.declare_parameter(
            'status_topic',
            '/perception/visualization/usv_01/topdown_status',
        )
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('pointcloud_display_rate_hz', 10.0)
        self.declare_parameter('pointcloud_max_points', 10000)
        self.declare_parameter('pointcloud_voxel_size', 0.20)
        self.declare_parameter('pointcloud_min_z', -1.0)
        self.declare_parameter('pointcloud_max_z', 8.0)
        self.declare_parameter('pointcloud_persistence_frames', 4)
        self.declare_parameter('tf_timeout_seconds', 0.05)
        self.declare_parameter('tf_queue_timeout_seconds', 0.30)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.display_rate = max(
            0.2,
            float(
                self.get_parameter('pointcloud_display_rate_hz').value
            ),
        )
        self.max_points = max(
            1, int(self.get_parameter('pointcloud_max_points').value)
        )
        self.voxel_size = max(
            0.0,
            float(self.get_parameter('pointcloud_voxel_size').value),
        )
        self.min_z = float(self.get_parameter('pointcloud_min_z').value)
        self.max_z = float(self.get_parameter('pointcloud_max_z').value)
        if self.max_z < self.min_z:
            raise ValueError('pointcloud_max_z must be >= pointcloud_min_z')
        self.persistence_frames = max(
            1,
            int(
                self.get_parameter('pointcloud_persistence_frames').value
            ),
        )
        self.tf_timeout = max(
            0.0, float(self.get_parameter('tf_timeout_seconds').value)
        )
        self.tf_queue_timeout = max(
            self.tf_timeout,
            float(self.get_parameter('tf_queue_timeout_seconds').value),
        )

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_processed_at = 0.0
        self.arrivals = deque(maxlen=50)
        self.received_frames = 0
        self.published_frames = 0
        self.rate_limited_frames = 0
        self.tf_failure_count = 0
        self.decode_failure_count = 0
        self.pending_cloud = None
        self.pending_since = 0.0
        self.projected_history = deque(maxlen=self.persistence_frames)
        self.last_cloud_stamp = -1.0
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.process_timer = self.create_timer(0.01, self._process_pending)
        self.get_logger().info(
            'Qt point-cloud projection %s -> %s (%s, %.1f Hz, max %d)'
            % (
                self.input_topic,
                self.output_topic,
                self.output_frame,
                self.display_rate,
                self.max_points,
            )
        )

    def _transform(self, source_frame, stamp):
        if not source_frame or source_frame == self.output_frame:
            return np.eye(3), np.zeros(3)
        transform = self.tf_buffer.lookup_transform(
            self.output_frame,
            source_frame,
            Time.from_msg(stamp),
        ).transform
        return (
            quaternion_rotation_matrix(transform.rotation),
            np.array([
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ], dtype=np.float64),
        )

    def _input_rate(self):
        if len(self.arrivals) < 2:
            return 0.0
        elapsed = self.arrivals[-1] - self.arrivals[0]
        return (
            (len(self.arrivals) - 1) / elapsed if elapsed > 1e-6 else 0.0
        )

    def _publish_status(self, cloud, output_count, processing_ms, latency_ms):
        payload = {
            'online': True,
            'input_topic': self.input_topic,
            'output_topic': self.output_topic,
            'frame_id': self.output_frame,
            'source_frame': cloud.header.frame_id,
            'input_rate_hz': self._input_rate(),
            'display_rate_limit_hz': self.display_rate,
            'input_points': int(cloud.width) * int(cloud.height),
            'draw_points': int(output_count),
            'processing_ms': float(processing_ms),
            'latency_ms': latency_ms,
            'received_frames': self.received_frames,
            'published_frames': self.published_frames,
            'rate_limited_frames': self.rate_limited_frames,
            'tf_failure_count': self.tf_failure_count,
            'decode_failure_count': self.decode_failure_count,
            'voxel_size': self.voxel_size,
            'min_z': self.min_z,
            'max_z': self.max_z,
            'max_points': self.max_points,
            'persistence_frames': self.persistence_frames,
        }
        status = String()
        status.data = json.dumps(payload, sort_keys=True)
        self.status_publisher.publish(status)

    def _on_cloud(self, cloud):
        arrived_at = time.monotonic()
        self.received_frames += 1
        self.arrivals.append(arrived_at)
        if arrived_at - self.last_processed_at < 0.8 / self.display_rate:
            self.rate_limited_frames += 1
            return
        self.last_processed_at = arrived_at
        self.pending_cloud = cloud
        self.pending_since = arrived_at

    def _process_pending(self):
        cloud = self.pending_cloud
        if cloud is None:
            return
        started_at = time.perf_counter()
        try:
            rotation, translation = self._transform(
                cloud.header.frame_id, cloud.header.stamp
            )
        except TransformException as error:
            if time.monotonic() - self.pending_since < self.tf_queue_timeout:
                return
            self.pending_cloud = None
            self.tf_failure_count += 1
            if self.tf_failure_count == 1 or self.tf_failure_count % 50 == 0:
                self.get_logger().warning(
                    'Dropping queued cloud without exact timestamped TF: %s'
                    % error
                )
            return
        self.pending_cloud = None
        try:
            points = point_cloud2.read_points_numpy(
                cloud,
                field_names=['x', 'y', 'z'],
                skip_nans=True,
            )
            projected = project_points(
                points,
                rotation,
                translation,
                self.min_z,
                self.max_z,
                self.voxel_size,
                self.max_points,
            )
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.decode_failure_count += 1
            self.get_logger().warning(
                'Point-cloud projection failed: %s' % error
            )
            return

        stamp_seconds = (
            float(cloud.header.stamp.sec)
            + float(cloud.header.stamp.nanosec) * 1e-9
        )
        if stamp_seconds < self.last_cloud_stamp:
            self.projected_history.clear()
        self.last_cloud_stamp = stamp_seconds
        self.projected_history.append(projected)
        displayed = np.concatenate(tuple(self.projected_history), axis=0)
        if displayed.shape[0] > self.max_points:
            indices = np.linspace(
                0, displayed.shape[0] - 1, self.max_points, dtype=np.int64
            )
            displayed = displayed[indices]

        output_header = cloud.header
        output_header.frame_id = self.output_frame
        output = xyz_array_to_pointcloud(output_header, displayed)
        self.publisher.publish(output)
        self.published_frames += 1

        processing_ms = (time.perf_counter() - started_at) * 1000.0
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        latency_ms = None
        if stamp_seconds > 0.0:
            latency_ms = max(0.0, (now_seconds - stamp_seconds) * 1000.0)
        self._publish_status(
            cloud, len(displayed), processing_ms, latency_ms
        )


def main(args=None):
    rclpy.init(args=args)
    node = QtPointCloudProjectionNode()
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
