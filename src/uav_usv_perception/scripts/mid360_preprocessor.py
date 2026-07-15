#!/usr/bin/env python3
"""Prepare fleet Mid-360 PointCloud2 data for downstream perception."""

from collections import deque
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from uav_usv_interfaces.msg import SensorStatus


class Mid360Preprocessor(Node):
    def __init__(self):
        super().__init__('mid360_preprocessor')
        self.declare_parameter(
            'input_topic', '/fleet/uplink/usv_01/mid360/points'
        )
        self.declare_parameter(
            'output_topic', '/perception/usv_01/points_filtered'
        )
        self.declare_parameter(
            'preview_topic', '/perception/usv_01/mid360/preview'
        )
        self.declare_parameter('status_topic', '/fleet/sensor_status')
        self.declare_parameter('vehicle_id', 'usv_01')
        self.declare_parameter('sensor_id', 'mid360')
        self.declare_parameter('frame_id', 'usv_01/mid360_link')
        self.declare_parameter('tf_target_frame', 'map')
        self.declare_parameter('expected_rate_hz', 10.0)
        self.declare_parameter('timeout_seconds', 1.0)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 70.0)
        self.declare_parameter('min_z', -1000.0)
        self.declare_parameter('max_z', 1000.0)
        self.declare_parameter('voxel_size', 0.12)
        self.declare_parameter('crop_self', True)
        self.declare_parameter('self_min_x', -4.3)
        self.declare_parameter('self_max_x', 2.5)
        self.declare_parameter('self_min_y', -1.8)
        self.declare_parameter('self_max_y', 1.8)
        self.declare_parameter('self_min_z', -2.4)
        self.declare_parameter('self_max_z', 0.35)
        self.declare_parameter('preview_enabled', True)
        self.declare_parameter('preview_rate_hz', 2.0)
        self.declare_parameter('preview_max_points', 5000)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.preview_topic = str(self.get_parameter('preview_topic').value)
        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.sensor_id = str(self.get_parameter('sensor_id').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.tf_target_frame = str(
            self.get_parameter('tf_target_frame').value
        )
        self.expected_rate = max(
            0.1, float(self.get_parameter('expected_rate_hz').value)
        )
        self.timeout = max(
            0.1, float(self.get_parameter('timeout_seconds').value)
        )
        self.min_range = max(
            0.0, float(self.get_parameter('min_range').value)
        )
        self.max_range = max(
            self.min_range,
            float(self.get_parameter('max_range').value),
        )
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        if self.max_z < self.min_z:
            raise ValueError('max_z must be greater than or equal to min_z')
        self.voxel_size = max(
            0.0, float(self.get_parameter('voxel_size').value)
        )
        self.crop_self = bool(self.get_parameter('crop_self').value)
        self.self_bounds = (
            float(self.get_parameter('self_min_x').value),
            float(self.get_parameter('self_max_x').value),
            float(self.get_parameter('self_min_y').value),
            float(self.get_parameter('self_max_y').value),
            float(self.get_parameter('self_min_z').value),
            float(self.get_parameter('self_max_z').value),
        )
        self.preview_enabled = bool(
            self.get_parameter('preview_enabled').value
        )
        preview_rate = max(
            0.1, float(self.get_parameter('preview_rate_hz').value)
        )
        self.preview_period = 1.0 / preview_rate
        self.preview_max_points = max(
            1, int(self.get_parameter('preview_max_points').value)
        )

        self.filtered_pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.preview_pub = self.create_publisher(
            PointCloud2, self.preview_topic, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(
            SensorStatus,
            str(self.get_parameter('status_topic').value),
            10,
        )
        service_name = '/perception/%s/mid360/set_visualization' % (
            self.vehicle_id
        )
        self.create_service(SetBool, service_name, self._set_preview)
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_status)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arrival_times = deque(maxlen=100)
        self.last_arrival = 0.0
        self.last_message_time = self.get_clock().now().to_msg()
        self.last_latency = 0.0
        self.last_processing_ms = 0.0
        self.last_input_points = 0
        self.last_output_points = 0
        self.total_messages = 0
        self.total_bytes = 0
        self.dropped_messages = 0
        self.last_preview_time = 0.0
        self.warned_layout = False
        self.last_cloud_frame = self.frame_id
        self.tf_available = False

        self.get_logger().info(
            'Mid-360 preprocessing: %s -> %s, preview=%s'
            % (self.input_topic, self.output_topic, self.preview_topic)
        )

    def _field_offsets(self, msg):
        fields = {field.name: field for field in msg.fields}
        required = []
        for name in ('x', 'y', 'z'):
            field = fields.get(name)
            if (
                field is None
                or field.datatype != PointField.FLOAT32
                or field.count != 1
            ):
                raise ValueError(
                    'PointCloud2 requires scalar FLOAT32 %s field' % name
                )
            required.append(field.offset)
        return required

    def _coordinates(self, msg, offsets, point_count):
        endian = '>' if msg.is_bigendian else '<'
        coordinates = []
        for offset in offsets:
            values = np.ndarray(
                shape=(point_count,),
                dtype=endian + 'f4',
                buffer=msg.data,
                offset=offset,
                strides=(msg.point_step,),
            )
            coordinates.append(values)
        return np.column_stack(coordinates)

    def _filter_indices(self, xyz):
        mask = np.isfinite(xyz).all(axis=1)
        squared_range = np.einsum('ij,ij->i', xyz, xyz)
        mask &= squared_range >= self.min_range * self.min_range
        mask &= squared_range <= self.max_range * self.max_range
        mask &= xyz[:, 2] >= self.min_z
        mask &= xyz[:, 2] <= self.max_z

        if self.crop_self:
            min_x, max_x, min_y, max_y, min_z, max_z = self.self_bounds
            inside_self = (
                (xyz[:, 0] >= min_x)
                & (xyz[:, 0] <= max_x)
                & (xyz[:, 1] >= min_y)
                & (xyz[:, 1] <= max_y)
                & (xyz[:, 2] >= min_z)
                & (xyz[:, 2] <= max_z)
            )
            mask &= ~inside_self

        indices = np.flatnonzero(mask)
        if self.voxel_size <= 0.0 or indices.size <= 1:
            return indices
        voxel_keys = np.floor(
            xyz[indices] / self.voxel_size
        ).astype(np.int32)
        _, unique_positions = np.unique(
            voxel_keys, axis=0, return_index=True
        )
        return indices[np.sort(unique_positions)]

    @staticmethod
    def _cloud_from_indices(msg, records, indices):
        output = PointCloud2()
        output.header = msg.header
        output.height = 1
        output.width = int(indices.size)
        output.fields = msg.fields
        output.is_bigendian = msg.is_bigendian
        output.point_step = msg.point_step
        output.row_step = output.point_step * output.width
        output.data = records[indices].copy().tobytes()
        output.is_dense = True
        return output

    def _on_cloud(self, msg):
        started = time.perf_counter()
        now_monotonic = time.monotonic()
        point_count = int(msg.width * msg.height)
        if point_count <= 0 or msg.point_step <= 0:
            return
        try:
            offsets = self._field_offsets(msg)
            records = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                point_count, msg.point_step
            )
            xyz = self._coordinates(msg, offsets, point_count)
            indices = self._filter_indices(xyz)
            output = self._cloud_from_indices(msg, records, indices)
        except (ValueError, TypeError) as exc:
            if not self.warned_layout:
                self.get_logger().error('Mid-360 cloud rejected: %s' % exc)
                self.warned_layout = True
            return

        self.filtered_pub.publish(output)
        if (
            self.preview_enabled
            and now_monotonic - self.last_preview_time >= self.preview_period
        ):
            preview_indices = indices
            if preview_indices.size > self.preview_max_points:
                sample = np.linspace(
                    0,
                    preview_indices.size - 1,
                    self.preview_max_points,
                    dtype=np.int64,
                )
                preview_indices = preview_indices[sample]
            self.preview_pub.publish(
                self._cloud_from_indices(msg, records, preview_indices)
            )
            self.last_preview_time = now_monotonic

        if self.last_arrival > 0.0:
            interval = now_monotonic - self.last_arrival
            expected_period = 1.0 / self.expected_rate
            if interval > 1.5 * expected_period:
                self.dropped_messages += max(
                    0, int(round(interval / expected_period)) - 1
                )
        self.last_arrival = now_monotonic
        self.arrival_times.append(now_monotonic)
        self.last_message_time = msg.header.stamp
        self.last_cloud_frame = msg.header.frame_id or self.frame_id
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000
            + int(msg.header.stamp.nanosec)
        )
        self.last_latency = max(0.0, (now_ns - stamp_ns) / 1e9)
        self.last_processing_ms = (time.perf_counter() - started) * 1000.0
        self.last_input_points = point_count
        self.last_output_points = output.width
        self.total_messages += 1
        self.total_bytes += len(msg.data)

    def _measured_rate(self):
        if len(self.arrival_times) < 2:
            return 0.0
        duration = self.arrival_times[-1] - self.arrival_times[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.arrival_times) - 1) / duration

    def _publish_status(self):
        now_monotonic = time.monotonic()
        age = (
            now_monotonic - self.last_arrival
            if self.last_arrival > 0.0
            else math.inf
        )
        rate = self._measured_rate()
        timed_out = age >= self.timeout
        try:
            self.tf_available = bool(self.tf_buffer.can_transform(
                self.tf_target_frame,
                self.last_cloud_frame,
                Time(),
            ))
        except Exception:
            self.tf_available = False
        status = SensorStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.vehicle_id = self.vehicle_id
        status.sensor_id = self.sensor_id
        status.uplink_topic = self.input_topic
        status.message_type = 'sensor_msgs/msg/PointCloud2'
        status.frame_id = self.last_cloud_frame
        status.last_message_time = self.last_message_time
        status.measured_rate_hz = float(rate)
        status.age_seconds = float(min(age, 9999.0))
        status.latency_seconds = float(min(self.last_latency, 9999.0))
        status.processing_time_ms = float(self.last_processing_ms)
        status.point_count = int(self.last_output_points)
        status.total_messages = int(self.total_messages)
        status.total_bytes = int(self.total_bytes)
        status.dropped_messages = int(self.dropped_messages)
        status.timed_out = timed_out
        status.tf_target_frame = self.tf_target_frame
        status.tf_available = self.tf_available
        status.healthy = (
            not timed_out
            and rate >= max(0.2, 0.5 * self.expected_rate)
            and self.tf_available
        )
        self.status_pub.publish(status)

    def _set_preview(self, request, response):
        self.preview_enabled = bool(request.data)
        response.success = True
        response.message = 'Mid-360 preview %s' % (
            'enabled' if self.preview_enabled else 'disabled'
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Mid360Preprocessor()
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
