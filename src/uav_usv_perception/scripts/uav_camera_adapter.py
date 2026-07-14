#!/usr/bin/env python3
"""Expose fleet UAV cameras through one stable ROS 2 interface."""

from collections import deque
from copy import deepcopy
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from uav_usv_interfaces.msg import SensorStatus


class CameraState:
    def __init__(self, vehicle_id, image_topic, info_topic):
        self.vehicle_id = vehicle_id
        self.image_topic = image_topic
        self.info_topic = info_topic
        self.camera_info = None
        self.arrivals = deque(maxlen=100)
        self.last_arrival = 0.0
        self.last_stamp = None
        self.last_frame = vehicle_id + '/camera_link'
        self.last_latency = 0.0
        self.last_processing_ms = 0.0
        self.pixel_count = 0
        self.total_messages = 0
        self.total_bytes = 0
        self.dropped_messages = 0


class UavCameraAdapter(Node):
    def __init__(self):
        super().__init__('uav_camera_adapter')
        self.declare_parameter(
            'vehicle_ids', ['uav_01', 'uav_02', 'uav_03', 'uav_04']
        )
        self.declare_parameter('expected_rate_hz', 15.0)
        self.declare_parameter('timeout_seconds', 1.0)
        self.declare_parameter('tf_target_frame', 'map')
        self.declare_parameter('status_topic', '/fleet/sensor_status')

        self.expected_rate = max(
            0.1, float(self.get_parameter('expected_rate_hz').value)
        )
        self.timeout = max(
            0.1, float(self.get_parameter('timeout_seconds').value)
        )
        self.tf_target_frame = str(
            self.get_parameter('tf_target_frame').value
        )
        vehicle_ids = [
            str(value) for value in self.get_parameter('vehicle_ids').value
        ]

        self.status_pub = self.create_publisher(
            SensorStatus,
            str(self.get_parameter('status_topic').value),
            20,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.states = {}
        self.image_publishers = {}
        self.info_publishers = {}

        for vehicle_id in vehicle_ids:
            raw_image = '/fleet/uplink/%s/camera' % vehicle_id
            raw_info = '/fleet/uplink/%s/camera_info_raw' % vehicle_id
            image_out = '/fleet/uplink/%s/camera/image_raw' % vehicle_id
            info_out = '/fleet/uplink/%s/camera/camera_info' % vehicle_id
            state = CameraState(vehicle_id, image_out, info_out)
            self.states[vehicle_id] = state
            self.image_publishers[vehicle_id] = self.create_publisher(
                Image, image_out, qos_profile_sensor_data
            )
            self.info_publishers[vehicle_id] = self.create_publisher(
                CameraInfo, info_out, qos_profile_sensor_data
            )
            self.create_subscription(
                CameraInfo,
                raw_info,
                lambda msg, key=vehicle_id: self._on_info(key, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                raw_image,
                lambda msg, key=vehicle_id: self._on_image(key, msg),
                qos_profile_sensor_data,
            )

        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            'UAV camera adapter ready for: %s' % ', '.join(vehicle_ids)
        )

    def _on_info(self, vehicle_id, msg):
        self.states[vehicle_id].camera_info = msg

    def _on_image(self, vehicle_id, msg):
        started = time.perf_counter()
        now = time.monotonic()
        state = self.states[vehicle_id]
        expected_period = 1.0 / self.expected_rate
        if state.last_arrival > 0.0:
            interval = now - state.last_arrival
            if interval > 1.5 * expected_period:
                state.dropped_messages += max(
                    0, int(round(interval / expected_period)) - 1
                )
        state.last_arrival = now
        state.arrivals.append(now)

        # Publishing the received message directly preserves data, encoding,
        # timestamp, and frame_id without a decode/re-encode cycle.
        self.image_publishers[vehicle_id].publish(msg)
        if state.camera_info is not None:
            info = deepcopy(state.camera_info)
            info.header = msg.header
            self.info_publishers[vehicle_id].publish(info)

        state.last_stamp = msg.header.stamp
        state.last_frame = msg.header.frame_id or state.last_frame
        stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000
            + int(msg.header.stamp.nanosec)
        )
        state.last_latency = max(
            0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        )
        state.last_processing_ms = (
            time.perf_counter() - started
        ) * 1000.0
        state.pixel_count = int(msg.width * msg.height)
        state.total_messages += 1
        state.total_bytes += len(msg.data)

    @staticmethod
    def _measured_rate(state):
        if len(state.arrivals) < 2:
            return 0.0
        duration = state.arrivals[-1] - state.arrivals[0]
        if duration <= 0.0:
            return 0.0
        return (len(state.arrivals) - 1) / duration

    def _publish_status(self):
        now = time.monotonic()
        for state in self.states.values():
            age = (
                now - state.last_arrival
                if state.last_arrival > 0.0
                else math.inf
            )
            rate = self._measured_rate(state)
            timed_out = age >= self.timeout
            try:
                tf_available = bool(self.tf_buffer.can_transform(
                    self.tf_target_frame,
                    state.last_frame,
                    Time(),
                ))
            except Exception:
                tf_available = False
            status = SensorStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.vehicle_id = state.vehicle_id
            status.sensor_id = 'uav_camera'
            status.uplink_topic = state.image_topic
            status.message_type = 'sensor_msgs/msg/Image'
            status.frame_id = state.last_frame
            if state.last_stamp is not None:
                status.last_message_time = state.last_stamp
            status.measured_rate_hz = float(rate)
            status.age_seconds = float(min(age, 9999.0))
            status.latency_seconds = float(min(state.last_latency, 9999.0))
            status.processing_time_ms = float(state.last_processing_ms)
            status.point_count = int(state.pixel_count)
            status.total_messages = int(state.total_messages)
            status.total_bytes = int(state.total_bytes)
            status.dropped_messages = int(state.dropped_messages)
            status.timed_out = timed_out
            status.tf_target_frame = self.tf_target_frame
            status.tf_available = tf_available
            status.healthy = (
                not timed_out
                and state.camera_info is not None
                and rate >= max(0.2, 0.4 * self.expected_rate)
                and tf_available
            )
            self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = UavCameraAdapter()
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
