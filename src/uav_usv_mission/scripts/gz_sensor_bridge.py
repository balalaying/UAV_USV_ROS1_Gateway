#!/usr/bin/env python3
"""Bridge Gazebo Harmonic sensor messages to ROS 2 without ABI mixing."""

import math
import threading
import time

from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.transport13 import Node as GzNode
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan


class GzSensorBridge(Node):
    def __init__(self):
        super().__init__('gz_sensor_bridge')
        self.gz_node = GzNode()
        self.lock = threading.Lock()
        self.received = {}
        self.shutting_down = False

        self.declare_parameter('world_name', 'cooperative_response_sim')
        self.declare_parameter(
            'uav_ids', ['uav_01', 'uav_02', 'uav_03', 'uav_04']
        )
        self.declare_parameter(
            'uav_model_names', [
                'x500_mono_cam_down_0',
                'x500_mono_cam_down_1',
                'x500_mono_cam_down_2',
                'x500_mono_cam_down_3',
            ]
        )
        self.declare_parameter(
            'usv_ids', ['usv_01', 'usv_02', 'usv_03', 'usv_04']
        )
        self.declare_parameter(
            'usv_source_names', ['own_01', 'own_02', 'own_03', 'own_04']
        )
        self.declare_parameter('bridge_usv_scans', False)
        self.declare_parameter('bridge_base_radar', True)
        self.declare_parameter('camera_max_rate', 15.0)
        world_name = str(self.get_parameter('world_name').value)
        uav_ids = [str(value) for value in self.get_parameter('uav_ids').value]
        uav_models = [
            str(value) for value in self.get_parameter('uav_model_names').value
        ]
        usv_ids = [str(value) for value in self.get_parameter('usv_ids').value]
        usv_sources = [
            str(value) for value in self.get_parameter('usv_source_names').value
        ]
        if len(uav_ids) != len(uav_models):
            raise ValueError('uav_ids and uav_model_names must have equal length')
        if len(usv_ids) != len(usv_sources):
            raise ValueError('usv_ids and usv_source_names must have equal length')
        camera_max_rate = max(
            0.0, float(self.get_parameter('camera_max_rate').value)
        )
        camera_min_period = (
            1.0 / camera_max_rate if camera_max_rate > 0.0 else 0.0
        )

        camera_topics = []
        for vehicle_id, model_name in zip(uav_ids, uav_models):
            camera_topics.append((
                '/world/%s/model/%s/link/camera_link/'
                'sensor/camera/image' % (world_name, model_name),
                '/fleet/uplink/%s/camera' % vehicle_id,
                vehicle_id + '/camera_link',
                '/world/%s/model/%s/link/camera_link/'
                'sensor/camera/camera_info' % (world_name, model_name),
                '/fleet/uplink/%s/camera_info_raw' % vehicle_id,
            ))
        for vehicle_id, source_name in zip(usv_ids, usv_sources):
            camera_topics.append((
                '/defense/%s/front_camera' % source_name,
                '/fleet/uplink/%s/camera' % vehicle_id,
                vehicle_id + '_front_camera',
                '',
                '',
            ))

        self.sensor_publishers = {}
        for (
            gz_topic,
            ros_topic,
            frame_id,
            gz_info_topic,
            ros_info_topic,
        ) in camera_topics:
            publisher = self.create_publisher(
                Image, ros_topic, qos_profile_sensor_data
            )
            self.sensor_publishers[ros_topic] = publisher
            callback = self._camera_callback(
                publisher, frame_id, ros_topic, camera_min_period
            )
            if not self.gz_node.subscribe(GzImage, gz_topic, callback):
                raise RuntimeError('Unable to subscribe to %s' % gz_topic)
            if gz_info_topic:
                info_publisher = self.create_publisher(
                    CameraInfo, ros_info_topic, qos_profile_sensor_data
                )
                self.sensor_publishers[ros_info_topic] = info_publisher
                info_callback = self._camera_info_callback(
                    info_publisher, frame_id, ros_info_topic, 1.0
                )
                if not self.gz_node.subscribe(
                    GzCameraInfo, gz_info_topic, info_callback
                ):
                    raise RuntimeError(
                        'Unable to subscribe to %s' % gz_info_topic
                    )

        if bool(self.get_parameter('bridge_usv_scans').value):
            for vehicle_id, source_name in zip(usv_ids, usv_sources):
                gz_topic = '/defense/%s/scan' % source_name
                ros_topic = '/%s/scan_raw' % vehicle_id
                publisher = self.create_publisher(
                    LaserScan, ros_topic, qos_profile_sensor_data
                )
                self.sensor_publishers[ros_topic] = publisher
                callback = self._scan_callback(
                    publisher, ros_topic, vehicle_id + '/front_lidar'
                )
                if not self.gz_node.subscribe(GzLaserScan, gz_topic, callback):
                    raise RuntimeError('Unable to subscribe to ' + gz_topic)

        if bool(self.get_parameter('bridge_base_radar').value):
            radar_topic = '/fleet/base/radar/scan'
            radar_publisher = self.create_publisher(
                LaserScan, radar_topic, qos_profile_sensor_data
            )
            self.sensor_publishers[radar_topic] = radar_publisher
            if not self.gz_node.subscribe(
                GzLaserScan,
                '/base/radar/scan',
                self._scan_callback(
                    radar_publisher, radar_topic, 'base_radar'
                ),
            ):
                raise RuntimeError('Unable to subscribe to /base/radar/scan')

        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'Gazebo sensor bridge ready: %d cameras, %d USV scans, radar=%s'
            % (
                len(camera_topics),
                len(usv_ids) if self.get_parameter('bridge_usv_scans').value else 0,
                self.get_parameter('bridge_base_radar').value,
            )
        )

    def _camera_callback(self, publisher, frame_id, topic, min_period):
        last_publish = 0.0

        def callback(source):
            nonlocal last_publish
            if self.shutting_down or not rclpy.ok():
                return
            now = time.monotonic()
            if min_period > 0.0 and now - last_publish < min_period:
                return
            last_publish = now
            encodings = {
                1: ('mono8', 1, False),
                2: ('mono16', 2, False),
                3: ('rgb8', 3, False),
                4: ('rgba8', 4, False),
                5: ('bgra8', 4, False),
                8: ('bgr8', 3, False),
                9: ('bgr8', 3, True),
                15: ('bayer_rggb8', 1, False),
                16: ('bayer_bggr8', 1, False),
                17: ('bayer_gbrg8', 1, False),
                18: ('bayer_grbg8', 1, False),
            }
            encoding = encodings.get(int(source.pixel_format_type))
            if encoding is None:
                self.get_logger().warn(
                    'Unsupported Gazebo pixel format %d on %s'
                    % (source.pixel_format_type, topic),
                    throttle_duration_sec=5.0,
                )
                return
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.height = int(source.height)
            msg.width = int(source.width)
            msg.encoding = encoding[0]
            msg.is_bigendian = 0
            if encoding[2]:
                image = np.frombuffer(source.data, dtype=np.uint16).reshape(
                    msg.height, -1
                )[:, :msg.width * encoding[1]]
                converted = np.right_shift(image, 8).astype(np.uint8)
                msg.step = msg.width * encoding[1]
                msg.data = converted.tobytes()
            else:
                msg.step = int(source.step) or msg.width * encoding[1]
                msg.data = bytes(source.data)
            try:
                publisher.publish(msg)
            except Exception:
                if not self.shutting_down and rclpy.ok():
                    raise
            self._count(topic)
        return callback

    def _camera_info_callback(self, publisher, frame_id, topic, min_period):
        distortion_models = {
            0: 'plumb_bob',
            1: 'rational_polynomial',
            2: 'equidistant',
        }

        last_publish = 0.0

        def callback(source):
            nonlocal last_publish
            if self.shutting_down or not rclpy.ok():
                return
            now = time.monotonic()
            if now - last_publish < min_period:
                return
            last_publish = now
            msg = CameraInfo()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.width = int(source.width)
            msg.height = int(source.height)
            msg.distortion_model = distortion_models.get(
                int(source.distortion.model), 'plumb_bob'
            )
            msg.d = [float(value) for value in source.distortion.k]
            msg.k = self._fixed_array(source.intrinsics.k, 9)
            msg.r = self._fixed_array(source.rectification_matrix, 9)
            msg.p = self._fixed_array(source.projection.p, 12)
            try:
                publisher.publish(msg)
            except Exception:
                if not self.shutting_down and rclpy.ok():
                    raise
            self._count(topic)

        return callback

    @staticmethod
    def _fixed_array(values, length):
        output = [float(value) for value in values[:length]]
        output.extend([0.0] * (length - len(output)))
        return output

    def _scan_callback(self, publisher, topic, frame_id):
        def callback(source):
            if self.shutting_down or not rclpy.ok():
                return
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.angle_min = float(source.angle_min)
            msg.angle_max = float(source.angle_max)
            msg.angle_increment = float(source.angle_step)
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = float(source.range_min)
            msg.range_max = float(source.range_max)
            msg.ranges = [float(value) for value in source.ranges]
            msg.intensities = [
                float(value) if math.isfinite(value) else 0.0
                for value in source.intensities
            ]
            try:
                publisher.publish(msg)
            except Exception:
                if not self.shutting_down and rclpy.ok():
                    raise
            self._count(topic)
        return callback

    def _count(self, topic):
        with self.lock:
            self.received[topic] = self.received.get(topic, 0) + 1

    def _report(self):
        with self.lock:
            counts = self.received
            self.received = {}
        summary = ', '.join(
            '%s=%.1fHz' % (topic, count / 5.0)
            for topic, count in sorted(counts.items())
        )
        self.get_logger().info('GZ sensor ingress: %s' % (summary or 'waiting'))


def main():
    rclpy.init()
    node = GzSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutting_down = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
