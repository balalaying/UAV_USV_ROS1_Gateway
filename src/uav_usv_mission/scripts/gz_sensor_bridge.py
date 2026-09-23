#!/usr/bin/env python3
"""Bridge Gazebo Harmonic sensor messages to ROS 1 without ABI mixing."""

import math
import subprocess
import threading
import time

from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.transport13 import Node as GzNode
import numpy as np
import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.node import Node
from uav_usv_ros1_compat.qos import qos_profile_sensor_data
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

        self.declare_parameter('world_name', 'heterogeneous_332')
        self.declare_parameter(
            'uav_ids', ['uav_01', 'uav_02', 'uav_03']
        )
        self.declare_parameter(
            'uav_model_names', [
                'uav_01',
                'uav_02',
                'uav_03',
            ]
        )
        self.declare_parameter(
            'usv_ids', ['usv_01', 'usv_02', 'usv_03']
        )
        self.declare_parameter(
            'usv_source_names', ['own_01', 'own_02', 'own_03']
        )
        self.declare_parameter('bridge_usv_scans', True)
        self.declare_parameter('bridge_usv_depth', True)
        self.declare_parameter('bridge_base_radar', True)
        self.declare_parameter(
            'base_radar_source_topic', '/defense/own_01/scan'
        )
        self.declare_parameter('camera_max_rate', 15.0)
        self.declare_parameter('usv_camera_horizontal_fov', 1.047)
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
        usv_camera_horizontal_fov = max(
            0.1,
            float(self.get_parameter('usv_camera_horizontal_fov').value),
        )
        camera_min_period = (
            1.0 / camera_max_rate if camera_max_rate > 0.0 else 0.0
        )
        available_topics = self._wait_for_sensor_topics(20.0)

        camera_topics = []
        depth_topics = []
        uav_image_candidates = sorted(
            topic for topic in available_topics
            if topic.endswith('/image')
            and any(token in topic.lower() for token in ('x500', 'mono_cam', '/model/uav_'))
        )
        uav_info_candidates = sorted(
            topic for topic in available_topics
            if topic.endswith('/camera_info')
            and any(token in topic.lower() for token in ('x500', 'mono_cam', '/model/uav_'))
        )
        for index, (vehicle_id, model_name) in enumerate(zip(uav_ids, uav_models)):
            expected_image = (
                '/world/%s/model/%s/link/camera_link/'
                'sensor/camera/image' % (world_name, model_name)
            )
            expected_info = (
                '/world/%s/model/%s/link/camera_link/'
                'sensor/camera/camera_info' % (world_name, model_name)
            )
            image_source = self._resolve_topic(
                    available_topics,
                    expected_image,
                    (vehicle_id, 'camera', 'image'),
                    suffix='/image',
                )
            info_source = self._resolve_topic(
                    available_topics,
                    expected_info,
                    (vehicle_id, 'camera', 'camera_info'),
                    suffix='/camera_info',
                )
            if not any(vehicle_id in topic for topic in available_topics):
                if index < len(uav_image_candidates):
                    image_source = uav_image_candidates[index]
                if index < len(uav_info_candidates):
                    info_source = uav_info_candidates[index]
            camera_topics.append((
                image_source,
                '/fleet/uplink/%s/camera/image_raw' % vehicle_id,
                vehicle_id + '/camera_link',
                info_source,
                '/fleet/uplink/%s/camera_info_raw' % vehicle_id,
                False,
            ))
        for vehicle_id, source_name in zip(usv_ids, usv_sources):
            expected_camera = '/defense/%s/front_camera' % source_name
            camera_topics.append((
                self._resolve_topic(
                    available_topics,
                    expected_camera,
                    (source_name, 'front_camera'),
                ),
                '/fleet/uplink/%s/camera' % vehicle_id,
                vehicle_id + '/camera_link',
                '',
                '/fleet/uplink/%s/camera_info_raw' % vehicle_id,
                True,
            ))
            if bool(self.get_parameter('bridge_usv_depth').value):
                depth_topics.append((
                    self._resolve_topic(
                        available_topics,
                        '/defense/%s/depth_camera' % source_name,
                        (source_name, 'depth_camera'),
                    ),
                    '/fleet/uplink/%s/depth/image_raw' % vehicle_id,
                    vehicle_id + '/depth_camera_link',
                    '/fleet/uplink/%s/depth/camera_info' % vehicle_id,
                ))

        self.sensor_publishers = {}
        for (
            gz_topic,
            ros_topic,
            frame_id,
            gz_info_topic,
            ros_info_topic,
            synthesize_info,
        ) in camera_topics:
            publisher = self.create_publisher(
                Image, ros_topic, qos_profile_sensor_data
            )
            self.sensor_publishers[ros_topic] = publisher
            callback = self._camera_callback(
                publisher,
                frame_id,
                ros_topic,
                camera_min_period,
                (
                    self.create_publisher(
                        CameraInfo, ros_info_topic, qos_profile_sensor_data
                    )
                    if synthesize_info else None
                ),
                usv_camera_horizontal_fov,
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

        for (
            gz_topic,
            ros_topic,
            frame_id,
            ros_info_topic,
        ) in depth_topics:
            publisher = self.create_publisher(
                Image, ros_topic, qos_profile_sensor_data
            )
            info_publisher = self.create_publisher(
                CameraInfo, ros_info_topic, qos_profile_sensor_data
            )
            self.sensor_publishers[ros_topic] = publisher
            self.sensor_publishers[ros_info_topic] = info_publisher
            callback = self._camera_callback(
                publisher,
                frame_id,
                ros_topic,
                camera_min_period,
                info_publisher,
                usv_camera_horizontal_fov,
            )
            if not self.gz_node.subscribe(GzImage, gz_topic, callback):
                raise RuntimeError('Unable to subscribe to %s' % gz_topic)

        if bool(self.get_parameter('bridge_usv_scans').value):
            for vehicle_id, source_name in zip(usv_ids, usv_sources):
                gz_topic = '/defense/%s/scan' % source_name
                gz_topic = self._resolve_topic(
                    available_topics,
                    gz_topic,
                    (source_name, 'scan'),
                    suffix='/scan',
                )
                ros_topic = '/fleet/uplink/%s/scan' % vehicle_id
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
            configured_radar_source = str(
                self.get_parameter('base_radar_source_topic').value
            ).strip()
            base_radar_source = configured_radar_source or '/base/radar/scan'
            if not configured_radar_source:
                base_radar_source = self._resolve_topic(
                    available_topics,
                    '/base/radar/scan',
                    ('base', 'radar', 'scan'),
                    suffix='/scan',
                )
            if (
                not configured_radar_source
                and base_radar_source == '/base/radar/scan'
            ):
                scan_candidates = [
                    topic for topic in available_topics
                    if topic.endswith('/scan')
                ]
                if scan_candidates and base_radar_source not in available_topics:
                    base_radar_source = scan_candidates[0]
                    self.get_logger().warn(
                        'No dedicated base radar found; using %s as Qt radar'
                        % base_radar_source
                    )
            self.get_logger().info(
                'Qt base radar source: %s -> %s'
                % (base_radar_source, radar_topic)
            )
            if not self.gz_node.subscribe(
                GzLaserScan,
                base_radar_source,
                self._scan_callback(
                    radar_publisher, radar_topic, 'base_radar'
                ),
            ):
                raise RuntimeError('Unable to subscribe to ' + base_radar_source)

        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'Gazebo sensor bridge ready: %d RGB cameras, %d depth cameras, '
            '%d USV scans, radar=%s'
            % (
                len(camera_topics),
                len(depth_topics),
                len(usv_ids) if self.get_parameter('bridge_usv_scans').value else 0,
                self.get_parameter('bridge_base_radar').value,
            )
        )

    def _wait_for_sensor_topics(self, timeout):
        """Wait briefly for Gazebo discovery, keeping explicit paths as fallback."""
        deadline = time.monotonic() + float(timeout)
        topics = []
        while time.monotonic() < deadline:
            try:
                output = subprocess.check_output(
                    ['gz', 'topic', '-l'],
                    stderr=subprocess.DEVNULL,
                    timeout=3.0,
                    universal_newlines=True,
                )
                topics = [line.strip() for line in output.splitlines() if line.strip()]
            except (OSError, subprocess.SubprocessError):
                pass
            sensor_ready = any(
                topic.endswith(('/image', '/front_camera', '/scan'))
                for topic in topics
            )
            if sensor_ready:
                break
            time.sleep(0.5)
        self.get_logger().info(
            'Gazebo discovery returned %d topics' % len(topics)
        )
        return topics

    @staticmethod
    def _resolve_topic(topics, expected, tokens, suffix=''):
        if expected in topics or not topics:
            return expected
        lowered = tuple(str(token).lower() for token in tokens)
        candidates = []
        for topic in topics:
            value = topic.lower()
            if suffix and not value.endswith(suffix.lower()):
                continue
            score = sum(token in value for token in lowered)
            if score:
                candidates.append((score, -len(topic), topic))
        if not candidates:
            return expected
        candidates.sort(reverse=True)
        return candidates[0][2]

    def _camera_callback(
        self,
        publisher,
        frame_id,
        topic,
        min_period,
        synthetic_info_publisher=None,
        horizontal_fov=1.047,
    ):
        last_publish = 0.0

        def callback(source):
            nonlocal last_publish
            if self.shutting_down or not ros1.ok():
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
                13: ('32FC1', 4, False),
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
                if synthetic_info_publisher is not None:
                    synthetic_info_publisher.publish(
                        self._camera_info_from_image(msg, horizontal_fov)
                    )
            except Exception:
                if not self.shutting_down and ros1.ok():
                    raise
            self._count(topic)
        return callback

    @staticmethod
    def _camera_info_from_image(image, horizontal_fov):
        width = max(1, int(image.width))
        height = max(1, int(image.height))
        focal = width / (2.0 * math.tan(0.5 * horizontal_fov))
        center_x = 0.5 * (width - 1)
        center_y = 0.5 * (height - 1)
        info = CameraInfo()
        info.header = image.header
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.D = [0.0] * 5
        info.K = [
            focal, 0.0, center_x,
            0.0, focal, center_y,
            0.0, 0.0, 1.0,
        ]
        info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.P = [
            focal, 0.0, center_x, 0.0,
            0.0, focal, center_y, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def _camera_info_callback(self, publisher, frame_id, topic, min_period):
        distortion_models = {
            0: 'plumb_bob',
            1: 'rational_polynomial',
            2: 'equidistant',
        }

        last_publish = 0.0

        def callback(source):
            nonlocal last_publish
            if self.shutting_down or not ros1.ok():
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
            msg.D = [float(value) for value in source.distortion.k]
            msg.K = self._fixed_array(source.intrinsics.k, 9)
            msg.R = self._fixed_array(source.rectification_matrix, 9)
            msg.P = self._fixed_array(source.projection.p, 12)
            try:
                publisher.publish(msg)
            except Exception:
                if not self.shutting_down and ros1.ok():
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
            if self.shutting_down or not ros1.ok():
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
                if not self.shutting_down and ros1.ok():
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
    ros1.init(name='gz_sensor_bridge')
    node = GzSensorBridge()
    try:
        ros1.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutting_down = True
        node.destroy_node()
        if ros1.ok():
            ros1.shutdown()


if __name__ == '__main__':
    main()
