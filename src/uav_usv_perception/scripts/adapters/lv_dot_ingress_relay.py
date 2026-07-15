#!/usr/bin/env python3
"""Relay selected ROS 2 sensor inputs into the isolated ROS 1 backend."""

import json
import socket
import struct
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


_MAGIC = b'LVD1'


class LvDotIngressRelay(Node):
    """Send only the latest sensor messages over a bounded TCP relay."""

    def __init__(self):
        super().__init__('lv_dot_ingress_relay')
        self.declare_parameter(
            'pointcloud_topic', '/perception/usv_01/points_filtered'
        )
        self.declare_parameter(
            'pose_topic', '/perception/lv_dot/usv_01/pose'
        )
        self.declare_parameter(
            'image_topic', '/fleet/uplink/uav_01/camera/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic',
            '/fleet/uplink/uav_01/camera/camera_info',
        )
        self.declare_parameter('listen_address', '0.0.0.0')
        self.declare_parameter('port', 19090)

        self.pending = {}
        self.pending_lock = threading.Lock()
        self.pending_event = threading.Event()
        self.stop_event = threading.Event()
        self.connection = None
        self.connection_lock = threading.Lock()
        self.counts = {
            'pointcloud': 0,
            'pose': 0,
            'image': 0,
            'camera_info': 0,
        }

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('pointcloud_topic').value),
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('pose_topic').value),
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        address = str(self.get_parameter('listen_address').value)
        port = int(self.get_parameter('port').value)
        self.worker = threading.Thread(
            target=self._serve, args=(address, port), daemon=True
        )
        self.worker.start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'LV-DOT ROS2 ingress listening on %s:%d' % (address, port)
        )

    @staticmethod
    def _header(message):
        return {
            'sec': int(message.header.stamp.sec),
            'nanosec': int(message.header.stamp.nanosec),
            'frame_id': message.header.frame_id,
        }

    def _queue(self, kind, metadata, payload=b''):
        metadata['kind'] = kind
        with self.pending_lock:
            self.pending[kind] = (metadata, payload)
        self.pending_event.set()

    def _on_pose(self, message):
        pose = message.pose
        self._queue('pose', {
            'header': self._header(message),
            'position': [
                pose.position.x, pose.position.y, pose.position.z
            ],
            'orientation': [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        })

    def _on_image(self, message):
        self._queue('image', {
            'header': self._header(message),
            'height': int(message.height),
            'width': int(message.width),
            'encoding': message.encoding,
            'is_bigendian': int(message.is_bigendian),
            'step': int(message.step),
        }, bytes(message.data))

    def _on_camera_info(self, message):
        self._queue('camera_info', {
            'header': self._header(message),
            'height': int(message.height),
            'width': int(message.width),
            'distortion_model': message.distortion_model,
            'd': list(message.d),
            'k': list(message.k),
            'r': list(message.r),
            'p': list(message.p),
            'binning_x': int(message.binning_x),
            'binning_y': int(message.binning_y),
        })

    def _on_pointcloud(self, message):
        self._queue('pointcloud', {
            'header': self._header(message),
            'height': int(message.height),
            'width': int(message.width),
            'fields': [
                {
                    'name': field.name,
                    'offset': int(field.offset),
                    'datatype': int(field.datatype),
                    'count': int(field.count),
                }
                for field in message.fields
            ],
            'is_bigendian': bool(message.is_bigendian),
            'point_step': int(message.point_step),
            'row_step': int(message.row_step),
            'is_dense': bool(message.is_dense),
        }, bytes(message.data))

    @staticmethod
    def _packet(metadata, payload):
        encoded = json.dumps(
            metadata, separators=(',', ':'), ensure_ascii=True
        ).encode('ascii')
        return (
            _MAGIC
            + struct.pack('!II', len(encoded), len(payload))
            + encoded
            + payload
        )

    def _disconnect(self):
        with self.connection_lock:
            connection = self.connection
            self.connection = None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass

    def _serve(self, address, port):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((address, port))
        server.listen(1)
        server.settimeout(0.5)
        try:
            while not self.stop_event.is_set():
                with self.connection_lock:
                    connected = self.connection is not None
                if not connected:
                    try:
                        connection, peer = server.accept()
                    except socket.timeout:
                        continue
                    connection.setsockopt(
                        socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
                    )
                    connection.settimeout(2.0)
                    with self.connection_lock:
                        self.connection = connection
                    self.get_logger().info(
                        'LV-DOT ROS1 ingress connected from %s:%d' % peer
                    )
                self.pending_event.wait(0.2)
                self.pending_event.clear()
                with self.pending_lock:
                    pending = list(self.pending.values())
                    self.pending.clear()
                for metadata, payload in pending:
                    try:
                        with self.connection_lock:
                            connection = self.connection
                        if connection is None:
                            break
                        connection.sendall(self._packet(metadata, payload))
                        self.counts[metadata['kind']] += 1
                    except (OSError, socket.timeout):
                        self._disconnect()
                        break
        finally:
            self._disconnect()
            server.close()

    def _report(self):
        with self.connection_lock:
            connected = self.connection is not None
        self.get_logger().info(
            'LV-DOT ingress connected=%s sent cloud=%d pose=%d '
            'image=%d camera_info=%d'
            % (
                connected,
                self.counts['pointcloud'],
                self.counts['pose'],
                self.counts['image'],
                self.counts['camera_info'],
            )
        )

    def destroy_node(self):
        self.stop_event.set()
        self.pending_event.set()
        self._disconnect()
        self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LvDotIngressRelay()
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
