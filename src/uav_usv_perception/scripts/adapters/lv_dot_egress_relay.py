#!/usr/bin/env python3
"""Receive whitelisted LV-DOT ROS 1 outputs over an isolated TCP link."""

import json
import socket
import struct
import threading

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


_MAGIC = b'LVO1'


def _read_exact(connection, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise ConnectionError('LV-DOT egress disconnected')
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


class LvDotEgressRelay(Node):
    """Publish only native dynamic-box and velocity MarkerArray outputs."""

    def __init__(self):
        super().__init__('lv_dot_egress_relay')
        self.declare_parameter('listen_address', '0.0.0.0')
        self.declare_parameter('port', 19091)
        self.declare_parameter(
            'bbox_topic', '/lv_dot/onboard_detector/dynamic_bboxes'
        )
        self.declare_parameter(
            'velocity_topic',
            '/lv_dot/onboard_detector/velocity_visualizaton',
        )
        self.output_publishers = {
            'dynamic_bboxes': self.create_publisher(
                MarkerArray,
                str(self.get_parameter('bbox_topic').value),
                10,
            ),
            'velocity_markers': self.create_publisher(
                MarkerArray,
                str(self.get_parameter('velocity_topic').value),
                10,
            ),
        }
        self.stop_event = threading.Event()
        self.counts = {'dynamic_bboxes': 0, 'velocity_markers': 0}
        address = str(self.get_parameter('listen_address').value)
        port = int(self.get_parameter('port').value)
        self.worker = threading.Thread(
            target=self._serve, args=(address, port), daemon=True
        )
        self.worker.start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'LV-DOT ROS1 egress listening on %s:%d' % (address, port)
        )

    @staticmethod
    def _marker(data):
        marker = Marker()
        header = data['header']
        marker.header.stamp.sec = int(header['sec'])
        marker.header.stamp.nanosec = int(header['nanosec'])
        marker.header.frame_id = header['frame_id']
        marker.ns = data.get('ns', '')
        marker.id = int(data.get('id', 0))
        marker.type = int(data.get('type', Marker.LINE_LIST))
        marker.action = int(data.get('action', Marker.ADD))
        position = data['position']
        orientation = data['orientation']
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.x = float(orientation[0])
        marker.pose.orientation.y = float(orientation[1])
        marker.pose.orientation.z = float(orientation[2])
        marker.pose.orientation.w = float(orientation[3])
        marker.text = data.get('text', '')
        for coordinates in data.get('points', []):
            point = Point()
            point.x = float(coordinates[0])
            point.y = float(coordinates[1])
            point.z = float(coordinates[2])
            marker.points.append(point)
        return marker

    def _serve(self, address, port):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((address, port))
        server.listen(1)
        server.settimeout(0.5)
        try:
            while not self.stop_event.is_set():
                try:
                    connection, peer = server.accept()
                except socket.timeout:
                    continue
                connection.settimeout(2.0)
                self.get_logger().info(
                    'LV-DOT ROS1 egress connected from %s:%d' % peer
                )
                try:
                    while not self.stop_event.is_set():
                        if _read_exact(connection, 4) != _MAGIC:
                            raise ValueError('invalid LV-DOT egress magic')
                        json_size = struct.unpack(
                            '!I', _read_exact(connection, 4)
                        )[0]
                        if json_size > 16 * 1024 * 1024:
                            raise ValueError('invalid LV-DOT egress size')
                        data = json.loads(
                            _read_exact(connection, json_size).decode('ascii')
                        )
                        kind = data.get('kind')
                        publisher = self.output_publishers.get(kind)
                        if publisher is None:
                            continue
                        output = MarkerArray()
                        output.markers = [
                            self._marker(marker)
                            for marker in data.get('markers', [])
                        ]
                        publisher.publish(output)
                        self.counts[kind] += 1
                except (ConnectionError, OSError, ValueError, json.JSONDecodeError):
                    pass
                finally:
                    connection.close()
        finally:
            server.close()

    def _report(self):
        self.get_logger().info(
            'LV-DOT egress received boxes=%d velocities=%d'
            % (
                self.counts['dynamic_bboxes'],
                self.counts['velocity_markers'],
            )
        )

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LvDotEgressRelay()
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
