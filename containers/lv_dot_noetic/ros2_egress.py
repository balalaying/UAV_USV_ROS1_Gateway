#!/usr/bin/env python3
"""Send only selected native LV-DOT MarkerArray outputs to ROS 2."""

import json
import os
import socket
import struct
import threading
import time

import rospy
from visualization_msgs.msg import MarkerArray


MAGIC = b'LVO1'


class EgressSender:
    def __init__(self):
        self.host = os.environ.get('LV_DOT_EGRESS_HOST', '127.0.0.1')
        self.port = int(os.environ.get('LV_DOT_EGRESS_PORT', '19091'))
        self.pending = {}
        self.lock = threading.Lock()
        self.event = threading.Event()
        topics = {
            'dynamic_bboxes': '/lv_dot/onboard_detector/dynamic_bboxes',
            'velocity_markers': (
                '/lv_dot/onboard_detector/velocity_visualizaton'
            ),
            'lidar_bboxes': '/onboard_detector/lidar_bboxes',
            'filtered_bboxes': '/onboard_detector/filtered_bboxes',
            'tracked_bboxes': '/onboard_detector/tracked_bboxes',
        }
        self.subscribers = [
            rospy.Subscriber(
                topic,
                MarkerArray,
                lambda message, key=kind: self.queue(key, message),
                queue_size=2,
            )
            for kind, topic in topics.items()
        ]

    @staticmethod
    def marker(marker):
        return {
            'header': {
                'sec': marker.header.stamp.secs,
                'nanosec': marker.header.stamp.nsecs,
                'frame_id': marker.header.frame_id,
            },
            'ns': marker.ns,
            'id': marker.id,
            'type': marker.type,
            'action': marker.action,
            'position': [
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ],
            'orientation': [
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w,
            ],
            'scale': [marker.scale.x, marker.scale.y, marker.scale.z],
            'color': [
                marker.color.r,
                marker.color.g,
                marker.color.b,
                marker.color.a,
            ],
            'lifetime': [marker.lifetime.secs, marker.lifetime.nsecs],
            'text': marker.text,
            'points': [[point.x, point.y, point.z] for point in marker.points],
        }

    def queue(self, kind, message):
        data = {
            'kind': kind,
            'markers': [self.marker(marker) for marker in message.markers],
        }
        with self.lock:
            self.pending[kind] = data
        self.event.set()

    def run(self):
        while not rospy.is_shutdown():
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                connection.connect((self.host, self.port))
                rospy.loginfo(
                    'LV-DOT egress connected to %s:%d', self.host, self.port
                )
                while not rospy.is_shutdown():
                    self.event.wait(0.2)
                    self.event.clear()
                    with self.lock:
                        pending = list(self.pending.values())
                        self.pending.clear()
                    for data in pending:
                        encoded = json.dumps(
                            data, separators=(',', ':'), ensure_ascii=True
                        ).encode('ascii')
                        connection.sendall(
                            MAGIC + struct.pack('!I', len(encoded)) + encoded
                        )
            except OSError as error:
                if not rospy.is_shutdown():
                    rospy.logwarn_throttle(
                        5.0, 'LV-DOT egress unavailable: %s' % error
                    )
                    time.sleep(1.0)
            finally:
                connection.close()


def main():
    rospy.init_node('lv_dot_ros2_egress', anonymous=False)
    EgressSender().run()


if __name__ == '__main__':
    main()
