#!/usr/bin/env python3
"""Receive the bounded LV-DOT TCP ingress and republish ROS 1 messages."""

import json
import os
import socket
import struct
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


MAGIC = b'LVD1'


def read_exact(connection, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise ConnectionError('LV-DOT ingress disconnected')
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


def apply_header(message, metadata):
    header = metadata['header']
    message.header.stamp = rospy.Time(
        int(header['sec']), int(header['nanosec'])
    )
    message.header.frame_id = header['frame_id']


def main():
    rospy.init_node('lv_dot_ros2_ingress', anonymous=False)
    host = os.environ.get('LV_DOT_INGRESS_HOST', '127.0.0.1')
    port = int(os.environ.get('LV_DOT_INGRESS_PORT', '19090'))
    publishers = {
        'pose': rospy.Publisher(
            '/lv_dot/input/usv_01/pose', PoseStamped, queue_size=10
        ),
        'pointcloud': rospy.Publisher(
            '/lv_dot/input/usv_01/points_filtered',
            PointCloud2,
            queue_size=2,
        ),
        'image': rospy.Publisher(
            '/lv_dot/input/uav_01/camera/image_raw', Image, queue_size=2
        ),
        'camera_info': rospy.Publisher(
            '/lv_dot/input/uav_01/camera/camera_info',
            CameraInfo,
            queue_size=2,
        ),
    }
    while not rospy.is_shutdown():
        connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            connection.connect((host, port))
            rospy.loginfo('LV-DOT ingress connected to %s:%d', host, port)
            while not rospy.is_shutdown():
                if read_exact(connection, 4) != MAGIC:
                    raise ValueError('invalid LV-DOT ingress magic')
                json_size, payload_size = struct.unpack(
                    '!II', read_exact(connection, 8)
                )
                if json_size > 1024 * 1024 or payload_size > 128 * 1024 * 1024:
                    raise ValueError('invalid LV-DOT ingress frame size')
                metadata = json.loads(
                    read_exact(connection, json_size).decode('ascii')
                )
                payload = read_exact(connection, payload_size)
                kind = metadata['kind']
                if kind == 'pose':
                    message = PoseStamped()
                    apply_header(message, metadata)
                    position = metadata['position']
                    orientation = metadata['orientation']
                    message.pose.position.x = position[0]
                    message.pose.position.y = position[1]
                    message.pose.position.z = position[2]
                    message.pose.orientation.x = orientation[0]
                    message.pose.orientation.y = orientation[1]
                    message.pose.orientation.z = orientation[2]
                    message.pose.orientation.w = orientation[3]
                elif kind == 'image':
                    message = Image()
                    apply_header(message, metadata)
                    message.height = metadata['height']
                    message.width = metadata['width']
                    message.encoding = metadata['encoding']
                    message.is_bigendian = metadata['is_bigendian']
                    message.step = metadata['step']
                    message.data = payload
                elif kind == 'pointcloud':
                    message = PointCloud2()
                    apply_header(message, metadata)
                    message.height = metadata['height']
                    message.width = metadata['width']
                    message.fields = [
                        PointField(
                            name=field['name'],
                            offset=field['offset'],
                            datatype=field['datatype'],
                            count=field['count'],
                        )
                        for field in metadata['fields']
                    ]
                    message.is_bigendian = metadata['is_bigendian']
                    message.point_step = metadata['point_step']
                    message.row_step = metadata['row_step']
                    message.is_dense = metadata['is_dense']
                    message.data = payload
                elif kind == 'camera_info':
                    message = CameraInfo()
                    apply_header(message, metadata)
                    message.height = metadata['height']
                    message.width = metadata['width']
                    message.distortion_model = metadata['distortion_model']
                    message.D = metadata['d']
                    message.K = metadata['k']
                    message.R = metadata['r']
                    message.P = metadata['p']
                    message.binning_x = metadata['binning_x']
                    message.binning_y = metadata['binning_y']
                else:
                    continue
                publishers[kind].publish(message)
        except (ConnectionError, OSError, ValueError) as error:
            if not rospy.is_shutdown():
                rospy.logwarn_throttle(
                    5.0, 'LV-DOT ingress unavailable: %s' % error
                )
                time.sleep(1.0)
        finally:
            connection.close()


if __name__ == '__main__':
    main()
