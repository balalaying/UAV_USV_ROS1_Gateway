#!/usr/bin/env python3
"""Expose passive LV-DOT stage outputs through one debug namespace."""

from collections import deque
from copy import deepcopy
import json
import re
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from uav_usv_interfaces.msg import TrackedObjectArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def cluster_markers_from_bboxes(message):
    """Convert bbox markers to lightweight DBSCAN cluster-centre markers."""
    output = MarkerArray()
    for incoming in message.markers:
        if incoming.action == Marker.DELETEALL:
            clear = Marker()
            clear.header = deepcopy(incoming.header)
            clear.ns = 'lv_dot_debug/clusters'
            clear.action = Marker.DELETEALL
            output.markers.append(clear)
            continue
        if incoming.action == Marker.DELETE:
            deleted = Marker()
            deleted.header = deepcopy(incoming.header)
            deleted.ns = 'lv_dot_debug/clusters'
            deleted.id = incoming.id
            deleted.action = Marker.DELETE
            output.markers.append(deleted)
            continue
        if incoming.action != Marker.ADD:
            continue

        point_count = 0
        match = re.search(r'points=(\d+)', incoming.text or '')
        if match:
            point_count = int(match.group(1))
        marker = Marker()
        marker.header = deepcopy(incoming.header)
        marker.ns = 'lv_dot_debug/clusters'
        marker.id = incoming.id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = deepcopy(incoming.pose)
        marker.scale.x = 0.55
        marker.scale.y = 0.55
        marker.scale.z = 0.55
        hue = abs(int(incoming.id)) % 5
        colors = (
            (0.95, 0.25, 0.75),
            (0.25, 0.85, 1.0),
            (1.0, 0.70, 0.20),
            (0.55, 1.0, 0.35),
            (0.75, 0.45, 1.0),
        )
        marker.color.r, marker.color.g, marker.color.b = colors[hue]
        marker.color.a = 1.0
        marker.lifetime = deepcopy(incoming.lifetime)
        marker.text = 'cluster_id=%d;points=%d' % (
            int(incoming.id), point_count
        )
        output.markers.append(marker)
    return output


class TopicRate:
    def __init__(self):
        self.arrivals = deque(maxlen=100)
        self.count = 0
        self.last_count = 0
        self.last_stamp = None

    def update(self, count=0, stamp=None):
        self.arrivals.append(time.monotonic())
        self.count += 1
        self.last_count = int(count)
        self.last_stamp = stamp

    def rate(self):
        if len(self.arrivals) < 2:
            return 0.0
        duration = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / duration if duration > 0 else 0.0


class LvDotDebugVisualizationNode(Node):
    def __init__(self):
        super().__init__('lv_dot_debug_visualization_node')
        defaults = {
            'filtered_cloud_topic': '/perception/usv_01/mid360/points_filtered',
            'bboxes_topic': (
                '/perception/lv_dot_ros2/diagnostics/lidar_bboxes'
            ),
            'tracks_topic': '/perception/lv_dot_ros2/tracks',
            'dynamic_topic': '/perception/lv_dot_ros2/dynamic_tracks',
            'debug_cloud_topic': '/perception/lv_dot/debug/cloud',
            'debug_clusters_topic': '/perception/lv_dot/debug/clusters',
            'debug_bboxes_topic': '/perception/lv_dot/debug/bboxes',
            'debug_tracks_topic': '/perception/lv_dot/debug/tracks',
            'debug_dynamic_topic': '/perception/lv_dot/debug/dynamic',
            'debug_status_topic': '/perception/lv_dot/debug/status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: str(self.get_parameter(name).value)

        self.cloud_pub = self.create_publisher(
            PointCloud2, value('debug_cloud_topic'), qos_profile_sensor_data
        )
        self.cluster_pub = self.create_publisher(
            MarkerArray, value('debug_clusters_topic'), 5
        )
        self.bbox_pub = self.create_publisher(
            MarkerArray, value('debug_bboxes_topic'), 5
        )
        self.track_pub = self.create_publisher(
            TrackedObjectArray, value('debug_tracks_topic'), 10
        )
        self.dynamic_pub = self.create_publisher(
            TrackedObjectArray, value('debug_dynamic_topic'), 10
        )
        self.status_pub = self.create_publisher(
            String, value('debug_status_topic'), 10
        )
        self.statistics = {
            name: TopicRate()
            for name in ('cloud', 'clusters', 'tracks', 'dynamic')
        }

        self.create_subscription(
            PointCloud2, value('filtered_cloud_topic'), self._cloud,
            qos_profile_sensor_data
        )
        self.create_subscription(
            MarkerArray, value('bboxes_topic'), self._bboxes,
            qos_profile_sensor_data
        )
        self.create_subscription(
            TrackedObjectArray, value('tracks_topic'), self._tracks, 10
        )
        self.create_subscription(
            TrackedObjectArray, value('dynamic_topic'), self._dynamic, 10
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            'Passive LV-DOT debug topics ready under /perception/lv_dot/debug'
        )

    def _cloud(self, message):
        self.statistics['cloud'].update(
            int(message.width) * int(message.height), message.header.stamp
        )
        self.cloud_pub.publish(message)

    def _bboxes(self, message):
        clusters = cluster_markers_from_bboxes(message)
        count = sum(
            marker.action == Marker.ADD for marker in message.markers
        )
        self.statistics['clusters'].update(count)
        self.cluster_pub.publish(clusters)
        self.bbox_pub.publish(message)

    def _tracks(self, message):
        self.statistics['tracks'].update(
            len(message.objects), message.header.stamp
        )
        self.track_pub.publish(message)

    def _dynamic(self, message):
        self.statistics['dynamic'].update(
            len(message.objects), message.header.stamp
        )
        self.dynamic_pub.publish(message)

    def _publish_status(self):
        now = time.monotonic()
        payload = {'mode': 'shadow', 'control_connected': False}
        for name, statistics in self.statistics.items():
            age = (
                now - statistics.arrivals[-1]
                if statistics.arrivals else None
            )
            payload[name] = {
                'rate_hz': statistics.rate(),
                'messages': statistics.count,
                'last_count': statistics.last_count,
                'age_seconds': age,
                'online': age is not None and age < 2.0,
            }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LvDotDebugVisualizationNode()
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
