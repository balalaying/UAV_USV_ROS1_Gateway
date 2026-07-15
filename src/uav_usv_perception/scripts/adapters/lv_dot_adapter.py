#!/usr/bin/env python3
"""Convert bridged LV-DOT visualization output into fleet observations."""

from dataclasses import dataclass
import math
import re
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


_UUID_NAMESPACE = uuid.UUID('d06825ee-9e55-4d9f-b91f-d37070678fb1')
_VELOCITY_PATTERN = re.compile(
    r'Vx\s*=\s*([-+0-9.eE]+).*?Vy\s*=\s*([-+0-9.eE]+)'
)


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _marker_dimensions(marker):
    if not marker.points:
        return (1.0, 1.0, 1.0)
    xs = [float(point.x) for point in marker.points]
    ys = [float(point.y) for point in marker.points]
    zs = [float(point.z) for point in marker.points]
    return (
        max(0.05, max(xs) - min(xs)),
        max(0.05, max(ys) - min(ys)),
        max(0.05, max(zs) - min(zs)),
    )


def _marker_volume(marker):
    dimensions = _marker_dimensions(marker)
    return dimensions[0] * dimensions[1] * dimensions[2]


@dataclass
class AdapterTrack:
    track_id: str
    position: tuple
    velocity: tuple
    first_seen: object
    last_stamp: object
    last_stamp_seconds: float
    last_arrival: float


class LvDotAdapter(Node):
    """Adapt LV-DOT MarkerArray topics without changing LV-DOT itself.

    Upstream LV-DOT exposes dynamic boxes and velocity text as visualization
    messages. It does not expose persistent IDs or confidence, so this adapter
    maintains IDs by nearest-neighbour association and marks confidence as an
    adapter parameter.
    """

    def __init__(self):
        super().__init__('lv_dot_adapter')
        self.declare_parameter(
            'bbox_topic', '/lv_dot/onboard_detector/dynamic_bboxes'
        )
        self.declare_parameter(
            'velocity_topic',
            '/lv_dot/onboard_detector/velocity_visualizaton',
        )
        self.declare_parameter(
            'output_topic', '/perception/lv_dot/observations'
        )
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('association_distance_m', 4.0)
        self.declare_parameter('track_timeout_seconds', 1.5)
        self.declare_parameter('velocity_match_distance_m', 2.0)
        self.declare_parameter('deduplication_distance_m', 1.0)
        self.declare_parameter('velocity_smoothing_alpha', 0.65)
        self.declare_parameter('default_confidence', 0.70)
        self.declare_parameter(
            'source_mask', int(TrackedObject.SOURCE_LIDAR)
        )
        self.declare_parameter('position_variance', 1.0)
        self.declare_parameter('velocity_variance', 2.0)
        self.declare_parameter('publish_empty_arrays', True)

        self.bbox_topic = str(self.get_parameter('bbox_topic').value)
        self.velocity_topic = str(self.get_parameter('velocity_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.association_distance = max(
            0.05,
            float(self.get_parameter('association_distance_m').value),
        )
        self.track_timeout = max(
            0.1, float(self.get_parameter('track_timeout_seconds').value)
        )
        self.velocity_match_distance = max(
            0.05,
            float(self.get_parameter('velocity_match_distance_m').value),
        )
        self.deduplication_distance = max(
            0.0,
            float(self.get_parameter('deduplication_distance_m').value),
        )
        self.velocity_alpha = min(
            1.0,
            max(
                0.0,
                float(
                    self.get_parameter('velocity_smoothing_alpha').value
                ),
            ),
        )
        self.default_confidence = min(
            1.0,
            max(0.0, float(self.get_parameter('default_confidence').value)),
        )
        self.source_mask = min(
            255, max(0, int(self.get_parameter('source_mask').value))
        )
        self.position_variance = max(
            0.0, float(self.get_parameter('position_variance').value)
        )
        self.velocity_variance = max(
            0.0, float(self.get_parameter('velocity_variance').value)
        )
        self.publish_empty = bool(
            self.get_parameter('publish_empty_arrays').value
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray, self.output_topic, 10
        )
        self.create_subscription(
            MarkerArray, self.bbox_topic, self._on_boxes, 10
        )
        self.create_subscription(
            MarkerArray, self.velocity_topic, self._on_velocities, 10
        )
        self.tracks = {}
        self.next_track_number = 1
        self.velocity_markers = []
        self.velocity_arrival = 0.0
        self.last_frame_warning = 0.0
        self.get_logger().info(
            'LV-DOT adapter %s + %s -> %s'
            % (self.bbox_topic, self.velocity_topic, self.output_topic)
        )

    def _on_velocities(self, message):
        parsed = []
        for marker in message.markers:
            match = _VELOCITY_PATTERN.search(marker.text or '')
            if match is None:
                continue
            parsed.append((
                (
                    float(marker.pose.position.x),
                    float(marker.pose.position.y),
                ),
                (float(match.group(1)), float(match.group(2)), 0.0),
            ))
        self.velocity_markers = parsed
        self.velocity_arrival = time.monotonic()

    def _velocity_for_position(self, position):
        if time.monotonic() - self.velocity_arrival > self.track_timeout:
            return None
        nearest = None
        nearest_distance = self.velocity_match_distance
        for marker_position, velocity in self.velocity_markers:
            distance = _distance(position, marker_position)
            if distance <= nearest_distance:
                nearest = velocity
                nearest_distance = distance
        return nearest

    def _associate(self, position, assigned):
        selected = None
        selected_distance = self.association_distance
        for track_id, track in self.tracks.items():
            if track_id in assigned:
                continue
            distance = _distance(position, track.position)
            if distance <= selected_distance:
                selected = track
                selected_distance = distance
        return selected

    def _new_track(self, position, stamp, stamp_seconds, arrival):
        track_id = 'lv_dot_%03d' % self.next_track_number
        self.next_track_number += 1
        track = AdapterTrack(
            track_id=track_id,
            position=position,
            velocity=(0.0, 0.0, 0.0),
            first_seen=stamp,
            last_stamp=stamp,
            last_stamp_seconds=stamp_seconds,
            last_arrival=arrival,
        )
        self.tracks[track_id] = track
        return track

    def _deduplicated_markers(self, markers):
        """Keep the largest box from each cluster of overlapping detections."""
        active = [
            marker for marker in markers
            if marker.action in (Marker.ADD, Marker.MODIFY)
        ]
        if self.deduplication_distance <= 0.0:
            return active

        selected = []
        for marker in sorted(active, key=_marker_volume, reverse=True):
            position = (
                float(marker.pose.position.x),
                float(marker.pose.position.y),
            )
            if any(
                _distance(position, (
                    float(other.pose.position.x),
                    float(other.pose.position.y),
                )) <= self.deduplication_distance
                for other in selected
            ):
                continue
            selected.append(marker)
        return selected

    def _tracked_object(self, marker, track):
        output = TrackedObject()
        output.uuid.uuid = list(
            uuid.uuid5(_UUID_NAMESPACE, track.track_id).bytes
        )
        output.track_id = track.track_id
        output.first_seen = track.first_seen
        output.last_update = track.last_stamp
        output.source_mask = self.source_mask
        output.classification = TrackedObject.CLASS_UNKNOWN
        output.pose.pose = marker.pose
        for index in (0, 7, 14):
            output.pose.covariance[index] = self.position_variance
        output.twist.twist.linear.x = track.velocity[0]
        output.twist.twist.linear.y = track.velocity[1]
        output.twist.twist.linear.z = track.velocity[2]
        for index in (0, 7, 14):
            output.twist.covariance[index] = self.velocity_variance
        dimensions = _marker_dimensions(marker)
        output.dimensions.x = dimensions[0]
        output.dimensions.y = dimensions[1]
        output.dimensions.z = dimensions[2]
        output.confidence = self.default_confidence
        return output

    def _on_boxes(self, message):
        arrival = time.monotonic()
        now = self.get_clock().now()
        output = TrackedObjectArray()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self.target_frame
        assigned = set()

        for marker in self._deduplicated_markers(message.markers):
            source_frame = marker.header.frame_id or self.target_frame
            if source_frame != self.target_frame:
                if arrival - self.last_frame_warning > 5.0:
                    self.get_logger().warning(
                        'Ignoring LV-DOT marker in %s; expected %s. '
                        'Bridge the upstream map TF instead of relabeling it.'
                        % (source_frame, self.target_frame)
                    )
                    self.last_frame_warning = arrival
                continue

            stamp = marker.header.stamp
            stamp_seconds = _stamp_seconds(stamp)
            if stamp_seconds <= 0.0:
                stamp = now.to_msg()
                stamp_seconds = now.nanoseconds * 1e-9
            position = (
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
            )
            track = self._associate(position, assigned)
            if track is None:
                track = self._new_track(
                    position, stamp, stamp_seconds, arrival
                )

            measured_velocity = self._velocity_for_position(position)
            if measured_velocity is None:
                dt = stamp_seconds - track.last_stamp_seconds
                if dt > 1e-3:
                    measured_velocity = (
                        (position[0] - track.position[0]) / dt,
                        (position[1] - track.position[1]) / dt,
                        (position[2] - track.position[2]) / dt,
                    )
            if measured_velocity is not None:
                track.velocity = tuple(
                    previous + self.velocity_alpha * (measured - previous)
                    for previous, measured in zip(
                        track.velocity, measured_velocity
                    )
                )
            track.position = position
            track.last_stamp = stamp
            track.last_stamp_seconds = stamp_seconds
            track.last_arrival = arrival
            assigned.add(track.track_id)
            output.objects.append(self._tracked_object(marker, track))

        expired = [
            track_id for track_id, track in self.tracks.items()
            if arrival - track.last_arrival > self.track_timeout
        ]
        for track_id in expired:
            del self.tracks[track_id]
        if output.objects or self.publish_empty:
            self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LvDotAdapter()
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
