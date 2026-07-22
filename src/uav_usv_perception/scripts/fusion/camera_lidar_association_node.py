#!/usr/bin/env python3
"""Associate USV camera detections with LV-DOT LiDAR clusters."""

from collections import deque
from copy import deepcopy
import hashlib
import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import AffiliatedDetection2DArray
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def stamp_seconds(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def rotation_matrix(rotation):
    x, y, z, w = (
        float(rotation.x), float(rotation.y),
        float(rotation.z), float(rotation.w),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def transform_xyz(point, transform):
    rotation = rotation_matrix(transform.rotation)
    translation = np.asarray([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=np.float64)
    return rotation @ np.asarray(point, dtype=np.float64) + translation


def marker_box(marker):
    if marker.type != Marker.LINE_LIST or not marker.points:
        return None
    values = np.asarray([
        (point.x, point.y, point.z) for point in marker.points
    ], dtype=np.float64)
    dimensions = values.max(axis=0) - values.min(axis=0)
    center = np.asarray([
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z,
    ], dtype=np.float64)
    half = 0.5 * dimensions
    corners = np.asarray([
        center + np.asarray((sx * half[0], sy * half[1], sz * half[2]))
        for sx, sy, sz in (
            (-1, -1, -1), (1, -1, -1),
            (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1),
            (1, 1, 1), (-1, 1, 1),
        )
    ])
    return {
        'id': int(marker.id),
        'center': center,
        'dimensions': dimensions,
        'corners': corners,
        'marker': marker,
    }


def project_point(point, camera_info):
    x, y, z = (float(value) for value in point)
    if x <= 1e-6:
        return None
    fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
    cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
    return (cx - fx * y / x, cy - fy * z / x)


def project_box(box, transform, camera_info):
    projected = []
    for corner in box['corners']:
        pixel = project_point(transform_xyz(corner, transform), camera_info)
        if pixel is not None:
            projected.append(pixel)
    if len(projected) < 4:
        return None
    values = np.asarray(projected)
    return (
        float(values[:, 0].min()), float(values[:, 1].min()),
        float(values[:, 0].max()), float(values[:, 1].max()),
    )


def detection_rect(detection, minimum_width=0.0, minimum_height=0.0):
    center = detection.bbox.center.position
    width = max(float(detection.bbox.size_x), float(minimum_width))
    height = max(float(detection.bbox.size_y), float(minimum_height))
    half_x = 0.5 * width
    half_y = 0.5 * height
    return (
        float(center.x) - half_x, float(center.y) - half_y,
        float(center.x) + half_x, float(center.y) + half_y,
    )


def rectangle_iou(left, right):
    width = max(0.0, min(left[2], right[2]) - max(left[0], right[0]))
    height = max(0.0, min(left[3], right[3]) - max(left[1], right[1]))
    intersection = width * height
    area_left = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    area_right = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = area_left + area_right - intersection
    return intersection / union if union > 1e-9 else 0.0


def association_score(
    detection, projection, pixel_gate=28.0,
    minimum_width=0.0, minimum_height=0.0,
):
    rectangle = detection_rect(detection, minimum_width, minimum_height)
    iou = rectangle_iou(rectangle, projection)
    center_x = 0.5 * (rectangle[0] + rectangle[2])
    center_y = 0.5 * (rectangle[1] + rectangle[3])
    inside = (
        projection[0] - pixel_gate <= center_x <= projection[2] + pixel_gate
        and projection[1] - pixel_gate <= center_y <= projection[3] + pixel_gate
    )
    if not inside and iou <= 0.0:
        return 0.0
    projected_center = (
        0.5 * (projection[0] + projection[2]),
        0.5 * (projection[1] + projection[3]),
    )
    distance = math.hypot(
        center_x - projected_center[0], center_y - projected_center[1]
    )
    proximity = max(0.0, 1.0 - distance / max(1.0, pixel_gate * 3.0))
    return max(iou, 0.35 * proximity)


def marker_copy(marker, namespace, color):
    output = deepcopy(marker)
    output.ns = namespace
    output.color.r, output.color.g, output.color.b, output.color.a = color
    return output


def clear_marker(header, namespace):
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.action = Marker.DELETEALL
    return marker


def stable_uuid(text):
    return list(hashlib.md5(text.encode('utf-8')).digest())


class CameraLidarAssociationNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_association_node')
        defaults = {
            'camera_detections_topic': '/perception/usv_01/camera/detections',
            'camera_info_topic': '/fleet/uplink/usv_01/camera/camera_info',
            'lidar_bboxes_topic': '/perception/lv_dot_ros2/diagnostics/lidar_bboxes',
            'lidar_tracks_topic': '/perception/lv_dot_ros2/tracks',
            'output_topic': '/perception/usv_01/camera_lidar/observations',
            'lidar_only_markers_topic': '/perception/usv_01/camera_lidar/lidar_only_bboxes',
            'camera_only_markers_topic': '/perception/usv_01/camera_lidar/camera_only_bboxes',
            'fused_markers_topic': '/perception/usv_01/camera_lidar/fused_bboxes',
            'status_topic': '/perception/usv_01/camera_lidar/status',
            'affiliated_detections_topic': '/perception/usv_01/camera/affiliated_detections',
            'vision_guided_observations_topic': '/perception/usv_01/vision_guided/observations',
            'camera_frame': 'usv_01/camera_link',
            'output_frame': 'map',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('sync_slop_seconds', 0.20)
        self.declare_parameter('minimum_association_score', 0.08)
        self.declare_parameter('pixel_gate', 32.0)
        self.declare_parameter('known_vessel_width', 2.6)
        self.declare_parameter('minimum_camera_depth', 4.0)
        self.declare_parameter('maximum_camera_depth', 120.0)
        self.declare_parameter('minimum_lidar_xy_extent', 0.20)
        self.declare_parameter('minimum_roi_width_pixels', 70.0)
        self.declare_parameter('minimum_roi_height_pixels', 44.0)
        self.declare_parameter('vision_guided_max_age_seconds', 0.35)
        self.declare_parameter('vision_guided_lidar_gate', 5.0)
        self.declare_parameter('enable_global_lidar_fallback', True)
        self.declare_parameter('affiliation_hold_seconds', 1.0)
        self.declare_parameter('affiliation_unknown_timeout', 2.0)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.sync_slop = max(
            0.01, float(self.get_parameter('sync_slop_seconds').value)
        )
        self.minimum_score = max(
            0.0, float(self.get_parameter('minimum_association_score').value)
        )
        self.pixel_gate = max(1.0, float(self.get_parameter('pixel_gate').value))
        self.known_width = max(
            0.1, float(self.get_parameter('known_vessel_width').value)
        )
        self.min_depth = max(
            0.1, float(self.get_parameter('minimum_camera_depth').value)
        )
        self.max_depth = max(
            self.min_depth,
            float(self.get_parameter('maximum_camera_depth').value),
        )
        self.minimum_lidar_xy_extent = max(
            0.0,
            float(self.get_parameter('minimum_lidar_xy_extent').value),
        )
        self.minimum_roi_width = max(
            0.0,
            float(self.get_parameter('minimum_roi_width_pixels').value),
        )
        self.minimum_roi_height = max(
            0.0,
            float(self.get_parameter('minimum_roi_height_pixels').value),
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.detection_queue = deque(maxlen=30)
        self.metadata_queue = deque(maxlen=30)
        self.latest_vision_guided = None
        self.latest_vision_guided_arrival = 0.0
        self.latest_global_boxes = []
        self.latest_global_boxes_arrival = 0.0
        self.vision_guided_max_age = float(
            self.get_parameter('vision_guided_max_age_seconds').value
        )
        self.vision_guided_lidar_gate = float(
            self.get_parameter('vision_guided_lidar_gate').value
        )
        self.enable_global_lidar_fallback = bool(
            self.get_parameter('enable_global_lidar_fallback').value
        )
        self.affiliation_hold_seconds = float(
            self.get_parameter('affiliation_hold_seconds').value
        )
        self.affiliation_unknown_timeout = float(
            self.get_parameter('affiliation_unknown_timeout').value
        )
        self.identity_memory = {}
        self.identity_inherited_total = 0
        self.latest_tracks = None
        self.first_seen = {}
        self.frames = 0
        self.matched_total = 0
        self.last_counts = {'lidar': 0, 'camera': 0, 'fused': 0}
        self.tf_failures = 0
        self.last_processing_ms = 0.0
        self.last_sync_error_ms = None
        self.last_projected_boxes = 0
        self.last_candidate_pairs = 0
        self.last_best_score = 0.0
        self.last_status_wall = 0.0
        self.output_publisher = self.create_publisher(
            TrackedObjectArray,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.marker_publishers = {
            'lidar': self.create_publisher(
                MarkerArray,
                str(self.get_parameter('lidar_only_markers_topic').value),
                qos_profile_sensor_data,
            ),
            'camera': self.create_publisher(
                MarkerArray,
                str(self.get_parameter('camera_only_markers_topic').value),
                qos_profile_sensor_data,
            ),
            'fused': self.create_publisher(
                MarkerArray,
                str(self.get_parameter('fused_markers_topic').value),
                qos_profile_sensor_data,
            ),
        }
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('camera_detections_topic').value),
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AffiliatedDetection2DArray,
            str(self.get_parameter('affiliated_detections_topic').value),
            self.metadata_queue.append,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackedObjectArray,
            str(self.get_parameter('vision_guided_observations_topic').value),
            self._on_vision_guided,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackedObjectArray,
            str(self.get_parameter('lidar_tracks_topic').value),
            self._on_tracks,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter('lidar_bboxes_topic').value),
            self._on_lidar_bboxes,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            'Camera-LiDAR association ready (Shadow Mode, no control output)'
        )

    def _on_detections(self, message):
        self.detection_queue.append(message)

    def _on_camera_info(self, message):
        self.camera_info = message

    def _on_tracks(self, message):
        self.latest_tracks = message

    def _on_vision_guided(self, message):
        self.latest_vision_guided = message
        self.latest_vision_guided_arrival = time.monotonic()
        boxes = self.latest_global_boxes
        if time.monotonic() - self.latest_global_boxes_arrival > 0.5:
            boxes = []
        header = deepcopy(message.header)
        self._publish_vision_guided(boxes, header, time.perf_counter())

    def _metadata_for(self, detection, stamp):
        target = stamp_seconds(stamp)
        best, error = None, math.inf
        for message in self.metadata_queue:
            delta = abs(stamp_seconds(message.header.stamp) - target)
            if delta < error:
                best, error = message, delta
        if best is None or error > self.sync_slop:
            return None
        return next((item for item in best.detections
                     if item.detection_id == detection.id), None)

    def _nearest_detections(self, stamp):
        target = stamp_seconds(stamp)
        selected = None
        error = math.inf
        for message in self.detection_queue:
            delta = abs(stamp_seconds(message.header.stamp) - target)
            if delta < error:
                selected, error = message, delta
        self.last_sync_error_ms = (
            None if not math.isfinite(error) else 1000.0 * error
        )
        return selected if error <= self.sync_slop else None

    def _lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.04),
            ).transform
        except TransformException:
            self.tf_failures += 1
            return None

    def _active_camera_frame(self):
        if self.camera_info is not None and self.camera_info.header.frame_id:
            return self.camera_info.header.frame_id
        return self.camera_frame

    def _nearest_track(self, center):
        if self.latest_tracks is None:
            return None
        best = None
        best_distance = 5.0
        for tracked in self.latest_tracks.objects:
            position = tracked.pose.pose.position
            distance = math.hypot(
                float(position.x) - center[0],
                float(position.y) - center[1],
            )
            if distance < best_distance:
                best, best_distance = tracked, distance
        return best

    @staticmethod
    def _detection_confidence(detection):
        if not detection.results:
            return 0.5
        return min(1.0, max(0.0, float(
            detection.results[0].hypothesis.score
        )))

    def _remember_identity(self, tracked):
        if tracked.affiliation == TrackedObject.AFFILIATION_UNKNOWN:
            return
        if tracked.affiliation_confidence <= 0.0:
            return
        self.identity_memory[tracked.track_id] = (
            int(tracked.affiliation),
            float(tracked.affiliation_confidence),
            time.monotonic(),
        )

    def _apply_identity_hold(self, tracked):
        remembered = self.identity_memory.get(tracked.track_id)
        if remembered is None:
            return
        affiliation, confidence, updated = remembered
        age = time.monotonic() - updated
        if age > self.affiliation_unknown_timeout:
            self.identity_memory.pop(tracked.track_id, None)
            return
        decay_window = max(
            1e-6,
            self.affiliation_unknown_timeout - self.affiliation_hold_seconds,
        )
        decay = 1.0
        if age > self.affiliation_hold_seconds:
            decay = max(
                0.0,
                1.0 - (age - self.affiliation_hold_seconds) / decay_window,
            )
        tracked.affiliation = affiliation
        tracked.affiliation_confidence = confidence * decay
        self.identity_inherited_total += 1

    def _tracked_from_box(self, box, stamp, source, detection=None):
        nearest = self._nearest_track(box['center'])
        if nearest is not None:
            tracked = deepcopy(nearest)
        else:
            tracked = TrackedObject()
            tracked.track_id = 'cluster_%03d' % box['id']
            tracked.uuid.uuid = stable_uuid(tracked.track_id)
            tracked.pose.pose.position.x = float(box['center'][0])
            tracked.pose.pose.position.y = float(box['center'][1])
            tracked.pose.pose.position.z = float(box['center'][2])
            tracked.pose.pose.orientation.w = 1.0
            tracked.dimensions.x = float(box['dimensions'][0])
            tracked.dimensions.y = float(box['dimensions'][1])
            tracked.dimensions.z = float(box['dimensions'][2])
            tracked.confidence = 0.62
        tracked.last_update = deepcopy(stamp)
        if tracked.track_id not in self.first_seen:
            self.first_seen[tracked.track_id] = deepcopy(stamp)
        tracked.first_seen = deepcopy(self.first_seen[tracked.track_id])
        if source == 'fused':
            camera_confidence = self._detection_confidence(detection)
            lidar_confidence = min(1.0, max(0.0, float(tracked.confidence)))
            tracked.source_mask = (
                TrackedObject.SOURCE_LIDAR
                | TrackedObject.SOURCE_CAMERA
                | TrackedObject.SOURCE_FUSED
            )
            tracked.classification = TrackedObject.CLASS_VESSEL
            tracked.class_name = 'vessel'
            tracked.class_confidence = float(camera_confidence)
            tracked.sensor_source = 'camera+lidar'
            tracked.confidence = 1.0 - (
                (1.0 - lidar_confidence) * (1.0 - camera_confidence)
            )
            metadata = self._metadata_for(detection, stamp)
            if metadata is not None:
                tracked.class_name = metadata.class_name or 'vessel'
                tracked.class_confidence = metadata.class_confidence
                tracked.affiliation = metadata.affiliation
                tracked.affiliation_confidence = metadata.affiliation_confidence
            self._remember_identity(tracked)
        else:
            tracked.source_mask = TrackedObject.SOURCE_LIDAR
            tracked.class_name = tracked.class_name or 'unknown'
            tracked.class_confidence = float(tracked.class_confidence)
            tracked.sensor_source = 'lidar'
            tracked.affiliation = TrackedObject.AFFILIATION_UNKNOWN
            tracked.affiliation_confidence = 0.0
            self._apply_identity_hold(tracked)
        return tracked

    def _camera_only_track(self, detection, transform, stamp):
        fx, fy = float(self.camera_info.k[0]), float(self.camera_info.k[4])
        cx, cy = float(self.camera_info.k[2]), float(self.camera_info.k[5])
        width = max(1.0, float(detection.bbox.size_x))
        depth = min(self.max_depth, max(
            self.min_depth, fx * self.known_width / width
        ))
        pixel_x = float(detection.bbox.center.position.x)
        pixel_y = float(detection.bbox.center.position.y)
        camera_point = np.asarray([
            depth,
            -(pixel_x - cx) * depth / max(1e-6, fx),
            -(pixel_y - cy) * depth / max(1e-6, fy),
        ])
        point = transform_xyz(camera_point, transform)
        tracked = TrackedObject()
        tracked.track_id = detection.id or 'camera_vessel'
        tracked.uuid.uuid = stable_uuid(tracked.track_id)
        if tracked.track_id not in self.first_seen:
            self.first_seen[tracked.track_id] = deepcopy(stamp)
        tracked.first_seen = deepcopy(self.first_seen[tracked.track_id])
        tracked.last_update = deepcopy(stamp)
        tracked.source_mask = TrackedObject.SOURCE_CAMERA
        tracked.classification = TrackedObject.CLASS_VESSEL
        tracked.class_name = 'vessel'
        tracked.class_confidence = self._detection_confidence(detection)
        tracked.sensor_source = 'camera'
        metadata = self._metadata_for(detection, stamp)
        if metadata is not None:
            tracked.class_name = metadata.class_name or 'vessel'
            tracked.class_confidence = metadata.class_confidence
            tracked.affiliation = metadata.affiliation
            tracked.affiliation_confidence = metadata.affiliation_confidence
        self._remember_identity(tracked)
        tracked.pose.pose.position.x = float(point[0])
        tracked.pose.pose.position.y = float(point[1])
        tracked.pose.pose.position.z = float(point[2])
        tracked.pose.pose.orientation.w = 1.0
        tracked.dimensions.x = 7.0
        tracked.dimensions.y = self.known_width
        tracked.dimensions.z = 2.5
        tracked.confidence = tracked.class_confidence * 0.65
        for index in (0, 7, 14):
            tracked.pose.covariance[index] = 16.0
        return tracked

    @staticmethod
    def _camera_marker(tracked, marker_id, header):
        marker = Marker()
        marker.header = header
        marker.ns = 'camera_lidar/camera_only'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = tracked.pose.pose
        marker.scale = tracked.dimensions
        marker.color.r = 0.15
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.nanosec = 250000000
        return marker

    @staticmethod
    def _object_marker(tracked, marker_id, header, namespace, color):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = tracked.pose.pose
        marker.scale = tracked.dimensions
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = json.dumps({
            'track_id': tracked.track_id,
            'class_name': tracked.class_name or 'unknown',
            'affiliation': int(tracked.affiliation),
            'affiliation_confidence': float(tracked.affiliation_confidence),
            'sensor_source': tracked.sensor_source,
            'speed': math.hypot(
                tracked.twist.twist.linear.x, tracked.twist.twist.linear.y
            ),
            'association_score': float(tracked.association_score),
            'bbox_point_count': int(tracked.bbox_point_count),
        }, sort_keys=True)
        marker.lifetime.nanosec = 300000000
        return marker

    def _publish_vision_guided(self, boxes, header, started):
        """Prefer camera-generated local geometry and append global fallback."""
        if (
            self.latest_vision_guided is None
            or time.monotonic() - self.latest_vision_guided_arrival
            > self.vision_guided_max_age
        ):
            return False
        guided = self.latest_vision_guided
        marker_arrays = {
            key: MarkerArray(markers=[clear_marker(header, namespace)])
            for key, namespace in (
                ('lidar', 'camera_lidar/lidar_only'),
                ('camera', 'camera_lidar/camera_only'),
                ('fused', 'camera_lidar/fused'),
            )
        }
        output = TrackedObjectArray()
        output.header.stamp = header.stamp
        output.header.frame_id = self.output_frame
        claimed_boxes = set()
        for index, tracked in enumerate(guided.objects):
            output.objects.append(deepcopy(tracked))
            self._remember_identity(tracked)
            source = tracked.sensor_source
            if source == 'camera+lidar':
                key, namespace, color = (
                    'fused', 'camera_lidar/fused', (0.1, 1.0, 0.25, 0.34)
                )
                center = tracked.pose.pose.position
                nearest_index, nearest_distance = None, self.vision_guided_lidar_gate
                for box_index, box in enumerate(boxes):
                    if box_index in claimed_boxes:
                        continue
                    distance = math.hypot(
                        center.x - box['center'][0], center.y - box['center'][1]
                    )
                    if distance < nearest_distance:
                        nearest_index, nearest_distance = box_index, distance
                if nearest_index is not None:
                    claimed_boxes.add(nearest_index)
            else:
                key, namespace, color = (
                    'camera', 'camera_lidar/camera_only', (0.15, 0.45, 1.0, 0.35)
                )
            marker_arrays[key].markers.append(self._object_marker(
                tracked, index, header, namespace, color
            ))
        for box_index, box in enumerate(boxes):
            if box_index in claimed_boxes:
                continue
            if not self.enable_global_lidar_fallback:
                continue
            marker_arrays['lidar'].markers.append(marker_copy(
                box['marker'], 'camera_lidar/lidar_only',
                (1.0, 0.75, 0.12, 1.0),
            ))
            output.objects.append(self._tracked_from_box(
                box, header.stamp, 'lidar'
            ))
        for key, publisher in self.marker_publishers.items():
            publisher.publish(marker_arrays[key])
        self.output_publisher.publish(output)
        fused_count = sum(
            item.sensor_source == 'camera+lidar' for item in guided.objects
        )
        camera_count = sum(item.sensor_source == 'camera' for item in guided.objects)
        self.frames += 1
        self.matched_total += fused_count
        self.last_counts = {
            'lidar': len(boxes) - len(claimed_boxes),
            'camera': camera_count,
            'fused': fused_count,
        }
        self.last_processing_ms = (time.perf_counter() - started) * 1000.0
        now_wall = time.monotonic()
        if now_wall - self.last_status_wall >= 1.0:
            self.last_status_wall = now_wall
            self._publish_status()
        return True

    def _on_lidar_bboxes(self, message):
        started = time.perf_counter()
        additions = [
            marker for marker in message.markers
            if marker.action == Marker.ADD
        ]
        if not additions:
            if message.markers:
                header = deepcopy(message.markers[0].header)
            elif self.latest_vision_guided is not None:
                header = deepcopy(self.latest_vision_guided.header)
            else:
                return
            self.latest_global_boxes = []
            self.latest_global_boxes_arrival = time.monotonic()
            # Camera-led observations publish from their own callback. LiDAR
            # marker callbacks only refresh the optional fallback cache.
            if (
                self.latest_vision_guided is not None
                and time.monotonic() - self.latest_vision_guided_arrival
                <= self.vision_guided_max_age
            ):
                return
            for key, namespace in (
                ('lidar', 'camera_lidar/lidar_only'),
                ('camera', 'camera_lidar/camera_only'),
                ('fused', 'camera_lidar/fused'),
            ):
                self.marker_publishers[key].publish(MarkerArray(
                    markers=[clear_marker(header, namespace)]
                ))
            output = TrackedObjectArray()
            output.header = header
            self.output_publisher.publish(output)
            self.last_counts = {'lidar': 0, 'camera': 0, 'fused': 0}
            return
        header = deepcopy(additions[0].header)
        boxes = [value for value in (
            marker_box(marker) for marker in additions
        ) if (
            value is not None
            and max(value['dimensions'][0], value['dimensions'][1])
            >= self.minimum_lidar_xy_extent
        )]
        self.latest_global_boxes = boxes
        self.latest_global_boxes_arrival = time.monotonic()
        if (
            self.latest_vision_guided is not None
            and time.monotonic() - self.latest_vision_guided_arrival
            <= self.vision_guided_max_age
        ):
            return
        detections_message = self._nearest_detections(header.stamp)
        detections = list(
            detections_message.detections
            if detections_message is not None else []
        )
        camera_transform = None
        map_transform = None
        if self.camera_info is not None:
            active_camera_frame = self._active_camera_frame()
            camera_transform = self._lookup(
                active_camera_frame, header.frame_id, header.stamp
            )
            map_transform = self._lookup(
                self.output_frame, active_camera_frame, header.stamp
            )
        candidates = []
        projected_boxes = 0
        if camera_transform is not None:
            for box_index, box in enumerate(boxes):
                projection = project_box(
                    box, camera_transform, self.camera_info
                )
                if projection is None:
                    continue
                projected_boxes += 1
                for detection_index, detection in enumerate(detections):
                    score = association_score(
                        detection, projection, self.pixel_gate,
                        self.minimum_roi_width, self.minimum_roi_height,
                    )
                    if score >= self.minimum_score:
                        candidates.append((score, box_index, detection_index))
        self.last_projected_boxes = projected_boxes
        self.last_candidate_pairs = len(candidates)
        self.last_best_score = max(
            (candidate[0] for candidate in candidates), default=0.0
        )
        candidates.sort(reverse=True)
        matched_boxes = {}
        matched_detections = set()
        for score, box_index, detection_index in candidates:
            if box_index in matched_boxes or detection_index in matched_detections:
                continue
            matched_boxes[box_index] = detection_index
            matched_detections.add(detection_index)

        marker_arrays = {
            key: MarkerArray(markers=[clear_marker(header, namespace)])
            for key, namespace in (
                ('lidar', 'camera_lidar/lidar_only'),
                ('camera', 'camera_lidar/camera_only'),
                ('fused', 'camera_lidar/fused'),
            )
        }
        output = TrackedObjectArray()
        output.header.stamp = header.stamp
        output.header.frame_id = self.output_frame
        for box_index, box in enumerate(boxes):
            if box_index in matched_boxes:
                detection = detections[matched_boxes[box_index]]
                marker_arrays['fused'].markers.append(marker_copy(
                    box['marker'], 'camera_lidar/fused', (0.1, 1.0, 0.25, 1.0)
                ))
                output.objects.append(self._tracked_from_box(
                    box, header.stamp, 'fused', detection
                ))
            else:
                marker_arrays['lidar'].markers.append(marker_copy(
                    box['marker'], 'camera_lidar/lidar_only',
                    (1.0, 0.75, 0.12, 1.0),
                ))
                output.objects.append(self._tracked_from_box(
                    box, header.stamp, 'lidar'
                ))
        if map_transform is not None:
            for index, detection in enumerate(detections):
                if index in matched_detections:
                    continue
                tracked = self._camera_only_track(
                    detection, map_transform, header.stamp
                )
                output.objects.append(tracked)
                marker_arrays['camera'].markers.append(
                    self._camera_marker(tracked, index, header)
                )
        for key, publisher in self.marker_publishers.items():
            publisher.publish(marker_arrays[key])
        self.output_publisher.publish(output)
        self.frames += 1
        self.matched_total += len(matched_boxes)
        self.last_counts = {
            'lidar': len(boxes) - len(matched_boxes),
            'camera': len(detections) - len(matched_detections),
            'fused': len(matched_boxes),
        }
        self.last_processing_ms = (time.perf_counter() - started) * 1000.0

    def _publish_status(self):
        message = String()
        message.data = json.dumps({
            'mode': 'shadow_camera_lidar_fusion',
            'frames': self.frames,
            'last_counts': self.last_counts,
            'lidar_only_count': self.last_counts['lidar'],
            'camera_only_count': self.last_counts['camera'],
            'fused_count': self.last_counts['fused'],
            'matched_total': self.matched_total,
            'tf_failures': self.tf_failures,
            'camera_info_online': self.camera_info is not None,
            'camera_frame': self._active_camera_frame(),
            'configured_camera_frame': self.camera_frame,
            'queued_detection_frames': len(self.detection_queue),
            'sync_error_ms': self.last_sync_error_ms,
            'projected_lidar_boxes': self.last_projected_boxes,
            'association_candidates': self.last_candidate_pairs,
            'best_association_score': self.last_best_score,
            'processing_ms': self.last_processing_ms,
            'control_connected': False,
            'perception_source': 'ground_truth',
            'vision_guided_preferred': True,
            'vision_guided_online': (
                self.latest_vision_guided is not None
                and time.monotonic() - self.latest_vision_guided_arrival
                <= self.vision_guided_max_age
            ),
            'identity_memory_tracks': len(self.identity_memory),
            'identity_inherited_total': self.identity_inherited_total,
        }, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarAssociationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
