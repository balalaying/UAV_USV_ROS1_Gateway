#!/usr/bin/env python3
"""Generate robust 3D observations from camera ROIs and Mid-360 points."""

from collections import deque
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from uav_usv_interfaces.msg import AffiliatedDetection2DArray
from uav_usv_interfaces.msg import TrackedObject, TrackedObjectArray
from visualization_msgs.msg import Marker, MarkerArray

# See the camera adapter for why both locations are required under colcon.
for _module_dir in (
    Path(sys.argv[0]).resolve().parent,
    Path(__file__).resolve().parents[1] / 'vision_guided',
):
    if str(_module_dir) not in sys.path:
        sys.path.insert(0, str(_module_dir))

from vision_guided_core import BboxSmoother
from vision_guided_core import bbox_is_plausible
from vision_guided_core import dbscan
from vision_guided_core import depth_filter
from vision_guided_core import project_camera_points
from vision_guided_core import robust_oriented_bbox
from vision_guided_core import roi_indices
from vision_guided_core import select_cluster
from vision_guided_core import unproject_camera_pixels


def stamp_seconds(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def stable_uuid(text):
    return list(hashlib.md5(text.encode('utf-8')).digest())


def transform_matrix(transform):
    q = transform.rotation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        rotation = np.eye(3)
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        rotation = np.asarray([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ])
    translation = np.asarray((
        transform.translation.x, transform.translation.y,
        transform.translation.z,
    ))
    return rotation, translation


def apply_transform(points, transform):
    rotation, translation = transform_matrix(transform)
    return np.asarray(points) @ rotation.T + translation


def cloud_xyz(message):
    fields = {field.name: field for field in message.fields}
    required = [fields.get(name) for name in ('x', 'y', 'z')]
    if any(field is None or field.datatype != PointField.FLOAT32 for field in required):
        raise ValueError('PointCloud2 must contain FLOAT32 x/y/z')
    count = int(message.width * message.height)
    endian = '>' if message.is_bigendian else '<'
    values = [np.ndarray(
        (count,), dtype=endian + 'f4', buffer=message.data,
        offset=field.offset, strides=(message.point_step,),
    ) for field in required]
    return np.column_stack(values).astype(np.float64, copy=False)


class VisionGuidedLidarRoiNode(Node):
    def __init__(self):
        super().__init__('vision_guided_lidar_roi_node')
        defaults = {
            'detections_topic': '/perception/usv_01/camera/affiliated_detections',
            'camera_info_topic': '/fleet/uplink/usv_01/camera/camera_info',
            'points_topic': '/perception/usv_01/mid360/points_filtered',
            'tracks_topic': '/perception/lv_dot_ros2/tracks',
            'observations_topic': '/perception/usv_01/vision_guided/observations',
            'roi_cloud_topic': '/perception/usv_01/vision_guided/roi_cloud',
            'roi_clusters_topic': '/perception/usv_01/vision_guided/roi_clusters',
            'roi_bboxes_topic': '/perception/usv_01/vision_guided/roi_bboxes',
            'camera_projection_topic': (
                '/perception/usv_01/vision_guided/camera_projection'
            ),
            'status_topic': '/perception/usv_01/vision_guided/status',
            'camera_frame': 'usv_01/camera_link',
            'output_frame': 'map',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('shadow_mode', True)
        numeric = {
            'sync_slop_seconds': 0.20, 'roi_expand_pixels': 14.0,
            'minimum_roi_width_pixels': 70.0,
            'minimum_roi_height_pixels': 44.0,
            'minimum_roi_points': 8, 'maximum_roi_points': 600,
            'minimum_depth': 1.0, 'maximum_depth': 90.0,
            'depth_percentile_low': 5.0, 'depth_percentile_high': 92.0,
            'depth_outlier_threshold': 1.2, 'local_cluster_epsilon': 0.85,
            'local_cluster_min_samples': 5, 'water_surface_z': 0.0,
            'water_surface_margin': 0.05, 'minimum_length': 0.15,
            'maximum_length': 18.0, 'minimum_width': 0.10,
            'maximum_width': 8.0, 'minimum_height': 0.05,
            'maximum_height': 8.0, 'minimum_confirmed_frames': 2,
            'track_prediction_gate': 3.0, 'bbox_smoothing_alpha': 0.45,
            'maximum_bbox_jump': 0.75, 'known_vessel_width': 2.6,
            'bbox_jump_confirmation_frames': 3,
            'tf_lookup_timeout_seconds': 0.20,
            'minimum_occluded_extent': 0.60,
        }
        for name, value in numeric.items():
            self.declare_parameter(name, value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.shadow_mode = bool(self.get_parameter('shadow_mode').value)
        self.sync_slop = float(self.get_parameter('sync_slop_seconds').value)
        self.expand = float(self.get_parameter('roi_expand_pixels').value)
        self.minimum_roi_width = float(
            self.get_parameter('minimum_roi_width_pixels').value
        )
        self.minimum_roi_height = float(
            self.get_parameter('minimum_roi_height_pixels').value
        )
        self.min_points = int(self.get_parameter('minimum_roi_points').value)
        self.max_points = int(self.get_parameter('maximum_roi_points').value)
        self.min_depth = float(self.get_parameter('minimum_depth').value)
        self.max_depth = float(self.get_parameter('maximum_depth').value)
        self.depth_low = float(self.get_parameter('depth_percentile_low').value)
        self.depth_high = float(self.get_parameter('depth_percentile_high').value)
        self.depth_threshold = float(self.get_parameter('depth_outlier_threshold').value)
        self.cluster_epsilon = float(self.get_parameter('local_cluster_epsilon').value)
        self.cluster_min_samples = int(self.get_parameter('local_cluster_min_samples').value)
        self.water_z = float(self.get_parameter('water_surface_z').value)
        self.water_margin = float(self.get_parameter('water_surface_margin').value)
        self.confirmed_frames = int(self.get_parameter('minimum_confirmed_frames').value)
        self.track_gate = float(self.get_parameter('track_prediction_gate').value)
        self.known_width = float(self.get_parameter('known_vessel_width').value)
        self.tf_lookup_timeout = float(
            self.get_parameter('tf_lookup_timeout_seconds').value
        )
        self.minimum_occluded_extent = float(
            self.get_parameter('minimum_occluded_extent').value
        )
        self.limits = {name: float(self.get_parameter(name).value) for name in (
            'minimum_length', 'maximum_length', 'minimum_width', 'maximum_width',
            'minimum_height', 'maximum_height',
        )}
        self.smoother = BboxSmoother(
            self.get_parameter('bbox_smoothing_alpha').value,
            self.get_parameter('maximum_bbox_jump').value,
            self.get_parameter('bbox_jump_confirmation_frames').value,
        )
        self.detections = deque(maxlen=30)
        self.camera_info = None
        self.tracks = None
        self.confirmation_counts = {}
        self.first_seen = {}
        self.last_status_wall = 0.0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=8.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.observations_pub = self.create_publisher(
            TrackedObjectArray, str(self.get_parameter('observations_topic').value),
            qos_profile_sensor_data,
        )
        self.roi_cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('roi_cloud_topic').value),
            qos_profile_sensor_data,
        )
        self.cluster_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('roi_clusters_topic').value),
            qos_profile_sensor_data,
        )
        self.bbox_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('roi_bboxes_topic').value),
            qos_profile_sensor_data,
        )
        self.camera_projection_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('camera_projection_topic').value),
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )
        self.create_subscription(
            AffiliatedDetection2DArray,
            str(self.get_parameter('detections_topic').value),
            self.detections.append, qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackedObjectArray, str(self.get_parameter('tracks_topic').value),
            self._on_tracks, qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter('points_topic').value),
            self._on_cloud, qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_status)
        self.stats = {
            'lidar_frames': 0, 'roi_extraction_frames': 0,
            'roi_empty_count': 0, 'roi_points_total': 0, 'roi_clusters': 0,
            'valid_3d_bboxes': 0, 'rejected_3d_bboxes': 0,
            'camera_lidar_matches': 0, 'camera_only_count': 0,
            'tf_failures': 0, 'track_id_switches': 0,
            'insufficient_roi_points': 0, 'depth_filter_rejections': 0,
            'cluster_selection_rejections': 0,
            'bbox_plausibility_rejections': 0,
            'bbox_jump_rejections': 0, 'unconfirmed_frames': 0,
            'cross_time_tf_frames': 0,
        }
        self.last = {'sync_ms': None, 'projection_ms': 0.0, 'roi_ms': 0.0,
                     'total_ms': 0.0, 'roi_points': 0,
                     'roi_inside_ratio': None,
                     'reprojection_center_error_px': None,
                     'camera_projection_depth_m': None,
                     'camera_from_lidar': None}
        self.get_logger().info(
            'Vision-guided LiDAR ROI active in Shadow Mode; control output disabled'
        )

    def _on_camera_info(self, message):
        self.camera_info = message

    def _on_tracks(self, message):
        self.tracks = message

    def _nearest_detections(self, stamp):
        target = stamp_seconds(stamp)
        best, error = None, math.inf
        for message in self.detections:
            delta = abs(stamp_seconds(message.header.stamp) - target)
            if delta < error:
                best, error = message, delta
        self.last['sync_ms'] = None if not math.isfinite(error) else error * 1000.0
        return best if error <= self.sync_slop else None

    def _lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target, source, Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            ).transform
        except TransformException:
            self.stats['tf_failures'] += 1
            return None

    def _active_camera_frame(self):
        if self.camera_info is not None and self.camera_info.header.frame_id:
            return self.camera_info.header.frame_id
        return self.camera_frame

    def _nearest_track(self, center):
        best, distance_best = None, self.track_gate
        if self.tracks is None:
            return None
        for tracked in self.tracks.objects:
            point = tracked.pose.pose.position
            distance = math.hypot(center[0] - point.x, center[1] - point.y)
            if distance < distance_best:
                best, distance_best = tracked, distance
        return best

    @staticmethod
    def _rectangle(detection):
        return (
            detection.center_x - 0.5 * detection.size_x,
            detection.center_y - 0.5 * detection.size_y,
            detection.center_x + 0.5 * detection.size_x,
            detection.center_y + 0.5 * detection.size_y,
        )

    def _expanded_roi_rectangle(self, detection):
        """Use visual detections as seeds, then search a vessel-sized ROI.

        The camera detector often sees only a small coloured target marker at
        long range. A LiDAR ROI built from that tiny box misses the target hull,
        producing separate Camera Only and LiDAR Only boxes. Expanding around
        the same visual centre keeps the match camera-led while collecting
        enough Mid-360 points for a real 3D box.
        """
        center_x = float(detection.center_x)
        center_y = float(detection.center_y)
        width = max(float(detection.size_x), self.minimum_roi_width)
        height = max(float(detection.size_y), self.minimum_roi_height)
        return (
            center_x - 0.5 * width,
            center_y - 0.5 * height,
            center_x + 0.5 * width,
            center_y + 0.5 * height,
        )

    def _camera_only(self, detection, map_from_camera, stamp):
        fx, fy, cx, cy = (
            self.camera_info.k[0], self.camera_info.k[4],
            self.camera_info.k[2], self.camera_info.k[5],
        )
        depth = min(self.max_depth, max(
            self.min_depth, fx * self.known_width / max(1.0, detection.size_x)
        ))
        camera_point = np.asarray((
            depth, -(detection.center_x - cx) * depth / max(1e-6, fx),
            -(detection.center_y - cy) * depth / max(1e-6, fy),
        ))
        center = apply_transform(camera_point.reshape(1, 3), map_from_camera)[0]
        tracked = TrackedObject()
        tracked.track_id = detection.detection_id
        tracked.uuid.uuid = stable_uuid(tracked.track_id)
        tracked.first_seen = deepcopy(self.first_seen.setdefault(
            tracked.track_id, deepcopy(stamp)
        ))
        tracked.last_update = deepcopy(stamp)
        tracked.source_mask = TrackedObject.SOURCE_CAMERA
        tracked.classification = TrackedObject.CLASS_VESSEL
        tracked.class_name = detection.class_name or 'vessel'
        tracked.class_confidence = detection.class_confidence
        tracked.sensor_source = 'camera'
        tracked.affiliation = detection.affiliation
        tracked.affiliation_confidence = detection.affiliation_confidence
        tracked.pose.pose.position.x, tracked.pose.pose.position.y, tracked.pose.pose.position.z = center
        tracked.pose.pose.orientation.w = 1.0
        tracked.dimensions.x, tracked.dimensions.y, tracked.dimensions.z = 7.0, self.known_width, 2.5
        tracked.confidence = 0.45 * detection.class_confidence
        tracked.bbox_point_count = 0
        tracked.association_score = 0.0
        for index in (0, 7, 14):
            tracked.pose.covariance[index] = 25.0
        return tracked

    def _fused_track(self, detection, box, stamp, association_score):
        nearest = self._nearest_track(box['center'])
        tracked = deepcopy(nearest) if nearest is not None else TrackedObject()
        if nearest is None:
            tracked.track_id = 'vision_%s' % detection.detection_id
            tracked.uuid.uuid = stable_uuid(tracked.track_id)
        tracked.first_seen = deepcopy(self.first_seen.setdefault(
            tracked.track_id, deepcopy(stamp)
        ))
        tracked.last_update = deepcopy(stamp)
        tracked.source_mask = (
            TrackedObject.SOURCE_CAMERA | TrackedObject.SOURCE_LIDAR
            | TrackedObject.SOURCE_FUSED
        )
        tracked.classification = TrackedObject.CLASS_VESSEL
        tracked.class_name = detection.class_name or 'vessel'
        tracked.class_confidence = detection.class_confidence
        tracked.sensor_source = 'camera+lidar'
        tracked.affiliation = detection.affiliation
        tracked.affiliation_confidence = detection.affiliation_confidence
        tracked.pose.pose.position.x, tracked.pose.pose.position.y, tracked.pose.pose.position.z = box['center']
        tracked.pose.pose.orientation.z = math.sin(0.5 * box['yaw'])
        tracked.pose.pose.orientation.w = math.cos(0.5 * box['yaw'])
        tracked.dimensions.x, tracked.dimensions.y, tracked.dimensions.z = box['dimensions']
        tracked.confidence = min(0.99, 0.55 + 0.25 * association_score +
                                 0.20 * detection.class_confidence)
        tracked.bbox_point_count = int(box['point_count'])
        tracked.association_score = float(association_score)
        for index, value in ((0, 0.20), (7, 0.20), (14, 0.35)):
            tracked.pose.covariance[index] = value
        return tracked

    @staticmethod
    def _bbox_marker(header, tracked, marker_id):
        marker = Marker()
        marker.header = header
        marker.ns = 'vision_guided/roi_bbox'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = tracked.pose.pose
        marker.scale = tracked.dimensions
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.25, 0.30
        marker.lifetime.nanosec = 300000000
        return marker

    @staticmethod
    def _cluster_marker(header, points, marker_id):
        marker = Marker()
        marker.header = header
        marker.ns = 'vision_guided/roi_cluster'
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 1.0
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        marker.lifetime.nanosec = 300000000
        return marker

    @staticmethod
    def _camera_projection_marker(
        header, detection, depth, rectangle, intrinsics, map_from_camera,
        marker_id
    ):
        left, top, right, bottom = rectangle
        pixels = np.asarray((
            (left, top), (right, top), (right, bottom), (left, bottom),
            (left, top),
        ), dtype=np.float64)
        points_camera = unproject_camera_pixels(
            pixels, np.full(len(pixels), float(depth)), intrinsics
        )
        points_map = apply_transform(points_camera, map_from_camera)
        marker = Marker()
        marker.header = header
        marker.ns = 'vision_guided/camera_projection'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 1.0
        marker.points = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in points_map
        ]
        marker.lifetime.nanosec = 300000000
        return marker

    def _on_cloud(self, message):
        started = time.perf_counter()
        now_wall = time.monotonic()
        self.stats['lidar_frames'] += 1
        output = TrackedObjectArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame
        detections = self._nearest_detections(message.header.stamp)
        if self.camera_info is None or detections is None:
            self.observations_pub.publish(output)
            self._maybe_publish_status(now_wall)
            return
        detection_stamp = detections.header.stamp
        map_from_cloud = self._lookup(
            self.output_frame, message.header.frame_id, message.header.stamp
        )
        active_camera_frame = self._active_camera_frame()
        map_from_camera = self._lookup(
            self.output_frame, active_camera_frame, detection_stamp
        )
        camera_from_map = self._lookup(
            active_camera_frame, self.output_frame, detection_stamp
        )
        camera_from_lidar = self._lookup(
            active_camera_frame, message.header.frame_id, detection_stamp
        )
        if any(value is None for value in (
            map_from_cloud, map_from_camera, camera_from_map,
            camera_from_lidar,
        )):
            self.observations_pub.publish(output)
            self._maybe_publish_status(now_wall)
            return
        try:
            points_cloud = cloud_xyz(message)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return
        points_map = apply_transform(points_cloud, map_from_cloud)
        # A camera ROI belongs to the image timestamp, while the point cloud
        # belongs to the LiDAR timestamp. Transform through map so a moving USV
        # is geometrically correct instead of assuming both frames are current.
        points_camera = apply_transform(points_map, camera_from_map)
        self.stats['cross_time_tf_frames'] += 1
        self.last['camera_from_lidar'] = {
            'translation': [
                float(camera_from_lidar.translation.x),
                float(camera_from_lidar.translation.y),
                float(camera_from_lidar.translation.z),
            ],
            'quaternion_xyzw': [
                float(camera_from_lidar.rotation.x),
                float(camera_from_lidar.rotation.y),
                float(camera_from_lidar.rotation.z),
                float(camera_from_lidar.rotation.w),
            ],
            'source_frame': message.header.frame_id,
            'target_frame': active_camera_frame,
        }
        valid = np.isfinite(points_cloud).all(axis=1)
        valid &= points_camera[:, 0] >= self.min_depth
        valid &= points_camera[:, 0] <= self.max_depth
        valid &= points_map[:, 2] >= self.water_z + self.water_margin
        projection_started = time.perf_counter()
        pixels, projected_valid = project_camera_points(
            points_camera, (
                self.camera_info.k[0], self.camera_info.k[4],
                self.camera_info.k[2], self.camera_info.k[5],
            )
        )
        valid &= projected_valid
        self.last['projection_ms'] = (time.perf_counter() - projection_started) * 1000.0
        available = valid.copy()
        accepted_points = []
        cluster_markers = MarkerArray()
        bbox_markers = MarkerArray()
        camera_projection_markers = MarkerArray()
        inside_ratios = []
        center_errors = []
        projection_depths = []
        roi_started = time.perf_counter()
        for detection_index, detection in enumerate(detections.detections):
            roi_rectangle = self._expanded_roi_rectangle(detection)
            indices = roi_indices(
                pixels, roi_rectangle, self.expand, available
            )
            if len(indices) > self.max_points:
                selection = np.linspace(0, len(indices) - 1, self.max_points, dtype=int)
                indices = indices[selection]
            if len(indices) < self.min_points:
                output.objects.append(self._camera_only(
                    detection, map_from_camera, message.header.stamp
                ))
                self.stats['camera_only_count'] += 1
                self.stats['roi_empty_count'] += 1
                self.stats['insufficient_roi_points'] += 1
                continue
            keep = depth_filter(
                points_camera[indices], self.depth_low, self.depth_high,
                self.depth_threshold,
            )
            indices = indices[keep]
            if len(indices) < self.min_points:
                output.objects.append(self._camera_only(
                    detection, map_from_camera, message.header.stamp
                ))
                self.stats['camera_only_count'] += 1
                self.stats['roi_empty_count'] += 1
                self.stats['depth_filter_rejections'] += 1
                continue
            labels = dbscan(
                points_map[indices], self.cluster_epsilon,
                self.cluster_min_samples,
            )
            selected, score = select_cluster(points_map[indices], labels)
            if selected is None or len(selected) < self.min_points:
                output.objects.append(self._camera_only(
                    detection, map_from_camera, message.header.stamp
                ))
                self.stats['camera_only_count'] += 1
                self.stats['rejected_3d_bboxes'] += 1
                self.stats['cluster_selection_rejections'] += 1
                continue
            selected_indices = indices[selected]
            box = robust_oriented_bbox(
                points_map[selected_indices], self.depth_low, self.depth_high
            )
            # Keep the final fused box on the measured target surface. The old
            # occlusion completion shifted its centre away from both the ROI
            # cloud and the camera projection, which looked like bad extrinsic
            # calibration in Qt. Full-shape completion belongs in a later shape
            # estimator, not in this geometric calibration path.
            if not bbox_is_plausible(box, self.limits):
                self.last['rejected_bbox_dimensions'] = [
                    float(value) for value in box['dimensions']
                ]
                output.objects.append(self._camera_only(
                    detection, map_from_camera, message.header.stamp
                ))
                self.stats['camera_only_count'] += 1
                self.stats['rejected_3d_bboxes'] += 1
                self.stats['bbox_plausibility_rejections'] += 1
                continue
            box, accepted = self.smoother.update(detection.detection_id, box)
            jump_rejected = not accepted
            if not accepted:
                self.stats['rejected_3d_bboxes'] += 1
                self.stats['bbox_jump_rejections'] += 1
            self.confirmation_counts[detection.detection_id] = (
                self.confirmation_counts.get(detection.detection_id, 0) + 1
            )
            if self.confirmation_counts[detection.detection_id] < self.confirmed_frames:
                self.stats['unconfirmed_frames'] += 1
                continue
            association_score = min(1.0, math.log1p(len(selected_indices)) / 5.0)
            if jump_rejected:
                association_score *= 0.5
            tracked = self._fused_track(
                detection, box, message.header.stamp, association_score
            )
            output.objects.append(tracked)
            accepted_points.append(points_map[selected_indices])
            available[selected_indices] = False
            cluster_markers.markers.append(self._cluster_marker(
                output.header, points_map[selected_indices], detection_index
            ))
            bbox_markers.markers.append(self._bbox_marker(
                output.header, tracked, detection_index
            ))
            selected_pixels = pixels[selected_indices]
            rectangle = roi_rectangle
            inside = (
                (selected_pixels[:, 0] >= rectangle[0])
                & (selected_pixels[:, 0] <= rectangle[2])
                & (selected_pixels[:, 1] >= rectangle[1])
                & (selected_pixels[:, 1] <= rectangle[3])
            )
            inside_ratios.append(float(np.mean(inside)))
            pixel_center = np.mean(selected_pixels, axis=0)
            detection_center = np.asarray((
                detection.center_x, detection.center_y
            ))
            center_errors.append(float(np.linalg.norm(
                pixel_center - detection_center
            )))
            projection_depth = float(np.median(
                points_camera[selected_indices, 0]
            ))
            projection_depths.append(projection_depth)
            camera_projection_markers.markers.append(
                self._camera_projection_marker(
                    output.header, detection, projection_depth,
                    roi_rectangle,
                    (
                        self.camera_info.k[0], self.camera_info.k[4],
                        self.camera_info.k[2], self.camera_info.k[5],
                    ),
                    map_from_camera, detection_index,
                )
            )
            self.stats['roi_clusters'] += 1
            self.stats['valid_3d_bboxes'] += 1
            self.stats['camera_lidar_matches'] += 1
        roi_points = np.vstack(accepted_points) if accepted_points else np.empty((0, 3))
        self.roi_cloud_pub.publish(point_cloud2.create_cloud_xyz32(
            output.header, roi_points.astype(np.float32).tolist()
        ))
        self.cluster_pub.publish(cluster_markers)
        self.bbox_pub.publish(bbox_markers)
        self.camera_projection_pub.publish(camera_projection_markers)
        self.observations_pub.publish(output)
        self.stats['roi_extraction_frames'] += 1
        self.stats['roi_points_total'] += len(roi_points)
        self.last['roi_points'] = len(roi_points)
        self.last['roi_inside_ratio'] = (
            float(np.mean(inside_ratios)) if inside_ratios else None
        )
        self.last['reprojection_center_error_px'] = (
            float(np.mean(center_errors)) if center_errors else None
        )
        self.last['camera_projection_depth_m'] = (
            float(np.mean(projection_depths)) if projection_depths else None
        )
        self.last['roi_ms'] = (time.perf_counter() - roi_started) * 1000.0
        self.last['total_ms'] = (time.perf_counter() - started) * 1000.0
        self._maybe_publish_status(now_wall)

    def _maybe_publish_status(self, now_wall):
        if now_wall - self.last_status_wall < 1.0:
            return
        self.last_status_wall = now_wall
        self._publish_status()

    def _publish_status(self):
        frames = max(1, self.stats['roi_extraction_frames'])
        payload = dict(self.stats)
        payload.update({
            'mode': 'shadow_vision_guided_perception',
            'shadow_mode': self.shadow_mode,
            'roi_average_points': self.stats['roi_points_total'] / frames,
            'average_sync_error_ms': self.last['sync_ms'],
            'average_projection_ms': self.last['projection_ms'],
            'average_roi_processing_ms': self.last['roi_ms'],
            'average_total_processing_ms': self.last['total_ms'],
            'roi_point_count': self.last['roi_points'],
            'roi_inside_ratio': self.last['roi_inside_ratio'],
            'reprojection_center_error_px': self.last[
                'reprojection_center_error_px'
            ],
            'camera_projection_depth_m': self.last[
                'camera_projection_depth_m'
            ],
            'camera_frame': self._active_camera_frame(),
            'configured_camera_frame': self.camera_frame,
            'camera_info_frame': (
                self.camera_info.header.frame_id if self.camera_info else ''
            ),
            'camera_intrinsics': (
                {
                    'fx': float(self.camera_info.k[0]),
                    'fy': float(self.camera_info.k[4]),
                    'cx': float(self.camera_info.k[2]),
                    'cy': float(self.camera_info.k[5]),
                    'width': int(self.camera_info.width),
                    'height': int(self.camera_info.height),
                } if self.camera_info else None
            ),
            'camera_from_lidar': self.last['camera_from_lidar'],
            'tf_query_mode': 'historical_cross_time_via_map',
            'last_rejected_bbox_dimensions': self.last.get(
                'rejected_bbox_dimensions'
            ),
            'control_connected': False,
            'perception_source': 'ground_truth',
        })
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        if not rclpy.ok():
            return
        try:
            self.status_pub.publish(message)
        except Exception as exc:  # Context can close between the check and publish.
            if rclpy.ok():
                self.get_logger().warning(
                    'Unable to publish ROI status: %s' % exc,
                    throttle_duration_sec=5.0,
                )


def main(args=None):
    rclpy.init(args=args)
    node = VisionGuidedLidarRoiNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
