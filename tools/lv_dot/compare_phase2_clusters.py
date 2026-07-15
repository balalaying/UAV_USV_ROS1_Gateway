#!/usr/bin/env python3
"""Compare native ROS 2 LiDAR clusters with the recorded ROS 1 LV-DOT baseline."""

import argparse
from bisect import bisect_left, bisect_right
import json
import math
from pathlib import Path
import re
import statistics

import numpy as np
import rosbag2_py
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray


INPUT_TOPIC = '/perception/usv_01/points_filtered'
POSE_TOPIC = '/perception/lv_dot/usv_01/pose'
ROS1_BOX_TOPIC = '/lv_dot/diagnostics/lidar_bboxes'
ROS2_BOX_TOPIC = '/perception/lv_dot_ros2/diagnostics/lidar_bboxes'
ROS2_DIAGNOSTIC_TOPIC = '/perception/lv_dot_ros2/diagnostics'


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('ros1_bag', type=Path)
    parser.add_argument('ros2_bag', type=Path)
    parser.add_argument('--resource-log', type=Path)
    parser.add_argument('--maximum-time-gap', type=float, default=0.15)
    parser.add_argument('--json-output', type=Path)
    return parser.parse_args()


def mean(values):
    return statistics.fmean(values) if values else None


def percentile(values, percentage):
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * percentage
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def nearest_index(times, requested):
    index = bisect_left(times, requested)
    candidates = [value for value in (index - 1, index)
                  if 0 <= value < len(times)]
    if not candidates:
        return None
    return min(candidates, key=lambda value: abs(times[value] - requested))


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def point_cloud_xyz(message):
    fields = {field.name: field for field in message.fields}
    required = [fields.get(name) for name in ('x', 'y', 'z')]
    if any(field is None for field in required):
        raise ValueError('PointCloud2 has no complete XYZ layout')
    if any(field.datatype != PointField.FLOAT32 or field.count != 1
           for field in required):
        raise ValueError('Comparison expects scalar FLOAT32 XYZ fields')
    count = int(message.width * message.height)
    endian = '>' if message.is_bigendian else '<'
    coordinates = [
        np.ndarray(
            shape=(count,), dtype=endian + 'f4', buffer=message.data,
            offset=field.offset, strides=(message.point_step,)
        )
        for field in required
    ]
    return np.column_stack(coordinates).astype(np.float64, copy=False)


def quaternion_rotation(orientation):
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


def transformed_proxy_points(cloud, pose):
    points = point_cloud_xyz(cloud)
    finite = np.isfinite(points).all(axis=1)
    local = points[finite]
    local = local[
        (local[:, 0] >= -10.0) & (local[:, 0] <= 10.0)
        & (local[:, 1] >= -10.0) & (local[:, 1] <= 10.0)
    ]
    rotation = quaternion_rotation(pose.pose.orientation)
    base = np.array([
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
    ], dtype=np.float64)
    sensor_offset = rotation @ np.array([0.9075, 0.0, 1.5625])
    transformed = local @ rotation.T + base + sensor_offset
    return transformed[
        (transformed[:, 2] >= 0.22) & (transformed[:, 2] <= 6.0)
    ]


def metadata(text):
    result = {}
    for item in text.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        try:
            result[key] = float(value)
        except ValueError:
            result[key] = value
    return result


def boxes(message):
    result = []
    for marker in message.markers:
        if marker.action not in (Marker.ADD, Marker.MODIFY):
            continue
        if marker.type == Marker.LINE_LIST and marker.points:
            local = np.array(
                [[point.x, point.y, point.z] for point in marker.points],
                dtype=np.float64,
            )
            minimum = local.min(axis=0)
            maximum = local.max(axis=0)
            local_center = 0.5 * (minimum + maximum)
            dimensions = maximum - minimum
        else:
            local_center = np.zeros(3)
            dimensions = np.array(
                [marker.scale.x, marker.scale.y, marker.scale.z],
                dtype=np.float64,
            )
        center = np.array([
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
        ]) + local_center
        result.append({
            'center': center,
            'dimensions': dimensions,
            'metadata': metadata(marker.text),
        })
    return result


def read_ros1(path):
    reader = open_reader(path)
    clouds = []
    poses = []
    markers = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        record_time = timestamp_ns * 1e-9
        if topic == INPUT_TOPIC:
            message = deserialize_message(serialized, PointCloud2)
            clouds.append((record_time, stamp_seconds(message.header.stamp), message))
        elif topic == POSE_TOPIC:
            message = deserialize_message(serialized, PoseStamped)
            poses.append((stamp_seconds(message.header.stamp), message))
        elif topic == ROS1_BOX_TOPIC:
            markers.append((record_time, boxes(deserialize_message(serialized, MarkerArray))))
    return clouds, poses, markers


def read_ros2(path):
    reader = open_reader(path)
    clouds = []
    markers = []
    diagnostics = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        record_time = timestamp_ns * 1e-9
        if topic == INPUT_TOPIC:
            message = deserialize_message(serialized, PointCloud2)
            clouds.append((record_time, stamp_seconds(message.header.stamp)))
        elif topic == ROS2_BOX_TOPIC:
            message = deserialize_message(serialized, MarkerArray)
            logical_stamp = 0.0
            for marker in message.markers:
                if marker.header.stamp.sec or marker.header.stamp.nanosec:
                    logical_stamp = stamp_seconds(marker.header.stamp)
                    break
            markers.append((record_time, logical_stamp, boxes(message)))
        elif topic == ROS2_DIAGNOSTIC_TOPIC:
            message = deserialize_message(serialized, DiagnosticArray)
            for status in message.status:
                diagnostics.append({item.key: item.value for item in status.values})
    return clouds, markers, diagnostics


def assign_ros1_frames(clouds, marker_samples, maximum_gap):
    record_times = [sample[0] for sample in clouds]
    assigned = []
    for record_time, frame_boxes in marker_samples:
        # ROS1 markers carry stamp zero. Associate each timer result only with
        # the latest input that had already arrived, never a future cloud.
        index = bisect_right(record_times, record_time) - 1
        if index < 0 or record_time - record_times[index] > maximum_gap:
            continue
        cloud_record, logical_stamp, cloud = clouds[index]
        assigned.append({
            'stamp': logical_stamp,
            'record_time': record_time,
            'input_record_time': cloud_record,
            'cloud': cloud,
            'boxes': frame_boxes,
        })
    return assigned


def assign_ros2_frames(clouds, marker_samples, maximum_gap):
    cloud_stamps = [sample[1] for sample in clouds]
    assigned = []
    for record_time, logical_stamp, frame_boxes in marker_samples:
        index = nearest_index(cloud_stamps, logical_stamp)
        if index is None or abs(cloud_stamps[index] - logical_stamp) > maximum_gap:
            continue
        cloud_record, _ = clouds[index]
        assigned.append({
            'stamp': logical_stamp,
            'record_time': record_time,
            'input_record_time': cloud_record,
            'boxes': frame_boxes,
        })
    return assigned


def greedy_matches(reference, candidate):
    available = set(range(len(candidate)))
    result = []
    for reference_index, reference_box in enumerate(reference):
        if not available:
            break
        candidate_index = min(
            available,
            key=lambda index: np.linalg.norm(
                reference_box['center'][:2] - candidate[index]['center'][:2]
            ),
        )
        available.remove(candidate_index)
        result.append((reference_index, candidate_index))
    return result


def resource_summary(path):
    if path is None or not path.is_file():
        return None
    samples = [json.loads(line) for line in path.read_text().splitlines()
               if line.strip()]
    active = [sample for sample in samples if sample.get('process_count', 0)]
    return {
        'samples': len(active),
        'cpu_percent_mean': mean([
            sample['detector_cpu_percent'] for sample in active
        ]),
        'cpu_percent_max': max(
            [sample['detector_cpu_percent'] for sample in active],
            default=None,
        ),
        'rss_mb_mean': mean([sample['detector_rss_mb'] for sample in active]),
        'rss_mb_max': max(
            [sample['detector_rss_mb'] for sample in active], default=None
        ),
    }


def main():
    args = parse_args()
    ros1_clouds, poses, ros1_marker_samples = read_ros1(args.ros1_bag)
    ros2_clouds, ros2_marker_samples, diagnostics = read_ros2(args.ros2_bag)
    ros1_frames = assign_ros1_frames(
        ros1_clouds, ros1_marker_samples, args.maximum_time_gap
    )
    ros2_frames = assign_ros2_frames(
        ros2_clouds, ros2_marker_samples, args.maximum_time_gap
    )
    ros2_stamps = [frame['stamp'] for frame in ros2_frames]
    pose_stamps = [sample[0] for sample in poses]

    cluster_count_differences = []
    exact_cluster_counts = 0
    center_xy_errors = []
    center_display_3d_errors = []
    size_xy_errors = []
    size_display_3d_errors = []
    point_count_differences = []
    point_count_relative_differences = []
    ros1_lag_ms = []
    ros2_lag_ms = []
    preprocessing_ms = []
    clustering_ms = []
    ros1_cluster_counts = []
    ros2_cluster_counts = []
    compared_frames = 0
    matched_clusters = 0

    for ros1_frame in ros1_frames:
        ros2_index = nearest_index(ros2_stamps, ros1_frame['stamp'])
        if ros2_index is None:
            continue
        ros2_frame = ros2_frames[ros2_index]
        if abs(ros2_frame['stamp'] - ros1_frame['stamp']) > args.maximum_time_gap:
            continue
        compared_frames += 1
        reference = ros1_frame['boxes']
        candidate = ros2_frame['boxes']
        difference = abs(len(reference) - len(candidate))
        ros1_cluster_counts.append(len(reference))
        ros2_cluster_counts.append(len(candidate))
        cluster_count_differences.append(difference)
        exact_cluster_counts += difference == 0
        ros1_lag_ms.append(max(
            0.0, (ros1_frame['record_time'] - ros1_frame['input_record_time']) * 1000.0
        ))
        ros2_lag_ms.append(max(
            0.0, (ros2_frame['record_time'] - ros2_frame['input_record_time']) * 1000.0
        ))

        proxy_points = None
        pose_index = nearest_index(pose_stamps, ros1_frame['stamp'])
        if pose_index is not None:
            proxy_points = transformed_proxy_points(
                ros1_frame['cloud'], poses[pose_index][1]
            )
        for reference_index, candidate_index in greedy_matches(reference, candidate):
            matched_clusters += 1
            first = reference[reference_index]
            second = candidate[candidate_index]
            center_xy_errors.append(float(np.linalg.norm(
                first['center'][:2] - second['center'][:2]
            )))
            center_display_3d_errors.append(float(np.linalg.norm(
                first['center'] - second['center']
            )))
            size_xy_errors.append(float(np.linalg.norm(
                first['dimensions'][:2] - second['dimensions'][:2]
            )))
            size_display_3d_errors.append(float(np.linalg.norm(
                first['dimensions'] - second['dimensions']
            )))
            item_metadata = second['metadata']
            if 'preprocess_ms' in item_metadata:
                preprocessing_ms.append(float(item_metadata['preprocess_ms']))
            if 'cluster_ms' in item_metadata:
                clustering_ms.append(float(item_metadata['cluster_ms']))
            if proxy_points is not None and 'points' in item_metadata:
                lower = first['center'] - 0.5 * first['dimensions'] - 1e-6
                upper = first['center'] + 0.5 * first['dimensions'] + 1e-6
                proxy_count = int(np.count_nonzero(np.all(
                    (proxy_points >= lower) & (proxy_points <= upper), axis=1
                )))
                actual_count = int(item_metadata['points'])
                point_count_differences.append(abs(actual_count - proxy_count))
                if proxy_count > 0:
                    point_count_relative_differences.append(
                        abs(actual_count - proxy_count) / proxy_count
                    )

    diagnostic_last = diagnostics[-1] if diagnostics else {}
    result = {
        'ros1_bag': str(args.ros1_bag),
        'ros2_bag': str(args.ros2_bag),
        'ros1_marker_frames': len(ros1_frames),
        'ros2_marker_frames': len(ros2_frames),
        'compared_frames': compared_frames,
        'matched_clusters': matched_clusters,
        'unmatched_ros1_clusters': sum(ros1_cluster_counts) - matched_clusters,
        'unmatched_ros2_clusters': sum(ros2_cluster_counts) - matched_clusters,
        'ros1_average_cluster_count': mean(ros1_cluster_counts),
        'ros2_average_cluster_count': mean(ros2_cluster_counts),
        'cluster_count_mean_absolute_difference': mean(cluster_count_differences),
        'cluster_count_exact_frame_rate': (
            exact_cluster_counts / compared_frames if compared_frames else 0.0
        ),
        'center_xy_error_mean_m': mean(center_xy_errors),
        'center_xy_error_p95_m': percentile(center_xy_errors, 0.95),
        'center_xy_error_max_m': max(center_xy_errors, default=None),
        'center_display_3d_error_mean_m': mean(center_display_3d_errors),
        'size_xy_error_mean_m': mean(size_xy_errors),
        'size_xy_error_p95_m': percentile(size_xy_errors, 0.95),
        'size_xy_error_max_m': max(size_xy_errors, default=None),
        'size_display_3d_error_mean_m': mean(size_display_3d_errors),
        'ros1_point_count_proxy_note': (
            'ROS1 MarkerArray has no cluster point count; the baseline value is '
            'a deterministic upper-bound count of transformed input points inside '
            'the recorded box before Gaussian sampling.'
        ),
        'point_count_proxy_absolute_difference_mean': mean(point_count_differences),
        'point_count_proxy_relative_difference_mean': mean(
            point_count_relative_differences
        ),
        'ros1_input_to_marker_lag_mean_ms': mean(ros1_lag_ms),
        'ros1_input_to_marker_lag_max_ms': max(ros1_lag_ms, default=None),
        'ros2_input_to_marker_lag_mean_ms': mean(ros2_lag_ms),
        'ros2_input_to_marker_lag_max_ms': max(ros2_lag_ms, default=None),
        'ros2_preprocessing_time_mean_ms': mean(preprocessing_ms),
        'ros2_clustering_time_mean_ms': mean(clustering_ms),
        'ros2_clustering_time_max_ms': max(clustering_ms, default=None),
        'ros2_final_diagnostics': diagnostic_last,
        'ros2_resources': resource_summary(args.resource_log),
    }
    encoded = json.dumps(result, indent=2, sort_keys=True)
    print(encoded)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(encoded + '\n')


if __name__ == '__main__':
    main()
