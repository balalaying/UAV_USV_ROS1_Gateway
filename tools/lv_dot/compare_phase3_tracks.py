#!/usr/bin/env python3
"""Compare ROS 1 LV-DOT tracking with native ROS 2 Phase 3 tracks."""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import re
import statistics

import numpy as np
import rosbag2_py
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from uav_usv_interfaces.msg import TrackedObjectArray
from visualization_msgs.msg import Marker, MarkerArray


TRUTH_TOPIC = '/perception/ground_truth/tracks'
ROS1_TRACK_TOPIC = '/lv_dot/onboard_detector/velocity_visualizaton'
ROS2_TRACK_TOPIC = '/perception/lv_dot_ros2/tracks'
ROS2_OBSERVATION_TOPIC = '/perception/lv_dot_ros2/observations'
ROS2_DIAGNOSTIC_TOPIC = '/perception/lv_dot_ros2/diagnostics'
INPUT_TOPIC = '/perception/usv_01/points_filtered'
VELOCITY_PATTERN = re.compile(
    r'Vx=([-+0-9.eE]+),\s*Vy=([-+0-9.eE]+)'
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('ros1_bag', type=Path)
    parser.add_argument('ros2_bag', type=Path)
    parser.add_argument('--resource-log', type=Path)
    parser.add_argument('--maximum-time-gap', type=float, default=0.15)
    parser.add_argument('--target-gate', type=float, default=3.0)
    parser.add_argument('--warmup', type=float, default=5.0)
    parser.add_argument('--identity-hysteresis', type=float, default=0.5)
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
    candidates = [item for item in (index - 1, index)
                  if 0 <= item < len(times)]
    if not candidates:
        return None
    return min(candidates, key=lambda item: abs(times[item] - requested))


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def truth_objects(message):
    return [item for item in message.objects
            if item.track_id in ('target_vessel', 'enemy_target')]


def read_ros1(path):
    reader = open_reader(path)
    truth = []
    tracks = []
    inputs = []
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic == TRUTH_TOPIC:
            message = deserialize_message(serialized, TrackedObjectArray)
            candidates = truth_objects(message)
            if not candidates:
                continue
            item = candidates[0]
            truth.append({
                'stamp': stamp_seconds(message.header.stamp),
                'record_time': record_ns * 1e-9,
                'id': item.track_id,
                'position': np.array([
                    item.pose.pose.position.x,
                    item.pose.pose.position.y,
                    item.pose.pose.position.z,
                ]),
                'velocity': np.array([
                    item.twist.twist.linear.x,
                    item.twist.twist.linear.y,
                    item.twist.twist.linear.z,
                ]),
            })
        elif topic == INPUT_TOPIC:
            message = deserialize_message(serialized, PointCloud2)
            inputs.append((stamp_seconds(message.header.stamp),
                           record_ns * 1e-9))
        elif topic == ROS1_TRACK_TOPIC:
            message = deserialize_message(serialized, MarkerArray)
            frame_tracks = []
            logical_stamp = 0.0
            for marker in message.markers:
                if marker.action not in (Marker.ADD, Marker.MODIFY):
                    continue
                match = VELOCITY_PATTERN.search(marker.text)
                if match is None:
                    continue
                if logical_stamp == 0.0:
                    logical_stamp = stamp_seconds(marker.header.stamp)
                frame_tracks.append({
                    # ROS1 has no persistent track ID. The marker ID is only a
                    # frame-local baseline identity proxy.
                    'id': f'ros1_marker_{marker.id}',
                    'position': np.array([
                        marker.pose.position.x,
                        marker.pose.position.y,
                        marker.pose.position.z,
                    ]),
                    'velocity': np.array([
                        float(match.group(1)), float(match.group(2)), 0.0,
                    ]),
                })
            if frame_tracks and logical_stamp > 0.0:
                tracks.append({
                    'stamp': logical_stamp,
                    'record_time': record_ns * 1e-9,
                    'tracks': frame_tracks,
                })
    attach_input_record_times(tracks, inputs)
    return truth, tracks


def read_ros2(path):
    reader = open_reader(path)
    tracks = []
    diagnostics = []
    observation_messages = 0
    nonempty_observation_messages = 0
    inputs = []
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic == ROS2_TRACK_TOPIC:
            message = deserialize_message(serialized, TrackedObjectArray)
            tracks.append({
                'stamp': stamp_seconds(message.header.stamp),
                'record_time': record_ns * 1e-9,
                'tracks': [{
                    'id': item.track_id,
                    'position': np.array([
                        item.pose.pose.position.x,
                        item.pose.pose.position.y,
                        item.pose.pose.position.z,
                    ]),
                    'velocity': np.array([
                        item.twist.twist.linear.x,
                        item.twist.twist.linear.y,
                        item.twist.twist.linear.z,
                    ]),
                } for item in message.objects],
            })
        elif topic == INPUT_TOPIC:
            message = deserialize_message(serialized, PointCloud2)
            inputs.append((stamp_seconds(message.header.stamp),
                           record_ns * 1e-9))
        elif topic == ROS2_OBSERVATION_TOPIC:
            message = deserialize_message(serialized, TrackedObjectArray)
            observation_messages += 1
            nonempty_observation_messages += bool(message.objects)
        elif topic == ROS2_DIAGNOSTIC_TOPIC:
            message = deserialize_message(serialized, DiagnosticArray)
            for status in message.status:
                diagnostics.append({item.key: item.value
                                    for item in status.values})
    attach_input_record_times(tracks, inputs)
    return tracks, diagnostics, observation_messages, nonempty_observation_messages


def attach_input_record_times(frames, inputs):
    input_stamps = [item[0] for item in inputs]
    for frame in frames:
        index = nearest_index(input_stamps, frame['stamp'])
        frame['input_record_time'] = (
            inputs[index][1] if index is not None else None
        )


def target_metrics(truth, frames, maximum_gap, gate, warmup,
                   identity_hysteresis):
    frame_times = [frame['stamp'] for frame in frames]
    if not truth or not frame_times:
        return {
            'evaluation_frames': 0,
            'valid_frames': 0,
            'detection_rate': 0.0,
        }, []
    output_frequency = (
        (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
        if len(frame_times) > 1 and frame_times[-1] > frame_times[0]
        else 0.0
    )
    start = truth[0]['stamp'] + warmup
    selected = []
    losses = []
    position_errors = []
    velocity_errors = []
    track_counts = []
    selected_ids = []
    nearest_ids = []
    record_lags = []
    evaluation_frames = 0
    previous_id = None
    for item in truth:
        if item['stamp'] < start:
            continue
        evaluation_frames += 1
        index = nearest_index(frame_times, item['stamp'])
        if index is None or abs(frame_times[index] - item['stamp']) > maximum_gap:
            losses.append(item['stamp'])
            selected.append(None)
            continue
        frame = frames[index]
        track_counts.append(len(frame['tracks']))
        if not frame['tracks']:
            losses.append(item['stamp'])
            selected.append(None)
            continue
        candidates = [(track, float(np.linalg.norm(
            track['position'][:2] - item['position'][:2]
        ))) for track in frame['tracks']]
        nearest, nearest_error = min(candidates, key=lambda pair: pair[1])
        if nearest_error > gate:
            losses.append(item['stamp'])
            selected.append(None)
            continue
        nearest_ids.append(nearest['id'])
        candidate = nearest
        position_error = nearest_error
        if previous_id is not None:
            previous_candidates = [pair for pair in candidates
                                   if pair[0]['id'] == previous_id]
            if previous_candidates:
                previous, previous_error = previous_candidates[0]
                if (previous_error <= gate and
                        previous_error <= nearest_error + identity_hysteresis):
                    candidate = previous
                    position_error = previous_error
        selected.append(candidate)
        selected_ids.append(candidate['id'])
        previous_id = candidate['id']
        position_errors.append(position_error)
        velocity_errors.append(float(np.linalg.norm(
            candidate['velocity'][:2] - item['velocity'][:2]
        )))
        if frame['input_record_time'] is not None:
            record_lags.append(max(
                0.0,
                (frame['record_time'] - frame['input_record_time']) * 1000.0,
            ))

    id_switches = 0
    previous_id = None
    for candidate in selected:
        if candidate is None:
            continue
        if previous_id is not None and candidate['id'] != previous_id:
            id_switches += 1
        previous_id = candidate['id']
    nearest_id_switches = sum(
        first != second for first, second in zip(nearest_ids, nearest_ids[1:])
    )

    longest_loss = 0.0
    current_start = None
    previous_time = None
    for truth_item, candidate in zip(
            [item for item in truth if item['stamp'] >= start], selected):
        if candidate is None:
            if current_start is None:
                current_start = truth_item['stamp']
            previous_time = truth_item['stamp']
        elif current_start is not None:
            longest_loss = max(longest_loss, previous_time - current_start)
            current_start = None
    if current_start is not None and previous_time is not None:
        longest_loss = max(longest_loss, previous_time - current_start)

    valid_frames = len(position_errors)
    return {
        'evaluation_frames': evaluation_frames,
        'valid_frames': valid_frames,
        'detection_rate': (
            valid_frames / evaluation_frames if evaluation_frames else 0.0
        ),
        'output_frequency_hz': output_frequency,
        'average_track_count': mean(track_counts),
        'position_error_mean_m': mean(position_errors),
        'position_error_p95_m': percentile(position_errors, 0.95),
        'position_error_max_m': max(position_errors, default=None),
        'velocity_error_mean_mps': mean(velocity_errors),
        'velocity_error_p95_mps': percentile(velocity_errors, 0.95),
        'velocity_error_max_mps': max(velocity_errors, default=None),
        'longest_loss_seconds': longest_loss,
        'id_switches': id_switches,
        'nearest_selection_id_switches': nearest_id_switches,
        'unique_target_ids': len(set(selected_ids)),
        'output_lag_mean_ms': mean(record_lags),
        'output_lag_max_ms': max(record_lags, default=None),
    }, selected


def direct_comparison(truth, ros1_selected, ros2_selected):
    position_differences = []
    velocity_differences = []
    compared = 0
    truth_after_warmup_count = min(len(ros1_selected), len(ros2_selected))
    for index in range(truth_after_warmup_count):
        first = ros1_selected[index]
        second = ros2_selected[index]
        if first is None or second is None:
            continue
        compared += 1
        position_differences.append(float(np.linalg.norm(
            first['position'][:2] - second['position'][:2]
        )))
        velocity_differences.append(float(np.linalg.norm(
            first['velocity'][:2] - second['velocity'][:2]
        )))
    return {
        'compared_target_frames': compared,
        'position_delta_mean_m': mean(position_differences),
        'position_delta_p95_m': percentile(position_differences, 0.95),
        'velocity_delta_mean_mps': mean(velocity_differences),
        'velocity_delta_p95_mps': percentile(velocity_differences, 0.95),
    }


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
    truth, ros1_frames = read_ros1(args.ros1_bag)
    (ros2_frames, diagnostics, observation_messages,
     nonempty_observation_messages) = read_ros2(args.ros2_bag)
    ros1_metrics, ros1_selected = target_metrics(
        truth, ros1_frames, args.maximum_time_gap, args.target_gate,
        args.warmup, args.identity_hysteresis,
    )
    ros2_metrics, ros2_selected = target_metrics(
        truth, ros2_frames, args.maximum_time_gap, args.target_gate,
        args.warmup, args.identity_hysteresis,
    )
    result = {
        'ros1_bag': str(args.ros1_bag),
        'ros2_bag': str(args.ros2_bag),
        'target_gate_m': args.target_gate,
        'warmup_seconds': args.warmup,
        'identity_hysteresis_m': args.identity_hysteresis,
        'truth_frames': len(truth),
        'ros1_track_frames': len(ros1_frames),
        'ros2_track_frames': len(ros2_frames),
        'ros1_baseline_identity_note': (
            'ROS1 velocity markers have no persistent track ID; marker.id is '
            'used only as a frame-local identity proxy.'
        ),
        'ros1': ros1_metrics,
        'ros2': ros2_metrics,
        'ros1_ros2': direct_comparison(
            truth, ros1_selected, ros2_selected
        ),
        'ros2_observation_messages': observation_messages,
        'ros2_nonempty_observation_messages': nonempty_observation_messages,
        'source_mux_safety': (
            nonempty_observation_messages == 0
        ),
        'ros2_final_diagnostics': diagnostics[-1] if diagnostics else {},
        'ros2_resources': resource_summary(args.resource_log),
    }
    encoded = json.dumps(result, indent=2, sort_keys=True)
    print(encoded)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(encoded + '\n')


if __name__ == '__main__':
    main()
