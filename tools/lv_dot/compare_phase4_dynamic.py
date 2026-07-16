#!/usr/bin/env python3
"""Compare ROS 1 and native ROS 2 LV-DOT dynamic classifications."""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import statistics

import numpy as np
import rosbag2_py
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message
from uav_usv_interfaces.msg import TrackedObjectArray


TRUTH_TOPIC = '/perception/ground_truth/tracks'
ROS1_DYNAMIC_TOPIC = '/perception/lv_dot/observations'
ROS2_DYNAMIC_TOPIC = '/perception/lv_dot_ros2/dynamic_tracks'
ROS2_OBSERVATION_TOPIC = '/perception/lv_dot_ros2/observations'
ROS2_DIAGNOSTIC_TOPIC = '/perception/lv_dot_ros2/diagnostics'


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('ros1_bag', type=Path)
    parser.add_argument('ros2_bag', type=Path)
    parser.add_argument('--resource-log', type=Path)
    parser.add_argument('--static-scene', action='store_true')
    parser.add_argument('--maximum-time-gap', type=float, default=0.15)
    parser.add_argument('--target-gate', type=float, default=3.0)
    parser.add_argument('--warmup', type=float, default=5.0)
    parser.add_argument('--identity-hysteresis', type=float, default=0.5)
    parser.add_argument('--json-output', type=Path)
    return parser.parse_args()


def mean(values):
    return statistics.fmean(values) if values else None


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def message_tracks(message):
    return [{
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
    } for item in message.objects]


def truth_object(message):
    candidates = [item for item in message.objects
                  if item.track_id in ('target_vessel', 'enemy_target')]
    return candidates[0] if candidates else None


def read_bag(path, dynamic_topic, read_diagnostics=False):
    reader = open_reader(path)
    truth = []
    frames = []
    diagnostics = []
    observation_messages = 0
    nonempty_observation_messages = 0
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic == TRUTH_TOPIC:
            message = deserialize_message(serialized, TrackedObjectArray)
            item = truth_object(message)
            if item is not None:
                truth.append({
                    'stamp': stamp_seconds(message.header.stamp),
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
        elif topic == dynamic_topic:
            message = deserialize_message(serialized, TrackedObjectArray)
            frames.append({
                'stamp': stamp_seconds(message.header.stamp),
                'record_time': record_ns * 1e-9,
                'tracks': message_tracks(message),
            })
        elif read_diagnostics and topic == ROS2_OBSERVATION_TOPIC:
            message = deserialize_message(serialized, TrackedObjectArray)
            observation_messages += 1
            nonempty_observation_messages += bool(message.objects)
        elif read_diagnostics and topic == ROS2_DIAGNOSTIC_TOPIC:
            message = deserialize_message(serialized, DiagnosticArray)
            for status in message.status:
                diagnostics.append({item.key: item.value
                                    for item in status.values})
    return {
        'truth': truth,
        'frames': frames,
        'diagnostics': diagnostics,
        'observation_messages': observation_messages,
        'nonempty_observation_messages': nonempty_observation_messages,
    }


def nearest_index(times, requested):
    index = bisect_left(times, requested)
    candidates = [item for item in (index - 1, index)
                  if 0 <= item < len(times)]
    return (min(candidates, key=lambda item: abs(times[item] - requested))
            if candidates else None)


def longest_false_run(stamps, flags):
    longest = 0.0
    start = None
    previous = None
    for stamp, flag in zip(stamps, flags):
        if flag:
            start = stamp if start is None else start
            previous = stamp
        elif start is not None:
            longest = max(longest, previous - start)
            start = None
    if start is not None and previous is not None:
        longest = max(longest, previous - start)
    return longest


def dynamic_metrics(truth, frames, static_scene, maximum_gap, gate, warmup,
                    identity_hysteresis):
    if not truth or not frames:
        return {
            'available': False,
            'evaluation_frames': 0,
            'detection_rate': None,
        }
    start = truth[0]['stamp'] + warmup
    frame_times = [frame['stamp'] for frame in frames]
    selected_ids = []
    nearest_ids = []
    detected_flags = []
    evaluated_stamps = []
    position_errors = []
    velocity_errors = []
    previous_id = None
    for item in truth:
        if item['stamp'] < start:
            continue
        evaluated_stamps.append(item['stamp'])
        index = nearest_index(frame_times, item['stamp'])
        if index is None or abs(frame_times[index] - item['stamp']) > maximum_gap:
            detected_flags.append(False)
            continue
        candidates = frames[index]['tracks']
        if not candidates:
            detected_flags.append(False)
            continue
        candidates_with_error = [
            (track, float(np.linalg.norm(
                track['position'][:2] - item['position'][:2]
            ))) for track in candidates
        ]
        nearest, nearest_error = min(
            candidates_with_error,
            key=lambda pair: pair[1],
        )
        selected = nearest
        error = nearest_error
        nearest_ids.append(nearest['id'])
        if previous_id is not None:
            previous = [pair for pair in candidates_with_error
                        if pair[0]['id'] == previous_id]
            if (previous and previous[0][1] <= gate and
                    previous[0][1] <= nearest_error + identity_hysteresis):
                selected, error = previous[0]
        valid = error <= gate
        detected_flags.append(valid)
        if valid:
            selected_ids.append(selected['id'])
            previous_id = selected['id']
            position_errors.append(error)
            velocity_errors.append(float(np.linalg.norm(
                selected['velocity'][:2] - item['velocity'][:2]
            )))

    frame_objects = 0
    false_objects = 0
    nonempty_frames = 0
    false_positive_frames = 0
    evaluated_output_frames = 0
    truth_times = [item['stamp'] for item in truth]
    for frame in frames:
        if frame['stamp'] < start:
            continue
        evaluated_output_frames += 1
        frame_objects += len(frame['tracks'])
        nonempty_frames += bool(frame['tracks'])
        truth_index = nearest_index(truth_times, frame['stamp'])
        target = truth[truth_index] if truth_index is not None else None
        frame_has_target = False
        for track in frame['tracks']:
            distance = (float(np.linalg.norm(
                track['position'][:2] - target['position'][:2]
            )) if target is not None else math.inf)
            is_target = distance <= gate and not static_scene
            frame_has_target = frame_has_target or is_target
            false_objects += not is_target
        if frame['tracks'] and not frame_has_target:
            false_positive_frames += 1

    id_switches = sum(first != second for first, second in zip(
        selected_ids, selected_ids[1:]
    ))
    nearest_id_switches = sum(first != second for first, second in zip(
        nearest_ids, nearest_ids[1:]
    ))
    first_detection = None
    if not static_scene:
        for item in truth:
            index = nearest_index(frame_times, item['stamp'])
            if (index is None or
                    abs(frame_times[index] - item['stamp']) > maximum_gap):
                continue
            if any(float(np.linalg.norm(
                    track['position'][:2] - item['position'][:2]
                    )) <= gate for track in frames[index]['tracks']):
                first_detection = item['stamp']
                break
    duration = frame_times[-1] - frame_times[0]
    evaluation_frames = len(detected_flags)
    valid_frames = sum(detected_flags)
    return {
        'available': True,
        'evaluation_frames': evaluation_frames,
        'valid_target_frames': valid_frames,
        'detection_rate': (
            valid_frames / evaluation_frames if evaluation_frames else 0.0
        ),
        'confirmation_delay_seconds': (
            None if static_scene or first_detection is None
            else first_detection - truth[0]['stamp']
        ),
        'longest_target_loss_seconds': (
            None if static_scene
            else longest_false_run(evaluated_stamps,
                                   [not item for item in detected_flags])
        ),
        'position_error_mean_m': mean(position_errors),
        'velocity_error_mean_mps': mean(velocity_errors),
        'id_switches': id_switches,
        'nearest_selection_id_switches': nearest_id_switches,
        'unique_dynamic_target_ids': len(set(selected_ids)),
        'output_frequency_hz': (
            (len(frames) - 1) / duration if duration > 0.0 else 0.0
        ),
        'nonempty_output_frames': nonempty_frames,
        'dynamic_object_count': frame_objects,
        'false_positive_objects': false_objects,
        'false_positive_object_rate': (
            false_objects / frame_objects if frame_objects else 0.0
        ),
        'false_positive_frames': false_positive_frames,
        'false_positive_frame_rate': (
            false_positive_frames / evaluated_output_frames
            if evaluated_output_frames else 0.0
        ),
    }


def resource_summary(path):
    if path is None or not path.is_file():
        return None
    samples = [json.loads(line) for line in path.read_text().splitlines()
               if line.strip()]
    active = [item for item in samples if item.get('process_count', 0)]
    return {
        'samples': len(active),
        'cpu_percent_mean': mean([
            item['detector_cpu_percent'] for item in active
        ]),
        'cpu_percent_max': max(
            [item['detector_cpu_percent'] for item in active], default=None
        ),
        'rss_mb_mean': mean([item['detector_rss_mb'] for item in active]),
        'rss_mb_max': max(
            [item['detector_rss_mb'] for item in active], default=None
        ),
    }


def main():
    args = parse_args()
    ros1 = read_bag(args.ros1_bag, ROS1_DYNAMIC_TOPIC)
    ros2 = read_bag(args.ros2_bag, ROS2_DYNAMIC_TOPIC, True)
    truth = ros1['truth']
    result = {
        'ros1_bag': str(args.ros1_bag),
        'ros2_bag': str(args.ros2_bag),
        'static_scene': args.static_scene,
        'truth_frames': len(truth),
        'target_gate_m': args.target_gate,
        'warmup_seconds': args.warmup,
        'identity_hysteresis_m': args.identity_hysteresis,
        'ros1': dynamic_metrics(
            truth, ros1['frames'], args.static_scene,
            args.maximum_time_gap, args.target_gate, args.warmup,
            args.identity_hysteresis,
        ),
        'ros2': dynamic_metrics(
            truth, ros2['frames'], args.static_scene,
            args.maximum_time_gap, args.target_gate, args.warmup,
            args.identity_hysteresis,
        ),
        'ros2_observation_messages': ros2['observation_messages'],
        'ros2_nonempty_observation_messages':
            ros2['nonempty_observation_messages'],
        'source_mux_safety': ros2['nonempty_observation_messages'] == 0,
        'ros2_final_diagnostics': (
            ros2['diagnostics'][-1] if ros2['diagnostics'] else {}
        ),
        'ros2_resources': resource_summary(args.resource_log),
    }
    encoded = json.dumps(result, indent=2, sort_keys=True)
    print(encoded)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(encoded + '\n')


if __name__ == '__main__':
    main()
