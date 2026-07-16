#!/usr/bin/env python3
"""Analyze LiDAR, camera, and fused target tracks against ground truth."""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    'ground_truth': '/perception/ground_truth/tracks',
    'lv_dot': '/perception/lv_dot/observations',
    'uav_camera': '/perception/uav_01/observations',
    'fusion': '/perception/fused/tracks',
}
SOURCE_LIDAR = 1
SOURCE_CAMERA = 2
SOURCE_FUSED = 8


def _stamp(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _object_stamp(tracked, message):
    value = _stamp(tracked.last_update)
    return value if value > 0.0 else _stamp(message.header.stamp)


def _position(tracked):
    value = tracked.pose.pose.position
    return (float(value.x), float(value.y), float(value.z))


def _velocity(tracked):
    value = tracked.twist.twist.linear
    return (float(value.x), float(value.y), float(value.z))


def _distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _read(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id='sqlite3'
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    type_map = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = {topic: name for name, topic in TOPICS.items()}
    data = {name: [] for name in TOPICS}
    while reader.has_next():
        topic, serialized, _recorded_at = reader.read_next()
        name = selected.get(topic)
        if name is None:
            continue
        message = deserialize_message(
            serialized, get_message(type_map[topic])
        )
        data[name].append(message)
    return data


def _truth_samples(messages, target_id):
    samples = []
    for message in messages:
        tracked = next(
            (
                item for item in message.objects
                if item.track_id == target_id
            ),
            message.objects[0] if message.objects else None,
        )
        if tracked is not None:
            samples.append((_object_stamp(tracked, message), tracked))
    return samples


def _candidate_samples(messages):
    unique = {}
    for message in messages:
        for tracked in message.objects:
            stamp = _object_stamp(tracked, message)
            unique[(round(stamp, 9), tracked.track_id)] = (stamp, tracked)
    return sorted(unique.values(), key=lambda item: item[0])


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _evaluate(
    truth_samples,
    candidates,
    time_gate,
    distance_gate,
    preferred_track_id='',
):
    candidate_times = [item[0] for item in candidates]
    position_errors = []
    velocity_errors = []
    timestamp_deltas = []
    confidences = []
    track_ids = []
    source_masks = []
    for truth_stamp, truth in truth_samples:
        center = bisect_left(candidate_times, truth_stamp)
        matches = []
        for candidate_stamp, tracked in candidates[
            max(0, center - 20):min(len(candidates), center + 21)
        ]:
            delta = abs(candidate_stamp - truth_stamp)
            if delta > time_gate:
                continue
            distance = _distance(_position(truth), _position(tracked))
            if distance <= distance_gate:
                matches.append((delta, distance, candidate_stamp, tracked))
        if not matches:
            continue
        preferred_ids = []
        if preferred_track_id:
            preferred_ids.append(preferred_track_id)
        if track_ids:
            preferred_ids.append(track_ids[-1])
        selected = None
        for track_id in preferred_ids:
            preferred = [
                item for item in matches if item[3].track_id == track_id
            ]
            if preferred:
                selected = min(preferred, key=lambda item: item[:2])
                break
        if selected is None:
            selected = min(matches, key=lambda item: item[:2])
        delta, position_error, _candidate_stamp, tracked = selected
        position_errors.append(position_error)
        velocity_errors.append(
            _distance(_velocity(truth), _velocity(tracked))
        )
        timestamp_deltas.append(delta * 1000.0)
        confidences.append(float(tracked.confidence))
        track_ids.append(tracked.track_id)
        source_masks.append(int(tracked.source_mask))

    id_switches = sum(
        current != previous
        for previous, current in zip(track_ids, track_ids[1:])
    )
    total = len(truth_samples)
    all_source_rate = (
        sum(
            bool(mask & SOURCE_LIDAR)
            and bool(mask & SOURCE_CAMERA)
            and bool(mask & SOURCE_FUSED)
            for mask in source_masks
        ) / float(len(source_masks))
        if source_masks else None
    )
    return {
        'ground_truth_samples': total,
        'matched_samples': len(track_ids),
        'detection_rate': len(track_ids) / float(total) if total else None,
        'mean_position_error_m': (
            statistics.fmean(position_errors) if position_errors else None
        ),
        'p95_position_error_m': _percentile(position_errors, 95.0),
        'mean_velocity_error_mps': (
            statistics.fmean(velocity_errors) if velocity_errors else None
        ),
        'mean_timestamp_delta_ms': (
            statistics.fmean(timestamp_deltas)
            if timestamp_deltas else None
        ),
        'mean_confidence': (
            statistics.fmean(confidences) if confidences else None
        ),
        'id_switches': id_switches,
        'unique_track_ids': sorted(set(track_ids)),
        'source_masks': sorted(set(source_masks)),
        'all_sensor_source_rate': all_source_rate,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--target-id', default='target_vessel')
    parser.add_argument('--time-gate', type=float, default=0.5)
    parser.add_argument('--distance-gate', type=float, default=12.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()

    data = _read(args.bag)
    truth = _truth_samples(data['ground_truth'], args.target_id)
    result = {
        'bag': str(args.bag),
        'target_id': args.target_id,
        'message_counts': {
            name: len(messages) for name, messages in data.items()
        },
    }
    for name in ('lv_dot', 'uav_camera', 'fusion'):
        result[name] = _evaluate(
            truth,
            _candidate_samples(data[name]),
            args.time_gate,
            args.distance_gate,
            preferred_track_id=(args.target_id if name == 'fusion' else ''),
        )
    text = json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True)
    if args.output:
        args.output.write_text(text + '\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
