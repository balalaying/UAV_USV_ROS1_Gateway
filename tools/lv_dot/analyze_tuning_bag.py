#!/usr/bin/env python3
"""Compute repeatable LV-DOT acceptance metrics from a rosbag2 directory."""

import argparse
from bisect import bisect_left
from collections import Counter
import json
import math
from pathlib import Path
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from visualization_msgs.msg import Marker


TRUTH_TOPIC = '/perception/ground_truth/tracks'
OBSERVATION_TOPIC = '/perception/lv_dot/observations'
SHADOW_METRICS_TOPIC = '/perception/lv_dot/shadow_metrics'
STAGE_TOPICS = {
    'lidar_bboxes': '/lv_dot/diagnostics/lidar_bboxes',
    'filtered_bboxes': '/lv_dot/diagnostics/filtered_bboxes',
    'tracked_bboxes': '/lv_dot/diagnostics/tracked_bboxes',
    'dynamic_bboxes': '/lv_dot/onboard_detector/dynamic_bboxes',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--target-id', default='target_vessel')
    parser.add_argument('--warmup-seconds', type=float, default=5.0)
    parser.add_argument('--maximum-time-gap', type=float, default=0.15)
    parser.add_argument('--association-distance', type=float, default=20.0)
    parser.add_argument('--resource-log', type=Path)
    parser.add_argument('--json-output', type=Path)
    return parser.parse_args()


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def vector_distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def position(tracked):
    point = tracked.pose.pose.position
    return (float(point.x), float(point.y), float(point.z))


def velocity(tracked):
    vector = tracked.twist.twist.linear
    return (float(vector.x), float(vector.y), float(vector.z))


def selected_truth(message, target_id):
    for tracked in message.objects:
        if tracked.track_id == target_id:
            return tracked
    return message.objects[0] if message.objects else None


def nearest_time_index(times, requested):
    index = bisect_left(times, requested)
    candidates = [candidate for candidate in (index - 1, index)
                  if 0 <= candidate < len(times)]
    return min(candidates, key=lambda candidate: abs(times[candidate] - requested))


def mean(values):
    return statistics.fmean(values) if values else None


def frequency(timestamps):
    if len(timestamps) < 2:
        return 0.0
    duration = timestamps[-1] - timestamps[0]
    return (len(timestamps) - 1) / duration if duration > 0.0 else 0.0


def marker_count(message):
    return sum(
        marker.action in (Marker.ADD, Marker.MODIFY)
        for marker in message.markers
    )


def resource_summary(path):
    if path is None or not path.is_file():
        return None
    samples = [json.loads(line) for line in path.read_text().splitlines()
               if line.strip()]
    result = {'samples': len(samples)}
    for key in ('host_cpu_percent', 'host_rss_mb',
                'lv_dot_cpu_percent', 'lv_dot_memory_mb'):
        values = [float(sample[key]) for sample in samples if key in sample]
        result[key + '_mean'] = mean(values)
        result[key + '_max'] = max(values) if values else None
    return result


def main():
    args = parse_args()
    if not (args.bag / 'metadata.yaml').is_file():
        raise SystemExit('Not a rosbag2 directory: %s' % args.bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_classes = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
    }

    counts = Counter()
    topic_times = {}
    truths = []
    observations = []
    stage_counts = {name: [] for name in STAGE_TOPICS}
    shadow_metrics = []
    dropped_by_sensor = Counter()
    first_timestamp = None
    last_timestamp = None

    stage_by_topic = {topic: name for name, topic in STAGE_TOPICS.items()}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns * 1e-9
        first_timestamp = timestamp if first_timestamp is None else first_timestamp
        last_timestamp = timestamp
        counts[topic] += 1
        topic_times.setdefault(topic, []).append(timestamp)
        if topic not in message_classes:
            continue
        if topic not in ({TRUTH_TOPIC, OBSERVATION_TOPIC,
                          SHADOW_METRICS_TOPIC,
                          '/fleet/sensor_status'} | set(stage_by_topic)):
            continue
        message = deserialize_message(serialized, message_classes[topic])
        if topic == TRUTH_TOPIC:
            tracked = selected_truth(message, args.target_id)
            if tracked is not None:
                truths.append((timestamp, tracked))
        elif topic == OBSERVATION_TOPIC:
            observations.append((timestamp, message.objects))
        elif topic == SHADOW_METRICS_TOPIC:
            try:
                shadow_metrics.append((timestamp, json.loads(message.data)))
            except (AttributeError, json.JSONDecodeError):
                pass
        elif topic in stage_by_topic:
            stage_counts[stage_by_topic[topic]].append(marker_count(message))
        elif topic == '/fleet/sensor_status':
            sensor = '%s/%s' % (message.vehicle_id, message.sensor_id)
            dropped_by_sensor[sensor] = max(
                dropped_by_sensor[sensor], int(message.dropped_messages)
            )

    if not truths:
        raise SystemExit('Bag does not contain target ground truth')
    observation_times = [sample[0] for sample in observations]
    evaluation_start = truths[0][0] + max(0.0, args.warmup_seconds)
    evaluated_truths = [sample for sample in truths if sample[0] >= evaluation_start]

    detections = []
    position_errors = []
    velocity_errors = []
    latencies = []
    matched_ids = []
    active_track_id = ''
    longest_loss = 0.0
    loss_started = None
    first_detection_time = None

    for truth_time, truth in evaluated_truths:
        matched = None
        matched_observation_time = None
        if observation_times:
            index = nearest_time_index(observation_times, truth_time)
            observation_time, objects = observations[index]
            if abs(observation_time - truth_time) <= args.maximum_time_gap:
                truth_position = position(truth)
                candidates = [
                    (vector_distance(truth_position, position(item)), item)
                    for item in objects
                ]
                continuing = [
                    item for distance, item in candidates
                    if item.track_id == active_track_id
                    and distance <= args.association_distance
                ]
                if continuing:
                    matched = continuing[0]
                if candidates:
                    distance, candidate = min(candidates, key=lambda item: item[0])
                    if matched is None and distance <= args.association_distance:
                        matched = candidate
                if matched is not None:
                    matched_observation_time = observation_time

        detected = matched is not None
        detections.append(detected)
        if detected:
            if first_detection_time is None:
                first_detection_time = truth_time
            if loss_started is not None:
                longest_loss = max(longest_loss, truth_time - loss_started)
                loss_started = None
            position_errors.append(
                vector_distance(position(truth), position(matched))
            )
            velocity_errors.append(
                vector_distance(velocity(truth), velocity(matched))
            )
            matched_ids.append(matched.track_id)
            active_track_id = matched.track_id
            update_stamp = stamp_seconds(matched.last_update)
            latency = matched_observation_time - update_stamp
            if 0.0 <= latency <= 5.0:
                latencies.append(latency)
        elif loss_started is None:
            loss_started = truth_time
    if loss_started is not None and evaluated_truths:
        longest_loss = max(longest_loss, evaluated_truths[-1][0] - loss_started)

    id_switches = sum(
        current != previous
        for previous, current in zip(matched_ids, matched_ids[1:])
    )
    duration = (last_timestamp - first_timestamp
                if first_timestamp is not None and last_timestamp is not None
                else 0.0)
    shadow_latencies = [
        float(payload['latency_ms'])
        for timestamp, payload in shadow_metrics
        if timestamp >= evaluation_start
        and payload.get('lv_dot_online')
        and isinstance(payload.get('latency_ms'), (int, float))
    ]
    result = {
        'bag': str(args.bag),
        'duration_seconds': duration,
        'bag_message_count': sum(counts.values()),
        'warmup_seconds': args.warmup_seconds,
        'raw_truth_frames': len(truths),
        'total_evaluation_frames': len(evaluated_truths),
        'valid_detection_frames': sum(detections),
        'detection_rate': (
            sum(detections) / len(detections) if detections else 0.0
        ),
        'first_detection_delay_seconds': (
            first_detection_time - truths[0][0]
            if first_detection_time is not None else None
        ),
        'longest_loss_seconds': longest_loss,
        'position_error_mean_m': mean(position_errors),
        'position_error_max_m': max(position_errors) if position_errors else None,
        'velocity_error_mean_mps': mean(velocity_errors),
        'velocity_error_max_mps': max(velocity_errors) if velocity_errors else None,
        'id_switches': id_switches,
        'latency_mean_ms': mean(latencies) * 1000.0 if latencies else None,
        'latency_max_ms': max(latencies) * 1000.0 if latencies else None,
        'shadow_reported_latency_mean_ms': mean(shadow_latencies),
        'shadow_reported_latency_max_ms': (
            max(shadow_latencies) if shadow_latencies else None
        ),
        'observation_frequency_hz': frequency(
            topic_times.get(OBSERVATION_TOPIC, [])
        ),
        'stage_average_counts': {
            name: mean(values) for name, values in stage_counts.items()
        },
        'dynamic_nonempty_rate': (
            sum(value > 0 for value in stage_counts['dynamic_bboxes'])
            / len(stage_counts['dynamic_bboxes'])
            if stage_counts['dynamic_bboxes'] else 0.0
        ),
        'topic_counts': dict(sorted(counts.items())),
        'topic_frequencies_hz': {
            topic: frequency(times) for topic, times in sorted(topic_times.items())
        },
        'dropped_messages': dict(sorted(dropped_by_sensor.items())),
        'resources': resource_summary(args.resource_log),
    }
    encoded = json.dumps(result, indent=2, sort_keys=True)
    print(encoded)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(encoded + '\n')


if __name__ == '__main__':
    main()
