#!/usr/bin/env python3
"""Merge ground truth and Phase 4 dynamic tracks by message timestamp."""

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


GROUND_TRUTH_TOPIC = '/perception/ground_truth/tracks'
DYNAMIC_TOPIC = '/perception/lv_dot_ros2/dynamic_tracks'


def _stamp_ns(message, fallback):
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(fallback)


def _read_topic(bag_path, selected_topic):
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
    if selected_topic not in type_map:
        raise RuntimeError(
            '%s does not contain %s' % (bag_path, selected_topic)
        )
    message_type = get_message(type_map[selected_topic])
    records = []
    while reader.has_next():
        topic, serialized, recorded_at = reader.read_next()
        if topic != selected_topic:
            continue
        message = deserialize_message(serialized, message_type)
        records.append((
            _stamp_ns(message, recorded_at),
            selected_topic,
            serialized,
            type_map[selected_topic],
        ))
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('ground_truth_bag', type=Path)
    parser.add_argument('phase4_bag', type=Path)
    parser.add_argument('output_bag', type=Path)
    args = parser.parse_args()
    if args.output_bag.exists():
        parser.error('output already exists: %s' % args.output_bag)

    records = _read_topic(args.ground_truth_bag, GROUND_TRUTH_TOPIC)
    records.extend(_read_topic(args.phase4_bag, DYNAMIC_TOPIC))
    records.sort(key=lambda item: (item[0], item[1]))
    if not records:
        raise RuntimeError('no records selected')

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(args.output_bag), storage_id='sqlite3'
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topic_types = {
        topic: type_name for _, topic, _, type_name in records
    }
    for topic, type_name in sorted(topic_types.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic,
            type=type_name,
            serialization_format='cdr',
            offered_qos_profiles='',
        ))
    for timestamp, topic, serialized, _ in records:
        writer.write(topic, serialized, timestamp)
    print(
        'Created %s with %d records (%d GT, %d dynamic)'
        % (
            args.output_bag,
            len(records),
            sum(record[1] == GROUND_TRUTH_TOPIC for record in records),
            sum(record[1] == DYNAMIC_TOPIC for record in records),
        )
    )


if __name__ == '__main__':
    main()
