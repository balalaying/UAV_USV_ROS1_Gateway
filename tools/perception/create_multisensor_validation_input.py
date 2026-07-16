#!/usr/bin/env python3
"""Create deterministic GT, camera, TF, and LV-DOT validation input."""

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ORIGINAL_TOPICS = (
    '/perception/ground_truth/tracks',
    '/fleet/uplink/uav_01/camera/image_raw',
    '/fleet/uplink/uav_01/camera/camera_info',
    '/tf',
    '/tf_static',
)
DYNAMIC_TOPIC = '/perception/lv_dot_ros2/dynamic_tracks'
PRIORITY = {
    '/tf_static': 0,
    '/tf': 1,
    '/perception/ground_truth/tracks': 2,
    '/fleet/uplink/uav_01/camera/camera_info': 3,
    '/fleet/uplink/uav_01/camera/image_raw': 4,
    DYNAMIC_TOPIC: 5,
}


def _message_stamp_ns(message, fallback):
    if hasattr(message, 'header'):
        stamp = message.header.stamp
    elif hasattr(message, 'transforms') and message.transforms:
        stamp = message.transforms[0].header.stamp
    else:
        return int(fallback)
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(fallback)


def _read_selected(bag_path, selected_topics):
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
    metadata_map = {
        item.name: item for item in reader.get_all_topics_and_types()
    }
    type_map = {name: item.type for name, item in metadata_map.items()}
    missing = set(selected_topics) - set(type_map)
    if missing:
        raise RuntimeError(
            '%s missing topics: %s' % (bag_path, sorted(missing))
        )
    message_types = {
        topic: get_message(type_map[topic]) for topic in selected_topics
    }
    records = []
    while reader.has_next():
        topic, serialized, recorded_at = reader.read_next()
        if topic not in selected_topics:
            continue
        message = deserialize_message(serialized, message_types[topic])
        records.append((
            _message_stamp_ns(message, recorded_at),
            PRIORITY.get(topic, 100),
            topic,
            serialized,
            type_map[topic],
            metadata_map[topic].offered_qos_profiles,
        ))
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('original_bag', type=Path)
    parser.add_argument('phase4_bag', type=Path)
    parser.add_argument('output_bag', type=Path)
    args = parser.parse_args()
    if args.output_bag.exists():
        parser.error('output already exists: %s' % args.output_bag)

    records = _read_selected(args.original_bag, ORIGINAL_TOPICS)
    records.extend(_read_selected(args.phase4_bag, (DYNAMIC_TOPIC,)))
    records.sort(key=lambda item: (item[0], item[1], item[2]))

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
    topic_metadata = {
        topic: (type_name, qos_profiles)
        for _, _, topic, _, type_name, qos_profiles in records
    }
    for topic, (type_name, qos_profiles) in sorted(topic_metadata.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic,
            type=type_name,
            serialization_format='cdr',
            offered_qos_profiles=qos_profiles,
        ))
    for (
        timestamp, _priority, topic, serialized, _type_name, _qos_profiles
    ) in records:
        writer.write(topic, serialized, timestamp)
    print('Created %s with %d messages' % (args.output_bag, len(records)))
    for topic in sorted(topic_metadata):
        print('  %s: %d' % (
            topic, sum(record[2] == topic for record in records)
        ))


if __name__ == '__main__':
    main()
