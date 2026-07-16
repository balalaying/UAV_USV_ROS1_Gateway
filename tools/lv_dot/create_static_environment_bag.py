#!/usr/bin/env python3
"""Create a reproducible static LV-DOT bag from one accepted sensor frame."""

import argparse
import copy
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from uav_usv_interfaces.msg import TrackedObjectArray


CLOUD_TOPIC = '/perception/usv_01/points_filtered'
USV_POSE_TOPIC = '/perception/lv_dot/usv_01/pose'
UAV_POSE_TOPIC = '/perception/lv_dot/uav_01/pose'
TRUTH_TOPIC = '/perception/ground_truth/tracks'
TF_TOPIC = '/tf'
TF_STATIC_TOPIC = '/tf_static'
MOTION_TOPIC = '/lv_dot/tuning/target_motion'


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('source_bag', type=Path)
    parser.add_argument('output_bag', type=Path)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--sample-offset', type=float, default=15.0)
    return parser.parse_args()


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def set_stamp(stamp, nanoseconds):
    stamp.sec = int(nanoseconds // 1_000_000_000)
    stamp.nanosec = int(nanoseconds % 1_000_000_000)


def stamp_message(message, nanoseconds):
    if hasattr(message, 'header'):
        set_stamp(message.header.stamp, nanoseconds)
    if isinstance(message, TrackedObjectArray):
        for item in message.objects:
            set_stamp(item.last_update, nanoseconds)
            item.twist.twist.linear.x = 0.0
            item.twist.twist.linear.y = 0.0
            item.twist.twist.linear.z = 0.0
            item.twist.twist.angular.x = 0.0
            item.twist.twist.angular.y = 0.0
            item.twist.twist.angular.z = 0.0
    if isinstance(message, TFMessage):
        for transform in message.transforms:
            set_stamp(transform.header.stamp, nanoseconds)


def topic_metadata(reader):
    return {item.name: item for item in reader.get_all_topics_and_types()}


def main():
    args = parse_args()
    if not (args.source_bag / 'metadata.yaml').is_file():
        raise SystemExit(f'Not a rosbag2 directory: {args.source_bag}')
    if args.output_bag.exists():
        raise SystemExit(f'Output already exists: {args.output_bag}')
    if args.duration <= 0.0 or args.rate <= 0.0:
        raise SystemExit('duration and rate must be positive')

    reader = open_reader(args.source_bag)
    metadata = topic_metadata(reader)
    first_record_ns = None
    selected_cloud = None
    selected_pose = None
    selected_uav_pose = None
    selected_truth = None
    selected_motion = String(data='static_environment')
    dynamic_transforms = {}
    static_transforms = {}

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if first_record_ns is None:
            first_record_ns = record_ns
        if topic == TF_TOPIC:
            message = deserialize_message(serialized, TFMessage)
            for transform in message.transforms:
                dynamic_transforms[transform.child_frame_id] = copy.deepcopy(
                    transform
                )
        elif topic == TF_STATIC_TOPIC:
            message = deserialize_message(serialized, TFMessage)
            for transform in message.transforms:
                static_transforms[transform.child_frame_id] = copy.deepcopy(
                    transform
                )
        elif topic == USV_POSE_TOPIC:
            selected_pose = deserialize_message(serialized, PoseStamped)
        elif topic == UAV_POSE_TOPIC:
            selected_uav_pose = deserialize_message(serialized, PoseStamped)
        elif topic == TRUTH_TOPIC:
            selected_truth = deserialize_message(
                serialized, TrackedObjectArray
            )
        elif topic == MOTION_TOPIC:
            selected_motion = deserialize_message(serialized, String)
        elif topic == CLOUD_TOPIC:
            elapsed = (record_ns - first_record_ns) * 1e-9
            if elapsed >= args.sample_offset:
                selected_cloud = deserialize_message(serialized, PointCloud2)
                break

    required = {
        CLOUD_TOPIC: selected_cloud,
        USV_POSE_TOPIC: selected_pose,
        TRUTH_TOPIC: selected_truth,
    }
    missing = [name for name, message in required.items() if message is None]
    if missing:
        raise SystemExit('Missing required source messages: ' + ', '.join(missing))
    if not dynamic_transforms:
        raise SystemExit('No dynamic TF snapshot was found before sample frame')

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.output_bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topics = [CLOUD_TOPIC, USV_POSE_TOPIC, TRUTH_TOPIC, TF_TOPIC]
    if selected_uav_pose is not None:
        topics.append(UAV_POSE_TOPIC)
    if static_transforms:
        topics.append(TF_STATIC_TOPIC)
    if MOTION_TOPIC in metadata:
        topics.append(MOTION_TOPIC)
    for topic in topics:
        original = metadata[topic]
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic,
            type=original.type,
            serialization_format=original.serialization_format,
            offered_qos_profiles=original.offered_qos_profiles,
        ))

    start_ns = (
        int(selected_cloud.header.stamp.sec) * 1_000_000_000
        + int(selected_cloud.header.stamp.nanosec)
    )
    if static_transforms:
        message = TFMessage(transforms=list(static_transforms.values()))
        stamp_message(message, start_ns)
        writer.write(TF_STATIC_TOPIC, serialize_message(message), start_ns)

    frame_count = max(2, int(round(args.duration * args.rate)))
    period_ns = int(round(1_000_000_000 / args.rate))
    for index in range(frame_count):
        stamp_ns = start_ns + index * period_ns
        messages = {
            TF_TOPIC: TFMessage(transforms=copy.deepcopy(
                list(dynamic_transforms.values())
            )),
            USV_POSE_TOPIC: copy.deepcopy(selected_pose),
            CLOUD_TOPIC: copy.deepcopy(selected_cloud),
            TRUTH_TOPIC: copy.deepcopy(selected_truth),
        }
        if selected_uav_pose is not None:
            messages[UAV_POSE_TOPIC] = copy.deepcopy(selected_uav_pose)
        if MOTION_TOPIC in topics:
            messages[MOTION_TOPIC] = copy.deepcopy(selected_motion)
            messages[MOTION_TOPIC].data = 'static_environment'
        for topic, message in messages.items():
            stamp_message(message, stamp_ns)
            writer.write(topic, serialize_message(message), stamp_ns)

    print(f'Created {args.output_bag}')
    print(f'frames={frame_count} rate_hz={args.rate:.3f}')
    print(f'cloud_frame={selected_cloud.header.frame_id}')
    print(f'dynamic_tf_count={len(dynamic_transforms)}')
    print(f'static_tf_count={len(static_transforms)}')


if __name__ == '__main__':
    main()
