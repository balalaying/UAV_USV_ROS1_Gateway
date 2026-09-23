#!/usr/bin/env python3
"""Standardize ROS 2 LV-DOT dynamic tracks as perception observations."""

from copy import deepcopy
import hashlib
import json
import math

import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.executors import ExternalShutdownException
from uav_usv_ros1_compat.node import Node
from uav_usv_ros1_compat.time_fields import time_is_zero
from uav_usv_ros1_compat.time_fields import time_parts
from std_msgs.msg import String
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray


def _stamp_is_zero(stamp):
    return time_is_zero(stamp)


def _uuid_is_zero(uuid_value):
    return not any(int(value) for value in uuid_value.uuid)


def _stable_uuid(track_id):
    value = bytearray(hashlib.sha256(track_id.encode('utf-8')).digest()[:16])
    value[6] = (value[6] & 0x0F) | 0x50
    value[8] = (value[8] & 0x3F) | 0x80
    return list(value)


def adapt_dynamic_message(message, fallback_stamp):
    """Return a V1-compatible observation array and conversion statistics."""
    output = TrackedObjectArray()
    output.header = deepcopy(message.header)
    if _stamp_is_zero(output.header.stamp):
        output.header.stamp = deepcopy(fallback_stamp)

    dropped_missing_id = 0
    clamped_confidence = 0
    for incoming in message.objects:
        track_id = incoming.track_id.strip()
        if not track_id:
            dropped_missing_id += 1
            continue

        tracked = deepcopy(incoming)
        tracked.track_id = track_id
        tracked.source_mask = int(tracked.source_mask) | int(
            TrackedObject.SOURCE_LIDAR
        )
        if not tracked.class_name:
            tracked.class_name = 'unknown'
        tracked.class_confidence = min(
            1.0, max(0.0, float(tracked.class_confidence))
        )
        tracked.sensor_source = 'lidar'
        confidence = float(tracked.confidence)
        if not math.isfinite(confidence):
            confidence = 0.0
        bounded_confidence = min(1.0, max(0.0, confidence))
        if bounded_confidence != confidence:
            clamped_confidence += 1
        tracked.confidence = bounded_confidence
        if _stamp_is_zero(tracked.last_update):
            tracked.last_update = deepcopy(output.header.stamp)
        if _stamp_is_zero(tracked.first_seen):
            tracked.first_seen = deepcopy(tracked.last_update)
        if _uuid_is_zero(tracked.uuid):
            tracked.uuid.uuid = _stable_uuid(track_id)
        output.objects.append(tracked)

    return output, {
        'input_count': len(message.objects),
        'output_count': len(output.objects),
        'dropped_missing_track_id': dropped_missing_id,
        'clamped_confidence': clamped_confidence,
    }


class LvDotObservationAdapter(Node):
    def __init__(self):
        super().__init__('lv_dot_observation_adapter')
        self.declare_parameter(
            'input_topic', '/perception/lv_dot/dynamic_tracks'
        )
        self.declare_parameter(
            'output_topic', '/perception/lv_dot/observations'
        )
        self.declare_parameter(
            'status_topic', '/perception/lv_dot/observation_status'
        )

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.publisher = self.create_publisher(
            TrackedObjectArray, self.output_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10
        )
        self.create_subscription(
            TrackedObjectArray,
            self.input_topic,
            self._on_dynamic_tracks,
            10,
        )
        self.message_count = 0
        self.get_logger().info(
            'LV-DOT observation adapter %s -> %s'
            % (self.input_topic, self.output_topic)
        )

    def _on_dynamic_tracks(self, message):
        output, statistics = adapt_dynamic_message(
            message, self.get_clock().now().to_msg()
        )
        self.publisher.publish(output)
        self.message_count += 1

        seconds, nanoseconds = time_parts(output.header.stamp)
        status = {
            'mode': 'shadow',
            'input_topic': self.input_topic,
            'output_topic': self.output_topic,
            'message_count': self.message_count,
            'frame_id': output.header.frame_id,
            'timestamp': {
                'sec': seconds,
                'nanosec': nanoseconds,
            },
            'compatibility': {
                'dynamic_probability': 'mapped_to_confidence',
                'motion_state': 'CONFIRMED_DYNAMIC',
                'sensor_source': 'source_mask',
                'covariance': 'pose.covariance/twist.covariance',
                'classification': 'preserved_without_inference',
            },
        }
        status.update(statistics)
        status_message = String()
        status_message.data = json.dumps(
            status, ensure_ascii=True, sort_keys=True
        )
        self.status_publisher.publish(status_message)


def main(args=None):
    ros1.init(args=args)
    node = LvDotObservationAdapter()
    try:
        ros1.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ros1.ok():
            ros1.shutdown()


if __name__ == '__main__':
    main()
