#!/usr/bin/env python3
"""Fuse TrackedObjectArray observations into stable fleet perception tracks."""

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray

_TRACKING_DIRECTORY = Path(__file__).resolve().parents[1] / 'tracking'
if _TRACKING_DIRECTORY.is_dir():
    sys.path.insert(0, str(_TRACKING_DIRECTORY))

from track_association import fused_confidence  # noqa: E402
from track_association import nearest_track  # noqa: E402
from track_association import planar_distance  # noqa: E402
from track_association import stable_uuid_bytes  # noqa: E402


@dataclass
class Candidate:
    topic: str
    observed_at: float
    received_at: float
    tracked: TrackedObject

    @property
    def position(self):
        point = self.tracked.pose.pose.position
        return (point.x, point.y, point.z)


@dataclass
class ActiveTrack:
    tracked: TrackedObject
    position: tuple
    last_seen: float
    observed_at: float


@dataclass
class ObservationBatch:
    topic: str
    observed_at: float
    received_at: float
    candidates: list


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quaternion_matrix(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
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


def _quaternion_multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _yaw(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def partition_ready_groups(
    groups, expected_topic_count, aggregation_wait, now_monotonic
):
    """Hold incomplete cross-source groups briefly without changing fusion."""
    ready = []
    waiting = []
    for group in groups:
        topic_count = len({item.topic for item in group})
        oldest_arrival = min(item.received_at for item in group)
        wait_elapsed = max(0.0, now_monotonic - oldest_arrival)
        if (
            aggregation_wait <= 0.0
            or topic_count >= expected_topic_count
            or wait_elapsed >= aggregation_wait
        ):
            ready.append(group)
        else:
            waiting.append(group)
    return ready, waiting


def select_synchronized_batches(histories, topics, sync_slop):
    """Select one temporally coherent non-empty batch from every source."""
    available = {}
    for topic in topics:
        batches = [
            batch for batch in histories.get(topic, ())
            if batch.candidates
        ]
        if not batches:
            return []
        available[topic] = batches

    watermark = min(
        max(batch.observed_at for batch in batches)
        for batches in available.values()
    )
    selected = []
    for topic in topics:
        batch = min(
            available[topic],
            key=lambda item: (
                abs(item.observed_at - watermark),
                -item.observed_at,
            ),
        )
        if abs(batch.observed_at - watermark) > sync_slop:
            return []
        selected.append(batch)
    return selected


class PerceptionFusionNode(Node):
    def __init__(self):
        super().__init__('perception_fusion_node')
        self.declare_parameter('input_topics', [
            '/perception/uav_01/observations',
            '/perception/usv_01/observations',
        ])
        self.declare_parameter('input_topics_csv', '')
        self.declare_parameter(
            'output_topic', '/perception/fused/tracks'
        )
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('max_input_age_seconds', 1.0)
        self.declare_parameter('sync_slop_seconds', 0.35)
        self.declare_parameter('aggregation_wait_seconds', 0.0)
        self.declare_parameter('observation_history_seconds', 0.0)
        self.declare_parameter('association_distance', 8.0)
        self.declare_parameter('track_timeout_seconds', 2.0)
        self.declare_parameter('smoothing_alpha', 0.75)
        self.declare_parameter('confidence_decay_per_second', 0.15)
        self.declare_parameter('tf_timeout_seconds', 0.1)
        self.declare_parameter('preferred_track_ids', ['target_vessel'])

        input_topics_csv = str(
            self.get_parameter('input_topics_csv').value
        )
        input_topics = (
            input_topics_csv.split(',')
            if input_topics_csv
            else self.get_parameter('input_topics').value
        )
        self.input_topics = tuple(dict.fromkeys(
            str(value).strip() for value in input_topics
            if str(value).strip()
        ))
        if not self.input_topics:
            raise ValueError('input_topics must contain at least one topic')
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.max_input_age = max(
            0.05, float(
                self.get_parameter('max_input_age_seconds').value
            )
        )
        self.sync_slop = max(
            0.0, float(self.get_parameter('sync_slop_seconds').value)
        )
        self.aggregation_wait = min(
            self.max_input_age,
            max(
                0.0,
                float(
                    self.get_parameter('aggregation_wait_seconds').value
                ),
            ),
        )
        self.history_seconds = max(
            0.0,
            float(
                self.get_parameter(
                    'observation_history_seconds'
                ).value
            ),
        )
        self.association_distance = max(
            0.01, float(
                self.get_parameter('association_distance').value
            )
        )
        self.track_timeout = max(
            self.max_input_age,
            float(self.get_parameter('track_timeout_seconds').value),
        )
        self.smoothing_alpha = min(
            1.0, max(0.0, float(
                self.get_parameter('smoothing_alpha').value
            ))
        )
        self.confidence_decay = max(
            0.0, float(
                self.get_parameter('confidence_decay_per_second').value
            )
        )
        self.tf_timeout = max(
            0.0, float(self.get_parameter('tf_timeout_seconds').value)
        )
        self.preferred_track_ids = tuple(
            str(value)
            for value in self.get_parameter('preferred_track_ids').value
            if str(value)
        )
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray, self.output_topic, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = []
        self.observation_history = {
            topic: [] for topic in self.input_topics
        }
        self.last_snapshot_signature = None
        self.tracks = {}
        self.next_track_number = 1
        self.last_tf_warning = {}
        for topic in self.input_topics:
            self.create_subscription(
                TrackedObjectArray,
                topic,
                lambda message, source_topic=topic: self._on_observations(
                    source_topic, message
                ),
                10,
            )
        self.create_timer(1.0 / publish_rate, self._process_and_publish)
        self.get_logger().info(
            'Fusion %s -> %s in %s'
            % (', '.join(self.input_topics), self.output_topic,
               self.target_frame)
        )

    def _warn_tf(self, frame, error):
        now = time.monotonic()
        if now - self.last_tf_warning.get(frame, 0.0) < 5.0:
            return
        self.last_tf_warning[frame] = now
        self.get_logger().warning(
            'Skipping observations in %s: %s' % (frame, error)
        )

    def _transform_object(self, tracked, source_frame, stamp):
        if not source_frame or source_frame == self.target_frame:
            return deepcopy(tracked)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException as error:
            self._warn_tf(source_frame, error)
            return None

        output = deepcopy(tracked)
        rotation = _quaternion_matrix(transform.rotation)
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ])
        point = output.pose.pose.position
        transformed_point = rotation @ np.array([
            point.x, point.y, point.z
        ]) + translation
        point.x, point.y, point.z = (
            float(transformed_point[0]),
            float(transformed_point[1]),
            float(transformed_point[2]),
        )

        source_orientation = output.pose.pose.orientation
        composed = _quaternion_multiply(
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
            (
                source_orientation.x,
                source_orientation.y,
                source_orientation.z,
                source_orientation.w,
            ),
        )
        source_orientation.x = composed[0]
        source_orientation.y = composed[1]
        source_orientation.z = composed[2]
        source_orientation.w = composed[3]

        for vector in (
            output.twist.twist.linear,
            output.twist.twist.angular,
        ):
            rotated = rotation @ np.array([vector.x, vector.y, vector.z])
            vector.x, vector.y, vector.z = map(float, rotated)

        transform_6d = np.zeros((6, 6))
        transform_6d[:3, :3] = rotation
        transform_6d[3:, 3:] = rotation
        pose_covariance = np.array(
            output.pose.covariance, dtype=float
        ).reshape((6, 6))
        twist_covariance = np.array(
            output.twist.covariance, dtype=float
        ).reshape((6, 6))
        output.pose.covariance = (
            transform_6d @ pose_covariance @ transform_6d.T
        ).reshape(-1).tolist()
        output.twist.covariance = (
            transform_6d @ twist_covariance @ transform_6d.T
        ).reshape(-1).tolist()
        return output

    def _on_observations(self, topic, message):
        received_at = time.monotonic()
        observed_at = _stamp_seconds(message.header.stamp)
        if observed_at <= 0.0:
            observed_at = self.get_clock().now().nanoseconds * 1e-9
        latest = []
        for tracked in message.objects:
            object_stamp = _stamp_seconds(tracked.last_update)
            if object_stamp <= 0.0:
                object_stamp = observed_at
            transformed = self._transform_object(
                tracked, message.header.frame_id, message.header.stamp
            )
            if transformed is None:
                continue
            candidate = Candidate(
                topic=topic,
                observed_at=object_stamp,
                received_at=received_at,
                tracked=transformed,
            )
            if self.history_seconds <= 0.0:
                self.pending.append(candidate)
            latest.append(candidate)
        if self.history_seconds > 0.0:
            self.observation_history[topic].append(ObservationBatch(
                topic=topic,
                observed_at=observed_at,
                received_at=received_at,
                candidates=latest,
            ))

    def _candidate_groups(self, candidates):
        groups = []
        for candidate in sorted(
            candidates, key=lambda item: (item.observed_at, item.topic)
        ):
            selected = None
            selected_distance = self.association_distance
            for group in groups:
                if candidate.topic in {item.topic for item in group}:
                    continue
                mean_time = sum(
                    item.observed_at for item in group
                ) / len(group)
                if abs(candidate.observed_at - mean_time) > self.sync_slop:
                    continue
                center = (
                    sum(item.position[0] for item in group) / len(group),
                    sum(item.position[1] for item in group) / len(group),
                )
                distance = planar_distance(candidate.position, center)
                if distance <= selected_distance:
                    selected = group
                    selected_distance = distance
            if selected is None:
                groups.append([candidate])
            else:
                selected.append(candidate)
        return groups

    def _select_track_id(self, group, assigned):
        candidate_ids = [
            item.tracked.track_id
            for item in group
            if item.tracked.track_id
        ]
        for preferred in self.preferred_track_ids:
            if preferred in candidate_ids and preferred not in assigned:
                return preferred
        for candidate_id in candidate_ids:
            if candidate_id in self.tracks and candidate_id not in assigned:
                return candidate_id
        position = (
            sum(item.position[0] for item in group) / len(group),
            sum(item.position[1] for item in group) / len(group),
            sum(item.position[2] for item in group) / len(group),
        )
        nearest = nearest_track(
            position,
            self.tracks,
            self.association_distance,
            assigned,
        )
        if nearest is not None:
            return nearest
        for candidate_id in candidate_ids:
            if (
                candidate_id not in self.tracks
                and candidate_id not in assigned
            ):
                return candidate_id
        while True:
            track_id = 'fused_%04d' % self.next_track_number
            self.next_track_number += 1
            if track_id not in self.tracks and track_id not in assigned:
                return track_id

    @staticmethod
    def _weighted(group, getter):
        weights = [max(0.05, item.tracked.confidence) for item in group]
        total = sum(weights)
        return sum(
            weight * getter(item.tracked)
            for weight, item in zip(weights, group)
        ) / total

    def _fuse_group(self, track_id, group, now_monotonic):
        output = TrackedObject()
        output.uuid.uuid = stable_uuid_bytes(track_id)
        output.track_id = track_id
        first_seen = [
            item.tracked.first_seen for item in group
            if _stamp_seconds(item.tracked.first_seen) > 0.0
        ]
        output.first_seen = min(
            first_seen, key=_stamp_seconds
        ) if first_seen else self.get_clock().now().to_msg()
        output.last_update = max(
            (item.tracked.last_update for item in group),
            key=_stamp_seconds,
        )

        source_mask = 0
        for item in group:
            source_mask |= int(item.tracked.source_mask)
        if len({item.topic for item in group}) > 1:
            source_mask |= TrackedObject.SOURCE_FUSED
        output.source_mask = source_mask
        classified = [
            item for item in group
            if item.tracked.classification != TrackedObject.CLASS_UNKNOWN
        ]
        if classified:
            semantic = max(
                classified,
                key=lambda item: max(
                    item.tracked.class_confidence,
                    item.tracked.confidence,
                ),
            ).tracked
            output.classification = semantic.classification
            output.class_name = semantic.class_name
            output.class_confidence = max(
                semantic.class_confidence, semantic.confidence
            )
        affiliated = [
            item.tracked for item in group
            if item.tracked.affiliation != TrackedObject.AFFILIATION_UNKNOWN
            and item.tracked.affiliation_confidence > 0.0
            and item.tracked.sensor_source != 'ground_truth'
        ]
        if affiliated:
            identity = max(
                affiliated,
                key=lambda tracked: (
                    tracked.affiliation_confidence,
                    tracked.sensor_source == 'camera+lidar',
                ),
            )
            output.affiliation = identity.affiliation
            output.affiliation_confidence = identity.affiliation_confidence
        else:
            output.affiliation = TrackedObject.AFFILIATION_UNKNOWN
            output.affiliation_confidence = 0.0
        source_names = []
        for bit, name in (
            (TrackedObject.SOURCE_LIDAR, 'lidar'),
            (TrackedObject.SOURCE_CAMERA, 'camera'),
            (TrackedObject.SOURCE_AIS, 'ais'),
        ):
            if source_mask & bit:
                source_names.append(name)
        output.sensor_source = (
            '+'.join(source_names) if source_names else 'ground_truth'
        )

        position = output.pose.pose.position
        position.x = self._weighted(
            group, lambda obj: obj.pose.pose.position.x
        )
        position.y = self._weighted(
            group, lambda obj: obj.pose.pose.position.y
        )
        position.z = self._weighted(
            group, lambda obj: obj.pose.pose.position.z
        )
        sin_yaw = self._weighted(
            group, lambda obj: math.sin(_yaw(obj.pose.pose.orientation))
        )
        cos_yaw = self._weighted(
            group, lambda obj: math.cos(_yaw(obj.pose.pose.orientation))
        )
        yaw = math.atan2(sin_yaw, cos_yaw)
        output.pose.pose.orientation.z = math.sin(0.5 * yaw)
        output.pose.pose.orientation.w = math.cos(0.5 * yaw)

        output.twist.twist.linear.x = self._weighted(
            group, lambda obj: obj.twist.twist.linear.x
        )
        output.twist.twist.linear.y = self._weighted(
            group, lambda obj: obj.twist.twist.linear.y
        )
        output.twist.twist.linear.z = self._weighted(
            group, lambda obj: obj.twist.twist.linear.z
        )
        output.twist.twist.angular.z = self._weighted(
            group, lambda obj: obj.twist.twist.angular.z
        )
        output.dimensions.x = self._weighted(
            group, lambda obj: obj.dimensions.x
        )
        output.dimensions.y = self._weighted(
            group, lambda obj: obj.dimensions.y
        )
        output.dimensions.z = self._weighted(
            group, lambda obj: obj.dimensions.z
        )

        weights = np.array([
            max(0.05, item.tracked.confidence) for item in group
        ])
        weights /= weights.sum()
        pose_covariances = np.array([
            item.tracked.pose.covariance for item in group
        ], dtype=float)
        twist_covariances = np.array([
            item.tracked.twist.covariance for item in group
        ], dtype=float)
        output.pose.covariance = np.average(
            pose_covariances, axis=0, weights=weights
        ).tolist()
        output.twist.covariance = np.average(
            twist_covariances, axis=0, weights=weights
        ).tolist()
        for axis, value in enumerate((position.x, position.y, position.z)):
            spread = sum(
                weight * (
                    item.position[axis] - value
                ) ** 2
                for weight, item in zip(weights, group)
            )
            output.pose.covariance[axis * 7] += float(spread)

        output.confidence = fused_confidence(
            item.tracked.confidence for item in group
        )
        existing = self.tracks.get(track_id)
        if existing is not None and self.smoothing_alpha < 1.0:
            alpha = self.smoothing_alpha
            old = existing.tracked
            position.x = alpha * position.x + (
                1.0 - alpha
            ) * old.pose.pose.position.x
            position.y = alpha * position.y + (
                1.0 - alpha
            ) * old.pose.pose.position.y
            position.z = alpha * position.z + (
                1.0 - alpha
            ) * old.pose.pose.position.z
            output.twist.twist.linear.x = (
                alpha * output.twist.twist.linear.x
                + (1.0 - alpha) * old.twist.twist.linear.x
            )
            output.twist.twist.linear.y = (
                alpha * output.twist.twist.linear.y
                + (1.0 - alpha) * old.twist.twist.linear.y
            )
            output.confidence = (
                alpha * output.confidence
                + (1.0 - alpha) * old.confidence
            )
            output.first_seen = old.first_seen
            # UNKNOWN never erases a confirmed camera identity during a short
            # single-source dropout. The normal track timeout still removes it.
            if (
                output.affiliation == TrackedObject.AFFILIATION_UNKNOWN
                and old.affiliation != TrackedObject.AFFILIATION_UNKNOWN
            ):
                output.affiliation = old.affiliation
                output.affiliation_confidence = (
                    old.affiliation_confidence * self.confidence_decay
                )

        observed_at = max(item.observed_at for item in group)
        self.tracks[track_id] = ActiveTrack(
            tracked=output,
            position=(position.x, position.y, position.z),
            last_seen=now_monotonic,
            observed_at=observed_at,
        )

    def _process_and_publish(self):
        now_monotonic = time.monotonic()
        if self.history_seconds > 0.0:
            for topic, batches in self.observation_history.items():
                self.observation_history[topic] = [
                    batch for batch in batches
                    if now_monotonic - batch.received_at
                    <= self.history_seconds
                ]
            selected_batches = select_synchronized_batches(
                self.observation_history,
                self.input_topics,
                self.sync_slop,
            )
            signature = tuple(
                (batch.topic, round(batch.observed_at, 9))
                for batch in selected_batches
            )
            if selected_batches and signature != self.last_snapshot_signature:
                candidates = [
                    candidate
                    for batch in selected_batches
                    for candidate in batch.candidates
                ]
                self.last_snapshot_signature = signature
            else:
                candidates = []
        else:
            candidates = [
                item for item in self.pending
                if now_monotonic - item.received_at <= self.max_input_age
            ]
        groups = self._candidate_groups(candidates)
        if self.history_seconds > 0.0:
            ready_groups = [
                group for group in groups
                if len({item.topic for item in group})
                == len(self.input_topics)
            ]
            waiting_groups = []
        else:
            ready_groups, waiting_groups = partition_ready_groups(
                groups,
                len(self.input_topics),
                self.aggregation_wait,
                now_monotonic,
            )
            self.pending = [
                item for group in waiting_groups for item in group
            ]
        assigned = set()
        for group in ready_groups:
            track_id = self._select_track_id(group, assigned)
            assigned.add(track_id)
            self._fuse_group(track_id, group, now_monotonic)

        expired = [
            track_id for track_id, track in self.tracks.items()
            if now_monotonic - track.last_seen > self.track_timeout
        ]
        for track_id in expired:
            self.tracks.pop(track_id, None)

        output = TrackedObjectArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.target_frame
        for track_id in sorted(self.tracks):
            track = self.tracks[track_id]
            tracked = deepcopy(track.tracked)
            age = max(0.0, now_monotonic - track.last_seen)
            tracked.confidence *= math.exp(-self.confidence_decay * age)
            output.objects.append(tracked)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
