#!/usr/bin/env python3
"""Publish selected Gazebo entities using the repository tracking contract."""

import math
import threading
import uuid

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
import rospy
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray


class GazeboEntityTruthBridge:
    def __init__(self):
        self.world_name = rospy.get_param('~world_name', 'heterogeneous_332')
        self.pose_topic = rospy.get_param(
            '~pose_topic', '/world/%s/pose/info' % self.world_name
        )
        self.output_topic = rospy.get_param(
            '~output_topic', '/fleet/perception/targets'
        )
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.publish_rate = max(0.5, float(
            rospy.get_param('~publish_rate_hz', 10.0)
        ))
        self.entities = {
            'friendly_ship': TrackedObject.AFFILIATION_FRIENDLY,
            'enemy_ship': TrackedObject.AFFILIATION_HOSTILE,
        }
        configured = rospy.get_param(
            '~entity_ids', ['friendly_ship', 'enemy_ship']
        )
        self.entity_ids = [str(item) for item in configured]
        self._poses = {}
        self._previous = {}
        self._lock = threading.Lock()
        self._publisher = rospy.Publisher(
            self.output_topic, TrackedObjectArray, queue_size=10
        )
        self._gz_node = GzTransportNode()
        if not self._gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose):
            raise RuntimeError('failed to subscribe to ' + self.pose_topic)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._publish
        )
        rospy.loginfo(
            'Gazebo entity truth bridge: %s -> %s',
            self.pose_topic, self.output_topic,
        )

    def _on_pose(self, message):
        now = rospy.Time.now().to_sec()
        updates = {}
        for pose in message.pose:
            if pose.name not in self.entity_ids:
                continue
            updates[pose.name] = (
                now,
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
        if updates:
            with self._lock:
                self._poses.update(updates)

    @staticmethod
    def _uuid_bytes(entity_id):
        return list(uuid.uuid5(uuid.NAMESPACE_DNS, 'uav-usv/' + entity_id).bytes)

    def _object(self, entity_id, sample):
        stamp = rospy.Time.from_sec(sample[0])
        result = TrackedObject()
        result.uuid.uuid = self._uuid_bytes(entity_id)
        result.track_id = entity_id
        result.first_seen = stamp
        result.last_update = stamp
        result.source_mask = TrackedObject.SOURCE_FUSED
        result.classification = TrackedObject.CLASS_VESSEL
        result.class_name = 'vessel'
        result.class_confidence = 1.0
        result.sensor_source = 'gazebo_ground_truth'
        result.pose.pose.position.x = sample[1]
        result.pose.pose.position.y = sample[2]
        result.pose.pose.position.z = sample[3]
        result.pose.pose.orientation.x = sample[4]
        result.pose.pose.orientation.y = sample[5]
        result.pose.pose.orientation.z = sample[6]
        result.pose.pose.orientation.w = sample[7]
        result.dimensions.x = 13.5 if entity_id == 'friendly_ship' else 7.0
        result.dimensions.y = 5.0 if entity_id == 'friendly_ship' else 3.3
        result.dimensions.z = 4.5 if entity_id == 'friendly_ship' else 2.2
        result.confidence = 1.0
        result.affiliation = self.entities.get(
            entity_id, TrackedObject.AFFILIATION_UNKNOWN
        )
        result.affiliation_confidence = 1.0
        previous = self._previous.get(entity_id)
        if previous is not None:
            dt = sample[0] - previous[0]
            if dt > 1.0e-3:
                result.twist.twist.linear.x = (sample[1] - previous[1]) / dt
                result.twist.twist.linear.y = (sample[2] - previous[2]) / dt
                result.twist.twist.linear.z = (sample[3] - previous[3]) / dt
                result.confidence = max(
                    0.8, min(1.0, 1.0 - abs(dt - 0.1) * 0.1)
                )
        self._previous[entity_id] = sample
        return result

    def _publish(self, _event):
        with self._lock:
            samples = dict(self._poses)
        output = TrackedObjectArray()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = self.map_frame
        for entity_id in self.entity_ids:
            sample = samples.get(entity_id)
            if sample is not None and math.isfinite(sample[1]):
                output.objects.append(self._object(entity_id, sample))
        self._publisher.publish(output)


def main():
    rospy.init_node('gz_entity_truth_bridge')
    GazeboEntityTruthBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
