#!/usr/bin/env python3
"""Convert Gazebo poses to the shared perception observation contract."""

import math
from pathlib import Path
import sys
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray

_TRACKING_DIRECTORY = Path(__file__).resolve().parents[1] / 'tracking'
if _TRACKING_DIRECTORY.is_dir():
    sys.path.insert(0, str(_TRACKING_DIRECTORY))

from track_association import stable_uuid_bytes  # noqa: E402


class EntityState:
    def __init__(self):
        self.pose = None
        self.first_seen = None
        self.last_update = None
        self.last_arrival = 0.0
        self.previous_position = None
        self.previous_time_ns = None
        self.previous_yaw = None
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw_rate = 0.0


class GroundTruthAdapter(Node):
    def __init__(self):
        super().__init__('ground_truth_adapter')
        self.declare_parameter(
            'pose_topic', '/world/minimal_dynamic_capture/pose/info'
        )
        self.declare_parameter('entity_name', '')
        self.declare_parameter('entity_names', ['target_vessel'])
        self.declare_parameter(
            'output_topic', '/perception/ground_truth/tracks'
        )
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('timeout_seconds', 1.0)
        self.declare_parameter('velocity_alpha', 0.35)
        self.declare_parameter('source_mask', 0)
        self.declare_parameter('classification', TrackedObject.CLASS_VESSEL)
        self.declare_parameter('dimensions', [7.0, 2.6, 3.5])
        self.declare_parameter('position_variance', 0.0025)
        self.declare_parameter('velocity_variance', 0.01)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        entity_name = str(self.get_parameter('entity_name').value)
        entity_names = [
            str(value)
            for value in self.get_parameter('entity_names').value
            if str(value)
        ]
        if entity_name:
            entity_names = [entity_name]
        self.entity_names = tuple(dict.fromkeys(entity_names))
        if not self.entity_names:
            raise ValueError('entity_names must contain at least one entity')
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.timeout = max(
            0.1, float(self.get_parameter('timeout_seconds').value)
        )
        self.velocity_alpha = min(
            1.0, max(0.0, float(
                self.get_parameter('velocity_alpha').value
            ))
        )
        self.source_mask = max(
            0, min(255, int(self.get_parameter('source_mask').value))
        )
        self.classification = max(
            0, min(255, int(self.get_parameter('classification').value))
        )
        dimensions = list(self.get_parameter('dimensions').value)
        if len(dimensions) != 3:
            raise ValueError('dimensions must contain x, y, z')
        self.dimensions = tuple(max(0.0, float(v)) for v in dimensions)
        self.position_variance = max(
            0.0, float(self.get_parameter('position_variance').value)
        )
        self.velocity_variance = max(
            0.0, float(self.get_parameter('velocity_variance').value)
        )
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray, self.output_topic, 10
        )
        self.states = {name: EntityState() for name in self.entity_names}
        self.lock = threading.Lock()
        self.gz_node = GzTransportNode()
        if not self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose):
            raise RuntimeError(
                'failed to subscribe Gazebo topic ' + self.pose_topic
            )
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            'Ground truth %s -> %s for %s'
            % (
                self.pose_topic,
                self.output_topic,
                ', '.join(self.entity_names),
            )
        )

    @staticmethod
    def _yaw(pose):
        orientation = pose.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _on_pose(self, message):
        poses = {
            pose.name: pose
            for pose in message.pose
            if pose.name in self.states
        }
        if not poses:
            return
        now = self.get_clock().now()
        arrival = time.monotonic()
        with self.lock:
            for name, pose in poses.items():
                state = self.states[name]
                position = (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                )
                yaw = self._yaw(pose)
                if (
                    state.previous_position is not None
                    and state.previous_time_ns is not None
                ):
                    dt = (now.nanoseconds - state.previous_time_ns) * 1e-9
                    if 1e-3 < dt < self.timeout * 2.0:
                        for index in range(3):
                            measured = (
                                position[index]
                                - state.previous_position[index]
                            ) / dt
                            state.velocity[index] += self.velocity_alpha * (
                                measured - state.velocity[index]
                            )
                        if state.previous_yaw is not None:
                            yaw_delta = math.atan2(
                                math.sin(yaw - state.previous_yaw),
                                math.cos(yaw - state.previous_yaw),
                            )
                            measured_rate = yaw_delta / dt
                            state.yaw_rate += self.velocity_alpha * (
                                measured_rate - state.yaw_rate
                            )
                state.pose = pose
                state.previous_position = position
                state.previous_time_ns = now.nanoseconds
                state.previous_yaw = yaw
                state.last_update = now.to_msg()
                state.last_arrival = arrival
                if state.first_seen is None:
                    state.first_seen = now.to_msg()

    def _tracked_object(self, name, state):
        tracked = TrackedObject()
        tracked.uuid.uuid = stable_uuid_bytes(name)
        tracked.track_id = name
        tracked.first_seen = state.first_seen
        tracked.last_update = state.last_update
        tracked.source_mask = self.source_mask
        tracked.classification = self.classification
        tracked.class_name = (
            'vessel'
            if self.classification == TrackedObject.CLASS_VESSEL
            else 'unknown'
        )
        tracked.class_confidence = 1.0
        tracked.sensor_source = 'ground_truth'
        tracked.pose.pose.position.x = float(state.pose.position.x)
        tracked.pose.pose.position.y = float(state.pose.position.y)
        tracked.pose.pose.position.z = float(state.pose.position.z)
        tracked.pose.pose.orientation.x = float(state.pose.orientation.x)
        tracked.pose.pose.orientation.y = float(state.pose.orientation.y)
        tracked.pose.pose.orientation.z = float(state.pose.orientation.z)
        tracked.pose.pose.orientation.w = float(state.pose.orientation.w)
        for index in (0, 7, 14):
            tracked.pose.covariance[index] = self.position_variance
        tracked.twist.twist.linear.x = state.velocity[0]
        tracked.twist.twist.linear.y = state.velocity[1]
        tracked.twist.twist.linear.z = state.velocity[2]
        tracked.twist.twist.angular.z = state.yaw_rate
        for index in (0, 7, 14):
            tracked.twist.covariance[index] = self.velocity_variance
        tracked.dimensions.x = self.dimensions[0]
        tracked.dimensions.y = self.dimensions[1]
        tracked.dimensions.z = self.dimensions[2]
        tracked.confidence = 1.0
        return tracked

    def _publish(self):
        now = self.get_clock().now().to_msg()
        arrival = time.monotonic()
        output = TrackedObjectArray()
        output.header.stamp = now
        output.header.frame_id = self.frame_id
        with self.lock:
            for name, state in self.states.items():
                if (
                    state.pose is None
                    or arrival - state.last_arrival > self.timeout
                ):
                    continue
                output.objects.append(self._tracked_object(name, state))
        self.publisher.publish(output)

    def destroy_node(self):
        self.gz_node.unsubscribe(self.pose_topic)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthAdapter()
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
