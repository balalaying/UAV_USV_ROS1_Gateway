#!/usr/bin/env python3
"""Convert one Gazebo target pose into the shared tracked-object contract."""

import math
import threading

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        self.declare_parameter(
            'pose_topic', '/world/minimal_dynamic_capture/pose/info'
        )
        self.declare_parameter('track_id', 'target_vessel')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('velocity_alpha', 0.35)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.track_id = str(self.get_parameter('track_id').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.velocity_alpha = float(
            self.get_parameter('velocity_alpha').value
        )
        publish_rate = max(1.0, float(
            self.get_parameter('publish_rate').value
        ))

        self.publisher = self.create_publisher(
            TrackedObjectArray, '/fleet/perception/targets', 10
        )
        self.gz_node = GzTransportNode()
        self.lock = threading.Lock()
        self.pose = None
        self.previous_position = None
        self.previous_time = None
        self.velocity = [0.0, 0.0, 0.0]
        self.previous_yaw = None
        self.yaw_rate = 0.0
        self.first_seen = None
        if not self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v):
            raise RuntimeError('failed to subscribe Gazebo topic ' + self.pose_topic)
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            'Tracking Gazebo entity %s from %s'
            % (self.track_id, self.pose_topic)
        )

    def _on_pose_v(self, msg):
        pose = next(
            (item for item in msg.pose if item.name == self.track_id), None
        )
        if pose is None:
            return
        now = self.get_clock().now()
        with self.lock:
            position = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            yaw = math.atan2(
                2.0 * (
                    pose.orientation.w * pose.orientation.z
                    + pose.orientation.x * pose.orientation.y
                ),
                1.0 - 2.0 * (
                    pose.orientation.y * pose.orientation.y
                    + pose.orientation.z * pose.orientation.z
                ),
            )
            if self.previous_position is not None and self.previous_time is not None:
                dt = (now - self.previous_time).nanoseconds * 1e-9
                if 1e-3 < dt < 1.0:
                    alpha = max(0.0, min(1.0, self.velocity_alpha))
                    for index in range(3):
                        measured = (
                            position[index] - self.previous_position[index]
                        ) / dt
                        self.velocity[index] += alpha * (
                            measured - self.velocity[index]
                        )
                    if self.previous_yaw is not None:
                        yaw_delta = math.atan2(
                            math.sin(yaw - self.previous_yaw),
                            math.cos(yaw - self.previous_yaw),
                        )
                        measured_rate = yaw_delta / dt
                        self.yaw_rate += alpha * (
                            measured_rate - self.yaw_rate
                        )
            self.pose = pose
            self.previous_position = position
            self.previous_time = now
            self.previous_yaw = yaw
            if self.first_seen is None:
                self.first_seen = now.to_msg()

    def _publish(self):
        with self.lock:
            if self.pose is None:
                return
            pose = self.pose
            velocity = tuple(self.velocity)
            yaw_rate = self.yaw_rate
            first_seen = self.first_seen

        now = self.get_clock().now().to_msg()
        tracked = TrackedObject()
        tracked.track_id = self.track_id
        tracked.first_seen = first_seen
        tracked.last_update = now
        tracked.source_mask = TrackedObject.SOURCE_UNKNOWN
        tracked.classification = TrackedObject.CLASS_VESSEL
        tracked.pose.pose.position.x = float(pose.position.x)
        tracked.pose.pose.position.y = float(pose.position.y)
        tracked.pose.pose.position.z = float(pose.position.z)
        tracked.pose.pose.orientation.x = float(pose.orientation.x)
        tracked.pose.pose.orientation.y = float(pose.orientation.y)
        tracked.pose.pose.orientation.z = float(pose.orientation.z)
        tracked.pose.pose.orientation.w = float(pose.orientation.w)
        tracked.twist.twist.linear.x = velocity[0]
        tracked.twist.twist.linear.y = velocity[1]
        tracked.twist.twist.linear.z = velocity[2]
        tracked.twist.twist.angular.z = yaw_rate
        tracked.dimensions.x = 7.0
        tracked.dimensions.y = 2.6
        tracked.dimensions.z = 3.5
        tracked.confidence = 1.0

        array = TrackedObjectArray()
        array.header.stamp = now
        array.header.frame_id = self.frame_id
        array.objects = [tracked]
        self.publisher.publish(array)

    def destroy_node(self):
        self.gz_node.unsubscribe(self.pose_topic)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
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
