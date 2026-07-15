#!/usr/bin/env python3
"""Expose one fleet vehicle pose as the standard input expected by LV-DOT."""

import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from uav_usv_interfaces.msg import VehicleState


class LvDotPoseAdapter(Node):
    def __init__(self):
        super().__init__('lv_dot_pose_adapter')
        self.declare_parameter('vehicle_id', 'usv_01')
        self.declare_parameter('input_mode', 'vehicle_state')
        self.declare_parameter('input_topic', '/fleet/state')
        self.declare_parameter(
            'gazebo_pose_topic', '/world/lv_dot_tuning/pose/info'
        )
        self.declare_parameter('gazebo_entity_name', 'usv_01')
        self.declare_parameter(
            'output_topic', '/perception/lv_dot/usv_01/pose'
        )
        self.declare_parameter('frame_id', 'map')
        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.input_mode = str(
            self.get_parameter('input_mode').value
        ).strip().lower()
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publisher = self.create_publisher(
            PoseStamped, output_topic, qos_profile_sensor_data
        )
        self.gz_node = None
        self.gz_topic = ''
        self.callback_lock = threading.Lock()
        self.stopping = False
        if self.input_mode == 'vehicle_state':
            self.create_subscription(
                VehicleState,
                input_topic,
                self._on_state,
                qos_profile_sensor_data,
            )
            source = '%s[%s]' % (input_topic, self.vehicle_id)
        elif self.input_mode == 'gazebo':
            self.gz_node = GzTransportNode()
            gazebo_topic = str(
                self.get_parameter('gazebo_pose_topic').value
            )
            self.gz_topic = gazebo_topic
            self.gazebo_entity_name = str(
                self.get_parameter('gazebo_entity_name').value
            )
            if not self.gz_node.subscribe(
                Pose_V, gazebo_topic, self._on_gazebo_pose
            ):
                raise RuntimeError(
                    'Unable to subscribe to Gazebo pose topic '
                    + gazebo_topic
                )
            source = '%s[%s]' % (
                gazebo_topic, self.gazebo_entity_name
            )
        else:
            raise ValueError('input_mode must be vehicle_state or gazebo')
        self.get_logger().info(
            'LV-DOT pose adapter %s -> %s' % (source, output_topic)
        )

    def _on_state(self, message):
        if message.vehicle_id != self.vehicle_id or not message.online:
            return
        output = PoseStamped()
        output.header = message.header
        output.header.frame_id = message.header.frame_id or self.frame_id
        output.pose = message.pose
        self.publisher.publish(output)

    def _on_gazebo_pose(self, message):
        with self.callback_lock:
            if self.stopping or not rclpy.ok():
                return
            for pose in message.pose:
                if pose.name != self.gazebo_entity_name:
                    continue
                output = PoseStamped()
                output.header.stamp = self.get_clock().now().to_msg()
                output.header.frame_id = self.frame_id
                output.pose.position.x = float(pose.position.x)
                output.pose.position.y = float(pose.position.y)
                output.pose.position.z = float(pose.position.z)
                output.pose.orientation.x = float(pose.orientation.x)
                output.pose.orientation.y = float(pose.orientation.y)
                output.pose.orientation.z = float(pose.orientation.z)
                output.pose.orientation.w = float(pose.orientation.w)
                self.publisher.publish(output)
                return

    def destroy_node(self):
        with self.callback_lock:
            self.stopping = True
        if self.gz_node is not None and self.gz_topic:
            self.gz_node.unsubscribe(self.gz_topic)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LvDotPoseAdapter()
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
