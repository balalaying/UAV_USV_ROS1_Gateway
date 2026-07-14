#!/usr/bin/env python3
"""Bridge Gazebo Harmonic PointCloudPacked into ROS 2 PointCloud2.

The Humble/Harmonic binary ros_gz bridge on this workstation uses an older
point-field enum layout.  Copying the packed payload while explicitly mapping
the field enum keeps the public ROS interface standard and lossless.
"""

from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock as RosClock
from sensor_msgs.msg import PointCloud2, PointField
import threading
import time


class GzPointCloudBridge(Node):
    def __init__(self):
        super().__init__('gz_pointcloud_bridge')
        self.declare_parameter('gz_topic', '/usv_01/mid360/points')
        self.declare_parameter('ros_topic', '/usv_01/mid360/points')
        self.declare_parameter('frame_id', 'usv_01/mid360_link')
        self.declare_parameter('gz_clock_topic', '/clock')
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('stamp_mode', 'source')

        self.gz_topic = str(self.get_parameter('gz_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        ros_topic = str(self.get_parameter('ros_topic').value)
        self.publish_clock = bool(
            self.get_parameter('publish_clock').value
        )
        self.stamp_mode = str(
            self.get_parameter('stamp_mode').value
        ).lower()
        if self.stamp_mode not in ('source', 'node'):
            raise ValueError('stamp_mode must be source or node')
        self.callback_lock = threading.Lock()
        self.stopping = False

        self.publisher = self.create_publisher(
            PointCloud2, ros_topic, qos_profile_sensor_data
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock_publisher = None
        if self.publish_clock:
            self.clock_publisher = self.create_publisher(
                RosClock, '/clock', clock_qos
            )
        self.gz_node = GzTransportNode()
        if not self.gz_node.subscribe(
            PointCloudPacked, self.gz_topic, self._on_pointcloud
        ):
            raise RuntimeError(
                'Unable to subscribe to Gazebo topic %s' % self.gz_topic
            )
        self.gz_clock_topic = str(
            self.get_parameter('gz_clock_topic').value
        )
        if self.publish_clock:
            if not self.gz_node.subscribe(
                GzClock, self.gz_clock_topic, self._on_clock
            ):
                raise RuntimeError(
                    'Unable to subscribe to Gazebo clock %s'
                    % self.gz_clock_topic
                )
        self.frames_received = 0
        self.get_logger().info(
            'Point cloud bridge ready: %s -> %s frame=%s'
            % (self.gz_topic, ros_topic, self.frame_id)
        )

    def destroy_node(self):
        self.stopping = True
        self.gz_node.unsubscribe(self.gz_topic)
        if self.publish_clock:
            self.gz_node.unsubscribe(self.gz_clock_topic)
        with self.callback_lock:
            pass
        time.sleep(0.05)
        super().destroy_node()

    def _on_clock(self, gz_msg):
        with self.callback_lock:
            if self.stopping or not rclpy.ok():
                return
            clock = RosClock()
            clock.clock.sec = gz_msg.sim.sec
            clock.clock.nanosec = gz_msg.sim.nsec
            self.clock_publisher.publish(clock)

    def _on_pointcloud(self, gz_msg):
        with self.callback_lock:
            if self.stopping or not rclpy.ok():
                return

            ros_msg = PointCloud2()
            if self.stamp_mode == 'node':
                ros_msg.header.stamp = self.get_clock().now().to_msg()
            else:
                ros_msg.header.stamp.sec = gz_msg.header.stamp.sec
                ros_msg.header.stamp.nanosec = gz_msg.header.stamp.nsec
            ros_msg.header.frame_id = self.frame_id
            ros_msg.height = gz_msg.height
            ros_msg.width = gz_msg.width
            ros_msg.is_bigendian = gz_msg.is_bigendian
            ros_msg.point_step = gz_msg.point_step
            ros_msg.row_step = gz_msg.row_step
            ros_msg.data = gz_msg.data
            ros_msg.is_dense = gz_msg.is_dense

            for gz_field in gz_msg.field:
                field = PointField()
                field.name = gz_field.name
                field.offset = gz_field.offset
                # Gazebo enums are zero-based; ROS PointField is one-based.
                field.datatype = gz_field.datatype + 1
                field.count = gz_field.count
                ros_msg.fields.append(field)

            self.publisher.publish(ros_msg)
            self.frames_received += 1
            if self.frames_received == 1:
                self.get_logger().info(
                    'First cloud: %dx%d, point_step=%d, fields=%s'
                    % (
                        ros_msg.width,
                        ros_msg.height,
                        ros_msg.point_step,
                        ','.join(field.name for field in ros_msg.fields),
                    )
                )


def main(args=None):
    rclpy.init(args=args)
    node = GzPointCloudBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
