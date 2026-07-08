#!/usr/bin/env python3
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import ControlLease
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import VehicleState


class UsvFleetAgent(Node):
    """USV edge agent: sensor uplink, authority checks and Nav2 commands."""

    def __init__(self):
        super().__init__('usv_fleet_agent')
        self.declare_parameter('vehicle_id', 'usv_01')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('camera_topic', '/boat/front_camera')
        self.declare_parameter('scan_topic', '/boat/scan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('arrival_tolerance', 3.0)

        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.arrival_tolerance = float(
            self.get_parameter('arrival_tolerance').value
        )
        prefix = '/fleet/uplink/%s' % self.vehicle_id

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        lease_qos = QoSProfile(depth=10)
        lease_qos.reliability = ReliabilityPolicy.RELIABLE
        lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(
            VehicleState, '/fleet/state', sensor_qos
        )
        self.ack_pub = self.create_publisher(
            CommandAck, '/fleet/command_ack', 20
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, self.goal_topic, 10
        )
        self.emergency_pub = self.create_publisher(
            Twist, '/model/simple_boat/cmd_vel', 10
        )
        self.odom_uplink_pub = self.create_publisher(
            Odometry, prefix + '/odom', sensor_qos
        )
        self.camera_uplink_pub = self.create_publisher(
            Image, prefix + '/camera', sensor_qos
        )
        self.scan_uplink_pub = self.create_publisher(
            LaserScan, prefix + '/scan', sensor_qos
        )

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.camera_uplink_pub.publish,
            sensor_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_uplink_pub.publish,
            sensor_qos,
        )
        self.create_subscription(
            ControlLease, '/fleet/control_lease', self._on_lease, lease_qos
        )
        self.create_subscription(
            FleetCommand, '/fleet/command', self._on_command, 20
        )

        self.odom = None
        self.lease = None
        self.active_command_id = ''
        self.active_target = None
        self.status_text = 'waiting for odometry'
        self.emergency_stop = False
        self.create_timer(0.2, self._update)
        self.get_logger().info(
            'USV fleet agent %s ready; uplink=%s/*'
            % (self.vehicle_id, prefix)
        )

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lease_is_valid(self, lease_id):
        return (
            self.lease is not None
            and not self.lease.revoked
            and self.lease.lease_id == lease_id
            and self._stamp_seconds(self.lease.valid_until)
            > self._now_seconds()
        )

    def _on_lease(self, msg):
        if msg.vehicle_id in (self.vehicle_id, '*'):
            self.lease = msg

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_uplink_pub.publish(msg)

    def _ack(self, command_id, status, message, progress=0.0):
        ack = CommandAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.command_id = command_id
        ack.vehicle_id = self.vehicle_id
        ack.status = status
        ack.progress = float(progress)
        ack.message = message
        self.ack_pub.publish(ack)

    def _on_command(self, msg):
        if msg.vehicle_id not in (self.vehicle_id, '*'):
            return
        if self._stamp_seconds(msg.expires_at) <= self._now_seconds():
            self._ack(
                msg.command_id,
                CommandAck.STATUS_REJECTED,
                'command expired',
            )
            return
        if (
            msg.command_type != FleetCommand.COMMAND_EMERGENCY_STOP
            and not self._lease_is_valid(msg.lease_id)
        ):
            self._ack(
                msg.command_id,
                CommandAck.STATUS_REJECTED,
                'invalid or expired control lease',
            )
            return

        self.active_command_id = msg.command_id
        self._ack(
            msg.command_id,
            CommandAck.STATUS_ACCEPTED,
            'USV command accepted',
        )

        if msg.command_type == FleetCommand.COMMAND_NAVIGATE:
            goal = PoseStamped()
            goal.header = msg.header
            goal.header.frame_id = goal.header.frame_id or 'map'
            goal.pose = msg.target_pose
            self.goal_pub.publish(goal)
            self.active_target = (
                msg.target_pose.position.x,
                msg.target_pose.position.y,
            )
            self.status_text = 'navigating to base-station target'
            self._ack(
                msg.command_id,
                CommandAck.STATUS_EXECUTING,
                self.status_text,
                0.05,
            )
        elif msg.command_type == FleetCommand.COMMAND_HOLD:
            self.emergency_stop = False
            self.active_target = None
            if self.odom is not None:
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = 'map'
                goal.pose = self.odom.pose.pose
                self.goal_pub.publish(goal)
            self.status_text = 'holding position'
            self._ack(
                msg.command_id,
                CommandAck.STATUS_SUCCEEDED,
                self.status_text,
                1.0,
            )
            self.active_command_id = ''
        elif msg.command_type == FleetCommand.COMMAND_EMERGENCY_STOP:
            self.emergency_stop = True
            self.active_target = None
            self.status_text = 'emergency stop'
            self._ack(
                msg.command_id,
                CommandAck.STATUS_SUCCEEDED,
                self.status_text,
                1.0,
            )
        else:
            self._ack(
                msg.command_id,
                CommandAck.STATUS_REJECTED,
                'command is not supported by USV agent',
            )
            self.active_command_id = ''

    def _update(self):
        if self.emergency_stop:
            self.emergency_pub.publish(Twist())

        if self.active_target is not None and self.odom is not None:
            pose = self.odom.pose.pose.position
            distance = math.hypot(
                pose.x - self.active_target[0],
                pose.y - self.active_target[1],
            )
            if distance <= self.arrival_tolerance:
                command_id = self.active_command_id
                self.active_target = None
                self.active_command_id = ''
                self.status_text = 'navigation target reached'
                self._ack(
                    command_id,
                    CommandAck.STATUS_SUCCEEDED,
                    self.status_text,
                    1.0,
                )

        state = VehicleState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.vehicle_id = self.vehicle_id
        state.vehicle_type = VehicleState.TYPE_USV
        state.online = self.odom is not None
        state.armed = True
        state.mode = 'NAV2'
        if self.odom is not None:
            state.pose = self.odom.pose.pose
            state.twist = self.odom.twist.twist
        state.battery_percent = -1.0
        state.active_command_id = self.active_command_id
        state.status_text = self.status_text
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = UsvFleetAgent()
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
