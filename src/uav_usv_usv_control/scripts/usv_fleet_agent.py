#!/usr/bin/env python3
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import rclpy
import time
from rclpy.action import ActionClient
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
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('arrival_tolerance', 3.0)
        self.declare_parameter(
            'emergency_cmd_topic', '/model/simple_boat/cmd_vel'
        )
        self.declare_parameter('simulate_unreachable', False)
        self.declare_parameter('manage_nav2_lifecycle', False)
        self.declare_parameter('nav2_lifecycle_retry_period', 2.0)
        self.declare_parameter('nav2_lifecycle_request_timeout', 15.0)

        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.navigate_action = self.get_parameter('navigate_action').value
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
        self.nav_client = ActionClient(
            self, NavigateToPose, self.navigate_action
        )
        self.manage_nav2_lifecycle = bool(
            self.get_parameter('manage_nav2_lifecycle').value
        )
        self.nav2_lifecycle_retry_period = max(
            0.5,
            float(
                self.get_parameter('nav2_lifecycle_retry_period').value
            ),
        )
        self.nav2_lifecycle_request_timeout = max(
            2.0,
            float(
                self.get_parameter('nav2_lifecycle_request_timeout').value
            ),
        )
        nav_namespace = self.navigate_action.rsplit('/', 1)[0]
        self.nav2_lifecycle_nodes = (
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        )
        self.nav2_get_state_clients = {
            name: self.create_client(
                GetState, '%s/%s/get_state' % (nav_namespace, name)
            )
            for name in self.nav2_lifecycle_nodes
        }
        self.nav2_change_state_clients = {
            name: self.create_client(
                ChangeState, '%s/%s/change_state' % (nav_namespace, name)
            )
            for name in self.nav2_lifecycle_nodes
        }
        self.emergency_pub = self.create_publisher(
            Twist,
            self.get_parameter('emergency_cmd_topic').value,
            10,
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
        self.nav_goal_handle = None
        self.pending_nav_goal = None
        self.last_feedback_ack = 0.0
        self.nav2_ready = False
        self.nav2_stack_activated = False
        self.nav2_lifecycle_index = 0
        self.nav2_lifecycle_future = None
        self.nav2_lifecycle_operation = ''
        self.nav2_lifecycle_node = ''
        self.nav2_lifecycle_request_started = 0.0
        self.nav2_lifecycle_retry_after = 0.0
        self.nav2_ready_logged = False
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
        self.last_feedback_ack = 0.0
        self._ack(
            msg.command_id,
            CommandAck.STATUS_ACCEPTED,
            'USV command accepted',
        )

        if msg.command_type == FleetCommand.COMMAND_NAVIGATE:
            if bool(self.get_parameter('simulate_unreachable').value):
                self.status_text = 'simulated navigation backend unreachable'
                self._ack(
                    msg.command_id,
                    CommandAck.STATUS_FAILED,
                    self.status_text,
                )
                self.active_command_id = ''
                return
            goal = PoseStamped()
            goal.header = msg.header
            goal.header.frame_id = goal.header.frame_id or 'map'
            goal.pose = msg.target_pose
            self.active_target = (
                msg.target_pose.position.x,
                msg.target_pose.position.y,
            )
            self.status_text = 'waiting for Nav2 action server'
            self.pending_nav_goal = (goal, msg.command_id)
            self._send_pending_nav_goal()
            self._ack(
                msg.command_id,
                CommandAck.STATUS_EXECUTING,
                self.status_text,
                0.05,
            )
        elif msg.command_type == FleetCommand.COMMAND_HOLD:
            self.emergency_stop = False
            self.active_target = None
            self.pending_nav_goal = None
            self._cancel_nav_goal()
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
            self.pending_nav_goal = None
            self._cancel_nav_goal()
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

    def _cancel_nav_goal(self):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def _send_pending_nav_goal(self):
        if self.pending_nav_goal is None:
            return
        if not self.nav2_ready:
            return
        pending_goal, command_id = self.pending_nav_goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pending_goal
        self.pending_nav_goal = None
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback, cid=command_id: (
                self._on_nav_feedback(feedback, cid)
            ),
        )
        future.add_done_callback(
            lambda response, cid=command_id: (
                self._on_nav_goal_response(response, cid)
            )
        )
        self.status_text = 'sending target to Nav2'

    def _on_nav_goal_response(self, future, command_id):
        if command_id != self.active_command_id:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_active_command('Nav2 goal request failed: %s' % exc)
            return
        if not goal_handle.accepted:
            self._fail_active_command('Nav2 rejected navigation goal')
            return
        self.nav_goal_handle = goal_handle
        self.status_text = 'Nav2 navigation active'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, cid=command_id: self._on_nav_result(result, cid)
        )

    def _on_nav_feedback(self, feedback_msg, command_id):
        if command_id != self.active_command_id:
            return
        distance = float(feedback_msg.feedback.distance_remaining)
        self.status_text = 'Nav2 active; %.1f m remaining' % distance
        now = time.monotonic()
        if now - self.last_feedback_ack < 1.0:
            return
        self.last_feedback_ack = now
        self._ack(
            self.active_command_id,
            CommandAck.STATUS_EXECUTING,
            self.status_text,
            max(0.05, min(0.95, 1.0 / (1.0 + distance))),
        )

    def _on_nav_result(self, future, command_id):
        if command_id != self.active_command_id:
            return
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
        except Exception as exc:
            self._fail_active_command('Nav2 result failed: %s' % exc)
            return
        self.active_command_id = ''
        self.active_target = None
        self.nav_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status_text = 'Nav2 navigation target reached'
            self._ack(
                command_id,
                CommandAck.STATUS_SUCCEEDED,
                self.status_text,
                1.0,
            )
        else:
            self.status_text = 'Nav2 navigation ended with status %d' % status
            self._ack(
                command_id,
                CommandAck.STATUS_FAILED,
                self.status_text,
            )

    def _fail_active_command(self, reason):
        command_id = self.active_command_id
        self.active_command_id = ''
        self.active_target = None
        self.nav_goal_handle = None
        self.status_text = reason
        self._ack(command_id, CommandAck.STATUS_FAILED, reason)

    def _start_lifecycle_request(self, node_name, operation, request):
        if operation == 'get':
            client = self.nav2_get_state_clients[node_name]
        else:
            client = self.nav2_change_state_clients[node_name]
        if not client.service_is_ready():
            return False
        self.nav2_lifecycle_future = client.call_async(request)
        self.nav2_lifecycle_operation = operation
        self.nav2_lifecycle_node = node_name
        self.nav2_lifecycle_request_started = time.monotonic()
        return True

    def _request_lifecycle_state(self, node_name):
        return self._start_lifecycle_request(
            node_name, 'get', GetState.Request()
        )

    def _request_lifecycle_transition(self, node_name, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        return self._start_lifecycle_request(node_name, 'change', request)

    def _retry_nav2_lifecycle(self, reason):
        self.get_logger().warning(
            'Nav2 lifecycle recovery for %s will retry: %s'
            % (self.vehicle_id, reason)
        )
        self.nav2_lifecycle_future = None
        self.nav2_lifecycle_operation = ''
        self.nav2_lifecycle_retry_after = (
            time.monotonic() + self.nav2_lifecycle_retry_period
        )

    def _advance_nav2_lifecycle(self):
        """Bring Nav2 nodes up sequentially without startup race conditions."""
        action_ready = self.nav_client.server_is_ready()
        self.nav2_ready = action_ready and (
            self.nav2_stack_activated or not self.manage_nav2_lifecycle
        )
        if self.nav2_ready:
            if not self.nav2_ready_logged:
                self.get_logger().info(
                    'Nav2 backend for %s is active and accepting goals'
                    % self.vehicle_id
                )
                self.nav2_ready_logged = True
            return

        self.nav2_ready_logged = False
        if not self.manage_nav2_lifecycle:
            return

        now = time.monotonic()
        if now < self.nav2_lifecycle_retry_after:
            return
        if self.nav2_lifecycle_future is not None:
            if not self.nav2_lifecycle_future.done():
                if (
                    now - self.nav2_lifecycle_request_started
                    > self.nav2_lifecycle_request_timeout
                ):
                    self._retry_nav2_lifecycle(
                        '%s request for %s timed out'
                        % (
                            self.nav2_lifecycle_operation,
                            self.nav2_lifecycle_node,
                        )
                    )
                return
            try:
                response = self.nav2_lifecycle_future.result()
            except Exception as exc:
                self._retry_nav2_lifecycle(str(exc))
                return

            node_name = self.nav2_lifecycle_node
            operation = self.nav2_lifecycle_operation
            self.nav2_lifecycle_future = None
            self.nav2_lifecycle_operation = ''
            if operation == 'change':
                if not response.success:
                    self._retry_nav2_lifecycle(
                        'transition rejected by %s' % node_name
                    )
                    return
                self.nav2_lifecycle_retry_after = now + 0.2
                return

            state_id = int(response.current_state.id)
            if state_id == State.PRIMARY_STATE_ACTIVE:
                self.nav2_lifecycle_index += 1
            elif state_id == State.PRIMARY_STATE_UNCONFIGURED:
                if self._request_lifecycle_transition(
                    node_name, Transition.TRANSITION_CONFIGURE
                ):
                    self.status_text = 'configuring Nav2 %s' % node_name
                return
            elif state_id == State.PRIMARY_STATE_INACTIVE:
                if self._request_lifecycle_transition(
                    node_name, Transition.TRANSITION_ACTIVATE
                ):
                    self.status_text = 'activating Nav2 %s' % node_name
                return
            else:
                self.nav2_lifecycle_retry_after = (
                    now + self.nav2_lifecycle_retry_period
                )
                return

        if self.nav2_lifecycle_index >= len(self.nav2_lifecycle_nodes):
            # The NavigateToPose action appears shortly after BT activation.
            self.nav2_stack_activated = True
            self.nav2_ready = self.nav_client.server_is_ready()
            self.nav2_lifecycle_retry_after = now + 0.5
            return

        node_name = self.nav2_lifecycle_nodes[self.nav2_lifecycle_index]
        if self._request_lifecycle_state(node_name):
            self.status_text = 'checking Nav2 %s' % node_name

    def _update(self):
        self._advance_nav2_lifecycle()
        self._send_pending_nav_goal()
        if self.emergency_stop:
            self.emergency_pub.publish(Twist())

        state = VehicleState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.vehicle_id = self.vehicle_id
        state.vehicle_type = VehicleState.TYPE_USV
        simulated_failure = bool(
            self.get_parameter('simulate_unreachable').value
        )
        state.online = (
            self.odom is not None
            and (self.nav2_ready or simulated_failure)
        )
        state.armed = True
        state.mode = 'NAV2' if self.nav2_ready else 'WAITING_NAV2'
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
