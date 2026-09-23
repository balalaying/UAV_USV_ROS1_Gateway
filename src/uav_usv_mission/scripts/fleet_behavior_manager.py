#!/usr/bin/env python3
"""Shadow behavior manager driven exclusively by Fleet World Model."""

import json
import time

import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.executors import ExternalShutdownException
from uav_usv_ros1_compat.node import Node
from uav_usv_ros1_compat.qos import DurabilityPolicy
from uav_usv_ros1_compat.qos import QoSProfile
from std_msgs.msg import String

from uav_usv_mission.behavior_policy import BehaviorPolicyConfig
from uav_usv_mission.behavior_policy import evaluate_fleet_behavior


class FleetBehaviorManager(Node):
    IMMEDIATE_TRANSITIONS = {'WAITING', 'DEGRADED', 'DEFENSE', 'CAPTURE'}

    def __init__(self):
        super().__init__('fleet_behavior_manager')
        self.declare_parameter('world_model_topic', '/fleet/world_model')
        self.declare_parameter(
            'behavior_state_topic', '/fleet/behavior/shadow_state'
        )
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('world_model_timeout_sec', 2.5)
        self.declare_parameter('default_behavior', 'SEARCH')
        self.declare_parameter('defense_trigger_distance_m', 120.0)
        self.declare_parameter('high_threat_score', 0.55)
        self.declare_parameter('critical_threat_score', 0.80)
        self.declare_parameter('transition_hold_seconds', 1.5)

        self.timeout = max(
            0.1, float(self.get_parameter('world_model_timeout_sec').value)
        )
        self.hold_seconds = max(
            0.0, float(self.get_parameter('transition_hold_seconds').value)
        )
        self.config = BehaviorPolicyConfig(
            default_behavior=str(
                self.get_parameter('default_behavior').value
            ),
            defense_trigger_distance_m=float(
                self.get_parameter('defense_trigger_distance_m').value
            ),
            high_threat_score=float(
                self.get_parameter('high_threat_score').value
            ),
            critical_threat_score=float(
                self.get_parameter('critical_threat_score').value
            ),
        )
        self.world_model = None
        self.world_model_received_at = None
        self.active_behavior = 'WAITING'
        self.candidate_behavior = 'WAITING'
        self.candidate_since = time.monotonic()
        self.active_decision = evaluate_fleet_behavior({}, self.config)
        self.sequence = 0

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            String,
            str(self.get_parameter('behavior_state_topic').value),
            qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('world_model_topic').value),
            self._on_world_model,
            10,
        )
        rate = max(0.5, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Shadow behavior manager reads %s and publishes %s at %.1f Hz; '
            'FleetCommand output is disabled'
            % (
                str(self.get_parameter('world_model_topic').value),
                str(self.get_parameter('behavior_state_topic').value),
                rate,
            )
        )

    def _on_world_model(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning(
                'Rejected malformed Fleet World Model JSON',
                throttle_duration_sec=5.0,
            )
            return
        if not isinstance(payload, dict):
            return
        self.world_model = payload
        self.world_model_received_at = time.monotonic()

    def _transition(self, requested, now):
        if requested == self.active_behavior:
            self.candidate_behavior = requested
            self.candidate_since = now
            return
        if requested != self.candidate_behavior:
            self.candidate_behavior = requested
            self.candidate_since = now
        if (
            requested in self.IMMEDIATE_TRANSITIONS
            or now - self.candidate_since >= self.hold_seconds
        ):
            self.active_behavior = requested

    def _publish(self):
        now = time.monotonic()
        if self.world_model_received_at is None:
            age = None
            decision = evaluate_fleet_behavior({}, self.config)
        else:
            age = max(0.0, now - self.world_model_received_at)
            if age > self.timeout:
                decision = evaluate_fleet_behavior(
                    {'schema_version': 'stale'}, self.config
                )
                decision['reason'] = 'fleet_world_model_timeout'
            else:
                decision = evaluate_fleet_behavior(
                    self.world_model, self.config
                )

        requested_behavior = decision['behavior']
        self._transition(requested_behavior, now)
        if self.active_behavior == requested_behavior:
            self.active_decision = dict(decision)
        output = dict(self.active_decision)
        output['candidate_behavior'] = requested_behavior
        output['behavior'] = self.active_behavior
        output['transition_pending'] = (
            self.active_behavior != self.candidate_behavior
        )
        output['transition_hold_seconds'] = self.hold_seconds
        output['world_model_age_seconds'] = age
        output['sequence'] = self.sequence
        output['published_at'] = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )
        self.sequence += 1

        msg = String()
        msg.data = json.dumps(
            output, ensure_ascii=False, separators=(',', ':')
        )
        self.publisher.publish(msg)


def main(args=None):
    ros1.init(args=args)
    node = FleetBehaviorManager()
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
