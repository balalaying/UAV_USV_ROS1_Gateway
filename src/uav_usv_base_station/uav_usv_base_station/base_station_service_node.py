#!/usr/bin/env python3
"""Read-only Base Station Service backed by /fleet/world_model."""

import json

import uav_usv_ros1_compat as ros1
from uav_usv_ros1_compat.executors import ExternalShutdownException
from uav_usv_ros1_compat.node import Node
from uav_usv_ros1_compat.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .service_cache import BaseStationServiceCache


class BaseStationServiceNode(Node):
    """Expose one cached client contract without changing Fleet World Model."""

    def __init__(self):
        super().__init__('base_station_service')
        self.declare_parameter('world_model_topic', '/fleet/world_model')
        self.declare_parameter('state_topic', '/base_station/state')
        self.declare_parameter('events_topic', '/base_station/events')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('target_history_length', 120)
        self.declare_parameter('event_history_length', 100)
        self.declare_parameter('base_station_id', 'base_station')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_station_x', 0.0)
        self.declare_parameter('base_station_y', 0.0)
        self.declare_parameter('base_station_z', 0.0)
        self.declare_parameter('base_station_roll', 0.0)
        self.declare_parameter('base_station_pitch', 0.0)
        self.declare_parameter('base_station_yaw', 0.0)
        self.declare_parameter('radar_display_range_m', 300.0)
        self.declare_parameter('communication_status', 'LOCAL_SIMULATION')

        self.communication_status = str(
            self.get_parameter('communication_status').value
        )
        self.cache = BaseStationServiceCache(
            base_station_id=str(
                self.get_parameter('base_station_id').value
            ),
            map_frame=str(self.get_parameter('map_frame').value),
            position={
                'x': float(self.get_parameter('base_station_x').value),
                'y': float(self.get_parameter('base_station_y').value),
                'z': float(self.get_parameter('base_station_z').value),
            },
            orientation={
                'roll': float(self.get_parameter('base_station_roll').value),
                'pitch': float(
                    self.get_parameter('base_station_pitch').value
                ),
                'yaw': float(self.get_parameter('base_station_yaw').value),
            },
            radar_display_range_m=float(
                self.get_parameter('radar_display_range_m').value
            ),
            history_length=int(
                self.get_parameter('target_history_length').value
            ),
            event_history_length=int(
                self.get_parameter('event_history_length').value
            ),
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), state_qos
        )
        self.events_pub = self.create_publisher(
            String, str(self.get_parameter('events_topic').value), 50
        )
        self.create_subscription(
            String,
            str(self.get_parameter('world_model_topic').value),
            self._on_world_model,
            10,
        )
        rate_hz = max(
            0.2, float(self.get_parameter('publish_rate_hz').value)
        )
        self.create_timer(1.0 / rate_hz, self._publish_state)
        self.get_logger().info(
            'Base Station Service started: %s -> %s'
            % (
                self.get_parameter('world_model_topic').value,
                self.get_parameter('state_topic').value,
            )
        )

    def _on_world_model(self, message):
        try:
            model = json.loads(message.data)
            if not isinstance(model, dict):
                raise ValueError('JSON root is not an object')
            events = self.cache.update_world_model(model)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                'Ignoring invalid Fleet World Model: %s' % error,
                throttle_duration_sec=5.0,
            )
            return
        for event in events:
            output = String()
            output.data = json.dumps(
                event, ensure_ascii=False, separators=(',', ':')
            )
            self.events_pub.publish(output)

    def _publish_state(self):
        output = String()
        output.data = json.dumps(
            self.cache.state(self.communication_status),
            ensure_ascii=False,
            separators=(',', ':'),
        )
        self.state_pub.publish(output)


def main(args=None):
    ros1.init(args=args)
    node = BaseStationServiceNode()
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
