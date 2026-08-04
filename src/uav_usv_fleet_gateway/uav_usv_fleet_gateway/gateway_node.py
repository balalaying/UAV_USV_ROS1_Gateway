#!/usr/bin/env python3
"""Read-only ROS 2 fleet data gateway."""

import json
from pathlib import Path
import socket
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from uav_usv_interfaces.msg import CaptureState
from uav_usv_interfaces.msg import SensorStatus
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState

from .fleet_registry import FleetRegistry
from .health_monitor import HealthMonitor
from .http_server import FleetHttpServer
from .message_converter import mission_from_ros
from .message_converter import sensor_from_ros
from .message_converter import target_from_ros
from .message_converter import vehicle_from_ros
from .protocol import ProtocolEncoder
from .rate_limiter import LatestValueStore
from .websocket_server import FleetWebSocketServer


def lan_addresses():
    addresses = set()
    try:
        output = subprocess.check_output(
            ['hostname', '-I'], text=True, timeout=1.0)
        for address in output.split():
            if ':' not in address and not address.startswith('127.'):
                addresses.add(address)
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None):
            address = item[4][0]
            if ':' not in address and not address.startswith('127.'):
                addresses.add(address)
    except OSError:
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(('8.8.8.8', 80))
        address = probe.getsockname()[0]
        if not address.startswith('127.'):
            addresses.add(address)
        probe.close()
    except OSError:
        pass
    return sorted(addresses)


def communication_profile(
    enable_world_model_publish,
    enable_world_model_summary_publish,
    include_world_model_in_snapshot,
    world_model_publish_rate_hz,
    world_model_summary_publish_rate_hz,
    world_model_topic='/fleet/world_model',
    world_model_summary_topic='/fleet/world_model_summary',
):
    """Summarize gateway bandwidth posture for shore clients."""
    full_enabled = bool(enable_world_model_publish)
    summary_enabled = bool(enable_world_model_summary_publish)
    include_full_snapshot = bool(include_world_model_in_snapshot)
    if full_enabled and include_full_snapshot:
        mode = 'local_full'
    elif summary_enabled and not full_enabled and not include_full_snapshot:
        mode = 'remote_summary'
    else:
        mode = 'custom'
    return {
        'mode': mode,
        'primary_data_source': 'fleet_world_model',
        'world_model_topic': str(world_model_topic),
        'world_model_summary_topic': str(world_model_summary_topic),
        'publish_world_model': full_enabled,
        'publish_world_model_summary': summary_enabled,
        'include_world_model_in_snapshot': include_full_snapshot,
        'world_model_publish_rate_hz': float(world_model_publish_rate_hz),
        'world_model_summary_publish_rate_hz': float(
            world_model_summary_publish_rate_hz
        ),
        'low_bandwidth_recommended': mode == 'remote_summary',
    }


class FleetGatewayNode(Node):
    def __init__(self):
        super().__init__('fleet_gateway')
        self._declare_parameters()
        self.gateway_name = str(self.get_parameter('gateway_name').value)
        self.protocol = ProtocolEncoder(source=self.gateway_name)
        self.health = HealthMonitor()
        self.registry = FleetRegistry(
            stale_timeout=self._float('stale_timeout_sec'),
            remove_timeout=self._float('remove_timeout_sec'),
            auto_remove=bool(self.get_parameter(
                'auto_remove_offline_vehicle').value),
        )
        self.latest_vehicles = LatestValueStore()
        self.latest_sensors = LatestValueStore()
        self.latest_sensor_streams = LatestValueStore()
        self.latest_world_model = {}
        self.latest_world_model_summary = {}
        self.latest_base_station_state = {}
        self.recent_base_station_events = []
        self.target_frame = ''
        self.perception_source = 'fleet_world_model'
        self.websocket = None
        self.http = None
        self._push_stop = threading.Event()
        self._push_thread = None
        self._gateway_subscriptions = []
        self._create_subscriptions()
        self._start_transports()
        self._start_push_loop()
        self._print_startup()

    def _declare_parameters(self):
        defaults = {
            'gateway_name': 'uav_usv_fleet_gateway',
            'fleet_id': 'demo',
            'bind_address': '0.0.0.0',
            'enable_websocket': True,
            'websocket_port': 8765,
            'websocket_path': '/ws',
            'enable_http_server': True,
            'http_port': 8080,
            'vehicle_publish_rate_hz': 10.0,
            'target_publish_rate_hz': 10.0,
            'snapshot_rate_hz': 1.0,
            'diagnostics_rate_hz': 1.0,
            'sensor_publish_rate_hz': 1.0,
            'sensor_stream_publish_rate_hz': 20.0,
            'enable_world_model_publish': True,
            'world_model_publish_rate_hz': 2.0,
            'enable_world_model_summary_publish': True,
            'world_model_summary_publish_rate_hz': 1.0,
            'include_world_model_in_snapshot': True,
            'stale_timeout_sec': 3.0,
            'remove_timeout_sec': 30.0,
            'max_clients': 8,
            'client_queue_size': 100,
            'auto_remove_offline_vehicle': False,
            'enable_mqtt': False,
            'world_model_topic': '/fleet/world_model',
            'world_model_summary_topic': '/fleet/world_model_summary',
            'enable_base_station_service': True,
            'base_station_state_topic': '/base_station/state',
            'base_station_events_topic': '/base_station/events',
            'enable_legacy_topic_fallback': False,
            'vehicle_state_topics': ['/fleet/state'],
            'perception_targets_topic': '/fleet/perception/targets',
            'sensor_status_topic': '/fleet/sensor_status',
            'sensor_stream_topic': '/fleet/gateway/sensor_stream',
            'mission_state_topic': '/capture/state',
            'perception_source_status_topic': '/perception/source_status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _float(self, name):
        return float(self.get_parameter(name).value)

    def _create_subscriptions(self):
        world_model_topic = str(self.get_parameter(
            'world_model_topic').value)
        world_model_summary_topic = str(self.get_parameter(
            'world_model_summary_topic').value)
        use_base_station_service = bool(self.get_parameter(
            'enable_base_station_service').value)
        if use_base_station_service:
            self._gateway_subscriptions.extend([
                self.create_subscription(
                    String,
                    str(self.get_parameter('base_station_state_topic').value),
                    self._on_base_station_state,
                    10,
                ),
                self.create_subscription(
                    String,
                    str(self.get_parameter('base_station_events_topic').value),
                    self._on_base_station_event,
                    50,
                ),
            ])
        else:
            self._gateway_subscriptions.extend([
                self.create_subscription(
                    String, world_model_topic, self._on_world_model, 10),
                self.create_subscription(
                    String, world_model_summary_topic,
                    self._on_world_model_summary, 10),
            ])
        vehicle_topics = list(self.get_parameter(
            'vehicle_state_topics').value)
        targets_topic = str(self.get_parameter(
            'perception_targets_topic').value)
        sensor_topic = str(self.get_parameter('sensor_status_topic').value)
        mission_topic = str(self.get_parameter('mission_state_topic').value)
        source_topic = str(self.get_parameter(
            'perception_source_status_topic').value)
        sensor_stream_topic = str(
            self.get_parameter('sensor_stream_topic').value)
        self._gateway_subscriptions.append(self.create_subscription(
            String, sensor_stream_topic, self._on_sensor_stream, 10))
        if bool(self.get_parameter('enable_legacy_topic_fallback').value):
            for topic in vehicle_topics:
                self._gateway_subscriptions.append(self.create_subscription(
                    VehicleState, str(topic), self._on_vehicle,
                    qos_profile_sensor_data))
            self._gateway_subscriptions.extend([
                self.create_subscription(
                    TrackedObjectArray, targets_topic, self._on_targets, 10),
                self.create_subscription(
                    SensorStatus, sensor_topic, self._on_sensor,
                    qos_profile_sensor_data),
                self.create_subscription(
                    CaptureState, mission_topic, self._on_mission, 10),
                self.create_subscription(String, source_topic,
                                         self._on_source_status, 10),
            ])
        self.topic_summary = {
            'base_station_service_enabled': use_base_station_service,
            'base_station_state': str(self.get_parameter(
                'base_station_state_topic').value),
            'base_station_events': str(self.get_parameter(
                'base_station_events_topic').value),
            'primary_world_model': world_model_topic,
            'world_model_summary': world_model_summary_topic,
            'legacy_fallback_enabled': bool(self.get_parameter(
                'enable_legacy_topic_fallback').value),
            'legacy_vehicle_state': vehicle_topics,
            'legacy_perception_targets': targets_topic,
            'legacy_sensor_status': sensor_topic,
            'sensor_stream': sensor_stream_topic,
            'legacy_mission_state': mission_topic,
            'legacy_perception_source_status': source_topic,
        }

    def _start_transports(self):
        bind = str(self.get_parameter('bind_address').value)
        if bool(self.get_parameter('enable_websocket').value):
            try:
                self.websocket = FleetWebSocketServer(
                    host=bind,
                    port=int(self.get_parameter('websocket_port').value),
                    path=str(self.get_parameter('websocket_path').value),
                    protocol=self.protocol,
                    hello_factory=self._hello,
                    snapshot_factory=self._snapshot,
                    queue_size=int(self.get_parameter(
                        'client_queue_size').value),
                    max_clients=int(self.get_parameter('max_clients').value),
                    sent_callback=lambda count: self.health.increment(
                        'sent_messages', count),
                    drop_callback=lambda count: self.health.increment(
                        'dropped_messages', count),
                )
                self.websocket.start()
                self.health.websocket_running = True
            except OSError as error:
                self.websocket = None
                self.health.websocket_running = False
                self.get_logger().error(
                    'WebSocket disabled after startup failure: %s' % error)
        if bool(self.get_parameter('enable_http_server').value):
            try:
                share = Path(get_package_share_directory(
                    'uav_usv_fleet_gateway'))
                self.http = FleetHttpServer(
                    bind, int(self.get_parameter('http_port').value),
                    share / 'web', self._http_health)
                self.http.start()
            except (OSError, FileNotFoundError) as error:
                self.http = None
                self.get_logger().error(
                    'HTTP server disabled after startup failure: %s' % error)

    def _start_push_loop(self):
        """Run WebSocket publication on wall time, independent of /clock."""
        self._push_stop.clear()
        self._push_thread = threading.Thread(
            target=self._push_loop,
            name='fleet-gateway-publisher',
            daemon=True,
        )
        self._push_thread.start()

    def _push_loop(self):
        tasks = {
            'vehicle': [
                1.0 / max(0.1, self._float('vehicle_publish_rate_hz')),
                self.broadcast_vehicle_state,
            ],
            'targets': [
                1.0 / max(0.1, self._float('target_publish_rate_hz')),
                self.broadcast_targets,
            ],
            'sensors': [
                1.0 / max(0.1, self._float('sensor_publish_rate_hz')),
                self._broadcast_sensor_status,
            ],
            'sensor_streams': [
                1.0 / max(
                    0.1, self._float('sensor_stream_publish_rate_hz')),
                self._broadcast_sensor_streams,
            ],
            'world_model': [
                1.0 / max(0.1, self._float('world_model_publish_rate_hz')),
                self.broadcast_world_model,
            ],
            'world_model_summary': [
                1.0 / max(
                    0.1,
                    self._float('world_model_summary_publish_rate_hz'),
                ),
                self.broadcast_world_model_summary,
            ],
            'snapshot': [
                1.0 / max(0.1, self._float('snapshot_rate_hz')),
                self.broadcast_snapshot,
            ],
            'base_station_snapshot': [
                1.0 / max(0.1, self._float('snapshot_rate_hz')),
                self.broadcast_base_station_snapshot,
            ],
            'diagnostics': [
                1.0 / max(0.1, self._float('diagnostics_rate_hz')),
                self._broadcast_diagnostics,
            ],
        }
        now = time.monotonic()
        deadlines = {name: now for name in tasks}
        while not self._push_stop.is_set():
            now = time.monotonic()
            for name, (interval, callback) in tasks.items():
                if now < deadlines[name]:
                    continue
                try:
                    callback()
                except Exception as error:  # Keep other streams alive.
                    self.get_logger().error(
                        'Gateway push %s failed: %s' % (name, error))
                deadlines[name] = now + interval
            next_deadline = min(deadlines.values())
            self._push_stop.wait(max(
                0.001, min(0.02, next_deadline - time.monotonic())))

    def _on_vehicle(self, message):
        now = time.monotonic()
        model = vehicle_from_ros(message, now)
        self.registry.update_vehicle(model)
        self.latest_vehicles.update(model.id, model.public())
        self.health.increment('received_messages')

    def _on_targets(self, message):
        now = time.monotonic()
        models = [target_from_ros(
            item, message.header, now, formal_source='source_mux')
            for item in message.objects]
        self.registry.update_targets(models)
        self.target_frame = str(message.header.frame_id)
        self.health.increment('received_messages')

    def _on_sensor(self, message):
        model = sensor_from_ros(message, time.monotonic())
        self.registry.update_sensor(model)
        self.latest_sensors.update(
            (model.vehicle_id, model.sensor_id), model.public())
        self.health.increment('received_messages')

    def _on_sensor_stream(self, message):
        """Cache the latest compressed frame without blocking a ROS callback."""
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.health.increment('dropped_messages')
            return
        if not isinstance(payload, dict):
            self.health.increment('dropped_messages')
            return
        message_type = str(payload.pop('message_type', 'sensor_stream'))
        stream_id = str(payload.get('stream_id') or payload.get(
            'vehicle_id') or message_type)
        self.latest_sensor_streams.update(
            '%s:%s' % (message_type, stream_id),
            {'message_type': message_type, 'data': payload},
        )
        self.health.increment('received_messages')

    def _on_mission(self, message):
        self.registry.update_mission(mission_from_ros(message))
        self.health.increment('received_messages')

    def _on_source_status(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            status = {
                'source': 'not_available',
                'raw_status': str(message.data),
            }
        self.registry.update_source_status(status)
        if status.get('source'):
            self.perception_source = str(status['source'])
        self.health.increment('received_messages')

    def _on_world_model(self, message):
        try:
            model = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Invalid Fleet World Model JSON')
            return
        if not isinstance(model, dict):
            return
        now = time.monotonic()
        self.registry.update_world_model(model, now)
        self.latest_world_model = model
        perception = model.get('perception') or {}
        self.perception_source = str(
            perception.get('primary_source') or 'fleet_world_model'
        )
        self.target_frame = str(model.get('map_frame') or 'map')
        self.health.increment('received_messages')

    def _on_base_station_state(self, message):
        """Cache the Base Station contract without recomputing situation."""
        try:
            state = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Invalid Base Station state JSON')
            return
        if not isinstance(state, dict) or state.get(
                'schema_version') != 'base_station_service.v1':
            self.get_logger().warning('Ignoring unsupported Base Station state')
            return
        now = time.monotonic()
        self.latest_base_station_state = state
        # Preserve legacy payloads as a view of the same immutable snapshot.
        self.registry.update_world_model(state, now)
        self.latest_world_model = state
        perception = state.get('perception') or {}
        self.perception_source = str(
            perception.get('primary_source') or 'base_station_service'
        )
        self.target_frame = str(state.get('map_frame') or 'map')
        self.health.increment('received_messages')

    def _on_base_station_event(self, message):
        try:
            event = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Invalid Base Station event JSON')
            return
        if not isinstance(event, dict) or event.get(
                'schema_version') != 'base_station_event.v1':
            return
        self.recent_base_station_events.append(event)
        del self.recent_base_station_events[:-100]
        self._broadcast('base_station_event', event, priority=2)
        self.health.increment('received_messages')

    def _on_world_model_summary(self, message):
        try:
            summary = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(summary, dict):
            self.registry.update_world_model_summary(summary)
            self.latest_world_model_summary = summary
            self.health.increment('received_messages')

    def _broadcast(self, message_type, data, priority=0):
        if self.websocket is None or not self.websocket.running:
            return
        self.websocket.broadcast(
            self.protocol.dumps(message_type, data), priority=priority)

    def broadcast_vehicle_state(self):
        """Broadcast the latest state of every registered vehicle at 10 Hz."""
        for value in self.registry.vehicles():
            alert = value.get('stale') or not value.get('online')
            priority = 2 if alert else 0
            self._broadcast('vehicle_state', value, priority=priority)

    def broadcast_targets(self):
        """Broadcast the latest formal perception target set at 10 Hz."""
        self._broadcast('perception_targets', {
            'frame_id': self.target_frame,
            'source': 'fleet_world_model',
            'selected_source': self.perception_source,
            'targets': self.registry.targets(),
        })

    def _broadcast_sensor_status(self):
        for value in self.registry.sensors():
            priority = 2 if not value.get('online') else 1
            self._broadcast('sensor_status', value, priority=priority)

    def _broadcast_sensor_streams(self):
        for item in self.latest_sensor_streams.pop_dirty():
            self._broadcast(
                item['message_type'], item['data'], priority=0)

    def broadcast_world_model(self):
        if not bool(self.get_parameter('enable_world_model_publish').value):
            return
        model = self.registry.world_model()
        if model:
            self._broadcast('fleet_world_model', model, priority=1)

    def broadcast_world_model_summary(self):
        if not bool(self.get_parameter(
                'enable_world_model_summary_publish').value):
            return
        summary = self.registry.world_model_summary()
        if summary:
            self._broadcast('fleet_world_model_summary', summary, priority=1)

    def broadcast_snapshot(self):
        """Broadcast a complete recovery snapshot at 1 Hz."""
        self._broadcast('fleet_snapshot', self._snapshot(), priority=1)

    def broadcast_base_station_snapshot(self):
        """Broadcast the authoritative shore-state payload at 1 Hz."""
        if self.latest_base_station_state:
            self._broadcast(
                'base_station_snapshot', self.latest_base_station_state,
                priority=1,
            )

    def _broadcast_diagnostics(self):
        self._broadcast(
            'gateway_diagnostics', self._diagnostics(), priority=1)

    def _diagnostics(self):
        vehicles = self.registry.vehicles()
        targets = self.registry.targets()
        clients = self.websocket.client_count if self.websocket else 0
        diagnostics = self.health.snapshot(clients, vehicles, targets)
        diagnostics['communication_profile'] = self._communication_profile()
        diagnostics['base_station_state_available'] = bool(
            self.latest_base_station_state)
        diagnostics['base_station_event_cache_size'] = len(
            self.recent_base_station_events)
        return diagnostics

    def _snapshot(self):
        snapshot = self.registry.snapshot(
            self._diagnostics(),
            include_world_model=bool(
                self.get_parameter('include_world_model_in_snapshot').value
            ),
        )
        snapshot['base_station_state'] = self.latest_base_station_state
        snapshot['recent_base_station_events'] = list(
            self.recent_base_station_events)
        return snapshot

    def _communication_profile(self):
        return communication_profile(
            self.get_parameter('enable_world_model_publish').value,
            self.get_parameter('enable_world_model_summary_publish').value,
            self.get_parameter('include_world_model_in_snapshot').value,
            self.get_parameter('world_model_publish_rate_hz').value,
            self.get_parameter('world_model_summary_publish_rate_hz').value,
            self.get_parameter('world_model_topic').value,
            self.get_parameter('world_model_summary_topic').value,
        )

    def _hello(self):
        return {
            'gateway_name': self.gateway_name,
            'fleet_id': str(self.get_parameter('fleet_id').value),
            'protocol_version': '1.0',
            'primary_data_source': (
                'base_station_service'
                if bool(self.get_parameter(
                    'enable_base_station_service').value)
                else 'fleet_world_model'
            ),
            'communication_profile': self._communication_profile(),
            'command_interface': {
                'enabled': False,
                'transport': 'websocket',
                'future_ros_output': '/fleet/command',
                'future_ack_input': '/fleet/command_ack',
                'accepted_commands': [
                    'submit_task',
                    'cancel_task',
                    'emergency_stop',
                ],
            },
            'use_sim_time': bool(self.get_parameter('use_sim_time').value),
            'websocket_path': str(
                self.get_parameter('websocket_path').value),
            'read_only': True,
        }

    def _http_health(self):
        diagnostics = self._diagnostics()
        return {
            'status': 'ok',
            'websocket': bool(
                self.websocket is not None and self.websocket.running),
            'ros_node': True,
            'clients': diagnostics['connected_clients'],
        }

    def _print_startup(self):
        bind = str(self.get_parameter('bind_address').value)
        addresses = (
            [bind] if bind not in ('0.0.0.0', '::') else lan_addresses()
        )
        self.get_logger().info('Fleet Gateway started (read-only)')
        self.get_logger().info('use_sim_time=%s' % bool(
            self.get_parameter('use_sim_time').value))
        self.get_logger().info('Subscriptions: %s' % json.dumps(
            self.topic_summary, ensure_ascii=False))
        if not addresses:
            self.get_logger().info(
                'Run `hostname -I` to find the LAN address.')
            addresses = ['<LAN-IP>']
        for address in addresses:
            if self.http:
                self.get_logger().info(
                    'HTTP: http://%s:%d' % (address, self.http.port))
            if self.websocket:
                self.get_logger().info(
                    'WebSocket: ws://%s:%d%s' % (
                        address, self.websocket.port, self.websocket.path))

    def destroy_node(self):
        self._push_stop.set()
        if self._push_thread is not None:
            self._push_thread.join(timeout=2.0)
            self._push_thread = None
        if self.http is not None:
            self.http.stop()
        if self.websocket is not None:
            self.websocket.stop()
        self.health.websocket_running = False
        try:
            return super().destroy_node()
        except KeyboardInterrupt:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = FleetGatewayNode()
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
