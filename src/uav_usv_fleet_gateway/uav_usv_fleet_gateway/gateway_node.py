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
        self.target_frame = ''
        self.perception_source = 'source_mux'
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
            'stale_timeout_sec': 3.0,
            'remove_timeout_sec': 30.0,
            'max_clients': 8,
            'client_queue_size': 100,
            'auto_remove_offline_vehicle': False,
            'enable_mqtt': False,
            'vehicle_state_topics': ['/fleet/state'],
            'perception_targets_topic': '/fleet/perception/targets',
            'sensor_status_topic': '/fleet/sensor_status',
            'mission_state_topic': '/capture/state',
            'perception_source_status_topic': '/perception/source_status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _float(self, name):
        return float(self.get_parameter(name).value)

    def _create_subscriptions(self):
        vehicle_topics = list(self.get_parameter(
            'vehicle_state_topics').value)
        for topic in vehicle_topics:
            self._gateway_subscriptions.append(self.create_subscription(
                VehicleState, str(topic), self._on_vehicle,
                qos_profile_sensor_data))
        targets_topic = str(self.get_parameter(
            'perception_targets_topic').value)
        sensor_topic = str(self.get_parameter('sensor_status_topic').value)
        mission_topic = str(self.get_parameter('mission_state_topic').value)
        source_topic = str(self.get_parameter(
            'perception_source_status_topic').value)
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
            'vehicle_state': vehicle_topics,
            'perception_targets': targets_topic,
            'sensor_status': sensor_topic,
            'mission_state': mission_topic,
            'perception_source_status': source_topic,
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
            'snapshot': [
                1.0 / max(0.1, self._float('snapshot_rate_hz')),
                self.broadcast_snapshot,
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
            'source': 'source_mux',
            'selected_source': self.perception_source,
            'targets': self.registry.targets(),
        })

    def _broadcast_sensor_status(self):
        for value in self.latest_sensors.values():
            priority = 2 if not value.get('online') else 1
            self._broadcast('sensor_status', value, priority=priority)

    def broadcast_snapshot(self):
        """Broadcast a complete recovery snapshot at 1 Hz."""
        self._broadcast('fleet_snapshot', self._snapshot(), priority=1)

    def _broadcast_diagnostics(self):
        self._broadcast(
            'gateway_diagnostics', self._diagnostics(), priority=1)

    def _diagnostics(self):
        vehicles = self.registry.vehicles()
        targets = self.registry.targets()
        clients = self.websocket.client_count if self.websocket else 0
        return self.health.snapshot(clients, vehicles, targets)

    def _snapshot(self):
        return self.registry.snapshot(self._diagnostics())

    def _hello(self):
        return {
            'gateway_name': self.gateway_name,
            'fleet_id': str(self.get_parameter('fleet_id').value),
            'protocol_version': '1.0',
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
        addresses = lan_addresses()
        self.get_logger().info('Mobile Fleet Demo started (read-only)')
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
        return super().destroy_node()


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
