import base64
import json
import os
import socket
import struct
from types import SimpleNamespace as NS

import pytest

import uav_usv_fleet_gateway.sensor_stream_adapter as adapter_module
from uav_usv_fleet_gateway.gateway_node import FleetGatewayNode
from uav_usv_fleet_gateway.protocol import ProtocolEncoder
from uav_usv_fleet_gateway.rate_limiter import LatestValueStore
from uav_usv_fleet_gateway.sensor_stream_adapter import SensorStreamAdapter
from uav_usv_fleet_gateway.sensor_stream_adapter import spectrum_payload
from uav_usv_fleet_gateway.websocket_server import FleetWebSocketServer


def _frame(sequence=100):
    return {
        'type': 'spectrum',
        'captured_at': 1790228140.14739,
        'start_hz': 2399948790.93684,
        'stop_hz': 2500046445.56842,
        'bin_hz': 61035.1552631579,
        'rbw_hz': 100000,
        'ref_level_dbm': 0,
        'temperature_c': 39.07,
        'peak_hz': 2461289121.97632,
        'peak_dbm': -56.71,
        'powers_dbm': [-91.25, -87.5, -56.71],
        'sequence': sequence,
    }


def _adapt(frame):
    return spectrum_payload(
        json.dumps(frame), 'uav_01', 'san60', 'uav_01_san60')


def test_spectrum_payload_preserves_frame_and_adds_stream_identity():
    frame = _frame()
    adapted = _adapt(frame)
    for field, value in frame.items():
        assert adapted[field] == value
    assert adapted['powers_dbm'] == frame['powers_dbm']
    assert adapted['sequence'] == 100
    assert adapted['message_type'] == 'spectrum_frame'
    assert adapted['vehicle_id'] == 'uav_01'
    assert adapted['sensor_id'] == 'san60'
    assert adapted['stream_id'] == 'uav_01_san60'
    assert 'payload' not in adapted


@pytest.mark.parametrize('mutate', [
    lambda frame: frame.update(type='not_spectrum'),
    lambda frame: frame.pop('powers_dbm'),
    lambda frame: frame.update(powers_dbm=[]),
    lambda frame: frame.update(bin_hz=0),
    lambda frame: frame.update(stop_hz=frame['start_hz'] - 1),
])
def test_spectrum_payload_rejects_invalid_frames(mutate):
    frame = _frame()
    mutate(frame)
    with pytest.raises(ValueError):
        _adapt(frame)


def test_spectrum_payload_rejects_invalid_json():
    with pytest.raises(json.JSONDecodeError):
        spectrum_payload('{bad', 'uav_01', 'san60', 'uav_01_san60')


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message, **kwargs):
        self.warnings.append((message, kwargs))


def _callback_adapter():
    adapter = SensorStreamAdapter.__new__(SensorStreamAdapter)
    adapter.last_publish = adapter_module.defaultdict(float)
    adapter.rates = {'san60': 10.0}
    adapter.san60_vehicle_id = 'uav_01'
    adapter.san60_sensor_id = 'san60'
    adapter.san60_stream_id = 'uav_01_san60'
    adapter.published = []
    adapter.logger = _Logger()
    adapter._publish = adapter.published.append
    adapter.get_logger = lambda: adapter.logger
    return adapter


def test_spectrum_callback_limits_100_hz_input_to_about_10_hz(monkeypatch):
    adapter = _callback_adapter()
    clock = [100.0]
    monkeypatch.setattr(adapter_module.time, 'monotonic', lambda: clock[0])
    for sequence in range(100):
        clock[0] = 100.0 + sequence * 0.01
        adapter._spectrum(NS(data=json.dumps(_frame(sequence))))
    assert 9 <= len(adapter.published) <= 10
    sequences = [item['sequence'] for item in adapter.published]
    assert sequences == sorted(sequences)
    assert any(right - left > 1
               for left, right in zip(sequences, sequences[1:]))
    assert not adapter.logger.warnings


def test_spectrum_callback_rate_limits_before_json_parse(monkeypatch):
    adapter = _callback_adapter()
    adapter._ready = lambda *_args: False

    def unexpected_parse(_value):
        raise AssertionError('rate-limited frame was parsed')

    monkeypatch.setattr(adapter_module.json, 'loads', unexpected_parse)
    adapter._spectrum(NS(data='{bad'))
    assert not adapter.published
    assert not adapter.logger.warnings


class _Health:
    def __init__(self):
        self.counts = {}

    def increment(self, name):
        self.counts[name] = self.counts.get(name, 0) + 1


def test_gateway_caches_latest_spectrum_and_builds_outer_envelope():
    gateway = FleetGatewayNode.__new__(FleetGatewayNode)
    gateway.latest_sensor_streams = LatestValueStore()
    gateway.health = _Health()
    first = _adapt(_frame(100))
    second = _adapt(_frame(110))
    gateway._on_sensor_stream(NS(data=json.dumps(first)))
    gateway._on_sensor_stream(NS(data=json.dumps(second)))

    key = 'spectrum_frame:uav_01_san60'
    assert list(gateway.latest_sensor_streams._values) == [key]
    cached = gateway.latest_sensor_streams.values()[0]
    assert cached['data']['sequence'] == 110
    assert cached['data']['powers_dbm'] == second['powers_dbm']
    assert 'message_type' not in cached['data']

    protocol = ProtocolEncoder(time_provider=lambda: 12.5)
    sent = []
    gateway._broadcast = lambda message_type, data, priority=0: sent.append(
        protocol.envelope(message_type, data))
    gateway._broadcast_sensor_streams()
    assert sent[0]['schema_version'] == '1.0'
    assert sent[0]['message_type'] == 'spectrum_frame'
    assert sent[0]['sequence'] == 1
    assert sent[0]['data']['sequence'] == 110
    assert sent[0]['data']['stream_id'] == 'uav_01_san60'


def _receive_websocket_frame(sock):
    first, second = sock.recv(2)
    length = second & 0x7f
    if length == 126:
        length = struct.unpack('!H', sock.recv(2))[0]
    elif length == 127:
        length = struct.unpack('!Q', sock.recv(8))[0]
    payload = bytearray()
    while len(payload) < length:
        payload.extend(sock.recv(length - len(payload)))
    return first & 0x0f, bytes(payload)


def _connect_websocket(port):
    sock = socket.create_connection(('127.0.0.1', port), timeout=2.0)
    key = base64.b64encode(os.urandom(16)).decode('ascii')
    request = (
        'GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\n'
        'Upgrade: websocket\r\nConnection: Upgrade\r\n'
        'Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: %s\r\n\r\n'
        % key
    )
    sock.sendall(request.encode('ascii'))
    response = bytearray()
    while not response.endswith(b'\r\n\r\n'):
        response.extend(sock.recv(1))
    assert b'101 Switching Protocols' in response
    return sock


def test_spectrum_frame_reaches_websocket_with_gateway_envelope():
    protocol = ProtocolEncoder(source='uav_usv_fleet_gateway')
    server = FleetWebSocketServer(
        '127.0.0.1', 0, '/ws', protocol,
        lambda: {}, lambda: {'vehicles': []})
    server.start()
    sock = _connect_websocket(server.port)
    try:
        _receive_websocket_frame(sock)  # gateway_hello
        _receive_websocket_frame(sock)  # fleet_snapshot
        adapted = _adapt(_frame(12345))
        message_type = adapted.pop('message_type')
        server.broadcast(protocol.dumps(message_type, adapted))
        message = json.loads(_receive_websocket_frame(sock)[1])
        assert message['schema_version'] == '1.0'
        assert message['message_type'] == 'spectrum_frame'
        assert message['sequence'] > 0
        assert message['data']['sequence'] == 12345
        assert message['data']['stream_id'] == 'uav_01_san60'
        assert message['data']['powers_dbm'] == _frame()['powers_dbm']
    finally:
        sock.close()
        server.stop()
