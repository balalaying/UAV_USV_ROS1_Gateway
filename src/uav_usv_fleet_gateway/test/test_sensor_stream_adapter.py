import base64
import json
import os
import socket
import struct
from types import SimpleNamespace as NS

import pytest
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

import uav_usv_fleet_gateway.sensor_stream_adapter as adapter_module
from uav_usv_fleet_gateway.gateway_node import FleetGatewayNode
from uav_usv_fleet_gateway.protocol import ProtocolEncoder
from uav_usv_fleet_gateway.rate_limiter import LatestValueStore
from uav_usv_fleet_gateway.sensor_stream_adapter import SensorStreamAdapter
from uav_usv_fleet_gateway.sensor_stream_adapter import cloud_payload
from uav_usv_fleet_gateway.sensor_stream_adapter import encode_camera_payload
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


def _cloud(points, frame_id='map'):
    header = Header(
        stamp=rospy.Time.from_sec(1234567890.123),
        frame_id=frame_id,
    )
    return point_cloud2.create_cloud_xyz32(header, points)


def _adapt_cloud(points, maximum=5000):
    return cloud_payload(
        _cloud(points), 'usv_01', 'usv_01_mid360', maximum)


def test_pointcloud_payload_preserves_identity_and_flattens_xyz():
    payload = _adapt_cloud([
        (1.1234, 2.2344, 0.5004),
        (1.2234, 2.3344, 0.5104),
    ])

    assert payload['message_type'] == 'pointcloud_frame'
    assert payload['stream_id'] == 'usv_01_mid360'
    assert payload['vehicle_id'] == 'usv_01'
    assert payload['frame_id'] == 'map'
    assert payload['timestamp'] == pytest.approx(1234567890.123)
    assert payload['point_count'] == 2
    assert payload['xyz'] == [
        1.123, 2.234, 0.5,
        1.223, 2.334, 0.51,
    ]
    assert len(payload['xyz']) == payload['point_count'] * 3


def test_pointcloud_payload_skips_nan_and_inf():
    payload = _adapt_cloud([
        (1.0, 2.0, 3.0),
        (float('nan'), 4.0, 5.0),
        (6.0, float('inf'), 7.0),
        (8.0, 9.0, 10.0),
    ])

    assert payload['point_count'] == 2
    assert payload['xyz'] == [1.0, 2.0, 3.0, 8.0, 9.0, 10.0]
    assert len(payload['xyz']) == payload['point_count'] * 3


def test_pointcloud_payload_accepts_empty_cloud():
    payload = _adapt_cloud([])

    assert payload['point_count'] == 0
    assert payload['xyz'] == []


def test_pointcloud_payload_rejects_missing_xyz_field():
    header = Header(frame_id='map')
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
    ]
    message = point_cloud2.create_cloud(header, fields, [(1.0, 2.0)])

    with pytest.raises(ValueError, match='missing fields: z'):
        cloud_payload(message, 'usv_01', 'usv_01_mid360', 5000)


def test_pointcloud_payload_limits_points_with_even_sampling():
    payload = _adapt_cloud(
        [(float(index), index + 0.1, index + 0.2) for index in range(9)],
        maximum=3,
    )

    assert payload['point_count'] == 3
    assert len(payload['xyz']) == payload['point_count'] * 3
    assert payload['xyz'] == [
        0.0, 0.1, 0.2,
        4.0, 4.1, 4.2,
        8.0, 8.1, 8.2,
    ]


def test_camera_payload_regression_keeps_camera_frame():
    image = Image()
    image.header = Header(frame_id='usv_01/camera_link')
    image.height = 1
    image.width = 2
    image.encoding = 'bgr8'
    image.step = 6
    image.data = bytes([0, 0, 255, 0, 255, 0])

    payload = encode_camera_payload(
        image, 'usv_01', 'usv_01_front', quality=72, max_width=720)

    assert payload['message_type'] == 'camera_frame'
    assert payload['vehicle_id'] == 'usv_01'
    assert payload['stream_id'] == 'usv_01_front'
    assert payload['encoding'] == 'image/jpeg'
    assert payload['width'] == 2
    assert payload['height'] == 1
    assert payload['data_base64']


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
    adapter.rates = {'pointcloud': 5.0, 'san60': 10.0}
    adapter.san60_vehicle_id = 'uav_01'
    adapter.san60_sensor_id = 'san60'
    adapter.san60_stream_id = 'uav_01_san60'
    adapter.pointcloud_max_points = 5000
    adapter.published = []
    adapter.logger = _Logger()
    adapter._publish = adapter.published.append
    adapter.get_logger = lambda: adapter.logger
    return adapter


def test_pointcloud_callback_drops_malformed_frame_without_raising():
    adapter = _callback_adapter()
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
    ]
    malformed = point_cloud2.create_cloud(
        Header(frame_id='map'), fields, [(1.0, 2.0)])

    adapter._cloud(malformed, 'usv_01', 'usv_01_mid360')

    assert adapter.published == []
    assert len(adapter.logger.warnings) == 1
    assert 'missing fields: z' in adapter.logger.warnings[0][0]
    assert adapter.logger.warnings[0][1] == {
        'throttle_duration_sec': 5.0,
    }


def test_pointcloud_callback_drops_read_error_without_raising(monkeypatch):
    adapter = _callback_adapter()

    def fail_read(*_args, **_kwargs):
        raise RuntimeError('damaged cloud')

    monkeypatch.setattr(adapter_module.point_cloud2, 'read_points', fail_read)
    adapter._cloud(
        _cloud([(1.0, 2.0, 3.0)]), 'usv_01', 'usv_01_mid360')

    assert adapter.published == []
    assert len(adapter.logger.warnings) == 1
    assert (
        'PointCloud2 read failed: damaged cloud'
        in adapter.logger.warnings[0][0]
    )


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




def test_gateway_caches_pointcloud_and_builds_outer_envelope():
    gateway = FleetGatewayNode.__new__(FleetGatewayNode)
    gateway.latest_sensor_streams = LatestValueStore()
    gateway.health = _Health()
    pointcloud = _adapt_cloud([(1.0, 2.0, 3.0)])

    gateway._on_sensor_stream(NS(data=json.dumps(pointcloud)))

    key = 'pointcloud_frame:usv_01_mid360'
    assert list(gateway.latest_sensor_streams._values) == [key]
    cached = gateway.latest_sensor_streams.values()[0]
    assert cached['message_type'] == 'pointcloud_frame'
    assert cached['data']['frame_id'] == 'map'
    assert cached['data']['xyz'] == [1.0, 2.0, 3.0]
    assert 'message_type' not in cached['data']

    protocol = ProtocolEncoder(time_provider=lambda: 12.5)
    sent = []
    gateway._broadcast = lambda message_type, data, priority=0: sent.append(
        protocol.envelope(message_type, data))
    gateway._broadcast_sensor_streams()

    assert sent[0]['schema_version'] == '1.0'
    assert sent[0]['message_type'] == 'pointcloud_frame'
    assert sent[0]['data']['stream_id'] == 'usv_01_mid360'
    assert sent[0]['data']['vehicle_id'] == 'usv_01'
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
