import base64
import asyncio
import json
import os
from pathlib import Path
import socket
import struct
import tempfile
import time
from urllib.request import urlopen

from uav_usv_fleet_gateway.http_server import FleetHttpServer
from uav_usv_fleet_gateway.protocol import ProtocolEncoder
from uav_usv_fleet_gateway.websocket_server import AsyncClientQueue
from uav_usv_fleet_gateway.websocket_server import FleetWebSocketServer


def _masked_text(text):
    payload = text.encode('utf-8')
    mask = b'\x01\x02\x03\x04'
    masked = bytes(value ^ mask[index % 4]
                   for index, value in enumerate(payload))
    return bytes([0x81, 0x80 | len(payload)]) + mask + masked


def _receive_frame(sock):
    first, second = sock.recv(2)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack('!H', sock.recv(2))[0]
    elif length == 127:
        length = struct.unpack('!Q', sock.recv(8))[0]
    payload = bytearray()
    while len(payload) < length:
        payload.extend(sock.recv(length - len(payload)))
    return first & 0x0F, bytes(payload)


def _connect(port):
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


def test_bounded_queue_drops_oldest_normal_message():
    async def scenario():
        queue = AsyncClientQueue(2)
        queue.put_nowait('old')
        queue.put_nowait('middle')
        queue.put_nowait('important', priority=2)
        assert queue.dropped == 1
        assert await queue.get() == (0x1, 'important')
        assert await queue.get() == (0x1, 'middle')

    asyncio.run(scenario())


def test_async_queue_preserves_alert_when_position_queue_is_full():
    async def scenario():
        queue = AsyncClientQueue(2)
        queue.put_nowait('warning', priority=2)
        queue.put_nowait('snapshot', priority=1)
        assert queue.put_nowait('position', priority=0) is False
        assert await queue.get() == (0x1, 'warning')
        assert await queue.get() == (0x1, 'snapshot')

    asyncio.run(scenario())


def test_async_queue_replaces_old_position_with_latest_position():
    async def scenario():
        queue = AsyncClientQueue(2)
        queue.put_nowait('position-oldest', priority=0)
        queue.put_nowait('position-old', priority=0)
        assert queue.put_nowait('position-latest', priority=0) is True
        assert await queue.get() == (0x1, 'position-old')
        assert await queue.get() == (0x1, 'position-latest')

    asyncio.run(scenario())


def test_websocket_hello_snapshot_ping_and_invalid_json():
    protocol = ProtocolEncoder()
    server = FleetWebSocketServer(
        '127.0.0.1', 0, '/ws', protocol,
        lambda: {'read_only': True},
        lambda: {'vehicles': [], 'targets': []}, queue_size=4)
    server.start()
    sock = _connect(server.port)
    try:
        hello = json.loads(_receive_frame(sock)[1])
        snapshot = json.loads(_receive_frame(sock)[1])
        assert hello['message_type'] == 'gateway_hello'
        assert snapshot['message_type'] == 'fleet_snapshot'

        sock.sendall(_masked_text('{"command":"ping"}'))
        assert json.loads(_receive_frame(sock)[1])['message_type'] == 'pong'
        sock.sendall(_masked_text('{bad'))
        error = json.loads(_receive_frame(sock)[1])
        assert error['data']['code'] == 'invalid_json'
    finally:
        sock.close()
        server.stop()


def test_websocket_request_snapshot_and_read_only_error():
    server = FleetWebSocketServer(
        '127.0.0.1', 0, '/ws', ProtocolEncoder(),
        lambda: {}, lambda: {'vehicles': [{'id': 'uav_01'}]})
    server.start()
    sock = _connect(server.port)
    try:
        _receive_frame(sock)
        _receive_frame(sock)
        sock.sendall(_masked_text('{"command":"request_snapshot"}'))
        message = json.loads(_receive_frame(sock)[1])
        assert message['data']['vehicles'][0]['id'] == 'uav_01'
        sock.sendall(_masked_text('{"command":"arm"}'))
        message = json.loads(_receive_frame(sock)[1])
        assert message['data']['code'] == 'unsupported_command'
    finally:
        sock.close()
        server.stop()


def test_http_root_and_health():
    with tempfile.TemporaryDirectory() as directory:
        Path(directory, 'index.html').write_text('mobile-demo')
        server = FleetHttpServer(
            '127.0.0.1', 0, directory,
            lambda: {'status': 'ok', 'clients': 0})
        server.start()
        try:
            root = urlopen(
                'http://127.0.0.1:%d/' % server.port,
                timeout=2.0).read().decode()
            health = json.loads(urlopen(
                'http://127.0.0.1:%d/health' % server.port,
                timeout=2.0).read())
            assert root == 'mobile-demo'
            assert health['status'] == 'ok'
        finally:
            server.stop()


def test_websocket_client_disconnect_cleanup():
    server = FleetWebSocketServer(
        '127.0.0.1', 0, '/ws', ProtocolEncoder(), lambda: {}, lambda: {})
    server.start()
    sock = _connect(server.port)
    _receive_frame(sock)
    _receive_frame(sock)
    assert server.client_count == 1
    sock.close()
    deadline = time.time() + 2.0
    while server.client_count and time.time() < deadline:
        time.sleep(0.02)
    try:
        assert server.client_count == 0
    finally:
        server.stop()


def test_idle_websocket_client_stays_connected():
    server = FleetWebSocketServer(
        '127.0.0.1', 0, '/ws', ProtocolEncoder(), lambda: {}, lambda: {})
    server.start()
    sock = _connect(server.port)
    try:
        _receive_frame(sock)
        _receive_frame(sock)
        time.sleep(1.2)
        server.broadcast(ProtocolEncoder().dumps('gateway_diagnostics', {}))
        message = json.loads(_receive_frame(sock)[1])
        assert message['message_type'] == 'gateway_diagnostics'
        assert server.client_count == 1
    finally:
        sock.close()
        server.stop()


def test_multiple_clients_receive_hello_then_snapshot():
    server = FleetWebSocketServer(
        '127.0.0.1', 0, '/ws', ProtocolEncoder(),
        lambda: {}, lambda: {'vehicles': []})
    server.start()
    clients = [_connect(server.port), _connect(server.port)]
    try:
        for sock in clients:
            first = json.loads(_receive_frame(sock)[1])
            second = json.loads(_receive_frame(sock)[1])
            assert first['message_type'] == 'gateway_hello'
            assert second['message_type'] == 'fleet_snapshot'
        assert server.client_count == 2
    finally:
        for sock in clients:
            sock.close()
        server.stop()
