#!/usr/bin/env python3
"""Public-side relay for fleet WebSocket data and the browser dashboard."""

import argparse
import asyncio
import json
from pathlib import Path
import signal
from urllib.parse import parse_qs, urlsplit

from .http_server import FleetHttpServer
from .websocket_server import ClientConnection
from .websocket_server import _read_async_frame


class FleetRemoteRelay:
    def __init__(
        self, host, port, web_root, http_host, http_port,
        token='', queue_size=80,
    ):
        self.host = str(host)
        self.port = int(port)
        self.http_host = str(http_host)
        self.http_port = int(http_port)
        self.web_root = Path(web_root)
        self.token = str(token)
        self.queue_size = int(queue_size)
        self.viewers = set()
        self.uplinks = set()
        self.cache = {}
        self.server = None
        self.http = None
        self.stop_event = asyncio.Event()
        self.received = 0
        self.forwarded = 0
        self.dropped = 0

    async def start(self):
        self.http = FleetHttpServer(
            self.http_host, self.http_port, self.web_root, self.health)
        self.http.start()
        self.http_port = self.http.port
        self.server = await asyncio.start_server(
            self._handle, self.host, self.port)
        self.port = int(self.server.sockets[0].getsockname()[1])
        print(
            'Remote relay dashboard: http://%s:%d' % (
                self.http_host, self.http_port),
            flush=True,
        )
        print(
            'Viewer WebSocket: ws://%s:%d/ws' % (self.host, self.port),
            flush=True,
        )
        print(
            'Outbound uplink target: ws://%s:%d/uplink' % (
                self.host, self.port),
            flush=True,
        )

    def health(self):
        return {
            'status': 'ok',
            'mode': 'remote_relay',
            'uplinks': len(self.uplinks),
            'viewers': len(self.viewers),
            'received': self.received,
            'forwarded': self.forwarded,
            'dropped': self.dropped,
        }

    async def _request(self, reader):
        line = (await reader.readline()).decode('latin1').strip()
        parts = line.split()
        if len(parts) != 3 or parts[0] != 'GET':
            raise ValueError('invalid websocket request')
        headers = {}
        while True:
            line = await reader.readline()
            if line in (b'\r\n', b'\n', b''):
                break
            name, value = line.decode('latin1').split(':', 1)
            headers[name.strip().lower()] = value.strip()
        return parts[1], headers

    async def _handshake(self, writer, target, headers):
        import base64
        import hashlib
        key = headers.get('sec-websocket-key')
        if headers.get('upgrade', '').lower() != 'websocket' or not key:
            writer.write(b'HTTP/1.1 400 Bad Request\r\n\r\n')
            await writer.drain()
            return False
        parsed = urlsplit(target)
        supplied = parse_qs(parsed.query).get('token', [''])[0]
        if self.token and supplied != self.token:
            writer.write(b'HTTP/1.1 401 Unauthorized\r\n\r\n')
            await writer.drain()
            return False
        accept = base64.b64encode(hashlib.sha1(
            (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode(
                'ascii')).digest()).decode('ascii')
        writer.write((
            'HTTP/1.1 101 Switching Protocols\r\n'
            'Upgrade: websocket\r\nConnection: Upgrade\r\n'
            'Sec-WebSocket-Accept: %s\r\n\r\n' % accept
        ).encode('ascii'))
        await writer.drain()
        return parsed.path

    async def _handle(self, reader, writer):
        client = None
        role_set = None
        try:
            target, headers = await self._request(reader)
            path = await self._handshake(writer, target, headers)
            if path not in ('/ws', '/uplink'):
                writer.close()
                await writer.wait_closed()
                return
            client = ClientConnection(
                writer, writer.get_extra_info('peername'),
                self.queue_size, self._drop)
            client.sender_task = asyncio.create_task(client.sender_loop())
            role_set = self.viewers if path == '/ws' else self.uplinks
            role_set.add(client)
            if path == '/ws':
                self._send_initial(client)
            while client.active:
                opcode, payload = await _read_async_frame(reader)
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    await client.send_frame(payload, 0xA)
                elif opcode == 0x1:
                    text = payload.decode('utf-8')
                    if path == '/uplink':
                        self._from_uplink(text)
                    else:
                        self._from_viewer(client, text)
        except (
            asyncio.IncompleteReadError, ConnectionError, OSError,
            UnicodeDecodeError, ValueError,
        ):
            pass
        finally:
            if role_set is not None and client is not None:
                role_set.discard(client)
            if client is not None:
                await client.close()

    def _drop(self, count):
        self.dropped += int(count)

    def _send_initial(self, client):
        order = ('gateway_hello', 'fleet_snapshot')
        for message_type in order:
            text = self.cache.get(message_type)
            if text:
                client.enqueue(text, priority=2)
        for key, text in self.cache.items():
            if key.startswith(('camera_frame:', 'pointcloud_frame:',
                               'fusion_debug:')):
                client.enqueue(text, priority=0)

    def _from_uplink(self, text):
        self.received += 1
        try:
            message = json.loads(text)
            message_type = str(message.get('message_type', 'unknown'))
            data = message.get('data') or {}
            stream_id = str(data.get('stream_id', ''))
            key = (
                message_type + ':' + stream_id
                if stream_id else message_type
            )
            self.cache[key] = text
        except (json.JSONDecodeError, TypeError):
            self.dropped += 1
            return
        priority = 1 if message_type in (
            'fleet_snapshot', 'gateway_diagnostics', 'sensor_status',
        ) else 0
        for viewer in tuple(self.viewers):
            if viewer.active and viewer.enqueue(text, priority=priority):
                self.forwarded += 1

    def _from_viewer(self, client, text):
        try:
            request = json.loads(text)
        except (json.JSONDecodeError, TypeError):
            return
        if request.get('command') == 'request_snapshot':
            snapshot = self.cache.get('fleet_snapshot')
            if snapshot:
                client.enqueue(snapshot, priority=2)

    async def run(self):
        await self.start()
        await self.stop_event.wait()
        self.server.close()
        await self.server.wait_closed()
        clients = tuple(self.viewers | self.uplinks)
        if clients:
            await asyncio.gather(
                *(client.close() for client in clients),
                return_exceptions=True)
        if self.http:
            self.http.stop()

    def stop(self):
        self.stop_event.set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind-address', default='127.0.0.1')
    parser.add_argument('--websocket-port', type=int, default=9765)
    parser.add_argument('--http-address', default='127.0.0.1')
    parser.add_argument('--http-port', type=int, default=9080)
    parser.add_argument('--web-root', required=True)
    parser.add_argument('--token', default='')
    args = parser.parse_args()

    relay = FleetRemoteRelay(
        args.bind_address, args.websocket_port, args.web_root,
        args.http_address, args.http_port, args.token)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(signum, relay.stop)
        except NotImplementedError:
            signal.signal(signum, lambda *_args: relay.stop())
    try:
        loop.run_until_complete(relay.run())
    finally:
        loop.close()


if __name__ == '__main__':
    main()
