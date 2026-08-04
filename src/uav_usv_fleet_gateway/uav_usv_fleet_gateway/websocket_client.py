"""Small dependency-free WebSocket client used by the outbound uplink."""

import base64
import hashlib
import os
import socket
import ssl
import struct
from urllib.parse import urlsplit


def _exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError('websocket closed')
        data.extend(chunk)
    return bytes(data)


class WebSocketClient:
    def __init__(self, url, timeout=10.0):
        self.url = str(url)
        self.timeout = float(timeout)
        self.socket = None

    def connect(self):
        parsed = urlsplit(self.url)
        if parsed.scheme not in ('ws', 'wss') or not parsed.hostname:
            raise ValueError('invalid websocket URL: %s' % self.url)
        port = parsed.port or (443 if parsed.scheme == 'wss' else 80)
        raw = socket.create_connection(
            (parsed.hostname, port), timeout=self.timeout)
        if parsed.scheme == 'wss':
            raw = ssl.create_default_context().wrap_socket(
                raw, server_hostname=parsed.hostname)
        path = parsed.path or '/'
        if parsed.query:
            path += '?' + parsed.query
        key = base64.b64encode(os.urandom(16)).decode('ascii')
        request = (
            'GET %s HTTP/1.1\r\n'
            'Host: %s:%d\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Sec-WebSocket-Version: 13\r\n'
            'Sec-WebSocket-Key: %s\r\n\r\n'
        ) % (path, parsed.hostname, port, key)
        raw.sendall(request.encode('ascii'))
        response = bytearray()
        while not response.endswith(b'\r\n\r\n'):
            response.extend(_exact(raw, 1))
            if len(response) > 16384:
                raw.close()
                raise ConnectionError('oversized websocket handshake')
        status = response.split(b'\r\n', 1)[0]
        if b' 101 ' not in status:
            raw.close()
            raise ConnectionError(status.decode('latin1', errors='replace'))
        expected = base64.b64encode(hashlib.sha1(
            (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode(
                'ascii')).digest())
        if expected.lower() not in response.lower():
            raw.close()
            raise ConnectionError('invalid websocket accept key')
        raw.settimeout(None)
        self.socket = raw
        return self

    def send_text(self, text):
        if self.socket is None:
            raise ConnectionError('websocket is not connected')
        payload = str(text).encode('utf-8')
        mask = os.urandom(4)
        header = bytearray([0x81])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        elif length <= 0xFFFF:
            header.append(0x80 | 126)
            header.extend(struct.pack('!H', length))
        else:
            header.append(0x80 | 127)
            header.extend(struct.pack('!Q', length))
        header.extend(mask)
        masked = bytes(
            value ^ mask[index % 4] for index, value in enumerate(payload))
        self.socket.sendall(bytes(header) + masked)

    def receive(self):
        if self.socket is None:
            raise ConnectionError('websocket is not connected')
        while True:
            first, second = _exact(self.socket, 2)
            opcode = first & 0x0F
            length = second & 0x7F
            if length == 126:
                length = struct.unpack('!H', _exact(self.socket, 2))[0]
            elif length == 127:
                length = struct.unpack('!Q', _exact(self.socket, 8))[0]
            mask = _exact(self.socket, 4) if second & 0x80 else b''
            payload = bytearray(_exact(self.socket, length))
            if mask:
                for index in range(length):
                    payload[index] ^= mask[index % 4]
            if opcode == 0x8:
                raise ConnectionError('websocket close frame')
            if opcode == 0x9:
                self._send_control(bytes(payload), 0xA)
                continue
            if opcode == 0x1:
                return bytes(payload).decode('utf-8')

    def _send_control(self, payload, opcode):
        mask = os.urandom(4)
        header = bytes([0x80 | opcode, 0x80 | len(payload)]) + mask
        masked = bytes(
            value ^ mask[index % 4] for index, value in enumerate(payload))
        self.socket.sendall(header + masked)

    def close(self):
        if self.socket is not None:
            try:
                self._send_control(b'', 0x8)
            except (ConnectionError, OSError):
                pass
            self.socket.close()
        self.socket = None
