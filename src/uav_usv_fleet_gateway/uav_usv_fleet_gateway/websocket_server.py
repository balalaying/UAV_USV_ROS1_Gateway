"""Small dependency-free, read-only RFC6455 text server."""

import base64
from collections import deque
import hashlib
import json
import socket
import socketserver
import struct
import threading


WEBSOCKET_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


def encode_frame(payload=b'', opcode=0x1):
    if isinstance(payload, str):
        payload = payload.encode('utf-8')
    length = len(payload)
    header = bytearray([0x80 | (opcode & 0x0F)])
    if length < 126:
        header.append(length)
    elif length <= 0xFFFF:
        header.append(126)
        header.extend(struct.pack('!H', length))
    else:
        header.append(127)
        header.extend(struct.pack('!Q', length))
    return bytes(header) + payload


def _read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            raise EOFError('websocket closed')
        data.extend(chunk)
    return bytes(data)


def read_frame(stream):
    header = _read_exact(stream, 2)
    first, second = header
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack('!H', _read_exact(stream, 2))[0]
    elif length == 127:
        length = struct.unpack('!Q', _read_exact(stream, 8))[0]
    if length > 1_048_576:
        raise ValueError('websocket frame exceeds 1 MiB')
    mask = _read_exact(stream, 4) if masked else b''
    payload = bytearray(_read_exact(stream, length))
    if masked:
        for index in range(length):
            payload[index] ^= mask[index % 4]
    return opcode, bytes(payload)


class BoundedMessageQueue:
    def __init__(self, maximum):
        self.maximum = max(1, int(maximum))
        self._items = deque()
        self._condition = threading.Condition()
        self.closed = False
        self.dropped = 0

    def put(self, payload, priority=0):
        with self._condition:
            if self.closed:
                return False
            if len(self._items) >= self.maximum:
                drop_index = next((
                    index for index, item in enumerate(self._items)
                    if item[0] < priority
                ), 0)
                del self._items[drop_index]
                self.dropped += 1
            self._items.append((int(priority), payload))
            self._condition.notify()
            return True

    def get(self, timeout=None):
        with self._condition:
            if not self._items and not self.closed:
                self._condition.wait(timeout)
            if self._items:
                return self._items.popleft()[1]
            return None

    def close(self):
        with self._condition:
            self.closed = True
            self._condition.notify_all()

    def __len__(self):
        with self._condition:
            return len(self._items)


class ClientConnection:
    def __init__(self, sock, address, queue_size, on_drop):
        self.sock = sock
        self.address = address
        self.queue = BoundedMessageQueue(queue_size)
        self.on_drop = on_drop
        self.active = True
        self._send_lock = threading.Lock()
        self._sender = threading.Thread(
            target=self._sender_loop, name='fleet-ws-sender', daemon=True)
        self._sender.start()

    def enqueue(self, text, priority=0):
        before = self.queue.dropped
        accepted = self.queue.put(text, priority)
        dropped = self.queue.dropped - before
        if dropped:
            self.on_drop(dropped)
        return accepted

    def send_control(self, payload, opcode):
        with self._send_lock:
            self.sock.sendall(encode_frame(payload, opcode))

    def _sender_loop(self):
        try:
            while self.active:
                payload = self.queue.get(1.0)
                if payload is None:
                    continue
                with self._send_lock:
                    self.sock.sendall(encode_frame(payload, 0x1))
        except (OSError, EOFError):
            self.active = False

    def close(self):
        self.active = False
        self.queue.close()
        try:
            self.send_control(b'', 0x8)
        except OSError:
            pass
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        if threading.current_thread() is not self._sender:
            self._sender.join(timeout=1.0)


class _ThreadingTcpServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


class _WebSocketHandler(socketserver.StreamRequestHandler):
    def handle(self):
        gateway = self.server.gateway
        try:
            path, headers = self._read_handshake()
            if path != gateway.path:
                self.wfile.write(b'HTTP/1.1 404 Not Found\r\n\r\n')
                return
            if headers.get('upgrade', '').lower() != 'websocket':
                self.wfile.write(b'HTTP/1.1 400 Bad Request\r\n\r\n')
                return
            key = headers.get('sec-websocket-key')
            if not key:
                self.wfile.write(b'HTTP/1.1 400 Bad Request\r\n\r\n')
                return
            if gateway.client_count >= gateway.max_clients:
                self.wfile.write(
                    b'HTTP/1.1 503 Service Unavailable\r\n\r\n')
                return
            digest = hashlib.sha1(
                (key + WEBSOCKET_GUID).encode('ascii')).digest()
            accept = base64.b64encode(digest).decode('ascii')
            response = (
                'HTTP/1.1 101 Switching Protocols\r\n'
                'Upgrade: websocket\r\nConnection: Upgrade\r\n'
                'Sec-WebSocket-Accept: %s\r\n\r\n' % accept
            )
            self.wfile.write(response.encode('ascii'))
            self.wfile.flush()
            client = gateway.register(self.request, self.client_address)
            if client is None:
                return
            while gateway.running and client.active:
                opcode, payload = read_frame(self.rfile)
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    client.send_control(payload, 0xA)
                elif opcode == 0x1:
                    gateway.handle_text(client, payload.decode('utf-8'))
        except (EOFError, OSError, ValueError, UnicodeDecodeError):
            pass
        finally:
            if 'client' in locals() and client is not None:
                gateway.unregister(client)

    def _read_handshake(self):
        line = self.rfile.readline(8192).decode('latin1').strip()
        parts = line.split()
        if len(parts) != 3 or parts[0] != 'GET':
            raise ValueError('invalid websocket request')
        headers = {}
        while True:
            line = self.rfile.readline(8192)
            if line in (b'\r\n', b'\n', b''):
                break
            name, value = line.decode('latin1').split(':', 1)
            headers[name.strip().lower()] = value.strip()
        return parts[1].split('?', 1)[0], headers


class FleetWebSocketServer:
    def __init__(self, host, port, path, protocol, hello_factory,
                 snapshot_factory, queue_size=100, max_clients=8,
                 heartbeat_interval=15.0, sent_callback=None,
                 drop_callback=None):
        self.host = str(host)
        self.port = int(port)
        self.path = str(path)
        self.protocol = protocol
        self.hello_factory = hello_factory
        self.snapshot_factory = snapshot_factory
        self.queue_size = int(queue_size)
        self.max_clients = int(max_clients)
        self.heartbeat_interval = float(heartbeat_interval)
        self.sent_callback = sent_callback or (lambda count: None)
        self.drop_callback = drop_callback or (lambda count: None)
        self.running = False
        self._clients = set()
        self._lock = threading.RLock()
        self._server = None
        self._thread = None

    @property
    def client_count(self):
        with self._lock:
            return len(self._clients)

    def start(self):
        if self.running:
            return
        server = _ThreadingTcpServer((self.host, self.port), _WebSocketHandler)
        server.gateway = self
        self.port = int(server.server_address[1])
        self._server = server
        self.running = True
        self._thread = threading.Thread(
            target=server.serve_forever, name='fleet-websocket', daemon=True)
        self._thread.start()

    def register(self, sock, address):
        with self._lock:
            if len(self._clients) >= self.max_clients:
                return None
            client = ClientConnection(
                sock, address, self.queue_size, self.drop_callback)
            # Queue the deterministic handshake messages before exposing this
            # client to concurrent periodic broadcasts.
            self.send_initial(client)
            self._clients.add(client)
            return client

    def unregister(self, client):
        with self._lock:
            self._clients.discard(client)
        client.close()

    def send_initial(self, client):
        self._send_client(client, self.protocol.dumps(
            'gateway_hello', self.hello_factory()), priority=2)
        self._send_client(client, self.protocol.dumps(
            'fleet_snapshot', self.snapshot_factory()), priority=2)

    def _send_client(self, client, text, priority=0):
        if client.enqueue(text, priority):
            self.sent_callback(1)
            return True
        return False

    def handle_text(self, client, text):
        try:
            request = json.loads(text)
        except (json.JSONDecodeError, TypeError):
            self._send_client(client, self.protocol.dumps('error', {
                'code': 'invalid_json',
                'message': 'Request is not valid JSON.',
            }), priority=2)
            return
        command = (
            request.get('command') if isinstance(request, dict) else None
        )
        if command == 'request_snapshot':
            self._send_client(client, self.protocol.dumps(
                'fleet_snapshot', self.snapshot_factory()), priority=2)
        elif command == 'ping':
            self._send_client(
                client, self.protocol.dumps('pong', {}), priority=2)
        else:
            self._send_client(client, self.protocol.dumps('error', {
                'code': 'unsupported_command',
                'message': 'The demo gateway is read-only.',
            }), priority=2)

    def broadcast(self, text, priority=0):
        with self._lock:
            clients = tuple(self._clients)
        sent = 0
        for client in clients:
            if client.active and client.enqueue(text, priority):
                sent += 1
        if sent:
            self.sent_callback(sent)
        return sent

    def stop(self):
        self.running = False
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        with self._lock:
            clients = tuple(self._clients)
            self._clients.clear()
        for client in clients:
            client.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._server = None
        self._thread = None
