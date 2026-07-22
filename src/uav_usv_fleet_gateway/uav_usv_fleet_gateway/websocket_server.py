"""Dependency-free asyncio RFC6455 server for read-only fleet data."""

import asyncio
import base64
import hashlib
import json
import struct
import threading


WEBSOCKET_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
MAX_FRAME_SIZE = 1_048_576


def encode_frame(payload=b'', opcode=0x1):
    """Encode one unmasked server frame."""
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
    """Synchronous frame reader retained for transport tests and tools."""
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
    if length > MAX_FRAME_SIZE:
        raise ValueError('websocket frame exceeds 1 MiB')
    mask = _read_exact(stream, 4) if masked else b''
    payload = bytearray(_read_exact(stream, length))
    if masked:
        for index in range(length):
            payload[index] ^= mask[index % 4]
    return opcode, bytes(payload)


async def _read_async_frame(reader):
    header = await reader.readexactly(2)
    first, second = header
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack('!H', await reader.readexactly(2))[0]
    elif length == 127:
        length = struct.unpack('!Q', await reader.readexactly(8))[0]
    if length > MAX_FRAME_SIZE:
        raise ValueError('websocket frame exceeds 1 MiB')
    mask = await reader.readexactly(4) if masked else b''
    payload = bytearray(await reader.readexactly(length))
    if masked:
        for index in range(length):
            payload[index] ^= mask[index % 4]
    return opcode, bytes(payload)


class AsyncClientQueue:
    """Bounded priority queue that discards old low-priority updates."""

    def __init__(self, maximum, on_drop=None):
        self.maximum = max(1, int(maximum))
        self.queue = asyncio.PriorityQueue(maxsize=self.maximum)
        self.on_drop = on_drop or (lambda count: None)
        self.serial = 0
        self.dropped = 0

    def put_nowait(self, payload, priority=0, opcode=0x1):
        self.serial += 1
        item = (-int(priority), self.serial, opcode, payload)
        if not self.queue.full():
            self.queue.put_nowait(item)
            return True

        buffered = []
        while not self.queue.empty():
            buffered.append(self.queue.get_nowait())
        incoming_priority = int(priority)
        priorities = [-entry[0] for entry in buffered]
        minimum = min(priorities)
        if incoming_priority < minimum:
            for entry in buffered:
                self.queue.put_nowait(entry)
            self._record_drop()
            return False

        drop_index = next(
            index for index, value in enumerate(priorities)
            if value == minimum)
        del buffered[drop_index]
        for entry in buffered:
            self.queue.put_nowait(entry)
        self.queue.put_nowait(item)
        self._record_drop()
        return True

    def _record_drop(self):
        self.dropped += 1
        self.on_drop(1)

    async def get(self):
        _priority, _serial, opcode, payload = await self.queue.get()
        return opcode, payload

    def __len__(self):
        return self.queue.qsize()


class ClientConnection:
    def __init__(self, writer, address, queue_size, on_drop):
        self.writer = writer
        self.address = address
        self.queue = AsyncClientQueue(queue_size, on_drop)
        self.active = True
        self.send_lock = asyncio.Lock()
        self.sender_task = None

    def enqueue(self, text, priority=0, opcode=0x1):
        if not self.active:
            return False
        return self.queue.put_nowait(text, priority, opcode)

    async def send_frame(self, payload, opcode=0x1):
        async with self.send_lock:
            self.writer.write(encode_frame(payload, opcode))
            await self.writer.drain()

    async def sender_loop(self):
        while self.active:
            opcode, payload = await self.queue.get()
            await self.send_frame(payload, opcode)

    async def close(self):
        if not self.active:
            return
        self.active = False
        if self.sender_task is not None:
            self.sender_task.cancel()
        try:
            await self.send_frame(b'', 0x8)
        except (ConnectionError, OSError):
            pass
        self.writer.close()
        try:
            await self.writer.wait_closed()
        except (ConnectionError, OSError):
            pass


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
        self._client_count = 0
        self._count_lock = threading.Lock()
        self._loop = None
        self._server = None
        self._thread = None
        self._ready = threading.Event()
        self._start_error = None

    @property
    def client_count(self):
        with self._count_lock:
            return self._client_count

    def _set_client_count(self):
        with self._count_lock:
            self._client_count = len(self._clients)

    def start(self):
        if self.running:
            return
        self._ready.clear()
        self._start_error = None
        self._thread = threading.Thread(
            target=self._event_loop_main,
            name='fleet-websocket-asyncio', daemon=True)
        self._thread.start()
        if not self._ready.wait(5.0):
            raise OSError('WebSocket asyncio loop startup timeout')
        if self._start_error is not None:
            raise OSError(str(self._start_error))

    def _event_loop_main(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._start_async())
        except OSError as error:
            self._start_error = error
            self._ready.set()
            self._loop.close()
            return
        self._ready.set()
        try:
            self._loop.run_forever()
        finally:
            pending = asyncio.all_tasks(self._loop)
            for task in pending:
                task.cancel()
            if pending:
                self._loop.run_until_complete(asyncio.gather(
                    *pending, return_exceptions=True))
            self._loop.close()

    async def _start_async(self):
        self._server = await asyncio.start_server(
            self._handle_client, self.host, self.port)
        self.port = int(self._server.sockets[0].getsockname()[1])
        self.running = True

    async def _read_handshake(self, reader):
        request_line = (await reader.readline()).decode('latin1').strip()
        parts = request_line.split()
        if len(parts) != 3 or parts[0] != 'GET':
            raise ValueError('invalid websocket request')
        headers = {}
        while True:
            line = await reader.readline()
            if line in (b'\r\n', b'\n', b''):
                break
            name, value = line.decode('latin1').split(':', 1)
            headers[name.strip().lower()] = value.strip()
        return parts[1].split('?', 1)[0], headers

    async def _reject(self, writer, status):
        writer.write(('HTTP/1.1 %s\r\nConnection: close\r\n\r\n'
                      % status).encode('ascii'))
        await writer.drain()
        writer.close()
        await writer.wait_closed()

    async def _handle_client(self, reader, writer):
        client = None
        try:
            path, headers = await self._read_handshake(reader)
            if path != self.path:
                await self._reject(writer, '404 Not Found')
                return
            if headers.get('upgrade', '').lower() != 'websocket':
                await self._reject(writer, '400 Bad Request')
                return
            key = headers.get('sec-websocket-key')
            if not key:
                await self._reject(writer, '400 Bad Request')
                return
            if len(self._clients) >= self.max_clients:
                await self._reject(writer, '503 Service Unavailable')
                return
            digest = hashlib.sha1(
                (key + WEBSOCKET_GUID).encode('ascii')).digest()
            accept = base64.b64encode(digest).decode('ascii')
            writer.write((
                'HTTP/1.1 101 Switching Protocols\r\n'
                'Upgrade: websocket\r\nConnection: Upgrade\r\n'
                'Sec-WebSocket-Accept: %s\r\n\r\n' % accept
            ).encode('ascii'))
            await writer.drain()

            client = ClientConnection(
                writer, writer.get_extra_info('peername'),
                self.queue_size, self.drop_callback)
            self._send_initial(client)
            self._clients.add(client)
            self._set_client_count()
            client.sender_task = asyncio.create_task(client.sender_loop())

            while self.running and client.active:
                opcode, payload = await _read_async_frame(reader)
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    await client.send_frame(payload, 0xA)
                elif opcode == 0x1:
                    self._handle_text(
                        client, payload.decode('utf-8'))
        except (
            asyncio.IncompleteReadError, ConnectionError, OSError,
            UnicodeDecodeError, ValueError,
        ):
            pass
        finally:
            if client is not None:
                self._clients.discard(client)
                self._set_client_count()
                await client.close()

    def _send_initial(self, client):
        self._send_client(client, self.protocol.dumps(
            'gateway_hello', self.hello_factory()), priority=2)
        self._send_client(client, self.protocol.dumps(
            'fleet_snapshot', self.snapshot_factory()), priority=2)

    def _send_client(self, client, text, priority=0):
        if client.enqueue(text, priority):
            self.sent_callback(1)
            return True
        return False

    def _handle_text(self, client, text):
        try:
            request = json.loads(text)
        except (json.JSONDecodeError, TypeError):
            self._send_client(client, self.protocol.dumps('error', {
                'code': 'invalid_json',
                'message': 'Request is not valid JSON.',
            }), priority=2)
            return
        command = (
            request.get('command') if isinstance(request, dict) else None)
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
        """Schedule a non-blocking broadcast from any ROS/background thread."""
        if not self.running or self._loop is None:
            return 0
        expected = self.client_count
        self._loop.call_soon_threadsafe(
            self._broadcast_in_loop, text, int(priority))
        return expected

    def _broadcast_in_loop(self, text, priority):
        sent = 0
        for client in tuple(self._clients):
            if client.active and client.enqueue(text, priority):
                sent += 1
        if sent:
            self.sent_callback(sent)

    async def _shutdown_async(self):
        self.running = False
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        clients = tuple(self._clients)
        self._clients.clear()
        self._set_client_count()
        if clients:
            await asyncio.gather(
                *(client.close() for client in clients),
                return_exceptions=True)
        self._server = None

    def stop(self):
        if self._loop is None:
            return
        if self._loop.is_running():
            future = asyncio.run_coroutine_threadsafe(
                self._shutdown_async(), self._loop)
            try:
                future.result(timeout=4.0)
            except (TimeoutError, RuntimeError):
                pass
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self.running = False
        self._thread = None
        self._loop = None
