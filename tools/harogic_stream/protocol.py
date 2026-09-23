#!/usr/bin/env python3
"""Small framed protocol used by the HAROGIC spectrum stream."""

import json
import socket
import struct
import zlib


MAGIC = b"HSP1"
HEADER = struct.Struct("!4sI")
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024


class ProtocolError(RuntimeError):
    pass


def encode_frame(message):
    raw = json.dumps(
        message, ensure_ascii=False, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    payload = zlib.compress(raw, level=3)
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ProtocolError("compressed frame is too large")
    return HEADER.pack(MAGIC, len(payload)) + payload


def send_frame(sock, message):
    data = encode_frame(message)
    sock.sendall(data)
    return len(data)


def _recv_exact(sock, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("peer closed the connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_frame(sock):
    magic, payload_size = HEADER.unpack(_recv_exact(sock, HEADER.size))
    if magic != MAGIC:
        raise ProtocolError("invalid frame magic")
    if payload_size <= 0 or payload_size > MAX_PAYLOAD_BYTES:
        raise ProtocolError("invalid payload size: {}".format(payload_size))
    payload = _recv_exact(sock, payload_size)
    try:
        raw = zlib.decompress(payload)
        message = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeError, zlib.error) as exc:
        raise ProtocolError("invalid frame payload: {}".format(exc)) from exc
    if not isinstance(message, dict):
        raise ProtocolError("frame payload must be a JSON object")
    return message, HEADER.size + payload_size


def connect(host, port, timeout=5.0):
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(None)
    return sock
