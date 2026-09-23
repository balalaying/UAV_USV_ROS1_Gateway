#!/usr/bin/env python3
"""Source-bound TCP hop relay for the TX/RX semantic-box chain."""

import argparse
import select
import socket
import socketserver
import time


def connect_from(source_host, target_host, target_port, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(timeout)
        sock.bind((source_host, 0))
        sock.connect((target_host, target_port))
        sock.settimeout(None)
        return sock
    except Exception:
        sock.close()
        raise


class RelayHandler(socketserver.BaseRequestHandler):
    def handle(self):
        source_ip = self.client_address[0]
        if self.server.allow_source and source_ip != self.server.allow_source:
            print("rejected connection from {}".format(source_ip), flush=True)
            return
        print(
            "accepted {} -> {}:{}".format(
                source_ip, self.server.server_address[0], self.server.server_address[1]
            ),
            flush=True,
        )
        try:
            target = connect_from(
                self.server.source_host,
                self.server.target_host,
                self.server.target_port,
                self.server.connect_timeout,
            )
        except OSError as exc:
            print("next hop connection failed: {}".format(exc), flush=True)
            return

        started = time.monotonic()
        byte_count = 0
        sockets = [self.request, target]
        try:
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.request.settimeout(None)
            while True:
                readable, _, _ = select.select(sockets, [], [], 5.0)
                if not readable:
                    continue
                for source in readable:
                    data = source.recv(65536)
                    if not data:
                        return
                    destination = target if source is self.request else self.request
                    destination.sendall(data)
                    byte_count += len(data)
        except OSError as exc:
            print("relay connection ended: {}".format(exc), flush=True)
        finally:
            target.close()
            print(
                "connection closed: {} bytes in {:.1f}s".format(
                    byte_count, time.monotonic() - started
                ),
                flush=True,
            )


class RelayServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-host", required=True)
    parser.add_argument("--listen-port", type=int, default=5000)
    parser.add_argument("--allow-source", required=True)
    parser.add_argument("--source-host", required=True)
    parser.add_argument("--target-host", required=True)
    parser.add_argument("--target-port", type=int, default=5000)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--name", default="relay")
    return parser.parse_args()


def main():
    args = parse_args()
    with RelayServer((args.listen_host, args.listen_port), RelayHandler) as server:
        server.allow_source = args.allow_source
        server.source_host = args.source_host
        server.target_host = args.target_host
        server.target_port = args.target_port
        server.connect_timeout = args.connect_timeout
        print(
            "{}: {}:{} <- {} | {}:* -> {}:{}".format(
                args.name,
                args.listen_host,
                args.listen_port,
                args.allow_source,
                args.source_host,
                args.target_host,
                args.target_port,
            ),
            flush=True,
        )
        try:
            server.serve_forever(poll_interval=0.5)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
