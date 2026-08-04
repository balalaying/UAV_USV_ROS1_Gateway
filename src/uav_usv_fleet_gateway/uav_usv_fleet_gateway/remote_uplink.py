#!/usr/bin/env python3
"""Forward a local gateway stream to a remote relay using outbound sockets."""

import argparse
import signal
import threading

from .websocket_client import WebSocketClient


class RemoteUplink:
    def __init__(self, source_url, relay_url, reconnect_seconds=2.0):
        self.source_url = source_url
        self.relay_url = relay_url
        self.reconnect_seconds = float(reconnect_seconds)
        self.stop_event = threading.Event()
        self.forwarded = 0

    def stop(self, *_args):
        self.stop_event.set()

    def run(self):
        while not self.stop_event.is_set():
            source = WebSocketClient(self.source_url)
            relay = WebSocketClient(self.relay_url)
            try:
                relay.connect()
                source.connect()
                print(
                    'Fleet uplink connected: %s -> %s'
                    % (self.source_url, self.relay_url),
                    flush=True,
                )
                while not self.stop_event.is_set():
                    relay.send_text(source.receive())
                    self.forwarded += 1
            except (ConnectionError, OSError, ValueError) as error:
                if not self.stop_event.is_set():
                    print('Fleet uplink reconnecting: %s' % error, flush=True)
            finally:
                source.close()
                relay.close()
            self.stop_event.wait(self.reconnect_seconds)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--source-url', default='ws://127.0.0.1:8765/ws')
    parser.add_argument(
        '--relay-url', default='ws://127.0.0.1:9765/uplink')
    parser.add_argument('--reconnect-seconds', type=float, default=2.0)
    args = parser.parse_args()
    uplink = RemoteUplink(
        args.source_url, args.relay_url, args.reconnect_seconds)
    signal.signal(signal.SIGINT, uplink.stop)
    signal.signal(signal.SIGTERM, uplink.stop)
    uplink.run()


if __name__ == '__main__':
    main()
