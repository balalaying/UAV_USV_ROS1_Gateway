#!/usr/bin/env python3
"""Print concise SAN60 spectrum summaries from the Gateway JSON WebSocket."""

import argparse
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GATEWAY_SOURCE = REPOSITORY_ROOT / 'src' / 'uav_usv_fleet_gateway'
if str(GATEWAY_SOURCE) not in sys.path:
    sys.path.insert(0, str(GATEWAY_SOURCE))

from uav_usv_fleet_gateway.websocket_client import WebSocketClient  # noqa: E402


def _scaled(value, divisor, precision):
    try:
        return ('%.*f' % (precision, float(value) / divisor))
    except (TypeError, ValueError):
        return '?'


def spectrum_summary(message):
    """Return a short summary without expanding the powers_dbm array."""
    if not isinstance(message, dict):
        raise ValueError('Gateway message must be a JSON object')
    data = message.get('data')
    if not isinstance(data, dict):
        raise ValueError('spectrum_frame data must be a JSON object')
    powers = data.get('powers_dbm')
    points = len(powers) if isinstance(powers, list) else '?'
    return '\n'.join((
        (
            'spectrum_frame gateway_seq=%s san60_seq=%s '
            'vehicle=%s sensor=%s stream=%s'
        ) % (
            message.get('sequence', '?'),
            data.get('sequence', '?'),
            data.get('vehicle_id', '?'),
            data.get('sensor_id', '?'),
            data.get('stream_id', '?'),
        ),
        '  band=%s-%s MHz bin=%s kHz RBW=%s kHz' % (
            _scaled(data.get('start_hz'), 1e6, 3),
            _scaled(data.get('stop_hz'), 1e6, 3),
            _scaled(data.get('bin_hz'), 1e3, 3),
            _scaled(data.get('rbw_hz'), 1e3, 3),
        ),
        '  Temp=%s C Peak=%s MHz / %s dBm points=%s' % (
            _scaled(data.get('temperature_c'), 1.0, 1),
            _scaled(data.get('peak_hz'), 1e6, 3),
            _scaled(data.get('peak_dbm'), 1.0, 1),
            points,
        ),
    ))


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Connect to the Gateway JSON WebSocket and print only SAN60 '
            'spectrum_frame summaries.'
        ))
    parser.add_argument(
        '--url', default='ws://127.0.0.1:8765/ws',
        help='Gateway JSON WebSocket URL (default: %(default)s)')
    parser.add_argument(
        '--count', type=int, default=0,
        help='stop after this many spectrum frames; 0 means run forever')
    parser.add_argument(
        '--connect-timeout', type=float, default=5.0,
        help='TCP/WebSocket connection timeout in seconds')
    args = parser.parse_args()
    if args.count < 0:
        parser.error('--count must be zero or positive')

    client = WebSocketClient(args.url, timeout=args.connect_timeout)
    received = 0
    try:
        client.connect()
        print('Connected to %s; waiting for spectrum_frame ...' % args.url,
              file=sys.stderr, flush=True)
        while args.count == 0 or received < args.count:
            raw = client.receive()
            try:
                message = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
            if not isinstance(message, dict):
                continue
            if message.get('message_type') != 'spectrum_frame':
                continue
            try:
                print(spectrum_summary(message), flush=True)
            except ValueError as error:
                print('Invalid spectrum_frame: %s' % error,
                      file=sys.stderr, flush=True)
                continue
            received += 1
    except KeyboardInterrupt:
        return 130
    except (ConnectionError, OSError, ValueError) as error:
        print('SAN60 Gateway WebSocket check failed: %s' % error,
              file=sys.stderr)
        return 1
    finally:
        client.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
