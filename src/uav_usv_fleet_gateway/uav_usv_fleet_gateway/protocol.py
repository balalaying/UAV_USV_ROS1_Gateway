"""Versioned JSON protocol shared by all transports."""

import json
import threading
import time


SCHEMA_VERSION = '1.0'
SOURCE = 'uav_usv_fleet_gateway'


class ProtocolEncoder:
    def __init__(self, source=SOURCE, time_provider=time.time):
        self.source = source
        self.time_provider = time_provider
        self._sequence = 0
        self._lock = threading.Lock()

    @property
    def sequence(self):
        with self._lock:
            return self._sequence

    def envelope(self, message_type, data, timestamp=None):
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
        return {
            'schema_version': SCHEMA_VERSION,
            'message_type': str(message_type),
            'timestamp': float(
                self.time_provider() if timestamp is None else timestamp),
            'sequence': sequence,
            'source': self.source,
            'data': data,
        }

    def dumps(self, message_type, data, timestamp=None):
        return json.dumps(
            self.envelope(message_type, data, timestamp),
            ensure_ascii=False,
            separators=(',', ':'),
            allow_nan=False,
        )


def error_data(code, message):
    return {'code': str(code), 'message': str(message)}
