"""Gateway counters and health snapshot."""

import threading
import time


class HealthMonitor:
    def __init__(self, monotonic=time.monotonic):
        self.monotonic = monotonic
        self.started_at = monotonic()
        self.received_messages = 0
        self.sent_messages = 0
        self.dropped_messages = 0
        self.websocket_running = False
        self._lock = threading.Lock()

    def increment(self, name, count=1):
        with self._lock:
            setattr(self, name, int(getattr(self, name)) + int(count))

    def snapshot(self, connected_clients, vehicles, targets):
        online = sum(1 for item in vehicles if item['online'])
        stale = sum(1 for item in vehicles if item['stale'])
        with self._lock:
            return {
                'connected_clients': int(connected_clients),
                'registered_vehicles': len(vehicles),
                'online_vehicles': online,
                'stale_vehicles': stale,
                'target_count': len(targets),
                'received_messages': self.received_messages,
                'sent_messages': self.sent_messages,
                'dropped_messages': self.dropped_messages,
                'uptime_sec': round(self.monotonic() - self.started_at, 3),
                'websocket_running': bool(self.websocket_running),
            }
