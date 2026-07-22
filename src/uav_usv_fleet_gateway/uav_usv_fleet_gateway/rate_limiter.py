"""Latest-value rate limiting helpers."""

import threading


class LatestValueStore:
    def __init__(self):
        self._values = {}
        self._dirty = set()
        self._lock = threading.RLock()

    def update(self, key, value):
        with self._lock:
            self._values[key] = value
            self._dirty.add(key)

    def pop_dirty(self):
        with self._lock:
            keys = tuple(self._dirty)
            self._dirty.clear()
            return [self._values[key] for key in keys if key in self._values]

    def values(self):
        with self._lock:
            return list(self._values.values())
