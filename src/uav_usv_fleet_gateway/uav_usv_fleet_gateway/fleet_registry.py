"""Thread-safe registry for latest fleet data."""

from copy import deepcopy
import threading
import time

from .models import FleetSnapshot


class FleetRegistry:
    def __init__(self, stale_timeout=3.0, remove_timeout=30.0,
                 auto_remove=False, monotonic=time.monotonic):
        self.stale_timeout = float(stale_timeout)
        self.remove_timeout = float(remove_timeout)
        self.auto_remove = bool(auto_remove)
        self.monotonic = monotonic
        self._vehicles = {}
        self._targets = {}
        self._sensors = {}
        self._mission = {'state': 'not_available'}
        self._source_status = {'source': 'not_available'}
        self._lock = threading.RLock()

    def update_vehicle(self, model):
        with self._lock:
            self._vehicles[model.id] = model

    def update_targets(self, models):
        with self._lock:
            incoming = {item.track_id: item for item in models}
            self._targets = incoming

    def update_sensor(self, model):
        with self._lock:
            self._sensors[(model.vehicle_id, model.sensor_id)] = model

    def update_mission(self, mission):
        with self._lock:
            self._mission = deepcopy(mission)

    def update_source_status(self, status):
        with self._lock:
            self._source_status = deepcopy(status)

    def _age_vehicles(self, now):
        remove = []
        for vehicle_id, model in self._vehicles.items():
            age = max(0.0, now - model.received_at)
            model.stale = age > self.stale_timeout
            if model.stale:
                model.online = False
            if self.auto_remove and age > self.remove_timeout:
                remove.append(vehicle_id)
        for vehicle_id in remove:
            del self._vehicles[vehicle_id]

    def vehicles(self):
        with self._lock:
            self._age_vehicles(self.monotonic())
            return [item.public() for item in sorted(
                self._vehicles.values(), key=lambda value: value.id)]

    def targets(self):
        with self._lock:
            return [item.public() for item in sorted(
                self._targets.values(), key=lambda value: value.track_id)]

    def sensors(self):
        with self._lock:
            return [item.public() for item in sorted(
                self._sensors.values(),
                key=lambda value: (value.vehicle_id, value.sensor_id))]

    def snapshot(self, gateway=None):
        with self._lock:
            snapshot = FleetSnapshot(
                vehicles=self.vehicles(),
                targets=self.targets(),
                sensors=self.sensors(),
                mission={
                    **deepcopy(self._mission),
                    'perception_source': deepcopy(self._source_status),
                },
                gateway=deepcopy(gateway or {}),
            )
            return snapshot.public()
