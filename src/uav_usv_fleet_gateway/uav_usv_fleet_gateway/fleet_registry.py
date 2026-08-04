"""Thread-safe registry for latest fleet data."""

from copy import deepcopy
import threading
import time

from .models import FleetSnapshot
from .message_converter import sensor_from_world_model
from .message_converter import target_from_world_model
from .message_converter import vehicle_from_world_model


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
        self._entities = []
        self._world_model = {}
        self._world_model_summary = {}
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

    def update_world_model_summary(self, summary):
        with self._lock:
            self._world_model_summary = deepcopy(summary)

    def update_world_model(self, model, received_at=None):
        received_at = self.monotonic() if received_at is None else received_at
        with self._lock:
            self._world_model = deepcopy(model)
            self._vehicles = {}
            fleet = model.get('fleet') or {}
            for item in list(fleet.get('uav') or []):
                vehicle = vehicle_from_world_model(item, received_at)
                if vehicle.id:
                    self._vehicles[vehicle.id] = vehicle
            for item in list(fleet.get('usv') or []):
                vehicle = vehicle_from_world_model(item, received_at)
                if vehicle.id:
                    self._vehicles[vehicle.id] = vehicle
            for item in list(fleet.get('unknown') or []):
                vehicle = vehicle_from_world_model(item, received_at)
                if vehicle.id:
                    self._vehicles[vehicle.id] = vehicle

            self._targets = {}
            for item in list(model.get('targets') or []):
                target = target_from_world_model(
                    item, received_at, formal_source='fleet_world_model'
                )
                if target.track_id:
                    self._targets[target.track_id] = target

            self._sensors = {}
            for vehicle_id, sensors in (model.get('sensors') or {}).items():
                for sensor_id, item in (sensors or {}).items():
                    sensor_item = dict(item)
                    sensor_item.setdefault('vehicle_id', vehicle_id)
                    sensor_item.setdefault('sensor_id', sensor_id)
                    sensor = sensor_from_world_model(sensor_item, received_at)
                    if sensor.vehicle_id and sensor.sensor_id:
                        self._sensors[(sensor.vehicle_id, sensor.sensor_id)] = (
                            sensor
                        )

            self._entities = list(model.get('entities') or [])
            mission = deepcopy(model.get('mission') or {})
            mission.setdefault('source', 'fleet_world_model')
            self._mission = mission
            self._source_status = {
                'source': (
                    model.get('perception', {}).get('primary_source')
                    or 'fleet_world_model'
                ),
                'world_model_schema': model.get('schema_version'),
            }

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

    def entities(self):
        with self._lock:
            return deepcopy(self._entities)

    def world_model(self):
        with self._lock:
            return deepcopy(self._world_model)

    def world_model_summary(self):
        with self._lock:
            return deepcopy(self._world_model_summary)

    def snapshot(self, gateway=None, include_world_model=True):
        with self._lock:
            snapshot = FleetSnapshot(
                vehicles=self.vehicles(),
                targets=self.targets(),
                sensors=self.sensors(),
                entities=self.entities(),
                world_model=(
                    self.world_model() if include_world_model else {}
                ),
                mission={
                    **deepcopy(self._mission),
                    'perception_source': deepcopy(self._source_status),
                },
                gateway=deepcopy(gateway or {}),
            )
            return snapshot.public()
