"""Thread-safe-free, pure cache used by the ROS base-station node.

The cache deliberately treats Fleet World Model as immutable input.  It only
keeps display-oriented history and events in its own process state.
"""

from collections import deque
from copy import deepcopy
import math
import time


def _stamp_seconds(stamp):
    if not isinstance(stamp, dict):
        return None
    try:
        if 'seconds' in stamp:
            return float(stamp['seconds'])
        return float(stamp.get('sec', 0)) + float(
            stamp.get('nanosec', 0)
        ) * 1.0e-9
    except (TypeError, ValueError):
        return None


def _position(item):
    if not isinstance(item, dict):
        return None
    pose = item.get('pose') or {}
    value = pose.get('position') if isinstance(pose, dict) else None
    if value is None:
        value = item.get('position')
    if not isinstance(value, dict):
        return None
    try:
        return {
            'x': float(value.get('x', 0.0)),
            'y': float(value.get('y', 0.0)),
            'z': float(value.get('z', 0.0)),
        }
    except (TypeError, ValueError):
        return None


class BaseStationServiceCache:
    """Cache, history and event manager for one immutable world-model stream."""

    SCHEMA_VERSION = 'base_station_service.v1'

    def __init__(
        self,
        base_station_id='base_station',
        map_frame='map',
        position=None,
        orientation=None,
        radar_display_range_m=300.0,
        history_length=120,
        event_history_length=200,
        clock=time.time,
    ):
        self.base_station_id = str(base_station_id)
        self.map_frame = str(map_frame)
        self.position = dict(position or {'x': 0.0, 'y': 0.0, 'z': 0.0})
        self.orientation = dict(orientation or {
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        })
        self.radar_display_range_m = max(50.0, float(radar_display_range_m))
        self.history_length = max(2, int(history_length))
        self.clock = clock
        self.world_model = {}
        self.received_at = None
        self.sequence = 0
        self.target_history = {}
        self.events = deque(maxlen=max(1, int(event_history_length)))
        self._target_ids = set()
        self._threat_levels = {}
        self._vehicle_online = {}
        self._sensor_healthy = {}

    def update_world_model(self, world_model, received_at=None):
        """Cache one decoded Fleet World Model and return newly generated events."""
        if not isinstance(world_model, dict):
            raise ValueError('world model must be a JSON object')
        now = self.clock() if received_at is None else float(received_at)
        self.world_model = deepcopy(world_model)
        self.received_at = now
        self.sequence += 1
        events = []
        targets = list(world_model.get('targets') or [])
        current_target_ids = {
            str(item.get('id') or item.get('uuid'))
            for item in targets if item.get('id') or item.get('uuid')
        }
        for target_id in sorted(current_target_ids - self._target_ids):
            events.append(self._event('target_appeared', 'INFO', {
                'target_id': target_id,
                'old_value': None,
                'new_value': 'present',
            }, now))
        for target_id in sorted(self._target_ids - current_target_ids):
            events.append(self._event('target_lost', 'WARNING', {
                'target_id': target_id,
                'old_value': 'present',
                'new_value': None,
            }, now))
        self._target_ids = current_target_ids
        self._update_target_history(targets, world_model, now)
        events.extend(self._update_threat_events(world_model, now))
        events.extend(self._update_mission_events(world_model, now))
        events.extend(self._update_vehicle_events(world_model, now))
        events.extend(self._update_sensor_events(world_model, now))
        self.events.extend(events)
        return events

    def _update_target_history(self, targets, world_model, received_at):
        default_stamp = _stamp_seconds(world_model.get('world_time'))
        if default_stamp is None:
            default_stamp = received_at
        active = set()
        for target in targets:
            target_id = target.get('id') or target.get('uuid')
            position = _position(target)
            if not target_id or position is None:
                continue
            target_id = str(target_id)
            active.add(target_id)
            timestamp = _stamp_seconds(target.get('timestamp')) or default_stamp
            velocity = target.get('velocity', {}).get('linear', {})
            heading = target.get('heading_rad')
            try:
                point = {
                    'position': position,
                    'heading_rad': None if heading is None else float(heading),
                    'velocity': {
                        'x': float(velocity.get('x', 0.0)),
                        'y': float(velocity.get('y', 0.0)),
                        'z': float(velocity.get('z', 0.0)),
                    },
                    'timestamp': float(timestamp),
                }
            except (AttributeError, TypeError, ValueError):
                continue
            history = self.target_history.setdefault(target_id, [])
            previous = history[-1] if history else None
            if previous is None or self._history_point_changed(previous, point):
                history.append(point)
                del history[:-self.history_length]
        for target_id in list(self.target_history):
            if target_id not in active and target_id not in self._target_ids:
                del self.target_history[target_id]

    @staticmethod
    def _history_point_changed(previous, current):
        old = previous['position']
        new = current['position']
        distance = math.sqrt(
            (old['x'] - new['x']) ** 2 + (old['y'] - new['y']) ** 2
            + (old['z'] - new['z']) ** 2
        )
        return distance >= 0.05 or current['timestamp'] > previous['timestamp'] + 0.2

    def _update_threat_events(self, world_model, now):
        events = []
        levels = {}
        for threat in world_model.get('threats') or []:
            target_id = threat.get('target_id')
            if not target_id:
                continue
            levels[str(target_id)] = str(
                threat.get('threat_level') or 'UNKNOWN'
            ).upper()
        for target_id, level in sorted(levels.items()):
            if self._threat_levels.get(target_id) != level:
                severity = 'WARNING' if level in ('HIGH', 'CRITICAL') else 'INFO'
                events.append(self._event('threat_changed', severity, {
                    'target_id': target_id,
                    'previous_level': self._threat_levels.get(target_id),
                    'threat_level': level,
                    'old_value': self._threat_levels.get(target_id),
                    'new_value': level,
                }, now))
        self._threat_levels = levels
        return events

    def _update_mission_events(self, world_model, now):
        mission = world_model.get('mission') or {}
        capture = mission.get('capture') or {}
        current = str(capture.get('state') or 'UNKNOWN')
        previous = getattr(self, '_mission_state', None)
        self._mission_state = current
        if previous is None or previous == current:
            return []
        return [self._event('mission_changed', 'INFO', {
            'previous_state': previous,
            'mission_state': current,
            'old_value': previous,
            'new_value': current,
        }, now)]

    def _update_vehicle_events(self, world_model, now):
        events = []
        current = {}
        fleet = world_model.get('fleet') or {}
        for group in ('uav', 'usv', 'unknown'):
            for vehicle in fleet.get(group) or []:
                vehicle_id = vehicle.get('id')
                if vehicle_id:
                    current[str(vehicle_id)] = bool(vehicle.get('online'))
        for vehicle_id, online in sorted(current.items()):
            old = self._vehicle_online.get(vehicle_id)
            if old is not None and old != online:
                events.append(self._event(
                    'vehicle_online' if online else 'vehicle_offline',
                    'INFO' if online else 'WARNING',
                    {
                        'vehicle_id': vehicle_id,
                        'online': online,
                        'old_value': old,
                        'new_value': online,
                    }, now,
                ))
        self._vehicle_online = current
        return events

    def _update_sensor_events(self, world_model, now):
        events = []
        current = {}
        for vehicle_id, sensors in (world_model.get('sensors') or {}).items():
            for sensor_id, sensor in (sensors or {}).items():
                key = '%s/%s' % (vehicle_id, sensor_id)
                healthy = bool(sensor.get('healthy')) and not bool(
                    sensor.get('timed_out')
                )
                current[key] = healthy
                old = self._sensor_healthy.get(key)
                if old is not None and old != healthy:
                    events.append(self._event(
                        'sensor_online' if healthy else 'sensor_offline',
                        'INFO' if healthy else 'WARNING',
                        {
                            'vehicle_id': vehicle_id,
                            'sensor_id': sensor_id,
                            'healthy': healthy,
                            'old_value': old,
                            'new_value': healthy,
                        },
                        now,
                    ))
        self._sensor_healthy = current
        return events

    def _event(self, event_type, severity, payload, timestamp):
        entity_id = (
            payload.get('target_id')
            or payload.get('vehicle_id')
            or payload.get('entity_id')
            or 'fleet'
        )
        return {
            'schema_version': 'base_station_event.v1',
            'event_type': event_type,
            'severity': severity,
            'timestamp': float(timestamp),
            'entity_id': str(entity_id),
            'payload': payload,
        }

    def _client_target(self, target, now):
        """Normalize a World Model target for read-only shore clients."""
        result = deepcopy(target) if isinstance(target, dict) else {}
        target_id = result.get('id') or result.get('uuid') or 'unknown'
        position = _position(result) or {'x': 0.0, 'y': 0.0, 'z': 0.0}
        velocity = result.get('velocity') or {}
        linear = velocity.get('linear') if isinstance(velocity, dict) else {}
        linear = linear if isinstance(linear, dict) else {}
        source_stream = str(result.get('source_stream') or 'unknown')
        generic_sources = result.get('source') or []
        if not isinstance(generic_sources, list):
            generic_sources = [str(generic_sources)]
        result.update({
            'target_id': str(target_id),
            'id': str(target_id),
            'frame_id': self.map_frame,
            'position': position,
            'velocity': {
                'linear': {
                    'x': float(linear.get('x', 0.0)),
                    'y': float(linear.get('y', 0.0)),
                    'z': float(linear.get('z', 0.0)),
                },
            },
            'heading_rad': float(result.get('heading_rad') or 0.0),
            'dimensions': deepcopy(result.get('dimensions') or {}),
            'classification': result.get('classification') or result.get('class') or 'UNKNOWN',
            'affiliation': result.get('affiliation') or 'UNKNOWN',
            'confidence': float(result.get('confidence') or 0.0),
            'sources': generic_sources,
            'source_mask': int(result.get('source_mask') or 0),
            'last_update': deepcopy(result.get('last_update') or result.get('timestamp') or {}),
            'age_seconds': float(result.get('stamp_age_seconds') or 0.0),
            'is_ground_truth_fallback': source_stream == 'ground_truth',
            'is_real_fusion': source_stream == 'fused_targets',
        })
        return result

    def state(self, communication_status='LOCAL_SIMULATION'):
        """Return a client-facing snapshot without mutating the source model."""
        model = deepcopy(self.world_model)
        fleet = model.get('fleet') or {}
        vehicles = [
            item for group in ('uav', 'usv', 'unknown')
            for item in fleet.get(group) or []
        ]
        online_ids = [
            str(item.get('id')) for item in vehicles if item.get('online')
        ]
        capture = (model.get('mission') or {}).get('capture') or {}
        world_time = deepcopy(model.get('world_time') or {})
        now = self.clock()
        targets = [
            self._client_target(target, now)
            for target in (model.get('targets') or [])
        ]
        perception = deepcopy(model.get('perception') or {})
        perception.setdefault('primary_source', 'UNKNOWN')
        perception['fused_target_count'] = len(
            (perception.get('fused_targets') or {}).get('objects') or []
        )
        perception['usv_track_count'] = len(
            (perception.get('usv_tracks') or {}).get('objects') or []
        )
        perception['source_status'] = {
            'primary_source': perception.get('primary_source'),
            'ground_truth_control_fallback': (
                perception.get('primary_source') == 'ground_truth'
            ),
        }
        return {
            'schema_version': self.SCHEMA_VERSION,
            'sequence': self.sequence,
            'timestamp': {
                'seconds': _stamp_seconds(world_time) or now,
                'frame_id': model.get('map_frame') or self.map_frame,
            },
            'received_at_wall_time': self.received_at,
            'source_world_time': world_time,
            'map_frame': model.get('map_frame') or self.map_frame,
            'base_station': {
                'id': self.base_station_id,
                'frame_id': model.get('map_frame') or self.map_frame,
                'position': deepcopy(self.position),
                'orientation': deepcopy(self.orientation),
                'radar_display_range_m': self.radar_display_range_m,
                'communication_status': communication_status,
                'connected_vehicle_ids': [
                    str(item.get('id')) for item in vehicles if item.get('id')
                ],
                'online_vehicle_ids': online_ids,
                'mission_status': capture.get('state') or 'UNKNOWN',
            },
            'fleet': deepcopy(fleet),
            'entities': deepcopy(model.get('entities') or []),
            'targets': targets,
            'target_history': deepcopy(self.target_history),
            'predictions': deepcopy(model.get('predictions') or []),
            'threats': deepcopy(model.get('threats') or []),
            'mission': deepcopy(model.get('mission') or {}),
            'perception': perception,
            'sensors': deepcopy(model.get('sensors') or {}),
            'communication': deepcopy(model.get('communication') or {}),
            'tf': deepcopy(model.get('tf') or {}),
            'environment': deepcopy(model.get('environment') or {}),
            'health': deepcopy(model.get('health') or {}),
            'recent_events': list(self.events),
            'source': {
                'topic_contract': 'fleet_world_model.v1',
                'world_model_schema': model.get('schema_version'),
                'available': bool(model),
                'read_only': True,
            },
        }
