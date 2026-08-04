from uav_usv_base_station.service_cache import BaseStationServiceCache


def world_model(targets=None, threats=None, capture_state='SEARCH'):
    return {
        'schema_version': 'fleet_world_model.v1',
        'world_time': {'seconds': 10.0},
        'map_frame': 'map',
        'fleet': {
            'uav': [{'id': 'uav_01', 'online': True}],
            'usv': [{'id': 'usv_01', 'online': True}],
            'unknown': [],
        },
        'targets': targets or [],
        'threats': threats or [],
        'sensors': {'usv_01': {'mid360': {'healthy': True, 'timed_out': False}}},
        'mission': {'capture': {'state': capture_state}},
        'entities': [],
        'predictions': [],
        'health': {},
    }


def target(x=1.0, y=2.0):
    return {
        'id': 'enemy_ship',
        'pose': {'position': {'x': x, 'y': y, 'z': 0.0}},
        'velocity': {'linear': {'x': 1.0, 'y': 0.0, 'z': 0.0}},
        'timestamp': {'seconds': 10.0},
    }


def test_cache_keeps_history_and_exposes_base_station_state():
    cache = BaseStationServiceCache(history_length=3, clock=lambda: 50.0)
    events = cache.update_world_model(world_model([target()]))
    assert events[0]['event_type'] == 'target_appeared'
    cache.update_world_model(world_model([target(3.0, 2.0)]))
    state = cache.state()
    assert state['schema_version'] == 'base_station_service.v1'
    assert state['base_station']['frame_id'] == 'map'
    assert state['base_station']['online_vehicle_ids'] == ['uav_01', 'usv_01']
    assert len(state['target_history']['enemy_ship']) == 2
    assert state['source']['read_only'] is True


def test_cache_emits_threat_mission_vehicle_and_sensor_changes():
    cache = BaseStationServiceCache(clock=lambda: 50.0)
    cache.update_world_model(world_model([target()]))
    changed = world_model(
        [target()],
        [{'target_id': 'enemy_ship', 'threat_level': 'HIGH'}],
        'APPROACHING',
    )
    changed['fleet']['usv'][0]['online'] = False
    changed['sensors']['usv_01']['mid360']['healthy'] = False
    events = cache.update_world_model(changed)
    event_types = {event['event_type'] for event in events}
    assert {'threat_changed', 'mission_changed', 'vehicle_offline', 'sensor_offline'} <= event_types
