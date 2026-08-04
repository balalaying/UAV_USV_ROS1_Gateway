from uav_usv_mission.behavior_policy import evaluate_fleet_behavior


def vehicle(vehicle_id, vehicle_type, x, y):
    return {
        'id': vehicle_id,
        'type': vehicle_type,
        'state_source': 'vehicle_state',
        'online': True,
        'stale': False,
        'pose': {'position': {'x': x, 'y': y, 'z': 0.0}},
    }


def base_model():
    return {
        'schema_version': 'fleet_world_model.v1',
        'map_frame': 'map',
        'fleet': {
            'uav': [
                vehicle('air_alpha', 'UAV', 0.0, 0.0),
                vehicle('air_beta', 'UAV', 20.0, 0.0),
            ],
            'usv': [
                vehicle('surface_alpha', 'USV', 0.0, 0.0),
                vehicle('surface_beta', 'USV', 80.0, 0.0),
            ],
        },
        'entities': [{
            'id': 'protected_asset',
            'type': 'PROTECTED_ASSET',
            'affiliation': 'FRIENDLY',
            'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
        }],
        'targets': [],
        'threats': [],
        'mission': {'capture': None, 'capture_roles': None},
    }


def test_no_target_recommends_search_without_control_commands():
    decision = evaluate_fleet_behavior(base_model())
    assert decision['behavior'] == 'SEARCH'
    assert decision['shadow_mode'] is True
    assert decision['control_enabled'] is False
    assert len(decision['recommendations']) == 4
    assert all(
        item['control_command'] is None
        for item in decision['recommendations']
    )


def test_high_threat_selects_nearest_surface_interceptor():
    model = base_model()
    model['targets'] = [{
        'id': 'hostile_vessel',
        'pose': {'position': {'x': 10.0, 'y': 0.0, 'z': 0.0}},
    }]
    model['threats'] = [{
        'target_id': 'hostile_vessel',
        'score': 0.90,
        'threat_level': 'CRITICAL',
        'distance_to_friendly_ship_m': 10.0,
    }]
    decision = evaluate_fleet_behavior(model)
    assert decision['behavior'] == 'DEFENSE'
    primary = next(
        item for item in decision['recommendations']
        if item['role'] == 'primary_interceptor'
    )
    assert primary['vehicle_id'] == 'surface_alpha'


def test_passive_capture_tracking_does_not_claim_capture_execution():
    model = base_model()
    model['targets'] = [{
        'id': 'hostile_vessel',
        'pose': {'position': {'x': 10.0, 'y': 0.0, 'z': 0.0}},
    }]
    model['threats'] = [{
        'target_id': 'hostile_vessel',
        'score': 0.65,
        'threat_level': 'HIGH',
        'distance_to_friendly_ship_m': 70.0,
    }]
    model['mission']['capture'] = {'state': 'TRACKING'}
    decision = evaluate_fleet_behavior(model)
    assert decision['behavior'] == 'DEFENSE'
    assert decision['reason'] == 'hostile_target_inside_defense_policy'


def test_active_capture_reuses_existing_dynamic_assignment():
    model = base_model()
    model['targets'] = [{
        'id': 'moving_target',
        'pose': {'position': {'x': 30.0, 'y': 5.0, 'z': 0.0}},
    }]
    model['mission']['capture'] = {'state': 'ENCIRCLING'}
    model['mission']['capture_roles'] = {
        'assignments': [{
            'vehicle_id': 'air_beta',
            'role': 'air_observer_02',
            'goal': {'position': {'x': 31.0, 'y': 12.0, 'z': 40.0}},
        }]
    }
    decision = evaluate_fleet_behavior(model)
    assert decision['behavior'] == 'CAPTURE'
    assert decision['recommendations'][0]['vehicle_id'] == 'air_beta'
    assert (
        decision['recommendations'][0]['action']
        == 'execute_existing_capture_assignment'
    )


def test_explicit_escort_request_is_read_from_world_model():
    model = base_model()
    model['mission']['behavior_request'] = {'behavior': 'ESCORT'}
    decision = evaluate_fleet_behavior(model)
    assert decision['behavior'] == 'ESCORT'
    assert decision['target_id'] == ''
    assert any(
        item['role'].startswith('surface_escort')
        for item in decision['recommendations']
    )


def test_invalid_world_model_is_degraded():
    decision = evaluate_fleet_behavior({
        'schema_version': 'unexpected',
        'map_frame': 'map',
    })
    assert decision['behavior'] == 'DEGRADED'
    assert decision['recommendations'] == []
