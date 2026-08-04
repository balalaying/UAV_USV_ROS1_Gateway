"""Pure Fleet World Model behavior policy.

The policy intentionally has no ROS dependencies. It converts one immutable
Fleet World Model snapshot into advisory assignments; command arbitration and
vehicle control remain outside this module.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional, Sequence


BEHAVIORS = (
    'WAITING',
    'SEARCH',
    'TRACK',
    'ESCORT',
    'DEFENSE',
    'CAPTURE',
    'RETURN',
    'DEGRADED',
)

ACTIVE_CAPTURE_STATES = {
    'APPROACHING',
    'ENCIRCLING',
    'HOLDING',
}


@dataclass(frozen=True)
class BehaviorPolicyConfig:
    default_behavior: str = 'SEARCH'
    defense_trigger_distance_m: float = 120.0
    high_threat_score: float = 0.55
    critical_threat_score: float = 0.80


def _position(item: Optional[Mapping[str, Any]]) -> Optional[Dict[str, float]]:
    if not item:
        return None
    value = item.get('pose', {}).get('position', item.get('position', {}))
    if not isinstance(value, Mapping):
        return None
    try:
        return {
            'x': float(value.get('x', 0.0)),
            'y': float(value.get('y', 0.0)),
            'z': float(value.get('z', 0.0)),
        }
    except (TypeError, ValueError):
        return None


def _distance(lhs: Optional[Mapping[str, Any]], rhs: Optional[Mapping[str, Any]]):
    left = _position(lhs)
    right = _position(rhs)
    if left is None or right is None:
        return math.inf
    return math.hypot(left['x'] - right['x'], left['y'] - right['y'])


def _fresh_vehicles(model: Mapping[str, Any], vehicle_type: str):
    key = 'uav' if vehicle_type == 'UAV' else 'usv'
    vehicles = model.get('fleet', {}).get(key, [])
    result = []
    for item in vehicles if isinstance(vehicles, Sequence) else []:
        if not isinstance(item, Mapping) or not item.get('id'):
            continue
        if bool(item.get('stale', False)):
            continue
        result.append(item)
    return sorted(result, key=lambda item: str(item.get('id')))


def _control_ready(item: Mapping[str, Any]):
    return (
        item.get('state_source') == 'vehicle_state'
        and bool(item.get('online', False))
        and not bool(item.get('stale', False))
    )


def _primary_target(model: Mapping[str, Any]):
    targets = [
        item for item in model.get('targets', [])
        if isinstance(item, Mapping)
    ]
    threats = [
        item for item in model.get('threats', [])
        if isinstance(item, Mapping)
    ]
    threats.sort(key=lambda item: float(item.get('score', 0.0)), reverse=True)
    if threats:
        target_id = str(threats[0].get('target_id', ''))
        target = next(
            (item for item in targets if str(item.get('id', '')) == target_id),
            None,
        )
        return target or {
            'id': target_id,
            'pose': {'position': threats[0].get('position', {})},
            'speed_mps': threats[0].get('speed_mps', 0.0),
        }, threats[0]
    return (targets[0], None) if targets else (None, None)


def _friendly_asset(model: Mapping[str, Any]):
    entities = [
        item for item in model.get('entities', [])
        if isinstance(item, Mapping)
    ]
    for item in entities:
        if (
            str(item.get('affiliation', '')).upper() == 'FRIENDLY'
            or str(item.get('type', '')).upper() == 'PROTECTED_ASSET'
        ):
            return item
    return None


def _goal_hint(reference, dx=0.0, dy=0.0, z=None):
    position = _position(reference)
    if position is None:
        return None
    return {
        'frame_id': 'map',
        'position': {
            'x': position['x'] + float(dx),
            'y': position['y'] + float(dy),
            'z': position['z'] if z is None else float(z),
        },
    }


def _recommendation(
    vehicle,
    role,
    action,
    target_id='',
    goal_hint=None,
    confidence=0.5,
):
    return {
        'vehicle_id': str(vehicle.get('id', '')),
        'vehicle_type': str(vehicle.get('type', 'UNKNOWN')),
        'control_ready': _control_ready(vehicle),
        'role': role,
        'action': action,
        'target_id': target_id,
        'goal_hint': goal_hint,
        'confidence': max(0.0, min(1.0, float(confidence))),
        'control_command': None,
    }


def _ordered_by_distance(vehicles, reference):
    return sorted(
        vehicles,
        key=lambda item: (_distance(item, reference), str(item.get('id', ''))),
    )


def _search_recommendations(uavs, usvs):
    result = []
    for index, vehicle in enumerate(uavs):
        result.append(_recommendation(
            vehicle,
            'wide_area_search_%02d' % (index + 1),
            'search_sector',
            confidence=0.65,
        ))
    for index, vehicle in enumerate(usvs):
        result.append(_recommendation(
            vehicle,
            'surface_patrol_%02d' % (index + 1),
            'patrol_watch_area',
            confidence=0.60,
        ))
    return result


def _track_recommendations(uavs, usvs, target, target_id):
    result = []
    for index, vehicle in enumerate(_ordered_by_distance(uavs, target)):
        result.append(_recommendation(
            vehicle,
            'primary_observer' if index == 0 else 'support_observer_%02d' % index,
            'observe_target',
            target_id,
            _goal_hint(target, z=35.0 + 4.0 * index),
            0.80,
        ))
    for index, vehicle in enumerate(_ordered_by_distance(usvs, target)):
        result.append(_recommendation(
            vehicle,
            'sensor_picket' if index == 0 else 'shadow_tracker_%02d' % index,
            'track_at_safe_distance',
            target_id,
            _goal_hint(target, dx=-25.0 - 8.0 * index),
            0.75,
        ))
    return result


def _escort_recommendations(uavs, usvs, friendly):
    friendly_id = str((friendly or {}).get('id', 'friendly_asset'))
    result = []
    for index, vehicle in enumerate(uavs):
        result.append(_recommendation(
            vehicle,
            'escort_overwatch_%02d' % (index + 1),
            'overwatch_friendly',
            friendly_id,
            _goal_hint(friendly, z=35.0 + 5.0 * index),
            0.75,
        ))
    for index, vehicle in enumerate(usvs):
        side = -1.0 if index % 2 == 0 else 1.0
        ring = 12.0 + 8.0 * (index // 2)
        result.append(_recommendation(
            vehicle,
            'surface_escort_%02d' % (index + 1),
            'escort_friendly',
            friendly_id,
            _goal_hint(friendly, dx=-ring, dy=side * ring),
            0.80,
        ))
    return result


def _defense_recommendations(uavs, usvs, target, target_id, friendly):
    result = []
    for index, vehicle in enumerate(_ordered_by_distance(uavs, target)):
        result.append(_recommendation(
            vehicle,
            'threat_observer' if index == 0 else 'defense_overwatch_%02d' % index,
            'maintain_threat_track',
            target_id,
            _goal_hint(target, z=38.0 + 5.0 * index),
            0.90,
        ))
    ordered_usvs = _ordered_by_distance(usvs, target)
    for index, vehicle in enumerate(ordered_usvs):
        if index == 0:
            role = 'primary_interceptor'
            action = 'intercept_threat'
            goal = _goal_hint(target)
        else:
            role = 'defense_screen_%02d' % index
            action = 'screen_friendly_asset'
            offset = 14.0 * index
            goal = _goal_hint(friendly or target, dy=offset)
        result.append(_recommendation(
            vehicle, role, action, target_id, goal, 0.90
        ))
    return result


def _capture_recommendations(model, uavs, usvs, target, target_id):
    roles = model.get('mission', {}).get('capture_roles')
    assignments = (
        roles.get('assignments', [])
        if isinstance(roles, Mapping) else []
    )
    vehicle_lookup = {
        str(item.get('id')): item for item in list(uavs) + list(usvs)
    }
    result = []
    for item in assignments if isinstance(assignments, Sequence) else []:
        if not isinstance(item, Mapping):
            continue
        vehicle = vehicle_lookup.get(str(item.get('vehicle_id', '')))
        if vehicle is None:
            continue
        result.append(_recommendation(
            vehicle,
            str(item.get('role', 'capture_participant')),
            'execute_existing_capture_assignment',
            target_id,
            {
                'frame_id': 'map',
                'position': item.get('goal', {}).get('position', {}),
            },
            0.95,
        ))
    if result:
        return result
    return _track_recommendations(uavs, usvs, target, target_id)


def _explicit_request(model):
    request = model.get('mission', {}).get('behavior_request')
    if isinstance(request, Mapping):
        request = request.get('behavior')
    request = str(request or '').upper()
    return request if request in BEHAVIORS else ''


def evaluate_fleet_behavior(
    model: Mapping[str, Any],
    config: BehaviorPolicyConfig = BehaviorPolicyConfig(),
):
    """Evaluate one Fleet World Model snapshot without producing commands."""
    schema = str(model.get('schema_version', '')) if model else ''
    map_frame = str(model.get('map_frame', '')) if model else ''
    if schema != 'fleet_world_model.v1' or map_frame != 'map':
        return {
            'schema_version': 'fleet_behavior_state.v1',
            'behavior': 'DEGRADED' if model else 'WAITING',
            'reason': (
                'invalid_world_model_contract' if model
                else 'waiting_for_world_model'
            ),
            'shadow_mode': True,
            'control_enabled': False,
            'publishes_fleet_command': False,
            'target_id': '',
            'threat': None,
            'fleet_readiness': {
                'uav_visible': 0,
                'usv_visible': 0,
                'uav_control_ready': 0,
                'usv_control_ready': 0,
            },
            'recommendations': [],
        }

    uavs = _fresh_vehicles(model, 'UAV')
    usvs = _fresh_vehicles(model, 'USV')
    target, threat = _primary_target(model)
    friendly = _friendly_asset(model)
    target_id = str((target or {}).get('id', ''))
    capture = model.get('mission', {}).get('capture')
    capture_state = (
        str(capture.get('state', '')).upper()
        if isinstance(capture, Mapping) else ''
    )
    request = _explicit_request(model)

    threat_score = float((threat or {}).get('score', 0.0))
    distance_to_friendly = (threat or {}).get(
        'distance_to_friendly_ship_m'
    )
    distance_is_close = (
        distance_to_friendly is not None
        and float(distance_to_friendly) <= config.defense_trigger_distance_m
    )

    if request in {'RETURN', 'ESCORT', 'SEARCH', 'TRACK'}:
        behavior = request
        reason = 'explicit_world_model_behavior_request'
    elif capture_state in ACTIVE_CAPTURE_STATES:
        behavior = 'CAPTURE'
        reason = 'existing_capture_mission_active'
    elif target and (
        threat_score >= config.critical_threat_score
        or (
            threat_score >= config.high_threat_score
            and distance_is_close
        )
    ):
        behavior = 'DEFENSE'
        reason = 'hostile_target_inside_defense_policy'
    elif target:
        behavior = 'TRACK'
        reason = 'target_available_below_defense_threshold'
    else:
        behavior = str(config.default_behavior).upper()
        if behavior not in BEHAVIORS:
            behavior = 'SEARCH'
        reason = 'no_current_target'

    if behavior == 'SEARCH':
        recommendations = _search_recommendations(uavs, usvs)
    elif behavior == 'TRACK':
        recommendations = _track_recommendations(
            uavs, usvs, target, target_id
        )
    elif behavior == 'ESCORT':
        recommendations = _escort_recommendations(uavs, usvs, friendly)
    elif behavior == 'DEFENSE':
        recommendations = _defense_recommendations(
            uavs, usvs, target, target_id, friendly
        )
    elif behavior == 'CAPTURE':
        recommendations = _capture_recommendations(
            model, uavs, usvs, target, target_id
        )
    elif behavior == 'RETURN':
        recommendations = [
            _recommendation(
                item, 'returning', 'return_to_home', confidence=0.80
            )
            for item in list(uavs) + list(usvs)
        ]
    else:
        recommendations = []

    return {
        'schema_version': 'fleet_behavior_state.v1',
        'behavior': behavior,
        'reason': reason,
        'shadow_mode': True,
        'control_enabled': False,
        'publishes_fleet_command': False,
        'target_id': target_id,
        'threat': {
            'score': threat_score,
            'level': str((threat or {}).get('threat_level', 'UNKNOWN')),
            'distance_to_friendly_ship_m': distance_to_friendly,
        } if threat else None,
        'fleet_readiness': {
            'uav_visible': len(uavs),
            'usv_visible': len(usvs),
            'uav_control_ready': sum(_control_ready(item) for item in uavs),
            'usv_control_ready': sum(_control_ready(item) for item in usvs),
        },
        'recommendations': recommendations,
    }
