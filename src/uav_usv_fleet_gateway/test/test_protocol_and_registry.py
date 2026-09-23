import math
from types import SimpleNamespace as NS

from uav_usv_fleet_gateway.fleet_registry import FleetRegistry
from uav_usv_fleet_gateway.gateway_node import communication_profile
from uav_usv_fleet_gateway.message_converter import quaternion_to_euler
from uav_usv_fleet_gateway.message_converter import target_from_ros
from uav_usv_fleet_gateway.message_converter import vehicle_from_ros
from uav_usv_fleet_gateway.models import TargetModel, VehicleModel
from uav_usv_fleet_gateway.protocol import ProtocolEncoder


def test_protocol_envelope_and_sequence():
    protocol = ProtocolEncoder(time_provider=lambda: 12.5)
    first = protocol.envelope('one', {})
    second = protocol.envelope('two', {})
    assert first['schema_version'] == '1.0'
    assert first['timestamp'] == 12.5
    assert second['sequence'] == first['sequence'] + 1


def test_communication_profile_modes():
    local = communication_profile(True, True, True, 2.0, 1.0)
    assert local['mode'] == 'local_full'
    assert local['publish_world_model'] is True
    assert local['low_bandwidth_recommended'] is False

    remote = communication_profile(False, True, False, 0.1, 1.0)
    assert remote['mode'] == 'remote_summary'
    assert remote['publish_world_model'] is False
    assert remote['publish_world_model_summary'] is True
    assert remote['low_bandwidth_recommended'] is True

    custom = communication_profile(True, True, False, 1.0, 1.0)
    assert custom['mode'] == 'custom'


def test_quaternion_to_euler_yaw():
    yaw = math.pi / 2.0
    roll, pitch, actual = quaternion_to_euler(
        0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    assert abs(roll) < 1e-6
    assert abs(pitch) < 1e-6
    assert actual == pytest.approx(yaw)


def _stamp(sec=1, nanosec=0):
    return NS(sec=sec, nanosec=nanosec)


def _pose(x=1.0, y=2.0, z=3.0):
    return NS(
        position=NS(x=x, y=y, z=z),
        orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _twist(x=0.0, y=0.0, z=0.0):
    return NS(
        linear=NS(x=x, y=y, z=z),
        angular=NS(x=0.0, y=0.0, z=0.0),
    )


def test_vehicle_conversion_preserves_null_battery():
    message = NS(
        header=NS(stamp=_stamp(5, 500_000_000), frame_id='map'),
        vehicle_id='uav_01', vehicle_type=1, online=True, armed=False,
        mode='GUIDED', pose=_pose(), twist=_twist(3.0, 4.0, 0.0),
        battery_percent=-1.0, active_command_id='', status_text='ready',
    )
    model = vehicle_from_ros(message, 10.0)
    assert model.id == 'uav_01'
    assert model.type == 'UAV'
    assert model.speed == 5.0
    assert model.battery['percentage'] is None
    assert model.battery['voltage'] is None
    assert model.last_update == 5.5


def test_target_array_conversion_fields():
    tracked = NS(
        track_id='target_01', last_update=_stamp(8),
        pose=NS(pose=_pose()), twist=NS(twist=_twist(1.0, 2.0, 0.0)),
        class_name='', classification=1, sensor_source='', source_mask=1,
        confidence=0.8, dimensions=NS(x=4.0, y=2.0, z=1.0),
    )
    model = target_from_ros(
        tracked, NS(stamp=_stamp(7), frame_id='map'), 9.0)
    assert model.track_id == 'target_01'
    assert model.class_name == 'vessel'
    assert model.sensor_source == 'lidar'
    assert model.bbox['length'] == 4.0


def test_registry_updates_same_vehicle_without_duplicates():
    clock = [0.0]
    registry = FleetRegistry(monotonic=lambda: clock[0])
    first = VehicleModel(id='usv_01', received_at=0.0)
    second = VehicleModel(id='usv_01', mode='NAV2', received_at=1.0)
    registry.update_vehicle(first)
    registry.update_vehicle(second)
    values = registry.vehicles()
    assert len(values) == 1
    assert values[0]['mode'] == 'NAV2'


def test_registry_stale_and_remove_timeout():
    clock = [0.0]
    registry = FleetRegistry(
        stale_timeout=2.0, remove_timeout=4.0, auto_remove=True,
        monotonic=lambda: clock[0])
    registry.update_vehicle(VehicleModel(
        id='usv_01', online=True, received_at=0.0))
    clock[0] = 3.0
    assert registry.vehicles()[0]['stale'] is True
    clock[0] = 5.0
    assert registry.vehicles() == []


def test_registry_replaces_target_set_by_track_id():
    registry = FleetRegistry()
    registry.update_targets([
        TargetModel(track_id='one'), TargetModel(track_id='two')])
    registry.update_targets([TargetModel(track_id='two', confidence=0.9)])
    targets = registry.targets()
    assert len(targets) == 1
    assert targets[0]['track_id'] == 'two'
    assert targets[0]['confidence'] == 0.9


def test_registry_accepts_fleet_world_model_snapshot():
    registry = FleetRegistry()
    registry.update_world_model({
        'schema_version': 'fleet_world_model.v1',
        'map_frame': 'map',
        'fleet': {
            'uav': [{
                'id': 'uav_01',
                'type': 'UAV',
                'state_source': 'tf_only',
                'online': False,
                'mode': 'TF_ONLY',
                'pose': {
                    'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                },
                'velocity': {
                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                },
                'header': {
                    'frame_id': 'map',
                    'stamp': {'sec': 10, 'nanosec': 0, 'seconds': 10.0},
                },
            }],
            'usv': [{
                'id': 'usv_01',
                'type': 'USV',
                'state_source': 'vehicle_state',
                'online': True,
                'armed': False,
                'mode': 'NAV2',
                'pose': {
                    'position': {'x': 4.0, 'y': 5.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                },
                'velocity': {
                    'linear': {'x': 1.0, 'y': 0.0, 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                },
                'header': {
                    'frame_id': 'map',
                    'stamp': {'sec': 10, 'nanosec': 0, 'seconds': 10.0},
                },
            }],
            'unknown': [],
        },
        'entities': [{
            'id': 'friendly_ship',
            'type': 'PROTECTED_ASSET',
            'affiliation': 'FRIENDLY',
        }],
        'targets': [{
            'id': 'enemy_ship',
            'classification': 'VESSEL',
            'class': 'VESSEL',
            'confidence': 0.8,
            'pose': {
                'position': {'x': 7.0, 'y': 8.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            },
            'velocity': {
                'linear': {'x': 0.5, 'y': 0.0, 'z': 0.0},
            },
            'dimensions': {'x': 3.0, 'y': 1.0, 'z': 1.0},
            'header': {
                'frame_id': 'map',
                'stamp': {'sec': 10, 'nanosec': 0, 'seconds': 10.0},
            },
        }],
        'sensors': {
            'usv_01': {
                'mid360': {
                    'vehicle_id': 'usv_01',
                    'sensor_id': 'mid360',
                    'message_type': 'PointCloud2',
                    'healthy': True,
                    'timed_out': False,
                    'rate_hz': 10.0,
                    'frame_id': 'usv_01/mid360_link',
                },
            },
        },
        'mission': {'capture': {'state': 'TRACKING'}},
        'perception': {'primary_source': 'fused_targets'},
    }, received_at=20.0)
    snapshot = registry.snapshot()
    assert [item['id'] for item in snapshot['vehicles']] == [
        'uav_01', 'usv_01']
    assert snapshot['vehicles'][0]['state_source'] == 'tf_only'
    assert snapshot['targets'][0]['track_id'] == 'enemy_ship'
    assert snapshot['entities'][0]['id'] == 'friendly_ship'
    assert snapshot['sensors'][0]['sensor_id'] == 'mid360'
    assert snapshot['world_model']['schema_version'] == 'fleet_world_model.v1'
    lightweight = registry.snapshot(include_world_model=False)
    assert lightweight['world_model'] == {}


def test_registry_stores_fleet_world_model_summary():
    registry = FleetRegistry()
    registry.update_world_model_summary({
        'schema_version': 'fleet_world_model.summary.v1',
        'world_model_schema_version': 'fleet_world_model.v1',
        'uav_count': 3,
        'usv_count': 3,
        'target_count': 1,
        'primary_source': 'fused_targets',
    })
    summary = registry.world_model_summary()
    assert summary['schema_version'] == 'fleet_world_model.summary.v1'
    assert summary['world_model_schema_version'] == 'fleet_world_model.v1'
    assert summary['uav_count'] == 3
    assert summary['usv_count'] == 3
    assert summary['target_count'] == 1


import pytest  # noqa: E402
