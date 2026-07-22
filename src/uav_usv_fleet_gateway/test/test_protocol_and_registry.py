import math
from types import SimpleNamespace as NS

from uav_usv_fleet_gateway.fleet_registry import FleetRegistry
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
        mode='PX4', pose=_pose(), twist=_twist(3.0, 4.0, 0.0),
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


import pytest  # noqa: E402
