import math
import threading
from types import SimpleNamespace as NS

import rospy

from uav_usv_interfaces.msg import FleetCommand
from uav_usv_fleet_gateway.gateway_node import FleetGatewayNode
from uav_usv_fleet_gateway.gateway_node import device_command_specs
from uav_usv_fleet_gateway.protocol_v1 import V1ProtocolEncoder
from uav_usv_fleet_gateway import uav_usv_gateway_v1_pb2 as pb


class _Registry:
    def vehicles(self):
        return [
            {'id': 'uav_01', 'online': True},
            {'id': 'usv_01', 'online': True},
        ]


class _ClockNow:
    nanoseconds = 100_000_000_000

    def to_msg(self):
        return rospy.Time(100, 0)


class _Clock:
    def now(self):
        return _ClockNow()


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _gateway(with_lease=True):
    node = object.__new__(FleetGatewayNode)
    node.protocol_v1 = V1ProtocolEncoder('test-gateway', 'test-boot')
    node._v1_command_lock = threading.Lock()
    node._active_v1_mission = None
    node._v1_device_lock = threading.Lock()
    node._v1_device_commands = {}
    node.registry = _Registry()
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: _Logger()
    node._float = lambda _name: 30.0
    lease = NS(lease_id='shore-lease', priority=200)
    node._valid_base_station_lease_v1 = (
        lambda _device: lease if with_lease else None
    )
    node.scheduled = []
    node._schedule_device_command_v1 = node.scheduled.append
    return node


def _request(command_name, device_code, parameters=None, command_id='cmd-1'):
    envelope = pb.GatewayEnvelope()
    envelope.spec_version = '1.0'
    envelope.message_type = 'control.command'
    command = envelope.control_command
    command.command_id = command_id
    command.command = command_name
    command.target.scope = pb.DEVICE
    command.target.device_codes.append(device_code)
    for name, value in (parameters or {}).items():
        parameter = command.parameters[name]
        if isinstance(value, str):
            parameter.string_value = value
        else:
            parameter.double_value = float(value)
    return envelope.SerializeToString()


def _response(node, payload):
    raw = node._handle_command_v1(payload)
    envelope = pb.GatewayEnvelope()
    envelope.ParseFromString(raw)
    return envelope.control_ack


def test_new_command_mapping_constants_are_distinct():
    specs = device_command_specs()
    assert specs['UAV_RETURN']['command_type'] == FleetCommand.COMMAND_RETURN
    assert specs['USV_RETURN']['command_type'] == FleetCommand.COMMAND_RETURN
    assert specs['USV_DEPART']['command_type'] == FleetCommand.COMMAND_NAVIGATE
    assert (
        specs['UAV_EMERGENCY_LAND']['command_type']
        == FleetCommand.COMMAND_EMERGENCY_LAND
    )
    assert FleetCommand.COMMAND_EMERGENCY_LAND != FleetCommand.COMMAND_LAND
    assert (
        FleetCommand.COMMAND_EMERGENCY_LAND
        != FleetCommand.COMMAND_EMERGENCY_STOP
    )


def test_return_commands_require_matching_device_and_normal_lease():
    for command_name, valid_device, invalid_device in (
        ('UAV_RETURN', 'uav_01', 'usv_01'),
        ('USV_RETURN', 'usv_01', 'uav_01'),
    ):
        node = _gateway(with_lease=True)
        ack = _response(node, _request(command_name, invalid_device))
        assert ack.status == pb.REJECTED
        assert ack.code == 'INVALID_DEVICE_TYPE'
        assert node.scheduled == []

        node = _gateway(with_lease=False)
        ack = _response(node, _request(command_name, valid_device))
        assert ack.status == pb.REJECTED
        assert ack.code == 'NO_VALID_CONTROL_LEASE'
        assert node.scheduled == []

        node = _gateway(with_lease=True)
        ack = _response(node, _request(command_name, valid_device))
        assert ack.status == pb.ACCEPTED
        assert len(node.scheduled) == 1
        assert node.scheduled[0].command_type == FleetCommand.COMMAND_RETURN


def test_emergency_land_bypasses_lease_and_uses_dedicated_type():
    node = _gateway(with_lease=False)
    ack = _response(node, _request('UAV_EMERGENCY_LAND', 'uav_01'))
    assert ack.status == pb.ACCEPTED
    assert len(node.scheduled) == 1
    command = node.scheduled[0]
    assert command.lease_id == ''
    assert command.priority == 200
    assert command.command_type == FleetCommand.COMMAND_EMERGENCY_LAND
    assert command.command_type != FleetCommand.COMMAND_LAND
    assert command.command_type != FleetCommand.COMMAND_EMERGENCY_STOP


def test_usv_depart_maps_valid_absolute_map_target():
    node = _gateway(with_lease=True)
    ack = _response(node, _request('USV_DEPART', 'usv_01', {
        'frame_id': 'map',
        'target_x_m': 12.5,
        'target_y_m': -8.25,
    }))
    assert ack.status == pb.ACCEPTED
    assert len(node.scheduled) == 1
    command = node.scheduled[0]
    assert command.command_type == FleetCommand.COMMAND_NAVIGATE
    assert command.target_pose.position.x == 12.5
    assert command.target_pose.position.y == -8.25
    assert command.target_pose.orientation.w == 1.0


def test_usv_depart_rejects_missing_or_invalid_target():
    valid = {
        'frame_id': 'map',
        'target_x_m': 12.5,
        'target_y_m': -8.25,
    }
    invalid_cases = [
        ({key: value for key, value in valid.items() if key != 'frame_id'},
         'MISSING_PARAMETER'),
        (dict(valid, frame_id='odom'), 'INVALID_TARGET_FRAME'),
        ({key: value for key, value in valid.items() if key != 'target_x_m'},
         'MISSING_PARAMETER'),
        ({key: value for key, value in valid.items() if key != 'target_y_m'},
         'MISSING_PARAMETER'),
        (dict(valid, target_x_m=math.nan), 'INVALID_TARGET_COORDINATE'),
        (dict(valid, target_y_m=math.inf), 'INVALID_TARGET_COORDINATE'),
    ]
    for index, (parameters, expected_code) in enumerate(invalid_cases):
        node = _gateway(with_lease=True)
        ack = _response(node, _request(
            'USV_DEPART', 'usv_01', parameters,
            command_id='depart-invalid-%d' % index,
        ))
        assert ack.status == pb.REJECTED
        assert ack.code == expected_code
        assert node.scheduled == []


def test_existing_six_device_commands_keep_their_mappings():
    cases = [
        ('UAV_TAKEOFF', 'uav_01', FleetCommand.COMMAND_TAKEOFF, True),
        ('UAV_HOVER', 'uav_01', FleetCommand.COMMAND_HOLD, True),
        ('UAV_LAND', 'uav_01', FleetCommand.COMMAND_LAND, True),
        ('USV_HOLD', 'usv_01', FleetCommand.COMMAND_HOLD, True),
        ('USV_STOP', 'usv_01', FleetCommand.COMMAND_HOLD, True),
        ('USV_EMERGENCY_STOP', 'usv_01',
         FleetCommand.COMMAND_EMERGENCY_STOP, False),
    ]
    for index, (name, device, expected_type, needs_lease) in enumerate(cases):
        node = _gateway(with_lease=needs_lease)
        ack = _response(node, _request(
            name, device, command_id='existing-%d' % index))
        assert ack.status == pb.ACCEPTED
        assert len(node.scheduled) == 1
        assert node.scheduled[0].command_type == expected_type
