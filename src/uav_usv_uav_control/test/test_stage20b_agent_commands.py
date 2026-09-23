import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS

from pymavlink import mavutil
import rospy

from uav_usv_interfaces.msg import CommandAck, FleetCommand


REPOSITORY = Path(__file__).resolve().parents[3]


def _install_fake_gz_modules(monkeypatch):
    modules = {
        'gz': ModuleType('gz'),
        'gz.msgs10': ModuleType('gz.msgs10'),
        'gz.msgs10.boolean_pb2': ModuleType('gz.msgs10.boolean_pb2'),
        'gz.msgs10.pose_v_pb2': ModuleType('gz.msgs10.pose_v_pb2'),
        'gz.transport13': ModuleType('gz.transport13'),
    }
    modules['gz.msgs10.boolean_pb2'].Boolean = type('Boolean', (), {})
    modules['gz.msgs10.pose_v_pb2'].Pose_V = type('Pose_V', (), {})
    modules['gz.transport13'].Node = type('GzNode', (), {})
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)


def _load_agent(monkeypatch, name, relative_path):
    _install_fake_gz_modules(monkeypatch)
    spec = importlib.util.spec_from_file_location(
        name, REPOSITORY / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module.mavutil,
        'mode_string_v10',
        lambda heartbeat: heartbeat.mode,
    )
    return module


def _command(command_id, command_type):
    message = FleetCommand()
    message.command_id = command_id
    message.command_type = command_type
    return message


def _uav_agent(module, command, previous_command_id=''):
    agent = object.__new__(module.UavFleetAgent)
    agent.lock = threading.Lock()
    agent.pending_command = command
    agent.active_command_id = previous_command_id
    agent.operation = 'navigate' if previous_command_id else None
    agent.guided_target = (1.0, 2.0, -3.0)
    agent.goal_gazebo = (2.0, 1.0, 3.0)
    agent.pose = None
    agent.local_position = None
    agent.heartbeat = NS(
        mode='GUIDED',
        base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
    )
    agent.status_text = 'test'
    agent.operation_started_at = 0.0
    agent.operation_mode_seen = False
    agent.last_operation_ack = 0.0
    agent.mode_transition_timeout = 10.0
    agent.return_timeout = 300.0
    agent.emergency_land_timeout = 180.0
    agent.acks = []
    agent.modes = []
    agent._ack = lambda *args: agent.acks.append(args)
    agent._set_mode = agent.modes.append
    return agent


def test_uav_return_enters_rtl_and_waits_for_observed_disarm(monkeypatch):
    module = _load_agent(
        monkeypatch,
        'stage20b_uav_agent_return',
        'src/uav_usv_uav_control/scripts/uav_fleet_agent.py',
    )
    agent = _uav_agent(
        module, _command('uav-return', FleetCommand.COMMAND_RETURN))
    agent._consume_command()
    assert agent.modes == ['RTL']
    assert agent.operation == 'return'
    assert agent.acks[-1][1] == CommandAck.STATUS_EXECUTING

    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    agent.heartbeat = NS(mode='RTL', base_mode=armed_flag)
    agent._update_operation()
    assert not any(ack[1] == CommandAck.STATUS_SUCCEEDED for ack in agent.acks)

    agent.heartbeat = NS(mode='RTL', base_mode=0)
    agent._update_operation()
    assert agent.acks[-1][1] == CommandAck.STATUS_SUCCEEDED


def test_uav_emergency_land_preempts_and_never_sends_disarm(monkeypatch):
    module = _load_agent(
        monkeypatch,
        'stage20b_uav_agent_emergency_land',
        'src/uav_usv_uav_control/scripts/uav_fleet_agent.py',
    )
    agent = _uav_agent(
        module,
        _command('uav-emergency-land', FleetCommand.COMMAND_EMERGENCY_LAND),
        previous_command_id='old-navigation',
    )
    # Any unexpected direct MAVLink/disarm access fails this test.
    agent.mav = object()
    agent._consume_command()
    assert agent.modes == ['LAND']
    assert agent.operation == 'emergency_land'
    assert agent.guided_target is None
    assert agent.goal_gazebo is None
    assert agent.acks[0][0] == 'old-navigation'
    assert agent.acks[0][1] == CommandAck.STATUS_CANCELED
    assert agent.acks[-1][1] == CommandAck.STATUS_EXECUTING

    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    agent.heartbeat = NS(mode='LAND', base_mode=armed_flag)
    agent._update_operation()
    assert not any(ack[1] == CommandAck.STATUS_SUCCEEDED for ack in agent.acks)
    agent.heartbeat = NS(mode='LAND', base_mode=0)
    agent._update_operation()
    assert agent.acks[-1][1] == CommandAck.STATUS_SUCCEEDED


def test_uav_emergency_land_bypasses_normal_agent_lease(monkeypatch):
    module = _load_agent(
        monkeypatch,
        'stage20b_uav_agent_emergency_lease',
        'src/uav_usv_uav_control/scripts/uav_fleet_agent.py',
    )
    agent = object.__new__(module.UavFleetAgent)
    agent.vehicle_id = 'uav_01'
    agent.lock = threading.Lock()
    agent.pending_command = None
    agent._lease_valid = lambda _lease_id: False
    agent.acks = []
    agent._ack = lambda *args: agent.acks.append(args)
    command = _command(
        'uav-emergency-no-lease', FleetCommand.COMMAND_EMERGENCY_LAND)
    command.vehicle_id = 'uav_01'
    original_time = rospy.Time
    command.expires_at = original_time(100, 0)
    monkeypatch.setattr(
        module.rospy,
        'Time',
        NS(now=lambda: original_time(1, 0)),
    )

    agent._on_command(command)
    assert agent.pending_command is command
    assert agent.acks[-1][1] == CommandAck.STATUS_ACCEPTED


def test_usv_return_enters_rtl_and_uses_fresh_wp_distance(monkeypatch):
    module = _load_agent(
        monkeypatch,
        'stage20b_usv_agent_return',
        'src/uav_usv_usv_control/scripts/usv_gz_fleet_agent.py',
    )
    command = _command('usv-return', FleetCommand.COMMAND_RETURN)
    agent = object.__new__(module.UsvArduRoverFleetAgent)
    agent.lock = threading.Lock()
    agent.pending_command = command
    agent.active_command_id = ''
    agent.operation = None
    agent.guided_target = (1.0, 2.0, 0.0)
    agent.goal_gazebo = (2.0, 1.0)
    agent.nav_controller_output = None
    agent.nav_controller_output_received_at = 0.0
    agent.operation_started_at = 0.0
    agent.operation_mode_seen = False
    agent.last_progress_ack = 0.0
    agent.mode_transition_timeout = 10.0
    agent.return_timeout = 300.0
    agent.arrival_tolerance = 1.0
    agent.heartbeat = NS(
        mode='GUIDED',
        base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
    )
    agent.status_text = 'test'
    agent.acks = []
    agent.modes = []
    agent._ack = lambda *args: agent.acks.append(args)
    agent._set_mode = agent.modes.append

    agent._consume_command()
    assert agent.modes == ['RTL']
    assert agent.operation == 'return'
    assert agent.acks[-1][1] == CommandAck.STATUS_EXECUTING

    agent.heartbeat = NS(mode='RTL', base_mode=0)
    agent.nav_controller_output = NS(wp_dist=0.0)
    agent.nav_controller_output_received_at = time.monotonic()
    agent._update_return()
    assert agent.modes == ['RTL', 'HOLD']
    assert agent.acks[-1][1] == CommandAck.STATUS_SUCCEEDED
