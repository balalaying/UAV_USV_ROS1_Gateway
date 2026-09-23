#!/usr/bin/env python3
"""Safe ROS 1 FleetCommand checks for the heterogeneous 3+3 SITL fleet."""

import argparse
import math
import sys
import threading
import time
import uuid

import rospy

from uav_usv_interfaces.msg import CommandAck, ControlLease, FleetCommand
from uav_usv_interfaces.msg import VehicleState


UAV_IDS = ('uav_01', 'uav_02', 'uav_03')
USV_IDS = ('usv_01', 'usv_02', 'usv_03')
VEHICLE_IDS = UAV_IDS + USV_IDS
TERMINAL_ACKS = {
    CommandAck.STATUS_SUCCEEDED,
    CommandAck.STATUS_REJECTED,
    CommandAck.STATUS_FAILED,
    CommandAck.STATUS_CANCELED,
}


class FleetTestError(RuntimeError):
    pass


class FleetTester:
    def __init__(self, timeout):
        self.timeout = float(timeout)
        self.lock = threading.Lock()
        self.states = {}
        self.state_times = {}
        self.leases = {}
        self.lease_times = {}
        self.acks = {}
        self.command_pub = rospy.Publisher(
            '/fleet/command', FleetCommand, queue_size=20
        )
        rospy.Subscriber(
            '/fleet/state', VehicleState, self._on_state, queue_size=100
        )
        rospy.Subscriber(
            '/fleet/control_lease', ControlLease, self._on_lease, queue_size=50
        )
        rospy.Subscriber(
            '/fleet/command_ack', CommandAck, self._on_ack, queue_size=100
        )

    def _on_state(self, message):
        if message.vehicle_id in VEHICLE_IDS:
            with self.lock:
                self.states[message.vehicle_id] = message
                self.state_times[message.vehicle_id] = time.monotonic()

    def _on_lease(self, message):
        if message.vehicle_id in VEHICLE_IDS or message.vehicle_id == '*':
            with self.lock:
                self.leases[message.vehicle_id] = message
                self.lease_times[message.vehicle_id] = time.monotonic()

    def _on_ack(self, message):
        with self.lock:
            self.acks.setdefault(message.command_id, []).append(message)
        print(
            'ACK vehicle=%s command=%s status=%s progress=%.2f message=%s'
            % (
                message.vehicle_id,
                message.command_id,
                self.ack_name(message.status),
                message.progress,
                message.message,
            ),
            flush=True,
        )

    @staticmethod
    def ack_name(status):
        names = {
            CommandAck.STATUS_RECEIVED: 'RECEIVED',
            CommandAck.STATUS_ACCEPTED: 'ACCEPTED',
            CommandAck.STATUS_EXECUTING: 'EXECUTING',
            CommandAck.STATUS_SUCCEEDED: 'SUCCEEDED',
            CommandAck.STATUS_REJECTED: 'REJECTED',
            CommandAck.STATUS_FAILED: 'FAILED',
            CommandAck.STATUS_CANCELED: 'CANCELED',
        }
        return names.get(status, str(status))

    @staticmethod
    def command_name(command_type):
        names = {
            FleetCommand.COMMAND_HOLD: 'HOLD',
            FleetCommand.COMMAND_NAVIGATE: 'NAVIGATE',
            FleetCommand.COMMAND_TAKEOFF: 'TAKEOFF',
            FleetCommand.COMMAND_LAND: 'LAND',
            FleetCommand.COMMAND_EMERGENCY_STOP: 'EMERGENCY_STOP',
        }
        return names.get(command_type, str(command_type))

    def state(self, vehicle_id):
        with self.lock:
            return self.states.get(vehicle_id), self.state_times.get(vehicle_id)

    def print_state(self, vehicle_id, label='STATE'):
        state, received = self.state(vehicle_id)
        if state is None:
            print('%s vehicle=%s unavailable' % (label, vehicle_id), flush=True)
            return
        age = time.monotonic() - received
        print(
            '%s vehicle=%s online=%s armed=%s mode=%s '
            'position=(%.3f, %.3f, %.3f) speed=%.3f age=%.2fs status=%s'
            % (
                label,
                vehicle_id,
                state.online,
                state.armed,
                state.mode,
                state.pose.position.x,
                state.pose.position.y,
                state.pose.position.z,
                math.sqrt(
                    state.twist.linear.x ** 2
                    + state.twist.linear.y ** 2
                    + state.twist.linear.z ** 2
                ),
                age,
                state.status_text,
            ),
            flush=True,
        )

    def _wait(self, description, predicate, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        rate = rospy.Rate(10)
        last_print = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            now = time.monotonic()
            if now - last_print >= 5.0:
                print('WAITING %s' % description, flush=True)
                last_print = now
            rate.sleep()
        raise FleetTestError('timeout waiting for ' + description)

    def wait_online(self, vehicle_ids):
        def ready():
            now = time.monotonic()
            with self.lock:
                return all(
                    vehicle_id in self.states
                    and self.states[vehicle_id].online
                    and now - self.state_times[vehicle_id] < 2.0
                    for vehicle_id in vehicle_ids
                )

        self._wait('online states for ' + ','.join(vehicle_ids), ready, 120.0)
        for vehicle_id in vehicle_ids:
            self.print_state(vehicle_id, 'ONLINE')

    def _valid_lease(self, vehicle_id):
        with self.lock:
            lease = self.leases.get(vehicle_id) or self.leases.get('*')
        if lease is None or lease.revoked or lease.valid_until <= rospy.Time.now():
            return None
        return lease

    def wait_lease(self, vehicle_id):
        lease = self._wait(
            'existing valid lease for ' + vehicle_id,
            lambda: self._valid_lease(vehicle_id),
            20.0,
        )
        print(
            'LEASE vehicle=%s owner=%s lease_id=%s priority=%d'
            % (vehicle_id, lease.owner_id, lease.lease_id, lease.priority),
            flush=True,
        )
        return lease

    def send_command(
        self, vehicle_id, command_type, target=None, parameters=None, timeout=None
    ):
        lease = None
        if command_type != FleetCommand.COMMAND_EMERGENCY_STOP:
            lease = self.wait_lease(vehicle_id)
        command = FleetCommand()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = 'map'
        command.command_id = 'fleet-test-%s-%s' % (
            vehicle_id, uuid.uuid4().hex[:10]
        )
        command.vehicle_id = vehicle_id
        command.lease_id = lease.lease_id if lease is not None else ''
        command.command_type = command_type
        command.priority = 200
        command.expires_at = rospy.Time.now() + rospy.Duration(15.0)
        if target is not None:
            command.target_pose.position.x = float(target[0])
            command.target_pose.position.y = float(target[1])
            command.target_pose.position.z = float(target[2])
        command.target_pose.orientation.w = 1.0
        command.parameters = list(parameters or [])
        print(
            'COMMAND vehicle=%s type=%s command_id=%s target=%s parameters=%s'
            % (
                vehicle_id,
                self.command_name(command_type),
                command.command_id,
                target,
                command.parameters,
            ),
            flush=True,
        )
        self.command_pub.publish(command)

        def terminal():
            with self.lock:
                messages = list(self.acks.get(command.command_id, ()))
            for message in messages:
                if message.status in TERMINAL_ACKS:
                    return message
            return None

        ack = self._wait(
            '%s terminal ACK from %s'
            % (self.command_name(command_type), vehicle_id),
            terminal,
            timeout,
        )
        if ack.status != CommandAck.STATUS_SUCCEEDED:
            raise FleetTestError(
                '%s %s: %s'
                % (vehicle_id, self.ack_name(ack.status), ack.message)
            )
        return ack

    def wait_disarmed(self, vehicle_id, timeout=60.0):
        self._wait(
            vehicle_id + ' disarmed',
            lambda: (
                self.state(vehicle_id)[0] is not None
                and not self.state(vehicle_id)[0].armed
            ),
            timeout,
        )

    def wait_usv_stopped(self, vehicle_id, timeout=15.0):
        stable_since = [None]

        def stopped():
            state, received = self.state(vehicle_id)
            if state is None or time.monotonic() - received >= 2.0:
                stable_since[0] = None
                return False
            speed = math.sqrt(
                state.twist.linear.x ** 2
                + state.twist.linear.y ** 2
                + state.twist.linear.z ** 2
            )
            if speed <= 0.15:
                if stable_since[0] is None:
                    stable_since[0] = time.monotonic()
                return time.monotonic() - stable_since[0] >= 2.0
            stable_since[0] = None
            return False

        self._wait(vehicle_id + ' stopped', stopped, timeout)

    def test_uav(self, vehicle_id):
        self.wait_online((vehicle_id,))
        initial, _ = self.state(vehicle_id)
        if initial.armed:
            raise FleetTestError(vehicle_id + ' is armed before test')
        start = (
            initial.pose.position.x,
            initial.pose.position.y,
            initial.pose.position.z,
        )
        self.print_state(vehicle_id, 'UAV START')
        self.send_command(
            vehicle_id, FleetCommand.COMMAND_TAKEOFF, parameters=[2.0], timeout=60.0
        )
        airborne, _ = self.state(vehicle_id)
        climb = airborne.pose.position.z - start[2]
        if climb < 1.6:
            raise FleetTestError('%s climb too small: %.3f m' % (vehicle_id, climb))
        self.print_state(vehicle_id, 'AFTER TAKEOFF')
        target = (
            airborne.pose.position.x + 3.0,
            airborne.pose.position.y,
            airborne.pose.position.z,
        )
        nav_start = (
            airborne.pose.position.x,
            airborne.pose.position.y,
            airborne.pose.position.z,
        )
        self.send_command(
            vehicle_id, FleetCommand.COMMAND_NAVIGATE, target=target, timeout=60.0
        )
        moved, _ = self.state(vehicle_id)
        displacement = math.hypot(
            moved.pose.position.x - nav_start[0],
            moved.pose.position.y - nav_start[1],
        )
        if displacement < 2.4:
            raise FleetTestError(
                '%s horizontal displacement too small: %.3f m'
                % (vehicle_id, displacement)
            )
        print('UAV MOVE vehicle=%s displacement=%.3f m' % (vehicle_id, displacement))
        self.print_state(vehicle_id, 'AFTER NAVIGATE')
        self.send_command(vehicle_id, FleetCommand.COMMAND_LAND, timeout=90.0)
        self.wait_disarmed(vehicle_id, 20.0)
        self.print_state(vehicle_id, 'UAV FINAL')
        print('PASS UAV %s' % vehicle_id, flush=True)

    def test_usv(self, vehicle_id):
        self.wait_online((vehicle_id,))
        initial, _ = self.state(vehicle_id)
        if initial.armed:
            raise FleetTestError(vehicle_id + ' is armed before test')
        start = (
            initial.pose.position.x,
            initial.pose.position.y,
            initial.pose.position.z,
        )
        target = (start[0] + 5.0, start[1], start[2])
        self.print_state(vehicle_id, 'USV START')
        print('USV TARGET vehicle=%s target=%s' % (vehicle_id, target), flush=True)
        self.send_command(
            vehicle_id, FleetCommand.COMMAND_NAVIGATE, target=target, timeout=90.0
        )
        current, _ = self.state(vehicle_id)
        displacement = math.hypot(
            current.pose.position.x - start[0],
            current.pose.position.y - start[1],
        )
        print(
            'USV MOVE vehicle=%s start=(%.3f, %.3f) target=(%.3f, %.3f) '
            'current=(%.3f, %.3f) displacement=%.3f m'
            % (
                vehicle_id,
                start[0], start[1], target[0], target[1],
                current.pose.position.x, current.pose.position.y, displacement,
            ),
            flush=True,
        )
        if displacement < 2.0:
            raise FleetTestError(
                '%s actual displacement too small: %.3f m'
                % (vehicle_id, displacement)
            )
        self.print_state(vehicle_id, 'AFTER NAVIGATE')
        self.send_command(vehicle_id, FleetCommand.COMMAND_HOLD, timeout=20.0)
        self.wait_usv_stopped(vehicle_id)
        self.print_state(vehicle_id, 'USV FINAL')
        print('PASS USV %s' % vehicle_id, flush=True)

    def _run_parallel_commands(self, commands):
        """Run independent FleetCommands concurrently and propagate failures."""
        errors = []
        errors_lock = threading.Lock()

        def worker(vehicle_id, command_type, target, parameters, timeout):
            try:
                self.send_command(
                    vehicle_id,
                    command_type,
                    target=target,
                    parameters=parameters,
                    timeout=timeout,
                )
            except Exception as error:
                with errors_lock:
                    errors.append((vehicle_id, error))

        threads = []
        for command in commands:
            thread = threading.Thread(target=worker, args=command)
            thread.daemon = True
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join()
        if errors:
            raise FleetTestError(
                'parallel command failed: '
                + '; '.join('%s: %s' % item for item in errors)
            )

    def test_parallel_all(self):
        self.wait_online(VEHICLE_IDS)
        initial = {}
        for vehicle_id in VEHICLE_IDS:
            state, _ = self.state(vehicle_id)
            if state.armed:
                raise FleetTestError(vehicle_id + ' is armed before parallel test')
            initial[vehicle_id] = (
                state.pose.position.x,
                state.pose.position.y,
                state.pose.position.z,
            )

        try:
            # Sequential takeoff limits risk while all six SITL instances are busy.
            for vehicle_id in UAV_IDS:
                self.send_command(
                    vehicle_id,
                    FleetCommand.COMMAND_TAKEOFF,
                    parameters=[2.0],
                    timeout=60.0,
                )
                state, _ = self.state(vehicle_id)
                climb = state.pose.position.z - initial[vehicle_id][2]
                if climb < 1.6:
                    raise FleetTestError(
                        '%s climb too small: %.3f m' % (vehicle_id, climb)
                    )
                self.print_state(vehicle_id, 'PARALLEL AIRBORNE')

            nav_start = {}
            commands = []
            for vehicle_id in VEHICLE_IDS:
                state, _ = self.state(vehicle_id)
                nav_start[vehicle_id] = (
                    state.pose.position.x,
                    state.pose.position.y,
                    state.pose.position.z,
                )
                distance = 3.0 if vehicle_id in UAV_IDS else 5.0
                target = (
                    nav_start[vehicle_id][0] + distance,
                    nav_start[vehicle_id][1],
                    nav_start[vehicle_id][2],
                )
                commands.append((
                    vehicle_id,
                    FleetCommand.COMMAND_NAVIGATE,
                    target,
                    None,
                    90.0,
                ))

            print('PARALLEL MOVE starting six NAVIGATE commands', flush=True)
            self._run_parallel_commands(commands)

            for vehicle_id in VEHICLE_IDS:
                state, _ = self.state(vehicle_id)
                displacement = math.hypot(
                    state.pose.position.x - nav_start[vehicle_id][0],
                    state.pose.position.y - nav_start[vehicle_id][1],
                )
                minimum = 2.4 if vehicle_id in UAV_IDS else 2.0
                print(
                    'PARALLEL MOVE vehicle=%s displacement=%.3f m'
                    % (vehicle_id, displacement),
                    flush=True,
                )
                if displacement < minimum:
                    raise FleetTestError(
                        '%s displacement too small: %.3f m'
                        % (vehicle_id, displacement)
                    )

            finish_commands = [
                (vehicle_id, FleetCommand.COMMAND_LAND, None, None, 90.0)
                for vehicle_id in UAV_IDS
            ] + [
                (vehicle_id, FleetCommand.COMMAND_HOLD, None, None, 20.0)
                for vehicle_id in USV_IDS
            ]
            print('PARALLEL FINISH starting UAV LAND and USV HOLD', flush=True)
            self._run_parallel_commands(finish_commands)
            for vehicle_id in UAV_IDS:
                self.wait_disarmed(vehicle_id, 30.0)
            for vehicle_id in USV_IDS:
                self.wait_usv_stopped(vehicle_id, 20.0)
            for vehicle_id in VEHICLE_IDS:
                self.print_state(vehicle_id, 'PARALLEL FINAL')
            print('PASS PARALLEL ALL', flush=True)
        except Exception:
            for vehicle_id in VEHICLE_IDS:
                state, _ = self.state(vehicle_id)
                if vehicle_id in UAV_IDS and state is not None and state.armed:
                    self.abort_vehicle(vehicle_id)
                elif vehicle_id in USV_IDS:
                    self.abort_vehicle(vehicle_id)
            raise

    def test_demo_100m(self):
        """Run a visible ~96 m rectangular route with four clear turns."""
        self.wait_online(VEHICLE_IDS)
        for vehicle_id in UAV_IDS:
            state, _ = self.state(vehicle_id)
            if state.armed:
                raise FleetTestError(vehicle_id + ' is armed before demo')

        cumulative = {vehicle_id: 0.0 for vehicle_id in VEHICLE_IDS}
        try:
            for vehicle_id in UAV_IDS:
                state, _ = self.state(vehicle_id)
                start_z = state.pose.position.z
                self.send_command(
                    vehicle_id,
                    FleetCommand.COMMAND_TAKEOFF,
                    parameters=[5.0],
                    timeout=75.0,
                )
                state, _ = self.state(vehicle_id)
                if state.pose.position.z - start_z < 4.3:
                    raise FleetTestError(vehicle_id + ' did not reach demo altitude')
                self.print_state(vehicle_id, 'DEMO AIRBORNE')

            # Twelve 8 m segments form a 24 x 24 m rectangle. Keeping every
            # UAV segment below 10 m avoids the known long-waypoint limitation.
            route = (
                ((8.0, 0.0),) * 3
                + ((0.0, 8.0),) * 3
                + ((-8.0, 0.0),) * 3
                + ((0.0, -8.0),) * 3
            )
            for index, (dx, dy) in enumerate(route, 1):
                before = {}
                commands = []
                for vehicle_id in VEHICLE_IDS:
                    state, _ = self.state(vehicle_id)
                    before[vehicle_id] = (
                        state.pose.position.x,
                        state.pose.position.y,
                        state.pose.position.z,
                    )
                    target = (
                        before[vehicle_id][0] + dx,
                        before[vehicle_id][1] + dy,
                        before[vehicle_id][2],
                    )
                    commands.append((
                        vehicle_id,
                        FleetCommand.COMMAND_NAVIGATE,
                        target,
                        None,
                        90.0,
                    ))
                print(
                    'DEMO SEGMENT %d/12 delta=(%.1f, %.1f)'
                    % (index, dx, dy),
                    flush=True,
                )
                self._run_parallel_commands(commands)
                for vehicle_id in VEHICLE_IDS:
                    state, _ = self.state(vehicle_id)
                    distance = math.hypot(
                        state.pose.position.x - before[vehicle_id][0],
                        state.pose.position.y - before[vehicle_id][1],
                    )
                    cumulative[vehicle_id] += distance
                    if distance < 6.0:
                        raise FleetTestError(
                            '%s demo segment %d too short: %.3f m'
                            % (vehicle_id, index, distance)
                        )
                    print(
                        'DEMO PROGRESS vehicle=%s segment=%.3f m total=%.3f m'
                        % (vehicle_id, distance, cumulative[vehicle_id]),
                        flush=True,
                    )

            finish_commands = [
                (vehicle_id, FleetCommand.COMMAND_LAND, None, None, 90.0)
                for vehicle_id in UAV_IDS
            ] + [
                (vehicle_id, FleetCommand.COMMAND_HOLD, None, None, 20.0)
                for vehicle_id in USV_IDS
            ]
            print('DEMO COMPLETE: landing UAVs and holding USVs', flush=True)
            self._run_parallel_commands(finish_commands)
            for vehicle_id in UAV_IDS:
                self.wait_disarmed(vehicle_id, 30.0)
            for vehicle_id in USV_IDS:
                self.wait_usv_stopped(vehicle_id, 20.0)
            for vehicle_id in VEHICLE_IDS:
                self.print_state(vehicle_id, 'DEMO FINAL')
                print(
                    'DEMO DISTANCE vehicle=%s total=%.3f m'
                    % (vehicle_id, cumulative[vehicle_id]),
                    flush=True,
                )
            print('PASS DEMO 100M', flush=True)
        except Exception:
            for vehicle_id in VEHICLE_IDS:
                state, _ = self.state(vehicle_id)
                if vehicle_id in UAV_IDS and state is not None and state.armed:
                    self.abort_vehicle(vehicle_id)
                elif vehicle_id in USV_IDS:
                    self.abort_vehicle(vehicle_id)
            raise

    def abort_vehicle(self, vehicle_id):
        """Best-effort safe action after a failed test; never continue testing."""
        state, _ = self.state(vehicle_id)
        try:
            if vehicle_id in UAV_IDS and state is not None and state.armed:
                print('SAFETY: requesting LAND for ' + vehicle_id, flush=True)
                self.send_command(
                    vehicle_id, FleetCommand.COMMAND_LAND, timeout=30.0
                )
            elif vehicle_id in USV_IDS:
                print('SAFETY: requesting EMERGENCY_STOP for ' + vehicle_id, flush=True)
                self.send_command(
                    vehicle_id,
                    FleetCommand.COMMAND_EMERGENCY_STOP,
                    timeout=10.0,
                )
        except Exception as error:
            print(
                'SAFETY ACTION FAILED vehicle=%s error=%s' % (vehicle_id, error),
                file=sys.stderr,
                flush=True,
            )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--check-only', action='store_true')
    selection.add_argument('--vehicle', choices=VEHICLE_IDS)
    selection.add_argument('--uavs', action='store_true')
    selection.add_argument('--usvs', action='store_true')
    selection.add_argument('--all', action='store_true')
    selection.add_argument('--parallel-all', action='store_true')
    selection.add_argument('--demo-100m', action='store_true')
    parser.add_argument('--timeout', type=float, default=75.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node('test_332_ardupilot_fleet', anonymous=True)
    tester = FleetTester(args.timeout)
    active_vehicle = None
    try:
        tester.wait_online(VEHICLE_IDS if args.check_only else (
            (args.vehicle,) if args.vehicle else
            UAV_IDS if args.uavs else
            USV_IDS if args.usvs else VEHICLE_IDS
        ))
        if args.check_only:
            return 0
        if args.parallel_all:
            tester.test_parallel_all()
            return 0
        if args.demo_100m:
            tester.test_demo_100m()
            return 0
        selected = (
            (args.vehicle,) if args.vehicle else
            UAV_IDS if args.uavs else
            USV_IDS if args.usvs else VEHICLE_IDS
        )
        for vehicle_id in selected:
            active_vehicle = vehicle_id
            if vehicle_id in UAV_IDS:
                tester.test_uav(vehicle_id)
            else:
                tester.test_usv(vehicle_id)
            active_vehicle = None
        return 0
    except FleetTestError as error:
        print('FAIL: %s' % error, file=sys.stderr, flush=True)
        if active_vehicle is not None:
            tester.abort_vehicle(active_vehicle)
        for vehicle_id in VEHICLE_IDS:
            tester.print_state(vehicle_id, 'FAILURE STATE')
        return 1


if __name__ == '__main__':
    sys.exit(main())
