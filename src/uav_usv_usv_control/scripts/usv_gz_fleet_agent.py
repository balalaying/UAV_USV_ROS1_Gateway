#!/usr/bin/env python3
"""ROS 1 fleet agent for one ArduRover SITL surface vessel."""

import math
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
from pymavlink import mavutil
import rospy

from sensor_msgs.msg import FluidPressure, Imu, MagneticField
from sensor_msgs.msg import NavSatFix, NavSatStatus
from uav_usv_interfaces.msg import CommandAck, ControlLease, FleetCommand
from uav_usv_interfaces.msg import VehicleState


class UsvArduRoverFleetAgent:
    def __init__(self):
        self.vehicle_id = rospy.get_param('~vehicle_id', 'usv_01')
        self.model_name = rospy.get_param('~model_name', self.vehicle_id)
        self.mavlink_url = rospy.get_param(
            '~mavlink_url', 'tcp:127.0.0.1:5790')
        self.pose_topic = rospy.get_param(
            '~pose_topic', '/world/heterogeneous_332/pose/info')
        self.arrival_tolerance = float(rospy.get_param('~arrival_tolerance', 0.5))
        # Force arming is disabled by default.  Normal ArduPilot pre-arm
        # checks remain authoritative unless explicitly enabled for diagnosis.
        self.force_arm = bool(rospy.get_param('~force_arm', False))
        self.arm_retry_seconds = max(
            0.5, float(rospy.get_param('~arm_retry_seconds', 1.0)))
        self.navigation_speed = max(
            0.5, float(rospy.get_param('~navigation_speed', 4.0)))
        self.mode_transition_timeout = max(
            2.0, float(rospy.get_param('~mode_transition_timeout', 10.0)))
        self.return_timeout = max(
            30.0, float(rospy.get_param('~return_timeout', 300.0)))

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.pose = None
        self.local_position = None
        self.heartbeat = None
        self.battery_percent = -1.0
        self.last_heartbeat = 0.0
        self.lease = None
        self.pending_command = None
        self.active_command_id = ''
        self.operation = None
        self.goal_gazebo = None
        self.guided_target = None
        self.nav_controller_output = None
        self.nav_controller_output_received_at = 0.0
        self.operation_started_at = 0.0
        self.operation_mode_seen = False
        self.status_text = 'connecting to ArduRover'
        self.mav = None
        self.target_system = 1
        self.target_component = 1
        self.last_setpoint = 0.0
        self.last_progress_ack = 0.0
        self.last_arm_request = 0.0
        self.last_mode_request = 0.0
        self.local_position_seen = False

        self.gz_node = GzTransportNode()
        if not self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose):
            raise RuntimeError('Unable to subscribe to ' + self.pose_topic)
        self.state_pub = rospy.Publisher('/fleet/state', VehicleState, queue_size=5)
        self.ack_pub = rospy.Publisher('/fleet/command_ack', CommandAck, queue_size=20)
        telemetry_prefix = '/fleet/uplink/%s' % self.vehicle_id
        self.imu_pub = rospy.Publisher(
            telemetry_prefix + '/imu/data_raw', Imu, queue_size=10)
        self.gps_pub = rospy.Publisher(
            telemetry_prefix + '/gps/fix', NavSatFix, queue_size=5)
        self.pressure_pub = rospy.Publisher(
            telemetry_prefix + '/barometer/pressure', FluidPressure,
            queue_size=10)
        self.magnetic_field_pub = rospy.Publisher(
            telemetry_prefix + '/magnetic_field', MagneticField,
            queue_size=10)
        rospy.Subscriber('/fleet/control_lease', ControlLease,
                         self._on_lease, queue_size=10)
        rospy.Subscriber('/fleet/command', FleetCommand,
                         self._on_command, queue_size=20)

        self.worker = threading.Thread(target=self._mavlink_loop, daemon=True)
        self.worker.start()
        self.timer = rospy.Timer(rospy.Duration(0.2), self._publish_state)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('ArduRover agent %s: MAVLink=%s model=%s',
                      self.vehicle_id, self.mavlink_url, self.model_name)

    def _on_pose(self, message):
        for pose in message.pose:
            if pose.name == self.model_name:
                with self.lock:
                    self.pose = pose
                return

    def _on_lease(self, message):
        if message.vehicle_id in (self.vehicle_id, '*'):
            with self.lock:
                self.lease = message

    def _lease_valid(self, lease_id):
        with self.lock:
            lease = self.lease
        return (lease is not None and not lease.revoked
                and lease.lease_id == lease_id
                and lease.valid_until > rospy.Time.now())

    def _ack(self, command_id, status, text, progress=0.0):
        message = CommandAck()
        message.header.stamp = rospy.Time.now()
        message.command_id = command_id
        message.vehicle_id = self.vehicle_id
        message.status = status
        message.progress = float(progress)
        message.message = text
        self.ack_pub.publish(message)

    def _on_command(self, message):
        if message.vehicle_id not in (self.vehicle_id, '*'):
            return
        if message.expires_at <= rospy.Time.now():
            self._ack(message.command_id, CommandAck.STATUS_REJECTED, 'command expired')
            return
        if (message.command_type != FleetCommand.COMMAND_EMERGENCY_STOP
                and not self._lease_valid(message.lease_id)):
            self._ack(message.command_id, CommandAck.STATUS_REJECTED,
                      'invalid or expired control lease')
            return
        with self.lock:
            self.pending_command = message
        self._ack(message.command_id, CommandAck.STATUS_ACCEPTED,
                  'ArduRover command accepted')

    def _publish_state(self, _event):
        with self.lock:
            pose = self.pose
            local = self.local_position
            heartbeat = self.heartbeat
            online = (self.mav is not None and heartbeat is not None
                      and time.monotonic() - self.last_heartbeat < 3.0)
            status_text = self.status_text
            command_id = self.active_command_id
            battery = self.battery_percent
        message = VehicleState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'map'
        message.vehicle_id = self.vehicle_id
        message.vehicle_type = VehicleState.TYPE_USV
        message.online = online
        message.armed = bool(heartbeat is not None and heartbeat.base_mode
                             & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        message.mode = (mavutil.mode_string_v10(heartbeat)
                        if heartbeat is not None else 'DISCONNECTED')
        if pose is not None:
            message.pose.position.x = pose.position.x
            message.pose.position.y = pose.position.y
            message.pose.position.z = pose.position.z
            message.pose.orientation.x = pose.orientation.x
            message.pose.orientation.y = pose.orientation.y
            message.pose.orientation.z = pose.orientation.z
            message.pose.orientation.w = pose.orientation.w
        if local is not None:
            message.twist.linear.x = math.hypot(local.vx, local.vy)
            message.twist.linear.z = local.vz
        message.battery_percent = battery
        message.active_command_id = command_id
        message.status_text = status_text
        self.state_pub.publish(message)

    def _mavlink_loop(self):
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                connection = mavutil.mavlink_connection(
                    self.mavlink_url, autoreconnect=True,
                    source_system=253, source_component=0)
                heartbeat = connection.wait_heartbeat(timeout=15.0)
                if heartbeat is None:
                    raise RuntimeError('ArduRover heartbeat timeout')
                with self.lock:
                    self.mav = connection
                    self.target_system = connection.target_system
                    self.target_component = connection.target_component
                    self.heartbeat = heartbeat
                    self.last_heartbeat = time.monotonic()
                    self.status_text = 'ArduRover connected; ready'
                self._request_telemetry()
                while not self.stop_event.is_set() and not rospy.is_shutdown():
                    self._receive_messages()
                    self._consume_command()
                    self._update_navigation()
                    self._update_return()
                    time.sleep(0.02)
            except Exception as error:
                with self.lock:
                    self.mav = None
                    self.status_text = 'ArduRover reconnecting: %s' % error
                rospy.logwarn_throttle(5.0, '%s: %s', self.vehicle_id, error)
                self.stop_event.wait(2.0)

    def _request_telemetry(self):
        for message_id, rate_hz in (
                (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
                (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2),
                (mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 5),
                (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 20),
                (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 10),
                (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5)):
            self.mav.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                message_id, 1000000.0 / rate_hz, 0, 0, 0, 0, 0)

    def _receive_messages(self):
        message = self.mav.recv_match(blocking=False)
        while message is not None:
            kind = message.get_type()
            with self.lock:
                if kind == 'HEARTBEAT':
                    self.heartbeat = message
                    self.last_heartbeat = time.monotonic()
                elif kind == 'LOCAL_POSITION_NED':
                    self.local_position = message
                elif kind == 'NAV_CONTROLLER_OUTPUT':
                    self.nav_controller_output = message
                    self.nav_controller_output_received_at = time.monotonic()
                elif kind == 'SYS_STATUS' and message.battery_remaining >= 0:
                    self.battery_percent = float(message.battery_remaining)
            if kind == 'STATUSTEXT':
                rospy.logwarn(
                    '%s ArduPilot STATUSTEXT severity=%s text=%s',
                    self.vehicle_id, message.severity, message.text,
                )
            elif kind == 'COMMAND_ACK':
                rospy.loginfo(
                    '%s ArduPilot COMMAND_ACK command=%s result=%s progress=%s',
                    self.vehicle_id, message.command, message.result,
                    getattr(message, 'progress', -1),
                )
            elif kind == 'HEARTBEAT':
                rospy.loginfo_throttle(
                    5.0,
                    '%s ArduPilot HEARTBEAT mode=%s armed=%s system_status=%s',
                    self.vehicle_id, mavutil.mode_string_v10(message),
                    bool(message.base_mode
                         & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                    message.system_status,
                )
            elif kind == 'ATTITUDE':
                rospy.loginfo_throttle(
                    2.0,
                    '%s ArduPilot ATTITUDE roll=%.2f pitch=%.2f yaw=%.2f deg',
                    self.vehicle_id, math.degrees(message.roll),
                    math.degrees(message.pitch), math.degrees(message.yaw),
                )
            elif kind == 'LOCAL_POSITION_NED':
                if not self.local_position_seen:
                    rospy.loginfo(
                        '%s ArduPilot LOCAL_POSITION_NED '
                        'x=%.3f y=%.3f z=%.3f vx=%.3f vy=%.3f vz=%.3f',
                        self.vehicle_id, message.x, message.y, message.z,
                        message.vx, message.vy, message.vz,
                    )
                    self.local_position_seen = True
            elif kind == 'SCALED_IMU':
                self._publish_scaled_imu(message)
            elif kind == 'SCALED_PRESSURE':
                self._publish_pressure(message)
            elif kind == 'GPS_RAW_INT':
                self._publish_gps(message)
            message = self.mav.recv_match(blocking=False)

    def _publish_scaled_imu(self, source):
        stamp = rospy.Time.now()
        frame_id = self.vehicle_id + '/base_link'

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = frame_id
        imu.orientation_covariance[0] = -1.0
        # MAVLink body axes are FRD; ROS body axes are FLU.
        imu.linear_acceleration.x = float(source.xacc) * 9.80665e-3
        imu.linear_acceleration.y = -float(source.yacc) * 9.80665e-3
        imu.linear_acceleration.z = -float(source.zacc) * 9.80665e-3
        imu.angular_velocity.x = float(source.xgyro) * 1.0e-3
        imu.angular_velocity.y = -float(source.ygyro) * 1.0e-3
        imu.angular_velocity.z = -float(source.zgyro) * 1.0e-3
        self.imu_pub.publish(imu)

        magnetic = MagneticField()
        magnetic.header.stamp = stamp
        magnetic.header.frame_id = frame_id
        # SCALED_IMU magnetometer values are milligauss; ROS uses tesla.
        magnetic.magnetic_field.x = float(source.xmag) * 1.0e-7
        magnetic.magnetic_field.y = -float(source.ymag) * 1.0e-7
        magnetic.magnetic_field.z = -float(source.zmag) * 1.0e-7
        self.magnetic_field_pub.publish(magnetic)

    def _publish_pressure(self, source):
        pressure = FluidPressure()
        pressure.header.stamp = rospy.Time.now()
        pressure.header.frame_id = self.vehicle_id + '/base_link'
        # SCALED_PRESSURE absolute pressure is hPa; ROS uses pascal.
        pressure.fluid_pressure = float(source.press_abs) * 100.0
        self.pressure_pub.publish(pressure)

    def _publish_gps(self, source):
        fix = NavSatFix()
        fix.header.stamp = rospy.Time.now()
        fix.header.frame_id = self.vehicle_id + '/gps_link'
        fix.status.status = (
            NavSatStatus.STATUS_FIX
            if int(source.fix_type) >= 2 else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(source.lat) * 1.0e-7
        fix.longitude = float(source.lon) * 1.0e-7
        fix.altitude = float(source.alt) * 1.0e-3
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(fix)

    def _set_mode(self, name):
        modes = self.mav.mode_mapping() or {}
        if name not in modes:
            raise RuntimeError('ArduRover mode unavailable: ' + name)
        self.mav.set_mode(modes[name])
        self.last_mode_request = time.monotonic()

    def _vehicle_armed(self):
        with self.lock:
            heartbeat = self.heartbeat
        return bool(
            heartbeat is not None
            and heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    def _arm(self):
        # ArduPilot defines 2989 as force-arm; 21196 is force-disarm.
        force_code = 2989 if self.force_arm else 0
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, force_code, 0, 0, 0, 0, 0)
        self.last_arm_request = time.monotonic()

    def _ensure_armed(self):
        with self.lock:
            heartbeat = self.heartbeat
        armed = bool(
            heartbeat is not None
            and heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        if (
            not armed
            and time.monotonic() - self.last_arm_request
            >= self.arm_retry_seconds
        ):
            self._arm()
        return armed

    def _ensure_guided(self):
        with self.lock:
            heartbeat = self.heartbeat
        mode = (
            mavutil.mode_string_v10(heartbeat)
            if heartbeat is not None else 'DISCONNECTED'
        )
        if (
            mode != 'GUIDED'
            and time.monotonic() - self.last_mode_request >= 1.0
        ):
            self._set_mode('GUIDED')

    def _consume_command(self):
        with self.lock:
            command = self.pending_command
            self.pending_command = None
        if command is None:
            return
        self.active_command_id = command.command_id
        if command.command_type == FleetCommand.COMMAND_NAVIGATE:
            with self.lock:
                local = self.local_position
                pose = self.pose
            if local is None or pose is None:
                self._ack(command.command_id, CommandAck.STATUS_REJECTED,
                          'ArduRover position not ready')
                self.active_command_id = ''
                return
            goal = command.target_pose.position
            self.guided_target = (local.x + goal.y - pose.position.y,
                                  local.y + goal.x - pose.position.x,
                                  0.0)
            self.goal_gazebo = (goal.x, goal.y)
            self._set_mode('GUIDED')
            self._set_navigation_speed()
            self._arm()
            with self.lock:
                self.operation = 'navigate'
                self.status_text = 'ArduRover navigating in GUIDED mode'
        elif command.command_type == FleetCommand.COMMAND_RETURN:
            if not self._vehicle_armed():
                with self.lock:
                    self.active_command_id = ''
                    self.status_text = 'ArduRover RTL requires armed vehicle'
                self._ack(
                    command.command_id,
                    CommandAck.STATUS_FAILED,
                    'ArduRover RTL requires an armed vehicle',
                )
                return
            try:
                self._set_mode('RTL')
            except RuntimeError as error:
                with self.lock:
                    self.active_command_id = ''
                    self.status_text = 'ArduRover RTL unavailable'
                self._ack(
                    command.command_id,
                    CommandAck.STATUS_FAILED,
                    str(error),
                )
                return
            now = time.monotonic()
            with self.lock:
                self.operation = 'return'
                self.guided_target = None
                self.goal_gazebo = None
                self.nav_controller_output = None
                self.nav_controller_output_received_at = 0.0
                self.operation_started_at = now
                self.operation_mode_seen = False
                self.last_progress_ack = now
                self.status_text = 'ArduRover returning home in RTL mode'
            self._ack(
                command.command_id,
                CommandAck.STATUS_EXECUTING,
                'ArduRover switching to RTL',
                0.05,
            )
        elif command.command_type in (
                FleetCommand.COMMAND_HOLD,
                FleetCommand.COMMAND_EMERGENCY_STOP):
            self._set_mode('HOLD')
            with self.lock:
                self.operation = 'hold'
                self.guided_target = None
                self.goal_gazebo = None
                self.active_command_id = ''
                self.status_text = 'ArduRover HOLD'
            self._ack(command.command_id, CommandAck.STATUS_SUCCEEDED,
                      'ArduRover holding', 1.0)
        else:
            self._ack(command.command_id, CommandAck.STATUS_REJECTED,
                      'unsupported ArduRover command')
            self.active_command_id = ''

    def _update_navigation(self):
        with self.lock:
            target = self.guided_target
            goal = self.goal_gazebo
            pose = self.pose
            command_id = self.active_command_id
        if target is None:
            return
        self._ensure_armed()
        self._ensure_guided()
        now = time.monotonic()
        if now - self.last_setpoint >= 0.5:
            self._send_local_target(*target)
            self.last_setpoint = now
        if pose is not None and goal is not None:
            distance = math.hypot(pose.position.x - goal[0],
                                  pose.position.y - goal[1])
            if distance <= self.arrival_tolerance:
                self._set_mode('HOLD')
                with self.lock:
                    self.operation = 'hold'
                    self.guided_target = None
                    self.goal_gazebo = None
                    self.active_command_id = ''
                    self.status_text = 'ArduRover target reached'
                self._ack(command_id, CommandAck.STATUS_SUCCEEDED,
                          'navigation target reached', 1.0)
            elif now - self.last_progress_ack >= 1.0:
                self._ack(command_id, CommandAck.STATUS_EXECUTING,
                          'distance %.1f m' % distance,
                          max(0.05, min(0.95, 1.0 - distance / 250.0)))
                self.last_progress_ack = now

    def _update_return(self):
        with self.lock:
            if self.operation != 'return':
                return
            heartbeat = self.heartbeat
            nav_output = self.nav_controller_output
            nav_received_at = self.nav_controller_output_received_at
            command_id = self.active_command_id
            started_at = self.operation_started_at
            mode_seen = self.operation_mode_seen

        now = time.monotonic()
        mode = (
            mavutil.mode_string_v10(heartbeat)
            if heartbeat is not None else 'DISCONNECTED'
        )
        if mode == 'RTL' and not mode_seen:
            mode_seen = True
            with self.lock:
                self.operation_mode_seen = True
                self.status_text = 'ArduRover RTL active'

        distance = None
        if (
            nav_output is not None
            and nav_received_at >= started_at
        ):
            candidate = float(nav_output.wp_dist)
            if math.isfinite(candidate):
                distance = candidate

        if (
            mode_seen
            and distance is not None
            and distance <= self.arrival_tolerance
        ):
            try:
                self._set_mode('HOLD')
            except RuntimeError as error:
                with self.lock:
                    self.operation = 'hold'
                    self.active_command_id = ''
                    self.status_text = 'ArduRover RTL arrived; HOLD failed'
                self._ack(
                    command_id,
                    CommandAck.STATUS_FAILED,
                    'ArduRover reached home but failed to enter HOLD: %s'
                    % error,
                )
                return
            with self.lock:
                self.operation = 'hold'
                self.active_command_id = ''
                self.status_text = 'ArduRover RTL complete; holding at home'
            self._ack(
                command_id,
                CommandAck.STATUS_SUCCEEDED,
                'ArduRover RTL destination reached; holding at home',
                1.0,
            )
        elif (
            not mode_seen
            and now - started_at >= self.mode_transition_timeout
        ):
            with self.lock:
                self.operation = 'hold'
                self.active_command_id = ''
                self.status_text = 'ArduRover failed to enter RTL'
            self._ack(
                command_id,
                CommandAck.STATUS_FAILED,
                'ArduRover did not enter RTL before timeout',
            )
        elif now - started_at >= self.return_timeout:
            with self.lock:
                self.operation = 'hold'
                self.active_command_id = ''
                self.status_text = 'ArduRover RTL timed out'
            self._ack(
                command_id,
                CommandAck.STATUS_FAILED,
                'ArduRover RTL timed out before reaching home',
            )
        elif now - self.last_progress_ack >= 1.0:
            detail = (
                'distance %.1f m' % distance
                if distance is not None else 'waiting for RTL distance'
            )
            self._ack(
                command_id,
                CommandAck.STATUS_EXECUTING,
                'ArduRover mode=%s; %s' % (mode, detail),
                0.5 if mode_seen else 0.1,
            )
            self.last_progress_ack = now

    def _send_local_target(self, north, east, down):
        mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        self.mav.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xffffffff,
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            north, east, down, 0, 0, 0, 0, 0, 0, 0, 0)

    def _set_navigation_speed(self):
        """Set Rover ground speed explicitly for every NAVIGATE command."""
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,  # ground speed
            self.navigation_speed,
            -1,  # leave throttle selection to ArduRover
            0, 0, 0, 0,
        )

    def shutdown(self):
        self.stop_event.set()
        self.worker.join(timeout=3.0)
        try:
            self.timer.shutdown()
            self.gz_node.unsubscribe(self.pose_topic)
        except Exception:
            pass


def main():
    rospy.init_node('usv_ardurover_fleet_agent')
    UsvArduRoverFleetAgent()
    rospy.spin()


if __name__ == '__main__':
    main()
