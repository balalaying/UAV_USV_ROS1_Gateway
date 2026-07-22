"""Convert ROS messages into transport-neutral models."""

import math

from .models import SensorModel, TargetModel, VehicleModel


VEHICLE_TYPES = {0: 'UNKNOWN', 1: 'UAV', 2: 'USV'}
CLASS_NAMES = {
    0: None,
    1: 'vessel',
    2: 'buoy',
    3: 'debris',
    4: 'landmark',
}
SOURCE_NAMES = {
    0: 'unknown',
    1: 'lidar',
    2: 'camera',
    4: 'ais',
    8: 'fusion',
}


def stamp_dict(stamp):
    sec = int(getattr(stamp, 'sec', 0))
    nanosec = int(getattr(stamp, 'nanosec', 0))
    return {
        'sec': sec,
        'nanosec': nanosec,
        'seconds': float(sec) + float(nanosec) * 1e-9,
    }


def quaternion_to_euler(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return None, None, None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sinp)
        if abs(sinp) >= 1 else math.asin(sinp)
    )
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _finite(value, unavailable_negative=False):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number) or (unavailable_negative and number < 0.0):
        return None
    return number


def source_name(mask):
    names = [name for bit, name in SOURCE_NAMES.items() if bit and mask & bit]
    return '+'.join(names) if names else SOURCE_NAMES.get(mask, 'unknown')


def vehicle_from_ros(message, received_at):
    pose = message.pose
    twist = message.twist
    quaternion = pose.orientation
    roll, pitch, yaw = quaternion_to_euler(
        float(quaternion.x), float(quaternion.y),
        float(quaternion.z), float(quaternion.w))
    linear = twist.linear
    angular = twist.angular
    vx, vy, vz = float(linear.x), float(linear.y), float(linear.z)
    vehicle_id = str(message.vehicle_id)
    return VehicleModel(
        id=vehicle_id,
        type=VEHICLE_TYPES.get(int(message.vehicle_type), 'UNKNOWN'),
        namespace='/' + vehicle_id.strip('/'),
        online=bool(message.online),
        last_update=stamp_dict(message.header.stamp)['seconds'],
        frame_id=str(message.header.frame_id),
        position={
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
        },
        orientation={
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'qx': float(quaternion.x), 'qy': float(quaternion.y),
            'qz': float(quaternion.z), 'qw': float(quaternion.w),
        },
        linear_velocity={'x': vx, 'y': vy, 'z': vz},
        angular_velocity={
            'x': float(angular.x), 'y': float(angular.y),
            'z': float(angular.z),
        },
        speed=math.sqrt(vx * vx + vy * vy + vz * vz),
        battery={
            'percentage': _finite(message.battery_percent, True),
            'voltage': None,
        },
        mode=str(message.mode) or None,
        armed=bool(message.armed),
        health={
            'status_text': str(message.status_text) or None,
            'active_command_id': str(message.active_command_id) or None,
        },
        received_at=float(received_at),
    )


def target_from_ros(message, header, received_at, formal_source='source_mux'):
    tracked_stamp = stamp_dict(message.last_update)
    if tracked_stamp['seconds'] <= 0.0:
        tracked_stamp = stamp_dict(header.stamp)
    pose = message.pose.pose
    twist = message.twist.twist
    _, _, yaw = quaternion_to_euler(
        float(pose.orientation.x), float(pose.orientation.y),
        float(pose.orientation.z), float(pose.orientation.w))
    class_name = str(getattr(message, 'class_name', '')) or CLASS_NAMES.get(
        int(message.classification))
    sensor_source = str(getattr(message, 'sensor_source', '')) or source_name(
        int(message.source_mask))
    return TargetModel(
        track_id=str(message.track_id),
        class_name=class_name,
        confidence=_finite(message.confidence),
        timestamp=tracked_stamp['seconds'],
        stamp=tracked_stamp,
        frame_id=str(header.frame_id),
        source=formal_source,
        sensor_source=sensor_source,
        position={
            'x': float(pose.position.x), 'y': float(pose.position.y),
            'z': float(pose.position.z),
        },
        velocity={
            'x': float(twist.linear.x), 'y': float(twist.linear.y),
            'z': float(twist.linear.z),
        },
        bbox={
            'length': _finite(message.dimensions.x),
            'width': _finite(message.dimensions.y),
            'height': _finite(message.dimensions.z),
            'yaw': yaw,
        },
        received_at=float(received_at),
    )


def sensor_from_ros(message, received_at):
    last_stamp = stamp_dict(message.last_message_time)
    message_type = str(message.message_type).lower()
    sensor_id = str(message.sensor_id)
    sensor_type = 'lidar' if any(
        token in (sensor_id + message_type).lower()
        for token in ('lidar', 'mid360', 'pointcloud')
    ) else (
        'camera'
        if 'image' in message_type or 'camera' in sensor_id.lower()
        else message_type or 'unknown'
    )
    healthy = bool(message.healthy) and not bool(message.timed_out)
    return SensorModel(
        vehicle_id=str(message.vehicle_id),
        sensor_id=sensor_id,
        sensor_type=sensor_type,
        online=healthy,
        frequency_hz=_finite(message.measured_rate_hz),
        last_update=last_stamp['seconds'],
        frame_id=str(message.frame_id),
        status=(
            'OK' if healthy
            else ('TIMEOUT' if message.timed_out else 'ERROR')
        ),
        topic=str(message.uplink_topic) or None,
        latency_sec=_finite(message.latency_seconds),
        point_count=int(message.point_count) if message.point_count else None,
        dropped_messages=int(message.dropped_messages),
        received_at=float(received_at),
    )


def mission_from_ros(message):
    return {
        'name': 'dynamic_capture',
        'state': str(message.state_name),
        'state_code': int(message.state),
        'target_id': str(message.target_id) or None,
        'reason': str(message.reason) or None,
        'configured_uavs': int(message.configured_uavs),
        'configured_usvs': int(message.configured_usvs),
        'active_uavs': int(message.active_uavs),
        'active_usvs': int(message.active_usvs),
        'allocation_generation': int(message.allocation_generation),
        'degraded': bool(message.degraded),
        'timestamp': stamp_dict(message.header.stamp),
        'frame_id': str(message.header.frame_id),
    }
