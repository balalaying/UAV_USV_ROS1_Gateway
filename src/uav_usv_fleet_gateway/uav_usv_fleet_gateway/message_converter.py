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


def _vector_from_dict(value):
    value = value or {}
    return {
        'x': _finite(value.get('x')),
        'y': _finite(value.get('y')),
        'z': _finite(value.get('z')),
    }


def _stamp_seconds(value):
    value = value or {}
    if 'seconds' in value:
        return _finite(value.get('seconds')) or 0.0
    return (
        float(value.get('sec', 0) or 0)
        + float(value.get('nanosec', 0) or 0) * 1e-9
    )


def vehicle_from_world_model(item, received_at):
    pose = item.get('pose') or {}
    position = _vector_from_dict(pose.get('position'))
    orientation = pose.get('orientation') or {}
    qx = _finite(orientation.get('x')) or 0.0
    qy = _finite(orientation.get('y')) or 0.0
    qz = _finite(orientation.get('z')) or 0.0
    qw = _finite(orientation.get('w')) or 1.0
    roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
    velocity = item.get('velocity') or {}
    linear = _vector_from_dict(velocity.get('linear'))
    angular = _vector_from_dict(velocity.get('angular'))
    vehicle_id = str(item.get('id') or '')
    health = dict(item.get('health') or {})
    health['state_source'] = str(item.get('state_source') or 'world_model')
    header = item.get('header') or {}
    stamp = (header.get('stamp') or item.get('last_update') or {})
    speed = item.get('speed_mps')
    if speed is None:
        vx = linear.get('x') or 0.0
        vy = linear.get('y') or 0.0
        vz = linear.get('z') or 0.0
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    return VehicleModel(
        id=vehicle_id,
        type=str(item.get('type') or 'UNKNOWN'),
        namespace='/' + vehicle_id.strip('/'),
        online=bool(item.get('online')),
        stale=bool(item.get('stale')),
        last_update=_stamp_seconds(stamp),
        frame_id=str(header.get('frame_id') or 'map'),
        position=position,
        orientation={
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
        },
        linear_velocity=linear,
        angular_velocity=angular,
        speed=_finite(speed),
        battery={
            'percentage': _finite(item.get('battery_percent'), True),
            'voltage': None,
        },
        mode=str(item.get('mode') or '') or None,
        armed=(
            bool(item.get('armed'))
            if item.get('armed') is not None else None
        ),
        health=health,
        state_source=str(item.get('state_source') or 'world_model'),
        received_at=float(received_at),
    )


def target_from_world_model(item, received_at, formal_source='world_model'):
    pose = item.get('pose') or {}
    position = _vector_from_dict(pose.get('position'))
    orientation = pose.get('orientation') or {}
    _, _, yaw = quaternion_to_euler(
        _finite(orientation.get('x')) or 0.0,
        _finite(orientation.get('y')) or 0.0,
        _finite(orientation.get('z')) or 0.0,
        _finite(orientation.get('w')) or 1.0,
    )
    velocity = _vector_from_dict(
        (item.get('velocity') or {}).get('linear')
    )
    dimensions = item.get('dimensions') or {}
    header = item.get('header') or {}
    stamp = (
        item.get('last_update')
        or item.get('header_stamp')
        or header.get('stamp')
        or {}
    )
    source = item.get('source')
    if isinstance(source, list):
        sensor_source = '+'.join(str(value) for value in source)
    else:
        sensor_source = str(
            item.get('sensor_source') or source or formal_source
        )
    return TargetModel(
        track_id=str(
            item.get('id') or item.get('track_id') or item.get('uuid') or ''
        ),
        class_name=str(
            item.get('class') or item.get('classification') or ''
        ) or None,
        confidence=_finite(item.get('confidence')),
        timestamp=_stamp_seconds(stamp),
        stamp={
            'sec': int((stamp or {}).get('sec', 0) or 0),
            'nanosec': int((stamp or {}).get('nanosec', 0) or 0),
            'seconds': _stamp_seconds(stamp),
        },
        frame_id=str(header.get('frame_id') or item.get('frame_id') or 'map'),
        source=str(formal_source),
        sensor_source=sensor_source,
        position=position,
        velocity=velocity,
        bbox={
            'length': _finite(dimensions.get('x')),
            'width': _finite(dimensions.get('y')),
            'height': _finite(dimensions.get('z')),
            'yaw': yaw,
        },
        received_at=float(received_at),
    )


def sensor_from_world_model(item, received_at):
    last_stamp = item.get('last_message_time') or {}
    return SensorModel(
        vehicle_id=str(item.get('vehicle_id') or ''),
        sensor_id=str(item.get('sensor_id') or ''),
        sensor_type=str(item.get('message_type') or 'unknown').lower(),
        online=bool(item.get('healthy')) and not bool(item.get('timed_out')),
        frequency_hz=_finite(item.get('rate_hz')),
        last_update=_stamp_seconds(last_stamp),
        frame_id=str(item.get('frame_id') or ''),
        status='OK' if item.get('healthy') else 'ERROR',
        topic=str(item.get('uplink_topic') or '') or None,
        latency_sec=_finite(item.get('latency_seconds')),
        point_count=int(item.get('point_count') or 0) or None,
        dropped_messages=int(item.get('dropped_messages') or 0),
        received_at=float(received_at),
    )


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
