"""Pure helpers for the fleet world-model data contract."""

import math
import re


def quaternion_yaw(x, y, z, w):
    """Return the map-frame yaw represented by a quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def planar_course(vx, vy, fallback=0.0):
    """Return course over ground, preserving heading while nearly stationary."""
    if math.hypot(vx, vy) < 1.0e-6:
        return float(fallback)
    return math.atan2(vy, vx)


def stream_is_usable(stream, map_frame):
    """Only a fresh, map-frame stream may become authoritative."""
    if not isinstance(stream, dict):
        return False
    header = stream.get('header') or {}
    return bool(
        stream.get('online')
        and not stream.get('stale', True)
        and header.get('frame_id') == map_frame
    )


def normalize_sensor_sources(sensor_source, generic_sources=()):
    """Convert publisher metadata into stable, transport-friendly labels."""
    tokens = []
    for raw in re.split(r'[,+|; ]+', str(sensor_source or '').strip()):
        token = raw.strip().upper().replace('-', '_')
        if token:
            tokens.append(token)
    for source in generic_sources:
        token = str(source).strip().upper().replace('-', '_')
        if token and token not in tokens and token != 'FUSION':
            tokens.append(token)
    return list(dict.fromkeys(tokens)) or ['UNKNOWN']
