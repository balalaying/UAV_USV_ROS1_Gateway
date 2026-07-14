"""Small, ROS-independent helpers used by perception tracking nodes."""

import math
import uuid


TRACK_UUID_NAMESPACE = uuid.UUID('4d4f28be-c5ec-4da8-a828-77c95c50177e')


def planar_distance(first, second):
    """Return XY distance between two indexable positions."""
    return math.hypot(first[0] - second[0], first[1] - second[1])


def fused_confidence(confidences):
    """Combine independent confidences without exceeding one."""
    miss_probability = 1.0
    found = False
    for value in confidences:
        confidence = min(1.0, max(0.0, float(value)))
        miss_probability *= 1.0 - confidence
        found = True
    return 1.0 - miss_probability if found else 0.0


def stable_uuid_bytes(track_id):
    """Generate a repeatable UUID byte sequence for a stable track ID."""
    return list(uuid.uuid5(TRACK_UUID_NAMESPACE, str(track_id)).bytes)


def nearest_track(position, tracks, maximum_distance, excluded=()):
    """Find the closest active track using its stored XYZ position."""
    excluded = set(excluded)
    best_id = None
    best_distance = float(maximum_distance)
    for track_id, track in tracks.items():
        if track_id in excluded:
            continue
        distance = planar_distance(position, track.position)
        if distance <= best_distance:
            best_id = track_id
            best_distance = distance
    return best_id
