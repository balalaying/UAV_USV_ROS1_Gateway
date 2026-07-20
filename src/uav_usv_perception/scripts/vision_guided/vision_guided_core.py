"""ROS-independent algorithms for camera-guided maritime perception."""

from collections import deque
from dataclasses import dataclass, field
import math
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


AFFILIATION_UNKNOWN = 0
AFFILIATION_FRIENDLY = 1
AFFILIATION_HOSTILE = 2
AFFILIATION_NEUTRAL = 3


@dataclass
class AffiliationState:
    history: deque
    confirmed: int = AFFILIATION_UNKNOWN
    confidence: float = 0.0
    last_update: float = 0.0
    lost_frames: int = 0
    switches: int = 0
    reason: str = 'new'


class AffiliationTemporalFilter:
    """Debounce visual affiliation while preserving short camera dropouts."""

    def __init__(
        self,
        history_size=7,
        confirmation_frames=3,
        switch_frames=4,
        hold_seconds=1.0,
        unknown_timeout=2.0,
        min_confidence=0.55,
    ):
        self.history_size = max(1, int(history_size))
        self.confirmation_frames = max(1, int(confirmation_frames))
        self.switch_frames = max(self.confirmation_frames, int(switch_frames))
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.unknown_timeout = max(self.hold_seconds, float(unknown_timeout))
        self.min_confidence = float(min_confidence)
        self.states: Dict[str, AffiliationState] = {}
        self.confirmations = 0
        self.switches = 0

    def update(self, key, affiliation, confidence, now=None):
        now = time.monotonic() if now is None else float(now)
        state = self.states.get(key)
        if state is None:
            state = AffiliationState(deque(maxlen=self.history_size))
            self.states[key] = state
        state.last_update = now
        state.lost_frames = 0
        affiliation = int(affiliation)
        confidence = min(1.0, max(0.0, float(confidence)))
        if confidence < self.min_confidence:
            affiliation = AFFILIATION_UNKNOWN
        state.history.append((affiliation, confidence))
        scores = {value: 0.0 for value in range(4)}
        counts = {value: 0 for value in range(4)}
        for value, score in state.history:
            scores[value] += score
            counts[value] += 1
        candidate = max(range(1, 4), key=lambda value: scores[value])
        required = (
            self.confirmation_frames
            if state.confirmed in (AFFILIATION_UNKNOWN, candidate)
            else self.switch_frames
        )
        if counts[candidate] >= required:
            previous = state.confirmed
            state.confirmed = candidate
            state.confidence = scores[candidate] / max(1, counts[candidate])
            if previous == AFFILIATION_UNKNOWN:
                self.confirmations += 1
                state.reason = 'confirmed_%d_frames' % counts[candidate]
            elif previous != candidate:
                state.switches += 1
                self.switches += 1
                state.reason = 'switched_%d_to_%d' % (previous, candidate)
        elif state.confirmed == AFFILIATION_UNKNOWN:
            state.confidence = max(scores.values()) / max(1, len(state.history))
            state.reason = 'waiting_confirmation'
        return state.confirmed, state.confidence, state.reason

    def mark_missing(self, key, now=None):
        now = time.monotonic() if now is None else float(now)
        state = self.states.get(key)
        if state is None:
            return AFFILIATION_UNKNOWN, 0.0, 'missing_new'
        state.lost_frames += 1
        age = now - state.last_update
        if age <= self.hold_seconds:
            return state.confirmed, state.confidence, 'camera_hold'
        if age >= self.unknown_timeout:
            state.confirmed = AFFILIATION_UNKNOWN
            state.confidence = 0.0
            state.reason = 'camera_timeout'
        else:
            fraction = (age - self.hold_seconds) / max(
                1e-6, self.unknown_timeout - self.hold_seconds
            )
            state.confidence *= max(0.0, 1.0 - fraction)
            state.reason = 'camera_confidence_decay'
        return state.confirmed, state.confidence, state.reason

    def expire(self, active_keys, now=None):
        active = set(active_keys)
        return {
            key: self.mark_missing(key, now)
            for key in list(self.states)
            if key not in active
        }


def project_camera_points(points_camera, intrinsics):
    """Project x-forward/y-left/z-up camera points into image pixels."""
    points = np.asarray(points_camera, dtype=np.float64)
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    depth = points[:, 0]
    valid = np.isfinite(points).all(axis=1) & (depth > 1e-6)
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = cx - fx * points[valid, 1] / depth[valid]
    pixels[valid, 1] = cy - fy * points[valid, 2] / depth[valid]
    return pixels, valid


def unproject_camera_pixels(pixels, depths, intrinsics):
    """Unproject pixels into the Gazebo x-forward/y-left/z-up camera frame."""
    pixels = np.asarray(pixels, dtype=np.float64).reshape((-1, 2))
    depths = np.asarray(depths, dtype=np.float64).reshape((-1,))
    if len(pixels) != len(depths):
        raise ValueError('pixels and depths must have equal length')
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if abs(fx) < 1e-9 or abs(fy) < 1e-9:
        raise ValueError('camera focal lengths must be non-zero')
    points = np.empty((len(pixels), 3), dtype=np.float64)
    points[:, 0] = depths
    points[:, 1] = -(pixels[:, 0] - cx) * depths / fx
    points[:, 2] = -(pixels[:, 1] - cy) * depths / fy
    return points


def roi_indices(pixels, rectangle, expand=0.0, available=None):
    left, top, right, bottom = (float(value) for value in rectangle)
    valid = np.isfinite(pixels).all(axis=1)
    valid &= pixels[:, 0] >= left - expand
    valid &= pixels[:, 0] <= right + expand
    valid &= pixels[:, 1] >= top - expand
    valid &= pixels[:, 1] <= bottom + expand
    if available is not None:
        valid &= np.asarray(available, dtype=bool)
    return np.flatnonzero(valid)


def depth_filter(points_camera, low_percentile, high_percentile, threshold):
    points = np.asarray(points_camera, dtype=np.float64)
    if len(points) == 0:
        return np.zeros(0, dtype=bool)
    depth = points[:, 0]
    low, high = np.percentile(depth, [low_percentile, high_percentile])
    median = float(np.median(depth))
    mad = float(np.median(np.abs(depth - median)))
    robust = max(float(threshold), 3.5 * 1.4826 * mad)
    return (depth >= low) & (depth <= high) & (np.abs(depth - median) <= robust)


def dbscan(points, epsilon, min_samples):
    """Bounded DBSCAN with a 3D spatial hash for ROI-scale point sets."""
    points = np.asarray(points, dtype=np.float64)
    count = len(points)
    labels = np.full(count, -1, dtype=np.int32)
    if count == 0:
        return labels
    epsilon = max(1e-6, float(epsilon))
    epsilon_sq = epsilon ** 2
    min_samples = max(1, int(min_samples))
    cells = np.floor(points / epsilon).astype(np.int64)
    buckets = {}
    for index, cell in enumerate(cells):
        key = tuple(int(value) for value in cell)
        buckets.setdefault(key, []).append(index)
    offsets = tuple(
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    )

    def neighborhood(index):
        cell = cells[index]
        candidates = []
        for dx, dy, dz in offsets:
            candidates.extend(buckets.get((
                int(cell[0] + dx),
                int(cell[1] + dy),
                int(cell[2] + dz),
            ), ()))
        candidates = np.asarray(candidates, dtype=np.int64)
        delta = points[candidates] - points[index]
        squared = np.einsum('ij,ij->i', delta, delta)
        return candidates[squared <= epsilon_sq]

    visited = np.zeros(count, dtype=bool)
    cluster_id = 0
    for seed in range(count):
        if visited[seed]:
            continue
        visited[seed] = True
        neighbors = neighborhood(seed)
        if len(neighbors) < min_samples:
            continue
        labels[seed] = cluster_id
        queue = list(neighbors)
        queued = set(queue)
        cursor = 0
        while cursor < len(queue):
            index = queue[cursor]
            cursor += 1
            if not visited[index]:
                visited[index] = True
                nearby = neighborhood(index)
                if len(nearby) >= min_samples:
                    for item in nearby:
                        item = int(item)
                        if item not in queued:
                            queue.append(item)
                            queued.add(item)
            if labels[index] < 0:
                labels[index] = cluster_id
        cluster_id += 1
    return labels


def robust_oriented_bbox(points, low=5.0, high=95.0):
    """Return a percentile/PCA oriented bbox in the input frame."""
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 3:
        return None
    xy = points[:, :2]
    median = np.median(xy, axis=0)
    radial = np.linalg.norm(xy - median, axis=1)
    radial_median = float(np.median(radial))
    radial_mad = float(np.median(np.abs(radial - radial_median)))
    radial_gate = radial_median + max(0.25, 4.5 * 1.4826 * radial_mad)
    robust_points = points[radial <= radial_gate]
    if len(robust_points) < 3:
        robust_points = points
    xy = robust_points[:, :2]
    median = np.median(xy, axis=0)
    centered = xy - median
    covariance = np.cov(centered.T)
    values, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, int(np.argmax(values))]
    if direction[0] < 0.0:
        direction *= -1.0
    lateral = np.asarray((-direction[1], direction[0]))
    axes = np.column_stack((direction, lateral))
    projected = centered @ axes
    mins = np.percentile(projected, low, axis=0)
    maxs = np.percentile(projected, high, axis=0)
    z_min, z_max = np.percentile(robust_points[:, 2], [low, high])
    local_center = 0.5 * (mins + maxs)
    center_xy = median + axes @ local_center
    dimensions = np.asarray((
        maxs[0] - mins[0], maxs[1] - mins[1], z_max - z_min
    ))
    return {
        'center': np.asarray((center_xy[0], center_xy[1], 0.5 * (z_min + z_max))),
        'dimensions': dimensions,
        'yaw': math.atan2(direction[1], direction[0]),
        'point_count': int(len(robust_points)),
    }


def bbox_is_plausible(box, limits):
    if box is None or not np.isfinite(box['dimensions']).all():
        return False
    length, width = sorted(box['dimensions'][:2], reverse=True)
    height = box['dimensions'][2]
    return (
        limits['minimum_length'] <= length <= limits['maximum_length']
        and limits['minimum_width'] <= width <= limits['maximum_width']
        and limits['minimum_height'] <= height <= limits['maximum_height']
    )


def complete_occluded_bbox(box, sensor_position, minimum_occluded_extent):
    """Give a single visible surface a conservative thickness behind it."""
    if box is None:
        return None
    extent = max(0.0, float(minimum_occluded_extent))
    if box['dimensions'][1] >= extent:
        return box
    result = dict(box)
    result['center'] = np.asarray(box['center'], dtype=np.float64).copy()
    result['dimensions'] = np.asarray(
        box['dimensions'], dtype=np.float64
    ).copy()
    missing = extent - result['dimensions'][1]
    minor_axis = np.asarray((-math.sin(box['yaw']), math.cos(box['yaw'])))
    radial = result['center'][:2] - np.asarray(sensor_position)[:2]
    if float(np.dot(minor_axis, radial)) < 0.0:
        minor_axis *= -1.0
    result['center'][:2] += 0.5 * missing * minor_axis
    result['dimensions'][1] = extent
    result['occlusion_completed'] = True
    return result


def select_cluster(points, labels, predicted_position=None):
    """Select one cluster using point support and optional predicted center."""
    best = None
    best_score = -math.inf
    for label in sorted(set(int(value) for value in labels if value >= 0)):
        indices = np.flatnonzero(labels == label)
        center = np.median(points[indices], axis=0)
        score = math.log1p(len(indices))
        if predicted_position is not None:
            distance = np.linalg.norm(center[:2] - np.asarray(predicted_position)[:2])
            score -= 0.5 * distance
        if score > best_score:
            best = indices
            best_score = score
    return best, best_score


class BboxSmoother:
    def __init__(
        self, alpha=0.45, maximum_jump=6.0,
        jump_confirmation_frames=3,
    ):
        self.alpha = min(1.0, max(0.0, float(alpha)))
        self.maximum_jump = max(0.0, float(maximum_jump))
        self.jump_confirmation_frames = max(1, int(jump_confirmation_frames))
        self.boxes = {}
        self.pending = {}
        self.pending_counts = {}

    def update(self, key, box):
        previous = self.boxes.get(key)
        if previous is None:
            self.boxes[key] = box
            return box, True
        jump = np.linalg.norm(box['center'][:2] - previous['center'][:2])
        if jump > self.maximum_jump:
            pending = self.pending.get(key)
            if (
                pending is not None
                and np.linalg.norm(
                    box['center'][:2] - pending['center'][:2]
                ) <= self.maximum_jump
            ):
                self.pending_counts[key] += 1
            else:
                self.pending[key] = box
                self.pending_counts[key] = 1
            if self.pending_counts[key] >= self.jump_confirmation_frames:
                self.boxes[key] = box
                self.pending.pop(key, None)
                self.pending_counts.pop(key, None)
                return box, True
            return previous, False
        self.pending.pop(key, None)
        self.pending_counts.pop(key, None)
        result = dict(box)
        result['center'] = (
            self.alpha * box['center'] + (1.0 - self.alpha) * previous['center']
        )
        result['dimensions'] = (
            self.alpha * box['dimensions']
            + (1.0 - self.alpha) * previous['dimensions']
        )
        delta = math.atan2(math.sin(box['yaw'] - previous['yaw']),
                           math.cos(box['yaw'] - previous['yaw']))
        result['yaw'] = previous['yaw'] + self.alpha * delta
        self.boxes[key] = result
        return result, True
