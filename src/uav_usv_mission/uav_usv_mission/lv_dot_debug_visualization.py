"""Thread-safe model and OpenGL widget for the LV-DOT debug pipeline."""

from collections import deque
from copy import deepcopy
import json
import math
import threading
import time

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from visualization_msgs.msg import Marker

from uav_usv_mission.perception_topdown import marker_segments_3d
from uav_usv_mission.perception_topdown import tracked_object_dict


DEBUG_STYLE = {
    'raw': {'color': (1.0, 1.0, 1.0, 0.58), 'size': 1.7},
    'filtered': {'color': (1.0, 1.0, 1.0, 0.92), 'size': 2.2},
    'clusters': {'color': (0.95, 0.28, 0.78, 1.0), 'size': 9.0},
    'bboxes': {'color': (1.0, 0.72, 0.15, 1.0), 'width': 2.0},
    'lidar_only_bboxes': {
        'color': (1.0, 0.75, 0.12, 1.0), 'width': 2.5,
    },
    'camera_only_bboxes': {
        'color': (0.15, 0.45, 1.0, 1.0), 'width': 2.5,
    },
    'camera_lidar_fused_bboxes': {
        'color': (0.10, 1.0, 0.25, 1.0), 'width': 3.0,
    },
    'calibration_roi': {'color': (0.10, 1.0, 0.20, 1.0), 'size': 4.0},
    'camera_projection': {
        'color': (1.0, 0.05, 0.05, 1.0), 'width': 3.0,
    },
    'calibration_bbox': {
        'color': (1.0, 0.85, 0.05, 1.0), 'width': 3.0,
    },
    'tracks': {'color': (1.0, 0.85, 0.18, 1.0), 'size': 10.0},
    'dynamic': {'color': (1.0, 0.18, 0.28, 1.0), 'size': 12.0},
    'fusion': {'color': (0.20, 1.0, 0.45, 1.0), 'size': 13.0},
}

TF_COLORS = {
    'x': (1.0, 0.18, 0.18, 1.0),
    'y': (0.18, 1.0, 0.30, 1.0),
    'z': (0.20, 0.50, 1.0, 1.0),
}

AFFILIATION_COLORS = {
    0: (1.0, 0.75, 0.12, 1.0),
    1: (0.10, 0.75, 1.0, 1.0),
    2: (1.0, 0.12, 0.12, 1.0),
    3: (0.62, 0.68, 0.70, 1.0),
}


class LvDotDebugModel:
    TRACK_LAYERS = ('tracks', 'dynamic', 'fusion')

    def __init__(self, history_length=160):
        self._lock = threading.Lock()
        self._generation = 0
        self._clouds = {
            'raw': np.empty((0, 3), dtype=np.float32),
            'filtered': np.empty((0, 3), dtype=np.float32),
            'calibration_roi': np.empty((0, 3), dtype=np.float32),
        }
        self._cloud_received = {
            'raw': 0.0, 'filtered': 0.0, 'calibration_roi': 0.0,
        }
        self._clusters = {}
        self._bboxes = {}
        self._association_bboxes = {
            layer: {} for layer in (
                'lidar_only_bboxes',
                'camera_only_bboxes',
                'camera_lidar_fused_bboxes',
                'camera_projection',
                'calibration_bbox',
            )
        }
        self._tracks = {layer: {} for layer in self.TRACK_LAYERS}
        self._frames = {}
        self._histories = {}
        self._status = {}
        self._history_length = max(10, int(history_length))

    def update_cloud(self, layer, points):
        if layer not in self._clouds:
            return
        values = np.asarray(points, dtype=np.float32).reshape((-1, 3))
        with self._lock:
            self._clouds[layer] = values
            self._cloud_received[layer] = time.monotonic()
            self._generation += 1

    def update_markers(self, layer, message, frame_id='map'):
        if layer not in (
            'clusters', 'bboxes',
            'lidar_only_bboxes', 'camera_only_bboxes',
            'camera_lidar_fused_bboxes', 'camera_projection',
            'calibration_bbox',
        ):
            return
        if layer == 'clusters':
            target = self._clusters
        elif layer == 'bboxes':
            target = self._bboxes
        else:
            target = self._association_bboxes[layer]
        now = time.monotonic()
        with self._lock:
            for marker in message.markers:
                if marker.action == Marker.DELETEALL:
                    target.clear()
                    continue
                key = (marker.ns, int(marker.id))
                if marker.action == Marker.DELETE:
                    target.pop(key, None)
                    continue
                if marker.action != Marker.ADD:
                    continue
                if marker.header.frame_id and marker.header.frame_id != frame_id:
                    continue
                if layer == 'clusters':
                    target[key] = {
                        'id': int(marker.id),
                        'x': float(marker.pose.position.x),
                        'y': float(marker.pose.position.y),
                        'z': float(marker.pose.position.z),
                        'text': marker.text,
                        'metadata': self._marker_metadata(marker.text),
                        'received_at': now,
                    }
                else:
                    segments = marker_segments_3d(marker)
                    if not segments:
                        continue
                    target[key] = {
                        'id': int(marker.id),
                        'segments': segments,
                        'text': marker.text,
                        'metadata': self._marker_metadata(marker.text),
                        'received_at': now,
                    }
            self._generation += 1

    @staticmethod
    def _marker_metadata(text):
        if not text:
            return {}
        try:
            value = json.loads(text)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            return {}

    def update_tracks(self, layer, message):
        if layer not in self._tracks:
            return
        now = time.monotonic()
        values = {
            obj.track_id: tracked_object_dict(
                obj, now,
                dynamic=(layer == 'dynamic'),
                fused=(layer == 'fusion'),
            )
            for obj in message.objects if obj.track_id
        }
        with self._lock:
            self._tracks[layer] = values
            for track_id, item in values.items():
                key = (layer, track_id)
                history = self._histories.setdefault(
                    key, deque(maxlen=self._history_length)
                )
                point = (item['x'], item['y'], item['z'])
                if not history or math.dist(point, history[-1]) >= 0.03:
                    history.append(point)
            self._generation += 1

    def update_status(self, status):
        with self._lock:
            self._status = dict(status)
            self._status['_received_at'] = time.monotonic()
            self._generation += 1

    def update_frame(self, key, frame_id, transform):
        translation = transform.translation
        rotation = transform.rotation
        with self._lock:
            self._frames[key] = {
                'frame_id': str(frame_id),
                'x': float(translation.x),
                'y': float(translation.y),
                'z': float(translation.z),
                'qx': float(rotation.x),
                'qy': float(rotation.y),
                'qz': float(rotation.z),
                'qw': float(rotation.w),
                'received_at': time.monotonic(),
            }
            self._generation += 1

    def clear_histories(self):
        with self._lock:
            self._histories.clear()
            self._generation += 1

    def snapshot(self):
        with self._lock:
            return {
                'generation': self._generation,
                'clouds': {
                    key: value.copy() for key, value in self._clouds.items()
                },
                'cloud_received': dict(self._cloud_received),
                'clusters': deepcopy(list(self._clusters.values())),
                'bboxes': deepcopy(list(self._bboxes.values())),
                'association_bboxes': {
                    key: deepcopy(list(value.values()))
                    for key, value in self._association_bboxes.items()
                },
                'tracks': deepcopy(self._tracks),
                'frames': deepcopy(self._frames),
                'histories': {
                    key: list(value) for key, value in self._histories.items()
                },
                'status': deepcopy(self._status),
            }


class LvDotDebugWidget(QWidget):
    """Passive map-frame top-view renderer for all LV-DOT stages."""

    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        self.visibility = {
            key: True for key in (
                'raw', 'filtered', 'clusters', 'bboxes',
                'lidar_only_bboxes', 'camera_only_bboxes',
                'camera_lidar_fused_bboxes', 'calibration_roi',
                'camera_projection', 'calibration_bbox',
                'tracks', 'dynamic', 'fusion', 'labels', 'grid',
                'tf',
            )
        }
        self.max_points = {
            'raw': 60000, 'filtered': 60000, 'calibration_roi': 10000,
        }
        self.trajectory_length = 100
        self.last_generation = -1
        self.last_render_at = 0.0
        self.render_intervals = deque(maxlen=100)
        self.last_render_ms = 0.0
        self.last_counts = {}
        self.labels = []
        self.color_mode = 'sensor_source'
        self.view_mode = 'oblique'
        self.view_center = np.zeros(3, dtype=np.float32)
        self.auto_center_pending = True

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor('#05090d')
        layout.addWidget(self.view)

        self.items = {}
        grid = gl.GLGridItem()
        grid.setSize(160.0, 160.0)
        grid.setSpacing(10.0, 10.0)
        grid.setColor((70, 88, 102, 150))
        self.view.addItem(grid)
        self.items['grid'] = grid
        for layer in ('raw', 'filtered', 'calibration_roi', 'clusters'):
            style = DEBUG_STYLE[layer]
            item = gl.GLScatterPlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=style['color'], size=style['size'], pxMode=True,
            )
            self.items[layer] = item
            self.view.addItem(item)
        bbox = gl.GLLinePlotItem(
            pos=np.empty((0, 3), dtype=np.float32),
            color=DEBUG_STYLE['bboxes']['color'],
            width=DEBUG_STYLE['bboxes']['width'], mode='lines',
            antialias=True,
        )
        self.items['bboxes'] = bbox
        self.view.addItem(bbox)
        for layer in (
            'lidar_only_bboxes',
            'camera_only_bboxes',
            'camera_lidar_fused_bboxes',
            'camera_projection',
            'calibration_bbox',
        ):
            style = DEBUG_STYLE[layer]
            item = gl.GLLinePlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=style['color'], width=style['width'], mode='lines',
                antialias=True,
            )
            self.items[layer] = item
            self.view.addItem(item)
        for axis in ('x', 'y', 'z'):
            tf_axis = gl.GLLinePlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=TF_COLORS[axis], width=3.0, mode='lines',
                antialias=True,
            )
            self.items['tf_' + axis] = tf_axis
            self.view.addItem(tf_axis)
        for layer in ('tracks', 'dynamic', 'fusion'):
            style = DEBUG_STYLE[layer]
            point = gl.GLScatterPlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=style['color'], size=style['size'], pxMode=True,
            )
            trail = gl.GLLinePlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=style['color'], width=2.0, mode='line_strip',
                antialias=True,
            )
            velocity = gl.GLLinePlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=style['color'], width=2.0, mode='lines',
                antialias=True,
            )
            self.items[layer] = point
            self.items[layer + '_trail'] = trail
            self.items[layer + '_velocity'] = velocity
            self.view.addItem(trail)
            self.view.addItem(velocity)
            self.view.addItem(point)
        self.reset_view()
        self.timer = QTimer(self)
        self.timer.setInterval(40)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    @staticmethod
    def _bounded(points, maximum):
        if len(points) <= maximum:
            return points
        indices = np.linspace(0, len(points) - 1, maximum, dtype=np.int64)
        return points[indices]

    def _clear_labels(self):
        for label in self.labels:
            self.view.removeItem(label)
        self.labels.clear()

    def _label(self, item, text, color):
        if not self.visibility['labels']:
            return
        label = gl.GLTextItem(
            pos=(item['x'] + 0.4, item['y'] + 0.4, item['z'] + 0.8),
            color=pg.mkColor(color), text=text,
            font=QFont('Sans Serif', 10),
        )
        self.view.addItem(label)
        self.labels.append(label)

    def _update_tracks(self, layer, tracks, histories, now):
        active = [
            item for item in tracks.values()
            if now - item.get('received_at', 0.0) < 2.0
        ] if self.visibility[layer] else []
        positions = np.asarray([
            (item['x'], item['y'], item['z']) for item in active
        ], dtype=np.float32).reshape((-1, 3))
        self.items[layer].setData(pos=positions)
        trail_data = []
        velocity_data = []
        for item in active:
            history = histories.get((layer, item['track_id']), [])[
                -self.trajectory_length:
            ]
            if len(history) > 1:
                if trail_data:
                    trail_data.append((np.nan, np.nan, np.nan))
                trail_data.extend(history)
            velocity_data.extend((
                (item['x'], item['y'], item['z']),
                (
                    item['x'] + 2.0 * item['vx'],
                    item['y'] + 2.0 * item['vy'],
                    item['z'],
                ),
            ))
            state = (
                'CONFIRMED_DYNAMIC' if layer == 'dynamic'
                else 'FUSED' if layer == 'fusion' else 'TRACKED'
            )
            self._label(
                item, '%s [%s]' % (item['track_id'], state),
                DEBUG_STYLE[layer]['color'],
            )
        self.items[layer + '_trail'].setData(pos=np.asarray(
            trail_data, dtype=np.float32
        ).reshape((-1, 3)))
        self.items[layer + '_velocity'].setData(pos=np.asarray(
            velocity_data, dtype=np.float32
        ).reshape((-1, 3)))
        return len(active)

    @staticmethod
    def _rotation_matrix(frame):
        x = frame['qx']
        y = frame['qy']
        z = frame['qz']
        w = frame['qw']
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            return np.eye(3, dtype=np.float32)
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        return np.asarray([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float32)

    def _update_tf(self, frames, now):
        axis_lines = {'x': [], 'y': [], 'z': []}
        active = []
        if self.visibility['tf']:
            active = [
                (key, frame) for key, frame in frames.items()
                if now - frame.get('received_at', 0.0) < 2.0
            ]
        basis = {
            'x': np.asarray((2.5, 0.0, 0.0), dtype=np.float32),
            'y': np.asarray((0.0, 2.5, 0.0), dtype=np.float32),
            'z': np.asarray((0.0, 0.0, 2.5), dtype=np.float32),
        }
        for key, frame in active:
            origin = np.asarray(
                (frame['x'], frame['y'], frame['z']), dtype=np.float32
            )
            rotation = self._rotation_matrix(frame)
            for axis, vector in basis.items():
                axis_lines[axis].extend((origin, origin + rotation @ vector))
            label_item = dict(frame)
            if key == 'radar':
                label_item['x'] += 0.5
                label_item['y'] += 0.5
            else:
                label_item['x'] -= 1.5
                label_item['y'] -= 0.8
            self._label(
                label_item,
                'MID-360 TF' if key == 'radar' else 'USV-01 base TF',
                (0.92, 0.96, 1.0, 1.0),
            )
        for axis, points in axis_lines.items():
            self.items['tf_' + axis].setData(pos=np.asarray(
                points, dtype=np.float32
            ).reshape((-1, 3)))
        return len(active)

    def refresh(self):
        started = time.perf_counter()
        snapshot = self.model.snapshot()
        if snapshot['generation'] == self.last_generation:
            return
        self.last_generation = snapshot['generation']
        now = time.monotonic()
        self._clear_labels()

        counts = {}
        for layer in ('raw', 'filtered', 'calibration_roi'):
            fresh = now - snapshot['cloud_received'][layer] < 2.0
            points = snapshot['clouds'][layer] if (
                self.visibility[layer] and fresh
            ) else np.empty((0, 3), dtype=np.float32)
            points = self._bounded(points, self.max_points[layer])
            self.items[layer].setData(pos=points)
            counts[layer] = len(points)

        clusters = [
            item for item in snapshot['clusters']
            if self.visibility['clusters']
            and now - item['received_at'] < 2.0
        ]
        self.items['clusters'].setData(pos=np.asarray([
            (item['x'], item['y'], item['z']) for item in clusters
        ], dtype=np.float32).reshape((-1, 3)))
        counts['clusters'] = len(clusters)

        bbox_lines = []
        bboxes = [
            item for item in snapshot['bboxes']
            if self.visibility['bboxes']
            and now - item['received_at'] < 2.0
        ]
        for item in bboxes:
            bbox_lines.extend(
                point for segment in item['segments'] for point in segment
            )
        self.items['bboxes'].setData(pos=np.asarray(
            bbox_lines, dtype=np.float32
        ).reshape((-1, 3)))
        counts['bboxes'] = len(bboxes)

        for layer in (
            'lidar_only_bboxes',
            'camera_only_bboxes',
            'camera_lidar_fused_bboxes',
            'camera_projection',
            'calibration_bbox',
        ):
            lines = []
            colors = []
            values = [
                item for item in snapshot['association_bboxes'][layer]
                if self.visibility[layer]
                and now - item['received_at'] < 2.0
            ]
            for item in values:
                segment_points = [
                    point for segment in item['segments'] for point in segment
                ]
                lines.extend(segment_points)
                affiliation = int(item.get('metadata', {}).get('affiliation', 0))
                colors.extend([
                    AFFILIATION_COLORS.get(affiliation, AFFILIATION_COLORS[0])
                ] * len(segment_points))
                metadata = item.get('metadata', {})
                if metadata and item['segments']:
                    anchor = item['segments'][0][0]
                    label_item = {
                        'x': anchor[0], 'y': anchor[1], 'z': anchor[2],
                    }
                    names = ('UNKNOWN', 'FRIENDLY', 'HOSTILE', 'NEUTRAL')
                    self._label(label_item, '%s %s %s %.2f %s v=%.1f a=%.2f n=%d' % (
                        metadata.get('track_id', '-'),
                        metadata.get('class_name', 'unknown'),
                        names[affiliation] if 0 <= affiliation < 4 else 'UNKNOWN',
                        float(metadata.get('affiliation_confidence', 0.0)),
                        metadata.get('sensor_source', '-'),
                        float(metadata.get('speed', 0.0)),
                        float(metadata.get('association_score', 0.0)),
                        int(metadata.get('bbox_point_count', 0)),
                    ), AFFILIATION_COLORS.get(
                        affiliation, DEBUG_STYLE[layer]['color']
                    ))
            positions = np.asarray(lines, dtype=np.float32).reshape((-1, 3))
            if (
                layer not in ('camera_projection', 'calibration_bbox')
                and self.color_mode == 'affiliation'
                and len(colors) == len(lines)
            ):
                self.items[layer].setData(
                    pos=positions, color=np.asarray(colors, dtype=np.float32)
                )
            else:
                self.items[layer].setData(
                    pos=positions, color=DEBUG_STYLE[layer]['color']
                )
            counts[layer] = len(values)

        for layer in ('tracks', 'dynamic', 'fusion'):
            counts[layer] = self._update_tracks(
                layer, snapshot['tracks'][layer], snapshot['histories'], now
            )
        counts['tf'] = self._update_tf(snapshot['frames'], now)
        if self.auto_center_pending:
            center = self._initial_view_center(snapshot, now)
            if center is not None:
                self.view_center = center
                self._apply_camera_position()
                self.auto_center_pending = False
        self.items['grid'].setVisible(self.visibility['grid'])
        self.last_counts = counts
        if self.last_render_at:
            self.render_intervals.append(now - self.last_render_at)
        self.last_render_at = now
        self.last_render_ms = (time.perf_counter() - started) * 1000.0

    def set_layer_visible(self, layer, visible):
        if layer in self.visibility:
            self.visibility[layer] = bool(visible)
            self.last_generation = -1

    def set_max_points(self, maximum):
        value = max(100, int(maximum))
        self.max_points = {
            'raw': value,
            'filtered': value,
            'calibration_roi': min(value, 20000),
        }
        self.last_generation = -1

    def set_trajectory_length(self, length):
        self.trajectory_length = max(10, int(length))
        self.last_generation = -1

    def set_color_mode(self, mode):
        self.color_mode = (
            'affiliation' if mode == 'affiliation' else 'sensor_source'
        )
        self.last_generation = -1

    def clear_histories(self):
        self.model.clear_histories()

    def reset_view(self):
        self.auto_center_pending = True
        self.set_view_mode(self.view_mode)

    def set_view_mode(self, mode):
        self.view_mode = 'topdown' if mode == 'topdown' else 'oblique'
        self._apply_camera_position()

    def _apply_camera_position(self):
        elevation = 89.0 if self.view_mode == 'topdown' else 36.0
        self.view.setCameraPosition(
            pos=pg.Vector(*self.view_center),
            distance=115.0, elevation=elevation, azimuth=-90.0,
        )

    @staticmethod
    def _initial_view_center(snapshot, now):
        base = snapshot['frames'].get('base')
        if base and now - base.get('received_at', 0.0) < 2.0:
            return np.asarray(
                (base['x'], base['y'], base['z']), dtype=np.float32
            )
        for layer in ('filtered', 'raw'):
            if now - snapshot['cloud_received'][layer] >= 2.0:
                continue
            points = snapshot['clouds'][layer]
            if len(points):
                return np.median(points, axis=0).astype(np.float32)
        return None

    def statistics(self):
        intervals = [value for value in self.render_intervals if value > 0]
        mean = sum(intervals) / len(intervals) if intervals else 0.0
        snapshot = self.model.snapshot()
        return {
            'fps': 1.0 / mean if mean else 0.0,
            'render_ms': self.last_render_ms,
            'counts': dict(self.last_counts),
            'status': snapshot['status'],
            'cloud_age': {
                key: max(0.0, time.monotonic() - value) if value else None
                for key, value in snapshot['cloud_received'].items()
            },
        }
