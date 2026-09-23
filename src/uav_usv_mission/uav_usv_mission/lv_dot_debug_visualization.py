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
from PyQt5.QtCore import QEvent
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from visualization_msgs.msg import Marker

from uav_usv_mission.perception_topdown import marker_segments_3d
from uav_usv_mission.perception_topdown import tracked_object_dict
from uav_usv_mission.pyqtgraph_compat import gl_text_item


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
        # This is shared with the ROS callback path.  Do not spend CPU
        # decoding and transforming a high-rate cloud which is hidden in Qt.
        self._cloud_layers_enabled = {
            'raw': False,
            'filtered': True,
            'calibration_roi': False,
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
            if not self._cloud_layers_enabled.get(layer, False):
                return
            self._clouds[layer] = values
            self._cloud_received[layer] = time.monotonic()
            self._generation += 1

    def set_cloud_layer_enabled(self, layer, enabled):
        if layer not in self._clouds:
            return
        with self._lock:
            self._cloud_layers_enabled[layer] = bool(enabled)

    def cloud_layer_enabled(self, layer):
        with self._lock:
            return bool(self._cloud_layers_enabled.get(layer, False))

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

    def snapshot(
        self, cloud_layers=None, marker_layers=None, track_layers=None,
        include_frames=True,
    ):
        """Return one immutable display snapshot.

        Hidden cloud layers are intentionally not copied.  Raw Mid-360 clouds
        are the largest objects in this model, and copying them for a disabled
        layer was the main avoidable Qt/OpenGL refresh cost.
        """
        active_cloud_layers = set(
            self._clouds if cloud_layers is None else cloud_layers
        )
        active_marker_layers = set(
            ('clusters', 'bboxes', *self._association_bboxes)
            if marker_layers is None else marker_layers
        )
        active_track_layers = set(
            self._tracks if track_layers is None else track_layers
        )
        with self._lock:
            return {
                'generation': self._generation,
                'clouds': {
                    key: (
                        value.copy() if key in active_cloud_layers
                        else np.empty((0, 3), dtype=np.float32)
                    )
                    for key, value in self._clouds.items()
                },
                'cloud_received': dict(self._cloud_received),
                'clusters': (
                    deepcopy(list(self._clusters.values()))
                    if 'clusters' in active_marker_layers else []
                ),
                'bboxes': (
                    deepcopy(list(self._bboxes.values()))
                    if 'bboxes' in active_marker_layers else []
                ),
                'association_bboxes': {
                    key: (
                        deepcopy(list(value.values()))
                        if key in active_marker_layers else []
                    )
                    for key, value in self._association_bboxes.items()
                },
                'tracks': {
                    key: (
                        deepcopy(value) if key in active_track_layers else {}
                    )
                    for key, value in self._tracks.items()
                },
                'frames': deepcopy(self._frames) if include_frames else {},
                'histories': {
                    key: list(value) for key, value in self._histories.items()
                    if key[0] in active_track_layers
                },
                'status': deepcopy(self._status),
            }


class LvDotDebugWidget(QWidget):
    """Passive map-frame top-view renderer for all LV-DOT stages."""

    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        # Match the web situation view by default: one recent LiDAR frame,
        # final fusion results, and the sensor pose.  The other stages remain
        # available as opt-in diagnostics instead of obscuring the target.
        self.visibility = {
            'raw': False,
            'filtered': True,
            'clusters': False,
            'bboxes': False,
            'lidar_only_bboxes': False,
            'camera_only_bboxes': False,
            'camera_lidar_fused_bboxes': True,
            'calibration_roi': False,
            'camera_projection': False,
            'calibration_bbox': False,
            'tracks': False,
            'dynamic': False,
            'fusion': True,
            'labels': True,
            'grid': True,
            'tf': True,
        }
        self.max_points = {
            'raw': 60000, 'filtered': 60000, 'calibration_roi': 10000,
        }
        self.trajectory_length = 60
        self.last_generation = -1
        self.last_render_at = 0.0
        self.render_intervals = deque(maxlen=100)
        self.last_render_ms = 0.0
        self.last_counts = {}
        self.labels = []
        self._drawing_labels = False
        self._labels_dirty = True
        self._last_labels_update = 0.0
        self.color_mode = 'sensor_source'
        self.view_mode = 'oblique'
        self.view_center = np.zeros(3, dtype=np.float32)
        self.auto_center_pending = True
        self.follow_sensor = True

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor('#05090d')
        # A wheel or drag is an explicit user camera choice.  Without this
        # event filter the moving USV TF re-applies the default camera
        # distance on every incoming point-cloud frame.
        self.view.installEventFilter(self)
        layout.addWidget(self.view)

        self.items = {}
        grid = gl.GLGridItem()
        grid.setSize(180.0, 180.0)
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
        # Rendering is decoupled from the ~17-20 Hz sensor stream.  A timer
        # tick without a new cloud simply repaints the already-uploaded scene.
        self.timer.setInterval(33)
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
        if not self.visibility['labels'] or not self._drawing_labels:
            return
        label = gl_text_item(
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
                (
                    frame.get('frame_id', 'MID-360 TF')
                    if key == 'radar'
                    else frame.get('frame_id', 'USV base TF')
                ),
                (0.92, 0.96, 1.0, 1.0),
            )
        for axis, points in axis_lines.items():
            self.items['tf_' + axis].setData(pos=np.asarray(
                points, dtype=np.float32
            ).reshape((-1, 3)))
        return len(active)

    def refresh(self):
        started = time.perf_counter()
        visible_cloud_layers = [
            layer for layer in ('raw', 'filtered', 'calibration_roi')
            if self.visibility[layer]
        ]
        visible_marker_layers = {
            layer for layer in (
                'clusters', 'bboxes', 'lidar_only_bboxes',
                'camera_only_bboxes', 'camera_lidar_fused_bboxes',
                'camera_projection', 'calibration_bbox',
            ) if self.visibility[layer]
        }
        visible_track_layers = {
            layer for layer in ('tracks', 'dynamic', 'fusion')
            if self.visibility[layer]
        }
        snapshot = self.model.snapshot(
            cloud_layers=visible_cloud_layers,
            marker_layers=visible_marker_layers,
            track_layers=visible_track_layers,
            include_frames=(self.visibility['tf'] or self.follow_sensor),
        )
        now = time.monotonic()
        if snapshot['generation'] == self.last_generation:
            self.view.update()
            self._record_render(now, started)
            return
        self.last_generation = snapshot['generation']
        self._drawing_labels = bool(
            self.visibility['labels'] and (
                self._labels_dirty
                or now - self._last_labels_update >= 0.25
            )
        )
        if self._drawing_labels:
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
                # Candidate-stage labels make a dense scan unreadable.  Keep
                # labels for the final 3D fused result only, like the web view.
                if (
                    layer == 'camera_lidar_fused_bboxes'
                    and metadata and item['segments']
                ):
                    anchor = item['segments'][0][0]
                    label_item = {
                        'x': anchor[0], 'y': anchor[1], 'z': anchor[2],
                    }
                    self._label(
                        label_item,
                        'FUSION %s' % metadata.get('track_id', item['id']),
                        DEBUG_STYLE[layer]['color'],
                    )
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
        if self.auto_center_pending or self.follow_sensor:
            center = self._initial_view_center(snapshot, now)
            if center is not None:
                if (
                    self.auto_center_pending
                    or np.linalg.norm(center - self.view_center) > 0.02
                ):
                    self.view_center = center
                    self._apply_camera_position()
                self.auto_center_pending = False
        self._position_grid()
        self.items['grid'].setVisible(self.visibility['grid'])
        self.last_counts = counts
        if self._drawing_labels:
            self._last_labels_update = now
            self._labels_dirty = False
        self._drawing_labels = False
        self._record_render(now, started)

    def _record_render(self, now, started):
        if self.last_render_at:
            self.render_intervals.append(now - self.last_render_at)
        self.last_render_at = now
        self.last_render_ms = (time.perf_counter() - started) * 1000.0

    def _position_grid(self):
        """Keep the map grid under the selected USV instead of map origin."""
        grid = self.items['grid']
        grid.resetTransform()
        grid.translate(
            float(self.view_center[0]), float(self.view_center[1]), 0.0
        )

    def eventFilter(self, watched, event):
        if watched is self.view and event.type() in (
            QEvent.Wheel,
            QEvent.MouseButtonPress,
        ):
            self.follow_sensor = False
            self.auto_center_pending = False
        return super().eventFilter(watched, event)

    def set_layer_visible(self, layer, visible):
        if layer in self.visibility:
            self.visibility[layer] = bool(visible)
            if layer in ('raw', 'filtered', 'calibration_roi'):
                self.model.set_cloud_layer_enabled(layer, visible)
            if layer == 'labels' and not visible:
                self._clear_labels()
            self._labels_dirty = True
            self.last_generation = -1

    def set_model(self, model):
        """Switch the displayed USV cache without recreating the GL canvas."""
        self.model = model
        for layer in ('raw', 'filtered', 'calibration_roi'):
            self.model.set_cloud_layer_enabled(
                layer, self.visibility.get(layer, False)
            )
        self.last_generation = -1
        self.auto_center_pending = True
        self._labels_dirty = True
        self._clear_labels()

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
        self.follow_sensor = True
        self.set_view_mode(self.view_mode)

    def set_view_mode(self, mode):
        self.view_mode = 'topdown' if mode == 'topdown' else 'oblique'
        self._apply_camera_position()

    def _apply_camera_position(self):
        elevation = 89.0 if self.view_mode == 'topdown' else 36.0
        self.view.setCameraPosition(
            pos=pg.Vector(*self.view_center),
            distance=95.0, elevation=elevation, azimuth=-90.0,
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
        # Status refresh runs from the Qt side panel; it needs timestamps and
        # counters only, not another copy of the point-cloud buffers.
        snapshot = self.model.snapshot(
            cloud_layers=(), marker_layers=(), track_layers=(),
            include_frames=False,
        )
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
