"""Thread-safe data model and PyQtGraph top-down perception display."""

from collections import deque
from copy import deepcopy
import importlib.util
import math
import os
import threading
import time

import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
import pyqtgraph as pg
from visualization_msgs.msg import Marker

gl = None
OPENGL_AVAILABLE = (
    importlib.util.find_spec('pyqtgraph.opengl') is not None
    and os.environ.get('QT_QPA_PLATFORM', '').lower()
    not in ('offscreen', 'minimal', 'minimalegl')
)


LAYER_STYLE = {
    'pointcloud': {'color': '#ffffff', 'size': 2.5},
    'clusters': {'color': '#ffe34d', 'width': 2.5},
    'tracks': {'color': '#ffd447', 'symbol': 'o', 'size': 10},
    'dynamic': {'color': '#ff4f78', 'symbol': 'o', 'size': 12},
    'fusion': {'color': '#42e66c', 'symbol': 's', 'size': 13},
    'ground_truth': {'color': '#4f9dff', 'symbol': 'x', 'size': 12},
    'uav': {'color': '#58a6ff', 'symbol': 't', 'size': 14},
    'usv': {'color': '#ffbc42', 'symbol': 's', 'size': 14},
}

CLASS_NAMES = {
    0: 'UNKNOWN',
    1: 'VESSEL',
    2: 'BUOY',
    3: 'DEBRIS',
    4: 'LANDMARK',
}

TRACK_LABEL_OFFSET = {
    'tracks': (0.8, 0.8),
    'dynamic': (0.8, 2.4),
    'fusion': (0.8, 4.0),
    'ground_truth': (0.8, 5.6),
}


def source_name(mask):
    names = []
    for bit, name in ((1, 'LiDAR'), (2, 'Camera'), (4, 'AIS'), (8, 'Fusion')):
        if int(mask) & bit:
            names.append(name)
    return '+'.join(names) if names else 'Ground Truth'


def quaternion_yaw(orientation):
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def quaternion_rotation_matrix(orientation):
    """Return the full 3D rotation represented by a Marker pose."""
    values = np.asarray([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ], dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = values / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def marker_segments_3d(marker):
    """Convert a supported Marker into world-frame 3D line segments."""
    rotation = quaternion_rotation_matrix(marker.pose.orientation)
    origin = np.asarray([
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z,
    ], dtype=np.float64)

    def world(point):
        transformed = rotation @ np.asarray(point, dtype=np.float64) + origin
        return tuple(float(value) for value in transformed)

    if marker.type in (Marker.LINE_LIST, Marker.LINE_STRIP):
        points = [
            world((point.x, point.y, point.z)) for point in marker.points
        ]
        if marker.type == Marker.LINE_LIST:
            return [
                (points[index], points[index + 1])
                for index in range(0, len(points) - 1, 2)
            ]
        return list(zip(points, points[1:]))
    if marker.type == Marker.CUBE:
        half = np.asarray([
            marker.scale.x,
            marker.scale.y,
            marker.scale.z,
        ], dtype=np.float64) * 0.5
        corners = [
            world((sx * half[0], sy * half[1], sz * half[2]))
            for sx, sy, sz in (
                (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
            )
        ]
        edge_indices = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        return [(corners[start], corners[end]) for start, end in edge_indices]
    return []


def marker_segments_xy(marker):
    """Convert supported map-frame Marker geometry to top-down segments."""
    yaw = quaternion_yaw(marker.pose.orientation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    origin_x = float(marker.pose.position.x)
    origin_y = float(marker.pose.position.y)

    def world(point):
        return (
            origin_x + cosine * float(point[0]) - sine * float(point[1]),
            origin_y + sine * float(point[0]) + cosine * float(point[1]),
        )

    if marker.type in (Marker.LINE_LIST, Marker.LINE_STRIP):
        points = [world((point.x, point.y)) for point in marker.points]
        if marker.type == Marker.LINE_LIST:
            return [
                (points[index], points[index + 1])
                for index in range(0, len(points) - 1, 2)
            ]
        return list(zip(points, points[1:]))
    if marker.type == Marker.CUBE:
        half_x = 0.5 * float(marker.scale.x)
        half_y = 0.5 * float(marker.scale.y)
        corners = [
            world((-half_x, -half_y)),
            world((half_x, -half_y)),
            world((half_x, half_y)),
            world((-half_x, half_y)),
        ]
        return list(zip(corners, corners[1:] + corners[:1]))
    return []


def tracked_object_dict(tracked, received_at, dynamic=False, fused=False):
    pose = tracked.pose.pose
    twist = tracked.twist.twist
    stamp = (
        float(tracked.last_update.sec)
        + float(tracked.last_update.nanosec) * 1e-9
    )
    return {
        'track_id': tracked.track_id,
        'x': float(pose.position.x),
        'y': float(pose.position.y),
        'z': float(pose.position.z),
        'yaw': quaternion_yaw(pose.orientation),
        'vx': float(twist.linear.x),
        'vy': float(twist.linear.y),
        'vz': float(twist.linear.z),
        'dimensions': (
            float(tracked.dimensions.x),
            float(tracked.dimensions.y),
            float(tracked.dimensions.z),
        ),
        'confidence': float(tracked.confidence),
        'source_mask': int(tracked.source_mask),
        'source': source_name(tracked.source_mask),
        'classification': int(tracked.classification),
        'classification_name': CLASS_NAMES.get(
            int(tracked.classification), 'UNKNOWN'
        ),
        'class_name': getattr(tracked, 'class_name', '') or CLASS_NAMES.get(
            int(tracked.classification), 'UNKNOWN'
        ).lower(),
        'sensor_source': getattr(tracked, 'sensor_source', '') or source_name(
            tracked.source_mask
        ),
        'affiliation': int(getattr(tracked, 'affiliation', 0)),
        'affiliation_confidence': float(
            getattr(tracked, 'affiliation_confidence', 0.0)
        ),
        'association_score': float(
            getattr(tracked, 'association_score', 0.0)
        ),
        'bbox_point_count': int(
            getattr(tracked, 'bbox_point_count', 0)
        ),
        'stamp': stamp,
        'received_at': received_at,
        'dynamic': bool(dynamic),
        'fused': bool(fused),
    }


def oriented_rectangle_segments(tracked):
    length = max(0.0, float(tracked['dimensions'][0]))
    width = max(0.0, float(tracked['dimensions'][1]))
    if length <= 0.05 or width <= 0.05:
        return []
    half_x = 0.5 * length
    half_y = 0.5 * width
    cosine = math.cos(tracked['yaw'])
    sine = math.sin(tracked['yaw'])

    def world(local_x, local_y):
        return (
            tracked['x'] + cosine * local_x - sine * local_y,
            tracked['y'] + sine * local_x + cosine * local_y,
        )

    corners = [
        world(-half_x, -half_y),
        world(half_x, -half_y),
        world(half_x, half_y),
        world(-half_x, half_y),
    ]
    return list(zip(corners, corners[1:] + corners[:1]))


class TopDownVisualizationModel:
    """Latest-frame cache shared by ROS callbacks and the Qt refresh timer."""

    TRACK_LAYERS = ('tracks', 'dynamic', 'fusion', 'ground_truth')

    def __init__(self, history_capacity=500):
        self._lock = threading.Lock()
        self._generation = 0
        self._points = np.empty((0, 2), dtype=np.float32)
        self._point_heights = np.empty((0,), dtype=np.float32)
        self._points_received_at = 0.0
        self._point_frame_count = 0
        self._point_status = {}
        self._clusters = {}
        self._tracks = {layer: {} for layer in self.TRACK_LAYERS}
        self._vehicles = {}
        self._roles = {}
        self._histories = {}
        self._history_capacity = max(10, int(history_capacity))

    def _changed(self):
        self._generation += 1

    def update_points(self, message):
        values = np.array(
            [(pose.position.x, pose.position.y) for pose in message.poses],
            dtype=np.float32,
        ).reshape((-1, 2))
        heights = np.array(
            [pose.position.z for pose in message.poses], dtype=np.float32
        )
        self.update_point_array(values, heights)

    def update_point_array(self, points, heights):
        """Replace the display cloud using already-decoded NumPy arrays."""
        values = np.asarray(points, dtype=np.float32).reshape((-1, 2))
        z_values = np.asarray(heights, dtype=np.float32).reshape((-1,))
        if len(values) != len(z_values):
            raise ValueError('point and height arrays must have equal length')
        with self._lock:
            self._points = values
            self._point_heights = z_values
            self._points_received_at = time.monotonic()
            self._point_frame_count += 1
            self._changed()

    def update_point_status(self, status):
        with self._lock:
            self._point_status = dict(status)
            self._point_status['_received_at'] = time.monotonic()
            self._changed()

    def update_clusters(self, message, frame_id='map'):
        now = time.monotonic()
        with self._lock:
            for marker in message.markers:
                if marker.action == Marker.DELETEALL:
                    self._clusters.clear()
                    continue
                key = (marker.ns, int(marker.id))
                if marker.action == Marker.DELETE:
                    self._clusters.pop(key, None)
                    continue
                marker_frame = marker.header.frame_id or frame_id
                if marker.action != Marker.ADD or marker_frame != frame_id:
                    continue
                segments = marker_segments_xy(marker)
                segments_3d = marker_segments_3d(marker)
                if not segments or not segments_3d:
                    continue
                coordinates = np.asarray([
                    point for segment in segments_3d for point in segment
                ], dtype=np.float64)
                center = np.mean(coordinates, axis=0)
                dimensions = np.ptp(coordinates, axis=0)
                self._clusters[key] = {
                    'track_id': 'cluster_%d' % marker.id,
                    'segments': segments,
                    'segments_3d': segments_3d,
                    'x': float(center[0]),
                    'y': float(center[1]),
                    'z': float(center[2]),
                    'source': 'LiDAR cluster',
                    'classification_name': 'CLUSTER',
                    'confidence': None,
                    'stamp': (
                        float(marker.header.stamp.sec)
                        + float(marker.header.stamp.nanosec) * 1e-9
                    ),
                    'received_at': now,
                    'dimensions': tuple(float(value) for value in dimensions),
                    'vx': 0.0,
                    'vy': 0.0,
                    'dynamic': False,
                    'fused': False,
                }
            self._changed()

    def update_tracks(self, layer, message, frame_id='map'):
        if layer not in self._tracks:
            return
        if message.header.frame_id and message.header.frame_id != frame_id:
            return
        now = time.monotonic()
        tracks = {
            tracked.track_id: tracked_object_dict(
                tracked,
                now,
                dynamic=(layer == 'dynamic'),
                fused=(layer == 'fusion'),
            )
            for tracked in message.objects
            if tracked.track_id
        }
        with self._lock:
            self._tracks[layer] = tracks
            for track_id, tracked in tracks.items():
                key = (layer, track_id)
                history = self._histories.setdefault(
                    key, deque(maxlen=self._history_capacity)
                )
                point = (tracked['x'], tracked['y'])
                if not history or math.hypot(
                    point[0] - history[-1][0],
                    point[1] - history[-1][1],
                ) >= 0.05:
                    history.append(point)
            self._changed()

    def update_vehicle(self, message, frame_id='map'):
        if message.header.frame_id and message.header.frame_id != frame_id:
            return
        now = time.monotonic()
        vehicle = {
            'track_id': message.vehicle_id,
            'vehicle_id': message.vehicle_id,
            'vehicle_type': int(message.vehicle_type),
            'x': float(message.pose.position.x),
            'y': float(message.pose.position.y),
            'z': float(message.pose.position.z),
            'yaw': quaternion_yaw(message.pose.orientation),
            'vx': float(message.twist.linear.x),
            'vy': float(message.twist.linear.y),
            'vz': float(message.twist.linear.z),
            'source': 'VehicleState',
            'classification_name': (
                'UAV' if int(message.vehicle_type) == 1 else 'USV'
            ),
            'confidence': 1.0,
            'stamp': (
                float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) * 1e-9
            ),
            'received_at': now,
            'dimensions': (0.0, 0.0, 0.0),
            'dynamic': True,
            'fused': False,
            'online': bool(message.online),
        }
        with self._lock:
            vehicle['role'] = self._roles.get(message.vehicle_id, '')
            self._vehicles[message.vehicle_id] = vehicle
            key = ('vehicle', message.vehicle_id)
            history = self._histories.setdefault(
                key, deque(maxlen=self._history_capacity)
            )
            point = (vehicle['x'], vehicle['y'])
            if not history or math.hypot(
                point[0] - history[-1][0], point[1] - history[-1][1]
            ) >= 0.05:
                history.append(point)
            self._changed()

    def update_vehicle_pose(
        self, vehicle_id, message, vehicle_type=2, frame_id='map'
    ):
        """Update a display-only vehicle pose when VehicleState is absent."""
        if message.header.frame_id and message.header.frame_id != frame_id:
            return
        now = time.monotonic()
        pose = message.pose
        vehicle = {
            'track_id': vehicle_id,
            'vehicle_id': vehicle_id,
            'vehicle_type': int(vehicle_type),
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
            'yaw': quaternion_yaw(pose.orientation),
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'source': 'PoseStamped',
            'classification_name': 'USV',
            'confidence': 1.0,
            'stamp': (
                float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) * 1e-9
            ),
            'received_at': now,
            'dimensions': (0.0, 0.0, 0.0),
            'dynamic': True,
            'fused': False,
            'online': True,
        }
        with self._lock:
            vehicle['role'] = self._roles.get(vehicle_id, '')
            self._vehicles[vehicle_id] = vehicle
            self._changed()

    def update_roles(self, assignments):
        roles = {
            item['vehicle_id']: item.get('role_name', '')
            for item in assignments
        }
        with self._lock:
            self._roles = roles
            for vehicle_id, vehicle in self._vehicles.items():
                vehicle['role'] = roles.get(vehicle_id, '')
            self._changed()

    def clear_histories(self):
        with self._lock:
            self._histories.clear()
            self._changed()

    def snapshot(self):
        with self._lock:
            return {
                'generation': self._generation,
                'points': self._points.copy(),
                'point_heights': self._point_heights.copy(),
                'points_received_at': self._points_received_at,
                'point_frame_count': self._point_frame_count,
                'point_status': dict(self._point_status),
                'clusters': deepcopy(list(self._clusters.values())),
                'tracks': deepcopy(self._tracks),
                'vehicles': deepcopy(self._vehicles),
                'histories': {
                    key: list(history)
                    for key, history in self._histories.items()
                },
            }


class PerceptionPlotWidget(pg.PlotWidget):
    def __init__(self, reset_callback, *args, **kwargs):
        self._reset_callback = reset_callback
        super().__init__(*args, **kwargs)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._reset_callback()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)


class PerceptionTopDownWidget(QWidget):
    """Interactive orthographic map view backed by latest lightweight data."""

    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        self.default_range = 35.0
        self.grid_spacing = 10.0
        self.point_size = 2.5
        self.max_display_points = 10000
        self.trajectory_length = 120
        self.follow_mode = 'none'
        self.view_mode = 'topdown'
        self.point_color_mode = 'fixed'
        self.single_target_cluster = True
        self.follow_target_id = ''
        self.layer_visibility = {
            'pointcloud': True,
            'clusters': True,
            'tracks': False,
            'dynamic': False,
            'fusion': False,
            'ground_truth': False,
            'uav': False,
            'usv': False,
            'trails': False,
            'velocity': False,
            'labels': False,
            'grid': True,
            'tf': True,
        }
        self.last_generation = -1
        self.last_snapshot = None
        self.rendered_generation = -1
        self.rendered_point_frame_count = 0
        self.rendered_frames = 0
        self.overwritten_point_frames = 0
        self.render_intervals = deque(maxlen=120)
        self.last_render_at = 0.0
        self.last_render_ms = 0.0
        self.last_drawn_points = 0
        self.last_point_frame_count = -1
        self.gl_point_frame_count = -1
        self.selected = None
        self.selection_callback = None
        self.setMinimumSize(760, 480)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.plot = PerceptionPlotWidget(
            self.reset_view, background='#0a1118'
        )
        self.plot.setAspectLocked(True)
        self.plot.setLabel('bottom', 'X', units='m')
        self.plot.setLabel('left', 'Y', units='m')
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.getPlotItem().setMenuEnabled(False)
        self.plot.getPlotItem().hideButtons()
        layout.addWidget(self.plot)

        self.opengl_available = OPENGL_AVAILABLE
        self.gl_view = None
        self.gl_items = {}
        self.gl_labels = []
        if self.opengl_available:
            self._build_gl_view(layout)

        self.point_item = pg.ScatterPlotItem(
            size=self.point_size,
            pen=None,
            brush=pg.mkBrush(LAYER_STYLE['pointcloud']['color']),
            pxMode=True,
        )
        self.cluster_item = pg.PlotDataItem(
            pen=pg.mkPen(
                LAYER_STYLE['clusters']['color'],
                width=LAYER_STYLE['clusters']['width'],
            ),
            connect='finite',
        )
        self.trail_items = {}
        self.track_items = {}
        self.outline_items = {}
        self.velocity_items = {}
        self.vehicle_items = {}
        self.label_items = []
        self.plot.addItem(self.point_item)
        self.plot.addItem(self.cluster_item)
        for layer in ('tracks', 'dynamic', 'fusion', 'ground_truth'):
            style = LAYER_STYLE[layer]
            scatter = pg.ScatterPlotItem(
                size=style['size'],
                symbol=style['symbol'],
                pen=pg.mkPen(style['color'], width=2),
                brush=pg.mkBrush(style['color']),
                pxMode=True,
            )
            scatter.sigClicked.connect(self._on_scatter_clicked)
            trail = pg.PlotDataItem(
                pen=pg.mkPen(style['color'], width=1), connect='finite'
            )
            velocity = pg.PlotDataItem(
                pen=pg.mkPen(style['color'], width=2), connect='finite'
            )
            outline = pg.PlotDataItem(
                pen=pg.mkPen(style['color'], width=2), connect='finite'
            )
            self.track_items[layer] = scatter
            self.trail_items[layer] = trail
            self.velocity_items[layer] = velocity
            self.outline_items[layer] = outline
            self.plot.addItem(trail)
            self.plot.addItem(velocity)
            self.plot.addItem(outline)
            self.plot.addItem(scatter)
        for layer in ('uav', 'usv'):
            style = LAYER_STYLE[layer]
            scatter = pg.ScatterPlotItem(
                size=style['size'],
                symbol=style['symbol'],
                pen=pg.mkPen(style['color'], width=2),
                brush=pg.mkBrush(style['color']),
                pxMode=True,
            )
            scatter.sigClicked.connect(self._on_scatter_clicked)
            self.vehicle_items[layer] = scatter
            self.trail_items[layer] = pg.PlotDataItem(
                pen=pg.mkPen(style['color'], width=1), connect='finite'
            )
            self.velocity_items[layer] = pg.PlotDataItem(
                pen=pg.mkPen(style['color'], width=2), connect='finite'
            )
            self.plot.addItem(self.trail_items[layer])
            self.plot.addItem(self.velocity_items[layer])
            self.plot.addItem(scatter)
        self.cluster_select_item = pg.ScatterPlotItem(
            size=9,
            symbol='o',
            pen=pg.mkPen(LAYER_STYLE['clusters']['color'], width=2),
            brush=None,
            pxMode=True,
        )
        self.cluster_select_item.sigClicked.connect(self._on_scatter_clicked)
        self.plot.addItem(self.cluster_select_item)
        self.height_colormap = pg.ColorMap(
            [0.0, 0.45, 1.0],
            [(48, 88, 145), (40, 224, 208), (255, 212, 71)],
        )

        self.reset_view()
        self.timer = QTimer(self)
        self.timer.setInterval(40)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def _build_gl_view(self, layout):
        global gl
        try:
            import pyqtgraph.opengl as gl_module
        except (ImportError, RuntimeError):
            self.opengl_available = False
            return
        gl = gl_module
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setBackgroundColor('#05090d')
        self.gl_view.setCameraPosition(
            distance=80.0, elevation=32.0, azimuth=-90.0
        )
        self.gl_view.hide()
        layout.addWidget(self.gl_view)

        grid = gl.GLGridItem()
        grid.setSize(100.0, 100.0)
        grid.setSpacing(self.grid_spacing, self.grid_spacing)
        grid.setColor((65, 83, 96, 130))
        self.gl_view.addItem(grid)
        self.gl_items['grid'] = grid
        self.gl_items['pointcloud'] = gl.GLScatterPlotItem(
            pos=np.empty((0, 3), dtype=np.float32),
            color=(1.0, 1.0, 1.0, 0.92),
            size=self.point_size,
            pxMode=True,
        )
        self.gl_view.addItem(self.gl_items['pointcloud'])
        for layer in (
            'clusters', 'tracks', 'dynamic', 'fusion', 'ground_truth',
            'uav', 'usv',
        ):
            color = pg.mkColor(LAYER_STYLE[layer]['color'])
            rgba = (
                color.redF(), color.greenF(), color.blueF(), 0.95
            )
            self.gl_items[layer] = gl.GLScatterPlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=rgba,
                size=float(LAYER_STYLE[layer].get('size', 8.0)),
                pxMode=True,
            )
            self.gl_view.addItem(self.gl_items[layer])
            for suffix, width in (
                ('outlines', 2.0), ('trails', 1.0), ('velocity', 2.0)
            ):
                item = gl.GLLinePlotItem(
                    pos=np.empty((0, 3), dtype=np.float32),
                    color=rgba,
                    width=width,
                    mode='lines',
                    antialias=True,
                )
                self.gl_items['%s_%s' % (layer, suffix)] = item
                self.gl_view.addItem(item)

        for axis, color in (
            ('x', (1.0, 0.2, 0.2, 1.0)),
            ('y', (0.2, 1.0, 0.3, 1.0)),
            ('z', (0.25, 0.55, 1.0, 1.0)),
        ):
            item = gl.GLLinePlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=color,
                width=3.0,
                mode='lines',
                antialias=True,
            )
            self.gl_items['tf_%s' % axis] = item
            self.gl_view.addItem(item)

    @staticmethod
    def _line_data(segments):
        x_values = []
        y_values = []
        for start, end in segments:
            x_values.extend((start[0], end[0], np.nan))
            y_values.extend((start[1], end[1], np.nan))
        return x_values, y_values

    def _clear_labels(self):
        for item in self.label_items:
            self.plot.removeItem(item)
        self.label_items.clear()

    def _add_label(self, x, y, text, color):
        label = pg.TextItem(
            text=text,
            color=color,
            anchor=(0.0, 1.0),
            border=None,
            fill=pg.mkBrush(7, 14, 20, 170),
        )
        label.setPos(x, y)
        self.plot.addItem(label)
        self.label_items.append(label)

    def _active(self, tracked, now, timeout=3.0):
        return now - tracked.get('received_at', 0.0) <= timeout

    def _draw_tracks(self, snapshot, now):
        for layer in ('tracks', 'dynamic', 'fusion', 'ground_truth'):
            visible = self.layer_visibility[layer]
            tracks = [
                item for item in snapshot['tracks'][layer].values()
                if self._active(item, now)
            ] if visible else []
            spots = [
                {
                    'pos': (item['x'], item['y']),
                    'data': dict(item, layer=layer),
                }
                for item in tracks
            ]
            self.track_items[layer].setData(spots=spots)

            outline_segments = []
            if visible:
                for item in tracks:
                    outline_segments.extend(oriented_rectangle_segments(item))
            x_values, y_values = self._line_data(outline_segments)
            self.outline_items[layer].setData(x_values, y_values)

            trail_segments = []
            if visible and self.layer_visibility['trails']:
                for item in tracks:
                    points = snapshot['histories'].get(
                        (layer, item['track_id']), []
                    )[-self.trajectory_length:]
                    trail_segments.extend(zip(points, points[1:]))
            x_values, y_values = self._line_data(trail_segments)
            self.trail_items[layer].setData(x_values, y_values)

            velocity_segments = []
            if visible and self.layer_visibility['velocity']:
                for item in tracks:
                    start = (item['x'], item['y'])
                    end = (
                        item['x'] + item['vx'] * 2.0,
                        item['y'] + item['vy'] * 2.0,
                    )
                    velocity_segments.extend(self._arrow_segments(start, end))
            x_values, y_values = self._line_data(velocity_segments)
            self.velocity_items[layer].setData(x_values, y_values)

            if visible and self.layer_visibility['labels']:
                label_offset = TRACK_LABEL_OFFSET[layer]
                for item in tracks:
                    self._add_label(
                        item['x'] + label_offset[0],
                        item['y'] + label_offset[1],
                        '%s | %s | %.0f%%' % (
                            item['track_id'],
                            item['source'],
                            100.0 * item['confidence'],
                        ),
                        LAYER_STYLE[layer]['color'],
                    )

    def _draw_vehicles(self, snapshot, now):
        for layer, vehicle_type in (('uav', 1), ('usv', 2)):
            vehicles = [
                item for item in snapshot['vehicles'].values()
                if item['vehicle_type'] == vehicle_type
                and item.get('online', True)
                and self._active(item, now)
            ] if self.layer_visibility[layer] else []
            spots = [
                {
                    'pos': (item['x'], item['y']),
                    'data': dict(item, layer=layer),
                }
                for item in vehicles
            ]
            self.vehicle_items[layer].setData(spots=spots)
            trail_segments = []
            if self.layer_visibility['trails']:
                for item in vehicles:
                    points = snapshot['histories'].get(
                        ('vehicle', item['vehicle_id']), []
                    )[-self.trajectory_length:]
                    trail_segments.extend(zip(points, points[1:]))
            x_values, y_values = self._line_data(trail_segments)
            self.trail_items[layer].setData(x_values, y_values)

            heading_segments = []
            if self.layer_visibility['velocity']:
                for item in vehicles:
                    start = (item['x'], item['y'])
                    speed = math.hypot(item['vx'], item['vy'])
                    if speed > 0.05:
                        end = (
                            item['x'] + item['vx'] * 2.0,
                            item['y'] + item['vy'] * 2.0,
                        )
                    else:
                        end = (
                            item['x'] + math.cos(item['yaw']) * 4.0,
                            item['y'] + math.sin(item['yaw']) * 4.0,
                        )
                    heading_segments.extend(self._arrow_segments(start, end))
            x_values, y_values = self._line_data(heading_segments)
            self.velocity_items[layer].setData(x_values, y_values)
            if self.layer_visibility['labels']:
                for item in vehicles:
                    role = (' | ' + item['role']) if item.get('role') else ''
                    self._add_label(
                        item['x'] + 0.8,
                        item['y'] + 0.8,
                        item['vehicle_id'] + role,
                        LAYER_STYLE[layer]['color'],
                    )

    @staticmethod
    def _arrow_segments(start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 0.05:
            return []
        angle = math.atan2(dy, dx)
        head = min(1.5, max(0.35, length * 0.25))
        return [
            (start, end),
            (
                end,
                (
                    end[0] - head * math.cos(angle - 0.55),
                    end[1] - head * math.sin(angle - 0.55),
                ),
            ),
            (
                end,
                (
                    end[0] - head * math.cos(angle + 0.55),
                    end[1] - head * math.sin(angle + 0.55),
                ),
            ),
        ]

    def _apply_follow(self, snapshot, now):
        selected = None
        if self.follow_mode == 'target':
            for layer in ('fusion', 'dynamic', 'ground_truth', 'tracks'):
                candidates = [
                    item for item in snapshot['tracks'][layer].values()
                    if self._active(item, now)
                    and (
                        not self.follow_target_id
                        or item['track_id'] == self.follow_target_id
                    )
                ]
                if candidates:
                    selected = candidates[0]
                    break
        elif self.follow_mode == 'usv':
            selected = next((
                item for item in snapshot['vehicles'].values()
                if item['vehicle_type'] == 2 and self._active(item, now)
            ), None)
        if selected is not None:
            half = self.default_range
            self.plot.setXRange(selected['x'] - half, selected['x'] + half)
            self.plot.setYRange(selected['y'] - half, selected['y'] + half)
            if self.gl_view is not None:
                self.gl_view.setCameraPosition(
                    pos=pg.Vector(selected['x'], selected['y'], 0.0)
                )

    def _point_colors(self, heights):
        if not len(heights) or self.point_color_mode == 'fixed':
            return None, None
        finite = heights[np.isfinite(heights)]
        if not len(finite):
            return None, None
        lower, upper = np.percentile(finite, (5.0, 95.0))
        span = max(0.10, float(upper - lower))
        normalized = np.clip((heights - lower) / span, 0.0, 1.0)
        qcolors = self.height_colormap.map(normalized, mode='qcolor')
        bytes_rgba = self.height_colormap.map(normalized, mode='byte')
        return qcolors, bytes_rgba.astype(np.float32) / 255.0

    @staticmethod
    def _gl_segment_positions(segments, z_value=0.0):
        if not segments:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray([
            (point[0], point[1], z_value)
            for segment in segments for point in segment
        ], dtype=np.float32)

    def _clear_gl_labels(self):
        if self.gl_view is None:
            return
        for item in self.gl_labels:
            self.gl_view.removeItem(item)
        self.gl_labels.clear()

    def _add_gl_label(self, item, layer):
        if self.gl_view is None or not self.layer_visibility['labels']:
            return
        offset = TRACK_LABEL_OFFSET.get(layer, (0.6, 0.6))
        label = gl.GLTextItem(
            pos=(
                item['x'] + offset[0],
                item['y'] + offset[1],
                item['z'] + 0.8,
            ),
            color=pg.mkColor(LAYER_STYLE[layer]['color']),
            text=str(item.get('track_id') or item.get('vehicle_id') or ''),
            font=QFont('Sans Serif', 10),
        )
        self.gl_view.addItem(label)
        self.gl_labels.append(label)

    def _update_gl_layer(self, layer, items, snapshot):
        positions = np.asarray([
            (item['x'], item['y'], item['z']) for item in items
        ], dtype=np.float32).reshape((-1, 3))
        self.gl_items[layer].setData(pos=positions)

        outline_positions = []
        trail_positions = []
        velocity_positions = []
        for item in items:
            outline_positions.extend(self._gl_segment_positions(
                oriented_rectangle_segments(item), item['z']
            ))
            if self.layer_visibility['trails']:
                history = snapshot['histories'].get(
                    (layer, item['track_id']), []
                )[-self.trajectory_length:]
                trail_positions.extend(self._gl_segment_positions(
                    list(zip(history, history[1:])), item['z']
                ))
            if self.layer_visibility['velocity']:
                velocity_positions.extend(self._gl_segment_positions(
                    self._arrow_segments(
                        (item['x'], item['y']),
                        (
                            item['x'] + item['vx'] * 2.0,
                            item['y'] + item['vy'] * 2.0,
                        ),
                    ),
                    item['z'],
                ))
            self._add_gl_label(item, layer)
        for suffix, positions_data in (
            ('outlines', outline_positions),
            ('trails', trail_positions),
            ('velocity', velocity_positions),
        ):
            positions_array = np.asarray(
                positions_data, dtype=np.float32
            ).reshape((-1, 3))
            self.gl_items['%s_%s' % (layer, suffix)].setData(
                pos=positions_array
            )

    def _update_gl_usv_tf(self, vehicles):
        axes = {'x': [], 'y': [], 'z': []}
        axis_length = 4.0
        for item in vehicles:
            start = (item['x'], item['y'], item['z'] + 0.2)
            cosine = math.cos(item['yaw'])
            sine = math.sin(item['yaw'])
            axes['x'].extend((
                start,
                (
                    start[0] + cosine * axis_length,
                    start[1] + sine * axis_length,
                    start[2],
                ),
            ))
            axes['y'].extend((
                start,
                (
                    start[0] - sine * axis_length,
                    start[1] + cosine * axis_length,
                    start[2],
                ),
            ))
            axes['z'].extend((
                start,
                (start[0], start[1], start[2] + axis_length),
            ))
            label = gl.GLTextItem(
                pos=(start[0], start[1], start[2] + axis_length + 0.5),
                color=pg.mkColor('#ffffff'),
                text='%s/base_link' % item['vehicle_id'],
                font=QFont('Sans Serif', 10),
            )
            self.gl_view.addItem(label)
            self.gl_labels.append(label)
        for axis, positions in axes.items():
            self.gl_items['tf_%s' % axis].setData(pos=np.asarray(
                positions, dtype=np.float32
            ).reshape((-1, 3)))

    def _refresh_gl(
        self, snapshot, now, points, heights, gl_colors, clusters
    ):
        if self.gl_view is None or self.view_mode != 'oblique':
            return
        self._clear_gl_labels()
        if points is not None:
            point_positions = np.column_stack((points, heights)).astype(
                np.float32, copy=False
            ) if len(points) else np.empty((0, 3), dtype=np.float32)
            point_kwargs = {
                'pos': point_positions,
                'size': self.point_size,
            }
            if gl_colors is not None:
                point_kwargs['color'] = gl_colors
            else:
                point_kwargs['color'] = (1.0, 1.0, 1.0, 0.92)
            self.gl_items['pointcloud'].setData(**point_kwargs)
            self.gl_point_frame_count = snapshot.get(
                'point_frame_count', 0
            )

        cluster_positions = np.asarray([
            (item['x'], item['y'], item['z']) for item in clusters
        ], dtype=np.float32).reshape((-1, 3))
        self.gl_items['clusters'].setData(pos=cluster_positions)
        cluster_lines = []
        for item in clusters:
            segments_3d = item.get('segments_3d')
            if segments_3d:
                cluster_lines.extend(
                    point for segment in segments_3d for point in segment
                )
            else:
                cluster_lines.extend(self._gl_segment_positions(
                    item['segments'], item['z']
                ))
            self._add_gl_label(item, 'clusters')
        self.gl_items['clusters_outlines'].setData(pos=np.asarray(
            cluster_lines, dtype=np.float32
        ).reshape((-1, 3)))
        self.gl_items['clusters_trails'].setData(
            pos=np.empty((0, 3), dtype=np.float32)
        )
        self.gl_items['clusters_velocity'].setData(
            pos=np.empty((0, 3), dtype=np.float32)
        )

        for layer in ('tracks', 'dynamic', 'fusion', 'ground_truth'):
            items = [
                item for item in snapshot['tracks'][layer].values()
                if self.layer_visibility[layer] and self._active(item, now)
            ]
            self._update_gl_layer(layer, items, snapshot)

        for layer, vehicle_type in (('uav', 1), ('usv', 2)):
            items = [
                item for item in snapshot['vehicles'].values()
                if self.layer_visibility[layer]
                and item['vehicle_type'] == vehicle_type
                and item.get('online', True)
                and self._active(item, now)
            ]
            self.gl_items[layer].setData(pos=np.asarray([
                (item['x'], item['y'], item['z']) for item in items
            ], dtype=np.float32).reshape((-1, 3)))
            trail_positions = []
            velocity_positions = []
            for item in items:
                if self.layer_visibility['trails']:
                    history = snapshot['histories'].get(
                        ('vehicle', item['vehicle_id']), []
                    )[-self.trajectory_length:]
                    trail_positions.extend(self._gl_segment_positions(
                        list(zip(history, history[1:])), item['z']
                    ))
                if self.layer_visibility['velocity']:
                    velocity_positions.extend(self._gl_segment_positions(
                        self._arrow_segments(
                            (item['x'], item['y']),
                            (
                                item['x'] + item['vx'] * 2.0,
                                item['y'] + item['vy'] * 2.0,
                            ),
                        ), item['z']
                    ))
                self._add_gl_label(item, layer)
            self.gl_items['%s_trails' % layer].setData(pos=np.asarray(
                trail_positions, dtype=np.float32
            ).reshape((-1, 3)))
            self.gl_items['%s_velocity' % layer].setData(pos=np.asarray(
                velocity_positions, dtype=np.float32
            ).reshape((-1, 3)))
            self.gl_items['%s_outlines' % layer].setData(
                pos=np.empty((0, 3), dtype=np.float32)
            )
        usv_tf_vehicles = [
            item for item in snapshot['vehicles'].values()
            if self.layer_visibility['tf']
            and item['vehicle_type'] == 2
            and item.get('online', True)
            and self._active(item, now)
        ]
        self._update_gl_usv_tf(usv_tf_vehicles)

    def refresh(self):
        started_at = time.perf_counter()
        snapshot = self.model.snapshot()
        now = time.monotonic()
        generation = snapshot['generation']
        following = self.follow_mode != 'none'
        point_age = now - snapshot['points_received_at']
        stale_transition = self.last_drawn_points > 0 and point_age > 3.0
        if (
            generation == self.last_generation
            and not following
            and not stale_transition
        ):
            return
        self.last_generation = generation
        self.last_snapshot = snapshot
        self._clear_labels()

        point_frame_count = snapshot.get('point_frame_count', 0)
        point_data_changed = (
            point_frame_count != self.last_point_frame_count
            or (
                self.view_mode == 'oblique'
                and point_frame_count != self.gl_point_frame_count
            )
            or stale_transition
        )
        points = heights = gl_colors = None
        if point_data_changed:
            point_fresh = point_age <= 3.0
            points = snapshot['points'] if (
                self.layer_visibility['pointcloud'] and point_fresh
            ) else np.empty((0, 2), dtype=np.float32)
            heights = snapshot['point_heights'] if len(points) else np.empty(
                (0,), dtype=np.float32
            )
            if len(points) > self.max_display_points:
                indices = np.linspace(
                    0, len(points) - 1,
                    self.max_display_points, dtype=np.int64,
                )
                points = points[indices]
                heights = heights[indices]
            point_brushes, gl_colors = self._point_colors(heights)
            point_options = {}
            if point_brushes is not None:
                point_options['brush'] = point_brushes
            else:
                point_options['brush'] = pg.mkBrush(
                    LAYER_STYLE['pointcloud']['color']
                )
            if self.view_mode == 'topdown':
                self.point_item.setData(
                    x=points[:, 0] if len(points) else [],
                    y=points[:, 1] if len(points) else [],
                    size=self.point_size,
                    **point_options,
                )
            self.last_point_frame_count = point_frame_count
            self.last_drawn_points = len(points)

        clusters = [
            cluster for cluster in snapshot['clusters']
            if self._active(cluster, now)
        ] if self.layer_visibility['clusters'] else []
        clusters = self._target_clusters(clusters, snapshot, now)
        segments = [
            segment for cluster in clusters for segment in cluster['segments']
        ]
        x_values, y_values = self._line_data(segments)
        self.cluster_item.setData(x_values, y_values)
        self.cluster_select_item.setData(spots=[
            {
                'pos': (cluster['x'], cluster['y']),
                'data': dict(cluster, layer='clusters'),
            }
            for cluster in clusters
        ])
        if self.layer_visibility['labels']:
            for cluster in clusters:
                self._add_label(
                    cluster['x'], cluster['y'], cluster['track_id'],
                    LAYER_STYLE['clusters']['color'],
                )

        self._draw_tracks(snapshot, now)
        self._draw_vehicles(snapshot, now)
        self._refresh_gl(
            snapshot, now, points, heights, gl_colors, clusters
        )
        self._apply_follow(snapshot, now)

        if self.last_render_at > 0.0:
            self.render_intervals.append(now - self.last_render_at)
        self.last_render_at = now
        point_frame_count = snapshot.get('point_frame_count', 0)
        if point_frame_count > self.rendered_point_frame_count + 1:
            self.overwritten_point_frames += (
                point_frame_count - self.rendered_point_frame_count - 1
            )
        self.rendered_point_frame_count = point_frame_count
        self.rendered_generation = generation
        self.rendered_frames += 1
        self.last_render_ms = (time.perf_counter() - started_at) * 1000.0

    def _on_scatter_clicked(self, _plot, spots):
        if not spots:
            return
        self.selected = spots[0].data()
        if self.selection_callback is not None:
            self.selection_callback(dict(self.selected))

    def set_layer_visible(self, layer, visible):
        if layer in self.layer_visibility:
            self.layer_visibility[layer] = bool(visible)
            if layer == 'pointcloud':
                self.last_point_frame_count = -1
                self.gl_point_frame_count = -1
            self.last_generation = -1
            self.refresh()

    def set_single_target_cluster(self, enabled):
        self.single_target_cluster = bool(enabled)
        self.last_generation = -1
        self.refresh()

    def _target_clusters(self, clusters, snapshot, now):
        if not self.single_target_cluster or len(clusters) <= 1:
            return clusters
        references = []
        for layer in ('dynamic', 'fusion', 'ground_truth', 'tracks'):
            references.extend(
                item for item in snapshot['tracks'][layer].values()
                if self._active(item, now)
            )
            if references:
                break
        if references:
            selected = min(
                clusters,
                key=lambda cluster: min(
                    math.hypot(
                        cluster['x'] - reference['x'],
                        cluster['y'] - reference['y'],
                    )
                    for reference in references
                ),
            )
            return [selected]
        selected = max(
            clusters,
            key=lambda cluster: max(0.0, cluster['dimensions'][0])
            * max(0.0, cluster['dimensions'][1])
            * max(0.1, cluster['dimensions'][2]),
        )
        return [selected]

    def set_grid_visible(self, visible):
        self.set_layer_visible('grid', visible)
        self.plot.showGrid(x=visible, y=visible, alpha=0.25)
        if self.gl_view is not None:
            self.gl_items['grid'].setVisible(bool(visible))

    def set_grid_spacing(self, spacing):
        self.grid_spacing = max(1.0, float(spacing))
        for name in ('bottom', 'left'):
            self.plot.getAxis(name).setTickSpacing(
                self.grid_spacing, self.grid_spacing / 5.0
            )
        if self.gl_view is not None:
            self.gl_items['grid'].setSpacing(
                self.grid_spacing, self.grid_spacing
            )

    def set_point_size(self, size):
        self.point_size = max(1.0, float(size))
        self.last_point_frame_count = -1
        self.gl_point_frame_count = -1
        self.last_generation = -1

    def set_point_color_mode(self, mode):
        self.point_color_mode = (
            mode if mode in ('height', 'fixed') else 'height'
        )
        self.last_point_frame_count = -1
        self.gl_point_frame_count = -1
        self.last_generation = -1

    def set_view_mode(self, mode):
        mode = mode if mode in ('topdown', 'oblique') else 'topdown'
        if mode == 'oblique' and self.gl_view is None:
            mode = 'topdown'
        self.view_mode = mode
        self.plot.setVisible(mode == 'topdown')
        if self.gl_view is not None:
            self.gl_view.setVisible(mode == 'oblique')
        if mode == 'oblique':
            self.gl_point_frame_count = -1
        else:
            self.last_point_frame_count = -1
        self.last_generation = -1
        self.reset_view()

    def set_display_range(self, half_range):
        self.default_range = max(5.0, float(half_range))
        self.reset_view()

    def set_trajectory_length(self, length):
        self.trajectory_length = max(2, int(length))
        self.last_generation = -1

    def set_max_display_points(self, value):
        self.max_display_points = max(100, int(value))
        self.last_point_frame_count = -1
        self.gl_point_frame_count = -1
        self.last_generation = -1

    def set_follow_mode(self, mode):
        self.follow_mode = (
            mode if mode in ('none', 'target', 'usv') else 'none'
        )
        self.last_generation = -1

    def reset_view(self):
        half = self.default_range
        self.plot.setXRange(-half, half, padding=0.0)
        self.plot.setYRange(-half, half, padding=0.0)
        if self.gl_view is not None:
            self.gl_items['grid'].setSize(2.0 * half, 2.0 * half)
            self.gl_view.setCameraPosition(
                pos=pg.Vector(0.0, 0.0, 0.0),
                distance=max(24.0, 2.1 * half),
                elevation=32.0,
                azimuth=-90.0,
            )

    def clear_trajectories(self):
        self.model.clear_histories()

    def point_status(self):
        if self.last_snapshot is None:
            return {}
        return dict(self.last_snapshot.get('point_status', {}))

    def display_statistics(self):
        intervals = [value for value in self.render_intervals if value > 0.0]
        mean_interval = (
            sum(intervals) / len(intervals) if intervals else 0.0
        )
        return {
            'render_rate_hz': 1.0 / mean_interval if mean_interval else 0.0,
            'render_ms': self.last_render_ms,
            'rendered_frames': self.rendered_frames,
            'drawn_points': self.last_drawn_points,
            'overwritten_point_frames': self.overwritten_point_frames,
        }
