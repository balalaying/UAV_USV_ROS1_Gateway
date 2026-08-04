"""Read-only Base Station radar presentation primitives for the Qt client.

The module deliberately has no ROS imports.  ``BaseStationStateParser`` turns
the Base Station Service JSON contract into small display records, the view
model owns selection and radar-relative geometry, and the widget only paints
the already prepared records.  The same records and conventions are suitable
for a future WebGL client.
"""

from dataclasses import dataclass, field
import math
import time

import numpy as np
from PyQt5.QtCore import QPoint, QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget


def _float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _position(value):
    if not isinstance(value, dict):
        return None
    pose = value.get('pose') or {}
    point = pose.get('position') if isinstance(pose, dict) else None
    point = point or value.get('position')
    if not isinstance(point, dict):
        return None
    return {
        'x': _float(point.get('x')),
        'y': _float(point.get('y')),
        'z': _float(point.get('z')),
    }


def _velocity(value):
    velocity = value.get('velocity') if isinstance(value, dict) else {}
    linear = velocity.get('linear') if isinstance(velocity, dict) else {}
    linear = linear if isinstance(linear, dict) else {}
    return {
        'x': _float(linear.get('x')),
        'y': _float(linear.get('y')),
        'z': _float(linear.get('z')),
    }


def transform_points_to_frame(points, transform):
    """Transform an Nx3 cloud using a geometry Transform-like object.

    It intentionally has no ROS dependency, which lets the display adapter
    verify its map-frame math independently from ROS subscription setup.
    """
    rotation = transform.rotation
    translation = transform.translation
    x, y, z, w = (
        float(rotation.x), float(rotation.y), float(rotation.z),
        float(rotation.w),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        matrix = np.eye(3, dtype=np.float32)
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        matrix = np.asarray([
            [1.0 - 2.0 * (y * y + z * z),
             2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),
             1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w),
             2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float32)
    offset = np.asarray(
        [translation.x, translation.y, translation.z], dtype=np.float32
    )
    return np.asarray(points, dtype=np.float32) @ matrix.T + offset


@dataclass
class SituationObject:
    object_id: str
    kind: str
    position: dict
    heading: float = 0.0
    velocity: dict = field(default_factory=dict)
    affiliation: str = 'UNKNOWN'
    classification: str = 'UNKNOWN'
    confidence: float = 0.0
    sources: list = field(default_factory=list)
    source_mask: int = 0
    online: bool = True
    stale: bool = False
    age_seconds: float = 0.0
    dimensions: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)


class BaseStationStateParser:
    """Validate and normalize the stable ``base_station_service.v1`` schema."""

    SCHEMA = 'base_station_service.v1'

    def parse(self, snapshot):
        if not isinstance(snapshot, dict):
            return self._empty('Base Station 状态不是 JSON 对象')
        if snapshot.get('schema_version') != self.SCHEMA:
            return self._empty('状态 schema 不匹配')

        base = snapshot.get('base_station') or {}
        base_position = _position(base) or {'x': 0.0, 'y': 0.0, 'z': 0.0}
        orientation = base.get('orientation') or {}
        result = {
            'valid': True,
            'error': '',
            'frame_id': str(snapshot.get('map_frame') or base.get('frame_id') or 'map'),
            'base_station': {
                'id': str(base.get('id') or 'base_station'),
                'position': base_position,
                'yaw': _float(orientation.get('yaw')),
                'communication_status': str(base.get('communication_status') or 'UNKNOWN'),
                'radar_display_range_m': max(50.0, _float(base.get('radar_display_range_m'), 300.0)),
            },
            'fleet': [],
            'entities': [],
            'targets': [],
            'target_history': snapshot.get('target_history') or {},
            'predictions': snapshot.get('predictions') or [],
            'threats': snapshot.get('threats') or [],
            'perception': snapshot.get('perception') or {},
            'health': snapshot.get('health') or {},
            'received_at': _float(snapshot.get('received_at_wall_time'), time.time()),
            'raw': snapshot,
        }
        fleet = snapshot.get('fleet') or {}
        for group in ('uav', 'usv', 'unknown'):
            for vehicle in fleet.get(group) or []:
                record = self._object(vehicle, group.upper())
                if record:
                    result['fleet'].append(record)
        for entity in snapshot.get('entities') or []:
            record = self._object(entity, 'ENTITY')
            if record and record.object_id not in ('base_station', 'shore_command_base'):
                result['entities'].append(record)
        for target in snapshot.get('targets') or []:
            record = self._object(target, 'TARGET')
            if record:
                result['targets'].append(record)
        return result

    @staticmethod
    def _empty(error):
        return {
            'valid': False, 'error': error, 'frame_id': 'map',
            'base_station': {
                'id': 'base_station',
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'yaw': 0.0,
                'communication_status': 'WAITING',
                'radar_display_range_m': 300.0,
            },
            'fleet': [], 'entities': [], 'targets': [], 'target_history': {},
            'predictions': [], 'threats': [], 'perception': {}, 'health': {},
            'received_at': time.time(), 'raw': {},
        }

    @staticmethod
    def _object(raw, kind):
        position = _position(raw)
        if position is None:
            return None
        velocity = _velocity(raw)
        heading = raw.get('heading_rad') if isinstance(raw, dict) else None
        if heading is None and math.hypot(velocity['x'], velocity['y']) > 0.05:
            heading = math.atan2(velocity['y'], velocity['x'])
        sources = raw.get('sources') or raw.get('source') or []
        if not isinstance(sources, list):
            sources = [str(sources)]
        return SituationObject(
            object_id=str(raw.get('target_id') or raw.get('id') or raw.get('uuid') or 'unknown'),
            kind=kind,
            position=position,
            heading=_float(heading),
            velocity=velocity,
            affiliation=str(raw.get('affiliation') or 'UNKNOWN').upper(),
            classification=str(raw.get('classification') or raw.get('class') or raw.get('type') or kind).upper(),
            confidence=_float(raw.get('confidence')),
            sources=[str(value) for value in sources],
            source_mask=int(_float(raw.get('source_mask'))),
            online=bool(raw.get('online', True)),
            stale=bool(raw.get('stale', False)),
            age_seconds=_float(raw.get('age_seconds', raw.get('stamp_age_seconds', 0.0))),
            dimensions=raw.get('dimensions') or {},
            raw=raw,
        )


class SituationViewModel:
    """Owns snapshot state, relative geometry, selection and display layers."""

    DEFAULT_RANGE = 300.0
    STALE_SECONDS = 2.0

    def __init__(self, parser=None, clock=time.time):
        self.parser = parser or BaseStationStateParser()
        self.clock = clock
        self.snapshot = self.parser.parse({})
        self.selected_id = None
        self.locked_id = None
        self.auto_range = False
        self.range_m = self.DEFAULT_RANGE
        self.layers = {
            'grid': True, 'rings': True, 'ticks': True, 'scan': True,
            'fleet': True, 'entities': True, 'targets': True, 'tracks': True,
            'predictions': True, 'velocity': True, 'threats': True,
            'labels': True, 'offline': True, 'stale': True, 'sources': True,
        }

    def update_snapshot(self, snapshot):
        self.snapshot = self.parser.parse(snapshot)
        configured = self.snapshot['base_station']['radar_display_range_m']
        if not self.auto_range:
            self.range_m = max(50.0, configured)

    def set_layer(self, layer, visible):
        if layer in self.layers:
            self.layers[layer] = bool(visible)

    def set_range(self, range_m):
        self.auto_range = False
        self.range_m = max(50.0, float(range_m))

    def enable_auto_range(self, enabled=True):
        self.auto_range = bool(enabled)

    def base_position(self):
        return self.snapshot['base_station']['position']

    def relative(self, item):
        base = self.base_position()
        dx = item.position['x'] - base['x']
        dy = item.position['y'] - base['y']
        yaw = self.snapshot['base_station']['yaw']
        return {
            'dx': dx,
            'dy': dy,
            'distance': math.hypot(dx, dy),
            'bearing_deg': (math.degrees(math.atan2(dy, dx) - yaw) + 360.0) % 360.0,
        }

    def objects(self):
        return self.snapshot['fleet'] + self.snapshot['entities'] + self.snapshot['targets']

    def visible_range(self):
        if not self.auto_range:
            return self.range_m
        distances = [self.relative(item)['distance'] for item in self.objects()]
        return max(50.0, min(500.0, max(distances, default=50.0) + 35.0))

    def select(self, object_id):
        self.selected_id = object_id

    def selected(self):
        for item in self.objects():
            if item.object_id == self.selected_id:
                return item
        return None

    def threat_for(self, object_id):
        for threat in self.snapshot['threats']:
            if str(threat.get('target_id')) == str(object_id):
                return threat
        return {}

    def state_age(self):
        received_at = self.snapshot.get('received_at') or self.clock()
        return max(0.0, self.clock() - received_at)

    def is_stale(self, item):
        return item.stale or item.age_seconds > self.STALE_SECONDS or self.state_age() > self.STALE_SECONDS


class RadarCanvas(QWidget):
    """Interactive 360-degree read-only radar canvas."""

    object_selected = pyqtSignal(object)

    COLOR = {
        'background': QColor('#061016'), 'grid': QColor('#15303b'),
        'axis': QColor('#315466'), 'ring': QColor('#2b5e75'),
        'text': QColor('#d9edf4'), 'muted': QColor('#88a6b5'),
        'base': QColor('#ffe082'), 'uav': QColor('#74b8ff'),
        'usv_01': QColor('#338cff'), 'usv_02': QColor('#39c978'),
        'usv_03': QColor('#30d9dc'), 'friendly': QColor('#ffd046'),
        'hostile': QColor('#ff5454'), 'unknown': QColor('#f4c857'),
        'neutral': QColor('#9da8ad'), 'fusion': QColor('#45e6a8'),
    }

    def __init__(self, view_model=None):
        super().__init__()
        self.model = view_model or SituationViewModel()
        self.scan_angle = 0.0
        self.pan = QPointF(0.0, 0.0)
        self._drag_origin = None
        self._last_hit_items = []
        self.setMinimumSize(660, 560)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self._scan_timer = QTimer(self)
        self._scan_timer.timeout.connect(self._advance_scan)
        self._scan_timer.start(40)

    def set_snapshot(self, snapshot):
        self.model.update_snapshot(snapshot)
        self.update()

    def set_layer_visible(self, layer, visible):
        self.model.set_layer(layer, visible)
        self.update()

    def set_range(self, range_m):
        self.model.set_range(range_m)
        self.update()

    def set_auto_range(self, enabled):
        self.model.enable_auto_range(enabled)
        self.update()

    def reset_view(self):
        self.pan = QPointF(0.0, 0.0)
        self.model.selected_id = None
        self.update()

    def _advance_scan(self):
        self.scan_angle = (self.scan_angle + 1.7) % 360.0
        if self.model.layers['scan']:
            self.update()

    def _geometry(self):
        margin = 62.0
        radius = max(60.0, min(self.width(), self.height()) * 0.5 - margin)
        return QPointF(self.width() * 0.5, self.height() * 0.53) + self.pan, radius

    def _screen(self, position, center, scale):
        base = self.model.base_position()
        return QPointF(
            center.x() + (position['x'] - base['x']) * scale,
            center.y() - (position['y'] - base['y']) * scale,
        )

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), self.COLOR['background'])
        center, radius = self._geometry()
        range_m = self.model.visible_range()
        scale = radius / range_m
        self._last_hit_items = []
        self._draw_grid(painter, center, radius, range_m, scale)
        self._draw_scan(painter, center, radius)
        self._draw_base(painter, center)
        self._draw_histories(painter, center, scale)
        self._draw_predictions(painter, center, scale)
        self._draw_objects(painter, center, scale)
        self._draw_header(painter, range_m)

    def _draw_grid(self, painter, center, radius, range_m, scale):
        if self.model.layers['grid']:
            painter.setPen(QPen(self.COLOR['grid'], 1))
            step = 25.0 if range_m <= 200.0 else 50.0
            limit = int(math.ceil(range_m / step))
            for index in range(-limit, limit + 1):
                offset = index * step * scale
                painter.drawLine(QPointF(center.x() - radius, center.y() + offset), QPointF(center.x() + radius, center.y() + offset))
                painter.drawLine(QPointF(center.x() + offset, center.y() - radius), QPointF(center.x() + offset, center.y() + radius))
        painter.setPen(QPen(self.COLOR['axis'], 1))
        painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius))
        if self.model.layers['rings']:
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(self.COLOR['ring'], 1, Qt.DotLine))
            for ratio in (0.25, 0.5, 0.75, 1.0):
                ring = radius * ratio
                painter.drawEllipse(center, ring, ring)
                painter.setPen(QPen(self.COLOR['muted'], 1))
                painter.drawText(center + QPointF(7, -ring - 5), '%.0f m' % (range_m * ratio))
                painter.setPen(QPen(self.COLOR['ring'], 1, Qt.DotLine))
        if self.model.layers['ticks']:
            painter.setPen(QPen(self.COLOR['muted'], 1))
            for degrees in range(0, 360, 30):
                angle = math.radians(degrees)
                outer = QPointF(center.x() + math.sin(angle) * radius, center.y() - math.cos(angle) * radius)
                inner = QPointF(center.x() + math.sin(angle) * (radius - 8), center.y() - math.cos(angle) * (radius - 8))
                painter.drawLine(inner, outer)
                if degrees % 90 == 0:
                    label = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}[degrees]
                    painter.drawText(outer + QPointF(-5, 5), label)
                elif degrees % 60 == 0:
                    painter.drawText(outer + QPointF(-9, 5), str(degrees))

    def _draw_scan(self, painter, center, radius):
        if not self.model.layers['scan']:
            return
        angle = math.radians(self.scan_angle)
        end = QPointF(center.x() + math.sin(angle) * radius, center.y() - math.cos(angle) * radius)
        painter.setPen(QPen(QColor(67, 229, 181, 210), 2))
        painter.drawLine(center, end)
        painter.setPen(QPen(QColor(67, 229, 181, 50), 12))
        for offset in (2, 4, 6, 8):
            fade = math.radians(self.scan_angle - offset)
            painter.drawLine(center, QPointF(center.x() + math.sin(fade) * radius, center.y() - math.cos(fade) * radius))

    def _draw_base(self, painter, center):
        painter.setBrush(self.COLOR['base'])
        painter.setPen(QPen(QColor('#fff2ba'), 2))
        painter.drawEllipse(center, 8, 8)
        painter.drawLine(center + QPointF(-15, 0), center + QPointF(15, 0))
        painter.drawLine(center + QPointF(0, -15), center + QPointF(0, 15))
        painter.setPen(self.COLOR['base'])
        painter.drawText(center + QPointF(14, -15), self.model.snapshot['base_station']['id'].upper())

    def _draw_histories(self, painter, center, scale):
        if not self.model.layers['tracks']:
            return
        painter.setPen(QPen(QColor(255, 113, 113, 150), 1.5))
        for points in self.model.snapshot['target_history'].values():
            screen_points = []
            for point in points:
                position = _position(point) or point.get('position') if isinstance(point, dict) else None
                if isinstance(position, dict):
                    screen_points.append(self._screen(position, center, scale))
            if len(screen_points) > 1:
                painter.drawPolyline(QPolygonF(screen_points))

    def _draw_predictions(self, painter, center, scale):
        if not self.model.layers['predictions']:
            return
        painter.setPen(QPen(QColor('#f2a84b'), 1.5, Qt.DashLine))
        for prediction in self.model.snapshot['predictions']:
            points = []
            for item in prediction.get('points', []) if isinstance(prediction, dict) else []:
                position = _position(item) or (item.get('position') if isinstance(item, dict) else None)
                if isinstance(position, dict):
                    points.append(self._screen(position, center, scale))
            if len(points) > 1:
                painter.drawPolyline(QPolygonF(points))

    def _draw_objects(self, painter, center, scale):
        for item in self.model.snapshot['fleet']:
            if not self.model.layers['fleet']:
                continue
            if (not item.online and not self.model.layers['offline']) or (self.model.is_stale(item) and not self.model.layers['stale']):
                continue
            self._draw_item(painter, item, center, scale)
        if self.model.layers['entities']:
            for item in self.model.snapshot['entities']:
                self._draw_item(painter, item, center, scale)
        if self.model.layers['targets']:
            for item in self.model.snapshot['targets']:
                self._draw_item(painter, item, center, scale)

    def _color_for(self, item):
        item_id = item.object_id.lower()
        if item.kind == 'UAV':
            return self.COLOR['uav']
        if item.kind == 'USV':
            return self.COLOR.get(item_id, self.COLOR['unknown'])
        if 'friendly' in item_id or item.affiliation == 'FRIENDLY':
            return self.COLOR['friendly']
        if 'enemy' in item_id or item.affiliation == 'HOSTILE':
            return self.COLOR['hostile']
        if item.affiliation == 'NEUTRAL':
            return self.COLOR['neutral']
        if item.raw.get('is_real_fusion'):
            return self.COLOR['fusion']
        return self.COLOR['unknown']

    def _draw_item(self, painter, item, center, scale):
        point = self._screen(item.position, center, scale)
        color = self._color_for(item)
        stale = self.model.is_stale(item)
        if stale or not item.online:
            color = QColor(color)
            color.setAlpha(95)
        selected = item.object_id == self.model.selected_id
        if selected:
            painter.setPen(QPen(QColor('#ffffff'), 2, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(point, 16, 16)
        if item.kind == 'TARGET':
            painter.setPen(QPen(color, 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(int(point.x() - 7), int(point.y() - 7), 14, 14)
        elif item.kind == 'UAV':
            self._draw_triangle(painter, point, item.heading, color, 10)
        elif item.kind == 'USV':
            self._draw_ship(painter, point, item.heading, color)
        else:
            painter.setPen(QPen(color.darker(150), 2))
            painter.setBrush(color)
            painter.drawRoundedRect(int(point.x() - 9), int(point.y() - 5), 18, 10, 2, 2)
        if self.model.layers['velocity'] and math.hypot(item.velocity.get('x', 0.0), item.velocity.get('y', 0.0)) > 0.05:
            length = min(42.0, 10.0 + math.hypot(item.velocity['x'], item.velocity['y']) * 4.0)
            painter.setPen(QPen(color, 1.5))
            painter.drawLine(point, QPointF(point.x() + math.cos(item.heading) * length, point.y() - math.sin(item.heading) * length))
        if self.model.layers['threats']:
            threat = self.model.threat_for(item.object_id)
            level = str(threat.get('threat_level') or '').upper()
            if level in ('HIGH', 'CRITICAL'):
                painter.setPen(QPen(QColor('#ff4c4c'), 2, Qt.DashLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(point, 18, 18)
        if self.model.layers['labels']:
            label = item.object_id.upper()
            if item.kind == 'UAV':
                label += ' %.1fm' % item.position['z']
            if self.model.layers['sources'] and item.kind == 'TARGET':
                source = 'REAL FUSION' if item.raw.get('is_real_fusion') else 'GROUND TRUTH' if item.raw.get('is_ground_truth_fallback') else ','.join(item.sources[:1])
                label += ' | ' + (source or 'UNKNOWN')
            painter.setPen(color.lighter(150))
            painter.drawText(point + QPointF(11, -10), label)
        self._last_hit_items.append((point, item))

    @staticmethod
    def _draw_triangle(painter, point, heading, color, size):
        painter.setPen(QPen(color.darker(150), 1.5))
        painter.setBrush(color)
        polygon = QPolygonF([
            QPointF(point.x() + math.cos(heading) * size, point.y() - math.sin(heading) * size),
            QPointF(point.x() + math.cos(heading + 2.45) * size * 0.8, point.y() - math.sin(heading + 2.45) * size * 0.8),
            QPointF(point.x() + math.cos(heading - 2.45) * size * 0.8, point.y() - math.sin(heading - 2.45) * size * 0.8),
        ])
        painter.drawPolygon(polygon)

    @staticmethod
    def _draw_ship(painter, point, heading, color):
        painter.save()
        painter.translate(point)
        painter.rotate(-math.degrees(heading))
        painter.setPen(QPen(color.darker(150), 1.5))
        painter.setBrush(color)
        painter.drawRoundedRect(-11, -5, 22, 10, 3, 3)
        painter.setPen(QPen(QColor('#ffffff'), 1.5))
        painter.drawLine(2, -4, 8, 0)
        painter.drawLine(8, 0, 2, 4)
        painter.restore()

    def _draw_header(self, painter, range_m):
        base = self.model.snapshot['base_station']
        perception = self.model.snapshot['perception']
        painter.setFont(QFont('Sans Serif', 10, QFont.Bold))
        painter.setPen(self.COLOR['text'])
        painter.drawText(16, 25, 'BASE STATION RADAR | frame: %s' % self.model.snapshot['frame_id'])
        painter.setFont(QFont('Sans Serif', 9))
        painter.setPen(self.COLOR['muted'])
        painter.drawText(16, 45, '中心: %s | 量程: %.0f m | 通信: %s | 主感知: %s' % (
            base['id'], range_m, base['communication_status'], perception.get('primary_source', 'UNKNOWN')
        ))
        if not self.model.snapshot['valid']:
            painter.setPen(QColor('#ff8b8b'))
            painter.drawText(16, 65, self.model.snapshot['error'])

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            hit = min(self._last_hit_items, key=lambda entry: (entry[0] - event.pos()).manhattanLength(), default=None)
            if hit is not None and (hit[0] - event.pos()).manhattanLength() <= 22:
                self.model.select(hit[1].object_id)
                self.object_selected.emit(hit[1])
                self.update()
            else:
                self.model.select(None)
                self.object_selected.emit(None)
                self.update()
        elif event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._drag_origin = event.pos()

    def mouseMoveEvent(self, event):
        if self._drag_origin is not None:
            delta = event.pos() - self._drag_origin
            self.pan += QPointF(delta.x(), delta.y())
            self._drag_origin = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._drag_origin = None

    def mouseDoubleClickEvent(self, event):
        del event
        self.reset_view()

    def wheelEvent(self, event):
        factor = 0.85 if event.angleDelta().y() > 0 else 1.18
        self.model.set_range(max(50.0, min(2000.0, self.model.visible_range() * factor)))
        self.update()
