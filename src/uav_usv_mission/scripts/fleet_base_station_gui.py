#!/usr/bin/env python3
import math
import sys
import threading
import time

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QObject, QPointF, Qt, pyqtSignal
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtGui import QPolygonF
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QHeaderView
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPlainTextEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QSplitter
from PyQt5.QtWidgets import QTabWidget
from PyQt5.QtWidgets import QTableWidget
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import SensorStatus
from uav_usv_interfaces.msg import VehicleState


class GuiSignals(QObject):
    image = pyqtSignal(QImage)
    scan = pyqtSignal(object)
    sensor = pyqtSignal(object)
    vehicle = pyqtSignal(object)
    defense = pyqtSignal(object)
    defense_own = pyqtSignal(object)
    defense_enemy = pyqtSignal(object)
    log = pyqtSignal(str)


class VideoMosaicLabel(QLabel):
    """Paint the camera mosaic over the full widget area."""

    def __init__(self, text=''):
        super().__init__(text)
        self._image = None
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return max(270, min(420, int(width * 9.0 / 32.0)))

    def resizeEvent(self, event):
        self.setFixedHeight(self.heightForWidth(max(1, self.width())))
        super().resizeEvent(event)

    def set_image(self, image):
        self._image = image.copy()
        self.update()

    def paintEvent(self, event):
        if self._image is None:
            super().paintEvent(event)
            return
        painter = QPainter(self)
        painter.drawImage(self.rect(), self._image)


class BaseStationGuiNode(Node):
    VEHICLE_NAMES = {
        'uav_01': '无人机01',
        'uav_02': '无人机02',
        'uav_03': '无人机03',
        'uav_04': '无人机04',
        'usv_01': '船01',
        'usv_02': '船02',
        'usv_03': '船03',
        'usv_04': '船04',
    }

    def __init__(self, signals):
        super().__init__('fleet_base_station_gui')
        self.signals = signals
        self.declare_parameter('defense_node_name', '/defense_sim_demo')

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        state_qos = QoSProfile(depth=5)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        state_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            Image,
            '/fleet/base/camera_mosaic',
            self._on_image,
            image_qos,
        )
        self.create_subscription(
            LaserScan,
            '/fleet/base/usv_scan',
            self._on_scan,
            sensor_qos,
        )
        self.create_subscription(
            SensorStatus,
            '/fleet/sensor_status',
            self._on_sensor,
            20,
        )
        self.create_subscription(
            VehicleState,
            '/fleet/state',
            self._on_vehicle,
            state_qos,
        )
        self.create_subscription(
            CommandAck,
            '/fleet/command_ack',
            self._on_ack,
            20,
        )
        self.create_subscription(
            String,
            '/defense/status',
            self._on_defense_status,
            10,
        )
        self.create_subscription(
            PoseArray,
            '/defense/own_ships',
            self._on_defense_own,
            10,
        )
        self.create_subscription(
            PoseArray,
            '/defense/enemy_ships',
            self._on_defense_enemy,
            10,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, '/fleet/base/operator_goal', 10
        )
        self.action_pub = self.create_publisher(
            String, '/fleet/base/operator_action', 10
        )
        defense_node_name = str(self.get_parameter('defense_node_name').value)
        defense_node_name = '/' + defense_node_name.strip('/')
        self.defense_param_client = self.create_client(
            SetParameters,
            '%s/set_parameters' % defense_node_name,
        )

    def _on_image(self, msg):
        encoding = msg.encoding.lower()
        if encoding not in ('bgr8', 'rgb8'):
            return
        image_format = (
            QImage.Format_BGR888
            if encoding == 'bgr8'
            else QImage.Format_RGB888
        )
        image = QImage(
            bytes(msg.data),
            msg.width,
            msg.height,
            msg.step,
            image_format,
        ).copy()
        self.signals.image.emit(image)

    def _on_scan(self, msg):
        sample_step = max(1, len(msg.ranges) // 720)
        points = []
        for index in range(0, len(msg.ranges), sample_step):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance):
                continue
            if msg.range_min <= distance <= msg.range_max:
                angle = msg.angle_min + index * msg.angle_increment
                points.append((angle, distance))
        self.signals.scan.emit((points, float(msg.range_max)))

    def _on_sensor(self, msg):
        self.signals.sensor.emit(
            (
                msg.vehicle_id,
                msg.sensor_id,
                msg.measured_rate_hz,
                msg.age_seconds,
                msg.total_messages,
                msg.total_bytes,
                msg.healthy,
            )
        )

    def _on_vehicle(self, msg):
        self.signals.vehicle.emit(
            (
                msg.vehicle_id,
                msg.online,
                msg.armed,
                msg.mode,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.status_text,
            )
        )

    def _on_ack(self, msg):
        labels = {
            CommandAck.STATUS_RECEIVED: '收到',
            CommandAck.STATUS_ACCEPTED: '接受',
            CommandAck.STATUS_EXECUTING: '执行中',
            CommandAck.STATUS_SUCCEEDED: '成功',
            CommandAck.STATUS_REJECTED: '拒绝',
            CommandAck.STATUS_FAILED: '失败',
            CommandAck.STATUS_CANCELED: '取消',
        }
        self.signals.log.emit(
            '%s  %s  %s  %.0f%%  %s'
            % (
                self.VEHICLE_NAMES.get(msg.vehicle_id, msg.vehicle_id),
                msg.command_id,
                labels.get(msg.status, str(msg.status)),
                msg.progress * 100.0,
                msg.message,
            )
        )

    def _on_defense_status(self, msg):
        fields = {}
        for token in msg.data.split():
            if '=' not in token:
                continue
            key, value = token.split('=', 1)
            fields[key] = value
        self.signals.defense.emit(fields)

    def _on_defense_own(self, msg):
        ships = []
        for index, pose in enumerate(msg.poses, start=1):
            ships.append(
                {
                    'name': 'own_%02d' % index,
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'yaw': self._yaw_from_pose(pose),
                }
            )
        self.signals.defense_own.emit(ships)

    def _on_defense_enemy(self, msg):
        ships = []
        for index, pose in enumerate(msg.poses, start=1):
            ships.append(
                {
                    'name': 'enemy_%02d' % index,
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'yaw': self._yaw_from_pose(pose),
                }
            )
        self.signals.defense_enemy.emit(ships)

    @staticmethod
    def _yaw_from_pose(pose):
        z = pose.orientation.z
        w = pose.orientation.w
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def publish_goal(self, x, y, altitude):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(altitude)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def publish_action(self, action):
        msg = String()
        msg.data = action
        self.action_pub.publish(msg)

    def set_defense_parameters(self, values):
        if not values:
            return False
        if not self.defense_param_client.wait_for_service(timeout_sec=0.05):
            self.signals.log.emit('防御参数服务未就绪，稍后再试')
            return False
        request = SetParameters.Request()
        for name, value in values.items():
            parameter = Parameter()
            parameter.name = name
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            )
            request.parameters.append(parameter)
        future = self.defense_param_client.call_async(request)
        future.add_done_callback(self._on_defense_parameters_set)
        return True

    def _on_defense_parameters_set(self, future):
        try:
            results = future.result().results
        except Exception as exc:
            self.signals.log.emit('防御参数写入失败: %s' % exc)
            return
        if all(result.successful for result in results):
            self.signals.log.emit('防御参数已实时更新')
            return
        reasons = [
            result.reason for result in results
            if not result.successful and result.reason
        ]
        self.signals.log.emit(
            '防御参数部分写入失败: %s' % ('; '.join(reasons) or '未知原因')
        )


class RadarWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.points = []
        self.range_max = 40.0
        self.setMinimumSize(520, 300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_scan(self, scan):
        self.points, self.range_max = scan
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#101820'))

        plot_width = max(260.0, self.width() - 250.0)
        center = QPointF(plot_width / 2.0 + 18.0, self.height() / 2.0 + 8.0)
        radius = max(10.0, min(plot_width, self.height()) / 2.0 - 38.0)
        painter.setPen(QPen(QColor('#35505f'), 1))
        for fraction in (0.25, 0.5, 0.75, 1.0):
            ring = radius * fraction
            painter.drawEllipse(center, ring, ring)
            painter.setPen(QPen(QColor('#496574'), 1))
            painter.drawText(
                center + QPointF(ring + 6.0, -4.0),
                '%.0fm' % (self.range_max * fraction),
            )
            painter.setPen(QPen(QColor('#35505f'), 1))
        painter.drawLine(
            QPointF(center.x() - radius, center.y()),
            QPointF(center.x() + radius, center.y()),
        )
        painter.drawLine(
            QPointF(center.x(), center.y() - radius),
            QPointF(center.x(), center.y() + radius),
        )
        painter.setPen(QPen(QColor('#6f8998'), 1))
        painter.drawText(center + QPointF(-14.0, -radius - 10.0), '前')
        painter.drawText(center + QPointF(-radius - 22.0, 4.0), '左')
        painter.drawText(center + QPointF(radius + 10.0, 4.0), '右')
        painter.drawText(center + QPointF(-14.0, radius + 20.0), '后')

        painter.setPen(QPen(QColor('#f3c969'), 2))
        painter.drawLine(center, center + QPointF(0.0, -radius * 0.36))
        painter.setBrush(QColor('#f3c969'))
        painter.drawEllipse(center, 5, 5)

        painter.setPen(QPen(QColor('#33e6c4'), 3))
        scale = radius / max(0.1, self.range_max)
        for angle, distance in self.points:
            x = center.x() + math.cos(angle) * distance * scale
            y = center.y() - math.sin(angle) * distance * scale
            painter.drawPoint(QPointF(x, y))

        info_x = int(plot_width + 34.0)
        painter.setPen(QColor('#d7e7ee'))
        painter.drawText(info_x, 26, '船载激光雷达局部视图')
        painter.setPen(QColor('#94aebc'))
        painter.drawText(info_x, 54, '中心黄点：当前船体')
        painter.drawText(info_x, 78, '黄色短线：船头方向')
        painter.setPen(QColor('#33e6c4'))
        painter.drawText(info_x, 102, '青色回波：雷达扫到的物体')
        painter.setPen(QColor('#94aebc'))
        painter.drawText(info_x, 126, '圆圈刻度：距离本船的半径')
        painter.drawText(info_x, 158, '量程：%.1f m' % self.range_max)
        painter.drawText(info_x, 182, '点数：%d' % len(self.points))
        painter.setPen(QPen(QColor('#35505f'), 1))
        painter.drawLine(info_x, 200, self.width() - 18, 200)
        painter.setPen(QColor('#b7c9d3'))
        painter.drawText(info_x, 226, '用途：观察船附近障碍物')
        painter.drawText(info_x, 250, '不是全局地图，只显示局部扫描')


class DefenseMapWidget(QWidget):
    OWN_LABELS = {
        'own_01': '我方01',
        'own_02': '我方02',
        'own_03': '我方03',
        'own_04': '我方04',
    }
    ENEMY_LABELS = {
        'enemy_01': '敌方01',
        'enemy_02': '敌方02',
        'enemy_03': '敌方03',
        'enemy_04': '敌方04',
    }

    def __init__(self):
        super().__init__()
        self.status = {}
        self.own_ships = []
        self.enemy_ships = []
        self.own_targets = {}
        self.enemy_states = {}
        self.base_x = 0.0
        self.base_y = 0.0
        self.defend_radius = 75.0
        self.trigger_radius = 190.0
        self.base_safety_radius = 18.0
        self.setMinimumSize(720, 560)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_status(self, fields):
        self.status = fields
        base = fields.get('base', '0:0').split(':')
        if len(base) >= 2:
            self.base_x = self._float(base[0], self.base_x)
            self.base_y = self._float(base[1], self.base_y)
        self.defend_radius = self._float(
            fields.get('defend_radius'), self.defend_radius
        )
        self.trigger_radius = self._float(
            fields.get('trigger_radius'), self.trigger_radius
        )
        self.base_safety_radius = self._float(
            fields.get('base_safety_radius'), self.base_safety_radius
        )
        self.own_targets = self._parse_own_targets(
            fields.get('own_targets', '')
        )
        self.enemy_states = self._parse_enemy_states(
            fields.get('enemy_states', '')
        )
        self.update()

    def set_own_ships(self, ships):
        self.own_ships = ships
        self.update()

    def set_enemy_ships(self, ships):
        self.enemy_ships = ships
        self.update()

    @staticmethod
    def _float(value, fallback=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return fallback

    def _parse_own_targets(self, text):
        targets = {}
        for item in text.split(','):
            parts = item.split(':')
            if len(parts) < 4:
                continue
            targets[parts[0]] = {
                'x': self._float(parts[1]),
                'y': self._float(parts[2]),
                'state': parts[3],
            }
        return targets

    def _parse_enemy_states(self, text):
        states = {}
        for item in text.split(','):
            parts = item.split(':')
            if len(parts) < 3:
                continue
            states[parts[0]] = {
                'state': parts[1],
                'distance': self._float(parts[2]),
            }
        return states

    def _world_to_screen(self, x, y, center, scale):
        return QPointF(
            center.x() + (x - self.base_x) * scale,
            center.y() - (y - self.base_y) * scale,
        )

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#f6fbfd'))

        margin = 36.0
        center = QPointF(self.width() * 0.48, self.height() * 0.53)
        world_radius = max(
            self.trigger_radius + 45.0,
            self._max_visible_radius() + 30.0,
        )
        scale = (min(self.width(), self.height()) / 2.0 - margin) / world_radius

        self._draw_grid(painter, center, scale, world_radius)
        self._draw_circle(
            painter,
            center,
            self.trigger_radius * scale,
            QColor(255, 126, 0, 45),
            QColor('#f06d00'),
            2,
            '预警圈 %.0fm' % self.trigger_radius,
        )
        self._draw_circle(
            painter,
            center,
            self.defend_radius * scale,
            QColor(0, 126, 255, 55),
            QColor('#0077cc'),
            2,
            '防守圈 %.0fm' % self.defend_radius,
        )
        self._draw_circle(
            painter,
            center,
            self.base_safety_radius * scale,
            QColor(214, 40, 40, 60),
            QColor('#c82124'),
            2,
            '安全圈 %.0fm' % self.base_safety_radius,
        )

        base = self._world_to_screen(self.base_x, self.base_y, center, scale)
        painter.setPen(QPen(QColor('#17324d'), 2))
        painter.setBrush(QColor('#25c46a'))
        painter.drawEllipse(base, 11, 11)
        painter.drawText(base + QPointF(14, -10), '大本营')

        for enemy in self.enemy_ships:
            self._draw_enemy(painter, enemy, center, scale)
        for own in self.own_ships:
            self._draw_own(painter, own, center, scale)

        painter.setPen(QPen(QColor('#43525b'), 1))
        mode = self.status.get('mode', 'waiting')
        mode_text = '防守中' if mode == 'guard' else '巡逻中'
        painter.drawText(
            18,
            24,
            'Defense Map | %s | threats=%s | blocked=%s'
            % (
                mode_text,
                self.status.get('threats', '-'),
                self.status.get('blocked', '-'),
            ),
        )

    def _max_visible_radius(self):
        points = []
        for ship in self.own_ships + self.enemy_ships:
            points.append((ship['x'], ship['y']))
        for target in self.own_targets.values():
            points.append((target['x'], target['y']))
        if not points:
            return self.trigger_radius
        return max(
            math.hypot(x - self.base_x, y - self.base_y)
            for x, y in points
        )

    def _draw_grid(self, painter, center, scale, world_radius):
        painter.setPen(QPen(QColor('#d7e1e6'), 1))
        step = 50.0
        limit = math.ceil(world_radius / step) * step
        value = -limit
        while value <= limit:
            p1 = self._world_to_screen(self.base_x - limit, self.base_y + value, center, scale)
            p2 = self._world_to_screen(self.base_x + limit, self.base_y + value, center, scale)
            painter.drawLine(p1, p2)
            p3 = self._world_to_screen(self.base_x + value, self.base_y - limit, center, scale)
            p4 = self._world_to_screen(self.base_x + value, self.base_y + limit, center, scale)
            painter.drawLine(p3, p4)
            value += step
        painter.setPen(QPen(QColor('#9db1bb'), 1))
        painter.drawLine(
            self._world_to_screen(self.base_x - limit, self.base_y, center, scale),
            self._world_to_screen(self.base_x + limit, self.base_y, center, scale),
        )
        painter.drawLine(
            self._world_to_screen(self.base_x, self.base_y - limit, center, scale),
            self._world_to_screen(self.base_x, self.base_y + limit, center, scale),
        )

    def _draw_circle(self, painter, center, radius, fill, line, width, text):
        painter.setBrush(fill)
        painter.setPen(QPen(line, width))
        painter.drawEllipse(center, radius, radius)
        painter.setPen(QPen(line, 1))
        painter.drawText(center + QPointF(radius + 8, -4), text)

    def _draw_enemy(self, painter, ship, center, scale):
        point = self._world_to_screen(ship['x'], ship['y'], center, scale)
        base = self._world_to_screen(self.base_x, self.base_y, center, scale)
        state = self.enemy_states.get(ship['name'], {}).get('state', '')
        blocked = state == 'blocked'
        painter.setPen(QPen(QColor('#d22c2c'), 1 if blocked else 2))
        painter.drawLine(point, base)
        painter.setBrush(QColor('#f05a52' if not blocked else '#9b9b9b'))
        painter.setPen(QPen(QColor('#822'), 2))
        self._draw_triangle(painter, point, ship['yaw'], 13)
        label = self.ENEMY_LABELS.get(ship['name'], ship['name'])
        if blocked:
            label += ' STOP'
        painter.drawText(point + QPointF(12, -10), label)

    def _draw_own(self, painter, ship, center, scale):
        point = self._world_to_screen(ship['x'], ship['y'], center, scale)
        target = self.own_targets.get(ship['name'])
        if target:
            target_point = self._world_to_screen(
                target['x'], target['y'], center, scale
            )
            painter.setPen(QPen(QColor('#008fc7'), 2, Qt.DashLine))
            painter.drawLine(point, target_point)
            painter.setBrush(QColor('#00d7ff'))
            painter.setPen(QPen(QColor('#00748f'), 2))
            painter.drawEllipse(target_point, 5, 5)
        painter.setBrush(QColor('#1f78ff'))
        painter.setPen(QPen(QColor('#0b3c84'), 2))
        self._draw_triangle(painter, point, ship['yaw'], 14)
        painter.drawText(
            point + QPointF(12, 18),
            self.OWN_LABELS.get(ship['name'], ship['name']),
        )

    def _draw_triangle(self, painter, center, yaw, size):
        nose = QPointF(
            center.x() + math.cos(yaw) * size,
            center.y() - math.sin(yaw) * size,
        )
        left = QPointF(
            center.x() + math.cos(yaw + 2.45) * size * 0.78,
            center.y() - math.sin(yaw + 2.45) * size * 0.78,
        )
        right = QPointF(
            center.x() + math.cos(yaw - 2.45) * size * 0.78,
            center.y() - math.sin(yaw - 2.45) * size * 0.78,
        )
        painter.drawPolygon(QPolygonF([nose, left, right]))


class BaseStationWindow(QMainWindow):
    VEHICLE_NAMES = {
        'uav_01': '无人机01',
        'uav_02': '无人机02',
        'uav_03': '无人机03',
        'uav_04': '无人机04',
        'usv_01': '船01',
        'usv_02': '船02',
        'usv_03': '船03',
        'usv_04': '船04',
    }
    SENSOR_NAMES = {
        'down_camera': '下视相机',
        'front_camera': '船首相机',
        'front_lidar': '船载雷达',
        'navigation': '导航里程计',
    }
    VEHICLE_ORDER = {
        'usv_01': 0,
        'uav_01': 1,
        'usv_02': 2,
        'uav_02': 3,
        'usv_03': 4,
        'uav_03': 5,
        'usv_04': 6,
        'uav_04': 7,
    }

    def __init__(self, node, signals):
        super().__init__()
        self.node = node
        self.last_image_time = 0.0
        self.pending_image = None
        self.sensor_rows = {}
        self.vehicle_rows = {}
        self.defense_param_values = {}
        self.defense_param_sliders = {}
        self.defense_param_labels = {}
        self.pending_defense_params = {}
        self.image_timer = QTimer(self)
        self.image_timer.setInterval(33)
        self.image_timer.timeout.connect(self._flush_image)
        self.image_timer.start()
        self.defense_param_timer = QTimer(self)
        self.defense_param_timer.setSingleShot(True)
        self.defense_param_timer.timeout.connect(self._send_pending_defense_params)
        self.setWindowTitle('UAV-USV 集群基站')
        self.resize(1500, 900)
        self._build_ui()

        signals.image.connect(self._queue_image)
        signals.scan.connect(self.radar.set_scan)
        signals.sensor.connect(self._update_sensor)
        signals.vehicle.connect(self._update_vehicle)
        signals.defense.connect(self._update_defense)
        signals.defense_own.connect(self._update_defense_own)
        signals.defense_enemy.connect(self._update_defense_enemy)
        signals.log.connect(self._append_log)

    def _build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel('UAV-USV 集群基站')
        title.setObjectName('title')
        subtitle = QLabel('统一感知接入  |  任务控制  |  状态监控')
        subtitle.setObjectName('subtitle')
        header.addWidget(title)
        header.addSpacing(18)
        header.addWidget(subtitle)
        header.addStretch()
        self.link_label = QLabel('数据链路等待中')
        self.link_label.setObjectName('link')
        header.addWidget(self.link_label)
        layout.addLayout(header)

        tabs = QTabWidget()
        layout.addWidget(tabs, 1)

        overview_tab = QWidget()
        overview_layout = QVBoxLayout(overview_tab)
        overview_layout.setContentsMargins(0, 0, 0, 0)
        overview_layout.setSpacing(10)
        tabs.addTab(overview_tab, '总览')

        defense_tab = QWidget()
        defense_layout = QVBoxLayout(defense_tab)
        defense_layout.setContentsMargins(0, 0, 0, 0)
        defense_layout.setSpacing(10)
        tabs.addTab(defense_tab, '防御任务')

        perception_tab = QWidget()
        perception_layout = QVBoxLayout(perception_tab)
        perception_layout.setContentsMargins(0, 0, 0, 0)
        perception_layout.setSpacing(10)
        tabs.addTab(perception_tab, '实时感知')

        control_tab = QWidget()
        control_layout = QVBoxLayout(control_tab)
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.setSpacing(10)
        tabs.addTab(control_tab, '基站控制')

        camera_group = QGroupBox('基站接收的实时视频（四列显示）')
        camera_layout = QVBoxLayout(camera_group)
        camera_layout.setContentsMargins(0, 0, 0, 0)
        camera_layout.setSpacing(0)
        self.camera = VideoMosaicLabel('等待 UAV / USV 相机数据')
        self.camera.setMinimumSize(960, 270)
        self.camera.setStyleSheet('background: #0d141a; color: #8fa5b2;')
        camera_layout.addWidget(self.camera)
        perception_layout.addWidget(camera_group, 3)

        status_splitter = QSplitter(Qt.Horizontal)
        self.radar = RadarWidget()
        perception_layout.addWidget(self.radar, 2)

        sensor_group = QGroupBox('传感器上行状态')
        sensor_layout = QVBoxLayout(sensor_group)
        self.sensor_table = QTableWidget(0, 7)
        self.sensor_table.setHorizontalHeaderLabels(
            ['载具', '传感器', '频率', '延迟', '消息数', '数据量', '状态']
        )
        self._configure_table(self.sensor_table)
        sensor_layout.addWidget(self.sensor_table)

        vehicle_group = QGroupBox('载具状态')
        vehicle_layout = QVBoxLayout(vehicle_group)
        self.vehicle_table = QTableWidget(0, 7)
        self.vehicle_table.setHorizontalHeaderLabels(
            ['载具', '在线', '解锁', '模式', 'X / m', 'Y / m', 'Z / m']
        )
        self._configure_table(self.vehicle_table)
        vehicle_layout.addWidget(self.vehicle_table)
        status_splitter.addWidget(sensor_group)
        status_splitter.addWidget(vehicle_group)
        status_splitter.setSizes([760, 620])
        overview_layout.addWidget(status_splitter, 1)

        control_group = QGroupBox('基站控制')
        controls = QGridLayout(control_group)
        self.target_x = self._spin(-250.0, 250.0, 24.0)
        self.target_y = self._spin(-200.0, 200.0, 8.0)
        self.altitude = self._spin(4.0, 80.0, 16.0)
        controls.addWidget(QLabel('目标 X'), 0, 0)
        controls.addWidget(self.target_x, 0, 1)
        controls.addWidget(QLabel('目标 Y'), 0, 2)
        controls.addWidget(self.target_y, 0, 3)
        controls.addWidget(QLabel('UAV 高度'), 0, 4)
        controls.addWidget(self.altitude, 0, 5)

        go_button = QPushButton('协同前往')
        go_button.clicked.connect(self._send_goal)
        takeoff_button = QPushButton('无人机起飞')
        takeoff_button.clicked.connect(
            lambda: self._send_action('TAKEOFF')
        )
        hold_button = QPushButton('全部保持')
        hold_button.clicked.connect(
            lambda: self._send_action('HOLD_ALL')
        )
        stop_button = QPushButton('紧急停止')
        stop_button.setObjectName('danger')
        stop_button.clicked.connect(
            lambda: self._send_action('EMERGENCY_STOP')
        )
        for column, button in enumerate(
            (go_button, takeoff_button, hold_button, stop_button)
        ):
            controls.addWidget(button, 1, column * 2, 1, 2)
        control_layout.addWidget(control_group)

        log_group = QGroupBox('基站命令回执')
        log_layout = QVBoxLayout(log_group)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setMinimumHeight(110)
        log_layout.addWidget(self.log)
        control_layout.addWidget(log_group, 1)

        self._build_defense_tab(defense_layout)

        self.setCentralWidget(root)
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #ffffff; color: #17242c; }
            QLabel#title { font-size: 25px; font-weight: 700; }
            QLabel#subtitle { color: #526873; font-size: 14px; }
            QLabel#link {
                background: #ffffff; color: #526873;
                padding: 7px 12px; border: 1px solid #ccd6dc;
            }
            QGroupBox {
                background: #ffffff; border: 1px solid #ccd6dc;
                margin-top: 12px; padding-top: 8px; font-weight: 600;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; }
            QTableWidget {
                background: white; alternate-background-color: #eef3f5;
                gridline-color: #d1dce1; border: 0;
            }
            QHeaderView::section {
                background: #263a44; color: white; padding: 6px;
                border: 0;
            }
            QPushButton {
                background: #176b87; color: white; border: 0;
                padding: 9px 14px; font-weight: 600;
            }
            QPushButton:hover { background: #1d819f; }
            QPushButton#danger { background: #b63737; }
            QPushButton#danger:hover { background: #d04444; }
            QTabWidget::pane {
                border: 1px solid #ccd6dc; background: #ffffff;
            }
            QTabBar::tab {
                background: #eef3f6; padding: 8px 18px;
                border: 1px solid #ccd6dc; font-weight: 600;
            }
            QTabBar::tab:selected {
                background: #176b87; color: white;
            }
            QDoubleSpinBox {
                background: white; border: 1px solid #aabcc5;
                padding: 6px;
            }
            QPlainTextEdit {
                background: #101820; color: #c9d8df;
                border: 0; font-family: monospace;
            }
            """
        )

    def _build_defense_tab(self, layout):
        self.defense_map = DefenseMapWidget()
        layout.addWidget(self.defense_map, 1)

        tuning_group = QGroupBox('防御参数实时调整')
        tuning_layout = QGridLayout(tuning_group)
        tuning_layout.setHorizontalSpacing(10)
        tuning_layout.setVerticalSpacing(6)
        sliders = [
            ('defend_radius', '防守半径', 40.0, 140.0, 75.0, 1.0, ' m'),
            ('trigger_radius', '预警半径', 100.0, 320.0, 190.0, 1.0, ' m'),
            ('own_guard_speed', '我方防守速度', 4.0, 30.0, 15.0, 0.5, ' m/s'),
            ('enemy_speed', '敌方进攻速度', 1.0, 14.0, 4.5, 0.1, ' m/s'),
            ('guard_stop_distance', '我方到点阈值', 8.0, 50.0, 20.0, 1.0, ' m'),
            (
                'enemy_guard_stop_distance',
                '敌方拦停距离',
                5.0,
                45.0,
                22.0,
                1.0,
                ' m',
            ),
            (
                'intercept_stop_distance',
                '近距离拦截距离',
                6.0,
                50.0,
                18.0,
                1.0,
                ' m',
            ),
            ('guard_spacing', '防守点间距', 12.0, 70.0, 28.0, 1.0, ' m'),
        ]
        for index, config in enumerate(sliders):
            self._add_defense_slider(tuning_layout, index, *config)
        layout.addWidget(tuning_group)

        summary_group = QGroupBox('防御任务状态')
        summary_layout = QGridLayout(summary_group)
        labels = [
            ('当前模式', 'defense_mode'),
            ('威胁数量', 'defense_threats'),
            ('已拦截', 'defense_blocked'),
            ('防守半径', 'defense_radius'),
            ('预警半径', 'warning_radius'),
            ('大本营安全半径', 'base_safety_radius'),
        ]
        self.defense_labels = {}
        for index, (title, key) in enumerate(labels):
            row = index // 3
            column = (index % 3) * 2
            summary_layout.addWidget(QLabel(title), row, column)
            value = QLabel('等待数据')
            value.setObjectName('defenseValue')
            value.setAlignment(Qt.AlignCenter)
            summary_layout.addWidget(value, row, column + 1)
            self.defense_labels[key] = value
        layout.addWidget(summary_group)

        hint_group = QGroupBox('显示说明')
        hint_layout = QVBoxLayout(hint_group)
        hint = QLabel(
            '蓝色为我方守卫船，红色为敌方船，绿色为大本营。'
            '橙色圈是预警范围，蓝色圈是防守位置半径，红色圈是大本营安全范围。'
            '虚线表示我方船当前要去的防守点。'
        )
        hint.setWordWrap(True)
        hint_layout.addWidget(hint)
        layout.addWidget(hint_group)

    def _add_defense_slider(
        self,
        layout,
        row,
        name,
        title,
        minimum,
        maximum,
        value,
        step,
        suffix,
    ):
        scale = int(round(1.0 / step))
        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(round(minimum * scale)), int(round(maximum * scale)))
        slider.setSingleStep(1)
        slider.setPageStep(max(1, int(round(5.0 * scale))))
        slider.setValue(int(round(value * scale)))
        value_label = QLabel()
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.defense_param_values[name] = float(value)
        self.defense_param_sliders[name] = (slider, scale)
        self.defense_param_labels[name] = (value_label, suffix, step)
        self._set_defense_slider_label(name, value)
        slider.valueChanged.connect(
            lambda raw, param=name, factor=scale: self._on_defense_slider(
                param,
                raw / float(factor),
            )
        )
        layout.addWidget(QLabel(title), row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(value_label, row, 2)

    def _set_defense_slider_label(self, name, value):
        label, suffix, step = self.defense_param_labels[name]
        decimals = 1 if step < 1.0 else 0
        label.setText(('%.*f' % (decimals, value)) + suffix)

    def _on_defense_slider(self, name, value):
        self.defense_param_values[name] = float(value)
        self.pending_defense_params[name] = float(value)
        self._set_defense_slider_label(name, value)
        self.defense_param_timer.start(120)

    def _send_pending_defense_params(self):
        values = dict(self.pending_defense_params)
        self.pending_defense_params.clear()
        if values:
            self.node.set_defense_parameters(values)

    @staticmethod
    def _configure_table(table):
        table.setAlternatingRowColors(True)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionMode(QTableWidget.NoSelection)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    @staticmethod
    def _ordered_insert_row(table, row_map, key, preferred_order):
        if key in row_map:
            return row_map[key]
        order_value = preferred_order.get(key, 1000 + len(row_map))
        row = 0
        for existing_key, existing_row in sorted(
            row_map.items(), key=lambda item: item[1]
        ):
            existing_order = preferred_order.get(
                existing_key, 1000 + existing_row
            )
            if existing_order > order_value:
                break
            row += 1
        table.insertRow(row)
        for existing_key, existing_row in list(row_map.items()):
            if existing_row >= row:
                row_map[existing_key] = existing_row + 1
        row_map[key] = row
        return row

    def _sensor_row(self, vehicle, sensor):
        sensor_order = {
            'front_camera': 0,
            'down_camera': 1,
            'front_lidar': 2,
            'navigation': 3,
        }
        preferred_order = {}
        for vehicle_id, vehicle_order in self.VEHICLE_ORDER.items():
            for sensor_id, order in sensor_order.items():
                preferred_order[(vehicle_id, sensor_id)] = (
                    vehicle_order * 10 + order
                )
        return self._ordered_insert_row(
            self.sensor_table,
            self.sensor_rows,
            (vehicle, sensor),
            preferred_order,
        )

    def _vehicle_row(self, vehicle):
        return self._ordered_insert_row(
            self.vehicle_table,
            self.vehicle_rows,
            vehicle,
            self.VEHICLE_ORDER,
        )

    @staticmethod
    def _spin(minimum, maximum, value):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(1)
        spin.setSingleStep(1.0)
        spin.setValue(value)
        spin.setSuffix(' m')
        return spin

    @staticmethod
    def _item(text, color=None):
        item = QTableWidgetItem(str(text))
        item.setTextAlignment(Qt.AlignCenter)
        if color is not None:
            item.setForeground(QColor(color))
            font = QFont(item.font())
            font.setBold(True)
            item.setFont(font)
        return item

    def _queue_image(self, image):
        self.pending_image = image

    def _flush_image(self):
        if self.pending_image is None:
            return
        image = self.pending_image
        self.pending_image = None
        self._update_image(image)

    def _update_image(self, image):
        self.last_image_time = time.monotonic()
        self.camera.set_image(image)
        self.link_label.setText('基站数据链路在线')
        self.link_label.setStyleSheet(
            'background: #d9f0e5; color: #176b47; '
            'padding: 7px 12px; border: 1px solid #8cc5a8;'
        )

    def _update_sensor(self, data):
        vehicle, sensor, rate, age, messages, total_bytes, healthy = data
        row = self._sensor_row(vehicle, sensor)
        values = [
            self.VEHICLE_NAMES.get(vehicle, vehicle),
            self.SENSOR_NAMES.get(sensor, sensor),
            '%.1f Hz' % rate,
            '%.3f s' % age,
            str(messages),
            '%.1f MB' % (total_bytes / 1048576.0),
        ]
        for column, value in enumerate(values):
            self.sensor_table.setItem(row, column, self._item(value))
        self.sensor_table.setItem(
            row,
            6,
            self._item('正常' if healthy else '中断',
                       '#16834a' if healthy else '#b63737'),
        )

    def _update_vehicle(self, data):
        vehicle, online, armed, mode, x, y, z, status = data
        row = self._vehicle_row(vehicle)
        values = [
            self.VEHICLE_NAMES.get(vehicle, vehicle),
            '在线' if online else '离线',
            '是' if armed else '否',
            mode,
            '%.2f' % x,
            '%.2f' % y,
            '%.2f' % z,
        ]
        for column, value in enumerate(values):
            color = None
            if column == 1:
                color = '#16834a' if online else '#b63737'
            self.vehicle_table.setItem(
                row, column, self._item(value, color)
            )
        self.vehicle_table.setToolTip('%s: %s' % (vehicle, status))

    def _update_defense(self, fields):
        mode_names = {
            'patrol': '巡逻',
            'guard': '防守',
        }
        mode = fields.get('mode', '-')
        values = {
            'defense_mode': mode_names.get(mode, mode),
            'defense_threats': fields.get('threats', '-'),
            'defense_blocked': fields.get('blocked', '-'),
            'defense_radius': fields.get('defend_radius', '-') + ' m',
            'warning_radius': fields.get('trigger_radius', '-') + ' m',
            'base_safety_radius': fields.get('base_safety_radius', '-') + ' m',
        }
        for key, value in values.items():
            self.defense_labels[key].setText(value)
        self._sync_defense_sliders(fields)
        self.defense_map.set_status(fields)
        if mode == 'guard':
            self.defense_labels['defense_mode'].setStyleSheet(
                'background: #ffe7d6; color: #9b3f00; padding: 8px;'
            )
        else:
            self.defense_labels['defense_mode'].setStyleSheet(
                'background: #d9f0e5; color: #176b47; padding: 8px;'
            )

    def _sync_defense_sliders(self, fields):
        if self.pending_defense_params:
            return
        for name, value_text in fields.items():
            if name not in self.defense_param_sliders:
                continue
            try:
                value = float(value_text)
            except ValueError:
                continue
            slider, scale = self.defense_param_sliders[name]
            raw = int(round(value * scale))
            raw = max(slider.minimum(), min(slider.maximum(), raw))
            slider.blockSignals(True)
            slider.setValue(raw)
            slider.blockSignals(False)
            self.defense_param_values[name] = raw / float(scale)
            self._set_defense_slider_label(name, raw / float(scale))

    def _update_defense_own(self, ships):
        self.defense_map.set_own_ships(ships)

    def _update_defense_enemy(self, ships):
        self.defense_map.set_enemy_ships(ships)

    def _append_log(self, text):
        timestamp = time.strftime('%H:%M:%S')
        self.log.appendPlainText('[%s] %s' % (timestamp, text))

    def _send_goal(self):
        self.node.publish_goal(
            self.target_x.value(),
            self.target_y.value(),
            self.altitude.value(),
        )
        self._append_log(
            '操作员下发协同目标 (%.1f, %.1f), UAV 高度 %.1f m'
            % (
                self.target_x.value(),
                self.target_y.value(),
                self.altitude.value(),
            )
        )

    def _send_action(self, action):
        self.node.publish_action(action)
        self._append_log('操作员下发动作: %s' % action)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setApplicationName('UAV-USV Fleet Base Station')
    signals = GuiSignals()
    node = BaseStationGuiNode(signals)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, daemon=True
    )
    spin_thread.start()

    window = BaseStationWindow(node, signals)
    window.show()
    result = app.exec_()

    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    sys.exit(result)


if __name__ == '__main__':
    main()
