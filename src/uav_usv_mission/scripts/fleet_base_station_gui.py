#!/usr/bin/env python3
import math
import signal
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
from PyQt5.QtWidgets import QAbstractItemView
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
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import CaptureAssignmentArray
from uav_usv_interfaces.msg import CaptureState
from uav_usv_interfaces.msg import CaptureTargetStatus
from uav_usv_interfaces.msg import SensorStatus
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class GuiSignals(QObject):
    image = pyqtSignal(object)
    scan = pyqtSignal(object)
    sensor = pyqtSignal(object)
    vehicle = pyqtSignal(object)
    defense = pyqtSignal(object)
    defense_own = pyqtSignal(object)
    defense_enemy = pyqtSignal(object)
    capture_targets = pyqtSignal(object)
    capture_status = pyqtSignal(object)
    capture_state = pyqtSignal(object)
    capture_roles = pyqtSignal(object)
    capture_target = pyqtSignal(object)
    capture_markers = pyqtSignal(object)
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
        # The ROS callback already detached this QImage from message memory.
        # Sharing the immutable frame avoids two full mosaic copies per refresh.
        self._image = image
        self.update()

    def paintEvent(self, event):
        if self._image is None:
            super().paintEvent(event)
            return
        painter = QPainter(self)
        painter.drawImage(self.rect(), self._image)


class BaseStationGuiNode(Node):
    VEHICLE_NAMES = {
        'uav_01': '主感知无人机01（PX4）',
        'uav_02': '无人机02（PX4）',
        'uav_03': '无人机03（PX4）',
        'uav_04': '无人机04（PX4）',
        'usv_01': '船01',
        'usv_02': '船02',
        'usv_03': '船03',
        'usv_04': '船04',
    }

    def __init__(self, signals):
        super().__init__('fleet_base_station_gui')
        self.signals = signals
        self.declare_parameter('capture_namespace', '')
        self.declare_parameter('defense_namespace', '')
        self.declare_parameter('defense_node_name', '/defense_sim_demo')
        self.declare_parameter('demo_mode', False)
        self.declare_parameter(
            'mid360_preview_service',
            '/perception/usv_01/mid360/set_visualization',
        )
        self.demo_mode = bool(self.get_parameter('demo_mode').value)
        self.capture_namespace = str(
            self.get_parameter('capture_namespace').value
        ).strip('/')
        self.defense_namespace = str(
            self.get_parameter('defense_namespace').value
        ).strip('/')

        def topic(namespace, name):
            if not namespace:
                return name
            return '/%s%s' % (namespace, name)

        self._topic = topic

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        state_qos = QoSProfile(depth=5)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        state_qos.durability = DurabilityPolicy.VOLATILE

        namespaces = []
        for namespace in (
            self.capture_namespace,
            self.defense_namespace,
        ):
            if namespace not in namespaces:
                namespaces.append(namespace)
        for namespace in namespaces:
            self.create_subscription(
                Image,
                self._topic(namespace, '/fleet/base/camera_mosaic'),
                lambda msg, source=namespace: self._on_image(msg, source),
                image_qos,
            )
            self.create_subscription(
                LaserScan,
                self._topic(namespace, '/fleet/base/radar/scan'),
                self._on_scan,
                image_qos,
            )
            self.create_subscription(
                SensorStatus,
                self._topic(namespace, '/fleet/sensor_status'),
                self._on_sensor,
                20,
            )
            self.create_subscription(
                VehicleState,
                self._topic(namespace, '/fleet/state'),
                self._on_vehicle,
                state_qos,
            )
            self.create_subscription(
                CommandAck,
                self._topic(namespace, '/fleet/command_ack'),
                self._on_ack,
                20,
            )
        self.create_subscription(
            TrackedObjectArray,
            self._topic(self.capture_namespace, '/fleet/perception/targets'),
            self._on_capture_targets,
            10,
        )
        self.create_subscription(
            String,
            self._topic(self.capture_namespace, '/fleet/capture/status'),
            self._on_capture_status,
            10,
        )
        self.create_subscription(
            CaptureState,
            self._topic(self.capture_namespace, '/capture/state'),
            self._on_capture_state,
            10,
        )
        self.create_subscription(
            CaptureAssignmentArray,
            self._topic(self.capture_namespace, '/capture/roles'),
            self._on_capture_roles,
            10,
        )
        self.create_subscription(
            CaptureTargetStatus,
            self._topic(self.capture_namespace, '/capture/target_status'),
            self._on_capture_target,
            10,
        )
        self.create_subscription(
            MarkerArray,
            self._topic(self.capture_namespace, '/capture/markers'),
            self._on_capture_markers,
            10,
        )
        self.create_subscription(
            String,
            self._topic(self.defense_namespace, '/defense/status'),
            self._on_defense_status,
            10,
        )
        self.create_subscription(
            PoseArray,
            self._topic(self.defense_namespace, '/defense/own_ships'),
            self._on_defense_own,
            10,
        )
        self.create_subscription(
            PoseArray,
            self._topic(self.defense_namespace, '/defense/enemy_ships'),
            self._on_defense_enemy,
            10,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self._topic(self.capture_namespace, '/fleet/base/operator_goal'),
            10,
        )
        self.capture_action_pub = self.create_publisher(
            String,
            self._topic(
                self.capture_namespace, '/fleet/base/operator_action'
            ),
            10,
        )
        self.defense_action_pub = self.create_publisher(
            String,
            self._topic(
                self.defense_namespace, '/fleet/base/operator_action'
            ),
            10,
        )
        defense_node_name = str(self.get_parameter('defense_node_name').value)
        defense_node_name = '/' + defense_node_name.strip('/')
        self.defense_param_client = self.create_client(
            SetParameters,
            '%s/%s/set_parameters'
            % (self._topic(self.defense_namespace, ''),
               defense_node_name.strip('/')),
        )
        self.mid360_preview_client = self.create_client(
            SetBool,
            str(self.get_parameter('mid360_preview_service').value),
        )

    def _on_image(self, msg, source=''):
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
        self.signals.image.emit((source or 'default', image))

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
                msg.frame_id,
                msg.measured_rate_hz,
                msg.age_seconds,
                msg.latency_seconds,
                msg.processing_time_ms,
                msg.point_count,
                msg.total_messages,
                msg.total_bytes,
                msg.dropped_messages,
                msg.healthy,
                msg.timed_out,
                msg.last_message_time.sec,
                msg.last_message_time.nanosec,
            )
        )

    def set_mid360_preview(self, enabled):
        if not self.mid360_preview_client.service_is_ready():
            self.signals.log.emit('Mid-360预览服务暂不可用')
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.mid360_preview_client.call_async(request)

        def finished(result_future):
            try:
                result = result_future.result()
                self.signals.log.emit(result.message)
            except Exception as exc:
                self.signals.log.emit('Mid-360预览切换失败: %s' % exc)

        future.add_done_callback(finished)

    def _on_vehicle(self, msg):
        self.signals.vehicle.emit(
            (
                msg.vehicle_id,
                msg.vehicle_type,
                msg.online,
                msg.armed,
                msg.mode,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.status_text,
            )
        )

    def _on_ack(self, msg):
        if msg.status in (
            CommandAck.STATUS_RECEIVED,
            CommandAck.STATUS_ACCEPTED,
            CommandAck.STATUS_EXECUTING,
        ):
            return
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

    def _on_capture_targets(self, msg):
        targets = []
        for obj in msg.objects:
            targets.append(
                {
                    'track_id': obj.track_id,
                    'class': obj.classification,
                    'source': obj.source_mask,
                    'confidence': obj.confidence,
                    'x': obj.pose.pose.position.x,
                    'y': obj.pose.pose.position.y,
                    'z': obj.pose.pose.position.z,
                }
            )
        self.signals.capture_targets.emit(targets)

    def _on_capture_status(self, msg):
        fields = {}
        for token in msg.data.split():
            if '=' not in token:
                continue
            key, value = token.split('=', 1)
            fields[key] = value
        self.signals.capture_status.emit(fields)

    def _on_capture_state(self, msg):
        self.signals.capture_state.emit({
            'state': int(msg.state),
            'state_name': msg.state_name,
            'target_id': msg.target_id,
            'reason': msg.reason,
            'configured_uavs': int(msg.configured_uavs),
            'configured_usvs': int(msg.configured_usvs),
            'active_uavs': int(msg.active_uavs),
            'active_usvs': int(msg.active_usvs),
            'generation': int(msg.allocation_generation),
            'degraded': bool(msg.degraded),
        })

    def _on_capture_roles(self, msg):
        assignments = []
        for item in msg.assignments:
            assignments.append({
                'vehicle_id': item.vehicle_id,
                'vehicle_type': int(item.vehicle_type),
                'role_type': int(item.role_type),
                'role_name': item.role_name,
                'x': float(item.target_pose.position.x),
                'y': float(item.target_pose.position.y),
                'z': float(item.target_pose.position.z),
                'cost': float(item.assignment_cost),
                'active': bool(item.active),
                'status': item.status,
            })
        self.signals.capture_roles.emit({
            'target_id': msg.target_id,
            'center_x': float(msg.capture_center.x),
            'center_y': float(msg.capture_center.y),
            'radius': float(msg.capture_radius),
            'generation': int(msg.generation),
            'assignments': assignments,
        })

    def _on_capture_target(self, msg):
        self.signals.capture_target.emit({
            'track_id': msg.track_id,
            'tracked': bool(msg.tracked),
            'confirmations': int(msg.confirmations),
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'z': float(msg.pose.position.z),
            'vx': float(msg.twist.linear.x),
            'vy': float(msg.twist.linear.y),
            'speed': float(msg.speed_mps),
            'turn_rate': float(msg.turn_rate_rps),
            'age': float(msg.track_age_s),
            'model': msg.prediction_model,
        })

    def _on_capture_markers(self, msg):
        prediction = []
        for marker in msg.markers:
            if marker.action != Marker.ADD or marker.ns != 'prediction':
                continue
            if marker.type == Marker.LINE_STRIP:
                prediction = [
                    (float(point.x), float(point.y), float(point.z))
                    for point in marker.points
                ]
                break
        self.signals.capture_markers.emit({'prediction': prediction})

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
        self.capture_action_pub.publish(msg)

    def publish_defense_action(self, action):
        msg = String()
        msg.data = action
        self.defense_action_pub.publish(msg)

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
        painter.drawText(info_x, 26, '基地雷达海域态势视图')
        painter.setPen(QColor('#94aebc'))
        painter.drawText(info_x, 54, '中心黄点：基地雷达位置')
        painter.drawText(info_x, 78, '黄色短线：雷达零度方向')
        painter.setPen(QColor('#33e6c4'))
        painter.drawText(info_x, 102, '青色回波：雷达扫到的物体')
        painter.setPen(QColor('#94aebc'))
        painter.drawText(info_x, 126, '圆圈刻度：距离基地的半径')
        painter.drawText(info_x, 158, '量程：%.1f m' % self.range_max)
        painter.drawText(info_x, 182, '点数：%d' % len(self.points))
        painter.setPen(QPen(QColor('#35505f'), 1))
        painter.drawLine(info_x, 200, self.width() - 18, 200)
        painter.setPen(QColor('#b7c9d3'))
        painter.drawText(info_x, 226, '用途：全局监视与船队避障')
        painter.drawText(info_x, 250, '显示基地周围实时雷达回波')


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
        self.own_history = {}
        self.enemy_history = {}
        self.capture_ring_radius = 125.0
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
        self.capture_ring_radius = self._float(
            fields.get('capture_ring_radius'), self.capture_ring_radius
        )
        self.update()

    def set_own_ships(self, ships):
        self.own_ships = ships
        self._append_history(self.own_history, ships)
        self.update()

    def set_enemy_ships(self, ships):
        self.enemy_ships = ships
        self._append_history(self.enemy_history, ships)
        self.update()

    @staticmethod
    def _append_history(history, ships):
        for ship in ships:
            points = history.setdefault(ship['name'], [])
            point = (ship['x'], ship['y'])
            if not points or math.hypot(
                point[0] - points[-1][0], point[1] - points[-1][1]
            ) >= 2.0:
                points.append(point)
                del points[:-100]

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

        self._draw_histories(painter, center, scale)
        retreat = next((
            ship for ship in self.enemy_ships
            if self.enemy_states.get(ship['name'], {}).get('state')
            in ('retreating', 'captured')
        ), None)
        if retreat is not None:
            capture_center = self._world_to_screen(
                retreat['x'], retreat['y'], center, scale
            )
            self._draw_circle(
                painter, capture_center,
                self.capture_ring_radius * scale,
                QColor(255, 40, 40, 20), QColor('#e02020'), 2,
                '动态围捕圈 %.0fm' % self.capture_ring_radius,
            )

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

    def _draw_histories(self, painter, center, scale):
        for history, color in (
            (self.own_history, QColor(15, 132, 220, 145)),
            (self.enemy_history, QColor(220, 55, 45, 125)),
        ):
            painter.setPen(QPen(color, 1.5))
            for points in history.values():
                if len(points) < 2:
                    continue
                polygon = QPolygonF([
                    self._world_to_screen(x, y, center, scale)
                    for x, y in points
                ])
                painter.drawPolyline(polygon)
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


class CaptureMapWidget(DefenseMapWidget):
    """Compact 2D view fed only by the existing capture interfaces."""

    def __init__(self):
        super().__init__()
        self.vehicles = {}
        self.assignments = {}
        self.target = None
        self.prediction = []
        self.capture_state = 'SEARCH'
        self.capture_radius = 28.0
        self.capture_center = (0.0, 0.0)
        self.vehicle_history = {}
        self.target_history = []
        self.setMinimumSize(500, 380)

    def set_vehicle(self, state):
        vehicle_id = state['vehicle_id']
        self.vehicles[vehicle_id] = state
        history = self.vehicle_history.setdefault(vehicle_id, [])
        point = (state['x'], state['y'])
        if not history or math.hypot(
            point[0] - history[-1][0], point[1] - history[-1][1]
        ) >= 0.8:
            history.append(point)
            del history[:-80]
        self.update()

    def set_capture_state(self, state):
        self.capture_state = state.get('state_name', 'SEARCH')
        self.update()

    def set_roles(self, roles):
        self.capture_center = (
            roles.get('center_x', 0.0), roles.get('center_y', 0.0)
        )
        self.capture_radius = max(1.0, roles.get('radius', 28.0))
        self.assignments = {
            item['vehicle_id']: item for item in roles.get('assignments', [])
        }
        self.update()

    def set_target(self, target):
        self.target = target
        point = (target.get('x', 0.0), target.get('y', 0.0))
        if not self.target_history or math.hypot(
            point[0] - self.target_history[-1][0],
            point[1] - self.target_history[-1][1],
        ) >= 0.5:
            self.target_history.append(point)
            del self.target_history[:-100]
        self.update()

    def set_markers(self, data):
        self.prediction = data.get('prediction', [])
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#f7fafc'))

        points = [(item['x'], item['y']) for item in self.vehicles.values()]
        if self.target is not None:
            points.append((self.target['x'], self.target['y']))
        points.extend((item['x'], item['y']) for item in self.assignments.values())
        points.extend((item[0], item[1]) for item in self.prediction)
        center_x, center_y = self.capture_center
        if self.target is not None:
            center_x = self.target['x']
            center_y = self.target['y']
        elif points:
            center_x = sum(point[0] for point in points) / len(points)
            center_y = sum(point[1] for point in points) / len(points)
        self.base_x, self.base_y = center_x, center_y
        visible = [
            math.hypot(x - center_x, y - center_y) for x, y in points
        ]
        world_radius = max(45.0, self.capture_radius * 1.8,
                           max(visible, default=0.0) + 18.0)
        center = QPointF(self.width() * 0.5, self.height() * 0.52)
        scale = max(
            0.1,
            (min(self.width(), self.height()) / 2.0 - 38.0) / world_radius,
        )
        self._draw_grid(painter, center, scale, world_radius)

        capture_center = self._world_to_screen(
            self.capture_center[0], self.capture_center[1], center, scale
        )
        self._draw_circle(
            painter, capture_center, self.capture_radius * scale,
            QColor(116, 74, 190, 18), QColor('#7049b8'), 2,
            '围捕半径 %.0fm' % self.capture_radius,
        )
        self._draw_capture_history(painter, self.target_history,
                                   QColor('#df3434'), center, scale)
        for vehicle_id, history in self.vehicle_history.items():
            color = QColor('#2580d8') if vehicle_id.startswith('uav_') \
                else QColor('#d19a00')
            self._draw_capture_history(painter, history, color, center, scale)

        if len(self.prediction) > 1:
            painter.setPen(QPen(QColor('#f06d00'), 2, Qt.DashLine))
            painter.drawPolyline(QPolygonF([
                self._world_to_screen(x, y, center, scale)
                for x, y, _z in self.prediction
            ]))

        if self.target is not None:
            point = self._world_to_screen(
                self.target['x'], self.target['y'], center, scale
            )
            painter.setBrush(QColor('#e13b36'))
            painter.setPen(QPen(QColor('#8d1714'), 2))
            painter.drawEllipse(point, 9, 9)
            painter.drawText(point + QPointF(12, -10),
                             self.target.get('track_id', 'enemy_target'))

        for vehicle_id, state in sorted(self.vehicles.items()):
            point = self._world_to_screen(state['x'], state['y'], center, scale)
            assignment = self.assignments.get(vehicle_id)
            if assignment and assignment.get('active'):
                goal = self._world_to_screen(
                    assignment['x'], assignment['y'], center, scale
                )
                painter.setPen(QPen(QColor('#73828a'), 1, Qt.DashLine))
                painter.drawLine(point, goal)
                painter.setBrush(QColor('#ffffff'))
                painter.setPen(QPen(QColor('#7049b8'), 2))
                painter.drawEllipse(goal, 5, 5)
            is_uav = vehicle_id.startswith('uav_')
            painter.setBrush(QColor('#2580d8' if is_uav else '#f2b51d'))
            painter.setPen(QPen(QColor('#124c7f' if is_uav else '#8b6500'), 2))
            yaw = math.atan2(state['vy'], state['vx']) if math.hypot(
                state['vx'], state['vy']
            ) > 0.05 else 0.0
            self._draw_triangle(painter, point, yaw, 11 if is_uav else 13)
            role = assignment.get('role_name', 'Standby') if assignment else 'Standby'
            painter.drawText(point + QPointF(10, 16),
                             '%s | %s' % (vehicle_id.upper(), role))

        painter.setPen(QPen(QColor('#283943'), 1))
        painter.drawText(16, 24, 'DYNAMIC CAPTURE | %s | vehicles=%d' % (
            self.capture_state, len(self.vehicles)
        ))

    def _draw_capture_history(self, painter, history, color, center, scale):
        if len(history) < 2:
            return
        painter.setPen(QPen(color, 1.4))
        painter.drawPolyline(QPolygonF([
            self._world_to_screen(x, y, center, scale) for x, y in history
        ]))


class BaseStationWindow(QMainWindow):
    VEHICLE_NAMES = {
        'uav_01': '主感知无人机01（PX4）',
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
        'mid360': 'Mid-360点云',
        'base_radar': '基地雷达',
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
        self.pending_images = {}
        self.sensor_rows = {}
        self.vehicle_rows = {}
        self.fleet_rows = {}
        self.vehicle_cache = {}
        self.capture_roles_cache = {}
        self.capture_state_cache = {}
        self.capture_target_cache = {}
        self.last_ros_message_time = 0.0
        self.defense_param_values = {}
        self.defense_param_sliders = {}
        self.defense_param_labels = {}
        self.pending_defense_params = {}
        self.image_timer = QTimer(self)
        self.image_timer.setInterval(33)
        self.image_timer.timeout.connect(self._flush_image)
        self.image_timer.start()
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(500)
        self.status_timer.timeout.connect(self._refresh_connection_status)
        self.status_timer.start()
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
        signals.capture_targets.connect(self._update_capture_targets)
        signals.capture_status.connect(self._update_capture_status)
        signals.capture_state.connect(self._update_capture_state)
        signals.capture_roles.connect(self._update_capture_roles)
        signals.capture_target.connect(self._update_capture_target)
        signals.capture_markers.connect(self._update_capture_markers)
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
        if self.node.demo_mode:
            demo_label = QLabel('DEMO MODE')
            demo_label.setObjectName('demoMode')
            header.addSpacing(18)
            header.addWidget(demo_label)
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

        capture_tab = QWidget()
        capture_layout = QVBoxLayout(capture_tab)
        capture_layout.setContentsMargins(0, 0, 0, 0)
        capture_layout.setSpacing(10)
        tabs.addTab(capture_tab, 'Dynamic Capture')

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

        status_bar = QHBoxLayout()
        self.system_status_label = self._status_card('SYSTEM', 'WAITING')
        self.uav_count_label = self._status_card('UAV', '0 / 4')
        self.usv_count_label = self._status_card('USV', '0 / 2')
        self.mission_state_label = self._status_card('MISSION', 'SEARCH')
        self.target_state_label = self._status_card('TARGET', 'WAITING')
        for card in (
            self.system_status_label,
            self.uav_count_label,
            self.usv_count_label,
            self.mission_state_label,
            self.target_state_label,
        ):
            status_bar.addWidget(card, 1)
        overview_layout.addLayout(status_bar)

        overview_splitter = QSplitter(Qt.Horizontal)
        fleet_group = QGroupBox('舰队列表')
        fleet_layout = QVBoxLayout(fleet_group)
        self.fleet_table = QTableWidget(0, 4)
        self.fleet_table.setHorizontalHeaderLabels(
            ['载具', '角色', '链路', '控制模式']
        )
        self._configure_table(self.fleet_table)
        self.fleet_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.fleet_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.fleet_table.itemSelectionChanged.connect(
            self._show_selected_vehicle
        )
        self.vehicle_detail = QLabel('点击载具查看详情')
        self.vehicle_detail.setWordWrap(True)
        self.vehicle_detail.setObjectName('vehicleDetail')
        fleet_layout.addWidget(self.fleet_table, 1)
        fleet_layout.addWidget(self.vehicle_detail)
        overview_splitter.addWidget(fleet_group)

        map_group = QGroupBox('动态围捕态势')
        map_layout = QVBoxLayout(map_group)
        self.capture_overview_map = CaptureMapWidget()
        map_layout.addWidget(self.capture_overview_map)
        overview_splitter.addWidget(map_group)

        mission_group = QGroupBox('任务控制')
        mission_layout = QVBoxLayout(mission_group)
        self.demo_summary = QLabel(
            'MISSION\nDYNAMIC CAPTURE\n\nSTATE\nSEARCH\n\nACTIVE\n0 VEHICLES'
        )
        self.demo_summary.setObjectName('demoSummary')
        self.demo_summary.setAlignment(Qt.AlignCenter)
        mission_layout.addWidget(self.demo_summary)
        for label, action, danger in (
            ('启动围捕', 'CAPTURE:enemy_target', False),
            ('暂停任务', 'HOLD_ALL', False),
            ('继续任务', 'CAPTURE:enemy_target', False),
            ('停止任务', 'CANCEL_CAPTURE', True),
            ('复位显示', 'RESET_VIEW', False),
        ):
            button = QPushButton(label)
            if danger:
                button.setObjectName('danger')
            if action == 'RESET_VIEW':
                button.clicked.connect(self._reset_capture_view)
            else:
                button.clicked.connect(
                    lambda _checked=False, command=action: self._send_action(command)
                )
            mission_layout.addWidget(button)
        mission_layout.addStretch()
        overview_splitter.addWidget(mission_group)
        overview_splitter.setSizes([330, 850, 260])
        overview_layout.addWidget(overview_splitter, 1)

        event_group = QGroupBox('事件日志')
        event_layout = QVBoxLayout(event_group)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        self.log.setMaximumHeight(150)
        event_layout.addWidget(self.log)
        overview_layout.addWidget(event_group)

        status_splitter = QSplitter(Qt.Horizontal)
        self.radar = RadarWidget()
        preview_controls = QHBoxLayout()
        preview_controls.addWidget(QLabel('Mid-360轻量预览'))
        self.mid360_preview_button = QPushButton('RViz点云预览：开启')
        self.mid360_preview_button.setCheckable(True)
        self.mid360_preview_button.setChecked(True)
        self.mid360_preview_button.toggled.connect(
            self._toggle_mid360_preview
        )
        preview_controls.addWidget(self.mid360_preview_button)
        preview_controls.addStretch()
        perception_layout.addLayout(preview_controls)
        perception_layout.addWidget(self.radar, 2)

        sensor_group = QGroupBox('传感器上行状态')
        sensor_layout = QVBoxLayout(sensor_group)
        self.sensor_table = QTableWidget(0, 10)
        self.sensor_table.setHorizontalHeaderLabels(
            [
                '载具', '传感器', 'Frame', '频率', '延迟',
                '点数', '丢帧', '处理', '数据量', '状态',
            ]
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
        perception_layout.addWidget(status_splitter, 1)

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

        control_layout.addStretch()

        self._build_defense_tab(defense_layout)
        self._build_capture_tab(capture_layout)

        self.setCentralWidget(root)
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #ffffff; color: #17242c; }
            QLabel#title { font-size: 25px; font-weight: 700; }
            QLabel#subtitle { color: #526873; font-size: 14px; }
            QLabel#demoMode {
                background: #173d55; color: white; padding: 7px 12px;
                font-weight: 700;
            }
            QLabel#statusCard {
                background: #eef5f8; border: 1px solid #c4d5dd;
                padding: 9px; font-weight: 700;
            }
            QLabel#demoSummary {
                background: #102b3a; color: #e9f7fc; padding: 14px;
                font-size: 15px; font-weight: 700;
            }
            QLabel#vehicleDetail {
                background: #f3f7f9; border: 1px solid #d2dde2;
                padding: 8px;
            }
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
        self._add_camera_group(
            layout, '防御任务实时感知画面', 'defense_camera'
        )
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

    def _build_capture_tab(self, layout):
        self._add_camera_group(
            layout, 'Dynamic Capture 实时传感器画面', 'capture_camera'
        )

        capture_splitter = QSplitter(Qt.Horizontal)
        map_group = QGroupBox('围捕态势、预测轨迹与分配连线')
        map_layout = QVBoxLayout(map_group)
        self.capture_detail_map = CaptureMapWidget()
        map_layout.addWidget(self.capture_detail_map)
        capture_splitter.addWidget(map_group)

        details = QWidget()
        details_layout = QVBoxLayout(details)
        target_group = QGroupBox('目标信息')
        target_layout = QGridLayout(target_group)
        self.capture_target_labels = {}
        for row, (title, key) in enumerate((
            ('Target', 'target'),
            ('Position', 'position'),
            ('Velocity', 'velocity'),
            ('Prediction', 'prediction'),
            ('Tracking', 'tracking'),
        )):
            target_layout.addWidget(QLabel(title), row, 0)
            value = QLabel('等待数据')
            value.setObjectName('captureValue')
            target_layout.addWidget(value, row, 1)
            self.capture_target_labels[key] = value
        details_layout.addWidget(target_group)

        assignment_group = QGroupBox('任务分配')
        assignment_layout = QVBoxLayout(assignment_group)
        self.assignment_table = QTableWidget(0, 5)
        self.assignment_table.setHorizontalHeaderLabels(
            ['载具', '角色', '任务点', '状态', '代价']
        )
        self._configure_table(self.assignment_table)
        assignment_layout.addWidget(self.assignment_table)
        details_layout.addWidget(assignment_group, 1)
        capture_splitter.addWidget(details)
        capture_splitter.setSizes([850, 520])
        layout.addWidget(capture_splitter, 1)

        targets_group = QGroupBox('感知目标列表')
        targets_layout = QVBoxLayout(targets_group)
        self.capture_table = QTableWidget(0, 6)
        self.capture_table.setHorizontalHeaderLabels(
            ['目标ID', '类型', '置信度', 'X / m', 'Y / m', '来源']
        )
        self._configure_table(self.capture_table)
        self.capture_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.capture_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        targets_layout.addWidget(self.capture_table)
        targets_group.setMaximumHeight(170)
        layout.addWidget(targets_group)

        command_group = QGroupBox('围捕控制')
        command_layout = QGridLayout(command_group)
        self.capture_status = QLabel('等待无人机巡逻感知')
        self.capture_status.setObjectName('captureStatus')
        command_layout.addWidget(QLabel('当前状态'), 0, 0)
        command_layout.addWidget(self.capture_status, 0, 1, 1, 3)

        capture_button = QPushButton('启动围捕')
        capture_button.clicked.connect(self._capture_selected_target)
        pause_button = QPushButton('暂停任务')
        pause_button.clicked.connect(lambda: self._send_action('HOLD_ALL'))
        continue_button = QPushButton('继续任务')
        continue_button.clicked.connect(self._capture_selected_target)
        cancel_button = QPushButton('停止任务')
        cancel_button.setObjectName('danger')
        cancel_button.clicked.connect(
            lambda: self._send_action('CANCEL_CAPTURE')
        )
        command_layout.addWidget(capture_button, 1, 0)
        command_layout.addWidget(pause_button, 1, 1)
        command_layout.addWidget(continue_button, 1, 2)
        command_layout.addWidget(cancel_button, 1, 3)
        layout.addWidget(command_group)

    @staticmethod
    def _status_card(title, value):
        label = QLabel('%s\n%s' % (title, value))
        label.setObjectName('statusCard')
        label.setAlignment(Qt.AlignCenter)
        return label

    def _add_camera_group(self, layout, title, attribute):
        group = QGroupBox(title)
        camera_layout = QVBoxLayout(group)
        camera_layout.setContentsMargins(0, 0, 0, 0)
        camera_layout.setSpacing(0)
        camera = VideoMosaicLabel('等待真实相机数据')
        camera.setMinimumSize(720, 180)
        camera.setStyleSheet('background: #0d141a; color: #8fa5b2;')
        camera_layout.addWidget(camera)
        setattr(self, attribute, camera)
        layout.addWidget(group, 0)

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
            'mid360': 2,
            'front_lidar': 3,
            'navigation': 4,
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

    def _fleet_row(self, vehicle):
        return self._ordered_insert_row(
            self.fleet_table,
            self.fleet_rows,
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

    def _queue_image(self, data):
        self._touch_ros()
        if isinstance(data, tuple):
            source, image = data
        else:
            source, image = 'default', data
        self.pending_images[source] = image

    def _flush_image(self):
        if not self.pending_images:
            return
        images = dict(self.pending_images)
        self.pending_images.clear()
        for source, image in images.items():
            self._update_image(image, source)

    def _update_image(self, image, source=''):
        self.last_image_time = time.monotonic()
        if source == 'defense':
            self.defense_camera.set_image(image)
        elif source == 'capture':
            self.capture_camera.set_image(image)
        else:
            # A single-world deployment shares the real sensor mosaic between
            # both task pages while keeping their controls and status separate.
            self.defense_camera.set_image(image)
            self.capture_camera.set_image(image)
        self.link_label.setText('基站数据链路在线')
        self.link_label.setStyleSheet(
            'background: #d9f0e5; color: #176b47; '
            'padding: 7px 12px; border: 1px solid #8cc5a8;'
        )

    def _update_sensor(self, data):
        self._touch_ros()
        (
            vehicle,
            sensor,
            frame_id,
            rate,
            age,
            latency,
            processing_ms,
            point_count,
            messages,
            total_bytes,
            dropped,
            healthy,
            timed_out,
            last_sec,
            last_nanosec,
        ) = data
        row = self._sensor_row(vehicle, sensor)
        values = [
            self.VEHICLE_NAMES.get(vehicle, vehicle),
            self.SENSOR_NAMES.get(sensor, sensor),
            frame_id or '-',
            '%.1f Hz' % rate,
            '%.1f ms' % (latency * 1000.0),
            str(point_count) if point_count else '-',
            str(dropped),
            '%.2f ms' % processing_ms if processing_ms else '-',
            '%.1f MB' % (total_bytes / 1048576.0),
        ]
        for column, value in enumerate(values):
            self.sensor_table.setItem(row, column, self._item(value))
        self.sensor_table.setItem(
            row,
            9,
            self._item(
                '正常' if healthy else ('超时' if timed_out else '等待'),
                '#16834a' if healthy else '#b63737',
            ),
        )
        self.sensor_table.setToolTip(
            '%s/%s  最近消息=%d.%09d  age=%.3fs  messages=%d'
            % (
                vehicle,
                sensor,
                last_sec,
                last_nanosec,
                age,
                messages,
            )
        )

    def _toggle_mid360_preview(self, enabled):
        self.mid360_preview_button.setText(
            'RViz点云预览：%s' % ('开启' if enabled else '关闭')
        )
        self.node.set_mid360_preview(enabled)

    def _update_vehicle(self, data):
        self._touch_ros()
        (
            vehicle, vehicle_type, online, armed, mode, x, y, z,
            vx, vy, status,
        ) = data
        state = {
            'vehicle_id': vehicle,
            'vehicle_type': vehicle_type,
            'online': online,
            'armed': armed,
            'mode': mode,
            'x': x,
            'y': y,
            'z': z,
            'vx': vx,
            'vy': vy,
            'status': status,
            'updated': time.monotonic(),
        }
        self.vehicle_cache[vehicle] = state
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
        role = self.capture_roles_cache.get(vehicle, {})
        fleet_row = self._fleet_row(vehicle)
        control = 'PX4 / %s' % mode if vehicle.startswith('uav_') \
            else 'Nav2 / %s' % mode
        fleet_values = [
            vehicle.upper(),
            role.get('role_name', 'Standby'),
            'ONLINE' if online else 'OFFLINE',
            control,
        ]
        for column, value in enumerate(fleet_values):
            color = '#16834a' if column == 2 and online else None
            if column == 2 and not online:
                color = '#b63737'
            self.fleet_table.setItem(
                fleet_row, column, self._item(value, color)
            )
        self.capture_overview_map.set_vehicle(state)
        self.capture_detail_map.set_vehicle(state)
        self._refresh_fleet_summary()

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

    def _update_capture_targets(self, targets):
        selected_id = None
        current = self.capture_table.currentRow()
        if current >= 0:
            item = self.capture_table.item(current, 0)
            if item is not None:
                selected_id = item.text()
        self.capture_table.setRowCount(0)
        class_names = {
            0: '未知',
            1: '船舶',
            2: '浮标',
            3: '漂浮物',
            4: '灯塔目标',
        }
        source_names = {
            1: '雷达',
            2: '相机',
            4: 'AIS',
            8: '融合',
        }
        restore_row = -1
        for row, target in enumerate(targets):
            self.capture_table.insertRow(row)
            values = [
                target['track_id'],
                class_names.get(target['class'], str(target['class'])),
                '%.0f%%' % (target['confidence'] * 100.0),
                '%.1f' % target['x'],
                '%.1f' % target['y'],
                source_names.get(target['source'], str(target['source'])),
            ]
            for column, value in enumerate(values):
                self.capture_table.setItem(row, column, self._item(value))
            if target['track_id'] == selected_id:
                restore_row = row
        if restore_row >= 0:
            self.capture_table.selectRow(restore_row)
        elif targets:
            self.capture_table.selectRow(0)

    def _update_capture_status(self, fields):
        mode = fields.get('mode', 'idle')
        if mode == 'capture':
            text = '围捕中: %s  目标( %s, %s )  半径 %s m' % (
                fields.get('target', '-'),
                fields.get('x', '-'),
                fields.get('y', '-'),
                fields.get('radius', '-'),
            )
            self.capture_status.setStyleSheet(
                'background: #ffe7d6; color: #9b3f00; padding: 8px;'
            )
        else:
            text = '空闲，等待选择目标'
            self.capture_status.setStyleSheet(
                'background: #d9f0e5; color: #176b47; padding: 8px;'
            )
        self.capture_status.setText(text)

    def _update_capture_state(self, state):
        self._touch_ros()
        previous = self.capture_state_cache.get('state_name')
        self.capture_state_cache = state
        self.capture_overview_map.set_capture_state(state)
        self.capture_detail_map.set_capture_state(state)
        state_name = state.get('state_name', 'SEARCH')
        target_id = state.get('target_id') or 'enemy_target'
        self.mission_state_label.setText('MISSION\n%s' % state_name)
        self.target_state_label.setText('TARGET\n%s' % target_id)
        active = state.get('active_uavs', 0) + state.get('active_usvs', 0)
        self.demo_summary.setText(
            'MISSION\nDYNAMIC CAPTURE\n\nSTATE\n%s\n\nACTIVE\n%d VEHICLES'
            % (state_name, active)
        )
        self.capture_status.setText(
            '%s | target=%s | generation=%s%s | %s'
            % (
                state_name,
                target_id,
                state.get('generation', 0),
                ' | DEGRADED' if state.get('degraded') else '',
                state.get('reason', ''),
            )
        )
        color = '#16834a' if state_name in ('HOLDING', 'SUCCESS') else '#9b3f00'
        background = '#d9f0e5' if state_name in ('HOLDING', 'SUCCESS') else '#ffe7d6'
        self.capture_status.setStyleSheet(
            'background: %s; color: %s; padding: 8px;' % (background, color)
        )
        if previous != state_name:
            self._append_log('Capture state: %s (%s)' % (
                state_name, state.get('reason', '')
            ))
        self._refresh_fleet_summary()

    def _update_capture_roles(self, roles):
        self._touch_ros()
        previous_generation = self.capture_roles_cache.get('_generation')
        self.capture_roles_cache = {
            item['vehicle_id']: item for item in roles.get('assignments', [])
        }
        self.capture_roles_cache['_generation'] = roles.get('generation', 0)
        self.capture_overview_map.set_roles(roles)
        self.capture_detail_map.set_roles(roles)
        self.assignment_table.setRowCount(0)
        for row, item in enumerate(roles.get('assignments', [])):
            self.assignment_table.insertRow(row)
            values = [
                item['vehicle_id'].upper(),
                item['role_name'],
                '(%.1f, %.1f, %.1f)' % (item['x'], item['y'], item['z']),
                item['status'] if item['active'] else 'INACTIVE',
                '%.1f' % item['cost'],
            ]
            for column, value in enumerate(values):
                self.assignment_table.setItem(row, column, self._item(value))
            fleet_row = self.fleet_rows.get(item['vehicle_id'])
            if fleet_row is not None:
                self.fleet_table.setItem(
                    fleet_row, 1, self._item(item['role_name'])
                )
        if previous_generation != roles.get('generation'):
            active_roles = [
                '%s=%s' % (item['vehicle_id'], item['role_name'])
                for item in roles.get('assignments', []) if item['active']
            ]
            self._append_log('Assignment generation %s: %s' % (
                roles.get('generation', 0), ', '.join(active_roles)
            ))

    def _update_capture_target(self, target):
        self._touch_ros()
        first_track = not self.capture_target_cache.get('tracked', False)
        self.capture_target_cache = target
        self.capture_overview_map.set_target(target)
        self.capture_detail_map.set_target(target)
        values = {
            'target': target.get('track_id', 'enemy_target'),
            'position': '(%.1f, %.1f, %.1f) m' % (
                target.get('x', 0.0), target.get('y', 0.0),
                target.get('z', 0.0),
            ),
            'velocity': '%.1f m/s  turn %.2f rad/s' % (
                target.get('speed', 0.0), target.get('turn_rate', 0.0)
            ),
            'prediction': '%s / 12 s' % (target.get('model') or 'unknown'),
            'tracking': '%s | confirmations=%d | age=%.2fs' % (
                'TRACKED' if target.get('tracked') else 'STALE',
                target.get('confirmations', 0), target.get('age', 0.0),
            ),
        }
        for key, value in values.items():
            self.capture_target_labels[key].setText(value)
        self.target_state_label.setText(
            'TARGET\n%s' % target.get('track_id', 'enemy_target')
        )
        if first_track and target.get('tracked'):
            self._append_log('Target detected: %s' % target.get('track_id'))

    def _update_capture_markers(self, data):
        self._touch_ros()
        self.capture_overview_map.set_markers(data)
        self.capture_detail_map.set_markers(data)

    def _touch_ros(self):
        self.last_ros_message_time = time.monotonic()

    def _refresh_connection_status(self):
        connected = (
            self.last_ros_message_time > 0.0
            and time.monotonic() - self.last_ros_message_time < 3.0
        )
        self.system_status_label.setText(
            'SYSTEM\n%s' % ('READY' if connected else 'WAITING')
        )
        self.link_label.setText(
            'ROS 2 已连接' if connected else '等待 ROS 2 数据'
        )
        self.link_label.setStyleSheet(
            ('background: #d9f0e5; color: #176b47; '
             'padding: 7px 12px; border: 1px solid #8cc5a8;')
            if connected else
            ('background: #fff4d7; color: #7a5700; '
             'padding: 7px 12px; border: 1px solid #dfc26d;')
        )

    def _refresh_fleet_summary(self):
        now = time.monotonic()
        online = [
            item for item in self.vehicle_cache.values()
            if item['online'] and now - item['updated'] < 3.0
        ]
        uavs = sum(item['vehicle_id'].startswith('uav_') for item in online)
        usvs = sum(item['vehicle_id'].startswith('usv_') for item in online)
        configured_uavs = self.capture_state_cache.get('configured_uavs', 4)
        configured_usvs = self.capture_state_cache.get('configured_usvs', 2)
        self.uav_count_label.setText('UAV\n%d / %d' % (uavs, configured_uavs))
        self.usv_count_label.setText('USV\n%d / %d' % (usvs, configured_usvs))

    def _show_selected_vehicle(self):
        row = self.fleet_table.currentRow()
        if row < 0:
            return
        item = self.fleet_table.item(row, 0)
        if item is None:
            return
        vehicle_id = item.text().lower()
        state = self.vehicle_cache.get(vehicle_id)
        if state is None:
            return
        role = self.capture_roles_cache.get(vehicle_id, {})
        control = 'PX4' if vehicle_id.startswith('uav_') else 'Nav2'
        self.vehicle_detail.setText(
            '%s\nRole: %s\n%s: %s\nMode: %s\nPosition: (%.1f, %.1f, %.1f)\nStatus: %s'
            % (
                vehicle_id.upper(), role.get('role_name', 'Standby'),
                control, 'CONNECTED' if state['online'] else 'OFFLINE',
                state['mode'], state['x'], state['y'], state['z'],
                state['status'],
            )
        )

    def _reset_capture_view(self):
        self.capture_overview_map.vehicle_history.clear()
        self.capture_overview_map.target_history.clear()
        self.capture_detail_map.vehicle_history.clear()
        self.capture_detail_map.target_history.clear()
        self.capture_overview_map.update()
        self.capture_detail_map.update()
        self._append_log('态势显示轨迹已复位（任务未停止）')

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

    def _capture_selected_target(self):
        row = self.capture_table.currentRow()
        if row < 0:
            self._append_log('请先选择一个围捕目标')
            return
        item = self.capture_table.item(row, 0)
        if item is None:
            self._append_log('目标行数据为空，无法围捕')
            return
        target_id = item.text()
        self.node.publish_action('CAPTURE:%s' % target_id)
        self._append_log('基站确认围捕目标: %s' % target_id)


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
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal_timer = QTimer()
    signal_timer.setInterval(200)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()
    try:
        result = app.exec_()
    except KeyboardInterrupt:
        result = 0
    finally:
        signal_timer.stop()
        try:
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    sys.exit(result)


if __name__ == '__main__':
    main()
