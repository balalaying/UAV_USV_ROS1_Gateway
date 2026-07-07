#!/usr/bin/env python3
import math
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QObject, QPointF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
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
from PyQt5.QtWidgets import QSplitter
from PyQt5.QtWidgets import QTableWidget
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
import rclpy
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
    log = pyqtSignal(str)


class BaseStationGuiNode(Node):
    VEHICLE_NAMES = {
        'uav_01': '无人机01',
        'uav_02': '无人机02',
        'uav_03': '无人机03',
        'usv_01': '船01',
        'usv_02': '船02',
        'usv_03': '船03',
    }

    def __init__(self, signals):
        super().__init__('fleet_base_station_gui')
        self.signals = signals

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        state_qos = QoSProfile(depth=5)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        state_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            Image,
            '/fleet/base/camera_mosaic',
            self._on_image,
            sensor_qos,
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
        self.goal_pub = self.create_publisher(
            PoseStamped, '/fleet/base/operator_goal', 10
        )
        self.action_pub = self.create_publisher(
            String, '/fleet/base/operator_action', 10
        )

    def _on_image(self, msg):
        if msg.encoding.lower() != 'bgr8':
            return
        image = QImage(
            bytes(msg.data),
            msg.width,
            msg.height,
            msg.step,
            QImage.Format_BGR888,
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


class RadarWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.points = []
        self.range_max = 40.0
        self.setMinimumSize(360, 280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_scan(self, scan):
        self.points, self.range_max = scan
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#101820'))

        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = max(10.0, min(self.width(), self.height()) / 2.0 - 24.0)
        painter.setPen(QPen(QColor('#35505f'), 1))
        for fraction in (0.25, 0.5, 0.75, 1.0):
            ring = radius * fraction
            painter.drawEllipse(center, ring, ring)
        painter.drawLine(
            QPointF(center.x() - radius, center.y()),
            QPointF(center.x() + radius, center.y()),
        )
        painter.drawLine(
            QPointF(center.x(), center.y() - radius),
            QPointF(center.x(), center.y() + radius),
        )

        painter.setPen(QPen(QColor('#33e6c4'), 3))
        scale = radius / max(0.1, self.range_max)
        for angle, distance in self.points:
            x = center.x() + math.cos(angle) * distance * scale
            y = center.y() - math.sin(angle) * distance * scale
            painter.drawPoint(QPointF(x, y))

        painter.setPen(QPen(QColor('#f3c969'), 2))
        painter.drawEllipse(center, 5, 5)
        painter.setPen(QColor('#b7c9d3'))
        painter.drawText(12, 22, 'USV LASER SCAN')
        painter.drawText(
            12, self.height() - 12, '量程 %.1f m' % self.range_max
        )


class BaseStationWindow(QMainWindow):
    VEHICLE_NAMES = {
        'uav_01': '无人机01',
        'uav_02': '无人机02',
        'uav_03': '无人机03',
        'usv_01': '船01',
        'usv_02': '船02',
        'usv_03': '船03',
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
    }

    def __init__(self, node, signals):
        super().__init__()
        self.node = node
        self.last_image_time = 0.0
        self.sensor_rows = {}
        self.vehicle_rows = {}
        self.setWindowTitle('UAV-USV 集群基站')
        self.resize(1500, 900)
        self._build_ui()

        signals.image.connect(self._update_image)
        signals.scan.connect(self.radar.set_scan)
        signals.sensor.connect(self._update_sensor)
        signals.vehicle.connect(self._update_vehicle)
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

        camera_group = QGroupBox('基站接收的实时视频（三组船机）')
        camera_layout = QVBoxLayout(camera_group)
        self.camera = QLabel('等待 UAV / USV 相机数据')
        self.camera.setAlignment(Qt.AlignCenter)
        self.camera.setMinimumHeight(390)
        self.camera.setStyleSheet('background: #0d141a; color: #8fa5b2;')
        camera_layout.addWidget(self.camera)
        layout.addWidget(camera_group, 5)

        lower = QSplitter(Qt.Horizontal)
        self.radar = RadarWidget()
        lower.addWidget(self.radar)

        telemetry = QWidget()
        telemetry_layout = QVBoxLayout(telemetry)
        telemetry_layout.setContentsMargins(0, 0, 0, 0)

        sensor_group = QGroupBox('传感器上行状态')
        sensor_layout = QVBoxLayout(sensor_group)
        self.sensor_table = QTableWidget(0, 7)
        self.sensor_table.setHorizontalHeaderLabels(
            ['载具', '传感器', '频率', '延迟', '消息数', '数据量', '状态']
        )
        self._configure_table(self.sensor_table)
        sensor_layout.addWidget(self.sensor_table)
        telemetry_layout.addWidget(sensor_group)

        vehicle_group = QGroupBox('载具状态')
        vehicle_layout = QVBoxLayout(vehicle_group)
        self.vehicle_table = QTableWidget(0, 7)
        self.vehicle_table.setHorizontalHeaderLabels(
            ['载具', '在线', '解锁', '模式', 'X / m', 'Y / m', 'Z / m']
        )
        self._configure_table(self.vehicle_table)
        vehicle_layout.addWidget(self.vehicle_table)
        telemetry_layout.addWidget(vehicle_group)
        lower.addWidget(telemetry)
        lower.setSizes([520, 850])
        layout.addWidget(lower, 4)

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
        layout.addWidget(control_group)

        log_group = QGroupBox('基站命令回执')
        log_layout = QVBoxLayout(log_group)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setMinimumHeight(110)
        log_layout.addWidget(self.log)
        layout.addWidget(log_group)

        self.setCentralWidget(root)
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #e9eef1; color: #17242c; }
            QLabel#title { font-size: 25px; font-weight: 700; }
            QLabel#subtitle { color: #526873; font-size: 14px; }
            QLabel#link {
                background: #d7e2e7; color: #526873;
                padding: 7px 12px; border: 1px solid #b8c7ce;
            }
            QGroupBox {
                background: #f8fafb; border: 1px solid #b9c7ce;
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

    def _update_image(self, image):
        self.last_image_time = time.monotonic()
        pixmap = QPixmap.fromImage(image)
        self.camera.setPixmap(
            pixmap.scaled(
                self.camera.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )
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
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    window = BaseStationWindow(node, signals)
    window.show()
    result = app.exec_()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    sys.exit(result)


if __name__ == '__main__':
    main()
