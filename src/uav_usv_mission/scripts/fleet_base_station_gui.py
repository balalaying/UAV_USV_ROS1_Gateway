#!/usr/bin/env python3
import json
import math
import signal
import sys
import threading
import time

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
import numpy as np
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
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QComboBox
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
from PyQt5.QtWidgets import QSpinBox
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
from rclpy.time import Time
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import CaptureAssignmentArray
from uav_usv_interfaces.msg import CaptureState
from uav_usv_interfaces.msg import CaptureTargetStatus
from uav_usv_interfaces.msg import SensorStatus
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState
from uav_usv_mission.perception_topdown import PerceptionTopDownWidget
from uav_usv_mission.perception_topdown import TopDownVisualizationModel
from uav_usv_mission.lv_dot_debug_visualization import LvDotDebugModel
from uav_usv_mission.lv_dot_debug_visualization import LvDotDebugWidget
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
    perception_metrics = pyqtSignal(object)
    perception_fusion_metrics = pyqtSignal(object)
    multisensor_metrics = pyqtSignal(object)
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


class CameraInsetLabel(QLabel):
    """Low-copy 16:9 camera preview used beside the perception canvas."""

    def __init__(self, text=''):
        super().__init__(text)
        self._image = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumHeight(120)
        self.setMaximumHeight(180)
        self.setStyleSheet('background: #05090d; color: #8fa5b2;')

    def set_image(self, image):
        self._image = image
        self.update()

    def paintEvent(self, event):
        if self._image is None:
            super().paintEvent(event)
            return
        painter = QPainter(self)
        scaled = self._image.scaled(
            self.size(), Qt.KeepAspectRatioByExpanding,
            Qt.SmoothTransformation,
        )
        x_offset = (self.width() - scaled.width()) // 2
        y_offset = (self.height() - scaled.height()) // 2
        painter.drawImage(x_offset, y_offset, scaled)


class BaseStationGuiNode(Node):
    VEHICLE_NAMES = {
        'uav_01': '我方无人机一号',
        'uav_02': '我方无人机二号',
        'uav_03': '我方无人机三号',
        'usv_01': '我方船一号（蓝色）',
        'usv_02': '我方船二号（绿色）',
        'usv_03': '我方船三号（青色）',
    }

    def __init__(
        self, signals, visualization_model=None, lv_dot_debug_model=None
    ):
        super().__init__('fleet_base_station_gui')
        self.signals = signals
        self.visualization_model = (
            visualization_model or TopDownVisualizationModel()
        )
        self.lv_dot_debug_model = lv_dot_debug_model or LvDotDebugModel()
        self.declare_parameter('capture_namespace', '')
        self.declare_parameter('defense_namespace', '')
        self.declare_parameter('defense_node_name', '/defense_sim_demo')
        self.declare_parameter('demo_mode', False)
        self.declare_parameter('enable_perception_topdown', True)
        self.declare_parameter('enable_lv_dot_debug', True)
        self.declare_parameter('enable_affiliation_qt_mode', True)
        self.declare_parameter(
            'topdown_points_topic',
            '/perception/visualization/usv_01/topdown_points',
        )
        self.declare_parameter(
            'topdown_status_topic',
            '/perception/visualization/usv_01/topdown_status',
        )
        self.declare_parameter(
            'topdown_lidar_bboxes_topic',
            '/perception/lv_dot_ros2/diagnostics/lidar_bboxes',
        )
        self.declare_parameter(
            'topdown_tracks_topic', '/perception/lv_dot_ros2/tracks'
        )
        self.declare_parameter(
            'topdown_dynamic_tracks_topic',
            '/perception/lv_dot_ros2/dynamic_tracks',
        )
        self.declare_parameter(
            'topdown_fused_tracks_topic', '/perception/fused/tracks'
        )
        self.declare_parameter(
            'topdown_ground_truth_topic',
            '/perception/ground_truth/tracks',
        )
        self.declare_parameter(
            'topdown_camera_topic',
            '/fleet/uplink/uav_01/camera/image_raw',
        )
        self.declare_parameter(
            'topdown_usv_pose_topic',
            '/perception/lv_dot/usv_01/pose',
        )
        self.declare_parameter(
            'mid360_preview_service',
            '/perception/usv_01/mid360/set_visualization',
        )
        self.declare_parameter(
            'lv_dot_debug_raw_cloud_topic',
            '/perception/visualization/usv_01/topdown_points',
        )
        self.declare_parameter(
            'lv_dot_debug_filtered_cloud_topic',
            '/perception/lv_dot/debug/cloud_filtered_map',
        )
        self.declare_parameter(
            'lv_dot_debug_clusters_topic',
            '/perception/lv_dot/debug/clusters',
        )
        self.declare_parameter(
            'lv_dot_debug_bboxes_topic',
            '/perception/lv_dot/debug/bboxes',
        )
        self.declare_parameter(
            'camera_lidar_lidar_only_bboxes_topic',
            '/perception/usv_01/camera_lidar/lidar_only_bboxes',
        )
        self.declare_parameter(
            'camera_lidar_camera_only_bboxes_topic',
            '/perception/usv_01/camera_lidar/camera_only_bboxes',
        )
        self.declare_parameter(
            'camera_lidar_fused_bboxes_topic',
            '/perception/usv_01/camera_lidar/fused_bboxes',
        )
        self.declare_parameter(
            'camera_lidar_calibration_roi_topic',
            '/perception/usv_01/vision_guided/roi_cloud',
        )
        self.declare_parameter(
            'camera_lidar_camera_projection_topic',
            '/perception/usv_01/vision_guided/camera_projection',
        )
        self.declare_parameter(
            'camera_lidar_calibration_bbox_topic',
            '/perception/usv_01/vision_guided/roi_bboxes',
        )
        self.declare_parameter(
            'camera_lidar_status_topic',
            '/perception/usv_01/camera_lidar/status',
        )
        self.declare_parameter(
            'vision_guided_status_topic',
            '/perception/usv_01/vision_guided/status',
        )
        self.declare_parameter(
            'camera_detection_status_topic',
            '/perception/usv_01/camera/detection_status',
        )
        self.declare_parameter(
            'lv_dot_debug_tracks_topic',
            '/perception/lv_dot/debug/tracks',
        )
        self.declare_parameter(
            'lv_dot_debug_dynamic_topic',
            '/perception/lv_dot/debug/dynamic',
        )
        self.declare_parameter(
            'lv_dot_debug_fusion_topic', '/perception/fused/tracks'
        )
        self.declare_parameter(
            'lv_dot_debug_status_topic', '/perception/lv_dot/debug/status'
        )
        self.declare_parameter('lv_dot_debug_fixed_frame', 'map')
        self.declare_parameter(
            'lv_dot_debug_base_frame', 'usv_01/base_link'
        )
        self.declare_parameter(
            'lv_dot_debug_radar_frame', 'usv_01/mid360_link'
        )
        self.demo_mode = bool(self.get_parameter('demo_mode').value)
        self.enable_perception_topdown = bool(
            self.get_parameter('enable_perception_topdown').value
        )
        self.enable_lv_dot_debug = bool(
            self.get_parameter('enable_lv_dot_debug').value
        )
        self.enable_affiliation_qt_mode = bool(
            self.get_parameter('enable_affiliation_qt_mode').value
        )
        self.lv_dot_debug_fixed_frame = str(
            self.get_parameter('lv_dot_debug_fixed_frame').value
        )
        self.lv_dot_debug_frames = {
            'base': str(self.get_parameter('lv_dot_debug_base_frame').value),
            'radar': str(
                self.get_parameter('lv_dot_debug_radar_frame').value
            ),
        }
        self.camera_lidar_status = {}
        self.vision_guided_status = {}
        self.camera_detection_status = {}
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
        topdown_qos = QoSProfile(depth=1)
        topdown_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        topdown_qos.durability = DurabilityPolicy.VOLATILE

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
        if self.enable_perception_topdown or self.enable_lv_dot_debug:
            self.create_subscription(
                Image,
                str(self.get_parameter('topdown_camera_topic').value),
                lambda message: self._on_image(
                    message, 'topdown_camera'
                ),
                image_qos,
            )
        if self.enable_perception_topdown:
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter('topdown_points_topic').value),
                self._on_topdown_points,
                topdown_qos,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('topdown_status_topic').value),
                self._on_topdown_status,
                10,
            )
            self.create_subscription(
                MarkerArray,
                str(self.get_parameter('topdown_lidar_bboxes_topic').value),
                self._on_topdown_lidar_bboxes,
                topdown_qos,
            )
            for parameter_name, layer in (
                ('topdown_tracks_topic', 'tracks'),
                ('topdown_dynamic_tracks_topic', 'dynamic'),
                ('topdown_fused_tracks_topic', 'fusion'),
                ('topdown_ground_truth_topic', 'ground_truth'),
            ):
                self.create_subscription(
                    TrackedObjectArray,
                    str(self.get_parameter(parameter_name).value),
                    lambda message, track_layer=layer: (
                        self.visualization_model.update_tracks(
                            track_layer, message
                        )
                    ),
                    topdown_qos,
                )
        if self.enable_lv_dot_debug:
            self.lv_dot_tf_buffer = Buffer()
            self.lv_dot_tf_listener = TransformListener(
                self.lv_dot_tf_buffer, self
            )
            self.lv_dot_tf_timer = self.create_timer(
                0.1, self._update_lv_dot_debug_tf
            )
            for parameter_name, layer in (
                ('lv_dot_debug_raw_cloud_topic', 'raw'),
                ('lv_dot_debug_filtered_cloud_topic', 'filtered'),
                (
                    'camera_lidar_calibration_roi_topic',
                    'calibration_roi',
                ),
            ):
                self.create_subscription(
                    PointCloud2,
                    str(self.get_parameter(parameter_name).value),
                    lambda message, cloud_layer=layer: (
                        self._on_lv_dot_debug_cloud(message, cloud_layer)
                    ),
                    topdown_qos,
                )
            for parameter_name, layer in (
                ('lv_dot_debug_clusters_topic', 'clusters'),
                ('lv_dot_debug_bboxes_topic', 'bboxes'),
                (
                    'camera_lidar_lidar_only_bboxes_topic',
                    'lidar_only_bboxes',
                ),
                (
                    'camera_lidar_camera_only_bboxes_topic',
                    'camera_only_bboxes',
                ),
                (
                    'camera_lidar_fused_bboxes_topic',
                    'camera_lidar_fused_bboxes',
                ),
                (
                    'camera_lidar_camera_projection_topic',
                    'camera_projection',
                ),
                (
                    'camera_lidar_calibration_bbox_topic',
                    'calibration_bbox',
                ),
            ):
                self.create_subscription(
                    MarkerArray,
                    str(self.get_parameter(parameter_name).value),
                    lambda message, marker_layer=layer: (
                        self.lv_dot_debug_model.update_markers(
                            marker_layer, message
                        )
                    ),
                    topdown_qos,
                )
            for parameter_name, layer in (
                ('lv_dot_debug_tracks_topic', 'tracks'),
                ('lv_dot_debug_dynamic_topic', 'dynamic'),
                ('lv_dot_debug_fusion_topic', 'fusion'),
            ):
                self.create_subscription(
                    TrackedObjectArray,
                    str(self.get_parameter(parameter_name).value),
                    lambda message, track_layer=layer: (
                        self.lv_dot_debug_model.update_tracks(
                            track_layer, message
                        )
                    ),
                    topdown_qos,
                )
            self.create_subscription(
                String,
                str(self.get_parameter('lv_dot_debug_status_topic').value),
                self._on_lv_dot_debug_status,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('vision_guided_status_topic').value),
                self._on_vision_guided_status,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('camera_detection_status_topic').value),
                self._on_camera_detection_status,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('camera_lidar_status_topic').value),
                self._on_camera_lidar_status,
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
            self._topic(
                self.capture_namespace,
                '/perception/lv_dot/shadow_metrics',
            ),
            self._on_perception_metrics,
            10,
        )
        self.create_subscription(
            String,
            self._topic(
                self.capture_namespace,
                '/perception/lv_dot/fusion_metrics',
            ),
            self._on_perception_fusion_metrics,
            10,
        )
        self.create_subscription(
            String,
            self._topic(
                self.capture_namespace,
                '/perception/multisensor/metrics',
            ),
            self._on_multisensor_metrics,
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
                msg.tf_target_frame,
                msg.tf_available,
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
        if self.enable_perception_topdown:
            self.visualization_model.update_vehicle(msg)
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
        if self.enable_perception_topdown:
            self.visualization_model.update_tracks('ground_truth', msg)
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
        if self.enable_perception_topdown:
            self.visualization_model.update_roles(assignments)

    def _on_topdown_points(self, msg):
        try:
            xyz = point_cloud2.read_points_numpy(
                msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
            xyz = np.asarray(xyz, dtype=np.float32).reshape((-1, 3))
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(
                'Invalid top-down PointCloud2: %s' % error
            )
            return
        self.visualization_model.update_point_array(xyz[:, :2], xyz[:, 2])

    @staticmethod
    def _pointcloud_xyz(msg):
        values = point_cloud2.read_points_numpy(
            msg, field_names=['x', 'y', 'z'], skip_nans=True
        )
        return np.asarray(values, dtype=np.float32).reshape((-1, 3))

    def _on_lv_dot_debug_cloud(self, msg, layer):
        try:
            xyz = self._pointcloud_xyz(msg)
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(
                'Invalid LV-DOT debug PointCloud2: %s' % error
            )
            return
        self.lv_dot_debug_model.update_cloud(layer, xyz)

    def _on_lv_dot_debug_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning('Invalid LV-DOT debug status JSON')
            return
        if isinstance(status, dict):
            self.lv_dot_debug_model.update_status(status)

    def _on_camera_lidar_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning(
                'Invalid camera-LiDAR fusion status JSON'
            )
            return
        if isinstance(status, dict):
            self.camera_lidar_status = status

    def _on_vision_guided_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(status, dict):
            self.vision_guided_status = status

    def _on_camera_detection_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(status, dict):
            self.camera_detection_status = status

    def _update_lv_dot_debug_tf(self):
        for key, frame_id in self.lv_dot_debug_frames.items():
            try:
                transform = self.lv_dot_tf_buffer.lookup_transform(
                    self.lv_dot_debug_fixed_frame, frame_id, Time()
                )
            except TransformException:
                continue
            self.lv_dot_debug_model.update_frame(
                key, frame_id, transform.transform
            )

    def _on_topdown_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning('Invalid top-down status JSON')
            return
        if isinstance(status, dict):
            self.visualization_model.update_point_status(status)

    def _on_topdown_lidar_bboxes(self, msg):
        self.visualization_model.update_clusters(msg)

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

    def _on_perception_metrics(self, msg):
        try:
            metrics = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning('Invalid LV-DOT shadow metrics JSON')
            return
        if isinstance(metrics, dict):
            self.signals.perception_metrics.emit(metrics)

    def _on_perception_fusion_metrics(self, msg):
        try:
            metrics = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning('Invalid LV-DOT fusion metrics JSON')
            return
        if isinstance(metrics, dict):
            self.signals.perception_fusion_metrics.emit(metrics)

    def _on_multisensor_metrics(self, msg):
        try:
            metrics = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning('Invalid multisensor metrics JSON')
            return
        if isinstance(metrics, dict):
            self.signals.multisensor_metrics.emit(metrics)

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
                             self.target.get('track_id', 'enemy_ship'))

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
        'uav_01': '我方无人机一号',
        'uav_02': '我方无人机二号',
        'uav_03': '我方无人机三号',
        'usv_01': '我方船一号（蓝色）',
        'usv_02': '我方船二号（绿色）',
        'usv_03': '我方船三号（青色）',
    }
    SENSOR_NAMES = {
        'down_camera': '下视相机',
        'uav_camera': '无人机相机',
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
    }

    def __init__(
        self, node, signals, visualization_model=None,
        lv_dot_debug_model=None,
    ):
        super().__init__()
        self.node = node
        self.visualization_model = (
            visualization_model or node.visualization_model
        )
        self.lv_dot_debug_model = (
            lv_dot_debug_model or node.lv_dot_debug_model
        )
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
        self.status_timer.timeout.connect(self._refresh_topdown_status)
        self.status_timer.timeout.connect(self._refresh_lv_dot_debug_status)

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
        signals.perception_metrics.connect(self._update_perception_metrics)
        signals.perception_fusion_metrics.connect(
            self._update_perception_fusion_metrics
        )
        signals.multisensor_metrics.connect(self._update_multisensor_metrics)
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

        debug_tab = QWidget()
        debug_layout = QVBoxLayout(debug_tab)
        debug_layout.setContentsMargins(0, 0, 0, 0)
        debug_layout.setSpacing(10)
        tabs.addTab(debug_tab, 'Perception Monitor')

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
            ('启动围捕', 'CAPTURE:enemy_ship', False),
            ('暂停任务', 'HOLD_ALL', False),
            ('继续任务', 'CAPTURE:enemy_ship', False),
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
        self.sensor_table = QTableWidget(0, 11)
        self.sensor_table.setHorizontalHeaderLabels(
            [
                '载具', '传感器', 'Frame', '频率', '延迟',
                '点数', '丢帧', '处理', '数据量', 'TF', '状态',
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

        self._build_lv_dot_debug_panel(debug_layout)

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

        if self.node.demo_mode:
            tabs.setCurrentWidget(debug_tab)

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

    def _build_topdown_panel(self, layout):
        if not self.node.enable_perception_topdown:
            disabled = QLabel('俯视感知画布已通过启动参数关闭')
            disabled.setAlignment(Qt.AlignCenter)
            layout.addWidget(disabled)
            self.topdown_widget = None
            return

        splitter = QSplitter(Qt.Horizontal)
        controls_group = QGroupBox('俯视显示控制')
        controls_group.setMaximumWidth(255)
        controls = QVBoxLayout(controls_group)
        controls.addWidget(QLabel('固定坐标系：map'))
        controls.addWidget(QLabel('视角：Z轴向下 / 正交投影'))

        view_mode = QComboBox()
        view_mode.addItem('Map俯视 2D', 'topdown')
        view_mode.addItem('斜俯视 3D', 'oblique')

        follow = QComboBox()
        follow.addItem('自由视角', 'none')
        follow.addItem('自动跟随目标', 'target')
        follow.addItem('自动跟随USV', 'usv')
        color_mode = QComboBox()
        color_mode.addItem('固定白色', 'fixed')
        color_mode.addItem('按高度着色', 'height')
        controls.addWidget(QLabel('显示模式'))
        controls.addWidget(view_mode)

        controls.addWidget(QLabel('点云颜色'))
        controls.addWidget(color_mode)
        controls.addWidget(QLabel('视角跟随'))
        controls.addWidget(follow)

        self.topdown_layer_checks = {}
        layer_definitions = (
            ('点云', 'pointcloud'),
            ('LiDAR聚类框', 'clusters'),
            ('全部Track', 'tracks'),
            ('动态Track', 'dynamic'),
            ('融合目标', 'fusion'),
            ('Ground Truth', 'ground_truth'),
            ('UAV位置', 'uav'),
            ('USV位置', 'usv'),
            ('轨迹尾线', 'trails'),
            ('速度箭头', 'velocity'),
            ('标签', 'labels'),
            ('网格', 'grid'),
            ('船体TF', 'tf'),
            ('相机画中画', 'camera'),
        )
        layer_grid = QGridLayout()
        default_layers = {'pointcloud', 'clusters', 'grid', 'tf', 'camera'}
        for index, (title, key) in enumerate(layer_definitions):
            checkbox = QCheckBox(title)
            checkbox.setChecked(key in default_layers)
            self.topdown_layer_checks[key] = checkbox
            layer_grid.addWidget(checkbox, index // 2, index % 2)
        controls.addLayout(layer_grid)
        target_cluster_only = QCheckBox('仅显示目标聚类3D框')
        target_cluster_only.setChecked(True)
        controls.addWidget(target_cluster_only)

        def double_spin(minimum, maximum, value, suffix, step):
            spin = QDoubleSpinBox()
            spin.setRange(minimum, maximum)
            spin.setValue(value)
            spin.setSingleStep(step)
            spin.setSuffix(suffix)
            return spin

        range_spin = double_spin(10.0, 500.0, 35.0, ' m', 5.0)
        grid_spin = double_spin(1.0, 100.0, 10.0, ' m', 1.0)
        point_spin = double_spin(1.0, 10.0, 3.5, ' px', 0.5)
        max_points = QSpinBox()
        max_points.setRange(100, 100000)
        max_points.setSingleStep(500)
        max_points.setValue(50000)
        history = QSpinBox()
        history.setRange(10, 500)
        history.setValue(120)
        history.setSuffix(' 点')
        for title, control in (
            ('半视野范围', range_spin),
            ('网格间距', grid_spin),
            ('点大小', point_spin),
            ('最大显示点数', max_points),
            ('轨迹长度', history),
        ):
            row = QHBoxLayout()
            row.addWidget(QLabel(title))
            row.addWidget(control)
            controls.addLayout(row)

        clear_button = QPushButton('清除轨迹')
        reset_button = QPushButton('恢复默认视角')
        controls.addWidget(clear_button)
        controls.addWidget(reset_button)
        controls.addStretch()
        splitter.addWidget(controls_group)

        plot_group = QGroupBox('Map俯视感知态势')
        plot_layout = QVBoxLayout(plot_group)
        self.topdown_widget = PerceptionTopDownWidget(
            self.visualization_model
        )
        if not self.topdown_widget.opengl_available:
            view_mode.model().item(1).setEnabled(False)
            view_mode.setItemText(1, '斜俯视 3D（OpenGL不可用）')
        plot_layout.addWidget(self.topdown_widget)
        splitter.addWidget(plot_group)

        details_group = QGroupBox('目标与点云信息')
        details_group.setMaximumWidth(300)
        details_layout = QVBoxLayout(details_group)
        self.topdown_selected_detail = QLabel(
            '未选择目标\n\n点击目标图标或聚类框中心查看详情。'
        )
        self.topdown_selected_detail.setWordWrap(True)
        self.topdown_selected_detail.setObjectName('vehicleDetail')
        self.topdown_point_status = QLabel('等待点云投影数据')
        self.topdown_point_status.setWordWrap(True)
        self.topdown_point_status.setObjectName('vehicleDetail')
        camera_title = QLabel('我方船一号（蓝色）融合相机')
        camera_title.setObjectName('subtitle')
        self.perception_camera = CameraInsetLabel(
            '等待我方船一号相机数据'
        )
        details_layout.addWidget(camera_title)
        details_layout.addWidget(self.perception_camera)
        details_layout.addWidget(self.topdown_selected_detail)
        details_layout.addWidget(self.topdown_point_status)
        details_layout.addStretch()
        splitter.addWidget(details_group)
        splitter.setSizes([230, 930, 270])
        layout.addWidget(splitter)

        for key, checkbox in self.topdown_layer_checks.items():
            if key == 'camera':
                checkbox.toggled.connect(self.perception_camera.setVisible)
                continue
            callback = (
                self.topdown_widget.set_grid_visible
                if key == 'grid'
                else lambda checked, layer=key: (
                    self.topdown_widget.set_layer_visible(layer, checked)
                )
            )
            checkbox.toggled.connect(callback)
        target_cluster_only.toggled.connect(
            self.topdown_widget.set_single_target_cluster
        )
        follow.currentIndexChanged.connect(
            lambda _index: self.topdown_widget.set_follow_mode(
                follow.currentData()
            )
        )
        view_mode.currentIndexChanged.connect(
            lambda _index: self.topdown_widget.set_view_mode(
                view_mode.currentData()
            )
        )
        if self.node.demo_mode and self.topdown_widget.opengl_available:
            view_mode.setCurrentIndex(1)
        color_mode.currentIndexChanged.connect(
            lambda _index: self.topdown_widget.set_point_color_mode(
                color_mode.currentData()
            )
        )
        range_spin.valueChanged.connect(
            self.topdown_widget.set_display_range
        )
        grid_spin.valueChanged.connect(
            self.topdown_widget.set_grid_spacing
        )
        point_spin.valueChanged.connect(self.topdown_widget.set_point_size)
        max_points.valueChanged.connect(
            self.topdown_widget.set_max_display_points
        )
        history.valueChanged.connect(
            self.topdown_widget.set_trajectory_length
        )
        clear_button.clicked.connect(
            self.topdown_widget.clear_trajectories
        )
        reset_button.clicked.connect(self.topdown_widget.reset_view)
        self.topdown_widget.selection_callback = (
            self._update_topdown_selection
        )
        for checkbox in self.topdown_layer_checks.values():
            checkbox.toggled.emit(checkbox.isChecked())
        target_cluster_only.toggled.emit(target_cluster_only.isChecked())
        if self.node.demo_mode:
            follow.setCurrentIndex(1)

    def _build_lv_dot_debug_panel(self, layout):
        if not self.node.enable_lv_dot_debug:
            label = QLabel('LV-DOT Debug Visualization 已关闭')
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label)
            self.lv_dot_debug_widget = None
            return

        splitter = QSplitter(Qt.Horizontal)
        controls_group = QGroupBox('感知图层')
        controls_group.setMaximumWidth(245)
        controls = QVBoxLayout(controls_group)
        controls.addWidget(QLabel('Fixed Frame: map'))
        controls.addWidget(QLabel('View: Top / Z轴向下'))
        controls.addWidget(QLabel('SHADOW MODE（仅显示）'))

        view_mode = QComboBox()
        view_mode.addItem('斜俯视 3D', 'oblique')
        view_mode.addItem('Map俯视 2D', 'topdown')
        controls.addWidget(QLabel('显示模式'))
        controls.addWidget(view_mode)

        affiliation_color_mode = QComboBox()
        affiliation_color_mode.addItem('Sensor Source Mode', 'sensor_source')
        affiliation_color_mode.addItem('Affiliation Mode', 'affiliation')
        if self.node.enable_affiliation_qt_mode:
            affiliation_color_mode.setCurrentIndex(1)
        controls.addWidget(QLabel('目标着色模式'))
        controls.addWidget(affiliation_color_mode)
        legend = QLabel(
            '来源: LiDAR黄 / Camera蓝 / 融合绿\n'
            '阵营: 友方青 / 敌方红 / 中立灰 / 未知黄'
        )
        legend.setWordWrap(True)
        controls.addWidget(legend)

        definitions = (
            ('Mid360原始点云', 'raw', True),
            ('过滤后点云', 'filtered', True),
            ('DBSCAN Clusters', 'clusters', True),
            ('原始3D BBox', 'bboxes', False),
            ('LiDAR Only（黄）', 'lidar_only_bboxes', True),
            ('Camera Only（蓝）', 'camera_only_bboxes', True),
            (
                'Camera+LiDAR（绿）',
                'camera_lidar_fused_bboxes', True,
            ),
            ('标定：Camera投影（红）', 'camera_projection', True),
            ('标定：LiDAR ROI点（绿）', 'calibration_roi', True),
            ('标定：最终3D框（黄）', 'calibration_bbox', True),
            ('Tracks + 轨迹', 'tracks', True),
            ('Dynamic状态', 'dynamic', True),
            ('Fusion Target', 'fusion', True),
            ('船体 / Mid360 TF', 'tf', True),
            ('Track标签', 'labels', True),
            ('Map网格', 'grid', True),
        )
        self.lv_dot_debug_checks = {}
        for title, layer, checked in definitions:
            checkbox = QCheckBox(title)
            checkbox.setChecked(checked)
            checkbox.toggled.connect(
                lambda checked, key=layer: (
                    self.lv_dot_debug_widget.set_layer_visible(key, checked)
                )
            )
            self.lv_dot_debug_checks[layer] = checkbox
            controls.addWidget(checkbox)

        max_points = QSpinBox()
        max_points.setRange(1000, 100000)
        max_points.setSingleStep(2000)
        max_points.setValue(60000)
        trail_length = QSpinBox()
        trail_length.setRange(10, 500)
        trail_length.setValue(100)
        for title, control in (
            ('每层最大点数', max_points),
            ('轨迹历史长度', trail_length),
        ):
            row = QHBoxLayout()
            row.addWidget(QLabel(title))
            row.addWidget(control)
            controls.addLayout(row)
        clear_button = QPushButton('清除Track轨迹')
        reset_button = QPushButton('恢复Map俯视')
        controls.addWidget(clear_button)
        controls.addWidget(reset_button)
        controls.addStretch()
        splitter.addWidget(controls_group)

        canvas_group = QGroupBox('Map俯视感知态势')
        canvas_layout = QVBoxLayout(canvas_group)
        self.lv_dot_debug_widget = LvDotDebugWidget(
            self.lv_dot_debug_model
        )
        canvas_layout.addWidget(self.lv_dot_debug_widget)
        splitter.addWidget(canvas_group)

        status_group = QGroupBox('目标、相机与点云信息')
        status_group.setMaximumWidth(310)
        status_layout = QVBoxLayout(status_group)
        self.lv_dot_debug_status = QLabel('等待LV-DOT Debug数据')
        self.lv_dot_debug_status.setWordWrap(True)
        self.lv_dot_debug_status.setObjectName('vehicleDetail')
        camera_title = QLabel('我方船一号（蓝色）融合相机')
        camera_title.setObjectName('subtitle')
        self.perception_camera = CameraInsetLabel(
            '等待我方船一号相机数据'
        )
        status_layout.addWidget(camera_title)
        status_layout.addWidget(self.perception_camera)
        status_layout.addWidget(self.lv_dot_debug_status)
        status_layout.addStretch()
        splitter.addWidget(status_group)
        splitter.setSizes([220, 1000, 280])
        layout.addWidget(splitter)

        max_points.valueChanged.connect(
            self.lv_dot_debug_widget.set_max_points
        )
        trail_length.valueChanged.connect(
            self.lv_dot_debug_widget.set_trajectory_length
        )
        clear_button.clicked.connect(
            self.lv_dot_debug_widget.clear_histories
        )
        reset_button.clicked.connect(self.lv_dot_debug_widget.reset_view)
        view_mode.currentIndexChanged.connect(
            lambda _index: self.lv_dot_debug_widget.set_view_mode(
                view_mode.currentData()
            )
        )
        affiliation_color_mode.currentIndexChanged.connect(
            lambda _index: self.lv_dot_debug_widget.set_color_mode(
                affiliation_color_mode.currentData()
            )
        )
        for checkbox in self.lv_dot_debug_checks.values():
            checkbox.toggled.emit(checkbox.isChecked())

    def _build_perception_monitor(self, layout):
        splitter = QSplitter(Qt.Vertical)
        topdown_container = QWidget()
        topdown_layout = QVBoxLayout(topdown_container)
        topdown_layout.setContentsMargins(0, 0, 0, 0)
        self._build_topdown_panel(topdown_layout)
        splitter.addWidget(topdown_container)

        details_container = QWidget()
        details_layout = QVBoxLayout(details_container)
        details_layout.setContentsMargins(0, 0, 0, 0)
        status_group = QGroupBox('多源感知 Shadow Mode')
        status_layout = QGridLayout(status_group)
        self.perception_metric_labels = {}
        fields = (
            ('Ground Truth', 'ground_truth_online'),
            ('LV-DOT', 'lv_dot_online'),
            ('Target', 'target_id'),
            ('Matched Track', 'matched_track_id'),
            ('Position Error', 'position_error_m'),
            ('Velocity Error', 'velocity_error_mps'),
            ('Detection Rate', 'detection_rate'),
            ('Track Stability', 'track_stability'),
            ('Latency', 'latency_ms'),
            ('Detected Targets', 'observation_count'),
            ('Detection Frequency', 'detection_frequency_hz'),
            ('LiDAR Clusters', 'lidar_bbox_count'),
            ('Filtered Boxes', 'filtered_bbox_count'),
            ('Tracked Boxes', 'tracked_bbox_count'),
            ('Window Samples', 'window_samples'),
        )
        for index, (title, key) in enumerate(fields):
            row = index // 2
            column = (index % 2) * 2
            status_layout.addWidget(QLabel(title), row, column)
            value = QLabel('等待数据')
            value.setObjectName('captureValue')
            status_layout.addWidget(value, row, column + 1)
            self.perception_metric_labels[key] = value
        details_layout.addWidget(status_group)

        comparison_group = QGroupBox('Sensor Layer')
        comparison_layout = QVBoxLayout(comparison_group)
        layer_controls = QHBoxLayout()
        layer_controls.addWidget(QLabel('显示层：'))
        self.perception_layer_checks = {}
        layer_definitions = (
            ('Ground Truth', 'ground_truth'),
            ('LV-DOT', 'lv_dot'),
            ('UAV Camera', 'uav_camera'),
            ('Fusion', 'fusion'),
        )
        for row, (title, key) in enumerate(layer_definitions):
            checkbox = QCheckBox(title)
            checkbox.setChecked(True)
            checkbox.toggled.connect(
                lambda checked, index=row: (
                    self.perception_comparison_table.setRowHidden(
                        index, not checked
                    )
                )
            )
            self.perception_layer_checks[key] = checkbox
            layer_controls.addWidget(checkbox)
        layer_controls.addStretch()
        comparison_layout.addLayout(layer_controls)

        self.perception_comparison_table = QTableWidget(4, 11)
        self.perception_comparison_table.setHorizontalHeaderLabels([
            'Source', 'Track ID', 'Position / m', 'Velocity / m/s',
            'Sensor Source', 'Confidence', 'Position Error',
            'Velocity Error', 'Time Delta', 'Updated', 'Status',
        ])
        self._configure_table(self.perception_comparison_table)
        self.perception_comparison_table.setVerticalHeaderLabels([
            'Ground Truth', 'LV-DOT', 'UAV Camera', 'Fusion'
        ])
        for row, (source, _key) in enumerate(layer_definitions):
            self.perception_comparison_table.setItem(
                row, 0, self._item(source)
            )
        comparison_layout.addWidget(self.perception_comparison_table)
        self.perception_fusion_summary = QLabel(
            '等待 /perception/multisensor/metrics'
        )
        self.perception_fusion_summary.setObjectName('vehicleDetail')
        comparison_layout.addWidget(self.perception_fusion_summary)
        self.perception_association_label = QLabel(
            '关联关系：等待多源观测'
        )
        self.perception_association_label.setObjectName('vehicleDetail')
        comparison_layout.addWidget(self.perception_association_label)
        details_layout.addWidget(comparison_group)

        flow_group = QGroupBox('Shadow Mode 数据流')
        flow_layout = QVBoxLayout(flow_group)
        flow = QLabel(
            'UAV Camera + USV Mid-360 -> isolated LV-DOT -> '
            'TrackedObjectArray -> shadow evaluator\n'
            '围捕输入仍保持 ground_truth；此页面只显示旁路对比结果。'
        )
        flow.setWordWrap(True)
        flow_layout.addWidget(flow)
        details_layout.addWidget(flow_group)
        details_layout.addStretch()
        splitter.addWidget(details_container)
        splitter.setSizes([610, 310])
        layout.addWidget(splitter)

    def _update_topdown_selection(self, item):
        age = max(0.0, time.monotonic() - item.get('received_at', 0.0))
        dimensions = item.get('dimensions', (0.0, 0.0, 0.0))
        confidence = item.get('confidence')
        confidence_text = (
            '-' if confidence is None else '%.1f %%' % (100.0 * confidence)
        )
        self.topdown_selected_detail.setText(
            'ID：%s\n图层：%s\n来源：%s\n类别：%s\n'
            '位置：(%.2f, %.2f, %.2f) m\n'
            '速度：(%.2f, %.2f) m/s\n'
            '尺寸：(%.2f, %.2f, %.2f) m\n'
            '置信度：%s\n数据年龄：%.2f s\n'
            '动态：%s\n融合：%s'
            % (
                item.get('track_id', '-'),
                item.get('layer', '-'),
                item.get('source', '-'),
                item.get('classification_name', '-'),
                item.get('x', 0.0),
                item.get('y', 0.0),
                item.get('z', 0.0),
                item.get('vx', 0.0),
                item.get('vy', 0.0),
                dimensions[0], dimensions[1], dimensions[2],
                confidence_text,
                age,
                '是' if item.get('dynamic') else '否',
                '是' if item.get('fused') else '否',
            )
        )

    def _refresh_topdown_status(self):
        if not getattr(self, 'topdown_widget', None):
            return
        status = self.topdown_widget.point_status()
        display = self.topdown_widget.display_statistics()
        if not status:
            self.topdown_point_status.setText('点云投影：等待数据')
            return
        age = max(
            0.0, time.monotonic() - status.get('_received_at', 0.0)
        )
        online = age <= 2.5
        latency = status.get('latency_ms')
        latency_text = '-' if latency is None else '%.1f ms' % latency
        self.topdown_point_status.setText(
            '点云投影：%s\nFrame：%s\n输入：%d 点 @ %.1f Hz\n'
            '绘制：%d 点\n处理：%.2f ms\n延迟：%s\n'
            '画布：%.1f FPS / %.2f ms\n覆盖帧：%d\n'
            'TF失败：%d\n数据年龄：%.2f s'
            % (
                'ONLINE' if online else 'STALE',
                status.get('frame_id', '-'),
                int(status.get('input_points', 0)),
                float(status.get('input_rate_hz', 0.0)),
                int(status.get('draw_points', 0)),
                float(status.get('processing_ms', 0.0)),
                latency_text,
                float(display.get('render_rate_hz', 0.0)),
                float(display.get('render_ms', 0.0)),
                int(display.get('overwritten_point_frames', 0)),
                int(status.get('tf_failure_count', 0)),
                age,
            )
        )

    def _refresh_lv_dot_debug_status(self):
        widget = getattr(self, 'lv_dot_debug_widget', None)
        label = getattr(self, 'lv_dot_debug_status', None)
        if widget is None or label is None:
            return
        statistics = widget.statistics()
        counts = statistics.get('counts', {})
        status = statistics.get('status', {})
        cloud = status.get('cloud', {})
        cluster = status.get('clusters', {})
        track = status.get('tracks', {})
        dynamic = status.get('dynamic', {})
        camera_lidar = dict(self.node.camera_lidar_status)
        vision_guided = dict(self.node.vision_guided_status)
        camera_detection = dict(self.node.camera_detection_status)
        association = camera_lidar.get('last_counts', {})
        ages = statistics.get('cloud_age', {})

        def age_text(value):
            return '-' if value is None else '%.2f s' % value

        label.setText(
            'MODE: SHADOW / DISPLAY ONLY\n'
            'Fixed Frame: map\n\n'
            '画布: %.1f FPS / %.2f ms\n'
            'Raw: %d点  age=%s\n'
            'Filtered: %d点  age=%s\n\n'
            'LV-DOT cloud: %.1f Hz / %d点\n'
            'DBSCAN: %.1f Hz / %d clusters\n'
            'BBox: %d\n'
            'LiDAR Only: %d（黄）\n'
            'Camera Only: %d（蓝）\n'
            'Camera+LiDAR: %d（绿）\n'
            'Camera检测: %d  友/敌/中/未知=%d/%d/%d/%d\n'
            '身份确认/切换: %d/%d\n'
            'ROI提取: %d帧 / %.1f点\n'
            'ROI 3D框: %d  rejected=%d\n'
            'ROI耗时: %.2f ms  TF失败=%d\n'
            '标定层: 红投影=%d  绿ROI=%d  黄框=%d\n'
            '同步差: %s  ROI框内率: %s\n'
            '重投影中心差: %s  深度: %s\n'
            'TF查询: %s\n'
            'Track: %.1f Hz / %d\n'
            'Dynamic: %.1f Hz / %d\n'
            'Fusion Target: %d\n'
            'TF: %d frames (base + MID-360)\n\n'
            '控制链连接: NO\n'
            'perception_source: ground_truth'
            % (
                float(statistics.get('fps', 0.0)),
                float(statistics.get('render_ms', 0.0)),
                int(counts.get('raw', 0)), age_text(ages.get('raw')),
                int(counts.get('filtered', 0)),
                age_text(ages.get('filtered')),
                float(cloud.get('rate_hz', 0.0)),
                int(cloud.get('last_count', 0)),
                float(cluster.get('rate_hz', 0.0)),
                int(cluster.get('last_count', 0)),
                int(counts.get('bboxes', 0)),
                int(counts.get('lidar_only_bboxes', 0)),
                int(counts.get('camera_only_bboxes', 0)),
                int(counts.get('camera_lidar_fused_bboxes', 0)),
                int(camera_detection.get('detections_last', 0)),
                int(camera_detection.get('friendly_detections', 0)),
                int(camera_detection.get('hostile_detections', 0)),
                int(camera_detection.get('neutral_detections', 0)),
                int(camera_detection.get('unknown_detections', 0)),
                int(camera_detection.get('identity_confirmations', 0)),
                int(camera_detection.get('identity_switches', 0)),
                int(vision_guided.get('roi_extraction_frames', 0)),
                float(vision_guided.get('roi_average_points', 0.0)),
                int(vision_guided.get('valid_3d_bboxes', 0)),
                int(vision_guided.get('rejected_3d_bboxes', 0)),
                float(vision_guided.get('average_roi_processing_ms', 0.0)),
                int(vision_guided.get('tf_failures', 0)),
                int(counts.get('camera_projection', 0)),
                int(counts.get('calibration_roi', 0)),
                int(counts.get('calibration_bbox', 0)),
                (
                    '-' if vision_guided.get('average_sync_error_ms') is None
                    else '%.1f ms' % float(
                        vision_guided.get('average_sync_error_ms')
                    )
                ),
                (
                    '-' if vision_guided.get('roi_inside_ratio') is None
                    else '%.1f%%' % (
                        100.0 * float(vision_guided.get('roi_inside_ratio'))
                    )
                ),
                (
                    '-' if vision_guided.get(
                        'reprojection_center_error_px'
                    ) is None else '%.1f px' % float(
                        vision_guided.get('reprojection_center_error_px')
                    )
                ),
                (
                    '-' if vision_guided.get(
                        'camera_projection_depth_m'
                    ) is None else '%.2f m' % float(
                        vision_guided.get('camera_projection_depth_m')
                    )
                ),
                vision_guided.get('tf_query_mode', '-'),
                float(track.get('rate_hz', 0.0)),
                int(track.get('last_count', 0)),
                float(dynamic.get('rate_hz', 0.0)),
                int(dynamic.get('last_count', 0)),
                int(counts.get('fusion', 0)),
                int(counts.get('tf', 0)),
            )
        )

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
            'uav_camera': 1,
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

    def _vehicle_item(self, vehicle_id, color=None):
        item = self._item(
            self.VEHICLE_NAMES.get(vehicle_id, vehicle_id), color
        )
        item.setData(Qt.UserRole, vehicle_id)
        item.setToolTip(vehicle_id)
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
        if source == 'topdown_camera':
            if hasattr(self, 'perception_camera'):
                self.perception_camera.set_image(image)
        elif source == 'defense':
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
            tf_target_frame,
            tf_available,
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
            '正常' if tf_available else '缺失',
        ]
        for column, value in enumerate(values):
            if column == 0:
                self.sensor_table.setItem(
                    row, column, self._vehicle_item(vehicle)
                )
            else:
                self.sensor_table.setItem(row, column, self._item(value))
        self.sensor_table.setItem(
            row,
            10,
            self._item(
                '正常' if healthy else ('超时' if timed_out else '等待'),
                '#16834a' if healthy else '#b63737',
            ),
        )
        self.sensor_table.setToolTip(
            '%s/%s  最近消息=%d.%09d  age=%.3fs  messages=%d  TF=%s->%s'
            % (
                vehicle,
                sensor,
                last_sec,
                last_nanosec,
                age,
                messages,
                tf_target_frame or '-',
                frame_id or '-',
            )
        )

    def _toggle_mid360_preview(self, enabled):
        self.mid360_preview_button.setText(
            'RViz点云预览：%s' % ('开启' if enabled else '关闭')
        )
        self.node.set_mid360_preview(enabled)

    def _update_perception_metrics(self, metrics):
        self._touch_ros()

        def number(key, unit, digits=2):
            value = metrics.get(key)
            if value is None:
                return '-'
            return ('%%.%df %%s' % digits) % (float(value), unit)

        values = {
            'ground_truth_online': (
                'ONLINE' if metrics.get('ground_truth_online') else 'OFFLINE'
            ),
            'lv_dot_online': (
                'ONLINE' if metrics.get('lv_dot_online') else 'OFFLINE'
            ),
            'target_id': metrics.get('target_id') or '-',
            'matched_track_id': metrics.get('matched_track_id') or '-',
            'position_error_m': number('position_error_m', 'm'),
            'velocity_error_mps': number('velocity_error_mps', 'm/s'),
            'detection_rate': (
                number('detection_rate', '%', 1)
                if metrics.get('detection_rate') is None
                else '%.1f %%' % (
                    100.0 * float(metrics['detection_rate'])
                )
            ),
            'track_stability': (
                '-'
                if metrics.get('track_stability') is None
                else '%.1f %%' % (
                    100.0 * float(metrics['track_stability'])
                )
            ),
            'latency_ms': number('latency_ms', 'ms'),
            'observation_count': str(metrics.get('observation_count', 0)),
            'detection_frequency_hz': number(
                'detection_frequency_hz', 'Hz', 1
            ),
            'lidar_bbox_count': str(metrics.get('lidar_bbox_count', 0)),
            'filtered_bbox_count': str(
                metrics.get('filtered_bbox_count', 0)
            ),
            'tracked_bbox_count': str(
                metrics.get('tracked_bbox_count', 0)
            ),
            'window_samples': str(metrics.get('window_samples', 0)),
        }
        for key, value in values.items():
            label = self.perception_metric_labels.get(key)
            if label is None:
                continue
            label.setText(value)
            if key in ('ground_truth_online', 'lv_dot_online'):
                online = value == 'ONLINE'
                label.setStyleSheet(
                    'color: %s; font-weight: 700;'
                    % ('#16834a' if online else '#b63737')
                )

    def _update_perception_fusion_metrics(self, metrics):
        self._touch_ros()
        sources = metrics.get('sources', {})
        source_order = ('ground_truth', 'lv_dot', 'uav_camera', 'fusion')

        def vector_text(value, unit=''):
            if not isinstance(value, (list, tuple)) or len(value) < 2:
                return '-'
            suffix = (' ' + unit) if unit else ''
            return '(%.2f, %.2f, %.2f)%s' % (
                float(value[0]),
                float(value[1]),
                float(value[2]) if len(value) > 2 else 0.0,
                suffix,
            )

        def number(value, unit='', digits=2):
            if value is None:
                return '-'
            suffix = (' ' + unit) if unit else ''
            return ('%%.%df%%s' % digits) % (float(value), suffix)

        for row, source_name in enumerate(source_order):
            source = sources.get(source_name, {})
            online = bool(source.get('online'))
            time_delta = source.get('timestamp_delta_ms')
            if time_delta is None:
                time_delta = source.get('latency_ms')
            timestamp = source.get('timestamp')
            values = (
                source_name.replace('_', ' ').title(),
                source.get('track_id') or '-',
                vector_text(source.get('position')),
                vector_text(source.get('velocity')),
                source.get('source') or 'UNKNOWN',
                number(source.get('confidence'), '', 2),
                number(source.get('position_error_m'), 'm', 2),
                number(source.get('velocity_error_mps'), 'm/s', 2),
                number(time_delta, 'ms', 1),
                number(timestamp, 's', 3),
                'ONLINE' if online else 'OFFLINE',
            )
            for column, value in enumerate(values):
                color = None
                if column == 10:
                    color = '#16834a' if online else '#b63737'
                self.perception_comparison_table.setItem(
                    row, column, self._item(value, color)
                )

        summary = metrics.get('summary', {})
        lv_dot = summary.get('lv_dot', {})
        uav_camera = summary.get('uav_camera', {})
        fusion = summary.get('fusion', {})
        self.perception_fusion_summary.setText(
            'Shadow Mode | control source: %s | '
            'LV-DOT/Camera/Fusion mean error: %s / %s / %s | '
            'ID switches: %d / %d / %d'
            % (
                metrics.get('control_source', 'ground_truth'),
                number(lv_dot.get('mean_position_error_m'), 'm', 2),
                number(
                    uav_camera.get('mean_position_error_m'), 'm', 2
                ),
                number(fusion.get('mean_position_error_m'), 'm', 2),
                int(lv_dot.get('id_switches') or 0),
                int(uav_camera.get('id_switches') or 0),
                int(fusion.get('id_switches') or 0),
            )
        )
        association = metrics.get('association', {})
        if association:
            self.perception_association_label.setText(
                '关联关系：%s + %s -> %s | sources=%s | rate=%s'
                % (
                    association.get('lv_dot_track_id') or '-',
                    association.get('uav_camera_track_id') or '-',
                    association.get('fusion_track_id') or '-',
                    association.get('fusion_sources') or 'UNKNOWN',
                    (
                        '%.1f %%' % (
                            100.0 * float(
                                association.get('association_rate') or 0.0
                            )
                        )
                    ),
                )
            )

    def _update_multisensor_metrics(self, metrics):
        self._update_perception_fusion_metrics(metrics)

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
            if column == 0:
                self.vehicle_table.setItem(
                    row, column, self._vehicle_item(vehicle)
                )
            else:
                self.vehicle_table.setItem(
                    row, column, self._item(value, color)
                )
        self.vehicle_table.setToolTip('%s: %s' % (vehicle, status))
        role = self.capture_roles_cache.get(vehicle, {})
        fleet_row = self._fleet_row(vehicle)
        control = 'PX4 / %s' % mode if vehicle.startswith('uav_') \
            else 'Nav2 / %s' % mode
        fleet_values = [
            self.VEHICLE_NAMES.get(vehicle, vehicle),
            role.get('role_name', 'Standby'),
            'ONLINE' if online else 'OFFLINE',
            control,
        ]
        for column, value in enumerate(fleet_values):
            color = '#16834a' if column == 2 and online else None
            if column == 2 and not online:
                color = '#b63737'
            if column == 0:
                self.fleet_table.setItem(
                    fleet_row, column, self._vehicle_item(vehicle)
                )
            else:
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
        target_id = state.get('target_id') or 'enemy_ship'
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
            'target': target.get('track_id', 'enemy_ship'),
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
            'TARGET\n%s' % target.get('track_id', 'enemy_ship')
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
        vehicle_id = item.data(Qt.UserRole) or item.text().lower()
        state = self.vehicle_cache.get(vehicle_id)
        if state is None:
            return
        role = self.capture_roles_cache.get(vehicle_id, {})
        control = 'PX4' if vehicle_id.startswith('uav_') else 'Nav2'
        self.vehicle_detail.setText(
            '%s\nRole: %s\n%s: %s\nMode: %s\nPosition: (%.1f, %.1f, %.1f)\nStatus: %s'
            % (
                self.VEHICLE_NAMES.get(vehicle_id, vehicle_id),
                role.get('role_name', 'Standby'),
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
    visualization_model = TopDownVisualizationModel()
    lv_dot_debug_model = LvDotDebugModel()
    node = BaseStationGuiNode(
        signals, visualization_model, lv_dot_debug_model
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, daemon=True
    )
    spin_thread.start()

    window = BaseStationWindow(
        node, signals, visualization_model, lv_dot_debug_model
    )
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
