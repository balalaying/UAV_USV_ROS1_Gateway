#!/usr/bin/env python3
"""Fleet World Model publisher.

This node is the first authoritative aggregation point for the fleet-level
state. It does not subscribe to raw camera images, point clouds or LV-DOT
debug topics. Instead, it consumes normalized ROS 2 interfaces and publishes a
single JSON world model that Qt, WebGL and future behavior managers can use as
their single source of truth.
"""

import json
import math
import time
import uuid

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from uav_usv_interfaces.msg import CaptureAssignmentArray
from uav_usv_interfaces.msg import CaptureState
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import SensorStatus
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_interfaces.msg import VehicleState
from uav_usv_mission.world_model_contract import normalize_sensor_sources
from uav_usv_mission.world_model_contract import planar_course
from uav_usv_mission.world_model_contract import quaternion_yaw
from uav_usv_mission.world_model_contract import stream_is_usable


def _stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _stamp_dict(stamp):
    return {
        'sec': int(stamp.sec),
        'nanosec': int(stamp.nanosec),
        'seconds': _stamp_to_float(stamp),
    }


def _header_dict(header):
    return {
        'stamp': _stamp_dict(header.stamp),
        'frame_id': header.frame_id,
    }


def _vector3_dict(value):
    return {
        'x': float(value.x),
        'y': float(value.y),
        'z': float(value.z),
    }


def _quaternion_dict(value):
    return {
        'x': float(value.x),
        'y': float(value.y),
        'z': float(value.z),
        'w': float(value.w),
    }


def _pose_dict(pose):
    return {
        'position': _vector3_dict(pose.position),
        'orientation': _quaternion_dict(pose.orientation),
    }


def _twist_dict(twist):
    return {
        'linear': _vector3_dict(twist.linear),
        'angular': _vector3_dict(twist.angular),
    }


def _speed_from_twist(twist):
    linear = twist.linear
    return math.sqrt(
        float(linear.x) * float(linear.x)
        + float(linear.y) * float(linear.y)
        + float(linear.z) * float(linear.z)
    )


def _uuid_to_string(value):
    raw = bytes(value.uuid)
    if raw == b'\x00' * 16:
        return ''
    return str(uuid.UUID(bytes=raw))


class FleetWorldModelNode(Node):
    VEHICLE_TYPES = {
        VehicleState.TYPE_UAV: 'UAV',
        VehicleState.TYPE_USV: 'USV',
        VehicleState.TYPE_UNKNOWN: 'UNKNOWN',
    }
    SOURCE_BITS = (
        (TrackedObject.SOURCE_LIDAR, 'USV_LIDAR'),
        (TrackedObject.SOURCE_CAMERA, 'CAMERA'),
        (TrackedObject.SOURCE_AIS, 'AIS'),
        (TrackedObject.SOURCE_FUSED, 'FUSION'),
    )
    CLASSES = {
        TrackedObject.CLASS_UNKNOWN: 'UNKNOWN',
        TrackedObject.CLASS_VESSEL: 'VESSEL',
        TrackedObject.CLASS_BUOY: 'BUOY',
        TrackedObject.CLASS_DEBRIS: 'DEBRIS',
        TrackedObject.CLASS_LANDMARK: 'LANDMARK',
    }
    AFFILIATIONS = {
        TrackedObject.AFFILIATION_UNKNOWN: 'UNKNOWN',
        TrackedObject.AFFILIATION_FRIENDLY: 'FRIENDLY',
        TrackedObject.AFFILIATION_HOSTILE: 'HOSTILE',
        TrackedObject.AFFILIATION_NEUTRAL: 'NEUTRAL',
    }
    ACK_STATUS = {
        CommandAck.STATUS_RECEIVED: 'RECEIVED',
        CommandAck.STATUS_ACCEPTED: 'ACCEPTED',
        CommandAck.STATUS_EXECUTING: 'EXECUTING',
        CommandAck.STATUS_SUCCEEDED: 'SUCCEEDED',
        CommandAck.STATUS_REJECTED: 'REJECTED',
        CommandAck.STATUS_FAILED: 'FAILED',
        CommandAck.STATUS_CANCELED: 'CANCELED',
    }

    def __init__(self):
        super().__init__('fleet_world_model')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('vehicle_state_topic', '/fleet/state')
        self.declare_parameter('sensor_status_topic', '/fleet/sensor_status')
        self.declare_parameter('command_ack_topic', '/fleet/command_ack')
        self.declare_parameter('truth_targets_topic', '/fleet/perception/targets')
        self.declare_parameter(
            'perception_source_status_topic', '/perception/source_status'
        )
        self.declare_parameter(
            'usv_tracks_topic', '/fleet/perception/usv_tracks'
        )
        self.declare_parameter(
            'fused_targets_topic', '/fleet/perception/fused_targets'
        )
        self.declare_parameter('capture_state_topic', '/capture/state')
        self.declare_parameter('capture_roles_topic', '/capture/roles')
        self.declare_parameter(
            'behavior_state_topic', '/fleet/behavior/shadow_state'
        )
        self.declare_parameter('world_model_topic', '/fleet/world_model')
        self.declare_parameter(
            'summary_topic', '/fleet/world_model_summary'
        )
        self.declare_parameter('stale_timeout_seconds', 2.5)
        self.declare_parameter('max_command_acks', 30)
        self.declare_parameter('max_tf_edges', 200)
        self.declare_parameter(
            'known_uav_ids', ['uav_01', 'uav_02', 'uav_03']
        )
        self.declare_parameter(
            'known_usv_ids', ['usv_01', 'usv_02', 'usv_03']
        )
        self.declare_parameter(
            'known_entity_ids', ['friendly_ship', 'enemy_ship']
        )

        self.map_frame = str(self.get_parameter('map_frame').value)
        rate = max(0.5, float(self.get_parameter('publish_rate_hz').value))
        self.stale_timeout = max(
            0.1, float(self.get_parameter('stale_timeout_seconds').value)
        )
        self.max_command_acks = max(
            1, int(self.get_parameter('max_command_acks').value)
        )
        self.max_tf_edges = max(1, int(self.get_parameter('max_tf_edges').value))
        self.known_uav_ids = [
            str(item) for item in self.get_parameter('known_uav_ids').value
        ]
        self.known_usv_ids = [
            str(item) for item in self.get_parameter('known_usv_ids').value
        ]
        self.known_entity_ids = [
            str(item) for item in self.get_parameter('known_entity_ids').value
        ]

        self.vehicles = {}
        self.sensors = {}
        self.command_acks = []
        self.selected_targets = None
        self.usv_tracks = None
        self.fused_targets = None
        self.perception_source_status = None
        self.capture_state = None
        self.capture_roles = None
        self.behavior_state = None
        self.tf_edges = {}
        self.static_tf_edges = set()

        self.model_pub = self.create_publisher(
            String, str(self.get_parameter('world_model_topic').value), 10
        )
        self.summary_pub = self.create_publisher(
            String, str(self.get_parameter('summary_topic').value), 10
        )

        self.create_subscription(
            VehicleState,
            str(self.get_parameter('vehicle_state_topic').value),
            self._on_vehicle_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SensorStatus,
            str(self.get_parameter('sensor_status_topic').value),
            self._on_sensor_status,
            20,
        )
        self.create_subscription(
            CommandAck,
            str(self.get_parameter('command_ack_topic').value),
            self._on_command_ack,
            30,
        )
        self.create_subscription(
            TrackedObjectArray,
            str(self.get_parameter('truth_targets_topic').value),
            self._on_selected_targets,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('perception_source_status_topic').value),
            self._on_perception_source_status,
            10,
        )
        self.create_subscription(
            TrackedObjectArray,
            str(self.get_parameter('usv_tracks_topic').value),
            self._on_usv_tracks,
            10,
        )
        self.create_subscription(
            TrackedObjectArray,
            str(self.get_parameter('fused_targets_topic').value),
            self._on_fused_targets,
            10,
        )
        self.create_subscription(
            CaptureState,
            str(self.get_parameter('capture_state_topic').value),
            self._on_capture_state,
            10,
        )
        self.create_subscription(
            CaptureAssignmentArray,
            str(self.get_parameter('capture_roles_topic').value),
            self._on_capture_roles,
            10,
        )
        behavior_qos = QoSProfile(depth=1)
        behavior_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            str(self.get_parameter('behavior_state_topic').value),
            self._on_behavior_state,
            behavior_qos,
        )
        tf_static_qos = QoSProfile(depth=50)
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(TFMessage, '/tf', self._on_tf, 50)
        self.create_subscription(
            TFMessage, '/tf_static', self._on_tf_static, tf_static_qos
        )

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Fleet World Model publishing %s and %s at %.1f Hz'
            % (
                str(self.get_parameter('world_model_topic').value),
                str(self.get_parameter('summary_topic').value),
                rate,
            )
        )

    def _on_vehicle_state(self, msg):
        if not msg.vehicle_id:
            return
        self.vehicles[msg.vehicle_id] = (msg, time.monotonic())

    def _on_sensor_status(self, msg):
        key = (msg.vehicle_id, msg.sensor_id)
        self.sensors[key] = (msg, time.monotonic())

    def _on_command_ack(self, msg):
        self.command_acks.append((msg, time.monotonic()))
        if len(self.command_acks) > self.max_command_acks:
            del self.command_acks[:-self.max_command_acks]

    def _on_selected_targets(self, msg):
        self.selected_targets = (msg, time.monotonic())

    def _on_perception_source_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self.perception_source_status = (payload, time.monotonic())

    def _on_usv_tracks(self, msg):
        self.usv_tracks = (msg, time.monotonic())

    def _on_fused_targets(self, msg):
        self.fused_targets = (msg, time.monotonic())

    def _on_capture_state(self, msg):
        self.capture_state = (msg, time.monotonic())

    def _on_capture_roles(self, msg):
        self.capture_roles = (msg, time.monotonic())

    def _on_behavior_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        if payload.get('schema_version') != 'fleet_behavior_state.v1':
            return
        self.behavior_state = (payload, time.monotonic())

    def _on_tf(self, msg):
        self._store_tf(msg, is_static=False)

    def _on_tf_static(self, msg):
        self._store_tf(msg, is_static=True)

    def _store_tf(self, msg, is_static):
        now = time.monotonic()
        for transform in msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self.tf_edges[key] = (transform, now)
            if is_static:
                self.static_tf_edges.add(key)
            else:
                self.static_tf_edges.discard(key)
        if len(self.tf_edges) > self.max_tf_edges:
            ordered = sorted(
                self.tf_edges.items(), key=lambda item: item[1][1]
            )
            for key, _ in ordered[:len(self.tf_edges) - self.max_tf_edges]:
                self.tf_edges.pop(key, None)
                self.static_tf_edges.discard(key)

    def _freshness(self, received_at, now):
        age = max(0.0, now - received_at)
        return {
            'age_seconds': age,
            'stale': age > self.stale_timeout,
        }

    def _vehicle_dict(self, msg, received_at, now):
        result = {
            'id': msg.vehicle_id,
            'type': self.VEHICLE_TYPES.get(msg.vehicle_type, 'UNKNOWN'),
            'state_source': 'vehicle_state',
            'online': bool(msg.online),
            'armed': bool(msg.armed),
            'mode': msg.mode,
            'pose': _pose_dict(msg.pose),
            'velocity': _twist_dict(msg.twist),
            'speed_mps': _speed_from_twist(msg.twist),
            'battery_percent': float(msg.battery_percent),
            'active_command_id': msg.active_command_id,
            'health': {
                'status_text': msg.status_text,
            },
            'header': _header_dict(msg.header),
        }
        result.update(self._freshness(received_at, now))
        return result

    def _tf_pose_for_child(self, child_frame):
        entry = self.tf_edges.get((self.map_frame, child_frame))
        if entry is None:
            return None
        transform, received_at = entry
        pose = {
            'position': _vector3_dict(transform.transform.translation),
            'orientation': _quaternion_dict(transform.transform.rotation),
        }
        return pose, transform.header.stamp, received_at

    def _tf_vehicle_dict(self, vehicle_id, vehicle_type, now):
        tf_entry = self._tf_pose_for_child(vehicle_id + '/base_link')
        if tf_entry is None:
            return None
        pose, stamp, received_at = tf_entry
        result = {
            'id': vehicle_id,
            'type': vehicle_type,
            'state_source': 'tf_only',
            'online': False,
            'armed': False,
            'mode': 'TF_ONLY',
            'pose': pose,
            'velocity': {
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            },
            'speed_mps': 0.0,
            'battery_percent': -1.0,
            'active_command_id': '',
            'health': {
                'status_text': (
                    'pose available from map TF; no VehicleState received'
                ),
            },
            'header': {
                'stamp': _stamp_dict(stamp),
                'frame_id': self.map_frame,
            },
        }
        result.update(self._freshness(received_at, now))
        return result

    def _entity_dict(self, entity_id, now):
        tf_entry = self._tf_pose_for_child(entity_id + '/base_link')
        if tf_entry is None:
            return None
        pose, stamp, received_at = tf_entry
        lowered = entity_id.lower()
        if 'enemy' in lowered or 'hostile' in lowered:
            affiliation = 'HOSTILE'
            entity_type = 'TARGET'
        elif 'friendly' in lowered or 'protect' in lowered:
            affiliation = 'FRIENDLY'
            entity_type = 'PROTECTED_ASSET'
        else:
            affiliation = 'UNKNOWN'
            entity_type = 'MISSION_ENTITY'
        result = {
            'id': entity_id,
            'type': entity_type,
            'state_source': 'tf_only',
            'pose': pose,
            'velocity': {
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            },
            'speed_mps': 0.0,
            'classification': 'VESSEL',
            'affiliation': affiliation,
            'confidence': 1.0,
            'source': ['TF'],
            'header': {
                'stamp': _stamp_dict(stamp),
                'frame_id': self.map_frame,
            },
        }
        result.update(self._freshness(received_at, now))
        return result

    def _target_sources(self, source_mask):
        result = []
        for bit, name in self.SOURCE_BITS:
            if int(source_mask) & int(bit):
                result.append(name)
        return result or ['UNKNOWN']

    def _target_dict(
        self, obj, array_stamp, array_frame, now, source_stream
    ):
        last_update = _stamp_to_float(obj.last_update)
        stamp_age = None
        if last_update > 0.0:
            stamp_age = max(0.0, self.get_clock().now().nanoseconds * 1e-9 - last_update)
        class_name = obj.class_name or self.CLASSES.get(
            obj.classification, 'UNKNOWN'
        )
        pose = _pose_dict(obj.pose.pose)
        velocity = _twist_dict(obj.twist.twist)
        speed = _speed_from_twist(obj.twist.twist)
        orientation = obj.pose.pose.orientation
        heading = quaternion_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        course = planar_course(
            float(obj.twist.twist.linear.x),
            float(obj.twist.twist.linear.y),
            fallback=heading,
        )
        generic_sources = self._target_sources(obj.source_mask)
        return {
            'id': obj.track_id or _uuid_to_string(obj.uuid),
            'uuid': _uuid_to_string(obj.uuid),
            'frame_id': array_frame,
            'coordinate_valid': array_frame == self.map_frame,
            'position': pose['position'],
            'pose': pose,
            'pose_covariance': [
                float(value) for value in obj.pose.covariance
            ],
            'velocity': velocity,
            'twist_covariance': [
                float(value) for value in obj.twist.covariance
            ],
            'speed_mps': speed,
            'heading_rad': heading,
            'course_over_ground_rad': course,
            'motion_state': 'MOVING' if speed >= 0.2 else 'STATIONARY',
            'dimensions': _vector3_dict(obj.dimensions),
            'class': class_name,
            'classification': self.CLASSES.get(
                obj.classification, 'UNKNOWN'
            ),
            'class_confidence': float(obj.class_confidence),
            'affiliation': self.AFFILIATIONS.get(
                obj.affiliation, 'UNKNOWN'
            ),
            'affiliation_confidence': float(obj.affiliation_confidence),
            'confidence': float(obj.confidence),
            'source': generic_sources,
            'source_details': normalize_sensor_sources(
                obj.sensor_source, generic_sources
            ),
            'source_mask': int(obj.source_mask),
            'sensor_source': obj.sensor_source,
            'source_stream': source_stream,
            'mmsi': int(obj.mmsi),
            'first_seen': _stamp_dict(obj.first_seen),
            'last_update': _stamp_dict(obj.last_update),
            'timestamp': (
                _stamp_dict(obj.last_update)
                if last_update > 0.0 else _stamp_dict(array_stamp)
            ),
            'stamp_age_seconds': stamp_age,
            'header_stamp': _stamp_dict(array_stamp),
            'bbox_point_count': int(obj.bbox_point_count),
            'association_score': float(obj.association_score),
        }

    def _target_array(self, entry, now, source_stream):
        if entry is None:
            return {
                'online': False,
                'age_seconds': None,
                'stale': True,
                'frame_valid': False,
                'usable': False,
                'objects': [],
            }
        msg, received_at = entry
        freshness = self._freshness(received_at, now)
        result = {
            'online': True,
            'age_seconds': freshness['age_seconds'],
            'stale': freshness['stale'],
            'header': _header_dict(msg.header),
            'objects': [
                self._target_dict(
                    obj,
                    msg.header.stamp,
                    msg.header.frame_id,
                    now,
                    source_stream,
                )
                for obj in msg.objects
            ],
        }
        result['frame_valid'] = msg.header.frame_id == self.map_frame
        result['usable'] = stream_is_usable(result, self.map_frame)
        return result

    def _xy_distance(self, lhs, rhs):
        return math.hypot(
            float(lhs.get('x', 0.0)) - float(rhs.get('x', 0.0)),
            float(lhs.get('y', 0.0)) - float(rhs.get('y', 0.0)),
        )

    def _tf_position(self, child_frame, parent_frame=None):
        parent = parent_frame or self.map_frame
        entry = self.tf_edges.get((parent, child_frame))
        if entry is None:
            return None
        transform, _ = entry
        return _vector3_dict(transform.transform.translation)

    def _prediction_dicts(self, targets):
        horizons = (2.0, 5.0, 10.0)
        predictions = []
        for target in targets:
            pose = target.get('pose', {})
            position = pose.get('position', {})
            orientation = pose.get('orientation', {})
            velocity = target.get('velocity', {}).get('linear', {})
            target_id = target.get('id') or target.get('uuid') or 'unknown'
            confidence = max(0.0, min(1.0, float(target.get('confidence', 0.0))))
            track_predictions = []
            for horizon in horizons:
                track_predictions.append({
                    'horizon_seconds': horizon,
                    'pose': {
                        'position': {
                            'x': float(position.get('x', 0.0))
                            + float(velocity.get('x', 0.0)) * horizon,
                            'y': float(position.get('y', 0.0))
                            + float(velocity.get('y', 0.0)) * horizon,
                            'z': float(position.get('z', 0.0))
                            + float(velocity.get('z', 0.0)) * horizon,
                        },
                        'orientation': orientation,
                    },
                    'confidence': confidence * max(0.2, 1.0 - horizon * 0.06),
                })
            predictions.append({
                'target_id': target_id,
                'frame_id': self.map_frame,
                'model': 'constant_velocity_v1',
                'source_stream': target.get('source_stream', 'unknown'),
                'speed_mps': float(target.get('speed_mps', 0.0)),
                'points': track_predictions,
            })
        return predictions

    def _threat_dicts(self, targets):
        friendly_position = (
            self._tf_position('friendly_ship/base_link')
            or self._tf_position('FRIENDLY_SHIP/base_link')
            or self._tf_position('shore_command_base/base_link')
            or self._tf_position('friendly_ship')
            or self._tf_position('FRIENDLY_SHIP')
            or self._tf_position('shore_command_base')
        )
        threats = []
        for target in targets:
            target_id = target.get('id') or target.get('uuid') or 'unknown'
            target_key = target_id.lower()
            affiliation = target.get('affiliation', 'UNKNOWN')
            classification = target.get('classification') or target.get('class')
            confidence = max(0.0, min(1.0, float(target.get('confidence', 0.0))))
            speed = float(target.get('speed_mps', 0.0))
            position = target.get('pose', {}).get('position', {})

            score = 0.0
            reasons = []
            if affiliation == 'HOSTILE':
                score += 0.45
                reasons.append('hostile_affiliation')
            if 'enemy' in target_key or 'hostile' in target_key:
                score += 0.25
                reasons.append('hostile_name_hint')
            if classification == 'VESSEL':
                score += 0.10
                reasons.append('vessel_class')
            if speed > 0.3:
                score += min(0.15, speed / 6.0 * 0.15)
                reasons.append('moving_target')
            if confidence > 0.0:
                score += min(0.15, confidence * 0.15)

            distance_to_friendly = None
            if friendly_position is not None:
                distance_to_friendly = self._xy_distance(
                    position, friendly_position
                )
                if distance_to_friendly < 50.0:
                    score += 0.25
                    reasons.append('close_to_friendly_ship')
                elif distance_to_friendly < 120.0:
                    score += 0.15
                    reasons.append('approaching_friendly_zone')
                elif distance_to_friendly < 250.0:
                    score += 0.05
                    reasons.append('within_maritime_watch_area')

            score = max(0.0, min(1.0, score))
            if score >= 0.8:
                level = 'CRITICAL'
            elif score >= 0.55:
                level = 'HIGH'
            elif score >= 0.25:
                level = 'MEDIUM'
            else:
                level = 'LOW'
            threats.append({
                'target_id': target_id,
                'frame_id': self.map_frame,
                'threat_level': level,
                'score': score,
                'model': 'heuristic_v1',
                'reasons': reasons,
                'distance_to_friendly_ship_m': distance_to_friendly,
                'position': position,
                'speed_mps': speed,
                'confidence': confidence,
            })
        threats.sort(key=lambda item: item['score'], reverse=True)
        return threats

    def _obstacle_dicts(self, targets):
        obstacles = []
        obstacle_classes = {'BUOY', 'DEBRIS', 'LANDMARK', 'UNKNOWN'}
        for target in targets:
            classification = target.get('classification') or target.get('class')
            target_id = target.get('id') or target.get('uuid') or 'unknown'
            target_key = target_id.lower()
            if classification == 'VESSEL':
                continue
            if (
                classification not in obstacle_classes
                and 'buoy' not in target_key
                and 'obstacle' not in target_key
            ):
                continue
            obstacles.append({
                'id': target_id,
                'frame_id': self.map_frame,
                'classification': classification,
                'pose': target.get('pose', {}),
                'dimensions': target.get('dimensions', {}),
                'confidence': float(target.get('confidence', 0.0)),
                'source': target.get('source', []),
                'source_stream': target.get('source_stream', 'unknown'),
            })
        return obstacles

    def _sensor_dict(self, msg, received_at, now):
        result = {
            'vehicle_id': msg.vehicle_id,
            'sensor_id': msg.sensor_id,
            'uplink_topic': msg.uplink_topic,
            'message_type': msg.message_type,
            'frame_id': msg.frame_id,
            'last_message_time': _stamp_dict(msg.last_message_time),
            'rate_hz': float(msg.measured_rate_hz),
            'age_seconds_reported': float(msg.age_seconds),
            'latency_seconds': float(msg.latency_seconds),
            'processing_time_ms': float(msg.processing_time_ms),
            'point_count': int(msg.point_count),
            'total_messages': int(msg.total_messages),
            'total_bytes': int(msg.total_bytes),
            'dropped_messages': int(msg.dropped_messages),
            'healthy': bool(msg.healthy),
            'timed_out': bool(msg.timed_out),
            'tf_target_frame': msg.tf_target_frame,
            'tf_available': bool(msg.tf_available),
            'header': _header_dict(msg.header),
        }
        result.update(self._freshness(received_at, now))
        return result

    def _perception_source_dict(self, now):
        if self.perception_source_status is None:
            return {
                'source': 'UNKNOWN',
                'topic': '',
                'online': False,
                'track_count': 0,
                'age_seconds': None,
                'stale': True,
            }
        payload, received_at = self.perception_source_status
        result = dict(payload)
        result.update(self._freshness(received_at, now))
        result['source'] = str(
            result.get('source') or 'UNKNOWN'
        ).upper()
        result['online'] = bool(result.get('online')) and not result['stale']
        return result

    def _mission_dict(self, now):
        state = None
        if self.capture_state is not None:
            msg, received_at = self.capture_state
            state = {
                'type': 'dynamic_capture',
                'state': msg.state_name,
                'state_id': int(msg.state),
                'target_id': msg.target_id,
                'reason': msg.reason,
                'configured_uavs': int(msg.configured_uavs),
                'configured_usvs': int(msg.configured_usvs),
                'active_uavs': int(msg.active_uavs),
                'active_usvs': int(msg.active_usvs),
                'allocation_generation': int(msg.allocation_generation),
                'degraded': bool(msg.degraded),
                'header': _header_dict(msg.header),
            }
            state.update(self._freshness(received_at, now))
        roles = None
        if self.capture_roles is not None:
            msg, received_at = self.capture_roles
            roles = {
                'target_id': msg.target_id,
                'capture_center': _vector3_dict(msg.capture_center),
                'capture_radius': float(msg.capture_radius),
                'generation': int(msg.generation),
                'assignments': [],
                'header': _header_dict(msg.header),
            }
            roles.update(self._freshness(received_at, now))
            for item in msg.assignments:
                roles['assignments'].append({
                    'vehicle_id': item.vehicle_id,
                    'vehicle_type': self.VEHICLE_TYPES.get(
                        item.vehicle_type, 'UNKNOWN'
                    ),
                    'role': item.role_name,
                    'role_type': int(item.role_type),
                    'active': bool(item.active),
                    'status': item.status,
                    'goal': _pose_dict(item.target_pose),
                    'assignment_cost': float(item.assignment_cost),
                    'estimated_time_to_goal': float(
                        getattr(item, 'estimated_time_to_goal', 0.0)
                    ),
                })
        behavior = None
        if self.behavior_state is not None:
            payload, received_at = self.behavior_state
            behavior = dict(payload)
            behavior.update(self._freshness(received_at, now))
        return {
            'capture': state,
            'capture_roles': roles,
            'behavior': behavior,
        }

    def _communication_dict(self, now):
        acks = []
        for msg, received_at in self.command_acks:
            item = {
                'command_id': msg.command_id,
                'vehicle_id': msg.vehicle_id,
                'status': self.ACK_STATUS.get(msg.status, 'UNKNOWN'),
                'status_id': int(msg.status),
                'progress': float(msg.progress),
                'message': msg.message,
                'header': _header_dict(msg.header),
            }
            item.update(self._freshness(received_at, now))
            acks.append(item)
        return {
            'recent_command_acks': acks,
        }

    def _tf_dict(self, now):
        edges = []
        for (parent, child), (transform, received_at) in sorted(
            self.tf_edges.items()
        ):
            key = (parent, child)
            item = {
                'parent_frame': parent,
                'child_frame': child,
                'static': key in self.static_tf_edges,
                'translation': _vector3_dict(transform.transform.translation),
                'rotation': _quaternion_dict(transform.transform.rotation),
                'header': _header_dict(transform.header),
            }
            if key in self.static_tf_edges:
                item.update({'age_seconds': 0.0, 'stale': False})
            else:
                item.update(self._freshness(received_at, now))
            edges.append(item)
        return {
            'edge_count': len(edges),
            'edges': edges,
        }

    def _build_model(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().to_msg()
        fleet = {'uav': [], 'usv': [], 'unknown': []}
        vehicles_with_state = set()
        for vehicle_id, (msg, received_at) in sorted(self.vehicles.items()):
            vehicles_with_state.add(vehicle_id)
            item = self._vehicle_dict(msg, received_at, now)
            if msg.vehicle_type == VehicleState.TYPE_UAV:
                fleet['uav'].append(item)
            elif msg.vehicle_type == VehicleState.TYPE_USV:
                fleet['usv'].append(item)
            else:
                fleet['unknown'].append(item)

        for vehicle_id in self.known_uav_ids:
            if vehicle_id in vehicles_with_state:
                continue
            item = self._tf_vehicle_dict(vehicle_id, 'UAV', now)
            if item is not None:
                fleet['uav'].append(item)
        for vehicle_id in self.known_usv_ids:
            if vehicle_id in vehicles_with_state:
                continue
            item = self._tf_vehicle_dict(vehicle_id, 'USV', now)
            if item is not None:
                fleet['usv'].append(item)

        fleet['uav'].sort(key=lambda item: item['id'])
        fleet['usv'].sort(key=lambda item: item['id'])
        fleet['unknown'].sort(key=lambda item: item['id'])

        entities = []
        for entity_id in self.known_entity_ids:
            item = self._entity_dict(entity_id, now)
            if item is not None:
                entities.append(item)
        entities.sort(key=lambda item: item['id'])

        sensors_by_vehicle = {}
        for (vehicle_id, sensor_id), (msg, received_at) in sorted(
            self.sensors.items()
        ):
            sensors_by_vehicle.setdefault(vehicle_id, {})[sensor_id] = (
                self._sensor_dict(msg, received_at, now)
            )

        fused = self._target_array(self.fused_targets, now, 'fused_targets')
        usv_tracks = self._target_array(self.usv_tracks, now, 'usv_tracks')
        # ``truth_targets`` is the configured input topic.  The cached
        # message is deliberately named ``selected_targets`` because it is
        # the source currently selected by the perception mux.
        truth = self._target_array(
            self.selected_targets, now, 'ground_truth'
        )
        primary_targets = (
            fused['objects'] if fused['objects'] else truth['objects']
        )
        predictions = self._prediction_dicts(primary_targets)
        threats = self._threat_dicts(primary_targets)
        obstacles = self._obstacle_dicts(primary_targets)

        model = {
            'schema_version': 'fleet_world_model.v1',
            'world_time': _stamp_dict(ros_now),
            'map_frame': self.map_frame,
            'fleet': fleet,
            'entities': entities,
            'targets': primary_targets,
            'predictions': predictions,
            'threats': threats,
            'obstacles': obstacles,
            'perception': {
                'primary_source': (
                    'fused_targets' if fused['objects'] else 'ground_truth'
                ),
                'fused_targets': fused,
                'usv_tracks': usv_tracks,
                'ground_truth': truth,
            },
            'sensors': sensors_by_vehicle,
            'mission': self._mission_dict(now),
            'communication': self._communication_dict(now),
            'tf': self._tf_dict(now),
            'environment': {
                'world': 'heterogeneous_332',
                'frame': self.map_frame,
            },
            'health': {
                'vehicle_state_count': len(self.vehicles),
                'vehicle_count': (
                    len(fleet['uav']) + len(fleet['usv'])
                    + len(fleet['unknown'])
                ),
                'uav_count': len(fleet['uav']),
                'usv_count': len(fleet['usv']),
                'entity_count': len(entities),
                'target_count': len(primary_targets),
                'prediction_count': len(predictions),
                'threat_count': len(threats),
                'obstacle_count': len(obstacles),
                'sensor_count': len(self.sensors),
                'behavior_state_online': self.behavior_state is not None,
                'behavior_state_stale': (
                    self.behavior_state is None
                    or now - self.behavior_state[1] > self.stale_timeout
                ),
                'stale_timeout_seconds': self.stale_timeout,
            },
        }
        return model

    def _publish(self):
        model = self._build_model()
        payload = json.dumps(model, ensure_ascii=False, separators=(',', ':'))
        msg = String()
        msg.data = payload
        self.model_pub.publish(msg)

        summary = {
            'schema_version': 'fleet_world_model.summary.v1',
            'world_model_schema_version': model['schema_version'],
            'world_time': model['world_time'],
            'map_frame': model['map_frame'],
            'uav_count': model['health']['uav_count'],
            'usv_count': model['health']['usv_count'],
            'entity_count': model['health']['entity_count'],
            'target_count': model['health']['target_count'],
            'threat_count': model['health']['threat_count'],
            'obstacle_count': model['health']['obstacle_count'],
            'sensor_count': model['health']['sensor_count'],
            'primary_source': model['perception']['primary_source'],
            'mission_state': (
                model['mission']['capture']['state']
                if model['mission']['capture'] else 'UNKNOWN'
            ),
            'behavior_state': (
                model['mission']['behavior']['behavior']
                if model['mission']['behavior'] else 'WAITING'
            ),
            'tf_edge_count': model['tf']['edge_count'],
        }
        summary_msg = String()
        summary_msg.data = json.dumps(
            summary, ensure_ascii=False, separators=(',', ':')
        )
        self.summary_pub.publish(summary_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetWorldModelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
