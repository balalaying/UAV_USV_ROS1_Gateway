"""UAV-USV Gateway protocol v1 protobuf helpers."""

import threading
import time
import uuid

from google.protobuf.timestamp_pb2 import Timestamp

from . import uav_usv_gateway_v1_pb2 as pb


SPEC_VERSION = '1.0'
SOURCE = 'uav_usv_fleet_gateway'


class V1ProtocolEncoder:
    def __init__(self, source=SOURCE, stream_epoch=None):
        self.source = str(source)
        self.stream_epoch = str(stream_epoch or uuid.uuid4().hex)
        self._sequences = {}
        self._lock = threading.Lock()

    def _wire_stream_id(self, stream_id):
        return '%s.%s' % (str(stream_id), self.stream_epoch)

    def _next_sequence(self, stream_id):
        stream_id = str(stream_id)
        with self._lock:
            value = self._sequences.get(stream_id, 0) + 1
            self._sequences[stream_id] = value
            return value

    def envelope(self, message_type, stream_id, body=None):
        message = pb.GatewayEnvelope()

        message.spec_version = SPEC_VERSION
        message.message_type = str(message_type)
        message.message_id = uuid.uuid4().hex
        message.monotonic_ns = time.monotonic_ns()
        wire_stream_id = self._wire_stream_id(stream_id)
        message.stream_id = wire_stream_id
        message.sequence = self._next_sequence(wire_stream_id)
        message.source = self.source

        timestamp = Timestamp()
        timestamp.GetCurrentTime()
        message.timestamp.CopyFrom(timestamp)

        if body is not None:
            field_name = body.DESCRIPTOR.name

            # DetectionBatch is shared by visual/lidar/radar/fused results,
            # so perception bodies are selected by message_type.
            perception_mapping = {
                'perception.visual_detections': 'visual_detections',
                'perception.lidar_detections': 'lidar_detections',
                'perception.radar_detections': 'radar_detections',
                'perception.fused_tracks': 'fused_tracks',
                'perception.lidar_pointcloud': 'lidar_pointcloud',
                'perception.radar_scan': 'radar_scan',
                'media.camera_jpeg': 'camera_frame',
            }

            target = perception_mapping.get(str(message_type))

            if target is None:
                mapping = {
                    'GatewayHello': 'gateway_hello',
                    'GatewayHeartbeat': 'gateway_heartbeat',
                    'ControlCommand': 'control_command',
                    'ControlAck': 'control_ack',
                    'ControlFeedback': 'control_feedback',
                    'ControlResult': 'control_result',
                    'MissionStatus': 'mission_status',
                    'PoseBatch': 'pose_batch',
                    'DeviceStatus': 'device_status',
                    'SystemError': 'system_error',
                }
                target = mapping.get(field_name)

            if target is None:
                raise ValueError(
                    'unsupported v1 body: %s (%s)'
                    % (field_name, message_type)
                )

            getattr(message, target).CopyFrom(body)

        return message

    def dumps(self, message_type, stream_id, body=None):
        return self.envelope(
            message_type, stream_id, body
        ).SerializeToString()
