#!/usr/bin/env python3
"""Detect simulation vessels in a USV camera stream without control output."""

import json
import math
from pathlib import Path
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_usv_interfaces.msg import AffiliatedDetection2D
from uav_usv_interfaces.msg import AffiliatedDetection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

# Support both colcon symlink-install (this file resolves into adapters/) and a
# regular install (the helper is copied beside this executable).
for _module_dir in (
    Path(sys.argv[0]).resolve().parent,
    Path(__file__).resolve().parents[1] / 'vision_guided',
):
    if str(_module_dir) not in sys.path:
        sys.path.insert(0, str(_module_dir))

from vision_guided_core import AFFILIATION_FRIENDLY
from vision_guided_core import AFFILIATION_HOSTILE
from vision_guided_core import AFFILIATION_NEUTRAL
from vision_guided_core import AFFILIATION_UNKNOWN
from vision_guided_core import AffiliationTemporalFilter


@dataclass
class VisualCandidate:
    rectangle: tuple
    class_name: str
    class_confidence: float
    affiliation: int
    affiliation_confidence: float
    marker_name: str


class DetectorBackend(ABC):
    """Replaceable camera detector contract; it has no ROS dependency."""

    @abstractmethod
    def detect(self, image):
        raise NotImplementedError


class FutureYoloDetectorBackend(DetectorBackend):
    """Explicit backend seam for a future real detector implementation."""

    def detect(self, image):
        raise RuntimeError(
            'yolo backend is not installed; use simulation_marker'
        )


class SimulationMarkerDetectorBackend(DetectorBackend):
    def __init__(self, min_pixels, padding, maximum_center_y_ratio):
        self.min_pixels = min_pixels
        self.padding = padding
        self.maximum_center_y_ratio = maximum_center_y_ratio

    def detect(self, image):
        return detect_affiliated_candidates(
            image,
            self.min_pixels,
            self.padding,
            self.maximum_center_y_ratio,
        )


def image_to_bgr(message):
    """Decode the uncompressed encodings used by the Gazebo bridge."""
    channels = {
        'mono8': 1,
        'rgb8': 3,
        'bgr8': 3,
        'rgba8': 4,
        'bgra8': 4,
    }.get(message.encoding)
    if channels is None:
        raise ValueError('unsupported image encoding: ' + message.encoding)
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        int(message.height), int(message.step)
    )
    image = rows[:, :int(message.width) * channels].reshape(
        int(message.height), int(message.width), channels
    )
    if message.encoding == 'mono8':
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if message.encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if message.encoding == 'rgba8':
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if message.encoding == 'bgra8':
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def detect_vessel_candidates(
    image,
    min_pixels=2,
    padding=6,
    maximum_center_y_ratio=0.88,
):
    """Return colored navigation-light candidates as image rectangles."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = (
        cv2.inRange(hsv, (35, 55, 55), (95, 255, 255)),  # green lamp
        cv2.inRange(hsv, (0, 85, 55), (14, 255, 255)),   # red hull/lamp
        cv2.inRange(hsv, (165, 85, 55), (179, 255, 255)),
        # The simulation target has a dark-blue hull. Requiring saturation
        # separates it from the pale, low-saturation sky and wave surface.
        cv2.inRange(hsv, (90, 60, 20), (125, 255, 225)),
    )
    height, width = image.shape[:2]
    candidates = []
    # Preserve distant 2-4 px identity plates; a 3x3 opening erased them.
    kernel = np.ones((2, 2), dtype=np.uint8)
    # Keep colour classes separate during connected-component extraction. A
    # blue water background must not swallow a small red/green vessel light.
    for mask in masks:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, box_width, box_height = cv2.boundingRect(contour)
            pixel_count = max(area, float(box_width * box_height))
            if pixel_count < float(min_pixels):
                continue
            # Reject GUI-like full-frame colors and underwater reflections.
            if box_width > 0.35 * width or box_height > 0.45 * height:
                continue
            center_y = y + 0.5 * box_height
            if center_y > float(maximum_center_y_ratio) * height:
                continue
            pad_x = max(int(padding), box_width * 2)
            pad_y = max(int(padding), box_height * 2)
            left = max(0, x - pad_x)
            top = max(0, y - pad_y)
            right = min(width - 1, x + box_width + pad_x)
            bottom = min(height - 1, y + box_height + pad_y)
            score = min(0.98, 0.58 + 0.08 * math.log1p(pixel_count))
            candidates.append((left, top, right, bottom, score))
    candidates.sort(key=lambda item: item[4], reverse=True)
    output = []
    for candidate in candidates:
        center = (
            0.5 * (candidate[0] + candidate[2]),
            0.5 * (candidate[1] + candidate[3]),
        )
        if any(
            math.hypot(
                center[0] - 0.5 * (old[0] + old[2]),
                center[1] - 0.5 * (old[1] + old[3]),
            ) < 8.0
            for old in output
        ):
            continue
        output.append(candidate)
    return output[:8]


def detect_affiliated_candidates(
    image, min_pixels=2, padding=6, maximum_center_y_ratio=0.88
):
    """Detect explicit local identity plates; never reads Gazebo truth."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    definitions = (
        ('friendly_blue', AFFILIATION_FRIENDLY, (98, 130, 70),
         (125, 255, 255)),
        ('hostile_red_low', AFFILIATION_HOSTILE, (0, 145, 80),
         (12, 255, 255)),
        ('hostile_red_high', AFFILIATION_HOSTILE, (170, 145, 80),
         (179, 255, 255)),
        ('neutral_green', AFFILIATION_NEUTRAL, (42, 125, 65),
         (88, 255, 255)),
    )
    height, width = image.shape[:2]
    # Preserve distant 2-4 px identity plates; a 3x3 opening erased them.
    kernel = np.ones((2, 2), dtype=np.uint8)
    candidates = []
    for name, affiliation, lower, upper in definitions:
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, box_width, box_height = cv2.boundingRect(contour)
            support = max(area, float(box_width * box_height))
            if support < float(min_pixels):
                continue
            if box_width > 0.25 * width or box_height > 0.35 * height:
                continue
            if y + 0.5 * box_height > maximum_center_y_ratio * height:
                continue
            aspect = box_width / max(1.0, float(box_height))
            if aspect < 0.12 or aspect > 8.0:
                continue
            expand_x = max(int(padding), int(3.5 * box_width))
            expand_y = max(int(padding), int(3.0 * box_height))
            rectangle = (
                max(0, x - expand_x), max(0, y - expand_y),
                min(width - 1, x + box_width + expand_x),
                min(height - 1, y + box_height + expand_y),
            )
            confidence = min(0.99, 0.62 + 0.06 * math.log1p(support))
            candidates.append(VisualCandidate(
                rectangle=rectangle,
                class_name='vessel',
                class_confidence=confidence,
                affiliation=affiliation,
                affiliation_confidence=confidence,
                marker_name=name,
            ))
    candidates.sort(key=lambda value: value.affiliation_confidence, reverse=True)
    output = []
    for candidate in candidates:
        left, top, right, bottom = candidate.rectangle
        center = (0.5 * (left + right), 0.5 * (top + bottom))
        def same_target(old):
            old_left, old_top, old_right, old_bottom = old.rectangle
            intersection = max(0.0, min(right, old_right) - max(left, old_left))
            intersection *= max(
                0.0, min(bottom, old_bottom) - max(top, old_top)
            )
            area = max(1.0, (right - left) * (bottom - top))
            old_area = max(
                1.0, (old_right - old_left) * (old_bottom - old_top)
            )
            overlap = intersection / min(area, old_area)
            center_distance = math.hypot(
                center[0] - 0.5 * (old_left + old_right),
                center[1] - 0.5 * (old_top + old_bottom),
            )
            return (
                candidate.affiliation == old.affiliation
                and (overlap >= 0.30 or center_distance < 14.0)
            )

        if any(same_target(old) for old in output):
            continue
        output.append(candidate)
    return output[:12]


def make_detection(message, rectangle, detection_id):
    left, top, right, bottom, score = rectangle
    detection = Detection2D()
    detection.header = message.header
    detection.id = 'camera_vessel_%03d' % detection_id
    detection.bbox.center.position.x = 0.5 * (left + right)
    detection.bbox.center.position.y = 0.5 * (top + bottom)
    detection.bbox.size_x = float(max(1, right - left))
    detection.bbox.size_y = float(max(1, bottom - top))
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = 'vessel'
    hypothesis.hypothesis.score = float(score)
    detection.results.append(hypothesis)
    return detection


def affiliated_message(header, candidate, detection_id, affiliation, confidence):
    left, top, right, bottom = candidate.rectangle
    output = AffiliatedDetection2D()
    output.header = header
    output.detection_id = detection_id
    output.center_x = 0.5 * (left + right)
    output.center_y = 0.5 * (top + bottom)
    output.size_x = float(max(1, right - left))
    output.size_y = float(max(1, bottom - top))
    output.class_name = candidate.class_name
    output.class_confidence = float(candidate.class_confidence)
    output.affiliation = int(affiliation)
    output.affiliation_confidence = float(confidence)
    return output


class UsvCameraDetectionNode(Node):
    def __init__(self):
        super().__init__('usv_camera_detection_node')
        self.declare_parameter(
            'image_topic', '/fleet/uplink/usv_01/camera/image_raw'
        )
        self.declare_parameter(
            'detections_topic', '/perception/usv_01/camera/detections'
        )
        self.declare_parameter(
            'debug_image_topic',
            '/perception/usv_01/camera/detections/image',
        )
        self.declare_parameter(
            'status_topic', '/perception/usv_01/camera/detection_status'
        )
        self.declare_parameter(
            'affiliated_detections_topic',
            '/perception/usv_01/camera/affiliated_detections',
        )
        self.declare_parameter('detector_backend', 'simulation_marker')
        self.declare_parameter('enable_affiliation_filter', True)
        self.declare_parameter('min_pixels', 2)
        self.declare_parameter('padding_pixels', 6)
        self.declare_parameter('maximum_center_y_ratio', 0.88)
        self.declare_parameter('max_rate_hz', 20.0)
        self.declare_parameter('visual_track_pixel_gate', 64.0)
        self.declare_parameter('affiliation_history_size', 7)
        self.declare_parameter('affiliation_confirmation_frames', 3)
        self.declare_parameter('affiliation_switch_frames', 4)
        self.declare_parameter('affiliation_hold_seconds', 1.0)
        self.declare_parameter('affiliation_unknown_timeout', 2.0)
        self.declare_parameter('affiliation_min_confidence', 0.55)
        self.min_pixels = max(1, int(self.get_parameter('min_pixels').value))
        self.padding = max(
            0, int(self.get_parameter('padding_pixels').value)
        )
        self.maximum_center_y_ratio = min(1.0, max(
            0.1,
            float(self.get_parameter('maximum_center_y_ratio').value),
        ))
        self.minimum_period = 1.0 / max(
            0.1, float(self.get_parameter('max_rate_hz').value)
        )
        self.last_processed = 0.0
        self.total_frames = 0
        self.total_detections = 0
        self.last_latency_ms = 0.0
        self.last_count = 0
        self.last_status_wall = 0.0
        self.enable_affiliation_filter = bool(
            self.get_parameter('enable_affiliation_filter').value
        )
        self.visual_tracks = {}
        self.next_visual_track = 1
        self.visual_track_gate = float(
            self.get_parameter('visual_track_pixel_gate').value
        )
        backend_name = str(self.get_parameter('detector_backend').value)
        if backend_name != 'simulation_marker':
            raise RuntimeError('unsupported detector_backend: ' + backend_name)
        self.backend = SimulationMarkerDetectorBackend(
            self.min_pixels, self.padding, self.maximum_center_y_ratio
        )
        self.affiliation_filter = AffiliationTemporalFilter(
            history_size=int(self.get_parameter('affiliation_history_size').value),
            confirmation_frames=int(self.get_parameter(
                'affiliation_confirmation_frames').value),
            switch_frames=int(self.get_parameter('affiliation_switch_frames').value),
            hold_seconds=float(self.get_parameter('affiliation_hold_seconds').value),
            unknown_timeout=float(self.get_parameter(
                'affiliation_unknown_timeout').value),
            min_confidence=float(self.get_parameter(
                'affiliation_min_confidence').value),
        )
        self.publisher = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
            qos_profile_sensor_data,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            qos_profile_sensor_data,
        )
        self.affiliated_publisher = self.create_publisher(
            AffiliatedDetection2DArray,
            str(self.get_parameter('affiliated_detections_topic').value),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info('USV camera detector ready (Shadow Mode)')

    def _assign_visual_ids(self, candidates):
        now = time.monotonic()
        assigned = set()
        output = []
        for candidate in candidates:
            left, top, right, bottom = candidate.rectangle
            center = (0.5 * (left + right), 0.5 * (top + bottom))
            best_id = None
            best_distance = self.visual_track_gate
            for track_id, state in self.visual_tracks.items():
                if track_id in assigned or now - state['updated'] > 2.0:
                    continue
                distance = math.hypot(
                    center[0] - state['center'][0], center[1] - state['center'][1]
                )
                if distance < best_distance:
                    best_id, best_distance = track_id, distance
            if best_id is None:
                best_id = 'camera_vessel_%04d' % self.next_visual_track
                self.next_visual_track += 1
            self.visual_tracks[best_id] = {'center': center, 'updated': now}
            assigned.add(best_id)
            output.append((best_id, candidate))
        stale = [key for key, value in self.visual_tracks.items()
                 if now - value['updated'] > 3.0]
        for key in stale:
            self.visual_tracks.pop(key, None)
        self.affiliation_filter.expire(assigned, now)
        return output

    def _on_image(self, message):
        now = time.monotonic()
        if now - self.last_processed < self.minimum_period:
            return
        self.last_processed = now
        started = time.perf_counter()
        try:
            image = image_to_bgr(message)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return
        candidates = self.backend.detect(image)
        identified = self._assign_visual_ids(candidates)
        output = Detection2DArray()
        output.header = message.header
        metadata = AffiliatedDetection2DArray()
        metadata.header = message.header
        affiliation_names = ('UNKNOWN', 'FRIENDLY', 'HOSTILE', 'NEUTRAL')
        colors = {
            AFFILIATION_UNKNOWN: (0, 220, 255),
            AFFILIATION_FRIENDLY: (255, 180, 40),
            AFFILIATION_HOSTILE: (20, 20, 255),
            AFFILIATION_NEUTRAL: (80, 210, 80),
        }
        for detection_id, candidate in identified:
            if self.enable_affiliation_filter:
                affiliation, affiliation_confidence, _ = (
                    self.affiliation_filter.update(
                        detection_id, candidate.affiliation,
                        candidate.affiliation_confidence,
                    )
                )
            else:
                affiliation = candidate.affiliation
                affiliation_confidence = candidate.affiliation_confidence
            rectangle = (*candidate.rectangle, candidate.class_confidence)
            detection = make_detection(message, rectangle, 0)
            detection.id = detection_id
            output.detections.append(detection)
            metadata.detections.append(affiliated_message(
                message.header, candidate, detection_id,
                affiliation, affiliation_confidence,
            ))
            left, top, right, bottom = candidate.rectangle
            color = colors[affiliation]
            cv2.rectangle(image, (left, top), (right, bottom), color, 2)
            cv2.putText(
                image,
                '%s vessel %.2f %s %.2f' % (
                    detection_id, candidate.class_confidence,
                    affiliation_names[affiliation], affiliation_confidence,
                ),
                (left, max(14, top - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
                cv2.LINE_AA,
            )
        self.publisher.publish(output)
        self.affiliated_publisher.publish(metadata)
        debug = Image()
        debug.header = message.header
        debug.height = image.shape[0]
        debug.width = image.shape[1]
        debug.encoding = 'bgr8'
        debug.is_bigendian = 0
        debug.step = image.shape[1] * 3
        debug.data = image.tobytes()
        self.debug_publisher.publish(debug)
        self.total_frames += 1
        self.last_count = len(identified)
        self.total_detections += len(identified)
        self.last_latency_ms = (time.perf_counter() - started) * 1000.0
        if now - self.last_status_wall >= 1.0:
            self.last_status_wall = now
            self._publish_status()

    def _publish_status(self):
        message = String()
        message.data = json.dumps({
            'mode': 'shadow_camera_detection',
            'frames': self.total_frames,
            'camera_frames': self.total_frames,
            'detections_total': self.total_detections,
            'camera_detections': self.total_detections,
            'detections_last': self.last_count,
            'processing_ms': self.last_latency_ms,
            'friendly_detections': sum(
                state.confirmed == AFFILIATION_FRIENDLY
                for state in self.affiliation_filter.states.values()
            ),
            'hostile_detections': sum(
                state.confirmed == AFFILIATION_HOSTILE
                for state in self.affiliation_filter.states.values()
            ),
            'neutral_detections': sum(
                state.confirmed == AFFILIATION_NEUTRAL
                for state in self.affiliation_filter.states.values()
            ),
            'unknown_detections': sum(
                state.confirmed == AFFILIATION_UNKNOWN
                for state in self.affiliation_filter.states.values()
            ),
            'identity_confirmations': self.affiliation_filter.confirmations,
            'identity_switches': self.affiliation_filter.switches,
            'affiliation_filter_enabled': self.enable_affiliation_filter,
            'control_connected': False,
            'perception_source': 'ground_truth',
        }, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = UsvCameraDetectionNode()
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
