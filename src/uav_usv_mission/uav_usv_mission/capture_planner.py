"""Scalable role assignment and capture-point generation."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Sequence

from uav_usv_mission.target_predictor import TargetPredictor
from uav_usv_mission.target_predictor import TargetState


@dataclass(frozen=True)
class CaptureAssignment:
    vehicle_id: str
    role: str
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class CapturePlan:
    center_x: float
    center_y: float
    capture_radius: float
    assignments: Dict[str, CaptureAssignment]


class CapturePlanner:
    """Place UAV observers around the target and one USV ahead of it."""

    def __init__(
        self,
        capture_radius=18.0,
        observation_altitude=22.0,
        uav_prediction_time=2.5,
        usv_prediction_time=9.0,
        usv_height=0.55,
    ):
        self.capture_radius = max(2.0, float(capture_radius))
        self.observation_altitude = float(observation_altitude)
        self.uav_prediction_time = max(0.0, float(uav_prediction_time))
        self.usv_prediction_time = max(0.0, float(usv_prediction_time))
        self.usv_height = float(usv_height)

    def plan(
        self,
        target: TargetState,
        prediction,
        uav_ids: Sequence[str],
        usv_ids: Iterable[str],
    ) -> CapturePlan:
        usv_ids = tuple(usv_ids)
        uav_anchor = TargetPredictor.point_at(
            prediction, self.uav_prediction_time
        )
        usv_anchor = TargetPredictor.point_at(
            prediction, self.usv_prediction_time
        )
        speed = math.hypot(target.vx, target.vy)
        if speed > 0.05:
            forward_x, forward_y = target.vx / speed, target.vy / speed
        else:
            forward_x, forward_y = 1.0, 0.0
        left_x, left_y = -forward_y, forward_x

        assignments = {}
        uav_count = len(uav_ids)
        for index, vehicle_id in enumerate(uav_ids):
            if uav_count == 1:
                angle = math.pi * 0.5
            else:
                angle = math.pi * 0.5 + 2.0 * math.pi * index / uav_count
            offset_x = self.capture_radius * (
                forward_x * math.cos(angle) + left_x * math.sin(angle)
            )
            offset_y = self.capture_radius * (
                forward_y * math.cos(angle) + left_y * math.sin(angle)
            )
            role = 'air_observer_%02d' % (index + 1)
            assignments[vehicle_id] = CaptureAssignment(
                vehicle_id=vehicle_id,
                role=role,
                x=uav_anchor.x + offset_x,
                y=uav_anchor.y + offset_y,
                z=self.observation_altitude + index * 2.0,
            )

        for index, vehicle_id in enumerate(usv_ids):
            lateral = (index - 0.5 * (max(1, len(usv_ids)) - 1)) * 8.0
            assignments[vehicle_id] = CaptureAssignment(
                vehicle_id=vehicle_id,
                role='surface_interceptor_%02d' % (index + 1),
                x=usv_anchor.x + left_x * lateral,
                y=usv_anchor.y + left_y * lateral,
                z=self.usv_height,
            )

        return CapturePlan(
            center_x=uav_anchor.x,
            center_y=uav_anchor.y,
            capture_radius=self.capture_radius,
            assignments=assignments,
        )
