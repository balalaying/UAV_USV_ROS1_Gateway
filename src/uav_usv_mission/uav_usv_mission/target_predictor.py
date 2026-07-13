"""CTRV target prediction without ROS transport dependencies."""

from dataclasses import dataclass
import math
from typing import List


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class TargetState:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float = 0.0
    yaw: float = math.nan
    yaw_rate: float = 0.0

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    @property
    def heading(self):
        if math.isfinite(self.yaw):
            return self.yaw
        if self.speed > 1e-4:
            return math.atan2(self.vy, self.vx)
        return 0.0


@dataclass(frozen=True)
class PredictedPoint:
    time_from_start: float
    x: float
    y: float
    z: float
    yaw: float = 0.0


class TargetPredictor:
    """Predict a coordinated-turn target over a deterministic horizon."""

    MODEL_NAME = 'CTRV'

    def __init__(self, horizon=12.0, step=1.0, turn_rate_epsilon=1e-3):
        self.horizon = max(0.1, float(horizon))
        self.step = max(0.05, float(step))
        self.turn_rate_epsilon = max(1e-6, float(turn_rate_epsilon))

    def _point(self, state, time_from_start):
        time_from_start = max(0.0, float(time_from_start))
        speed = state.speed
        yaw = state.heading
        yaw_rate = float(state.yaw_rate)
        predicted_yaw = normalize_angle(yaw + yaw_rate * time_from_start)
        if abs(yaw_rate) > self.turn_rate_epsilon:
            x = state.x + speed / yaw_rate * (
                math.sin(predicted_yaw) - math.sin(yaw)
            )
            y = state.y + speed / yaw_rate * (
                math.cos(yaw) - math.cos(predicted_yaw)
            )
        else:
            x = state.x + speed * math.cos(yaw) * time_from_start
            y = state.y + speed * math.sin(yaw) * time_from_start
        return PredictedPoint(
            time_from_start=time_from_start,
            x=x,
            y=y,
            z=state.z + state.vz * time_from_start,
            yaw=predicted_yaw,
        )

    def predict(self, state: TargetState) -> List[PredictedPoint]:
        points = []
        sample_count = int(self.horizon / self.step)
        for index in range(sample_count + 1):
            points.append(self._point(
                state, min(index * self.step, self.horizon)
            ))
        if points[-1].time_from_start < self.horizon:
            points.append(self._point(state, self.horizon))
        return points

    @staticmethod
    def point_at(prediction, time_from_start):
        """Interpolate position and wrapped heading from sampled prediction."""
        if not prediction:
            raise ValueError('prediction must contain at least one point')
        requested = max(0.0, float(time_from_start))
        if requested <= prediction[0].time_from_start:
            return prediction[0]
        for before, after in zip(prediction, prediction[1:]):
            if requested <= after.time_from_start:
                duration = after.time_from_start - before.time_from_start
                ratio = 0.0 if duration <= 0.0 else (
                    (requested - before.time_from_start) / duration
                )
                yaw_delta = normalize_angle(after.yaw - before.yaw)
                return PredictedPoint(
                    time_from_start=requested,
                    x=before.x + ratio * (after.x - before.x),
                    y=before.y + ratio * (after.y - before.y),
                    z=before.z + ratio * (after.z - before.z),
                    yaw=normalize_angle(before.yaw + ratio * yaw_delta),
                )
        return prediction[-1]
