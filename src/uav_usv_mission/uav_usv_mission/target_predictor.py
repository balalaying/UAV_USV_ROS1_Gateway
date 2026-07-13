"""Constant-velocity target prediction without ROS transport dependencies."""

from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class TargetState:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float = 0.0


@dataclass(frozen=True)
class PredictedPoint:
    time_from_start: float
    x: float
    y: float
    z: float


class TargetPredictor:
    """Build a deterministic prediction horizon from a tracked target."""

    def __init__(self, horizon=12.0, step=1.0):
        self.horizon = max(0.1, float(horizon))
        self.step = max(0.05, float(step))

    def predict(self, state: TargetState) -> List[PredictedPoint]:
        points = []
        sample_count = int(self.horizon / self.step)
        for index in range(sample_count + 1):
            time_from_start = min(index * self.step, self.horizon)
            points.append(PredictedPoint(
                time_from_start=time_from_start,
                x=state.x + state.vx * time_from_start,
                y=state.y + state.vy * time_from_start,
                z=state.z + state.vz * time_from_start,
            ))
        if points[-1].time_from_start < self.horizon:
            points.append(PredictedPoint(
                time_from_start=self.horizon,
                x=state.x + state.vx * self.horizon,
                y=state.y + state.vy * self.horizon,
                z=state.z + state.vz * self.horizon,
            ))
        return points

    @staticmethod
    def point_at(prediction, time_from_start):
        """Linearly interpolate one point from a sampled prediction."""
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
                return PredictedPoint(
                    time_from_start=requested,
                    x=before.x + ratio * (after.x - before.x),
                    y=before.y + ratio * (after.y - before.y),
                    z=before.z + ratio * (after.z - before.z),
                )
        return prediction[-1]
