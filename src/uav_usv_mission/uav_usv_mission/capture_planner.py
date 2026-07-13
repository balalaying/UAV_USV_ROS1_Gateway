"""State-aware scalable role assignment and capture-point generation."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Mapping, Sequence

from uav_usv_mission.target_predictor import TargetPredictor
from uav_usv_mission.target_predictor import TargetState
from uav_usv_mission.target_predictor import normalize_angle


TYPE_UAV = 1
TYPE_USV = 2
ROLE_AIR_OBSERVER = 1
ROLE_SURFACE_INTERCEPTOR = 2


@dataclass(frozen=True)
class VehicleKinematics:
    vehicle_id: str
    vehicle_type: int
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class CaptureSlot:
    role: str
    role_type: int
    vehicle_type: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class CaptureAssignment:
    vehicle_id: str
    vehicle_type: int
    role: str
    role_type: int
    x: float
    y: float
    z: float
    cost: float


@dataclass(frozen=True)
class CapturePlan:
    center_x: float
    center_y: float
    capture_radius: float
    assignments: Dict[str, CaptureAssignment]


class CapturePlanner:
    """Generate typed slots and globally minimize vehicle-to-slot cost."""

    def __init__(
        self,
        capture_radius=18.0,
        observation_altitude=22.0,
        uav_prediction_time=2.5,
        usv_prediction_time=9.0,
        usv_height=0.55,
        usv_spacing=10.0,
        heading_weight=2.5,
        velocity_weight=1.5,
        switch_penalty=5.0,
    ):
        self.capture_radius = max(2.0, float(capture_radius))
        self.observation_altitude = float(observation_altitude)
        self.uav_prediction_time = max(0.0, float(uav_prediction_time))
        self.usv_prediction_time = max(0.0, float(usv_prediction_time))
        self.usv_height = float(usv_height)
        self.usv_spacing = max(2.0, float(usv_spacing))
        self.heading_weight = max(0.0, float(heading_weight))
        self.velocity_weight = max(0.0, float(velocity_weight))
        self.switch_penalty = max(0.0, float(switch_penalty))

    @staticmethod
    def _basis(yaw):
        return math.cos(yaw), math.sin(yaw), -math.sin(yaw), math.cos(yaw)

    def _uav_slots(self, anchor, count):
        forward_x, forward_y, left_x, left_y = self._basis(anchor.yaw)
        slots = []
        for index in range(count):
            angle = math.pi * 0.5 + 2.0 * math.pi * index / max(1, count)
            offset_x = self.capture_radius * (
                forward_x * math.cos(angle) + left_x * math.sin(angle)
            )
            offset_y = self.capture_radius * (
                forward_y * math.cos(angle) + left_y * math.sin(angle)
            )
            slots.append(CaptureSlot(
                role='air_observer_%02d' % (index + 1),
                role_type=ROLE_AIR_OBSERVER,
                vehicle_type=TYPE_UAV,
                x=anchor.x + offset_x,
                y=anchor.y + offset_y,
                z=self.observation_altitude + 2.0 * (index % 2),
            ))
        return slots

    def _usv_slots(self, anchor, count):
        _, _, left_x, left_y = self._basis(anchor.yaw)
        slots = []
        for index in range(count):
            lateral = (index - 0.5 * (count - 1)) * self.usv_spacing
            slots.append(CaptureSlot(
                role='surface_interceptor_%02d' % (index + 1),
                role_type=ROLE_SURFACE_INTERCEPTOR,
                vehicle_type=TYPE_USV,
                x=anchor.x + left_x * lateral,
                y=anchor.y + left_y * lateral,
                z=self.usv_height,
            ))
        return slots

    def _cost(self, vehicle, slot, previous_roles):
        dx = slot.x - vehicle.x
        dy = slot.y - vehicle.y
        distance = math.hypot(dx, dy)
        nominal_speed = 8.0 if vehicle.vehicle_type == TYPE_UAV else 3.0
        bearing = math.atan2(dy, dx) if distance > 1e-6 else vehicle.yaw
        heading_error = abs(normalize_angle(bearing - vehicle.yaw))

        speed = math.hypot(vehicle.vx, vehicle.vy)
        if speed > 0.05 and distance > 1e-6:
            velocity_heading = math.atan2(vehicle.vy, vehicle.vx)
            velocity_error = abs(normalize_angle(bearing - velocity_heading))
        else:
            velocity_error = 0.0
        cost = (
            distance / nominal_speed
            + self.heading_weight * heading_error
            + self.velocity_weight * velocity_error
        )
        previous = previous_roles.get(vehicle.vehicle_id)
        if previous and previous != slot.role:
            cost += self.switch_penalty
        return cost

    @staticmethod
    def _hungarian(costs):
        """Return the minimum-cost column for each row in O(n^3)."""
        count = len(costs)
        if count == 0:
            return []
        u = [0.0] * (count + 1)
        v = [0.0] * (count + 1)
        p = [0] * (count + 1)
        way = [0] * (count + 1)
        for row in range(1, count + 1):
            p[0] = row
            column0 = 0
            minimum = [math.inf] * (count + 1)
            used = [False] * (count + 1)
            while True:
                used[column0] = True
                row0 = p[column0]
                delta = math.inf
                column1 = 0
                for column in range(1, count + 1):
                    if used[column]:
                        continue
                    current = costs[row0 - 1][column - 1] - u[row0] - v[column]
                    if current < minimum[column]:
                        minimum[column] = current
                        way[column] = column0
                    if minimum[column] < delta:
                        delta = minimum[column]
                        column1 = column
                for column in range(count + 1):
                    if used[column]:
                        u[p[column]] += delta
                        v[column] -= delta
                    else:
                        minimum[column] -= delta
                column0 = column1
                if p[column0] == 0:
                    break
            while True:
                column1 = way[column0]
                p[column0] = p[column1]
                column0 = column1
                if column0 == 0:
                    break
        result = [-1] * count
        for column in range(1, count + 1):
            if p[column]:
                result[p[column] - 1] = column - 1
        return result

    def _assign_group(self, vehicles, slots, previous_roles):
        if not vehicles:
            return {}
        costs = [
            [self._cost(vehicle, slot, previous_roles) for slot in slots]
            for vehicle in vehicles
        ]
        selected = self._hungarian(costs)
        assignments = {}
        for row, column in enumerate(selected):
            vehicle = vehicles[row]
            slot = slots[column]
            assignments[vehicle.vehicle_id] = CaptureAssignment(
                vehicle_id=vehicle.vehicle_id,
                vehicle_type=vehicle.vehicle_type,
                role=slot.role,
                role_type=slot.role_type,
                x=slot.x,
                y=slot.y,
                z=slot.z,
                cost=costs[row][column],
            )
        return assignments

    def plan(
        self,
        target: TargetState,
        prediction,
        vehicles: Iterable[VehicleKinematics],
        previous_roles: Mapping[str, str] = None,
    ) -> CapturePlan:
        vehicles = tuple(vehicle for vehicle in vehicles if vehicle.vehicle_id)
        previous_roles = previous_roles or {}
        uavs = sorted(
            (item for item in vehicles if item.vehicle_type == TYPE_UAV),
            key=lambda item: item.vehicle_id,
        )
        usvs = sorted(
            (item for item in vehicles if item.vehicle_type == TYPE_USV),
            key=lambda item: item.vehicle_id,
        )
        uav_anchor = TargetPredictor.point_at(
            prediction, self.uav_prediction_time
        )
        usv_anchor = TargetPredictor.point_at(
            prediction, self.usv_prediction_time
        )
        assignments = self._assign_group(
            uavs, self._uav_slots(uav_anchor, len(uavs)), previous_roles
        )
        assignments.update(self._assign_group(
            usvs, self._usv_slots(usv_anchor, len(usvs)), previous_roles
        ))
        return CapturePlan(
            center_x=uav_anchor.x,
            center_y=uav_anchor.y,
            capture_radius=self.capture_radius,
            assignments=assignments,
        )
