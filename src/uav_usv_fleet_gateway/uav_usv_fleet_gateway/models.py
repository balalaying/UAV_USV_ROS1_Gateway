"""Transport-neutral internal fleet models."""

from copy import deepcopy
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


def nullable_vector():
    return {'x': None, 'y': None, 'z': None}


@dataclass
class VehicleModel:
    id: str
    type: str = 'UNKNOWN'
    namespace: str = ''
    online: bool = False
    stale: bool = False
    last_update: float = 0.0
    frame_id: str = ''
    position: Dict[str, Optional[float]] = field(
        default_factory=nullable_vector)
    orientation: Dict[str, Optional[float]] = field(default_factory=lambda: {
        'roll': None, 'pitch': None, 'yaw': None,
        'qx': None, 'qy': None, 'qz': None, 'qw': None,
    })
    linear_velocity: Dict[str, Optional[float]] = field(
        default_factory=nullable_vector)
    angular_velocity: Dict[str, Optional[float]] = field(
        default_factory=nullable_vector)
    speed: Optional[float] = None
    battery: Dict[str, Optional[float]] = field(default_factory=lambda: {
        'percentage': None, 'voltage': None,
    })
    mode: Optional[str] = None
    armed: Optional[bool] = None
    health: Dict[str, Any] = field(default_factory=dict)
    received_at: float = 0.0

    def public(self):
        value = asdict(self)
        value.pop('received_at', None)
        return value


@dataclass
class TargetModel:
    track_id: str
    class_name: Optional[str] = None
    confidence: Optional[float] = None
    timestamp: float = 0.0
    stamp: Dict[str, int] = field(default_factory=dict)
    frame_id: str = ''
    source: str = 'source_mux'
    sensor_source: Optional[str] = None
    position: Dict[str, Optional[float]] = field(
        default_factory=nullable_vector)
    velocity: Dict[str, Optional[float]] = field(
        default_factory=nullable_vector)
    bbox: Dict[str, Optional[float]] = field(default_factory=lambda: {
        'length': None, 'width': None, 'height': None, 'yaw': None,
    })
    received_at: float = 0.0

    def public(self):
        value = asdict(self)
        value.pop('received_at', None)
        return value


@dataclass
class SensorModel:
    vehicle_id: str
    sensor_id: str
    sensor_type: str = 'unknown'
    online: bool = False
    frequency_hz: Optional[float] = None
    last_update: float = 0.0
    frame_id: str = ''
    status: str = 'NOT_AVAILABLE'
    topic: Optional[str] = None
    latency_sec: Optional[float] = None
    point_count: Optional[int] = None
    dropped_messages: Optional[int] = None
    received_at: float = 0.0

    def public(self):
        value = asdict(self)
        value.pop('received_at', None)
        return value


@dataclass
class FleetSnapshot:
    vehicles: List[Dict[str, Any]] = field(default_factory=list)
    targets: List[Dict[str, Any]] = field(default_factory=list)
    sensors: List[Dict[str, Any]] = field(default_factory=list)
    mission: Dict[str, Any] = field(default_factory=dict)
    gateway: Dict[str, Any] = field(default_factory=dict)

    def public(self):
        return deepcopy(asdict(self))
