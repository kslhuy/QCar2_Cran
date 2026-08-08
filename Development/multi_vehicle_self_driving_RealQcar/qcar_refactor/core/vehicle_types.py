from __future__ import annotations

import numpy as np
from dataclasses import dataclass
import math
from typing import Sequence
# Data structure definitions will be used for :
# Sensor
# Observer
# Controller
# PathPlanner
# GUI
# V2V

@dataclass
class SensorData:
    """Raw vehicle data in SI units.

    ``motor_tach`` is signed longitudinal speed in m/s where a backend can
    provide it; otherwise it is that backend's documented approximation.
    ``gyro_z`` is yaw rate in rad/s, ``accelerometer`` is m/s^2, timestamps
    are seconds, and GPS stores local ``[x_m, y_m, yaw_rad]``.
    """
    motor_tach: float
    gyro_z: float
    accelerometer: np.ndarray  # [ax, ay, az] in m/s^2
    sensor_timestamp: float  # seconds
    gps_valid: bool
    gps_position: np.ndarray  # shape (3,)
    gps_timestamp: float  # seconds


@dataclass(frozen=True)
class LaserScanSample:
    """Platform-neutral planar LiDAR frame using ROS ``LaserScan`` semantics.

    This is a vehicle data contract.  IO adapters create it from native sensor
    frames and localisation algorithms consume it; neither dependency owns the
    type or imports the other.
    """

    timestamp_ns: int
    frame_id: str
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float
    time_increment_s: float
    scan_time_s: float
    range_min_m: float
    range_max_m: float
    ranges_m: Sequence[float]
    intensities: Sequence[float] = ()

    def __post_init__(self) -> None:
        _non_negative_int(self.timestamp_ns, "timestamp_ns")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("LaserScanSample.frame_id must be a non-empty string")
        for name, value in (
            ("angle_min_rad", self.angle_min_rad),
            ("angle_max_rad", self.angle_max_rad),
            ("angle_increment_rad", self.angle_increment_rad),
            ("time_increment_s", self.time_increment_s),
            ("scan_time_s", self.scan_time_s),
            ("range_min_m", self.range_min_m),
            ("range_max_m", self.range_max_m),
        ):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"LaserScanSample.{name} must be finite")
        if self.angle_max_rad < self.angle_min_rad:
            raise ValueError("LaserScanSample.angle_max_rad must not be less than angle_min_rad")
        if self.angle_increment_rad <= 0.0:
            raise ValueError("LaserScanSample.angle_increment_rad must be positive")
        if self.time_increment_s < 0.0 or self.scan_time_s < 0.0:
            raise ValueError("LaserScanSample time increments must be non-negative")
        if self.range_min_m < 0.0 or self.range_max_m <= self.range_min_m:
            raise ValueError("LaserScanSample ranges must satisfy 0 <= range_min_m < range_max_m")
        ranges = tuple(float(value) for value in self.ranges_m)
        intensities = tuple(float(value) for value in self.intensities)
        if not ranges:
            raise ValueError("LaserScanSample.ranges_m must contain at least one bin")
        if intensities and len(intensities) != len(ranges):
            raise ValueError("LaserScanSample.intensities must be empty or match ranges_m")
        object.__setattr__(self, "ranges_m", ranges)
        object.__setattr__(self, "intensities", intensities)

    @property
    def timestamp_s(self) -> float:
        return self.timestamp_ns / 1_000_000_000

    def to_ros_fields(self) -> dict[str, object]:
        seconds, nanoseconds = divmod(self.timestamp_ns, 1_000_000_000)
        return {
            "header": {"stamp": {"sec": seconds, "nanosec": nanoseconds}, "frame_id": self.frame_id},
            "angle_min": self.angle_min_rad,
            "angle_max": self.angle_max_rad,
            "angle_increment": self.angle_increment_rad,
            "time_increment": self.time_increment_s,
            "scan_time": self.scan_time_s,
            "range_min": self.range_min_m,
            "range_max": self.range_max_m,
            "ranges": self.ranges_m,
            "intensities": self.intensities,
        }


@dataclass(frozen=True)
class PoseMeasurement:
    """One optional map-frame pose correction from a positioning source."""

    timestamp_ns: int
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    valid: bool

    def __post_init__(self) -> None:
        _non_negative_int(self.timestamp_ns, "timestamp_ns")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("PoseMeasurement.frame_id must be a non-empty string")
        for name, value in (("x_m", self.x_m), ("y_m", self.y_m), ("yaw_rad", self.yaw_rad)):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"PoseMeasurement.{name} must be finite")
        if not isinstance(self.valid, bool):
            raise ValueError("PoseMeasurement.valid must be a boolean")

## Observer data structures
@dataclass
class VehicleStateEstimate:
    """Observer output: metres, radians, m/s, m/s^2, seconds, and validity."""
    timestamp: float
    x: float
    y: float
    theta: float
    velocity: float
    acceleration: float
    gps_valid: bool
    # Overall observer health for control and fleet publication. This is not
    # equivalent to instantaneous GPS measurement availability.
    valid: bool = True

## Controller data structures
@dataclass
class ControlInput:
    """Actuator request with target velocity in m/s.

    Throttle and steering are backend-normalized commands. Each IO backend
    documents its scale, sign convention, and physical approximation.
    """
    throttle: float
    steering: float
    target_velocity: float
    source: str = ""      # e.g. "pid", "pure_pursuit", "zero"

### Controller data structures
@dataclass
class ControllerReference:
    "target point for the controller to follow"
    target_x: float
    target_y: float
    target_theta: float
    target_velocity: float
    is_finished: bool = False


@dataclass(frozen=True)
class V2VMessage:
    """Generic V2V transport envelope.

    Payload semantics belong to consumers such as the fleet state store.
    Receive timestamps are meaningful only on the local receiver.
    """

    sender_id: int
    message_type: str
    payload: dict
    sequence: int
    sent_at_monotonic: float
    sent_at_perf_counter_ns: int
    received_at_monotonic: float = 0.0
    received_at_perf_counter_ns: int = 0


def _non_negative_int(value: object, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
