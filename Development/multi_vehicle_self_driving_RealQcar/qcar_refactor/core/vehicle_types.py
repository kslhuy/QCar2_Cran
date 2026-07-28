import numpy as np
from dataclasses import dataclass
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
