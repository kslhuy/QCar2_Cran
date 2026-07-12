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
    """Observer output: metres, radians, m/s, m/s^2, and seconds."""
    timestamp: float
    x: float
    y: float
    theta: float
    velocity: float
    acceleration: float
    gps_valid: bool

## Controller data structures
@dataclass
class ControlCommand:
    """Actuator request with target velocity in m/s.

    Throttle and steering are backend-normalized commands. Each IO backend
    documents its scale, sign convention, and physical approximation.
    """
    throttle: float
    steering: float
    target_velocity: float
    source: str = ""      # e.g. "pid", "pure_pursuit", "zero"

### PathPlanner data structures
@dataclass
class PlannerTarget:
    "target for the planner to follow"
    target_x: float
    target_y: float
    target_theta: float
    target_velocity: float
    is_finished: bool = False


## GUI data structures
@dataclass
class GuiCommand:
    "command from the GUI to control the system"
    command: str          # "START", "STOP", "EMERGENCY_STOP", "RESET", "SET_VELOCITY", "SET_PATH"
    payload: dict         # e.g. {"velocity": 0.6} or {"path": "pp_waypoints.csv"}


## V2V data structures
@dataclass
class V2VState:
    "vehicle state shared through V2V broadcast/vehicle-to-vehicle messages"
    vehicle_id: int
    timestamp: float
    x: float
    y: float
    theta: float
    velocity: float

@dataclass
class V2VMessage:
    "V2V message structure"
    sender_id: int
    timestamp: float
    message_type: str    
    payload: dict

    @property
    def vehicle_id(self) -> int:
        """Backward-compatible alias. Prefer sender_id in new V2V code."""
        return self.sender_id
