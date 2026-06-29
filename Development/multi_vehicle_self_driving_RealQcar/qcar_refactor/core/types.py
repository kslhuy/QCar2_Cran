import numpy as np
from dataclasses import dataclass
# Data structure definitions will be used for :
# Sensor
# Observer
# Controller
# PathPlanner
# GUI
# V2V

## Sensor data structures
@dataclass
class SensorData:
    "raw sensor data from the vehicle"
    motor_tach: float
    gyro_z: float
    accelerometer: np.ndarray  # shape (3,)
    sensor_timestamp: float
    gps_valid: bool
    gps_position: np.ndarray  # shape (3,)
    gps_timestamp: float

## Observer data structures
@dataclass
class VehicleStateEstimate:
    "state estimation of the observer, can be seen as the output of the observer"
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
    "minimal control input to get written to the vehicle"
    throttle: float       # 0.0 to max_throttle
    steering: float       # -max_steering to +max_steering
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


# Backward-compatible alias for old imports. Prefer V2VState in new code.
VBoradcastState = V2VState

@dataclass
class V2VMessage:
    "V2V message structure"
    vehicle_id: int
    timestamp: float
    message_type: str    
    payload: dict         


# Backward-compatible alias for old imports. Prefer V2VMessage in new code.
VBroadcastMessage = V2VMessage
