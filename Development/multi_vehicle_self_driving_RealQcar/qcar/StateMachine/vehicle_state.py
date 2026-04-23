"""
Simplified Vehicle States

Only contains the essential states that represent meaningful vehicle behaviors.
Intermediate states are handled internally within each main state.
"""

from enum import Enum, auto


class VehicleState(Enum):
    """Simplified vehicle operational states"""

    # System initialization
    INITIALIZING = auto()  # System startup and component initialization

    # Ready state
    WAITING_FOR_START = auto()  # Ready to begin operation, waiting for start command

    # Autonomous operation
    FOLLOWING_PATH = auto()  # Following predefined waypoint path
    FOLLOWING_LEADER = auto()  # Following another vehicle (platoon/convoy mode)
    TAXI_MODE = auto()  # Operating autonomously as a taxi, servicing random nodes

    # Manual control
    MANUAL_MODE = auto()  # Direct manual control from Ground Station

    # Calibration
    CALIBRATING = auto()  # Active calibration sequences (throttle/steering/accel)

    # Stopped state
    STOPPED = auto()  # Vehicle stopped (manual stop, safety stop, etc.)


class Gear(Enum):
    """Vehicle transmission gears"""

    DRIVE_1 = 0.14  # Low speed / Safe mode
    DRIVE_2 = 0.2  # Medium speed
    DRIVE_3 = 0.3  # High speed / Full power
    DRIVE_4 = 0.4  # Highest speed / Maximum power
    DRIVE_5 = 0.5  # Highest speed / Maximum power




class StateTransitionReason(Enum):
    """Reasons for state transitions"""

    # System events
    INITIALIZATION_COMPLETE = auto()
    START_COMMAND = auto()
    STOP_COMMAND = auto()

    # Autonomous operation
    PATH_READY = auto()
    LEADER_DETECTED = auto()
    LEADER_LOST = auto()
    TAXI_MODE_ACTIVATED = auto()

    # Manual control
    MANUAL_MODE_ACTIVATED = auto()

    # Safety events
    EMERGENCY_STOP = auto()
    COLLISION_RISK = auto()

    # Calibration events
    CALIBRATION_COMPLETE = auto()

    # System events
    SHUTDOWN = auto()
    ERROR = auto()
