"""
Simplified Vehicle States

Only contains the essential states that represent meaningful vehicle behaviors.
Intermediate states are handled internally within each main state.
"""
from enum import Enum, auto


class VehicleState(Enum):
    """Simplified vehicle operational states"""
    
    # System initialization
    INITIALIZING = auto()         # System startup and component initialization
    
    # Ready state  
    WAITING_FOR_START = auto()    # Ready to begin operation, waiting for start command
    
    # Autonomous operation
    FOLLOWING_PATH = auto()       # Following predefined waypoint path
    FOLLOWING_LEADER = auto()     # Following another vehicle (platoon/convoy mode)
    
    # Manual control
    MANUAL_MODE = auto()          # Direct manual control from Ground Station
    
    # Stopped state
    STOPPED = auto()              # Vehicle stopped (manual stop, safety stop, etc.)


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
    
    # Manual control
    MANUAL_MODE_ACTIVATED = auto()
    
    # Safety events
    EMERGENCY_STOP = auto()
    COLLISION_RISK = auto()
    
    # System events
    SHUTDOWN = auto()
    ERROR = auto()