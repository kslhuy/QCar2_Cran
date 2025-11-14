"""
Simplified State Machine Package for Vehicle Control System

This package contains a simplified state machine with only the essential states:
- INITIALIZING: System initialization and setup
- WAITING_FOR_START: Ready to begin operation
- FOLLOWING_PATH: Autonomous path following 
- FOLLOWING_LEADER: Following another vehicle (platoon mode)
- STOPPED: Vehicle stopped (manual stop or safety)

Each state handles its own transition logic internally.
"""

from .vehicle_state import VehicleState
from .state_base import StateBase
from .vehicle_state_machine import VehicleStateMachine

__all__ = ['VehicleState', 'StateBase', 'VehicleStateMachine']