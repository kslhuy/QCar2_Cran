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

from .vehicle_state import VehicleState, Gear

__all__ = ["VehicleState", "Gear", "StateBase", "VehicleStateMachine"]


def __getattr__(name):
    """Enums remain usable without loading controllers or hardware adapters."""
    if name == "StateBase":
        from .state_base import StateBase
        return StateBase
    if name == "VehicleStateMachine":
        from .vehicle_state_machine import VehicleStateMachine
        return VehicleStateMachine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
