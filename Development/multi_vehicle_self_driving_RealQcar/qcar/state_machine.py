"""
State Machine for Vehicle Control System
"""
from enum import Enum, auto
from typing import Optional, Callable
import time


class VehicleState(Enum):
    """Vehicle operational states"""
    INITIALIZING = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_START = auto()
    NAVIGATING_TO_START = auto()
    FOLLOWING_PATH = auto()
    EMERGENCY_STOP = auto()
    STOPPED = auto()
    SHUTTING_DOWN = auto()
    ERROR = auto()
    
    # Platoon states
    PLATOON_LEADER_FORMING = auto()    # Leader moving slowly, waiting for followers
    PLATOON_FOLLOWER_SEARCHING = auto() # Follower looking for leader via YOLO
    PLATOON_FOLLOWER_FORMING = auto()   # Follower adjusting spacing to leader
    PLATOON_ACTIVE = auto()             # Platoon formed, coordinated motion
    PLATOON_LOST = auto()               # Lost connection to platoon


class VehicleStateMachine:
    """State machine for managing vehicle behavior"""
    
    def __init__(self, logger=None):
        self.state = VehicleState.INITIALIZING
        self.previous_state = None
        self.state_entry_time = time.time()
        self.logger = logger
        
        # State transition callbacks
        self._on_state_enter_callbacks = {}
        self._on_state_exit_callbacks = {}
        
        # Valid state transitions
        self._valid_transitions = {
            VehicleState.INITIALIZING: [
                VehicleState.WAITING_FOR_CONNECTION,
                VehicleState.WAITING_FOR_START,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.WAITING_FOR_CONNECTION: [
                VehicleState.WAITING_FOR_START,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.WAITING_FOR_START: [
                VehicleState.NAVIGATING_TO_START,
                VehicleState.FOLLOWING_PATH,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.NAVIGATING_TO_START: [
                VehicleState.FOLLOWING_PATH,
                VehicleState.EMERGENCY_STOP,
                VehicleState.STOPPED,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.FOLLOWING_PATH: [
                VehicleState.NAVIGATING_TO_START,
                VehicleState.STOPPED,
                VehicleState.EMERGENCY_STOP,
                VehicleState.PLATOON_LEADER_FORMING,
                VehicleState.PLATOON_FOLLOWER_SEARCHING,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.EMERGENCY_STOP: [
                VehicleState.FOLLOWING_PATH,
                VehicleState.NAVIGATING_TO_START,
                VehicleState.STOPPED,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.STOPPED: [
                VehicleState.FOLLOWING_PATH,
                VehicleState.PLATOON_LEADER_FORMING,
                VehicleState.PLATOON_FOLLOWER_SEARCHING,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.PLATOON_LEADER_FORMING: [
                VehicleState.PLATOON_ACTIVE,
                VehicleState.FOLLOWING_PATH,
                VehicleState.STOPPED,
                VehicleState.EMERGENCY_STOP,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.PLATOON_FOLLOWER_SEARCHING: [
                VehicleState.PLATOON_FOLLOWER_FORMING,
                VehicleState.FOLLOWING_PATH,
                VehicleState.PLATOON_LOST,
                VehicleState.STOPPED,
                VehicleState.EMERGENCY_STOP,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.PLATOON_FOLLOWER_FORMING: [
                VehicleState.PLATOON_ACTIVE,
                VehicleState.PLATOON_FOLLOWER_SEARCHING,
                VehicleState.PLATOON_LOST,
                VehicleState.FOLLOWING_PATH,
                VehicleState.STOPPED,
                VehicleState.EMERGENCY_STOP,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.PLATOON_ACTIVE: [
                VehicleState.PLATOON_LOST,
                VehicleState.FOLLOWING_PATH,
                VehicleState.STOPPED,
                VehicleState.EMERGENCY_STOP,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.PLATOON_LOST: [
                VehicleState.PLATOON_FOLLOWER_SEARCHING,
                VehicleState.FOLLOWING_PATH,
                VehicleState.STOPPED,
                VehicleState.EMERGENCY_STOP,
                VehicleState.ERROR,
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.ERROR: [
                VehicleState.SHUTTING_DOWN
            ],
            VehicleState.SHUTTING_DOWN: []
        }
    
    def register_on_enter(self, state: VehicleState, callback: Callable):
        """Register callback for when entering a state"""
        self._on_state_enter_callbacks[state] = callback
    
    def register_on_exit(self, state: VehicleState, callback: Callable):
        """Register callback for when exiting a state"""
        self._on_state_exit_callbacks[state] = callback
    
    def can_transition_to(self, new_state: VehicleState) -> bool:
        """Check if transition to new state is valid"""
        return new_state in self._valid_transitions.get(self.state, [])
    
    def transition_to(self, new_state: VehicleState, force: bool = False) -> bool:
        """
        Transition to a new state
        
        Args:
            new_state: Target state
            force: If True, bypass validation (use with caution)
            
        Returns:
            True if transition was successful
        """
        if new_state == self.state:
            return True
        
        if not force and not self.can_transition_to(new_state):
            if self.logger:
                self.logger.log_warning(
                    f"Invalid state transition: {self.state.name} -> {new_state.name}"
                )
            return False
        
        # Execute exit callback
        if self.state in self._on_state_exit_callbacks:
            try:
                self._on_state_exit_callbacks[self.state]()
            except Exception as e:
                if self.logger:
                    self.logger.log_error(f"Error in state exit callback: {self.state.name}", e)
        
        # Log transition
        if self.logger:
            self.logger.log_state_transition(self.state.name, new_state.name)
        
        # Update state
        self.previous_state = self.state
        self.state = new_state
        self.state_entry_time = time.time()
        
        # Execute enter callback
        if new_state in self._on_state_enter_callbacks:
            try:
                self._on_state_enter_callbacks[new_state]()
            except Exception as e:
                if self.logger:
                    self.logger.log_error(f"Error in state enter callback: {new_state.name}", e)
        
        return True
    
    def get_time_in_state(self) -> float:
        """Get time spent in current state (seconds)"""
        return time.time() - self.state_entry_time
    
    def is_operational(self) -> bool:
        """Check if vehicle is in an operational state"""
        return self.state in [
            VehicleState.NAVIGATING_TO_START,
            VehicleState.FOLLOWING_PATH,
            VehicleState.PLATOON_LEADER_FORMING,
            VehicleState.PLATOON_FOLLOWER_SEARCHING,
            VehicleState.PLATOON_FOLLOWER_FORMING,
            VehicleState.PLATOON_ACTIVE
        ]
    
    def is_stopped(self) -> bool:
        """Check if vehicle is stopped"""
        return self.state in [
            VehicleState.STOPPED,
            VehicleState.EMERGENCY_STOP
        ]
    
    def should_control(self) -> bool:
        """Check if control commands should be sent (active driving)"""
        return self.state in [
            VehicleState.NAVIGATING_TO_START,
            VehicleState.FOLLOWING_PATH,
            VehicleState.PLATOON_LEADER_FORMING,
            VehicleState.PLATOON_FOLLOWER_SEARCHING,
            VehicleState.PLATOON_FOLLOWER_FORMING,
            VehicleState.PLATOON_ACTIVE
        ]
    
    def should_shutdown(self) -> bool:
        """Check if system should shutdown"""
        return self.state == VehicleState.SHUTTING_DOWN
    
    def is_in_platoon(self) -> bool:
        """Check if vehicle is in any platoon state"""
        return self.state in [
            VehicleState.PLATOON_LEADER_FORMING,
            VehicleState.PLATOON_FOLLOWER_SEARCHING,
            VehicleState.PLATOON_FOLLOWER_FORMING,
            VehicleState.PLATOON_ACTIVE,
            VehicleState.PLATOON_LOST
        ]
    
    def is_platoon_leader(self) -> bool:
        """Check if vehicle is platoon leader"""
        return self.state in [
            VehicleState.PLATOON_LEADER_FORMING,
            VehicleState.PLATOON_ACTIVE
        ] and hasattr(self, '_is_leader') and self._is_leader
    
    def is_platoon_follower(self) -> bool:
        """Check if vehicle is platoon follower"""
        return self.state in [
            VehicleState.PLATOON_FOLLOWER_SEARCHING,
            VehicleState.PLATOON_FOLLOWER_FORMING,
            VehicleState.PLATOON_ACTIVE
        ] and hasattr(self, '_is_leader') and not self._is_leader
