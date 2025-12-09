"""
Event-Driven Vehicle State Machine

This state machine manages vehicle states using an event-driven architecture.
Commands from Ground Station are converted to events that are dispatched to 
the current state, which decides whether to handle them and transition.

States:
- INITIALIZING: System startup and initialization
- WAITING_FOR_START: Ready, waiting for start command  
- FOLLOWING_PATH: Autonomous path following
- FOLLOWING_LEADER: Following another vehicle (platoon mode)
- STOPPED: Vehicle stopped (manual/safety/emergency)
"""
import time
from typing import Dict, Any, Tuple, Optional
from .vehicle_state import VehicleState, StateTransitionReason
from .state_base import StateBase
from .initializing_state import InitializingState
from .waiting_for_start_state import WaitingForStartState
from .following_path_state import FollowingPathState
from .following_leader_state import FollowingLeaderState
from .stopped_state import StoppedState
from .manual_mode_state import ManualModeState


class VehicleStateMachine:
    """Simplified event-driven state machine with direct transitions"""
    
    def __init__(self, vehicle_logic, logger=None):
        self.vehicle_logic = vehicle_logic
        self.logger = logger
        
        # Current state tracking
        self.state = VehicleState.INITIALIZING
        self.previous_state = None
        self.state_entry_time = time.time()
        
        # State handlers - each handles its own logic and transitions
        self.state_handlers: Dict[VehicleState, StateBase] = {
            VehicleState.INITIALIZING: InitializingState(vehicle_logic),
            VehicleState.WAITING_FOR_START: WaitingForStartState(vehicle_logic),
            VehicleState.FOLLOWING_PATH: FollowingPathState(vehicle_logic),
            VehicleState.FOLLOWING_LEADER: FollowingLeaderState(vehicle_logic),
            VehicleState.MANUAL_MODE: ManualModeState(vehicle_logic),
            VehicleState.STOPPED: StoppedState(vehicle_logic)
        }
        
        # Statistics
        self.transition_count = 0
        self.state_history = [(VehicleState.INITIALIZING, time.time())]
        
        # Enter the initial state
        try:
            self.state_handlers[VehicleState.INITIALIZING].enter()
            if logger:
                logger.logger.info(f" State machine initialized in {self.state.name} state")
        except Exception as e:
            if logger:
                logger.log_error("Failed to enter INITIALIZING state", e)

    
    def initialize_event_system(self):
        """
        Initialize the event system by connecting to command handler
        Much simpler now - just establish the connection
        """
        if hasattr(self.vehicle_logic, 'command_handler'):
            self.vehicle_logic.command_handler.register_state_listeners(self)
            if self.logger:
                self.logger.logger.info("[E] Event system initialized")
        else:
            if self.logger:
                self.logger.logger.warning("[!] Cannot initialize event system - no command handler found")
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float]:
        """
        Update the state machine and get control commands
        
        Args:
            dt: Time delta since last update
            sensor_data: Dictionary containing all sensor readings
        
        Returns:
            Tuple[float, float]: (throttle_command, steering_command)
        """
        try:
            # Ensure event system is initialized
            if hasattr(self.vehicle_logic, 'command_handler') and not hasattr(self.vehicle_logic.command_handler, 'state_machine'):
                self.initialize_event_system()
            
            # Get current state handler
            current_state = self.state_handlers[self.state]
            
            # Log current state periodically (every 5 seconds)
            if hasattr(self.vehicle_logic, 'loop_counter') and self.vehicle_logic.loop_counter % 1000 == 0:
                if self.logger:
                    time_in_state = self.get_time_in_state()
                    self.logger.logger.info(f"[STATE] Current: {self.state.name} (for {time_in_state:.1f}s)")
            
            # Update the current state
            throttle, steering, transition = current_state.update(dt, sensor_data)
            
            # Check if state wants to transition
            if transition:
                new_state, reason = transition
                self._transition_to(new_state, reason)
            
            return throttle, steering
            
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"State machine update error in {self.state.name}", e)
            return 0.0, 0.0  # Safe fallback
    
    def _transition_to(self, new_state: VehicleState, reason: StateTransitionReason):
        """Execute state transition"""
        if new_state == self.state:
            return  # No change needed
        
        old_state = self.state
        
        try:
            # Exit current state
            self.state_handlers[self.state].exit()
            
            # Log transition
            if self.logger:
                self.logger.log_state_transition(
                    old_state.name, 
                    new_state.name
                )
                # Log the reason separately
                self.logger.logger.info(f"Transition reason: {reason.name}")
            
            # Update state
            self.previous_state = self.state
            self.state = new_state
            self.state_entry_time = time.time()
            
            # Enter new state with special handling for STOPPED state
            if new_state == VehicleState.STOPPED:
                # Set stop reason based on transition reason
                stopped_handler = self.state_handlers[VehicleState.STOPPED]
                if hasattr(stopped_handler, 'set_stop_reason'):
                    stop_reason_map = {
                        StateTransitionReason.STOP_COMMAND: "Ground Station stop command",
                        StateTransitionReason.EMERGENCY_STOP: "Emergency stop triggered",
                        StateTransitionReason.COLLISION_RISK: "Collision avoidance activated",
                        StateTransitionReason.ERROR: "System error detected",
                        StateTransitionReason.SHUTDOWN: "System shutdown"
                    }
                    stop_reason = stop_reason_map.get(reason, f"State transition: {reason.name}")
                    stop_type = "emergency" if reason in [StateTransitionReason.EMERGENCY_STOP, StateTransitionReason.COLLISION_RISK] else "manual"
                    stopped_handler.set_stop_reason(stop_reason, stop_type)
            
            success = self.state_handlers[new_state].enter()
            
            if not success:
                # State enter failed - this is a critical error
                if self.logger:
                    self.logger.log_error(f"Failed to enter state: {new_state.name}")
                # Force transition to STOPPED for safety
                if new_state != VehicleState.STOPPED:
                    self.state = VehicleState.STOPPED
                    stopped_handler = self.state_handlers[VehicleState.STOPPED]
                    if hasattr(stopped_handler, 'set_stop_reason'):
                        stopped_handler.set_stop_reason("State initialization failure", "emergency")
                    self.state_handlers[VehicleState.STOPPED].enter()
            
            # Update statistics
            self.transition_count += 1
            self.state_history.append((new_state, time.time()))
            
            # Keep history manageable
            if len(self.state_history) > 100:
                self.state_history = self.state_history[-50:]
                
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"State transition error: {old_state.name} -> {new_state.name}", e)
            
            # Emergency fallback to STOPPED
            try:
                self.state = VehicleState.STOPPED
                self.state_handlers[VehicleState.STOPPED].enter()
            except:
                pass  # Last resort - just continue
    
    def force_transition_to(self, new_state: VehicleState, reason: str = "Manual override"):
        """
        Force transition to a specific state (for external control)
        
        Args:
            new_state: Target state
            reason: Reason for forced transition
        """
        if self.logger:
            self.logger.logger.info(f"[F] Forced transition to {new_state.name}: {reason}")
        
        # Create a manual reason
        manual_reason = StateTransitionReason.START_COMMAND if new_state != VehicleState.STOPPED else StateTransitionReason.STOP_COMMAND
        self._transition_to(new_state, manual_reason)
    
    def emergency_stop(self, reason: str = "Emergency stop"):
        """
        Immediately transition to STOPPED state
        
        Args:
            reason: Reason for emergency stop
        """
        if self.logger:
            self.logger.log_warning(f"[!] Emergency stop: {reason}")
        
        # The stop reason will be set during the transition process
        self._transition_to(VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
    
    def dispatch_event_to_current_state(self, command_type, data: Dict[str, Any] = None, source: str = "manual"):
        """
        Manually dispatch an event to the current state (for testing/debugging)
        
        Args:
            command_type: CommandType enum to dispatch
            data: Event data  
            source: Source of the event
            
        Returns:
            bool: True if event was handled
        """
        if hasattr(self.vehicle_logic, 'command_handler'):
            return self.vehicle_logic.command_handler.dispatch_manual_event(command_type, data, source)
        return False
    
    # === Convenience methods for external queries ===
    
    def get_time_in_state(self) -> float:
        """Get time spent in current state"""
        return time.time() - self.state_entry_time
    
    def is_operational(self) -> bool:
        """Check if vehicle is in an operational state"""
        return self.state in [
            VehicleState.FOLLOWING_PATH,
            VehicleState.FOLLOWING_LEADER
        ]
    
    def is_stopped(self) -> bool:
        """Check if vehicle is stopped"""
        return self.state == VehicleState.STOPPED
    
    def is_ready(self) -> bool:
        """Check if vehicle is ready to start"""
        return self.state == VehicleState.WAITING_FOR_START
    
    def is_initializing(self) -> bool:
        """Check if vehicle is still initializing"""
        return self.state == VehicleState.INITIALIZING
    
    def should_control(self) -> bool:
        """Check if control commands should be sent (active driving)"""
        return self.state in [
            VehicleState.FOLLOWING_PATH,
            VehicleState.FOLLOWING_LEADER
        ]
    
    def get_current_state_handler(self) -> StateBase:
        """Get the current state handler for direct access if needed"""
        return self.state_handlers[self.state]
    
    def get_state_statistics(self) -> Dict[str, Any]:
        """Get statistics about state machine operation"""
        return {
            'current_state': self.state.name,
            'time_in_current_state': self.get_time_in_state(),
            'total_transitions': self.transition_count,
            'previous_state': self.previous_state.name if self.previous_state else None,
            'state_history_length': len(self.state_history)
        }
    
    def get_state_history(self, last_n: int = 10) -> list:
        """Get recent state history"""
        return self.state_history[-last_n:] if last_n > 0 else self.state_history
    
    def get_current_state_info(self) -> Dict[str, Any]:
        """Get detailed information about current state"""
        return {
            'state': self.state.name,
            'time_in_state': self.get_time_in_state(),
            'handler_class': self.state_handlers[self.state].__class__.__name__,
            'is_operational': self.is_operational(),
            'is_ready': self.is_ready(),
            'is_stopped': self.is_stopped(),
            'is_initializing': self.is_initializing()
        }