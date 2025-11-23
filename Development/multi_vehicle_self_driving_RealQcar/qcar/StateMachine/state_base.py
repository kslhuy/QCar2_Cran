"""
Base State Class for Event-Driven State Machine

All states inherit from this base class and implement:
- enter, update, exit methods for state lifecycle
- handle_event method to respond to commands and trigger direct transitions

This is much simpler than the previous approach - events trigger immediate
transitions without needing pending_transition mechanisms.
"""
from typing import Dict, Any, Tuple, Optional
from .vehicle_state import VehicleState, StateTransitionReason
import time


# Import CommandType enum - use a local import to avoid circular dependencies
def get_command_type():
    import sys
    import os
    
    # Add parent directory to sys.path for imports
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if parent_dir not in sys.path:
        sys.path.append(parent_dir)
    
    try:
        from command_handler import CommandType
        return CommandType
    except ImportError:
        # Fallback if import fails - create minimal enum
        from enum import Enum
        class CommandType(Enum):
            STOP = "stop"
            START = "start" 
            EMERGENCY_STOP = "emergency_stop"
            SET_VELOCITY = "set_velocity"
            SET_PATH = "set_path"
            SET_PARAMS = "set_params"
            ENABLE_PLATOON_LEADER = "enable_platoon_leader"
            ENABLE_PLATOON_FOLLOWER = "enable_platoon_follower"
            DISABLE_PLATOON = "disable_platoon"
            ACTIVATE_V2V = "activate_v2v"
            DISABLE_V2V = "disable_v2v"
            SHUTDOWN = "shutdown"
            RESET = "reset"
        return CommandType


class StateBase:
    """Base class for all vehicle states with direct event-driven transitions"""
    
    def __init__(self, vehicle_logic):
        """
        Initialize state with reference to vehicle controller
        
        Args:
            vehicle_logic: Reference to main VehicleLogic instance
        """
        self.vehicle_logic = vehicle_logic
        self.logger = vehicle_logic.logger
        self.config = vehicle_logic.config
        
        # State-specific data
        self.state_entry_time = None
        self.state_data = {}
    
    def enter(self) -> bool:
        """
        Called when entering this state
        
        Returns:
            bool: True if successful, False to abort transition
        """
        self.state_entry_time = time.time()
        self.state_data = {}
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """
        Called every control loop iteration while in this state
        
        Args:
            dt: Time delta since last update
            sensor_data: Dictionary containing all sensor readings
        
        Returns:
            Tuple containing:
            - float: throttle_command
            - float: steering_command  
            - Optional[Tuple[VehicleState, StateTransitionReason]]: Next state transition if needed
        """
        # Check for emergency stop conditions first
        if self.should_transition_to_stopped(sensor_data):
            return 0.0, 0.0, (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # Base implementation - no movement, no transition
        return 0.0, 0.0, None
    
    def exit(self):
        """Called when exiting this state"""
        pass
    
    def _init_controllers(self):
        """
        Initialize controllers specific to this state.
        Override this in subclasses to initialize state-specific controllers.
        Called once during state initialization.
        """
        pass
    
    def get_time_in_state(self) -> float:
        """Get time spent in current state"""
        import time
        if self.state_entry_time:
            return time.time() - self.state_entry_time
        return 0.0
    
    # === Single Event Handler Method ===
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle a command type and optionally trigger a state transition
        
        Args:
            command_type: CommandType enum (e.g., CommandType.START, CommandType.STOP)
            data: Optional event data
            
        Returns:
            Optional[Tuple[VehicleState, StateTransitionReason]]: 
                - None if event not handled or no transition needed
                - (new_state, reason) if transition should occur
        """
        data = data or {}
        
        # Get CommandType enum for comparisons
        CommandType = get_command_type()
        
        # Log the event
        if self.logger:
            self.logger.logger.info(f"[CMD] State {self.__class__.__name__} received command: {command_type}")
        
        # Handle common events that most states should support
        if command_type == CommandType.STOP:
            # Get the source/reason from the data if available
            source = data.get('source', 'Ground Station')
            if self.logger:
                self.logger.logger.info(f"[STOP] Stop command from {source} accepted in {self.__class__.__name__}")
            return (VehicleState.STOPPED, StateTransitionReason.STOP_COMMAND)
        
        elif command_type == CommandType.EMERGENCY_STOP:
            reason = data.get('reason', 'Emergency command')
            if self.logger:
                self.logger.logger.warning(f"[!] Emergency stop ({reason}) accepted in {self.__class__.__name__}")
            return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        elif command_type == CommandType.SET_VELOCITY:
            # Handle velocity updates without transitioning
            v_ref = data.get('v_ref')
            if v_ref is not None and 0 <= v_ref <= 2.0:
                if hasattr(self.vehicle_logic, 'v_ref'):
                    self.vehicle_logic.v_ref = v_ref
                    if self.logger:
                        self.logger.logger.info(f"[OK] Velocity updated to {v_ref} in {self.__class__.__name__}")
                    return None  # No state transition
            else:
                if self.logger:
                    self.logger.logger.warning(f"[!] Invalid velocity: {v_ref}")
            return None
        
        elif command_type == CommandType.SET_PATH:
            # Handle path updates without transitioning
            node_sequence = data.get('node_sequence')
            if node_sequence and isinstance(node_sequence, list):
                if hasattr(self.vehicle_logic, 'update_path'):
                    self.vehicle_logic.update_path(node_sequence)
                    if self.logger:
                        self.logger.logger.info(f"[OK] Path updated in {self.__class__.__name__}")
                    return None  # No state transition
            else:
                if self.logger:
                    self.logger.logger.warning(f"[!] Invalid path update")
            return None
        
        elif command_type == CommandType.ACTIVATE_V2V:
            # Handle V2V activation
            peer_vehicles = data.get('peer_vehicles', [])
            peer_ips = data.get('peer_ips', [])
            
            if peer_vehicles and peer_ips:
                if hasattr(self.vehicle_logic, 'activate_v2v'):
                    success = self.vehicle_logic.activate_v2v(peer_vehicles, peer_ips)
                    if success and self.logger:
                        self.logger.logger.info(f"[OK] V2V activated in {self.__class__.__name__}")
                    elif self.logger:
                        self.logger.logger.warning(f"[!] V2V activation failed in {self.__class__.__name__}")
                    return None  # No state transition
            else:
                if self.logger:
                    self.logger.logger.warning(f"[!] Invalid V2V activation data")
            return None
        
        elif command_type == CommandType.DISABLE_V2V:
            # Handle V2V deactivation
            if hasattr(self.vehicle_logic, 'disable_v2v'):
                self.vehicle_logic.disable_v2v()
                if self.logger:
                    self.logger.logger.info(f"[OK] V2V disabled in {self.__class__.__name__}")
                return None  # No state transition
            return None
        
        # Default: Event not handled by this state
        if self.logger:
            self.logger.logger.info(f"🚫 Command '{command_type}' ignored in {self.__class__.__name__}")
        return None
    
    # === Helper Methods ===
    
    def should_transition_to_stopped(self, sensor_data: Dict[str, Any]) -> bool:
        """
        Common check for emergency stop conditions
        All states can use this to check for safety stops
        """
        # Check collision avoidance system
        if hasattr(self.vehicle_logic, 'collision_avoidance'):
            yolo_data = sensor_data.get('yolo_data', {})
            emergency_stop, _ = self.vehicle_logic.collision_avoidance.check_collision_risk(
                car_distance=yolo_data.get('car_dist'),
                person_distance=yolo_data.get('person_dist'),
                current_velocity=sensor_data.get('velocity', 0.0)
            )
            if emergency_stop:
                return True
        
        return False
    
    def validate_event_data(self, data: Dict[str, Any], required_fields: list) -> bool:
        """
        Validate that event data contains required fields
        
        Args:
            data: Event data dictionary
            required_fields: List of required field names
            
        Returns:
            bool: True if all required fields are present
        """
        for field in required_fields:
            if field not in data:
                if self.logger:
                    self.logger.logger.warning(f"Missing required field '{field}' in event data")
                return False
        return True