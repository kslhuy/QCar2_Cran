"""
Waiting for Start State - Event-Driven Implementation

Handles the ready state where vehicle waits for start command.
Uses single handle_event method with direct transitions.
Much simpler than the previous approach!
"""
import time
from typing import Dict, Any, Tuple, Optional
from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason

# Import CommandType once at module level
import sys
import os

# Add parent directory to sys.path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

try:
    from command_handler import CommandType
    COMMAND_TYPE_AVAILABLE = True
except ImportError as e:
    print(f"ERROR: Cannot import CommandType: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None


class WaitingForStartState(StateBase):
    """Handler for WAITING_FOR_START state with direct event transitions"""
    
    def enter(self) -> bool:
        """Initialize waiting for start state"""
        super().enter()
        self.logger.logger.info("Entering WAITING_FOR_START state")
        
        # Initialize state data
        self.state_data = {
            'ready_for_commands': True,
            'auto_start_delay': self.config.timing.start_delay if hasattr(self.config, 'timing') else 1.0,
            'auto_start_enabled': False,  # Can be enabled for testing
            'platoon_mode_requested': False,
            'platoon_leader_id': None
        }
        

        
        self.logger.logger.info("Ready for start command")
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle waiting for start commands"""
        
        # No movement while waiting
        throttle, steering = 0.0, 0.0
        
        # Check for emergency stop conditions first (handled by base class)
        emergency_transition = super().update(dt, sensor_data)
        if emergency_transition[2]:
            return emergency_transition
        
        # Optional: Auto-start logic (disabled by default)
        if self.state_data['auto_start_enabled']:
            wait_time = self.get_time_in_state()
            if wait_time > self.state_data['auto_start_delay']:
                self.logger.logger.info("🤖 Auto-start triggered")
                return throttle, steering, (VehicleState.FOLLOWING_PATH, StateTransitionReason.START_COMMAND)
        
        # Stay in waiting state
        return throttle, steering, None
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle command types with direct transitions
        
        Args:
            command_type: CommandType enum (e.g., CommandType.START, CommandType.STOP)
            data: Optional event data
            
        Returns:
            Optional[Tuple[VehicleState, StateTransitionReason]]: Transition if needed
        """
        data = data or {}
        
        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            # Fallback to base class if CommandType not available
            return super().handle_event(command_type, data)
        
        if not self.state_data['ready_for_commands']:
            if self.logger:
                self.logger.logger.warning(f"[!] Ignoring '{command_type}' command - state not ready")
            return None
        
        # Handle start command - DIRECT TRANSITION!
        if command_type == CommandType.START:
            self.logger.logger.info("🚀 Start command received - transitioning immediately!")
            
            # Check if we should start in platoon mode
            if self.state_data['platoon_mode_requested']:
                # Setup platoon mode
                if hasattr(self.vehicle_logic, 'platoon_controller'):
                    self.vehicle_logic.platoon_controller.enable_follower_mode(
                        self.state_data['platoon_leader_id']
                    )
                return (VehicleState.FOLLOWING_LEADER, StateTransitionReason.START_COMMAND)
            else:
                # Start in regular path following mode
                return (VehicleState.FOLLOWING_PATH, StateTransitionReason.START_COMMAND)
        
        # Handle platoon follower setup
        elif command_type == CommandType.ENABLE_PLATOON_FOLLOWER:
            if not self.validate_event_data(data, ['leader_id']):
                return None
            
            leader_id = data.get('leader_id')
            self.logger.logger.info(f"🔗 Platoon follower mode configured (leader: {leader_id})")
            
            # Setup platoon configuration but don't start yet - wait for start command
            self.state_data['platoon_mode_requested'] = True
            self.state_data['platoon_leader_id'] = leader_id
            
            # Configure platoon controller if available
            if hasattr(self.vehicle_logic, 'platoon_controller'):
                self.vehicle_logic.platoon_controller.configure_follower(leader_id)
            
            self.logger.logger.info("📋 Platoon mode configured - waiting for start command")
            return None  # No transition, just configuration
        
        # Handle velocity updates - store for when we start
        elif command_type == CommandType.SET_VELOCITY:
            v_ref = data.get('v_ref')
            if v_ref is not None and 0 <= v_ref <= 2.0:
                if hasattr(self.vehicle_logic, 'v_ref'):
                    self.vehicle_logic.v_ref = v_ref
                    self.logger.logger.info(f"[OK] Velocity preset to {v_ref} m/s for when movement starts")
                    return None
            self.logger.logger.warning(f"[WARN] Invalid velocity update while waiting: {v_ref}")
            return None
        
        # Handle path updates - store for when we start  
        elif command_type == CommandType.SET_PATH:
            node_sequence = data.get('node_sequence')
            if node_sequence and isinstance(node_sequence, list):
                if hasattr(self.vehicle_logic, 'path_controller'):
                    self.vehicle_logic.path_controller.update_path(node_sequence)
                    self.logger.logger.info(f"✅ Path updated with {len(node_sequence)} nodes")
                    return None
            self.logger.logger.warning("⚠️ Invalid path update while waiting")
            return None
        
        # Let base class handle common events (stop, emergency_stop)
        return super().handle_event(command_type, data)
    
    def exit(self):
        """Clean up waiting state"""
        self.logger.logger.info("Exiting WAITING_FOR_START state")
        
        wait_time = self.get_time_in_state()
        mode = "platoon" if self.state_data['platoon_mode_requested'] else "autonomous"
        self.logger.logger.info(f"Waited for {wait_time:.1f}s before starting in {mode} mode")
        
        super().exit()