"""
Stopped State - Simplified Event-Driven Implementation

Handles stopped vehicle state (safety stop, manual stop, emergency stop).
Can transition back to WAITING_FOR_START when cleared to resume.
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


class StoppedState(StateBase):
    """Handler for STOPPED state with simplified event handling"""
    
    def enter(self) -> bool:
        """Initialize stopped state"""
        super().enter()
        self.logger.logger.info("[STOP] Entering STOPPED state")
        
        # Initialize state data
        self.state_data = {
            'stop_reason': 'Unknown',
            'can_resume': True,
            'manual_stop': False,
            'safety_stop': False,
            'emergency_stop': False,
            'resume_commanded': False
        }
        
        # Ensure vehicle is actually stopped
        self._ensure_vehicle_stopped()
        
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle stopped state - no movement, check for resume conditions"""
        
        # Always send zero commands while stopped
        throttle, steering = 0.0, 0.0
        
        # # Check if we should resume operation (automatic safety checks)
        # if self._should_auto_resume(sensor_data):
        #     return throttle, steering, (VehicleState.FOLLOWING_PATH, StateTransitionReason.START_COMMAND)
        
        # Manual resume is now handled via handle_event method
        
        # Periodic status logging
        self._periodic_status_logging()
        
        # Stay stopped
        return throttle, steering, None
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle commands while stopped
        
        Args:
            command_type: CommandType enum (e.g., CommandType.START, CommandType.STOP)
            data: Optional event data
            
        Returns:
            Optional state transition
        """
        data = data or {}
        
        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            # Fallback to base class if CommandType not available
            return super().handle_event(command_type, data)
        
        # Handle manual control commands - transition to manual mode
        if command_type == CommandType.MANUAL_CONTROL:
            self.logger.logger.info("[STOP] Manual control command received - transitioning to MANUAL_MODE")
            return (VehicleState.MANUAL_MODE, StateTransitionReason.START_COMMAND)
        
        # Handle calibrate command - GPS-only recalibration without full reinitialization
        elif command_type == CommandType.CALIBRATE:
            self.logger.logger.info("GPS Calibration command received - performing GPS-only recalibration")
            if self._recalibrate_gps():
                self.logger.logger.info("✓ GPS recalibration completed successfully")
            else:
                self.logger.log_error("GPS recalibration failed")
            # Stay in STOPPED state - no transition needed
            return None
        
        # Handle initial position updates - set the vehicle's starting position (with optional GPS recalibration)
        elif command_type == CommandType.SET_INITIAL_POSITION:
            import numpy as np
            x = data.get('x')
            y = data.get('y')
            theta = data.get('theta', 0.0)
            calibrate = data.get('calibrate', False)  # Default to False for backward compatibility
            
            if x is not None and y is not None:
                try:
                    # Create initial pose array [x, y, theta]
                    initial_pose = np.array([float(x), float(y), float(theta)])
                    
                    if calibrate:
                        # Full GPS recalibration with new position
                        self.logger.logger.info(f"Setting initial position WITH GPS recalibration: ({x:.2f}, {y:.2f}, theta={theta:.2f})")
                    else:
                        # Just reset observer without GPS recalibration
                        self.logger.logger.info(f"Setting initial position WITHOUT GPS recalibration: ({x:.2f}, {y:.2f}, theta={theta:.2f})")
                    if self._recalibrate_gps(initial_pose, calibrate=calibrate):
                        self.logger.logger.info("✓ Initial position set successfully")
                    else:
                        self.logger.log_error("Failed to set initial position")
                            
                    
                    return None
                    
                except Exception as e:
                    self.logger.log_error(f"Failed to set initial position ({x}, {y}, {theta})", e)
                    return None
            else:
                self.logger.logger.warning(f"Invalid initial position data: x={x}, y={y}")
                return None
        
        # Handle start/resume command
        elif command_type == CommandType.START or command_type == CommandType.START_PLATOON:
            self.logger.logger.info(" Start/Resume command received")
            
            # NOTE: Platoon configuration (formation, position, role) is PRESERVED during stop.
            # The vehicle will resume with the same platoon setup it had before stopping.
            # Only the 'enabled' flag is toggled - formation data stays intact.
            
            return (VehicleState.WAITING_FOR_START, StateTransitionReason.START_COMMAND)

        # Handle repeated stop commands (acknowledge but stay stopped)
        elif command_type in [CommandType.STOP, CommandType.EMERGENCY_STOP]:
            self.logger.logger.info(f"[i] Already stopped - acknowledging {command_type}")
            return None
        
        # Handle other commands while stopped
        elif command_type == CommandType.SET_VELOCITY:
            v_ref = data.get('v_ref')
            if v_ref is not None and 0 <= v_ref <= 2.0:
                self.vehicle_logic.v_ref = v_ref
                self.logger.logger.info(f"Velocity preset to {v_ref} m/s for when movement resumes")
            return None
        
        # Use base class handler for other commands
        else:
            return super().handle_event(command_type, data)
    
    def _should_auto_resume(self, sensor_data: Dict[str, Any]) -> bool:
        """Check if conditions are met to automatically resume (emergency clears)"""
        # Only auto-resume if it was an emergency stop and emergency has cleared
        if self.state_data.get('emergency_stop', False):
            return self._is_emergency_cleared_now()
        return False
    
    def _is_emergency_cleared_now(self) -> bool:
        """Check if emergency conditions have been cleared right now"""
        if not hasattr(self.vehicle_logic, 'collision_avoidance'):
            return True  # If no collision avoidance, assume clear
        
        # This would need current sensor data - simplified for now
        return True  # Placeholder: Assume emergency cleared for demo purposes
    
    def _ensure_vehicle_stopped(self):
        """Ensure the vehicle hardware is actually stopped"""
        if hasattr(self.vehicle_logic, 'qcar'):
            try:
                self.vehicle_logic.qcar.write(throttle=0, steering=0)
            except:
                pass  # Hardware might not be available
        
        
    
    def _periodic_status_logging(self):
        """Log stopped status periodically"""
        if (hasattr(self.vehicle_logic, 'loop_counter') and
            self.vehicle_logic.loop_counter % 1000 == 0):  # Every 5 seconds at 200Hz
            
            stop_time = self.get_time_in_state()
            self.logger.logger.info(f"[STOP] Vehicle stopped for {stop_time:.1f}s - Reason: {self.state_data.get('stop_reason', 'Unknown')}")
    
    def set_stop_reason(self, reason: str, stop_type: str = 'manual'):
        """Set the reason for stopping and type"""
        self.state_data['stop_reason'] = reason
        
        if stop_type == 'emergency':
            self.state_data['emergency_stop'] = True
            self.state_data['safety_stop'] = True
        elif stop_type == 'safety':
            self.state_data['safety_stop'] = True
        else:
            self.state_data['manual_stop'] = True
    
    def exit(self):
        """Clean up when leaving stopped state"""
        self.logger.logger.info("Exiting STOPPED state")
        
        # Log stop statistics
        stop_time = self.get_time_in_state()
        self.logger.logger.info(f"Vehicle was stopped for {stop_time:.1f}s")
        self.logger.logger.info(f"Stop reason: {self.state_data.get('stop_reason', 'Unknown')}")
        
        # Clear any stop flags
        if hasattr(self.vehicle_logic, 'emergency_stop_commanded'):
            self.vehicle_logic.emergency_stop_commanded = False
        
        super().exit()