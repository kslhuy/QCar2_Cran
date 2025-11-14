"""
Following Leader State - Simplified Event-Driven Implementation

Handles following another vehicle (platoon/convoy mode).
Uses single handle_event method for command processing.
"""
import time
import numpy as np
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


class FollowingLeaderState(StateBase):
    """Handler for FOLLOWING_LEADER state with simplified event handling"""
    
    def enter(self) -> bool:
        """Initialize leader following mode"""
        super().enter()
        self.logger.logger.info("[FOLLOW] Entering FOLLOWING_LEADER state")
        
        # Initialize state data
        self.state_data = {
            'session_start_time': time.time(),
            'leader_detected': True,
            'leader_distance': None,
            'target_distance': 2.0,  # Default following distance in meters
            'formation_stable': False,
            'last_leader_detection_time': time.time(),
            'leader_lost_timeout': 3.0,  # Time before giving up on leader
            'formation_timeout': 10.0,  # Time to establish formation
            'spacing_stable_threshold': 0.5,  # Distance tolerance for stable spacing
            'speed_adjustment_factor': 0.3  # How aggressively to adjust speed for spacing
        }
        
        # Reset speed controller integral to prevent windup
        if hasattr(self.vehicle_logic, 'speed_controller'):
            self.vehicle_logic.speed_controller.ei = 0
            self.logger.logger.info("Speed controller integral reset for leader following")
        
        # Configure platoon controller if available
        if hasattr(self.vehicle_logic, 'platoon_controller'):
            self.vehicle_logic.platoon_controller.enable_as_follower()
            self.logger.logger.info("Platoon controller configured as follower")
        
        self.logger.logger.info(f"Target following distance: {self.state_data['target_distance']:.1f}m")
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Update leader following control"""
        
        # Extract sensor data
        x = sensor_data['x']
        y = sensor_data['y']
        theta = sensor_data['theta']
        velocity = sensor_data['velocity']
        yolo_data = sensor_data.get('yolo_data', {})
        
        # Check if emergency stop requested
        if self.should_transition_to_stopped(sensor_data):
            return 0.0, 0.0, (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # Check for stop command - now handled via events
        # NOTE: Stop commands now come through handle_event
        
        # Update leader detection status
        self._update_leader_detection(yolo_data)
        
        # Check if leader is lost
        if self._is_leader_lost():
            self.logger.log_warning("👻 Leader lost, returning to path following")
            return 0.0, 0.0, (VehicleState.FOLLOWING_PATH, StateTransitionReason.LEADER_LOST)
        
        # Check formation timeout
        if self._is_formation_timeout():
            self.logger.log_warning("⏰ Formation timeout, returning to path following")
            return 0.0, 0.0, (VehicleState.FOLLOWING_PATH, StateTransitionReason.LEADER_LOST)
        
        # Check startup delay
        if self.vehicle_logic.elapsed_time() < self.config.timing.start_delay:
            return 0.0, 0.0, None
        
        # === CONTROL COMPUTATION ===
        
        # Compute following control
        u, delta = self._compute_following_control(x, y, theta, velocity, dt)
        
        # Monitor formation status
        self._monitor_formation_status()
        
        # Periodic logging
        self._periodic_logging(velocity)
        
        return u, delta, None
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle events while following leader
        
        Args:
            command_type: CommandType enum (e.g., CommandType.STOP, CommandType.DISABLE_PLATOON)
            data: Optional event data
            
        Returns:
            Optional state transition
        """
        data = data or {}
        
        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            # Fallback to base class if CommandType not available
            return super().handle_event(command_type, data)
        
        # Handle platoon disable command
        if command_type == CommandType.DISABLE_PLATOON:
            self.logger.logger.info("🔗 Disabling platoon mode - returning to path following")
            if hasattr(self.vehicle_logic, 'platoon_controller'):
                self.vehicle_logic.platoon_controller.disable()
            return (VehicleState.FOLLOWING_PATH, StateTransitionReason.PLATOON_COMMAND)
        
        # Handle velocity updates immediately (no state change) - handled by base class
        
        # Let base class handle common events (stop, emergency_stop, set_velocity, etc.)
        return super().handle_event(command_type, data)
    
    def _check_stop_command(self) -> bool:
        """Check if stop command has been received"""
        return self.check_stop_command()
    
    def _update_leader_detection(self, yolo_data: Dict[str, Any]):
        """Update leader detection status and distance"""
        cars_detected = yolo_data.get('cars', False)
        car_distance = yolo_data.get('car_dist')
        
        if cars_detected and car_distance is not None:
            # Leader is visible
            self.state_data['leader_detected'] = True
            self.state_data['leader_distance'] = car_distance
            self.state_data['last_leader_detection_time'] = time.time()
        else:
            # No leader detected
            self.state_data['leader_detected'] = False
            self.state_data['leader_distance'] = None
    
    def _is_leader_lost(self) -> bool:
        """Check if leader has been lost for too long"""
        if not self.state_data['leader_detected']:
            time_since_detection = time.time() - self.state_data['last_leader_detection_time']
            return time_since_detection > self.state_data['leader_lost_timeout']
        return False
    
    def _is_formation_timeout(self) -> bool:
        """Check if formation establishment has timed out"""
        formation_time = self.get_time_in_state()
        return formation_time > self.state_data['formation_timeout'] and not self.state_data['formation_stable']
    
    def _compute_following_control(self, x: float, y: float, theta: float, velocity: float, dt: float) -> Tuple[float, float]:
        """Compute control commands for leader following"""
        
        # Speed control with spacing adjustment
        u = self._compute_speed_with_spacing(velocity, dt)
        
        # Steering control - use path following as base if available
        delta = self._compute_steering_control(x, y, theta, velocity)
        
        return u, delta
    
    def _compute_speed_with_spacing(self, velocity: float, dt: float) -> float:
        """Compute speed control with spacing adjustment"""
        if not hasattr(self.vehicle_logic, 'speed_controller'):
            return 0.0
        
        # Base speed reference
        base_v_ref = self.vehicle_logic.v_ref
        
        # Apply YOLO adjustments
        yolo_gain = getattr(self.vehicle_logic, 'yolo_gain', 1.0)
        v_ref_adjusted = base_v_ref * yolo_gain
        
        # Spacing adjustment
        if self.state_data['leader_distance'] is not None:
            distance_error = self.state_data['leader_distance'] - self.state_data['target_distance']
            
            # If too close, slow down; if too far, speed up
            spacing_adjustment = distance_error * self.state_data['speed_adjustment_factor']
            v_ref_adjusted += spacing_adjustment
            
            # Limit speed adjustments
            v_ref_adjusted = max(0.1, min(v_ref_adjusted, base_v_ref * 1.2))
        
        return self.vehicle_logic.speed_controller.update(velocity, v_ref_adjusted, dt)
    
    def _compute_steering_control(self, x: float, y: float, theta: float, velocity: float) -> float:
        """Compute steering control for leader following"""
        
        # For now, use the same path following steering if available
        # In a more advanced implementation, this could include leader tracking
        if (self.config.steering.enable_steering_control and
            hasattr(self.vehicle_logic, 'steering_controller') and
            self.vehicle_logic.steering_controller):
            
            # Use look-ahead point
            p = np.array([x, y]) + np.array([np.cos(theta), np.sin(theta)]) * 0.2
            return self.vehicle_logic.steering_controller.update(p, theta, max(velocity, 0.1))
        
        return 0.0
    
    def _monitor_formation_status(self):
        """Monitor and update formation status"""
        if self.state_data['leader_distance'] is not None:
            distance_error = abs(self.state_data['leader_distance'] - self.state_data['target_distance'])
            
            # Check if spacing is stable
            if distance_error < self.state_data['spacing_stable_threshold']:
                if not self.state_data['formation_stable']:
                    self.state_data['formation_stable'] = True
                    self.logger.logger.info(f"[OK] Formation stable at {self.state_data['leader_distance']:.2f}m")
            else:
                self.state_data['formation_stable'] = False
    
    def _periodic_logging(self, velocity: float):
        """Log leader following performance periodically"""
        if (hasattr(self.vehicle_logic, 'loop_counter') and
            self.vehicle_logic.loop_counter % 200 == 0):  # Every second at 200Hz
            
            leader_dist_str = f"{self.state_data['leader_distance']:.2f}m" if self.state_data['leader_distance'] else "N/A"
            formation_status = "[OK] STABLE" if self.state_data['formation_stable'] else "[STATE] FORMING"
            
            self.logger.logger.debug(
                f"Leader following - Distance: {leader_dist_str}, "
                f"Target: {self.state_data['target_distance']:.1f}m, "
                f"Status: {formation_status}, V: {velocity:.2f}m/s"
            )
    
    def exit(self):
        """Clean up when leaving leader following state"""
        self.logger.logger.info("[FOLLOW] Exiting FOLLOWING_LEADER state")
        
        # Log session statistics
        session_time = self.get_time_in_state()
        self.logger.logger.info(f"Leader following session duration: {session_time:.1f}s")
        
        if self.state_data['formation_stable']:
            self.logger.logger.info("Formation was established successfully")
        else:
            self.logger.logger.info("Formation was not fully established")
        
        # Disable platoon controller if available
        if hasattr(self.vehicle_logic, 'platoon_controller'):
            self.vehicle_logic.platoon_controller.disable()
            self.logger.logger.info("Platoon controller disabled")
        
        super().exit()