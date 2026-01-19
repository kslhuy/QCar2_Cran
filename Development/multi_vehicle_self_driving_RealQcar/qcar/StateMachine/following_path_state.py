"""
Following Path State - Simplified Event-Driven Implementation

Handles autonomous path following using predefined waypoints.
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



# Import LaneFusion for modular lane-assisted path following
try:
    from Controller.LaneFusion import (
        LaneFusion, 
        LaneFusionConfig, 
        FusionStrategy
    )
    from Controller.LaneFusion.config_loader_lane_fusion import get_lane_fusion_config
    LANE_FUSION_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: LaneFusion not available: {e}")
    LANE_FUSION_AVAILABLE = False
    LaneFusion = None
    LaneFusionConfig = None
    FusionStrategy = None
    get_lane_fusion_config = None


class FollowingPathState(StateBase):
    """Handler for FOLLOWING_PATH state with simplified event handling"""
    
    def __init__(self, vehicle_logic):
        """Initialize the following path state"""
        super().__init__(vehicle_logic)
        
        # State-specific controllers (initialized when needed)
        self.steering_controller = None
        self.speed_controller = None
        self._controllers_initialized = False
        
        # Lane fusion system for combining waypoint steering with lane detection
        # Lane detection runs in yolo_server, results received via yolo_data dict
        self.lane_fusion = None
        self._lane_fusion_enabled = False
    
    def _init_controllers(self, force: bool = False):
        """
        Initialize controllers for this state using ControllerManager.
        Controllers are created and cached centrally in controller_manager.
        """
        # self.logger.logger.info("[PATH] Initializing controllers...")
        if self._controllers_initialized and not force:
            return
        
        # Get controllers from ControllerManager (creates them if needed)
        if hasattr(self.vehicle_logic, 'controller_manager'):
            cm = self.vehicle_logic.controller_manager
            
            # Speed controller (PID for path following)
            # TODO : Instead of get_speed_controller(), use get_longitudinal_controller()
            # Currently only PID is available for path following
            self.speed_controller = cm.get_speed_controller()
            if self.speed_controller:
                self.vehicle_logic.speed_controller = self.speed_controller  # Backward compatibility
                self.logger.logger.info("[PATH] Speed controller obtained from ControllerManager")
            
            # Steering controller (uses waypoints from vehicle_logic)
            self.steering_controller = cm.get_steering_controller()
            if self.steering_controller:
                self.vehicle_logic.steering_controller = self.steering_controller  # Backward compatibility
                self.logger.logger.info("[PATH] Steering controller obtained from ControllerManager")
        else:
            self.logger.logger.warning("[PATH] ControllerManager not available")
        
        # Initialize Lane Fusion system for lane-assisted path following
        self._init_lane_fusion()
        
        self._controllers_initialized = True
    
    def _init_lane_fusion(self, config: dict = None):
        """
        Initialize the modular lane fusion system from YAML config.
        
        The lane fusion system combines waypoint-based steering with lane detection
        corrections to improve path following, especially with poor GPS.
        
        Configuration is loaded from: Controller/LaneFusion/config_lane_fusion.yaml
        
        Args:
            config: Optional override configuration dict
        """
        if not LANE_FUSION_AVAILABLE:
            self.logger.logger.warning("[PATH] LaneFusion not available - lane assist disabled")
            return
        
        try:
            # Load configuration from YAML file
            yaml_config = get_lane_fusion_config()
            settings = yaml_config.get_fusion_settings()
            
            # Check if enabled in config
            if not settings.enabled:
                self.logger.logger.info("[PATH] Lane Fusion disabled in config")
                self._lane_fusion_enabled = False
                return
            
            # Build config dict from YAML settings
            fusion_settings = {
                'strategy': settings.strategy,
                'max_lane_weight': settings.max_lane_weight,
                'min_confidence': settings.min_confidence,
                'lane_gain': settings.lane_gain,
                'smoothing_factor': settings.smoothing_factor,
                'max_steering': settings.max_steering,
                'deadband': settings.deadband,
                'switch_threshold': settings.switch_threshold,
                'enable_curvature_compensation': settings.enable_curvature_compensation,
                'debug_logging': settings.debug_logging
            }
            
            # Override with provided config (for programmatic changes)
            if config:
                fusion_settings.update(config)
            
            # Create fusion config
            fusion_config = LaneFusionConfig(
                strategy=FusionStrategy(fusion_settings['strategy']),
                max_lane_weight=fusion_settings['max_lane_weight'],
                min_confidence=fusion_settings['min_confidence'],
                lane_gain=fusion_settings['lane_gain'],
                smoothing_factor=fusion_settings['smoothing_factor'],
                max_steering=fusion_settings['max_steering'],
                deadband=fusion_settings['deadband'],
                switch_threshold=fusion_settings['switch_threshold'],
                enable_curvature_compensation=fusion_settings['enable_curvature_compensation'],
                debug_logging=fusion_settings['debug_logging']
            )
            
            # Create lane fusion instance
            self.lane_fusion = LaneFusion(config=fusion_config, logger=self.logger)
            self._lane_fusion_enabled = True
            
            self.logger.logger.info(
                f"[PATH] Lane Fusion initialized from YAML: strategy={fusion_settings['strategy']}, "
                f"max_weight={fusion_settings['max_lane_weight']}, min_conf={fusion_settings['min_confidence']}"
            )
            
        except Exception as e:
            self.logger.log_error("Failed to initialize Lane Fusion", e)
            self._lane_fusion_enabled = False
    
    def configure_lane_fusion(self, **kwargs):
        """
        Configure lane fusion parameters at runtime.
        
        Args:
            strategy: 'adaptive', 'weighted', 'cascaded', 'switch', 'lane_priority'
            max_lane_weight: Maximum lane steering influence (0.0-1.0)
            min_confidence: Minimum lane confidence to use (0.0-1.0)
            lane_gain: Gain applied to lane steering correction
            enabled: Enable/disable lane fusion
            
        Example:
            following_state.configure_lane_fusion(
                strategy='cascaded',
                max_lane_weight=0.5,
                enabled=True
            )
        """
        if 'enabled' in kwargs:
            self._lane_fusion_enabled = kwargs.pop('enabled')
            self.logger.logger.info(f"[PATH] Lane fusion enabled: {self._lane_fusion_enabled}")
        
        if self.lane_fusion is None:
            return
        
        # Update individual parameters
        for key, value in kwargs.items():
            if key == 'strategy':
                self.lane_fusion.config.strategy = FusionStrategy(value)
            elif hasattr(self.lane_fusion.config, key):
                setattr(self.lane_fusion.config, key, value)
        
        self.logger.logger.info(f"[PATH] Lane fusion config updated: {kwargs}")
    
    def update_path(self, new_waypoint_sequence: np.ndarray):
        """
        Update the waypoint sequence and reset the steering controller
        
        Args:
            new_waypoint_sequence: New waypoint sequence to follow
        """
        try:
            # Update vehicle logic waypoint sequence
            self.vehicle_logic.waypoint_sequence = new_waypoint_sequence
            
            # Reset steering controller with new waypoints
            if self.steering_controller:
                self.steering_controller.reset(new_waypoint_sequence)
                self.logger.logger.info("[PATH] Steering controller updated with new path")
            else:
                # Try to initialize if not already done
                self._init_controllers()
                
        except Exception as e:
            self.logger.log_error("Failed to update steering controller path", e)
    
    def enter(self) -> bool:
        """Initialize path following mode"""
        super().enter()
        self.logger.logger.info("[PATH] Entering FOLLOWING_PATH state")
        
        # Initialize controllers if not already done
        self._init_controllers()
        
        # Initialize state data
        self.state_data = {
            'session_start_time': time.time(),
            'lap_count': 0,
            'last_waypoint_index': 0,
            'navigation_to_start_completed': False,
            'path_following_active': False
        }
        
        # Reset speed controller integral to prevent windup
        if self.speed_controller:
            if hasattr(self.speed_controller, 'ei'):
                self.speed_controller.ei = 0
            elif hasattr(self.speed_controller, 'reset'):
                self.speed_controller.reset()
            self.logger.logger.info("Speed controller reset")
        elif hasattr(self.vehicle_logic, 'speed_controller'):
            # Fallback to vehicle_logic controller if state controller not available
            if hasattr(self.vehicle_logic.speed_controller, 'ei'):
                self.vehicle_logic.speed_controller.ei = 0
            elif hasattr(self.vehicle_logic.speed_controller, 'reset'):
                self.vehicle_logic.speed_controller.reset()
            self.logger.logger.info("Speed controller reset (fallback)")
        
        # Get current waypoint index for continuity
        if self.steering_controller:
            self.state_data['last_waypoint_index'] = self.steering_controller.get_waypoint_index()
            self.logger.logger.info(f"Continuing from waypoint index: {self.state_data['last_waypoint_index']}")
        else:
            self.logger.logger.warning("[PATH] No steering controller available")
        
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Update path following control"""
        
        # print(sensor_data)
        # Extract sensor data
        x = sensor_data['x']
        y = sensor_data['y']
        theta = sensor_data['theta']
        velocity = sensor_data['velocity']
        
        
        
        # Handle startup delay
        if self.vehicle_logic.elapsed_time() < self.config.timing.start_delay:
            return 0.0, 0.0, None
        
        # === CONTROL COMPUTATION ===
        
        # Speed control
        u = self._compute_speed_control(velocity, dt)
        
        # Steering control with lane fusion
        # yolo_data = self.vehicle_logic.yolo_manager.get_yolo_data() if hasattr(self.vehicle_logic, 'yolo_manager') else None
        yolo_data = sensor_data.get('yolo_data', None)
        if yolo_data and not yolo_data.get('is_valid', True):  # Stale data
            # Could skip lane fusion or use more conservative weight
            yolo_data = None
            
        delta = self._compute_steering_control(x, y, theta, velocity, yolo_data)
        
        # Monitor progress
        self._monitor_progress()
        
        # Periodic logging
        self._periodic_logging(x, y, theta, velocity)
        
        # # Control computation and periodic logging
        # u = 0.05
        # delta = 0
        return u, delta, None
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle events while path following
        
        Args:
            command_type: CommandType enum (e.g., CommandType.STOP, CommandType.ENABLE_PLATOON_LEADER)
            data: Optional event data
            
        Returns:
            Optional state transition
        """
        data = data or {}
        
        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            # Fallback to base class if CommandType not available
            self.logger.logger.warning(f"CommandType not available in FollowingPathState - using base handler for {command_type}")
            return super().handle_event(command_type, data)
        
        # Handle START_PLATOON command - check formation to decide state transition
        if command_type == CommandType.START_PLATOON:
            # Debug: Log received data
            self.logger.logger.info(f"[PLATOON] START_PLATOON received with data: {data}")
            
            if not self.validate_event_data(data, ['leader_id']):
                self.logger.logger.error("[PLATOON] Missing 'leader_id' in command data!")
                return None
            
            # ✅ CRITICAL: Check if platoon setup was completed first
            if not (hasattr(self.vehicle_logic, 'platoon_controller') and 
                    self.vehicle_logic.platoon_controller and
                    self.vehicle_logic.platoon_controller.setup_complete):
                self.logger.logger.warning(
                    "[PLATOON] START_PLATOON rejected - SETUP_PLATOON_FORMATION has not been received yet! "
                    "Please send SETUP_PLATOON_FORMATION command first before starting platoon."
                )
                return None
            
            leader_id = data.get('leader_id')
            
            # ✅ Check formation to determine if this vehicle should follow a leader
            is_leader = self.vehicle_logic.platoon_controller.is_leader
            my_position = getattr(self.vehicle_logic.platoon_controller, 'my_position', None)
            
            self.logger.logger.info(f"[PLATOON] START_PLATOON received (leader_id={leader_id})")
            self.logger.logger.info(f"[PLATOON] My formation: is_leader={is_leader}, position={my_position}")
            
            if is_leader:
                # Leaders stay in FOLLOWING_PATH state (they follow their path, not another vehicle)
                self.logger.logger.info(f"[PLATOON] I am LEADER - staying in FOLLOWING_PATH state")
                # Just enable platoon mode for leader
                self.vehicle_logic.platoon_controller.enabled = True
                return None  # No state transition - continue path following as leader
            
            else:
                # Followers transition to FOLLOWING_LEADER state
                self.logger.logger.info(f"[PLATOON] I am FOLLOWER-{my_position} - transitioning to FOLLOWING_LEADER state (following vehicle {leader_id})")
                
                # Enable platoon controller for follower mode
                self.vehicle_logic.platoon_controller.enable_as_follower()
                self.vehicle_logic.platoon_controller.leader_car_id = leader_id
                self.vehicle_logic.platoon_controller.enabled = True
                
                # Transition to following leader state
                return (VehicleState.FOLLOWING_LEADER, StateTransitionReason.START_COMMAND)
        
        # Handle path updates specific to this state
        elif command_type == CommandType.SET_PATH:
            node_sequence = data.get('node_sequence')
            if node_sequence and isinstance(node_sequence, list):
                # Generate new waypoint sequence from node sequence
                if (hasattr(self.vehicle_logic, 'roadmap') and 
                    self.vehicle_logic.roadmap):
                    try:
                        new_waypoints = self.vehicle_logic.roadmap.generate_path(node_sequence)
                        self.update_path(new_waypoints)
                        self.logger.logger.info(f"[OK] Path updated in {self.__class__.__name__}")
                        return None  # No state transition
                    except Exception as e:
                        self.logger.log_error("Failed to generate path from nodes", e)
                else:
                    self.logger.logger.warning("[!] No roadmap available for path generation")
            else:
                self.logger.logger.warning(f"[!] Invalid path update data: {data}")
            return None
        
        # Handle velocity updates immediately (no state change) - handled by base class
        
        # Let base class handle common events (stop, emergency_stop, set_velocity, etc.)
        return super().handle_event(command_type, data)
    
    def _should_follow_leader(self, sensor_data: Dict[str, Any]) -> bool:
        """Check if we should transition to following a leader"""
        yolo_data = sensor_data.get('yolo_data', {})
        
        # Check for cars ahead and platoon mode
        if (yolo_data.get('cars', False) and 
            hasattr(self.vehicle_logic, 'platoon_controller') and
            self.vehicle_logic.platoon_controller):
            
            car_distance = yolo_data.get('car_dist')
            if car_distance and car_distance < 3.0:  # Car within 3 meters
                # Check if platoon mode should be activated
                if getattr(self.vehicle_logic.platoon_controller, 'should_activate', lambda: False)():
                    return True
        
        return False
    
    def _check_navigation_to_start(self, x: float, y: float) -> bool:
        """Check if navigation to start position is completed"""
        # If we don't have a steering controller, try to initialize first
        if not self.steering_controller:
            self._init_controllers()
            
        # If still no steering controller, assume we're ready
        if not self.steering_controller:
            return True
        
        # Check distance to first waypoint of the planned path
        try:
            target_wp = self.steering_controller.wp[:, 0]
            current_pos = np.array([x, y])
            dist_to_target = np.linalg.norm(target_wp - current_pos)
            
            if dist_to_target < 0.5:  # Within 50cm of start
                return True
        except:
            # If we can't check, assume navigation is complete
            return True
        
        return False
    
    def _compute_speed_control(self, velocity: float, dt: float) -> float:
        """Compute speed control command"""
        if not self.speed_controller:
            return 0.0
        
        # Apply YOLO adjustments to reference velocity
        yolo_gain = getattr(self.vehicle_logic, 'yolo_gain', 1.0)
        v_ref_adjusted = self.vehicle_logic.v_ref * yolo_gain
        # print (f"Adjusted v_ref: {v_ref_adjusted:.2f} m/s (YOLO gain: {yolo_gain:.2f})")
        return self.speed_controller.update(velocity, v_ref_adjusted, dt)
    
    def _extract_lane_data(self, yolo_data: dict) -> dict:
        """
        Extract and validate lane detection data from YOLO packet.
        
        This helper provides a clean interface to the lane data transmitted
        from yolo_server_virtual.py, handling missing/invalid data gracefully.
        
        Args:
            yolo_data: Dict from YOLO receiver with lane_* keys
            
        Returns:
            Structured dict with validated lane data:
            - valid: bool - Whether lane data is usable
            - confidence: float - Detection confidence [0-1]
            - steering: float - Suggested steering correction
            - curvature: float - Lane curvature proxy
            - offset: float - Lateral offset from center
            - left_detected: bool - Left lane marker visible
            - right_detected: bool - Right lane marker visible
        """
        if not yolo_data:
            return {'valid': False, 'confidence': 0.0, 'steering': 0.0,
                    'curvature': 0.0, 'offset': 0.0, 
                    'left_detected': False, 'right_detected': False}
        
        confidence = yolo_data.get('lane_confidence', 0.0)
        is_valid = confidence > 0.1  # Minimum threshold for usable lane data
        
        return {
            'valid': is_valid,
            'confidence': confidence,
            'steering': yolo_data.get('lane_steering', 0.0),
            'curvature': yolo_data.get('lane_slope', 0.0),  # slope used as curvature proxy
            'offset': yolo_data.get('lane_intercept', 0.0),
            'left_detected': yolo_data.get('lane_left_detected', False),
            'right_detected': yolo_data.get('lane_right_detected', False),
        }
    
    def _compute_steering_control(self, x: float, y: float, theta: float, velocity: float, 
                                     yolo_data: dict = None) -> float:
        """
        Compute steering control command with optional lane fusion.
        
        Lane detection runs in yolo_server_virtual.py (where the camera is).
        Results are sent here via UDP as yolo_data dict.
        LaneFusion converts this dict to standardized format for fusion.
        
        Args:
            x, y: Vehicle position
            theta: Vehicle heading
            velocity: Current velocity
            yolo_data: Dict with lane data from YOLO server
                      (keys: lane_confidence, lane_steering, lane_slope, lane_intercept,
                             lane_left_detected, lane_right_detected)
            
        Returns:
            Steering command in radians
        """
        if not self.vehicle_logic.controller_manager.config.enable_steering_control or not self.steering_controller:
            return 0.0
        
        # Primary: Waypoint-based steering (global path following)
        # Add lookahead point slightly ahead of vehicle
        lookahead_offset = 0.2  # meters
        p = np.array([x, y]) + np.array([np.cos(theta), np.sin(theta)]) * lookahead_offset
        waypoint_steering = self.steering_controller.update(p, theta, max(velocity, 0.1))
        
        # Secondary: Lane-based correction using modular LaneFusion system
        if self._lane_fusion_enabled and self.lane_fusion is not None:
            # LaneFusion internally converts yolo_data to LaneDetectionResult
            fusion_result = self.lane_fusion.compute_steering(
                waypoint_steering=waypoint_steering,
                yolo_data=yolo_data,
                velocity=velocity
            )
            final_steering = fusion_result.final_steering
            
            # Log lane fusion activity periodically (every ~0.5 sec at 200Hz)
            if fusion_result.lane_valid and self.vehicle_logic.loop_counter % 100 == 0:
                lane_info = self._extract_lane_data(yolo_data)
                lane_status = "L" if lane_info['left_detected'] else "-"
                lane_status += "R" if lane_info['right_detected'] else "-"
                self.logger.logger.info(
                    f"[LANE] [{lane_status}] conf={fusion_result.lane_confidence:.2f}, "
                    f"wp={waypoint_steering:.3f}, lane={fusion_result.lane_steering:.3f}, "
                    f"w={fusion_result.lane_weight_used:.2f}, final={final_steering:.3f}"
                )
        else:
            # Lane fusion disabled or not available - use waypoint steering only
            final_steering = waypoint_steering
        
        return np.clip(final_steering, -0.5, 0.5)
    
    def get_lane_fusion_stats(self) -> dict:
        """
        Get lane fusion statistics for monitoring and debugging.
        
        Returns:
            Dict with fusion statistics including lane usage rate
        """
        if self.lane_fusion is None:
            return {'enabled': False, 'lane_fusion_available': False}
        
        stats = self.lane_fusion.get_statistics()
        stats['enabled'] = self._lane_fusion_enabled
        stats['lane_fusion_available'] = True
        return stats
    
    def _monitor_progress(self):
        """Monitor waypoint progress and lap completion"""
        if not self.steering_controller:
            return
        
        current_waypoint_index = self.steering_controller.get_waypoint_index()
        
        if current_waypoint_index != self.state_data['last_waypoint_index']:
            self.state_data['last_waypoint_index'] = current_waypoint_index
            
            # Check if we've completed a lap
            if hasattr(self.vehicle_logic, 'waypoint_sequence'):
                total_waypoints = self.vehicle_logic.waypoint_sequence.shape[1]
                if (current_waypoint_index == 0 and 
                    self.state_data['last_waypoint_index'] > total_waypoints * 0.8):
                    
                    self.state_data['lap_count'] += 1
                    lap_time = time.time() - self.state_data['session_start_time']
                    self.logger.logger.info(f"🏁 Completed lap {self.state_data['lap_count']} in {lap_time:.1f}s")
                    self.state_data['session_start_time'] = time.time()  # Reset for next lap
    
    def _periodic_logging(self, x: float, y: float, theta: float, velocity: float):
        """Log performance metrics periodically"""
        if (self.steering_controller and
            hasattr(self.vehicle_logic, 'loop_counter')):
            
            if self.vehicle_logic.loop_counter % 200 == 0:  # Every second at 200Hz
                errors = self.steering_controller.get_errors()
                cross_track_error = errors[0]
                heading_error = errors[1]
                
                self.logger.logger.debug(
                    f"Path following - CTE: {cross_track_error:.3f}m, "
                    f"HE: {heading_error:.3f}rad, V: {velocity:.2f}m/s"
                )
    
    def exit(self):
        """Clean up when leaving path following state"""
        self.logger.logger.info("[PATH] Exiting FOLLOWING_PATH state")
        
        # Log final statistics
        session_time = self.get_time_in_state()
        self.logger.logger.info(f"Path following session duration: {session_time:.1f}s")
        
        if self.state_data['lap_count'] > 0:
            self.logger.logger.info(f"Completed {self.state_data['lap_count']} laps in this session")
        
        # Log final waypoint position
        if self.steering_controller:
            final_waypoint_index = self.steering_controller.get_waypoint_index()
            self.logger.logger.info(f"Final waypoint index: {final_waypoint_index}")
        
        super().exit()