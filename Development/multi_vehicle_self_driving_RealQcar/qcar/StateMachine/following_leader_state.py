"""
Following Leader State - Clean and Simple Implementation

Handles following another vehicle (platoon/convoy mode).
Delegates all platoon logic to PlatoonController.

LATERAL CONTROL MODES:
  - 'pure_pursuit', 'stanley', 'lookahead', 'hybrid': Follow leader's position directly
  - 'path': Follow predefined waypoints (like FOLLOWING_PATH state) while maintaining
            longitudinal spacing with leader
            
USAGE:
  Set lateral_controller_type in config:
    - For leader tracking: lateral_controller_type = 'pure_pursuit' (or other)
    - For path following: lateral_controller_type = 'path'
"""
import time
import numpy as np
from typing import Dict, Any, Tuple, Optional
from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason

# Import CommandType
import sys
import os
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

# Import modular longitudinal controllers
try:
    from Controller.longitudinal_controllers import ControllerFactory
    LONGITUDINAL_CONTROLLER_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Cannot import longitudinal_controllers: {e}")
    LONGITUDINAL_CONTROLLER_AVAILABLE = False
    ControllerFactory = None

# Import modular lateral controllers
try:
    from Controller.lateral_controllers import LateralControllerFactory
    LATERAL_CONTROLLER_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Cannot import lateral_controllers: {e}")
    LATERAL_CONTROLLER_AVAILABLE = False
    LateralControllerFactory = None

# Import controller config loader
try:
    from Controller.config_controller_loader import get_controller_config
    CONFIG_LOADER_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Cannot import config_loader: {e}")
    CONFIG_LOADER_AVAILABLE = False
    get_controller_config = None

# Import SteeringController for path-following mode
try:
    from Controller.lateral_controllers import StanleyController
    STEERING_CONTROLLER_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Cannot import SteeringController: {e}")
    STEERING_CONTROLLER_AVAILABLE = False
    StanleyController = None


class FollowingLeaderState(StateBase):
    """Handler for FOLLOWING_LEADER state - simplified to use PlatoonController"""
    
    def __init__(self, vehicle_logic):
        """Initialize with configurable controller types"""
        super().__init__(vehicle_logic)
        
        # Load controller configuration from YAML (required)
        self.controller_config = get_controller_config()
        self.longitudinal_controller_type = self.controller_config.get_longitudinal_controller_type()
        self.lateral_controller_type = self.controller_config.get_lateral_controller_type()
        self.logger.logger.info(f"Loaded controller config: Long={self.longitudinal_controller_type}, Lat={self.lateral_controller_type}")
        
        self.longitudinal_controller = None
        self.lateral_controller = None
        self.steering_controller = None  # For path-following lateral mode
        
    def enter(self) -> bool:
        """Initialize leader following mode"""
        super().enter()
        print("[DEBUG] ===============================")
        print("[DEBUG] ENTERING FOLLOWING_LEADER STATE")
        print("[DEBUG] ===============================")
        self.logger.logger.info("[FOLLOW] Entering FOLLOWING_LEADER state")
        
        # Check if formation data exists and set up if missing
        if hasattr(self.vehicle_logic, 'platoon_controller'):
            self.vehicle_logic.platoon_controller.enable_as_follower()
            self.logger.logger.info("Platoon controller configured as follower")
        
        # Initialize controllers
        self._initialize_longitudinal_controller()
        self._initialize_lateral_controller()
        
        # Initialize path-following steering controller if in path mode
        if self.lateral_controller_type == 'path':
            self._initialize_steering_controller()
        
        return True
    
    def _initialize_longitudinal_controller(self):
        """Initialize the selected longitudinal controller from YAML config"""
        params = self.controller_config.get_longitudinal_params()
        
        self.longitudinal_controller = ControllerFactory.create(
            self.longitudinal_controller_type,
            params,
            logger=self.logger.logger
        )
        self.logger.logger.info(f"Initialized {self.longitudinal_controller_type.upper()} longitudinal controller from config")
    
    def _initialize_lateral_controller(self):
        """Initialize the selected lateral controller from YAML config"""
        # Skip factory creation if using path mode (uses SteeringController instead)
        if self.lateral_controller_type == 'path':
            self.logger.logger.info("Using path-following lateral control mode (SteeringController)")
            return
        
        params = self.controller_config.get_lateral_params()
        
        self.lateral_controller = LateralControllerFactory.create(
            self.lateral_controller_type,
            params,
            logger=self.logger.logger
        )
        self.logger.logger.info(f"Initialized {self.lateral_controller_type.upper()} lateral controller from config")
    
    def _initialize_steering_controller(self):
        """Initialize SteeringController for path-following lateral mode"""
        if not STEERING_CONTROLLER_AVAILABLE:
            self.logger.logger.error("SteeringController not available for path mode")
            return
        
        # Check if we have waypoint sequence available
        if not hasattr(self.vehicle_logic, 'waypoint_sequence') or self.vehicle_logic.waypoint_sequence is None:
            self.logger.logger.warning("[FOLLOW] Cannot initialize steering controller - no waypoint sequence")
            return
        
        try:
            # Initialize steering controller with current waypoint sequence
            self.steering_controller = StanleyController(
                waypoints=self.vehicle_logic.waypoint_sequence,
                config=self.controller_config,
                logger=self.logger,
                cyclic=True
            )
            self.logger.logger.info("[FOLLOW] Path-following steering controller initialized successfully")
        except Exception as e:
            self.logger.log_error("Failed to initialize path-following steering controller", e)
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Update leader following control using PlatoonController"""
        
        # Debug to see if this state is being called
        if hasattr(self.vehicle_logic, 'loop_counter') and self.vehicle_logic.loop_counter % 1000 == 0:
            print(f"[DEBUG] FollowingLeaderState.update called - loop_counter: {getattr(self.vehicle_logic, 'loop_counter', 'N/A')}")
        
        # # Check if emergency stop requested
        # if self.should_transition_to_stopped(sensor_data):
        #     return 0.0, 0.0, (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # Check startup delay
        if self.vehicle_logic.elapsed_time() < self.config.timing.start_delay:
            return 0.0, 0.0, None
        
        # Extract basic sensor data
        x, y, theta = sensor_data['x'], sensor_data['y'], sensor_data['theta']
        velocity = sensor_data['velocity']
        yolo_data = sensor_data.get('yolo_data', {})
        
        # Update Data for PlatoonController (perception and V2V data) 
        self._update_platoon_controller(yolo_data)
        
        # Get V2V leader data
        v2v_data = self._get_v2v_leader_data()
        
        # Compute control using PlatoonController
        u, delta = self._compute_control(dt , x, y, theta, velocity, v2v_data)
        
        # Enhanced periodic logging for debugging
        self._log_status(velocity, u, v2v_data)
        
        return u, delta, None
    
    # ===== SIMPLIFIED HELPER METHODS =====
    
    def _update_platoon_controller(self, yolo_data: Dict[str, Any]):
        """Update PlatoonController with perception data"""
        if not hasattr(self.vehicle_logic, 'platoon_controller'):
            return
            
        pc = self.vehicle_logic.platoon_controller
        
        # Update with YOLO perception data
        if self.vehicle_logic.yolo_manager.yolo_enabled:
            cars_detected = yolo_data.get('cars', [])
            car_distance = yolo_data.get('car_dist')
            # Check if any car is detected (cars is an array, check if any element is non-zero)
            has_car = any(cars_detected) if isinstance(cars_detected, (list, tuple)) else bool(cars_detected)
            pc.update_leader_info(
                detected=has_car and car_distance is not None,
                distance=car_distance,
                velocity=None  # Velocity comes from V2V
            )
        
        # # Update with V2V velocity data
        # if hasattr(self.vehicle_logic, 'v2v_manager'):
        #     pc.update_leader_velocity_from_v2v(
        #         self.vehicle_logic.v2v_manager, 
        #         self.vehicle_logic.vehicle_id
        #     )
    
    def _get_v2v_leader_data(self) -> Optional[Dict[str, Any]]:
        """Get leader data from V2V"""
        if not hasattr(self.vehicle_logic, 'platoon_controller'):
            return None
        
        # Check if platoon controller is properly configured
        pc = self.vehicle_logic.platoon_controller
        
        # If this vehicle is actually the leader, it shouldn't be following anyone
        if hasattr(pc, 'is_leader') and pc.is_leader:
            self.logger.logger.warning("Leader vehicle trying to get V2V leader data")
            # TODO : Consider transitioning back to FOLLOWING_PATH state
            return None
        
        # TODO: Should have the choice to use data direct from v2v_manager or Estimated by Observer
        v2v_data = self.vehicle_logic.platoon_controller.get_direct_leader_data_from_v2v(
            self.vehicle_logic.v2v_manager,
            self.vehicle_logic.vehicle_id
        )
        
        # Log V2V data status occasionally for debugging
        if (hasattr(self.vehicle_logic, 'loop_counter') and 
            self.vehicle_logic.loop_counter % 1000 == 0):
            if v2v_data is not None:
                self.logger.logger.debug(f"[FOLLOW] V2V Leader data available")
            else:
                self.logger.logger.debug(f"[FOLLOW] No V2V leader data received")
        
        return v2v_data
    
    def _compute_control(self, dt: float, x: float, y: float, theta: float, velocity: float, v2v_data: Optional[Dict[str, Any]]) -> Tuple[float, float]:
        """Compute control commands using modular longitudinal and lateral controllers"""
        base_velocity = self.vehicle_logic.v_ref
        
        # No V2V data - stop
        if v2v_data is None:
            if hasattr(self.vehicle_logic, 'loop_counter') and self.vehicle_logic.loop_counter % 200 == 0:
                self.logger.logger.warning("[FOLLOW] V2V data is None - stopping")
            return 0.0, 0.0
        
        # Prepare follower state
        follower_state = {
            'x': x,
            'y': y,
            'theta': theta,
            'velocity': velocity,
            'target_velocity': base_velocity
        }
        
        # Prepare leader state from V2V data
        leader_state = {
            'x': v2v_data.get('x', 0.0),
            'y': v2v_data.get('y', 0.0),
            'theta': v2v_data.get('theta', 0.0),
            'velocity': v2v_data.get('velocity', 0.0)
        }
        
       
        # Compute throttle using modular longitudinal controller
        u = self.longitudinal_controller.compute_throttle(follower_state, leader_state, dt)
        
        # Compute steering based on lateral control mode
        if self.lateral_controller_type == 'path':
            # Path-following mode: use steering controller with waypoints
            delta = self._compute_path_steering(x, y, theta, velocity)
        else:
            # Leader-following mode: use lateral controller to follow leader position
            delta = self.lateral_controller.compute_steering(follower_state, leader_state, dt)
        
        # Log following leader control data
        self.logger.log_following_leader_control(
            follower_state=follower_state,
            leader_state=leader_state,
            u=u,
            delta=delta
        )
        
        return u, delta
    
    def _compute_path_steering(self, x: float, y: float, theta: float, velocity: float) -> float:
        """Compute steering control using path-following (like in FOLLOWING_PATH state)"""
        if not self.vehicle_logic.controller_config.enable_steering_control or not self.steering_controller:
            return 0.0
        
        # Use look-ahead point (0.2m forward from vehicle center)
        p = np.array([x, y]) + np.array([np.cos(theta), np.sin(theta)]) * 0.2
        return self.steering_controller.update(p, theta, max(velocity, 0.1))
    
    def _log_status(self, velocity: float, throttle_cmd: float = None, v2v_data: Optional[Dict[str, Any]] = None):
        """Enhanced periodic logging for debugging"""
        if (hasattr(self.vehicle_logic, 'loop_counter') and
            self.vehicle_logic.loop_counter % 200 == 0):  # Every second at 200Hz
            
            status = "unknown"
            leader_info = ""
            spacing_info = ""
            
            if hasattr(self.vehicle_logic, 'platoon_controller'):
                pc = self.vehicle_logic.platoon_controller
                
                # Check for V2V leader data first (preferred for followers)
                if v2v_data is not None:
                    leader_velocity = v2v_data.get('velocity', 0.0)
                    status = "following_v2v"
                    leader_info = f"L_vel: {leader_velocity:.2f}m/s"
                elif pc.is_spacing_stable():
                    status = "stable"
                elif pc.leader_detected:
                    status = f"following_yolo (dist: {pc.leader_distance:.2f}m)"
                else:
                    status = "no_leader"
            
            # Enhanced logging with control command details
            if throttle_cmd is not None:
                self.logger.logger.info(
                    f"[FOLLOW] {status} | V: {velocity:.2f}m/s | "
                    f"Throttle: {throttle_cmd:.3f} | {leader_info} | {spacing_info}"
                )
            else:
                self.logger.logger.info(f"[FOLLOW] Status: {status}, velocity: {velocity:.2f}m/s")
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """Handle events while following leader"""
        data = data or {}
        
        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            return super().handle_event(command_type, data)
        
        # Handle platoon disable command
        if command_type == CommandType.DISABLE_PLATOON:
            self.logger.logger.info("Disabling platoon mode - returning to path following")
            if hasattr(self.vehicle_logic, 'platoon_controller'):
                self.vehicle_logic.platoon_controller.disable()
            return (VehicleState.FOLLOWING_PATH, StateTransitionReason.PLATOON_COMMAND)
        
        # Let base class handle common events (stop, emergency_stop, set_velocity, etc.)
        return super().handle_event(command_type, data)
    
    def exit(self):
        """Clean up when leaving leader following state"""
        self.logger.logger.info("[FOLLOW] Exiting FOLLOWING_LEADER state")
        
        # Log session statistics
        session_time = self.get_time_in_state()
        self.logger.logger.info(f"Leader following session duration: {session_time:.1f}s")
        
        # Disable platoon controller if available
        if hasattr(self.vehicle_logic, 'platoon_controller'):
            if self.vehicle_logic.platoon_controller.is_spacing_stable():
                self.logger.logger.info("Formation was established successfully")
            else:
                self.logger.logger.info("Formation was not fully established")
            self.vehicle_logic.platoon_controller.disable()
        
        # Reset controllers
        self.longitudinal_controller.reset()
        if self.lateral_controller:
            self.lateral_controller.reset()
        self.logger.logger.info("Controllers reset")
        
        super().exit()