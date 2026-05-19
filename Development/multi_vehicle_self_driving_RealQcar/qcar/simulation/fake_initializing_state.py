"""
Simplified Initializing State for Fake Vehicle

This state quickly initializes the fake vehicle components 
and transitions to WAITING_FOR_START for testing.
"""
import time
import sys
import os
import numpy as np
from typing import Dict, Any, Tuple, Optional
from StateMachine.state_base import StateBase
from StateMachine.vehicle_state import VehicleState, StateTransitionReason
import traceback


# Add parent directory to fix import issues
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import CommandType with proper error handling - import from parent qcar directory
try:
    # command_handler.py is in the parent qcar directory
    import sys
    import os
    qcar_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # Go up to qcar directory
    if qcar_dir not in sys.path:
        sys.path.insert(0, qcar_dir)
    
    from command_handler import CommandType
    COMMAND_TYPE_AVAILABLE = True
    print(f"[+] CommandType imported successfully from {qcar_dir}")
except ImportError as e:
    print(f"INFO: CommandType not available in fake vehicle: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None


class FakeInitializingState(StateBase):
    """Handler for INITIALIZING state - simplified for fake vehicle (matches real flow)"""
    
    # Timing constants (faster than real init for testing)
    INITIAL_DELAY = 0.3  # Wait before starting initialization (faster than real: 1.0s)
    STEP_DELAY = 0.2  # Delay between initialization steps (faster than real: 0.5s)
    TIMEOUT = 30.0  # Maximum initialization time (faster than real: 30.0s)
    
    def enter(self) -> bool:
        """Initialize fake vehicle components (matches real InitializingState structure)"""
        super().enter()
        self.logger.logger.info("Entering FAKE INITIALIZING state")
        
        # Match real InitializingState state_data structure
        self.state_data = {
            'initialization_start': time.time(),
            'components_initialized': False,
            'ready_to_start': False,
            'last_step_time': time.time()
        }
        
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle fake initialization process (matches real InitializingState flow)"""
        throttle, steering = 0.0, 0.0
        current_time = time.time()
        elapsed_time = current_time - self.state_data['initialization_start']
        
        # Check for timeout (matches real InitializingState)
        if elapsed_time > self.TIMEOUT:
            self.logger.log_warning(f"[!] Fake initialization timeout ({self.TIMEOUT}s), proceeding anyway")
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        # Wait for initial system settle time (matches real InitializingState)
        if elapsed_time < self.INITIAL_DELAY:
            return throttle, steering, None
        
        # Initialize components (matches real InitializingState)
        if not self.state_data['components_initialized']:
            if self._initialize_all_components():
                self.state_data['components_initialized'] = True
                self.state_data['last_step_time'] = current_time
                self.logger.logger.info("[STEP 1/2] All fake components initialized")
                time.sleep(0.1)  # Faster settle time for testing
            else:
                self._log_initialization_progress(elapsed_time)
                return throttle, steering, None
        
        # Finalize and transition (matches real InitializingState)
        if self.state_data['components_initialized'] and not self.state_data['ready_to_start']:
            if current_time - self.state_data['last_step_time'] < self.STEP_DELAY:
                return throttle, steering, None
            
            self.state_data['ready_to_start'] = True
            self.logger.logger.info("[STEP 2/2] Fake system ready to start")
            time.sleep(0.1)  # Faster settle time for testing
        
        # Transition to WAITING_FOR_START (matches real InitializingState)
        if self.state_data['ready_to_start']:
            self.logger.logger.info(f"[SUCCESS] Fake initialization complete in {elapsed_time:.1f}s")
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        return throttle, steering, None
    
    def _log_initialization_progress(self, elapsed_time: float):
        """Log initialization progress (matches real InitializingState)"""
        if elapsed_time % 3.0 < 0.1:  # Log every 3 seconds (faster than real: 5s)
            self.logger.logger.info(f"[FAKE INIT] Component initialization in progress... ({elapsed_time:.1f}s elapsed)")
    
    def handle_event(self, command_type, data: Dict[str, Any] = None):
        """Handle commands during fake initialization"""
        if COMMAND_TYPE_AVAILABLE and CommandType:
            if command_type == CommandType.EMERGENCY_STOP:
                print(f"[!] FakeInitializingState: Emergency stop during initialization")
                return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
            
            # Fake vehicles ignore perception commands (no real camera/YOLO)
            if command_type in [CommandType.ACTIVATE_PERCEPTION, CommandType.DISABLE_PERCEPTION]:
                print(f"[-] FakeInitializingState: Ignoring perception command (fake vehicle)")
                return None
        
        # Ignore other commands during initialization
        print(f"[-] FakeInitializingState: Ignoring command during initialization")
        return None
    
    def _initialize_all_components(self) -> bool:
        """Initialize all required fake components in sequence (matches real InitializingState)"""
        initialization_steps = [
            ("Path planning", self._initialize_path_planning, 0.1),
            ("Mock QCar hardware", self._initialize_mock_qcar, 0.1),
            ("Mock perception (YOLO)", self._initialize_mock_perception, 0.1),
            ("Telemetry logging", self._initialize_telemetry, 0.05)
        ]
        
        try:
            self.logger.logger.info("Starting fake component initialization...")
            
            for idx, (component_name, init_func, settle_time) in enumerate(initialization_steps, 1):
                if not init_func():
                    self.logger.log_error(f"{component_name} initialization failed")
                    return False
                
                self.logger.logger.info(f"  [{idx}/{len(initialization_steps)}] {component_name} ready")
                time.sleep(settle_time)  # Faster settle times for testing
            
            self.logger.logger.info(f"All {len(initialization_steps)} fake components initialized successfully!")
            return True
            
        except Exception as e:
            self.logger.log_error("Fake component initialization failed", e)
            traceback.print_exc()
            return False
    
    def _initialize_mock_qcar(self) -> bool:
        """Initialize mock QCar hardware and GPS (matches real _initialize_qcar structure)"""
        try:
            # Get the parent fake vehicle instance to access mock components
            parent_fake_vehicle = getattr(self.vehicle_logic, '_parent_fake_vehicle', None)
            
            if not parent_fake_vehicle:
                self.logger.log_error("Parent fake vehicle not found!")
                return False
            
            # Inject mock hardware from the fake vehicle
            self.vehicle_logic.qcar = parent_fake_vehicle.mock_qcar
            self.vehicle_logic.gps = parent_fake_vehicle.mock_gps
            
            # Disable YOLO for fake vehicles (no camera/perception)
            if hasattr(self.vehicle_logic, 'yolo_manager'):
                self.vehicle_logic.yolo_manager.yolo_enabled = False
                self.vehicle_logic.yolo_manager.yolo = None
                self.vehicle_logic.yolo_manager.yolo_drive = None
                self.logger.logger.info("Mock perception disabled (fake vehicle has no camera)")
            
            self.init_pose = self.vehicle_logic.gps.last_data
            self.logger.logger.info(
                f"Initial pose: x={self.init_pose[0]:.2f}, "
                f"y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f}"
            )

            
            # Initialize state estimator with mock GPS (matches real _initialize_state_estimator)
            if not self._initialize_state_estimator():
                return False
            
            self.logger.logger.info("Mock QCar hardware initialized with state estimation")
            return True
            
        except Exception as e:
            self.logger.log_error("Mock QCar initialization failed", e)
            traceback.print_exc()
            return False
    

    
    def _initialize_state_estimator(self) -> bool:
        """Initialize local state estimator (matches real InitializingState)"""
        try:
            parent_fake_vehicle = getattr(self.vehicle_logic, '_parent_fake_vehicle', None)
            
            # Retrieve disturbance mode from mock vehicle to ensure observer matches simulation
            disturbance_mode = parent_fake_vehicle.mock_qcar.disturbance_mode
            
            success = self.vehicle_logic.vehicle_observer.initialize_local_estimator(
                gps=parent_fake_vehicle.mock_gps,
                initial_pose=self.init_pose,
                estimator_params={
                    'use_qcar_ekf': False, # Use fallback EKF for simulation
                    'disturbance_mode': disturbance_mode 
                }  
            )
            
            if success:
                self.logger.logger.info(
                    f"Local estimator initialized at pose: "
                    f"x={self.init_pose[0]:.2f}, y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f} "
                    f"(Mode: {disturbance_mode})"
                )
            else:
                self.logger.log_error("Local estimator initialization failed")
            
            return success
            
        except Exception as e:
            self.logger.log_error("State estimator initialization failed", e)
            traceback.print_exc()
            return False
  
    
    def _initialize_telemetry(self) -> bool:
        """Initialize telemetry logging if enabled (matches real InitializingState)"""
        try:
            logging_cfg = self.config.logging
            data_logging_enabled = any(
                bool(getattr(logging_cfg, attr, False))
                for attr in (
                    "enable_telemetry_logging",
                    "enable_fleet_estimation_logging",
                    "enable_local_estimation_logging",
                    "enable_following_leader_logging",
                    "enable_trust_weight_logging",
                )
            )
            if data_logging_enabled:
                self.vehicle_logic.logger.setup_telemetry_logging(
                    logging_cfg.data_log_dir,
                    enable_telemetry=bool(
                        getattr(logging_cfg, "enable_telemetry_logging", True)
                    ),
                )
            return True
        except Exception as e:
            self.logger.log_error("Telemetry initialization failed", e)
            return False
    
    def _initialize_path_planning(self) -> bool:
        """Initialize path planning system (matches real InitializingState)"""
        try:
            from hal.products.mats import SDCSRoadMap
            
            # Create roadmap
            self.vehicle_logic.roadmap = SDCSRoadMap(
                leftHandTraffic=self.config.path.left_hand_traffic,
                useSmallMap=True
            )
            
            # Set node sequence
            self.vehicle_logic.node_sequence = self.config.path.valid_nodes
            print(f"      Node sequence ({len(self.vehicle_logic.node_sequence)} nodes): {self.vehicle_logic.node_sequence}")
            
            # Generate and validate waypoint sequence
            waypoints = self.vehicle_logic.roadmap.generate_path(self.vehicle_logic.node_sequence)
            if not self._validate_waypoint_sequence(waypoints):
                print(f"      [!] Invalid waypoint sequence generated")
                return False
            
            self.vehicle_logic.waypoint_sequence = waypoints
            print(f"      Generated path with {waypoints.shape[1]} waypoints")
            return True
            
        except Exception as e:
            print(f"      [!] Path planning initialization failed: {e}")
            traceback.print_exc()
            return False
    
    def _validate_waypoint_sequence(self, waypoints) -> bool:
        """Validate generated waypoint sequence (matches real InitializingState)"""
        if waypoints is None:
            print(f"      [!] Failed to generate path for nodes: {self.vehicle_logic.node_sequence}")
            return False
        
        if not isinstance(waypoints, np.ndarray):
            print(f"      [!] Invalid waypoint type: {type(waypoints)}")
            return False
        
        if waypoints.shape[0] < 2 or waypoints.shape[1] < 2:
            print(f"      [!] Invalid waypoint shape: {waypoints.shape}")
            return False
        
        return True
    

        
    def _check_initial_position(self, sensor_data: Dict[str, Any]) -> bool:
        """Override position check - fake vehicles start at valid positions"""
        print(f"[*] FakeInitializingState: Skipping position check for fake vehicle")
        return True
    
    def exit(self):
        """Clean up initialization state"""
        self.logger.logger.info("Exiting FAKE INITIALIZING state")
        
        # Log initialization summary
        init_time = self.get_time_in_state()
        self.logger.logger.info(f"Fake initialization completed in {init_time:.1f}s")
        
        super().exit()
