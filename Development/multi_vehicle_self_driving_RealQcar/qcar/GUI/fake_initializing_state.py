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
    TIMEOUT = 15.0  # Maximum initialization time (faster than real: 30.0s)
    
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
            
            # Initialize mock GPS reading (simulate _wait_for_gps)
            if not self._wait_for_mock_gps():
                return False
            
            # Initialize state estimator with mock GPS (matches real _initialize_state_estimator)
            if not self._initialize_state_estimator():
                return False
            
            self.logger.logger.info("Mock QCar hardware initialized with state estimation")
            return True
            
        except Exception as e:
            self.logger.log_error("Mock QCar initialization failed", e)
            traceback.print_exc()
            return False
    
    def _wait_for_mock_gps(self) -> bool:
        """Wait for initial mock GPS reading (matches real _wait_for_gps)"""
        self.logger.logger.info("Waiting for initial mock GPS reading...")
        
        # Mock GPS is always ready - just read once
        if self.vehicle_logic.gps.readGPS():
            self.init_pose = np.array([
                self.vehicle_logic.gps.position[0],
                self.vehicle_logic.gps.position[1],
                self.vehicle_logic.gps.orientation[2]
            ])
            self.logger.logger.info(
                f"Initial pose: x={self.init_pose[0]:.2f}, "
                f"y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f}"
            )
            return True
        
        self.logger.log_error("Mock GPS reading failed")
        return False
    
    def _initialize_state_estimator(self) -> bool:
        """Initialize local state estimator (matches real InitializingState)"""
        try:
            parent_fake_vehicle = getattr(self.vehicle_logic, '_parent_fake_vehicle', None)
            
            success = self.vehicle_logic.vehicle_observer.initialize_local_estimator(
                gps=parent_fake_vehicle.mock_gps,
                initial_pose=self.init_pose,
                estimator_params={'use_qcar_ekf': False}  # Use fallback EKF for simulation
            )
            
            if success:
                self.logger.logger.info(
                    f"Local estimator initialized at pose: "
                    f"x={self.init_pose[0]:.2f}, y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f}"
                )
            else:
                self.logger.log_error("Local estimator initialization failed")
            
            return success
            
        except Exception as e:
            self.logger.log_error("State estimator initialization failed", e)
            traceback.print_exc()
            return False
    
    def _initialize_mock_perception(self) -> bool:
        """Initialize mock perception systems (YOLO) - matches real _initialize_perception"""
        try:
            # For fake vehicles, YOLO is completely optional
            # Just disable YOLO and continue - no camera needed for simulation
            self.vehicle_logic.yolo_manager.yolo_enabled = False
            self.vehicle_logic.yolo_manager.yolo = None
            self.vehicle_logic.yolo_manager.yolo_drive = None
            
            self.logger.logger.info("Mock perception: YOLO disabled (not needed for fake vehicle)")
            return True
            
        except Exception as e:
            # YOLO is optional - log warning but continue
            self.logger.logger.warning(f"Mock perception initialization warning: {e}")
            return True
    
    def _initialize_telemetry(self) -> bool:
        """Initialize telemetry logging if enabled (matches real InitializingState)"""
        try:
            if self.config.logging.enable_telemetry_logging:
                self.vehicle_logic.logger.setup_telemetry_logging(self.config.logging.data_log_dir)
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
    
    def should_transition_to_stopped(self, sensor_data: Dict[str, Any]) -> bool:
        """Override emergency stop checks during fake initialization"""
        # During fake initialization, never trigger emergency stops
        # This allows the system to initialize properly with mock hardware
        return False
        
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