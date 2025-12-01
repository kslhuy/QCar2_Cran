"""
Initializing State - Simplified Event-Driven Implementation

Handles system initialization and setup.
Transitions to WAITING_FOR_START when initialization is complete.
"""
import time
from typing import Dict, Any, Tuple, Optional
from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason

# Import CommandType once at module level
import sys
import os

# import time
import numpy as np
from pal.products.qcar import QCar, QCarGPS

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


class InitializingState(StateBase):
    """Handler for INITIALIZING state with simplified event handling"""
    
    def enter(self) -> bool:
        """Initialize system components"""
        super().enter()
        self.logger.logger.info("Entering INITIALIZING state")
        
        # Reset any previous state
        self.state_data = {
            'initialization_start': time.time(),
            'components_initialized': False,
            'initial_position_checked': False,
            'ready_to_start': False,
            'last_step_time': time.time(),  # Track timing for delays
            'step_delay': 0.5  # 500ms delay between major steps
        }
        
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle initialization process"""
        
        # No control commands during initialization
        throttle, steering = 0.0, 0.0
        
        # # Check if emergency stop requested
        # if self.should_transition_to_stopped(sensor_data):
        #     return throttle, steering, (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # === INITIALIZATION SEQUENCE ===
        current_time = time.time()
        
        # Step 1: Check if all components are ready (with delay)
        if not self.state_data['components_initialized']:
            # Allow some time for system to settle before starting
            if current_time - self.state_data['initialization_start'] < 1.0:
                return throttle, steering, None  # Wait 1 second before starting
            
            components_ready = self._check_components_ready()
            if components_ready:
                self.state_data['components_initialized'] = True
                self.state_data['last_step_time'] = current_time
                self.logger.logger.info("✅ All components initialized")
                time.sleep(0.2)  # Small delay to let components settle
        
        # # Step 2: Check initial position (if path following enabled) - with delay
        # if (self.state_data['components_initialized'] and 
        #     not self.state_data['initial_position_checked']):
            
        #     # Wait for step delay before proceeding
        #     if current_time - self.state_data['last_step_time'] < self.state_data['step_delay']:
        #         return throttle, steering, None  # Wait for delay
            
        #     if self.config.steering.enable_steering_control:
        #         position_ok = self._check_initial_position(sensor_data)
        #         if position_ok:
        #             self.state_data['initial_position_checked'] = True
        #             self.state_data['last_step_time'] = current_time
        #             self.logger.logger.info("✅ Initial position verified")
        #             time.sleep(0.2)  # Small delay
        #     else:
        #         # Skip position check if steering control disabled
        #         self.state_data['initial_position_checked'] = True
        #         self.state_data['last_step_time'] = current_time
        #         self.logger.logger.info("✅ Position check skipped (steering disabled)")
        #         time.sleep(0.1)  # Small delay

        # Step 3: Mark as ready to start - with final delay
        if (self.state_data['components_initialized'] and 
            self.state_data['initial_position_checked'] and
            not self.state_data['ready_to_start']):
            
            # Wait for step delay before finalizing
            if current_time - self.state_data['last_step_time'] < self.state_data['step_delay']:
                return throttle, steering, None  # Wait for delay
            
            self.state_data['ready_to_start'] = True
            self.logger.logger.info("✅ System ready to start")
            time.sleep(0.3)  # Final delay before transition
        
        # Transition to WAITING_FOR_START when ready
        if self.state_data['ready_to_start']:
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        # Check for timeout (safety measure)
        initialization_time = time.time() - self.state_data['initialization_start']
        if initialization_time > 30.0:  # 30 second timeout
            self.logger.log_warning("[!] Initialization timeout, proceeding anyway")
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        return throttle, steering, None
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle commands during initialization
        
        Args:
            command_type: CommandType enum
            data: Optional event data
            
        Returns:
            Optional state transition
        """
        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            # Fallback to base class if CommandType not available
            return super().handle_event(command_type, data)
        
        # During initialization, only accept emergency stop
        if command_type == CommandType.EMERGENCY_STOP:
            self.logger.logger.warning("[!] Emergency stop during initialization")
            return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        # Ignore all other commands during initialization
        if self.logger:
            self.logger.logger.info(f"🚫 Ignoring '{command_type}' command during initialization")
        return None
    
    def _check_components_ready(self) -> bool:
        """Initialize and check if all required components are ready"""
        try:
            self.logger.logger.info("🔧 Starting component initialization...")
            
            # Initialize path planning
            if not self._initialize_path_planning():
                self.logger.log_error("Path planning initialization failed")
                return False
            time.sleep(0.2)  # Allow path planning to settle
            
            # # Initialize network (if enabled)
            # if not self._initialize_network_2_GroundStation():
            #     self.logger.log_error("Network initialization failed")
            #     return False
            # time.sleep(0.3)  # Allow network to establish connections
            
            # Initialize QCar hardware
            if not self._initialize_qcar():
                self.logger.log_error("QCar hardware initialization failed")
                return False
            time.sleep(0.2)  # Allow hardware to settle
            
            # Initialize controllers
            if not self._initialize_controllers():
                self.logger.log_error("Controllers initialization failed")
                return False
            time.sleep(0.2)  # Allow controllers to initialize
            
            # Initialize perception
            if not self._initialize_perception():
                self.logger.log_error("Perception initialization failed")
                return False
            time.sleep(0.2)  # Allow perception to start
            
            # Setup telemetry logging
            if self.config.logging.enable_telemetry_logging:
                self.vehicle_logic.logger.setup_telemetry_logging(self.config.logging.data_log_dir)
                time.sleep(0.1)  # Allow logging setup
            
            self.logger.logger.info("All components initialization complete")
            return True
            
        except Exception as e:
            self.logger.log_error("Component initialization failed", e)
            return False
    
    # def _check_initial_position(self, sensor_data: Dict[str, Any]) -> bool:
    #     """Check if vehicle is at appropriate starting position"""
    #     try:
    #         # Get current position
    #         x = sensor_data.get('x', 0.0)
    #         y = sensor_data.get('y', 0.0)
    #         theta = sensor_data.get('theta', 0.0)
            
    #         # For now, just check if we have valid position data
    #         # In a full implementation, this would check against start nodes
    #         if abs(x) > 100 or abs(y) > 100:  # Sanity check for valid coordinates
    #             self.logger.log_warning(f"Position seems invalid: ({x:.2f}, {y:.2f})")
    #             return False
            
    #         self.logger.logger.info(f"Initial position: ({x:.2f}, {y:.2f}, {theta:.2f})")
    #         return True
            
    #     except Exception as e:
    #         self.logger.log_error("Position check failed", e)
    #         return False
        
    def check_initial_position(self) -> bool:
        import numpy as np
        """Check if vehicle is at start position"""
        try:
            # For simplified state machine, we don't need complex transitions
            # Just check if we're at the start position
            if hasattr(self, 'roadmap') and self.roadmap and hasattr(self, 'node_sequence'):
                start_node_reached, init_waypoint_seq = self.roadmap.initial_check(
                    self.init_pose,
                    self.node_sequence,
                    self.waypoint_sequence
                )
                
                if not start_node_reached:
                    # Log detailed information about position mismatch
                    target_node = self.node_sequence[0]
                    target_pose = self.roadmap.get_node_pose(target_node).squeeze()
                    current_dist = np.linalg.norm(self.init_pose[:2] - target_pose[:2])
                    
                    self.vehicle_logger.log_warning("="*60)
                    self.vehicle_logger.log_warning("NOT AT START POSITION")
                    self.vehicle_logger.log_warning(f"  Current position: ({self.init_pose[0]:.2f}, {self.init_pose[1]:.2f}, {self.init_pose[2]:.2f})")
                    self.vehicle_logger.log_warning(f"  Target node: {target_node}")
                    self.vehicle_logger.log_warning(f"  Target position: ({target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f})")
                    self.vehicle_logger.log_warning(f"  Distance to start: {current_dist:.2f}m")
                    self.vehicle_logger.log_warning("="*60)
                    
                    # Update waypoint sequence to navigate to start
                    self.waypoint_sequence = init_waypoint_seq
                    if hasattr(self, 'steering_controller') and self.steering_controller:
                        self.steering_controller.reset(self.waypoint_sequence)
                    return False
                else:
                    self.vehicle_logger.logger.info("Vehicle is at start position")
                    return True
            else:
                # If no roadmap/steering, assume position is OK
                return True
            
        except Exception as e:
            self.vehicle_logger.log_error("Initial position check failed", e)
            return False
    
    def exit(self):
        """Clean up initialization state"""
        self.logger.logger.info("[INIT] Exiting INITIALIZING state")
        
        # Log initialization summary
        init_time = self.get_time_in_state()
        self.logger.logger.info(f"Initialization completed in {init_time:.1f}s")
        
        super().exit()
    
    # ========== INITIALIZATION METHODS ==========
    # These methods are moved from VehicleLogic to the state machine
    
    def _initialize_path_planning(self) -> bool:
        """Initialize path planning system"""
        try:
            if self.config.steering.enable_steering_control:
                from hal.products.mats import SDCSRoadMap
                import numpy as np
                
                self.vehicle_logic.roadmap = SDCSRoadMap(
                    leftHandTraffic=self.config.path.left_hand_traffic,
                    useSmallMap=True
                )
                
                # Get valid nodes for current configuration
                valid_nodes = self.config.path.valid_nodes
                self.vehicle_logic.node_sequence = valid_nodes
                self.logger.logger.info(f"Selected node sequence ({len(self.vehicle_logic.node_sequence)} nodes): {self.vehicle_logic.node_sequence}")
                
                # Generate waypoint sequence
                try:
                    self.vehicle_logic.waypoint_sequence = self.vehicle_logic.roadmap.generate_path(self.vehicle_logic.node_sequence)
                except Exception as path_error:
                    self.logger.log_error(
                        f"Roadmap.generate_path() failed for nodes {self.vehicle_logic.node_sequence}",
                        path_error
                    )
                    return False
                
                # Validate waypoint sequence
                if self.vehicle_logic.waypoint_sequence is None:
                    self.logger.log_error(
                        f"Failed to generate path for node sequence: {self.vehicle_logic.node_sequence}. "
                        f"Check if these nodes exist in your roadmap and can be connected."
                    )
                    return False
                
                if not isinstance(self.vehicle_logic.waypoint_sequence, np.ndarray):
                    self.logger.log_error(
                        f"Waypoint sequence has wrong type: {type(self.vehicle_logic.waypoint_sequence)}"
                    )
                    return False
                
                if self.vehicle_logic.waypoint_sequence.shape[0] < 2 or self.vehicle_logic.waypoint_sequence.shape[1] < 2:
                    self.logger.log_error(
                        f"Waypoint sequence has invalid shape: {self.vehicle_logic.waypoint_sequence.shape}"
                    )
                    return False
                
                self.logger.logger.info(
                    f"Generated path with {self.vehicle_logic.waypoint_sequence.shape[1]} waypoints"
                )
            
            return True
            
        except Exception as e:
            self.logger.log_error("Path planning initialization failed", e)
            return False
    
    def _initialize_network_2_GroundStation(self) -> bool:
        """Initialize network communication"""
        try:
            from ground_station_client import GroundStationClient
            
            self.logger.logger.info("Creating Ground Station client...")
            
            # Create Ground Station client
            self.vehicle_logic.client_Ground_Station = GroundStationClient(
                config=self.config,
                logger=self.vehicle_logic.logger,
                kill_event=self.vehicle_logic.kill_event
            )
            time.sleep(0.2)  # Allow client object to settle
            
            # Initialize network connection
            self.logger.logger.info("Initializing network connection...")
            if not self.vehicle_logic.client_Ground_Station.initialize_network():
                return False
            time.sleep(0.5)  # Allow network initialization to complete
            
            # Start network threads
            self.logger.logger.info("Starting network threads...")
            if not self.vehicle_logic.client_Ground_Station.start_threads():
                return False
            time.sleep(0.3)  # Allow threads to start properly
            
            self.logger.logger.info("Ground Station communication initialized")
            return True
            
        except Exception as e:
            self.logger.log_error("Ground Station initialization failed", e)
            return False
    
    def _initialize_qcar(self) -> bool:
        """Initialize QCar hardware"""
        try:
          
            from Controller.controllers import StateEstimator
            
            self.logger.logger.info("Initializing QCar hardware...")
            
            self.vehicle_logic.qcar = QCar(
                readMode=1,
                frequency=200
            )
            self.logger.logger.info(self.vehicle_logic.qcar)
            
            # time.sleep(0.5)  # Allow QCar hardware to initialize

            self.logger.logger.info("Initializing GPS...")
            self.vehicle_logic.gps = QCarGPS(
                initialPose=self.config.path.calibration_pose, 
                calibrate=self.config.path.calibrate
            )
            time.sleep(0.3)  # Allow GPS to initialize

            # Wait for initial GPS reading
            self.logger.logger.info("Waiting for initial GPS reading...")
            gps_received = False
            timeout = 10.0
            start = time.time()
            
            while not gps_received and (time.time() - start) < timeout:
                gps_received = self.vehicle_logic.gps.readGPS()
                time.sleep(0.1)  # Check every 100ms
            
            if not gps_received:
                self.logger.log_error("Failed to receive initial GPS reading")
                self.init_pose = None
                return False
            
            self.logger.logger.info("GPS reading received")
            
            # Get initial pose for EKF
            self.init_pose = np.array([
                self.vehicle_logic.gps.position[0],
                self.vehicle_logic.gps.position[1],
                self.vehicle_logic.gps.orientation[2]
            ])
            self.logger.logger.info(f" Initial pose: x={self.init_pose[0]:.2f}, y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f}")
            
            # Initialize state estimator through VehicleObserver
            self.logger.logger.info(" Initializing state estimator via VehicleObserver...")
            
            # Let VehicleObserver handle StateEstimator creation and management
            success = self.vehicle_logic.vehicle_observer.initialize_state_estimator(
                gps=self.vehicle_logic.gps,
                initial_pose=self.init_pose,
                logger=self.logger,
                use_ekf=self.config.steering.enable_steering_control
            )
            
            if not success:
                self.logger.log_error("Failed to initialize StateEstimator via VehicleObserver")
                return False
                
            time.sleep(0.3)  # Allow state estimator to initialize
            
            self.logger.logger.info(" QCar hardware initialized with EKF fusion via VehicleObserver")
            return True
            
        except Exception as e:
            self.logger.log_error("QCar initialization failed", e)
            return False
    
    def _initialize_controllers(self) -> bool:
        """Initialize control systems"""
        try:
            from Controller.controllers import SpeedController, SteeringController
            
            self.logger.logger.info(" Initializing controllers...")
            
            # Speed controller
            self.vehicle_logic.speed_controller = SpeedController(
                config=self.config,
                logger=self.vehicle_logic.vehicle_logger
            )
            time.sleep(0.1)  # Allow speed controller to initialize
            
            # Steering controller
            if self.config.steering.enable_steering_control:
                self.vehicle_logic.steering_controller = SteeringController(
                    waypoints=self.vehicle_logic.waypoint_sequence,
                    config=self.config,
                    logger=self.vehicle_logic.vehicle_logger
                )
                time.sleep(0.2)  # Allow steering controller to initialize
                self.check_initial_position()  # Ensure initial position is set in steering controller
            
            self.logger.logger.info("Controllers initialized")
            return True
            
        except Exception as e:
            self.logger.log_error("Controller initialization failed", e)
            return False
    
    def _initialize_perception(self) -> bool:
        """Initialize perception systems"""
        try:
            from Yolo.YoLo import YOLOReceiver, YOLODriveLogic
            
            # Create YOLO components
            yolo_receiver = YOLOReceiver()
            
            pulse_length = (
                self.config.timing.controller_update_rate *
                self.config.yolo.pulse_length_multiplier
            )
            
            yolo_drive = YOLODriveLogic(
                pulseLength=pulse_length,
            )
            
            # Initialize YOLOManager with components
            self.vehicle_logic.yolo_manager.initialize(yolo_receiver, yolo_drive)
            
            self.logger.logger.info("Perception systems initialized")
            return True
            
        except Exception as e:
            self.logger.log_error("Perception initialization failed", e)
            return False