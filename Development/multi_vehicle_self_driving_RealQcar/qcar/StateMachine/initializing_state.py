"""
Initializing State - Simplified Event-Driven Implementation

Handles system initialization and setup.
Transitions to WAITING_FOR_START when initialization is complete.
"""
import time
import sys
import os
import numpy as np
from typing import Dict, Any, Tuple, Optional

from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason
from Yolo.YoLo import YOLOReceiver, YOLODriveLogic
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from hal.products.mats import SDCSRoadMap
from ground_station_client import GroundStationClient



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
    
    # Timing constants
    INITIAL_DELAY = 1.0  # Wait before starting initialization
    STEP_DELAY = 0.5  # Delay between initialization steps
    TIMEOUT = 30.0  # Maximum initialization time
    
    def enter(self) -> bool:
        """Initialize system components"""
        super().enter()
        self.logger.logger.info("Entering INITIALIZING state")
        
        self.state_data = {
            'initialization_start': time.time(),
            'components_initialized': False,
            'ready_to_start': False,
            'last_step_time': time.time()
        }
        
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle initialization process"""
        throttle, steering = 0.0, 0.0
        current_time = time.time()
        elapsed_time = current_time - self.state_data['initialization_start']
        
        # Check for timeout
        if elapsed_time > self.TIMEOUT:
            self.logger.log_warning(f"[!] Initialization timeout ({self.TIMEOUT}s), proceeding anyway")
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        # Wait for initial system settle time
        if elapsed_time < self.INITIAL_DELAY:
            return throttle, steering, None
        
        # Initialize components
        if not self.state_data['components_initialized']:
            if self._initialize_all_components():
                self.state_data['components_initialized'] = True
                self.state_data['last_step_time'] = current_time
                self.logger.logger.info("[STEP 1/2] All components initialized")
                time.sleep(0.2)
            else:
                self._log_initialization_progress(elapsed_time)
                return throttle, steering, None
        
        # Finalize and transition
        if self.state_data['components_initialized'] and not self.state_data['ready_to_start']:
            if current_time - self.state_data['last_step_time'] < self.STEP_DELAY:
                return throttle, steering, None
            
            self.state_data['ready_to_start'] = True
            self.logger.logger.info("[STEP 2/2] System ready to start")
            time.sleep(0.3)
        
        # Transition to WAITING_FOR_START
        if self.state_data['ready_to_start']:
            self.logger.logger.info(f"[SUCCESS] Initialization complete in {elapsed_time:.1f}s")
            return throttle, steering, (VehicleState.WAITING_FOR_START, StateTransitionReason.INITIALIZATION_COMPLETE)
        
        return throttle, steering, None
    
    def _log_initialization_progress(self, elapsed_time: float):
        """Log initialization progress every 5 seconds"""
        if elapsed_time % 5.0 < 0.1:
            self.logger.logger.info(f"[INIT] Component initialization in progress... ({elapsed_time:.1f}s elapsed)")
    
    def handle_event(self, command_type, data: Dict[str, Any] = None) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """Handle commands during initialization - only emergency stop is accepted"""
        if not COMMAND_TYPE_AVAILABLE:
            return super().handle_event(command_type, data)
        
        if command_type == CommandType.EMERGENCY_STOP:
            self.logger.logger.warning("[!] Emergency stop during initialization")
            return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)
        
        self.logger.logger.info(f"Ignoring '{command_type}' command during initialization")
        return None
    
    def _initialize_all_components(self) -> bool:
        """Initialize all required components in sequence"""
        initialization_steps = [
            ("Path planning", self._initialize_path_planning, 0.2),
            ("QCar hardware", self._initialize_qcar, 0.2),
            # Perception is now activated via Ground Station command
            ("Telemetry logging", self._initialize_telemetry, 0.1)
        ]
        
        try:
            self.logger.logger.info("Starting component initialization...")
            
            for idx, (component_name, init_func, settle_time) in enumerate(initialization_steps, 1):
                if not init_func():
                    self.logger.log_error(f"{component_name} initialization failed")
                    return False
                
                self.logger.logger.info(f"  [{idx}/{len(initialization_steps)}] {component_name} ready")
                time.sleep(settle_time)
            
            self.logger.logger.info(f"All {len(initialization_steps)} components initialized successfully!")
            return True
            
        except Exception as e:
            self.logger.log_error("Component initialization failed", e)
            return False
    
    def _initialize_telemetry(self) -> bool:
        """Initialize telemetry logging if enabled"""
        try:
            if self.config.logging.enable_telemetry_logging:
                self.vehicle_logic.logger.setup_telemetry_logging(self.config.logging.data_log_dir)
            return True
        except Exception as e:
            self.logger.log_error("Telemetry initialization failed", e)
            return False
    
    def check_initial_position(self) -> bool:
        """Check if vehicle is at start position"""
        try:
            if not (hasattr(self, 'roadmap') and self.roadmap and hasattr(self, 'node_sequence')):
                return True
            
            start_node_reached, init_waypoint_seq = self.roadmap.initial_check(
                self.init_pose,
                self.node_sequence,
                self.waypoint_sequence
            )
            
            if not start_node_reached:
                self._log_position_mismatch(init_waypoint_seq)
                return False
            
            self.vehicle_logger.logger.info("Vehicle is at start position")
            return True
            
        except Exception as e:
            self.vehicle_logger.log_error("Initial position check failed", e)
            return False
    
    def _log_position_mismatch(self, init_waypoint_seq):
        """Log detailed information when vehicle is not at start position"""
        target_node = self.node_sequence[0]
        target_pose = self.roadmap.get_node_pose(target_node).squeeze()
        current_dist = np.linalg.norm(self.init_pose[:2] - target_pose[:2])
        
        self.vehicle_logger.log_warning("=" * 60)
        self.vehicle_logger.log_warning("NOT AT START POSITION")
        self.vehicle_logger.log_warning(f"  Current: ({self.init_pose[0]:.2f}, {self.init_pose[1]:.2f}, {self.init_pose[2]:.2f})")
        self.vehicle_logger.log_warning(f"  Target node: {target_node}")
        self.vehicle_logger.log_warning(f"  Target: ({target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f})")
        self.vehicle_logger.log_warning(f"  Distance: {current_dist:.2f}m")
        self.vehicle_logger.log_warning("=" * 60)
        
        # Update waypoint sequence to navigate to start
        self.waypoint_sequence = init_waypoint_seq
        if hasattr(self, 'steering_controller') and self.steering_controller:
            self.steering_controller.reset(self.waypoint_sequence)
    
    def exit(self):
        """Clean up initialization state"""
        init_time = self.get_time_in_state()
        self.logger.logger.info(f"[INIT] Exiting - completed in {init_time:.1f}s")
        super().exit()
    
    # ========== COMPONENT INITIALIZATION METHODS ==========
    
    def _initialize_path_planning(self) -> bool:
        """Initialize path planning system"""
        if not self.vehicle_logic.controller_manager.config.enable_steering_control:
            return True
        
        try:
            
            # Create roadmap
            self.vehicle_logic.roadmap = SDCSRoadMap(
                leftHandTraffic=self.config.path.left_hand_traffic,
                useSmallMap=True
            )
            
            # Set node sequence
            self.vehicle_logic.node_sequence = self.config.path.valid_nodes
            self.logger.logger.info(
                f"Node sequence ({len(self.vehicle_logic.node_sequence)} nodes): "
                f"{self.vehicle_logic.node_sequence}"
            )
            
            # Generate and validate waypoint sequence
            waypoints = self.vehicle_logic.roadmap.generate_path(self.vehicle_logic.node_sequence)
            if not self._validate_waypoint_sequence(waypoints):
                return False
            
            self.vehicle_logic.waypoint_sequence = waypoints
            self.logger.logger.info(f"Generated path with {waypoints.shape[1]} waypoints")
            return True
            
        except Exception as e:
            self.logger.log_error("Path planning initialization failed", e)
            return False
    
    def _validate_waypoint_sequence(self, waypoints) -> bool:
        """Validate generated waypoint sequence"""
        if waypoints is None:
            self.logger.log_error(
                f"Failed to generate path for nodes: {self.vehicle_logic.node_sequence}"
            )
            return False
        
        if not isinstance(waypoints, np.ndarray):
            self.logger.log_error(f"Invalid waypoint type: {type(waypoints)}")
            return False
        
        if waypoints.shape[0] < 2 or waypoints.shape[1] < 2:
            self.logger.log_error(f"Invalid waypoint shape: {waypoints.shape}")
            return False
        
        return True
    
    def _initialize_network_2_GroundStation(self) -> bool:
        """Initialize network communication with Ground Station"""
        try:
            
            # Create client
            self.vehicle_logic.client_Ground_Station = GroundStationClient(
                config=self.config,
                logger=self.vehicle_logic.logger,
                kill_event=self.vehicle_logic.kill_event
            )
            time.sleep(0.2)
            
            # Initialize network
            if not self.vehicle_logic.client_Ground_Station.initialize_network():
                return False
            time.sleep(0.5)
            
            # Start threads
            if not self.vehicle_logic.client_Ground_Station.start_threads():
                return False
            time.sleep(0.3)
            
            self.logger.logger.info("Ground Station communication initialized")
            return True
            
        except Exception as e:
            self.logger.log_error("Ground Station initialization failed", e)
            return False
    
    def _initialize_qcar(self) -> bool:
        """
        Initialize QCar hardware and GPS
        
        Initialization Flow:
        1. Create QCar and GPS instances (hardware/simulation)
        2. Wait for first GPS reading to get initial pose
        3. Initialize VehicleObserver's local estimator with GPS and initial pose
        
        Note: VehicleObserver already exists (created in vehicle_logic.__init__),
              but its local estimator needs GPS data to initialize properly.
        """
        try:
            
            # Initialize QCar and GPS based on physical/simulation mode
            if self.vehicle_logic.vehicle_type == "Limo":
                self.logger.logger.info("Limo Car detected - skipping QCar initialization")
                
            elif not IS_PHYSICAL_QCAR and self.vehicle_logic.vehicle_type == "Qcar":
                self.logger.logger.info("QCar Simulation mode detected")
                from qvl.multi_agent import readRobots
                self._initialize_simulated_qcar(readRobots)
            else:
                self._initialize_physical_qcar()
            
            # Wait for GPS and initialize state estimator
            if not self._wait_for_gps():
                return False
            
            if not self._initialize_state_estimator():
                return False
            
            self.logger.logger.info("QCar hardware initialized with state estimation")
            return True
            
        except Exception as e:
            self.logger.log_error("QCar initialization failed", e)
            return False
    
    def _initialize_simulated_qcar(self, readRobots):
        """Initialize simulated QCar"""
        
        robotsDir = readRobots()
        name = f"QC2_{self.vehicle_logic.vehicle_id}"
        car_config = robotsDir[name]
        
        self.vehicle_logic.qcar = QCar(
            readMode=1,
            frequency=100,
            hilPort=car_config["hilPort"]
        )
        
        # Check if calibration was requested
        calibrate_gps = getattr(self.vehicle_logic, 'calibration_requested', False)
        if calibrate_gps:
            self.logger.logger.info("📍 GPS calibration requested - calibrating GPS")
            self.vehicle_logic.calibration_requested = False  # Reset flag
        
        self.vehicle_logic.gps = QCarGPS(
            initialPose=self.config.path.calibration_pose,
            calibrate=calibrate_gps,
            gpsPort=car_config["gpsPort"],
            lidarIdealPort=car_config["lidarIdealPort"]
        )
    
    def _initialize_physical_qcar(self):
        """Initialize physical QCar"""
        
        self.vehicle_logic.qcar = QCar(readMode=1, frequency=self.config.timing.controller_update_rate)
        time.sleep(0.3)
        
        # Check if calibration was requested, otherwise use config setting
        calibrate_gps = getattr(self.vehicle_logic, 'calibration_requested', self.config.path.calibrate)
        if getattr(self.vehicle_logic, 'calibration_requested', False):
            self.logger.logger.info("GPS calibration requested - calibrating GPS")
            self.vehicle_logic.calibration_requested = False  # Reset flag
        self.logger.logger.info(f"Calibration pose: {self.config.path.calibration_pose}")
        
        self.vehicle_logic.gps = QCarGPS(
            initialPose=self.config.path.calibration_pose,
            calibrate=calibrate_gps
        )
        time.sleep(0.3)
    
    def _wait_for_gps(self, timeout: float = 10.0) -> bool:
        """Wait for initial GPS reading"""
        self.logger.logger.info("Waiting for initial GPS reading...")
        start = time.time()
        
        while (time.time() - start) < timeout:
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
            time.sleep(0.1)
        
        self.logger.log_error("GPS timeout - no reading received")
        return False
    
    def _initialize_state_estimator(self) -> bool:
        """Initialize local state estimator through VehicleObserver"""
        try:
            # Use refactored VehicleObserver API
            # Estimator type is already set in VehicleObserver constructor
            # We just need to initialize it with GPS and initial pose
            success = self.vehicle_logic.vehicle_observer.initialize_local_estimator(
                gps=self.vehicle_logic.gps,
                initial_pose=self.init_pose,
                estimator_params={'use_qcar_ekf': self.vehicle_logic.controller_manager.config.enable_steering_control}
            )
            
            if success:
                self.logger.logger.info(
                    f"Local estimator initialized at pose: "
                    f"x={self.init_pose[0]:.2f}, y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f}"
                )
                time.sleep(0.1)
            else:
                self.logger.log_error("Local estimator initialization failed")
            
            return success
            
        except Exception as e:
            self.logger.log_error("State estimator initialization failed", e)
            return False
    
    def _initialize_perception(self) -> bool:
        """Initialize perception systems (YOLO) - launch as subprocess"""
        try:
            
            if not IS_PHYSICAL_QCAR:

                import subprocess
                import os
                
                # Calculate YOLO port for this car
                yolo_port = f'1866{self.vehicle_logic.vehicle_id}'
                
                # Construct path to yolo_server_virtual.py
                yolo_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                        '..', 'Yolo', 'yolo_server_virtual.py')
                
                self.logger.logger.info(f"Launching YOLO server on port {yolo_port}...")
                
                # Launch YOLO server as subprocess in new console window for debugging
                yolo_process = subprocess.Popen(
                    [
                        'python', yolo_script,
                        '-idx', str(self.vehicle_logic.vehicle_id),
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    creationflags=subprocess.CREATE_NEW_CONSOLE if os.name == 'nt' else 0
                )
                
                # Store process handle for cleanup
                self.vehicle_logic.yolo_process = yolo_process
                
                # Wait for YOLO server to start
                self.logger.logger.info("Waiting for YOLO server to initialize...")
                time.sleep(2.5)
                
                # Check if process is still running
                if yolo_process.poll() is not None:
                    self.logger.log_error("YOLO server failed to start")
                    return False
                
                # Create YOLOReceiver to connect to the server
                self.logger.logger.info("Connecting to YOLO server...")
                yolo_receiver = YOLOReceiver(
                    ip='localhost',
                    nonBlocking=True,
                    port=yolo_port
                )

                # yolo_receiver = None  # YOLO not used on physical QCar

            else:
                yolo_receiver = YOLOReceiver(
                    ip='localhost',
                    nonBlocking=True,
                )
                # yolo_receiver = None  # YOLO not used on physical QCar
            # Create YOLO drive logic with proper pulse length
            pulse_length = (
                self.config.timing.controller_update_rate *
                self.config.yolo.pulse_length_multiplier
            )
            yolo_drive_logic = YOLODriveLogic(
                stopSignThreshold=0.6,
                trafficThreshold=1.7,
                carThreshold=0.3,
                yieldThreshold=0.9,
                personThreshold=0.6,
                pulseLength=pulse_length
            )
            
            # Initialize YOLOManager
            self.vehicle_logic.yolo_manager.initialize(yolo_receiver, yolo_drive_logic)
            
            self.logger.logger.info("YOLO system enabled successfully")
            return True
            
        except Exception as e:
            self.logger.log_error("Perception initialization failed", e)
            # Clean up process if it was started
            if hasattr(self.vehicle_logic, 'yolo_process'):
                try:
                    self.vehicle_logic.yolo_process.terminate()
                except:
                    pass
            # YOLO is optional - continue without it
            self.logger.logger.warning("Vehicle will operate without YOLO perception")
            return True