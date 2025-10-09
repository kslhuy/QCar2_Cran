"""
Main Vehicle Controller - Integrates all components
"""
import numpy as np
import time
import random
from typing import Optional
from threading import Event

from pal.products.qcar import QCar, QCarGPS
from hal.products.mats import SDCSRoadMap

from config import VehicleControlConfig
from logging_utils import VehicleLogger, PerformanceMonitor
from state_machine import VehicleState, VehicleStateMachine
from network_client import NetworkClient
from controllers import SpeedController, SteeringController, StateEstimator
from safety import ControlValidator, SensorHealthMonitor, CollisionAvoidance, WatchdogTimer
from utils import YOLOReceiver, YOLODriveLogic
from platoon_controller import PlatoonController, PlatoonConfig


class VehicleController:
    """Main vehicle controller class"""
    
    def __init__(self, config: VehicleControlConfig, kill_event: Event):
        self.config = config
        self.kill_event = kill_event

        # calibrationPose = [0,2,-np.pi/2]
        
        # Setup logging
        self.logger = VehicleLogger(
            car_id=config.network.car_id,
            log_dir=config.logging.log_dir,
            log_level=config.logging.log_level
        )
        
        self.logger.logger.info("="*60)
        self.logger.logger.info(f"Vehicle Controller Initialized - Car ID: {config.network.car_id}")
        self.logger.logger.info("="*60)
        
        # Performance monitoring
        self.perf_monitor = PerformanceMonitor(self.logger)
        
        # State machine
        self.state_machine = VehicleStateMachine(self.logger)
        
        # Platoon controller
        platoon_config = PlatoonConfig()
        self.platoon_controller = PlatoonController(platoon_config, self.logger)
        
        # Safety systems
        self.validator = ControlValidator(config, self.logger)
        self.sensor_health = SensorHealthMonitor(config, self.logger)
        self.collision_avoidance = CollisionAvoidance(config, self.logger)
        self.watchdog = WatchdogTimer(config.safety.watchdog_timeout, self.logger)
        
        # Components (initialized later)
        self.network = None
        self.qcar = None
        self.gps = None
        self.yolo = None
        self.yolo_drive = None
        self.state_estimator = None
        self.speed_controller = None
        self.steering_controller = None
        
        # Path planning
        self.roadmap = None
        self.waypoint_sequence = None
        self.node_sequence = None
        
        # Control state
        self.v_ref = config.speed.v_ref
        self.yolo_gain = 1.0
        
        # Timing
        self.start_time = time.time()
        self.loop_counter = 0
        self.telemetry_counter = 0
        
    def elapsed_time(self) -> float:
        """Get elapsed time since start"""
        return time.time() - self.start_time
    
    def initialize(self) -> bool:
        """Initialize all systems"""
        self.logger.logger.info("Initializing vehicle systems...")
        
        try:
            # Initialize path planning
            if not self._initialize_path_planning():
                return False
            
            # Initialize network (if enabled)
            if not self._initialize_network():
                return False
            
            # Initialize QCar hardware
            if not self._initialize_qcar():
                return False
            
            # Initialize controllers
            if not self._initialize_controllers():
                return False
            
            # Initialize perception
            if not self._initialize_perception():
                return False
            
            # Setup telemetry logging
            if self.config.logging.enable_telemetry_logging:
                self.logger.setup_telemetry_logging(self.config.logging.data_log_dir)
            
            self.logger.logger.info("All systems initialized successfully")
            return True
            
        except Exception as e:
            self.logger.log_error("Initialization failed", e)
            return False
    
    def _initialize_path_planning(self) -> bool:
        """Initialize path planning system"""
        try:
            if self.config.steering.enable_steering_control:
                self.roadmap = SDCSRoadMap(
                    leftHandTraffic=self.config.path.left_hand_traffic,
                    useSmallMap=True
                )
                
                # Get valid nodes for current configuration
                valid_nodes = self.config.path.valid_nodes
                # self.logger.logger.info(f"Valid nodes from config: {valid_nodes}")
                
                # # Generate node sequence
                # random.shuffle(valid_nodes)
                # num_nodes = random.randint(6, 10)  # Match original: 6 to 10 nodes
                
                # # Ensure we don't exceed available nodes
                # num_nodes = min(num_nodes, len(valid_nodes))
                
                # self.node_sequence = valid_nodes[:num_nodes] + [valid_nodes[0]]
                self.node_sequence = valid_nodes
                self.logger.logger.info(f"Selected node sequence ({len(self.node_sequence)} nodes): {self.node_sequence}")
                
                # Generate waypoint sequence
                try:
                    self.waypoint_sequence = self.roadmap.generate_path(self.node_sequence)
                except Exception as path_error:
                    self.logger.log_error(
                        f"Roadmap.generate_path() failed for nodes {self.node_sequence}",
                        path_error
                    )
                    return False
                
                # Validate waypoint sequence
                if self.waypoint_sequence is None:
                    self.logger.log_error(
                        f"Failed to generate path for node sequence: {self.node_sequence}. "
                        f"Check if these nodes exist in your roadmap and can be connected."
                    )
                    return False
                
                if not isinstance(self.waypoint_sequence, np.ndarray):
                    self.logger.log_error(
                        f"Waypoint sequence has wrong type: {type(self.waypoint_sequence)}"
                    )
                    return False
                
                if self.waypoint_sequence.shape[0] < 2 or self.waypoint_sequence.shape[1] < 2:
                    self.logger.log_error(
                        f"Waypoint sequence has invalid shape: {self.waypoint_sequence.shape}"
                    )
                    return False
                
                self.logger.logger.info(
                    f"Generated path with {self.waypoint_sequence.shape[1]} waypoints"
                )
            
            return True
            
        except Exception as e:
            self.logger.log_error("Path planning initialization failed", e)
            return False
    
    def _initialize_network(self) -> bool:
        """Initialize network communication"""
        if not self.config.network.is_remote_enabled:
            self.logger.logger.info("Remote control disabled")
            return True
        
        try:
            self.state_machine.transition_to(VehicleState.WAITING_FOR_CONNECTION)
            
            self.network = NetworkClient(
                host_ip=self.config.network.host_ip,
                port=self.config.network.port,
                car_id=self.config.network.car_id,
                logger=self.logger,
                config=self.config
            )
            
            # Try to connect
            if not self.network.connect():
                self.logger.log_warning("Failed to connect to host PC")
                return False
            
            self.logger.logger.info("Network connection established")
            return True
            
        except Exception as e:
            self.logger.log_error("Network initialization failed", e)
            return False
    
    def _initialize_qcar(self) -> bool:
        """Initialize QCar hardware"""
        try:
            self.qcar = QCar(
                readMode=1,
                frequency=self.config.timing.controller_update_rate
            )

            self.gps = QCarGPS(initialPose=self.config.path.calibration_pose, calibrate=self.config.path.calibrate)

            # Wait for initial GPS reading
            self.logger.logger.info("Waiting for initial GPS reading...")
            gps_received = False
            timeout = 10.0
            start = time.time()
            
            while not gps_received and (time.time() - start) < timeout:
                gps_received = self.gps.readGPS()
                time.sleep(0.1)
            
            if not gps_received:
                self.logger.log_error("Failed to receive initial GPS reading")
                return False
            
            # Get initial pose for EKF
            initial_pose = None
            if self.config.steering.enable_steering_control:
                initial_pose = np.array([
                    self.gps.position[0],
                    self.gps.position[1],
                    self.gps.orientation[2]
                ])
                self.logger.logger.info(f"Initial pose: x={initial_pose[0]:.2f}, y={initial_pose[1]:.2f}, theta={initial_pose[2]:.2f}")
            
            # Initialize state estimator with EKF
            self.state_estimator = StateEstimator(
                gps=self.gps,
                initial_pose=initial_pose,
                logger=self.logger,
                use_ekf=self.config.steering.enable_steering_control
            )
            
            self.logger.logger.info("QCar hardware initialized with EKF fusion")
            return True
            
        except Exception as e:
            self.logger.log_error("QCar initialization failed", e)
            return False
    
    def _initialize_controllers(self) -> bool:
        """Initialize control systems"""
        try:
            # Speed controller
            self.speed_controller = SpeedController(
                config=self.config,
                logger=self.logger
            )
            
            # Steering controller
            if self.config.steering.enable_steering_control:
                self.steering_controller = SteeringController(
                    waypoints=self.waypoint_sequence,
                    config=self.config,
                    logger=self.logger,
                    cyclic=True
                )
            
            self.logger.logger.info("Controllers initialized")
            return True
            
        except Exception as e:
            self.logger.log_error("Controller initialization failed", e)
            return False
    
    def _initialize_perception(self) -> bool:
        """Initialize perception systems"""
        try:
            self.yolo = YOLOReceiver()
            
            pulse_length = (
                self.config.timing.controller_update_rate *
                self.config.yolo.pulse_length_multiplier
            )
            
            self.yolo_drive = YOLODriveLogic(
                stopSignThreshold=self.config.yolo.stop_sign_threshold,
                trafficThreshold=self.config.yolo.traffic_threshold,
                carThreshold=self.config.yolo.car_threshold,
                yieldThreshold=self.config.yolo.yield_threshold,
                personThreshold=self.config.yolo.person_threshold,
                pulseLength=pulse_length
            )
            
            self.logger.logger.info("Perception systems initialized")
            return True
            
        except Exception as e:
            self.logger.log_error("Perception initialization failed", e)
            return False
    
    def check_initial_position(self) -> bool:
        """Check if vehicle is at start position"""
        try:
            # Read QCar sensors first
            self.qcar.read()
            
            # Update state with current sensor readings
            motor_tach = self.qcar.motorTach
            gyro_z = self.qcar.gyroscope[2] if hasattr(self.qcar, 'gyroscope') else 0.0
            
            self.state_estimator.update(
                motor_tach=motor_tach,
                steering=0.0,
                dt=0.005,
                gyro_z=gyro_z
            )
            
            x, y, theta, _, _ = self.state_estimator.get_state()
            init_pose = np.array([x, y, theta])
            
            self.logger.logger.info(f"Initial pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
            
            # Transition to WAITING_FOR_START first (required by state machine)
            if self.state_machine.state == VehicleState.WAITING_FOR_CONNECTION:
                self.state_machine.transition_to(VehicleState.WAITING_FOR_START)
            
            # Check if at start node
            start_node_reached, init_waypoint_seq = self.roadmap.initial_check(
                init_pose,
                self.node_sequence,
                self.waypoint_sequence
            )
            
            if not start_node_reached:
                # Log detailed information about position mismatch
                target_node = self.node_sequence[0]
                target_pose = self.roadmap.get_node_pose(target_node).squeeze()
                current_dist = np.linalg.norm(init_pose[:2] - target_pose[:2])
                
                self.logger.log_warning("="*60)
                self.logger.log_warning("NOT AT START POSITION - Navigating to start")
                self.logger.log_warning(f"  Current position: ({init_pose[0]:.2f}, {init_pose[1]:.2f}, {init_pose[2]:.2f})")
                self.logger.log_warning(f"  Target node: {target_node}")
                self.logger.log_warning(f"  Target position: ({target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f})")
                self.logger.log_warning(f"  Distance to start: {current_dist:.2f}m")
                self.logger.log_warning(f"  Generated {init_waypoint_seq.shape[1]} waypoints to reach start")
                self.logger.log_warning("="*60)
                
                self.waypoint_sequence = init_waypoint_seq
                self.steering_controller.reset(self.waypoint_sequence)
                # Now we can transition from WAITING_FOR_START to NAVIGATING_TO_START
                self.state_machine.transition_to(VehicleState.NAVIGATING_TO_START)
                # Track this so we know to transition to FOLLOWING_PATH later
                self.start_node_reached = False
            else:
                target_node = self.node_sequence[0]
                self.logger.logger.info("="*60)
                self.logger.logger.info("AT START POSITION")
                self.logger.logger.info(f"  Current position: ({init_pose[0]:.2f}, {init_pose[1]:.2f}, {init_pose[2]:.2f})")
                self.logger.logger.info(f"  Start node: {target_node}")
                self.logger.logger.info("  Ready to follow planned path")
                self.logger.logger.info("="*60)
                self.start_node_reached = True
            
            return True
            
        except Exception as e:
            self.logger.log_error("Initial position check failed", e)
            return False
    
    def run(self):
        """Main control loop"""
        self.logger.logger.info("Starting control loop...")
        
        # Context managers for hardware
        with self.qcar, self.gps, self.yolo:
            try:
                # Check initial position
                if not self.check_initial_position():
                    self.state_machine.transition_to(VehicleState.ERROR)
                    return
                
                # Start in appropriate mode based on initial position check
                if self.start_node_reached:
                    # Already at start position, go directly to following path
                    if self.state_machine.state == VehicleState.WAITING_FOR_START:
                        # Reset speed controller integral to prevent windup
                        self.speed_controller.ei = 0
                        self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
                        self.logger.logger.info("Starting path following")
                else:
                    # Need to navigate to start first
                    # Reset speed controller integral to prevent windup
                    self.speed_controller.ei = 0
                    self.logger.logger.info("Navigating to start position...")
                
                # CRITICAL: Reset start_time NOW (like original: t0 = time.time(), t = 0)
                # This ensures elapsed_time() starts from 0 when control loop begins
                self.start_time = time.time()
                self.loop_counter = 0
                self.telemetry_counter = 0
                
                # Main control loop
                target_dt = 1.0 / self.config.timing.controller_update_rate
                last_loop_time = time.time()
                
                while not self.kill_event.is_set():
                    loop_start = time.time()
                    
                    # Calculate actual dt from last iteration (like original vehicle_control.py)
                    actual_dt = loop_start - last_loop_time
                    last_loop_time = loop_start
                    
                    # Reset watchdog
                    self.watchdog.reset()
                    
                    # Run one control iteration with ACTUAL dt, not fixed dt
                    if not self._control_iteration(actual_dt):
                        self.logger.log_error("Control iteration failed")
                        break
                    
                    # Performance monitoring
                    loop_time = time.time() - loop_start
                    self.perf_monitor.log_loop_time(loop_time)
                    
                    # Check for excessive loop time
                    if loop_time > self.config.safety.max_loop_time_warning:
                        self.logger.log_warning(
                            f"Loop time exceeded threshold: {loop_time*1000:.2f}ms"
                        )
                    
                    # Sleep to maintain loop rate (use target_dt)
                    sleep_time = target_dt - loop_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    
                    self.loop_counter += 1
                    
                    # Check if experiment time exceeded
                    if self.elapsed_time() > self.config.timing.tf:
                        self.logger.logger.info("Experiment time limit reached")
                        break
                
            except KeyboardInterrupt:
                self.logger.logger.info("Control loop interrupted by user")
            except Exception as e:
                self.logger.log_error("Control loop error", e)
            finally:
                self._shutdown()
    
    def _control_iteration(self, dt: float) -> bool:
        """Single control iteration"""
        try:
            # Read QCar sensors
            self.qcar.read()
            
            # Get current steering (for EKF prediction)
            # Note: delta from previous iteration is used for EKF update
            motor_tach = self.qcar.motorTach
            gyro_z = self.qcar.gyroscope[2] if hasattr(self.qcar, 'gyroscope') else 0.0
            
            # Get last steering command for EKF
            last_steering = getattr(self, '_last_steering', 0.0)
            
            # Update state estimate with EKF fusion (GPS + IMU + odometry)
            self.state_estimator.update(
                motor_tach=motor_tach,
                steering=last_steering,
                dt=dt,
                gyro_z=gyro_z
            )
            
            x, y, theta, velocity, state_valid = self.state_estimator.get_state()
            
            # # Validate state
            # if not self.validator.validate_state(x, y, theta):
            #     return False
            
            # # Check sensor health
            # gps_healthy = self.sensor_health.check_gps_health(
            #     self.state_estimator.was_updated()
            # )
            
            # Read YOLO detections
            self.yolo.read()
            
            # Process YOLO and get velocity gain
            self.yolo_gain = self.yolo_drive.check_yolo(
                self.yolo.stopSign,
                self.yolo.trafficlight,
                self.yolo.cars,
                self.yolo.yieldSign,
                self.yolo.person
            )
            
            # ============ PLATOON MODE LOGIC ============
            if self.state_machine.is_in_platoon():
                self._handle_platoon_states(x, y, velocity)
            # ============================================
            
            # Check if we're navigating to start and have reached it
            if self.state_machine.state == VehicleState.NAVIGATING_TO_START:
                # Check distance to first waypoint of the actual path
                if self.steering_controller:
                    # Get the target waypoint (first waypoint in sequence)
                    target_wp = self.steering_controller.wp[:, 0]
                    current_pos = np.array([x, y])
                    dist_to_target = np.linalg.norm(target_wp - current_pos)
                    
                    if dist_to_target < 0.3:  # Within 30cm of start
                        self.logger.logger.info("Reached start position, beginning path following")
                        self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
                        self.start_node_reached = True
            
            # Check collision risk (but not in platoon mode - that's handled by spacing controller)
            if not self.state_machine.is_in_platoon():
                emergency_stop, reason = self.collision_avoidance.check_collision_risk(
                    car_distance=self.yolo_drive.carDist,
                    person_distance=self.yolo_drive.personDist,
                    current_velocity=velocity
                )
                
                # Handle emergency stop state transitions
                if emergency_stop and self.state_machine.state == VehicleState.FOLLOWING_PATH:
                    self.state_machine.transition_to(VehicleState.EMERGENCY_STOP)
                elif not emergency_stop and self.state_machine.state == VehicleState.EMERGENCY_STOP:
                    # Recover from emergency stop when danger clears
                    # Reset speed controller integral to prevent windup during stop
                    self.speed_controller.ei = 0
                    self.logger.logger.info("Danger cleared, resuming path following")
                    self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
            else:
                emergency_stop = False  # Platoon spacing controller handles safety
            
            # EMERGENCY_STOP or STOPPED: Send zero commands immediately
            if self.state_machine.state in [VehicleState.EMERGENCY_STOP, VehicleState.STOPPED]:
                self.qcar.write(throttle=0, steering=0)
                u = 0
                delta = 0
                self._last_steering = 0
            # Normal operation: compute and send control commands
            elif self.state_machine.should_control():
                # Compute control commands
                u, delta = self._compute_control(x, y, theta, velocity, dt)
                
                # Store steering for next EKF update
                self._last_steering = delta
                
                # # Validate and send commands
                # _, u = self.validator.validate_throttle(u)
                # _, delta = self.validator.validate_steering(delta)
                self.qcar.write(throttle=u, steering=delta)
            else:
                # Any other state: stop
                self.qcar.write(throttle=0, steering=0)
                u = 0
                delta = 0
                self._last_steering = 0
            
            # Send telemetry (only every N iterations to reduce overhead)
            if self.loop_counter % 20 == 0:  # Send every 20 iterations = 10Hz at 200Hz loop
                self._send_telemetry(x, y, theta, velocity, u, delta)
            
            # Receive commands (only occasionally to reduce overhead)
            if self.loop_counter % 10 == 0:  # Check every 10 iterations = 20Hz
                self._receive_commands()
            
            return True
            
        except Exception as e:
            self.logger.log_error("Control iteration error", e)
            return False
    
    def _handle_platoon_states(self, x: float, y: float, velocity: float):
        """Handle platoon state transitions and logic"""
        current_state = self.state_machine.state
        
        # Update platoon controller with current sensor data
        if self.platoon_controller.enabled:
            # Update leader detection from YOLO
            self.platoon_controller.update_leader_info(
                detected=self.yolo.cars,
                distance=self.yolo_drive.carDist if self.yolo.cars else None,
                velocity=self._get_leader_velocity_from_network()
            )
        
        # === LEADER STATE MACHINE ===
        if current_state == VehicleState.PLATOON_LEADER_FORMING:
            # Check if all followers are ready
            if self.platoon_controller.are_all_followers_ready():
                self.logger.logger.info("🚗🚗🚗 All followers ready! Activating platoon")
                self.state_machine.transition_to(VehicleState.PLATOON_ACTIVE)
                self.platoon_controller.reset()
        
        # === FOLLOWER STATE MACHINE ===
        elif current_state == VehicleState.PLATOON_FOLLOWER_SEARCHING:
            # Looking for leader via YOLO
            if self.yolo.cars and self.yolo_drive.carDist is not None:
                if self.yolo_drive.carDist < 3.0:  # Detected car within 3 meters
                    self.logger.logger.info(f"🔍 Leader detected at {self.yolo_drive.carDist:.2f}m, adjusting spacing")
                    self.state_machine.transition_to(VehicleState.PLATOON_FOLLOWER_FORMING)
                    self.platoon_controller.formation_start_time = time.time()
            
            # Check timeout
            if self.platoon_controller.has_formation_timeout():
                self.logger.log_warning("⏰ Formation timeout, returning to normal mode")
                self.platoon_controller.disable_platoon_mode()
                self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
        
        elif current_state == VehicleState.PLATOON_FOLLOWER_FORMING:
            # Adjusting spacing to leader
            if self.platoon_controller.has_lost_leader():
                self.logger.log_warning("❌ Lost leader during formation")
                self.state_machine.transition_to(VehicleState.PLATOON_LOST)
            elif self.platoon_controller.is_spacing_stable():
                self.logger.logger.info("✅ Spacing stable, ready for platoon")
                # Broadcast readiness via network (will be handled in telemetry)
                self.state_machine.transition_to(VehicleState.PLATOON_ACTIVE)
            elif self.platoon_controller.has_formation_timeout():
                self.logger.log_warning("⏰ Formation timeout, returning to normal mode")
                self.platoon_controller.disable_platoon_mode()
                self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
        
        elif current_state == VehicleState.PLATOON_ACTIVE:
            # Active platoon - check for lost leader (followers only)
            if not self.platoon_controller.is_leader:
                if self.platoon_controller.has_lost_leader():
                    self.logger.log_warning("❌ Lost leader in active platoon")
                    self.state_machine.transition_to(VehicleState.PLATOON_LOST)
        
        elif current_state == VehicleState.PLATOON_LOST:
            # Try to re-find leader
            if self.yolo.cars and self.yolo_drive.carDist is not None:
                if self.yolo_drive.carDist < 3.0:
                    self.logger.logger.info("🔍 Leader re-detected, resuming formation")
                    self.state_machine.transition_to(VehicleState.PLATOON_FOLLOWER_FORMING)
                    self.platoon_controller.formation_start_time = time.time()
            
            # Give up after timeout
            if self.platoon_controller.has_formation_timeout():
                self.logger.log_warning("⏰ Giving up on platoon, returning to normal mode")
                self.platoon_controller.disable_platoon_mode()
                self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
    
    def _get_leader_velocity_from_network(self) -> Optional[float]:
        """Get leader velocity from network telemetry"""
        if not self.network or not self.network.connected:
            return None
        
        # This will be populated from received telemetry
        # For now, return None (will be implemented in network update)
        return getattr(self, '_leader_velocity_from_network', None)
    
    def _compute_platoon_velocity(self, current_velocity: float) -> float:
        """Compute target velocity based on platoon state"""
        state = self.state_machine.state
        
        if state == VehicleState.PLATOON_LEADER_FORMING:
            # Leader: move slowly during formation
            return self.platoon_controller.get_leader_velocity()
        
        elif state == VehicleState.PLATOON_FOLLOWER_SEARCHING:
            # Follower searching: move slowly to avoid catching up too fast
            return self.platoon_controller.get_leader_velocity()
        
        elif state == VehicleState.PLATOON_FOLLOWER_FORMING:
            # Follower forming: use spacing controller
            base_v = self.platoon_controller.get_leader_velocity()
            return self.platoon_controller.compute_follower_velocity(
                current_velocity=current_velocity,
                base_velocity=base_v
            )
        
        elif state == VehicleState.PLATOON_ACTIVE:
            # Active platoon
            if self.platoon_controller.is_leader:
                # Leader: use active speed
                return self.platoon_controller.get_active_velocity()
            else:
                # Follower: maintain spacing at active speed
                base_v = self.platoon_controller.get_active_velocity()
                return self.platoon_controller.compute_follower_velocity(
                    current_velocity=current_velocity,
                    base_velocity=base_v
                )
        
        elif state == VehicleState.PLATOON_LOST:
            # Lost platoon: slow down
            return self.platoon_controller.get_leader_velocity() * 0.5
        
        else:
            # Fallback: normal velocity
            return self.v_ref
    
    def _compute_control(self, x: float, y: float, theta: float, velocity: float, dt: float):
        """Compute control commands"""
        # Speed control with startup delay to prevent integral windup
        # During the first second (startDelay), keep throttle at 0 to let sensors settle
        # This matches the original: if t < startDelay: u = 0, delta = 0
        if self.elapsed_time() < self.config.timing.start_delay:
            # Startup delay - no control
            return 0.0, 0.0
        
        # === PLATOON VELOCITY CONTROL ===
        if self.state_machine.is_in_platoon():
            v_ref_adjusted = self._compute_platoon_velocity(velocity)
        else:
            # Normal mode: use YOLO-adjusted velocity
            v_ref_adjusted = self.v_ref * self.yolo_gain
        
        u = self.speed_controller.update(velocity, v_ref_adjusted, dt)
        
        # Steering control
        delta = 0.0
        if self.config.steering.enable_steering_control and self.steering_controller:
            # Use look-ahead point (0.2m forward from vehicle center)
            # This matches the original vehicle_control.py implementation
            p = np.array([x, y]) + np.array([np.cos(theta), np.sin(theta)]) * 0.2
            delta = self.steering_controller.update(p, theta, max(velocity, 0.1))
        
        return u, delta
    
    def _send_telemetry(self, x: float, y: float, theta: float, velocity: float, u: float, delta: float):
        """Send telemetry data"""
        # Prepare telemetry data
        telemetry = {
            'timestamp': time.time(),
            'time': self.elapsed_time(),
            'x': float(x),
            'y': float(y),
            'th': float(theta),           # Changed from 'theta' to 'th' for GUI compatibility
            'v': float(velocity),         # Changed from 'velocity' to 'v' for GUI compatibility
            'u': float(u),                # Changed from 'throttle' to 'u' for GUI compatibility
            'delta': float(delta),        # Changed from 'steering' to 'delta' for compatibility
            'v_ref': float(self.v_ref * self.yolo_gain),
            'yolo_gain': float(self.yolo_gain),
            'waypoint_index': self.steering_controller.get_waypoint_index() if self.steering_controller else 0,
            'cross_track_error': float(self.steering_controller.get_errors()[0]) if self.steering_controller else 0.0,
            'heading_error': float(self.steering_controller.get_errors()[1]) if self.steering_controller else 0.0,
            'state': self.state_machine.state.name,
            'gps_valid': self.state_estimator.state_valid,
            # Platoon telemetry
            # **self.platoon_controller.get_telemetry()
        }
        
        # Log to file
        if self.config.logging.enable_telemetry_logging:
            self.logger.log_telemetry(telemetry)
        
        # Send over network
        if self.network and self.network.connected:
            send_rate = self.config.timing.telemetry_send_rate
            if self.telemetry_counter % (self.config.timing.controller_update_rate // send_rate) == 0:
                self.network.send_telemetry(telemetry)
        
        self.telemetry_counter += 1
    
    def _receive_commands(self):
        """Receive commands from network"""
        if self.network and self.network.connected:
            commands = self.network.receive_commands()
            if commands:
                self._process_commands(commands)
    
    def _process_commands(self, commands: dict):
        """Process received commands"""
        try:
            # Check command type
            cmd_type = commands.get('type', '')
            
            # Handle different command types
            if cmd_type == 'stop':
                self.logger.logger.info("🛑 STOP command received from GUI")
                self.state_machine.transition_to(VehicleState.STOPPED)
                
            elif cmd_type == 'start':
                self.logger.logger.info("▶️  START command received from GUI")
                # Transition from STOPPED back to FOLLOWING_PATH
                if self.state_machine.state == VehicleState.STOPPED:
                    # Reset speed controller integral to prevent windup during stop
                    self.speed_controller.ei = 0
                    self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
                else:
                    self.logger.log_warning(f"Cannot start from state {self.state_machine.state.name}")
                    
            elif cmd_type == 'set_params':
                # Handle parameter updates
                if 'v_ref' in commands:
                    new_v_ref = float(commands['v_ref'])
                    if 0 <= new_v_ref <= 2.0:
                        self.v_ref = new_v_ref
                        self.logger.logger.info(f"Updated v_ref to {new_v_ref} m/s")
                    else:
                        self.logger.log_warning(f"Invalid v_ref value: {new_v_ref}")
                
                if 'node_sequence' in commands:
                    self.logger.logger.info(f"Received new node sequence: {commands['node_sequence']}")
                    # TODO: Implement path update if needed
            
            elif cmd_type == 'enable_platoon':
                # Enable platoon mode
                role = commands.get('role', 'follower')  # 'leader' or 'follower'
                leader_id = commands.get('leader_id', None)
                
                if role == 'leader':
                    self.logger.logger.info("🚗 Enabling platoon mode as LEADER")
                    self.platoon_controller.enable_as_leader()
                    self.state_machine.transition_to(VehicleState.PLATOON_LEADER_FORMING)
                elif role == 'follower':
                    self.logger.logger.info(f"🚗 Enabling platoon mode as FOLLOWER (leader: {leader_id})")
                    self.platoon_controller.enable_as_follower()
                    self.state_machine.transition_to(VehicleState.PLATOON_FOLLOWER_SEARCHING)
                else:
                    self.logger.log_warning(f"Invalid platoon role: {role}")
                
                # Synchronize path if node_sequence provided
                if 'node_sequence' in commands:
                    self.logger.logger.info(f"Synchronizing platoon path: {commands['node_sequence']}")
                    # TODO: Update node_sequence to match platoon
            
            elif cmd_type == 'disable_platoon':
                # Disable platoon mode
                self.logger.logger.info("🚗 Disabling platoon mode")
                self.platoon_controller.disable()
                
                # Transition back to normal navigation
                if self.state_machine.state == VehicleState.STOPPED:
                    self.state_machine.transition_to(VehicleState.STOPPED)
                else:
                    self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
                    
            elif cmd_type == 'shutdown':
                self.logger.logger.info("Shutdown command received")
                self.kill_event.set()
            
            else:
                # Legacy command format support
                if 'command' in commands:
                    cmd = commands['command']
                    if cmd == 'stop':
                        self.logger.logger.info("🛑 STOP command received (legacy format)")
                        self.state_machine.transition_to(VehicleState.STOPPED)
                    elif cmd == 'resume' or cmd == 'start':
                        self.logger.logger.info("▶️  START command received (legacy format)")
                        if self.state_machine.state == VehicleState.STOPPED:
                            self.state_machine.transition_to(VehicleState.FOLLOWING_PATH)
                    elif cmd == 'shutdown':
                        self.kill_event.set()
                
                if 'v_ref' in commands:
                    new_v_ref = float(commands['v_ref'])
                    if 0 <= new_v_ref <= 2.0:
                        self.v_ref = new_v_ref
                        self.logger.logger.info(f"Updated v_ref to {new_v_ref} m/s")
        
        except Exception as e:
            self.logger.log_error("Failed to process commands", e)
    
    def _shutdown(self):
        """Shutdown all systems"""
        self.logger.logger.info("Shutting down...")
        self.state_machine.transition_to(VehicleState.SHUTTING_DOWN)
        
        # Stop vehicle
        if self.qcar:
            try:
                self.qcar.write(throttle=0, steering=0)
            except:
                pass
        
        # Close network
        if self.network:
            self.network.close()
        
        # Log final statistics
        self.logger.logger.info("="*60)
        self.logger.logger.info("Final Statistics:")
        self.logger.logger.info(f"Total iterations: {self.loop_counter}")
        self.logger.logger.info(f"Total time: {self.elapsed_time():.2f}s")
        
        perf_stats = self.perf_monitor.get_statistics()
        if 'loop_time' in perf_stats:
            self.logger.logger.info(
                f"Average loop frequency: {perf_stats['loop_time']['frequency']:.1f} Hz"
            )
        
        self.logger.logger.info("="*60)
        
        # Close logger
        self.logger.close()
