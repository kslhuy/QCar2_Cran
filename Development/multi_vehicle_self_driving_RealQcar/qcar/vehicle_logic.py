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
from StateMachine import VehicleState, VehicleStateMachine
from ground_station_client import GroundStationClient
from controllers import SpeedController, SteeringController, StateEstimator
from safety import ControlValidator, SensorHealthMonitor, CollisionAvoidance, WatchdogTimer
from utils import YOLOReceiver, YOLODriveLogic
from platoon_controller import PlatoonController, PlatoonConfig
from command_handler import CommandHandler
from v2v_communication import V2VCommunication


class VehicleLogic:
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
        

        
        # Platoon controller
        platoon_config = PlatoonConfig()
        # self.platoon_controller = PlatoonController(platoon_config, self.logger)
        
        # Command handler for centralized command processing
        self.command_handler = CommandHandler(self.logger, config)
        
        # V2V Communication system - High-performance UDP
        self.v2v_communication = V2VCommunication(
            vehicle_id=config.network.car_id,
            logger=self.logger.logger,
            base_port=8000,  # Dedicated V2V port range (8000+ for better separation)
            status_callback=self._handle_v2v_status_change  # Add callback for immediate status reporting
        )
        
        # Setup V2V message handlers
        self._setup_v2v_handlers()
        
        # Safety systems
        # self.validator = ControlValidator(config, self.logger)
        # self.sensor_health = SensorHealthMonitor(config, self.logger)
        self.collision_avoidance = CollisionAvoidance(config, self.logger)
        self.watchdog = WatchdogTimer(config.safety.watchdog_timeout, self.logger)
        
        # Components (initialized later)
        self.logger.logger.info("Creating Ground Station client...")
        # # Create Ground Station client
        # self.client_Ground_Station = GroundStationClient(
        #     config=self.config,
        #     logger=self.logger,
        #     kill_event=self.kill_event
        # )
        self._initialize_network_2_GroundStation()
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

        # State machine - use simplified version with internal transition logic
        self.state_machine = VehicleStateMachine(self, self.logger)
        
        # Timing
        self.start_time = time.time()
        self.loop_counter = 0
        self.telemetry_counter = 0
        
    def elapsed_time(self) -> float:
        """Get elapsed time since start"""
        return time.time() - self.start_time
       
    
    def run(self):
        """Main control loop"""
        self.logger.logger.info("Starting control loop...")
        
        # Start the state machine in INITIALIZING state
        # The state machine will handle all initialization through its states
        
        # CRITICAL: Reset start_time NOW
        self.start_time = time.time()
        self.loop_counter = 0
        self.telemetry_counter = 0
        
        # Main control loop
        target_dt = 1.0 / self.config.timing.controller_update_rate
        last_loop_time = time.time()
        
        try:
            while not self.kill_event.is_set():
                loop_start = time.time()
                actual_dt = loop_start - last_loop_time
                last_loop_time = loop_start
                
                # Reset watchdog
                if hasattr(self, 'watchdog'):
                    self.watchdog.reset()
                
                # Run one control iteration
                if not self._control_iteration(actual_dt):
                    self.logger.log_error("Control iteration failed")
                    break
                
                # Performance monitoring
                loop_time = time.time() - loop_start
                if hasattr(self, 'perf_monitor'):
                    self.perf_monitor.log_loop_time(loop_time)
                
                # Sleep to maintain loop rate
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
            # Check if components are initialized
            if self.qcar is None or self.state_estimator is None:
                # During initialization, just update state machine without sensor readings
                if self.state_machine.state == VehicleState.INITIALIZING:
                    # Minimal sensor data for initialization
                    sensor_data = {
                        'x': 0.0, 'y': 0.0, 'theta': 0.0, 'velocity': 0.0,
                        'motor_tach': 0.0, 'gyro_z': 0.0,
                        'yolo_data': {
                            'stop_sign': [0]*7, 'traffic_light': [0]*7, 'cars': [0]*7,
                            'yield_sign': [0]*7, 'person': [0]*7, 'car_dist': 0.0, 'person_dist': 0.0
                        },
                        'state_valid': False
                    }
                    
                    # Update state machine - this will handle initialization
                    u, delta = self.state_machine.update(dt, sensor_data)
                    
                    # Don't send commands during initialization
                    return True
                else:
                    self.logger.log_error("Components not initialized but not in INITIALIZING state")
                    return False
            
            # Normal operation - all components should be ready
            # Read QCar sensors
            self.qcar.read()
            
            # Get current steering (for EKF prediction)
            motor_tach = self.qcar.motorTach
            gyro_z = self.qcar.gyroscope[2] if hasattr(self.qcar, 'gyroscope') else 0.0
            
            # Get last steering command for EKF
            last_steering = getattr(self, '_last_steering', 0.0)
            
            # Update state estimate with EKF fusion (GPS + IMU + odometry) 
            # EKF stays outside the state machine as requested
            self.state_estimator.update(
                motor_tach=motor_tach,
                steering=last_steering,
                dt=dt,
                gyro_z=gyro_z
            )
            
            x, y, theta, velocity, state_valid = self.state_estimator.get_state()
            
            # Read YOLO detections (with safety check)
            if self.yolo is not None:
                self.yolo.read()
                
                # Process YOLO and get velocity gain
                if self.yolo_drive is not None:
                    try:
                        self.yolo_gain = self.yolo_drive.check_yolo(
                            self.yolo.stopSign,
                            self.yolo.trafficlight,
                            self.yolo.cars,
                            self.yolo.yieldSign,
                            self.yolo.person
                        )
                    except Exception as e:
                        if self.loop_counter % 100 == 0:  # Log occasionally
                            self.logger.log_error("YOLO drive error", e)
                        self.yolo_gain = 1.0  # Default gain
                else:
                    self.yolo_gain = 1.0
                    
                # Prepare YOLO data
                yolo_data = {
                    'stop_sign': self.yolo.stopSign,
                    'traffic_light': self.yolo.trafficlight,
                    'cars': self.yolo.cars,
                    'yield_sign': self.yolo.yieldSign,
                    'person': self.yolo.person,
                    'car_dist': getattr(self.yolo_drive, 'carDist', 0.0),
                    'person_dist': getattr(self.yolo_drive, 'personDist', 0.0)
                }
            else:
                # No YOLO available
                
                self.yolo_gain = 1.0
                yolo_data = {
                    'stop_sign': [0]*7, 'traffic_light': [0]*7, 'cars': [0]*7,
                    'yield_sign': [0]*7, 'person': [0]*7, 'car_dist': 0.0, 'person_dist': 0.0
                }
            
            # Prepare sensor data for state machine
            sensor_data = {
                'x': x,
                'y': y,
                'theta': theta,
                'velocity': velocity,
                'motor_tach': motor_tach,
                'gyro_z': gyro_z,
                'yolo_data': yolo_data,
                'state_valid': state_valid
            }
            
            # Update state machine - it handles all state logic and transitions internally
            u, delta = self.state_machine.update(dt, sensor_data)
            
            # Store steering for next EKF update
            self._last_steering = delta
            
            # Send commands to vehicle hardware
            if self.qcar is not None:
                self.qcar.write(throttle=u, steering=delta)
            
            # Send telemetry (non-blocking via queue)
            if self.loop_counter % 10 == 0:  # Send telemetry every 10 iterations (20Hz at 200Hz loop for better V2V)
                self._queue_telemetry(x, y, theta, velocity, u, delta)
            
            # Process V2V messages (separate from telemetry sending)
            if self.loop_counter % 5 == 0:  # Process V2V messages every 5 iterations = 40Hz
                self._process_v2v_messages()
            
            # Receive commands (non-blocking via queue)
            if self.loop_counter % 10 == 0:  # Check every 10 iterations = 20Hz
                self._process_queued_commands()
            
            return True
            
        except Exception as e:
            self.logger.log_error("Control iteration error", e)
            return False
    
    def _queue_telemetry(self, x: float, y: float, theta: float, velocity: float, u: float, delta: float):
        """Queue telemetry data for non-blocking transmission"""
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
            'state': self.state_machine.state.name if hasattr(self.state_machine, 'state') and self.state_machine.state else 'UNKNOWN',
            'gps_valid': self.state_estimator.state_valid if self.state_estimator else False,
            # V2V UDP status in telemetry for GUI updates
            'v2v_active': self.v2v_communication.is_active if hasattr(self, 'v2v_communication') else False,
            'v2v_peers': len(self.v2v_communication.get_connected_peers()) if hasattr(self, 'v2v_communication') else 0,
            'v2v_protocol': 'UDP' if hasattr(self, 'v2v_communication') and self.v2v_communication.is_active else 'None',
            'v2v_rate_hz': self.v2v_communication.get_statistics().get('actual_rate_hz', 0.0) if hasattr(self, 'v2v_communication') and self.v2v_communication.is_active else 0.0
            # Platoon telemetry
            # **self.platoon_controller.get_telemetry()
        }
        
        # Log to file
        if self.config.logging.enable_telemetry_logging:
            self.logger.log_telemetry(telemetry)
        
        # Queue for network transmission (non-blocking)
        if self.client_Ground_Station:
            try:
                # Check if client has is_connected method, if not assume connected
                is_connected = getattr(self.client_Ground_Station, 'is_connected', lambda: True)()
                if is_connected:
                    send_rate = self.config.timing.telemetry_send_rate
                    if self.telemetry_counter % (self.config.timing.controller_update_rate // send_rate) == 0:
                        self.client_Ground_Station.queue_telemetry(telemetry)
            except Exception as e:
                if self.loop_counter % 100 == 0:  # Log error only occasionally to avoid spam
                    self.logger.log_error("Telemetry transmission error", e)
        
            # Send V2V telemetry separately and simplified
            self._send_v2v_telemetry(x, y, theta, velocity)
        
        self.telemetry_counter += 1
    
    def _send_v2v_telemetry(self, x: float, y: float, theta: float, velocity: float):
        """Send V2V telemetry data - optimized for UDP high-frequency"""
        try:
            if not hasattr(self, 'v2v_communication') or not self.v2v_communication.is_active:
                return
            
            # Send V2V telemetry using optimized high-frequency method (20Hz rate-limited)
            if self.telemetry_counter % 10 == 0:  # Try every 10 iterations, but V2V will rate-limit to 20Hz
                vehicle_state = {
                    'x': x, 
                    'y': y, 
                    'theta': theta, 
                    'velocity': velocity,
                    'v_ref': self.v_ref * self.yolo_gain,
                    'state': self.state_machine.state.name if hasattr(self.state_machine, 'state') and self.state_machine.state else 'UNKNOWN',
                    'waypoint_index': self.steering_controller.get_waypoint_index() if self.steering_controller else 0,
                    'gps_valid': self.state_estimator.state_valid if self.state_estimator else False
                }
                
                # Use high-frequency optimized V2V telemetry method (UDP rate-limited to 20Hz)
                success = self.v2v_communication.send_telemetry(vehicle_state)
                
                # Log performance stats occasionally
                if success and self.telemetry_counter % 100 == 0:  # Every 100 tries
                    v2v_stats = self.v2v_communication.get_statistics()
                    self.logger.logger.info(f"[V2V UDP] {v2v_stats['actual_rate_hz']:.1f}Hz to {v2v_stats['peer_count']} peers (Sent: {v2v_stats['messages_sent']}, Recv: {v2v_stats['messages_received']})")
            
            # Report V2V status to Ground Station every 5 seconds (more frequent for UDP monitoring)
            if self.telemetry_counter % 100 == 0:
                self._report_v2v_status()
                
        except Exception as e:
            self.logger.log_error("V2V telemetry sending error", e)
    
    def _report_v2v_status(self):
        """Report V2V connection status - simplified"""
        try:
            if not hasattr(self, 'v2v_communication') or not self.v2v_communication.is_active:
                return
            
            v2v_stats = self.v2v_communication.get_statistics()
            connected_peers = self.v2v_communication.get_connected_peers()
            is_fully_connected = self.v2v_communication.is_fully_connected()
            connection_summary = self.v2v_communication.get_connection_summary()
            
            # Report to Ground Station with UDP performance metrics
            status_data = {
                'status': 'connected' if is_fully_connected else 'active',
                'vehicle_id': self.config.network.car_id,
                'connected_peers': len(connected_peers),
                'expected_peers': len([v for v in self.v2v_communication.peer_vehicles if v != self.config.network.car_id]),
                'peer_list': connected_peers,
                'messages_sent': v2v_stats.get('messages_sent', 0),
                'messages_received': v2v_stats.get('messages_received', 0),
                'packets_dropped': v2v_stats.get('packets_dropped', 0),
                'send_rate_hz': v2v_stats.get('actual_rate_hz', 0.0),
                'target_rate_hz': v2v_stats.get('target_rate_hz', 20.0),
                'protocol': 'UDP',
                'is_fully_connected': is_fully_connected,
                'timestamp': time.time()
            }
            
            self._report_v2v_status_to_gs(status_data)
            
            # Log status
            if is_fully_connected:
                self.logger.logger.info(
                    f"V2V STATUS: {connection_summary} | "
                    f"Sent: {v2v_stats.get('messages_sent', 0)}, Received: {v2v_stats.get('messages_received', 0)}"
                )
            else:
                expected_count = len([v for v in self.v2v_communication.peer_vehicles if v != self.config.network.car_id])
                self.logger.logger.warning(
                    f"V2V STATUS: {connection_summary} | "
                    f"Connected: {len(connected_peers)}/{expected_count} | Peers: {connected_peers}"
                )
                
        except Exception as e:
            self.logger.log_error("V2V status reporting error", e)
    
    def _process_v2v_messages(self):
        """Process incoming V2V messages - called frequently for responsive communication"""
        try:
            if not hasattr(self, 'v2v_communication') or not self.v2v_communication.is_active:
                return
            
            # Get and process pending messages (higher throughput for UDP)
            messages = self.v2v_communication.get_messages(max_count=20)  # Process more messages per iteration
            
            # Debug: Print when we actually get messages (less frequent for UDP performance)
            if messages and len(messages) > 5:  # Only log when significant message volume
                self.logger.logger.debug(f"[V2V-UDP] Car {self.config.network.car_id} processing {len(messages)} V2V messages")
            
            # Process each message immediately
            for message in messages:
                if message.message_type == 'telemetry':
                    # Initialize storage if needed
                    if not hasattr(self, 'peer_data_storage'):
                        self.peer_data_storage = {}
                        self.peer_message_counts = {}
                    
                    # Store peer data for analysis
                    self.peer_data_storage[message.sender_id] = {
                        'position': message.data.get('position', [0, 0]),
                        'velocity': message.data.get('velocity', 0),
                        'heading': message.data.get('heading', 0),
                        'state': message.data.get('state', 'UNKNOWN'),
                        'last_update': time.time(),
                        'v_ref': message.data.get('v_ref', 0),
                        'yolo_gain': message.data.get('yolo_gain', 1.0)
                    }
                    
                    self.peer_message_counts[message.sender_id] = self.peer_message_counts.get(message.sender_id, 0) + 1
                    
                    # # Debug: Print every received message for troubleshooting
                    # print(f"[V2V] Car {self.config.network.car_id} PROCESSED telemetry from Car {message.sender_id} (#{self.peer_message_counts[message.sender_id]})")
            
            # Log message processing every 2 seconds
            if messages and self.loop_counter % 400 == 0:
                total_received = sum(self.peer_message_counts.values()) if hasattr(self, 'peer_message_counts') else 0
                self.logger.logger.info(
                    f"V2V MESSAGES: Processed {len(messages)} messages | "
                    f"Total received: {total_received} | "
                    f"Peers: {list(self.peer_message_counts.keys()) if hasattr(self, 'peer_message_counts') else []}"
                )
                
        except Exception as e:
            self.logger.log_error("V2V message processing error", e)
            print(f"[V2V ERROR] Car {self.config.network.car_id} message processing error: {e}")
    
    def _analyze_peer_data(self):
        """Analyze collected peer data for safety and coordination insights"""
        try:
            if not hasattr(self, 'peer_data_storage') or not self.peer_data_storage:
                return
            
            current_time = time.time()
            my_x, my_y, my_theta, my_velocity, _ = self.state_estimator.get_state() if self.state_estimator else (0, 0, 0, 0, True)
            
            analysis_summary = []
            close_peers = []
            
            for peer_id, peer_data in self.peer_data_storage.items():
                # Check if data is recent (within last 5 seconds)
                data_age = current_time - peer_data.get('last_update', 0)
                if data_age > 5.0:
                    continue
                
                # Calculate relative position and distance
                peer_pos = peer_data.get('position', [0, 0])
                distance = ((my_x - peer_pos[0])**2 + (my_y - peer_pos[1])**2)**0.5
                
                peer_vel = peer_data.get('velocity', 0)
                peer_state = peer_data.get('state', 'UNKNOWN')
                msg_count = self.peer_message_counts.get(peer_id, 0)
                
                analysis_summary.append(
                    f"Car {peer_id}: Dist={distance:.1f}m, Vel={peer_vel:.1f}m/s, "
                    f"State={peer_state}, Msgs={msg_count}, Age={data_age:.1f}s"
                )
                
                # Track close peers for safety
                if distance < 10.0:  # Within 10 meters
                    close_peers.append((peer_id, distance, peer_vel))
            
            # Log comprehensive analysis
            if analysis_summary:
                self.logger.logger.info(
                    f"V2V PEER ANALYSIS: {len(analysis_summary)} active peers | " + 
                    " | ".join(analysis_summary)
                )
            
            # Log close peers for safety awareness
            if close_peers:
                close_summary = ", ".join([f"Car {pid}({dist:.1f}m@{vel:.1f}m/s)" for pid, dist, vel in close_peers])
                self.logger.logger.warning(f"V2V CLOSE PEERS: {close_summary}")
            
            # Report fleet status when fully connected - only if we have active peer data
            if analysis_summary:
                connected_peers = self.v2v_communication.get_connected_peers()
                if len(connected_peers) >= len(self.v2v_communication.peer_vehicles) - 1:
                    self.logger.logger.info(
                        f"V2V FLEET STATUS: FULLY CONNECTED | "
                        f"Car {self.config.network.car_id} coordinating with {len(connected_peers)} peers | "
                        f"Active data from {len(analysis_summary)} vehicles | Summary: " + " | ".join(analysis_summary)
                    )
            
        except Exception as e:
            self.logger.log_error("V2V peer data analysis error", e)
    
    def _process_queued_commands(self):
        """Process commands from queue (non-blocking)"""
        if self.client_Ground_Station:
            try:
                # Check if client has is_connected method, if not assume connected
                is_connected = getattr(self.client_Ground_Station, 'is_connected', lambda: True)()
                if is_connected:
                    commands = self.client_Ground_Station.get_latest_commands()
                    if commands:
                        # Use centralized command handler instead of direct processing
                        success = self.command_handler.process_command(commands)
                        if not success:
                            self.logger.log_warning("Failed to process command")
            except Exception as e:
                if self.loop_counter % 100 == 0:  # Log error only occasionally
                    self.logger.log_error("Command processing error", e)
    
    def _process_commands(self, commands: dict):
        """Legacy method - now redirects to command handler"""
        self.command_handler.process_command(commands)
    

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
            
            # For simplified state machine, we don't need complex transitions
            # Just check if we're at the start position
            if hasattr(self, 'roadmap') and self.roadmap and hasattr(self, 'node_sequence'):
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
                    self.logger.log_warning("NOT AT START POSITION")
                    self.logger.log_warning(f"  Current position: ({init_pose[0]:.2f}, {init_pose[1]:.2f}, {init_pose[2]:.2f})")
                    self.logger.log_warning(f"  Target node: {target_node}")
                    self.logger.log_warning(f"  Target position: ({target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f})")
                    self.logger.log_warning(f"  Distance to start: {current_dist:.2f}m")
                    self.logger.log_warning("="*60)
                    
                    # Update waypoint sequence to navigate to start
                    self.waypoint_sequence = init_waypoint_seq
                    if hasattr(self, 'steering_controller') and self.steering_controller:
                        self.steering_controller.reset(self.waypoint_sequence)
                    return False
                else:
                    self.logger.logger.info("✅ Vehicle is at start position")
                    return True
            else:
                # If no roadmap/steering, assume position is OK
                return True
            
        except Exception as e:
            self.logger.log_error("Initial position check failed", e)
            return False
    
    def _initialize_network_2_GroundStation(self) -> bool:
        """Initialize network communication"""
        try:
            self.logger.logger.info("Creating Ground Station client...")
            
            # Create Ground Station client
            self.client_Ground_Station = GroundStationClient(
                config=self.config,
                logger=self.logger,
                kill_event=self.kill_event
            )
            time.sleep(0.2)  # Allow client object to settle
            
            # Initialize network connection
            self.logger.logger.info("Initializing network connection...")
            if not self.client_Ground_Station.initialize_network():
                return False
            time.sleep(0.5)  # Allow network initialization to complete
            
            # Start network threads
            self.logger.logger.info("Starting network threads...")
            if not self.client_Ground_Station.start_threads():
                return False
            time.sleep(0.3)  # Allow threads to start properly
            
            self.logger.logger.info("Ground Station communication initialized")
            return True
            
        except Exception as e:
            self.logger.log_error("Ground Station initialization failed", e)
            return False
    
    def _setup_v2v_handlers(self):
        """Setup V2V message handlers for cooperative driving"""
        try:
            # Initialize peer data storage
            if not hasattr(self, 'peer_data_storage'):
                self.peer_data_storage = {}
                self.peer_message_counts = {}
            
            # Handler for receiving telemetry from other vehicles
            def handle_peer_telemetry(message):
                peer_id = message.sender_id
                peer_data = message.data
                current_time = time.time()
                
                # Store peer data for analysis
                self.peer_data_storage[peer_id] = {
                    'position': peer_data.get('position', [0, 0]),
                    'heading': peer_data.get('heading', 0),
                    'velocity': peer_data.get('velocity', 0),
                    'v_ref': peer_data.get('v_ref', 0),
                    'yolo_gain': peer_data.get('yolo_gain', 1.0),
                    'state': peer_data.get('state', 'UNKNOWN'),
                    'waypoint_index': peer_data.get('waypoint_index', 0),
                    'cross_track_error': peer_data.get('cross_track_error', 0.0),
                    'heading_error': peer_data.get('heading_error', 0.0),
                    'gps_valid': peer_data.get('gps_valid', False),
                    'is_fully_connected': peer_data.get('is_fully_connected', False),
                    'last_update': current_time,
                    'timestamp': peer_data.get('timestamp', current_time)
                }
                
                # Count messages from each peer
                self.peer_message_counts[peer_id] = self.peer_message_counts.get(peer_id, 0) + 1
                msg_count = self.peer_message_counts.get(peer_id, 0)
                
                # Enhanced logging of received telemetry - log every 5 messages (independent of loop counter)
                if msg_count % 5 == 0:
                    position = peer_data.get('position', [0, 0])
                    velocity = peer_data.get('velocity', 0)
                    state = peer_data.get('state', 'UNKNOWN')
                    
                    self.logger.logger.info(
                        f"V2V TELEMETRY RECEIVED: From Car {peer_id} | "
                        f"Position: ({position[0]:.2f}, {position[1]:.2f}), "
                        f"Vel: {velocity:.2f}m/s, State: {state} | "
                        f"Messages: {msg_count}, Latency: {(current_time - peer_data.get('timestamp', current_time)) * 1000:.1f}ms"
                    )
                
                # Use peer data for collision avoidance, coordination, etc.
                if hasattr(self, 'collision_avoidance'):
                    try:
                        # Only call if method exists
                        if hasattr(self.collision_avoidance, 'update_peer_vehicle'):
                            self.collision_avoidance.update_peer_vehicle(peer_id, peer_data)
                    except Exception as e:
                        # Silently ignore collision avoidance update errors
                        pass
            
            # Handler for receiving driving intents from other vehicles
            def handle_peer_intent(message):
                peer_id = message.sender_id
                intent_data = message.data
                intention = intent_data.get('intention', 'unknown')
                
                self.logger.logger.info(f"V2V: Peer {peer_id} intent: {intention}")
                
                # TODO: React to peer intentions (lane change, merge, etc.)
            
            # Handler for receiving warnings from other vehicles
            def handle_peer_warning(message):
                peer_id = message.sender_id
                warning_data = message.data
                warning_type = warning_data.get('warning_type', 'unknown')
                urgency = warning_data.get('urgency', 'low')
                
                self.logger.logger.warning(
                    f"V2V: Warning from peer {peer_id}: {warning_type} (urgency: {urgency})"
                )
                
                # TODO: React to warnings (emergency brake, route change, etc.)
                if urgency == 'critical' and hasattr(self, 'state_machine'):
                    # Example: trigger emergency response for critical warnings
                    pass
            
            # Register message handlers
            self.v2v_communication.register_message_handler('telemetry', handle_peer_telemetry)
            self.v2v_communication.register_message_handler('intent', handle_peer_intent)
            self.v2v_communication.register_message_handler('warning', handle_peer_warning)
            
            self.logger.logger.info("V2V message handlers registered")
            
        except Exception as e:
            self.logger.log_error("V2V handler setup failed", e)
    
    def activate_v2v(self, peer_vehicles: list, peer_ips: list) -> bool:
        """Activate V2V communication with specified peers"""
        try:
            success = self.v2v_communication.activate(peer_vehicles, peer_ips)
            if success:
                # Don't block the main loop waiting for connections
                # Just report that activation started
                
                total_expected = len([v for v in peer_vehicles if v != self.config.network.car_id])
                
                self.logger.logger.info(f"V2V service started. Waiting for {total_expected} peers...")
                
                # Report V2V activation to Ground Station (status='activating')
                self._report_v2v_status_to_gs({
                    'status': 'activating',
                    'expected_peers': total_expected,
                    'connected_peers': 0,
                    'peer_list': [],
                    'vehicle_id': self.config.network.car_id,
                    'timestamp': time.time()
                })
                
                # Return True if service started successfully
                return True
            else:
                self.logger.logger.error("Failed to activate V2V communication")
                # Report failure to Ground Station
                self._report_v2v_status_to_gs({
                    'status': 'failed',
                    'error': 'activation_failed',
                    'vehicle_id': self.config.network.car_id,
                    'timestamp': time.time()
                })
                return False
        except Exception as e:
            self.logger.log_error("V2V activation error", e)
            # Report error to Ground Station
            self._report_v2v_status_to_gs({
                'status': 'error',
                'error': str(e),
                'vehicle_id': self.config.network.car_id,
                'timestamp': time.time()
            })
            return False
    
    def disable_v2v(self):
        """Disable V2V communication"""
        try:
            self.v2v_communication.deactivate()
            self.logger.logger.info("V2V communication disabled")
            
            # Report V2V disconnect to Ground Station
            self._report_v2v_status_to_gs({
                'status': 'disconnected',
                'vehicle_id': self.config.network.car_id,
                'timestamp': time.time()
            })
            
        except Exception as e:
            self.logger.log_error("V2V disable error", e)
    
    def _handle_v2v_status_change(self, event_type: str, peer_id: int):
        """Handle immediate V2V status changes from the communication module"""
        try:
            connected_peers = len(self.v2v_communication.get_connected_peers())
            total_expected = len([v for v in self.v2v_communication.peer_vehicles if v != self.config.network.car_id])
            
            if event_type == 'connection_established':
                self.logger.logger.info(f"V2V: Peer {peer_id} connected ({connected_peers}/{total_expected} peers)")
                
                # Check if we're fully connected
                if connected_peers >= total_expected and total_expected > 0:
                    status = 'connected'
                    self.logger.logger.info("V2V: All peers connected - mesh is fully established!")
                else:
                    status = 'active'
                    
            elif event_type == 'connection_lost':
                self.logger.logger.info(f"V2V: Peer {peer_id} disconnected ({connected_peers}/{total_expected} peers)")
                status = 'active' if connected_peers > 0 else 'disconnected'
            else:
                return  # Unknown event type
            
            # Report status change immediately to Ground Station
            self._report_v2v_status_to_gs({
                'status': status,
                'vehicle_id': self.config.network.car_id,
                'connected_peers': connected_peers,
                'expected_peers': total_expected,
                'peer_list': self.v2v_communication.get_connected_peers(),
                'event': event_type,
                'affected_peer': peer_id,
                'timestamp': time.time()
            })
            
        except Exception as e:
            self.logger.log_error(f"Error handling V2V status change: {event_type}", e)
    
    def _report_v2v_status_to_gs(self, status_data: dict):
        """Report V2V connection status to Ground Station"""
        try:
            if self.client_Ground_Station:
                # Create V2V status report
                v2v_report = {
                    'type': 'v2v_status',
                    'car_id': self.config.network.car_id,
                    'data': status_data
                }
                
                # Send immediately (high priority)
                self.client_Ground_Station.queue_telemetry(v2v_report)
                
                self.logger.logger.info(f"V2V status reported to GS: {status_data['status']}")
            else:
                self.logger.logger.warning("Cannot report V2V status - no Ground Station connection")
                
        except Exception as e:
            self.logger.log_error("Failed to report V2V status to Ground Station", e)
    def _shutdown(self):
        """Shutdown all systems"""
        self.logger.logger.info("Shutting down...")
        
        # Use emergency_stop method instead of non-existent SHUTTING_DOWN state
        try:
            self.state_machine.emergency_stop("System shutdown requested")
        except Exception as e:
            self.logger.log_error("Error during state machine shutdown", e)
        
        # Stop vehicle
        if self.qcar:
            try:
                self.qcar.write(throttle=0, steering=0)
            except Exception as e:
                self.logger.log_error("Error stopping vehicle", e)
        
        # Close network handler and stop threads
        if self.client_Ground_Station:
            self.client_Ground_Station.close()
        
        # Shutdown V2V communication
        if hasattr(self, 'v2v_communication'):
            try:
                self.v2v_communication.deactivate()
            except Exception as e:
                self.logger.log_error("V2V shutdown error", e)
        
        # Log final statistics
        self.logger.logger.info("="*60)
        self.logger.logger.info("Final Statistics:")
        self.logger.logger.info(f"Total iterations: {self.loop_counter}")
        self.logger.logger.info(f"Total time: {self.elapsed_time():.2f}s")
        
        # Log network statistics
        if self.client_Ground_Station:
            net_stats = self.client_Ground_Station.get_statistics()
            self.logger.logger.info(f"Network - Telemetry sent: {net_stats['telemetry_sent']}, Commands received: {net_stats['commands_received']}")
            if net_stats['queue_overflows'] > 0:
                self.logger.logger.info(f"Network queue overflows: {net_stats['queue_overflows']}")
        
        perf_stats = self.perf_monitor.get_statistics()
        if 'loop_time' in perf_stats:
            self.logger.logger.info(
                f"Average loop frequency: {perf_stats['loop_time']['frequency']:.1f} Hz"
            )
        
        self.logger.logger.info("="*60)
        
        # Close logger
        self.logger.close()
