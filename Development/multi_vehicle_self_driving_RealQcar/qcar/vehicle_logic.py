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
        
        # State machine - use simplified version with internal transition logic
        self.state_machine = VehicleStateMachine(self, self.logger)
        
        # Platoon controller
        platoon_config = PlatoonConfig()
        self.platoon_controller = PlatoonController(platoon_config, self.logger)
        
        # Command handler for centralized command processing
        self.command_handler = CommandHandler(self.logger, config)
        
        # V2V Communication system
        self.v2v_communication = V2VCommunication(
            vehicle_id=config.network.car_id,
            logger=self.logger.logger,
            base_port=config.network.base_port + 1000  # Use separate port range for V2V
        )
        
        # Setup V2V message handlers
        self._setup_v2v_handlers()
        
        # Safety systems
        # self.validator = ControlValidator(config, self.logger)
        # self.sensor_health = SensorHealthMonitor(config, self.logger)
        self.collision_avoidance = CollisionAvoidance(config, self.logger)
        self.watchdog = WatchdogTimer(config.safety.watchdog_timeout, self.logger)
        
        # Components (initialized later)
        self.client_Ground_Station = None
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
            if self.loop_counter % 20 == 0:  # Send every 20 iterations = 10Hz at 200Hz loop
                self._queue_telemetry(x, y, theta, velocity, u, delta)
            
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
            'state': self.state_machine.state.name,
            'gps_valid': self.state_estimator.state_valid if self.state_estimator else False,
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
        
            # Send telemetry via V2V communication
            if hasattr(self, 'v2v_communication') and self.v2v_communication.is_active:
                try:
                    if self.telemetry_counter % 40 == 0:  # Send V2V telemetry at 5Hz (200/40)
                        self.v2v_communication.send_telemetry({
                            'x': x, 'y': y, 'theta': theta, 'velocity': velocity
                        })
                        
                        # Periodically report V2V statistics to Ground Station
                        if self.telemetry_counter % 200 == 0:  # Every 10 seconds
                            v2v_stats = self.v2v_communication.get_statistics()
                            self._report_v2v_status_to_gs({
                                'status': 'active',
                                'vehicle_id': self.config.network.car_id,
                                'connected_peers': v2v_stats.get('connected_peers', 0),
                                'messages_sent': v2v_stats.get('messages_sent', 0),
                                'messages_received': v2v_stats.get('messages_received', 0),
                                'timestamp': time.time()
                            })
                            
                except Exception as e:
                    if self.loop_counter % 100 == 0:
                        self.logger.log_error("V2V telemetry error", e)
        
        self.telemetry_counter += 1
    
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
    
    def _setup_v2v_handlers(self):
        """Setup V2V message handlers for cooperative driving"""
        try:
            # Handler for receiving telemetry from other vehicles
            def handle_peer_telemetry(message):
                peer_id = message.sender_id
                peer_data = message.data
                
                # Log peer vehicle information occasionally
                if self.loop_counter % 200 == 0:  # Every second at 200Hz
                    self.logger.logger.info(
                        f"V2V: Peer {peer_id} at ({peer_data.get('position', [0,0])[0]:.1f}, "
                        f"{peer_data.get('position', [0,0])[1]:.1f}), "
                        f"vel={peer_data.get('velocity', 0):.1f}m/s"
                    )
                
                # TODO: Use peer data for collision avoidance, coordination, etc.
                # if hasattr(self, 'collision_avoidance'):
                #     self.collision_avoidance.update_peer_vehicle(peer_id, peer_data)
            
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
                # Wait a moment for connections to establish
                time.sleep(2.0)
                
                # Get actual connected peers
                connected_peers = self.v2v_communication.get_connected_peers()
                total_expected = len([v for v in peer_vehicles if v != self.config.network.car_id])
                
                self.logger.logger.info(f"V2V communication activated: {len(connected_peers)}/{total_expected} peers connected")
                self.logger.logger.info(f"V2V connected to vehicles: {connected_peers}")
                
                # Report V2V success to Ground Station
                self._report_v2v_status_to_gs({
                    'status': 'connected',
                    'expected_peers': total_expected,
                    'connected_peers': len(connected_peers),
                    'peer_list': connected_peers,
                    'vehicle_id': self.config.network.car_id,
                    'timestamp': time.time()
                })
                
                return len(connected_peers) > 0  # Success if at least one connection
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
