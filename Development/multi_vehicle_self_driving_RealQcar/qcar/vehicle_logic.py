"""
Main Vehicle Controller - Integrates all components
"""
import numpy as np
import time
# import random
# from typing import Optional
from threading import Event

from pal.products.qcar import QCar, QCarGPS , IS_PHYSICAL_QCAR
# from hal.products.mats import SDCSRoadMap

from config import VehicleControlConfig
from logging_utils import VehicleLogger, PerformanceMonitor
from StateMachine import VehicleState, VehicleStateMachine
from ground_station_client import GroundStationClient
from Controller.controllers import SpeedController, SteeringController, StateEstimator
from safety import ControlValidator, SensorHealthMonitor, CollisionAvoidance, WatchdogTimer
from Yolo.YoLo import YOLOReceiver, YOLODriveLogic, YOLOManager
from Controller.platoon_controller import PlatoonController, PlatoonConfig
from command_handler import CommandHandler
from V2V.v2v_manager import V2VManager, V2VBroadcastConfig
from Observer.VehicleObserverSimple import VehicleObserver


class VehicleLogic:
    """Main vehicle controller class"""
    
    def __init__(self, config: VehicleControlConfig, kill_event: Event):
        self.config = config
        self.kill_event = kill_event

        # Vehicle identification
        # vehicle_id: Connection/network ID (used for Ground Station communication, file naming, etc.)
        self.vehicle_id = config.network.car_id
        
        # vehicle_position: Position in platoon formation (1=leader, 2=first follower, 3=second follower, etc.)
        # Initially set to vehicle_id, but can be changed by Ground Station platoon formation commands
        self.vehicle_position = config.network.car_id
        # Setup logging
        self.vehicle_logger = VehicleLogger(
            car_id=config.network.car_id,
            log_dir=config.logging.log_dir,
            log_level=config.logging.log_level
        )
        
        self.vehicle_logger.logger.info("="*60)
        self.vehicle_logger.logger.info(f"Vehicle Controller Initialized - Car ID: {config.network.car_id}")
        self.vehicle_logger.logger.info("="*60)
        
        # Performance monitoring
        self.perf_monitor = PerformanceMonitor(self.vehicle_logger)
        

        
        # Platoon controller
        platoon_config = PlatoonConfig()
        self.platoon_controller = PlatoonController(platoon_config, self.vehicle_logger)
        
        # Command handler for centralized command processing
        self.command_handler = CommandHandler(self.vehicle_logger, config)
        
        # V2V Manager - Complete V2V system (handles communication internally)
        v2v_config = V2VBroadcastConfig(
            local_state_frequency=25.0,  # Hz - High frequency for local states
            fleet_state_frequency=10.0,   # Hz - Lower frequency for fleet states
            heartbeat_frequency=1.0      # Hz - Very low frequency for heartbeats
        )
        self.v2v_manager = V2VManager(
            vehicle_id=config.network.car_id,
            logger=self.vehicle_logger.logger,
            config=v2v_config,
            vehicle_observer=None,  # Will be set later when vehicle_observer is created
            base_port=8000,
            status_callback=self._handle_v2v_status_change,
            vehicle_logic=self  # Pass reference to self for Ground Station reporting
        )
        
        # Safety systems
        # self.validator = ControlValidator(config, self.vehicle_logger)
        # self.sensor_health = SensorHealthMonitor(config, self.vehicle_logger)
        self.collision_avoidance = CollisionAvoidance(config, self.vehicle_logger)
        self.watchdog = WatchdogTimer(config.safety.watchdog_timeout, self.vehicle_logger)
        
        # Components (initialized later)
        self.vehicle_logger.logger.info("Creating Ground Station client...")
        # # Create Ground Station client
        # self.client_Ground_Station = GroundStationClient(
        #     config=self.config,
        #     logger=self.vehicle_logger,
        #     kill_event=self.kill_event
        # )
        self._initialize_network_2_GroundStation()
        self.logger.logger.info("Initializing QCar hardware...")
            
        # self.qcar = QCar(
        #     readMode=1,
        #     frequency=self.config.timing.controller_update_rate
        # )
        self.qcar = None
        self.gps = None
        
        # StateEstimator is now managed by VehicleObserver

        self.speed_controller = None
        self.steering_controller = None
        
        # YOLO Manager - handles all YOLO-related functionality
        self.yolo_manager = YOLOManager(self.vehicle_logger)
        
        # Path planning
        self.roadmap = None
        self.waypoint_sequence = None
        self.node_sequence = None
        
        # Control state (Can be set by Ground Station commands)
        self.v_ref = config.speed.v_ref
        
        # Calibration state flag (set by CALIBRATE command)
        self.calibration_requested = False

        # State machine - use simplified version with internal transition logic
        self.state_machine = VehicleStateMachine(self, self.vehicle_logger)
        
        # Timing
        self.start_time = time.time()
        self.loop_counter = 0
        self.telemetry_counter = 0
        
        # Component update rates and timing
        self.controller_rate = config.timing.controller_update_rate if IS_PHYSICAL_QCAR else 100  # 200 for real vehicle, 100 for sim
        self.observer_rate = config.timing.observer_rate if IS_PHYSICAL_QCAR else 100  # 200 for real vehicle, 100 for sim
        self.telemetry_send_rate = getattr(config.timing, 'telemetry_send_rate', 10)
        
        # Timing trackers for different update rates
        self._last_observer_time = 0.0
        self._last_control_time = 0.0
        self._last_telemetry_send = 0.0
        
        # V2V status cache (updated every 1 second to avoid repeated queries)
        self._v2v_status_cache = {}
        self._v2v_status_cache_time = 0.0
        
        # Initialize Vehicle Observer for local and fleet state estimation
        # Fleet size starts at 1 and will be expanded when V2V activates
        # Initial pose will be set by StateEstimator during initialization
        self.vehicle_observer = VehicleObserver(
            vehicle_id=config.network.car_id,
            config={
                'observer_rate': self.observer_rate,
                'fleet_observer_rate': getattr(config.timing, 'fleet_observer_rate', 50),
                'observer': {
                    'local_observer_type': 'ekf',
                    'enable_distributed': True,
                    'consensus_gain': 0.3
                }
            },
            logger=self.vehicle_logger,
            state_estimator=None  # Will be set later during initialization
        )
        
        # Connect VehicleObserver to V2VManager
        self.v2v_manager.update_vehicle_observer(self.vehicle_observer)
        
        # Initialize the event system to connect command_handler to state_machine
        # This allows ground station commands to be properly routed to the current state
        self.state_machine.initialize_event_system()
        
    def elapsed_time(self) -> float:
        """Get elapsed time since start"""
        return time.time() - self.start_time
    
    @property
    def logger(self):
        """Backward compatibility property for accessing the vehicle logger"""
        return self.vehicle_logger
       
    
    def run(self):
        """Main control loop"""
        self.vehicle_logger.logger.info("Starting control loop...")
        
        # Start the state machine in INITIALIZING state
        # The state machine will handle all initialization through its states
        
        # CRITICAL: Reset start_time NOW
        self.start_time = time.time()
        self.loop_counter = 0
        self.telemetry_counter = 0
        
        # Main control loop
        target_dt = 1.0 / self.controller_rate
        last_loop_time = time.time()
        
        try:
            while not self.kill_event.is_set():
                loop_start = time.time()
                actual_dt = loop_start - last_loop_time
                last_loop_time = loop_start
                
                # Reset watchdog
                if hasattr(self, 'watchdog'):
                    self.watchdog.reset()
                
                # 1. Sensor Data Reading and GPS Update
                self._update_sensor_data(actual_dt)
                
                # 2. Observer Update (handles both local and fleet internally)
                if self._should_update_observer(loop_start):
                    self._observer_update(actual_dt)
                
                # 3. Control Logic (high frequency)
                if self._should_update_control(loop_start):
                    if not self._control_logic_update(actual_dt):
                        self.vehicle_logger.log_error("Control logic failed")
                        break
                
                # 4. Communication Tasks (each manages own rate internally)
                self._send_telemetry_to_ground_station()  # 10Hz internal rate-limiting
                self._process_queued_commands()  # No rate limit - process as fast as possible
                self._broadcast_v2v_state()  # V2VManager handles internal rate-limiting
                
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
                    self.vehicle_logger.logger.info("Experiment time limit reached")
                    break
        
        except KeyboardInterrupt:
            self.vehicle_logger.logger.info("Control loop interrupted by user")
        except Exception as e:
            self.vehicle_logger.log_error("Control loop error", e)
        finally:
            self._shutdown()
                
    # ===== Component Update Rate Control Methods =====
    def _should_update_observer(self, current_time: float) -> bool:
        """Check if local observer should update based on rate"""
        if current_time - self._last_observer_time >= 1.0 / self.observer_rate:
            self._last_observer_time = current_time
            return True
        return False
    
    def _should_update_control(self, current_time: float) -> bool:
        """Check if control should update based on rate"""
        if current_time - self._last_control_time >= 1.0 / self.controller_rate:
            self._last_control_time = current_time
            return True
        return False
    
    # ===== Sensor Data Reading Methods =====
    def _update_sensor_data(self, dt: float):
        """Update sensor data using VehicleObserver - called every loop iteration"""
        try:
            if self.qcar is not None:
                # Use VehicleObserver to update sensor data
                self.vehicle_observer.update_sensor_data(self.qcar)
                
                # Handle YOLO logic using YOLOManager
                self.yolo_manager.update_yolo_data(self.loop_counter)
                
        except Exception as e:
            self.vehicle_logger.log_error("Sensor data update error", e)
    

    
    # ===== Observer Update Methods =====
    def _observer_update(self, dt: float):
        """Unified observer update - handles both local and fleet observer internally"""
        try:
            # StateEstimator is now managed directly by VehicleObserver during initialization
            
            # Get last steering command for EKF
            last_steering = getattr(self, '_last_steering', 0.0)
            
            # Update observer (internally handles local and fleet timing)
            state_info = self.vehicle_observer.update_observer(
                dt, 
                last_steering
            )
            
            
            # Log observer state occasionally
            if self.loop_counter % 200 == 0:  # Every 2 seconds at 100Hz
                self.vehicle_logger.logger.debug(
                    f"Observer: Pos=({state_info['x']:.2f}, {state_info['y']:.2f}, {state_info['theta']:.2f}), "
                    f"Vel={state_info['velocity']:.2f}, Valid={state_info['state_valid']}"
                )
                
                # Log fleet observer status
                fleet_states = self.vehicle_observer.get_fleet_states()
                active_vehicles = np.sum(np.any(fleet_states != 0, axis=0))
                self.vehicle_logger.logger.debug(f"Fleet Observer: {active_vehicles} active vehicles in fleet")
                
        except Exception as e:
            self.vehicle_logger.log_error("Observer update error", e)
    
    # ===== Control Logic Methods =====
    def _control_logic_update(self, dt: float) -> bool:
        """Control logic update - state machine and vehicle commands"""
        try:
            # Check if components are initialized
            if self.qcar is None or (hasattr(self, 'vehicle_observer') and self.vehicle_observer.get_state_estimator() is None):
                return self._handle_initialization_control(dt)
            
            # Get current state from VehicleObserver
            state_info = self.vehicle_observer.get_estimated_state_for_control()
            x, y, theta, velocity = state_info['x'], state_info['y'], state_info['theta'], state_info['velocity']
            state_valid = state_info['state_valid']
            
            # Prepare YOLO data
            yolo_data = self.yolo_manager.get_yolo_data()
            
            # Prepare sensor data for state machine
            sensor_data = {
                'x': x, 'y': y, 'theta': theta, 'velocity': velocity,
                'motor_tach': state_info['motor_tach'],
                'gyro_z': state_info['gyro_z'],
                'yolo_data': yolo_data,
                'state_valid': state_valid
            }
            
            # Update state machine - it handles all state logic and transitions
            u, delta = self.state_machine.update(dt, sensor_data)
            
            # Store steering and throttle for next EKF update and telemetry
            self._last_steering = delta
            self._last_u = u
            
            # Send commands to vehicle hardware
            if self.qcar is not None:
                self.qcar.write(throttle=u, steering=delta)
            
            return True
            
        except Exception as e:
            self.vehicle_logger.log_error("Control logic update error", e)
            return False
    
    def _handle_initialization_control(self, dt: float) -> bool:
        """Handle control during initialization phase"""
        try:
            # During initialization, just update state machine without sensor readings
            if hasattr(self.state_machine, 'state') and self.state_machine.state == VehicleState.INITIALIZING:
                # Minimal sensor data for initialization
                sensor_data = {
                    'x': 0.0, 'y': 0.0, 'theta': 0.0, 'velocity': 0.0,
                    'motor_tach': 0.0, 'gyro_z': 0.0,
                    'yolo_data': self.yolo_manager.get_default_yolo_data(),
                    'state_valid': False
                }
                
                # Update state machine - this will handle initialization
                u, delta = self.state_machine.update(dt, sensor_data)
                
                # Don't send commands during initialization
                return True
            else:
                self.vehicle_logger.log_error("Components not initialized but not in INITIALIZING state")
                return False
                
        except Exception as e:
            self.vehicle_logger.log_error("Initialization control error", e)
            return False
    

    
    # ===== Communication Handling Methods =====
    def _send_telemetry_to_ground_station(self):
        """Send telemetry to Ground Station with internal 10Hz rate-limiting"""
        try:
            if not hasattr(self, 'vehicle_observer') or self.vehicle_observer is None:
                return
            
            # Rate-limiting: only send at telemetry_send_rate (10Hz)
            current_time = time.time()
            telemetry_interval = 1.0 / self.telemetry_send_rate
            if current_time - self._last_telemetry_send < telemetry_interval:
                return  # Skip this cycle
            
            # Build telemetry data
            telemetry = self._build_telemetry_data()
            
            # Log to file
            if self.config.logging.enable_telemetry_logging:
                self.vehicle_logger.log_telemetry(telemetry)
            
            # Send to Ground Station
            if self.client_Ground_Station:
                try:
                    is_connected = getattr(self.client_Ground_Station, 'is_connected', lambda: True)()
                    if is_connected:
                        self.client_Ground_Station.queue_telemetry(telemetry)
                        self.telemetry_counter += 1
                except Exception as e:
                    # Log errors occasionally to avoid spam
                    if self.loop_counter % 100 == 0:
                        self.vehicle_logger.log_error("Telemetry transmission error", e)
            
            self._last_telemetry_send = current_time
            
        except Exception as e:
            self.vehicle_logger.log_error("Telemetry sending error", e)
    
    def _build_telemetry_data(self) -> dict:
        """Build telemetry data dictionary - pure data collection"""
        # Get current state from VehicleObserver
        state_info = self.vehicle_observer.get_estimated_state_for_control()
        
        # Get cached V2V status (avoid repeated expensive queries)
        v2v_status = self._get_v2v_status_cache()
        
        # Get platoon status from platoon_controller
        platoon_status = self._get_platoon_status()
        
        # Get controller data safely
        waypoint_index = self.steering_controller.get_waypoint_index() if self.steering_controller else 0
        errors = self.steering_controller.get_errors() if self.steering_controller else (0.0, 0.0)
        
        return {
            'timestamp': time.time(),
            'time': self.elapsed_time(),
            'x': float(state_info['x']),
            'y': float(state_info['y']),
            'th': float(state_info['theta']),
            'v': float(state_info['velocity']),
            'u': float(getattr(self, '_last_u', 0.0)),
            'delta': float(getattr(self, '_last_steering', 0.0)),
            'v_ref': float(self.v_ref * self.yolo_manager.get_yolo_gain()),
            'yolo_gain': float(self.yolo_manager.get_yolo_gain()),
            'waypoint_index': waypoint_index,
            'cross_track_error': float(errors[0]),
            'heading_error': float(errors[1]),
            'state': self.state_machine.state.name if hasattr(self.state_machine, 'state') and self.state_machine.state else 'UNKNOWN',
            'gps_valid': self.vehicle_observer.is_state_valid() if hasattr(self, 'vehicle_observer') and self.vehicle_observer else False,
            # V2V status from cache
            **v2v_status,
            # Platoon status
            **platoon_status
        }
    
    def _get_v2v_status_cache(self) -> dict:
        """Get V2V status with caching to avoid repeated queries (updated every 1 second)"""
        current_time = time.time()
        
        # Update cache every 1 second (V2V status doesn't change frequently)
        if current_time - self._v2v_status_cache_time > 1.0:
            try:
                if hasattr(self, 'v2v_manager') and self.v2v_manager:
                    is_active = self.v2v_manager.is_active()
                    self._v2v_status_cache = {
                        'v2v_active': is_active,
                        'v2v_peers': len(self.v2v_manager.v2v_communication.peer_vehicles) if is_active else 0,
                        'v2v_protocol': 'UDP-Manager' if is_active else 'None',
                        'v2v_local_rate': self.v2v_manager.config.local_state_frequency,
                        'v2v_fleet_rate': self.v2v_manager.config.fleet_state_frequency
                    }
                else:
                    self._v2v_status_cache = {
                        'v2v_active': False,
                        'v2v_peers': 0,
                        'v2v_protocol': 'None',
                        'v2v_local_rate': 0.0,
                        'v2v_fleet_rate': 0.0
                    }
                self._v2v_status_cache_time = current_time
            except Exception as e:
                self.vehicle_logger.logger.warning(f"Error updating V2V status cache: {e}")
                # Return safe defaults on error
                self._v2v_status_cache = {
                    'v2v_active': False,
                    'v2v_peers': 0,
                    'v2v_protocol': 'None',
                    'v2v_local_rate': 0.0,
                    'v2v_fleet_rate': 0.0
                }
        
        return self._v2v_status_cache.copy()
    
    def _get_platoon_status(self) -> dict:
        """Get platoon status from platoon_controller for telemetry"""
        try:
            if hasattr(self, 'platoon_controller') and self.platoon_controller:
                return {
                    'platoon_enabled': self.platoon_controller.enabled,
                    'platoon_is_leader': self.platoon_controller.is_leader,
                    'platoon_position': getattr(self.platoon_controller, 'my_position', None),
                    'platoon_leader_id': self.platoon_controller.leader_car_id,
                    'platoon_setup_complete': getattr(self.platoon_controller, 'setup_complete', False)
                }
            else:
                return {
                    'platoon_enabled': False,
                    'platoon_is_leader': False,
                    'platoon_position': None,
                    'platoon_leader_id': None,
                    'platoon_setup_complete': False
                }
        except Exception as e:
            self.vehicle_logger.logger.error(f"Error getting platoon status: {e}")
            return {
                'platoon_enabled': False,
                'platoon_is_leader': False,
                'platoon_position': None,
                'platoon_leader_id': None,
                'platoon_setup_complete': False
            }
    
    def _broadcast_v2v_state(self):
        """Broadcast vehicle state to V2V network - V2VManager handles rate-limiting internally"""
        try:
            if not hasattr(self, 'v2v_manager') or self.v2v_manager is None:
                return
            
            # V2VManager.update_broadcast() handles all rate-limiting:
            # - Local state: 20Hz, Fleet state: 5Hz, Heartbeat: 1Hz
            self.v2v_manager.update_broadcast()
            
            # Periodic logging of V2V activity (every 5 seconds)
            if hasattr(self, '_last_v2v_log_time'):
                if time.time() - self._last_v2v_log_time > 5.0:
                    self._log_v2v_activity()
                    self._last_v2v_log_time = time.time()
            else:
                self._last_v2v_log_time = time.time()
                
        except Exception as e:
            self.vehicle_logger.log_error("V2V state broadcast error", e)
    
    def _log_v2v_activity(self):
        """Log V2V communication activity summary"""
        try:
            if hasattr(self, 'v2v_manager') and self.v2v_manager.is_active():
                status = self.v2v_manager.get_connection_status()
                stats = status.get('communication_stats', {})
                
                self.vehicle_logger.logger.debug(
                    f"V2V Activity - Fleet: {status.get('fleet_size', 0)}, "
                    f"Rate: {stats.get('actual_rate_hz', 0.0):.1f}Hz, "
                    f"Sent: {stats.get('messages_sent', 0)}, "
                    f"Recv: {stats.get('messages_received', 0)}"
                )
                
        except Exception as e:
            self.vehicle_logger.logger.error(f"V2V activity logging error: {e}")
    
    # V2V messages are now processed automatically via V2VManager
    # No need for manual polling - this eliminates duplicate processing
    
    
    
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
                            self.vehicle_logger.log_warning("Failed to process command")
            except Exception as e:
                if self.loop_counter % 100 == 0:  # Log error only occasionally
                    self.vehicle_logger.log_error("Command processing error", e)
    
    def _process_commands(self, commands: dict):
        """Legacy method - now redirects to command handler"""
        self.command_handler.process_command(commands)
    

    
    
    def _initialize_network_2_GroundStation(self) -> bool:
        """Initialize network communication"""
        try:
            self.vehicle_logger.logger.info("Creating Ground Station client...")
            
            # Create Ground Station client
            self.client_Ground_Station = GroundStationClient(
                config=self.config,
                logger=self.vehicle_logger,
                kill_event=self.kill_event
            )
            time.sleep(0.2)  # Allow client object to settle
            
            # Initialize network connection
            self.vehicle_logger.logger.info("Initializing network connection...")
            if not self.client_Ground_Station.initialize_network():
                return False
            time.sleep(0.5)  # Allow network initialization to complete
            
            # Start network threads
            self.vehicle_logger.logger.info("Starting network threads...")
            if not self.client_Ground_Station.start_threads():
                return False
            time.sleep(0.3)  # Allow threads to start properly
            
            self.vehicle_logger.logger.info("Ground Station communication initialized")
            return True
            
        except Exception as e:
            self.vehicle_logger.log_error("Ground Station initialization failed", e)
            return False
    
    
    def reinitialize_fleet_estimation(self, peer_vehicles: list) -> bool:
        """Reinitialize fleet estimation when V2V is activated - called by V2VManager"""
        try:
            if not self.vehicle_observer:
                return False
                
            # Calculate actual fleet size: peers + this vehicle
            actual_fleet_size = len(peer_vehicles) + 1
            
            self.vehicle_logger.logger.info(f"Reinitializing fleet estimation for {actual_fleet_size} vehicles")
            self.vehicle_observer.reinitialize_fleet_estimation(actual_fleet_size, peer_vehicles)
            return True
            
        except Exception as e:
            self.vehicle_logger.logger.error(f"Fleet estimation reinitialization error: {e}")
            return False
    
    def _handle_v2v_status_change(self, event_type: str, peer_id: int):
        """
        Handle immediate V2V status changes from V2VManager
        This is now just a simple forwarder to existing logging
        """
        try:
            if event_type == 'peer_connected':
                self.vehicle_logger.logger.info(f"Vehicle {peer_id} connected to V2V network")
            elif event_type == 'peer_disconnected':
                self.vehicle_logger.logger.warning(f"Vehicle {peer_id} disconnected from V2V network")
            elif event_type == 'v2v_activated':
                peer_data = peer_id if isinstance(peer_id, dict) else {}
                fleet_size = peer_data.get('fleet_size', 'unknown')
                self.vehicle_logger.logger.info(f"V2V system activated with fleet size: {fleet_size}")
            elif event_type == 'v2v_deactivated':
                self.vehicle_logger.logger.info(f"V2V system deactivated")
            else:
                self.vehicle_logger.logger.debug(f"V2V status change: {event_type}")
                
        except Exception as e:
            self.vehicle_logger.logger.error(f"V2V status change handling error: {e}")
    
    def report_v2v_status_to_gs(self, status_data: dict):
        """Report V2V connection status to Ground Station - public method for V2VManager"""
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
                
                self.vehicle_logger.logger.info(f"V2V status reported to GS: {status_data['status']}")
            else:
                self.vehicle_logger.logger.warning("Cannot report V2V status - no Ground Station connection")
                
        except Exception as e:
            self.vehicle_logger.logger.error(f"Failed to report V2V status to Ground Station: {e}")
    
    
    def _shutdown(self):
        """Shutdown all systems"""
        self.vehicle_logger.logger.info("Shutting down...")
        
        # Use emergency_stop method instead of non-existent SHUTTING_DOWN state
        try:
            self.state_machine.emergency_stop("System shutdown requested")
        except Exception as e:
            self.vehicle_logger.log_error("Error during state machine shutdown", e)
        
        # Stop vehicle hardware
        if self.qcar:
            try:
                self.qcar.write(throttle=0, steering=0)
                self.vehicle_logger.logger.info("Vehicle stopped (throttle=0, steering=0)")
            except Exception as e:
                self.vehicle_logger.log_error("Error stopping vehicle", e)
        
        # if IS_PHYSICAL_QCAR:
        #     # Stop QUARC models controlling hardware (LiDAR, GPS, etc.)
        #     self._stop_quarc_models()
        
        # Close network handler and stop threads
        if self.client_Ground_Station:
            self.client_Ground_Station.close()
        
        # Shutdown V2V Manager (which handles V2V communication internally)
        if hasattr(self, 'v2v_manager'):
            try:
                self.v2v_manager.disable_v2v()
            except Exception as e:
                self.vehicle_logger.logger.error(f"V2V Manager shutdown error: {e}")
        
        # Log final statistics
        self.vehicle_logger.logger.info("="*60)
        self.vehicle_logger.logger.info("Final Statistics:")
        self.vehicle_logger.logger.info(f"Total iterations: {self.loop_counter}")
        self.vehicle_logger.logger.info(f"Total time: {self.elapsed_time():.2f}s")
        
        # Log network statistics
        if self.client_Ground_Station:
            net_stats = self.client_Ground_Station.get_statistics()
            self.vehicle_logger.logger.info(f"Network - Telemetry sent: {net_stats['telemetry_sent']}, Commands received: {net_stats['commands_received']}")
            if net_stats['queue_overflows'] > 0:
                self.vehicle_logger.logger.info(f"Network queue overflows: {net_stats['queue_overflows']}")
        
        perf_stats = self.perf_monitor.get_statistics()
        if 'loop_time' in perf_stats:
            self.vehicle_logger.logger.info(
                f"Average loop frequency: {perf_stats['loop_time']['frequency']:.1f} Hz"
            )
        
        self.vehicle_logger.logger.info("="*60)
        
        # Close logger
        self.vehicle_logger.close()
