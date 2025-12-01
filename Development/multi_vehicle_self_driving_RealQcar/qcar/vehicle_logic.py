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
from Controller.controllers import SpeedController, SteeringController, StateEstimator
from safety import ControlValidator, SensorHealthMonitor, CollisionAvoidance, WatchdogTimer
from Yolo.YoLo import YOLOReceiver, YOLODriveLogic, YOLOManager
from platoon_controller import PlatoonController, PlatoonConfig
from command_handler import CommandHandler
from V2V.v2v_communication import V2VCommunication
from V2V.v2v_manager import V2VManager, V2VBroadcastConfig
from Observer.VehicleObserverSimple import VehicleObserver


class VehicleLogic:
    """Main vehicle controller class"""
    
    def __init__(self, config: VehicleControlConfig, kill_event: Event):
        self.config = config
        self.kill_event = kill_event

        # calibrationPose = [0,2,-np.pi/2]
        
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
        # self.platoon_controller = PlatoonController(platoon_config, self.vehicle_logger)
        
        # Command handler for centralized command processing
        self.command_handler = CommandHandler(self.vehicle_logger, config)
        
        # V2V Communication system - High-performance UDP
        self.v2v_communication = V2VCommunication(
            vehicle_id=config.network.car_id,
            logger=self.vehicle_logger.logger,
            base_port=8000,  # Dedicated V2V port range (8000+ for better separation)
            status_callback=self._handle_v2v_status_change  # Add callback for immediate status reporting
        )
        
        # V2V Manager - High-level V2V logic and message handling
        v2v_config = V2VBroadcastConfig(
            local_state_frequency=20.0,  # 20Hz for local states
            fleet_state_frequency=5.0,   # 5Hz for fleet states
            heartbeat_frequency=1.0      # 1Hz for heartbeats
        )
        self.v2v_manager = V2VManager(
            vehicle_id=config.network.car_id,
            v2v_communication=self.v2v_communication,
            vehicle_observer=None,  # Will be set later
            logger=self.vehicle_logger.logger,
            config=v2v_config
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
        
        # Control state
        self.v_ref = config.speed.v_ref

        # State machine - use simplified version with internal transition logic
        self.state_machine = VehicleStateMachine(self, self.vehicle_logger)
        
        # Timing
        self.start_time = time.time()
        self.loop_counter = 0
        self.telemetry_counter = 0
        
        # Component update rates and timing
        self.controller_rate = config.timing.controller_update_rate
        self.observer_rate = getattr(config.timing, 'observer_rate', 100)
        self.communication_rate = getattr(config.timing, 'communication_rate', 20)
        
        # Timing trackers for different update rates
        self._last_observer_time = 0.0
        self._last_communication_time = 0.0
        self._last_control_time = 0.0
        
        # Vehicle state tracking
        self.current_pos = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.velocity = 0.0
        self.prev_pos = None
        self.prev_time = None
        
        # Initialize Vehicle Observer for local and fleet state estimation
        # Determine fleet size from config or default to 2 vehicles
        fleet_size = getattr(config.network, 'fleet_size', 2)
        initial_pose = getattr(config, 'initial_pose', [0.0, 0.0, 0.0])
        
        self.vehicle_observer = VehicleObserver(
            vehicle_id=config.network.car_id,
            fleet_size=fleet_size,
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
            initial_pose=np.array(initial_pose),
            state_estimator=None  # Will be set later during initialization
        )
        
        # Connect VehicleObserver to V2VManager
        self.v2v_manager.vehicle_observer = self.vehicle_observer
        
        # Sensor data caching (now handled by observer)
        self.sensor_data_cache = {
            'motor_tach': 0.0,
            'gyro_z': 0.0,
            'state_valid': False,
            'timestamp': 0.0
        }
        
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
                
                # 4. Communication Handling (medium frequency)
                if self._should_update_communication(loop_start):
                    self._handle_communication()
                
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
    
    def _should_update_communication(self, current_time: float) -> bool:
        """Check if communication should update based on rate"""
        if current_time - self._last_communication_time >= 1.0 / self.communication_rate:
            self._last_communication_time = current_time
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
                
                # Update sensor cache for compatibility
                sensor_data = self.vehicle_observer.get_sensor_data()
                self.sensor_data_cache.update(sensor_data)
                
        except Exception as e:
            self.vehicle_logger.log_error("Sensor data update error", e)
    

    
    # ===== Observer Update Methods =====
    def _observer_update(self, dt: float):
        """Unified observer update - handles both local and fleet observer internally"""
        try:
            # StateEstimator is now managed directly by VehicleObserver during initialization
            # No need for manual setting here
            
            # Get last steering command for EKF
            last_steering = getattr(self, '_last_steering', 0.0)
            
            # Update observer (internally handles local and fleet timing)
            state_info = self.vehicle_observer.update_observer(
                dt, 
                last_steering
            )
            
            # Update current position and velocity for compatibility
            self.current_pos = [state_info['x'], state_info['y'], state_info['theta']]
            self.velocity = state_info['velocity']
            self.sensor_data_cache['state_valid'] = state_info['state_valid']
            
            
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
    def _handle_communication(self):
        """Handle all communication tasks"""
        try:
            current_time = time.time()
            
            # 1. Send telemetry
            self._send_telemetry()
            
            # 2. Process ground station commands
            self._process_queued_commands()
            
            # 3. Broadcast own state for V2V
            self._broadcast_v2v_state()
            
            # V2V status is now reported only when it changes (event-driven)
            # Removed periodic reporting since TCP/IP ensures reliable delivery
            
        except Exception as e:
            self.vehicle_logger.log_error("Communication handling error", e)
    
    def _send_telemetry(self):
        """Send telemetry data to ground station"""
        try:
            if not hasattr(self, 'vehicle_observer') or self.vehicle_observer is None:
                return
                
            # Get state from VehicleObserver
            state_info = self.vehicle_observer.get_estimated_state_for_control()
            x, y, theta, velocity = state_info['x'], state_info['y'], state_info['theta'], state_info['velocity']
            u = getattr(self, '_last_u', 0.0)
            delta = getattr(self, '_last_steering', 0.0)
            
            self._queue_telemetry(x, y, theta, velocity, u, delta)
            
        except Exception as e:
            self.vehicle_logger.log_error("Telemetry sending error", e)
    
    def _broadcast_v2v_state(self):
        """Broadcast vehicle state to V2V network using V2VManager"""
        try:
            if not hasattr(self, 'v2v_manager') or self.v2v_manager is None:
                return
                
            # V2VManager handles all broadcast logic with different frequencies
            # Local state: 20Hz, Fleet state: 5Hz, Heartbeat: 1Hz
            broadcast_sent = self.v2v_manager.update_broadcast()
            
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
            if hasattr(self, 'v2v_manager') and self.v2v_manager:
                stats = self.v2v_manager.get_statistics()
                comm_stats = self.v2v_communication.get_statistics() if hasattr(self, 'v2v_communication') else {}
                
                self.vehicle_logger.logger.info(f"[V2V] Activity Summary - Broadcasts: Local={stats.get('local_broadcasts', 0)}, Fleet={stats.get('fleet_broadcasts', 0)}")
                self.vehicle_logger.logger.info(f"[V2V] Messages: Sent={comm_stats.get('messages_sent', 0)}, Recv={comm_stats.get('messages_received', 0)}")
                self.vehicle_logger.logger.info(f"[V2V] Peers: {len(self.v2v_communication.get_connected_peers()) if hasattr(self, 'v2v_communication') else 0} connected")
                
        except Exception as e:
            self.vehicle_logger.log_error("V2V activity logging error", e)
    
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
            'v_ref': float(self.v_ref * self.yolo_manager.get_yolo_gain()),
            'yolo_gain': float(self.yolo_manager.get_yolo_gain()),
            'waypoint_index': self.steering_controller.get_waypoint_index() if self.steering_controller else 0,
            'cross_track_error': float(self.steering_controller.get_errors()[0]) if self.steering_controller else 0.0,
            'heading_error': float(self.steering_controller.get_errors()[1]) if self.steering_controller else 0.0,
            'state': self.state_machine.state.name if hasattr(self.state_machine, 'state') and self.state_machine.state else 'UNKNOWN',
            'gps_valid': self.vehicle_observer.is_state_valid() if hasattr(self, 'vehicle_observer') and self.vehicle_observer else False,
            # V2V Manager status in telemetry for GUI updates
            'v2v_active': (hasattr(self, 'v2v_communication') and self.v2v_communication.is_active) if hasattr(self, 'v2v_communication') else False,
            'v2v_peers': len(self.v2v_communication.get_connected_peers()) if (hasattr(self, 'v2v_communication') and self.v2v_communication.is_active) else 0,
            'v2v_protocol': 'UDP-Manager' if (hasattr(self, 'v2v_manager') and self.v2v_manager.is_active()) else 'None',
            'v2v_local_rate': self.v2v_manager.config.local_state_frequency if hasattr(self, 'v2v_manager') else 0.0,
            'v2v_fleet_rate': self.v2v_manager.config.fleet_state_frequency if hasattr(self, 'v2v_manager') else 0.0
            # Platoon telemetry
            # **self.platoon_controller.get_telemetry()
        }
        
        # Log to file
        if self.config.logging.enable_telemetry_logging:
            self.vehicle_logger.log_telemetry(telemetry)
        
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
                    self.vehicle_logger.log_error("Telemetry transmission error", e)
        
        self.telemetry_counter += 1
    
    def _report_v2v_status(self):
        """Report V2V connection status - simplified"""
        try:
            if not hasattr(self, 'v2v_communication') or not self.v2v_communication.is_active:
                # Report inactive status
                self._report_v2v_status_to_gs({
                    'status': 'inactive',
                    'vehicle_id': self.config.network.car_id,
                    'connected_peers': 0,
                    'expected_peers': 0,
                    'peer_list': [],
                    'timestamp': time.time()
                })
                return
            
            v2v_stats = self.v2v_communication.get_statistics()
            connected_peers = self.v2v_communication.get_connected_peers()
            is_fully_connected = self.v2v_communication.is_fully_connected()
            connection_summary = self.v2v_communication.get_connection_summary()
            
            # Calculate expected peers
            expected_peers = len([v for v in self.v2v_communication.peer_vehicles if v != self.config.network.car_id])
            
            # Determine status
            if is_fully_connected and expected_peers > 0:
                status = 'active'  # Use 'active' instead of 'connected' for periodic updates
            elif len(connected_peers) > 0:
                status = 'active'
            else:
                status = 'disconnected'
            
            # Report to Ground Station with UDP performance metrics
            status_data = {
                'status': status,
                'vehicle_id': self.config.network.car_id,
                'connected_peers': len(connected_peers),
                'expected_peers': expected_peers,
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
            
            # Log status occasionally
            if hasattr(self, '_v2v_log_counter'):
                self._v2v_log_counter += 1
            else:
                self._v2v_log_counter = 1
                
            if self._v2v_log_counter % 10 == 0:  # Every 20 seconds (10 * 2 second interval)
                if is_fully_connected:
                    self.vehicle_logger.logger.debug(
                        f"V2V STATUS: {connection_summary} | "
                        f"Sent: {v2v_stats.get('messages_sent', 0)}, Received: {v2v_stats.get('messages_received', 0)}"
                    )
                else:
                    self.vehicle_logger.logger.debug(
                        f"V2V STATUS: {connection_summary} | "
                        f"Connected: {len(connected_peers)}/{expected_peers} | Peers: {connected_peers}"
                    )
                
        except Exception as e:
            self.vehicle_logger.log_error("V2V status reporting error", e)
    
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
    
    
    def activate_v2v(self, peer_vehicles: list, peer_ips: list) -> bool:
        """Activate V2V communication with specified peers"""
        try:
            if not self.v2v_communication or not self.v2v_manager:
                self.vehicle_logger.log_error("V2V communication not initialized")
                return False
            
            self.vehicle_logger.logger.info(f"Activating V2V communication with peers: {peer_vehicles} at IPs: {peer_ips}")
            
            # Calculate actual fleet size: peers + this vehicle
            actual_fleet_size = len(peer_vehicles) + 1
            
            # Reinitialize VehicleObserver fleet estimation with actual fleet information
            if self.vehicle_observer:
                self.vehicle_logger.logger.info(f"Reinitializing fleet estimation for {actual_fleet_size} vehicles")
                self.vehicle_observer.reinitialize_fleet_estimation(actual_fleet_size, peer_vehicles)
            
            # Activate V2V communication layer
            comm_success = self.v2v_communication.activate(peer_vehicles, peer_ips)
            
            if comm_success:
                # Activate V2V manager (which uses the communication layer)
                manager_success = self.v2v_manager.activate(peer_vehicles, peer_ips)
                
                if manager_success:
                    self.vehicle_logger.logger.info(f"V2V communication activated successfully for fleet of {actual_fleet_size} vehicles")
                    total_expected = len([v for v in peer_vehicles if v != self.config.network.car_id])

                    # Report activation status immediately
                    self._report_v2v_status_to_gs({
                        'status': 'activated',
                        'peer_count': len(peer_vehicles),
                        'peer_vehicles': peer_vehicles,                    
                        'expected_peers': total_expected,
                        'peer_list': [],
                        'vehicle_id': self.config.network.car_id,
                        'timestamp': time.time(),
                        'fleet_size': actual_fleet_size,
                        'protocol': 'UDP-Manager'
                    })
                    
                    return True
                else:
                    self.vehicle_logger.log_error("V2V manager activation failed")
                    # Clean up communication if manager failed
                    self.v2v_communication.deactivate()
                    return False
            else:
                self.vehicle_logger.log_error("V2V communication layer activation failed")
                return False
                
        except Exception as e:
            self.vehicle_logger.log_error("V2V activation failed", e)
            return False
    
    def disable_v2v(self):
        """Disable V2V communication"""
        try:
            self.v2v_communication.deactivate()
            self.vehicle_logger.logger.info("V2V communication disabled")
            
            # Report V2V disconnect to Ground Station
            self._report_v2v_status_to_gs({
                'status': 'disconnected',
                'vehicle_id': self.config.network.car_id,
                'timestamp': time.time()
            })
            
        except Exception as e:
            self.vehicle_logger.log_error("V2V disable error", e)
    
    def _handle_v2v_status_change(self, event_type: str, peer_id: int):
        """Handle immediate V2V status changes from the communication module"""
        try:
            connected_peers = len(self.v2v_communication.get_connected_peers())
            total_expected = len([v for v in self.v2v_communication.peer_vehicles if v != self.config.network.car_id])
            
            if event_type == 'connection_established':
                self.vehicle_logger.logger.info(f"V2V: Peer {peer_id} connected ({connected_peers}/{total_expected} peers)")
                
                # Check if we're fully connected
                if connected_peers >= total_expected and total_expected > 0:
                    status = 'connected'
                    self.vehicle_logger.logger.info("V2V: All peers connected - mesh is fully established!")
                else:
                    status = 'active'
                    
            elif event_type == 'connection_lost':
                self.vehicle_logger.logger.info(f"V2V: Peer {peer_id} disconnected ({connected_peers}/{total_expected} peers)")
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
            self.vehicle_logger.log_error(f"Error handling V2V status change: {event_type}", e)
    
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
                
                self.vehicle_logger.logger.info(f"V2V status reported to GS: {status_data['status']}")
            else:
                self.vehicle_logger.logger.warning("Cannot report V2V status - no Ground Station connection")
                
        except Exception as e:
            self.vehicle_logger.log_error("Failed to report V2V status to Ground Station", e)
    
    def _stop_quarc_models(self):
        """Stop QUARC models controlling hardware components"""
        try:
            import subprocess
            import socket
            
            self.vehicle_logger.logger.info("Stopping QUARC models (hardware control)...")
            
            # Get local IP or use localhost for QUARC models running locally
            quarc_target = "tcpip://localhost:17000"
            
            # Try to stop QUARC models using quarc_run command
            cmd = ["quarc_run", "-q", "-Q", "-t", quarc_target, "*.rt-linux_qcar2"]
            
            try:
                result = subprocess.run(cmd, 
                                      capture_output=True, 
                                      text=True, 
                                      timeout=5)
                
                if result.returncode == 0:
                    self.vehicle_logger.logger.info("QUARC models stopped successfully")
                else:
                    self.vehicle_logger.logger.warning(f"QUARC stop returned code {result.returncode}")
                    if result.stderr:
                        self.vehicle_logger.logger.warning(f"QUARC error: {result.stderr.strip()}")
                        
            except subprocess.TimeoutExpired:
                self.vehicle_logger.logger.warning("QUARC stop command timed out")
            except FileNotFoundError:
                self.vehicle_logger.logger.warning("quarc_run not found - QUARC models may still be running")
                self.vehicle_logger.logger.warning("Hardware (LiDAR, GPS) may need manual shutdown")
            except Exception as e:
                self.vehicle_logger.logger.error(f"Error running quarc_run: {e}")
                
            # Test if QUARC is still running
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1)
                result = sock.connect_ex(("localhost", 17000))
                sock.close()
                
                if result == 0:
                    self.vehicle_logger.logger.warning("QUARC service still accessible - hardware may still be active")
                else:
                    self.vehicle_logger.logger.info("QUARC service stopped - hardware should be inactive")
                    
            except Exception as e:
                self.vehicle_logger.logger.debug(f"Cannot test QUARC connection: {e}")
                
        except Exception as e:
            self.vehicle_logger.logger.error(f"Error stopping QUARC models: {e}")
    
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
        
        # Stop QUARC models controlling hardware (LiDAR, GPS, etc.)
        self._stop_quarc_models()
        
        # Close network handler and stop threads
        if self.client_Ground_Station:
            self.client_Ground_Station.close()
        
        # Shutdown V2V Manager and communication
        if hasattr(self, 'v2v_manager'):
            try:
                self.v2v_manager.deactivate()
            except Exception as e:
                self.vehicle_logger.logger.error(f"V2V manager shutdown error: {e}")
        
        if hasattr(self, 'v2v_communication'):
            try:
                self.v2v_communication.deactivate()
            except Exception as e:
                self.vehicle_logger.logger.error(f"V2V communication shutdown error: {e}")
        
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
