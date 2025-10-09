import multiprocessing
import time
import math
import logging
import numpy as np
import os
import sys
import socket
import json
from collections import deque
from typing import Any, Optional, Dict, List
import threading

# QLabs imports - these will be created inside each process
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from pal.utilities.math import wrap_to_pi

from qvl.real_time import QLabsRealTime

# Physical QCar imports (for EKF and GPS mode)
try:
    from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
    from hal.content.qcar_functions import QCarEKF
    PHYSICAL_QCAR_AVAILABLE = True
except ImportError:
    PHYSICAL_QCAR_AVAILABLE = False
    QCar = None
    QCarGPS = None
    QCarEKF = None
    IS_PHYSICAL_QCAR = False

# Controller imports
from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle

# Other imports
from CommHandler import CommHandler
from VehicleLeaderController import VehicleLeaderController
from VehicleFollowerController import VehicleFollowerController
from StateQueue import StateQueue
from VehicleObserver import VehicleObserver
from src.Trust.TriPTrustModel import TriPTrustModel
from GraphBasedTrust import GraphBasedTrustEvaluator
from CamLidarFusion import CamLidarFusion

from src.GPS_sim.md_gps_sync import GPSSync
from md_logging_config import (
    get_individual_vehicle_logger, get_communication_logger, get_gps_logger, 
    get_control_logger, get_observer_logger, get_fleet_observer_logger,
    get_trust_logger, disable_all_logging, set_module_logging
)

# Import performance monitoring
try:
    from performance_monitor import perf_monitor
    PERFORMANCE_MONITORING = True
except ImportError:
    PERFORMANCE_MONITORING = False
    perf_monitor = None

# Import component classes
from src.Component_based.CommunicationComponent import CommunicationComponent
from src.Component_based.ConfigValidator import ConfigValidator
from src.Component_based.ControlComponent import ControlComponent
from src.Component_based.GPSComponent import GPSComponent


def vehicle_process_main(vehicle_config: Dict, stop_event: multiprocessing.Event, 
                        status_queue: multiprocessing.Queue, start_event: multiprocessing.Event):
    """
    Main function that runs in each vehicle process.
    
    Args:
        vehicle_config: Dictionary containing all vehicle configuration
        stop_event: Multiprocessing event to signal process shutdown
        status_queue: Queue for sending status updates to main process
        start_event: Multiprocessing event to signal all vehicles are initialized
    """
    
    # Extract configuration
    vehicle_id = vehicle_config['vehicle_id']
    controller_type = vehicle_config['controller_type']
    is_leader = vehicle_config['is_leader']
    # fleet_size = vehicle_config['fleet_size']
    # initial_pose = vehicle_config['initial_pose']
    
    # # Communication config
    # target_ip = vehicle_config['target_ip']
    # send_port = vehicle_config['send_port']
    # recv_port = vehicle_config['recv_port']
    # ack_port = vehicle_config['ack_port']
    
    # # GPS config
    # gps_server_ip = vehicle_config['gps_server_ip']
    # gps_server_port = vehicle_config['gps_server_port']
    
    # # Control parameters
    # max_steering = vehicle_config.get('max_steering', 0.6)
    # lookahead_distance = vehicle_config.get('lookahead_distance', 7.0)
    # update_rate = vehicle_config.get('update_rate', 100)
    # observer_rate = vehicle_config.get('observer_rate', 100)
    # gps_update_rate = vehicle_config.get('gps_update_rate', 50)
    
    # Vehicle scale and spawn info
    vehicle_scale = vehicle_config['vehicle_scale']
    spawn_location = vehicle_config['spawn_location']
    spawn_rotation = vehicle_config['spawn_rotation']
    
    try:
        # Create QLabs connection inside this process
        qlabs = QuanserInteractiveLabs()
        try:
            qlabs.open("localhost")
            print(f"Vehicle {vehicle_id}: Connected to QLabs")
        except Exception as e:
            print(f"Vehicle {vehicle_id}: Error connecting to QLabs: {e}")
            status_queue.put({'vehicle_id': vehicle_id, 'status': 'failed', 'error': str(e)})
            return
        
        # Create QCar object inside this process
        qcar = QLabsQCar2(qlabs)
        
        # Spawn or connect to the vehicle
        try:
            qcar.spawn_id(
                actorNumber=vehicle_id,
                location=spawn_location,
                rotation=spawn_rotation,
                scale=vehicle_scale
            )
            time.sleep(0.2)  # Allow some time for the vehicle to spawn

            # print(f"Vehicle {vehicle_id}: Spawn success: {success}")
            print(f"Vehicle {vehicle_id}: Spawn parameters - location: {spawn_location}, rotation: {spawn_rotation}, scale: {vehicle_scale}")
            
            # if not success:
            #     print(f"Vehicle {vehicle_id}: Failed to spawn vehicle - spawn_id returned False")
                # Don't return here, continue with the process as the vehicle might still be usable
            
        except Exception as spawn_error:
            print(f"Vehicle {vehicle_id}: Exception during spawn: {spawn_error}")
            # Continue anyway, the vehicle might still work
        
        print(f"Vehicle {vehicle_id}: Successfully spawned at {spawn_location}")
        
        # Create the vehicle instance
        vehicle = VehicleProcess2(
            vehicle_id=vehicle_id,
            qcar=qcar,
            qlabs=qlabs,
            controller_type=controller_type,
            is_leader=is_leader,
            vehicle_config=vehicle_config,
            stop_event=stop_event,
            status_queue=status_queue
        )
        
        # Signal successful initialization
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'initialized'})
        
        # Wait for all vehicles to initialize before starting the run loop
        print(f"Vehicle {vehicle_id}: Waiting for fleet initialization...")
        start_event.wait()  # Blocks until main process sets the event (after all vehicles are initialized)
        print(f"Vehicle {vehicle_id}: Fleet initialized, starting run loop")
        
        # Run the vehicle with emergency stopping
        try:
            vehicle.run()
        except KeyboardInterrupt:
            print(f"Vehicle {vehicle_id}: KeyboardInterrupt caught, stopping immediately")
        except Exception as e:
            print(f"Vehicle {vehicle_id}: Unexpected error during run: {e}")
        finally:
            # EMERGENCY STOP: Ensure vehicle is stopped before process exits
            try:
                if hasattr(vehicle, 'qcar') and vehicle.qcar is not None:
                    vehicle.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0 ,headlights=False,
                            leftTurnSignal=False,
                            rightTurnSignal=False,
                            brakeSignal=False,
                            reverseSignal=False
                    )
                    print(f"Vehicle {vehicle_id}: Emergency stop - Virtual QCar stopped")
                if hasattr(vehicle, 'physical_qcar') and vehicle.physical_qcar is not None:
                    vehicle.physical_qcar.write(0, 0)
                    print(f"Vehicle {vehicle_id}: Emergency stop - Physical QCar stopped")
            except Exception as stop_error:
                print(f"Vehicle {vehicle_id}: Error during emergency stop: {stop_error}")
        
        # After run() exits, call stop to clean up
        vehicle.stop()
        
    except Exception as e:
        print(f"Vehicle {vehicle_id}: Process error: {e}")
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'error', 'error': str(e)})
    finally:
        print(f"Vehicle {vehicle_id}: Process shutting down")
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'shutdown'})


class VehicleProcess2:
    """
    Vehicle class that runs in its own process and manages its own control logic.
    Each process creates its own QLabs connection and QCar object.
    """
    
    def __init__(self, vehicle_id: int, qcar: QLabsQCar2, qlabs: QuanserInteractiveLabs,
                 controller_type: str = "CACC", is_leader: bool = False, 
                 vehicle_config: Dict = None, stop_event: multiprocessing.Event = None,
                 status_queue: multiprocessing.Queue = None):
        """
        Initialize a vehicle process instance.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            qcar: QLabsQCar2 instance for this vehicle (created in this process)
            qlabs: QuanserInteractiveLabs instance (created in this process)
            controller_type: Type of controller ("CACC" or "IDM")
            is_leader: Whether this vehicle is the leader
            vehicle_config: Configuration dictionary
            stop_event: Multiprocessing event for shutdown signaling
            status_queue: Queue for sending status updates
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.qlabs = qlabs
        self.controller_type = controller_type
        self.is_leader = is_leader
        self.config = vehicle_config
        self.stop_event = stop_event
        self.status_queue = status_queue

        # Multiprocessing-safe running flag
        self.running = multiprocessing.Event()
        self.running.set()

        # Initialize Vehicle Postion
        spawn_location = vehicle_config.get('spawn_location', [0, 0, 0])
        spawn_rotation = vehicle_config.get('spawn_rotation', [0, 0, 0])
        initial_pose = np.array([spawn_location[0], spawn_location[1], spawn_rotation[2]])
        
        # Control parameters (needed early for physical QCar initialization)
        self.max_steering = vehicle_config.get('max_steering', 0.6)
        self.lookahead_distance = vehicle_config.get('lookahead_distance', 7.0)
        self.k_steering = 2.0
        self.update_rate = vehicle_config.get('general_update_rate', 100)
        self.controller_rate = vehicle_config.get('controller_rate', 100)
        self.observer_rate = vehicle_config.get('observer_rate', 100)
        self.gps_update_rate = vehicle_config.get('gps_update_rate', 5)

        print(f"Vehicle {self.vehicle_id}: Update rate: {vehicle_config.get('general_update_rate')}, Control rate: {vehicle_config.get('controller_rate')}, Observer rate: {vehicle_config.get('observer_rate')}, GPS update rate: {self.gps_update_rate}")

        # QCar mode configuration - check if we should use physical QCar API for leader
        # TODO : instead of use is_leader , depend on config file if we need to use that 
        self.use_control_observer_mode = vehicle_config.get('use_control_observer_mode', False) 

        self.use_physical_qcar = vehicle_config.get('use_physical_qcar', False) and is_leader and PHYSICAL_QCAR_AVAILABLE
        print(f"Vehicle {vehicle_id}: use_physical_qcar={self.use_physical_qcar}, PHYSICAL_QCAR_AVAILABLE={PHYSICAL_QCAR_AVAILABLE}, is_leader={is_leader}")
        self.enable_steering_control = vehicle_config.get('enable_steering_control', True)
        self.calibrate = vehicle_config.get('calibrate', False)
        
        # Initialize QCar interfaces based on mode
        self.physical_qcar = None
        self.gps = None
        self.ekf = None

        if self.use_physical_qcar and not self.use_control_observer_mode:
            try:
                self.rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
                QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=vehicle_id)

                # Initialize physical QCar interface for leader
                self.physical_qcar = QCar(readMode=1, frequency=self.controller_rate)
                self.gps = QCarGPS(initialPose=initial_pose, calibrate=self.calibrate)
                # # Initialize GPS for EKF if steering control is enabled
                if self.enable_steering_control:
                    try:
                        self.gps = QCarGPS(readMode=1, frequency=self.gps_update_rate)
                        # Initialize EKF for state estimation

                        print(f"Vehicle {self.vehicle_id}: Physical QCar with GPS and EKF initialized")
                    except Exception as gps_error:
                        print(f"Vehicle {self.vehicle_id}: GPS initialization failed: {gps_error}")
                        self.gps = None
                        self.ekf = None
                        
                else:
                    self.gps = None
                   
                    print(f"Vehicle {self.vehicle_id}: Physical QCar initialized without GPS/EKF")
                    
            except Exception as e:
                print(f"Vehicle {self.vehicle_id}: Failed to initialize physical QCar: {e}")
                self.use_physical_qcar = False
                self.physical_qcar = None
                # self.gps = None
                # self.ekf = None
                # self.ekf_initialized = False

        

        
        # Extract configuration values
        self.fleet_size = vehicle_config.get('fleet_size', 3)
        self.target_ip = vehicle_config['target_ip']
        self.send_port = vehicle_config['send_port']
        self.recv_port = vehicle_config['recv_port']
        self.ack_port = vehicle_config['ack_port']
        
        # NEW: Extract fleet graph configuration from vehicle_config
        self.fleet_graph = vehicle_config.get('fleet_graph', [])  # Full adjacency matrix
        self.connected_vehicles = vehicle_config.get('connected_vehicles', [])  # List of connected vehicle IDs
        self.num_connections = vehicle_config.get('num_connections', 0)  # Number of connections
        
        print(f"Vehicle {self.vehicle_id}: Fleet graph connections: {self.connected_vehicles}")
        print(f"Vehicle {self.vehicle_id}: Number of connections: {self.num_connections}")

        # NEW: Chain-following configuration
        self.following_target = vehicle_config.get('following_target', None)
        if self.following_target is not None:
            print(f"Vehicle {self.vehicle_id}: Configured to follow vehicle {self.following_target}")
        else:
            print(f"Vehicle {self.vehicle_id}: Leader vehicle (no following target)")
        
        # Vehicle state - initialize with spawn location and rotation
        spawn_location = vehicle_config.get('spawn_location', [0, 0, 0])
        spawn_rotation = vehicle_config.get('spawn_rotation', [0, 0, 0])
        self.current_pos = list(spawn_location)
        self.current_rot = list(spawn_rotation)
        self.velocity = 0.0
        self.prev_pos = None
        self.prev_time = None
        
        # Initialize logging for this process
        self.logger = get_individual_vehicle_logger(vehicle_id)
        self.comm_logger = get_communication_logger(vehicle_id)
        self.gps_logger = get_gps_logger(vehicle_id)
        self.control_logger = get_control_logger(vehicle_id)
        self.observer_logger = get_observer_logger(vehicle_id)
        self.fleet_observer_logger = get_fleet_observer_logger(vehicle_id)
        self.trust_logger = get_trust_logger(vehicle_id)
        
        # Configure logging for performance
        self.configure_logging_for_performance()
        
        # GPS Time Synchronization
        gps_server_ip = vehicle_config.get('gps_server_ip', '127.0.0.1')
        gps_server_port = vehicle_config.get('gps_server_port', 8001)
        self.gps_sync = GPSSync(gps_server_ip, gps_server_port, vehicle_id)
        self.sync_interval = 5.0
        self.last_sync_attempt = time.time()
        
        # Local simulation (high rate, low jitter):

        # broadcast_rate ≈ 100 Hz
        # max_age_seconds: 0.35–0.40
        # max_delay_threshold: 0.12
        # max_queue_size: 64 ( (100 * 0.4) *1.2 = 48 → choose power-of-two 64)
        # Physical lab LAN / moderate jitter (~40–60 Hz effective):

        # max_age_seconds: 0.5
        # max_delay_threshold: 0.18–0.20
        # max_queue_size: 64–80 (choose 80 if not power-of-two constraint)
        # Higher latency / experimental:

        # max_age_seconds: 0.8–1.0
        # max_delay_threshold: 0.30–0.40
        # max_queue_size: 100–120 (if rate ~100 Hz) or 60 (if rate ~60 Hz)
        
        # State Queue for managing received states
        self.state_queue = StateQueue(
            max_queue_size=64,
            max_age_seconds=0.5,  # Increased from 0.2s for local network reliability
            max_delay_threshold=0.4,
            logger=self.logger
        )
        # World transform timeout & metrics (configurable)
        self.world_tf_timeout = vehicle_config.get('world_tf_timeout', 0.05)  # seconds
        self.world_tf_timeouts = 0
        self.world_tf_errors = 0
        self.world_tf_last_warning = 0.0
        self._prev_time_monotonic = None  # For velocity dt stability

        # Initialize communication handler
        # Use non-blocking mode for process-based vehicles with bidirectional support
        try:
            # Get peer communication configuration for bidirectional communication
            peer_ports = vehicle_config.get('peer_ports', {})
            communication_mode = vehicle_config.get('communication_mode', 'unidirectional')

            self.comm = CommHandler(
                vehicle_id=self.vehicle_id,
                target_ip=self.target_ip,
                send_port=self.send_port,
                recv_port=self.recv_port,
                ack_port=self.ack_port,
                logger=self.comm_logger,
                mode='non_blocking',
                peer_ports=peer_ports,
                communication_mode=communication_mode,
                topology=vehicle_config.get('fleet_graph', {}).get('type', 'fully_connected'),
                fleet_size=self.fleet_size,
            )
            print(f"Vehicle {self.vehicle_id}: Communication initialized in {communication_mode} mode")
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Communication initialization failed: {e}")
            self.comm = None

        # Initialize communication in non-blocking mode
        if self.comm is not None:
            self.comm.initialize_sockets()
            print(f"Vehicle {self.vehicle_id}: Communication sockets initialized")
        else:
            print(f"Vehicle {self.vehicle_id}: Warning - Communication not available")

        # Control components based on vehicle role
        try:
            if self.is_leader:
                self.leader_controller = VehicleLeaderController(
                    vehicle_id=self.vehicle_id,
                    Qcar_RT=self.physical_qcar,
                    config=self.config,
                    logger=self.control_logger
                )
                self.follower_controller = None
                print(f"Vehicle {self.vehicle_id}: Leader controller initialized")
            else:
                self.follower_controller = VehicleFollowerController(
                    vehicle_id=self.vehicle_id,
                    controller_type=self.controller_type,
                    config=self.config,
                    logger=self.control_logger
                )
                self.leader_controller = None
                print(f"Vehicle {self.vehicle_id}: Follower controller ({self.controller_type}) initialized")
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Controller initialization failed: {e}")
            self.logger.error(f"Vehicle {self.vehicle_id}: Controller initialization failed: {e}")
            self.leader_controller = None
            self.follower_controller = None
            # Don't stop the event here, let the vehicle continue with basic operation


        try:
            self.observer = VehicleObserver(
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                config=self.config,  # Pass proper config
                logger=self.logger,
                initial_pose=initial_pose
            )
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Observer initialization failed: {e}")
            self.observer = None

        # Initialize control input tracking
        self.current_control_input = np.array([0.0, 0.0])

        # Sensor data tracking
        self.last_gyroscope_z = 0.0
        self.last_motor_tach = 0.0

        self.gps_data_cache = {
            'position': None,
            'rotation': None,
            'velocity': 0.0,
            'available': False,
            'timestamp': 0.0,
            'last_update': 0.0
        }

        # Leader tracking for followers
        self.leader_vehicle = None
        self.leader_state = None

        # Initialize Graph-Based Trust Model for connected vehicles only
        self.trust_enabled = vehicle_config.get('enable_trust_evaluation', True)
        
        if self.trust_enabled and len(self.connected_vehicles) > 0:
            # Create trust models only for connected vehicles (based on fleet graph)
            trust_models = {}
            for target_vehicle_id in self.connected_vehicles:
                if target_vehicle_id != self.vehicle_id:  # Don't create trust model for self
                    trust_models[target_vehicle_id] = TriPTrustModel()
                    # print(f"Vehicle {self.vehicle_id}: Created trust model for connected vehicle {target_vehicle_id}")
            
            # Create simplified graph-based trust evaluator
            self.trust_evaluator = GraphBasedTrustEvaluator(
                vehicle_id=self.vehicle_id,
                connected_vehicles=self.connected_vehicles,
                trust_models=trust_models
            )
            
            print(f"Vehicle {self.vehicle_id}: Graph-based trust evaluation enabled for {len(trust_models)} connected vehicles")
            
            # Log trust initialization details
            self.trust_logger.info(f"TRUST_INIT: Graph-based trust evaluation initialized - "
                                 f"connected_vehicles={self.connected_vehicles}, "
                                 f"trust_models_count={len(trust_models)}")
        else:
            self.trust_evaluator = None
            print(f"Vehicle {self.vehicle_id}: Trust evaluation disabled or no connected vehicles")
            self.trust_logger.info(f"TRUST_INIT: Trust evaluation disabled - "
                                 f"trust_enabled={self.trust_enabled}, "
                                 f"connected_vehicles_count={len(self.connected_vehicles)}")
        
        # Initialize components
        self._initialize_components(vehicle_config)
        
        # Initialize CamLidarFusion thread (only for vehicles with following targets)
        self.cam_lidar_thread = None
        self.cam_lidar_thread_running = threading.Event()
        if self.following_target is not None and hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None:
            self.cam_lidar_thread_running.set()
            self.cam_lidar_thread = threading.Thread(
                target=self._cam_lidar_fusion_thread,
                name=f"CamLidarFusion-V{self.vehicle_id}",
                daemon=True
            )
            self.cam_lidar_thread.start()
            self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion thread started")
        
        print(f"Vehicle {self.vehicle_id} process initialized successfully")
        self.logger.info(f"Vehicle {self.vehicle_id} process initialized successfully")

    def _initialize_components(self, vehicle_config: Dict[str, Any]):
        """Initialize the component-based architecture."""
        try:
            # # Validate configuration
            # validator = ConfigValidator(self.logger)
            # if not validator.validate_vehicle_config(vehicle_config, self.vehicle_id):
            #     self.logger.error(f"Vehicle {self.vehicle_id}: Configuration validation failed")
            #     # Continue anyway, but log the issues

            # Initialize GPS Component
            self.gps_component = GPSComponent(
                vehicle_id=self.vehicle_id,
                qcar=self.qcar,
                qlabs=self.qlabs,
                gps_sync=self.gps_sync,
                logger=self.gps_logger,
                use_physical_qcar=self.use_physical_qcar,
                physical_qcar=self.physical_qcar,
                gps=getattr(self, 'gps', None),
                enable_steering_control=self.enable_steering_control
            )

            # Initialize Control Component
            self.control_component = ControlComponent(
                vehicle_id=self.vehicle_id,
                is_leader=self.is_leader,
                config=vehicle_config,
                leader_controller=self.leader_controller,
                follower_controller=self.follower_controller,
                gps_component=self.gps_component,
                state_queue=self.state_queue,
                gps_sync=self.gps_sync,
                qcar=self.qcar,
                physical_qcar=self.physical_qcar,
                logger=self.control_logger
            )

            # Initialize Communication Component
            self.comm_component = CommunicationComponent(
                vehicle_id=self.vehicle_id,
                config=vehicle_config,
                comm_handler=self.comm,
                observer=self.observer,
                state_queue=self.state_queue,
                gps_sync=self.gps_sync,
                trust_evaluator=self.trust_evaluator,
                cam_lidar_fusion=self.cam_lidar_fusion,
                logger=self.comm_logger
            )
            
            # Initialize communication sockets
            if self.comm_component.initialize():
                self.logger.info(f"Vehicle {self.vehicle_id}: Communication component initialized successfully")
            else:
                self.logger.warning(f"Vehicle {self.vehicle_id}: Communication component initialization failed")
            
            # Initialize CamLidarFusion Component (only for vehicles with following targets)
            if self.following_target is not None:
                try:
                    self.cam_lidar_fusion = CamLidarFusion(
                        qcar=self.qcar,
                        vehicle_id=self.vehicle_id,
                        image_width=640,
                        image_height=480,
                        scale_factor=0.1,
                        verbose=False,
                        logger=self.logger
                    )
                    self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion component initialized for following vehicle {self.following_target}")
                except Exception as e:
                    self.logger.warning(f"Vehicle {self.vehicle_id}: CamLidarFusion initialization failed: {e}")
                    self.cam_lidar_fusion = None
            else:
                self.cam_lidar_fusion = None
                self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion not initialized (no following target)")
            
            # Set up cross-references between components
            self.control_component.comm_component = self.comm_component
            self.comm_component.control_component = self.control_component

            self.logger.info(f"Vehicle {self.vehicle_id}: All Components initialized successfully")

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Component initialization failed: {e}")
            # Set components to None to prevent crashes
            self.gps_component = None
            self.control_component = None
            self.comm_component = None

    def configure_logging_for_performance(self):
        """Configure logging settings for optimal performance."""
        disable_all_logging()
        set_module_logging('observer', True)
        set_module_logging('communication', True)
        set_module_logging('fleet_observer', True)  # Enable fleet observer logging
        set_module_logging('control', True)  # Enable trust logging
        set_module_logging('debug', True)  # Enable debug level logs
        
        # # Set fleet observer logger to INFO level to see the velocity debug messages
        self.fleet_observer_logger.setLevel(logging.INFO)
        self.comm_logger.setLevel(logging.INFO)  # Already set to INFO - this enables info messages
        self.observer_logger.setLevel(logging.INFO)
        # self.trust_logger.setLevel(logging.INFO)  # Enable trust logging at INFO level
    
    
    # --------------------- Communication Methods ---------------------

    def run(self):
        """Main run loop for the vehicle process."""
        self.logger.info(f"Vehicle {self.vehicle_id}: Starting main run loop")
        
        # Initialize timing
        general_update_dt = 1.0 / self.update_rate
        control_dt = 1.0 / self.controller_rate
        observer_dt = 1.0 / self.observer_rate
        gps_dt = 1.0 / self.gps_update_rate
        
        last_general_update_time = time.time()
        last_control_time = time.time()
        last_observer_time = time.time()
        last_gps_time = time.time()
        last_status_time = time.time()
        last_send_communication_time = time.time()
        last_trust_summary_time = time.time()  # For periodic trust logging
        

        
        try:
            while self.running.is_set() and not self.stop_event.is_set():
                current_time = time.time()
                # print(f"Vehicle {self.vehicle_id}: Main loop iteration at {current_time:.3f}")
                
                # Handle communication FIRST to ensure we have latest state data
                if self.comm_component:
                    self.comm_component.handle_communication()
                else:
                    self.logger.warning(f"Vehicle {self.vehicle_id}: Communication component not available")

                # GPS data collection
                if current_time - last_gps_time >= gps_dt:
                    if self.gps_component:
                        self.gps_component.update_gps_data()
                    else:
                        self.logger.warning(f"Vehicle {self.vehicle_id}: GPS component not available")
                    last_gps_time = current_time
                
                # Observer update
                if current_time - last_observer_time >= observer_dt:
                    self.observer_update()
                    last_observer_time = current_time
                
                # Control loop
                if current_time - last_control_time >= control_dt:
                    if self.control_component:
                        forward_speed, steering_angle = self.control_component.update_control()
                        self.control_component.apply_control_commands(forward_speed, steering_angle)
                        # Update current control input for observer
                        self.current_control_input = self.control_component.get_current_control_input()
                    else:
                        self.logger.warning(f"Vehicle {self.vehicle_id}: Control component not available")
                    last_control_time = current_time
                
                # # Log StateQueue stats periodically for debugging
                # if current_time - last_status_time >= 2.0:  # Every 2 seconds
                #     queue_stats = self.state_queue.get_queue_stats()
                #     self.comm_logger.info(f"Vehicle {self.vehicle_id}: StateQueue stats - size: {queue_stats['current_queue_size']}, "
                #                         f"total_received: {queue_stats['total_received']}, valid: {queue_stats['valid_states']}, "
                #                         f"acceptance_rate: {queue_stats['acceptance_rate']:.1f}%")
                
                # Send status update periodically
                if current_time - last_status_time >= 5.0:
                    self.send_status_update()
                    last_status_time = current_time
                
                # # Log trust summary periodically (every 10 seconds)
                # if current_time - last_trust_summary_time >= 10.0:
                #     if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
                #         self.trust_evaluator.log_trust_summary(self.trust_logger)
                #         # Also log trust statistics
                #         stats = self.trust_evaluator.get_trust_statistics()
                #         if stats['count'] > 0:
                #             self.trust_logger.info(f"TRUST_STATS: count={stats['count']}, "
                #                                  f"mean={stats['mean']:.3f}, min={stats['min']:.3f}, "
                #                                  f"max={stats['max']:.3f}")
                #     last_trust_summary_time = current_time
                
                # Small sleep to prevent excessive CPU usage
                time.sleep(0.001)  # 1ms
                
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Main loop error: {e}")
            # self.logger.error(f"Vehicle {self.vehicle_id}: Main loop error: {e}")
        finally:
            self.cleanup()



    def _cam_lidar_fusion_thread(self):
        """Thread for running CamLidarFusion updates independently of main I/O loop."""
        update_dt = 1.0 / self.update_rate  # Same rate as general updates
        
        self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion thread started with {self.update_rate}Hz update rate")
        
        while self.cam_lidar_thread_running.is_set() and not self.stop_event.is_set():
            try:
                start_time = time.time()
                
                # Update CamLidarFusion
                if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None:
                    self.cam_lidar_fusion.update_sensor_data()
                
                # Calculate sleep time to maintain desired update rate
                elapsed_time = time.time() - start_time
                sleep_time = max(0.001, update_dt - elapsed_time)  # Minimum 1ms sleep
                
                time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Vehicle {self.vehicle_id}: CamLidarFusion thread error: {e}")
                time.sleep(0.1)  # Brief pause on error
        
        self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion thread stopped")


    def observer_update(self):
        """
        Update observer with current GPS data, sensor data, and control inputs.
        Similar to Vehicle.py observer_loop but simplified for process context.
        """
        if self.observer is None:
            return
            
        try:
            current_time = self.gps_sync.get_synced_time()
            
            # Get GPS data
            gps_data = self.gps_component.get_cached_gps_data() if self.gps_component else {'available': False, 'position': [0, 0, 0], 'rotation': [0, 0, 0], 'velocity': 0.0}
            measured_state = None
            if gps_data['available']:
                # Convert GPS data to numpy array format expected by observer
                # Observer expects [x, y, theta, v] format
                measured_state = np.array([
                    gps_data['position'][0],  # x
                    gps_data['position'][1],  # y
                    gps_data['rotation'][2],  # theta (yaw angle)
                    gps_data['velocity']      # velocity
                ])
            # TODO : Should use gyroscope data if we use physic RT model 
            if self.physical_qcar is not None and self.vehicle_id == 0:
                motor_tach = self.physical_qcar.motorTach
                gyroscope_z = self.physical_qcar.gyroscope[2]
            else:
                motor_tach = self.velocity  # Fallback to velocity estimate
                gyroscope_z = 0  # Always numeric to satisfy logging formatting
            
            # -------- Update observer local state
            estimated_state = self.observer.update_local_state(
                measured_state=measured_state,
                control_input=self.current_control_input,
                timestamp=current_time,
                motor_tach=motor_tach,
                gyroscope_z=gyroscope_z
            )

            # -------- Update distributed observer with received states from other vehicles
            try:
                # Get distributed fleet state estimates
                fleet_states = self.observer.update_distributed_estimates(self.current_control_input ,estimated_state, current_time )
                
                # Log distributed observer status
                if fleet_states is not None:
                    self.observer_logger.debug(f"Vehicle {self.vehicle_id}: Distributed observer updated - "
                                             f"Fleet size: {fleet_states.shape[1]}, "
                                             f"State dim: {fleet_states.shape[0]}")
                    
                    # Update fleet state estimates cache using helper
                    for vehicle_idx in range(fleet_states.shape[1]):
                        vehicle_state = fleet_states[:, vehicle_idx]
                        # self._set_fleet_vehicle_estimate(vehicle_idx, vehicle_state, current_time)  # Removed - now handled by CommunicationComponent
                        if vehicle_idx != self.vehicle_id:
                            self.observer_logger.debug(
                                f"Vehicle {self.vehicle_id}: Estimated vehicle {vehicle_idx} - "
                                f"pos=({vehicle_state[0]:.3f}, {vehicle_state[1]:.3f}), vel={vehicle_state[3]:.3f}")
                
            except Exception as dist_error:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Distributed observer error: {dist_error}")
            
            # Use the returned estimated_state directly instead of making redundant call
            if estimated_state is not None and len(estimated_state) >= 4:
                # Convert numpy array [x, y, theta, v] to our cache format
                # Debug log to verify observer update
                self.observer_logger.debug(f"Observer updated: pos=({estimated_state[0]:.3f}, {estimated_state[1]:.3f}), "
                                         f"vel={estimated_state[3]:.3f}, "
                                         f"ekf_init={getattr(self.observer, 'ekf_initialized', False)}")
                
                # NEW: Broadcast this vehicle's updated state to all other vehicles
                # This is the logical place since observer maintains the most accurate state estimate
                if self.comm_component:
                    current_state = self.get_best_available_state()
                    self.comm_component.broadcast_own_state(current_state)
                
                # NEW: Broadcast fleet estimates if distributed observer is working
                if self.comm_component and hasattr(self.comm_component, 'fleet_state_estimates') and len(self.comm_component.fleet_state_estimates) > 1:
                    self.comm_component.broadcast_fleet_estimates()
                
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Invalid estimated state from observer: {estimated_state}")
                
        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Observer update error: {e}")

    # --------------------- Trust Evaluation Methods ---------------------
    # --------------------- Helper / Refactor Methods ---------------------
    #region Helper Methods

    def get_observer_state_direct(self) -> Optional[dict]:
        """
        Get state directly from observer without caching.
        
        Returns:
            Observer state dictionary with essential fields only: position, rotation, velocity
        """
        if self.observer is None:
            return None
            
        try:
            # Get the local state directly from observer (more efficient)
            local_state = self.observer.get_local_state()
            if local_state is not None and len(local_state) >= 4:
                return {
                    'position': [local_state[0], local_state[1], 0.0],  # [x, y, z] format
                    'rotation': [0.0, 0.0, local_state[2]],  # [roll, pitch, yaw] format
                    'velocity': local_state[3]
                }
            else:
                # Fallback to the formatted method if local state is not available
                return self.observer.get_estimated_state_for_control()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Failed to get observer state: {e}")
            return None

    def get_best_available_state(self) -> dict:
        """
        Factory method that returns the best available state estimate.
        Priority: Observer EKF > GPS Component > Raw GPS fallback
        
        Returns:
            Best available state estimate with essential fields: position, rotation, velocity
        """
        # Try observer first (check if EKF is initialized through observer attributes)
        if self.observer is not None:
            observer_state = self.get_observer_state_direct()
            if observer_state and hasattr(self.observer, 'ekf_initialized') and self.observer.ekf_initialized:
                return observer_state
        
        # Try GPS component next
        if self.gps_component:
            return self.gps_component.get_current_state()
        
        # Fallback to raw GPS if observer not available or EKF not initialized
        return {
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity
        }
        
    def get_state_for_control(self) -> dict:
        """Get current vehicle state for control algorithms using observer estimates."""
        return self.get_best_available_state()


    def get_relative_distances(self) -> Dict[str, Dict[str, float]]:
        """
        Get relative distance measurements from camera-LiDAR fusion.
        Only available for vehicles with following targets.
        
        Returns:
            Dictionary of detected objects with relative distance information,
            or empty dict if CamLidarFusion not available (vehicle not following)
        """
        if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None:
            return self.cam_lidar_fusion.get_relative_distances()
        else:
            # Return empty dict for vehicles not using following
            return {}
    
    def get_vehicle_relative_distances(self) -> List[Dict[str, float]]:
        """
        Get relative distances specifically to detected vehicles.
        Only available for vehicles with following targets.
        
        Returns:
            List of vehicle detections with distance information,
            or empty list if CamLidarFusion not available (vehicle not following)
        """
        if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None:
            return self.cam_lidar_fusion.get_vehicle_distances()
        else:
            # Return empty list for vehicles not using following
            return []

    # def get_interpolated_leader_state(self, target_time: Optional[float] = None) -> Optional[dict]:
    #     """Get interpolated leader state from state queue."""
    #     if target_time is None:
    #         target_time = time.time()
        
    #     # Add debug info about queue state
    #     queue_stats = self.state_queue.get_queue_stats()
    #     self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Queue stats - size: {queue_stats['current_queue_size']}, "
    #           f"total_received: {queue_stats['total_received']}, valid: {queue_stats['valid_states']}")
        
    #     # Get all states to see what's in the queue (pass GPS time for validation)
    #     current_gps_time = self.gps_sync.get_synced_time()
    #     all_states = self.state_queue.get_all_states(sender_id=0, current_gps_time=current_gps_time)  # Leader is vehicle 0
    #     self.comm_logger.debug(f"Vehicle {self.vehicle_id}: States from leader in queue: {len(all_states)}")
        
    #     result = self.state_queue.get_interpolated_state(target_time, sender_id=0, current_gps_time=current_gps_time)
    #     self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Interpolated result: {result}")
    #     return result

    def get_interpolated_target_state(self, target_time: Optional[float] = None) -> Optional[dict]:
        """
        NEW: Get interpolated state from the target vehicle this one should follow.
        For chain-following: Vehicle 1 follows 0, Vehicle 2 follows 1, Vehicle 3 follows 2, etc.
        """
        if target_time is None:
            target_time = time.time()
        
        # Determine which vehicle to get state from
        if self.following_target is None:
            # This is a leader, no target to follow
            return None
        
        target_vehicle_id = self.following_target
        
        # # Add debug info about queue state for the target vehicle
        # queue_stats = self.state_queue.get_queue_stats()
        # self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Looking for target vehicle {target_vehicle_id} - Queue stats - size: {queue_stats['current_queue_size']}")
        
        # # Get all states from the target vehicle (pass GPS time for validation)
        current_gps_time = self.gps_sync.get_synced_time()
        # all_states = self.state_queue.get_all_states(sender_id=target_vehicle_id, current_gps_time=current_gps_time)
        # self.comm_logger.debug(f"Vehicle {self.vehicle_id}: States from target vehicle {target_vehicle_id} in queue: {len(all_states)}")
        
        # Get interpolated state from the target vehicle (pass GPS time for validation)
        result = self.state_queue.get_interpolated_state(target_time, sender_id=target_vehicle_id, current_gps_time=current_gps_time)
        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Interpolated result from vehicle {target_vehicle_id}: {result}")
        return result
    # ---------------------------------------------------------------------
    #endregion Helper Methods

    def send_status_update(self):
        """Send status update to main process."""
        try:
            status = {
                'vehicle_id': self.vehicle_id,
                'status': 'running',
                # 'position': self.current_pos,
                # 'velocity': self.velocity,
                'timestamp': time.time()
            }
            self.status_queue.put(status)
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Status update error: {e}")

    def cleanup(self):
        """Clean up resources before shutting down."""
        try:
            self.logger.info(f"Vehicle {self.vehicle_id}: Cleaning up resources")
            
            # CRITICAL: Stop the vehicle immediately with zero commands
            try:
                if self.use_control_observer_mode and self.vehicle_id == 0 :
                    print(f"Vehicle {self.vehicle_id}: Emergency stop - Physical QCar zero commands sent (control observer mode)")
                elif self.use_physical_qcar and self.physical_qcar is not None:
                    # Stop physical QCar immediately
                    self.physical_qcar.write(0, 0)
                    print(f"Vehicle {self.vehicle_id}: Emergency stop - Physical QCar zero commands sent")
                else:
                    # Stop virtual QCar immediately
                    self.qcar.set_velocity_and_request_state(
                        forward=0.0, 
                        turn=0.0,
                        headlights=False,
                        leftTurnSignal=False,
                        rightTurnSignal=False,
                        brakeSignal=False,
                        reverseSignal=False
                    )
                    print(f"Vehicle {self.vehicle_id}: Emergency stop - Virtual QCar zero commands sent")
            except Exception as e:
                print(f"Vehicle {self.vehicle_id}: Error during emergency stop: {e}")
            
            # Clean up communication
            if hasattr(self, 'comm') and self.comm is not None:
                self.comm.cleanup()
            
            # Clean up CamLidarFusion thread
            if hasattr(self, 'cam_lidar_thread_running') and self.cam_lidar_thread_running is not None:
                self.cam_lidar_thread_running.clear()
            if hasattr(self, 'cam_lidar_thread') and self.cam_lidar_thread is not None:
                self.cam_lidar_thread.join(timeout=1.0)  # Wait up to 1 second for thread to finish
                self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion thread joined")
            
            # Clean up components
            if self.comm_component:
                self.comm_component.cleanup()
            if self.gps_component:
                self.gps_component.cleanup()
            if self.control_component:
                self.control_component.cleanup()
            
            # Clean up controllers safely
            if self.leader_controller is not None:
                self.leader_controller.stop_control()
            if self.follower_controller is not None:
                self.follower_controller.stop_control()

            # # Clean up GPS sync
            # if hasattr(self, 'gps_sync') and self.gps_sync is not None:
            #     self.gps_sync.cleanup()
                
            # Clean up physical QCar resources
            if self.use_physical_qcar:
                if self.physical_qcar is not None:
                    # Close physical QCar connection
                    try:
                        if hasattr(self.physical_qcar, 'close'):
                            self.physical_qcar.close()
                    except:
                        pass  # Ignore errors during cleanup
                if self.gps is not None and hasattr(self.gps, 'close'):
                    # Close GPS connection
                    try:
                        self.gps.close()
                    except:
                        pass  # Ignore errors during cleanup

            # Clean up logging handlers to ensure async threads are properly closed
            self._cleanup_logging_handlers()
            
            # Shutdown transform executor to avoid thread leakage
            # self._shutdown_world_tf_executor()
                        
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")
            print(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")
    
    def _cleanup_logging_handlers(self):
        """Clean up logging handlers to ensure async logging threads are properly closed."""
        try:
            # Import the global logging config
            from md_logging_config import fleet_logging
            
            # Get all loggers that might have been created for this vehicle
            logger_names_to_cleanup = [
                f'vehicle_{self.vehicle_id}',
                f'communication_vehicle_{self.vehicle_id}',
                f'gps_vehicle_{self.vehicle_id}',
                f'control_vehicle_{self.vehicle_id}',
                f'observer_vehicle_{self.vehicle_id}',
                f'fleet_observer_vehicle_{self.vehicle_id}',
                f'trust_vehicle_{self.vehicle_id}'
            ]
            
            # Close handlers for each logger
            for logger_name in logger_names_to_cleanup:
                if logger_name in fleet_logging.loggers:
                    logger = fleet_logging.loggers[logger_name]
                    for handler in logger.handlers:
                        try:
                            handler.close()
                        except Exception as e:
                            print(f"Error closing handler for {logger_name}: {e}")
                    
                    # Clear handlers list
                    logger.handlers.clear()
                    
            print(f"Vehicle {self.vehicle_id}: Logging handlers cleaned up")
            
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error during logging cleanup: {e}")
    
    def stop(self):
        """Gracefully stop the vehicle process control logic."""
        self.logger.info(f"Vehicle {self.vehicle_id}: Stopping control logic")
        
        # First stop the running flag to exit the main loop
        self.running.clear()
        
        # Stop CamLidarFusion thread
        if hasattr(self, 'cam_lidar_thread_running') and self.cam_lidar_thread_running is not None:
            self.cam_lidar_thread_running.clear()
        if hasattr(self, 'cam_lidar_thread') and self.cam_lidar_thread is not None:
            self.cam_lidar_thread.join(timeout=1.0)  # Wait up to 1 second for thread to finish
            self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion thread stopped")
        
        # # Stop all control immediately with zero commands
        # try:
        #     if self.use_physical_qcar and self.physical_qcar is not None:
        #         # Stop physical QCar
        #         self.physical_qcar.write(0, 0)
        #         print(f"Vehicle {self.vehicle_id}: Physical QCar stopped with zero commands")
        #     else:
        #         # Stop virtual QCar
        #         self.qcar.set_velocity_and_request_state(
        #                 forward=0.0, 
        #                 turn=0.0,
        #                 headlights=False,
        #                 leftTurnSignal=False,
        #                 rightTurnSignal=False,
        #                 brakeSignal=False,
        #                 reverseSignal=False
        #             )
        #         print(f"Vehicle {self.vehicle_id}: Virtual QCar stopped with zero commands")
        # except Exception as e:
        #     print(f"Vehicle {self.vehicle_id}: Error stopping vehicle: {e}")
        
        # Close QLabs connection
        try:
            if hasattr(self, 'qlabs') and self.qlabs is not None:
                self.qlabs.close()
                print(f"Vehicle {self.vehicle_id}: QLabs connection closed")
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error closing QLabs: {e}")
            
        self.logger.info(f"Vehicle {self.vehicle_id}: Control logic stopped")