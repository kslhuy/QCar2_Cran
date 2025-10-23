import multiprocessing
import threading
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

# QLabs imports - these will be created inside each process
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from pal.utilities.math import wrap_to_pi
from qvl.multi_agent import readRobots


from qvl.real_time import QLabsRealTime

# Physical QCar imports (for EKF and GPS mode)
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from hal.content.qcar_functions import QCarEKF
PHYSICAL_QCAR_AVAILABLE = True

# Controller imports
from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle

# Other imports
from CommHandler import CommHandler
from VehicleLeaderController_RTmodel import VehicleLeaderController_RTmodel
from VehicleFollowerController import VehicleFollowerController
from StateQueue import StateQueue
from VehicleObserver import VehicleObserver
from src.Trust.TriPTrustModel import TriPTrustModel
from GraphBasedTrust import GraphBasedTrustEvaluator
from CamLidarFusion import CamLidarFusion
from src.Weight.Weight_Trust_module import WeightTrustModule  # Week 3: Adaptive weights


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


def vehicle_process_main(vehicle_config: Dict , stop_event: multiprocessing.Event, 
                        status_queue: multiprocessing.Queue, start_event_control: multiprocessing.Event):
    """
    Main function that runs in each vehicle process.
    
    Args:
        vehicle_config: Dictionary containing all vehicle configuration
        qcar: Pre-spawned QLabsQCar2 instance for this vehicle
        stop_event: Multiprocessing event to signal process shutdown
        status_queue: Queue for sending status updates to main process
        start_event: Multiprocessing event for synchronized control start
    """
    
    # Extract configuration
    vehicle_id = vehicle_config['vehicle_id']
    controller_type = vehicle_config['controller_type']
    is_leader = vehicle_config['is_leader']


    try:
        # print(f"Vehicle {vehicle_id}: Spawn success: {success}")
                    
        # Create the vehicle instance
        vehicle = VehicleProcess(
            vehicle_id=vehicle_id,
            qcar=None,
            qlabs=None,
            controller_type=controller_type,
            is_leader=is_leader,
            vehicle_config=vehicle_config,
            stop_event=stop_event,
            status_queue=status_queue,
            start_event=start_event_control
        )
        
        # Signal successful initialization
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'initialized'})
        

        # Wait for all vehicles to initialize before starting the run loop
        print(f"Vehicle {vehicle_id}: Waiting for fleet initialization...")
        # NOTE: Removed start_event.wait() here - now control waits inside run loop
        # start_event_run.wait()
        print(f"Vehicle {vehicle_id}: Starting run loop (observer/communication active, control waiting)")

        # Run the vehicle with emergency stopping
        try:
            vehicle.run()
        except KeyboardInterrupt:
            print(f"Vehicle {vehicle_id}: KeyboardInterrupt caught, stopping immediately")
        except Exception as e:
            print(f"Vehicle {vehicle_id}: Unexpected error during run: {e}")
        finally:
            # EMERGENCY STOP: Ensure vehicle is stopped before process exits
            # vehicle.physical_qcar.write(0, 0)
            vehicle.physical_qcar.read_write_std(throttle= 0, steering= 0)

            
        # After run() exits, call cleanup to clean up
        vehicle.cleanup()
        
    except Exception as e:
        print(f"Vehicle {vehicle_id}: Process error: {e}")
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'error', 'error': str(e)})
    finally:
        print(f"Vehicle {vehicle_id}: Process shutting down")
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'shutdown'})





class VehicleProcess:
    """
    Vehicle class that runs in its own process and manages its own control logic.
    Each process creates its own QLabs connection and QCar object.
    """
    
    def __init__(self, vehicle_id: int, qcar: QLabsQCar2, qlabs: QuanserInteractiveLabs,
                 controller_type: str = "CACC", is_leader: bool = False, 
                 vehicle_config: Dict = None, stop_event: multiprocessing.Event = None,
                 status_queue: multiprocessing.Queue = None, start_event: multiprocessing.Event = None):
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
            start_event: Multiprocessing event for synchronized control start
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.qlabs = qlabs
        self.controller_type = controller_type
        self.is_leader = is_leader
        self.config = vehicle_config
        self.stop_event = stop_event
        self.status_queue = status_queue
        self.start_event = start_event

        # Initialize Vehicle Postion
        spawn_location = vehicle_config.get('spawn_location', [0, 0, 0])
        spawn_rotation = vehicle_config.get('spawn_rotation', [0, 0, 0])
        initial_pose = np.array([spawn_location[0], spawn_location[1], np.deg2rad(spawn_rotation[2])])
        

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
        
        # -----------Initialize QCar interfaces based on mode
        self.physical_qcar = None
        self.use_physical_qcar = True

        if not IS_PHYSICAL_QCAR:
            robotsDir = readRobots()
            # THIS VERSION OF VEHICLE CONTROL NEEDS THE CARS TO BE INITIALIZED USING THE MULTIAGENT CLASS
            name = f"QC2_{self.vehicle_id}"
            Car_rt_config = robotsDir[name]
            calibrate=False
        else:
            calibrate =  'y' in input('do you want to recalibrate?(y/n)')
        # Initialize physical QCar interface for leader

        print(Car_rt_config["actorNumber"], Car_rt_config["gpsPort"], Car_rt_config["lidarIdealPort"])

        self.physical_qcar = QCar(readMode=1, frequency=self.controller_rate , hilPort=Car_rt_config["hilPort"])
        # # Initialize GPS for EKF if steering control is enabled
        if self.enable_steering_control or calibrate:
            self.gps = QCarGPS(initialPose=initial_pose,calibrate=calibrate, 
                gpsPort=Car_rt_config["gpsPort"], lidarIdealPort=Car_rt_config["lidarIdealPort"])
        else:
            self.gps = memoryview(b'')
            print(f"Vehicle {self.vehicle_id}: Physical QCar initialized without GPS/EKF")
        
        # -------------- Multiprocessing-safe running flag
        self.running = multiprocessing.Event()
        self.running.set()
        
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
        
        # Leader state tracking (for followers) - initialize as None
        self.leader_state = None  # Will be updated when receiving states from leader vehicle
        

        
        # Configure logging for performance
        self.configure_logging_for_performance()
        
        # GPS Time Synchronization
        gps_server_ip = vehicle_config.get('gps_server_ip', '127.0.0.1')
        gps_server_port = vehicle_config.get('gps_server_port', 8001)
        self.gps_sync = GPSSync(gps_server_ip, gps_server_port, vehicle_id)
        self.sync_interval = 5.0
        self.last_sync_attempt = time.time()
        
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
                self.leader_controller = VehicleLeaderController_RTmodel(
                    vehicle_id=self.vehicle_id,
                    Qcar_RT=self.physical_qcar,
                    gps=self.gps,
                    ekf=None,
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

        


        # Initialize control input tracking
        self.current_control_input = np.array([0.0, 0.0])

        # Sensor data tracking
        self.last_gyroscope_z = 0.0
        self.last_motor_tach = 0.0
        self.acceleration_IMU = np.zeros(3)



        self.gps_data_cache = {
            'pos': None,
            'rot': None,
            'vel': 0.0,
            'available': False,
            'timestamp': 0.0,
            'last_update': 0.0
        }





        # Initialize Graph-Based Trust Model for connected vehicles only
        self.trust_enabled = vehicle_config.get('enable_trust_evaluation', True)
        
        if self.trust_enabled and len(self.connected_vehicles) > 0:
            # Create trust models only for connected vehicles (based on fleet graph)
            trust_models = {}
            for target_vehicle_id in self.connected_vehicles:
                if target_vehicle_id != self.vehicle_id:  # Don't create trust model for self
                    trust_models[target_vehicle_id] = TriPTrustModel()
                    # print(f"Vehicle {self.vehicle_id}: Created trust model for connected vehicle {target_vehicle_id}")
            
            # Create hybrid buffered graph-based trust evaluator
            trust_update_interval = vehicle_config.get('trust_update_interval', 0.5)
            max_buffer_size = vehicle_config.get('trust_buffer_size', 20)
            
            self.trust_evaluator = GraphBasedTrustEvaluator(
                vehicle_id=self.vehicle_id,
                connected_vehicles=self.connected_vehicles,
                trust_models=trust_models,
                trust_update_interval=trust_update_interval,
                max_buffer_size=max_buffer_size
            )
            
            print(f"Vehicle {self.vehicle_id}: Hybrid buffered trust evaluation enabled - "
                  f"{len(trust_models)} connected vehicles, "
                  f"update_interval={trust_update_interval}s, buffer_size={max_buffer_size}")
            
            # Log trust initialization details
            self.trust_logger.info(f"TRUST_INIT: Hybrid buffered trust evaluation initialized - "
                                 f"connected_vehicles={self.connected_vehicles}, "
                                 f"trust_models_count={len(trust_models)}, "
                                 f"update_interval={trust_update_interval}s, "
                                 f"buffer_size={max_buffer_size}")
        else:
            self.trust_evaluator = None
            print(f"Vehicle {self.vehicle_id}: Trust evaluation disabled or no connected vehicles")
            self.trust_logger.info(f"TRUST_INIT: Trust evaluation disabled - "
                                 f"trust_enabled={self.trust_enabled}, "
                                 f"connected_vehicles_count={len(self.connected_vehicles)}")
        
        # ---- TASK 1.2: Initialize quality tracking for adaptive weight calculation
        from collections import deque
        self.neighbor_quality = {}
        for neighbor_id in self.connected_vehicles:
            if neighbor_id == self.vehicle_id:
                continue  # Skip self
            
            self.neighbor_quality[neighbor_id] = {
                'last_recv_time': 0.0,              # When last message received
                'recv_count': 0,                     # Total messages received
                'expected_count': 0,                 # Expected messages (for drop rate)
                'drop_count': 0,                     # Dropped/missed messages
                'recent_innovations': deque(maxlen=10),  # Last 10 innovation values
                'last_covariance': None,             # Last received covariance trace
                'last_distance': None,               # Last computed distance
                'last_state': None                   # Last received state (for prediction)
            }
        # ---- TASK 1.2 END
        self.comm_start_time = time.time()  # Track when communication started
        self.logger.info(f"Vehicle {self.vehicle_id}: Initialized quality tracking for {len(self.neighbor_quality)} neighbors")
        
        # VERIFICATION LOG: List all neighbors being tracked
        if len(self.neighbor_quality) > 0:
            neighbor_ids = list(self.neighbor_quality.keys())
            self.logger.info(f"[VERIFY] V{self.vehicle_id}: Tracking quality for neighbors: {neighbor_ids}")
        else:
            self.logger.warning(f"[VERIFY] V{self.vehicle_id}: No neighbors to track (neighbor_quality is empty)!")
        
        # Connect quality metrics to trust evaluator
        if self.trust_evaluator is not None:
            self.trust_evaluator.set_vehicle_process_reference(self)
            self.logger.info(f"Vehicle {self.vehicle_id}: Connected quality metrics to trust evaluator")

        # ═══════════════════════════════════════════════════════════
        # WEEK 3: Initialize WeightTrustModule for adaptive weights
        # ═══════════════════════════════════════════════════════════
        if self.trust_enabled and len(self.connected_vehicles) > 0:
            try:
                # Get trust threshold and kappa from config
                trust_threshold = vehicle_config.get('trust_threshold', 0.5)
                kappa = vehicle_config.get('kappa', 5)
                
                # Initialize WeightTrustModule with vehicle_id for logging
                self.weight_trust_module = WeightTrustModule(
                    graph=self.fleet_graph,
                    trust_threshold=trust_threshold,
                    kappa=kappa,
                    vehicle_id=self.vehicle_id
                )
                
                self.logger.info(f"Vehicle {self.vehicle_id}: WeightTrustModule initialized - "
                               f"trust_threshold={trust_threshold}, kappa={kappa}")
                self.logger.info(f"[WEEK3] V{self.vehicle_id}: Adaptive weight calculation enabled")
            except Exception as e:
                self.logger.error(f"Vehicle {self.vehicle_id}: WeightTrustModule initialization failed: {e}")
                self.weight_trust_module = None
        else:
            self.weight_trust_module = None
            self.logger.info(f"Vehicle {self.vehicle_id}: WeightTrustModule disabled (trust not enabled)")

        # ═══════════════════════════════════════════════════════════
        # Initialize VehicleObserver with Week 3 integration
        # ═══════════════════════════════════════════════════════════
        try:
            self.observer = VehicleObserver(
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                config=self.config,
                logger=self.logger,
                initial_pose=initial_pose,
                vehicle_process=self,  # Week 3: Pass self reference for trust access
                weight_trust_module=self.weight_trust_module  # Week 3: Pass weight module
            )
            
            if self.weight_trust_module is not None:
                self.logger.info(f"[WEEK3] V{self.vehicle_id}: Observer initialized with trust-based adaptive weights")
            else:
                self.logger.info(f"Vehicle {self.vehicle_id}: Observer initialized with default equal weights")
                
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Observer initialization failed: {e}")
            self.observer = None

        # Initialize CamLidarFusion Component (only for vehicles with following targets)
        enable_cam_lidar_fusion = vehicle_config.get('enable_cam_lidar_fusion', False)
        if self.following_target is not None and enable_cam_lidar_fusion:
            try:
                # Disable visualization by default to reduce initialization overhead
                enable_visualization_camera = vehicle_config.get('enable_visualization_camera', False)
                enable_visualization_lidar = vehicle_config.get('enable_visualization_lidar', False)

                self.cam_lidar_fusion = CamLidarFusion(
                    qcar=self.qcar,
                    vehicle_id=self.vehicle_id,
                    image_width=640,
                    image_height=480,
                    scale_factor=0.1,
                    verbose=False,
                    logger=self.logger,
                    enable_visualization_camera=enable_visualization_camera,
                    show_lidar_plot=enable_visualization_lidar
                )
                if self.cam_lidar_fusion and self.cam_lidar_fusion.initialized:
                    print(f"Vehicle {self.vehicle_id}: CamLidarFusion component initialized for following vehicle {self.following_target}")
                    # self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion component initialized for following vehicle {self.following_target}")
                else:
                    print(f"Vehicle {self.vehicle_id}: CamLidarFusion initialization incomplete - vehicle may still function")
                    # self.logger.warning(f"Vehicle {self.vehicle_id}: CamLidarFusion initialization incomplete - vehicle may still function")
                    self.cam_lidar_fusion = None
            except Exception as e:
                print(f"Vehicle {self.vehicle_id}: CamLidarFusion initialization failed: {e}")
                # self.logger.warning(f"Vehicle {self.vehicle_id}: CamLidarFusion initialization failed: {e}")
                self.cam_lidar_fusion = None
                # Don't fail the entire vehicle initialization for CamLidarFusion issues
        else:
            self.cam_lidar_fusion = None
            if not enable_cam_lidar_fusion:
                self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion disabled by configuration")
            else:
                self.logger.info(f"Vehicle {self.vehicle_id}: CamLidarFusion not initialized (no following target)")




        # Initialize CamLidarFusion (moved to main loop to avoid multiprocessing issues)
        # Thread-based approach removed due to self.qcar threading conflicts

        # Communication thread
        self.comm_thread = None
        self.comm_thread_running = False
        
        # Start communication thread
        self.start_communication_thread()


        # ------ Final initialization steps

        print(f"Vehicle {self.vehicle_id} process initialized successfully")
        self.logger.info(f"Vehicle {self.vehicle_id} process initialized successfully")

    def configure_logging_for_performance(self):

        # Initialize logging for this process
        self.logger = get_individual_vehicle_logger(self.vehicle_id)
        self.comm_logger = get_communication_logger(self.vehicle_id)
        self.gps_logger = get_gps_logger(self.vehicle_id)
        self.control_logger = get_control_logger(self.vehicle_id)
        self.observer_logger = get_observer_logger(self.vehicle_id)
        self.fleet_observer_logger = get_fleet_observer_logger(self.vehicle_id)
        self.trust_logger = get_trust_logger(self.vehicle_id)

        """Configure logging settings for optimal performance."""
        disable_all_logging()
        set_module_logging('control', True)
        set_module_logging('observer', True)
        set_module_logging('communication', True)
        set_module_logging('fleet_observer', True)  # Enable fleet observer logging
        # set_module_logging('trust', True)  # Enable trust logging
        # set_module_logging('weight', True)  # Enable weight logging
        
        # # Set fleet observer logger to INFO level to see the velocity debug messages
        self.fleet_observer_logger.setLevel(logging.INFO)
        # self.comm_logger.setLevel(logging.DEBUG)  # Already set to INFO - this enables info messages
        self.comm_logger.setLevel(logging.INFO)  
        self.observer_logger.setLevel(logging.INFO)
        self.trust_logger.setLevel(logging.INFO)  # Enable trust logging at INFO level
        self.control_logger.setLevel(logging.DEBUG)  # Enable control logging at DEBUG level

        # self.weight_logger.setLevel(logging.INFO)  # Enable weight logging at INFO level

    # ------------------------------------ GPS Update Methods
    # region GPS Update Methods
    def readData_Qcar(self):
        """Update GPS data from QCar sensors (supports both virtual and physical QCar modes)."""
        try:
            gps_start_time = time.perf_counter()
            current_time = time.time()
            self._update_physical_qcar_data(current_time)


                
            # gps_duration = (time.perf_counter() - gps_start_time) * 1000
            # if gps_duration > 2.0:  # Log if GPS update takes >2ms
            #     self.gps_logger.warning(f"Vehicle {self.vehicle_id}: GPS update took {gps_duration:.4f}ms")
                
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: GPS update error: {e}")
            self.gps_data_cache['available'] = False
            print(f"Vehicle {self.vehicle_id}: GPS update exception: {e}")





    def _update_physical_qcar_data(self, current_time):
        """Update using physical QCar API with EKF (similar to vehicle_control.py)."""
        if self.physical_qcar is None:
            return
            
        # Read from physical QCar sensors
        self.physical_qcar.read()
        
        available = False  # Initialize available flag
        
        if self.enable_steering_control and hasattr(self, 'gps') and self.gps is not None:
            # GPS and EKF update
            if self.gps.readGPS():
                # GPS data available
                y_gps = np.array([
                    self.gps.position[0],
                    self.gps.position[1], 
                    self.gps.orientation[2]
                ])
                self.current_pos = [y_gps[0], y_gps[1], 0.0]
                self.current_rot = [0.0, 0.0, y_gps[2]]
                
                available = True
                # Update GPS cache
                self.gps_data_cache.update({
                    'pos': self.current_pos.copy(),
                    'rot': self.current_rot.copy(),
                    'available': available,
                    'timestamp': current_time,
                    'last_update': current_time
                })
            else:
                available = False

        # Update GPS cache
        self.velocity = self.physical_qcar.motorTach
        self.last_gyroscope_z = self.physical_qcar.gyroscope[2]
        self.acceleration_IMU = self.physical_qcar.accelerometer
        self.gps_data_cache['vel'] = self.velocity

        self.gps_data_cache['available'] = available

    def get_cached_gps_data(self):
        """Get cached GPS data."""
        return self.gps_data_cache.copy()

    # endregion   GPS Update Methods

    # --------------------- Timing Helper Methods ---------------------
    # region Timing Helper Methods
    
    def calculate_real_dt(self, context_name: str, target_rate: float, use_gps_time: bool = False) -> float:
        """
        Calculate real elapsed time (dt) for any periodic update (controller, observer, etc.).
        
        Args:
            context_name: Name of the context (e.g., 'control', 'observer', 'gps')
            target_rate: Target update rate in Hz
            use_gps_time: If True, use GPS-synchronized time; otherwise use system time
            
        Returns:
            Calculated dt clamped between 1ms and 100ms for stability
        """
        # Get current time based on preference
        current_time = self.gps_sync.get_synced_time() if use_gps_time else time.time()
        
        # Create unique attribute name for this context
        last_time_attr = f'_last_{context_name}_time'
        
        # Calculate dt
        if not hasattr(self, last_time_attr):
            # First iteration - use nominal dt
            setattr(self, last_time_attr, current_time)
            dt = 1.0 / target_rate
        else:
            # Calculate actual elapsed time
            last_time = getattr(self, last_time_attr)
            dt = current_time - last_time
            # Clamp dt to reasonable bounds to avoid instability
            dt = max(0.001, min(dt, 0.1))  # Between 1ms and 100ms
        
        # Update last time for next iteration
        setattr(self, last_time_attr, current_time)
        
        return dt
    
    def log_timing_performance(self, context_name: str, dt: float, target_dt: float, 
                              logger, extra_info: str = ""):
        """
        Log timing performance for debugging.
        
        Args:
            context_name: Name of the context (e.g., 'LEADER_DT', 'FOLLOWER_DT')
            dt: Actual elapsed time
            target_dt: Target elapsed time
            logger: Logger instance to use
            extra_info: Additional information to log
        """
        if logger.isEnabledFor(logging.DEBUG):
            timing_error = (dt - target_dt) 
            base_msg = (f"V{self.vehicle_id} {context_name}: dt={dt:.4f}s "
                       f"(target={target_dt:.4f}s, error={timing_error:.1f})")
            if extra_info:
                base_msg += f" {extra_info}"
            logger.debug(base_msg)
    
    # endregion Timing Helper Methods

    # --------------------- Control Logic Methods ---------------------
    # region Control Logic Methods
    
    
    
    def leader_control_logic(self):
        """Leader control logic that delegates to VehicleLeaderController."""
        if not self.is_leader or self.leader_controller is None:
            return
            
        # Get state for control (uses observer EKF estimates when available)
        control_state = self.get_state_for_control()
        
        # Calculate real dt using helper method
        dt = self.calculate_real_dt('control', self.controller_rate, use_gps_time=False)
        
        # Log timing performance
        target_dt = 1.0 / self.controller_rate
        self.log_timing_performance('LEADER_DT', dt, target_dt, self.control_logger)
        
        # Compute control commands using the dedicated leader controller
        forward_speed, steering_angle = self.leader_controller.compute_control_auto(
            current_pos=control_state['pos'],
            current_rot=control_state['rot'],
            velocity=control_state['vel'],
            dt=dt  # Use actual elapsed time
        )

        # Debug: Print leader control commands
        if self.control_logger.isEnabledFor(logging.DEBUG):
            self.control_logger.debug(
                f"V{self.vehicle_id} LEADER_CMD: fwd={forward_speed:.3f}, steer={steering_angle:.3f}"
            )

        # Update control input for observer
        # Convert controller outputs to observer format [steering, acceleration]
        self.current_control_input = np.array([steering_angle, forward_speed])
        
        # Store steering angle for physical QCar EKF
        self._last_steering_angle = steering_angle
        
        # Apply control commands to vehicle based on mode
        # NOTE: Previously, when use_control_observer_mode was True we skipped sending any commands.
        # This caused the leader to remain at its spawn position for virtual (non-physical) runs.
        # We now only suppress direct virtual commands if BOTH observer mode is enabled AND a physical QCar is active.
        

        self.physical_qcar.write(forward_speed, steering_angle)
            

   
        # Normalize numeric types (convert numpy types to plain Python floats) before broadcasting
        control_state['pos'] = [float(x) for x in control_state['pos']]
        control_state['rot'] = [float(r) for r in control_state['rot']]
        control_state['vel'] = float(control_state['vel'])

        # # Extra debug for movement diagnosis
        # self.control_logger.debug(
        #     f"Vehicle {self.vehicle_id}: Cmd fwd={forward_speed:.3f}, steer={steering_angle:.3f}, src={control_state['source']}"
        # )




    def follower_control_logic(self):
        """Follower control logic that delegates to VehicleFollowerController."""
        if self.is_leader or self.follower_controller is None:
            print(f"Vehicle {self.vehicle_id}: Skipping follower control - is_leader={self.is_leader}, controller_exists={self.follower_controller is not None}")
            return
        
        # print(f"Vehicle {self.vehicle_id}: Executing follower control logic")
        try:
            # Use GPS-synchronized time for better prediction
            current_gps_time = self.gps_sync.get_synced_time()
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Follower control - GPS time: {current_gps_time}")
            
            # Get latest valid target state directly (simplified approach)
            target_data = None
            
            if self.leader_state is not None:
                # Check if the state is recent enough (within 2 seconds for 3+ vehicle fleets)
                state_age = current_gps_time - self.leader_state.get('timestamp', 0)
                if state_age <= 2.0:  # Increased threshold for multi-vehicle fleets
                    target_data = {
                        'pos': self.leader_state.get('pos', [0, 0, 0]),
                        'rot': self.leader_state.get('rot', [0, 0, 0]),
                        'vel': self.leader_state.get('v', 0.0),
                        'timestamp': self.leader_state.get('timestamp', current_gps_time),
                        'state_age': state_age
                    }
                    # print(f"Vehicle {self.vehicle_id}: Using target data from vehicle {self.following_target}, age={state_age:.3f}s")
                    # self.comm_logger.info(f"Vehicle {self.vehicle_id}: Using target data from vehicle {self.following_target}, age={state_age:.3f}s")
                else:
                    self.comm_logger.info(f"Vehicle {self.vehicle_id}: Target state too old ({state_age:.3f}s) from vehicle {self.following_target}")
            
            # No valid target data available
            if target_data is None:
                self.comm_logger.info(f"Vehicle {self.vehicle_id}: No valid target data available from vehicle {self.following_target}, stopping vehicle")
                self.current_control_input = np.array([0.0, 0.0])
                # Stop vehicle if no target data
                self.physical_qcar.write(0.0, 0.0)

                return
            
            # Get current vehicle local state for control
            current_state = self.get_state_for_control()
            
            # Calculate real dt using helper method (use GPS time for follower)
            dt = self.calculate_real_dt('control', self.controller_rate, use_gps_time=True)
            
            # Log timing performance with extra follower-specific info
            target_dt = 1.0 / self.controller_rate
            extra_info = f"following=V{self.following_target}, state_age={target_data['state_age']:.3f}s"
            self.log_timing_performance('FOLLOWER_DT', dt, target_dt, self.control_logger, extra_info)
            
            # Compute control commands that follow the target vehicle
            forward_speed, steering_angle = self.follower_controller.compute_vehicle_following_control(
                current_pos=current_state['pos'],
                current_rot=current_state['rot'],
                current_velocity=current_state['vel'],
                leader_pos=target_data['pos'],  # Position of the target vehicle to follow
                leader_rot=target_data['rot'],  # Rotation of the target vehicle to follow
                leader_velocity=target_data['vel'],  # Velocity of the target vehicle to follow
                leader_timestamp=target_data['timestamp'],
                dt=dt  # Use actual elapsed time instead of nominal
            )

            # # Compute control commands that follow a trajectory
            # forward_speed_trj, steering_angle_trj = self.follower_controller.compute_trajectory_following_control(
            #     current_pos=current_state['pos'],
            #     current_rot=current_state['rot'],
            #     current_velocity=current_state['vel'])

            # Optional: Fusion 2 control commands 
            
            
            # Debug logging for control commands
            if self.control_logger.isEnabledFor(logging.DEBUG):
                self.control_logger.debug(
                    f"V{self.vehicle_id} FOLLOWER_CMD: fwd={forward_speed:.3f}, steer={steering_angle:.3f}"
                )
            
            # Update control input for observer
            self.current_control_input = np.array([steering_angle, forward_speed])
            
            # Apply control commands to vehicle
            self.physical_qcar.write(forward_speed, steering_angle)

            # self.physical_qcar.write(forward_speed_trj, steering_angle_trj)


        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Follower control error: {e}")
            # Reset control input on error
            self.current_control_input = np.array([0.0, 0.0])
            self.physical_qcar.write(0.0, 0.0)
    # --------------------- Control Logic Methods ---------------------
    #endregion Control Logic Methods
   

    # --------------------- Main Run Loop ---------------------
    def run(self):
        """Main run loop for the vehicle process."""
        self.logger.info(f"Vehicle {self.vehicle_id}: Starting main run loop")
        
        # Initialize timing
        general_update_dt = 1.0 / self.update_rate
        control_dt = 1.0 / self.controller_rate
        observer_dt = 1.0 / self.observer_rate
        gps_dt = 1.0 / self.gps_update_rate
        
        last_control_time = time.time()
        last_observer_time = time.time()
        last_gps_time = time.time()
        last_status_time = time.time()
        last_send_communication_time = time.time()
        last_trust_summary_time = time.time()  # For periodic trust logging
        last_trust_update_time = time.time()   # For periodic trust calculation
        last_cam_lidar_time = time.time()  # For CamLidarFusion updates
        last_gps_sync_time = time.time()  # For GPS time synchronization
        
        # CRITICAL: Perform initial GPS sync before starting
        try:
            self.gps_sync.sync_with_gps()
            self.gps_logger.info(f"Vehicle {self.vehicle_id}: Initial GPS sync completed, offset={self.gps_sync.gps_time_offset:.3f}s")
        except Exception as e:
            self.gps_logger.error(f"Vehicle {self.vehicle_id}: Initial GPS sync failed: {e}")

        
        try:
            while self.running.is_set() and not self.stop_event.is_set():
                current_time = time.time()

                # GPS time synchronization (every 5 seconds)
                if current_time - last_gps_sync_time >= self.sync_interval:
                    try:
                        self.gps_sync.sync_with_gps()
                        last_gps_sync_time = current_time
                    except Exception as e:
                        self.gps_logger.error(f"Vehicle {self.vehicle_id}: GPS sync failed: {e}")


                # Observer update
                if current_time - last_observer_time >= observer_dt:
                    observer_start = time.time()
                    self.observer_update()
                    observer_duration = time.time() - observer_start
                    if observer_duration > observer_dt:
                        self.observer_logger.warning(
                            f"[TIMING] V{self.vehicle_id}: Observer update took {observer_duration*1000:.1f}ms "
                            f"(exceeds {observer_dt*1000:.1f}ms interval) - may cause broadcast delays!"
                        )
                    last_observer_time = current_time
                
                # CamLidarFusion update (moved from separate thread to avoid multiprocessing issues)
                if current_time - last_cam_lidar_time >= general_update_dt:
                    if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None and self.cam_lidar_fusion.initialized:
                        try:
                            self.cam_lidar_fusion.update_sensor_data()
                        except Exception as e:
                            self.logger.error(f"Vehicle {self.vehicle_id}: CamLidarFusion update error: {e}")
                    last_cam_lidar_time = current_time
                
                # Control loop - only execute when fleet is ready
                if current_time - last_control_time >= control_dt and self.start_event.is_set():
                    if self.is_leader:
                        self.leader_control_logic()
                        # print(f"Vehicle {self.vehicle_id}: Leader control logic executed")
                    else:
                        # print(f"Vehicle {self.vehicle_id}: Follower control logic executed")
                        self.follower_control_logic()
                    last_control_time = current_time
                
                # Periodic trust update - calculate trust from buffered data
                if self.trust_enabled and hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
                    if self.trust_evaluator.should_update_trust(current_time):
                        # Get our LOCAL state from our observer (for local trust comparison)
                        our_local_state = self.get_best_available_state()
                        
                        # Get OUR distributed observer's FLEET estimates (for global trust comparison)
                        our_fleet_estimates = self.get_fleet_estimates()
                        
                        # Update all trust scores using buffered data (local + global)
                        # - Local trust: Compare received direct messages with our_local_state
                        # - Global trust: Compare received fleet estimates with our_fleet_estimates
                        self.trust_evaluator.update_all_trust_scores(
                            our_local_state=our_local_state,
                            our_fleet_estimates=our_fleet_estimates,
                            logger=self.trust_logger
                        )
                        
                        last_trust_update_time = current_time



                
                # Send status update periodically
                if current_time - last_status_time >= 5.0:
                    self.send_status_update()  
                    last_status_time = current_time
                
                
                # Small sleep to prevent excessive CPU usage
                time.sleep(0.001)  # 1ms
                
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Main loop error: {e}")
            # self.logger.error(f"Vehicle {self.vehicle_id}: Main loop error: {e}")
        finally:
            # self.cleanup()
            pass

    # CamLidarFusion thread method removed (moved to main loop)

    def observer_update(self):
        """
        Update observer with current GPS data, sensor data, and control inputs.
        Similar to Vehicle.py observer_loop but simplified for process context.
        """
        if self.observer is None:
            return
            
        try:
            current_time = self.gps_sync.get_synced_time()
            
            # Optional: Calculate and log observer update timing
            # dt = self.calculate_real_dt('observer', self.observer_rate, use_gps_time=True)
            # target_dt = 1.0 / self.observer_rate
            # self.log_timing_performance('OBSERVER_DT', dt, target_dt, self.observer_logger)
            
            self.readData_Qcar()
            # Get GPS data
            gps_data = self.get_cached_gps_data()
            measured_state = None
            if gps_data['available']:
                # Convert GPS data to numpy array format expected by observer
                # Observer expects [x, y, theta, v] format
                measured_state = np.array([
                    gps_data['pos'][0],  # x
                    gps_data['pos'][1],  # y
                    gps_data['rot'][2],  # theta (yaw angle)
                    gps_data['vel']      # velocity
                ])

            
            # -------- Update observer local state
            estimated_state = self.observer.update_local_state(
                measured_state=measured_state,
                control_input=self.current_control_input,
                timestamp=current_time,
                motor_tach=self.velocity,
                gyroscope_z=self.last_gyroscope_z,
                acceleration=self.acceleration_IMU
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
                        self._set_fleet_vehicle_estimate(vehicle_idx, vehicle_state, current_time)
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
                
                # # EVENT-DRIVEN BROADCASTING: Only broadcast when new state data is available
                # # or when maximum time interval exceeded (for stationary vehicles)
                # if self.observer.should_broadcast_state():
                #     self.broadcast_own_state()
                #     self.observer.mark_state_broadcasted()  # Clear the flag and update timestamp
                    
                #     # Also broadcast fleet estimates when we have new state
                #     if hasattr(self, 'fleet_state_estimates') and len(self.fleet_state_estimates) > 1:
                #         self.broadcast_fleet_estimates()
                # # No else clause - we silently skip broadcasts when not needed (rate limiting working)


                # This is the logical place since observer maintains the most accurate state estimate
                self.broadcast_own_state()
                
                # NEW: Broadcast fleet estimates if distributed observer is working
                if hasattr(self, 'fleet_state_estimates') and len(self.fleet_state_estimates) > 1:
                    self.broadcast_fleet_estimates()

                
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Invalid estimated state from observer: {estimated_state}")
                
        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Observer update error: {e}")

    # --------------------- Communication Methods ---------------------
    #region Communication Methods

    def start_communication_thread(self):
        """Start a separate thread for handling communication."""
        if self.comm is not None and self.comm_thread is None:
            self.comm_thread_running = True
            self.comm_thread = threading.Thread(target=self.communication_thread_loop, name=f"CommThread_V{self.vehicle_id}")
            self.comm_thread.daemon = True
            self.comm_thread.start()
            self.logger.info(f"Vehicle {self.vehicle_id}: Communication thread started")

    def communication_thread_loop(self):
        """Dedicated thread for handling communication."""
        while self.comm_thread_running and self.running.is_set():
            try:
                self.handle_communication()
                time.sleep(0.005)  # Small sleep to prevent excessive CPU usage
            except Exception as e:
                self.comm_logger.error(f"Vehicle {self.vehicle_id}: Communication thread error: {e}")
                time.sleep(0.005)  # Longer sleep on error

    def broadcast_fleet_estimates(self):
        """
        Broadcast fleet state estimates from distributed observer.
        This sends the global view of all vehicle states.
        """
        if self.comm is None or not hasattr(self, 'fleet_state_estimates'):
            return
            
        try:
            current_time = self.gps_sync.get_synced_time()
            
            # Build message using helper
            fleet_message = self._build_fleet_estimates_message(current_time)
            if not fleet_message:
                return
            
            
            # Broadcast fleet estimates to all vehicles
            success = self.comm.send_fleet_estimates_broadcast(fleet_message)
            
            if success:
                self.observer_logger.debug(f"Vehicle {self.vehicle_id}: Broadcasted fleet estimates for {len(self.fleet_state_estimates)} vehicles")
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Failed to broadcast fleet estimates")
                
        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Fleet broadcast error: {e}")


    def broadcast_own_state(self):
        """
        Broadcast this vehicle's current state to all other vehicles in the fleet.
        This method provides bidirectional communication where every vehicle shares its state.
        """
        if self.comm is None:
            return
            
        try:
            # Get the current best available state (simplified)
            current_state = self.get_best_available_state()
            
            # TASK 1.1: Get covariance trace from observer for uncertainty tracking
            try:
                # VERIFICATION LOG: Check observer status
                has_observer = self.observer is not None
                has_P_local = hasattr(self.observer, 'P_local') if has_observer else False
                P_local_not_none = (self.observer.P_local is not None) if has_P_local else False
                
                if has_observer and has_P_local and P_local_not_none:
                    cov_trace = float(np.trace(self.observer.P_local))
                    # VERIFICATION LOG: Covariance successfully extracted
                    self.comm_logger.debug(f"[VERIFY] V{self.vehicle_id}: Broadcasting with covariance_trace={cov_trace:.4f}")
                else:
                    cov_trace = 1.0  # Default uncertainty if covariance not available
                    # VERIFICATION LOG: Using default covariance - show why
                    self.comm_logger.warning(
                        f"[VERIFY] V{self.vehicle_id}: Observer covariance not available "
                        f"(has_observer={has_observer}, has_P_local={has_P_local}, "
                        f"P_local_not_none={P_local_not_none}), using default=1.0"
                    )
            except Exception as e:
                self.comm_logger.warning(f"[VERIFY] V{self.vehicle_id}: Failed to get covariance: {e}")
                cov_trace = 1.0
            
            # FIXED: Use GPS synchronized time instead of system time for consistency
            gps_timestamp = self.gps_sync.get_synced_time()
            system_timestamp = time.time()
            
            # CRITICAL DEBUG: Check if observer's last update time matches current GPS time
            observer_last_update = getattr(self.observer, 'last_update_time', None) if self.observer else None
            time_since_observer_update = gps_timestamp - observer_last_update if observer_last_update else None
            
            # CRITICAL DEBUG: Log all timestamps to diagnose 3-second delay
            obs_update_str = f"{observer_last_update:.3f}" if observer_last_update is not None else "None"
            time_since_str = f"{time_since_observer_update:.3f}" if time_since_observer_update is not None else "None"
            
            # self.comm_logger.warning(
            #     f"[BROADCAST_TIMING] V{self.vehicle_id}: gps_timestamp={gps_timestamp:.3f}, "
            #     f"system_time={system_timestamp:.3f}, diff={gps_timestamp-system_timestamp:.3f}s, "
            #     f"observer_last_update={obs_update_str}, "
            #     f"time_since_obs_update={time_since_str}s"
            # )
            
            # Add only necessary fields for network transmission
            broadcast_state = {
                'vehicle_id': self.vehicle_id,
                'pos': [float(x) for x in current_state['pos']],
                'rot': [float(r) for r in current_state['rot']],
                'vel': float(current_state['vel']),
                'timestamp': gps_timestamp,  # FIXED: Use GPS synchronized time
                'ctrl_u': [float(x) for x in self.current_control_input],
                'covariance_trace': cov_trace  # TASK 1.1: NEW - uncertainty metric for adaptive weighting
            }
            
            # CRITICAL DEBUG LOG: Show timing information
            if time_since_observer_update is not None and time_since_observer_update > 0.5:
                self.comm_logger.warning(
                    f"[TIMING] V{self.vehicle_id}: Broadcasting state with OLD observer data! "
                    f"Observer last updated {time_since_observer_update:.3f}s ago. "
                    f"Current GPS time={gps_timestamp:.3f}, Observer update time={observer_last_update:.3f}"
                )
            else:
                self.comm_logger.debug(
                    f"[TIMING] V{self.vehicle_id}: Broadcasting fresh state. "
                    f"Time since observer update={time_since_observer_update:.3f}s"
                )
            
            # Broadcast state to all other vehicles
            self.comm.send_state_broadcast(broadcast_state)
            
            # Enhanced debug logging
            self.comm_logger.debug(
                f"Vehicle {self.vehicle_id}: Broadcasted state - pos={broadcast_state['pos']}, "
                f"vel={broadcast_state['vel']:.3f}, cov_trace={cov_trace:.4f}, ctrl_u={broadcast_state['ctrl_u']}, "
                f"timestamp={gps_timestamp:.3f}"
            )
            
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Failed to broadcast own state: {e}")

    def handle_communication(self):
        """
        Handle non-blocking communication with direct processing.
        
        FIXED: Removed the intermediate process_received_state() method that was causing
        data loss and unnecessary complexity. Now all communication data is processed
        directly without additional conversions or method calls that could lose data.
        
        This ensures that the exact data received in handle_communication() is preserved
        and processed correctly for both vehicle states and fleet estimates.
        """
        try:
            # Check for incoming messages
            if self.comm is not None:
                received_data = self.comm.receive_state_non_blocking()
                if received_data:
                    # Process data directly without unnecessary intermediate method
                    self._process_communication_data_direct(received_data)
                else:
                    # Only log this occasionally to avoid spam
                    if not hasattr(self, '_last_no_data_log') or time.time() - self._last_no_data_log > 1.0:
                        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: No data received from peers")
                        self._last_no_data_log = time.time()
            else:
                # Log communication handler not available
                if not hasattr(self, '_last_comm_error_log') or time.time() - self._last_comm_error_log > 10.0:
                    self.comm_logger.error(f"Vehicle {self.vehicle_id}: Communication handler not available")
                    self._last_comm_error_log = time.time()
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Communication error: {e}")
            print(f"Vehicle {self.vehicle_id}: Communication error: {e}")  # Keep print for critical errors

    def _process_communication_data_direct(self, received_data: dict):
        """Process received communication data directly without data loss."""
        try:
            # Handle different message types based on 'type' or 'msg_type' field
            msg_type = received_data.get('type', received_data.get('msg_type', 'vehicle_state'))
            
            if msg_type == 'fleet_estimates':
                # Process fleet estimates from distributed observer
                self._handle_fleet_estimates_direct(received_data)
            else:
                # Process individual vehicle state directly
                self._handle_vehicle_state_direct(received_data)
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Error processing communication data: {e}")

    def _handle_fleet_estimates_direct(self, fleet_message: dict):
        """Handle fleet estimates directly without data conversion loss."""
        try:
            sender_id = fleet_message.get('sender_id', fleet_message.get('vehicle_id', -1))
            estimates = fleet_message.get('estimates', {})
            message_timestamp = fleet_message.get('timestamp', time.time())
            seq = fleet_message.get('seq', -1)
            
            # Log fleet estimates with complete info but concise format
            est_data = {}
            for veh_id, est in estimates.items():
                pos = est.get('pos', [0, 0])
                rot = est.get('rot', [0, 0, 0])
                vel = est.get('vel', 0.0)
                est_data[f"V{veh_id}"] = f"Pos=({pos[0]:.4f},{pos[1]:.4f}) Rot={rot[2]:.4f} Vel={vel:.4f}"
            
            self.fleet_observer_logger.info(f"RECV_FLEET From=V{sender_id} Seq={seq} T={message_timestamp:.3f} {est_data}")
            
            # Update our own fleet estimates with external observations
            if not hasattr(self, 'external_fleet_estimates'):
                self.external_fleet_estimates = {}
            
            self.external_fleet_estimates[sender_id] = {
                'timestamp': message_timestamp,
                'estimates': estimates
            }
            
            # Add fleet estimates to observer if available
            if self.observer is not None:
                success = self.observer.add_received_state_fleet(
                    sender_id=sender_id,
                    fleet_estimates=estimates,
                    timestamp=message_timestamp
                )
                # if success:
                #     self.observer_logger.info(f"Vehicle {self.vehicle_id}: Added fleet estimates to observer - "
                #                              f"sender is ={sender_id}")
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Observer not initialized for fleet estimates")
            
            # Buffer fleet state for trust evaluation
            if self.trust_enabled and sender_id != self.vehicle_id:
                if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
                    # Get relative distance from sensor fusion if available
                    relative_distance = None
                    if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None:
                        relative_distance = self.cam_lidar_fusion.get_closest_distance()
                    self.trust_evaluator.buffer_fleet_state(
                        sender_id=sender_id,
                        fleet_state=fleet_message,
                        timestamp=message_timestamp,
                        relative_distance=relative_distance
                    )

        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Error processing fleet estimates: {e}")

    def _handle_vehicle_state_direct(self, received_state: dict):
        """Handle individual vehicle state directly without data conversion loss."""
        try:
            # Extract sender information
            sender_id = received_state.get('vehicle_id', received_state.get('id', received_state.get('sender_id', -1)))
            seq = received_state.get('seq', -1)
            timestamp = received_state.get('timestamp', time.time())
            
            # CRITICAL DEBUG: Check message age immediately upon receipt
            current_gps_time = self.gps_sync.get_synced_time()
            current_system_time = time.time()
            message_age = current_gps_time - timestamp
            
            # DEBUG: Show GPS offset to diagnose 3-second delay
            gps_offset = self.gps_sync.gps_time_offset
            system_vs_gps = current_system_time - current_gps_time
            
            # Extract state data directly without conversion
            pos = received_state.get('pos', received_state.get('position', [0, 0, 0]))
            rot = received_state.get('rot', received_state.get('rotation', [0, 0, 0]))
            vel = received_state.get('v', received_state.get('vel', received_state.get('velocity', 0.0)))
            control = received_state.get('ctrl_u', received_state.get('control_input', [0.0, 0.0]))
            
            
            #------ VERIFICATION LOG: Check if covariance_trace is in received state
            has_covariance = 'covariance_trace' in received_state
            cov_value = received_state.get('covariance_trace', 'N/A')
            
            #------ Log with all the original data preserved + message age + GPS offset
            self.comm_logger.info(f"STATE_RECV From=V{sender_id} Seq={seq} T={timestamp:.3f} "
                                f"Pos=({pos[0]:.4f},{pos[1]:.4f}) Rot={rot[2]:.4f} Vel={vel:.4f} "
                                f"Control=({control[0]:.3f},{control[1]:.3f}) "
                                f"[VERIFY] has_cov={has_covariance} cov={cov_value} "
                                f"[AGE] {message_age:.3f}s (recv_at={current_gps_time:.3f}) "
                                f"[GPS_OFFSET] {gps_offset:.3f}s [SYS_VS_GPS] {system_vs_gps:.3f}s")
            
            # # Add to state queue with original data structure preserved
            # success = self.state_queue.add_state(received_state, self.gps_sync)
            
            # # Log state queue result for debugging trust
            # if self.trust_enabled:
            #     self.comm_logger.debug(f"Vehicle {self.vehicle_id}: State from V{sender_id} queue_add={success}")
            
            # if not success:
            #     # self.comm_logger.warning(f"Vehicle {self.vehicle_id}: State rejected by queue! "
            #     #                        f"Data: sender={sender_id}, seq={seq}, timestamp={timestamp}")
            #     # # Log the exact data that was rejected
            #     # self.comm_logger.warning(f"Vehicle {self.vehicle_id}: REJECTED_DATA_DETAILS: {received_state}")
            #     pass
            # else:
            #     # queue_stats = self.state_queue.get_queue_stats()
            #     # self.comm_logger.debug(f"Vehicle {self.vehicle_id}: State added to queue successfully. "
            #     #                      f"Queue size: {queue_stats['current_queue_size']}")
                
            # Update leader_state for fallback (preserve original structure)
            if sender_id == self.following_target and 'pos' in received_state :
                self.leader_state = received_state.copy()  # Make a copy to preserve original , Leader here mean the vehicle we follow
                self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Updated leader_state fallback")

            # Feed received state to distributed observer to use (if available)
            if self.observer is not None and sender_id >= 0:
                try:
                    # Convert to observer format but preserve all original data
                    if len(pos) >= 2:
                        state_array = np.array([
                            float(pos[0]),  # x
                            float(pos[1]),  # y  
                            float(rot[2]) if len(rot) > 2 else 0.0,  # theta (yaw)
                            float(vel)      # velocity
                        ])
                        
                        # Control input (ensure exactly 2 elements)
                        control_array = np.array([float(control[0]), float(control[1])]) if len(control) >= 2 else np.array([0.0, 0.0])
                        
                        # Add to observer with original timestamp
                        self.observer.add_received_state(
                            sender_id=sender_id,
                            state=state_array,
                            control=control_array,
                            timestamp=timestamp
                        )
                        #--------------- Log: Confirm observer received the correct data

                        # self.observer_logger.info(f"Vehicle {self.vehicle_id}: Added state from vehicle {sender_id} to observer - "
                        #                          f"pos=({pos[0]:.3f}, {pos[1]:.3f}), vel={vel:.3f}, control={control}")
                    else:
                        self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Invalid position data from vehicle {sender_id}: {pos}")
                    
                except Exception as obs_error:
                    self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Error feeding state to observer: {obs_error}")
            
            # ---- TASK 1.4: Update quality tracking and compute innovation
            if hasattr(self, 'neighbor_quality') and sender_id in self.neighbor_quality:
                current_time = time.time()
                q = self.neighbor_quality[sender_id]
                
                # Update receive timing
                q['last_recv_time'] = current_time
                q['recv_count'] += 1
                
                # Extract covariance if available
                if 'covariance_trace' in received_state:
                    q['last_covariance'] = received_state['covariance_trace']
                    # VERIFICATION LOG: Covariance extracted successfully (safely handle None)
                    cov_val = q['last_covariance']
                    cov_str = f"{cov_val:.4f}" if cov_val is not None else "None"
                    self.comm_logger.debug(f"[VERIFY] V{self.vehicle_id}: Extracted covariance={cov_str} from V{sender_id}")
                else:
                    # VERIFICATION LOG: Covariance missing
                    self.comm_logger.warning(f"[VERIFY] V{self.vehicle_id}: No covariance_trace in message from V{sender_id}")
                
                # Compute innovation (prediction error) BEFORE updating last_state
                try:
                    predicted_state = self._predict_neighbor_state(sender_id, current_time)
                    
                    if predicted_state is not None:
                        # Position innovation - use already extracted pos variable
                        recv_pos = np.array(pos[:2])  # [x, y]
                        pred_pos = np.array(predicted_state['pos'][:2])
                        position_innovation = np.linalg.norm(recv_pos - pred_pos)
                        
                        # Velocity innovation (if available) - use already extracted vel variable
                        velocity_innovation = abs(vel - predicted_state['vel']) if vel is not None else 0.0
                        
                        # Combined innovation (position weighted more heavily)
                        total_innovation = position_innovation + 0.5 * velocity_innovation
                        
                        # Store in recent history
                        q['recent_innovations'].append(total_innovation)
                        
                        # # VERIFICATION LOG: Innovation computed and stored
                        # innov_buffer_len = len(q['recent_innovations'])
                        # self.comm_logger.info(
                        #     f"[VERIFY] V{self.vehicle_id}: Innovation from V{sender_id}: "
                        #     f"pos={position_innovation:.3f}m, vel={velocity_innovation:.3f}m/s, "
                        #     f"total={total_innovation:.3f}, buffer_size={innov_buffer_len}/10"
                        # )
                        
                        # Log large innovations (potential anomaly)
                        if position_innovation > 2.0:  # 2m threshold
                            self.comm_logger.warning(
                                f"Vehicle {self.vehicle_id}: Large innovation from V{sender_id}: "
                                f"{position_innovation:.3f}m. Received: ({recv_pos[0]:.2f}, {recv_pos[1]:.2f}), "
                                f"Predicted: ({pred_pos[0]:.2f}, {pred_pos[1]:.2f})"
                            )
                
                except Exception as innov_error:
                    self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Failed to compute innovation for V{sender_id}: {innov_error}")
                
                # Store received state for NEXT prediction (after innovation computation)
                q['last_state'] = {
                    'pos': pos[:2] if len(pos) >= 2 else [0.0, 0.0],
                    'rot': rot,
                    'vel': vel,
                    'timestamp': timestamp
                }
                # ---- End TASK 1.4:
                # Buffer state for trust evaluation (no immediate calculation)
                if self.trust_enabled and sender_id != self.vehicle_id:
                    if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
                        # Get relative distance from sensor fusion if available
                        relative_distance = None
                        if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None:
                            relative_distance = self.cam_lidar_fusion.get_closest_distance()
                        
                        # Buffer the state (evaluation happens at fixed intervals)
                        self.trust_evaluator.buffer_local_state(
                            sender_id=sender_id,
                            local_state=received_state,
                            relative_distance=relative_distance,
                            timestamp=timestamp
                        )
                        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Buffered local state from V{sender_id} for trust eval")
                    else:
                        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Cannot buffer - trust_evaluator not available")
                else:
                    if not self.trust_enabled:
                        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Trust not enabled, skipping buffer")
                    elif sender_id == self.vehicle_id:
                        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Skipping own state buffer")
                    
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Error handling vehicle state: {e}")
            # Log the full received_state for debugging
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: ERROR_DATA_DUMP: {received_state}")

    #endregion Communication Methods




    # --------------------- Trust Evaluation Methods ---------------------
    ############ Trust Evaluation Methods ############
    #region Trust Evaluation Methods
    
    # NOTE: Trust evaluation is now handled via buffering + periodic updates
    # States are buffered in _handle_vehicle_state_direct() and _handle_fleet_estimates_direct()
    # Trust is calculated periodically in run() main loop via update_all_trust_scores()

    def get_trust_score(self, vehicle_id: int) -> float:
        """
        Get current trust score for a vehicle.
        
        Args:
            vehicle_id: ID of the vehicle
            
        Returns:
            Trust score (0.0 to 1.0), defaults to 1.0 if trust evaluation disabled
        """
        if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
            return self.trust_evaluator.get_trust_score(vehicle_id)
        return 1.0

    def get_connected_vehicle_trust_scores(self) -> Dict[int, float]:
        """
        Get current trust scores for all connected vehicles.
        
        Returns:
            Dictionary mapping vehicle_id to trust_score for connected vehicles
        """
        if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
            return self.trust_evaluator.get_all_trust_scores()
        return {}

    def is_vehicle_trusted(self, vehicle_id: int, threshold: float = 0.5) -> bool:
        """
        Check if a connected vehicle is trusted above a threshold.
        
        Args:
            vehicle_id: ID of the vehicle to check
            threshold: Trust threshold (0.0 to 1.0)
            
        Returns:
            True if vehicle is connected and trusted above threshold
        """
        if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
            return self.trust_evaluator.is_vehicle_trusted(vehicle_id, threshold)
        return True  # Default to trusted if trust evaluation disabled

    def _adjust_control_based_on_trust(self, target_id: int, trust_score: float):
        """
        Legacy method - replaced by GraphBasedTrustEvaluator.
        Kept for compatibility.
        """
        pass

    # Simplified trust methods - most functionality moved to GraphBasedTrustEvaluator
    

    def get_current_trust_weights(self) -> Dict[int, float]:
        """
        Get current trust weights for connected vehicles.
        """
        if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
            return self.trust_evaluator.get_all_trust_scores()
        return {}


    def get_trust_data(self, target_vehicle_id: int) -> Optional[Dict[str, float]]:
        """Get trust data for a vehicle."""
        if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
            trust_score = self.trust_evaluator.get_trust_score(target_vehicle_id)
            return {
                'final_score': trust_score,
                'trust_score': trust_score
            }
        return None

    def evaluate_trust_for_target(self, target_vehicle_id: int, neighbors_list: List[Any] = None, 
                                 instant_idx: int = None) -> Optional[Dict[str, float]]:
        """Legacy method - replaced by graph-based evaluation."""
        return self.get_trust_data(target_vehicle_id)

    #endregion Trust Evaluation Methods



   



    # --------------------- Helper / Refactor Methods ---------------------
    #region Helper Methods 

    def _ensure_fleet_state_store(self):
        """Ensure the fleet state estimates dict exists."""
        if not hasattr(self, 'fleet_state_estimates') or self.fleet_state_estimates is None:
            self.fleet_state_estimates = {}

    def _set_fleet_vehicle_estimate(self, vehicle_idx: int, vehicle_state: np.ndarray, timestamp: float):
        """Refactored helper: store per-vehicle distributed observer estimate from numpy array."""
        self._ensure_fleet_state_store()
        try:
            self.fleet_state_estimates[vehicle_idx] = {
                'pos': [float(vehicle_state[0]), float(vehicle_state[1]), 0.0],
                'rot': [0.0, 0.0, float(vehicle_state[2])],
                'vel': float(vehicle_state[3]),
                'timestamp': timestamp
            }
        except Exception:
            # Silent fail to avoid impacting real-time loop
            pass

    def _build_fleet_estimates_message(self, timestamp: float) -> dict:
        """Create the fleet estimates message dict (refactored, single authoritative builder)."""
        if not hasattr(self, 'fleet_state_estimates') or not self.fleet_state_estimates:
            return {}
        msg = {
            'msg_type': 'fleet_estimates',
            'sender_id': self.vehicle_id,
            'timestamp': timestamp,
            'fleet_size': len(self.fleet_state_estimates),
            'estimates': {}
        }
        for vid, state in self.fleet_state_estimates.items():
            msg['estimates'][vid] = {
                'pos': state['pos'][:2],
                'rot': state['rot'],
                'vel': state['vel'],
                'timestamp': state['timestamp']
            }
        return msg

    def get_observer_state_direct(self) -> Optional[dict]:
        """
        Get state directly from observer without caching.
        
        Returns:
            Observer state dictionary with essential fields only: pos, rot, vel
        """
        if self.observer is None:
            return None
            
        try:
            # Get the local state directly from observer (more efficient)
            local_state = self.observer.get_local_state()
            if local_state is not None and len(local_state) >= 4:
                return {
                    'pos': [local_state[0], local_state[1], 0.0],  # [x, y, z] format
                    'rot': [0.0, 0.0, local_state[2]],  # [roll, pitch, yaw] format
                    'vel': local_state[3]
                }
            else:
                # Fallback to the formatted method if local state is not available
                return self.observer.get_estimated_state_for_control()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Failed to get observer state: {e}")
            return None

    def get_best_available_state(self) -> dict:
        """
        Factory method that returns the best available state estimate (OUR LOCAL STATE).
        Priority: Observer EKF > Raw GPS fallback
        
        Returns:
            Best available state estimate with essential fields: pos, rot, vel
        """
        # Try observer first (check if EKF is initialized through observer attributes)
        if self.observer is not None:
            observer_state = self.get_observer_state_direct()
            if observer_state and hasattr(self.observer, 'ekf_initialized') and self.observer.ekf_initialized:
                return observer_state
        
        # Fallback to raw GPS if observer not available or EKF not initialized
        return {
            'pos': self.current_pos.copy(),
            'rot': self.current_rot.copy(),
            'vel': self.velocity
        }

    def _predict_neighbor_state(self, neighbor_id: int, target_time: float) -> Optional[dict]:
        """
        TASK 1.3: Predict neighbor's state at target_time using constant velocity model.
        Used for computing innovation (prediction error) for quality tracking.
        
        Args:
            neighbor_id: ID of neighbor vehicle
            target_time: Time to predict state at
            
        Returns:
            Predicted state dict or None if no previous state available
        """
        if neighbor_id not in self.neighbor_quality:
            return None
        
        last_state = self.neighbor_quality[neighbor_id].get('last_state')
        if last_state is None:
            # No previous state stored, cannot predict
            return None
        
        # Time since last state
        dt = target_time - last_state.get('timestamp', target_time)
        dt = max(0.0, min(dt, 1.0))  # Clamp to [0, 1] second for safety
        
        # Extract state components (handle both formats)
        if 'pos' in last_state:
            # Format: {pos: [x, y], rot: [roll, pitch, yaw], vel: v}
            x, y = last_state['pos'][0], last_state['pos'][1]
            if 'rot' in last_state and last_state['rot'] is not None:
                if len(last_state['rot']) >= 3:
                    theta = last_state['rot'][2]  # yaw angle is at index 2
                else:
                    theta = last_state['rot'][0] if len(last_state['rot']) > 0 else 0.0
            else:
                theta = 0.0
            v = last_state.get('vel', last_state.get('v', 0.0))
        else:
            # Format: {x: x, y: y, theta: theta, v: v}
            x = last_state.get('x', 0.0)
            y = last_state.get('y', 0.0)
            theta = last_state.get('theta', 0.0)
            v = last_state.get('v', 0.0)
        
        # Constant velocity prediction (simple kinematic model)
        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt
        theta_pred = theta  # Assume constant heading
        v_pred = v  # Assume constant velocity
        
        return {
            'pos': [x_pred, y_pred],
            'rot': [theta_pred],
            'vel': v_pred,
            'timestamp': target_time
        }

    def get_fleet_estimates(self) -> dict:
        """
        Get OUR distributed observer's fleet state estimates for ALL vehicles.
        This is what our observer thinks about the entire fleet's states.
        
        Returns:
            Dictionary mapping vehicle_id -> state dict with pos, rot, vel, timestamp
            Returns empty dict if no fleet estimates available
        """
        if hasattr(self, 'fleet_state_estimates') and self.fleet_state_estimates:
            return self.fleet_state_estimates.copy()
        return {}

    def get_quality_metrics_for_vehicle(self, vehicle_id: int) -> Optional[Dict]:
        """
        Get quality metrics for a specific vehicle.
        Used by trust evaluator to enhance trust calculation.
        
        Args:
            vehicle_id: Target vehicle ID
            
        Returns:
            Dictionary with quality metrics or None if not available
        """
        # VERIFICATION LOG: Check if neighbor_quality exists
        if not hasattr(self, 'neighbor_quality'):
            self.comm_logger.warning(f"[VERIFY] V{self.vehicle_id}: neighbor_quality attribute missing!")
            return None
            
        if vehicle_id not in self.neighbor_quality:
            self.comm_logger.warning(f"[VERIFY] V{self.vehicle_id}: Vehicle {vehicle_id} not in neighbor_quality dict")
            return None
        
        q = self.neighbor_quality[vehicle_id]
        current_time = time.time()
        
        # 1. Message Age
        age = current_time - q['last_recv_time'] if q['last_recv_time'] > 0 else 1.0
        
        # 2. Drop Rate
        drop_rate = self._compute_drop_rate(vehicle_id)
        
        # 3. Covariance
        covariance = q.get('last_covariance', 1.0)
        if covariance is None or np.isnan(covariance):
            covariance = 1.0
        
        # 4. Innovation (mean of recent values)
        innov_buffer_size = len(q['recent_innovations'])
        if innov_buffer_size > 0:
            innovation = np.mean(q['recent_innovations'])
            # Safety check for NaN
            if innovation is None or np.isnan(innovation):
                innovation = 0.0
        else:
            innovation = 0.0
        
        # VERIFICATION LOG: Quality metrics computed
        self.comm_logger.debug(
            f"[VERIFY] V{self.vehicle_id}: Quality metrics for V{vehicle_id}: "
            f"age={age:.3f}s, drop_rate={drop_rate:.3f}, cov={covariance:.4f}, "
            f"innovation={innovation:.3f}m (from {innov_buffer_size} samples)"
        )
        
        return {
            'age': age,
            'drop_rate': drop_rate,
            'covariance': covariance,
            'innovation': innovation
        }
    
    def _compute_drop_rate(self, vehicle_id: int) -> float:
        """Compute packet drop rate for a vehicle."""
        if vehicle_id not in self.neighbor_quality:
            return 0.5
        
        q = self.neighbor_quality[vehicle_id]
        elapsed = time.time() - self.comm_start_time
        
        if elapsed > 1.0:  # After 1 second of operation
            expected = elapsed * (self.update_rate / 10)  # Approximate broadcast rate
            actual = q['recv_count']
            return max(0.0, min(1.0, (expected - actual) / max(expected, 1)))
        
        return 0.0

    def get_state_for_control(self) -> dict:
        """Get current vehicle state for control algorithms using observer estimates."""
        return self.get_best_available_state()




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
            # First stop the running flag to exit the main loop
            self.running.clear()


            # CRITICAL: Stop the vehicle immediately with zero commands
            try:
                # self.physical_qcar.write(0, 0)
                self.physical_qcar.read_write_std(0, 0)
                print(f"Vehicle {self.vehicle_id}: Emergency stop - Physical QCar zero commands sent")
            except Exception as e:
                print(f"Vehicle {self.vehicle_id}: Error during emergency stop: {e}")
            

            
            # Clean up controllers safely
            if self.leader_controller is not None:
                self.leader_controller.stop_control()
            if self.follower_controller is not None:
                self.follower_controller.stop_control()


            # Stop communication thread
            self.stop_communication_thread()
            
            # Clean up communication
            if hasattr(self, 'comm') and self.comm is not None:
                self.comm.cleanup()

                

            # CamLidarFusion thread cleanup removed (moved to main loop)
            if not IS_PHYSICAL_QCAR:
                cmd = QLabsRealTime().terminate_all_real_time_models()
                time.sleep(1)

            # if self.qcar is not None:
            #     # Send zero velocity command to stop the vehicle
            #     self.qcar.set_velocity_and_request_state(
            #             forward=0.0, 
            #             turn=0.0,
            #             headlights=False,
            #             leftTurnSignal=False,
            #             rightTurnSignal=False,
            #             brakeSignal=False,
            #             reverseSignal=False
            #         )            
            # # Close QLabs connection
            # try:
            #     if self.qlabs is not None:
            #         self.qlabs.close()
            #         print(f"Vehicle {self.vehicle_id}: QLabs connection closed")
            # except Exception as e:
            #     print(f"Vehicle {self.vehicle_id}: Error closing QLabs: {e}")

            # Clean up logging handlers to ensure async threads are properly closed
            from md_logging_config import cleanup_all_logging
            cleanup_all_logging()
            
                        
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")
            print(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")

    def stop_communication_thread(self):
        """Stop the communication thread."""
        if self.comm_thread is not None:
            self.comm_thread_running = False
            self.comm_thread.join(timeout=1.0)
            self.comm_thread = None
            self.logger.info(f"Vehicle {self.vehicle_id}: Communication thread stopped")
    
