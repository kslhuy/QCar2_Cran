import os
import time
import numpy as np
import logging
from typing import Any, Optional
from qvl.real_time import QLabsRealTime

from src.OpenRoad import OpenRoad
from hal.products.mats import SDCSRoadMap
from src.Controller.ControllerLeader import SpeedController, SteeringController

# Physical QCar imports for EKF-based control
try:
    from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
    from hal.content.qcar_functions import QCarEKF
    from pal.utilities.math import wrap_to_pi
    PHYSICAL_QCAR_AVAILABLE = True
except ImportError:
    PHYSICAL_QCAR_AVAILABLE = False
    QCar = None
    QCarGPS = None
    QCarEKF = None
    IS_PHYSICAL_QCAR = False
    wrap_to_pi = None


class VehicleLeaderController:
    """
    Dedicated controller for leader vehicles that encapsulates all the complex
    leader control logic from ControlLeader class.
    """

    def __init__(self, vehicle_id: int, Qcar_RT: None, config=None, logger=None):
        """
        Initialize the leader controller.
        
        Args:
            vehicle_id: ID of the vehicle this controller manages
            config: Configuration object containing parameters
            logger: Logger instance for this controller
        """
        self.vehicle_id = vehicle_id
        self.config = config
        self.logger = logger or logging.getLogger(f"LeaderController_{vehicle_id}")
        
        # Setup dedicated waypoint logger
        self.waypoint_logger = self._setup_waypoint_logger()
        
        # Control parameters (from ControlLeader)
        self.controllerUpdateRate = 100
        self.K_p = 0.1
        self.K_i = 0.08
        self.enableSteeringControl = config.get('enable_steering_control', True) if config else True
        self.K_stanley = 0.8
        self.startDelay = 1
        self.calibrate = config.get('calibrate', True) if config else True

        
        # Control state
        self.start_time = None
        self.delta = 0  # Steering command
        self.u = 0      # Throttle command
        self.max_steering = config.get('max_steering', 0.6) if config else 0.6
        
        # Path and controllers
        self.waypointSequence = None
        self.InitialPose = None
        self.speedController = None
        self.steeringController = None


        # Initialize components
        self._init_path_and_controllers()
        
        # Initialize physical QCar interface and EKF components
        self.qcar = None
        self.ekf = None
        self.gps = None
        self.use_control_observer_mode = config.get('use_control_observer_mode', False) if config else False
        print(f"Vehicle {self.vehicle_id}: use_control_observer_mode = {self.use_control_observer_mode}")

        if Qcar_RT is not None and not self.use_control_observer_mode:
            self.qcar = Qcar_RT
            self.logger.info(f"Vehicle {self.vehicle_id}: Using provided QCar real-time interface")

        elif self.use_control_observer_mode and PHYSICAL_QCAR_AVAILABLE and Qcar_RT is None:
            try:
                self.rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
                QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=vehicle_id)
                # Initialize physical QCar interface
                self.qcar = QCar(readMode=1, frequency=self.controllerUpdateRate)
                
                # Initialize EKF and GPS if steering control is enabled
                if self.enableSteeringControl or self.calibrate:
                    # Use the InitialPose that was generated in _init_path_and_controllers
                    initial_pose = self.InitialPose if self.InitialPose is not None else [0, 0, 0]
                    calibration_pose = config.get('calibration_pose', [0, 2, -np.pi/2]) if config else [0, 2, -np.pi/2]
                    
                    self.ekf = QCarEKF(x_0=initial_pose)
                    self.gps = QCarGPS(initialPose=calibration_pose, calibrate=self.calibrate)
                    self.logger.info(f"Vehicle {self.vehicle_id}: Initialized EKF with initial pose {initial_pose} and GPS for observer mode")
                else:
                    self.gps = memoryview(b'')
                    self.logger.info(f"Vehicle {self.vehicle_id}: Initialized QCar without GPS")
                    
            except Exception as e:
                self.logger.error(f"Vehicle {self.vehicle_id}: Failed to initialize observer mode: {e}")
                self.use_control_observer_mode = False
                self.qcar = None
                self.ekf = None
                self.gps = None
        # else:
        #     # For process-based architecture or when observer mode disabled, 
        #     # don't create separate QLabsRealTime/QCar - these are managed by VehicleProcess
        #     if not self.use_control_observer_mode:
        #         try:
        #             self.rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
        #             self.LeaderIndex = config.get('vehicle_id', 0) if config else 0
        #             QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=self.LeaderIndex)
        #             self.qcar = QCar(readMode=1, frequency=self.controllerUpdateRate)
        #         except Exception as e:
        #             self.logger.warning(f"Failed to initialize QLabsRealTime: {e}")
        #             self.qcar = None

        

        
        self.logger.info(f"Leader controller initialized for vehicle {vehicle_id}")
    
    def _setup_waypoint_logger(self):
        """Setup dedicated logger for waypoint information."""
        waypoint_logger = logging.getLogger(f"Waypoint_Vehicle_{self.vehicle_id}")
        waypoint_logger.setLevel(logging.INFO)
        
        # Create file handler for waypoint log
        log_file = f"logs/waypoint_vehicle_{self.vehicle_id}.log"
        os.makedirs("logs", exist_ok=True)
        
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.INFO)
        
        # Create formatter
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        
        # Add handler if not already added
        if not waypoint_logger.handlers:
            waypoint_logger.addHandler(file_handler)
        
        return waypoint_logger
    
    def _init_path_and_controllers(self):
        """Initialize path generation and controllers."""
        # Get configuration parameters
        road_type = self.config.get('road_type', 'OpenRoad') if self.config else "OpenRoad"
        node_sequence = self.config.get('node_sequence', [0, 1]) if self.config else [0, 1]
        
        # print(f"Vehicle {self.vehicle_id}: Initializing path with road_type={road_type}, node_sequence={node_sequence}")
        
        # Generate path
        self.waypointSequence, self.InitialPose = self._generate_path(road_type, node_sequence, enable_logging=False)

        # print(f"Vehicle {self.vehicle_id}: Generated {len(self.waypointSequence) if self.waypointSequence is not None else 0} waypoints")
        # if self.waypointSequence is not None and len(self.waypointSequence) > 0:
        #     print(f"Vehicle {self.vehicle_id}: First waypoint: {self.waypointSequence[0] if hasattr(self.waypointSequence, '__getitem__') else 'N/A'}")
        #     print(f"Vehicle {self.vehicle_id}: Initial pose: {self.InitialPose}")

        # Initialize speed controller
        self.speedController = SpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        
        # Initialize steering controller
        if self.enableSteeringControl:
            self.steeringController = SteeringController(
                waypoints=self.waypointSequence,
                k=self.K_stanley
            )
        
        self.logger.info(f"Generated path with {len(self.waypointSequence)} waypoints")
    
    def _generate_path(self, road_type: str, node_sequence: list , enable_logging:bool = False):
        """Generate path for leader vehicle (from ControlLeader.PathGeneration)."""
        if road_type == "OpenRoad":
            roadmap = OpenRoad()
        elif road_type == "Studio":
            roadmap = SDCSRoadMap()
        else:
            raise ValueError(f"Unknown road type: {road_type}")
        
        waypointSequence = roadmap.generate_path(node_sequence)
        InitialPose = roadmap.get_node_pose(node_sequence[0]).squeeze()
        
        if (enable_logging := True):
            # Log to dedicated waypoint log file with full details
            self.waypoint_logger.info(f"Vehicle {self.vehicle_id} - Road Type: {road_type}")
            self.waypoint_logger.info(f"Vehicle {self.vehicle_id} - Node Sequence: {node_sequence}")
            
            # Log full waypoint sequence details
            self.waypoint_logger.info(f"Vehicle {self.vehicle_id} - Total Waypoints: {waypointSequence.shape[1] if len(waypointSequence.shape) > 1 else len(waypointSequence)}")
            
            # Log each waypoint individually for full detail
            if len(waypointSequence.shape) > 1:
                # 2D array: waypointSequence[0] = x coordinates, waypointSequence[1] = y coordinates
                for i in range(waypointSequence.shape[1]):
                    x = waypointSequence[0, i]
                    y = waypointSequence[1, i]
                    self.waypoint_logger.info(f"Vehicle {self.vehicle_id} - Waypoint {i}: x={x:.6f}, y={y:.6f}")
            else:
                # 1D array
                for i, waypoint in enumerate(waypointSequence):
                    self.waypoint_logger.info(f"Vehicle {self.vehicle_id} - Waypoint {i}: {waypoint}")
            self.waypoint_logger.info(f"Vehicle {self.vehicle_id} - Initial Pose: x={InitialPose[0]:.6f}, y={InitialPose[1]:.6f}, theta={InitialPose[2]:.6f}")
        
        
        return waypointSequence, InitialPose
    
    def _get_vref(self, t):
        """Get reference velocity based on time (from ControlLeader.vref)."""
        road_type = self.config.get('road_type', 'OpenRoad') if self.config else "OpenRoad"
        
        if road_type == "OpenRoad":
            if t < 5:
                v_ref = 2
            elif t < 10:
                v_ref = 0
            elif t < 15:
                v_ref = -0.5
            elif t < 20:
                v_ref = 1
            else:
                v_ref = 2
            return v_ref
        elif road_type == "Studio":
            return 0.3
        else:
            return 0.3
    
    def start_control(self, dt=0.1):
        """Start the control session (call this when vehicle starts)."""
        if self.start_time is None:
            self.start_time = time.time()
            self.logger.info("Leader control session started")


    def compute_control_w_observer(self, dt=0.1):
        """
        Compute control commands using physical QCar with EKF observer (like vehicle_control.py).
        This method reads sensors directly and uses EKF for state estimation.
        
        Args:
            dt: Time step for control update
            
        Returns:
            tuple: (forward_speed, steering_angle) control commands
        """
        if not PHYSICAL_QCAR_AVAILABLE:
            self.logger.error("Physical QCar API not available, falling back to regular control")
            return 0.0, 0.0
            
        if self.start_time is None:
            self.start_control(dt)
        
        # Calculate elapsed time
        t = time.time() - self.start_time
        
        try:
            # Read from sensors (like vehicle_control.py)
            self.qcar.read()
            
            if self.enableSteeringControl:
                # Try to read GPS
                if hasattr(self, 'gps') and self.gps and hasattr(self.gps, 'readGPS'):
                    if self.gps.readGPS():
                        # GPS data available - update EKF with GPS
                        y_gps = np.array([
                            self.gps.position[0],
                            self.gps.position[1],
                            self.gps.orientation[2]
                        ])
                        # print("EKF update")
                        self.ekf.update(
                            [self.qcar.motorTach, self.delta],
                            dt,
                            y_gps,
                            self.qcar.gyroscope[2],
                        )
                    else:
                        # No GPS data - update EKF without GPS
                        self.ekf.update(
                            [self.qcar.motorTach, self.delta],
                            dt,
                            None,
                            self.qcar.gyroscope[2],
                        )
                
                # Extract state from EKF (like vehicle_control.py)
                if hasattr(self, 'ekf') and self.ekf:
                    x = self.ekf.x_hat[0,0]
                    y = self.ekf.x_hat[1,0]
                    th = self.ekf.x_hat[2,0]
                    
                    # Position for steering controller (with look-ahead point)
                    p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
                else:
                    # Fallback if EKF not available
                    self.logger.warning("EKF not available, using zero position")
                    x, y, th = 0.0, 0.0, 0.0
                    p = np.array([x, y])
            else:
                # No steering control
                x, y, th = 0.0, 0.0, 0.0
                p = np.array([x, y])
            
            # Get velocity from motor tachometer
            v = self.qcar.motorTach

            # print(self.qcar.gyroscope[2] , "gyro z and delta " + str(self.delta))
            
            # Get reference velocity
            vref = self._get_vref(t)
            
            # Control logic (similar to vehicle_control.py)
            if t < self.startDelay:
                self.u = 0
                self.delta = 0
            else:
                # Speed control
                self.u = self.speedController.update(v, vref, dt)
                
                # Steering control
                if self.enableSteeringControl and self.steeringController:
                    self.delta = self.steeringController.update(p, th, v)
                else:
                    self.delta = 0
            
            # Convert and clamp commands
            forward_speed = self.u
            steering_angle = self.delta
            
            # Clamp to safe ranges
            forward_speed = max(-2.0, min(2.0, forward_speed))
            steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
            
            # Apply control commands to QCar (like vehicle_control.py)
            self.qcar.write(forward_speed, steering_angle)
            
            return forward_speed, steering_angle
            
        except Exception as e:
            self.logger.error(f"Error in compute_control_w_observer: {e}")
            return 0.0, 0.0

    
    def compute_control(self, current_pos, current_rot, velocity, dt=0.1):
        """
        Compute control commands based on current vehicle state.
        Uses the provided state directly since local observer handles state estimation.
        
        Args:
            current_pos: Current position [x, y, z] from Vehicle's observer
            current_rot: Current rotation [roll, pitch, yaw] from Vehicle's observer  
            velocity: Current velocity from Vehicle's observer
            dt: Time step for control update
            
        Returns:
            tuple: (forward_speed, steering_angle) control commands
        """
        if self.start_time is None:
            self.start_control(dt)
        
        # Calculate elapsed time
        t = time.time() - self.start_time
        
        # Get reference velocity
        vref = self._get_vref(t)
        
        # Extract position and orientation directly from provided state
        x, y = current_pos[0], current_pos[1]
        th = current_rot[2]  # Heading in radians
        
        # Position for steering controller (with look-ahead point)
        p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
        
        # Use provided velocity directly
        v = velocity
        
        # Control logic (from ControlLeader.run())
        if t < self.startDelay:
            self.u = 0
            self.delta = 0
            # if self.vehicle_id == 0:  # Debug for leader
            #     print(f"Leader in start delay: t={t:.3f}, delay={self.startDelay}")
        else:
            # Speed control
            self.u = self.speedController.update(v, vref, dt)
            
            # Steering control
            if self.enableSteeringControl:
                self.delta = self.steeringController.update(p, th, v)
                # if self.vehicle_id == 0:  # Debug for leader
                #     print(f"Leader steering: pos=({p[0]:.3f}, {p[1]:.3f}), heading={th:.3f}, "
                #           f"velocity={v:.3f}, vref={vref:.3f}, steering={self.delta:.3f}")
            else:
                self.delta = 0
        
        # Convert and clamp commands
        forward_speed = self.u
        steering_angle = self.delta
        
        # Clamp to safe ranges
        forward_speed = max(-2.0, min(2.0, forward_speed))
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
        
        # For process-based architecture, don't apply commands here
        # VehicleProcess will handle applying commands to its QCar instance
        # self.qcar.write(forward_speed, steering_angle)
        
        return forward_speed, steering_angle
    
    def compute_control_auto(self, current_pos=None, current_rot=None, velocity=None, dt=0.1):
        """
        Automatically choose between observer mode and external state mode.
        
        Args:
            current_pos: Current position [x, y, z] (optional, used for external state mode)
            current_rot: Current rotation [roll, pitch, yaw] (optional, used for external state mode)
            velocity: Current velocity (optional, used for external state mode)
            dt: Time step for control update
            
        Returns:
            tuple: (forward_speed, steering_angle) control commands
        """
        if self.use_control_observer_mode and PHYSICAL_QCAR_AVAILABLE:
            # Use observer mode with EKF
            return self.compute_control_w_observer(dt)
        else:
            # Use external state mode (requires state parameters)
            if current_pos is None or current_rot is None or velocity is None:
                self.logger.warning("External state mode requires position, rotation, and velocity parameters")
                return 0.0, 0.0
            return self.compute_control(current_pos, current_rot, velocity, dt)
    
    def get_control_state(self):
        """Get current control state information."""
        if self.start_time is None:
            return {
                'initialized': False,
                'elapsed_time': 0,
                'throttle_command': 0,
                'steering_command': 0,
                'reference_velocity': 0,
                'waypoint_count': len(self.waypointSequence) if self.waypointSequence is not None else 0
            }
        
        elapsed_time = time.time() - self.start_time
        return {
            'initialized': True,
            'elapsed_time': elapsed_time,
            'throttle_command': self.u,
            'steering_command': self.delta,
            'reference_velocity': self._get_vref(elapsed_time),
            'waypoint_count': len(self.waypointSequence) if self.waypointSequence is not None else 0
        }
    
    def stop_control(self):
        """Stop the control session and cleanup."""
        self.start_time = None
        
        # Stop QCar immediately based on mode
        if self.use_control_observer_mode and self.qcar is not None:
            # In observer mode, we control QCar directly - STOP IMMEDIATELY
            try:
                self.qcar.write(0, 0)
                print(f"Vehicle {self.vehicle_id}: QCar stopped with zero commands (control observer mode)")
            except Exception as e:
                self.logger.error(f"Error stopping QCar: {e}")
        
        # Reset control commands to zero
        self.u = 0
        self.delta = 0
        
        self.logger.info("Leader control session stopped")
    
    def cleanup(self):
        """Cleanup resources."""
        self.stop_control()
        
        # Close QCar connections if in observer mode
        if self.use_control_observer_mode and self.qcar is not None:
            try:
                # Close QCar and GPS resources
                if hasattr(self.qcar, 'close'):
                    self.qcar.close()
                if hasattr(self.gps, 'close') and callable(self.gps.close):
                    self.gps.close()
            except Exception as e:
                self.logger.error(f"Error during cleanup: {e}")
        
        self.logger.info("Leader controller cleanup completed")
    
    def reset_control(self):
        """Reset the control session (restart timing)."""
        self.stop_control()
        self.start_control()
        self.logger.info("Leader control session reset")
