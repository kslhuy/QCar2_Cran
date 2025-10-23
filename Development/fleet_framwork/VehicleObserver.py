import numpy as np
import threading
import time
import logging
import re
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
from hal.content.qcar_functions import QCarEKF
from md_logging_config import get_observer_logger, get_fleet_observer_logger, get_weight_logger
from DataLogger import ObserverDataLogger


class VehicleObserver:
    """
    Observer system for vehicles in a fleet, providing both local state estimation
    and distributed state estimation for all vehicles in the fleet.
    
    Inspired by the observer.py implementation with Kalman filtering and distributed
    observer capabilities.
    """

    def __init__(self, vehicle_id: int, fleet_size: int, config=None, logger=None, initial_pose=None, 
                 vehicle_process=None, weight_trust_module=None):
        """
        Initialize the Vehicle Observer.
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in the fleet
            config: Configuration object containing observer parameters
            logger: Logger instance for this observer
            initial_pose: Initial pose [x, y, theta] for the vehicle
            vehicle_process: Reference to VehicleProcess for trust integration (Week 3)
            weight_trust_module: WeightTrustModule instance for adaptive weights (Week 3)
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.config = config
        
        # Week 3: References for trust-based adaptive weights
        self.vehicle_process = vehicle_process
        self.weight_trust_module = weight_trust_module
        self.use_trust_weights = (weight_trust_module is not None and vehicle_process is not None)
        # Use specialized observer logger with timing precision
        self.logger = logger if logger is not None else get_observer_logger(vehicle_id)
        # Add dedicated fleet observer logger for fleet estimation data
        self.fleet_logger = get_fleet_observer_logger(vehicle_id)
        # Add dedicated weight logger for adaptive weight tracking (Week 3)
        self.weight_logger = get_weight_logger(vehicle_id)
        
        # Initialize data logger for plotting and MATLAB export
        # Check config for data logging preferences
        create_new_run = config.get('data_logging', {}).get('create_new_run', True) if config else True
        custom_run_name = config.get('data_logging', {}).get('custom_run_name', None) if config else None
        

        from DataLogger import ObserverDataLogger
        self.data_logger = ObserverDataLogger(
            vehicle_id, fleet_size, 
            log_dir="data_logs", 
            create_new_run=create_new_run,
            custom_run_name=custom_run_name
        )

        
        # State dimensions: [x, y, theta, v] - position, orientation, velocity
        self.state_dim = 4
        self.control_dim = 2  # [steering, acceleration]
        
        # Observer configuration
        self.observer_config = self._get_observer_config()



        # Local state estimation
        self.local_state = np.zeros(self.state_dim)
        self.local_state_prev = np.zeros(self.state_dim)
        self.local_state[:3] = initial_pose if initial_pose is not None else [0.0, 0.0, 0.0]  # Set x, y, theta
        self.local_state[3] = 0.0  # Initialize velocity to 0        

        # Distributed state estimation - estimates for all vehicles
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
        self.fleet_states_prev = np.zeros((self.state_dim, self.fleet_size))
        
        # Initialize ALL vehicles in fleet with host vehicle's state as starting estimate
        # This ensures all vehicles start with reasonable velocity estimates rather than zero
        for i in range(self.fleet_size):
            if i == self.vehicle_id:
                # Initialize own state with the provided initial pose
                self.fleet_states[:, i] = self.local_state
            else:
                # Initialize other vehicles with zero state (will be corrected by distributed observer)
                # This prevents bias from assuming all vehicles start at the same location
                self.fleet_states[:, i] = np.zeros(self.state_dim).copy()
        
        
        # # Ensure the host vehicle's state is set (redundant but explicit)
        # self.fleet_states[:, self.vehicle_id] = self.local_state
        
        # Log initial fleet state setup
        self.fleet_logger.info(f"FLEET_INIT: Initialized fleet with host vehicle {self.vehicle_id} at its pose, "
                              f"others at origin to avoid initial bias")
        for i in range(self.fleet_size):
            state = self.fleet_states[:, i]
            self.fleet_logger.info(f"FLEET_INIT_V{i}: pos=({state[0]:.3f},{state[1]:.3f}), "
                                  f"theta={state[2]:.3f}, vel={state[3]:.3f}")
              
        # QCarEKF for local state estimation (inspired by VehicleLeaderController)
        self.ekf = None
        self.qcar_ekf_initialized = False
        
        # Kalman filter matrices for local estimation (fallback if QCarEKF not used)
        if self.observer_config["local_observer_type"] == "kalman":
            if initial_pose is not None:
                self.initialize_ekf(initial_pose=initial_pose)

            self.P_local = np.eye(self.state_dim) * 0.1  # Local covariance matrix
            self.R_local = np.diag([0.01, 0.01, 0.0003, 0.01])  # Measurement noise
            self.Q_local = np.diag([0.005, 0.005, 0.001, 0.01])  # Process noise

        
        # GPS availability tracking
        self.gps_available = False
        self.last_gps_update = 0.0
        
        # Distributed observer parameters
        self.distributed_weights = self._initialize_distributed_weights()
        self.communication_graph = self._initialize_communication_graph()
        
        # Data storage for received states from other vehicles
        self.received_states = defaultdict(list)  # vehicle_id -> list of (timestamp, state)
        self.received_controls = defaultdict(list)  # vehicle_id -> list of (timestamp, control)
        
        # Separate data storage for fleet estimates from distributed observers
        self.received_states_fleet = defaultdict(list)  # vehicle_id -> list of (timestamp, fleet_estimates)
        
        # Timing parameters
        self.dt = 1 / self.config.get("observer_rate", 100)   # 100 Hz default
        self.max_state_age = 0.5  # Maximum age of received states to use (seconds)

        self.max_history = 64  # Smaller history for fleet estimates as they are larger

        
        # Tolerances for state validation
        self.tolerances = np.array([5.0, 2.0, np.deg2rad(8), 2.0])  # [x, y, theta, v]
        
        # Logging and monitoring
        self.estimation_log = []
        self.validation_log = []
        
        # Thread safety
        self.lock = threading.RLock()
        
        self.logger.info(f"VehicleObserver initialized for vehicle {vehicle_id} "
                        f"(Fleet size: {fleet_size}, Observer type: {self.observer_config['local_observer_type']})")
    #region Observer Configuration
    def _get_observer_config(self) -> dict:
        """Get observer configuration with defaults."""
        default_config = {
            "local_observer_type": "kalman",  # "kalman", "luenberger", or "direct"
            "enable_distributed": True,
            "distributed_observer_type": "consensus",
            "enable_noise_measurement": False,
            "enable_prediction": True,
            "consensus_gain": 0.3
        }

        return self.config.get('observer', default_config)

    #endregion

    #region Init Distributed Graph

    def _initialize_distributed_weights(self) -> np.ndarray:
        """Initialize weights for distributed observer consensus."""
        # Simple uniform weighting for now
        weights = np.ones(self.fleet_size + 1) / (self.fleet_size + 1)
        weights[0] = 0.5  # Weight for local measurement
        weights[1:] = 0.5 / self.fleet_size  # Weights for other vehicles
        
        return weights


    def _initialize_communication_graph(self) -> np.ndarray:
        """Initialize communication graph adjacency matrix."""
        # For now, assume all vehicles can communicate with each other
        graph = np.ones((self.fleet_size, self.fleet_size))
        np.fill_diagonal(graph, 0)  # No self-loops
        
        return graph
    #endregion

    #region Local Observer 
    def initialize_ekf(self, initial_pose: Optional[np.ndarray] = None):
        """
        Initialize the QCarEKF for local state estimation.
        
        Args:
            initial_pose: Initial pose [x, y, theta] or None for default [0, 0, 0]
        """
        ekf_init_start = time.perf_counter()
        
        if initial_pose is None:
            initial_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        
        try:
            # Initialize QCarEKF with initial pose (similar to VehicleLeaderController)
            self.ekf = QCarEKF(x_0=initial_pose)
            self.qcar_ekf_initialized = True

            
            ekf_init_time = (time.perf_counter() - ekf_init_start) * 1000
            
            self.logger.info(f"EKF_INIT: Success in {ekf_init_time:.3f}ms, "
                           f"InitialPose=({initial_pose[0]:.5f},{initial_pose[1]:.5f}), "
                           f"InitialTheta={np.rad2deg(initial_pose[2]):.1f}°")
            
            self.logger.info(f"QCarEKF initialized for vehicle {self.vehicle_id} "
                           f"with initial pose: {initial_pose}")
            
        except Exception as e:
            ekf_init_time = (time.perf_counter() - ekf_init_start) * 1000
            self.logger.error(f"EKF_INIT: Failed after {ekf_init_time:.3f}ms - {e}")
            self.logger.error(f"Failed to initialize QCarEKF: {e}")
            self.qcar_ekf_initialized = False
            self.ekf = None
    
    def get_system_matrices(self, state: np.ndarray, control: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get linearized system matrices A and B for the current state.
        
        Args:
            state: Current state [x, y, theta, v]
            control: Current control input [steering, acceleration]
            
        Returns:
            A: State transition matrix
            B: Control input matrix
        """
        x, y, theta, v = state
        dt = self.dt
        
        # Linearized bicycle model matrices
        A = np.array([
            [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt, np.sin(theta) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        B = np.array([
            [0, 0],
            [0, 0],
            [0, dt],  # steering affects heading rate
            [dt, 0]   # acceleration affects velocity
        ])
        
        return A, B
    
    def update_local_state(self, measured_state: Optional[np.ndarray], control_input: np.ndarray, 
                          timestamp: float, motor_tach: float = 0.0, gyroscope_z: float = 99 , acceleration: np.ndarray = None) -> np.ndarray:
        """
        Update local state estimation using QCarEKF or fallback methods.
        
        Args:
            measured_state: GPS measurement [x, y, theta, v] or None if GPS not available
            control_input: Applied control input [steering, acceleration]
            timestamp: GPS-synchronized timestamp
            motor_tach: Motor tachometer reading (velocity measurement)
            gyroscope_z: Z-axis gyroscope reading (angular velocity)
            
        Returns:
            Updated local state estimate
        """
        # Start timing for observer performance analysis
        observer_start_time = time.perf_counter()
        
        with self.lock:
            # Step 1: Time calculation and validation
            dt_calc_start = time.perf_counter()
            if hasattr(self, 'last_update_time'):
                dt = timestamp - self.last_update_time
                dt = max(0.001, min(0.1, dt))  # Clamp dt between 1ms and 100ms
            else:
                dt = self.dt
            self.last_update_time = timestamp
            dt_calc_time = (time.perf_counter() - dt_calc_start) * 1000
            
            # Step 2: Observer method selection and state update
            state_update_start = time.perf_counter()
            
            # Use QCarEKF if initialized and enabled
            if (self.observer_config["local_observer_type"] == "kalman" and 
                self.qcar_ekf_initialized and self.ekf is not None):
                

                self.local_state = self._qcar_ekf_update(
                    measured_state, control_input, dt, motor_tach, gyroscope_z
                )
                method_used = "qcar_ekf"
                
            elif self.observer_config["local_observer_type"] == "kalman":
                # Fallback to custom Kalman filter if QCarEKF not available
                if measured_state is not None:
                    self.local_state = self._kalman_filter_update(measured_state, control_input, dt)
                    method_used = "kalman_update"
                else:
                    # Prediction only
                    self.local_state = self._kalman_prediction_only(control_input, dt)
                    method_used = "kalman_prediction"
                    
            elif self.observer_config["local_observer_type"] == "luenberger":
                if measured_state is not None:
                    self.local_state = self._luenberger_observer_update(measured_state, control_input, dt)
                    method_used = "luenberger_update"
                else:
                    # Prediction only
                    A, B = self.get_system_matrices(self.local_state, control_input)
                    self.local_state = A @ self.local_state + B @ control_input
                    method_used = "luenberger_prediction"
                    
            else:  # direct measurement
                if measured_state is not None:
                    self.local_state = measured_state.copy()
                    # Update velocity from motor tach if available
                    if motor_tach != 0.0:
                        self.local_state[3] = motor_tach
                    method_used = "direct_measurement"
                else:
                    method_used = "direct_hold"
                # If no measurement available, keep previous state

            # if self.vehicle_id == 0 :
            #     print(f"Vehicle {self.vehicle_id}: State updated using {method_used}")

            state_update_time = (time.perf_counter() - state_update_start) * 1000
            
            # Step 3: GPS availability tracking
            gps_tracking_start = time.perf_counter()
            self.gps_available = measured_state is not None
            if self.gps_available:
                self.last_gps_update = timestamp
            gps_tracking_time = (time.perf_counter() - gps_tracking_start) * 1000
            
            # Step 4: Fleet state update
            fleet_update_start = time.perf_counter()
            self.fleet_states[:, self.vehicle_id] = self.local_state
            fleet_update_time = (time.perf_counter() - fleet_update_start) * 1000
            
            # Step 5: Logging and bookkeeping
            logging_start = time.perf_counter()
            self.estimation_log.append({
                'timestamp': timestamp,
                'vehicle_id': self.vehicle_id,
                'local_state': self.local_state.copy(),
                'measured_state': measured_state.copy() if measured_state is not None else None,
                'control_input': control_input.copy(),
                'gps_available': self.gps_available,
                'dt': dt
            })
            
            # Log to enhanced data logger for plotting
            if self.data_logger is not None:
                # Extract actual acceleration from the acceleration parameter if provided
                actual_accel = acceleration[0] if acceleration is not None and len(acceleration) > 1 else 0.0
                
                self.data_logger.log_local_state(
                    timestamp=timestamp,
                    local_state=self.local_state,
                    measured_state=measured_state,
                    control_input=control_input,
                    gps_available=self.gps_available,
                    dt=dt,
                    actual_acceleration=actual_accel
                )
            
            logging_time = (time.perf_counter() - logging_start) * 1000
            
            # Calculate total observer time
            total_observer_time = (time.perf_counter() - observer_start_time) * 1000
            
            # Log detailed timing information for observer performance analysis
            self.logger.info(f"OBS_TIMING: Total={total_observer_time:.3f}ms, DTCalc={dt_calc_time:.3f}ms, "
                           f"StateUpdate={state_update_time:.3f}ms, GPSTrack={gps_tracking_time:.3f}ms, "
                           f"FleetUpdate={fleet_update_time:.3f}ms, Logging={logging_time:.3f}ms")
            
            # Log observer state and method information
            self.logger.info(f"OBS_STATE: Method={method_used}, GPS_avail={self.gps_available}, "
                           f"EKF_init={self.qcar_ekf_initialized}, DT={dt:.4f}s, "
                           f"Pos=({self.local_state[0]:.3f},{self.local_state[1]:.3f}), "
                           f"Theta={np.rad2deg(self.local_state[2]):.1f}deg, Vel={self.local_state[3]:.3f}")
            
            # Log input data details
            if measured_state is not None:
                self.logger.info(f"OBS_INPUT: GPS=({measured_state[0]:.3f},{measured_state[1]:.3f}), "
                               f"GPS_Theta={np.rad2deg(measured_state[2]):.1f}deg, GPS_Vel={measured_state[3]:.3f}, "
                               f"MotorTach={motor_tach:.3f}, Gyro={gyroscope_z:.3f}")
            else:
                self.logger.info(f"OBS_INPUT: GPS=None, MotorTach={motor_tach:.3f}, Gyro={gyroscope_z:.3f}")
            
            # Log control input
            self.logger.info(f"OBS_CONTROL: Steering={control_input[0]:.3f}, Accel={control_input[1]:.3f}")
            
            # Performance warning if observer update is taking too long
            if total_observer_time > 5.0:  # More than 5ms
                self.logger.warning(f"OBS_PERFORMANCE: Observer update slow ({total_observer_time:.3f}ms) - "
                                  f"Target: <5ms for 100Hz control loop")
            
            self.logger.debug(f"Local state updated: pos=({self.local_state[0]:.3f}, {self.local_state[1]:.3f}), "
                            f"theta={np.rad2deg(self.local_state[2]):.1f}°, v={self.local_state[3]:.3f}, "
                            f"GPS={self.gps_available}")
            
            return self.local_state.copy()
    
    def _qcar_ekf_update(self, measured_state: Optional[np.ndarray], control_input: np.ndarray, 
                        dt: float, motor_tach: float, gyroscope_z: float = 99) -> np.ndarray:
        """
        Update state using QCarEKF (inspired by VehicleLeaderController implementation).
        
        Args:
            measured_state: GPS measurement [x, y, theta, v] or None if not available
            control_input: Control input [steering, acceleration]
            dt: Time step
            motor_tach: Motor tachometer reading
            gyroscope_z: Z-axis gyroscope reading
            
        Returns:
            Updated state estimate [x, y, theta, v]
        """
        # Start timing for EKF operation
        ekf_start_time = time.perf_counter()
        
        try:
            # Step 1: Prepare control input
            control_prep_start = time.perf_counter()
            steering = control_input[0]
            control_prep_time = (time.perf_counter() - control_prep_start) * 1000
            
            # Step 2: EKF Update
            ekf_update_start = time.perf_counter()
            if gyroscope_z == 99:
                gyroscope_z = None
            if measured_state is not None:
                # QCarEKF expects GPS measurement as [x, y, theta]
                y_gps = np.array([measured_state[0], measured_state[1], measured_state[2]])
                
                # Update EKF with GPS measurement
                self.ekf.update([motor_tach, steering], dt, y_gps, gyroscope_z)
                update_type = "with_gps"
            else:
                # Update EKF without GPS measurement (prediction only)
                self.ekf.update([motor_tach, steering], dt, None, gyroscope_z)
                update_type = "prediction_only"
            ekf_update_time = (time.perf_counter() - ekf_update_start) * 1000
            
            # Step 3: Extract state from EKF
            state_extract_start = time.perf_counter()
            x = self.ekf.x_hat[0, 0]
            y = self.ekf.x_hat[1, 0] 
            theta = self.ekf.x_hat[2, 0]
            
            # Use motor tachometer for velocity (more reliable than GPS-derived velocity)
            v = motor_tach
            
            # Return state in our format [x, y, theta, v]
            estimated_state = np.array([x, y, theta, v])
            state_extract_time = (time.perf_counter() - state_extract_start) * 1000
            
            # Calculate total EKF time
            total_ekf_time = (time.perf_counter() - ekf_start_time) * 1000
            
            # Log detailed EKF timing
            self.logger.info(f"EKF_TIMING: Total={total_ekf_time:.3f}ms, ControlPrep={control_prep_time:.3f}ms, "
                           f"EKFUpdate={ekf_update_time:.3f}ms, StateExtract={state_extract_time:.3f}ms")
            
            # # Log EKF operation details
            # self.logger.info(f"EKF_OPERATION: Type={update_type}, DT={dt:.4f}s, "
            #                f"Input=[MotorTach={motor_tach:.3f}, Steering={steering:.3f}, Gyro={gyroscope_z:.3f}]")
            
            # Log EKF state output
            self.logger.info(f"EKF_STATE: Out=({x:.3f},{y:.3f}), Theta={np.rad2deg(theta):.1f}°, Vel={v:.3f}")
            
            # if measured_state is not None:
            #     # Log GPS vs EKF comparison
            #     gps_x, gps_y, gps_theta = measured_state[0], measured_state[1], measured_state[2]
            #     pos_error = np.sqrt((x - gps_x)**2 + (y - gps_y)**2)
            #     theta_error = abs(theta - gps_theta)
            #     self.logger.info(f"EKF_GPS_COMPARE: PosError={pos_error:.3f}m, ThetaError={np.rad2deg(theta_error):.1f}°")
            
            # Performance warning
            if total_ekf_time > 3.0:  # More than 3ms for EKF update
                self.logger.warning(f"EKF_PERFORMANCE: EKF update slow ({total_ekf_time:.3f}ms)")
            
            self.logger.debug(f"QCarEKF update: GPS={measured_state is not None}, "
                            f"motor_tach={motor_tach:.3f}, steering={steering:.3f}")
            
            return estimated_state
            
        except Exception as e:
            total_time = (time.perf_counter() - ekf_start_time) * 1000
            self.logger.error(f"EKF_ERROR: QCarEKF update failed after {total_time:.3f}ms - {e}")
            # Fallback to previous state
            return self.local_state.copy()
    
    def _kalman_filter_update(self, measurement: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """Kalman filter update for local state estimation (fallback method)."""
        # Update dt for system matrices
        self.dt = dt
        A, B = self.get_system_matrices(self.local_state, control)
        
        # Prediction step
        x_pred = A @ self.local_state + B @ control
        P_pred = A @ self.P_local @ A.T + self.Q_local
        
        # Update step
        C = np.eye(self.state_dim)  # Direct state measurement
        y = measurement - C @ x_pred  # Innovation
        S = C @ P_pred @ C.T + self.R_local  # Innovation covariance
        K = P_pred @ C.T @ np.linalg.inv(S)  # Kalman gain
        
        # State and covariance update
        x_updated = x_pred + K @ y
        self.P_local = (np.eye(self.state_dim) - K @ C) @ P_pred
        
        return x_updated
    
    def _kalman_prediction_only(self, control: np.ndarray, dt: float) -> np.ndarray:
        """Kalman filter prediction step only (when no GPS measurement available)."""
        # Update dt for system matrices
        self.dt = dt
        A, B = self.get_system_matrices(self.local_state, control)
        
        # Prediction step only
        x_pred = A @ self.local_state + B @ control
        self.P_local = A @ self.P_local @ A.T + self.Q_local
        
        return x_pred
    
    def _luenberger_observer_update(self, measurement: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """Luenberger observer update for local state estimation."""
        # Update dt for system matrices
        self.dt = dt
        A, B = self.get_system_matrices(self.local_state, control)
        
        # Observer gain (simplified - could be optimized)
        L = np.eye(self.state_dim) * 0.1
        
        # Observer update
        x_updated = A @ self.local_state + B @ control + L @ (measurement - self.local_state)
        
        return x_updated

    #endregion


    #region Distributed Observer
    def update_distributed_estimates(self, control: np.ndarray, local_state: np.ndarray, timestamp: float) -> np.ndarray:
        """
        Update distributed state estimates for all vehicles in the fleet.
        Similar to MATLAB Observer.m Distributed_Observer function.
        
        This method estimates the state of ALL vehicles in the platoon using:
        1. Local measurements (when available)
        2. Received states from other vehicles
        3. Distributed consensus algorithm
        4. Same dynamic model for all vehicles
        
        Args:
            timestamp: Current GPS-synchronized timestamp
            
        Returns:
            Updated fleet state estimates [state_dim x fleet_size]
        """
        if not self.observer_config["enable_distributed"]:
            return self.fleet_states.copy()
        
        # Start timing for distributed observer performance
        distributed_start_time = time.perf_counter()
        
        with self.lock:
            # Initialize new fleet state estimates
            fleet_states_new = self.fleet_states.copy()
            
            # # -------- Log initial fleet_states_new
            # self.fleet_logger.info(f"FLEET_STATES_NEW_INIT: Vehicle {self.vehicle_id} - Initial copy from fleet_states")
            # for i in range(self.fleet_size):
            #     state = fleet_states_new[:, i]
            #     self.fleet_logger.info(f"FLEET_STATES_NEW_INIT_V{i}: pos=({state[0]:.5f},{state[1]:.5f}), "
            #                          f"theta={state[2]:.5f}, vel={state[3]:.5f}")
            
            # Get communication weights (equally distributed as requested)
            weights = self._get_distributed_weights()
            
            # For each vehicle j in the fleet, estimate its state
            for j in range(self.fleet_size):
                vehicle_start = time.perf_counter()
                
                # For other vehicles, use distributed observer
                estimated_state = self._distributed_observer_each(
                    host_id=self.vehicle_id,
                    target_id=j,
                    weights=weights,
                    timestamp=timestamp,
                    current_estimates=fleet_states_new,
                    control=control
                )
                fleet_states_new[:, j] = estimated_state
                # update_method = "distributed"
                

            
                vehicle_time = (time.perf_counter() - vehicle_start) * 1000
                # Log updated fleet_states_new for this vehicle
                # same meaning = Log individual vehicle estimation update to fleet logger
                # self.fleet_logger.info(f"FLEET_STATES_NEW_UPDATE_V{j}: pos=({estimated_state[0]:.5f},{estimated_state[1]:.5f}), "
                #                      f"theta={estimated_state[2]:.5f}, vel={estimated_state[3]:.5f}",
                #                      f"Time={vehicle_time:.5f}ms")
                

            # Update fleet states
            self.fleet_states_prev = self.fleet_states.copy()
            self.fleet_states = fleet_states_new
            
            # Log fleet states to enhanced data logger for plotting
            if self.data_logger is not None:
                self.data_logger.log_fleet_states(
                    timestamp=timestamp,
                    fleet_states=self.fleet_states
                )

            # # Log different new estimate and previous fleet_states_new before assignment
            # self.fleet_logger.info(f"FLEET_STATES_NEW_FINAL: Vehicle {self.vehicle_id} - Complete fleet_states_new before assignment")
            # for i in range(self.fleet_size):
            #     new_state = fleet_states_new[:, i]
            #     old_state = self.fleet_states[:, i]
            #     pos_diff = np.linalg.norm(new_state[:2] - old_state[:2])
            #     vel_diff = abs(new_state[3] - old_state[3])
            #     self.fleet_logger.info(f"FLEET_STATES_NEW_FINAL_V{i}: pos=({new_state[0]:.5f},{new_state[1]:.5f}), "
            #                          f"theta={new_state[2]:.5f}, vel={new_state[3]:.5f}, "
            #                          f"pos_change={pos_diff:.5f}, vel_change={vel_diff:.5f}")
            
            
            # Calculate total distributed observer time
            total_distributed_time = (time.perf_counter() - distributed_start_time) * 1000
            
            # Log fleet estimation timing to dedicated fleet logger
            self.fleet_logger.info(f"DIST_TIMING: Total={total_distributed_time:.3f}ms, "
                                 f"FleetSize={self.fleet_size}, Timestamp={timestamp:.3f}")
            
            # Log fleet state summary to dedicated fleet logger
            # self.fleet_logger.info(f"DIST_FLEET: loop estimates for all vehicles")

            # # Log detailed fleet states for all vehicles
            # for i in range(self.fleet_size):
            #     vehicle_state = self.fleet_states[:, i]
            #     self.fleet_logger.info(f"FLEET_STATE_V{i}: pos=({vehicle_state[0]:.5f},{vehicle_state[1]:.5f}), "
            #                           f"vel=({vehicle_state[2]:.5f},{vehicle_state[3]:.5f})")
            
            # Performance warning to dedicated fleet logger
            if total_distributed_time > 10.0:  # More than 10ms
                self.fleet_logger.warning(f"DIST_PERFORMANCE: Distributed update slow ({total_distributed_time:.3f}ms)")
            
            self.fleet_logger.debug(f"Distributed estimates updated for {self.fleet_size} vehicles")
            
            return self.fleet_states.copy()
    
    def _distributed_observer_each_new(self, host_id: int, target_id: int, weights: np.ndarray, 
                                  timestamp: float, current_estimates: np.ndarray, control: np.ndarray) -> np.ndarray:
        """
        Distributed observer for estimating one vehicle's state using consensus algorithm.
        Similar to MATLAB Observer.m Distributed_Observer_Each function.
        
        This implements the distributed observer equation:
        
        Where:
        - x_{ij} is host i's estimate of vehicle j's state
        - A is the system dynamics matrix (same for all vehicles)
        - L_j is the observer gain for vehicle j
        - C_j is the observation matrix for vehicle j  
        - y_j is the measurement from vehicle j
        - W_{ik} is the communication weight between vehicles i and k
        
        Args:
            host_id: ID of the vehicle doing the estimation (self.vehicle_id)
            target_id: ID of the vehicle being estimated
            weights: Communication weights matrix [fleet_size x fleet_size]
            timestamp: Current timestamp
            current_estimates: Current fleet state estimates [state_dim x fleet_size]
            
        Returns:
            Updated state estimate for target vehicle [state_dim,]
        """
        # Current estimate of target vehicle by this host
        x_ij = current_estimates[:, target_id].copy()
        
        # Step 1: Dynamics prediction term A*x_{ij}
        # Use the same dynamics model for all vehicles (as in MATLAB)
        # A = self._get_system_dynamics_matrix()
        A, B = self.get_system_matrices(self.local_state, control)
        dynamics_term = A @ x_ij + B @ control  # Using host's control input as approximation
        
        # Step 2: Local measurement correction term L_j*(C_j*x_{ij} - y_j)
        measurement_term = np.zeros(self.state_dim)
        
        # Only apply measurement correction if we have direct measurement of target
        if target_id == host_id:
            # For own vehicle, use local state as measurement
            C = self._get_observation_matrix()
            L = self._get_observer_gain(target_id)
            
            # Use our own local state as measurement
            y_j = self.local_state  # Full state measurement [x, y, theta, v]
            measurement_error = C @ x_ij - y_j
            measurement_term = L @ measurement_error
        else:
            # For other vehicles, use received local state as measurement with time alignment
            received_state, received_timestamp = self._get_latest_received_state_with_time(target_id, timestamp)
            if received_state is not None:
                # Time-align the received state to current timestamp
                time_delay = timestamp - received_timestamp
                if time_delay > 0 and time_delay < self.max_state_age:
                    # Get the received control input for propagation
                    received_control = self._get_latest_control(target_id, received_timestamp)
                    if received_control is not None:
                        # Propagate received state forward to current time
                        y_j = self._propagate_state_forward(received_state, received_control, time_delay)
                    else:
                        # Use zero control if no control data available
                        y_j = self._propagate_state_forward(received_state, np.zeros(2), time_delay)
                else:
                    # Use received state directly if delay is minimal or too large
                    y_j = received_state
                
                # Apply measurement correction with time-aligned state
                C = self._get_observation_matrix()
                L = self._get_observer_gain(target_id)
                measurement_error = C @ x_ij - y_j
                measurement_term = L @ measurement_error
        
        # Step 3: Consensus term \sum_k W_{ik} * (x_{kj} - x_{ij})
        # NEW: Use fleet estimates from other vehicles for true distributed consensus
        consensus_term = np.zeros(self.state_dim)
        consensus_count = 0
        
        for k in range(self.fleet_size):
            if k != host_id:  # Don't include self in consensus
                weight_ik = weights[host_id, k] if weights.ndim > 1 else weights[k]
                
                # Try to get fleet estimate of target vehicle from vehicle k
                x_kj_fleet = self._get_latest_state_fleet(sender_id=k, target_vehicle_id=target_id, timestamp=timestamp)
                
                if x_kj_fleet is not None and weight_ik > 0:
                    # Use fleet estimate from vehicle k about target vehicle j
                    state_diff = x_kj_fleet - x_ij
                    consensus_term += weight_ik * state_diff
                    consensus_count += 1
                    
                    self.fleet_logger.debug(f"Consensus: V{host_id} using V{k}'s estimate of V{target_id}, "
                                          f"diff_norm={np.linalg.norm(state_diff):.3f}")
                elif weight_ik > 0:
                    # Fallback: Use current fleet estimate (less accurate but still useful)
                    x_kj_fallback = current_estimates[:, target_id]
                    state_diff = x_kj_fallback - x_ij
                    consensus_term += weight_ik * state_diff * 0.5  # Reduced weight for fallback
                    
                    self.fleet_logger.debug(f"Consensus fallback: V{host_id} for V{target_id}, "
                                          f"no fleet estimate from V{k}")
        
        # Log consensus effectiveness
        if consensus_count > 0:
            self.fleet_logger.debug(f"Consensus: V{host_id} estimating V{target_id} using {consensus_count} fleet estimates")
        else:
            self.fleet_logger.debug(f"Consensus: V{host_id} estimating V{target_id} with no external fleet estimates")
        
        # Step 4: Combine all terms with time step integration
        # \dot{x}_{ij} = dynamics_term - measurement_term + consensus_term
        state_derivative = dynamics_term - measurement_term + consensus_term
        
        # Euler integration
        x_ij_new = x_ij + self.dt * state_derivative
        
        # Apply state constraints if needed
        x_ij_new = self._apply_state_constraints(x_ij_new)
        
        return x_ij_new
    

    def _distributed_observer_each_original(self, host_id: int, target_id: int, weights: np.ndarray, 
                                  timestamp: float, current_estimates: np.ndarray, control: np.ndarray) -> np.ndarray:
        """
        Distributed observer for estimating one vehicle's state using consensus algorithm.
        Implements the consensus formula:
        output = A*(x_hat_i_j(:, host_id) + Sig + w_i0 * (x_bar_j - x_hat_i_j(:, host_id))) + B*u_j
        
        Where:
        - x_hat_i_j(:, host_id) is the current estimate of target vehicle j by host i
        - Sig is the consensus term from neighboring vehicles
        - w_i0 is the measurement weight 
        - x_bar_j is the measurement (local state from target vehicle j)
        - u_j is the control input of target vehicle j
        
        Args:
            host_id: ID of the vehicle doing the estimation (self.vehicle_id)
            target_id: ID of the vehicle being estimated
            weights: Communication weights matrix [fleet_size x fleet_size]
            timestamp: Current timestamp
            current_estimates: Current fleet state estimates [state_dim x fleet_size]
            control: Control input [steering, acceleration]
            
        Returns:
            Updated state estimate for target vehicle [state_dim,]
        """
        # Current estimate of target vehicle by this host: x_hat_i_j(:, host_id)
        x_hat_i_j = current_estimates[:, target_id].copy()
        
        # Get system matrices A and B using a consistent reference state
        # Use the target vehicle's current estimate for consistency across all observers
        A, B = self.get_system_matrices(x_hat_i_j, control)
        
        # Step 1: Compute consensus term Sig = \sum_k W_{ik} * (x_{kj} - x_{ij})
        # Use proper distributed consensus based on received fleet estimates
        Sig = np.zeros(self.state_dim)
        consensus_count = 0
        total_consensus_weight = 0.0
        
        for k in range(self.fleet_size):
            if k != host_id:  # Don't include self in consensus
                # Get weight for communication from host_id to vehicle k
                weight_ik = weights[host_id, k] if weights.ndim > 1 else (1.0 / (self.fleet_size - 1))
                
                # Try to get fleet estimate of target vehicle from vehicle k
                x_kj_fleet = self._get_latest_state_fleet(sender_id=k, target_vehicle_id=target_id, timestamp=timestamp)
                
                if x_kj_fleet is not None and weight_ik > 0:
                    # Use fleet estimate from vehicle k about target vehicle j
                    state_diff = x_kj_fleet - x_hat_i_j
                    Sig += weight_ik * state_diff
                    consensus_count += 1
                    total_consensus_weight += weight_ik
                    
                    self.fleet_logger.debug(f"Fleet Consensus: V{host_id} using V{k}'s fleet estimate of V{target_id}, "
                                          f"weight={weight_ik:.3f}, diff_norm={np.linalg.norm(state_diff):.3f}")
                elif weight_ik > 0:
                    # Fallback: Use received individual state from vehicle k (if k == target_id)
                    if k == target_id:
                        received_state_k = self._get_latest_state(k, timestamp)
                        if received_state_k is not None:
                            state_diff = received_state_k - x_hat_i_j
                            reduced_weight = weight_ik * 0.3  # Reduced weight for individual state
                            Sig += reduced_weight * state_diff
                            consensus_count += 1
                            total_consensus_weight += reduced_weight
                            
                            self.fleet_logger.debug(f"Individual Consensus fallback: V{host_id} using V{k}'s individual state, "
                                                   f"weight={reduced_weight:.3f}")
        
        # Apply consensus gain to the total consensus term
        consensus_gain = self.observer_config.get("consensus_gain", 0.3)
        if total_consensus_weight > 0:
            Sig = Sig * (consensus_gain / total_consensus_weight)  # Normalize by total weight
        
        # Log consensus effectiveness
        if consensus_count > 0:
            self.fleet_logger.debug(f"Fleet Consensus: V{host_id} estimating V{target_id} using {consensus_count} estimates, "
                                   f"total_weight={total_consensus_weight:.3f}, consensus_norm={np.linalg.norm(Sig):.3f}")
        else:
            self.fleet_logger.debug(f"Fleet Consensus: V{host_id} estimating V{target_id} with no external estimates")
        
        # If no consensus neighbors available, use stronger measurement weight
        measurement_weight_boost = 1.0
        if consensus_count == 0:
            measurement_weight_boost = 2.0  # Stronger measurement when no consensus available
        elif consensus_count < (self.fleet_size - 1) / 2:
            measurement_weight_boost = 1.5  # Medium boost for partial consensus
        
        # Step 2: Get measurement x_bar_j for target vehicle j
        x_bar_j = np.zeros(self.state_dim)
        w_i0 = 0.0  # Measurement weight
        
        if target_id == host_id:
            # For own vehicle, use local state as measurement
            x_bar_j = self.local_state.copy()
            base_weight = self.observer_config.get("consensus_gain", 0.3)
            w_i0 = base_weight * measurement_weight_boost
        else:
            # For other vehicles, use received local state as measurement with time alignment
            received_state, received_timestamp = self._get_latest_received_state_with_time(target_id, timestamp)
            
            if received_state is not None:
                # Time-align the received state to current timestamp
                time_delay = timestamp - received_timestamp
                if time_delay > 0 and time_delay < self.max_state_age:
                    # Get the received control input for propagation
                    received_control = self._get_latest_control(target_id, received_timestamp)
                    if received_control is not None:
                        # Propagate received state forward to current time
                        x_bar_j = self._propagate_state_forward(received_state, received_control, time_delay)
                    else:
                        # Use zero control if no control data available
                        x_bar_j = self._propagate_state_forward(received_state, np.zeros(2), time_delay)
                else:
                    # Use received state directly if delay is minimal or too large
                    x_bar_j = received_state
                
                # Calculate measurement weight based on data freshness and consensus availability
                base_weight = self.observer_config.get("consensus_gain", 0.3)
                freshness_factor = max(0.1, 1.0 - (time_delay / self.max_state_age))  # Fresher data gets higher weight
                w_i0 = base_weight * measurement_weight_boost * freshness_factor
                
                # Cap the measurement weight to prevent instability
                w_i0 = min(w_i0, 0.7)
                
                self.fleet_logger.debug(f"Measurement update V{host_id}->V{target_id}: "
                                       f"delay={time_delay:.3f}s, freshness={freshness_factor:.3f}, "
                                       f"weight={w_i0:.3f}")
            else:
                # No measurement available for other vehicle
                # Use very weak correction to prevent divergence
                x_bar_j = x_hat_i_j.copy()  # Use current estimate as "measurement" 
                w_i0 = 0.01  # Very small weight to maintain stability
                
                self.fleet_logger.debug(f"No measurement available V{host_id}->V{target_id}, using minimal correction")
        
        # Step 3: Get control input u_j for target vehicle
        if target_id == host_id:
            # Use own control input
            u_j = control.copy()
        else:
            # Use received control input from target vehicle
            received_control = self._get_latest_control(target_id, timestamp)
            if received_control is not None:
                u_j = received_control
            else:
                # Use zero control if no control data available
                u_j = np.zeros(self.control_dim)
        
        # Step 4: Apply consensus formula
        # output = A*(x_hat_i_j + Sig + w_i0 * (x_bar_j - x_hat_i_j)) + B*u_j
        measurement_correction = w_i0 * (x_bar_j - x_hat_i_j)
        inner_term = x_hat_i_j + Sig + measurement_correction
        output = A @ inner_term + B @ u_j
        
        # Apply state constraints if needed
        output = self._apply_state_constraints(output)
        
        # Log added state information
        # if target_id == host_id:  # Only log for other vehicles
        #    self.fleet_logger.info(f"estimate {host_id}: "
        #                     f"pos=({output})")  

        return output


    def _distributed_observer_each(self, host_id: int, target_id: int, weights: np.ndarray, 
                                  timestamp: float, current_estimates: np.ndarray, control: np.ndarray) -> np.ndarray:
        """
        Main distributed observer method that calls the original implementation with fleet consensus.
        """
        return self._distributed_observer_each_original(
            host_id=host_id,
            target_id=target_id,
            weights=weights,
            timestamp=timestamp,
            current_estimates=current_estimates,
            control=control
        )


    #endregion


    def add_received_state(self, sender_id: int, state: np.ndarray, control: np.ndarray,
                          timestamp: float) -> bool:
        """
        Add a received state from another vehicle for distributed estimation.
        
        Args:
            sender_id: ID of the vehicle that sent the state
            state: Received state [x, y, theta, v]
            control: Received control input [steering, acceleration]
            timestamp: GPS-synchronized timestamp
            
        Returns:
            True if state was accepted, False if rejected (too old, invalid, etc.)
        """
        current_time = time.time()
        
        # Validate timestamp
        if current_time - timestamp > self.max_state_age:
            self.fleet_logger.debug(f"Rejected old state from vehicle {sender_id} "
                                  f"(age: {current_time - timestamp:.3f}s)")
            return False
        
        # Validate state dimensions
        if len(state) != self.state_dim or len(control) != self.control_dim:
            self.fleet_logger.warning(f"Invalid state/control dimensions from vehicle {sender_id}")
            return False
        
        with self.lock:
            # Clean old data
            self._cleanup_old_data(sender_id, current_time)
            
            # Add new data
            self.received_states[sender_id].append((timestamp, state.copy()))
            self.received_controls[sender_id].append((timestamp, control.copy()))
            
            # Keep only recent data
            max_history = 100
            if len(self.received_states[sender_id]) > max_history:
                self.received_states[sender_id] = self.received_states[sender_id][-max_history:]
                self.received_controls[sender_id] = self.received_controls[sender_id][-max_history:]
        # self.fleet_logger.info(f"Added state from vehicle {sender_id}: "
        #                        f"pos=({state[0]:.3f}, {state[1]:.3f}), v={state[3]:.3f} , control=({control[0]:.3f}, {control[1]:.3f})")
        return True
    
    def add_received_state_fleet(self, sender_id: int, fleet_estimates: Dict, 
                               timestamp: float) -> bool:
        """
        Add received fleet estimates from another vehicle's distributed observer.
        
        Args:
            sender_id: ID of the vehicle sending the fleet estimates
                        fleet_estimates: Dictionary in structured format ONLY:
                                {
                                    <vehicle_id:int or str>: {
                                             'pos': [x, y],              # required (first two elements of position)
                                             'rot': [roll, pitch, yaw],  # required (yaw used)
                                             'vel': <velocity>,          # required float
                                             'timestamp': <t>,           # optional passthrough
                                             'source': <str>             # optional metadata
                                    }, ...
                                }
                                Accepts numeric keys (int or numeric string). 'Vj' prefixed keys and legacy string summaries are no longer accepted.
            timestamp: GPS-synchronized timestamp of the fleet estimates
            
        Returns:
            True if successfully added, False otherwise
        """
        current_time = time.time()

        # Validate timestamp
        if current_time - timestamp > self.max_state_age:
            self.fleet_logger.debug(
                f"Rejected old fleet estimates from vehicle {sender_id} "
                f"(age: {current_time - timestamp:.3f}s)")
            return False

        # Validate fleet estimates format (must be non-empty dict)
        if not isinstance(fleet_estimates, dict) or len(fleet_estimates) == 0:
            self.fleet_logger.warning(
                f"Invalid fleet estimates format from vehicle {sender_id}")
            return False

        with self.lock:
            # Clean old fleet data
            self._cleanup_old_fleet_data(sender_id, current_time)

            # Add new fleet estimates
            self.received_states_fleet[sender_id].append(
                (timestamp, fleet_estimates.copy()))

            # Keep only recent data
            if len(self.received_states_fleet[sender_id]) > self.max_history:
                self.received_states_fleet[sender_id] = \
                    self.received_states_fleet[sender_id][-self.max_history:]

        # # Validate entries adhere to structured schema (light check)
        # invalid = []
        # for k, v in fleet_estimates.items():
        #     if not isinstance(v, dict):
        #         invalid.append(k); continue
        #     pos = v.get('pos') or v.get('position')
        #     rot = v.get('rot') or v.get('rotation')
        #     vel = v.get('vel') or v.get('velocity')
        #     if not (isinstance(pos, (list, tuple)) and len(pos) >= 2 and
        #             isinstance(rot, (list, tuple)) and len(rot) >= 3 and
        #             isinstance(vel, (int, float))):
        #         invalid.append(k)
        # if invalid:
        #     self.fleet_logger.warning(
        #         f"Structured fleet estimates from {sender_id} contain invalid entries: {invalid}")
        # else:
        #     self.fleet_logger.debug(
        #         f"Added structured fleet estimates from vehicle {sender_id}: {len(fleet_estimates)} entries")
        return True
    
    def _cleanup_old_data(self, sender_id: int, current_time: float):
        """Remove old data that exceeds the maximum age."""
        cutoff_time = current_time - self.max_state_age
        
        # Filter states
        self.received_states[sender_id] = [
            (ts, state) for ts, state in self.received_states[sender_id]
            if ts >= cutoff_time
        ]
        
        # Filter controls
        self.received_controls[sender_id] = [
            (ts, control) for ts, control in self.received_controls[sender_id]
            if ts >= cutoff_time
        ]
    
    def _cleanup_old_fleet_data(self, sender_id: int, current_time: float):
        """Remove old fleet data that exceeds the maximum age."""
        cutoff_time = current_time - self.max_state_age
        
        # Filter fleet states
        self.received_states_fleet[sender_id] = [
            (ts, fleet_estimates) for ts, fleet_estimates in self.received_states_fleet[sender_id]
            if ts >= cutoff_time
        ]
    
    
    def _get_latest_state(self, vehicle_id: int, timestamp: float) -> Optional[np.ndarray]:
        """Get the most recent state from a specific vehicle."""
        if vehicle_id not in self.received_states or not self.received_states[vehicle_id]:
            return None
        
        # Find the most recent state within the time window
        cutoff_time = timestamp - self.max_state_age
        valid_states = [
            (ts, state) for ts, state in self.received_states[vehicle_id]
            if ts >= cutoff_time
        ]
        
        if not valid_states:
            return None
        
        # Return the most recent state
        _, latest_state = max(valid_states, key=lambda x: x[0])
        return latest_state
    
    def _get_latest_control(self, vehicle_id: int, timestamp: float) -> Optional[np.ndarray]:
        """Get the most recent control input from a specific vehicle."""
        if vehicle_id not in self.received_controls or not self.received_controls[vehicle_id]:
            return None
        
        # Find the most recent control within the time window
        cutoff_time = timestamp - self.max_state_age
        valid_controls = [
            (ts, control) for ts, control in self.received_controls[vehicle_id]
            if ts >= cutoff_time
        ]
        
        if not valid_controls:
            return None
        
        # Return the most recent control
        _, latest_control = max(valid_controls, key=lambda x: x[0])
        return latest_control
    
    def _get_latest_state_fleet(self, sender_id: int, target_vehicle_id: int, timestamp: float) -> Optional[np.ndarray]:
        """
        Get the most recent fleet estimate of a target vehicle from a specific sender.
        
        Args:
            sender_id: ID of the vehicle that sent the fleet estimates
            target_vehicle_id: ID of the vehicle whose state we want to extract
            timestamp: Current timestamp for age validation
            
        Returns:
            State array [x, y, theta, v] or None if not available
        """
        if sender_id not in self.received_states_fleet or not self.received_states_fleet[sender_id]:
            return None
        
        # Find the most recent fleet estimates within the time window
        cutoff_time = timestamp - self.max_state_age
        valid_fleet_estimates = [
            (ts, fleet_estimates) for ts, fleet_estimates in self.received_states_fleet[sender_id]
            if ts >= cutoff_time
        ]
        
        if not valid_fleet_estimates:
            return None
        
        # Get the most recent fleet estimates
        _, latest_fleet_estimates = max(valid_fleet_estimates, key=lambda x: x[0])
        
        # Key lookup: numeric id or numeric string only
        entry = None
        for k in (target_vehicle_id, str(target_vehicle_id)):
            if k in latest_fleet_estimates:
                entry = latest_fleet_estimates[k]
                break
        if entry is None:
            return None

        if not isinstance(entry, dict):
            self.fleet_logger.debug(
                f"Non-dict fleet entry ignored sender={sender_id} target={target_vehicle_id} type={type(entry)}")
            return None

        pos = entry.get('pos') or entry.get('position')
        rot = entry.get('rot') or entry.get('rotation')
        vel = entry.get('vel') or entry.get('velocity')
        if pos is None or rot is None or vel is None:
            return None
        if len(pos) < 2 or len(rot) < 3:
            return None
        try:
            return np.array([
                float(pos[0]),
                float(pos[1]),
                float(rot[2]),
                float(vel)
            ])
        except Exception as e:
            self.fleet_logger.warning(
                f"Fleet estimate conversion error sender={sender_id} target={target_vehicle_id}: {e}")
            return None
    
    def get_local_state(self) -> np.ndarray:
        """Get the current local state estimate."""
        with self.lock:
            return self.local_state.copy()
    
    def get_fleet_states(self) -> np.ndarray:
        """Get the current fleet state estimates."""
        with self.lock:
            return self.fleet_states.copy()
    
    def get_vehicle_state(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get the state estimate for a specific vehicle."""
        if 0 <= vehicle_id < self.fleet_size:
            with self.lock:
                return self.fleet_states[:, vehicle_id].copy()
        return None
    
    def get_received_fleet_estimates(self, sender_id: Optional[int] = None) -> Dict:
        """
        Get received fleet estimates from other vehicles' distributed observers.
        
        Args:
            sender_id: ID of the sender vehicle. If None, returns all fleet estimates.
            
        Returns:
            Dictionary of fleet estimates or empty dict if none available
        """
        with self.lock:
            if sender_id is not None:
                if sender_id in self.received_states_fleet and self.received_states_fleet[sender_id]:
                    # Return the most recent fleet estimates from the specified sender
                    timestamp, fleet_estimates = self.received_states_fleet[sender_id][-1]
                    return {
                        'sender_id': sender_id,
                        'timestamp': timestamp,
                        'estimates': fleet_estimates.copy()
                    }
                else:
                    return {}
            else:
                # Return all fleet estimates from all senders
                all_estimates = {}
                for sender, estimates_list in self.received_states_fleet.items():
                    if estimates_list:
                        timestamp, fleet_estimates = estimates_list[-1]  # Most recent
                        all_estimates[sender] = {
                            'timestamp': timestamp,
                            'estimates': fleet_estimates.copy()
                        }
                return all_estimates



    def get_estimated_state_for_control(self) -> dict:
        """
        Get estimated state in format suitable for vehicle controllers.
        
        Returns:
            Dictionary with estimated state information using short keys: pos, rot, vel
        """
        with self.lock:
            # Calculate time since last GPS update
            time_since_gps = time.time() - self.last_gps_update if self.last_gps_update > 0 else float('inf')
            
            # Determine state source
            if self.gps_available and time_since_gps < 0.5:
                source = "gps_recent"
            elif self.qcar_ekf_initialized:
                source = "ekf_estimate"
            else:
                source = "fallback"
            
            result = {
                'pos': [self.local_state[0], self.local_state[1], 0.0],  # [x, y, z] format
                'rot': [0.0, 0.0, self.local_state[2]],  # [roll, pitch, yaw] format  
                'vel': self.local_state[3],
                'estimated': True,
                'gps_available': self.gps_available,
                'time_since_gps': time_since_gps,
                'ekf_initialized': self.qcar_ekf_initialized,
                'source': source
            }
            
            # Log state request for control (at debug level to avoid spam)
            self.logger.debug(f"CONTROL_STATE_REQUEST: Source={source}, "
                            f"GPS_available={self.gps_available}, "
                            f"Time_since_GPS={time_since_gps:.3f}s, "
                            f"EKF_init={self.qcar_ekf_initialized}")
            
            return result
    
    def _get_distributed_weights(self) -> np.ndarray:
        """
        Get communication weights matrix for distributed observer.
        
        WEEK 3 INTEGRATION: Uses trust-based adaptive weights if available,
        otherwise falls back to equal weight distribution.
        
        Returns:
            Communication weights matrix [fleet_size x fleet_size]
        """
        # ═══════════════════════════════════════════════════════════
        # WEEK 3: Trust-Based Adaptive Weights
        # ═══════════════════════════════════════════════════════════
        if self.use_trust_weights:
            try:
                # Get trust scores from vehicle process
                trust_evaluator = getattr(self.vehicle_process, 'trust_evaluator', None)
                
                if trust_evaluator is not None:
                    # Get all trust scores (0-1 range from Week 1)
                    trust_scores = trust_evaluator.get_all_trust_scores()
                    
                    # Convert to numpy array with correct indexing
                    trust_array = np.zeros(self.fleet_size)
                    for vid, score in trust_scores.items():
                        if 0 <= vid < self.fleet_size:
                            trust_array[vid] = score
                    
                    # Call Week 2: calculate_weights_trust_v2()
                    result = self.weight_trust_module.calculate_weights_trust_v2(
                        vehicle_index=self.vehicle_id,
                        trust_scores=trust_array,
                        config=None  # Use default config
                    )
                    
                    # Extract weights (shape: 1 x num_vehicles+1)
                    # Remove virtual node weight (index 0) to get vehicle weights only
                    weights_with_virtual = result['weights'].flatten()
                    vehicle_weights = weights_with_virtual[1:]  # Skip index 0 (virtual node)
                    
                    # Log detailed weight calculation results
                    self.weight_logger.info(f"======== Distributed Observer Weights ========")
                    self.weight_logger.info(f"Trust-based adaptive weight calculation active")
                    self.weight_logger.info(f"Trusted neighbors: {result['trusted_neighbors']}")
                    self.weight_logger.info(f"Virtual node weight (w0): {weights_with_virtual[0]:.4f}")
                    self.weight_logger.info(f"Self weight: {vehicle_weights[self.vehicle_id]:.4f}")
                    
                    # Log neighbor weights
                    for neighbor_id in result['trusted_neighbors']:
                        if 0 <= neighbor_id < len(vehicle_weights):
                            self.weight_logger.info(
                                f"  Neighbor V{neighbor_id}: weight={vehicle_weights[neighbor_id]:.4f}, "
                                f"trust={trust_array[neighbor_id]:.4f}"
                            )
                    
                    # Log weight sum for validation
                    total_weight = np.sum(vehicle_weights)
                    self.weight_logger.info(f"Total vehicle weights sum: {total_weight:.6f}")
                    
                    # Also log to fleet logger for cross-reference
                    self.fleet_logger.info(
                        f"TRUST_WEIGHTS: V{self.vehicle_id} using adaptive weights. "
                        f"Trusted neighbors: {result['trusted_neighbors']}, "
                        f"Virtual weight: {weights_with_virtual[0]:.3f}, "
                        f"Self weight: {vehicle_weights[self.vehicle_id]:.3f}"
                    )
                    
                    # Create weight matrix (this vehicle's row only - others use default)
                    weights = np.ones((self.fleet_size, self.fleet_size))
                    np.fill_diagonal(weights, 0)
                    
                    # Normalize default weights for other vehicles
                    for i in range(self.fleet_size):
                        if i != self.vehicle_id:
                            row_sum = np.sum(weights[i, :])
                            if row_sum > 0:
                                weights[i, :] = weights[i, :] / row_sum
                    
                    # Replace this vehicle's row with trust-based weights
                    weights[self.vehicle_id, :] = vehicle_weights
                    
                    return weights
                    
            except Exception as e:
                self.fleet_logger.warning(
                    f"TRUST_WEIGHTS: Error calculating adaptive weights: {e}. "
                    f"Falling back to equal weights."
                )
        
        # ═══════════════════════════════════════════════════════════
        # FALLBACK: Equal Weight Distribution (Original Implementation)
        # ═══════════════════════════════════════════════════════════
        self.weight_logger.info(f"======== Distributed Observer Weights ========")
        self.weight_logger.info(f"Using equal weight distribution (trust-based weights not available)")
        
        # Create fully connected communication graph (all equal except diagonal 0s)
        weights = np.ones((self.fleet_size, self.fleet_size))
        np.fill_diagonal(weights, 0)  # No self-communication
        
        # Normalize weights for each row (each vehicle distributes weight equally among neighbors)
        for i in range(self.fleet_size):
            row_sum = np.sum(weights[i, :])
            if row_sum > 0:
                # Equal weight distribution among all other vehicles
                weights[i, :] = weights[i, :] / row_sum
            else:
                # If no neighbors (shouldn't happen in fully connected graph)
                weights[i, :] = 0.0
        
        # Log equal weight values
        equal_weight = 1.0 / (self.fleet_size - 1) if self.fleet_size > 1 else 0.0
        self.weight_logger.info(f"Equal weight per neighbor: {equal_weight:.4f}")
        self.weight_logger.info(f"Number of neighbors: {self.fleet_size - 1}")
        
        # Additional check: ensure weights are symmetric for consistency
        # In distributed consensus, typically we want W_ij = W_ji for stability
        weights = (weights + weights.T) / 2.0
        
        # Re-normalize after symmetrization
        for i in range(self.fleet_size):
            row_sum = np.sum(weights[i, :])
            if row_sum > 0:
                weights[i, :] = weights[i, :] / row_sum
        
        return weights
    

    def _get_observation_matrix(self) -> np.ndarray:
        """
        Get observation matrix C for GPS measurements.
        
        Returns:
            Observation matrix C [measurement_dim x state_dim]
        """
        # GPS measures position [x, y]
        C = np.identity(self.state_dim)

        
        return C
    
    def _get_observer_gain(self, vehicle_id: int) -> np.ndarray:
        """
        Get observer gain matrix L for vehicle.
        
        Args:
            vehicle_id: ID of vehicle
            
        Returns:
            Observer gain matrix L [state_dim x state_dim] for full state correction
        """
        # Full state gain matrix for complete state correction
        L = np.eye(self.state_dim)
        
        # Tunable gains for each state component
        position_gain = 0.5  # x, y position correction
        heading_gain = 0.3   # theta heading correction  
        velocity_gain = 0.8  # velocity correction (increased for better velocity tracking)
        
        L[0, 0] = position_gain  # x correction
        L[1, 1] = position_gain  # y correction
        L[2, 2] = heading_gain   # theta correction
        L[3, 3] = velocity_gain  # velocity correction
        
        return L
    
    def _get_latest_received_state_with_time(self, vehicle_id: int, timestamp: float) -> Tuple[Optional[np.ndarray], float]:
        """
        Get latest received state from another vehicle with its timestamp.
        
        Args:
            vehicle_id: ID of vehicle to get state from
            timestamp: Current timestamp
            
        Returns:
            Tuple of (latest received state, state timestamp) or (None, 0.0) if not available
        """
        if vehicle_id not in self.received_states or not self.received_states[vehicle_id]:
            return None, 0.0
        
        # Find the most recent state within time window
        cutoff_time = timestamp - self.max_state_age
        valid_states = [
            (ts, state) for ts, state in self.received_states[vehicle_id]
            if ts >= cutoff_time
        ]
        
        if not valid_states:
            return None, 0.0
        
        # Return the most recent state with its timestamp
        most_recent_timestamp, most_recent_state = max(valid_states, key=lambda x: x[0])
        return most_recent_state, most_recent_timestamp
    
    def _propagate_state_forward(self, state: np.ndarray, control: np.ndarray, time_delay: float) -> np.ndarray:
        """
        Propagate a received state forward in time using vehicle dynamics.
        
        Args:
            state: Received state [x, y, theta, v] at past time
            control: Control input [steering, acceleration] applied during delay
            time_delay: Time delay to propagate forward (seconds)
            
        Returns:
            Propagated state at current time
        """
        if time_delay <= 0:
            return state
        
        # Use system dynamics to propagate state forward
        A, B = self.get_system_matrices(state, control)
        
        # Correct forward integration using discrete-time system dynamics
        # For discrete system: x(k+1) = A*x(k) + B*u(k)
        # For continuous approximation over time_delay: x(t+dt) ≈ A*x(t) + B*u(t)
        propagated_state = A @ state + B @ control
        
        # Apply constraints to keep state physically reasonable
        propagated_state = self._apply_state_constraints(propagated_state)
        
        # Enhanced logging for velocity propagation debugging
        self.fleet_logger.debug(f"State propagation: delay={time_delay:.3f}s, "
                               f"orig_vel={state[3]:.3f}, prop_vel={propagated_state[3]:.3f}, "
                               f"control_accel={control[1]:.3f}")
        
        return propagated_state
    
    def _apply_state_constraints(self, state: np.ndarray) -> np.ndarray:
        """
        Apply physical constraints to state estimate.
        
        Args:
            state: State vector to constrain
            
        Returns:
            Constrained state vector
        """
        constrained_state = state.copy()
        
        # Angle normalization
        if len(constrained_state) > 2:
            constrained_state[2] = np.arctan2(np.sin(constrained_state[2]), np.cos(constrained_state[2]))
        
        # Velocity constraints
        if len(constrained_state) > 3:
            max_velocity = 4.0  # m/s
            constrained_state[3] = np.clip(constrained_state[3], -max_velocity, max_velocity)
        
        return constrained_state



    #region Not Use  

    def validate_state_estimate(self, vehicle_id: int, true_state: np.ndarray) -> dict:
        """
        Validate the state estimate against ground truth.
        
        Args:
            vehicle_id: ID of vehicle to validate
            true_state: Ground truth state
            
        Returns:
            Dictionary with validation results
        """
        estimated_state = self.get_vehicle_state(vehicle_id)
        if estimated_state is None:
            return {'valid': False, 'error': 'No estimate available'}
        
        # Calculate absolute errors
        errors = np.abs(estimated_state - true_state)
        
        # Check if errors are within tolerances
        is_valid = np.all(errors <= self.tolerances)
        violating_elements = np.where(errors > self.tolerances)[0].tolist()
        
        validation_result = {
            'valid': is_valid,
            'errors': errors,
            'tolerances': self.tolerances,
            'violating_elements': violating_elements,
            'max_error': np.max(errors),
            'rms_error': np.sqrt(np.mean(errors**2))
        }
        
        # Log validation results
        self.validation_log.append({
            'timestamp': time.time(),
            'vehicle_id': vehicle_id,
            'validation': validation_result
        })
        
        if not is_valid:
            self.fleet_logger.warning(f"State validation failed for vehicle {vehicle_id}: "
                                    f"errors={errors}, violating={violating_elements}")
        
        return validation_result
    
    def get_observer_stats(self) -> dict:
        """Get statistics about the observer performance."""
        with self.lock:
            stats = {
                'vehicle_id': self.vehicle_id,
                'fleet_size': self.fleet_size,
                'config': self.observer_config,
                'local_state': self.local_state.copy(),
                'fleet_states': self.fleet_states.copy(),
                'received_data_counts': {
                    vid: len(states) for vid, states in self.received_states.items()
                },
                'received_fleet_data_counts': {
                    vid: len(fleet_data) for vid, fleet_data in self.received_states_fleet.items()
                },
                'estimation_log_size': len(self.estimation_log),
                'validation_log_size': len(self.validation_log)
            }
            
            # Add Kalman filter covariance if using Kalman filter
            if self.observer_config["local_observer_type"] == "kalman":
                stats['kalman_covariance'] = self.P_local.copy()
                stats['kalman_trace'] = np.trace(self.P_local)
            
            return stats

    def reset_observer(self, initial_state: Optional[np.ndarray] = None):
        """Reset the observer to initial conditions."""
        with self.lock:
            if initial_state is not None:
                self.local_state = initial_state.copy()
                self.fleet_states[:, self.vehicle_id] = initial_state.copy()
                # Reinitialize EKF with new initial pose if using QCarEKF
                if (self.observer_config["local_observer_type"] == "kalman" and 
                    len(initial_state) >= 3):
                    self.initialize_ekf(initial_state[:3])  # [x, y, theta]
            else:
                self.local_state = np.zeros(self.state_dim)
                self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
                # Reinitialize EKF with default pose
                if self.observer_config["local_observer_type"] == "kalman":
                    self.initialize_ekf()
            
            # Reset Kalman filter covariance
            if self.observer_config["local_observer_type"] == "kalman":
                self.P_local = np.eye(self.state_dim) * 0.1
            
            # Reset GPS tracking
            self.gps_available = False
            self.last_gps_update = 0.0
            
            # Clear data history
            self.received_states.clear()
            self.received_controls.clear()
            self.received_states_fleet.clear()
            self.estimation_log.clear()
            self.validation_log.clear()
            
            self.logger.info(f"Observer reset for vehicle {self.vehicle_id}")
    #endregion
    
    #region Data Export and Plotting
    #endregion
    def save_data_for_plotting(self) -> str:
        """
        Save observer data in multiple formats for plotting and analysis.
        
        Returns:
            Directory path where data was saved
        """
        if self.data_logger is None:
            self.logger.error("DataLogger not available. Cannot save data for plotting.")
            return ""
        
        # Close the data logger to save all data
        data_dir = self.data_logger.data_dir
        self.data_logger.close()
        
        # Reinitialize data logger for continued operation if needed
        try:
            self.data_logger = ObserverDataLogger(self.vehicle_id, self.fleet_size, log_dir="data_logs")
        except ImportError:
            self.data_logger = None
        
        self.logger.info(f"Data saved for plotting in directory: {data_dir}")
        return data_dir
    
    def export_matlab_data(self, filename: Optional[str] = None) -> str:
        """
        Export observer data to MATLAB .mat format.
        
        Args:
            filename: Optional custom filename
            
        Returns:
            Path to the saved MATLAB file
        """
        if self.data_logger is None:
            self.logger.error("DataLogger not available. Cannot export MATLAB data.")
            return ""
        
        matlab_path = self.data_logger.save_matlab_data(filename)
        self.logger.info(f"MATLAB data exported to: {matlab_path}")
        return matlab_path
        """
            filename: Optional custom filename
            
        Returns:
            Path to the saved MATLAB file
        matlab_path = self.data_logger.save_matlab_data(filename)
        self.logger.info(f"MATLAB data exported to: {matlab_path}")
        return matlab_path
        """
    
    def plot_trajectories(self, show_gps: bool = True, save_fig: bool = True):
        """
        Plot vehicle trajectories using the data logger.
        
        Args:
            show_gps: Whether to show GPS measurements
            save_fig: Whether to save the figure
        """
        try:
            from DataLogger import FleetDataVisualizer
        except ImportError:
            self.logger.error("DataLogger module not found. Cannot create plots.")
            return
        
        # Ensure data is saved first
        data_dir = self.data_logger.data_dir
        
        # Create visualizer and plot
        visualizer = FleetDataVisualizer(data_dir)
        
        # Plot individual trajectory
        visualizer.plot_vehicle_trajectory(self.vehicle_id, show_gps, save_fig)
        
        # Plot fleet trajectories
        visualizer.plot_fleet_trajectories(self.vehicle_id, save_fig)
        
        # Plot state comparison
        visualizer.plot_state_comparison(self.vehicle_id, save_fig)
        
        self.logger.info(f"Plots generated for vehicle {self.vehicle_id}")
    
    def close_data_logger(self):
        """Close the data logger and save all data."""
        if hasattr(self, 'data_logger'):
            self.data_logger.close()
            self.logger.info(f"Data logger closed for vehicle {self.vehicle_id}")
    
    def __del__(self):
        """Destructor to ensure data logger is closed."""
        try:
            self.close_data_logger()
        except:
            pass  # Ignore errors during cleanup
