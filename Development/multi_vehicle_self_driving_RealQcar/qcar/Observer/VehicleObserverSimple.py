"""
Vehicle Observer Manager - Coordinates local and fleet state estimation

This is a manager class that:
1. Controls update rates for local and fleet observers
2. Delegates state estimation to specialized estimators (local_state_estimators, fleet_state_estimators)
3. Provides unified interface for accessing state data
4. Manages sensor data reading and caching

Architecture:
- LocalStateEstimator: Handles local vehicle state (EKF, Luenberger, etc.)
- FleetStateEstimator: Handles distributed fleet estimation (Consensus, Distributed Kalman, etc.)
- VehicleObserver: Manager that coordinates both and provides data access
"""

import numpy as np
import threading
import time
from typing import Dict, List, Optional, Tuple
from collections import defaultdict

from Observer.local_state_estimators import LocalEstimatorFactory, LocalStateEstimatorBase
from Observer.fleet_state_estimators import FleetEstimatorFactory, FleetStateEstimatorBase


class VehicleObserver:
    """
    Vehicle Observer Manager
    
    Coordinates local and fleet state estimation with pluggable algorithms.
    Provides unified interface for data access to other systems.
    """

    def __init__(self, vehicle_id: int, config=None, logger=None, 
                 local_estimator_type: str = 'ekf',
                 fleet_estimator_type: str = 'consensus'):
        """
        Initialize the Vehicle Observer Manager.
        
        Args:
            vehicle_id: ID of the host vehicle
            config: Configuration object
            logger: Logger instance
            local_estimator_type: Type of local state estimator ('ekf', 'luenberger', 'dead_reckoning')
            fleet_estimator_type: Type of fleet state estimator ('consensus', 'distributed_kalman')
            
        Note:
            - Fleet size is initially 1 (just this vehicle)
            - Fleet will be reinitialized when V2V activates via reinitialize_fleet_estimation()
            - State estimators are created by factories and can be swapped at runtime
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = max(vehicle_id + 1, 1)  # At least large enough for this vehicle
        self.config = config or {}
        self.vehicle_logger = logger
        
        # State dimensions: [x, y, theta, v] - position, orientation, velocity
        self.state_dim = 4
        
        # Observer configuration
        self.observer_config = self._get_observer_config()
        
        # ===== Local State Estimator (pluggable) =====
        self.local_estimator_type = local_estimator_type
        self.local_estimator: Optional[LocalStateEstimatorBase] = None
        # Will be initialized later via initialize_local_estimator()
        
        # ===== Fleet State Estimator (pluggable) =====
        self.fleet_estimator_type = fleet_estimator_type
        self.fleet_estimator: Optional[FleetStateEstimatorBase] = None
        # Fleet estimator will be created when V2V is activated (not at initialization)
        # This saves resources and ensures clean state when V2V starts
        self.v2v_active = False  # Track if V2V is active
        
        # ===== State Cache (for quick access) =====
        self.local_state = np.zeros(self.state_dim)
        self.position = np.zeros(3)  # [x, y, theta]
        self.velocity = 0.0
        self.gps_valid = False  # GPS validity flag
        
        # Fleet states (managed by fleet_estimator but cached here)
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
        
        # ===== Sensor Data Cache =====
        self.sensor_data = {
            'motor_tach': 0.0,
            'gyro_z': 0.0,
            'timestamp': 0.0,
            'gps_valid': False,
            'gps_position': np.zeros(3),  # [x, y, theta]
            'gps_updated': False
        }
        
        # ===== Control and Dynamics Cache =====
        # self.last_velocity = 0.0
        self.acceleration = 0.0
        self.control_input = {'steering': 0.0, 'throttle': 0.0}
        # self.last_update_time = 0.0
        
        # ===== GPS Reference =====
        self.gps = None  # Will be set during initialize_local_estimator
        
        # ===== Timing Control =====
        self.local_observer_rate = self.observer_config.get("observer_rate", 100)
        self.fleet_observer_rate = self.observer_config.get("fleet_observer_rate", 50)
        self._last_fleet_observer_time = 0.0
        
        # ===== Thread Safety =====
        self.lock = threading.RLock()
        
        self.vehicle_logger.logger.info(
            f"VehicleObserver initialized: vehicle_id={vehicle_id}, "
            # f"local_estimator={local_estimator_type}, fleet_estimator={fleet_estimator_type}"
        )

    # ===== Factory Methods for Creating Estimators =====
    
    def _create_fleet_estimator(self):
        """Create fleet state estimator using factory"""
        try:
            fleet_config = {
                'consensus_gain': self.observer_config.get('consensus_gain', 0.3),
                'observer_gain': self.observer_config.get('observer_gain', 0.1),
            }
            
            self.fleet_estimator = FleetEstimatorFactory.create(
                estimator_type=self.fleet_estimator_type,
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                state_dim=self.state_dim,
                config=fleet_config,
                logger=self.vehicle_logger
            )
            
            self.vehicle_logger.logger.info(f"Fleet estimator created: {self.fleet_estimator_type}")
            
        except Exception as e:
            self.vehicle_logger.log_error(f"Failed to create fleet estimator: {self.fleet_estimator_type}", e)
            # Fallback to consensus
            self.fleet_estimator = FleetEstimatorFactory.create(
                estimator_type='consensus',
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                state_dim=self.state_dim,
                config={'consensus_gain': 0.3},
                logger=self.vehicle_logger
            )
    
    def initialize_local_estimator(self, gps=None, initial_pose=None, 
                                   estimator_params: Dict = None):
        """
        Initialize local state estimator using factory
        
        Args:
            gps: GPS instance (for EKF)
            initial_pose: Initial pose [x, y, theta]
            estimator_params: Additional parameters for the estimator
            
        Returns:
            bool: True if initialization successful
        """
        try:
            estimator_params = estimator_params or {}
            
            # Store GPS reference at observer level for centralized sensor reading
            self.gps = gps
            
            self.local_estimator = LocalEstimatorFactory.create(
                estimator_type=self.local_estimator_type,
                initial_pose=initial_pose,
                gps=gps,
                logger=self.vehicle_logger,
                **estimator_params
            )
            
            self.vehicle_logger.logger.info(
                f"Local estimator initialized: {self.local_estimator_type}"
            )
            
            return True
            
        except Exception as e:
            self.vehicle_logger.log_error(
                f"Local estimator initialization failed: {self.local_estimator_type}", e
            )
            return False
    
    # ===== Timing Control =====
    
    def _should_update_fleet_observer(self, current_time: float) -> bool:
        """Check if fleet observer should update based on its rate (independent of local observer)"""
        if current_time - self._last_fleet_observer_time >= 1.0 / self.fleet_observer_rate:
            self._last_fleet_observer_time = current_time
            return True
        return False

    # ===== Configuration =====
    
    def _get_observer_config(self) -> dict:
        """Get observer configuration with defaults."""
        default_config = {
            "observer_rate": 100,
            "fleet_observer_rate": 50,
            "local_observer_type": "ekf",
            "enable_distributed": True,
            "consensus_gain": 0.3
        }
        
        return default_config

    def update_sensor_data(self, qcar):
        """
        Update sensor data from QCar hardware AND GPS.
        This centralizes all sensor reading in one place.
        YOLO logic is handled separately in vehicle_logic.py
        """
        try:
            if qcar is not None:
                # Read QCar sensors - handle case where readTask might not exist
                try:
                    qcar.read()
                except AttributeError as read_error:
                    if "readTask" in str(read_error):
                        # QCar object doesn't have readTask, try alternative approach
                        # or skip reading if hardware is not properly initialized
                        self.vehicle_logger.log_warning("QCar readTask not available, skipping sensor read")
                        return False
                    else:
                        raise  # Re-raise if it's a different attribute error
                
                # Update sensor data cache
                with self.lock:
                    # Read accelerometer (x, y, z axes)
                    accel_x = qcar.accelerometer[0] if hasattr(qcar, 'accelerometer') else 0.0
                    accel_y = qcar.accelerometer[1] if hasattr(qcar, 'accelerometer') else 0.0
                    # Calculate horizontal acceleration magnitude (norm of x and y)
                    accel_magnitude = np.sqrt(accel_x**2 + accel_y**2)
                    
                    # Read GPS once here (centralized GPS reading)
                    gps_valid = False
                    gps_position = np.zeros(3)  # [x, y, theta]
                    gps_updated = False
                    
                    if self.gps is not None:
                        try:
                            gps_updated = self.gps.readGPS()
                            if gps_updated:
                                gps_valid = True
                                gps_position = np.array([
                                    self.gps.position[0],
                                    self.gps.position[1],
                                    self.gps.orientation[2]
                                ])
                        except Exception as gps_error:
                            self.vehicle_logger.log_warning(f"GPS read failed: {gps_error}")
                            gps_valid = False
                    
                    self.sensor_data.update({
                        'motor_tach': qcar.motorTach,
                        'gyro_z': qcar.gyroscope[2] if hasattr(qcar, 'gyroscope') else 0.0,
                        'accelerometer': qcar.accelerometer if hasattr(qcar, 'accelerometer') else np.zeros(3),
                        'accel_magnitude': accel_magnitude,
                        'timestamp': time.time(),
                        'gps_valid': gps_valid,
                        'gps_position': gps_position,
                        'gps_updated': gps_updated
                    })
                
                return True
                
        except Exception as e:
            self.vehicle_logger.log_error("Sensor data update error", e)
            return False

    def update_observer(self, dt: float, last_steering: float = 0.0, throttle: float = 0.0) -> dict:
        """
        Main observer update method that handles both local and fleet observer updates.
        Local observer is called every time (vehicle_logic controls the rate).
        Fleet observer has independent rate control.
        
        Args:
            dt: Time step
            last_steering: Last steering command
            throttle: Throttle command (for control input tracking)
            
        Returns:
            dict: Current state information compatible with vehicle_logic
        """
        current_time = time.time()
        
        try:
            # Update control input cache
            self.control_input = {'steering': last_steering, 'throttle': throttle}
            self.acceleration = float(self.sensor_data['accel_magnitude'])

            # # Get acceleration from accelerometer if available, otherwise calculate from velocity
            # if 'accel_magnitude' in self.sensor_data and self.sensor_data['accel_magnitude'] != 0.0:
            #     # Use measured acceleration from accelerometer (preferred)
            #     self.acceleration = float(self.sensor_data['accel_magnitude'])
            #     self.last_velocity = self.sensor_data['motor_tach']
            # elif dt > 0 and self.last_update_time > 0:
            #     # Fallback: Calculate acceleration from velocity change
            #     current_velocity = self.sensor_data['motor_tach']
            #     self.acceleration = (current_velocity - self.last_velocity) / dt
            #     self.last_velocity = current_velocity
            # else:
            #     self.acceleration = 0.0
            #     self.last_velocity = self.sensor_data['motor_tach']
            
            # self.last_update_time = current_time
            
            # Update local observer (always - rate controlled by vehicle_logic)
            state_info = self._update_local_observer(dt, last_steering)
            
            # Update fleet observer if it's time (independent rate control)
            if self._should_update_fleet_observer(current_time):
                self._update_fleet_observer_internal(dt)
            
            return state_info
            
        except Exception as e:
            self.vehicle_logger.log_error("Observer update error", e)
            # Return last known state instead of zeros
            return self._get_last_known_state()

    def _update_local_observer(self, dt: float, last_steering: float = 0.0) -> dict:
        """
        Update local state estimation using pluggable local estimator.
        This is called every time vehicle_logic calls update_observer().
        
        Args:
            dt: Time step
            last_steering: Last steering command
            
        Returns:
            dict: Current state information compatible with vehicle_logic
        """
        try:
            if self.local_estimator is None:
                self.vehicle_logger.log_error("Local estimator not initialized! Cannot update observer.")
                raise RuntimeError("VehicleObserver: local_estimator is None - observer cannot function")
            
            # Prepare GPS data dict for estimator (if GPS is valid)
            gps_data = None
            if self.sensor_data.get('gps_valid', False):
                gps_data = {
                    'x': self.sensor_data['gps_position'][0],
                    'y': self.sensor_data['gps_position'][1],
                    'theta': self.sensor_data['gps_position'][2],
                    'valid': True
                }
            
            # Update local estimator with sensor data
            success = self.local_estimator.update(
                motor_tach=self.sensor_data['motor_tach'],
                steering=last_steering,
                dt=dt,
                gyro_z=self.sensor_data['gyro_z'],
                gps_data=gps_data  # Pass GPS data from centralized sensor reading
            )
            
            if not success:
                return self._get_last_known_state()
            
            # Get current state from estimator (returns numpy array directly)
            state = self.local_estimator.get_state()
            
            # Update local state cache
            # GPS validity is tracked at observer level based on actual GPS reading
            gps_valid = self.sensor_data.get('gps_valid', False)
            
            with self.lock:
                self.local_state = state.copy()
                self.position = state[:3].copy()  # [x, y, theta]
                self.velocity = float(state[3])
                self.gps_valid = gps_valid  # GPS validity from sensor data
            
            return {
                'x': float(state[0]), 'y': float(state[1]), 
                'theta': float(state[2]), 'velocity': float(state[3]),
                'gps_valid': gps_valid,
                'position': self.position.copy(),
                'local_state': self.local_state.copy()
            }
            
        except Exception as e:
            self.vehicle_logger.log_error("Local observer update error", e)
            return self._get_last_known_state()

    def _update_fleet_observer_internal(self, dt: float):
        """
        Update fleet observer estimates using pluggable fleet estimator.
        This is called based on fleet observer update rate.
        Only runs when V2V is active.
        
        Args:
            dt: Time step
        """
        try:
            # Only update fleet observer if V2V is active
            if not self.v2v_active:
                return
            
            if not self.observer_config["enable_distributed"]:
                return
            
            if self.fleet_estimator is None:
                return
            
            current_time_ns = time.time_ns()  # Use nanoseconds for consistency with V2V timestamps
            
            # Get control input (steering, throttle) - use zeros as default
            control = np.array([0.0, 0.0])  # Will be passed from vehicle_logic in future
            
            # Update fleet estimates using pluggable estimator
            current_local = self.local_state.copy()
            self.fleet_states = self.fleet_estimator.update(
                local_state=current_local,
                dt=dt,
                current_time=current_time_ns,  # Pass nanoseconds
                control=control
            )
            
            # Verify own state is correctly set in fleet_states
            if self.vehicle_id < self.fleet_size:
                own_fleet_state = self.fleet_states[:, self.vehicle_id]
                if np.allclose(own_fleet_state, 0.0) and not np.allclose(current_local, 0.0):
                    # Own state is zeros but local state is not - this is the bug!
                    self.vehicle_logger.logger.warning(
                        f"VehicleObserver WARNING: Own state in fleet is zeros but local state is not!\n"
                        f"  local_state: x={current_local[0]:.3f}, y={current_local[1]:.3f}, "
                        f"theta={current_local[2]:.3f}, v={current_local[3]:.3f}\n"
                        f"  fleet_states[{self.vehicle_id}]: x={own_fleet_state[0]:.3f}, "
                        f"y={own_fleet_state[1]:.3f}, theta={own_fleet_state[2]:.3f}, v={own_fleet_state[3]:.3f}"
                    )
                    # Force update
                    self.fleet_estimator.fleet_states[:, self.vehicle_id] = current_local.copy()
                    self.fleet_states = self.fleet_estimator.get_fleet_states()
            
        except Exception as e:
            self.vehicle_logger.log_error("Fleet observer update error", e)

    def set_local_estimator(self, estimator: LocalStateEstimatorBase):
        """
        Set or change the local state estimator.
        Allows different types of estimators to be used at runtime.
        
        Args:
            estimator: LocalStateEstimatorBase instance
        """
        self.local_estimator = estimator
        self.vehicle_logger.logger.info(
            f"Local estimator set for vehicle {self.vehicle_id}: {type(estimator).__name__}"
        )
    
    def set_fleet_estimator(self, estimator: FleetStateEstimatorBase):
        """
        Set or change the fleet state estimator.
        
        Args:
            estimator: FleetStateEstimatorBase instance
        """
        self.fleet_estimator = estimator
        self.vehicle_logger.logger.info(
            f"Fleet estimator set for vehicle {self.vehicle_id}: {type(estimator).__name__}"
        )

    def get_local_estimator(self) -> Optional[LocalStateEstimatorBase]:
        """
        Get the current local state estimator instance.
        
        Returns:
            LocalStateEstimatorBase instance or None
        """
        return self.local_estimator
    
    def get_fleet_estimator(self) -> Optional[FleetStateEstimatorBase]:
        """
        Get the current fleet state estimator instance.
        
        Returns:
            FleetStateEstimatorBase instance or None
        """
        return self.fleet_estimator

    def add_received_state(self, sender_id: int, state: np.ndarray, timestamp: float) -> bool:
        """
        Add received state from another vehicle.
        Delegates to fleet estimator for processing.
        
        Args:
            sender_id: ID of the vehicle that sent the state
            state: Received state [x, y, theta, v]
            timestamp: Timestamp of the state
            
        Returns:
            bool: True if state was added successfully
        """
        try:
            if sender_id == self.vehicle_id:
                return False  # Don't store own state
            
            if self.fleet_estimator is None:
                return False
            
            # Delegate to fleet estimator
            return self.fleet_estimator.add_received_state(sender_id, state, timestamp)
            
        except Exception as e:
            self.vehicle_logger.log_error("Add received state error", e)
            return False

    def get_local_state(self) -> np.ndarray:
        """Get current local state estimate."""
        with self.lock:
            return self.local_state.copy()

    def get_fleet_states(self) -> np.ndarray:
        """Get current fleet state estimates."""
        with self.lock:
            return self.fleet_states.copy()

    def get_vehicle_state(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get state estimate for a specific vehicle."""
        if 0 <= vehicle_id < self.fleet_size:
            with self.lock:
                return self.fleet_states[:, vehicle_id].copy()
        return None

    def get_current_position(self) -> List[float]:
        """Get current position [x, y, theta] compatible with vehicle_logic."""
        with self.lock:
            return [float(self.position[0]), float(self.position[1]), float(self.position[2])]

    def get_current_velocity(self) -> float:
        """Get current velocity compatible with vehicle_logic."""
        with self.lock:
            return float(self.velocity)

    def is_gps_valid(self) -> bool:
        """Check if GPS is valid."""
        with self.lock:
            return self.gps_valid

    def get_sensor_data(self) -> dict:
        """Get current sensor data."""
        with self.lock:
            return self.sensor_data.copy()

    # Old helper methods removed - fleet estimator handles data management internally

    def _get_last_known_state(self) -> dict:
        """Get last known state when estimation fails - preserves last valid state."""
        with self.lock:
            return {
                'x': float(self.local_state[0]),
                'y': float(self.local_state[1]),
                'theta': float(self.local_state[2]),
                'velocity': float(self.local_state[3]),
                'gps_valid': False,  # Mark as invalid but keep last position
                'position': self.position.copy(),
                'local_state': self.local_state.copy()
            }

    def get_estimated_state_for_control(self) -> dict:
        """
        Get state information formatted for control systems.
        Compatible with existing vehicle_logic state format.
        """
        with self.lock:
            return {
                'x': float(self.local_state[0]),
                'y': float(self.local_state[1]), 
                'theta': float(self.local_state[2]),
                'velocity': float(self.local_state[3]),
                'motor_tach': self.sensor_data['motor_tach'],
                'gyro_z': self.sensor_data['gyro_z'],
                'gps_valid': self.gps_valid
            }
    
    def get_local_state_for_broadcast(self) -> dict:
        """
        Get local state information for V2V broadcasting.
        High-frequency, local sensor-based estimates.
        Includes acceleration and control inputs for future use in cooperative control.
        """
        with self.lock:
            return {
                'vehicle_id': self.vehicle_id,
                'x': float(self.local_state[0]),
                'y': float(self.local_state[1]),
                'theta': float(self.local_state[2]),
                'velocity': float(self.local_state[3]),
                'acceleration': float(self.acceleration),
                'control_input': {
                    'steering': float(self.control_input['steering']),
                    'throttle': float(self.control_input['throttle'])
                },
                'gps_valid': self.gps_valid,
                'source': 'local_sensors'
            }
    
    def get_fleet_state_for_broadcast(self) -> dict:
        """
        Get fleet state information for V2V broadcasting.
        Lower-frequency, consensus-based fleet estimates.
        """
        with self.lock:
            fleet_data = {}
            for vehicle_id in range(self.fleet_size):
                # Include all vehicles in fleet (zeros or not) for proper fleet estimation
                # The receiver can decide whether to use the data based on confidence/age
                fleet_data[vehicle_id] = {
                    'x': float(self.fleet_states[0, vehicle_id]),
                    'y': float(self.fleet_states[1, vehicle_id]),
                    'theta': float(self.fleet_states[2, vehicle_id]),
                    'velocity': float(self.fleet_states[3, vehicle_id]),
                    'confidence': 1.0 if vehicle_id == self.vehicle_id else 0.8  # Higher confidence for own state
                }
            
            return {
                'sender_id': self.vehicle_id,
                'fleet_states': fleet_data,
                'source': 'fleet_consensus'
            }

    def reinitialize_fleet_estimation(self, new_fleet_size: int, peer_vehicle_ids: List[int]):
        """
        Reinitialize fleet estimation when V2V is activated with actual fleet information.
        This should be called when V2V activation provides the real fleet size and peer IDs.
        
        Args:
            new_fleet_size: Actual number of vehicles in the fleet (including this vehicle)
            peer_vehicle_ids: List of peer vehicle IDs that will be connected
        """
        with self.lock:
            old_fleet_size = self.fleet_size
            self.fleet_size = new_fleet_size
            
            # Mark V2V as active - fleet observer will start updating
            self.v2v_active = True
            
            # Create fresh fleet estimator with new fleet size (no old data to copy)
            try:
                fleet_config = {
                    'consensus_gain': self.observer_config.get('consensus_gain', 0.3),
                    'observer_gain': self.observer_config.get('observer_gain', 0.1),
                }
                
                # Create new fleet estimator with correct size
                self.fleet_estimator = FleetEstimatorFactory.create(
                    estimator_type=self.fleet_estimator_type,
                    vehicle_id=self.vehicle_id,
                    fleet_size=self.fleet_size,
                    state_dim=self.state_dim,
                    config=fleet_config,
                    logger=self.vehicle_logger
                )
                
                # Initialize only own state in fleet - others will be updated as V2V data arrives
                if self.vehicle_id < self.fleet_size:
                    current_local = self.local_state.copy()
                    self.fleet_estimator.fleet_states[:, self.vehicle_id] = current_local
                    self.vehicle_logger.logger.info(
                        f"VehicleObserver: V2V activated - Initialized own state in fleet - "
                        f"vehicle_{self.vehicle_id}: x={current_local[0]:.3f}, y={current_local[1]:.3f}, "
                        f"theta={current_local[2]:.3f}, v={current_local[3]:.3f}"
                    )
                
                # Update cached fleet states
                self.fleet_states = self.fleet_estimator.get_fleet_states()
                
                # Log the complete fleet state after reinit
                self.vehicle_logger.logger.info("VehicleObserver: Fleet states after V2V activation:")
                for vid in range(self.fleet_size):
                    fs = self.fleet_states[:, vid]
                    self.vehicle_logger.logger.info(
                        f"  vehicle_{vid}: x={fs[0]:.3f}, y={fs[1]:.3f}, theta={fs[2]:.3f}, v={fs[3]:.3f}"
                    )
                
                
            except Exception as e:
                self.vehicle_logger.log_error("Fleet estimation reinitialization failed", e)

    def reset_observer(self, initial_pose: Optional[np.ndarray] = None):
        """Reset observer state."""
        with self.lock:
            # Reset local estimator
            if self.local_estimator is not None:
                self.local_estimator.reset(initial_pose)
            
            # Reset fleet estimator
            if self.fleet_estimator is not None:
                self.fleet_estimator.reset()
            
            # Reset local state cache
            if initial_pose is not None:
                self.local_state[:3] = initial_pose
                self.local_state[3] = 0.0
                self.position = initial_pose.copy()
            else:
                self.local_state = np.zeros(self.state_dim)
                self.position = np.zeros(3)
            
            self.velocity = 0.0
            self.gps_valid = False
            
            # Reset acceleration and control tracking
            # self.last_velocity = 0.0
            self.acceleration = 0.0
            self.control_input = {'steering': 0.0, 'throttle': 0.0}
            # self.last_update_time = 0.0
            
            # Update fleet states from fleet estimator
            if self.fleet_estimator is not None:
                self.fleet_states = self.fleet_estimator.get_fleet_states()

    def __del__(self):
        """Cleanup on destruction."""
        pass