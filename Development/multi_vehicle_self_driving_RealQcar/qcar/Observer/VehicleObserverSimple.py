"""
Simplified Vehicle Observer for local and fleet state estimation
Integrated with vehicle_logic.py
"""

import numpy as np
import threading
import time
from typing import Dict, List, Optional, Tuple
from collections import defaultdict


class VehicleObserver:
    """
    Simplified Vehicle Observer for local and fleet state estimation.
    Handles sensor data reading and state estimation in one place.
    """

    def __init__(self, vehicle_id: int, fleet_size: int, config=None, logger=None, initial_pose=None, state_estimator=None):
        """
        Initialize the simplified Vehicle Observer.
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in the fleet
            config: Configuration object
            logger: Logger instance
            initial_pose: Initial pose [x, y, theta] for the vehicle
            state_estimator: StateEstimator instance (child component)
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.config = config
        self.vehicle_logger = logger
        
        # State estimator as child component
        self.state_estimator = state_estimator
        
        # State dimensions: [x, y, theta, v] - position, orientation, velocity
        self.state_dim = 4
        
        # Observer configuration
        self.observer_config = self._get_observer_config()
        
        # Current estimated state [x, y, theta, velocity]
        self.local_state = np.zeros(self.state_dim)
        if initial_pose is not None:
            self.local_state[:3] = initial_pose  # Set x, y, theta
            self.local_state[3] = 0.0  # Initialize velocity to 0
        
        # Fleet state estimates - estimates for all vehicles
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
        
        # Initialize fleet states
        for i in range(self.fleet_size):
            if i == self.vehicle_id:
                self.fleet_states[:, i] = self.local_state.copy()
            else:
                self.fleet_states[:, i] = np.zeros(self.state_dim)
        
        # Sensor data cache
        self.sensor_data = {
            'motor_tach': 0.0,
            'gyro_z': 0.0,
            'state_valid': False,
            'timestamp': 0.0
        }
        
        # EKF state estimation variables
        self.state_valid = False
        self.position = np.zeros(3)  # [x, y, theta]
        self.velocity = 0.0
        
        # Communication data
        self.received_states = defaultdict(list)  # vehicle_id -> list of (timestamp, state)
        self.max_state_age = 2.0  # Maximum age of received states (seconds)
        
        # Thread safety
        self.lock = threading.RLock()
        
        # Timing
        self.dt = 1.0 / self.config.get("observer_rate", 100) if config else 0.01
        
        # Internal update rates and timing for local and fleet observers
        self.local_observer_rate = self.config.get("observer_rate", 100) if config else 100
        self.fleet_observer_rate = self.config.get("fleet_observer_rate", 50) if config else 50
        
        # Timing trackers for different observer updates
        self._last_local_observer_time = 0.0
        self._last_fleet_observer_time = 0.0
        
        self.vehicle_logger.logger.info(f"VehicleObserver initialized for vehicle {vehicle_id} (Fleet size: {fleet_size})")

    def _should_update_local_observer(self, current_time: float) -> bool:
        """Check if local observer should update based on its rate"""
        if current_time - self._last_local_observer_time >= 1.0 / self.local_observer_rate:
            self._last_local_observer_time = current_time
            return True
        return False
    
    def _should_update_fleet_observer(self, current_time: float) -> bool:
        """Check if fleet observer should update based on its rate"""
        if current_time - self._last_fleet_observer_time >= 1.0 / self.fleet_observer_rate:
            self._last_fleet_observer_time = current_time
            return True
        return False

    def _get_observer_config(self) -> dict:
        """Get observer configuration with defaults."""
        default_config = {
            "local_observer_type": "ekf",
            "enable_distributed": True,
            "consensus_gain": 0.3
        }
        
        if self.config and 'observer' in self.config:
            return {**default_config, **self.config['observer']}
        return default_config

    def update_sensor_data(self, qcar):
        """
        Update sensor data from QCar hardware.
        This replaces the _update_sensor_data method from vehicle_logic.
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
                    self.sensor_data.update({
                        'motor_tach': qcar.motorTach,
                        'gyro_z': qcar.gyroscope[2] if hasattr(qcar, 'gyroscope') else 0.0,
                        'timestamp': time.time()
                    })
                
                return True
                
        except Exception as e:
            self.vehicle_logger.log_error("Sensor data update error", e)
            return False

    def update_observer(self, dt: float, last_steering: float = 0.0) -> dict:
        """
        Main observer update method that handles both local and fleet observer updates
        based on their individual update rates.
        
        Args:
            dt: Time step
            last_steering: Last steering command
            
        Returns:
            dict: Current state information compatible with vehicle_logic
        """
        current_time = time.time()
        state_info = self._get_default_state()
        
        try:
            # Update local observer if it's time
            if self._should_update_local_observer(current_time):
                state_info = self._update_local_observer(dt, last_steering)
            
            # # Update fleet observer if it's time (independent of local observer)
            # if self._should_update_fleet_observer(current_time):
            #     self._update_fleet_observer_internal(dt)
            
            return state_info
            
        except Exception as e:
            self.vehicle_logger.log_error("Observer update error", e)
            return self._get_default_state()

    def _update_local_observer(self, dt: float, last_steering: float = 0.0) -> dict:
        """
        Update local state estimation using the internal state estimator.
        This is called based on local observer update rate.
        
        Args:
            dt: Time step
            last_steering: Last steering command
            
        Returns:
            dict: Current state information compatible with vehicle_logic
        """
        try:
            if self.state_estimator is None:
                return self._get_default_state()
            
            # Update state estimate with sensor fusion
            self.state_estimator.update(
                motor_tach=self.sensor_data['motor_tach'],
                steering=last_steering,
                dt=dt,
                gyro_z=self.sensor_data['gyro_z']
            )
            
            # Get current state
            x, y, theta, velocity, state_valid = self.state_estimator.get_state()
            
            # Update local state
            with self.lock:
                self.local_state = np.array([x, y, theta, velocity])
                self.position = np.array([x, y, theta])
                self.velocity = velocity
                self.state_valid = state_valid
                self.sensor_data['state_valid'] = state_valid
            
            # Update own state in fleet estimates
            self.fleet_states[:, self.vehicle_id] = self.local_state.copy()
            
            return {
                'x': x, 'y': y, 'theta': theta, 'velocity': velocity,
                'state_valid': state_valid,
                'position': self.position.copy(),
                'local_state': self.local_state.copy()
            }
            
        except Exception as e:
            self.vehicle_logger.log_error("Local observer update error", e)
            return self._get_default_state()

    def _update_fleet_observer_internal(self, dt: float):
        """
        Update fleet observer estimates using received peer data.
        This is called based on fleet observer update rate.
        
        Args:
            dt: Time step
        """
        try:
            if not self.observer_config["enable_distributed"]:
                return
            
            current_time = time.time()
            
            # Update fleet estimates using consensus algorithm
            for vehicle_id in range(self.fleet_size):
                if vehicle_id == self.vehicle_id:
                    continue  # Skip own vehicle (updated in local observer)
                
                # Get latest received state for this vehicle
                latest_state = self._get_latest_received_state(vehicle_id, current_time)
                
                if latest_state is not None:
                    # Simple consensus update
                    consensus_gain = self.observer_config["consensus_gain"]
                    
                    # Update fleet estimate using consensus
                    current_estimate = self.fleet_states[:, vehicle_id].copy()
                    
                    # Consensus term: move estimate towards received state
                    consensus_update = consensus_gain * (latest_state - current_estimate)
                    
                    # Update fleet state
                    self.fleet_states[:, vehicle_id] = current_estimate + dt * consensus_update
            
            # Clean up old data
            self._cleanup_old_data(current_time)
            
        except Exception as e:
            self.vehicle_logger.log_error("Fleet observer update error", e)

    def set_state_estimator(self, state_estimator):
        """
        Set or change the state estimator.
        Allows different types of state estimators to be used.
        
        Args:
            state_estimator: StateEstimator instance (EKF, UKF, etc.)
        """
        self.state_estimator = state_estimator
        self.vehicle_logger.logger.info(f"State estimator set for vehicle {self.vehicle_id}: {type(state_estimator).__name__}")

    def initialize_state_estimator(self, gps, initial_pose=None, logger=None, use_ekf=True):
        """
        Initialize StateEstimator within VehicleObserver for better encapsulation.
        
        State Estimator could be any types (EKF, UKF , UIO , custom)...

        Args:
            gps: GPS instance
            initial_pose: Initial pose [x, y, theta] 
            logger: Logger instance
            use_ekf: Whether to use EKF
        
        Returns:
            bool: True if initialization successful
        """
        try:
            from Controller.controllers import StateEstimator
            
            state_estimator = StateEstimator(
                gps=gps,
                initial_pose=initial_pose,
                logger=self.vehicle_logger,
                use_ekf=use_ekf
            )
            
            self.set_state_estimator(state_estimator)
            
            self.vehicle_logger.logger.info(f"StateEstimator initialized within VehicleObserver for vehicle {self.vehicle_id}")
            
            return True
            
        except Exception as e:
            self.vehicle_logger.log_error("StateEstimator initialization failed in VehicleObserver", e)
            return False

    def get_state_estimator(self):
        """
        Get the current state estimator instance.
        
        Returns:
            StateEstimator instance or None
        """
        return self.state_estimator

    def add_received_state(self, sender_id: int, state: np.ndarray, timestamp: float) -> bool:
        """
        Add received state from another vehicle.
        
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
            
            with self.lock:
                # Add to received states list
                self.received_states[sender_id].append((timestamp, state.copy()))
                
                # Keep only recent data
                max_history = 10
                if len(self.received_states[sender_id]) > max_history:
                    self.received_states[sender_id] = self.received_states[sender_id][-max_history:]
            
            return True
            
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

    def is_state_valid(self) -> bool:
        """Check if current state is valid."""
        with self.lock:
            return self.state_valid

    def get_sensor_data(self) -> dict:
        """Get current sensor data."""
        with self.lock:
            return self.sensor_data.copy()

    def _get_latest_received_state(self, vehicle_id: int, current_time: float) -> Optional[np.ndarray]:
        """Get the latest received state for a vehicle."""
        if vehicle_id not in self.received_states:
            return None
        
        states_list = self.received_states[vehicle_id]
        if not states_list:
            return None
        
        # Get most recent state within time limit
        for timestamp, state in reversed(states_list):
            if current_time - timestamp <= self.max_state_age:
                return state
        
        return None

    def _cleanup_old_data(self, current_time: float):
        """Clean up old received data."""
        try:
            for vehicle_id in list(self.received_states.keys()):
                states_list = self.received_states[vehicle_id]
                
                # Remove old states
                valid_states = [(ts, state) for ts, state in states_list 
                               if current_time - ts <= self.max_state_age]
                
                if valid_states:
                    self.received_states[vehicle_id] = valid_states
                else:
                    # Remove vehicle if no valid states
                    del self.received_states[vehicle_id]
                    
        except Exception as e:
            self.vehicle_logger.log_error("Data cleanup error", e)

    def _get_default_state(self) -> dict:
        """Get default state when estimation fails."""
        return {
            'x': 0.0, 'y': 0.0, 'theta': 0.0, 'velocity': 0.0,
            'state_valid': False,
            'position': np.array([0.0, 0.0, 0.0]),
            'local_state': np.zeros(4)
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
                'state_valid': self.state_valid
            }
    
    def get_local_state_for_broadcast(self) -> dict:
        """
        Get local state information for V2V broadcasting.
        High-frequency, local sensor-based estimates.
        """
        with self.lock:
            return {
                'vehicle_id': self.vehicle_id,
                'x': float(self.local_state[0]),
                'y': float(self.local_state[1]),
                'theta': float(self.local_state[2]),
                'velocity': float(self.local_state[3]),
                'timestamp': time.time(),
                'state_valid': self.state_valid,
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
                if np.any(self.fleet_states[:, vehicle_id] != 0):  # Only include non-zero states
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
                'timestamp': time.time(),
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
            
            # Create new fleet states array with correct size
            new_fleet_states = np.zeros((self.state_dim, self.fleet_size))
            
            # Copy existing local state to new array
            if self.vehicle_id < self.fleet_size:
                new_fleet_states[:, self.vehicle_id] = self.local_state.copy()
            
            # Copy any existing peer states if they still exist
            for vehicle_id in range(min(old_fleet_size, self.fleet_size)):
                if vehicle_id != self.vehicle_id and vehicle_id < old_fleet_size:
                    new_fleet_states[:, vehicle_id] = self.fleet_states[:, vehicle_id].copy()
            
            self.fleet_states = new_fleet_states
            
            # Update peer tracking - clear states for vehicles not in new peer list
            current_peer_ids = set(self.received_states.keys())
            valid_peer_ids = set(peer_vehicle_ids)
            
            # Remove states for peers no longer in the fleet
            for peer_id in current_peer_ids - valid_peer_ids:
                if peer_id in self.received_states:
                    del self.received_states[peer_id]
            
            self.vehicle_logger.logger.info(f"VehicleObserver fleet estimation reinitialized: "
                           f"Fleet size: {old_fleet_size} -> {self.fleet_size}, "
                           f"Peer IDs: {peer_vehicle_ids}")

    def reset_observer(self, initial_pose: Optional[np.ndarray] = None):
        """Reset observer state."""
        with self.lock:
            if initial_pose is not None:
                self.local_state[:3] = initial_pose
                self.local_state[3] = 0.0
                self.position = initial_pose.copy()
            else:
                self.local_state = np.zeros(self.state_dim)
                self.position = np.zeros(3)
            
            self.velocity = 0.0
            self.state_valid = False
            
            # Reset fleet states
            for i in range(self.fleet_size):
                if i == self.vehicle_id:
                    self.fleet_states[:, i] = self.local_state.copy()
                else:
                    self.fleet_states[:, i] = np.zeros(self.state_dim)
            
            # Clear received data
            self.received_states.clear()

    def __del__(self):
        """Cleanup on destruction."""
        pass