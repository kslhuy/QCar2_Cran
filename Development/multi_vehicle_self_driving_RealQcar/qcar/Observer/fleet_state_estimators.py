"""
Fleet State Estimators for Distributed Observation

Provides different distributed state estimation strategies with a common interface.
Easy to switch between different algorithms (Consensus, Kalman Consensus, etc.).
"""
import numpy as np
import time
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple
from collections import defaultdict


class FleetStateEstimatorBase(ABC):
    """Base class for all fleet state estimators"""
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 4, 
                 config: Dict = None, logger=None):
        """
        Initialize base fleet estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 4: x, y, theta, v)
            config: Configuration dict
            logger: Logger instance
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.state_dim = state_dim
        self.config = config or {}
        self.logger = logger
        
        # Fleet state estimates [state_dim x fleet_size]
        self.fleet_states = np.zeros((state_dim, fleet_size))
        
        # Communication data storage
        self.received_states = defaultdict(list)  # vehicle_id -> [(timestamp_ns, state)]
        self.max_state_age_ns = int(1.0 * 1e9)  # 1 second in nanoseconds
    
    @abstractmethod
    def update(self, local_state: np.ndarray, dt: float, 
               current_time: int, control: np.ndarray) -> np.ndarray:
        """
        Update fleet state estimates
        
        Args:
            local_state: Host vehicle's local state estimate [state_dim]
            dt: Time step
            current_time: Current timestamp in nanoseconds
            control: Current control input [steering, throttle]
            
        Returns:
            Updated fleet states [state_dim x fleet_size]
        """
        pass
    
    @abstractmethod
    def add_received_local_state(self, sender_id: int, state: np.ndarray, timestamp_ns: int) -> bool:
        """
        Add received state from another vehicle
        
        Args:
            sender_id: ID of the vehicle that sent the state
            state: Received state [state_dim]
            timestamp_ns: Timestamp in nanoseconds
            
        Returns:
            True if successfully added
        """
        pass

    @abstractmethod
    def add_received_state(self, sender_id: int, state: np.ndarray, timestamp_ns: int) -> bool:
        """
        Add received state from another vehicle
        
        Args:
            sender_id: ID of the vehicle that sent the state
            state: Received state [state_dim]
            timestamp_ns: Timestamp in nanoseconds
            
        Returns:
            True if successfully added
        """
        pass
    
    def get_fleet_states(self) -> np.ndarray:
        """Get current fleet state estimates"""
        return self.fleet_states.copy()
    
    def get_vehicle_state(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get state estimate for specific vehicle"""
        if 0 <= vehicle_id < self.fleet_size:
            return self.fleet_states[:, vehicle_id].copy()
        return None
    
    def reset(self):
        """Reset fleet estimator"""
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
        self.received_states.clear()


class ConsensusFleetEstimator(FleetStateEstimatorBase):
    """
    Consensus-based distributed fleet estimator
    Simple and robust consensus algorithm
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 4,
                 config: Dict = None, logger=None):
        """
        Initialize consensus fleet estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension
            config: Configuration dict with 'consensus_gain'
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # Consensus gain (0-1, higher = faster consensus but less stable)
        self.consensus_gain = self.config.get('consensus_gain', 0.3)
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """Update fleet estimates using consensus
        
        Args:
            local_state: Own vehicle state [x, y, theta, v]
            dt: Time step in seconds
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
        """
        try:
            # Update own state in fleet - CRITICAL: Always keep own state current
            self.fleet_states[:, self.vehicle_id] = local_state.copy()
            
            # # Debug log every 50 updates
            # if hasattr(self, '_update_counter'):
            #     self._update_counter += 1
            # else:
            #     self._update_counter = 0
            
            # if self._update_counter % 50 == 0 and self.logger:
            #     self.logger.logger.info(
            #         f"ConsensusFleet update #{self._update_counter}: "
            #         f"vehicle_{self.vehicle_id} local_state=({local_state[0]:.3f}, {local_state[1]:.3f}, "
            #         f"{local_state[2]:.3f}, {local_state[3]:.3f})"
            #     )
            
            # Update estimates for other vehicles using consensus
            for vehicle_id in range(self.fleet_size):
                if vehicle_id == self.vehicle_id:
                    continue  # Skip own vehicle
                
                # Get latest received state
                latest_state = self._get_latest_received_state(vehicle_id, current_time_ns)
                
                if latest_state is not None:
                    # Simple consensus: move estimate towards received state
                    current_estimate = self.fleet_states[:, vehicle_id].copy()
                    
                    # Update fleet state with consensus 
                    self.fleet_states[:, vehicle_id] = current_estimate + self.consensus_gain * (latest_state - current_estimate)
                    
                    # # Debug log consensus updates every 50 iterations
                    # if self._update_counter % 50 == 0 and self.logger:
                    #     self.logger.logger.info(
                    #         f"ConsensusFleet: Updated vehicle_{vehicle_id} estimate - "
                    #         f"received: ({latest_state[0]:.3f}, {latest_state[1]:.3f}, {latest_state[2]:.3f}, {latest_state[3]:.3f}), "
                    #         f"new: ({self.fleet_states[0, vehicle_id]:.3f}, {self.fleet_states[1, vehicle_id]:.3f}, "
                    #         f"{self.fleet_states[2, vehicle_id]:.3f}, {self.fleet_states[3, vehicle_id]:.3f})"
                    #     )
            
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states.copy()
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Consensus fleet update error", e)
            return self.fleet_states.copy()
    
    def add_received_state(self, sender_id: int, state: np.ndarray, timestamp_ns: int) -> bool:
        """Add received state from another vehicle
        
        Args:
            sender_id: ID of sender vehicle
            state: Received state vector [x, y, theta, v]
            timestamp_ns: Timestamp in nanoseconds (directly from V2V message)
        """
        try:
            if sender_id == self.vehicle_id:
                return False  # Don't store own state
            
            # Store timestamp in nanoseconds - no conversion needed
            self.received_states[sender_id].append((timestamp_ns, state.copy()))
            
            # Keep only recent data (max 10 history)
            if len(self.received_states[sender_id]) > 10:
                self.received_states[sender_id] = self.received_states[sender_id][-10:]
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received state error", e)
            return False
    
    def _get_latest_received_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Get the latest received state for a vehicle
        
        Args:
            vehicle_id: ID of the target vehicle
            current_time_ns: Current time in nanoseconds
            
        Returns:
            Most recent state within max_state_age_ns, or None if no valid state
        """
        if vehicle_id not in self.received_states:
            return None
        
        states_list = self.received_states[vehicle_id]
        if not states_list:
            return None
        
        # Get most recent state within time limit (all in nanoseconds - no conversion!)
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns <= self.max_state_age_ns:
                return state
        
        return None
    
    def _cleanup_old_data(self, current_time_ns: int):
        """Clean up old received data"""
        try:
            for vehicle_id in list(self.received_states.keys()):
                states_list = self.received_states[vehicle_id]
                
                # Remove old states (all in nanoseconds)
                valid_states = [(ts_ns, state) for ts_ns, state in states_list 
                               if current_time_ns - ts_ns <= self.max_state_age_ns]
                
                if valid_states:
                    self.received_states[vehicle_id] = valid_states
                else:
                    del self.received_states[vehicle_id]
                    
        except Exception as e:
            if self.logger:
                self.logger.log_error("Data cleanup error", e)


class DistributedKalmanEstimator(FleetStateEstimatorBase):
    """
    Distributed Kalman Filter for fleet estimation
    More sophisticated, uses dynamics model and consensus
    Inspired by VehicleObserver.py _distributed_observer_each
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 4,
                 config: Dict = None, logger=None):
        """
        Initialize distributed Kalman estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension
            config: Configuration dict
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # Observer gains
        self.observer_gain = self.config.get('observer_gain', 0.1)
        self.consensus_gain = self.config.get('consensus_gain', 0.2)
        
        # Communication weights (uniform for now)
        self.weights = np.ones(fleet_size) / fleet_size
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """Update using distributed observer with dynamics
        
        Args:
            local_state: Own vehicle state [x, y, theta, v]
            dt: Time step in seconds
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
        """
        try:
            # Update own state in fleet
            self.fleet_states[:, self.vehicle_id] = local_state.copy()
            
            # Update estimates for other vehicles
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue
                
                # Distributed observer for this vehicle
                self.fleet_states[:, target_id] = self._distributed_observer_update(
                    target_id, current_time_ns, control, dt
                )
            
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states.copy()
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Distributed Kalman update error", e)
            return self.fleet_states.copy()
    
    def _distributed_observer_update(self, target_id: int, current_time_ns: int,
                                     control: np.ndarray, dt: float) -> np.ndarray:
        """
        Distributed observer update for one target vehicle
        Combines dynamics prediction, measurement correction, and consensus
        """
        # Current estimate
        x_ij = self.fleet_states[:, target_id].copy()
        
        # 1. Dynamics prediction (bicycle model)
        x, y, theta, v = x_ij
        steering, throttle = control[0], control[1] if len(control) > 1 else 0.0
        
        # Simple bicycle model
        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt
        theta_pred = theta + (v * np.tan(steering) / 1.0) * dt  # L=1.0m wheelbase
        v_pred = v + throttle * dt
        
        dynamics_term = np.array([x_pred, y_pred, theta_pred, v_pred])
        
        # 2. Measurement correction (if we have data from target)
        measurement_term = np.zeros(self.state_dim)
        latest_state = self._get_latest_received_state(target_id, current_time_ns)
        
        if latest_state is not None:
            # Measurement correction: L * (y - x_pred)
            measurement_error = latest_state - dynamics_term
            measurement_term = self.observer_gain * measurement_error
        
        # 3. Consensus term (simple for now - can be enhanced)
        consensus_term = np.zeros(self.state_dim)
        
        # Combine all terms
        x_ij_new = dynamics_term + measurement_term + consensus_term
        
        # Apply state constraints
        x_ij_new = self._apply_state_constraints(x_ij_new)
        
        return x_ij_new
    
    def _apply_state_constraints(self, state: np.ndarray) -> np.ndarray:
        """Apply physical constraints to state"""
        # Normalize angle to [-pi, pi]
        state[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))
        
        # Velocity constraints (reasonable for QCar)
        state[3] = np.clip(state[3], -2.0, 2.0)
        
        return state
    
    def add_received_state(self, sender_id: int, state: np.ndarray, timestamp_ns: int) -> bool:
        """Add received state from another vehicle
        
        Args:
            sender_id: ID of sender vehicle
            state: Received state vector [x, y, theta, v]
            timestamp_ns: Timestamp in nanoseconds (directly from V2V message)
        """
        try:
            if sender_id == self.vehicle_id:
                return False
            
            # Store timestamp in nanoseconds - no conversion needed
            self.received_states[sender_id].append((timestamp_ns, state.copy()))
            
            # Keep only recent data
            if len(self.received_states[sender_id]) > 10:
                self.received_states[sender_id] = self.received_states[sender_id][-10:]
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received state error", e)
            return False
    
    def _get_latest_received_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Get the latest received state for a vehicle
        
        Args:
            vehicle_id: ID of the target vehicle
            current_time_ns: Current time in nanoseconds
            
        Returns:
            Most recent state within max_state_age_ns, or None if no valid state
        """
        if vehicle_id not in self.received_states:
            return None
        
        states_list = self.received_states[vehicle_id]
        if not states_list:
            return None
        
        # All in nanoseconds - no conversion needed
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns <= self.max_state_age_ns:
                return state
        
        return None
    
    def _cleanup_old_data(self, current_time_ns: int):
        """Clean up old received data"""
        try:
            for vehicle_id in list(self.received_states.keys()):
                states_list = self.received_states[vehicle_id]
                
                # All in nanoseconds
                valid_states = [(ts_ns, state) for ts_ns, state in states_list 
                               if current_time_ns - ts_ns <= self.max_state_age_ns]
                
                if valid_states:
                    self.received_states[vehicle_id] = valid_states
                else:
                    del self.received_states[vehicle_id]
                    
        except Exception as e:
            if self.logger:
                self.logger.log_error("Data cleanup error", e)

class DistributedLubergerEstimator(FleetStateEstimatorBase):
    """
    Distributed Lunberger Observer for fleet longitudinal estimation
    More sophisticated, uses dynamics model and consensus
    Inspired by VehicleObserver.py _distributed_observer_each
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 3,
                 config: Dict = None, logger=None):
        """
        Initialize distributed Lunberger Observer
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension
            config: Configuration dict
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # System matrices of the longutinal model
        Ai = np.array([0, 1, 0],
                     [0, 0, 1],
                     [0, 0, 0])
        self.A = np.block([
                        [Ai if i == j else np.zeros_like(Ai) for j in range(fleet_size)]
                        for i in range(fleet_size)
                    ])
        Bi = np.array(0, 0, 1)
        self.B = np.block([
            [Bi if i == j else np.zeros_like(Bi) for j in range(fleet_size)]
            for i in range(fleet_size)
        ])

        self.m_i = 0.5 # kg
        self.tau_i = 0.16 
        self.rho_i = 0.12
        self.Cd_i = 0.035 
        self.AF_i = 0.22
        self.mu_i = 0.01

        # Observer gains
        self.observer_gain = self.config.get('observer_gain', 0.1)
        self.consensus_gain = self.config.get('consensus_gain', 0.2)
        
        # Communication weights (uniform for now)
        self.weights = np.ones(fleet_size) / fleet_size
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """Update using distributed observer with dynamics
        
        Args:
            local_state: Own vehicle state [x, y, theta, v]
            dt: Time step in seconds
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
        """
        try:
            # Update own state in fleet
            self.fleet_states[:, self.vehicle_id] = local_state.copy()
            
            # Update estimates for other vehicles
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue
                
                # Distributed observer for this vehicle
                self.fleet_states[:, target_id] = self._distributed_observer_update(
                    target_id, current_time_ns, control, dt
                )
            
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states.copy()
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Distributed Kalman update error", e)
            return self.fleet_states.copy()
    
    def _distributed_observer_update(self, target_id: int, current_time_ns: int,
                                     collective_control: np.ndarray, dt: float) -> np.ndarray:
        """
        Distributed observer update for one target vehicle
        Combines dynamics prediction, measurement correction, and consensus
        """
        # Distributed observer dimention
        dim_distributed_observer = self.state_dim * self.fleet_size
        # Current estimate
        x_i = self.fleet_states.copy()

        
        # 1. Dynamics prediction (collective longitudinal model)
        u = collective_control if len(collective_control) > self.fleet_size else 0.0
        
        # collective longitudinal model
        # dx = Ax + Bf
        f = np.zeros(self.fleet_size)
        for i in range(self.fleet_size):
            state = self._get_latest_received_state(i, current_time_ns)
            v_i = state[4]
            a_i = state[5]
            f[i] = self._get_nonlinear_term_phi_i(v_i, a_i)
        
        dynamics_term = x_i + (self.A @ x_i + self.B @ f) * dt
        
        # 2. Measurement correction (if we have data from target)
        measurement_term = np.zeros(self.state_dim)
        # Question: How to make sure what's the meaning of the state_list in _get_latest_received_state?
        local_measurement = self._get_latest_received_state(self.vehicle_id, current_time_ns) # Assume all the state in the host vehicle is measurable
        measure_position = local_measurement[0] # Get the local measurement from all the avilable measurement

        if local_measurement is not None:
            # Measurement correction: L * (y - C * x_pred)
            # estimated_measurement = C @ dynamics_term
            # measurement_error = local_measurement - estimated_measurement
            estimated_position = dynamics_term[self.vehicle_id + self.vehicle_id * self.state_dim] # Catch the estimated position from the observer state
            measurement_error = measure_position - estimated_position
            measurement_term = self.observer_gain * measurement_error

        # 3. Consensus term (simple for now - can be enhanced)
        # To Do : use the _get_latest_received_state to get the neibour's state, it is already the real communicarion
        consensus_term = np.zeros(dim_distributed_observer)
        
        # Combine all terms
        x_i_new = dynamics_term + measurement_term + consensus_term
        
        return x_i_new
    
    def add_received_state(self, sender_id: int, state: np.ndarray, timestamp_ns: int) -> bool:
        """Add received state from another vehicle
        
        Args:
            sender_id: ID of sender vehicle
            state: Received state vector [x, y, theta, v, acceleration]
            timestamp_ns: Timestamp in nanoseconds (directly from V2V message)
        """
        try:
            if sender_id == self.vehicle_id:
                return False
            
            # Store timestamp in nanoseconds - no conversion needed
            self.received_states[sender_id].append((timestamp_ns, state.copy()))
            
            # Keep only recent data
            if len(self.received_states[sender_id]) > 10:
                self.received_states[sender_id] = self.received_states[sender_id][-10:]
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received state error", e)
            return False
    
    def _get_latest_received_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Get the latest received state for a vehicle
        
        Args:
            vehicle_id: ID of the target vehicle
            current_time_ns: Current time in nanoseconds
            
        Returns:
            Most recent state within max_state_age_ns, or None if no valid state
        """
        if vehicle_id not in self.received_states:
            return None
        
        states_list = self.received_states[vehicle_id]
        if not states_list:
            return None
        
        # All in nanoseconds - no conversion needed
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns <= self.max_state_age_ns:
                return state
        
        return None
    
    def _cleanup_old_data(self, current_time_ns: int):
        """Clean up old received data"""
        try:
            for vehicle_id in list(self.received_states.keys()):
                states_list = self.received_states[vehicle_id]
                
                # All in nanoseconds
                valid_states = [(ts_ns, state) for ts_ns, state in states_list 
                               if current_time_ns - ts_ns <= self.max_state_age_ns]
                
                if valid_states:
                    self.received_states[vehicle_id] = valid_states
                else:
                    del self.received_states[vehicle_id]
                    
        except Exception as e:
            if self.logger:
                self.logger.log_error("Data cleanup error", e)
    
    def _get_nonlinear_term_phi_i(self, v_i, a_i, g_s: float = 9.81):
        """
        Compute the nonlinear term phi_i(v_i, a_i) using vehicle parameters on this estimator.

        Args:
            v_i: Velocity (scalar or numpy array)
            a_i: Acceleration (scalar or numpy array)
            g_s: Gravitational acceleration constant
        """
        v_i = np.asarray(v_i)
        a_i = np.asarray(a_i)

        try:
            m_i = self.m_i
            tau_i = self.tau_i
            rho_i = self.rho_i
            Cd_i = self.Cd_i
            AF_i = self.AF_i
            mu_i = self.mu_i
        except AttributeError as exc:
            raise AttributeError(
                "Vehicle parameters (m_i, tau_i, rho_i, Cd_i, AF_i, mu_i) must be set on the estimator before calling nonlear_term_phi_i"
            ) from exc

        # Aerodynamic drag term: - (1 / (2 m_i tau_i)) * rho_i * Cd_i * AF_i * (v_i^2 + 2 tau_i v_i a_i)
        drag_term = -(rho_i * Cd_i * AF_i / (2.0 * m_i * tau_i)) * (v_i**2 + 2.0 * tau_i * v_i * a_i)

        # Acceleration damping term: - (1 / tau_i) * a_i
        accel_term = -(1.0 / tau_i) * a_i

        # Grade/rolling resistance term: + (1 / tau_i) * mu_i * g_s
        grade_term = (1.0 / tau_i) * mu_i * g_s

        return drag_term + accel_term + grade_term

class FleetEstimatorFactory:
    """Factory to create fleet state estimators by name"""
    
    ESTIMATOR_TYPES = {
        'consensus': ConsensusFleetEstimator,
        'distributed_kalman': DistributedKalmanEstimator,
    }
    
    @staticmethod
    def create(estimator_type: str, vehicle_id: int, fleet_size: int,
               state_dim: int = 4, config: Dict = None, logger=None):
        """
        Create a fleet state estimator
        
        Args:
            estimator_type: One of 'consensus', 'distributed_kalman'
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 4)
            config: Configuration dict
            logger: Logger instance
            
        Returns:
            Fleet state estimator instance
        """
        if estimator_type not in FleetEstimatorFactory.ESTIMATOR_TYPES:
            raise ValueError(
                f"Unknown fleet estimator type: {estimator_type}. "
                f"Available: {list(FleetEstimatorFactory.ESTIMATOR_TYPES.keys())}"
            )
        
        estimator_class = FleetEstimatorFactory.ESTIMATOR_TYPES[estimator_type]
        return estimator_class(
            vehicle_id=vehicle_id,
            fleet_size=fleet_size,
            state_dim=state_dim,
            config=config,
            logger=logger
        )
