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
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5, 
                 config: Dict = None, logger=None):
        """
        Initialize base fleet estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5: x, y, theta, v, a)
            config: Configuration dict
            logger: Logger instance
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.state_dim = state_dim
        self.config = config or {}
        self.logger = logger
        
        # Fleet state estimates [state_dim x fleet_size]
        # state_dim = 5 for [x, y, theta, v, a]
        self.fleet_states = np.zeros((state_dim, fleet_size))
        
        # Communication data storage
        self.received_local_states = defaultdict(list)  # vehicle_id -> [(timestamp_ns, state)]

        self.received_fleet_states = defaultdict(list) # vehicle_sender_id -> [(timestamp_ns, fleet_state)]


        self.max_state_age_ns = int(1.0 * 1e9)  # 1 second in nanoseconds
    
    @abstractmethod
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
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
    
    def add_received_local_state(self, sender_id: int, state: np.ndarray, timestamp_ns: int) -> bool:
        """Add a received LOCAL state from another vehicle and store it in history.

        Default implementation used by most estimators. Subclasses may override
        when custom validation or handling is required.
        """
        try:
            if sender_id == self.vehicle_id:
                return False  # Don't store own state

            # Validate state dimension
            if state.shape[0] != self.state_dim:
                if self.logger:
                    self.logger.log_error(
                        f"State dimension mismatch: expected {self.state_dim}, got {state.shape[0]}"
                    )
                return False

            # Store timestamp in nanoseconds
            self.received_local_states[sender_id].append((timestamp_ns, state.copy()))

            # Keep only recent history (default 10 entries)
            if len(self.received_local_states[sender_id]) > 10:
                self.received_local_states[sender_id] = self.received_local_states[sender_id][-10:]

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received local state error", e)
            return False

    def add_received_fleet_state(self, sender_id: int, fleet_estimates: Dict, timestamp_ns: int) -> bool:
        """Add a received FLEET state broadcast from another vehicle.

        Default implementation stores the raw fleet dictionary in
        `received_fleet_states` and also unpacks per-vehicle states into
        `received_states` for easy access by other algorithms.
        """
        try:
            if sender_id == self.vehicle_id:
                return False

            # Check for new vehicle IDs and expand capacity if required
            try:
                max_id_in_msg = max((int(vid) for vid in fleet_estimates.keys()), default=0)
            except Exception:
                max_id_in_msg = 0

            if max_id_in_msg >= self.fleet_size:
                self._ensure_fleet_capacity(max_id_in_msg)

            # Store the raw fleet dictionary with timestamp
            self.received_fleet_states[sender_id].append((timestamp_ns, fleet_estimates))


            # Limit history for fleet snapshots per neighbor (default 5)
            if len(self.received_fleet_states[sender_id]) > 5:
                self.received_fleet_states[sender_id] = self.received_fleet_states[sender_id][-5:]

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received fleet state error", e)
            return False

    def _get_latest_fleet_data(self, neighbor_id: int, current_time_ns: int) -> Optional[Dict]:
        """Return the newest fleet dictionary from a neighbor that is still valid.

        Returns None if there is no recent valid snapshot.
        """
        if neighbor_id not in self.received_fleet_states:
            return None

        history = self.received_fleet_states[neighbor_id]
        if not history:
            return None

        # Iterate backwards to find newest valid data
        for ts_ns, fleet_data in reversed(history):
            if (current_time_ns - ts_ns) <= self.max_state_age_ns:
                return fleet_data
        return None

    def _get_latest_received_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Return the most recent received state for `vehicle_id` within the age limit.

        Returns None if no recent state is available.
        """
        if vehicle_id not in self.received_local_states:
            return None

        states_list = self.received_local_states[vehicle_id]
        if not states_list:
            return None

        # Iterate backwards (newest first) and return first valid entry
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns <= self.max_state_age_ns:
                return state

        return None

    
    def get_fleet_states(self) -> np.ndarray:
        """Get current fleet state estimates"""
        return self.fleet_states.copy()
    
    def get_vehicle_state(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get state estimate for specific vehicle"""
        if 0 <= vehicle_id < self.fleet_size:
            return self.fleet_states[:, vehicle_id].copy()
        return None
    
    # Problem: Car_2 was trying to access index 2 in a fleet_states array that only had size 2 (indices 0-1), 
    # causing IndexError: index 2 is out of bounds for axis 1 with size 2.

    # Root Cause: When V2V activated with 2 cars (Car_0 and Car_1), the fleet was initialized with size 2. 
    # When Car_2 later joined, it tried to write its state to index 2, which didn't exist.

    # Solution: Added auto-expansion logic to fleet state estimators:

        # 1  Added _ensure_fleet_capacity() helper method to base class
        # 2  Modified ConsensusFleetEstimator.update() to auto-expand before writing
        # 3  Modified DistributedKalmanEstimator.update() to auto-expand and also update weights array
        # 4  The fleet_states array now dynamically grows to accommodate new vehicles with higher IDs
    
    def _ensure_fleet_capacity(self, min_vehicle_id: int):
        """
        Ensure fleet_states array can accommodate the given vehicle_id.
        Auto-expands the array if needed.
        
        Args:
            min_vehicle_id: Minimum vehicle ID that must be accommodated
        """
        if min_vehicle_id >= self.fleet_states.shape[1]:
            old_fleet_size = self.fleet_states.shape[1]
            new_fleet_size = min_vehicle_id + 1
            
            # Expand fleet_states array
            new_fleet_states = np.zeros((self.state_dim, new_fleet_size))
            new_fleet_states[:, :old_fleet_size] = self.fleet_states
            self.fleet_states = new_fleet_states
            self.fleet_size = new_fleet_size
            
            if self.logger:
                self.logger.logger.info(
                    f"{self.__class__.__name__}: Auto-expanded fleet size {old_fleet_size} -> {new_fleet_size} "
                    f"to accommodate vehicle_{min_vehicle_id}"
                )
    
    def reset(self):
        """Reset fleet estimator"""
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
        self.received_local_states.clear()
        self.received_fleet_states.clear()

    def _cleanup_old_data(self, current_time_ns: int):
        """Clean up old entries from both received_states and received_fleet_states."""
        try:
            for vehicle_id in list(self.received_local_states.keys()):
                states_list = self.received_local_states[vehicle_id]

                # Remove old states (all in nanoseconds)
                valid_states = [(ts_ns, state) for ts_ns, state in states_list
                               if current_time_ns - ts_ns <= self.max_state_age_ns]

                if valid_states:
                    self.received_local_states[vehicle_id] = valid_states
                else:
                    del self.received_local_states[vehicle_id]


            # Clean fleet snapshots
            for sender_id in list(self.received_fleet_states.keys()):
                history = self.received_fleet_states[sender_id]
                valid_history = [
                    (ts, data) for ts, data in history
                    if (current_time_ns - ts) <= self.max_state_age_ns
                ]

                if valid_history:
                    self.received_fleet_states[sender_id] = valid_history
                else:
                    del self.received_fleet_states[sender_id]

        except Exception as e:
            if self.logger:
                self.logger.log_error("Data cleanup error", e)
class ConsensusFleetEstimator(FleetStateEstimatorBase):
    """
    Consensus-based distributed fleet estimator
    Simple and robust consensus algorithm
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        """
        Initialize consensus fleet estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5: x, y, theta, v, a)
            config: Configuration dict with 'consensus_gain'
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # Consensus gain (0-1, higher = faster consensus but less stable)
        self.consensus_gain = self.config.get('consensus_gain', 0.3)
        self.direct_gain = self.config.get('direct_gain', 0.5)
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """
        Update fleet estimates using Consensus + Direct measurements
        
        Algorithm for Target T (estimated by Host H):
        New_Est(T) = Old_Est(T) 
                   + k_consensus * Sum(Neighbor_N's Est(T) - Host_H's Est(T))
                   + k_direct    * (Target_T's Self_Report - Host_H's Est(T))
        """
        try:
            # 1. Ensure capacity and set own state (Ground Truth for self)
            self._ensure_fleet_capacity(self.vehicle_id)
            self.fleet_states[:, self.vehicle_id] = local_state.copy()
            
            # 2. Update estimates for every other vehicle in the fleet
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue  # Skip self
                
                # --- Step A: Get Current Estimate ---
                current_est = self.fleet_states[:, target_id].copy()
                total_correction = np.zeros_like(current_est)
                
                # --- Step B: Calculate Consensus Term (What neighbors think of Target) ---
                # "I trust my neighbors N1, N2... to tell me where Target T is"
                neighbor_count = 0
                consensus_accum = np.zeros_like(current_est)
                
                for neighbor_id, history in self.received_fleet_states.items():
                    # Get neighbor's latest fleet view
                    neighbor_fleet_dict = self._get_latest_fleet_data(neighbor_id, current_time_ns)
                    
                    if neighbor_fleet_dict and target_id in neighbor_fleet_dict:
                        # Extract what Neighbor thinks of Target
                        neigh_est_dict = neighbor_fleet_dict[target_id]
                        neigh_est_vec = np.array([
                            neigh_est_dict['x'], neigh_est_dict['y'], 
                            neigh_est_dict['theta'], neigh_est_dict['velocity'],
                            neigh_est_dict.get('acceleration', 0.0)
                        ])
                        
                        # Add difference (Neighbor - Self)
                        consensus_accum += (neigh_est_vec - current_est)
                        neighbor_count += 1
                
                if neighbor_count > 0:
                    # Average the consensus difference and apply gain
                    # Using average prevents gain from exploding with many neighbors
                    total_correction += self.consensus_gain * (consensus_accum / neighbor_count)

                # --- Step C: Calculate Direct Term (What Target says about itself) ---
                # "I trust Target T to tell me where Target T is" (Highest Confidence)
                direct_state = self._get_latest_received_state(target_id, current_time_ns)
                
                if direct_state is not None:
                    # Innovation: Direct_Broadcast - Current_Estimate
                    total_correction += self.direct_gain * (direct_state - current_est)
                
                # --- Step D: Apply Update ---
                self.fleet_states[:, target_id] = current_est + total_correction

            # 3. Cleanup old data from both storages
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states.copy()
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Consensus fleet update error", e)
            return self.fleet_states.copy()
    



class DistributedKalmanEstimator(FleetStateEstimatorBase):
    """
    Distributed Kalman Filter for fleet estimation
    More sophisticated, uses dynamics model and consensus
    Inspired by VehicleObserver.py _distributed_observer_each
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        """
        Initialize distributed Kalman estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5: x, y, theta, v, a)
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
            local_state: Own vehicle state [x, y, theta, v, a]
            dt: Time step in seconds
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
        """
        try:
            # Ensure fleet capacity for this vehicle and expand weights if needed
            if self.vehicle_id >= self.fleet_states.shape[1]:
                old_fleet_size = self.fleet_states.shape[1]
                self._ensure_fleet_capacity(self.vehicle_id)
                
                # Expand weights array to match new fleet size
                new_weights = np.ones(self.fleet_size) / self.fleet_size
                new_weights[:old_fleet_size] = self.weights[:old_fleet_size] * (old_fleet_size / self.fleet_size)
                self.weights = new_weights
            
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
    


class DistributedLuenbergerEstimator(FleetStateEstimatorBase):
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
        Ai = np.array([[0, 1, 0],
                     [0, 0, 1],
                     [0, 0, 0]])
        self.A = np.block([
                        [Ai if i == j else np.zeros_like(Ai) for j in range(fleet_size)]
                        for i in range(fleet_size)
                    ])
        Bi = np.array([0, 0, 1])
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
            
            
            
                
            # Distributed observer for this vehicle
            self.fleet_states = self._distributed_observer_update(
                current_time_ns, control, dt
            )
            
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Distributed Kalman update error", e)
            return self.fleet_states
    
    def _distributed_observer_update(self, current_time_ns: int,
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
            v_i = state.get("velocity", 0.0)
            a_i = state.get("acceleration", 0.0)
            v_i = state[4] # Test value
            a_i = state[5] # Test value
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
            estimated_index = self.vehicle_id + self.vehicle_id * self.state_dim
            estimated_position = dynamics_term[estimated_index] # Catch the estimated position from the observer state
            measurement_error = measure_position - estimated_position
            measurement_term = self.observer_gain * measurement_error

        # 3. Consensus term (simple for now - can be enhanced)
        # TODO : use the _get_latest_received_state to get the neibour's state, it is already the real communicarion
        consensus_term = np.zeros(dim_distributed_observer)
        
        # Combine all terms
        x_i_new = dynamics_term + measurement_term + consensus_term
        x_i_new = 0.1217 # Test value
        
        return x_i_new

    
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
        'distributed_luenberger': DistributedLuenbergerEstimator,
    }
    
    @staticmethod
    def create(estimator_type: str, vehicle_id: int, fleet_size: int,
               state_dim: int = 5, config: Dict = None, logger=None):
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
