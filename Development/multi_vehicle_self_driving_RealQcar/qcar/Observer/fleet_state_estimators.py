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

# Canonical ordering for state fields used in V2V/local state messages.
STATE_FIELDS = ("x", "y", "theta", "velocity", "acceleration")


def _normalize_state_array(state_array, expected_dim: int, logger=None) -> Optional[np.ndarray]:
    """
    Convert any array-like to a 1D numpy array with the expected dimension.
    Pads with zeros or truncates if needed to avoid shape errors in downstream algorithms.
    """
    try:
        arr = np.asarray(state_array, dtype=float).flatten()
        if arr.shape[0] == expected_dim:
            return arr
        if arr.shape[0] > expected_dim:
            if logger:
                logger.logger.debug(
                    f"State dim {arr.shape[0]} larger than expected {expected_dim}, truncating"
                )
            return arr[:expected_dim]
        # Pad with zeros when shorter than expected
        if logger:
            logger.logger.debug(
                f"State dim {arr.shape[0]} smaller than expected {expected_dim}, padding with zeros"
            )
        return np.pad(arr, (0, expected_dim - arr.shape[0]), mode="constant")
    except Exception as exc:
        if logger:
            logger.log_error("Failed to normalize state array", exc)
        return None


def _state_dict_to_array(state_dict: Dict, expected_dim: int, logger=None) -> Optional[np.ndarray]:
    """
    Convert a state dict (as used in communication/log layers) to ndarray in canonical order.
    """
    try:
        base_arr = np.array([state_dict.get(k, 0.0) for k in STATE_FIELDS], dtype=float)
        return _normalize_state_array(base_arr, expected_dim, logger=logger)
    except Exception as exc:
        if logger:
            logger.log_error("Failed to convert state dict to array", exc)
        return None


class FleetStateEstimatorBase(ABC):
    """Base class for all fleet state estimators"""
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5, 
                 config: Dict = None, logger=None):
        """
        Initialize base fleet estimator
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5: x, y, theta, v, a) based on vehicle model
            config: Configuration dict
            logger: Logger instance
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.state_dim = state_dim
        self.config = config or {}
        self.logger = logger
        
        # Fleet state estimates [state_dim x fleet_size]
        self.fleet_states = np.zeros((self.state_dim, fleet_size))
        print(f"fleet_states initialized with shape: {self.fleet_states.shape}")
        
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
    
    def add_received_local_state(self, sender_id: int, state: Dict, timestamp_ns: int) -> bool:
        """Add a received LOCAL state (dict or ndarray) and store ndarray in history.

        Communication/log layers hand us dicts; algorithms want numpy arrays.
        We normalize here so downstream consumers always see ndarray.
        """
        # # print(f"Adding received local state from vehicle_id {sender_id}")
        try:
            if sender_id == self.vehicle_id:
                return False  # Don't store own state

            state_vec: Optional[np.ndarray] = None
            if isinstance(state, dict):
                state_vec = _state_dict_to_array(state, self.state_dim, logger=self.logger)
            else:
                state_vec = _normalize_state_array(state, self.state_dim, logger=self.logger)

            if state_vec is None:
                return False

            # Store timestamp in nanoseconds (keep ndarray only)
            self.received_local_states[sender_id].append((timestamp_ns, state_vec.copy()))

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
        """Return the most recent received state (as ndarray) within the age limit.

        Returns None if no recent state is available or conversion fails.
        """
        if vehicle_id not in self.received_local_states:
            print(f"vehicle_id {vehicle_id} not in self.received_local_states")
            return None

        states_list = self.received_local_states[vehicle_id]
        if not states_list:
            print(f"states_list for vehicle_id {vehicle_id} is empty")
            return None

        # Iterate backwards (newest first) and return first valid entry
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns > self.max_state_age_ns:
                continue

            # History stores ndarray; older entries might still be dict, so normalize defensively
            if isinstance(state, dict):
                state_vec = _state_dict_to_array(state, self.state_dim, logger=self.logger)
            else:
                state_vec = _normalize_state_array(state, self.state_dim, logger=self.logger)

            if state_vec is not None:
                return state_vec
            
        print(f"No valid recent state for vehicle_id {vehicle_id}")
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
    



class DistributedLuenbergerEstimator(FleetStateEstimatorBase):
    """
    Distributed Luenberger Observer for fleet longitudinal estimation
    More sophisticated, uses dynamics model and consensus
    Inspired by VehicleObserver.py _distributed_observer_each
    
    Communication Topology:
    - Uses adjacency matrix to define which FOLLOWER vehicles can communicate
    - Adjacency matrix is [observer_size x observer_size], NOT including leader (vehicle 0)
    - Matrix indices 0..observer_size-1 correspond to follower vehicles 1..observer_size
    - Default: Chain topology (follower i talks to follower i-1 and i+1)
    - Can override with config['adjacency_matrix']
    - Consensus term only considers neighbors defined in adjacency matrix
    
    Example config for 4-vehicle fleet (1 leader + 3 followers):
        config = {
            'observer_gain': 0.1,
            'consensus_gain': 0.2,
            'adjacency_matrix': [  # Optional: custom topology [3x3] for 3 followers
                [0, 1, 0],  # Follower 1 (vehicle 1) connected to follower 2 (vehicle 2)
                [1, 0, 1],  # Follower 2 (vehicle 2) connected to followers 1 and 3
                [0, 1, 0]   # Follower 3 (vehicle 3) connected to follower 2 (vehicle 2)
            ]
        }
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        """
        Initialize distributed Luenberger Observer
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet, including the leader labeled 0
            state_dim: State dimension (default 5: x, y, theta, v, a) based on vehicle model
            config: Configuration dict
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # System matrices of the longitudinal model
        tau = 0.16  # Time constant
        self.h = 1.0     # time headway
        self.d = 0.8    # Desired distance
        self.observer_size = self.fleet_size - 1  # Exclude leader from observer size

        A_tau = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0/tau],
        ])

        B_tau = np.array([
            [0.0],
            [0.0],
            [1.0/tau],
        ])

        A_h = np.array([
            [0.0, 0.0, self.h],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        # B_delta = blkdiag(B_tau, B_tau, B_tau)
        self.B_delta = np.block([
            [B_tau,              np.zeros((self.observer_size, 1)), np.zeros((self.observer_size, 1))],
            [np.zeros((self.observer_size, 1)),  B_tau,             np.zeros((self.observer_size, 1))],
            [np.zeros((self.observer_size, 1)),  np.zeros((self.observer_size, 1)), B_tau],
        ])

        A_h_tau = A_h + A_tau

        # A_delta = [A_h_tau, 0, 0; A_h, A_h_tau, 0; A_h, A_h, A_h_tau]
        Z = np.zeros((self.observer_size, self.observer_size))
        self.A_delta = np.block([
            [A_h_tau, Z,      Z],
            [A_h,     A_h_tau, Z],
            [A_h,     A_h,    A_h_tau],
        ])

        self.Cv = np.array([-self.h,1])
        self.Cd = np.array([-1,0])
        self.Cf = np.array([[1, -self.h, 0],
                            [0, 1, 0]])
        self.Cp = np.array([[-1, 0, 0],
                            [0, 0, 0]])
        self.local_measurement_dim = 2 # measuring relative position and velocity pi - pi-1 vi

        # Observer gains
        self.observer_gain = self.config.get('observer_gain', 0.1)
        self.consensus_gain = self.config.get('consensus_gain', 0.2)
        
        self.logger.logger.info(
            f"Vehicle {self.vehicle_id}: Observer gain: {self.observer_gain}, Consensus gain: {self.consensus_gain}"
        )

        # Communication weights (uniform for now)
        self.weights = np.ones(self.observer_size) / self.observer_size
        
        # Communication adjacency matrix - defines which FOLLOWER vehicles can communicate
        # Note: Adjacency matrix is [observer_size x observer_size], NOT including leader (vehicle 0)
        # Matrix indices 0..observer_size-1 correspond to follower vehicles 1..observer_size
        # Default: chain topology for platoon (each follower talks to neighbors)
        # Can be overridden by config['adjacency_matrix']
        if 'adjacency_matrix' in self.config:
            self.adjacency_matrix = np.array(self.config['adjacency_matrix'])
        else:
            # Default chain topology: follower i connected to follower i-1 and i+1
            self.adjacency_matrix = self._create_chain_topology()

        # 🔧 添加：打印邻接矩阵用于调试
        if self.logger:
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Adjacency matrix initialized "
                f"[{self.observer_size}x{self.observer_size}]:\n{self.adjacency_matrix}"
            )

        # Validate adjacency matrix dimensions (should be observer_size, not fleet_size)
        if self.adjacency_matrix.shape != (self.observer_size, self.observer_size):
            if self.logger:
                self.logger.logger.warning(
                    f"Adjacency matrix shape {self.adjacency_matrix.shape} doesn't match observer_size {self.observer_size}. "
                    f"Using default chain topology."
                )
            self.adjacency_matrix = self._create_chain_topology()
        
        # 🔧 在初始化阶段计算并缓存邻居列表
        self.my_neighbors = self.get_neighbors(self.vehicle_id)
        
        # 🔧 在初始化阶段计算并缓存输出矩阵Ci
        self.Ci = self.compute_output_matrix_Ci(self.vehicle_id)
        
        # 🔧 在日志中打印邻居列表和矩阵信息
        if self.logger:
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Neighbors list initialized: {self.my_neighbors}"
            )
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Output matrix Ci computed with shape: {self.Ci.shape}"
            )
    
    def _create_chain_topology(self) -> np.ndarray:
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
    Distributed Luenberger Observer for fleet longitudinal estimation
    More sophisticated, uses dynamics model and consensus
    Inspired by VehicleObserver.py _distributed_observer_each
    
    Communication Topology:
    - Uses adjacency matrix to define which FOLLOWER vehicles can communicate
    - Adjacency matrix is [observer_size x observer_size], NOT including leader (vehicle 0)
    - Matrix indices 0..observer_size-1 correspond to follower vehicles 1..observer_size
    - Default: Chain topology (follower i talks to follower i-1 and i+1)
    - Can override with config['adjacency_matrix']
    - Consensus term only considers neighbors defined in adjacency matrix
    
    Example config for 4-vehicle fleet (1 leader + 3 followers):
        config = {
            'observer_gain': 0.1,
            'consensus_gain': 0.2,
            'adjacency_matrix': [  # Optional: custom topology [3x3] for 3 followers
                [0, 1, 0],  # Follower 1 (vehicle 1) connected to follower 2 (vehicle 2)
                [1, 0, 1],  # Follower 2 (vehicle 2) connected to followers 1 and 3
                [0, 1, 0]   # Follower 3 (vehicle 3) connected to follower 2 (vehicle 2)
            ]
        }
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        """
        Initialize distributed Luenberger Observer
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension
            config: Configuration dict
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)
        
        # System matrices of the longitudinal model
        tau = 0.16  # Time constant
        self.h = 1.0     # time headway
        self.d = 0.8    # Desired distance
        self.observer_size = self.fleet_size - 1  # Exclude leader from observer size

        A_tau = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0/tau],
        ])

        B_tau = np.array([
            [0.0],
            [0.0],
            [1.0/tau],
        ])

        A_h = np.array([
            [0.0, 0.0, self.h],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        # B_delta = blkdiag(B_tau, B_tau, B_tau)
        self.B_delta = np.block([
            [B_tau,              np.zeros((self.observer_size, 1)), np.zeros((self.observer_size, 1))],
            [np.zeros((self.observer_size, 1)),  B_tau,             np.zeros((self.observer_size, 1))],
            [np.zeros((self.observer_size, 1)),  np.zeros((self.observer_size, 1)), B_tau],
        ])

        A_h_tau = A_h + A_tau

        # A_delta = [A_h_tau, 0, 0; A_h, A_h_tau, 0; A_h, A_h, A_h_tau]
        Z = np.zeros((self.observer_size, self.observer_size))
        self.A_delta = np.block([
            [A_h_tau, Z,      Z],
            [A_h,     A_h_tau, Z],
            [A_h,     A_h,    A_h_tau],
        ])

        self.Cv = np.array([-self.h,1])
        self.Cd = np.array([-1,0])
        self.Cf = np.array([[1, -self.h, 0],
                            [0, 1, 0]])
        self.Cp = np.array([[-1, 0, 0],
                            [0, 0, 0]])
        self.local_measurement_dim = 2 # measuring relative position and velocity pi - pi-1 vi

        # Observer gains
        self.observer_gain = self.config.get('observer_gain', 0.1)
        self.consensus_gain = self.config.get('consensus_gain', 0.2)
        
        # Communication weights (uniform for now)
        self.weights = np.ones(self.observer_size) / self.observer_size
        
        # Communication adjacency matrix - defines which FOLLOWER vehicles can communicate
        # Note: Adjacency matrix is [observer_size x observer_size], NOT including leader (vehicle 0)
        # Matrix indices 0..observer_size-1 correspond to follower vehicles 1..observer_size
        # Default: chain topology for platoon (each follower talks to neighbors)
        # Can be overridden by config['adjacency_matrix']
        if 'adjacency_matrix' in self.config:
            self.adjacency_matrix = np.array(self.config['adjacency_matrix'])
        else:
            # Default chain topology: follower i connected to follower i-1 and i+1
            self.adjacency_matrix = self._create_chain_topology()

        # 🔧 添加：打印邻接矩阵用于调试
        if self.logger:
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Adjacency matrix initialized "
                f"[{self.observer_size}x{self.observer_size}]:\n{self.adjacency_matrix}"
            )

        # Validate adjacency matrix dimensions (should be observer_size, not fleet_size)
        if self.adjacency_matrix.shape != (self.observer_size, self.observer_size):
            if self.logger:
                self.logger.logger.warning(
                    f"Adjacency matrix shape {self.adjacency_matrix.shape} doesn't match observer_size {self.observer_size}. "
                    f"Using default chain topology."
                )
            self.adjacency_matrix = self._create_chain_topology()
        
        # 🔧 在初始化阶段计算并缓存邻居列表
        self.my_neighbors = self.get_neighbors(self.vehicle_id)
        
        # 🔧 在初始化阶段计算并缓存输出矩阵Ci
        self.Ci = self.compute_output_matrix_Ci(self.vehicle_id)
        
        # 🔧 在日志中打印邻居列表和矩阵信息
        if self.logger:
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Neighbors list initialized: {self.my_neighbors}"
            )
            self.logger.logger.info(
                f"Vehicle {self.vehicle_id}: Output matrix Ci computed with shape: {self.Ci.shape}"
            )
    
    def _create_chain_topology(self) -> np.ndarray:
        """
        创建链式拓扑的邻接矩阵（只包含跟随者，不包含领导者）
        
        矩阵维度: [observer_size x observer_size]
        矩阵索引: 0..observer_size-1 对应车辆ID 1..observer_size
        
        连接规则（以车辆ID表示）：
        - 车辆1: 与车辆2相连（如果存在）
        - 车辆i (2 <= i <= observer_size-1): 与车辆i-1和i+1相连
        - 车辆observer_size: 与车辆observer_size-1相连
        
        Returns:
            adjacency_matrix: [observer_size x observer_size] 邻接矩阵
        """
        adj = np.zeros((self.observer_size, self.observer_size))
        
        # 遍历跟随者（矩阵索引0对应车辆1，索引1对应车辆2，...）
        for i in range(self.observer_size):
            # Connect to previous follower (if not the first follower)
            if i > 0:
                adj[i, i-1] = 1
            # Connect to next follower (if not the last follower)
            if i < self.observer_size - 1:
                adj[i, i+1] = 1
        
        return adj
    
    def _create_fully_connected_topology(self) -> np.ndarray:
        """
        创建全连接拓扑的邻接矩阵（所有跟随者互相通信，不包含领导者）
        
        矩阵维度: [observer_size x observer_size]
        所有跟随者之间全连接
        
        Returns:
            adjacency_matrix: [observer_size x observer_size] 邻接矩阵
        """
        adj = np.ones((self.observer_size, self.observer_size))
        # No self-loops
        np.fill_diagonal(adj, 0)
        return adj
    
    def get_neighbors(self, vehicle_id: int) -> List[int]:
        """
        获取指定跟随者车辆的所有邻居ID列表
        
        注意：邻接矩阵只包含跟随者（车辆1到observer_size），不包含领导者（车辆0）
        矩阵索引映射：矩阵索引 i = vehicle_id - 1
        
        Args:
            vehicle_id: 车辆ID (1 到 observer_size，即跟随者)
        
        Returns:
            neighbors: 邻居车辆ID列表（也是跟随者的ID）
        """
        # 领导者（车辆0）没有邻居，或者ID超出范围
        if vehicle_id < 1 or vehicle_id > self.observer_size:
            return []
        
        # 将车辆ID转换为矩阵索引（车辆1对应索引0，车辆2对应索引1，...）
        matrix_idx = vehicle_id - 1
        
        # Find all neighbors where adjacency_matrix[matrix_idx, neighbor_matrix_idx] > 0
        neighbors = []
        for j in range(self.observer_size):
            if self.adjacency_matrix[matrix_idx, j] > 0:
                # 将矩阵索引转换回车辆ID
                neighbor_vehicle_id = j + 1
                neighbors.append(neighbor_vehicle_id)
        
        return neighbors
    
    def compute_output_matrix_Ci(self, i: int) -> np.ndarray:
        """
        计算输出矩阵Ci，其中i从1开始
        
        对于车队中的第i辆车，其输出矩阵的结构为：
        - C1 = [Cf, 0, 0]     
        - C2 = [Cp, Cf, 0]     
        - C3 = [0, Cp, Cf]  
        
        规律：Ci在第i个位置放Cf，在第i-1个位置放Cp（如果i>1），其余位置补0
        
        Args:
            i: 车辆索引，从1开始 (1 <= i <= fleet_size)
        
        Returns:
            Ci: 输出矩阵，形状为 [2, 3*observer_size]
        """
        if i < 1 or i > self.observer_size:
            raise ValueError(f"车辆索引 i={i} 超出范围 [1, {self.observer_size}]")
        
        # 获取Cf和Cp的维度
        # Cf: [2, 3] - 自车观测矩阵
        # Cp: [2, 3] - 前车关系观测矩阵
        measurement_dim = self.Cf.shape[0]  # 2
        state_block_dim = self.Cf.shape[1]  # 3
        
        # 创建零矩阵块
        zero_block = np.zeros((measurement_dim, state_block_dim))
        
        # 构建Ci矩阵：在第i-1位置放Cp（如果存在），第i位置放Cf，其余位置补0
        blocks = []
        for j in range(1, self.observer_size + 1):
            if j == i - 1 and i > 1:
                # 第i-1个位置：观测与前车的关系
                blocks.append(self.Cp)
            elif j == i:
                # 第i个位置：观测自己
                blocks.append(self.Cf)
            else:
                # 其余位置：补零
                blocks.append(zero_block)
        
        # 水平拼接所有块
        Ci = np.hstack(blocks)
        
        return Ci
    
    def update(self, local_state: np.ndarray, dt: float, 
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        """Update using distributed observer with dynamics
        
        Args:
            local_state: Own vehicle state [x, y, theta, v, a] - used ONLY for local measurement calculation
            dt: Time step in seconds
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
        
        Note:
            local_state is NOT stored in fleet_states. It is used ONLY for measurement calculation.
            fleet_states contains ONLY distributed observer estimates for all vehicles.
        """
        try:
            # Ensure we have capacity
            self._ensure_fleet_capacity(self.vehicle_id)
            
            # Note: local_state is used ONLY for measurement calculation in the observer update
            # It is NOT directly assigned to fleet_states

            # Distributed observer for this vehicle
            self.estimated_state = self._distributed_luenberger_observer_update(
                local_state, current_time_ns, control, dt
            )
            # Transfer the estimated states back to fleet_states. 
            self.fleet_states = self._transfer_estimated_states_to_fleet_states(self.estimated_state, local_state, current_time_ns)
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Distributed Luenberger update error", e)
            return self.fleet_states
    
    def _transfer_fleet_states_to_estimated_states(self, fleet_states: np.ndarray, current_time_ns: int) -> np.ndarray:
        """
        将完整的车队状态矩阵转换为分布式观测器的估计状态格式
        
        将绝对状态转换为相对状态：
        - 位置估计: pi - p0 + di0
        - 速度估计: vi - v0
        - 加速度估计: ai - a0
        
        其中 di0 = vehicle_id * d + h * sum(v_k for k=1 to vehicle_id)
        即：
        - vehicle_id=1: di0 = 1*d + h*v1
        - vehicle_id=2: di0 = 2*d + h*(v1+v2)
        - vehicle_id=3: di0 = 3*d + h*(v1+v2+v3)
        
        Args:
            fleet_states: 完整的车队状态矩阵 [state_dim x fleet_size]
                         每列代表一辆车的状态 [x, y, theta, velocity, acceleration]

        Returns:
            estimated_state: 分布式观测器估计的状态向量 [3*observer_size]
                           格式: [p1-p0+d10, v1-v0, a1-a0, p2-p0+d20, v2-v0, a2-a0, ...]
        """
        # 获取领导者（车辆0）的绝对状态
        state_leader = self._get_latest_received_state(0,current_time_ns)
        if state_leader is not None:
            p0 = state_leader[0]  # 领导者的位置
            v0 = state_leader[3]  # 领导者的速度
            a0 = state_leader[4]  # 领导者的加速度
        else:
            p0 = fleet_states[0, 0]  # 领导者的位置
            v0 = fleet_states[3, 0]  # 领导者的速度
            a0 = fleet_states[4, 0]  # 领导者的加速度
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No recent leader state, using current estimate"
                )
        
        # 初始化估计状态矩阵 [3 x observer_size]
        estimated_state_mat = np.zeros((3, self.observer_size))
        
        # 对每辆跟随车辆（vehicle_id >= 1）计算相对状态
        for vehicle_id in range(1, min(self.fleet_size, self.observer_size + 1)):
            col_idx = vehicle_id - 1
            
            # 从车队状态中获取绝对状态
            pi = fleet_states[0, vehicle_id]  # x 位置
            vi = fleet_states[3, vehicle_id]  # 速度
            ai = fleet_states[4, vehicle_id]  # 加速度
            
            # 🔧 计算 di0 = vehicle_id * d + h * sum(v_k for k=1 to vehicle_id)
            # 关键：使用统一的速度来源，确保与逆转换一致
            di0 = vehicle_id * self.d
            
            # 累加从车辆1到vehicle_id的所有绝对速度
            velocity_sum = 0.0
            for k in range(1, vehicle_id + 1):
                # 优先使用 V2V 接收的最新速度（更准确），否则使用 fleet_states
                state_k = self._get_latest_received_state(k, current_time_ns)
                if state_k is not None:
                    vk = state_k[3]  # V2V 接收的绝对速度
                else:
                    vk = fleet_states[3, k]  # Fallback: fleet_states 中的绝对速度
                velocity_sum += vk
            
            di0 += self.h * velocity_sum
            
            # 计算相对状态
            relative_position = pi - p0 + di0      # pi - p0 + di0
            relative_velocity = vi - v0             # vi - v0
            relative_accel = ai - a0                # ai - a0
            
            # 存储到估计矩阵
            estimated_state_mat[0, col_idx] = relative_position
            estimated_state_mat[1, col_idx] = relative_velocity
            estimated_state_mat[2, col_idx] = relative_accel
        
        # 将矩阵展平为向量（列优先）
        estimated_state = estimated_state_mat.flatten(order="F")
        
        return estimated_state

    def _transfer_estimated_states_to_fleet_states(self, estimated_state: np.ndarray, local_state: np.ndarray, current_time_ns: int) -> np.ndarray:
        """
        将分布式观测器的估计状态转换为完整的车队状态矩阵
        
        分布式观测器估计的是相对状态：
        - 位置估计: pi - p0 + di0
        - 速度估计: vi - v0
        - 加速度估计: ai - a0
        
        其中 di0 = vehicle_id * d + h * sum(v_k for k=1 to vehicle_id)
        
        需要计算出绝对状态:
        - pi = 估计值 + p0 - di0
        - vi = 估计值 + v0
        - ai = 估计值 + a0
        
        Args:
            estimated_state: 分布式观测器估计的状态向量 [3*observer_size]
                           格式: [p1-p0+d10, v1-v0, a1-a0, p2-p0+d20, v2-v0, a2-a0, ...]
            local_state: 当前车辆的本地状态 [x, y, theta, v, a]
            current_time_ns: 当前时间戳（纳秒）

        Returns:
            fleet_states: 完整的车队状态矩阵 [state_dim x fleet_size]
        """
        # 将估计状态向量重塑为 [3 x observer_size] 矩阵（列优先），
        # 列 0 对应车辆 1，列 1 对应车辆 2，...，列 observer_size-1 对应车辆 observer_size
        estimated_state_mat = estimated_state.reshape((3, self.observer_size), order="F")
        
        # 初始化输出矩阵，保留原有的 y 和 theta 信息
        fleet_states_new = self.fleet_states.copy()
        
        # 获取领导者（车辆0）的绝对状态
        state_leader = self._get_latest_received_state(0,current_time_ns)
        if state_leader is not None:
            p0 = state_leader[0]  # 领导者的位置
            v0 = state_leader[3]  # 领导者的速度
            a0 = state_leader[4]  # 领导者的加速度
        else:
            p0 = self.fleet_states[0, 0]  # 领导者的位置
            v0 = self.fleet_states[3, 0]  # 领导者的速度
            a0 = self.fleet_states[4, 0]  # 领导者的加速度

        
        # 领导者的状态保持不变（直接从 fleet_states 读取）
        fleet_states_new[:, 0] = self.fleet_states[:, 0]
        
        # 对每辆跟随车辆（vehicle_id >= 1）计算绝对状态
        # estimated_state_mat 的列索引 j = vehicle_id - 1
        for vehicle_id in range(1, self.fleet_size):
            col_idx = vehicle_id - 1
            if col_idx >= self.observer_size:
                break  # 防止越界

            # 从估计矩阵中提取相对状态
            relative_position = estimated_state_mat[0, col_idx]  # pi - p0 + di0
            relative_velocity = estimated_state_mat[1, col_idx]  # vi - v0
            relative_accel = estimated_state_mat[2, col_idx]     # ai - a0
            
            # 🔧 计算 di0 = vehicle_id * d + h * sum(v_k for k=1 to vehicle_id)
            # 关键：必须与正向转换使用完全相同的速度来源，保证可逆性
            di0 = vehicle_id * self.d
            
            # 累加从车辆1到vehicle_id的所有绝对速度
            velocity_sum = 0.0
            for k in range(1, vehicle_id + 1):
                # 优先使用 V2V 接收的最新速度（更准确），否则使用 fleet_states
                state_k = self._get_latest_received_state(k, current_time_ns)
                if state_k is not None:
                    vk = state_k[3]  # V2V 接收的绝对速度
                else:
                    vk = self.fleet_states[3, k]  # Fallback: fleet_states 中的绝对速度
                velocity_sum += vk
            
            di0 += self.h * velocity_sum
            
            # 计算绝对状态
            pi = relative_position + p0 - di0
            vi = relative_velocity + v0
            ai = relative_accel + a0
            
            # 更新车队状态矩阵
            fleet_states_new[0, vehicle_id] = pi  # x 位置
            fleet_states_new[3, vehicle_id] = vi  # 速度
            fleet_states_new[4, vehicle_id] = ai  # 加速度
            
            # y 和 theta 保持原值（或者可以根据需要更新）
            fleet_states_new[1, vehicle_id] = self.fleet_states[1, vehicle_id]  # y (已经在 copy 中保留)
            fleet_states_new[2, vehicle_id] = self.fleet_states[2, vehicle_id]  # theta (已经在 copy 中保留)
        
        return fleet_states_new

    def _distributed_luenberger_observer_update(self, local_state: np.ndarray, current_time_ns: int,
                                     collective_control: np.ndarray, dt: float) -> np.ndarray:
        """
        Distributed observer update for one target vehicle
        Combines dynamics prediction, measurement correction, and consensus
        """
        # Distributed observer dimension (3: pi-p0+di0, vi-v0, ai-a0)
        longitudinal_state_dim = 3
        dim_distributed_observer = longitudinal_state_dim * self.observer_size

        # Distributed observer state (x_vec: pi-p0+di0, vi-v0, ai-a0)
        x_vec = self._transfer_fleet_states_to_estimated_states(self.fleet_states, current_time_ns)

        # Get leader state with fallback to current estimate
        state_leader = self._get_latest_received_state(0, current_time_ns)
        if state_leader is not None:
            v0 = state_leader[3]  # 领导者的速度
            p0 = state_leader[0]  # 领导者的位置
        else:
            v0 = self.fleet_states[3, 0]  # 使用当前估计的速度
            p0 = self.fleet_states[0, 0]  # 使用当前估计的位置
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No recent leader state of the leader 0, using current estimate"
                )

        # Get the control input of all follower vehicles
        # Todo: get the latest control input from V2V messages
        # For now, we use a default throttle value for testing
        test_throttle_value = 0.075
        collective_control = np.zeros(self.observer_size)
        for i in range(self.observer_size):
            collective_control[i] = test_throttle_value
           
        # 1. Dynamics prediction (collective longitudinal model)
        
        # collective longitudinal model: dx = Ax + Bu
        dynamics_term =  self.A_delta @ x_vec + self.B_delta @ collective_control
        
        # 2. Measurement correction (if we have data from target)
        measurement_term = np.zeros(dim_distributed_observer)  
        # Use latest self measurement (assumed available) to correct position
        local_measurement = np.zeros(self.local_measurement_dim)
        """
        Todo: get the latest relative position from sensors (Lidar/Camera)
        For now, we use the V2V local state messages to calculate the relative position measurement
        """
        # 直接从 local_state 读取自己的状态，避免依赖 fleet_states 的准确性
        p_i = local_state[0]  # Self position from local_state input
        v_i = local_state[3]  # Self velocity from local_state input
        
        # 仅从通信获取前车状态 
        state_prev = self._get_latest_received_state(self.vehicle_id - 1, current_time_ns)
        if state_prev is not None:
            p_prev = state_prev[0]  # Previous vehicle position from received state
        else:
            p_prev = self.fleet_states[0, self.vehicle_id - 1]
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No recent local state of previous vehicle {self.vehicle_id - 1}, using current estimate"
                )
        
        local_measurement[0] = p_i - p_prev  # relative position
        local_measurement[1] = v_i  # velocity 
        
        estimated_measurement = np.zeros(self.local_measurement_dim)
        # 🔧 使用初始化时缓存的Ci矩阵
        estimated_measurement = self.Ci @ x_vec + self.Cv * v0 + self.Cd * self.d  # estimated relative position and velocity

        measurement_error = local_measurement - estimated_measurement
        # yi -hat_yi
        measurement_term = self.observer_gain @ measurement_error
        
        # 3. Consensus term - 基于邻接矩阵的共识
        consensus_term = np.zeros(dim_distributed_observer)
        neighbor_count = 0
        consensus_accum = np.zeros(dim_distributed_observer)
        
        # 🔧 使用初始化阶段缓存的邻居列表
        if self.logger:
            self.logger.logger.debug(
                f"Vehicle {self.vehicle_id}: Processing consensus with neighbors: {self.my_neighbors}"
            )
        
        # Loop through each neighbor defined by adjacency matrix
        for neighbor_id in self.my_neighbors:
            # --- Step 1: Try to get FLEET state (Primary Source) ---
            neighbor_fleet_dict = self._get_latest_fleet_data(neighbor_id, current_time_ns)
            
            # Validate fleet data completeness
            is_complete_fleet = False
            missing_vehicles = []
            
            if neighbor_fleet_dict is not None:
                # Check if fleet data contains all necessary vehicles
                expected_vehicles = set(range(self.fleet_size))
                received_vehicles = set(int(vid) for vid in neighbor_fleet_dict.keys())
                missing_vehicles = list(expected_vehicles - received_vehicles)
                
                if not missing_vehicles:
                    is_complete_fleet = True
                    if self.logger:
                        self.logger.logger.info(
                            f"Vehicle {self.vehicle_id}: Complete fleet_state from neighbor {neighbor_id}"
                        )
                else:
                    if self.logger:
                        self.logger.logger.warning(
                            f"Vehicle {self.vehicle_id}: Incomplete fleet_state from neighbor {neighbor_id}, "
                            f"missing vehicles: {missing_vehicles}"
                        )
            
            # --- Step 2: Fallback to LOCAL states if needed ---
            if not is_complete_fleet:
                if self.logger:
                    self.logger.logger.warning(
                        f"Vehicle {self.vehicle_id}: Using fallback strategy for neighbor {neighbor_id}"
                    )
                
                # Initialize with empty fleet dict if None
                if neighbor_fleet_dict is None:
                    neighbor_fleet_dict = {}
                    if self.logger:
                        self.logger.logger.warning(
                            f"Vehicle {self.vehicle_id}: No fleet_state from neighbor {neighbor_id}, "
                            f"building from scratch"
                        )
                
                # Fill missing vehicles with local state broadcasts or current estimates
                for vid in missing_vehicles:
                    # Try to get vehicle's self-report (LOCAL state)
                    vehicle_local_state = self._get_latest_received_state(vid, current_time_ns)
                    
                    
                    
                    if vehicle_local_state is not None:
                        # Use received local state
                        neighbor_fleet_dict[vid] = {
                            'x': float(vehicle_local_state[0]),
                            'y': float(vehicle_local_state[1]),
                            'theta': float(vehicle_local_state[2]),
                            'velocity': float(vehicle_local_state[3]),
                            'acceleration': float(vehicle_local_state[4] if len(vehicle_local_state) > 4 else 0.0)
                        }
                        if self.logger:
                            self.logger.logger.info(
                                f"Vehicle {self.vehicle_id}: Filled vehicle_{vid} using its local_state"
                            )
                    else:
                        # Use current estimate as last resort
                        neighbor_fleet_dict[vid] = {
                            'x': float(self.fleet_states[0, vid]),
                            'y': float(self.fleet_states[1, vid]),
                            'theta': float(self.fleet_states[2, vid]),
                            'velocity': float(self.fleet_states[3, vid]),
                            'acceleration': float(self.fleet_states[4, vid])
                        }
                        if self.logger:
                            self.logger.logger.info(
                                f"Vehicle {self.vehicle_id}: Filled vehicle_{vid} using current estimate "
                                f"(no local_state available)"
                            )
            
            # --- Step 3: Build neighbor's complete fleet_states matrix ---
            neighbor_fleet_states = np.zeros((self.state_dim, self.fleet_size))
            
            for vid, vehicle_state in neighbor_fleet_dict.items():
                try:
                    vid_int = int(vid)
                    if 0 <= vid_int < self.fleet_size:
                        neighbor_fleet_states[0, vid_int] = vehicle_state.get('x', 0.0)
                        neighbor_fleet_states[1, vid_int] = vehicle_state.get('y', 0.0)
                        neighbor_fleet_states[2, vid_int] = vehicle_state.get('theta', 0.0)
                        neighbor_fleet_states[3, vid_int] = vehicle_state.get('velocity', vehicle_state.get('v', 0.0))
                        neighbor_fleet_states[4, vid_int] = vehicle_state.get('acceleration', 0.0)
                    if self.logger:
                        self.logger.logger.info(
                            f"Vehicle {self.vehicle_id}: Neighbor {neighbor_id} vehicle_{vid_int} state: "
                            f"x={neighbor_fleet_states[0, vid_int]:.3f}, v={neighbor_fleet_states[3, vid_int]:.3f}"
                        )
                except (ValueError, TypeError) as e:
                    if self.logger:
                        self.logger.logger.error(
                            f"Vehicle {self.vehicle_id}: Invalid vehicle_id {vid} from neighbor {neighbor_id}: {e}"
                        )
                    continue
            
            # --- Step 4: Convert to distributed observer state format ---
            neighbor_x_vec = self._transfer_fleet_states_to_estimated_states(
                neighbor_fleet_states, current_time_ns
            )
            
            # --- Step 5: Calculate consensus difference with adjacency weight ---
            my_matrix_idx = self.vehicle_id - 1
            neighbor_matrix_idx = neighbor_id - 1
            weight = self.adjacency_matrix[my_matrix_idx, neighbor_matrix_idx]
            
            # Accumulate weighted difference: weight * (neighbor_estimate - own_estimate)
            consensus_diff = x_vec - neighbor_x_vec
            consensus_accum += weight * consensus_diff
            neighbor_count += 1
            
            if self.logger:
                self.logger.logger.info(
                    f"Vehicle {self.vehicle_id}: Consensus with neighbor {neighbor_id}, "
                    f"weight={weight:.3f}, neighbor_x_vec={np.array2string(neighbor_x_vec, precision=3)}, "
                    f"data_source={'fleet_state' if is_complete_fleet else 'fallback'}"
                )
        
        # --- Step 6: Apply consensus gain (使用矩阵增益并归一化) ---
        if neighbor_count > 0:
            # 使用矩阵增益：[9×9] @ [9×1] = [9×1]
            # 归一化防止邻居数量导致的增益爆炸
            consensus_term = (self.consensus_gain/ neighbor_count) @ consensus_accum
            
            # 🔧 添加数值保护：防止共识项过大导致发散
            consensus_norm = np.linalg.norm(consensus_term)
            max_consensus_threshold = 50.0  # 根据物理约束设置
            if consensus_norm > max_consensus_threshold:
                if self.logger:
                    self.logger.logger.warning(
                        f"Vehicle {self.vehicle_id}: Consensus term too large ({consensus_norm:.2f}), "
                        f"clamping to {max_consensus_threshold}"
                    )
                consensus_term = consensus_term / consensus_norm * max_consensus_threshold
        
            if self.logger:
                self.logger.logger.info(
                    f"Vehicle {self.vehicle_id}: Final consensus term applied, "
                    f"neighbor_count={neighbor_count}, consensus_norm={np.linalg.norm(consensus_term):.4f}"
                )
        else:
            consensus_term = np.zeros(dim_distributed_observer)
            if self.logger:
                self.logger.logger.warning(
                    f"Vehicle {self.vehicle_id}: No valid neighbors for consensus"
                )
        
        # Combine all terms (注意共识项是减号，符合理论公式)
        x_i_new = x_vec + dt * (dynamics_term + measurement_term - consensus_term)
        
        # 🔧 添加状态约束：防止数值溢出导致发散
        x_i_new = np.clip(x_i_new, -1e4, 1e4)
        
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
    
    # Base estimator types (always available)
    ESTIMATOR_TYPES = {
        'consensus': ConsensusFleetEstimator,
        'distributed_luenberger': DistributedLuenbergerEstimator,
    }
    
    # Trust-based estimators are loaded lazily to avoid circular imports
    _trust_estimators_loaded = False
    
    @classmethod
    def _load_trust_estimators(cls):
        """Lazily load trust-based estimators to avoid circular imports"""
        if cls._trust_estimators_loaded:
            return
        
        try:
            from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (
                TrustBasedFleetEstimator,
                TrustBasedKalmanEstimator
            )
            cls.ESTIMATOR_TYPES['trust_consensus'] = TrustBasedFleetEstimator
            cls.ESTIMATOR_TYPES['trust_kalman'] = TrustBasedKalmanEstimator
            cls._trust_estimators_loaded = True
        except ImportError as e:
            # Trust-based estimators not available
            pass
    
    @staticmethod
    def create(estimator_type: str, vehicle_id: int, fleet_size: int,
               state_dim: int = 5, config: Dict = None, logger=None):
        """
        Create a fleet state estimator
        
        Args:
            estimator_type: One of 'consensus', 'distributed_kalman', 
                           'distributed_luenberger', 'trust_consensus', 'trust_kalman'
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 3) based on vehicle model
            config: Configuration dict
            logger: Logger instance
            
        Returns:
            Fleet state estimator instance
        """
        # Try to load trust-based estimators if requesting one
        if estimator_type.startswith('trust_'):
            FleetEstimatorFactory._load_trust_estimators()
        
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
    
    @classmethod
    def get_available_types(cls) -> List[str]:
        """Get list of available estimator types"""
        cls._load_trust_estimators()
        return list(cls.ESTIMATOR_TYPES.keys())
