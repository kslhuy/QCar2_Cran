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
        # print(f"Adding received local state from vehicle_id {sender_id}")
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
    
    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        """
        Initialize distributed Lunberger Observer
        
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
        
        # Communication weights (uniform for now)
        self.weights = np.ones(self.observer_size) / self.observer_size
    
    def compute_output_matrix_Ci(self, i: int) -> np.ndarray:
        """
        计算输出矩阵Ci，其中i从1开始
        
        对于车队中的第i辆车，其输出矩阵的结构为：
        - C1 = [Cf, 0, 0]      (第1辆车：只观测自己)
        - C2 = [Cp, Cf, 0]     (第2辆车：观测与前车关系 + 自己)
        - C3 = [0, Cp, Cf]     (第3辆车：观测与前车关系 + 自己)
        
        规律：Ci在第i个位置放Cf，在第i-1个位置放Cp（如果i>1），其余位置补0
        
        Args:
            i: 车辆索引，从1开始 (1 <= i <= fleet_size)
        
        Returns:
            Ci: 输出矩阵，形状为 [2, 3*fleet_size]
        """
        if i < 1 or i > self.fleet_size:
            raise ValueError(f"车辆索引 i={i} 超出范围 [1, {self.fleet_size}]")
        
        # 获取Cf和Cp的维度
        # Cf: [2, 3] - 自车观测矩阵
        # Cp: [2, 3] - 前车关系观测矩阵
        measurement_dim = self.Cf.shape[0]  # 2
        state_block_dim = self.Cf.shape[1]  # 3
        
        # 创建零矩阵块
        zero_block = np.zeros((measurement_dim, state_block_dim))
        
        # 构建Ci矩阵：在第i-1位置放Cp（如果存在），第i位置放Cf，其余位置补0
        blocks = []
        for j in range(1, self.fleet_size + 1):
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
            local_state: Own vehicle state [x, y, theta, v]
            dt: Time step in seconds
            current_time_ns: Current time in nanoseconds
            control: Control input [steering, throttle]
        """
        try:
            # # Ensure we have capacity and write our own latest local state into fleet_states
            # self._ensure_fleet_capacity(self.vehicle_id)
            # self.fleet_states[:, self.vehicle_id] = local_state.copy()

            # Distributed observer for this vehicle
            self.estimated_state = self._distributed_observer_update(
                current_time_ns, control, dt
            )
            # Transfer the estimated states back to fleet_states. 
            self.fleet_states = self._transfer_estimated_states_to_fleet_states(self.estimated_state)
            # Cleanup old data
            self._cleanup_old_data(current_time_ns)
            
            return self.fleet_states
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Distributed Luenberger update error", e)
            return self.fleet_states
    
    def _transfer_estimated_states_to_fleet_states(self, estimated_state: np.ndarray) -> np.ndarray:
        """
        将分布式观测器的估计状态转换为完整的车队状态矩阵
        
        分布式观测器估计的是相对状态：
        - 位置估计: pi - p0 + di0
        - 速度估计: vi - v0
        - 加速度估计: ai - a0
        
        其中 di0 = i*d + h*sum(vk) for k=1 to i
        
        需要计算出绝对状态:
        - pi = 估计值 + p0 - di0
        - vi = 估计值 + v0
        - ai = 估计值 + a0
        
        Args:
            estimated_state: 分布式观测器估计的状态向量 [3*fleet_size]
                           格式: [p1-p0+d10, v1-v0, a1-a0, p2-p0+d20, v2-v0, a2-a0, ...]
        
        Returns:
            fleet_states: 完整的车队状态矩阵 [state_dim x fleet_size]
        """
        # 将估计状态向量重塑为 [3 x fleet_size] 矩阵（列优先）
        estimated_state_mat = estimated_state.reshape((3, self.fleet_size), order='F')
        
        # 初始化输出矩阵，保留原有的 y 和 theta 信息
        fleet_states_new = self.fleet_states.copy()
        
        # 获取领导者（车辆0）的绝对状态
        p0 = self.fleet_states[0, 0]  # 领导者的位置
        v0 = self.fleet_states[3, 0]  # 领导者的速度
        a0 = self.fleet_states[4, 0]  # 领导者的加速度
        
        # 领导者的状态保持不变（直接从 fleet_states 读取）
        fleet_states_new[:, 0] = self.fleet_states[:, 0]
        
        # 对每辆跟随车辆（i >= 1）计算绝对状态
        for i in range(1, self.fleet_size):
            # 从估计矩阵中提取相对状态
            relative_position = estimated_state_mat[0, i]  # pi - p0 + di0
            relative_velocity = estimated_state_mat[1, i]  # vi - v0
            relative_accel = estimated_state_mat[2, i]     # ai - a0
            
            # 计算 di0 = i*d + h*sum(vk) for k=1 to i
            di0 = i * self.d
            if i > 1:
                # 累加前面车辆的速度
                velocity_sum = 0.0
                for k in range(1, i):
                    # 使用当前 fleet_states 中的速度
                    velocity_sum += self.fleet_states[3, k]
                
                # h 是时间间隔参数，从系统矩阵中获取
                di0 += self.h * velocity_sum
            
            # 计算绝对状态
            pi = relative_position + p0 - di0
            vi = relative_velocity + v0
            ai = relative_accel + a0
            
            # 更新车队状态矩阵
            fleet_states_new[0, i] = pi  # x 位置
            fleet_states_new[3, i] = vi  # 速度
            fleet_states_new[4, i] = ai  # 加速度
            
            # y 和 theta 保持原值（或者可以根据需要更新）
            fleet_states_new[1, i] = self.fleet_states[1, i]  # y (已经在 copy 中保留)
            fleet_states_new[2, i] = self.fleet_states[2, i]  # theta (已经在 copy 中保留)
        
        return fleet_states_new

    def _distributed_observer_update(self, current_time_ns: int,
                                     collective_control: np.ndarray, dt: float) -> np.ndarray:
        """
        Distributed observer update for one target vehicle
        Combines dynamics prediction, measurement correction, and consensus
        """
        # Distributed observer dimension (3: x, v, a)
        longitudinal_state_dim = 3
        dim_distributed_observer = longitudinal_state_dim * self.fleet_size
        # Current estimate
        x_i_mat5 = self.fleet_states.copy()
        x_i_mat3 = x_i_mat5[[0, 3, 4], :]  # 选择第0、3、4行，所有列 # extract the x,v,a states 
        # Get the current estimate (position, velocity, acceleration) for all vehicles:
        x_vec = x_i_mat3.flatten(order="F") # column-major flattening
        own_state = x_i_mat3[:, self.vehicle_id]

        # Get the control input of all vehicles
        # Todo: get the latest control input from V2V messages
        # For now, we use a default throttle value for testing
        test_throttle_value = 0.075
        collective_control = np.zeros(self.fleet_size)
        for i in range(self.fleet_size):
            collective_control[i] = test_throttle_value

        # Get the collective nonlinear term phi_i for all vehicles
        f = np.zeros(self.fleet_size)
        v_i = own_state[1]  # self velocity
        a_i = own_state[2]  # self acceleration
        f[self.vehicle_id] = self._get_nonlinear_term_phi_i(v_i, a_i)
        for i in range(self.fleet_size):
            if i == self.vehicle_id:
                continue
            state5 = self._get_latest_received_state(i, current_time_ns) # To check which kind of state we get.
            if state5 is None:
                # Use current estimate from fleet_states if no received state
                v_i = x_i_mat5[3, i]  # velocity from current estimate
                a_i = x_i_mat5[4, i]  # acceleration from current estimate
            else:
                v_i = state5[3]  # velocity from received state (state_dim=5) in the original system
                a_i = state5[4]  # acceleration from received state (state_dim=5) in the original system
           
        # 1. Dynamics prediction (collective longitudinal model)
        
        # collective longitudinal model
        # dx = Ax + Bu

        state_leader = self._get_latest_received_state(0, current_time_ns)
        v0 = state_leader[3] if state_leader is not None else 0.0 # velocity of the leader vehicle

        dynamics_term = x_vec + (self.A_delta @ x_vec + self.B_delta @ collective_control) * dt
        
        # 2. Measurement correction (if we have data from target)
        measurement_term = np.zeros(dim_distributed_observer)  # Use 3*fleet_size dimension
        # Use latest self measurement (assumed available) to correct position
        local_measurement = np.zeros(self.local_measurement_dim)
        """
        Todo: get the latest relative position from sensors (Lidar/Camera)
        For now, we use the V2V local state messages to calculate the relative position measurement
        """
        local_measurement[0] = x_i_mat3[0, self.vehicle_id] - x_i_mat3[0, self.vehicle_id - 1]  # relative position  
        local_measurement[1] = v_i # velocity 
        
        estimated_measurement = np.zeros(self.local_measurement_dim)
        Ci = self.compute_output_matrix_Ci(self.vehicle_id)  # Get Ci for this vehicle (1-based index)
        estimated_measurement = Ci @ x_vec + self.Cv * v0 + self.Cd * self.d  # estimated relative position and velocity

        measurement_error = local_measurement - estimated_measurement
        measurement_term = self.observer_gain * measurement_error
        
        # 3. Consensus term
        consensus_term = np.zeros(dim_distributed_observer)
        
        # Loop through all neighbors who have sent fleet state broadcasts
        for neighbor_id in self.received_fleet_states.keys():
            if neighbor_id == self.vehicle_id:
                continue  # Skip self
                
            # Get neighbor's latest fleet estimate
            neighbor_fleet_dict = self._get_latest_fleet_data(neighbor_id, current_time_ns)
            
            if neighbor_fleet_dict is None:
                continue  # No valid data from this neighbor
            
            # Extract neighbor's estimate for all vehicles (build 3xN matrix)
            neighbor_x_mat3 = np.zeros((3, self.fleet_size))
            
            for vid in range(self.fleet_size):
                if vid in neighbor_fleet_dict:
                    vehicle_state = neighbor_fleet_dict[vid]
                    # Extract x, v from fleet state (acceleration may not be in fleet broadcast)
                    neighbor_x_mat3[0, vid] = vehicle_state.get('x', 0.0)
                    neighbor_x_mat3[1, vid] = vehicle_state.get('velocity', vehicle_state.get('v', 0.0))
                    neighbor_x_mat3[2, vid] = vehicle_state.get('acceleration', 0.0)
            
            # Flatten neighbor's estimate (column-major)
            neighbor_x_vec = neighbor_x_mat3.flatten(order="F")
            
            # Accumulate consensus difference: (neighbor_estimate - own_estimate)
            consensus_term += (neighbor_x_vec - x_vec)
            neighbor_count += 1
        
        # Apply consensus gain (average over neighbors to prevent explosion)
        if neighbor_count > 0:
            consensus_term = self.consensus_gain @ consensus_term
        
        # Combine all terms
        x_i_new = dynamics_term + measurement_term + consensus_term
        
        
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
            state_dim: State dimension (default 3) based on vehicle model
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
