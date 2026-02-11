"""
Longitudinal Controllers for Vehicle Following

Based on the distirbuted luenberger observer and CACC control law.

control commands:
u_{i} = sum (K_{ij} (Fi \hat{x}_{i} - Fj \hat{x}_{i})) from j =0 to i-1
parameters:
    - u_{i} is the control throttle of vehicle i
    - K_{ij} is the control gain, configured by the extral congig file.
    - Fi is the index matrix. 
    - \hat{x}_{i} is the estimated state from the distributed luenberger observer i.
    - i is the index of the vehicle in the platoon, starting from 1. 0 is the leader vehicle.

"""
import numpy as np
import time
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, TYPE_CHECKING
from ..longitudinal_controllers import LongitudinalControllerBase

if TYPE_CHECKING:
    from Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator


class StateFeedbackControllerNoObserver(LongitudinalControllerBase):
    """
    State Feedback Controller without Observer - uses true V2V communication states
    
    This controller bypasses the distributed Luenberger observer and directly uses
    true vehicle states from V2V communication to compute control commands.
    
    Useful for:
    - Performance benchmarking (comparing observer-based vs ground truth control)
    - Debugging and validation
    - Scenarios where direct state measurement is available and reliable
    
    Note: Still requires an observer instance to access V2V communication infrastructure
    (received_local_states), but uses true states instead of observer estimates.
    """
    
    def __init__(self, 
                 max_throttle=0.3,
                 throttle_smoothing=0.7,
                 observer=None,
                 config=None,
                 logger=None):
        """
        Initialize State Feedback controller that uses true V2V states (no observer estimation).
        
        Args:
            max_throttle: Maximum throttle output
            throttle_smoothing: Exponential smoothing factor for throttle (0-1, higher = smoother)
            observer: Observer instance (needed for V2V communication access via received_local_states)
            config: Optional config object (takes precedence)
            logger: Logger instance
        """
        self.logger = logger
        self.observer = observer
        
        # Use config if provided
        if config and hasattr(config, 'get_longitudinal_params'):
            params = config.get_longitudinal_params('state_feedback')
            self.max_throttle = params.get('max_throttle', max_throttle)
            self.throttle_smoothing = params.get('throttle_smoothing', throttle_smoothing)
        else:
            self.max_throttle = max_throttle
            self.throttle_smoothing = throttle_smoothing
        
        # Controller state
        self.prev_throttle = 0.0

        # Store K matrices directly for each vehicle
        # K_matrices[vehicle_id][j] = K_{vehicle_id,j}
        self.K_all_vehicles = {
            1: {
                0: np.array([[-0.1729,-0.4856,-0.0746]])
            },
            2: {
                0: np.array([[-0.1766,-0.4659,-0.0822]]),
                1: np.array([[-0.0028,-0.0056,0.0038]])
            },
            3: {
                0: np.array([[-0.1895,-0.5314,-0.0981]]),
                1: np.array([[-0.0026,-0.0053,0.0039]]),
                2: np.array([[-0.0010,-0.0010,0.0049]])
            }
        }

        
        # Extract K matrices for current vehicle
        self.K_matrices = []
        if self.observer is not None:
            vehicle_id = self.observer.vehicle_id
            if vehicle_id in self.K_all_vehicles:
                # Get K_i0, K_i1, ..., K_i(i-1) for vehicle i
                for j in range(vehicle_id):
                    if j in self.K_all_vehicles[vehicle_id]:
                        self.K_matrices.append(self.K_all_vehicles[vehicle_id][j])
                    else:
                        self.K_matrices.append(None)
                        if self.logger:
                            self.logger.warning(f"K{vehicle_id}{j} not found, using None")
                
                if self.logger:
                    self.logger.info(f"Vehicle {vehicle_id}: Loaded {len(self.K_matrices)} K matrices")
            else:
                if self.logger:
                    self.logger.warning(f"No K matrices defined for vehicle {vehicle_id}")
        else:
            if self.logger:
                self.logger.warning("No observer instance, K matrices not initialized")
        
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute throttle using state feedback controller based on true V2V communication states.
        
        This method bypasses observer estimates and directly uses true vehicle states from
        V2V communication for control computation.
        
        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity' (following base controller interface)
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (not used - gets data from V2V)
            dt: Time step (seconds)
            
        Returns:
            throttle: Computed throttle command (0 to max_throttle)
        """
        # Check if observer is available
        if self.observer is None:
            if self.logger:
                self.logger.warning("StateFeedbackControllerNoObserver: No observer instance provided, returning zero throttle")
            return 0.0
        
        # Convert follower_state dict to numpy array for get_true_estimated_states
        # follower_state: {x, y, theta, velocity, ...}
        local_state = None
        if follower_state:
            try:
                local_state = np.array([
                    follower_state.get('x', 0.0),
                    follower_state.get('y', 0.0),
                    follower_state.get('theta', 0.0),
                    follower_state.get('velocity', 0.0),
                    follower_state.get('acceleration', 0.0)
                ])
            except Exception as e:
                if self.logger:
                    self.logger.warning(f"Failed to convert follower_state to array: {e}")
        
        # Get true vehicle states from V2V communication (bypassing observer estimates)
        # The observer instance is used only to access V2V communication data
        estimated_states = self.get_true_estimated_states(local_state=local_state)
        
        # Check if we have valid estimated states
        if estimated_states is None or len(estimated_states) == 0:
            if self.logger:
                self.logger.warning("StateFeedbackControllerNoObserver: No estimated states available from V2V communication")
            return 0.0
        
        # Get vehicle information
        vehicle_id = self.observer.vehicle_id
        num_vehicles = self.observer.observer_size  # Number of follower vehicles (not including leader)
        
        # Calculate index matrix Fi for this vehicle
        Fi = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=vehicle_id)
    
        # State feedback control law: u_i = sum_{j=0}^{i-1} K_{ij} * (F_i - F_j) * estimated_states
        # For j=0 (leader), F_0 is zero matrix, so K_{i0} * F_i * estimated_states
        throttle_raw = 0.15  # Base throttle to maintain speed, will be adjusted by feedback terms
        
        # First term: K_{i0} * F_i * estimated_states (j=0)
        # This represents control based on this vehicle's relative state to leader
        if len(self.K_matrices) > 0 and self.K_matrices[0] is not None:
            K_i0 = self.K_matrices[0]
            control_input = K_i0 @ (Fi @ estimated_states)
            throttle_raw += control_input[0]
        
        # Sum over preceding vehicles j=1 to i-1
        # This represents control based on relative states between this vehicle and preceding vehicles
        for j in range(1, vehicle_id):
            if j < len(self.K_matrices) and self.K_matrices[j] is not None:
                K_ij = self.K_matrices[j]
                Fj = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=j)
                control_input = K_ij @ ((Fi - Fj) @ estimated_states)
                throttle_raw += control_input[0]

        # Special handling for braking (negative throttle)
        if throttle_raw < 0:
            # More aggressive smoothing for braking to prevent jerky stops
            smoothing_factor = 0.85
            throttle_raw = (smoothing_factor * self.prev_throttle + 
                          (1 - smoothing_factor) * throttle_raw)
            throttle_raw = max(throttle_raw, 0.0)  # No negative throttle output
        
        # Apply exponential smoothing to final throttle command
        throttle = (self.throttle_smoothing * self.prev_throttle + 
                   (1 - self.throttle_smoothing) * throttle_raw)
        
        # Ensure throttle is non-negative
        throttle = min(max(throttle, 0.0), self.max_throttle)
        
        # Store for next iteration
        self.prev_throttle = throttle
        
        return throttle
    
    def reset(self):
        """Reset controller state"""
        self.prev_throttle = 0.0

    def get_true_estimated_states(self, local_state: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Get true estimated states from V2V communication by reading actual vehicle states.
        
        Computes relative states with desired spacing:
        - estimated_state[0] = p1 - p0 + d10
        - estimated_state[1] = v1 - v0
        - estimated_state[2] = a1 - a0
        - estimated_state[3] = p2 - p0 + d20
        - estimated_state[4] = v2 - v0
        - estimated_state[5] = a2 - a0
        - ... and so on for all follower vehicles
        
        Where d_i0 = vehicle_id * d + h * sum(v_k for k=1 to vehicle_id)
        
        Args:
            local_state: Local vehicle state [x, y, theta, v, a] (used for self when vehicle unavailable from V2V)
        
        Returns:
            estimated_state: Array of shape [3 * observer_size] with relative states
                            Returns None if observer or V2V communication data is not available
        """
        # Check if observer is available with required communication data
        if self.observer is None:
            if self.logger:
                self.logger.warning("get_true_estimated_states: No observer instance, cannot get V2V data")
            return None
        
        # Check if observer has the necessary communication data structures
        if not hasattr(self.observer, 'received_local_states'):
            if self.logger:
                self.logger.warning("get_true_estimated_states: Observer doesn't have received_local_states")
            return None
        
        # Get observer parameters
        num_vehicles = self.observer.observer_size  # Number of follower vehicles (not including leader)
        vehicle_id = self.observer.vehicle_id
        
        # Get time for state lookup
        current_time_ns = int(time.time() * 1e9) if hasattr(time, 'time') else 0
        
        # Initialize state matrix [3 x observer_size]
        estimated_state_mat = np.zeros((3, num_vehicles))
        
        # Get leader (vehicle 0) true state from V2V communication
        state_leader = self._get_true_vehicle_state(0, current_time_ns)
        if state_leader is None:
            if self.logger:
                self.logger.warning("get_true_estimated_states: Cannot get leader state from V2V")
            return None
        
        p0 = state_leader[0]  # Leader position
        v0 = state_leader[3]  # Leader velocity
        a0 = state_leader[4]  # Leader acceleration
        
        # Get observer parameters for distance calculation
        d = self.observer.d if hasattr(self.observer, 'd') else 0.4
        h = self.observer.h if hasattr(self.observer, 'h') else 0.3
        
        # For each follower vehicle, compute relative state from true V2V data
        for follower_id in range(1, num_vehicles + 1):
            col_idx = follower_id - 1
            
            # Get true state of follower vehicle from V2V communication
            # Special case: use local_state for self
            if follower_id == vehicle_id and local_state is not None:
                state_i = local_state
            else:
                state_i = self._get_true_vehicle_state(follower_id, current_time_ns)
            
            if state_i is None:
                if self.logger:
                    self.logger.warning(f"get_true_estimated_states: Cannot get state for vehicle {follower_id}")
                return None
            
            pi = state_i[0]  # Position
            vi = state_i[3]  # Velocity
            ai = state_i[4]  # Acceleration
            
            # Calculate desired spacing d_i0 = follower_id * d + h * sum(v_k for k=1..follower_id)
            di0 = follower_id * d
            
            # Sum true velocities from vehicle 1 to current follower
            velocity_sum = 0.0
            for k in range(1, follower_id + 1):
                # Use local_state for self, V2V for others
                if k == vehicle_id and local_state is not None:
                    state_k = local_state
                else:
                    state_k = self._get_true_vehicle_state(k, current_time_ns)
                
                if state_k is not None:
                    velocity_sum += state_k[3]
                else:
                    if self.logger:
                        self.logger.warning(f"get_true_estimated_states: Cannot get velocity for vehicle {k}")
                    return None
            
            di0 += h * velocity_sum
            
            # Calculate relative states (ground truth)
            estimated_state_mat[0, col_idx] = pi - p0 + di0  # Relative position with spacing
            estimated_state_mat[1, col_idx] = vi - v0          # Relative velocity
            estimated_state_mat[2, col_idx] = ai - a0          # Relative acceleration
        
        # Flatten to vector format [3*observer_size]
        estimated_states = estimated_state_mat.flatten(order='F')
        
        return estimated_states
    
    def _get_true_vehicle_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """
        Get the most recent true vehicle state from V2V communication.
        
        Args:
            vehicle_id: ID of the vehicle (0 for leader, 1..N for followers)
            current_time_ns: Current timestamp in nanoseconds
        
        Returns:
            state: State vector [x, y, theta, v, a] or None if not available
        """
        if not hasattr(self.observer, '_get_latest_received_state'):
            return None
        
        # Use observer's method to get latest received state
        return self.observer._get_latest_received_state(vehicle_id, current_time_ns)

    def calculate_Fi(self, num_vehicles: int, vehicle_index: int) -> np.ndarray:
        """
        Calculate the index matrix Fi for vehicle i in a platoon of num_vehicles
        
        Args:
            num_vehicles: Total number of vehicles in the platoon, not including the leader
            vehicle_index: Index of the current vehicle (1-based)
        
        Returns:
            Fi: Index matrix as a numpy array of shape (n0, n) where n0=3, n=3*num_vehicles
            The matrix places a 3x3 identity matrix at the i-th block position, zeros elsewhere
        """
        n0 = 3
        n = n0 * num_vehicles
        Fi = np.zeros((n0, n))
        
        # Place n0 x n0 identity matrix at the i-th block position
        block_start = n0 * (vehicle_index - 1)
        Fi[:, block_start:block_start + n0] = np.eye(n0)

        
        return Fi


