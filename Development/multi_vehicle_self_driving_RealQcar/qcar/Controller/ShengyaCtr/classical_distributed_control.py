"""
Longitudinal Controllers for Vehicle Following

Based on the distirbuted luenberger observer and CACC control law.

control commands:
u_1 = K_{10} F_1 estimated_state
u_2 = K_{20} F_2 estimated_state + K_{21} (F_2 - F_1) estimated_state + K32 (F_2 - F_3)  estimated_state
u_3 = K_{30} F_3 estimated_state + K_{32} (F_3 - F_2) estimated_state
parameters:
    - u_{i} is the control throttle of vehicle i
    - K_{ij} is the control gain, configured by the extral congig file.
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


class ClassicalDistributedController(LongitudinalControllerBase):
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
                 leader_fix_throttle=0.1,
                 K_all_vehicles=None,
                 observer=None,
                 config=None,
                 logger=None):
        """
        Adapted for config-based parameter and K matrix loading.
        """
        self.logger = logger
        self.observer = observer

        # Use config if provided
        if config and hasattr(config, 'get_longitudinal_params'):
            params = config.get_longitudinal_params('classical_distributed') if 'classical_distributed' in config.get_available_longitudinal_types() else config.get_longitudinal_params('state_feedback')
            self.max_throttle = params.get('max_throttle', max_throttle)
            self.throttle_smoothing = params.get('throttle_smoothing', throttle_smoothing)
            self.leader_fix_throttle = params.get('leader_fix_throttle', leader_fix_throttle)
            # Load K matrices from config if available
            self.K_all_vehicles = params.get('K_all_vehicles', K_all_vehicles)
        else:
            self.max_throttle = max_throttle
            self.throttle_smoothing = throttle_smoothing
            self.leader_fix_throttle = leader_fix_throttle
            self.K_all_vehicles = K_all_vehicles

        # Controller state
        self.prev_throttle = 0.0

        # Default hardcoded K matrices if not provided by config
        if self.K_all_vehicles is None:
            self.K_all_vehicles = {
                1: {
                    0: np.array([[-0.3105,-0.5413,0.0062]])
                },
                2: {
                    0: np.array([[-0.3203,-0.4258,-0.0517]]),
                    1: np.array([[-0.0481,-0.0799,0.0417]])
                },
                3: {
                    0: np.array([[-0.3555,-0.4238,-0.0850]]),
                    1: np.array([[-0.0351,-0.0553,0.0272]]),
                    2: np.array([[-0.0336,-0.0528,0.0339]])
                }
            }

        # Extract K matrices for current vehicle
        self.K_matrices = []
        if self.observer is not None:
            vehicle_id = self.observer.vehicle_id
            if vehicle_id in self.K_all_vehicles:
                for j in range(vehicle_id+1):
                    if j in self.K_all_vehicles[vehicle_id]:
                        self.K_matrices.append(self.K_all_vehicles[vehicle_id][j])
                    else:
                        self.K_matrices.append(None)
                        if self.logger:
                            self.logger.warning(f"K{vehicle_id}{j} not found, using None")
                if self.logger:
                    self.logger.info(f"Vehicle {vehicle_id}: Loaded {len(self.K_matrices)} K matrices (config-based)")
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
        Compute throttle using distributed control law:
        u_1 = K_{10} F_1 estimated_state
        u_2 = K_{20} F_2 estimated_state + K_{21} (F_2 - F_1) estimated_state + K_{32} (F_2 - F_3) estimated_state
        u_3 = K_{30} F_3 estimated_state + K_{32} (F_3 - F_2) estimated_state
        """
        if self.observer is None:
            if self.logger:
                self.logger.warning("No observer instance provided, returning zero throttle")
            return 0.0

        # Convert follower_state dict to numpy array for get_true_estimated_states
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

        estimated_states = self.get_true_estimated_states(local_state=local_state)
        if estimated_states is None or len(estimated_states) == 0:
            if self.logger:
                self.logger.warning("No estimated states available from V2V communication")
            return 0.0

        vehicle_id = self.observer.vehicle_id
        num_vehicles = self.observer.observer_size
        Fi = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=vehicle_id)

        # Get leader velocity for feedforward calculation
        current_time_ns = int(time.time() * 1e9)
        state_leader = self._get_true_vehicle_state(0, current_time_ns)
        leader_velocity = state_leader[3] if state_leader is not None else follower_state.get('velocity', 0.0)

        throttle_raw = self.feedforward_throttle(leader_velocity)

        # Distributed control law (strictly following the provided equations):
        # u_1 = K_{10} F_1 x_hat
        # u_2 = K_{20} F_2 x_hat + K_{21} (F_2 - F_1) x_hat + K_{32} (F_2 - F_3) x_hat
        # u_3 = K_{30} F_3 x_hat + K_{32} (F_3 - F_2) x_hat
        #
        # K_matrices index convention for vehicle i:
        # - index 0 => K_{i0}
        # - index 1 => K_{i1}
        # - index 2 => K_{i2}
        if vehicle_id == 1:
            if len(self.K_matrices) > 0 and self.K_matrices[0] is not None:
                K10 = self.K_matrices[0]
                throttle_raw += (K10 @ (Fi @ estimated_states))[0]
            elif self.logger:
                self.logger.warning("K10 is missing for vehicle 1")

        elif vehicle_id == 2:
            F1 = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=1)
            F3 = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=3) if num_vehicles >= 3 else None

            if len(self.K_matrices) > 0 and self.K_matrices[0] is not None:
                K20 = self.K_matrices[0]
                throttle_raw += (K20 @ (Fi @ estimated_states))[0]
            elif self.logger:
                self.logger.warning("K20 is missing for vehicle 2")

            if len(self.K_matrices) > 1 and self.K_matrices[1] is not None:
                K21 = self.K_matrices[1]
                throttle_raw += (K21 @ ((Fi - F1) @ estimated_states))[0]
            elif self.logger:
                self.logger.warning("K21 is missing for vehicle 2")

            # For vehicle 2 third term, use K32 as requested:
            # u2 third term = K32 * (F2 - F3) * x_hat
            K32_for_u2 = None
            if isinstance(self.K_all_vehicles, dict):
                K32_for_u2 = self.K_all_vehicles.get(3, {}).get(2)
            if K32_for_u2 is None and len(self.K_matrices) > 2:
                # Backward-compatible fallback if K32 is not explicitly provided
                K32_for_u2 = self.K_matrices[2]

            if F3 is not None and K32_for_u2 is not None:
                throttle_raw += (K32_for_u2 @ ((Fi - F3) @ estimated_states))[0]
            elif F3 is None and self.logger:
                self.logger.warning("F3 is unavailable (num_vehicles < 3), skipping K32(F2-F3) term")
            elif self.logger:
                self.logger.warning("K32 is missing for vehicle 2 third term")

        elif vehicle_id == 3:
            F2 = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=2)

            if len(self.K_matrices) > 0 and self.K_matrices[0] is not None:
                K30 = self.K_matrices[0]
                throttle_raw += (K30 @ (Fi @ estimated_states))[0]
            elif self.logger:
                self.logger.warning("K30 is missing for vehicle 3")

            if len(self.K_matrices) > 2 and self.K_matrices[2] is not None:
                K32 = self.K_matrices[2]
                throttle_raw += (K32 @ ((Fi - F2) @ estimated_states))[0]
            elif self.logger:
                self.logger.warning("K32 is missing for vehicle 3")
        else:
            if self.logger:
                self.logger.warning(
                    f"Vehicle {vehicle_id}: no strict formula defined (supported: 1,2,3), using feedforward only"
                )

        throttle = min(throttle_raw, self.max_throttle)
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
            
            # Calculate desired spacing d_i0
            di0 = follower_id * d

            # Special rule requested for d30 when current vehicle is 3:
            # d30 = 3*d + h*(v2 + v2 + v3)
            if vehicle_id == 3 and follower_id == 3:
                state_2 = self._get_true_vehicle_state(2, current_time_ns)
                if local_state is not None:
                    state_3 = local_state
                else:
                    state_3 = self._get_true_vehicle_state(3, current_time_ns)

                if state_2 is None or state_3 is None:
                    if self.logger:
                        self.logger.warning("get_true_estimated_states: Cannot get v2/v3 for custom d30")
                    return None

                v2 = state_2[3]
                v3 = state_3[3]
                di0 = 3 * d + h * (v3 + v3 + v3)
                if self.logger:
                    self.logger.info(f"Custom d30 for vehicle 3: d30 = {di0:.3f} (3*d + h*(v2+v2+v3))")
            else:
                # Default rule: d_i0 = i*d + h*sum(v_k, k=1..i)
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


    def feedforward_throttle(self, leader_velocity: float):
        """Compute feedforward throttle based on leader velocity (target velocity).
        
        Args:
            leader_velocity: Leader's velocity, used as the target velocity for all followers
        
        Returns:
            throttle_ff: Feedforward throttle to maintain leader velocity
        """
        v_desired = leader_velocity
        # throttle_ff = 0.329609 * v_desired**2 - 0.000272 * v_desired + 0.038744
        throttle_ff = 0.001889 * v_desired**2 + 0.155285 * v_desired + 0.005629
        return throttle_ff
