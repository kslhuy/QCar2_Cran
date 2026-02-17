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
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, TYPE_CHECKING
from ..longitudinal_controllers import LongitudinalControllerBase

if TYPE_CHECKING:
    from Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator


class StateFeedbackController(LongitudinalControllerBase):
    """
    CACC-based longitudinal controller
    Uses spacing error and velocity error to compute acceleration,
    then converts to throttle command
    """
    
    def __init__(self, 
                 max_throttle=0.3,
                 throttle_smoothing=0.7,
                 leader_fix_throttle=0.15,
                 observer=None,
                 config=None,
                 logger=None):
        """
        Initialize State Feedback longitudinal controller based on the distributed luenberger observer.
        
        Args:
            max_throttle: Maximum throttle output
            throttle_smoothing: Exponential smoothing factor for throttle (0-1, higher = smoother)
            leader_fix_throttle: Base throttle to maintain speed (should match leader's constant throttle)
            observer: DistributedLuenbergerEstimator instance for state estimation
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
            self.leader_fix_throttle = params.get('leader_fix_throttle', leader_fix_throttle)
        else:
            self.max_throttle = max_throttle
            self.throttle_smoothing = throttle_smoothing
            self.leader_fix_throttle = leader_fix_throttle
        
        # Controller state
        self.prev_throttle = 0.0

        # Store K matrices directly for each vehicle
        # K_matrices[vehicle_id][j] = K_{vehicle_id,j}
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
        Compute throttle using state feedback controller based on distributed luenberger observer states.
        
        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity' (following base controller interface)
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (not used - gets data from observer)
            dt: Time step (seconds)
            
        Returns:
            throttle: Computed throttle command (0 to max_throttle)
        """
        # Check if observer is available
        if self.observer is None:
            if self.logger:
                self.logger.warning("StateFeedbackController: No observer instance provided, returning zero throttle")
            return 0.0
        
        # Get fleet states from observer's fleet estimator
        # The observer maintains estimated states for all vehicles
        estimated_states = self.observer.estimated_state
        
        # Check if we have valid estimated states
        if estimated_states is None or len(estimated_states) == 0:
            if self.logger:
                self.logger.warning("StateFeedbackController: No estimated states available from observer")
            return 0.0
        
        # Get vehicle information
        vehicle_id = self.observer.vehicle_id
        num_vehicles = self.observer.observer_size  # Number of follower vehicles (not including leader)
        
        # Calculate index matrix Fi for this vehicle
        Fi = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=vehicle_id)
    
        # State feedback control law: u_i = sum_{j=0}^{i-1} K_{ij} * (F_i - F_j) * estimated_states
        # For j=0 (leader), F_0 is zero matrix, so K_{i0} * F_i * estimated_states
        throttle_raw = self.feedforward_throttle(follower_state)
        
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
        
        # Apply exponential smoothing to final throttle command
        throttle = (self.throttle_smoothing * self.prev_throttle + 
                   (1 - self.throttle_smoothing) * throttle_raw)
        
        # Ensure throttle is non-negative
        throttle = min(throttle, self.max_throttle)
        
        # Store for next iteration
        self.prev_throttle = throttle
        
        return throttle
    
    def reset(self):
        """Reset controller state"""
        self.prev_throttle = 0.0


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

    def feedforward_throttle(self, follower_state: Dict[str, float]):
        """Given target velocity, compute required throttle"""

        v_desired = follower_state.get("velocity", 0.0)
        # throttle_ff = 0.329609 * v_desired**2 - 0.000272 * v_desired + 0.038744
        # throttle_ff = 0.001889 * v_desired**2 + 0.155285 * v_desired + 0.005629
        throttle_ff = 0.156385 * v_desired + 0.005230
        return throttle_ff

