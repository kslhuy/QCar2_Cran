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
                 observer=None,
                 config=None,
                 logger=None):
        """
        Initialize State Feedback longitudinal controller based on the distributed luenberger observer.
        
        Args:
            max_throttle: Maximum throttle output
            throttle_smoothing: Exponential smoothing factor for throttle (0-1, higher = smoother)
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
        else:
            self.max_throttle = max_throttle
            self.throttle_smoothing = throttle_smoothing
        
        # Controller state
        self.prev_throttle = 0.0

        # Store K matrices directly for each vehicle
        # K_matrices[vehicle_id][j] = K_{vehicle_id,j}
        self.K_all_vehicles = {
            1: {
                0: np.array([[-0.1572, -0.4379, -0.0683]])
            },
            2: {
                0: np.array([[-0.1602, -0.3715, -0.0888]]),
                1: np.array([[-0.0084, -0.0189, 0.0109]])
            },
            3: {
                0: np.array([[-0.1733, -0.4259, -0.1236]]),
                1: np.array([[-0.0086, -0.0193, 0.0126]]),
                2: np.array([[-0.0053, -0.0045, 0.0156]])
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
        
    def compute_throttle(self, fleet_states: np.ndarray,  
                        dt: float, current_time_ns: int) -> float:
        """
        Compute throttle using state feedback controller based on distributed luenberger observer states.
        
        Args:
            - fleet_states: Complete fleet state matrix [state_dim x fleet_size], the first column is the leader vehicle state. The meaning of each row is defined as 
                - postion x 
                - position y
                - theta
                - velocity
                - acceleration
            - dt: Time step (seconds)
            - current_time_ns: Current time in nanoseconds
        Returns:
            - throttle: Computed throttle command (0 to max_throttle)
        """
        # Check if observer is available
        if self.observer is None:
            if self.logger:
                self.logger.warning("StateFeedbackController: No observer instance provided, returning zero throttle")
            return 0.0
        
        # Transform fleet states to the distributed luenberger observer states
        estimated_states, _ = self.observer._transfer_fleet_states_to_estimated_states(fleet_states, current_time_ns)

        vehicle_id = self.observer.vehicle_id
        num_vehicles = self.observer.fleet_size - 1
        Fi = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=vehicle_id)
    
        # State feedback control law: u_i = sum_{j=0}^{i-1} K_{ij} * (F_i - F_j) * estimated_states
        # For j=0 (leader), F_0 is zero matrix, so K_{i0} * F_i * estimated_states
        throttle_raw = 0.0
        
        # First term: K_{i0} * F_i * estimated_states (j=0)
        if len(self.K_matrices) > 0 and self.K_matrices[0] is not None:
            K_i0 = self.K_matrices[0]
            throttle_raw += (K_i0 @ (Fi @ estimated_states))[0]
        
        # Sum over preceding vehicles j=1 to i-1
        for j in range(1, vehicle_id):
            if j < len(self.K_matrices) and self.K_matrices[j] is not None:
                K_ij = self.K_matrices[j]
                Fj = self.calculate_Fi(num_vehicles=num_vehicles, vehicle_index=j)
                throttle_raw += (K_ij @ ((Fi - Fj) @ estimated_states))[0]

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
        throttle = max(throttle, 0.0)
        
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


