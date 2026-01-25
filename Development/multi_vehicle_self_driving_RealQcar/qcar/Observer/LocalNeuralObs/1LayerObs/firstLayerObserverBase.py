"""
First-Layer Observers for Two-Layer Neural Observer Architecture

Provides the base class and factory function for first-layer observers that 
estimate both states and unknown inputs (tire force residuals).

Available observer types:
- 'qlpv': qLPV Augmented-State Observer with LMI-based gain design
- 'qlpv_kalman': qLPV Augmented-State Observer with EKF-style Kalman gains  
- 'differentiator_uio': Differentiator + UIO-style observer with LMI gains
- 'differentiator_uio_ekf': Differentiator + UIO observer with EKF gains

All observers implement the same interface for easy swapping.
"""

import numpy as np
from typing import Optional, Dict, Tuple
from abc import ABC, abstractmethod


class FirstLayerObserverBase(ABC):
    """
    Base class for first-layer observers
    
    Estimates:
        - State x̂ (6D or 8D)
        - Unknown input ŵ = [w_r, w_f]ᵀ (tire force residuals)
    
    The 6D state includes: 
        - v_x, v_y, ψ, r, X, Y
        where  - v_x: Longitudinal velocity
               - v_y: Lateral velocity
               - ψ: Yaw angle
               - r: Yaw rate
               - X: X-coordinate
               - Y: Y-coordinate
    
    The 8D state adds:
        - a_x, a_y (Accelerations)
    """
    
    def __init__(self, state_dim: int = 6, unknown_input_dim: int = 2,
                 sample_time: float = 0.02):
        """
        Initialize observer base
        
        Args:
            state_dim: State vector dimension (default 6D)
            unknown_input_dim: Unknown input dimension (tire residuals, default 2)
            sample_time: Sample time Ts [s]
        """
        self.state_dim = state_dim
        self.unknown_input_dim = unknown_input_dim
        self.Ts = sample_time
        
        # State estimate [v_x, v_y, ψ, r, X, Y]
        self.state_hat = np.zeros(state_dim)
        
        # Unknown input estimate [w_r, w_f]
        self.f_uk_hat = np.zeros(unknown_input_dim)
        
        # Observer matrices (set by concrete implementations)
        self.A = None
        self.B = None
        self.C = None
        self.D = None
    
    @abstractmethod
    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None,
               gps_available: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update observer with new measurement
        
        Args:
            measurement: Measurement vector y (format depends on implementation)
            control_input: Control input u = [δ, a]ᵀ (steering, acceleration)
            f_nn: Neural network output (for second-layer integration)
            acceleration: Optional IMU acceleration [a_x, a_y, a_z]
        
        Returns:
            Tuple of (state_estimate, unknown_input_estimate)
                - state_estimate: 6D state [v_x, v_y, ψ, r, X, Y]
                - unknown_input_estimate: [w_r, w_f] tire residuals
        """
        pass
    
    def get_state(self) -> np.ndarray:
        """Get current 6D state estimate [v_x, v_y, ψ, r, X, Y]"""
        return self.state_hat.copy()
    
    def get_unknown_input(self) -> np.ndarray:
        """Get current unknown input estimate [w_r, w_f]"""
        return self.f_uk_hat.copy()
    
    def reset(self, initial_state: Optional[np.ndarray] = None):
        """Reset observer state"""
        if initial_state is not None:
            initial_state = np.asarray(initial_state).reshape(-1)
            if len(initial_state) >= self.state_dim:
                self.state_hat = initial_state[:self.state_dim].copy()
            else:
                self.state_hat[:len(initial_state)] = initial_state
        else:
            self.state_hat = np.zeros(self.state_dim)
        
        self.f_uk_hat = np.zeros(self.unknown_input_dim)
    
    def get_4d_state(self) -> np.ndarray:
        """
        Extract 4D state for LocalStateEstimatorBase compatibility
        
        Returns:
            numpy array [X, Y, ψ, v_x] - position, heading, velocity
        """
        # Map: [v_x, v_y, ψ, r, X, Y] -> [X, Y, ψ, v_x]
        vx, vy, psi, r, X, Y = self.state_hat
        return np.array([X, Y, psi, vx])


# All available first-layer observer types
AVAILABLE_OBSERVER_TYPES = [
    'qlpv',              # qLPV with LMI-based gain design
    'qlpv_kalman',       # qLPV with EKF-style Kalman gains
    'differentiator_uio',     # Differentiator + UIO with LMI gains
    'differentiator_uio_ekf', # Differentiator + UIO with EKF gains
    'z_layer1',               # Z-style Sample-and-Hold Observer
]


def create_first_layer_observer(observer_type: str = 'qlpv', **kwargs) -> FirstLayerObserverBase:
    """
    Factory function to create first-layer observer instances
    
    Args:
        observer_type: Type of observer:
            - 'qlpv': qLPV Augmented-State Observer with LMI-based gain design
            - 'qlpv_kalman': qLPV Augmented-State Observer with EKF Kalman gains
            - 'differentiator_uio': Differentiator + UIO-style observer with LMI gains
            - 'differentiator_uio_ekf': Differentiator + UIO observer with EKF gains
            - 'z_layer1': Z-style Sample-and-Hold Observer
        **kwargs: Additional arguments for observer initialization
            - sample_time: Sample time [s] (default 0.02)
            - vehicle_params: Vehicle parameters dict
            - observer_gains: Observer gain parameters
            - Q, R, P0: EKF noise covariances (for kalman/ekf variants)
    
    Returns:
        First-layer observer instance (6D state, 2D unknown input)
    
    Example:
        >>> observer = create_first_layer_observer('qlpv', sample_time=0.02)
        >>> state, w = observer.update(measurement, control)
        >>> # Or use EKF-based observer
        >>> observer = create_first_layer_observer('qlpv_kalman', sample_time=0.02)
    """
    if observer_type == 'qlpv':
        # Import here to avoid circular imports
        from qlpv_observer import qLPVAugmentedObserver
        return qLPVAugmentedObserver(**kwargs)
    elif observer_type == 'qlpv_kalman':
        # Import here to avoid circular imports
        from qlpv_observer_kalma import qLPVKalmanObserver
        return qLPVKalmanObserver(**kwargs)
    elif observer_type == 'differentiator_uio':
        # Import here to avoid circular imports
        from differentiator_uio_observer import DifferentiatorUIOObserver
        return DifferentiatorUIOObserver(**kwargs)
    elif observer_type == 'differentiator_uio_ekf':
        # Import here to avoid circular imports
        from differentiator_uio_ekf import DifferentiatorUIOEKF
        return DifferentiatorUIOEKF(**kwargs)
    elif observer_type == 'z_layer1':
        # Import here to avoid circular imports
        from z_layer1_observer import ZLayer1Observer
        return ZLayer1Observer(**kwargs)
    else:
        raise ValueError(f"Unknown observer type: {observer_type}. "
                        f"Available: {AVAILABLE_OBSERVER_TYPES}")
