"""
Differentiator + UIO Observer with EKF-Style Dynamic Gains

Combines the best of both approaches:
1. Differentiator-based yaw rate derivative estimation (ṙ_hat)
2. UIO-style tire residual estimation from filtered differentiation 
3. EKF-style dynamic Kalman gain computation

Key improvements over DifferentiatorUIOObserver:
- Dynamic gains via EKF covariance propagation
- HighGain differentiator (faster, more robust)
- State clamping for numerical stability
- Aligned min_vx = 0.5 with parameters_qcar.yaml
- Tuned Q/R matrices for better estimation

State: x = [v_x, v_y, ψ, r, X, Y]ᵀ (6D)
Unknown inputs: w = [w_r, w_f]ᵀ (tire force residuals)
Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ (6D with lateral acceleration)

References:
    - Extended Kalman Filter for nonlinear state estimation
    - Dirty derivative / High-gain differentiator for rdot
    - UIO (Unknown Input Observer) for disturbance estimation
"""

import numpy as np
from typing import Optional, Dict, Tuple
from dataclasses import dataclass

# Import differentiators from centralized module
import sys
from pathlib import Path
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

from differentiators import (
    DirtyDerivative,
    HighGainDifferentiator,
    SlidingModeDifferentiator,
    create_differentiator,
    create_differentiator_from_config,
)

# Import observer components
from differentiator_uio_observer import WEstimatorUIOStyle

# Import centralized qLPV vehicle dynamics
sys.path.insert(0, str(parent_dir.parent))
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
    IDX8_VX, IDX8_VY, IDX8_PSI, IDX8_R, IDX8_X, IDX8_Y, IDX8_AX, IDX8_AY, STATE_DIM_8D,
    MEAS8_IDX_VX, MEAS8_IDX_R, MEAS8_IDX_PSI, MEAS8_IDX_X, MEAS8_IDX_Y, MEAS8_IDX_AY, MEAS8_IDX_AX, MEAS_DIM_7D,
    create_qlpv_dynamics,
)

try:
    from firstLayerObserverBase import FirstLayerObserverBase
except ImportError:
    from abc import ABC, abstractmethod
    class FirstLayerObserverBase(ABC):
        def __init__(self, state_dim: int = 4, unknown_input_dim: int = 2, sample_time: float = 0.02):
            self.state_dim = state_dim
            self.unknown_input_dim = unknown_input_dim
            self.Ts = sample_time
            self.state_hat = np.zeros(state_dim)
            self.f_uk_hat = np.zeros(unknown_input_dim)


class DifferentiatorUIOEKF(FirstLayerObserverBase):
    """
    Differentiator + UIO Observer with EKF-Style Dynamic Gains
    
    Combines:
    - HighGain differentiator for rdot estimation
    - UIO-style tire residual estimation 
    - EKF dynamic gain computation
    
    State vector: x = [v_x, v_y, ψ, r, X, Y]ᵀ
    Tire residuals: w = [w_r, w_f]ᵀ (estimated via UIO-style)
    """
    
    # State indices (6D state)
    IDX_VX = 0
    IDX_VY = 1
    IDX_PSI = 2
    IDX_R = 3
    IDX_X = 4
    IDX_Y = 5
    STATE_DIM = 6
    
    # Measurement indices (6D measurement)
    MEAS_IDX_VX = 0
    MEAS_IDX_R = 1
    MEAS_IDX_PSI = 2
    MEAS_IDX_X = 3
    MEAS_IDX_Y = 4
    MEAS_IDX_AY = 5
    MEAS_DIM = 6
    # 8D State indices
    IDX8_VX = 0
    IDX8_VY = 1
    IDX8_PSI = 2
    IDX8_R = 3
    IDX8_X = 4
    IDX8_Y = 5
    IDX8_AX = 6
    IDX8_AY = 7
    STATE_DIM_8D = 8
    
    # 7D Measurement indices
    MEAS8_IDX_VX = 0
    MEAS8_IDX_R = 1
    MEAS8_IDX_PSI = 2
    MEAS8_IDX_X = 3
    MEAS8_IDX_Y = 4
    MEAS8_IDX_AY = 5
    MEAS8_IDX_AX = 6
    MEAS_DIM_7D = 7

    def __init__(self, sample_time: float = 0.02,
                 vehicle_params: Optional[Dict] = None,
                 Q: Optional[np.ndarray] = None,
                 R: Optional[np.ndarray] = None,
                 P0: Optional[np.ndarray] = None,
                 tau_rdot: float = 0.02,  # Faster than default
                 ridge: float = 1e-6,
                 diff_type: str = 'highgain',  # Better differentiator
                 diff_params: Optional[Dict] = None,
                 use_8d_system: bool = False,
                 **kwargs):
        """
        Initialize Differentiator + UIO Observer with EKF Gains
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict
            Q: Process noise covariance
            R: Measurement noise covariance
            P0: Initial error covariance
            tau_rdot: Time constant for dirty derivative [s]
            ridge: Ridge regularization for w estimation
            diff_type: Differentiator type ('dirty', 'highgain', 'sliding')
            diff_params: Additional parameters for differentiator
            use_8d_system: Use 8D state vector
        """
        self.use_8d_system = use_8d_system
        if use_8d_system:
            self.state_dim = self.STATE_DIM_8D
            self.meas_dim = self.MEAS_DIM_7D
        else:
            self.state_dim = self.STATE_DIM
            self.meas_dim = self.MEAS_DIM
            
        super().__init__(
            state_dim=self.state_dim,
            unknown_input_dim=2,
            sample_time=sample_time
        )
        
        # Store diff type
        self.diff_type = diff_type
        
        # Vehicle parameters
        self.params = self._default_params()
        if vehicle_params is not None:
            self.params.update(vehicle_params)
        
        # Extract commonly used parameters
        self.lf = self.params['lf']
        self.lr = self.params['lr']
        self.m = self.params['m']
        self.Iz = self.params['Iz']
        self.Cf = self.params['Cf']
        self.Cr = self.params['Cr']
        self.mu = self.params.get('mu', 0.01)
        self.g = 9.81
        
        # State estimate
        self.state_hat = np.zeros(self.state_dim)
        
        # UIO-style w estimator with configurable differentiator
        diff_kwargs = diff_params or {}
        if diff_type == 'highgain':
            diff_kwargs.setdefault('omega', 50.0)  # Faster bandwidth
            diff_kwargs.setdefault('zeta', 0.707)  # Butterworth damping
        
        self.w_estimator = WEstimatorUIOStyle(
            Ts=sample_time,
            params=self.params,
            tau_rdot=tau_rdot,
            ridge=ridge,
            diff_type=diff_type,
            diff_params=diff_kwargs
        )
        
        # Tire residual estimates
        self.w_hat = np.zeros(2)
        
        # Minimum velocity threshold (from parameters_qcar.yaml)
        self.min_vx = self.params.get('vx_min', 0.1)  # Default fallback only for safety
        
        # Centralized vehicle dynamics (single source of truth)
        # Centralized vehicle dynamics
        # min_vx loaded from params
        self.min_vx = self.params.get('vx_min', 0.1) 
        self.dynamics = create_qlpv_dynamics(
            vehicle_params=self.params, 
            min_vx=self.min_vx,
            use_8d_system=use_8d_system
        )
        
        # === EKF Components ===
        # Error covariance
        self.P = P0 if P0 is not None else self._default_P0()
        
        # Process noise covariance
        self.Q = Q if Q is not None else self._default_Q()
        
        # Measurement noise covariance
        self.R = R if R is not None else self._default_R()
        
        # Kalman gain (computed dynamically)
        self.K = np.zeros((self.state_dim, self.meas_dim))
        
        # Numerical Jacobian step size
        self.epsilon = 1e-6
        
        # Diagnostics
        self.rdot_hat = 0.0
        self.residual = np.zeros(2)
        self.innovation = np.zeros(self.meas_dim)
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters (QCar scale) - uses centralized defaults"""
        params = get_default_vehicle_params()
        # params['vx_min'] = 0.15  # Override for EKF observer
        return params
    
    def _default_Q(self) -> np.ndarray:
        """Default process noise covariance Q"""
        if self.use_8d_system:
             return np.diag([
                 0.01,   # vx
                 0.1,    # vy
                 1e-4,   # psi
                 0.02,   # r
                 0.01,   # X
                 0.01,   # Y
                 0.05,   # ax
                 0.05,   # ay
             ])
        
        return np.diag([
            0.01,   # vx - well measured
            0.1,    # vy - less observable
            1e-4,   # psi - kinematic
            0.02,   # r - from gyro
            0.01,   # X - position
            0.01,   # Y - position
        ])
    
    def _default_R(self) -> np.ndarray:
        """Default measurement noise covariance R"""
        if self.use_8d_system:
             return np.diag([
                 0.1**2,   # vx
                 0.01**2,  # r
                 0.02**2,  # psi
                 0.2**2,   # X
                 0.2**2,   # Y
                 0.2**2,   # ay
                 0.2**2,   # ax
             ])
             
        return np.diag([
            0.1**2,   # vx - encoder
            0.01**2,  # r - gyro
            0.02**2,  # psi - heading
            0.2**2,   # X - position
            0.2**2,   # Y - position
            0.2**2,   # ay - accelerometer
        ])
    
    def _default_P0(self) -> np.ndarray:
        """Default initial error covariance P0"""
        if self.use_8d_system:
              return np.diag([
                  1.0,    # vx
                  1.0,    # vy
                  0.1,    # psi
                  0.1,    # r
                  5.0,    # X
                  5.0,    # Y
                  1.0,    # ax
                  1.0,    # ay
              ])
              
        return np.diag([
            1.0,    # vx
            1.0,    # vy
            0.1,    # psi
            0.1,    # r
            5.0,    # X
            5.0,    # Y
        ])
    
    def compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        """Compute scheduling parameters - delegates to centralized dynamics"""
        return self.dynamics.compute_scheduling_params(state, delta)
    
    def f_continuous(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """Continuous-time state dynamics - delegates to centralized dynamics"""
        return self.dynamics.f_continuous(x, u, w)
    
    def h_meas(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """Measurement function - delegates to centralized dynamics"""
        return self.dynamics.h_meas(x, u, w)
    
    
    def _numerical_jacobian_F(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """Compute Jacobian of f wrt x using central differences"""
        n = len(x)
        F = np.zeros((n, n))
        
        for j in range(n):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[j] += self.epsilon
            x_minus[j] -= self.epsilon
            
            f_plus = self.f_continuous(x_plus, u, w)
            f_minus = self.f_continuous(x_minus, u, w)
            
            F[:, j] = (f_plus - f_minus) / (2 * self.epsilon)
        
        return F
    
    def _numerical_jacobian_H(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """Compute Jacobian of h wrt x using central differences"""
        n = len(x)
        m = self.meas_dim
        H = np.zeros((m, n))
        
        for j in range(n):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[j] += self.epsilon
            x_minus[j] -= self.epsilon
            
            h_plus = self.h_meas(x_plus, u, w)
            h_minus = self.h_meas(x_minus, u, w)
            
            H[:, j] = (h_plus - h_minus) / (2 * self.epsilon)
        
        return H
    
    def update(self, measurement: np.ndarray, 
               control_input: np.ndarray,
               acceleration: Optional[np.ndarray] = None,
               gps_available: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update observer with new measurement
        
        Args:
            measurement: Measurement vector
            control_input: Control input [δ, a]
            acceleration: Full acceleration [ax, ay, az]
            gps_available: GPS validity flag
        
        Returns:
            Tuple of (state_estimate, tire_residuals)
        """
        # Process measurement
        y = self._process_measurement(measurement, acceleration)
        
        # Determine R_effective based on GPS
        R_effective = self.R.copy()
        if not gps_available:
             # Reduce confidence in GPS
             if self.use_8d_system:
                  R_effective[self.MEAS8_IDX_X, self.MEAS8_IDX_X] *= 1e6
                  R_effective[self.MEAS8_IDX_Y, self.MEAS8_IDX_Y] *= 1e6
                  R_effective[self.MEAS8_IDX_PSI, self.MEAS8_IDX_PSI] *= 1e6
             else:
                  R_effective[self.MEAS_IDX_X, self.MEAS_IDX_X] *= 1e6
                  R_effective[self.MEAS_IDX_Y, self.MEAS_IDX_Y] *= 1e6
                  R_effective[self.MEAS_IDX_PSI, self.MEAS_IDX_PSI] *= 1e6

        # Control input
        u = control_input.reshape(-1)
        delta = u[0]
        
        # === Step 1: UIO-Style w estimation ===
        if self.use_8d_system:
             r_meas = y[self.MEAS8_IDX_R]
             ay_meas = y[self.MEAS8_IDX_AY]
        else:
             r_meas = y[self.MEAS_IDX_R]
             ay_meas = y[self.MEAS_IDX_AY]
             
        self.w_hat, self.rdot_hat, self.residual = self.w_estimator.estimate(
            self.state_hat, r_meas, ay_meas, delta
        )
        
        # === Step 2: EKF Prediction ===
        # State prediction (Euler integration)
        x_dot = self.f_continuous(self.state_hat, u, self.w_hat)
        x_pred = self.state_hat + self.Ts * x_dot
        
        # Jacobian F (state transition)
        F = self._numerical_jacobian_F(self.state_hat, u, self.w_hat)
        
        # Discretize F: Φ ≈ I + F·Ts
        Phi = np.eye(self.state_dim) + F * self.Ts
        
        # Covariance prediction
        P_pred = Phi @ self.P @ Phi.T + self.Q
        
        # === Step 3: EKF Update ===
        # Jacobian H (measurement)
        H = self._numerical_jacobian_H(x_pred, u, self.w_hat)
        
        # Innovation covariance
        S = H @ P_pred @ H.T + R_effective
        
        # Kalman gain
        try:
            self.K = P_pred @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Fallback to pseudo-inverse
            self.K = P_pred @ H.T @ np.linalg.pinv(S)
        
        # Innovation
        y_pred = self.h_meas(x_pred, u, self.w_hat)
        self.innovation = y - y_pred
        
        # Wrap heading innovation to [-pi, pi]
        # Prevents instability when angle wraps from pi to -pi
        if gps_available:
            if self.use_8d_system:
                idx_psi_y = self.MEAS8_IDX_PSI
            else:
                idx_psi_y = self.MEAS_IDX_PSI
            self.innovation[idx_psi_y] = (self.innovation[idx_psi_y] + np.pi) % (2 * np.pi) - np.pi
        
        if not gps_available:
             # Zero innovation for GPS components to be safe, though R boosting handles it
             if self.use_8d_system:
                  self.innovation[self.MEAS8_IDX_X] = 0.0
                  self.innovation[self.MEAS8_IDX_Y] = 0.0
                  self.innovation[self.MEAS8_IDX_PSI] = 0.0
             else:
                  self.innovation[self.MEAS_IDX_X] = 0.0
                  self.innovation[self.MEAS_IDX_Y] = 0.0
                  self.innovation[self.MEAS_IDX_PSI] = 0.0
        
        # State update
        self.state_hat = x_pred + self.K @ self.innovation
        
        # Wrap heading state to [-pi, pi]
        # Ensures estimated yaw stays in the same range as GPS sensors
        if self.use_8d_system:
             idx_psi_state = self.IDX8_PSI
        else:
             idx_psi_state = self.IDX_PSI
        self.state_hat[idx_psi_state] = (self.state_hat[idx_psi_state] + np.pi) % (2 * np.pi) - np.pi
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(self.state_dim) - self.K @ H
        self.P = I_KH @ P_pred @ I_KH.T + self.K @ R_effective @ self.K.T
        
        # === Step 4: State clamping for numerical stability ===
        self.state_hat[0] = np.clip(self.state_hat[0], -10.0, 10.0) # vx
        self.state_hat[1] = np.clip(self.state_hat[1], -5.0, 5.0)   # vy
        self.state_hat[3] = np.clip(self.state_hat[3], -10.0, 10.0) # r
        
        # Clamp w_hat too
        self.w_hat = np.clip(self.w_hat, -500.0, 500.0)
        
        # Copy to base class for interface compatibility
        self.f_uk_hat = self.w_hat.copy()
        
        return self.state_hat.copy(), self.w_hat.copy()
    
    def _process_measurement(self, measurement: np.ndarray, 
                             acceleration: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Process input measurement to get full measurement vector.
        
        Handles:
            - Full 7D: [v_x, r, ψ, X, Y, a_y, a_x]
            - No GPS 4D: [v_x, r, a_y, a_x]
            - Legacy 6D: [v_x, r, ψ, X, Y, a_y]
            - Legacy 2D: [v_x, r]
            
        Args:
            measurement: Input measurement (various formats)
            acceleration: Optional 3D acceleration [a_x, a_y, a_z]
            
        Returns:
            Full measurement vector y
        """
        measurement = measurement.reshape(-1)
        n_input = len(measurement)
        y = np.zeros(self.meas_dim)
        
        # 1. Basic Kinematics (always present)
        val_vx = measurement[0]
        val_r = measurement[1]
        
        if self.use_8d_system:
            y[self.MEAS8_IDX_VX] = val_vx
            y[self.MEAS8_IDX_R] = val_r
        else:
            y[self.MEAS_IDX_VX] = val_vx
            y[self.MEAS_IDX_R] = val_r
            
        # 2. Extract values based on input dimension
        val_psi = None
        val_X = None
        val_Y = None
        val_ay = None
        val_ax = None
        
        if n_input >= 7:
            # Full 7D: [vx, r, psi, X, Y, ay, ax]
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]
            val_ay = measurement[5]
            val_ax = measurement[6]
        elif n_input == 6:
            # Legacy 6D: [vx, r, psi, X, Y, ay]
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]
            val_ay = measurement[5]
        elif n_input == 4:
            # No GPS 4D: [vx, r, ay, ax]
            val_ay = measurement[2]
            val_ax = measurement[3]
        elif n_input == 5:
            # Partial 5D: [vx, r, psi, X, Y]
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]
        
        # 3. Fill from Acceleration argument if missing
        if acceleration is not None:
             if val_ax is None: val_ax = acceleration[0]
             if val_ay is None: val_ay = acceleration[1]
             
        # 4. Fill missing GPS/State vars with current estimate
        if self.use_8d_system:
            idx_psi = self.IDX8_PSI
            idx_X = self.IDX8_X
            idx_Y = self.IDX8_Y
        else:
            idx_psi = self.IDX_PSI
            idx_X = self.IDX_X
            idx_Y = self.IDX_Y
            
        if val_psi is None: val_psi = self.state_hat[idx_psi]
        if val_X is None: val_X = self.state_hat[idx_X]
        if val_Y is None: val_Y = self.state_hat[idx_Y]
        
        # 5. Populate Result Vector
        if self.use_8d_system:
            y[self.MEAS8_IDX_PSI] = val_psi
            y[self.MEAS8_IDX_X] = val_X
            y[self.MEAS8_IDX_Y] = val_Y
            y[self.MEAS8_IDX_AY] = val_ay if val_ay is not None else 0.0
            y[self.MEAS8_IDX_AX] = val_ax if val_ax is not None else 0.0
        else:
            y[self.MEAS_IDX_PSI] = val_psi
            y[self.MEAS_IDX_X] = val_X
            y[self.MEAS_IDX_Y] = val_Y
            y[self.MEAS_IDX_AY] = val_ay if val_ay is not None else (val_r * val_vx)
            # 6D system typically doesn't hold AX in y
        
        return y
    
    def get_state(self) -> np.ndarray:
        """Get current 6D state estimate"""
        return self.state_hat.copy()
    
    def get_tire_residuals(self) -> np.ndarray:
        """Get current tire residual estimates [w_r, w_f]"""
        return self.w_hat.copy()
    
    def get_unknown_input(self) -> np.ndarray:
        """Get unknown input estimate (alias for tire residuals)"""
        return self.w_hat.copy()
    
    def get_rdot_estimate(self) -> float:
        """Get filtered yaw rate derivative"""
        return self.rdot_hat
    
    def get_kalman_gain(self) -> np.ndarray:
        """Get current Kalman gain matrix"""
        return self.K.copy()
    
    def get_covariance(self) -> np.ndarray:
        """Get error covariance matrix"""
        return self.P.copy()
    
    def reset(self, initial_state: Optional[np.ndarray] = None, 
              initial_position: Optional[np.ndarray] = None):
        """Reset observer state"""
        if initial_state is not None:
            initial_state = initial_state.reshape(-1)
            if len(initial_state) >= self.state_dim:
                self.state_hat = initial_state[:self.state_dim].copy()
            else:
                self.state_hat = np.zeros(self.state_dim)
                self.state_hat[:len(initial_state)] = initial_state
        else:
            self.state_hat = np.zeros(self.state_dim)
            if initial_position is not None:
                if self.use_8d_system:
                     self.state_hat[self.IDX8_X] = initial_position[0]
                     self.state_hat[self.IDX8_Y] = initial_position[1]
                else:
                     self.state_hat[self.IDX_X] = initial_position[0]
                     self.state_hat[self.IDX_Y] = initial_position[1]
        
        # Reset covariance
        self.P = self._default_P0()
        
        # Reset w estimator
        r0 = self.state_hat[self.IDX_R]
        self.w_estimator.reset(r0)
        
        # Reset estimates
        self.w_hat = np.zeros(2)
        self.f_uk_hat = np.zeros(2)
        self.K = np.zeros((self.state_dim, self.meas_dim))
        
        # Reset diagnostics
        self.rdot_hat = 0.0
        self.residual = np.zeros(2)
        self.innovation = np.zeros(self.meas_dim)


def create_differentiator_uio_ekf(sample_time: float = 0.02,
                                   vehicle_params: Optional[Dict] = None,
                                   **kwargs) -> DifferentiatorUIOEKF:
    """
    Factory function to create Differentiator + UIO + EKF observer
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dictionary
        **kwargs: Additional arguments
    
    Returns:
        Configured DifferentiatorUIOEKF instance
    """
    return DifferentiatorUIOEKF(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        **kwargs
    )


# ============================================================
# Test code
# ============================================================
if __name__ == '__main__':
    print("="*60)
    print("Differentiator + UIO + EKF Observer Test")
    print("="*60)
    
    # Create observer
    obs = create_differentiator_uio_ekf(sample_time=0.02)
    
    # Initial state
    initial = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    obs.reset(initial)
    
    print(f"Initial state: {obs.get_state()}")
    
    # Simple test
    n_steps = 50
    for i in range(n_steps):
        # Fake measurement
        measurement = np.array([1.0, 0.0, 0.0, float(i)*0.02, 0.0, 0.0])
        control = np.array([0.0, 0.5])  # Slight steering, acceleration
        
        state, w = obs.update(measurement, control)
    
    print(f"Final state: {obs.get_state()}")
    print(f"Tire residuals: {obs.get_tire_residuals()}")
    print(f"Kalman gain norm: {np.linalg.norm(obs.get_kalman_gain()):.4f}")
    print("\n✅ Basic test PASSED")
