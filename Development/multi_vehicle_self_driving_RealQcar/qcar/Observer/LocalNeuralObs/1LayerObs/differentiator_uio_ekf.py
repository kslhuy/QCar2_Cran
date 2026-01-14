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
- Aligned min_vx = 0.15 with vehicle model
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
    
    def __init__(self, sample_time: float = 0.02,
                 vehicle_params: Optional[Dict] = None,
                 Q: Optional[np.ndarray] = None,
                 R: Optional[np.ndarray] = None,
                 P0: Optional[np.ndarray] = None,
                 tau_rdot: float = 0.02,  # Faster than default
                 ridge: float = 1e-6,
                 diff_type: str = 'highgain',  # Better differentiator
                 diff_params: Optional[Dict] = None,
                 **kwargs):
        """
        Initialize Differentiator + UIO Observer with EKF Gains
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict
            Q: Process noise covariance (6×6)
            R: Measurement noise covariance (6×6)
            P0: Initial error covariance (6×6)
            tau_rdot: Time constant for dirty derivative [s]
            ridge: Ridge regularization for w estimation
            diff_type: Differentiator type ('dirty', 'highgain', 'sliding')
            diff_params: Additional parameters for differentiator
        """
        super().__init__(
            state_dim=self.STATE_DIM,
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
        self.state_hat = np.zeros(self.STATE_DIM)
        
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
        
        # Minimum velocity threshold (aligned with vehicle model)
        self.min_vx = 0.15  # [m/s]
        
        # === EKF Components ===
        # Error covariance
        self.P = P0 if P0 is not None else self._default_P0()
        
        # Process noise covariance
        self.Q = Q if Q is not None else self._default_Q()
        
        # Measurement noise covariance
        self.R = R if R is not None else self._default_R()
        
        # Kalman gain (computed dynamically)
        self.K = np.zeros((self.STATE_DIM, self.MEAS_DIM))
        
        # Numerical Jacobian step size
        self.epsilon = 1e-6
        
        # Diagnostics
        self.rdot_hat = 0.0
        self.residual = np.zeros(2)
        self.innovation = np.zeros(self.MEAS_DIM)
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters (QCar scale) - uses centralized defaults"""
        params = get_default_vehicle_params()
        params['vx_min'] = 0.15  # Override for EKF observer
        return params
    
    def _default_Q(self) -> np.ndarray:
        """Default process noise covariance Q (6×6)"""
        return np.diag([
            0.01,   # vx - well measured
            0.1,    # vy - less observable
            1e-4,   # psi - kinematic
            0.02,   # r - from gyro
            0.01,   # X - position
            0.01,   # Y - position
        ])
    
    def _default_R(self) -> np.ndarray:
        """Default measurement noise covariance R (6×6)"""
        return np.diag([
            0.1**2,   # vx - encoder
            0.01**2,  # r - gyro
            0.02**2,  # psi - heading
            0.2**2,   # X - position
            0.2**2,   # Y - position
            0.2**2,   # ay - accelerometer
        ])
    
    def _default_P0(self) -> np.ndarray:
        """Default initial error covariance P0 (6×6)"""
        return np.diag([
            1.0,    # vx
            1.0,    # vy
            0.1,    # psi
            0.1,    # r
            5.0,    # X
            5.0,    # Y
        ])
    
    def compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        """Compute scheduling parameters"""
        return SchedulingParameters.from_state_and_input(state, delta, self.min_vx)
    
    def f_continuous(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """
        Continuous-time state dynamics
        
        ẋ = f(x, u, w)
        
        Args:
            x: State [vx, vy, psi, r, X, Y]
            u: Control [δ, a]
            w: Tire residuals [wr, wf]
        
        Returns:
            State derivative
        """
        vx = max(abs(x[self.IDX_VX]), self.min_vx)
        vy = x[self.IDX_VY]
        psi = x[self.IDX_PSI]
        r = x[self.IDX_R]
        
        delta = u[0]
        accel = u[1] if len(u) > 1 else 0.0
        
        wr = w[0]
        wf = w[1]
        
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        cos_delta = np.cos(delta)
        sin_delta = np.sin(delta)
        
        # Slip angles
        alpha_f = delta - vy / vx - self.lf * r / vx
        alpha_r = -vy / vx + self.lr * r / vx
        
        # Tire forces (linear + residuals)
        Fyf = self.Cf * alpha_f + wf
        Fyr = self.Cr * alpha_r + wr
        
        # State derivatives
        vx_dot = accel - self.mu * self.g + r * vy - Fyf * sin_delta / self.m
        vy_dot = (Fyr + Fyf * cos_delta) / self.m - r * vx
        psi_dot = r
        r_dot = (self.lf * Fyf * cos_delta - self.lr * Fyr) / self.Iz
        X_dot = vx * cos_psi - vy * sin_psi
        Y_dot = vx * sin_psi + vy * cos_psi
        
        return np.array([vx_dot, vy_dot, psi_dot, r_dot, X_dot, Y_dot])
    
    def h_meas(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """
        Measurement function y = h(x, u, w)
        
        Returns:
            Predicted measurement [vx, r, psi, X, Y, ay]
        """
        vx = max(abs(x[self.IDX_VX]), self.min_vx)
        vy = x[self.IDX_VY]
        r = x[self.IDX_R]
        
        delta = u[0]
        cos_delta = np.cos(delta)
        
        wr = w[0]
        wf = w[1]
        
        # Slip angles
        alpha_f = delta - vy / vx - self.lf * r / vx
        alpha_r = -vy / vx + self.lr * r / vx
        
        # Tire forces
        Fyf = self.Cf * alpha_f + wf
        Fyr = self.Cr * alpha_r + wr
        
        # Lateral acceleration
        ay = (Fyr + Fyf * cos_delta) / self.m
        
        return np.array([
            x[self.IDX_VX],
            r,
            x[self.IDX_PSI],
            x[self.IDX_X],
            x[self.IDX_Y],
            ay,
        ])
    
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
        m = self.MEAS_DIM
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
               control_input: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update observer with new measurement
        
        Args:
            measurement: Measurement vector [vx, r, psi, X, Y, ay]
            control_input: Control input [δ, a]
        
        Returns:
            Tuple of (state_estimate, tire_residuals)
        """
        # Process measurement
        y = self._process_measurement(measurement)
        
        # Control input
        u = control_input.reshape(-1)
        delta = u[0]
        
        # === Step 1: UIO-Style w estimation ===
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
        Phi = np.eye(self.STATE_DIM) + F * self.Ts
        
        # Covariance prediction
        P_pred = Phi @ self.P @ Phi.T + self.Q
        
        # === Step 3: EKF Update ===
        # Jacobian H (measurement)
        H = self._numerical_jacobian_H(x_pred, u, self.w_hat)
        
        # Innovation covariance
        S = H @ P_pred @ H.T + self.R
        
        # Kalman gain
        try:
            self.K = P_pred @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Fallback to pseudo-inverse
            self.K = P_pred @ H.T @ np.linalg.pinv(S)
        
        # Innovation
        y_pred = self.h_meas(x_pred, u, self.w_hat)
        self.innovation = y - y_pred
        
        # State update
        self.state_hat = x_pred + self.K @ self.innovation
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(self.STATE_DIM) - self.K @ H
        self.P = I_KH @ P_pred @ I_KH.T + self.K @ self.R @ self.K.T
        
        # === Step 4: State clamping for numerical stability ===
        self.state_hat[self.IDX_VX] = np.clip(self.state_hat[self.IDX_VX], -10.0, 10.0)
        self.state_hat[self.IDX_VY] = np.clip(self.state_hat[self.IDX_VY], -5.0, 5.0)
        self.state_hat[self.IDX_R] = np.clip(self.state_hat[self.IDX_R], -10.0, 10.0)
        
        # Clamp w_hat too
        self.w_hat = np.clip(self.w_hat, -500.0, 500.0)
        
        # Copy to base class for interface compatibility
        self.f_uk_hat = self.w_hat.copy()
        
        return self.state_hat.copy(), self.w_hat.copy()
    
    def _process_measurement(self, measurement: np.ndarray) -> np.ndarray:
        """Process input measurement to get full 6D measurement vector"""
        measurement = measurement.reshape(-1)
        y = np.zeros(self.MEAS_DIM)
        
        if len(measurement) >= 6:
            y = measurement[:6].copy()
        elif len(measurement) == 5:
            y[:5] = measurement[:5]
            y[5] = self.state_hat[self.IDX_R] * self.state_hat[self.IDX_VX]
        elif len(measurement) == 2:
            y[self.MEAS_IDX_VX] = measurement[0]
            y[self.MEAS_IDX_R] = measurement[1]
            y[self.MEAS_IDX_PSI] = self.state_hat[self.IDX_PSI]
            y[self.MEAS_IDX_X] = self.state_hat[self.IDX_X]
            y[self.MEAS_IDX_Y] = self.state_hat[self.IDX_Y]
        else:
            raise ValueError(f"Unsupported measurement dimension: {len(measurement)}")
        
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
    
    def reset(self, initial_state: Optional[np.ndarray] = None):
        """Reset observer state"""
        if initial_state is not None:
            initial_state = initial_state.reshape(-1)
            if len(initial_state) >= self.STATE_DIM:
                self.state_hat = initial_state[:self.STATE_DIM].copy()
            else:
                self.state_hat = np.zeros(self.STATE_DIM)
                self.state_hat[:len(initial_state)] = initial_state
        else:
            self.state_hat = np.zeros(self.STATE_DIM)
        
        # Reset covariance
        self.P = self._default_P0()
        
        # Reset w estimator
        r0 = self.state_hat[self.IDX_R]
        self.w_estimator.reset(r0)
        
        # Reset estimates
        self.w_hat = np.zeros(2)
        self.f_uk_hat = np.zeros(2)
        self.K = np.zeros((self.STATE_DIM, self.MEAS_DIM))
        
        # Reset diagnostics
        self.rdot_hat = 0.0
        self.residual = np.zeros(2)
        self.innovation = np.zeros(self.MEAS_DIM)


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
