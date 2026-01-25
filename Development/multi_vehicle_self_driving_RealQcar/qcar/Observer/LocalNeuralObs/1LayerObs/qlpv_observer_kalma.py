"""
qLPV Augmented-State Observer with EKF-style Gain Computation

Implements a quasi-Linear Parameter-Varying (qLPV) augmented-state observer
with tire-residual estimation using Extended Kalman Filter (EKF) methodology
for dynamic observer gain computation.

State: x = [v_x, v_y, ψ, r, X, Y]ᵀ (6D)
Augmented state: x_a = [x; w_r; w_f]ᵀ (8D with tire residuals)
Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ (6D with lateral acceleration)

Observer Equation:
    ẋ̂_a = A_a(ρ̂)·x̂_a + B_a(ρ̂)·u + L_a(ρ̂)·(y − C_a(ρ̂)·x̂_a − D(ρ̂)·u)
EKF Observer Equations:
    Predict:
        x̂_a⁻ = f(x̂_a, u)
        P⁻ = F·P·Fᵀ + Q
    
    Update:
        K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
        x̂_a = x̂_a⁻ + K·(y - h(x̂_a⁻, u))
        P = (I - K·H)·P⁻·(I - K·H)ᵀ + K·R·Kᵀ  (Joseph form)

where:
    - x̂_a = [x̂; ŵ_r; ŵ_f] is the augmented state estimate
    - ρ = {1/v_x, sin(δ), cos(δ), v_x, v_y, sin(ψ), cos(ψ)} scheduling parameters
    - w = [w_r, w_f] are tire force residuals (unknown inputs)
    - a_y provides algebraic constraint on w
    - P is the error covariance matrix
    - K is the Kalman gain (computed dynamically)
    - Q is process noise covariance
    - R is measurement noise covariance
    - F, H are Jacobians of f and h
References:
    - qLPV vehicle dynamics with tire-residual estimation
    - Extended Kalman Filter for nonlinear state estimation
    - UIO (Unknown Input Observer) for disturbance estimation
"""

import numpy as np
from typing import Optional, Dict, Tuple
from dataclasses import dataclass

# Import base class
import sys
from pathlib import Path
from scipy.linalg import expm
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

try:
    from firstLayerObserverBase import FirstLayerObserverBase
except ImportError:
    # Fallback: define minimal base class
    from abc import ABC, abstractmethod
    class FirstLayerObserverBase(ABC):
        def __init__(self, state_dim: int = 4, unknown_input_dim: int = 2, sample_time: float = 0.02):
            self.state_dim = state_dim
            self.unknown_input_dim = unknown_input_dim
            self.Ts = sample_time
            self.state_hat = np.zeros(state_dim)
            self.f_uk_hat = np.zeros(unknown_input_dim)

# Import centralized qLPV vehicle dynamics
sys.path.insert(0, str(parent_dir.parent))
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM, AUGMENTED_DIM, IDX_WR, IDX_WF,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
    IDX8_VX, IDX8_VY, IDX8_PSI, IDX8_R, IDX8_X, IDX8_Y, IDX8_AX, IDX8_AY, STATE_DIM_8D,
    MEAS8_IDX_VX, MEAS8_IDX_R, MEAS8_IDX_PSI, MEAS8_IDX_X, MEAS8_IDX_Y, MEAS8_IDX_AY, MEAS8_IDX_AX, MEAS_DIM_7D,
    AUGMENTED_DIM_10D, IDX8_WR, IDX8_WF,
    create_qlpv_dynamics,
)


# Note: SchedulingParameters is imported from qlpv_vehicle_dynamics_obs


class qLPVKalmanObserver(FirstLayerObserverBase):
    """
    qLPV Augmented-State Observer with EKF-style Kalman Gain Computation
    
    Estimates both vehicle states and unknown tire force residuals using
    an augmented-state qLPV observer structure with dynamically computed
    Kalman gains based on error covariance propagation.
    
    State vector: x = [v_x, v_y, ψ, r, X, Y]ᵀ
        - v_x: Longitudinal velocity (body frame)
        - v_y: Lateral velocity (body frame)
        - ψ: Yaw angle
        - r: Yaw rate
        - X: Global X position
        - Y: Global Y position
    
    Tire residuals: w = [w_r, w_f]ᵀ
        - w_r: Rear tire force residual = F_yr - C_r·α_r
        - w_f: Front tire force residual = F_yf - C_f·α_f
    
    Augmented state: x_a = [x; w]ᵀ (8-dimensional)
    
    Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ
        - a_y: Lateral acceleration (gives algebraic handle on w)
    
    Key Feature: Kalman gain K is computed at each time step using the
    EKF predict-update cycle, providing optimal state estimation.
    """
    
    # State indices (6D state)
    IDX_VX = 0
    IDX_VY = 1
    IDX_PSI = 2
    IDX_R = 3
    IDX_X = 4
    IDX_Y = 5
    STATE_DIM = 6
    
    # Augmented state includes tire residuals
    IDX_WR = 6  # Rear tire residual
    IDX_WF = 7  # Front tire residual
    AUGMENTED_DIM = 8
    
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
    
    # 8D Augmented state
    IDX8_WR = 8
    IDX8_WF = 9
    AUGMENTED_DIM_10D = 10
    
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
                 include_gyro_bias: bool = False,
                 use_8d_system: bool = False,
                 **kwargs):
        """
        Initialize qLPV Augmented-State Observer with EKF-style Gain Computation
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict
            Q: Process noise covariance
            R: Measurement noise covariance
            P0: Initial error covariance
            include_gyro_bias: Whether to include gyro bias state
            use_8d_system: Use 8D state system
        """
        self.use_8d_system = use_8d_system
        if use_8d_system:
             self.state_dim = self.STATE_DIM_8D
             self.augmented_dim = self.AUGMENTED_DIM_10D
             self.meas_dim = self.MEAS_DIM_7D
        else:
             self.state_dim = self.STATE_DIM
             self.augmented_dim = self.AUGMENTED_DIM
             self.meas_dim = self.MEAS_DIM

        # Initialize base class
        super().__init__(
            state_dim=self.state_dim,
            unknown_input_dim=2,  # [w_r, w_f]
            sample_time=sample_time
        )
        
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
        self.mu = self.params.get('mu', 0.01) # Default small friction
        self.g = 9.81  # Gravity
        
        # Augmented state
        self.state_augmented = np.zeros(self.augmented_dim)
        
        # Initialize state estimate
        self.state_hat = np.zeros(self.state_dim)
        
        # Tire residual estimates
        self.w_hat = np.zeros(2)  # [w_r, w_f]
        
        # =====================================================
        # EKF Covariance Matrices (Q, R, P)
        # =====================================================
        
        # Process noise covariance Q
        if Q is not None:
            self.Q = Q
        else:
            self.Q = self._default_Q() * sample_time
        
        # Measurement noise covariance R
        if R is not None:
            self.R = R
        else:
            self.R = self._default_R()
        
        # Error covariance matrix P
        if P0 is not None:
            self.P = P0.copy()
        else:
            self.P = self._default_P0()
        
        # Kalman gain K - computed dynamically
        self.K = np.zeros((self.augmented_dim, self.meas_dim))
        
        # Store last innovation for diagnostics
        self.innovation = np.zeros(self.meas_dim)
        
        # Gyro bias estimation (optional)
        self.include_gyro_bias = include_gyro_bias
        self.gyro_bias = 0.0
        
        # UIO residual for a_y constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0  # m·ã_y ≈ w_r + cos(δ)·w_f
        
        # Minimum velocity threshold (from parameters_qcar.yaml)
        # Lower value allows estimation closer to zero when vehicle stops
        self.min_vx = self.params['vx_min']
        
        # Centralized vehicle dynamics
        self.dynamics = create_qlpv_dynamics(
            vehicle_params=self.params,
            min_vx=self.min_vx,
            use_8d_system=use_8d_system
        )
        
        # Numerical Jacobian step size
        self.epsilon = 1e-6
        

    
    def _safe_inverse(self, S: np.ndarray) -> np.ndarray:
        """Compute safe inverse of S"""
        try:
            return np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Add small regularization
            S_reg = S + np.eye(S.shape[0]) * 1e-6
            return np.linalg.inv(S_reg)
            
    def _discretize_augmented(self, A_a: np.ndarray, B_a: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Discretize continuous augmented system matrices using ZOH.
        
        Args:
            A_a: Continuous augmented state matrix
            B_a: Continuous augmented input matrix
            dt: Sample time
            
        Returns:
            Tuple of (A_ad, B_ad)
        """
        n_states = A_a.shape[0]
        n_inputs = B_a.shape[1]
        
        # Van Loan's matrix exponential method
        # M = [[A_a, B_a], [0, 0]]
        M = np.zeros((n_states + n_inputs, n_states + n_inputs))
        M[:n_states, :n_states] = A_a
        M[:n_states, n_states:] = B_a
        
        # Compute matrix exponential
        M_exp = expm(M * dt)
        
        # Extract discrete matrices
        A_ad = M_exp[:n_states, :n_states]
        B_ad = M_exp[:n_states, n_states:]
        
        return A_ad, B_ad
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters - uses centralized defaults"""
        return get_default_vehicle_params()
    
    def _default_Q(self) -> np.ndarray:
        """
        Default process noise covariance Q (8×8) or (10×10)
        
        xa = [vx, vy, psi, r, X, Y, wr, wf] or [vx, vy, psi, r, X, Y, ax, ay, wr, wf]
        
        - Small for kinematics (vx, psi, X, Y)
        - Moderate for dynamics (vy, r)
        - Large for tire residuals w (random-walk assumption)
        """
        if self.use_8d_system:
             # Extend Q to 10D
             # 8D state: [vx, vy, psi, r, X, Y, ax, ay] + [wr, wf]
             return np.diag([
                0.01,   # vx
                0.1,    # vy
                1e-4,   # psi
                0.02,   # r
                0.01,   # X
                0.01,   # Y
                0.05,   # ax (new state)
                0.05,   # ay (new state)
                5.0,    # wr
                5.0,    # wf
             ])
             
        return np.diag([
            0.01,   # vx - longitudinal velocity (well measured)
            0.1,    # vy - lateral velocity (less observable)
            1e-4,   # psi - yaw angle (kinematic)
            0.02,   # r - yaw rate (from gyro)
            0.01,   # X - position (from GPS)
            0.01,   # Y - position (from GPS)
            5.0,    # wr - rear tire residual (slowly varying)
            5.0,    # wf - front tire residual (slowly varying)
        ])
    
    def _default_R(self) -> np.ndarray:
        """
        Default measurement noise covariance R (6×6) or (7×7)
        
        y = [vx, r, psi, X, Y, ay] (+ ax)
        
        Tune based on actual sensor characteristics.
        """
        if self.use_8d_system:
             # y = [vx, r, psi, X, Y, ay, ax]
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
            0.1**2,   # vx - velocity sensor noise (good encoder)
            0.01**2,  # r - gyroscope noise (good IMU)
            0.02**2,  # psi - heading noise (integrated gyro)
            0.2**2,   # X - position noise (reduced for small vehicle)
            0.2**2,   # Y - position noise (reduced for small vehicle)
            0.2**2,   # ay - accelerometer noise
        ])
    
    def _default_P0(self) -> np.ndarray:
        """
        Default initial error covariance P0
        
        Reflects initial uncertainty in state estimate.
        """
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
                 100.0,  # wr
                 100.0,  # wf
             ])
             
        return np.diag([
            1.0,    # vx
            1.0,    # vy
            0.1,    # psi
            0.1,    # r
            5.0,    # X
            5.0,    # Y
            100.0,  # wr - high initial uncertainty
            100.0,  # wf - high initial uncertainty
        ])
    
        if self.use_8d_system:
             P0_8d = np.diag([
                 1.0,    # vx
                 1.0,    # vy
                 0.1,    # psi
                 0.1,    # r
                 5.0,    # X
                 5.0,    # Y
                 1.0,    # ax
                 1.0,    # ay
                 100.0,  # wr
                 100.0,  # wf
             ])
             return P0_8d
             
        return P0_default
    
    # =========================================================
    # Numerical Jacobian Computation
    # =========================================================
    
    def _numerical_jacobian(self, func, x: np.ndarray) -> np.ndarray:
        """
        Compute numerical Jacobian of func at x using central differences
        
        Args:
            func: Function f(x) -> y
            x: Point to compute Jacobian at
            
        Returns:
            Jacobian matrix J[i,j] = df_i/dx_j
        """
        n = len(x)
        f0 = func(x)
        m = len(f0)
        J = np.zeros((m, n))
        
        for j in range(n):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[j] += self.epsilon
            x_minus[j] -= self.epsilon
            J[:, j] = (func(x_plus) - func(x_minus)) / (2 * self.epsilon)
        
        return J
    
    def compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        """Compute scheduling parameters from current state and input"""
        return self.dynamics.compute_scheduling_params(state, delta)
    
    def compute_slip_angles(self, state: np.ndarray, delta: float) -> Tuple[float, float]:
        """Compute front and rear slip angles - delegates to centralized dynamics"""
        return self.dynamics.compute_slip_angles(state, delta)
    
    def compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute state matrix A(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_A_matrix(rho)
    
    def compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute input matrix B(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_B_matrix(rho)
    
    def compute_E_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute residual injection matrix E(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_E_matrix(rho)
    
    def compute_C_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute output matrix C(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_C_matrix(rho)
    
    def compute_D_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute feedthrough matrix D(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_D_matrix(rho)
    
    def compute_F_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute residual-to-output matrix F(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_F_matrix(rho)
    
    def compute_augmented_matrices(self, rho: SchedulingParameters) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute augmented system matrices - delegates to centralized dynamics"""
        return self.dynamics.compute_augmented_matrices(rho)
    
    def compute_ay_innovation(self, measurement: np.ndarray, state_hat: np.ndarray,
                              control_input: np.ndarray, rho: SchedulingParameters) -> float:
        """
        Compute lateral acceleration innovation for UIO-style residual estimation
        
        ã_y = a_y - (C_ay·x̂ + D_ay·u)
        
        Constraint: m·ã_y ≈ w_r + cos(δ)·w_f
        
        Args:
            measurement: Measurement vector [v_x, r, ψ, X, Y, a_y]
            state_hat: State estimate [v_x, v_y, ψ, r, X, Y]
            control_input: Control [δ, a]
            rho: Scheduling parameters
            
        Returns:
            a_y innovation value
        """
        C = self.compute_C_matrix(rho)
        D = self.compute_D_matrix(rho)
        
        # Extract a_y row
        C_ay = C[self.MEAS_IDX_AY, :]
        D_ay = D[self.MEAS_IDX_AY, :]
        
        # Predicted a_y from linear model
        ay_predicted = C_ay @ state_hat + D_ay @ control_input
        
        # Actual a_y measurement
        ay_measured = measurement[self.MEAS_IDX_AY]
        
        # Innovation
        ay_innovation = ay_measured - ay_predicted
        
        return ay_innovation
    
    # =========================================================
    # Discrete Dynamics for EKF
    # =========================================================
    
    def f_discrete(self, xa: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Discrete-time state transition function with kinematic/dynamic blending.
        
        xa[k+1] = f(xa[k], u[k])
        
        At low speeds, uses kinematic bicycle model to avoid singularities.
        At higher speeds, uses full qLPV dynamic model.
        Smooth blending in transition region.
        
        Args:
            xa: Augmented state [vx, vy, psi, r, X, Y, wr, wf]
            u: Control input [δ, a]
            
        Returns:
            Predicted augmented state at next time step
        """
        # Extract 6D state and tire residuals
        x = xa[:self.state_dim]
        w = xa[self.state_dim:]
        
        # Use blended dynamics (kinematic at low speed, dynamic at high speed)
        # Thresholds: pure kinematic below 0.1 m/s, pure dynamic above 0.3 m/s
        blend_vx_low = 0.1   # Pure kinematic below this
        blend_vx_high = 0.3  # Pure dynamic above this
        
        # # RK4 Integration with blended dynamics
        # k1 = self.dynamics.f_continuous(x, u, w)
        # k2 = self.dynamics.f_continuous(x + 0.5 * self.Ts * k1, u, w)
        # k3 = self.dynamics.f_continuous(x + 0.5 * self.Ts * k2, u, w)
        # k4 = self.dynamics.f_continuous(x + self.Ts * k3, u, w)
        
        # x_next = x + (self.Ts / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        x_next = x + self.dynamics.f_continuous(x, u, w)*self.Ts
        


        
        # At very low speeds, apply additional decay and clamping
        vx_abs = abs(x_next[self.IDX_VX])
        
        # # Clamp very small velocities to zero (dead zone)
        # VELOCITY_DEAD_ZONE = 0.02  # Below this, snap to zero
        # if vx_abs < VELOCITY_DEAD_ZONE:
        #     x_next[self.IDX_VX] = 0.0
        #     x_next[self.IDX_VY] = 0.0
        #     x_next[self.IDX_R] = 0.0  # Also zero yaw rate when stopped
        # elif vx_abs < blend_vx_low:
        #     # Exponential decay of vy and r at low speed
        #     decay_rate = 0.85  # Per time step (faster decay)
        #     x_next[self.IDX_VY] *= decay_rate
        #     x_next[self.IDX_R] *= decay_rate
        
        # Tire residual dynamics (random-walk: ẇ = 0)
        # At low speed, also decay tire residuals since they're less meaningful
        w_next = w.copy()
        if vx_abs < blend_vx_low:
            w_next *= 0.9  # Faster decay towards zero when stopped
        
        # Assemble next state
        xa_next = np.zeros(self.augmented_dim)
        xa_next[:self.state_dim] = x_next
        xa_next[self.state_dim:] = w_next
        
        return xa_next
    
    def h_meas(self, xa: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Measurement function with kinematic/dynamic blending.
        
        y = h(xa, u) = [vx, r, psi, X, Y, ay]
        
        At low speeds, uses kinematic approximation for ay.
        At high speeds, uses full tire force model.
        
        Args:
            xa: Augmented state [vx, vy, psi, r, X, Y, wr, wf]
            u: Control input [δ, a]
            
        Returns:
            Predicted measurement vector
        """
        # Extract state and tire residuals
        x = xa[:self.state_dim]
        w = xa[self.state_dim:]
        
        # Use centralized measurement function
        return self.dynamics.h_meas(x, u, w)
    
    # =========================================================
    # EKF Predict + Update
    # =========================================================
    
    def ekf_predict(self, xa: np.ndarray, P: np.ndarray, 
                    u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF Prediction Step
        
        x̂⁻ = f(x̂, u)
        P⁻ = F·P·Fᵀ + Q
        
        Args:
            xa: Current augmented state estimate
            P: Current error covariance
            u: Control input
            
        Returns:
            Tuple of (predicted state, predicted covariance)
        """
        # Calculate scheduling parameters
        # xa is augmented state, we need first 6 elements
        rho = self.compute_scheduling_params(xa[:self.state_dim], u[0])
        
        # Compute continuous augmented matrices
        A_a, B_a, _ = self.compute_augmented_matrices(rho)
        
        # Discretize using ZOH
        A_ad, B_ad = self._discretize_augmented(A_a, B_a, self.Ts)
        
        # State prediction using discrete linear dynamics: x[k+1] = A_d·x[k] + B_d·u[k]
        xa_pred = A_ad @ xa + B_ad @ u
        
        # Jacobian F is just the discrete state matrix A_ad
        F = A_ad
        
        # Covariance prediction
        P_pred = F @ P @ F.T + self.Q
        
        return xa_pred, P_pred
    
    def ekf_update(self, xa_pred: np.ndarray, P_pred: np.ndarray,
                   y_meas: np.ndarray, u: np.ndarray,
                   R_matrix: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        EKF Update Step
        
        K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
        x̂ = x̂⁻ + K·(y - h(x̂⁻, u))
        P = (I - K·H)·P⁻·(I - K·H)ᵀ + K·R·Kᵀ  (Joseph form)
        
        Args:
            xa_pred: Predicted augmented state
            P_pred: Predicted error covariance
            y_meas: Actual measurement vector
            u: Control input
            R_matrix: Optional R matrix to use (ovverrides self.R)
            
        Returns:
            Tuple of (updated state, updated covariance, innovation, predicted measurement)
        """
        # Use provided R or default
        R = R_matrix if R_matrix is not None else self.R
        
        # Calculate scheduling parameters from predicted state
        # xa_pred is augmented state, we need first 6 elements
        rho = self.compute_scheduling_params(xa_pred[:self.state_dim], u[0])
        
        # Compute analytical C matrix (qLPV)
        # We need augmented C matrix to account for tire residuals w
        # compute_augmented_matrices returns C_a (meas_dim x augmented_dim)
        _, _, C_a = self.compute_augmented_matrices(rho)
        
        # H is simply C_a in the linear/qLPV update
        H = C_a
        
        # Compute Feedthrough D matrix 
        # If 8D system, D assumption is usually 0 if measurements are states
        # dynamics.compute_D_matrix returns 6x2 (for 6D system) which might be wrong shape for 8D (7x8)
        # So we handle D dimension carefully
        if self.use_8d_system:
             D = np.zeros((self.meas_dim, 2))
        else:
             D = self.compute_D_matrix(rho)
             
        # Measurement prediction using Linear/qLPV model: y = C_a·x_a + D·u
        y_pred = C_a @ xa_pred + D @ u
        
        # Innovation (measurement residual)
        innov = y_meas - y_pred
        
        # Wrap heading innovation to [-pi, pi]
        # Prevents instability when angle wraps from pi to -pi
        if self.use_8d_system:
            idx_psi_meas = self.MEAS8_IDX_PSI
        else:
            idx_psi_meas = self.MEAS_IDX_PSI
             
        innov[idx_psi_meas] = (innov[idx_psi_meas] + np.pi) % (2 * np.pi) - np.pi
        
        # Innovation covariance S = H·P⁻·Hᵀ + R
        S = H @ P_pred @ H.T + R
        
        # Kalman gain K = P⁻·Hᵀ·S⁻¹
        try:
            K = P_pred @ H.T @ self._safe_inverse(S)
        except np.linalg.LinAlgError:
            # Fallback: use pseudo-inverse
            K = P_pred @ H.T @ np.linalg.pinv(S)
        
        # State update
        xa_upd = xa_pred + K @ innov
        
        # Covariance update (Joseph form for numerical stability)
        I = np.eye(P_pred.shape[0])
        IKH = I - K @ H
        P_upd = IKH @ P_pred @ IKH.T + K @ R @ K.T
        
        # Enforce symmetry and PSD
        P_upd = 0.5 * (P_upd + P_upd.T)
        
        return xa_upd, P_upd, innov, y_pred
    
    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None,
               gps_available: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update qLPV augmented-state observer with EKF predict-update cycle
        
        Args:
            measurement: Measurement vector.
            control_input: Control [δ, a] 
            f_nn: Neural network output (not used)
            acceleration: Full 3D acceleration [a_x, a_y, a_z] if a_y not in measurement
            gps_available: Whether GPS measurements [X, Y] (and possibly ψ) are valid
            
        Returns:
            Tuple of (state_estimate, tire_residual_estimate)
        """
        measurement = measurement.reshape(-1)
        
        # Determine effective R matrix and handle missing measurements
        # If gps_available is False, we boost variance for X, Y (, psi?)
        
        R_effective = self.R.copy()
        
        # Identify indices based on system
        # Indice are the same for 6d and 8d
        idx_X = self.MEAS8_IDX_X 
        idx_Y = self.MEAS8_IDX_Y
        idx_PSI = self.MEAS8_IDX_PSI
             
        if not gps_available or len(measurement) <= 4:
            # Boost variance for GPS related measurements
            # If measurement is 4D (no GPS), we force this logic even if gps_available was passed as True
            high_variance = 1e6
            R_effective[idx_X, idx_X] = high_variance
            R_effective[idx_Y, idx_Y] = high_variance
            # Often psi comes from magnetometer/IMU so it might still be valid? 
            # But prompt implies different C, so maybe psi is also untrusted or linked to GPS heading?
            # Assuming psi is untrusted too for consistency with C-matrix logic
            R_effective[idx_PSI, idx_PSI] = high_variance
        
        # Also handle partial measurements (e.g. initialisation)
        # This logic overrides the 'process_measurement' padding if we trust R scaling
        
        # if len(measurement) == 2:
        #     # Minimal fallback
        #     # indices for vx, r
        #      pass 
             
        # Process measurement to get full vector
        y = self._process_measurement(measurement, acceleration)
        
        # Control input
        u = control_input.reshape(-1)
        delta = u[0]
        
        # =====================================================
        # EKF PREDICT
        # =====================================================
        xa_pred, P_pred = self.ekf_predict(self.state_augmented, self.P, u)
        
        # =====================================================
        # EKF UPDATE
        # =====================================================
        xa_upd, P_upd, innov, y_pred = self.ekf_update(xa_pred, P_pred, y, u, R_matrix=R_effective)
        
        # Store updated state and covariance
        self.state_augmented = xa_upd
        self.P = P_upd
        
        # Wrap heading state to [-pi, pi]
        # Ensures estimated yaw stays in the same range as GPS sensors
        # self.IDX_PSI = 2 same for 6d and 8d
        self.state_augmented[self.IDX_PSI] = (self.state_augmented[self.IDX_PSI] + np.pi) % (2 * np.pi) - np.pi
        
        # Store Kalman gain for diagnostics - recompute using R_effective
        # Store Kalman gain for diagnostics - recompute using R_effective and analytical matrices
        try:
            rho = self.compute_scheduling_params(xa_pred[:self.state_dim], delta)
            _, _, C_a = self.compute_augmented_matrices(rho)
            H_curr = C_a
            
            S_curr = H_curr @ P_pred @ H_curr.T + R_effective
            self.K = P_pred @ H_curr.T @ np.linalg.pinv(S_curr)
        except Exception:
            self.K = np.zeros((self.augmented_dim, self.meas_dim))
        
        # Store innovation for diagnostics
        self.innovation = innov
        
        # Extract state and residual estimates
        self.state_hat = self.state_augmented[:self.state_dim].copy()
        self.w_hat = self.state_augmented[self.state_dim:].copy()
        
        # Update UIO-style residual constraint
        rho = self.compute_scheduling_params(self.state_hat, delta)
        self.ay_innovation = self.compute_ay_innovation(y, self.state_hat, u, rho)
        self.w_constraint = self.m * self.ay_innovation  # ≈ w_r + cos(δ)·w_f
        
        # Copy tire residuals to base class attribute for interface compatibility
        self.f_uk_hat = self.w_hat.copy()
        
        # Optional gyro bias update
        if self.include_gyro_bias:
            if self.use_8d_system:
                 idx_r_meas = self.MEAS8_IDX_R
                 idx_r_state = self.IDX8_R
            else:
                 idx_r_meas = self.MEAS_IDX_R
                 idx_r_state = self.IDX_R
                 
            r_error = y[idx_r_meas] - self.state_hat[idx_r_state]
            self.gyro_bias += 0.001 * r_error
        
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
        """Get current 6D state estimate [v_x, v_y, ψ, r, X, Y]"""
        return self.state_hat.copy()
    
    def get_tire_residuals(self) -> np.ndarray:
        """Get current tire residual estimates [w_r, w_f]"""
        return self.w_hat.copy()
    
    def get_augmented_state(self) -> np.ndarray:
        """Get full augmented state [x; w]"""
        return self.state_augmented.copy()
    
    def get_unknown_input(self) -> np.ndarray:
        """Get unknown input estimate (alias for tire residuals)"""
        return self.w_hat.copy()
    
    def get_ay_constraint(self) -> float:
        """
        Get the a_y-based constraint on tire residuals
        
        Returns m·ã_y ≈ w_r + cos(δ)·w_f
        """
        return self.w_constraint
    
    def check_uio_rank_condition(self, delta: float) -> bool:
        """
        Check UIO existence condition (rank condition)
        
        rank([C·E; F]) = rank(E) = 2
        
        This should be satisfied for normal driving conditions.
        
        Args:
            delta: Current steering angle
            
        Returns:
            True if rank condition is satisfied
        """
        rho = self.compute_scheduling_params(self.state_hat, delta)
        
        C = self.compute_C_matrix(rho)
        E = self.compute_E_matrix(rho)
        F = self.compute_F_matrix(rho)
        
        # Stack CE and F
        CE = C @ E
        stacked = np.vstack([CE, F])
        
        # Compute ranks
        rank_stacked = np.linalg.matrix_rank(stacked)
        rank_E = np.linalg.matrix_rank(E)
        
        return rank_stacked == rank_E == 2
    
    def reset(self, initial_state: Optional[np.ndarray] = None, 
              initial_position: Optional[np.ndarray] = None):
        """
        Reset observer state
        
        Args:
            initial_state: Initial state [v_x, v_y, ψ, r, X, Y] or [v_x, v_y, ψ, r]
            initial_position: Initial position [X, Y] if not in initial_state
        """
        if initial_state is not None:
            initial_state = initial_state.reshape(-1)
            if len(initial_state) >= self.state_dim:
                self.state_hat = initial_state[:self.state_dim].copy()
            else:
                self.state_hat = np.zeros(self.state_dim)
                self.state_hat[:len(initial_state)] = initial_state
                if initial_position is not None:
                    if self.use_8d_system:
                         self.state_hat[self.IDX8_X] = initial_position[0]
                         self.state_hat[self.IDX8_Y] = initial_position[1]
                    else:
                         self.state_hat[self.IDX_X] = initial_position[0]
                         self.state_hat[self.IDX_Y] = initial_position[1]
        else:
            self.state_hat = np.zeros(self.state_dim)
            if initial_position is not None:
                if self.use_8d_system:
                     self.state_hat[self.IDX8_X] = initial_position[0]
                     self.state_hat[self.IDX8_Y] = initial_position[1]
                else:
                     self.state_hat[self.IDX_X] = initial_position[0]
                     self.state_hat[self.IDX_Y] = initial_position[1]
        
        # Reset augmented state
        self.state_augmented = np.zeros(self.augmented_dim)
        self.state_augmented[:self.state_dim] = self.state_hat
        
        # Reset tire residuals
        self.w_hat = np.zeros(2)
        self.f_uk_hat = np.zeros(2)
        
        # Reset EKF covariance to initial
        self.P = self._default_P0()
        
        # Reset Kalman gain
        self.K = np.zeros((self.augmented_dim, self.meas_dim))
        
        # Reset innovation
        self.innovation = np.zeros(self.meas_dim)
        
        # Reset gyro bias
        self.gyro_bias = 0.0
        
        # Reset innovations
        self.ay_innovation = 0.0
        self.w_constraint = 0.0
    
    # =========================================================
    # EKF Diagnostic Getters
    # =========================================================
    
    def get_covariance(self) -> np.ndarray:
        """Get current error covariance matrix P (8×8)"""
        return self.P.copy()
    
    def get_kalman_gain(self) -> np.ndarray:
        """Get current Kalman gain matrix K (8×6)"""
        return self.K.copy()
    
    def get_innovation(self) -> np.ndarray:
        """Get last innovation (measurement residual) vector"""
        return self.innovation.copy()
    
    def get_state_uncertainty(self) -> np.ndarray:
        """
        Get state estimation uncertainty (standard deviations)
        
        Returns diagonal of sqrt(P) for state portion
        """
        return np.sqrt(np.diag(self.P)[:self.STATE_DIM])
    
    def get_residual_uncertainty(self) -> np.ndarray:
        """
        Get tire residual estimation uncertainty (standard deviations)
        
        Returns diagonal of sqrt(P) for residual portion
        """
        return np.sqrt(np.diag(self.P)[self.STATE_DIM:])
    
    # =========================================================
    # Tuning Methods
    # =========================================================
    
    def set_Q(self, Q: np.ndarray):
        """
        Set process noise covariance Q
        
        Args:
            Q: Process noise covariance (8×8)
        """
        if Q.shape != (self.AUGMENTED_DIM, self.AUGMENTED_DIM):
            raise ValueError(f"Q must be {self.AUGMENTED_DIM}×{self.AUGMENTED_DIM}")
        self.Q = Q.copy()
    
    def set_R(self, R: np.ndarray):
        """
        Set measurement noise covariance R
        
        Args:
            R: Measurement noise covariance (6×6)
        """
        if R.shape != (self.MEAS_DIM, self.MEAS_DIM):
            raise ValueError(f"R must be {self.MEAS_DIM}×{self.MEAS_DIM}")
        self.R = R.copy()
    
    def set_P(self, P: np.ndarray):
        """
        Set error covariance P (for re-initialization)
        
        Args:
            P: Error covariance (8×8)
        """
        if P.shape != (self.AUGMENTED_DIM, self.AUGMENTED_DIM):
            raise ValueError(f"P must be {self.AUGMENTED_DIM}×{self.AUGMENTED_DIM}")
        self.P = P.copy()


def create_qlpv_observer(sample_time: float = 0.02,
                         vehicle_params: Optional[Dict] = None,
                         **kwargs) -> qLPVKalmanObserver:
    """
    Factory function to create qLPV augmented-state observer
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dictionary
        **kwargs: Additional arguments passed to constructor
        
    Returns:
        Configured qLPVAugmentedObserver instance
    """
    return qLPVKalmanObserver(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        **kwargs
    )
