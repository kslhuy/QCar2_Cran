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
)


# Note: SchedulingParameters is imported from qlpv_vehicle_dynamics_obs


class qLPVAugmentedObserver(FirstLayerObserverBase):
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
    
    def __init__(self, sample_time: float = 0.02, 
                 vehicle_params: Optional[Dict] = None,
                 Q: Optional[np.ndarray] = None,
                 R: Optional[np.ndarray] = None,
                 P0: Optional[np.ndarray] = None,
                 include_gyro_bias: bool = False,
                 **kwargs):
        """
        Initialize qLPV Augmented-State Observer with EKF-style Gain Computation
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict with keys:
                - 'lf': Distance from CG to front axle [m]
                - 'lr': Distance from CG to rear axle [m]
                - 'm': Vehicle mass [kg]
                - 'Iz': Yaw moment of inertia [kg·m²]
                - 'Cf': Front cornering stiffness [N/rad]
                - 'Cr': Rear cornering stiffness [N/rad]
                - 'mu': Road friction coefficient
            Q: Process noise covariance (8×8 for augmented state)
            R: Measurement noise covariance (6×6 for measurements)
            P0: Initial error covariance (8×8 for augmented state)
            include_gyro_bias: Whether to include gyro bias state (for long runs)
        """
        # Initialize base class with 6D state
        super().__init__(
            state_dim=self.STATE_DIM,
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
        
        # Augmented state: [x; w_r; w_f] = [vx, vy, psi, r, X, Y, wr, wf]
        self.state_augmented = np.zeros(self.AUGMENTED_DIM)
        
        # Initialize state estimate
        self.state_hat = np.zeros(self.STATE_DIM)
        
        # Tire residual estimates
        self.w_hat = np.zeros(2)  # [w_r, w_f]
        
        # =====================================================
        # EKF Covariance Matrices (Q, R, P)
        # =====================================================
        
        # Process noise covariance Q (8×8)
        # Small for kinematics, moderate for vy/r, large for w (random-walk)
        if Q is not None:
            self.Q = Q
        else:
            self.Q = self._default_Q() * sample_time
        
        # Measurement noise covariance R (6×6)
        # y = [vx, r, psi, X, Y, ay]
        if R is not None:
            self.R = R
        else:
            self.R = self._default_R()
        
        # Error covariance matrix P (8×8)
        if P0 is not None:
            self.P = P0.copy()
        else:
            self.P = self._default_P0()
        
        # Kalman gain K (8×6) - computed dynamically
        self.K = np.zeros((self.AUGMENTED_DIM, self.MEAS_DIM))
        
        # Store last innovation for diagnostics
        self.innovation = np.zeros(self.MEAS_DIM)
        
        # Gyro bias estimation (optional)
        self.include_gyro_bias = include_gyro_bias
        self.gyro_bias = 0.0
        
        # UIO residual for a_y constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0  # m·ã_y ≈ w_r + cos(δ)·w_f
        
        # Minimum velocity threshold
        self.min_vx = 0.5  # [m/s]
        
        # Numerical Jacobian step size
        self.epsilon = 1e-6
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters - uses centralized defaults"""
        return get_default_vehicle_params()
    
    def _default_Q(self) -> np.ndarray:
        """
        Default process noise covariance Q (8×8)
        
        xa = [vx, vy, psi, r, X, Y, wr, wf]
        
        - Small for kinematics (vx, psi, X, Y)
        - Moderate for dynamics (vy, r)
        - Large for tire residuals w (random-walk assumption)
        """
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
        Default measurement noise covariance R (6×6)
        
        y = [vx, r, psi, X, Y, ay]
        
        Tune based on actual sensor characteristics.
        """
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
        Default initial error covariance P0 (8×8)
        
        Reflects initial uncertainty in state estimate.
        """
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
        return SchedulingParameters.from_state_and_input(state, delta, self.min_vx)
    
    def compute_slip_angles(self, state: np.ndarray, delta: float) -> Tuple[float, float]:
        """
        Compute front and rear slip angles
        
        α_f = δ - v_y/v_x - l_f·r/v_x
        α_r = -v_y/v_x + l_r·r/v_x
        
        Args:
            state: State vector [v_x, v_y, ψ, r, X, Y]
            delta: Steering angle
            
        Returns:
            Tuple of (alpha_f, alpha_r)
        """
        vx = max(abs(state[self.IDX_VX]), self.min_vx)
        vy = state[self.IDX_VY]
        r = state[self.IDX_R]
        
        alpha_f = delta - vy / vx - self.lf * r / vx
        alpha_r = -vy / vx + self.lr * r / vx
        
        return alpha_f, alpha_r
    
    def compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute state matrix A(ρ) for qLPV system
        
        State: [v_x, v_y, ψ, r, X, Y]
        
        Uses linearized single-track model with linear tire assumption.
        """
        A = np.zeros((self.STATE_DIM, self.STATE_DIM))
        
        # Shortcuts
        inv_vx = rho.inv_vx
        cos_d = rho.cos_delta
        sin_d = rho.sin_delta
        vx = rho.vx
        vy = rho.vy
        cos_psi = rho.cos_psi
        sin_psi = rho.sin_psi
        
        # v_x dynamics: v̇_x = -μg + ... (friction term as constant, not in A)
        A[0, 0] = -self.mu * self.g / vx  # Longitudinal friction approx
        A[0, 1] = self.Cf * sin_d / (self.m * vx)
        A[0, 3] = self.Cf * self.lf * sin_d / (self.m * vx) + vy
        
        # v_y dynamics: v̇_y = F_yr/m + F_yf·cos(δ)/m - r·v_x
        A[1, 1] = -(self.Cr + self.Cf * cos_d) / (self.m * vx)
        A[1, 3] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.m * vx) - vx
        
        # ψ dynamics: ψ̇ = r
        A[2, 3] = 1.0
        
        # r dynamics: ṙ = (l_f·F_yf·cos(δ) - l_r·F_yr) / I_z
        A[3, 1] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.Iz * vx)
        A[3, 3] = -(self.Cf * self.lf**2 * cos_d + self.Cr * self.lr**2) / (self.Iz * vx)
        
        # X dynamics: Ẋ = v_x·cos(ψ) - v_y·sin(ψ)
        A[4, 0] = cos_psi
        A[4, 1] = -sin_psi
        A[4, 2] = -vx * sin_psi - vy * cos_psi  # ∂/∂ψ
        
        # Y dynamics: Ẏ = v_x·sin(ψ) + v_y·cos(ψ)
        A[5, 0] = sin_psi
        A[5, 1] = cos_psi
        A[5, 2] = vx * cos_psi - vy * sin_psi  # ∂/∂ψ
        
        return A
    
    def compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute input matrix B(ρ)
        
        Input: u = [δ, a]ᵀ (steering, acceleration)
        """
        B = np.zeros((self.STATE_DIM, 2))
        
        cos_d = rho.cos_delta
        sin_d = rho.sin_delta
        
        # v_x: affected by acceleration
        B[0, 0] = -self.Cf * sin_d / self.m  # ∂/∂δ (steering effect)
        B[0, 1] = 1.0  # ∂/∂a (direct acceleration)
        
        # v_y: affected by steering (front tire force direction change)
        B[1, 0] = self.Cf * cos_d / self.m
        
        # r: affected by steering through front tire moment
        B[3, 0] = self.Cf * self.lf * cos_d / self.Iz
        
        return B
    
    def compute_E_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute residual injection matrix E(ρ)
        
        Residual: w = [w_r, w_f]ᵀ (rear and front tire force residuals)
        
        E(ρ) = [[0,        -sin(δ)/m     ],
                [1/m,       cos(δ)/m     ],
                [0,         0            ],
                [-l_r/I_z,  l_f·cos(δ)/I_z],
                [0,         0            ],
                [0,         0            ]]
        """
        E = np.zeros((self.STATE_DIM, 2))
        
        cos_d = rho.cos_delta
        sin_d = rho.sin_delta
        
        # v_x: front tire residual affects through sin(δ)
        E[0, 1] = -sin_d / self.m
        
        # v_y: both residuals contribute
        E[1, 0] = 1.0 / self.m  # w_r
        E[1, 1] = cos_d / self.m  # w_f·cos(δ)
        
        # r: both residuals create moment
        E[3, 0] = -self.lr / self.Iz  # -l_r·w_r / I_z
        E[3, 1] = self.lf * cos_d / self.Iz  # l_f·w_f·cos(δ) / I_z
        
        return E
    
    def compute_C_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute output matrix C(ρ)
        
        Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ
        
        First 5 rows are simple state selections.
        Row 6 (a_y) is computed from lateral dynamics:
            a_y = v̇_y + r·v_x = F_yr/m + F_yf·cos(δ)/m
        
        Using F_y = C·α + w:
            a_y = (C_r·α_r + w_r)/m + (C_f·α_f + w_f)·cos(δ)/m
        
        Linear part C_ay involves state-dependent slip angles.
        """
        C = np.zeros((self.MEAS_DIM, self.STATE_DIM))
        
        # Direct measurements
        C[self.MEAS_IDX_VX, self.IDX_VX] = 1.0  # v_x
        C[self.MEAS_IDX_R, self.IDX_R] = 1.0    # r
        C[self.MEAS_IDX_PSI, self.IDX_PSI] = 1.0  # ψ
        C[self.MEAS_IDX_X, self.IDX_X] = 1.0    # X
        C[self.MEAS_IDX_Y, self.IDX_Y] = 1.0    # Y
        
        # a_y measurement: C_ay(ρ)·x
        # C_ay = [0, -(C_r + C_f·cos(δ))/(m·v_x), 0, (C_r·l_r - C_f·l_f·cos(δ))/(m·v_x), 0, 0]
        inv_vx = rho.inv_vx
        cos_d = rho.cos_delta
        
        C[self.MEAS_IDX_AY, self.IDX_VY] = -(self.Cr + self.Cf * cos_d) / (self.m * rho.vx)
        C[self.MEAS_IDX_AY, self.IDX_R] = (self.Cr * self.lr - self.Cf * self.lf * cos_d) / (self.m * rho.vx)
        
        return C
    
    def compute_D_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute feedthrough matrix D(ρ) from input to output
        
        Only a_y has feedthrough from steering:
            D_ay = [C_f·cos(δ)/m, 0]
        """
        D = np.zeros((self.MEAS_DIM, 2))
        
        cos_d = rho.cos_delta
        
        # Only a_y row has feedthrough
        D[self.MEAS_IDX_AY, 0] = self.Cf * cos_d / self.m  # Steering effect
        
        return D
    
    def compute_F_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute residual-to-output matrix F(ρ)
        
        Only a_y is affected by tire residuals:
            F_ay = [1/m, cos(δ)/m]
        
        This is KEY for UIO-style residual estimation!
        """
        F = np.zeros((self.MEAS_DIM, 2))
        
        cos_d = rho.cos_delta
        
        # a_y row: a_y = ... + w_r/m + w_f·cos(δ)/m
        F[self.MEAS_IDX_AY, 0] = 1.0 / self.m  # w_r contribution
        F[self.MEAS_IDX_AY, 1] = cos_d / self.m  # w_f contribution
        
        return F
    
    def compute_augmented_matrices(self, rho: SchedulingParameters) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute augmented system matrices for x_a = [x; w]
        
        Augmented dynamics (assuming ẇ ≈ 0):
            A_a = [[A(ρ), E(ρ)], [0, 0]]
            B_a = [[B(ρ)], [0]]
            C_a = [C(ρ), F(ρ)]
        
        Returns:
            Tuple of (A_a, B_a, C_a)
        """
        A = self.compute_A_matrix(rho)
        B = self.compute_B_matrix(rho)
        E = self.compute_E_matrix(rho)
        C = self.compute_C_matrix(rho)
        F = self.compute_F_matrix(rho)
        
        # Augmented state matrix (8×8)
        A_a = np.zeros((self.AUGMENTED_DIM, self.AUGMENTED_DIM))
        A_a[:self.STATE_DIM, :self.STATE_DIM] = A
        A_a[:self.STATE_DIM, self.STATE_DIM:] = E
        # Lower right block is zeros (ẇ = 0 assumption)
        
        # Augmented input matrix (8×2)
        B_a = np.zeros((self.AUGMENTED_DIM, 2))
        B_a[:self.STATE_DIM, :] = B
        
        # Augmented output matrix (6×8)
        C_a = np.zeros((self.MEAS_DIM, self.AUGMENTED_DIM))
        C_a[:, :self.STATE_DIM] = C
        C_a[:, self.STATE_DIM:] = F
        
        return A_a, B_a, C_a
    
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
        Discrete-time state transition function
        
        xa[k+1] = f(xa[k], u[k])
        
        Uses Euler discretization of qLPV dynamics.
        
        Args:
            xa: Augmented state [vx, vy, psi, r, X, Y, wr, wf]
            u: Control input [δ, a]
            
        Returns:
            Predicted augmented state at next time step
        """
        # Extract state components
        vx = max(abs(xa[self.IDX_VX]), self.min_vx)
        vy = xa[self.IDX_VY]
        psi = xa[self.IDX_PSI]
        r = xa[self.IDX_R]
        X = xa[self.IDX_X]
        Y = xa[self.IDX_Y]
        wr = xa[self.IDX_WR]
        wf = xa[self.IDX_WF]
        
        delta = u[0]
        accel = u[1] if len(u) > 1 else 0.0
        
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        cos_delta = np.cos(delta)
        sin_delta = np.sin(delta)
        
        # Compute tire slip angles
        alpha_f = delta - vy / vx - self.lf * r / vx
        alpha_r = -vy / vx + self.lr * r / vx
        
        # Tire forces (including residuals)
        Fyf = self.Cf * alpha_f + wf  # Front lateral force
        Fyr = self.Cr * alpha_r + wr  # Rear lateral force
        
        # State derivatives (continuous-time)
        vx_dot = accel - self.mu * self.g + r * vy - Fyf * sin_delta / self.m
        vy_dot = (Fyr + Fyf * cos_delta) / self.m - r * vx
        psi_dot = r
        r_dot = (self.lf * Fyf * cos_delta - self.lr * Fyr) / self.Iz
        X_dot = vx * cos_psi - vy * sin_psi
        Y_dot = vx * sin_psi + vy * cos_psi
        
        # Tire residual dynamics (random-walk: ẇ = 0)
        wr_dot = 0.0
        wf_dot = 0.0
        
        # Euler discretization
        xa_next = np.array([
            xa[self.IDX_VX] + self.Ts * vx_dot,
            xa[self.IDX_VY] + self.Ts * vy_dot,
            xa[self.IDX_PSI] + self.Ts * psi_dot,
            xa[self.IDX_R] + self.Ts * r_dot,
            xa[self.IDX_X] + self.Ts * X_dot,
            xa[self.IDX_Y] + self.Ts * Y_dot,
            xa[self.IDX_WR] + self.Ts * wr_dot,
            xa[self.IDX_WF] + self.Ts * wf_dot,
        ])
        
        return xa_next
    
    def h_meas(self, xa: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Measurement function
        
        y = h(xa, u) = [vx, r, psi, X, Y, ay]
        
        Args:
            xa: Augmented state [vx, vy, psi, r, X, Y, wr, wf]
            u: Control input [δ, a]
            
        Returns:
            Predicted measurement vector
        """
        vx = max(abs(xa[self.IDX_VX]), self.min_vx)
        vy = xa[self.IDX_VY]
        psi = xa[self.IDX_PSI]
        r = xa[self.IDX_R]
        X = xa[self.IDX_X]
        Y = xa[self.IDX_Y]
        wr = xa[self.IDX_WR]
        wf = xa[self.IDX_WF]
        
        delta = u[0]
        cos_delta = np.cos(delta)
        
        # Compute tire slip angles
        alpha_f = delta - vy / vx - self.lf * r / vx
        alpha_r = -vy / vx + self.lr * r / vx
        
        # Tire forces (including residuals)
        Fyf = self.Cf * alpha_f + wf
        Fyr = self.Cr * alpha_r + wr
        
        # Lateral acceleration: a_y = (Fyr + Fyf·cos(δ)) / m
        ay = (Fyr + Fyf * cos_delta) / self.m
        
        # Measurement vector
        y = np.array([
            xa[self.IDX_VX],  # vx
            r,                 # r
            psi,               # psi
            X,                 # X
            Y,                 # Y
            ay,                # ay
        ])
        
        return y
    
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
        # State prediction using discrete dynamics
        xa_pred = self.f_discrete(xa, u)
        
        # Compute Jacobian F = df/dx (numerical)
        F = self._numerical_jacobian(lambda z: self.f_discrete(z, u), xa)
        
        # Covariance prediction
        P_pred = F @ P @ F.T + self.Q
        
        return xa_pred, P_pred
    
    def ekf_update(self, xa_pred: np.ndarray, P_pred: np.ndarray,
                   y_meas: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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
            
        Returns:
            Tuple of (updated state, updated covariance, innovation, predicted measurement)
        """
        # Measurement prediction
        y_pred = self.h_meas(xa_pred, u)
        
        # Compute Jacobian H = dh/dx (numerical)
        H = self._numerical_jacobian(lambda z: self.h_meas(z, u), xa_pred)
        
        # Innovation (measurement residual)
        innov = y_meas - y_pred
        
        # Innovation covariance S = H·P⁻·Hᵀ + R
        S = H @ P_pred @ H.T + self.R
        
        # Kalman gain K = P⁻·Hᵀ·S⁻¹
        try:
            K = P_pred @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Fallback: use pseudo-inverse
            K = P_pred @ H.T @ np.linalg.pinv(S)
        
        # State update
        xa_upd = xa_pred + K @ innov
        
        # Covariance update (Joseph form for numerical stability)
        I = np.eye(P_pred.shape[0])
        IKH = I - K @ H
        P_upd = IKH @ P_pred @ IKH.T + K @ self.R @ K.T
        
        return xa_upd, P_upd, innov, y_pred
    
    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update qLPV augmented-state observer with EKF predict-update cycle
        
        EKF Observer:
            1. Predict: x̂⁻ = f(x̂, u), P⁻ = F·P·Fᵀ + Q
            2. Update:  K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
                        x̂ = x̂⁻ + K·(y - h(x̂⁻, u))
                        P = (I - K·H)·P⁻
        
        Args:
            measurement: Measurement vector. Can be:
                - [v_x, r, ψ, X, Y, a_y] (6D full)
                - [v_x, r, ψ, X, Y] (5D, a_y computed from acceleration)
                - [v_x, r] or [v_x, ψ] (2D minimal, other states predicted)
            control_input: Control [δ, a] (steering, acceleration)
            f_nn: Neural network output (not used in this observer, for interface compatibility)
            acceleration: Full 3D acceleration [a_x, a_y, a_z] if a_y not in measurement
            
        Returns:
            Tuple of (state_estimate, tire_residual_estimate)
        """
        # Process measurement to get full 6D vector
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
        xa_upd, P_upd, innov, y_pred = self.ekf_update(xa_pred, P_pred, y, u)
        
        # Store updated state and covariance
        self.state_augmented = xa_upd
        self.P = P_upd
        self.K = P_pred @ self._numerical_jacobian(
            lambda z: self.h_meas(z, u), xa_pred
        ).T @ np.linalg.pinv(
            self._numerical_jacobian(lambda z: self.h_meas(z, u), xa_pred) @ P_pred @ 
            self._numerical_jacobian(lambda z: self.h_meas(z, u), xa_pred).T + self.R
        )  # Store Kalman gain for diagnostics
        
        # Store innovation for diagnostics
        self.innovation = innov
        
        # Extract state and residual estimates
        self.state_hat = self.state_augmented[:self.STATE_DIM].copy()
        self.w_hat = self.state_augmented[self.STATE_DIM:].copy()
        
        # Update UIO-style residual constraint
        rho = self.compute_scheduling_params(self.state_hat, delta)
        self.ay_innovation = self.compute_ay_innovation(y, self.state_hat, u, rho)
        self.w_constraint = self.m * self.ay_innovation  # ≈ w_r + cos(δ)·w_f
        
        # Copy tire residuals to base class attribute for interface compatibility
        self.f_uk_hat = self.w_hat.copy()
        
        # Optional gyro bias update
        if self.include_gyro_bias:
            # Simple bias estimation from yaw rate residual
            r_error = y[self.MEAS_IDX_R] - self.state_hat[self.IDX_R]
            self.gyro_bias += 0.001 * r_error  # Slow adaptation
        
        return self.state_hat.copy(), self.w_hat.copy()
    
    def _process_measurement(self, measurement: np.ndarray, 
                             acceleration: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Process input measurement to get full 6D measurement vector
        
        Args:
            measurement: Input measurement (various formats)
            acceleration: Optional 3D acceleration [a_x, a_y, a_z]
            
        Returns:
            Full 6D measurement [v_x, r, ψ, X, Y, a_y]
        """
        measurement = measurement.reshape(-1)
        y = np.zeros(self.MEAS_DIM)
        
        if len(measurement) >= 6:
            # Full measurement provided
            y = measurement[:6].copy()
        elif len(measurement) == 5:
            # [v_x, r, ψ, X, Y] - need a_y from acceleration
            y[:5] = measurement[:5]
            if acceleration is not None:
                y[5] = acceleration[1]  # Extract a_y from [a_x, a_y, a_z]
            else:
                # Estimate a_y from dynamics (v̇_y + r·v_x)
                # Use current state estimate as approximation
                y[5] = self.state_hat[self.IDX_R] * self.state_hat[self.IDX_VX]
        elif len(measurement) == 2:
            # Minimal: [v_x, r] or [v_x, ψ]
            y[self.MEAS_IDX_VX] = measurement[0]
            y[self.MEAS_IDX_R] = measurement[1]  # Assume second is r
            # Fill others from state prediction
            y[self.MEAS_IDX_PSI] = self.state_hat[self.IDX_PSI]
            y[self.MEAS_IDX_X] = self.state_hat[self.IDX_X]
            y[self.MEAS_IDX_Y] = self.state_hat[self.IDX_Y]
            if acceleration is not None:
                y[self.MEAS_IDX_AY] = acceleration[1]
        else:
            raise ValueError(f"Unsupported measurement dimension: {len(measurement)}")
        
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
            if len(initial_state) >= self.STATE_DIM:
                self.state_hat = initial_state[:self.STATE_DIM].copy()
            else:
                self.state_hat[:len(initial_state)] = initial_state
                if initial_position is not None:
                    self.state_hat[self.IDX_X] = initial_position[0]
                    self.state_hat[self.IDX_Y] = initial_position[1]
        else:
            self.state_hat = np.zeros(self.STATE_DIM)
            if initial_position is not None:
                self.state_hat[self.IDX_X] = initial_position[0]
                self.state_hat[self.IDX_Y] = initial_position[1]
        
        # Reset augmented state
        self.state_augmented = np.zeros(self.AUGMENTED_DIM)
        self.state_augmented[:self.STATE_DIM] = self.state_hat
        
        # Reset tire residuals
        self.w_hat = np.zeros(2)
        self.f_uk_hat = np.zeros(2)
        
        # Reset EKF covariance to initial
        self.P = self._default_P0()
        
        # Reset Kalman gain
        self.K = np.zeros((self.AUGMENTED_DIM, self.MEAS_DIM))
        
        # Reset innovation
        self.innovation = np.zeros(self.MEAS_DIM)
        
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
                         **kwargs) -> qLPVAugmentedObserver:
    """
    Factory function to create qLPV augmented-state observer
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dictionary
        **kwargs: Additional arguments passed to constructor
        
    Returns:
        Configured qLPVAugmentedObserver instance
    """
    return qLPVAugmentedObserver(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        **kwargs
    )
