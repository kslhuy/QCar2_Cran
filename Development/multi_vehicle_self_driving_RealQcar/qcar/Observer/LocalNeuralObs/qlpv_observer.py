"""
qLPV Augmented-State Observer for First-Layer Observer Architecture

Implements a quasi-Linear Parameter-Varying (qLPV) augmented-state observer
with tire-residual estimation for vehicle state and disturbance estimation.

State: x = [v_x, v_y, ψ, r, X, Y]ᵀ (6D)
Augmented state: x_a = [x; w_r; w_f]ᵀ (8D with tire residuals)
Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ (6D with lateral acceleration)

Observer Equation:
    ẋ̂_a = A_a(ρ̂)·x̂_a + B_a(ρ̂)·u + L_a(ρ̂)·(y − C_a(ρ̂)·x̂_a − D(ρ̂)·u)

where:
    - x̂_a = [x̂; ŵ_r; ŵ_f] is the augmented state estimate
    - ρ = {1/v_x, sin(δ), cos(δ), v_x, v_y, sin(ψ), cos(ψ)} scheduling parameters
    - w = [w_r, w_f] are tire force residuals (unknown inputs)
    - a_y provides algebraic constraint on w

References:
    - qLPV vehicle dynamics with tire-residual estimation
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
    from uio_observers import FirstLayerObserverBase
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


@dataclass
class SchedulingParameters:
    """Scheduling parameters ρ for qLPV system"""
    inv_vx: float      # 1/v_x
    sin_delta: float   # sin(δ)
    cos_delta: float   # cos(δ)
    vx: float          # v_x
    vy: float          # v_y
    sin_psi: float     # sin(ψ)
    cos_psi: float     # cos(ψ)

    @classmethod
    def from_state_and_input(cls, state: np.ndarray, delta: float, 
                              min_vx: float = 0.5) -> 'SchedulingParameters':
        """
        Compute scheduling parameters from state and steering input
        
        Args:
            state: State vector [v_x, v_y, ψ, r, X, Y]
            delta: Steering angle
            min_vx: Minimum velocity to avoid singularity
        """
        vx = max(abs(state[0]), min_vx)  # Avoid division by zero
        vy = state[1]
        psi = state[2]
        
        return cls(
            inv_vx=1.0 / vx,
            sin_delta=np.sin(delta),
            cos_delta=np.cos(delta),
            vx=vx,
            vy=vy,
            sin_psi=np.sin(psi),
            cos_psi=np.cos(psi)
        )


class qLPVAugmentedObserver(FirstLayerObserverBase):
    """
    qLPV Augmented-State Observer with Tire-Residual Estimation
    
    Estimates both vehicle states and unknown tire force residuals using
    an augmented-state qLPV observer structure.
    
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
                 observer_gains: Optional[Dict] = None,
                 include_gyro_bias: bool = False,
                 **kwargs):
        """
        Initialize qLPV Augmented-State Observer
        
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
            observer_gains: Observer gain parameters
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
        self.mu = self.params.get('mu', 1.0)
        self.g = 9.81  # Gravity
        
        # Augmented state: [x; w_r; w_f]
        self.state_augmented = np.zeros(self.AUGMENTED_DIM)
        
        # Initialize state estimate
        self.state_hat = np.zeros(self.STATE_DIM)
        
        # Tire residual estimates
        self.w_hat = np.zeros(2)  # [w_r, w_f]
        
        # Observer gains
        self.observer_gains = observer_gains or self._default_gains()
        self._initialize_observer_gains()
        
        # Gyro bias estimation (optional)
        self.include_gyro_bias = include_gyro_bias
        self.gyro_bias = 0.0
        
        # UIO residual for a_y constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0  # m·ã_y ≈ w_r + cos(δ)·w_f
        
        # Minimum velocity threshold
        self.min_vx = 0.5  # [m/s]
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters (typical passenger car scale)"""
        return {
            'lf': 0.11,      # Distance CG to front axle [m] (QCar scale)
            'lr': 0.11,      # Distance CG to rear axle [m]
            'm': 3.5,        # Mass [kg] (QCar scale)
            'Iz': 0.05,      # Yaw inertia [kg·m²]
            'Cf': 50.0,      # Front cornering stiffness [N/rad]
            'Cr': 50.0,      # Rear cornering stiffness [N/rad]
            'mu': 1.0,       # Road friction coefficient
        }
    
    def _default_gains(self) -> Dict:
        """Default observer gains"""
        return {
            'L_state': np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5]),  # State observer gains
            'L_residual': np.array([[0.5], [0.5]]),  # Residual observer gains
            'alpha_w': 0.1,  # Residual dynamics time constant
        }
    
    def _initialize_observer_gains(self):
        """Initialize observer gain matrices"""
        gains = self.observer_gains
        
        # State observer gain (6×6 for state estimation from 6 measurements)
        if isinstance(gains.get('L_state'), np.ndarray) and gains['L_state'].shape == (self.STATE_DIM, self.MEAS_DIM):
            self.L_state = gains['L_state']
        else:
            # Default: diagonal gain matching measurement to state
            self.L_state = np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5])
        
        # Residual observer gain (2×6 for tire residual estimation from 6 measurements)
        if isinstance(gains.get('L_residual'), np.ndarray) and gains['L_residual'].shape == (2, self.MEAS_DIM):
            self.L_residual = gains['L_residual']
        else:
            # Default: mainly driven by a_y measurement (index 5)
            self.L_residual = np.zeros((2, self.MEAS_DIM))
            self.L_residual[0, self.MEAS_IDX_AY] = 0.5  # w_r from a_y
            self.L_residual[1, self.MEAS_IDX_AY] = 0.5  # w_f from a_y
        
        # Augmented observer gain (8×6) = stack of L_state (6×6) and L_residual (2×6)
        self.L_augmented = np.vstack([self.L_state, self.L_residual])
    
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
    
    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update qLPV augmented-state observer with new measurement
        
        Observer equation (discrete):
            x̂_a[k+1] = x̂_a[k] + Ts · (A_a(ρ̂)·x̂_a + B_a(ρ̂)·u + L_a·(y - C_a(ρ̂)·x̂_a - D(ρ̂)·u))
        
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
        accel = u[1] if len(u) > 1 else 0.0
        
        # Compute scheduling parameters from current state estimate
        rho = self.compute_scheduling_params(self.state_hat, delta)
        
        # Get augmented system matrices
        A_a, B_a, C_a = self.compute_augmented_matrices(rho)
        D = self.compute_D_matrix(rho)
        
        # Predicted output
        y_pred = C_a @ self.state_augmented + D @ u
        
        # Innovation (measurement residual)
        innovation = y - y_pred
        
        # Augmented state dynamics (continuous-time)
        x_a_dot = A_a @ self.state_augmented + B_a @ u + self.L_augmented @ innovation
        
        # Discrete update (Euler integration)
        self.state_augmented = self.state_augmented + self.Ts * x_a_dot
        
        # Extract state and residual estimates
        self.state_hat = self.state_augmented[:self.STATE_DIM].copy()
        self.w_hat = self.state_augmented[self.STATE_DIM:].copy()
        
        # Update UIO-style residual constraint
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
        
        # Reset gyro bias
        self.gyro_bias = 0.0
        
        # Reset innovations
        self.ay_innovation = 0.0
        self.w_constraint = 0.0


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
