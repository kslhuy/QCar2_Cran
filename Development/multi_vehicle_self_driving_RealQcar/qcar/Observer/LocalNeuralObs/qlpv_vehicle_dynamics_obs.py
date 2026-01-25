"""
Centralized qLPV Vehicle Dynamics for Observer Architecture

This module provides a single source of truth for qLPV vehicle dynamics
used by all observers in the LocalNeuralObs architecture.

State: x = [v_x, v_y, ψ, r, X, Y]ᵀ (6D)
    - v_x: Longitudinal velocity (body frame)
    - v_y: Lateral velocity (body frame)
    - ψ: Yaw angle
    - r: Yaw rate
    - X: Global X position
    - Y: Global Y position

Tire residuals: w = [w_r, w_f]ᵀ
    - w_r: Rear tire force residual = F_yr - C_r·α_r
    - w_f: Front tire force residual = F_yf - C_f·α_f

Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ (6D with lateral acceleration)

Observer Equation:
    ẋ = A(ρ)·x + B(ρ)·u + E(ρ)·w
    y = C(ρ)·x + D(ρ)·u + F(ρ)·w

where ρ = {1/v_x, sin(δ), cos(δ), v_x, v_y, sin(ψ), cos(ψ)} are scheduling parameters.

References:
    - qLPV vehicle dynamics with tire-residual estimation
    - UIO (Unknown Input Observer) for disturbance estimation
"""

import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass


# =============================================================================
# State and Measurement Index Constants
# =============================================================================

# State indices (6D state)
IDX_VX = 0
IDX_VY = 1
IDX_PSI = 2
IDX_R = 3
IDX_X = 4
IDX_Y = 5
STATE_DIM = 6

# Augmented state indices (8D with tire residuals)
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

# =============================================================================
# 8D System Constants (New System)
# =============================================================================

# State indices (8D state: [v_x, v_y, ψ, r, X, Y, a_x, a_y]ᵀ)
IDX8_VX = 0
IDX8_VY = 1
IDX8_PSI = 2
IDX8_R = 3
IDX8_X = 4
IDX8_Y = 5
IDX8_AX = 6
IDX8_AY = 7
STATE_DIM_8D = 8

# Measurement indices (7D measurement: [v_x, r, ψ, X, Y, a_y, a_x]ᵀ)
MEAS8_IDX_VX = 0
MEAS8_IDX_R = 1
MEAS8_IDX_PSI = 2
MEAS8_IDX_X = 3
MEAS8_IDX_Y = 4
MEAS8_IDX_AY = 5
MEAS8_IDX_AX = 6
MEAS_DIM_7D = 7

# Augmented 8D System (with 2 tire residuals) -> 10D
# x_a = [x(8); w(2)]
IDX8_WR = 8
IDX8_WF = 9
AUGMENTED_DIM_10D = 10


# =============================================================================
# Vehicle Parameters (Single Source of Truth: parameters_qcar.yaml)
# =============================================================================

# Path to YAML file (single source of truth)
import yaml
from pathlib import Path

_YAML_PARAMS_PATH = Path(__file__).parent.parent.parent / "GUI" / "vehiclemodels" / "parameters" / "parameters_qcar.yaml"

# Required parameter keys
_REQUIRED_YAML_KEYS = ['a', 'b', 'm', 'I_z', 'Cf', 'Cr']

# Cached params (loaded once on module import)
_CACHED_VEHICLE_PARAMS: Optional[Dict] = None


def load_vehicle_params_from_yaml(yaml_path: Optional[Path] = None) -> Dict:
    """
    Load vehicle parameters from YAML file (single source of truth)
    
    Args:
        yaml_path: Path to YAML file. Uses default parameters_qcar.yaml if None.
        
    Returns:
        Dict of vehicle parameters with observer-compatible keys
        
    Raises:
        FileNotFoundError: If YAML file does not exist
        ValueError: If required keys are missing from YAML
    """
    path = yaml_path or _YAML_PARAMS_PATH
    
    if not path.exists():
        raise FileNotFoundError(
            f"Vehicle parameter YAML not found at: {path}\n"
            f"Please ensure parameters_qcar.yaml exists in GUI/vehiclemodels/parameters/"
        )
    
    with open(path, 'r') as f:
        yaml_data = yaml.safe_load(f)
    
    # Check required keys exist in YAML
    missing_keys = [k for k in _REQUIRED_YAML_KEYS if k not in yaml_data]
    if missing_keys:
        raise ValueError(
            f"Missing required keys in YAML file: {missing_keys}\n"
            f"Required keys: {_REQUIRED_YAML_KEYS}"
        )
    
    # Map YAML keys to observer parameter keys
    params = {
        'lf': yaml_data['a'],           # 'a' in YAML = front axle distance
        'lr': yaml_data['b'],           # 'b' in YAML = rear axle distance
        'm': yaml_data['m'],
        'Iz': yaml_data['I_z'],
        'Cf': yaml_data['Cf'],
        'Cr': yaml_data['Cr'],
        'mu': yaml_data.get('mu'),  
        'vx_min': yaml_data.get('longitudinal', {}).get('v_switch', 0.5),
    }
    return params


def get_vehicle_params() -> Dict:
    """
    Get vehicle parameters (cached from YAML).
    
    This is the preferred method for getting vehicle parameters.
    Parameters are loaded once from YAML and cached for efficiency.
    
    Returns:
        Dict of vehicle parameters
    """
    global _CACHED_VEHICLE_PARAMS
    if _CACHED_VEHICLE_PARAMS is None:
        _CACHED_VEHICLE_PARAMS = load_vehicle_params_from_yaml()
    return _CACHED_VEHICLE_PARAMS.copy()


# Backward compatibility aliases
def get_default_vehicle_params() -> Dict:
    """Deprecated: Use get_vehicle_params() instead."""
    return get_vehicle_params()


def get_vehicle_params_from_yaml() -> Dict:
    """Deprecated: Use get_vehicle_params() instead."""
    return get_vehicle_params()


# =============================================================================
# Scheduling Parameters
# =============================================================================

@dataclass
class SchedulingParameters:
    """
    Scheduling parameters ρ for qLPV system
    
    These parameters are computed from the current state and steering input,
    and are used to parameterize the state-space matrices.
    """
    inv_vx: float      # 1/v_x
    sin_delta: float   # sin(δ)
    cos_delta: float   # cos(δ)
    vx: float          # v_x (clamped to min_vx)
    vy: float          # v_y
    sin_psi: float     # sin(ψ)
    cos_psi: float     # cos(ψ)

    @classmethod
    def from_state_and_input(cls, state: np.ndarray, delta: float, 
                              min_vx: float = 0.1) -> 'SchedulingParameters':
        """
        Compute scheduling parameters from state and steering input
        
        Args:
            state: State vector [v_x, v_y, ψ, r, X, Y]
            delta: Steering angle [rad]
            min_vx: Minimum velocity to avoid singularity [m/s]
            
        Returns:
            SchedulingParameters instance
        """
        vx = max(abs(state[IDX_VX]), min_vx)  # Avoid division by zero
        vy = state[IDX_VY]
        psi = state[IDX_PSI]
        
        return cls(
            inv_vx=1.0 / vx,
            sin_delta=np.sin(delta),
            cos_delta=np.cos(delta),
            vx=vx,
            vy=vy,
            sin_psi=np.sin(psi),
            cos_psi=np.cos(psi)
        )


# =============================================================================
# qLPV Vehicle Dynamics Class
# =============================================================================

class QLPVVehicleDynamicsObs:
    """
    Centralized qLPV Vehicle Dynamics for Observer Use
    
    Provides all state-space matrices and dynamics functions used by
    the various observers in the LocalNeuralObs architecture.
    
    State vector: x = [v_x, v_y, ψ, r, X, Y]ᵀ
    Control input: u = [δ, a]ᵀ (steering, acceleration)
    Tire residuals: w = [w_r, w_f]ᵀ
    Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ
    """
    
    def __init__(self, vehicle_params: Optional[Dict] = None, min_vx: Optional[float] = None):
        """
        Initialize qLPV vehicle dynamics
        
        Args:
            vehicle_params: Vehicle parameters dict (uses defaults if None)
            min_vx: Minimum velocity threshold [m/s]. If None, uses vx_min from YAML.
        """
        # Vehicle parameters
        self.params = get_default_vehicle_params()
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
        self.g = 9.81  # Gravity [m/s²]
        
        # Minimum velocity threshold (from YAML if not explicitly provided)
        # Lower value allows estimation closer to zero when vehicle stops
        self.min_vx = min_vx if min_vx is not None else self.params['vx_min']
        
        # Default system type
        self.use_8d_system = False
    
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
            delta: Steering angle [rad]
            
        Returns:
            Tuple of (alpha_f, alpha_r) [rad]
        """
        vx = max(abs(state[IDX_VX]), self.min_vx)
        vy = state[IDX_VY]
        r = state[IDX_R]
        
        alpha_f = delta - vy / vx - self.lf * r / vx
        alpha_r = -vy / vx + self.lr * r / vx
        
        return alpha_f, alpha_r
    
    # =========================================================================
    # State-Space Matrices
    # =========================================================================
    
    def compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute state matrix A(ρ) for qLPV system
        
        State: [v_x, v_y, ψ, r, X, Y]
        
        Uses linearized single-track model with linear tire assumption.
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            A matrix (6×6)
        """
        A = np.zeros((STATE_DIM, STATE_DIM))
        
        # Shortcuts
        inv_vx = rho.inv_vx
        cos_d = rho.cos_delta
        sin_d = rho.sin_delta
        vx = rho.vx
        vy = rho.vy
        cos_psi = rho.cos_psi
        sin_psi = rho.sin_psi
        
        # v_x dynamics: v̇_x = accel - μg + r·v_y - Fyf·sin(δ)/m
        # where Fyf = Cf·αf = Cf·(δ - vy/vx - lf·r/vx)
        # ∂vx_dot/∂vx = -μg/vx (friction approximation)
        # ∂vx_dot/∂vy = r + Cf·sin(δ)/(m·vx) (from r·vy and -Fyf·sin(δ)/m)
        # ∂vx_dot/∂r = vy + Cf·lf·sin(δ)/(m·vx) (from r·vy and -Fyf·sin(δ)/m)
        A[IDX_VX, IDX_VX] = -self.mu * self.g / vx  # Longitudinal friction approx
        A[IDX_VX, IDX_VY] =  self.Cf * sin_d / (self.m * vx)  # Note: r term omitted (scheduling param)
        A[IDX_VX, IDX_R] = vy + self.Cf * self.lf * sin_d / (self.m * vx)  # vy from r·vy coupling
        
        # v_y dynamics: v̇_y = F_yr/m + F_yf·cos(δ)/m - r·v_x
        A[IDX_VY, IDX_VY] = -(self.Cr + self.Cf * cos_d) / (self.m * vx)
        A[IDX_VY, IDX_R] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.m * vx) - vx
        
        # ψ dynamics: ψ̇ = r
        A[IDX_PSI, IDX_R] = 1.0
        
        # r dynamics: ṙ = (l_f·F_yf·cos(δ) - l_r·F_yr) / I_z
        A[IDX_R, IDX_VY] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.Iz * vx)
        A[IDX_R, IDX_R] = -(self.Cf * self.lf**2 * cos_d + self.Cr * self.lr**2) / (self.Iz * vx)
        
        # X dynamics: Ẋ = v_x·cos(ψ) - v_y·sin(ψ)
        A[IDX_X, IDX_VX] = cos_psi
        A[IDX_X, IDX_VY] = -sin_psi
        # A[IDX_X, IDX_PSI] = -vx * sin_psi - vy * cos_psi  # ∂/∂ψ
        
        # Y dynamics: Ẏ = v_x·sin(ψ) + v_y·cos(ψ)
        A[IDX_Y, IDX_VX] = sin_psi
        A[IDX_Y, IDX_VY] = cos_psi
        # A[IDX_Y, IDX_PSI] = vx * cos_psi - vy * sin_psi  # ∂/∂ψ
        
        return A
    
    def compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute input matrix B(ρ)
        
        Input: u = [δ, a]ᵀ (steering, acceleration)
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            B matrix (6×2)
        """
        B = np.zeros((STATE_DIM, 2))
        
        cos_d = rho.cos_delta
        sin_d = rho.sin_delta
        
        # v_x: affected by steering and acceleration
        B[IDX_VX, 0] = -self.Cf * sin_d / self.m  # ∂/∂δ (steering effect)
        B[IDX_VX, 1] = 1.0  # ∂/∂a (direct acceleration)
        
        # v_y: affected by steering (front tire force direction change)
        B[IDX_VY, 0] = self.Cf * cos_d / self.m
        
        # r: affected by steering through front tire moment
        B[IDX_R, 0] = self.Cf * self.lf * cos_d / self.Iz
        
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
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            E matrix (6×2)
        """
        E = np.zeros((STATE_DIM, 2))
        
        cos_d = rho.cos_delta
        sin_d = rho.sin_delta
        
        # v_x: front tire residual affects through sin(δ)
        E[IDX_VX, 1] = -sin_d / self.m
        
        # v_y: both residuals contribute
        E[IDX_VY, 0] = 1.0 / self.m  # w_r
        E[IDX_VY, 1] = cos_d / self.m  # w_f·cos(δ)
        
        # r: both residuals create moment
        E[IDX_R, 0] = -self.lr / self.Iz  # -l_r·w_r / I_z
        E[IDX_R, 1] = self.lf * cos_d / self.Iz  # l_f·w_f·cos(δ) / I_z
        
        return E
    
    def compute_C_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute output matrix C(ρ)
        
        Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ
        
        First 5 rows are simple state selections.
        Row 6 (a_y) is computed from lateral dynamics:
            a_y = v̇_y + r·v_x = F_yr/m + F_yf·cos(δ)/m
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            C matrix (6×6)
        """
        C = np.zeros((MEAS_DIM, STATE_DIM))
        
        # Direct measurements
        C[MEAS_IDX_VX, IDX_VX] = 1.0    # v_x
        C[MEAS_IDX_R, IDX_R] = 1.0       # r
        C[MEAS_IDX_PSI, IDX_PSI] = 1.0   # ψ
        C[MEAS_IDX_X, IDX_X] = 1.0       # X
        C[MEAS_IDX_Y, IDX_Y] = 1.0       # Y
        
        # a_y measurement: C_ay(ρ)·x
        # C_ay = [0, -(C_r + C_f·cos(δ))/(m·v_x), 0, (C_r·l_r - C_f·l_f·cos(δ))/(m·v_x), 0, 0]
        cos_d = rho.cos_delta
        vx = rho.vx
        
        C[MEAS_IDX_AY, IDX_VY] = -(self.Cr + self.Cf * cos_d) / (self.m * vx)
        C[MEAS_IDX_AY, IDX_R] = (self.Cr * self.lr - self.Cf * self.lf * cos_d) / (self.m * vx)
        
        return C
    
    def compute_D_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute feedthrough matrix D(ρ) from input to output
        
        Only a_y has feedthrough from steering:
            D_ay = [C_f·cos(δ)/m, 0]
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            D matrix (6×2)
        """
        D = np.zeros((MEAS_DIM, 2))
        
        cos_d = rho.cos_delta
        
        # Only a_y row has feedthrough
        D[MEAS_IDX_AY, 0] = self.Cf * cos_d / self.m  # Steering effect
        
        return D
    
    def compute_F_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute residual-to-output matrix F(ρ)
        
        Only a_y is affected by tire residuals:
            F_ay = [1/m, cos(δ)/m]
        
        This is KEY for UIO-style residual estimation!
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            F matrix (6×2)
        """
        F = np.zeros((MEAS_DIM, 2))
        
        cos_d = rho.cos_delta
        
        # a_y row: a_y = ... + w_r/m + w_f·cos(δ)/m
        F[MEAS_IDX_AY, 0] = 1.0 / self.m  # w_r contribution
        F[MEAS_IDX_AY, 1] = cos_d / self.m  # w_f contribution
        
        return F
    
    # =========================================================================
    # Augmented System Matrices
    # =========================================================================
    
    def compute_augmented_matrices(self, rho: SchedulingParameters) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute augmented system matrices for x_a = [x; w]
        
        Augmented dynamics (assuming ẇ ≈ 0):
            A_a = [[A(ρ), E(ρ)], [0, 0]]
            B_a = [[B(ρ)], [0]]
            C_a = [C(ρ), F(ρ)]
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            Tuple of (A_a, B_a, C_a)
        """
        A = self.compute_A_matrix(rho)
        B = self.compute_B_matrix(rho)
        E = self.compute_E_matrix(rho)
        C = self.compute_C_matrix(rho)
        F = self.compute_F_matrix(rho)
        
        # Augmented state matrix (8×8)
        A_a = np.zeros((AUGMENTED_DIM, AUGMENTED_DIM))
        A_a[:STATE_DIM, :STATE_DIM] = A
        A_a[:STATE_DIM, STATE_DIM:] = E
        # Lower right block is zeros (ẇ = 0 assumption)
        
        # Augmented input matrix (8×2)
        B_a = np.zeros((AUGMENTED_DIM, 2))
        B_a[:STATE_DIM, :] = B
        
        # Augmented output matrix (6×8)
        C_a = np.zeros((MEAS_DIM, AUGMENTED_DIM))
        C_a[:, :STATE_DIM] = C
        C_a[:, STATE_DIM:] = F
        
        return A_a, B_a, C_a
    
    # =========================================================================
    # Kinematic Model (for Low-Speed Operation)
    # =========================================================================
    
    def f_kinematic(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Kinematic bicycle model for low-speed operation.
        Matches vehicle_dynamics_qlpv.py simplified kinematic model.
        
        State: [vx, vy, psi, r, X, Y]
        """
        vx = x[IDX_VX]
        vy = x[IDX_VY]
        psi = x[IDX_PSI]
        r = x[IDX_R]
        
        delta = u[0]
        accel = u[1] if len(u) > 1 else 0.0
        
        lf = self.lf
        lr = self.lr
        lwb = lf + lr
        m = self.m
        g = self.g
        
        # Get Rolling resistance (default from vehicle_dynamics_qlpv logic)
        Cr_roll = self.params.get('Cr_roll', 0.015)
        
        # Compute beta (sideslip angle)
        # matches: beta = math.atan2(lr * math.tan(delta), lwb)
        if abs(delta) > 0.001:
            beta = np.arctan2(lr * np.tan(delta), lwb)
        else:
            beta = 0.0
        
        # Apply friction even in kinematic mode for proper stopping
        F_roll_kin = m * g * Cr_roll
        if abs(vx) > 0.02:
            friction_decel_kin = F_roll_kin / m * np.sign(vx)
        else:
            friction_decel_kin = 0.0
            
        # Velocity derivative with friction
        vx_dot = accel - friction_decel_kin
        
        # Force complete stop when very slow and no acceleration commanded
        if abs(vx) < 0.02 and abs(accel) < 0.01:
            vx_dot = -vx / 0.1  # Damp to zero quickly
            
        # Position derivatives (incorporating beta)
        # matches: vx * math.cos(psi + beta)
        X_dot = vx * np.cos(psi + beta)
        
        # matches: vx * math.sin(psi + beta)
        Y_dot = vx * np.sin(psi + beta)
        
        # Yaw rate (psi_dot)
        # matches: vx / lwb * math.tan(delta) * math.cos(beta)
        if abs(vx) > 0.01:
            psi_dot = vx / lwb * np.tan(delta) * np.cos(beta)
        else:
            psi_dot = 0.0
            
        # Simplified dynamics for other states
        # matches: 0.0 for r_dot and vy_dot
        r_dot = 0.0
        vy_dot = 0.0
        
        return np.array([vx_dot, vy_dot, psi_dot, r_dot, X_dot, Y_dot])
    
    def f_blended(self, x: np.ndarray, u: np.ndarray, w: np.ndarray,
                  blend_vx_low: float = 0.1, blend_vx_high: float = 0.3) -> np.ndarray:
        """
        Blended dynamics that smoothly transitions between kinematic and dynamic models.
        
        Uses smooth blending function to avoid discontinuities:
            - |vx| < blend_vx_low: Pure kinematic model
            - |vx| > blend_vx_high: Pure dynamic model
            - blend_vx_low <= |vx| <= blend_vx_high: Smooth blend
        
        Args:
            x: State [vx, vy, psi, r, X, Y]
            u: Control [δ, a]
            w: Tire residuals [wr, wf] (used only in dynamic model)
            blend_vx_low: Velocity threshold for pure kinematic [m/s]
            blend_vx_high: Velocity threshold for pure dynamic [m/s]
        
        Returns:
            Blended state derivative ẋ (6D)
        """
        vx_abs = abs(x[IDX_VX])
        
        # Compute blending factor (0 = kinematic, 1 = dynamic)
        if vx_abs <= blend_vx_low:
            alpha = 0.0
        elif vx_abs >= blend_vx_high:
            alpha = 1.0
        else:
            # Smooth cosine blend
            t = (vx_abs - blend_vx_low) / (blend_vx_high - blend_vx_low)
            alpha = 0.5 * (1.0 - np.cos(np.pi * t))
        
        # Compute both models
        x_dot_kin = self.f_kinematic(x, u)
        
        if alpha < 1.0:
            # Need kinematic contribution
            if alpha > 0.0:
                # Need dynamic contribution too
                x_dot_dyn = self.f_continuous(x, u, w)
                return (1.0 - alpha) * x_dot_kin + alpha * x_dot_dyn
            else:
                return x_dot_kin
        else:
            # Pure dynamic
            return self.f_continuous(x, u, w)
    
    def is_low_speed(self, vx: float, threshold: float = 0.15) -> bool:
        """
        Check if vehicle is in low-speed regime where kinematic model is preferred.
        
        Args:
            vx: Longitudinal velocity [m/s]
            threshold: Speed threshold [m/s]
            
        Returns:
            True if |vx| < threshold
        """
        return abs(vx) < threshold
    
    # =========================================================================
    # Continuous-Time Dynamics
    # =========================================================================
    
    def f_continuous(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """
        Continuous-time state dynamics (dynamic bicycle model)
        
        ẋ = f(x, u, w)
        
        Args:
            x: State [vx, vy, psi, r, X, Y]
            u: Control [δ, a]
            w: Tire residuals [wr, wf]
        
        Returns:
            State derivative ẋ
        """
        vx = max(abs(x[IDX_VX]), self.min_vx)
        vy = x[IDX_VY]
        psi = x[IDX_PSI]
        r = x[IDX_R]
        
        delta = u[0]
        accel = u[1] 
        
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
        
        Args:
            x: State [vx, vy, psi, r, X, Y]
            u: Control [δ, a]
            w: Tire residuals [wr, wf]
        
        Returns:
            Predicted measurement [vx, r, psi, X, Y, ay]
        """
        vx = max(abs(x[IDX_VX]), self.min_vx)
        vy = x[IDX_VY]
        r = x[IDX_R]
        
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
            x[IDX_VX],
            r,
            x[IDX_PSI],
            x[IDX_X],
            x[IDX_Y],
            ay,
        ])
    
    def h_meas_blended(self, x: np.ndarray, u: np.ndarray, w: np.ndarray,
                       blend_vx_low: float = 0.1, blend_vx_high: float = 0.3) -> np.ndarray:
        """
        Blended measurement function for kinematic/dynamic transition.
        
        At low speeds, lateral acceleration is approximated as centripetal: ay ≈ r·vx
        At high speeds, uses full tire force model.
        
        Args:
            x: State [vx, vy, psi, r, X, Y]
            u: Control [δ, a]
            w: Tire residuals [wr, wf]
            blend_vx_low: Velocity threshold for pure kinematic [m/s]
            blend_vx_high: Velocity threshold for pure dynamic [m/s]
        
        Returns:
            Predicted measurement [vx, r, psi, X, Y, ay]
        """
        vx = x[IDX_VX]
        vx_abs = abs(vx)
        r = x[IDX_R]
        
        # Compute blending factor (0 = kinematic, 1 = dynamic)
        if vx_abs <= blend_vx_low:
            alpha = 0.0
        elif vx_abs >= blend_vx_high:
            alpha = 1.0
        else:
            t = (vx_abs - blend_vx_low) / (blend_vx_high - blend_vx_low)
            alpha = 0.5 * (1.0 - np.cos(np.pi * t))
        
        # Kinematic ay (centripetal approximation)
        ay_kin = r * vx
        
        # Dynamic ay (full tire model)
        if alpha > 0:
            y_dyn = self.h_meas(x, u, w)
            ay_dyn = y_dyn[MEAS_IDX_AY]
            ay = (1.0 - alpha) * ay_kin + alpha * ay_dyn
        else:
            ay = ay_kin
        
        return np.array([
            vx,
            r,
            x[IDX_PSI],
            x[IDX_X],
            x[IDX_Y],
            ay,
        ])
    
    # =========================================================================
    # UIO-Style Residual Computation Helpers
    # =========================================================================
    
    def compute_rdot_linear(self, xhat: np.ndarray, delta: float) -> float:
        """
        Compute linear prediction for yaw rate derivative
        
        ṙ_lin = f(vx, vy, r, delta, params)
        
        Args:
            xhat: State estimate [vx, vy, psi, r, X, Y]
            delta: Steering angle
            
        Returns:
            rdot_lin: Linear predicted yaw rate derivative
        """
        vx, vy, psi, r, X, Y = xhat
        vx_eff = max(abs(vx), self.min_vx)
        c = np.cos(delta)
        
        rdot_lin = (
            - (self.lf*self.Cf*c - self.lr*self.Cr) / (self.Iz*vx_eff) * vy
            - (self.lf**2 * self.Cf*c + self.lr**2 * self.Cr) / (self.Iz*vx_eff) * r
            + (self.lf*self.Cf*c) / self.Iz * delta
        )
        
        return rdot_lin
    
    def compute_ay_linear(self, xhat: np.ndarray, delta: float) -> float:
        """
        Compute linear prediction for lateral acceleration
        
        a_y_lin = f(vx, vy, r, delta, params)
        
        Args:
            xhat: State estimate [vx, vy, psi, r, X, Y]
            delta: Steering angle
            
        Returns:
            ay_lin: Linear predicted lateral acceleration
        """
        vx, vy, psi, r, X, Y = xhat
        vx_eff = max(abs(vx), self.min_vx)
        c = np.cos(delta)
        
        ay_lin = (
            - (self.Cr + self.Cf*c) / (self.m*vx_eff) * vy
            + (self.Cr*self.lr - self.Cf*self.lf*c) / (self.m*vx_eff) * r
            + (self.Cf*c) / self.m * delta
        )
        
        return ay_lin
    
    def compute_w_estimation_matrix(self, delta: float) -> np.ndarray:
        """
        Compute M matrix for w estimation from residuals
        
        M·w = b, where b = [rdot_res, ay_res]ᵀ
        
        M = [[-lr/Iz, lf·cos(δ)/Iz],
             [1/m,    cos(δ)/m    ]]
        
        Args:
            delta: Steering angle
            
        Returns:
            M matrix (2×2)
        """
        c = np.cos(delta)
        
        M = np.array([
            [-self.lr/self.Iz,  self.lf*c/self.Iz],
            [1.0/self.m,        c/self.m         ]
        ], dtype=float)
        
        return M



# =============================================================================
# qLPV Vehicle Dynamics Class (8D State)
# =============================================================================

class QLPVVehicleDynamicsObs8D(QLPVVehicleDynamicsObs):
    """
    Centralized qLPV Vehicle Dynamics for Observer Use (8D State)
    
    Extends the standard 6D model by treating accelerations as states.
    
    State vector: x = [v_x, v_y, ψ, r, X, Y, a_x, a_y]ᵀ
    Control input: u = [δ, a]ᵀ (steering, acceleration)
    Tire residuals: w = [w_r, w_f]ᵀ
    Measurements: y = [v_x, r, ψ, X, Y, a_y, a_x]ᵀ
    """
    
    def __init__(self, vehicle_params: Optional[Dict] = None, min_vx: Optional[float] = None):
        super().__init__(vehicle_params, min_vx)
        self.use_8d_system = True
        
    def f_continuous(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """
        Continuous-time state dynamics for 8D system
        
        ẋ = f(x, u, w)
        
        x = [v_x, v_y, ψ, r, X, Y, a_x, a_y]
        
        Assumes random walk for accelerations: ȧ = 0
        """
        # Use base 6D dynamics for physical states
        x_6d = x[:STATE_DIM]
        # w should be same [wr, wf]
        x_dot_6d = super().f_continuous(x_6d, u, w)
        
        # Zero dynamics for accelerations
        return np.concatenate([x_dot_6d, np.zeros(2)])
    
    def compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute state matrix A(ρ) for 8D qLPV system
        
        State: [v_x, v_y, ψ, r, X, Y, a_x, a_y]
        
        Assumes random walk model for accelerations: 
            ȧ_x = 0
            ȧ_y = 0
        
        The physical couplings remain for the first 6 states.
        """
        # Get 6D A matrix first
        A_6d = super().compute_A_matrix(rho)
        
        # Create 8D A matrix
        A = np.zeros((STATE_DIM_8D, STATE_DIM_8D))
        
        # Fill top-left 6x6 with standard dynamics
        A[:STATE_DIM, :STATE_DIM] = A_6d
        
        # Acceleration states are modeled as random walks (zeros in dynamics rows)
        # But they might affect other states if we wanted to use them 
        # (e.g. v_dot = a_x), but here we keep the physical model 
        # driving v_dot and treating a_x, a_y as separate estimated states.
        
        return A
    
    def compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute input matrix B(ρ) for 8D system"""
        B_6d = super().compute_B_matrix(rho)
        
        B = np.zeros((STATE_DIM_8D, 2))
        B[:STATE_DIM, :] = B_6d
        
        return B
    
    def compute_E_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute residual injection matrix E(ρ) for 8D system"""
        E_6d = super().compute_E_matrix(rho)
        
        E = np.zeros((STATE_DIM_8D, 2))
        E[:STATE_DIM, :] = E_6d
        
        return E
    
    def compute_F_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute residual feedthrough matrix F(ρ) for 8D system
        
        Maps tire residuals [w_r, w_f] to measurements.
        This is CRITICAL for UIO-style residual estimation!
        
        Even though a_y is a state in 8D system, the physical relationship
        is: a_y = (F_yr + F_yf·cos(δ))/m = ... + (w_r + w_f·cos(δ))/m
        
        Without F in C_a, residuals have NO observability through measurements.
        
        Args:
            rho: Scheduling parameters
            
        Returns:
            F matrix (7×2) - feedthrough from [w_r, w_f] to measurements
        """
        F = np.zeros((MEAS_DIM_7D, 2))
        
        cos_d = rho.cos_delta
        
        # a_y measurement depends on tire residuals:
        # a_y = (F_yr + F_yf·cos(δ))/m = (C_r·α_r + w_r + (C_f·α_f + w_f)·cos(δ))/m
        # Therefore: ∂a_y/∂w_r = 1/m, ∂a_y/∂w_f = cos(δ)/m
        F[MEAS8_IDX_AY, 0] = 1.0 / self.m  # w_r contribution to a_y
        F[MEAS8_IDX_AY, 1] = cos_d / self.m  # w_f contribution to a_y
        
        return F
    
    def compute_C_matrix(self, rho: SchedulingParameters, gps_available: bool = True) -> np.ndarray:
        """
        Compute output matrix C(ρ) for 8D system
        
        Measurements: y = [v_x, r, ψ, X, Y, a_y, a_x]ᵀ
        
        If gps_available is False:
            The rows for X, Y, and potentially ψ are zeroed out (or handled by observer).
            Strictly speaking, C should reflect available measurements.
            However, typically we keep C size fixed and use R=infinity for missing.
            But here we return the FULL C matrix. The observer can slice it 
            or we can zero it out here. 
            
            Let's return the full theoretical C matrix here.
            The dynamic switching logic (gps vs no gps) involves changing 
            WHICH usage of C is done (e.g. for LMI design).
        
        Args:
            rho: Scheduling parameters
            gps_available: Whether GPS measurements (X, Y) are available.
                           Note: If False, this function currently still returns 
                           full ideal C. Observer gain design should use reduced C.
            
        Returns:
            C matrix (7×8)
        """
        C = np.zeros((MEAS_DIM_7D, STATE_DIM_8D))
        
        # 1. v_x
        C[MEAS8_IDX_VX, IDX8_VX] = 1.0
        
        # 2. r
        C[MEAS8_IDX_R, IDX8_R] = 1.0
        
        # 3. ψ (heading)
        C[MEAS8_IDX_PSI, IDX8_PSI] = 1.0
        
        # 4. X
        C[MEAS8_IDX_X, IDX8_X] = 1.0
        
        # 5. Y
        C[MEAS8_IDX_Y, IDX8_Y] = 1.0
        
        # 6. a_y (measured directly or linked to state a_y)
        # Using the state a_y directly as measurement model: y_ay = a_y
        C[MEAS8_IDX_AY, IDX8_AY] = 1.0
        
        # 7. a_x (measured directly or linked to state a_x)
        C[MEAS8_IDX_AX, IDX8_AX] = 1.0
        
        # Note: We are using the "Direct State Measurement" model for a_x, a_y here
        # instead of the physical model (a_y = Fyr/m + ...) because we added them as states.
        # This implies the observer is "filtering" the accelerometer values.
        
        if not gps_available:
            # Zero out GPS rows to reflect NO connection to state (infinite variance)
            # This allows C*hat_x prediction to be 0 for these, but strictly 
            # for gain design we want to remove the rows. 
            # Retaining zeros means y = 0*x, which isn't right if y is random/missing.
            # Best pattern: Return FULL C, let Observer handle slicing or R-matrix.
            pass

        return C
    
    def h_meas(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """
        Measurement function y = h(x, u, w) for 8D system
        
        CRITICAL: Includes tire residual contribution to a_y measurement!
        
        Physical model: a_y_measured = a_y_state + (w_r + w_f·cos(δ))/m
        
        This ensures consistency with the F matrix in C_a, where the tire
        residuals contribute to the predicted a_y measurement.
        
        Args:
            x: State [vx, vy, psi, r, X, Y, ax, ay]
            u: Control [δ, a] - needed for cos(δ) term
            w: Tire residuals [w_r, w_f]
            
        Returns:
            y: [vx, r, psi, X, Y, ay, ax] where ay includes residual contribution
        """
        y = np.zeros(MEAS_DIM_7D)
        
        y[MEAS8_IDX_VX] = x[IDX8_VX]
        y[MEAS8_IDX_R] = x[IDX8_R]
        y[MEAS8_IDX_PSI] = x[IDX8_PSI]
        y[MEAS8_IDX_X] = x[IDX8_X]
        y[MEAS8_IDX_Y] = x[IDX8_Y]
        y[MEAS8_IDX_AX] = x[IDX8_AX]
        
        # CRITICAL FIX: Include tire residual contribution to a_y
        # a_y = a_y_state + F·w = a_y_state + (w_r + w_f·cos(δ))/m
        delta = u[0] if len(u) > 0 else 0.0
        cos_delta = np.cos(delta)
        wr = w[0] if len(w) > 0 else 0.0
        wf = w[1] if len(w) > 1 else 0.0
        y[MEAS8_IDX_AY] = x[IDX8_AY] + (wr + wf * cos_delta) / self.m
        
        return y
    
    def f_kinematic(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Kinematic bicycle model for 8D system at low speeds.
        
        State: [v_x, v_y, ψ, r, X, Y, a_x, a_y]
        
        Args:
            x: State vector (8D)
            u: Control [δ, a]
        
        Returns:
            State derivative ẋ (8D)
        """
        # Use 6D kinematic for physical states (inherits damping behavior)
        x_6d = x[:STATE_DIM]
        x_dot_6d = super().f_kinematic(x_6d, u)
        
        # Acceleration states: decay towards zero when stopped
        vx = x[IDX8_VX]
        accel = u[1] if len(u) > 1 else 0.0
        
        # Damping coefficient (same as 6D model)
        damping = 2.0
        
        # Target accelerations based on kinematic model
        # ax should match vx_dot from kinematic model
        ax_target = accel - damping * vx
        ay_target = 0.0  # Kinematic: no lateral acceleration
        
        # First-order dynamics to drive accelerations towards target
        tau = 0.1  # Time constant
        ax_dot = (ax_target - x[IDX8_AX]) / tau
        ay_dot = (ay_target - x[IDX8_AY]) / tau
        
        return np.concatenate([x_dot_6d, [ax_dot, ay_dot]])
    
    def f_blended(self, x: np.ndarray, u: np.ndarray, w: np.ndarray,
                  blend_vx_low: float = 0.1, blend_vx_high: float = 0.3) -> np.ndarray:
        """
        Blended dynamics for 8D system.
        
        Args:
            x: State (8D)
            u: Control [δ, a]
            w: Tire residuals [wr, wf]
            blend_vx_low: Pure kinematic threshold
            blend_vx_high: Pure dynamic threshold
        
        Returns:
            Blended state derivative (8D)
        """
        vx_abs = abs(x[IDX8_VX])
        
        # Compute blending factor
        if vx_abs <= blend_vx_low:
            alpha = 0.0
        elif vx_abs >= blend_vx_high:
            alpha = 1.0
        else:
            t = (vx_abs - blend_vx_low) / (blend_vx_high - blend_vx_low)
            alpha = 0.5 * (1.0 - np.cos(np.pi * t))
        
        x_dot_kin = self.f_kinematic(x, u)
        
        if alpha < 1.0:
            if alpha > 0.0:
                x_dot_dyn = self.f_continuous(x, u, w)
                return (1.0 - alpha) * x_dot_kin + alpha * x_dot_dyn
            else:
                return x_dot_kin
        else:
            return self.f_continuous(x, u, w)
    
    def h_meas_blended(self, x: np.ndarray, u: np.ndarray, w: np.ndarray,
                       blend_vx_low: float = 0.1, blend_vx_high: float = 0.3) -> np.ndarray:
        """
        Blended measurement for 8D system (direct state measurement).
        
        For 8D system, accelerations are states measured directly, so blending
        is less critical. Returns direct measurement.
        """
        return self.h_meas(x, u, w)

    def compute_augmented_matrices(self, rho: SchedulingParameters) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute augmented system matrices for 8D system
        
        CRITICAL FIX: Include F matrix in C_a for tire residual observability!
        
        Without F in C_a, the a_y measurement provides NO information to
        update tire residual estimates w_r, w_f. This causes Kalman gain
        for residuals to be effectively zero, resulting in w estimates
        stuck at their initial values (typically zero).
        
        Physical relationship: a_y = (F_yr + F_yf·cos(δ))/m
                             = (C_r·α_r + w_r + (C_f·α_f + w_f)·cos(δ))/m
        
        Returns:
            Tuple of (A_a (10x10), B_a (10x2), C_a (7x10))
        """
        A = self.compute_A_matrix(rho)
        B = self.compute_B_matrix(rho)
        E = self.compute_E_matrix(rho)
        C = self.compute_C_matrix(rho)
        F = self.compute_F_matrix(rho)  # CRITICAL: F provides observability of w!
        
        # Construct 10D matrices
        A_a = np.zeros((AUGMENTED_DIM_10D, AUGMENTED_DIM_10D))
        A_a[:STATE_DIM_8D, :STATE_DIM_8D] = A
        A_a[:STATE_DIM_8D, STATE_DIM_8D:] = E
        
        B_a = np.zeros((AUGMENTED_DIM_10D, 2))
        B_a[:STATE_DIM_8D, :] = B
        
        C_a = np.zeros((MEAS_DIM_7D, AUGMENTED_DIM_10D))
        C_a[:, :STATE_DIM_8D] = C
        # CRITICAL FIX: Include F matrix for tire residual observability!
        # Even though a_y is a state, the physical coupling means:
        # y_ay = a_y_state + (w_r + w_f·cos(δ))/m (residual contribution)
        C_a[:, STATE_DIM_8D:] = F
        
        return A_a, B_a, C_a

# =============================================================================
# Factory Functions
# =============================================================================


def create_qlpv_dynamics(vehicle_params: Optional[Dict] = None, 
                          min_vx: Optional[float] = None,
                          use_8d_system: bool = False) -> QLPVVehicleDynamicsObs:
    """
    Factory function to create qLPV vehicle dynamics instance
    
    Args:
        vehicle_params: Vehicle parameters dictionary
        min_vx: Minimum velocity threshold [m/s]
        use_8d_system: If True, returns QLPVVehicleDynamicsObs8D (8D state)
        
    Returns:
        Configured QLPVVehicleDynamicsObs (or subclass) instance
    """
    if use_8d_system:
        return QLPVVehicleDynamicsObs8D(vehicle_params=vehicle_params, min_vx=min_vx)
        
    return QLPVVehicleDynamicsObs(
        vehicle_params=vehicle_params,
        min_vx=min_vx
    )


# =============================================================================
# Test Code
# =============================================================================

if __name__ == '__main__':
    print("=" * 60)
    print("qLPV Vehicle Dynamics Module Test")
    print("=" * 60)
    
    # Create dynamics instance
    dynamics = create_qlpv_dynamics()
    
    # Test state
    x = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [vx, vy, psi, r, X, Y]
    u = np.array([0.1, 0.5])  # [delta, accel]
    w = np.array([0.0, 0.0])  # [wr, wf]
    
    # Compute scheduling parameters
    rho = dynamics.compute_scheduling_params(x, u[0])
    print(f"Scheduling params: vx={rho.vx:.2f}, cos_δ={rho.cos_delta:.4f}")
    
    # Compute matrices
    A = dynamics.compute_A_matrix(rho)
    B = dynamics.compute_B_matrix(rho)
    E = dynamics.compute_E_matrix(rho)
    C = dynamics.compute_C_matrix(rho)
    
    print(f"\nMatrix shapes:")
    print(f"  A: {A.shape}")
    print(f"  B: {B.shape}")
    print(f"  E: {E.shape}")
    print(f"  C: {C.shape}")
    
    # Test dynamics
    x_dot = dynamics.f_continuous(x, u, w)
    print(f"\nState derivative: {x_dot}")
    
    # Test measurement
    y = dynamics.h_meas(x, u, w)
    print(f"Measurement: {y}")
    
    print("\n✅ Module test PASSED")
