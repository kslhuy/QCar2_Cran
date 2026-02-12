"""
qLPV Vehicle Dynamics Model for Observer Testing

This vehicle dynamics model exactly matches the qLPV observer equations,
enabling proper testing of the observer's state and tire residual estimation.

State Vector Conventions:
    qLPV:     x = [X, Y, δ, v_x, ψ, r, v_y]ᵀ  (7D, CommonRoad convention)
    Observer: x = [v_x, v_y, ψ, r, X, Y]       (6D)

Control Input: u = [δ_dot, a]ᵀ (steering rate, acceleration)

Tire Models:
    - 'static_linear':  F = C * α (simple, no load transfer)
    - 'dynamic_linear': F = μ * Cs * Fz * α (with load transfer)
    - 'pacejka':        Magic formula (nonlinear, most realistic)

References:
    - qLPV vehicle dynamics from qlpv_observer.py
    - CommonRoad vehicle model conventions
"""
__author__ = "Observer Test Framework"
__version__ = "2.0"
__status__ = "Development"

import math
import numpy as np
from typing import Tuple, Dict
from dataclasses import dataclass
from enum import Enum

from .utils.steering_constraints import steering_constraints
from .utils.acceleration_constraints import acceleration_constraints


# =============================================================================
# Constants
# =============================================================================
GRAVITY = 9.81  # m/s²
AIR_DENSITY = 1.225  # kg/m³

# Velocity thresholds
VX_KINEMATIC_THRESHOLD = 0.1  # Switch to kinematic model below this
VX_STOP_THRESHOLD = 0.05  # Consider stopped below this
VX_MIN_SAFE = 0.5  # Minimum for slip angle calculation


class TireMode(Enum):
    """Tire model selection enumeration."""
    STATIC_LINEAR = 'static_linear'
    DYNAMIC_LINEAR = 'dynamic_linear'
    PACEJKA = 'pacejka'


@dataclass
class StateIndices:
    """State vector indices for qLPV model (CommonRoad convention)."""
    X: int = 0      # Global X position [m]
    Y: int = 1      # Global Y position [m]
    DELTA: int = 2  # Steering angle [rad]
    VX: int = 3     # Longitudinal velocity [m/s]
    PSI: int = 4    # Yaw angle [rad]
    R: int = 5      # Yaw rate [rad/s]
    VY: int = 6     # Lateral velocity [m/s]
    DIM: int = 7    # State dimension


# Global state index instance
IDX = StateIndices()


# =============================================================================
# Helper Functions
# =============================================================================
def _get_param(obj, name: str, default: float) -> float:
    """Safely get parameter with default value."""
    return getattr(obj, name, default)


def _compute_slip_angles(vy: float, r: float, delta: float,
                         lf: float, lr: float, vx_safe: float) -> Tuple[float, float]:
    """
    Compute front and rear slip angles.
    
    Args:
        vy: Lateral velocity [m/s]
        r: Yaw rate [rad/s]
        delta: Steering angle [rad]
        lf: Distance CG to front axle [m]
        lr: Distance CG to rear axle [m]
        vx_safe: Safe longitudinal velocity (clamped) [m/s]
    
    Returns:
        (alpha_f, alpha_r): Front and rear slip angles [rad]
    """
    alpha_f = delta - (vy + lf * r) / vx_safe
    alpha_r = -(vy - lr * r) / vx_safe
    return alpha_f, alpha_r


def _compute_normal_loads(m: float, lf: float, lr: float,
                          h_cg: float, a_long: float) -> Tuple[float, float]:
    """
    Compute vertical tire loads with longitudinal load transfer.
    
    Args:
        m: Vehicle mass [kg]
        lf: Distance CG to front axle [m]
        lr: Distance CG to rear axle [m]
        h_cg: Center of gravity height [m]
        a_long: Longitudinal acceleration [m/s²]
    
    Returns:
        (Fz_f, Fz_r): Front and rear normal loads [N]
    """
    wheelbase = lf + lr
    Fz_f = (m * GRAVITY * lr - m * a_long * h_cg) / wheelbase
    Fz_r = (m * GRAVITY * lf + m * a_long * h_cg) / wheelbase
    return max(Fz_f, 0.01), max(Fz_r, 0.01)


def _magic_formula(alpha: float, B: float, C: float, D: float, E: float) -> float:
    """
    Pacejka Magic Formula for tire lateral force.
    
    F = D * sin(C * arctan(B*α - E*(B*α - arctan(B*α))))
    """
    Ba = B * alpha
    return D * math.sin(C * math.atan(Ba - E * (Ba - math.atan(Ba))))


# =============================================================================
# Tire Force Models
# =============================================================================
def compute_tire_forces_linear(alpha_f: float, alpha_r: float,
                               Cf: float, Cr: float) -> Tuple[float, float]:
    """
    Linear tire model: F_y = C * α
    
    Args:
        alpha_f: Front slip angle [rad]
        alpha_r: Rear slip angle [rad]
        Cf: Front cornering stiffness [N/rad]
        Cr: Rear cornering stiffness [N/rad]
    
    Returns:
        (Fyf, Fyr): Lateral tire forces [N]
    """
    return Cf * alpha_f, Cr * alpha_r


def compute_tire_forces_load_transfer(alpha_f: float, alpha_r: float,
                                      p, a_long: float) -> Tuple[float, float]:
    """
    Linear tire model with dynamic load transfer.
    
    Formula: Fy = μ_road * Cs * Fz * α
    
    Args:
        alpha_f: Front slip angle [rad]
        alpha_r: Rear slip angle [rad]
        p: Vehicle parameters
        a_long: Longitudinal acceleration [m/s²]
    
    Returns:
        (Fyf, Fyr): Lateral tire forces [N]
    """
    # Vehicle parameters
    h_cg = _get_param(p, 'h_cg', 0.07)
    mu_road = _get_param(p, 'mu_road', 1.0)
    Csf = _get_param(p, 'Csf', 30.0)  # Normalized stiffness [N/N/rad]
    Csr = _get_param(p, 'Csr', 30.0)
    
    # Compute normal loads
    Fz_f, Fz_r = _compute_normal_loads(p.m, p.a, p.b, h_cg, a_long)
    
    # Dynamic cornering stiffness: C_dyn = μ * Cs * Fz
    Cf_dyn = mu_road * Csf * Fz_f
    Cr_dyn = mu_road * Csr * Fz_r
    
    return Cf_dyn * alpha_f, Cr_dyn * alpha_r


def compute_tire_forces_pacejka(alpha_f: float, alpha_r: float,
                                p, vx: float, a_long: float = 0.0) -> Tuple[float, float]:
    """
    Pacejka Magic Formula tire model with load transfer.
    
    Args:
        alpha_f: Front slip angle [rad]
        alpha_r: Rear slip angle [rad]
        p: Vehicle parameters with tire sub-object
        vx: Longitudinal velocity [m/s]
        a_long: Longitudinal acceleration [m/s²]
    
    Returns:
        (Fyf, Fyr): Lateral tire forces [N]
    """
    h_cg = _get_param(p, 'h_cg', 0.1)
    
    # Tire parameters
    tire = p.tire
    mu = tire.p_dy1   # Peak friction coefficient
    Cy = tire.p_cy1   # Shape factor
    Ky = tire.p_ky1   # Stiffness factor
    Ey = tire.p_ey1   # Curvature factor
    
    # Normal loads with load transfer
    Fz_f, Fz_r = _compute_normal_loads(p.m, p.a, p.b, h_cg, a_long)
    
    # Peak lateral forces
    Dyf, Dyr = mu * Fz_f, mu * Fz_r
    
    # Stiffness factors: B = Ky / (Cy * Dy)
    Byf = -Ky / (Cy * Dyf) if abs(Dyf) > 0.1 else 20.0
    Byr = -Ky / (Cy * Dyr) if abs(Dyr) > 0.1 else 20.0
    
    return _magic_formula(alpha_f, Byf, Cy, Dyf, Ey), _magic_formula(alpha_r, Byr, Cy, Dyr, Ey)


def compute_tire_forces(alpha_f: float, alpha_r: float, p,
                        tire_mode: str, vx: float = 1.0,
                        a_long: float = 0.0) -> Tuple[float, float]:
    """
    Unified tire force computation with mode selection.
    
    Args:
        alpha_f: Front slip angle [rad]
        alpha_r: Rear slip angle [rad]
        p: Vehicle parameters
        tire_mode: 'static_linear', 'dynamic_linear', or 'pacejka'
        vx: Longitudinal velocity [m/s]
        a_long: Longitudinal acceleration [m/s²]
    
    Returns:
        (Fyf, Fyr): Lateral tire forces [N]
    """
    Cf = _get_param(p, 'Cf', 120.0)
    Cr = _get_param(p, 'Cr', 120.0)
    
    if tire_mode == 'static_linear':
        return compute_tire_forces_linear(alpha_f, alpha_r, Cf, Cr)
    elif tire_mode == 'dynamic_linear':
        return compute_tire_forces_load_transfer(alpha_f, alpha_r, p, a_long)
    elif tire_mode == 'pacejka' and hasattr(p, 'tire'):
        return compute_tire_forces_pacejka(alpha_f, alpha_r, p, vx, a_long)
    else:
        # Fallback to dynamic linear
        return compute_tire_forces_load_transfer(alpha_f, alpha_r, p, a_long)


# =============================================================================
# Tire Residuals and Measurements
# =============================================================================
def get_tire_residuals(type_true_tire : str,alpha_f: float, alpha_r: float,
                       Cf: float, Cr: float, p,
                       vx: float = 1.0, a_long: float = 0.0) -> Tuple[float, float]:
    """
    Compute tire residuals (ground truth for observer testing).
    
    Residual = True tire force - Linear tire force
        w_r = F_yr_true - C_r * α_r
        w_f = F_yf_true - C_f * α_f
    
    Args:
        alpha_f: Front slip angle [rad]
        alpha_r: Rear slip angle [rad]
        Cf: Front cornering stiffness [N/rad]
        Cr: Rear cornering stiffness [N/rad]
        p: Vehicle parameters
        vx: Longitudinal velocity [m/s]
        a_long: Longitudinal acceleration [m/s²]
    
    Returns:
        (w_r, w_f): Tire force residuals [N]
    """
    # Linear forces (baseline from observer's linear model)
    # Must use the SAME Cf/Cr as the observer to get correct residuals
    # Observer uses Cf=120, Cr=120 from parameters_qcar.yaml
    # obs_Cf = 120.0  # Should match observer's Cf from YAML
    # obs_Cr = 120.0  # Should match observer's Cr from YAML
    obs_Cf = Cf
    obs_Cr = Cr
    Fyf_linear, Fyr_linear = compute_tire_forces_linear(alpha_f, alpha_r, obs_Cf, obs_Cr)
    
    # True forces
    
    if type_true_tire == 'pacejka':
        Fyf_true, Fyr_true = compute_tire_forces_pacejka(alpha_f, alpha_r, p, vx, a_long)
    elif type_true_tire == 'dynamic_linear':
        Fyf_true, Fyr_true = compute_tire_forces_load_transfer(alpha_f, alpha_r, p, a_long)
    elif type_true_tire == 'static_linear':
        Fyf_true, Fyr_true = compute_tire_forces_linear(alpha_f, alpha_r, Cf, Cr)
    
    return  Fyr_true - Fyr_linear, Fyf_true - Fyf_linear , Fyf_true, Fyr_true


def get_lateral_acceleration(Fyf: float, Fyr: float, delta: float, m: float) -> float:
    """
    Compute lateral acceleration: a_y = (F_yr + F_yf·cos(δ)) / m
    """
    return (Fyr + Fyf * math.cos(delta)) / m


# =============================================================================
# State Conversion Utilities
# =============================================================================
def state_qlpv_to_observer(x_qlpv: np.ndarray) -> np.ndarray:
    """
    Convert qLPV state to observer state format.
    
    qLPV:     [X, Y, δ, v_x, ψ, r, v_y] → Observer: [v_x, v_y, ψ, r, X, Y]
    """
    return np.array([
        x_qlpv[IDX.VX],   # v_x
        x_qlpv[IDX.VY],   # v_y
        x_qlpv[IDX.PSI],  # ψ
        x_qlpv[IDX.R],    # r
        x_qlpv[IDX.X],    # X
        x_qlpv[IDX.Y],    # Y
    ])


def state_observer_to_qlpv(x_obs: np.ndarray, delta: float = 0.0) -> np.ndarray:
    """
    Convert observer state to qLPV state format.
    
    Observer: [v_x, v_y, ψ, r, X, Y] → qLPV: [X, Y, δ, v_x, ψ, r, v_y]
    """
    return np.array([
        x_obs[4],   # X
        x_obs[5],   # Y
        delta,      # δ
        x_obs[0],   # v_x
        x_obs[2],   # ψ
        x_obs[3],   # r
        x_obs[1],   # v_y
    ])


# =============================================================================
# Dynamics Sub-functions
# =============================================================================
def _kinematic_dynamics(vx: float, psi: float, delta: float, u: list,
                        lf: float, lr: float, m: float, Cr_roll: float) -> list:
    """
    Simplified kinematic model for low velocities.
    
    Args:
        vx: Longitudinal velocity [m/s]
        psi: Yaw angle [rad]
        delta: Steering angle [rad]
        u: Control inputs [δ_dot, a]
        lf, lr: Axle distances [m]
        m: Vehicle mass [kg]
        Cr_roll: Rolling resistance coefficient
    Returns:
        State derivatives (7D list)
    """
    # wheelbase = lf + lr
    # beta = math.atan2(lr * math.tan(delta), wheelbase) if abs(delta) > 0.001 else 0.0
    # beta : Slip angle

    
    # Velocity derivative
    vx_dot = u[1]
    

    
    return np.array([
        vx * math.cos(psi),  # Ẋ
        vx * math.sin(psi),  # Ẏ
        u[0],               # δ̇
        vx_dot,              # v̇_x
        0.0,                 # ψ̇
        0.0,                 # ṙ (simplified)
        0.0,                 # v̇_y (simplified)
    ])


def _dynamic_qlpv(vx: float, vy: float, psi: float, r: float, delta: float,
                  u: list, Fyf: float, Fyr: float,
                  lf: float, lr: float, m: float, Iz: float,
                  mu: float, Cr_roll: float, Cd_aero: float, Af: float,
                  cos_psi: float, sin_psi: float,
                  cos_delta: float, sin_delta: float,
                  disturbances: list = None) -> list:
    """
    Full qLPV dynamic model equations.
    
    Args:
        vx, vy: Body-frame velocities [m/s]
        psi: Yaw angle [rad]
        r: Yaw rate [rad/s]
        delta: Steering angle [rad]
        u: Control inputs [δ_dot, a]
        Fyf, Fyr: Lateral tire forces [N]
        lf, lr: Axle distances [m]
        m: Mass [kg]
        Iz: Yaw inertia [kg·m²]
        mu: Road friction coefficient
        Cr_roll: Rolling resistance coefficient
        Cd_aero: Aerodynamic drag coefficient
        Af: Frontal area [m²]
        cos_psi, sin_psi: Precomputed trig values
        cos_delta, sin_delta: Precomputed trig values
        disturbances: Optional [d_vx, d_vy, d_r] additive disturbances
    
    Returns:
        State derivatives (7D list)
    """
    # Rolling resistance and aerodynamic drag
    F_roll = m * GRAVITY * Cr_roll
    vx_clamped = np.clip(vx, -10.0, 10.0)
    F_drag = 0.5 * AIR_DENSITY * Cd_aero * Af * vx_clamped * abs(vx_clamped)

    
    # State derivatives (qLPV equations)
    vx_dot = u[1] - mu * GRAVITY + r * vy - (Fyf * sin_delta) / m
    vy_dot = (Fyr + Fyf * cos_delta) / m - r * vx
    psi_dot = r
    r_dot = (lf * Fyf * cos_delta - lr * Fyr) / Iz
    X_dot = vx * cos_psi - vy * sin_psi
    Y_dot = vx * sin_psi + vy * cos_psi
    
    # Apply General Disturbances if provided
    # disturbances = [d_vx, d_vy, d_r]
    if disturbances is not None and len(disturbances) >= 3:
        vx_dot += disturbances[0]
        vy_dot += disturbances[1]
        r_dot += disturbances[2]
    
    return np.array([X_dot, Y_dot, u[0], vx_dot, psi_dot, r_dot, vy_dot])


# =============================================================================
# Main Dynamics Function
# =============================================================================
def vehicle_dynamics_qlpv(x, u_init, p, tire_mode: str = 'pacejka', 
                          disturbances: list = None):
    """
    qLPV vehicle dynamics matching observer model.
    
    Args:
        x: State vector [X, Y, δ, v_x, ψ, r, v_y] (7D)
        u_init: Control input [δ_dot, a] (steering rate, acceleration)
        p: Vehicle parameters object
        tire_mode: Tire model ('static_linear', 'dynamic_linear', 'pacejka')
        disturbances: Optional [d_vx, d_vy, d_r] additive disturbances to derivatives
    
    Returns:
        f: State derivatives (7D list)
    

    """
    # Handle NaN inputs
    if np.isnan(x).any():
        return [0.0] * IDX.DIM
    
    # Extract states
    X, Y, delta, vx, psi, r, vy = x
    
    # Vehicle parameters
    lf, lr = p.a, p.b
    m, Iz = p.m, p.I_z
    mu = _get_param(p, 'mu', 0.01)
    Cr_roll = _get_param(p, 'Cr_roll', 0.015)
    Cd_aero = _get_param(p, 'Cd_aero', 0.3)
    Af = _get_param(p, 'Af', 0.05)
    
    # Apply constraints to control inputs
    # u = [
    #     steering_constraints(delta, u_init[0], p.steering),
    #     acceleration_constraints(vx, u_init[1], p.longitudinal)
    # ]
    u = u_init
    
    # Minimum velocity for slip angle calculation
    vx_min = _get_param(p.longitudinal, 'v_switch', 0.5) if hasattr(p, 'longitudinal') else 0.5
    vx_safe = max(abs(vx), vx_min)
    
    # Compute slip angles and tire forces
    alpha_f, alpha_r = _compute_slip_angles(vy, r, delta, lf, lr, vx_safe)
    Fyf, Fyr = compute_tire_forces(alpha_f, alpha_r, p, tire_mode, vx, u[1])
    
    # Precompute trig functions
    cos_psi, sin_psi = math.cos(psi), math.sin(psi)
    cos_delta, sin_delta = math.cos(delta), math.sin(delta)
    
    # Switch to kinematic model at low velocities
    if abs(vx) < VX_KINEMATIC_THRESHOLD:
        f = _kinematic_dynamics(vx, psi, delta, u, lf, lr, m, Cr_roll)
        # Apply disturbances to kinematic model too if applicable?
        # Kinematic model derivatives: [X_dot, Y_dot, delta_dot, vx_dot, psi_dot, r_dot, vy_dot]
        # vx_dot is at index 3
        # vy_dot is 6, r_dot is 5
        if disturbances is not None and len(disturbances) >= 3:
            f[3] += disturbances[0] # vx_dot
            f[6] += disturbances[1] # vy_dot (0 for kin)
            f[5] += disturbances[2] # r_dot (0 for kin)
        return f
    
    # Full dynamic model
    return _dynamic_qlpv(
        vx, vy, psi, r, delta, u, Fyf, Fyr,
        lf, lr, m, Iz, mu, Cr_roll, Cd_aero, Af,
        cos_psi, sin_psi, cos_delta, sin_delta,
        disturbances=disturbances
    )


# =============================================================================
# Vehicle Model Class
# =============================================================================
class QLPVVehicleModel:
    """
    qLPV vehicle dynamics wrapper with tire residual tracking.
    
    Provides:
        - State integration with qLPV dynamics
        - True tire residual computation (ground truth)
        - Measurement generation for observer testing
    
    Attributes:
        state: Current state vector [X, Y, δ, v_x, ψ, r, v_y]
        w_r, w_f: Tire residuals (rear, front)
        Fyf, Fyr: True tire forces
        a_y: Lateral acceleration
    """
    
    def __init__(self, params, sample_time: float = 0.02, tire_mode: str = 'pacejka'):
        """
        Initialize qLPV vehicle model.
        
        Args:
            params: Vehicle parameters object
            sample_time: Integration time step [s]
            tire_mode: Tire model selection
        """
        self.params = params
        self.Ts = sample_time
        self.tire_mode = tire_mode
        
        # Cornering stiffness for residual computation
        self.Cf = _get_param(params, 'Cf', 120.0)
        self.Cr = _get_param(params, 'Cr', 120.0)
        
        # Initialize state and tire variables
        self._reset_variables()
    
    def _reset_variables(self):
        """Reset all state and tire variables to zero."""
        self.state = np.zeros(IDX.DIM)
        self.w_r = self.w_f = 0.0
        self.Fyf = self.Fyr = 0.0
        self.Fyf_linear = self.Fyr_linear = 0.0
        self.a_y = 0.0
        self.alpha_f = self.alpha_r = 0.0
    
    def reset(self, initial_state: np.ndarray):
        """
        Reset vehicle state.
        
        Args:
            initial_state: Initial state (7D qLPV or 6D observer format)
        """
        if len(initial_state) == IDX.DIM:
            self.state = initial_state.copy()
        elif len(initial_state) == 6:
            self.state = state_observer_to_qlpv(initial_state, 0.0)
        else:
            raise ValueError(f"Invalid state dimension: {len(initial_state)}, expected 6 or 7")
        
        # Reset tire variables
        self.w_r = self.w_f = 0.0
        self.Fyf = self.Fyr = 0.0
        self.Fyf_linear = self.Fyr_linear = 0.0
        self.a_y = 0.0
    
    def step(self, control_input: np.ndarray, disturbances: list = None) -> np.ndarray:
        """
        Integrate dynamics for one time step.
        
        Args:
            control_input: Control [δ_dot, a]
            disturbances: Optional [d_vx, d_vy, d_r] additive disturbances
        
        Returns:
            New state vector (7D)
        """
        # Compute and apply derivatives
        f = vehicle_dynamics_qlpv(self.state, control_input, self.params,
                                  tire_mode=self.tire_mode,
                                  disturbances=disturbances)
        self.state += np.array(f) * self.Ts
        
        # Update tire information
        self._update_tire_info(control_input[1])
        
        return self.state.copy()
    
    def _update_tire_info(self, a_long: float = 0.0):
        """
        Update tire forces, residuals, and lateral acceleration.
        
        Args:
            a_long: Longitudinal acceleration for load transfer [m/s²]
        """
        vx = max(abs(self.state[IDX.VX]), VX_MIN_SAFE)
        vy = self.state[IDX.VY]
        r = self.state[IDX.R]
        delta = self.state[IDX.DELTA]
        
        # Slip angles
        self.alpha_f, self.alpha_r = _compute_slip_angles(
            vy, r, delta, self.params.a, self.params.b, vx)
        
        # True tire forces
        self.Fyf, self.Fyr = compute_tire_forces(
            self.alpha_f, self.alpha_r, self.params, self.tire_mode, vx, a_long)
        
        # Linear (reference) tire forces
        self.Fyf_linear, self.Fyr_linear = compute_tire_forces_linear(
            self.alpha_f, self.alpha_r, self.Cf, self.Cr)
        
        # Residuals: w = F_true - F_linear
        self.w_r = self.Fyr - self.Fyr_linear
        self.w_f = self.Fyf - self.Fyf_linear
        
        # Lateral acceleration
        self.a_y = get_lateral_acceleration(self.Fyf, self.Fyr, delta, self.params.m)
    
    # -------------------------------------------------------------------------
    # State Getters
    # -------------------------------------------------------------------------
    def get_true_state(self) -> np.ndarray:
        """Get state in observer format [v_x, v_y, ψ, r, X, Y]."""
        return state_qlpv_to_observer(self.state)
    
    def get_observer_measurement(self) -> np.ndarray:
        """Get measurement vector [v_x, r, ψ, X, Y, a_y]."""
        return np.array([
            self.state[IDX.VX],
            self.state[IDX.R],
            self.state[IDX.PSI],
            self.state[IDX.X],
            self.state[IDX.Y],
            self.a_y,
        ])
    
    # -------------------------------------------------------------------------
    # Tire Information Getters
    # -------------------------------------------------------------------------
    def get_true_residuals(self) -> np.ndarray:
        """Get true tire residuals [w_r, w_f]."""
        return np.array([self.w_r, self.w_f])
    
    def get_true_tire_forces(self) -> np.ndarray:
        """Get true tire forces [Fyr, Fyf]."""
        return np.array([self.Fyr, self.Fyf])
    
    def get_linear_tire_forces(self) -> np.ndarray:
        """Get linear (reference) tire forces [Fyr_linear, Fyf_linear]."""
        return np.array([self.Fyr_linear, self.Fyf_linear])
    
    def get_slip_angles(self) -> np.ndarray:
        """Get slip angles [alpha_r, alpha_f] in radians."""
        return np.array([self.alpha_r, self.alpha_f])
    
    def get_tire_info(self) -> Dict[str, float]:
        """
        Get complete tire information dictionary.
        
        Returns:
            Dict with keys: Fyr_true, Fyf_true, Fyr_linear, Fyf_linear,
                           w_r, w_f, alpha_r, alpha_f
        """
        return {
            'Fyr_true': self.Fyr,
            'Fyf_true': self.Fyf,
            'Fyr_linear': self.Fyr_linear,
            'Fyf_linear': self.Fyf_linear,
            'w_r': self.w_r,
            'w_f': self.w_f,
            'alpha_r': self.alpha_r,
            'alpha_f': self.alpha_f,
        }
