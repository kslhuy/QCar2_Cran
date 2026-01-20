"""
qLPV Vehicle Dynamics Model for Observer Testing

This vehicle dynamics model exactly matches the qLPV observer equations,
enabling proper testing of the observer's state and tire residual estimation.

State: x = [X, Y, δ, v_x, ψ, r, v_y]ᵀ (7D, CommonRoad convention)
       Indices: 0=X, 1=Y, 2=δ, 3=vx, 4=ψ, 5=r, 6=vy

Observer State: [v_x, v_y, ψ, r, X, Y] (6D)
                Mapping: x[3]=vx, x[6]=vy, x[4]=ψ, x[5]=r, x[0]=X, x[1]=Y

Key difference from observer:
- Fake vehicle KNOWS true tire forces and residuals (w_r, w_f)
- Observer ESTIMATES tire residuals from measurements

Control input: u = [δ_dot, a]ᵀ (steering rate, acceleration)

References:
    - qLPV vehicle dynamics from qlpv_observer.py
    - CommonRoad vehicle model conventions
"""
__author__ = "Observer Test Framework"
__version__ = "1.0"
__status__ = "Development"

import math
import numpy as np
from typing import Tuple, Optional


# Apply steering and acceleration constraints
from .utils.steering_constraints import steering_constraints
from .utils.acceleration_constraints import acceleration_constraints

def vehicle_dynamics_qlpv(x, u_init, p,  tire_mode: str = 'pacejka' ):
    """
    qLPV vehicle dynamics matching observer model
    
    This model uses the EXACT same equations as the qLPV observer,
    but can optionally compute true nonlinear tire forces for ground truth.
    
    Syntax:
        f = vehicle_dynamics_qlpv(x, u, p)
    
    Inputs:
        :param x: vehicle state vector [X, Y, δ, v_x, ψ, r, v_y]
        :param u_init: vehicle input vector [δ_dot, a]
        :param p: vehicle parameter object
        :param tire_mode: Tire model selection: 'static_linear', 'dynamic_linear', 'pacejka'
                         If None, falls back to use_pacejka for backward compatibility
    
    Outputs:
        :return f: right-hand side of differential equations (7D)
    
    State definitions match CommonRoad convention:
        x[0] = X     - x-position in global coordinate system [m]
        x[1] = Y     - y-position in global coordinate system [m]
        x[2] = δ     - steering angle of front wheels [rad]
        x[3] = v_x   - velocity in x-direction (body frame) [m/s]
        x[4] = ψ     - yaw angle [rad]
        x[5] = r     - yaw rate [rad/s]
        x[6] = v_y   - lateral velocity (body frame) [m/s]
    """
    # Gravity constant
    g = 9.81  # [m/s^2]
    
    # Extract vehicle parameters
    lf = p.a           # Distance from CG to front axle [m]
    lr = p.b           # Distance from CG to rear axle [m]
    m = p.m            # Vehicle mass [kg]
    Iz = p.I_z         # Yaw moment of inertia [kg·m²]
    
    # Get cornering stiffness from parameters or use defaults
    Cf = getattr(p, 'Cf', 120.0)  # Increased from 50.0 for better grip
    Cr = getattr(p, 'Cr', 120.0)  # Increased from 50.0 for better grip
    
    # Road friction coefficient for load transfer effects (small value)
    mu = getattr(p, 'mu', 0.01)  # Road friction coefficient 
    
    # Rolling resistance and drag coefficients for realistic stopping behavior
    Cr_roll = getattr(p, 'Cr_roll', 0.015)  # Rolling resistance coefficient (typical: 0.01-0.02)
    Cd_aero = getattr(p, 'Cd_aero', 0.3)    # Aerodynamic drag coefficient
    Af = getattr(p, 'Af', 0.05)             # Frontal area [m^2] (scaled for QCar)
    rho_air = 1.225                          # Air density [kg/m³]
    
    # Extract states
    X, Y, delta, vx, psi, r, vy = x
    
    # Robustness: Handle potential NaNs by returning zero derivatives
    if np.isnan(x).any():
        return [0.0] * 7

    
    u = list()
    u.append(steering_constraints(delta, u_init[0], p.steering))
    u.append(acceleration_constraints(vx, u_init[1], p.longitudinal))
    
    # Minimum velocity to avoid singularity (from parameters_qcar.yaml longitudinal.v_switch)
    vx_min = getattr(p.longitudinal, 'v_switch', 0.5) if hasattr(p, 'longitudinal') else 0.5
    vx_safe = max(abs(vx), vx_min)
    
    # Compute slip angles
    # α_f = δ - (v_y + l_f·r) / v_x
    # α_r = -(v_y - l_r·r) / v_x
    alpha_f = delta - (vy + lf * r) / vx_safe
    alpha_r = -(vy - lr * r) / vx_safe
    
    # Compute tire forces
    a_long = u[1]
    

    
    if tire_mode == 'static_linear':
        # Simple linear model: F = C * alpha (no load transfer)
        Fyf, Fyr = compute_tire_forces_linear(alpha_f, alpha_r, Cf, Cr)
    elif tire_mode == 'dynamic_linear':
        # Linear model with load transfer
        Fyf, Fyr = compute_tire_forces_load_transfer(alpha_f, alpha_r, p, a_long)
    elif tire_mode == 'pacejka' and hasattr(p, 'tire'):
        # Pacejka nonlinear tire model
        Fyf, Fyr = compute_tire_forces_pacejka(alpha_f, alpha_r, p, vx, a_long)
    else:
        # Fallback to dynamic linear if pacejka requested but no tire params
        Fyf, Fyr = compute_tire_forces_load_transfer(alpha_f, alpha_r, p, a_long)
    
    # Trigonometric functions
    cos_psi = math.cos(psi)
    sin_psi = math.sin(psi)
    cos_delta = math.cos(delta)
    sin_delta = math.sin(delta)
    
    # Switch to kinematic model for very low velocities
    if abs(vx) < 0.1:
        # Simplified kinematic model
        lwb = lf + lr  # wheelbase
        beta = math.atan2(lr * math.tan(delta), lwb) if abs(delta) > 0.001 else 0.0
        
        # Apply friction even in kinematic mode for proper stopping
        F_roll_kin = m * g * Cr_roll
        if abs(vx) > 0.01:
            friction_decel_kin = F_roll_kin / m * np.sign(vx)
        else:
            friction_decel_kin = 0.0
        
        # Velocity derivative with friction (stops the vehicle)
        vx_dot_kin = u[1] - friction_decel_kin
        
        # Force complete stop when very slow and no acceleration commanded
        if abs(vx) < 0.02 and abs(u[1]) < 0.01:
            vx_dot_kin = -vx / 0.1  # Damp to zero quickly
        
        f = [
            vx * math.cos(psi + beta),          # Ẋ
            vx * math.sin(psi + beta),          # Ẏ
            u[0],                                # δ̇
            vx_dot_kin,                          # v̇_x (with friction)
            vx / lwb * math.tan(delta) * math.cos(beta) if abs(vx) > 0.01 else 0.0,  # ψ̇
            0.0,                                 # ṙ (simplified)
            0.0,                                 # v̇_y (simplified)
        ]
    else:
        # Full qLPV dynamics (matching observer equations)
        
        # Aerodynamic drag: F_drag = 0.5 * rho * Cd * Af * vx^2 (opposes motion)
        # Sign-preserving drag with safety clamp to prevent overflow
        F_roll = m * g * Cr_roll
        vx_clamped = np.clip(vx, -100.0, 100.0) # Safety clamp for drag calculation only
        F_drag = 0.5 * rho_air * Cd_aero * Af * vx_clamped * abs(vx_clamped) 
        
        # Friction force direction (opposes velocity)
        if abs(vx) > 0.01:  # Only apply when moving
            friction_decel = (F_roll + F_drag) / m * np.sign(vx)
        else:
            friction_decel = 0.0
            # Also damp out residual velocity when nearly stopped
            if abs(vx) < 0.05:
                vx = 0.0  # Stop completely when very slow
        
        # v̇_x = a - μg + r·v_y - (F_yf·sin(δ))/m - friction_decel
        vx_dot = u[1] - mu * g + r * vy - (Fyf * sin_delta) / m - friction_decel
        
        # v̇_y = (F_yr + F_yf·cos(δ))/m - r·v_x
        vy_dot = (Fyr + Fyf * cos_delta) / m - r * vx
        
        # ψ̇ = r
        psi_dot = r
        
        # ṙ = (l_f·F_yf·cos(δ) - l_r·F_yr) / I_z
        r_dot = (lf * Fyf * cos_delta - lr * Fyr) / Iz
        
        # Ẋ = v_x·cos(ψ) - v_y·sin(ψ)
        X_dot = vx * cos_psi - vy * sin_psi
        
        # Ẏ = v_x·sin(ψ) + v_y·cos(ψ)
        Y_dot = vx * sin_psi + vy * cos_psi
        
        # State derivatives in CommonRoad order
        f = [
            X_dot,      # x[0] = X
            Y_dot,      # x[1] = Y
            u[0],       # x[2] = δ (steering rate)
            vx_dot,     # x[3] = v_x
            psi_dot,    # x[4] = ψ
            r_dot,      # x[5] = r
            vy_dot,     # x[6] = v_y
        ]
    
    return f


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
        Tuple of (Fyf, Fyr) - lateral tire forces [N]
    """
    Fyf = Cf * alpha_f
    Fyr = Cr * alpha_r
    return Fyf, Fyr


def compute_tire_forces_load_transfer(alpha_f: float, alpha_r: float,
                                       p, a_long: float) -> Tuple[float, float]:
    """
    New tire model based on dynamic load transfer and normalized stiffness.
    
    Formula: Fy,i = mu * Cs,i * Fz,i * alpha_i
    
    Load transfer:
        Fzf = (m*g*lr - m*a_long*h_cg) / (lf + lr)
        Fzr = (m*g*lf + m*a_long*h_cg) / (lf + lr)
    """
    g = 9.81
    m = p.m
    lf = p.a
    lr = p.b
    h_cg = getattr(p, 'h_cg', 0.07) # Center of gravity height
    mu_road = getattr(p, 'mu_road', 1.0) # Friction coefficient
    
    # Normalized cornering stiffness (N/N/rad)
    Csf = getattr(p, 'Csf', 30.0) 
    Csr = getattr(p, 'Csr', 30.0)

    # Vertical forces with longitudinal load transfer
    Fzf = (m * g * lr - m * a_long * h_cg) / (lf + lr)
    Fzr = (m * g * lf + m * a_long * h_cg) / (lf + lr)
    
    # Non-negative check
    Fzf = max(Fzf, 0.0)
    Fzr = max(Fzr, 0.0)
    
    # Dynamic Cornering Stiffness (Ci = mu_road * Cs_i * Fz_i)
    Cf_dyn = mu_road * Csf * Fzf
    Cr_dyn = mu_road * Csr * Fzr
    
    # Lateral tire forces
    Fyf = Cf_dyn * alpha_f
    Fyr = Cr_dyn * alpha_r
    
    return Fyf, Fyr


def compute_tire_forces_pacejka(alpha_f: float, alpha_r: float,
                                 p, vx: float, a_long: float = 0.0) -> Tuple[float, float]:
    """
    Pacejka tire model with longitudinal load transfer
    
    Args:
        alpha_f: Front slip angle [rad]
        alpha_r: Rear slip angle [rad]
        p: Vehicle parameters with tire sub-object
        vx: Longitudinal velocity [m/s]
        a_long: Longitudinal acceleration for load transfer [m/s^2]

    
    Returns:
        Tuple of (Fyf, Fyr) - lateral tire forces [N]
    """
    g = 9.81
    m = p.m
    lf = p.a
    lr = p.b
    h_cg = getattr(p, 'h_cg', 0.1)
    
    # Tire parameters
    tire = p.tire
    mu = tire.p_dy1       # Peak friction coefficient
    Cy = tire.p_cy1       # Shape factor C
    Ky = tire.p_ky1       # Stiffness factor for B computation
    Ey = tire.p_ey1       # Curvature factor E
    
    # Compute normal forces with dynamic load transfer
    Fz_f = (m * g * lr - m * a_long * h_cg) / (lf + lr)
    Fz_r = (m * g * lf + m * a_long * h_cg) / (lf + lr)
    
    # Ensure loads are non-negative
    Fz_f = max(Fz_f, 0.01)
    Fz_r = max(Fz_r, 0.01)
    
    # Peak lateral force
    Dyf = mu * Fz_f
    Dyr = mu * Fz_r
    
    # Stiffness factor B = Ky / (Cy * Dy)
    Byf = -Ky / (Cy * Dyf) if abs(Dyf) > 0.1 else 20.0
    Byr = -Ky / (Cy * Dyr) if abs(Dyr) > 0.1 else 20.0
    
    # Magic formula: F = D * sin(C * arctan(B*α - E*(B*α - arctan(B*α))))
    def magic_formula(alpha, B, C, D, E):
        Ba = B * alpha
        Fy = D * math.sin(C * math.atan(Ba - E * (Ba - math.atan(Ba))))
        return Fy
    
    Fyf = magic_formula(alpha_f, Byf, Cy, Dyf, Ey)
    Fyr = magic_formula(alpha_r, Byr, Cy, Dyr, Ey)
    
    return Fyf, Fyr


def get_tire_residuals(alpha_f: float, alpha_r: float,
                       Cf: float, Cr: float, p,
                       vx: float = 1.0, a_long: float = 0.0) -> Tuple[float, float]:
    """
    Compute tire residuals (ground truth for observer testing)
    
    w_r = F_yr_pacejka - F_yr_linear = F_yr_true - C_r * α_r
    w_f = F_yf_pacejka - F_yf_linear = F_yf_true - C_f * α_f
    
    These are the TRUE residuals that the observer should estimate.
    
    Args:
        alpha_f: Front slip angle [rad]
        alpha_r: Rear slip angle [rad]
        Cf: Front cornering stiffness [N/rad]
        Cr: Rear cornering stiffness [N/rad]
        p: Vehicle parameters with tire sub-object
        vx: Longitudinal velocity [m/s]
        a_long: Longitudinal acceleration for load transfer [m/s^2]
    
    Returns:
        Tuple of (w_r, w_f) - tire force residuals [N]
    """
    # Linear tire forces (Reference model used by observer)
    Fyf_linear, Fyr_linear = compute_tire_forces_linear(alpha_f, alpha_r, Cf, Cr)
    
    # True nonlinear tire forces (Truth model)
    if hasattr(p, 'tire'):
        Fyf_true, Fyr_true = compute_tire_forces_pacejka(alpha_f, alpha_r, p, vx, a_long)
    else:
        # If no Pacejka, use the dynamic linear model as truth
        Fyf_true, Fyr_true = compute_tire_forces_load_transfer(alpha_f, alpha_r, p, a_long)
    
    # Residuals = True - Linear
    w_r = Fyr_true - Fyr_linear
    w_f = Fyf_true - Fyf_linear
    
    return w_r, w_f


def get_lateral_acceleration(Fyf: float, Fyr: float, delta: float,
                              m: float) -> float:
    """
    Compute lateral acceleration for observer measurement
    
    a_y = (F_yr + F_yf * cos(δ)) / m
    
    Args:
        Fyf: Front lateral tire force [N]
        Fyr: Rear lateral tire force [N]
        delta: Steering angle [rad]
        m: Vehicle mass [kg]
    
    Returns:
        a_y: Lateral acceleration [m/s²]
    """
    return (Fyr + Fyf * math.cos(delta)) / m


def state_qlpv_to_observer(x_qlpv: np.ndarray) -> np.ndarray:
    """
    Convert qLPV state vector to observer state order
    
    qLPV:     [X, Y, δ, v_x, ψ, r, v_y] (7D)
    Observer: [v_x, v_y, ψ, r, X, Y]    (6D)
    
    Args:
        x_qlpv: qLPV state vector (7D)
    
    Returns:
        Observer state vector (6D)
    """
    return np.array([
        x_qlpv[3],  # v_x
        x_qlpv[6],  # v_y
        x_qlpv[4],  # ψ
        x_qlpv[5],  # r
        x_qlpv[0],  # X
        x_qlpv[1],  # Y
    ])


def state_observer_to_qlpv(x_obs: np.ndarray, delta: float = 0.0) -> np.ndarray:
    """
    Convert observer state vector to qLPV state order
    
    Observer: [v_x, v_y, ψ, r, X, Y]    (6D)
    qLPV:     [X, Y, δ, v_x, ψ, r, v_y] (7D)
    
    Args:
        x_obs: Observer state vector (6D)
        delta: Steering angle to include [rad]
    
    Returns:
        qLPV state vector (7D)
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


class QLPVVehicleModel:
    """
    Class wrapper for qLPV vehicle dynamics with tire residual tracking
    
    This class maintains the vehicle state and provides:
    - State integration with qLPV dynamics
    - True tire residual computation (ground truth)
    - Measurement generation for observer testing
    """
    
    # State indices (CommonRoad convention)
    IDX_X = 0
    IDX_Y = 1
    IDX_DELTA = 2
    IDX_VX = 3
    IDX_PSI = 4
    IDX_R = 5
    IDX_VY = 6
    STATE_DIM = 7
    
    def __init__(self, params, sample_time: float = 0.02,
                  tire_mode= 'pacejka'):
        """
        Initialize qLPV vehicle model
        
        Args:
            params: Vehicle parameters object
            sample_time: Integration time step [s]
            tire_mode: Tire model selection: 'static_linear', 'dynamic_linear', 'pacejka'
        """
        self.params = params
        self.Ts = sample_time
        self.tire_mode = tire_mode
        # Extract cornering stiffness for residual computation
        self.Cf = getattr(params, 'Cf', 120.0)
        self.Cr = getattr(params, 'Cr', 120.0)
        
        # State vector [X, Y, δ, v_x, ψ, r, v_y]
        self.state = np.zeros(self.STATE_DIM)
        
        # True tire residuals (ground truth)
        self.w_r = 0.0  # Rear tire residual
        self.w_f = 0.0  # Front tire residual
        
        # True tire forces
        self.Fyf = 0.0  # Front lateral force
        self.Fyr = 0.0  # Rear lateral force
        
        # Lateral acceleration (for observer measurement)
        self.a_y = 0.0
        
        # Slip angles
        self.alpha_f = 0.0
        self.alpha_r = 0.0
    
    def reset(self, initial_state: np.ndarray):
        """
        Reset vehicle state
        
        Args:
            initial_state: Initial state vector (7D qLPV or 6D observer format)
        """
        if len(initial_state) == 7:
            self.state = initial_state.copy()
        elif len(initial_state) == 6:
            # Convert from observer format
            self.state = state_observer_to_qlpv(initial_state, 0.0)
        else:
            raise ValueError(f"Invalid state dimension: {len(initial_state)}")
        
        self.w_r = 0.0
        self.w_f = 0.0
        self.Fyf = 0.0
        self.Fyr = 0.0
        self.a_y = 0.0
    
    def step(self, control_input: np.ndarray) -> np.ndarray:
        """
        Integrate vehicle dynamics for one time step
        
        Args:
            control_input: Control [δ_dot, a] (steering rate, acceleration)
        
        Returns:
            New state vector (7D)
        """
        # Compute derivatives
        f = vehicle_dynamics_qlpv(self.state, control_input, self.params,
                                   tire_mode=self.tire_mode)
        
        # Euler integration
        for i in range(self.STATE_DIM):
            self.state[i] += f[i] * self.Ts
        
        # Update tire forces and residuals with dynamic load transfer
        self._update_tire_info(control_input[1])
        
        return self.state.copy()
    
    def _update_tire_info(self, a_long: float = 0.0):
        """
        Update tire forces, residuals, and lateral acceleration
        
        Args:
            a_long: Longitudinal acceleration for load transfer [m/s^2]
        """
        vx = max(abs(self.state[self.IDX_VX]), 0.5)
        vy = self.state[self.IDX_VY]
        r = self.state[self.IDX_R]
        delta = self.state[self.IDX_DELTA]
        
        lf = self.params.a
        lr = self.params.b
        
        # Slip angles
        self.alpha_f = delta - (vy + lf * r) / vx
        self.alpha_r = -(vy - lr * r) / vx
        
        # True tire forces (Truth model)
        if self.tire_mode == 'pacejka' and hasattr(self.params, 'tire'):
            self.Fyf, self.Fyr = compute_tire_forces_pacejka(
                self.alpha_f, self.alpha_r, self.params, vx, a_long)
        elif self.tire_mode == 'static_linear':
            self.Fyf, self.Fyr = compute_tire_forces_load_transfer(
                self.alpha_f, self.alpha_r, self.params, vx, a_long)
        elif self.tire_mode == 'dynamic_linear':
            self.Fyf, self.Fyr = compute_tire_forces_linear(
                self.alpha_f, self.alpha_r, self.params, vx, a_long)
        
        # Tire residuals (ground truth)
        # Compares Truth model with Static Linear reference (Cf, Cr)
        self.w_r, self.w_f = get_tire_residuals(
            self.alpha_f, self.alpha_r, self.Cf, self.Cr, self.params, vx, a_long)
        
        # Lateral acceleration
        self.a_y = get_lateral_acceleration(
            self.Fyf, self.Fyr, delta, self.params.m)
    
    def get_observer_state(self) -> np.ndarray:
        """Get state in observer format [v_x, v_y, ψ, r, X, Y]"""
        return state_qlpv_to_observer(self.state)
    
    def get_observer_measurement(self) -> np.ndarray:
        """
        Get measurement vector for observer [v_x, r, ψ, X, Y, a_y]
        
        Returns:
            Measurement vector (6D)
        """
        return np.array([
            self.state[self.IDX_VX],   # v_x
            self.state[self.IDX_R],    # r
            self.state[self.IDX_PSI],  # ψ
            self.state[self.IDX_X],    # X
            self.state[self.IDX_Y],    # Y
            self.a_y,                   # a_y
        ])
    
    def get_true_residuals(self) -> np.ndarray:
        """Get true tire residuals [w_r, w_f]"""
        return np.array([self.w_r, self.w_f])
    
    def get_true_tire_forces(self) -> np.ndarray:
        """Get true tire forces [Fyr, Fyf]"""
        return np.array([self.Fyr, self.Fyf])
