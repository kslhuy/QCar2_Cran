"""
Differentiator + UIO-Style State and W Estimator Observer

Implements a first-layer observer that combines:
1. qLPV-style state estimation from scheduling parameters
2. UIO-style tire residual estimation from filtered differentiation and least-squares

State: x = [v_x, v_y, ψ, r, X, Y]ᵀ (6D)
Unknown inputs: w = [w_r, w_f]ᵀ (tire force residuals)
Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ (6D with lateral acceleration)

Key difference from qLPVAugmentedObserver:
    - w is estimated from residuals each step (UIO-style) rather than as augmented states
    - Uses dirty derivative filtering for rdot estimation
    - Solves w_hat = argmin ||M·w - b||² with ridge regularization

References:
    - Dirty derivative filtering for noisy differentiation
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


class DirtyDerivative:
    """
    Low-pass differentiator using Tustin (bilinear) discretization.
    
    Continuous transfer function:
        ydot = (s / (τs + 1)) * y
    
    Discretized with Tustin (bilinear transform):
        ydot_k = α·ydot_{k-1} + β·(y_k - y_{k-1})
    
    where:
        α = (2τ - Ts) / (2τ + Ts)
        β = 2 / (2τ + Ts)
    
    Args:
        Ts: Sample time [s]
        tau: Filter time constant [s] (larger = more smoothing)
        y0: Initial value of signal
    """
    
    def __init__(self, Ts: float, tau: float, y0: float = 0.0):
        self.Ts = Ts
        self.tau = tau
        
        # Tustin discretization coefficients
        self.alpha = (2*tau - Ts) / (2*tau + Ts)
        self.beta = 2.0 / (2*tau + Ts)
        
        # State variables
        self.y_prev = float(y0)
        self.ydot = 0.0
    
    def update(self, y: float) -> float:
        """
        Update differentiator with new measurement
        
        Args:
            y: Current signal value
            
        Returns:
            Filtered derivative estimate
        """
        y = float(y)
        self.ydot = self.alpha * self.ydot + self.beta * (y - self.y_prev)
        self.y_prev = y
        return self.ydot
    
    def reset(self, y0: float = 0.0):
        """Reset differentiator state"""
        self.y_prev = float(y0)
        self.ydot = 0.0
    
    def get_derivative(self) -> float:
        """Get current derivative estimate"""
        return self.ydot


class HighGainDifferentiator:
    """
    2nd-order High-Gain Observer for differentiation.
    
    State-space form:
        x1 = y_hat (signal estimate)
        x2 = ydot_hat (derivative estimate)
    
    Dynamics (continuous):
        ẋ1 = x2 + L1·(y - x1)
        ẋ2 = L2·(y - x1)
    
    Discretized with Euler:
        x1_{k+1} = x1_k + Ts·(x2_k + L1·(y - x1_k))
        x2_{k+1} = x2_k + Ts·L2·(y - x1_k)
    
    Gain design (pole placement at -ω with damping ζ):
        L1 = 2·ζ·ω
        L2 = ω²
    
    Args:
        Ts: Sample time [s]
        omega: Observer bandwidth [rad/s] (higher = faster but more noise)
        zeta: Damping ratio (0.707 = Butterworth, 1.0 = critically damped)
        y0: Initial signal value
        ydot_max: Maximum derivative magnitude for anti-windup (None = no limit)
    """
    
    def __init__(self, Ts: float = 0.01, omega: float = 30.0, zeta: float = 1.0,
                 y0: float = 0.0, ydot_max: Optional[float] = None):
        self.Ts = Ts
        self.omega = float(omega)
        self.zeta = float(zeta)
        self.ydot_max = ydot_max
        
        # Observer gains (pole placement)
        self.L1 = 2.0 * self.zeta * self.omega
        self.L2 = self.omega ** 2
        
        # State variables
        self.y_hat = float(y0)
        self.ydot_hat = 0.0
    
    def update(self, y: float) -> float:
        """
        Update differentiator with new measurement
        
        Args:
            y: Current signal value
            
        Returns:
            Derivative estimate
        """
        y = float(y)
        e = y - self.y_hat
        
        # Discrete update (Euler)
        self.y_hat = self.y_hat + self.Ts * (self.ydot_hat + self.L1 * e)
        self.ydot_hat = self.ydot_hat + self.Ts * self.L2 * e
        
        # Anti-windup saturation
        if self.ydot_max is not None:
            self.ydot_hat = np.clip(self.ydot_hat, -self.ydot_max, self.ydot_max)
        
        return self.ydot_hat
    
    def reset(self, y0: float = 0.0):
        """Reset differentiator state"""
        self.y_hat = float(y0)
        self.ydot_hat = 0.0
    
    def get_derivative(self) -> float:
        """Get current derivative estimate"""
        return self.ydot_hat
    
    def set_bandwidth(self, omega: float, zeta: Optional[float] = None):
        """
        Update observer bandwidth dynamically
        
        Args:
            omega: New bandwidth [rad/s]
            zeta: New damping ratio (optional, keeps current if None)
        """
        self.omega = float(omega)
        if zeta is not None:
            self.zeta = float(zeta)
        self.L1 = 2.0 * self.zeta * self.omega
        self.L2 = self.omega ** 2


class SlidingModeDifferentiator:
    """
    Super-Twisting Sliding Mode Differentiator (Levant's robust differentiator).
    
    Provides finite-time convergence and robustness to bounded noise/disturbances.
    
    Continuous-time form:
        ẏ_hat = v_hat + k1·|e|^(1/2)·sign(e)
        v̇_hat = k2·sign(e)
        e = y - y_hat
    
    where v_hat converges to ydot.
    
    Discretized with semi-implicit Euler for improved stability:
        y_hat_{k+1} = y_hat_k + Ts·(v_hat_k + k1·|e_k|^0.5·sgn(e_k))
        v_hat_{k+1} = v_hat_k + Ts·k2·sgn(e_k)
    
    Smoothing options for sgn():
        - 'epsilon': sgn_ε(e) = e / (|e| + ε)
        - 'tanh': tanh(e / ε)
        - 'saturation': sat(e / ε)
    
    Gain tuning guidelines:
        - For noise bound L and Lipschitz constant M:
          k1 ≥ 1.5·sqrt(M), k2 ≥ 1.1·M
        - Higher k1, k2 = faster convergence but more noise amplification
    
    Args:
        Ts: Sample time [s]
        k1: First gain (affects convergence rate)
        k2: Second gain (affects robustness)
        epsilon: Smoothing parameter (smaller = more aggressive)
        y0: Initial signal value
        smoothing: Smoothing type ('epsilon', 'tanh', 'saturation')
        v_max: Maximum derivative magnitude for anti-windup
    """
    
    def __init__(self, Ts: float = 0.01, k1: float = 20.0, k2: float = 200.0,
                 epsilon: float = 1e-3, y0: float = 0.0, 
                 smoothing: str = 'epsilon', v_max: Optional[float] = None):
        self.Ts = Ts
        self.k1 = float(k1)
        self.k2 = float(k2)
        self.epsilon = float(epsilon)
        self.smoothing = smoothing
        self.v_max = v_max
        
        # State variables
        self.y_hat = float(y0)
        self.v_hat = 0.0  # This is ydot_hat
    
    def _smooth_sign(self, e: float) -> float:
        """
        Smoothed sign function to reduce chattering
        
        Args:
            e: Error signal
            
        Returns:
            Smoothed sign value in [-1, 1]
        """
        if self.smoothing == 'tanh':
            return np.tanh(e / self.epsilon)
        elif self.smoothing == 'saturation':
            return np.clip(e / self.epsilon, -1.0, 1.0)
        else:  # 'epsilon' (default)
            return e / (abs(e) + self.epsilon)
    
    def update(self, y: float) -> float:
        """
        Update differentiator with new measurement
        
        Args:
            y: Current signal value
            
        Returns:
            Derivative estimate
        """
        y = float(y)
        e = y - self.y_hat
        
        # Smooth sign function
        s = self._smooth_sign(e)
        
        # Super-twisting injection terms
        sqrt_e = np.sqrt(abs(e) + 1e-12)  # Small offset for numerical stability
        inj1 = self.k1 * sqrt_e * s
        inj2 = self.k2 * s
        
        # Semi-implicit Euler discretization (better stability)
        self.y_hat = self.y_hat + self.Ts * (self.v_hat + inj1)
        self.v_hat = self.v_hat + self.Ts * inj2
        
        # Anti-windup saturation
        if self.v_max is not None:
            self.v_hat = np.clip(self.v_hat, -self.v_max, self.v_max)
        
        return self.v_hat
    
    def reset(self, y0: float = 0.0):
        """Reset differentiator state"""
        self.y_hat = float(y0)
        self.v_hat = 0.0
    
    def get_derivative(self) -> float:
        """Get current derivative estimate"""
        return self.v_hat
    
    def set_gains(self, k1: Optional[float] = None, k2: Optional[float] = None):
        """
        Update gains dynamically
        
        Args:
            k1: New first gain (optional)
            k2: New second gain (optional)
        """
        if k1 is not None:
            self.k1 = float(k1)
        if k2 is not None:
            self.k2 = float(k2)


def create_differentiator(diff_type: str = 'dirty', Ts: float = 0.01, 
                          y0: float = 0.0, **kwargs):
    """
    Factory function to create differentiator instances
    
    Args:
        diff_type: Type of differentiator:
            - 'dirty': Low-pass filtered (DirtyDerivative)
            - 'highgain': High-gain observer (HighGainDifferentiator)
            - 'sliding': Super-twisting sliding mode (SlidingModeDifferentiator)
        Ts: Sample time [s]
        y0: Initial signal value
        **kwargs: Additional parameters for specific differentiator type
        
    Returns:
        Differentiator instance
    """
    if diff_type == 'dirty':
        tau = kwargs.get('tau', 0.05)
        return DirtyDerivative(Ts=Ts, tau=tau, y0=y0)
    elif diff_type == 'highgain':
        omega = kwargs.get('omega', 30.0)
        zeta = kwargs.get('zeta', 1.0)
        ydot_max = kwargs.get('ydot_max', None)
        return HighGainDifferentiator(Ts=Ts, omega=omega, zeta=zeta, 
                                       y0=y0, ydot_max=ydot_max)
    elif diff_type == 'sliding':
        k1 = kwargs.get('k1', 20.0)
        k2 = kwargs.get('k2', 200.0)
        epsilon = kwargs.get('epsilon', 1e-3)
        smoothing = kwargs.get('smoothing', 'epsilon')
        v_max = kwargs.get('v_max', None)
        return SlidingModeDifferentiator(Ts=Ts, k1=k1, k2=k2, epsilon=epsilon,
                                          y0=y0, smoothing=smoothing, v_max=v_max)
    else:
        raise ValueError(f"Unknown differentiator type: {diff_type}. "
                        f"Available: 'dirty', 'highgain', 'sliding'")


class WEstimatorUIOStyle:
    """
    UIO-Style Tire Residual Estimator
    
    Estimates w = [w_r, w_f]ᵀ using:
        1) Yaw dynamics residual: rdot - rdot_lin
        2) Lateral acceleration residual: ay - ay_lin
    
    Linear model prediction:
        rdot_lin = f(vx, vy, r, delta, params)
        ay_lin = f(vx, vy, r, delta, params)
    
    Residual equations:
        rdot_res = (-lr/Iz)·w_r + (lf·cos(δ)/Iz)·w_f
        ay_res = (1/m)·w_r + (cos(δ)/m)·w_f
    
    Matrix form: M·w = b
        M = [[-lr/Iz, lf·cos(δ)/Iz],
             [1/m,    cos(δ)/m    ]]
        b = [rdot_res, ay_res]ᵀ
    
    Solved using ridge-regularized least squares.
    
    Args:
        Ts: Sample time [s]
        params: Vehicle parameters dict
        tau_rdot: Time constant for rdot filtering [s] (for dirty derivative)
        ridge: Ridge regularization coefficient
        diff_type: Differentiator type ('dirty', 'highgain', 'sliding')
        diff_params: Additional parameters for the differentiator
    """
    
    def __init__(self, Ts: float, params: Dict, tau_rdot: float = 0.05, 
                 ridge: float = 1e-6, diff_type: str = 'dirty',
                 diff_params: Optional[Dict] = None):
        self.Ts = Ts
        self.p = params
        self.ridge = ridge
        self.diff_type = diff_type
        
        # Create differentiator based on type
        diff_kwargs = diff_params or {}
        if diff_type == 'dirty':
            diff_kwargs.setdefault('tau', tau_rdot)
            self.rdot_filt = DirtyDerivative(Ts=Ts, y0=0.0, **diff_kwargs)
        elif diff_type == 'highgain':
            diff_kwargs.setdefault('omega', 30.0)
            diff_kwargs.setdefault('zeta', 1.0)
            self.rdot_filt = HighGainDifferentiator(Ts=Ts, y0=0.0, **diff_kwargs)
        elif diff_type == 'sliding':
            diff_kwargs.setdefault('k1', 20.0)
            diff_kwargs.setdefault('k2', 200.0)
            diff_kwargs.setdefault('epsilon', 1e-3)
            self.rdot_filt = SlidingModeDifferentiator(Ts=Ts, y0=0.0, **diff_kwargs)
        else:
            raise ValueError(f"Unknown differentiator type: {diff_type}")
        
        # Store latest values for diagnostics
        self.rdot_hat = 0.0
        self.rdot_lin = 0.0
        self.ay_lin = 0.0
        self.residual = np.zeros(2)
    
    def compute_lin_terms(self, xhat: np.ndarray, delta: float) -> Tuple[float, float]:
        """
        Compute linear prediction terms for rdot and ay
        
        Args:
            xhat: State estimate [vx, vy, psi, r, X, Y]
            delta: Steering angle
            
        Returns:
            Tuple of (rdot_lin, ay_lin)
        """
        vx, vy, psi, r, X, Y = xhat
        
        m = self.p["m"]
        Iz = self.p["Iz"]
        lf = self.p["lf"]
        lr = self.p["lr"]
        Cf = self.p["Cf"]
        Cr = self.p["Cr"]
        vx_min = self.p.get("vx_min", 0.5)
        
        vx_eff = max(abs(vx), vx_min)
        c = np.cos(delta)
        
        # rdot linear part (from linear tire model)
        # ṙ = (lf·Cf·cos(δ) - lr·Cr)/Iz · αR - (lf²·Cf·cos(δ) + lr²·Cr)/(Iz·vx) · r
        rdot_lin = (
            - (lf*Cf*c - lr*Cr) / (Iz*vx_eff) * vy
            - (lf**2 * Cf*c + lr**2 * Cr) / (Iz*vx_eff) * r
            + (lf*Cf*c) / Iz * delta
        )
        
        # ay linear part (from lateral dynamics)
        # ay = Fyr/m + Fyf·cos(δ)/m
        ay_lin = (
            - (Cr + Cf*c) / (m*vx_eff) * vy
            + (Cr*lr - Cf*lf*c) / (m*vx_eff) * r
            + (Cf*c) / m * delta
        )
        
        return rdot_lin, ay_lin
    
    def estimate(self, xhat: np.ndarray, r_meas: float, ay_meas: float, 
                 delta: float) -> Tuple[np.ndarray, float, np.ndarray]:
        """
        Estimate tire residuals from measurements and state estimate
        
        Args:
            xhat: State estimate [vx, vy, psi, r, X, Y]
            r_meas: Measured yaw rate (gyro)
            ay_meas: Measured lateral acceleration (IMU)
            delta: Steering angle
            
        Returns:
            Tuple of (w_hat, rdot_hat, residual_vector)
                w_hat: Tire residual estimates [wr, wf]
                rdot_hat: Filtered yaw rate derivative
                residual_vector: [rdot_res, ay_res]
        """
        # 1) Filtered derivative of measured yaw rate
        self.rdot_hat = self.rdot_filt.update(r_meas)
        
        # 2) Linear predicted terms using state estimate
        self.rdot_lin, self.ay_lin = self.compute_lin_terms(xhat, delta)
        
        # 3) Build residual vector b = [rdot_res, ay_res]
        b = np.array([
            self.rdot_hat - self.rdot_lin,
            ay_meas - self.ay_lin
        ], dtype=float)
        self.residual = b.copy()
        
        # 4) Build M(delta) matrix
        m = self.p["m"]
        Iz = self.p["Iz"]
        lf = self.p["lf"]
        lr = self.p["lr"]
        c = np.cos(delta)
        
        M = np.array([
            [-lr/Iz,      lf*c/Iz],
            [ 1.0/m,      c/m    ]
        ], dtype=float)
        
        # 5) Solve w_hat = argmin ||M·w - b||² with ridge regularization
        MtM = M.T @ M + self.ridge * np.eye(2)
        w_hat = np.linalg.solve(MtM, M.T @ b)
        
        return w_hat, self.rdot_hat, b
    
    def reset(self, r0: float = 0.0):
        """Reset estimator state"""
        self.rdot_filt.reset(r0)
        self.rdot_hat = 0.0
        self.rdot_lin = 0.0
        self.ay_lin = 0.0
        self.residual = np.zeros(2)


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
        vx = max(abs(state[0]), min_vx)
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


class DifferentiatorUIOObserver(FirstLayerObserverBase):
    """
    Differentiator + UIO-Style State and Tire Residual Observer
    
    Combines qLPV-style state estimation with UIO-style tire residual estimation.
    
    State vector: x = [v_x, v_y, ψ, r, X, Y]ᵀ
        - v_x: Longitudinal velocity (body frame)
        - v_y: Lateral velocity (body frame)
        - ψ: Yaw angle
        - r: Yaw rate
        - X: Global X position
        - Y: Global Y position
    
    Tire residuals: w = [w_r, w_f]ᵀ (estimated via UIO-style approach)
        - w_r: Rear tire force residual = F_yr - C_r·α_r
        - w_f: Front tire force residual = F_yf - C_f·α_f
    
    Key features:
        - Dirty derivative filtering for rdot estimation
        - Ridge-regularized least squares for w estimation
        - Luenberger-style state observer with innovation feedback
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
                 observer_gains: Optional[Dict] = None,
                 tau_rdot: float = 0.05,
                 ridge: float = 1e-6,
                 diff_type: str = 'dirty',
                 diff_params: Optional[Dict] = None,
                 **kwargs):
        """
        Initialize Differentiator + UIO-Style Observer
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict with keys:
                - 'lf': Distance from CG to front axle [m]
                - 'lr': Distance from CG to rear axle [m]
                - 'm': Vehicle mass [kg]
                - 'Iz': Yaw moment of inertia [kg·m²]
                - 'Cf': Front cornering stiffness [N/rad]
                - 'Cr': Rear cornering stiffness [N/rad]
            observer_gains: Observer gain parameters
            tau_rdot: Time constant for rdot filtering [s] (for dirty derivative)
            ridge: Ridge regularization for w estimation
            diff_type: Differentiator type ('dirty', 'highgain', 'sliding')
            diff_params: Additional parameters for the differentiator
        """
        # Initialize base class with 6D state
        super().__init__(
            state_dim=self.STATE_DIM,
            unknown_input_dim=2,  # [w_r, w_f]
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
        
        # Initialize state estimate
        self.state_hat = np.zeros(self.STATE_DIM)
        
        # UIO-style w estimator with configurable differentiator
        self.w_estimator = WEstimatorUIOStyle(
            Ts=sample_time,
            params=self.params,
            tau_rdot=tau_rdot,
            ridge=ridge,
            diff_type=diff_type,
            diff_params=diff_params
        )
        
        # Tire residual estimates
        self.w_hat = np.zeros(2)  # [w_r, w_f]
        
        # Observer gains
        self.observer_gains = observer_gains or self._default_gains()
        self._initialize_observer_gains()
        
        # Minimum velocity threshold
        self.min_vx = 0.5  # [m/s]
        
        # Diagnostics
        self.rdot_hat = 0.0
        self.residual = np.zeros(2)
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters (QCar scale)"""
        return {
            'lf': 0.11,      # Distance CG to front axle [m]
            'lr': 0.11,      # Distance CG to rear axle [m]
            'm': 3.5,        # Mass [kg]
            'Iz': 0.05,      # Yaw inertia [kg·m²]
            'Cf': 50.0,      # Front cornering stiffness [N/rad]
            'Cr': 50.0,      # Rear cornering stiffness [N/rad]
            'vx_min': 0.5,   # Minimum velocity for numerical stability
        }
    
    def _default_gains(self) -> Dict:
        """Default observer gains"""
        return {
            'L_state': np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5]),  # State observer gains
        }
    
    def _initialize_observer_gains(self):
        """Initialize observer gain matrices"""
        gains = self.observer_gains
        
        # State observer gain (6×6 for state estimation from 6 measurements)
        if isinstance(gains.get('L_state'), np.ndarray) and gains['L_state'].shape == (self.STATE_DIM, self.MEAS_DIM):
            self.L_state = gains['L_state']
        else:
            # Default: diagonal gain
            self.L_state = np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5])
    
    def compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        """Compute scheduling parameters from current state and input"""
        return SchedulingParameters.from_state_and_input(state, delta, self.min_vx)
    
    def compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute state matrix A(ρ) for qLPV system
        
        State: [v_x, v_y, ψ, r, X, Y]
        """
        A = np.zeros((self.STATE_DIM, self.STATE_DIM))
        
        inv_vx = rho.inv_vx
        cos_d = rho.cos_delta
        sin_d = rho.sin_delta
        vx = rho.vx
        vy = rho.vy
        cos_psi = rho.cos_psi
        sin_psi = rho.sin_psi
        
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
        A[4, 2] = -vx * sin_psi - vy * cos_psi
        
        # Y dynamics: Ẏ = v_x·sin(ψ) + v_y·cos(ψ)
        A[5, 0] = sin_psi
        A[5, 1] = cos_psi
        A[5, 2] = vx * cos_psi - vy * sin_psi
        
        return A
    
    def compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute input matrix B(ρ)
        
        Input: u = [δ, a]ᵀ (steering, acceleration)
        """
        B = np.zeros((self.STATE_DIM, 2))
        
        cos_d = rho.cos_delta
        
        # v_x: affected by acceleration
        B[0, 1] = 1.0  # Direct acceleration
        
        # v_y: affected by steering
        B[1, 0] = self.Cf * cos_d / self.m
        
        # r: affected by steering
        B[3, 0] = self.Cf * self.lf * cos_d / self.Iz
        
        return B
    
    def compute_E_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """
        Compute residual injection matrix E(ρ)
        
        Residual: w = [w_r, w_f]ᵀ
        """
        E = np.zeros((self.STATE_DIM, 2))
        
        cos_d = rho.cos_delta
        
        # v_y: both residuals contribute
        E[1, 0] = 1.0 / self.m
        E[1, 1] = cos_d / self.m
        
        # r: both residuals create moment
        E[3, 0] = -self.lr / self.Iz
        E[3, 1] = self.lf * cos_d / self.Iz
        
        return E
    
    def compute_C_matrix(self) -> np.ndarray:
        """
        Compute output matrix C
        
        Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ
        """
        C = np.zeros((self.MEAS_DIM, self.STATE_DIM))
        
        # Direct measurements
        C[self.MEAS_IDX_VX, self.IDX_VX] = 1.0
        C[self.MEAS_IDX_R, self.IDX_R] = 1.0
        C[self.MEAS_IDX_PSI, self.IDX_PSI] = 1.0
        C[self.MEAS_IDX_X, self.IDX_X] = 1.0
        C[self.MEAS_IDX_Y, self.IDX_Y] = 1.0
        # a_y is handled separately in update
        
        return C
    
    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update observer with new measurement
        
        Observer equation:
            x̂[k+1] = x̂[k] + Ts·(A(ρ̂)·x̂ + B(ρ̂)·u + E(ρ̂)·ŵ + L·(y - C·x̂))
        
        UIO-style w estimation:
            ŵ = argmin ||M·w - b||² where b = [rdot_res, ay_res]
        
        Args:
            measurement: Measurement vector (various formats supported)
            control_input: Control [δ, a] (steering, acceleration)
            f_nn: Neural network output (for interface compatibility)
            acceleration: Full 3D acceleration [a_x, a_y, a_z]
            
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
        
        # Get system matrices
        A = self.compute_A_matrix(rho)
        B = self.compute_B_matrix(rho)
        E = self.compute_E_matrix(rho)
        C = self.compute_C_matrix()
        
        # === UIO-Style w estimation ===
        r_meas = y[self.MEAS_IDX_R]
        ay_meas = y[self.MEAS_IDX_AY]
        self.w_hat, self.rdot_hat, self.residual = self.w_estimator.estimate(
            self.state_hat, r_meas, ay_meas, delta
        )
        
        # === State observer update ===
        # Predicted output (without a_y for simplicity)
        y_pred = C @ self.state_hat
        
        # Innovation (measurement residual)
        innovation = y - y_pred
        # Set a_y innovation to 0 since it's used for w estimation
        innovation[self.MEAS_IDX_AY] = 0.0
        
        # State dynamics (continuous-time)
        x_dot = A @ self.state_hat + B @ u + E @ self.w_hat + self.L_state @ innovation
        
        # Discrete update (Euler integration)
        self.state_hat = self.state_hat + self.Ts * x_dot
        
        # Copy to base class attribute for interface compatibility
        self.f_uk_hat = self.w_hat.copy()
        
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
                y[5] = acceleration[1]  # Extract a_y
            else:
                y[5] = self.state_hat[self.IDX_R] * self.state_hat[self.IDX_VX]
        elif len(measurement) == 2:
            # Minimal: [v_x, r]
            y[self.MEAS_IDX_VX] = measurement[0]
            y[self.MEAS_IDX_R] = measurement[1]
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
    
    def get_unknown_input(self) -> np.ndarray:
        """Get unknown input estimate (alias for tire residuals)"""
        return self.w_hat.copy()
    
    def get_rdot_estimate(self) -> float:
        """Get filtered yaw rate derivative"""
        return self.rdot_hat
    
    def get_residual_vector(self) -> np.ndarray:
        """Get residual vector [rdot_res, ay_res] used for w estimation"""
        return self.residual.copy()
    
    def reset(self, initial_state: Optional[np.ndarray] = None, 
              initial_position: Optional[np.ndarray] = None):
        """
        Reset observer state
        
        Args:
            initial_state: Initial state [v_x, v_y, ψ, r, X, Y] or partial
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
        
        # Reset w estimator
        r0 = self.state_hat[self.IDX_R] if initial_state is not None else 0.0
        self.w_estimator.reset(r0)
        
        # Reset tire residuals
        self.w_hat = np.zeros(2)
        self.f_uk_hat = np.zeros(2)
        
        # Reset diagnostics
        self.rdot_hat = 0.0
        self.residual = np.zeros(2)


def create_differentiator_uio_observer(sample_time: float = 0.02,
                                        vehicle_params: Optional[Dict] = None,
                                        **kwargs) -> DifferentiatorUIOObserver:
    """
    Factory function to create Differentiator + UIO-Style observer
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dictionary
        **kwargs: Additional arguments passed to constructor
        
    Returns:
        Configured DifferentiatorUIOObserver instance
    """
    return DifferentiatorUIOObserver(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        **kwargs
    )
