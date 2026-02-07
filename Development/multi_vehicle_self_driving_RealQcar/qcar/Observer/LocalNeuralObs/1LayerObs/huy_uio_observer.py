"""
Generalized Unknown Input Observer (UIO) for First-Layer Observer Architecture

Implements a Generalized UIO (Teacher) for vehicle state estimation with unknown
tire force residual estimation. Based on the descriptor-form generalized observer
structure for systems with unknown inputs.

State: x = [v_x, v_y, ψ, r, X, Y]ᵀ (6D)
Unknown inputs: w = [w_r, w_f]ᵀ (tire force residuals)
Measurements: Mode-dependent
    - gps_on:  y = [v_x, r, ψ, X, Y]ᵀ (5D)
    - gps_off: y = [v_x, r, ψ]ᵀ (3D)

Observer Structure (Generalized Descriptor Form):
    E(ζ)·ζ̇ = φ_ζ(ζ̂, u)
    y_χ = H(ζ)·ζ
    η̇ = P_ζ·φ_ζ(ζ̂, u) + K·(y_χ - H·ζ̂)
    ζ̂ = η + Q_ζ·y_χ

where P_ζ·E + Q_ζ·H = I (left inverse via pseudoinverse).

The augmented descriptor state ζ includes:
    - χ₁: Output and its derivative states
    - χ₂: Higher-order output derivatives
    - x: Vehicle state
    - w: Unknown inputs (tire residuals)

References:
    - Generalized UIO for descriptor systems with unknown inputs
    - qLPV vehicle dynamics with tire-residual estimation
"""

import numpy as np
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
import sys
from pathlib import Path

# Import CVXPY for LMI-based gain design (optional)
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False

# Setup path for imports
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

# Import base class
try:
    from firstLayerObserverBase import FirstLayerObserverBase
except ImportError:
    from abc import ABC, abstractmethod
    class FirstLayerObserverBase(ABC):
        def __init__(self, state_dim: int = 6, unknown_input_dim: int = 2, sample_time: float = 0.02):
            self.state_dim = state_dim
            self.unknown_input_dim = unknown_input_dim
            self.Ts = sample_time
            self.state_hat = np.zeros(state_dim)
            self.f_uk_hat = np.zeros(unknown_input_dim)
        
        @abstractmethod
        def update(self, measurement, control_input, f_nn=None, acceleration=None, gps_available=True):
            pass

# Import centralized qLPV vehicle dynamics
sys.path.insert(0, str(parent_dir.parent))
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    create_qlpv_dynamics,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
)


# =============================================================================
# Utility Functions
# =============================================================================

def wrap_to_pi(angle_rad: float) -> float:
    """Wrap angle to (-π, π]."""
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


# =============================================================================
# LMI-based gain design helpers (optional, requires CVXPY)
# =============================================================================

def validate_observer_gain(A: np.ndarray, C: np.ndarray, L: np.ndarray,
                           max_real_part: float = -0.1) -> bool:
    """
    Validate that observer gain produces stable error dynamics.
    """
    try:
        A_cl = A - L @ C
        eigenvalues = np.linalg.eigvals(A_cl)
        max_eig_real = np.max(np.real(eigenvalues))
        return max_eig_real < max_real_part
    except Exception:
        return False


def compute_lmi_observer_gain(A: np.ndarray, C: np.ndarray,
                              decay_rate: float = 1.0,
                              verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using Lyapunov LMI.

    Solve for P > 0 and Y = P @ L such that:
        A^T P + P A - C^T Y^T - Y C + decay_rate * P < 0
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available. Install with: pip install cvxpy")

    n = A.shape[0]
    m = C.shape[0]

    P = cp.Variable((n, n), symmetric=True)
    Y = cp.Variable((n, m))

    eps = 1e-6
    lmi_constraint = A.T @ P + P @ A - C.T @ Y.T - Y @ C + decay_rate * P

    constraints = [
        P >> eps * np.eye(n),
        P << 1e4 * np.eye(n),
        lmi_constraint << -eps * np.eye(n),
    ]

    objective = cp.Minimize(cp.trace(P) + 0.01 * cp.norm(Y, "fro"))
    problem = cp.Problem(objective, constraints)

    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=10000, eps=1e-6)
    except Exception as e:
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except Exception:
            raise ValueError(f"LMI solver failed: {e}")

    if problem.status not in ["optimal", "optimal_inaccurate"]:
        raise ValueError(f"LMI problem infeasible. Status: {problem.status}")

    P_val = P.value
    Y_val = Y.value
    if P_val is None or Y_val is None:
        raise ValueError("LMI solver returned None values")

    try:
        L = np.linalg.solve(P_val, Y_val)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val

    if not validate_observer_gain(A, C, L):
        raise ValueError("Computed gain does not produce stable observer")

    return L


# =============================================================================
# HuyUIOObserver - Generalized UIO for First Layer Architecture
# =============================================================================

class HuyUIOObserver(FirstLayerObserverBase):
    """
    Generalized Unknown Input Observer (UIO) for Vehicle State Estimation
    
    Implements the descriptor-form generalized observer structure for estimating
    both vehicle states and unknown tire force residuals. Supports GPS on/off
    modes with dynamic measurement matrix sizing.
    
    State vector: x = [v_x, v_y, ψ, r, X, Y]ᵀ
        - v_x: Longitudinal velocity (body frame)
        - v_y: Lateral velocity (body frame)
        - ψ: Yaw angle
        - r: Yaw rate
        - X: Global X position
        - Y: Global Y position
    
    Tire residuals: w = [w_r, w_f]ᵀ
        - w_r: Rear tire force residual
        - w_f: Front tire force residual
    
    Measurement modes:
        - GPS on:  y = [v_x, r, ψ, X, Y]ᵀ (5D)
        - GPS off: y = [v_x, r, ψ]ᵀ (3D)
    
    Key Features:
        - Descriptor-form observer for systems with unknown inputs
        - Mode-switching for GPS availability
        - Yaw angle wrapping in innovation computation
        - Integration with centralized qLPV vehicle dynamics
    """
    
    # State indices (from centralized module)
    IDX_VX = IDX_VX
    IDX_VY = IDX_VY
    IDX_PSI = IDX_PSI
    IDX_R = IDX_R
    IDX_X = IDX_X
    IDX_Y = IDX_Y
    STATE_DIM = STATE_DIM
    
    # Unknown input dimension
    UNKNOWN_INPUT_DIM = 2  # [w_r, w_f]
    
    def __init__(
        self,
        sample_time: float = 0.02,
        vehicle_params: Optional[Dict] = None,
        delta2: float = 1e-3,
        use_y_dot_meas: bool = False,
        observer_gain_scale: float = 1.0,
        use_lmi_gain: bool = True,
        lmi_decay_rate: float = 0.5,
        lmi_recompute_interval: int = 0,
        lmi_max_gain: float = 100.0,
        print_lmi_gain: bool = False,
        lmi_print_every: int = 1,
        dynamics_model = None,
        **kwargs
    ):
        """
        Initialize Generalized UIO Observer
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict (uses YAML defaults if None)
            delta2: Regularization parameter for D matrix in H
            use_y_dot_meas: Whether to use output derivative measurements
            observer_gain_scale: Scaling factor for default observer gains
            use_lmi_gain: Enable LMI-based gain computation (requires CVXPY)
            lmi_decay_rate: Decay rate used in LMI design
            lmi_recompute_interval: Recompute LMI gain every N updates (0 = cache per mode)
            lmi_max_gain: Clip magnitude of LMI gain for stability
            print_lmi_gain: Print LMI gain when computed
            lmi_print_every: Print LMI gain every N recomputations
            dynamics_model: Optional shared dynamics model instance
            **kwargs: Additional arguments (for compatibility)
        """
        # Initialize base class
        super().__init__(
            state_dim=STATE_DIM,
            unknown_input_dim=self.UNKNOWN_INPUT_DIM,
            sample_time=sample_time
        )
        
        # Vehicle parameters from centralized source
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
        self.min_vx = self.params.get('vx_min', 0.5)
        
        # Observer parameters
        self.delta2 = float(delta2)
        self.use_y_dot_meas = use_y_dot_meas
        self.observer_gain_scale = observer_gain_scale
        self.use_lmi_gain = use_lmi_gain
        self.lmi_decay_rate = float(lmi_decay_rate)
        self.lmi_recompute_interval = int(lmi_recompute_interval)
        self.lmi_max_gain = float(lmi_max_gain)
        self.print_lmi_gain = bool(print_lmi_gain)
        self.lmi_print_every = max(1, int(lmi_print_every))
        
        # Create centralized dynamics instance
        if dynamics_model is not None:
            self.dynamics = dynamics_model
        else:
            self.dynamics = create_qlpv_dynamics(
                vehicle_params=self.params,
                min_vx=self.min_vx,
                use_8d_system=False
            )
            
        # Covariance-like monitor (P) - Initialize roughly
        # This provides a metric for estimation confidence, similar to Kalman P
        self.P = np.eye(self.state_dim + self.UNKNOWN_INPUT_DIM) * 0.1
        
        # Internal observer state (allocated lazily per mode)
        self.eta = None       # Observer internal state
        self.zhat = None      # Descriptor state estimate
        self._y_prev = None   # Previous measurement (for ydot)
        self._mode = None     # Current mode ("gps_on" or "gps_off")
        
        # Tire residual estimates (w_r, w_f)
        self.w_hat = np.zeros(self.UNKNOWN_INPUT_DIM)
        
        # Internal state for control processing
        self.current_steering_angle = 0.0

        # Store last innovation for diagnostics
        self.innovation = None

        # LMI gain cache and diagnostics
        self._lmi_gain_cache = {}
        self._lmi_last_update = {}
        self._last_lmi_error = None
        self._step_count = 0
        self._gain_method = "default"
        
        # Numerical Jacobian epsilon
        self.epsilon = 1e-5
        
        # Disturbance decay rate (making unknown input dynamic)
        self.w_decay_rate = 5.0  # Decays to zero if not sustained
        
        # Tire info for logging (compatibility with qlpv_observer_kalma)
        self.tire_info_layer_1 = {
            'Fyr_linear': 0.0,
            'Fyf_linear': 0.0,
            'alpha_r': 0.0,
            'alpha_f': 0.0
        }

    
    # =========================================================================
    # Measurement Matrices (Mode-Dependent)
    # =========================================================================
    
    # =========================================================================
    # Measurement Matrices (Mode-Dependent)
    # =========================================================================
    
    # measurement_C removed - using centralized dynamics in update()

    
    @staticmethod
    def reg_D(ny: int) -> np.ndarray:
        """
        Regularization D matrix (ny × 2) with full column rank
        
        Args:
            ny: Number of measurements
            
        Returns:
            D matrix with first two rows having identity structure
        """
        D = np.zeros((ny, 2))
        D[0, 0] = 1.0
        D[1, 1] = 1.0
        return D
    
    def compute_gain(
        self,
        nzeta: int,
        ny: int,
        C: np.ndarray,
        Ewx: np.ndarray,
        H: np.ndarray,
        Pzeta: np.ndarray,
        zhat: Optional[np.ndarray],
        u_phys: np.ndarray
    ) -> np.ndarray:
        """
        Compute observer gain K.
        Attempts to load pre-calculated LMI gain from parameters.
        Fallback to robust default.
        
        Args:
            nzeta: Descriptor state dimension
            ny: Measurement dimension
        """
        # Check if gain is provided in params (e.g. from LMI offline design)
        # Key format: 'lmi_K_gps_on' or 'lmi_K_gps_off' based on mode would be ideal
        # For now, simplistic check
        param_key = f'lmi_K_{self._mode}' if self._mode else 'lmi_K'
        
        if param_key in self.params:
             K_loaded = np.asarray(self.params[param_key])
             if K_loaded.shape == (nzeta, ny):
                 self._gain_method = "preloaded"
                 return K_loaded * self.observer_gain_scale

        # Avoid LMI near standstill where dynamics/observability can be ill-conditioned
        try:
            vx_now = float(self.state_hat[IDX_VX])
        except Exception:
            vx_now = 0.0
        if abs(vx_now) < (self.min_vx + 1e-3):
            self._gain_method = "default_low_speed"
            return self._robust_default_K(nzeta, ny)

        if self.use_lmi_gain and CVXPY_AVAILABLE:
            cache_key = (self._mode, nzeta, ny)
            if self.lmi_recompute_interval <= 0 and cache_key in self._lmi_gain_cache:
                self._gain_method = "lmi_cached"
                return self._lmi_gain_cache[cache_key]

            if self.lmi_recompute_interval > 0:
                last = self._lmi_last_update.get(cache_key, -1)
                if (self._step_count - last) < self.lmi_recompute_interval and cache_key in self._lmi_gain_cache:
                    self._gain_method = "lmi_cached"
                    return self._lmi_gain_cache[cache_key]

            try:
                K_lmi = self._compute_lmi_gain(nzeta, ny, C, Ewx, H, Pzeta, zhat, u_phys)
                self._lmi_gain_cache[cache_key] = K_lmi
                self._lmi_last_update[cache_key] = self._step_count
                self._gain_method = "lmi"
                if self.print_lmi_gain and (self._step_count % self.lmi_print_every == 0):
                    print(f"[HuyUIOObserver] LMI gain (mode={self._mode}, step={self._step_count}):\n{K_lmi}")
                return K_lmi
            except Exception as e:
                self._last_lmi_error = str(e)

        self._gain_method = "default"
        return self._robust_default_K(nzeta, ny)

    def _robust_default_K(self, nzeta: int, ny: int) -> np.ndarray:
        """
        Robust default observer gain K (fallback)
        
        Args:
            nzeta: Descriptor state dimension
            ny: Measurement dimension
            
        Returns:
            K matrix (nzeta × ny)
        """
        K = np.zeros((nzeta, ny))
        scale = self.observer_gain_scale
        
        # Structure of zeta: [chi1(ny), chi2(ny), x(6), w(2)]
        # chi1 corresponds to direct output, chi2 to output derivatives
        # Use very conservative gains to ensure stability
        
        # Small correction on chi1 (output states) - these converge naturally
        for i in range(ny):
            K[i, i] = 1.0 * scale
        
        # Small correction on chi2 (derivative states)
        for i in range(ny):
            K[ny + i, i] = 0.2 * scale
        
        # Correction on x part - only measured states
        # x starts at index 2*ny in zeta
        # Measurement maps: y[0]->vx, y[1]->r, y[2]->psi, (y[3]->X, y[4]->Y for gps_on)
        state_map = [IDX_VX, IDX_R, IDX_PSI, IDX_X, IDX_Y]
        for i in range(min(ny, len(state_map))):
            xi = 2 * ny + state_map[i]
            if xi < nzeta:
                K[xi, i] = 0.5 * scale
        
        # No direct correction on w (tire residuals) - let the observer structure handle it
        
        return K

    def _jacobian_phi_zeta(self, zhat: np.ndarray, u: np.ndarray,
                           C: np.ndarray, Ewx: np.ndarray) -> np.ndarray:
        """
        Numerical Jacobian of phi_zeta with respect to zeta.
        """
        n = zhat.size
        f0 = self._phi_zeta(zhat, u, C, Ewx)
        J = np.zeros((n, n))

        for i in range(n):
            dz = np.zeros(n)
            dz[i] = self.epsilon
            fi = self._phi_zeta(zhat + dz, u, C, Ewx)
            J[:, i] = (fi - f0) / self.epsilon

        return np.clip(J, -1e4, 1e4)

    def _compute_lmi_gain(self, nzeta: int, ny: int, C: np.ndarray,
                          Ewx: np.ndarray, H: np.ndarray,
                          Pzeta: np.ndarray, zhat: Optional[np.ndarray],
                          u_phys: np.ndarray) -> np.ndarray:
        """
        Compute LMI-based observer gain using local linearization.
        """
        if zhat is None:
            raise ValueError("No zhat available for LMI gain computation")

        if not np.all(np.isfinite(zhat)):
            raise ValueError("Non-finite zhat for LMI gain computation")

        J = self._jacobian_phi_zeta(zhat, u_phys, C, Ewx)
        if not np.all(np.isfinite(J)):
            raise ValueError("Non-finite Jacobian for LMI gain computation")

        A_eff = Pzeta @ J
        A_eff = np.clip(A_eff, -1e3, 1e3)

        K = compute_lmi_observer_gain(A_eff, H, decay_rate=self.lmi_decay_rate, verbose=False)

        if self.lmi_max_gain is not None:
            K = np.clip(K, -self.lmi_max_gain, self.lmi_max_gain)

        if K.shape != (nzeta, ny):
            raise ValueError("LMI gain has incorrect shape")

        return K
    
    # =========================================================================
    # E Matrix from Centralized Dynamics
    # =========================================================================
    
    def compute_E_delta(self, delta: float) -> np.ndarray:
        """
        Compute residual injection matrix E(δ) using centralized dynamics
        
        E(δ) maps unknown tire residuals w = [w_r, w_f] to state derivatives.
        
        Args:
            delta: Steering angle [rad]
            
        Returns:
            E matrix (6×2)
        """
        # Use centralized dynamics to compute E matrix
        rho = self.dynamics.compute_scheduling_params(self.state_hat, delta)
        return self.dynamics.compute_E_matrix(rho)
    
    # =========================================================================
    # Known Dynamics (phi)
    # =========================================================================
    
    def phi(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Known part of dynamics (nominal bicycle model)
        
        Uses centralized dynamics with zero residuals.
        
        Args:
            x: State vector [v_x, v_y, ψ, r, X, Y]
            u: Control input [δ, a]
            
        Returns:
            State derivative from known dynamics
        """
        # Ensure state values are bounded to prevent numerical issues
        x_safe = x.copy()
        x_safe[IDX_VX] = np.clip(x_safe[IDX_VX], self.min_vx, 50.0)
        x_safe[IDX_VY] = np.clip(x_safe[IDX_VY], -10.0, 10.0)
        x_safe[IDX_R] = np.clip(x_safe[IDX_R], -5.0, 5.0)
        
        # Use centralized dynamics with zero tire residuals
        w_zero = np.zeros(2)
        result = self.dynamics.f_continuous(x_safe, u, w_zero)
        
        # Clip result to prevent overflow
        return np.clip(result, -1e6, 1e6)
    
    def jacobian_phi(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Numerical Jacobian of phi with respect to x
        
        Args:
            x: State vector
            u: Control input
            
        Returns:
            Jacobian matrix (6×6)
        """
        n = x.size
        f0 = self.phi(x, u)
        J = np.zeros((n, n))
        
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = self.epsilon
            fi = self.phi(x + dx, u)
            J[:, i] = (fi - f0) / self.epsilon
        
        return J
    
    # =========================================================================
    # Descriptor State Splitting
    # =========================================================================
    
    def _split_zeta(self, z: np.ndarray, ny: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Split descriptor state ζ into components
        
        ζ = [χ₁, χ₂, x, w]ᵀ
        
        Args:
            z: Descriptor state vector
            ny: Measurement dimension
            
        Returns:
            Tuple of (χ₁, χ₂, x̂, ŵ)
        """
        chi1 = z[0:ny]
        chi2 = z[ny:2*ny]
        xhat = z[2*ny:2*ny + self.state_dim]
        what = z[2*ny + self.state_dim:]
        return chi1, chi2, xhat, what
    
    # =========================================================================
    # Descriptor Dynamics
    # =========================================================================
    
    def _phi_zeta(self, zhat: np.ndarray, u: np.ndarray, C: np.ndarray, Ewx: np.ndarray) -> np.ndarray:
        """
        Compute descriptor state dynamics φ_ζ
        
        φ_ζ = [χ̇₁, χ̇₂, ẋ, ẇ]ᵀ where:
            - χ̇₁ = χ₂
            - χ̇₂ ≈ C·(J·φ + J·(E·w)) (scaled for numerical stability)
            - ẋ = φ(x,u) + E·w
            - ẇ = 0 (random walk assumption)
        
        Args:
            zhat: Descriptor state estimate
            u: Control input
            C: Output matrix
            Ewx: Residual injection matrix E(δ)
            
        Returns:
            Descriptor state derivative
        """
        ny = C.shape[0]
        _, chi2, xhat, what = self._split_zeta(zhat, ny)
        
        # Known dynamics with bounds
        phi_x = self.phi(xhat, u)  # 6D, already bounded
        J = self.jacobian_phi(xhat, u)  # 6×6
        
        # Clip Jacobian to prevent extreme values
        J = np.clip(J, -1e3, 1e3)
        
        # Approximate bar_phi = J·φ (for χ₂ dynamics) with scaling
        bar_phi = J @ phi_x
        bar_phi = np.clip(bar_phi, -1e3, 1e3)  # Clip large values
        
        # State dynamics with residual injection
        x_dot = phi_x + Ewx @ what
        x_dot = np.clip(x_dot, -1e3, 1e3)
        
        # Output dynamics - use simplified chi2_dot to avoid numerical issues
        # chi1_dot = chi2 (direct relationship)
        # chi2_dot is approximated more conservatively
        chi1_dot = chi2
        
        # Scale chi2_dot to prevent explosion
        chi2_dot_raw = C @ (bar_phi + J @ (Ewx @ what))
        chi2_dot = np.clip(chi2_dot_raw, -100.0, 100.0)  # Conservative clipping
        
        # Residual dynamics (more dynamic: decay/mean-reversion)
        # w_dot = -λ * w ("dynamic" unknown input)
        w_dot = -self.w_decay_rate * what
        
        return np.concatenate([chi1_dot, chi2_dot, x_dot, w_dot])
    
    # =========================================================================
    # Observer State Allocation
    # =========================================================================
    
    def _ensure_mode_consistency(self, mode: str, y_meas: np.ndarray, 
                               Qzeta_prev: Optional[np.ndarray] = None) -> bool:
        """
        Ensure internal state eta is consistent with current mode.
        If mode changes, project old estimate to new eta space to preserve continuity.
        
        Args:
            mode: Target mode
            y_meas: Current measurement
            Qzeta_prev: (Optional) Q matrix from previous step/mode for better projection
            
        Returns:
            True if reallocation occurred
        """
        if self._mode == mode and self.eta is not None:
            return False
            
        # Switching modes or initialization
        old_mode = self._mode
        old_zhat = self.zhat.copy() if self.zhat is not None else None
        
        # Setup for new mode
        if mode == "gps_on":
            ny = 5
        else:
            ny = 3
            
        nzeta = 2 * ny + self.state_dim + self.UNKNOWN_INPUT_DIM
        
        # We need to compute Qzeta for the NEW mode to initialize eta correctly
        # eta_new = zhat_old - Qzeta_new * y_new
        # However, we don't have Qzeta_new yet (it depends on matrices we haven't computed).
        # Strategy: Initialize eta with zeros, but fill in the 'x' and 'w' components 
        # from old_zhat (or state_hat) if available.
        # This acts as a prediction step.
        
        self.eta = np.zeros(nzeta)
        self.zhat = np.zeros(nzeta)
        
        # Fill estimate from previous state
        # Structure of zhat: [chi1, chi2, x, w]
        # x is at index 2*ny
        idx_x_start = 2 * ny
        idx_w_start = 2 * ny + self.state_dim
        
        if old_zhat is not None:
             # Best effort: use current state estimates
             self.eta[idx_x_start:idx_x_start+self.state_dim] = self.state_hat
             self.eta[idx_w_start:idx_w_start+self.UNKNOWN_INPUT_DIM] = self.w_hat
        elif len(self.state_hat) > 0:
             self.eta[idx_x_start:idx_x_start+self.state_dim] = self.state_hat
             
        # Initialize chi1 part with measurements (y_meas should be passed processed)
        if y_meas is not None:
             n_copy = min(len(y_meas), ny)
             self.eta[0:n_copy] = y_meas[:n_copy]

        self._y_prev = None # Reset derivative computation on mode switch
        self._mode = mode
        return True
    
    # =========================================================================
    # Main Update Method (Interface Implementation)
    # =========================================================================
    
    def update(
        self,
        measurement: np.ndarray,
        control_input: np.ndarray,
        f_nn: Optional[np.ndarray] = None,
        acceleration: Optional[np.ndarray] = None,
        gps_available: bool = True,
        dt: Optional[float] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update observer with new measurement
        
        Implements the generalized UIO update step:
            1. Build measurement y_χ
            2. Compute descriptor matrices E, H
            3. Compute P_ζ, Q_ζ via pseudoinverse
            4. Algebraic estimate: ζ̂ = η + Q_ζ·y_χ
            5. Compute innovation with yaw wrapping
            6. Integrate: η̇ = P_ζ·φ_ζ + K·innovation
            7. Update: ζ̂ = η + Q_ζ·y_χ
        
        Args:
            measurement: Measurement vector
                - GPS on: [v_x, r, ψ, X, Y] or [v_x, r, ψ, X, Y, a_y]
                - GPS off: [v_x, r, ψ] or [v_x, r, ψ, a_y]
            control_input: Control input u = [δ, a]
            f_nn: Neural network output (unused, for interface compatibility)
            acceleration: Optional IMU acceleration (unused in this observer)
            gps_available: Whether GPS is available
            
        Returns:
            Tuple of (state_estimate, unknown_input_estimate)
                - state_estimate: [v_x, v_y, ψ, r, X, Y]
                - unknown_input_estimate: [w_r, w_f]
        """
        self._step_count += 1
        # Determine current time step
        current_dt = dt if dt is not None else self.Ts

        # Process Control Input
        u_raw = control_input.reshape(-1)
        throttle_cmd = u_raw[1] if len(u_raw) > 1 else 0.0
        steering_cmd = u_raw[0]
        
        # Construct state vector for dynamics module
        current_state = self.state_hat.copy()
        
        # Use centralized dynamics to process control
        accel, _, new_steering_angle = self.dynamics.process_control_inputs(
            throttle_cmd, steering_cmd, current_state, 
            self.current_steering_angle, current_dt
        )
        self.current_steering_angle = new_steering_angle
        
        # Compute scheduling parameters for C matrix calculation
        # We need these BEFORE ensuring mode consistency because we might normally use them
        # But for mode consistency we only need NY which is checking size.
        
        # Process measurement and Determine Mode based on Size
        measurement = np.asarray(measurement).flatten()
        meas_len = len(measurement)
        
        # Automatic mode detection ("ez know by check the size")
        if meas_len >= 5:
            mode = "gps_on"
            ny = 5
        else:
            mode = "gps_off"
            ny = 3
            
        # Extract y vector based on mode
        # GPS ON: [vx, r, psi, X, Y] (indices 0,1,2,3,4 of 6D standard)
        # GPS OFF: [vx, r, psi] (indices 0,1,2 of 6D standard)
        # Note: Centralized C returns lines for [vx, r, psi, X, Y, a_y]
        
        if mode == "gps_on":
             y_meas = measurement[:5]
             psi_idx = 2
        else:
             y_meas = measurement[:3]
             psi_idx = 2
             
        # Wrap yaw
        y_meas[psi_idx] = wrap_to_pi(y_meas[psi_idx])

        # Ensure mode consistency and allocate if needed
        self._ensure_mode_consistency(mode, y_meas)
        
        # Calculate C using centralized dynamics
        # Need Rho.
        rho = self.dynamics.compute_scheduling_params(self.state_hat, new_steering_angle)
        C_full = self.dynamics.compute_C_matrix(rho) # 6x6
        
        # Extract relevant rows of C
        if mode == "gps_on":
             C = C_full[:5, :]
        else:
             C = C_full[:3, :]

        
        # Build y_χ (optionally with ẏ; default: just y)
        if self.use_y_dot_meas:
            if self._y_prev is None:
                y_dot = np.zeros_like(y_meas)
            else:
                y_dot = (y_meas - self._y_prev) / self.Ts
            self._y_prev = y_meas.copy()
            Cchi = np.zeros((ny, ny))
            y_chi = y_meas + Cchi @ y_dot
        else:
            y_chi = y_meas
        
        

        
        # Form physical control vector [delta, a]
        u_phys = np.array([self.current_steering_angle, accel])
        
        # Helper for delta
        delta = float(u_phys[0])
        Ewx = self.compute_E_delta(delta)  # 6×2
        
        # Build descriptor matrices E, H
        nzeta = 2 * ny + self.state_dim + self.UNKNOWN_INPUT_DIM
        CB = C @ Ewx  # ny × 2
        
        # Descriptor E matrix: (2ny+nx) × (2ny+nx+nmu)
        E_top = np.hstack([
            np.eye(ny),
            np.zeros((ny, ny)),
            np.zeros((ny, self.state_dim)),
            np.zeros((ny, self.UNKNOWN_INPUT_DIM))
        ])
        E_mid = np.hstack([
            np.zeros((ny, ny)),
            np.eye(ny),
            np.zeros((ny, self.state_dim)),
            -CB
        ])
        E_bot = np.hstack([
            np.zeros((self.state_dim, ny)),
            np.zeros((self.state_dim, ny)),
            np.eye(self.state_dim),
            np.zeros((self.state_dim, self.UNKNOWN_INPUT_DIM))
        ])
        E_desc = np.vstack([E_top, E_mid, E_bot])  # (2ny+nx) × nzeta
        
        # H matrix: [I, Cchi, C, D_δ₂]
        Cchi = np.zeros((ny, ny))
        D = self.reg_D(ny)
        D_delta2 = self.delta2 * D
        H = np.hstack([np.eye(ny), Cchi, C, D_delta2])  # ny × nzeta
        
        # Compute P_ζ, Q_ζ such that P_ζ·E + Q_ζ·H = I (via regularized pseudoinverse)
        # E_desc is (2ny+nx) × nzeta, H is ny × nzeta
        # M = [E_desc; H] is (3ny+nx) × nzeta
        M = np.vstack([E_desc, H])  # (3ny+nx) × nzeta
        
        # Use regularized pseudoinverse for numerical stability
        # L = M^T (M M^T + λI)^{-1}
        # Replacing explicit inv with pinv for better stability
        MMT = M @ M.T
        # MMT_reg = MMT + reg_lambda * np.eye(MMT.shape[0])
        # L = M.T @ np.linalg.inv(MMT_reg)
        L = M.T @ np.linalg.pinv(MMT, rcond=1e-5)
        
        # Split L to get Pzeta and Qzeta
        # Pzeta multiplies phi_z which has dimension (2ny+nx+nmu) but E_desc has (2ny+nx) rows
        # So Pzeta should be nzeta × (2ny+nx)
        n_E_rows = 2 * ny + self.state_dim  # rows of E_desc
        Pzeta = L[:, :n_E_rows]   # nzeta × (2ny+nx)
        Qzeta = L[:, n_E_rows:]   # nzeta × ny
        
        # Algebraic estimate
        self.zhat = self.eta + Qzeta @ y_chi

        # Observer gain (LMI-based when available)
        K = self.compute_gain(nzeta, ny, C, Ewx, H, Pzeta, self.zhat, u_phys)
        
        # Innovation with yaw wrapping
        innov = y_chi - H @ self.zhat
        if psi_idx >= 0:
            innov[psi_idx] = wrap_to_pi(innov[psi_idx])
        self.innovation = innov.copy()
        
        # Descriptor dynamics
        phi_z = self._phi_zeta(self.zhat, u_phys, C, Ewx)
        
        # phi_z has dimension nzeta = (2ny + nx + nmu)
        # Pzeta has dimension nzeta × (2ny + nx)
        # We need to extract the E_desc part (first 2ny + nx elements)
        n_E_rows = 2 * ny + self.state_dim
        phi_z_E = phi_z[:n_E_rows]  # Extract dynamics corresponding to E_desc rows
        
        # Clip phi_z_E to prevent numerical overflow
        phi_z_E = np.clip(phi_z_E, -1e4, 1e4)
        
        eta_dot = Pzeta @ phi_z_E + K @ innov
        
        # Clip eta_dot to prevent runaway integration
        eta_dot = np.clip(eta_dot, -1e4, 1e4)
        
        # Integrate
        self.eta = self.eta + self.Ts * eta_dot
        
        # Clip eta to reasonable bounds
        max_eta = 1e6
        self.eta = np.clip(self.eta, -max_eta, max_eta)
        
        self.zhat = self.eta + Qzeta @ y_chi
        
        # Clip zhat as well
        self.zhat = np.clip(self.zhat, -max_eta, max_eta)
        
        # Extract state and residual estimates
        _, _, xhat, what = self._split_zeta(self.zhat, ny)
        
        # Store estimates
        self.state_hat = xhat.copy()
        self.w_hat = what.copy()
        self.f_uk_hat = what.copy()  # Required for base class interface
        
        # Calculate tire info for logging (compatibility with qlpv_observer_kalma)
        self.tire_info_layer_1 = self.dynamics._calculate_tire_info(
            self.state_hat[IDX_VX], 
            self.state_hat[IDX_VY], 
            self.state_hat[IDX_R], 
            self.current_steering_angle,
            self.w_hat[0],  # w_r
            self.w_hat[1]   # w_f
        )
        
        return self.state_hat.copy(), self.w_hat.copy()
    
    def _process_measurement(self, measurement: np.ndarray, mode: str) -> np.ndarray:
        """
        Process measurement to match expected format for given mode
        
        Args:
            measurement: Raw measurement (may include extra values like a_y)
            mode: Current mode
            
        Returns:
            Processed measurement matching expected dimension
        """
        measurement = np.asarray(measurement).flatten()
        
        if mode == "gps_on":
            # Expected: [v_x, r, ψ, X, Y]
            if len(measurement) >= 5:
                # If 6D measurement [v_x, r, ψ, X, Y, a_y], extract first 5
                # Map from standard format to UIO format
                y = np.array([
                    measurement[MEAS_IDX_VX],   # v_x
                    measurement[MEAS_IDX_R],    # r
                    measurement[MEAS_IDX_PSI],  # ψ
                    measurement[MEAS_IDX_X],    # X
                    measurement[MEAS_IDX_Y],    # Y
                ])
            else:
                y = measurement[:5]
        else:
            # GPS off: [v_x, r, ψ]
            if len(measurement) >= 3:
                y = np.array([
                    measurement[MEAS_IDX_VX],   # v_x
                    measurement[MEAS_IDX_R],    # r
                    measurement[MEAS_IDX_PSI],  # ψ
                ])
            else:
                y = measurement[:3]
        
        # Wrap yaw angle
        psi_idx = 2  # ψ is always at index 2 in our measurement
        y[psi_idx] = wrap_to_pi(y[psi_idx])
        
        return y
    
    # =========================================================================
    # Reset Method
    # =========================================================================
    
    def reset(self, initial_state: Optional[np.ndarray] = None,
              initial_position: Optional[np.ndarray] = None):
        """
        Reset observer to initial state
        
        Args:
            initial_state: Initial state estimate [v_x, v_y, ψ, r, X, Y]
            initial_position: Optional initial position [X, Y] (overrides initial_state)
        """
        # Call base class reset
        super().reset(initial_state)
        
        # Handle initial position override
        if initial_position is not None:
            initial_position = np.asarray(initial_position).flatten()
            if len(initial_position) >= 2:
                self.state_hat[IDX_X] = initial_position[0]
                self.state_hat[IDX_Y] = initial_position[1]
            if len(initial_position) >= 3:
                self.state_hat[IDX_PSI] = initial_position[2]
        
        # Reset internal observer state
        self.eta = None
        self.zhat = None
        self._y_prev = None
        self._mode = None
        
        # Reset tire residual estimates
        self.w_hat = np.zeros(self.UNKNOWN_INPUT_DIM)
        self.f_uk_hat = np.zeros(self.UNKNOWN_INPUT_DIM)
        
        # Reset innovation
        self.innovation = None

        # Reset LMI caches
        self._lmi_gain_cache = {}
        self._lmi_last_update = {}
        self._last_lmi_error = None
        self._step_count = 0
        self._gain_method = "default"
    
    # =========================================================================
    # Diagnostic Getters
    # =========================================================================
    
    def get_tire_residuals(self) -> np.ndarray:
        """Get tire force residual estimates [w_r, w_f]"""
        return self.w_hat.copy()
    
    def get_descriptor_state(self) -> Optional[np.ndarray]:
        """Get full descriptor state ζ̂ for diagnostics"""
        return self.zhat.copy() if self.zhat is not None else None
    
    def get_internal_state(self) -> Optional[np.ndarray]:
        """Get internal observer state η for diagnostics"""
        return self.eta.copy() if self.eta is not None else None
    
    def get_innovation(self) -> Optional[np.ndarray]:
        """Get last computed innovation"""
        return self.innovation.copy() if self.innovation is not None else None
    
    def get_current_mode(self) -> Optional[str]:
        """Get current measurement mode"""
        return self._mode
    
    def get_augmented_state(self) -> np.ndarray:
        """
        Get augmented state [x̂; ŵ] for compatibility with other observers
        
        Returns:
            8D augmented state [v_x, v_y, ψ, r, X, Y, w_r, w_f]
        """
        return np.concatenate([self.state_hat, self.w_hat])
    
    def get_state(self) -> np.ndarray:
        """Get current 6D state estimate [v_x, v_y, ψ, r, X, Y]"""
        return self.state_hat.copy()
    
    def get_unknown_input(self) -> np.ndarray:
        """Get unknown input estimate (alias for tire residuals)"""
        return self.w_hat.copy()
    
    def get_unknown_input_estimate(self) -> np.ndarray:
        """Get estimate of unknown inputs (tire residuals)"""
        return self.w_hat.copy()
    
    # =========================================================================
    # Observer Tuning
    # =========================================================================
    
    def set_observer_gain_scale(self, scale: float):
        """
        Set scaling factor for default observer gains
        
        Args:
            scale: Gain scaling factor (default 1.0)
        """
        self.observer_gain_scale = scale
    
    def set_delta2(self, delta2: float):
        """
        Set regularization parameter for D matrix
        
        Args:
            delta2: Regularization value (default 1e-3)
        """
        self.delta2 = float(delta2)


# =============================================================================
# Factory Function
# =============================================================================

def create_huy_uio_observer(
    sample_time: float = 0.02,
    vehicle_params: Optional[Dict] = None,
    **kwargs
) -> HuyUIOObserver:
    """
    Factory function to create Generalized UIO observer
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dictionary
        **kwargs: Additional arguments passed to constructor
            - delta2: Regularization parameter (default 1e-3)
            - use_y_dot_meas: Use output derivative measurements (default False)
            - observer_gain_scale: Gain scaling factor (default 1.0)
            
    Returns:
        Configured HuyUIOObserver instance
    """
    return HuyUIOObserver(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        **kwargs
    )


# =============================================================================
# Test Code
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Generalized UIO Observer Test")
    print("=" * 60)
    
    # Create observer with reduced gain scale for stability
    observer = create_huy_uio_observer(sample_time=0.02, observer_gain_scale=0.5)
    print(f"Observer created with state_dim={observer.state_dim}")
    
    # Create dynamics for simulation
    dynamics = create_qlpv_dynamics()
    
    # "True" simulated state - start with stable conditions
    x_true = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Low speed, straight
    w_true = np.array([5.0, 8.0])    # Small tire residuals [N]
    u = np.array([0.01, 0.05])       # Small steering/accel
    
    dt = 0.02
    gps_available = True
    
    print(f"\nSimulating with GPS {'on' if gps_available else 'off'}...")
    print(f"True initial state: {x_true}")
    print(f"True tire residuals: {w_true}")
    
    # Initialize observer with true initial state
    observer.reset(initial_state=x_true.copy())
    
    errors_state = []
    errors_w = []
    
    for k in range(300):
        # Simulate truth using centralized dynamics
        x_dot = dynamics.f_continuous(x_true, u, w_true)
        x_true = x_true + dt * x_dot
        
        # Clamp state to prevent simulation divergence
        x_true[IDX_VX] = np.clip(x_true[IDX_VX], 0.5, 20.0)
        x_true[IDX_PSI] = wrap_to_pi(x_true[IDX_PSI])
        
        # Build measurements according to GPS availability
        if gps_available:
            # y = [v_x, r, ψ, X, Y, a_y] (6D standard format)
            y = np.array([
                x_true[IDX_VX] + 0.01 * np.random.randn(),
                x_true[IDX_R] + 0.005 * np.random.randn(),
                wrap_to_pi(x_true[IDX_PSI] + 0.01 * np.random.randn()),
                x_true[IDX_X] + 0.05 * np.random.randn(),
                x_true[IDX_Y] + 0.05 * np.random.randn(),
                0.0,  # a_y placeholder
            ])
        else:
            # y = [v_x, r, ψ, -, -, a_y] (GPS entries will be ignored)
            y = np.array([
                x_true[IDX_VX] + 0.01 * np.random.randn(),
                x_true[IDX_R] + 0.005 * np.random.randn(),
                wrap_to_pi(x_true[IDX_PSI] + 0.01 * np.random.randn()),
                0.0, 0.0, 0.0  # Placeholder
            ])
        
        # Observer update
        xhat, what = observer.update(y, u, gps_available=gps_available)
        
        # Check for NaN and break early
        if np.any(np.isnan(xhat)) or np.any(np.isnan(what)):
            print(f"WARNING: NaN detected at k={k}, stopping early")
            break
        
        # Track errors
        errors_state.append(np.linalg.norm(x_true - xhat))
        errors_w.append(np.linalg.norm(w_true - what))
        
        # Print progress
        if k % 50 == 0:
            print(f"  k={k:3d}: state_err={errors_state[-1]:.4f}, w_err={errors_w[-1]:.4f}, mode={observer.get_current_mode()}")
        
        # Switch GPS off at k=150
        if k == 150:
            gps_available = False
            print(f"\n--- GPS dropout at k={k} ---")
    
    print(f"\nFinal Results:")
    print(f"  True state:  {x_true}")
    print(f"  Estimated:   {xhat}")
    print(f"  State error: {np.linalg.norm(x_true - xhat):.4f}")
    print(f"\n  True w:      {w_true}")
    print(f"  Estimated w: {what}")
    print(f"  w error:     {np.linalg.norm(w_true - what):.4f}")
    print(f"\n  Mode: {observer.get_current_mode()}")
    
    if errors_state:
        print(f"\n  Mean state error: {np.mean(errors_state):.4f}")
        print(f"  Final state error: {errors_state[-1]:.4f}")
        print(f"  Mean w error: {np.mean(errors_w):.4f}")
    
    print("\n✅ HuyUIOObserver test completed")
