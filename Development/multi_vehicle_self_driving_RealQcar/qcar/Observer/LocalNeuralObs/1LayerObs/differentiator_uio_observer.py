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

# Import differentiators from centralized module
from differentiators import (
    DirtyDerivative,
    HighGainDifferentiator,
    SlidingModeDifferentiator,
    create_differentiator,
    create_differentiator_from_config,
    load_differentiator_config,
)

# Import centralized qLPV vehicle dynamics
sys.path.insert(0, str(parent_dir.parent))
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
)

# Import CVXPY for LMI-based gain design
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False
    print("Warning: cvxpy not available. LMI-based gain design disabled, using default gains.")

# Import scipy for pole placement fallback
try:
    from scipy.signal import place_poles
    from scipy.linalg import solve_continuous_lyapunov
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


def compute_pole_placement_gain(A: np.ndarray, C: np.ndarray,
                                 desired_poles: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Compute observer gain using pole placement (fallback method).
    
    Places observer error dynamics poles at stable locations.
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)
        desired_poles: Desired closed-loop poles. If None, uses default stable poles.
        
    Returns:
        L: Observer gain matrix (n × m)
    """
    n = A.shape[0]
    
    if desired_poles is None:
        # Default: place poles at stable locations with good damping
        # Poles spread in left half-plane for smooth convergence
        desired_poles = np.array([-2.0, -2.5, -3.0, -3.5, -4.0, -4.5])
    
    if not SCIPY_AVAILABLE:
        # Simple fallback: use diagonal gain based on A structure
        L = np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5])
        return L
    
    try:
        # Pole placement for observer: place_poles works on (A - L*C)^T = A^T - C^T * L^T
        result = place_poles(A.T, C.T, desired_poles)
        L = result.gain_matrix.T
        return L
    except Exception as e:
        # Fallback to simple diagonal gain
        return np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5])


def validate_observer_gain(A: np.ndarray, C: np.ndarray, L: np.ndarray,
                           max_real_part: float = -0.1) -> bool:
    """
    Validate that observer gain produces stable error dynamics.
    
    Checks that all eigenvalues of (A - L @ C) have negative real parts.
    
    Args:
        A: State matrix
        C: Output matrix  
        L: Observer gain
        max_real_part: Maximum allowed real part of eigenvalues
        
    Returns:
        True if gain is valid (all eigenvalues stable)
    """
    try:
        A_cl = A - L @ C
        eigenvalues = np.linalg.eigvals(A_cl)
        max_eig_real = np.max(np.real(eigenvalues))
        return max_eig_real < max_real_part
    except:
        return False


def compute_lmi_observer_gain(A: np.ndarray, C: np.ndarray, 
                              decay_rate: float = 1.0,
                              verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using LMI-based design with Lyapunov stability.
    
    For the observer error dynamics:
        ė = (A - LC) e
    
    We solve for P > 0 and Y = P @ L such that:
        AᵀP + PA - CᵀYᵀ - YC + γP ≺ 0
    
    where γ (decay_rate) controls the minimum exponential decay rate.
    
    After solving, recover: L = P⁻¹ @ Y
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)
        decay_rate: Minimum decay rate γ > 0 (larger = faster convergence)
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
        
    Raises:
        ValueError: If the LMI problem is infeasible or CVXPY not available
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available. Install with: pip install cvxpy")
    
    n = A.shape[0]  # State dimension
    m = C.shape[0]  # Measurement dimension
    
    # Decision variables
    P = cp.Variable((n, n), symmetric=True)  # Lyapunov matrix
    Y = cp.Variable((n, m))  # Y = P @ L
    
    # LMI constraint: Aᵀ P + P A - Cᵀ Yᵀ - Y C + γ P ≺ 0
    eps = 1e-6  # Small positive value for strict inequality
    lmi_constraint = A.T @ P + P @ A - C.T @ Y.T - Y @ C + decay_rate * P
    
    # Add regularization to prevent ill-conditioned P
    gamma_reg = 0.01  # Regularization weight
    
    constraints = [
        P >> eps * np.eye(n),  # P > 0 (positive definite)
        P << 1e4 * np.eye(n),  # Upper bound on P for conditioning
        lmi_constraint << -eps * np.eye(n),  # LMI < 0
    ]
    
    # Objective: minimize trace(P) + regularization on Y
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    # Solve the SDP
    problem = cp.Problem(objective, constraints)
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=10000, eps=1e-6)
    except Exception as e:
        # Try alternative solver
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except:
            raise ValueError(f"LMI solver failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"LMI problem infeasible. Status: {problem.status}")
    
    # Recover observer gain: L = P⁻¹ @ Y
    P_val = P.value
    Y_val = Y.value
    
    if P_val is None or Y_val is None:
        raise ValueError("LMI solver returned None values")
    
    try:
        L = np.linalg.solve(P_val, Y_val)
    except np.linalg.LinAlgError:
        # Fallback to pseudo-inverse if P is nearly singular
        L = np.linalg.pinv(P_val) @ Y_val
    
    # Validate the computed gain
    if not validate_observer_gain(A, C, L):
        raise ValueError("Computed gain does not produce stable observer")
    
    # Limit gain magnitude to prevent numerical issues
    L = np.clip(L, -100.0, 100.0)
    
    return L


# =============================================================================
# Polytopic qLPV LMI Gain Scheduling Design
# =============================================================================

@dataclass
class PolytopicVertex:
    """Represents a vertex of the scheduling parameter polytope"""
    vx: float       # Longitudinal velocity
    delta: float    # Steering angle
    psi: float      # Yaw angle (for position dynamics)
    
    def to_tuple(self) -> tuple:
        return (self.vx, self.delta, self.psi)


class QLPVGainScheduler:
    """
    Polytopic qLPV Gain Scheduling for Observer Design
    
    For a qLPV system with scheduling parameters ρ ∈ Ω, we design observer
    gains at the vertices of a polytope covering Ω, then use convex
    interpolation for real-time gain scheduling.
    
    The polytope vertices are defined by:
        - vx ∈ [vx_min, vx_max]
        - delta ∈ [-delta_max, delta_max]
        - psi: discretized for kinematic terms (optional)
    
    For each vertex i, we compute L_i such that (A_i - L_i @ C_i) is Hurwitz.
    A common Lyapunov matrix P ensures stability across the entire polytope.
    
    Real-time gain: L(ρ) = Σ α_i(ρ) · L_i  where Σ α_i = 1, α_i ≥ 0
    
    Reference:
        - Apkarian et al., "Self-scheduled H∞ control of linear 
          parameter-varying systems"
        - Scherer, "Mixed H2/H∞ Control for LPV Systems"
    """
    
    def __init__(self, 
                 vehicle_params: Dict,
                 vx_range: Tuple[float, float] = (0.5, 3.0),
                 delta_max: float = 0.4,
                 n_vx_vertices: int = 3,
                 n_delta_vertices: int = 3,
                 decay_rate: float = 1.0,
                 use_common_lyapunov: bool = True,
                 verbose: bool = False):
        """
        Initialize polytopic qLPV gain scheduler
        
        Args:
            vehicle_params: Vehicle parameter dictionary
            vx_range: (vx_min, vx_max) velocity range [m/s]
            delta_max: Maximum steering angle magnitude [rad]
            n_vx_vertices: Number of velocity grid points
            n_delta_vertices: Number of steering grid points
            decay_rate: Minimum decay rate γ for Lyapunov LMI (smaller = more robust)
            use_common_lyapunov: If True, use single P for all vertices (robust)
                                 If False, compute independent gains per vertex
            verbose: Print solver output
        """
        self.params = vehicle_params
        self.vx_range = vx_range
        self.delta_max = delta_max
        self.decay_rate = decay_rate
        self.use_common_lyapunov = use_common_lyapunov
        self.verbose = verbose
        
        # Extract vehicle params
        self.lf = vehicle_params.get('lf', 0.11)
        self.lr = vehicle_params.get('lr', 0.11)
        self.m = vehicle_params.get('m', 2.5)
        self.Iz = vehicle_params.get('Iz', 0.02)
        self.Cf = vehicle_params.get('Cf', 50.0)
        self.Cr = vehicle_params.get('Cr', 50.0)
        
        # Generate polytope vertices
        self.vertices = self._generate_vertices(n_vx_vertices, n_delta_vertices)
        self.n_vertices = len(self.vertices)
        
        # Storage for gains computed at each vertex
        self.vertex_gains: Dict[tuple, np.ndarray] = {}
        
        # Common Lyapunov matrix (if using robust design)
        self.P_common: Optional[np.ndarray] = None
        
        # Flag to track if gains have been computed
        self._gains_computed = False
        
        # Cached interpolation weights
        self._last_weights: Optional[np.ndarray] = None
        self._last_rho: Optional[tuple] = None
        
        # Default gain for fallback
        self._default_gain = self._compute_robust_default_gain()
    
    def _compute_robust_default_gain(self) -> np.ndarray:
        """Compute a robust default gain using moderate pole placement"""
        # Design a well-tuned diagonal gain based on typical vehicle dynamics
        # Higher gains for directly measured states, lower for estimated
        L = np.zeros((6, 6))
        
        # vx measurement -> vx state (direct, high gain)
        L[0, 0] = 5.0
        
        # r measurement -> vy state (coupled through dynamics)
        L[1, 1] = 3.0
        
        # psi measurement -> psi state (direct)
        L[2, 2] = 5.0
        
        # r measurement -> r state (direct, high gain)
        L[3, 1] = 5.0
        
        # X measurement -> X state (direct)
        L[4, 3] = 3.0
        
        # Y measurement -> Y state (direct)
        L[5, 4] = 3.0
        
        return L
    
    def _generate_vertices(self, n_vx: int, n_delta: int) -> list:
        """Generate vertices of the scheduling parameter polytope"""
        vx_values = np.linspace(self.vx_range[0], self.vx_range[1], n_vx)
        delta_values = np.linspace(-self.delta_max, self.delta_max, n_delta)
        
        # For now, use psi = 0 (kinematic terms cos/sin don't affect core dynamics)
        psi_values = [0.0]
        
        vertices = []
        for vx in vx_values:
            for delta in delta_values:
                for psi in psi_values:
                    vertices.append(PolytopicVertex(vx=vx, delta=delta, psi=psi))
        
        return vertices
    
    def _compute_A_at_vertex(self, vertex: PolytopicVertex) -> np.ndarray:
        """Compute A matrix at a polytope vertex"""
        vx = max(vertex.vx, 0.5)  # Avoid division by zero, use reasonable min
        delta = vertex.delta
        psi = vertex.psi
        
        cos_d = np.cos(delta)
        sin_d = np.sin(delta)
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        vy = 0.0  # Nominal vy at vertex
        
        A = np.zeros((6, 6))
        
        # v_y dynamics
        A[1, 1] = -(self.Cr + self.Cf * cos_d) / (self.m * vx)
        A[1, 3] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.m * vx) - vx
        
        # ψ dynamics
        A[2, 3] = 1.0
        
        # r dynamics
        A[3, 1] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.Iz * vx)
        A[3, 3] = -(self.Cf * self.lf**2 * cos_d + self.Cr * self.lr**2) / (self.Iz * vx)
        
        # X dynamics
        A[4, 0] = cos_psi
        A[4, 1] = -sin_psi
        A[4, 2] = -vx * sin_psi - vy * cos_psi
        
        # Y dynamics
        A[5, 0] = sin_psi
        A[5, 1] = cos_psi
        A[5, 2] = vx * cos_psi - vy * sin_psi
        
        return A
    
    def _compute_C_at_vertex(self, vertex: PolytopicVertex) -> np.ndarray:
        """Compute C matrix at a polytope vertex"""
        C = np.zeros((6, 6))
        
        # Direct measurements: y = [vx, r, psi, X, Y, ay]
        # Mapping to state x = [vx, vy, psi, r, X, Y]
        C[0, 0] = 1.0  # y[0] = vx = x[0]
        C[1, 3] = 1.0  # y[1] = r = x[3]
        C[2, 2] = 1.0  # y[2] = psi = x[2]
        C[3, 4] = 1.0  # y[3] = X = x[4]
        C[4, 5] = 1.0  # y[4] = Y = x[5]
        # y[5] = a_y handled separately
        
        return C
    
    def compute_gains_lmi(self) -> bool:
        """
        Compute observer gains at all polytope vertices using LMI.
        
        If use_common_lyapunov=True, solves a single SDP for all vertices
        with a common Lyapunov matrix P, ensuring robust stability.
        
        If use_common_lyapunov=False, solves independent SDPs per vertex.
        
        Returns:
            True if all gains computed successfully, False otherwise
        """
        if not CVXPY_AVAILABLE:
            print("Warning: CVXPY not available, using pole placement fallback")
            return self._compute_gains_pole_placement()
        
        n = 6  # State dimension
        m = 6  # Measurement dimension
        
        if self.use_common_lyapunov:
            success = self._compute_gains_common_lyapunov(n, m)
        else:
            success = self._compute_gains_independent(n, m)
        
        # Validate all computed gains
        if success:
            success = self._validate_all_gains()
        
        if not success:
            print("Warning: LMI gains failed validation, using pole placement fallback")
            return self._compute_gains_pole_placement()
        
        return True
    
    def _compute_gains_pole_placement(self) -> bool:
        """Fallback: compute gains using pole placement at each vertex"""
        success = True
        
        for vertex in self.vertices:
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex)
            
            try:
                L = compute_pole_placement_gain(A, C)
                if validate_observer_gain(A, C, L):
                    self.vertex_gains[vertex.to_tuple()] = L
                else:
                    self.vertex_gains[vertex.to_tuple()] = self._default_gain.copy()
            except Exception as e:
                self.vertex_gains[vertex.to_tuple()] = self._default_gain.copy()
        
        self._gains_computed = True
        return success
    
    def _validate_all_gains(self) -> bool:
        """Validate that all computed gains produce stable observers"""
        for vertex in self.vertices:
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex)
            L = self.vertex_gains.get(vertex.to_tuple())
            
            if L is None:
                return False
            
            if not validate_observer_gain(A, C, L, max_real_part=0.0):
                if self.verbose:
                    print(f"Warning: Gain at vertex {vertex} is unstable")
                return False
        
        return True
    
    def _compute_gains_common_lyapunov(self, n: int, m: int) -> bool:
        """
        Compute gains with a single common Lyapunov matrix (robust qLPV).
        
        Solves:
            min trace(P) + γ_reg * Σ ||Y_i||_F
            s.t. P > 0
                 P < P_max * I
                 A_i^T P + P A_i - C_i^T Y_i^T - Y_i C_i + γ P < 0  ∀i
        
        Then L_i = P^{-1} Y_i for each vertex i.
        """
        # Common Lyapunov matrix
        P = cp.Variable((n, n), symmetric=True)
        
        # Separate Y for each vertex (allows different gains)
        Y_list = [cp.Variable((n, m)) for _ in range(self.n_vertices)]
        
        # Bounds for numerical stability
        P_min = 1e-4
        P_max = 1e4
        
        constraints = [
            P >> P_min * np.eye(n),
            P << P_max * np.eye(n),
        ]
        
        for i, vertex in enumerate(self.vertices):
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex)
            Y = Y_list[i]
            
            # LMI: A^T P + P A - C^T Y^T - Y C + γ P < 0
            lmi = A.T @ P + P @ A - C.T @ Y.T - Y @ C + self.decay_rate * P
            constraints.append(lmi << -1e-5 * np.eye(n))
        
        # Objective: minimize trace(P) with regularization on Y
        gamma_reg = 0.001
        reg_term = sum(cp.norm(Y, 'fro') for Y in Y_list)
        objective = cp.Minimize(cp.trace(P) + gamma_reg * reg_term)
        
        problem = cp.Problem(objective, constraints)
        
        try:
            problem.solve(solver=cp.SCS, verbose=self.verbose, max_iters=10000, eps=1e-5)
        except Exception as e:
            if self.verbose:
                print(f"SCS solver failed: {e}")
            try:
                problem.solve(solver=cp.CVXOPT, verbose=self.verbose)
            except Exception as e2:
                print(f"LMI solver failed: {e}, {e2}")
                return False
        
        if problem.status not in ['optimal', 'optimal_inaccurate']:
            if self.verbose:
                print(f"Polytopic LMI infeasible: {problem.status}")
            return False
        
        # Extract results
        self.P_common = P.value
        
        if self.P_common is None:
            return False
        
        for i, vertex in enumerate(self.vertices):
            Y_val = Y_list[i].value
            if Y_val is None:
                return False
            
            try:
                L = np.linalg.solve(self.P_common, Y_val)
                # Limit gain magnitude
                L = np.clip(L, -50.0, 50.0)
                self.vertex_gains[vertex.to_tuple()] = L
            except np.linalg.LinAlgError:
                L = np.linalg.pinv(self.P_common) @ Y_val
                L = np.clip(L, -50.0, 50.0)
                self.vertex_gains[vertex.to_tuple()] = L
        
        self._gains_computed = True
        return True
    
    def _compute_gains_independent(self, n: int, m: int) -> bool:
        """Compute independent gains for each vertex (less robust but simpler)"""
        success = True
        
        for vertex in self.vertices:
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex)
            
            try:
                L = compute_lmi_observer_gain(A, C, decay_rate=self.decay_rate, 
                                              verbose=self.verbose)
                self.vertex_gains[vertex.to_tuple()] = L
            except Exception as e:
                if self.verbose:
                    print(f"Warning: Failed to compute gain at vertex {vertex}: {e}")
                # Use pole placement fallback for this vertex
                try:
                    L = compute_pole_placement_gain(A, C)
                    self.vertex_gains[vertex.to_tuple()] = L
                except:
                    self.vertex_gains[vertex.to_tuple()] = self._default_gain.copy()
                    success = False
        
        self._gains_computed = True
        return success
    
    def compute_interpolation_weights(self, vx: float, delta: float) -> np.ndarray:
        """
        Compute convex interpolation weights for current scheduling parameters.
        
        Uses Gaussian-like weighting based on distance to each vertex for 
        smoother interpolation than inverse-distance.
        
        Args:
            vx: Current longitudinal velocity
            delta: Current steering angle
            
        Returns:
            weights: Array of weights α_i such that Σ α_i = 1, α_i ≥ 0
        """
        # Clamp to polytope bounds
        vx = np.clip(vx, self.vx_range[0], self.vx_range[1])
        delta = np.clip(delta, -self.delta_max, self.delta_max)
        
        # Normalize parameters to [0, 1] for distance computation
        vx_range = self.vx_range[1] - self.vx_range[0]
        delta_range = 2 * self.delta_max
        
        vx_norm = (vx - self.vx_range[0]) / vx_range if vx_range > 0 else 0.5
        delta_norm = (delta + self.delta_max) / delta_range if delta_range > 0 else 0.5
        
        current_point = np.array([vx_norm, delta_norm])
        
        # Compute Gaussian weights (smoother than inverse distance)
        sigma = 0.3  # Width of Gaussian
        weights = np.zeros(self.n_vertices)
        
        for i, vertex in enumerate(self.vertices):
            vx_v_norm = (vertex.vx - self.vx_range[0]) / vx_range if vx_range > 0 else 0.5
            delta_v_norm = (vertex.delta + self.delta_max) / delta_range if delta_range > 0 else 0.5
            vertex_point = np.array([vx_v_norm, delta_v_norm])
            
            dist_sq = np.sum((current_point - vertex_point) ** 2)
            weights[i] = np.exp(-dist_sq / (2 * sigma ** 2))
        
        # Normalize weights
        weight_sum = np.sum(weights)
        if weight_sum > 0:
            weights = weights / weight_sum
        else:
            weights = np.ones(self.n_vertices) / self.n_vertices
        
        return weights
    
    def get_scheduled_gain(self, vx: float, delta: float) -> np.ndarray:
        """
        Get interpolated observer gain for current operating point.
        
        L(ρ) = Σ α_i(ρ) · L_i
        
        Args:
            vx: Current longitudinal velocity
            delta: Current steering angle
            
        Returns:
            L: Interpolated observer gain matrix (6 × 6)
        """
        if not self._gains_computed:
            return self._default_gain.copy()
        
        # Check cache
        current_rho = (round(vx, 3), round(delta, 3))
        if self._last_rho == current_rho and self._last_weights is not None:
            weights = self._last_weights
        else:
            weights = self.compute_interpolation_weights(vx, delta)
            self._last_weights = weights
            self._last_rho = current_rho
        
        # Interpolate gains
        L = np.zeros((6, 6))
        for i, vertex in enumerate(self.vertices):
            gain = self.vertex_gains.get(vertex.to_tuple(), self._default_gain)
            L += weights[i] * gain
        
        return L
    
    def get_vertex_info(self) -> str:
        """Return string description of polytope vertices"""
        lines = [f"qLPV Gain Scheduler: {self.n_vertices} vertices"]
        lines.append(f"  vx range: [{self.vx_range[0]:.2f}, {self.vx_range[1]:.2f}] m/s")
        lines.append(f"  delta range: [{-self.delta_max:.2f}, {self.delta_max:.2f}] rad")
        lines.append(f"  Common Lyapunov: {self.use_common_lyapunov}")
        lines.append(f"  Decay rate: {self.decay_rate}")
        lines.append(f"  Gains computed: {self._gains_computed}")
        return "\n".join(lines)


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
        
        # Create differentiator using factory function
        diff_kwargs = diff_params or {}
        if diff_type == 'dirty':
            diff_kwargs.setdefault('tau', tau_rdot)
        self.rdot_filt = create_differentiator(diff_type=diff_type, Ts=Ts, y0=0.0, **diff_kwargs)
        
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


# Note: SchedulingParameters is imported from qlpv_vehicle_dynamics_obs


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
                 use_lmi_gains: bool = True,
                 use_gain_scheduling: bool = True,
                 lmi_decay_rate: float = 0.5,
                 vx_range: Tuple[float, float] = (0.5, 3.0),
                 delta_max: float = 0.4,
                 n_vx_vertices: int = 3,
                 n_delta_vertices: int = 3,
                 use_common_lyapunov: bool = True,
                 **kwargs):
        """
        Initialize Differentiator + UIO-Style Observer with qLPV Gain Scheduling
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict with keys:
                - 'lf': Distance from CG to front axle [m]
                - 'lr': Distance from CG to rear axle [m]
                - 'm': Vehicle mass [kg]
                - 'Iz': Yaw moment of inertia [kg·m²]
                - 'Cf': Front cornering stiffness [N/rad]
                - 'Cr': Rear cornering stiffness [N/rad]
            observer_gains: Observer gain parameters (ignored if use_lmi_gains=True)
            tau_rdot: Time constant for rdot filtering [s] (for dirty derivative)
            ridge: Ridge regularization for w estimation
            diff_type: Differentiator type ('dirty', 'highgain', 'sliding')
            diff_params: Additional parameters for the differentiator
            use_lmi_gains: If True, compute observer gains using LMI (requires cvxpy)
            use_gain_scheduling: If True, use polytopic qLPV gain scheduling
                                 If False, use single LMI gain at nominal point
            lmi_decay_rate: Minimum decay rate γ for LMI-based gain design
            vx_range: (vx_min, vx_max) velocity range for gain scheduling [m/s]
            delta_max: Maximum steering angle for gain scheduling [rad]
            n_vx_vertices: Number of velocity grid points for polytope
            n_delta_vertices: Number of steering grid points for polytope
            use_common_lyapunov: Use single Lyapunov matrix for robust stability
        """
        # Initialize base class with 6D state
        super().__init__(
            state_dim=self.STATE_DIM,
            unknown_input_dim=2,  # [w_r, w_f]
            sample_time=sample_time
        )
        
        # Store diff type
        self.diff_type = diff_type
        
        # Vehicle parameters (use centralized defaults)
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
        
        # Centralized qLPV dynamics for matrix computation
        self._dynamics = QLPVVehicleDynamicsObs(self.params, min_vx=0.5)
        
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
        
        # LMI and gain scheduling parameters
        self.use_lmi_gains = use_lmi_gains
        self.use_gain_scheduling = use_gain_scheduling
        self.lmi_decay_rate = lmi_decay_rate
        self.vx_range = vx_range
        self.delta_max = delta_max
        
        # Gain scheduler (initialized in _initialize_observer_gains)
        self._gain_scheduler: Optional[QLPVGainScheduler] = None
        
        # Observer gains (computed via LMI or default)
        self.observer_gains = observer_gains or self._default_gains()
        self._initialize_observer_gains(
            n_vx_vertices=n_vx_vertices,
            n_delta_vertices=n_delta_vertices,
            use_common_lyapunov=use_common_lyapunov
        )
        
        # Minimum velocity threshold
        self.min_vx = 0.5  # [m/s]
        
        # Diagnostics
        self.rdot_hat = 0.0
        self.residual = np.zeros(2)
        
        # Cache current scheduled gain
        self._current_L = self.L_state.copy()
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters (QCar scale) - uses centralized defaults"""
        return get_default_vehicle_params()
    
    def _default_gains(self) -> Dict:
        """Default observer gains - well-tuned for typical vehicle dynamics"""
        # Construct a well-tuned gain matrix for the measurement -> state mapping
        # Measurements: y = [vx, r, psi, X, Y, a_y]
        # States: x = [vx, vy, psi, r, X, Y]
        L = np.zeros((6, 6))
        
        # vx measurement (y[0]) -> vx state (x[0]) - direct, high gain
        L[0, 0] = 10.0
        
        # r measurement (y[1]) -> vy state (x[1]) - coupled through dynamics
        L[1, 1] = 5.0
        # r measurement also helps r state
        L[3, 1] = 10.0
        
        # psi measurement (y[2]) -> psi state (x[2]) - direct
        L[2, 2] = 10.0
        
        # X measurement (y[3]) -> X state (x[4]) - direct
        L[4, 3] = 5.0
        
        # Y measurement (y[4]) -> Y state (x[5]) - direct
        L[5, 4] = 5.0
        
        # a_y measurement (y[5]) -> vy state (x[1]) - lateral dynamics coupling
        L[1, 5] = 2.0
        
        return {
            'L_state': L,
        }
    
    def _initialize_observer_gains(self, n_vx_vertices: int = 3, 
                                    n_delta_vertices: int = 3,
                                    use_common_lyapunov: bool = True):
        """
        Initialize observer gain matrices with qLPV gain scheduling.
        
        If use_gain_scheduling=True and CVXPY is available, creates a polytopic
        gain scheduler that computes LMI-based gains at grid vertices and uses
        convex interpolation for real-time gain scheduling.
        
        If use_gain_scheduling=False, computes a single LMI gain at nominal point.
        
        Args:
            n_vx_vertices: Number of velocity grid points
            n_delta_vertices: Number of steering grid points
            use_common_lyapunov: Use common Lyapunov matrix for robust stability
        """
        gains = self.observer_gains
        
        # Check if custom gains were provided
        if isinstance(gains.get('L_state'), np.ndarray) and gains['L_state'].shape == (self.STATE_DIM, self.MEAS_DIM):
            self.L_state = gains['L_state']
            self._gain_method = 'custom'
            return
        
        # Try polytopic qLPV gain scheduling
        if self.use_gain_scheduling and self.use_lmi_gains and CVXPY_AVAILABLE:
            try:
                self._gain_scheduler = QLPVGainScheduler(
                    vehicle_params=self.params,
                    vx_range=self.vx_range,
                    delta_max=self.delta_max,
                    n_vx_vertices=n_vx_vertices,
                    n_delta_vertices=n_delta_vertices,
                    decay_rate=self.lmi_decay_rate,
                    use_common_lyapunov=use_common_lyapunov,
                    verbose=False
                )
                
                if self._gain_scheduler.compute_gains_lmi():
                    # Get nominal gain for L_state (used as fallback)
                    nominal_vx = (self.vx_range[0] + self.vx_range[1]) / 2
                    self.L_state = self._gain_scheduler.get_scheduled_gain(nominal_vx, 0.0)
                    self._gain_method = 'qlpv_scheduled'
                    return
                else:
                    print("Warning: Polytopic LMI failed, trying single-point LMI")
            except Exception as e:
                print(f"Warning: qLPV gain scheduling failed ({e}). Trying single-point LMI.")
        
        # Try single-point LMI-based gain design
        if self.use_lmi_gains and CVXPY_AVAILABLE:
            try:
                self.L_state = self._compute_lmi_gains()
                self._gain_method = 'lmi_single'
                return
            except Exception as e:
                print(f"Warning: LMI gain design failed ({e}). Using default gains.")
        
        # Try pole placement
        if SCIPY_AVAILABLE:
            try:
                nominal_vx = (self.vx_range[0] + self.vx_range[1]) / 2
                nominal_state = np.array([nominal_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
                rho = self.compute_scheduling_params(nominal_state, 0.0)
                A = self.compute_A_matrix(rho)
                C = self.compute_C_matrix()
                self.L_state = compute_pole_placement_gain(A, C)
                self._gain_method = 'pole_placement'
                return
            except Exception as e:
                pass
        
        # Fallback: well-tuned default gain
        self.L_state = self._default_gains()['L_state']
        self._gain_method = 'default'
    
    def _compute_lmi_gains(self) -> np.ndarray:
        """
        Compute observer gains using LMI-based design at a single nominal point.
        
        Uses a nominal operating point (straight driving at moderate speed)
        to compute the A matrix, then solves the Lyapunov LMI.
        
        Returns:
            L: Observer gain matrix (6 × 6)
        """
        # Nominal operating point: straight driving at mid-range vx
        nominal_vx = (self.vx_range[0] + self.vx_range[1]) / 2
        nominal_state = np.array([nominal_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
        nominal_delta = 0.0
        
        # Compute A matrix at nominal point
        rho = self.compute_scheduling_params(nominal_state, nominal_delta)
        A = self.compute_A_matrix(rho)
        
        # Output matrix C
        C = self.compute_C_matrix()
        
        # Solve LMI for observer gain
        L = compute_lmi_observer_gain(A, C, decay_rate=self.lmi_decay_rate, verbose=False)
        
        return L
    
    def get_scheduled_gain(self, vx: float, delta: float) -> np.ndarray:
        """
        Get observer gain for current operating point.
        
        If using qLPV gain scheduling, returns interpolated gain.
        Otherwise returns the fixed L_state gain.
        
        Args:
            vx: Current longitudinal velocity
            delta: Current steering angle
            
        Returns:
            L: Observer gain matrix (6 × 6)
        """
        if self._gain_scheduler is not None and self._gain_method == 'qlpv_scheduled':
            return self._gain_scheduler.get_scheduled_gain(vx, delta)
        else:
            return self.L_state
    
    def get_gain_scheduler_info(self) -> Optional[str]:
        """Get information about the gain scheduler configuration"""
        if self._gain_scheduler is not None:
            return self._gain_scheduler.get_vertex_info()
        return None
    
    def get_gain_method(self) -> str:
        """
        Return the method used to compute observer gains.
        
        Returns one of:
            - 'qlpv_scheduled': Polytopic qLPV gain scheduling with LMI
            - 'lmi_single': Single-point LMI-based gain
            - 'custom': User-provided custom gains
            - 'default': Default diagonal gains (fallback)
        """
        return getattr(self, '_gain_method', 'unknown')
    
    def get_current_gain(self) -> np.ndarray:
        """Get the currently active observer gain matrix"""
        return self._current_L.copy()
    
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
        Update observer with new measurement using qLPV gain scheduling.
        
        Observer equation:
            x̂[k+1] = x̂[k] + Ts·(A(ρ̂)·x̂ + B(ρ̂)·u + E(ρ̂)·ŵ + L(ρ̂)·(y - C·x̂))
        
        where L(ρ̂) is the scheduled observer gain based on current vx and δ.
        
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
        
        # Get current velocity for gain scheduling
        vx = max(abs(self.state_hat[self.IDX_VX]), self.min_vx)
        
        # Get scheduled observer gain L(ρ)
        self._current_L = self.get_scheduled_gain(vx, delta)
        
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
        
        # === State observer update with scheduled gain ===
        # Predicted output (without a_y for simplicity)
        y_pred = C @ self.state_hat
        
        # Innovation (measurement residual)
        innovation = y - y_pred
        # Set a_y innovation to 0 since it's used for w estimation
        innovation[self.MEAS_IDX_AY] = 0.0
        
        # State dynamics with scheduled gain (continuous-time)
        x_dot = A @ self.state_hat + B @ u + E @ self.w_hat + self._current_L @ innovation
        
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
