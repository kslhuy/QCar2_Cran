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
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM, AUGMENTED_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
)


# Note: SchedulingParameters is imported from qlpv_vehicle_dynamics_obs


# =============================================================================
# LMI-Based Observer Gain Design Functions
# =============================================================================

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
        desired_poles = np.array([-2.0, -2.5, -3.0, -3.5, -4.0, -4.5])
    
    if not SCIPY_AVAILABLE:
        # Simple fallback: use diagonal gain
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
    
    # LMI constraint
    eps = 1e-6
    lmi_constraint = A.T @ P + P @ A - C.T @ Y.T - Y @ C + decay_rate * P
    
    # Regularization
    gamma_reg = 0.01
    
    constraints = [
        P >> eps * np.eye(n),
        P << 1e4 * np.eye(n),
        lmi_constraint << -eps * np.eye(n),
    ]
    
    # Objective: minimize trace(P) + regularization on Y
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    # Solve the SDP
    problem = cp.Problem(objective, constraints)
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=10000, eps=1e-6)
    except Exception as e:
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
        L = np.linalg.pinv(P_val) @ Y_val
    
    # Validate the computed gain
    if not validate_observer_gain(A, C, L):
        raise ValueError("Computed gain does not produce stable observer")
    
    # Limit gain magnitude
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
                 delta_max: float = 0.33,
                 n_vx_vertices: int = 3,
                 n_delta_vertices: int = 3,
                 decay_rate: float = 1.0,
                 use_common_lyapunov: bool = True,
                 include_psi_scheduling: bool = False,
                 n_psi_vertices: int = 3,
                 use_hinf: bool = True,
                 hinf_gamma: float = 1.0,
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
            include_psi_scheduling: If True, include yaw angle in polytope vertices
            n_psi_vertices: Number of yaw angle grid points (if include_psi_scheduling=True)
            use_hinf: If True, add H∞ disturbance attenuation constraint
            hinf_gamma: H∞ attenuation level for tire residual disturbance
            verbose: Print solver output
        """
        self.params = vehicle_params
        self.vx_range = vx_range
        self.delta_max = delta_max
        self.decay_rate = decay_rate
        self.use_common_lyapunov = use_common_lyapunov
        self.include_psi_scheduling = include_psi_scheduling
        self.n_psi_vertices = n_psi_vertices
        self.use_hinf = use_hinf
        self.hinf_gamma = hinf_gamma
        self.verbose = verbose
        
        # Extract vehicle params
        self.lf = vehicle_params.get('lf', 0.11)
        self.lr = vehicle_params.get('lr', 0.11)
        self.m = vehicle_params.get('m', 2.5)
        self.Iz = vehicle_params.get('Iz', 0.02)
        self.Cf = vehicle_params.get('Cf', 50.0)
        self.Cr = vehicle_params.get('Cr', 50.0)
        self.mu = vehicle_params.get('mu', 0.01)  # Road friction coefficient
        self.g = 9.81  # Gravity [m/s²]
        
        # Centralized vehicle dynamics (single source of truth)
        self.dynamics = QLPVVehicleDynamicsObs(
            vehicle_params=vehicle_params,
            min_vx=vx_range[0]
        )
        
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
        """Compute a robust default gain using moderate settings"""
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
        
        # Include psi scheduling if enabled (for wide yaw angle operation)
        if self.include_psi_scheduling:
            psi_values = np.linspace(-np.pi/4, np.pi/4, self.n_psi_vertices)
        else:
            psi_values = [0.0]
        
        vertices = []
        for vx in vx_values:
            for delta in delta_values:
                for psi in psi_values:
                    vertices.append(PolytopicVertex(vx=vx, delta=delta, psi=psi))
        
        return vertices
    
    def _compute_A_at_vertex(self, vertex: PolytopicVertex) -> np.ndarray:
        """Compute A matrix at a polytope vertex - delegates to centralized dynamics"""
        # Create dummy state from vertex for scheduling parameters
        x_dummy = np.array([vertex.vx, 0.0, vertex.psi, 0.0, 0.0, 0.0])
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        return self.dynamics.compute_A_matrix(rho)
    
    def _compute_C_at_vertex(self, vertex: PolytopicVertex) -> np.ndarray:
        """Compute C matrix at a polytope vertex - delegates to centralized dynamics"""
        x_dummy = np.array([vertex.vx, 0.0, vertex.psi, 0.0, 0.0, 0.0])
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        return self.dynamics.compute_C_matrix(rho)
    
    def _compute_E_at_vertex(self, vertex: PolytopicVertex) -> np.ndarray:
        """Compute E matrix at a polytope vertex - delegates to centralized dynamics"""
        x_dummy = np.array([vertex.vx, 0.0, vertex.psi, 0.0, 0.0, 0.0])
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        return self.dynamics.compute_E_matrix(rho)
    
    def _compute_F_at_vertex(self, vertex: PolytopicVertex) -> np.ndarray:
        """Compute F matrix at a polytope vertex - delegates to centralized dynamics"""
        x_dummy = np.array([vertex.vx, 0.0, vertex.psi, 0.0, 0.0, 0.0])
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        return self.dynamics.compute_F_matrix(rho)
    
    
    def _check_observability(self, A: np.ndarray, C: np.ndarray) -> Tuple[bool, int]:
        """Check if (A, C) pair is observable
        
        Computes the observability matrix O = [C; CA; CA²; ...; CA^(n-1)]
        and checks if it has full column rank.
        
        Args:
            A: State matrix (n × n)
            C: Output matrix (m × n)
            
        Returns:
            Tuple of (is_observable, rank)
        """
        n = A.shape[0]
        O = C.copy()
        A_power = np.eye(n)
        
        for i in range(1, n):
            A_power = A_power @ A
            O = np.vstack([O, C @ A_power])
        
        rank = np.linalg.matrix_rank(O)
        return rank == n, rank

    
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
        """Fallback: compute gains using pole placement at each vertex
        
        Checks observability before attempting pole placement. If unobservable,
        uses a structured high-gain fallback instead of simple diagonal.
        """
        success = True
        
        for vertex in self.vertices:
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex)
            
            # Check observability
            is_obs, rank = self._check_observability(A, C)
            if not is_obs and self.verbose:
                print(f"Warning: System unobservable at vertex {vertex} (rank={rank}/6)")
            
            try:
                if is_obs:
                    L = compute_pole_placement_gain(A, C)
                    if validate_observer_gain(A, C, L):
                        self.vertex_gains[vertex.to_tuple()] = L
                    else:
                        # Even with observability, pole placement may fail
                        self.vertex_gains[vertex.to_tuple()] = self._compute_structured_fallback_gain(vertex)
                else:
                    # Unobservable: use structured fallback
                    self.vertex_gains[vertex.to_tuple()] = self._compute_structured_fallback_gain(vertex)
            except Exception as e:
                if self.verbose:
                    print(f"Pole placement failed at {vertex}: {e}")
                self.vertex_gains[vertex.to_tuple()] = self._compute_structured_fallback_gain(vertex)
        
        self._gains_computed = True
        return success
    
    def _compute_structured_fallback_gain(self, vertex: PolytopicVertex) -> np.ndarray:
        """Compute a structured high-gain fallback for unobservable/failed cases
        
        Uses higher gains for directly measured states and physics-based coupling.
        """
        L = np.zeros((6, 6))
        
        # Measurement y = [vx, r, psi, X, Y, ay] -> State x = [vx, vy, psi, r, X, Y]
        # High gain for direct measurements
        L[0, 0] = 10.0  # vx from y[0]=vx
        L[2, 2] = 10.0  # psi from y[2]=psi  
        L[3, 1] = 10.0  # r from y[1]=r
        L[4, 3] = 5.0   # X from y[3]=X
        L[5, 4] = 5.0   # Y from y[4]=Y
        
        # Coupled gains for unobserved vy using r and ay
        L[1, 1] = 3.0   # vy from r (dynamics coupling)
        L[1, 5] = 5.0   # vy from ay (direct observation of lateral dynamics)
        
        return L
    
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
        
        If use_hinf=True, also adds H∞ bounded real lemma constraint for
        disturbance attenuation:
            [[A^T P + PA - C^T Y^T - YC + γP,   PE - YF],
             [(PE - YF)^T,                      -γ²I   ]] < 0
        
        where E maps disturbance to state and F maps disturbance to output.
        
        Then L_i = P^{-1} Y_i for each vertex i.
        """
        # Try H∞ first, fall back to standard if it fails
        if self.use_hinf:
            success = self._solve_hinf_lmi(n, m)
            if success:
                return True
            if self.verbose:
                print("H∞ LMI failed, trying standard stability LMI...")
        
        # Standard stability LMI (no H∞ constraint)
        return self._solve_standard_lmi(n, m)
    
    def _solve_standard_lmi(self, n: int, m: int) -> bool:
        """Solve standard stability LMI without H∞ constraint"""
        # Common Lyapunov matrix
        P = cp.Variable((n, n), symmetric=True)
        
        # Separate Y for each vertex
        Y_list = [cp.Variable((n, m)) for _ in range(self.n_vertices)]
        
        # Bounds for numerical stability
        P_min = 1e-3  # Increased from 1e-4 for better conditioning
        P_max = 1e3   # Decreased from 1e4 for better conditioning
        
        constraints = [
            P >> P_min * np.eye(n),
            P << P_max * np.eye(n),
        ]
        
        for i, vertex in enumerate(self.vertices):
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex)
            Y = Y_list[i]
            
            # Standard stability LMI: A^T P + PA - C^T Y^T - YC + γP < 0
            lmi = A.T @ P + P @ A - C.T @ Y.T - Y @ C + self.decay_rate * P
            constraints.append(lmi << -1e-4 * np.eye(n))
        
        # Objective: minimize trace(P) with regularization on Y
        gamma_reg = 0.01  # Increased regularization
        reg_term = sum(cp.norm(Y, 'fro') for Y in Y_list)
        objective = cp.Minimize(cp.trace(P) + gamma_reg * reg_term)
        
        return self._solve_and_extract(P, Y_list, objective, constraints, n)
    
    def _solve_hinf_lmi(self, n: int, m: int) -> bool:
        """Solve H∞ bounded real lemma LMI with disturbance rejection"""
        # Common Lyapunov matrix
        P = cp.Variable((n, n), symmetric=True)
        
        # Separate Y for each vertex
        Y_list = [cp.Variable((n, m)) for _ in range(self.n_vertices)]
        
        # Bounds for numerical stability
        P_min = 1e-3
        P_max = 1e3
        
        constraints = [
            P >> P_min * np.eye(n),
            P << P_max * np.eye(n),
        ]
        
        # Disturbance dimension (2 for tire residuals [w_r, w_f])
        p = 2
        
        for i, vertex in enumerate(self.vertices):
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex)
            E = self._compute_E_at_vertex(vertex)
            F = self._compute_F_at_vertex(vertex)
            Y = Y_list[i]
            
            # Build 2x2 block LMI:
            # [[A^T P + PA - C^T Y^T - YC + γP,   PE - YF],
            #  [(PE - YF)^T,                      -γ²I   ]] < 0
            block_11 = A.T @ P + P @ A - C.T @ Y.T - Y @ C + self.decay_rate * P
            block_12 = P @ E - Y @ F
            block_22 = -self.hinf_gamma**2 * np.eye(p)
            
            # Construct block matrix using cp.bmat
            hinf_lmi = cp.bmat([
                [block_11, block_12],
                [block_12.T, block_22]
            ])
            constraints.append(hinf_lmi << -1e-4 * np.eye(n + p))
        
        # Objective: minimize trace(P) with regularization on Y
        gamma_reg = 0.01
        reg_term = sum(cp.norm(Y, 'fro') for Y in Y_list)
        objective = cp.Minimize(cp.trace(P) + gamma_reg * reg_term)
        
        return self._solve_and_extract(P, Y_list, objective, constraints, n)
    
    def _solve_and_extract(self, P, Y_list, objective, constraints, n: int) -> bool:
        """Solve the SDP and extract gains, with proper validation"""
        problem = cp.Problem(objective, constraints)
        
        # Try multiple solvers with increasing precision
        solvers_to_try = [
            (cp.SCS, {'max_iters': 20000, 'eps': 1e-7}),
            (cp.SCS, {'max_iters': 50000, 'eps': 1e-8}),
        ]
        
        # Add CVXOPT if available
        try:
            import cvxopt
            solvers_to_try.append((cp.CVXOPT, {}))
        except ImportError:
            pass
        
        solved = False
        for solver, solver_opts in solvers_to_try:
            try:
                problem.solve(solver=solver, verbose=self.verbose, **solver_opts)
                if problem.status == 'optimal':
                    solved = True
                    break
                elif problem.status == 'optimal_inaccurate':
                    # Check if P is actually positive definite
                    if P.value is not None:
                        min_eig = np.min(np.linalg.eigvalsh(P.value))
                        if min_eig > 1e-4:
                            solved = True
                            break
                        elif self.verbose:
                            print(f"P not positive definite (min_eig={min_eig:.4e}), trying better solver...")
            except Exception as e:
                if self.verbose:
                    print(f"Solver {solver} failed: {e}")
                continue
        
        if not solved:
            if self.verbose:
                print("All solvers failed to produce valid solution")
            return False
        
        # Extract and validate P
        self.P_common = P.value
        
        if self.P_common is None:
            return False
        
        # Final validation: P must be positive definite
        min_eig = np.min(np.linalg.eigvalsh(self.P_common))
        if min_eig < 1e-6:
            if self.verbose:
                print(f"P has invalid eigenvalue: {min_eig:.4e}")
            return False
        
        # Extract gains
        for i, vertex in enumerate(self.vertices):
            Y_val = Y_list[i].value
            if Y_val is None:
                return False
            
            try:
                L = np.linalg.solve(self.P_common, Y_val)
                L = np.clip(L, -100.0, 100.0)  # Increased clip range
                self.vertex_gains[vertex.to_tuple()] = L
            except np.linalg.LinAlgError:
                L = np.linalg.pinv(self.P_common) @ Y_val
                L = np.clip(L, -100.0, 100.0)
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
        
        Uses Gaussian-like weighting for smooth interpolation.
        
        Args:
            vx: Current longitudinal velocity
            delta: Current steering angle
            
        Returns:
            weights: Array of weights α_i such that Σ α_i = 1, α_i ≥ 0
        """
        # Clamp to polytope bounds
        vx = np.clip(vx, self.vx_range[0], self.vx_range[1])
        delta = np.clip(delta, -self.delta_max, self.delta_max)
        
        # Normalize parameters to [0, 1]
        vx_range = self.vx_range[1] - self.vx_range[0]
        delta_range = 2 * self.delta_max
        
        vx_norm = (vx - self.vx_range[0]) / vx_range if vx_range > 0 else 0.5
        delta_norm = (delta + self.delta_max) / delta_range if delta_range > 0 else 0.5
        
        current_point = np.array([vx_norm, delta_norm])
        
        # Compute Gaussian weights
        sigma = 0.3
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
    
    def get_all_gains_summary(self) -> str:
        """Return summary of all computed gains at vertices"""
        lines = ["\nComputed Observer Gains at Polytope Vertices:"]
        lines.append("=" * 60)
        
        if not self._gains_computed:
            lines.append("  Gains not yet computed. Call compute_gains_lmi() first.")
            return "\n".join(lines)
        
        for i, vertex in enumerate(self.vertices):
            L = self.vertex_gains.get(vertex.to_tuple())
            if L is not None:
                lines.append(f"\nVertex {i+1}: vx={vertex.vx:.2f} m/s, δ={vertex.delta:.2f} rad")
                lines.append(f"  Gain L ({L.shape[0]}×{L.shape[1]}):")
                lines.append(f"    Eigenvalues of (A-LC): {self._get_closed_loop_eigs(vertex, L)}")
                lines.append(f"    Gain matrix diagonal: {np.diag(L)}")
                lines.append(f"    Frobenius norm: {np.linalg.norm(L, 'fro'):.4f}")
        
        if self.P_common is not None:
            lines.append(f"\nCommon Lyapunov matrix P:")
            lines.append(f"  Condition number: {np.linalg.cond(self.P_common):.2e}")
            lines.append(f"  Min eigenvalue: {np.min(np.linalg.eigvalsh(self.P_common)):.4f}")
            lines.append(f"  Max eigenvalue: {np.max(np.linalg.eigvalsh(self.P_common)):.4f}")
        
        return "\n".join(lines)
    
    def _get_closed_loop_eigs(self, vertex: PolytopicVertex, L: np.ndarray) -> str:
        """Get closed-loop eigenvalues for a vertex"""
        A = self._compute_A_at_vertex(vertex)
        C = self._compute_C_at_vertex(vertex)
        A_cl = A - L @ C
        eigs = np.linalg.eigvals(A_cl)
        real_parts = np.real(eigs)
        return f"[{', '.join([f'{r:.2f}' for r in sorted(real_parts)])}]"


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
                 use_gain_scheduling: bool = False,
                 lmi_decay_rate: float = 0.5,
                 vx_range: Tuple[float, float] = (0.5, 3.0),
                 delta_max: float = 0.4,
                 n_vx_vertices: int = 3,
                 n_delta_vertices: int = 3,
                 verbose: bool = False,
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
            use_gain_scheduling: Whether to use polytopic qLPV gain scheduling
            lmi_decay_rate: Decay rate for LMI observer gain design
            vx_range: Velocity range for gain scheduling (vx_min, vx_max)
            delta_max: Maximum steering angle for gain scheduling
            n_vx_vertices: Number of velocity grid points for polytope
            n_delta_vertices: Number of steering grid points for polytope
            verbose: Print solver/debug output
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
        
        # Observer gains and scheduling
        self.observer_gains = observer_gains or self._default_gains()
        self.use_gain_scheduling = use_gain_scheduling
        self.lmi_decay_rate = lmi_decay_rate
        self.verbose = verbose
        
        # Initialize gain scheduler if enabled
        self.gain_scheduler: Optional[QLPVGainScheduler] = None
        if use_gain_scheduling:
            self.gain_scheduler = QLPVGainScheduler(
                vehicle_params=self.params,
                vx_range=vx_range,
                delta_max=delta_max,
                n_vx_vertices=n_vx_vertices,
                n_delta_vertices=n_delta_vertices,
                decay_rate=lmi_decay_rate,
                use_common_lyapunov=True,
                verbose=verbose
            )
            # Compute LMI gains at polytope vertices
            self.gain_scheduler.compute_gains_lmi()
        
        self._initialize_observer_gains()
        
        # Gyro bias estimation (optional)
        self.include_gyro_bias = include_gyro_bias
        self.gyro_bias = 0.0
        
        # UIO residual for a_y constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0  # m·ã_y ≈ w_r + cos(δ)·w_f
        
        # Minimum velocity threshold
        self.min_vx = 0.5  # [m/s]
        
        # Centralized vehicle dynamics (single source of truth)
        self.dynamics = QLPVVehicleDynamicsObs(
            vehicle_params=self.params,
            min_vx=self.min_vx
        )
    
    def _default_params(self) -> Dict:
        """Default vehicle parameters - uses centralized defaults"""
        return get_default_vehicle_params()
    
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
            # Higher gains for faster residual convergence
            self.L_residual = np.zeros((2, self.MEAS_DIM))
            self.L_residual[0, self.MEAS_IDX_AY] = 2.0  # w_r from a_y
            self.L_residual[1, self.MEAS_IDX_AY] = 2.0  # w_f from a_y
        
        # Augmented observer gain (8×6) = stack of L_state (6×6) and L_residual (2×6)
        self.L_augmented = np.vstack([self.L_state, self.L_residual])
    
    def _get_scheduled_gain(self, vx: float, delta: float) -> np.ndarray:
        """
        Get observer gain for current operating point.
        
        If gain scheduling is enabled, uses the polytopic LMI gain scheduler
        to interpolate gains based on current (vx, delta). Otherwise returns
        the default static observer gain.
        
        Args:
            vx: Current longitudinal velocity [m/s]
            delta: Current steering angle [rad]
            
        Returns:
            L_state: Observer gain matrix (6×6) for state estimation
        """
        if self.use_gain_scheduling and self.gain_scheduler is not None:
            # Get interpolated LMI gain from gain scheduler
            return self.gain_scheduler.get_scheduled_gain(vx, delta)
        else:
            # Use default static gain
            return self.L_state
    
    def compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        """Compute scheduling parameters from current state and input - delegates to centralized dynamics"""
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
    
    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update qLPV augmented-state observer with new measurement
        
        Uses predict-then-correct structure:
            1. Predict: x̂_a⁻ = f(x̂_a, u) using nonlinear dynamics
            2. Correct: x̂_a = x̂_a⁻ + L·(y - h(x̂_a⁻, u))
        
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
        
        # Extract current state and residuals
        x = self.state_augmented[:self.STATE_DIM]
        w = self.state_augmented[self.STATE_DIM:]
        
        # =====================================================
        # PREDICT: Use nonlinear dynamics (like EKF)
        # =====================================================
        # State derivative from centralized nonlinear dynamics
        x_dot = self.dynamics.f_continuous(x, u, w)
        
        # Tire residual dynamics (random-walk: ẇ = 0)
        w_dot = np.zeros(2)
        
        # Euler discretization for prediction
        xa_pred = np.zeros(self.AUGMENTED_DIM)
        xa_pred[:self.STATE_DIM] = x + self.Ts * x_dot
        xa_pred[self.STATE_DIM:] = w + self.Ts * w_dot
        
        # =====================================================
        # CORRECT: Use measurement innovation with scheduled gain
        # =====================================================
        # Predicted measurement using nonlinear measurement function
        y_pred = self.dynamics.h_meas(xa_pred[:self.STATE_DIM], u, xa_pred[self.STATE_DIM:])
        
        # Innovation (measurement residual)
        innovation = y - y_pred
        
        # Get observer gain (scheduled if enabled, otherwise static)
        rho = self.compute_scheduling_params(xa_pred[:self.STATE_DIM], delta)
        L_state = self._get_scheduled_gain(rho.vx, delta)
        
        # Build augmented gain matrix: stack L_state (6×6) and L_residual (2×6)
        L_augmented = np.vstack([L_state, self.L_residual])
        
        # Correction step (scale by Ts for proper discrete-time application of continuous-time gain)
        # Continuous-time: ẋ = Ax + Bu + L(y - Cx)
        # Discretized: x[k+1] = x_pred + Ts * L * innovation
        self.state_augmented = xa_pred + self.Ts * L_augmented @ innovation
        
        # Clamp states to prevent numerical overflow
        # State: [vx, vy, psi, r, X, Y, wr, wf]
        self.state_augmented[0] = np.clip(self.state_augmented[0], -10.0, 10.0)  # vx
        self.state_augmented[1] = np.clip(self.state_augmented[1], -5.0, 5.0)    # vy
        self.state_augmented[3] = np.clip(self.state_augmented[3], -10.0, 10.0)  # r
        self.state_augmented[6] = np.clip(self.state_augmented[6], -500.0, 500.0)  # wr
        self.state_augmented[7] = np.clip(self.state_augmented[7], -500.0, 500.0)  # wf
        
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
    
    def get_gain_scheduler(self) -> Optional[QLPVGainScheduler]:
        """
        Get the gain scheduler if enabled
        
        Returns:
            QLPVGainScheduler instance or None if not using gain scheduling
        """
        return self.gain_scheduler
    
    def get_gains_summary(self) -> str:
        """
        Get summary of observer gains
        
        Returns:
            String description of gains (from scheduler if enabled, or default)
        """
        if self.gain_scheduler is not None:
            return self.gain_scheduler.get_all_gains_summary()
        else:
            lines = ["Observer Gains (default, no scheduling):"]
            lines.append("=" * 60)
            lines.append(f"\nL_state ({self.L_state.shape[0]}×{self.L_state.shape[1]}):")
            lines.append(f"  Diagonal: {np.diag(self.L_state)}")
            lines.append(f"  Frobenius norm: {np.linalg.norm(self.L_state, 'fro'):.4f}")
            lines.append(f"\nL_residual ({self.L_residual.shape[0]}×{self.L_residual.shape[1]}):")
            lines.append(f"  {self.L_residual}")
            lines.append(f"\nL_augmented ({self.L_augmented.shape[0]}×{self.L_augmented.shape[1]}):")
            lines.append(f"  Frobenius norm: {np.linalg.norm(self.L_augmented, 'fro'):.4f}")
            return "\n".join(lines)


def create_qlpv_observer(sample_time: float = 0.02,
                         vehicle_params: Optional[Dict] = None,
                         use_gain_scheduling: bool = False,
                         **kwargs) -> qLPVAugmentedObserver:
    """
    Factory function to create qLPV augmented-state observer
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dictionary
        use_gain_scheduling: Enable polytopic qLPV gain scheduling
        **kwargs: Additional arguments passed to constructor
        
    Returns:
        Configured qLPVAugmentedObserver instance
    """
    return qLPVAugmentedObserver(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        use_gain_scheduling=use_gain_scheduling,
        **kwargs
    )
