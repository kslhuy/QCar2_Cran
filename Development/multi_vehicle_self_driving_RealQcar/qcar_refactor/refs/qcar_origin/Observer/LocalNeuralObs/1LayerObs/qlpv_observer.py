"""
qLPV Augmented-State Observer for First-Layer Observer Architecture

Implements a quasi-Linear Parameter-Varying (qLPV) augmented-state observer
with tire-residual estimation for vehicle state and disturbance estimation.
Uses LMI-based polytopic gain scheduling for robust observer gain design.

State: x = [v_x, v_y, psi, r, X, Y]^T (6D)
Augmented state: x_a = [x; w_r; w_f]^T (8D with tire residuals)
Measurements: y = [v_x, r, psi, X, Y, a_y]^T (6D with lateral acceleration)

Observer Equation (predict-correct with LMI-scheduled gain):
    Predict:
        x_hat_a^- = A_d(rho) * x_hat_a + B_d(rho) * u    (ZOH discretized)

    Correct:
        x_hat_a = x_hat_a^- + L(rho) * (y - C_a(rho) * x_hat_a^- - D(rho) * u)

where:
    - x_hat_a = [x_hat; w_hat_r; w_hat_f] is the augmented state estimate
    - rho = {1/v_x, sin(delta), cos(delta), v_x, v_y, sin(psi), cos(psi)}
    - w = [w_r, w_f] are tire force residuals (unknown inputs)
    - a_y provides algebraic constraint on w
    - L(rho) is the LMI-scheduled observer gain (polytopic interpolation)

References:
    - qLPV vehicle dynamics with tire-residual estimation
    - UIO (Unknown Input Observer) for disturbance estimation
    - Polytopic LMI gain scheduling for robust observer design
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

# Import scipy for matrix operations
try:
    from scipy.linalg import expm
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
        def __init__(self, state_dim: int = 4, unknown_input_dim: int = 2, sample_time: float = 0.01):
            self.state_dim = state_dim
            self.unknown_input_dim = unknown_input_dim
            self.Ts = sample_time
            self.state_hat = np.zeros(state_dim)
            self.f_uk_hat = np.zeros(unknown_input_dim)

# Import centralized qLPV vehicle dynamics
sys.path.insert(0, str(parent_dir.parent))
from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Observer.LocalNeuralObs.qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    QLPVVehicleDynamicsObs8D,
    create_qlpv_dynamics,
    get_default_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM, AUGMENTED_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
    IDX8_VX, IDX8_VY, IDX8_PSI, IDX8_R, IDX8_X, IDX8_Y, IDX8_AX, IDX8_AY, STATE_DIM_8D,
    MEAS8_IDX_VX, MEAS8_IDX_R, MEAS8_IDX_PSI, MEAS8_IDX_X, MEAS8_IDX_Y, MEAS8_IDX_AY, MEAS8_IDX_AX, MEAS_DIM_7D,
    AUGMENTED_DIM_10D,
)

# Optional differentiator for r_dot pseudo-measurement
try:
    from differentiators import create_differentiator_from_config
    DIFFERENTIATOR_AVAILABLE = True
except ImportError:
    DIFFERENTIATOR_AVAILABLE = False

# Import NeuralQLPVGainScheduler from 2LayerObs (reuse for first layer)
try:
    layer2_dir = parent_dir.parent / '2LayerObs'
    sys.path.insert(0, str(layer2_dir))
    from Design_LMI_neural import (
        NeuralQLPVGainScheduler,
        NeuralPolytopicVertex,
        discretize_system_zoh,
        compute_discrete_hinf_lmi_observer_gain,
        compute_discrete_l2_lmi_observer_gain,
        compute_discrete_h2_lmi_observer_gain,
        compute_discrete_contraction_lmi,
        validate_discrete_observer_gain,
    )
    NEURAL_SCHEDULER_AVAILABLE = True
except ImportError as _e:
    NEURAL_SCHEDULER_AVAILABLE = False
    print(f"Warning: NeuralQLPVGainScheduler not available: {_e}")


# Note: SchedulingParameters is imported from qlpv_vehicle_dynamics_obs


# =============================================================================
# LMI-Based Observer Gain Design Functions
# =============================================================================

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
        e_dot = (A - LC) e

    We solve for P > 0 and Y = P @ L such that:
        A^T P + PA - C^T Y^T - YC + gamma*P < 0

    where gamma (decay_rate) controls the minimum exponential decay rate.

    After solving, recover: L = P^{-1} @ Y

    Args:
        A: State matrix (n x n)
        C: Output matrix (m x n)
        decay_rate: Minimum decay rate gamma > 0 (larger = faster convergence)
        verbose: Print solver output

    Returns:
        L: Observer gain matrix (n x m)

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

    # Recover observer gain: L = P^{-1} @ Y
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

    For a qLPV system with scheduling parameters rho in Omega, we design
    observer gains at the vertices of a polytope covering Omega, then use
    convex interpolation for real-time gain scheduling.

    The polytope vertices are defined by:
        - vx in [vx_min, vx_max]
        - delta in [-delta_max, delta_max]

    For each vertex i, we compute L_i such that (A_i - L_i @ C_i) is Hurwitz.
    A common Lyapunov matrix P ensures stability across the entire polytope.

    Real-time gain: L(rho) = Sum alpha_i(rho) * L_i  where Sum alpha_i = 1, alpha_i >= 0

    Reference:
        - Apkarian et al., "Self-scheduled H-inf control of linear
          parameter-varying systems"
        - Scherer, "Mixed H2/H-inf Control for LPV Systems"
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

        self.lf = vehicle_params.get('lf', 0.11)
        self.lr = vehicle_params.get('lr', 0.11)
        self.m = vehicle_params.get('m', 2.5)
        self.Iz = vehicle_params.get('Iz', 0.02)
        self.Cf = vehicle_params.get('Cf', 50.0)
        self.Cr = vehicle_params.get('Cr', 50.0)
        self.mu = vehicle_params.get('mu', 0.01)
        self.g = 9.81

        self.use_8d_system = vehicle_params.get('use_8d_system', False)

        self.dynamics = create_qlpv_dynamics(
            vehicle_params=vehicle_params,
            min_vx=vx_range[0],
            use_8d_system=self.use_8d_system
        )

        if self.use_8d_system:
            self.state_dim = STATE_DIM_8D
            self.meas_dim = MEAS_DIM_7D
        else:
            self.state_dim = STATE_DIM
            self.meas_dim = MEAS_DIM

        self.vertices = self._generate_vertices(n_vx_vertices, n_delta_vertices)
        self.n_vertices = len(self.vertices)

        self.vertex_gains: Dict[tuple, np.ndarray] = {}
        self.P_common: Optional[np.ndarray] = None
        self._gains_computed = False
        self._last_weights: Optional[np.ndarray] = None
        self._last_rho: Optional[tuple] = None
        self._default_gain = self._compute_robust_default_gain()

    def _compute_robust_default_gain(self) -> np.ndarray:
        """Compute a robust default gain using moderate settings"""
        L = np.zeros((self.state_dim, self.meas_dim))
        L[0, 0] = 5.0   # vx -> vx
        L[1, 1] = 3.0   # r -> vy
        L[2, 2] = 5.0   # psi -> psi
        L[3, 1] = 5.0   # r -> r
        L[4, 3] = 3.0   # X -> X
        L[5, 4] = 3.0   # Y -> Y
        return L

    def _generate_vertices(self, n_vx: int, n_delta: int) -> list:
        vx_values = np.linspace(self.vx_range[0], self.vx_range[1], n_vx)
        delta_values = np.linspace(-self.delta_max, self.delta_max, n_delta)
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
        x_dummy = np.zeros(self.state_dim)
        x_dummy[IDX_VX] = vertex.vx
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        return self.dynamics.compute_A_matrix(rho)

    def _compute_C_at_vertex(self, vertex: PolytopicVertex, gps_available: bool = True) -> np.ndarray:
        x_dummy = np.zeros(self.state_dim)
        x_dummy[IDX_VX] = vertex.vx
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        C = self.dynamics.compute_C_matrix(rho, gps_available=gps_available)
        if not gps_available:
            if self.use_8d_system:
                keep_indices = [MEAS8_IDX_VX, MEAS8_IDX_R, MEAS8_IDX_AY, MEAS8_IDX_AX]
            else:
                keep_indices = [MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_AY]
            C = C[keep_indices, :]
        return C

    def _compute_E_at_vertex(self, vertex: PolytopicVertex) -> np.ndarray:
        x_dummy = np.zeros(self.state_dim)
        x_dummy[IDX_VX] = vertex.vx
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        return self.dynamics.compute_E_matrix(rho)

    def _compute_F_at_vertex(self, vertex: PolytopicVertex, gps_available: bool = True) -> np.ndarray:
        x_dummy = np.zeros(self.state_dim)
        x_dummy[IDX_VX] = vertex.vx
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        F = self.dynamics.compute_F_matrix(rho)
        if not gps_available:
            if self.use_8d_system:
                keep_indices = [MEAS8_IDX_VX, MEAS8_IDX_R, MEAS8_IDX_AY, MEAS8_IDX_AX]
            else:
                keep_indices = [MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_AY]
            F = F[keep_indices, :]
        return F

    def _check_observability(self, A: np.ndarray, C: np.ndarray) -> Tuple[bool, int]:
        n = A.shape[0]
        O = C.copy()
        A_power = np.eye(n)
        for i in range(1, n):
            A_power = A_power @ A
            O = np.vstack([O, C @ A_power])
        rank = np.linalg.matrix_rank(O)
        return rank == n, rank

    def compute_gains_lmi(self) -> bool:
        if not CVXPY_AVAILABLE:
            print("Warning: CVXPY not available, using fallback gains")
            return self._compute_fallback_gains()
        n = self.state_dim
        m_full = self.meas_dim
        success_gps = False
        if self.use_common_lyapunov:
            success_gps = self._compute_gains_common_lyapunov(n, m_full, gps_available=True)
        else:
            success_gps = self._compute_gains_independent(n, m_full, gps_available=True)
        if not success_gps and self.verbose:
            print("Warning: LMI gains for GPS case failed validation")
        if self.use_8d_system:
            m_no_gps = 4
        else:
            m_no_gps = 3
        success_no_gps = False
        if self.use_common_lyapunov:
            success_no_gps = self._compute_gains_common_lyapunov(n, m_no_gps, gps_available=False)
        else:
            success_no_gps = self._compute_gains_independent(n, m_no_gps, gps_available=False)
        if not success_no_gps and self.verbose:
            print("Warning: LMI gains for No-GPS case failed validation")
        if not success_gps:
            self._compute_fallback_gains(gps_available=True)
        if not success_no_gps:
            self._compute_fallback_gains(gps_available=False)
        return True

    def _compute_fallback_gains(self, gps_available: bool = True) -> bool:
        for vertex in self.vertices:
            key = (vertex.to_tuple(), gps_available)
            self.vertex_gains[key] = self._compute_structured_fallback_gain(vertex, gps_available)
        self._gains_computed = True
        return True

    def _compute_structured_fallback_gain(self, vertex: PolytopicVertex, gps_available: bool = True) -> np.ndarray:
        n = self.state_dim
        if gps_available:
            m = self.meas_dim
        else:
            m = 4 if self.use_8d_system else 3
        L = np.zeros((n, m))
        L[0, 0] = 5.0
        return L

    def _compute_gains_common_lyapunov(self, n: int, m: int, gps_available: bool = True) -> bool:
        if self.use_hinf:
            success = self._solve_hinf_lmi(n, m, gps_available)
            if success:
                return True
            if self.verbose:
                print("H-inf LMI failed, trying standard stability LMI...")
        return self._solve_standard_lmi(n, m, gps_available)

    def _solve_standard_lmi(self, n: int, m: int, gps_available: bool = True) -> bool:
        P = cp.Variable((n, n), symmetric=True)
        Y_list = [cp.Variable((n, m)) for _ in range(self.n_vertices)]
        P_min = 1e-3
        P_max = 1e3
        constraints = [
            P >> P_min * np.eye(n),
            P << P_max * np.eye(n),
        ]
        for i, vertex in enumerate(self.vertices):
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex, gps_available=gps_available)
            Y = Y_list[i]
            lmi = A.T @ P + P @ A - C.T @ Y.T - Y @ C + self.decay_rate * P
            constraints.append(lmi << -1e-4 * np.eye(n))
        gamma_reg = 0.01
        reg_term = sum(cp.norm(Y, 'fro') for Y in Y_list)
        objective = cp.Minimize(cp.trace(P) + gamma_reg * reg_term)
        return self._solve_and_extract(P, Y_list, objective, constraints, n, gps_available)

    def _solve_hinf_lmi(self, n: int, m: int, gps_available: bool) -> bool:
        P = cp.Variable((n, n), symmetric=True)
        Y_list = [cp.Variable((n, m)) for _ in range(self.n_vertices)]
        P_min = 1e-3
        P_max = 1e3
        constraints = [
            P >> P_min * np.eye(n),
            P << P_max * np.eye(n),
        ]
        p = 2  # Disturbance dimension
        for i, vertex in enumerate(self.vertices):
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex, gps_available=gps_available)
            E = self._compute_E_at_vertex(vertex)
            F = self._compute_F_at_vertex(vertex, gps_available=gps_available)
            Y = Y_list[i]
            block_11 = A.T @ P + P @ A - C.T @ Y.T - Y @ C + self.decay_rate * P
            block_12 = P @ E - Y @ F
            block_22 = -self.hinf_gamma**2 * np.eye(p)
            hinf_lmi = cp.bmat([
                [block_11, block_12],
                [block_12.T, block_22]
            ])
            constraints.append(hinf_lmi << -1e-4 * np.eye(n + p))
        gamma_reg = 0.01
        reg_term = sum(cp.norm(Y, 'fro') for Y in Y_list)
        objective = cp.Minimize(cp.trace(P) + gamma_reg * reg_term)
        return self._solve_and_extract(P, Y_list, objective, constraints, n, gps_available)

    def _solve_and_extract(self, P, Y_list, objective, constraints, n: int, gps_available: bool) -> bool:
        problem = cp.Problem(objective, constraints)
        solvers_to_try = [
            (cp.SCS, {'max_iters': 20000, 'eps': 1e-7}),
            (cp.SCS, {'max_iters': 50000, 'eps': 1e-8}),
        ]
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
                    if P.value is not None:
                        min_eig = np.min(np.linalg.eigvalsh(P.value))
                        if min_eig > 1e-4:
                            solved = True
                            break
            except Exception as e:
                if self.verbose:
                    print(f"Solver {solver} failed: {e}")
                continue
        if not solved:
            if self.verbose:
                print("All solvers failed to produce valid solution")
            return False
        self.P_common = P.value
        if self.P_common is None:
            return False
        min_eig = np.min(np.linalg.eigvalsh(self.P_common))
        if min_eig < 1e-6:
            if self.verbose:
                print(f"P has invalid eigenvalue: {min_eig:.4e}")
            return False
        for i, vertex in enumerate(self.vertices):
            Y_val = Y_list[i].value
            if Y_val is None:
                return False
            try:
                L = np.linalg.solve(self.P_common, Y_val)
                L = np.clip(L, -100.0, 100.0)
                self.vertex_gains[(vertex.to_tuple(), gps_available)] = L
            except np.linalg.LinAlgError:
                L = np.linalg.pinv(self.P_common) @ Y_val
                L = np.clip(L, -100.0, 100.0)
                self.vertex_gains[(vertex.to_tuple(), gps_available)] = L
        self._gains_computed = True
        return True

    def _compute_gains_independent(self, n: int, m: int, gps_available: bool) -> bool:
        success = True
        for vertex in self.vertices:
            A = self._compute_A_at_vertex(vertex)
            C = self._compute_C_at_vertex(vertex, gps_available=gps_available)
            key = (vertex.to_tuple(), gps_available)
            try:
                L = compute_lmi_observer_gain(A, C, decay_rate=self.decay_rate,
                                              verbose=self.verbose)
                self.vertex_gains[key] = L
            except Exception as e:
                if self.verbose:
                    print(f"Warning: Failed to compute gain at vertex {vertex}: {e}")
                self.vertex_gains[key] = self._compute_structured_fallback_gain(vertex, gps_available)
                success = False
        self._gains_computed = True
        return success

    def compute_interpolation_weights(self, vx: float, delta: float) -> np.ndarray:
        vx = np.clip(vx, self.vx_range[0], self.vx_range[1])
        delta = np.clip(delta, -self.delta_max, self.delta_max)
        vx_range = self.vx_range[1] - self.vx_range[0]
        delta_range = 2 * self.delta_max
        vx_norm = (vx - self.vx_range[0]) / vx_range if vx_range > 0 else 0.5
        delta_norm = (delta + self.delta_max) / delta_range if delta_range > 0 else 0.5
        current_point = np.array([vx_norm, delta_norm])
        sigma = 0.3
        weights = np.zeros(self.n_vertices)
        for i, vertex in enumerate(self.vertices):
            vx_v_norm = (vertex.vx - self.vx_range[0]) / vx_range if vx_range > 0 else 0.5
            delta_v_norm = (vertex.delta + self.delta_max) / delta_range if delta_range > 0 else 0.5
            vertex_point = np.array([vx_v_norm, delta_v_norm])
            dist_sq = np.sum((current_point - vertex_point) ** 2)
            weights[i] = np.exp(-dist_sq / (2 * sigma ** 2))
        weight_sum = np.sum(weights)
        if weight_sum > 0:
            weights = weights / weight_sum
        else:
            weights = np.ones(self.n_vertices) / self.n_vertices
        return weights

    def get_scheduled_gain(self, vx: float, delta: float, gps_available: bool = True) -> np.ndarray:
        if not self._gains_computed:
            return self._default_gain.copy()
        current_rho = (round(vx, 3), round(delta, 3))
        if self._last_rho == current_rho and self._last_weights is not None:
            weights = self._last_weights
        else:
            weights = self.compute_interpolation_weights(vx, delta)
            self._last_weights = weights
            self._last_rho = current_rho
        L_accum = None
        for i, vertex in enumerate(self.vertices):
            gain = self.vertex_gains.get((vertex.to_tuple(), gps_available))
            if gain is None:
                gain = self._default_gain
            if L_accum is None:
                L_accum = weights[i] * gain
            else:
                L_accum += weights[i] * gain
        if L_accum is None:
            return self._default_gain.copy()
        return L_accum

    def get_vertex_info(self) -> str:
        lines = [f"qLPV Gain Scheduler: {self.n_vertices} vertices"]
        lines.append(f"  vx range: [{self.vx_range[0]:.2f}, {self.vx_range[1]:.2f}] m/s")
        lines.append(f"  delta range: [{-self.delta_max:.2f}, {self.delta_max:.2f}] rad")
        lines.append(f"  Common Lyapunov: {self.use_common_lyapunov}")
        lines.append(f"  Decay rate: {self.decay_rate}")
        lines.append(f"  Gains computed: {self._gains_computed}")
        return "\n".join(lines)

    def get_all_gains_summary(self) -> str:
        lines = ["\nComputed Observer Gains at Polytope Vertices:"]
        lines.append("=" * 60)
        if not self._gains_computed:
            lines.append("  Gains not yet computed. Call compute_gains_lmi() first.")
            return "\n".join(lines)
        for i, vertex in enumerate(self.vertices):
            L = self.vertex_gains.get((vertex.to_tuple(), True))
            if L is not None:
                lines.append(f"\nVertex {i+1}: vx={vertex.vx:.2f} m/s, delta={vertex.delta:.2f} rad")
                lines.append(f"  Gain L ({L.shape[0]}x{L.shape[1]}):")
                lines.append(f"    Eigenvalues of (A-LC): {self._get_closed_loop_eigs(vertex, L)}")
                diag = np.diag(L) if L.shape[0] == L.shape[1] else L.diagonal()
                lines.append(f"    Gain matrix diagonal: {diag}")
                lines.append(f"    Frobenius norm: {np.linalg.norm(L, 'fro'):.4f}")
        if self.P_common is not None:
            lines.append(f"\nCommon Lyapunov matrix P:")
            lines.append(f"  Condition number: {np.linalg.cond(self.P_common):.2e}")
            lines.append(f"  Min eigenvalue: {np.min(np.linalg.eigvalsh(self.P_common)):.4f}")
            lines.append(f"  Max eigenvalue: {np.max(np.linalg.eigvalsh(self.P_common)):.4f}")
        return "\n".join(lines)

    def _get_closed_loop_eigs(self, vertex: PolytopicVertex, L: np.ndarray) -> str:
        A = self._compute_A_at_vertex(vertex)
        C = self._compute_C_at_vertex(vertex)
        try:
            A_cl = A - L @ C
            eigs = np.linalg.eigvals(A_cl)
            real_parts = np.real(eigs)
            return f"[{', '.join([f'{r:.2f}' for r in sorted(real_parts)])}]"
        except:
            return "[computation failed]"


# =============================================================================
# qLPV Augmented-State Observer (Kalman-structured with LMI gains)
# =============================================================================

class qLPVAugmentedObserver(FirstLayerObserverBase):
    """
    qLPV Augmented-State Observer with LMI Gain Scheduling

    Structured like the EKF-style Kalman observer (predict-correct with ZOH
    discretization, control processing, still-condition handling, active
    measurement indices, velocity blending, tire residual equality constraint,
    observability gating) but uses LMI-scheduled gains instead of Kalman gains.

    State vector: x = [v_x, v_y, psi, r, X, Y]^T
    Tire residuals: w = [w_r, w_f]^T
    Augmented state: x_a = [x; w]^T (8-dimensional)
    Measurements: y = [v_x, r, psi, X, Y, a_y, a_x]^T (7D)
    """

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
                 use_8d_system: bool = False,
                 dynamics_model = None,
                 **kwargs):
        """
        Initialize qLPV Augmented-State Observer with LMI Gains

        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict
            observer_gains: Observer gain parameters
            include_gyro_bias: Whether to include gyro bias state
            use_gain_scheduling: Whether to use polytopic qLPV gain scheduling
            lmi_decay_rate: Decay rate for LMI observer gain design
            vx_range: Velocity range for gain scheduling
            delta_max: Maximum steering angle
            n_vx_vertices: Number of velocity grid points
            n_delta_vertices: Number of steering grid points
            verbose: Print solver/debug output
            use_8d_system: Use 8D state vector (including ax, ay)
            dynamics_model: Existing QLPVVehicleDynamicsObs instance (optional)
        """
        self.use_8d_system = use_8d_system
        self.disturbance_mode = kwargs.get('disturbance_mode', 'tire')
        self.udim = 3 if self.disturbance_mode == 'general' else 2

        if use_8d_system:
            self.state_dim = STATE_DIM_8D
            self.augmented_dim = STATE_DIM_8D + self.udim
            self.meas_dim = MEAS_DIM_7D
        else:
            self.state_dim = STATE_DIM
            self.augmented_dim = STATE_DIM + self.udim
            self.meas_dim = MEAS_DIM

        # Initialize base class
        super().__init__(
            state_dim=self.state_dim,
            unknown_input_dim=self.udim,
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
        self.mu = self.params.get('mu', 0.01)
        self.g = 9.81

        # Augmented state: [x; w_r; w_f]
        self.state_augmented = np.zeros(self.augmented_dim)

        # Initialize state estimate
        self.state_hat = np.zeros(self.state_dim)

        # Tire residual estimates
        self.w_hat = np.zeros(self.udim)

        # =====================================================
        # Tire Residual Cross-Correlation (Observability Fix)
        # =====================================================
        self.tire_correlation = kwargs.get('tire_correlation', 0.8)

        self.tire_info_layer_1 = {
            'Fyr_est': 0.0,
            'Fyf_est': 0.0,
            'Fyr_linear_only': 0.0,
            'Fyf_linear_only': 0.0,
            'alpha_r': 0.0,
            'alpha_f': 0.0
        }

        # =====================================================
        # Observer gains and scheduling (LMI-based)
        # =====================================================
        self.observer_gains = observer_gains or self._default_gains()
        self.use_gain_scheduling = use_gain_scheduling
        self.lmi_decay_rate = lmi_decay_rate
        self.verbose = verbose

        # Minimum velocity threshold
        self.min_vx = self.params.get('vx_min', 0.1)

        # Centralized vehicle dynamics (single source of truth)
        # Must be initialized BEFORE gain scheduler
        if dynamics_model is not None:
            self.dynamics = dynamics_model
        else:
            self.dynamics = create_qlpv_dynamics(
                vehicle_params=self.params,
                min_vx=self.min_vx,
                use_8d_system=use_8d_system,
                disturbance_mode=self.disturbance_mode
            )

        # Initialize gain scheduler if enabled
        # Prefer NeuralQLPVGainScheduler (discrete-time, H∞/L2/H2/contraction)
        # over legacy QLPVGainScheduler (continuous-time only)
        self.gain_scheduler = None
        self.lmi_method = kwargs.get('lmi_method', 'hinf')
        self.hinf_gamma = kwargs.get('hinf_gamma', 5.0)
        self.use_discrete_lmi = kwargs.get('use_discrete_lmi', True)
        self.contraction_rate = kwargs.get('contraction_rate', 0.95)
        if use_gain_scheduling:
            if vehicle_params is None:
                vehicle_params = {}
            vehicle_params_sched = vehicle_params.copy()
            vehicle_params_sched['use_8d_system'] = use_8d_system
            if NEURAL_SCHEDULER_AVAILABLE and self.use_discrete_lmi:
                # Use NeuralQLPVGainScheduler (discrete-time LMI, superior design)
                self.gain_scheduler = NeuralQLPVGainScheduler(
                    vehicle_params=vehicle_params_sched,
                    vx_range=vx_range,
                    delta_max=delta_max,
                    n_vx_vertices=n_vx_vertices,
                    n_delta_vertices=n_delta_vertices,
                    decay_rate=lmi_decay_rate,
                    lmi_method=self.lmi_method,
                    hinf_gamma=self.hinf_gamma,
                    use_common_lyapunov=True,
                    discrete=True,
                    sample_time=sample_time,
                    contraction_rate=self.contraction_rate,
                    verbose=verbose,
                    disturbance_mode=self.disturbance_mode,
                    dynamics_model=self.dynamics,
                )
                self.gain_scheduler.compute_gains_lmi()
                if verbose:
                    print(f"Using NeuralQLPVGainScheduler (discrete, method={self.lmi_method})")
            else:
                # Fallback to legacy QLPVGainScheduler (continuous-time)
                self.gain_scheduler = QLPVGainScheduler(
                    vehicle_params=vehicle_params_sched,
                    vx_range=vx_range,
                    delta_max=delta_max,
                    n_vx_vertices=n_vx_vertices,
                    n_delta_vertices=n_delta_vertices,
                    decay_rate=lmi_decay_rate,
                    use_common_lyapunov=True,
                    verbose=verbose
                )
                self.gain_scheduler.compute_gains_lmi()
                if verbose:
                    print("Using legacy QLPVGainScheduler (continuous-time)")

        self._initialize_observer_gains()

        # Store last innovation for diagnostics
        self.innovation = np.zeros(self.meas_dim)

        # Gyro bias estimation (optional)
        self.include_gyro_bias = include_gyro_bias
        self.gyro_bias = 0.0

        # UIO residual for a_y constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0

        # Internal state for control processing
        self.current_steering_angle = 0.0

        # =====================================================
        # Optional r_dot differentiator (pseudo-measurement)
        # =====================================================
        self.use_rdot_differentiator = kwargs.get('use_rdot_differentiator', False)
        self.rdot_diff_type = kwargs.get('rdot_diff_type', 'highgain')
        self.rdot_diff_config_path = kwargs.get('rdot_diff_config_path', None)
        self.rdot_diff_overrides = kwargs.get('rdot_diff_overrides', {})
        rdot_override = dict(self.rdot_diff_overrides) if isinstance(self.rdot_diff_overrides, dict) else {}
        for key in ('omega', 'zeta', 'ydot_max', 'tau', 'k1', 'k2', 'epsilon', 'smoothing', 'v_max'):
            kw_key = f'rdot_diff_{key}'
            if kw_key in kwargs:
                rdot_override[key] = kwargs[kw_key]
        self.rdot_diff_overrides = rdot_override
        self.rdot_meas_var = kwargs.get('rdot_meas_var', 0.25)
        self.rdot_diff = None
        self.rdot_meas = 0.0
        if self.use_rdot_differentiator and DIFFERENTIATOR_AVAILABLE:
            self.rdot_diff = create_differentiator_from_config(
                diff_type=self.rdot_diff_type,
                Ts=self.Ts,
                config_path=self.rdot_diff_config_path,
                **self.rdot_diff_overrides
            )

        # =====================================================
        # Observability-based gating
        # =====================================================
        self.enable_observability_gating = kwargs.get('enable_observability_gating', False)
        self.vx_min_observable = kwargs.get('vx_min_observable', 0.3)
        self.delta_min_observable = kwargs.get('delta_min_observable', 0.02)
        self.r_min_observable = kwargs.get('r_min_observable', 0.05)
        self.yaw_wrap_window = kwargs.get('yaw_wrap_window', 5)
        self.yaw_wrap_threshold = kwargs.get('yaw_wrap_threshold', 2.5)
        self.yaw_wrap_counter = 0
        self.prev_psi_meas = 0.0
        self.observability_flags = np.ones(self.udim)
        self.gated_decay_rate_dvx = kwargs.get('gated_decay_rate_dvx', 2.0)
        self.gated_decay_rate_dvy = kwargs.get('gated_decay_rate_dvy', 3.0)
        self.gated_decay_rate_dr = kwargs.get('gated_decay_rate_dr', 5.0)

    # =========================================================
    # Helper methods
    # =========================================================

    def _apply_decay(self, value: float, rate: float, dt: float) -> float:
        """Exponential decay helper for disturbance states."""
        if rate is None or rate <= 0.0:
            return value
        return value * np.exp(-rate * dt)

    def _compute_observability_flags(self, vx: float, delta: float, r: float,
                                      psi_meas: float) -> np.ndarray:
        """Compute per-component observability flags based on driving conditions."""
        flags = np.ones(self.udim)
        psi_jump = abs(psi_meas - self.prev_psi_meas)
        if psi_jump > self.yaw_wrap_threshold:
            self.yaw_wrap_counter = self.yaw_wrap_window
        yaw_wrapping = self.yaw_wrap_counter > 0
        low_speed = abs(vx) < self.vx_min_observable
        straight_driving = (abs(delta) < self.delta_min_observable and
                           abs(r) < self.r_min_observable)
        if self.disturbance_mode == 'general':
            flags[0] = 1.0
            if low_speed:
                flags[1] = 0.0
            if low_speed or straight_driving or yaw_wrapping:
                flags[2] = 0.0
        else:
            if low_speed:
                flags[0] = 0.0
                flags[1] = 0.0
            if straight_driving:
                flags[1] = 0.5
        return flags

    def _apply_observability_gating(self, w_new: np.ndarray, w_prev: np.ndarray,
                                     flags: np.ndarray, dt: float) -> np.ndarray:
        """Apply observability gating to disturbance estimates."""
        w_gated = w_new.copy()
        if self.disturbance_mode == 'general' and self.udim >= 3:
            decay_rates = [self.gated_decay_rate_dvx,
                          self.gated_decay_rate_dvy,
                          self.gated_decay_rate_dr]
        else:
            decay_rates = [self.gated_decay_rate_dvy, self.gated_decay_rate_dvy]
        for i in range(min(len(flags), len(w_gated))):
            if flags[i] < 1.0:
                if flags[i] == 0.0:
                    w_gated[i] = self._apply_decay(w_prev[i], decay_rates[i], dt)
                else:
                    w_decayed = self._apply_decay(w_prev[i], decay_rates[i], dt)
                    w_gated[i] = flags[i] * w_new[i] + (1.0 - flags[i]) * w_decayed
        return w_gated

    def _discretize_augmented(self, A_a: np.ndarray, B_a: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Discretize continuous augmented system matrices using ZOH (Van Loan method).

        Args:
            A_a: Continuous augmented state matrix
            B_a: Continuous augmented input matrix
            dt: Sample time

        Returns:
            Tuple of (A_ad, B_ad) - discrete state and input matrices
        """
        n_states = A_a.shape[0]
        n_inputs = B_a.shape[1]
        if SCIPY_AVAILABLE:
            M = np.zeros((n_states + n_inputs, n_states + n_inputs))
            M[:n_states, :n_states] = A_a
            M[:n_states, n_states:] = B_a
            M_exp = expm(M * dt)
            A_ad = M_exp[:n_states, :n_states]
            B_ad = M_exp[:n_states, n_states:]
        else:
            A_ad = np.eye(n_states) + A_a * dt
            B_ad = B_a * dt
        return A_ad, B_ad

    def _default_params(self) -> Dict:
        """Default vehicle parameters - uses centralized defaults"""
        return get_default_vehicle_params()

    def _default_gains(self) -> Dict:
        """Default observer gains"""
        # Build default L_state with correct dimensions (state_dim × meas_dim)
        L_def = np.zeros((self.state_dim, self.meas_dim))
        if self.use_8d_system:
            L_def[IDX8_VX, MEAS8_IDX_VX] = 2.0
            L_def[IDX8_VY, MEAS8_IDX_R] = 2.0
            L_def[IDX8_PSI, MEAS8_IDX_PSI] = 1.0
            L_def[IDX8_R, MEAS8_IDX_R] = 2.0
            L_def[IDX8_X, MEAS8_IDX_X] = 0.5
            L_def[IDX8_Y, MEAS8_IDX_Y] = 0.5
            L_def[IDX8_AX, MEAS8_IDX_AX] = 5.0
            L_def[IDX8_AY, MEAS8_IDX_AY] = 5.0
        else:
            L_def[IDX_VX, MEAS_IDX_VX] = 2.0
            L_def[IDX_VY, MEAS_IDX_R] = 2.0
            L_def[IDX_PSI, MEAS_IDX_PSI] = 1.0
            L_def[IDX_R, MEAS_IDX_R] = 2.0
            L_def[IDX_X, MEAS_IDX_X] = 0.5
            L_def[IDX_Y, MEAS_IDX_Y] = 0.5
        return {
            'L_state': L_def,
            'L_residual': np.zeros((self.udim, self.meas_dim)),
            'alpha_w': 0.1,
        }

    def _initialize_observer_gains(self):
        """Initialize observer gain matrices"""
        gains = self.observer_gains
        if isinstance(gains.get('L_state'), np.ndarray) and gains['L_state'].shape == (self.state_dim, self.meas_dim):
            self.L_state = gains['L_state']
        else:
            # Always create (state_dim × meas_dim) gain matrix
            self.L_state = np.zeros((self.state_dim, self.meas_dim))
            if self.use_8d_system:
                self.L_state[IDX8_VX, MEAS8_IDX_VX] = 2.0
                self.L_state[IDX8_VY, MEAS8_IDX_R] = 2.0
                self.L_state[IDX8_PSI, MEAS8_IDX_PSI] = 1.0
                self.L_state[IDX8_R, MEAS8_IDX_R] = 2.0
                self.L_state[IDX8_X, MEAS8_IDX_X] = 0.5
                self.L_state[IDX8_Y, MEAS8_IDX_Y] = 0.5
                self.L_state[IDX8_AX, MEAS8_IDX_AX] = 5.0
                self.L_state[IDX8_AY, MEAS8_IDX_AY] = 5.0
            else:
                self.L_state[IDX_VX, MEAS_IDX_VX] = 2.0
                self.L_state[IDX_VY, MEAS_IDX_R] = 2.0
                self.L_state[IDX_PSI, MEAS_IDX_PSI] = 1.0
                self.L_state[IDX_R, MEAS_IDX_R] = 2.0
                self.L_state[IDX_X, MEAS_IDX_X] = 0.5
                self.L_state[IDX_Y, MEAS_IDX_Y] = 0.5
        if isinstance(gains.get('L_residual'), np.ndarray) and gains['L_residual'].shape == (self.udim, self.meas_dim):
            self.L_residual = gains['L_residual']
        else:
            self.L_residual = np.zeros((self.udim, self.meas_dim))
            idx_ay = MEAS8_IDX_AY if self.use_8d_system else MEAS_IDX_AY
            self.L_residual[0, idx_ay] = 2.0
            if self.udim >= 2:
                self.L_residual[1, idx_ay] = 2.0
        self.L_augmented = np.vstack([self.L_state, self.L_residual])

    def _get_scheduled_gain(self, vx: float, delta: float, gps_available: bool = True) -> np.ndarray:
        """Get observer gain for current operating point (LMI-scheduled or static)."""
        if self.use_gain_scheduling and self.gain_scheduler is not None:
            if isinstance(self.gain_scheduler, NeuralQLPVGainScheduler) if NEURAL_SCHEDULER_AVAILABLE else False:
                # NeuralQLPVGainScheduler.get_scheduled_gain(vx, delta) - no gps_available
                return self.gain_scheduler.get_scheduled_gain(vx, delta)
            else:
                # Legacy QLPVGainScheduler.get_scheduled_gain(vx, delta, gps_available)
                return self.gain_scheduler.get_scheduled_gain(vx, delta, gps_available=gps_available)
        else:
            return self.L_state

    # =========================================================
    # Dynamics delegation methods
    # =========================================================

    def compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        return self.dynamics.compute_scheduling_params(state, delta)

    def compute_slip_angles(self, state: np.ndarray, delta: float) -> Tuple[float, float]:
        return self.dynamics.compute_slip_angles(state, delta)

    def compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        return self.dynamics.compute_A_matrix(rho)

    def compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        return self.dynamics.compute_B_matrix(rho)

    def compute_E_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        return self.dynamics.compute_E_matrix(rho)

    def compute_C_matrix(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> np.ndarray:
        return self.dynamics.compute_C_matrix(rho, active_indices=active_indices)

    def compute_D_matrix(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> np.ndarray:
        return self.dynamics.compute_D_matrix(rho, active_indices=active_indices)

    def compute_F_matrix(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> np.ndarray:
        return self.dynamics.compute_F_matrix(rho, active_indices=active_indices)

    def compute_augmented_matrices(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.dynamics.compute_augmented_matrices(rho, active_indices=active_indices)



    # =========================================================
    # Main Update (Kalman-structured with LMI gain)
    # =========================================================

    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None,
               gps_available: bool = True,
               dt: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update qLPV augmented-state observer with predict-correct cycle.

        Uses ZOH-discretized qLPV dynamics for prediction and LMI-scheduled
        gains for correction, with Kalman-style structure including:
        - Control processing (steering_cmd, throttle_cmd -> delta, accel)
        - Still-condition handling with smooth velocity blending
        - Active measurement indices for flexible GPS/IMU handling
        - Tire residual equality constraint (pseudo-measurement)
        - Observability-based gating

        Args:
            measurement: Measurement vector
            control_input: Control [delta, a] OR [steering_cmd, throttle_cmd]
            f_nn: Neural network output (not used in first layer)
            acceleration: Full 3D acceleration [a_x, a_y, a_z]
            gps_available: Whether GPS measurements are valid
            dt: Sample time (if None, use self.Ts)

        Returns:
            Tuple of (state_estimate, tire_residual_estimate)
        """
        measurement = measurement.reshape(-1)
        y_full = self._process_measurement(measurement, acceleration)
        current_dt = dt if dt is not None else self.Ts

        # =====================================================
        # Determine active measurement indices
        # =====================================================
        idx_X = MEAS8_IDX_X
        idx_Y = MEAS8_IDX_Y
        idx_PSI = MEAS8_IDX_PSI
        idx_R = MEAS8_IDX_R
        idx_VX = MEAS8_IDX_VX
        idx_AY = MEAS8_IDX_AY
        idx_AX = MEAS8_IDX_AX

        n_meas_in = len(measurement)
        gps_meas_available = False
        imu_ay_available = False
        imu_ax_available = False

        if n_meas_in >= 7:
            gps_meas_available = True
            imu_ay_available = True
            imu_ax_available = True
        elif n_meas_in == 6:
            gps_meas_available = True
            imu_ay_available = True
        elif n_meas_in == 5:
            gps_meas_available = True
        elif n_meas_in == 4:
            imu_ay_available = True
            imu_ax_available = True
        elif n_meas_in == 2:
            pass

        if not gps_available:
            gps_meas_available = False

        if acceleration is not None:
            if len(acceleration) > 0:
                imu_ax_available = True
            if len(acceleration) > 1:
                imu_ay_available = True

        # Build active measurement indices in canonical order
        active_indices = [idx_VX, idx_R]
        if gps_meas_available:
            active_indices.extend([idx_PSI, idx_X, idx_Y])
        if imu_ay_available:
            active_indices.append(idx_AY)
        if imu_ax_available:
            active_indices.append(idx_AX)

        # Slice full measurement vector to active channels
        y = y_full[active_indices]

        # Cached measurements for gating
        r_meas = y_full[idx_R]
        ay_meas = y_full[idx_AY]
        psi_meas = y_full[idx_PSI]
        w_prev = self.state_augmented[self.state_dim:].copy()

        # =====================================================
        # Process Control Input
        # =====================================================
        u_raw = control_input.reshape(-1)
        current_state = self.state_hat.copy()

        throttle_cmd = u_raw[1] if len(u_raw) > 1 else 0.0
        steering_cmd = u_raw[0]

        # Use centralized dynamics to process control
        accel, _, new_steering_angle = self.dynamics.process_control_inputs(
            throttle_cmd, steering_cmd, current_state,
            self.current_steering_angle, current_dt
        )

        self.current_steering_angle = new_steering_angle

        # Form physical control vector [delta, a]
        u = np.array([self.current_steering_angle, accel])

        # Override integrated steering with measurement if available
        if len(u_raw) > 2:
            self.current_steering_angle = u_raw[2]
            u[0] = self.current_steering_angle

        delta = u[0]

        # =====================================================
        # STILL CONDITION CHECK
        # =====================================================
        measured_vx = 0.0
        if idx_VX in active_indices:
            idx_in_y = active_indices.index(idx_VX)
            measured_vx = y[idx_in_y]

        throttle_near_zero = abs(throttle_cmd) < 0.05
        steering_near_zero = abs(steering_cmd) < 0.05
        velocity_near_zero = abs(measured_vx) < 0.1

        # Smooth blending factor [V_STILL, V_NOMINAL]
        V_STILL = 0.1
        V_NOMINAL = 0.4
        abs_vx = abs(measured_vx)
        if abs_vx <= V_STILL:
            velocity_blend = 0.0
        elif abs_vx >= V_NOMINAL:
            velocity_blend = 1.0
        else:
            t = (abs_vx - V_STILL) / (V_NOMINAL - V_STILL)
            velocity_blend = 3.0 * t**2 - 2.0 * t**3

        is_vehicle_still = velocity_near_zero and throttle_near_zero

        if is_vehicle_still:
            # =====================================================
            # KINEMATIC FALLBACK (for start/stop stability)
            # =====================================================
            xa_upd = self.state_augmented.copy()

            xa_upd[IDX_VX] = measured_vx
            xa_upd[IDX_VY] = 0.0
            xa_upd[IDX_R] = y_full[idx_R]

            if gps_meas_available:
                xa_upd[IDX_X] = y_full[idx_X]
                xa_upd[IDX_Y] = y_full[idx_Y]
                xa_upd[IDX_PSI] = y_full[idx_PSI]

            # Zero out disturbances when stationary
            xa_upd[self.state_dim:] = 0.0

            innov = np.zeros(len(y))
            y_pred = y.copy()

            if self.rdot_diff is not None:
                self.rdot_meas = 0.0

        else:
            # =====================================================
            # FULL qLPV OBSERVER (vehicle is moving)
            # =====================================================
            self.tire_info_layer_1 = self.dynamics._calculate_tire_info(
                self.state_augmented[IDX_VX], self.state_augmented[IDX_VY],
                self.state_augmented[IDX_R], delta,
                self.state_augmented[self.state_dim],
                self.state_augmented[self.state_dim + 1]
            )

            # =====================================================
            # PREDICT: ZOH-discretized qLPV dynamics
            # =====================================================
            rho = self.compute_scheduling_params(self.state_augmented[:self.state_dim], u[0])

            # Compute continuous augmented matrices
            A_a, B_a, _ = self.compute_augmented_matrices(rho)

            # Discretize using ZOH
            A_ad, B_ad = self._discretize_augmented(A_a, B_a, current_dt)

            # State prediction: x[k+1] = A_d * x[k] + B_d * u[k]
            xa_pred = A_ad @ self.state_augmented + B_ad @ u

            # =====================================================
            # CORRECT: LMI-scheduled gain correction
            # =====================================================
            rho_pred = self.compute_scheduling_params(xa_pred[:self.state_dim], u[0])
            _, _, C_a = self.compute_augmented_matrices(rho_pred, active_indices=active_indices)
            D = self.compute_D_matrix(rho_pred, active_indices=active_indices)

            # Measurement prediction: y = C_a * x_a + D * u
            y_pred = C_a @ xa_pred + D @ u

            # Innovation
            innov = y - y_pred

            # Wrap heading innovation to [-pi, pi]
            if self.use_8d_system:
                idx_psi_meas = MEAS8_IDX_PSI
            else:
                idx_psi_meas = MEAS_IDX_PSI

            psi_in_meas = False
            idx_in_innov = -1
            if active_indices is not None:
                if idx_psi_meas in active_indices:
                    idx_in_innov = active_indices.index(idx_psi_meas)
                    psi_in_meas = True
            else:
                idx_in_innov = idx_psi_meas
                psi_in_meas = True

            if psi_in_meas and idx_in_innov < len(innov):
                innov[idx_in_innov] = (innov[idx_in_innov] + np.pi) % (2 * np.pi) - np.pi

            # Get LMI-scheduled observer gain for state part
            L_state_sched = self._get_scheduled_gain(rho_pred.vx, delta, gps_available=gps_meas_available)

            # Build augmented gain: stack L_state and L_residual
            if not gps_meas_available:
                if self.use_8d_system:
                    keep_indices_res = [MEAS8_IDX_VX, MEAS8_IDX_R, MEAS8_IDX_AY, MEAS8_IDX_AX]
                else:
                    keep_indices_res = [MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_AY]
                L_res_active = self.L_residual[:, keep_indices_res]
            else:
                L_res_active = self.L_residual

            # Check dimension compatibility and adjust if needed
            n_active = len(active_indices)
            if L_state_sched.shape[1] != n_active:
                L_state_active = np.zeros((self.state_dim, n_active))
                L_state_active[0, 0] = 5.0  # vx -> vx
                if n_active > 1:
                    L_state_active[3, 1] = 5.0  # r -> r
            else:
                L_state_active = L_state_sched

            if L_res_active.shape[1] != n_active:
                L_res_active = np.zeros((self.udim, n_active))

            L_augmented = np.vstack([L_state_active, L_res_active])

            # LMI correction step: x_hat_a = x_hat_a^- + L * (y - y_hat)
            xa_upd = xa_pred + L_augmented @ innov

            # Blend disturbance estimates with zero during transition
            if velocity_blend < 1.0:
                xa_upd[self.state_dim:] *= velocity_blend

            # =====================================================
            # TIRE RESIDUAL EQUALITY CONSTRAINT (Observability Fix)
            # =====================================================
            if self.disturbance_mode == 'tire' and self.tire_correlation > 0:
                H_eq = np.zeros((1, self.augmented_dim))
                H_eq[0, self.state_dim] = 1.0      # w_r
                H_eq[0, self.state_dim + 1] = -1.0  # -w_f

                y_eq_pred = H_eq @ xa_upd
                y_eq_meas = 0.0

                eq_error = y_eq_meas - y_eq_pred[0]
                eq_gain = self.tire_correlation * 0.5
                xa_upd[self.state_dim] += eq_gain * eq_error
                xa_upd[self.state_dim + 1] -= eq_gain * eq_error

            # Optional r_dot pseudo-measurement update
            if self.use_rdot_differentiator and self.rdot_diff is not None and self.disturbance_mode == 'general':
                self.rdot_meas = self.rdot_diff.update(r_meas)
            else:
                self.rdot_meas = 0.0

        # =====================================================
        # Post-update processing
        # =====================================================
        self.state_augmented = xa_upd

        # Wrap heading state to [-pi, pi]
        self.state_augmented[IDX_PSI] = (self.state_augmented[IDX_PSI] + np.pi) % (2 * np.pi) - np.pi

        # =====================================================
        # ADDITIONAL STILL CONDITION (post-update safety)
        # =====================================================
        if abs(measured_vx) < V_STILL and throttle_near_zero:
            self.state_augmented[IDX_VX] = 0.0
            self.state_augmented[IDX_VY] = 0.0
            self.state_augmented[IDX_R] = 0.0
            self.state_augmented[self.state_dim:] = 0.0

        # =====================================================
        # OBSERVABILITY-BASED GATING
        # =====================================================
        if self.enable_observability_gating and self.udim >= 2:
            d_start = self.state_dim
            w_new = self.state_augmented[d_start:d_start + self.udim].copy()
            vx_est = self.state_augmented[IDX_VX]
            r_est = self.state_augmented[IDX_R]
            self.observability_flags = self._compute_observability_flags(
                vx=vx_est, delta=delta, r=r_est, psi_meas=psi_meas
            )
            w_gated = self._apply_observability_gating(w_new, w_prev,
                                                        self.observability_flags, current_dt)
            self.state_augmented[d_start:d_start + self.udim] = w_gated
            if self.yaw_wrap_counter > 0:
                self.yaw_wrap_counter -= 1
            self.prev_psi_meas = psi_meas

        # Clamp states
        self.state_augmented[IDX_VX] = np.clip(self.state_augmented[IDX_VX], -10.0, 10.0)
        self.state_augmented[IDX_VY] = np.clip(self.state_augmented[IDX_VY], -5.0, 5.0)
        self.state_augmented[IDX_R] = np.clip(self.state_augmented[IDX_R], -10.0, 10.0)
        idx_wr = self.state_dim
        idx_wf = self.state_dim + 1
        self.state_augmented[idx_wr] = np.clip(self.state_augmented[idx_wr], -500.0, 500.0)
        if self.udim >= 2:
            self.state_augmented[idx_wf] = np.clip(self.state_augmented[idx_wf], -500.0, 500.0)

        # Store innovation for diagnostics
        self.innovation = np.zeros(self.meas_dim)
        if active_indices is not None and len(innov) == len(active_indices):
            self.innovation[active_indices] = innov
        else:
            self.innovation[:len(innov)] = innov

        # Extract state and residual estimates
        self.state_hat = self.state_augmented[:self.state_dim].copy()
        self.w_hat = self.state_augmented[self.state_dim:].copy()

        # Update UIO-style residual constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0

        # Copy tire residuals to base class attribute
        self.f_uk_hat = self.w_hat.copy()

        return self.state_hat.copy(), self.w_hat.copy()

    def _process_measurement(self, measurement: np.ndarray,
                             acceleration: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Process input measurement to get full measurement vector.

        Handles:
            - Full 7D: [v_x, r, psi, X, Y, a_y, a_x]
            - No GPS 4D: [v_x, r, a_y, a_x]
            - Legacy 6D: [v_x, r, psi, X, Y, a_y]
            - Legacy 2D: [v_x, r]
        """
        measurement = measurement.reshape(-1)
        n_input = len(measurement)
        y = np.zeros(self.meas_dim)

        val_vx = measurement[0] if n_input > 0 else 0.0
        val_r = measurement[1] if n_input > 1 else 0.0

        if self.use_8d_system:
            y[MEAS8_IDX_VX] = val_vx
            y[MEAS8_IDX_R] = val_r
        else:
            y[MEAS_IDX_VX] = val_vx
            y[MEAS_IDX_R] = val_r

        val_psi = None
        val_X = None
        val_Y = None
        val_ay = None
        val_ax = None

        if n_input >= 7:
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]
            val_ay = measurement[5]
            val_ax = measurement[6]
        elif n_input == 6:
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]
            val_ay = measurement[5]
        elif n_input == 4:
            val_ay = measurement[2]
            val_ax = measurement[3]
        elif n_input == 5:
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]

        if acceleration is not None:
            if val_ax is None and len(acceleration) > 0:
                val_ax = acceleration[0]
            if val_ay is None and len(acceleration) > 1:
                val_ay = acceleration[1]

        if self.use_8d_system:
            idx_psi = IDX8_PSI
            idx_X = IDX8_X
            idx_Y = IDX8_Y
        else:
            idx_psi = IDX_PSI
            idx_X = IDX_X
            idx_Y = IDX_Y

        if val_psi is None: val_psi = self.state_hat[idx_psi]
        if val_X is None: val_X = self.state_hat[idx_X]
        if val_Y is None: val_Y = self.state_hat[idx_Y]

        if self.use_8d_system:
            y[MEAS8_IDX_PSI] = val_psi
            y[MEAS8_IDX_X] = val_X
            y[MEAS8_IDX_Y] = val_Y
            y[MEAS8_IDX_AY] = val_ay if val_ay is not None else 0.0
            y[MEAS8_IDX_AX] = val_ax if val_ax is not None else 0.0
        else:
            y[MEAS_IDX_PSI] = val_psi
            y[MEAS_IDX_X] = val_X
            y[MEAS_IDX_Y] = val_Y
            y[MEAS_IDX_AY] = val_ay if val_ay is not None else (val_r * val_vx)

        return y

    # =========================================================
    # Getters
    # =========================================================

    def get_state(self) -> np.ndarray:
        return self.state_hat.copy()

    def get_tire_residuals(self) -> np.ndarray:
        return self.w_hat.copy()

    def get_augmented_state(self) -> np.ndarray:
        return self.state_augmented.copy()

    def get_unknown_input(self) -> np.ndarray:
        return self.w_hat.copy()

    def get_unknown_input_estimate(self) -> np.ndarray:
        start_idx = self.state_dim
        end_idx = self.state_dim + self.udim
        return self.state_augmented[start_idx:end_idx].copy()

    def get_ay_constraint(self) -> float:
        return self.w_constraint

    def get_innovation(self) -> np.ndarray:
        return self.innovation.copy()

    def get_observability_flags(self) -> np.ndarray:
        return self.observability_flags.copy()

    def get_yaw_wrap_status(self) -> Tuple[bool, int]:
        return (self.yaw_wrap_counter > 0, self.yaw_wrap_counter)

    def check_uio_rank_condition(self, delta: float) -> bool:
        rho = self.compute_scheduling_params(self.state_hat, delta)
        C = self.compute_C_matrix(rho)
        E = self.compute_E_matrix(rho)
        F = self.compute_F_matrix(rho)
        CE = C @ E
        stacked = np.vstack([CE, F])
        rank_stacked = np.linalg.matrix_rank(stacked)
        rank_E = np.linalg.matrix_rank(E)
        return rank_stacked == rank_E == 2

    def reset(self, initial_state: Optional[np.ndarray] = None,
              initial_position: Optional[np.ndarray] = None):
        """Reset observer state"""
        if initial_state is not None:
            initial_state = initial_state.reshape(-1)
            if len(initial_state) >= self.state_dim:
                self.state_hat = initial_state[:self.state_dim].copy()
            else:
                self.state_hat = np.zeros(self.state_dim)
                self.state_hat[:len(initial_state)] = initial_state
                if initial_position is not None:
                    if self.use_8d_system:
                        self.state_hat[IDX8_X] = initial_position[0]
                        self.state_hat[IDX8_Y] = initial_position[1]
                    else:
                        self.state_hat[IDX_X] = initial_position[0]
                        self.state_hat[IDX_Y] = initial_position[1]
        else:
            self.state_hat = np.zeros(self.state_dim)
            if initial_position is not None:
                if self.use_8d_system:
                    self.state_hat[IDX8_X] = initial_position[0]
                    self.state_hat[IDX8_Y] = initial_position[1]
                else:
                    self.state_hat[IDX_X] = initial_position[0]
                    self.state_hat[IDX_Y] = initial_position[1]
        self.current_steering_angle = 0.0
        if self.rdot_diff is not None:
            self.rdot_diff.reset(0.0)
            self.rdot_meas = 0.0
        self.yaw_wrap_counter = 0
        self.prev_psi_meas = 0.0
        self.observability_flags = np.ones(self.udim)
        self.state_augmented = np.zeros(self.augmented_dim)
        self.state_augmented[:self.state_dim] = self.state_hat
        self.w_hat = np.zeros(self.udim)
        self.f_uk_hat = np.zeros(self.udim)
        self.gyro_bias = 0.0
        self.ay_innovation = 0.0
        self.w_constraint = 0.0
        self.innovation = np.zeros(self.meas_dim)

    def get_gain_scheduler(self) -> Optional[QLPVGainScheduler]:
        return self.gain_scheduler

    def get_gains_summary(self) -> str:
        if self.gain_scheduler is not None:
            return self.gain_scheduler.get_all_gains_summary()
        else:
            lines = ["Observer Gains (default, no scheduling):"]
            lines.append("=" * 60)
            lines.append(f"\nL_state ({self.L_state.shape[0]}x{self.L_state.shape[1]}):")
            if self.L_state.shape[0] == self.L_state.shape[1]:
                lines.append(f"  Diagonal: {np.diag(self.L_state)}")
            lines.append(f"  Frobenius norm: {np.linalg.norm(self.L_state, 'fro'):.4f}")
            lines.append(f"\nL_residual ({self.L_residual.shape[0]}x{self.L_residual.shape[1]}):")
            lines.append(f"  {self.L_residual}")
            lines.append(f"\nL_augmented ({self.L_augmented.shape[0]}x{self.L_augmented.shape[1]}):")
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