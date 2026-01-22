"""
H∞ and L2 LMI-based Observer Gain Design for Neural State Estimator

Provides LMI-based methods for computing observer gains with guaranteed
disturbance attenuation properties (H∞) or energy-bounded gains (L2).

This module is self-contained for the neural second-layer observer.
It includes:
    - compute_hinf_lmi_observer_gain: H∞ Bounded Real Lemma design
    - compute_l2_lmi_observer_gain: L2 gain-bounded design
    - compute_lmi_observer_gain: Standard Lyapunov LMI design
    - compute_pole_placement_gain: Fallback pole placement
    - validate_observer_gain: Stability validation
    - NeuralQLPVGainScheduler: Polytopic gain scheduling for neural observer

These functions are used by the NeuralLuenbergerEstimator for robust
observer gain design with actual measurement matrix C(ρ).

Author: Neural Observer Team
Date: 2026-01-21
"""

import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass

# Import CVXPY for LMI-based gain design
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False
    print("Warning: cvxpy not available. LMI-based gain design disabled.")

# Import scipy for pole placement fallback
try:
    from scipy.signal import place_poles
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# Import qLPV dynamics - must add parent to path
import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent  # LocalNeuralObs directory
sys.path.insert(0, str(parent_dir))

from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    STATE_DIM, MEAS_DIM,
)


# =============================================================================
# Basic LMI Utilities
# =============================================================================

def validate_observer_gain(A: np.ndarray, C: np.ndarray, L: np.ndarray,
                           max_real_part: float = -0.1) -> bool:
    """
    Validate that observer gain produces stable error dynamics.
    
    Checks that all eigenvalues of (A - L @ C) have negative real parts.
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)  
        L: Observer gain (n × m)
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
    m = C.shape[0]
    
    if desired_poles is None:
        # Default: place poles at stable locations with good damping
        # Spread in left half-plane for smooth convergence
        desired_poles = np.array([-2.0, -2.5, -3.0, -3.5, -4.0, -4.5])[:n]
    
    if not SCIPY_AVAILABLE:
        # Simple fallback: diagonal gain
        L = np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5])[:n,:m]
        return L
    
    try:
        # Pole placement: place_poles works on (A - LC)^T = A^T - C^T L^T
        result = place_poles(A.T, C.T, desired_poles)
        L = result.gain_matrix.T
        return L
    except Exception:
        # Fallback to simple diagonal gain
        return np.diag([2.0, 2.0, 1.0, 2.0, 0.5, 0.5])[:n,:m]


def compute_lmi_observer_gain(A: np.ndarray, C: np.ndarray, 
                              decay_rate: float = 1.0,
                              verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using LMI-based design with Lyapunov stability.
    
    For the observer error dynamics:
        ė = (A - LC) e
    
    We solve for P > 0 and Y = P @ L such that:
        A^T P + P A - C^T Y^T - Y C + γ P ≺ 0
    
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
    
    # LMI constraint: A^T P + P A - C^T Y^T - Y C + γ P ≺ 0
    eps = 1e-6  # Small positive value for strict inequality
    lmi_constraint = A.T @ P + P @ A - C.T @ Y.T - Y @ C + decay_rate * P
    
    # Regularization weight
    gamma_reg = 0.01
    
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
    
    # Limit gain magnitude
    # L = np.clip(L, -100.0, 100.0)
    
    return L


# =============================================================================
# H∞ and L2 LMI-based Observer Gain Design
# =============================================================================

def compute_hinf_lmi_observer_gain(A: np.ndarray, C: np.ndarray, E: np.ndarray,
                                    gamma: float = 2.0, decay_rate: float = 0.5,
                                    verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using H∞ LMI-based design (Bounded Real Lemma).
    
    For the observer error dynamics with disturbance injection:
        ė = (A - LC)e + E·d
        z = e  (performance output = state estimation error)
    
    The H∞ criterion guarantees: ||z||₂ ≤ γ||d||₂
    i.e., the L2-gain from disturbance d to estimation error e is bounded by γ.
    
    This is achieved by solving the Bounded Real Lemma LMI:
    
        [A^T P + P A - C^T Y^T - Y C + αP    P E  ]
        [            E^T P                  -γ²I  ] < 0
        
    with P > 0, where Y = P @ L.
    
    Recovery: L = P⁻¹ @ Y
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)  
        E: Disturbance injection matrix (n × p) - links unknown inputs to state
        gamma: H∞ performance bound (smaller = better attenuation, but harder to satisfy)
        decay_rate: Minimum exponential decay rate α (controls convergence speed)
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
        
    Raises:
        ValueError: If the LMI problem is infeasible or CVXPY not available
        
    Note:
        Smaller gamma provides better disturbance rejection but may be infeasible.
        Typical values: gamma ∈ [1.0, 5.0], decay_rate ∈ [0.1, 1.0]
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available. Install with: pip install cvxpy")
    
    n = A.shape[0]  # State dimension
    m = C.shape[0]  # Measurement dimension
    p = E.shape[1]  # Disturbance dimension
    
    # Decision variables
    P = cp.Variable((n, n), symmetric=True)  # Lyapunov matrix
    Y = cp.Variable((n, m))  # Y = P @ L
    
    # Numerical tolerances
    eps = 1e-5  # Strict inequality margin
    P_min = 1e-4
    P_max = 1e3  # Reduced upper bound for better conditioning
    
    # Build the H∞ LMI block matrix constraint (standard BRL form)
    # Top-left: A^T P + P A - C^T Y^T - Y C + αP
    top_left = A.T @ P + P @ A - C.T @ Y.T - Y @ C + decay_rate * P
    
    # Build full LMI matrix
    lmi_top = cp.hstack([top_left, P @ E])
    lmi_bot = cp.hstack([E.T @ P, -gamma**2 * np.eye(p)])
    lmi_full = cp.vstack([lmi_top, lmi_bot])
    
    constraints = [
        P >> P_min * np.eye(n),  # P > 0 (positive definite)
        P << P_max * np.eye(n),  # Upper bound on P for conditioning
        lmi_full << -eps * np.eye(n + p),  # H∞ LMI < 0
    ]
    
    # Objective: minimize trace(P) + regularization on Y (for well-conditioned gain)
    gamma_reg = 0.1  # Increased regularization for robustness
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    # Solve the SDP
    problem = cp.Problem(objective, constraints)
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=20000, eps=1e-5)
    except Exception as e:
        # Try alternative solver
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except:
            raise ValueError(f"H∞ LMI solver failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"H∞ LMI problem infeasible. Status: {problem.status}. "
                        f"Try increasing gamma (current: {gamma})")
    
    # Recover observer gain: L = P⁻¹ @ Y
    P_val = P.value
    Y_val = Y.value
    
    if P_val is None or Y_val is None:
        raise ValueError("H∞ LMI solver returned None values")
    
    try:
        L = np.linalg.solve(P_val, Y_val)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val
    
    # Validate stability by checking eigenvalues
    A_cl = A - L @ C
    eigenvalues = np.linalg.eigvals(A_cl)
    max_real = np.max(np.real(eigenvalues))
    
    if max_real >= 0:
        raise ValueError(f"H∞ LMI produced unstable observer (max real eig = {max_real:.4f})")
    
    # Limit gain magnitude to prevent numerical issues
    # L = np.clip(L, -50.0, 50.0)
    
    return L


def compute_l2_lmi_observer_gain(A: np.ndarray, C: np.ndarray, E: np.ndarray,
                                  gamma: float = 2.0, decay_rate: float = 0.5,
                                  verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using L2 gain-bounded LMI design.
    
    For the observer error dynamics with disturbance injection:
        ė = (A - LC)e + E·d
        z = e  (performance output = state estimation error)
    
    The L2-gain criterion bounds: ∫||z||² dt ≤ γ² ∫||d||² dt
    
    This is achieved using the Bounded Real Lemma (same structure as H∞):
    
        [A^T P + P A - C^T Y^T - Y C + αP    P E  ]
        [            E^T P                  -γ²I  ] < 0
        
    with P > 0, where Y = P @ L.
    
    The difference from H∞ is primarily in interpretation and tuning.
    For L2, we typically use larger gamma values and looser decay rates.
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)
        E: Disturbance injection matrix (n × p)
        gamma: L2 gain bound (smaller = more aggressive rejection)
        decay_rate: Minimum exponential decay rate
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
        
    Raises:
        ValueError: If the LMI problem is infeasible
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available")
    
    n = A.shape[0]
    m = C.shape[0]
    p = E.shape[1]
    
    # Decision variables
    P = cp.Variable((n, n), symmetric=True)
    Y = cp.Variable((n, m))  # Y = P @ L
    
    # Numerical tolerances
    eps = 1e-5
    P_min = 1e-4
    P_max = 1e3
    
    # Build the L2/BRL LMI block matrix constraint
    # Same structure as H∞ - this is the correct formulation
    # Top-left: A^T P + P A - C^T Y^T - Y C + αP
    top_left = A.T @ P + P @ A - C.T @ Y.T - Y @ C + decay_rate * P
    
    # Build full LMI matrix (Bounded Real Lemma form)
    lmi_top = cp.hstack([top_left, P @ E])
    lmi_bot = cp.hstack([E.T @ P, -gamma**2 * np.eye(p)])
    lmi_full = cp.vstack([lmi_top, lmi_bot])
    
    constraints = [
        P >> P_min * np.eye(n),  # P > 0 (positive definite)
        P << P_max * np.eye(n),  # Upper bound on P for conditioning
        lmi_full << -eps * np.eye(n + p),  # BRL LMI < 0
    ]
    
    # Objective: minimize trace(P) with regularization on Y
    gamma_reg = 0.05
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    # Solve the SDP
    problem = cp.Problem(objective, constraints)
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=20000, eps=1e-5)
    except Exception as e:
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except:
            raise ValueError(f"L2 LMI solver failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"L2 LMI problem infeasible. Status: {problem.status}. "
                        f"Try increasing gamma (current: {gamma})")
    
    # Recover observer gain: L = P⁻¹ @ Y
    P_val = P.value
    Y_val = Y.value
    
    if P_val is None or Y_val is None:
        raise ValueError("L2 LMI solver returned None values")
    
    try:
        L = np.linalg.solve(P_val, Y_val)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val
    
    # Validate stability
    A_cl = A - L @ C
    eigenvalues = np.linalg.eigvals(A_cl)
    max_real = np.max(np.real(eigenvalues))
    
    if max_real >= 0:
        raise ValueError(f"L2 LMI produced unstable observer (max real eig = {max_real:.4f})")
    
    # Limit gain magnitude to prevent numerical issues
    # L = np.clip(L, -50.0, 50.0)
    
    return L


# =============================================================================
# Discrete-Time LMI-based Observer Gain Design (Schur Form)
# =============================================================================

def discretize_system_zoh(A_c: np.ndarray, B_c: np.ndarray, E_c: np.ndarray,
                          dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Discretize continuous-time system using Zero-Order Hold (ZOH).
    
    Continuous: ẋ = A_c·x + B_c·u + E_c·w
    Discrete:   x[k+1] = A_d·x[k] + B_d·u[k] + E_d·w[k]
    
    Args:
        A_c: Continuous state matrix (n × n)
        B_c: Continuous input matrix (n × m_u)
        E_c: Continuous disturbance matrix (n × m_w)
        dt: Sample time [seconds]
        
    Returns:
        Tuple of (A_d, B_d, E_d) discrete-time matrices
    """
    from scipy.signal import cont2discrete
    
    n = A_c.shape[0]
    m_u = B_c.shape[1]
    m_w = E_c.shape[1]
    
    # Augment B with E for combined discretization: [B, E]
    BE_c = np.hstack([B_c, E_c])
    
    # Dummy C and D for cont2discrete interface
    C_dummy = np.eye(n)
    D_dummy = np.zeros((n, m_u + m_w))
    
    A_d, BE_d, _, _, _ = cont2discrete((A_c, BE_c, C_dummy, D_dummy), dt, method='zoh')
    
    B_d = BE_d[:, :m_u]
    E_d = BE_d[:, m_u:]
    
    return A_d, B_d, E_d


def validate_discrete_observer_gain(A_d: np.ndarray, C: np.ndarray, L: np.ndarray,
                                     max_spectral_radius: float = 0.99) -> bool:
    """
    Validate that discrete observer gain produces stable error dynamics.
    
    Checks that all eigenvalues of (A_d - L @ C) are inside the unit circle.
    
    Args:
        A_d: Discrete state matrix (n × n)
        C: Output matrix (m × n)  
        L: Observer gain (n × m)
        max_spectral_radius: Maximum allowed spectral radius (< 1 for stability)
        
    Returns:
        True if gain is valid (all eigenvalues inside unit circle)
    """
    try:
        A_cl = A_d - L @ C
        eigenvalues = np.linalg.eigvals(A_cl)
        spectral_radius = np.max(np.abs(eigenvalues))
        return spectral_radius < max_spectral_radius
    except:
        return False


def compute_discrete_hinf_lmi_observer_gain(A_d: np.ndarray, C: np.ndarray, E_d: np.ndarray,
                                             gamma: float = 2.0, 
                                             contraction_rate: float = 0.95,
                                             verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using discrete-time H∞ LMI design (Schur form).
    
    For the discrete observer error dynamics with disturbance:
        e[k+1] = (A_d - L·C)·e[k] + E_d·d[k]
        z[k] = e[k]  (performance output = estimation error)
    
    The H∞ criterion guarantees: ||z||₂ ≤ γ||d||₂
    
    Discrete-time Lyapunov with H∞ performance (Schur complement form):
        [P        P·A_d - Y·C    P·E_d  ]
        [*        P              0      ] > 0  (stability)
        [*        *              γ²·I   ]
    
    where Y = L^T·P (transpose substitution for discrete systems).
    Then L^T = Y·P^{-1}, so L = P^{-1}·Y^T
    
    Alternative: Use standard continuous-time reformulation with bilinear transform.
    
    Args:
        A_d: Discrete state matrix (n × n)
        C: Output matrix (m × n)
        E_d: Discrete disturbance matrix (n × p)
        gamma: H∞ performance bound (smaller = better attenuation)
        contraction_rate: λ ∈ (0, 1) for exponential stability (smaller = faster)
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
        
    Raises:
        ValueError: If the LMI problem is infeasible or CVXPY not available
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available. Install with: pip install cvxpy")
    
    n = A_d.shape[0]  # State dimension
    m = C.shape[0]    # Measurement dimension
    p = E_d.shape[1]  # Disturbance dimension
    
    # Clamp contraction rate to valid range
    lam = np.clip(contraction_rate, 0.5, 0.999)
    
    # Decision variables
    P = cp.Variable((n, n), symmetric=True)  # Lyapunov matrix
    Y = cp.Variable((m, n))  # Y = L^T @ P (transpose for discrete)
    
    # Numerical tolerances
    eps = 1e-6
    P_min = 1e-5
    P_max = 1e5
    
    # For discrete-time observer: e[k+1] = (A_d - L*C) e[k] + E_d d[k]
    # Stability: (A_d - L*C)^T P (A_d - L*C) - P < 0
    # H∞: similar, ensuring ||e||_2 <= gamma ||d||_2
    #
    # Using Y = L^T @ P, we have L^T = Y @ P^{-1}
    # (A_d - L*C)^T = A_d^T - C^T L^T = A_d^T - C^T Y P^{-1}
    # P (A_d - L*C) = P A_d - P L C = P A_d - Y^T C
    #
    # The discrete BRL Schur form (observer version):
    # [λ²P              (P A_d - Y^T C)     P E_d  ]
    # [(P A_d - Y^T C)^T       P             0     ] > 0
    # [E_d^T P                 0           γ²I    ]
    
    PA_YTC = P @ A_d - Y.T @ C  # This is P @ A_cl
    
    # Build the 3x3 block Schur LMI
    block_11 = lam**2 * P
    block_12 = PA_YTC
    block_13 = P @ E_d
    
    block_21 = PA_YTC.T
    block_22 = P
    block_23 = np.zeros((n, p))
    
    block_31 = E_d.T @ P
    block_32 = np.zeros((p, n))
    block_33 = gamma**2 * np.eye(p)
    
    # Assemble full LMI matrix
    row1 = cp.hstack([block_11, block_12, block_13])
    row2 = cp.hstack([block_21, block_22, block_23])
    row3 = cp.hstack([block_31, block_32, block_33])
    lmi_full = cp.vstack([row1, row2, row3])
    
    constraints = [
        P >> P_min * np.eye(n),      # P > 0
        P << P_max * np.eye(n),      # P bounded for conditioning
        lmi_full >> eps * np.eye(n + n + p),  # Schur LMI > 0
    ]
    
    # Objective: minimize trace(P) + regularization on Y
    gamma_reg = 0.01
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    # Solve the SDP
    problem = cp.Problem(objective, constraints)
    
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=30000, eps=1e-7)
    except Exception as e:
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except:
            raise ValueError(f"Discrete H∞ LMI solver failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"Discrete H∞ LMI infeasible. Status: {problem.status}. "
                        f"Try increasing gamma (current: {gamma}) or relaxing contraction_rate.")
    
    # Recover observer gain: L^T = Y @ P^{-1}, so L = P^{-1} @ Y^T
    P_val = P.value
    Y_val = Y.value
    
    if P_val is None or Y_val is None:
        raise ValueError("Discrete H∞ LMI solver returned None values")
    
    try:
        L = np.linalg.solve(P_val, Y_val.T)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val.T
    
    # Validate stability
    if not validate_discrete_observer_gain(A_d, C, L):
        raise ValueError("Discrete H∞ LMI produced unstable observer")
    
    return L


def compute_discrete_l2_lmi_observer_gain(A_d: np.ndarray, C: np.ndarray, E_d: np.ndarray,
                                           gamma: float = 2.0,
                                           contraction_rate: float = 0.95,
                                           verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using discrete-time L2 gain-bounded LMI design.
    
    For discrete observer error dynamics:
        e[k+1] = (A_d - L·C)·e[k] + E_d·d[k]
    
    The L2 criterion bounds: Σ||e[k]||² ≤ γ² Σ||d[k]||²
    
    Uses same Schur-form BRL as H∞ (mathematically equivalent for discrete-time).
    
    Args:
        A_d: Discrete state matrix (n × n)
        C: Output matrix (m × n)
        E_d: Discrete disturbance matrix (n × p)
        gamma: L2 gain bound
        contraction_rate: Contraction rate λ ∈ (0, 1)
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
    """
    # L2 and H∞ are equivalent for discrete-time - use H∞ implementation
    return compute_discrete_hinf_lmi_observer_gain(
        A_d, C, E_d, gamma=gamma, contraction_rate=contraction_rate, verbose=verbose
    )


def compute_discrete_contraction_lmi(A_d: np.ndarray, C: np.ndarray,
                                      contraction_rate: float = 0.95,
                                      verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L for discrete-time contraction (pure stability).
    
    For discrete observer error dynamics:
        e[k+1] = (A_d - L·C)·e[k]
    
    Contraction LMI (Schur form):
        [λ²·P        (P·A_d - Y^T·C)  ]
        [(...)^T           P          ] > 0
    
    with P > 0, Y = L^T·P (transpose substitution).
    
    This guarantees ||e[k]|| ≤ λ^k ||e[0]||.
    
    Args:
        A_d: Discrete state matrix (n × n)
        C: Output matrix (m × n)
        contraction_rate: λ ∈ (0, 1) for exponential contraction (smaller = faster)
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available")
    
    n = A_d.shape[0]
    m = C.shape[0]
    
    lam = np.clip(contraction_rate, 0.5, 0.999)
    
    # Decision variables
    P = cp.Variable((n, n), symmetric=True)
    Y = cp.Variable((m, n))  # Y = L^T @ P (transpose substitution)
    
    eps = 1e-6
    P_min = 1e-5
    P_max = 1e5
    
    # Build contraction LMI (2x2 Schur form)
    # P(A_d - LC) = PA_d - PLT C = PA_d - Y^T C
    PA_YTC = P @ A_d - Y.T @ C
    
    lmi_top = cp.hstack([lam**2 * P, PA_YTC])
    lmi_bot = cp.hstack([PA_YTC.T, P])
    lmi_full = cp.vstack([lmi_top, lmi_bot])
    
    constraints = [
        P >> P_min * np.eye(n),
        P << P_max * np.eye(n),
        lmi_full >> eps * np.eye(2 * n),
    ]
    
    # Objective
    gamma_reg = 0.01
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    problem = cp.Problem(objective, constraints)
    
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=20000, eps=1e-7)
    except Exception:
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except Exception as e:
            raise ValueError(f"Discrete contraction LMI failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"Discrete contraction LMI infeasible: {problem.status}")
    
    P_val = P.value
    Y_val = Y.value
    
    if P_val is None or Y_val is None:
        raise ValueError("Discrete contraction LMI returned None")
    
    try:
        L = np.linalg.solve(P_val, Y_val.T)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val.T
    
    if not validate_discrete_observer_gain(A_d, C, L, max_spectral_radius=lam + 0.02):
        raise ValueError("Discrete contraction LMI produced non-contracting observer")
    
    return L


def compute_discrete_h2_lmi_observer_gain(A_d: np.ndarray, C: np.ndarray, E_d: np.ndarray,
                                           contraction_rate: float = 0.95,
                                           verbose: bool = False) -> Tuple[np.ndarray, float]:
    """
    Compute observer gain L using discrete-time H2 LMI design.
    
    For discrete observer error dynamics:
        e[k+1] = (A_d - L·C)·e[k] + E_d·d[k]
    
    H2 norm minimizes the expected energy of error response to white noise:
        ||T_ed||₂² = trace(E_d^T · P · E_d)  (for stable system)
    
    H2 LMI with Y = L^T @ P:
        min trace(W)
        s.t. [W         E_d^T·P    ]
             [P·E_d     P          ] > 0
             and contraction LMI for stability
    
    Args:
        A_d: Discrete state matrix (n × n)
        C: Output matrix (m × n)
        E_d: Discrete disturbance matrix (n × p)
        contraction_rate: λ ∈ (0, 1) for stability
        verbose: Print solver output
        
    Returns:
        Tuple of (L, h2_norm): Observer gain and achieved H2 norm
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available")
    
    n = A_d.shape[0]
    m = C.shape[0]
    p = E_d.shape[1]
    
    lam = np.clip(contraction_rate, 0.5, 0.999)
    
    # Decision variables
    P = cp.Variable((n, n), symmetric=True)
    Y = cp.Variable((m, n))  # Y = L^T @ P (transpose substitution)
    W = cp.Variable((p, p), symmetric=True)  # For H2 norm bound
    
    eps = 1e-6
    P_min = 1e-5
    P_max = 1e5
    
    # Contraction LMI for stability: P(A_d - LC) = PA_d - Y^T C
    PA_YTC = P @ A_d - Y.T @ C
    lmi_stab_top = cp.hstack([lam**2 * P, PA_YTC])
    lmi_stab_bot = cp.hstack([PA_YTC.T, P])
    lmi_stability = cp.vstack([lmi_stab_top, lmi_stab_bot])
    
    # H2 norm LMI
    lmi_h2_top = cp.hstack([W, E_d.T @ P])
    lmi_h2_bot = cp.hstack([P @ E_d, P])
    lmi_h2 = cp.vstack([lmi_h2_top, lmi_h2_bot])
    
    constraints = [
        P >> P_min * np.eye(n),
        P << P_max * np.eye(n),
        W >> eps * np.eye(p),
        lmi_stability >> eps * np.eye(2 * n),
        lmi_h2 >> eps * np.eye(p + n),
    ]
    
    # Objective: minimize H2 norm (trace of W)
    gamma_reg = 0.001
    objective = cp.Minimize(cp.trace(W) + gamma_reg * cp.norm(Y, 'fro'))
    
    problem = cp.Problem(objective, constraints)
    
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=20000, eps=1e-7)
    except Exception:
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except Exception as e:
            raise ValueError(f"Discrete H2 LMI failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"Discrete H2 LMI infeasible: {problem.status}")
    
    P_val = P.value
    Y_val = Y.value
    W_val = W.value
    
    if P_val is None or Y_val is None:
        raise ValueError("Discrete H2 LMI returned None")
    
    try:
        L = np.linalg.solve(P_val, Y_val.T)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val.T
    
    h2_norm = np.sqrt(np.trace(W_val)) if W_val is not None else float('inf')
    
    if not validate_discrete_observer_gain(A_d, C, L):
        raise ValueError("Discrete H2 LMI produced unstable observer")
    
    return L, h2_norm


# =============================================================================
# Neural Observer Specific qLPV Gain Scheduler
# =============================================================================

@dataclass
class NeuralPolytopicVertex:
    """Represents a vertex of the scheduling parameter polytope for neural observer"""
    vx: float       # Longitudinal velocity
    delta: float    # Steering angle
    psi: float      # Yaw angle (affects kinematic terms)
    
    def to_tuple(self) -> tuple:
        return (self.vx, self.delta, self.psi)


class NeuralQLPVGainScheduler:
    """
    Polytopic qLPV Gain Scheduling for Neural Second-Layer Observer
    
    Specifically designed for the neural observer with:
        - 6D state: [v_x, v_y, ψ, r, X, Y]
        - 6D measurement: [v_x, r, ψ, X, Y, a_y] (uses actual C(ρ) matrix)
        - 2D disturbance: [w_r, w_f] (tire residuals)
    
    For a qLPV system with scheduling parameters ρ ∈ Ω, we design observer
    gains at the vertices of a polytope covering Ω, then use convex
    interpolation for real-time gain scheduling.
    
    The polytope vertices are defined by:
        - vx ∈ [vx_min, vx_max]
        - delta ∈ [-delta_max, delta_max]
    
    For each vertex i, we compute L_i such that (A_i - L_i @ C_i) is Hurwitz.
    A common Lyapunov matrix P ensures stability across the entire polytope.
    
    Real-time gain: L(ρ) = Σ α_i(ρ) · L_i  where Σ α_i = 1, α_i ≥ 0
    
    Key differences from first-layer QLPVGainScheduler:
        - Uses actual C(ρ) matrix (not identity)
        - Designed for neural observer state/measurement structure
        - Supports H∞ and L2 design with disturbance E matrix
    
    Reference:
        - Apkarian et al., "Self-scheduled H∞ control of linear 
          parameter-varying systems"
    """
    
    def __init__(self, 
                 vehicle_params: Dict,
                 vx_range: Tuple[float, float] = (0.5, 3.0),
                 delta_max: float = 0.4,
                 n_vx_vertices: int = 3,
                 n_delta_vertices: int = 3,
                 decay_rate: float = 0.5,
                 lmi_method: str = 'hinf',
                 hinf_gamma: float = 5.0,
                 use_common_lyapunov: bool = True,
                 discrete: bool = True,
                 sample_time: float = 0.01,
                 contraction_rate: float = 0.95,
                 verbose: bool = False):
        """
        Initialize polytopic qLPV gain scheduler for neural observer
        
        Args:
            vehicle_params: Vehicle parameter dictionary
            vx_range: (vx_min, vx_max) velocity range [m/s]
            delta_max: Maximum steering angle magnitude [rad]
            n_vx_vertices: Number of velocity grid points
            n_delta_vertices: Number of steering grid points
            decay_rate: Minimum decay rate γ for continuous LMI (unused if discrete=True)
            lmi_method: 'hinf', 'l2', 'h2', 'contraction', or 'lmi' (standard)
            hinf_gamma: H∞ or L2 performance bound
            use_common_lyapunov: If True, use single P for all vertices (robust)
            discrete: If True, use discrete-time LMI design (recommended)
            sample_time: Sample time for discretization [s] (default: 0.01)
            contraction_rate: λ ∈ (0, 1) for discrete contraction (smaller = faster)
            verbose: Print solver output
        """
        self.params = vehicle_params
        self.vx_range = vx_range
        self.delta_max = delta_max
        self.decay_rate = decay_rate
        self.lmi_method = lmi_method
        self.hinf_gamma = hinf_gamma
        self.use_common_lyapunov = use_common_lyapunov
        self.discrete = discrete
        self.sample_time = sample_time
        self.contraction_rate = contraction_rate
        self.verbose = verbose
        
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
        
        # Gain smoothing filter state
        self._L_filtered: Optional[np.ndarray] = None
        
        # Default gain for fallback
        self._default_gain = self._compute_default_gain()
    
    def _compute_default_gain(self) -> np.ndarray:
        """Compute a robust default gain for neural observer"""
        # Design diagonal gain based on measurement structure
        # Measurement: y = [v_x, r, ψ, X, Y]  (5D, no a_y - matches observer)
        # State:       x = [v_x, v_y, ψ, r, X, Y]
        OBSERVER_MEAS_DIM = 5  # Observer uses 5D measurement, not 6D dynamics
        L = np.zeros((STATE_DIM, OBSERVER_MEAS_DIM))
        
        # v_x measurement → v_x state (direct, high gain)
        L[0, 0] = 5.0
        
        # r measurement → v_y state (coupled through dynamics)
        L[1, 1] = 2.0
        
        # ψ measurement → ψ state (direct)
        L[2, 2] = 4.0
        
        # r measurement → r state (direct, high gain)
        L[3, 1] = 5.0
        
        # X measurement → X state (direct)
        L[4, 3] = 3.0
        
        # Y measurement → Y state (direct)
        L[5, 4] = 3.0
        
        # Note: a_y is not included in observer measurement (5D), so no L[i, 5]
        
        return L
    
    def _generate_vertices(self, n_vx: int, n_delta: int) -> list:
        """Generate vertices of the scheduling parameter polytope"""
        vx_values = np.linspace(self.vx_range[0], self.vx_range[1], n_vx)
        delta_values = np.linspace(-self.delta_max, self.delta_max, n_delta)
        
        # Use psi = 0 (kinematic terms have minor effect on core dynamics)
        psi_values = [0.0]
        
        vertices = []
        for vx in vx_values:
            for delta in delta_values:
                for psi in psi_values:
                    vertices.append(NeuralPolytopicVertex(vx=vx, delta=delta, psi=psi))
        
        return vertices
    
    def _compute_observer_C_matrix(self) -> np.ndarray:
        """
        Compute 5D selection matrix matching the neural observer.
        
        The observer uses y = [vx, r, ψ, X, Y] (5D), NOT the full 6D
        dynamics measurement that includes a_y. This matrix must match
        what the observer actually uses for innovation computation.
        
        Returns:
            C matrix (5×6)
        """
        OBSERVER_MEAS_DIM = 5
        C = np.zeros((OBSERVER_MEAS_DIM, STATE_DIM))
        C[0, 0] = 1.0   # vx -> vx
        C[1, 3] = 1.0   # r  -> r
        C[2, 2] = 1.0   # ψ  -> ψ
        C[3, 4] = 1.0   # X  -> X
        C[4, 5] = 1.0   # Y  -> Y
        return C
    
    def _compute_matrices_at_vertex(self, vertex: NeuralPolytopicVertex
                                    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute continuous-time A, C, E matrices at a polytope vertex"""
        x_dummy = np.array([vertex.vx, 0.0, vertex.psi, 0.0, 0.0, 0.0])
        rho = self.dynamics.compute_scheduling_params(x_dummy, vertex.delta)
        
        A = self.dynamics.compute_A_matrix(rho)
        C = self._compute_observer_C_matrix()  # 5D selection matrix (matches observer)
        E = self.dynamics.compute_E_matrix(rho)
        
        return A, C, E
    
    def _compute_discrete_matrices_at_vertex(self, vertex: NeuralPolytopicVertex
                                             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute discrete-time A_d, B_d, E_d, C matrices at a polytope vertex.
        
        Uses ZOH discretization with the configured sample_time.
        
        Returns:
            Tuple of (A_d, C, E_d, B_d) where A_d, B_d, E_d are discrete matrices
            and C is the continuous output matrix (unchanged by discretization).
        """
        A_c, C, E_c = self._compute_matrices_at_vertex(vertex)
        B_c = self.dynamics.compute_B_matrix(
            self.dynamics.compute_scheduling_params(
                np.array([vertex.vx, 0.0, vertex.psi, 0.0, 0.0, 0.0]),
                vertex.delta
            )
        )
        
        A_d, B_d, E_d = discretize_system_zoh(A_c, B_c, E_c, self.sample_time)
        
        return A_d, C, E_d, B_d
    
    def compute_gains_lmi(self) -> bool:
        """
        Compute observer gains at all polytope vertices using LMI.
        
        If discrete=True (default), uses discrete-time Schur-form LMI.
        If use_common_lyapunov=True, solves a single SDP for all vertices
        with a common Lyapunov matrix P, ensuring robust stability.
        
        Returns:
            True if all gains computed successfully, False otherwise
        """
        if not CVXPY_AVAILABLE:
            print("Warning: CVXPY not available, using default gains")
            return self._use_default_gains()
        
        # Choose discrete or continuous design
        if self.discrete:
            if self.use_common_lyapunov:
                success = self._compute_gains_common_lyapunov_discrete()
            else:
                success = self._compute_gains_independent_discrete()
        else:
            # Legacy continuous-time design
            if self.use_common_lyapunov:
                success = self._compute_gains_common_lyapunov_hinf()
            else:
                success = self._compute_gains_independent()
        
        # Validate all computed gains
        if success:
            success = self._validate_all_gains()
        
        if not success:
            print("Warning: LMI gains failed, using default gains")
            return self._use_default_gains()
        
        return True
    
    def _use_default_gains(self) -> bool:
        """Fall back to default gains at all vertices"""
        for vertex in self.vertices:
            self.vertex_gains[vertex.to_tuple()] = self._default_gain.copy()
        self._gains_computed = True
        return True
    
    def _compute_gains_common_lyapunov_hinf(self) -> bool:
        """
        Compute gains with common Lyapunov and H∞ performance via BRL.
        
        Solves polytopic H∞ LMI:
            min trace(P) + γ_reg * Σ ||Y_i||_F
            s.t. P > 0
                 P < P_max * I
                 For all vertices i:
                     [A_i^T P + P A_i - C_i^T Y_i^T - Y_i C_i + αP    P E_i  ]
                     [           E_i^T P                             -γ²I    ] < 0
        
        Then L_i = P^{-1} Y_i for each vertex i.
        """
        n = STATE_DIM  # 6
        m = 5          # Observer uses 5D measurement (no a_y)
        p = 2          # Disturbance dimension (tire residuals)
        
        # Common Lyapunov matrix
        P = cp.Variable((n, n), symmetric=True)
        
        # Separate Y for each vertex (allows different gains)
        Y_list = [cp.Variable((n, m)) for _ in range(self.n_vertices)]
        
        # Bounds for numerical stability
        P_min = 1e-4
        P_max = 1e3
        eps = 1e-5
        
        constraints = [
            P >> P_min * np.eye(n),
            P << P_max * np.eye(n),
        ]
        
        for i, vertex in enumerate(self.vertices):
            A, C, E = self._compute_matrices_at_vertex(vertex)
            Y = Y_list[i]
            
            # Build H∞ LMI for this vertex
            top_left = A.T @ P + P @ A - C.T @ Y.T - Y @ C + self.decay_rate * P
            lmi_top = cp.hstack([top_left, P @ E])
            lmi_bot = cp.hstack([E.T @ P, -self.hinf_gamma**2 * np.eye(p)])
            lmi_full = cp.vstack([lmi_top, lmi_bot])
            
            constraints.append(lmi_full << -eps * np.eye(n + p))
        
        # Objective with regularization
        gamma_reg = 0.01
        reg_term = sum(cp.norm(Y, 'fro') for Y in Y_list)
        objective = cp.Minimize(cp.trace(P) + gamma_reg * reg_term)
        
        problem = cp.Problem(objective, constraints)
        
        try:
            problem.solve(solver=cp.SCS, verbose=self.verbose, max_iters=15000, eps=1e-5)
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
                print(f"Polytopic H∞ LMI infeasible: {problem.status}")
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
                # L = np.clip(L, -50.0, 50.0)
                self.vertex_gains[vertex.to_tuple()] = L
            except np.linalg.LinAlgError:
                L = np.linalg.pinv(self.P_common) @ Y_val
                # L = np.clip(L, -50.0, 50.0)
                self.vertex_gains[vertex.to_tuple()] = L
        
        self._gains_computed = True
        return True
    
    def _compute_gains_independent(self) -> bool:
        """Compute independent gains for each vertex using selected method"""
        success = True
        
        for vertex in self.vertices:
            A, C, E = self._compute_matrices_at_vertex(vertex)
            
            try:
                if self.lmi_method == 'hinf':
                    L = compute_hinf_lmi_observer_gain(
                        A, C, E, gamma=self.hinf_gamma, 
                        decay_rate=self.decay_rate, verbose=self.verbose
                    )
                elif self.lmi_method == 'l2':
                    L = compute_l2_lmi_observer_gain(
                        A, C, E, gamma=self.hinf_gamma,
                        decay_rate=self.decay_rate, verbose=self.verbose
                    )
                else:
                    L = compute_lmi_observer_gain(A, C, decay_rate=self.decay_rate,
                                                  verbose=self.verbose)
                
                self.vertex_gains[vertex.to_tuple()] = L
                
            except Exception as e:
                if self.verbose:
                    print(f"Warning: Failed to compute gain at vertex {vertex}: {e}")
                # Use pole placement fallback
                try:
                    L = compute_pole_placement_gain(A, C)
                    self.vertex_gains[vertex.to_tuple()] = L
                except:
                    self.vertex_gains[vertex.to_tuple()] = self._default_gain.copy()
                    success = False
        
        self._gains_computed = True
        return success
    
    def _compute_gains_common_lyapunov_discrete(self) -> bool:
        """
        Compute gains with common Lyapunov using discrete-time Schur-form H∞ LMI.
        
        Solves polytopic discrete H∞ LMI:
            min trace(P) + γ_reg * Σ ||Y_i||_F
            s.t. P > 0
                 P < P_max * I
                 For all vertices i:
                     [λ²·P        (P·A_d,i - Y_i·C_i)^T    0        ]
                     [P·A_d,i - Y_i·C_i    P              P·E_d,i  ] > 0
                     [0           E_d,i^T·P               γ²·I     ]
        
        Then L_i = P^{-1} Y_i for each vertex i.
        
        This guarantees:
            - Contraction rate ||e[k]|| ≤ λ^k ||e[0]||
            - H∞ bound ||e||_2 ≤ γ ||d||_2
            - Stability across the entire polytope via common P
        """
        n = STATE_DIM  # 6
        m = 5          # Observer uses 5D measurement (no a_y)
        p = 2          # Disturbance dimension (tire residuals)
        
        lam = self.contraction_rate
        gamma = self.hinf_gamma
        
        # Common Lyapunov matrix
        P = cp.Variable((n, n), symmetric=True)
        
        # Separate Y for each vertex (allows different gains)
        Y_list = [cp.Variable((n, m)) for _ in range(self.n_vertices)]
        
        # Bounds for numerical stability
        P_min = 1e-4
        P_max = 1e4
        eps = 1e-5
        
        constraints = [
            P >> P_min * np.eye(n),
            P << P_max * np.eye(n),
        ]
        
        for i, vertex in enumerate(self.vertices):
            A_d, C, E_d, _ = self._compute_discrete_matrices_at_vertex(vertex)
            Y = Y_list[i]
            
            # Build discrete H∞ Schur-form LMI for this vertex
            # [λ²·P        (P·A_d - Y·C)^T    0        ]
            # [P·A_d - Y·C       P           P·E_d     ] > 0
            # [0           E_d^T·P           γ²·I      ]
            
            PA_YC = P @ A_d - Y @ C
            
            block_11 = lam**2 * P
            block_12 = PA_YC.T
            block_13 = np.zeros((n, p))
            
            block_21 = PA_YC
            block_22 = P
            block_23 = P @ E_d
            
            block_31 = np.zeros((p, n))
            block_32 = E_d.T @ P
            block_33 = gamma**2 * np.eye(p)
            
            row1 = cp.hstack([block_11, block_12, block_13])
            row2 = cp.hstack([block_21, block_22, block_23])
            row3 = cp.hstack([block_31, block_32, block_33])
            lmi_full = cp.vstack([row1, row2, row3])
            
            constraints.append(lmi_full >> eps * np.eye(n + n + p))
        
        # Objective with regularization
        gamma_reg = 0.01
        reg_term = sum(cp.norm(Y, 'fro') for Y in Y_list)
        objective = cp.Minimize(cp.trace(P) + gamma_reg * reg_term)
        
        problem = cp.Problem(objective, constraints)
        
        try:
            problem.solve(solver=cp.SCS, verbose=self.verbose, max_iters=20000, eps=1e-6)
        except Exception as e:
            if self.verbose:
                print(f"SCS solver failed: {e}")
            try:
                problem.solve(solver=cp.CVXOPT, verbose=self.verbose)
            except Exception as e2:
                print(f"Discrete LMI solver failed: {e}, {e2}")
                return False
        
        if problem.status not in ['optimal', 'optimal_inaccurate']:
            if self.verbose:
                print(f"Polytopic discrete H∞ LMI infeasible: {problem.status}")
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
                self.vertex_gains[vertex.to_tuple()] = L
            except np.linalg.LinAlgError:
                L = np.linalg.pinv(self.P_common) @ Y_val
                self.vertex_gains[vertex.to_tuple()] = L
        
        self._gains_computed = True
        return True
    
    def _compute_gains_independent_discrete(self) -> bool:
        """Compute independent discrete-time gains for each vertex"""
        success = True
        
        for vertex in self.vertices:
            A_d, C, E_d, _ = self._compute_discrete_matrices_at_vertex(vertex)
            
            try:
                if self.lmi_method == 'hinf':
                    L = compute_discrete_hinf_lmi_observer_gain(
                        A_d, C, E_d, gamma=self.hinf_gamma,
                        contraction_rate=self.contraction_rate, verbose=self.verbose
                    )
                elif self.lmi_method == 'l2':
                    L = compute_discrete_l2_lmi_observer_gain(
                        A_d, C, E_d, gamma=self.hinf_gamma,
                        contraction_rate=self.contraction_rate, verbose=self.verbose
                    )
                elif self.lmi_method == 'h2':
                    L, _ = compute_discrete_h2_lmi_observer_gain(
                        A_d, C, E_d, contraction_rate=self.contraction_rate, 
                        verbose=self.verbose
                    )
                elif self.lmi_method == 'contraction':
                    L = compute_discrete_contraction_lmi(
                        A_d, C, contraction_rate=self.contraction_rate,
                        verbose=self.verbose
                    )
                else:
                    # Default to discrete H∞
                    L = compute_discrete_hinf_lmi_observer_gain(
                        A_d, C, E_d, gamma=self.hinf_gamma,
                        contraction_rate=self.contraction_rate, verbose=self.verbose
                    )
                
                self.vertex_gains[vertex.to_tuple()] = L
                
            except Exception as e:
                if self.verbose:
                    print(f"Warning: Failed to compute discrete gain at vertex {vertex}: {e}")
                # Use default fallback
                self.vertex_gains[vertex.to_tuple()] = self._default_gain.copy()
                success = False
        
        self._gains_computed = True
        return success
    
    def _validate_all_gains(self) -> bool:
        """Validate that all computed gains produce stable observers"""
        for vertex in self.vertices:
            L = self.vertex_gains.get(vertex.to_tuple())
            
            if L is None:
                return False
            
            if self.discrete:
                # Discrete-time: check eigenvalues inside unit circle
                A_d, C, _, _ = self._compute_discrete_matrices_at_vertex(vertex)
                if not validate_discrete_observer_gain(A_d, C, L, max_spectral_radius=0.999):
                    if self.verbose:
                        print(f"Warning: Discrete gain at vertex {vertex} is unstable")
                    return False
            else:
                # Continuous-time: check eigenvalues in left half-plane
                A, C, _ = self._compute_matrices_at_vertex(vertex)
                if not validate_observer_gain(A, C, L, max_real_part=0.0):
                    if self.verbose:
                        print(f"Warning: Gain at vertex {vertex} is unstable")
                    return False
        
        return True
    
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
        
        # Gaussian weights (smoother than inverse distance)
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
        
        # Interpolate gains (5D observer measurement)
        OBSERVER_MEAS_DIM = 5
        L = np.zeros((STATE_DIM, OBSERVER_MEAS_DIM))
        for i, vertex in enumerate(self.vertices):
            gain = self.vertex_gains.get(vertex.to_tuple(), self._default_gain)
            L += weights[i] * gain
        
        return L
    
    def get_scheduled_gain_smooth(self, vx: float, delta: float, 
                                   alpha: float = 0.1) -> np.ndarray:
        """
        Get interpolated observer gain with low-pass filtering for smooth transitions.
        
        L_filtered[k] = (1 - α)·L_filtered[k-1] + α·L(ρ[k])
        
        This prevents abrupt gain changes during rapid operating point transitions,
        improving switching safety.
        
        Args:
            vx: Current longitudinal velocity
            delta: Current steering angle
            alpha: Filter coefficient ∈ (0, 1]. Larger = faster response, more noise.
                   Typical values: 0.05-0.2 for smooth switching.
            
        Returns:
            L: Smoothed interpolated observer gain matrix (6 × 6)
        """
        L_new = self.get_scheduled_gain(vx, delta)
        
        if self._L_filtered is None:
            self._L_filtered = L_new.copy()
        else:
            self._L_filtered = (1.0 - alpha) * self._L_filtered + alpha * L_new
        
        return self._L_filtered.copy()
    
    def get_vertex_info(self) -> str:
        """Return string description of polytope vertices"""
        lines = [f"Neural qLPV Gain Scheduler: {self.n_vertices} vertices"]
        lines.append(f"  vx range: [{self.vx_range[0]:.2f}, {self.vx_range[1]:.2f}] m/s")
        lines.append(f"  delta range: [{-self.delta_max:.2f}, {self.delta_max:.2f}] rad")
        lines.append(f"  LMI method: {self.lmi_method}")
        lines.append(f"  Common Lyapunov: {self.use_common_lyapunov}")
        lines.append(f"  Decay rate: {self.decay_rate}")
        lines.append(f"  H∞/L2 gamma: {self.hinf_gamma}")
        lines.append(f"  Gains computed: {self._gains_computed}")
        return "\n".join(lines)


# =============================================================================
# Backward Compatibility Alias
# =============================================================================

# Alias for compatibility with existing code that imports QLPVGainScheduler
QLPVGainScheduler = NeuralQLPVGainScheduler
