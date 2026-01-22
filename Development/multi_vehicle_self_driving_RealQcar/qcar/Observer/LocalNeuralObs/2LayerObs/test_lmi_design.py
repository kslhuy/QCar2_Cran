"""
LMI Observer Gain Design Test

Tests the H-infinity and L2 LMI-based observer gain design functions
using the qLPV vehicle dynamics model.

This test:
1. Uses the centralized QLPVVehicleDynamicsObs model
2. No fallbacks - strict testing of LMI design
3. Logs all computed gains and diagnostics
4. Tests at multiple operating points (vertex polytope)

Author: Auto-generated for debugging LMI design issues
Date: 2026-01-21
"""

import numpy as np
import sys
import logging
from pathlib import Path
from dataclasses import dataclass
from typing import Tuple, Optional, List, Dict

# Setup paths
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir.parent))           # LocalNeuralObs
sys.path.insert(0, str(parent_dir))                  # 2LayerObs
sys.path.insert(0, str(parent_dir.parent / "1LayerObs"))  # 1LayerObs (for differentiator_uio_observer)

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Import qLPV dynamics
from qlpv_vehicle_dynamics_obs import (
    QLPVVehicleDynamicsObs,
    SchedulingParameters,
    get_vehicle_params,
    STATE_DIM,
)

# Import LMI design functions
from Design_LMI_neural import (
    compute_hinf_lmi_observer_gain,
    compute_l2_lmi_observer_gain,
    CVXPY_AVAILABLE,
)


# =============================================================================
# Diagnostic Utilities
# =============================================================================

@dataclass
class LMITestResult:
    """Results from a single LMI design test"""
    method: str
    vx: float
    delta: float
    success: bool
    L: Optional[np.ndarray] = None
    error_message: str = ""
    eigenvalues: Optional[np.ndarray] = None
    max_real_eig: float = float('inf')
    condition_number: float = float('inf')
    gamma: float = 0.0


def log_matrix(name: str, mat: np.ndarray, precision: int = 4):
    """Log a matrix with nice formatting"""
    logger.info(f"\n{name}:")
    for i, row in enumerate(mat):
        row_str = "  [" + ", ".join([f"{v:>10.{precision}f}" for v in row]) + "]"
        logger.info(row_str)


def check_observability(A: np.ndarray, C: np.ndarray) -> Tuple[bool, int]:
    """
    Check observability of (A, C) pair
    
    Returns:
        Tuple of (is_observable, rank)
    """
    n = A.shape[0]
    O = C.copy()
    for i in range(1, n):
        O = np.vstack([O, C @ np.linalg.matrix_power(A, i)])
    
    rank = np.linalg.matrix_rank(O)
    is_observable = rank == n
    
    return is_observable, rank


def analyze_stability(A: np.ndarray, C: np.ndarray, L: np.ndarray) -> Dict:
    """
    Analyze closed-loop observer stability
    
    Returns dict with:
        - eigenvalues: closed-loop eigenvalues
        - max_real: maximum real part
        - is_stable: True if all eigenvalues have negative real part
        - condition_number: condition number of A_cl
    """
    A_cl = A - L @ C
    eigenvalues = np.linalg.eigvals(A_cl)
    max_real = np.max(np.real(eigenvalues))
    
    return {
        'eigenvalues': eigenvalues,
        'max_real': max_real,
        'is_stable': max_real < 0,
        'condition_number': np.linalg.cond(A_cl),
    }


# =============================================================================
# LMI Design Test Functions
# =============================================================================

def test_hinf_design_at_point(
    dynamics: QLPVVehicleDynamicsObs,
    vx: float,
    delta: float,
    gamma: float = 2.0,
    decay_rate: float = 0.5
) -> LMITestResult:
    """
    Test H-infinity LMI design at a specific operating point
    
    Args:
        dynamics: qLPV dynamics model
        vx: Longitudinal velocity [m/s]
        delta: Steering angle [rad]
        gamma: H-infinity performance bound
        decay_rate: Minimum exponential decay rate
    
    Returns:
        LMITestResult with all diagnostics
    """
    result = LMITestResult(method='hinf', vx=vx, delta=delta, success=False, gamma=gamma)
    
    logger.info("="*80)
    logger.info(f"H-infinity LMI Design Test at vx={vx:.2f} m/s, δ={np.rad2deg(delta):.1f}°")
    logger.info("="*80)
    
    # Build state at operating point
    state = np.array([vx, 0.0, 0.0, 0.0, 0.0, 0.0])  # [vx, vy, psi, r, X, Y]
    
    # Compute scheduling parameters
    rho = dynamics.compute_scheduling_params(state, delta)
    
    logger.info(f"Scheduling parameters:")
    logger.info(f"  inv_vx={rho.inv_vx:.4f}, cos_δ={rho.cos_delta:.4f}, sin_δ={rho.sin_delta:.4f}")
    
    # Compute system matrices
    A = dynamics.compute_A_matrix(rho)
    E = dynamics.compute_E_matrix(rho)
    C = np.eye(STATE_DIM)  # Full state output
    
    log_matrix("A(ρ) matrix", A)
    log_matrix("E(ρ) matrix", E)
    
    # Check observability
    is_obs, obs_rank = check_observability(A, C)
    logger.info(f"\nObservability: {'✓ Observable' if is_obs else '✗ NOT Observable'} (rank={obs_rank}/{STATE_DIM})")
    
    # Open-loop eigenvalues
    eig_ol = np.linalg.eigvals(A)
    logger.info(f"Open-loop eigenvalues (A):")
    for i, e in enumerate(eig_ol):
        logger.info(f"  λ{i+1} = {e.real:>8.4f} + {e.imag:>8.4f}j")
    
    # Try H-infinity design
    logger.info(f"\nAttempting H∞ LMI design (γ={gamma}, α={decay_rate})...")
    
    try:
        L = compute_hinf_lmi_observer_gain(A, C, E, gamma=gamma, decay_rate=decay_rate, verbose=True)
        
        result.L = L
        log_matrix("Observer Gain L (H∞)", L)
        
        # Analyze stability
        stability = analyze_stability(A, C, L)
        result.eigenvalues = stability['eigenvalues']
        result.max_real_eig = stability['max_real']
        result.condition_number = stability['condition_number']
        
        logger.info(f"\nClosed-loop eigenvalues (A - L·C):")
        for i, e in enumerate(stability['eigenvalues']):
            logger.info(f"  λ{i+1} = {e.real:>8.4f} + {e.imag:>8.4f}j")
        
        logger.info(f"\nStability check: {'✓ STABLE' if stability['is_stable'] else '✗ UNSTABLE'}")
        logger.info(f"Max real eigenvalue: {stability['max_real']:.6f}")
        logger.info(f"Condition number: {stability['condition_number']:.2e}")
        
        if stability['is_stable']:
            result.success = True
            logger.info("✓ H∞ LMI design SUCCEEDED")
        else:
            result.error_message = f"Unstable (max_real={stability['max_real']:.4f})"
            logger.error(f"✗ H∞ LMI design produced unstable observer")
        
    except Exception as e:
        result.error_message = str(e)
        logger.error(f"✗ H∞ LMI design FAILED: {e}")
    
    return result


def test_l2_design_at_point(
    dynamics: QLPVVehicleDynamicsObs,
    vx: float,
    delta: float,
    gamma: float = 2.0,
    decay_rate: float = 0.5
) -> LMITestResult:
    """
    Test L2 LMI design at a specific operating point
    """
    result = LMITestResult(method='l2', vx=vx, delta=delta, success=False, gamma=gamma)
    
    logger.info("="*80)
    logger.info(f"L2 LMI Design Test at vx={vx:.2f} m/s, δ={np.rad2deg(delta):.1f}°")
    logger.info("="*80)
    
    # Build state at operating point
    state = np.array([vx, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(state, delta)
    
    # Compute system matrices
    A = dynamics.compute_A_matrix(rho)
    E = dynamics.compute_E_matrix(rho)
    C = np.eye(STATE_DIM)
    
    log_matrix("A(ρ) matrix", A)
    
    logger.info(f"\nAttempting L2 LMI design (γ={gamma}, α={decay_rate})...")
    
    try:
        L = compute_l2_lmi_observer_gain(A, C, E, gamma=gamma, decay_rate=decay_rate, verbose=True)
        
        result.L = L
        log_matrix("Observer Gain L (L2)", L)
        
        # Analyze stability
        stability = analyze_stability(A, C, L)
        result.eigenvalues = stability['eigenvalues']
        result.max_real_eig = stability['max_real']
        result.condition_number = stability['condition_number']
        
        logger.info(f"\nClosed-loop eigenvalues (A - L·C):")
        for i, e in enumerate(stability['eigenvalues']):
            logger.info(f"  λ{i+1} = {e.real:>8.4f} + {e.imag:>8.4f}j")
        
        logger.info(f"\nStability check: {'✓ STABLE' if stability['is_stable'] else '✗ UNSTABLE'}")
        logger.info(f"Max real eigenvalue: {stability['max_real']:.6f}")
        
        if stability['is_stable']:
            result.success = True
            logger.info("✓ L2 LMI design SUCCEEDED")
        else:
            result.error_message = f"Unstable (max_real={stability['max_real']:.4f})"
            logger.error(f"✗ L2 LMI design produced unstable observer")
        
    except Exception as e:
        result.error_message = str(e)
        logger.error(f"✗ L2 LMI design FAILED: {e}")
    
    return result


def test_polytope_vertices(
    dynamics: QLPVVehicleDynamicsObs,
    vx_range: Tuple[float, float] = (0.5, 3.0),
    delta_range: Tuple[float, float] = (-0.4, 0.4),
    method: str = 'hinf',
    gamma: float = 2.0
) -> List[LMITestResult]:
    """
    Test LMI design at polytope vertices (corner cases)
    
    Args:
        dynamics: qLPV dynamics model
        vx_range: (vx_min, vx_max) in m/s
        delta_range: (delta_min, delta_max) in rad
        method: 'hinf' or 'l2'
        gamma: Performance bound
    
    Returns:
        List of LMITestResult for each vertex
    """
    logger.info("\n" + "="*80)
    logger.info(f"POLYTOPE VERTEX TEST ({method.upper()})")
    logger.info(f"vx ∈ [{vx_range[0]}, {vx_range[1]}] m/s")
    logger.info(f"δ  ∈ [{np.rad2deg(delta_range[0]):.1f}, {np.rad2deg(delta_range[1]):.1f}]°")
    logger.info("="*80 + "\n")
    
    # Generate all 4 vertices
    vertices = [
        (vx_range[0], delta_range[0]),  # Low speed, left turn
        (vx_range[0], delta_range[1]),  # Low speed, right turn
        (vx_range[1], delta_range[0]),  # High speed, left turn
        (vx_range[1], delta_range[1]),  # High speed, right turn
    ]
    
    # Add nominal point
    vx_nom = (vx_range[0] + vx_range[1]) / 2
    vertices.append((vx_nom, 0.0))  # Nominal: mid-speed, straight
    
    results = []
    
    for vx, delta in vertices:
        if method == 'hinf':
            result = test_hinf_design_at_point(dynamics, vx, delta, gamma=gamma)
        else:
            result = test_l2_design_at_point(dynamics, vx, delta, gamma=gamma)
        
        results.append(result)
    
    return results


def print_summary(results: List[LMITestResult]):
    """Print summary table of all test results"""
    logger.info("\n" + "="*80)
    logger.info("SUMMARY")
    logger.info("="*80)
    
    logger.info(f"\n{'Method':<8} {'vx [m/s]':>10} {'δ [deg]':>10} {'Success':>10} {'max(Re(λ))':>12} {'γ':>6}")
    logger.info("-"*60)
    
    for r in results:
        status = "✓" if r.success else "✗"
        delta_deg = np.rad2deg(r.delta)
        max_eig_str = f"{r.max_real_eig:.4f}" if r.max_real_eig != float('inf') else "N/A"
        logger.info(f"{r.method:<8} {r.vx:>10.2f} {delta_deg:>10.1f} {status:>10} {max_eig_str:>12} {r.gamma:>6.1f}")
    
    # Count successes
    n_success = sum(1 for r in results if r.success)
    n_total = len(results)
    logger.info("-"*60)
    logger.info(f"Success rate: {n_success}/{n_total} ({100*n_success/n_total:.1f}%)")
    
    # List failures
    failures = [r for r in results if not r.success]
    if failures:
        logger.info("\nFailed cases:")
        for r in failures:
            logger.info(f"  - {r.method} at vx={r.vx:.2f}, δ={np.rad2deg(r.delta):.1f}°: {r.error_message}")


# =============================================================================
# Main Test
# =============================================================================

def main():
    """Main test function"""
    logger.info("\n" + "="*80)
    logger.info("LMI OBSERVER GAIN DESIGN TEST")
    logger.info("="*80)
    
    if not CVXPY_AVAILABLE:
        logger.error("CVXPY not available! Cannot run LMI tests.")
        return 1
    
    # Load vehicle parameters
    params = get_vehicle_params()
    logger.info("\nVehicle parameters:")
    for key, val in params.items():
        logger.info(f"  {key}: {val}")
    
    # Create dynamics model
    dynamics = QLPVVehicleDynamicsObs(params)
    
    all_results = []
    
    # ========== Test 1: Single point at nominal conditions ==========
    logger.info("\n\n" + "#"*80)
    logger.info("# TEST 1: NOMINAL OPERATING POINT")
    logger.info("#"*80)
    
    # Nominal: 1.5 m/s, straight
    result_hinf_nom = test_hinf_design_at_point(dynamics, vx=1.5, delta=0.0, gamma=2.0)
    all_results.append(result_hinf_nom)
    
    result_l2_nom = test_l2_design_at_point(dynamics, vx=1.5, delta=0.0, gamma=2.0)
    all_results.append(result_l2_nom)
    
    # ========== Test 2: Relaxed gamma ==========
    logger.info("\n\n" + "#"*80)
    logger.info("# TEST 2: RELAXED GAMMA (γ=5.0)")
    logger.info("#"*80)
    
    result_hinf_relaxed = test_hinf_design_at_point(dynamics, vx=1.5, delta=0.0, gamma=5.0)
    all_results.append(result_hinf_relaxed)
    
    result_l2_relaxed = test_l2_design_at_point(dynamics, vx=1.5, delta=0.0, gamma=5.0)
    all_results.append(result_l2_relaxed)
    
    # ========== Test 3: Polytope vertices (H∞) ==========
    logger.info("\n\n" + "#"*80)
    logger.info("# TEST 3: POLYTOPE VERTICES (H∞)")
    logger.info("#"*80)
    
    vertex_results_hinf = test_polytope_vertices(
        dynamics, 
        vx_range=(0.5, 3.0),
        delta_range=(-0.4, 0.4),
        method='hinf',
        gamma=5.0  # Use relaxed gamma
    )
    all_results.extend(vertex_results_hinf)
    
    # ========== Test 4: Polytope vertices (L2) ==========
    logger.info("\n\n" + "#"*80)
    logger.info("# TEST 4: POLYTOPE VERTICES (L2)")
    logger.info("#"*80)
    
    vertex_results_l2 = test_polytope_vertices(
        dynamics, 
        vx_range=(0.5, 3.0),
        delta_range=(-0.4, 0.4),
        method='l2',
        gamma=5.0
    )
    all_results.extend(vertex_results_l2)
    
    # ========== Summary ==========
    print_summary(all_results)
    
    return 0 if all(r.success for r in all_results) else 1


if __name__ == '__main__':
    exit(main())
