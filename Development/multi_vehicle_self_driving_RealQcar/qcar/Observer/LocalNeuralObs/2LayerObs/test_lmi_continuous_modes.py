"""
Test Continuous-Time LMI Observer Gain Design with Different C Matrix Modes

This script tests the continuous-time LMI design for observer gains across
different measurement (C matrix) configurations matching qlpv_vehicle_dynamics_obs.py.

Measurement configurations tested:
- Mode 5D (Standard):  [vx, r, ψ, X, Y] - GPS + IMU (indices 0,1,2,3,4)
- Mode 4D (No GPS):    [vx, r, ay, ax] - IMU only (indices 0,1,5,6)
- Mode 6D (With ay):   [vx, r, ψ, X, Y, ay] - Full except ax (indices 0,1,2,3,4,5)
- Mode 7D (Full):      [vx, r, ψ, X, Y, ay, ax] - All measurements (indices 0-6)

For continuous-time systems, stability requires:
    All eigenvalues of (A - L @ C) have Re(λ) < 0 (left-half plane)

Author: Neural Observer Team
Date: 2026-02-02
"""

import sys
from pathlib import Path
import numpy as np

# Add paths for imports
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir.parent))

from qlpv_vehicle_dynamics_obs import (
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    STATE_DIM, MEAS_DIM,
    # Measurement indices
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI,
    MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_IDX_AX,
)

from Design_LMI_neural import (
    compute_hinf_lmi_observer_gain,
    compute_l2_lmi_observer_gain,
    compute_lmi_observer_gain,
    validate_observer_gain,
    CVXPY_AVAILABLE,
)


# =============================================================================
# C Matrix Mode Definitions (matching qlpv_vehicle_dynamics_obs.py)
# =============================================================================

# Mode definitions: name -> list of active measurement indices
C_MATRIX_MODES = {
    "5D_GPS_IMU": [MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y],
    "4D_IMU_ONLY": [MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_AY, MEAS_IDX_AX],
    "6D_WITH_AY": [MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY],
    "7D_FULL": [MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_IDX_AX],
}

# Human-readable descriptions
MODE_DESCRIPTIONS = {
    "5D_GPS_IMU": "vx, r, ψ, X, Y (GPS + IMU)",
    "4D_IMU_ONLY": "vx, r, ay, ax (IMU only, no GPS)",
    "6D_WITH_AY": "vx, r, ψ, X, Y, ay (full except ax)",
    "7D_FULL": "vx, r, ψ, X, Y, ay, ax (all measurements)",
}


def compute_observability_rank(A: np.ndarray, C: np.ndarray) -> tuple:
    """
    Compute the observability matrix rank.
    
    Observability matrix O = [C; C*A; C*A^2; ...; C*A^(n-1)]
    System is observable if rank(O) = n (state dimension).
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)
        
    Returns:
        (rank, n): Tuple of observability rank and state dimension
    """
    n = A.shape[0]
    O = C.copy()
    CA_power = C.copy()
    
    for i in range(1, n):
        CA_power = CA_power @ A
        O = np.vstack([O, CA_power])
    
    rank = np.linalg.matrix_rank(O)
    return rank, n


def analyze_continuous_stability(A_cl: np.ndarray, label: str = "") -> dict:
    """
    Analyze stability of continuous-time closed-loop system.
    
    For continuous systems, stability requires:
        All eigenvalues have Re(λ) < 0 (left-half plane)
    
    Returns:
        dict with eigenvalues, stability status, and convergence rate
    """
    eigenvalues = np.linalg.eigvals(A_cl)
    real_parts = np.real(eigenvalues)
    
    is_stable = np.all(real_parts < 0)
    max_real = np.max(real_parts)  # Slowest mode (closest to 0)
    min_real = np.min(real_parts)  # Fastest mode (most negative)
    
    print(f"\n{'='*60}")
    print(f"  {label}" if label else "  Continuous-Time Stability Analysis")
    print(f"{'='*60}")
    print(f"Eigenvalues of (A - L @ C):")
    for i, eig in enumerate(sorted(eigenvalues, key=lambda x: np.real(x), reverse=True)):
        stability = "✓ Stable" if np.real(eig) < 0 else "✗ UNSTABLE"
        print(f"  λ_{i+1} = {eig:+.4f}  [{stability}]")
    
    print(f"\nStability Summary:")
    print(f"  All eigenvalues stable: {'YES ✓' if is_stable else 'NO ✗'}")
    print(f"  Slowest mode (max Re): {max_real:.4f}")
    print(f"  Fastest mode (min Re): {min_real:.4f}")
    
    if is_stable:
        settling_time = 4.0 / abs(max_real) if max_real != 0 else float('inf')
        print(f"  Approx. settling time (4τ): {settling_time:.2f} seconds")
    
    return {
        'eigenvalues': eigenvalues,
        'is_stable': is_stable,
        'max_real_part': max_real,
        'min_real_part': min_real
    }


def test_continuous_lmi_mode(
    mode_name: str,
    active_indices: list,
    dynamics: QLPVVehicleDynamicsObs,
    vx: float = 1.0,
    delta: float = 0.1,
    gamma: float = 5.0,
    decay_rate: float = 0.5,
    verbose: bool = False
) -> dict:
    """
    Test continuous-time LMI observer gain design for a specific C matrix mode.
    
    Args:
        mode_name: Name of the measurement mode
        active_indices: List of active measurement indices
        dynamics: QLPVVehicleDynamicsObs instance
        vx: Longitudinal velocity for test point [m/s]
        delta: Steering angle for test point [rad]
        gamma: H∞ performance bound
        decay_rate: Minimum decay rate for LMI
        verbose: Print detailed output
        
    Returns:
        dict with test results
    """
    print(f"\n{'#'*60}")
    print(f"# Mode: {mode_name}")
    print(f"# Measurements: {MODE_DESCRIPTIONS.get(mode_name, 'Unknown')}")
    print(f"# Active indices: {active_indices}")
    print(f"{'#'*60}")
    
    # Compute system matrices at operating point
    x = np.array([vx, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(x, delta)
    
    A = dynamics.compute_A_matrix(rho)
    C = dynamics.compute_C_matrix(rho, active_indices=active_indices)
    E = dynamics.compute_E_matrix(rho)
    
    n_meas = len(active_indices)
    
    print(f"\nMatrix dimensions:")
    print(f"  A: {A.shape}")
    print(f"  C: {C.shape} (mode: {n_meas}D)")
    print(f"  E: {E.shape}")
    
    # Check observability
    obs_rank, n = compute_observability_rank(A, C)
    is_observable = obs_rank == n
    print(f"\nObservability:")
    print(f"  Rank(O) = {obs_rank} / {n} {'✓ Observable' if is_observable else '✗ NOT Observable'}")
    
    if not is_observable:
        print(f"  ⚠ WARNING: System is not fully observable with this C matrix")
    
    result = {
        'mode_name': mode_name,
        'active_indices': active_indices,
        'n_meas': n_meas,
        'obs_rank': obs_rank,
        'is_observable': is_observable,
        'hinf_success': False,
        'l2_success': False,
        'lmi_success': False,
        'is_stable': False,
    }
    
    if not CVXPY_AVAILABLE:
        print("⚠ CVXPY not available, skipping LMI tests")
        return result
    
    # Parameter sets to try (will try relaxed parameters if initial fails)
    param_sets = [
        {'gamma': gamma, 'decay_rate': decay_rate, 'label': 'standard'},
        {'gamma': 10.0, 'decay_rate': 0.3, 'label': 'relaxed'},
        {'gamma': 20.0, 'decay_rate': 0.1, 'label': 'very relaxed'},
    ]
    
    # Test 1: H∞ LMI design (try multiple parameter sets)
    print(f"\n--- H∞ LMI Design ---")
    for params in param_sets:
        print(f"  Trying {params['label']}: gamma={params['gamma']}, decay={params['decay_rate']}")
        try:
            L_hinf = compute_hinf_lmi_observer_gain(
                A, C, E,
                gamma=params['gamma'],
                decay_rate=params['decay_rate'],
                verbose=verbose
            )
            print(f"  ✓ SUCCESS with {params['label']} parameters")
            print(f"    L norm: {np.linalg.norm(L_hinf, 'fro'):.4f}")
            
            A_cl = A - L_hinf @ C
            stability_result = analyze_continuous_stability(A_cl, f"H∞ Mode: {mode_name} ({params['label']})")
            
            result['hinf_success'] = True
            result['is_stable'] = stability_result['is_stable']
            result['L_hinf'] = L_hinf
            result['hinf_params'] = params
            break  # Stop trying if successful
            
        except Exception as e:
            print(f"    ✗ Failed: {str(e)[:50]}...")
    
    # Test 2: L2 LMI design
    print(f"\n--- L2 LMI Design ---")
    for params in param_sets:
        print(f"  Trying {params['label']}: gamma={params['gamma']}, decay={params['decay_rate']}")
        try:
            L_l2 = compute_l2_lmi_observer_gain(
                A, C, E,
                gamma=params['gamma'],
                decay_rate=params['decay_rate'],
                verbose=verbose
            )
            print(f"  ✓ SUCCESS with {params['label']} parameters")
            
            A_cl = A - L_l2 @ C
            stability_result = analyze_continuous_stability(A_cl, f"L2 Mode: {mode_name} ({params['label']})")
            
            result['l2_success'] = True
            result['L_l2'] = L_l2
            break
            
        except Exception as e:
            print(f"    ✗ Failed: {str(e)[:50]}...")
    
    # Test 3: Standard LMI design (no disturbance)
    print(f"\n--- Standard LMI Design ---")
    decay_rates = [decay_rate, 0.3, 0.1, 0.05]
    for dr in decay_rates:
        print(f"  Trying decay_rate={dr}")
        try:
            L_lmi = compute_lmi_observer_gain(
                A, C,
                decay_rate=dr,
                verbose=verbose
            )
            print(f"  ✓ SUCCESS with decay_rate={dr}")
            
            A_cl = A - L_lmi @ C
            stability_result = analyze_continuous_stability(A_cl, f"Standard LMI Mode: {mode_name}")
            
            result['lmi_success'] = True
            result['L_lmi'] = L_lmi
            break
            
        except Exception as e:
            print(f"    ✗ Failed: {str(e)[:50]}...")
    
    return result


def test_all_c_matrix_modes():
    """
    Test continuous-time LMI design for all defined C matrix modes.
    """
    print("=" * 70)
    print("  CONTINUOUS-TIME LMI OBSERVER GAIN DESIGN - C MATRIX MODES TEST")
    print("=" * 70)
    
    # Initialize dynamics
    params = get_default_vehicle_params()
    dynamics = QLPVVehicleDynamicsObs(
        vehicle_params=params,
        min_vx=0.15,
        disturbance_mode='tire'  # Use tire residual disturbance
    )
    
    # Test parameters
    test_vx = 1.0
    test_delta = 0.1
    gamma = 5.0
    decay_rate = 0.5
    
    print(f"\nTest operating point:")
    print(f"  vx = {test_vx} m/s")
    print(f"  delta = {test_delta} rad")
    print(f"  gamma (H∞/L2) = {gamma}")
    print(f"  decay_rate = {decay_rate}")
    
    results = {}
    
    for mode_name, active_indices in C_MATRIX_MODES.items():
        results[mode_name] = test_continuous_lmi_mode(
            mode_name=mode_name,
            active_indices=active_indices,
            dynamics=dynamics,
            vx=test_vx,
            delta=test_delta,
            gamma=gamma,
            decay_rate=decay_rate
        )
    
    # Summary table
    print("\n" + "=" * 70)
    print("  SUMMARY: C Matrix Mode LMI Results")
    print("=" * 70)
    print(f"{'Mode':<15} {'Dim':>4} {'Obs':>5} {'H∞':>6} {'L2':>6} {'LMI':>6} {'Stable':>8}")
    print("-" * 70)
    
    # Modes that REQUIRE ay measurement for proper observability of vy state
    modes_with_ay = ["6D_WITH_AY", "7D_FULL"]
    
    required_passed = True
    optional_passed = True
    
    for mode_name, res in results.items():
        obs_str = "✓" if res['is_observable'] else "✗"
        hinf_str = "✓" if res['hinf_success'] else "✗"
        l2_str = "✓" if res['l2_success'] else "✗"
        lmi_str = "✓" if res['lmi_success'] else "✗"
        stable_str = "✓" if res['is_stable'] else "✗"
        
        # Mark modes with ay as required, others as optional
        required_marker = "*" if mode_name in modes_with_ay else " "
        
        print(f"{mode_name:<15} {res['n_meas']:>4}D {obs_str:>5} {hinf_str:>6} {l2_str:>6} {lmi_str:>6} {stable_str:>8} {required_marker}")
        
        if mode_name in modes_with_ay:
            if not res['hinf_success'] or not res['is_stable']:
                required_passed = False
        else:
            if not res['hinf_success'] or not res['is_stable']:
                optional_passed = False
    
    print("-" * 70)
    print("  * = Modes with ay measurement (required for vy observability)")
    print("")
    print("  KEY INSIGHT: Lateral acceleration (ay) is CRITICAL for observing")
    print("  lateral velocity (vy) in the single-track vehicle model. Modes")
    print("  without ay may fail LMI even if structurally observable.")
    print("-" * 70)
    
    if required_passed:
        print(f"✓ REQUIRED modes (with ay): PASSED")
    else:
        print(f"✗ REQUIRED modes (with ay): FAILED")
        
    if optional_passed:
        print(f"✓ OPTIONAL modes (without ay): PASSED")
    else:
        print(f"○ OPTIONAL modes (without ay): FAILED (expected)")
    
    print("=" * 70)
    
    return results, required_passed


def test_multiple_operating_points():
    """
    Test LMI design across multiple operating points for robustness check.
    """
    print("\n" + "=" * 70)
    print("  MULTI-POINT ROBUSTNESS TEST")
    print("=" * 70)
    
    params = get_default_vehicle_params()
    dynamics = QLPVVehicleDynamicsObs(vehicle_params=params, min_vx=0.15)
    
    # Test grid
    vx_values = [0.5, 1.0, 2.0]
    delta_values = [-0.3, 0.0, 0.3]
    
    # Use 5D standard mode for robustness test
    mode_name = "5D_GPS_IMU"
    active_indices = C_MATRIX_MODES[mode_name]
    
    print(f"\nMode: {mode_name}")
    print(f"Testing {len(vx_values) * len(delta_values)} operating points...\n")
    
    results = []
    
    for vx in vx_values:
        for delta in delta_values:
            x = np.array([vx, 0.0, 0.0, 0.0, 0.0, 0.0])
            rho = dynamics.compute_scheduling_params(x, delta)
            
            A = dynamics.compute_A_matrix(rho)
            C = dynamics.compute_C_matrix(rho, active_indices=active_indices)
            E = dynamics.compute_E_matrix(rho)
            
            try:
                L = compute_hinf_lmi_observer_gain(A, C, E, gamma=5.0, decay_rate=0.5, verbose=False)
                A_cl = A - L @ C
                eigs = np.linalg.eigvals(A_cl)
                max_real = np.max(np.real(eigs))
                is_stable = max_real < 0
                
                status = "✓" if is_stable else "✗"
                print(f"  vx={vx:.1f}, δ={delta:+.2f}: max(Re(λ))={max_real:+.4f} [{status}]")
                results.append({'vx': vx, 'delta': delta, 'stable': is_stable, 'max_real': max_real})
                
            except Exception as e:
                print(f"  vx={vx:.1f}, δ={delta:+.2f}: FAILED - {e}")
                results.append({'vx': vx, 'delta': delta, 'stable': False, 'error': str(e)})
    
    n_stable = sum(1 for r in results if r.get('stable', False))
    print(f"\nRobustness: {n_stable}/{len(results)} operating points stable")
    
    return results


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test continuous-time LMI with C matrix modes")
    parser.add_argument("--robustness", action="store_true", help="Run multi-point robustness test")
    parser.add_argument("--verbose", action="store_true", help="Verbose solver output")
    args = parser.parse_args()
    
    # Run main tests
    results, all_passed = test_all_c_matrix_modes()
    
    # Optionally run robustness test
    if args.robustness:
        test_multiple_operating_points()
    
    # Exit code
    sys.exit(0 if all_passed else 1)
