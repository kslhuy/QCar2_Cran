"""
Test Discrete-Time LMI Observer Gain Design with Different C Matrix Modes

This script tests the discrete-time LMI design for observer gains across
different measurement (C matrix) configurations matching qlpv_vehicle_dynamics_obs.py.

Measurement configurations tested:
- Mode 5D (Standard):  [vx, r, ψ, X, Y] - GPS + IMU (indices 0,1,2,3,4)
- Mode 4D (No GPS):    [vx, r, ay, ax] - IMU only (indices 0,1,5,6)
- Mode 6D (With ay):   [vx, r, ψ, X, Y, ay] - Full except ax (indices 0,1,2,3,4,5)
- Mode 7D (Full):      [vx, r, ψ, X, Y, ay, ax] - All measurements (indices 0-6)

For discrete-time systems, stability requires:
    All eigenvalues of (A_d - L @ C) have |λ| < 1 (inside unit circle)

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
    compute_discrete_hinf_lmi_observer_gain,
    compute_discrete_l2_lmi_observer_gain,
    compute_discrete_contraction_lmi,
    discretize_system_zoh,
    validate_discrete_observer_gain,
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


def analyze_discrete_stability(A_cl: np.ndarray, label: str = "") -> dict:
    """
    Analyze stability of discrete-time closed-loop system.
    
    For discrete systems, stability requires:
        All eigenvalues have |λ| < 1 (inside unit circle)
    
    Returns:
        dict with eigenvalues, stability status, and spectral radius
    """
    eigenvalues = np.linalg.eigvals(A_cl)
    magnitudes = np.abs(eigenvalues)
    
    spectral_radius = np.max(magnitudes)
    is_stable = spectral_radius < 1.0
    min_magnitude = np.min(magnitudes)
    
    print(f"\n{'='*60}")
    print(f"  {label}" if label else "  Discrete-Time Stability Analysis")
    print(f"{'='*60}")
    print(f"Eigenvalues of (A_d - L @ C):")
    for i, eig in enumerate(sorted(eigenvalues, key=lambda x: np.abs(x), reverse=True)):
        stability = "✓ Stable" if np.abs(eig) < 1 else "✗ UNSTABLE"
        print(f"  λ_{i+1} = {eig:+.4f}  |λ| = {np.abs(eig):.4f}  [{stability}]")
    
    print(f"\nStability Summary:")
    print(f"  All eigenvalues inside unit circle: {'YES ✓' if is_stable else 'NO ✗'}")
    print(f"  Spectral radius (max |λ|): {spectral_radius:.6f}")
    print(f"  Fastest mode (min |λ|): {min_magnitude:.6f}")
    
    if is_stable:
        # Approximate convergence rate (number of steps to decay by 1/e)
        if spectral_radius > 0:
            decay_steps = -1 / np.log(spectral_radius) if spectral_radius < 1 else float('inf')
            print(f"  Approx. decay constant: {decay_steps:.2f} steps")
    
    return {
        'eigenvalues': eigenvalues,
        'is_stable': is_stable,
        'spectral_radius': spectral_radius,
        'min_magnitude': min_magnitude
    }


def test_discrete_lmi_mode(
    mode_name: str,
    active_indices: list,
    dynamics: QLPVVehicleDynamicsObs,
    dt: float = 0.02,
    vx: float = 1.0,
    delta: float = 0.1,
    gamma: float = 5.0,
    contraction_rate: float = 0.95,
    verbose: bool = False
) -> dict:
    """
    Test discrete-time LMI observer gain design for a specific C matrix mode.
    
    Args:
        mode_name: Name of the measurement mode
        active_indices: List of active measurement indices
        dynamics: QLPVVehicleDynamicsObs instance
        dt: Sample time [seconds]
        vx: Longitudinal velocity for test point [m/s]
        delta: Steering angle for test point [rad]
        gamma: H∞ performance bound
        contraction_rate: Contraction rate λ ∈ (0, 1)
        verbose: Print detailed output
        
    Returns:
        dict with test results
    """
    print(f"\n{'#'*60}")
    print(f"# Mode: {mode_name}")
    print(f"# Measurements: {MODE_DESCRIPTIONS.get(mode_name, 'Unknown')}")
    print(f"# Active indices: {active_indices}")
    print(f"# Sample time: {dt} s ({1/dt:.0f} Hz)")
    print(f"{'#'*60}")
    
    # Compute continuous-time system matrices at operating point
    x = np.array([vx, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(x, delta)
    
    A_c = dynamics.compute_A_matrix(rho)
    B_c = dynamics.compute_B_matrix(rho)
    C = dynamics.compute_C_matrix(rho, active_indices=active_indices)
    E_c = dynamics.compute_E_matrix(rho)
    
    # Discretize the system
    A_d, B_d, E_d = discretize_system_zoh(A_c, B_c, E_c, dt)
    
    n_meas = len(active_indices)
    
    print(f"\nMatrix dimensions:")
    print(f"  A_c: {A_c.shape} → A_d: {A_d.shape}")
    print(f"  C: {C.shape} (mode: {n_meas}D)")
    print(f"  E_c: {E_c.shape} → E_d: {E_d.shape}")
    
    # Check observability of discrete system
    obs_rank, n = compute_observability_rank(A_d, C)
    is_observable = obs_rank == n
    print(f"\nObservability (discrete):")
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
        'contraction_success': False,
        'is_stable': False,
        'dt': dt,
    }
    
    if not CVXPY_AVAILABLE:
        print("⚠ CVXPY not available, skipping LMI tests")
        return result
    
    # Parameter sets to try (will try relaxed parameters if initial fails)
    param_sets = [
        {'gamma': gamma, 'contraction_rate': contraction_rate, 'label': 'standard'},
        {'gamma': 10.0, 'contraction_rate': 0.98, 'label': 'relaxed'},
        {'gamma': 20.0, 'contraction_rate': 0.99, 'label': 'very relaxed'},
    ]
    
    # Test 1: H∞ LMI design (try multiple parameter sets)
    print(f"\n--- Discrete H∞ LMI Design ---")
    for params in param_sets:
        print(f"  Trying {params['label']}: gamma={params['gamma']}, λ={params['contraction_rate']}")
        try:
            L_hinf = compute_discrete_hinf_lmi_observer_gain(
                A_d, C, E_d,
                gamma=params['gamma'],
                contraction_rate=params['contraction_rate'],
                verbose=verbose
            )
            print(f"  ✓ SUCCESS with {params['label']} parameters")
            print(f"    L norm: {np.linalg.norm(L_hinf, 'fro'):.4f}")
            
            A_cl = A_d - L_hinf @ C
            stability_result = analyze_discrete_stability(A_cl, f"Discrete H∞ Mode: {mode_name} ({params['label']})")
            
            result['hinf_success'] = True
            result['is_stable'] = stability_result['is_stable']
            result['L_hinf'] = L_hinf
            result['hinf_params'] = params
            result['spectral_radius'] = stability_result['spectral_radius']
            break  # Stop trying if successful
            
        except Exception as e:
            print(f"    ✗ Failed: {str(e)[:60]}...")
    
    # Test 2: L2 LMI design
    print(f"\n--- Discrete L2 LMI Design ---")
    for params in param_sets:
        print(f"  Trying {params['label']}: gamma={params['gamma']}, λ={params['contraction_rate']}")
        try:
            L_l2 = compute_discrete_l2_lmi_observer_gain(
                A_d, C, E_d,
                gamma=params['gamma'],
                contraction_rate=params['contraction_rate'],
                verbose=verbose
            )
            print(f"  ✓ SUCCESS with {params['label']} parameters")
            
            A_cl = A_d - L_l2 @ C
            stability_result = analyze_discrete_stability(A_cl, f"Discrete L2 Mode: {mode_name} ({params['label']})")
            
            result['l2_success'] = True
            result['L_l2'] = L_l2
            break
            
        except Exception as e:
            print(f"    ✗ Failed: {str(e)[:60]}...")
    
    # Test 3: Contraction LMI design (no disturbance)
    print(f"\n--- Discrete Contraction LMI Design ---")
    contraction_rates = [contraction_rate, 0.98, 0.99, 0.999]
    for cr in contraction_rates:
        print(f"  Trying contraction_rate={cr}")
        try:
            L_contraction = compute_discrete_contraction_lmi(
                A_d, C,
                contraction_rate=cr,
                verbose=verbose
            )
            print(f"  ✓ SUCCESS with contraction_rate={cr}")
            
            A_cl = A_d - L_contraction @ C
            stability_result = analyze_discrete_stability(A_cl, f"Discrete Contraction Mode: {mode_name}")
            
            result['contraction_success'] = True
            result['L_contraction'] = L_contraction
            break
            
        except Exception as e:
            print(f"    ✗ Failed: {str(e)[:50]}...")
    
    return result


def test_all_c_matrix_modes(dt: float = 0.02):
    """
    Test discrete-time LMI design for all defined C matrix modes.
    
    Args:
        dt: Sample time [seconds]
    """
    print("=" * 70)
    print("  DISCRETE-TIME LMI OBSERVER GAIN DESIGN - C MATRIX MODES TEST")
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
    contraction_rate = 0.95
    
    print(f"\nTest operating point:")
    print(f"  vx = {test_vx} m/s")
    print(f"  delta = {test_delta} rad")
    print(f"  dt = {dt} s ({1/dt:.0f} Hz)")
    print(f"  gamma (H∞/L2) = {gamma}")
    print(f"  contraction_rate = {contraction_rate}")
    
    results = {}
    
    for mode_name, active_indices in C_MATRIX_MODES.items():
        results[mode_name] = test_discrete_lmi_mode(
            mode_name=mode_name,
            active_indices=active_indices,
            dynamics=dynamics,
            dt=dt,
            vx=test_vx,
            delta=test_delta,
            gamma=gamma,
            contraction_rate=contraction_rate
        )
    
    # Summary table
    print("\n" + "=" * 70)
    print("  SUMMARY: C Matrix Mode Discrete LMI Results")
    print("=" * 70)
    print(f"{'Mode':<15} {'Dim':>4} {'Obs':>5} {'H∞':>6} {'L2':>6} {'Contr':>6} {'Stable':>8} {'ρ':>8}")
    print("-" * 70)
    
    # Modes that REQUIRE ay measurement for proper observability of vy state
    modes_with_ay = ["6D_WITH_AY", "7D_FULL"]
    
    required_passed = True
    optional_passed = True
    
    for mode_name, res in results.items():
        obs_str = "✓" if res['is_observable'] else "✗"
        hinf_str = "✓" if res['hinf_success'] else "✗"
        l2_str = "✓" if res['l2_success'] else "✗"
        contr_str = "✓" if res['contraction_success'] else "✗"
        stable_str = "✓" if res['is_stable'] else "✗"
        rho_str = f"{res.get('spectral_radius', float('nan')):.4f}"
        
        # Mark modes with ay as required, others as optional
        required_marker = "*" if mode_name in modes_with_ay else " "
        
        print(f"{mode_name:<15} {res['n_meas']:>4}D {obs_str:>5} {hinf_str:>6} {l2_str:>6} {contr_str:>6} {stable_str:>8} {rho_str:>8} {required_marker}")
        
        if mode_name in modes_with_ay:
            if not res['hinf_success'] or not res['is_stable']:
                required_passed = False
        else:
            if not res['hinf_success'] or not res['is_stable']:
                optional_passed = False
    
    print("-" * 70)
    print("  * = Modes with ay measurement (required for vy observability)")
    print("  ρ = spectral radius (must be < 1 for stability)")
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


def test_multiple_operating_points(dt: float = 0.02):
    """
    Test discrete LMI design across multiple operating points for robustness check.
    
    Args:
        dt: Sample time [seconds]
    """
    print("\n" + "=" * 70)
    print("  MULTI-POINT ROBUSTNESS TEST (DISCRETE-TIME)")
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
    print(f"Sample time: {dt} s ({1/dt:.0f} Hz)")
    print(f"Testing {len(vx_values) * len(delta_values)} operating points...\n")
    
    results = []
    
    for vx in vx_values:
        for delta in delta_values:
            x = np.array([vx, 0.0, 0.0, 0.0, 0.0, 0.0])
            rho = dynamics.compute_scheduling_params(x, delta)
            
            A_c = dynamics.compute_A_matrix(rho)
            B_c = dynamics.compute_B_matrix(rho)
            C = dynamics.compute_C_matrix(rho, active_indices=active_indices)
            E_c = dynamics.compute_E_matrix(rho)
            
            # Discretize
            A_d, _, E_d = discretize_system_zoh(A_c, B_c, E_c, dt)
            
            try:
                L = compute_discrete_hinf_lmi_observer_gain(
                    A_d, C, E_d, gamma=5.0, contraction_rate=0.95, verbose=False
                )
                A_cl = A_d - L @ C
                eigs = np.linalg.eigvals(A_cl)
                spectral_radius = np.max(np.abs(eigs))
                is_stable = spectral_radius < 1.0
                
                status = "✓" if is_stable else "✗"
                print(f"  vx={vx:.1f}, δ={delta:+.2f}: ρ={spectral_radius:.6f} [{status}]")
                results.append({'vx': vx, 'delta': delta, 'stable': is_stable, 'spectral_radius': spectral_radius})
                
            except Exception as e:
                print(f"  vx={vx:.1f}, δ={delta:+.2f}: FAILED - {e}")
                results.append({'vx': vx, 'delta': delta, 'stable': False, 'error': str(e)})
    
    n_stable = sum(1 for r in results if r.get('stable', False))
    print(f"\nRobustness: {n_stable}/{len(results)} operating points stable")
    
    return results


def test_different_sample_rates():
    """
    Test discrete LMI design across different sample rates.
    """
    print("\n" + "=" * 70)
    print("  SAMPLE RATE SENSITIVITY TEST")
    print("=" * 70)
    
    params = get_default_vehicle_params()
    dynamics = QLPVVehicleDynamicsObs(vehicle_params=params, min_vx=0.15)
    
    # Sample rates to test
    sample_rates = [10, 20, 50, 100, 200]  # Hz
    
    # Use 5D standard mode
    mode_name = "5D_GPS_IMU"
    active_indices = C_MATRIX_MODES[mode_name]
    
    vx = 1.0
    delta = 0.1
    
    print(f"\nMode: {mode_name}")
    print(f"Operating point: vx={vx} m/s, δ={delta} rad")
    print(f"\nTesting {len(sample_rates)} sample rates...\n")
    
    results = []
    
    x = np.array([vx, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(x, delta)
    
    A_c = dynamics.compute_A_matrix(rho)
    B_c = dynamics.compute_B_matrix(rho)
    C = dynamics.compute_C_matrix(rho, active_indices=active_indices)
    E_c = dynamics.compute_E_matrix(rho)
    
    for freq in sample_rates:
        dt = 1.0 / freq
        
        # Discretize
        A_d, _, E_d = discretize_system_zoh(A_c, B_c, E_c, dt)
        
        try:
            L = compute_discrete_hinf_lmi_observer_gain(
                A_d, C, E_d, gamma=5.0, contraction_rate=0.95, verbose=False
            )
            A_cl = A_d - L @ C
            eigs = np.linalg.eigvals(A_cl)
            spectral_radius = np.max(np.abs(eigs))
            is_stable = spectral_radius < 1.0
            L_norm = np.linalg.norm(L, 'fro')
            
            status = "✓" if is_stable else "✗"
            print(f"  {freq:4d} Hz (dt={dt:.4f}s): ρ={spectral_radius:.6f}, ||L||={L_norm:.2f} [{status}]")
            results.append({
                'freq': freq, 'dt': dt, 'stable': is_stable, 
                'spectral_radius': spectral_radius, 'L_norm': L_norm
            })
            
        except Exception as e:
            print(f"  {freq:4d} Hz (dt={dt:.4f}s): FAILED - {e}")
            results.append({'freq': freq, 'dt': dt, 'stable': False, 'error': str(e)})
    
    n_stable = sum(1 for r in results if r.get('stable', False))
    print(f"\nStability: {n_stable}/{len(results)} sample rates successful")
    
    return results


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test discrete-time LMI with C matrix modes")
    parser.add_argument("--dt", type=float, default=0.02, help="Sample time in seconds (default: 0.02)")
    parser.add_argument("--robustness", action="store_true", help="Run multi-point robustness test")
    parser.add_argument("--sample-rates", action="store_true", help="Run sample rate sensitivity test")
    parser.add_argument("--verbose", action="store_true", help="Verbose solver output")
    args = parser.parse_args()
    
    # Run main tests
    results, all_passed = test_all_c_matrix_modes(dt=args.dt)
    
    # Optionally run robustness test
    if args.robustness:
        test_multiple_operating_points(dt=args.dt)
    
    # Optionally run sample rate test
    if args.sample_rates:
        test_different_sample_rates()
    
    # Exit code
    sys.exit(0 if all_passed else 1)
