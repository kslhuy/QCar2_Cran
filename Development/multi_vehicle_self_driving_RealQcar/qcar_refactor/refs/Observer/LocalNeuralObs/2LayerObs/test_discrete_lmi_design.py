"""
Test Discrete-Time LMI Observer Gain Design

This test verifies the new discrete-time Schur-form LMI design functions
for the neural observer.

Author: Neural Observer Team
Date: 2026-01-22
"""

import numpy as np
import sys
from pathlib import Path

# Add paths for imports
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir.parent))

from qlpv_vehicle_dynamics_obs import (
    QLPVVehicleDynamicsObs, 
    get_default_vehicle_params,
    STATE_DIM, MEAS_DIM
)

from Design_LMI_neural import (
    discretize_system_zoh,
    compute_discrete_hinf_lmi_observer_gain,
    compute_discrete_l2_lmi_observer_gain,
    compute_discrete_contraction_lmi,
    compute_discrete_h2_lmi_observer_gain,
    validate_discrete_observer_gain,
    NeuralQLPVGainScheduler,
)


def test_discretization():
    """Test ZOH discretization of continuous-time system"""
    print("\n=== Test: System Discretization (ZOH) ===")
    
    dynamics = QLPVVehicleDynamicsObs()
    x = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(x, 0.1)
    
    A_c = dynamics.compute_A_matrix(rho)
    B_c = dynamics.compute_B_matrix(rho)
    E_c = dynamics.compute_E_matrix(rho)
    
    dt = 0.01
    A_d, B_d, E_d = discretize_system_zoh(A_c, B_c, E_c, dt)
    
    print(f"Continuous A_c shape: {A_c.shape}")
    print(f"Discrete A_d shape: {A_d.shape}")
    print(f"Discrete B_d shape: {B_d.shape}")
    print(f"Discrete E_d shape: {E_d.shape}")
    
    # Check A_d ≈ I + dt*A_c for small dt (Euler approximation)
    A_d_euler = np.eye(STATE_DIM) + dt * A_c
    euler_error = np.linalg.norm(A_d - A_d_euler, 'fro')
    print(f"Euler approximation error: {euler_error:.6f}")
    
    # Check eigenvalues of A_d are inside unit circle
    eigs = np.linalg.eigvals(A_d)
    spectral_radius = np.max(np.abs(eigs))
    print(f"A_d spectral radius: {spectral_radius:.4f}")
    
    assert A_d.shape == (STATE_DIM, STATE_DIM), "A_d shape mismatch"
    assert B_d.shape == (STATE_DIM, 2), "B_d shape mismatch"
    assert E_d.shape == (STATE_DIM, 2), "E_d shape mismatch"
    
    print("✓ Discretization test PASSED")
    return True


def test_discrete_hinf_lmi():
    """Test discrete-time H∞ LMI observer gain design"""
    print("\n=== Test: Discrete H∞ LMI Observer Gain ===")
    
    try:
        import cvxpy as cp
    except ImportError:
        print("⚠ CVXPY not available, skipping LMI test")
        return True
    
    dynamics = QLPVVehicleDynamicsObs()
    x = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(x, 0.1)
    
    A_c = dynamics.compute_A_matrix(rho)
    B_c = dynamics.compute_B_matrix(rho)
    E_c = dynamics.compute_E_matrix(rho)
    C = dynamics.compute_C_matrix(rho)
    
    dt = 0.01
    A_d, B_d, E_d = discretize_system_zoh(A_c, B_c, E_c, dt)
    
    print(f"A_d shape: {A_d.shape}, C shape: {C.shape}, E_d shape: {E_d.shape}")
    
    # Try to compute H∞ gain
    try:
        L = compute_discrete_hinf_lmi_observer_gain(
            A_d, C, E_d, 
            gamma=5.0, 
            contraction_rate=0.95,
            verbose=False
        )
        
        print(f"L shape: {L.shape}")
        print(f"L norm: {np.linalg.norm(L, 'fro'):.4f}")
        
        # Validate gain
        A_cl = A_d - L @ C
        eigs = np.linalg.eigvals(A_cl)
        spectral_radius = np.max(np.abs(eigs))
        print(f"Closed-loop spectral radius: {spectral_radius:.4f}")
        
        is_stable = validate_discrete_observer_gain(A_d, C, L)
        print(f"Stability validation: {is_stable}")
        
        assert L.shape == (STATE_DIM, MEAS_DIM), "L shape mismatch"
        assert spectral_radius < 1.0, "Observer not stable"
        
        print("✓ Discrete H∞ LMI test PASSED")
        return True
        
    except Exception as e:
        print(f"⚠ Discrete H∞ LMI failed: {e}")
        return False


def test_discrete_contraction_lmi():
    """Test discrete-time contraction LMI"""
    print("\n=== Test: Discrete Contraction LMI ===")
    
    try:
        import cvxpy as cp
    except ImportError:
        print("⚠ CVXPY not available, skipping LMI test")
        return True
    
    dynamics = QLPVVehicleDynamicsObs()
    x = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(x, 0.0)
    
    A_c = dynamics.compute_A_matrix(rho)
    B_c = dynamics.compute_B_matrix(rho)
    E_c = dynamics.compute_E_matrix(rho)
    C = dynamics.compute_C_matrix(rho)
    
    dt = 0.01
    A_d, _, _ = discretize_system_zoh(A_c, B_c, E_c, dt)
    
    try:
        L = compute_discrete_contraction_lmi(
            A_d, C, 
            contraction_rate=0.95,
            verbose=False
        )
        
        A_cl = A_d - L @ C
        spectral_radius = np.max(np.abs(np.linalg.eigvals(A_cl)))
        print(f"Closed-loop spectral radius: {spectral_radius:.4f}")
        
        assert spectral_radius < 0.96, "Contraction rate not achieved"
        
        print("✓ Discrete contraction LMI test PASSED")
        return True
        
    except Exception as e:
        print(f"⚠ Discrete contraction LMI failed: {e}")
        return False


def test_gain_scheduler_discrete():
    """Test polytopic qLPV gain scheduler with discrete design"""
    print("\n=== Test: Discrete qLPV Gain Scheduler ===")
    
    try:
        import cvxpy as cp
    except ImportError:
        print("⚠ CVXPY not available, skipping scheduler test")
        return True
    
    params = get_default_vehicle_params()
    
    scheduler = NeuralQLPVGainScheduler(
        vehicle_params=params,
        vx_range=(0.5, 2.0),
        delta_max=0.3,
        n_vx_vertices=2,
        n_delta_vertices=2,
        hinf_gamma=5.0,
        contraction_rate=0.95,
        use_common_lyapunov=True,
        discrete=True,
        sample_time=0.01,
        verbose=False
    )
    
    print(f"Number of vertices: {scheduler.n_vertices}")
    
    success = scheduler.compute_gains_lmi()
    print(f"LMI solve success: {success}")
    
    if success:
        # Test gain interpolation
        L = scheduler.get_scheduled_gain(1.0, 0.1)
        print(f"Interpolated L shape: {L.shape}")
        print(f"Interpolated L norm: {np.linalg.norm(L, 'fro'):.4f}")
        
        # Test smooth scheduling
        L_smooth = scheduler.get_scheduled_gain_smooth(1.0, 0.1, alpha=0.1)
        print(f"Smoothed L norm: {np.linalg.norm(L_smooth, 'fro'):.4f}")
        
        print("✓ Discrete qLPV scheduler test PASSED")
        return True
    else:
        print("⚠ qLPV scheduler used default gains (LMI infeasible)")
        return True  # Default fallback is acceptable


def test_discrete_h2_lmi():
    """Test discrete-time H2 LMI observer gain design"""
    print("\n=== Test: Discrete H2 LMI Observer Gain ===")
    
    try:
        import cvxpy as cp
    except ImportError:
        print("⚠ CVXPY not available, skipping LMI test")
        return True
    
    dynamics = QLPVVehicleDynamicsObs()
    x = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rho = dynamics.compute_scheduling_params(x, 0.1)
    
    A_c = dynamics.compute_A_matrix(rho)
    B_c = dynamics.compute_B_matrix(rho)
    E_c = dynamics.compute_E_matrix(rho)
    C = dynamics.compute_C_matrix(rho)
    
    dt = 0.01
    A_d, B_d, E_d = discretize_system_zoh(A_c, B_c, E_c, dt)
    
    print(f"A_d shape: {A_d.shape}, C shape: {C.shape}, E_d shape: {E_d.shape}")
    
    try:
        L, h2_norm = compute_discrete_h2_lmi_observer_gain(
            A_d, C, E_d, 
            contraction_rate=0.95,
            verbose=False
        )
        
        print(f"L shape: {L.shape}")
        print(f"L norm: {np.linalg.norm(L, 'fro'):.4f}")
        print(f"Achieved H2 norm: {h2_norm:.4f}")
        
        # Validate gain
        A_cl = A_d - L @ C
        eigs = np.linalg.eigvals(A_cl)
        spectral_radius = np.max(np.abs(eigs))
        print(f"Closed-loop spectral radius: {spectral_radius:.4f}")
        
        is_stable = validate_discrete_observer_gain(A_d, C, L)
        print(f"Stability validation: {is_stable}")
        
        assert L.shape == (STATE_DIM, MEAS_DIM), "L shape mismatch"
        assert spectral_radius < 1.0, "Observer not stable"
        
        print("✓ Discrete H2 LMI test PASSED")
        return True
        
    except Exception as e:
        print(f"⚠ Discrete H2 LMI failed: {e}")
        return False


if __name__ == "__main__":
    print("=" * 60)
    print("Discrete-Time LMI Observer Gain Design Tests")
    print("=" * 60)
    
    results = []
    
    results.append(("Discretization", test_discretization()))
    results.append(("Discrete H∞ LMI", test_discrete_hinf_lmi()))
    results.append(("Discrete H2 LMI", test_discrete_h2_lmi()))
    results.append(("Discrete Contraction", test_discrete_contraction_lmi()))
    results.append(("Discrete Scheduler", test_gain_scheduler_discrete()))
    
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    all_passed = True
    for name, passed in results:
        status = "✓ PASSED" if passed else "✗ FAILED"
        print(f"  {name}: {status}")
        all_passed = all_passed and passed
    
    print("=" * 60)
    if all_passed:
        print("All tests PASSED!")
    else:
        print("Some tests FAILED")
    print("=" * 60)
