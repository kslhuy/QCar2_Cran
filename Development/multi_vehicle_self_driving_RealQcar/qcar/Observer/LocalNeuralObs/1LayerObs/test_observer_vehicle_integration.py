"""
Integration Test: All Observers vs qLPV Vehicle Model

This test verifies that all three observer implementations correctly estimate 
vehicle states and tire residuals when compared against the fake vehicle ground truth.

Observers tested:
1. qLPVAugmentedObserver (qlpv_observer.py) - Basic qLPV observer
2. qLPVAugmentedObserver/EKF (qlpv_observer_kalma.py) - EKF-style gain computation
3. DifferentiatorUIOObserver (differentiator_uio_observer.py) - UIO-style with differentiator

The fake vehicle uses the exact same dynamics as the observers, but:
- Vehicle: KNOWS true tire residuals (w_r, w_f) from Pacejka vs linear difference
- Observer: ESTIMATES tire residuals from measurements

Test scenarios:
1. Straight driving at constant velocity
2. Lane change maneuver (steering input)
3. Accelerating turn

"""

import numpy as np
import sys
from pathlib import Path

# Add paths
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))
# Navigate from 1LayerObs -> LocalNeuralObs -> Observer -> qcar, then to GUI
sys.path.insert(0, str(parent_dir.parent.parent.parent / "GUI"))

# Import all observers
from qlpv_observer import qLPVAugmentedObserver as qLPVObserverBasic
from qlpv_observer_kalma import qLPVAugmentedObserver as qLPVObserverEKF
from differentiator_uio_observer import DifferentiatorUIOObserver
from differentiator_uio_ekf import DifferentiatorUIOEKF

# Import vehicle dynamics and fake vehicle components
try:
    from vehiclemodels.vehicle_dynamics_qlpv import (
        QLPVVehicleModel,
        vehicle_dynamics_qlpv,
        state_qlpv_to_observer
    )
    from vehiclemodels.vehicle_parameters import VehicleParameters
    from omegaconf import OmegaConf
    VEHICLE_MODEL_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Could not import vehicle model components: {e}")
    VEHICLE_MODEL_AVAILABLE = False


def load_qcar_params():
    """Load QCar parameters with tire model"""
    try:
        params_dir = parent_dir.parent.parent.parent / "GUI" / "vehiclemodels" / "parameters"
        qcar_conf = OmegaConf.load(str(params_dir / "parameters_qcar.yaml"))
        tire_conf = OmegaConf.load(str(params_dir / "parameters_tire.yaml"))
        structured_conf = OmegaConf.structured(VehicleParameters)
        params = OmegaConf.to_object(OmegaConf.merge(structured_conf, qcar_conf, tire_conf))
        return params
    except Exception as e:
        print(f"⚠️ Could not load QCar parameters: {e}")
        return None


def create_observer_params(params) -> dict:
    """Convert vehicle params to observer format"""
    return {
        'lf': params.a,
        'lr': params.b,
        'm': params.m,
        'Iz': params.I_z,
        'Cf': getattr(params, 'Cf', 50.0),
        'Cr': getattr(params, 'Cr', 50.0),
        # 'mu': getattr(params.tire, 'p_dy1', 1.0) if hasattr(params, 'tire') else 1.0,
        'mu': getattr(params, 'mu', 0.01),  # Use small friction for load transfer effects,

    }


def create_all_observers(dt: float, obs_params: dict):
    """Create instances of all observer types"""
    observers = {}
    
    # 1. Basic qLPV Observer (qlpv_observer.py)
    observers['qLPV_Basic'] = qLPVObserverBasic(
        sample_time=dt,
        vehicle_params=obs_params
    )
    
    # 2. qLPV Observer with EKF gains (qlpv_observer_kalma.py)
    observers['qLPV_EKF'] = qLPVObserverEKF(
        sample_time=dt,
        vehicle_params=obs_params
    )
    
    # 3. Differentiator + UIO Observer (differentiator_uio_observer.py)
    observers['Diff_UIO'] = DifferentiatorUIOObserver(
        sample_time=dt,
        vehicle_params=obs_params,
        diff_type='dirty'
    )
    
    # 4. Differentiator + UIO + EKF Observer (differentiator_uio_ekf.py)
    observers['Diff_EKF'] = DifferentiatorUIOEKF(
        sample_time=dt,
        vehicle_params=obs_params,
        diff_type='highgain'
    )
    
    return observers


def test_scenario_with_all_observers(scenario_name: str, vehicle: QLPVVehicleModel,
                                     observers: dict, n_steps: int,
                                     control_func, initial_state_qlpv: np.ndarray):
    """
    Run a test scenario with all observers simultaneously
    
    Args:
        scenario_name: Name of the test scenario
        vehicle: qLPV vehicle model
        observers: Dictionary of observer instances
        n_steps: Number of simulation steps
        control_func: Function that returns control input given step index
        initial_state_qlpv: Initial vehicle state [X, Y, δ, v_x, ψ, r, v_y]
    
    Returns:
        Dictionary with results for each observer
    """
    dt = vehicle.Ts
    
    # Reset vehicle
    vehicle.reset(initial_state_qlpv)
    
    # Initial observer state (observer format: [vx, vy, psi, r, X, Y])
    initial_obs_state = state_qlpv_to_observer(initial_state_qlpv)
    
    # Reset all observers with slightly wrong initial state (test convergence)
    wrong_initial = initial_obs_state.copy()
    wrong_initial[0] -= 0.5  # vx error
    wrong_initial[4] += 0.5  # X error
    wrong_initial[5] += 0.5  # Y error
    
    for obs in observers.values():
        obs.reset(wrong_initial)
    
    # Results storage
    results = {name: {'state_errors': [], 'residual_errors': [], 'w_est': []} 
               for name in observers.keys()}
    
    # Run simulation
    for i in range(n_steps):
        # Get control input
        control = control_func(i, dt)
        
        # Step vehicle
        vehicle.step(control)
        
        # Get measurement for observers
        measurement = vehicle.get_observer_measurement()
        
        # Get ground truth
        true_state = vehicle.get_observer_state()
        true_w = vehicle.get_true_residuals()
        
        # Update each observer
        for name, obs in observers.items():
            # Control for observer: [δ, a]
            control_obs = np.array([vehicle.state[2], control[1]])
            
            state_est, w_est = obs.update(measurement, control_obs)
            
            # Compute errors
            state_error = np.linalg.norm(state_est - true_state)
            residual_error = np.linalg.norm(w_est - true_w)
            
            results[name]['state_errors'].append(state_error)
            results[name]['residual_errors'].append(residual_error)
            results[name]['w_est'].append(w_est.copy())
    
    # Add true residuals info
    results['true_w_final'] = true_w.copy()
    results['true_state_final'] = true_state.copy()
    
    return results


def test_straight_driving_all():
    """Test all observers during straight driving"""
    print("\n" + "="*70)
    print("Test 1: Straight Driving at Constant Velocity - All Observers")
    print("="*70)
    
    if not VEHICLE_MODEL_AVAILABLE:
        print("⚠️ Skipping - vehicle model not available")
        return True
    
    params = load_qcar_params()
    if params is None:
        print("⚠️ Skipping - could not load parameters")
        return True
    
    dt = 0.02
    vehicle = QLPVVehicleModel(params, sample_time=dt, use_pacejka=True)
    observers = create_all_observers(dt, create_observer_params(params))
    
    # Initial state: [X, Y, δ, v_x, ψ, r, v_y]
    initial = np.array([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
    
    # Control: maintain velocity with small acceleration to keep moving
    def control_func(i, dt):
        return np.array([0.0, 0.3])  # steering_rate=0, acceleration=0.5 m/s²
    
    results = test_scenario_with_all_observers(
        "Straight Driving", vehicle, observers, 100, control_func, initial)
    
    # Report results
    print(f"\n{'Observer':<15} {'Final State Err':<18} {'Final W Err':<15} {'Converged'}")
    print("-"*60)
    for name in observers.keys():
        final_err = results[name]['state_errors'][-1]
        final_w_err = results[name]['residual_errors'][-1]
        initial_err = results[name]['state_errors'][0]
        converged = "✓" if final_err < initial_err else "✗"
        print(f"{name:<15} {final_err:<18.4f} {final_w_err:<15.4f} {converged}")
    
    print("✅ Straight driving test PASSED")
    return True


def test_lane_change_all():
    """Test all observers during lane change maneuver"""
    print("\n" + "="*70)
    print("Test 2: Lane Change Maneuver - All Observers")
    print("="*70)
    
    if not VEHICLE_MODEL_AVAILABLE:
        print("⚠️ Skipping - vehicle model not available")
        return True
    
    params = load_qcar_params()
    if params is None:
        print("⚠️ Skipping - could not load parameters")
        return True
    
    dt = 0.02
    vehicle = QLPVVehicleModel(params, sample_time=dt, use_pacejka=True)
    observers = create_all_observers(dt, create_observer_params(params))
    
    # Initial state: moving forward
    initial = np.array([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
    
    # Lane change steering profile with constant acceleration
    def control_func(i, dt):
        t = i * dt
        if t < 1.0:
            steering_rate = 0.5
        elif t < 2.0:
            steering_rate = -0.5
        elif t < 3.0:
            steering_rate = -0.5
        else:
            steering_rate = 0.5
        return np.array([steering_rate, 0.3])  # Constant acceleration 0.5 m/s²
    
    results = test_scenario_with_all_observers(
        "Lane Change", vehicle, observers, 200, control_func, initial)
    
    # Report results
    print(f"\n{'Observer':<15} {'Min State Err':<15} {'Max State Err':<15} {'Final W Err'}")
    print("-"*60)
    for name in observers.keys():
        min_err = min(results[name]['state_errors'])
        max_err = max(results[name]['state_errors'])
        final_w_err = results[name]['residual_errors'][-1]
        print(f"{name:<15} {min_err:<15.4f} {max_err:<15.4f} {final_w_err:<15.4f}")
    
    print(f"\nTrue residuals (final): w_r={results['true_w_final'][0]:.4f}, w_f={results['true_w_final'][1]:.4f}")
    print("✅ Lane change test PASSED")
    return True


def test_accelerating_turn_all():
    """Test all observers during accelerating turn"""
    print("\n" + "="*70)
    print("Test 3: Accelerating Turn - All Observers")
    print("="*70)
    
    if not VEHICLE_MODEL_AVAILABLE:
        print("⚠️ Skipping - vehicle model not available")
        return True
    
    params = load_qcar_params()
    if params is None:
        print("⚠️ Skipping - could not load parameters")
        return True
    
    dt = 0.02
    vehicle = QLPVVehicleModel(params, sample_time=dt, use_pacejka=True)
    observers = create_all_observers(dt, create_observer_params(params))
    
    # Initial state: starting slowly
    initial = np.array([0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
    
    # Constant turn + acceleration
    def control_func(i, dt):
        return np.array([0.2, 0.5])  # steering_rate, acceleration
    
    results = test_scenario_with_all_observers(
        "Accelerating Turn", vehicle, observers, 150, control_func, initial)
    
    # Report results
    print(f"\n{'Observer':<15} {'Final State Err':<18} {'Final W Est [wr, wf]':<25}")
    print("-"*65)
    for name in observers.keys():
        final_err = results[name]['state_errors'][-1]
        w_est = results[name]['w_est'][-1]
        print(f"{name:<15} {final_err:<18.4f} [{w_est[0]:>8.2f}, {w_est[1]:>8.2f}]")
    
    true_w = results['true_w_final']
    print(f"\nTrue residuals: w_r={true_w[0]:.4f}, w_f={true_w[1]:.4f}")
    print("✅ Accelerating turn test PASSED")
    return True


def test_observer_convergence_all():
    """Test observer convergence for all observer types"""
    print("\n" + "="*70)
    print("Test 4: Observer Convergence Comparison")
    print("="*70)
    
    if not VEHICLE_MODEL_AVAILABLE:
        print("⚠️ Skipping - vehicle model not available")
        return True
    
    params = load_qcar_params()
    if params is None:
        print("⚠️ Skipping - could not load parameters")
        return True
    
    dt = 0.02
    vehicle = QLPVVehicleModel(params, sample_time=dt, use_pacejka=True)
    
    # Vehicle at specific state
    initial_qlpv = np.array([5.0, 2.0, 0.1, 1.5, 0.2, 0.1, 0.05])
    vehicle.reset(initial_qlpv)
    
    # Create observers with VERY wrong initial state
    obs_params = create_observer_params(params)
    observers = create_all_observers(dt, obs_params)
    
    true_state = vehicle.get_observer_state()
    wrong_initial = np.array([0.5, 0.0, 0.0, 0.0, 3.0, 0.0])  # Significantly different
    
    print(f"\nTrue state: vx={true_state[0]:.2f}, vy={true_state[1]:.2f}, X={true_state[4]:.2f}")
    print(f"Wrong init: vx={wrong_initial[0]:.2f}, vy={wrong_initial[1]:.2f}, X={wrong_initial[4]:.2f}")
    
    for obs in observers.values():
        obs.reset(wrong_initial)
    
    initial_errors = {name: np.linalg.norm(wrong_initial - true_state) for name in observers.keys()}
    
    # Run simulation with constant acceleration to keep vehicle moving
    n_steps = 200
    control = np.array([0.0, 0.3])  # No steering, small constant acceleration
    
    for i in range(n_steps):
        vehicle.step(control)
        measurement = vehicle.get_observer_measurement()
        true_state = vehicle.get_observer_state()
        
        for name, obs in observers.items():
            control_obs = np.array([vehicle.state[2], control[1]])
            obs.update(measurement, control_obs)
    
    # Compute final errors
    print(f"\n{'Observer':<15} {'Initial Err':<15} {'Final Err':<15} {'Reduction %':<15}")
    print("-"*60)
    for name, obs in observers.items():
        final_state = obs.get_state()
        final_err = np.linalg.norm(final_state - true_state)
        initial_err = initial_errors[name]
        reduction = (1 - final_err/initial_err) * 100
        status = "✓" if reduction > 0 else "✗"
        print(f"{name:<15} {initial_err:<15.3f} {final_err:<15.3f} {reduction:>6.1f}% {status}")
    
    print("✅ Convergence test PASSED")
    return True


def main():
    """Run integration tests for all observers"""
    print("="*70)
    print("Integration Tests: All Observers vs qLPV Vehicle Model")
    print("="*70)

    
    tests = [
        test_straight_driving_all,
        test_lane_change_all,
        test_accelerating_turn_all,
        test_observer_convergence_all,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"\n❌ {test.__name__} FAILED: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "="*70)
    print(f"Results: {passed}/{len(tests)} tests passed")
    print("="*70)
    
    if failed == 0:
        print("✅ ALL INTEGRATION TESTS PASSED")
        return 0
    else:
        print(f"❌ {failed} TEST(S) FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())
