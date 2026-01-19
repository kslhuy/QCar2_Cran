"""
Multi-Layer Observer Comparison Test

Compares all Layer 1 + Layer 2 observer combinations to find the best architecture.

Layer 1 Observers (from 1LayerObs/):
    1. qLPV_Basic - Basic qLPV with LMI gain scheduling
    2. qLPV_EKF - EKF-style gain computation
    3. Diff_UIO - UIO with differentiator
    4. Diff_EKF - UIO with EKF

Layer 2 Configurations (from 2LayerObs/):
    Gain Design Methods: default, hinf, l2, lmi, qlpv_scheduled
    Loss Functions: measurement_full, composite_uio

Test Scenarios:
    1. Straight driving at constant velocity
    2. Lane change maneuver
    3. Accelerating turn

Metrics:
    - State estimation error
    - Tire residual estimation error
    - Convergence rate
"""

import numpy as np
import sys
import warnings
from pathlib import Path
from typing import Dict, List, Tuple, Optional, TYPE_CHECKING
from dataclasses import dataclass, field
import time

# Suppress specific numpy warnings for cleaner output
warnings.filterwarnings('ignore', category=RuntimeWarning, message='Mean of empty slice')
warnings.filterwarnings('ignore', category=RuntimeWarning, message='overflow encountered')
warnings.filterwarnings('ignore', category=RuntimeWarning, message='invalid value encountered')

# Add paths - order matters!
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))
sys.path.insert(0, str(parent_dir / "1LayerObs"))
sys.path.insert(0, str(parent_dir / "2LayerObs"))
# Add Observer directory (contains local_state_estimators.py)
sys.path.insert(0, str(parent_dir.parent))
# Navigate to GUI for vehicle model
sys.path.insert(0, str(parent_dir.parent.parent / "GUI"))

# Check for PyTorch
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("⚠️ PyTorch not available, neural estimator tests will be skipped")

# Import Layer 1 observers
try:
    from qlpv_observer import qLPVAugmentedObserver as qLPVObserverBasic
    from qlpv_observer_kalma import qLPVAugmentedObserver as qLPVObserverEKF
    from differentiator_uio_observer import DifferentiatorUIOObserver
    from differentiator_uio_ekf import DifferentiatorUIOEKF
    LAYER1_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Could not import Layer 1 observers: {e}")
    LAYER1_AVAILABLE = False

# Import Layer 2 neural estimator
try:
    from neural_state_estimator import NeuralLuenbergerEstimator
    LAYER2_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Could not import Layer 2 estimator: {e}")
    LAYER2_AVAILABLE = False

# Import vehicle model
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
    print(f"⚠️ Could not import vehicle model: {e}")
    VEHICLE_MODEL_AVAILABLE = False


# =============================================================================
# Data Classes for Results
# =============================================================================

@dataclass
class TestMetrics:
    """Metrics for a single test run"""
    state_errors: List[float] = field(default_factory=list)
    residual_errors: List[float] = field(default_factory=list)
    convergence_time: float = 0.0
    final_state_error: float = 0.0
    final_residual_error: float = 0.0
    mean_state_error: float = 0.0
    mean_residual_error: float = 0.0


@dataclass
class ConfigurationResult:
    """Results for a specific configuration"""
    layer1_type: str
    gain_method: str
    loss_type: str
    straight_metrics: Optional[TestMetrics] = None
    lane_change_metrics: Optional[TestMetrics] = None
    accel_turn_metrics: Optional[TestMetrics] = None
    overall_score: float = 0.0


# =============================================================================
# Helper Functions
# =============================================================================

def load_qcar_params():
    """Load QCar parameters with tire model"""
    try:
        params_dir = parent_dir.parent.parent / "GUI" / "vehiclemodels" / "parameters"
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
        'mu': getattr(params, 'mu', 0.01),
    }


def create_layer1_observer(obs_type: str, dt: float, obs_params: dict):
    """Create a Layer 1 observer by type"""
    if obs_type == 'qLPV_Basic':
        return qLPVObserverBasic(
            sample_time=dt,
            vehicle_params=obs_params,
            use_gain_scheduling=True,
            verbose=False
        )
    elif obs_type == 'qLPV_EKF':
        return qLPVObserverEKF(
            sample_time=dt,
            vehicle_params=obs_params
        )
    elif obs_type == 'Diff_UIO':
        return DifferentiatorUIOObserver(
            sample_time=dt,
            vehicle_params=obs_params,
            diff_type='dirty'
        )
    elif obs_type == 'Diff_EKF':
        return DifferentiatorUIOEKF(
            sample_time=dt,
            vehicle_params=obs_params,
            diff_type='highgain'
        )
    else:
        raise ValueError(f"Unknown Layer 1 type: {obs_type}")


def create_neural_estimator(
    layer1_type: str,
    gain_method: str,
    loss_type: str,
    initial_pose: np.ndarray = None
):
    """Create a NeuralLuenbergerEstimator with specific configuration"""
    config = {
        'use_first_layer': True,
        'first_layer_type': 'qlpv' if 'qLPV' in layer1_type else 'differentiator_uio',
        'gain_design_method': gain_method,
        'loss_type': loss_type,
        # Gain design parameters
        'hinf_gamma': 2.0,
        'l2_gamma': 2.0,
        'lmi_decay_rate': 0.5,
        'use_gain_scheduling': gain_method == 'qlpv_scheduled',
        'vx_range': [0.5, 3.0],
        'delta_max': 0.4,
    }
    
    if initial_pose is None:
        initial_pose = np.array([0.0, 0.0, 0.0])
    
    return NeuralLuenbergerEstimator(initial_pose=initial_pose, config=config)


def compute_metrics(state_errors: List[float], residual_errors: List[float], 
                    dt: float, threshold: float = 0.5) -> TestMetrics:
    """Compute test metrics from error history"""
    metrics = TestMetrics()
    metrics.state_errors = state_errors
    metrics.residual_errors = residual_errors
    metrics.final_state_error = state_errors[-1] if state_errors else 0.0
    metrics.final_residual_error = residual_errors[-1] if residual_errors else 0.0
    metrics.mean_state_error = np.mean(state_errors) if state_errors else 0.0
    metrics.mean_residual_error = np.mean(residual_errors) if residual_errors else 0.0
    
    # Convergence time: first time error drops below threshold
    for i, err in enumerate(state_errors):
        if err < threshold:
            metrics.convergence_time = i * dt
            break
    else:
        metrics.convergence_time = len(state_errors) * dt  # Did not converge
    
    return metrics


# =============================================================================
# Test Scenarios
# =============================================================================

def run_scenario(
    scenario_name: str,
    vehicle: 'QLPVVehicleModel',
    neural_estimator,
    n_steps: int,
    control_func,
    initial_state_qlpv: np.ndarray,
    dt: float,
    gps_rate: float = 10.0  # GPS update rate in Hz
) -> TestMetrics:
    """
    Run a single test scenario with realistic sensor rates.
    
    Args:
        scenario_name: Name of the scenario
        vehicle: qLPV vehicle model
        neural_estimator: Layer 2 neural estimator
        n_steps: Number of simulation steps
        control_func: Function that returns control input given step index
        initial_state_qlpv: Initial vehicle state [X, Y, δ, v_x, ψ, r, v_y]
        dt: Sample time (IMU rate = 1/dt)
        gps_rate: GPS update rate in Hz (default 10 Hz)
    
    Returns:
        TestMetrics with collected data
    """
    # Reset vehicle
    vehicle.reset(initial_state_qlpv)
    
    # Reset neural estimator with slightly wrong initial state
    initial_obs_state = state_qlpv_to_observer(initial_state_qlpv)
    wrong_initial = np.array([
        initial_obs_state[4] + 0.5,    # X error
        initial_obs_state[5] + 0.5,    # Y error
        initial_obs_state[2],           # ψ correct
        initial_obs_state[0] - 0.5     # vx error
    ])
    neural_estimator.reset(wrong_initial[:3])  # Reset with [X, Y, ψ]
    neural_estimator.state_nn_6d[0] = wrong_initial[3]  # Set vx
    
    # Calculate GPS update interval (steps between valid GPS)
    imu_rate = 1.0 / dt  # e.g., 50 Hz at dt=0.02
    gps_update_interval = max(1, int(imu_rate / gps_rate))  # e.g., 5 steps for 10Hz GPS at 50Hz IMU
    
    state_errors = []
    residual_errors = []
    
    # Run simulation
    for i in range(n_steps):
        # Get control input
        control = control_func(i, dt)
        
        # Step vehicle
        vehicle.step(control)
        
        # Get ground truth (for error calculation only)
        true_state = vehicle.get_observer_state()  # [vx, vy, ψ, r, X, Y]
        true_w = vehicle.get_true_residuals()
        
        # Get measurement (what the observer actually sees)
        # measurement: [vx, vy, ψ, r, X, Y, a_y] with sensor noise
        measurement = vehicle.get_observer_measurement()
        
        # Extract IMU measurements (always available at high rate)
        vx_meas = measurement[0]  # Motor tachometer / wheel encoder
        r_meas = measurement[3]   # Gyroscope z-axis
        accel = np.array([0.0, measurement[6]]) if len(measurement) > 6 else None  # IMU a_y
        
        # GPS data (only valid at gps_rate Hz)
        gps_valid = (i % gps_update_interval == 0)
        
        if gps_valid:
            gps_data = {
                'x': measurement[4] + 0.01 * np.random.randn(),      # X with GPS noise
                'y': measurement[5] + 0.01 * np.random.randn(),      # Y with GPS noise
                'theta': measurement[2] + 0.001 * np.random.randn(), # ψ with GPS noise
                'valid': True
            }
        else:
            gps_data = {'valid': False}
        
        # Update neural estimator using measurements (not true state)
        neural_estimator.update(
            motor_tach=vx_meas,
            steering=vehicle.state[2],  # Current steering angle from vehicle
            throttle=control[1],
            dt=dt,
            gyro_z=r_meas,
            gps_data=gps_data,
            accel=accel
        )
        
        # Get estimates
        state_est = neural_estimator.get_state_6d()
        w_est = neural_estimator.get_tire_residuals()
        
        # Compute errors (compare to true state, not measurement)
        state_error = np.linalg.norm(state_est - true_state)
        residual_error = np.linalg.norm(w_est - true_w)
        
        state_errors.append(state_error)
        residual_errors.append(residual_error)
    
    return compute_metrics(state_errors, residual_errors, dt)


def test_straight_driving(vehicle, neural_estimator, dt: float) -> TestMetrics:
    """Test straight driving scenario"""
    initial = np.array([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
    
    def control_func(i, dt):
        return np.array([0.0, 0.3])  # No steering, small acceleration
    
    return run_scenario("Straight", vehicle, neural_estimator, 100, control_func, initial, dt)


def test_lane_change(vehicle, neural_estimator, dt: float) -> TestMetrics:
    """Test lane change scenario"""
    initial = np.array([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
    
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
        return np.array([steering_rate, 0.3])
    
    return run_scenario("Lane Change", vehicle, neural_estimator, 200, control_func, initial, dt)


def test_accelerating_turn(vehicle, neural_estimator, dt: float) -> TestMetrics:
    """Test accelerating turn scenario"""
    initial = np.array([0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
    
    def control_func(i, dt):
        return np.array([0.2, 0.5])  # Constant steering + acceleration
    
    return run_scenario("Accel Turn", vehicle, neural_estimator, 150, control_func, initial, dt)


# =============================================================================
# Main Comparison Test
# =============================================================================

def run_multilayer_comparison():
    """Run comprehensive comparison of all Layer 1 + Layer 2 combinations"""
    print("=" * 80)
    print("Multi-Layer Observer Comparison Test")
    print("=" * 80)
    
    if not all([TORCH_AVAILABLE, LAYER1_AVAILABLE, LAYER2_AVAILABLE, VEHICLE_MODEL_AVAILABLE]):
        print("❌ Missing required components. Cannot run comparison.")
        return []
    
    # Load parameters
    params = load_qcar_params()
    if params is None:
        print("❌ Could not load vehicle parameters")
        return []
    
    dt = 0.02
    vehicle = QLPVVehicleModel(params, sample_time=dt, use_pacejka=True)
    
    # Define configurations to test
    layer1_types = ['qLPV_Basic', 'qLPV_EKF', 'Diff_UIO', 'Diff_EKF']
    gain_methods = ['default', 'hinf', 'l2']  # Skip qlpv_scheduled for speed
    loss_types = ['measurement_full', 'composite_uio']
    
    results: List[ConfigurationResult] = []
    
    total_configs = len(layer1_types) * len(gain_methods) * len(loss_types)
    config_num = 0
    
    print(f"\nTesting {total_configs} configurations...")
    print("-" * 80)
    
    for layer1_type in layer1_types:
        for gain_method in gain_methods:
            for loss_type in loss_types:
                config_num += 1
                config_name = f"{layer1_type} + {gain_method} + {loss_type}"
                print(f"\n[{config_num}/{total_configs}] Testing: {config_name}")
                
                result = ConfigurationResult(
                    layer1_type=layer1_type,
                    gain_method=gain_method,
                    loss_type=loss_type
                )
                
                try:
                    # Create neural estimator
                    neural_estimator = create_neural_estimator(
                        layer1_type, gain_method, loss_type
                    )
                    
                    # Run test scenarios
                    print("  Running straight driving...", end=" ")
                    result.straight_metrics = test_straight_driving(vehicle, neural_estimator, dt)
                    print(f"✓ (err: {result.straight_metrics.final_state_error:.4f})")
                    
                    print("  Running lane change...", end=" ")
                    result.lane_change_metrics = test_lane_change(vehicle, neural_estimator, dt)
                    print(f"✓ (err: {result.lane_change_metrics.final_state_error:.4f})")
                    
                    print("  Running accelerating turn...", end=" ")
                    result.accel_turn_metrics = test_accelerating_turn(vehicle, neural_estimator, dt)
                    print(f"✓ (err: {result.accel_turn_metrics.final_state_error:.4f})")
                    
                    # Compute overall score (lower is better)
                    result.overall_score = (
                        result.straight_metrics.mean_state_error +
                        result.lane_change_metrics.mean_state_error +
                        result.accel_turn_metrics.mean_state_error
                    ) / 3.0
                    
                except Exception as e:
                    print(f"  ❌ Error: {e}")
                    result.overall_score = float('inf')
                
                results.append(result)
    
    return results


def print_results_table(results: List[ConfigurationResult]):
    """Print formatted results table"""
    print("\n" + "=" * 130)
    print("RESULTS SUMMARY - State Estimation Error")
    print("=" * 130)
    
    # Sort by overall score
    sorted_results = sorted(results, key=lambda r: r.overall_score if not np.isnan(r.overall_score) else float('inf'))
    
    # Header for state errors
    print(f"{'Rank':<5} {'Layer1':<12} {'Gain Method':<12} {'Loss Type':<18} "
          f"{'Straight':<10} {'LaneChg':<10} {'AccelTurn':<10} {'AvgStateErr':<12}")
    print("-" * 130)
    
    for rank, result in enumerate(sorted_results, 1):
        if result.overall_score == float('inf') or np.isnan(result.overall_score):
            print(f"{rank:<5} {result.layer1_type:<12} {result.gain_method:<12} "
                  f"{result.loss_type:<18} {'FAILED':<10} {'-':<10} {'-':<10} {'N/A':<12}")
        else:
            straight = result.straight_metrics.final_state_error if result.straight_metrics else 0
            lane = result.lane_change_metrics.final_state_error if result.lane_change_metrics else 0
            accel = result.accel_turn_metrics.final_state_error if result.accel_turn_metrics else 0
            
            print(f"{rank:<5} {result.layer1_type:<12} {result.gain_method:<12} "
                  f"{result.loss_type:<18} {straight:<10.4f} {lane:<10.4f} "
                  f"{accel:<10.4f} {result.overall_score:<12.4f}")
    
    # Second table for residual errors
    print("\n" + "=" * 130)
    print("RESULTS SUMMARY - Tire Residual (w) Estimation Error")
    print("=" * 130)
    
    print(f"{'Rank':<5} {'Layer1':<12} {'Gain Method':<12} {'Loss Type':<18} "
          f"{'Straight':<10} {'LaneChg':<10} {'AccelTurn':<10} {'AvgResidErr':<12}")
    print("-" * 130)
    
    # Sort by average residual error
    residual_sorted = sorted(results, key=lambda r: (
        np.nanmean([
            r.straight_metrics.final_residual_error if r.straight_metrics else float('nan'),
            r.lane_change_metrics.final_residual_error if r.lane_change_metrics else float('nan'),
            r.accel_turn_metrics.final_residual_error if r.accel_turn_metrics else float('nan')
        ]) if r.straight_metrics else float('inf')
    ))
    
    for rank, result in enumerate(residual_sorted, 1):
        if result.straight_metrics is None:
            print(f"{rank:<5} {result.layer1_type:<12} {result.gain_method:<12} "
                  f"{result.loss_type:<18} {'FAILED':<10} {'-':<10} {'-':<10} {'N/A':<12}")
        else:
            straight_w = result.straight_metrics.final_residual_error
            lane_w = result.lane_change_metrics.final_residual_error if result.lane_change_metrics else 0
            accel_w = result.accel_turn_metrics.final_residual_error if result.accel_turn_metrics else 0
            avg_w = np.nanmean([straight_w, lane_w, accel_w])
            
            if np.isnan(avg_w):
                print(f"{rank:<5} {result.layer1_type:<12} {result.gain_method:<12} "
                      f"{result.loss_type:<18} {'nan':<10} {'nan':<10} {'nan':<10} {'nan':<12}")
            else:
                print(f"{rank:<5} {result.layer1_type:<12} {result.gain_method:<12} "
                      f"{result.loss_type:<18} {straight_w:<10.4f} {lane_w:<10.4f} "
                      f"{accel_w:<10.4f} {avg_w:<12.4f}")
    
    # Combined summary table
    print("\n" + "=" * 140)
    print("COMBINED RANKING (State + Residual)")
    print("=" * 140)
    
    # Calculate combined score: weighted average of state and residual errors
    combined_scores = []
    for result in results:
        if result.straight_metrics and not np.isnan(result.overall_score):
            avg_residual = np.nanmean([
                result.straight_metrics.final_residual_error,
                result.lane_change_metrics.final_residual_error if result.lane_change_metrics else float('nan'),
                result.accel_turn_metrics.final_residual_error if result.accel_turn_metrics else float('nan')
            ])
            if not np.isnan(avg_residual):
                combined = result.overall_score + avg_residual * 0.5  # Weight residual less
                combined_scores.append((result, result.overall_score, avg_residual, combined))
    
    combined_scores.sort(key=lambda x: x[3])
    
    print(f"{'Rank':<5} {'Layer1':<12} {'Gain Method':<12} {'Loss Type':<18} "
          f"{'AvgStateErr':<12} {'AvgResidErr':<12} {'CombinedScore':<14}")
    print("-" * 140)
    
    for rank, (result, state_err, resid_err, combined) in enumerate(combined_scores[:10], 1):
        print(f"{rank:<5} {result.layer1_type:<12} {result.gain_method:<12} "
              f"{result.loss_type:<18} {state_err:<12.4f} {resid_err:<12.4f} {combined:<14.4f}")
    
    # Best configuration
    if combined_scores:
        best_result, best_state, best_resid, best_combined = combined_scores[0]
        print("\n" + "=" * 140)
        print("🏆 BEST CONFIGURATION:")
        print(f"   Layer 1: {best_result.layer1_type}")
        print(f"   Gain Method: {best_result.gain_method}")
        print(f"   Loss Type: {best_result.loss_type}")
        print(f"   Average State Error: {best_state:.4f}")
        print(f"   Average Residual Error: {best_resid:.4f}")
        print(f"   Combined Score: {best_combined:.4f}")
        print("=" * 140)


def main():
    """Main entry point"""
    print("\n" + "=" * 80)
    print("Starting Multi-Layer Observer Comparison...")
    print("=" * 80 + "\n")
    
    start_time = time.time()
    
    results = run_multilayer_comparison()
    
    if results:
        print_results_table(results)
    
    elapsed = time.time() - start_time
    print(f"\nTotal test time: {elapsed:.1f} seconds")
    
    return 0 if results else 1


if __name__ == '__main__':
    sys.exit(main())
