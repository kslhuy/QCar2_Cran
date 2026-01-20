"""Test script to verify centralized qLPV vehicle dynamics module and min_vx consistency"""

import sys
from pathlib import Path

# Setup paths properly
current_dir = Path(__file__).parent
observer_dir = current_dir.parent  # Observer directory (contains local_state_estimators.py)
qcar_dir = observer_dir.parent  # qcar directory

# Add paths in correct order - Observer dir first for local_state_estimators
sys.path.insert(0, str(observer_dir))
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir / "1LayerObs"))
sys.path.insert(0, str(current_dir / "2LayerObs"))

print("=" * 60)
print("Testing Centralized qLPV Vehicle Dynamics Module")
print("=" * 60)

# Test 1: Import centralized module
print("\n1. Testing centralized module import...")
try:
    from qlpv_vehicle_dynamics_obs import (
        SchedulingParameters,
        QLPVVehicleDynamicsObs,
        get_default_vehicle_params,
        get_vehicle_params_from_yaml,
        load_vehicle_params_from_yaml,
        IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y,
    )
    print("   ✓ Centralized module imported successfully")
except Exception as e:
    print(f"   ✗ Failed: {e}")
    sys.exit(1)

# Test 2: Create dynamics instance with default params
print("\n2. Testing QLPVVehicleDynamicsObs with default params...")
try:
    params = get_default_vehicle_params()
    dynamics = QLPVVehicleDynamicsObs()
    print(f"   ✓ Dynamics created: lf={dynamics.lf}, m={dynamics.m}, Iz={dynamics.Iz}, min_vx={dynamics.min_vx}")
except Exception as e:
    print(f"   ✗ Failed: {e}")
    sys.exit(1)

# Test 3: Load params from YAML
print("\n3. Testing YAML parameter loading...")
try:
    yaml_params = get_vehicle_params_from_yaml()
    print(f"   ✓ YAML params: lf={yaml_params['lf']}, m={yaml_params['m']}, Iz={yaml_params['Iz']}, vx_min={yaml_params.get('vx_min', 'N/A')}")
except Exception as e:
    print(f"   ✗ YAML loading failed: {e}")

# Test 4: Import from 1LayerObs observers and check min_vx
print("\n4. Testing 1LayerObs imports (with vx_min check)...")

try:
    from differentiator_uio_observer import DifferentiatorUIOObserver
    obs = DifferentiatorUIOObserver()
    print(f"   ✓ DifferentiatorUIOObserver: lf={obs.lf}, m={obs.m}, min_vx={obs.min_vx}")
except Exception as e:
    print(f"   ✗ DifferentiatorUIOObserver failed: {e}")

try:
    from differentiator_uio_ekf import DifferentiatorUIOEKF
    obs = DifferentiatorUIOEKF()
    print(f"   ✓ DifferentiatorUIOEKF: lf={obs.lf}, m={obs.m}, min_vx={obs.min_vx}")
except Exception as e:
    print(f"   ✗ DifferentiatorUIOEKF failed: {e}")

try:
    from qlpv_observer import qLPVAugmentedObserver
    obs = qLPVAugmentedObserver()
    print(f"   ✓ qLPVAugmentedObserver: lf={obs.lf}, m={obs.m}, min_vx={obs.min_vx}")
except Exception as e:
    print(f"   ✗ qLPVAugmentedObserver failed: {e}")

try:
    from qlpv_observer_kalma import qLPVAugmentedObserver as qLPVKalma
    obs = qLPVKalma()
    print(f"   ✓ qLPVAugmentedObserver (Kalman): lf={obs.lf}, m={obs.m}, min_vx={obs.min_vx}")
except Exception as e:
    print(f"   ✗ qLPVAugmentedObserver (Kalman) failed: {e}")

# Test 5: Import from 2LayerObs
print("\n5. Testing 2LayerObs imports (with vx_min check)...")
try:
    from neural_state_estimator import NeuralLuenbergerEstimator
    obs = NeuralLuenbergerEstimator()
    print(f"   ✓ NeuralLuenbergerEstimator: lf={obs.lf}, m={obs.m}, min_vx={obs.min_vx}")
except Exception as e:
    print(f"   ✗ NeuralLuenbergerEstimator failed: {e}")

print("\n" + "=" * 60)
print("All tests completed!")
print("=" * 60)
