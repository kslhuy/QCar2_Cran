
import sys
import os
import numpy as np
from pathlib import Path

# Add project root to path
# Assuming we are running from project root or somewhere close
# Adjust path as necessary relative to this script
current_dir = Path(__file__).parent.absolute()

# We need to find the 'qcar' package.
# Based on file paths: Development/multi_vehicle_self_driving_RealQcar/
# If we place this test in Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/LocalNeuralObs/
# then project root is ../../../../
project_root = current_dir.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

try:
    # Try importing using sys.path manipulation since '2LayerObs' is not a valid python identifier for direct import
    sys.path.insert(0, str(current_dir / "2LayerObs"))
    sys.path.insert(0, str(current_dir / "1LayerObs"))
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    from qlpv_observer_kalma import qLPVKalmanObserver
except ImportError as e:
    print(f"Import Error: {e}")
    # Try alternate structure if above fails
    # ...
    raise e

def test_shared_dynamics_model():
    print("Testing Shared QLPV Dynamics Model...")
    
    # 1. Initialize Neural State Estimator (2nd Layer)
    # We need a minimal config
    config = {
        'input_dim': 6,
        'hidden_dim': 64,
        'output_dim': 2,
        'learning_rate': 0.001,
        'weight_decay': 1e-4,
        'learning_mode': 'none', # No learning needed for this test
        'sample_time': 0.02,
        'batch_size': 32,
        'load_pretrained': False,
        'gain_design_method': 'default', # Avoid complex LMI if not needed
        'first_layer_type': 'qlpv_kalman', # We want to test this specific integration
        'use_first_layer': True,
        
        # Weights (dummy)
        'weight_vx': 1.0, 'weight_vy': 1.0, 'weight_psi': 1.0, 
        'weight_psi_dot': 1.0, 'weight_X': 1.0, 'weight_Y': 1.0,
        'weight_ref_vx': 1.0, 'weight_ref_vy': 1.0, 'weight_ref_psi': 1.0,
        'weight_ref_r': 1.0, 'weight_ref_X': 1.0, 'weight_ref_Y': 1.0,
        'weight_uio_vx': 1.0, 'weight_uio_vy': 1.0, 'weight_uio_psi': 1.0,
        'weight_uio_r': 1.0, 'weight_uio_X': 1.0, 'weight_uio_Y': 1.0
    }
    
    estimator = NeuralLuenbergerEstimator(config=config)
    
    # 2. Check connections
    if estimator.first_layer_observer is None:
        print("FAIL: First layer observer not initialized.")
        return False
        
    print(f"Estimator initialized with first layer type: {type(estimator.first_layer_observer)}")
    
    # 3. Verify Dynamics Model Identity
    # The neural estimator has 'self.dynamics'
    # The first layer observer (QLPVObsKalman) has 'self.dynamics'
    
    main_dynamics = estimator.dynamics
    layer1_dynamics = estimator.first_layer_observer.dynamics
    
    print(f"Main Dynamics ID: {id(main_dynamics)}")
    print(f"Layer1 Dynamics ID: {id(layer1_dynamics)}")
    
    if main_dynamics is layer1_dynamics:
        print("SUCCESS: Dynamics model instance is SHARED.")
    else:
        print("FAIL: Dynamics model instances are DIFFERENT.")
        return False
        
    # 4. Verify Parameter Consistency
    # Modify one parameter in main and check if it reflects in layer1
    # Note: modifying params directly might be dangerous if not carefully handled, but for test is OK.
    original_lf = main_dynamics.params['lf']
    main_dynamics.params['lf'] = 999.99
    
    if layer1_dynamics.params['lf'] == 999.99:
        print("SUCCESS: Parameter modification reflected in both references (Expected).")
    else:
        print("FAIL: Parameter modification NOT reflected.")
        return False
    
    # Restore
    main_dynamics.params['lf'] = original_lf
    
    return True

if __name__ == "__main__":
    try:
        if test_shared_dynamics_model():
            print("\nTest Passed!")
            sys.exit(0)
        else:
            print("\nTest Failed!")
            sys.exit(1)
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
