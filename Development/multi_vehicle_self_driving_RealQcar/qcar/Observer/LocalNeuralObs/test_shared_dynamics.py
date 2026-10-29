
# Preserve direct-file launches while using the canonical package internally.
if __package__ != "qcar.Observer.LocalNeuralObs":
    import importlib.util as _import_util
    import sys as _sys
    from pathlib import Path as _Path
    _root = _Path(__file__).resolve().parents[2]
    if "qcar" not in _sys.modules:
        _spec = _import_util.spec_from_file_location("qcar", _root / "__init__.py", submodule_search_locations=[str(_root)])
        _pkg = _import_util.module_from_spec(_spec)
        _sys.modules["qcar"] = _pkg
        _spec.loader.exec_module(_pkg)
    __package__ = "qcar.Observer.LocalNeuralObs"


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

try:
    # Try importing using sys.path manipulation since '2LayerObs' is not a valid python identifier for direct import
    
    from qcar.Observer.LocalNeuralObs import _layer_2LayerObs_neural_state_estimator_NeuralLuenbergerEstimator as NeuralLuenbergerEstimator
    from qcar.Observer.LocalNeuralObs import _layer_1LayerObs_qlpv_observer_kalma_qLPVKalmanObserver as qLPVKalmanObserver
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
        raise AssertionError("First layer observer was not initialized")
        
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
        raise AssertionError("The neural and first-layer observers must share their dynamics model")
        
    # 4. Verify Parameter Consistency
    # Modify one parameter in main and check if it reflects in layer1
    # Note: modifying params directly might be dangerous if not carefully handled, but for test is OK.
    original_lf = main_dynamics.params['lf']
    main_dynamics.params['lf'] = 999.99
    
    if layer1_dynamics.params['lf'] == 999.99:
        print("SUCCESS: Parameter modification reflected in both references (Expected).")
    else:
        print("FAIL: Parameter modification NOT reflected.")
        raise AssertionError("The shared model did not expose the same parameters")
    
    # Restore
    main_dynamics.params['lf'] = original_lf
    

if __name__ == "__main__":
    try:
        test_shared_dynamics_model()
        print("\nTest Passed!")
        sys.exit(0)
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
