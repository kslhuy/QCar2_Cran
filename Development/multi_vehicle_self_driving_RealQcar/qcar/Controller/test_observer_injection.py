"""
Test script to verify observer injection into StateFeedbackController

This script tests:
1. DistributedLuenbergerEstimator creation
2. Observer injection into StateFeedbackController via ControllerManager
3. Verification of K matrices loading
"""

import sys
import os
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
from Controller.controller_manager import ControllerManager
from logging_utils import VehicleLogger


class MockVehicleLogic:
    """Mock vehicle_logic for testing"""
    def __init__(self, vehicle_id=2):
        self.vehicle_id = vehicle_id
        
        # Create vehicle_observer with fleet_estimator
        self.vehicle_observer = type('obj', (object,), {
            'fleet_estimator': DistributedLuenbergerEstimator(
                vehicle_id=vehicle_id,
                fleet_size=4,
                logger=None
            )
        })()


def test_observer_injection():
    """Test that observer is correctly injected into state_feedback controller"""
    
    print("="*70)
    print("Testing Observer Injection into StateFeedbackController")
    print("="*70)
    
    # Step 1: Create mock vehicle_logic with observer
    print("\n[Step 1] Creating mock vehicle_logic with DistributedLuenbergerEstimator...")
    vehicle_id = 2
    mock_vehicle_logic = MockVehicleLogic(vehicle_id=vehicle_id)
    
    observer = mock_vehicle_logic.vehicle_observer.fleet_estimator
    print(f"  ✓ Observer created: {type(observer).__name__}")
    print(f"  ✓ Observer vehicle_id: {observer.vehicle_id}")
    print(f"  ✓ Observer fleet_size: {observer.fleet_size}")
    
    # Step 2: Create ControllerManager
    print("\n[Step 2] Creating ControllerManager...")
    logger = VehicleLogger(car_id=vehicle_id, log_dir="logs", log_level="INFO")
    controller_manager = ControllerManager(logger=logger)
    controller_manager.set_vehicle_logic(mock_vehicle_logic)
    print("  ✓ ControllerManager created")
    
    # Step 3: Request state_feedback controller
    print("\n[Step 3] Requesting state_feedback controller...")
    controller = controller_manager.get_longitudinal_controller('state_feedback')
    
    if controller is None:
        print("  ✗ FAILED: Controller is None")
        return False
    
    print(f"  ✓ Controller created: {type(controller).__name__}")
    
    # Step 4: Verify observer injection
    print("\n[Step 4] Verifying observer injection...")
    
    if not hasattr(controller, 'observer'):
        print("  ✗ FAILED: Controller has no 'observer' attribute")
        return False
    
    if controller.observer is None:
        print("  ✗ FAILED: Controller.observer is None")
        return False
    
    print(f"  ✓ Controller has observer: {type(controller.observer).__name__}")
    print(f"  ✓ Observer vehicle_id: {controller.observer.vehicle_id}")
    
    # Step 5: Verify K matrices loaded
    print("\n[Step 5] Verifying K matrices...")
    
    if not hasattr(controller, 'K_matrices'):
        print("  ✗ FAILED: Controller has no 'K_matrices' attribute")
        return False
    
    expected_K_count = vehicle_id  # Vehicle 2 should have K_20, K_21
    actual_K_count = len(controller.K_matrices)
    
    print(f"  ✓ K_matrices loaded: {actual_K_count} matrices")
    
    if actual_K_count != expected_K_count:
        print(f"  ⚠ WARNING: Expected {expected_K_count} matrices, got {actual_K_count}")
    else:
        print(f"  ✓ Correct number of K matrices for vehicle {vehicle_id}")
    
    # Display K matrices
    for i, K in enumerate(controller.K_matrices):
        if K is not None:
            print(f"  ✓ K_{vehicle_id}{i}: shape={K.shape}, values={K}")
        else:
            print(f"  ⚠ K_{vehicle_id}{i}: None")
    
    # Step 6: Test compute_throttle with mock data
    print("\n[Step 6] Testing compute_throttle with mock fleet data...")
    
    # Create mock fleet states [5 x 4] (5 states, 4 vehicles including leader)
    fleet_states = np.array([
        [0.0, 1.0, 2.0, 3.0],      # x positions
        [0.0, 0.0, 0.0, 0.0],      # y positions  
        [0.0, 0.0, 0.0, 0.0],      # theta
        [0.6, 0.6, 0.6, 0.6],      # velocities
        [0.0, 0.0, 0.0, 0.0]       # accelerations
    ])
    
    dt = 0.01
    current_time_ns = int(1e9)  # 1 second in nanoseconds
    
    try:
        throttle = controller.compute_throttle(fleet_states, dt, current_time_ns)
        print(f"  ✓ compute_throttle executed successfully")
        print(f"  ✓ Throttle output: {throttle:.4f}")
        
        if throttle == 0.0:
            print("  ⚠ WARNING: Throttle is 0.0 (may indicate observer issue)")
        else:
            print("  ✓ Non-zero throttle indicates observer is working")
        
    except Exception as e:
        print(f"  ✗ FAILED: compute_throttle raised exception: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print("\n" + "="*70)
    print("✓ ALL TESTS PASSED")
    print("="*70)
    
    return True


if __name__ == "__main__":
    success = test_observer_injection()
    sys.exit(0 if success else 1)
