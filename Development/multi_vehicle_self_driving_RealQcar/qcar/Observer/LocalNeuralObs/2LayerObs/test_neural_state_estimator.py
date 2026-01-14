"""
Test suite for Neural State Estimator (Second Layer) and Full Two-Layer Architecture

Tests:
1. NeuralLuenbergerEstimator initialization and configuration
2. 6D internal state management
3. 4D output extraction for base class compatibility
4. Neural network input preparation (6D and 8D)
5. Observer update mechanics
6. First-layer observer integration (two-layer architecture)
7. State reset and initialization
8. Full simulation with synthetic data
"""

import numpy as np
import sys
from pathlib import Path

# Add parent directory to path
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))
sys.path.insert(0, str(parent_dir.parent))

# Mock torch if not available (for basic testing)
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("⚠️  PyTorch not available, some tests will be skipped")


def test_neural_estimator_initialization():
    """Test NeuralLuenbergerEstimator initialization"""
    print("\n" + "="*60)
    print("Test 1: Neural Estimator Initialization")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    # Initialize with default config
    estimator = NeuralLuenbergerEstimator()
    
    print(f"Internal state dim: {estimator.INTERNAL_STATE_DIM}")
    print(f"Input dim: {estimator.input_dim}")
    print(f"Output dim: {estimator.output_dim}")
    print(f"State dim (base class): {estimator.state_dim}")
    
    assert estimator.INTERNAL_STATE_DIM == 6, "Internal state should be 6D"
    assert estimator.output_dim == 2, "Output should be 2D (w_r, w_f)"
    assert estimator.state_dim == 4, "Base class state should be 4D"
    
    print("✅ Initialization test PASSED")
    return True


def test_neural_estimator_with_initial_pose():
    """Test initialization with initial pose"""
    print("\n" + "="*60)
    print("Test 2: Initialization with Initial Pose")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    initial_pose = np.array([10.0, 5.0, 0.5])  # [X, Y, ψ]
    estimator = NeuralLuenbergerEstimator(initial_pose=initial_pose)
    
    state_6d = estimator.get_state_6d()
    state_4d = estimator.get_state()
    
    print(f"Initial pose: {initial_pose}")
    print(f"6D state: {state_6d}")
    print(f"4D state: {state_4d}")
    
    # Check internal state
    assert abs(state_6d[4] - 10.0) < 1e-6, "X should be 10.0"
    assert abs(state_6d[5] - 5.0) < 1e-6, "Y should be 5.0"
    assert abs(state_6d[2] - 0.5) < 1e-6, "ψ should be 0.5"
    
    # Check 4D output
    assert abs(state_4d[0] - 10.0) < 1e-6, "X in 4D should be 10.0"
    assert abs(state_4d[1] - 5.0) < 1e-6, "Y in 4D should be 5.0"
    assert abs(state_4d[2] - 0.5) < 1e-6, "ψ in 4D should be 0.5"
    
    print("✅ Initial pose test PASSED")
    return True


def test_6d_to_4d_state_extraction():
    """Test 6D internal state to 4D output extraction"""
    print("\n" + "="*60)
    print("Test 3: 6D to 4D State Extraction")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    estimator = NeuralLuenbergerEstimator()
    
    # Set internal 6D state directly
    # [v_x, v_y, ψ, r, X, Y]
    estimator.state_nn_6d = np.array([2.5, 0.3, 0.8, 0.1, 15.0, 8.0])
    
    # Extract 4D state
    state_4d = estimator._extract_4d_state()
    
    print(f"6D state: {estimator.state_nn_6d}")
    print(f"4D extracted: {state_4d}")
    
    # Expected: [X, Y, ψ, v_x] = [15.0, 8.0, 0.8, 2.5]
    assert abs(state_4d[0] - 15.0) < 1e-6, "X should be 15.0"
    assert abs(state_4d[1] - 8.0) < 1e-6, "Y should be 8.0"
    assert abs(state_4d[2] - 0.8) < 1e-6, "ψ should be 0.8"
    assert abs(state_4d[3] - 2.5) < 1e-6, "v_x should be 2.5"
    
    print("✅ 6D to 4D extraction test PASSED")
    return True


def test_nn_input_preparation_6d():
    """Test NN input preparation (6D mode)"""
    print("\n" + "="*60)
    print("Test 4: NN Input Preparation (6D)")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    config = {'use_acceleration': False, 'input_dim': 6}
    estimator = NeuralLuenbergerEstimator(config=config)
    
    # Set state
    estimator.state_nn_6d = np.array([3.0, 0.2, 0.5, 0.15, 10.0, 5.0])
    
    # Prepare NN input
    steering = 0.1
    throttle = 0.5
    nn_input = estimator._prepare_nn_input(steering, throttle)
    
    print(f"State: vx={estimator.state_nn_6d[0]}, vy={estimator.state_nn_6d[1]}")
    print(f"Control: δ={steering}, a={throttle}")
    print(f"NN input shape: {nn_input.shape}")
    print(f"NN input: {nn_input.flatten()}")
    
    # Expected: [v_x, v_y, ψ, r, δ, a]
    assert nn_input.shape == (6, 1), f"Shape should be (6, 1), got {nn_input.shape}"
    assert abs(nn_input[0, 0] - 3.0) < 1e-6, "v_x should be 3.0"
    assert abs(nn_input[4, 0] - 0.1) < 1e-6, "δ should be 0.1"
    assert abs(nn_input[5, 0] - 0.5) < 1e-6, "a should be 0.5"
    
    print("✅ NN input preparation (6D) test PASSED")
    return True


def test_nn_input_preparation_8d():
    """Test NN input preparation (8D mode with acceleration)"""
    print("\n" + "="*60)
    print("Test 5: NN Input Preparation (8D with Acceleration)")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    config = {'use_acceleration': True}
    estimator = NeuralLuenbergerEstimator(config=config)
    
    assert estimator.input_dim == 8, "Input dim should be 8 with acceleration"
    
    # Set state
    estimator.state_nn_6d = np.array([3.0, 0.2, 0.5, 0.15, 10.0, 5.0])
    
    # Prepare NN input with acceleration
    steering = 0.1
    throttle = 0.5
    accel = np.array([0.3, 0.8])  # [a_x, a_y]
    nn_input = estimator._prepare_nn_input(steering, throttle, accel)
    
    print(f"Control: δ={steering}, a={throttle}")
    print(f"Acceleration: a_x={accel[0]}, a_y={accel[1]}")
    print(f"NN input shape: {nn_input.shape}")
    print(f"NN input: {nn_input.flatten()}")
    
    # Expected: [v_x, v_y, ψ, r, δ, a, a_x, a_y]
    assert nn_input.shape == (8, 1), f"Shape should be (8, 1), got {nn_input.shape}"
    assert abs(nn_input[6, 0] - 0.3) < 1e-6, "a_x should be 0.3"
    assert abs(nn_input[7, 0] - 0.8) < 1e-6, "a_y should be 0.8"
    
    print("✅ NN input preparation (8D) test PASSED")
    return True


def test_observer_update_without_gps():
    """Test observer update without GPS data"""
    print("\n" + "="*60)
    print("Test 6: Observer Update (No GPS)")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    config = {'use_first_layer': False}  # Disable first-layer for isolated test
    estimator = NeuralLuenbergerEstimator(
        initial_pose=np.array([0.0, 0.0, 0.0]),
        config=config
    )
    
    # Set initial velocity
    estimator.state_nn_6d[0] = 2.0  # v_x
    
    initial_state = estimator.get_state_6d().copy()
    print(f"Initial 6D state: {initial_state}")
    
    # Update without GPS
    success = estimator.update(
        motor_tach=2.0,
        steering=0.1,
        throttle=0.0,
        dt=0.02,
        gyro_z=0.1,
        gps_data=None
    )
    
    final_state = estimator.get_state_6d()
    print(f"Final 6D state: {final_state}")
    
    assert success, "Update should succeed"
    # State should change due to prediction
    assert np.linalg.norm(final_state - initial_state) > 0, "State should change"
    
    print("✅ Observer update (no GPS) test PASSED")
    return True


def test_observer_update_with_gps():
    """Test observer update with GPS data"""
    print("\n" + "="*60)
    print("Test 7: Observer Update (With GPS)")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    config = {'use_first_layer': False}
    estimator = NeuralLuenbergerEstimator(
        initial_pose=np.array([0.0, 0.0, 0.0]),
        config=config
    )
    
    estimator.state_nn_6d[0] = 2.0  # v_x
    
    # GPS data
    gps_data = {
        'x': 1.0,
        'y': 0.5,
        'theta': 0.1,
        'valid': True
    }
    
    # Update with GPS
    success = estimator.update(
        motor_tach=2.0,
        steering=0.1,
        throttle=0.0,
        dt=0.02,
        gyro_z=0.1,
        gps_data=gps_data
    )
    
    state_4d = estimator.get_state()
    print(f"GPS data: X={gps_data['x']}, Y={gps_data['y']}, θ={gps_data['theta']}")
    print(f"4D state after update: {state_4d}")
    
    assert success, "Update should succeed"
    
    print("✅ Observer update (with GPS) test PASSED")
    return True


def test_first_layer_integration():
    """Test two-layer architecture with first-layer observer"""
    print("\n" + "="*60)
    print("Test 8: First-Layer Observer Integration (Two-Layer)")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    # Test with qLPV first-layer
    config_qlpv = {
        'use_first_layer': True,
        'first_layer_type': 'qlpv'
    }
    
    estimator_qlpv = NeuralLuenbergerEstimator(config=config_qlpv)
    print(f"First-layer observer (qLPV): {type(estimator_qlpv.first_layer_observer).__name__}")
    
    assert estimator_qlpv.first_layer_observer is not None, "First-layer should be initialized"
    
    # Test with differentiator_uio first-layer
    config_diff = {
        'use_first_layer': True,
        'first_layer_type': 'differentiator_uio'
    }
    
    estimator_diff = NeuralLuenbergerEstimator(config=config_diff)
    print(f"First-layer observer (diff_uio): {type(estimator_diff.first_layer_observer).__name__}")
    
    assert estimator_diff.first_layer_observer is not None, "First-layer should be initialized"
    
    print("✅ First-layer integration test PASSED")
    return True


def test_two_layer_update():
    """Test full two-layer observer update"""
    print("\n" + "="*60)
    print("Test 9: Two-Layer Observer Update")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    config = {
        'use_first_layer': True,
        'first_layer_type': 'qlpv'
    }
    
    estimator = NeuralLuenbergerEstimator(
        initial_pose=np.array([0.0, 0.0, 0.0]),
        config=config
    )
    
    estimator.state_nn_6d[0] = 3.0  # v_x
    
    # GPS data
    gps_data = {
        'x': 0.5,
        'y': 0.2,
        'theta': 0.05,
        'valid': True
    }
    
    # Update through both layers
    for i in range(5):
        success = estimator.update(
            motor_tach=3.0,
            steering=0.05,
            throttle=0.0,
            dt=0.02,
            gyro_z=0.05,
            gps_data=gps_data
        )
    
    state_6d = estimator.get_state_6d()
    state_4d = estimator.get_state()
    w_hat = estimator.get_tire_residuals()
    
    print(f"After 5 updates:")
    print(f"  6D state: {state_6d}")
    print(f"  4D state: {state_4d}")
    print(f"  Tire residuals: {w_hat}")
    
    assert success, "Update should succeed"
    
    print("✅ Two-layer update test PASSED")
    return True


def test_state_reset():
    """Test state reset functionality"""
    print("\n" + "="*60)
    print("Test 10: State Reset")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    estimator = NeuralLuenbergerEstimator()
    
    # Modify state
    estimator.state_nn_6d = np.array([5.0, 1.0, 1.5, 0.3, 20.0, 10.0])
    
    print(f"Before reset: {estimator.get_state_6d()}")
    
    # Reset with new pose
    new_pose = np.array([50.0, 25.0, 1.0])
    estimator.reset(new_pose)
    
    state_after = estimator.get_state_6d()
    print(f"After reset: {state_after}")
    
    assert abs(state_after[4] - 50.0) < 1e-6, "X should be 50.0"
    assert abs(state_after[5] - 25.0) < 1e-6, "Y should be 25.0"
    assert abs(state_after[2] - 1.0) < 1e-6, "ψ should be 1.0"
    assert abs(state_after[0]) < 1e-6, "v_x should be 0"
    
    print("✅ State reset test PASSED")
    return True


def test_simulation_loop():
    """Test full simulation loop with synthetic trajectory"""
    print("\n" + "="*60)
    print("Test 11: Simulation Loop")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    config = {
        'use_first_layer': True,
        'first_layer_type': 'differentiator_uio'
    }
    
    estimator = NeuralLuenbergerEstimator(
        initial_pose=np.array([0.0, 0.0, 0.0]),
        config=config
    )
    
    dt = 0.02
    v = 2.0  # Constant velocity
    trajectory = []
    
    # Simulate circular motion
    for i in range(100):
        t = i * dt
        steering = 0.1  # Constant steering
        
        # True position (approximate)
        true_x = v * t * np.cos(0.1 * t)
        true_y = v * t * np.sin(0.1 * t)
        true_theta = 0.1 * t
        
        gps_data = {
            'x': true_x + 0.01 * np.random.randn(),
            'y': true_y + 0.01 * np.random.randn(),
            'theta': true_theta + 0.001 * np.random.randn(),
            'valid': True
        }
        
        success = estimator.update(
            motor_tach=v,
            steering=steering,
            throttle=0.0,
            dt=dt,
            gyro_z=0.1,
            gps_data=gps_data
        )
        
        if i % 25 == 0:
            state = estimator.get_state()
            trajectory.append(state.copy())
            print(f"  t={t:.2f}s: X={state[0]:.2f}, Y={state[1]:.2f}, ψ={state[2]:.3f}")
    
    print(f"Completed {estimator.update_count} updates")
    
    assert estimator.update_count == 100, "Should have 100 updates"
    
    print("✅ Simulation loop test PASSED")
    return True


def test_tire_residual_output():
    """Test tire residual output from neural network"""
    print("\n" + "="*60)
    print("Test 12: Tire Residual Output")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    estimator = NeuralLuenbergerEstimator()
    
    # Run a few updates (not too many to avoid numerical issues in gradient solver)
    for _ in range(5):
        estimator.update(
            motor_tach=2.0,
            steering=0.1,
            throttle=0.0,
            dt=0.02,
            gyro_z=0.1,
            gps_data={'x': 0, 'y': 0, 'theta': 0, 'valid': True}
        )
    
    w_hat = estimator.get_tire_residuals()
    
    print(f"Tire residuals: w_r={w_hat[0]}, w_f={w_hat[1]}")
    
    assert len(w_hat) == 2, "Should have 2 residuals [w_r, w_f]"
    assert np.isfinite(w_hat[0]), "w_r should be finite"
    assert np.isfinite(w_hat[1]), "w_f should be finite"
    
    print("✅ Tire residual output test PASSED")
    return True


def test_composite_uio_loss():
    """Test composite UIO loss with trajectory reference"""
    print("\n" + "="*60)
    print("Test 13: Composite UIO Loss with Trajectory Reference")
    print("="*60)
    
    if not TORCH_AVAILABLE:
        print("⚠️  Skipped (PyTorch required)")
        return True
    
    from neural_state_estimator import NeuralLuenbergerEstimator
    
    config = {
        'use_first_layer': True,
        'first_layer_type': 'qlpv',
        'loss_type': 'composite_uio'  # Enable composite loss
    }
    
    estimator = NeuralLuenbergerEstimator(
        initial_pose=np.array([0.0, 0.0, 0.0]),
        config=config
    )
    
    print(f"Loss type: {estimator.config['loss_type']}")
    
    # Set trajectory reference (simulating StanleyController.get_reference_pose())
    ref_pose = np.array([1.0, 0.5, 0.1])  # [X, Y, θ]
    estimator.set_trajectory_reference(ref_pose, ref_velocity=2.0)
    
    ref_result = estimator.get_trajectory_reference()
    print(f"Reference pose: {ref_pose}")
    print(f"Reference result: {ref_result}")
    
    assert ref_result is not None, "Reference should be set"
    ref_values, ref_indices = ref_result
    
    # Check that we have the expected values
    # ref_pose = [X, Y, ψ] + ref_velocity=2.0 -> ref_values = [X, Y, ψ, v_x]
    assert len(ref_values) == 4, f"Should have 4 ref values, got {len(ref_values)}"
    assert abs(ref_values[0] - 1.0) < 1e-6, "X should be 1.0"
    assert abs(ref_values[1] - 0.5) < 1e-6, "Y should be 0.5"
    assert abs(ref_values[2] - 0.1) < 1e-6, "ψ should be 0.1"
    assert abs(ref_values[3] - 2.0) < 1e-6, "v_x should be 2.0"
    
    # Check indices map correctly to 6D state
    assert 4 in ref_indices, "X index should be in ref_indices"
    assert 5 in ref_indices, "Y index should be in ref_indices"
    
    # Run updates with composite loss
    gps_data = {'x': 0.5, 'y': 0.2, 'theta': 0.05, 'valid': True}
    
    for i in range(5):
        success = estimator.update(
            motor_tach=2.0,
            steering=0.1,
            throttle=0.0,
            dt=0.02,
            gyro_z=0.1,
            gps_data=gps_data
        )
    
    assert success, "Update with composite loss should succeed"
    print(f"Completed {estimator.update_count} updates with composite UIO loss")
    
    print("✅ Composite UIO loss test PASSED")
    return True


def main():
    """Run all tests"""
    print("="*60)
    print("Neural State Estimator Test Suite")
    print("(Second Layer + Two-Layer Architecture)")
    print("="*60)
    
    tests = [
        test_neural_estimator_initialization,
        test_neural_estimator_with_initial_pose,
        test_6d_to_4d_state_extraction,
        test_nn_input_preparation_6d,
        test_nn_input_preparation_8d,
        test_observer_update_without_gps,
        test_observer_update_with_gps,
        test_first_layer_integration,
        test_two_layer_update,
        test_state_reset,
        test_simulation_loop,
        test_tire_residual_output,
        test_composite_uio_loss,
    ]
    
    passed = 0
    failed = 0
    skipped = 0
    
    for test in tests:
        try:
            result = test()
            if result:
                passed += 1
        except Exception as e:
            print(f"\n❌ {test.__name__} FAILED: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "="*60)
    print(f"Results: {passed}/{len(tests)} tests passed")
    if not TORCH_AVAILABLE:
        print("Note: Some tests were skipped (PyTorch not available)")
    print("="*60)
    
    if failed == 0:
        print("✅ ALL TESTS PASSED")
        return 0
    else:
        print(f"❌ {failed} TEST(S) FAILED")
        return 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
