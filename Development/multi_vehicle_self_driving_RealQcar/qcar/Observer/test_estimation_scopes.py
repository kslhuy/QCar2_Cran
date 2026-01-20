"""
Test script for Estimation Scopes

This script demonstrates the estimation scopes visualization system
using mock data without requiring actual QCar hardware.

Run from the qcar directory:
    python -m Observer.test_estimation_scopes
"""

import numpy as np
import time
import sys

# Try to import the scope manager
try:
    from Observer.estimation_scopes import (
        EstimationScopeManager,
        LocalStatePreset,
        LocalEstimationErrorPreset, 
        LocalControlPreset,
        FleetStatePreset,
        FleetPositionPreset,
        FleetConsensusPreset,
        ScopeDataPlayer,
        MULTISCOPE_AVAILABLE
    )
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure you run this from the qcar directory.")
    sys.exit(1)


def generate_mock_local_data(t: float) -> dict:
    """Generate mock local estimator data."""
    # Circular motion
    radius = 1.0
    omega = 0.5  # rad/s
    
    x = radius * np.cos(omega * t)
    y = radius * np.sin(omega * t)
    theta = omega * t + np.pi/2
    velocity = 0.5 + 0.1 * np.sin(t)
    
    # Add some noise for GPS
    noise = 0.05
    x_gps = x + np.random.randn() * noise
    y_gps = y + np.random.randn() * noise
    theta_gps = theta + np.random.randn() * noise * 0.5
    
    return {
        'x': x,
        'y': y,
        'theta': theta,
        'velocity': velocity,
        'acceleration': 0.1 * np.cos(t),
        'x_gps': x_gps,
        'y_gps': y_gps,
        'theta_gps': theta_gps,
        'v_ref': 0.5,
        'steering': 0.1 * np.sin(t),
        'throttle': 0.05 + 0.02 * np.sin(t),
    }


def generate_mock_fleet_data(t: float, num_vehicles: int = 3) -> dict:
    """Generate mock fleet estimator data."""
    fleet_states = np.zeros((5, num_vehicles))
    
    for i in range(num_vehicles):
        offset = i * 2 * np.pi / num_vehicles
        radius = 0.5 + i * 0.3
        omega = 0.3 + i * 0.1
        
        fleet_states[0, i] = radius * np.cos(omega * t + offset)  # x
        fleet_states[1, i] = radius * np.sin(omega * t + offset)  # y
        fleet_states[2, i] = omega * t + offset + np.pi/2  # theta
        fleet_states[3, i] = 0.3 + 0.1 * np.sin(t + offset)  # velocity
        fleet_states[4, i] = 0.05 * np.cos(t + offset)  # acceleration
    
    trust_scores = {i: 0.8 + 0.2 * np.sin(t + i) for i in range(num_vehicles)}
    
    return {
        'fleet_states': fleet_states,
        'consensus_error': 0.1 + 0.05 * np.abs(np.sin(t)),
        'trust_scores': trust_scores,
    }


def test_local_scopes():
    """Test local estimator scopes."""
    print("\n" + "="*60)
    print("Testing Local Estimator Scopes")
    print("="*60)
    
    if not MULTISCOPE_AVAILABLE:
        print("MultiScope not available. Skipping visualization test.")
        print("Testing data generation only...")
        
        for i in range(10):
            t = i * 0.1
            data = generate_mock_local_data(t)
            print(f"t={t:.1f}: x={data['x']:.3f}, y={data['y']:.3f}, v={data['velocity']:.3f}")
        
        print("Local data generation test passed!")
        return True
    
    # Create scope manager with local presets
    mgr = EstimationScopeManager.create_default_local_scopes(fps=30, time_window=30.0)
    # Use manual mode for main thread GUI updates
    mgr.start(threaded=False)
    
    print("Running for 10 seconds... (Close window to exit)")
    
    try:
        t0 = time.time()
        while time.time() - t0 < 10.0:
            t = time.time() - t0
            data = generate_mock_local_data(t)
            mgr.sample(t, data)
            mgr.update() # Manual update
            time.sleep(0.01)  # 100 Hz sample rate
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        mgr.stop()
    
    print("Local scope test completed!")
    return True


def test_fleet_scopes():
    """Test fleet estimator scopes."""
    print("\n" + "="*60)
    print("Testing Fleet Estimator Scopes")
    print("="*60)
    
    if not MULTISCOPE_AVAILABLE:
        print("MultiScope not available. Skipping visualization test.")
        print("Testing data generation only...")
        
        for i in range(10):
            t = i * 0.1
            data = generate_mock_fleet_data(t, num_vehicles=3)
            print(f"t={t:.1f}: vehicles in fleet: {data['fleet_states'].shape[1]}")
        
        print("Fleet data generation test passed!")
        return True
    
    # Create scope manager with fleet presets
    mgr = EstimationScopeManager.create_default_fleet_scopes(fps=30, time_window=30.0, max_vehicles=3)
    # Use manual mode for main thread GUI updates
    mgr.start(threaded=False)
    
    print("Running for 10 seconds... (Close window to exit)")
    
    try:
        t0 = time.time()
        while time.time() - t0 < 10.0:
            t = time.time() - t0
            data = generate_mock_fleet_data(t, num_vehicles=3)
            mgr.sample(t, data)
            mgr.update() # Manual update
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        mgr.stop()
    
    print("Fleet scope test completed!")
    return True


def test_all_scopes():
    """Test all scopes together."""
    print("\n" + "="*60)
    print("Testing All Scopes Together")
    print("="*60)
    
    if not MULTISCOPE_AVAILABLE:
        print("MultiScope not available. Testing data merging only...")
        
        for i in range(10):
            t = i * 0.1
            local_data = generate_mock_local_data(t)
            fleet_data = generate_mock_fleet_data(t, num_vehicles=3)
            
            # Merge data
            combined = {**local_data, **fleet_data}
            print(f"t={t:.1f}: {len(combined)} data keys")
        
        print("Data merging test passed!")
        return True
    
    # Create scope manager with all presets
    mgr = EstimationScopeManager.create_all_scopes(fps=30, time_window=30.0, max_vehicles=3)
    # Use manual mode for main thread GUI updates
    mgr.start(threaded=False)
    
    print(f"Active presets: {list(mgr.presets.keys())}")
    print("Running for 15 seconds... (Close windows to exit)")
    
    try:
        t0 = time.time()
        while time.time() - t0 < 15.0:
            t = time.time() - t0
            
            # Merge local and fleet data
            local_data = generate_mock_local_data(t)
            fleet_data = generate_mock_fleet_data(t, num_vehicles=3)
            combined = {**local_data, **fleet_data}
            
            mgr.sample(t, combined)
            mgr.update() # Manual update
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        mgr.stop()
    
    print("All scopes test completed!")
    return True


def test_recording():
    """Test data recording functionality."""
    print("\n" + "="*60)
    print("Testing Recording Functionality")
    print("="*60)
    
    mgr = EstimationScopeManager(fps=30, time_window=30.0)
    
    # Start recording even without MultiScope
    filepath = mgr.start_recording()
    print(f"Recording to: {filepath}")
    
    mgr._running = True  # Enable sampling to recorder
    
    for i in range(100):
        t = i * 0.01
        data = generate_mock_local_data(t)
        mgr.sample(t, data)
    
    mgr.stop_recording()
    mgr._running = False
    
    # Check file exists
    import os
    if os.path.exists(filepath):
        print(f"Recording file created successfully!")
        file_size = os.path.getsize(filepath)
        print(f"File size: {file_size} bytes")
        return True
    else:
        print("Recording file not found!")
        return False


def test_playback_local():
    """Test local data playback functionality."""
    print("\n" + "="*60)
    print("Testing Local Playback Functionality")
    print("="*60)
    
    if not MULTISCOPE_AVAILABLE:
        print("MultiScope not available. Skipping playback test.")
        return True

    # 1. Create a recording with LOCAL data only
    print("Step 1: Creating a recording with local data...")
    mgr = EstimationScopeManager(fps=30, time_window=10.0)
    
    # Define local-only columns
    local_columns = ['x', 'y', 'theta', 'velocity', 'acceleration',
                     'x_gps', 'y_gps', 'theta_gps', 'steering', 'throttle', 'v_ref']
    filepath = mgr.start_recording(columns=local_columns, name='local')
    mgr._running = True
    
    # Generate 5 seconds of LOCAL data
    for i in range(150):  # 150 samples @ ~33ms = ~5 seconds
        t = i * 0.033
        local_data = generate_mock_local_data(t)
        mgr.sample(t, local_data)
    
    mgr.stop_recording()
    mgr._running = False
    
    print(f"Recorded to {filepath}")
    
    # 2. Play it back with LOCAL scopes only
    print("Step 2: Playing back with local scopes...")
    
    play_mgr = EstimationScopeManager.create_default_local_scopes(fps=30, time_window=10.0)
    play_mgr.start(threaded=False)
    
    print(f"Active presets: {list(play_mgr.presets.keys())}")
    
    player = ScopeDataPlayer(filepath)
    if player.load():
        player.play(play_mgr, speed=0.9) 
    
    play_mgr.stop()
    print("Local playback completed!")
    return True


def test_playback_fleet():
    """Test fleet data playback functionality."""
    print("\n" + "="*60)
    print("Testing Fleet Playback Functionality")
    print("="*60)
    
    if not MULTISCOPE_AVAILABLE:
        print("MultiScope not available. Skipping playback test.")
        return True

    num_vehicles = 3
    
    # 1. Create a recording with FLEET data only
    print("Step 1: Creating a recording with fleet data...")
    mgr = EstimationScopeManager(fps=30, time_window=10.0)
    
    # Define fleet-only columns (flattened)
    state_names = ['x', 'y', 'theta', 'v', 'a']
    fleet_columns = ['consensus_error']
    for v_idx in range(num_vehicles):
        for s_name in state_names:
            fleet_columns.append(f"fleet_{s_name}_{v_idx}")
    for v_idx in range(num_vehicles):
        fleet_columns.append(f"trust_{v_idx}")
    
    filepath = mgr.start_recording(columns=fleet_columns, max_vehicles=num_vehicles, name='fleet')
    mgr._running = True
    
    # Generate 5 seconds of FLEET data
    for i in range(150):  # 150 samples @ ~33ms = ~5 seconds
        t = i * 0.033
        fleet_data = generate_mock_fleet_data(t, num_vehicles=num_vehicles)
        mgr.sample(t, fleet_data)
    
    mgr.stop_recording()
    mgr._running = False
    
    print(f"Recorded to {filepath}")
    
    # 2. Play it back with FLEET scopes only
    print("Step 2: Playing back with fleet scopes...")
    
    play_mgr = EstimationScopeManager.create_default_fleet_scopes(
        fps=30, 
        time_window=10.0, 
        max_vehicles=num_vehicles
    )
    play_mgr.start(threaded=False)
    
    print(f"Active presets: {list(play_mgr.presets.keys())}")
    
    player = ScopeDataPlayer(filepath)
    if player.load(max_vehicles=num_vehicles):
        player.play(play_mgr, speed=0.9) 
    
    play_mgr.stop()
    print("Fleet playback completed!")
    return True


if __name__ == "__main__":
    print("="*60)
    print("Estimation Scopes Test Suite")
    print("="*60)
    
    if not MULTISCOPE_AVAILABLE:
        print("\nWARNING: MultiScope not available!")
        print("Running in headless mode (data generation tests only)")
    
    # Run tests
    results = {}
    
    # results['recording'] = test_recording()
    results['playback_local'] = test_playback_local()
    results['playback_fleet'] = test_playback_fleet()
    # results['local'] = test_local_scopes()
    # results['fleet'] = test_fleet_scopes()
    # results['all'] = test_all_scopes()
    
    # Summary
    print("\n" + "="*60)
    print("Test Results Summary")
    print("="*60)
    for name, passed in results.items():
        status = "PASSED" if passed else "FAILED"
        print(f"  {name}: {status}")
    
    all_passed = all(results.values())
    print("\n" + ("All tests passed!" if all_passed else "Some tests failed!"))
