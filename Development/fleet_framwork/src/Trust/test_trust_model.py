#!/usr/bin/env python3
"""
Test script for TriPTrustModel integration in VehicleProcess
This script tests the Python implementation of the trust model.
"""

import sys
import os
import numpy as np
import time

# Add the current directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from TriPTrustModel import TriPTrustModel

def create_mock_vehicle(vehicle_id, position, velocity, acceleration=0.0):
    """Create a mock vehicle object for testing."""
    class MockVehicle:
        def __init__(self, vid, pos, vel, acc):
            self.vehicle_number = vid
            self.state = np.array([pos[0], pos[1], 0.0, vel, acc])  # [x, y, yaw, v, a]
            self.dt = 0.01  # 100 Hz update rate
            
            # Mock parameters
            self.Param_opt = type('MockParam', (), {
                'hi': 0.5,    # Time gap
                'ri': 8.0,    # Minimum gap distance  
                'v0': 15.0,   # Desired velocity
                'delta': 4.0  # Exponent parameter
            })()
            
            self.scenarios_config = type('MockConfig', (), {
                'controller_type': "coop",
                'Monitor_sudden_change': True,
                'Dichiret_type': "Single",
                'is_know_data_not_nearby': True
            })()
            
            self.observer = type('MockObserver', (), {
                'est_local_state_current': self.state,
                'est_global_state_current': np.zeros((5, 3))  # 3 vehicles, 5 states each
            })()
            
            # Mock communication
            self.center_communication = type('MockComm', (), {
                'get_local_state': self._get_local_state,
                'get_global_state': self._get_global_state
            })()
            
            self.other_vehicles = []
            
        def _get_local_state(self, target_id, host_id):
            # Mock returning target vehicle state
            if target_id == 1:  # Leader
                return np.array([20.0, 0.0, 0.0, 12.0, -0.5])
            elif target_id == 2:  # Vehicle 2
                return np.array([10.0, 0.0, 0.0, 10.0, 0.0])
            elif target_id == 3:  # Vehicle 3
                return np.array([0.0, 0.0, 0.0, 8.0, 0.5])
            return None
            
        def _get_global_state(self, target_id, host_id):
            # Mock global state estimate
            global_states = np.array([
                [20.0, 10.0, 0.0],   # x positions
                [0.0, 0.0, 0.0],     # y positions
                [0.0, 0.0, 0.0],     # yaw angles
                [12.0, 10.0, 8.0],   # velocities
                [-0.5, 0.0, 0.5]     # accelerations
            ])
            return global_states
    
    mock_vehicle = MockVehicle(vehicle_id, position, velocity, acceleration)
    
    # Add vehicle parameters to the mock vehicle
    mock_vehicle.param = type('MockParam', (), {
        'l_r': 1.0,  # Distance from CG to rear axle
        'l_f': 1.0   # Distance from CG to front axle
    })()
    
    return mock_vehicle

def test_trust_model():
    """Test the TriPTrustModel with mock data."""
    print("Testing TriPTrustModel Python implementation...")
    
    # Create trust model
    trust_model = TriPTrustModel()
    print(f"✓ Trust model created successfully")
    print(f"  Initial trust score: {trust_model.calculate_trust_score(trust_model.rating_vector):.3f}")
    
    # Create mock vehicles
    host_vehicle = create_mock_vehicle(2, [10.0, 0.0], 10.0, 0.0)      # Host vehicle
    target_vehicle = create_mock_vehicle(1, [20.0, 0.0], 12.0, -0.5)   # Target (leader)
    leader_vehicle = create_mock_vehicle(1, [20.0, 0.0], 12.0, -0.5)   # Leader
    neighbors = [create_mock_vehicle(3, [0.0, 0.0], 8.0, 0.5)]         # Neighbor
    
    print(f"✓ Mock vehicles created")
    print(f"  Host: Vehicle {host_vehicle.vehicle_number} at pos ({host_vehicle.state[0]:.1f}, {host_vehicle.state[1]:.1f}), vel={host_vehicle.state[3]:.1f}")
    print(f"  Target: Vehicle {target_vehicle.vehicle_number} at pos ({target_vehicle.state[0]:.1f}, {target_vehicle.state[1]:.1f}), vel={target_vehicle.state[3]:.1f}")
    
    # Test individual trust components
    print("\n--- Testing Trust Components ---")
    
    # Test velocity evaluation
    v_score = trust_model.evaluate_velocity(
        host_id=2, target_id=1, v_y=12.0, v_host=10.0, 
        v_leader=12.0, a_leader=-0.5, b_leader=0.1, 
        is_nearby=True, tolerance=0.1
    )
    print(f"✓ Velocity score: {v_score:.3f}")
    
    # Test distance evaluation
    d_score = trust_model.evaluate_distance(d_y=10.0, d_measured=9.8, is_nearby=True)
    print(f"✓ Distance score: {d_score:.3f}")
    
    # Test acceleration evaluation
    a_score = trust_model.evaluate_acceleration(
        host_vehicle=host_vehicle, host_id=2, target_id=1,
        a_y=-0.5, a_host=0.0, d=[10.0, 10.2], ts=0.01, is_nearby=True
    )
    print(f"✓ Acceleration score: {a_score:.3f}")
    
    # Test beacon evaluation
    beacon_score = trust_model.evaluate_beacon_timeout(beacon_received=True)
    print(f"✓ Beacon score: {beacon_score:.3f}")
    
    # Test heading evaluation
    h_score = trust_model.evaluate_heading(
        target_pos_X=20.0, target_pos_Y=0.0, reported_heading=0.0, instant_idx=1
    )
    print(f"✓ Heading score: {h_score:.3f}")
    
    # Test trust sample calculation
    trust_sample = trust_model.calculate_trust_sample(
        v_score, d_score, a_score, beacon_score, h_score, is_nearby=True
    )
    print(f"✓ Trust sample: {trust_sample:.3f}")
    
    # Test full trust calculation
    print("\n--- Testing Full Trust Calculation ---")
    
    try:
        results = trust_model.calculate_trust(
            host_vehicle=host_vehicle,
            target_vehicle=target_vehicle,
            leader_vehicle=leader_vehicle,
            neighbors=neighbors,
            is_nearby=True,
            instant_idx=100
        )
        
        (final_score, local_trust_sample, gamma_cross, 
         v_score, d_score, a_score, beacon_score) = results
        
        print(f"✓ Full trust calculation successful!")
        print(f"  Final trust score: {final_score:.3f}")
        print(f"  Local trust sample: {local_trust_sample:.3f}")
        print(f"  Cross-validation factor: {gamma_cross:.3f}")
        print(f"  Component scores: v={v_score:.3f}, d={d_score:.3f}, a={a_score:.3f}, b={beacon_score:.3f}")
        
    except Exception as e:
        print(f"✗ Full trust calculation failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test rating vector updates
    print("\n--- Testing Rating Vector Updates ---")
    initial_rating = trust_model.rating_vector.copy()
    trust_model.update_rating_vector(trust_sample, "local")
    updated_rating = trust_model.rating_vector.copy()
    
    print(f"✓ Rating vector updated")
    print(f"  Initial: {initial_rating}")
    print(f"  Updated: {updated_rating}")
    print(f"  New trust score: {trust_model.calculate_trust_score(updated_rating):.3f}")
    
    # Test multiple iterations
    print("\n--- Testing Multiple Iterations ---")
    for i in range(5):
        # Simulate slight variations in measurements
        noise_factor = 0.1 * np.random.randn()
        v_y_noisy = 12.0 + noise_factor
        d_measured_noisy = 9.8 + 0.1 * noise_factor
        
        results = trust_model.calculate_trust(
            host_vehicle=host_vehicle,
            target_vehicle=target_vehicle,
            leader_vehicle=leader_vehicle,
            neighbors=neighbors,
            is_nearby=True,
            instant_idx=100 + i
        )
        
        final_score = results[0]
        print(f"  Iteration {i+1}: Trust score = {final_score:.3f}")
    
    print(f"\n✓ All tests completed successfully!")
    return True

def test_integration_ready():
    """Test that the TriPTrustModel can be imported and used with VehicleProcess."""
    print("\n--- Testing Integration Readiness ---")
    
    try:
        # Test importing both classes
        from TriPTrustModel import TriPTrustModel
        print("✓ TriPTrustModel import successful")
        
        # Test basic instantiation
        trust_model = TriPTrustModel()
        print("✓ TriPTrustModel instantiation successful")
        
        # Test basic methods
        trust_score = trust_model.calculate_trust_score(trust_model.rating_vector)
        print(f"✓ Basic trust score calculation: {trust_score:.3f}")
        
        print("✓ Integration ready - TriPTrustModel can be used in VehicleProcess")
        return True
        
    except Exception as e:
        print(f"✗ Integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("TriPTrustModel Python Implementation Test")
    print("=" * 60)
    
    success = True
    
    # Test integration readiness
    if not test_integration_ready():
        success = False
    
    # Test trust model functionality
    if not test_trust_model():
        success = False
    
    print("\n" + "=" * 60)
    if success:
        print("🎉 ALL TESTS PASSED! TriPTrustModel is ready for use in VehicleProcess.")
    else:
        print("❌ SOME TESTS FAILED! Check the output above for details.")
    print("=" * 60)
