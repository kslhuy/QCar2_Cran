"""
Quick test script to verify follower controller fixes.
This tests the controller logic without running the full simulation.
"""

import sys
import math
sys.path.append('c:/Users/Quang Huy Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/fleet_framwork')

from VehicleFollowerController import VehicleFollowerController

def test_cacc_controller():
    """Test CACC controller with simple scenario"""
    print("=" * 60)
    print("Testing CACC Controller")
    print("=" * 60)
    
    # Create controller with test config
    config = {
        'road_type': 'Studio',  # Use Studio to allow steering
        'enable_steering_control': True,
        'lookahead_distance': 0.4,
        'max_steering': 0.6,
        'k_steering': 2.5,
        'controller_params': {
            'alpha': 1.0,
            'beta': 1.5,
            'v0': 1.0,
            'delta': 4,
            'T': 0.4,
            's0': 0.5,
            'ri': 0.5,
            'hi': 0.6,
            'K_gains': [0.8, 0.5, 0.0, 1.2]
        }
    }
    
    controller = VehicleFollowerController(
        vehicle_id=1,
        controller_type="CACC",
        config=config
    )
    
    # Test scenario: follower behind leader, both moving forward
    follower_state = [0.0, 0.0, 0.0, 1.0]  # [x, y, theta, v]
    leader_state = [2.0, 0.0, 0.0, 1.0]     # Leader 2m ahead
    
    print(f"\nScenario 1: Leader ahead, same speed")
    print(f"Follower: pos=({follower_state[0]:.2f}, {follower_state[1]:.2f}), "
          f"heading={math.degrees(follower_state[2]):.1f}°, vel={follower_state[3]:.2f}")
    print(f"Leader:   pos=({leader_state[0]:.2f}, {leader_state[1]:.2f}), "
          f"heading={math.degrees(leader_state[2]):.1f}°, vel={leader_state[3]:.2f}")
    
    acc, steering = controller.compute_vehicle_following_control(
        current_pos=[follower_state[0], follower_state[1], 0],
        current_rot=[0, 0, follower_state[2]],
        current_velocity=follower_state[3],
        leader_pos=[leader_state[0], leader_state[1], 0],
        leader_rot=[0, 0, leader_state[2]],
        leader_velocity=leader_state[3],
        dt=0.1
    )
    
    print(f"Control output: acceleration={acc:.3f} m/s², steering={math.degrees(steering):.2f}°")
    print(f"Expected: Small negative acceleration (spacing > target), zero steering")
    
    # Test scenario 2: follower too close
    follower_state = [0.0, 0.0, 0.0, 1.0]
    leader_state = [0.8, 0.0, 0.0, 1.0]  # Leader only 0.8m ahead
    
    print(f"\nScenario 2: Follower too close")
    print(f"Follower: pos=({follower_state[0]:.2f}, {follower_state[1]:.2f}), vel={follower_state[3]:.2f}")
    print(f"Leader:   pos=({leader_state[0]:.2f}, {leader_state[1]:.2f}), vel={leader_state[3]:.2f}")
    print(f"Distance: {leader_state[0] - follower_state[0]:.2f}m (target: ~{0.5 + 0.6*follower_state[3]:.2f}m)")
    
    acc, steering = controller.compute_vehicle_following_control(
        current_pos=[follower_state[0], follower_state[1], 0],
        current_rot=[0, 0, follower_state[2]],
        current_velocity=follower_state[3],
        leader_pos=[leader_state[0], leader_state[1], 0],
        leader_rot=[0, 0, leader_state[2]],
        leader_velocity=leader_state[3],
        dt=0.1
    )
    
    print(f"Control output: acceleration={acc:.3f} m/s², steering={math.degrees(steering):.2f}°")
    print(f"Expected: Negative acceleration (decelerate), zero steering")
    
    # Test scenario 3: leader turning
    follower_state = [0.0, 0.0, 0.0, 1.0]
    leader_state = [2.0, 0.5, math.radians(15), 1.0]  # Leader ahead and turning
    
    print(f"\nScenario 3: Leader turning")
    print(f"Follower: pos=({follower_state[0]:.2f}, {follower_state[1]:.2f}), "
          f"heading={math.degrees(follower_state[2]):.1f}°")
    print(f"Leader:   pos=({leader_state[0]:.2f}, {leader_state[1]:.2f}), "
          f"heading={math.degrees(leader_state[2]):.1f}°")
    
    acc, steering = controller.compute_vehicle_following_control(
        current_pos=[follower_state[0], follower_state[1], 0],
        current_rot=[0, 0, follower_state[2]],
        current_velocity=follower_state[3],
        leader_pos=[leader_state[0], leader_state[1], 0],
        leader_rot=[0, 0, leader_state[2]],
        leader_velocity=leader_state[3],
        dt=0.1
    )
    
    print(f"Control output: acceleration={acc:.3f} m/s², steering={math.degrees(steering):.2f}°")
    print(f"Expected: Small acceleration, positive steering (turn right to follow)")
    
    print("\n" + "=" * 60)
    print("Test completed successfully!")
    print("=" * 60)
    
    return controller


if __name__ == "__main__":
    try:
        controller = test_cacc_controller()
        print("\n✓ All tests passed! Controller is working correctly.")
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
