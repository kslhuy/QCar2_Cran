"""
Test script for modular longitudinal controllers
Demonstrates easy switching between different controllers
"""
import numpy as np
import sys
import os

# Add parent directory to path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from Controller.longitudinal_controllers import ControllerFactory


def test_controller(controller_type: str, follower_state: dict, leader_state: dict, dt: float = 0.005):
    """Test a specific controller type"""
    print(f"\n{'='*60}")
    print(f"Testing {controller_type.upper()} Controller")
    print(f"{'='*60}")
    
    # Create controller
    if controller_type == 'cacc':
        params = {
            's0': 1.5,
            'h': 0.5,
            'K': np.array([[0.2, 0.05]]),
            'acc_to_throttle_gain': 0.5,
            'max_throttle': 0.3
        }
    elif controller_type == 'pi':
        params = {
            'kp': 0.1,
            'ki': 1.0,
            'max_throttle': 0.3
        }
    elif controller_type == 'hybrid':
        params = {
            'cacc_params': {
                's0': 1.5,
                'h': 0.5,
                'K': np.array([[0.2, 0.05]]),
                'acc_to_throttle_gain': 0.5,
                'max_throttle': 0.3
            },
            'pi_params': {
                'kp': 0.1,
                'ki': 1.0,
                'max_throttle': 0.3
            }
        }
    else:
        print(f"Unknown controller type: {controller_type}")
        return
    
    controller = ControllerFactory.create(controller_type, params)
    
    # Test scenario: follower trying to catch up to leader
    print(f"\nInitial State:")
    print(f"  Follower: x={follower_state['x']:.2f}m, v={follower_state['velocity']:.2f}m/s")
    print(f"  Leader:   x={leader_state['x']:.2f}m, v={leader_state['velocity']:.2f}m/s")
    print(f"  Spacing:  {leader_state['x'] - follower_state['x']:.2f}m")
    
    # Simulate for a few steps
    print(f"\nSimulation Results (5 steps):")
    print(f"{'Step':<6} {'Spacing':<10} {'Follower_V':<12} {'Leader_V':<10} {'Throttle':<10}")
    print(f"{'-'*60}")
    
    for i in range(5):
        # Compute throttle
        throttle = controller.compute_throttle(follower_state, leader_state, dt)
        
        # Simple physics update (for demonstration)
        spacing = leader_state['x'] - follower_state['x']
        
        print(f"{i:<6} {spacing:<10.3f} {follower_state['velocity']:<12.3f} "
              f"{leader_state['velocity']:<10.3f} {throttle:<10.3f}")
        
        # Update follower velocity (simplified physics)
        acc = throttle * 2.0  # Simplified acceleration model
        follower_state['velocity'] += acc * dt
        follower_state['velocity'] = max(0, follower_state['velocity'])  # No negative velocity
        
        # Update positions
        follower_state['x'] += follower_state['velocity'] * dt
        leader_state['x'] += leader_state['velocity'] * dt
    
    # Reset for next test
    controller.reset()
    print(f"\nController reset complete")


def main():
    """Run tests for all controller types"""
    print("\n" + "="*60)
    print("MODULAR LONGITUDINAL CONTROLLER TEST")
    print("="*60)
    
    # Define test scenario
    follower_state = {
        'x': 0.0,
        'y': 0.0,
        'theta': 0.0,
        'velocity': 0.3,
        'target_velocity': 0.5
    }
    
    leader_state = {
        'x': 3.0,  # 3m ahead
        'y': 0.0,
        'theta': 0.0,
        'velocity': 0.5
    }
    
    # Test each controller type
    for controller_type in ['pi', 'cacc', 'hybrid']:
        # Reset states for each test
        test_follower = follower_state.copy()
        test_leader = leader_state.copy()
        
        test_controller(controller_type, test_follower, test_leader)
    
    # Demonstrate easy switching
    print(f"\n{'='*60}")
    print("DEMONSTRATION: Easy Controller Switching")
    print(f"{'='*60}")
    print("\nTo switch controllers, simply change ONE line:")
    print("\n  # In following_leader_state.py")
    print("  self.controller_type = 'cacc'  # ← Change this to 'pi' or 'hybrid'")
    print("\nThat's it! The system will automatically use the new controller.")
    
    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print("\nNext steps:")
    print("1. Choose your preferred controller type")
    print("2. Update the config or code with your choice")
    print("3. Tune parameters for your specific application")
    print("4. Run real tests and compare performance")
    print("\nSee README_CONTROLLERS.md for detailed documentation.")


if __name__ == '__main__':
    main()
