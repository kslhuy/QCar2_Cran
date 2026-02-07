
import sys
import os
import time
import numpy as np

# Adjust path to find the qcar package
import pathlib
current_file = pathlib.Path(__file__).resolve()
# We expect the structure: .../Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/test_tire_force_smoothing.py
project_root = current_file.parents[2] 
qcar_path = project_root / 'qcar'

if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

if str(qcar_path) not in sys.path:
    sys.path.insert(0, str(qcar_path))

from qcar.simulation.mock_vehicle import MockQCar

def test_smoothing():
    print("Testing Tire Force Smoothing...")
    
    # Configuration for qLPV Matrix model
    config = {
        'vehicle': {
            'id': 0,
            'model_type': 'qlpv_matrix',
            'tire_model': 'pacejka',
            'params_file': 'qcar'
        },
        'disturbances': {
            'mode': 'tire'
        },
        'sensors': {
            'gps': {'noise_std': 0.0}
        }
    }
    
    # Instantiate car
    try:
        car = MockQCar(config)
    except Exception as e:
        print(f"Failed to init car: {e}")
        return

    # Set parameters for test
    # Constant steering to generate potential lateral force
    steering_input = 0.5 # rad approx
    car.write(throttle=0.0, steering=steering_input) # Throttle 0, we will manually set velocity
    
    # Ramp velocity from 0.0 to 1.0
    velocities = np.linspace(0.0, 1.0, 101) # 0.0, 0.01, ..., 1.0
    
    print(f"\n{'Vx (m/s)':<10} | {'Fyf_true (N)':<15} | {'Delta Fyf':<15}")
    print("-" * 50)
    
    prev_Fyf = 0.0
    max_delta = 0.0
    
    passed = True
    
    for v in velocities:
        # Override state directly to simulate ramping up speed
        # qLPV Matrix State: [v_x, v_y, ψ, r, X, Y]
        # We need some lateral velocity/yaw rate to have slip angle -> force
        # Let's assume steady state cornering kinematics for testing:
        # beta approx 0 -> vy = 0
        # r = v / L * tan(delta)
        
        L = car.params.a + car.params.b
        r = (v / L) * np.tan(steering_input) if abs(v) > 0.001 else 0.0
        vy = 0.0 # Simplify
        
        car.state_obs = np.array([v, vy, 0.0, r, 0.0, 0.0])
        # Also update velocity property which might be used internally
        car.velocity = v
        car.angular_velocity = r
        car.lateral_velocity = vy
        car.current_steering_angle = steering_input

        # Trigger calculation (using internal method to isolate just the calc)
        # But best to call step() or just the calc function if possible?
        # mock_vehicle's _calculate_tire_info is called at end of step.
        # Let's call it directly to avoid side effects of integration
        car._calculate_tire_info(v, vy, r, steering_input)
        
        tire_info = car.get_tire_info()
        fyf = tire_info['Fyf_true']
        
        delta_fyf = abs(fyf - prev_Fyf)
        if v > 0.01: # Skip first step delta
            if delta_fyf > max_delta:
                max_delta = delta_fyf
        
        # Check specific points
        if abs(v - 0.05) < 0.001:
            if abs(fyf) > 1e-4:
                print(f"FAIL: Force at 0.05m/s should be ~0 (Kinematic), got {fyf}")
                passed = False
                
        if abs(v - 0.5) < 0.001:
             # Just print for info
             pass

        if v >= 0.08 and v <= 0.52: # Print around interest area
             print(f"{v:<10.2f} | {fyf:<15.4f} | {delta_fyf:<15.4f}")

        prev_Fyf = fyf
        
    print("-" * 50)
    print(f"Max Delta Force step: {max_delta:.4f}")
    
    if max_delta > 50.0: # Arbitrary large jump threshold (e.g. if it jumped from 0 to 200N in one step)
        print("FAIL: Large discontinuity detected")
        passed = False
    elif passed:
        print("PASS: Transitions appear smooth")
        
    if passed:
        print("\nSUCCESS: Tire force smoothing verification passed.")
    else:
        print("\nFAILURE: Tire force smoothing verification failed.")

if __name__ == "__main__":
    test_smoothing()
