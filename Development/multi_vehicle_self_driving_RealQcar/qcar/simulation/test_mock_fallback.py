import sys
import os
import time
import numpy as np

# Adjust path to find the qcar package
import pathlib
current_file = pathlib.Path(__file__).resolve()
# We expect the structure: .../Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/test_mock_fallback.py
# So project root is 2 levels up from here (qcar/simulation/ -> qcar/ -> root)
project_root = current_file.parents[2] 
qcar_path = project_root / 'qcar'

if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

if str(qcar_path) not in sys.path:
    sys.path.insert(0, str(qcar_path))

from qcar.simulation.mock_vehicle import MockQCar

def test_kinematic_fallback():
    print("Testing Kinematic Fallback...")
    
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
            'gps': {} # Add minimal config if needed
        }
    }
    
    # Instantiate car
    try:
        car = MockQCar(config)
    except Exception as e:
        print(f"Failed to init car: {e}")
        # Make dummy config for gps if it failed
        config['sensors']['gps'] = {'noise_std': 0.0}
        car = MockQCar(config)

    # 1. Test Start from Stop
    print("\n[Test 1] Start from Stop")
    car.velocity = 0.0
    car.state_obs[0] = 0.0
    
    # Apply throttle
    car.write(throttle=0.1, steering=0.0)
    for _ in range(10):
        car.read() # Simulation step
        
    print(f"Velocity after acceleration: {car.velocity:.4f}")
    if car.velocity <= 0.0:
        print("FAIL: Car did not accelerate")
    else:
        print("PASS: Car accelerated")
        
    # 2. Test Stop from Motion (Coasting/Braking)
    print("\n[Test 2] Stop from Motion (Throttle = 0)")
    # Set velocity to something small but moving
    car.state_obs[0] = 0.2 
    car.velocity = 0.2
    
    car.write(throttle=0.0, steering=0.0)
    
    print(f"Initial Velocity: {car.velocity:.4f}")
    
    # Run simulation for a few seconds
    steps = 50 # 50 * ~0.02s = 1s
    for i in range(steps):
        car.read()
        if i % 10 == 0:
            print(f"Step {i}: v={car.velocity:.4f}, vx_obs={car.state_obs[0]:.4f}")
            
    print(f"Final Velocity: {car.velocity:.4f}")
    
    if abs(car.velocity) < 0.001:
        print("PASS: Car stopped completely")
    else:
        print("FAIL: Car did not stop completely")

    # 3. Test Low Speed Input
    print("\n[Test 3] Low Speed Input (Throttle small but non-zero)")
    car.write(throttle=0.015, steering=0.0) # Just above threshold
    for _ in range(10):
         car.read()
         
    print(f"Velocity with small input: {car.velocity:.4f}")
    if car.velocity > 0:
         print("PASS: Car moves with small input")
    else:
         print("FAIL: Car should move")

if __name__ == "__main__":
    test_kinematic_fallback()
