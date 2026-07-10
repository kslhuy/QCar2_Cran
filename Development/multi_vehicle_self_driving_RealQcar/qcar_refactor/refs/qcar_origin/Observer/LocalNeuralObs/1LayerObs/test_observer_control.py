
import numpy as np
import sys
import os
from pathlib import Path

# Add current directory to path
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

# Add parent directories to path for imports
sys.path.append(str(current_dir.parent.parent.parent)) # qcar
sys.path.append(str(current_dir.parent)) # LocalNeuralObs

from qlpv_observer_kalma import qLPVKalmanObserver

def test_observer_control_processing():
    print("Testing qLPVKalmanObserver Control Processing...")
    
    # Initialize observer
    observer = qLPVKalmanObserver(sample_time=0.02)
    
    # 1. Test Initialization
    print(f"Initial steering angle: {observer.current_steering_angle}")
    assert observer.current_steering_angle == 0.0, "Initial steering angle should be 0.0"
    
    # 2. Test Process Control = True
    # Input: Full right steer (1.0), Full throttle (1.0)
    # Measurement: Dummy (stopped)
    measurement = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # v_x, r, psi, X, Y, a_y
    control_raw = np.array([1.0, 1.0])
    
    # Run update
    observer.update(measurement, control_raw, process_control=True)
    
    # Check steering angle integration
    # steering_rate ~ 5.0 rad/s (depends on params, but non-zero)
    # new_angle = 0 + rate * dt
    dt = 0.02
    print(f"Steering angle after 1 step (raw input): {observer.current_steering_angle}")
    assert observer.current_steering_angle > 0.0, "Steering angle should increase with positive steering command"
    
    # Check acceleration effect (indirectly via state prediction or internal check if we could access u)
    # But we can verify steering angle is stored
    steering_step1 = observer.current_steering_angle
    
    # Run another step
    observer.update(measurement, control_raw, process_control=True)
    print(f"Steering angle after 2 steps (raw input): {observer.current_steering_angle}")
    assert observer.current_steering_angle > steering_step1, "Steering angle should continue to integrate"
    
    # 3. Test Process Control = False (Direct Physical Input)
    # Input: Steering angle = 0.5 rad, Accel = 1.0 m/s^2
    control_phys = np.array([0.5, 1.0])
    
    observer.update(measurement, control_phys, process_control=False)
    
    print(f"Steering angle after direct input: {observer.current_steering_angle}")
    assert abs(observer.current_steering_angle - 0.5) < 1e-6, "Steering angle should match direct physical input"
    
    # 4. Test Reset
    observer.reset()
    print(f"Steering angle after reset: {observer.current_steering_angle}")
    assert observer.current_steering_angle == 0.0, "Steering angle should be 0.0 after reset"
    
    print("All tests passed!")

if __name__ == "__main__":
    test_observer_control_processing()
