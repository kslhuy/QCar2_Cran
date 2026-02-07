import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add module directory to path to allow import despite '1LayerObs' name
current_dir = os.getcwd() # Development/multi_vehicle_self_driving_RealQcar
module_dir = os.path.join(current_dir, 'qcar', 'Observer', 'LocalNeuralObs', '1LayerObs')
sys.path.append(module_dir)

# Also add qcar root for dependencies (like qlpv_vehicle_dynamics_obs)
# qlpv_vehicle_dynamics_obs is in Observer/LocalNeuralObs/ (parent of 1LayerObs)
sys.path.append(os.path.join(current_dir, 'qcar', 'Observer', 'LocalNeuralObs'))

try:
    from qlpv_observer_kalma import qLPVAugmentedObserver
except ImportError as e:
    print(f"Import failed: {e}")
    # try importing without package if in sys.path
    import qlpv_observer_kalma
    qLPVAugmentedObserver = qlpv_observer_kalma.qLPVAugmentedObserver

def test_wrapping_stability():
    print("Testing Observer Stability at Wrap Boundary...")
    
    # 1. Setup Observer
    dt = 0.02
    obs = qLPVAugmentedObserver(sample_time=dt, use_8d_system=False)
    
    # Manually force state to near PI
    # x = [vx, vy, psi, r, X, Y]
    # Use a safe initial state
    obs.state_augmented[:6] = np.array([1.0, 0.0, 3.10, 0.0, 0.0, 0.0])
    obs.P = np.eye(obs.augmented_dim) * 0.1 
    
    # 2. Simulate crossing the boundary
    # Vehicle moving in circle: yaw rate = 0.5 rad/s
    u = np.array([0.1, 0.0]) # delta, accel
    r_sim = 0.5
    vx_sim = 1.0
    
    states = []
    innovations = []
    kalman_gains = []
    
    for i in range(50):
        # True physics evolution
        psi_true = 3.10 + r_sim * i * dt
        # Wrap true psi (this is what the sensor sees)
        psi_meas = (psi_true + np.pi) % (2 * np.pi) - np.pi
        
        # Measurement vector: [vx, r, psi, X, Y, ay]
        meas = np.array([vx_sim, r_sim, psi_meas, 1.0, 1.0, 0.0])
        
        # Update
        est, _ = obs.update(meas, u, gps_available=True)
        
        psi_est = est[2]
        states.append(psi_est) 
        innovations.append(obs.innovation[2]) # Psi innov
        kalman_gains.append(np.linalg.norm(obs.K))
        
        print(f"Step {i}: True={psi_true:.4f}, Meas={psi_meas:.4f}, Est={psi_est:.4f}, Innov={obs.innovation[2]:.4f}")

    # Plot results
    try:
        plt.figure(figsize=(10, 8))
        plt.subplot(3,1,1)
        plt.plot(states, label='Est Psi')
        plt.axhline(np.pi, color='r', linestyle='--')
        plt.axhline(-np.pi, color='r', linestyle='--')
        plt.legend()
        plt.title('Heading Estimate')
        
        plt.subplot(3,1,2)
        plt.plot(innovations, label='Innov Psi')
        plt.legend()
        plt.title('Heading Innovation')

        plt.subplot(3,1,3)
        plt.plot(kalman_gains, label='Kalman Gain Norm')
        plt.legend()
        plt.title('Kalman Gain Norm')
        
        plt.tight_layout()
        plt.savefig('wrap_test.png')
        print("Test complete. Check wrap_test.png artifact.")
    except Exception as e:
        print(f"Plotting failed: {e}")

def test_low_speed_stability():
    print("\nTesting Low Speed Stability...")
    dt = 0.02
    obs = qLPVAugmentedObserver(sample_time=dt)
    
    # Low speed (VERY low)
    vx_low = 0.01
    obs.state_augmented[0] = vx_low
    
    meas = np.array([vx_low, 0.0, 0.0, 0.0, 0.0, 0.0]) 
    u = np.array([0.0, 0.0])
    
    max_k = 0
    
    for i in range(10):
        est, _ = obs.update(meas, u)
        k_norm = np.linalg.norm(obs.K)
        max_k = max(max_k, k_norm)
        print(f"Step {i}: vx={est[0]:.4f}, K_norm={k_norm:.2f}")
        
    print(f"Max Kalman Gain Norm: {max_k}")
    if max_k > 1e4:
        print("FAIL: Gain exploded at low speed")
    else:
        print("PASS: Gain reasonable")

if __name__ == "__main__":
    test_wrapping_stability()
    test_low_speed_stability()
