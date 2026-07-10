
import os
import sys
import numpy as np
import unittest
import time
import yaml

# Adjust path to allow imports from qcar package if run directly
current_dir = os.path.dirname(os.path.abspath(__file__))
# ShengyaObs is in Observer, which is in qcar
# Path structure: .../qcar/Observer/ShengyaObs
# We want to add the directory containing 'qcar' to sys.path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
if project_root not in sys.path:
    sys.path.append(project_root)

from qcar.Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
from qcar.Observer.fleet_state_estimators import FleetEstimatorFactory

class MockLogger:
    def __init__(self):
        self.logger = self
    
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def warning(self, msg):
        print(f"[WARN] {msg}")
        
    def error(self, msg):
        print(f"[ERROR] {msg}")
    
    def debug(self, msg):
        # print(f"[DEBUG] {msg}")
        pass
        
    def log_error(self, msg, e):
        print(f"[LOG_ERROR] {msg}: {e}")

class TestDistributedLuenbergerEstimator(unittest.TestCase):
    
    def setUp(self):
        self.vehicle_id = 2
        # car1.yaml has 9x9 matrices, implying 3 followers -> fleet_size = 4
        self.fleet_size = 4 
        self.state_dim = 5
        self.config = {
            'observer_gain': 0.15,
            'consensus_gain': 0.25,
            # 'adjacency_matrix': [[0, 1], [1, 0]] # Optional, usage depends on logic
        }
        self.logger = MockLogger()

    def test_initialization_and_matrix_shapes(self):
        """Test if A_delta and B_delta are constructed with correct shapes based on observer_size"""
        print("\n--- Testing Initialization and Matrix Shapes ---")
        estimator = DistributedLuenbergerEstimator(
            vehicle_id=self.vehicle_id,
            fleet_size=self.fleet_size,
            state_dim=self.state_dim,
            config=self.config,
            logger=self.logger
        )
        
        observer_size = self.fleet_size - 1 # 3
        
        # Check observer size
        self.assertEqual(estimator.observer_size, observer_size)
        
        # Check B_delta shape: should be (3 * observer_size, observer_size) or similar depending on B_tau
        # B_tau is (3, 1). B_delta is block diag of B_tau.
        # So shape should be (3 * observer_size, 1 * observer_size) ?? 
        # Wait, let's look at the implementation of B_delta construction:
        # B_tau is 3x1.
        # np.block([[B_tau, 0], [0, B_tau]]) -> (6, 2)
        expected_B_shape = (3 * observer_size, observer_size)
        print(f"B_delta shape: {estimator.B_delta.shape}, Expected: {expected_B_shape}")
        self.assertEqual(estimator.B_delta.shape, expected_B_shape)

        # Check A_delta shape: should be (3 * observer_size, 3 * observer_size)
        # A_h_tau is 3x3.
        # np.block([[A.., 0], [A.., A..]]) -> (3*obs, 3*obs)
        expected_A_shape = (3 * observer_size, 3 * observer_size)
        print(f"A_delta shape: {estimator.A_delta.shape}, Expected: {expected_A_shape}")
        self.assertEqual(estimator.A_delta.shape, expected_A_shape)
        
        # Check adjacency matrix shape
        self.assertEqual(estimator.adjacency_matrix.shape, (observer_size, observer_size))

    def test_config_loading_logic(self):
        """Test if config is loaded from extra_configs if available (car1.yaml exists)"""
        print("\n--- Testing Config Loading ---")
        # We know car1.yaml exists in extra_configs and likely has specific gains.
        # We will check if the gains in the estimator match what's in the file 
        # OR just check that it ran without error and logged "Loading extra config".
        
        # If car1.yaml loads, observer_gain might be a list/array, not the scalar 0.15 passed in config.
        estimator = DistributedLuenbergerEstimator(
            vehicle_id=1, # car1 should trigger file loading
            fleet_size=3,
            state_dim=5,
            config=self.config, # Base config
            logger=self.logger
        )
        
        print(f"Loaded Observer Gain Type: {type(estimator.observer_gain)}")
        print(f"Loaded Consensus Gain Type: {type(estimator.consensus_gain)}")
        
        # If loaded from yaml, it usually becomes a numpy array or list, whereas we passed a float.
        # If car1.yaml has matrices, it will be np.ndarray.
        self.assertTrue(isinstance(estimator.observer_gain, (np.ndarray, list)), "Observer gain should be array/list from yaml")
        
    def test_update_step(self):
        """Run a single update step to ensure no runtime errors"""
        print("\n--- Testing Update Step ---")
        estimator = DistributedLuenbergerEstimator(
            vehicle_id=self.vehicle_id,
            fleet_size=self.fleet_size,
            state_dim=self.state_dim, # 5: x, y, theta, v, a
            config=self.config,
            logger=self.logger
        )
        
        # Mock inputs
        local_state = np.array([10.0, 5.0, 0.1, 2.0, 0.5]) # x, y, theta, v, a
        dt = 0.1
        current_time_ns = int(time.time() * 1e9)
        control = np.array([0.1, 0.2]) # steering, throttle
        
        # Pre-populate some fleet state so it's not all zeros (optional)
        # Leader (0)
        estimator.add_received_local_state(0, {'x': 20.0, 'y': 5.0, 'velocity': 2.2, 'acceleration': 0.0, 'theta': 0.1}, current_time_ns)
        # Previous follower (if any, self.vehicle_id-1 -> 0, which is leader)
        
        # Update
        new_fleet_states = estimator.update(local_state, dt, current_time_ns, control)
        
        print("Update executed successfully.")
        print(f"New Fleet States shape: {new_fleet_states.shape}")
        self.assertEqual(new_fleet_states.shape, (5, self.fleet_size))
        
        # Check that self state is updated (it might rely on local_state or estimator)
        # DistributedLuenberger update logic usually puts estimated relative state back into fleet_states.
        # Let's just ensure it's not crashing.

if __name__ == '__main__':
    unittest.main()
