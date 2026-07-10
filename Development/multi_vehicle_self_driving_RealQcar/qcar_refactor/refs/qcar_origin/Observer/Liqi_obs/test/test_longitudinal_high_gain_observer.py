from ..longitudinal_high_gain_observer import LongitudinalHighGainFleetStateEstimator
import unittest
import numpy as np


class TestLongitudinalHighGainFleetStateEstimator(unittest.TestCase):
    def setUp(self):
        config = {
            'consensus_gain': 1.0,
            'direct_gain0': 10.0,
            'L0': [[1], [2], [1]],
            'alpha0': 0.1,
            'beta0': 0.2,
            'laplacian_matrix': np.array([[0, -1], [-1, 0]]),
        }
        self.estimator1 = LongitudinalHighGainFleetStateEstimator(
            vehicle_id=0,
            fleet_size=2,
            state_dim=3,
            config = config
        )
        self.estimator2 = LongitudinalHighGainFleetStateEstimator(
            vehicle_id=1,
            fleet_size=2,
            state_dim=3,
            config = config
        )

    def test_update(self):
        local_state = np.array([0.0, 0.0, 0.0])
        dt = 0.1
        current_time_ns = 100000000
        control = np.array([1.0])
        updated_state = self.estimator.update(local_state, dt, current_time_ns, control)
        print("Updated state:", updated_state)
        self.assertEqual(updated_state.shape, (3,))

    def test_control_input(self):
        local_state = np.array([0.0, 0.0, 0.0])
        dt = 0.1
        current_time_ns = 100000000
        control = np.array([1.0])
        updated_state = self.estimator.update(local_state, dt, current_time_ns, control)
        self.assertEqual(control.shape, (1,))