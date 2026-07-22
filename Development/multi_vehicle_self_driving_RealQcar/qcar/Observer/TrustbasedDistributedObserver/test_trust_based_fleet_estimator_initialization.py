import os
import sys
import unittest
from unittest.mock import patch

import numpy as np


QCAR_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
if QCAR_ROOT not in sys.path:
    sys.path.insert(0, QCAR_ROOT)

from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (  # noqa: E402
    TrustBasedFleetEstimator,
)


class TargetInitializationTests(unittest.TestCase):
    def setUp(self):
        logger_patch = patch(
            "Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator."
            "TrustWeightLogger.start",
            return_value="",
        )
        logger_patch.start()
        self.addCleanup(logger_patch.stop)

        self.estimator = TrustBasedFleetEstimator(
            vehicle_id=1,
            fleet_size=2,
            config={
                "trust": {"max_message_age_s": 1.0},
                "weight": {
                    "startup_fixed_duration_s": 1.0,
                    "include_target_self_fleet_estimate": True,
                },
                "observer": {
                    "attack_mitigation": False,
                    "dynamics_prediction_mode": "none",
                    "enable_output_low_pass": False,
                    "rollback_enabled": False,
                },
                "timestamp_alignment": {"enabled": False},
            },
            logger=None,
        )
        self.addCleanup(self.estimator.trust_weight_logger.stop)
        self.current_time_ns = 1_000_000_000
        self.host_state = np.array([2.0, -1.0, 0.2, 0.5, 0.0])

    @staticmethod
    def _state_dict(state):
        return {
            "x": float(state[0]),
            "y": float(state[1]),
            "theta": float(state[2]),
            "velocity": float(state[3]),
            "acceleration": float(state[4]),
        }

    def _update(self, current_time_ns=None):
        if current_time_ns is None:
            current_time_ns = self.current_time_ns
        return self.estimator.update(
            local_state=self.host_state,
            dt=0.1,
            current_time_ns=current_time_ns,
            control=np.zeros(2, dtype=float),
        )

    def test_first_fresh_direct_packet_seeds_target_exactly(self):
        target_state = np.array([-1.064, -0.673, -0.6942, 0.4, 0.0])
        self.estimator.add_received_local_state(
            sender_id=0,
            state=self._state_dict(target_state),
            timestamp_ns=self.current_time_ns,
        )

        fleet_states = self._update()

        self.assertTrue(self.estimator.is_target_initialized(0))
        np.testing.assert_allclose(fleet_states[:, 0], target_state, atol=1e-12)

    def test_first_estimate_is_independent_of_initial_neighbor_availability(self):
        target_state = np.array([-1.064, -0.673, -0.6942, 0.4, 0.0])
        target_state_dict = self._state_dict(target_state)
        self.estimator.add_received_local_state(
            sender_id=0,
            state=target_state_dict,
            timestamp_ns=self.current_time_ns,
        )
        direct_only = self._update()
        direct_only_weights = self.estimator._startup_target_weights_cache[0]

        self.estimator.reset()
        self.estimator.add_received_local_state(
            sender_id=0,
            state=target_state_dict,
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.add_received_fleet_state(
            sender_id=0,
            fleet_estimates={0: target_state_dict},
            timestamp_ns=self.current_time_ns,
        )
        with_neighbor = self._update()
        neighbor_weights = self.estimator._startup_target_weights_cache[0]

        self.assertEqual(direct_only_weights["neighbors"], {})
        self.assertIn(0, neighbor_weights["neighbors"])
        np.testing.assert_allclose(direct_only[:, 0], target_state, atol=1e-12)
        np.testing.assert_allclose(with_neighbor[:, 0], target_state, atol=1e-12)

    def test_target_stays_uninitialized_without_fresh_direct_packet(self):
        stale_state = np.array([8.0, 7.0, 0.3, 0.2, 0.0])
        self.estimator.add_received_local_state(
            sender_id=0,
            state=self._state_dict(stale_state),
            timestamp_ns=self.current_time_ns - 2_000_000_000,
        )

        fleet_states = self._update()

        self.assertFalse(self.estimator.is_target_initialized(0))
        np.testing.assert_allclose(fleet_states[:, 0], np.zeros(5), atol=1e-12)

    def test_nonfinite_direct_packet_does_not_initialize_target(self):
        invalid_state = np.array([np.nan, -0.5, 0.1, 0.2, 0.0])
        self.estimator.add_received_local_state(
            sender_id=0,
            state=self._state_dict(invalid_state),
            timestamp_ns=self.current_time_ns,
        )

        fleet_states = self._update()

        self.assertFalse(self.estimator.is_target_initialized(0))
        np.testing.assert_allclose(fleet_states[:, 0], np.zeros(5), atol=1e-12)

    def test_clean_reference_alone_does_not_seed_control_path(self):
        clean_state = np.array([-1.0, 2.0, 0.3, 0.2, 0.0])
        self.estimator.add_received_clean_local_state(
            sender_id=0,
            state=self._state_dict(clean_state),
            timestamp_ns=self.current_time_ns,
        )

        fleet_states = self._update()

        self.assertFalse(self.estimator.is_target_initialized(0))
        np.testing.assert_allclose(fleet_states[:, 0], np.zeros(5), atol=1e-12)

    def test_negative_sender_id_is_not_initialized(self):
        target_state = np.array([9.0, 8.0, 0.2, 0.1, 0.0])
        self.estimator.add_received_local_state(
            sender_id=-1,
            state=self._state_dict(target_state),
            timestamp_ns=self.current_time_ns,
        )

        fleet_states = self._update()

        self.assertFalse(self.estimator.is_target_initialized(-1))
        np.testing.assert_allclose(fleet_states[:, 0], np.zeros(5), atol=1e-12)

    def test_zero_origin_is_a_valid_initialized_target_state(self):
        origin_state = np.zeros(5, dtype=float)
        self.estimator.add_received_local_state(
            sender_id=0,
            state=self._state_dict(origin_state),
            timestamp_ns=self.current_time_ns,
        )

        fleet_states = self._update()

        self.assertTrue(self.estimator.is_target_initialized(0))
        np.testing.assert_allclose(fleet_states[:, 0], origin_state, atol=1e-12)

    def test_reset_clears_initialization_and_allows_reseed(self):
        first_state = np.array([1.0, 2.0, 0.1, 0.3, 0.0])
        second_state = np.array([-3.0, 4.0, -0.2, 0.1, 0.0])
        self.estimator.add_received_local_state(
            sender_id=0,
            state=self._state_dict(first_state),
            timestamp_ns=self.current_time_ns,
        )
        self._update()
        self.assertTrue(self.estimator.is_target_initialized(0))

        self.estimator.reset()

        self.assertFalse(self.estimator.is_target_initialized(0))
        np.testing.assert_allclose(
            self.estimator.fleet_states[:, 0], np.zeros(5), atol=1e-12
        )

        reseed_time_ns = self.current_time_ns + 1_000_000_000
        self.estimator.add_received_local_state(
            sender_id=0,
            state=self._state_dict(second_state),
            timestamp_ns=reseed_time_ns,
        )
        fleet_states = self._update(reseed_time_ns)

        self.assertTrue(self.estimator.is_target_initialized(0))
        np.testing.assert_allclose(fleet_states[:, 0], second_state, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
