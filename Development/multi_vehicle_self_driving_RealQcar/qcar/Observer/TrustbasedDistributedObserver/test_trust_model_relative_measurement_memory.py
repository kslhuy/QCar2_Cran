import os
import sys
import unittest

import numpy as np


QCAR_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
if QCAR_ROOT not in sys.path:
    sys.path.insert(0, QCAR_ROOT)

from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (  # noqa: E402
    TrustBasedFleetEstimator,
)
from Observer.TrustbasedDistributedObserver.trust_model import TrustScore  # noqa: E402


class CleanV2VRelativeMeasurementTests(unittest.TestCase):
    def setUp(self):
        self.estimator = TrustBasedFleetEstimator(
            vehicle_id=1,
            fleet_size=2,
            config={"trust": {"max_message_age_s": 1.0}},
            logger=None,
        )
        self.current_time_ns = 1_000_000_000
        self.estimator.host_state = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "velocity": 1.0,
            "acceleration": 0.0,
        }
        self.estimator.fleet_states[:, 1] = np.array([0.0, 0.0, 0.0, 1.0, 0.0])

    def test_clean_v2v_relative_measurement_overrides_attacked_geometry(self):
        self.estimator.add_received_local_state(
            sender_id=0,
            state={
                "x": 100.0,
                "y": 0.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.add_received_clean_local_state(
            sender_id=0,
            state={
                "x": 5.0,
                "y": 0.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )

        trust_scores = self.estimator._update_trust_scores(self.current_time_ns)

        self.assertIn(0, trust_scores)
        trust = self.estimator.get_trust_score(0)
        self.assertIsNotNone(trust)
        self.assertEqual(trust.relative_measurement_source, "v2v_clean_local_state")
        self.assertTrue(trust.relative_distance_measurement_used)
        self.assertAlmostEqual(trust.y_local_distance, 5.0, places=6)
        self.assertAlmostEqual(trust.y_true_distance, 100.0, places=6)
        self.assertAlmostEqual(trust.y_local_rel_velocity, 0.0, places=6)

    def test_external_relative_measurement_keeps_priority(self):
        self.estimator.add_received_local_state(
            sender_id=0,
            state={
                "x": 100.0,
                "y": 0.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.add_received_clean_local_state(
            sender_id=0,
            state={
                "x": 5.0,
                "y": 0.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.set_external_relative_measurement(
            target_id=0,
            distance=7.0,
            relative_velocity=0.25,
            timestamp_ns=self.current_time_ns,
            source="yolo_relative",
            measurement_confidence=0.9,
        )

        self.estimator._update_trust_scores(self.current_time_ns)

        trust = self.estimator.get_trust_score(0)
        self.assertIsNotNone(trust)
        self.assertEqual(trust.relative_measurement_source, "yolo_relative")
        self.assertAlmostEqual(trust.y_local_distance, 7.0, places=6)
        self.assertAlmostEqual(trust.y_local_rel_velocity, 0.25, places=6)

    def test_gamma_self_uses_clean_v2v_bearing_when_range_matches(self):
        self.estimator.trust_model.config.use_relative_velocity_in_relative_trust = False
        self.estimator.trust_model.config.use_relative_bearing_in_gamma_self = True
        self.estimator.trust_model.config.gamma_self_bearing_tau2 = 0.25
        self.estimator.add_received_local_state(
            sender_id=0,
            state={
                "x": 0.0,
                "y": 5.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.add_received_clean_local_state(
            sender_id=0,
            state={
                "x": 0.0,
                "y": 5.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.fleet_states[:, 0] = np.array([5.0, 0.0, 0.0, 1.0, 0.0])

        self.estimator._update_trust_scores(self.current_time_ns)

        trust = self.estimator.get_trust_score(0)
        self.assertIsNotNone(trust)
        self.assertAlmostEqual(trust.y_local_distance, 5.0, places=6)
        self.assertLess(trust.gamma_self, 0.01)

    def test_direct_channel_recovers_from_local_trust_even_if_final_trust_is_low(self):
        self.estimator.add_received_local_state(
            sender_id=0,
            state={
                "x": 6.0,
                "y": 0.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.fleet_states[:, 0] = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
        self.estimator.trust_model.trust_scores[0] = TrustScore(
            vehicle_id=0,
            final_score=0.2,
            local_trust_sample=1.0,
            global_trust_sample=0.1,
            flag_target_attack=False,
            flag_global_est_check=True,
            flag_local_est_check=False,
        )

        updated_state, components = self.estimator._trust_weighted_update_with_components(
            target_id=0,
            current_time_ns=self.current_time_ns,
            trust_scores={0: 0.2},
            control=np.zeros(2, dtype=float),
            dt=0.1,
        )

        self.assertGreater(components["weights"]["w0"], 0.0)
        self.assertGreater(components["direct"]["weight"], 0.0)
        self.assertIsNotNone(components["direct"]["state"])
        self.assertGreater(updated_state[0], 0.0)


    def test_global_only_low_trust_does_not_trigger_rollback_quarantine(self):
        self.estimator.trust_model.trust_scores[0] = TrustScore(
            vehicle_id=0,
            final_score=0.05,
            local_trust_sample=0.98,
            global_trust_sample=0.01,
            flag_target_attack=False,
            flag_global_est_check=True,
            flag_local_est_check=False,
        )

        signals = self.estimator._build_rollback_trigger_signals({0: 0.05})

        self.assertFalse(signals[0]["trust_below_threshold"])
        self.assertFalse(signals[0]["flag_global_est_check"])
        self.assertEqual(self.estimator._get_current_malicious_vehicle_ids({0: 0.05}), set())
        self.assertFalse(
            self.estimator._has_active_attack_flags(
                self.estimator.trust_model.get_trust_score(0)
            )
        )


    def test_attack_recovery_reset_reanchors_to_clean_v2v_state(self):
        self.estimator.fleet_states[:, 0] = np.array([100.0, 0.0, 0.0, 1.0, 0.0])
        self.estimator.add_received_clean_local_state(
            sender_id=0,
            state={
                "x": 5.0,
                "y": 0.0,
                "theta": 0.0,
                "velocity": 1.0,
                "acceleration": 0.0,
            },
            timestamp_ns=self.current_time_ns,
        )
        self.estimator.trust_model.trust_scores[0] = TrustScore(
            vehicle_id=0,
            final_score=0.1,
            local_trust_sample=1.0,
            global_trust_sample=0.0,
            flag_global_est_check=True,
        )
        self.estimator.rollback.malicious_vehicles.add(0)

        self.estimator.reset_attack_recovery_state()

        self.assertAlmostEqual(self.estimator.fleet_states[0, 0], 5.0, places=6)
        self.assertEqual(self.estimator.rollback.malicious_vehicles, set())
        self.assertIsNone(self.estimator.get_trust_score(0))


if __name__ == "__main__":
    unittest.main()
