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


if __name__ == "__main__":
    unittest.main()
