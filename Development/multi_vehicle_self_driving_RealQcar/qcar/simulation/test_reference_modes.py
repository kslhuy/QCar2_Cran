"""Reference-assisted experiments stay explicit and retain their selected inputs."""
import contextlib
import io
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

from config_io import load_yaml_mapping
from Observer.VehicleObserverSimple import VehicleObserver
from Observer.TrustbasedDistributedObserver.estimator_config import ReferenceSettings
from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import TrustBasedFleetEstimator
from simulation.observer_pipeline import run_pipeline


class ReferenceModeTests(unittest.TestCase):
    now = 1_000_000_000

    def make_estimator(self, **settings):
        observer = {"dynamics_prediction_mode": "none", **settings}
        with patch("Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator.TrustWeightLogger.start", return_value=""):
            estimator = TrustBasedFleetEstimator(0, 2, config={"observer": observer})
        self.addCleanup(estimator.close)
        estimator.host_state = dict(x=0., y=0., theta=0., velocity=1., acceleration=0.)
        return estimator

    @staticmethod
    def packet(x):
        return dict(x=x, y=0., theta=0., velocity=1., acceleration=0.)

    def test_clean_v2v_can_supply_relative_trust_when_selected(self):
        estimator = self.make_estimator(relative_measurement_source="clean_v2v", require_relative_measurement=True)
        estimator.add_received_local_state(1, self.packet(100.), self.now)
        estimator.add_received_clean_local_state(1, self.packet(5.), self.now)
        estimator.fleet_states[:, 1] = [100., 0., 0., 1., 0.]
        scores = estimator._update_trust_scores(self.now)
        self.assertIn(1, scores)
        trust = estimator.get_trust_score(1)
        self.assertEqual(trust.relative_measurement_source, "v2v_clean_local_state")
        self.assertAlmostEqual(trust.y_local_distance, 5.)
        self.assertTrue(estimator.get_reference_status()["test_reference_active"])

    def test_clean_selection_does_not_switch_to_sensor_if_reference_is_missing(self):
        estimator = self.make_estimator(relative_measurement_source="clean_v2v", require_relative_measurement=True)
        estimator.add_received_local_state(1, self.packet(5.), self.now)
        estimator.set_external_relative_measurement(1, 5., 0., self.now, "radar")
        self.assertEqual(estimator._update_trust_scores(self.now), {})
        self.assertEqual(estimator.target_availability[1], "clean_v2v_reference_unavailable")

    def test_clean_prediction_uses_clean_motion_without_target_control(self):
        estimator = self.make_estimator(dynamics_prediction_mode="clean_data")
        estimator.add_received_clean_local_state(1, self.packet(5.), self.now)
        predicted = estimator._predict_dynamics(np.array([2., 0., 0., 0., 0.]), None, .1,
            target_id=1, current_time_ns=self.now)
        np.testing.assert_allclose(predicted, [2.1, 0., 0., 1., 0.])
        anchored = estimator._predict_dynamics(np.zeros(5), None, .1, target_id=1,
            current_time_ns=self.now, force_clean_pose_anchor=True)
        self.assertAlmostEqual(anchored[0], 5.0)  # clean reference already at output time
        estimator.received_clean_local_states.clear()
        with self.assertRaises(RuntimeError):
            estimator._predict_dynamics(np.zeros(5), None, .1, target_id=1, current_time_ns=self.now)

    def test_future_and_expired_clean_reference_are_unavailable(self):
        estimator = self.make_estimator(relative_measurement_source="clean_v2v", require_relative_measurement=True)
        estimator.add_received_clean_local_state(1, self.packet(5.), self.now + 1)
        self.assertIsNone(estimator._get_latest_clean_received_state_with_timestamp(1, self.now))
        estimator.add_received_local_state(1, self.packet(5.), self.now)
        estimator.add_received_clean_local_state(1, self.packet(5.), self.now)
        estimator._initialized_target_ids.add(1)
        self.assertFalse(estimator.is_target_available(1, self.now + 201_000_000))

    def test_selected_recovery_channel_is_used_without_changing_source(self):
        for source, expected in [("estimate", 100.), ("direct_v2v", 9.), ("clean_v2v", 5.)]:
            with self.subTest(source=source):
                estimator = self.make_estimator(recovery_state_source=source)
                estimator.fleet_states[:, 1] = [100., 0., 0., 1., 0.]
                estimator.add_received_local_state(1, self.packet(9.), self.now)
                estimator.add_received_clean_local_state(1, self.packet(5.), self.now)
                estimator.reset_attack_recovery_state(current_time_ns=self.now)
                self.assertAlmostEqual(estimator.fleet_states[0, 1], expected)

    def test_missing_clean_recovery_does_not_use_direct_channel(self):
        estimator = self.make_estimator(recovery_state_source="clean_v2v")
        estimator.fleet_states[:, 1] = [100., 0., 0., 1., 0.]
        estimator.add_received_local_state(1, self.packet(9.), self.now)
        estimator.reset_attack_recovery_state(current_time_ns=self.now)
        self.assertAlmostEqual(estimator.fleet_states[0, 1], 100.)
        self.assertEqual(estimator.target_availability[1], "clean_v2v_recovery_unavailable")

    def test_clean_reference_modes_run_through_actual_three_vehicle_stack(self):
        for mode in ["clean_data", "mixed_clean_data", "relative_host_anchor_mixed"]:
            with self.subTest(mode=mode):
                result = run_pipeline(1., relative_source="clean_v2v", prediction_mode=mode)
                self.assertEqual(result["relative_sources"], ["v2v_clean_local_state"])
                self.assertTrue(result["reference_inputs"]["test_reference_active"])
                self.assertLess(result["fleet_position_rmse_m"], .15)

    def make_following_state(self, source):
        from StateMachine.following_leader_state import FollowingLeaderState
        state = FollowingLeaderState.__new__(FollowingLeaderState)
        state.sensor_acc_config = dict(gap_source=source, max_distance=10., measurement_max_age_s=.2)
        state.sensor_acc_controller = Mock(compute_throttle=Mock(return_value=.12))
        manager = Mock(get_latest_local_state_raw=Mock(return_value=self.packet(5.)))
        observer = Mock(get_relative_measurement_for_control=Mock(return_value=None))
        state.vehicle_logic = SimpleNamespace(v2v_manager=manager, vehicle_observer=observer)
        return state, manager, observer

    def test_clean_host_anchor_modes_run_during_a_v2v_attack(self):
        for mode in ["mixed_clean_data", "relative_host_anchor_mixed"]:
            with self.subTest(mode=mode):
                result = run_pipeline(3., "v2v_pose_bias", relative_source="clean_v2v",
                    prediction_mode=mode, force_clean_pose_anchor=True)
                self.assertEqual(result["relative_sources"], ["v2v_clean_local_state"])
                self.assertTrue(result["reference_inputs"]["force_clean_pose_anchor"])
                self.assertTrue(any(row["flagged"] for row in result["trust_trace"]))
                self.assertLess(result["fleet_position_rmse_m"], .3)

    def test_controller_can_select_clean_gap_and_does_not_reuse_stale_sensor_gap(self):
        state, manager, observer = self.make_following_state("clean_v2v")
        follower = dict(x=0., y=0., sensor_leader_distance_filtered=99.)
        self.assertEqual(state._compute_sensor_acc_command(follower, {"vehicle_id": 1}, .01), .12)
        self.assertEqual(follower["sensor_leader_distance_filtered"], 5.)
        self.assertEqual(follower["sensor_leader_distance_source"], "clean_v2v")
        manager.get_latest_local_state_raw.assert_called_with(1, channel="clean", max_age_s=.2)
        observer.get_relative_measurement_for_control.assert_not_called()
        manager.get_latest_local_state_raw.return_value = None
        self.assertIsNone(state._compute_sensor_acc_command(follower, {"vehicle_id": 1}, .01))

    def test_independent_gap_selection_does_not_read_clean_channel(self):
        state, manager, observer = self.make_following_state("relative_sensor")
        self.assertIsNone(state._compute_sensor_acc_command({"x": 0., "y": 0.}, {"vehicle_id": 1}, .01))
        manager.get_latest_local_state_raw.assert_not_called()

    def test_v2v_snapshot_is_age_checked_and_does_not_mutate_cached_packet(self):
        from V2V.v2v_manager import V2VManager
        manager = V2VManager.__new__(V2VManager)
        manager._lock = threading.RLock()
        cached = self.packet(5.)
        manager.received_local_states_clean = {1: [(self.now, cached)]}
        packet = manager.get_latest_local_state_raw(1, "clean", .2, self.now)
        packet["x"] = 99.
        self.assertEqual(cached["x"], 5.)
        self.assertIsNone(manager.get_latest_local_state_raw(1, "clean", .2, self.now + 201_000_000))

    def test_reinitializing_observer_closes_previous_estimator(self):
        logger = SimpleNamespace(logger=Mock(), log_error=Mock(), log_warning=Mock())
        observer = VehicleObserver(0, logger=logger)
        self.addCleanup(observer.stop)
        with patch("Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator.TrustWeightLogger.start", return_value=""):
            observer.reinitialize_fleet_estimation(2, [1])
            previous = observer.fleet_estimator
            with patch.object(previous, "close", wraps=previous.close) as close:
                observer.reinitialize_fleet_estimation(2, [1])
                close.assert_called_once()
            self.assertIsNot(observer.fleet_estimator, previous)
            self.assertFalse(previous.embedded_core_shadow.available)

    def test_reference_source_typo_is_an_error(self):
        with self.assertRaises(ValueError):
            ReferenceSettings.from_config({"relative_measurement_source": "clena_v2v"})


class ConfigurationCacheTests(unittest.TestCase):
    def test_cached_configuration_is_independent_and_reloads_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text("values: [1]\n", encoding="utf-8")
            first = load_yaml_mapping(path)
            first["values"].append(2)
            self.assertEqual(load_yaml_mapping(path), {"values": [1]})
            revision = path.stat().st_mtime_ns
            path.write_text("values: [3]\n", encoding="utf-8")
            os.utime(path, ns=(revision + 1_000_000_000, revision + 1_000_000_000))
            self.assertEqual(load_yaml_mapping(path), {"values": [3]})

    def test_configuration_loader_rejects_non_mapping_and_python_tags(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text("[1, 2, 3]", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_yaml_mapping(path)
            path.write_text("!!python/object/apply:builtins.print ['must not execute']", encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stdout(output), self.assertRaises(yaml.YAMLError):
                load_yaml_mapping(path)
            self.assertEqual(output.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
