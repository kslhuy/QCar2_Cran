"""Analytic and failure-contract checks for the estimator/control boundaries."""
import threading
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from config_io import load_yaml_mapping
from Observer.fleet_state_estimators import ConsensusFleetEstimator, _state_dict_to_array
from Observer.KalmaNet.Robust.innovation_trust import InnovationTrustFilter
from Observer.local_state_estimators import EKFStateEstimator
from Observer.relative_state_estimators import SA_ACC_UIO_Estimator
from Observer.TrustbasedDistributedObserver.contamination_rollback import ContaminationRollback
from Observer.TrustbasedDistributedObserver.external_measurement_cache import ExternalMeasurementCache
from Observer.TrustbasedDistributedObserver.estimator_config import normalize_vehicle_model_config
from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import TrustBasedFleetEstimator
from Observer.TrustbasedDistributedObserver.weight_trust_module import WeightConfig, WeightTrustModule
from Observer.VehicleObserverSimple import VehicleObserver
from electronics.trust_observer_native import TrustObserverShadow
from simulation.observer_pipeline import run_pipeline
from simulation.relative_sensor import MockRelativeSensor


class FlowMathTests(unittest.TestCase):
    now = 1_800_000_000_000_000_000

    @staticmethod
    def packet(x=2., velocity=1.):
        return dict(x=x, y=0., theta=0., velocity=velocity, acceleration=0.)

    def estimator(self, **observer):
        instance = TrustBasedFleetEstimator(0, 3, config={
            "observer": {"dynamics_prediction_mode": "dead_reckoning",
                         "enable_output_low_pass": False, "rollback_enabled": False,
                         **observer}, "logging": {"enabled": False}})
        self.addCleanup(instance.close)
        return instance

    def test_joseph_covariance_for_reduced_and_zero_gains(self):
        for trust in (0., .35, 1.):
            with self.subTest(trust=trust):
                core = InnovationTrustFilter(config={"longitudinal_model": "simple_acceleration"})
                core.predict_trust = lambda features: np.full(4, trust)
                prior = core.covariance + core.process
                gain = prior / (prior + core.noise**2) * trust
                expected = (1 - gain)**2 * prior + gain**2 * core.noise**2
                core.update(0., 0., 0., .02, gps_data=dict(valid=True, x=0., y=0., theta=0.))
                np.testing.assert_allclose(core.covariance, expected, rtol=1e-13)

    def test_nonfinite_initial_local_state_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            InnovationTrustFilter([np.nan, 0., 0.])

    def test_current_measurements_do_not_get_an_extra_prediction_step(self):
        estimator = self.estimator()
        estimator.add_received_local_state(1, self.packet(), self.now)
        first = estimator.update(np.zeros(5), .1, self.now, np.zeros(2))
        self.assertAlmostEqual(first[0, 1], 2.)
        estimator.add_received_local_state(1, self.packet(2.1), self.now + 100_000_000)
        second = estimator.update(np.zeros(5), .1, self.now + 100_000_000, np.zeros(2))
        self.assertAlmostEqual(second[0, 1], 2.1, places=12)

    def test_controller_read_propagates_without_advancing_cached_state(self):
        estimator = self.estimator()
        estimator.add_received_local_state(1, self.packet(), self.now)
        estimator.update(np.zeros(5), .1, self.now, np.zeros(2))
        predicted = estimator.get_target_state_at_time(1, self.now + 30_000_000)
        self.assertAlmostEqual(predicted[0], 2.03)
        self.assertEqual(estimator.fleet_states[0, 1], 2.)
        self.assertEqual(estimator._target_state_time_ns[1], self.now)

    def test_estimate_timestamp_is_distinct_from_packet_send_time(self):
        estimator = self.estimator()
        data = dict(self.packet(), estimate_timestamp_ns=self.now - 100_000_000,
                    control_input={"steering": 0., "throttle": .1})
        estimator.add_received_local_state(1, data, self.now)
        self.assertEqual(estimator.received_local_states[1][-1][0], self.now - 100_000_000)
        self.assertEqual(estimator._received_control_inputs[1]["timestamp_ns"], self.now)
        state = estimator.update(np.zeros(5), .1, self.now, np.zeros(2))
        self.assertAlmostEqual(state[0, 1], 2.1)

    def test_packet_validation_is_atomic_and_reordering_keeps_newest_state(self):
        estimator = self.estimator()
        estimator.add_received_local_state(1, self.packet(3.), self.now + 2)
        estimator.add_received_local_state(1, self.packet(2.), self.now + 1)
        latest = estimator._get_latest_received_state_with_timestamp(1, self.now + 2)
        self.assertEqual(latest[1][0], 3.)
        malformed = dict(self.packet(99.), control_input={"steering": 0.})
        self.assertFalse(estimator.add_received_local_state(1, malformed, self.now + 3))
        self.assertEqual(len(estimator.received_local_states[1]), 2)

    def test_future_fleet_snapshot_is_rejected_at_read_time(self):
        estimator = self.estimator()
        estimator.add_received_fleet_state(2, {1: self.packet()}, self.now + 1)
        self.assertIsNone(estimator._get_latest_fleet_data_with_timestamp(2, self.now))

    def test_external_cache_preserves_nanoseconds_and_returns_snapshots(self):
        cache = ExternalMeasurementCache(.01)
        cache.set(1, 2., 1., self.now + 1)
        self.assertIsNone(cache.get(1, self.now))
        result = cache.get(1, self.now + 1)
        self.assertEqual(result["timestamp_ns"], self.now + 1)
        result["distance"] = 99.
        self.assertEqual(cache.get(1, self.now + 1)["distance"], 2.)
        self.assertFalse(cache.set(1, 3., 0., self.now))
        self.assertIsNone(cache.get(1, self.now + 10_000_002))

    def test_external_cache_concurrent_reads_and_writes(self):
        cache = ExternalMeasurementCache(.2)
        errors = []
        def write():
            try:
                for tick in range(300):
                    cache.set(1, 2., 0., self.now + tick)
            except Exception as error:
                errors.append(error)
        worker = threading.Thread(target=write)
        worker.start()
        for _ in range(300):
            entry = cache.get(1, self.now + 300)
            if entry is not None:
                entry.clear()
        worker.join()
        self.assertEqual(errors, [])
        self.assertEqual(cache.get(1, self.now + 300)["distance"], 2.)

    def test_malformed_fleet_snapshot_cannot_poison_or_mutate_estimator(self):
        estimator = self.estimator()
        data = {1: self.packet()}
        self.assertTrue(estimator.add_received_fleet_state(2, data, self.now))
        data[1]["x"] = 100.
        self.assertEqual(estimator.received_fleet_states[2][-1][1][1]["x"], 2.)
        self.assertFalse(estimator.add_received_fleet_state(2, {1: self.packet(np.nan)}, self.now))
        self.assertIsNone(_state_dict_to_array({"x": 1.}, 5))

    def test_configured_packet_age_is_respected(self):
        estimator = TrustBasedFleetEstimator(0, 2, config={
            "fleet": {"max_state_age_ns": 10_000_000}, "logging": {"enabled": False}})
        self.addCleanup(estimator.close)
        estimator.add_received_local_state(1, self.packet(), self.now)
        self.assertIsNone(estimator._get_latest_received_state_with_timestamp(1, self.now + 10_000_001))

    def test_runtime_config_preserves_native_settings_and_history_limits(self):
        observer = VehicleObserver.__new__(VehicleObserver)
        observer.stop = lambda: None
        merged = observer._merge_trust_child_into_fleet_config({}, {
            "embedded_core": {"enabled": True, "mode": "shadow"},
            "fleet": {"max_state_age_ns": 10_000_000, "received_local_states_limit": 2}})
        observer.fleet_config_defaults = merged["fleet"]
        observer.fleet_estimator_type = "trust_consensus"
        observer.vehicle_geometry_config = {}
        observer._merge_v2v_manifest_into_fleet_config = lambda cfg: None
        cfg = observer._resolve_fleet_estimator_config()
        self.assertTrue(cfg["embedded_core"]["enabled"])
        self.assertEqual(cfg["max_state_age_ns"], 10_000_000)
        cfg["logging"] = {"enabled": False}
        cfg["embedded_core"]["enabled"] = False
        estimator = TrustBasedFleetEstimator(0, 2, config=cfg)
        self.addCleanup(estimator.close)
        for tick in range(3):
            estimator.add_received_local_state(1, self.packet(), self.now + tick)
        self.assertEqual(len(estimator.received_local_states[1]), 2)

    def test_relative_descriptor_reconstruction_and_stability(self):
        cfg = load_yaml_mapping("Observer/config_relative_estimators.yaml")["relative"]["sa_acc_uio"]
        observer = SA_ACC_UIO_Estimator(cfg)
        self.addCleanup(observer.stop_recording)
        np.testing.assert_allclose(observer.Pz @ observer.E + observer.Qz @ observer.Ce, np.eye(4), atol=1e-10)
        self.assertLess(max(abs(np.linalg.eigvals(observer.N))), 1.)
        singular = dict(cfg, k3=cfg["k1"])
        with self.assertRaisesRegex(ValueError, "rank"):
            SA_ACC_UIO_Estimator(singular)
        with self.assertRaises(ValueError):
            SA_ACC_UIO_Estimator(dict(cfg, tau=0.))

    def test_bad_model_parameters_fail_before_simulation(self):
        for model in ({"wheelbase": 0.}, {"longitudinal_model": "typo"},
                      {"velocity_lag_model": {"tau": -1.}}, {"max_velocity": np.nan},
                      {"longitudinal_model": "velocity_lag_lookup", "velocity_lag_lookup_model": {
                          "throttle_breakpoints": [0., 0.], "steady_state_velocity_breakpoints": [0., 1.]}}):
            with self.subTest(model=model), self.assertRaises(ValueError):
                normalize_vehicle_model_config(model)

    def test_range_rate_is_radial_and_signed(self):
        sensor = MockRelativeSensor(dict(latency_s=0, drop_probability=0,
            range_noise_std_m=0, range_rate_noise_std_mps=0, bearing_noise_std_rad=0), 0)
        packets = sensor.sample([0., 0., 0., 1., 0.], {
            1: (self.now, [3., 4., 0., 0., 0.])}, self.now)
        self.assertAlmostEqual(packets[0]["relative_velocity"], -.6)

    def test_explicit_weight_factors_are_preserved(self):
        cfg = WeightConfig.from_dict({"flag_w0_target_attack_factor": 1.,
                                     "flag_w0_global_est_check_factor": .3})
        self.assertEqual(cfg.flag_w0_target_attack_factor, 1.)
        self.assertEqual(cfg.flag_w0_global_est_check_factor, .3)
        with self.assertRaises(ValueError):
            WeightConfig(weight_type="typo")

    def test_randomized_consensus_weights_are_nonnegative_and_normalized(self):
        rng = np.random.default_rng(14)
        for mode in ("equal", "trust_based", "paper"):
            weights = WeightTrustModule(0, 6, config=WeightConfig(weight_type=mode))
            for _ in range(30):
                scores = {i: float(rng.random()) for i in range(1, 6)}
                neighbors = {i: {1: self.packet()} for i in range(2, 6)}
                result = weights.calculate_weights_for_target(1, scores, neighbors, np.zeros(5))
                values = [result["w0"], result["w_self"], *result["neighbors"].values()]
                self.assertTrue(np.all(np.isfinite(values)))
                self.assertGreaterEqual(min(values), 0.)
                self.assertAlmostEqual(sum(values), 1., places=12)
                if mode != "equal":  # equal is an intentionally uncapped ablation
                    self.assertTrue(all(value <= weights.config.w_cap + 1e-12 for value in result["neighbors"].values()))

    def test_rollback_uses_recorded_order_and_wrapped_heading(self):
        def predict(state, control, dt, target_id, **kwargs):
            state = state.copy()
            state[0] += dt
            return state
        replay = ContaminationRollback(5, 0, 2, predict_fn=predict)
        previous = np.array([0., 0., np.pi - .01, 1., 0.])
        direct = np.array([1., 0., -np.pi + .01, 1., 0.])
        component = {"direct": {"source": 1, "weight": .5, "state": direct},
                     "prediction": {"dt": 1., "order": "predict_then_correct"}}
        result = replay._replay_without_malicious(component, previous, set(), 1)
        self.assertAlmostEqual(result[0], 1.)
        self.assertAlmostEqual(abs(result[2]), np.pi)

    def test_selected_qcar_ekf_does_not_switch_on_initialization_error(self):
        with patch("Observer.local_state_estimators.QCarEKF", None):
            with self.assertRaisesRegex(RuntimeError, "Selected QCarEKF"):
                EKFStateEstimator(config={"use_qcar_ekf": True})
        estimator = EKFStateEstimator(config={"use_qcar_ekf": False})
        for _ in range(50):
            self.assertTrue(estimator.update(0., 0., 0., .02))
        np.testing.assert_allclose(estimator.P, estimator.P.T, atol=1e-12)
        self.assertGreaterEqual(np.linalg.eigvalsh(estimator.P).min(), 0.)

    def test_native_authority_failure_does_not_switch_to_python(self):
        shadow = TrustObserverShadow(config={"enabled": False}, trust_config={})
        self.addCleanup(shadow.close)
        shadow.mode, shadow.available = "native_authority", True
        with self.assertRaisesRegex(RuntimeError, "authority failed"):
            shadow._trip_authority("prediction", "test mismatch")
        self.assertEqual(shadow.mode, "native_authority")
        with self.assertRaises(RuntimeError):
            shadow.should_use_native(None)

    def test_reordered_sensor_packet_does_not_replace_newer_input(self):
        observer = VehicleObserver.__new__(VehicleObserver)
        observer.stop = lambda: None
        observer.lock = threading.RLock()
        observer.sensor_data = {"relative_measurements_by_target": {}}
        observer._last_relative_distance_by_target = {}
        observer.set_relative_target = lambda target: None
        observer.enable_relative = True
        observer._publish_relative_measurement(1, 2., 0., self.now, "radar", 1.)
        observer._publish_relative_measurement(1, 9., 0., self.now - 1, "radar", 1.)
        self.assertEqual(observer.sensor_data["relative_measurements_by_target"][1]["distance"], 2.)

    def test_basic_consensus_wraps_headings_and_rejects_unstable_gains(self):
        observer = ConsensusFleetEstimator(0, 2, config={"direct_gain": .5, "consensus_gain": .3})
        observer.fleet_states[2, 1] = np.pi - .01
        observer.add_received_local_state(1, dict(self.packet(), theta=-np.pi + .01), self.now)
        result = observer.update(np.zeros(5), .02, self.now, np.zeros(2))
        self.assertAlmostEqual(abs(result[2, 1]), np.pi)
        with self.assertRaises(ValueError):
            ConsensusFleetEstimator(0, 2, config={"direct_gain": .8, "consensus_gain": .8})

    def test_alternative_observer_errors_do_not_return_old_states(self):
        observer = ConsensusFleetEstimator(0, 2)
        with patch.object(observer, "_cleanup_old_data", side_effect=ValueError("test failure")):
            with self.assertRaisesRegex(RuntimeError, "consensus"):
                observer.update(np.zeros(5), .02, self.now, np.zeros(2))
        from Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
        observer = DistributedLuenbergerEstimator.__new__(DistributedLuenbergerEstimator)
        observer.stop_recording = lambda: None
        observer.consensus_estimator = None
        observer.vehicle_id, observer.fleet_size, observer.state_dim = 0, 2, 5
        observer.fleet_states, observer.logger = np.zeros((5, 2)), None
        with patch.object(observer, "_distributed_luenberger_observer_update", side_effect=ValueError("test failure")):
            with self.assertRaisesRegex(RuntimeError, "Luenberger"):
                observer.update(np.zeros(5), .02, self.now, np.zeros(2))

    def test_distributed_gain_loader_has_no_missing_file_substitution(self):
        from Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
        with tempfile.TemporaryDirectory() as folder:
            harness = SimpleNamespace(_extra_config_dir=folder, observer_size=1)
            with self.assertRaises(FileNotFoundError):
                DistributedLuenbergerEstimator._load_K_matrices_from_yaml(harness)

    def test_distributed_consensus_uses_the_senders_complete_reference_frame(self):
        from Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
        observer = DistributedLuenbergerEstimator(1, 4, config={"debug_recording": False})
        self.addCleanup(observer.close)
        observer.add_received_local_state(0, self.packet(100., 0.), self.now)
        snapshot = {vehicle: self.packet(-vehicle * .8, 0.) for vehicle in range(4)}
        observer.add_received_fleet_state(2, snapshot, self.now)
        np.testing.assert_allclose(observer._compute_consensus_term(np.zeros(9), self.now), 0., atol=1e-12)

    def test_distributed_luenberger_four_vehicle_runtime_smoke(self):
        from Observer.ShengyaObs.distributed_luenberger_estimator import DistributedLuenbergerEstimator
        observers = []
        fields = ("x", "y", "theta", "velocity", "acceleration")
        for vehicle in range(4):
            observer = DistributedLuenbergerEstimator(vehicle, 4, config={"debug_recording": False})
            self.addCleanup(observer.close)
            observers.append(observer)
        for tick in range(20):
            stamp = self.now + tick * 40_000_000
            snapshots = [{vehicle: dict(zip(fields, observer.fleet_states[:, vehicle]))
                          for vehicle in range(4)} for observer in observers]
            for host, observer in enumerate(observers):
                for sender in range(4):
                    if sender != host:
                        observer.add_received_local_state(sender, self.packet(sender * .8, 0.), stamp)
                        observer.add_received_fleet_state(sender, snapshots[sender], stamp)
                result = observer.update(np.array([host * .8, 0., 0., 0., 0.]), .04, stamp, np.zeros(2))
                self.assertTrue(np.isfinite(result).all())

    def test_estimator_drives_cacc_and_stops_when_leader_is_unavailable(self):
        result = run_pipeline(3., "relative_dropout", closed_loop=True)
        self.assertTrue(result["closed_loop"])
        self.assertGreater(result["controller_steps"], 100)
        self.assertGreater(result["missing_leader_stops"], 30)
        self.assertLessEqual(result["maximum_command"], .3 + 1e-9)
        self.assertGreater(result["minimum_center_gap_m"], .3)


if __name__ == "__main__":
    unittest.main()
