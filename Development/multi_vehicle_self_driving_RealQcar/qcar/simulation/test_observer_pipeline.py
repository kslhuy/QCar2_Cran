"""Integration and failure-path tests for the configured observer chain."""
import socket
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

from Observer.VehicleObserverSimple import VehicleObserver
from Observer.fleet_state_estimators import FleetEstimatorFactory
from Observer.relative_state_estimators import SA_ACC_UIO_Estimator
from Observer.TrustbasedDistributedObserver.external_measurement_cache import ExternalMeasurementCache
from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import TrustBasedFleetEstimator
from simulation.observer_pipeline import run_pipeline
from simulation.relative_sensor import LoopbackRelativeSensor, MockRelativeSensor
from simulation.mock_sensors import MockQCarGPS


class ObserverPipelineTests(unittest.TestCase):
    def test_three_vehicle_chain_uses_only_uio_sensor_evidence(self):
        result = run_pipeline(3.0)
        self.assertEqual(result["local_backends"], ["innovation_trust"] * 3)
        self.assertEqual(result["uio_targets"], [[1, 2], [0, 2], [0, 1]])
        self.assertEqual(result["relative_sources"], ["relative_uio:simulated_radar"])
        self.assertGreater(result["available_samples"], 1700)
        self.assertLess(result["local_position_rmse_m"], 0.1)
        self.assertLess(result["fleet_position_rmse_m"], 0.15)

    def test_relative_dropout_withholds_targets_and_recovers(self):
        result = run_pipeline(3.0, "relative_dropout")
        self.assertGreater(result["unavailable_samples"], 100)
        self.assertGreater(result["available_samples"], 1000)
        self.assertEqual(result["relative_sources"], ["relative_uio:simulated_radar"])

    def test_v2v_bias_is_detected_without_clean_reference(self):
        result = run_pipeline(5.0, "v2v_pose_bias")
        attack = [row for row in result["trust_trace"] if 1.6 < row["time"] < 3.2]
        self.assertTrue(any(row["flagged"] for row in attack))
        self.assertLess(min(row["local"] for row in attack), 0.1)

    def test_sensor_models_latency_fov_range_and_expiry(self):
        sensor = MockRelativeSensor(dict(rate_hz=20, latency_s=0.03, drop_probability=0,
            field_of_view_deg=90, max_range_m=5, range_noise_std_m=0,
            range_rate_noise_std_mps=0, bearing_noise_std_rad=0), 0)
        host = [0, 0, 0, 1, 0]
        peers = {1: (1_000_000_000, [3, 0, 0, 2, 0]),
                 2: (1_000_000_000, [-3, 0, 0, 0, 0]),
                 3: (1_000_000_000, [6, 0, 0, 0, 0])}
        self.assertEqual(sensor.sample(host, peers, 1_000_000_000), [])
        output = sensor.sample(host, peers, 1_030_000_000)
        self.assertEqual([row["target_id"] for row in output], [1])
        self.assertEqual(output[0]["timestamp_ns"], 1_000_000_000)
        self.assertEqual(output[0]["distance"], 3.0)
        self.assertEqual(output[0]["relative_velocity"], 1.0)
        self.assertEqual(sensor.sample(host, peers, 2_000_000_000), [])

    def test_loopback_sensor_connects_separate_endpoints(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocator:
            allocator.bind(("127.0.0.1", 0))
            base_port = allocator.getsockname()[1]
        config = dict(world_base_port=base_port, vehicle_ids=[0, 1], latency_s=0,
                      drop_probability=0, range_noise_std_m=0, range_rate_noise_std_mps=0)
        first = LoopbackRelativeSensor(0, config)
        self.addCleanup(first.close)
        second = LoopbackRelativeSensor(1, config)
        self.addCleanup(second.close)
        a = SimpleNamespace(x=0., y=0., heading=0., velocity=0.5, lateral_velocity=0.)
        b = SimpleNamespace(x=2., y=0., heading=0., velocity=0.5, lateral_velocity=0.)
        now = time.time_ns()
        received = []
        for step in range(4):
            stamp = now + step * 60_000_000
            first.poll(a, stamp)
            received.extend(second.poll(b, stamp))
        self.assertTrue(received)
        self.assertEqual(received[-1]["source"], "simulated_radar")
        self.assertAlmostEqual(received[-1]["relative_velocity"], 0., places=6)

    def test_future_and_stale_cache_entries_are_unavailable(self):
        cache = ExternalMeasurementCache(0.2)
        cache.set(1, 2., 0., timestamp_ns=1_000_000_000)
        self.assertIsNone(cache.get(1, 999_000_000))
        self.assertIsNone(cache.get(1, 1_201_000_000))

    def test_strict_trust_never_substitutes_clean_channel(self):
        with patch("Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator.TrustWeightLogger.start", return_value=""):
            estimator = TrustBasedFleetEstimator(0, 2, config={
                "require_relative_measurement": True, "dynamics_prediction_mode": "none"})
        self.addCleanup(estimator.embedded_core_shadow.close)
        self.addCleanup(estimator.trust_weight_logger.stop)
        state = dict(x=2., y=0., theta=0., velocity=0.5, acceleration=0.)
        estimator.add_received_local_state(1, state, 1_000_000_000)
        estimator.add_received_clean_local_state(1, state, 1_000_000_000)
        estimator.update(np.zeros(5), .04, 1_000_000_000, np.zeros(2))
        self.assertFalse(estimator.is_target_available(1, 1_000_000_000))
        self.assertIsNone(estimator.get_trust_score(1))
        estimator.set_external_relative_measurement(1, 2., 0., 1_000_000_000, "relative_uio:radar")
        estimator.update(np.zeros(5), .04, 1_000_000_000, np.zeros(2))
        self.assertTrue(estimator.is_target_available(1, 1_000_000_000))
        self.assertFalse(estimator.is_target_available(1, 1_201_000_000))

    def test_bad_algorithm_and_gain_design_fail_explicitly(self):
        with self.assertRaises(ValueError):
            FleetEstimatorFactory.create("trust_kalman", 0, 2)
        with patch("Observer.relative_state_estimators.CVXPY_AVAILABLE", False):
            with self.assertRaises(RuntimeError):
                SA_ACC_UIO_Estimator({})
        with open("Observer/config_relative_estimators.yaml", encoding="utf-8") as handle:
            config = yaml.safe_load(handle)["relative"]["sa_acc_uio"]
        config["L"] = np.zeros((4, 2)).tolist()
        with self.assertRaises(ValueError):
            SA_ACC_UIO_Estimator(config)

    def test_missing_range_rate_is_not_replaced_by_zero(self):
        with open("Observer/config_relative_estimators.yaml", encoding="utf-8") as handle:
            config = yaml.safe_load(handle)["relative"]["sa_acc_uio"]
        observer = SA_ACC_UIO_Estimator(config)
        with self.assertRaises(RuntimeError):
            observer.update(np.array([2., np.nan]), .01, None, host_vehicle_state=np.zeros(5))

    def test_local_estimator_update_failure_does_not_return_last_state(self):
        observer = VehicleObserver.__new__(VehicleObserver)
        observer.local_estimator_type = "ekf"
        observer.local_estimator = Mock(update=Mock(return_value=False))
        observer.vehicle_logger = SimpleNamespace(log_error=lambda *args: None)
        observer.sensor_data = dict(motor_tach=0., gyro_z=0., accelerometer=np.zeros(3))
        observer.stop = lambda: None
        with self.assertRaises(RuntimeError):
            observer._update_local_observer(.01)

    def test_gps_properties_publish_measured_sample(self):
        gps = MockQCarGPS([0, 0, 0], {})
        gps.update_true_state(10, 20, 1)
        gps.last_data = np.array([9.9, 20.1, 1.02])
        np.testing.assert_allclose(gps.position[:2], gps.last_data[:2])
        self.assertEqual(gps.orientation[2], gps.last_data[2])

    def test_configured_sensor_noise_changes_measurement_not_plant(self):
        import copy
        import contextlib
        import io
        from simulation.config import SimulationConfig
        from simulation.mock_vehicle import MockQCar
        noisy_config = SimulationConfig.load_config()
        clean_config = copy.deepcopy(noisy_config)
        clean_config["sensors"]["imu"]["has_noise"] = False
        clean_config["sensors"]["wheel"]["has_noise"] = False
        with contextlib.redirect_stdout(io.StringIO()):
            noisy, clean = MockQCar(noisy_config), MockQCar(clean_config)
        for car in (noisy, clean):
            car.write(.1, .1)
            car.step(.01)
        np.testing.assert_allclose(noisy.get_true_state(), clean.get_true_state())
        self.assertNotEqual(noisy.motorTach, clean.motorTach)
        self.assertNotEqual(noisy.gyroscope[2], clean.gyroscope[2])
        self.assertFalse(np.allclose(noisy.accelerometer, clean.accelerometer))

    def test_following_stop_policy_does_not_select_another_controller(self):
        from StateMachine.following_leader_state import FollowingLeaderState
        state = FollowingLeaderState.__new__(FollowingLeaderState)
        state.trust_longitudinal_fusion_config = dict(unavailable_policy="stop", low_trust_policy="stop")
        state._get_trust_fusion_opinion = lambda *args: None
        state._compute_trust_fusion_alpha_for_mode = lambda *args: None
        state._set_longitudinal_fusion_log_fields = Mock()
        state._compute_sensor_acc_command = Mock(side_effect=AssertionError("Must not switch to sensor ACC"))
        self.assertEqual(state._apply_trust_longitudinal_fusion(.2, {}, {}, .01), 0.)
        state._get_trust_fusion_opinion = lambda *args: 0.1
        state._compute_trust_fusion_alpha_for_mode = lambda *args: 0.1
        self.assertEqual(state._apply_trust_longitudinal_fusion(.2, {}, {}, .01), 0.)

    def test_missing_checkpoint_propagates_and_does_not_select_ekf(self):
        logger = SimpleNamespace(logger=Mock(), log_error=Mock(), log_warning=Mock())
        observer = VehicleObserver(0, logger=logger)
        self.addCleanup(observer.stop)
        observer.local_estimator_type = "robust_kalman_net"
        with self.assertRaises(RuntimeError):
            observer.initialize_local_estimator(initial_pose=np.zeros(3), estimator_params={
                "estimator_backend": "innovation_trust", "model_path": "missing_pipeline_checkpoint.npz"})
        self.assertIsNone(observer.local_estimator)

    def test_observer_schedule_uses_elapsed_observer_time(self):
        from vehicle_logic import VehicleLogic
        logic = VehicleLogic.__new__(VehicleLogic)
        logic._last_observer_time = 1.0
        logic.observer_rate = 50
        self.assertFalse(logic._should_update_observer(1.005))
        self.assertTrue(logic._should_update_observer(1.025))
        self.assertAlmostEqual(logic._scheduled_observer_dt, .025)


if __name__ == "__main__":
    unittest.main()
