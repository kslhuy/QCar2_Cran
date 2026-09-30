"""Runtime invariants and regression tests (python -m unittest test_innovation_trust)."""
import copy
import unittest
from pathlib import Path

import numpy as np
import torch

from innovation_trust import InnovationTrustFilter
from simulation_benchmark import HERE, attacked_samples, generate_drive, model_config, run_estimator
from robustKLnet import RobustKalmanNetStateEstimator


class InnovationTrustTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.path = HERE/"models/innovation_trust_sim.npz"
        cls.drive = generate_drive(39, duration=8.)

    def core(self):
        return InnovationTrustFilter(self.drive["initial"], model_config(), self.path)

    def adapter(self, **extra):
        cfg = dict(model_config(), model_path=str(self.path), device="cpu",
                   estimator_backend="innovation_trust", enable_ekf_comparator=False,
                   comparator_record_to_file=False, publish_clean_reference_output=False)
        cfg.update(extra)
        return RobustKalmanNetStateEstimator(self.drive["initial"][:3], cfg)

    def test_adapter_matches_core_for_every_tick(self):
        for scenario in ("clean", "gps_jump", "wheel_bias", "gps_dropout"):
            samples, _, _ = attacked_samples(self.drive, scenario, 123)
            expected, _ = run_estimator(self.core(), samples)
            actual, _ = run_estimator(self.adapter(), samples)
            np.testing.assert_allclose(actual, expected, atol=3e-7)

    def test_numpy_mlp_matches_training_framework(self):
        core = self.core()
        w1, b1, w2, b2 = core.weights
        f = np.random.default_rng(4).normal(size=(32, core.FEATURE_COUNT))
        h = torch.relu(torch.tensor(f) @ torch.tensor(w1).T+torch.tensor(b1))
        expected = torch.sigmoid(h @ torch.tensor(w2).T+torch.tensor(b2)).numpy().reshape(-1)
        np.testing.assert_allclose(core.predict_trust(f), expected, atol=1e-12)

    def test_reset_removes_attack_memory(self):
        estimator = self.adapter()
        samples, _, _ = attacked_samples(self.drive, "gps_wheel", 6)
        run_estimator(estimator, samples)
        estimator.reset(self.drive["initial"][:3])
        actual, _ = run_estimator(estimator, self.drive["samples"])
        expected, _ = run_estimator(self.adapter(), self.drive["samples"])
        np.testing.assert_array_equal(actual, expected)

    def test_stale_and_nonfinite_gps_cannot_correct(self):
        for bad in (dict(x=100., y=-100., theta=2., valid=True, position_valid=False),
                    dict(x=np.nan, y=np.inf, theta=np.nan, valid=True)):
            a, b = self.core(), self.core()
            for sample in self.drive["samples"][:50]:
                a.update(**dict(sample, gps_data=bad))
                b.update(**dict(sample, gps_data=None))
            np.testing.assert_allclose(a.get_state(), b.get_state(), atol=1e-12)
            np.testing.assert_array_equal(a.last_trust[:3], 0)

    def test_nonfinite_wheel_is_unavailable(self):
        estimator = self.core()
        sample = dict(self.drive["samples"][0], motor_tach=np.nan)
        estimator.update(**sample)
        self.assertTrue(np.isfinite(estimator.get_state()).all())
        self.assertEqual(estimator.last_trust[3], 0)

    def test_heading_is_wrapped(self):
        estimator = self.core()
        estimator.reset([0, 0, np.pi-.001])
        for sample in self.drive["samples"][:100]:
            estimator.update(**dict(sample, gps_data=None, gyro_z=.5))
            self.assertLessEqual(abs(estimator.state[2]), np.pi)

    def test_clean_reference_cannot_be_published(self):
        with self.assertRaisesRegex(ValueError, "own estimate"):
            self.adapter(publish_clean_reference_output=True)

    def test_simulator_attack_interface_still_works(self):
        estimator = self.adapter()
        self.assertTrue(estimator.start_sensor_attack(dict(
            attack_mode="gps", gps_attack_types=["jump"],
            force_gps_attack=True, gps_attack_prob=1., seed=3)))
        run_estimator(estimator, self.drive["samples"][:30])
        self.assertIsNotNone(estimator.last_sensor_failure_metadata)
        self.assertTrue(np.isfinite(estimator.get_state()).all())
        self.assertTrue(estimator.stop_sensor_attack())

    def test_covariance_and_gain_bounds_during_long_outage(self):
        estimator = self.core()
        for sample in self.drive["samples"]:
            estimator.update(**dict(sample, gps_data=None))
            self.assertTrue((estimator.covariance > 0).all())
            self.assertTrue(((estimator.last_gain >= 0) & (estimator.last_gain <= 1)).all())
            self.assertTrue(((estimator.last_trust >= 0) & (estimator.last_trust <= 1)).all())

    def test_vehicle_observer_initialization_uses_simulation_overrides(self):
        from types import SimpleNamespace
        from Observer.VehicleObserverSimple import VehicleObserver
        from simulation.robust_estimator_config import simulation_estimator_params
        import logging
        # Exercise the actual merge/factory path without starting fleet services.
        observer = VehicleObserver.__new__(VehicleObserver)
        # No services/resources were started by this initialization-only harness.
        observer.stop = lambda: None
        observer.local_estimator_type = "robust_kalman_net"
        observer.local_config_defaults = dict(common=dict(v_lpf_alpha=.15), robust_kalman_net=dict(
            model_path="models/robust_kalmannet.best_robust.pt", publish_clean_reference_output=True))
        observer.vehicle_geometry_config = {"wheelbase": .2}
        observer.enable_relative = False
        observer.vehicle_logger = SimpleNamespace(logger=logging.getLogger("test"),
            log_error=lambda *a: self.fail(str(a)), log_warning=lambda *a: None,
            log_info=lambda *a: None)
        cfg = model_config()
        plant = SimpleNamespace(longitudinal_model="velocity_lag_lookup", steering_model="default",
            params=SimpleNamespace(a=.13, b=.13, longitudinal=SimpleNamespace(v_switch=.1)),
            velocity_lag_tau=.301, velocity_gain=6.424, velocity_lag_deadband=0.,
            velocity_lag_lookup_tau=.301,
            velocity_lag_lookup_throttle_breakpoints=np.array(cfg["velocity_lag_lookup_model"]["throttle_breakpoints"]),
            velocity_lag_lookup_velocity_breakpoints=np.array(cfg["velocity_lag_lookup_model"]["steady_state_velocity_breakpoints"]))
        self.assertTrue(observer.initialize_local_estimator(initial_pose=self.drive["initial"][:3],
                                                           estimator_params=simulation_estimator_params(plant)))
        self.assertEqual(observer.local_estimator.trust_filter.wheelbase, .26)
        self.assertEqual(observer.v_lpf_alpha, 1.)
        self.assertFalse(observer.local_estimator.publish_clean_reference_output)
        run_estimator(observer.local_estimator, self.drive["samples"][:50])

    def test_frozen_wheel_cannot_enter_pose_prediction(self):
        # Long freeze near cruising speed used to accumulate a large position
        # error even when the final learned velocity update rejected the sensor.
        estimator = self.core()
        frozen = None
        for i, sample in enumerate(self.drive["samples"]):
            if i == 100:
                frozen = sample["motor_tach"]
            if frozen is not None:
                sample = dict(sample, motor_tach=frozen)
            estimator.update(**sample)
            if i > 110:
                self.assertEqual(estimator.last_trust[3], 0.)
        self.assertLess(abs(estimator.state[3]-self.drive["truth"][-1, 3]), .04)

    def test_isolated_gyro_step_during_gps_outage(self):
        from innovation_trust import wrap
        estimator = self.core()
        for i, sample in enumerate(self.drive["samples"]):
            if i >= 100:
                sample = dict(sample, gyro_z=sample["gyro_z"]+.8, gps_data=None)
            estimator.update(**sample)
        self.assertTrue(estimator.gyro_fault_latched)
        self.assertLess(abs(wrap(estimator.state[2]-self.drive["truth"][-1, 2])), .05)


if __name__ == "__main__":
    unittest.main()
