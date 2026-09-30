"""Live attack and observer-switch regressions; no sockets or hardware needed."""
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

QCAR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(QCAR))

from Observer.local_state_estimators import EKFStateEstimator
from Observer.KalmaNet.Robust.robustKLnet import RobustKalmanNetStateEstimator
from Observer.KalmaNet.Robust.runtime_sensor_attack import RuntimeSensorAttackMixin
from simulation.robust_estimator_config import simulation_estimator_params, simulation_motion_params


class LiveSensorAttackTests(unittest.TestCase):
    def setUp(self):
        self.pose = np.array([-1.064, -0.673, -0.694])
        self.plant = SimpleNamespace(
            longitudinal_model="velocity_lag", steering_model="default",
            disturbance_mode="none",
            params=SimpleNamespace(a=.13, b=.13, longitudinal=SimpleNamespace(v_switch=.1)),
            velocity_lag_tau=.301, velocity_gain=6.424, velocity_lag_deadband=0.,
            velocity_lag_lookup_tau=.301,
            velocity_lag_lookup_throttle_breakpoints=np.array([0., .1, .2]),
            velocity_lag_lookup_velocity_breakpoints=np.array([0., .5, 1.]),
        )
        self.ekf = EKFStateEstimator(self.pose, dict(
            simulation_motion_params(self.plant), use_qcar_ekf=False))
        self.robust = RobustKalmanNetStateEstimator(
            self.pose, simulation_estimator_params(self.plant))

    def sample(self, tick=0):
        return dict(motor_tach=.5 + tick*.02, steering=.1 + tick*.01,
                    gyro_z=.2 + tick*.02, throttle=.1, dt=.01,
                    acceleration=np.array([.3 + tick*.1, .2, 9.81]),
                    gps_data=dict(x=-1. + tick*.05, y=-.6 + tick*.02,
                                  theta=-.65, valid=True, position_valid=True))

    def test_all_ui_attacks_deliver_same_inputs_to_both_filters(self):
        cases = [(target, kind) for target in ("imu", "steering", "velocity", "random")
                 for kind in RuntimeSensorAttackMixin.DEFAULT_BRANCH_ATTACK_TYPES]
        cases += [("gps", kind) for kind in RuntimeSensorAttackMixin.DEFAULT_GPS_ATTACK_TYPES]
        for target, kind in cases:
            with self.subTest(target=target, kind=kind):
                config = dict(target_sensor=target, enabled_attacks=[kind] if target != "gps" else ["bias"],
                              gps_attack_types=[kind] if target == "gps" else ["jump"],
                              attack_prob=1., gps_attack_prob=0.,
                              min_attack_steps=3, max_attack_steps=3, seed=42,
                              max_branches_attacked=1)
                for estimator in (self.ekf, self.robust):
                    estimator.stop_sensor_attack()
                    estimator.reset(self.pose)
                    self.assertTrue(estimator.start_sensor_attack(config))
                with patch.object(self.ekf, "_update_fallback_ekf", wraps=self.ekf._update_fallback_ekf) as ekf_update, \
                     patch.object(self.robust.trust_filter, "update", wraps=self.robust.trust_filter.update) as robust_update:
                    for tick in range(5):  # Includes the next repeated burst.
                        sample = self.sample(tick)
                        original = copy.deepcopy(sample)
                        self.assertTrue(self.ekf.update(**sample))
                        self.assertTrue(self.robust.update(**sample))
                        keys = ("motor_tach", "steering", "throttle", "dt", "gyro_z", "gps_data", "acceleration")
                        ekf_inputs = dict(zip(keys, ekf_update.call_args.args))
                        robust_inputs = robust_update.call_args.kwargs
                        for key in keys:
                            if key == "gps_data":
                                self.assertEqual(ekf_inputs[key], robust_inputs[key])
                            else:
                                np.testing.assert_allclose(ekf_inputs[key], robust_inputs[key])
                        self.assertEqual(self.ekf.get_sensor_attack_status(), self.robust.get_sensor_attack_status())
                        self.assertTrue(self.ekf.get_sensor_attack_status()["local_sensor_attack_active"])
                        np.testing.assert_array_equal(sample["acceleration"], original["acceleration"])
                        self.assertEqual(sample["gps_data"], original["gps_data"])

    def test_ekf_attack_changes_estimate_and_stop_restores_clean_inputs(self):
        clean = EKFStateEstimator(self.pose, dict(simulation_motion_params(self.plant), use_qcar_ekf=False))
        self.assertTrue(self.ekf.start_sensor_attack(dict(target_sensor="velocity", enabled_attacks=["zero_out"], seed=4)))
        for tick in range(5):
            self.ekf.update(**self.sample(tick))
            clean.update(**self.sample(tick))
        self.assertGreater(np.linalg.norm(self.ekf.get_state()-clean.get_state()), .01)
        self.ekf.stop_sensor_attack()
        status = self.ekf.get_sensor_attack_status()
        self.assertTrue(status["local_sensor_attack_supported"])
        self.assertFalse(status["local_sensor_attack_enabled"])
        self.assertFalse(status["local_sensor_attack_active"])
        self.ekf.reset(self.pose)
        clean.reset(self.pose)
        self.ekf.update(**self.sample())
        clean.update(**self.sample())
        np.testing.assert_array_equal(self.ekf.get_state(), clean.get_state())

    def test_vehicle_command_accepts_both_estimators(self):
        from vehicle_logic import VehicleLogic
        logic = VehicleLogic.__new__(VehicleLogic)
        logic.vehicle_logger = Mock()
        logic.vehicle_id = 0
        for kind, estimator in (("ekf", self.ekf), ("robust_kalman_net", self.robust)):
            logic.vehicle_observer = SimpleNamespace(local_estimator_type=kind,
                                                     get_local_estimator=lambda: estimator)
            self.assertTrue(logic.start_local_sensor_attack(dict(target_sensor="gps")))
            self.assertTrue(estimator.get_sensor_attack_status()["local_sensor_attack_enabled"])
            self.assertTrue(logic.stop_local_sensor_attack())
        logic.vehicle_observer.local_estimator_type = "luenberger"
        self.assertFalse(logic.start_local_sensor_attack())

    def test_live_switch_keeps_pose_and_simulation_checkpoint(self):
        from StateMachine.state_base import StateBase
        from Observer.VehicleObserverSimple import VehicleObserver
        observer = VehicleObserver.__new__(VehicleObserver)
        observer.stop = lambda: None
        observer.vehicle_logger = Mock()
        observer.vehicle_id = 0
        observer.local_estimator = self.ekf
        observer.local_estimator_type = "ekf"
        observer.local_config_defaults = dict(common={}, ekf=dict(use_qcar_ekf=True),
                                             robust_kalman_net=dict(model_path="wrong.pt", publish_clean_reference_output=True))
        logic = SimpleNamespace(vehicle_observer=observer, vehicle_logger=observer.vehicle_logger,
                                config=SimpleNamespace(), _parent_fake_vehicle=SimpleNamespace(mock_qcar=self.plant),
                                invalidate_periodic_status_cache=Mock())
        state = StateBase(logic)
        for kind in ("robust_kalman_net", "ekf", "robust_kalman_net"):
            self.assertTrue(state._switch_local_observer(kind))
            self.assertEqual(observer.local_estimator_type, kind)
            np.testing.assert_allclose(observer.local_estimator.get_state()[:3], self.pose, atol=1e-6)
            self.assertFalse(observer.get_local_sensor_attack_status()["local_sensor_attack_enabled"])
            if kind == "robust_kalman_net":
                self.assertIsNotNone(observer.local_estimator.trust_filter)
                self.assertFalse(observer.local_estimator.publish_clean_reference_output)
            else:
                self.assertFalse(observer.local_estimator.use_qcar_ekf)
        self.assertEqual(observer.local_config_defaults["robust_kalman_net"]["model_path"], "wrong.pt")


if __name__ == "__main__":
    unittest.main()
