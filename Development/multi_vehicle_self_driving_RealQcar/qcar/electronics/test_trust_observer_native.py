"""Golden parity tests for the native Trust/Distributed Observer core."""

from __future__ import annotations

import math
import unittest
from pathlib import Path

import numpy as np
import yaml

from Observer.TrustbasedDistributedObserver.trust_model import (
    TriPTrustModel,
    TrustConfig,
    TrustScore,
)
from Observer.TrustbasedDistributedObserver.weight_trust_module import (
    WeightConfig,
    WeightTrustModule,
)
from Observer.TrustbasedDistributedObserver.estimator_config import (
    normalize_vehicle_model_config,
)
from Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (
    TrustBasedFleetEstimator,
)

from .trust_observer_native import NativeTrustObserverCore


NATIVE_LIBRARY = Path(__file__).resolve().parent / "native" / "build" / (
    "cran_electronics_core.dll"
)


@unittest.skipUnless(
    NATIVE_LIBRARY.is_file(),
    "Build electronics/native before running native parity tests",
)
class NativeTrustObserverParityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.trust_values = {
            "num_trust_levels": 5,
            "dirichlet_update_rate": 0.1,
            "dirichlet_type": "Dual",
            "dirichlet_method": "matlab",
            "dirichlet_C": 0.2,
            "dirichlet_wt_local": 0.4,
            "dirichlet_wt_global": 0.5,
            "ema_alpha": 0.5,
            "trust_threshold": 0.5,
            "monitor_sudden_change": False,
            "attack_detection_window": 10,
            "trust_decay_lambda": 0.2,
            "distributed_trust_fallback": 0.2,
        }
        self.core = NativeTrustObserverCore(
            library_path=str(NATIVE_LIBRARY), trust_config=self.trust_values
        )

    def tearDown(self) -> None:
        self.core.close()

    def _python_received_step(
        self,
        model: TriPTrustModel,
        target_id: int,
        local_sample: float,
        global_sample: float,
    ) -> TrustScore:
        trust = model.trust_scores.setdefault(
            target_id, TrustScore(vehicle_id=target_id)
        )
        trust.local_trust_sample = float(local_sample)
        trust.global_trust_sample = float(global_sample)
        trust.final_score = model._dirichlet_matlab(
            target_id, local_sample, global_sample
        )
        trust.trust_levels = model._update_trust_levels(
            target_id, local_sample, global_sample
        )
        history = model.trust_history.get(target_id, [])
        if history:
            alpha = float(np.clip(model.config.ema_alpha, 0.0, 1.0))
            trust.final_score = float(
                np.clip(
                    alpha * trust.final_score + (1.0 - alpha) * history[-1],
                    0.0,
                    1.0,
                )
            )
        model.previous_trust_local[target_id] = float(local_sample)
        model.previous_trust_global[target_id] = float(global_sample)
        model.last_beacon_times[target_id] = 1.0
        model.beacon_receive_counts[target_id] += 1
        model._set_attack_flags(trust)
        model._update_history(target_id, trust.final_score)
        return trust

    def _assert_trust_equal(self, python: TrustScore, native: dict) -> None:
        self.assertAlmostEqual(native["final_score"], python.final_score, places=13)
        np.testing.assert_allclose(
            native["trust_levels"], python.trust_levels, rtol=0.0, atol=1e-14
        )
        for flag in (
            "flag_target_attack",
            "flag_global_est_check",
            "flag_local_est_check",
        ):
            self.assertEqual(native[flag], bool(getattr(python, flag)))

    def test_local_component_geometric_mean_matches_python(self) -> None:
        config = TrustConfig.from_dict(self.trust_values)
        model = TriPTrustModel(vehicle_id=0, config=config)
        trust = TrustScore(
            vehicle_id=1,
            velocity_score=0.82,
            distance_score=0.67,
            acceleration_score=0.91,
            heading_score=0.73,
            beacon_score=1.0,
            quality_factor=0.88,
        )
        expected = model._compute_local_trust_sample(trust)
        actual = self.core.local_trust_sample(
            [
                trust.velocity_score,
                trust.distance_score,
                trust.acceleration_score,
                trust.heading_score,
                trust.beacon_score,
                trust.quality_factor,
            ]
        )
        self.assertAlmostEqual(actual, expected, places=14)

    def test_matlab_dirichlet_and_missing_beacon_replay(self) -> None:
        config = TrustConfig.from_dict(self.trust_values)
        model = TriPTrustModel(vehicle_id=0, config=config)
        target_id = 2

        for step in range(60):
            local = float(np.clip(0.62 + 0.3 * math.sin(step * 0.31), 0.0, 1.0))
            global_value = float(
                np.clip(0.58 + 0.35 * math.cos(step * 0.23), 0.0, 1.0)
            )
            if step > 0 and step % 9 == 0:
                python = model.update_missing_observation(
                    target_id=target_id,
                    current_time_ns=int((2.0 + step * 0.04) * 1e9),
                )
                native = self.core.trust_step(
                    target_id=target_id,
                    local_trust_sample=python.local_trust_sample,
                    global_trust_sample=python.global_trust_sample,
                    missing_observation=True,
                )
            else:
                python = self._python_received_step(
                    model, target_id, local, global_value
                )
                native = self.core.trust_step(
                    target_id=target_id,
                    local_trust_sample=local,
                    global_trust_sample=global_value,
                )
            self._assert_trust_equal(python, native)

    def test_observer_correction_matches_python_with_angle_wrap(self) -> None:
        current = np.array([1.0, -0.5, math.pi - 0.04, 1.8, -0.5])
        direct = np.array([1.4, -0.1, -math.pi + 0.03, 2.8, 6.0])
        neighbors = np.array(
            [
                [0.8, -0.2, -math.pi + 0.01, 1.4, 0.3],
                [1.2, -0.7, math.pi - 0.08, -1.0, -7.0],
            ]
        )
        weights = np.array([0.25, 0.15])
        direct_weight = 0.35

        expected = current.copy()
        for state, weight in [(direct, direct_weight), *zip(neighbors, weights)]:
            residual = state - current
            residual[2] = math.atan2(math.sin(residual[2]), math.cos(residual[2]))
            expected += float(weight) * residual
        expected[2] = math.atan2(math.sin(expected[2]), math.cos(expected[2]))
        expected[3] = np.clip(expected[3], -2.0, 2.0)
        expected[4] = np.clip(expected[4], -5.0, 5.0)

        actual = self.core.observer_correct(
            current_state=current,
            direct_state=direct,
            direct_weight=direct_weight,
            neighbor_states=neighbors,
            neighbor_weights=weights,
            max_velocity=2.0,
            max_acceleration=5.0,
        )
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-14)

    @staticmethod
    def _apply_recovery_scale(weights: dict, scale: float) -> dict:
        scaled = {
            "w0": float(weights.get("w0", 0.0)) * scale,
            "w_self": float(weights.get("w_self", 0.0)),
            "neighbors": dict(weights.get("neighbors", {})),
        }
        scaled["w_self"] += float(weights.get("w0", 0.0)) - scaled["w0"]
        used = scaled["w0"] + scaled["w_self"] + sum(
            scaled["neighbors"].values()
        )
        scaled["w_self"] += 1.0 - used
        return scaled

    def _assert_weights_equal(self, expected: dict, actual: dict) -> None:
        self.assertAlmostEqual(expected["w0"], actual["w0"], places=14)
        self.assertAlmostEqual(expected["w_self"], actual["w_self"], places=14)
        expected_neighbors = dict(expected.get("neighbors", {}))
        actual_neighbors = dict(actual.get("neighbors", {}))
        for neighbor_id in set(expected_neighbors) | set(actual_neighbors):
            self.assertAlmostEqual(
                expected_neighbors.get(neighbor_id, 0.0),
                actual_neighbors.get(neighbor_id, 0.0),
                places=14,
            )

    def test_all_target_weight_modes_match_python(self) -> None:
        target_id = 4
        direct = np.ones(5)
        source_scores = {1: 0.91, 2: 0.74, 4: 0.83, 5: 0.61}
        fleets = {
            neighbor_id: {target_id: {"x": float(neighbor_id)}}
            for neighbor_id in source_scores
        }
        base_values = dict(
            w0_fixed=0.4,
            w_self_base=0.2,
            w_cap=0.18,
            kappa=3,
            trust_threshold=0.5,
            include_target_self_fleet_estimate=True,
            use_gamma_self_weight_adaptation=True,
            gamma_self_weight_floor=0.25,
            flag_w0_target_attack_factor=0.2,
            flag_w0_global_est_check_factor=1.4,
            flag_w0_local_est_check_factor=0.6,
            local_bad_zero_w0_neighbor_total_cap=0.05,
        )
        target_trust = TrustScore(
            vehicle_id=target_id,
            local_trust_sample=0.32,
            gamma_self=0.43,
            flag_target_attack=True,
            flag_local_est_check=True,
        )

        for mode in ("startup", "equal", "paper", "trust_based"):
            config = WeightConfig(weight_type=mode, **base_values)
            module = WeightTrustModule(
                vehicle_id=0, fleet_size=6, config=config
            )
            candidates = [
                (neighbor_id, source_scores[neighbor_id])
                for neighbor_id, fleet in fleets.items()
                if target_id in fleet
                and module._allow_fleet_source_for_target(neighbor_id, target_id)
            ]
            if mode == "startup":
                expected = module.calculate_startup_weights_for_target(
                    target_id, fleets, direct
                )
                local_trust = target_trust.local_trust_sample
                native_target = target_trust
            elif mode == "paper":
                expected = module.calculate_paper_weights_for_target(
                    target_id=target_id,
                    opinion_scores=source_scores,
                    target_local_trust=target_trust.local_trust_sample,
                    neighbor_fleet_estimates=fleets,
                    direct_measurement=direct,
                )
                local_trust = target_trust.local_trust_sample
                native_target = target_trust
            else:
                expected = module.calculate_weights_for_target(
                    target_id=target_id,
                    trust_scores=source_scores,
                    neighbor_fleet_estimates=fleets,
                    direct_measurement=direct,
                    target_trust_obj=target_trust,
                )
                local_trust = target_trust.local_trust_sample
                native_target = target_trust

            recovery_scale = 0.37
            expected = self._apply_recovery_scale(expected, recovery_scale)
            actual = self.core.observer_weights(
                mode=mode,
                weight_config=config,
                neighbor_ids=[item[0] for item in candidates],
                neighbor_trust_scores=[item[1] for item in candidates],
                direct_available=True,
                target_trust=native_target,
                target_local_trust=local_trust,
                direct_recovery_scale=recovery_scale,
            )
            self._assert_weights_equal(expected, actual)

    @staticmethod
    def _prediction_options() -> dict:
        return {
            "anchor_position_weight": 0.8,
            "estimate_position_weight": 0.2,
            "clean_theta_weight": 0.75,
            "host_theta_weight": 0.25,
            "target_velocity_weight": 0.2,
            "host_velocity_weight": 0.8,
            "target_acceleration_weight": 0.3,
            "host_acceleration_weight": 0.7,
            "use_anchor_bearing": True,
        }

    @staticmethod
    def _prediction_harness(model: dict, mode: str):
        harness = object.__new__(TrustBasedFleetEstimator)
        harness.default_vehicle_model = dict(model)
        harness.vehicle_model_overrides = {}
        harness.dynamics_prediction_mode = mode
        harness.state_dim = 5
        harness.logger = None
        harness.received_clean_local_states = {}
        options = NativeTrustObserverParityTests._prediction_options()
        harness.relative_host_anchor_anchor_position_weight = options[
            "anchor_position_weight"
        ]
        harness.relative_host_anchor_estimate_position_weight = options[
            "estimate_position_weight"
        ]
        harness.relative_host_anchor_clean_theta_weight = options[
            "clean_theta_weight"
        ]
        harness.relative_host_anchor_host_theta_weight = options[
            "host_theta_weight"
        ]
        harness.relative_host_anchor_target_velocity_weight = options[
            "target_velocity_weight"
        ]
        harness.relative_host_anchor_host_velocity_weight = options[
            "host_velocity_weight"
        ]
        harness.relative_host_anchor_target_acceleration_weight = options[
            "target_acceleration_weight"
        ]
        harness.relative_host_anchor_host_acceleration_weight = options[
            "host_acceleration_weight"
        ]
        harness.relative_host_anchor_use_bearing = options["use_anchor_bearing"]
        return harness

    def test_vehicle_prediction_models_match_python(self) -> None:
        state = np.array([0.8, -0.4, math.pi - 0.12, 0.72, -0.16])
        control = np.array([0.17, 0.31])
        dt = 0.04
        model_variants = [
            {"longitudinal_model": "constant_velocity"},
            {
                "longitudinal_model": "velocity_lag",
                "velocity_lag_model": {
                    "enabled": True,
                    "tau": 0.34,
                    "velocity_gain": 3.2,
                    "throttle_deadband": 0.03,
                },
            },
            {
                "longitudinal_model": "velocity_lag_lookup",
                "velocity_lag_lookup_model": {
                    "enabled": True,
                    "tau": 0.29,
                    "throttle_breakpoints": [-0.5, 0.0, 0.5],
                    "steady_state_velocity_breakpoints": [-1.0, 0.0, 1.4],
                },
            },
            {"longitudinal_model": "velocity_command", "velocity_command_tau": 0.27},
            {
                "longitudinal_model": "acceleration_lag",
                "accel_lag_model": {"enabled": True, "tau": 0.32, "input_gain": 1.3},
            },
            {"longitudinal_model": "simple_acceleration"},
        ]
        for raw_model in model_variants:
            model = normalize_vehicle_model_config(
                {
                    "dynamics_prediction_mode": "model",
                    "wheelbase": 0.256,
                    "max_velocity": 2.0,
                    "max_acceleration": 2.0,
                    "max_steering": 0.5,
                    **raw_model,
                },
                "model",
            )
            harness = self._prediction_harness(model, "model")
            expected = harness._apply_state_constraints(
                harness._predict_dynamics(
                    state.copy(), control, dt, target_id=3, current_time_ns=None
                ),
                target_id=3,
            )
            actual = self.core.observer_predict(
                corrected_state=state,
                clean_state=None,
                control=control,
                dt=dt,
                prediction_mode="model",
                model_config=model,
                prediction_options=self._prediction_options(),
                force_clean_pose_anchor=False,
                attack_anchor_active=False,
                host_anchor=None,
            )
            np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-13)

    def test_clean_and_attack_anchor_prediction_match_python(self) -> None:
        model = normalize_vehicle_model_config(
            {
                "dynamics_prediction_mode": "mixed_clean_data",
                "longitudinal_model": "velocity_lag",
                "velocity_lag_model": {
                    "enabled": True,
                    "tau": 0.31,
                    "velocity_gain": 2.8,
                },
                "max_velocity": 2.0,
                "max_acceleration": 2.0,
            },
            "mixed_clean_data",
        )
        state = np.array([1.1, 0.2, 0.35, 0.64, 0.12])
        clean = np.array([1.3, 0.1, -0.28, 0.71, 0.08])
        control = np.array([-0.09, 0.26])
        dt = 0.05
        anchor = {
            "host_x": 0.4,
            "host_y": -0.2,
            "host_theta": 0.18,
            "host_velocity": 0.55,
            "host_acceleration": -0.04,
            "distance": 0.92,
            "sign": 1.0,
            "relative_x": 0.86,
            "relative_y": 0.24,
        }
        harness = self._prediction_harness(model, "mixed_clean_data")
        harness.received_clean_local_states = {3: [(0, clean.copy())]}
        expected = harness._apply_state_constraints(
            harness._predict_dynamics(
                state.copy(),
                control,
                dt,
                target_id=3,
                current_time_ns=None,
                force_clean_pose_anchor=True,
                attack_relative_host_anchor_active=True,
                host_anchor_snapshot=anchor,
            ),
            target_id=3,
        )
        actual = self.core.observer_predict(
            corrected_state=state,
            clean_state=clean,
            control=control,
            dt=dt,
            prediction_mode="mixed_clean_data",
            model_config=model,
            prediction_options=self._prediction_options(),
            force_clean_pose_anchor=True,
            attack_anchor_active=True,
            host_anchor=anchor,
        )
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-13)

        harness.dynamics_prediction_mode = "clean_data"
        expected_clean = harness._apply_state_constraints(
            harness._predict_dynamics(
                state.copy(),
                control,
                dt,
                target_id=3,
                current_time_ns=None,
                force_clean_pose_anchor=True,
            ),
            target_id=3,
        )
        actual_clean = self.core.observer_predict(
            corrected_state=state,
            clean_state=clean,
            control=control,
            dt=dt,
            prediction_mode="clean_data",
            model_config=model,
            prediction_options=self._prediction_options(),
            force_clean_pose_anchor=True,
            attack_anchor_active=False,
            host_anchor=None,
        )
        np.testing.assert_allclose(
            actual_clean, expected_clean, rtol=0.0, atol=1e-13
        )

    def test_none_dead_reckoning_and_no_control_prediction_match_python(self) -> None:
        state = np.array([-0.2, 0.7, -2.8, 0.54, 0.19])
        dt = 0.06
        model = normalize_vehicle_model_config(
            {
                "dynamics_prediction_mode": "model",
                "longitudinal_model": "velocity_lag",
                "velocity_lag_model": {
                    "enabled": True,
                    "tau": 0.3,
                    "velocity_gain": 3.0,
                },
                "max_velocity": 2.0,
                "max_acceleration": 2.0,
            },
            "model",
        )
        for mode in ("none", "dead_reckoning", "mixed_clean_data"):
            harness = self._prediction_harness(model, mode)
            expected = harness._apply_state_constraints(
                harness._predict_dynamics(
                    state.copy(),
                    None,
                    dt,
                    target_id=2,
                    current_time_ns=None,
                    attack_relative_host_anchor_active=False,
                ),
                target_id=2,
            )
            actual = self.core.observer_predict(
                corrected_state=state,
                clean_state=None,
                control=None,
                dt=dt,
                prediction_mode=mode,
                model_config=model,
                prediction_options=self._prediction_options(),
                force_clean_pose_anchor=False,
                attack_anchor_active=False,
                host_anchor=None,
            )
            np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-13)

    def test_real_estimator_four_stage_replay_opens_test_gate(self) -> None:
        config_path = (
            Path(__file__).resolve().parent.parent
            / "Observer"
            / "TrustbasedDistributedObserver"
            / "config_trust_estimator.yaml"
        )
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        config["embedded_core"]["authority_min_comparisons_per_stage"] = 20
        estimator = TrustBasedFleetEstimator(
            vehicle_id=0,
            fleet_size=3,
            state_dim=5,
            config=config,
            logger=None,
        )
        try:
            dt = 0.04
            host = np.array([0.0, 0.0, 0.0, 0.4, 0.0])
            for step in range(35):
                timestamp_ns = int((1.0 + step * dt) * 1e9)
                x_target = 1.0 + 0.45 * step * dt
                target = {
                    "x": x_target,
                    "y": 0.05,
                    "theta": 0.02,
                    "velocity": 0.45,
                    "acceleration": 0.0,
                    "control_input": {"steering": 0.01, "throttle": 0.1},
                }
                estimator.add_received_local_state(1, target, timestamp_ns)
                estimator.add_received_clean_local_state(1, target, timestamp_ns)
                estimator.add_received_fleet_state(
                    1,
                    {
                        0: {
                            "x": host[0],
                            "y": host[1],
                            "theta": host[2],
                            "velocity": host[3],
                            "acceleration": host[4],
                        },
                        1: target,
                    },
                    timestamp_ns,
                )
                estimator.add_received_fleet_state(
                    2,
                    {
                        1: {
                            "x": x_target + 0.01,
                            "y": 0.045,
                            "theta": 0.021,
                            "velocity": 0.44,
                            "acceleration": 0.0,
                        }
                    },
                    timestamp_ns,
                )
                host[0] += host[3] * dt
                estimator.update(
                    host.copy(),
                    dt,
                    timestamp_ns,
                    np.array([0.0, 0.1]),
                )

            status = estimator.get_embedded_core_status()
            self.assertTrue(status["available"], status["reason"])
            self.assertTrue(status["parity_pass"], status)
            self.assertTrue(status["authority_ready"], status["authority_blockers"])
            self.assertEqual(status["failures"], 0)
            for stage in ("trust", "weight", "observer", "prediction"):
                self.assertGreaterEqual(status[f"{stage}_comparisons"], 20)

            authority = estimator.set_embedded_core_mode("native_authority")
            self.assertTrue(authority["mode_request_accepted"], authority)
            self.assertTrue(authority["authority_active"], authority)

            # Native results remain continuously checked after handover.  A
            # deliberately wrong Python reference must immediately fail safe.
            bad_reference = TrustScore(
                vehicle_id=99,
                final_score=-10.0,
                local_trust_sample=0.4,
                global_trust_sample=0.6,
                trust_levels=np.zeros(5),
            )
            mismatch = estimator.embedded_core_shadow.compare_trust(
                target_id=99,
                python_trust=bad_reference,
                missing_observation=False,
            )
            self.assertFalse(mismatch["passed"])
            failed_safe = estimator.get_embedded_core_status()
            self.assertEqual(failed_safe["mode"], "shadow")
            self.assertEqual(failed_safe["failback_count"], 1)
            self.assertEqual(failed_safe["last_failback_stage"], "trust")
            self.assertFalse(failed_safe["authority_ready"])

            reset = estimator.reset_embedded_core_parity()
            self.assertEqual(reset["mode"], "shadow")
            self.assertEqual(reset["comparisons"], 0)
            self.assertEqual(reset["failback_count"], 0)
            estimator.update(
                host.copy(),
                dt,
                int((1.0 + 36 * dt) * 1e9),
                np.array([0.0, 0.1]),
            )
            after_reset = estimator.get_embedded_core_status()
            self.assertEqual(after_reset["failures"], 0, after_reset)
            self.assertGreater(after_reset["comparisons"], 0)
        finally:
            estimator.__del__()


if __name__ == "__main__":
    unittest.main()
