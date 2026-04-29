"""Regression checks for trust fleet estimator prediction mode selection."""

import os
import sys
import numpy as np


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
OBSERVER_DIR = os.path.dirname(CURRENT_DIR)
QCAR_DIR = os.path.dirname(OBSERVER_DIR)
for path in (CURRENT_DIR, OBSERVER_DIR, QCAR_DIR):
    if path not in sys.path:
        sys.path.append(path)

from trust_based_fleet_estimator import TrustBasedFleetEstimator
from weight_trust_module import WeightConfig, WeightTrustModule


def _make_estimator(mode: str) -> TrustBasedFleetEstimator:
    estimator = TrustBasedFleetEstimator.__new__(TrustBasedFleetEstimator)
    estimator.dynamics_prediction_mode = (
        TrustBasedFleetEstimator._normalize_dynamics_prediction_mode(mode)
    )
    estimator.enable_output_low_pass = False
    estimator.output_low_pass_alpha = 1.0
    estimator.default_vehicle_model = {
        "dynamics_prediction_mode": estimator.dynamics_prediction_mode,
        "wheelbase": 0.5,
        "max_velocity": 10.0,
        "max_acceleration": 10.0,
        "max_steering": 1.0,
        "longitudinal_model": "simple_acceleration",
    }
    estimator.vehicle_model_overrides = {}
    return estimator


def test_prediction_mode_model_uses_existing_vehicle_model():
    estimator = _make_estimator("model")
    state = np.array([0.0, 0.0, 0.0, 1.0, 0.2], dtype=float)
    control = np.array([0.1, 0.5], dtype=float)

    predicted = estimator._predict_dynamics(state, control, 0.2, target_id=1)

    assert predicted[0] > state[0]
    assert predicted[2] > state[2]
    assert predicted[3] > state[3]
    assert abs(predicted[4] - 0.5) <= 1e-9


def test_prediction_mode_dead_reckoning_propagates_pose_only():
    estimator = _make_estimator("dead_reckoning")
    state = np.array([1.0, -2.0, np.pi / 4.0, 2.0, 0.3], dtype=float)
    control = np.array([0.3, 0.9], dtype=float)

    predicted = estimator._predict_dynamics(state, control, 0.5, target_id=1)

    expected_xy = np.array(
        [
            1.0 + 2.0 * np.cos(np.pi / 4.0) * 0.5,
            -2.0 + 2.0 * np.sin(np.pi / 4.0) * 0.5,
        ],
        dtype=float,
    )
    assert np.allclose(predicted[:2], expected_xy)
    assert abs(predicted[2] - state[2]) <= 1e-9
    assert abs(predicted[3] - state[3]) <= 1e-9
    assert abs(predicted[4] - state[4]) <= 1e-9


def test_prediction_mode_none_leaves_state_unchanged():
    estimator = _make_estimator("none")
    state = np.array([3.0, 4.0, -0.2, 1.5, -0.4], dtype=float)
    control = np.array([0.4, 1.2], dtype=float)

    predicted = estimator._predict_dynamics(state, control, 0.3, target_id=1)

    assert np.allclose(predicted, state)


def test_startup_target_weights_remain_stable_during_warmup():
    estimator = TrustBasedFleetEstimator.__new__(TrustBasedFleetEstimator)
    estimator.vehicle_id = 0
    estimator.weight_module = WeightTrustModule(
        vehicle_id=0,
        fleet_size=4,
        config=WeightConfig(
            weight_type="trust_based",
            w0_fixed=0.3,
            w_self_base=0.2,
            w_cap=0.4,
            kappa=5,
            enable_smoothing=False,
        ),
    )
    estimator._startup_target_weights_cache = {}

    direct_state = np.array([1.0, 2.0, 0.0, 0.5, 0.1], dtype=float)
    initial_neighbors = {2: {1: np.ones(5, dtype=float)}}
    expanded_neighbors = {
        2: {1: np.ones(5, dtype=float)},
        3: {1: np.full(5, 2.0, dtype=float)},
    }

    first = estimator._get_startup_target_weights(
        target_id=1,
        neighbor_fleet_estimates=initial_neighbors,
        direct_state=direct_state,
    )
    second = estimator._get_startup_target_weights(
        target_id=1,
        neighbor_fleet_estimates=expanded_neighbors,
        direct_state=direct_state,
    )

    assert first == second
    assert abs(first["w0"] - 0.3) <= 1e-9
    assert abs(first["w_self"] - 0.3) <= 1e-9
    assert abs(first["neighbors"][2] - 0.4) <= 1e-9
    total = first["w0"] + first["w_self"] + sum(first["neighbors"].values())
    assert total <= 1.0 + 1e-12


def test_output_low_pass_filter_blends_final_state():
    estimator = _make_estimator("model")
    estimator.enable_output_low_pass = True
    estimator.output_low_pass_alpha = 0.25

    prev_state = np.array([0.0, 0.0, 0.0, 1.0, 0.0], dtype=float)
    new_state = np.array([4.0, -2.0, 0.4, 3.0, 2.0], dtype=float)

    filtered = estimator._apply_output_low_pass_filter(
        previous_state=prev_state,
        new_state=new_state,
        target_id=1,
    )

    expected = 0.75 * prev_state + 0.25 * new_state
    assert np.allclose(filtered[[0, 1, 3, 4]], expected[[0, 1, 3, 4]])
    assert abs(filtered[2] - 0.1) <= 1e-9


def test_output_low_pass_filter_wraps_theta_on_shortest_path():
    estimator = _make_estimator("model")
    estimator.enable_output_low_pass = True
    estimator.output_low_pass_alpha = 0.5

    prev_state = np.array([0.0, 0.0, np.pi - 0.1, 0.0, 0.0], dtype=float)
    new_state = np.array([0.0, 0.0, -np.pi + 0.1, 0.0, 0.0], dtype=float)

    filtered = estimator._apply_output_low_pass_filter(
        previous_state=prev_state,
        new_state=new_state,
        target_id=1,
    )

    assert abs(filtered[2] - np.pi) <= 0.11


def run_all():
    test_prediction_mode_model_uses_existing_vehicle_model()
    test_prediction_mode_dead_reckoning_propagates_pose_only()
    test_prediction_mode_none_leaves_state_unchanged()
    test_startup_target_weights_remain_stable_during_warmup()
    test_output_low_pass_filter_blends_final_state()
    test_output_low_pass_filter_wraps_theta_on_shortest_path()
    print("All trust fleet prediction-mode tests passed.")


if __name__ == "__main__":
    run_all()
