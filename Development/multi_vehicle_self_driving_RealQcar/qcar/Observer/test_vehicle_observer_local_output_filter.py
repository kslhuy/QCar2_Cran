"""Regression checks for VehicleObserver local-output low-pass filtering."""

import os
import sys
import threading
import numpy as np


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
QCAR_DIR = os.path.dirname(CURRENT_DIR)
for path in (CURRENT_DIR, QCAR_DIR):
    if path not in sys.path:
        sys.path.append(path)

from VehicleObserverSimple import VehicleObserver


def _make_observer(alpha: float) -> VehicleObserver:
    observer = VehicleObserver.__new__(VehicleObserver)
    observer.state_dim = 5
    observer.vehicle_logger = None
    observer.local_output_lpf_alpha = alpha
    observer._local_output_filter_initialized = False
    observer.local_state = np.zeros(5, dtype=float)
    observer.sensor_data = {"accelerometer": np.array([0.7, 0.0, 0.0], dtype=float)}
    observer.lock = threading.RLock()
    observer.vehicle_id = 0
    observer.control_input = {"steering": 0.1, "throttle": 0.2}
    observer.gps_valid = True
    observer.local_recorder = None
    observer.fleet_recorder = None
    observer.local_estimator = None
    observer.relative_estimator = None
    observer._build_v2v_time_payload_locked = lambda: {}
    return observer


def test_local_output_lpf_passthroughs_first_state():
    observer = _make_observer(alpha=0.25)
    state = np.array([1.0, 2.0, 0.3, 4.0, 0.5], dtype=float)

    filtered = observer._apply_local_output_lpf(state)

    assert np.allclose(filtered, state)
    assert observer._local_output_filter_initialized is True


def test_local_output_lpf_blends_and_wraps_heading():
    observer = _make_observer(alpha=0.25)
    previous = np.array([0.0, 0.0, np.pi - 0.1, 1.0, 0.0], dtype=float)
    observer.local_state = previous.copy()
    observer._local_output_filter_initialized = True
    new_state = np.array([4.0, -2.0, -np.pi + 0.1, 3.0, 2.0], dtype=float)

    filtered = observer._apply_local_output_lpf(new_state)

    expected = 0.75 * previous + 0.25 * new_state
    assert np.allclose(filtered[[0, 1, 3, 4]], expected[[0, 1, 3, 4]])
    assert abs(filtered[2] - (np.pi - 0.05)) <= 1e-9


def test_local_output_lpf_fills_acceleration_for_legacy_4d_state():
    observer = _make_observer(alpha=1.0)
    state = np.array([1.0, 2.0, 0.2, 3.0], dtype=float)

    filtered = observer._apply_local_output_lpf(state)

    assert np.allclose(filtered[:4], state)
    assert abs(filtered[4] - 0.7) <= 1e-9


def test_broadcast_uses_filtered_local_state():
    observer = _make_observer(alpha=0.25)
    observer.local_state = np.array([1.0, 2.0, 0.4, 3.0, 0.6], dtype=float)

    payload = observer.get_local_state_for_broadcast()

    assert payload["x"] == 1.0
    assert payload["y"] == 2.0
    assert payload["theta"] == 0.4
    assert payload["velocity"] == 3.0
    assert payload["acceleration"] == 0.6


def run_all():
    test_local_output_lpf_passthroughs_first_state()
    test_local_output_lpf_blends_and_wraps_heading()
    test_local_output_lpf_fills_acceleration_for_legacy_4d_state()
    test_broadcast_uses_filtered_local_state()
    print("All VehicleObserver local-output LPF tests passed.")


if __name__ == "__main__":
    run_all()
