"""Regression tests for adaptive trust scoring in turns."""

import os
import sys
import numpy as np


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.append(CURRENT_DIR)

from trust_model import TriPTrustModel, TrustConfig, VehicleData


def test_adaptive_accel_and_heading_in_turn():
    """Normal turning dynamics should not collapse local acceleration/heading trust."""
    model = TriPTrustModel(vehicle_id=0, config=TrustConfig())

    t0 = int(1.0e9)
    t1 = int(1.1e9)

    host0 = {"x": 0.0, "y": 0.0, "theta": 0.00, "velocity": 0.90, "acceleration": 0.10}
    target0 = VehicleData(
        vehicle_id=1,
        x=0.95,
        y=0.05,
        theta=0.06,
        velocity=0.95,
        acceleration=0.12,
        timestamp_ns=t0,
    )
    model.update_beacon_reception(1, True, t0 / 1e9)
    model.calculate_trust(
        host_state=host0,
        target_data=target0,
        current_time_ns=t0,
        has_fleet_data=True,
    )

    # Same turn, mild natural mismatch in acceleration and heading.
    host1 = {"x": 0.09, "y": 0.01, "theta": 0.18, "velocity": 0.98, "acceleration": 0.20}
    target1 = VehicleData(
        vehicle_id=1,
        x=1.02,
        y=0.12,
        theta=0.28,
        velocity=1.03,
        acceleration=0.30,
        timestamp_ns=t1,
    )
    model.update_beacon_reception(1, True, t1 / 1e9)
    trust = model.calculate_trust(
        host_state=host1,
        target_data=target1,
        current_time_ns=t1,
        has_fleet_data=True,
    )

    print(f"acceleration_score={trust.acceleration_score:.3f}")
    print(f"heading_score={trust.heading_score:.3f}")
    assert trust.acceleration_score > 0.60
    assert trust.heading_score > 0.55


def test_theta_contribution_is_bounded_in_mahalanobis():
    """Theta term should be bounded to avoid dominating global trust in turns."""
    cfg = TrustConfig(theta_contribution_cap=1.8)
    model = TriPTrustModel(vehicle_id=0, config=cfg)

    # Large heading discrepancy with close position/velocity context.
    x_host = np.array([10.0, 3.0, 2.70, 1.00, 0.10], dtype=float)
    x_peer = np.array([10.1, 2.9, -2.60, 1.05, 0.15], dtype=float)

    dist, contributions = model._mahalanobis_components(x_host, x_peer, yaw_rate=1.2)
    print(f"mahalanobis_contributions={contributions}")
    assert contributions.shape[0] == 5
    assert contributions[2] <= 1.8 + 1e-9
    assert abs(dist - float(np.sum(contributions))) <= 1e-9


def run_all():
    test_adaptive_accel_and_heading_in_turn()
    test_theta_contribution_is_bounded_in_mahalanobis()
    print("All adaptive trust-model tests passed.")


if __name__ == "__main__":
    run_all()
