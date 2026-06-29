"""Regression checks for Limo-specific CACC limiting."""

import numpy as np

from Controller.longitudinal_controllers import CACCLongitudinalController


def run_test():
    dt = 0.1

    controller = CACCLongitudinalController(
        vehicle_type="Limo",
        s0=0.2,
        h=0.5,
        K=np.array([[0.3, 0.15]]),
        leader_acceleration_weight=1.0,
        limo_max_speed=0.7,
        limo_max_accel=0.25,
        limo_max_decel=0.45,
        limo_leader_speed_margin=0.08,
        limo_gap_closing_gain=0.20,
        limo_close_gap_gain=0.60,
    )

    follower = {
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0,
        "velocity": 0.55,
        "acceleration": 0.0,
    }
    leader = {
        "x": 1.4,
        "y": 0.0,
        "theta": 0.0,
        "velocity": 0.55,
        "acceleration": 1.5,
    }

    cmd = controller.compute_throttle(follower, leader, dt)
    print("Limo cmd with aggressive leader acceleration:", cmd)
    assert cmd <= 0.7, "Limo command must stay under limo_max_speed"

    follower_close = dict(follower)
    follower_close["velocity"] = 0.60
    leader_close = dict(leader)
    leader_close["x"] = 0.15
    leader_close["velocity"] = 0.20
    leader_close["acceleration"] = 0.0

    controller.reset()
    cmd_close = controller.compute_throttle(follower_close, leader_close, dt)
    print("Limo cmd when gap is too small:", cmd_close)
    assert cmd_close <= leader_close["velocity"] + 0.08 + 1e-6


if __name__ == "__main__":
    run_test()
