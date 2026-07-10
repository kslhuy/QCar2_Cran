"""Unit test for CACC controller leader acceleration error weight."""
import numpy as np

from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Controller.longitudinal_controllers import CACCLongitudinalController


def run_test():
    # simple follower/leader states including accelerations
    follower = {"x": 0.0, "y": 0.0, "velocity": 1.0, "acceleration": 0.0}
    leader = {"x": 1.0, "y": 0.0, "velocity": 1.0, "acceleration": 0.5}
    dt = 0.1

    # baseline controller without acceleration weight
    ctrl1 = CACCLongitudinalController(leader_acceleration_weight=0.0)
    u1 = ctrl1.compute_throttle(follower, leader, dt)

    # controller with weight should react to acceleration error
    ctrl2 = CACCLongitudinalController(leader_acceleration_weight=1.0)
    u2 = ctrl2.compute_throttle(follower, leader, dt)

    print("Throttle without weight:", u1)
    print("Throttle with weight:", u2)
    assert u2 >= u1, "Expected additional error term to produce equal or larger throttle"

    # update the weight dynamically and verify change
    ctrl2.update_params({"leader_acceleration_weight": 0.0})
    u3 = ctrl2.compute_throttle(follower, leader, dt)
    assert abs(u3 - u1) < 1e-6, "update_params should modify weight"
    print("Parameter update successful")


if __name__ == "__main__":
    run_test()
