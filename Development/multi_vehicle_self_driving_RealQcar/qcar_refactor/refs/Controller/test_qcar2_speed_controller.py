"""Regression checks for the qcar2_hardware-inspired speed controller."""

from Controller.longitudinal_controllers import QCar2SpeedController


def run_test():
    controller = QCar2SpeedController(
        kp=20.0,
        kd=0.1,
        km=0.0047,
        use_affine_feedforward=True,
        ff_speed_slope=6.63,
        ff_speed_intercept=-0.31,
        nominal_battery_voltage=12.0,
        max_throttle=0.3,
    )

    dt = 0.015
    follower = {
        "velocity": 0.0,
        "motor_tach": 0.0,
        "battery_voltage": 12.0,
        "target_velocity": 1.0,
    }

    u1 = controller.compute_throttle(follower, None, dt)
    print("Initial command:", u1)
    assert u1 > 0.15, "Feedforward should provide a substantial base throttle"

    follower["battery_voltage"] = 6.0
    controller.reset()
    u_low_batt = controller.compute_throttle(follower, None, dt)
    print("Low-battery compensated command:", u_low_batt)
    assert u_low_batt > u1, "Lower battery voltage should increase compensated command"

    follower["target_velocity"] = 0.0
    u_stop = controller.compute_throttle(follower, None, dt)
    print("Stop command:", u_stop)
    assert u_stop == 0.0, "Zero target speed should reset throttle command"


if __name__ == "__main__":
    run_test()
