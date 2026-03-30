"""
First-Order Lag Motor Model for QCar2

Extracted from Calibration/06_throttle_acceleration_model_calibration.py.
Provides a physically-accurate throttle-to-acceleration mapping using
calibrated parameters (tau, k_th, k_br, c_v, c_fric).

Model equations (discrete):
    motor_accel_req = k_th * max(u, 0) - k_br * max(-u, 0)
    motor_accel[k+1] = motor_accel[k] + (dt/tau) * (motor_accel_req - motor_accel[k])
    drag_accel = c_v * v + c_fric * sign(v)
    kinematic_accel = motor_accel - drag_accel
    v[k+1] = v + kinematic_accel * dt

Default parameters are from QLabs calibration (throttle_drag_model.yaml).
For real QCar hardware, override tau and gains from motor_model_id.yaml.

References:
    - Calibration/results/06_throttle_acceleration_model_calibration/throttle_drag_model.yaml
    - Calibration/results/02_motor_model_identification/motor_model_id.yaml
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass
class MotorModelConfig:
    """Configuration for the first-order lag motor model."""

    enabled: bool = False
    # Motor time constant (first-order lag) [s]
    #   QLabs calibrated: 0.318 s
    #   Real QCar hardware: ~1.82 s
    tau: float = 0.318
    # Throttle gain (positive throttle -> forward thrust) [m/s^2 per throttle unit]
    k_th: float = 4.5795
    # Brake gain (negative throttle -> reverse thrust) [m/s^2 per throttle unit]
    k_br: float = 4.5772
    # Velocity-proportional drag coefficient [1/s]
    c_v: float = 5.0
    # Coulomb (static) friction coefficient [m/s^2]
    c_fric: float = 2.0

    @classmethod
    def from_dict(cls, d: dict) -> "MotorModelConfig":
        """Create config from a dictionary (e.g., YAML section)."""
        return cls(
            enabled=d.get("enabled", False),
            tau=d.get("tau", 0.318),
            k_th=d.get("k_th", 4.5795),
            k_br=d.get("k_br", 4.5772),
            c_v=d.get("c_v", 5.0),
            c_fric=d.get("c_fric", 2.0),
        )


class AccelDragMotorModel:
    """
    Stateless first-order lag motor model for fleet estimation.

    Unlike the calibration script's AccelDragMotorSim, this version is
    *stateless* — it takes motor_accel as an input and returns the updated
    motor_accel along with predicted velocity and kinematic acceleration.
    This allows the fleet estimator to manage per-vehicle state externally.
    """

    def __init__(self, config: MotorModelConfig):
        self.cfg = config

    def predict(
        self,
        throttle: float,
        v: float,
        motor_accel: float,
        dt: float,
    ) -> tuple:
        """
        Predict velocity and acceleration given current state and throttle.

        Args:
            throttle: Throttle command (dimensionless, typically -0.3 to 0.3)
            v: Current velocity [m/s]
            motor_accel: Current internal motor acceleration state [m/s^2]
            dt: Time step [s]

        Returns:
            (v_new, a_kinematic, motor_accel_new):
                v_new           — predicted velocity [m/s]
                a_kinematic     — net kinematic acceleration [m/s^2]
                motor_accel_new — updated motor acceleration state [m/s^2]
        """
        if dt <= 0:
            return v, 0.0, motor_accel

        cfg = self.cfg

        # Decompose throttle into forward / brake
        u_th = max(throttle, 0.0)
        u_br = max(-throttle, 0.0)
        motor_accel_req = cfg.k_th * u_th - cfg.k_br * u_br

        # First-order lag on motor acceleration
        motor_accel_new = motor_accel + (dt / cfg.tau) * (
            motor_accel_req - motor_accel
        )

        # Drag / friction model
        # Matches the calibration YAML: kinematic_accel = motor_accel - c_v*v - c_fric*sign(v)
        # Use smoothed sign (tanh) near v=0 to avoid discontinuous friction chattering
        smooth_sign = np.tanh(v / 0.02)  # transitions over ~±2 cm/s
        drag_accel = cfg.c_v * v + cfg.c_fric * smooth_sign

        # Net kinematic acceleration
        a_kinematic = motor_accel_new - drag_accel

        # Velocity update
        v_new = v + a_kinematic * dt

        return float(v_new), float(a_kinematic), float(motor_accel_new)
