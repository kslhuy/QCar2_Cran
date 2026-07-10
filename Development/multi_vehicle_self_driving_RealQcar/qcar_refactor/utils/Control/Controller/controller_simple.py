"""
Simple path-following controller.

This controller combines:
- PID velocity control for throttle
- pure-pursuit-style heading control for steering

It is intentionally small and independent from IO, observer, path loading, GUI,
and V2V. The runtime decides whether the car is allowed to drive before calling
this controller.
"""

import logging
import math

from core.types import ControlCommand, PlannerTarget, VehicleStateEstimate
from utils.control.controller.controller_base import ControllerBase


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class ControllerSimple(ControllerBase):
    """
    Minimal controller for static path following.

    The output is already clipped to configured throttle and steering limits.
    """

    def __init__(
        self,
        config: dict | None = None,
        vehicle_id: int = 0,
        logger=None,
        **overrides,
    ) -> None:
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._vehicle_id = vehicle_id
        config = dict(config or {})
        config.update(overrides)
        self.kp_velocity = float(config.get("kp_velocity", 0.2))
        self.ki_velocity = float(config.get("ki_velocity", 0.0))
        self.kd_velocity = float(config.get("kd_velocity", 0.0))
        self.feedforward_gain = float(config.get("feedforward_gain", 0.0))
        self.steering_gain = float(config.get("steering_gain", 1.0))
        self.max_throttle = abs(float(config.get("max_throttle", 0.10)))
        self.min_throttle = -abs(float(config.get("min_throttle", -0.10)))
        self.max_steering = abs(float(config.get("max_steering", 0.48)))
        self.integral_limit = abs(float(config.get("integral_limit", 1.0)))
        self._integral_error = 0.0
        self._previous_error = None

    def reset(self) -> None:
        self._integral_error = 0.0
        self._previous_error = None

    def compute(
        self,
        state: VehicleStateEstimate,
        target: PlannerTarget,
        dt: float,
    ) -> ControlCommand:
        if target.is_finished:
            self.reset()
            return self.zero_command(source="path_finished")

        safe_dt = max(float(dt), 1e-6)
        target_velocity = max(0.0, float(target.target_velocity))

        throttle = self._compute_throttle(
            current_velocity=float(state.velocity),
            target_velocity=target_velocity,
            dt=safe_dt,
        )
        steering = self._compute_steering(state, target)

        return ControlCommand(
            throttle=throttle,
            steering=steering,
            target_velocity=target_velocity,
            source="simple_path_controller",
        )

    def zero_command(self, source: str = "zero") -> ControlCommand:
        return ControlCommand(
            throttle=0.0,
            steering=0.0,
            target_velocity=0.0,
            source=source,
        )

    @property
    def integral_error(self) -> float:
        return self._integral_error

    def _compute_throttle(
        self,
        current_velocity: float,
        target_velocity: float,
        dt: float,
    ) -> float:
        error = target_velocity - current_velocity
        self._integral_error += error * dt
        self._integral_error = _clip(
            self._integral_error,
            -self.integral_limit,
            self.integral_limit,
        )

        if self._previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._previous_error) / dt
        self._previous_error = error

        throttle = (
            self.feedforward_gain * target_velocity
            + self.kp_velocity * error
            + self.ki_velocity * self._integral_error
            + self.kd_velocity * derivative
        )
        return _clip(throttle, self.min_throttle, self.max_throttle)

    def _compute_steering(
        self,
        state: VehicleStateEstimate,
        target: PlannerTarget,
    ) -> float:
        dx = float(target.target_x) - float(state.x)
        dy = float(target.target_y) - float(state.y)

        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            heading_error = _wrap_to_pi(float(target.target_theta) - float(state.theta))
        else:
            target_heading = math.atan2(dy, dx)
            heading_error = _wrap_to_pi(target_heading - float(state.theta))

        steering = self.steering_gain * heading_error
        return _clip(steering, -self.max_steering, self.max_steering)
