"""Bounded 2D direct-predecessor fleet follower controller."""

from __future__ import annotations

import math

from core.types import ControlInput, ControllerReference, VehicleStateEstimate
from .controller_fleet_base import ControllerFleetBase
def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ControllerFleet2D(ControllerFleetBase):
    """Follow a virtual target behind the direct predecessor, never its centre."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        super().__init__(config, vehicle_id, logger)
        self.steering_gain = float(self._config.get("steering_gain", 1.0))
        self.max_steering = abs(float(self._config.get("max_steering", 0.48)))

    def compute(self, state: VehicleStateEstimate, target: ControllerReference, dt: float) -> ControlInput:
        del dt
        desired_gap = self.desired_gap_m + self.time_headway_s * max(0.0, state.velocity)
        virtual_x = target.target_x - desired_gap * math.cos(target.target_theta)
        virtual_y = target.target_y - desired_gap * math.sin(target.target_theta)
        distance = math.hypot(target.target_x - state.x, target.target_y - state.y)
        target_velocity = self._clip(target.target_velocity + self.gap_gain * (distance - desired_gap), 0.0, self.max_velocity)
        throttle = self._clip(self.kp_velocity * (target_velocity - state.velocity), self.min_throttle, self.max_throttle)
        heading = math.atan2(virtual_y - state.y, virtual_x - state.x)
        steering = self._clip(self.steering_gain * _wrap(heading - state.theta), -self.max_steering, self.max_steering)
        return ControlInput(throttle, steering, target_velocity, "fleet_2d_controller")
