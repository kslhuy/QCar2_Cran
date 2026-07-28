"""Bounded longitudinal fleet follower for straight-road evaluation."""

from __future__ import annotations

from core.vehicle_types import ControlInput, ControllerReference, VehicleStateEstimate
from .controller_fleet_base import ControllerFleetBase


class ControllerFleetLongitudinal(ControllerFleetBase):
    """Use front-vehicle gap for speed control and leave steering at zero."""

    def compute(self, state: VehicleStateEstimate, target: ControllerReference, dt: float) -> ControlInput:
        del dt
        desired_gap = self.desired_gap_m + self.time_headway_s * max(0.0, state.velocity)
        gap = (target.target_x - state.x) if abs(target.target_theta) < 0.785 else (target.target_y - state.y)
        target_velocity = self._clip(target.target_velocity + self.gap_gain * (gap - desired_gap), 0.0, self.max_velocity)
        throttle = self._clip(self.kp_velocity * (target_velocity - state.velocity), self.min_throttle, self.max_throttle)
        return ControlInput(throttle, 0.0, target_velocity, "fleet_longitudinal_controller")
