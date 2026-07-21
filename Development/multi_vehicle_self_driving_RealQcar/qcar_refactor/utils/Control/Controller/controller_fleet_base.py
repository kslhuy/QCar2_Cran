"""Base interface for fleet-specific command producers."""

from __future__ import annotations

from utils.control.controller.controller_base import ControllerBase


class ControllerFleetBase(ControllerBase):
    """Base for controllers that accept predecessor-derived references."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        super().__init__(config, vehicle_id, logger)
        self.desired_gap_m = float(self._config.get("desired_gap_m", 0.8))
        self.time_headway_s = float(self._config.get("time_headway_s", 0.4))
        self.gap_gain = float(self._config.get("gap_gain", 0.8))
        self.kp_velocity = float(self._config.get("kp_velocity", 0.3))
        self.max_velocity = float(self._config.get("max_velocity", 1.0))
        self.max_throttle = abs(float(self._config.get("max_throttle", 0.15)))
        self.min_throttle = -abs(float(self._config.get("min_throttle", -0.15)))

    @property
    def supports_fleet_reference(self) -> bool:
        return True

    def reset(self) -> None:
        return None

    @staticmethod
    def _clip(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, float(value)))
