"""Manual controller selected temporarily by the vehicle runtime."""

from __future__ import annotations

from math import isfinite
import time

from core.types import ControlInput, ControllerReference, VehicleStateEstimate

from .controller_base import ControllerBase


class ControllerManual(ControllerBase):
    """Return the latest operator input through the standard controller contract."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        super().__init__(config, vehicle_id, logger)
        self._timeout_s = _positive_float(self._config.get("command_timeout_s", 0.25), "command_timeout_s")
        self._max_throttle = _unit_interval(self._config.get("max_throttle", 0.35), "max_throttle")
        self._max_steering = _positive_float(self._config.get("max_steering", 0.30), "max_steering")
        self.reset()

    def reset(self) -> None:
        self._throttle = 0.0
        self._steering = 0.0
        self._last_input_at: float | None = None

    @property
    def uses_planner_completion(self) -> bool:
        """Manual operation is independent of the currently loaded path."""
        return False

    def set_input(self, throttle: float, steering: float, now_monotonic: float | None = None) -> None:
        """Store the newest validated operator input for the next compute call."""
        self._throttle = _clip(float(throttle), -self._max_throttle, self._max_throttle)
        self._steering = _clip(float(steering), -self._max_steering, self._max_steering)
        self._last_input_at = time.monotonic() if now_monotonic is None else float(now_monotonic)

    def input_age_s(self, now_monotonic: float | None = None) -> float | None:
        """Return local receive age for dashboard diagnostics."""
        if self._last_input_at is None:
            return None
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        return max(0.0, now - self._last_input_at)

    def compute(
        self,
        state: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
    ) -> ControlInput:
        """Return fresh input or a safe zero command after the deadman timeout."""
        age = self.input_age_s()
        if age is None or age > self._timeout_s:
            return ControlInput(0.0, 0.0, 0.0, "manual_timeout")
        return ControlInput(self._throttle, self._steering, 0.0, "manual_controller")


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _positive_float(value: object, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not isfinite(float(value)) or value <= 0.0:
        raise ValueError(f"manual controller {name} must be a positive finite number")
    return float(value)


def _unit_interval(value: object, name: str) -> float:
    result = _positive_float(value, name)
    if result > 1.0:
        raise ValueError(f"manual controller {name} must be in (0, 1]")
    return result
