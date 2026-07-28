"""Runtime selection and delegation for controller implementations."""

from __future__ import annotations

from collections.abc import Callable, Mapping

from core.vehicle_types import ControlInput, ControllerReference, VehicleStateEstimate
from utils.control.controller.controller_base import ControllerBase

from .manager_base import ManagerBase


class ControllerCapabilityError(RuntimeError):
    """Raised when the selected controller cannot consume a fleet reference."""


class ControllerManager(ManagerBase[ControllerBase]):
    """Select, lazily construct, and delegate to configured controllers."""

    def __init__(
        self,
        configured_controller: ControllerBase,
        builders: Mapping[str, Callable[[], ControllerBase]] | None = None,
    ) -> None:
        super().__init__(configured_controller, builders)

    @property
    def supports_fleet_reference(self) -> bool:
        return bool(getattr(self._active, "supports_fleet_reference", False))

    @property
    def uses_planner_completion(self) -> bool:
        return bool(getattr(self._active, "uses_planner_completion", True))

    def select(self, name: str) -> ControllerBase:
        controller = super().select(name)
        controller.reset()
        return controller

    def reset(self) -> None:
        """Reset the selected controller without changing its profile."""
        self._active.reset()

    def set_input(self, throttle: float, steering: float) -> None:
        setter = getattr(self._active, "set_input", None)
        if not callable(setter):
            raise RuntimeError(f"Controller '{self.active_name}' does not accept direct input")
        setter(throttle, steering)

    def input_age_s(self) -> float | None:
        getter = getattr(self._active, "input_age_s", None)
        return getter() if callable(getter) else None

    def compute(
        self,
        state: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
    ) -> ControlInput:
        return self._active.compute(state, target, dt)

    def compute_fleet(
        self,
        state: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
    ) -> ControlInput:
        if not self.supports_fleet_reference:
            raise ControllerCapabilityError("selected controller does not support fleet references")
        return self.compute(state, target, dt)

    def _validate_utility(self, utility: object, name: str) -> None:
        if not callable(getattr(utility, "reset", None)) or not callable(getattr(utility, "compute", None)):
            raise TypeError(f"{name} must provide reset() and compute()")
