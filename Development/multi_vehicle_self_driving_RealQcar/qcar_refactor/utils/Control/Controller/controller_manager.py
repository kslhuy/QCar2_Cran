"""Runtime selection of configured controller implementations."""

from __future__ import annotations

from collections.abc import Callable, Mapping

from core.types import ControlInput, ControllerReference, VehicleStateEstimate

from .controller_base import ControllerBase


class ControllerCapabilityError(RuntimeError):
    """Raised when the selected controller cannot consume a requested reference."""


class ControllerManager:
    """Select, lazily construct, and delegate to configured controllers.

    The manager owns controller lifetime only. It does not read commands,
    inspect fleet state, or write to IO. A caller may select only profiles
    explicitly supplied by the composition root.
    """

    CONFIGURED = "configured"

    def __init__(
        self,
        configured_controller: ControllerBase,
        builders: Mapping[str, Callable[[], ControllerBase]] | None = None,
    ) -> None:
        _validate_controller(configured_controller, "configured_controller")
        self._controllers: dict[str, ControllerBase] = {self.CONFIGURED: configured_controller}
        self._builders = dict(builders or {})
        if self.CONFIGURED in self._builders:
            raise ValueError("'configured' is reserved for the configured controller")
        self._active_name = self.CONFIGURED

    @property
    def active_name(self) -> str:
        """Return the selected controller profile name."""
        return self._active_name

    @property
    def supports_fleet_reference(self) -> bool:
        """Expose the selected controller capability to the runtime."""
        return bool(getattr(self._active, "supports_fleet_reference", False))

    @property
    def uses_planner_completion(self) -> bool:
        """Return whether the active controller treats planner completion as a stop."""
        return bool(getattr(self._active, "uses_planner_completion", True))

    def has_profile(self, name: str) -> bool:
        """Return whether a named controller profile is configured."""
        return name in self._controllers or name in self._builders

    def is_selected(self, name: str) -> bool:
        """Return whether a named controller profile is currently active."""
        return self._active_name == name

    def select(self, name: str) -> ControllerBase:
        """Select a configured controller profile and reset its local state."""
        if not isinstance(name, str) or not name:
            raise ValueError("controller profile name must be a non-empty string")
        controller = self._controllers.get(name)
        if controller is None:
            builder = self._builders.get(name)
            if builder is None:
                raise KeyError(f"Controller profile is not configured: '{name}'")
            controller = builder()
            _validate_controller(controller, f"Controller builder '{name}' result")
            self._controllers[name] = controller
        self._active_name = name
        controller.reset()
        return controller

    def restore_configured(self) -> ControllerBase:
        """Restore the configured controller and reset its local state."""
        return self.select(self.CONFIGURED)

    def reset(self) -> None:
        """Reset the selected controller without changing the active profile."""
        self._active.reset()

    def set_input(self, throttle: float, steering: float) -> None:
        """Pass operator input only to an active input-capable controller."""
        setter = getattr(self._active, "set_input", None)
        if not callable(setter):
            raise RuntimeError(f"Controller '{self._active_name}' does not accept direct input")
        setter(throttle, steering)

    def input_age_s(self) -> float | None:
        """Return active controller input age when it exposes that diagnostic."""
        getter = getattr(self._active, "input_age_s", None)
        return getter() if callable(getter) else None

    def compute(
        self,
        state: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
    ) -> ControlInput:
        """Compute one command with the selected controller."""
        return self._active.compute(state, target, dt)

    def compute_fleet(
        self,
        state: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
    ) -> ControlInput:
        """Compute from a predecessor-derived reference with capability validation."""
        if not self.supports_fleet_reference:
            raise ControllerCapabilityError("selected controller does not support fleet references")
        return self.compute(state, target, dt)

    @property
    def _active(self) -> ControllerBase:
        return self._controllers[self._active_name]


def _validate_controller(controller: object, name: str) -> None:
    """Accept ControllerBase implementations and minimal test doubles."""
    if not callable(getattr(controller, "reset", None)) or not callable(getattr(controller, "compute", None)):
        raise TypeError(f"{name} must provide reset() and compute()")
