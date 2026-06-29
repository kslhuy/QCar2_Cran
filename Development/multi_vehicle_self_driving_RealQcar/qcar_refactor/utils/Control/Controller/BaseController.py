"""
Base controller interface.

Controllers convert an estimated vehicle state plus a planner target into a
vehicle command. They should not read sensors, load paths, write hardware, or
know about GUI/V2V.
"""

import logging
from abc import ABC, abstractmethod

from core.Types import ControlCommand, PlannerTarget, VehicleStateEstimate


class BaseController(ABC):
    """Interface every controller implementation should follow."""

    @abstractmethod
    def reset(self) -> None:
        """Reset internal controller memory, such as PID integral error."""
        ...

    @abstractmethod
    def compute(
        self,
        state: VehicleStateEstimate,
        target: PlannerTarget,
        dt: float,
    ) -> ControlCommand:
        """Compute one throttle/steering command."""
        ...


class NullController(BaseController):
    """Safe no-op controller for tests and startup."""

    def __init__(self, logger=None) -> None:
        self._logger = logger or logging.getLogger(self.__class__.__name__)

    def reset(self) -> None:
        pass

    def compute(
        self,
        state: VehicleStateEstimate,
        target: PlannerTarget,
        dt: float,
    ) -> ControlCommand:
        return ControlCommand(
            throttle=0.0,
            steering=0.0,
            target_velocity=0.0,
            source="null_controller",
        )
