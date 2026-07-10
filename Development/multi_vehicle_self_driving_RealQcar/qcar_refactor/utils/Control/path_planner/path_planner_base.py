"""
Base path planner interface.

Path planners convert the current vehicle state into a planner target. They
should not know about vehicle hardware, observers, controllers, GUI, or V2V.
"""

from abc import ABC, abstractmethod
from typing import Iterable, Optional

from core.types import PlannerTarget, VehicleStateEstimate


class PathPlannerBase(ABC):
    """Interface every path planner implementation should follow."""

    @abstractmethod
    def load_path(self, path_source) -> None:
        """Load waypoints from a file path or iterable of points."""
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset internal progress to the beginning of the path."""
        ...

    @abstractmethod
    def update(self, state: VehicleStateEstimate) -> PlannerTarget:
        """Return the current target for the controller."""
        ...

    @abstractmethod
    def set_target_velocity(self, target_velocity: float) -> None:
        """Set the desired path-following speed."""
        ...

    @abstractmethod
    def is_finished(self) -> bool:
        """Return True after the final waypoint has been reached."""
        ...


class PathPlannerNull(PathPlannerBase):
    """Safe no-op planner for tests and bring-up."""

    def __init__(self, target_velocity: float = 0.0) -> None:
        self._target_velocity = float(target_velocity)
        self._finished = True

    def load_path(self, path_source) -> None:
        self._finished = True

    def reset(self) -> None:
        self._finished = True

    def update(self, state: VehicleStateEstimate) -> PlannerTarget:
        return PlannerTarget(
            target_x=float(state.x),
            target_y=float(state.y),
            target_theta=float(state.theta),
            target_velocity=0.0,
            is_finished=True,
        )

    def set_target_velocity(self, target_velocity: float) -> None:
        self._target_velocity = float(target_velocity)

    def is_finished(self) -> bool:
        return self._finished
