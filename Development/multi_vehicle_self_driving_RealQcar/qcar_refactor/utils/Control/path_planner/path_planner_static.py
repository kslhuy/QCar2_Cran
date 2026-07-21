"""
Static waypoint path planner.

This planner is intentionally small:
- load a fixed waypoint list
- find a lookahead target from the current state
- report finished near the final waypoint

It does not compute throttle or steering. That remains the controller's job.
"""

import csv
import logging
import math
from typing import Iterable, Optional

import numpy as np

from core.types import ControllerReference, VehicleStateEstimate
from .path_planner_base import PathPlannerBase


class PathPlannerStatic(PathPlannerBase):
    """
    Minimal static waypoint planner.

    Waypoints are stored as an N x 3 array: [x, y, theta]. CSV files from the
    existing `path_rich_output/pp_waypoints.csv` are supported by default using
    columns 1, 2, and the last column.
    """

    def __init__(
        self,
        config: dict,
        vehicle_id: int = 0,
        logger=None,
        **overrides,
    ) -> None:
        effective_config = dict(config)
        effective_config.update(overrides)
        super().__init__(effective_config, vehicle_id, logger)
        path_source = self._config.get("path_source")
        self._waypoints = np.zeros((0, 3), dtype=float)
        self._target_velocity = float(self._config.get("target_velocity", 0.6))
        self._lookahead_distance = max(0.0, float(self._config.get("lookahead_distance", 0.5)))
        self._finish_tolerance = max(0.0, float(self._config.get("finish_tolerance", 0.15)))
        self._x_column = int(self._config.get("x_column", 1))
        self._y_column = int(self._config.get("y_column", 2))
        self._theta_column = int(self._config.get("theta_column", -1))
        self._max_search_ahead = max(1, int(self._config.get("max_search_ahead", 50)))
        self._current_index = 0
        self._finished = True

        if path_source is not None:
            self.load_path(path_source)

    def load_path(self, path_source) -> None:
        """Load waypoints from a CSV path or iterable."""
        if isinstance(path_source, (str, bytes)):
            points = self._load_csv(path_source)
        else:
            points = self._coerce_waypoints(path_source)

        if points.size == 0:
            self._waypoints = np.zeros((0, 3), dtype=float)
            self._finished = True
            self._current_index = 0
            self._log_warning("Loaded empty path")
            return

        self._waypoints = points
        self.reset()
        self._log_info(f"Loaded {len(self._waypoints)} waypoints")

    def reset(self) -> None:
        self._current_index = 0
        self._finished = len(self._waypoints) == 0

    def update(self, state: VehicleStateEstimate) -> ControllerReference:
        if len(self._waypoints) == 0:
            self._finished = True
            return ControllerReference(
                target_x=float(state.x),
                target_y=float(state.y),
                target_theta=float(state.theta),
                target_velocity=0.0,
                is_finished=True,
            )

        closest_index = self._find_closest_index(state)
        self._current_index = max(self._current_index, closest_index)

        target_index = self._find_lookahead_index(state)
        self._current_index = max(self._current_index, target_index)

        final = self._waypoints[-1]
        final_distance = self._distance_xy(state.x, state.y, final[0], final[1])
        if target_index == len(self._waypoints) - 1 and final_distance <= self._finish_tolerance:
            self._finished = True
            self._current_index = len(self._waypoints) - 1
            return self._make_target(final, is_finished=True)

        return self._make_target(self._waypoints[target_index], is_finished=False)

    def set_target_velocity(self, target_velocity: float) -> None:
        self._target_velocity = max(0.0, float(target_velocity))

    def is_finished(self) -> bool:
        return self._finished

    @property
    def current_index(self) -> int:
        return self._current_index

    @property
    def waypoints(self) -> np.ndarray:
        return self._waypoints.copy()

    def _find_closest_index(self, state: VehicleStateEstimate) -> int:
        search_end = min(
            len(self._waypoints),
            self._current_index + self._max_search_ahead + 1,
        )
        search = self._waypoints[self._current_index : search_end, :2]
        deltas = search - np.array([state.x, state.y], dtype=float)
        distances = np.linalg.norm(deltas, axis=1)
        return self._current_index + int(np.argmin(distances))

    def _find_lookahead_index(self, state: VehicleStateEstimate) -> int:
        for index in range(self._current_index, len(self._waypoints)):
            point = self._waypoints[index]
            distance = self._distance_xy(state.x, state.y, point[0], point[1])
            if distance >= self._lookahead_distance:
                return index
        return len(self._waypoints) - 1

    def _make_target(self, waypoint: np.ndarray, is_finished: bool) -> ControllerReference:
        return ControllerReference(
            target_x=float(waypoint[0]),
            target_y=float(waypoint[1]),
            target_theta=float(waypoint[2]),
            target_velocity=0.0 if is_finished else self._target_velocity,
            is_finished=bool(is_finished),
        )

    def _load_csv(self, path: str) -> np.ndarray:
        rows = []
        with open(path, newline="") as file_obj:
            reader = csv.reader(file_obj)
            for row in reader:
                numeric = self._parse_numeric_row(row)
                if numeric is not None:
                    rows.append(numeric)
        return self._coerce_waypoints(rows)

    def _parse_numeric_row(self, row) -> Optional[list]:
        try:
            return [float(value) for value in row]
        except (TypeError, ValueError):
            return None

    def _coerce_waypoints(self, path_source) -> np.ndarray:
        data = np.asarray(list(path_source), dtype=float)
        if data.size == 0:
            return np.zeros((0, 3), dtype=float)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 2:
            raise ValueError("Path must contain at least x and y columns")

        if data.shape[1] == 2:
            x = data[:, 0]
            y = data[:, 1]
            theta = self._heading_from_xy(x, y)
        elif data.shape[1] == 3:
            x = data[:, 0]
            y = data[:, 1]
            theta = data[:, 2]
        else:
            x = data[:, self._x_column]
            y = data[:, self._y_column]
            theta = data[:, self._theta_column]

        return np.column_stack((x, y, theta)).astype(float)

    def _heading_from_xy(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        theta = np.zeros_like(x, dtype=float)
        if len(x) < 2:
            return theta

        dx = np.diff(x)
        dy = np.diff(y)
        segment_heading = np.arctan2(dy, dx)
        theta[:-1] = segment_heading
        theta[-1] = segment_heading[-1]
        return theta

    def _distance_xy(self, x0: float, y0: float, x1: float, y1: float) -> float:
        return float(math.hypot(float(x1) - float(x0), float(y1) - float(y0)))

    def _log_info(self, message: str) -> None:
        if hasattr(self._logger, "logger"):
            self._logger.logger.info(message)
        else:
            self._logger.info(message)

    def _log_warning(self, message: str) -> None:
        if hasattr(self._logger, "log_warning"):
            self._logger.log_warning(message)
        elif hasattr(self._logger, "logger"):
            self._logger.logger.warning(message)
        else:
            self._logger.warning(message)
