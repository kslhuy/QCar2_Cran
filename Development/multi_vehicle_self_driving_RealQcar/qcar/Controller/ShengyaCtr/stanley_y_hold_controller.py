"""
Stanley Variant for Fixed Y-Lane Holding.

This controller is designed for experiments where each vehicle should stay
near a fixed lateral position y_ref (for example y = -0.62).
"""

import math
from threading import Lock
from typing import Dict, Optional

import numpy as np


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class StanleyYHoldController:
    """
    Stanley-style lateral controller that tracks a fixed global y reference.

    Control law:
        delta = k_heading * heading_error + atan2(-k_e * e_y, v + k_soft)

    Where:
        e_y = y - y_ref
    """

    def __init__(
        self,
        y_ref: float = -0.62,
        heading_ref: float = 0.0,
        k_e: float = 0.6,
        k_soft: float = 0.5,
        k_heading: float = 2.2,
        use_leader_heading: bool = False,
        max_steering: float = 0.55,
        config=None,
        logger=None,
        **kwargs,
    ):
        self.logger = logger

        if config and hasattr(config, "get_lateral_params"):
            params = config.get_lateral_params("stanley_y_hold")
            self.y_ref = params.get("y_ref", y_ref)
            self.heading_ref = params.get("heading_ref", heading_ref)
            self.k_e = params.get("k_e", k_e)
            self.k_soft = params.get("k_soft", k_soft)
            self.k_heading = params.get("k_heading", k_heading)
            self.use_leader_heading = params.get("use_leader_heading", use_leader_heading)
            self.max_steering_angle = params.get("max_steering", max_steering)
        else:
            self.y_ref = y_ref
            self.heading_ref = heading_ref
            self.k_e = k_e
            self.k_soft = k_soft
            self.k_heading = k_heading
            self.use_leader_heading = use_leader_heading
            self.max_steering_angle = max_steering

        self.cross_track_error = 0.0
        self.heading_error = 0.0
        self._lock = Lock()

    def _compute_steering(self, y: float, theta: float, speed: float, theta_ref: float) -> float:
        """Internal Stanley-style computation for fixed y reference."""
        crosstrack_error = y - self.y_ref
        heading_error = wrap_to_pi(theta_ref - theta)

        cte_steering = math.atan2(-self.k_e * crosstrack_error, speed + self.k_soft)
        steering_cmd = self.k_heading * heading_error + cte_steering

        with self._lock:
            self.cross_track_error = crosstrack_error
            self.heading_error = heading_error

        return float(np.clip(steering_cmd, -self.max_steering_angle, self.max_steering_angle))

    def compute_steering(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        """Compute steering for FOLLOWING_LEADER-like interfaces."""
        _ = dt  # Interface compatibility

        y = follower_state["y"]
        theta = follower_state["theta"]
        speed = follower_state.get("velocity", 0.0)

        if self.use_leader_heading and leader_state is not None:
            theta_ref = leader_state.get("theta", self.heading_ref)
        else:
            theta_ref = self.heading_ref

        return self._compute_steering(y=y, theta=theta, speed=speed, theta_ref=theta_ref)

    def update(self, p: np.ndarray, th: float, speed: float) -> float:
        """Compute steering for FOLLOWING_PATH-like interfaces."""
        y = float(p[1])
        return self._compute_steering(y=y, theta=th, speed=speed, theta_ref=self.heading_ref)

    def get_waypoint_index(self) -> int:
        """Compatibility method for path-following progress monitoring."""
        return 0

    def get_reference_pose(self) -> tuple:
        """Return virtual reference pose for telemetry."""
        return np.array([0.0, self.y_ref]), self.heading_ref

    def get_errors(self) -> tuple:
        """Get current control errors for telemetry."""
        with self._lock:
            return self.cross_track_error, self.heading_error

    def reset(self, waypoints: Optional[np.ndarray] = None):
        """Reset controller state. Waypoints are ignored by design."""
        _ = waypoints
        with self._lock:
            self.cross_track_error = 0.0
            self.heading_error = 0.0
