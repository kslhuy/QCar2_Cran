"""path_rich.py: Lightweight rich trajectory utilities built on SDCSRoadMap.

This module extends the existing node-sequence path generation workflow by:
- producing richer trajectory data: [s, x, y, heading, curvature, vx, ax]
- optionally applying a simple local obstacle-avoidance offset in Cartesian space

It is intentionally lightweight and pure Python so it can be integrated quickly
without adopting a full ROS local-planning stack.
"""

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence

import numpy as np

from hal.products.mats import SDCSRoadMap


@dataclass
class LocalObstacle:
    """Simple circular obstacle model used by RichSDCSPlanner."""

    x: float
    y: float
    radius: float = 0.20
    influence: float = 0.75
    clearance: float = 0.10
    vx: float = 0.0
    vy: float = 0.0

    def position_at(self, dt: float) -> np.ndarray:
        """Predict obstacle position after dt seconds (constant velocity)."""
        return np.array(
            [self.x + self.vx * dt, self.y + self.vy * dt],
            dtype=float,
        )


@dataclass
class SpeedZone:
    """Manual speed shaping over a path-distance interval [s_start, s_end]."""

    s_start: float
    s_end: float
    speed_scale: float = 1.0
    min_speed: Optional[float] = None
    max_speed: Optional[float] = None


class RichSDCSPlanner:
    """SDCS roadmap wrapper that returns rich trajectories.

    Trajectory columns:
    [s, x, y, heading, curvature, vx, ax]
    """

    def __init__(
        self,
        leftHandTraffic: bool = False,
        useSmallMap: bool = False,
        sample_ds: float = 0.02,
        is_cyclic: bool = True,
        trajectory_smoothing: float = 0.35,
    ):
        self.roadmap = SDCSRoadMap(
            leftHandTraffic=leftHandTraffic,
            useSmallMap=useSmallMap,
        )
        self.sample_ds = float(sample_ds)
        self.is_cyclic = bool(is_cyclic)
        self.trajectory_smoothing = float(np.clip(trajectory_smoothing, 0.0, 0.95))

        # Cached route and previous local plan for real-time replanning.
        self._base_path: Optional[np.ndarray] = None
        self._base_s: Optional[np.ndarray] = None
        self._last_local_traj: Optional[np.ndarray] = None
        self._speed_zones: List[SpeedZone] = []

    def generate_path(self, nodeSequence: Sequence[int]) -> Optional[np.ndarray]:
        """Generate a 2xN xy path from node sequence."""
        return self.roadmap.generate_path(nodeSequence=nodeSequence)

    def set_route(self, nodeSequence: Sequence[int]) -> Optional[np.ndarray]:
        """Cache a global route for repeated real-time local replanning."""
        path_xy = self.generate_path(nodeSequence=nodeSequence)
        if path_xy is None or path_xy.size == 0:
            self.clear_route()
            return None

        self._base_path = self._resample_path(path_xy, self.sample_ds)
        self._base_s = self._path_s(self._base_path[0, :], self._base_path[1, :])
        self._last_local_traj = None
        return self._base_path.copy()

    def clear_route(self) -> None:
        """Clear cached route and any planner temporal state."""
        self._base_path = None
        self._base_s = None
        self._last_local_traj = None

    def set_speed_zones(self, zones: Optional[Iterable[SpeedZone]]) -> None:
        """Set planner-level speed zones in route-distance coordinates [m]."""
        if zones is None:
            self._speed_zones = []
            return
        self._speed_zones = self._resolve_speed_zones(speed_zones=list(zones))

    def clear_speed_zones(self) -> None:
        """Remove all planner-level speed zones."""
        self._speed_zones = []

    def generate_rich_trajectory(
        self,
        nodeSequence: Sequence[int],
        desired_speed: float = 0.60,
        min_speed: float = 0.15,
        kappa_speed_gain: float = 2.0,
        obstacles: Optional[Iterable[LocalObstacle]] = None,
        max_speed: Optional[float] = None,
        speed_zones: Optional[Iterable[SpeedZone]] = None,
        hard_turn_kappa: Optional[float] = None,
        hard_turn_speed: Optional[float] = None,
    ) -> Optional[np.ndarray]:
        """Generate a rich trajectory from a node sequence.

        Returns:
            Nx7 array with columns [s, x, y, heading, curvature, vx, ax]
            or None if no path exists.
        """
        path_xy = self.generate_path(nodeSequence=nodeSequence)
        if path_xy is None or path_xy.size == 0:
            return None

        path_xy = self._resample_path(path_xy, self.sample_ds)

        if obstacles:
            path_xy = self.apply_local_avoidance(path_xy, obstacles)
            path_xy = self._resample_path(path_xy, self.sample_ds)

        active_speed_zones = self._resolve_speed_zones(speed_zones=speed_zones)
        return self._to_rich_trajectory(
            path_xy=path_xy,
            desired_speed=desired_speed,
            min_speed=min_speed,
            kappa_speed_gain=kappa_speed_gain,
            max_speed=max_speed,
            speed_zones=active_speed_zones,
            hard_turn_kappa=hard_turn_kappa,
            hard_turn_speed=hard_turn_speed,
        )

    def path_to_rich_trajectory(
        self,
        path_xy: np.ndarray,
        desired_speed: float = 0.60,
        min_speed: float = 0.15,
        kappa_speed_gain: float = 2.0,
        obstacles: Optional[Iterable[LocalObstacle]] = None,
        max_speed: Optional[float] = None,
        speed_zones: Optional[Iterable[SpeedZone]] = None,
        hard_turn_kappa: Optional[float] = None,
        hard_turn_speed: Optional[float] = None,
    ) -> Optional[np.ndarray]:
        """Convert any custom 2xN path into a rich Nx7 trajectory."""
        if path_xy is None or path_xy.size == 0:
            return None
        if path_xy.ndim != 2 or path_xy.shape[0] != 2:
            raise ValueError("path_xy must be a 2xN array.")

        path_xy = self._resample_path(path_xy, self.sample_ds)
        if obstacles:
            path_xy = self.apply_local_avoidance(path_xy, obstacles)
            path_xy = self._resample_path(path_xy, self.sample_ds)

        active_speed_zones = self._resolve_speed_zones(speed_zones=speed_zones)
        return self._to_rich_trajectory(
            path_xy=path_xy,
            desired_speed=desired_speed,
            min_speed=min_speed,
            kappa_speed_gain=kappa_speed_gain,
            max_speed=max_speed,
            speed_zones=active_speed_zones,
            hard_turn_kappa=hard_turn_kappa,
            hard_turn_speed=hard_turn_speed,
        )

    def update_realtime(
        self,
        current_pose: Sequence[float],
        obstacles: Optional[Iterable[LocalObstacle]] = None,
        lookahead_distance: float = 3.0,
        behind_distance: float = 0.5,
        prediction_time: float = 0.25,
        desired_speed: float = 0.60,
        min_speed: float = 0.15,
        kappa_speed_gain: float = 2.0,
        max_speed: Optional[float] = None,
        speed_zones: Optional[Iterable[SpeedZone]] = None,
        hard_turn_kappa: Optional[float] = None,
        hard_turn_speed: Optional[float] = None,
    ) -> Optional[np.ndarray]:
        """Replan a local rich trajectory around the current pose.

        This method expects a cached route from set_route(...), then replans
        only a rolling local horizon each cycle for real-time usage.
        """
        if self._base_path is None:
            raise RuntimeError(
                "No cached route available. Call set_route(nodeSequence) first."
            )

        if current_pose is None or len(current_pose) < 2:
            raise ValueError("current_pose must contain at least [x, y].")

        p = np.asarray(current_pose, dtype=float).reshape(-1)
        x0, y0 = float(p[0]), float(p[1])

        base_x = self._base_path[0, :]
        base_y = self._base_path[1, :]

        idx_center = int(np.argmin(np.hypot(base_x - x0, base_y - y0)))
        ds_nom = self._nominal_spacing(base_x, base_y)
        n_back = max(int(np.ceil(behind_distance / max(ds_nom, 1e-6))), 1)
        n_front = max(int(np.ceil(lookahead_distance / max(ds_nom, 1e-6))), 2)

        seg_idx = self._segment_indices(
            center_idx=idx_center,
            n_back=n_back,
            n_front=n_front,
            n_points=self._base_path.shape[1],
            is_cyclic=self.is_cyclic,
        )
        if seg_idx.size < 3:
            return None

        local_base = self._base_path[:, seg_idx]
        local_path = local_base.copy()

        if obstacles:
            predicted = self._predict_obstacles(
                obstacles=obstacles,
                prediction_time=prediction_time,
            )
            local_avoided = self.apply_local_avoidance(local_base, predicted)
            local_path = self._blend_avoidance(local_base, local_avoided)

        local_path = self._resample_path(local_path, self.sample_ds)

        if self._last_local_traj is not None and self.trajectory_smoothing > 0.0:
            local_path = self._smooth_xy_path(
                new_path_xy=local_path,
                prev_traj=self._last_local_traj,
                alpha=self.trajectory_smoothing,
            )

        active_speed_zones = self._resolve_speed_zones(speed_zones=speed_zones)
        speed_s = None
        if active_speed_zones:
            speed_s = self._map_path_to_base_s(path_xy=local_path)

        traj = self._to_rich_trajectory(
            path_xy=local_path,
            desired_speed=desired_speed,
            min_speed=min_speed,
            kappa_speed_gain=kappa_speed_gain,
            max_speed=max_speed,
            speed_zones=active_speed_zones,
            speed_s=speed_s,
            hard_turn_kappa=hard_turn_kappa,
            hard_turn_speed=hard_turn_speed,
        )
        self._last_local_traj = traj
        return traj

    def apply_local_avoidance(
        self,
        path_xy: np.ndarray,
        obstacles: Iterable[LocalObstacle],
    ) -> np.ndarray:
        """Apply a lightweight lateral offset around nearby obstacles.

        This is a heuristic nudge, not a full optimization-based local planner.
        """
        x = path_xy[0, :].copy()
        y = path_xy[1, :].copy()
        n_pts = x.shape[0]
        if n_pts < 3:
            return path_xy

        obstacle_list = list(obstacles)
        if not obstacle_list:
            return path_xy

        tangents = self._compute_tangents(x, y)
        normals = np.vstack((-tangents[1, :], tangents[0, :]))
        offsets = np.zeros(n_pts, dtype=float)

        ds_nom = self._nominal_spacing(x, y)
        for obs in obstacle_list:
            dist = np.hypot(x - obs.x, y - obs.y)
            idx = int(np.argmin(dist))
            nearest = float(dist[idx])
            if nearest > obs.influence:
                continue

            rel = np.array([obs.x - x[idx], obs.y - y[idx]])
            side = np.sign(np.dot(normals[:, idx], rel))
            if side == 0:
                side = 1.0

            amp = (obs.radius + obs.clearance) * (
                1.0 - nearest / max(obs.influence, 1e-6)
            )
            sigma = max(obs.influence / max(ds_nom, 1e-3), 1.0)
            index_axis = np.arange(n_pts, dtype=float)
            weight = np.exp(-0.5 * ((index_axis - float(idx)) / sigma) ** 2)
            offsets += (-side * amp) * weight

        max_shift = max(o.radius + o.clearance for o in obstacle_list)
        offsets = np.clip(offsets, -1.5 * max_shift, 1.5 * max_shift)

        x_shifted = x + offsets * normals[0, :]
        y_shifted = y + offsets * normals[1, :]
        return np.vstack((x_shifted, y_shifted))

    @staticmethod
    def _segment_indices(
        center_idx: int,
        n_back: int,
        n_front: int,
        n_points: int,
        is_cyclic: bool,
    ) -> np.ndarray:
        raw = np.arange(center_idx - n_back, center_idx + n_front + 1, dtype=int)
        if is_cyclic:
            return np.mod(raw, n_points)

        valid = raw[(raw >= 0) & (raw < n_points)]
        if valid.size == 0:
            return np.array([center_idx], dtype=int)
        return valid

    @staticmethod
    def _blend_avoidance(
        base_path_xy: np.ndarray,
        avoided_path_xy: np.ndarray,
        edge_fraction: float = 0.20,
    ) -> np.ndarray:
        """Fade avoidance to zero near local horizon boundaries."""
        n_pts = base_path_xy.shape[1]
        if n_pts < 5:
            return avoided_path_xy

        edge_pts = max(1, int(n_pts * edge_fraction))
        edge_pts = min(edge_pts, n_pts // 2)
        if edge_pts < 1:
            return avoided_path_xy

        w = np.ones(n_pts, dtype=float)
        ramp = np.linspace(0.0, 1.0, edge_pts, endpoint=True)
        w[:edge_pts] = ramp
        w[-edge_pts:] = ramp[::-1]

        delta = avoided_path_xy - base_path_xy
        return base_path_xy + delta * w[np.newaxis, :]

    @staticmethod
    def _predict_obstacles(
        obstacles: Iterable[LocalObstacle],
        prediction_time: float,
    ) -> Iterable[LocalObstacle]:
        pred = []
        dt = max(float(prediction_time), 0.0)
        for obs in obstacles:
            p = obs.position_at(dt)
            pred.append(
                LocalObstacle(
                    x=float(p[0]),
                    y=float(p[1]),
                    radius=obs.radius,
                    influence=obs.influence,
                    clearance=obs.clearance,
                    vx=obs.vx,
                    vy=obs.vy,
                )
            )
        return pred

    @staticmethod
    def _smooth_xy_path(
        new_path_xy: np.ndarray,
        prev_traj: np.ndarray,
        alpha: float,
    ) -> np.ndarray:
        """Temporal smoothing of local path to reduce frame-to-frame jitter."""
        if prev_traj is None or prev_traj.size == 0 or alpha <= 0.0:
            return new_path_xy

        prev_xy = np.vstack((prev_traj[:, 1], prev_traj[:, 2]))
        s_new = RichSDCSPlanner._path_s(new_path_xy[0, :], new_path_xy[1, :])
        s_prev = RichSDCSPlanner._path_s(prev_xy[0, :], prev_xy[1, :])

        x_prev_i = np.interp(s_new, s_prev, prev_xy[0, :])
        y_prev_i = np.interp(s_new, s_prev, prev_xy[1, :])

        x = (1.0 - alpha) * new_path_xy[0, :] + alpha * x_prev_i
        y = (1.0 - alpha) * new_path_xy[1, :] + alpha * y_prev_i
        return np.vstack((x, y))

    def _resolve_speed_zones(
        self,
        speed_zones: Optional[Iterable[SpeedZone]],
    ) -> List[SpeedZone]:
        zones_in = self._speed_zones if speed_zones is None else list(speed_zones)
        zones_out: List[SpeedZone] = []

        for zone in zones_in:
            if not isinstance(zone, SpeedZone):
                raise TypeError(
                    "speed_zones must contain SpeedZone entries. "
                    f"Got {type(zone).__name__}."
                )

            s0 = float(zone.s_start)
            s1 = float(zone.s_end)
            if s1 < s0:
                s0, s1 = s1, s0

            scale = float(zone.speed_scale)
            if scale < 0.0:
                raise ValueError("SpeedZone.speed_scale must be >= 0.0.")

            zmin = None if zone.min_speed is None else float(zone.min_speed)
            zmax = None if zone.max_speed is None else float(zone.max_speed)
            if zmin is not None and zmin < 0.0:
                raise ValueError("SpeedZone.min_speed must be >= 0.0.")
            if zmax is not None and zmax < 0.0:
                raise ValueError("SpeedZone.max_speed must be >= 0.0.")
            if zmin is not None and zmax is not None and zmin > zmax:
                raise ValueError(
                    "SpeedZone.min_speed must be <= SpeedZone.max_speed."
                )

            zones_out.append(
                SpeedZone(
                    s_start=s0,
                    s_end=s1,
                    speed_scale=scale,
                    min_speed=zmin,
                    max_speed=zmax,
                )
            )

        return zones_out

    def _map_path_to_base_s(self, path_xy: np.ndarray) -> Optional[np.ndarray]:
        if self._base_path is None or self._base_s is None:
            return None

        px = path_xy[0, :]
        py = path_xy[1, :]
        bx = self._base_path[0, :]
        by = self._base_path[1, :]

        dx = px[:, np.newaxis] - bx[np.newaxis, :]
        dy = py[:, np.newaxis] - by[np.newaxis, :]
        nearest_idx = np.argmin(dx * dx + dy * dy, axis=1)
        s_route = self._base_s[nearest_idx].astype(float)

        if self.is_cyclic and self._base_s.shape[0] > 1:
            s_route = self._unwrap_monotonic_s(
                s_route,
                s_period=float(self._base_s[-1]),
            )

        return np.maximum.accumulate(s_route)

    @staticmethod
    def _unwrap_monotonic_s(s: np.ndarray, s_period: float) -> np.ndarray:
        if s.size < 2 or s_period <= 0.0:
            return s.astype(float)

        out = s.astype(float).copy()
        offset = 0.0
        jump_threshold = 0.5 * s_period
        for i in range(1, out.size):
            cur = out[i] + offset
            prev = out[i - 1]
            if cur + jump_threshold < prev:
                offset += s_period
                cur = out[i] + offset
            out[i] = cur
        return out

    @staticmethod
    def _apply_speed_zones(
        vx: np.ndarray,
        s_axis: np.ndarray,
        speed_zones: Sequence[SpeedZone],
        min_speed: float,
        max_speed: float,
    ) -> np.ndarray:
        if not speed_zones:
            return vx

        v_out = vx.copy()
        for zone in speed_zones:
            mask = (s_axis >= zone.s_start) & (s_axis <= zone.s_end)
            if not np.any(mask):
                continue

            zone_v = v_out[mask] * zone.speed_scale
            if zone.min_speed is not None:
                zone_v = np.maximum(zone_v, zone.min_speed)
            if zone.max_speed is not None:
                zone_v = np.minimum(zone_v, zone.max_speed)

            zone_v = np.clip(zone_v, min_speed, max_speed)
            v_out[mask] = zone_v

        return v_out

    @staticmethod
    def _nominal_spacing(x: np.ndarray, y: np.ndarray) -> float:
        ds = np.hypot(np.diff(x), np.diff(y))
        if ds.size == 0:
            return 0.01
        return float(max(np.median(ds), 1e-4))

    @staticmethod
    def _compute_tangents(x: np.ndarray, y: np.ndarray) -> np.ndarray:
        dx = np.gradient(x)
        dy = np.gradient(y)
        mag = np.hypot(dx, dy) + 1e-9
        return np.vstack((dx / mag, dy / mag))

    @staticmethod
    def _path_s(x: np.ndarray, y: np.ndarray) -> np.ndarray:
        ds = np.hypot(np.diff(x), np.diff(y))
        return np.hstack(([0.0], np.cumsum(ds)))

    def _resample_path(self, path_xy: np.ndarray, ds_target: float) -> np.ndarray:
        x = path_xy[0, :]
        y = path_xy[1, :]
        s = self._path_s(x, y)
        s_total = float(s[-1])
        if s_total < 1e-6:
            return path_xy

        ds_target = max(float(ds_target), 1e-3)
        s_query = np.arange(0.0, s_total, ds_target)
        if s_query.size == 0 or s_query[-1] != s_total:
            s_query = np.hstack((s_query, s_total))

        x_new = np.interp(s_query, s, x)
        y_new = np.interp(s_query, s, y)
        return np.vstack((x_new, y_new))

    def _to_rich_trajectory(
        self,
        path_xy: np.ndarray,
        desired_speed: float,
        min_speed: float,
        kappa_speed_gain: float,
        max_speed: Optional[float] = None,
        speed_zones: Optional[Sequence[SpeedZone]] = None,
        speed_s: Optional[np.ndarray] = None,
        hard_turn_kappa: Optional[float] = None,
        hard_turn_speed: Optional[float] = None,
    ) -> np.ndarray:
        x = path_xy[0, :]
        y = path_xy[1, :]
        s = self._path_s(x, y)

        desired_speed = float(max(desired_speed, 0.0))
        min_speed = float(max(min_speed, 0.0))
        if max_speed is None:
            max_speed_eff = desired_speed
        else:
            max_speed_eff = float(max(max_speed, min_speed))
        max_speed_eff = float(max(max_speed_eff, min_speed))

        if x.shape[0] < 3:
            heading = np.zeros_like(x)
            kappa = np.zeros_like(x)
            vx = np.full_like(x, desired_speed, dtype=float)
            vx = np.clip(vx, min_speed, max_speed_eff)
            ax = np.zeros_like(x)
            return np.column_stack((s, x, y, heading, kappa, vx, ax))

        dx = np.gradient(x, s + 1e-6)
        dy = np.gradient(y, s + 1e-6)
        ddx = np.gradient(dx, s + 1e-6)
        ddy = np.gradient(dy, s + 1e-6)

        heading = np.unwrap(np.arctan2(dy, dx))
        denom = np.power(dx * dx + dy * dy, 1.5) + 1e-9
        kappa = (dx * ddy - dy * ddx) / denom

        vx = desired_speed / (1.0 + float(kappa_speed_gain) * np.abs(kappa))
        vx = np.clip(vx, min_speed, max_speed_eff)

        if speed_zones:
            if speed_s is None:
                s_axis = s
            else:
                s_axis = np.asarray(speed_s, dtype=float).reshape(-1)
                if s_axis.shape[0] != vx.shape[0]:
                    raise ValueError("speed_s must have the same length as path points.")

            vx = self._apply_speed_zones(
                vx=vx,
                s_axis=s_axis,
                speed_zones=speed_zones,
                min_speed=min_speed,
                max_speed=max_speed_eff,
            )

        if hard_turn_kappa is not None and hard_turn_speed is not None:
            kappa_threshold = float(abs(hard_turn_kappa))
            turn_speed_cap = float(max(hard_turn_speed, 0.0))
            turn_speed_cap = min(turn_speed_cap, max_speed_eff)
            turn_mask = np.abs(kappa) >= kappa_threshold
            vx[turn_mask] = np.minimum(vx[turn_mask], turn_speed_cap)

        vx = np.clip(vx, min_speed, max_speed_eff)
        ax = np.gradient(vx, s + 1e-6)

        return np.column_stack((s, x, y, heading, kappa, vx, ax))
