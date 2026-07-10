"""PP-map helper functions for `FollowingPathState`."""

from typing import Any, Dict, Iterable, Optional, Tuple

import numpy as np


def _log_warning(logger: Any, message: str) -> None:
    if logger is None:
        return
    try:
        logger.warning(message)
    except Exception:
        pass


def build_pp_waypoint_array(
    rich_planner: Any,
    waypoint_sequence: Optional[np.ndarray],
    params: Optional[Dict[str, Any]] = None,
    obstacles: Optional[Iterable[Any]] = None,
    logger: Any = None,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]]:
    """
    Build PP controller waypoint data from a [2, N] path.

    Returns:
        Tuple:
            - pp_waypoint_array: (N, 7) waypoint table expected by PP controller
            - speed_profile_base: base (design-speed) profile from rich trajectory
            - s_axis: route arc-length axis
            - track_length: route length used for cyclic projection
            - design_speed: desired_speed used to build base profile
    """
    if rich_planner is None or waypoint_sequence is None:
        return None

    try:
        waypoint_array = np.asarray(waypoint_sequence, dtype=float)
    except Exception:
        _log_warning(logger, "[PATH] Invalid waypoint sequence type for PP mode")
        return None

    if waypoint_array.ndim != 2 or waypoint_array.shape[0] < 2 or waypoint_array.shape[1] < 2:
        _log_warning(
            logger,
            f"[PATH] Invalid waypoint sequence for PP mode: {getattr(waypoint_array, 'shape', None)}",
        )
        return None

    cfg = params or {}
    desired_speed = float(cfg.get("desired_speed", 0.7))

    rich = rich_planner.path_to_rich_trajectory(
        path_xy=waypoint_array[:2, :],
        desired_speed=desired_speed,
        min_speed=cfg.get("min_speed", 0.15),
        kappa_speed_gain=cfg.get("kappa_speed_gain", 2.0),
        obstacles=obstacles,
        max_speed=cfg.get("max_speed", 0.8),
        hard_turn_kappa=cfg.get("hard_turn_kappa", 0.85),
        hard_turn_speed=cfg.get("hard_turn_speed", 0.32),
    )
    if rich is None or rich.shape[0] < 3:
        _log_warning(logger, "[PATH] path_rich failed to generate trajectory for PP mode")
        return None

    pp_waypoints = np.zeros((rich.shape[0], 7), dtype=float)
    pp_waypoints[:, 0] = rich[:, 1]  # x
    pp_waypoints[:, 1] = rich[:, 2]  # y
    pp_waypoints[:, 2] = rich[:, 5]  # reference speed
    pp_waypoints[:, 5] = rich[:, 4]  # curvature
    pp_waypoints[:, 6] = rich[:, 3]  # heading

    s_axis = rich[:, 0].astype(float)
    track_length = float(max(s_axis[-1], 1e-6))
    speed_profile_base = rich[:, 5].astype(float).copy()

    return pp_waypoints, speed_profile_base, s_axis, track_length, desired_speed


def update_pp_runtime_speed_profile(
    pp_waypoint_array: Optional[np.ndarray],
    pp_speed_profile_base: Optional[np.ndarray],
    profile_design_speed: float,
    v_ref_runtime: float,
) -> Optional[float]:
    """
    Scale PP waypoint speed profile using runtime v_ref.

    Returns:
        speed_scale if profile was updated, otherwise None.
    """
    if pp_waypoint_array is None or pp_speed_profile_base is None:
        return None

    if pp_speed_profile_base.shape[0] != pp_waypoint_array.shape[0]:
        return None

    v_ref = float(max(v_ref_runtime, 0.0))
    design_speed = max(float(profile_design_speed), 1e-3)
    speed_scale = v_ref / design_speed

    scaled_profile = pp_speed_profile_base * speed_scale
    scaled_profile = np.clip(scaled_profile, 0.0, v_ref)
    pp_waypoint_array[:, 2] = scaled_profile

    return speed_scale


def project_to_route_frenet(
    pp_waypoint_array: Optional[np.ndarray],
    pp_s_axis: Optional[np.ndarray],
    pp_track_length: float,
    x: float,
    y: float,
) -> Tuple[float, float]:
    """Project cartesian point onto current PP route and return (s, d)."""
    if pp_waypoint_array is None or pp_s_axis is None:
        return 0.0, 0.0

    path_xy = pp_waypoint_array[:, :2]
    if path_xy.shape[0] < 2:
        return 0.0, 0.0

    p = np.array([float(x), float(y)], dtype=float)
    seg_start = path_xy[:-1, :]
    seg_end = path_xy[1:, :]
    seg_vec = seg_end - seg_start
    seg_len_sq = np.sum(seg_vec * seg_vec, axis=1)

    with np.errstate(divide="ignore", invalid="ignore"):
        rel = p[None, :] - seg_start
        t = np.sum(rel * seg_vec, axis=1) / np.maximum(seg_len_sq, 1e-9)
    t = np.clip(t, 0.0, 1.0)

    proj = seg_start + seg_vec * t[:, None]
    diff = p[None, :] - proj
    dist_sq = np.sum(diff * diff, axis=1)
    i = int(np.argmin(dist_sq))

    tangent = seg_vec[i]
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm < 1e-9:
        return float(pp_s_axis[i]), 0.0

    tangent_unit = tangent / tangent_norm
    normal_unit = np.array([-tangent_unit[1], tangent_unit[0]], dtype=float)
    d = float(np.dot(p - proj[i], normal_unit))
    s = float(pp_s_axis[i] + t[i] * tangent_norm)

    if pp_track_length > 1e-6:
        s = s % pp_track_length
    return s, d
