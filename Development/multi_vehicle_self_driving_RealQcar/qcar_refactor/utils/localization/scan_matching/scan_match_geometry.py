"""Private NumPy geometry helpers shared by scan-matching localizers."""

from __future__ import annotations

import math

import numpy as np

from core.vehicle_types import LaserScanSample, PoseMeasurement


def scan_points(scan: LaserScanSample, max_points: int, *, minimum_range_m: float | None = None) -> np.ndarray:
    ranges = np.asarray(scan.ranges_m, dtype=float)
    angles = scan.angle_min_rad + np.arange(ranges.size, dtype=float) * scan.angle_increment_rad
    minimum = scan.range_min_m if minimum_range_m is None else max(scan.range_min_m, minimum_range_m)
    valid = np.isfinite(ranges) & (ranges >= minimum) & (ranges <= scan.range_max_m)
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])))
    if len(points) <= max_points:
        return points
    return points[np.linspace(0, len(points) - 1, max_points, dtype=int)]


def icp_transform(source: np.ndarray, target: np.ndarray, *, max_correspondence_distance_m: float, min_correspondences: int, max_iterations: int, convergence_tolerance_m: float) -> tuple[np.ndarray, np.ndarray, int, float] | None:
    transformed = source.copy()
    total_rotation = np.identity(2)
    total_translation = np.zeros(2)
    threshold_squared = max_correspondence_distance_m * max_correspondence_distance_m
    for _ in range(max_iterations):
        nearest_indices, squared = _nearest_squared_distances(transformed, target)
        matched = squared <= threshold_squared
        if int(np.count_nonzero(matched)) < min_correspondences:
            return None
        rotation, translation = best_fit_transform(transformed[matched], target[nearest_indices[matched]])
        transformed = transformed @ rotation.T + translation
        total_rotation = rotation @ total_rotation
        total_translation = rotation @ total_translation + translation
        if float(np.linalg.norm(translation)) < convergence_tolerance_m and abs(math.atan2(rotation[1, 0], rotation[0, 0])) < convergence_tolerance_m:
            break
    nearest_indices, squared = _nearest_squared_distances(transformed, target)
    matched = squared <= threshold_squared
    correspondence_count = int(np.count_nonzero(matched))
    if correspondence_count < min_correspondences:
        return None
    return total_rotation, total_translation, correspondence_count, float(math.sqrt(float(np.mean(squared[matched]))))


def best_fit_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    left, _, right_t = np.linalg.svd((source - source_center).T @ (target - target_center))
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_t[-1, :] *= -1.0
        rotation = right_t.T @ left.T
    return rotation, target_center - rotation @ source_center


def transform_points(points: np.ndarray, pose: PoseMeasurement) -> np.ndarray:
    return points @ rotation_matrix(pose.yaw_rad).T + np.array((pose.x_m, pose.y_m))


def rotation_matrix(yaw_rad: float) -> np.ndarray:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return np.array(((cosine, -sine), (sine, cosine)))


def pose_with_timestamp(pose: PoseMeasurement, timestamp_ns: int, valid: bool) -> PoseMeasurement:
    return PoseMeasurement(timestamp_ns, pose.frame_id, pose.x_m, pose.y_m, pose.yaw_rad, valid)


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def validate_matcher_parameters(max_correspondence_distance_m: float, min_correspondences: int, max_iterations: int, max_points: int, convergence_tolerance_m: float) -> None:
    if not math.isfinite(max_correspondence_distance_m) or max_correspondence_distance_m <= 0.0:
        raise ValueError("max_correspondence_distance_m must be positive and finite")
    if not isinstance(min_correspondences, int) or isinstance(min_correspondences, bool) or min_correspondences < 3:
        raise ValueError("min_correspondences must be an integer of at least 3")
    if not isinstance(max_iterations, int) or isinstance(max_iterations, bool) or max_iterations <= 0:
        raise ValueError("max_iterations must be a positive integer")
    if not isinstance(max_points, int) or isinstance(max_points, bool) or max_points < min_correspondences:
        raise ValueError("max_points must be an integer no smaller than min_correspondences")
    if not math.isfinite(convergence_tolerance_m) or convergence_tolerance_m <= 0.0:
        raise ValueError("convergence_tolerance_m must be positive and finite")


def _nearest_squared_distances(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    squared_distances = np.einsum("ijk,ijk->ij", source[:, None, :] - target[None, :, :], source[:, None, :] - target[None, :, :])
    nearest_indices = np.argmin(squared_distances, axis=1)
    return nearest_indices, squared_distances[np.arange(len(source)), nearest_indices]
