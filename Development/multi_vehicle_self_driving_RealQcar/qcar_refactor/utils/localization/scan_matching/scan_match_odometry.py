"""Scan-to-scan planar LiDAR odometry localization manager."""

from __future__ import annotations

import math

import numpy as np

from core.vehicle_types import LaserScanSample, PoseMeasurement
from .scan_match_algorithm import ScanMatchAlgorithm
from .scan_match_geometry import (
    icp_transform,
    pose_with_timestamp,
    rotation_matrix,
    scan_points,
    validate_matcher_parameters,
    wrap_pi,
)


class ScanMatchingLidarOdometry(ScanMatchAlgorithm):
    """Estimate planar motion by ICP between consecutive LiDAR scans.

    It composes relative transforms after an externally supplied map pose, so
    it is odometry rather than an absolute GPS replacement.
    """

    def __init__(
        self,
        *,
        output_frame: str = "map",
        max_correspondence_distance_m: float = 1.0,
        min_correspondences: int = 12,
        max_iterations: int = 12,
        max_points: int = 360,
        convergence_tolerance_m: float = 1e-4,
    ) -> None:
        super().__init__()
        if not isinstance(output_frame, str) or not output_frame:
            raise ValueError("output_frame must be a non-empty string")
        validate_matcher_parameters(
            max_correspondence_distance_m,
            min_correspondences,
            max_iterations,
            max_points,
            convergence_tolerance_m,
        )
        self._output_frame = output_frame
        self._max_correspondence_distance_m = float(max_correspondence_distance_m)
        self._min_correspondences = min_correspondences
        self._max_iterations = max_iterations
        self._max_points = max_points
        self._convergence_tolerance_m = float(convergence_tolerance_m)
        self._previous_points: np.ndarray | None = None
        self._pose = PoseMeasurement(0, output_frame, 0.0, 0.0, 0.0, False)
        self.last_correspondence_count = 0
        self.last_rmse_m: float | None = None

    def _on_start(self, initial_pose: PoseMeasurement | None) -> None:
        self._initialize(initial_pose)

    def _on_reset(self, initial_pose: PoseMeasurement | None) -> None:
        self._initialize(initial_pose)

    def _initialize(self, initial_pose: PoseMeasurement | None) -> None:
        pose = initial_pose or PoseMeasurement(0, self._output_frame, 0.0, 0.0, 0.0, False)
        self._pose = PoseMeasurement(
            pose.timestamp_ns,
            self._output_frame,
            pose.x_m,
            pose.y_m,
            pose.yaw_rad,
            pose.valid,
        )
        self._previous_points = None
        self.last_correspondence_count = 0
        self.last_rmse_m = None

    def localize(self, scan: LaserScanSample) -> PoseMeasurement:
        current_points = scan_points(scan, self._max_points)
        if self._previous_points is None:
            self._previous_points = current_points
            self.last_correspondence_count = 0
            self.last_rmse_m = None
            self._pose = pose_with_timestamp(
                self._pose, scan.timestamp_ns, len(current_points) >= self._min_correspondences
            )
            return self._pose
        if len(current_points) < self._min_correspondences or len(self._previous_points) < self._min_correspondences:
            self.last_correspondence_count = 0
            self.last_rmse_m = None
            return pose_with_timestamp(self._pose, scan.timestamp_ns, False)
        transform = icp_transform(
            current_points,
            self._previous_points,
            max_correspondence_distance_m=self._max_correspondence_distance_m,
            min_correspondences=self._min_correspondences,
            max_iterations=self._max_iterations,
            convergence_tolerance_m=self._convergence_tolerance_m,
        )
        if transform is None:
            self.last_correspondence_count = 0
            self.last_rmse_m = None
            return pose_with_timestamp(self._pose, scan.timestamp_ns, False)
        rotation, translation, correspondence_count, rmse_m = transform
        position = rotation_matrix(self._pose.yaw_rad) @ translation + np.array((self._pose.x_m, self._pose.y_m))
        self._pose = PoseMeasurement(
            scan.timestamp_ns,
            self._output_frame,
            float(position[0]),
            float(position[1]),
            wrap_pi(self._pose.yaw_rad + math.atan2(rotation[1, 0], rotation[0, 0])),
            True,
        )
        self._previous_points = current_points
        self.last_correspondence_count = correspondence_count
        self.last_rmse_m = rmse_m
        return self._pose
