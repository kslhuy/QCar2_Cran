"""Fixed-reference-scan planar LiDAR map localization manager."""

from __future__ import annotations

import math

import numpy as np

from core.vehicle_types import LaserScanSample, PoseMeasurement
from .scan_match_algorithm import ScanMatchAlgorithm
from .scan_match_geometry import (
    icp_transform,
    pose_with_timestamp,
    scan_points,
    transform_points,
    validate_matcher_parameters,
    wrap_pi,
)


class ReferenceScanLidarLocalizer(ScanMatchAlgorithm):
    """Register each scan against one calibrated, map-frame reference scan."""

    def __init__(
        self,
        reference_scan: LaserScanSample,
        reference_pose: PoseMeasurement,
        *,
        output_frame: str = "map",
        minimum_range_m: float = 2.0,
        max_correspondence_distance_m: float = 1.0,
        min_correspondences: int = 12,
        max_iterations: int = 16,
        max_points: int = 360,
        convergence_tolerance_m: float = 1e-4,
    ) -> None:
        super().__init__()
        if not isinstance(reference_scan, LaserScanSample):
            raise TypeError("reference_scan must be a LaserScanSample")
        if not isinstance(reference_pose, PoseMeasurement):
            raise TypeError("reference_pose must be a PoseMeasurement")
        if not isinstance(output_frame, str) or not output_frame:
            raise ValueError("output_frame must be a non-empty string")
        if not math.isfinite(minimum_range_m) or minimum_range_m < 0.0:
            raise ValueError("minimum_range_m must be non-negative and finite")
        validate_matcher_parameters(
            max_correspondence_distance_m,
            min_correspondences,
            max_iterations,
            max_points,
            convergence_tolerance_m,
        )
        reference_points = scan_points(reference_scan, max_points, minimum_range_m=minimum_range_m)
        if len(reference_points) < min_correspondences:
            raise ValueError("reference_scan has too few static returns for scan matching")
        self._output_frame = output_frame
        self._reference_pose = PoseMeasurement(
            reference_pose.timestamp_ns,
            output_frame,
            reference_pose.x_m,
            reference_pose.y_m,
            reference_pose.yaw_rad,
            reference_pose.valid,
        )
        self._reference_points = transform_points(reference_points, self._reference_pose)
        self._minimum_range_m = float(minimum_range_m)
        self._max_correspondence_distance_m = float(max_correspondence_distance_m)
        self._min_correspondences = min_correspondences
        self._max_iterations = max_iterations
        self._max_points = max_points
        self._convergence_tolerance_m = float(convergence_tolerance_m)
        self._pose = self._reference_pose
        self.last_correspondence_count = 0
        self.last_rmse_m: float | None = None

    def _on_start(self, initial_pose: PoseMeasurement | None) -> None:
        self._initialize(initial_pose)

    def _on_reset(self, initial_pose: PoseMeasurement | None) -> None:
        self._initialize(initial_pose)

    def _initialize(self, initial_pose: PoseMeasurement | None) -> None:
        pose = initial_pose or self._reference_pose
        self._pose = PoseMeasurement(
            pose.timestamp_ns,
            self._output_frame,
            pose.x_m,
            pose.y_m,
            pose.yaw_rad,
            pose.valid,
        )
        self.last_correspondence_count = 0
        self.last_rmse_m = None

    def localize(self, scan: LaserScanSample) -> PoseMeasurement:
        local_points = scan_points(scan, self._max_points, minimum_range_m=self._minimum_range_m)
        if len(local_points) < self._min_correspondences:
            self.last_correspondence_count = 0
            self.last_rmse_m = None
            return pose_with_timestamp(self._pose, scan.timestamp_ns, False)
        transform = icp_transform(
            transform_points(local_points, self._pose),
            self._reference_points,
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
        position = rotation @ np.array((self._pose.x_m, self._pose.y_m)) + translation
        self._pose = PoseMeasurement(
            scan.timestamp_ns,
            self._output_frame,
            float(position[0]),
            float(position[1]),
            wrap_pi(self._pose.yaw_rad + math.atan2(rotation[1, 0], rotation[0, 0])),
            True,
        )
        self.last_correspondence_count = correspondence_count
        self.last_rmse_m = rmse_m
        return self._pose
