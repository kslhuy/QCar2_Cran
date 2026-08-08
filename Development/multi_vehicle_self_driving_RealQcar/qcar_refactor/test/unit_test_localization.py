"""Unit tests for platform-neutral LiDAR scan and pose contracts."""

from __future__ import annotations
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import tempfile
import unittest

import numpy as np

from core.vehicle_types import LaserScanSample, PoseMeasurement
from utils.localization import (
    ScanMatchAlgorithm,
    ReferenceScanLidarLocalizer,
    ScanMatchingLidarOdometry,
    plot_laser_scan_artifact,
    write_laser_scan_artifact,
)


def _scan(*, timestamp_ns: int = 1_250_000_000, ranges=(float("inf"), 2.0, 3.0)) -> LaserScanSample:
    return LaserScanSample(
        timestamp_ns=timestamp_ns,
        frame_id="laser",
        angle_min_rad=-math.pi / 2.0,
        angle_max_rad=math.pi / 2.0,
        angle_increment_rad=math.pi / 2.0,
        time_increment_s=0.0,
        scan_time_s=0.05,
        range_min_m=0.05,
        range_max_m=20.0,
        ranges_m=ranges,
    )


class _ClosestRangeLocalizer(ScanMatchAlgorithm):
    """Tiny deterministic subclass proving the shared algorithm seam."""

    def __init__(self) -> None:
        super().__init__()
        self.initial_pose = None

    def _on_start(self, initial_pose):
        self.initial_pose = initial_pose

    def localize(self, scan: LaserScanSample) -> PoseMeasurement:
        closest = min((value for value in scan.ranges_m if math.isfinite(value)), default=0.0)
        return PoseMeasurement(
            timestamp_ns=scan.timestamp_ns,
            frame_id="map",
            x_m=closest,
            y_m=0.0,
            yaw_rad=0.0,
            valid=closest > 0.0,
        )


class TestLocalizationContracts(unittest.TestCase):
    def test_ros_shaped_scan_exports_standard_fields(self):
        scan = _scan()

        fields = scan.to_ros_fields()

        self.assertEqual(fields["header"], {"stamp": {"sec": 1, "nanosec": 250_000_000}, "frame_id": "laser"})
        self.assertEqual(fields["ranges"], (float("inf"), 2.0, 3.0))
        self.assertEqual(fields["intensities"], ())
        self.assertEqual(scan.timestamp_s, 1.25)

    def test_algorithm_base_enforces_lifecycle_and_returns_pose(self):
        localizer = _ClosestRangeLocalizer()
        initial = PoseMeasurement(0, "map", 1.0, 2.0, 0.1, True)
        with self.assertRaises(RuntimeError):
            localizer.update(_scan())

        localizer.start(initial)
        pose = localizer.update(_scan())

        self.assertIs(localizer.initial_pose, initial)
        self.assertEqual((pose.x_m, pose.y_m, pose.yaw_rad, pose.valid), (2.0, 0.0, 0.0, True))
        localizer.close()
        with self.assertRaises(RuntimeError):
            localizer.update(_scan())

    def test_records_compact_scan_data_and_ros_compatible_metadata(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            data_path, metadata_path = write_laser_scan_artifact(_scan(), temporary_directory)
            plot_path = plot_laser_scan_artifact(metadata_path)

            self.assertEqual(data_path.name, "first_scan.npz")
            self.assertTrue(data_path.is_file())
            self.assertTrue(plot_path.is_file())
            metadata = metadata_path.read_text(encoding="utf-8")
            self.assertIn('"schema": "sensor_msgs/msg/LaserScan-compatible"', metadata)
            self.assertIn('"frame_id": "laser"', metadata)


class TestScanMatchingLidarOdometry(unittest.TestCase):
    def test_composes_relative_scan_motion_from_known_map_pose(self):
        # Fixed asymmetric landmarks avoid the rotational ambiguity that a
        # symmetric synthetic scan would introduce.
        landmarks_world = np.array(
            ((3.0, 1.0), (4.0, -2.0), (6.0, 2.5), (7.5, -1.5), (9.0, 0.5), (5.5, 3.5)),
            dtype=float,
        )
        initial = PoseMeasurement(0, "map", 10.0, -3.0, 0.0, True)
        next_pose = PoseMeasurement(50_000_000, "map", 10.35, -2.80, 0.08, True)
        localizer = ScanMatchingLidarOdometry(
            max_correspondence_distance_m=1.5,
            min_correspondences=5,
            max_iterations=30,
        )
        localizer.start(initial)

        first = _scan_from_world_landmarks(landmarks_world, initial, timestamp_ns=0)
        second = _scan_from_world_landmarks(landmarks_world, next_pose, timestamp_ns=50_000_000)
        first_estimate = localizer.update(first)
        second_estimate = localizer.update(second)

        self.assertTrue(first_estimate.valid)
        self.assertTrue(second_estimate.valid)
        self.assertGreaterEqual(localizer.last_correspondence_count, 5)
        self.assertLess(localizer.last_rmse_m or math.inf, 0.03)
        # The synthetic scans are angularly binned exactly like a real
        # LaserScan, so permit the small quantization residual here.
        self.assertLess(
            math.hypot(second_estimate.x_m - next_pose.x_m, second_estimate.y_m - next_pose.y_m),
            0.03,
        )
        self.assertLess(abs(second_estimate.yaw_rad - next_pose.yaw_rad), 0.01)

    def test_reference_scan_matching_returns_map_pose_without_odometry_composition(self):
        landmarks_world = np.array(
            ((3.0, 1.0), (4.0, -2.0), (6.0, 2.5), (7.5, -1.5), (9.0, 0.5), (5.5, 3.5)),
            dtype=float,
        )
        reference_pose = PoseMeasurement(0, "map", 10.0, -3.0, 0.0, True)
        next_pose = PoseMeasurement(50_000_000, "map", 10.35, -2.80, 0.08, True)
        reference_scan = _scan_from_world_landmarks(landmarks_world, reference_pose, timestamp_ns=0)
        localizer = ReferenceScanLidarLocalizer(
            reference_scan,
            reference_pose,
            minimum_range_m=0.05,
            max_correspondence_distance_m=1.5,
            min_correspondences=5,
            max_iterations=30,
        )
        localizer.start(reference_pose)

        estimate = localizer.update(_scan_from_world_landmarks(landmarks_world, next_pose, timestamp_ns=50_000_000))

        self.assertTrue(estimate.valid)
        self.assertGreaterEqual(localizer.last_correspondence_count, 5)
        self.assertLess(math.hypot(estimate.x_m - next_pose.x_m, estimate.y_m - next_pose.y_m), 0.03)
        self.assertLess(abs(estimate.yaw_rad - next_pose.yaw_rad), 0.01)


def _scan_from_world_landmarks(
    landmarks_world: np.ndarray, pose: PoseMeasurement, *, timestamp_ns: int
) -> LaserScanSample:
    cosine = math.cos(pose.yaw_rad)
    sine = math.sin(pose.yaw_rad)
    world_from_sensor = np.array(((cosine, -sine), (sine, cosine)))
    sensor_points = (landmarks_world - np.array((pose.x_m, pose.y_m))) @ world_from_sensor
    ranges = np.full(720, np.inf, dtype=float)
    angles = np.arctan2(sensor_points[:, 1], sensor_points[:, 0])
    distances = np.linalg.norm(sensor_points, axis=1)
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / len(ranges)
    indices = np.floor((angles - angle_min) / angle_increment).astype(int) % len(ranges)
    np.minimum.at(ranges, indices, distances)
    return LaserScanSample(
        timestamp_ns=timestamp_ns,
        frame_id="laser",
        angle_min_rad=angle_min,
        angle_max_rad=angle_min + (len(ranges) - 1) * angle_increment,
        angle_increment_rad=angle_increment,
        time_increment_s=0.0,
        scan_time_s=0.05,
        range_min_m=0.05,
        range_max_m=20.0,
        ranges_m=tuple(ranges),
    )


if __name__ == "__main__":
    unittest.main()
