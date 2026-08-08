"""Unit tests for CARLA-to-shared-LiDAR localisation conversion."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from utils.io.io_carla import carla_lidar_to_laser_scan


class TestCarlaLidarConversion(unittest.TestCase):
    def test_converts_carla_right_handed_points_to_ros_scan(self):
        points = np.array(
            [
                2.0, 0.0, 0.0, 0.5,  # ahead: 0 rad
                0.0, 3.0, 0.0, 0.4,  # CARLA right: ROS -pi/2
                4.0, 0.0, 0.0, 0.3,  # same bin as first but farther
                40.0, 0.0, 0.0, 0.2,  # beyond configured range
                1.0, 0.0, 1.0, 0.1,  # outside the horizontal scan plane
            ],
            dtype=np.float32,
        )
        data = type("Lidar", (), {"timestamp": 2.5, "raw_data": points.tobytes()})()

        scan = carla_lidar_to_laser_scan(
            data,
            {
                "frame_id": "base_laser",
                "bin_count": 8,
                "range_min_m": 0.05,
                "range_max_m": 20.0,
                "planar_z_tolerance_m": 0.05,
            },
        )

        self.assertEqual(scan.timestamp_ns, 2_500_000_000)
        self.assertEqual(scan.frame_id, "base_laser")
        self.assertAlmostEqual(scan.ranges_m[4], 2.0)  # zero angle
        self.assertAlmostEqual(scan.ranges_m[2], 3.0)  # -pi/2 after Y inversion
        self.assertTrue(math.isinf(scan.ranges_m[0]))

if __name__ == "__main__":
    unittest.main()
