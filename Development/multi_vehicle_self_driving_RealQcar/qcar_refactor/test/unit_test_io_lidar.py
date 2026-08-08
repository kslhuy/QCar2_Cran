"""Unit tests for the shared optional-LiDAR behavior of ``IOBase``."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.vehicle_types import LaserScanSample
from utils.io.io_base import IONull


def _scan(timestamp_ns: int) -> LaserScanSample:
    return LaserScanSample(
        timestamp_ns=timestamp_ns,
        frame_id="laser",
        angle_min_rad=0.0,
        angle_max_rad=0.0,
        angle_increment_rad=1.0,
        time_increment_s=0.0,
        scan_time_s=0.0,
        range_min_m=0.05,
        range_max_m=20.0,
        ranges_m=(1.0,),
    )


_CONFIG = {
    "write": {"max_throttle": 1.0, "max_steering": 1.0},
    "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
    "sensors": {"lidar": {"queue_capacity": 2}},
}


class TestIOLidarQueue(unittest.TestCase):
    def setUp(self) -> None:
        self.io = IONull(_CONFIG)

    def test_drains_each_published_scan_once_in_order(self):
        self.io._publish_lidar_scan(_scan(1))
        self.io._publish_lidar_scan(_scan(2))

        self.assertEqual([scan.timestamp_ns for scan in self.io.drain_lidar_scans()], [1, 2])
        self.assertEqual(self.io.drain_lidar_scans(), ())
        self.assertEqual(self.io.lidar_status()["accepted"], 2)

    def test_drops_oldest_scan_when_local_buffer_is_full(self):
        self.io._publish_lidar_scan(_scan(1))
        self.io._publish_lidar_scan(_scan(2))
        self.io._publish_lidar_scan(_scan(3))

        self.assertEqual([scan.timestamp_ns for scan in self.io.drain_lidar_scans()], [2, 3])
        self.assertEqual(self.io.lidar_status()["dropped"], 1)


if __name__ == "__main__":
    unittest.main()
