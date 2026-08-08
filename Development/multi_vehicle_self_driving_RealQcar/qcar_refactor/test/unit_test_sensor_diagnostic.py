"""Tests for the bounded, explicit ground-station sensor diagnostic protocol."""

from __future__ import annotations

import math
import unittest

from core.vehicle_types import LaserScanSample
from utils.ground_station.sensor_diagnostic import (
    SensorDiagnosticError,
    decode_lidar_diagnostic,
    encode_lidar_diagnostic,
)


def _scan() -> LaserScanSample:
    return LaserScanSample(
        timestamp_ns=123_000_000,
        frame_id="qcar_lidar",
        angle_min_rad=-math.pi,
        angle_max_rad=math.pi / 2.0,
        angle_increment_rad=math.pi / 2.0,
        time_increment_s=0.0,
        scan_time_s=0.05,
        range_min_m=0.05,
        range_max_m=12.0,
        ranges_m=(1.0, math.inf, 2.0, math.inf),
    )


class TestSensorDiagnosticProtocol(unittest.TestCase):
    def test_round_trip_preserves_the_normalized_scan_and_invalid_ranges(self):
        frame = decode_lidar_diagnostic(encode_lidar_diagnostic(_scan(), vehicle_id=3))

        self.assertEqual(frame.vehicle_id, 3)
        self.assertEqual(frame.scan.frame_id, "qcar_lidar")
        self.assertEqual(frame.scan.ranges_m[0], 1.0)
        self.assertTrue(math.isinf(frame.scan.ranges_m[1]))
        self.assertEqual(frame.scan.ranges_m[2], 2.0)

    def test_rejects_unrecognized_or_malformed_packets(self):
        with self.assertRaises(SensorDiagnosticError):
            decode_lidar_diagnostic(b'{"schema":"wrong"}')


if __name__ == "__main__":
    unittest.main()
