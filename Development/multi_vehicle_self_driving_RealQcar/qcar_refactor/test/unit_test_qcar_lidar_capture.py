"""Tests for the read-only direct-QCar-LiDAR diagnostic capture."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from extra.platform.qcar.lidar_capture import capture_qcar_lidar, plot_qcar_lidar_capture


class _Clock:
    def __init__(self) -> None:
        self.value = 0.0

    def monotonic(self) -> float:
        return self.value

    def sleep(self, duration_s: float) -> None:
        self.value += max(0.0, float(duration_s))


class _FakeQCarLidar:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.read_count = 0
        self.terminate_count = 0
        self.angles = np.zeros(4)
        self.distances = np.zeros(4)

    def read(self) -> bool:
        self.read_count += 1
        self.angles = np.array([0.0, np.pi / 2.0, np.pi, 3.0 * np.pi / 2.0])
        self.distances = np.array([1.0, 2.0, 0.0, 3.0])
        return True

    def terminate(self) -> None:
        self.terminate_count += 1


class TestQCarLidarCapture(unittest.TestCase):
    def test_capture_reads_only_lidar_and_writes_plot_ready_artifacts(self):
        clock = _Clock()
        lidar = _FakeQCarLidar()
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            result = capture_qcar_lidar(
                output_directory=directory,
                duration_s=1.0,
                poll_rate_hz=10.0,
                lidar_factory=lambda **_kwargs: lidar,
                monotonic=clock.monotonic,
                sleep=clock.sleep,
            )
            plot_path = plot_qcar_lidar_capture(result.csv_path)

            metadata = json.loads(result.metadata_path.read_text(encoding="utf-8"))
            self.assertGreaterEqual(result.scan_count, 2)
            self.assertEqual(result.point_count, result.scan_count * 4)
            self.assertEqual(lidar.read_count, result.scan_count)
            self.assertEqual(lidar.terminate_count, 1)
            self.assertEqual(metadata["actuator_writes"], 0)
            self.assertFalse(metadata["qcar_actuator_created"])
            self.assertFalse(metadata["qcar_gps_created"])
            self.assertFalse(metadata["lidar_to_gps_service_changed"])
            self.assertTrue(plot_path.is_file())


if __name__ == "__main__":
    unittest.main()
