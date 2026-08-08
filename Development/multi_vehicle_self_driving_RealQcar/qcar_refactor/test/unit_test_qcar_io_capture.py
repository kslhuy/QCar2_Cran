"""Local tests for the read-only physical-QCar IO capture utility."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
import math

import numpy as np

from extra.platform.qcar.io_capture import (
    capture_qcar_io,
    plot_qcar_io_capture,
    plot_qcar_io_lidar_capture,
)


class _FakeQCar:
    def __init__(self) -> None:
        self.read_count = 0
        self.terminate_count = 0
        self.motorTach = 0.0
        self.gyroscope = np.zeros(3)
        self.accelerometer = np.zeros(3)

    def read(self) -> None:
        self.read_count += 1
        self.motorTach = float(self.read_count)
        self.gyroscope = np.array([0.0, 0.0, self.read_count / 10.0])
        self.accelerometer = np.array([self.read_count, 2.0, 9.81])

    def terminate(self) -> None:
        self.terminate_count += 1


class _FakeLidar:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.angles = np.array([0.0, math.pi / 2.0, math.pi, -math.pi / 2.0])
        self.distances = np.array([1.0, 2.0, 3.0, 0.0])
        self.read_count = 0
        self.terminate_count = 0

    def read(self) -> bool:
        self.read_count += 1
        return True

    def terminate(self) -> None:
        self.terminate_count += 1


class TestQCarIOCapture(unittest.TestCase):
    def test_capture_reads_only_and_plot_renders_from_csv(self):
        qcar = _FakeQCar()
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            result = capture_qcar_io(
                output_directory=directory,
                duration_s=1.0,
                sample_rate_hz=10.0,
                qcar_factory=lambda **_kwargs: qcar,
            )
            plot_path = plot_qcar_io_capture(result.csv_path)

            self.assertGreaterEqual(result.sample_count, 2)
            self.assertEqual(qcar.read_count, result.sample_count)
            self.assertEqual(qcar.terminate_count, 1)
            self.assertTrue(result.csv_path.is_file())
            self.assertTrue(result.metadata_path.is_file())
            self.assertTrue(plot_path.is_file())

    def test_capture_with_lidar_exercises_the_local_io_worker_and_writes_normalized_scan(self):
        qcar = _FakeQCar()
        lidar = _FakeLidar()
        observed_scans = []
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            result = capture_qcar_io(
                output_directory=directory,
                duration_s=1.0,
                sample_rate_hz=20.0,
                qcar_factory=lambda **_kwargs: qcar,
                lidar_factory=lambda **_kwargs: lidar,
                include_lidar=True,
                lidar_scan_observer=observed_scans.append,
            )
            plot_path = plot_qcar_io_lidar_capture(result.lidar_csv_path)

            self.assertGreaterEqual(result.lidar_scan_count, 2)
            self.assertGreater(result.lidar_point_count, 0)
            self.assertIsNotNone(result.lidar_csv_path)
            self.assertEqual(result.lidar_status["read_failures"], 0)
            self.assertTrue(result.lidar_csv_path.is_file())
            self.assertTrue(plot_path.is_file())
            self.assertGreater(lidar.read_count, 0)
            self.assertEqual(len(observed_scans), result.lidar_scan_count)
            self.assertEqual(lidar.terminate_count, 1)
            self.assertEqual(qcar.terminate_count, 1)


if __name__ == "__main__":
    unittest.main()
