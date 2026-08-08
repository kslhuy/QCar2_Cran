"""Unit tests for the local QCar LiDAR IO worker and PAL conversion."""

from __future__ import annotations

import copy
import math
import time
import unittest

import numpy as np

from extra.platform.qcar.process_runner import qcar_resource_context
from utils.io.io_qcar2 import IOQCar2, qcar_lidar_to_laser_scan


_CONFIG = {
    "write": {"max_throttle": 0.1, "max_steering": 0.48},
    "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
    "sensors": {
        "lidar": {
            "enabled": True,
            "source": "qcar_hardware",
            "poll_rate_hz": 100.0,
            "queue_capacity": 8,
            "frame_id": "qcar_lidar",
            "bin_count": 8,
            "range_min_m": 0.05,
            "range_max_m": 12.0,
            "angle_min_rad": -math.pi,
            "angle_offset_rad": 0.0,
            "angle_sign": 1.0,
        }
    },
}


class _FakeQCar:
    def __init__(self) -> None:
        self.motorTach = 0.0
        self.gyroscope = np.zeros(3)
        self.accelerometer = np.zeros(3)
        self.writes: list[tuple[float, float]] = []
        self.terminated = 0

    def read(self) -> None:
        pass

    def write(self, *, throttle: float, steering: float) -> None:
        self.writes.append((throttle, steering))

    def terminate(self) -> None:
        self.terminated += 1


class _FakeLidar:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.angles = np.array([0.0, math.pi / 2.0, math.pi, -math.pi / 2.0])
        self.distances = np.array([1.0, 2.0, 3.0, 0.0])
        self.read_count = 0
        self.terminated = 0

    def read(self) -> bool:
        self.read_count += 1
        return True

    def terminate(self) -> None:
        self.terminated += 1


class _FakeLidarManager:
    def __init__(self, lidar: _FakeLidar) -> None:
        self.lidar = lidar
        self.acquire_count = 0
        self.release_count = 0

    def acquire(self) -> _FakeLidar:
        self.acquire_count += 1
        return self.lidar

    def release(self) -> None:
        self.release_count += 1
        self.lidar.terminate()


class TestQCarLidarConversion(unittest.TestCase):
    def test_bins_direct_pal_angles_and_preserves_invalid_ranges(self):
        scan = qcar_lidar_to_laser_scan(
            [0.0, math.pi / 2.0, math.pi, -math.pi / 2.0],
            [1.0, 2.0, 3.0, 0.0],
            timestamp_ns=123,
            scan_config=_CONFIG["sensors"]["lidar"],
        )

        self.assertEqual(scan.frame_id, "qcar_lidar")
        self.assertEqual(len(scan.ranges_m), 8)
        self.assertAlmostEqual(scan.ranges_m[4], 1.0)
        self.assertAlmostEqual(scan.ranges_m[6], 2.0)
        self.assertAlmostEqual(scan.ranges_m[0], 3.0)
        self.assertTrue(math.isinf(scan.ranges_m[2]))


class TestQCarLidarWorker(unittest.TestCase):
    def test_worker_publishes_scans_without_owning_the_lidar_resource(self):
        qcar = _FakeQCar()
        lidar = _FakeLidar()
        io = IOQCar2(_CONFIG, qcar=qcar, lidar=lidar)
        try:
            io.read_to_cache()
            deadline = time.monotonic() + 1.0
            while io.lidar_status()["accepted"] == 0 and time.monotonic() < deadline:
                time.sleep(0.01)
            scans = io.drain_lidar_scans()
            status = io.lidar_status()

            self.assertTrue(scans)
            self.assertEqual(scans[-1].frame_id, "qcar_lidar")
            self.assertGreater(lidar.read_count, 0)
            self.assertEqual(status["source"], "qcar_hardware")
            self.assertTrue(status["worker_running"])
            self.assertEqual(status["read_failures"], 0)
        finally:
            io.close()

        self.assertEqual(lidar.terminated, 0)
        self.assertEqual(qcar.writes[-1], (0.0, 0.0))

    def test_runtime_switch_acquires_and_releases_the_managed_resource_on_demand(self):
        qcar = _FakeQCar()
        lidar = _FakeLidar()
        manager = _FakeLidarManager(lidar)
        config = copy.deepcopy(_CONFIG)
        config["sensors"]["lidar"]["enabled"] = False
        io = IOQCar2(config, qcar=qcar, lidar_manager=manager)
        try:
            io.read_to_cache()
            time.sleep(0.03)
            self.assertEqual(lidar.read_count, 0)
            self.assertEqual(manager.acquire_count, 0)
            self.assertTrue(io.set_lidar_enabled(True))
            self.assertEqual(manager.acquire_count, 1)
            self.assertTrue(io.lidar_status()["resource_open"])
            io.read_to_cache()
            deadline = time.monotonic() + 1.0
            while lidar.read_count == 0 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertGreater(lidar.read_count, 0)
            self.assertTrue(io.lidar_status()["enabled"])
            self.assertTrue(io.set_lidar_enabled(False))
            deadline = time.monotonic() + 1.0
            while io.lidar_status()["worker_running"] and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertFalse(io.lidar_status()["worker_running"])
            self.assertEqual(manager.release_count, 1)
            self.assertEqual(lidar.terminated, 1)
            self.assertFalse(io.lidar_status()["resource_open"])
        finally:
            io.close()


class TestQCarPlatformResources(unittest.TestCase):
    def test_platform_context_owns_enabled_lidar_and_terminates_it_before_qcar(self):
        qcar = _FakeQCar()
        lidar = _FakeLidar()
        lidar_config = {**_CONFIG["sensors"]["lidar"], "num_measurements": 384, "filtering": True}

        with qcar_resource_context(
            qcar_factory=lambda **_kwargs: qcar,
            lidar_factory=lambda **_kwargs: lidar,
            lidar_config=lidar_config,
        ) as resources:
            self.assertIs(resources["qcar"], qcar)
            self.assertIs(resources["lidar"], lidar)

        self.assertEqual(lidar.terminated, 1)
        self.assertEqual(qcar.terminated, 1)

    def test_platform_context_defers_disabled_lidar_until_the_adapter_requests_it(self):
        qcar = _FakeQCar()
        lidars: list[_FakeLidar] = []
        lidar_config = {**_CONFIG["sensors"]["lidar"], "enabled": False, "num_measurements": 384, "filtering": True}

        def create_lidar(**_kwargs):
            lidar = _FakeLidar()
            lidars.append(lidar)
            return lidar

        with qcar_resource_context(
            qcar_factory=lambda **_kwargs: qcar,
            lidar_factory=create_lidar,
            lidar_config=lidar_config,
        ) as resources:
            self.assertNotIn("lidar", resources)
            manager = resources["lidar_manager"]
            self.assertEqual(lidars, [])
            lidar = manager.acquire()
            self.assertEqual(lidars, [lidar])
            manager.release()
            self.assertEqual(lidar.terminated, 1)

        self.assertEqual(qcar.terminated, 1)


if __name__ == "__main__":
    unittest.main()
