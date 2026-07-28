"""Fake-device tests for the QCar IO adapter without PAL or QLabs."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_types import ControlInput
from utils.io.io_qcar2 import IOQCar2


_CONFIG = {
    "write": {"max_throttle": 0.1, "max_steering": 0.48},
    "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
}


class _FakeQCar:
    def __init__(self):
        self.motorTach = 0.4
        self.gyroscope = np.array([0.0, 0.0, 0.2])
        self.accelerometer = np.array([1.0, 2.0, 9.81])
        self.read_count = 0
        self.writes = []

    def read(self):
        self.read_count += 1

    def write(self, throttle, steering):
        self.writes.append((throttle, steering))


class _FakeStream:
    def __init__(self):
        self.terminated = False

    def terminate(self):
        self.terminated = True


class _FakeGPS:
    def __init__(self):
        self.position = [1.0, 2.0]
        self.orientation = [0.0, 0.0, 0.3]
        self._gps_client = _FakeStream()
        self._lidar_client = _FakeStream()
        self.lidar = _FakeStream()

    def readGPS(self):
        return True


class TestIOQCar2Fake(unittest.TestCase):
    def setUp(self):
        self.qcar = _FakeQCar()
        self.gps = _FakeGPS()
        self.io = IOQCar2(_CONFIG, qcar=self.qcar, gps=self.gps)

    def test_reads_standard_sensor_and_gps_data(self):
        self.io.read_to_cache()
        data = self.io.read()

        self.assertEqual(self.qcar.read_count, 1)
        self.assertAlmostEqual(data.motor_tach, 0.4)
        self.assertAlmostEqual(data.gyro_z, 0.2)
        np.testing.assert_array_equal(data.accelerometer, [1.0, 2.0, 9.81])
        self.assertTrue(data.gps_valid)
        np.testing.assert_array_equal(data.gps_position, [1.0, 2.0, 0.3])

    def test_write_is_clipped_and_stop_is_idempotent(self):
        self.io.write(ControlInput(1.0, 1.0, 0.5, "test"))
        self.io.stop()
        self.io.stop()

        self.assertEqual(self.qcar.writes[0], (0.1, 0.48))
        self.assertEqual(self.qcar.writes[-1], (0.0, 0.0))

    def test_close_does_not_release_externally_owned_gps_streams(self):
        self.io.close()
        self.io.close()

        self.assertFalse(self.gps._gps_client.terminated)
        self.assertFalse(self.gps._lidar_client.terminated)
        self.assertFalse(self.gps.lidar.terminated)

    def test_qcar_object_is_required(self):
        with self.assertRaisesRegex(ValueError, "qcar object"):
            IOQCar2(_CONFIG, qcar=None)


if __name__ == "__main__":
    unittest.main()
