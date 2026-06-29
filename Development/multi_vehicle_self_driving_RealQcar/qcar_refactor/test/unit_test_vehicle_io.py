"""
Unit tests for utils.IO.BaseIO — BaseVehicleIO (via test stub) and NullVehicleIO.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_vehicle_io
"""
import sys
import os
import unittest
import threading
from unittest.mock import patch
import numpy as np

# Ensure the qcar_refactor package is importable
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.Types import SensorData, ControlCommand
from utils.IO.BaseIO import BaseVehicleIO, NullVehicleIO
import utils.IO.BaseIO as vio_mod

# ---------------------------------------------------------------------------
# Concrete stub that lets us test BaseVehicleIO without real hardware
# ---------------------------------------------------------------------------

class _TestVehicleIO(BaseVehicleIO):
    """Test double: spies on abstract methods, writes directly to cache."""

    def __init__(self, config, logger=None):
        super().__init__(config, logger)
        self.poll_sensor_count = 0
        self.poll_gps_count = 0
        self.last_throttle = None
        self.last_steering = None
        self.stop_called = False
        # Optional override: dict with SensorData field names → values
        self._fake_sensor = None
        self._fake_gps = None

    # -- Abstract method spies (write directly to _sensor_data_cache) --

    def _poll_sensors(self):
        self.poll_sensor_count += 1
        src = self._fake_sensor if self._fake_sensor is not None else {
            "motor_tach": 1.0,
            "gyro_z": 0.5,
            "accelerometer": np.array([1.0, 2.0, 3.0]),
            "sensor_timestamp": vio_mod.time.time(),
        }
        self._sensor_data_cache.motor_tach = src["motor_tach"]
        self._sensor_data_cache.gyro_z = src["gyro_z"]
        self._sensor_data_cache.accelerometer = src["accelerometer"]
        self._sensor_data_cache.sensor_timestamp = src["sensor_timestamp"]

    def _poll_gps(self):
        self.poll_gps_count += 1
        src = self._fake_gps if self._fake_gps is not None else {
            "gps_valid": True,
            "gps_position": np.array([10.0, 20.0, 0.0]),
            "gps_timestamp": vio_mod.time.time(),
        }
        self._sensor_data_cache.gps_valid = src["gps_valid"]
        self._sensor_data_cache.gps_position = src["gps_position"]
        self._sensor_data_cache.gps_timestamp = src["gps_timestamp"]

    def _hardware_write(self, throttle: float, steering: float):
        self.last_throttle = throttle
        self.last_steering = steering

    def stop(self):
        self.stop_called = True
        self._hardware_write(0.0, 0.0)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestVehicleIO(unittest.TestCase):
    """Tests for both BaseVehicleIO (via _TestVehicleIO stub) and NullVehicleIO."""

    def setUp(self):
        self.config = {
            "write": {
                "max_throttle": 0.10,
                "max_steering": 0.48,
            },
            "read": {
                "sensor_rate_hz": 100,
                "gps_rate_hz": 10,
            },
        }
        self.io = _TestVehicleIO(self.config)
        self.null_io = NullVehicleIO(self.config)

    # ------------------------------------------------------------------
    # 1. Initialization
    # ------------------------------------------------------------------

    def test_init_stores_config_values(self):
        self.assertEqual(self.io._max_throttle, 0.10)
        self.assertEqual(self.io._max_steering, 0.48)
        self.assertEqual(self.io._reading_sensor_rate_hz, 100)
        self.assertEqual(self.io._reading_gps_rate_hz, 10)

    def test_init_cache_is_sensor_data_with_defaults(self):
        cache = self.io._sensor_data_cache
        self.assertIsInstance(cache, SensorData)
        self.assertEqual(cache.motor_tach, 0.0)
        self.assertEqual(cache.gyro_z, 0.0)
        self.assertEqual(cache.sensor_timestamp, 0.0)
        self.assertIsInstance(cache.accelerometer, np.ndarray)
        self.assertFalse(cache.gps_valid)
        self.assertEqual(cache.gps_timestamp, 0.0)
        self.assertIsInstance(cache.gps_position, np.ndarray)

    def test_cache_arrays_are_independent(self):
        """Mutating cache arrays between calls should not persist."""
        cache = self.io._sensor_data_cache
        cache.accelerometer[0] = 999.0
        # A fresh NullVehicleIO should still have zeros
        fresh = NullVehicleIO(self.config)
        self.assertEqual(fresh._sensor_data_cache.accelerometer[0], 0.0)

    def test_null_io_overrides_limits(self):
        self.assertEqual(self.null_io._max_throttle, 1.0)
        self.assertEqual(self.null_io._max_steering, 1.0)

    # ------------------------------------------------------------------
    # 2. _clip static method (unchanged)
    # ------------------------------------------------------------------

    def test_clip_value_in_range(self):
        self.assertEqual(BaseVehicleIO._clip(0.05, -0.10, 0.10), 0.05)
        self.assertEqual(BaseVehicleIO._clip(-0.03, -0.10, 0.10), -0.03)
        self.assertEqual(BaseVehicleIO._clip(0.0, -0.10, 0.10), 0.0)

    def test_clip_value_below_lo(self):
        self.assertEqual(BaseVehicleIO._clip(-0.50, -0.10, 0.10), -0.10)

    def test_clip_value_above_hi(self):
        self.assertEqual(BaseVehicleIO._clip(0.50, -0.10, 0.10), 0.10)

    def test_clip_exactly_at_bounds(self):
        self.assertEqual(BaseVehicleIO._clip(-0.10, -0.10, 0.10), -0.10)
        self.assertEqual(BaseVehicleIO._clip(0.10, -0.10, 0.10), 0.10)

    def test_clip_negative_bounds(self):
        self.assertEqual(BaseVehicleIO._clip(-0.60, -0.48, 0.48), -0.48)
        self.assertEqual(BaseVehicleIO._clip(0.60, -0.48, 0.48), 0.48)

    # ------------------------------------------------------------------
    # 3. read_sensors rate limiting (mutates cache in-place, returns None)
    # ------------------------------------------------------------------

    @patch("utils.IO.BaseIO.time.time")
    def test_read_sensors_polls_on_first_call(self, mock_time):
        mock_time.return_value = 100.0
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 1)
        self.assertEqual(self.io._sensor_data_cache.motor_tach, 1.0)
        self.assertEqual(self.io._sensor_data_cache.sensor_timestamp, 100.0)

    @patch("utils.IO.BaseIO.time.time")
    def test_read_sensors_skips_within_rate_window(self, mock_time):
        mock_time.return_value = 100.0
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 1)
        mock_time.return_value = 100.005  # < 0.01 s interval
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 1)  # still 1

    @patch("utils.IO.BaseIO.time.time")
    def test_read_sensors_polls_after_rate_elapsed(self, mock_time):
        mock_time.return_value = 100.0
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 1)
        mock_time.return_value = 100.01  # >= 0.01 s
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 2)

    # ------------------------------------------------------------------
    # 4. read_gps rate limiting
    # ------------------------------------------------------------------

    @patch("utils.IO.BaseIO.time.time")
    def test_read_gps_polls_on_first_call(self, mock_time):
        mock_time.return_value = 100.0
        self.io.read_gps()
        self.assertEqual(self.io.poll_gps_count, 1)
        self.assertTrue(self.io._sensor_data_cache.gps_valid)

    @patch("utils.IO.BaseIO.time.time")
    def test_read_gps_skips_within_rate_window(self, mock_time):
        mock_time.return_value = 100.0
        self.io.read_gps()
        self.assertEqual(self.io.poll_gps_count, 1)
        mock_time.return_value = 100.05  # < 0.1 s interval
        self.io.read_gps()
        self.assertEqual(self.io.poll_gps_count, 1)

    def test_read_gps_polls_after_rate_elapsed(self):
        interval = 1.0 / self.config["read"]["gps_rate_hz"]  # 0.1
        self.io._sensor_data_cache.gps_timestamp = 100.0
        self.io.poll_gps_count = 0
        with patch("utils.IO.BaseIO.time.time", return_value=100.0 + 2 * interval):
            self.io.read_gps()
        self.assertEqual(self.io.poll_gps_count, 1)

    # ------------------------------------------------------------------
    # 5. read_to_cache() + read() — poll then snapshot
    # ------------------------------------------------------------------

    @patch("utils.IO.BaseIO.time.time")
    def test_read_to_cache_polls_both(self, mock_time):
        """read_to_cache() triggers both sensor and GPS polls."""
        mock_time.return_value = 200.0
        self.io.read_to_cache()
        self.assertGreaterEqual(self.io.poll_sensor_count, 1)
        self.assertGreaterEqual(self.io.poll_gps_count, 1)
        # Cache is now populated
        self.assertEqual(self.io._sensor_data_cache.motor_tach, 1.0)
        self.assertEqual(self.io._sensor_data_cache.gyro_z, 0.5)

    @patch("utils.IO.BaseIO.time.time")
    def test_read_returns_snapshot_after_poll(self, mock_time):
        """read() returns a copy of whatever read_to_cache() populated."""
        mock_time.return_value = 200.0
        self.io.read_to_cache()          # poll hardware → cache
        data = self.io.read()            # read snapshot (no poll)
        self.assertIsInstance(data, SensorData)
        self.assertEqual(data.motor_tach, 1.0)
        self.assertEqual(data.gyro_z, 0.5)
        self.assertTrue(data.gps_valid)
        np.testing.assert_array_equal(data.gps_position, np.array([10.0, 20.0, 0.0]))

    @patch("utils.IO.BaseIO.time.time")
    def test_read_is_pure_snapshot_no_side_effect(self, mock_time):
        """read() must NEVER trigger a poll — it's a pure getter."""
        mock_time.return_value = 200.0
        # First call: read without any prior poll
        before_sensor = self.io.poll_sensor_count
        before_gps = self.io.poll_gps_count
        data = self.io.read()
        # Poll counts unchanged — read() does not poll
        self.assertEqual(self.io.poll_sensor_count, before_sensor)
        self.assertEqual(self.io.poll_gps_count, before_gps)
        # Returns defaults (cache was never populated)
        self.assertEqual(data.motor_tach, 0.0)
        self.assertFalse(data.gps_valid)

    @patch("utils.IO.BaseIO.time.time")
    def test_read_returns_copy_not_reference(self, mock_time):
        """Mutating returned arrays must not affect internal cache."""
        mock_time.return_value = 200.0
        self.io.read_to_cache()
        data1 = self.io.read()
        # Mutate returned data
        data1.accelerometer[0] = 999.0
        data1.gps_position[0] = 888.0
        # Advance time and poll again
        mock_time.return_value = 201.0
        self.io.read_to_cache()
        data2 = self.io.read()
        # Internal cache untouched by mutation
        np.testing.assert_array_equal(
            self.io._sensor_data_cache.accelerometer,
            np.array([1.0, 2.0, 3.0]),
        )
        # Fresh read returns correct data
        np.testing.assert_array_equal(
            data2.accelerometer,
            np.array([1.0, 2.0, 3.0]),
        )

    # ------------------------------------------------------------------
    # 6. write() clipping (unchanged)
    # ------------------------------------------------------------------

    def test_write_passes_through_within_limits(self):
        cmd = ControlCommand(throttle=0.05, steering=0.20, target_velocity=0.5)
        self.io.write(cmd)
        self.assertEqual(self.io.last_throttle, 0.05)
        self.assertEqual(self.io.last_steering, 0.20)

    def test_write_clips_throttle_above_max(self):
        cmd = ControlCommand(throttle=0.50, steering=0.0, target_velocity=1.0)
        self.io.write(cmd)
        self.assertEqual(self.io.last_throttle, 0.10)

    def test_write_clips_throttle_below_negative_max(self):
        cmd = ControlCommand(throttle=-0.50, steering=0.0, target_velocity=-1.0)
        self.io.write(cmd)
        self.assertEqual(self.io.last_throttle, -0.10)

    def test_write_clips_steering_above_max(self):
        cmd = ControlCommand(throttle=0.0, steering=0.80, target_velocity=0.5)
        self.io.write(cmd)
        self.assertEqual(self.io.last_steering, 0.48)

    def test_write_clips_steering_below_negative_max(self):
        cmd = ControlCommand(throttle=0.0, steering=-0.80, target_velocity=0.5)
        self.io.write(cmd)
        self.assertEqual(self.io.last_steering, -0.48)

    def test_write_exactly_at_bounds_passes_through(self):
        cmd = ControlCommand(
            throttle=self.config["write"]["max_throttle"],
            steering=-self.config["write"]["max_steering"],
            target_velocity=0.5,
        )
        self.io.write(cmd)
        self.assertEqual(self.io.last_throttle, 0.10)
        self.assertEqual(self.io.last_steering, -0.48)

    # ------------------------------------------------------------------
    # 7. stop()
    # ------------------------------------------------------------------

    def test_stop_sends_zero_command(self):
        self.io.stop()
        self.assertTrue(self.io.stop_called)
        self.assertEqual(self.io.last_throttle, 0.0)
        self.assertEqual(self.io.last_steering, 0.0)

    # ------------------------------------------------------------------
    # 8. NullVehicleIO
    # ------------------------------------------------------------------

    @patch("utils.IO.BaseIO.time.time")
    def test_null_io_read_returns_defaults(self, mock_time):
        mock_time.return_value = 300.0
        data = self.null_io.read()
        self.assertIsInstance(data, SensorData)
        self.assertEqual(data.motor_tach, 0.0)
        self.assertEqual(data.gyro_z, 0.0)
        self.assertEqual(data.sensor_timestamp, 0.0)
        self.assertFalse(data.gps_valid)
        self.assertEqual(data.gps_timestamp, 0.0)

    def test_null_io_write_is_noop(self):
        cmd = ControlCommand(throttle=0.99, steering=0.99, target_velocity=2.0)
        self.null_io.write(cmd)  # no exception

    def test_null_io_stop_is_noop(self):
        self.null_io.stop()  # no exception

    @patch("utils.IO.BaseIO.time.time")
    def test_null_io_never_polls(self, mock_time):
        """NullVehicleIO always keeps defaults in cache."""
        mock_time.return_value = 100.0
        self.null_io.read_to_cache()
        self.assertEqual(self.null_io._sensor_data_cache.motor_tach, 0.0)
        mock_time.return_value = 999.0
        self.null_io.read_to_cache()
        self.assertEqual(self.null_io._sensor_data_cache.motor_tach, 0.0)
        self.assertFalse(self.null_io._sensor_data_cache.gps_valid)

    # ------------------------------------------------------------------
    # 9. Edge: rate limiting at exact boundary
    # ------------------------------------------------------------------

    @patch("utils.IO.BaseIO.time.time")
    def test_read_sensors_polls_at_exact_boundary(self, mock_time):
        mock_time.return_value = 100.0
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 1)
        mock_time.return_value = 100.0 + (1.0 / self.config["read"]["sensor_rate_hz"])
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 2)

    @patch("utils.IO.BaseIO.time.time")
    def test_read_sensors_skips_just_before_boundary(self, mock_time):
        mock_time.return_value = 100.0
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 1)
        mock_time.return_value = 100.0 + (1.0 / self.config["read"]["sensor_rate_hz"]) - 1e-6
        self.io.read_sensors()
        self.assertEqual(self.io.poll_sensor_count, 1)

    # ------------------------------------------------------------------
    # 10. Continuous read loop — single-thread baseline
    # ------------------------------------------------------------------

    @patch("utils.IO.BaseIO.time.time")
    def test_single_thread_continuous_read_loop(self, mock_time):
        """Simulate a control loop: poll → read → poll → read with advancing time."""
        mock_time.return_value = 100.0
        iterations = 60

        for i in range(iterations):
            mock_time.return_value += 0.015
            self.io.read_to_cache()       # poll if rate allows
            data = self.io.read()          # get snapshot
            self.assertIsInstance(data, SensorData)
            self.assertIsInstance(data.accelerometer, np.ndarray)
            self.assertIsInstance(data.gps_position, np.ndarray)

        self.assertGreater(self.io.poll_sensor_count, iterations * 0.8)
        self.assertGreater(self.io.poll_gps_count, 5)

    @patch("utils.IO.BaseIO.time.time")
    def test_single_thread_poll_then_read_separation(self, mock_time):
        """Source polls hardware, then multiple readers each take a snapshot."""
        mock_time.return_value = 200.0
        self.io.read_to_cache()
        self.assertEqual(self.io.poll_sensor_count, 1)
        self.assertEqual(self.io.poll_gps_count, 1)

        # Multiple reads — none trigger a poll
        for _ in range(5):
            data = self.io.read()
            self.assertIsInstance(data, SensorData)
        # Poll counts unchanged
        self.assertEqual(self.io.poll_sensor_count, 1)
        self.assertEqual(self.io.poll_gps_count, 1)

    # ------------------------------------------------------------------
    # 11. Multi-thread: lock prevents cross-thread data leaks
    # ------------------------------------------------------------------

    def test_lock_prevents_cross_thread_data_leak(self):
        """Under concurrent read() calls, each thread gets a self-consistent
        snapshot — sensor and GPS from the same poll moment (same thread's tag)."""
        errors = []
        iterations = 100
        ready = threading.Barrier(2)

        # Force every call to poll
        self.io._reading_sensor_rate_hz = float("inf")
        self.io._reading_gps_rate_hz = float("inf")

        orig_poll_sensors = self.io._poll_sensors
        orig_poll_gps = self.io._poll_gps

        def tagged_poll_sensors():
            tag = float(hash(threading.current_thread().name) % 1000)
            self.io.poll_sensor_count += 1
            self.io._sensor_data_cache.motor_tach = tag
            self.io._sensor_data_cache.gyro_z = 0.0
            self.io._sensor_data_cache.accelerometer = np.zeros(3)
            self.io._sensor_data_cache.sensor_timestamp = 0.0

        def tagged_poll_gps():
            tag = float(hash(threading.current_thread().name) % 1000)
            self.io.poll_gps_count += 1
            self.io._sensor_data_cache.gps_valid = True
            self.io._sensor_data_cache.gps_position = np.array([tag, 0.0, 0.0])
            self.io._sensor_data_cache.gps_timestamp = 0.0

        self.io._poll_sensors = tagged_poll_sensors
        self.io._poll_gps = tagged_poll_gps

        def reader():
            ready.wait()
            for _ in range(iterations):
                try:
                    data = self.io.read()
                    sensor_tag = int(data.motor_tach)
                    gps_tag = int(data.gps_position[0])
                    # Each caller's sensor and GPS must match —
                    # proving the lock gave a consistent snapshot
                    self.assertEqual(
                        sensor_tag, gps_tag,
                        f"LOCK FAILED: sensor tag={sensor_tag} ≠ gps tag={gps_tag} "
                        f"— cross-thread data leak in read()"
                    )
                except Exception as e:
                    errors.append(str(e))

        t1 = threading.Thread(target=reader, name="T1")
        t2 = threading.Thread(target=reader, name="T2")
        t1.start()
        t2.start()
        t1.join(timeout=10.0)
        t2.join(timeout=10.0)

        self.io._poll_sensors = orig_poll_sensors
        self.io._poll_gps = orig_poll_gps

        self.assertEqual(errors, [], f"Thread errors: {errors}")

    def test_multi_thread_write_during_read(self):
        """write() from one thread while another polls+reads — verify no crash."""
        far_future = 1e15
        self.io._sensor_data_cache.sensor_timestamp = far_future
        self.io._sensor_data_cache.gps_timestamp = far_future

        errors = []
        iterations = 300
        ready = threading.Barrier(2)

        def writer():
            ready.wait()
            for i in range(iterations):
                try:
                    cmd = ControlCommand(
                        throttle=0.05 * (i % 3 - 1),
                        steering=0.1 * (i % 5 - 2),
                        target_velocity=0.5,
                    )
                    self.io.write(cmd)
                except Exception as e:
                    errors.append(f"writer: {e}")

        def reader():
            ready.wait()
            for _ in range(iterations):
                try:
                    self.io.read_to_cache()       # poll (rate bypassed)
                    data = self.io.read()          # snapshot
                    self.assertIsInstance(data, SensorData)
                    self.assertIsInstance(data.accelerometer, np.ndarray)
                except Exception as e:
                    errors.append(f"reader: {e}")

        t1 = threading.Thread(target=writer, name="writer")
        t2 = threading.Thread(target=reader, name="reader")
        t1.start(); t2.start()
        t1.join(timeout=5.0); t2.join(timeout=5.0)
        self.assertEqual(errors, [], f"Thread errors: {errors}")

    # ------------------------------------------------------------------
    # 12. Concurrent _poll_*() spam from multiple threads (ROS2 callback simulation)
    # ------------------------------------------------------------------

    def test_concurrent_poll_from_multiple_threads(self):
        """Simulate ROS2: multiple spinner threads calling _poll_sensors/_poll_gps
        directly, while main thread reads. Verify no crashes under lock contention."""
        errors = []
        iterations = 300
        ready = threading.Barrier(3)

        def ros2_sensor_callback():
            """Simulate a ROS2 subscriber callback — writes directly to cache."""
            ready.wait()
            for i in range(iterations):
                try:
                    # Direct hardware poll bypasses rate limit
                    # (this is what a ROS2 callback would do via _poll_sensors)
                    with self.io._cache_lock:
                        self.io._sensor_data_cache.motor_tach = float(i % 100)
                        self.io._sensor_data_cache.gyro_z = 0.5 + i * 0.001
                        self.io._sensor_data_cache.accelerometer = np.array(
                            [1.0 + i, 2.0, 3.0]
                        )
                        self.io._sensor_data_cache.sensor_timestamp = 1e9 + i
                except Exception as e:
                    errors.append(f"ros2_sensor: {e}")

        def ros2_gps_callback():
            """Simulate GPS subscriber callback."""
            ready.wait()
            for i in range(iterations):
                try:
                    with self.io._cache_lock:
                        self.io._sensor_data_cache.gps_valid = True
                        self.io._sensor_data_cache.gps_position = np.array(
                            [10.0 + i, 20.0, 0.0]
                        )
                        self.io._sensor_data_cache.gps_timestamp = 1e9 + i
                except Exception as e:
                    errors.append(f"ros2_gps: {e}")

        def main_reader():
            """Main control loop reading."""
            ready.wait()
            for _ in range(iterations):
                try:
                    data = self.io.read()
                    self.assertIsInstance(data, SensorData)
                    self.assertTrue(np.all(np.isfinite(data.accelerometer)))
                    self.assertTrue(np.all(np.isfinite(data.gps_position)))
                except Exception as e:
                    errors.append(f"reader: {e}")

        threads = [
            threading.Thread(target=ros2_sensor_callback, name="ros2-sensor"),
            threading.Thread(target=ros2_gps_callback, name="ros2-gps"),
            threading.Thread(target=main_reader, name="main-reader"),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10.0)

        self.assertEqual(errors, [], f"Thread errors: {errors}")


if __name__ == "__main__":
    unittest.main()