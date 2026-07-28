"""
Unit tests for the EKF observer.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_ekf
"""
import csv
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_types import SensorData, VehicleStateEstimate
from utils.control.observer.observer_ekf import ObserverEKF


EKF_CONFIG = {"wheelbase": 0.3}


ARTIFACT_CSV = os.path.join(
    os.path.dirname(__file__),
    "artifacts",
    "qcar_io_background_buffer.csv",
)


def _bool_from_csv(value):
    return str(value).strip().lower() in ("true", "1", "1.0", "yes")


def _make_sensor(
    motor_tach=0.0,
    gyro_z=0.0,
    accel_x=0.0,
    accel_y=0.0,
    timestamp=1.0,
    gps_valid=False,
    gps_position=(0.0, 0.0, 0.0),
    gps_timestamp=None,
):
    return SensorData(
        motor_tach=float(motor_tach),
        gyro_z=float(gyro_z),
        accelerometer=np.array([float(accel_x), float(accel_y), 9.81], dtype=float),
        sensor_timestamp=float(timestamp),
        gps_valid=bool(gps_valid),
        gps_position=np.asarray(gps_position, dtype=float),
        gps_timestamp=float(timestamp if gps_timestamp is None else gps_timestamp),
    )


def _copy_with_gps_invalid(sensor_data):
    return SensorData(
        motor_tach=sensor_data.motor_tach,
        gyro_z=sensor_data.gyro_z,
        accelerometer=sensor_data.accelerometer.copy(),
        sensor_timestamp=sensor_data.sensor_timestamp,
        gps_valid=False,
        gps_position=sensor_data.gps_position.copy(),
        gps_timestamp=sensor_data.gps_timestamp,
    )


def _load_csv(path):
    """Load the IO artifact into SensorData samples and wall-time dt values."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    sensor_list = []
    dt_list = []
    prev_wall_time = None

    for row in rows:
        sensor_ts = float(row["sensor_timestamp"])
        wall_time = float(row.get("wall_time", sensor_ts))
        if prev_wall_time is None:
            dt = 0.01
        else:
            dt = max(0.001, wall_time - prev_wall_time)
        prev_wall_time = wall_time

        sensor_list.append(
            SensorData(
                motor_tach=float(row["motor_tach"]),
                gyro_z=float(row["gyro_z"]),
                accelerometer=np.array(
                    [
                        float(row["accel_x"]),
                        float(row["accel_y"]),
                        float(row["accel_z"]),
                    ],
                    dtype=float,
                ),
                sensor_timestamp=sensor_ts,
                gps_valid=_bool_from_csv(row["gps_valid"]),
                gps_position=np.array(
                    [
                        float(row["gps_x"]),
                        float(row["gps_y"]),
                        float(row["gps_yaw"]),
                    ],
                    dtype=float,
                ),
                gps_timestamp=float(row["gps_timestamp"]),
            )
        )
        dt_list.append(dt)

    return sensor_list, dt_list


def _load_csv_metadata(path):
    """Load optional plotting metadata that is not part of SensorData."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    def column(name, default=0.0):
        values = []
        for row in rows:
            try:
                values.append(float(row.get(name, default)))
            except (TypeError, ValueError):
                values.append(float(default))
        return np.asarray(values, dtype=float)

    return {
        "wall_time": column("wall_time"),
        "cmd_throttle": column("cmd_throttle"),
        "cmd_steering": column("cmd_steering"),
    }


class TestEKFDeterministic(unittest.TestCase):
    """EKF contract tests that do not depend on recorded QLabs artifacts."""

    def setUp(self):
        self.ekf = ObserverEKF(EKF_CONFIG)
        self.ekf.start()

    def tearDown(self):
        self.ekf.stop()

    def test_start_initializes_zero_state(self):
        state = self.ekf.get_latest()
        self.assertIsInstance(state, VehicleStateEstimate)
        self.assertEqual(state.x, 0.0)
        self.assertEqual(state.y, 0.0)
        self.assertEqual(state.theta, 0.0)
        self.assertEqual(state.velocity, 0.0)

    def test_update_before_start_raises(self):
        ekf = ObserverEKF(EKF_CONFIG)
        with self.assertRaises(RuntimeError):
            ekf.update(_make_sensor(), dt=0.01)

    def test_gps_measurement_pulls_state_toward_pose(self):
        sensor = _make_sensor(
            motor_tach=0.5,
            timestamp=1.0,
            gps_valid=True,
            gps_position=(2.0, -1.0, 0.25),
        )
        estimate = self.ekf.update(sensor, dt=0.05)

        self.assertTrue(estimate.gps_valid)
        self.assertGreater(estimate.x, 0.0)
        self.assertLess(estimate.y, 0.0)
        self.assertGreater(estimate.theta, 0.0)
        self.assertGreater(estimate.velocity, 0.0)

    def test_velocity_tracks_motor_tach_without_gps(self):
        estimate = None
        for i in range(40):
            sensor = _make_sensor(
                motor_tach=1.0,
                timestamp=1.0 + 0.02 * i,
                gps_valid=False,
            )
            estimate = self.ekf.update(sensor, dt=0.02)

        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.velocity, 1.0, delta=0.2)

    def test_acceleration_is_reported_as_horizontal_magnitude(self):
        sensor = _make_sensor(accel_x=0.3, accel_y=0.4)
        estimate = self.ekf.update(sensor, dt=0.01)
        self.assertAlmostEqual(estimate.acceleration, 0.5)
        latest = self.ekf.get_latest()
        self.assertAlmostEqual(latest.acceleration, 0.5)

    def test_heading_stays_wrapped_during_large_yaw_motion(self):
        for i in range(100):
            sensor = _make_sensor(
                motor_tach=0.1,
                gyro_z=4.0,
                timestamp=1.0 + 0.05 * i,
            )
            estimate = self.ekf.update(sensor, dt=0.05)
            self.assertGreaterEqual(estimate.theta, -np.pi)
            self.assertLessEqual(estimate.theta, np.pi)

    def test_get_latest_matches_last_update(self):
        last = None
        for i in range(10):
            last = self.ekf.update(
                _make_sensor(motor_tach=0.2, timestamp=1.0 + i * 0.01),
                dt=0.01,
            )
        latest = self.ekf.get_latest()
        self.assertAlmostEqual(latest.x, last.x)
        self.assertAlmostEqual(latest.y, last.y)
        self.assertAlmostEqual(latest.theta, last.theta)
        self.assertAlmostEqual(latest.velocity, last.velocity)

    def test_stop_then_restart_resets_state(self):
        self.ekf.update(_make_sensor(motor_tach=1.0), dt=0.1)
        self.ekf.stop()
        self.ekf.start(initial_pose=[1.0, 2.0, 0.5])

        state = self.ekf.get_latest()
        self.assertEqual(state.x, 1.0)
        self.assertEqual(state.y, 2.0)
        self.assertEqual(state.theta, 0.5)
        self.assertEqual(state.velocity, 0.0)

    def test_gps_dropout_stays_finite(self):
        for i in range(50):
            sensor = _make_sensor(
                motor_tach=0.4,
                gyro_z=0.1,
                accel_x=0.02,
                timestamp=1.0 + i * 0.02,
                gps_valid=False,
            )
            estimate = self.ekf.update(sensor, dt=0.02)
            self.assertTrue(
                np.all(
                    np.isfinite(
                        [estimate.x, estimate.y, estimate.theta, estimate.velocity]
                    )
                )
            )


class TestEKFWithRecordedData(unittest.TestCase):
    """Replay recorded QCar2 IO data through the EKF."""

    @classmethod
    def setUpClass(cls):
        if not os.path.exists(ARTIFACT_CSV):
            raise unittest.SkipTest(f"Artifact CSV not found: {ARTIFACT_CSV}")
        cls.sensor_data, cls.dt_list = _load_csv(ARTIFACT_CSV)
        if not cls.sensor_data:
            raise unittest.SkipTest(f"Artifact CSV is empty: {ARTIFACT_CSV}")

    def setUp(self):
        self.ekf = ObserverEKF(EKF_CONFIG)
        self.ekf.start()

    def tearDown(self):
        self.ekf.stop()

    def test_artifact_dt_values_are_reasonable(self):
        dt = np.asarray(self.dt_list, dtype=float)
        self.assertTrue(np.all(np.isfinite(dt)))
        self.assertGreater(dt.min(), 0.0)
        self.assertLess(dt.max(), 0.5)

    def test_full_replay_no_nan(self):
        for sensor, dt in zip(self.sensor_data, self.dt_list):
            estimate = self.ekf.update(sensor, dt=dt)
            self.assertTrue(
                np.all(
                    np.isfinite(
                        [estimate.x, estimate.y, estimate.theta, estimate.velocity]
                    )
                ),
                f"NaN/inf at timestamp {sensor.sensor_timestamp}",
            )

    def test_full_replay_velocity_in_bounds(self):
        for sensor, dt in zip(self.sensor_data, self.dt_list):
            estimate = self.ekf.update(sensor, dt=dt)
            self.assertGreaterEqual(estimate.velocity, -3.0)
            self.assertLessEqual(estimate.velocity, 3.0)

    def test_heading_stays_in_pi_range(self):
        for sensor, dt in zip(self.sensor_data, self.dt_list):
            estimate = self.ekf.update(sensor, dt=dt)
            self.assertGreaterEqual(estimate.theta, -np.pi)
            self.assertLessEqual(estimate.theta, np.pi)

    def test_tracks_gps_when_unique_fixes_available(self):
        gps_errors = []
        gps_timestamps = []
        for sensor, dt in zip(self.sensor_data, self.dt_list):
            estimate = self.ekf.update(sensor, dt=dt)
            if sensor.gps_valid:
                gps_timestamps.append(sensor.gps_timestamp)
                gps_errors.append(
                    np.hypot(
                        estimate.x - sensor.gps_position[0],
                        estimate.y - sensor.gps_position[1],
                    )
                )

        unique_fixes = np.unique(np.asarray(gps_timestamps, dtype=float))
        if unique_fixes.size < 2:
            self.skipTest("Too few unique GPS fixes in artifact")

        warm_start = min(20, len(gps_errors) // 2)
        mean_error = float(np.mean(gps_errors[warm_start:]))
        self.assertLess(
            mean_error,
            2.0,
            f"Mean GPS position error {mean_error:.3f} m exceeds 2.0 m",
        )

    def test_get_latest_matches_last_update(self):
        last = None
        for sensor, dt in zip(self.sensor_data[:50], self.dt_list[:50]):
            last = self.ekf.update(sensor, dt=dt)

        latest = self.ekf.get_latest()
        self.assertAlmostEqual(latest.x, last.x, places=4)
        self.assertAlmostEqual(latest.y, last.y, places=4)
        self.assertAlmostEqual(latest.theta, last.theta, places=4)
        self.assertAlmostEqual(latest.velocity, last.velocity, places=4)

    def test_forced_gps_dropout_no_nan(self):
        for sensor, dt in zip(self.sensor_data, self.dt_list):
            estimate = self.ekf.update(_copy_with_gps_invalid(sensor), dt=dt)
            self.assertTrue(
                np.all(
                    np.isfinite(
                        [estimate.x, estimate.y, estimate.theta, estimate.velocity]
                    )
                )
            )

    def test_plot_estimation_vs_gps(self):
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.skipTest(f"matplotlib unavailable: {exc}")

        estimates = []
        gps = []
        gps_valid = []
        gps_timestamp = []
        motor_tach = []
        pos_errors = []

        for sensor, dt in zip(self.sensor_data, self.dt_list):
            estimate = self.ekf.update(sensor, dt=dt)
            estimates.append([estimate.x, estimate.y, estimate.theta, estimate.velocity])
            gps.append(sensor.gps_position.copy())
            gps_valid.append(1.0 if sensor.gps_valid else 0.0)
            gps_timestamp.append(sensor.gps_timestamp)
            motor_tach.append(sensor.motor_tach)
            if sensor.gps_valid:
                pos_errors.append(
                    np.hypot(
                        estimate.x - sensor.gps_position[0],
                        estimate.y - sensor.gps_position[1],
                    )
                )
            else:
                pos_errors.append(np.nan)

        t = np.cumsum(np.asarray(self.dt_list, dtype=float))
        t = t - t[0]
        estimates = np.asarray(estimates, dtype=float)
        gps = np.asarray(gps, dtype=float)
        gps_valid = np.asarray(gps_valid, dtype=float)
        gps_timestamp = np.asarray(gps_timestamp, dtype=float)
        motor_tach = np.asarray(motor_tach, dtype=float)
        pos_errors = np.asarray(pos_errors, dtype=float)
        metadata = _load_csv_metadata(ARTIFACT_CSV)
        cmd_throttle = metadata["cmd_throttle"][: len(t)]
        cmd_steering = metadata["cmd_steering"][: len(t)]

        valid_mask = gps_valid > 0.5
        packet_mask = np.zeros_like(valid_mask, dtype=bool)
        packet_mask[0] = bool(valid_mask[0])
        packet_mask[1:] = valid_mask[1:] & (gps_timestamp[1:] != gps_timestamp[:-1])

        ekf_heading = np.unwrap(estimates[:, 2])
        gps_heading = np.full_like(ekf_heading, np.nan)
        gps_heading[valid_mask] = np.unwrap(gps[valid_mask, 2])

        fig = plt.figure(figsize=(14, 10), constrained_layout=True)
        grid = fig.add_gridspec(3, 2, height_ratios=[2.2, 1.0, 1.0])

        ax_xy = fig.add_subplot(grid[0, 0])
        ax_err = fig.add_subplot(grid[0, 1])
        ax_heading = fig.add_subplot(grid[1, :])
        ax_velocity = fig.add_subplot(grid[2, 0])
        ax_command = fig.add_subplot(grid[2, 1], sharex=ax_velocity)

        ekf_color = "tab:blue"
        gps_color = "tab:orange"
        error_color = "tab:red"
        tach_color = "0.35"
        throttle_color = "tab:green"
        steering_color = "tab:purple"

        ax_xy.plot(estimates[:, 0], estimates[:, 1], color=ekf_color, linewidth=1.2, label="EKF")
        ax_xy.plot(gps[:, 0], gps[:, 1], color=gps_color, linewidth=1.0, alpha=0.65, label="GPS cached")
        ax_xy.scatter(
            gps[packet_mask, 0],
            gps[packet_mask, 1],
            color=gps_color,
            s=16,
            edgecolors="none",
            label="GPS packets",
        )
        ax_xy.set_xlabel("X [m]")
        ax_xy.set_ylabel("Y [m]")
        ax_xy.set_title("Trajectory")
        ax_xy.axis("equal")
        ax_xy.legend(loc="best", fontsize=8)

        ax_err.plot(t, pos_errors, color=error_color, linewidth=1.0)
        if np.any(np.isfinite(pos_errors)):
            p95 = float(np.nanpercentile(pos_errors, 95))
            ax_err.axhline(p95, color="0.4", linestyle="--", linewidth=0.8, label=f"p95={p95:.2f} m")
        ax_err.set_xlabel("Time [s]")
        ax_err.set_ylabel("Error [m]")
        ax_err.set_title("Position Error")
        ax_err.legend(loc="best", fontsize=8)

        ax_heading.plot(t, ekf_heading, color=ekf_color, linewidth=1.0, label="EKF theta")
        ax_heading.scatter(t[packet_mask], gps_heading[packet_mask], color=gps_color, s=12, label="GPS yaw")
        ax_heading.set_ylabel("Heading [rad]")
        ax_heading.set_title("Heading")
        ax_heading.legend(loc="best", fontsize=8)

        ax_velocity.plot(t, estimates[:, 3], color=ekf_color, linewidth=1.0, label="EKF v")
        ax_velocity.plot(t, motor_tach, color=tach_color, linewidth=0.9, alpha=0.85, label="motor tach")
        ax_velocity.set_ylabel("v [m/s]")
        ax_velocity.set_title("Velocity")
        ax_velocity.legend(loc="best", fontsize=8)

        ax_command.step(t, cmd_throttle, where="post", color=throttle_color, linewidth=1.0, label="throttle")
        ax_command.step(t, cmd_steering, where="post", color=steering_color, linewidth=1.0, label="steering")
        ax_command.set_ylabel("Command")
        ax_command.set_title("Input Command")
        ax_command.legend(loc="best", fontsize=8)

        output_dir = os.path.join(os.path.dirname(__file__), "artifacts")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "ekf_estimation_vs_gps.png")
        fig.savefig(output_path, dpi=120)
        plt.close(fig)
        print(f"[PLOT] saved {output_path}")


if __name__ == "__main__":
    unittest.main()
