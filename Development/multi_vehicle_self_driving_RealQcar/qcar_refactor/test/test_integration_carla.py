"""Direct-run CARLA spawn, IO-read, CSV, and plot integration tests."""

import csv
import sys
import unittest
from math import pi, sin
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.module_factory import build_vehicle_modules
from core.vehicle_types import ControlInput
from core.vehicle_config import load_config
from core.vehicle_logic import VehicleRuntime
from test.helper_artifacts import create_artifact_run


@unittest.skipUnless(__name__ == "__main__", "run this CARLA integration test file directly")
class TestLiveCarlaSmoke(unittest.TestCase):
    _DRIVE_DURATION_S = 15.0

    def _build_runtime(self):
        # This test validates CARLA session and IO behavior only. Avoid loading
        # the configured path-following mission when run from any directory.
        config = load_config(
            vehicle_config_file="config_vehicle_carla.yaml",
            selection_overrides={"observer": "null", "planner": "null", "controller": "null"},
        )
        modules = build_vehicle_modules(config)
        return config, modules, VehicleRuntime(
            config,
            modules.io,
            modules.observer,
            modules.planner,
            modules.controller_manager,
            modules.v2v,
            modules.ground_station,
            simulation=modules.simulation,
        )

    def test_spawn_tick_read_stop_cleanup(self):
        config, modules, runtime = self._build_runtime()
        try:
            runtime.start()
            self.assertIsNotNone(modules.simulation.ego_actor)
            telemetry = runtime.step(dt=config.runtime["loop_rate_hz"] ** -1)
            self.assertGreater(telemetry.sensor_data.sensor_timestamp, 0.0)
        finally:
            runtime.shutdown()

    def test_io_writes_steering_reads_sensor_series_and_writes_artifacts(self):
        config, modules, runtime = self._build_runtime()
        fixed_delta_s = float(config.module("simulation")["fixed_delta_seconds"])
        sample_count = max(2, round(self._DRIVE_DURATION_S / fixed_delta_s))
        samples = []
        try:
            runtime.start()
            for index in range(sample_count):
                elapsed_s = index * fixed_delta_s
                command = ControlInput(
                    throttle=0.20,
                    steering=0.30 * sin(2.0 * pi * elapsed_s / 5.0),
                    target_velocity=0.0,
                    source="carla_io_rate_test",
                )
                modules.io.write(command)
                modules.simulation.tick()
                modules.io.read_to_cache()
                sensor = modules.io.read()
                samples.append(
                    (
                        elapsed_s,
                        command.throttle,
                        command.steering,
                        sensor.sensor_timestamp,
                        sensor.motor_tach,
                        sensor.gyro_z,
                        np.asarray(sensor.accelerometer, dtype=float).copy(),
                        np.asarray(sensor.gps_position, dtype=float).copy(),
                        sensor.gps_valid,
                    )
                )
        finally:
            runtime.shutdown()

        sensor_timestamps = np.asarray([sample[3] for sample in samples], dtype=float)
        steering = np.asarray([sample[2] for sample in samples], dtype=float)
        self.assertEqual(len(samples), sample_count)
        self.assertTrue(np.all(np.diff(sensor_timestamps) >= 0.0))
        self.assertGreaterEqual(sensor_timestamps[-1] - sensor_timestamps[0], self._DRIVE_DURATION_S - 2.0 * fixed_delta_s)
        self.assertGreater(np.max(np.abs(steering)), 0.1)
        self.assertTrue(all(sample[8] for sample in samples))
        self.assertTrue(all(sample[6].shape == (3,) for sample in samples))
        artifacts = create_artifact_run(
            category="integration",
            platform="carla",
            test_name="io_sensor_trace",
            metadata={"duration_s": self._DRIVE_DURATION_S, "sample_count": sample_count},
        )
        self._write_sensor_artifacts(samples, artifacts.raw_directory, artifacts.figures_directory)

    def _write_sensor_artifacts(self, samples, raw_directory: Path, figures_directory: Path) -> None:
        csv_path = raw_directory / "carla_sensor_trace.csv"
        with csv_path.open("w", newline="", encoding="ascii") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "write_time_s", "throttle", "steering_rad", "sensor_time_s",
                    "speed_mps", "gyro_z_radps", "accel_x_mps2", "accel_y_mps2",
                    "accel_z_mps2", "gps_x_m", "gps_y_m", "gps_yaw_rad", "gps_valid",
                ]
            )
            for sample in samples:
                writer.writerow([*sample[:6], *sample[6], *sample[7], sample[8]])

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        values = np.asarray([sample[:6] for sample in samples], dtype=float)
        acceleration = np.asarray([sample[6] for sample in samples], dtype=float)
        position = np.asarray([sample[7] for sample in samples], dtype=float)
        relative_position = position[:, :2] - position[0, :2]
        write_time_s = values[:, 0]
        sensor_time_s = values[:, 3] - values[0, 3]
        write_rate_hz = self._rate_hz(write_time_s)
        sensor_rate_hz = self._rate_hz(sensor_time_s)

        figure, axes = plt.subplots(5, 1, figsize=(10, 14), constrained_layout=True)
        axes[0].plot(write_time_s, values[:, 1], label="throttle command")
        axes[0].plot(write_time_s, values[:, 2], label="steering command (rad)")
        axes[0].set_ylabel("IO write")
        axes[0].grid(True)
        axes[0].legend()
        axes[1].plot(sensor_time_s, values[:, 4], label="signed speed (m/s)")
        axes[1].plot(sensor_time_s, values[:, 5], label="yaw rate (rad/s)")
        axes[1].set_ylabel("speed / yaw rate")
        axes[1].grid(True)
        axes[1].legend()
        axes[2].plot(sensor_time_s, acceleration[:, 0], label="ax")
        axes[2].plot(sensor_time_s, acceleration[:, 1], label="ay")
        axes[2].plot(sensor_time_s, acceleration[:, 2], label="az")
        axes[2].set_ylabel("acceleration (m/s^2)")
        axes[2].grid(True)
        axes[2].legend()
        axes[3].plot(relative_position[:, 0], relative_position[:, 1], marker=".")
        axes[3].set_xlabel("relative x (m)")
        axes[3].set_ylabel("relative y (m)")
        axes[3].set_title("CARLA ego pose trace from initial sample")
        axes[3].grid(True)
        axes[3].axis("equal")
        axes[4].plot(write_time_s, write_rate_hz, label="command write rate")
        axes[4].plot(sensor_time_s, sensor_rate_hz, label="sensor read rate")
        axes[4].set_xlabel("CARLA simulation time (s)")
        axes[4].set_ylabel("rate (Hz)")
        axes[4].set_ylim(bottom=0.0)
        axes[4].grid(True)
        axes[4].legend()

        plot_path = figures_directory / "carla_sensor_trace.png"
        figure.savefig(plot_path, dpi=150)
        plt.close(figure)
        self.assertTrue(csv_path.is_file())
        self.assertGreater(csv_path.stat().st_size, 0)
        self.assertTrue(plot_path.is_file())
        self.assertGreater(plot_path.stat().st_size, 0)

    @staticmethod
    def _rate_hz(timestamps: np.ndarray) -> np.ndarray:
        rates = np.full(timestamps.shape, np.nan, dtype=float)
        intervals = np.diff(timestamps)
        rates[1:] = np.divide(1.0, intervals, out=np.full_like(intervals, np.nan), where=intervals > 0.0)
        return rates


if __name__ == "__main__":
    unittest.main()
