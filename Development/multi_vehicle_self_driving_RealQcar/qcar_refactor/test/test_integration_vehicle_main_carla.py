"""Direct-run CARLA entry-point integration test with telemetry artifacts."""

import csv
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core import vehicle_main
from core.vehicle_logic import VehicleRuntime
from test.helper_artifacts import create_artifact_run


class _CapturingRuntime(VehicleRuntime):
    """Capture real vehicle-main telemetry without changing production code."""

    instances: list["_CapturingRuntime"] = []

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.telemetry = []
        self.instances.append(self)

    def step(self, dt=None):
        sample = super().step(dt)
        self.telemetry.append(sample)
        return sample


@unittest.skipUnless(__name__ == "__main__", "run this CARLA vehicle-main integration test file directly")
class TestVehicleMainCarlaIntegration(unittest.TestCase):
    """Exercise load_config -> module_factory -> VehicleRuntime -> CARLA IO."""

    def setUp(self):
        _CapturingRuntime.instances.clear()

    def test_vehicle_main_runs_carla_profile_and_reads_io(self):
        with patch("core.vehicle_process.VehicleRuntime", _CapturingRuntime), patch("core.vehicle_main.time.sleep"):
            result = vehicle_main.main(["--vehicle-config", "config_vehicle_carla.yaml", "--cycles", "800"])

        self.assertEqual(result, 0)
        self.assertEqual(len(_CapturingRuntime.instances), 1)
        runtime = _CapturingRuntime.instances[0]
        self.assertEqual(len(runtime.telemetry), 800)
        self.assertTrue(all(sample.sensor_data.gps_valid for sample in runtime.telemetry))
        self.assertGreater(runtime.telemetry[-1].sensor_data.sensor_timestamp, 0.0)
        artifacts = create_artifact_run(
            category="integration",
            platform="carla",
            test_name="vehicle_main",
            metadata={"cycles": len(runtime.telemetry), "vehicle_config": "config_vehicle_carla.yaml"},
        )
        self._write_artifacts(runtime.telemetry, artifacts.raw_directory, artifacts.figures_directory)

    def _write_artifacts(self, telemetry, raw_directory: Path, figures_directory: Path) -> None:
        rows = []
        for sample in telemetry:
            sensor = sample.sensor_data
            rows.append(
                {
                    "time_s": sensor.sensor_timestamp,
                    "state": sample.state.name,
                    "gps_x_m": sensor.gps_position[0],
                    "gps_y_m": sensor.gps_position[1],
                    "gps_yaw_rad": sensor.gps_position[2],
                    "speed_mps": sensor.motor_tach,
                    "gyro_z_radps": sensor.gyro_z,
                    "accel_x_mps2": sensor.accelerometer[0],
                    "accel_y_mps2": sensor.accelerometer[1],
                    "accel_z_mps2": sensor.accelerometer[2],
                    "estimate_x_m": sample.estimate.x,
                    "estimate_y_m": sample.estimate.y,
                }
            )

        csv_path = raw_directory / "vehicle_main_carla.csv"
        with csv_path.open("w", newline="", encoding="ascii") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        time_s = [row["time_s"] - rows[0]["time_s"] for row in rows]
        figure, axes = plt.subplots(3, 1, figsize=(10, 10), constrained_layout=True)
        axes[0].plot([row["gps_x_m"] for row in rows], [row["gps_y_m"] for row in rows], label="IOCarla GPS")
        axes[0].plot([row["estimate_x_m"] for row in rows], [row["estimate_y_m"] for row in rows], label="EKF estimate")
        axes[0].set_title("vehicle_main CARLA telemetry")
        axes[0].set_xlabel("x (m)")
        axes[0].set_ylabel("y (m)")
        axes[0].axis("equal")
        axes[0].grid(True)
        axes[0].legend()
        axes[1].plot(time_s, [row["speed_mps"] for row in rows], label="speed")
        axes[1].plot(time_s, [row["gyro_z_radps"] for row in rows], label="yaw rate")
        axes[1].set_ylabel("speed / yaw rate")
        axes[1].grid(True)
        axes[1].legend()
        axes[2].plot(time_s, [row["accel_x_mps2"] for row in rows], label="ax")
        axes[2].plot(time_s, [row["accel_y_mps2"] for row in rows], label="ay")
        axes[2].plot(time_s, [row["accel_z_mps2"] for row in rows], label="az")
        axes[2].set_xlabel("CARLA simulation time (s)")
        axes[2].set_ylabel("acceleration (m/s^2)")
        axes[2].grid(True)
        axes[2].legend()
        plot_path = figures_directory / "vehicle_main_carla.png"
        figure.savefig(plot_path, dpi=150)
        plt.close(figure)

        self.assertTrue(csv_path.is_file())
        self.assertGreater(csv_path.stat().st_size, 0)
        self.assertTrue(plot_path.is_file())
        self.assertGreater(plot_path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
