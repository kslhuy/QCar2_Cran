"""Direct-run live CARLA control-pipeline integration test."""

import csv
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.commands import CommandType, VehicleCommand
from core.module_factory import build_vehicle_modules
from core.vehicle_config import load_config
from core.vehicle_logic import VehicleRuntime
from core.vehicle_state_machine import State
from test.helper_artifacts import create_artifact_run


@unittest.skipUnless(__name__ == "__main__", "run this CARLA control integration test file directly")
class TestLiveCarlaControlPipeline(unittest.TestCase):
    """Exercise runtime -> controller -> IOCarla for a low-speed path."""

    _DURATION_S = 45.0

    def test_runtime_controls_carla_and_writes_artifacts(self):
        config = load_config(vehicle_config_file="config_vehicle_carla.yaml")
        modules = build_vehicle_modules(config)
        runtime = VehicleRuntime(
            config,
            modules.io,
            modules.observer,
            modules.planner,
            modules.controller_manager,
            modules.v2v,
            modules.ground_station,
            simulation=modules.simulation,
        )
        fixed_delta_s = float(config.module("simulation")["fixed_delta_seconds"])
        rows = []

        try:
            runtime.start()
            modules.simulation.tick()
            modules.io.read_to_cache()
            initial_sensor = modules.io.read()
            route = self._long_route(initial_sensor.gps_position)
            modules.planner.load_path(route)
            self.assertEqual(runtime.handle_command(VehicleCommand(CommandType.START)).runtime_state, State.RUNNING.name)
            for _ in range(max(2, round(self._DURATION_S / fixed_delta_s))):
                telemetry = runtime.step(dt=fixed_delta_s)
                rows.append(
                    {
                        "time_s": telemetry.sensor_data.sensor_timestamp,
                        "gps_x": telemetry.sensor_data.gps_position[0],
                        "gps_y": telemetry.sensor_data.gps_position[1],
                        "estimate_x": telemetry.estimate.x,
                        "estimate_y": telemetry.estimate.y,
                        "speed_mps": telemetry.sensor_data.motor_tach,
                        "target_x": telemetry.target.target_x,
                        "target_y": telemetry.target.target_y,
                        "target_speed_mps": telemetry.target.target_velocity,
                        "throttle": telemetry.command.throttle,
                        "steering_rad": telemetry.command.steering,
                        "state": telemetry.state.name,
                    }
                )
                if telemetry.target.is_finished:
                    break
        finally:
            runtime.shutdown()

        self.assertGreaterEqual(len(rows), 850)
        self.assertFalse(modules.planner.is_finished())
        self.assertEqual(rows[-1]["state"], "RUNNING")
        self.assertGreater(modules.planner.current_index, 6)
        self.assertTrue(all(row["state"] == "RUNNING" for row in rows))
        self.assertGreater(max(row["throttle"] for row in rows), 0.01)
        self.assertGreater(max(abs(row["steering_rad"]) for row in rows), 0.01)
        gps_xy = np.asarray([[row["gps_x"], row["gps_y"]] for row in rows], dtype=float)
        self.assertGreater(np.linalg.norm(gps_xy[-1] - gps_xy[0]), 0.5)
        artifacts = create_artifact_run(
            category="integration",
            platform="carla",
            test_name="control_pipeline",
            metadata={"duration_s": self._DURATION_S, "samples": len(rows)},
        )
        self._write_artifacts(rows, np.asarray(route, dtype=float), artifacts.raw_directory, artifacts.figures_directory)

    @staticmethod
    def _long_route(initial_position: np.ndarray) -> list[list[float]]:
        """Create a gentle, non-straight route from the spawned pose.

        The sine path has an approximately 12 m minimum curvature radius,
        avoiding the tight turns that are unsuitable for the Tesla Model 3.
        """
        start_x, start_y = float(initial_position[0]), float(initial_position[1])
        x_offset = np.linspace(0.0, 14.0, 57)
        y_offset = 1.2 * np.sin(2.0 * np.pi * x_offset / 24.0)
        points = np.column_stack((start_x + x_offset, start_y + y_offset))
        headings = np.arctan2(np.diff(points[:, 1]), np.diff(points[:, 0]))
        headings = np.append(headings, headings[-1])
        return np.column_stack((points, headings)).tolist()

    def _write_artifacts(
        self, rows: list[dict], route: np.ndarray, raw_directory: Path, figures_directory: Path
    ) -> None:
        csv_path = raw_directory / "integration_carla_control.csv"
        with csv_path.open("w", newline="", encoding="ascii") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        time_s = np.asarray([row["time_s"] for row in rows], dtype=float)
        time_s -= time_s[0]
        gps = np.asarray([[row["gps_x"], row["gps_y"]] for row in rows], dtype=float)
        estimate = np.asarray([[row["estimate_x"], row["estimate_y"]] for row in rows], dtype=float)
        target = np.asarray([[row["target_x"], row["target_y"]] for row in rows], dtype=float)
        estimation_error = estimate - gps
        estimation_error_norm = np.linalg.norm(estimation_error, axis=1)

        figure, axes = plt.subplots(4, 1, figsize=(10, 13), constrained_layout=True)
        axes[0].plot(gps[:, 0], gps[:, 1], label="IOCarla GPS pose")
        axes[0].plot(estimate[:, 0], estimate[:, 1], label="EKF estimate")
        axes[0].plot(route[:, 0], route[:, 1], linestyle="--", label="planned route")
        axes[0].plot(target[:, 0], target[:, 1], linestyle=":", label="planner target")
        axes[0].set_title("CARLA Control Pipeline")
        axes[0].set_xlabel("x (m)")
        axes[0].set_ylabel("y (m)")
        axes[0].axis("equal")
        axes[0].grid(True)
        axes[0].legend()
        axes[1].plot(time_s, [row["speed_mps"] for row in rows], label="IOCarla speed")
        axes[1].plot(time_s, [row["target_speed_mps"] for row in rows], label="planner target speed")
        axes[1].set_ylabel("speed (m/s)")
        axes[1].grid(True)
        axes[1].legend()
        axes[2].plot(time_s, [row["throttle"] for row in rows], label="controller throttle")
        axes[2].plot(time_s, [row["steering_rad"] for row in rows], label="controller steering")
        axes[2].set_xlabel("CARLA simulation time (s)")
        axes[2].set_ylabel("command")
        axes[2].grid(True)
        axes[2].legend()
        axes[3].plot(time_s, estimation_error[:, 0], label="EKF x - GPS x")
        axes[3].plot(time_s, estimation_error[:, 1], label="EKF y - GPS y")
        axes[3].plot(time_s, estimation_error_norm, label="position error norm")
        axes[3].set_xlabel("CARLA simulation time (s)")
        axes[3].set_ylabel("estimation error (m)")
        axes[3].grid(True)
        axes[3].legend()
        figure.savefig(figures_directory / "integration_carla_control.png", dpi=150)
        plt.close(figure)


if __name__ == "__main__":
    unittest.main()
