"""Direct-run CARLA integration test for the SDCS small-map path planner."""

import csv
import math
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
from utils.control.path_planner import SDCSSmallMapRoadMap


@unittest.skipUnless(__name__ == "__main__", "run this CARLA integration test file directly")
class TestLiveCarlaSDCSSmallMapPath(unittest.TestCase):
    """Verify a Tesla follows the direct CARLA-frame SDCS node route."""

    _DURATION_S = 120.0

    def test_follows_sdcs_small_map_route_and_writes_artifacts(self):
        roadmap = SDCSSmallMapRoadMap()
        node_sequence = (0, 2, 4, 6, 10)
        loop = 1
        # The runtime expands a finite loop by returning from the last node to
        # the first.  Use that same closed route for the telemetry comparison;
        # otherwise the valid 10 -> 0 return leg is reported as cross-track
        # error against the open 0 -> 10 reference.
        route_node_sequence = (*node_sequence, node_sequence[0])
        config = load_config(
            vehicle_config_file="config_vehicle_carla.yaml",
            selection_overrides={"ground_station": "null"},
            value_overrides={
                "mission": {"node_sequence": list(node_sequence), "loop": loop, "target_velocity": 5.0},
                "modules": {
                    "observer": {
                        "initial_pose": {"x": -1.14, "y": 1.053177, "theta": -math.pi / 2.0}
                    },
                    "planner": {"node_sequence": list(node_sequence), "loop": loop, "target_velocity": 5.0},
                    "simulation": {
                        "spawn_transform": {"x": -1.14, "y": -1.053177, "z": 0.5, "yaw": 90.0}
                    },
                },
            },
        )
        route = roadmap.generate_path(route_node_sequence)
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
            self.assertEqual(runtime.handle_command(VehicleCommand(CommandType.START)).runtime_state, State.RUNNING.name)
            for _ in range(round(self._DURATION_S / fixed_delta_s)):
                telemetry = runtime.step(dt=fixed_delta_s)
                lidar_scans = runtime.io.drain_lidar_scans()
                finite_lidar_bins = sum(
                    sum(math.isfinite(distance) for distance in scan.ranges_m)
                    for scan in lidar_scans
                )
                rows.append(
                    {
                        "time_s": telemetry.sensor_data.sensor_timestamp,
                        "gps_x_m": telemetry.sensor_data.gps_position[0],
                        "gps_y_m": telemetry.sensor_data.gps_position[1],
                        "estimate_x_m": telemetry.estimate.x,
                        "estimate_y_m": telemetry.estimate.y,
                        "speed_mps": telemetry.sensor_data.motor_tach,
                        "target_x_m": telemetry.target.target_x,
                        "target_y_m": telemetry.target.target_y,
                        "target_speed_mps": telemetry.target.target_velocity,
                        "throttle": telemetry.command.throttle,
                        "steering_rad": telemetry.command.steering,
                        "lidar_scan_count": len(lidar_scans),
                        "lidar_finite_bin_count": finite_lidar_bins,
                        "state": telemetry.state.name,
                    }
                )
                if telemetry.target.is_finished:
                    break
        finally:
            runtime.shutdown()

        np.testing.assert_allclose(route[0], roadmap.node_pose(node_sequence[0])[:2])
        gps = np.asarray([[row["gps_x_m"], row["gps_y_m"]] for row in rows], dtype=float)
        targets = np.asarray([[row["target_x_m"], row["target_y_m"]] for row in rows], dtype=float)
        cross_track_error = _nearest_route_distance(gps, route[:, :2])
        artifacts = create_artifact_run(
            category="integration",
            platform="carla",
            test_name="sdcs_path",
            metadata={"node_sequence": list(node_sequence), "loop": loop, "samples": len(rows)},
        )
        self._write_artifacts(rows, route, cross_track_error, artifacts.raw_directory, artifacts.figures_directory)

        self.assertGreaterEqual(len(rows), 1000)
        self.assertTrue(all(row["state"] == State.RUNNING.name for row in rows[:-1]))
        self.assertEqual(rows[-1]["state"], State.STOPPED.name)
        self.assertEqual(rows[-1]["target_speed_mps"], 0.0)
        self.assertGreater(np.linalg.norm(targets - route[0, :2], axis=1).max(), 10.0)
        self.assertGreater(max(row["speed_mps"] for row in rows), 0.15)
        self.assertGreater(max(row["throttle"] for row in rows), 0.01)
        self.assertGreater(max(abs(row["steering_rad"]) for row in rows), 0.02)
        self.assertGreater(sum(row["lidar_scan_count"] for row in rows), 0)
        self.assertGreater(max(row["lidar_finite_bin_count"] for row in rows), 0)
        self.assertLess(np.percentile(cross_track_error, 90.0), 2.0)
        self.assertLess(np.linalg.norm(gps[-1] - route[-1, :2]), 0.5)

    def _write_artifacts(
        self,
        rows: list[dict],
        route: np.ndarray,
        cross_track_error: np.ndarray,
        raw_directory: Path,
        figures_directory: Path,
    ) -> None:
        csv_path = raw_directory / "integration_carla_sdcs_path.csv"
        with csv_path.open("w", newline="", encoding="ascii") as file:
            writer = csv.DictWriter(file, fieldnames=[*rows[0], "cross_track_error_m"])
            writer.writeheader()
            for row, error_m in zip(rows, cross_track_error):
                writer.writerow({**row, "cross_track_error_m": float(error_m)})

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        time_s = np.asarray([row["time_s"] for row in rows], dtype=float)
        time_s -= time_s[0]
        gps = np.asarray([[row["gps_x_m"], row["gps_y_m"]] for row in rows], dtype=float)
        estimate = np.asarray([[row["estimate_x_m"], row["estimate_y_m"]] for row in rows], dtype=float)
        target = np.asarray([[row["target_x_m"], row["target_y_m"]] for row in rows], dtype=float)

        figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
        axes[0, 0].plot(route[:, 0], route[:, 1], "--", color="0.35", label="SDCS node route")
        axes[0, 0].plot(gps[:, 0], gps[:, 1], label="IOCarla GPS")
        axes[0, 0].plot(estimate[:, 0], estimate[:, 1], label="EKF estimate")
        axes[0, 0].plot(target[:, 0], target[:, 1], ":", label="planner target")
        axes[0, 0].set_title("CARLA SDCS Small-Map Path Following")
        axes[0, 0].set_xlabel("x (m)")
        axes[0, 0].set_ylabel("y (m)")
        axes[0, 0].axis("equal")
        axes[0, 0].grid(True)
        axes[0, 0].legend()

        axes[0, 1].plot(time_s, [row["speed_mps"] for row in rows], label="IOCarla speed")
        axes[0, 1].plot(time_s, [row["target_speed_mps"] for row in rows], label="target speed")
        axes[0, 1].set_xlabel("CARLA simulation time (s)")
        axes[0, 1].set_ylabel("speed (m/s)")
        axes[0, 1].grid(True)
        axes[0, 1].legend()

        axes[1, 0].plot(time_s, [row["throttle"] for row in rows], label="throttle")
        axes[1, 0].plot(time_s, [row["steering_rad"] for row in rows], label="steering")
        axes[1, 0].set_xlabel("CARLA simulation time (s)")
        axes[1, 0].set_ylabel("control input")
        axes[1, 0].grid(True)
        axes[1, 0].legend()

        axes[1, 1].plot(time_s, cross_track_error, label="GPS-to-route distance")
        axes[1, 1].set_xlabel("CARLA simulation time (s)")
        axes[1, 1].set_ylabel("cross-track error (m)")
        axes[1, 1].grid(True)
        axes[1, 1].legend()

        plot_path = figures_directory / "integration_carla_sdcs_path.png"
        figure.savefig(plot_path, dpi=160)
        plt.close(figure)
        self.assertTrue(csv_path.is_file())
        self.assertTrue(plot_path.is_file())


def _nearest_route_distance(points: np.ndarray, route: np.ndarray) -> np.ndarray:
    deltas = points[:, None, :] - route[None, :, :]
    return np.sqrt(np.sum(deltas * deltas, axis=2)).min(axis=1)


if __name__ == "__main__":
    unittest.main()
