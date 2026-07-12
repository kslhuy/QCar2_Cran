"""Two independent virtual vehicles running the planner/controller pipeline."""

import csv
from dataclasses import dataclass
import math
import multiprocessing
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime, run_vehicle_process


_ROOT = Path(__file__).resolve().parents[1]
_ARTIFACT_DIR = _ROOT / "test" / "artifacts" / "multi_process_virtual_control"


@dataclass(frozen=True)
class VirtualControlTestConfig:
    """Self-contained simulation settings for this integration test."""

    duration_s: float = 30.0
    dt_s: float = 0.02
    target_velocity_mps: float = 0.30

    @property
    def cycles(self) -> int:
        return round(self.duration_s / self.dt_s)


_SIMULATION = VirtualControlTestConfig()


def _virtual_control_worker(vehicle_id, route, simulation, output_queue):
    spec = VehicleProcessSpec(
        vehicle_id=vehicle_id,
        vehicle_config_file="config_vehicle_virtual.yaml",
        value_overrides={
            "mission": {"path": route, "target_velocity": simulation.target_velocity_mps},
            "modules": {
                "planner": {"path_source": route, "target_velocity": simulation.target_velocity_mps},
            },
        },
    )
    runtime = build_vehicle_process_runtime(spec)
    telemetry = run_vehicle_process(runtime, cycles=simulation.cycles, dt=simulation.dt_s)
    output_queue.put(
        {
            "vehicle_id": vehicle_id,
            "rows": [_serialize(sample) for sample in telemetry],
        }
    )


def _serialize(sample):
    return {
        "time_s": float(sample.sensor_data.sensor_timestamp),
        "gps_x_m": float(sample.sensor_data.gps_position[0]),
        "gps_y_m": float(sample.sensor_data.gps_position[1]),
        "estimate_x_m": float(sample.estimate.x),
        "estimate_y_m": float(sample.estimate.y),
        "speed_mps": float(sample.estimate.velocity),
        "target_x_m": float(sample.target.target_x),
        "target_y_m": float(sample.target.target_y),
        "throttle": float(sample.command.throttle),
        "steering_rad": float(sample.command.steering),
        "state": sample.state.name,
    }


def _curved_route(direction):
    """Return a smooth route with an observable controller response."""
    return [
        [round(x, 3), round(direction * 0.55 * math.sin(math.pi * x / 6.0), 3)]
        for x in (0.4 * index for index in range(16))
    ]


class TestMultiProcessVirtualControlIntegration(unittest.TestCase):
    def test_two_virtual_vehicles_follow_distinct_curves_and_save_plot(self):
        routes = {
            1: _curved_route(direction=1.0),
            2: _curved_route(direction=-1.0),
        }
        context = multiprocessing.get_context("spawn")
        output_queue = context.Queue()
        processes = [
            context.Process(target=_virtual_control_worker, args=(vehicle_id, route, _SIMULATION, output_queue))
            for vehicle_id, route in routes.items()
        ]

        try:
            for process in processes:
                process.start()
            # Receive before join: a child can otherwise block flushing its
            # telemetry payload into the multiprocessing pipe.
            results = [output_queue.get(timeout=30) for _ in processes]
            for process in processes:
                process.join(timeout=30)
                self.assertEqual(process.exitcode, 0)
        finally:
            for process in processes:
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=5)

        rows_by_vehicle = {result["vehicle_id"]: result["rows"] for result in results}
        self.assertEqual(set(rows_by_vehicle), {1, 2})
        self.assertTrue(all(len(rows) == _SIMULATION.cycles for rows in rows_by_vehicle.values()))
        for rows in rows_by_vehicle.values():
            self._assert_safe_runtime_progression(rows)
        self.assertTrue(all(max(abs(row["throttle"]) for row in rows) > 0.01 for rows in rows_by_vehicle.values()))
        self.assertGreater(max(row["steering_rad"] for row in rows_by_vehicle[1]), 0.01)
        self.assertLess(min(row["steering_rad"] for row in rows_by_vehicle[2]), -0.01)

        self._write_artifacts(rows_by_vehicle, routes)
        self.assertTrue((_ARTIFACT_DIR / "vehicle_1.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "vehicle_2.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "trajectories.png").is_file())

    def _assert_safe_runtime_progression(self, rows):
        """A long run may finish its route, then must remain safely stopped."""
        self.assertEqual(rows[0]["state"], "RUNNING")
        stopped = False
        for row in rows:
            self.assertNotEqual(row["state"], "ERROR")
            if row["state"] == "STOPPED":
                stopped = True
                self.assertEqual(row["throttle"], 0.0)
                self.assertEqual(row["steering_rad"], 0.0)
            else:
                self.assertFalse(stopped, "runtime resumed after path-completion stop")
                self.assertEqual(row["state"], "RUNNING")

    def _write_artifacts(self, rows_by_vehicle, routes):
        _ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
        for vehicle_id, rows in rows_by_vehicle.items():
            with (_ARTIFACT_DIR / f"vehicle_{vehicle_id}.csv").open("w", newline="", encoding="ascii") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(2, 1, figsize=(10, 9), constrained_layout=True)
        colors = {1: "tab:blue", 2: "tab:orange"}
        for vehicle_id, rows in rows_by_vehicle.items():
            color = colors[vehicle_id]
            route = routes[vehicle_id]
            axes[0].plot(*zip(*route), linestyle="--", color=color, alpha=0.55, label=f"vehicle {vehicle_id} path")
            axes[0].plot(
                [row["estimate_x_m"] for row in rows],
                [row["estimate_y_m"] for row in rows],
                color=color,
                label=f"vehicle {vehicle_id} estimate",
            )
            time_s = [row["time_s"] for row in rows]
            axes[1].plot(time_s, [row["steering_rad"] for row in rows], color=color, label=f"vehicle {vehicle_id} steering")

        axes[0].set_title("Independent Virtual Vehicle Controller Trajectories")
        axes[0].set_xlabel("x [m]")
        axes[0].set_ylabel("y [m]")
        axes[0].axis("equal")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[1].set_title("Controller Steering Commands")
        axes[1].set_xlabel("virtual time [s]")
        axes[1].set_ylabel("steering [rad]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        figure.savefig(_ARTIFACT_DIR / "trajectories.png", dpi=150)
        plt.close(figure)


if __name__ == "__main__":
    unittest.main()
