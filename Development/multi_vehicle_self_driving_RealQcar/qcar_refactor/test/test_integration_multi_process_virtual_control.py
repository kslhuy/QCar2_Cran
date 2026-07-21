"""Run two scenario-defined virtual controller workers on distinct curves."""

import csv
from dataclasses import dataclass
from pathlib import Path
import sys
import unittest


_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

try:
    from .helper_simulator_process import run_two_vehicle_workers
except ImportError:
    from helper_simulator_process import run_two_vehicle_workers

from extra.simulator.virtual import load_virtual_setup


_SCENARIO_FILE = _ROOT / "config" / "scenarios" / "virtual_two_vehicle_control.yaml"
_ARTIFACT_DIR = _ROOT / "test" / "artifacts" / "virtual_multi_process_control"


@dataclass(frozen=True)
class VirtualControlTestConfig:
    """Test-owned execution duration; routes and speed belong to the scenario."""

    duration_s: float = 30.0
    dt_s: float = 0.02

    @property
    def cycles(self) -> int:
        return round(self.duration_s / self.dt_s)


_SIMULATION = VirtualControlTestConfig()


class TestMultiProcessVirtualControlIntegration(unittest.TestCase):
    def test_two_virtual_vehicles_follow_distinct_curves_and_save_plot(self):
        setup = load_virtual_setup(_SCENARIO_FILE)
        routes = {vehicle.vehicle_id: [list(point) for point in vehicle.route] for vehicle in setup.vehicles}
        results = run_two_vehicle_workers(
            self,
            project_root=_ROOT,
            runner_module="extra.simulator.virtual.process_runner",
            setup_file=_SCENARIO_FILE,
            cycles=_SIMULATION.cycles,
            extra_args=["--dt", str(_SIMULATION.dt_s)],
            require_v2v_trace=False,
        )

        rows_by_vehicle = {vehicle_id: result["rows"] for vehicle_id, result in results.items()}
        self.assertEqual(set(rows_by_vehicle), {1, 2})
        self.assertTrue(all(len(rows) == _SIMULATION.cycles for rows in rows_by_vehicle.values()))
        self.assertTrue(all(max(abs(row["throttle"]) for row in rows) > 0.01 for rows in rows_by_vehicle.values()))
        self.assertGreater(max(row["steering_rad"] for row in rows_by_vehicle[1]), 0.01)
        self.assertLess(min(row["steering_rad"] for row in rows_by_vehicle[2]), -0.01)

        self._write_artifacts(rows_by_vehicle, routes)
        self.assertTrue((_ARTIFACT_DIR / "vehicle_1.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "vehicle_2.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "trajectories.png").is_file())

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
