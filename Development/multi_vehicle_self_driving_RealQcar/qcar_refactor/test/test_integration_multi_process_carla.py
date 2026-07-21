"""Tutorial: launch two CARLA vehicle-control processes in one shared world.

Start a CARLA server first, then run this file directly:

    python qcar_refactor/test/test_integration_carla_multi_process.py

The parent test loads ``config/scenarios/carla_two_vehicle.yaml`` and starts one
``extra.simulator.carla.process_runner`` child per vehicle. Every child
connects to the same CARLA ``host:port``. Only the vehicle whose manifest entry
sets ``tick_owner: true`` calls ``world.tick()``; follower processes wait for
the completed frame. Each child owns its own CARLA actor, sensors, runtime,
planner, controller, and IO adapter.

To change the test, edit the scenario file: add vehicle entries, choose unique
spawn transforms, assign distinct routes, and keep exactly one tick owner. The
parent test owns CSV/PNG generation and writes results under
``test/artifacts/carla_multi_process/``. It also terminates child processes if
setup or the readiness barrier fails.
"""

import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest


_ROOT = Path(__file__).resolve().parents[1]
_SCENARIO_FILE = _ROOT / "config" / "scenarios" / "carla_two_vehicle.yaml"


@unittest.skipUnless(__name__ == "__main__", "run this CARLA multi-process integration test file directly")
class TestCarlaMultiProcess(unittest.TestCase):
    def test_two_vehicle_processes_record_independent_trajectories(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            start_file = directory / "start.signal"
            record_dir = _ROOT / "test" / "artifacts" / "carla_multi_process"
            processes = []
            try:
                # The parent is only a launcher and artifact collector. Each child receives its own scenario entry through the manifest.
                for vehicle_id in (1, 2):
                    ready_file = directory / f"vehicle_{vehicle_id}.ready"
                    command = [
                        sys.executable, "-m", "extra.simulator.carla.process_runner",
                        "--setup-file", str(_SCENARIO_FILE), "--vehicle-id", str(vehicle_id),
                        "--cycles", "120", "--ready-file", str(ready_file),
                        "--start-file", str(start_file),
                    ]
                    processes.append(
                        subprocess.Popen(
                            command,
                            cwd=_ROOT,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                        )
                    )

                self._wait_for_files([directory / "vehicle_1.ready", directory / "vehicle_2.ready"], processes)
                start_file.touch()
                results = {}
                for process in processes:
                    output, _ = process.communicate(timeout=45)
                    self.assertEqual(process.returncode, 0, output)
                    result = self._result_from_output(output)
                    results[result["vehicle_id"]] = result["rows"]
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.terminate()
                        process.wait(timeout=10)

            for vehicle_id in (1, 2):
                self._write_artifacts(record_dir, vehicle_id, results[vehicle_id])
                csv_path = record_dir / f"carla_multi_vehicle_{vehicle_id}.csv"
                plot_path = record_dir / f"carla_multi_vehicle_{vehicle_id}.png"
                self.assertTrue(csv_path.is_file())
                self.assertGreater(csv_path.stat().st_size, 0)
                self.assertTrue(plot_path.is_file())
                self.assertGreater(plot_path.stat().st_size, 0)

    def _wait_for_files(self, paths, processes, timeout_s=35.0):
        """Wait longer than one worker's CARLA client connection timeout."""
        deadline = time.monotonic() + timeout_s
        while not all(path.exists() for path in paths):
            exited = [process for process in processes if process.poll() is not None]
            if exited:
                outputs = []
                for process in exited:
                    output, _ = process.communicate()
                    outputs.append(f"worker exited with {process.returncode}:\n{output}")
                self.fail("CARLA vehicle worker exited before readiness:\n" + "\n".join(outputs))
            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for vehicle processes to become ready")
            time.sleep(0.05)

    def _result_from_output(self, output):
        for line in reversed(output.splitlines()):
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict) and "vehicle_id" in value and "rows" in value:
                return value
        self.fail(f"Vehicle process did not emit a telemetry result:\n{output}")

    def _write_artifacts(self, record_dir, vehicle_id, rows):
        self.assertTrue(rows)
        record_dir.mkdir(parents=True, exist_ok=True)
        csv_path = record_dir / f"carla_multi_vehicle_{vehicle_id}.csv"
        with csv_path.open("w", newline="", encoding="ascii") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        time_s = [row["time_s"] - rows[0]["time_s"] for row in rows]
        figure, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
        axes[0].plot([row["gps_x_m"] for row in rows], [row["gps_y_m"] for row in rows])
        axes[0].set_title(f"CARLA vehicle {vehicle_id} trajectory")
        axes[0].set_xlabel("x (m)")
        axes[0].set_ylabel("y (m)")
        axes[0].axis("equal")
        axes[0].grid(True)
        axes[1].plot(time_s, [row["speed_mps"] for row in rows], label="speed")
        axes[1].plot(time_s, [row["throttle"] for row in rows], label="throttle")
        axes[1].plot(time_s, [row["steering_rad"] for row in rows], label="steering")
        axes[1].set_xlabel("CARLA simulation time (s)")
        axes[1].grid(True)
        axes[1].legend()
        figure.savefig(record_dir / f"carla_multi_vehicle_{vehicle_id}.png", dpi=150)
        plt.close(figure)


if __name__ == "__main__":
    unittest.main()
