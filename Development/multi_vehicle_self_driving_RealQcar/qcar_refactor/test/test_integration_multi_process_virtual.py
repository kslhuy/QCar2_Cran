"""Process-level integration test proving the generic runner supports virtual IO."""

import multiprocessing
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime, run_vehicle_process


def _virtual_vehicle_worker(vehicle_id, route, output_queue):
    spec = VehicleProcessSpec(
        vehicle_id=vehicle_id,
        vehicle_config_file="config_vehicle_virtual.yaml",
        value_overrides={
            "mission": {"path": route, "target_velocity": 0.3},
            "modules": {"planner": {"path_source": route}},
        },
    )
    runtime = build_vehicle_process_runtime(spec)
    telemetry = run_vehicle_process(runtime, cycles=40, dt=0.02)
    output_queue.put(
        {
            "vehicle_id": vehicle_id,
            "samples": len(telemetry),
            "last_timestamp": telemetry[-1].sensor_data.sensor_timestamp,
        }
    )


class TestMultiProcessVirtualIntegration(unittest.TestCase):
    def test_two_virtual_vehicle_processes_run_independently(self):
        context = multiprocessing.get_context("spawn")
        output_queue = context.Queue()
        routes = {
            1: [[0.0, 0.0], [1.0, 0.2], [2.0, 0.0]],
            2: [[0.0, 1.0], [1.0, 1.2], [2.0, 1.0]],
        }
        processes = [
            context.Process(target=_virtual_vehicle_worker, args=(vehicle_id, route, output_queue))
            for vehicle_id, route in routes.items()
        ]
        for process in processes:
            process.start()
        for process in processes:
            process.join(timeout=30)
            self.assertEqual(process.exitcode, 0)

        results = [output_queue.get(timeout=5) for _ in processes]
        self.assertEqual({result["vehicle_id"] for result in results}, {1, 2})
        self.assertTrue(all(result["samples"] == 40 for result in results))
        self.assertTrue(all(result["last_timestamp"] > 0.0 for result in results))


if __name__ == "__main__":
    unittest.main()
