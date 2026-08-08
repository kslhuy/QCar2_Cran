"""Process-level test for two scenario-defined virtual vehicle workers."""

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


_SCENARIO_FILE = _ROOT / "config" / "scenarios" / "test" / "virtual_two_vehicle.yaml"


class TestMultiProcessVirtualIntegration(unittest.TestCase):
    def test_two_virtual_vehicle_processes_run_independently(self):
        results = run_two_vehicle_workers(
            self,
            project_root=_ROOT,
            runner_module="extra.platform.virtual.process_runner",
            setup_file=_SCENARIO_FILE,
            cycles=40,
            require_v2v_trace=False,
        )

        self.assertEqual(set(results), {1, 2})
        self.assertTrue(all(len(result["rows"]) == 40 for result in results.values()))
        self.assertTrue(all(result["rows"][-1]["time_s"] > 0.0 for result in results.values()))


if __name__ == "__main__":
    unittest.main()
