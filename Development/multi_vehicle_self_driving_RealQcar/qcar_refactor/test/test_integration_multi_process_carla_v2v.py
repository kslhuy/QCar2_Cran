"""Direct-run CARLA V2V latency and packet-loss diagnostic test.

Start CARLA on ``127.0.0.1:2000``, then run this file directly. The latency
calculation uses monotonic clocks from two local worker processes, so it is a
localhost diagnostic only; it is not valid for vehicles on separate machines.
"""

from pathlib import Path
import sys
import unittest


_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

try:
    from .helper_simulator_process import run_two_vehicle_workers
    from .helper_v2v_diagnostics import assert_two_vehicle_v2v_results, write_v2v_artifacts
except ImportError:
    from helper_simulator_process import run_two_vehicle_workers
    from helper_v2v_diagnostics import assert_two_vehicle_v2v_results, write_v2v_artifacts


_SCENARIO_FILE = _ROOT / "config" / "scenarios" / "test" / "carla_two_vehicle_v2v.yaml"
_ARTIFACT_DIR = _ROOT / "test" / "artifacts" / "carla_multi_process_v2v"
_CYCLES = 240


@unittest.skipUnless(__name__ == "__main__", "run this CARLA V2V integration test file directly")
class TestCarlaMultiProcessV2V(unittest.TestCase):
    def test_two_carla_vehicles_plot_peer_state_latency_and_packet_loss(self):
        results = run_two_vehicle_workers(
            self,
            project_root=_ROOT,
            runner_module="test.helper_v2v_trace",
            setup_file=_SCENARIO_FILE,
            cycles=_CYCLES,
            extra_args=["--platform", "carla"],
        )

        assert_two_vehicle_v2v_results(self, results, _CYCLES)
        write_v2v_artifacts(results, _ARTIFACT_DIR, "CARLA")
        self.assertTrue((_ARTIFACT_DIR / "vehicle_1_v2v.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "vehicle_2_v2v.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "peer_state_latency_loss.png").is_file())


if __name__ == "__main__":
    unittest.main()
