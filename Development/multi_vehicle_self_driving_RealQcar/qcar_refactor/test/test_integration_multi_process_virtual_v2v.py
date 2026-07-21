"""Run two virtual process-runner workers and plot V2V peer diagnostics."""

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


_SCENARIO_FILE = _ROOT / "config" / "scenarios" / "virtual_two_vehicle.yaml"
_ARTIFACT_DIR = _ROOT / "test" / "artifacts" / "virtual_multi_process_v2v"
_DURATION_S = 6.0
_DT_S = 0.02
_CYCLES = round(_DURATION_S / _DT_S)


class TestMultiProcessVirtualV2VIntegration(unittest.TestCase):
    def test_two_virtual_vehicle_processes_plot_peer_state_latency_and_packet_loss(self):
        results = run_two_vehicle_workers(
            self,
            project_root=_ROOT,
            runner_module="test.helper_v2v_trace",
            setup_file=_SCENARIO_FILE,
            cycles=_CYCLES,
            extra_args=["--platform", "virtual", "--dt", str(_DT_S), "--realtime"],
        )

        assert_two_vehicle_v2v_results(self, results, _CYCLES)
        write_v2v_artifacts(results, _ARTIFACT_DIR, "Virtual")
        self.assertTrue((_ARTIFACT_DIR / "vehicle_1_v2v.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "vehicle_2_v2v.csv").is_file())
        self.assertTrue((_ARTIFACT_DIR / "peer_state_latency_loss.png").is_file())


if __name__ == "__main__":
    unittest.main()
