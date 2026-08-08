"""Run two virtual process-runner workers and plot V2V peer diagnostics."""

from pathlib import Path
import sys
import unittest


_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from test.helper_artifacts import create_artifact_run

try:
    from .helper_simulator_process import run_two_vehicle_workers
    from .helper_v2v_diagnostics import assert_two_vehicle_v2v_results, write_v2v_artifacts
except ImportError:
    from helper_simulator_process import run_two_vehicle_workers
    from helper_v2v_diagnostics import assert_two_vehicle_v2v_results, write_v2v_artifacts


_SCENARIO_FILE = _ROOT / "config" / "scenarios" / "test" / "virtual_two_vehicle.yaml"
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
        artifacts = create_artifact_run(
            category="integration",
            platform="virtual",
            test_name="multi_process_v2v",
            metadata={"scenario": _SCENARIO_FILE.name, "vehicle_ids": [1, 2], "cycles": _CYCLES},
        )
        write_v2v_artifacts(results, artifacts.raw_directory, artifacts.figures_directory, "Virtual")
        self.assertTrue((artifacts.raw_directory / "vehicle_1_v2v.csv").is_file())
        self.assertTrue((artifacts.raw_directory / "vehicle_2_v2v.csv").is_file())
        self.assertTrue((artifacts.figures_directory / "peer_state_latency_loss.png").is_file())


if __name__ == "__main__":
    unittest.main()
