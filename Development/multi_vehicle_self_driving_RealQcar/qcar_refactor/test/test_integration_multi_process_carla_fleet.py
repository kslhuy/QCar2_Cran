"""Direct-run CARLA fleet-control integration; start CARLA before running."""

from pathlib import Path
import sys
import unittest

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
try:
    from .helper_fleet import run_fleet_integration
except ImportError:
    from helper_fleet import run_fleet_integration

_SCENARIO = _ROOT / "config" / "scenarios" / "test" / "carla_three_vehicle_fleet_control.yaml"
_ARTIFACTS = _ROOT / "test" / "artifacts" / "carla_fleet_control"
_FLEET_ORDER = (1, 2, 3)


@unittest.skipUnless(__name__ == "__main__", "run directly with a CARLA server available")
class TestCarlaFleetControl(unittest.TestCase):
    def test_three_carla_processes_build_fleet_and_record_artifacts(self):
        run_fleet_integration(
            self,
            project_root=_ROOT,
            runner_module="extra.simulator.carla.process_runner",
            setup_file=_SCENARIO,
            cycles=800,
            extra_args=["--build-fleet"],
            timeout_s=90.0,
            fleet_order=_FLEET_ORDER,
            platform_name="CARLA",
            artifact_directory=_ARTIFACTS,
            summary_filename="fleet_carla_summary.png",
            minimum_gap_m=4.0,
            minimum_steering_rad=0.02,
            desired_gap_m=5.0,
            time_headway_s=0.5,
            maximum_steady_state_spacing_error_m=2.5,
            display_y_sign=-1.0,
            exported_position_prefix="carla",
            trajectory_aspect="equal",
        )

if __name__ == "__main__":
    unittest.main()
