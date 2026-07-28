"""Real-time four-process virtual fleet-control integration with artifacts."""

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

from extra.simulator.virtual import load_virtual_setup

_SCENARIO = _ROOT / "config" / "scenarios" / "test" / "virtual_four_vehicle_fleet_control.yaml"
_ARTIFACTS = _ROOT / "test" / "artifacts" / "virtual_fleet_control"
_FLEET_ORDER = (1, 2, 3, 4)


class TestVirtualFleetControl(unittest.TestCase):
    def test_four_process_fleet_follows_from_declared_spawns_and_records_artifacts(self):
        cycles, dt = 600, 0.02
        setup = load_virtual_setup(_SCENARIO)
        self.assertEqual(tuple(vehicle.vehicle_id for vehicle in setup.vehicles), _FLEET_ORDER)
        expected_x = {vehicle_id: -0.8 * (vehicle_id - 1) for vehicle_id in _FLEET_ORDER}
        for vehicle in setup.vehicles:
            self.assertAlmostEqual(vehicle.initial_pose[0], expected_x[vehicle.vehicle_id])
            self.assertEqual(vehicle.initial_pose[1:], (0.0, 0.0))
            spec = vehicle.to_process_spec(setup.simulation_profile)
            self.assertAlmostEqual(spec.value_overrides["modules"]["io"]["initial_x"], expected_x[vehicle.vehicle_id])
            self.assertAlmostEqual(
                spec.value_overrides["modules"]["observer"]["initial_pose"]["x"], expected_x[vehicle.vehicle_id]
            )

        run_fleet_integration(
            self,
            project_root=_ROOT,
            runner_module="extra.simulator.virtual.process_runner",
            setup_file=_SCENARIO,
            cycles=cycles,
            extra_args=["--dt", str(dt), "--realtime", "--build-fleet"],
            timeout_s=cycles * dt + 30.0,
            fleet_order=_FLEET_ORDER,
            platform_name="Virtual",
            artifact_directory=_ARTIFACTS,
            summary_filename="fleet_virtual_summary.png",
            validation_position_columns=("estimate_x_m", "estimate_y_m"),
            minimum_gap_m=0.30,
            minimum_steering_rad=0.01,
            desired_gap_m=0.8,
            time_headway_s=0.5,
            maximum_steady_state_spacing_error_m=0.8,
            require_final_active=True,
        )


if __name__ == "__main__":
    unittest.main()
