"""Manual local-CARLA live LiDAR viewer.

Run directly after the CARLA SDCS world is ready::

    python test/test_integration_carla_lidar_viewer.py --duration-s 30

The test selects the existing one-vehicle CARLA scenario, keeps the actor
stationary at its scenario spawn, and displays scans only after ``IOCarla``
has converted them to the common ``LaserScanSample`` contract. It does not
start ``VehicleRuntime`` or issue a driving command.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.vehicle_config import load_config
from extra.ground_station.utils.live_plotting import LiveLidarViewer
from extra.platform.carla.scenario import load_carla_setup
from extra.platform.carla.session import CarlaSession
from test.helper_artifacts import create_artifact_run
from utils.io.io_carla import IOCarla
from utils.localization import plot_laser_scan_artifact, write_laser_scan_artifact


def main(argv: list[str] | None = None) -> int:
    arguments = _parse_arguments(argv)
    artifact_paths = run_carla_lidar_viewer(
        scenario_file=arguments.scenario_file,
        vehicle_id=arguments.vehicle_id,
        duration_s=arguments.duration_s,
        realtime=not arguments.as_fast_as_possible,
    )
    print("CARLA live LiDAR viewer completed.")
    for name, path in artifact_paths.items():
        print(f"{name}: {path}")
    return 0


def run_carla_lidar_viewer(
    *,
    scenario_file: str = "config/scenarios/test/carla_sdcs_small_map.yaml",
    vehicle_id: int = 0,
    duration_s: float = 30.0,
    realtime: bool = True,
) -> dict[str, Path]:
    """Show live normalized CARLA scans and save the final scan artifact."""

    if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool) or vehicle_id < 0:
        raise ValueError("vehicle_id must be a non-negative integer")
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be positive and finite")
    setup = load_carla_setup(scenario_file)
    try:
        selected_vehicle = next(item for item in setup.vehicles if item.vehicle_id == vehicle_id)
    except StopIteration as error:
        raise ValueError(f"Scenario {scenario_file} has no vehicle_id {vehicle_id}") from error
    config = load_config(
        vehicle_config_file=selected_vehicle.vehicle_config_file,
        selection_overrides={"simulation": setup.simulation_profile, "ground_station": "null"},
        value_overrides={
            "modules": {
                "simulation": {
                    "spawn_transform": selected_vehicle.spawn_transform,
                    "tick_owner": selected_vehicle.tick_owner,
                    "warmup_ticks": 0,
                }
            }
        },
    )
    fixed_delta_s = float(config.module("simulation")["fixed_delta_seconds"])
    session = CarlaSession(config.module("simulation"))
    io = IOCarla(config.module("io"), session, vehicle_id=vehicle_id)
    viewer: LiveLidarViewer | None = None
    last_scan = None
    rendered_scan_count = 0
    try:
        # Load CARLA and start its native client before importing/creating the
        # Qt window. On Windows this avoids a native DLL load-order conflict
        # between the CARLA client and Qt rendering libraries.
        session.start()
        viewer = LiveLidarViewer(title=f"CARLA LiDAR — scenario vehicle {vehicle_id}")
        deadline = time.monotonic() + duration_s
        next_tick_at = time.monotonic()
        while time.monotonic() < deadline and viewer.process_events():
            session.tick()
            io.read_to_cache()
            scans = io.drain_lidar_scans()
            for scan in scans:
                viewer.show_scan(scan, vehicle_id=vehicle_id)
                last_scan = scan
                rendered_scan_count += 1
            if realtime:
                next_tick_at += fixed_delta_s
                time.sleep(max(0.0, next_tick_at - time.monotonic()))
    finally:
        io.close()
        session.close()
        if viewer is not None:
            viewer.close()

    if last_scan is None or rendered_scan_count == 0:
        raise RuntimeError("CARLA LiDAR produced no normalized scans")
    artifact_run = create_artifact_run(
        category="diagnostic",
        platform="carla",
        test_name="lidar_viewer",
        metadata={
            "scenario_file": scenario_file,
            "vehicle_id": vehicle_id,
            "duration_s": duration_s,
            "rendered_scan_count": rendered_scan_count,
        },
    )
    data_path, metadata_path = write_laser_scan_artifact(last_scan, artifact_run.raw_directory)
    figure_path = plot_laser_scan_artifact(
        metadata_path, output_path=artifact_run.figures_directory / "last_live_scan.png"
    )
    return {
        "manifest": artifact_run.manifest_path,
        "scan_csv": data_path,
        "scan_metadata": metadata_path,
        "scan_plot": figure_path,
    }


def _parse_arguments(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Show the live normalized LiDAR scan from an existing CARLA scenario.")
    parser.add_argument("--scenario-file", default="config/scenarios/test/carla_sdcs_small_map.yaml")
    parser.add_argument("--vehicle-id", type=int, default=0)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--as-fast-as-possible", action="store_true")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(main())
