"""Run one manifest-defined CARLA vehicle in its own process."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from core.commands import CommandSource, CommandType, VehicleCommand
from extra.platform.base import ScenarioPlatformProcessManager
from extra.platform.carla.scenario import CarlaSetup, CarlaVehicleSetup, load_carla_setup


class CarlaProcessManager(ScenarioPlatformProcessManager):
    """CARLA setup adapter for the shared extra-layer process manager."""

    platform_name = "CARLA"

    def load_setup(self, setup_file: str) -> CarlaSetup:
        return load_carla_setup(setup_file)

    def build_process_spec(self, setup: CarlaSetup, vehicle: CarlaVehicleSetup):
        fleet_spec = setup.fleet.to_runtime_spec() if setup.fleet is not None else None
        return vehicle.to_process_spec(setup.simulation_profile, fleet_spec)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run one vehicle from a multi-process CARLA setup")
    parser.add_argument("--setup-file", required=True)
    parser.add_argument("--vehicle-id", required=True, type=int)
    parser.add_argument("--cycles", type=int, help="stop after this many control-loop cycles")
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--start-file", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, help="launcher shutdown signal file")
    parser.add_argument("--build-fleet", action="store_true")
    args = parser.parse_args(argv)

    manager = CarlaProcessManager(args.setup_file, args.vehicle_id)
    rows = []

    def on_ready(runtime, context):
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.touch()
        manager.wait_for_start_signal(args.start_file)
        if context.vehicle.tick_owner:
            # Give followers time to enter wait_for_tick before frame one.
            time.sleep(1.0)

    def on_step(runtime, telemetry, context):
        if args.cycles is not None:
            rows.append(manager.serialize_telemetry(telemetry, runtime.fleet))
        if context.vehicle.tick_owner:
            time.sleep(float(runtime.config.module("simulation")["fixed_delta_seconds"]))

    def on_running(runtime, _context):
        if args.build_fleet:
            runtime.handle_command(VehicleCommand(CommandType.BUILD_FLEET, source=CommandSource.SIMULATOR))

    context, telemetry = manager.run(
        args.cycles,
        dt=lambda runtime, _context: float(runtime.config.module("simulation")["fixed_delta_seconds"]),
        should_stop=args.stop_file.exists if args.stop_file is not None else None,
        on_ready=on_ready,
        on_running=on_running if args.build_fleet else None,
        on_step=on_step,
        collect_telemetry=args.cycles is not None,
    )
    print(
        json.dumps(
            {
                "vehicle_id": context.vehicle.vehicle_id,
                "rows": rows or [manager.serialize_telemetry(item) for item in telemetry],
            }
        ),
        flush=True,
    )
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
