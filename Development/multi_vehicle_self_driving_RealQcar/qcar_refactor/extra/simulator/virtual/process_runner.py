"""Run one YAML-defined virtual vehicle in an independent process."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from core.vehicle_process import build_vehicle_process_runtime, run_vehicle_process
from core.types import GuiCommand
from extra.simulator.base import BaseSimulatorProcessManager
from extra.simulator.virtual.scenario import VirtualSetup, VirtualVehicleSetup, load_virtual_setup


class VirtualProcessManager(BaseSimulatorProcessManager):
    """Virtual setup adapter for the shared extra-layer process manager."""

    platform_name = "virtual"

    def load_setup(self, setup_file: str) -> VirtualSetup:
        return load_virtual_setup(setup_file)

    def build_process_spec(self, setup: VirtualSetup, vehicle: VirtualVehicleSetup):
        return vehicle.to_process_spec(setup.simulation_profile)


def run_virtual_vehicle(vehicle: VirtualVehicleSetup, cycles: int, dt_s: float, realtime: bool = False):
    """Run one deterministic virtual vehicle through the shared lifecycle.

    Set ``realtime`` only when testing wall-clock transports such as UDP V2V.
    The default remains unpaced for fast deterministic control tests.
    """
    if dt_s <= 0:
        raise ValueError("dt_s must be positive")
    runtime = build_vehicle_process_runtime(vehicle.to_process_spec())

    def on_step(_telemetry):
        if realtime:
            time.sleep(dt_s)

    return run_vehicle_process(runtime, cycles=cycles, dt=dt_s, on_step=on_step)


def main(argv=None) -> int:
    """Run one virtual scenario entry and emit serializable telemetry."""
    parser = argparse.ArgumentParser(description="Run one vehicle from a multi-process virtual setup")
    parser.add_argument("--setup-file", required=True)
    parser.add_argument("--vehicle-id", required=True, type=int)
    parser.add_argument("--cycles", type=int, default=120)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--start-file", type=Path, required=True)
    parser.add_argument("--realtime", action="store_true", help="pace virtual steps for wall-clock transport tests")
    parser.add_argument("--build-fleet", action="store_true")
    args = parser.parse_args(argv)

    manager = VirtualProcessManager(args.setup_file, args.vehicle_id)
    rows = []

    def on_ready(_runtime, _context):
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.touch()
        manager.wait_for_start_signal(args.start_file)

    def on_step(runtime, telemetry, _context):
        rows.append(manager.serialize_telemetry(telemetry, runtime.fleet))
        if args.realtime:
            time.sleep(args.dt)

    def on_running(runtime, _context):
        if args.build_fleet:
            runtime.handle_command(GuiCommand("BUILD_FLEET", {}))

    context, telemetry = manager.run(
        args.cycles,
        dt=args.dt,
        on_ready=on_ready,
        on_running=on_running if args.build_fleet else None,
        on_step=on_step if args.realtime else None,
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
