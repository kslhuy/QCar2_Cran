"""Run one manifest-defined CARLA vehicle in its own process."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime, run_vehicle_process
from extra.launch.carla_scenario import parse_simulation_setup


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run one vehicle from a multi-process CARLA setup")
    parser.add_argument("--setup-file", required=True)
    parser.add_argument("--vehicle-id", required=True, type=int)
    parser.add_argument("--cycles", type=int, default=120)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--start-file", type=Path, required=True)
    args = parser.parse_args(argv)

    setup = parse_simulation_setup(["--setup-file", args.setup_file])
    vehicle = next((item for item in setup.vehicles if item.vehicle_id == args.vehicle_id), None)
    if vehicle is None:
        raise ValueError(f"Vehicle {args.vehicle_id} is not in the simulation setup")

    route = [list(point) for point in vehicle.route]
    spec = VehicleProcessSpec(
        vehicle_id=vehicle.vehicle_id,
        vehicle_config_file=vehicle.vehicle_config_file,
        value_overrides={
            "vehicle_id": vehicle.vehicle_id,
            "mission": {"path": route},
            "modules": {
                "planner": {"path_source": route},
                "simulation": {
                    "host": setup.host,
                    "port": setup.port,
                    "spawn_transform": vehicle.spawn_transform,
                    "tick_owner": vehicle.tick_owner,
                    "warmup_ticks": 0,
                },
            },
        },
    )
    runtime = build_vehicle_process_runtime(spec)

    def on_ready(_runtime):
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.touch()
        _wait_for_start(args.start_file)
        if vehicle.tick_owner:
            # Give follower processes time to enter wait_for_tick before frame one.
            time.sleep(1.0)

    dt = float(runtime.config.module("simulation")["fixed_delta_seconds"])
    def on_step(_telemetry):
        if vehicle.tick_owner:
            time.sleep(dt)

    telemetry = run_vehicle_process(runtime, args.cycles, dt=dt, on_ready=on_ready, on_step=on_step)
    print(json.dumps({"vehicle_id": vehicle.vehicle_id, "rows": [_serialize_telemetry(item) for item in telemetry]}), flush=True)
    return 0


def _wait_for_start(path: Path, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not path.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for multi-vehicle start signal")
        time.sleep(0.02)


def _serialize_telemetry(telemetry) -> dict:
    sensor = telemetry.sensor_data
    return {
        "time_s": float(sensor.sensor_timestamp),
        "gps_x_m": float(sensor.gps_position[0]),
        "gps_y_m": float(sensor.gps_position[1]),
        "speed_mps": float(sensor.motor_tach),
        "throttle": float(telemetry.command.throttle),
        "steering_rad": float(telemetry.command.steering),
    }


if __name__ == "__main__":
    raise SystemExit(main())
