"""Command-line entry point for one configured vehicle runtime."""

from __future__ import annotations

import argparse
import logging
import time

from core.vehicle_config import load_config
from core.module_factory import build_vehicle_modules
from core.vehicle_logic import VehicleRuntime


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run one QCar refactor vehicle actor")
    parser.add_argument("--vehicle-id", type=int, help="override the configured vehicle ID")
    parser.add_argument("--cycles", type=int, default=None, help="stop after this many control-loop cycles")
    parser.add_argument("--vehicle-config", help="vehicle profile file in config/")
    parser.add_argument("--headless", action="store_true", help="use config_vehicle_headless.yaml")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    value_overrides = {}
    if args.vehicle_id is not None:
        value_overrides["vehicle_id"] = args.vehicle_id
    vehicle_config_file = args.vehicle_config or (
        "config_vehicle_headless.yaml" if args.headless else "config_vehicle.yaml"
    )
    config = load_config(
        vehicle_config_file=vehicle_config_file,
        value_overrides=value_overrides,
    )
    modules = build_vehicle_modules(config)
    runtime = VehicleRuntime(
        config,
        modules.io,
        modules.observer,
        modules.planner,
        modules.controller,
        modules.v2v,
        simulation=modules.simulation,
    )
    period_s = 1.0 / config.runtime["loop_rate_hz"]

    try:
        runtime.start()
        cycles = 0
        while args.cycles is None or cycles < args.cycles:
            started_at = time.monotonic()
            runtime.step()
            cycles += 1
            time.sleep(max(0.0, period_s - (time.monotonic() - started_at)))
    except KeyboardInterrupt:
        return 0
    finally:
        runtime.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
