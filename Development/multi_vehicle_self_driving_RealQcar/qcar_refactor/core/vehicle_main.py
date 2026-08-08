"""Command-line entry point for one configured vehicle runtime."""

from __future__ import annotations

import argparse
import logging
import time

from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run one QCar refactor vehicle actor")
    parser.add_argument("--vehicle-id", type=int, help="override the configured vehicle ID")
    parser.add_argument("--cycles", type=int, default=None, help="stop after this many control-loop cycles")
    parser.add_argument("--vehicle-config", help="vehicle profile file in config/")
    parser.add_argument("--headless", action="store_true", help="use config_vehicle_headless.yaml")
    parser.add_argument("--ground-station", choices=("null", "tcp_client"), help="ground-station bridge profile")
    parser.add_argument("--ground-station-host", help="override TCP ground-station host")
    parser.add_argument("--ground-station-port", type=int, help="override TCP ground-station listener port")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    value_overrides = {}
    selection_overrides = {}
    if args.ground_station is not None:
        selection_overrides["ground_station"] = args.ground_station
    if args.ground_station_host is not None or args.ground_station_port is not None:
        ground_station_values = {}
        if args.ground_station_host is not None:
            ground_station_values["server_host"] = args.ground_station_host
        if args.ground_station_port is not None:
            ground_station_values["server_port"] = args.ground_station_port
        value_overrides.setdefault("modules", {})["ground_station"] = ground_station_values
    vehicle_config_file = args.vehicle_config or (
        "config_vehicle_headless.yaml" if args.headless else "config_vehicle.yaml"
    )
    runtime = build_vehicle_process_runtime(
        VehicleProcessSpec(
            vehicle_id=args.vehicle_id,
            vehicle_config_file=vehicle_config_file,
            selection_overrides=selection_overrides or None,
            value_overrides=value_overrides,
        )
    )
    period_s = 1.0 / runtime.config.runtime["loop_rate_hz"]

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
