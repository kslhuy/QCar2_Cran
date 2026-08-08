"""Run the ground-station TCP listener without an operator terminal UI."""

from __future__ import annotations

import argparse
import threading
import time

from .server import GroundStationServer
from ..configuration import load_ground_station_configuration
from ..ground_station_type import GroundStationConfigurationError
from ..utils.logging import get_ground_station_logger


def main(argv: list[str] | None = None) -> int:
    """Start the vehicle listener as a standalone, headless process."""
    parser = argparse.ArgumentParser(description="Run the headless ground-station vehicle listener")
    parser.add_argument("--host", help="Temporary TCP listener address override")
    parser.add_argument("--port", type=int, help="Temporary TCP listener port override")
    parser.add_argument("--duration-s", type=float, default=None, help="stop automatically after this duration")
    args = parser.parse_args(argv)
    try:
        configuration = load_ground_station_configuration().with_overrides(
            host=args.host,
            port=args.port,
        )
    except GroundStationConfigurationError as error:
        parser.error(str(error))
    if args.duration_s is not None and args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")

    server = GroundStationServer(
        configuration.listener_host,
        configuration.listener_port,
        max_frame_bytes=configuration.max_frame_bytes,
        logger=get_ground_station_logger("listener"),
    )
    stop_requested = threading.Event()
    try:
        server.start()
        print(
            "Ground-station vehicle listener running on "
            f"{configuration.listener_host}:{server.port}"
        )
        started_at = time.monotonic()
        while not stop_requested.wait(0.2):
            if args.duration_s is not None and time.monotonic() - started_at >= args.duration_s:
                break
    except KeyboardInterrupt:
        return 0
    finally:
        stop_requested.set()
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
