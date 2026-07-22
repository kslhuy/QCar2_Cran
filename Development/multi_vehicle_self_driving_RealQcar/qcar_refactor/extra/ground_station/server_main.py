"""Run the ground-station TCP listener without an operator terminal UI."""

from __future__ import annotations

import argparse
import threading
import time

from .server import GroundStationServer


def main(argv: list[str] | None = None) -> int:
    """Start the vehicle listener as a standalone, headless process."""
    parser = argparse.ArgumentParser(description="Run the QCar ground-station vehicle listener")
    parser.add_argument("--host", default="0.0.0.0", help="TCP listener address")
    parser.add_argument("--port", type=int, default=5000, help="TCP listener port")
    parser.add_argument("--duration-s", type=float, default=None, help="stop automatically after this duration")
    args = parser.parse_args(argv)
    if args.duration_s is not None and args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")

    server = GroundStationServer(args.host, args.port)
    stop_requested = threading.Event()
    try:
        server.start()
        print(f"Ground-station vehicle listener running on {args.host}:{server.port}")
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
