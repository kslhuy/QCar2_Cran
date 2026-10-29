"""Portable entry points: python -m qcar COMMAND [arguments]."""
from __future__ import annotations

import argparse
import runpy
import sys

COMMANDS = {
    "vehicle": "qcar.vehicle_main",
    "fake": "qcar.simulation.fake_vehicle_real_logic",
    "pipeline": "qcar.simulation.observer_pipeline",
    "batch": "qcar.simulation.mean_attack_batch",
    "gui": "qcar.GUI.app_main",
    "workflow": "qcar.Observer.KalmaNet.Robust.workflow",
    "doctor": "qcar.portability",
}


def main(argv=None):
    """Dispatch without changing the working directory or Python import path."""
    args = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=COMMANDS)
    if not args or args[0] in {"-h", "--help"}:
        parser.parse_args(args)
        return 0
    command = parser.parse_args(args[:1]).command
    original = sys.argv
    try:
        sys.argv = [f"qcar {command}", *args[1:]]
        runpy.run_module(COMMANDS[command], run_name="__main__", alter_sys=True)
    finally:
        sys.argv = original
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
