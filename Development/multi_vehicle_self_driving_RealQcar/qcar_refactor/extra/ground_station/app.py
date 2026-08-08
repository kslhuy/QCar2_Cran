"""Canonical entry point for the operator-side ground-station program."""

from __future__ import annotations

import argparse
from typing import Sequence

from .core import listener
from .presentation import terminal


def main(argv: Sequence[str] | None = None) -> int:
    """Dispatch a terminal or headless listener without duplicating settings."""

    parser = argparse.ArgumentParser(
        prog="python -m extra.ground_station",
        description="Run the operator-side ground-station program.",
    )
    commands = parser.add_subparsers(dest="program", required=True)
    commands.add_parser(
        "terminal", add_help=False, help="Run the dashboard and typed command terminal."
    )
    commands.add_parser("server", add_help=False, help="Run only the vehicle TCP listener.")
    arguments, remaining = parser.parse_known_args(argv)
    if arguments.program == "terminal":
        return terminal.main(list(remaining))
    return listener.main(list(remaining))
