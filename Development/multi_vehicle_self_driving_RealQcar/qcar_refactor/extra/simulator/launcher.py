"""Launch every worker in one virtual or CARLA scenario."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from extra.simulator.carla.scenario import load_carla_setup
from extra.simulator.virtual.scenario import load_virtual_setup


_PROJECT_ROOT = Path(__file__).resolve().parents[2]


def main(argv=None) -> int:
    """Start scenario workers, synchronize their startup, and await completion."""
    parser = argparse.ArgumentParser(description="Launch every vehicle in a virtual or CARLA scenario")
    parser.add_argument("--platform", choices=("virtual", "carla"), required=True)
    parser.add_argument("--setup-file", required=True, help="scenario YAML file")
    parser.add_argument("--cycles", type=int, help="stop each vehicle after this many control-loop cycles")
    parser.add_argument("--ready-timeout-s", type=float, default=45.0, help="startup-barrier timeout")
    parser.add_argument("--build-fleet", action="store_true", help="send BUILD_FLEET after every worker starts")
    parser.add_argument("--dt", type=float, default=0.02, help="virtual fixed step in seconds")
    parser.add_argument("--realtime", action="store_true", help="pace virtual workers in wall-clock time")
    args = parser.parse_args(argv)

    if args.cycles is not None and args.cycles <= 0:
        parser.error("--cycles must be positive")
    if args.ready_timeout_s <= 0:
        parser.error("--ready-timeout-s must be positive")
    if args.platform != "virtual" and (args.realtime or args.dt != 0.02):
        parser.error("--dt and --realtime apply only to the virtual platform")

    setup_file = Path(args.setup_file).resolve()
    vehicle_ids = _vehicle_ids(args.platform, setup_file)
    commands = _worker_commands(args, setup_file, vehicle_ids)
    interrupted = _InterruptFlag()

    with tempfile.TemporaryDirectory(prefix="qcar-scenario-") as temporary:
        temporary_dir = Path(temporary)
        start_file = temporary_dir / "start.signal"
        stop_file = temporary_dir / "stop.signal"
        processes: list[subprocess.Popen] = []
        try:
            processes = [_start_worker(command) for command in commands(temporary_dir, start_file, stop_file)]
            if not _wait_until_ready(
                [temporary_dir / f"vehicle_{vehicle_id}.ready" for vehicle_id in vehicle_ids],
                processes,
                args.ready_timeout_s,
                interrupted,
            ):
                print("Ctrl+C received; stopping scenario workers...", file=sys.stderr)
                return 130
            start_file.touch()
            return _wait_for_workers(processes, interrupted)
        finally:
            interrupted.restore()
            _stop_remaining(processes, stop_file)


def _vehicle_ids(platform: str, setup_file: Path) -> tuple[int, ...]:
    setup = load_virtual_setup(setup_file) if platform == "virtual" else load_carla_setup(setup_file)
    return tuple(vehicle.vehicle_id for vehicle in setup.vehicles)


def _start_worker(command: list[str]) -> subprocess.Popen:
    """Start a worker in its own signal group for graceful Ctrl+C shutdown."""
    options = {
        "cwd": _PROJECT_ROOT,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.STDOUT,
        "text": True,
    }
    if os.name == "nt":
        options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        options["start_new_session"] = True
    return subprocess.Popen(command, **options)


def _worker_commands(args, setup_file: Path, vehicle_ids: tuple[int, ...]):
    runner_module = f"extra.simulator.{args.platform}.process_runner"

    def build(temporary_dir: Path, start_file: Path, stop_file: Path) -> list[list[str]]:
        commands = []
        for vehicle_id in vehicle_ids:
            command = [
                sys.executable,
                "-m",
                runner_module,
                "--setup-file",
                str(setup_file),
                "--vehicle-id",
                str(vehicle_id),
                "--ready-file",
                str(temporary_dir / f"vehicle_{vehicle_id}.ready"),
                "--start-file",
                str(start_file),
                "--stop-file",
                str(stop_file),
            ]
            if args.cycles is not None:
                command.extend(("--cycles", str(args.cycles)))
            if args.build_fleet:
                command.append("--build-fleet")
            if args.platform == "virtual":
                command.extend(("--dt", str(args.dt)))
                if args.realtime:
                    command.append("--realtime")
            commands.append(command)
        return commands

    return build


class _InterruptFlag:
    """Turn Ctrl+C into a non-blocking flag checked by the launcher wait loops."""

    def __init__(self) -> None:
        self.requested = False
        self._previous_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._handle)

    def _handle(self, _signum, _frame) -> None:
        self.requested = True

    def restore(self) -> None:
        signal.signal(signal.SIGINT, self._previous_handler)


def _wait_until_ready(
    paths: list[Path],
    processes: list[subprocess.Popen],
    timeout_s: float,
    interrupted: _InterruptFlag,
) -> bool:
    deadline = time.monotonic() + timeout_s
    while not all(path.exists() for path in paths):
        if interrupted.requested:
            return False
        exited = [process for process in processes if process.poll() is not None]
        if exited:
            details = "\n".join(_process_output(process) for process in exited)
            raise RuntimeError(f"Scenario worker exited before readiness:\n{details}")
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for scenario workers to become ready")
        time.sleep(0.05)
    return True


def _wait_for_workers(processes: list[subprocess.Popen], interrupted: _InterruptFlag) -> int:
    return_code = 0
    remaining = set(processes)
    while remaining:
        if interrupted.requested:
            print("Ctrl+C received; stopping scenario workers...", file=sys.stderr)
            return 130
        completed = [process for process in remaining if process.poll() is not None]
        for process in completed:
            output = _process_output(process)
            if process.returncode != 0:
                return_code = process.returncode or 1
                print(output, file=sys.stderr, end="" if output.endswith("\n") else "\n")
            else:
                _print_completion_summary(output)
            remaining.remove(process)
        if remaining:
            time.sleep(0.05)
    return return_code


def _process_output(process: subprocess.Popen) -> str:
    output, _ = process.communicate()
    return output or ""


def _print_completion_summary(output: str) -> None:
    """Print the final worker result without flooding the terminal with telemetry."""
    for line in reversed(output.splitlines()):
        try:
            result = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(result, dict) or not isinstance(result.get("rows"), list):
            continue
        vehicle_id = result.get("vehicle_id", "unknown")
        rows = result["rows"]
        final_time = rows[-1].get("time_s") if rows and isinstance(rows[-1], dict) else None
        suffix = f" at simulation time {final_time:.2f}s" if isinstance(final_time, (int, float)) else ""
        print(f"Vehicle {vehicle_id} completed {len(rows)} cycle(s){suffix}.")
        return
    print("A scenario worker completed without a telemetry summary.")


def _stop_remaining(processes: list[subprocess.Popen], stop_file: Path) -> None:
    active = [process for process in processes if process.poll() is None]
    if active:
        stop_file.touch()

    deadline = time.monotonic() + 5.0
    while active and time.monotonic() < deadline:
        active = [process for process in active if process.poll() is None]
        if active:
            time.sleep(0.05)

    for process in active:
        process.terminate()
    for process in active:
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


if __name__ == "__main__":
    raise SystemExit(main())
