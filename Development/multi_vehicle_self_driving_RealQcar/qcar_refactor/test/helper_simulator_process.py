"""Shared subprocess harness for multi-vehicle simulator integration tests."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any


def run_vehicle_workers(
    test_case: Any,
    *,
    project_root: Path,
    runner_module: str,
    setup_file: Path,
    cycles: int,
    extra_args: list[str] | None = None,
    timeout_s: float = 75.0,
    require_v2v_trace: bool = True,
    vehicle_ids: tuple[int, ...] = (1, 2),
) -> dict[int, dict[str, Any]]:
    """Run any ordered set of scenario-selected workers and return JSON results."""
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        start_file = directory / "start.signal"
        processes = []
        try:
            for vehicle_id in vehicle_ids:
                command = [
                    sys.executable,
                    "-m", runner_module,
                    "--setup-file", str(setup_file),
                    "--vehicle-id", str(vehicle_id),
                    "--cycles",     str(cycles),
                    "--ready-file", str(directory / f"vehicle_{vehicle_id}.ready"),
                    "--start-file", str(start_file),
                ]
                if extra_args is not None:
                    command.extend(extra_args)
                processes.append(
                    subprocess.Popen(
                        command,
                        cwd=project_root,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                    )
                )

            _wait_for_ready_files(
                [directory / f"vehicle_{vehicle_id}.ready" for vehicle_id in vehicle_ids], processes, timeout_s
            )
            start_file.touch()
            results = {}
            for process in processes:
                output, _ = process.communicate(timeout=timeout_s)
                test_case.assertEqual(process.returncode, 0, output)
                result = _result_from_output(test_case, output, require_v2v_trace)
                results[result["vehicle_id"]] = result
            return results
        finally:
            for process in processes:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=10)


def _wait_for_ready_files(paths: list[Path], processes: list[subprocess.Popen[str]], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while not all(path.exists() for path in paths):
        exited = [process for process in processes if process.poll() is not None]
        if exited:
            messages = []
            for process in exited:
                output, _ = process.communicate()
                messages.append(f"worker exited with {process.returncode}:\n{output}")
            raise AssertionError("Simulator worker exited before readiness:\n" + "\n".join(messages))
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for simulator workers")
        time.sleep(0.05)


def _result_from_output(test_case: Any, output: str, require_v2v_trace: bool) -> dict[str, Any]:
    for line in reversed(output.splitlines()):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        required_keys = {"vehicle_id", "rows"}
        if require_v2v_trace:
            required_keys.add("v2v_trace")
        if isinstance(value, dict) and required_keys <= value.keys():
            return value
    test_case.fail(f"Simulator worker did not emit expected telemetry:\n{output}")
    raise AssertionError("unreachable")


# Existing non-fleet tests still import this older name.
run_two_vehicle_workers = run_vehicle_workers
