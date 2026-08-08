"""Shared local and remote launcher backends for platform process runners.

The backends coordinate readiness and process lifetime only.  They do not
construct platform devices or send runtime commands; those remain owned by a
platform resource context and the vehicle-side ground-station path.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Sequence

from extra.platform.carla.scenario import load_carla_setup
from extra.platform.virtual.scenario import load_virtual_setup


_PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class RemoteProcessEndpoint:
    """Ephemeral remote process endpoint, not a persistent deployment target."""

    vehicle_id: int
    endpoint: str
    configuration: Any


@dataclass(frozen=True)
class LocalProcessEndpoint:
    """Ephemeral local process endpoint selected by a scenario launcher."""

    vehicle_id: int
    endpoint: str
    command: tuple[str, ...]
    ready_file: Path


@dataclass(frozen=True)
class LaunchEndpointResult:
    """One backend-independent lifecycle result for a vehicle process."""

    vehicle_id: int
    endpoint: str
    ready: bool
    healthy: bool | None
    logs: str
    shutdown: str
    failure: str | None = None
    exit_status: int | None = None
    preflight: dict[str, object] | None = None


@dataclass(frozen=True)
class LaunchReport:
    """Results from one local or remote launcher operation."""

    backend: str
    endpoints: tuple[LaunchEndpointResult, ...]
    operation: str = "run"
    interrupted: bool = False

    @property
    def ok(self) -> bool:
        if self.interrupted:
            return False
        if self.operation == "stop":
            return all(
                result.healthy is True
                and result.shutdown == "graceful"
                and result.failure is None
                for result in self.endpoints
            )
        return all(
            result.ready and result.healthy is True and result.failure is None
            for result in self.endpoints
        )

    @property
    def exit_code(self) -> int:
        if self.interrupted:
            return 130
        for result in self.endpoints:
            if result.exit_status not in (None, 0):
                return result.exit_status
        return 0 if self.ok else 1


@dataclass
class _LocalLaunchState:
    target: LocalProcessEndpoint
    process: subprocess.Popen | None = None
    ready: bool = False
    healthy: bool | None = None
    logs: str = ""
    shutdown: str = "not_requested"
    failure: str | None = None
    exit_status: int | None = None


class _LocalLaunchBackend:
    """Local subprocess transport used internally by :class:`PlatformLauncher`."""

    def __init__(self, working_directory: str | Path) -> None:
        self._working_directory = Path(working_directory)

    def _start(self, command: Sequence[str]) -> subprocess.Popen:
        options: dict[str, Any] = {
            "cwd": self._working_directory,
            "stdout": subprocess.PIPE,
            "stderr": subprocess.STDOUT,
            "text": True,
        }
        if os.name == "nt":
            options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            options["start_new_session"] = True
        return subprocess.Popen(list(command), **options)

    def launch(
        self,
        targets: Sequence[LocalProcessEndpoint],
        *,
        start_file: Path,
        stop_file: Path,
        ready_timeout_s: float,
        stop_requested: Callable[[], bool],
    ) -> LaunchReport:
        """Run local workers and return the same lifecycle report as remote endpoints."""

        states = [_LocalLaunchState(target) for target in targets]
        interrupted = False
        lifecycle_error: str | None = None
        try:
            for state in states:
                try:
                    state.process = self._start(state.target.command)
                except Exception as error:
                    lifecycle_error = f"Unable to start local worker: {error}"
                    state.failure = lifecycle_error
                    state.shutdown = "not_started"
                    break
            if lifecycle_error is not None:
                for state in states:
                    if state.failure is None and state.process is None:
                        state.failure = "Not started because another local worker failed to start"
                        state.shutdown = "not_started"
            else:
                deadline = time.monotonic() + ready_timeout_s
                while not all(state.target.ready_file.exists() for state in states):
                    if stop_requested():
                        interrupted = True
                        lifecycle_error = "Launch interrupted before readiness"
                        break
                    exited = [state for state in states if state.process is not None and state.process.poll() is not None]
                    if exited:
                        vehicle_ids = ", ".join(str(state.target.vehicle_id) for state in exited)
                        lifecycle_error = f"Local worker exited before readiness: vehicle {vehicle_ids}"
                        break
                    if time.monotonic() >= deadline:
                        lifecycle_error = "Timed out waiting for local platform workers to become ready"
                        break
                    time.sleep(0.05)
                if lifecycle_error is None:
                    for state in states:
                        state.ready = True
                    start_file.touch()
                    while any(state.process is not None and state.process.poll() is None for state in states):
                        if stop_requested():
                            interrupted = True
                            lifecycle_error = "Launch interrupted by operator"
                            break
                        time.sleep(0.05)
                if lifecycle_error is not None:
                    for state in states:
                        if state.process is not None and state.failure is None:
                            state.failure = lifecycle_error
        finally:
            processes = [state.process for state in states if state.process is not None]
            shutdowns = self._stop_remaining(processes, stop_file)
            for state in states:
                process = state.process
                if process is None:
                    state.healthy = False
                    continue
                state.exit_status = process.poll()
                state.logs = _process_output(process)
                if id(process) in shutdowns:
                    state.shutdown = shutdowns[id(process)]
                elif state.shutdown == "not_requested":
                    state.shutdown = "completed" if state.exit_status == 0 else "failed"
                if state.failure is None and state.exit_status != 0:
                    state.failure = f"Local worker exited with status {state.exit_status}"
                state.healthy = state.failure is None and state.exit_status == 0
        return LaunchReport(
            backend="local",
            endpoints=tuple(
                LaunchEndpointResult(
                    vehicle_id=state.target.vehicle_id,
                    endpoint=state.target.endpoint,
                    ready=state.ready,
                    healthy=state.healthy,
                    logs=state.logs,
                    shutdown=state.shutdown,
                    failure=state.failure,
                    exit_status=state.exit_status,
                )
                for state in states
            ),
            operation="run",
            interrupted=interrupted,
        )

    @staticmethod
    def _stop_remaining(processes: Sequence[subprocess.Popen], stop_file: Path) -> dict[int, str]:
        """Request graceful stop, then terminate only non-responsive local workers."""

        active = [process for process in processes if process.poll() is None]
        requested = list(active)
        shutdowns: dict[int, str] = {}
        if active:
            stop_file.touch()
        deadline = time.monotonic() + 5.0
        while active and time.monotonic() < deadline:
            active = [process for process in active if process.poll() is None]
            if active:
                time.sleep(0.05)
        graceful = [process for process in requested if process.poll() is not None]
        for process in graceful:
            shutdowns[id(process)] = "graceful"
        for process in active:
            process.terminate()
            shutdowns[id(process)] = "terminated"
        for process in active:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
                shutdowns[id(process)] = "killed"
        return shutdowns


class _RemoteLaunchBackend:
    """Authenticated remote transport used internally by :class:`PlatformLauncher`.

    ``client_factory`` is supplied by an integration such as deployment, which
    keeps SSH and release-management dependencies out of this platform module.
    """

    def __init__(self, client_factory: Callable[[Any], Any]) -> None:
        self._client_factory = client_factory

    def start(self, targets: Sequence[RemoteProcessEndpoint]) -> LaunchReport:
        """Preflight every target before any remote process start is attempted."""

        reports: dict[int, dict[str, object]] = {}
        results: list[LaunchEndpointResult] = []
        with ExitStack() as stack:
            clients: list[tuple[RemoteProcessEndpoint, Any]] = []
            for target in targets:
                clients.append((target, stack.enter_context(self._client_factory(target.configuration))))
            for target, client in clients:
                report = client.preflight()
                reports[target.vehicle_id] = asdict(report)
            failures = [target for target in targets if not reports[target.vehicle_id]["ok"]]
            if failures:
                for target in targets:
                    report = reports[target.vehicle_id]
                    failed = not report["ok"]
                    results.append(
                        LaunchEndpointResult(
                            vehicle_id=target.vehicle_id,
                            endpoint=target.endpoint,
                            ready=False,
                            healthy=False if failed else None,
                            logs="",
                            shutdown="not_requested",
                            failure=(
                                "Remote preflight failed"
                                if failed
                                else "Not started because another remote endpoint failed preflight"
                            ),
                            preflight=report,
                        )
                    )
                return LaunchReport(backend="remote", endpoints=tuple(results), operation="start")
            blocked_by: int | None = None
            for target, client in clients:
                report = reports[target.vehicle_id]
                if blocked_by is not None:
                    results.append(
                        LaunchEndpointResult(
                            vehicle_id=target.vehicle_id,
                            endpoint=target.endpoint,
                            ready=False,
                            healthy=None,
                            logs="",
                            shutdown="not_requested",
                            failure=f"Not started because vehicle {blocked_by} failed to start",
                            preflight=report,
                        )
                    )
                    continue
                try:
                    started = client.start()
                    logs = self._collect_logs(client)
                    results.append(
                        LaunchEndpointResult(
                            vehicle_id=target.vehicle_id,
                            endpoint=target.endpoint,
                            ready=started.exit_status == 0,
                            healthy=started.exit_status == 0,
                            logs=logs,
                            shutdown="not_requested",
                            failure=None if started.exit_status == 0 else "Remote start command failed",
                            exit_status=started.exit_status,
                            preflight=report,
                        )
                    )
                    if started.exit_status != 0:
                        blocked_by = target.vehicle_id
                except Exception as error:
                    blocked_by = target.vehicle_id
                    results.append(
                        LaunchEndpointResult(
                            vehicle_id=target.vehicle_id,
                            endpoint=target.endpoint,
                            ready=False,
                            healthy=False,
                            logs=self._collect_logs(client),
                            shutdown="not_requested",
                            failure=f"Remote start failed: {error}",
                            preflight=report,
                        )
                    )
        return LaunchReport(backend="remote", endpoints=tuple(results), operation="start")

    def stop(self, targets: Sequence[RemoteProcessEndpoint]) -> LaunchReport:
        """Stop every remote endpoint and report each shutdown outcome."""

        results: list[LaunchEndpointResult] = []
        with ExitStack() as stack:
            clients = [
                (target, stack.enter_context(self._client_factory(target.configuration)))
                for target in targets
            ]
            for target, client in clients:
                try:
                    stopped = client.stop()
                    results.append(
                        LaunchEndpointResult(
                            vehicle_id=target.vehicle_id,
                            endpoint=target.endpoint,
                            ready=False,
                            healthy=stopped.exit_status == 0,
                            logs=self._collect_logs(client),
                            shutdown="graceful" if stopped.exit_status == 0 else "failed",
                            failure=None if stopped.exit_status == 0 else "Remote stop command failed",
                            exit_status=stopped.exit_status,
                        )
                    )
                except Exception as error:
                    results.append(
                        LaunchEndpointResult(
                            vehicle_id=target.vehicle_id,
                            endpoint=target.endpoint,
                            ready=False,
                            healthy=False,
                            logs=self._collect_logs(client),
                            shutdown="failed",
                            failure=f"Remote stop failed: {error}",
                        )
                    )
        return LaunchReport(backend="remote", endpoints=tuple(results), operation="stop")

    @staticmethod
    def _collect_logs(client: Any) -> str:
        tail_logs = getattr(client, "tail_logs", None)
        if not callable(tail_logs):
            return ""
        try:
            result = tail_logs(100)
        except Exception as error:
            return f"Unable to collect logs: {error}"
        return "\n".join(part for part in (result.stdout, result.stderr) if part)


class PlatformLauncher:
    """One lifecycle interface that selects local or remote launch transport.

    Ground-station and deployment callers use this class rather than exposing a
    local-versus-remote choice to operators. The selected targets determine the
    transport; vehicle runtime commands remain separate from this service
    lifecycle interface.
    """

    def __init__(
        self,
        *,
        working_directory: str | Path | None = None,
        client_factory: Callable[[Any], Any] | None = None,
    ) -> None:
        self._working_directory = None if working_directory is None else Path(working_directory)
        self._client_factory = client_factory

    def launch(
        self,
        targets: Sequence[LocalProcessEndpoint | RemoteProcessEndpoint],
        *,
        operation: str,
        start_file: Path | None = None,
        stop_file: Path | None = None,
        ready_timeout_s: float | None = None,
        stop_requested: Callable[[], bool] | None = None,
    ) -> LaunchReport:
        """Run, start, or stop one homogeneous set of configured endpoints."""

        if not targets:
            raise ValueError("Platform launcher requires at least one target")
        local_targets = all(isinstance(target, LocalProcessEndpoint) for target in targets)
        remote_targets = all(isinstance(target, RemoteProcessEndpoint) for target in targets)
        if not (local_targets or remote_targets):
            raise ValueError("Platform launcher cannot mix local and remote targets in one operation")
        if local_targets:
            if operation != "run":
                raise ValueError("Local platform targets support only the 'run' operation")
            if self._working_directory is None:
                raise ValueError("Local platform launch requires a working_directory")
            if start_file is None or stop_file is None or ready_timeout_s is None:
                raise ValueError("Local platform launch requires start/stop files and a readiness timeout")
            return _LocalLaunchBackend(self._working_directory).launch(
                targets,
                start_file=start_file,
                stop_file=stop_file,
                ready_timeout_s=ready_timeout_s,
                stop_requested=stop_requested or (lambda: False),
            )
        if self._client_factory is None:
            raise ValueError("Remote platform launch requires a client_factory")
        backend = _RemoteLaunchBackend(self._client_factory)
        if operation == "start":
            return backend.start(targets)
        if operation == "stop":
            return backend.stop(targets)
        raise ValueError("Remote platform targets support only the 'start' and 'stop' operations")


def main(argv=None) -> int:
    """Launch every local virtual or CARLA worker from one platform scenario."""

    parser = argparse.ArgumentParser(description="Launch every vehicle in a platform scenario")
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
    launcher = PlatformLauncher(working_directory=_PROJECT_ROOT)

    with tempfile.TemporaryDirectory(prefix="qcar-scenario-") as temporary:
        temporary_dir = Path(temporary)
        start_file = temporary_dir / "start.signal"
        stop_file = temporary_dir / "stop.signal"
        try:
            worker_commands = commands(temporary_dir, start_file, stop_file)
            targets = [
                LocalProcessEndpoint(
                    vehicle_id=vehicle_id,
                    endpoint=f"local:{vehicle_id}",
                    command=tuple(command),
                    ready_file=temporary_dir / f"vehicle_{vehicle_id}.ready",
                )
                for vehicle_id, command in zip(vehicle_ids, worker_commands, strict=True)
            ]
            report = launcher.launch(
                targets,
                operation="run",
                start_file=start_file,
                stop_file=stop_file,
                ready_timeout_s=args.ready_timeout_s,
                stop_requested=lambda: interrupted.requested,
            )
        finally:
            interrupted.restore()
    _print_launch_report(report)
    return report.exit_code


def _vehicle_ids(platform: str, setup_file: Path) -> tuple[int, ...]:
    setup = load_virtual_setup(setup_file) if platform == "virtual" else load_carla_setup(setup_file)
    return tuple(vehicle.vehicle_id for vehicle in setup.vehicles)


def _worker_commands(args, setup_file: Path, vehicle_ids: tuple[int, ...]):
    runner_module = f"extra.platform.{args.platform}.process_runner"

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
    """Turn Ctrl+C into a non-blocking flag checked by launcher wait loops."""

    def __init__(self) -> None:
        self.requested = False
        self._previous_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._handle)

    def _handle(self, _signum, _frame) -> None:
        self.requested = True

    def restore(self) -> None:
        signal.signal(signal.SIGINT, self._previous_handler)


def _print_launch_report(report: LaunchReport) -> None:
    """Present the common result contract without flooding the local terminal."""

    if report.interrupted:
        print("Ctrl+C received; stopping scenario workers...", file=sys.stderr)
    for result in report.endpoints:
        if result.failure is not None:
            print(
                f"Vehicle {result.vehicle_id} ({result.endpoint}): {result.failure}",
                file=sys.stderr,
            )
            if result.logs:
                print(result.logs, file=sys.stderr, end="" if result.logs.endswith("\n") else "\n")
        elif result.logs:
            _print_completion_summary(result.logs)


def _process_output(process) -> str:
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
    print("A platform worker completed without a telemetry summary.")


if __name__ == "__main__":
    raise SystemExit(main())
