"""Command-line interface for auditable Linux-vehicle deployment."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Sequence

from .bundle import BundleError, build_bundle, load_bundle_spec, verify_bundle
from .deployment_targets import (
    describe_deployment_targets,
    deploy_targets,
    load_deployment_targets,
    preflight_targets,
    start_targets,
    stop_targets,
)
from .remote import SSHDeploymentClient
from .configuration import load_deployment_target
from .deployment_type import DeploymentError, DeploymentTarget


def _print(value: object) -> None:
    print(json.dumps(value, indent=2, sort_keys=True, default=str))


class _TargetsDeployProgress:
    """Render phase/vehicle deployment progress without corrupting JSON stdout."""

    def update(self, label: str, completed: int, total: int) -> None:
        width = 24
        filled = 0 if total <= 0 else round(width * completed / total)
        bar = "#" * filled + "-" * (width - filled)
        line = f"[{bar}] {completed}/{total} {label}"
        # SSH password prompts cannot coordinate cursor movement with a
        # carriage-return bar, so use one stable line per deployment phase.
        print(line, file=sys.stderr, flush=True)

    def close(self) -> None:
        """Keep the progress callback usable in the deployment cleanup path."""


def _target_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--target-file",
        required=True,
        help="Path to a local, unversioned deployment target YAML file.",
    )


def _mutating_target_arguments(parser: argparse.ArgumentParser) -> None:
    _target_arguments(parser)
    parser.add_argument("--vehicle-id", type=int, required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Confirm this explicit mutating operation.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate target selection and print the action without connecting or changing the vehicle.",
    )


def _targets_arguments(parser: argparse.ArgumentParser, *, mutating: bool = False) -> None:
    parser.add_argument(
        "--targets-file",
        required=True,
        help="Local deployment-targets YAML file that lists one Linux vehicle endpoint per entry.",
    )
    if mutating:
        parser.add_argument(
            "--yes",
            action="store_true",
            help="Confirm this explicit mutating deployment-targets operation.",
        )
        parser.add_argument(
            "--dry-run",
            action="store_true",
            help="Print the reviewed target plan without connecting, packaging, uploading, or starting.",
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m extra.deployment",
        description="Create and deploy a verified source/config bundle to one Linux vehicle endpoint.",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    package = commands.add_parser("package", help="Build a deterministic allowlisted deployment bundle.")
    package.add_argument("--source", help="Source-root directory.")
    package.add_argument(
        "--include",
        action="append",
        help="File or directory relative to --source. Repeat for each selected item.",
    )
    package.add_argument(
        "--bundle-spec",
        help="Versioned YAML bundle selection. It supplies source_root and include.",
    )
    package.add_argument("--output", required=True, help="Output .tar.gz path.")

    verify = commands.add_parser("verify", help="Verify a local bundle manifest and hashes.")
    verify.add_argument("--bundle", required=True)

    preflight = commands.add_parser("preflight", help="Run non-mutating SSH target checks.")
    _target_arguments(preflight)

    deploy = commands.add_parser("deploy", help="Stage, verify, and atomically activate a bundle.")
    _mutating_target_arguments(deploy)
    deploy.add_argument("--bundle", required=True)

    start = commands.add_parser("start", help="Start the configured entry point from the active release.")
    _mutating_target_arguments(start)

    stop = commands.add_parser("stop", help="Run the configured stop command for the explicit target.")
    _mutating_target_arguments(stop)

    rollback = commands.add_parser(
        "rollback", help="Atomically activate one previously verified release."
    )
    _mutating_target_arguments(rollback)
    rollback.add_argument("--release", required=True, help="Release name returned by deploy.")

    logs = commands.add_parser("logs", help="Tail the configured active vehicle log.")
    _target_arguments(logs)
    logs.add_argument("--lines", type=int, default=100)

    artifacts = commands.add_parser("fetch-artifact", help="Retrieve one file or directory from the active release.")
    _target_arguments(artifacts)
    artifacts.add_argument("--artifact", required=True, help="Path relative to the active release.")
    artifacts.add_argument("--output-dir", required=True)

    targets_preflight = commands.add_parser(
        "targets-preflight",
        help="Run read-only preflight checks for every configured deployment target.",
    )
    targets_preflight.set_defaults(targets_operation="preflight")
    _targets_arguments(targets_preflight)

    targets_deploy = commands.add_parser(
        "targets-deploy",
        help="Package once and sequentially deploy the verified bundle to each configured target.",
    )
    targets_deploy.set_defaults(targets_operation="deploy")
    _targets_arguments(targets_deploy, mutating=True)

    targets_start = commands.add_parser(
        "targets-start",
        help="Start every configured active release after all targets pass preflight.",
    )
    targets_start.set_defaults(targets_operation="start")
    _targets_arguments(targets_start, mutating=True)

    targets_stop = commands.add_parser(
        "targets-stop",
        help="Request graceful shutdown from every configured physical-vehicle target.",
    )
    targets_stop.set_defaults(targets_operation="stop")
    _targets_arguments(targets_stop, mutating=True)
    return parser


def _require_explicit_target(arguments: argparse.Namespace, target: DeploymentTarget) -> None:
    if arguments.vehicle_id != target.vehicle_id:
        raise DeploymentError(
            f"--vehicle-id {arguments.vehicle_id} does not match target-file vehicle_id {target.vehicle_id}"
        )
    if arguments.host.strip().lower() != target.host.lower():
        raise DeploymentError(
            f"--host {arguments.host!r} does not match target-file host {target.host!r}"
        )
    if not arguments.yes:
        raise DeploymentError("Mutating deployment operations require --yes")


def _dry_run(command: str, target: DeploymentTarget, **extra: object) -> int:
    _print({"dry_run": True, "command": command, "host": target.host, "vehicle_id": target.vehicle_id, **extra})
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        if arguments.command == "package":
            if arguments.bundle_spec:
                if arguments.source or arguments.include:
                    raise BundleError("--bundle-spec cannot be combined with --source or --include")
                source, includes = load_bundle_spec(arguments.bundle_spec)
            else:
                if not arguments.source or not arguments.include:
                    raise BundleError("package requires --bundle-spec or both --source and --include")
                source, includes = arguments.source, arguments.include
            _print(asdict(build_bundle(source, includes, arguments.output)))
            return 0
        if arguments.command == "verify":
            _print(asdict(verify_bundle(arguments.bundle)))
            return 0

        if getattr(arguments, "targets_operation", None) is not None:
            deployment_targets = load_deployment_targets(arguments.targets_file)
            operation = arguments.targets_operation
            if operation == "preflight":
                reports = preflight_targets(deployment_targets)
                _print(
                    {
                        "deployment_targets": describe_deployment_targets(deployment_targets),
                        "preflight": reports,
                    }
                )
                return 0 if all(item["report"]["ok"] for item in reports) else 2
            if not arguments.yes:
                raise DeploymentError("Mutating deployment-targets operations require --yes")
            if arguments.dry_run:
                _print(
                    {
                        "dry_run": True,
                        "command": f"targets-{operation}",
                        "deployment_targets": describe_deployment_targets(deployment_targets),
                    }
                )
                return 0
            if operation == "deploy":
                progress = _TargetsDeployProgress()
                try:
                    result = deploy_targets(deployment_targets, progress=progress.update)
                finally:
                    progress.close()
                _print(result)
                return 0
            report = (
                start_targets(deployment_targets)
                if operation == "start"
                else stop_targets(deployment_targets)
            )
            _print({"launch": asdict(report)})
            return 0 if report.ok else 2

        target = load_deployment_target(arguments.target_file)
        if arguments.command == "preflight":
            with SSHDeploymentClient(target) as client:
                report = client.preflight()
            _print(asdict(report))
            return 0 if report.ok else 2

        if arguments.command in {"deploy", "start", "stop", "rollback"}:
            _require_explicit_target(arguments, target)
            if arguments.dry_run:
                extra = {}
                if arguments.command == "deploy":
                    extra["bundle"] = str(Path(arguments.bundle).resolve())
                elif arguments.command == "rollback":
                    extra["release"] = arguments.release
                return _dry_run(arguments.command, target, **extra)

            with SSHDeploymentClient(target) as client:
                if arguments.command == "deploy":
                    _print({"release": client.deploy_bundle(arguments.bundle)})
                elif arguments.command == "start":
                    _print(asdict(client.start()))
                elif arguments.command == "stop":
                    _print(asdict(client.stop()))
                else:
                    _print({"release": client.rollback(arguments.release)})
            return 0

        with SSHDeploymentClient(target) as client:
            if arguments.command == "logs":
                _print(asdict(client.tail_logs(arguments.lines)))
            elif arguments.command == "fetch-artifact":
                _print(
                    {
                        "artifact": str(
                            client.fetch_artifact(arguments.artifact, arguments.output_dir)
                        )
                    }
                )
        return 0
    except (BundleError, DeploymentError) as error:
        print(f"error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
