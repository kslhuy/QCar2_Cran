"""Config-driven deployment actions for multiple Linux vehicle endpoints.

This is deployment orchestration only.  It does not create a runtime
``utils.fleet.FleetManager`` or participate in vehicle-to-vehicle control.
"""

from __future__ import annotations

from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable

import yaml

from .bundle import build_bundle, load_bundle_spec, verify_bundle
from .remote import SSHDeploymentClient
from .configuration import load_deployment_target
from .deployment_type import (
    BundleSummary,
    DeploymentError,
    DeploymentTarget,
    DeploymentTargetEntry,
    DeploymentTargets,
    PreflightReport,
)
from extra.platform.launcher import LaunchReport, PlatformLauncher, RemoteProcessEndpoint


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise DeploymentError(f"Deployment-targets field {name!r} must be a mapping")
    return value


def _required_string(values: dict[str, Any], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value.strip():
        raise DeploymentError(f"Deployment-targets requires a non-empty {name!r}")
    return value.strip()


def _reject_unknown(values: dict[str, Any], allowed: set[str], name: str) -> None:
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise DeploymentError(
            f"Deployment-targets field {name!r} has unsupported fields: {', '.join(unknown)}"
        )


def _resolve_config_path(parent: Path, value: str) -> Path:
    candidate = Path(value).expanduser()
    return (candidate if candidate.is_absolute() else parent / candidate).resolve()


def load_deployment_targets(path: str | Path) -> DeploymentTargets:
    """Read a local deployment-targets file and its separate target files."""

    config_path = Path(path).expanduser().resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except OSError as error:
        raise DeploymentError(f"Cannot read deployment-targets file {config_path}: {error}") from error
    except yaml.YAMLError as error:
        raise DeploymentError(f"Invalid YAML in deployment-targets file {config_path}: {error}") from error

    if not isinstance(raw, dict):
        raise DeploymentError("Deployment-targets file must contain a mapping")
    _reject_unknown(raw, {"deployment_targets"}, "root")
    targets_config = _mapping(raw.get("deployment_targets"), "deployment_targets")
    _reject_unknown(targets_config, {"bundle", "targets"}, "deployment_targets")
    bundle_values = _mapping(targets_config.get("bundle"), "deployment_targets.bundle")
    _reject_unknown(bundle_values, {"specification", "output"}, "deployment_targets.bundle")
    bundle_spec = _resolve_config_path(
        config_path.parent, _required_string(bundle_values, "specification")
    )
    bundle_output = _resolve_config_path(
        config_path.parent, _required_string(bundle_values, "output")
    )
    raw_targets = targets_config.get("targets")
    if not isinstance(raw_targets, list) or not raw_targets:
        raise DeploymentError("Deployment-targets requires a non-empty 'targets' list")

    entries: list[DeploymentTargetEntry] = []
    target_paths: set[Path] = set()
    vehicle_ids: set[int] = set()
    hosts: set[str] = set()
    for index, item in enumerate(raw_targets):
        values = _mapping(item, f"targets[{index}]")
        _reject_unknown(values, {"file"}, f"targets[{index}]")
        path_value = values.get("file")
        if not isinstance(path_value, str) or not path_value.strip():
            raise DeploymentError(f"Deployment-targets targets[{index}] requires a non-empty 'file'")
        target_path = _resolve_config_path(config_path.parent, path_value.strip())
        if target_path in target_paths:
            raise DeploymentError(f"Deployment-targets repeats target file {target_path}")
        target = load_deployment_target(target_path)
        if target.vehicle_id in vehicle_ids:
            raise DeploymentError(f"Deployment-targets repeats vehicle_id {target.vehicle_id}")
        host_key = target.host.lower()
        if host_key in hosts:
            raise DeploymentError(f"Deployment-targets repeats host {target.host!r}")
        target_paths.add(target_path)
        vehicle_ids.add(target.vehicle_id)
        hosts.add(host_key)
        entries.append(DeploymentTargetEntry(path=target_path, target=target))

    return DeploymentTargets(
        path=config_path,
        bundle_spec=bundle_spec,
        bundle_output=bundle_output,
        targets=tuple(entries),
    )


def describe_deployment_targets(targets: DeploymentTargets) -> dict[str, object]:
    """Return only the reviewable, non-secret parts of a deployment target plan."""

    return {
        "targets_file": str(targets.path),
        "bundle": {
            "specification": str(targets.bundle_spec),
            "output": str(targets.bundle_output),
        },
        "targets": [
            {
                "file": str(entry.path),
                "vehicle_id": entry.target.vehicle_id,
                "host": entry.target.host,
                "expected_hostname": entry.target.expected_hostname,
            }
            for entry in targets.targets
        ],
    }


ClientFactory = Callable[[DeploymentTarget], SSHDeploymentClient]
ProgressCallback = Callable[[str, int, int], None]

def preflight_targets(
    targets: DeploymentTargets, *, client_factory: ClientFactory = SSHDeploymentClient
) -> list[dict[str, object]]:
    """Check every vehicle before any deployment-target mutation occurs."""

    reports: list[dict[str, object]] = []
    with ExitStack() as stack:
        for entry in targets.targets:
            client = stack.enter_context(client_factory(entry.target))
            report: PreflightReport = client.preflight()
            reports.append(
                {
                    "vehicle_id": entry.target.vehicle_id,
                    "host": entry.target.host,
                    "report": asdict(report),
                }
            )
    return reports


def deploy_targets(
    targets: DeploymentTargets,
    *,
    client_factory: ClientFactory = SSHDeploymentClient,
    progress: ProgressCallback | None = None,
) -> dict[str, object]:
    """Package once, preflight all endpoints, then activate the same bundle sequentially.

    Activation is atomic per vehicle.  The command intentionally does not start
    any vehicle process, so the operator can inspect the deployment result first.
    """

    total_steps = 1 + 2 * len(targets.targets)
    completed_steps = 0

    def update(label: str) -> None:
        if progress is not None:
            progress(label, completed_steps, total_steps)

    update("Packaging selected bundle")
    source_root, includes = load_bundle_spec(targets.bundle_spec)
    bundle: BundleSummary = build_bundle(source_root, includes, targets.bundle_output)
    verify_bundle(bundle.path)
    completed_steps += 1
    update("Bundle verified")

    reports: list[dict[str, object]] = []
    releases: list[dict[str, object]] = []
    with ExitStack() as stack:
        clients: list[tuple[DeploymentTargetEntry, SSHDeploymentClient]] = []
        for entry in targets.targets:
            clients.append((entry, stack.enter_context(client_factory(entry.target))))
        for entry, client in clients:
            report = client.preflight()
            reports.append(
                {
                    "vehicle_id": entry.target.vehicle_id,
                    "host": entry.target.host,
                    "report": asdict(report),
                }
            )
            completed_steps += 1
            update(f"Preflight vehicle {entry.target.vehicle_id}")
        failures = [item for item in reports if not item["report"]["ok"]]
        if failures:
            names = ", ".join(
                f"vehicle {item['vehicle_id']} ({item['host']})" for item in failures
            )
            raise DeploymentError(f"Deployment-target preflight failed; no bundle was uploaded: {names}")
        for entry, client in clients:
            releases.append(
                {
                    "vehicle_id": entry.target.vehicle_id,
                    "host": entry.target.host,
                    "release": client.deploy_bundle(bundle.path),
                }
            )
            completed_steps += 1
            update(f"Deployed vehicle {entry.target.vehicle_id}")
    return {"bundle": asdict(bundle), "preflight": reports, "releases": releases}


def start_targets(
    targets: DeploymentTargets, *, client_factory: ClientFactory = SSHDeploymentClient
) -> LaunchReport:
    """Start all configured active releases after every target passes preflight."""

    targets = [
        RemoteProcessEndpoint(
            vehicle_id=entry.target.vehicle_id,
            endpoint=entry.target.host,
            configuration=entry.target,
        )
        for entry in targets.targets
    ]
    return PlatformLauncher(client_factory=client_factory).launch(targets, operation="start")


def stop_targets(
    targets: DeploymentTargets, *, client_factory: ClientFactory = SSHDeploymentClient
) -> LaunchReport:
    """Request graceful shutdown for every configured Linux vehicle endpoint."""

    targets = [
        RemoteProcessEndpoint(
            vehicle_id=entry.target.vehicle_id,
            endpoint=entry.target.host,
            configuration=entry.target,
        )
        for entry in targets.targets
    ]
    return PlatformLauncher(client_factory=client_factory).launch(targets, operation="stop")
