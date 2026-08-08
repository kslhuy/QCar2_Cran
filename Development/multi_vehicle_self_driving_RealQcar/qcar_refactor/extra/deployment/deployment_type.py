"""Typed contracts shared by the Linux-vehicle deployment side program."""

from __future__ import annotations

from dataclasses import dataclass
from getpass import getpass
import os
from pathlib import Path

__all__ = [
    "BundleSummary",
    "CommandResult",
    "DeploymentError",
    "DeploymentTarget",
    "DeploymentTargetEntry",
    "DeploymentTargets",
    "PreflightReport",
]


class DeploymentError(RuntimeError):
    """Raised when a deployment request is invalid or unsafe."""


@dataclass(frozen=True)
class BundleSummary:
    """Verified identity and contents of one deployment archive."""

    path: Path
    manifest_sha256: str
    file_count: int
    total_bytes: int


@dataclass(frozen=True)
class DeploymentTarget:
    """SSH, release, and process settings for one Linux vehicle endpoint."""

    vehicle_id: int
    host: str
    username: str
    remote_root: str
    known_hosts: Path
    port: int = 22
    identity_file: Path | None = None
    password_env: str | None = None
    password_prompt: bool = False
    expected_hostname: str | None = None
    connect_timeout_s: float = 10.0
    command_timeout_s: float = 30.0
    start_command: str | None = None
    stop_command: str | None = None
    log_path_template: str = "{remote_root}/logs/vehicle-{vehicle_id}.log"
    required_commands: tuple[str, ...] = ("python3", "tar", "sha256sum")
    required_python_modules: tuple[str, ...] = ()

    def render(self, template: str, *, release: str | None = None) -> str:
        """Render the documented placeholders in local trusted configuration."""

        values = {
            "vehicle_id": self.vehicle_id,
            "remote_root": self.remote_root,
            "release": release or f"{self.remote_root}/current",
        }
        try:
            return template.format(**values)
        except KeyError as error:
            raise DeploymentError(
                f"Unsupported placeholder in deployment target configuration: {error.args[0]!r}"
            ) from error

    def password(self) -> str | None:
        """Resolve an optional password reference without persisting a secret."""

        if self.password_env:
            value = os.environ.get(self.password_env)
            if value:
                return value
        if self.password_prompt:
            value = getpass(f"SSH password for {self.username}@{self.host}: ")
            if value:
                return value
            raise DeploymentError("SSH password prompt received an empty value")
        if self.password_env:
            raise DeploymentError(
                f"Password environment variable {self.password_env!r} is not set"
            )
        return None


@dataclass(frozen=True)
class CommandResult:
    """Completed remote shell command and its captured output."""

    command: str
    exit_status: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class PreflightReport:
    """Read-only target capability and health check result."""

    ok: bool
    hostname: str
    architecture: str
    free_space: str
    missing_commands: tuple[str, ...]
    missing_python_modules: tuple[str, ...]


@dataclass(frozen=True)
class DeploymentTargetEntry:
    """One Linux endpoint selected by a local deployment-targets file."""

    path: Path
    target: DeploymentTarget


@dataclass(frozen=True)
class DeploymentTargets:
    """Local endpoint inventory and shared bundle settings; no credentials here."""

    path: Path
    bundle_spec: Path
    bundle_output: Path
    targets: tuple[DeploymentTargetEntry, ...]
