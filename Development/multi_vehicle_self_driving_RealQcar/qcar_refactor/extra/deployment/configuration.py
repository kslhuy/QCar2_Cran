"""Deployment-side configuration for one generic Linux vehicle endpoint.

This module intentionally describes the operator's SSH/release environment,
not a vehicle-runtime profile.  A selected bundle and release command choose
the QCar, Limo, ROS 2, or other Linux platform.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


from .deployment_type import DeploymentError, DeploymentTarget


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise DeploymentError(f"Deployment target field {name!r} must be a mapping")
    return value


def _required_string(values: dict[str, Any], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value.strip():
        raise DeploymentError(f"Deployment target requires a non-empty {name!r}")
    return value.strip()


def _reject_unknown(values: dict[str, Any], allowed: set[str], name: str) -> None:
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise DeploymentError(f"Deployment target field {name!r} has unsupported fields: {', '.join(unknown)}")


def load_deployment_target(path: str | Path) -> DeploymentTarget:
    """Load one ignored local deployment endpoint file without credentials.

    The required layout is ``deployment_target`` with separate ``ssh``,
    ``release``, and ``preflight`` mappings.
    """

    config_path = Path(path).expanduser().resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except OSError as error:
        raise DeploymentError(f"Cannot read deployment target file {config_path}: {error}") from error
    except yaml.YAMLError as error:
        raise DeploymentError(f"Invalid YAML in deployment target file {config_path}: {error}") from error
    if not isinstance(raw, dict):
        raise DeploymentError("Deployment target file must contain a mapping")

    _reject_unknown(raw, {"deployment_target"}, "root")
    definition = _mapping(raw.get("deployment_target"), "deployment_target")
    _reject_unknown(
        definition,
        {"vehicle_id", "expected_hostname", "ssh", "release", "preflight"},
        "deployment_target",
    )
    ssh = _mapping(definition.get("ssh"), "deployment_target.ssh")
    _reject_unknown(
        ssh,
        {
            "host",
            "port",
            "username",
            "identity_file",
            "password_env",
            "password_prompt",
            "known_hosts",
            "connect_timeout_s",
            "password",
        },
        "deployment_target.ssh",
    )
    if "password" in ssh:
        raise DeploymentError("Deployment target files must not contain a password; use password_env")
    release = _mapping(definition.get("release"), "deployment_target.release")
    _reject_unknown(
        release,
        {"root", "command_timeout_s", "start_command", "stop_command", "log_path_template"},
        "deployment_target.release",
    )
    preflight = _mapping(definition.get("preflight", {}), "deployment_target.preflight")
    _reject_unknown(
        preflight,
        {"required_commands", "required_python_modules"},
        "deployment_target.preflight",
    )
    target_values = {
        "vehicle_id": definition.get("vehicle_id"),
        "expected_hostname": definition.get("expected_hostname"),
        "host": ssh.get("host"),
        "port": ssh.get("port", 22),
        "username": ssh.get("username"),
        "identity_file": ssh.get("identity_file"),
        "password_env": ssh.get("password_env"),
        "password_prompt": ssh.get("password_prompt", False),
        "known_hosts": ssh.get("known_hosts"),
        "connect_timeout_s": ssh.get("connect_timeout_s", 10.0),
        "remote_root": release.get("root"),
        "command_timeout_s": release.get("command_timeout_s", 30.0),
    }
    deployment_values = {
        "start_command": release.get("start_command"),
        "stop_command": release.get("stop_command"),
        "log_path_template": release.get("log_path_template"),
        "required_commands": preflight.get("required_commands", ("python3", "tar", "sha256sum")),
        "required_python_modules": preflight.get("required_python_modules", ()),
    }

    try:
        vehicle_id = int(target_values.get("vehicle_id"))
    except (TypeError, ValueError) as error:
        raise DeploymentError("Deployment target requires an integer 'vehicle_id'") from error
    if vehicle_id < 0:
        raise DeploymentError("Deployment target vehicle_id must be non-negative")

    remote_root = _required_string(target_values, "remote_root")
    if not remote_root.startswith("/"):
        raise DeploymentError("Deployment target remote_root must be an absolute POSIX path")

    known_hosts = Path(_required_string(target_values, "known_hosts")).expanduser()
    identity_value = target_values.get("identity_file")
    identity_file = Path(identity_value).expanduser() if isinstance(identity_value, str) and identity_value else None
    password_env = target_values.get("password_env")
    if password_env is not None and (not isinstance(password_env, str) or not password_env.strip()):
        raise DeploymentError("Deployment target password_env must be a non-empty environment-variable name")
    password_prompt = target_values.get("password_prompt", False)
    if not isinstance(password_prompt, bool):
        raise DeploymentError("Deployment target password_prompt must be true or false")
    if identity_file is None and not password_env and not password_prompt:
        raise DeploymentError(
            "Deployment target requires identity_file, password_env, or password_prompt"
        )

    required_commands = deployment_values.get("required_commands", ("python3", "tar", "sha256sum"))
    if not isinstance(required_commands, (list, tuple)) or not all(
        isinstance(command, str) and command.strip() for command in required_commands
    ):
        raise DeploymentError("preflight.required_commands must be a list of command names")
    required_python_modules = deployment_values.get("required_python_modules", ())
    if not isinstance(required_python_modules, (list, tuple)) or not all(
        isinstance(module, str) and module.strip() and _is_python_module_name(module.strip())
        for module in required_python_modules
    ):
        raise DeploymentError(
            "preflight.required_python_modules must be a list of dotted Python module names"
        )

    def _optional_command(name: str) -> str | None:
        value = deployment_values.get(name)
        if value is None:
            return None
        if not isinstance(value, str) or not value.strip():
            raise DeploymentError(f"release.{name} must be a non-empty command string")
        return value.strip()

    try:
        port = int(target_values.get("port", 22))
        connect_timeout_s = float(target_values.get("connect_timeout_s", 10.0))
        command_timeout_s = float(target_values.get("command_timeout_s", 30.0))
    except (TypeError, ValueError) as error:
        raise DeploymentError("Deployment target port and timeouts must be numeric") from error
    if not 1 <= port <= 65535 or connect_timeout_s <= 0 or command_timeout_s <= 0:
        raise DeploymentError("Deployment target port and timeouts must be positive")

    log_path_template = deployment_values.get("log_path_template")
    if log_path_template is None:
        log_path_template = "{remote_root}/logs/vehicle-{vehicle_id}.log"
    if not isinstance(log_path_template, str) or not log_path_template.strip():
        raise DeploymentError("release.log_path_template must be a non-empty string")

    return DeploymentTarget(
        vehicle_id=vehicle_id,
        host=_required_string(target_values, "host"),
        username=_required_string(target_values, "username"),
        remote_root=remote_root.rstrip("/"),
        known_hosts=known_hosts,
        port=port,
        identity_file=identity_file,
        password_env=password_env.strip() if isinstance(password_env, str) else None,
        password_prompt=password_prompt,
        expected_hostname=(
            target_values["expected_hostname"].strip()
            if isinstance(target_values.get("expected_hostname"), str)
            and target_values["expected_hostname"].strip()
            else None
        ),
        connect_timeout_s=connect_timeout_s,
        command_timeout_s=command_timeout_s,
        start_command=_optional_command("start_command"),
        stop_command=_optional_command("stop_command"),
        log_path_template=log_path_template.strip(),
        required_commands=tuple(command.strip() for command in required_commands),
        required_python_modules=tuple(module.strip() for module in required_python_modules),
    )


def _is_python_module_name(value: str) -> bool:
    """Restrict remote import checks to normal dotted Python module names."""

    return all(part.isidentifier() for part in value.split("."))
