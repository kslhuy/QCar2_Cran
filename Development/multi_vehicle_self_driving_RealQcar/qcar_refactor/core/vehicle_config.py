"""Load one selected, validated configuration bundle for a vehicle runtime."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any, Mapping

import yaml


class ConfigError(ValueError):
    """Raised when a runtime configuration is incomplete or unsafe."""


@dataclass(frozen=True)
class ConfigVehicle:
    """Selected configuration for one vehicle actor.

    Module-owned sections live in ``modules`` so adding a new utility does not
    require adding another dataclass attribute.
    """

    vehicle_id: int
    runtime: dict[str, Any]
    mission: dict[str, Any]
    modules: dict[str, dict[str, Any]]

    def module(self, name: str) -> dict[str, Any]:
        """Return a selected module section by its registered name."""
        try:
            return self.modules[name]
        except KeyError as exc:
            raise ConfigError(f"Selected configuration has no '{name}' module") from exc


_CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def load_config(
    config_dir: str | Path | None = None,
    vehicle_config_file: str | Path = "config_vehicle.yaml",
    selection_overrides: Mapping[str, str] | None = None,
    value_overrides: Mapping[str, Any] | None = None,
) -> ConfigVehicle:
    """Load selected profiles, then apply resolved-value overrides.

    ``selection_overrides`` changes profile names before resolution, for
    example ``{"io": "null"}``. ``value_overrides`` changes fields in
    the selected bundle, for example ``{"runtime": {"loop_rate_hz": 50}}``.
    The loader never merges complete YAML files into each other.
    """
    directory = Path(config_dir) if config_dir is not None else _CONFIG_DIR
    vehicle_path = Path(vehicle_config_file)
    if not vehicle_path.is_absolute():
        vehicle_path = directory / vehicle_path
    vehicle = _load_yaml(vehicle_path)
    vehicle_section = _mapping(vehicle, "vehicle")
    runtime = deepcopy(_mapping(vehicle, "runtime"))
    mission = deepcopy(_mapping(vehicle, "mission"))
    module_selection = _mapping(vehicle, "modules")
    _apply_selection_overrides(module_selection, selection_overrides or {})

    module_configs = {
        name: _select(
            _load_yaml(_module_config_path(directory, name)),
            _string(module_selection, name),
            name,
        )
        for name in module_selection
    }

    # Shared vehicle runtime values are injected only into the modules that own them.
    if "planner" in module_configs:
        if "path" in mission:
            module_configs["planner"].setdefault("path_source", mission["path"])
        module_configs["planner"].setdefault("target_velocity", mission.get("target_velocity", 0.0))
    if "observer" in module_configs and "wheelbase" in module_configs.get("model", {}):
        module_configs["observer"].setdefault("wheelbase", module_configs["model"]["wheelbase"])
    if "io" in module_configs:
        module_configs["io"]["timing"] = {"loop_rate_hz": runtime["loop_rate_hz"]}

    values: dict[str, Any] = {
        "vehicle_id": vehicle_section["car_id"],
        "runtime": runtime,
        "mission": mission,
        "modules": module_configs,
    }
    _apply_value_overrides(values, value_overrides or {})
    if "io" in values["modules"]:
        values["modules"]["io"]["timing"]["loop_rate_hz"] = values["runtime"]["loop_rate_hz"]
    _validate(values)
    return ConfigVehicle(**values)


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="ascii") as file:
            data = yaml.safe_load(file)
    except FileNotFoundError as exc:
        raise ConfigError(f"Configuration file not found: {path}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"Invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"Configuration root must be a mapping: {path}")
    return data


def _mapping(source: Mapping[str, Any], key: str) -> dict[str, Any]:
    value = source.get(key)
    if not isinstance(value, dict):
        raise ConfigError(f"Expected mapping for '{key}'")
    return value


def _string(source: Mapping[str, Any], key: str, default: str | None = None) -> str:
    value = source.get(key, default)
    if not isinstance(value, str) or not value:
        raise ConfigError(f"Expected non-empty string for '{key}'")
    return value


def _select(profiles: Mapping[str, Any], profile: str, kind: str) -> dict[str, Any]:
    value = profiles.get(profile)
    if not isinstance(value, dict):
        raise ConfigError(f"Unknown {kind} profile: '{profile}'")
    return deepcopy(value)


def _module_config_path(directory: Path, module_name: str) -> Path:
    if not isinstance(module_name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", module_name):
        raise ConfigError(f"Invalid module name: '{module_name}'")
    return directory / f"config_{module_name}.yaml"


def _apply_value_overrides(values: dict[str, Any], overrides: Mapping[str, Any]) -> None:
    for key, override in overrides.items():
        if key not in values:
            raise ConfigError(f"Unknown configuration override: '{key}'")
        if isinstance(values[key], dict):
            if not isinstance(override, Mapping):
                raise ConfigError(f"Override for '{key}' must be a mapping")
            _merge(values[key], override)
        else:
            values[key] = override


def _apply_selection_overrides(modules: dict[str, Any], selection: Mapping[str, str]) -> None:
    """Apply CLI profile selection before profiles are resolved."""
    if not isinstance(selection, Mapping):
        raise ConfigError("selection_overrides must be a mapping")
    for key, value in selection.items():
        if not isinstance(key, str) or not isinstance(value, str) or not value:
            raise ConfigError("selection_overrides must map module names to non-empty profile names")
        modules[key] = value


def _merge(target: dict[str, Any], override: Mapping[str, Any]) -> None:
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(target.get(key), dict):
            _merge(target[key], value)
        else:
            target[key] = deepcopy(value)


def _positive_number(value: Any, name: str) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ConfigError(f"'{name}' must be a positive number")


def _validate(values: Mapping[str, Any]) -> None:
    vehicle_id = values["vehicle_id"]
    if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool) or vehicle_id < 0:
        raise ConfigError("'vehicle_id' must be a non-negative integer")
    _positive_number(values["runtime"].get("loop_rate_hz"), "runtime.loop_rate_hz")
    modules = _mapping(values, "modules")
    model = _mapping(modules, "model")
    io = _mapping(modules, "io")
    v2v = _mapping(modules, "v2v")
    _positive_number(model.get("wheelbase"), "model.wheelbase")

    write = _mapping(io, "write")
    read = _mapping(io, "read")
    max_throttle = write.get("max_throttle")
    if not isinstance(max_throttle, (int, float)) or isinstance(max_throttle, bool) or not 0 < max_throttle <= 1:
        raise ConfigError("'io.write.max_throttle' must be in (0, 1]")
    _positive_number(write.get("max_steering"), "io.write.max_steering")
    _positive_number(read.get("sensor_rate_hz"), "io.read.sensor_rate_hz")
    _positive_number(read.get("gps_rate_hz"), "io.read.gps_rate_hz")

    if v2v.get("enabled"):
        port = v2v.get("base_port")
        if not isinstance(port, int) or isinstance(port, bool) or not 1 <= port <= 65535:
            raise ConfigError("'v2v.base_port' must be an integer in [1, 65535]")
