"""Fixed-path YAML configuration for the ground-station side program."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .ground_station_type import (
    GroundStationConfiguration,
    GroundStationConfigurationError,
    validate_ground_station_configuration,
)


GROUND_STATION_CONFIGURATION_PATH = Path(__file__).with_name("config") / "ground_station.yaml"


def load_ground_station_configuration() -> GroundStationConfiguration:
    """Load the one versioned ground-station configuration at its fixed path."""

    config_path = GROUND_STATION_CONFIGURATION_PATH.resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except OSError as error:
        raise GroundStationConfigurationError(
            f"Cannot read fixed ground-station configuration {config_path}: {error}"
        ) from error
    except yaml.YAMLError as error:
        raise GroundStationConfigurationError(
            f"Invalid YAML in fixed ground-station configuration {config_path}: {error}"
        ) from error
    if not isinstance(raw, dict) or not isinstance(raw.get("ground_station"), dict):
        raise GroundStationConfigurationError("Ground-station configuration requires a 'ground_station' mapping")

    application = raw["ground_station"]
    listener = _mapping(application.get("listener"), "ground_station.listener")
    terminal = _mapping(application.get("terminal", {}), "ground_station.terminal")
    _reject_unknown(listener, {"host", "port", "max_frame_bytes"}, "ground_station.listener")
    _reject_unknown(
        terminal,
        {
            "dashboard_refresh_hz",
            "lidar_refresh_hz",
            "stale_after_s",
            "acknowledgement_timeout_s",
            "dashboard_enabled",
            "input_enabled",
        },
        "ground_station.terminal",
    )
    return validate_ground_station_configuration(
        GroundStationConfiguration(
            listener_host=listener.get("host", "0.0.0.0"),
            listener_port=listener.get("port", 5000),
            max_frame_bytes=listener.get(
                "max_frame_bytes", GroundStationConfiguration().max_frame_bytes
            ),
            dashboard_refresh_hz=terminal.get("dashboard_refresh_hz", 4.0),
            lidar_refresh_hz=terminal.get("lidar_refresh_hz", 20.0),
            stale_after_s=terminal.get("stale_after_s", 1.0),
            acknowledgement_timeout_s=terminal.get("acknowledgement_timeout_s", 3.0),
            dashboard_enabled=terminal.get("dashboard_enabled", True),
            input_enabled=terminal.get("input_enabled", True),
        )
    )


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise GroundStationConfigurationError(f"{name} must be a mapping")
    return value


def _reject_unknown(values: dict[str, Any], allowed: set[str], name: str) -> None:
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise GroundStationConfigurationError(f"{name} has unsupported fields: {', '.join(unknown)}")
