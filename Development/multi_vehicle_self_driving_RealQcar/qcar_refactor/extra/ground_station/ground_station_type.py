"""Typed contracts shared by the operator-side ground-station program."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
import socket
import threading
from typing import Any

from core.commands import VehicleCommand
from core.vehicle_types import LaserScanSample
from utils.ground_station.monitoring import MonitoringSnapshot
from utils.ground_station.protocol import DEFAULT_MAX_FRAME_BYTES

__all__ = [
    "CommandDelivery",
    "DisconnectedVehicle",
    "GroundStationCommandRequest",
    "GroundStationConfiguration",
    "GroundStationConfigurationError",
    "VehicleSession",
    "validate_ground_station_configuration",
]


class GroundStationConfigurationError(ValueError):
    """Raised when the local operator application configuration is invalid."""


@dataclass(frozen=True)
class GroundStationConfiguration:
    """Validated listener and terminal settings for one operator application."""

    listener_host: str = "0.0.0.0"
    listener_port: int = 5000
    max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES
    dashboard_refresh_hz: float = 4.0
    lidar_refresh_hz: float = 20.0
    stale_after_s: float = 1.0
    acknowledgement_timeout_s: float = 3.0
    dashboard_enabled: bool = True
    input_enabled: bool = True

    def with_overrides(
        self,
        *,
        host: str | None = None,
        port: int | None = None,
        refresh_hz: float | None = None,
        lidar_refresh_hz: float | None = None,
        stale_after_s: float | None = None,
        acknowledgement_timeout_s: float | None = None,
        dashboard_enabled: bool | None = None,
        input_enabled: bool | None = None,
    ) -> "GroundStationConfiguration":
        """Return a validated copy with explicit CLI overrides applied."""

        values = {
            "listener_host": host,
            "listener_port": port,
            "dashboard_refresh_hz": refresh_hz,
            "lidar_refresh_hz": lidar_refresh_hz,
            "stale_after_s": stale_after_s,
            "acknowledgement_timeout_s": acknowledgement_timeout_s,
            "dashboard_enabled": dashboard_enabled,
            "input_enabled": input_enabled,
        }
        overridden = replace(self, **{name: value for name, value in values.items() if value is not None})
        return validate_ground_station_configuration(overridden)


@dataclass(frozen=True)
class GroundStationCommandRequest:
    """One terminal action, optionally containing a typed vehicle command."""

    action: str
    vehicle_id: int | None = None
    command: VehicleCommand | None = None


@dataclass
class VehicleSession:
    """Server-owned state for one successfully registered vehicle connection."""

    vehicle_id: int
    session_id: str
    connection: socket.socket
    address: tuple[str, int]
    capabilities: dict[str, object]
    connected_at_monotonic: float
    last_received_monotonic: float
    last_monitoring_monotonic: float | None = None
    monitoring_received_monotonic: deque[float] = field(default_factory=lambda: deque(maxlen=32))
    latest_snapshot: MonitoringSnapshot | None = None
    latest_lidar_scan: LaserScanSample | None = None
    last_lidar_monotonic: float | None = None
    last_command_result: dict[str, object] | None = None
    last_error: str = ""
    _send_lock: threading.Lock = field(default_factory=threading.Lock, repr=False)


@dataclass(frozen=True)
class DisconnectedVehicle:
    """Read-only recent-disconnect record retained for operator diagnostics."""

    vehicle_id: int
    address: tuple[str, int]
    disconnected_at_monotonic: float
    latest_snapshot: MonitoringSnapshot | None
    last_command_result: dict[str, object]
    last_error: str


@dataclass(frozen=True)
class CommandDelivery:
    """Result of attempting to place a command on a registered TCP session."""

    accepted: bool
    reason: str = ""


def validate_ground_station_configuration(
    configuration: GroundStationConfiguration,
) -> GroundStationConfiguration:
    """Validate field-level configuration invariants for the operator app."""

    if not isinstance(configuration.listener_host, str) or not configuration.listener_host.strip():
        raise GroundStationConfigurationError("ground_station.listener.host must be a non-empty string")
    if (
        not isinstance(configuration.listener_port, int)
        or isinstance(configuration.listener_port, bool)
        or not 0 <= configuration.listener_port <= 65535
    ):
        raise GroundStationConfigurationError("ground_station.listener.port must be an integer in [0, 65535]")
    if (
        not isinstance(configuration.max_frame_bytes, int)
        or isinstance(configuration.max_frame_bytes, bool)
        or configuration.max_frame_bytes < 64
    ):
        raise GroundStationConfigurationError(
            "ground_station.listener.max_frame_bytes must be an integer of at least 64"
        )
    for value, name in (
        (configuration.dashboard_refresh_hz, "ground_station.terminal.dashboard_refresh_hz"),
        (configuration.lidar_refresh_hz, "ground_station.terminal.lidar_refresh_hz"),
        (configuration.stale_after_s, "ground_station.terminal.stale_after_s"),
        (configuration.acknowledgement_timeout_s, "ground_station.terminal.acknowledgement_timeout_s"),
    ):
        _positive_number(value, name)
    for value, name in (
        (configuration.dashboard_enabled, "ground_station.terminal.dashboard_enabled"),
        (configuration.input_enabled, "ground_station.terminal.input_enabled"),
    ):
        if not isinstance(value, bool):
            raise GroundStationConfigurationError(f"{name} must be true or false")
    return configuration


def _positive_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise GroundStationConfigurationError(f"{name} must be a positive number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise GroundStationConfigurationError(f"{name} must be a positive number") from error
    if result <= 0.0:
        raise GroundStationConfigurationError(f"{name} must be a positive number")
    return result
