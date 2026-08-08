"""Executable ground-station applications and their presentation code."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "GroundStationCommandHandler",
    "GroundStationCommandRequest",
    "GroundStationConfiguration",
    "GroundStationConfigurationError",
    "GroundStationServer",
    "CommandDelivery",
    "DisconnectedVehicle",
    "VehicleSession",
    "load_ground_station_configuration",
]

_LAZY_EXPORTS = {
    "GroundStationCommandHandler": (".core.command_handler", "GroundStationCommandHandler"),
    "GroundStationCommandRequest": (".ground_station_type", "GroundStationCommandRequest"),
    "GroundStationConfiguration": (".ground_station_type", "GroundStationConfiguration"),
    "GroundStationConfigurationError": (".ground_station_type", "GroundStationConfigurationError"),
    "GroundStationServer": (".core.server", "GroundStationServer"),
    "CommandDelivery": (".ground_station_type", "CommandDelivery"),
    "DisconnectedVehicle": (".ground_station_type", "DisconnectedVehicle"),
    "VehicleSession": (".ground_station_type", "VehicleSession"),
    "load_ground_station_configuration": (".configuration", "load_ground_station_configuration"),
}


def __getattr__(name: str) -> Any:
    """Keep lightweight diagnostic modules importable in a vehicle bundle."""

    try:
        module_name, attribute = _LAZY_EXPORTS[name]
    except KeyError as error:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from error
    value = getattr(import_module(module_name, __name__), attribute)
    globals()[name] = value
    return value
