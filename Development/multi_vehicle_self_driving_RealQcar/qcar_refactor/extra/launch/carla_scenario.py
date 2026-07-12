"""Parse a shared-simulator setup before launching vehicle processes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


class SimulationSetupError(ValueError):
    """Raised when a multi-vehicle simulation manifest is invalid."""


@dataclass(frozen=True)
class SimulationVehicleSetup:
    """One vehicle process's simulation-owned setup values."""

    vehicle_id: int
    vehicle_config_file: str
    spawn_transform: dict[str, float]
    route: tuple[tuple[float, float], ...]
    tick_owner: bool = False


@dataclass(frozen=True)
class SimulationSetup:
    """Shared CARLA endpoint and independently controlled vehicle setups."""

    host: str
    port: int
    vehicles: tuple[SimulationVehicleSetup, ...]


def parse_simulation_setup(argv=None) -> SimulationSetup:
    """Parse ``--setup-file`` and return one validated simulation setup."""
    parser = argparse.ArgumentParser(description="Parse a multi-vehicle CARLA simulation setup")
    parser.add_argument("--setup-file", required=True, type=Path, help="JSON simulation setup manifest")
    args = parser.parse_args(argv)
    return load_simulation_setup(args.setup_file)


def load_simulation_setup(path: str | Path) -> SimulationSetup:
    """Load a JSON manifest without importing CARLA or starting vehicles."""
    setup_path = Path(path)
    try:
        with setup_path.open("r", encoding="ascii") as file:
            data = json.load(file)
    except FileNotFoundError as exc:
        raise SimulationSetupError(f"Simulation setup file not found: {setup_path}") from exc
    except json.JSONDecodeError as exc:
        raise SimulationSetupError(f"Invalid JSON simulation setup: {exc}") from exc
    if not isinstance(data, dict):
        raise SimulationSetupError("Simulation setup root must be an object")

    host = _string(data, "host", default="127.0.0.1")
    port = _port(data.get("port", 2000))
    raw_vehicles = data.get("vehicles")
    if not isinstance(raw_vehicles, list) or not raw_vehicles:
        raise SimulationSetupError("Simulation setup requires a non-empty 'vehicles' list")

    vehicles = tuple(_vehicle_setup(item) for item in raw_vehicles)
    vehicle_ids = [vehicle.vehicle_id for vehicle in vehicles]
    if len(set(vehicle_ids)) != len(vehicle_ids):
        raise SimulationSetupError("Simulation vehicle IDs must be unique")
    if sum(vehicle.tick_owner for vehicle in vehicles) != 1:
        raise SimulationSetupError("Exactly one simulation vehicle must have tick_owner=true")
    return SimulationSetup(host=host, port=port, vehicles=vehicles)


def _vehicle_setup(data: Any) -> SimulationVehicleSetup:
    if not isinstance(data, dict):
        raise SimulationSetupError("Each simulation vehicle must be an object")
    vehicle_id = data.get("vehicle_id")
    if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool) or vehicle_id < 0:
        raise SimulationSetupError("Simulation vehicle_id must be a non-negative integer")
    config_file = _string(data, "vehicle_config_file", default="config_vehicle_carla.yaml")
    spawn_transform = _spawn_transform(data.get("spawn_transform"))
    route = _route(data.get("route"))
    tick_owner = data.get("tick_owner", False)
    if not isinstance(tick_owner, bool):
        raise SimulationSetupError("Simulation tick_owner must be a boolean")
    return SimulationVehicleSetup(vehicle_id, config_file, spawn_transform, route, tick_owner)


def _spawn_transform(data: Any) -> dict[str, float]:
    if not isinstance(data, dict):
        raise SimulationSetupError("Simulation vehicle requires spawn_transform")
    result = {}
    for key in ("x", "y", "z", "yaw"):
        value = data.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise SimulationSetupError(f"spawn_transform.{key} must be numeric")
        result[key] = float(value)
    return result


def _route(data: Any) -> tuple[tuple[float, float], ...]:
    if not isinstance(data, list) or len(data) < 2:
        raise SimulationSetupError("Simulation vehicle route requires at least two XY waypoints")
    route = []
    for point in data:
        if not isinstance(point, list) or len(point) < 2:
            raise SimulationSetupError("Each route waypoint must contain numeric X and Y values")
        x, y = point[:2]
        if not all(isinstance(value, (int, float)) and not isinstance(value, bool) for value in (x, y)):
            raise SimulationSetupError("Each route waypoint must contain numeric X and Y values")
        route.append((float(x), float(y)))
    return tuple(route)


def _string(data: dict[str, Any], key: str, default: str | None = None) -> str:
    value = data.get(key, default)
    if not isinstance(value, str) or not value:
        raise SimulationSetupError(f"Simulation {key} must be a non-empty string")
    return value


def _port(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 65535:
        raise SimulationSetupError("Simulation port must be an integer in [1, 65535]")
    return value
