"""Common scenario validation for platform network and fleet overrides."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from core.vehicle_config import ConfigError, load_module_profile
from core.vehicle_process import VehicleProcessSpec
from utils.fleet import (
    FleetError,
    FleetFormation,
    FleetFormationBuilder,
    FleetMember,
    FleetPolicy,
    FleetRegistry,
    FleetRuntimeSpec,
)


@dataclass(frozen=True)
class FleetSetup:
    """Scenario-owned initial fleet and its reusable policy profile."""

    profile: str
    registry: FleetRegistry

    @property
    def formation(self) -> FleetFormation:
        return self.registry.snapshot()

    def to_runtime_spec(self) -> FleetRuntimeSpec:
        """Return the platform-neutral fleet inputs for one vehicle process."""
        return FleetRuntimeSpec(self.registry)


def load_yaml_mapping(path: str | Path, platform_name: str, error_type: type[Exception]) -> dict[str, Any]:
    """Load one scenario YAML document without starting a platform runtime."""
    setup_path = Path(path)
    try:
        with setup_path.open("r", encoding="ascii") as file:
            data = yaml.safe_load(file)
    except FileNotFoundError as exc:
        raise error_type(f"{platform_name} scenario file not found: {setup_path}") from exc
    except yaml.YAMLError as exc:
        raise error_type(f"Invalid YAML {platform_name} scenario: {exc}") from exc
    if not isinstance(data, dict):
        raise error_type(f"{platform_name} scenario root must be a mapping")
    return data


def load_simulation_profile(profile: str, error_type: type[Exception]) -> dict[str, Any]:
    """Load the shared simulation profile selected by a scenario."""
    try:
        return load_module_profile("simulation", profile)
    except ConfigError as exc:
        raise error_type(str(exc)) from exc


def parse_fleet_setup(
    scenario: Mapping[str, Any],
    vehicles: tuple[Any, ...],
    error_type: type[Exception],
) -> FleetSetup | None:
    """Parse an optional scenario-level fleet without starting a runtime."""
    raw_fleet = scenario.get("fleet")
    if raw_fleet is None:
        return None
    fleet = require_mapping(raw_fleet, "fleet", error_type)
    profile = read_string(fleet, "profile", error_type)
    formation_id = read_string(fleet, "formation_id", error_type)
    raw_members = fleet.get("members")
    if not isinstance(raw_members, list):
        raise error_type("fleet.members must be a list")
    try:
        policy = FleetPolicy.from_mapping(load_module_profile("fleet", profile))
        members = tuple(
            FleetMember.from_mapping(require_mapping(member, "fleet.members item", error_type))
            for member in raw_members
        )
        builder = FleetFormationBuilder()
        formation = builder.build(
            formation_id,
            members,
            policy,
            available_vehicle_ids=(vehicle.vehicle_id for vehicle in vehicles),
        )
        builder.validate_transport_routes(formation, _v2v_peer_ids_by_vehicle(vehicles, error_type))
    except (ConfigError, FleetError) as exc:
        raise error_type(str(exc)) from exc
    return FleetSetup(profile, FleetRegistry(formation))


def build_vehicle_process_spec(
    *,
    vehicle_id: int,
    vehicle_config_file: str,
    route: tuple[tuple[float, float], ...] | None,
    node_sequence: tuple[int, ...] | None = None,
    loop: int | str = 0,
    target_velocity: float | None,
    module_overrides: Mapping[str, Any] | None = None,
    selection_overrides: Mapping[str, str] | None = None,
    v2v: Mapping[str, Any] | None = None,
    v2v_profile: str | None = None,
    fleet_spec: FleetRuntimeSpec | None = None,
) -> VehicleProcessSpec:
    """Build the common process contract from one parsed simulator vehicle."""
    if (route is None) == (node_sequence is None):
        raise ValueError("A scenario mission must select exactly one of route or node_sequence")
    resolved_modules = deepcopy(dict(module_overrides or {}))
    planner = dict(resolved_modules.get("planner", {}))
    mission: dict[str, Any]
    if route is not None:
        resolved_route = [list(point) for point in route]
        planner["path_source"] = resolved_route
        # Scenario XY waypoints intentionally override a vehicle profile's
        # default SDCS route.
        planner["node_sequence"] = None
        mission = {"path": resolved_route, "node_sequence": None}
    else:
        resolved_nodes = list(node_sequence or ())
        planner["path_source"] = None
        planner["node_sequence"] = resolved_nodes
        planner["loop"] = loop
        mission = {"path": None, "node_sequence": resolved_nodes, "loop": loop}
    if target_velocity is not None:
        planner["target_velocity"] = target_velocity
    resolved_modules["planner"] = planner

    selected_modules = dict(selection_overrides or {})
    if v2v is not None:
        resolved_modules["v2v"] = deepcopy(dict(v2v))
        selected_modules["v2v"] = v2v_profile or "udp_default"
    # A mission combines one route representation with scalar control settings.
    if target_velocity is not None:
        mission["target_velocity"] = target_velocity
    return VehicleProcessSpec(
        vehicle_id=vehicle_id,
        vehicle_config_file=vehicle_config_file,
        selection_overrides=selected_modules or None,
        value_overrides={"mission": mission, "modules": resolved_modules},
        fleet_spec=fleet_spec,
    )


def require_mapping(value: Any, name: str, error_type: type[Exception]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise error_type(f"{name} must be a mapping")
    return value


def read_string(source: Mapping[str, Any], key: str, error_type: type[Exception], default: str | None = None) -> str:
    value = source.get(key, default)
    if not isinstance(value, str) or not value:
        raise error_type(f"{key} must be a non-empty string")
    return value


def parse_mission_path(data: Any, error_type: type[Exception]) -> tuple[tuple[float, float], ...]:
    if not isinstance(data, list) or len(data) < 2:
        raise error_type("mission.path requires at least two XY waypoints")
    route = []
    for point in data:
        if not isinstance(point, list) or len(point) < 2:
            raise error_type("Each mission.path waypoint must contain numeric X and Y values")
        x, y = point[:2]
        if not all(isinstance(value, (int, float)) and not isinstance(value, bool) for value in (x, y)):
            raise error_type("Each mission.path waypoint must contain numeric X and Y values")
        route.append((float(x), float(y)))
    return tuple(route)


def parse_mission_node_sequence(data: Any, error_type: type[Exception]) -> tuple[int, ...]:
    if not isinstance(data, list) or len(data) < 2:
        raise error_type("mission.node_sequence requires at least two SDCS node IDs")
    if any(not isinstance(node_id, int) or isinstance(node_id, bool) or not 0 <= node_id <= 10 for node_id in data):
        raise error_type("mission.node_sequence IDs must be integers in [0, 10]")
    if any(first == second for first, second in zip(data, data[1:])):
        raise error_type("mission.node_sequence cannot contain adjacent duplicate node IDs")
    return tuple(data)


def parse_mission_loop(data: Any, error_type: type[Exception]) -> int | str:
    loop = data.get("loop", 0)
    if loop not in (0, 1, 2, "inf") or isinstance(loop, bool):
        raise error_type("mission.loop must be 0, 1, 2, or 'inf'")
    return loop


def validate_unique_vehicle_ids(vehicles: tuple[Any, ...], platform_name: str, error_type: type[Exception]) -> None:
    vehicle_ids = [vehicle.vehicle_id for vehicle in vehicles]
    if len(set(vehicle_ids)) != len(vehicle_ids):
        raise error_type(f"{platform_name} scenario vehicle IDs must be unique")


def parse_v2v_endpoint(data: Any, error_type: type[Exception]) -> dict[str, Any] | None:
    """Validate one platform scenario's per-vehicle V2V endpoint override."""
    if data is None:
        return None
    data = require_mapping(data, "v2v", error_type)
    result = dict(data)
    bind_ip = data.get("bind_ip", "0.0.0.0")
    if not isinstance(bind_ip, str) or not bind_ip:
        raise error_type("v2v.bind_ip must be a non-empty string")
    result["bind_ip"] = bind_ip
    if "local_port" in data:
        result["local_port"] = _port(data.get("local_port"), "v2v.local_port", error_type)
    peers = data.get("peers", [])
    if not isinstance(peers, list):
        raise error_type("v2v.peers must be a list")
    for peer in peers:
        if not isinstance(peer, dict):
            raise error_type("Each v2v peer must be a mapping")
        peer_id = peer.get("vehicle_id")
        if not isinstance(peer_id, int) or isinstance(peer_id, bool) or peer_id < 0:
            raise error_type("v2v.peers.vehicle_id must be a non-negative integer")
        if "port" in peer:
            _port(peer.get("port"), "v2v.peers.port", error_type)
        peer_ip = peer.get("ip")
        if peer_ip is not None and (not isinstance(peer_ip, str) or not peer_ip):
            raise error_type("v2v.peers.ip must be a non-empty string")
    result["peers"] = peers
    return result


def parse_v2v_profile(data: dict[str, Any], error_type: type[Exception]) -> str:
    """Return the selected V2V profile for a scenario endpoint override."""
    profile = read_string(data, "v2v_profile", error_type, default="udp_default")
    if profile == "null":
        raise error_type("v2v_profile cannot be 'null' when v2v endpoints are configured")
    return profile


def validate_unique_v2v_local_ports(vehicles: tuple[Any, ...], platform_name: str, error_type: type[Exception]) -> None:
    """Reject explicit V2V port collisions in one multi-vehicle scenario."""
    ports = [vehicle.v2v["local_port"] for vehicle in vehicles if vehicle.v2v is not None and "local_port" in vehicle.v2v]
    if len(set(ports)) != len(ports):
        raise error_type(f"{platform_name} scenario V2V local ports must be unique")


def _v2v_peer_ids_by_vehicle(vehicles: tuple[Any, ...], error_type: type[Exception]) -> dict[int, tuple[int, ...]]:
    peers_by_vehicle: dict[int, tuple[int, ...]] = {}
    for vehicle in vehicles:
        endpoint = getattr(vehicle, "v2v", None)
        if endpoint is None:
            raise error_type(f"Fleet vehicle {vehicle.vehicle_id} requires a V2V endpoint")
        peers = endpoint.get("peers", [])
        peers_by_vehicle[vehicle.vehicle_id] = tuple(peer["vehicle_id"] for peer in peers)
    return peers_by_vehicle


def _port(value: Any, name: str, error_type: type[Exception]) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 65535:
        raise error_type(f"{name} must be an integer in [1, 65535]")
    return value
