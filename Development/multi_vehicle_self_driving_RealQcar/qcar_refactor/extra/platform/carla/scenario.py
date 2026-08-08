"""Load a YAML CARLA scenario before starting independent vehicle processes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from typing import Any

from extra.platform.scenario import (
    build_vehicle_process_spec,
    FleetSetup,
    load_simulation_profile,
    load_yaml_mapping,
    parse_mission_loop,
    parse_mission_node_sequence,
    parse_mission_path,
    parse_fleet_setup,
    parse_v2v_endpoint,
    parse_v2v_profile,
    read_string,
    require_mapping,
    validate_unique_v2v_local_ports,
    validate_unique_vehicle_ids,
)
from utils.fleet import FleetRuntimeSpec


class CarlaSetupError(ValueError):
    """Raised when a multi-vehicle CARLA scenario is invalid."""


@dataclass(frozen=True)
class CarlaVehicleSetup:
    """One vehicle's scenario-owned values and optional network endpoints."""

    vehicle_id: int
    vehicle_config_file: str
    spawn_transform: dict[str, float]
    route: tuple[tuple[float, float], ...] | None
    node_sequence: tuple[int, ...] | None = None
    loop: int | str = 0
    target_velocity: float | None = None
    tick_owner: bool = False
    v2v: dict[str, Any] | None = None
    v2v_profile: str | None = None
    controller_profile: str | None = None

    def to_process_spec(
        self,
        simulation_profile: str,
        fleet_spec: FleetRuntimeSpec | None = None,
    ):
        """Convert one CARLA scenario entry to the shared process contract."""
        # Scenarios spawn actors in CARLA's native frame. IOCarla converts
        # sensor poses to the project's right-handed frame, so the observer
        # must start in that same project frame.
        initial_pose = {
            "x": self.spawn_transform["x"],
            "y": -self.spawn_transform["y"],
            "theta": -math.radians(self.spawn_transform["yaw"]),
        }
        return build_vehicle_process_spec(
            vehicle_id=self.vehicle_id,
            vehicle_config_file=self.vehicle_config_file,
            route=self.route,
            node_sequence=self.node_sequence,
            loop=self.loop,
            target_velocity=self.target_velocity,
            selection_overrides={
                "simulation": simulation_profile,
                **({"controller": self.controller_profile} if self.controller_profile else {}),
            },
            module_overrides={
                "simulation": {
                    "spawn_transform": self.spawn_transform,
                    "tick_owner": self.tick_owner,
                    "warmup_ticks": 0,
                },
                "observer": {"initial_pose": initial_pose},
            },
            v2v=self.v2v,
            v2v_profile=self.v2v_profile,
            fleet_spec=fleet_spec,
        )


@dataclass(frozen=True)
class CarlaSetup:
    """One shared CARLA simulation profile and independently controlled vehicles."""

    simulation_profile: str
    simulation_config: dict[str, Any]
    vehicles: tuple[CarlaVehicleSetup, ...]
    fleet: FleetSetup | None = None

    @property
    def host(self) -> str:
        return str(self.simulation_config["host"])

    @property
    def port(self) -> int:
        return int(self.simulation_config["port"])


def parse_carla_setup(argv=None) -> CarlaSetup:
    """Parse ``--setup-file`` and return one validated YAML CARLA scenario."""
    parser = argparse.ArgumentParser(description="Parse a multi-vehicle CARLA scenario")
    parser.add_argument("--setup-file", required=True, help="YAML CARLA scenario file")
    args = parser.parse_args(argv)
    return load_carla_setup(args.setup_file)


def load_carla_setup(path: str | Path) -> CarlaSetup:
    """Load a scenario without importing CARLA or starting a vehicle process."""
    data = load_yaml_mapping(path, "CARLA", CarlaSetupError)

    simulation_profile = read_string(data, "simulation_profile", CarlaSetupError)
    simulation_config = load_simulation_profile(simulation_profile, CarlaSetupError)
    if simulation_config.get("implementation") != "carla":
        raise CarlaSetupError("CARLA scenario simulation_profile must select a CARLA profile")

    raw_vehicles = data.get("vehicles")
    if not isinstance(raw_vehicles, list) or not raw_vehicles:
        raise CarlaSetupError("CARLA scenario requires a non-empty 'vehicles' list")
    vehicles = tuple(_parse_vehicle_scenario(item) for item in raw_vehicles)
    _validate_scenario_vehicles(vehicles, int(simulation_config["port"]))
    return CarlaSetup(
        simulation_profile,
        simulation_config,
        vehicles,
        parse_fleet_setup(data, vehicles, CarlaSetupError),
    )


def _parse_vehicle_scenario(data: Any) -> CarlaVehicleSetup:
    if not isinstance(data, dict):
        raise CarlaSetupError("Each CARLA scenario vehicle must be a mapping")
    vehicle_id = data.get("vehicle_id")
    if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool) or vehicle_id < 0:
        raise CarlaSetupError("CARLA scenario vehicle_id must be a non-negative integer")
    mission = require_mapping(data.get("mission"), "mission", CarlaSetupError)
    tick_owner = data.get("tick_owner", False)
    if not isinstance(tick_owner, bool):
        raise CarlaSetupError("CARLA scenario tick_owner must be a boolean")
    target_velocity = mission.get("target_velocity")
    if target_velocity is not None and (
        not isinstance(target_velocity, (int, float)) or isinstance(target_velocity, bool) or target_velocity <= 0
    ):
        raise CarlaSetupError("mission.target_velocity must be a positive number")
    if "ground_station" in data:
        raise CarlaSetupError("ground_station is not supported until the bridge is implemented")
    v2v = parse_v2v_endpoint(data.get("v2v"), CarlaSetupError)
    has_path = "path" in mission
    has_node_sequence = "node_sequence" in mission
    if has_path == has_node_sequence:
        raise CarlaSetupError("mission requires exactly one of path or node_sequence")
    return CarlaVehicleSetup(
        vehicle_id=vehicle_id,
        vehicle_config_file=read_string(data, "vehicle_config_file", CarlaSetupError, default="config_vehicle_carla.yaml"),
        spawn_transform=_parse_spawn_transform(data.get("spawn_transform")),
        route=parse_mission_path(mission["path"], CarlaSetupError) if has_path else None,
        node_sequence=(
            parse_mission_node_sequence(mission["node_sequence"], CarlaSetupError)
            if has_node_sequence else None
        ),
        loop=parse_mission_loop(mission, CarlaSetupError),
        target_velocity=float(target_velocity) if target_velocity is not None else None,
        tick_owner=tick_owner,
        v2v=v2v,
        v2v_profile=parse_v2v_profile(data, CarlaSetupError) if v2v is not None else None,
        controller_profile=data.get("controller_profile"),
    )


def _validate_scenario_vehicles(vehicles: tuple[CarlaVehicleSetup, ...], carla_port: int) -> None:
    validate_unique_vehicle_ids(vehicles, "CARLA", CarlaSetupError)
    if sum(vehicle.tick_owner for vehicle in vehicles) != 1:
        raise CarlaSetupError("Exactly one CARLA scenario vehicle must have tick_owner=true")

    validate_unique_v2v_local_ports(vehicles, "CARLA", CarlaSetupError)
    v2v_ports = [vehicle.v2v["local_port"] for vehicle in vehicles if vehicle.v2v is not None and "local_port" in vehicle.v2v]
    if carla_port in set(v2v_ports):
        raise CarlaSetupError("CARLA and V2V ports must use separate values")


def _parse_spawn_transform(data: Any) -> dict[str, float]:
    value = require_mapping(data, "spawn_transform", CarlaSetupError)
    result = {}
    for key in ("x", "y", "z", "yaw"):
        coordinate = value.get(key)
        if not isinstance(coordinate, (int, float)) or isinstance(coordinate, bool):
            raise CarlaSetupError(f"spawn_transform.{key} must be numeric")
        result[key] = float(coordinate)
    return result
