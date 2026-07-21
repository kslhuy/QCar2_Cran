"""Load a YAML virtual-vehicle scenario for independent local processes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Any

from extra.simulator.scenario import (
    build_vehicle_process_spec,
    FleetSetup,
    load_simulation_profile,
    load_yaml_mapping,
    parse_mission_path,
    parse_fleet_setup,
    parse_v2v_endpoint,
    parse_v2v_profile,
    read_string,
    require_mapping,
    validate_unique_v2v_local_ports,
    validate_unique_vehicle_ids,
)


class VirtualSetupError(ValueError):
    """Raised when a multi-vehicle virtual scenario is invalid."""


@dataclass(frozen=True)
class VirtualVehicleSetup:
    """One virtual vehicle's process-owned configuration and mission."""

    vehicle_id: int
    route: tuple[tuple[float, float], ...]
    target_velocity: float = 0.30
    vehicle_config_file: str = "config_vehicle_virtual.yaml"
    v2v: dict[str, Any] | None = None
    v2v_profile: str | None = None
    controller_profile: str | None = None
    initial_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def to_process_spec(self, simulation_profile: str = "null"):
        """Convert this platform scenario entry to the shared process spec."""
        return build_vehicle_process_spec(
            vehicle_id=self.vehicle_id,
            vehicle_config_file=self.vehicle_config_file,
            route=self.route,
            target_velocity=self.target_velocity,
            selection_overrides={
                "simulation": simulation_profile,
                **({"controller": self.controller_profile} if self.controller_profile else {}),
            },
            module_overrides={"io": {
                "initial_x": self.initial_pose[0], "initial_y": self.initial_pose[1], "initial_theta": self.initial_pose[2],
            }, "observer": {"initial_pose": {
                "x": self.initial_pose[0], "y": self.initial_pose[1], "theta": self.initial_pose[2],
            }}},
            v2v=self.v2v,
            v2v_profile=self.v2v_profile,
        )


@dataclass(frozen=True)
class VirtualSetup:
    """A collection of independent deterministic virtual vehicles."""

    simulation_profile: str
    vehicles: tuple[VirtualVehicleSetup, ...]
    fleet: FleetSetup | None = None


def parse_virtual_setup(argv=None) -> VirtualSetup:
    """Parse ``--setup-file`` and return one validated YAML virtual scenario."""
    parser = argparse.ArgumentParser(description="Parse a multi-vehicle virtual scenario")
    parser.add_argument("--setup-file", required=True, help="YAML virtual scenario file")
    args = parser.parse_args(argv)
    return load_virtual_setup(args.setup_file)


def load_virtual_setup(path: str | Path) -> VirtualSetup:
    """Load a virtual scenario without starting vehicle processes."""
    data = load_yaml_mapping(path, "Virtual", VirtualSetupError)
    simulation_profile = read_string(data, "simulation_profile", VirtualSetupError)
    simulation_config = load_simulation_profile(simulation_profile, VirtualSetupError)
    if simulation_config.get("implementation") != "null":
        raise VirtualSetupError("Virtual scenario simulation_profile must select the 'null' profile")

    raw_vehicles = data.get("vehicles")
    if not isinstance(raw_vehicles, list) or not raw_vehicles:
        raise VirtualSetupError("Virtual scenario requires a non-empty 'vehicles' list")
    vehicles = tuple(_parse_virtual_vehicle(item) for item in raw_vehicles)
    _validate_vehicle_ids(vehicles)
    return VirtualSetup(simulation_profile, vehicles, parse_fleet_setup(data, vehicles, VirtualSetupError))


def _parse_virtual_vehicle(data: Any) -> VirtualVehicleSetup:
    if not isinstance(data, dict):
        raise VirtualSetupError("Each virtual scenario vehicle must be a mapping")
    vehicle_id = data.get("vehicle_id")
    if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool) or vehicle_id < 0:
        raise VirtualSetupError("Virtual scenario vehicle_id must be a non-negative integer")

    mission = require_mapping(data.get("mission"), "mission", VirtualSetupError)
    target_velocity = mission.get("target_velocity", 0.30)
    if not isinstance(target_velocity, (int, float)) or isinstance(target_velocity, bool) or target_velocity <= 0:
        raise VirtualSetupError("mission.target_velocity must be a positive number")
    if "ground_station" in data:
        raise VirtualSetupError("ground_station is not supported until the bridge is implemented")
    v2v = parse_v2v_endpoint(data.get("v2v"), VirtualSetupError)
    initial_pose = data.get("initial_pose", [0.0, 0.0, 0.0])
    if not isinstance(initial_pose, list) or len(initial_pose) != 3 or not all(isinstance(value, (int, float)) for value in initial_pose):
        raise VirtualSetupError("initial_pose must be [x_m, y_m, yaw_rad]")
    return VirtualVehicleSetup(
        vehicle_id=vehicle_id,
        vehicle_config_file=read_string(data, "vehicle_config_file", VirtualSetupError, default="config_vehicle_virtual.yaml"),
        route=parse_mission_path(mission.get("path"), VirtualSetupError),
        target_velocity=float(target_velocity),
        v2v=v2v,
        v2v_profile=parse_v2v_profile(data, VirtualSetupError) if v2v is not None else None,
        controller_profile=data.get("controller_profile"),
        initial_pose=tuple(float(value) for value in initial_pose),
    )


def _validate_vehicle_ids(vehicles: tuple[VirtualVehicleSetup, ...]) -> None:
    validate_unique_vehicle_ids(vehicles, "Virtual", VirtualSetupError)
    validate_unique_v2v_local_ports(vehicles, "Virtual", VirtualSetupError)
