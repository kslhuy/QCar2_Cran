"""Composition root for concrete vehicle utility implementations.

This module is the only core location that maps configuration profile names to
concrete utility classes. ``vehicle_logic`` receives constructed modules and
therefore stays independent of platform-specific implementations.
"""

from __future__ import annotations

from dataclasses import dataclass

from core.vehicle_config import ConfigError, ConfigVehicle


@dataclass(frozen=True)
class VehicleModules:
    """Concrete modules required by one ``VehicleRuntime`` instance."""

    io: object
    observer: object
    planner: object
    controller_manager: object
    v2v: object
    ground_station: object
    simulation: object | None = None


def build_vehicle_modules(config: ConfigVehicle, logger=None, resources: dict | None = None) -> VehicleModules:
    """Build selected modules with lazy imports for optional backends."""
    simulation = build_simulation(config, logger, resources)
    return VehicleModules(
        io=build_io(config, logger, resources, simulation),
        observer=build_observer_manager(config, logger),
        planner=build_path_planner_manager(config, logger),
        controller_manager=build_controller_manager(config, logger),
        v2v=build_v2v(config, logger),
        ground_station=build_ground_station(config, logger),
        simulation=simulation,
    )


def build_io(config: ConfigVehicle, logger=None, resources: dict | None = None, simulation=None):
    io_config = config.module("io")
    implementation = io_config.get("implementation")
    if implementation == "null":
        from utils.io.io_base import IONull

        return IONull(io_config, config.vehicle_id, logger)
    if implementation == "qcar":
        from utils.io.io_qcar2 import IOQCar2

        devices = resources or {}
        qcar = devices.get("qcar")
        if qcar is None:
            raise ConfigError("QCar IO requires resources['qcar'] from an external device bootstrap")
        return IOQCar2(
            io_config,
            qcar=qcar,
            gps=devices.get("gps"),
            lidar=devices.get("lidar"),
            lidar_manager=devices.get("lidar_manager"),
            vehicle_id=config.vehicle_id,
            logger=logger,
        )
    if implementation == "virtual":
        from utils.io.io_virtual import IOVirtual

        return IOVirtual(io_config, config.vehicle_id, logger)
    if implementation == "carla":
        from utils.io.io_carla import IOCarla

        if simulation is None:
            raise ConfigError("CARLA IO requires a selected CARLA simulation session")
        return IOCarla(io_config, session=simulation, vehicle_id=config.vehicle_id, logger=logger)
    raise ConfigError(f"Unsupported IO implementation for runtime: '{implementation}'")


def build_simulation(config: ConfigVehicle, logger=None, resources: dict | None = None):
    simulation_config = config.module("simulation")
    implementation = simulation_config.get("implementation")
    if implementation == "null":
        return None
    if implementation == "carla":
        from extra.platform.carla.session import CarlaSession

        dependencies = resources or {}
        return CarlaSession(
            simulation_config,
            client=dependencies.get("carla_client"),
            carla_api=dependencies.get("carla_api"),
            logger=logger,
        )
    raise ConfigError(f"Unsupported simulation implementation: '{implementation}'")


def build_observer(config: ConfigVehicle, logger=None):
    observer_config = config.module("observer")
    implementation = observer_config.get("implementation")
    if implementation == "null":
        from utils.control.observer.observer_base import ObserverNull

        return ObserverNull(observer_config, config.vehicle_id, logger)
    if implementation == "ekf":
        from utils.control.observer.observer_ekf import ObserverEKF

        return ObserverEKF(observer_config, config.vehicle_id, logger)
    raise ConfigError(f"Unsupported observer implementation: '{implementation}'")


def build_planner(config: ConfigVehicle, logger=None):
    return _build_planner_profile(config.module("planner"), config.vehicle_id, logger)


def _build_planner_profile(planner_config: dict, vehicle_id: int, logger=None):
    implementation = planner_config.get("implementation")
    if implementation == "null":
        from utils.control.path_planner.path_planner_base import PathPlannerNull

        return PathPlannerNull(planner_config, vehicle_id, logger)
    if implementation == "static":
        from utils.control.path_planner.path_planner_static import PathPlannerStatic

        return PathPlannerStatic(planner_config, vehicle_id, logger)
    if implementation == "sdcs_small_map":
        from utils.control.path_planner.path_planner_sdcs_small_map import PathPlannerSDCSSmallMap

        return PathPlannerSDCSSmallMap(planner_config, vehicle_id, logger)
    raise ConfigError(f"Unsupported planner implementation: '{implementation}'")


def build_observer_manager(config: ConfigVehicle, logger=None):
    """Wrap the configured observer in its stable runtime-facing manager."""
    from utils.control.managers import ObserverManager

    return ObserverManager(build_observer(config, logger))


def build_path_planner_manager(config: ConfigVehicle, logger=None):
    """Wrap the configured path planner in its stable runtime-facing manager."""
    from utils.control.managers import PathPlannerManager

    planner_config = config.module("planner")
    runtime_profiles = planner_config.get("runtime_profiles", {})
    if not isinstance(runtime_profiles, dict):
        raise ConfigError("planner.runtime_profiles must be a mapping")
    builders = {}
    for profile_name, profile_config in runtime_profiles.items():
        if not isinstance(profile_name, str) or not profile_name:
            raise ConfigError("planner runtime profile names must be non-empty strings")
        if not isinstance(profile_config, dict):
            raise ConfigError(f"planner.runtime_profiles.{profile_name} must be a mapping")
        builders[profile_name] = (
            lambda profile_config=dict(profile_config): _build_planner_profile(
                profile_config, config.vehicle_id, logger
            )
        )
    return PathPlannerManager(build_planner(config, logger), builders)


def build_controller(config: ConfigVehicle, logger=None):
    return _build_controller_profile(config.module("controller"), config.vehicle_id, logger)


def _build_controller_profile(controller_config: dict, vehicle_id: int, logger=None):
    implementation = controller_config.get("implementation")
    if implementation == "null":
        from utils.control.controller.controller_base import ControllerNull

        return ControllerNull(controller_config, vehicle_id, logger)
    if implementation == "simple":
        from utils.control.controller.controller_simple import ControllerSimple

        return ControllerSimple(controller_config, vehicle_id, logger)
    if implementation == "fleet_2d":
        from utils.control.controller.controller_fleet.controller_fleet_2d import ControllerFleet2D

        return ControllerFleet2D(controller_config, vehicle_id, logger)
    if implementation == "fleet_longitudinal":
        from utils.control.controller.controller_fleet.controller_fleet_longitudinal import ControllerFleetLongitudinal

        return ControllerFleetLongitudinal(controller_config, vehicle_id, logger)
    if implementation == "manual":
        from utils.control.controller.controller_manual import ControllerManual

        return ControllerManual(controller_config, vehicle_id, logger)
    raise ConfigError(f"Unsupported controller implementation: '{implementation}'")


def build_controller_manager(config: ConfigVehicle, logger=None):
    """Build the configured controller plus explicitly allowed runtime profiles."""
    from utils.control.managers import ControllerManager
    controller_config = config.module("controller")
    manual_config = controller_config.get("manual")
    if manual_config is not None and not isinstance(manual_config, dict):
        raise ConfigError("controller.manual must be a mapping")
    runtime_profiles = controller_config.get("runtime_profiles", {})
    if not isinstance(runtime_profiles, dict):
        raise ConfigError("controller.runtime_profiles must be a mapping")
    profiles = dict(runtime_profiles)
    if manual_config is not None:
        profiles.setdefault("manual", {"implementation": "manual", **manual_config})
    builders = {}
    for profile_name, profile_config in profiles.items():
        if not isinstance(profile_name, str) or not profile_name:
            raise ConfigError("controller runtime profile names must be non-empty strings")
        if not isinstance(profile_config, dict):
            raise ConfigError(f"controller.runtime_profiles.{profile_name} must be a mapping")
        builders[profile_name] = (
            lambda profile_config=dict(profile_config): _build_controller_profile(
                profile_config, config.vehicle_id, logger
            )
        )
    return ControllerManager(build_controller(config, logger), builders)


def build_v2v(config: ConfigVehicle, logger=None):
    v2v_config = config.module("v2v")
    implementation = v2v_config.get("implementation")
    if implementation == "null":
        from utils.v2v.v2v_base import V2VNull

        return V2VNull(v2v_config, config.vehicle_id, logger)
    if implementation == "udp":
        from utils.v2v.v2v_udp import V2VUdp

        return V2VUdp(v2v_config, config.vehicle_id, logger)
    raise ConfigError(f"Unsupported V2V implementation: '{implementation}'")


def build_ground_station(config: ConfigVehicle, logger=None):
    """Build the runtime-facing ground-station facade."""
    from utils.ground_station.runtime_facade import GroundStationRuntimeFacade

    bridge_config = config.module("ground_station")
    return GroundStationRuntimeFacade(
        build_ground_station_bridge(config, logger),
        command_batch_size=int(bridge_config.get("command_batch_size", 8)),
    )


def build_ground_station_bridge(config: ConfigVehicle, logger=None):
    """Build the selected transport bridge behind the ground-station facade."""
    bridge_config = config.module("ground_station")
    implementation = bridge_config.get("implementation")
    if implementation == "null" or not bridge_config.get("enabled", True):
        from utils.ground_station.bridge_base import NullGroundStationBridge

        return NullGroundStationBridge(bridge_config, config.vehicle_id, logger)
    if implementation == "tcp_client":
        from utils.ground_station.bridge_tcp import GroundStationClientBridge

        return GroundStationClientBridge(bridge_config, config.vehicle_id, logger)
    raise ConfigError(f"Unsupported ground-station implementation: '{implementation}'")
