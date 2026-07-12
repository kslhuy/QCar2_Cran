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
    controller: object
    v2v: object
    simulation: object | None = None


def build_vehicle_modules(config: ConfigVehicle, logger=None, resources: dict | None = None) -> VehicleModules:
    """Build selected modules with lazy imports for optional backends."""
    simulation = build_simulation(config, logger, resources)
    return VehicleModules(
        io=build_io(config, logger, resources, simulation),
        observer=build_observer(config, logger),
        planner=build_planner(config, logger),
        controller=build_controller(config, logger),
        v2v=build_v2v(config, logger),
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
        return IOQCar2(io_config, qcar=qcar, gps=devices.get("gps"), vehicle_id=config.vehicle_id, logger=logger)
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
        from extra.simulation.carla.session import CarlaSession

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
    planner_config = config.module("planner")
    implementation = planner_config.get("implementation")
    if implementation == "null":
        from utils.control.path_planner.path_planner_base import PathPlannerNull

        return PathPlannerNull(planner_config, config.vehicle_id, logger)
    if implementation == "static":
        from utils.control.path_planner.path_planner_static import PathPlannerStatic

        return PathPlannerStatic(planner_config, config.vehicle_id, logger)
    raise ConfigError(f"Unsupported planner implementation: '{implementation}'")


def build_controller(config: ConfigVehicle, logger=None):
    controller_config = config.module("controller")
    implementation = controller_config.get("implementation")
    if implementation == "null":
        from utils.control.controller.controller_base import ControllerNull

        return ControllerNull(controller_config, config.vehicle_id, logger)
    if implementation == "simple":
        from utils.control.controller.controller_simple import ControllerSimple

        return ControllerSimple(controller_config, config.vehicle_id, logger)
    raise ConfigError(f"Unsupported controller implementation: '{implementation}'")


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
