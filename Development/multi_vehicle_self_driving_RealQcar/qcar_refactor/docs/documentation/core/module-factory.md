# `core/module_factory.py`

## 1. Introduction

This is the composition root: it maps selected [[vehicle-config|ConfigVehicle]]
profiles to concrete injected utilities. It is the sole core module that imports
platform-specific implementations, and it only constructs them—it never starts
them or changes vehicle state.

## 2. Code structure

`VehicleModules` is a frozen bundle initialized with IO, observer manager,
planner manager, controller manager, V2V, ground station, and optional
simulation. Every builder takes `ConfigVehicle`, optional logger, and where
needed borrowed `resources` such as QCar/CARLA dependencies.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `VehicleModules` | seven service fields | frozen dependency bundle | Holds all dependencies needed by one `VehicleRuntime`. |
| `build_vehicle_modules(config, logger, resources)` | resolved config and borrowed resources | `VehicleModules` | Builds simulation first, then all runtime services without starting them. |
| `build_io(config, logger, resources, simulation)` | IO profile, device/session resources | concrete IO adapter | Selects null/QCar/virtual/CARLA IO and rejects missing QCar or CARLA session. |
| `build_simulation(config, logger, resources)` | simulation profile, optional CARLA client/API | `CarlaSession` or `None` | Selects null or constructs a CARLA session. |
| `build_observer(config, logger)` | observer profile | observer | Selects null or EKF observer. |
| `build_planner(config, logger)` | configured planner profile | planner | Delegates configured profile construction. |
| `_build_planner_profile(planner_config, vehicle_id, logger)` | planner mapping and local ID | planner | Selects null, static, or SDCS-small-map planner. |
| `build_observer_manager(config, logger)` | observer profile | `ObserverManager` | Wraps configured observer behind the runtime manager. |
| `build_path_planner_manager(config, logger)` | planner profile and runtime profiles | `PathPlannerManager` | Builds configured planner plus lazy named planner builders, including `sdcs_map`. |
| `build_controller(config, logger)` | configured controller profile | controller | Delegates configured controller construction. |
| `_build_controller_profile(controller_config, vehicle_id, logger)` | controller mapping and local ID | controller | Selects null, simple, fleet-2D, fleet-longitudinal, or manual controller. |
| `build_controller_manager(config, logger)` | controller profile/manual/runtime profiles | `ControllerManager` | Builds configured controller and lazy allowed profiles; exposes manual when configured. |
| `build_v2v(config, logger)` | V2V profile | V2V adapter | Selects null or UDP transport. |
| `build_ground_station(config, logger)` | ground-station profile | `GroundStationRuntimeFacade` | Wraps selected bridge with configured command batch size. |
| `build_ground_station_bridge(config, logger)` | ground-station profile | bridge | Selects null bridge when disabled/null, otherwise TCP client bridge. |

## 3. Special data and cross-references

`resources` is a borrowed mapping, not a global device registry: QCar IO requires
`resources["qcar"]`, optional GPS uses `resources["gps"]`, and CARLA session
creation may use `carla_client`/`carla_api`. `VehicleModules.simulation` is
`None` for non-CARLA profiles. Lazy profile builders preserve the configured
planner/controller instance while allowing an operator-selected SDCS/manual
profile to be restored safely afterward.

## 4. Position in the project

[[vehicle-process|build_vehicle_process_runtime]] calls this factory before
creating [[vehicle-runtime|VehicleRuntime]]. It instantiates [[io-base|IOBase]]
and communication/control services but does not call `start`, tick CARLA, own
physical calibration, or transition [[vehicle-state-machine|StateMachine]].

## 5. Use and verification

Resolve configuration first, then call
`build_vehicle_modules(config, resources={...})`. Production QCar runs must
supply an externally bootstrapped calibrated device object; CARLA requires its
selected session profile. `test/unit_test_vehicle_process.py` and
`test/unit_test_vehicle_runtime.py` cover composition; focused IO/controller/
planner tests cover selected concrete implementations.
