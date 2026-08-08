# Active API Inventory

This is the definition-level companion to the mechanism pages. Each entry
names the active module's classes/functions and gives its purpose; the linked
module pages explain algorithms and safety behavior in more depth.

## Core

| Module | Classes and functions | Purpose |
| --- | --- | --- |
| `commands.py` | `CommandError`, `CommandType`, `CommandSource`, `CommandOutcome`, `VehicleCommand`, `CommandResult`; `_validate_payload`, string/vehicle helpers | Defines and strictly validates transport-independent commands and acknowledgements. |
| `command_handler.py` | `CommandHandling`, `VehicleCommandHandler.handle/reset` and manual/fleet/SDCS helpers | Turns valid intent into state/planner/controller/safe-stop actions. |
| `module_factory.py` | `VehicleModules`; `build_vehicle_modules` and `build_*` profile helpers | Builds selected adapters/managers without starting them. |
| `vehicle_config.py` | `ConfigError`, `ConfigVehicle`; `load_config`, `load_module_profile`, merge/validation helpers | Resolves and validates profile YAML plus scoped overrides. |
| `vehicle_logic.py` | `RuntimeTelemetry`, `VehicleRuntime`; `start`, `handle_command`, `step`, `shutdown` and safe-write helpers | Owns the control-loop order and global safety lifecycle. |
| `vehicle_main.py` | `main` | Parses standalone runtime options and runs one actor. |
| `vehicle_process.py` | `VehicleProcessSpec`; `build_vehicle_process_runtime`, `run_vehicle_process` | Supplies the platform-neutral one-process launch contract. |
| `vehicle_state_machine.py` | `State`, `StateMachine` | Permits lifecycle transitions and decides whether motion is allowed. |
| `vehicle_types.py` | sensor, estimate, reference, command, and V2V dataclasses | Defines immutable data exchanged between runtime services. |

## Utilities

| Package | Classes and functions | Purpose |
| --- | --- | --- |
| `utils/control/controller` | `ControllerBase`, `ControllerNull`, `ControllerSimple`, `ControllerManual`, `ControllerFleetBase`, `ControllerFleet2D`, `ControllerFleetLongitudinal`; `compute`, `reset`, and input/reference helpers | Converts a controller reference into bounded actuation requests. |
| `utils/control/managers` | `ManagerBase`, `ControllerManager`, `ObserverManager`, `PathPlannerManager`; `select`, `restore_configured`, and delegated service methods | Exposes configured and lazy runtime profiles through stable interfaces. |
| `utils/control/observer` | `ObserverBase`, `ObserverNull`, `ObserverEKF`, `ObserverHighGain`, `ObserverLuenberger`; `start`, `update`, `get_latest`, `stop` | Produces a vehicle-state estimate from sensor data. |
| `utils/control/path_planner` | `PathPlannerBase`, `PathPlannerNull`, `PathPlannerStatic`, `PathPlannerSDCSSmallMap`, `SDCSSmallMapRoadMap`, `SDCSMapNode`; load/reset/update/node-route helpers | Creates controller references from waypoint or directed SDCS-map routes. |
| `utils/io` | `IOBase`, `IONull`, `IOVirtual`, `IOCarla`, `IOQCar2`, ROS stub adapters; `read_to_cache`, `read`, `write`, `stop`, `close` | Owns platform sensor polling and final clipped actuator writes. |
| `utils/ground_station` | bridge protocol/frame types, `GroundStationRuntimeFacade`, TCP/null bridges, `MonitoringSnapshot`; framing, queue, monitoring, and acknowledgement functions | Implements the vehicle-side command/monitoring bridge behind a runtime facade. |
| `utils/v2v` | `V2VBase`, `V2VNull`, `V2VUdp`, message/result dataclasses; `start`, `publish`, `drain_received`, `stop` | Transports generic peer messages with freshness/rate diagnostics. |
| `utils/fleet` | formation/policy/state/result dataclasses, `FleetRegistry`, `FleetStateMachine`, `FleetPeerStore`, `FleetManager`, `FleetFollowingPolicy`, observer variants; formation, codec, lifecycle, and cycle functions | Owns one vehicle-local formation lifecycle, peer validation, following reference, and advisory distributed estimates. |

## Rule

Public lifecycle methods are called through injected interfaces. Private
helpers validate, serialize, format, or manage local implementation state;
they do not provide a second control path around `VehicleRuntime`.
