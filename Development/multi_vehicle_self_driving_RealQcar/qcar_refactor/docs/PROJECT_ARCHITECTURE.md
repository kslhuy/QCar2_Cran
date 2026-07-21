# QCar Refactor Project Architecture

## Purpose

This project provides a modular vehicle-control runtime for QCar hardware, a deterministic mathematical vehicle, and CARLA. One vehicle actor always follows the same core control pipeline; only configuration and platform adapters change.

`REFACTOR_MASTER_TODO.md` is the implementation checklist. This document defines the project structure, ownership rules, runtime flow, usage, and extension points.

## Architecture At A Glance

```text
configuration / scenario
        |
        v
core.vehicle_config -> core.module_factory
        |                    |
        |                    v
        +--------------> selected modules
                              |
                              v
                       core.VehicleRuntime
                              |
       +----------------------+----------------------+
       |                      |                      |
       v                      v                      v
  utils.io              utils.control           utils.v2v
       |             observer/planner/controller     |
       |                                              v
       +------------------- extra.simulator     future utils.fleet
                                      |
                                      v
                             CARLA / virtual platform
```

## Design Principles

1. One vehicle runtime owns one vehicle actor. `VehicleRuntime` coordinates lifecycle and the control loop; it does not contain platform APIs or fleet policy.
2. Utilities are reusable and narrow. IO adapts a vehicle, control utilities calculate estimates and commands, and V2V transports generic messages.
3. `extra/` owns executable integration code. Simulators, launch workers, sessions, and the future ground-station application do not belong in `core/` or `utils/`.
4. Configuration selects implementations. Core code depends on module contracts, not on a QCar, CARLA, or virtual backend.
5. Safety is centralized. Only `RUNNING` may issue a non-zero command. IO, V2V, fleet, and GUI code cannot bypass the state machine.
6. External resources have one owner. Sessions own simulator clients, actors, world settings, and cleanup. IO only uses an already-created vehicle resource.
7. Test instrumentation stays in `test/`. Synthetic V2V messages, latency traces, CSV files, and plots are not production transport or fleet behavior.
8. One module, one interface. `VehicleRuntime` interacts with each utility only through its injected public interface; it does not import or coordinate that utility's policy objects, caches, state machines, sockets, or backend helpers. This limits runtime-facing interfaces, not platform connections: an IO adapter may combine multiple vehicle-platform devices and sensor streams behind its standard read/write contract. For example, the runtime obtains fleet lifecycle and reference results only from `FleetManager`, then applies vehicle safety and controller decisions itself. New utilities must provide one similarly bounded runtime-facing interface rather than exposing their internal collaborators to `core/`.

## Project Layout

```text
config/                         Named profiles and reusable scenarios
core/                           Platform-neutral vehicle runtime and composition
utils/                          Reusable vehicle adapters and algorithms
extra/                          Platform/application integration and executable workers
test/                           Unit, integration, diagnostic helpers, and artifacts
refs/                           Legacy/reference implementations; not runtime dependencies
docs/                           This architecture guide and the master TODO
```

### `core/`: Runtime And Composition

| Module | Responsibility | Key rule |
|---|---|---|
| `types.py` | Shared data contracts such as `SensorData`, `VehicleStateEstimate`, `ControlCommand`, `ControllerReference`, `GuiCommand`, and `V2VMessage` | Keep contracts backend-neutral and use SI-facing values where available. |
| `vehicle_state_machine.py` | Safety states and allowed transitions | Only `RUNNING` permits non-zero actuation. |
| `vehicle_config.py` | Loads vehicle and selected module profiles, validates composition, and applies overrides | A config loader selects profiles; it does not create backend resources. |
| `module_factory.py` | Maps selected profiles to concrete IO, control, V2V, and simulation modules | This is the only core mapping from configuration to implementations. |
| `vehicle_logic.py` | `VehicleRuntime`: starts modules, runs one control iteration, and shuts down safely | It receives constructed modules and remains platform-independent. |
| `vehicle_process.py` | Builds and runs one independently configured vehicle runtime | It does not create child processes or know CARLA/virtual session details. |
| `vehicle_main.py` | Single-vehicle command-line entry point | Use it for a configured actor, not multi-vehicle scenario orchestration. |

### `utils/`: Reusable Vehicle Utilities

| Area | Contents | Responsibility | Must not own |
|---|---|---|---|
| `utils/io/` | `IOBase`, QCar, virtual, CARLA, and null adapters | Read sensor data, cache it, apply bounded actuator commands, and release adapter-local resources | Simulator startup, actor spawning, world ticking, fleet policy |
| `utils/control/observer/` | Observer base and EKF | Convert sensor samples and last command into a local state estimate | IO polling, path selection, actuator writes |
| `utils/control/path_planner/` | Planner base and static waypoint planner | Convert an estimate and mission path into a planner target | Vehicle IO or controller math |
| `utils/control/controller/` | Controller base and path controller | Convert estimate and target into a `ControlCommand` | State transitions or direct backend control |
| `utils/v2v/` | Null and UDP V2V adapters | Generic `publish(message_type, payload)` and `drain_received()` transport, routing, size limits, and counters | Fleet state schema, peer cache, observer sharing, actuator control |
| `utils/fleet/` | Fleet placeholders and future peer-snapshot utilities | Interpret fleet-specific V2V payloads, membership, freshness, and peer snapshots | UDP socket ownership, direct actuator writes, state-machine transitions |
| `utils/ground_station/` | Future vehicle-side bridge | Queue commands and publish monitoring snapshots to a ground station | Desktop GUI or direct vehicle control |

### `extra/`: Integration And Applications

| Area | Contents | Responsibility |
|---|---|---|
| `extra/simulator/base.py` | Common process-manager lifecycle | Select one vehicle from a scenario, build one `VehicleProcessSpec`, and run one runtime through platform callbacks. |
| `extra/simulator/scenario.py` | Common scenario parsing | YAML loading, simulation-profile selection, mission parsing, V2V endpoint validation, and process-spec assembly. |
| `extra/simulator/carla/` | `CarlaSession`, CARLA scenario parser, process runner | CARLA client/world/actor ownership, synchronous tick coordination, CARLA-specific process lifecycle. |
| `extra/simulator/virtual/` | Virtual scenario parser and process runner | Independent deterministic mathematical vehicle processes. No shared virtual session exists. |
| `extra/ground_station/` | Future executable ground-station server and GUI | Receives vehicle bridge messages and presents telemetry/commands without containing vehicle control logic. |

## Runtime Flow

One `VehicleRuntime.step()` follows this order:

```text
optional simulation tick
    -> IO reads current sensor data
    -> observer updates local estimate
    -> planner produces target
    -> state machine checks drive permission and path completion
    -> controller computes command when RUNNING
    -> IO applies command
    -> completed RuntimeTelemetry is returned
```

The runtime starts the selected simulation session first, then observer, planner/controller reset, and V2V. Shutdown is safe and idempotent: command zero is applied before dependencies are stopped or closed.

For CARLA synchronous mode, the simulation session owns `world.tick()`, updates its latest snapshot, then `IOCarla` reads that completed frame. A multi-vehicle CARLA scenario designates exactly one `tick_owner`; other actors wait for completed frames.

## Configuration And Scenarios

### Profile Configuration

`config/config_vehicle_*.yaml` is a single-vehicle composition template. It selects named module profiles:

```yaml
vehicle:
  car_id: 0

runtime:
  loop_rate_hz: 20

modules:
  model: carla_model3
  io: carla
  observer: ekf_carla
  planner: static_carla
  controller: simple_carla
  v2v: "null"
  simulation: carla_sync
  ground_station: "null"
```

Selected profile definitions live in `config/config_<module>.yaml`. `config_simulation.yaml` contains named shared simulation profiles. `selection_overrides` changes profile names before resolution; `value_overrides` changes values after resolution.

### Multi-Vehicle Scenarios

`config/scenarios/*.yaml` describes instances, not reusable module profiles. Every scenario starts with a `simulation_profile`, then lists vehicle-specific values:

```yaml
simulation_profile: carla_sync

vehicles:
  - vehicle_id: 1
    vehicle_config_file: config_vehicle_carla.yaml
    tick_owner: true
    spawn_transform: {x: -13.1, y: 0.8, z: 0.5, yaw: 0.0}
    mission:
      path: [[-13.1, -0.8], [-10.0, -0.2]]
      target_velocity: 0.4
```

Virtual scenarios explicitly select `simulation_profile: "null"`. Scenario files own vehicle IDs, missions, CARLA spawn transforms, tick ownership, and deployment endpoint overrides. They must not duplicate shared CARLA host, port, timestep, or sensor settings.

## CARLA Boundary

`CarlaSession` owns the CARLA client connection, original world settings, ego/sensor actors, callbacks, synchronous ticking, and cleanup. `IOCarla` only reads the session snapshot and applies a `ControlCommand` to the existing ego actor.

The module boundary uses SI-facing defaults. Current CARLA mapping uses signed forward velocity for `motor_tach`, project-frame IMU yaw rate and acceleration, local project pose, and CARLA simulation elapsed time. The current project-frame conversion is:

```text
project_x = carla_x
project_y = -carla_y
project_yaw = wrap_radians(-radians(carla_yaw_degrees))
```

Project steering radians are converted to CARLA normalized steering; negative throttle maps to brake in the MVP. CARLA sensor/frame mapping remains a platform approximation and should be validated for each selected map and vehicle profile.

## How To Use The Project

### Run One Vehicle

From `qcar_refactor/`, use the normal runtime entry point:

```powershell
python -m core.vehicle_main --vehicle-config config_vehicle_virtual.yaml --cycles 200
```

Use a profile appropriate to the available platform. QCar hardware requires external device bootstrap resources; CARLA requires a running CARLA server and a CARLA profile.

### Run A Scenario Worker

Scenario workers are normally launched by an integration parent process. A worker receives a scenario path, vehicle ID, cycle count, ready marker, and start marker. CARLA and virtual process runners share this protocol; CARLA adds session/tick ownership internally.

### Run Tests

```powershell
python -m unittest discover -s test -p "unit_test_*.py"
python test/test_integration_multi_process_virtual.py
python test/test_integration_multi_process_virtual_v2v.py
```

Direct CARLA tests require a ready local CARLA server. Test-only V2V workers publish `TEST_STATE_V1` and write diagnostic traces; this is not a fleet message format.

## Extension Guide

### Add An IO Backend

1. Implement the `IOBase` contract in `utils/io/`.
2. Define a named profile in `config/config_io.yaml`.
3. Add one factory branch in `core/module_factory.py`; lazy-import optional dependencies there.
4. Keep client/session startup outside the IO adapter. Add an `extra/simulator/<platform>/` session only when the backend owns external resources.
5. Add fake-resource unit tests before live integration tests.

### Add A Control Algorithm

1. Implement the appropriate observer, planner, or controller base contract under `utils/control/`.
2. Add a named config profile.
3. Add a factory branch and focused unit tests.
4. Verify the algorithm through virtual integration before hardware or CARLA validation.

### Add Fleet Or V2V Behavior

1. Keep the V2V adapter payload generic.
2. Define production fleet message schemas and peer snapshots in `utils/fleet/`.
3. Make freshness and membership decisions in fleet, using V2V receive metadata only as local transport information.
4. Keep fleet advisory until a separately reviewed operation strategy consumes it; fleet never writes actuators directly.

### Add Ground-Station Monitoring

1. Define a versioned vehicle monitoring snapshot: estimate, runtime state, last command, IO/observer health, V2V counters, and later fleet summaries.
2. Implement a vehicle-side bridge in `utils/ground_station/`.
3. Implement the server/GUI in `extra/ground_station/`.
4. The GUI enqueues validated commands and displays snapshots; it does not call IO or controllers directly.

## Verification Rules

- Unit tests use fake hardware, fake CARLA resources, and temporary files when validating error paths.
- Multi-process integration tests use canonical `config/scenarios/*.yaml` files and subprocess workers.
- Virtual integration is the first closed-loop validation stage.
- CARLA integration follows only with a ready local server.
- QLabs remains deferred while its license is unavailable.
- Physical QCar validation is last and starts at low throttle.
