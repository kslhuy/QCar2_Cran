# Refactor Master TODO

This is the single implementation checklist for the refactor, including CARLA. Complete the modules in order because each acceptance gate is the dependency for the next module.

Design references:

- `MINIMAL_REFACTOR_PLAN.md`: architecture, state-machine migration, and legacy compatibility.
- `CARLA_STARTUP_IO_TODO.md`: CARLA data mapping, session ownership, configuration, and shutdown detail.

## Configuration And Composition Rules

`load_config()` has two explicit override inputs. `selection_overrides` changes profile names before the profiles are resolved; `value_overrides` changes fields in the resulting selected sections. Do not combine these levels in one dictionary.

```python
config = load_config(
    selection_overrides={"io": "null", "observer": "null"},
    value_overrides={"runtime": {"loop_rate_hz": 50}},
)
```

Use a dedicated `config_vehicle_*.yaml` file for a stable runtime mode, such as `config_vehicle_headless.yaml`. Use `selection_overrides` only for temporary experiments or tests. `core.vehicle_main` selects a vehicle config with `--vehicle-config`; `--headless` is a shortcut for the headless config file, not a hard-coded set of utility profiles.

`ConfigVehicle` stores module-owned sections in `modules`, not as one dataclass attribute per utility. Use `config.module("io")`, `config.module("observer")`, and so on. Value overrides follow the same structure:

```python
value_overrides={"modules": {"controller": {"kp_velocity": 0.4}}}
```

The loader resolves each entry in `config_vehicle.modules` from `config/config_<module>.yaml`; no module assembly list is maintained in the loader. Adding a utility requires its profile file and factory registration so its configuration can be validated and optional dependencies can remain lazy. It does not require changing the `ConfigVehicle` dataclass.

`core/module_factory.py` is the composition root. It maps selected implementation names to concrete utility classes and optional platform integrations with lazy imports. `core/vehicle_logic.py` only coordinates injected modules and must not import concrete IO, observer, planner, controller, V2V, or platform-session implementations.

Project entry scripts follow one standard method: load one vehicle config, build modules through `core.module_factory`, construct `VehicleRuntime`, and call `start()`/`step()`/`shutdown()` inside `try`/`finally`. Scripts do not create hardware or simulation clients, spawn actors, poll sensors, or apply actuator commands directly.

Package ownership is deliberate: reusable vehicle adapters and algorithms live under `utils/`; optional platform startup and session ownership live under `extra/simulation/`; executable ground-station applications and presentation code live under `extra/ground_station/`. `utils/ground_station/` is reserved for the vehicle-side ground-station command and telemetry bridge, not a desktop GUI.

## Multi-Vehicle Network And Scenario Rules

Keep simulation, V2V, and ground-station endpoints separate. A CARLA port is a shared simulator **server** endpoint: every vehicle process may connect to the same `host:port`, but exactly one synchronous-tick owner advances the shared world. It is not a per-vehicle network port.

V2V is a vehicle-to-vehicle UDP transport. Each active vehicle binds a unique local UDP port, normally `v2v.base_port + vehicle_id`; peers send UDP packets to those ports. The legacy implementation uses a separate V2V range (default `7000 + vehicle_id`), while the refactored `udp_default` profile currently uses `8000 + vehicle_id`. Never overlap this range with CARLA or ground-station ports.

The legacy ground station is a TCP server with one listener per vehicle at `ground_station.base_port + vehicle_id` (default `5000 + vehicle_id`). Each vehicle is a TCP client: it connects to that assigned remote listener, uploads telemetry, and receives commands over the same bidirectional connection. The refactored protocol should eventually use one TCP listener and an initial `vehicle_id` registration message; retain the per-vehicle TCP-port scheme only when legacy compatibility is required.

`config_vehicle_*.yaml` remains a single-vehicle runtime template. A multi-vehicle scenario owns per-instance values: vehicle ID, spawn transform, mission, tick owner, V2V endpoint/peers, and ground-station destination. It must not duplicate shared CARLA session settings such as host, port, fixed delta, world defaults, or sensor defaults.

## Current Foundation

- [x] Shared dataclasses in `core/types.py`.
- [x] Minimal safety state machine in `core/vehicle_state_machine.py`.
- [x] Base/null IO, observer, planner, controller, and V2V interfaces.
- [x] QCar IO adapter, EKF observer, static planner, simple controller, and UDP V2V implementation.
- [x] Deterministic `IOVirtual` bicycle-model adapter for closed-loop integration tests.
- [x] Focused unit tests and one virtual closed-loop integration test.
- [x] Constructors accept a module-owned config dictionary, vehicle ID, and logger.

## Step 1: Define And Load Configuration

Owner: `core/vehicle_config.py` and `config/`.

- [x] Keep `config_vehicle.yaml` limited to car ID, runtime, mission, and the selected `modules` map.
- [x] Keep `config_io.yaml` limited to vehicle read/write adapter profiles.
- [x] Keep one `config_<module>.yaml` file per module profile family.
- [x] Keep `config_v2v.yaml` limited to reusable V2V profiles.
- [x] Add `config_simulation.yaml` for CARLA/other simulation session profiles.
- [x] Load YAML files relative to the project config directory.
- [x] Select profiles explicitly; do not recursively merge unrelated files.
- [x] Apply precedence: selected profile, runtime selection, then CLI override.
- [x] Validate required keys, types, positive rates, ports, and safety limits.
- [x] Return one config bundle containing only selected module sections.
- [x] Add config-loader tests for valid config, missing profile, invalid values, and unavailable backend selection.
- [x] Load selected module profile files directly from the vehicle `modules` map.

Acceptance gate: a null/headless configuration loads without CARLA, ROS, QLabs, or hardware packages.

## Step 2: Stabilize Shared Contracts

Owner: `core/types.py`, base utility classes, and tests.

- [x] Record module-boundary defaults: metres, seconds, m/s, radians, rad/s, and m/s^2.
- [x] Document each backend approximation for `motor_tach`, GPS pose, IMU, and actuator commands.
- [x] Update `ObserverBase` documentation and annotations to use `SensorData` rather than dictionaries.
- [x] Standardize idempotent `stop()` and `close()` behavior for IO and sessions.
- [x] Protect command-cache access consistently with sensor-cache access.
- [x] Convert the state-machine script assertions into discovered `unittest` tests.
- [x] Mark unsupported observer variants clearly or implement them independently.

Acceptance gate: all headless tests pass and every shared field has an explicit unit or approximation note.

## Step 3: Build The Headless Runtime

Owner: `core/vehicle_logic.py` and `core/vehicle_main.py`.

- [x] Create module factories for IO, observer, planner, controller, and V2V.
- [ ] Add ground-station bridge factory with its Step 8 implementation.
- [x] Start dependencies in order and unwind already-started modules after a startup failure.
- [x] Keep `StateMachine` as the safety supervisor only.
- [x] Implement path following as the first `RUNNING` operation strategy.
- [x] Force zero command whenever the state is not `RUNNING`, on exceptions, and before shutdown.
- [x] Process one loop in this order: commands, simulation tick if present, IO read, observer, V2V receive, planner, controller, IO write, telemetry, V2V broadcast.
- [x] Use monotonic loop scheduling and validate `dt`.
- [x] Add runtime tests for READY, START, STOP, emergency stop, path completion, module failure, and shutdown order.

Acceptance gate: null IO starts in READY, drives only after START, and always writes zero before exit.

## Step 4: Complete Vehicle IO

Owner: `utils/io/`.

- [x] Keep IO classes limited to an already-created vehicle object and its sensor source.
- [x] Finish common IO lifecycle behavior and command-cache locking.
- [x] Provide `IOVirtual` for deterministic control-loop integration tests.
- [x] Keep `IOQCar2` as a physical-device adapter; it receives externally owned QCar/GPS objects and does not own a QLabs session.
- [ ] Finish ROS adapters only after their external transport contracts are defined.
- [x] Keep QLabs live work blocked while the license is expired; retain it as reference only.
- [x] Add fake-hardware tests for each completed IO adapter.

Acceptance gate: changing IO backend does not alter observer, planner, controller, state machine, or runtime safety logic.

## Step 5: Add CARLA Simulation And IO

Owners: `extra/simulation/carla/session.py` and `utils/io/io_carla.py`.

- [x] Add `extra/simulation/carla/session.py`.
- [x] Make `CarlaSession` own client connection, world settings, ego/sensor actors, sensor callbacks, synchronous ticks, and cleanup.
- [x] Keep CARLA client/world/actor handles private to the session.
- [x] Expose an ego actor and immutable/latest sensor snapshot to `IOCarla`.
- [x] Add `IOCarla` as a narrow adapter for the already-created ego vehicle.
- [x] Override synchronous `IOCarla.read_to_cache()` to refresh once after each `CarlaSession.tick()`; do not change `IOBase` timing for the MVP.
- [x] Map local pose, signed speed, IMU, steering scale/sign, throttle, brake, and safe stop according to the CARLA design reference.
- [x] Do not let `IOCarla` create clients, tick worlds, spawn actors, or restore world settings.
- [x] Add fake session tests and fake actor tests without CARLA installed.
- [x] Add direct-run live CARLA smoke, control, and `vehicle_main` tests that record CSV data and plot artifacts.

Acceptance gate: selecting CARLA changes only session and IO construction; the shared control pipeline remains unchanged.

## Step 6: Set Up Multiple Independent Vehicles

Owners: `core/vehicle_process.py`, CARLA launch adapters, and tests.

- [x] Build one independent `VehicleRuntime` from one platform-neutral process specification, including config selections, overrides, and externally owned resources.
- [x] Reject a process specification whose vehicle ID conflicts with an override.
- [x] Start, step, and shut down one process-owned runtime in `try`/`finally`.
- [x] Add focused process-builder/lifecycle tests, a two-process virtual-IO integration test, and a two-process virtual closed-loop controller test with CSV/trajectory-plot artifacts.
- [x] Add a two-process virtual-IO integration test through the platform-neutral vehicle-process base.
- [x] Add a parsed simulation setup manifest with per-vehicle IDs, config files, spawn transforms, routes, and exactly one synchronous tick owner.
- [ ] Replace inline JSON test manifests with reusable YAML scenario files under `config/scenarios/`.
- [ ] Make a scenario select one shared `config_simulation.yaml` session profile instead of duplicating CARLA host and port.
- [ ] Keep `config_simulation.yaml` limited to shared session/world/sensor defaults; remove single-vehicle spawn and route assumptions from it.
- [ ] Extend each scenario vehicle entry with optional `v2v` UDP endpoint/peer overrides and ground-station TCP destination settings.
- [ ] Validate unique vehicle IDs, one CARLA tick owner, unique local V2V bind endpoints, and no collision between configured V2V and ground-station port ranges.
- [ ] Add a simulation process launcher that starts one vehicle runtime process per parsed vehicle setup.
- [ ] Make the tick-owner process configure/tick the shared CARLA world; non-owner processes must wait for completed frames and never call `world.tick()`.
- [ ] Pass each process only its own spawn transform, path, vehicle ID, and externally owned resources; do not share Python runtime objects across processes.
- [x] Add a direct-run two-process CARLA control integration test with distinct trajectories, actor ownership/cleanup checks, CSV records, and per-vehicle plots.

Acceptance gate: two independent headless vehicles can start, step, and shut down without V2V, fleet policy, or shared actuator state; the parsed CARLA setup makes one process the only synchronous tick owner.

## Step 7: Add V2V And Fleet Utilities

Owners: `utils/v2v/` and `utils/fleet/`.

- [x] Connect the existing null/UDP V2V adapter through the runtime factory.
- [ ] Apply each scenario vehicle's V2V endpoint and peer list as a `modules.v2v` override when launching independent processes.
- [ ] Ensure V2V cannot write actuator commands or bypass the state machine.
- [ ] Define fleet utility contracts for peer membership, latest `V2VState` snapshots, vehicle roles, and stale-peer removal.
- [ ] Keep fleet utilities independent of UDP sockets, vehicle IO, actuator commands, and state-machine transitions.
- [ ] Keep platoon/fleet control policy out of this step; it remains an optional operation strategy after fleet data is reliable.
- [ ] Add unit tests for peer registration, timeout/removal, snapshot queries, and V2V/fleet isolation.
- [ ] Run two local V2V instances and verify peer-state exchange without a fleet policy.

Acceptance gate: two local vehicles exchange peer state, while neither V2V nor fleet utilities can affect local actuator writes or safety transitions.

## Step 8: Add Ground-Station Client, Server, And GUI Interface

Owners: `utils/ground_station/` and `extra/ground_station/`.

- [ ] Add `NullGroundStationBridge` and a queue-based vehicle-side bridge interface in `utils/ground_station/`.
- [ ] Add `GroundStationClientBridge` for the vehicle-side TCP connection, framing, command receive queue, and telemetry publish queue.
- [ ] Add `GroundStationServer` in `extra/ground_station/` to accept vehicle connections and route messages to the GUI interface.
- [ ] Implement the visual ground-station main program in `extra/ground_station/`; it displays telemetry and enqueues commands but contains no vehicle control logic.
- [ ] Define the minimal legacy-compatible protocol: TCP, length-prefixed MessagePack, vehicle ID, supported commands, and compatibility telemetry.
- [ ] Make the new server use one configured TCP listener and identify a vehicle through its initial registration message; support the legacy `base_port + vehicle_id` listeners only through an explicit compatibility mode.
- [ ] Support only START, STOP, EMERGENCY_STOP, RESET, SET_VELOCITY, and SET_PATH first; reject unsupported commands explicitly.
- [ ] Add bridge factory, framing, translation, reconnect, server routing, GUI queue, and runtime command-mapping tests.

Acceptance gate: replacing the null bridge with a client/server/GUI connection does not change vehicle control, fleet utilities, or safety code.

## Step 9: Validate In Stages

- [x] Run all headless unit and integration tests.
- [x] Run the null/headless runtime.
- [x] Run the CARLA single-vehicle smoke test through `core.vehicle_main` with `test_integration_vehicle_main_carla.py`.
- [x] Run the CARLA closed-loop path test at low speed with `test_integration_carla_control.py` on the configured local scene.
- [ ] Run QLabs only after the license is available again.
- [ ] Run physical QCar at low throttle only after headless and simulation checks pass.

## Step 10: Add Optional Features

- [ ] Add fleet/platoon policy after V2V transport and fleet utilities are stable.
- [ ] Add manual driving, taxi, calibration, perception, SysID, neural observers, scopes, trust, and attack features one module at a time.
- [ ] Keep optional features out of the default import and startup path.
- [ ] Remove legacy coupling only after the replacement behavior has tests.

Acceptance gate: the minimal runtime stays importable and testable without every optional dependency.
