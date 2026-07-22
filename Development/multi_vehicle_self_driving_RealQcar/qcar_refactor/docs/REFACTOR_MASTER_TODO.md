# Refactor Master TODO

This is the single implementation checklist for the refactor, including CARLA. Complete the modules in order because each acceptance gate is the dependency for the next module.

Design reference: `PROJECT_ARCHITECTURE.md` defines the project structure, ownership boundaries, runtime flow, configuration, usage, and extension rules.

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

Package ownership is deliberate: reusable vehicle adapters and algorithms live under `utils/`; simulation platform orchestration lives under `extra/simulator/<platform>/`. `core/vehicle_process.py` builds and runs exactly one vehicle actor; it does not create child processes or know simulator sessions. `extra/simulator/base.py` owns the platform-neutral worker lifecycle and telemetry serialisation, while `extra/simulator/scenario.py` owns common YAML loading, simulation-profile lookup, mission parsing, V2V endpoint validation, unique-ID checks, and common `VehicleProcessSpec` assembly. Every scenario begins with `simulation_profile`: CARLA selects `carla_sync`, while the current isolated virtual runtime selects `"null"`. Both `extra/simulator/carla/` and `extra/simulator/virtual/` provide `scenario.py` and V2V-agnostic `process_runner.py`; their workers share `--setup-file`, `--vehicle-id`, `--cycles`, `--ready-file`, and `--start-file` arguments. `test/helper_v2v_trace.py` is the separate test-worker entry point: it uses the same platform managers but owns the synthetic `TEST_STATE_V1` message and localhost trace capture. Only `extra/simulator/carla/session.py` owns a session because CARLA has a shared external server/world; `IOVirtual` owns an isolated local mathematical model and therefore needs no virtual session. Executable ground-station applications and presentation code live under `extra/ground_station/`. `utils/ground_station/` is reserved for the vehicle-side ground-station command and telemetry bridge, not a desktop GUI.

## Multi-Vehicle Network And Scenario Rules

Keep simulation, V2V, and ground-station endpoints separate. A CARLA port is a shared simulator **server** endpoint: every vehicle process may connect to the same `host:port`, but exactly one synchronous-tick owner advances the shared world. It is not a per-vehicle network port.

V2V is a vehicle-to-vehicle UDP transport. Each active vehicle binds a unique local UDP port, normally `v2v.base_port + vehicle_id`; peers send UDP packets to those ports. The legacy implementation uses a separate V2V range (default `7000 + vehicle_id`), while the refactored `udp_default` profile currently uses `8000 + vehicle_id`. Never overlap this range with CARLA or ground-station ports.

V2V packet contents are untrusted. Invalid envelopes must be dropped without changing vehicle state. V2V only validates the generic envelope and retains local receive timestamps; fleet owns payload decoding and stale-peer decisions. Monotonic timestamps must not be compared across machines or used as observer timestamps. Virtual control tests remain unpaced by default; virtual tests using wall-clock UDP must enable the runner's `--realtime` option or deliberately pace their steps.

The legacy ground station uses one TCP listener per vehicle at `ground_station.base_port + vehicle_id` (default `5000 + vehicle_id`) and infers vehicle identity from the listener. The refactor replaces that scheme with one configured TCP listener and an initial `vehicle_id` registration frame. The refactored implementation must not retain per-vehicle TCP listeners or a legacy compatibility bridge.

`config_vehicle_*.yaml` remains a single-vehicle runtime template. Every multi-vehicle scenario explicitly selects a top-level `simulation_profile` from `config_simulation.yaml`, then owns per-instance values: vehicle ID, spawn transform, mission, tick owner, V2V endpoint/peers, and ground-station destination. It must not duplicate shared CARLA settings such as host, port, fixed delta, world defaults, or sensor defaults.

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
- [x] Process one loop in this order: commands, simulation tick if present, IO read, observer, planner, controller, IO write, telemetry. V2V lifecycle is started and stopped by the runtime; fleet will later consume and publish generic V2V messages.
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

Owners: `extra/simulator/carla/session.py` and `utils/io/io_carla.py`.

- [x] Add `extra/simulator/carla/session.py`.
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
- [x] Give virtual and CARLA the same scenario/process-runner boundary; keep a session module only for externally shared simulation worlds.
- [x] Add a parsed simulation setup manifest with per-vehicle IDs, config files, spawn transforms, routes, and exactly one synchronous tick owner.
- [x] Replace inline JSON test manifests with reusable YAML scenario files under `config/scenarios/`.
- [x] Make a scenario select one shared `config_simulation.yaml` session profile instead of duplicating CARLA host and port.
- [x] Keep `config_simulation.yaml` limited to shared session/world/sensor defaults; retain only a named standalone `default_spawn_transform` fallback.
- [x] Extend CARLA and virtual scenario vehicle entries with optional V2V profile, bind endpoint, and peer overrides.
- [x] Reject ground-station scenario settings until a bridge owns and consumes them in Step 8.
- [x] Validate unique vehicle IDs, one CARLA tick owner, unique local V2V bind endpoints, and no collision between configured V2V and ground-station port ranges.
- [x] Add a simulation process launcher that starts one vehicle runtime process per parsed vehicle setup.
- [x] Make CARLA and virtual multi-process integration tests launch canonical `config/scenarios/*.yaml` files through subprocess workers; keep only execution duration and artifact assertions inside tests.
- [x] Make the tick-owner process configure/tick the shared CARLA world; non-owner processes wait for completed frames and never call `world.tick()`.
- [x] Pass each process only its own spawn transform, path, vehicle ID, and external endpoint settings; do not share Python runtime objects across processes.
- [x] Add a direct-run two-process CARLA control integration test with distinct trajectories, actor ownership/cleanup checks, CSV records, and per-vehicle plots.

Acceptance gate: two independent headless vehicles can start, step, and shut down without V2V, fleet policy, or shared actuator state; the parsed CARLA setup makes one process the only synchronous tick owner.

## Step 7: Finish V2V Transport And Build Fleet Utilities

Owners: `utils/v2v/` and `utils/fleet/`.

- [x] Connect the existing null/UDP V2V adapter through the runtime factory.
- [x] Apply each CARLA or virtual scenario vehicle's V2V endpoint and peer list as a `modules.v2v` override and select its V2V profile when launching independent processes.
- [x] Ensure V2V cannot write actuator commands or bypass the state machine; invalid packets are dropped rather than failing the control loop.
- [x] Run two local virtual vehicle processes and verify generic UDP message exchange without a fleet policy.
- [x] Record test-owned `STATE` payload trajectories, localhost receive latency, and sequence-gap packet-loss diagnostics; add the equivalent direct-run CARLA test.
- [x] Replace the transitional state-specific V2V API (`broadcast_local_state()` and `get_peer_states()`) with generic `publish(message_type, payload)` and `drain_received()` transport methods.
- [x] Keep UDP envelope metadata in V2V: sender ID, message type, sequence, generic payload, send timestamp, receive timestamp, transport counters, and endpoint routing.
- [x] Retain `received_at_monotonic` only as local transport metadata; fleet will use it for stale-peer decisions and never compare it across machines or use it as an observer timestamp.
- [x] Add high-resolution local diagnostic timestamps at `sendto()` and `recvfrom()`; plot transport latency and runtime queue delay separately in localhost tests.
- [x] Default local V2V ports to `base_port + vehicle_id` and peer ports to `base_port + peer_id`; retain explicit scenario ports only as deployment overrides.
- [x] Adopt legacy transport hardening without legacy coupling: configurable UDP socket buffers, selected-peer sends, optional per-message rate limits, thread-safe counters, and a fixed 2048-byte encoded-datagram maximum.
- [x] Make virtual and CARLA V2V diagnostics use the same worker protocol, multi-peer result schema, and test-only CSV/plot helper; retain platform-specific session and timing hooks only.
- [x] Move synthetic V2V state publishing, message draining, and localhost trace capture into `test/helper_v2v_trace.py`. Keep production simulation runners and `utils/v2v/` independent of test payload schemas and vehicle-state encoding.

### Step 7.1: Define Fleet Contracts And Configuration

Owner: `utils/fleet/`, `core/types.py`, and `config/`.

- [x] Define immutable value contracts in `utils/fleet/`: `FleetRole`, `FleetMember`, `FleetFormation`, `FleetStatus`, `FleetPeerSnapshot`, and `FleetPhase`. Add a separate mutable `FleetRegistry` as the sole owner of membership changes.
- [x] Define fleet message schemas with an explicit encoded `schema_version` field.
- [x] Support exactly one leader and one or more ordered followers. Each follower has one predecessor; duplicate vehicle IDs, duplicate order values, missing predecessors, and a non-unique leader are configuration errors.
- [x] Keep initial formation membership and order in the multi-vehicle scenario, where all vehicle IDs are visible. Do not put a complete multi-vehicle formation in `config_vehicle_*.yaml`.
- [x] Add `config_fleet.yaml` for reusable fleet-policy profiles. Each profile specifies fleet-message publish rates, peer timeout, communication topology (`leader_follower` leader-centred star, `predecessor_chain` directed predecessor links, `loop` ring, or `vehicle_vehicle` full mesh), and explicit directed/bidirectional edge semantics.
- [x] Keep V2V endpoints and generic transport routing in the scenario/V2V profile. Fleet topology validates the recipients required for fleet-owned messages; it does not create sockets or duplicate endpoint addresses.
- [x] Validate that topology supports the selected following policy. Direct-predecessor following requires every follower to receive its predecessor estimate; a leader-centred star is therefore valid only for a leader-reference policy or when the required predecessor links are also configured.
- [x] Remove the stale `V2VState` contract from `core/types.py`; `FleetPeerSnapshot` is now the fleet-owned state contract.
- [x] Add fleet payload encoding/decoding, peer-state cache, and freshness policy in `utils/fleet/` during Step 7.4.
- [x] Define `VEHICLE_STATE_ESTIMATE` as the fleet-owned V2V encoding of one vehicle's local `VehicleStateEstimate`, with source timestamp, sequence, encoded `schema_version`, and validity/health metadata. It is not fleet lifecycle state; `utils/v2v/` continues to transport only generic messages.
- [x] Define `FleetStatus` separately for `formation_id`, membership revision, `source_vehicle_id`, member order/role, `FleetPhase`, and peer-health summary. One globally unique vehicle ID is used as the fleet member identity; do not introduce a second unrelated ID.
- [x] Keep `VEHICLE_STATE_ESTIMATE` as the initial V2V data path: it carries the existing self-estimation result from `utils/control/observer` before a fleet-state observer exists. Preserve this publish/decode/snapshot pattern when a future fleet-state observer adds `FLEET_STATE_ESTIMATE` and `FleetStateEstimate`; do not treat fleet-state and self estimates as independent measurements or fuse them without a defined correlation model.
- [x] Keep fleet utilities independent of UDP sockets, vehicle IO, direct actuator writes, CARLA/QCar APIs, and global safety-state transitions.
- [x] Document timestamp rules: fleet uses a local V2V receive-monotonic time only for age/freshness; source timestamps are not compared between machines.
- [x] Add unit tests for formation validation and V2V/fleet ownership isolation.
- [x] Add payload round-trip and malformed-payload tests with the Step 7.4 codec.

Acceptance gate: a valid scenario can build one initial fleet definition for every vehicle process, and invalid membership, topology, or V2V route configuration fails without starting control.

### Step 7.2: Build The Initial Fleet Formation And Dynamic Registry

Owner: `utils/fleet/` and scenario parsing.

- [x] Add `FleetFormationBuilder` to validate the scenario's initial membership/order and initialise the local `FleetRegistry`; it does not freeze membership for the lifetime of the fleet.
- [x] Make `FleetRegistry` own the mutable member list and controlled membership operations (`join`, `leave`, role/order update) with validation and an incrementing membership revision. Callers receive immutable `FleetFormation`/`FleetStatus` snapshots, not the registry's mutable internal list.
- [x] Expose local role, current ordered member snapshot, predecessor, successor list, expected peer IDs, and membership revision through queries.
- [x] Resolve predecessor from the explicit member-order value, not the line order in YAML. For example, order `0` is the leader and order `2` follows order `1`, even if their YAML entries are reversed. This is the required deterministic behaviour.
- [x] Validate that every initial member has a process specification and an enabled compatible V2V endpoint before the scenario accepts the formation. Resolve fleet-message recipients from the configured topology and reject missing required links.
- [ ] Validate dynamic member updates against their future process/V2V registration path before the registry accepts the change.
- [x] Do not create processes, sockets, simulators, actors, or controllers in the formation builder. `extra/simulator/` still owns processes and sessions; `core/vehicle_process.py` still owns one runtime.
- [x] Add unit tests for two- and three-vehicle formations, YAML-order-independent predecessor selection, topology-route rejection, join/leave rejection, membership revision changes, and a vehicle not participating in a fleet.

Acceptance gate: every participating vehicle can resolve its initial role and predecessor from the same scenario without cross-process Python objects, while future membership changes have one validated registry path.

### Step 7.3: Add Fleet Lifecycle Without Expanding The Safety State Machine

Owner: `utils/fleet/`, `core/vehicle_logic.py`, and `core/vehicle_state_machine.py`.

- [x] Add a local `FleetPhase` owned by fleet utilities: `DISABLED -> BUILDING -> ACTIVE -> CANCELLING -> DISABLED`, with `FAULT -> CANCELLING` for fleet-data or policy failures.
- [x] Keep the existing vehicle state machine as the global safety supervisor. Fleet phase is an operating mode, not another copy of `READY`, `RUNNING`, `STOPPED`, or `EMERGENCY_STOP`.
- [x] Define explicit build and cancel requests. A build is allowed only from a safe running precondition with a valid `FleetRegistry` snapshot; cancelling or faulting commands zero through the existing safety path before leaving fleet operation.
- [ ] Define membership-update requests and registry revision checks now. The baseline runtime may reject join, leave, leader handover, and reordering while `ACTIVE`, but later dynamic-fleet work must use these controlled operations rather than mutate a public list.
- [x] Keep fleet commands local/test-driven until Step 8 extends the ground-station protocol; do not add a GUI dependency to fleet construction.
- [x] Add lifecycle tests for peer timeout during build, emergency stop, and runtime shutdown with an active fleet. Successful build, invalid build rejection, cancel, and fault transitions are covered.

Acceptance gate: fleet activation and cancellation are observable, idempotent local transitions that cannot bypass the vehicle safety supervisor.

### Step 7.4: Publish And Maintain Fleet Peer Snapshots

Owner: `utils/fleet/` with generic `utils/v2v/` transport.

- [x] Publish the existing local observer result as `VEHICLE_STATE_ESTIMATE` at the configured rate after `utils/control/observer` updates. This is the baseline V2V data path, not `FleetStatus` and not a fleet-state observer output.
- [x] Drain generic V2V messages in fleet, decode `VEHICLE_STATE_ESTIMATE`, and maintain a latest valid peer snapshot keyed by `source_vehicle_id` and membership revision.
- [x] Record sender ID, source sequence, source timestamp, local receive time, validity, and freshness in every peer snapshot. Reject wrong-role, unexpected-peer, duplicate/out-of-order, stale, malformed, and obsolete-membership-revision messages according to explicit policy.
- [x] Provide immutable snapshot queries for the current fleet and the immediate predecessor. The registry/cache remains internally mutable so it can remove departed or stale peers safely.
- [x] Make freshness loss explicit and safe: a stale or missing predecessor prevents fleet control from issuing a following command and triggers the fleet cancellation/fault policy.
- [x] Keep observer instances local. A future distributed observer is a local fleet component that consumes peer snapshots and produces a separate estimate; it is never a shared cross-process object.
- [x] Add deterministic tests for receive ordering, timeout/removal, sequence gaps, malformed packets, and V2V/fleet isolation using injected local timestamps.

Acceptance gate: a follower obtains only validated, fresh predecessor snapshots through V2V, and transport loss cannot create a fictitious fleet state or actuator command.

### Step 7.5: Add Fleet Controller Base And Initial Longitudinal Controller

Owner: `utils/fleet/`, `utils/control/`, and `core/vehicle_logic.py`.

- [x] Add `utils/control/controller/controller_fleet/controller_fleet_base.py` with `ControllerFleetBase`, a general fleet-controller base derived from `ControllerBase`. Its `compute()` method has the same `(estimate, ControllerReference, dt)` contract as every other controller and advertises support for predecessor-derived references; do not add fleet lifecycle parameters to controller computation.
- [x] In fleet-controller mode, define `ControllerReference` to represent the direct front vehicle's current reference state. `FleetFollowingPolicy` maps a validated predecessor `FleetPeerSnapshot` into this reference only while `FleetPhase` is `ACTIVE`; it does not select or invoke a controller.
- [x] Add `utils/control/controller/controller_fleet/controller_fleet_longitudinal.py` with `ControllerFleetLongitudinal`, derived from `ControllerFleetBase`. It implements the first longitudinal following algorithm using ego state and a validated front-vehicle reference.
- [x] Add `utils/control/controller/controller_fleet/controller_fleet_2d.py` with `ControllerFleet2D`, derived from `ControllerFleetBase`. It uses the front-vehicle pose/heading, desired spacing, and ego state to construct a virtual 2D target behind the front vehicle, then calculates both throttle and steering within the shared command bounds.
- [x] Keep the leader on its normal path/velocity controller. A follower uses the selected `ControllerFleetLongitudinal` or `ControllerFleet2D` only when fleet data is valid; fleet utility code still does not calculate actuator commands or access IO/V2V sockets.
- [x] Keep `ControllerFleetLongitudinal` limited to longitudinal tests. `ControllerFleet2D` must not steer directly toward the front vehicle's centre; it must use a defined virtual-target/path geometry and be validated independently before CARLA use.
- [x] Define explicit throttle/steering command bounds, desired-gap/target-speed limits, predecessor freshness requirements, and a zero-command/fleet-cancel fallback for invalid inputs.
- [x] Make the runtime select exactly one command-producing controller per step: it uses a fleet reference only when the global vehicle state permits driving and the injected `FleetManager` result represents an active, valid follower reference; otherwise it uses the normal controller reference. Fleet utilities do not select, invoke, or overwrite controller commands.
- [x] Move fleet-specific peer processing and lifecycle coordination out of `VehicleRuntime` and into `FleetManager`. Define one fleet runtime result/context that accepts the ego estimate, drained generic V2V messages, and local monotonic time; it validates peers, updates fleet phase, returns an optional V2V publication, an optional follower `ControllerReference`, and an explicit fault/cancel intent. `VehicleRuntime` remains the loop owner: it drains/publishes through V2V, combines its safety state with the manager result, invokes the injected controller, writes IO commands, and applies any returned fault/cancel intent through the global safety state machine. Do not let `FleetManager` select controllers, write actuators, own sockets, or transition the vehicle safety state directly.
- [x] Expose fleet behavior to the vehicle only through the injected `FleetManager` interface. Do not import fleet policy, peer-store, state-machine, or controller implementation classes into `core/vehicle_logic.py`; apply the same one-utility/one-interface rule to every runtime utility.
- [x] Run fleet control at the normal vehicle-runtime loop rate initially. Add a local controller rate limit only when the selected algorithm requires it; do not add another control thread.
- [x] Add unit tests for `ControllerFleetBase` contract behaviour, longitudinal spacing response, 2D virtual-target geometry, throttle/steering bounds, stale predecessor fallback, leader behaviour, and controller/fleet separation. See `FLEET_FOLLOWING_POLICY_SCHEME.md`.

Acceptance gate: a fresh predecessor snapshot is converted into the same `ControllerReference` contract and consumed only by the selected `ControllerFleetBase`; stale or invalid data leaves the vehicle in the existing safe-stop path.

### Step 7.5A: Reinforce Runtime-Facing Module Interfaces

Owner: `core/`, `utils/control/`, `utils/io/`, and `utils/v2v/`.

- [x] Replace controller capability probing in `VehicleRuntime` with one documented `ControllerBase` contract for normal and fleet-reference computation. The runtime may select the reference from its state-machine result and `FleetManager` result, but it must not use `getattr()` to discover a second controller entry point.
- [x] Make `ControllerFleetLongitudinal` derive from `ControllerFleetBase`, not `ControllerFleet2D`. Move shared fleet-following bounds/configuration into the fleet controller base or a package-local shared helper; never import a sibling controller's private helper.
- [x] Keep IO adapters free to use multiple platform connections and sensor streams behind `IOBase`. Move only externally owned QCar/GPS/LiDAR client cleanup out of `IOQCar2`; a device bootstrap/session owns resources that it created.
- [x] Inject the optional `FleetManager` during `VehicleRuntime` construction instead of assigning `runtime.fleet` after construction in a simulator process manager.
- [x] Preserve `V2VBase` as a versatile generic envelope transport: `publish(message_type, payload, targets)`, `drain_received()`, and transport status. Do not put fleet schemas, ground-station TCP, observer algorithms, or application-specific callbacks inside V2V.
- [ ] When a second V2V consumer is introduced, route one drained generic message batch to registered utility-facing consumers from the orchestration layer; do not let multiple utilities drain the transport independently or add a speculative callback framework now.
- [x] Add focused regression tests for the controller contract, fleet construction injection, and IO resource ownership.
- [ ] Add a generic V2V batch-routing test when a second production V2V consumer exists.

Acceptance gate: every implemented utility has one explicit runtime-facing interface; platform-specific adapters may own multiple device connections internally, while V2V remains transport-generic and can support future consumers without fleet coupling.

### Step 7.6: Validate The Fleet In Virtual Simulation First

Owner: `test/`, `extra/simulator/virtual/`, and fleet utilities.

- [x] Add a canonical two-vehicle virtual fleet scenario with one leader, one follower, distinct initial positions, V2V endpoints, a static formation, and a reproducible mission.
- [x] Add a real-time two-process virtual integration test that builds the fleet, activates it, records local and peer estimates, and controls the follower from its predecessor snapshot.
- [x] Record CSV data and plots for trajectories, inter-vehicle spacing, target versus actual spacing, follower/leader speeds, fleet phase, V2V sequence gaps, and local snapshot age.
- [~] Deferred: add deterministic fault-injection cases for delayed, dropped, malformed, and stale predecessor messages. Verify the documented cancellation/safe-stop behavior before safety-fault qualification.
- [x] Establish quantitative acceptance thresholds for spacing error, command limits, timeout response, and clean shutdown before moving the same policy to CARLA.

Acceptance gate: the virtual fleet test demonstrates bounded follower behavior under normal traffic. Communication-fault qualification is explicitly deferred.

### Step 7.7: Repeat The Validated Fleet Scenario In CARLA

Owner: `test/`, `extra/simulator/carla/`, and fleet utilities.

- [x] Reuse the fleet contracts, worker protocol, message schema, and controller strategy from the virtual test. CARLA-specific code remains limited to session, actor, sensor, and world-tick handling.
- [x] Add a direct-run three-process CARLA fleet integration test with exactly one synchronous tick owner, distinct spawn transforms, peer-state records, and the same fleet-phase assertions.
- [x] Write CARLA CSV/plot artifacts equivalent to the virtual test and record CARLA sensor/control timing separately from local V2V snapshot age.
- [~] Deferred: re-run communication-fault tests where practical and document any simulator-specific timing limitation before safety-fault qualification.

Acceptance gate: the same static leader/follower policy works in CARLA without changing V2V transport, fleet contracts, or control ownership. CARLA communication-fault qualification is explicitly deferred.

### Step 7.8: Defer Research Extensions Until The Baseline Is Stable

Owner: future `utils/fleet/` research work.

- [x] Add `DistributedObserverBase` and the default `DistributedObserverFake`. The fake creates a collective estimate from direct measurements, local-observer output, and validated V2V snapshots without calculation; it is the integration contract for later algorithms.
- [x] Add an advisory `DistributedObserverLuenberger` prototype with bounded prediction/correction gains and unit tests. It is not selected by scenarios and cannot provide a fleet-control reference.
- [ ] Reproduce and evaluate the legacy distributed Luenberger observer offline before connecting it to follower control. See `DISTRIBUTED_LUENBERGER_OBSERVER_NOTES.md`.
- [ ] Evaluate sampled-data behavior, measurement noise, delay, dropouts, out-of-order packets, bounded gains, and string stability against the static-fleet baseline.
- [ ] Treat any distributed-observer output as advisory until bounded and stale-data behavior has been demonstrated. Publish it with a distinct schema and validity metadata.
- [ ] Design and validate 2D kinematic fleet estimation/control separately; the legacy longitudinal Luenberger observer is not a direct 2D implementation.
- [ ] Add dynamic joining, leaving, reordering, and leader handover only after ground-station command ownership, membership policy, and safety review are complete.

Final Step 7 acceptance gate: two independent vehicle processes can form a validated static leader/follower fleet, exchange fleet-owned estimates through generic V2V, follow only with fresh predecessor data, cancel safely, and produce repeatable virtual and CARLA evidence. Communication-fault qualification is explicitly deferred. V2V, fleet, IO, and the safety supervisor retain their separate ownership boundaries.

## Step 8: Add Command, Ground-Station, And Deployment Interfaces

Owners: `core/`, `utils/ground_station/`, `extra/ground_station/`, and `extra/deployment/`.

Principle: a vehicle command is a core domain contract, not a GUI or TCP object. `VehicleCommandHandler` owns typed command dispatch to the vehicle state machine, planner, and injected `FleetManager`; `VehicleRuntime` owns the resulting control reset, safe actuator stop, and control loop. Every runtime utility is reached through one injected interface; the runtime does not import or coordinate a utility's internal helpers. `GroundStationCommandHandler` owns CLI parsing and target routing; `GroundStationRuntimeFacade` owns queued command pumping, acknowledgement publication, and monitoring construction; the underlying bridge owns TCP transport only. Ground-station presentation code cannot call a vehicle runtime directly. Deployment is a separate SSH/SFTP concern and must not be mixed with the runtime control connection.

Network direction: the ground station owns one configured TCP listener. Each vehicle opens an outbound connection and must register its own `vehicle_id` before it can exchange telemetry or commands. The listener never infers identity from the TCP source address or a per-vehicle port. This is a new protocol; the refactor does not retain the legacy per-vehicle-listener design.

### Step 8.1: Define The Core Command Contract

Owner: `core/`.

- [x] Add `core/commands.py` with a typed `CommandType`, immutable `VehicleCommand`, command source, command ID, timestamp, payload, and a serializable command acknowledgement/result contract.
- [x] Support only `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, `SET_VELOCITY`, `SET_PATH`, `BUILD_FLEET`, and `CANCEL_FLEET`. Validate each payload at this boundary and explicitly reject unknown commands.
- [x] Define acknowledgement outcomes separately from TCP delivery: applied, deferred, rejected, or failed. Every result includes the original command ID, vehicle ID, resulting runtime state, and a machine-readable reason code plus an operator-readable reason.
- [x] Replace `GuiCommand` directly with `VehicleCommand` across core, simulator runners, and tests. Do not add a compatibility adapter or retain a GUI-named command type.
- [x] Pass every `VehicleCommand` to the injected `FleetManager` first. It handles only recognised fleet lifecycle commands and returns an intent/status; `VehicleCommandHandler` applies the resulting state transition and `VehicleRuntime` alone applies actuator-safe stop work.
- [x] Add unit tests for command parsing, IDs, payload validation, state-machine routing, fleet-command acknowledgement, and safe rejection.

Acceptance gate: tests, CLI, and a future GUI can request the same typed command without importing TCP, Qt, or a simulator module.

### Step 8.2: Define Ground-Station Protocol And Vehicle Bridge

Owner: `utils/ground_station/` and `core/module_factory.py`.

- [x] Define one versioned TCP protocol using bounded, length-prefixed MessagePack frames. Support exactly `REGISTER`, `REGISTER_ACK`, `MONITORING_SNAPSHOT`, `COMMAND_REQUEST`, `COMMAND_ACK`, and `ERROR` frames in the first implementation.
- [x] Specify registration before coding the client: the first valid vehicle frame must be `REGISTER` with protocol version, vehicle ID, client-session ID, and optional capability metadata. The server validates it and returns `REGISTER_ACK`; only then may either side send normal monitoring or command frames.
- [x] Define session ownership and reconnect rules: reject an active duplicate vehicle ID, remove a disconnected session from the registry, retain a bounded recent-disconnect diagnostic record, and permit a newly registered connection after disconnect. Do not identify a vehicle from its IP address, source port, or a `base_port + vehicle_id` listener.
- [x] Define an immutable monitoring snapshot containing vehicle ID, runtime/fleet phase, local estimate health, last command/result, IO/observer health, V2V counters, and fleet peer summaries. Keep it independent of UDP V2V envelopes and unsynchronised remote timestamps.
- [x] Add `GroundStationBridgeBase`, `NullGroundStationBridge`, and `GroundStationClientBridge`. The client owns the reconnecting TCP connection, protocol framing, registration state, and bounded receive/send queues; it never calls `VehicleRuntime.handle_command()` from its network thread.
- [x] Define queue policy: preserve FIFO structural commands to a fixed bound and report overflow as a rejected acknowledgement; coalesce monitoring snapshots to the latest value; prioritise command acknowledgements over telemetry.
- [x] Extend module construction and `VehicleRuntime` so each loop drains a bounded batch of queued `VehicleCommand` objects, applies them through the existing runtime path, publishes the resulting acknowledgement, and emits monitoring snapshots at a configured rate.
- [x] Add a `tcp_client` ground-station profile with server host, listener port, connection/reconnect timing, queue bounds, frame-size limit, monitoring rate, and enabled flag. Keep `null` as the default. Do not add a legacy bridge or a per-vehicle TCP-port profile.
- [x] Add unit tests for framing and frame-size limits, registration ordering/version validation, duplicate-ID rejection, queue bounds, reconnect behavior, command-to-runtime mapping, acknowledgement correlation, and null-bridge behavior.

Acceptance gate: a vehicle can run unchanged with the null bridge or exchange typed commands and telemetry through the TCP bridge without introducing a GUI dependency.

### Step 8.3: Build And Validate A CLI Ground Station First

Owner: `extra/ground_station/` and `test/`.

- [x] Add `GroundStationServer` with one listener, a vehicle-session registry keyed by successfully registered vehicle ID, latest monitoring snapshot, and command/acknowledgement routing. The registry tracks connection/session state rather than a fixed set of expected vehicles.
- [x] Add a CLI server entry point that starts the listener and exposes registered vehicle status.
- [x] Add a continuously refreshing terminal dashboard for integration testing. For every registered vehicle, display connection/registration state, runtime and fleet phase, estimate validity, position/heading/speed, last monitoring age, last command acknowledgement, IO/observer health, V2V receive/drop counters, and fleet peer summary. Use only the server's latest monitoring snapshots; the dashboard does not query or control vehicles directly.
- [x] Keep the terminal dashboard asynchronous from TCP receive and command routing. Refresh at a bounded display rate, retain bounded monitoring history for rates, and show stale or disconnected data explicitly rather than blocking while waiting for a vehicle.
- [x] Add `GroundStationCommandHandler` and an interactive CLI command surface for `list`, `status`, `start`, `stop`, `emergency-stop`, `reset`, `set-velocity`, `set-path`, `build-fleet`, and `cancel-fleet`, targeted by vehicle ID. The handler parses terminal syntax and routes only typed `VehicleCommand` values through the server; it never accesses a vehicle runtime directly.
- [x] Require every CLI command to display its acknowledgement or rejection reason. A successful TCP send alone is not a successful vehicle command.
- [x] Add localhost integration tests using the real server, one vehicle bridge, and CLI-equivalent commands. Validate command acknowledgement, telemetry updates, dashboard snapshot rendering, fleet build/cancel, disconnect, and reconnect.

Acceptance gate: operators can monitor live state, diagnose stale/disconnected vehicles, and command one or more simulated vehicles from a terminal through the production protocol before a GUI exists.

Before using the TCP bridge on a physical-vehicle network outside a trusted laboratory subnet, add authenticated registration, an explicit vehicle/server allow-list, and transport encryption or an isolated VPN. The current implementation is intentionally a localhost/trusted-test-network protocol, not an Internet-facing control service.

### Step 8.3A: Preserve Runtime Ownership Boundaries

Owner: `core/`, `utils/fleet/`, and `utils/ground_station/`.

- [x] Move `MonitoringSnapshot` from `core/` to `utils/ground_station/`, because it is an operator-facing transport/dashboard contract rather than a general vehicle-domain type.
- [x] Add `GroundStationRuntimeFacade` to own bounded bridge command draining, command-result retention/acknowledgement publication, and monitoring-snapshot construction. It accepts a runtime safety callback and never accesses the state machine, IO, controller, fleet internals, or a TCP socket directly.
- [x] Remove ground-station queue, acknowledgement, and monitoring helper methods from `VehicleRuntime`. The runtime now invokes only the facade's `process_pending()` and `publish_monitoring()` operations.
- [x] Add `FleetManager.attach_transport()` and `FleetManager.run_cycle()` so fleet owns its generic V2V drain/publication work. The manager receives a generic V2V facade and still does not own UDP sockets, direct actuator writes, or vehicle state-machine transitions.
- [x] Move normal fleet stop/cancel and fleet-fault lifecycle cleanup behind `FleetManager.stop_for_vehicle()` and `FleetManager.abort()`. `VehicleRuntime` only applies the corresponding zero command or global vehicle-state transition.
- [x] Add focused facade tests for ground-station command/monitoring behavior and fleet V2V exchange, then retain runtime and localhost TCP integration coverage.

Acceptance gate: `VehicleRuntime` contains no ground-station protocol/monitoring helper or fleet transport-exchange helper. It remains the sole owner of actuator-safe writes and the global vehicle safety state machine.

### Step 8.3B: Add Ground-Station Manual Control For Vehicle Tests

Owner: `core/`, `utils/control/`, `utils/ground_station/`, `extra/ground_station/`, and `test/`.

Principle: manual operation is a vehicle-side `ControllerBase` implementation, not a ground-station actuator channel. The ground station produces typed input and displays status; `VehicleCommandHandler` validates mode changes; `ControllerManual` clips and expires the input; `VehicleRuntime` remains the sole IO writer and global safety-state owner while selecting either the configured controller or `ControllerManual`.

- [x] Add typed `ENABLE_MANUAL`, `DISABLE_MANUAL`, and `MANUAL_INPUT` commands. Validate throttle in `[-1, 1]`, finite steering, and no unexpected payload fields.
- [x] Add `utils/control/controller/controller_manual.py` with `ControllerManual`, a normal `ControllerBase` implementation with a monotonic input timeout. It holds only the latest vehicle-side input, clips it to the selected controller profile's `manual` limits, and returns a zero command when stale.
- [x] Build the manual controller from the selected `controller` profile. `ENABLE_MANUAL` selects it, `DISABLE_MANUAL` restores the configured controller, and runtime restart/shutdown clears manual mode.
- [x] Extend `VehicleCommandHandler` to allow manual enable only while the vehicle is `RUNNING`, reject it while fleet operation is not disabled, reject input before enable, and deactivate manual control on any non-running vehicle state.
- [x] Extend `VehicleRuntime` to use manual output only while active, bypass automatic path/fleet control during manual operation, stop on manual-input timeout, and preserve IO clipping as the final actuator boundary.
- [x] Coalesce `MANUAL_INPUT` at the vehicle TCP bridge to one latest input rather than queuing stale steering/throttle values. Keep structural commands FIFO. Do not send a per-input acknowledgement for high-rate manual input.
- [x] Add CLI commands `enable-manual`, `disable-manual`, `manual`, and Windows `manual-drive`. The keyboard loop sends typed input at 20 Hz and sends `STOP` on exit; it never accesses a vehicle runtime or IO directly.
- [x] Add dashboard control-mode display plus unit and localhost TCP integration coverage for input clipping, timeout, command validation, coalescing, and end-to-end manual control.

Acceptance gate: an operator can manually drive a configured virtual vehicle only through the production ground-station protocol. Stale manual input stops the vehicle, fleet/manual mixing is rejected, and no ground-station code calls vehicle IO or a runtime directly.

#### Controller Selection Cleanup

- [x] Replace manual-specific controller selection results with the generic `CommandHandling.controller_profile` intent. `VehicleCommandHandler` owns command semantics; `VehicleRuntime` only applies a selected profile through `ControllerManager`.
- [x] Add optional `FleetPolicy.follower_controller_profile` and propagate it through `FleetStepResult`. A fleet follower can request a configured controller profile without `VehicleRuntime` naming a fleet-controller implementation.
- [x] Move fleet-reference capability validation into `ControllerManager.compute_fleet()`. `VehicleRuntime` retains only the safe-stop response to a returned capability failure and the generic fleet hold action.
- [x] Remove manual-specific selection helpers from `VehicleRuntime`. Planner-completion behavior is now declared by the active controller through `uses_planner_completion`.

### Step 8.4: Add A Separate Real-Vehicle Deployment CLI

Owner: `extra/deployment/`.

- [ ] Add a deployment CLI that uploads an explicitly selected source/config bundle to a configured physical vehicle using SSH/SFTP, verifies a manifest/hash, starts or stops the remote vehicle entry point, tails logs, and retrieves artifacts.
- [ ] Keep host credentials, deployment destinations, and SSH settings outside vehicle configuration profiles and out of source control.
- [ ] Provide dry-run mode and require an explicit target vehicle ID/host for every mutating deployment operation.
- [ ] Test packaging and manifest generation locally; perform real upload/start validation only on an approved physical vehicle at low throttle.

Acceptance gate: deployment is repeatable and auditable without making the ground-station TCP connection responsible for code upload or process management.

### Step 8.5: Add The Qt/PySide Ground-Station Interface Last

Owner: `extra/ground_station/`.

- [ ] Build a Qt/PySide application that consumes the existing server session registry and submits the same typed command requests as the CLI.
- [ ] Display per-vehicle connection, runtime/fleet state, command acknowledgements, estimate health, V2V/fleet counters, and real-time plots from monitoring snapshots.
- [ ] Keep plotting and UI updates asynchronous from networking. Bound history buffers and downsample only for rendering; never block command acknowledgement or vehicle telemetry handling on plot work.
- [ ] Add UI integration tests for server-state updates and command enqueueing. The GUI must not contain vehicle control or fleet formation logic.

Acceptance gate: replacing the CLI client with Qt/PySide changes presentation only; command semantics, TCP framing, vehicle control, fleet utilities, and safety ownership remain unchanged.

## Step 9: Validate In Stages

- [x] Run all headless unit and integration tests.
- [x] Run the null/headless runtime.
- [x] Run the CARLA single-vehicle smoke test through `core.vehicle_main` with `test_integration_vehicle_main_carla.py`.
- [x] Run the CARLA closed-loop path test at low speed with `test_integration_carla_control.py` on the configured local scene.
- [ ] Run the scenario-driven two-process CARLA test with `test_integration_carla_multi_process.py` after the CARLA server is ready.
- [ ] Run QLabs only after the license is available again.
- [ ] Run physical QCar at low throttle only after headless and simulation checks pass.

## Step 10: Add Optional Features

- [ ] Add fleet/platoon policy after V2V transport and fleet utilities are stable.
- [ ] Add manual driving, taxi, calibration, perception, SysID, neural observers, scopes, trust, and attack features one module at a time.
- [ ] Keep optional features out of the default import and startup path.
- [ ] Remove legacy coupling only after the replacement behavior has tests.

Acceptance gate: the minimal runtime stays importable and testable without every optional dependency.
