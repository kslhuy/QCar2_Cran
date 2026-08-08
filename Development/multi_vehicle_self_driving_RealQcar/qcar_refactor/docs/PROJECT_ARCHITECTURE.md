# QCar Refactor Project Architecture

## Purpose

This project provides a modular vehicle-control runtime for QCar hardware, a deterministic mathematical vehicle, and CARLA. One vehicle actor always follows the same core control pipeline; only configuration and platform adapters change.

`REFACTOR_MASTER_TODO.md` is the implementation checklist. This is the canonical architecture reference and practical tutorial: it explains project structure, ownership rules, runtime flow, fleet following, configuration, operation, and extension points.

## Architecture At A Glance

```text
configuration / scenario
        |
        v
core.vehicle_config --->  core.module_factory
        |                     |
        |                     v
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
       |                                             v
       +----------------   extra.platform       utils.fleet
                                  |
                                  v
                      CARLA / virtual platform
```

## Design Principles

1. One vehicle runtime owns one vehicle actor. `VehicleRuntime` coordinates lifecycle, the control loop, and actuator-safe side effects; it does not contain platform APIs, fleet protocol processing, ground-station transport behavior, or command decoding.
2. Utilities are reusable and narrow. IO adapts a vehicle, control utilities calculate estimates and commands, and V2V transports generic messages.
3. `extra/` owns executable integration code. Simulators, launch workers, sessions, and the ground-station application do not belong in `core/` or `utils/`.
4. Configuration selects implementations. Core code depends on module contracts, not on a QCar, CARLA, or virtual backend.
5. Safety is centralized. Only `RUNNING` may issue a non-zero command. IO, V2V, fleet, and GUI code cannot bypass the state machine.
6. External resources have one owner. Sessions own simulator clients, actors, world settings, and cleanup. IO only uses an already-created vehicle resource.
7. Test instrumentation stays in `test/`. Synthetic V2V messages, latency traces, CSV files, and plots are not production transport or fleet behavior.
8. One module, one interface. `VehicleRuntime` interacts with each utility only through its injected public interface; it does not import or coordinate that utility's policy objects, caches, state machines, sockets, or backend helpers. This limits runtime-facing interfaces, not platform connections: an IO adapter may combine multiple vehicle-platform devices and sensor streams behind its standard read/write contract. For example, the runtime obtains fleet lifecycle and reference results only from `FleetManager`, then applies vehicle safety and controller decisions itself. New utilities must provide one similarly bounded runtime-facing interface rather than exposing their internal collaborators to `core/`.
9. Scenario configuration is classified by intent. Current/operator scenarios live directly in `config/scenarios/` and are valid launch inputs. Automated-test-only scenarios live in `config/scenarios/test/`; production or operator documentation must not treat them as current defaults.
10. Durable test evidence has one classified, unique run layout. Tests create results through `test.helper_artifacts`, separate raw data, derived data, figures, and logs, and write a manifest before recording evidence. Test artifacts are never vehicle-runtime or deployment inputs.

## Project Layout

```text
config/                         Named profiles and reusable scenarios
core/                           Platform-neutral vehicle runtime and composition
utils/                          Reusable vehicle adapters and algorithms
extra/                          Platform/application integration and executable workers
test/                           Unit, integration, diagnostic helpers, fixtures, and generated artifacts
refs/                           Legacy/reference implementations; not runtime dependencies
docs/                           This architecture guide and the master TODO
```

### Public Code Structure And Entry Points

This is the maintained map of production code. Class and function names below
are public structure, not a promise that callers may bypass the owning facade.
Tests extend alongside the code they validate; individual test scenario YAML
files under `config/scenarios/test/` are intentionally not documented here.

### Standard Test Artifact Procedure

Every test that intentionally keeps durable output uses `test.helper_artifacts.create_artifact_run()`. It creates one unique run instead of overwriting a previous CSV or plot:

```text
test/artifacts/
  <category>/                    integration | diagnostic
    <platform>/                  carla | virtual | qcar | common
      <test_name>/
        <run_id>/                UTC timestamp + process/unique suffix
          manifest.json
          raw/                   captured scans, frames, telemetry, worker traces
          derived/               processed traces, metrics, localisation output
          figures/               plots and rendered images
          logs/                  text logs and command output
```

The manifest records the schema version, category, platform, test name, run ID, creation time, and test-provided metadata such as scenario file, vehicle IDs, duration, or sensor settings. A test must create the run before durable recording begins, put each output in its classified subdirectory, and assert only the files created in that run. This makes a result traceable without coupling one test to an old run.

`test/artifacts/` ignores newly generated runs. New tests must never write directly below that directory. Any recording required as a deterministic test input belongs under `test/fixtures/` and must be documented as an input, not treated as the latest result; for example, `test/fixtures/qcar/qcar_io_background_buffer.csv` is the EKF replay input. A deployment bundle's internal `payload/` directory exists only inside a packaged release and is not a repository test-output location.

Physical-target tools write relative to the active release so deployment can fetch them safely. After retrieval, place them in `test/artifacts/integration/qcar/<test_name>/<run_id>/` with the same `manifest.json` layout; retain the remote release/bundle identifier in that manifest.

#### `config/`: Selection, Not Executable Code

```text
config/
  config_vehicle_*.yaml          current single-vehicle composition templates
  config_<module>.yaml           named reusable module profiles
  scenarios/
    *.yaml                       current/operator scenario launch inputs
    test/*.yaml                  automated-test-only scenario inputs
```

Configuration files define no Python classes or functions. `config_vehicle_*.yaml`
selects the named module profiles; `config/scenarios/*.yaml` supplies current
per-vehicle mission and platform values. Test scenario configuration remains
co-located with the test suite under `config/scenarios/test/` and grows with
test coverage, rather than becoming an operator default.

#### `core/`: One Vehicle Runtime

| Module | Class definitions | Function definitions | Purpose |
|---|---|---|---|
| `commands.py` | `CommandType`, `CommandSource`, `CommandOutcome`, `VehicleCommand`, `CommandResult` | `VehicleCommand.from_mapping()`, `to_mapping()`; `CommandResult.from_mapping()`, `to_mapping()` | Define and validate transport-neutral command and acknowledgement contracts. |
| `command_handler.py` | `CommandHandling`, `VehicleCommandHandler` | `VehicleCommandHandler.handle()` | Interpret a validated command into planner/controller/fleet intent without writing an actuator. |
| `vehicle_state_machine.py` | `State`, `StateMachine` | `StateMachine.handle_command()`, `should_drive()` | Own legal safety transitions and the permission to drive. |
| `vehicle_config.py` | `ConfigVehicle`, `ConfigError` | `load_config()`, `load_module_profile()` | Resolve selected YAML profiles and validate a vehicle composition. |
| `module_factory.py` | `VehicleModules` | `build_vehicle_modules()`, `build_io()`, `build_planner()`, `build_controller_manager()` | Construct selected runtime utilities at the composition boundary. |
| `vehicle_logic.py` | `RuntimeTelemetry`, `VehicleRuntime` | `VehicleRuntime.start()`, `handle_command()`, `step()`, `shutdown()` | Run one ordered control cycle and apply all actuator-safe lifecycle effects. |
| `vehicle_process.py` | `VehicleProcessSpec` | `build_vehicle_process_runtime()`, `run_vehicle_process()` | Build/run one independently configured actor; it does not launch child processes. |
| `vehicle_main.py` | — | `main()` | Provide the single-vehicle executable entry point. |

#### `utils/`: Reusable Domain Utilities

| Area | Class definitions | Function definitions | Purpose |
|---|---|---|---|
| `io/` | `IOBase`, `IONull`, `IOQCar2`, `IOVirtual`, `IOCarla` | `read_to_cache()`, `read()`, `write()`, `close()` | Adapt platform sensors and bounded actuator output to the common IO contract. |
| `control/observer/` | `ObserverBase`, `ObserverNull`, `ObserverEKF` | `start()`, `update()`, `get_latest()` | Produce a local vehicle state estimate from sensor data. |
| `control/path_planner/` | `PathPlannerBase`, `PathPlannerStatic`, `PathPlannerSDCSSmallMap`, `SDCSSmallMapRoadMap` | `load_path()`, `set_node_sequence()`, `update()`, `is_finished()` | Convert a path or SDCS node route into controller references. |
| `control/managers/` | `ControllerManager`, `ObserverManager`, `PathPlannerManager` | `select()`, `restore_configured()` and each utility's delegated public operation | Expose a selected or lazy runtime profile through one stable interface. |
| `control/controller/` | `ControllerBase`, `ControllerSimple`, `ControllerManual`, fleet-controller classes | `compute()`, `compute_fleet()`, `set_input()` | Convert an estimate and reference into a clipped abstract control request. |
| `fleet/` | `FleetManager`, `FleetRegistry`, `FleetStateMachine`, `FleetFormationBuilder`, peer-store/contracts | `build_fleet_manager()`, `run_cycle()`, `handle_command()` | Own fleet membership, peer freshness, V2V interpretation, and follower references. |
| `v2v/` | `V2VBase`, `V2VNull`, `V2VUdp` | `start()`, `publish()`, `drain_received()`, `stop()` | Transport generic vehicle-to-vehicle messages without fleet control policy. |
| `ground_station/` | `GroundStationRuntimeFacade`, bridge/protocol/monitoring contracts | `process_pending()`, `record_command_result()`, `publish_monitoring()` | Bridge queued ground-station commands and monitoring to the vehicle runtime. |

#### `extra/`: Executable Integration

| Area | Class definitions | Function definitions | Purpose |
|---|---|---|---|
| `platform/base.py` | `PlatformProcessContext`, `BasePlatformProcessManager`, `ScenarioPlatformProcessManager`, `StartPolicy` | `prepare()`, `run_platform_process()` | Share the resource-to-`VehicleProcessSpec` lifecycle and explicit start policy for every executable platform. |
| `platform/scenario.py` | `FleetSetup` | `load_yaml_mapping()`, `build_vehicle_process_spec()`, `parse_mission_path()`, `parse_mission_node_sequence()` | Parse common platform scenario values and assemble one process-local specification. |
| `platform/carla/` | `CarlaSession`, `CarlaSetup`, `CarlaVehicleSetup`, `CarlaProcessManager` | `load_carla_setup()`, process-runner `main()` | Own CARLA client/world/actor lifecycle and CARLA worker startup. |
| `platform/virtual/` | `VirtualSetup`, `VirtualVehicleSetup`, `VirtualProcessManager` | `load_virtual_setup()`, process-runner `main()` | Parse and launch deterministic virtual vehicle workers. |
| `platform/launcher.py` | `PlatformLauncher`, `LaunchReport`, `LaunchEndpointResult` | scenario-launcher `main()`, `launch()` | Present one lifecycle interface while selecting local subprocess or authenticated remote transport from the configured targets. |
| `platform/qcar/` | `QCarLidarResourceManager` | QCar resource context, process-runner `main()` | Own the PAL QCar session and defer QCar LiDAR creation until local IO receives the typed diagnostic command; physical runtime remains operator-gated in `READY`. |
| `platform/ros2/` | — | ROS 2 resource context | Own a future ROS 2 node lifecycle; a separate IO adapter will consume the supplied resource. |
| `deployment/deployment_type.py` | bundle, target, command-result, preflight, and target-inventory contracts | — | Keep deployment data contracts independent from bundle, SSH, and CLI service behavior. |
| `ground_station/ground_station_type.py` | configuration, command-request, session, disconnect, and command-delivery contracts | configuration validation | Keep operator-side data contracts independent from listener, terminal, and plotting behavior. |
| `ground_station/core/` | `GroundStationServer`, `GroundStationCommandHandler` | listener `main()`, `GroundStationCommandHandler.parse()`, `route()` | Own TCP listener/session lifecycle and typed command routing without direct hardware access. |
| `ground_station/presentation/` | `GroundStationDashboard`, `GroundStationTerminal` | terminal `main()` | Render received state and submit only typed commands through the core server route. |
| `ground_station/utils/` | live-plotting and logging helpers | `get_ground_station_logger()` | Share presentation support without creating vehicle IO, a runtime, or a second command protocol. |

### `core/`: Runtime And Composition

| Module | Responsibility | Key rule |
|---|---|---|
| `vehicle_types.py`, `commands.py` | Shared sensor/control/V2V contracts and typed `VehicleCommand`/`CommandResult` | Keep contracts transport- and backend-neutral. |
| `command_handler.py` | `VehicleCommandHandler` maps a typed command to state-machine, planner, and `FleetManager` semantics | It returns safety intent only; it never writes IO or uses a network connection. |
| `vehicle_state_machine.py` | Safety states and allowed transitions | Only `RUNNING` permits non-zero actuation. |
| `vehicle_config.py` | Loads vehicle and selected module profiles, validates composition, and applies overrides | A config loader selects profiles; it does not create backend resources. |
| `module_factory.py` | Maps selected profiles to concrete IO, control, V2V, simulation, and ground-station facade modules | This is the only core mapping from configuration to implementations. It builds the control utility managers and wraps the selected ground-station bridge in `GroundStationRuntimeFacade`. |
| `vehicle_logic.py` | `VehicleRuntime`: starts prepared modules, invokes runtime-facing facades, performs safety actions, runs one control iteration, and shuts down safely | It receives prepared `ControllerManager`, ground-station facade, and any transport-bound fleet manager; it performs no module construction or cross-module setup. |
| `vehicle_process.py` | Builds and runs one independently configured vehicle runtime | It does not create child processes or know CARLA/virtual session details. |
| `vehicle_main.py` | Single-vehicle command-line entry point | Use it for a configured actor, not multi-vehicle scenario orchestration. |

### `utils/`: Reusable Vehicle Utilities

| Area | Contents | Responsibility | Must not own |
|---|---|---|---|
| `utils/io/` | `IOBase`, QCar, virtual, CARLA, and null adapters | Read sensor data, cache it, apply bounded actuator commands, and release adapter-local resources | Simulator startup, actor spawning, world ticking, fleet policy |
| `utils/control/observer/` | Observer base and EKF | Convert sensor samples and last command into a local state estimate | IO polling, path selection, actuator writes |
| `utils/control/path_planner/` | Planner base and static waypoint planner | Convert an estimate and mission path into a planner target | Vehicle IO or controller math |
| `utils/control/` | Observer, planner, and controller implementations plus `managers/` | `ControllerManager`, `ObserverManager`, and `PathPlannerManager` expose selected utilities through stable runtime interfaces; controllers convert estimate/target or validated manual input into `ControlInput` | State transitions, TCP input, or direct backend control |
| `utils/v2v/` | Null and UDP V2V adapters | Generic `publish(message_type, payload)` and `drain_received()` transport, routing, size limits, and counters | Fleet state schema, peer cache, observer sharing, actuator control |
| `utils/fleet/` | Fleet manager, formation/state contracts, peer store, following policy, and `fleet_utils/distributed_observer/` | Interpret fleet-specific V2V payloads, membership, freshness, peer snapshots, advisory distributed estimates, and fleet-owned generic V2V exchange | UDP socket ownership, direct actuator writes, vehicle state-machine transitions |
| `utils/ground_station/` | Monitoring contract, runtime facade, protocol, null bridge, and reconnecting TCP client bridge | Queue structural commands, coalesce high-rate manual inputs, publish acknowledgements, and build/publish monitoring snapshots | Desktop GUI, fleet policy, or direct vehicle control |

### Fleet Utils File Relationships

`utils/fleet/` is the fleet-domain boundary. It interprets generic V2V messages but never creates a UDP socket, simulator actor, or control-loop process.

```text
fleet_types.py
    Shared immutable contracts: member, policy, formation, peer snapshot,
    distributed-estimate configuration, status, and per-step result.
        |
        +--> fleet_base.py          validates and builds a FleetFormation
        +--> fleet_registry.py      owns revisioned, mutable membership snapshots
        +--> fleet_state_machine.py owns one member's fleet lifecycle
        +--> fleet_message.py       encodes/decodes fleet-specific V2V payloads
        +--> fleet_peer_store.py    validates, stores, and expires peer snapshots
        +--> fleet_following_policy.py maps a predecessor snapshot to a controller reference
        +--> fleet_utils/distributed_observer/
        |       base.py, fake.py, luenberger.py, factory.py
        |       create/update only a fleet advisory estimate
        +--> fleet_manager.py       the single runtime-facing fleet interface
        +--> fleet_runtime.py       builds a process-local FleetManager from FleetRuntimeSpec
```

Fleet composition follows this dependency path:

```text
scenario parser
    -> FleetSetup.to_runtime_spec()
    -> VehicleProcessSpec.fleet_spec
    -> core.vehicle_process.build_vehicle_process_runtime()
    -> utils.fleet.build_fleet_manager()
    -> VehicleRuntime(fleet=FleetManager)
```

`extra/platform/` owns the common runner/launcher contract and canonical virtual/CARLA scenario implementations. It does not construct `FleetManager` or a distributed observer. QCar and future ROS 2 resource contexts use the same `VehicleProcessSpec.resources` path. `VehicleRuntime` receives only `FleetManager`, preserving the one-module/one-interface rule. The ground station sees one `PlatformLauncher` lifecycle workflow—prepare, service start/stop, status, and logs—rather than a local-versus-remote choice; target configuration selects the internal transport. This service lifecycle is separate from vehicle runtime commands: a ground-station `START`/`STOP` controls an already-running runtime and must never upload code, open hardware, or implicitly start motion. The operational per-vehicle fleet architecture is documented in [FLEET_ARCHITECTURE.md](FLEET_ARCHITECTURE.md).

### `extra/`: Integration And Applications

| Area | Contents | Responsibility |
|---|---|---|
| `extra/platform/base.py` | Shared resource/process lifecycle | Merge a platform-owned resource context into one `VehicleProcessSpec`, then invoke the platform-neutral runtime with an explicit start policy. |
| `extra/platform/scenario.py` | Common platform scenario parsing | YAML loading, scenario mission parsing, V2V endpoint validation, fleet specification, and process-spec assembly. |
| `extra/platform/carla/` | `CarlaSession`, CARLA scenario parser, process runner | CARLA client/world/actor ownership, synchronous tick coordination, CARLA-specific process lifecycle. |
| `extra/platform/virtual/` | Virtual scenario parser and process runner | Independent deterministic mathematical vehicle processes. No shared virtual session exists. |
| `extra/platform/launcher.py` | Local and remote launcher backends plus canonical scenario CLI | Synchronize local workers or preflight/start remote endpoints; neither backend owns hardware, ROS, or simulator resources. |
| `extra/platform/qcar/` | PAL QCar/LiDAR resource context and process runner | Create/neutralize/terminate the physical QCar. Its LiDAR manager is platform-owned but creates/terminates the PAL scanner only on explicit local-IO demand. |
| `extra/platform/ros2/` | ROS 2 resource context | Initialise/shut down a future `rclpy` node; the future ROS IO adapter receives it through the same resource map. |
| `extra/deployment/` | Bundle policy, SSH/SFTP client, `deployment_type` contracts, target configuration, and deployment CLI | Operator-side release transport only. It stages reviewed bundles and controls a remote process lifecycle without becoming a vehicle runtime, TCP server, or fleet manager. |
| `extra/ground_station/` | fixed-path YAML configuration, `ground_station_type` contracts, core listener/session routing, terminal presentation, and optional live plotting | Operator-side TCP listener and presentation. It routes typed commands and presents bounded received scans without direct vehicle hardware access. |

### Side-Program Configuration

`extra/deployment/` and `extra/ground_station/` are side programs: they run on
the operator machine and compose or communicate with vehicle runtimes, but are
not selected by `core.module_factory`. Only deployment has machine-specific
operator configuration:

```text
extra/<side-program>/
  config/
    templates/                  reviewed examples committed to Git
    local/                      ignored operator-machine settings
  ... program code and entry point ...
```

Deployment additionally has `config/bundles/` for versioned source-selection
policies. Its local `deployment_target` splits logical identity, `ssh`,
`release`, and `preflight`; a `deployment_targets` inventory chooses one bundle
and one or more target files. The selected release command chooses a platform;
the deployment schema does not encode QCar-specific hardware behavior.

Ground station always uses the one versioned YAML at
`extra/ground_station/config/ground_station.yaml`; it has no template, local
copy, or CLI-selected YAML. This is intentionally different from
`config/config_ground_station.yaml`, which configures the *vehicle-side*
outbound TCP bridge. The vehicle bridge must target the operator machine's
selected address and the fixed listener port `5000`, with a compatible frame
limit.

## Runtime Flow

One `VehicleRuntime.step()` follows this order:

```text
ground-station runtime facade drains a bounded command batch
    -> VehicleCommandHandler applies command semantics and returns safety intent
    -> VehicleRuntime resets control or applies an actuator-safe stop when requested
    -> optional simulation tick
    -> IO reads current sensor data
    -> observer updates local estimate
    -> FleetManager drains/publishes its generic V2V messages and returns a fleet decision
    -> planner produces target
    -> state machine checks drive permission and path completion
    -> `ControllerManager` delegates to the selected configured controller when RUNNING
    -> IO applies command
    -> ground-station runtime facade builds and publishes the latest monitoring snapshot
    -> completed RuntimeTelemetry is returned
```

The runtime starts the selected simulation session first, then observer, planner/controller reset, and V2V. Shutdown is safe and idempotent: command zero is applied before dependencies are stopped or closed.

### SDCS Small-Map Planner

`PathPlannerSDCSSmallMap` is a right-hand-traffic, directed roadmap for CARLA's SDCS small-map scene, with node IDs `0` through `10`. Select `planner: sdcs_small_map` and provide `mission.node_sequence`, for example `[0, 2, 4, 6, 0]`; each adjacent pair is joined through the shortest legal directed route. Node poses and curve radii are stored directly in the CARLA project frame in metres and radians. The planner therefore has no texture, pixel, image-origin, or UV-orientation dependency, and does not require the Quanser `hal` package at runtime.

With the custom SDCS CARLA world running on port `2000`, run `python test/test_integration_carla_sdcs_path.py` to follow its test-owned node route. It writes one run under `test/artifacts/integration/carla/sdcs_path/<run-id>/`, with telemetry in `raw/`, a plot in `figures/`, and run metadata in `manifest.json`. The test starts at node `0`; its CARLA spawn is `x=-1.14`, `y=-1.053177`, `yaw=90`, which `IOCarla` converts to the project-frame pose `(-1.14, 1.053177, -pi/2)`.

For CARLA synchronous mode, the simulation session owns `world.tick()`, updates its latest snapshot, then `IOCarla` reads that completed frame. A multi-vehicle CARLA scenario designates exactly one `tick_owner`; other actors wait for completed frames.

### Fleet Command Dispatch And `BUILD_FLEET`

`BUILD_FLEET` is not a vehicle-state-machine transition. It is a lifecycle command handled only by an injected `FleetManager`. The command path is deliberately ordered as follows:

```text
VehicleCommand(BUILD_FLEET)
    -> VehicleCommandHandler verifies target vehicle ID
    -> injected FleetManager handles fleet lifecycle commands
        -> accepted while READY: fleet phase DISABLED -> PREPARED
        -> accepted while RUNNING: fleet phase DISABLED -> BUILDING
        -> rejected: fleet is already PREPARED/BUILDING/ACTIVE
    -> no FleetManager: rejected as fleet_manager_unavailable
```

The acknowledgement `fleet_manager_unavailable` means the vehicle is running but was not launched with a fleet definition. It does **not** mean that a running fleet-enabled vehicle is forbidden from building a fleet.

A `FleetManager` is created only by a scenario runner when the selected scenario has a top-level `fleet:` definition. The normal single-vehicle command, `python -m core.vehicle_main ...`, constructs no fleet manager. To use `build-fleet`, launch a fleet-enabled virtual or CARLA scenario through `extra.platform.launcher`; every intended member must be listed in the scenario's fleet configuration. A READY vehicle accepts the command as `PREPARED`; it discards inbound peer state and does not publish V2V state, update its distributed observer, evaluate peers, or start a timeout until `START` changes the vehicle safety state to `RUNNING` and promotes the fleet to `BUILDING`.

After fleet operation begins, each member enters `BUILDING`. It publishes its V2V estimate and waits for fresh estimates from every expected peer. It becomes `ACTIVE` only when all required peers are fresh before the configured peer timeout. Missing/invalid local estimates, unavailable V2V routes, stale peers, or a timeout fault the fleet and trigger the runtime's safe-stop path.

The permitted combined state vectors are deliberately small. `READY` always writes zero output and may use either the configured or armed manual controller; it may also have a `DISABLED` or `PREPARED` fleet. `RUNNING` may be automatic with a disabled/prepared/building/active fleet, or manual with a disabled fleet or a leader's building/active fleet. A building/active follower must use its fleet-selected controller and cannot retain manual mode. `STOPPED` and `ERROR` always cancel fleet operation, restore the configured controller, and write zero output. `BUILD_FLEET` remains the static, payload-free scenario command. A dynamic ordered-member-vector command needs a separate revisioned protocol: validate the entire ordered vector, deliver the identical revision to every listed local manager, await all acknowledgements, then activate; it must not reuse `BUILD_FLEET`'s payload.

## Fleet-Following Architecture

Fleet following is an application of the existing observer, V2V, controller, and IO contracts. It does not create a second actuator path. The leader continues to use its normal planner/controller pair; an active follower receives a validated predecessor reference and uses a selected fleet controller.

```text
ego observer                         generic V2V transport
    |                                        |
    v                                        v
VehicleStateEstimate                  FleetPeerSnapshot
    |                                        |
    +----------- FleetFollowingPolicy -------+
                            |
                            v
        ControllerFleetBase.compute(estimate, reference, dt)
                            |
                            v
      ControllerFleetLongitudinal or ControllerFleet2D
                            |
                            v
                     ControlInput -> IO adapter
```

### Fleet Ownership Boundaries

| Component | Owns | Does not own |
|---|---|---|
| `utils/v2v/` | Generic envelopes, UDP routing, receive metadata, packet counters | Fleet payload schemas, peer policy, actuator control |
| `utils/fleet/FleetManager` | Formation lifecycle, V2V fleet exchange, peer freshness, fleet publication, and follower reference eligibility | UDP sockets, concrete controller algorithms, IO, vehicle safety transitions |
| `FleetPeerSnapshot` | A validated remote self estimate with sender, sequence, local receive time, and freshness | Sockets, IO, or mutable membership |
| `FleetFollowingPolicy` | Direct-predecessor lookup and conversion of a fresh peer snapshot into `ControllerReference` | Throttle, steering, controller selection, or transport operations |
| `ControllerFleetBase` and subclasses | Convert a valid fleet reference and ego estimate into one bounded `ControlInput` | V2V messages, fleet registry changes, path parsing, or IO |
| `VehicleRuntime` | Ordered control-loop composition, global safety state, controller selection, and the single IO write | Peer-cache details, fleet algorithm internals, or socket ownership |

### Fleet Data Models

Do not use a generic type named `FleetState`: it is ambiguous between an individual vehicle estimate, fleet lifecycle, and a collective estimator result.

| Model | Owner | Meaning | Baseline transport use |
|---|---|---|---|
| `VehicleStateEstimate` | `utils/control/observer` | One vehicle's local pose, heading, speed, and health | Published as `VEHICLE_STATE_ESTIMATE` |
| `FleetPeerSnapshot` | `utils/fleet/` | Validated remote `VehicleStateEstimate` plus local receive/freshness metadata | Constructed after fleet message decoding |
| `FleetStatus` | `utils/fleet/` | Formation ID, revision, source vehicle ID, role/order, phase, and peer-health summary | Local monitoring state by default |
| `DistributedFleetEstimate` | Fleet distributed observer | One vehicle's advisory collective estimate | Local only until a defined message contract needs it |

The initial system publishes only the self estimate. Each `config_fleet.yaml` policy selects a process-local `distributed_observer`; production profiles select `fake`, which preserves the future integration point without affecting follower control. The `luenberger_experimental` profile selects the fixed-gain prototype for offline evaluation only. A distributed observer must not be fused into the ego observer or follower controller until its correlation, delay, and validity rules are defined.

### Topology And Lifecycle

Fleet membership and order belong to a scenario. Reusable communication policy belongs to `config_fleet.yaml`; the scenario's V2V endpoint map still supplies concrete IP addresses and ports.

| Topology | Graph | Suitable use |
|---|---|---|
| `predecessor_chain` | Each member sends to its direct successor | Direct-predecessor following; current default |
| `leader_follower` | Leader connected to every follower | Leader-reference or supervisory policies |
| `loop` | `leader -> follower_1 -> ... -> follower_last -> leader` | Ring-based distributed algorithms |
| `vehicle_vehicle` | Every member connects to every other member | Small-fleet experiments and all-peer algorithms |

For direct-predecessor following, every follower must receive its predecessor estimate. A leader-only star is insufficient for a second follower unless it also includes the predecessor links. `BUILD_FLEET` moves a running member into `BUILDING`; each member publishes self estimates and waits for all required peers. It enters `ACTIVE` only when those estimates are fresh before `peer_timeout_s`. Invalid ego estimates, missing routes, stale peers, or timeout invoke the existing safe-stop path.

### Fleet-Building Tutorial

Fleet identity and fleet lifecycle are separate. The scenario fixes each member's `vehicle_id`, role, and order; `BUILD_FLEET` cannot change any of them. It affects only the local `FleetManager` in the targeted vehicle process.

```text
build-fleet 2
    -> vehicle process 2 only
    -> retain configured role=follower and member_order=1
    -> move its local phase from DISABLED to PREPARED or BUILDING
    -> do not contact or reconfigure vehicle processes 1 or 3
```

For a three-member predecessor chain, the normal dependency is:

```text
vehicle 1: leader, normal path controller, publishes its ego estimate
vehicle 2: follower, receives vehicle 1, publishes its ego estimate
vehicle 3: follower, receives vehicle 2
```

Every member must be `RUNNING` and locally `BUILDING` or `ACTIVE` before the formation can become active. The leader does not become a follower: it keeps its normal planner/controller while publishing V2V fleet data. A follower can use fleet control only after its direct predecessor produces fresh estimates.

`BUILD_FLEET` has two valid entry cases:

| Vehicle safety state when command arrives | Local fleet result | Meaning |
|---|---|---|
| `READY` | `DISABLED -> PREPARED` | Store the build request without publishing or starting a timeout. The next `START` promotes it to `BUILDING`. |
| `RUNNING` | `DISABLED -> BUILDING` | Immediately publish estimates and wait for required peers. |

The current ground-station command targets one vehicle because all core `VehicleCommand` values are local to one vehicle process. Use this safe manual sequence for a static three-vehicle formation:

```text
reset 1
reset 2
reset 3

build-fleet 1
build-fleet 2
build-fleet 3

start 1
start 2
start 3
```

The build commands are deliberately sent while all members are `READY`, where they remain `PREPARED` without a peer timeout. Start all members promptly afterwards. The leader begins publication, then followers receive predecessor estimates and transition from `BUILDING` to `ACTIVE`. If a follower enters `BUILDING` while its predecessor is not publishing, it eventually receives `fleet peer build timeout` and follows the normal safe-stop path.

For normal scenario operation, prefer the launcher with `--build-fleet`: it starts every worker and sends the local build request to every member after startup. A future ground-station group command may fan out the same per-vehicle build/start commands with acknowledgement tracking, but it must not turn `BUILD_FLEET` into a dynamic membership or role-reassignment command.

### Controller Contract And Current 2D Algorithm

Fleet controllers use the ordinary controller contract:

```python
compute(ego_estimate, controller_reference, dt) -> ControlInput
```

When the fleet is active, `FleetFollowingPolicy` maps the direct predecessor snapshot to a `ControllerReference` containing predecessor pose, heading, and speed. A controller never receives V2V messages or fleet lifecycle state.

`ControllerFleetLongitudinal` is limited to straight-road evaluation. `ControllerFleet2D` is the current practical baseline for both throttle and steering. It creates a virtual target behind the predecessor rather than steering toward the predecessor centre:

```text
d_desired = desired_gap_m + time_headway_s * max(ego_speed, 0)
virtual_target = predecessor_position - d_desired * heading_unit_vector

target_speed = clamp(predecessor_speed + gap_gain * (front_distance - d_desired))
throttle = clamp(kp_velocity * (target_speed - ego_speed))
steering = clamp(steering_gain * wrapped_heading_error_to(virtual_target))
```

The controller applies configured throttle, steering, and speed limits. It is a proportional virtual-target baseline, not MPC, CACC, or a string-stability-guaranteed platooning law. It currently has no curvature feedforward, acceleration feedforward, delay compensation, or preview horizon. Those are valid future controller replacements because the fleet reference and IO interfaces remain unchanged.

### Fleet Extension And Validation Guide

1. Add or revise a following algorithm under `utils/control/controller/controller_fleet/`; derive it from `ControllerFleetBase` and preserve the standard `compute()` contract.
2. Add algorithm unit tests for command bounds, spacing response, zero/negative gap, stale input, and 2D geometry.
3. Validate the same policy in a real-time virtual multi-process scenario before CARLA.
4. Repeat the scenario in CARLA with conservative speeds, larger gaps, and controlled spawn positions.
5. Add a distributed observer only as an advisory fleet utility first. Define its message schema, delay handling, and fusion rules before allowing it to affect control.

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

`config/scenarios/*.yaml` describes current operator-facing instances, not reusable module profiles. Every scenario starts with a `simulation_profile`, then lists vehicle-specific values. Automated-test-only equivalents belong under `config/scenarios/test/`:

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

Virtual scenarios explicitly select `simulation_profile: "null"`. Scenario files own vehicle IDs, missions, CARLA spawn transforms, tick ownership, and deployment endpoint overrides. They must not duplicate shared CARLA host, port, timestep, or sensor defaults. The standard CARLA LiDAR rig is a `carla_sync` session sensor because `CarlaSession` creates it; a direct test may add an isolated extra sensor only when it is genuinely test-specific.

## CARLA Boundary

`CarlaSession` owns the CARLA client connection, original world settings, ego/sensor actors, callbacks, synchronous ticking, and cleanup. `IOCarla` reads the session snapshot, adapts it to project contracts, and applies a `ControlInput` to the existing ego actor.

The module boundary uses SI-facing defaults. Current CARLA mapping uses signed forward velocity for `motor_tach`, project-frame IMU yaw rate and acceleration, local project pose, and CARLA simulation elapsed time. The current project-frame conversion is:

```text
project_x = carla_x
project_y = -carla_y
project_yaw = wrap_radians(-radians(carla_yaw_degrees))
```

Project steering radians are converted to CARLA normalized steering; negative throttle maps to brake in the MVP. CARLA sensor/frame mapping remains a platform approximation and should be validated for each selected map and vehicle profile.

## Sensor And Localisation Architecture

`IOBase` is the platform-adapter boundary for vehicle sensing and actuation. It is therefore the public source of normalized IMU, wheel/tachometer, GPS, LiDAR, and camera data for a runtime. The native resource lifecycle remains outside IO: `CarlaSession` owns CARLA sensor actors, a QCar platform context owns a hardware driver it created, and a ROS 2 context owns its node/subscriptions. Each supplies its already-created resource to the corresponding IO adapter.

### One Common LiDAR Adapter Boundary

LiDAR must have one public acquisition API, rather than one runtime path for
CARLA and another for QCar. Its data plane runs where the sensor runs: inside
the local vehicle runtime for QCar/Limo or inside the local simulator process
for CARLA. `core/vehicle_types.py` owns the dependency-free normalized
`LaserScanSample` input and `PoseMeasurement` output contracts. `IOBase` owns
the bounded local queue, private publish operation, one-time drain, and queue
diagnostics. A source descriptor will later add source ID,
`base_link`/sensor frames, static extrinsics, expected rate, and freshness
policy. IO never creates or terminates a native hardware resource.

`LaserScanSample` retains ROS 2 `sensor_msgs/msg/LaserScan` field semantics
without a ROS import, but it is a vehicle data contract—not an IO-specific or
localisation-specific type. The `utils/localization/scan_matching/` algorithms
depend only on the core contracts and produce `PoseMeasurement`; they do not
know whether a scan came from CARLA, PAL, or ROS 2.

The implemented QCar path follows that boundary: the physical platform
context owns the `QCarLidarResourceManager`, while `IOQCar2` owns only its
local polling thread and normalized queue publication. With the normal
disabled profile, the manager does **not** create PAL LiDAR at `targets-start`;
the typed `lidar start <id>` command asks local IO to acquire it and `lidar
stop <id>` stops the worker then releases it. An explicit stationary capture
may still use a short-lived direct PAL resource. Its operational profile
remains disabled by default until the stationary PAL probe validates that
vehicle's frame and range convention.

```text
QCar LiDAR ──> local IOQCar2 polling worker ─┐
CARLA callback ─> local IOCarla conversion ──┼─> IOBase local scan queue
ROS 2 callback ─> local IORos2 conversion ───┘              │
                                                            v
                                      IOBase.drain_lidar_scans() (once only)
                                                            │
                                                            v
                               local scan matching ─> PoseMeasurement ─> runtime / control
                                                            │
                                                            └─> ground station: health + pose only
```

The selected IO adapter performs its small native conversion: CARLA axis and
point-cloud projection, PAL angle/range conversion, or ROS-message copying.
`IOBase` owns the common queue/drain semantics and diagnostics; the runtime
does not import platform code.  `extra/platform/` remains a lifecycle boundary
only: it may create a CARLA actor, PAL client, or ROS node and pass the raw
handle to local IO, but it must not hold the normalized scan buffer or move
the continuous scan stream through the ground station.  Raw scans leave the
vehicle only for an explicit bounded capture artifact or diagnostic request.
A PAL service that already emits a calibrated map pose is **not** a LiDAR scan
source: it is a separate optional `PoseMeasurement` source and must share the
correction path with scan matching and GPS.

The Qt/pyqtgraph sensor viewer is an optional ground-station presentation
component. `lidar start <id>` is a typed command on the existing registered
vehicle TCP connection. The vehicle runtime enables its local IO adapter,
forwards only the newest normalized scan at the configured diagnostic rate,
and the server stores it for the viewer. `lidar stop <id>` stops local
acquisition, releases a managed QCar PAL scanner, and stops forwarding;
closing the viewer or terminal sends the same typed stop command. The viewer
refresh loop is independent of the lower-rate terminal dashboard. This is not
a driving command and cannot start a route or write an actuator. It is valid
in `READY`, `RUNNING`, and `STOPPED`, allowing a stationary diagnostic without
restarting vehicle motion.

The producer remains local: CARLA converts its session-owned callbacks in
`IOCarla`, while QCar polls the PAL client in `IOQCar2`. The runtime reads the
latest immutable scan without draining the local localisation queue, then the
ground-station facade rate-limits the optional display copy. Thus CARLA, QCar,
and a future ROS 2 adapter share the command/viewer flow without SSH, UDP, or
a second physical-hardware owner. The standalone UDP capture viewer remains a
low-level troubleshooting tool only. Its camera pane is reserved for the
future `CameraFrame` contract.

The required propagation inputs are IMU and wheel/tachometer data. GPS and LiDAR localisation are optional positioning sources. A camera is a general optional sensor: its normalized frames may feed perception/detection, recording, or a selected camera-localisation algorithm. Only the latter produces an optional map-frame `PoseMeasurement`. A source being disabled, unavailable, or stale must not prevent IMU/wheel propagation; it simply contributes no correction. LiDAR has one extra platform-neutral stage because its normalized `LaserScanSample` is processed by a `ScanMatchAlgorithm` before it becomes a map-frame `PoseMeasurement`.

```text
native GPS ─────────────────────────────────────> IO adapter ──> PoseMeasurement
native LiDAR ──> local IO adapter / `IOBase` queue ──> LaserScanSample ──> scan matching ──> PoseMeasurement
native camera ──────────────────────────────────> IO adapter ──> CameraFrame ──> perception/detection
                                                                  └─> camera localiser ──> PoseMeasurement
IMU + wheel/tachometer ─────────────────────────> IO adapter ──> observer propagation
                                                                       │
optional, valid PoseMeasurement corrections ───────────────────────────┘
```

Sensors do not share one required frequency. IMU/wheel data is normally frequent, while GPS, LiDAR scans, camera frames, and camera-derived poses arrive independently and may be delayed. Every sample and derived measurement must retain its acquisition timestamp, validity, source identity, frame, and sequence identity. The target IO contract keeps the latest required propagation inputs and bounded queues of optional scans, frames, and pose measurements; it must not repeat a previous correction merely because the control loop is faster.

At a control iteration, the observer first propagates with the newest required IMU/wheel data, then processes each newly received optional correction once in timestamp order. It rejects a correction older than its configured freshness limit and records diagnostics for stale, dropped, invalid, or out-of-order inputs. The exact delayed-measurement policy—discard, bounded historical replay, or another explicitly validated method—must be selected before an operational correction is enabled.

Configuration ownership follows the same split:

| Configuration | Owns |
|---|---|
| `config_io.yaml` | The selected IO adapter's required IMU/wheel inputs and optional GPS, LiDAR, or camera acquisition adapters, their source/topic/device settings, frame IDs, expected rates, bounded queue limits, and static extrinsics. |
| `config_simulation.yaml` | CARLA-only sensor actor setup: blueprint, transform, attributes, and scan projection settings. `IOCarla` references the session-owned result rather than creating a second sensor. |
| `config_localization.yaml` | Platform-neutral scan-matching and camera-localisation algorithm parameters, map/reference data, fusion policy, source enablement, measurement freshness, covariance, and output frame. |

This is a target extension. The current runtime still performs one synchronous IO/observer update and uses GPS through the current sensor contract; it does not yet expose the multi-rate optional-source/fusion interface described above. No vehicle profile may select an unconsumed LiDAR/GPS/camera source before its adapter, validation, and runtime integration land together.

## How To Use The Project

### Run Vehicles

Run all commands below from `qcar_refactor/`. `core.vehicle_main` starts exactly one vehicle actor; select the platform through `--vehicle-config`.

#### One Virtual Vehicle

The virtual profile has no external simulator dependency and is the quickest way to exercise the full control loop:

```powershell
python -m core.vehicle_main --vehicle-config config_vehicle_virtual.yaml --vehicle-id 1 --cycles 200
```

Omit `--cycles` to run until `Ctrl+C`. `--vehicle-id` is optional for one vehicle, but supply a unique ID whenever another vehicle is active, especially when using V2V.

#### One CARLA Vehicle

Start a compatible CARLA server first, with the host and port selected by `carla_sync` in `config/config_simulation.yaml` (the default is `127.0.0.1:2000`). Then, in a separate terminal with the CARLA Python API available, run:

```powershell
python -m core.vehicle_main --vehicle-config config_vehicle_carla.yaml --vehicle-id 1 --cycles 800
```

`config_vehicle_carla.yaml` is the safe, reusable CARLA composition: it selects CARLA IO, an SDCS-capable planner, and the shared `carla_sync` session profile but no active route. `carla_sync` includes the standard horizontal 2-D LiDAR rig; a scenario or direct test supplies its map-specific spawn, mission, and observer initial pose. It creates and owns one ego vehicle and its sensors, then restores CARLA world settings and destroys those actors on shutdown. Do not start a second standalone CARLA vehicle with this command in the same synchronous world; use a CARLA scenario instead so exactly one process owns world ticks.

#### Multiple Vehicles or a Scenario

A scenario YAML under `config/scenarios/` defines the vehicle IDs, per-vehicle missions, V2V endpoints, and platform-specific values. Use the scenario launcher to start every selected worker, wait for all workers to become ready, and then release them together:

```powershell
# Two virtual vehicles from config/scenarios/test/virtual_two_vehicle.yaml.
python -m extra.platform.launcher --platform virtual --setup-file config/scenarios/test/virtual_two_vehicle.yaml

# Two CARLA vehicles from config/scenarios/test/carla_two_vehicle.yaml.
# Start the CARLA server first, then run this command.
python -m extra.platform.launcher --platform carla --setup-file config/scenarios/test/carla_two_vehicle.yaml
```

For interactive fleet operation through the ground-station CLI, use the dedicated three-vehicle scenarios. Start the CARLA server before the CARLA command and start the ground-station side program before either command:

```powershell
# Three deterministic virtual vehicles. --realtime keeps simulation aligned
# with the operator terminal; --build-fleet starts fleet lifecycle after startup.
python -m extra.platform.launcher --platform virtual --realtime --setup-file config/scenarios/virtual_three_vehicle_cli_fleet.yaml --build-fleet

# Three CARLA vehicles with the same leader/follower command workflow.
python -m extra.platform.launcher --platform carla --setup-file config/scenarios/carla_three_vehicle_cli_fleet.yaml --build-fleet
```

Both commands run until `Ctrl+C`, just like the single-vehicle entry point. `Ctrl+C` creates a shared shutdown signal that every worker observes in its control loop, allowing each runtime to write its safe zero command and release platform resources before the launcher exits; a non-responsive worker is force-stopped only after a five-second grace period. Add `--cycles <count>` for a bounded run. For virtual scenarios, add `--realtime` whenever an operator or wall-clock V2V communication is involved; without it, the virtual workers run as quickly as possible and can finish a short route before a CLI command is processed. Add `--build-fleet` to send `BUILD_FLEET` after every worker enters the running state.

The launcher selects the matching worker module and manages its temporary readiness, start, and shutdown files. The files under `test/` still validate this behaviour, but are not the normal operational interface. For CARLA, the scenario must have exactly one `tick_owner: true`; all other workers wait for that process's completed world frames.

QCar hardware is started by `extra.platform.qcar.process_runner`, not by the virtual/CARLA launcher. Its PAL resource context sends neutral output before runtime construction and on every exit; the process remains `READY` until the configured ground-station path issues a valid `START` command.

### Run The Ground Station

Run the vehicle listener without an operator UI when a separate headless server process is required:

```powershell
python -m extra.ground_station server
```

`server_main` owns only the TCP listener and vehicle session registry, so it can run independently of any terminal frontend. It accepts vehicle registration, monitoring, and acknowledgement traffic but does not provide an operator command surface.

For normal interactive operation, run the combined listener and operator terminal instead:

```powershell
python -m extra.ground_station terminal
```

The interactive terminal uses `prompt_toolkit` and has separate vehicle-status, command/acknowledgement, and command-input panes. Install it in the ground-station environment with `python -m pip install prompt_toolkit`. The display refreshes without replacing typed input. Use `--no-dashboard` for plain log-only operation. Do not start `server` and the combined `terminal` program on the same host/port: the terminal already starts its own listener.

Start a vehicle with the TCP bridge in another terminal. The default `tcp_client` profile uses `127.0.0.1:5000`; override it for a remote ground-station machine:

```powershell
python -m core.vehicle_main --vehicle-config config_vehicle_virtual.yaml
```

The vehicle registers before normal messages are exchanged. The dashboard displays only received snapshots. In the ground-station prompt, enter commands such as `start 0`, `set-velocity 0 0.6`, `build-fleet 0`, and `emergency-stop 0`; the prompt waits for the vehicle `COMMAND_ACK` and the dashboard keeps the latest result visible.

The current TCP bridge is for localhost or a trusted laboratory subnet. Before using it on a physical-vehicle network outside that boundary, add authenticated registration, a server/vehicle allow-list, and encryption or an isolated VPN.

### Vehicle Command Reference

All vehicle commands target one non-negative `vehicle_id`. The terminal converts these commands into a typed `VehicleCommand`; a successfully delivered TCP message is not a successful command until the vehicle returns a `COMMAND_ACK` with outcome `applied`.

| Terminal command | Typed command | Arguments and effect |
|---|---|---|
| `start <id>` | `START` | Enter `RUNNING` when the current state permits it. |
| `stop <id>` | `STOP` | Stop the vehicle and request a safe zero actuator command. |
| `emergency-stop <id>` | `EMERGENCY_STOP` | Immediately enter the emergency-stop state and request a safe zero actuator command. |
| `reset <id>` | `RESET` | Reset an emergency-stopped or error runtime when the state machine permits it. |
| `set-velocity <id> <m/s>` | `SET_VELOCITY` | Set a finite, non-negative planner target velocity. |
| `set-path <id> <csv-path>` | `SET_PATH` | Load a non-empty planner path file. The path must be accepted by the selected planner. |
| `enable-sdcs-map <id> <0\|1\|2\|inf> <node> <node> [...]` | `ENABLE_SDCS_MAP` | Select an SDCS route for a READY/RUNNING single vehicle or fleet leader. `0` leaves the route open; `1` returns once to the first node; `2` completes two circuits; `inf` repeats until disabled. Followers reject this command. |
| `disable-sdcs-map <id>` | `DISABLE_SDCS_MAP` | Restore the configured planner. Outside a fleet it safely stops; an active leader continues its fleet using the restored route. |
| `build-fleet <id>` | `BUILD_FLEET` | Prepare the selected static formation in `READY`, or begin its lifecycle in `RUNNING`. |
| `cancel-fleet <id>` | `CANCEL_FLEET` | Cancel fleet operation and safely stop the vehicle when required by the fleet manager. |
| `enable-manual <id>` | `ENABLE_MANUAL` | Arm/select the manual controller in `READY` or `RUNNING`. Input is accepted only in `RUNNING`; a building/active fleet follower cannot use manual control, while its leader can. |
| `disable-manual <id>` | `DISABLE_MANUAL` | Restore the configured controller. |
| `manual <id> <throttle> <steering-rad>` | `MANUAL_INPUT` | Update manual input. Throttle must be finite and in `[-1, 1]`; steering must be finite. Manual mode must already be enabled. |
| `manual-drive <id>` | — | Starts the interactive keyboard loop; it first sends `ENABLE_MANUAL`, then streams coalesced `MANUAL_INPUT` values. |

`help`, `list`, `status <id>`, `quit`, and `exit` are ground-station terminal actions, not vehicle commands. At the typed-command boundary, `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, and `CANCEL_FLEET` may also include an optional non-empty `reason` payload. Unknown commands or unexpected payload fields are rejected.

### Ground-Station Command Helper

Use the following operator sequences as command-level smoke tests. Check the dashboard acknowledgement and the `Runtime`, `Fleet phase/role`, and `Mode` columns after every structural command; TCP delivery alone does not prove that the vehicle accepted it.

| Goal | Commands | Expected result |
|---|---|---|
| Run the three-vehicle CARLA fleet | Start the ground-station CLI, then launch `carla_three_vehicle_cli_fleet.yaml` with `--build-fleet`. | The launcher starts every vehicle and requests fleet build. The dashboard progresses from `building/<role>` to `active/<role>`. Do not send `build-fleet` again. |
| Run the three-vehicle virtual fleet | Start the ground-station CLI, then launch `virtual_three_vehicle_cli_fleet.yaml` with `--realtime --build-fleet`. | The same fleet lifecycle is visible without CARLA. `--realtime` is required for interactive commands; without it, virtual simulation advances faster than the operator terminal. |
| Stop one vehicle safely | `stop <id>` | The vehicle becomes `STOPPED`, writes zero actuator input, and leaves fleet operation. Its peers will eventually stop if their required predecessor becomes unavailable. |
| Resume a stopped automatic vehicle | `reset <id>` then `start <id>` | The vehicle returns to `READY`, then `RUNNING`, using its configured controller. Rebuild the static fleet for every intended member before expecting fleet operation. |
| Arm manual control before driving | `reset <id>` then `enable-manual <id>` then `start <id>` | Manual mode may be selected only after reset returns the vehicle to `READY`; it remains zero-output until `RUNNING`. |
| Drive manually | `enable-manual <id>`, `start <id>` when needed, then `manual-drive <id>` or `manual <id> <throttle> <steering-rad>` | `Mode` becomes `manual`; input is accepted only while `RUNNING`. A building/active follower rejects manual mode. |
| Return from manual to automatic | `disable-manual <id>` | The configured controller is restored. It does not itself start a stopped vehicle. |
| Run an SDCS map route | `enable-sdcs-map <leader-id> inf 0 2 4 6 10` | Use only on a standalone vehicle or fleet leader. The leader pauses for one safe zero command, then followers continue from its published motion. |
| Restore the configured route | `disable-sdcs-map <leader-id>` | The active leader resumes the configured route; a standalone vehicle stops safely. |
| Cancel a formation | `cancel-fleet <id>` for each intended member | Each commanded member safely stops and transitions its local fleet manager to `disabled`. |

The scenario launcher starts vehicles automatically. Therefore, when it is used with `--build-fleet`, the normal fleet command is the launcher invocation, not a second set of `start` and `build-fleet` commands in the CLI. CLI commands remain useful for monitoring, emergency stop, cancellation, and controlled restart tests.

### Manual Virtual Driving

The selected controller profile owns an optional `manual` subsection. `VehicleRuntime` builds `ControllerManual` from it and temporarily selects that normal `ControllerBase` implementation only while manual mode is enabled. Manual control can be armed in `READY`, but input and non-zero actuation remain unavailable until `RUNNING`. A building or active fleet follower cannot use manual control; the fleet leader can. In the ground-station terminal, use either a one-shot command or the Windows keyboard loop:

```text
enable-manual 0
manual 0 0.20 0.10
manual-drive 0
```

`manual-drive` waits for the `ENABLE_MANUAL` acknowledgement, then sends `MANUAL_INPUT` at 20 Hz. On Windows, it polls the physical arrow-key state independently, so hold Up/Down together with Left/Right to control throttle and steering simultaneously. `Space` zeros the requested input and `Q` sends `STOP` and exits. Releasing either arrow returns only that axis to zero. `ControllerManual` clips input to its configured limits and outputs a zero command when input is older than its monotonic `command_timeout_s`; it does not use operator wall-clock time. `MANUAL_INPUT` is coalesced to its latest value and does not produce a per-input acknowledgement, while enable/disable/stop commands remain FIFO and acknowledged. Additional lazy profiles belong under `controller.runtime_profiles`; a fleet policy may request one through `follower_controller_profile` when an active follower needs a fleet controller.

### Run A Scenario Worker

Scenario workers are normally launched by an integration parent process. A worker receives a scenario path, vehicle ID, cycle count, ready marker, and start marker. CARLA and virtual process runners share this protocol; CARLA adds session/tick ownership internally.

```powershell
python -m extra.platform.launcher `
  --platform carla `
  --setup-file config/scenarios/carla_three_vehicle_cli_fleet.yaml `
  --build-fleet
```

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
4. Keep client/session startup outside the IO adapter. Add an `extra/platform/<platform>/` resource context only when the backend owns external resources; it supplies those resources through `VehicleProcessSpec.resources`.
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

### Extend The Ground Station

1. Keep `VehicleCommand` and `CommandResult` in `core/`, and keep `MonitoringSnapshot` in `utils/ground_station/`; do not introduce GUI-specific command types.
2. Keep TCP framing and reconnecting vehicle transport behind `GroundStationBridgeBase` in `utils/ground_station/`.
3. Keep vehicle command semantics in `VehicleCommandHandler`: it is the only command dispatcher for the vehicle state machine, planner, and injected `FleetManager`.
4. Keep queue draining, command acknowledgement publication, and monitoring construction in `GroundStationRuntimeFacade`; `VehicleRuntime` only supplies its public state and the safety callback.
5. `extra/ground_station/` owns listener/session registry, `GroundStationCommandHandler`, terminal presentation, and later Qt/PySide presentation. A presentation must use that handler and the same typed server route as the terminal.
6. A ground-station presentation may enqueue commands and display received snapshots, but it must not call IO, controllers, V2V, fleet internals, or `VehicleRuntime` directly.
7. Manual operation must remain a vehicle-side control source. The presentation sends `ENABLE_MANUAL`, `DISABLE_MANUAL`, and coalesced `MANUAL_INPUT` commands; it must not send raw actuator calls or bypass the manual-input timeout.

## Verification Rules

- Unit tests use fake hardware, fake CARLA resources, and temporary files when validating error paths.
- Multi-process integration tests use canonical `config/scenarios/*.yaml` files and subprocess workers.
- Virtual integration is the first closed-loop validation stage.
- CARLA integration follows only with a ready local server.
- QLabs remains deferred while its license is unavailable.
- Physical QCar validation is last and starts at low throttle.
