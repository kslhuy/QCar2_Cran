# QCar Refactor Plan

This plan is based on `refs/qcar_origin/vehicle_logic.py` and the current refactor folder structure. The goal is to make the project easier to read, test, and modify while keeping the useful patterns from the reference implementation.

- Basic GUI
- Controller
- Observer
- Path planner
- V2V communication
- A simple state machine

Keep the same state-machine idea from the current project, but make it smaller and more standard: one small module owns the vehicle mode, and the other modules stay independent.

Everything else should either stay in `refs/` as reference code or be connected later through clear optional modules.

> Implementation tracking is centralized in `docs/REFACTOR_MASTER_TODO.md`. This document provides architecture, migration rationale, and detailed reference material.

## Project Overview

The refactor is currently between the reusable-module phase and the runtime-integration phase. The control pipeline is no longer only a proposal: shared dataclasses, the state machine, base and QCar IO, observer, static planner, simple controller, UDP V2V, and focused tests are present. The central runtime, config loader/factories, GUI bridge, fleet layer, CARLA adapter, and ROS adapters are not complete.

Current status:

| Area | Status | Evidence | Next decision |
|---|---|---|---|
| Shared contracts | Implemented, needs clarification | `core/types.py` defines sensor, estimate, command, planner, GUI, and V2V dataclasses | Document units, coordinate frames, and timestamp domains |
| State machine | Implemented as a safety supervisor | `core/vehicle_state_machine.py` enforces that only `RUNNING` may drive; it does not yet reproduce legacy path, platoon, taxi, manual, or calibration modes | Keep safety state separate and migrate operation modes behind runtime strategies |
| Vehicle IO | Partially implemented | `IOBase`, `IONull`, and `IOQCar2` exist; CARLA and ROS adapters are stubs | Keep simulation sessions outside IO, then add backends |
| Observer | Implemented for the first milestone | EKF implementation and tests exist | Align the base type annotation and documentation with `SensorData` |
| Planner/controller | Implemented for the first milestone | Static waypoint planner and simple controller have focused tests | Integrate them through the runtime coordinator |
| V2V | Implemented for the first milestone | Null and UDP implementations include localhost exchange and cleanup tests | Integrate without allowing V2V to control actuators |
| GUI | Not implemented | No refactored GUI bridge exists | Build a queue-based null bridge before a visual application |
| Runtime/config | Not implemented | `vehicle_config.py`, `vehicle_logic.py`, and `vehicle_main.py` are empty | This is the next critical path |
| Fleet | Not implemented | `utils/fleet/` contains empty placeholders | Keep it out of the first single-vehicle runtime |
| Verification | Good module coverage, no full runtime yet | 91 selected unit/integration tests pass; the state script is not discovered as tests | Add config, runtime lifecycle, and safe-shutdown tests |

## Problems And Benefits

| Current problem | Planned change | Benefit | Risk or cost |
|---|---|---|---|
| Legacy `VehicleLogic` owns lifecycle, control, communication, and research features | Keep a small coordinator and move behavior behind module interfaces | Individual modules can be tested and replaced independently | More explicit wiring and lifecycle code is required |
| Runtime/config files are empty while utility modules already exist | Implement config composition and factories before adding more adapters | Creates one supported construction path and prevents backend-specific branching | Config validation must be designed carefully |
| Units, frames, and timestamp domains are implicit | Define a minimum SI-facing boundary and let adapters convert when possible | QCar, CARLA, ROS, observers, and controllers receive predictable default units without pretending that the platforms are physically identical | Exact actuator and sensor equivalence remains future calibration work |
| CARLA simulation time differs from `IOBase` wall-time polling | Refresh `IOCarla` once per synchronous simulation tick | CARLA frame time is preserved without changing QCar IO behavior | Runtime must keep exactly one tick owner |
| Optional backends can fail during package import | Lazy-load backend dependencies in factories or concrete constructors | Core tests and QCar runs do not require CARLA or ROS installations | Import errors move to backend selection time and need clear messages |
| Shutdown ownership is not standardized | Give the runtime/session one clear owner for every external resource | Exceptions and repeated shutdown calls leave the vehicle in a predictable safe state | Each adapter must implement an idempotent lifecycle |
| State-machine tests are script assertions | Convert them to `unittest.TestCase` | Test discovery and CI can prove safety transitions | Small test maintenance cost |
| The legacy state machine mixes safety transitions with control and feature implementations | Keep the current safety supervisor and move path/manual/platoon/taxi/calibration behavior to operation strategies or services | Original behavior can be migrated without rebuilding the legacy `VehicleLogic` coupling | Full legacy compatibility becomes incremental rather than immediate |
| GUI and fleet features could expand the scope before one runtime works | Finish a headless single-vehicle runtime first | Produces an executable milestone sooner and reduces debugging variables | Advanced workflows arrive later |
| Simulator startup and vehicle IO could become coupled | Use a separate simulation session and a narrow vehicle IO adapter | Simulation lifecycle stays replaceable and testable | Requires a small session interface in runtime orchestration |

The detailed steps below describe the intended module boundaries. The status-aware checklist under **Suggested Work Order** is the source of truth for implementation order.

## Legacy Reference Run-Loop

The legacy `VehicleLogic.run()` is the reference run cycle. Each loop schedules these modules:

1. Safety and timing
   - `WatchdogTimer.reset()`
   - `PerformanceMonitor.log_loop_time()`
   - Controller/observer/command/V2V rate limiters

2. Sensor and observer path
   - `_update_sensor_data()`
   - `VehicleObserver.update_sensor_data(self.qcar)`
   - `_observer_update()`
   - `VehicleObserver.update_observer(dt, last_steering, last_u)`
   - Optional data feeds inside observer update:
     - Online SysID ZMQ
     - Online calibration ZMQ
     - Robust KalmanNet dataset recorder
     - Scope streamer

3. Control path
   - `_control_logic_update()`
   - `VehicleStateMachine.update(dt, sensor_data)`
   - State handlers under `StateMachine/`
   - Controller modules reached indirectly through state handlers and `ControllerManager`
   - Hardware/simulation actuator write through `self.qcar.write(throttle, steering)`

4. Ground station and basic GUI communication
   - `_send_telemetry_to_ground_station()`
   - `_broadcast_periodic_status()`
   - `_process_queued_commands()`
   - `CommandHandler.process_command()`
   - `GroundStationClient` is referenced but its import is commented out in the current refactor file, so the current startup path is not self-contained.

5. V2V communication
   - `_broadcast_v2v_state()`
   - `V2VManager.update_broadcast()`
   - `V2VManager.update_vehicle_observer(self.vehicle_observer)`
   - Optional `V2VAttackInjector`
   - V2V status callbacks and Ground Station status reporting

6. Extra systems still coupled to the runtime object
   - Platoon controller
   - YOLO/perception references
   - Taxi mode
   - Calibration modes
   - Online SysID
   - Online calibration
   - Robust KalmanNet recording
   - Scope streaming
   - Advanced state-machine modes

## Main Problems To Fix

1. `VehicleLogic` is doing too many jobs: lifecycle, sensors, observer, state machine, controllers, telemetry, GUI commands, V2V, calibration, logging, and optional research tools.
2. Some optional systems are only partially disabled. For example, `yolo_manager` initialization is commented out, but `_control_logic_update()` still reads `self.yolo_manager`.
3. Ground Station communication is referenced from the loop, but `GroundStationClient` import is commented out in `vehicle_logic.py`.
4. The state machine is useful, but state handlers should delegate IO, observer, planner, controller, V2V, and GUI details to separate utilities.
5. Core runtime dependencies should stay separated from research/experiment dependencies, so small changes do not break unrelated systems.

## Target Refactored Architecture

Create a small runtime coordinator and make each active module expose a simple interface.

Current standard project locations:

- `qcar_refactor/core/types.py`
- `qcar_refactor/core/vehicle_state_machine.py`
- `qcar_refactor/core/vehicle_config.py`
- `qcar_refactor/core/vehicle_logic.py`
- `qcar_refactor/core/vehicle_main.py`
- `qcar_refactor/config/config_vehicle.yaml`
- `qcar_refactor/config/config_model.yaml`, `config_observer.yaml`, `config_planner.yaml`, and `config_controller.yaml`
- `qcar_refactor/config/config_io.yaml`
- `qcar_refactor/utils/io/`
- `qcar_refactor/utils/control/observer/`
- `qcar_refactor/utils/control/path_planner/`
- `qcar_refactor/utils/control/controller/`
- `qcar_refactor/utils/v2v/`
- `qcar_refactor/utils/ground_station/` for vehicle-side command and telemetry bridges
- `qcar_refactor/extra/ground_station/` for executable visual ground-station applications

The refactored loop should be:

```text
while running:
    read vehicle sensors
    update observer
    receive/process V2V data
    update state machine
    update path planner target
    compute controller command
    write command to vehicle
    publish telemetry to GUI
    process GUI commands
    broadcast local state over V2V
    sleep to maintain loop rate
```

## Step-By-Step Implementation Plan

## Current Step 1-8 Review

This review is based on the current refactor files under `qcar_refactor` as of this plan update.

### What Looks Good

- Step 4 state machine exists as `core/vehicle_state_machine.py`.
- Step 5 vehicle IO exists under `utils/io/`.
- Step 6 observer exists under `utils/control/observer/`.
- Step 7 path planner exists under `utils/control/path_planner/`.
- Step 8 controller exists under `utils/control/controller/`.
- Step 9 V2V exists under `utils/v2v/` with null and UDP implementations.
- The integration test `test/test_integration_control_loop.py` passes with virtual IO, EKF, static waypoint planner, and simple path controller.

### Problems To Fix Before Runtime/GUI

1. Project naming is standardized to the current folder structure.
   - Shared dataclasses live in `core/types.py`.
   - Runtime/state-machine files live in `core/`.
   - Reusable modules live in `utils/`.
   - IO lives in `utils/io/`.
   - Observer, path planner, and controller live under `utils/control/`.
   - V2V lives in `utils/v2v/`.
   - Ground-station bridge code belongs in `utils/ground_station/`; visual application code belongs in `extra/ground_station/`.

2. The V2V state type should be `V2VState`.
   - V2V is both a broadcast mechanism and vehicle-to-vehicle communication.
   - Use `V2VState` as the standard type name.
   - Keep `VBoradcastState` only as a temporary backward-compatible alias if older code imports it.

3. Config should stay separated by purpose.
   - No `minimal_config.yaml` is needed.
   - Runtime-level config belongs in `config/config_vehicle.yaml`.
   - Utility-level config files use the `config_<module>.yaml` style, for example `config_observer.yaml` and `config_io.yaml`.
   - Do not mix runtime config and utility config without a small loader that makes the ownership clear.

4. Keep the reference state-machine pattern, but separate it better from utilities.
   - The reference state machine is well-functioning and should remain the design pattern.
   - The refactor should make state handlers easier to read and modify by moving IO, observer, planner, controller, V2V, and GUI details into utility modules.
   - State machine code should decide mode and transitions; it should not implement controller math, sensor polling, path loading, or V2V protocol details.
   - `test/unit_test_state_machine.py` is currently a script with assertions, so `python -m unittest test.unit_test_state_machine` reports no discovered tests. Convert it to `unittest.TestCase` before relying on automated test runs.

5. EKF acceleration semantics are defined as horizontal acceleration magnitude.
   - `VehicleStateEstimate.acceleration` from EKF means `sqrt(ax**2 + ay**2)`.
   - The z-axis is ignored because it mostly contains gravity.
   - Unit tests should check this convention directly.

6. `NullVehicleIO` should match the base class contract more strictly.
   - `BaseVehicleIO._poll_sensors()` and `_poll_gps()` expect subclasses to write directly into `_sensor_data_cache`.
   - `NullVehicleIO` currently returns dictionaries instead.
   - It works because the cache starts with safe defaults, but it is better for learning and consistency to mutate the cache in those methods.

7. Path planner and controller are good enough to continue.
   - `StaticWaypointPlanner` and `SimplePathController` have focused unit tests.
   - `test/test_integration_control_loop.py` passes, so the step 6-8 chain works at a useful first level.

### Do Before Runtime Integration

- [x] Add `V2VState` in `core/types.py`.
- [x] Use current project naming: `core/types.py`, `utils/io`, `utils/control`, `utils/v2v`.
- [x] Remove the `minimal_config.yaml` direction from the plan.
- [ ] Convert `unit_test_state_machine.py` to a real `unittest.TestCase`.
- [x] Define EKF acceleration as `sqrt(ax**2 + ay**2)`.
- [x] Make `NullVehicleIO` mutate `_sensor_data_cache`.
- [ ] Define units, coordinate frames, steering convention, and timestamp domains in `core/types.py` or a contract document.
- [ ] Separate IO rate-limit clocks from sensor sample timestamps.
- [ ] Align `ObserverBase.update()` with the `SensorData` dataclass contract.
- [ ] Make optional CARLA/ROS dependencies lazy imports.

### Step 1: Freeze The Current Runtime

Do not delete the current `vehicle_logic.py` immediately. Keep it as the legacy reference until the refactored runtime can start and stop cleanly.

Actions:

- Add or complete `core/vehicle_logic.py` instead of rewriting the legacy reference first.
- Keep `vehicle_main.py` working for the legacy path.
- Add a refactored launcher or launcher argument later, after the refactored runtime is tested.

Acceptance check:

- Existing files remain importable.
- No optional research folder needs to be edited in this step.

### Step 2: Define Shared Data Contracts

Use `core/types.py` for plain dataclasses. Use these everywhere in the refactored runtime.

Required types:

- `VehicleStateEstimate`
  - `timestamp`
  - `x`
  - `y`
  - `theta`
  - `velocity`
  - `acceleration`
  - `gps_valid`

- `ControlCommand`
  - `throttle`
  - `steering`
  - `target_velocity`
  - `source`

- `PlannerTarget`
  - `target_x`
  - `target_y`
  - `target_theta`
  - `target_velocity`
  - `is_finished`

- `GuiCommand`
  - `command`
  - `payload`

- `V2VState`
  - `vehicle_id`
  - `timestamp`
  - `x`
  - `y`
  - `theta`
  - `velocity`

Acceptance check:

- Refactored modules exchange only these dataclasses or dictionaries converted at module boundaries.

### Step 3: Keep Config Ownership Clear

Do not create `minimal_config.yaml`. Keep config separated by ownership:

- Runtime config: `config/config_vehicle.yaml`.
- Module utility config: `config/config_<module>.yaml`, for example `config/config_controller.yaml` and `config/config_io.yaml`.

The runtime may load multiple config files, but the loader should make it obvious which module owns which settings.

Suggested fields:

```yaml
vehicle:
  car_id: 0
  vehicle_type: Qcar
  max_throttle: 0.10
  max_steering: 0.48

timing:
  loop_rate_hz: 100
  observer_rate_hz: 100
  controller_rate_hz: 100
  ground_station_rate_hz: 10
  v2v_rate_hz: 25

control:
  target_velocity: 0.6
  longitudinal: pid
  lateral: pure_pursuit

path:
  default_path: path_rich_output/pp_waypoints.csv

ground_station:
  enabled: true
  host: 0.0.0.0
  base_port: 5000

v2v:
  enabled: true
  base_port: 8000
```

Acceptance check:

- The runtime can load its runtime config and pass each utility only the config section it owns.
- Control utilities do not depend on IO config.
- IO utilities do not depend on control config.

### Step 4: Refactor The State Machine

Keep the state-machine pattern from the reference project. It is important for safety and for beginner-friendly debugging because it makes clear when the car is allowed to drive.

Use the reference `VehicleStateMachine` pattern, but make it more separate from utilities. State handlers can call utility interfaces, but should not implement IO, observer math, controller math, path loading, V2V protocol code, or GUI transport.

Use `core/vehicle_state_machine.py` and keep the reference state-machine pattern.

Core states:

- `INITIALIZING`
- `READY`
- `RUNNING`
- `STOPPED`
- `ERROR`

State meanings:

- `INITIALIZING`: Load config and start vehicle IO, observer, planner, V2V, and GUI.
- `READY`: Everything is initialized. The vehicle is stopped and waiting for a GUI start command.
- `RUNNING`: Observer, planner, and controller are active. This is the only state allowed to send non-zero throttle.
- `STOPPED`: Safe stopped state after user stop, emergency stop, or path finished.
- `ERROR`: A module failed. Always command zero throttle and zero steering.

Allowed transitions:

```text
INITIALIZING -> READY     when all required modules start successfully
INITIALIZING -> ERROR     when startup fails

READY -> RUNNING          when GUI sends START
READY -> STOPPED          when GUI sends STOP or EMERGENCY_STOP

RUNNING -> STOPPED        when GUI sends STOP, path finishes, or emergency stop is received
RUNNING -> ERROR          when observer, planner, controller, vehicle IO, or runtime has a serious error

STOPPED -> READY          when GUI sends RESET or CLEAR_STOP
STOPPED -> RUNNING        optional later; avoid this in the first version

ERROR -> READY            only after RESET and all modules are healthy
```

Required interface:

- `state`
- `handle_command(command: GuiCommand) -> None`
- `mark_ready() -> None`
- `mark_error(reason: str) -> None`
- `should_drive() -> bool`
- `safe_command_required() -> bool`
- `get_status() -> dict`

Runtime rule:

```python
if state_machine.should_drive():
    target = planner.update(estimate)
    command = controller.compute(estimate, target, dt)
else:
    command = zero_command()
```

Acceptance check:

- Only `RUNNING` allows non-zero throttle.
- `STOPPED`, `ERROR`, `READY`, and `INITIALIZING` always produce zero throttle and zero steering.
- GUI `START`, `STOP`, `EMERGENCY_STOP`, and `RESET` commands cause predictable transitions.

### Step 5: Build The Vehicle IO Layer

Use `utils/io/io_base.py` and concrete IO implementations under `utils/io/` to isolate QCar/simulation hardware access.

Responsibilities:

- Initialize `self.qcar`.
- Read sensor data needed by the observer.
- Write `ControlCommand` to the vehicle.
- Clip throttle and steering.
- Provide a `NullVehicleIO` or simulation fallback for tests.

Do not put observer, planner, controller, GUI, or V2V logic in this layer.

Acceptance check:

- The refactored runtime can run with `NullVehicleIO` without physical hardware.
- Vehicle writes always pass through one clipping function.

### Step 6: Build The Observer

Use `utils/control/observer/`.

Start by wrapping the existing `VehicleObserverSimple.VehicleObserver` if it is stable enough. The wrapper should expose only:

- `start(initial_sensor_data)`
- `update(sensor_data, dt, last_command) -> VehicleStateEstimate`
- `get_latest() -> VehicleStateEstimate`
- `stop()`

Keep these out of the core observer path:

- Online SysID
- Online calibration
- Robust KalmanNet dataset
- Scope streaming
- YOLO relative measurement handling
- Dynamic observer selection commands

Acceptance check:

- The controller receives a valid `VehicleStateEstimate` every control tick.
- Missing GPS produces a safe estimate instead of raising.

### Step 7: Build The Path Planner

Use `utils/control/path_planner/`.

Use existing path data first. Avoid changing `path_rich.py` until the refactored path-planning path runs.

Required interface:

- `load_path(path_config)`
- `reset()`
- `update(state: VehicleStateEstimate) -> PlannerTarget`
- `set_target_velocity(v_ref)`
- `is_finished()`

The first planner should provide a target waypoint or lookahead point. It does not need obstacle avoidance, taxi logic, traffic rules, or dynamic route generation.

Acceptance check:

- Given a static waypoint CSV and a fake state, the planner returns a valid target.

### Step 8: Build The Controller

Use `utils/control/controller/`.

Use existing simple controller code if possible:

- Longitudinal: PID velocity controller
- Lateral: pure pursuit, Stanley, or the simplest existing stable path follower

Required interface:

- `reset()`
- `compute(state: VehicleStateEstimate, target: PlannerTarget, dt: float) -> ControlCommand`

Do not use `ControllerManager` at first unless it can be called without state-machine dependencies.

Acceptance check:

- Controller returns clipped or clippable throttle/steering.
- Controller returns zero throttle and zero steering when path is finished or runtime is stopped.

### Step 9: Add V2V Carefully

The first V2V implementation now lives under the current refactor structure:

- `utils/v2v/v2v_base.py`
- `utils/v2v/v2v_udp.py`
- `utils/v2v/__init__.py`
- `test/unit_test_v2v.py`

Do not import `refs/V2V/v2v_manager.py` in the first refactored implementation. That manager is powerful, but it also knows about fleet observers, trust reports, platoons, attack injection, vehicle logic, and Ground Station reporting. For the refactored runtime, use a small UDP adapter first. Later, after the refactored runtime is stable, you can decide whether to wrap the legacy manager.

#### Step 9.1: Fix Shared V2V Types

Add this to `core/types.py`:

```python
@dataclass
class V2VState:
    vehicle_id: int
    timestamp: float
    x: float
    y: float
    theta: float
    velocity: float
```

Optional temporary compatibility:

```python
VBoradcastState = V2VState
```

Also add a simple message type:

```python
@dataclass
class V2VMessage:
    sender_id: int
    timestamp: float
    message_type: str
    payload: dict
```

Use only one message type in the first version:

- `STATE`

Do not add trust, attacks, fleet estimates, platoon commands, warnings, or intents yet.

#### Step 9.2: Define `V2VBase`

Required interface:

```python
class V2VBase(ABC):
    def start(self) -> None: ...
    def process_received_messages(self) -> None: ...
    def broadcast_local_state(self, state: VehicleStateEstimate) -> bool: ...
    def get_peer_states(self) -> dict[int, V2VState]: ...
    def get_status(self) -> dict: ...
    def stop(self) -> None: ...
```

Also add `V2VNull`:

- `start()` does nothing.
- `broadcast_local_state()` returns `False`.
- `get_peer_states()` returns `{}`.
- `get_status()` returns `{"enabled": False, "active": False, "peer_count": 0}`.
- This lets the runtime and GUI work before real V2V is ready.

#### Step 9.3: Implement `V2VUdp`

Use standard library `socket`, `json`, `threading`, `queue`, and `time`.

Recommended config:

```yaml
v2v:
  enabled: true
  vehicle_id: 0
  bind_ip: "0.0.0.0"
  base_port: 8000
  peers:
    - vehicle_id: 1
      ip: "127.0.0.1"
    - vehicle_id: 2
      ip: "127.0.0.1"
  broadcast_rate_hz: 20
  peer_timeout_s: 2.0
```

Port rule:

```text
local_port = base_port + vehicle_id
peer_port = base_port + peer_vehicle_id
```

Message format:

```json
{
  "sender_id": 0,
  "timestamp": 123.456,
  "message_type": "STATE",
  "payload": {
    "x": 1.2,
    "y": 0.3,
    "theta": 0.1,
    "velocity": 0.5
  }
}
```

Threading rule:

- One receive thread may read UDP packets and push decoded messages into a `queue.Queue`.
- `process_received_messages()` drains the queue and updates `_peer_states`.
- `get_peer_states()` returns a copy.
- Use a lock around `_peer_states`.
- `stop()` must close sockets and join the receive thread.

Safety rule:

- V2V must never write vehicle commands.
- V2V must never change the state machine directly.
- V2V failure must not stop local vehicle control.
- V2V exceptions should be logged and reported through `get_status()`.

#### Step 9.4: V2V Tests

Add focused tests:

- `test_null_v2v_status_is_safe`
- `test_udp_v2v_one_vehicle_starts_with_zero_peers`
- `test_udp_v2v_two_instances_exchange_state_on_localhost`
- `test_udp_v2v_ignores_own_messages`
- `test_udp_v2v_peer_timeout_removes_or_marks_stale_peer`
- `test_udp_v2v_stop_closes_thread`

Use high local ports in tests, for example `18000 + vehicle_id`, to avoid conflict with real cars.

Acceptance check:

- One vehicle can run with V2V enabled and zero peers.
- Two localhost V2V instances can exchange `V2VState`.
- Stopping V2V leaves no receive thread alive.
- Runtime can replace `V2VUdp` with `V2VNull` without changing controller, observer, planner, or state machine.

### Step 10: Add Basic Ground-Station Bridge Carefully

The ground-station integration should be split into two layers:

- Ground-station bridge: command and telemetry interface used by the runtime.
- Ground-station application: the visual Tkinter app or future web app.

Start with the bridge. Do not start by copying the large GUI from `refs/GUI`, because it includes calibration, observer switching, remote scope, platoon controls, deployment helpers, and advanced widgets. Use it as a reference only.

Create:

- `utils/ground_station/bridge.py`
- `extra/ground_station/main.py` later, only after the bridge tests pass
- `test/unit_test_ground_station_bridge.py`

#### Step 10.1: Define The Bridge Interface

`GroundStationBridge` should expose:

```python
class GroundStationBridge:
    def start(self) -> None: ...
    def get_commands(self) -> list[GuiCommand]: ...
    def publish_telemetry(self, telemetry: dict) -> None: ...
    def get_latest_telemetry(self) -> dict: ...
    def stop(self) -> None: ...
```

Also add `NullGroundStationBridge`:

- returns no commands
- stores latest telemetry in memory
- useful for integration tests

#### Step 10.2: Define Allowed Commands

Only support these commands first:

- `START`
- `STOP`
- `EMERGENCY_STOP`
- `RESET`
- `SET_VELOCITY`
- `SET_PATH`

Command examples:

```python
GuiCommand(command="START", payload={})
GuiCommand(command="STOP", payload={})
GuiCommand(command="EMERGENCY_STOP", payload={})
GuiCommand(command="RESET", payload={})
GuiCommand(command="SET_VELOCITY", payload={"velocity": 0.4})
GuiCommand(command="SET_PATH", payload={"path": "refs/PathPlanner/path_rich_output/pp_waypoints.csv"})
```

The bridge should validate commands:

- Unknown command is ignored or returned as an error status.
- `SET_VELOCITY` clamps or rejects negative/unsafe values.
- `SET_PATH` should only pass a string path to runtime; the bridge should not load the path itself.

#### Step 10.3: Define Telemetry Payload

The runtime should publish one simple dictionary:

```python
{
  "timestamp": 123.456,
  "vehicle_id": 0,
  "state": "RUNNING",
  "x": 1.0,
  "y": 0.2,
  "theta": 0.1,
  "velocity": 0.5,
  "acceleration": 0.0,
  "throttle": 0.04,
  "steering": 0.1,
  "target_x": 1.5,
  "target_y": 0.2,
  "target_velocity": 0.5,
  "gps_valid": true,
  "v2v_active": true,
  "v2v_peer_count": 1,
  "v2v_peers": [1]
}
```

The ground station should display telemetry, not compute control.

#### Step 10.4: Runtime Command Handling Rule

Runtime handles commands in this order:

1. Get commands from the ground-station bridge.
2. For `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, call state machine.
3. For `SET_VELOCITY`, call `planner.set_target_velocity()`.
4. For `SET_PATH`, call `planner.load_path()` and `planner.reset()`.
5. If any command fails, publish an error field in telemetry.

The ground-station bridge must not directly call controller, observer, vehicle IO, or V2V.

#### Step 10.5: First Ground-Station App

After `GroundStationBridge` tests pass, create a small Tkinter app under `extra/ground_station/`. Keep it boring and clear:

- State label
- Position/velocity labels
- V2V status label
- Start button
- Stop button
- Emergency stop button
- Reset button
- Target velocity entry/button
- Path entry/button

Do not include:

- Calibration
- Online SysID
- Robust KalmanNet
- Attack injection
- Taxi
- Platoon setup
- Remote scope
- Observer/controller switching

Important GUI threading rule:

- Tkinter widgets must only be updated from the Tkinter main thread.
- Runtime and the ground-station bridge should communicate through queues or thread-safe variables.
- Do not let a Tkinter callback write directly to the vehicle.

#### Step 10.6: GUI Tests

Start with bridge tests, not visual tests:

- `test_null_ground_station_bridge_returns_no_commands`
- `test_ground_station_bridge_queues_start_stop_commands`
- `test_ground_station_bridge_stores_latest_telemetry`
- `test_ground_station_bridge_validates_velocity_command`
- `test_runtime_command_mapping_start_stop`
- `test_runtime_command_mapping_set_velocity`

Acceptance check:

- Runtime can run with `NullGroundStationBridge`.
- Runtime can process queued ground-station commands without a real Tk window.
- The ground station can start/stop/reset and display telemetry without importing observer, controller, planner, IO, or V2V internals.

### Step 11: Implement `core/vehicle_logic.py`

The refactored runtime should own lifecycle only. It should not contain controller math, observer math, path planning logic, V2V protocol logic, or GUI implementation details.

The runtime must use `core/vehicle_state_machine.py` for state transitions and safety decisions.

Pseudo-structure:

```python
class VehicleLogic:
    def __init__(self, config, kill_event):
        self.vehicle_io = VehicleIO(config)
        self.observer = Observer(config)
        self.planner = PathPlanner(config)
        self.controller = Controller(config)
        self.v2v = V2V(config)
        self.ground_station = GroundStationBridge(config)
        self.state_machine = StateMachine()

    def run(self):
        self.start_modules()
        self.state_machine.mark_ready()
        while not self.kill_event.is_set():
            self.process_ground_station_commands()
            sensor_data = self.vehicle_io.read()
            estimate = self.observer.update(sensor_data, dt, self.last_command)
            self.v2v.process_received_messages()
            if self.state_machine.should_drive():
                target = self.planner.update(estimate)
                command = self.controller.compute(estimate, target, dt)
            else:
                command = zero_command()
            self.vehicle_io.write(command)
            self.v2v.broadcast_local_state(estimate)
            self.ground_station.publish_telemetry(...)
            sleep_to_rate()
        self.shutdown()
```

Acceptance check:

- Runtime can start, idle in `READY`, start path following, stop, and shut down cleanly.

### Step 12: Add A Runtime Launcher

Add one of these:

- `core/vehicle_main.py`
- or a small root launcher that imports `core.vehicle_main`

Recommended first pass: keep the legacy launcher untouched and add the refactored launcher beside the refactored runtime.

Acceptance check:

- The refactored launcher can load runtime/config utility files and start `core/vehicle_logic.py`.

### Step 13: Add Focused Tests

Add tests under `qcar_refactor/test/`.

Suggested tests:

- `test_config_loads.py`
- `test_state_machine_transitions.py`
- `test_planner_returns_target.py`
- `test_controller_returns_safe_command.py`
- `test_runtime_stops_on_ground_station_stop.py`
- `test_v2v_status_without_peers.py`

Acceptance check:

- Tests run without physical QCar hardware.

### Step 14: Cut Legacy Couplings After Refactored Runtime Works

Only after the refactored runtime passes tests, remove or isolate legacy dependencies from the core runtime path.

Move behind optional flags:

- Calibration
- Online SysID
- Online calibration
- Robust KalmanNet
- YOLO
- Taxi
- V2V attack injection
- Scope streaming
- Advanced platoon control
- Full event-driven state machine

Keep available but not imported by the refactored runtime:

- `Calibration/`
- `Observer/KalmaNet/`
- `Observer/LocalNeuralObs/`
- `Observer/TrustbasedDistributedObserver/`
- `V2V/AttackModule/`
- advanced GUI widgets

Acceptance check:

- Importing `core/vehicle_logic.py` does not import optional research modules.

## Runtime Module Boundaries

| Module | Owns | Must Not Own |
|---|---|---|
| `core/vehicle_logic.py` | Loop, lifecycle, module orchestration | Controller math, observer math, path generation |
| `core/vehicle_state_machine.py` | Runtime mode and safety transitions | Controller math, sensor reading, ground-station transport |
| `utils/io/io_base.py` and concrete IO classes | Hardware/sim read-write and clipping | Ground station, planner, V2V |
| `utils/control/observer/` | Local state estimate | Controller decisions |
| `utils/control/path_planner/` | Waypoint target selection | Actuator command computation |
| `utils/control/controller/` | Throttle/steering command | Sensor reading, V2V |
| `utils/v2v/` | Peer state send/receive | Fleet control policy |
| `utils/ground_station/` | Commands and telemetry transport | Vehicle logic or desktop presentation |

## First Refactor Milestone

The first milestone should be intentionally small:

1. Start runtime.
2. Load static waypoints.
3. Estimate local state.
4. Start from the ground station.
5. Follow path with one controller.
6. Broadcast local state over V2V.
7. Show telemetry in the ground station.
8. Stop safely from the ground station.

Do not include platooning, attacks, calibration, or neural observers in this milestone.

## Safety Rules For The Refactored Runtime

- Default command is always `throttle=0.0`, `steering=0.0`.
- If observer fails, command zero.
- If planner has no target, command zero.
- If a ground-station emergency stop is received, command zero immediately and switch to `STOPPED`.
- If the loop raises an exception, command zero before shutdown.
- All actuator writes pass through one clipping function.
- V2V failure must not stop local emergency stop or local vehicle control.

## Historical Detailed Work Reference

Use `docs/REFACTOR_MASTER_TODO.md` as the only active checklist. The material below preserves detailed rationale and acceptance criteria for the master steps. A checked item means that implementation and focused tests exist, not merely that a file has been created.

### Implementation Remarks

1. Cross-platform units should be practical, not over-specified.
   - For the first runtime, use SI-facing defaults at module boundaries: distance in metres, time in seconds, linear speed in m/s, angles in radians, angular speed in rad/s, and acceleration in m/s^2.
   - A backend adapter converts its native values when the conversion is known. If a sensor has different physics or no direct equivalent, document the approximation and validity instead of forcing false equivalence.
   - Keep the current `ControlCommand(throttle, steering, target_velocity)` contract for the first runtime. Do not block runtime integration on a universal low-level vehicle model.
   - After QCar and CARLA both run, evaluate a backend-specific inner actuator controller that maps desired speed/road-wheel angle to native throttle, brake, steering, or ROS commands. That is a later interface change and requires calibration tests.

2. Treat this as an incremental migration, not a full recode.
   - Reuse working implementations behind adapters when their behavior is stable. Rewrite only code that cannot be separated from legacy `VehicleLogic` or cannot satisfy the new lifecycle/safety contract.
   - Keep the original ground station as an external application first. It already uses a TCP, length-prefixed MessagePack command/telemetry protocol and expects telemetry fields such as `x`, `y`, `th`, `v`, `u`, and `delta`.
   - Implement a `LegacyGroundStationBridge` on the refactored vehicle side that translates that protocol to `GuiCommand` and the new telemetry snapshot. Do not copy the original ground station's deployment, platoon, scope, and research logic into the runtime.
   - Build a new visual application under `extra/ground_station/` only if the legacy protocol cannot support the minimal commands safely or a separate product requirement justifies it.

3. Keep the config loader small and explicit.
   - Use `core/vehicle_config.py` to load and compose files; utilities must not open YAML files themselves.
   - Resolve default config paths relative to the project/config directory, not the current working directory.
   - Apply precedence in one direction: selected module profile, vehicle runtime values, then explicit command-line overrides.
   - Select named profiles explicitly from `config_<module>.yaml` according to `config_vehicle.yaml[modules]`. Avoid an unrestricted recursive merge because duplicate keys can silently override safety values.
   - Validate shared structure centrally, but let each utility validate backend-specific fields. Return one typed/read-only config bundle and pass each module only its owned section.
   - Use the legacy `VehicleMainConfig` as a field reference only. Do not inherit its fleet, YOLO, calibration, deployment, or research configuration in the minimal runtime.

The loader flow should stay close to this shape:

```python
vehicle = load_yaml_mapping(vehicle_path)
selected_modules = vehicle["modules"]
module_configs = {
    name: select_profile(
        load_yaml_mapping(config_dir / f"config_{name}.yaml"),
        profile_name,
    )
    for name, profile_name in selected_modules.items()
}
vehicle_config = ConfigVehicle(
    vehicle_id=vehicle["vehicle"]["car_id"],
    runtime=vehicle["runtime"],
    mission=vehicle["mission"],
    modules=module_configs,
)
```

`vehicle_config.py` should only load, select, compose, and validate. Factories consume the resulting sections and construct modules separately. This keeps config errors distinguishable from hardware or network startup errors.

### State-Machine Compatibility Remark

The current state machine can replace the original top-level safety lifecycle, but it cannot by itself reproduce every original function. This is intentional: the original state handlers compute control, access sensors and networking through `vehicle_logic`, change configuration, and run feature-specific behavior. Copying those handlers would recreate the coupling this refactor is trying to remove.

Use two layers:

1. `StateMachine` remains the safety supervisor: `INITIALIZING`, `CALIBRATING`, `READY`, `RUNNING`, `STOPPED`, and `ERROR`.
2. The runtime selects an operation strategy while the supervisor is `RUNNING`, such as path following, manual control, platoon following, or taxi. A strategy may request a safe transition, but it may not bypass `StateMachine.should_drive()` or write IO directly.

Suggested compatibility mapping:

| Original state | Refactored representation | Owner |
|---|---|---|
| `INITIALIZING` | `INITIALIZING` | Runtime lifecycle and state machine |
| `WAITING_FOR_START` | `READY` | State machine |
| `FOLLOWING_PATH` | `RUNNING` plus path-following strategy | Planner/controller/runtime |
| `FOLLOWING_LEADER` | `RUNNING` plus platoon strategy | Future fleet/platoon service |
| `TAXI_MODE` | `RUNNING` plus taxi mission strategy | Future mission service |
| `MANUAL_MODE` | `RUNNING` plus validated manual-command strategy | GUI bridge/runtime |
| `CALIBRATING` | `CALIBRATING` plus calibration service | Optional calibration module |
| `STOPPED` | `STOPPED` | State machine and zero-command safety path |
| Legacy transition failure | `ERROR` | State machine and runtime lifecycle |

For the first milestone, only migrate `WAITING_FOR_START`, `FOLLOWING_PATH`, `STOPPED`, and emergency/error behavior. Keep platoon, taxi, manual driving, online calibration, observer/controller switching, perception, scopes, and gear commands explicitly unsupported until their owning modules exist. This produces partial but honest compatibility instead of a state machine that accepts commands it cannot execute safely.

### Completed Foundation

- [x] Preserve the legacy implementation under `refs/qcar_origin/` as behavior reference.
- [x] Define `SensorData`, `VehicleStateEstimate`, `ControlCommand`, `PlannerTarget`, `GuiCommand`, `V2VState`, and `V2VMessage` in `core/types.py`.
- [x] Implement the minimal safety state machine in `core/vehicle_state_machine.py`.
- [x] Implement `IOBase`, `IONull`, command clipping, buffered reads, and the QCar adapter. Historical QLabs support remains reference-only because the current license is expired.
- [x] Implement the first EKF observer, static waypoint planner, and simple path controller.
- [x] Implement null and UDP V2V adapters with peer timeout and clean thread shutdown.
- [x] Add focused IO, observer, planner, controller, EKF, and V2V tests.
- [x] Add a virtual closed-loop integration test for IO -> observer -> planner -> controller.

### Phase 1: Stabilize Cross-Module Contracts

- [ ] Record the minimum SI-facing conventions without requiring identical platform physics.
  - [ ] Use metres, seconds, m/s, radians, rad/s, and m/s^2 as the default module-boundary units.
  - [ ] Document `motor_tach` as the current observer speed input and record each backend's source/conversion; defer renaming until QCar and CARLA mappings are measured.
  - [ ] Define pose as local `[x_m, y_m, yaw_rad]` and document each backend's frame conversion.
  - [ ] Define whether each sample timestamp is wall, hardware, ROS, or simulation time.
  - [ ] Record validity/approximation behavior when a backend cannot supply an equivalent sensor value.
- [ ] For synchronous CARLA, let `IOCarla` refresh its cache once per simulation tick; defer a shared `IOBase` timing change until another backend requires it.
- [ ] Standardize `close()` or lifecycle cleanup in `IOBase`; require idempotent `stop()`/`close()` behavior.
- [ ] Protect `_command_cache` with the same consistency expected by `get_last_command()`.
- [ ] Change `ObserverBase.update()` annotations and docs from `dict` to `SensorData`.
- [ ] Remove module-level YAML loading from observer implementations and inject model/config values from `vehicle_config.py`.
- [ ] Replace duplicated placeholder observer files or clearly mark unsupported observer implementations.
- [ ] Convert `test/unit_test_state_machine.py` from script assertions to a discoverable `unittest.TestCase`.
- [ ] Ensure importing `core` and base utilities does not require CARLA, ROS, QLabs, or physical hardware packages.

Acceptance gate: all existing headless tests pass after the contract changes, shared fields have default units/frames, and platform-specific approximations are visible rather than silently treated as identical measurements.

### Phase 2: Implement Config Composition And Factories

- [ ] Implement `core/vehicle_config.py`.
  - [ ] Load `config/config_vehicle.yaml` and the selected `config_<module>.yaml` profile files.
  - [ ] Resolve paths relative to the project/config directory and support explicit alternate paths.
  - [ ] Require every loaded YAML root and selected profile to be a mapping.
  - [ ] Validate required sections, positive rates, port ranges, and safety limits.
  - [ ] Normalize vehicle types such as `qcar`, `qcar_ros`, `limo_ros`, `carla`, and `null`.
  - [ ] Select IO, control, V2V, and GUI profiles explicitly instead of deep-merging unrelated files.
  - [ ] Apply documented precedence: utility profile, runtime selection, then command-line overrides.
  - [ ] Reject unknown override keys and conflicting safety limits.
  - [ ] Return a typed/read-only bundle with runtime, IO, observer, planner, controller, V2V, and GUI sections.
  - [ ] Pass each utility only its owned configuration plus explicitly shared timing values.
- [ ] Add factories for IO, observer, planner, controller, V2V, and GUI bridge.
- [ ] Lazy-load concrete backend modules inside their factory branches.
- [ ] Add config/factory tests for valid, missing, invalid, and unavailable-backend cases.

Acceptance gate: a validated configuration can construct a complete null/headless module set without optional hardware packages installed.

### Phase 3: Implement The Headless Runtime

- [ ] Implement `core/vehicle_logic.py` as lifecycle and loop coordinator only.
  - [ ] Inject or construct modules through factories rather than hard-coding concrete classes.
  - [ ] Start modules in dependency order and unwind already-started modules if startup fails.
  - [ ] Process commands, read IO, estimate state, update V2V, plan, control, write, and publish telemetry in a documented order.
  - [ ] Force a zero command whenever the state is not `RUNNING`.
  - [ ] Force a zero command before cleanup on normal exit and exceptions.
  - [ ] Use monotonic time for loop scheduling and clamp/reject invalid `dt`.
  - [ ] Support an optional simulation-session lifecycle hook such as `tick()` without importing backend APIs into control modules.
- [ ] Keep `StateMachine` responsible only for safety/lifecycle transitions; it must not return throttle or steering as the legacy state machine did.
- [ ] Add a small operation-strategy selection point in the runtime, with path following as the only required first strategy.
- [ ] Map legacy `WAITING_FOR_START`, `FOLLOWING_PATH`, `STOPPED`, and emergency behavior to `READY`, `RUNNING`, `STOPPED`, and `ERROR` tests.
- [ ] Reject unsupported legacy operation commands until their strategy/service is implemented; never silently treat them as successful.
- [ ] Implement `core/vehicle_main.py` with `--config`, `--car-id`, and `--vehicle-type` overrides.
- [ ] Handle `KeyboardInterrupt`, external kill events, startup failure, and repeated shutdown safely.
- [ ] Add runtime tests for READY idle, START, STOP, emergency stop, path completion, module failure, and shutdown order.

Acceptance gate: the headless runtime starts with null IO, remains stopped in `READY`, runs path following only after `START`, rejects unsupported legacy modes, and always writes zero before exit.

### Phase 4: Add The GUI Bridge

- [ ] Add `utils/ground_station/bridge.py` with a small bridge interface, command queue, and telemetry snapshot.
- [ ] Add `NullGroundStationBridge` for headless tests.
- [ ] Support `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, `SET_VELOCITY`, and `SET_PATH` with validation.
- [ ] Keep all actuator writes in the runtime; ground-station callbacks may only enqueue commands.
- [ ] Document the minimal legacy ground-station wire contract: TCP connection, four-byte length prefix, MessagePack payload, command `type`, and required telemetry fields.
- [ ] Add `LegacyGroundStationBridge` that connects the refactored runtime to the original ground station without importing Tkinter modules into the vehicle runtime.
- [ ] Translate legacy `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, velocity, and path messages to the new command types; reject unsupported advanced commands explicitly.
- [ ] Publish compatibility telemetry fields (`x`, `y`, `th`, `v`, `u`, `delta`, `state`) from the new runtime snapshot.
- [ ] Add protocol framing, translation, reconnect, bridge, and runtime-command-mapping tests.
- [ ] Run the original ground station against the refactored null/simulation runtime before deciding whether a new visual application is needed.

Acceptance gate: replacing `NullGroundStationBridge` with `LegacyGroundStationBridge` does not change observer, planner, controller, IO, or V2V code, and the original ground station can perform minimal start/stop/telemetry workflows.

### Phase 5: Add Simulation Sessions And Vehicle Backends

- [x] Implement `extra/simulation/carla_session.py` and `IOCarla` using `docs/CARLA_STARTUP_IO_TODO.md`.
- [ ] Keep CARLA/QLabs client connection, actor lifecycle, ticking, and cleanup in simulation sessions; keep `utils/io` limited to already-created vehicle adapters.
- [ ] Finish QCar ROS and Limo ROS adapters after the same IO contract is stable.
- [ ] Treat a future `QLabsSession` as the same simulation-session pattern, but do not schedule QLabs live validation while the license is expired.
- [ ] Keep backend startup/session ownership outside control modules and IO utilities.
- [ ] Add fake-backend unit tests before live integration tests.
- [ ] Run staged validation:
  - [ ] Null/headless runtime.
  - [ ] CARLA single vehicle.
  - [ ] QLabs single QCar when a valid QLabs license is available again.
  - [ ] Physical QCar at low throttle.
  - [ ] Two vehicles exchanging V2V state.

Acceptance gate: selecting a backend changes only construction/session wiring, not the state-machine or control pipeline.

### Phase 6: Multi-Vehicle Setup, Fleet, And Optional Features

- [ ] Build independent vehicle runtime setups with unique IDs before adding V2V/fleet behavior.
- [ ] Define foundational `utils/fleet/` membership and peer-snapshot utilities after V2V transport and the single-vehicle runtime are stable.
- [ ] Keep fleet policy separate from V2V transport and local safety.
- [ ] Add platooning, trust, attacks, calibration, SysID, neural observers, YOLO, taxi, and scope streaming one feature at a time behind optional interfaces.
- [ ] Do not import optional research modules from the default runtime path.
- [ ] Remove obsolete legacy coupling only after equivalent refactored behavior has tests.

Acceptance gate: the minimal runtime remains importable and testable when every optional feature dependency is absent.
