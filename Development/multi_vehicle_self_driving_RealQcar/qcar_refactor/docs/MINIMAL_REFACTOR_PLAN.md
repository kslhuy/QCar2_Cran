# QCar Refactor Plan

This plan is based on `qcar_refactor/vehicle_logic.py` and the current refactor folder structure. The goal is to make the project easier to read, test, and modify while keeping the useful patterns from the reference implementation.

- Basic GUI
- Controller
- Observer
- Path planner
- V2V communication
- A simple state machine

Keep the same state-machine idea from the current project, but make it smaller and more standard: one small module owns the vehicle mode, and the other modules stay independent.

Everything else should either stay in `refs/` as reference code or be connected later through clear optional modules.

## Current Run-Loop Modules

`VehicleLogic.run()` is the main run cycle. Each loop currently schedules these modules:

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

- `qcar_refactor/core/Types.py`
- `qcar_refactor/core/VehicleStateMachine.py`
- `qcar_refactor/core/VehicleLogic.py`
- `qcar_refactor/core/VehicleMain.py`
- `qcar_refactor/config/ConfigRuntime.yaml` or another runtime-level config file
- `qcar_refactor/config/ConfigControl.yaml`
- `qcar_refactor/config/ConfigIO.yaml`
- `qcar_refactor/utils/IO/`
- `qcar_refactor/utils/Control/Observer/`
- `qcar_refactor/utils/Control/PathPlanner/`
- `qcar_refactor/utils/Control/Controller/`
- `qcar_refactor/utils/V2V/`
- `qcar_refactor/utils/GUI/` if the visual app becomes large enough to split from the bridge

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

- Step 4 state machine exists as `core/VehicleStateMachine.py`.
- Step 5 vehicle IO exists under `utils/IO/BaseIO.py`.
- Step 6 observer exists under `utils/Control/Observer/ObserverEKF.py`.
- Step 7 path planner exists under `utils/Control/PathPlanner/`.
- Step 8 controller exists under `utils/Control/Controller/`.
- The integration test `test/test_integration_control_loop.py` passes with virtual IO, EKF, static waypoint planner, and simple path controller.

### Problems To Fix Before V2V/GUI

1. Project naming is standardized to the current folder structure.
   - Shared dataclasses live in `core/Types.py`.
   - Runtime/state-machine files live in `core/`.
   - Reusable modules live in `utils/`.
   - IO lives in `utils/IO/`.
   - Observer, path planner, and controller live under `utils/Control/`.
   - V2V should live in `utils/V2V/`.
   - GUI bridge can start in `core/GuiBridge.py`; visual GUI code can later live in `utils/GUI/`.

2. The V2V state type should be `V2VState`.
   - V2V is both a broadcast mechanism and vehicle-to-vehicle communication.
   - Use `V2VState` as the standard type name.
   - Keep `VBoradcastState` only as a temporary backward-compatible alias if older code imports it.

3. Config should stay separated by purpose.
   - No `minimal_config.yaml` is needed.
   - Runtime-level config belongs in a runtime config file, for example `ConfigRuntime.yaml`.
   - Utility-level config files should keep the `ConfigName.yaml` style, for example `ConfigControl.yaml` and `ConfigIO.yaml`.
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

### Do Before Step 9

- [x] Add `V2VState` in `core/Types.py`.
- [x] Use current project naming: `core/Types.py`, `utils/IO`, `utils/Control`, `utils/V2V`.
- [x] Remove the `minimal_config.yaml` direction from the plan.
- [ ] Convert `unit_test_state_machine.py` to a real `unittest.TestCase`.
- [x] Define EKF acceleration as `sqrt(ax**2 + ay**2)`.
- [x] Make `NullVehicleIO` mutate `_sensor_data_cache`.

### Step 1: Freeze The Current Runtime

Do not delete the current `vehicle_logic.py` immediately. Keep it as the legacy reference until the refactored runtime can start and stop cleanly.

Actions:

- Add or complete `core/VehicleLogic.py` instead of rewriting the legacy reference first.
- Keep `vehicle_main.py` working for the legacy path.
- Add a refactored launcher or launcher argument later, after the refactored runtime is tested.

Acceptance check:

- Existing files remain importable.
- No optional research folder needs to be edited in this step.

### Step 2: Define Shared Data Contracts

Use `core/Types.py` for plain dataclasses. Use these everywhere in the refactored runtime.

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

- Runtime config: one runtime-level file, for example `config/ConfigRuntime.yaml`.
- Control utility config: `config/ConfigControl.yaml`.
- IO utility config: `config/ConfigIO.yaml`.

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
  gui_rate_hz: 10
  v2v_rate_hz: 25

control:
  target_velocity: 0.6
  longitudinal: pid
  lateral: pure_pursuit

path:
  default_path: path_rich_output/pp_waypoints.csv

gui:
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

Use `core/VehicleStateMachine.py` and keep the reference state-machine pattern.

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

Use `utils/IO/BaseIO.py` and concrete IO implementations under `utils/IO/` to isolate QCar/simulation hardware access.

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

Use `utils/Control/Observer/`.

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

Use `utils/Control/PathPlanner/`.

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

Use `utils/Control/Controller/`.

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

Create the V2V code under the current refactor structure:

- `utils/V2V/BaseV2V.py`
- `utils/V2V/UdpV2V.py`
- `utils/V2V/__init__.py`
- `test/unit_test_v2v.py`

Do not import `refs/V2V/v2v_manager.py` in the first refactored implementation. That manager is powerful, but it also knows about fleet observers, trust reports, platoons, attack injection, vehicle logic, and Ground Station reporting. For the refactored runtime, use a small UDP adapter first. Later, after the refactored runtime is stable, you can decide whether to wrap the legacy manager.

#### Step 9.1: Fix Shared V2V Types

Add this to `core/Types.py`:

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

#### Step 9.2: Define `BaseV2V`

Required interface:

```python
class BaseV2V(ABC):
    def start(self) -> None: ...
    def update_receive(self) -> None: ...
    def broadcast_state(self, state: VehicleStateEstimate) -> bool: ...
    def get_peer_states(self) -> dict[int, V2VState]: ...
    def get_status(self) -> dict: ...
    def stop(self) -> None: ...
```

Also add `NullV2V`:

- `start()` does nothing.
- `broadcast_state()` returns `False`.
- `get_peer_states()` returns `{}`.
- `get_status()` returns `{"enabled": False, "active": False, "peer_count": 0}`.
- This lets the runtime and GUI work before real V2V is ready.

#### Step 9.3: Implement `UdpV2V`

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
- `update_receive()` drains the queue and updates `_peer_states`.
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
- Runtime can replace `UdpV2V` with `NullV2V` without changing controller, observer, planner, or state machine.

### Step 10: Add Basic GUI Bridge Carefully

The GUI should be split into two layers:

- GUI bridge: command and telemetry interface used by the runtime.
- GUI application: the visual Tkinter app or future web app.

Start with the bridge. Do not start by copying the large GUI from `refs/GUI`, because it includes calibration, observer switching, remote scope, platoon controls, deployment helpers, and advanced widgets. Use it as a reference only.

Create:

- `core/GuiBridge.py`
- `utils/GUI/BasicGuiApp.py` later, only after the bridge tests pass
- `test/unit_test_gui_bridge.py`

#### Step 10.1: Define The Bridge Interface

`GuiBridge` should expose:

```python
class GuiBridge:
    def start(self) -> None: ...
    def get_commands(self) -> list[GuiCommand]: ...
    def publish_telemetry(self, telemetry: dict) -> None: ...
    def get_latest_telemetry(self) -> dict: ...
    def stop(self) -> None: ...
```

Also add `NullGuiBridge`:

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

GUI should display telemetry, not compute control.

#### Step 10.4: Runtime Command Handling Rule

Runtime handles commands in this order:

1. Get commands from GUI bridge.
2. For `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, call state machine.
3. For `SET_VELOCITY`, call `planner.set_target_velocity()`.
4. For `SET_PATH`, call `planner.load_path()` and `planner.reset()`.
5. If any command fails, publish an error field in telemetry.

The GUI bridge must not directly call controller, observer, vehicle IO, or V2V.

#### Step 10.5: First GUI App

After `GuiBridge` tests pass, create a small Tkinter app. Keep it boring and clear:

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
- Runtime and GUI bridge should communicate through queues or thread-safe variables.
- Do not let a Tkinter callback write directly to the vehicle.

#### Step 10.6: GUI Tests

Start with bridge tests, not visual tests:

- `test_null_gui_bridge_returns_no_commands`
- `test_gui_bridge_queues_start_stop_commands`
- `test_gui_bridge_stores_latest_telemetry`
- `test_gui_bridge_validates_velocity_command`
- `test_runtime_command_mapping_start_stop`
- `test_runtime_command_mapping_set_velocity`

Acceptance check:

- Runtime can run with `NullGuiBridge`.
- Runtime can process queued GUI commands without a real Tk window.
- GUI can start/stop/reset and display telemetry without importing observer, controller, planner, IO, or V2V internals.

### Step 11: Implement `core/VehicleLogic.py`

The refactored runtime should own lifecycle only. It should not contain controller math, observer math, path planning logic, V2V protocol logic, or GUI implementation details.

The runtime must use `core/VehicleStateMachine.py` for state transitions and safety decisions.

Pseudo-structure:

```python
class VehicleLogic:
    def __init__(self, config, kill_event):
        self.vehicle_io = VehicleIO(config)
        self.observer = Observer(config)
        self.planner = PathPlanner(config)
        self.controller = Controller(config)
        self.v2v = V2V(config)
        self.gui = GuiBridge(config)
        self.state_machine = StateMachine()

    def run(self):
        self.start_modules()
        self.state_machine.mark_ready()
        while not self.kill_event.is_set():
            self.process_gui_commands()
            sensor_data = self.vehicle_io.read()
            estimate = self.observer.update(sensor_data, dt, self.last_command)
            self.v2v.update_receive()
            if self.state_machine.should_drive():
                target = self.planner.update(estimate)
                command = self.controller.compute(estimate, target, dt)
            else:
                command = zero_command()
            self.vehicle_io.write(command)
            self.v2v.broadcast_state(estimate)
            self.gui.publish_telemetry(...)
            sleep_to_rate()
        self.shutdown()
```

Acceptance check:

- Runtime can start, idle in `READY`, start path following, stop, and shut down cleanly.

### Step 12: Add A Runtime Launcher

Add one of these:

- `core/VehicleMain.py`
- or a small root launcher that imports `core.VehicleMain`

Recommended first pass: keep the legacy launcher untouched and add the refactored launcher beside the refactored runtime.

Acceptance check:

- The refactored launcher can load runtime/config utility files and start `core/VehicleLogic.py`.

### Step 13: Add Focused Tests

Add tests under `qcar_refactor/test/`.

Suggested tests:

- `test_config_loads.py`
- `test_state_machine_transitions.py`
- `test_planner_returns_target.py`
- `test_controller_returns_safe_command.py`
- `test_runtime_stops_on_gui_stop.py`
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

- Importing `core/VehicleLogic.py` does not import optional research modules.

## Runtime Module Boundaries

| Module | Owns | Must Not Own |
|---|---|---|
| `core/VehicleLogic.py` | Loop, lifecycle, module orchestration | Controller math, observer math, path generation |
| `core/VehicleStateMachine.py` | Runtime mode and safety transitions | Controller math, sensor reading, GUI transport |
| `utils/IO/BaseIO.py` and concrete IO classes | Hardware/sim read-write and clipping | GUI, planner, V2V |
| `utils/Control/Observer/` | Local state estimate | Controller decisions |
| `utils/Control/PathPlanner/` | Waypoint target selection | Actuator command computation |
| `utils/Control/Controller/` | Throttle/steering command | Sensor reading, V2V |
| `utils/V2V/` | Peer state send/receive | Fleet control policy |
| `core/GuiBridge.py` | Commands and telemetry transport | Vehicle logic |

## First Refactor Milestone

The first milestone should be intentionally small:

1. Start runtime.
2. Load static waypoints.
3. Estimate local state.
4. Start from GUI.
5. Follow path with one controller.
6. Broadcast local state over V2V.
7. Show telemetry in GUI.
8. Stop safely from GUI.

Do not include platooning, attacks, calibration, or neural observers in this milestone.

## Safety Rules For The Refactored Runtime

- Default command is always `throttle=0.0`, `steering=0.0`.
- If observer fails, command zero.
- If planner has no target, command zero.
- If GUI emergency stop is received, command zero immediately and switch to `STOPPED`.
- If the loop raises an exception, command zero before shutdown.
- All actuator writes pass through one clipping function.
- V2V failure must not stop local emergency stop or local vehicle control.

## Suggested Work Order

Use this as the main todo list.

- [ ] Step 1: Freeze the current runtime.
  - [ ] Keep the current `vehicle_logic.py` as the legacy reference.
  - [ ] Do not delete or rewrite the full state machine yet.
  - [ ] Do not edit optional research modules yet.

- [ ] Step 2: Add shared data types.
  - [ ] Use `core/Types.py`.
  - [ ] Add `VehicleStateEstimate`.
  - [ ] Add `ControlCommand`.
  - [ ] Add `PlannerTarget`.
  - [ ] Add `GuiCommand`.
  - [x] Add `V2VState`.

- [ ] Step 3: Keep config ownership clear.
  - [ ] Use a runtime config file for runtime settings, for example `ConfigRuntime.yaml`.
  - [ ] Use utility config files for utility settings, for example `ConfigControl.yaml` and `ConfigIO.yaml`.
  - [ ] Pass each module only the config section it owns.

- [ ] Step 4: Refactor the state machine.
  - [ ] Use `core/VehicleStateMachine.py`.
  - [ ] Add states: `INITIALIZING`, `READY`, `RUNNING`, `STOPPED`, `ERROR`.
  - [ ] Add transitions for `START`, `STOP`, `EMERGENCY_STOP`, and `RESET`.
  - [ ] Add `should_drive()`.
  - [ ] Add tests proving only `RUNNING` can drive.

- [ ] Step 5: Add vehicle IO.
  - [ ] Use `utils/IO/BaseIO.py`.
  - [x] Fix `NullVehicleIO` to write directly to `_sensor_data_cache`.
  - [ ] Add or finish real/simulation QCar IO wrapper.
  - [ ] Add one command clipping function for throttle and steering.

- [ ] Step 6: Add the observer wrapper.
  - [ ] Use `utils/Control/Observer/`.
  - [ ] Wrap or simplify `VehicleObserverSimple.VehicleObserver`.
  - [ ] Return `VehicleStateEstimate`.
  - [x] Define EKF acceleration as horizontal magnitude `sqrt(ax**2 + ay**2)`.
  - [ ] Disable SysID, calibration, KalmanNet recording, scope streaming, and YOLO in this path.

- [ ] Step 7: Add the path planner.
  - [ ] Use `utils/Control/PathPlanner/`.
  - [ ] Load a static waypoint CSV.
  - [ ] Return a `PlannerTarget`.
  - [ ] Detect when the path is finished.

- [ ] Step 8: Add the controller.
  - [ ] Use `utils/Control/Controller/`.
  - [ ] Use a simple longitudinal controller.
  - [ ] Use a simple lateral path-following controller.
  - [ ] Return `ControlCommand`.
  - [ ] Return zero command when target/path is invalid.

- [ ] Step 9: Add the V2V wrapper.
  - [x] Add `V2VState` to `core/Types.py`.
  - [ ] Create `utils/V2V/BaseV2V.py`.
  - [ ] Add `NullV2V`.
  - [ ] Create `utils/V2V/UdpV2V.py`.
  - [ ] Broadcast only local `VehicleStateEstimate` as `STATE`.
  - [ ] Receive peer states as `V2VState`.
  - [ ] Add localhost two-vehicle exchange test.
  - [ ] Add stop/thread cleanup test.
  - [ ] Keep attack injection, trust logic, fleet observer fusion, and platoon logic disabled in the first core path.

- [ ] Step 10: Add the basic GUI bridge.
  - [ ] Create `core/GuiBridge.py`.
  - [ ] Add `NullGuiBridge`.
  - [ ] Support `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, `SET_VELOCITY`, and `SET_PATH`.
  - [ ] Publish simple telemetry dictionary.
  - [ ] Add bridge tests before building a real Tkinter window.
  - [ ] Build a basic Tkinter app only after bridge tests pass.
  - [ ] Keep calibration, attack, taxi, observer switching, remote scope, and advanced platoon controls out of the first GUI.

- [ ] Step 11: Add the refactored runtime.
  - [ ] Create or complete `core/VehicleLogic.py`.
  - [ ] Instantiate vehicle IO, observer, planner, controller, V2V, GUI bridge, and state machine.
  - [ ] Ensure only `state_machine.should_drive()` can produce non-zero throttle.
  - [ ] Always write zero command during shutdown and errors.

- [ ] Step 12: Add a refactored launcher.
  - [ ] Create or complete `core/VehicleMain.py`.
  - [ ] Add `--config`, `--car-id`, and safe default arguments.
  - [ ] Keep legacy `vehicle_main.py` unchanged for now.

- [ ] Step 13: Add focused tests.
  - [ ] Add `test/test_config_loads.py`.
  - [ ] Add or convert `test/unit_test_state_machine.py` to `unittest.TestCase`.
  - [ ] Add or keep `test/unit_test_path_planner.py`.
  - [ ] Add or keep `test/unit_test_controller.py`.
  - [ ] Add `test/test_runtime_stops_on_gui_stop.py`.
  - [ ] Add `test/unit_test_v2v.py`.

- [ ] Step 14: Test in stages.
  - [ ] Run with `NullVehicleIO`.
  - [ ] Run in simulation.
  - [ ] Run one physical QCar at low `max_throttle`.
  - [ ] Run two vehicles and verify V2V peer discovery.

- [ ] Step 15: Clean up legacy coupling only after the refactored runtime works.
  - [ ] Keep research modules available but not imported by `core/VehicleLogic.py`.
  - [ ] Move calibration, SysID, KalmanNet, YOLO, taxi, attack injection, scope streaming, and advanced platoon features behind optional paths.
