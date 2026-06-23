# Minimal QCar Refactor Plan

This plan is based on `qcar_refactor/vehicle_logic.py` as it exists now. The goal is to reduce the project to a minimal working system with only:

- Basic GUI
- Controller
- Observer
- Path planner
- V2V communication
- A simple state machine

Keep the same state-machine idea from the current project, but make it smaller and more standard: one small module owns the vehicle mode, and the other modules stay independent.

Everything else should either be removed from the minimal runtime path or placed behind optional feature flags.

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
4. The state machine pulls in many non-minimal states: taxi, calibration, manual, following leader, stopped, waiting, and path following.
5. The minimal dependencies are not separated from research/experiment dependencies, so small changes can break unrelated systems.

## Target Minimal Architecture

Create a small runtime coordinator and make each active module expose a simple interface.

Recommended new files:

- `qcar_refactor/minimal_runtime.py`
- `qcar_refactor/minimal_config.yaml`
- `qcar_refactor/core/types.py`
- `qcar_refactor/core/rate.py`
- `qcar_refactor/core/gui_bridge.py`
- `qcar_refactor/core/vehicle_io.py`
- `qcar_refactor/StateMachine/minimal_state_machine.py`
- `qcar_refactor/Observer/minimal_observer.py`
- `qcar_refactor/Controller/minimal_controller.py`
- `qcar_refactor/PathPlanner/minimal_path_planner.py`
- `qcar_refactor/V2V/minimal_v2v.py`

The minimal loop should be:

```text
while running:
    read vehicle sensors
    update observer
    receive/process V2V data
    update minimal state machine
    update path planner target
    compute controller command
    write command to vehicle
    publish telemetry to GUI
    process GUI commands
    broadcast local state over V2V
    sleep to maintain loop rate
```

## Step-By-Step Implementation Plan

### Step 1: Freeze The Current Runtime

Do not delete the current `vehicle_logic.py` immediately. Keep it as the legacy reference until the minimal runtime can start and stop cleanly.

Actions:

- Add `minimal_runtime.py` instead of rewriting `vehicle_logic.py` first.
- Keep `vehicle_main.py` working for the legacy path.
- Add a new launcher argument later, for example `python vehicle_main.py --minimal`, after the minimal runtime is tested.

Acceptance check:

- Existing files remain importable.
- No optional research folder needs to be edited in this step.

### Step 2: Define Minimal Data Contracts

Create `core/types.py` with plain dataclasses. Use these everywhere in the minimal runtime.

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

- `V2VPeerState`
  - `vehicle_id`
  - `timestamp`
  - `x`
  - `y`
  - `theta`
  - `velocity`

Acceptance check:

- Minimal modules exchange only these dataclasses or dictionaries converted at module boundaries.

### Step 3: Create A Minimal Config

Create `minimal_config.yaml` with only the fields needed by the minimal runtime.

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

- The minimal runtime can load this config without depending on `VehicleMainConfig`.

### Step 4: Create The Minimal State Machine

Keep a state machine in the minimal project. It is important for safety and for beginner-friendly debugging because it makes clear when the car is allowed to drive.

Do not use the full existing `VehicleStateMachine` in the first minimal version. The current state machine imports many non-minimal modes such as taxi, calibration, manual mode, leader following, and advanced path behavior. Keep that full version as a reference, but create a small state machine for the minimal runtime.

Create `StateMachine/minimal_state_machine.py`.

Minimal states:

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
INITIALIZING -> READY     when all minimal modules start successfully
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

Minimal runtime rule:

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

### Step 5: Build The Minimal Vehicle IO Layer

Create `core/vehicle_io.py` to isolate QCar/simulation hardware access.

Responsibilities:

- Initialize `self.qcar`.
- Read sensor data needed by the observer.
- Write `ControlCommand` to the vehicle.
- Clip throttle and steering.
- Provide a `NullVehicleIO` or simulation fallback for tests.

Do not put observer, planner, controller, GUI, or V2V logic in this layer.

Acceptance check:

- The minimal runtime can run with `NullVehicleIO` without physical hardware.
- Vehicle writes always pass through one clipping function.

### Step 6: Wrap The Observer

Create `Observer/minimal_observer.py`.

Start by wrapping the existing `VehicleObserverSimple.VehicleObserver` if it is stable enough. The wrapper should expose only:

- `start(initial_sensor_data)`
- `update(sensor_data, dt, last_command) -> VehicleStateEstimate`
- `get_latest() -> VehicleStateEstimate`
- `stop()`

Disable these in the minimal observer path:

- Online SysID
- Online calibration
- Robust KalmanNet dataset
- Scope streaming
- YOLO relative measurement handling
- Dynamic observer selection commands

Acceptance check:

- The controller receives a valid `VehicleStateEstimate` every control tick.
- Missing GPS produces a safe estimate instead of raising.

### Step 7: Wrap The Path Planner

Create `PathPlanner/minimal_path_planner.py`.

Use existing path data first. Avoid changing `path_rich.py` until the minimal system runs.

Required interface:

- `load_path(path_config)`
- `reset()`
- `update(state: VehicleStateEstimate) -> PlannerTarget`
- `set_target_velocity(v_ref)`
- `is_finished()`

The minimal planner should provide a target waypoint or lookahead point. It does not need obstacle avoidance, taxi logic, traffic rules, or dynamic route generation.

Acceptance check:

- Given a static waypoint CSV and a fake state, the planner returns a valid target.

### Step 8: Wrap The Controller

Create `Controller/minimal_controller.py`.

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

### Step 9: Wrap V2V

Create `V2V/minimal_v2v.py`.

Start by wrapping the existing `V2VManager`, but disable optional attack and trust/reporting paths for the first pass.

Required interface:

- `start(observer=None)`
- `update_receive()`
- `broadcast_state(state: VehicleStateEstimate)`
- `get_peer_states() -> dict[int, V2VPeerState]`
- `get_status() -> dict`
- `stop()`

The minimal runtime should use V2V only to share local state and receive peer states. Fleet observer fusion can be added later after the base loop is stable.

Acceptance check:

- One vehicle can run with V2V enabled and zero peers.
- Two vehicles can see each other as peers.

### Step 10: Create A Basic GUI Bridge

Create `core/gui_bridge.py` or simplify the existing GUI connector.

Minimal GUI features:

- Connect/disconnect vehicle
- Start path following
- Stop vehicle
- Emergency stop
- Set target velocity
- Select/reload path
- Show telemetry:
  - vehicle ID
  - current state
  - x, y, theta
  - velocity
  - throttle
  - steering
  - GPS valid
  - V2V active
  - peer count

Do not include these in the minimal GUI:

- Calibration controls
- Online SysID controls
- Robust KalmanNet controls
- Attack injection controls
- Taxi controls
- Advanced platoon setup
- Remote scope streaming

Acceptance check:

- GUI can command start/stop and display telemetry without knowing internal controller/observer classes.

### Step 11: Implement `minimal_runtime.py`

The minimal runtime should own lifecycle only. It should not contain controller math, observer math, path planning logic, V2V protocol logic, or GUI implementation details.

The minimal runtime must use `StateMachine/minimal_state_machine.py` for state transitions and safety decisions.

Pseudo-structure:

```python
class MinimalVehicleRuntime:
    def __init__(self, config, kill_event):
        self.vehicle_io = VehicleIO(config)
        self.observer = MinimalObserver(config)
        self.planner = MinimalPathPlanner(config)
        self.controller = MinimalController(config)
        self.v2v = MinimalV2V(config)
        self.gui = GuiBridge(config)
        self.state_machine = MinimalStateMachine()

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

### Step 12: Add A Minimal Launcher

Add one of these:

- `vehicle_minimal_main.py`
- or `vehicle_main.py --minimal`

Recommended first pass: create `vehicle_minimal_main.py` so the legacy launcher is untouched.

Acceptance check:

- `python vehicle_minimal_main.py --config minimal_config.yaml --car-id 0` starts the minimal runtime.

### Step 13: Add Focused Tests

Add tests under `qcar_refactor/Test/minimal/`.

Suggested tests:

- `test_minimal_config_loads.py`
- `test_minimal_state_machine_transitions.py`
- `test_planner_returns_target.py`
- `test_controller_returns_safe_command.py`
- `test_runtime_stops_on_gui_stop.py`
- `test_v2v_status_without_peers.py`

Acceptance check:

- Tests run without physical QCar hardware.

### Step 14: Cut Legacy Couplings After Minimal Runtime Works

Only after the minimal runtime passes tests, remove or isolate legacy dependencies from the minimal path.

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

Keep available but not imported by minimal runtime:

- `Calibration/`
- `Observer/KalmaNet/`
- `Observer/LocalNeuralObs/`
- `Observer/TrustbasedDistributedObserver/`
- `V2V/AttackModule/`
- advanced GUI widgets

Acceptance check:

- Importing `minimal_runtime.py` does not import optional research modules.

## Minimal Runtime Module Boundaries

| Module | Owns | Must Not Own |
|---|---|---|
| `minimal_runtime.py` | Loop, lifecycle, module orchestration | Controller math, observer math, path generation |
| `StateMachine/minimal_state_machine.py` | Runtime mode and safety transitions | Controller math, sensor reading, GUI transport |
| `core/vehicle_io.py` | Hardware/sim read-write and clipping | GUI, planner, V2V |
| `Observer/minimal_observer.py` | Local state estimate | Controller decisions |
| `PathPlanner/minimal_path_planner.py` | Waypoint target selection | Actuator command computation |
| `Controller/minimal_controller.py` | Throttle/steering command | Sensor reading, V2V |
| `V2V/minimal_v2v.py` | Peer state send/receive | Fleet control policy |
| `core/gui_bridge.py` | Commands and telemetry transport | Vehicle logic |

## First Minimal Milestone

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

## Safety Rules For The Minimal Runtime

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

- [ ] Step 2: Add shared minimal data types.
  - [ ] Create `core/types.py`.
  - [ ] Add `VehicleStateEstimate`.
  - [ ] Add `ControlCommand`.
  - [ ] Add `PlannerTarget`.
  - [ ] Add `GuiCommand`.
  - [ ] Add `V2VPeerState`.

- [ ] Step 3: Add minimal configuration.
  - [ ] Create `minimal_config.yaml`.
  - [ ] Include vehicle ID, limits, timing rates, controller selection, path, GUI, and V2V settings.
  - [ ] Load it without depending on `VehicleMainConfig`.

- [ ] Step 4: Add the minimal state machine.
  - [ ] Create `StateMachine/minimal_state_machine.py`.
  - [ ] Add states: `INITIALIZING`, `READY`, `RUNNING`, `STOPPED`, `ERROR`.
  - [ ] Add transitions for `START`, `STOP`, `EMERGENCY_STOP`, and `RESET`.
  - [ ] Add `should_drive()`.
  - [ ] Add tests proving only `RUNNING` can drive.

- [ ] Step 5: Add vehicle IO.
  - [ ] Create `core/vehicle_io.py`.
  - [ ] Add `NullVehicleIO` for tests.
  - [ ] Add real/simulation QCar IO wrapper.
  - [ ] Add one command clipping function for throttle and steering.

- [ ] Step 6: Add the minimal observer wrapper.
  - [ ] Create `Observer/minimal_observer.py`.
  - [ ] Wrap or simplify `VehicleObserverSimple.VehicleObserver`.
  - [ ] Return `VehicleStateEstimate`.
  - [ ] Disable SysID, calibration, KalmanNet recording, scope streaming, and YOLO in this path.

- [ ] Step 7: Add the minimal path planner.
  - [ ] Create `PathPlanner/minimal_path_planner.py`.
  - [ ] Load a static waypoint CSV.
  - [ ] Return a `PlannerTarget`.
  - [ ] Detect when the path is finished.

- [ ] Step 8: Add the minimal controller.
  - [ ] Create `Controller/minimal_controller.py`.
  - [ ] Use a simple longitudinal controller.
  - [ ] Use a simple lateral path-following controller.
  - [ ] Return `ControlCommand`.
  - [ ] Return zero command when target/path is invalid.

- [ ] Step 9: Add the minimal V2V wrapper.
  - [ ] Create `V2V/minimal_v2v.py`.
  - [ ] Broadcast local `VehicleStateEstimate`.
  - [ ] Receive peer states as `V2VPeerState`.
  - [ ] Keep attack injection and trust logic disabled in the minimal path.

- [ ] Step 10: Add the basic GUI bridge.
  - [ ] Create `core/gui_bridge.py`.
  - [ ] Support `START`, `STOP`, `EMERGENCY_STOP`, `RESET`, and target velocity commands.
  - [ ] Publish simple telemetry.
  - [ ] Keep calibration, attack, taxi, and advanced platoon controls out of the minimal GUI.

- [ ] Step 11: Add the minimal runtime.
  - [ ] Create `minimal_runtime.py`.
  - [ ] Instantiate vehicle IO, observer, planner, controller, V2V, GUI bridge, and minimal state machine.
  - [ ] Ensure only `state_machine.should_drive()` can produce non-zero throttle.
  - [ ] Always write zero command during shutdown and errors.

- [ ] Step 12: Add a minimal launcher.
  - [ ] Create `vehicle_minimal_main.py`.
  - [ ] Add `--config`, `--car-id`, and safe default arguments.
  - [ ] Keep legacy `vehicle_main.py` unchanged for now.

- [ ] Step 13: Add focused tests.
  - [ ] Add `Test/minimal/test_minimal_config_loads.py`.
  - [ ] Add `Test/minimal/test_minimal_state_machine_transitions.py`.
  - [ ] Add `Test/minimal/test_planner_returns_target.py`.
  - [ ] Add `Test/minimal/test_controller_returns_safe_command.py`.
  - [ ] Add `Test/minimal/test_runtime_stops_on_gui_stop.py`.
  - [ ] Add `Test/minimal/test_v2v_status_without_peers.py`.

- [ ] Step 14: Test in stages.
  - [ ] Run with `NullVehicleIO`.
  - [ ] Run in simulation.
  - [ ] Run one physical QCar at low `max_throttle`.
  - [ ] Run two vehicles and verify V2V peer discovery.

- [ ] Step 15: Clean up legacy coupling only after the minimal runtime works.
  - [ ] Keep research modules available but not imported by `minimal_runtime.py`.
  - [ ] Move calibration, SysID, KalmanNet, YOLO, taxi, attack injection, scope streaming, and advanced platoon features behind optional paths.
