# `core/vehicle_logic.py` — Vehicle Runtime

## 1. Introduction

`VehicleRuntime` owns one vehicle actor's ordered control loop and the final
safe-actuation decision. It receives every utility as an injected dependency;
it never imports a concrete IO, simulator, or transport backend.

## 2. Code structure

`VehicleRuntime(config, io, observer, planner, controller_manager, v2v,
ground_station, simulation=None, fleet=None, logger=None)` stores injected
services, creates one `StateMachine` and `VehicleCommandHandler`, and starts
with `_started=False`. `RuntimeTelemetry` is a frozen sample emitted after each
successful step.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `RuntimeTelemetry` | state, dt, sensor, estimate, target, command | frozen step record | Captures one completed iteration for monitoring/tests. |
| `VehicleRuntime.__init__(...)` | resolved config and injected services | runtime instance | Sets service ownership, command handler, state machine, logger, and timing state. |
| `fleet` | none | optional fleet manager | Returns the injected fleet interface. |
| `ground_station` | none | runtime facade | Returns the injected ground-station facade. |
| `start()` | none | none or `RuntimeError` | Starts simulation/observer/V2V/ground station, resets planner/controller, rolls back on error, then marks READY. |
| `handle_command(command)` | `VehicleCommand` | recorded `CommandResult` | Applies `CommandHandling` profile/manual/reset/safe-stop requests and records result with ground station. |
| `step(dt=None)` | optional positive control period | `RuntimeTelemetry` | Drains commands, ticks simulation, reads IO, estimates, runs fleet/planner, controls, writes, and publishes monitoring. |
| `shutdown()` | none | none | Safely zeros output, restores controller, stops services, closes IO/session/fleet, and clears started flag. |
| `_resolve_dt(dt)` | supplied period or monotonic clock | positive float seconds | Uses supplied deterministic dt or elapsed monotonic time with loop-rate fallback. |
| `_write_zero(source)` | audit source string | zero `ControlInput` | Attempts final zero write; tolerates an already-closed IO adapter. |
| `_compute_control_input(estimate, target, dt, fleet_step)` | estimate/reference/period/fleet result | `ControlInput` | Holds during fleet build, uses fleet controller when appropriate, aborts unsupported fleet mode, otherwise uses normal controller. |
| `_rollback_startup(started)` | successfully started services | none | Stops/closes services in reverse order and closes IO after startup failure. |

## 3. Special data and cross-references

`RuntimeTelemetry.sensor_data` is measured [[vehicle-types|SensorData]];
`estimate` is observer state; `target` is planner/fleet
`ControllerReference`; `command` is the final `ControlInput` written or forced
to zero. `dt` is seconds and `state` is [[vehicle-state-machine|State]] after
the iteration.

The actual order is command drain → optional simulation tick → IO cache/read →
observer → fleet cycle → planner/fleet target → completion decision → controller
→ IO write → monitoring. Any exception aborts fleet when present, marks ERROR,
attempts zero, then raises `RuntimeError`.

## 4. Position in the project

[[vehicle-process|run_vehicle_process]] is the normal caller. This is the
only layer joining [[vehicle-state-machine|StateMachine]] drive permission to
final [[io-base|IOBase]] output; utilities cannot bypass it.

## 5. Use and verification

Build through [[module-factory|build_vehicle_modules]], call `start()`, call
`step()` repeatedly, and always call `shutdown()` in `finally`. Verify ordered
calls, profile selection, error zeroing, fleet holds, and shutdown with
`test/unit_test_vehicle_runtime.py` and integration control-loop tests.
