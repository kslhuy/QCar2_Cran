# `core/vehicle_process.py`

## 1. Introduction

This file supplies the portable one-process lifecycle used by direct execution
and simulator workers. It transforms a serializable process specification into
one live [[vehicle-runtime|VehicleRuntime]] and guarantees shutdown.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `VehicleProcessSpec` | ID, config file, selection/value overrides, resources, optional fleet spec | frozen launch record | Keeps worker arguments serializable and makes external resources explicit. |
| `build_vehicle_process_runtime(spec, logger)` | `VehicleProcessSpec` | `VehicleRuntime` | Enforces matching vehicle ID override, resolves config, builds modules, optionally creates/attaches local fleet, and composes runtime. |
| `run_vehicle_process(runtime, cycles, dt, on_ready, on_running, on_step, collect_telemetry, should_stop)` | live runtime, loop controls, optional callbacks | telemetry list | Starts runtime, signals READY, issues simulator START, steps until count/stop, invokes callbacks, and always shuts down. |

## 3. Special data and cross-references

`VehicleProcessSpec.resources` is a mapping for borrowed platform dependencies;
`fleet_spec` is process-local, so workers do not share a mutable fleet manager.
Callbacks receive runtime only at ready/running barriers and each telemetry
sample on step; they do not receive an alternate actuator interface.

## 4. Position in the project

[[vehicle-main|vehicle_main]] and `extra/simulator` call this module. It calls
[[vehicle-config|load_config]] then [[module-factory|build_vehicle_modules]];
the resulting runtime owns control-loop safety.

## 5. Use and verification

Create `VehicleProcessSpec(vehicle_id=..., vehicle_config_file=...)`, build it,
or call `run_vehicle_process` for a self-contained lifecycle. `cycles` must be
positive or `None`. Verify conflict detection, callbacks, collection, stop
predicate, and guaranteed shutdown in `test/unit_test_vehicle_process.py` and
multi-process virtual/CARLA tests.
