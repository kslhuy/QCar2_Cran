# `utils/control/path_planner/path_planner_base.py`

## 1. Introduction

`PathPlannerBase` defines reference generation; `PathPlannerNull` produces a stopped safe reference.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `PathPlannerBase(config, vehicle_id=0, logger=None)` | Planner profile, local ID, optional logger | Base planner state | Copies configuration and stores identity/logger. |
| `load_path(path_source)` | Path file or iterable points | Planner state side effect | Abstract route-loading contract. |
| `reset()` | Planner progress state | Reset side effect | Abstract route-progress reset. |
| `update(state)` | Current `VehicleStateEstimate` | `ControllerReference` | Abstract reference-generation contract. |
| `set_target_velocity(target_velocity)` | Desired speed | Planner state side effect | Abstract speed-setting contract. |
| `is_finished()` | Planner progress state | `bool` | Abstract route-completion query. |
| `PathPlannerNull(config, vehicle_id=0, logger=None)` | Base configuration | Null planner | Initializes a finished safe planner. |
| `PathPlannerNull.load_path(path_source)` / `reset()` | Any path/no inputs | Finished state | Keep null planner finished. |
| `PathPlannerNull.update(state)` | Current estimate | Finished zero-velocity `ControllerReference` | Holds current pose and signals safe completion. |
| `PathPlannerNull.set_target_velocity(target_velocity)` / `is_finished()` | Desired speed / state | Stored speed / `True` | Retains configured speed but always reports finished. |

## 3. Special data and cross-references

`update` maps an estimate to [[vehicle-types|ControllerReference]]; completion is a planner signal, not an actuator command.

## 4. Position in the project

Called by `PathPlannerManager` from runtime; it does not transition vehicle state.

## 5. Use and verification

`test/unit_test_path_planner.py` covers base/null expectations.
