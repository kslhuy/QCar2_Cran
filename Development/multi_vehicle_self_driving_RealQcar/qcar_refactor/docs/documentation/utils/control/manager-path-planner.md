# `utils/control/managers/manager_pathplanner.py`

## 1. Introduction

`PathPlannerManager` delegates one selected planner and gates SDCS-specific node routes.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `PathPlannerManager(configured_planner, builders=None)` | Configured `PathPlannerBase` and optional builders | Planner manager | Initializes generic selection with planner contract. |
| `load_path(path_source)` | Path file or iterable waypoint source | Active planner side effect | Delegates generic route loading. |
| `reset()` | Active planner | Reset side effect | Resets route-progress state. |
| `update(state)` | Current `VehicleStateEstimate` | `ControllerReference` | Delegates reference generation. |
| `set_target_velocity(target_velocity)` | Desired speed | Active planner side effect | Delegates target-speed update. |
| `set_node_sequence(node_sequence, *, loop=0)` | SDCS node sequence and loop policy | Active planner side effect or `ValueError` | Requires selected planner's SDCS setter before delegation. |
| `is_finished()` | Active planner | `bool` | Delegates route-completion query. |
| `_validate_utility(utility, name)` | Candidate utility | Validation side effect or `TypeError` | Requires normal path-planner methods. |

## 3. Special data and cross-references

`update` returns [[vehicle-types|ControllerReference]]; SDCS node IDs/loop are planner inputs, not command semantics.

## 4. Position in the project

Runtime/command handler selects via this manager; it cannot drive vehicle.

## 5. Use and verification

`test/unit_test_path_planner.py` and command-handler tests verify selection.
