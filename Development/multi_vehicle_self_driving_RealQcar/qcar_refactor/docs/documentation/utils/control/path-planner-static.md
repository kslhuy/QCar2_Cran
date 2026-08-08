# `utils/control/path_planner/path_planner_static.py`

## 1. Introduction

`PathPlannerStatic` follows a fixed waypoint sequence with closest/look-ahead targeting.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `PathPlannerStatic(config, vehicle_id=0, logger=None, **overrides)` | Path source, speed, lookahead, tolerance, column settings | Static planner | Merges overrides, initializes empty `N x 3` waypoint array, and loads configured path when present. |
| `load_path(path_source)` | CSV path or iterable numeric points | Waypoint/progress state | Loads/coerces `[x, y, theta]` rows and resets progress/finish state. |
| `reset()` | No inputs | Initial progress/unfinished state | Clears index and recomputes empty-path completion. |
| `update(state)` | Current `VehicleStateEstimate` | `ControllerReference` | Selects monotonic closest/lookahead target; marks finished only at final waypoint within tolerance. |
| `set_target_velocity(target_velocity)` / `is_finished()` | Desired speed / planner state | Non-negative speed / `bool` | Changes requested speed or queries completion. |
| `current_index` / `waypoints` | Internal path state | Index / copied `ndarray` | Expose progress and prevent caller mutation of stored waypoints. |
| `_find_closest_index(state)` / `_find_lookahead_index(state)` | Current estimate | Waypoint index | Search bounded forward window, then first point at lookahead distance. |
| `_make_target(waypoint, is_finished)` | Waypoint and completion flag | `ControllerReference` | Builds zero-velocity finished target or active-speed target. |
| `_load_csv(path)` / `_parse_numeric_row(row)` | CSV path/row | Coerced rows / numeric row or `None` | Ignore non-numeric rows before waypoint coercion. |
| `_coerce_waypoints(path_source)` / `_heading_from_xy(x, y)` | Numeric row iterable / XY arrays | `N x 3` waypoint array | Accept 2/3/multi-column paths and derive headings from XY where needed. |
| `_distance_xy(...)` / `_log_info(message)` / `_log_warning(message)` | Coordinates or message | Distance / logger side effect | Supply geometric distance and compatible logger adapters. |

## 3. Special data and cross-references

This module participates in the ordered [[vehicle-runtime|VehicleRuntime]] control loop and uses the shared [[vehicle-types|vehicle contracts]].

Waypoints are numeric rows containing planar pose/reference data; current index and finish state make progress explicit.

## 4. Position in the project

Selected through `PathPlannerManager`; `VehicleRuntime` decides what completion means for motion.

## 5. Use and verification

`test/unit_test_path_planner.py` covers parsing, targets, and completion.
