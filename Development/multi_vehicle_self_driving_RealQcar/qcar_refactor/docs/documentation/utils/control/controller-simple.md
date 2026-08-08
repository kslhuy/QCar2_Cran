# `utils/control/controller/controller_simple.py`

## 1. Introduction

`ControllerSimple` is the normal path-tracking controller: longitudinal PI-like speed control plus geometric steering.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ControllerSimple(config, vehicle_id=0, logger=None, **overrides)` | PID, feed-forward, steering, and bound settings | Path controller | Merges overrides and initializes velocity integral/derivative memory. |
| `reset()` | No inputs | Cleared integral and previous error | Resets controller memory. |
| `compute(state, target, dt)` | Estimate, planner reference, loop step | Bounded `ControlInput` | Stops/reset on finished target; otherwise combines velocity PID-like throttle and heading steering. |
| `zero_command(source='zero')` | Source label | Zero `ControlInput` | Produces an explicit safe command. |
| `integral_error` | Internal velocity memory | `float` | Exposes accumulated speed error for inspection/tests. |
| `_compute_throttle(current_velocity, target_velocity, dt)` | Current/target speed and safe step | Clipped throttle | Applies feed-forward plus P/I/D terms with integral and output limits. |
| `_compute_steering(state, target)` | Estimate and reference pose | Clipped steering | Uses target-point heading, or target yaw at coincident position. |
| `_clip(value, lower, upper)` / `_wrap_to_pi(angle)` | Scalar/bounds or angle | Bounded/wrapped `float` | Saturate commands and normalize heading error to `[-pi, pi)`. |

## 3. Special data and cross-references

It consumes estimated pose/speed and a planner reference, retains an integral speed error, and returns normalized [[vehicle-types|ControlInput]].

## 4. Position in the project

Used through `ControllerManager` by [[vehicle-runtime|VehicleRuntime]]; it does not choose waypoints or send output itself.

## 5. Use and verification

`test/unit_test_controller.py` verifies bounded throttle, steering, and reset.
