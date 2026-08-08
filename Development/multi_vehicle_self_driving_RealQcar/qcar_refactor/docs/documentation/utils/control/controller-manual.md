# `utils/control/controller/controller_manual.py`

## 1. Introduction

`ControllerManual` converts recent operator axes into a bounded command and rejects stale input through its configured timeout.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ControllerManual(config, vehicle_id=0, logger=None)` | Profile timeout/throttle/steering limits | Manual controller or `ValueError` | Validates limits, stores deadman timeout, and clears cached input. |
| `reset()` | No inputs | Zero axes and no receive time | Clears operator input state. |
| `uses_planner_completion` | Controller state | `False` | Manual operation ignores current planner completion. |
| `set_input(throttle, steering, now_monotonic=None)` | Operator axes and optional monotonic time | Cached clipped axes/timestamp | Bounds latest input for the next `compute`. |
| `input_age_s(now_monotonic=None)` | Optional monotonic time | Age in seconds or `None` | Returns local receive age for diagnostics/deadman logic. |
| `compute(state, target, dt)` | State/reference/step ignored for direct manual control | `ControlInput` | Returns cached command while fresh; returns zero `manual_timeout` command after timeout. |
| `_clip(value, lower, upper)` | Numeric value and bounds | Bounded `float` | Saturates axis value. |
| `_positive_float(value, name)` / `_unit_interval(value, name)` | Profile scalar | Validated `float` or `ValueError` | Require positive finite limits, with throttle constrained to `(0, 1]`. |

## 3. Special data and cross-references

Cached throttle/steering are normalized command requests and their monotonic timestamp is the safety freshness signal.

## 4. Position in the project

Selected by [[manager-controller|ControllerManager]] only after manual mode is armed; lifecycle remains in [[vehicle-runtime|VehicleRuntime]].

## 5. Use and verification

`test/unit_test_controller.py` and `test/unit_test_command_handler.py` cover timeout and manual eligibility.
