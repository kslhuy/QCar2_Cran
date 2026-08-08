# `utils/control/controller/controller_base.py`

## 1. Introduction

`ControllerBase` is the injected controller interface; `ControllerNull` is its safe no-op implementation.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ControllerBase(config, vehicle_id=0, logger=None)` | Controller profile, local ID, optional logger | Base controller state | Copies configuration and stores identity/logger. |
| `supports_fleet_reference` | Selected controller state | `bool` | Defaults false; subclasses opt in to predecessor-derived references. |
| `uses_planner_completion` | Selected controller state | `bool` | Defaults true so a finished planner target stops autonomous control. |
| `reset()` | Controller memory | Reset side effect | Abstract lifecycle hook for integral/history state. |
| `compute(state, target, dt)` | `VehicleStateEstimate`, `ControllerReference`, positive loop step | `ControlInput` | Abstract one-step controller contract. |
| `ControllerNull(config, vehicle_id=0, logger=None)` | Base controller configuration | Null controller | Initializes the safe startup/test implementation. |
| `ControllerNull.reset()` | No inputs | No side effect | Safe no-op. |
| `ControllerNull.compute(state, target, dt)` | State/reference/step ignored | Zero `ControlInput` | Returns zero throttle/steering/velocity with `null_controller` source. |

## 3. Special data and cross-references

Inputs are [[vehicle-types|VehicleStateEstimate]] and `ControllerReference`; outputs are normalized `ControlInput`, later clipped by IO.

## 4. Position in the project

[[manager-controller|ControllerManager]] and runtime call the interface; controllers never read IO/write hardware.

## 5. Use and verification

`test/unit_test_controller.py` verifies null/base behavior.
