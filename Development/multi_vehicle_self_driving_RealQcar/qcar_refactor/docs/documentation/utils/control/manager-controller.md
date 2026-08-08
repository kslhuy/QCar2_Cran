# `utils/control/managers/manager_controller.py`

## 1. Introduction

`ControllerManager` adds controller delegation, manual input, and fleet-capability validation.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ControllerCapabilityError(RuntimeError)` | Fleet-incompatible selected profile | Capability exception | Signals that a normal controller cannot consume fleet reference. |
| `ControllerManager(configured_controller, builders=None)` | Configured `ControllerBase` and optional builders | Controller manager | Initializes generic selection with controller contract. |
| `supports_fleet_reference` / `uses_planner_completion` | Active controller | Capability `bool` | Exposes active controller declarations with safe defaults. |
| `select(name)` | Allowed controller profile | Reset selected `ControllerBase` | Selects via base manager then resets controller memory. |
| `reset()` | Active controller | Reset side effect | Resets without changing selected profile. |
| `set_input(throttle, steering)` | Direct manual axes | Active-controller side effect or `RuntimeError` | Delegates only when active controller supports a callable setter. |
| `input_age_s()` | Active controller | Age `float` or `None` | Returns manual input age only when supported. |
| `compute(state, target, dt)` | Estimate, reference, step | `ControlInput` | Delegates normal controller computation. |
| `compute_fleet(state, target, dt)` | Fleet estimate/reference/step | `ControlInput` or `ControllerCapabilityError` | Gates fleet computation on active capability. |
| `_validate_utility(utility, name)` | Candidate utility | Validation side effect or `TypeError` | Requires callable `reset` and `compute`. |

## 3. Special data and cross-references

Manual axes are normalized input; fleet reference capability prevents incompatible controller use. See [[vehicle-types|control data]].

## 4. Position in the project

Runtime invokes manager; it never writes IO or decides state permission.

## 5. Use and verification

`test/unit_test_controller.py` and runtime fleet/manual tests verify delegation.
