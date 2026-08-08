# `utils/control/controller/controller_fleet/controller_fleet_longitudinal.py`

## 1. Introduction

`ControllerFleetLongitudinal` controls longitudinal following from the common fleet controller base.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ControllerFleetLongitudinal.compute(state, target, dt)` | Follower estimate and predecessor reference; `dt` unused | Bounded `ControlInput` with zero steering | Computes desired gap using speed headway, selects X/Y longitudinal gap from predecessor heading, and applies proportional speed control. |

## 3. Special data and cross-references

This module participates in the ordered [[vehicle-runtime|VehicleRuntime]] control loop and uses the shared [[vehicle-types|vehicle contracts]].

The reference's target speed and spacing-derived longitudinal error are physical following inputs.

## 4. Position in the project

`FleetManager` supplies reference; runtime owns command write and safe stop.

## 5. Use and verification

`test/unit_test_controller.py` and `test/unit_test_fleet.py` cover follower behavior.
