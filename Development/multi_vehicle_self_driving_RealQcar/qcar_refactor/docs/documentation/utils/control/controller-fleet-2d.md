# `utils/control/controller/controller_fleet/controller_fleet_2d.py`

## 1. Introduction

`ControllerFleet2D` tracks a two-dimensional fleet reference using pose and heading error.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `_wrap(angle)` | Heading error in radians | Angle in `[-pi, pi)` | Normalizes a follower heading error. |
| `ControllerFleet2D(config, vehicle_id=0, logger=None)` | Common fleet profile plus steering gain/limit | 2D follower controller | Initializes direct-predecessor tracking gains. |
| `compute(state, target, dt)` | Follower estimate and predecessor reference; `dt` unused | Bounded `ControlInput` | Creates a virtual target one desired gap behind predecessor, then computes gap-corrected speed and heading steering. |

## 3. Special data and cross-references

This module participates in the ordered [[vehicle-runtime|VehicleRuntime]] control loop and uses the shared [[vehicle-types|vehicle contracts]].

Reference pose and speed carry predecessor-relative desired motion; output is bounded `ControlInput`.

## 4. Position in the project

It is one follower controller behind `ControllerManager`, never a fleet coordinator.

## 5. Use and verification

`test/unit_test_controller.py` covers fleet control output.
