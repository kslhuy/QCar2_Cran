# `utils/control/controller/controller_fleet/controller_fleet_base.py`

## 1. Introduction

`ControllerFleetBase` supplies common configuration and fleet capability for follower controllers.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ControllerFleetBase(config, vehicle_id=0, logger=None)` | Gap/headway/gain/velocity/throttle profile | Fleet controller base | Stores common follower limits and gains. |
| `supports_fleet_reference` | Controller state | `True` | Declares compatibility with predecessor-derived references. |
| `reset()` | No inputs | No side effect | Fleet variants have no common retained control memory. |
| `_clip(value, lower, upper)` | Numeric value and bounds | Bounded `float` | Static saturation helper shared by follower algorithms. |

## 3. Special data and cross-references

The same `VehicleStateEstimate`/`ControllerReference` contract applies; fleet references are explicitly capability-gated.

## 4. Position in the project

`ControllerManager` selects it only for fleet work; fleet lifecycle remains in [[fleet-manager|FleetManager]].

## 5. Use and verification

`test/unit_test_controller.py` and `test/unit_test_fleet.py` cover capability selection.
