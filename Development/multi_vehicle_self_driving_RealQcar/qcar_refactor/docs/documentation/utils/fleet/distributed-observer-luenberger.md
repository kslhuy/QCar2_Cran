# `utils/fleet/.../distributed_observer_luenberger.py`

## 1. Introduction

`DistributedObserverLuenberger` is an advisory bounded prediction/correction prototype, not a controller-ready implementation.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `DistributedObserverLuenberger(config, vehicle_id, logger=None)` | Gain profile, local ID, optional logger | Luenberger observer | Validates/stores position, heading, velocity, and acceleration correction gains. |
| `update(local_estimate, peer_snapshots, membership_revision, measurements, dt)` | Local/peer estimates, revision, measurements, step | `DistributedFleetEstimate` | Carries predictions across cycles and corrects available local/peer measurements. |
| `_correct(previous, measurement, dt)` | Previous advisory estimate, measurement, step | Corrected `VehicleStateEstimate` | Blends state components with configured gains and wraps heading. |
| `_gain(config, key, default)` / `_wrap(angle)` | Profile scalar/key/default or angle | Validated gain / wrapped angle | Validate finite gain and normalize heading to `[-pi, pi)`. |

## 3. Special data and cross-references

Internal state is keyed by vehicle ID; prediction uses previous velocity/acceleration and yaw, correction uses current measurement. Output source is DISTRIBUTED_OBSERVER.

## 4. Position in the project

Called only through [[fleet-manager|FleetManager]] after peer validation; it remains advisory and does not select controller output.

## 5. Use and verification

`test/unit_test_fleet.py` verifies gain validation, lifecycle, prediction/correction.
