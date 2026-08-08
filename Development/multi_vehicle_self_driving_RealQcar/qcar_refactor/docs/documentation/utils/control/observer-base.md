# `utils/control/observer/observer_base.py`

## 1. Introduction

`ObserverBase` defines the state-estimator lifecycle; `ObserverNull` supplies a safe default estimate.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ObserverBase(config, vehicle_id=0, logger=None)` | Observer profile, local ID, optional logger | Base observer state | Copies configuration and stores identity/logger. |
| `start(initial_pose=None)` | Optional `[x, y, theta]` pose | Initialization side effect | Abstract estimator lifecycle start. |
| `update(sensor_data, dt, last_command=None)` | `SensorData`, step, optional prior `ControlInput` | `VehicleStateEstimate` | Abstract one-step state-estimation contract. |
| `get_latest()` | Observer state | Cached `VehicleStateEstimate` | Abstract non-recomputing estimate accessor. |
| `stop()` | Observer resources | Shutdown side effect | Abstract cleanup lifecycle hook. |
| `assess_estimate(estimate)` | Candidate estimate | Estimate with finite-value validity | Marks non-finite timestamp/state values invalid. |
| `ObserverNull(config, vehicle_id=0, logger=None)` | Base observer configuration | Null observer | Initializes a safe invalid zero-state cache. |
| `ObserverNull.start(initial_pose=None)` | Optional pose | Cached timestamp/pose side effect | Sets starting pose in the null cache. |
| `ObserverNull.update(sensor_data, dt, last_command=None)` | Sensor snapshot; step/command ignored | Invalid zero `VehicleStateEstimate` | Returns safe null estimate while retaining GPS-valid flag/time. |
| `ObserverNull.get_latest()` / `stop()` | Cached state / no inputs | Cached estimate / log side effect | Exposes cache and performs no resource cleanup. |

## 3. Special data and cross-references

This module participates in the ordered [[vehicle-runtime|VehicleRuntime]] control loop and uses the shared [[vehicle-types|vehicle contracts]].

An estimate contains pose, speed, validity and timestamp; assessment clamps/normalizes quality before controllers consume it.

## 4. Position in the project

`VehicleRuntime` owns update cadence; observers do not read hardware directly.

## 5. Use and verification

`test/unit_test_vehicle_observer.py` covers base/null behavior.
