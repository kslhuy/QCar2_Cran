# `utils/control/managers/manager_observer.py`

## 1. Introduction

`ObserverManager` delegates the selected observer lifecycle and update interface.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ObserverManager(configured_observer, builders=None)` | Configured `ObserverBase` and optional builders | Observer manager | Initializes generic selection with observer contract. |
| `start(initial_pose=None)` | Optional `[x, y, theta]` pose | Active observer startup side effect | Delegates estimator initialization. |
| `update(sensor_data, dt, last_command=None)` | `SensorData`, step, optional last `ControlInput` | `VehicleStateEstimate` | Delegates one estimation update. |
| `get_latest()` | Active observer state | Cached `VehicleStateEstimate` | Returns estimate without recomputation. |
| `stop()` | Active observer | Shutdown side effect | Delegates observer cleanup. |
| `_validate_utility(utility, name)` | Candidate utility | Validation side effect or `TypeError` | Requires callable start/update/get_latest/stop contract. |

## 3. Special data and cross-references

Input is [[vehicle-types|SensorData]] plus previous `ControlInput`; output is `VehicleStateEstimate`.

## 4. Position in the project

[[vehicle-runtime|VehicleRuntime]] owns call order and safety.

## 5. Use and verification

`test/unit_test_vehicle_observer.py` verifies manager delegation.
