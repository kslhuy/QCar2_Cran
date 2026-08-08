# `utils/control/observer/observer_luenberger.py`

## 1. Introduction

This inactive historical module is byte-identical to `observer_ekf.py` and
exports `ObserverEKF`, not a Luenberger observer class. Current configuration
marks the `luenberger` profile unsupported.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `_wrap(angle)` | Heading angle | Angle in `[-pi, pi)` | Normalizes heading and GPS innovation. |
| `ObserverEKF(config, vehicle_id=0, logger=None, wheelbase=None)` | Profile and optional wheelbase override | EKF observer | This historical Luenberger module actually exports `ObserverEKF`; it contains the copied EKF implementation. |
| `start(initial_pose=None)` / `stop()` | Optional pose / no inputs | Initialized EKF / unstarted flag | Starts or marks stopped the copied observer lifecycle. |
| `update(sensor_data, dt, last_command=None)` | Sensor snapshot, step, optional command | Assessed `VehicleStateEstimate` or `RuntimeError` | Fuses tach/gyro/acceleration and optional GPS through the copied EKF. |
| `get_latest()` / `_horizontal_acceleration(accel)` | Filter state / IMU sequence | Estimate / horizontal magnitude | Reads filter state or derives planar acceleration. |
| `_EKF(initial_pose=None, wheelbase=0.3)` | Optional pose and wheelbase | Internal 5D filter | Initializes copied `[x, y, theta, v, a]` filter/covariance. |
| `_EKF.update(...)` | Tach, gyro, step, acceleration, optional GPS/command | Updated state/covariance | Applies bicycle/gyro prediction and GPS or IMU correction. |
| `_EKF.get_state()` / `_EKF.reset(initial_pose=None)` | Filter state / optional pose | State copy / reset state | Reads or resets copied filter. |

## 3. Special data and cross-references

If selected directly, its inputs/outputs would be [[vehicle-types|SensorData]]
to `VehicleStateEstimate`; no separate Luenberger algorithm is implemented.

## 4. Position in the project

[[module-factory|module_factory]] does not import this file and
`config_observer.yaml` explicitly rejects the Luenberger profile. It has no
current runtime owner; [[manager-observer|ObserverManager]] only manages
factory-exposed profiles.

## 5. Use and verification

No current focused test selects this unsupported implementation. The active EKF
contract is covered by `test/unit_test_vehicle_observer.py` and
`test/unit_test_ekf.py`; enabling this profile requires factory support and new
profile-specific tests.
