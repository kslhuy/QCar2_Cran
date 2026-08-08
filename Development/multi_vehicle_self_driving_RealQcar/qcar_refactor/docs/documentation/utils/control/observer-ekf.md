# `utils/control/observer/observer_ekf.py`

## 1. Introduction

`ObserverEKF` wraps the local `_EKF` model to fuse motion/IMU/GPS into a vehicle estimate.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `_wrap(angle)` | Heading angle | Angle in `[-pi, pi)` | Normalizes EKF heading and GPS innovation. |
| `ObserverEKF(config, vehicle_id=0, logger=None, wheelbase=None)` | Profile and optional wheelbase override | EKF observer | Stores wheelbase and unstarted internal five-state EKF. |
| `start(initial_pose=None)` | Optional list/dict pose or configured pose | Initialized `_EKF` | Creates EKF with `[x, y, theta]` initial state. |
| `update(sensor_data, dt, last_command=None)` | Sensor snapshot, step, optional last command | Assessed `VehicleStateEstimate` or `RuntimeError` | Fuses tach/gyro/horizontal acceleration and optional GPS with command-derived steering/throttle. |
| `get_latest()` | Internal EKF state | Current `VehicleStateEstimate` | Returns current filter state or safe invalid default before start. |
| `stop()` | No inputs | Unstarted flag | Marks observer stopped. |
| `_horizontal_acceleration(accel)` | IMU acceleration sequence | Horizontal acceleration magnitude | Computes `hypot(ax, ay)` and ignores Z/gravity. |
| `_EKF(initial_pose=None, wheelbase=0.3)` | Optional pose and bicycle wheelbase | Internal 5D filter | Initializes `[x, y, theta, v, a]`, covariance, and process/measurement noise. |
| `_EKF.update(motor_tach, gyro_z, dt, accel_x=0.0, gps=None, steering=None, throttle=0.0)` | Motion/IMU/GPS/command measurements | Updated state/covariance | Predicts bicycle/gyro kinematics, then applies GPS or tach/gyro/acceleration Kalman correction. |
| `_EKF.get_state()` / `_EKF.reset(initial_pose=None)` | Filter state / optional pose | State copy / reset filter state | Exposes copied state or restores initial covariance/state. |

## 3. Special data and cross-references

This module participates in the ordered [[vehicle-runtime|VehicleRuntime]] control loop and uses the shared [[vehicle-types|vehicle contracts]].

The internal state is planar pose/velocity and covariance; GPS validity controls correction while IMU/tachometer drive prediction.

## 4. Position in the project

Selected through `ObserverManager`; it does not decide GPS calibration or LiDAR localization policy.

## 5. Use and verification

`test/unit_test_vehicle_observer.py` verifies fusion and estimate validity.
