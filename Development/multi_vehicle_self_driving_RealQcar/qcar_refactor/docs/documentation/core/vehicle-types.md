# `core/vehicle_types.py`

## 1. Introduction

This file defines the cross-layer data records. It separates sensor
measurements, observer estimates, controller references, actuator requests,
and generic V2V envelopes so no utility can silently substitute one meaning for
another.

## 2. Code structure

All definitions are dataclasses initialized directly from their listed fields;
none contains control or transport algorithms.

| Definition | Inputs | Output | Purpose |
| --- | --- | --- | --- |
| `SensorData` | tachometer, gyro, accelerometer, sensor timestamp, GPS valid/pose/timestamp | mutable measurement record | Carries normalized raw sensor data from IO. |
| `VehicleStateEstimate` | timestamp, pose, velocity, acceleration, GPS validity, overall validity | mutable estimate record | Carries observer state used for control/fleet publication. |
| `ControlInput` | throttle, steering, target velocity, optional source | mutable command record | Carries controller demand before [[io-base|IOBase]] clips it. |
| `ControllerReference` | target x/y/yaw, target velocity, optional finished flag | mutable reference record | Carries planner or fleet target for a controller. |
| `V2VMessage` | sender/type/payload/sequence/send times/optional receive times | frozen message envelope | Carries generic peer transport data without fleet-specific interpretation. |

## 3. Special data and cross-references

`SensorData.motor_tach` is signed longitudinal speed or documented backend
approximation in m/s; `gyro_z` is yaw rate rad/s; `accelerometer=[ax, ay, az]`
in m/s²; `gps_position=[x_m, y_m, yaw_rad]`; timestamps are seconds; and
`gps_valid` means that GPS sample is usable.

`VehicleStateEstimate` has local pose `[x, y, theta]`, velocity m/s,
acceleration m/s², source timestamp, `gps_valid`, and overall `valid` health
(which is not identical to current GPS availability). `ControlInput.throttle`
and `.steering` are backend-normalized, `.target_velocity` is m/s, and `.source`
is audit text. `ControllerReference` names desired planar pose/speed and
`is_finished` completion. `V2VMessage` distinguishes sender sequence/send time
from local receiver timestamps; payload semantics belong to
[[fleet-manager|FleetManager]].

## 4. Position in the project

[[io-base|IOBase]] produces `SensorData`; observers produce
`VehicleStateEstimate`; planner/fleet produce `ControllerReference`; controllers
produce `ControlInput`; V2V produces `V2VMessage`. [[vehicle-runtime|VehicleRuntime]]
is the only component that orders those exchanges.

## 5. Use and verification

Construct records by keyword and preserve the documented units/validity flags.
Their use is verified across `test/unit_test_vehicle_io.py`,
`unit_test_vehicle_observer.py`, `unit_test_controller.py`,
`unit_test_path_planner.py`, `unit_test_v2v.py`, and runtime/fleet tests.
