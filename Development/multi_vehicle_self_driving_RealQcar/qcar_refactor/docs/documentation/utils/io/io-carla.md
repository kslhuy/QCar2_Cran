# `utils/io/io_carla.py`

## 1. Introduction

`IOCarla` adapts one ego actor from an externally owned CARLA session to
[[io-base|IOBase]]. It reads the session snapshot after a tick and maps a
bounded project command to CARLA control. It never starts, ticks, destroys, or
otherwise owns the CARLA session.

## 2. Code structure

`IOCarla(config, session, vehicle_id=0, logger=None, control_factory=None)`
requires a session exposing `ego_actor`, `get_snapshot()`, and vehicle-control
creation. Config optionally supplies steering scale/sign. `control_factory` is
an injectable test seam; otherwise the session builds the CARLA control object.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `_wrap_radians(angle)` | radians | wrapped radians | Normalizes heading to `[-pi, pi)`. |
| `read_to_cache()` | session state after tick | cache mutation | Refreshes sensors and GPS once; it bypasses base rate gating because CARLA tick owns sampling. |
| `_poll_sensors()` | actor transform/velocity, snapshot | cache mutation | Projects velocity onto actor forward direction and publishes IMU fields. |
| `_poll_gps()` | actor transform, snapshot | cache mutation | Converts CARLA pose to project `[x, -y, -yaw]` coordinates. |
| `_hardware_write(throttle, steering)` | bounded request | `actor.apply_control` | Maps positive throttle to CARLA throttle, negative throttle to brake, and transforms steering scale/sign. |

## 3. Special data and cross-references

CARLA's Y axis and yaw convention are converted by negating both values before
they enter [[vehicle-types|SensorData]]. This is the project-frame pose used
by observers and planners; scenario authors must not mirror it a second time.
The session snapshot supplies sensor timestamps and IMU values, while the actor
supplies geometric pose and velocity.

## 4. Position in the project

The CARLA scenario/session in `extra/simulator` owns actor lifecycle and calls
the runtime only after ticking. [[module-factory|module_factory]] injects the
session, and [[vehicle-runtime|VehicleRuntime]] treats this adapter as an
ordinary [[io-base|IOBase]]. The adapter has no authority to advance the world
or decide global lifecycle transitions.

## 5. Use and verification

Select `io: carla` with a valid externally bootstrapped simulation session.
The runtime tick order must be session tick, `read_to_cache()`, control, then
`write()`. `test/unit_test_io_carla.py` verifies frame conversion and commands;
`test/test_integration_control_carla.py` and
`test/test_integration_carla_sdcs_path.py` exercise the complete CARLA route.
If ego spawn is unavailable during startup, the write hook returns without
actuating; the session owner must surface the startup failure.
