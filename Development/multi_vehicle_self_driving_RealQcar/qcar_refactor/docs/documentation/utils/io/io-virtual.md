# `utils/io/io_virtual.py`

## 1. Introduction

`IOVirtual` is the deterministic, headless implementation of
[[io-base|IOBase]]. It is used by tests and virtual scenarios to expose a
noisy sensor stream from a configurable vehicle model without a simulator
process. [[vehicle-runtime|VehicleRuntime]] owns the control cadence; this
adapter advances exactly one fixed model sample for each `read_to_cache()`.

## 2. Code structure

`IOVirtual(config, vehicle_id=0, logger=None, **overrides)` takes the ordinary
IO limits/rates plus model parameters (step, wheelbase, mass, actuator time
constants, drag, noise, seed, and initial pose). Keyword overrides replace
top-level config values before initialization. It stores physical state,
actuator state, deterministic random generator, and last command.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `_wrap_to_pi(angle)` | radians | wrapped radians | Normalizes yaw to `[-pi, pi)`. |
| `read_to_cache()` | none | cache mutation | Advances one sample, then publishes sensor and GPS cache fields. |
| `_step_dynamics()` | last command | state mutation | Builds the six-state vector, advances RK4, clamps reverse speed, and increments simulated time. |
| `_rk4_step(state, throttle, steering, dt)` | state and command | next state | Classical fourth-order Runge-Kutta integration. |
| `_continuous_dynamics(state, throttle, steering)` | six-state vector | derivative vector | Kinematic bicycle yaw plus first-order actuator lag and longitudinal dynamics. |
| `_longitudinal_acceleration(v, motor)` | speed, motor state | acceleration | Drive acceleration minus rolling, viscous, and quadratic air drag. |
| `_poll_sensors()` | internal state | cache mutation | Publishes noisy tachometer, gyro, and 3-axis accelerometer. |
| `_poll_gps()` | internal pose and step count | cache mutation | Publishes noisy `[x, y, yaw]` only on configured GPS sample steps. |
| `_hardware_write(throttle, steering)` | bounded request | last-command mutation | Saves the command that will drive the next integration step. |
| `true_state()` | none | `(x, y, yaw, velocity, acceleration)` | Returns unnoised model state for assertions. |
| `actuator_state()` | none | `(motor_command, steering_actual)` | Returns lagged actuator state for assertions. |

## 3. Special data and cross-references

The integration state is `[x_m, y_m, yaw_rad, velocity_mps, motor_command,
steering_actual]`. The last two elements are normalized actuator states, so
they show response lag rather than instantaneous controller demand.
`gps_position=[x_m, y_m, yaw_rad]` is only valid every `gps_period_steps`; the
other [[vehicle-types|SensorData]] fields are emitted each sample. A seeded
NumPy generator makes the noise sequence repeatable for a fixed configuration.

## 4. Position in the project

The module is selected by [[module-factory|module_factory]] for virtual
profiles and consumed through [[io-base|IOBase]]. It substitutes for the
sensor/actuator boundary only; [[vehicle-runtime|VehicleRuntime]] still owns
the observer, planner, controller, command lifecycle, and safe zero output.

## 5. Use and verification

Set `io: virtual` and include a reproducible `seed` in the IO profile. Call
`read_to_cache()` once per control step before reading and writing. Verify with
`test/unit_test_io_virtual.py`; `test/test_integration_control_loop.py` covers
the adapter in a closed control loop. A stopped or closed instance inherits the
base class zero-output behavior.
