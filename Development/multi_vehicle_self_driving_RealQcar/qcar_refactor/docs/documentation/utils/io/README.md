# IO Utilities

## 1. Introduction

`utils/io` is the sensor-and-actuator boundary for one
[[vehicle-runtime|VehicleRuntime]]. It gives the runtime a common cached
sensor read and bounded command write interface while isolating the difference
between a virtual model, CARLA, QCar hardware, and future ROS 2 transports.
It does not own vehicle lifecycle, planning, localization, or external session
startup.

## 2. File structure and variations

| Source | Main definitions | Role and variation |
| --- | --- | --- |
| [io_base.py](io-base.md) | `IOBase`, `IONull` | Template contract: rate-limited cache, clipping, safe stop, and idempotent close. `IONull` is the no-hardware implementation. |
| [io_virtual.py](io-virtual.md) | `IOVirtual` | Deterministic bicycle-model backend; it advances simulated time when read. |
| [io_carla.py](io-carla.md) | `IOCarla` | Converts a session-owned CARLA actor/snapshot to the common project frame; the external CARLA session owns ticks and teardown. |
| [io_qcar2.py](io-qcar2.md) | `IOQCar2` | Reads/writes already-created Quanser QCar and optional GPS objects; a physical bootstrap owns device creation and calibration. |
| [io_limo_ros2.py](io-ros-stubs.md) | `IOLimoROS2` | Empty extension point for a ROS 2 differential-drive backend. |
| [io_qcar2_ros2.py](io-ros-stubs.md) | `IOQCar2ROS2` | Empty extension point for a ROS 2 QCar backend. |
| `__init__.py` | lazy exports | Import convenience only; no additional runtime behavior. |

All concrete implementations obey [[io-base|IOBase]]: publish a cached
[[vehicle-types|SensorData]], accept a [[vehicle-types|ControlInput]], and
delegate only their hardware-specific poll/write boundary. They differ in
backend ownership, clock source, coordinate convention, and actuator API.

## 3. Shared data and cross-references

- [[vehicle-types|SensorData]] contains `motor_tach` (longitudinal speed-like
  tachometer), `gyro_z` (yaw rate), `accelerometer[3]` (body acceleration),
  sensor timestamp, GPS validity, `gps_position=[x, y, yaw]`, and GPS timestamp.
  The precise sensor source varies, but the physical interpretation exposed to
  the observer is stable.
- [[vehicle-types|ControlInput]] contains normalized `throttle`, `steering`,
  requested target velocity, and source. `IOBase.write()` clips throttle and
  steering before any backend receives them.
- The cache lock protects snapshots shared by the control loop and backend
  callbacks. The lifecycle lock protects `write`, `stop`, and `close` so a
  closed adapter cannot be actuated.

## 4. Position in the project

[[vehicle-runtime|VehicleRuntime]] calls `read_to_cache()`, `read()`, and
`write()` in its ordered control loop. [[module-factory|module_factory]]
chooses an implementation from the selected IO profile and injects any
externally owned session/device resource. `utils/io` is intentionally unable
to transition the global state machine or select a planner; the runtime owns
those decisions and calls zero output on unsafe paths.

## 5. Use and verification

Select `io: virtual`, `carla`, `qcar2`, or `null` through the active module
profile used by `core/module_factory.py`. A minimal loop is:

```python
io.read_to_cache()
sensor = io.read()
io.write(command)       # clips then performs the backend write
io.stop()                # explicit safe zero
```

Focused tests are `test/unit_test_vehicle_io.py`,
`test/unit_test_io_virtual.py`, `test/unit_test_io_carla.py`, and
`test/unit_test_io_qcar_fake.py`. The QLabs/physical checks in
`test/unit_test_io_qcar_qlab.py` and `test/unit_test_io_qcar_phys.py` require
the named external environment and are not ordinary unit-test prerequisites.

## Conclusion

Every IO implementation obeys the same cached-read and bounded-write contract;
it differs only at the hardware or simulator boundary, while lifecycle,
planning, and final safety authority remain with [[vehicle-runtime|VehicleRuntime]].
