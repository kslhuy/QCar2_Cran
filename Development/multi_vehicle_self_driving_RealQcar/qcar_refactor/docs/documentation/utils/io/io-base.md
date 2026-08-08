# `utils/io/io_base.py`

## 1. Introduction

`IOBase` is the abstract IO contract used by [[vehicle-runtime|VehicleRuntime]].
It owns the thread-safe [[vehicle-types|SensorData]] cache, configured polling
cadence, command clipping, and idempotent safe stop/close behavior. It leaves
hardware reads and writes to a concrete adapter. `IONull` is the safe concrete
implementation for headless and isolated tests.

## 2. Code structure

`IOBase(config, vehicle_id=0, logger=None)` requires `config.read` sensor/GPS
rates and `config.write` throttle/steering limits. It stores copied config,
vehicle ID, locks, caches, and lifecycle flags. `IONull` has the same inputs
but overrides limits to accept the normal test command range and owns no device.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `IOBase._poll_sensors()` | cache under lock | cache mutation | Abstract backend hook for tachometer, gyro, accelerometer, and timestamp. |
| `read_sensors()` | none | none | Polls only when the configured sensor period has elapsed. |
| `IOBase._poll_gps()` | cache under lock | cache mutation | Abstract backend hook for GPS validity, pose, and timestamp. |
| `read_gps()` | none | none | Polls only when the configured GPS period has elapsed. |
| `read_to_cache()` | none | none | Runs both rate-limited cache refreshes. |
| `read()` | none | copied `SensorData` | Returns a copy so callers cannot mutate the shared cache. |
| `write(command)` | `ControlInput` | hardware side effect | Rejects closed adapters, clips throttle/steering, caches the bounded command, and calls the backend hook. |
| `get_last_command()` | none | copied `ControlInput` | Returns the last bounded command for diagnostics. |
| `IOBase._hardware_write(t, s)` | bounded throttle, steering | hardware side effect | Abstract actuator hook. |
| `stop()` | none | zero hardware write | Once-only safe zero output and `safe_stop` cache record. |
| `close()` | none | none | Stops once, releases adapter-local resources, and forbids future writes. |
| `_close_impl()` | none | none | Optional concrete cleanup; externally owned sessions remain external. |
| `_default_sensor_data()` / `_default_gps_data()` | none | field dictionaries | Supply zero/invalid cache defaults. |
| `_clip(value, lo, hi)` | numeric value and bounds | bounded numeric value | Saturates a backend request. |
| `IONull._poll_sensors()` / `_poll_gps()` | cache under lock | safe cache mutation | Publishes zero IMU/tachometer and invalid GPS. |
| `IONull._hardware_write(t, s)` | bounded values | none | Intentionally discards commands. |

## 3. Special data and cross-references

`_sensor_data_cache` is the mutable producer-side form of
[[vehicle-types|SensorData]]; its GPS tuple is `[x_m, y_m, yaw_rad]` when
valid. `_command_cache` is a bounded [[vehicle-types|ControlInput]], not the
raw controller demand, so diagnostics represent what an actuator was allowed
to receive. `_cache_lock` guards both data objects, while `_lifecycle_lock`
serializes shutdown against output writes.

Polling currently compares wall-clock `time.time()` against cached timestamps;
concrete backends may publish their own measurement timestamp. Consumers must
therefore use the documented validity and freshness behavior rather than assume
a common sensor clock.

## 4. Position in the project

[[vehicle-runtime|VehicleRuntime]] is the direct caller and owns when zero
output is required. Concrete implementations in [[utils/io/README|IO Utilities]]
implement the protected hooks. This file deliberately does not create a
vehicle, manage ROS executors, tick CARLA, calibrate sensors, or decide whether
the vehicle may move.

## 5. Use and verification

Implement only the protected hooks for a normal adapter:

```python
class BackendIO(IOBase):
    def _poll_sensors(self): ...      # update _sensor_data_cache under lock
    def _poll_gps(self): ...
    def _hardware_write(self, throttle, steering): ...
```

`test/unit_test_vehicle_io.py` verifies rate gating, defensive read copies,
clipping, stop/close idempotence, and `IONull`. A backend test must also show
that `stop()` produces the backend's zero command and that writes after
`close()` fail safely.
