# `utils/io/io_qcar2.py`

## 1. Introduction

`IOQCar2` is the direct Python/Quanser adapter for an already-created QCar and
optional GPS source. It conforms to [[io-base|IOBase]] while keeping physical
resource creation, calibration, and teardown outside the adapter. This
separation is essential: `IOQCar2` must not hide whether a run uses calibrated
GPS, LiDAR localization, or a device session owned elsewhere.

## 2. Code structure

`IOQCar2(config, qcar, gps=None, vehicle_id=0, logger=None)` requires a
non-null QCar object and optional GPS object. It stores references only; it
does not open or close them.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `__init__(config, qcar, gps, vehicle_id, logger)` | IO profile and live device objects | initialized adapter | Validates QCar presence and retains externally owned resources. |
| `_poll_sensors()` | QCar `read()` and attributes | cache mutation | Reads motor tachometer, Z gyro, 3-axis accelerometer, and a local timestamp. |
| `_poll_gps()` | optional `gps.readGPS()` | cache mutation | Publishes valid `[position[0], position[1], orientation[2]]` only for a successful GPS read. |
| `_hardware_write(throttle, steering)` | bounded request | `qcar.write(...)` | Sends the normalized final command to the Quanser API. |

Read exceptions are logged and leave the previous cache data intact except that
the failed/absent GPS path marks GPS invalid and timestamps that result.

## 3. Special data and cross-references

The QCar motor tachometer is exposed as `SensorData.motor_tach`; gyroscope and
accelerometer become `gyro_z` and `accelerometer[3]`. Successful GPS becomes
[[vehicle-types|SensorData]] `gps_position=[x, y, yaw]`; its coordinate frame
and transform to LiDAR/localization must be established by the physical
bootstrap/calibration procedure, not inferred in this file.

The QCar and GPS references are borrowed resources. `IOBase.close()` may stop
output but does not close them because this class does not implement local
resource cleanup.

## 4. Position in the project

[[module-factory|module_factory]] can select this adapter only when a caller
supplies bootstrapped device objects. [[vehicle-runtime|VehicleRuntime]]
calls it through [[io-base|IOBase]]; a future physical session owns QCar/GPS/
LiDAR construction, calibration artifact loading, TF/localization startup, and
cleanup. The adapter does not run SLAM, choose GPS versus LiDAR localization,
or decide whether autonomous movement is permitted.

## 5. Use and verification

Create physical resources first, then inject them:

```python
io = IOQCar2(io_config, qcar=created_qcar, gps=created_gps, vehicle_id=0)
```

`test/unit_test_io_qcar_fake.py` verifies the fake device path and missing-QCar
rejection. `test/unit_test_io_qcar_qlab.py` and
`test/unit_test_io_qcar_phys.py` are external-environment checks. Before an
autonomous physical run, add the calibration/freshness/safe-stop smoke tests
listed in [[TODO|Documentation and Physical-Vehicle TODO]].
