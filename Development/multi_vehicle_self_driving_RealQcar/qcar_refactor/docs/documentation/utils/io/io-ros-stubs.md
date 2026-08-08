# `utils/io/io_limo_ros2.py` and `utils/io/io_qcar2_ros2.py`

## 1. Introduction

`IOLimoROS2` and `IOQCar2ROS2` are currently empty subclasses of
[[io-base|IOBase]]; neither is a working ROS 2 interface and neither can be
selected by the module factory. They reserve the backend boundary for a
replaceable, open-source ROS localization/control stack without changing the
runtime IO contract.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `IOLimoROS2(IOBase)` | none today | none | Placeholder for a ROS 2 differential-drive adapter; steering semantics will differ from QCar. |
| `IOQCar2ROS2(IOBase)` | none today | none | Placeholder for a ROS 2 QCar adapter. |

Neither class currently defines an initializer or any required protected hook,
so instantiation would remain abstract and no ROS node, publisher, subscriber,
executor, or safety behavior exists.

## 3. Special data and cross-references

The completed adapters must translate ROS messages into
[[vehicle-types|SensorData]]: tachometer/speed, yaw rate, acceleration,
measurement timestamps, GPS/localized `[x, y, yaw]`, and explicit validity.
They must publish bounded [[vehicle-types|ControlInput]] to the selected QCar
command message. ROS time, receipt time, frame ID, namespace, and freshness
must be retained long enough to reject stale or cross-vehicle data.

For LiDAR localization, the transform chain must make the physical meaning
explicit: `map -> odom -> base_link -> lidar`, plus the chosen GPS/ENU frame.
The LiDAR-to-body/GPS extrinsic is a calibrated transform, not a SLAM output.

## 4. Position in the project

The future ROS adapter belongs below [[vehicle-runtime|VehicleRuntime]] and
above the ROS graph. [[module-factory|module_factory]] should select it by an
IO profile only after the adapter provides the same lifecycle guarantees as
[[io-base|IOBase]]. Cartographer or another open-source localization node
should remain an external producer of localization data; this adapter consumes
that data rather than owning planner or SLAM policy.

## 5. Use and verification

Implementation checklist before selection is enabled:

1. Constructor receives namespace/frame configuration and creates or receives a
   managed ROS node/executor.
2. Subscriptions update the cache under the IO lock and validate message
   timestamps, freshness, frame IDs, and vehicle namespace.
3. `_hardware_write()` publishes the bounded command; `stop()` and `close()`
   publish a confirmed zero command and stop only adapter-owned ROS resources.
4. A profile/factory branch injects the adapter without leaking ROS imports into
   `core`.
5. Unit tests use a fake ROS transport; integration tests verify topics, TF,
   stale-data behavior, namespace isolation, safe stop, and a calibrated
   localization drive.

These steps are tracked in [[TODO|Documentation and Physical-Vehicle TODO]].
