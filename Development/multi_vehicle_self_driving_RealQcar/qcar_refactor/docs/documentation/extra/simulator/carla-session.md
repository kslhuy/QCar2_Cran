# `extra/simulator/carla/session.py`

## 1. Introduction

CarlaSession owns CARLA client/world synchronous settings, ego/sensor actors, and sensor snapshots.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `CarlaSensorSnapshot(frame, timestamp, accelerometer=(0, 0, 0), gyro_z=0)` | Frame/timestamp and project-facing IMU values | Frozen sensor snapshot | Keeps latest sensor data free of direct CARLA object references. |
| `CarlaSession(config, client=None, carla_api=None, logger=None)` | Session configuration and optional injected CARLA client/API | Session instance | Owns all client, world, actor, sensor, and cleanup resources. |
| `ego_actor` / `get_snapshot()` | Session state | Spawned actor / thread-safe snapshot | Expose only required actor and immutable latest values. |
| `make_vehicle_control(throttle, steer, brake)` | Normalized control values | CARLA `VehicleControl` | Creates control object without exposing CARLA API handle. |
| `start()` | Configured session state | Spawned/started session or error | Initializes client/world, optionally syncs world, spawns actors, and warms up. |
| `tick()` | Tick-owner/follower world state | Updated `CarlaSensorSnapshot` | Advances world or waits for owner frame, then updates snapshot. |
| `stop()` / `close()` | Session/actor state | Brake request / permanently closed session | Brakes ego, destroys actors, restores settings, and prevents restart. |
| `_ensure_client()` / `_configure_synchronous_world()` / `_restore_world_settings()` | Configuration and CARLA world | Ready client/world or restored settings | Acquire dependency/client and safely record/apply/restore world settings. |
| `_spawn_ego()` / `_spawn_transform()` / `_spawn_sensors()` / `_make_transform(values)` | World, blueprint/configuration, transform values | Spawned actor(s) / CARLA transform | Create ego and enabled attached sensors. |
| `_sensor_callback(name)` / `_world_timestamp()` / `_snapshot_timestamp(snapshot)` | Sensor callback or world snapshot | Callback / project timestamp | Convert sensor/world data into timestamped project-facing snapshot values. |
| `_destroy_actor(actor)` / `_log(level, message, *args)` | Actor or optional logger data | Best-effort cleanup/log side effect | Clean up without masking lifecycle failures. |

## 3. Special data and cross-references

Snapshot contains timestamp, gyro, accelerometer; actor transform supplies pose. Session owns CARLA resources and must be stopped/restored after a run.

## 4. Position in the project

Injected into [[io-carla|IOCarla]] by factory; IO does not tick or destroy it.

## 5. Use and verification

`test/unit_test_carla_session.py` covers client/world configuration, snapshots,
tick-owner/follower behavior, safe stop, cleanup, and settings restoration.
`test/unit_test_io_carla.py` and CARLA integration tests cover its IO boundary.
