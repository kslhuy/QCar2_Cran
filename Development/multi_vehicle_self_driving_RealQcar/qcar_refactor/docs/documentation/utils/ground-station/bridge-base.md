# `utils/ground_station/bridge_base.py`

## 1. Introduction

Defines the vehicle-side transport contract. The bridge transports queued typed
commands and monitoring; it never invokes runtime or IO methods.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `GroundStationBridgeBase.start()` | none | none | Abstract transport startup. |
| `GroundStationBridgeBase.stop()` | none | none | Abstract transport shutdown. |
| `GroundStationBridgeBase.drain_commands(limit)` | positive batch limit | `list[VehicleCommand]` | Abstract nonblocking command dequeue. |
| `GroundStationBridgeBase.publish_snapshot(snapshot)` | `MonitoringSnapshot` | none | Abstract monitoring publication. |
| `GroundStationBridgeBase.publish_ack(result)` | `CommandResult` | none | Abstract acknowledgement publication. |
| `GroundStationBridgeBase.get_status()` | none | diagnostics mapping | Abstract bridge health report. |
| `NullGroundStationBridge.__init__(config, vehicle_id, logger)` | optional profile/ID/logger | null bridge | Stores no transport resources. |
| `NullGroundStationBridge.start()` / `stop()` | none | `None` | Safe no-op lifecycle. |
| `NullGroundStationBridge.drain_commands(limit)` | batch limit | empty list | Supplies no operator commands. |
| `NullGroundStationBridge.publish_snapshot(snapshot)` / `publish_ack(result)` | snapshot/result | `None` | Discards outbound data. |
| `NullGroundStationBridge.get_status()` | none | disabled mapping | Reports no active bridge. |

## 3. Special data and cross-references

The bridge accepts [[commands|VehicleCommand]] only as queued intent and
emits `CommandResult`/`MonitoringSnapshot` as observation. The null variant
preserves this contract without opening a socket.

## 4. Position in the project

[[runtime-facade|GroundStationRuntimeFacade]] owns bridge use;
[[vehicle-runtime|VehicleRuntime]] supplies the command-application callback.
Neither bridge implementation can write [[io-base|IOBase]].

## 5. Use and verification

Factory selects null or TCP bridge. Verify null lifecycle and absence of
transport side effects in `test/unit_test_ground_station.py`.
