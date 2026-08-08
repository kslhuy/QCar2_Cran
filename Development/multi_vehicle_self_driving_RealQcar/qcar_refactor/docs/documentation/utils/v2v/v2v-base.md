# `utils/v2v/v2v_base.py`

## 1. Introduction

`V2VBase` defines generic peer-message transport. `V2VNull` implements the
same contract without networking for tests and single-vehicle runs. Payload
meaning and fleet policy remain outside this package.

## 2. Code structure

Both classes initialize with a copied config mapping, local vehicle ID, and
optional logger. `V2VBase` is abstract; `V2VNull` provides no-op concrete
behavior.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `V2VBase.__init__(config, vehicle_id, logger)` | profile, local ID, logger | base instance | Stores copied configuration, normalized ID, and logger. |
| `V2VBase.start()` | none | none | Abstract hook to acquire/start transport resources. |
| `V2VBase.publish(message_type, payload, target_vehicle_ids)` | message kind, mapping, optional peer IDs | boolean | Abstract request to deliver one generic message. |
| `V2VBase.drain_received()` | none | `list[V2VMessage]` | Abstract nonblocking retrieval of received messages. |
| `V2VBase.get_status()` | none | mapping | Abstract transport diagnostics. |
| `V2VBase.stop()` | none | none | Abstract release/stop hook. |
| `V2VNull.__init__(config, vehicle_id, logger)` | same base inputs | null adapter | Initializes the inherited safe no-op adapter. |
| `V2VNull.start()` | none | `None` | Performs no resource acquisition. |
| `V2VNull.publish(...)` | normal publish inputs | `False` | Delivers nothing. |
| `V2VNull.drain_received()` | none | empty list | Reports no peers/messages. |
| `V2VNull.get_status()` | none | disabled status mapping | Reports disabled/inactive zero-peer state. |
| `V2VNull.stop()` | none | `None` | Performs no cleanup. |

## 3. Special data and cross-references

`publish` receives a string message type, JSON-like payload mapping, and either
all configured peers (`None`) or explicit target IDs. `drain_received` returns
[[vehicle-types|V2VMessage]], whose sender, sequence, send time, and local
receive time remain distinct. The base contract does not interpret payload.

## 4. Position in the project

[[fleet-manager|FleetManager]] is the main consumer. [[module-factory|module_factory]]
selects null or UDP transport; neither implementation changes
[[vehicle-runtime|VehicleRuntime]] state or writes a vehicle command.

## 5. Use and verification

Select the null profile for offline/test runs, otherwise select UDP. Verify the
base/null contract with `test/unit_test_v2v.py`; concrete socket behavior is
covered by the UDP page's tests.
