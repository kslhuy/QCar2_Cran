# `extra/ground_station/core/command_handler.py`

## 1. Introduction

Parses one terminal line into a typed `VehicleCommand` request and routes it to
the already-connected vehicle session.

## 2. Code structure

| Definition                                                           | Inputs                                                 | Output                                          | Algorithm or purpose                                                                                   |
| -------------------------------------------------------------------- | ------------------------------------------------------ | ----------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `GroundStationCommandRequest(action, vehicle_id=None, command=None)` | Parsed action, optional target, optional typed command | Frozen request record                           | Represents terminal intent; command-less actions include `help`, `list`, `status`, and `manual-drive`. |
| `GroundStationCommandHandler.parse(text)`                            | Terminal command line                                  | `GroundStationCommandRequest` or `ValueError`   | Shell-splits and validates lifecycle, fleet, manual, velocity, path, and SDCS-map syntax.              |
| `GroundStationCommandHandler.route(server, request)`                 | Server and a command-bearing request                   | `CommandDelivery` or `ValueError`               | Delegates delivery to `GroundStationServer.send_command`.                                              |
| `_required_vehicle_id(parts)` / `_vehicle_id(value)`                 | Token list or textual ID                               | Non-negative `int` or `ValueError`              | Require and validate the target vehicle ID.                                                            |
| `_loop(value)`                                                       | Textual loop value                                     | `0`, `1`, `2`, or `inf`; otherwise `ValueError` | Normalizes the SDCS-map loop mode.                                                                     |

## 3. Special data and cross-references

`parse` constructs [[commands|VehicleCommand]] with a local source. It only
checks terminal syntax; [[core/command-handler|CommandHandler]] inside the vehicle
decides whether the requested operation is eligible.

## 4. Position in the project

This is the operator-side syntax boundary. It routes through
[[server|GroundStationServer]], never imports a runtime, state machine, fleet
manager, or IO adapter.

## 5. Use and verification

`test/unit_test_ground_station.py` covers parsing, SDCS loop validation, and
command routing; `test/test_integration_ground_station.py` covers delivery to a
connected vehicle.
