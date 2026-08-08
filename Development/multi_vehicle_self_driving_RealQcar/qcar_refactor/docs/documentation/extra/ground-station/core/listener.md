# `extra/ground_station/core/listener.py`

## 1. Introduction

Headless listener implementation selected by the canonical ground-station
side-program entry point.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `main(argv: list[str] \| None = None) -> int` | Local config path, optional listener overrides, optional duration | Process status code | Loads operator listener settings, starts `GroundStationServer`, waits for timeout or Ctrl-C, and always stops it. |

## 3. Special data and cross-references

The local application config and optional `--host`, `--port`, and `--duration-s`
identify the listener only. No vehicle configuration, controller, or actuator
data is accepted here.

## 4. Position in the project

It owns the process lifetime above [[server|GroundStationServer]]; the
vehicle-side facade and [[vehicle-runtime|VehicleRuntime]] retain command and
safety authority.

## 5. Use and verification

`test/test_integration_ground_station.py` exercises the listener lifecycle and
registration path.
