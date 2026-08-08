# `utils/fleet/fleet_message.py`

## 1. Introduction

This file encodes/decodes fleet-owned state-estimate payloads inside generic V2V envelopes.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetMessageError(FleetError)` | Invalid fleet V2V payload | Fleet-message exception | Distinguishes codec validation failures. |
| `encode_vehicle_state_estimate(estimate, status)` | Local `VehicleStateEstimate` and fleet status | Versioned payload mapping | Encodes formation/revision/source/timestamp/validity and state for generic V2V transport. |
| `decode_vehicle_state_estimate(message)` | Generic `V2VMessage` | Validated `FleetPeerSnapshot` or `FleetMessageError` | Requires fleet-state schema and reconstructs remote snapshot. |
| `_finite_number(value, name)` / `_non_negative_int(value, name)` | Raw payload field | Validated float/int or `FleetMessageError` | Validate numeric state/timestamp and sequence/revision fields. |

## 3. Special data and cross-references

`VEHICLE_STATE_ESTIMATE` is the only codec type and schema version is 1. The payload has membership revision, member order, role, source time, validity, and estimate `[x,y,theta,velocity,acceleration,gps_valid]`.

## 4. Position in the project

[[fleet-manager|FleetManager]] uses this codec with [[v2v-base|V2VBase]]; codec code never sends packets.

## 5. Use and verification

`test/unit_test_fleet.py` verifies schema, role, numeric, and timestamp rejection.
