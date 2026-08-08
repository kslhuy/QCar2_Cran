# `utils/fleet/fleet_following_policy.py`

## 1. Introduction

`FleetFollowingPolicy` converts a valid predecessor snapshot into a follower reference.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetFollowingPolicy.make_reference(ego_estimate, predecessor, status)` | Local estimate, fresh predecessor snapshot, and fleet status | Fleet `ControllerReference` or `None` | Produces direct-predecessor reference only for active healthy followers; leader/no-peer/invalid states yield no override. |

## 3. Special data and cross-references

`FleetPeerSnapshot.estimate` supplies predecessor `[x,y,theta,velocity]`; the returned reference is fleet-owned and has `is_finished=False`.

## 4. Position in the project

[[fleet-manager|FleetManager]] calls it for active followers; controller/runtime retain actuation and safety ownership.

## 5. Use and verification

`test/unit_test_fleet.py` verifies inactive, missing, invalid, and valid-predecessor behavior.
