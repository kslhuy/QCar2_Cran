# `utils/fleet/fleet_peer_store.py`

## 1. Introduction

`FleetPeerStore` is the process-local validated cache of current-membership peer estimates.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetPeerStore(registry, vehicle_id)` | Shared registry and local vehicle ID | Peer-store state | Tracks expected inbound peers and bounded decode/drop counters. |
| `ingest(messages)` | Generic V2V messages | Updated fresh peer snapshots/counters | Decodes eligible fleet messages and accepts current-revision, sequence-monotonic peer state. |
| `prune_stale(now_monotonic)` | Local monotonic time | Updated freshness state | Removes snapshots beyond formation communication timeout. |
| `snapshots()` / `predecessor_snapshot()` | Cache state | Ordered snapshots / predecessor or `None` | Expose current remote measurements. |
| `all_expected_fresh()` / `peer_health()` | Expected peer/cache state | `bool` / `(peer_id, fresh)` tuple | Report local inbound-peer health. |
| `counters()` / `clear()` | Cache state | Counter copy / cleared cache | Expose diagnostics or discard peer state. |
| `_sync_membership(revision)` | Registry membership revision | Updated expected peers/cache | Clears/rebuilds peer expectations when membership changes. |

## 3. Special data and cross-references

Freshness uses `received_at_monotonic` on this receiver only; remote source clocks are never compared. Counters distinguish malformed, unexpected, wrong-role, obsolete, invalid, out-of-order, gap, and stale data.

## 4. Position in the project

Called by [[fleet-manager|FleetManager]] after generic V2V drain; it cannot change formation policy or drive.

## 5. Use and verification

`test/unit_test_fleet.py` and `test/unit_test_v2v.py` verify decode, freshness, revision reset, and counters.
