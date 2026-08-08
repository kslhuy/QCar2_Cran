# `extra/ground_station/presentation/dashboard.py`

## 1. Introduction

Renders a dependency-free, read-only terminal dashboard from server session
rows.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `GroundStationDashboard(stale_after_s=1.0)` | Positive local staleness threshold | Dashboard instance or `ValueError` | Stores the threshold used to mark connected sessions stale. |
| `render(rows, now_monotonic=None)` | Session-row mappings and optional local monotonic time | Multiline dashboard `str` | Renders connection, runtime, fleet, estimate, reference, V2V, and command-result columns. |
| `_counter(status, *keys)` | Status mapping and compatible counter keys | Integer counter | Returns the first integer counter present. |
| `_fleet_status(snapshot, fleet)` | Snapshot and fleet mappings | Display `str` | Formats validated fleet phase and leader/follower role. |
| `_control_reference(value)` / `_state_vector(snapshot)` | Reference or snapshot mapping | Display `str` | Formats values or reports unavailable data. |
| `_peer_freshness(fleet)` | Fleet mapping | Peer-age display `str` | Formats expected peer IDs with locally observed receive ages. |
| `_event(last_command, fleet)` | Last command text and fleet mapping | Event display `str` | Prefers the retained fleet reason over last command outcome. |

## 3. Special data and cross-references

Its input is the mapping returned by [[server|GroundStationServer.session_rows]].
Staleness is computed with local monotonic time; snapshot data and V2V counters
are presentation data, not a control input.

## 4. Position in the project

Presentation only. It cannot send commands, alter a session, or alter vehicle
lifecycle.

## 5. Use and verification

`test/unit_test_ground_station.py` covers normal, stale, disconnected, and
fleet/V2V dashboard rows.
