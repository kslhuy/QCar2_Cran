# `utils/control/path_planner/path_planner_sdcs_small_map.py`

## 1. Introduction

The SDCS small-map planner converts selected intersection node routes into a sampled static route.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `SDCSMapNode(node_id, x_m, y_m, heading_rad)` | Node ID and CARLA-project-frame pose | Frozen node record | Represents one directed small-map roadmap node. |
| `SDCSSmallMapRoadMap(sample_distance_m=0.02)` | Positive sampling distance | Directed roadmap or `ValueError` | Builds 11 nodes, directed edges, and sampled edge paths. |
| `node_ids` / `node_pose(node_id)` | Roadmap / node ID | ID tuple / `(x, y, heading)` | Expose known nodes and validate pose lookup. |
| `generate_path(node_sequence)` | At least two non-adjacent-valid node IDs | XY `ndarray` route | Concatenates Dijkstra shortest directed edge paths. |
| `_node(...)` / `_validate_node_sequence(...)` / `_shortest_edges(...)` | Node ID/sequence/start-goal | Validated node/sequence/edge tuple | Validate graph inputs and run shortest-path search by edge length. |
| `_make_edge_path(...)` / `_sample_straight_curve_straight(...)` / `_sample_hermite(...)` | Endpoint nodes/poses/radius | Sampled XY path | Prefer tangent line-arc-line geometry; fall back to cubic Hermite interpolation. |
| `_sample_line(...)` / `_sample_arc(...)` / `_path_length(path)` | Geometry endpoints/center/radius/path | Sampled path / length | Sample primitives at configured resolution and measure polyline length. |
| `PathPlannerSDCSSmallMap(config, vehicle_id=0, logger=None, **overrides)` | Static planner config and optional node/loop overrides | SDCS planner | Creates roadmap, clears generic route when nodes configured, then builds SDCS route. |
| `node_sequence` / `available_node_ids` / `loop` / `route_node_sequence` | Planner/roadmap state | Requested nodes, available IDs, policy, expanded route | Expose configured and expanded route state; `inf` reports one cycle. |
| `node_pose(node_id)` | SDCS node ID | Project-frame pose | Delegates validated roadmap lookup. |
| `set_node_sequence(node_sequence, *, loop=0)` | SDCS nodes and `0`, `1`, `2`, or `inf` loop policy | Generated waypoint/progress state | Expands finite cycles, generates directed route, updates config, then loads static waypoints. |
| `load_path(path_source)` / `update(state)` | Generic path / vehicle state | Cleared SDCS route / `ControllerReference` | Generic path clears node policy; `inf` resets/restarts after one closed route finishes. |
| `_cross(...)` / `_wrap_pi(...)` / `_wrap_two_pi(...)` / `_join_paths(...)` | Geometry vectors/angles/paths | Scalar angle/cross or joined path | Support directed geometry construction. |
| `_validate_loop(loop)` / `_expand_loop_sequence(sequence, loop)` | Loop policy and requested nodes | Validated policy / finite node sequence | Implement explicit `0`, `1`, `2`, and `inf` route-completion semantics. |

## 3. Special data and cross-references

This module participates in the ordered [[vehicle-runtime|VehicleRuntime]] control loop and uses the shared [[vehicle-types|vehicle contracts]].

Node IDs identify physical SDCS map locations; `[x, y, yaw]` waypoints are generated with straight/arc/Hermite geometry. `loop=0/1/2/inf` expands route completion as documented.

## 4. Position in the project

Selected through `PathPlannerManager`; command semantics and fleet eligibility remain in core.

## 5. Use and verification

`test/unit_test_path_planner.py` and SDCS command-handler tests verify nodes, loops, and route update.
