"""Right-hand-traffic planner for the 11-node Quanser SDCS small map.

The roadmap is intentionally self-contained: it does not import the licensed
Quanser ``hal`` package at runtime. Its nodes are stored directly in the
CARLA project frame in metres and radians.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, Sequence

import numpy as np

from .path_planner_static import PathPlannerStatic


@dataclass(frozen=True)
class SDCSMapNode:
    """One directed-roadmap node in the CARLA project frame."""

    node_id: int
    x_m: float
    y_m: float
    heading_rad: float


class SDCSSmallMapRoadMap:
    """Self-contained right-hand-traffic graph for SDCS small-map nodes 0-10."""

    _NODE_POSES_WORLD = (
        (-1.140000, 1.053177, -math.pi / 2.0),
        (1.114677, 0.643236, math.pi / 2.0),
        (8.322812, -9.144114, 0.0),
        (8.322812, -6.872355, math.pi),
        (17.785625, 0.643236, math.pi / 2.0),
        (15.513866, 0.643236, -math.pi / 2.0),
        (7.366283, 9.200761, math.pi),
        (9.228099, 6.929003, 0.0),
        (-7.425767, 9.200761, math.pi),
        (-7.425767, 6.929003, 0.0),
        (-11.900960, -3.900281, -42.0 * math.pi / 180.0),
    )
    _INNER_LANE_RADIUS_M = 5.218212
    _OUTER_LANE_RADIUS_M = 7.481429
    _ONE_WAY_STREET_RADIUS_M = 5.978311
    _EDGE_CONFIGS = (
        (0, 2, _OUTER_LANE_RADIUS_M),
        (1, 7, _INNER_LANE_RADIUS_M),
        (1, 8, _OUTER_LANE_RADIUS_M),
        (2, 4, _OUTER_LANE_RADIUS_M),
        (3, 1, _INNER_LANE_RADIUS_M),
        (4, 6, _OUTER_LANE_RADIUS_M),
        (5, 3, _INNER_LANE_RADIUS_M),
        (6, 0, _OUTER_LANE_RADIUS_M),
        (6, 8, 0.0),
        (7, 5, _OUTER_LANE_RADIUS_M),
        (8, 10, _ONE_WAY_STREET_RADIUS_M),
        (9, 0, _INNER_LANE_RADIUS_M),
        (9, 7, 0.0),
        (10, 1, _INNER_LANE_RADIUS_M),
        (10, 2, _INNER_LANE_RADIUS_M),
    )

    def __init__(self, sample_distance_m: float = 0.02) -> None:
        if not isinstance(sample_distance_m, (int, float)) or isinstance(sample_distance_m, bool) or sample_distance_m <= 0.0:
            raise ValueError("sample_distance_m must be a positive number")
        self._sample_distance_m = float(sample_distance_m)
        self._nodes = tuple(
            SDCSMapNode(index, *pose)
            for index, pose in enumerate(self._NODE_POSES_WORLD)
        )
        self._edges = {node.node_id: [] for node in self._nodes}
        self._edge_paths: dict[tuple[int, int], np.ndarray] = {}
        for source, target, radius_m in self._EDGE_CONFIGS:
            path = self._make_edge_path(self._nodes[source], self._nodes[target], radius_m)
            self._edges[source].append((target, path, self._path_length(path)))
            self._edge_paths[(source, target)] = path

    @property
    def node_ids(self) -> tuple[int, ...]:
        return tuple(node.node_id for node in self._nodes)

    def node_pose(self, node_id: int) -> tuple[float, float, float]:
        node = self._node(node_id)
        return node.x_m, node.y_m, node.heading_rad

    def generate_path(self, node_sequence: Sequence[int]) -> np.ndarray:
        """Return a metre-based XY route through requested roadmap nodes."""
        sequence = self._validate_node_sequence(node_sequence)
        route_parts: list[np.ndarray] = []
        for source, target in zip(sequence, sequence[1:]):
            for edge_source, edge_target in self._shortest_edges(source, target):
                edge = self._edge_paths[(edge_source, edge_target)]
                route_parts.append(edge if not route_parts else edge[1:])
        return np.vstack(route_parts)

    def _node(self, node_id: int) -> SDCSMapNode:
        if not isinstance(node_id, int) or isinstance(node_id, bool) or node_id not in self.node_ids:
            raise ValueError(f"SDCS small-map node must be an integer in [0, 10], got {node_id!r}")
        return self._nodes[node_id]

    def _validate_node_sequence(self, node_sequence: Sequence[int]) -> tuple[int, ...]:
        if not isinstance(node_sequence, Sequence) or isinstance(node_sequence, (str, bytes)):
            raise ValueError("node_sequence must be a sequence of small-map node IDs")
        sequence = tuple(node_sequence)
        if len(sequence) < 2:
            raise ValueError("node_sequence requires at least two node IDs")
        for node_id in sequence:
            self._node(node_id)
        if any(source == target for source, target in zip(sequence, sequence[1:])):
            raise ValueError("node_sequence cannot contain adjacent duplicate node IDs")
        return sequence

    def _shortest_edges(self, start: int, goal: int) -> tuple[tuple[int, int], ...]:
        if start == goal:
            return ()
        queue: list[tuple[float, int]] = [(0.0, start)]
        cost = {start: 0.0}
        previous: dict[int, tuple[int, int]] = {}
        while queue:
            current_cost, node_id = heapq.heappop(queue)
            if current_cost != cost.get(node_id):
                continue
            if node_id == goal:
                break
            for target, _path, length in self._edges[node_id]:
                candidate = current_cost + length
                if candidate < cost.get(target, math.inf):
                    cost[target] = candidate
                    previous[target] = (node_id, target)
                    heapq.heappush(queue, (candidate, target))
        if goal not in previous:
            raise ValueError(f"No directed SDCS small-map route exists from node {start} to node {goal}")
        route: list[tuple[int, int]] = []
        node_id = goal
        while node_id != start:
            edge = previous[node_id]
            route.append(edge)
            node_id = edge[0]
        return tuple(reversed(route))

    def _make_edge_path(self, start: SDCSMapNode, end: SDCSMapNode, radius_m: float) -> np.ndarray:
        start_xy = np.array((start.x_m, start.y_m), dtype=float)
        end_xy = np.array((end.x_m, end.y_m), dtype=float)
        if radius_m <= 0.0:
            return self._sample_line(start_xy, end_xy)
        path = self._sample_straight_curve_straight(start_xy, start.heading_rad, end_xy, end.heading_rad, radius_m)
        return path if path is not None else self._sample_hermite(start_xy, start.heading_rad, end_xy, end.heading_rad, radius_m)

    def _sample_straight_curve_straight(
        self,
        start: np.ndarray,
        start_heading: float,
        end: np.ndarray,
        end_heading: float,
        radius_m: float,
    ) -> np.ndarray | None:
        tangent_start = np.array((math.cos(start_heading), math.sin(start_heading)), dtype=float)
        tangent_end = np.array((math.cos(end_heading), math.sin(end_heading)), dtype=float)
        direction = 1.0 if _cross(tangent_start, end - start) >= 0.0 else -1.0
        normal_start = direction * radius_m * np.array((-tangent_start[1], tangent_start[0]))
        normal_end = direction * radius_m * np.array((-tangent_end[1], tangent_end[0]))
        heading_delta = _wrap_pi(end_heading - start_heading)
        tolerance = 1e-3

        if abs(heading_delta) < tolerance:
            line_direction = end - start
            line_length = float(np.linalg.norm(line_direction))
            if line_length <= tolerance or abs(_cross(tangent_start, line_direction / line_length)) > tolerance:
                return None
            return self._sample_line(start, end)

        if abs(abs(heading_delta) - math.pi) < tolerance:
            return None

        matrix = np.column_stack((tangent_start, -tangent_end))
        try:
            alpha, beta = np.linalg.solve(matrix, (end + normal_end) - (start + normal_start))
        except np.linalg.LinAlgError:
            return None
        if alpha < -tolerance or beta > tolerance:
            return None

        center = start + normal_start + alpha * tangent_start
        arc_start = center - normal_start
        arc_end = center - normal_end
        line_one = self._sample_line(start, arc_start)
        arc = self._sample_arc(center, arc_start, arc_end, radius_m, direction)
        line_two = self._sample_line(arc_end, end)
        return _join_paths(line_one, arc, line_two)

    def _sample_hermite(
        self,
        start: np.ndarray,
        start_heading: float,
        end: np.ndarray,
        end_heading: float,
        radius_m: float,
    ) -> np.ndarray:
        distance = float(np.linalg.norm(end - start))
        tangent_length = min(0.5 * distance, max(radius_m, 0.05))
        tangent_start = tangent_length * np.array((math.cos(start_heading), math.sin(start_heading)))
        tangent_end = tangent_length * np.array((math.cos(end_heading), math.sin(end_heading)))
        count = max(2, int(math.ceil(max(distance, self._sample_distance_m) / self._sample_distance_m)) + 1)
        s = np.linspace(0.0, 1.0, count)
        h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
        h10 = s**3 - 2.0 * s**2 + s
        h01 = -2.0 * s**3 + 3.0 * s**2
        h11 = s**3 - s**2
        return (
            h00[:, None] * start
            + h10[:, None] * tangent_start
            + h01[:, None] * end
            + h11[:, None] * tangent_end
        )

    def _sample_line(self, start: np.ndarray, end: np.ndarray) -> np.ndarray:
        length = float(np.linalg.norm(end - start))
        count = max(2, int(math.ceil(max(length, self._sample_distance_m) / self._sample_distance_m)) + 1)
        return np.linspace(start, end, count)

    def _sample_arc(
        self,
        center: np.ndarray,
        start: np.ndarray,
        end: np.ndarray,
        radius_m: float,
        direction: float,
    ) -> np.ndarray:
        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        end_angle = math.atan2(end[1] - center[1], end[0] - center[0])
        delta = _wrap_two_pi(direction * (end_angle - start_angle))
        count = max(2, int(math.ceil(max(radius_m * delta, self._sample_distance_m) / self._sample_distance_m)) + 1)
        angles = start_angle + direction * np.linspace(0.0, delta, count)
        return center + radius_m * np.column_stack((np.cos(angles), np.sin(angles)))

    @staticmethod
    def _path_length(path: np.ndarray) -> float:
        return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


class PathPlannerSDCSSmallMap(PathPlannerStatic):
    """Static waypoint planner that generates a route from SDCS node IDs."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None, **overrides) -> None:
        effective_config = dict(config)
        effective_config.update(overrides)
        self._roadmap = SDCSSmallMapRoadMap(float(effective_config.get("sample_distance_m", 0.02)))
        configured_nodes = effective_config.get("node_sequence")
        if configured_nodes is not None:
            effective_config.pop("path_source", None)
        self._node_sequence: tuple[int, ...] = ()
        self._route_node_sequence: tuple[int, ...] = ()
        self._loop: int | str = 0
        super().__init__(effective_config, vehicle_id, logger)
        if configured_nodes is not None:
            self.set_node_sequence(configured_nodes, loop=effective_config.get("loop", 0))

    @property
    def node_sequence(self) -> tuple[int, ...]:
        return self._node_sequence

    @property
    def available_node_ids(self) -> tuple[int, ...]:
        return self._roadmap.node_ids

    @property
    def loop(self) -> int | str:
        """Return the requested route completion policy."""
        return self._loop

    @property
    def route_node_sequence(self) -> tuple[int, ...]:
        """Return the expanded finite route, or one cycle for ``inf``."""
        return self._route_node_sequence

    def node_pose(self, node_id: int) -> tuple[float, float, float]:
        return self._roadmap.node_pose(node_id)

    def set_node_sequence(self, node_sequence: Sequence[int], *, loop: int | str = 0) -> None:
        """Generate a directed node route with an explicit completion policy.

        ``0`` ends at the last requested node.  ``1`` returns to the first
        node once, ``2`` completes two closed circuits, and ``"inf"`` repeats
        a closed circuit until an operator disables the map route.
        """
        sequence = tuple(node_sequence)
        resolved_loop = _validate_loop(loop)
        route_sequence = _expand_loop_sequence(sequence, resolved_loop)
        waypoints = self._roadmap.generate_path(route_sequence)
        self._node_sequence = sequence
        self._route_node_sequence = route_sequence
        self._loop = resolved_loop
        self._config["node_sequence"] = list(sequence)
        self._config["loop"] = resolved_loop
        PathPlannerStatic.load_path(self, waypoints)

    def load_path(self, path_source: Iterable[Sequence[float]] | str) -> None:
        """Load a generic waypoint path and clear the active SDCS node route."""
        self._node_sequence = ()
        self._route_node_sequence = ()
        self._loop = 0
        PathPlannerStatic.load_path(self, path_source)

    def update(self, state):
        """Restart the closed route at its origin for an infinite loop."""
        target = PathPlannerStatic.update(self, state)
        if self._loop != "inf" or not target.is_finished:
            return target
        PathPlannerStatic.reset(self)
        return PathPlannerStatic.update(self, state)


def _cross(first: np.ndarray, second: np.ndarray) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def _wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_two_pi(angle: float) -> float:
    return float(angle) % (2.0 * math.pi)


def _join_paths(*paths: np.ndarray) -> np.ndarray:
    result = paths[0]
    for path in paths[1:]:
        result = np.vstack((result, path[1:]))
    return result


def _validate_loop(loop: int | str) -> int | str:
    if loop in (0, 1, 2) and not isinstance(loop, bool):
        return int(loop)
    if loop == "inf":
        return loop
    raise ValueError("loop must be 0, 1, 2, or 'inf'")


def _expand_loop_sequence(sequence: Sequence[int], loop: int | str) -> tuple[int, ...]:
    """Expand node IDs without adding adjacent duplicate nodes.

    A finite loop count is the number of closed circuits.  The supplied path
    itself is preserved for ``0``.  A path that already ends at its origin is
    treated as one closed circuit and is not closed twice.
    """
    base = tuple(sequence)
    if loop == 0:
        return base
    closed = base[-1] == base[0]
    circuit = base if closed else base + (base[0],)
    if loop == "inf" or loop == 1:
        return circuit
    return circuit + tuple(circuit[1:])
