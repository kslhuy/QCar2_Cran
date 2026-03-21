"""
Contamination Rollback

Provides trust-triggered contamination rollback for the distributed observer.
When a vehicle is newly flagged as malicious, the estimator replays recent
steps while excluding the malicious source's contributions.
"""

import numpy as np
from typing import Dict, List, Optional
from collections import deque


class ContaminationRollback:
    """Trust-triggered contamination rollback for distributed fleet estimation."""

    def __init__(
        self,
        state_dim: int,
        vehicle_id: int,
        fleet_size: int,
        enabled: bool = False,
        window_size: int = 15,
        trust_threshold: float = 0.5,
    ):
        self.state_dim = state_dim
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.enabled = enabled
        self.trust_threshold = trust_threshold
        self.buffer: deque = deque(maxlen=max(window_size, 1))
        self.malicious_vehicles: set = set()
        self.stats = {
            "total_rollbacks": 0,
            "vehicles_flagged": [],
            "rollback_times_ns": [],
        }

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def record(
        self,
        current_time_ns: int,
        pre_update_states: np.ndarray,
        target_components: Dict[int, Dict],
    ) -> None:
        """Store replay data for trust-triggered contamination rollback."""
        if not self.enabled:
            return

        safe_targets: Dict[int, Dict] = {}
        for target_id, comp in target_components.items():
            direct = comp.get("direct", {})
            safe_direct = {
                "source": int(direct.get("source", -1)),
                "delta": np.asarray(
                    direct.get("delta", np.zeros(self.state_dim)), dtype=float
                ).copy(),
            }

            safe_neighbors: Dict[int, np.ndarray] = {}
            for nid, ndelta in comp.get("neighbors", {}).items():
                safe_neighbors[int(nid)] = np.asarray(ndelta, dtype=float).copy()

            safe_targets[int(target_id)] = {
                "direct": safe_direct,
                "neighbors": safe_neighbors,
                "dynamics_delta": np.asarray(
                    comp.get("dynamics_delta", np.zeros(self.state_dim)), dtype=float
                ).copy(),
            }

        self.buffer.append(
            {
                "time_ns": int(current_time_ns),
                "pre_update_states": np.asarray(pre_update_states, dtype=float).copy(),
                "targets": safe_targets,
            }
        )

    def check_and_trigger(
        self,
        trust_scores: Dict[int, float],
        current_time_ns: int,
        fleet_states: np.ndarray,
    ) -> np.ndarray:
        """Detect newly malicious vehicles and trigger rollback if needed.

        Returns the (possibly corrected) fleet_states array.
        """
        if not self.enabled:
            return fleet_states

        threshold = float(np.clip(self.trust_threshold, 0.0, 1.0))
        newly_malicious: List[int] = []

        for vehicle_id, trust_val in trust_scores.items():
            if vehicle_id == self.vehicle_id:
                continue
            if trust_val < threshold:
                if vehicle_id not in self.malicious_vehicles:
                    self.malicious_vehicles.add(vehicle_id)
                    newly_malicious.append(vehicle_id)
            else:
                if vehicle_id in self.malicious_vehicles:
                    self.malicious_vehicles.remove(vehicle_id)

        for malicious_id in newly_malicious:
            fleet_states = self._trigger(malicious_id, current_time_ns, fleet_states)

        return fleet_states

    def reset(self) -> None:
        """Clear all rollback state."""
        self.buffer.clear()
        self.malicious_vehicles.clear()
        self.stats = {
            "total_rollbacks": 0,
            "vehicles_flagged": [],
            "rollback_times_ns": [],
        }

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _trigger(
        self,
        malicious_vehicle_id: int,
        current_time_ns: int,
        fleet_states: np.ndarray,
    ) -> np.ndarray:
        """Replay buffered steps while excluding malicious source contributions."""
        if not self.buffer:
            return fleet_states

        oldest = self.buffer[0]
        corrected_states = np.asarray(oldest["pre_update_states"], dtype=float).copy()
        current_self_state = fleet_states[:, self.vehicle_id].copy()

        for step in list(self.buffer):
            step_targets = step.get("targets", {})
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue
                comp = step_targets.get(target_id)
                if comp is None:
                    continue
                corrected_states[:, target_id] = self._replay_without_malicious(
                    comp=comp,
                    previous_state=corrected_states[:, target_id],
                    malicious_vehicle_id=malicious_vehicle_id,
                )

        corrected_states[:, self.vehicle_id] = current_self_state
        self.buffer.clear()
        self.stats["total_rollbacks"] += 1
        self.stats["vehicles_flagged"].append(int(malicious_vehicle_id))
        self.stats["rollback_times_ns"].append(int(current_time_ns))

        return corrected_states

    @staticmethod
    def _apply_state_constraints(state: np.ndarray) -> np.ndarray:
        """Apply physical constraints to state (duplicated for self-contained replay)."""
        constrained = state.copy()
        constrained[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))
        constrained[3] = np.clip(state[3], -2.0, 2.0)
        if len(state) > 4:
            constrained[4] = np.clip(state[4], -5.0, 5.0)
        return constrained

    def _replay_without_malicious(
        self, comp: Dict, previous_state: np.ndarray, malicious_vehicle_id: int
    ) -> np.ndarray:
        """Rebuild one target update while excluding malicious contributors."""
        delta = np.zeros(self.state_dim)

        direct = comp.get("direct", {})
        direct_src = int(direct.get("source", -1))
        if direct_src != malicious_vehicle_id:
            delta += np.asarray(
                direct.get("delta", np.zeros(self.state_dim)), dtype=float
            )

        for neighbor_id, ndelta in comp.get("neighbors", {}).items():
            if int(neighbor_id) == malicious_vehicle_id:
                continue
            delta += np.asarray(ndelta, dtype=float)

        delta += np.asarray(
            comp.get("dynamics_delta", np.zeros(self.state_dim)), dtype=float
        )
        return self._apply_state_constraints(previous_state + delta)
