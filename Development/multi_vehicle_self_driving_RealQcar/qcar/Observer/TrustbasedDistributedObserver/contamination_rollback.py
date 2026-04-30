"""
Contamination Rollback

Provides trust-triggered contamination rollback for the distributed observer.
When a vehicle is newly flagged as malicious, the estimator replays recent
steps while excluding the malicious source's contributions.
"""

import numpy as np
from typing import Callable, Dict, List, Optional, Set
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
        predict_fn: Optional[
            Callable[[np.ndarray, Optional[np.ndarray], float, int], np.ndarray]
        ] = None,
        constraints_fn: Optional[Callable[[np.ndarray, int], np.ndarray]] = None,
        trusted_state_fn: Optional[
            Callable[[int], Optional[tuple]]
        ] = None,
        logger=None,
    ):
        self.state_dim = state_dim
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.enabled = enabled
        self.trust_threshold = trust_threshold
        self.predict_fn = predict_fn
        self.constraints_fn = constraints_fn
        self.trusted_state_fn = trusted_state_fn
        self.logger = logger
        self.buffer: deque = deque(maxlen=max(window_size, 1))
        self.malicious_vehicles: set = set()
        self.stats = {
            "total_rollbacks": 0,
            "vehicles_flagged": [],
            "rollback_times_ns": [],
        }
        self.last_event = self._build_event(
            current_time_ns=None,
            triggered=False,
            newly_flagged=[],
        )

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
            safe_direct = {"source": int(direct.get("source", -1))}
            direct_state = direct.get("state")
            if direct_state is not None:
                safe_direct["weight"] = float(direct.get("weight", 0.0))
                safe_direct["state"] = np.asarray(direct_state, dtype=float).copy()
            else:
                safe_direct["delta"] = np.asarray(
                    direct.get("delta", np.zeros(self.state_dim)), dtype=float
                ).copy()

            safe_neighbors: Dict[int, Dict[str, np.ndarray]] = {}
            for nid, neighbor_data in comp.get("neighbors", {}).items():
                neighbor_id = int(nid)
                if isinstance(neighbor_data, dict) and neighbor_data.get("state") is not None:
                    safe_neighbors[neighbor_id] = {
                        "weight": float(neighbor_data.get("weight", 0.0)),
                        "state": np.asarray(
                            neighbor_data.get("state", np.zeros(self.state_dim)),
                            dtype=float,
                        ).copy(),
                    }
                else:
                    safe_neighbors[neighbor_id] = {
                        "delta": np.asarray(neighbor_data, dtype=float).copy()
                    }

            prediction = comp.get("prediction", {})
            control = prediction.get("control")
            safe_prediction = {
                "dt": float(prediction.get("dt", 0.0)),
                "control": None,
                "force_clean_pose_anchor": bool(
                    prediction.get("force_clean_pose_anchor", False)
                ),
            }
            if control is not None:
                safe_prediction["control"] = np.asarray(control, dtype=float).copy()

            safe_targets[int(target_id)] = {
                "direct": safe_direct,
                "neighbors": safe_neighbors,
                "prediction": safe_prediction,
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
        trigger_signals: Optional[Dict[int, Dict[str, object]]] = None,
    ) -> np.ndarray:
        """Detect newly malicious vehicles and trigger rollback if needed.

        Returns the (possibly corrected) fleet_states array.
        """
        if not self.enabled:
            return fleet_states

        threshold = float(np.clip(self.trust_threshold, 0.0, 1.0))
        newly_malicious: List[int] = []
        active_malicious: Set[int] = set()
        trigger_signals = trigger_signals or {}
        active_trigger_reasons: Dict[int, List[str]] = {}
        newly_flagged_reasons: Dict[int, List[str]] = {}

        for vehicle_id, trust_val in trust_scores.items():
            if vehicle_id == self.vehicle_id:
                continue
            signal = trigger_signals.get(int(vehicle_id), {})
            reasons: List[str] = []
            if bool(signal.get("flag_local_est_check", False)):
                reasons.append("local_est_check")
            if bool(signal.get("flag_global_est_check", False)):
                reasons.append("global_est_check")
            if bool(signal.get("trust_below_threshold", trust_val < threshold)):
                reasons.append("final_trust")

            if reasons:
                active_malicious.add(int(vehicle_id))
                active_trigger_reasons[int(vehicle_id)] = reasons
                if vehicle_id not in self.malicious_vehicles:
                    newly_malicious.append(vehicle_id)
                    newly_flagged_reasons[int(vehicle_id)] = reasons
        self.malicious_vehicles = set(active_malicious)

        if newly_malicious:
            fleet_states = self._trigger(
                active_malicious,
                current_time_ns,
                fleet_states,
                newly_flagged=newly_malicious,
                active_trigger_reasons=active_trigger_reasons,
                newly_flagged_reasons=newly_flagged_reasons,
            )
        else:
            self.last_event = self._build_event(
                current_time_ns=current_time_ns,
                triggered=False,
                newly_flagged=[],
                active_trigger_reasons=active_trigger_reasons,
                newly_flagged_reasons={},
            )

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
        self.last_event = self._build_event(
            current_time_ns=None,
            triggered=False,
            newly_flagged=[],
        )

    def get_status(self) -> Dict[str, object]:
        """Return the latest rollback status snapshot."""
        return {
            "enabled": bool(self.enabled),
            "triggered": bool(self.last_event.get("triggered", False)),
            "event_time_ns": self.last_event.get("event_time_ns"),
            "active_malicious": list(self.last_event.get("active_malicious", [])),
            "newly_flagged": list(self.last_event.get("newly_flagged", [])),
            "active_trigger_reasons": dict(
                self.last_event.get("active_trigger_reasons", {})
            ),
            "newly_flagged_reasons": dict(
                self.last_event.get("newly_flagged_reasons", {})
            ),
            "total_rollbacks": int(self.stats.get("total_rollbacks", 0)),
        }

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _trigger(
        self,
        malicious_ids: Set[int],
        current_time_ns: int,
        fleet_states: np.ndarray,
        newly_flagged: Optional[List[int]] = None,
        active_trigger_reasons: Optional[Dict[int, List[str]]] = None,
        newly_flagged_reasons: Optional[Dict[int, List[str]]] = None,
    ) -> np.ndarray:
        """Replay buffered steps while excluding all malicious source contributions."""
        if not self.buffer:
            self.last_event = self._build_event(
                current_time_ns=current_time_ns,
                triggered=False,
                newly_flagged=newly_flagged or [],
                active_trigger_reasons=active_trigger_reasons or {},
                newly_flagged_reasons=newly_flagged_reasons or {},
            )
            return fleet_states

        oldest = self.buffer[0]
        corrected_states = np.asarray(oldest["pre_update_states"], dtype=float).copy()
        current_self_state = fleet_states[:, self.vehicle_id].copy()
        target_replay_start_ns: Dict[int, Optional[int]] = {}

        for target_id in range(self.fleet_size):
            if target_id == self.vehicle_id or int(target_id) not in malicious_ids:
                continue
            trusted_entry = None
            if callable(self.trusted_state_fn):
                trusted_entry = self.trusted_state_fn(int(target_id))
            if trusted_entry is None:
                continue
            trusted_state, trusted_time_ns = trusted_entry
            trusted_state = np.asarray(trusted_state, dtype=float).copy()
            if trusted_state.size < self.state_dim:
                trusted_state = np.pad(
                    trusted_state, (0, self.state_dim - trusted_state.size), mode="constant"
                )
            corrected_states[:, target_id] = self._apply_state_constraints(
                trusted_state[: self.state_dim], target_id=target_id
            )
            target_replay_start_ns[int(target_id)] = (
                None if trusted_time_ns is None else int(trusted_time_ns)
            )

        for step in list(self.buffer):
            step_targets = step.get("targets", {})
            step_time_ns = int(step.get("time_ns", 0))
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue
                comp = step_targets.get(target_id)
                if comp is None:
                    continue
                replay_start_ns = target_replay_start_ns.get(int(target_id))
                if replay_start_ns is not None and step_time_ns <= replay_start_ns:
                    continue
                corrected_states[:, target_id] = self._replay_without_malicious(
                    comp=comp,
                    previous_state=corrected_states[:, target_id],
                    malicious_ids=malicious_ids,
                    target_id=target_id,
                    step_time_ns=step_time_ns,
                )

        corrected_states[:, self.vehicle_id] = current_self_state
        self.buffer.clear()
        self.stats["total_rollbacks"] += 1
        self.stats["vehicles_flagged"].extend(int(mid) for mid in malicious_ids)
        self.stats["rollback_times_ns"].append(int(current_time_ns))
        self.last_event = self._build_event(
            current_time_ns=current_time_ns,
            triggered=True,
            newly_flagged=newly_flagged or [],
            active_trigger_reasons=active_trigger_reasons or {},
            newly_flagged_reasons=newly_flagged_reasons or {},
        )
        self._log_trigger(
            current_time_ns,
            malicious_ids,
            newly_flagged or [],
            active_trigger_reasons or {},
        )

        return corrected_states

    def _build_event(
        self,
        current_time_ns: Optional[int],
        triggered: bool,
        newly_flagged: List[int],
        active_trigger_reasons: Optional[Dict[int, List[str]]] = None,
        newly_flagged_reasons: Optional[Dict[int, List[str]]] = None,
    ) -> Dict[str, object]:
        return {
            "enabled": bool(self.enabled),
            "triggered": bool(triggered),
            "event_time_ns": None if current_time_ns is None else int(current_time_ns),
            "active_malicious": sorted(int(vid) for vid in self.malicious_vehicles),
            "newly_flagged": sorted(int(vid) for vid in newly_flagged),
            "active_trigger_reasons": {
                int(vid): list(reasons)
                for vid, reasons in (active_trigger_reasons or {}).items()
            },
            "newly_flagged_reasons": {
                int(vid): list(reasons)
                for vid, reasons in (newly_flagged_reasons or {}).items()
            },
            "total_rollbacks": int(self.stats.get("total_rollbacks", 0)),
        }

    def _log_trigger(
        self,
        current_time_ns: int,
        malicious_ids: Set[int],
        newly_flagged: List[int],
        active_trigger_reasons: Dict[int, List[str]],
    ) -> None:
        if self.logger is None or not hasattr(self.logger, "logger"):
            return
        self.logger.logger.info(
            "Contamination rollback triggered at %d ns; active malicious=%s; newly flagged=%s; reasons=%s",
            int(current_time_ns),
            sorted(int(vid) for vid in malicious_ids),
            sorted(int(vid) for vid in newly_flagged),
            {
                int(vid): list(reasons)
                for vid, reasons in active_trigger_reasons.items()
            },
        )

    def _apply_state_constraints(
        self, state: np.ndarray, target_id: int = -1
    ) -> np.ndarray:
        """Apply physical constraints to replayed state."""
        if callable(self.constraints_fn):
            constrained = self.constraints_fn(
                np.asarray(state, dtype=float).copy(), int(target_id)
            )
            return np.asarray(constrained, dtype=float).copy()

        constrained = state.copy()
        constrained[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))
        constrained[3] = np.clip(state[3], -2.0, 2.0)
        if len(state) > 4:
            constrained[4] = np.clip(state[4], -5.0, 5.0)
        return constrained

    def _replay_without_malicious(
        self,
        comp: Dict,
        previous_state: np.ndarray,
        malicious_ids: Set[int],
        target_id: int,
        step_time_ns: Optional[int] = None,
    ) -> np.ndarray:
        """Rebuild one target update while excluding all malicious contributors."""
        previous_state = np.asarray(previous_state, dtype=float).copy()
        total_correction = np.zeros(self.state_dim)

        direct = comp.get("direct", {})
        direct_src = int(direct.get("source", -1))
        if direct_src not in malicious_ids:
            if direct.get("state") is not None:
                direct_weight = float(direct.get("weight", 0.0))
                direct_state = np.asarray(direct.get("state"), dtype=float)
                total_correction += direct_weight * (direct_state - previous_state)
            else:
                total_correction += np.asarray(
                    direct.get("delta", np.zeros(self.state_dim)), dtype=float
                )

        for neighbor_id, neighbor_data in comp.get("neighbors", {}).items():
            if int(neighbor_id) in malicious_ids:
                continue
            if isinstance(neighbor_data, dict) and neighbor_data.get("state") is not None:
                neighbor_weight = float(neighbor_data.get("weight", 0.0))
                neighbor_state = np.asarray(neighbor_data.get("state"), dtype=float)
                total_correction += neighbor_weight * (neighbor_state - previous_state)
            else:
                total_correction += np.asarray(
                    neighbor_data.get("delta", np.zeros(self.state_dim)), dtype=float
                )

        consensus_state = self._apply_state_constraints(
            previous_state + total_correction, target_id=target_id
        )
        prediction = comp.get("prediction", {})
        dt = float(prediction.get("dt", 0.0))
        control = prediction.get("control")
        force_clean_pose_anchor = bool(
            prediction.get("force_clean_pose_anchor", False)
        )
        if control is not None:
            control = np.asarray(control, dtype=float).copy()

        if callable(self.predict_fn) and dt > 0.0:
            predicted_state = self.predict_fn(
                consensus_state,
                control,
                dt,
                int(target_id),
                current_time_ns=step_time_ns,
                force_clean_pose_anchor=force_clean_pose_anchor,
            )
            return self._apply_state_constraints(predicted_state, target_id=target_id)

        return consensus_state
