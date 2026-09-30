"""
V2V attack metadata normalization for trust logs.

This tracker mirrors GUI/script attack ground truth into a compact structure for
TrustWeightLogger. It does not influence trust scoring or estimator dynamics.
"""

import json
from typing import Any, Dict, List, Optional

import numpy as np


class V2VAttackStatusTracker:
    """Normalize V2V attack status payloads and build trust-log metadata."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Clear all tracked V2V attack metadata."""
        self._status: Dict[str, Any] = {}
        self._scenarios: Dict[str, Dict[str, Any]] = {}
        self._enable_time_s: Optional[float] = None
        self._disable_time_s: Optional[float] = None
        self._last_event = ""
        self._last_event_time_s: Optional[float] = None
        self._events: List[Dict[str, float]] = []
        self._value_snapshot: Dict[int, Dict[str, Any]] = {}

    def set_status(self, status: Optional[Dict[str, Any]]) -> None:
        """
        Store V2V attack metadata supplied by VehicleLogic.

        This is command ground truth for CSV logging. Trust-model attack flags
        remain separate.
        """
        if not isinstance(status, dict):
            return

        clock_s = self._float_or_none(status.get("elapsed_time"))
        if clock_s is None:
            clock_s = self._float_or_none(status.get("current_time"))

        manual_disable_time = self._float_or_none(status.get("manual_disable_time"))
        enabled = bool(status.get("enabled", True))
        attack_active = bool(status.get("attack_active", False)) and enabled
        previous_enabled = self._status.get("enabled")
        event_time_s = manual_disable_time if manual_disable_time is not None else clock_s

        if enabled:
            if previous_enabled is not True:
                self._append_event("enable", event_time_s)
        elif previous_enabled is True or manual_disable_time is not None:
            self._append_event("disable", event_time_s)

        value_snapshot = self._normalize_value_snapshot(
            status.get("attack_value_snapshot")
        )
        if value_snapshot:
            self._value_snapshot = value_snapshot
        elif not attack_active or not enabled:
            self._value_snapshot = {}

        raw_scenarios: List[Any] = []
        for key in (
            "all_scenario_details",
            "scenario_details",
            "configured_scenarios",
            "active_scenario_details",
        ):
            raw_value = status.get(key)
            if isinstance(raw_value, list):
                raw_scenarios.extend(raw_value)

        for raw_scenario in raw_scenarios:
            scenario = self._normalize_scenario(
                raw_scenario=raw_scenario,
                clock_s=clock_s,
                enabled=enabled,
                manual_disable_time=manual_disable_time,
            )
            if scenario is None:
                continue
            scenario_key = self._scenario_key(scenario)
            existing = self._scenarios.get(scenario_key)
            if existing is not None and not enabled:
                existing_end = float(existing.get("t_end", float("inf")))
                scenario_end = float(scenario.get("t_end", float("inf")))
                if np.isfinite(existing_end) and (
                    not np.isfinite(scenario_end) or scenario_end > existing_end
                ):
                    scenario["t_end"] = existing_end
            self._scenarios[scenario_key] = scenario

        if not raw_scenarios and not attack_active:
            close_time = manual_disable_time if manual_disable_time is not None else clock_s
            if close_time is not None:
                for scenario in self._scenarios.values():
                    if scenario.get("active", False) or not np.isfinite(
                        float(scenario.get("t_end", float("inf")))
                    ):
                        scenario["t_end"] = max(float(close_time), scenario["t_start"])
                    scenario["active"] = False

        self._status = {
            "enabled": enabled,
            "attack_active": attack_active,
            "elapsed_time": clock_s,
        }

    def get_log_data(
        self,
        current_time_ns: int,
        fleet_size: int,
        fallback_clock_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Build flattened V2V attack metadata for TrustWeightLogger."""
        clock_s = self._float_or_none(self._status.get("elapsed_time"))
        if clock_s is None:
            if fallback_clock_s is None:
                fallback_clock_s = max(float(current_time_ns), 0.0) / 1e9
            clock_s = float(fallback_clock_s)

        enabled = bool(self._status.get("enabled", False))
        attack_value_snapshot = (
            self._value_snapshot
            if enabled and bool(self._status.get("attack_active", False))
            else {}
        )
        scenarios = list(self._scenarios.values())
        active_scenarios = [
            s for s in scenarios if self._scenario_is_active_at_clock(s, clock_s, enabled)
        ]
        display_scenarios = active_scenarios if active_scenarios else scenarios

        by_vehicle: Dict[int, Dict[str, Any]] = {}
        for vehicle_id in range(int(fleet_size)):
            vehicle_scenarios = [
                s for s in scenarios if self._scenario_targets_vehicle(s, vehicle_id)
            ]
            vehicle_snapshot = attack_value_snapshot.get(vehicle_id, {})
            if not vehicle_scenarios and not vehicle_snapshot:
                continue
            active_for_vehicle = [
                s
                for s in vehicle_scenarios
                if self._scenario_is_active_at_clock(s, clock_s, enabled)
            ]
            scenario_fields = {
                str(field)
                for scenario in vehicle_scenarios
                for field in scenario.get("target_fields", [])
            }
            snapshot_values = vehicle_snapshot.get("values", {})
            snapshot_fields = set(vehicle_snapshot.get("fields", [])) | set(
                snapshot_values.keys()
            )
            start_s = (
                min(float(s["t_start"]) for s in vehicle_scenarios)
                if vehicle_scenarios
                else float("nan")
            )
            end_s = (
                max(float(s["t_end"]) for s in vehicle_scenarios)
                if vehicle_scenarios
                else float("nan")
            )

            combined_types = self._unique_values(vehicle_scenarios, "type")
            combined_names = self._unique_values(vehicle_scenarios, "name")
            combined_data_types = self._unique_values(vehicle_scenarios, "data_type")
            combined_modifications = self._unique_values(
                vehicle_scenarios, "modification"
            )
            for key, dest in (
                ("types", combined_types),
                ("names", combined_names),
                ("data_types", combined_data_types),
                ("modifications", combined_modifications),
            ):
                for value in vehicle_snapshot.get(key, []):
                    value = str(value)
                    if value and value not in dest:
                        dest.append(value)

            by_vehicle[vehicle_id] = {
                "active": bool(active_for_vehicle)
                or bool(vehicle_snapshot.get("active", False)),
                "types": combined_types,
                "names": combined_names,
                "data_types": combined_data_types,
                "modifications": combined_modifications,
                "fields": sorted(scenario_fields | snapshot_fields),
                "start_s": start_s,
                "end_s": end_s,
                "attacker_id": (
                    vehicle_scenarios[0].get("attacker_id")
                    if vehicle_scenarios
                    else vehicle_snapshot.get("attacker_id")
                ),
                "values": snapshot_values,
            }

        return {
            "enabled": enabled,
            "active": bool(active_scenarios),
            "clock_s": float(clock_s),
            "scenario_count": len(scenarios),
            "active_count": len(active_scenarios),
            "types": self._unique_values(display_scenarios, "type"),
            "names": self._unique_values(display_scenarios, "name"),
            "data_types": self._unique_values(display_scenarios, "data_type"),
            "enable_time_s": self._enable_time_s,
            "disable_time_s": self._disable_time_s,
            "last_event": self._last_event,
            "last_event_time_s": self._last_event_time_s,
            "events": self._events_json(),
            "intervals": self._intervals_json(scenarios),
            "start_s": (
                min(float(s["t_start"]) for s in display_scenarios)
                if display_scenarios
                else float("nan")
            ),
            "end_s": (
                max(float(s["t_end"]) for s in display_scenarios)
                if display_scenarios
                else float("nan")
            ),
            "by_vehicle": by_vehicle,
        }

    @staticmethod
    def _enum_value(value: Any) -> Any:
        return getattr(value, "value", value)

    @staticmethod
    def _float_or_none(value: Any) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _list_or_empty(cls, value: Any) -> List[Any]:
        if value is None:
            return []
        if isinstance(value, list):
            return value
        if isinstance(value, tuple):
            return list(value)
        return [value]

    def _normalize_scenario(
        self,
        raw_scenario: Any,
        clock_s: Optional[float],
        enabled: bool,
        manual_disable_time: Optional[float],
    ) -> Optional[Dict[str, Any]]:
        """Normalize injected-attack scenario metadata for CSV logging."""
        if isinstance(raw_scenario, dict):
            getter = raw_scenario.get
        else:
            getter = lambda key, default=None: getattr(raw_scenario, key, default)

        t_start = self._float_or_none(getter("t_start", getter("start_s")))
        t_end = self._float_or_none(getter("t_end", getter("end_s")))
        if t_start is None:
            return None
        if t_end is None:
            t_end = float("inf")

        if manual_disable_time is not None and (
            not np.isfinite(t_end) or t_end > manual_disable_time
        ):
            t_end = max(float(manual_disable_time), t_start)

        fields = [str(v) for v in self._list_or_empty(getter("target_fields", []))]
        victims = []
        for victim in self._list_or_empty(getter("victim_ids", [])):
            try:
                victims.append(int(victim))
            except (TypeError, ValueError):
                continue

        attacker_id = getter("attacker_id")
        try:
            attacker_id = int(attacker_id)
        except (TypeError, ValueError):
            attacker_id = None

        name = str(getter("name", getter("scenario_name", "")) or "")
        attack_type = str(
            self._enum_value(getter("type", getter("attack_type", ""))) or ""
        )
        modification = str(
            self._enum_value(getter("modification", getter("modification_type", "")))
            or ""
        )
        data_type = str(self._enum_value(getter("data_type", "")) or "").lower()

        interval_active = False
        if clock_s is not None:
            interval_active = bool(t_start <= clock_s <= t_end)
        active = bool(getter("active", False) or interval_active) and bool(enabled)

        return {
            "name": name,
            "type": attack_type,
            "modification": modification,
            "data_type": data_type,
            "target_fields": fields,
            "t_start": float(t_start),
            "t_end": float(t_end),
            "attacker_id": attacker_id,
            "victim_ids": victims,
            "active": active,
        }

    def _normalize_value_snapshot(self, raw_snapshot: Any) -> Dict[int, Dict[str, Any]]:
        """Normalize live per-vehicle attack values for CSV logging."""
        if not isinstance(raw_snapshot, dict):
            return {}

        by_vehicle_raw = raw_snapshot.get("by_vehicle", raw_snapshot)
        if not isinstance(by_vehicle_raw, dict):
            return {}

        normalized: Dict[int, Dict[str, Any]] = {}
        for raw_vehicle_id, raw_entry in by_vehicle_raw.items():
            try:
                vehicle_id = int(raw_vehicle_id)
            except (TypeError, ValueError):
                continue
            if not isinstance(raw_entry, dict):
                continue

            raw_values = raw_entry.get("values", {})
            if not isinstance(raw_values, dict):
                raw_values = {}

            values: Dict[str, Dict[str, Optional[float]]] = {}
            for raw_field, raw_value_triplet in raw_values.items():
                if not isinstance(raw_value_triplet, dict):
                    continue
                field = str(raw_field)
                original = self._finite_or_none(raw_value_triplet.get("original"))
                modified = self._finite_or_none(raw_value_triplet.get("modified"))
                delta = self._finite_or_none(raw_value_triplet.get("delta"))
                if original is None and modified is None and delta is None:
                    continue
                values[field] = {
                    "original": original,
                    "modified": modified,
                    "delta": delta,
                }

            attacker_id = raw_entry.get("attacker_id")
            try:
                attacker_id = int(attacker_id)
            except (TypeError, ValueError):
                attacker_id = None

            fields = [str(v) for v in self._list_or_empty(raw_entry.get("fields", []))]
            if values:
                fields = sorted(set(fields) | set(values.keys()))

            normalized[vehicle_id] = {
                "active": bool(raw_entry.get("active", False)),
                "attacker_id": attacker_id,
                "types": [str(v) for v in self._list_or_empty(raw_entry.get("types", []))],
                "names": [str(v) for v in self._list_or_empty(raw_entry.get("names", []))],
                "modifications": [
                    str(v)
                    for v in self._list_or_empty(raw_entry.get("modifications", []))
                ],
                "data_types": [
                    str(v) for v in self._list_or_empty(raw_entry.get("data_types", []))
                ],
                "fields": fields,
                "values": values,
            }
        return normalized

    @staticmethod
    def _scenario_key(scenario: Dict[str, Any]) -> str:
        fields = ",".join(str(v) for v in scenario.get("target_fields", []))
        return (
            f"{scenario.get('name', '')}|{scenario.get('attacker_id')}|"
            f"{scenario.get('t_start')}|{scenario.get('data_type')}|"
            f"{scenario.get('type')}|{fields}"
        )

    @staticmethod
    def _finite_or_none(value: Any) -> Optional[float]:
        try:
            fvalue = float(value)
        except (TypeError, ValueError):
            return None
        if not np.isfinite(fvalue):
            return None
        return fvalue

    def _append_event(self, event: str, event_time_s: Optional[float]) -> None:
        """Remember exact attack enable/disable commands for CSV plotting."""
        if event_time_s is None:
            return

        event_time_s = float(event_time_s)
        if self._events:
            last = self._events[-1]
            if (
                last.get("event") == event
                and abs(float(last.get("time_s", -1.0)) - event_time_s) < 1e-6
            ):
                return

        self._events.append({"event": event, "time_s": event_time_s})
        self._last_event = event
        self._last_event_time_s = event_time_s
        if event == "enable":
            self._enable_time_s = event_time_s
        elif event == "disable":
            self._disable_time_s = event_time_s

    def _intervals_json(self, scenarios: List[Dict[str, Any]]) -> str:
        """Serialize known attack intervals in a compact, plot-friendly form."""
        intervals: List[Dict[str, Any]] = []
        for scenario in sorted(
            scenarios,
            key=lambda s: (
                float(s.get("t_start", 0.0)),
                str(s.get("name", "")),
                str(s.get("data_type", "")),
            ),
        ):
            fields = scenario.get("target_fields", [])
            if not isinstance(fields, (list, tuple)):
                fields = [fields] if fields is not None else []
            victims = scenario.get("victim_ids", [])
            if not isinstance(victims, (list, tuple)):
                victims = [victims] if victims is not None else []
            intervals.append(
                {
                    "name": str(scenario.get("name", "")),
                    "type": str(scenario.get("type", "")),
                    "modification": str(scenario.get("modification", "")),
                    "data_type": str(scenario.get("data_type", "")),
                    "target_fields": list(fields),
                    "start_s": self._finite_or_none(scenario.get("t_start")),
                    "end_s": self._finite_or_none(scenario.get("t_end")),
                    "attacker_id": scenario.get("attacker_id"),
                    "victim_ids": list(victims),
                    "active": bool(scenario.get("active", False)),
                }
            )
        return json.dumps(intervals, separators=(",", ":"))

    def _events_json(self) -> str:
        return json.dumps(self._events, separators=(",", ":"))

    @staticmethod
    def _unique_values(scenarios: List[Dict[str, Any]], key: str) -> List[str]:
        values: List[str] = []
        for scenario in scenarios:
            value = scenario.get(key)
            if value is None or value == "":
                continue
            value = str(value)
            if value not in values:
                values.append(value)
        return values

    @staticmethod
    def _scenario_targets_vehicle(scenario: Dict[str, Any], vehicle_id: int) -> bool:
        data_type = str(scenario.get("data_type", "")).lower()
        attacker_id = scenario.get("attacker_id")
        victims = scenario.get("victim_ids", [])

        local_target = data_type in ("local", "both") and attacker_id == vehicle_id
        fleet_target = data_type in ("fleet", "both") and (
            -1 in victims or vehicle_id in victims
        )
        return bool(local_target or fleet_target)

    @staticmethod
    def _scenario_is_active_at_clock(
        scenario: Dict[str, Any], clock_s: Optional[float], enabled: bool
    ) -> bool:
        if not enabled or clock_s is None:
            return False
        return bool(scenario["t_start"] <= clock_s <= scenario["t_end"])
