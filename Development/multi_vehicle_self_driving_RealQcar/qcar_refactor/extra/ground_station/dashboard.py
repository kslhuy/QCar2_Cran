"""Read-only terminal rendering for ground-station session snapshots."""

from __future__ import annotations

import time
from typing import Any, Iterable, Mapping


class GroundStationDashboard:
    """Build a compact, dependency-free status view from server session rows."""

    def __init__(self, *, stale_after_s: float = 1.0) -> None:
        if stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be positive")
        self._stale_after_s = float(stale_after_s)

    def render(self, rows: Iterable[Mapping[str, Any]], now_monotonic: float | None = None) -> str:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        entries = tuple(rows)
        connected_count = sum(1 for row in entries if row.get("connection_state") == "connected")
        lines = [
            f"Ground station | connected: {connected_count} | recent disconnects: {len(entries) - connected_count} | stale after: {self._stale_after_s:.1f} s",
            "ID  Link         Runtime     Mode    Input   Fleet phase/role    State [x, y, yaw; v]         Reference [x, y; v]       Estimate  Peers [id:age]       V2V rx/loss/drop  Rate  Age    Event",
            "--  -----------  ----------  ------  ------  -----------------  ---------------------------  ------------------------  --------  -------------------  ----------------  ----  -----  -----",
        ]
        for row in entries:
            snapshot = row.get("snapshot")
            last_monitoring = row.get("last_monitoring_monotonic")
            if not isinstance(last_monitoring, (int, float)):
                last_monitoring = row.get("connected_at_monotonic", now)
            age_s = max(0.0, now - float(last_monitoring))
            link = str(row.get("connection_state", "connected"))
            if link == "connected" and age_s > self._stale_after_s:
                link = "stale"
            if not isinstance(snapshot, Mapping):
                lines.append(
                    f"{int(row['vehicle_id']):<2}  {link:<11}  awaiting registration snapshot"
                )
                continue
            v2v = snapshot.get("v2v_status", {})
            if not isinstance(v2v, Mapping):
                v2v = {}
            fleet = snapshot.get("fleet_summary", {})
            if not isinstance(fleet, Mapping):
                fleet = {}
            received = _counter(v2v, "messages_received", "received_packets", "received", "packets_received")
            estimated_loss = _counter(v2v, "estimated_packets_lost", "packets_lost")
            dropped = _counter(v2v, "dropped_packets", "dropped", "packets_dropped")
            fleet_status = _fleet_status(snapshot, fleet)
            result = row.get("last_command_result") or snapshot.get("last_command_result") or {}
            if not isinstance(result, Mapping):
                result = {}
            last_command = str(result.get("outcome", "-"))
            reason = str(result.get("reason_code", ""))
            if reason:
                last_command = f"{last_command}:{reason}"
            manual_input_age = snapshot.get("manual_input_age_s")
            manual_input = "-" if manual_input_age is None else f"{float(manual_input_age):.2f}s"
            state = _state_vector(snapshot)
            control_reference = _control_reference(snapshot.get("control_reference"))
            peer_freshness = _peer_freshness(fleet)
            event = _event(last_command, fleet)
            lines.append(
                f"{int(row['vehicle_id']):<2}  {link:<11}  "
                f"{str(snapshot.get('runtime_state', '?')):<10}  "
                f"{str(snapshot.get('control_mode', 'auto')):<6}  "
                f"{manual_input:<6}  "
                f"{fleet_status:<17}  "
                f"{state:<27}  "
                f"{control_reference:<24}  "
                f"{'valid' if snapshot.get('estimate_valid') else 'invalid':<8}  "
                f"{peer_freshness:<19}  "
                f"{received:>5}/{estimated_loss:>4}/{dropped:<4}  {float(row.get('monitoring_rate_hz', 0.0)):>3.0f}Hz  {age_s:>4.2f}s  {event}"
            )
        lines.append("")
        lines.append("Commands: start <id> | stop <id> | emergency-stop <id> | reset <id>")
        lines.append("          set-velocity <id> <m/s> | set-path <id> <csv-path> | build-fleet <id> | cancel-fleet <id>")
        lines.append("          enable-manual <id> | disable-manual <id> | manual <id> <throttle> <steering> | manual-drive <id>")
        lines.append("          list | status <id> | help | quit")
        return "\n".join(lines)


def _counter(status: Mapping[str, Any], *keys: str) -> int:
    for key in keys:
        value = status.get(key)
        if isinstance(value, int) and not isinstance(value, bool):
            return value
    return 0


def _fleet_status(snapshot: Mapping[str, Any], fleet: Mapping[str, Any]) -> str:
    """Render both lifecycle phase and local formation role for operators."""
    if not fleet.get("configured", False):
        return "unavailable"
    phase = str(snapshot.get("fleet_phase", "?"))
    role = str(fleet.get("role", "?"))
    if phase not in {"disabled", "prepared", "building", "active", "cancelling", "fault"}:
        return "unavailable"
    if role not in {"leader", "follower"}:
        return phase
    return f"{phase}/{role}"


def _control_reference(value: Any) -> str:
    if not isinstance(value, Mapping):
        return "unavailable"
    try:
        reference = (
            f"{float(value['target_x_m']):.1f}, {float(value['target_y_m']):.1f}; "
            f"{float(value['target_velocity_mps']):.2f}"
        )
    except (KeyError, TypeError, ValueError):
        return "unavailable"
    return f"{reference} done" if value.get("is_finished") else reference


def _state_vector(snapshot: Mapping[str, Any]) -> str:
    """Format the current estimate in the same compact form as its reference."""
    try:
        return (
            f"{float(snapshot['x_m']):.1f}, {float(snapshot['y_m']):.1f}, "
            f"{float(snapshot['heading_rad']):.2f}; {float(snapshot['velocity_mps']):.2f}"
        )
    except (KeyError, TypeError, ValueError):
        return "unavailable"


def _peer_freshness(fleet: Mapping[str, Any]) -> str:
    """Show each required inbound peer and its local receive age."""
    expected = fleet.get("expected_peer_ids", ())
    ages = fleet.get("peer_ages_s", {})
    if not isinstance(expected, list) or not expected:
        return "-"
    if not isinstance(ages, Mapping):
        ages = {}
    items = []
    for vehicle_id in expected:
        key = str(vehicle_id)
        age = ages.get(key)
        if isinstance(age, (int, float)) and not isinstance(age, bool):
            items.append(f"{vehicle_id}:{float(age):.2f}s")
        else:
            items.append(f"{vehicle_id}:missing")
    return ", ".join(items)


def _event(last_command: str, fleet: Mapping[str, Any]) -> str:
    """Prefer the retained fleet reason because it explains autonomous stops."""
    reason = fleet.get("reason")
    if isinstance(reason, str) and reason:
        return f"fleet:{reason}"
    return last_command
