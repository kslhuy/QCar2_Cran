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
            "ID  Link         Runtime     Mode    Input   Fleet      x [m]    y [m]   yaw    v [m/s]  Est  IO/Obs  Peers  V2V rx/drop  Rate  Age    Last command",
            "--  -----------  ----------  ------  ------  ---------  -------  -------  -----  -------  ---  ------  -----  -----------  ----  -----  ------------",
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
            dropped = _counter(v2v, "dropped_packets", "dropped", "packets_dropped")
            peers = _counter(fleet, "peer_count")
            result = row.get("last_command_result") or snapshot.get("last_command_result") or {}
            if not isinstance(result, Mapping):
                result = {}
            last_command = str(result.get("outcome", "-"))
            reason = str(result.get("reason_code", ""))
            if reason:
                last_command = f"{last_command}:{reason}"
            manual_input_age = snapshot.get("manual_input_age_s")
            manual_input = "-" if manual_input_age is None else f"{float(manual_input_age):.2f}s"
            lines.append(
                f"{int(row['vehicle_id']):<2}  {link:<11}  "
                f"{str(snapshot.get('runtime_state', '?')):<10}  "
                f"{str(snapshot.get('control_mode', 'auto')):<6}  "
                f"{manual_input:<6}  "
                f"{str(snapshot.get('fleet_phase', '?')):<9}  "
                f"{float(snapshot.get('x_m', 0.0)):>7.2f}  "
                f"{float(snapshot.get('y_m', 0.0)):>7.2f}  "
                f"{float(snapshot.get('heading_rad', 0.0)):>5.2f}  "
                f"{float(snapshot.get('velocity_mps', 0.0)):>7.2f}  "
                f"{'ok' if snapshot.get('estimate_valid') else 'bad':<3}  "
                f"{'ok' if snapshot.get('io_healthy') else 'bad'}/{'ok' if snapshot.get('observer_healthy') else 'bad':<3}  "
                f"{peers:>5}  "
                f"{received:>5}/{dropped:<5}  {float(row.get('monitoring_rate_hz', 0.0)):>3.0f}Hz  {age_s:>4.2f}s  {last_command}"
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
