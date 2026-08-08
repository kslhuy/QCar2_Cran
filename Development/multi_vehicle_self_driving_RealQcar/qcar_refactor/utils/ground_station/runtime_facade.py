"""Runtime-facing ground-station facade built on a transport bridge."""

from __future__ import annotations

from collections.abc import Callable

from core.commands import CommandResult, CommandSource, CommandType, VehicleCommand
from core.vehicle_types import ControllerReference, LaserScanSample, VehicleStateEstimate

from .monitoring import MonitoringSnapshot


class GroundStationRuntimeFacade:
    """Own ground-station command pumping, acknowledgements, and monitoring.

    The facade does not interpret vehicle command semantics. It invokes the
    supplied runtime safety callback on the control-loop thread, so the TCP
    bridge never accesses a state machine or an actuator.
    """

    def __init__(self, bridge, command_batch_size: int = 8) -> None:
        if not isinstance(command_batch_size, int) or isinstance(command_batch_size, bool) or command_batch_size <= 0:
            raise ValueError("ground-station command_batch_size must be a positive integer")
        self._bridge = bridge
        self._command_batch_size = command_batch_size
        self._last_command_result: CommandResult | None = None
        self._lidar_diagnostic_enabled = False
        self._last_lidar_timestamp_ns: int | None = None

    @property
    def last_command_result(self) -> CommandResult | None:
        """Return the most recently handled local or external command result."""
        return self._last_command_result

    def start(self) -> None:
        self._bridge.start()

    def stop(self) -> None:
        self._bridge.stop()

    def process_pending(self, apply_command: Callable[[VehicleCommand], CommandResult]) -> None:
        """Apply one bounded FIFO batch on the vehicle control-loop thread."""
        for command in self._bridge.drain_commands(self._command_batch_size):
            self.record_command_result(command, apply_command(command))

    def record_command_result(self, command: VehicleCommand, result: CommandResult) -> CommandResult:
        """Retain the result and acknowledge only ground-station-origin commands."""
        self._last_command_result = result
        if command.source == CommandSource.GROUND_STATION and command.command_type != CommandType.MANUAL_INPUT:
            self._bridge.publish_ack(result)
        return result

    def set_lidar_diagnostic_enabled(self, enabled: bool) -> None:
        """Select whether new local scans may be forwarded for operator display."""

        if not isinstance(enabled, bool):
            raise ValueError("LiDAR diagnostic enabled state must be boolean")
        self._lidar_diagnostic_enabled = enabled
        self._last_lidar_timestamp_ns = None

    def publish_lidar_scan(self, scan: LaserScanSample | None) -> None:
        """Forward only a newly acquired selected diagnostic scan to the bridge."""

        if not self._lidar_diagnostic_enabled or scan is None:
            return
        if not isinstance(scan, LaserScanSample):
            raise TypeError("LiDAR diagnostics require a LaserScanSample")
        if self._last_lidar_timestamp_ns == scan.timestamp_ns:
            return
        self._last_lidar_timestamp_ns = scan.timestamp_ns
        self._bridge.publish_lidar_scan(scan)

    def publish_monitoring(
        self,
        *,
        vehicle_id: int,
        runtime_state: str,
        estimate: VehicleStateEstimate,
        control_reference: ControllerReference,
        fleet=None,
        v2v=None,
        control_mode: str = "auto",
        manual_input_age_s: float | None = None,
        io_healthy: bool = True,
    ) -> None:
        """Build and publish one operator-facing snapshot from public module state."""
        fleet_status = fleet.status() if fleet is not None else None
        peer_snapshots = fleet.snapshots() if fleet is not None else ()
        peer_ages_s = (
            {str(vehicle_id): round(age_s, 3) for vehicle_id, age_s in fleet.peer_ages_s().items()}
            if fleet is not None
            else {}
        )
        fleet_summary = {
            "configured": fleet_status is not None,
            "formation_id": fleet_status.formation_id if fleet_status is not None else "",
            "role": fleet_status.role.value if fleet_status is not None else "",
            "member_order": fleet_status.member_order if fleet_status is not None else None,
            "member_count": len(fleet_status.peer_health) + 1 if fleet_status is not None else 0,
            "reason": fleet_status.reason if fleet_status is not None else "",
            "peer_count": len(peer_snapshots),
            "peer_health": list(fleet_status.peer_health) if fleet_status is not None else [],
            "expected_peer_ids": [vehicle_id for vehicle_id, _healthy in fleet_status.peer_health]
            if fleet_status is not None
            else [],
            "peer_ages_s": peer_ages_s,
        }
        try:
            v2v_status = dict(v2v.get_status()) if v2v is not None else {"enabled": False}
        except Exception:
            v2v_status = {"healthy": False}
        self._bridge.publish_snapshot(
            MonitoringSnapshot(
                vehicle_id=vehicle_id,
                runtime_state=runtime_state,
                fleet_phase=fleet_status.phase.value if fleet_status is not None else "disabled",
                estimate_valid=bool(estimate.valid),
                x_m=float(estimate.x),
                y_m=float(estimate.y),
                heading_rad=float(estimate.theta),
                velocity_mps=float(estimate.velocity),
                io_healthy=bool(io_healthy),
                observer_healthy=bool(estimate.valid),
                v2v_status=v2v_status,
                fleet_summary=fleet_summary,
                control_reference={
                    "target_x_m": float(control_reference.target_x),
                    "target_y_m": float(control_reference.target_y),
                    "target_heading_rad": float(control_reference.target_theta),
                    "target_velocity_mps": float(control_reference.target_velocity),
                    "is_finished": bool(control_reference.is_finished),
                },
                control_mode=str(control_mode),
                manual_input_age_s=manual_input_age_s,
                last_command_result=self._last_command_result,
            )
        )

    def get_status(self) -> dict[str, object]:
        """Expose bridge diagnostics without exposing its internal transport state."""
        return self._bridge.get_status()
