"""Fleet lifecycle, peer cache, and publication coordination for one vehicle."""

from __future__ import annotations

import time
from collections.abc import Iterable
from dataclasses import replace
from math import hypot

from core.commands import CommandType, VehicleCommand
from core.types import V2VMessage, VehicleStateEstimate

from .fleet_message import VEHICLE_STATE_ESTIMATE, encode_vehicle_state_estimate
from .fleet_peer_store import FleetPeerStore
from .fleet_registry import FleetRegistry
from .fleet_state_machine import FleetPhase, FleetStateMachine
from .fleet_utils.distributed_observer import DistributedObserverBase, DistributedObserverFake
from .fleet_following_policy import FleetFollowingPolicy
from .fleet_types import (
    FleetCommandResult,
    FleetPeerSnapshot,
    FleetPublication,
    FleetStatus,
    FleetStepResult,
)


class FleetManager:
    """Coordinate local fleet state without owning sockets or actuator commands."""

    def __init__(
        self,
        registry: FleetRegistry,
        vehicle_id: int,
        distributed_observer: DistributedObserverBase | None = None,
    ) -> None:
        self._state = FleetStateMachine(registry, vehicle_id)
        self._registry = registry
        self._vehicle_id = int(vehicle_id)
        self._peers = FleetPeerStore(registry, vehicle_id)
        self._distributed_observer = distributed_observer or DistributedObserverFake({}, vehicle_id)
        self._distributed_observer.start()
        self._distributed_latest = None
        self._build_deadline: float | None = None
        self._last_publication_at: float | None = None
        self._transport = None

    @property
    def phase(self) -> FleetPhase:
        return self._state.phase

    def status(self) -> FleetStatus:
        return self._state.status(self._peers.peer_health())

    def attach_transport(self, transport) -> None:
        """Attach the generic V2V facade used for the fleet's own messages."""
        if not callable(getattr(transport, "drain_received", None)) or not callable(getattr(transport, "publish", None)):
            raise TypeError("fleet transport must provide drain_received() and publish()")
        self._transport = transport

    def request_build(self, *, vehicle_running: bool, now_monotonic: float | None = None) -> bool:
        if not self._state.request_build(vehicle_running=vehicle_running):
            return False
        if vehicle_running:
            self._start_build(now_monotonic)
        return True

    def start_for_vehicle(self, *, now_monotonic: float | None = None) -> bool:
        """Start a previously prepared formation once the vehicle is RUNNING."""
        if not self._state.begin_running():
            return False
        self._start_build(now_monotonic)
        return True

    def is_follower(self) -> bool:
        return self.status().role.value == "follower"

    def _start_build(self, now_monotonic: float | None) -> None:
        self._peers.clear()
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        self._build_deadline = now + self._registry.snapshot().policy.communication.peer_timeout_s
        self._last_publication_at = None

    def handle_command(
        self,
        command: VehicleCommand,
        *,
        vehicle_running: bool,
        now_monotonic: float | None = None,
    ) -> FleetCommandResult:
        """Handle fleet lifecycle commands without changing vehicle safety state."""
        if command.command_type == CommandType.BUILD_FLEET:
            if not self.request_build(vehicle_running=vehicle_running, now_monotonic=now_monotonic):
                return FleetCommandResult(
                    handled=True,
                    accepted=False,
                    reason="fleet_already_prepared_or_operating",
                )
            return FleetCommandResult(handled=True)
        if command.command_type == CommandType.CANCEL_FLEET:
            self.stop_for_vehicle(str(command.payload.get("reason", "cancel_command")))
            return FleetCommandResult(handled=True, stop_vehicle=True, reason="fleet_cancel")
        return FleetCommandResult(handled=False)

    def stop_for_vehicle(self, reason: str = "") -> None:
        """End fleet operation for a vehicle state transition or shutdown."""
        self.request_cancel(reason)
        self.complete_cancellation()

    def abort(self, reason: str) -> None:
        """End fleet operation after a fleet or runtime fault without touching IO."""
        self.fault(reason)
        self.complete_cancellation()

    def request_cancel(self, reason: str = "") -> bool:
        self._build_deadline = None
        return self._state.request_cancel(reason)

    def fault(self, reason: str) -> bool:
        self._build_deadline = None
        return self._state.fault(reason)

    def complete_cancellation(self) -> bool:
        self._build_deadline = None
        return self._state.complete_cancellation()

    def process_ego_estimate(self, estimate: VehicleStateEstimate) -> str | None:
        """Reject fleet operation when the local observer has no usable state."""
        if self.phase in (FleetPhase.BUILDING, FleetPhase.ACTIVE) and not estimate.valid:
            return "fleet local observer estimate invalid"
        return None

    def step(
        self,
        estimate: VehicleStateEstimate,
        messages: Iterable[V2VMessage],
        now_monotonic: float,
        *,
        dt: float = 0.0,
        measurements=None,
    ) -> FleetStepResult:
        """Process one fleet cycle and return decisions without touching IO."""
        if self.phase == FleetPhase.PREPARED:
            # Preparation deliberately has no position or peer-data processing.
            # ``run_cycle`` may drain transport input, but that input cannot
            # become a usable peer cache until BUILDING begins.
            return FleetStepResult(self.status())
        fault_reason = self.process_ego_estimate(estimate)
        if fault_reason is None:
            fault_reason = self.process_received(messages, now_monotonic)
        if fault_reason is not None:
            return FleetStepResult(self.status(), fault_reason=fault_reason)

        self._distributed_latest = self._distributed_observer.update(
            local_estimate=estimate,
            peer_snapshots=self._peers.snapshots(),
            membership_revision=self._registry.snapshot().membership_revision,
            measurements=measurements,
            dt=dt,
        )
        status = self.status()
        publication = self.build_publication(estimate, now_monotonic)
        if status.phase == FleetPhase.BUILDING and status.role.value == "follower":
            return FleetStepResult(
                status,
                publication=publication,
                hold_command=True,
                distributed_estimate=self._distributed_latest,
            )
        if status.phase == FleetPhase.ACTIVE and status.role.value == "follower":
            target = FleetFollowingPolicy().make_reference(estimate, self.predecessor_snapshot(), status)
            if target is None:
                return FleetStepResult(
                    status,
                    publication=publication,
                    fault_reason="fleet follower has no valid predecessor",
                    distributed_estimate=self._distributed_latest,
                )
            return FleetStepResult(
                status,
                publication=publication,
                target=target,
                controller_profile=self._registry.snapshot().policy.follower_controller_profile,
                distributed_estimate=self._distributed_latest,
            )
        return FleetStepResult(
            status,
            publication=publication,
            distributed_estimate=self._distributed_latest,
        )

    def run_cycle(
        self,
        estimate: VehicleStateEstimate,
        *,
        dt: float = 0.0,
        now_monotonic: float | None = None,
        measurements=None,
    ) -> FleetStepResult:
        """Exchange generic V2V messages and return the fleet decision for this cycle."""
        if self._transport is None:
            raise RuntimeError("FleetManager has no attached V2V transport")
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        result = self.step(
            estimate,
            self._transport.drain_received(),
            now,
            dt=dt,
            measurements=measurements,
        )
        if result.fault_reason is not None:
            self.abort(result.fault_reason)
            return replace(result, status=self.status())
        if result.publication is not None:
            self._transport.publish(
                result.publication.message_type,
                result.publication.payload,
                list(result.publication.target_vehicle_ids),
            )
        return result

    def process_received(self, messages: Iterable[V2VMessage], now_monotonic: float) -> str | None:
        """Store inbound estimates and return a safe-stop reason when peer data fails."""
        self._peers.ingest(messages)
        self._peers.prune_stale(now_monotonic)
        if self.phase == FleetPhase.BUILDING:
            if self._peers.all_expected_fresh():
                self._state.activate()
            elif self._build_deadline is not None and now_monotonic >= self._build_deadline:
                return "fleet peer build timeout"
        elif self.phase == FleetPhase.ACTIVE and not self._peers.all_expected_fresh():
            return "fleet peer became stale"
        return None

    def build_publication(
        self,
        estimate: VehicleStateEstimate,
        now_monotonic: float,
    ) -> FleetPublication | None:
        """Create a rate-limited fleet payload for the configured outbound peers."""
        if self.phase not in (FleetPhase.BUILDING, FleetPhase.ACTIVE):
            return None
        formation = self._registry.snapshot()
        period = 1.0 / formation.policy.communication.estimate_rate_hz
        if self._last_publication_at is not None and now_monotonic - self._last_publication_at < period:
            return None
        self._last_publication_at = now_monotonic
        return FleetPublication(
            message_type=VEHICLE_STATE_ESTIMATE,
            payload=encode_vehicle_state_estimate(estimate, self.status()),
            target_vehicle_ids=formation.outbound_peer_ids(self._vehicle_id),
        )

    def snapshots(self) -> tuple[FleetPeerSnapshot, ...]:
        return self._peers.snapshots()

    def predecessor_snapshot(self) -> FleetPeerSnapshot | None:
        return self._peers.predecessor_snapshot()

    def predecessor_age_s(self, now_monotonic: float | None = None) -> float | None:
        """Return the local receive age of the predecessor snapshot."""
        predecessor = self.predecessor_snapshot()
        if predecessor is None:
            return None
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        return max(0.0, now - predecessor.received_at_monotonic)

    def predecessor_gap_m(self, ego_estimate: VehicleStateEstimate) -> float | None:
        """Return current Euclidean spacing to the validated predecessor."""
        predecessor = self.predecessor_snapshot()
        if predecessor is None:
            return None
        return hypot(predecessor.estimate.x - ego_estimate.x, predecessor.estimate.y - ego_estimate.y)

    def counters(self) -> dict[str, int]:
        return self._peers.counters()

    def distributed_estimate(self):
        """Return the latest pass-through or calculated fleet estimate."""
        return self._distributed_latest

    def shutdown(self) -> None:
        """Release the observer owned by this fleet manager."""
        self._distributed_observer.stop()
