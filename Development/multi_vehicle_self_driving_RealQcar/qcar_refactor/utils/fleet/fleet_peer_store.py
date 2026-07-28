"""Validated, process-local cache of fleet peer estimates."""

from __future__ import annotations

from collections.abc import Iterable

from core.vehicle_types import V2VMessage

from .fleet_message import FleetMessageError, VEHICLE_STATE_ESTIMATE, decode_vehicle_state_estimate
from .fleet_registry import FleetRegistry
from .fleet_types import FleetPeerSnapshot


class FleetPeerStore:
    """Keep latest valid snapshots; never compare remote clocks for freshness."""

    def __init__(self, registry: FleetRegistry, vehicle_id: int) -> None:
        self._registry = registry
        self._vehicle_id = int(vehicle_id)
        self._snapshots: dict[int, FleetPeerSnapshot] = {}
        self._membership_revision = registry.snapshot().membership_revision
        self._counters: dict[str, int] = {
            "accepted": 0,
            "malformed": 0,
            "unexpected": 0,
            "wrong_role": 0,
            "obsolete_membership": 0,
            "out_of_order": 0,
            "sequence_gap": 0,
            "invalid": 0,
            "stale_removed": 0,
        }

    def ingest(self, messages: Iterable[V2VMessage]) -> None:
        """Accept only current-membership messages from configured inbound peers."""
        formation = self._registry.snapshot()
        self._sync_membership(formation.membership_revision)
        expected = set(formation.inbound_peer_ids(self._vehicle_id))
        for message in messages:
            if message.message_type != VEHICLE_STATE_ESTIMATE:
                continue
            try:
                snapshot = decode_vehicle_state_estimate(message)
            except FleetMessageError:
                self._counters["malformed"] += 1
                continue
            if snapshot.source_vehicle_id not in expected:
                self._counters["unexpected"] += 1
                continue
            if snapshot.membership_revision != formation.membership_revision:
                self._counters["obsolete_membership"] += 1
                continue
            expected_member = formation.member(snapshot.source_vehicle_id)
            if snapshot.role != expected_member.role or snapshot.member_order != expected_member.member_order:
                self._counters["wrong_role"] += 1
                continue
            if not snapshot.valid:
                self._counters["invalid"] += 1
                self._snapshots.pop(snapshot.source_vehicle_id, None)
                continue
            previous = self._snapshots.get(snapshot.source_vehicle_id)
            if previous is not None and snapshot.source_sequence <= previous.source_sequence:
                self._counters["out_of_order"] += 1
                continue
            if previous is not None and snapshot.source_sequence > previous.source_sequence + 1:
                self._counters["sequence_gap"] += snapshot.source_sequence - previous.source_sequence - 1
            self._snapshots[snapshot.source_vehicle_id] = snapshot
            self._counters["accepted"] += 1

    def prune_stale(self, now_monotonic: float) -> None:
        """Remove expired entries using the receiver's local monotonic clock."""
        timeout = self._registry.snapshot().policy.communication.peer_timeout_s
        self._sync_membership(self._registry.snapshot().membership_revision)
        stale_ids = [
            vehicle_id
            for vehicle_id, snapshot in self._snapshots.items()
            if now_monotonic - snapshot.received_at_monotonic > timeout
        ]
        for vehicle_id in stale_ids:
            del self._snapshots[vehicle_id]
            self._counters["stale_removed"] += 1

    def snapshots(self) -> tuple[FleetPeerSnapshot, ...]:
        self._sync_membership(self._registry.snapshot().membership_revision)
        return tuple(self._snapshots[vehicle_id] for vehicle_id in sorted(self._snapshots))

    def predecessor_snapshot(self) -> FleetPeerSnapshot | None:
        formation = self._registry.snapshot()
        self._sync_membership(formation.membership_revision)
        predecessor = formation.predecessor(self._vehicle_id)
        return None if predecessor is None else self._snapshots.get(predecessor.vehicle_id)

    def all_expected_fresh(self) -> bool:
        formation = self._registry.snapshot()
        self._sync_membership(formation.membership_revision)
        expected = formation.inbound_peer_ids(self._vehicle_id)
        return all(vehicle_id in self._snapshots for vehicle_id in expected)

    def peer_health(self) -> tuple[tuple[int, bool], ...]:
        formation = self._registry.snapshot()
        self._sync_membership(formation.membership_revision)
        expected = formation.inbound_peer_ids(self._vehicle_id)
        return tuple((vehicle_id, vehicle_id in self._snapshots) for vehicle_id in expected)

    def counters(self) -> dict[str, int]:
        return dict(self._counters)

    def clear(self) -> None:
        """Discard peer state before a new fleet run begins."""
        self._snapshots.clear()

    def _sync_membership(self, membership_revision: int) -> None:
        if membership_revision != self._membership_revision:
            self._snapshots.clear()
            self._membership_revision = membership_revision
