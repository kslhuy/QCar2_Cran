"""Fleet formation construction and membership validation."""

from __future__ import annotations

from typing import Iterable, Mapping

from .fleet_types import FleetError, FleetFormation, FleetMember, FleetPolicy, FleetRole, FollowingPolicy


class FleetFormationBuilder:
    """Build validated formation snapshots from scenario-owned membership."""

    def build(
        self,
        formation_id: str,
        members: Iterable[FleetMember],
        policy: FleetPolicy,
        *,
        membership_revision: int = 0,
        available_vehicle_ids: Iterable[int] | None = None,
    ) -> FleetFormation:
        if not isinstance(formation_id, str) or not formation_id:
            raise FleetError("fleet formation_id must be a non-empty string")
        if not isinstance(membership_revision, int) or isinstance(membership_revision, bool) or membership_revision < 0:
            raise FleetError("fleet membership_revision must be a non-negative integer")
        ordered = tuple(sorted(tuple(members), key=lambda member: member.member_order))
        _validate_members(ordered, available_vehicle_ids)
        formation = FleetFormation(formation_id, membership_revision, ordered, policy)
        _validate_policy_routes(formation)
        return formation

    def validate_transport_routes(
        self,
        formation: FleetFormation,
        peer_ids_by_vehicle: Mapping[int, Iterable[int]],
    ) -> None:
        """Check scenario V2V peers can carry every topology-required route."""
        for member in formation.members:
            configured_peers = set(peer_ids_by_vehicle.get(member.vehicle_id, ()))
            required_peers = set(formation.outbound_peer_ids(member.vehicle_id))
            required_peers.update(formation.inbound_peer_ids(member.vehicle_id))
            missing = sorted(required_peers - configured_peers)
            if missing:
                raise FleetError(f"Fleet vehicle {member.vehicle_id} is missing V2V routes/peer endpoints {missing}")


def _validate_members(
    members: tuple[FleetMember, ...],
    available_vehicle_ids: Iterable[int] | None,
) -> None:
    if len(members) < 2:
        raise FleetError("A fleet requires one leader and at least one follower")
    vehicle_ids = [member.vehicle_id for member in members]
    orders = [member.member_order for member in members]
    if len(set(vehicle_ids)) != len(vehicle_ids):
        raise FleetError("Fleet member vehicle IDs must be unique")
    if len(set(orders)) != len(orders):
        raise FleetError("Fleet member_order values must be unique")
    if orders != list(range(len(members))):
        raise FleetError("Fleet member_order values must be consecutive and start at zero")
    leaders = [member for member in members if member.role == FleetRole.LEADER]
    if len(leaders) != 1 or members[0].role != FleetRole.LEADER:
        raise FleetError("Fleet order zero must be the single leader")
    if any(member.role != FleetRole.FOLLOWER for member in members[1:]):
        raise FleetError("Every fleet member after the leader must be a follower")
    if available_vehicle_ids is not None:
        missing = sorted(set(vehicle_ids) - set(available_vehicle_ids))
        if missing:
            raise FleetError(f"Fleet members have no scenario vehicle process: {missing}")


def _validate_policy_routes(formation: FleetFormation) -> None:
    if formation.policy.following_policy != FollowingPolicy.DIRECT_PREDECESSOR:
        return
    for follower in formation.members[1:]:
        predecessor = formation.predecessor(follower.vehicle_id)
        assert predecessor is not None
        if follower.vehicle_id not in formation.outbound_peer_ids(predecessor.vehicle_id):
            raise FleetError(
                "Fleet topology does not provide the direct predecessor route "
                f"{predecessor.vehicle_id} -> {follower.vehicle_id}"
            )
