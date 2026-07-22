"""Fleet value contracts shared by formation, state, and peer handling."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Mapping

from core.types import ControllerReference, VehicleStateEstimate

if TYPE_CHECKING:
    from .fleet_state_machine import FleetPhase


class FleetError(ValueError):
    """Raised when fleet configuration or membership is invalid."""


class FleetRole(str, Enum):
    LEADER = "leader"
    FOLLOWER = "follower"


class FleetTopology(str, Enum):
    LEADER_FOLLOWER = "leader_follower"
    PREDECESSOR_CHAIN = "predecessor_chain"
    LOOP = "loop"
    VEHICLE_VEHICLE = "vehicle_vehicle"


class EdgeDirection(str, Enum):
    DIRECTED = "directed"
    BIDIRECTIONAL = "bidirectional"


class FollowingPolicy(str, Enum):
    DIRECT_PREDECESSOR = "direct_predecessor"
    LEADER_REFERENCE = "leader_reference"


@dataclass(frozen=True)
class FleetCommunication:
    topology: FleetTopology
    edge_direction: EdgeDirection
    estimate_rate_hz: float
    peer_timeout_s: float

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "FleetCommunication":
        try:
            topology = FleetTopology(_required_string(data, "topology"))
            edge_direction = EdgeDirection(_required_string(data, "edge_direction"))
        except ValueError as exc:
            raise FleetError(f"Unsupported fleet communication setting: {exc}") from exc
        return cls(
            topology,
            edge_direction,
            _positive_number(data.get("ego_estimate_rate_hz"), "communication.ego_estimate_rate_hz"),
            _positive_number(data.get("peer_timeout_s"), "communication.peer_timeout_s"),
        )


@dataclass(frozen=True)
class FleetPolicy:
    following_policy: FollowingPolicy
    communication: FleetCommunication
    follower_controller_profile: str | None = None

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "FleetPolicy":
        try:
            following_policy = FollowingPolicy(_required_string(data, "following_policy"))
        except ValueError as exc:
            raise FleetError(f"Unsupported fleet following policy: {exc}") from exc
        communication_data = data.get("communication")
        if not isinstance(communication_data, Mapping):
            raise FleetError("fleet communication must be a mapping")
        controller_profile = data.get("follower_controller_profile")
        if controller_profile is not None and (not isinstance(controller_profile, str) or not controller_profile):
            raise FleetError("fleet follower_controller_profile must be a non-empty string or null")
        return cls(following_policy, FleetCommunication.from_mapping(communication_data), controller_profile)


@dataclass(frozen=True)
class FleetMember:
    vehicle_id: int
    role: FleetRole
    member_order: int

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "FleetMember":
        vehicle_id = data.get("vehicle_id")
        member_order = data.get("member_order")
        if not _valid_vehicle_id(vehicle_id):
            raise FleetError("fleet member vehicle_id must be a non-negative integer")
        if not isinstance(member_order, int) or isinstance(member_order, bool) or member_order < 0:
            raise FleetError("fleet member member_order must be a non-negative integer")
        try:
            role = FleetRole(_required_string(data, "role"))
        except ValueError as exc:
            raise FleetError(f"Unsupported fleet member role: {exc}") from exc
        return cls(int(vehicle_id), role, member_order)


@dataclass(frozen=True)
class FleetFormation:
    formation_id: str
    membership_revision: int
    members: tuple[FleetMember, ...]
    policy: FleetPolicy

    @property
    def leader(self) -> FleetMember:
        return self.members[0]

    def member(self, vehicle_id: int) -> FleetMember:
        for member in self.members:
            if member.vehicle_id == vehicle_id:
                return member
        raise FleetError(f"Vehicle {vehicle_id} is not a member of fleet '{self.formation_id}'")

    def predecessor(self, vehicle_id: int) -> FleetMember | None:
        member = self.member(vehicle_id)
        if member.role == FleetRole.LEADER:
            return None
        return self.members[self.members.index(member) - 1]

    def successors(self, vehicle_id: int) -> tuple[FleetMember, ...]:
        member = self.member(vehicle_id)
        index = self.members.index(member)
        return () if index + 1 >= len(self.members) else (self.members[index + 1],)

    def outbound_peer_ids(self, vehicle_id: int) -> tuple[int, ...]:
        self.member(vehicle_id)
        return tuple(target for source, target in topology_edges(self.members, self.policy.communication) if source == vehicle_id)

    def inbound_peer_ids(self, vehicle_id: int) -> tuple[int, ...]:
        self.member(vehicle_id)
        return tuple(source for source, target in topology_edges(self.members, self.policy.communication) if target == vehicle_id)


@dataclass(frozen=True)
class FleetStatus:
    formation_id: str
    membership_revision: int
    source_vehicle_id: int
    member_order: int
    role: FleetRole
    phase: "FleetPhase"
    peer_health: tuple[tuple[int, bool], ...] = ()
    reason: str = ""


@dataclass(frozen=True)
class FleetPeerSnapshot:
    source_vehicle_id: int
    member_order: int
    role: FleetRole
    membership_revision: int
    estimate: VehicleStateEstimate
    source_sequence: int
    source_timestamp: float
    received_at_monotonic: float
    valid: bool = True


@dataclass(frozen=True)
class FleetPublication:
    message_type: str
    payload: dict[str, object]
    target_vehicle_ids: tuple[int, ...]


class DistributedEstimateSource(str, Enum):
    """Origin of one state supplied to a distributed observer."""

    MEASUREMENT = "measurement"
    LOCAL_OBSERVER = "local_observer"
    V2V = "v2v"
    DISTRIBUTED_OBSERVER = "distributed_observer"


@dataclass(frozen=True)
class DistributedVehicleEstimate:
    """One vehicle estimate and the source selected by a fleet observer."""

    vehicle_id: int
    estimate: VehicleStateEstimate
    source: DistributedEstimateSource


@dataclass(frozen=True)
class DistributedFleetEstimate:
    """Collection of one vehicle's pass-through or calculated fleet state."""

    observer_vehicle_id: int
    membership_revision: int
    estimates: tuple[DistributedVehicleEstimate, ...]

    def estimate_for(self, vehicle_id: int) -> VehicleStateEstimate | None:
        for item in self.estimates:
            if item.vehicle_id == vehicle_id:
                return item.estimate
        return None

    def source_for(self, vehicle_id: int) -> DistributedEstimateSource | None:
        for item in self.estimates:
            if item.vehicle_id == vehicle_id:
                return item.source
        return None


@dataclass(frozen=True)
class FleetStepResult:
    """Fleet decision returned to the vehicle runtime for one control step."""

    status: FleetStatus
    publication: FleetPublication | None = None
    target: ControllerReference | None = None
    controller_profile: str | None = None
    fault_reason: str | None = None
    hold_command: bool = False
    distributed_estimate: DistributedFleetEstimate | None = None


@dataclass(frozen=True)
class FleetCommandResult:
    """Fleet lifecycle command intent for the vehicle safety supervisor."""

    handled: bool
    accepted: bool = True
    stop_vehicle: bool = False
    reason: str = ""


def topology_edges(
    members: tuple[FleetMember, ...],
    communication: FleetCommunication,
) -> tuple[tuple[int, int], ...]:
    """Return directed transport routes implied by one fleet communication policy."""
    ids = tuple(member.vehicle_id for member in members)
    if communication.topology == FleetTopology.LEADER_FOLLOWER:
        raw_edges = tuple((ids[0], vehicle_id) for vehicle_id in ids[1:])
    elif communication.topology == FleetTopology.PREDECESSOR_CHAIN:
        raw_edges = tuple((ids[index], ids[index + 1]) for index in range(len(ids) - 1))
    elif communication.topology == FleetTopology.LOOP:
        raw_edges = tuple((ids[index], ids[(index + 1) % len(ids)]) for index in range(len(ids)))
    else:
        raw_edges = tuple(
            (source, target)
            for source in ids
            for target in ids
            if source != target
        )
    if communication.edge_direction == EdgeDirection.DIRECTED or communication.topology == FleetTopology.VEHICLE_VEHICLE:
        return raw_edges
    return tuple(sorted(set(raw_edges + tuple((target, source) for source, target in raw_edges))))


def _required_string(data: Mapping[str, object], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        raise FleetError(f"fleet {key} must be a non-empty string")
    return value


def _positive_number(value: object, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise FleetError(f"fleet {name} must be a positive number")
    return float(value)


def _valid_vehicle_id(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
