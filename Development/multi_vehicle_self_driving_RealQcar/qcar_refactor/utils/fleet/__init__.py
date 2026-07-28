"""Fleet formation, lifecycle, and peer-exchange utilities."""

from .fleet_base import FleetFormationBuilder
from .fleet_utils.distributed_observer import (
    DistributedObserverBase,
    DistributedObserverFake,
    DistributedObserverLuenberger,
    build_distributed_observer,
)
from .fleet_manager import FleetManager
from .fleet_runtime import FleetRuntimeSpec, build_fleet_manager
from .fleet_message import (
    FLEET_MESSAGE_SCHEMA_VERSION,
    VEHICLE_STATE_ESTIMATE,
    FleetMessageError,
    decode_vehicle_state_estimate,
    encode_vehicle_state_estimate,
)
from .fleet_peer_store import FleetPeerStore
from .fleet_state_machine import FleetPhase, FleetStateMachine
from .fleet_types import (
    EdgeDirection,
    DistributedObserverConfig,
    FleetCommunication,
    FleetError,
    FleetFormation,
    FleetMember,
    FleetPeerSnapshot,
    FleetPolicy,
    FleetRole,
    FleetStatus,
    FleetTopology,
    FollowingPolicy,
    DistributedEstimateSource,
    DistributedFleetEstimate,
    DistributedVehicleEstimate,
    FleetCommandResult,
    FleetStepResult,
)
from .fleet_registry import FleetRegistry

__all__ = [
    "EdgeDirection",
    "FleetCommunication",
    "FleetError",
    "FleetFormation",
    "FleetFormationBuilder",
    "DistributedObserverBase",
    "DistributedObserverConfig",
    "DistributedObserverFake",
    "DistributedObserverLuenberger",
    "build_distributed_observer",
    "DistributedEstimateSource",
    "DistributedFleetEstimate",
    "DistributedVehicleEstimate",
    "FleetManager",
    "FleetRuntimeSpec",
    "FleetCommandResult",
    "FleetMessageError",
    "FleetStateMachine",
    "FleetMember",
    "FleetPhase",
    "FleetPeerSnapshot",
    "FleetPeerStore",
    "FleetPolicy",
    "FleetRegistry",
    "FleetRole",
    "FleetStatus",
    "FleetStepResult",
    "FleetTopology",
    "FLEET_MESSAGE_SCHEMA_VERSION",
    "FollowingPolicy",
    "VEHICLE_STATE_ESTIMATE",
    "build_fleet_manager",
    "decode_vehicle_state_estimate",
    "encode_vehicle_state_estimate",
]
