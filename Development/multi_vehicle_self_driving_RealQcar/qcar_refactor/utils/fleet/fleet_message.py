"""Fleet-owned encoding for local vehicle state estimates."""

from __future__ import annotations

from math import isfinite
from typing import Mapping

from core.vehicle_types import V2VMessage, VehicleStateEstimate

from .fleet_types import FleetError, FleetPeerSnapshot, FleetRole, FleetStatus


VEHICLE_STATE_ESTIMATE = "VEHICLE_STATE_ESTIMATE"
FLEET_MESSAGE_SCHEMA_VERSION = 1


class FleetMessageError(FleetError):
    """Raised when a generic V2V payload is not a valid fleet message."""

def encode_vehicle_state_estimate(
    estimate: VehicleStateEstimate,
    status: FleetStatus,
) -> dict[str, object]:
    """Encode one local observer result without adding transport metadata."""
    return {
        "schema_version": FLEET_MESSAGE_SCHEMA_VERSION,
        "membership_revision": status.membership_revision,
        "member_order": status.member_order,
        "role": status.role.value,
        "source_timestamp": _finite_number(estimate.timestamp, "estimate.timestamp"),
        "valid": bool(estimate.valid),
        "estimate": {
            "x": _finite_number(estimate.x, "estimate.x"),
            "y": _finite_number(estimate.y, "estimate.y"),
            "theta": _finite_number(estimate.theta, "estimate.theta"),
            "velocity": _finite_number(estimate.velocity, "estimate.velocity"),
            "acceleration": _finite_number(estimate.acceleration, "estimate.acceleration"),
            "gps_valid": bool(estimate.gps_valid),
        },
    }


def decode_vehicle_state_estimate(message: V2VMessage) -> FleetPeerSnapshot:
    """Decode a fleet estimate using only local receive time for freshness."""
    if message.message_type != VEHICLE_STATE_ESTIMATE:
        raise FleetMessageError(f"Unsupported fleet message type: {message.message_type}")
    if not isinstance(message.payload, Mapping):
        raise FleetMessageError("fleet payload must be a mapping")
    payload = message.payload
    if payload.get("schema_version") != FLEET_MESSAGE_SCHEMA_VERSION:
        raise FleetMessageError("unsupported fleet message schema_version")
    revision = _non_negative_int(payload.get("membership_revision"), "membership_revision")
    member_order = _non_negative_int(payload.get("member_order"), "member_order")
    try:
        role = FleetRole(payload.get("role"))
    except ValueError as exc:
        raise FleetMessageError("role must be a supported fleet role") from exc
    source_timestamp = _finite_number(payload.get("source_timestamp"), "source_timestamp")
    valid = payload.get("valid")
    if not isinstance(valid, bool):
        raise FleetMessageError("valid must be a boolean")
    estimate_data = payload.get("estimate")
    if not isinstance(estimate_data, Mapping):
        raise FleetMessageError("estimate must be a mapping")
    gps_valid = estimate_data.get("gps_valid")
    if not isinstance(gps_valid, bool):
        raise FleetMessageError("estimate.gps_valid must be a boolean")
    received_at = _finite_number(message.received_at_monotonic, "received_at_monotonic")

    return FleetPeerSnapshot(
        source_vehicle_id=_non_negative_int(message.sender_id, "sender_id"),
        member_order=member_order,
        role=role,
        membership_revision=revision,
        estimate=VehicleStateEstimate(
            timestamp=source_timestamp,
            x=_finite_number(estimate_data.get("x"), "estimate.x"),
            y=_finite_number(estimate_data.get("y"), "estimate.y"),
            theta=_finite_number(estimate_data.get("theta"), "estimate.theta"),
            velocity=_finite_number(estimate_data.get("velocity"), "estimate.velocity"),
            acceleration=_finite_number(estimate_data.get("acceleration"), "estimate.acceleration"),
            gps_valid=gps_valid,
            valid=valid,
        ),
        source_sequence=_non_negative_int(message.sequence, "sequence"),
        source_timestamp=source_timestamp,
        received_at_monotonic=received_at,
        valid=valid,
    )


def _finite_number(value: object, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not isfinite(value):
        raise FleetMessageError(f"{name} must be a finite number")
    return float(value)


def _non_negative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise FleetMessageError(f"{name} must be a non-negative integer")
    return int(value)
