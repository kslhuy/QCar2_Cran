"""Typed, transport-independent vehicle command contracts."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from types import MappingProxyType
from typing import Any, Mapping
from uuid import uuid4
import time


class CommandError(ValueError):
    """Raised when an external command is malformed or unsupported."""


class CommandType(str, Enum):
    """Commands supported by the current vehicle runtime."""

    START = "START"
    STOP = "STOP"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RESET = "RESET"
    SET_VELOCITY = "SET_VELOCITY"
    SET_PATH = "SET_PATH"
    ENABLE_SDCS_MAP = "ENABLE_SDCS_MAP"
    DISABLE_SDCS_MAP = "DISABLE_SDCS_MAP"
    BUILD_FLEET = "BUILD_FLEET"
    CANCEL_FLEET = "CANCEL_FLEET"
    ENABLE_MANUAL = "ENABLE_MANUAL"
    DISABLE_MANUAL = "DISABLE_MANUAL"
    MANUAL_INPUT = "MANUAL_INPUT"


class CommandSource(str, Enum):
    """Origin recorded for audit and acknowledgement purposes."""

    LOCAL = "local"
    GROUND_STATION = "ground_station"
    SIMULATOR = "simulator"
    RUNTIME = "runtime"


class CommandOutcome(str, Enum):
    """Execution outcome, distinct from TCP delivery success."""

    APPLIED = "applied"
    DEFERRED = "deferred"
    REJECTED = "rejected"
    FAILED = "failed"


@dataclass(frozen=True)
class VehicleCommand:
    """One validated request to a vehicle runtime.

    ``issued_at_epoch_s`` is for operator audit only. It must not be used as a
    cross-machine control or observer timestamp.
    """

    command_type: CommandType
    payload: Mapping[str, Any] = field(default_factory=dict)
    command_id: str = field(default_factory=lambda: uuid4().hex)
    source: CommandSource = CommandSource.LOCAL
    issued_at_epoch_s: float = field(default_factory=time.time)
    target_vehicle_id: int | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.command_type, CommandType):
            try:
                object.__setattr__(self, "command_type", CommandType(self.command_type))
            except (TypeError, ValueError) as exc:
                raise CommandError("Unsupported command type") from exc
        if not isinstance(self.source, CommandSource):
            try:
                object.__setattr__(self, "source", CommandSource(self.source))
            except (TypeError, ValueError) as exc:
                raise CommandError("Unsupported command source") from exc
        if not isinstance(self.command_id, str) or not self.command_id.strip():
            raise CommandError("command_id must be a non-empty string")
        if not isinstance(self.issued_at_epoch_s, (int, float)) or isinstance(self.issued_at_epoch_s, bool):
            raise CommandError("issued_at_epoch_s must be a finite number")
        if not isfinite(float(self.issued_at_epoch_s)):
            raise CommandError("issued_at_epoch_s must be a finite number")
        if self.target_vehicle_id is not None and (
            not isinstance(self.target_vehicle_id, int)
            or isinstance(self.target_vehicle_id, bool)
            or self.target_vehicle_id < 0
        ):
            raise CommandError("target_vehicle_id must be a non-negative integer or null")
        if not isinstance(self.payload, Mapping):
            raise CommandError("command payload must be a mapping")
        copied_payload = dict(self.payload)
        _validate_payload(self.command_type, copied_payload)
        object.__setattr__(self, "payload", MappingProxyType(copied_payload))

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "VehicleCommand":
        """Decode a serializable command mapping at a transport boundary."""
        if not isinstance(data, Mapping):
            raise CommandError("command must be a mapping")
        try:
            return cls(
                command_type=CommandType(data["command_type"]),
                payload=data.get("payload", {}),
                command_id=data.get("command_id", uuid4().hex),
                source=CommandSource(data.get("source", CommandSource.GROUND_STATION.value)),
                issued_at_epoch_s=data.get("issued_at_epoch_s", time.time()),
                target_vehicle_id=data.get("target_vehicle_id"),
            )
        except KeyError as exc:
            raise CommandError("command_type is required") from exc
        except (TypeError, ValueError) as exc:
            raise CommandError("Unsupported command type or source") from exc

    def to_mapping(self) -> dict[str, Any]:
        """Return the transport-safe representation of this command."""
        return {
            "command_type": self.command_type.value,
            "payload": dict(self.payload),
            "command_id": self.command_id,
            "source": self.source.value,
            "issued_at_epoch_s": float(self.issued_at_epoch_s),
            "target_vehicle_id": self.target_vehicle_id,
        }


@dataclass(frozen=True)
class CommandResult:
    """Serializable acknowledgement generated by the vehicle runtime."""

    command_id: str
    vehicle_id: int
    outcome: CommandOutcome
    runtime_state: str
    reason_code: str = ""
    reason: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.command_id, str) or not self.command_id.strip():
            raise CommandError("command_id must be a non-empty string")
        _vehicle_id(self.vehicle_id)
        if not isinstance(self.outcome, CommandOutcome):
            try:
                object.__setattr__(self, "outcome", CommandOutcome(self.outcome))
            except (TypeError, ValueError) as exc:
                raise CommandError("command result contains an invalid outcome") from exc
        if not isinstance(self.runtime_state, str) or not self.runtime_state.strip():
            raise CommandError("runtime_state must be a non-empty string")
        if not isinstance(self.reason_code, str) or not isinstance(self.reason, str):
            raise CommandError("command result reason fields must be strings")

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "CommandResult":
        if not isinstance(data, Mapping):
            raise CommandError("command result must be a mapping")
        try:
            return cls(
                command_id=_required_string(data, "command_id"),
                vehicle_id=_vehicle_id(data.get("vehicle_id")),
                outcome=CommandOutcome(data["outcome"]),
                runtime_state=_required_string(data, "runtime_state"),
                reason_code=_optional_string(data.get("reason_code")),
                reason=_optional_string(data.get("reason")),
            )
        except KeyError as exc:
            raise CommandError("command result outcome is required") from exc
        except (TypeError, ValueError) as exc:
            raise CommandError("command result contains an invalid value") from exc

    def to_mapping(self) -> dict[str, Any]:
        return {
            "command_id": self.command_id,
            "vehicle_id": self.vehicle_id,
            "outcome": self.outcome.value,
            "runtime_state": self.runtime_state,
            "reason_code": self.reason_code,
            "reason": self.reason,
        }


def _validate_payload(command_type: CommandType, payload: Mapping[str, Any]) -> None:
    allowed = {
        CommandType.START: {"reason"},
        CommandType.STOP: {"reason"},
        CommandType.EMERGENCY_STOP: {"reason"},
        CommandType.RESET: {"reason"},
        CommandType.SET_VELOCITY: {"velocity"},
        CommandType.SET_PATH: {"path"},
        CommandType.ENABLE_SDCS_MAP: {"nodes", "loop"},
        CommandType.DISABLE_SDCS_MAP: set(),
        CommandType.BUILD_FLEET: set(),
        CommandType.CANCEL_FLEET: {"reason"},
        CommandType.ENABLE_MANUAL: set(),
        CommandType.DISABLE_MANUAL: set(),
        CommandType.MANUAL_INPUT: {"throttle", "steering"},
    }[command_type]
    unexpected = set(payload).difference(allowed)
    if unexpected:
        raise CommandError(f"Unsupported payload fields for {command_type.value}: {sorted(unexpected)}")
    if command_type == CommandType.SET_VELOCITY:
        velocity = payload.get("velocity")
        if not isinstance(velocity, (int, float)) or isinstance(velocity, bool) or not isfinite(float(velocity)):
            raise CommandError("SET_VELOCITY requires a finite numeric 'velocity'")
        if float(velocity) < 0.0:
            raise CommandError("SET_VELOCITY velocity must be non-negative")
    if command_type == CommandType.SET_PATH:
        path = payload.get("path")
        if not isinstance(path, str) or not path.strip():
            raise CommandError("SET_PATH requires a non-empty string 'path'")
    if command_type == CommandType.ENABLE_SDCS_MAP:
        nodes = payload.get("nodes")
        if not isinstance(nodes, list) or len(nodes) < 2:
            raise CommandError("ENABLE_SDCS_MAP requires a 'nodes' list with at least two node IDs")
        if any(
            not isinstance(node_id, int) or isinstance(node_id, bool) or not 0 <= node_id <= 10
            for node_id in nodes
        ):
            raise CommandError("ENABLE_SDCS_MAP nodes must be integers in [0, 10]")
        if any(first == second for first, second in zip(nodes, nodes[1:])):
            raise CommandError("ENABLE_SDCS_MAP nodes cannot contain adjacent duplicates")
        loop = payload.get("loop")
        if loop not in (0, 1, 2, "inf") or isinstance(loop, bool):
            raise CommandError("ENABLE_SDCS_MAP loop must be 0, 1, 2, or 'inf'")
    if command_type == CommandType.MANUAL_INPUT:
        for key in ("throttle", "steering"):
            value = payload.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not isfinite(float(value)):
                raise CommandError(f"MANUAL_INPUT requires a finite numeric '{key}'")
        if not -1.0 <= float(payload["throttle"]) <= 1.0:
            raise CommandError("MANUAL_INPUT throttle must be in [-1, 1]")
    if "reason" in payload and (not isinstance(payload["reason"], str) or not payload["reason"].strip()):
        raise CommandError("command reason must be a non-empty string")


def _required_string(data: Mapping[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise CommandError(f"{key} must be a non-empty string")
    return value


def _optional_string(value: Any) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        raise CommandError("command result reason fields must be strings")
    return value


def _vehicle_id(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise CommandError("vehicle_id must be a non-negative integer")
    return value
