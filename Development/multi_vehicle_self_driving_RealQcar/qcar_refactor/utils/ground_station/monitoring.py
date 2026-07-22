"""Ground-station monitoring contracts independent of the vehicle control loop."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from types import MappingProxyType
from typing import Any, Mapping

from core.commands import CommandResult


@dataclass(frozen=True)
class MonitoringSnapshot:
    """Latest vehicle state made available to an operator-facing bridge."""

    vehicle_id: int
    runtime_state: str
    fleet_phase: str
    estimate_valid: bool
    x_m: float
    y_m: float
    heading_rad: float
    velocity_mps: float
    io_healthy: bool
    observer_healthy: bool
    v2v_status: Mapping[str, Any]
    fleet_summary: Mapping[str, Any]
    control_mode: str = "auto"
    manual_input_age_s: float | None = None
    last_command_result: CommandResult | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.vehicle_id, int) or isinstance(self.vehicle_id, bool) or self.vehicle_id < 0:
            raise ValueError("vehicle_id must be a non-negative integer")
        if not isinstance(self.runtime_state, str) or not self.runtime_state:
            raise ValueError("runtime_state must be a non-empty string")
        if not isinstance(self.fleet_phase, str) or not self.fleet_phase:
            raise ValueError("fleet_phase must be a non-empty string")
        if not isinstance(self.estimate_valid, bool) or not isinstance(self.io_healthy, bool) or not isinstance(self.observer_healthy, bool):
            raise ValueError("monitoring health fields must be boolean")
        for value, name in (
            (self.x_m, "x_m"),
            (self.y_m, "y_m"),
            (self.heading_rad, "heading_rad"),
            (self.velocity_mps, "velocity_mps"),
        ):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not isfinite(float(value)):
                raise ValueError(f"{name} must be a finite number")
        if not isinstance(self.v2v_status, Mapping) or not isinstance(self.fleet_summary, Mapping):
            raise ValueError("monitoring status sections must be mappings")
        if self.control_mode not in {"auto", "manual"}:
            raise ValueError("control_mode must be 'auto' or 'manual'")
        if self.manual_input_age_s is not None and (
            not isinstance(self.manual_input_age_s, (int, float))
            or isinstance(self.manual_input_age_s, bool)
            or not isfinite(float(self.manual_input_age_s))
            or float(self.manual_input_age_s) < 0.0
        ):
            raise ValueError("manual_input_age_s must be a non-negative finite number or null")
        if self.last_command_result is not None and not isinstance(self.last_command_result, CommandResult):
            raise ValueError("last_command_result must use CommandResult")
        object.__setattr__(self, "v2v_status", MappingProxyType(dict(self.v2v_status)))
        object.__setattr__(self, "fleet_summary", MappingProxyType(dict(self.fleet_summary)))

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "MonitoringSnapshot":
        if not isinstance(data, Mapping):
            raise ValueError("monitoring snapshot must be a mapping")
        command_data = data.get("last_command_result")
        return cls(
            vehicle_id=_vehicle_id(data.get("vehicle_id")),
            runtime_state=_string(data.get("runtime_state"), "runtime_state"),
            fleet_phase=_string(data.get("fleet_phase"), "fleet_phase"),
            estimate_valid=_bool(data.get("estimate_valid"), "estimate_valid"),
            x_m=_number(data.get("x_m"), "x_m"),
            y_m=_number(data.get("y_m"), "y_m"),
            heading_rad=_number(data.get("heading_rad"), "heading_rad"),
            velocity_mps=_number(data.get("velocity_mps"), "velocity_mps"),
            io_healthy=_bool(data.get("io_healthy"), "io_healthy"),
            observer_healthy=_bool(data.get("observer_healthy"), "observer_healthy"),
            v2v_status=dict(_mapping(data.get("v2v_status"), "v2v_status")),
            fleet_summary=dict(_mapping(data.get("fleet_summary"), "fleet_summary")),
            control_mode=data.get("control_mode", "auto"),
            manual_input_age_s=data.get("manual_input_age_s"),
            last_command_result=(CommandResult.from_mapping(command_data) if command_data is not None else None),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "runtime_state": self.runtime_state,
            "fleet_phase": self.fleet_phase,
            "estimate_valid": self.estimate_valid,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "heading_rad": self.heading_rad,
            "velocity_mps": self.velocity_mps,
            "io_healthy": self.io_healthy,
            "observer_healthy": self.observer_healthy,
            "v2v_status": dict(self.v2v_status),
            "fleet_summary": dict(self.fleet_summary),
            "control_mode": self.control_mode,
            "manual_input_age_s": self.manual_input_age_s,
            "last_command_result": (
                self.last_command_result.to_mapping() if self.last_command_result is not None else None
            ),
        }


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _vehicle_id(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("vehicle_id must be a non-negative integer")
    return value


def _number(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value
