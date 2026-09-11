"""Shared contracts for the CRAN/SEGULA electronics digital twin.

The contracts deliberately separate vehicle-mounted sensors from the sensors on
the electronics board.  A :class:`VehiclePlantSample` is ground truth supplied
by a vehicle simulator; an :class:`ElectronicsSensorFrame` is what the board's
own peripherals measure after their independent timing, noise and faults.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, Mapping, Optional, Tuple


Vector3 = Tuple[float, float, float]


class MCUState(str, Enum):
    """Power/execution state of any MCU, SoC or logical compute node."""
    OFF = "off"
    BOOTING = "booting"
    RUNNING = "running"
    RESET = "reset"
    BROWNOUT = "brownout"


class FrameStatus(str, Enum):
    OK = "ok"
    CORRUPTED = "corrupted"
    DROPPED = "dropped"


@dataclass(frozen=True)
class VehiclePlantSample:
    """Physical state exposed by the vehicle plant, not a vehicle sensor."""

    timestamp_ns: int
    x_m: float
    y_m: float
    heading_rad: float
    velocity_x_mps: float
    velocity_y_mps: float
    yaw_rate_rps: float
    acceleration_x_mps2: float = 0.0
    acceleration_y_mps2: float = 0.0


@dataclass(frozen=True)
class ElectronicsSensorFrame:
    """One independently sampled sensor frame produced by the PCB twin."""

    sequence: int
    timestamp_ns: int
    acceleration_mps2: Vector3
    angular_rate_rps: Vector3
    magnetic_field_ut: Vector3
    position_m: Tuple[float, float]
    heading_rad: float
    temperature_c: float
    imu_valid: bool = True
    magnetometer_valid: bool = True
    gnss_valid: bool = True
    gnss_fresh: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class BusFrame:
    frame_id: int
    requested_at_ns: int
    source: str
    destination: str
    payload: bytes
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class BusDelivery:
    frame: BusFrame
    available_at_ns: int
    status: FrameStatus
    payload: bytes
    corrupted_bits: int = 0
    reason: str = ""


@dataclass(frozen=True)
class PowerSnapshot:
    input_voltage_v: float
    rail_5v_v: float
    rail_3v3_v: float
    input_current_a: float
    input_power_w: float
    power_good: bool
    brownout: bool

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ElectronicsSnapshot:
    vehicle_id: int
    timestamp_ns: int
    nav_mcu_state: MCUState
    com_mcu_state: MCUState
    power: PowerSnapshot
    nav_sensor_frame: Optional[ElectronicsSensorFrame]
    com_sensor_frame: Optional[ElectronicsSensorFrame]
    nav_protocol: str
    nav_com_interface: str
    vehicle_interface: str
    bus_statistics: Mapping[str, Mapping[str, Any]]
    firmware_status: Mapping[str, Any] = field(default_factory=dict)

    @property
    def healthy(self) -> bool:
        return bool(
            self.power.power_good
            and self.nav_mcu_state == MCUState.RUNNING
            and self.com_mcu_state == MCUState.RUNNING
        )

    @property
    def sensor_node_state(self) -> MCUState:
        """Hardware-neutral alias for the legacy NAV MCU state."""
        return self.nav_mcu_state

    @property
    def compute_node_state(self) -> MCUState:
        """Hardware-neutral alias for the legacy COM MCU state."""
        return self.com_mcu_state

    @property
    def sensor_node_sensor_frame(self) -> Optional[ElectronicsSensorFrame]:
        return self.nav_sensor_frame

    @property
    def compute_node_sensor_frame(self) -> Optional[ElectronicsSensorFrame]:
        return self.com_sensor_frame

    def to_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        result["nav_mcu_state"] = self.nav_mcu_state.value
        result["com_mcu_state"] = self.com_mcu_state.value
        result["sensor_node_state"] = self.sensor_node_state.value
        result["compute_node_state"] = self.compute_node_state.value
        result["sensor_node_sensor_frame"] = (
            None
            if self.sensor_node_sensor_frame is None
            else self.sensor_node_sensor_frame.to_dict()
        )
        result["compute_node_sensor_frame"] = (
            None
            if self.compute_node_sensor_frame is None
            else self.compute_node_sensor_frame.to_dict()
        )
        result["sensor_compute_protocol"] = self.nav_protocol
        result["sensor_compute_interface"] = self.nav_com_interface
        result["healthy"] = self.healthy
        return result
