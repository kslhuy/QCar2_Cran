"""Functional electronic models for power, MCUs and PCB-mounted sensors."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from .types import ElectronicsSensorFrame, MCUState, PowerSnapshot, VehiclePlantSample


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _quantize(value: float, quantum: float) -> float:
    if quantum <= 0.0:
        return value
    return round(value / quantum) * quantum


@dataclass
class PowerConfig:
    input_voltage_v: float = 12.0
    battery_internal_resistance_ohm: float = 0.08
    buck_output_v: float = 5.0
    buck_efficiency: float = 0.90
    buck_min_input_v: float = 5.5
    ldo_output_v: float = 3.3
    ldo_dropout_v: float = 0.25
    brownout_threshold_v: float = 2.9


class PowerDomainModel:
    """Functional model of the battery, 5 V buck and 3.3 V LDO rails."""

    def __init__(self, config: Optional[PowerConfig] = None) -> None:
        self.config = config or PowerConfig()
        self._input_voltage_override: Optional[float] = None
        self._rail_3v3_scale = 1.0
        self._snapshot = PowerSnapshot(
            input_voltage_v=self.config.input_voltage_v,
            rail_5v_v=0.0,
            rail_3v3_v=0.0,
            input_current_a=0.0,
            input_power_w=0.0,
            power_good=False,
            brownout=True,
        )

    def set_input_voltage(self, voltage_v: Optional[float]) -> None:
        self._input_voltage_override = None if voltage_v is None else max(0.0, float(voltage_v))

    def set_3v3_rail_scale(self, scale: float) -> None:
        self._rail_3v3_scale = max(0.0, float(scale))

    def step(self, load_current_3v3_a: float) -> PowerSnapshot:
        cfg = self.config
        nominal_input = (
            cfg.input_voltage_v
            if self._input_voltage_override is None
            else self._input_voltage_override
        )
        load_current = max(0.0, float(load_current_3v3_a))
        loaded_input = max(
            0.0, nominal_input - load_current * cfg.battery_internal_resistance_ohm
        )
        if loaded_input >= cfg.buck_min_input_v:
            rail_5v = min(cfg.buck_output_v, loaded_input)
        else:
            rail_5v = max(0.0, loaded_input - 0.35)

        rail_3v3 = min(cfg.ldo_output_v, max(0.0, rail_5v - cfg.ldo_dropout_v))
        rail_3v3 *= self._rail_3v3_scale
        output_power = rail_3v3 * load_current
        input_current = (
            output_power / max(loaded_input * cfg.buck_efficiency, 1e-9)
            if loaded_input > 0.0
            else 0.0
        )
        brownout = rail_3v3 < cfg.brownout_threshold_v
        self._snapshot = PowerSnapshot(
            input_voltage_v=loaded_input,
            rail_5v_v=rail_5v,
            rail_3v3_v=rail_3v3,
            input_current_a=input_current,
            input_power_w=loaded_input * input_current,
            power_good=not brownout,
            brownout=brownout,
        )
        return self._snapshot

    @property
    def snapshot(self) -> PowerSnapshot:
        return self._snapshot


class MCUModel:
    """Power/reset/boot state machine for one MCU."""

    def __init__(
        self,
        name: str,
        *,
        boot_time_s: float = 0.05,
        brownout_threshold_v: float = 2.9,
    ) -> None:
        self.name = name
        self.boot_time_ns = max(0, int(round(boot_time_s * 1e9)))
        self.brownout_threshold_v = float(brownout_threshold_v)
        self.state = MCUState.OFF
        self._boot_complete_at_ns = 0
        self._reset_asserted = False

    def set_reset(self, asserted: bool) -> None:
        self._reset_asserted = bool(asserted)

    def reset(self) -> None:
        self.state = MCUState.OFF
        self._boot_complete_at_ns = 0
        self._reset_asserted = False

    def update(self, now_ns: int, rail_voltage_v: float) -> MCUState:
        powered = rail_voltage_v >= self.brownout_threshold_v
        if not powered:
            self.state = MCUState.BROWNOUT if rail_voltage_v > 0.0 else MCUState.OFF
            self._boot_complete_at_ns = 0
            return self.state
        if self._reset_asserted:
            self.state = MCUState.RESET
            self._boot_complete_at_ns = 0
            return self.state
        if self.state in {MCUState.OFF, MCUState.BROWNOUT, MCUState.RESET}:
            self.state = MCUState.BOOTING
            self._boot_complete_at_ns = int(now_ns) + self.boot_time_ns
        if self.state == MCUState.BOOTING and int(now_ns) >= self._boot_complete_at_ns:
            self.state = MCUState.RUNNING
        return self.state


@dataclass
class SensorSuiteConfig:
    imu_rate_hz: float = 200.0
    gnss_rate_hz: float = 10.0
    magnetometer_rate_hz: float = 100.0
    accel_noise_std_mps2: float = 0.025
    gyro_noise_std_rps: float = 0.002
    magnetometer_noise_std_ut: float = 0.15
    gnss_position_noise_std_m: float = 0.05
    gnss_heading_noise_std_rad: float = 0.02
    accel_bias_walk_std_mps2_sqrt_s: float = 0.0005
    gyro_bias_walk_std_rps_sqrt_s: float = 0.00005
    accel_quantum_mps2: float = 0.001
    gyro_quantum_rps: float = 0.0001
    magnetometer_quantum_ut: float = 0.01
    temperature_c: float = 30.0


class ElectronicsSensorSuite:
    """Independent IIM-42652/NEO-M9N/IIS2MDC-style sensor source."""

    def __init__(self, config: Optional[SensorSuiteConfig] = None, *, seed: int = 0) -> None:
        self.config = config or SensorSuiteConfig()
        self._rng = random.Random(seed)
        self._sequence = 0
        self._last_imu_ns = -1
        self._last_gnss_ns = -1
        self._last_mag_ns = -1
        self._accel_bias = [0.0, 0.0, 0.0]
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._last_accel = (0.0, 0.0, 9.81)
        self._last_gyro = (0.0, 0.0, 0.0)
        self._last_mag = (25.0, 0.0, 40.0)
        self._last_position = (0.0, 0.0)
        self._last_heading = 0.0
        self._last_frame: Optional[ElectronicsSensorFrame] = None
        self._faults: Dict[str, Dict[str, Any]] = {}

    def reset(self) -> None:
        self._sequence = 0
        self._last_imu_ns = -1
        self._last_gnss_ns = -1
        self._last_mag_ns = -1
        self._accel_bias = [0.0, 0.0, 0.0]
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._last_frame = None

    def set_fault(self, sensor: str, mode: str = "none", value: float = 0.0) -> None:
        sensor_name = str(sensor).strip().lower()
        mode_name = str(mode).strip().lower()
        if mode_name == "none":
            self._faults.pop(sensor_name, None)
            return
        if mode_name not in {"dropout", "freeze", "bias", "noise_scale"}:
            raise ValueError(f"Unsupported sensor fault mode: {mode}")
        self._faults[sensor_name] = {"mode": mode_name, "value": float(value)}

    @property
    def last_frame(self) -> Optional[ElectronicsSensorFrame]:
        return self._last_frame

    def sample(
        self,
        plant: VehiclePlantSample,
        dt_s: float,
    ) -> Optional[ElectronicsSensorFrame]:
        now_ns = int(plant.timestamp_ns)
        imu_due = self._due(now_ns, self._last_imu_ns, self.config.imu_rate_hz)
        gnss_due = self._due(now_ns, self._last_gnss_ns, self.config.gnss_rate_hz)
        mag_due = self._due(now_ns, self._last_mag_ns, self.config.magnetometer_rate_hz)
        if not (imu_due or gnss_due or mag_due) and self._last_frame is not None:
            return self._last_frame

        if imu_due:
            self._last_imu_ns = now_ns
            self._update_biases(max(0.0, dt_s))
            self._last_accel, self._last_gyro = self._sample_imu(plant)
        if gnss_due:
            self._last_gnss_ns = now_ns
            self._last_position, self._last_heading = self._sample_gnss(plant)
        if mag_due:
            self._last_mag_ns = now_ns
            self._last_mag = self._sample_magnetometer(plant)

        imu_valid = self._fault_mode("imu") != "dropout"
        gnss_valid = self._fault_mode("gnss") != "dropout"
        mag_valid = self._fault_mode("magnetometer") != "dropout"
        self._last_frame = ElectronicsSensorFrame(
            sequence=self._sequence,
            timestamp_ns=now_ns,
            acceleration_mps2=self._last_accel,
            angular_rate_rps=self._last_gyro,
            magnetic_field_ut=self._last_mag,
            position_m=self._last_position,
            heading_rad=self._last_heading,
            temperature_c=self.config.temperature_c,
            imu_valid=imu_valid,
            magnetometer_valid=mag_valid,
            gnss_valid=gnss_valid,
            gnss_fresh=bool(gnss_due and gnss_valid),
        )
        self._sequence += 1
        return self._last_frame

    def _sample_imu(self, plant: VehiclePlantSample) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        if self._fault_mode("imu") == "freeze" and self._last_frame is not None:
            return self._last_accel, self._last_gyro
        noise_scale = self._fault_value("imu", "noise_scale", default=1.0)
        added_bias = self._fault_value("imu", "bias", default=0.0)
        accel_truth = (plant.acceleration_x_mps2, plant.acceleration_y_mps2, 9.81)
        gyro_truth = (0.0, 0.0, plant.yaw_rate_rps)
        accel = tuple(
            _quantize(
                accel_truth[i]
                + self._accel_bias[i]
                + added_bias
                + self._rng.gauss(0.0, self.config.accel_noise_std_mps2 * noise_scale),
                self.config.accel_quantum_mps2,
            )
            for i in range(3)
        )
        gyro = tuple(
            _quantize(
                gyro_truth[i]
                + self._gyro_bias[i]
                + added_bias
                + self._rng.gauss(0.0, self.config.gyro_noise_std_rps * noise_scale),
                self.config.gyro_quantum_rps,
            )
            for i in range(3)
        )
        return accel, gyro

    def _sample_gnss(self, plant: VehiclePlantSample) -> Tuple[Tuple[float, float], float]:
        if self._fault_mode("gnss") == "freeze" and self._last_frame is not None:
            return self._last_position, self._last_heading
        noise_scale = self._fault_value("gnss", "noise_scale", default=1.0)
        added_bias = self._fault_value("gnss", "bias", default=0.0)
        position = (
            plant.x_m + added_bias + self._rng.gauss(0.0, self.config.gnss_position_noise_std_m * noise_scale),
            plant.y_m + added_bias + self._rng.gauss(0.0, self.config.gnss_position_noise_std_m * noise_scale),
        )
        heading = plant.heading_rad + self._rng.gauss(
            0.0, self.config.gnss_heading_noise_std_rad * noise_scale
        )
        return position, heading

    def _sample_magnetometer(self, plant: VehiclePlantSample) -> Tuple[float, float, float]:
        if self._fault_mode("magnetometer") == "freeze" and self._last_frame is not None:
            return self._last_mag
        noise_scale = self._fault_value("magnetometer", "noise_scale", default=1.0)
        added_bias = self._fault_value("magnetometer", "bias", default=0.0)
        horizontal_ut = 25.0
        truth = (
            horizontal_ut * math.cos(plant.heading_rad),
            horizontal_ut * math.sin(plant.heading_rad),
            40.0,
        )
        return tuple(
            _quantize(
                component
                + added_bias
                + self._rng.gauss(0.0, self.config.magnetometer_noise_std_ut * noise_scale),
                self.config.magnetometer_quantum_ut,
            )
            for component in truth
        )

    def _update_biases(self, dt_s: float) -> None:
        scale = math.sqrt(max(dt_s, 0.0))
        for i in range(3):
            self._accel_bias[i] += self._rng.gauss(
                0.0, self.config.accel_bias_walk_std_mps2_sqrt_s * scale
            )
            self._gyro_bias[i] += self._rng.gauss(
                0.0, self.config.gyro_bias_walk_std_rps_sqrt_s * scale
            )

    @staticmethod
    def _due(now_ns: int, last_ns: int, rate_hz: float) -> bool:
        if rate_hz <= 0.0:
            return False
        period_ns = max(1, int(round(1e9 / rate_hz)))
        return last_ns < 0 or now_ns - last_ns >= period_ns

    def _fault_mode(self, sensor: str) -> str:
        return str(self._faults.get(sensor, {}).get("mode", "none"))

    def _fault_value(self, sensor: str, mode: str, *, default: float) -> float:
        fault = self._faults.get(sensor, {})
        if fault.get("mode") != mode:
            return default
        return float(fault.get("value", default))

