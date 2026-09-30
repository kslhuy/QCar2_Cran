"""Adapters connecting the independent electronics twin to vehicle software."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

import numpy as np

from .board import ElectronicsDigitalTwin
from .types import ElectronicsSnapshot, VehiclePlantSample


class MockQCarElectronicsAdapter:
    """Feed vehicle plant truth to PCB sensors after every MockQCar step."""

    def __init__(self, twin: ElectronicsDigitalTwin) -> None:
        self.twin = twin
        self._sim_time_ns = 0
        self._previous_vx: Optional[float] = None
        self._previous_vy: Optional[float] = None

    def reset(self) -> None:
        self._sim_time_ns = 0
        self._previous_vx = None
        self._previous_vy = None
        self.twin.reset()

    def __call__(self, qcar: Any, dt_s: float) -> ElectronicsSnapshot:
        dt_s = max(0.0, float(dt_s))
        self._sim_time_ns += max(1, int(round(dt_s * 1e9)))
        vx = float(getattr(qcar, "velocity", 0.0))
        vy = float(getattr(qcar, "lateral_velocity", 0.0))
        yaw_rate = float(getattr(qcar, "angular_velocity", 0.0))
        if self._previous_vx is None or dt_s <= 0.0:
            ax = 0.0
            ay = yaw_rate * vx
        else:
            ax = (vx - self._previous_vx) / dt_s - yaw_rate * vy
            ay = (vy - self._previous_vy) / dt_s + yaw_rate * vx
        self._previous_vx = vx
        self._previous_vy = vy
        plant = VehiclePlantSample(
            timestamp_ns=self._sim_time_ns,
            x_m=float(getattr(qcar, "x", 0.0)),
            y_m=float(getattr(qcar, "y", 0.0)),
            heading_rad=float(getattr(qcar, "heading", 0.0)),
            velocity_x_mps=vx,
            velocity_y_mps=vy,
            yaw_rate_rps=yaw_rate,
            acceleration_x_mps2=ax,
            acceleration_y_mps2=ay,
        )
        return self.twin.step(plant, dt_s)


class ElectronicsDataGateway:
    """Explicit policy for auxiliary use or fusion with vehicle sensor data."""

    VALID_MODES = {"auxiliary", "prefer_electronics", "weighted_fusion"}

    def __init__(
        self,
        mode: str = "auxiliary",
        *,
        electronics_weight: float = 0.5,
    ) -> None:
        normalized = str(mode).strip().lower()
        if normalized not in self.VALID_MODES:
            raise ValueError(f"Unsupported electronics fusion mode: {mode}")
        self.mode = normalized
        self.electronics_weight = float(np.clip(electronics_weight, 0.0, 1.0))

    @classmethod
    def from_config(cls, config: Optional[Mapping[str, Any]]) -> "ElectronicsDataGateway":
        cfg = dict(config or {})
        return cls(
            mode=str(cfg.get("mode", "auxiliary")),
            electronics_weight=float(cfg.get("electronics_weight", 0.5)),
        )

    def build_update(
        self,
        vehicle_sensor_data: Mapping[str, Any],
        snapshot: ElectronicsSnapshot,
    ) -> Dict[str, Any]:
        update: Dict[str, Any] = {"electronics": snapshot.to_dict()}
        frame = snapshot.compute_node_sensor_frame
        if self.mode == "auxiliary" or frame is None:
            return update

        alpha = 1.0 if self.mode == "prefer_electronics" else self.electronics_weight
        if frame.imu_valid:
            vehicle_accel = np.asarray(
                vehicle_sensor_data.get("accelerometer", np.zeros(3)), dtype=float
            ).reshape(-1)
            if vehicle_accel.size < 3:
                vehicle_accel = np.pad(vehicle_accel, (0, 3 - vehicle_accel.size))
            electronics_accel = np.asarray(frame.acceleration_mps2, dtype=float)
            update["accelerometer"] = (
                alpha * electronics_accel + (1.0 - alpha) * vehicle_accel[:3]
            )
            update["gyro_z"] = (
                alpha * float(frame.angular_rate_rps[2])
                + (1.0 - alpha) * float(vehicle_sensor_data.get("gyro_z", 0.0))
            )
            update["electronics_fusion_applied"] = True

        if frame.gnss_valid:
            vehicle_gps = np.asarray(
                vehicle_sensor_data.get("gps_position", np.zeros(3)), dtype=float
            ).reshape(-1)
            if vehicle_gps.size < 3:
                vehicle_gps = np.pad(vehicle_gps, (0, 3 - vehicle_gps.size))
            electronics_gps = np.asarray(
                [frame.position_m[0], frame.position_m[1], frame.heading_rad],
                dtype=float,
            )
            update["gps_position"] = (
                alpha * electronics_gps + (1.0 - alpha) * vehicle_gps[:3]
            )
            update["gps_valid"] = True
            update["gps_fresh"] = bool(frame.gnss_fresh)
            update["gps_has_fix"] = True
            update["electronics_fusion_applied"] = True
        return update
