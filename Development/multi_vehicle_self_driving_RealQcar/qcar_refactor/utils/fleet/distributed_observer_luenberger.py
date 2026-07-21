"""Advisory distributed Luenberger-style observer prototype."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from core.types import VehicleStateEstimate

from .distributed_observer_base import DistributedObserverBase
from .fleet_types import (
    DistributedEstimateSource,
    DistributedFleetEstimate,
    DistributedVehicleEstimate,
    FleetPeerSnapshot,
)


class DistributedObserverLuenberger(DistributedObserverBase):
    """Bounded per-state prediction/correction prototype for offline evaluation.

    This is not the paper's full interconnected nonlinear observer and must not
    drive a fleet controller. It provides a stable test surface for the future
    longitudinal model: each available local, measured, or V2V state corrects
    an internal kinematic prediction with bounded Luenberger gains.
    """

    def __init__(self, config: Mapping[str, object], vehicle_id: int, logger=None) -> None:
        super().__init__(config, vehicle_id, logger)
        self._position_gain = _gain(config, "position_gain", 0.35)
        self._heading_gain = _gain(config, "heading_gain", 0.35)
        self._velocity_gain = _gain(config, "velocity_gain", 0.45)
        self._acceleration_gain = _gain(config, "acceleration_gain", 0.30)
        self._state_by_vehicle: dict[int, VehicleStateEstimate] = {}

    def update(
        self,
        *,
        local_estimate: VehicleStateEstimate,
        peer_snapshots: Sequence[FleetPeerSnapshot],
        membership_revision: int,
        measurements: Mapping[int, VehicleStateEstimate] | None = None,
        dt: float = 0.0,
    ) -> DistributedFleetEstimate:
        if not self._started:
            raise RuntimeError("Distributed observer has not started")
        if dt < 0.0:
            raise ValueError("Distributed observer dt cannot be negative")

        inputs: dict[int, VehicleStateEstimate] = dict(measurements or {})
        inputs.setdefault(self._vehicle_id, local_estimate)
        for snapshot in peer_snapshots:
            inputs.setdefault(snapshot.source_vehicle_id, snapshot.estimate)

        for vehicle_id, measurement in inputs.items():
            self._state_by_vehicle[int(vehicle_id)] = self._correct(
                self._state_by_vehicle.get(int(vehicle_id)), measurement, float(dt)
            )

        self._latest = DistributedFleetEstimate(
            observer_vehicle_id=self._vehicle_id,
            membership_revision=int(membership_revision),
            estimates=tuple(
                DistributedVehicleEstimate(
                    vehicle_id, estimate, DistributedEstimateSource.DISTRIBUTED_OBSERVER
                )
                for vehicle_id, estimate in sorted(self._state_by_vehicle.items())
            ),
        )
        return self._latest

    def _correct(
        self,
        previous: VehicleStateEstimate | None,
        measurement: VehicleStateEstimate,
        dt: float,
    ) -> VehicleStateEstimate:
        if previous is None:
            return measurement
        predicted_velocity = previous.velocity + previous.acceleration * dt
        predicted_theta = previous.theta
        predicted_x = previous.x + previous.velocity * math.cos(predicted_theta) * dt
        predicted_y = previous.y + previous.velocity * math.sin(predicted_theta) * dt
        corrected_theta = _wrap(
            predicted_theta + self._heading_gain * _wrap(measurement.theta - predicted_theta)
        )
        return VehicleStateEstimate(
            timestamp=measurement.timestamp,
            x=predicted_x + self._position_gain * (measurement.x - predicted_x),
            y=predicted_y + self._position_gain * (measurement.y - predicted_y),
            theta=corrected_theta,
            velocity=predicted_velocity + self._velocity_gain * (measurement.velocity - predicted_velocity),
            acceleration=previous.acceleration + self._acceleration_gain * (measurement.acceleration - previous.acceleration),
            gps_valid=measurement.gps_valid,
            valid=measurement.valid,
        )


def _gain(config: Mapping[str, object], key: str, default: float) -> float:
    value = config.get(key, default)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{key} must be a number in [0, 1]")
    return float(value)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
