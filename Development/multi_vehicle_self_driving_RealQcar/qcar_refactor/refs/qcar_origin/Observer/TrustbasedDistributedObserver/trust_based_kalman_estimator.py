"""
Trust-Based Kalman Fleet Estimator

Extends TrustBasedFleetEstimator with Kalman-style prediction-correction.
Uses trust scores to weight the measurement update (lower trust = higher R).
"""

import numpy as np
from typing import Dict, Optional

from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Observer.TrustbasedDistributedObserver.trust_based_fleet_estimator import (
    TrustBasedFleetEstimator,
)


class TrustBasedKalmanEstimator(TrustBasedFleetEstimator):
    """
    Trust-Based Distributed Kalman Estimator

    Extends TrustBasedFleetEstimator with Kalman-style prediction-correction.
    Uses trust scores to weight the measurement update.
    """

    def __init__(
        self,
        vehicle_id: int,
        fleet_size: int,
        state_dim: int = 5,
        config: Dict = None,
        logger=None,
    ):
        """Initialize with Kalman-specific parameters"""
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)

        # Kalman filter parameters
        self.process_noise = self.config.get("process_noise", 0.01)
        self.measurement_noise = self.config.get("measurement_noise", 0.1)

        # Covariance matrices per target
        self.covariances: Dict[int, np.ndarray] = {}

        if self.logger:
            self.logger.logger.info(
                f"TrustBasedKalmanEstimator: Using Kalman-style updates with "
                f"Q={self.process_noise}, R={self.measurement_noise}"
            )

    def _trust_weighted_update(
        self,
        target_id: int,
        current_time_ns: int,
        trust_scores: Dict[int, float],
        control: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Override to use Kalman-style prediction-correction with trust-weighted R
        """
        # Current estimate
        current_est = self.fleet_states[:, target_id].copy()

        # Get or initialize covariance
        if target_id not in self.covariances:
            self.covariances[target_id] = np.eye(self.state_dim) * 1.0
        P = self.covariances[target_id]

        # === Prediction Step ===
        target_ctrl = self._get_target_control(target_id, control)
        predicted = self._predict_dynamics(
            current_est, target_ctrl, dt, target_id=target_id
        )

        # Process noise
        Q = np.eye(self.state_dim) * self.process_noise
        P_pred = P + Q

        # === Measurement Update ===
        direct_state = self._get_latest_received_state(target_id, current_time_ns)

        if direct_state is not None:
            # Trust-weighted measurement noise
            target_trust = trust_scores.get(target_id, 0.5)
            # Lower trust = higher measurement noise
            trust_factor = max(target_trust, 0.1)
            R = np.eye(self.state_dim) * (self.measurement_noise / trust_factor)

            # Kalman gain
            S = P_pred + R
            K = P_pred @ np.linalg.inv(S)

            # Update
            innovation = direct_state - predicted
            new_est = predicted + K @ innovation
            P_new = (np.eye(self.state_dim) - K) @ P_pred

            self.covariances[target_id] = P_new
        else:
            # No measurement - just use prediction
            new_est = predicted
            self.covariances[target_id] = P_pred

        # === Consensus Correction (trust-weighted) ===
        consensus_correction = np.zeros(self.state_dim)
        neighbor_count = 0

        for neighbor_id, history in self.received_fleet_states.items():
            neighbor_fleet = self._get_latest_fleet_data(neighbor_id, current_time_ns)
            if neighbor_fleet is None or target_id not in neighbor_fleet:
                continue

            neighbor_trust = trust_scores.get(neighbor_id, 0.0)
            if neighbor_trust < self.weight_config.trust_threshold:
                continue

            neigh_est = neighbor_fleet[target_id]
            neigh_vec = np.array(
                [
                    neigh_est.get("x", 0.0),
                    neigh_est.get("y", 0.0),
                    neigh_est.get("theta", 0.0),
                    neigh_est.get("velocity", 0.0),
                    neigh_est.get("acceleration", 0.0),
                ]
            )

            w = neighbor_trust * self.consensus_gain
            consensus_correction += w * (neigh_vec - new_est)
            neighbor_count += 1

        if neighbor_count > 0:
            new_est += consensus_correction / neighbor_count

        return self._apply_state_constraints(new_est)

    def reset(self):
        """Reset including covariances"""
        super().reset()
        self.covariances.clear()
