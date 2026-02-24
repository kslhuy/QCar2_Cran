"""
Trust-Based Distributed Fleet Estimator

Combines the TriP Trust Model with distributed state estimation.
Provides adaptive weights based on trust scores for consensus-based fleet estimation.

This estimator integrates:
1. TriP Trust Model for trust evaluation
2. Weight Trust Module for adaptive weight calculation
3. Distributed Observer for consensus-based state estimation

Features:
- Trust-aware consensus weights
- Attack detection and mitigation
- Automatic trust score updates
- Integration with V2V communication
- Compatible with the new Observer system architecture
"""

import numpy as np
import time
from typing import Dict, List, Optional, Tuple
from collections import defaultdict

# Import base class and utilities from fleet_state_estimators
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Observer.fleet_state_estimators import (
    FleetStateEstimatorBase,
    _normalize_state_array,
    _state_dict_to_array,
    STATE_FIELDS,
)

# Import trust components
from Observer.TrustbasedDistributedObserver.trust_model import (
    TriPTrustModel,
    TrustConfig,
    TrustScore,
    VehicleData,
)
from Observer.TrustbasedDistributedObserver.weight_trust_module import (
    WeightTrustModule,
    WeightConfig,
    WeightResult,
)
from Observer.TrustbasedDistributedObserver.trust_logger import TrustWeightLogger


class TrustBasedFleetEstimator(FleetStateEstimatorBase):
    """
    Trust-Based Distributed Fleet Estimator

    Extends the base fleet estimator with trust-aware consensus weights.
    Uses the TriP Trust Model for trust evaluation and the Weight Trust Module
    for adaptive weight calculation.

    Algorithm:
    For each target vehicle T estimated by host H:
    1. Evaluate trust for all vehicles using TriP model
    2. Calculate adaptive weights based on trust scores
    3. Update estimate using weighted consensus:
       x̂_new(T) = x̂_old(T)
                 + w0 * (T's_broadcast - x̂_old(T))
                 + Σ w_N * (Neighbor_N's_estimate(T) - x̂_old(T))

    Where:
    - w0: Weight for direct measurement (from target)
    - w_N: Trust-weighted weight for neighbor N's estimate
    """

    def __init__(
        self,
        vehicle_id: int,
        fleet_size: int,
        state_dim: int = 5,
        config: Dict = None,
        logger=None,
    ):
        """
        Initialize Trust-Based Fleet Estimator

        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in fleet
            state_dim: State dimension (default 5: x, y, theta, v, a)
            config: Configuration dict with trust and weight parameters
            logger: Logger instance
        """
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)

        # Parse configuration
        trust_config_dict = self.config.get("trust", {})
        weight_config_dict = self.config.get("weight", {})

        # Create Trust Configuration
        self.trust_config = TrustConfig(
            weight_velocity=trust_config_dict.get("weight_velocity", 2.0),
            weight_distance=trust_config_dict.get("weight_distance", 2.0),
            weight_acceleration=trust_config_dict.get("weight_acceleration", 0.3),
            weight_heading=trust_config_dict.get("weight_heading", 1.0),
            trust_threshold=trust_config_dict.get("trust_threshold", 0.5),
            dirichlet_update_rate=trust_config_dict.get("dirichlet_update_rate", 0.1),
            dirichlet_type=trust_config_dict.get("dirichlet_type", "Dual"),
            monitor_sudden_change=trust_config_dict.get("monitor_sudden_change", True),
            ema_alpha=trust_config_dict.get("ema_alpha", 0.3),
        )

        # Create Weight Configuration
        self.weight_config = WeightConfig(
            weight_type=weight_config_dict.get("weight_type", "trust_based"),
            w0_fixed=weight_config_dict.get("w0_fixed", 0.3),
            w_self_base=weight_config_dict.get("w_self_base", 0.2),
            w_cap=weight_config_dict.get("w_cap", 0.4),
            kappa=weight_config_dict.get("kappa", 5),
            eta=weight_config_dict.get("eta", 0.15),
            enable_smoothing=weight_config_dict.get("enable_smoothing", True),
            trust_threshold=trust_config_dict.get("trust_threshold", 0.5),
        )

        # Initialize Trust Model
        self.trust_model = TriPTrustModel(
            vehicle_id=vehicle_id, config=self.trust_config, logger=logger
        )

        # Initialize Weight Module
        self.weight_module = WeightTrustModule(
            vehicle_id=vehicle_id,
            fleet_size=fleet_size,
            config=self.weight_config,
            logger=logger,
        )

        # Observer gains (for dynamics correction)
        self.observer_gain = self.config.get("observer_gain", 0.1)
        self.consensus_gain = self.config.get("consensus_gain", 0.2)

        # Cache for host state (for trust evaluation)
        self.host_state: Dict = {}

        # Cache for current weight result
        self.current_weight_result: Optional[WeightResult] = None

        # Attack mitigation enabled
        self.attack_mitigation_enabled = self.config.get("attack_mitigation", True)

        # Statistics
        self.stats = {
            "trust_updates": 0,
            "weight_updates": 0,
            "attacks_detected": 0,
            "mitigations_applied": 0,
        }

        if self.logger:
            self.logger.logger.info(
                f"TrustBasedFleetEstimator initialized for vehicle_{vehicle_id} "
                f"with fleet_size={fleet_size}, state_dim={state_dim}, "
                f"weight_type={self.weight_config.weight_type}"
            )

        # Initialize specialized logger for trusts & weights
        self.trust_weight_logger = TrustWeightLogger(
            output_dir=os.path.dirname(os.path.abspath(__file__)), max_vehicles=10
        )
        self.trust_weight_logger.start(vehicle_id)
        self._init_time = time.time()

    def update(
        self,
        local_state: np.ndarray,
        dt: float,
        current_time_ns: int,
        control: np.ndarray,
    ) -> np.ndarray:
        """
        Update fleet state estimates using trust-based consensus

        Args:
            local_state: Host vehicle's local state estimate [state_dim]
            dt: Time step in seconds
            current_time_ns: Current timestamp in nanoseconds
            control: Current control input [steering, throttle]

        Returns:
            Updated fleet states [state_dim x fleet_size]
        """
        try:
            # 1. Ensure capacity and set own state
            self._ensure_fleet_capacity(self.vehicle_id)
            self.fleet_states[:, self.vehicle_id] = local_state.copy()

            # Cache host state for trust evaluation
            self.host_state = {
                "x": local_state[0],
                "y": local_state[1],
                "theta": local_state[2],
                "velocity": local_state[3],
                "acceleration": local_state[4] if len(local_state) > 4 else 0.0,
            }

            # 2. Update trust scores for all known vehicles
            trust_scores = self._update_trust_scores(current_time_ns)

            # 3. Calculate adaptive weights based on trust
            weight_result = self.weight_module.calculate_weights(trust_scores)

            # Cache weight result for use in _trust_weighted_update
            self.current_weight_result = weight_result

            # 3.5. Log trust and weight data together using our specialized logger
            log_data = {
                "w0": weight_result.w0,
                "w_self": weight_result.w_self,
                "neighbors": {},
            }

            for vehicle_id, trust_score in trust_scores.items():
                trust_result = self.trust_model.get_trust_score(vehicle_id)
                neighbor_weight = weight_result.neighbor_weights.get(vehicle_id, 0.0)

                if trust_result is not None:
                    log_data["neighbors"][vehicle_id] = {
                        "trust_score": trust_result.final_score,
                        "velocity_score": trust_result.velocity_score,
                        "distance_score": trust_result.distance_score,
                        "acceleration_score": trust_result.acceleration_score,
                        "heading_score": trust_result.heading_score,
                        "beacon_score": trust_result.beacon_score,
                        "quality_factor": trust_result.quality_factor,
                        "w_neighbor": neighbor_weight,
                        "flag_target_attack": trust_result.flag_target_attack,
                        "flag_local_est_check": trust_result.flag_local_est_check,
                        "flag_global_est_check": trust_result.flag_global_est_check,
                    }
                else:
                    log_data["neighbors"][vehicle_id] = {
                        "trust_score": trust_score,
                        "w_neighbor": neighbor_weight,
                    }

            if not hasattr(self, "_init_time"):
                self._init_time = time.time()
            self.trust_weight_logger.record(time.time() - self._init_time, log_data)

            # Also keep backwards compatibility with the system logger if requested
            if self.logger and hasattr(self.logger, "log_trust_weight"):
                for vehicle_id, trust_score in trust_scores.items():
                    trust_result = self.trust_model.get_trust_score(vehicle_id)
                    trust_data = {}
                    if trust_result is not None:
                        trust_data = {
                            "trust_score": trust_result.final_score,
                            "velocity_score": trust_result.velocity_score,
                            "distance_score": trust_result.distance_score,
                            "acceleration_score": trust_result.acceleration_score,
                            "heading_score": trust_result.heading_score,
                            "beacon_score": trust_result.beacon_score,
                            "quality_factor": trust_result.quality_factor,
                            "flag_target_attack": trust_result.flag_target_attack,
                            "flag_local_est_check": trust_result.flag_local_est_check,
                            "flag_global_est_check": trust_result.flag_global_est_check,
                        }
                    else:
                        trust_data = {"trust_score": trust_score}

                    neighbor_weight = weight_result.neighbor_weights.get(
                        vehicle_id, 0.0
                    )
                    weight_data = {
                        "w0": weight_result.w0,
                        "w_self": weight_result.w_self,
                        "w_neighbor": neighbor_weight,
                    }
                    self.logger.log_trust_weight(vehicle_id, trust_data, weight_data)

            # 4. Update estimates for other vehicles using trust-weighted consensus
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue

                self.fleet_states[:, target_id] = self._trust_weighted_update(
                    target_id=target_id,
                    current_time_ns=current_time_ns,
                    trust_scores=trust_scores,
                    control=control,
                    dt=dt,
                )

            # 5. Check for attacks and apply mitigation
            if self.attack_mitigation_enabled:
                self._apply_attack_mitigation(trust_scores, current_time_ns)

            # 6. Cleanup old data
            self._cleanup_old_data(current_time_ns)

            return self.fleet_states.copy()

        except Exception as e:
            if self.logger:
                self.logger.log_error("TrustBasedFleetEstimator update error", e)
            return self.fleet_states.copy()

    def _update_trust_scores(self, current_time_ns: int) -> Dict[int, float]:
        """
        Update trust scores for all vehicles with received data

        Returns:
            Dict of {vehicle_id: trust_score}
        """
        trust_scores = {}

        for vehicle_id in list(self.received_local_states.keys()):
            if vehicle_id == self.vehicle_id:
                continue

            # Get latest state from vehicle
            latest_state = self._get_latest_received_state(vehicle_id, current_time_ns)
            if latest_state is None:
                continue

            # Create VehicleData object
            target_data = VehicleData(
                vehicle_id=vehicle_id,
                x=latest_state[0],
                y=latest_state[1],
                theta=latest_state[2],
                velocity=latest_state[3],
                acceleration=latest_state[4] if len(latest_state) > 4 else 0.0,
                timestamp_ns=current_time_ns,  # Use current time as proxy
            )

            # Update beacon reception tracking
            self.trust_model.update_beacon_reception(
                vehicle_id, True, current_time_ns / 1e9
            )

            # Calculate trust score
            trust_result = self.trust_model.calculate_trust(
                host_state=self.host_state,
                target_data=target_data,
                current_time_ns=current_time_ns,
            )

            trust_scores[vehicle_id] = trust_result.final_score
            self.stats["trust_updates"] += 1

        return trust_scores

    def _trust_weighted_update(
        self,
        target_id: int,
        current_time_ns: int,
        trust_scores: Dict[int, float],
        control: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Update estimate for a target vehicle using pre-calculated consensus weights

        All weight calculation is handled by weight_module.calculate_weights()
        This method simply applies the weights to the consensus update.

        Algorithm:
        1. Start with current estimate
        2. Add direct measurement correction (if available, using w0 from weight_result)
        3. Add neighbor consensus correction (using neighbor_weights from weight_result)
        4. Apply dynamics prediction correction (if no direct measurement)
        """
        # Current estimate for target
        current_est = self.fleet_states[:, target_id].copy()
        total_correction = np.zeros(self.state_dim)

        # Get direct measurement from target
        direct_state = self._get_latest_received_state(target_id, current_time_ns)

        # Use pre-calculated weights from weight_module
        if self.current_weight_result is None:
            # Fallback: no weights calculated yet, return current estimate
            return current_est

        # === Direct Measurement Correction (Node 0) ===
        if direct_state is not None:
            w0 = self.current_weight_result.w0
            direct_correction = w0 * (direct_state - current_est)
            total_correction += direct_correction

        # === Neighbor Consensus Correction ===
        for neighbor_id, history in self.received_fleet_states.items():
            if neighbor_id == self.vehicle_id:
                continue

            # Get neighbor's estimate of target
            neighbor_fleet = self._get_latest_fleet_data(neighbor_id, current_time_ns)
            if neighbor_fleet is None or target_id not in neighbor_fleet:
                continue

            # Get pre-calculated weight for this neighbor
            w_neighbor = self.current_weight_result.neighbor_weights.get(
                neighbor_id, 0.0
            )
            if w_neighbor <= 0:
                continue  # Skip if no weight assigned

            # Extract neighbor's estimate of target
            neigh_est_dict = neighbor_fleet[target_id]
            neigh_est = np.array(
                [
                    neigh_est_dict.get("x", 0.0),
                    neigh_est_dict.get("y", 0.0),
                    neigh_est_dict.get("theta", 0.0),
                    neigh_est_dict.get("velocity", 0.0),
                    neigh_est_dict.get("acceleration", 0.0),
                ]
            )

            # Apply weight (already normalized, no division needed)
            neighbor_correction = w_neighbor * (neigh_est - current_est)
            total_correction += neighbor_correction

        # === Dynamics Prediction (optional) ===
        # Only apply if no direct measurement available
        if direct_state is None and dt > 0:
            dynamics_pred = self._predict_dynamics(current_est, control, dt)
            dynamics_correction = self.observer_gain * (dynamics_pred - current_est)
            total_correction += dynamics_correction

        # === Apply Update ===
        new_est = current_est + total_correction

        # Apply state constraints
        new_est = self._apply_state_constraints(new_est)

        self.stats["weight_updates"] += 1

        return new_est

    def _predict_dynamics(
        self, state: np.ndarray, control: np.ndarray, dt: float
    ) -> np.ndarray:
        """
        Predict next state using simple dynamics model

        Uses bicycle model for position/heading update
        """
        x, y, theta, v = state[:4]
        a = state[4] if len(state) > 4 else 0.0

        steering = control[0] if len(control) > 0 else 0.0
        throttle = control[1] if len(control) > 1 else 0.0

        # Wheelbase (QCar)
        L = 0.256  # meters

        # Simple bicycle model
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + (v * np.tan(steering) / L) * dt
        v_new = v + throttle * dt
        a_new = throttle / dt if dt > 0 else a

        return np.array([x_new, y_new, theta_new, v_new, a_new])

    def _apply_state_constraints(self, state: np.ndarray) -> np.ndarray:
        """Apply physical constraints to state"""
        constrained = state.copy()

        # Normalize angle to [-pi, pi]
        constrained[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))

        # Velocity constraints (reasonable for QCar)
        constrained[3] = np.clip(state[3], -2.0, 2.0)

        # Acceleration constraints
        if len(state) > 4:
            constrained[4] = np.clip(state[4], -5.0, 5.0)

        return constrained

    def _apply_attack_mitigation(
        self, trust_scores: Dict[int, float], current_time_ns: int
    ):
        """
        Apply attack mitigation based on trust model flags
        """
        attack_flags = self.trust_model.get_attack_flags()

        for vehicle_id, flags in attack_flags.items():
            if flags.get("target_attack", False):
                # Vehicle may be under attack - reduce influence
                self.stats["attacks_detected"] += 1

                if self.logger:
                    self.logger.logger.warning(
                        f"[TRUST] Vehicle_{vehicle_id} flagged for potential attack, "
                        f"reducing influence"
                    )

                # Reduce trust score temporarily
                if vehicle_id in trust_scores:
                    trust_scores[vehicle_id] *= 0.5
                    self.stats["mitigations_applied"] += 1

            if flags.get("local_est_check", False):
                # Local measurements unreliable - rely more on consensus
                if self.logger:
                    self.logger.logger.debug(
                        f"[TRUST] Vehicle_{vehicle_id} local estimates flagged for verification"
                    )

    def _ensure_fleet_capacity(self, min_vehicle_id: int):
        """
        Ensure fleet capacity and expand weight module if needed

        Overrides base class to also expand trust/weight modules
        """
        if min_vehicle_id >= self.fleet_states.shape[1]:
            old_size = self.fleet_states.shape[1]

            # Call parent expansion
            super()._ensure_fleet_capacity(min_vehicle_id)

            # Expand weight module
            self.weight_module.update_fleet_size(self.fleet_size)

    # ===== Extended API for Trust Access =====

    def get_trust_score(self, vehicle_id: int) -> Optional[TrustScore]:
        """Get detailed trust score for a vehicle"""
        return self.trust_model.get_trust_score(vehicle_id)

    def get_all_trust_scores(self) -> Dict[int, float]:
        """Get all trust scores as simple dict"""
        return self.trust_model.get_all_trust_scores()

    def get_trusted_vehicles(self, threshold: float = None) -> List[int]:
        """Get list of trusted vehicle IDs"""
        return self.trust_model.get_trusted_vehicles(threshold)

    def is_vehicle_trusted(self, vehicle_id: int) -> bool:
        """Check if a vehicle is currently trusted"""
        return self.trust_model.is_vehicle_trusted(vehicle_id)

    def get_attack_flags(self) -> Dict[int, Dict[str, bool]]:
        """Get attack detection flags for all vehicles"""
        return self.trust_model.get_attack_flags()

    def get_current_weights(self) -> np.ndarray:
        """Get current consensus weights"""
        return self.weight_module.get_weights_array()

    def get_statistics(self) -> Dict[str, int]:
        """Get estimator statistics"""
        return self.stats.copy()

    def add_neighbor_trust_report(
        self, reporter_id: int, target_id: int, trust_score: float
    ):
        """
        Add trust report from neighbor for cross-validation

        Used when neighbors share their trust assessments
        """
        self.trust_model.add_neighbor_trust_report(reporter_id, target_id, trust_score)

    def reset(self):
        """Reset estimator including trust and weight modules"""
        super().reset()
        self.trust_model.reset()
        self.weight_module.reset()
        self.host_state = {}
        self.stats = {
            "trust_updates": 0,
            "weight_updates": 0,
            "attacks_detected": 0,
            "mitigations_applied": 0,
        }
        self._init_time = time.time()

    def __del__(self):
        if hasattr(self, "trust_weight_logger"):
            self.trust_weight_logger.stop()


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
        predicted = self._predict_dynamics(current_est, control, dt)

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


# Factory function for easy creation
def create_trust_based_estimator(
    estimator_type: str,
    vehicle_id: int,
    fleet_size: int,
    state_dim: int = 5,
    config: Dict = None,
    logger=None,
):
    """
    Factory function to create trust-based estimators

    Args:
        estimator_type: 'trust_consensus' or 'trust_kalman'
        vehicle_id: Host vehicle ID
        fleet_size: Fleet size
        state_dim: State dimension
        config: Configuration dict
        logger: Logger instance

    Returns:
        Trust-based fleet estimator instance
    """
    if estimator_type == "trust_consensus":
        return TrustBasedFleetEstimator(
            vehicle_id, fleet_size, state_dim, config, logger
        )
    elif estimator_type == "trust_kalman":
        return TrustBasedKalmanEstimator(
            vehicle_id, fleet_size, state_dim, config, logger
        )
    else:
        raise ValueError(
            f"Unknown trust-based estimator type: {estimator_type}. "
            f"Available: ['trust_consensus', 'trust_kalman']"
        )
