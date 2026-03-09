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
from collections import defaultdict, deque

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
            weight_velocity=trust_config_dict.get("weight_velocity", 1.0),
            velocity_tolerance=trust_config_dict.get("velocity_tolerance", 0.3),
            min_velocity_tolerance=trust_config_dict.get("min_velocity_tolerance", 0.05),
            turn_velocity_tolerance_gain=trust_config_dict.get(
                "turn_velocity_tolerance_gain", 0.35
            ),
            accel_velocity_tolerance_gain=trust_config_dict.get(
                "accel_velocity_tolerance_gain", 0.15
            ),
            weight_distance=trust_config_dict.get("weight_distance", 2.0),
            weight_acceleration=trust_config_dict.get("weight_acceleration", 0.3),
            weight_heading=trust_config_dict.get("weight_heading", 1.0),
            trust_threshold=trust_config_dict.get("trust_threshold", 0.5),
            dirichlet_update_rate=trust_config_dict.get("dirichlet_update_rate", 0.1),
            dirichlet_type=trust_config_dict.get("dirichlet_type", "Dual"),
            dirichlet_method=trust_config_dict.get("dirichlet_method", "ema"),
            dirichlet_C=trust_config_dict.get("dirichlet_C", 0.2),
            dirichlet_wt_local=trust_config_dict.get("dirichlet_wt_local", 0.4),
            dirichlet_wt_global=trust_config_dict.get("dirichlet_wt_global", 0.5),
            monitor_sudden_change=trust_config_dict.get("monitor_sudden_change", True),
            ema_alpha=trust_config_dict.get("ema_alpha", 0.3),
            use_message_age_quality=trust_config_dict.get("use_message_age_quality", True),
            distributed_trust_fallback=trust_config_dict.get(
                "distributed_trust_fallback", 0.5
            ),
            distributed_trust_covariance_diag=tuple(
                trust_config_dict.get(
                    "distributed_trust_covariance_diag",
                    [1.5, 1.0, 0.5, 0.5, 0.25],
                )
            ),
            distributed_local_tau2_diag=tuple(
                trust_config_dict.get(
                    "distributed_local_tau2_diag",
                    [1.5, 0.5],
                )
            ),
            use_generalized_trust_vector=trust_config_dict.get(
                "use_generalized_trust_vector", False
            ),
            trust_vector_theta_min=trust_config_dict.get(
                "trust_vector_theta_min",
                trust_config_dict.get("trust_threshold", 0.5),
            ),
            max_message_age_s=trust_config_dict.get("max_message_age_s", 1.0),
            expected_beacon_interval_s=trust_config_dict.get(
                "expected_beacon_interval_s", 0.1
            ),
            sudden_change_threshold=trust_config_dict.get(
                "sudden_change_threshold", 0.5
            ),
            attack_detection_window=trust_config_dict.get(
                "attack_detection_window", 10
            ),
            use_physical_constraints_check=trust_config_dict.get(
                "use_physical_constraints_check", True
            ),
            use_temporal_consistency_check=trust_config_dict.get(
                "use_temporal_consistency_check", True
            ),
            max_velocity=trust_config_dict.get("max_velocity", 30.0),
            max_acceleration=trust_config_dict.get("max_acceleration", 4.0),
            max_deceleration=trust_config_dict.get("max_deceleration", -8.0),
            max_jerk=trust_config_dict.get("max_jerk", 10.0),
            temporal_pos_tolerance_m=trust_config_dict.get(
                "temporal_pos_tolerance_m", 2.0
            ),
            temporal_vel_tolerance=trust_config_dict.get(
                "temporal_vel_tolerance", 1.0
            ),
            trust_decay_lambda=trust_config_dict.get("trust_decay_lambda", 0.2),
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

        # Generalized trust vector O_i(j)
        self.generalized_trust_vector: Dict[int, float] = {self.vehicle_id: 1.0}

        # Attack mitigation enabled
        
        self.attack_mitigation_enabled = self.config.get("attack_mitigation", True)
        self.turn_steering_threshold = self.config.get("trust", {}).get("turn_steering_threshold", 0.05)
        # print("Attack mitigation enabled:", self.attack_mitigation_enabled)

        # Prediction-only mode settings (MATLAB parity)
        self.use_predict_observer = bool(self.config.get("use_predict_observer", False))
        self.max_predict_only_time = float(
            self.config.get("max_predict_only_time", 3.0)
        )
        self.n_good = int(self.config.get("n_good", 3))
        self.blend_thresh = float(self.config.get("blend_thresh", 3.0))
        tol_default = np.array([3.5, 2.0, np.deg2rad(8.0), 2.0, 1.0], dtype=float)
        tol_cfg = self.config.get("similarity_tolerances", tol_default.tolist())
        self.similarity_tolerances = np.asarray(tol_cfg, dtype=float).flatten()
        if self.similarity_tolerances.size < self.state_dim:
            self.similarity_tolerances = np.pad(
                self.similarity_tolerances,
                (0, self.state_dim - self.similarity_tolerances.size),
                mode="edge",
            )
        else:
            self.similarity_tolerances = self.similarity_tolerances[: self.state_dim]
        self.predict_only_counter: Dict[int, int] = defaultdict(int)
        self.predict_only_timer: Dict[int, float] = defaultdict(float)
        self.is_in_prediction_mode: Dict[int, bool] = defaultdict(bool)
        self.self_belief: float = 1.0
        self.self_belief_log: List[float] = []

        # Contamination rollback (trust-triggered replay)
        self.rollback_enabled = bool(self.config.get("rollback_enabled", False))
        self.rollback_window_size = int(self.config.get("rollback_window_size", 15))
        self.rollback_buffer = deque(maxlen=max(self.rollback_window_size, 1))
        self.malicious_vehicles = set()
        self.rollback_stats = {
            "total_rollbacks": 0,
            "vehicles_flagged": [],
            "rollback_times_ns": [],
        }

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
            output_dir=os.path.dirname(os.path.abspath(__file__)), max_vehicles=max(10, fleet_size)
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

            # 2.1 Apply attack mitigation before weight computation/update
            if self.attack_mitigation_enabled:
                self._apply_attack_mitigation(trust_scores, current_time_ns)

            # 2.5 Build generalized trust vector O_i(j) when enabled
            if self.trust_config.use_generalized_trust_vector:
                self.generalized_trust_vector = self.trust_model.compute_generalized_trust_vector(
                    all_vehicle_ids=list(range(self.fleet_size)),
                    direct_neighbor_trust=trust_scores,
                    neighbor_opinions=self.trust_model.neighbor_trust_reports,
                )
            else:
                self.generalized_trust_vector = {
                    self.vehicle_id: 1.0,
                    **{k: float(v) for k, v in trust_scores.items()},
                }

            # 3. Calculate adaptive weights based on trust/opinion
            # For paper mode, this summary is mainly for logging; per-target
            # weights are computed in _trust_weighted_update.
            weight_source_scores = (
                self.generalized_trust_vector
                if self.weight_config.weight_type == "paper"
                else trust_scores
            )
            weight_result = self.weight_module.calculate_weights(weight_source_scores)

            # Cache weight result for use in _trust_weighted_update
            self.current_weight_result = weight_result

            # 3.5. Log trust and weight data together using our specialized logger
            log_data = {
                "w0": weight_result.w0,
                "w_self": weight_result.w_self,
                "total_neighbor_weight": weight_result.total_neighbor_weight,
                "trusted_neighbor_count": len(weight_result.trusted_neighbors),
                "mean_direct_trust": weight_result.mean_source_trust,
                "weighted_neighbor_trust": weight_result.weighted_neighbor_trust,
                "direct_trust": {k: float(v) for k, v in trust_scores.items()},
                "generalized_trust": {
                    k: float(v) for k, v in self.generalized_trust_vector.items()
                },
                "is_turning": int(abs(control[0]) >= self.turn_steering_threshold),
                "host_steering": float(control[0]),
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
                        "local_trust": trust_result.local_trust_sample,
                        "global_trust": trust_result.global_trust_sample,
                        "gamma_host": trust_result.gamma_host,
                        "gamma_local_peer": trust_result.gamma_local_peer,
                        "gamma_self": trust_result.gamma_self,
                        "d_host_mean": trust_result.d_host_mean,
                        "d_local_mean": trust_result.d_local_mean,
                        "d_self": trust_result.d_self,
                        "mi_veh_id": trust_result.mi_veh_id,
                        "mi_dist": trust_result.mi_dist,
                        "mi_elem_idx": trust_result.mi_elem_idx,
                        "mi_elem_val": trust_result.mi_elem_val,
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
            pre_update_states = self.fleet_states.copy()
            step_targets: Dict[int, Dict] = {}
            confidence_scores: List[float] = []
            target_confidence: Dict[int, float] = {self.vehicle_id: 1.0}
            target_prediction_mode: Dict[int, bool] = {self.vehicle_id: False}

            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue

                current_est = self.fleet_states[:, target_id].copy()

                if type(self) is TrustBasedFleetEstimator:
                    normal_est, components = self._trust_weighted_update_with_components(
                        target_id=target_id,
                        current_time_ns=current_time_ns,
                        trust_scores=trust_scores,
                        control=control,
                        dt=dt,
                    )
                else:
                    # Keep child estimator behavior (e.g., TrustBasedKalmanEstimator)
                    normal_est = self._trust_weighted_update(
                        target_id=target_id,
                        current_time_ns=current_time_ns,
                        trust_scores=trust_scores,
                        control=control,
                        dt=dt,
                    )
                    components = {
                        "direct": {"source": -1, "delta": (normal_est - current_est)},
                        "neighbors": {},
                        "dynamics_delta": np.zeros(self.state_dim),
                    }

                predicted_only_est = self._apply_state_constraints(
                    self._predict_dynamics(current_est, control, dt)
                )
                final_est, confidence = self._apply_prediction_mode_switch(
                    target_id=target_id,
                    normal_est=normal_est,
                    predicted_est=predicted_only_est,
                    dt=dt,
                )

                self.fleet_states[:, target_id] = final_est
                confidence_scores.append(confidence)
                target_confidence[target_id] = float(confidence)
                target_prediction_mode[target_id] = bool(
                    self.is_in_prediction_mode.get(target_id, False)
                )
                step_targets[target_id] = components

            # 5. Self-belief from per-target confidence (MATLAB parity)
            if confidence_scores:
                self.self_belief = float(np.mean(confidence_scores))
            else:
                self.self_belief = 1.0
            self.self_belief_log.append(self.self_belief)

            # 5.1 Log richer trust/weight/estimation state
            fleet_estimates = {}
            for vid in range(self.fleet_size):
                state_vec = self.fleet_states[:, vid]
                fleet_estimates[vid] = {
                    "x": float(state_vec[0]),
                    "y": float(state_vec[1]),
                    "theta": float(state_vec[2]),
                    "velocity": float(state_vec[3]) if len(state_vec) > 3 else 0.0,
                    "acceleration": float(state_vec[4]) if len(state_vec) > 4 else 0.0,
                }

            log_data["self_belief"] = float(self.self_belief)
            log_data["estimation_confidence"] = target_confidence
            log_data["prediction_mode"] = target_prediction_mode
            log_data["prediction_mode_count"] = int(
                sum(1 for v in target_prediction_mode.values() if v)
            )
            log_data["fleet_estimates"] = fleet_estimates

            if not hasattr(self, "_init_time"):
                self._init_time = time.time()
            self.trust_weight_logger.record(time.time() - self._init_time, log_data)

            # 6. Store rollback data and trigger if new malicious vehicles detected
            if self.rollback_enabled:
                self._record_rollback_step(
                    current_time_ns=current_time_ns,
                    pre_update_states=pre_update_states,
                    target_components=step_targets,
                )
                self._check_and_trigger_rollback(
                    trust_scores=trust_scores, current_time_ns=current_time_ns
                )

            # 7. Cleanup old data
            self._cleanup_old_data(current_time_ns)

            return self.fleet_states.copy()

        except Exception as e:
            if self.logger:
                self.logger.log_error("TrustBasedFleetEstimator update error", e)
            return self.fleet_states.copy()

    def _get_latest_received_state_with_timestamp(
        self, vehicle_id: int, current_time_ns: int
    ) -> Optional[Tuple[int, np.ndarray]]:
        """
        Return latest valid (timestamp_ns, state_vec) for a vehicle.
        """
        if vehicle_id not in self.received_local_states:
            return None
        history = self.received_local_states[vehicle_id]
        for ts_ns, state in reversed(history):
            if (current_time_ns - ts_ns) > self.max_state_age_ns:
                continue
            if isinstance(state, dict):
                state_vec = _state_dict_to_array(
                    state, self.state_dim, logger=self.logger
                )
            else:
                state_vec = _normalize_state_array(
                    state, self.state_dim, logger=self.logger
                )
            if state_vec is not None:
                return ts_ns, state_vec
        return None

    def _get_latest_fleet_data_with_timestamp(
        self, neighbor_id: int, current_time_ns: int
    ) -> Optional[Tuple[int, Dict]]:
        """
        Return latest valid (timestamp_ns, fleet_data) from a neighbor.
        """
        if neighbor_id not in self.received_fleet_states:
            return None
        history = self.received_fleet_states[neighbor_id]
        for ts_ns, fleet_data in reversed(history):
            if (current_time_ns - ts_ns) <= self.max_state_age_ns:
                return ts_ns, fleet_data
        return None

    def _collect_neighbor_estimates_for_target(
        self, target_id: int, current_time_ns: int
    ) -> Dict[int, VehicleData]:
        """
        Collect neighbors' distributed estimates about one target vehicle.
        """
        estimates: Dict[int, VehicleData] = {}
        for neighbor_id in list(self.received_fleet_states.keys()):
            if neighbor_id == self.vehicle_id:
                continue
            fleet_entry = self._get_latest_fleet_data_with_timestamp(
                neighbor_id, current_time_ns
            )
            if fleet_entry is None:
                continue
            fleet_ts_ns, fleet_data = fleet_entry
            if target_id not in fleet_data:
                continue
            est_dict = fleet_data[target_id]
            estimates[neighbor_id] = VehicleData(
                vehicle_id=target_id,
                x=est_dict.get("x", 0.0),
                y=est_dict.get("y", 0.0),
                theta=est_dict.get("theta", 0.0),
                velocity=est_dict.get("velocity", 0.0),
                acceleration=est_dict.get("acceleration", 0.0),
                timestamp_ns=fleet_ts_ns,
            )
        return estimates

    def _collect_neighbor_estimates_of_vehicle(
        self, vehicle_id: int, current_time_ns: int
    ) -> Dict[int, VehicleData]:
        """
        Collect neighbors' distributed estimates about a specific vehicle.

        Unlike _collect_neighbor_estimates_for_target, this does NOT exclude
        the vehicle itself from being a reporter (useful for collecting
        estimates of the HOST vehicle from all neighbors).

        Returns:
            Dict[reporter_id -> VehicleData] with each neighbor's estimate
            of the requested vehicle.
        """
        estimates: Dict[int, VehicleData] = {}
        for neighbor_id in list(self.received_fleet_states.keys()):
            if neighbor_id == self.vehicle_id:
                continue  # Skip our own data
            fleet_entry = self._get_latest_fleet_data_with_timestamp(
                neighbor_id, current_time_ns
            )
            if fleet_entry is None:
                continue
            fleet_ts_ns, fleet_data = fleet_entry
            if vehicle_id not in fleet_data:
                continue
            est_dict = fleet_data[vehicle_id]
            estimates[neighbor_id] = VehicleData(
                vehicle_id=vehicle_id,
                x=est_dict.get("x", 0.0),
                y=est_dict.get("y", 0.0),
                theta=est_dict.get("theta", 0.0),
                velocity=est_dict.get("velocity", 0.0),
                acceleration=est_dict.get("acceleration", 0.0),
                timestamp_ns=fleet_ts_ns,
            )
        return estimates
    def _update_trust_scores(self, current_time_ns: int) -> Dict[int, float]:
        """
        Update trust scores for all known vehicles.

        Vehicles without fresh packets are still updated via decay.

        Returns:
            Dict of {vehicle_id: trust_score}
        """
        trust_scores: Dict[int, float] = {}

        known_vehicle_ids = set(range(self.fleet_size))
        known_vehicle_ids.update(self.received_local_states.keys())
        known_vehicle_ids.update(self.trust_model.get_all_trust_scores().keys())
        known_vehicle_ids.discard(self.vehicle_id)

        for vehicle_id in sorted(known_vehicle_ids):
            latest = self._get_latest_received_state_with_timestamp(
                vehicle_id, current_time_ns
            )

            if latest is None:
                # Missing/expired packet path: decay trust but keep stateful tracking.
                trust_result = self.trust_model.update_missing_observation(
                    target_id=vehicle_id, current_time_ns=current_time_ns
                )
                trust_scores[vehicle_id] = trust_result.final_score
                self.stats["trust_updates"] += 1
                continue

            packet_ts_ns, latest_state = latest

            target_data = VehicleData(
                vehicle_id=vehicle_id,
                x=latest_state[0],
                y=latest_state[1],
                theta=latest_state[2],
                velocity=latest_state[3],
                acceleration=latest_state[4] if len(latest_state) > 4 else 0.0,
                timestamp_ns=packet_ts_ns,
            )

            self.trust_model.update_beacon_reception(
                vehicle_id, True, packet_ts_ns / 1e9
            )

            # Gather distributed estimates about this target for paper-style DT
            neighbor_estimates = self._collect_neighbor_estimates_for_target(
                vehicle_id, current_time_ns
            )
            # Also collect each neighbor's global estimate of the HOST vehicle.
            # gamma_local_peer needs both to compute implied relative states.
            neighbor_host_estimates = self._collect_neighbor_estimates_of_vehicle(
                self.vehicle_id, current_time_ns
            )
            host_target_estimate = self.fleet_states[:, vehicle_id].copy()

            target_fleet_data = None
            fleet_entry = self._get_latest_fleet_data_with_timestamp(vehicle_id, current_time_ns)
            if fleet_entry is not None:
                _, target_fleet_data = fleet_entry

            trust_result = self.trust_model.calculate_trust(
                host_state=self.host_state,
                target_data=target_data,
                neighbor_estimates=neighbor_estimates,
                neighbor_host_estimates=neighbor_host_estimates,
                host_target_estimate=host_target_estimate,
                host_fleet_estimates=self.fleet_states.copy(),
                target_fleet_estimates=target_fleet_data,
                current_time_ns=current_time_ns,
                has_fleet_data=(target_fleet_data is not None),
            )

            trust_scores[vehicle_id] = trust_result.final_score
            self.stats["trust_updates"] += 1

        return trust_scores

    def _trust_weighted_update_with_components(
        self,
        target_id: int,
        current_time_ns: int,
        trust_scores: Dict[int, float],
        control: np.ndarray,
        dt: float,
    ) -> Tuple[np.ndarray, Dict]:
        """
        Update estimate for a target vehicle and return replayable contribution terms.

        Contributions are structured so rollback can remove specific malicious sources.
        """
        current_est = self.fleet_states[:, target_id].copy()
        total_correction = np.zeros(self.state_dim)

        components = {
            "direct": {"source": target_id, "delta": np.zeros(self.state_dim)},
            "neighbors": {},
            "dynamics_delta": np.zeros(self.state_dim),
        }

        # Get direct measurement from target
        direct_state = self._get_latest_received_state(target_id, current_time_ns)

        # Cache available neighbor fleet snapshots containing this target
        neighbor_fleet_estimates: Dict[int, Dict] = {}
        for neighbor_id in self.received_fleet_states.keys():
            if neighbor_id == self.vehicle_id:
                continue

            # Get neighbor's estimate of target
            neighbor_fleet = self._get_latest_fleet_data(neighbor_id, current_time_ns)
            if neighbor_fleet is None or target_id not in neighbor_fleet:
                continue
            neighbor_fleet_estimates[neighbor_id] = neighbor_fleet

        if self.weight_config.weight_type == "paper":
            opinion_scores = (
                self.generalized_trust_vector
                if self.generalized_trust_vector
                else trust_scores
            )
            target_trust_obj = self.trust_model.get_trust_score(target_id)
            target_local_trust = (
                target_trust_obj.local_trust_sample
                if target_trust_obj is not None
                else trust_scores.get(target_id, 0.0)
            )

            target_weights = self.weight_module.calculate_paper_weights_for_target(
                target_id=target_id,
                opinion_scores=opinion_scores,
                target_local_trust=target_local_trust,
                neighbor_fleet_estimates=neighbor_fleet_estimates,
                direct_measurement=direct_state,
            )

            if direct_state is not None and target_weights["w0"] > 0:
                direct_delta = target_weights["w0"] * (direct_state - current_est)
                total_correction += direct_delta
                components["direct"] = {"source": target_id, "delta": direct_delta}

            for neighbor_id, neighbor_fleet in neighbor_fleet_estimates.items():
                w_neighbor = target_weights["neighbors"].get(neighbor_id, 0.0)
                if w_neighbor <= 0:
                    continue
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
                neighbor_delta = w_neighbor * (neigh_est - current_est)
                total_correction += neighbor_delta
                components["neighbors"][neighbor_id] = neighbor_delta
        else:
            target_weights = self.weight_module.calculate_weights_for_target(
                target_id=target_id,
                trust_scores=trust_scores,
                neighbor_fleet_estimates=neighbor_fleet_estimates,
                direct_measurement=direct_state,
            )

            # === Direct Measurement Correction (Node 0) ===
            if direct_state is not None and target_weights["w0"] > 0:
                direct_delta = target_weights["w0"] * (direct_state - current_est)
                total_correction += direct_delta
                components["direct"] = {"source": target_id, "delta": direct_delta}

            # === Neighbor Consensus Correction ===
            for neighbor_id, neighbor_fleet in neighbor_fleet_estimates.items():
                w_neighbor = target_weights["neighbors"].get(neighbor_id, 0.0)
                if w_neighbor <= 0:
                    continue

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
                neighbor_delta = w_neighbor * (neigh_est - current_est)
                total_correction += neighbor_delta
                components["neighbors"][neighbor_id] = neighbor_delta

        # === Dynamics Prediction (optional) ===
        # Only apply if no direct measurement available
        if direct_state is None and dt > 0:
            dynamics_pred = self._predict_dynamics(current_est, control, dt)
            dynamics_correction = self.observer_gain * (dynamics_pred - current_est)
            total_correction += dynamics_correction
            components["dynamics_delta"] = dynamics_correction

        # === Apply Update ===
        new_est = current_est + total_correction

        # Apply state constraints
        new_est = self._apply_state_constraints(new_est)

        self.stats["weight_updates"] += 1

        return new_est, components

    def _trust_weighted_update(
        self,
        target_id: int,
        current_time_ns: int,
        trust_scores: Dict[int, float],
        control: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Backward-compatible wrapper returning only updated state.
        """
        new_est, _ = self._trust_weighted_update_with_components(
            target_id=target_id,
            current_time_ns=current_time_ns,
            trust_scores=trust_scores,
            control=control,
            dt=dt,
        )
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

    def _apply_prediction_mode_switch(
        self,
        target_id: int,
        normal_est: np.ndarray,
        predicted_est: np.ndarray,
        dt: float,
    ) -> Tuple[np.ndarray, float]:
        """
        MATLAB-inspired prediction-only switching logic.
        """
        if not self.use_predict_observer:
            self.is_in_prediction_mode[target_id] = False
            return normal_est, 1.0

        tol = np.maximum(self.similarity_tolerances[: self.state_dim], 1e-6)
        diff = normal_est - predicted_est
        normalized_diff = diff / tol
        confidence = float(np.exp(-0.5 * np.mean(normalized_diff**2)))
        is_ok = bool(np.all(np.abs(diff) <= tol))

        if is_ok:
            self.predict_only_counter[target_id] += 1
        else:
            self.predict_only_counter[target_id] = 0

        in_pred_mode = self.predict_only_counter[target_id] < max(self.n_good, 1)
        self.is_in_prediction_mode[target_id] = in_pred_mode

        if in_pred_mode and (not is_ok):
            self.predict_only_timer[target_id] += max(dt, 0.0)
        else:
            self.predict_only_timer[target_id] = 0.0

        if self.predict_only_timer[target_id] >= self.max_predict_only_time:
            # Force recovery to normal mode after prolonged prediction-only operation
            self.predict_only_counter[target_id] = max(self.n_good, 1)
            self.predict_only_timer[target_id] = 0.0
            self.is_in_prediction_mode[target_id] = False
            return normal_est, confidence

        if not in_pred_mode:
            return normal_est, confidence

        # In prediction-only mode:
        # - if mismatch is moderate, blend
        # - else rely on pure prediction to isolate suspicious direct/neighbor data
        diff_norm = float(np.linalg.norm(normalized_diff))
        if (not is_ok) and diff_norm < max(self.blend_thresh, 1e-6):
            alpha = min(0.5, diff_norm / max(self.blend_thresh, 1e-6))
            blended = (1.0 - alpha) * normal_est + alpha * predicted_est
            return self._apply_state_constraints(blended), confidence
        return predicted_est, confidence

    def _record_rollback_step(
        self,
        current_time_ns: int,
        pre_update_states: np.ndarray,
        target_components: Dict[int, Dict],
    ) -> None:
        """Store replay data for trust-triggered contamination rollback."""
        if not self.rollback_enabled:
            return

        safe_targets: Dict[int, Dict] = {}
        for target_id, comp in target_components.items():
            direct = comp.get("direct", {})
            safe_direct = {
                "source": int(direct.get("source", -1)),
                "delta": np.asarray(
                    direct.get("delta", np.zeros(self.state_dim)), dtype=float
                ).copy(),
            }

            safe_neighbors: Dict[int, np.ndarray] = {}
            for nid, ndelta in comp.get("neighbors", {}).items():
                safe_neighbors[int(nid)] = np.asarray(ndelta, dtype=float).copy()

            safe_targets[int(target_id)] = {
                "direct": safe_direct,
                "neighbors": safe_neighbors,
                "dynamics_delta": np.asarray(
                    comp.get("dynamics_delta", np.zeros(self.state_dim)), dtype=float
                ).copy(),
            }

        self.rollback_buffer.append(
            {
                "time_ns": int(current_time_ns),
                "pre_update_states": np.asarray(pre_update_states, dtype=float).copy(),
                "targets": safe_targets,
            }
        )

    def _check_and_trigger_rollback(
        self, trust_scores: Dict[int, float], current_time_ns: int
    ) -> None:
        if not self.rollback_enabled:
            return

        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
        newly_malicious: List[int] = []

        for vehicle_id, trust_val in trust_scores.items():
            if vehicle_id == self.vehicle_id:
                continue
            if trust_val < threshold:
                if vehicle_id not in self.malicious_vehicles:
                    self.malicious_vehicles.add(vehicle_id)
                    newly_malicious.append(vehicle_id)
            else:
                if vehicle_id in self.malicious_vehicles:
                    self.malicious_vehicles.remove(vehicle_id)

        for malicious_id in newly_malicious:
            self._trigger_contamination_rollback(malicious_id, current_time_ns)

    def _trigger_contamination_rollback(
        self, malicious_vehicle_id: int, current_time_ns: int
    ) -> None:
        """
        Replay buffered steps while excluding malicious source contributions.
        """
        if not self.rollback_buffer:
            return

        oldest = self.rollback_buffer[0]
        corrected_states = np.asarray(oldest["pre_update_states"], dtype=float).copy()
        current_self_state = self.fleet_states[:, self.vehicle_id].copy()

        for step in list(self.rollback_buffer):
            step_targets = step.get("targets", {})
            for target_id in range(self.fleet_size):
                if target_id == self.vehicle_id:
                    continue
                comp = step_targets.get(target_id)
                if comp is None:
                    continue
                corrected_states[:, target_id] = self._replay_step_without_malicious(
                    comp=comp,
                    previous_state=corrected_states[:, target_id],
                    malicious_vehicle_id=malicious_vehicle_id,
                )

        corrected_states[:, self.vehicle_id] = current_self_state
        self.fleet_states = corrected_states

        self.rollback_buffer.clear()
        self.rollback_stats["total_rollbacks"] += 1
        self.rollback_stats["vehicles_flagged"].append(int(malicious_vehicle_id))
        self.rollback_stats["rollback_times_ns"].append(int(current_time_ns))

    def _replay_step_without_malicious(
        self, comp: Dict, previous_state: np.ndarray, malicious_vehicle_id: int
    ) -> np.ndarray:
        """Rebuild one target update while excluding malicious contributors."""
        delta = np.zeros(self.state_dim)

        direct = comp.get("direct", {})
        direct_src = int(direct.get("source", -1))
        if direct_src != malicious_vehicle_id:
            delta += np.asarray(direct.get("delta", np.zeros(self.state_dim)), dtype=float)

        for neighbor_id, ndelta in comp.get("neighbors", {}).items():
            if int(neighbor_id) == malicious_vehicle_id:
                continue
            delta += np.asarray(ndelta, dtype=float)

        delta += np.asarray(comp.get("dynamics_delta", np.zeros(self.state_dim)), dtype=float)
        return self._apply_state_constraints(previous_state + delta)

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

                # if self.logger:
                #     self.logger.logger.warning(
                #         f"[TRUST] Vehicle_{vehicle_id} flagged for potential attack, "
                #         f"reducing influence"
                #     )

                # Reduce trust score temporarily
                if vehicle_id in trust_scores:
                    trust_scores[vehicle_id] = float(
                        np.clip(trust_scores[vehicle_id] * 0.5, 0.0, 1.0)
                    )
                    self.stats["mitigations_applied"] += 1

            # if flags.get("local_est_check", False):
            #     # Local measurements unreliable - rely more on consensus
            #     if self.logger:
            #         self.logger.logger.debug(
            #             f"[TRUST] Vehicle_{vehicle_id} local estimates flagged for verification"
            #         )

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

    def get_generalized_trust_vector(self) -> Dict[int, float]:
        """Get generalized trust vector O_i(j)."""
        return self.generalized_trust_vector.copy()

    def get_trusted_vehicles(self, threshold: float = None) -> List[int]:
        """Get list of trusted vehicle IDs"""
        return self.trust_model.get_trusted_vehicles(threshold)

    def is_vehicle_trusted(self, vehicle_id: int) -> bool:
        """Check if a vehicle is currently trusted"""
        return self.trust_model.is_vehicle_trusted(vehicle_id)

    def get_attack_flags(self) -> Dict[int, Dict[str, bool]]:
        """Get attack detection flags for all vehicles"""
        return self.trust_model.get_attack_flags()

    def get_self_belief(self) -> float:
        """Get latest self-belief value derived from prediction-mode confidence."""
        return float(self.self_belief)

    def is_vehicle_in_prediction_mode(self, vehicle_id: int) -> bool:
        """Check whether a specific target is currently in prediction-only mode."""
        return bool(self.is_in_prediction_mode.get(vehicle_id, False))

    def get_current_weights(self) -> np.ndarray:
        """Get current consensus weights"""
        return self.weight_module.get_weights_array()

    def get_statistics(self) -> Dict[str, object]:
        """Get estimator statistics"""
        data = self.stats.copy()
        data["self_belief"] = float(self.self_belief)
        data["rollbacks"] = self.rollback_stats.copy()
        return data

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
        self.generalized_trust_vector = {self.vehicle_id: 1.0}
        self.stats = {
            "trust_updates": 0,
            "weight_updates": 0,
            "attacks_detected": 0,
            "mitigations_applied": 0,
        }
        self.predict_only_counter.clear()
        self.predict_only_timer.clear()
        self.is_in_prediction_mode.clear()
        self.self_belief = 1.0
        self.self_belief_log = []
        self.rollback_buffer.clear()
        self.malicious_vehicles.clear()
        self.rollback_stats = {
            "total_rollbacks": 0,
            "vehicles_flagged": [],
            "rollback_times_ns": [],
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
