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
from Observer.TrustbasedDistributedObserver.motor_model import (
    AccelDragMotorModel,
    MotorModelConfig,
)
from Observer.TrustbasedDistributedObserver.external_measurement_cache import (
    ExternalMeasurementCache,
)
from Observer.TrustbasedDistributedObserver.contamination_rollback import (
    ContaminationRollback,
)


class TrustBasedFleetEstimator(FleetStateEstimatorBase):
    """
    Trust-Based Distributed Fleet Estimator

    Extends the base fleet estimator with trust-aware consensus weights.
    Uses the TriP Trust Model for trust evaluation and the Weight Trust Module
    for adaptive weight calculation.

    Algorithm (Correction-then-Prediction):
    For each target vehicle T estimated by host H:
    1. Evaluate trust for all vehicles using TriP model
    2. Calculate adaptive weights based on trust scores
    3. Consensus correction:
       x̂_corrected(T) = x̂_old(T)
                       + w0 * (T's_broadcast - x̂_old(T))
                       + Σ w_N * (Neighbor_N's_estimate(T) - x̂_old(T))
    4. Dynamics prediction:
       x̂_new(T) = f(x̂_corrected(T), u_T, dt)

    Where:
    - w0: Weight for direct measurement (from target)
    - w_N: Trust-weighted weight for neighbor N's estimate
    - f(): Bicycle kinematics + motor model
    - u_T: Target's control input [steering, throttle]
    """

    def __init__(
        self,
        vehicle_id: int,
        fleet_size: int,
        state_dim: int = 5,
        config: Dict = None,
        logger=None,
    ):
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)

        # Parse configuration using from_dict (eliminates ~130 lines of boilerplate)
        trust_config_dict = self.config.get("trust", {})
        weight_config_dict = self.config.get("weight", {})

        self.trust_config = TrustConfig.from_dict(trust_config_dict)
        self.weight_config = WeightConfig.from_dict(weight_config_dict)

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

        # External relative measurements (e.g. YOLO / radar)
        self._ext_cache = ExternalMeasurementCache(
            max_age_s=self.trust_config.max_message_age_s
        )

        # Cache for current weight result
        self.current_weight_result: Optional[WeightResult] = None

        # Generalized trust vector O_i(j)
        self.generalized_trust_vector: Dict[int, float] = {self.vehicle_id: 1.0}

        # Attack mitigation enabled
        self.attack_mitigation_enabled = self.config.get("attack_mitigation", True)
        self.turn_steering_threshold = self.config.get("trust", {}).get(
            "turn_steering_threshold", 0.1
        )

        # Prediction-only mode settings (MATLAB parity)
        self.use_predict_observer = bool(
            self.config.get("use_predict_observer", False)
        )
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
        self.rollback = ContaminationRollback(
            state_dim=self.state_dim,
            vehicle_id=self.vehicle_id,
            fleet_size=self.fleet_size,
            enabled=bool(self.config.get("rollback_enabled", False)),
            window_size=int(self.config.get("rollback_window_size", 15)),
            trust_threshold=self.trust_config.trust_threshold,
        )

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

        # ---- Motor model for dynamics prediction ----
        vehicle_config = self.config.get("vehicle", {})
        accel_lag_cfg = vehicle_config.get("accel_lag_model", {})
        self.accel_lag_enabled = bool(accel_lag_cfg.get("enabled", False))
        self.accel_lag_tau = max(float(accel_lag_cfg.get("tau", 0.318)), 1e-6)
        self.accel_lag_gain = float(accel_lag_cfg.get("input_gain", 1.0))
        motor_cfg_dict = vehicle_config.get("motor_model", {})
        self.motor_model_config = MotorModelConfig.from_dict(motor_cfg_dict)
        self.motor_model = AccelDragMotorModel(self.motor_model_config)
        # Per-vehicle persistent motor acceleration state [m/s^2]
        self._motor_accel_state: Dict[int, float] = {}
        # Per-vehicle cached control inputs from V2V
        self._received_control_inputs: Dict[int, Dict[str, float]] = {}

        if self.logger:
            self.logger.logger.info(
                f"Acceleration lag model {'ENABLED' if self.accel_lag_enabled else 'DISABLED'}"
                f" (tau={self.accel_lag_tau}, gain={self.accel_lag_gain})"
            )
            self.logger.logger.info(
                f"Motor model {'ENABLED' if self.motor_model_config.enabled else 'DISABLED'}"
                f" (tau={self.motor_model_config.tau}, k_th={self.motor_model_config.k_th})"
            )

        # Initialize specialized logger for trusts & weights
        self.trust_weight_logger = TrustWeightLogger(
            output_dir=os.path.dirname(os.path.abspath(__file__)),
            max_vehicles=max(10, fleet_size),
        )
        self.trust_weight_logger.start(vehicle_id)
        self._init_time = time.time()
        self._time_reference: Optional[Dict[str, object]] = None
        self._startup_reference_time_ns: Optional[int] = None

    # ------------------------------------------------------------------
    # Override add_received_local_state to also cache control_input
    # ------------------------------------------------------------------
    def add_received_local_state(
        self, sender_id: int, state, timestamp_ns: int
    ) -> bool:
        """Intercept to cache the sender's control_input before converting to ndarray."""
        if isinstance(state, dict):
            ctrl = state.get("control_input", {})
            if ctrl:
                self._received_control_inputs[sender_id] = {
                    "steering": float(ctrl.get("steering", 0.0)),
                    "throttle": float(ctrl.get("throttle", 0.0)),
                }
        return super().add_received_local_state(sender_id, state, timestamp_ns)

    def set_time_reference(
        self, time_reference: Optional[Dict[str, object]]
    ) -> Optional[Dict[str, object]]:
        """Store shared V2V timing metadata for trust logging/alignment."""
        if not isinstance(time_reference, dict):
            self._time_reference = None
            return None

        raw_reference_ns = time_reference.get(
            "reference_time_ns", time_reference.get("epoch_time_ns")
        )
        try:
            reference_time_ns = (
                int(raw_reference_ns) if raw_reference_ns is not None else None
            )
        except (TypeError, ValueError):
            reference_time_ns = None

        source = str(time_reference.get("source", "local")).strip() or "local"
        normalized: Dict[str, object] = {
            "source": source,
            "reference_time_ns": reference_time_ns,
        }
        for key in ("reference_vehicle_id", "leader_id"):
            if key not in time_reference or time_reference.get(key) is None:
                continue
            try:
                normalized[key] = int(time_reference[key])
            except (TypeError, ValueError):
                continue

        self._time_reference = normalized
        return dict(normalized)

    def _get_log_time_s(self, current_time_ns: int) -> float:
        """Return trust-log time directly in the active V2V time domain."""
        return max(float(current_time_ns), 0.0) / 1e9

    def _get_startup_elapsed_s(self, current_time_ns: int) -> float:
        """Return elapsed observer time since the first update in this run."""
        if self._startup_reference_time_ns is None:
            self._startup_reference_time_ns = int(current_time_ns)
        elapsed_ns = int(current_time_ns) - self._startup_reference_time_ns
        return max(float(elapsed_ns), 0.0) / 1e9

    def _use_startup_fixed_weights(self, current_time_ns: int) -> bool:
        """Whether trust-based weights should be bypassed during startup."""
        duration_s = float(getattr(self.weight_config, "startup_fixed_duration_s", 0.0))
        return (
            duration_s > 0.0
            and self._get_startup_elapsed_s(current_time_ns) < duration_s
        )

    # ------------------------------------------------------------------
    # External relative measurement delegation
    # ------------------------------------------------------------------
    def set_external_relative_measurement(
        self,
        target_id: int,
        distance: float,
        relative_velocity: Optional[float] = None,
        timestamp_ns: Optional[int] = None,
        source: str = "external_sensor",
        measurement_confidence: Optional[float] = None,
    ) -> bool:
        """Store externally measured host-target relative states (e.g., YOLO/radar)."""
        return self._ext_cache.set(
            target_id, distance, relative_velocity,
            timestamp_ns, source, measurement_confidence,
        )

    def clear_external_relative_measurement(self, target_id: Optional[int] = None) -> None:
        """Clear cached external relative measurement(s)."""
        self._ext_cache.clear(target_id)

    # ------------------------------------------------------------------
    # Control input helpers
    # ------------------------------------------------------------------
    def _get_target_control(self, target_id: int, host_control: np.ndarray) -> np.ndarray:
        """Return the target vehicle's latest control input from V2V cache."""
        cached = self._received_control_inputs.get(target_id)
        if cached is not None:
            return np.array([cached["steering"], cached["throttle"]])
        return host_control

    # ==================================================================
    #   MAIN UPDATE
    # ==================================================================
    def update(
        self,
        local_state: np.ndarray,
        dt: float,
        current_time_ns: int,
        control: np.ndarray,
    ) -> np.ndarray:
        """
        Update fleet state estimates using trust-based consensus.

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
                self.generalized_trust_vector = (
                    self.trust_model.compute_generalized_trust_vector(
                        all_vehicle_ids=list(range(self.fleet_size)),
                        direct_neighbor_trust=trust_scores,
                        neighbor_opinions=self.trust_model.neighbor_trust_reports,
                    )
                )
            else:
                self.generalized_trust_vector = {
                    self.vehicle_id: 1.0,
                    **{k: float(v) for k, v in trust_scores.items()},
                }

            # 3. Calculate adaptive weights based on trust/opinion
            weight_source_scores = (
                self.generalized_trust_vector
                if self.weight_config.weight_type == "paper"
                else trust_scores
            )
            if self._use_startup_fixed_weights(current_time_ns):
                weight_source_scores = {
                    vid: 1.0 for vid in weight_source_scores.keys()
                }
            weight_result = self.weight_module.calculate_weights(weight_source_scores)
            self.current_weight_result = weight_result

            # 4. Update estimates for other vehicles
            pre_update_states = self.fleet_states.copy()
            step_targets: Dict[int, Dict] = {}
            confidence_scores: List[float] = []
            target_confidence: Dict[int, float] = {self.vehicle_id: 1.0}
            target_prediction_mode: Dict[int, bool] = {self.vehicle_id: False}

            for target_id in trust_scores.keys():
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
                    # Correction-then-Prediction: propagate the
                    # consensus-corrected state through dynamics
                    # x̂_new = f(x̂_corrected, u, dt)
                    target_ctrl = self._get_target_control(target_id, control)

                    saved_motor = self._motor_accel_state.get(target_id, 0.0)
                    consensus_est = normal_est.copy()
                    normal_est = self._apply_state_constraints(
                        self._predict_dynamics(
                            consensus_est, target_ctrl, dt, target_id=target_id
                        )
                    )
                    components["dynamics_delta"] = normal_est - consensus_est
                    primary_motor = self._motor_accel_state.get(target_id, saved_motor)
                    # Predicted-only from old state for mode switch comparison
                    self._motor_accel_state[target_id] = saved_motor
                    predicted_only_est = self._apply_state_constraints(
                        self._predict_dynamics(
                            current_est, target_ctrl, dt, target_id=target_id
                        )
                    )
                    self._motor_accel_state[target_id] = primary_motor

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
                    target_ctrl = self._get_target_control(target_id, control)
                    predicted_only_est = self._apply_state_constraints(
                        self._predict_dynamics(
                            current_est, target_ctrl, dt, target_id=target_id
                        )
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

            # 5.1 Log trust/weight/estimation state
            self._log_update(
                trust_scores, weight_result, control,
                target_confidence, target_prediction_mode,
                current_time_ns=current_time_ns,
            )

            # 6. Contamination rollback
            if self.rollback.enabled:
                self.rollback.record(
                    current_time_ns=current_time_ns,
                    pre_update_states=pre_update_states,
                    target_components=step_targets,
                )
                self.fleet_states = self.rollback.check_and_trigger(
                    trust_scores=trust_scores,
                    current_time_ns=current_time_ns,
                    fleet_states=self.fleet_states,
                )

            # 7. Cleanup old data
            self._cleanup_old_data(current_time_ns)

            return self.fleet_states.copy()

        except Exception as e:
            if self.logger:
                self.logger.log_error("TrustBasedFleetEstimator update error", e)
            return self.fleet_states.copy()

    # ==================================================================
    #   TRUST SCORE UPDATE
    # ==================================================================
    def _update_trust_scores(self, current_time_ns: int) -> Dict[int, float]:
        """Update trust scores for all known vehicles."""
        trust_scores: Dict[int, float] = {}

        known_vehicle_ids = set()
        known_vehicle_ids.update(self.received_local_states.keys())
        known_vehicle_ids.update(self.trust_model.get_all_trust_scores().keys())
        known_vehicle_ids.discard(self.vehicle_id)

        for vehicle_id in sorted(known_vehicle_ids):
            # Ensure fleet_states can accommodate this vehicle_id
            # (vehicle IDs may be non-contiguous, e.g. [0, 2])
            self._ensure_fleet_capacity(vehicle_id)
            
            latest = self._get_latest_received_state_with_timestamp(
                vehicle_id, current_time_ns
            )

            if latest is None:
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

            # Attach external relative measurement if available
            external_rel = self._ext_cache.get(vehicle_id, current_time_ns)
            if external_rel is not None:
                target_data.distance_from_host = float(
                    external_rel.get("distance", float("nan"))
                )
                target_data.relative_velocity_from_host = float(
                    external_rel.get("relative_velocity", float("nan"))
                )
                target_data.relative_measurement_confidence = float(
                    external_rel.get("confidence", float("nan"))
                )
                target_data.relative_measurement_source = str(
                    external_rel.get("source", "external_sensor")
                )
                target_data.relative_measurement_timestamp_ns = int(
                    external_rel.get("timestamp_ns", 0.0)
                )

            self.trust_model.update_beacon_reception(
                vehicle_id, True, packet_ts_ns / 1e9
            )

            # Gather distributed estimates (merged collector)
            neighbor_estimates = self._collect_neighbor_estimates(
                vehicle_id, current_time_ns, exclude_self=True
            )
            neighbor_host_estimates = self._collect_neighbor_estimates(
                self.vehicle_id, current_time_ns, exclude_self=True
            )
            host_target_estimate = self.fleet_states[:, vehicle_id].copy()

            target_fleet_data = None
            fleet_entry = self._get_latest_fleet_data_with_timestamp(
                vehicle_id, current_time_ns
            )
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

    # ==================================================================
    #   NEIGHBOR ESTIMATE COLLECTION (merged from two methods)
    # ==================================================================
    def _collect_neighbor_estimates(
        self, vehicle_id: int, current_time_ns: int, exclude_self: bool = True
    ) -> Dict[int, VehicleData]:
        """
        Collect neighbors' distributed estimates about a specific vehicle.

        Args:
            vehicle_id: The vehicle whose estimates we want to collect.
            current_time_ns: Current time for age filtering.
            exclude_self: If True, skip the host vehicle as a reporter.
        """
        estimates: Dict[int, VehicleData] = {}
        for neighbor_id in list(self.received_fleet_states.keys()):
            if exclude_self and neighbor_id == self.vehicle_id:
                continue
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

    # ==================================================================
    #   DATA RETRIEVAL HELPERS
    # ==================================================================
    def _get_latest_received_state_with_timestamp(
        self, vehicle_id: int, current_time_ns: int
    ) -> Optional[Tuple[int, np.ndarray]]:
        """Return latest valid (timestamp_ns, state_vec) for a vehicle."""
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
        """Return latest valid (timestamp_ns, fleet_data) from a neighbor."""
        if neighbor_id not in self.received_fleet_states:
            return None
        history = self.received_fleet_states[neighbor_id]
        for ts_ns, fleet_data in reversed(history):
            if (current_time_ns - ts_ns) <= self.max_state_age_ns:
                return ts_ns, fleet_data
        return None

    # ==================================================================
    #   TRUST-WEIGHTED UPDATE (unified paper / non-paper)
    # ==================================================================
    def _trust_weighted_update_with_components(
        self,
        target_id: int,
        current_time_ns: int,
        trust_scores: Dict[int, float],
        control: np.ndarray,
        dt: float,
    ) -> Tuple[np.ndarray, Dict]:
        """Update estimate for a target vehicle and return replayable contribution terms."""
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
            neighbor_fleet = self._get_latest_fleet_data(neighbor_id, current_time_ns)
            if neighbor_fleet is None or target_id not in neighbor_fleet:
                continue
            neighbor_fleet_estimates[neighbor_id] = neighbor_fleet

        use_startup_fixed_weights = self._use_startup_fixed_weights(current_time_ns)

        # Calculate weights (paper or trust-based - unified call)
        if use_startup_fixed_weights:
            startup_trust_scores = {
                vid: 1.0
                for vid in set(trust_scores.keys())
                | set(neighbor_fleet_estimates.keys())
                | {target_id}
            }
            target_weights = self.weight_module.calculate_weights_for_target(
                target_id=target_id,
                trust_scores=startup_trust_scores,
                neighbor_fleet_estimates=neighbor_fleet_estimates,
                direct_measurement=direct_state,
            )
        elif self.weight_config.weight_type == "paper":
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
        else:
            target_weights = self.weight_module.calculate_weights_for_target(
                target_id=target_id,
                trust_scores=trust_scores,
                neighbor_fleet_estimates=neighbor_fleet_estimates,
                direct_measurement=direct_state,
            )

        # === Flag-Driven w₀ Adaptation ===
        trust_obj = self.trust_model.get_trust_score(target_id)
        if trust_obj is not None and not use_startup_fixed_weights:
            target_weights = self.weight_module.apply_flag_adaptation(
                target_weights, trust_obj, self.weight_config
            )

        # === Direct Measurement Correction ===
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

        # === Apply Consensus Correction ===
        # Dynamics propagation f(x̂_corrected, u, dt) is applied in update()
        new_est = self._apply_state_constraints(current_est + total_correction)
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
        """Backward-compatible wrapper returning only updated state."""
        new_est, _ = self._trust_weighted_update_with_components(
            target_id=target_id,
            current_time_ns=current_time_ns,
            trust_scores=trust_scores,
            control=control,
            dt=dt,
        )
        return new_est

    # ==================================================================
    #   DYNAMICS & CONSTRAINTS
    # ==================================================================
    def _predict_dynamics(
        self,
        state: np.ndarray,
        control: np.ndarray,
        dt: float,
        target_id: int = -1,
    ) -> np.ndarray:
        """Predict next state using bicycle kinematics + first-order lag motor model."""
        x, y, theta, v = state[:4]
        a = state[4] if len(state) > 4 else 0.0

        steering = control[0] if len(control) > 0 else 0.0
        throttle = control[1] if len(control) > 1 else 0.0

        vehicle_cfg = self.config.get("vehicle", {})
        L = vehicle_cfg.get("wheelbase", 0.256)

        # Bicycle kinematics
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + (v * np.tan(steering) / L) * dt

        # Velocity / acceleration prediction
        if self.accel_lag_enabled and dt > 0:
            a_new = a + dt * (
                -(1.0 / self.accel_lag_tau) * a
                + (self.accel_lag_gain / self.accel_lag_tau) * throttle
            )
            v_new = v + a_new * dt
        elif self.motor_model_config.enabled and dt > 0:
            # print("Motor model enabled, using dynamic prediction")
            motor_accel = self._motor_accel_state.get(target_id, 0.0)
            v_new, a_new, motor_accel_new = self.motor_model.predict(
                throttle=throttle, v=v, motor_accel=motor_accel, dt=dt,
            )
            self._motor_accel_state[target_id] = motor_accel_new
        else:
            # print("Motor model disabled or dt=0, using simple kinematic prediction")
            a_new = a
            v_new = v + a * dt

        return np.array([x_new, y_new, theta_new, v_new, a_new])

    def _apply_state_constraints(self, state: np.ndarray) -> np.ndarray:
        """Apply physical constraints to state."""
        constrained = state.copy()
        constrained[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))
        constrained[3] = np.clip(state[3], -2.0, 2.0)
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
        """MATLAB-inspired prediction-only switching logic."""
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
            self.predict_only_counter[target_id] = max(self.n_good, 1)
            self.predict_only_timer[target_id] = 0.0
            self.is_in_prediction_mode[target_id] = False
            return normal_est, confidence

        if not in_pred_mode:
            return normal_est, confidence

        # In prediction-only mode: blend or pure prediction
        diff_norm = float(np.linalg.norm(normalized_diff))
        if (not is_ok) and diff_norm < max(self.blend_thresh, 1e-6):
            alpha = min(0.5, diff_norm / max(self.blend_thresh, 1e-6))
            blended = (1.0 - alpha) * normal_est + alpha * predicted_est
            return self._apply_state_constraints(blended), confidence
        return predicted_est, confidence

    # ==================================================================
    #   ATTACK MITIGATION
    # ==================================================================
    def _apply_attack_mitigation(
        self, trust_scores: Dict[int, float], current_time_ns: int
    ):
        """Apply attack mitigation based on trust model flags."""
        attack_flags = self.trust_model.get_attack_flags()
        for vehicle_id, flags in attack_flags.items():
            if flags.get("target_attack", False):
                self.stats["attacks_detected"] += 1
                if vehicle_id in trust_scores:
                    trust_scores[vehicle_id] = float(
                        np.clip(trust_scores[vehicle_id] * 0.5, 0.0, 1.0)
                    )
                    self.stats["mitigations_applied"] += 1

    # ==================================================================
    #   LOGGING (extracted from update())
    # ==================================================================
    def _log_update(
        self,
        trust_scores: Dict[int, float],
        weight_result: WeightResult,
        control: np.ndarray,
        target_confidence: Dict[int, float],
        target_prediction_mode: Dict[int, bool],
        current_time_ns: int,
    ) -> None:
        """Build and emit per-step trust/weight log data."""
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
            "startup_fixed_weights": int(
                self._use_startup_fixed_weights(current_time_ns)
            ),
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
                    "rel_meas_used_global": int(
                        trust_result.relative_measurement_used_global
                    ),
                    "yolo_rel_meas_used_global": int(
                        trust_result.yolo_relative_measurement_used_global
                    ),
                    "rel_dist_meas_used": int(
                        trust_result.relative_distance_measurement_used
                    ),
                    "rel_vel_meas_used": int(
                        trust_result.relative_velocity_measurement_used
                    ),
                    "y_local_distance": trust_result.y_local_distance,
                    "y_true_distance": trust_result.y_true_distance,
                    "yolo_true_rel_dist_error": trust_result.yolo_true_rel_dist_error,
                    "y_local_rel_velocity": trust_result.y_local_rel_velocity,
                    "y_true_rel_velocity": trust_result.y_true_rel_velocity,
                    "yolo_true_rel_vel_error": trust_result.yolo_true_rel_vel_error,
                    "yolo_rel_distance": trust_result.yolo_rel_distance,
                    "yolo_rel_velocity": trust_result.yolo_rel_velocity,
                    "d_host_mean": trust_result.d_host_mean,
                    "d_local_mean": trust_result.d_local_mean,
                    "d_self": trust_result.d_self,
                    "mi_veh_id": trust_result.mi_veh_id,
                    "mi_dist": trust_result.mi_dist,
                    "mi_elem_idx": trust_result.mi_elem_idx,
                    "mi_elem_val": trust_result.mi_elem_val,
                    "v2v_details": trust_result.v2v_details,
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

        # Backward-compat system logger
        if self.logger and hasattr(self.logger, "log_trust_weight"):
            self._log_backward_compat(trust_scores, weight_result)

        # Fleet estimates snapshot
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

        self.trust_weight_logger.record(self._get_log_time_s(current_time_ns), log_data)

    def _log_backward_compat(
        self, trust_scores: Dict[int, float], weight_result: WeightResult
    ) -> None:
        """Backward-compatible logging via the system logger."""
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

            neighbor_weight = weight_result.neighbor_weights.get(vehicle_id, 0.0)
            weight_data = {
                "w0": weight_result.w0,
                "w_self": weight_result.w_self,
                "w_neighbor": neighbor_weight,
            }
            self.logger.log_trust_weight(vehicle_id, trust_data, weight_data)

    # ==================================================================
    #   FLEET CAPACITY
    # ==================================================================
    def _ensure_fleet_capacity(self, min_vehicle_id: int):
        """Ensure fleet capacity and expand weight module if needed."""
        if min_vehicle_id >= self.fleet_states.shape[1]:
            super()._ensure_fleet_capacity(min_vehicle_id)
            self.weight_module.update_fleet_size(self.fleet_size)

    # ==================================================================
    #   PUBLIC API
    # ==================================================================
    def get_trust_score(self, vehicle_id: int) -> Optional[TrustScore]:
        """Get detailed trust score for a vehicle."""
        return self.trust_model.get_trust_score(vehicle_id)

    def get_all_trust_scores(self) -> Dict[int, float]:
        """Get all trust scores as simple dict."""
        return self.trust_model.get_all_trust_scores()

    def get_generalized_trust_vector(self) -> Dict[int, float]:
        """Get generalized trust vector O_i(j)."""
        return self.generalized_trust_vector.copy()

    def get_trusted_vehicles(self, threshold: float = None) -> List[int]:
        """Get list of trusted vehicle IDs."""
        return self.trust_model.get_trusted_vehicles(threshold)

    def is_vehicle_trusted(self, vehicle_id: int) -> bool:
        """Check if a vehicle is currently trusted."""
        return self.trust_model.is_vehicle_trusted(vehicle_id)

    def get_attack_flags(self) -> Dict[int, Dict[str, bool]]:
        """Get attack detection flags for all vehicles."""
        return self.trust_model.get_attack_flags()

    def get_self_belief(self) -> float:
        """Get latest self-belief value derived from prediction-mode confidence."""
        return float(self.self_belief)

    def is_vehicle_in_prediction_mode(self, vehicle_id: int) -> bool:
        """Check whether a specific target is currently in prediction-only mode."""
        return bool(self.is_in_prediction_mode.get(vehicle_id, False))

    def get_current_weights(self) -> np.ndarray:
        """Get current consensus weights."""
        return self.weight_module.get_weights_array()

    def get_statistics(self) -> Dict[str, object]:
        """Get estimator statistics."""
        data = self.stats.copy()
        data["self_belief"] = float(self.self_belief)
        data["rollbacks"] = self.rollback.stats.copy()
        return data

    def add_neighbor_trust_report(
        self, reporter_id: int, target_id: int, trust_score: float
    ):
        """Add trust report from neighbor for cross-validation."""
        self.trust_model.add_neighbor_trust_report(reporter_id, target_id, trust_score)

    def reset(self):
        """Reset estimator including trust and weight modules."""
        super().reset()
        self.trust_model.reset()
        self.weight_module.reset()
        self.host_state = {}
        self._ext_cache.clear()
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
        self.rollback.reset()
        self._init_time = time.time()
        self._startup_reference_time_ns = None

    def __del__(self):
        if hasattr(self, "trust_weight_logger"):
            self.trust_weight_logger.stop()


# ======================================================================
#   Backward-compat re-export + factory
# ======================================================================
from Observer.TrustbasedDistributedObserver.trust_based_kalman_estimator import (  # noqa: E402
    TrustBasedKalmanEstimator,
)


def create_trust_based_estimator(
    estimator_type: str,
    vehicle_id: int,
    fleet_size: int,
    state_dim: int = 5,
    config: Dict = None,
    logger=None,
):
    """
    Factory function to create trust-based estimators.

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
