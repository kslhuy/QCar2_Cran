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
from copy import copy
from typing import Any, Dict, List, Optional, Set, Tuple
from collections import defaultdict, deque

# Import base class and utilities from fleet_state_estimators
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Observer.fleet_state_estimators import (
    FleetStateEstimatorBase,
    _normalize_state_array,
    _state_dict_to_array,
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
from Observer.TrustbasedDistributedObserver.external_measurement_cache import (
    ExternalMeasurementCache,
)
from Observer.TrustbasedDistributedObserver.contamination_rollback import (
    ContaminationRollback,
)
from Observer.TrustbasedDistributedObserver.estimator_config import (
    ObserverSettings,
    PredictionSettings,
    RollbackSettings,
    as_bool,
    as_float,
    dict_section,
    load_vehicle_model_overrides,
    normalize_dynamics_prediction_mode,
    normalize_vehicle_model_config,
    vehicle_model_for_target,
)
from Observer.TrustbasedDistributedObserver.v2v_attack_status import (
    V2VAttackStatusTracker,
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

        trust_config_dict = self._get_config_section("trust")
        weight_config_dict = self._get_config_section("weight")
        vehicle_config = self._get_config_section("vehicle")

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

        self.observer_settings = ObserverSettings.from_config(
            self.config, trust_config_dict
        )
        self.observer_gain = self.observer_settings.observer_gain
        self.consensus_gain = self.observer_settings.consensus_gain
        self.attack_mitigation_enabled = (
            self.observer_settings.attack_mitigation_enabled
        )
        self.turn_steering_threshold = (
            self.observer_settings.turn_steering_threshold
        )

        # Cache for host state (for trust evaluation)
        self.host_state: Dict = {}
        self.controller_debug_snapshot: Dict[str, Any] = {}
        self.received_clean_local_states: Dict[int, List[Tuple[int, np.ndarray]]] = (
            defaultdict(list)
        )

        # External relative measurements (e.g. YOLO / radar)
        self._ext_cache = ExternalMeasurementCache(
            max_age_s=self.trust_config.max_message_age_s
        )

        # Cache for current weight result
        self.current_weight_result: Optional[WeightResult] = None

        # Generalized trust vector O_i(j)
        self.generalized_trust_vector: Dict[int, float] = {self.vehicle_id: 1.0}
        self._direct_recovery_state: Dict[int, Dict[str, float]] = {}
        self._direct_trust_delay_state: Dict[int, Dict[str, float]] = {}
        self._rollback_trigger_delay_state: Dict[int, int] = {}

        # Prediction/output settings
        self._init_prediction_settings(vehicle_config)

        # Contamination rollback (trust-triggered replay)
        self.rollback_settings = RollbackSettings.from_config(self.config)
        self.rollback_trusted_state_guard_steps = (
            self.rollback_settings.trusted_state_guard_steps
        )
        self.rollback_trusted_state_history_size = (
            self.rollback_settings.trusted_state_history_size
        )
        self.rollback_on_final_trust = self.rollback_settings.on_final_trust
        self.rollback_on_local_est_check = self.rollback_settings.on_local_est_check
        self.rollback_on_global_est_check = self.rollback_settings.on_global_est_check
        self.rollback_trigger_delay_steps = self.rollback_settings.trigger_delay_steps
        self.rollback_startup_suppress_duration_s = (
            self.rollback_settings.startup_suppress_duration_s
        )
        self._rollback_trusted_state_history: Dict[int, deque] = {}
        self._rollback_trusted_relative_anchor_history: Dict[int, deque] = {}
        self.rollback = ContaminationRollback(
            state_dim=self.state_dim,
            vehicle_id=self.vehicle_id,
            fleet_size=self.fleet_size,
            enabled=self.rollback_settings.enabled,
            window_size=self.rollback_settings.window_size,
            trust_threshold=self.trust_config.trust_threshold,
            predict_fn=self._predict_dynamics,
            constraints_fn=self._apply_state_constraints,
            trusted_state_fn=self._get_rollback_trusted_state_entry,
            logger=logger,
        )

        # Statistics
        self.stats = self._make_default_stats()

        if self.logger:
            self.logger.logger.info(
                f"TrustBasedFleetEstimator initialized for vehicle_{vehicle_id} "
                f"with fleet_size={fleet_size}, state_dim={state_dim}, "
                f"weight_type={self.weight_config.weight_type}"
            )

        # ---- Per-target vehicle model for dynamics prediction ----
        self._init_vehicle_model_settings(vehicle_config)

        self._log_prediction_settings()

        # Initialize specialized logger for trusts & weights
        logging_config = self._get_config_section("logging")
        trust_recording_overwrite = as_bool(
            logging_config.get(
                "trust_recording_overwrite",
                logging_config.get("recording_overwrite", True),
            ),
            True,
        )
        self.trust_weight_logger = TrustWeightLogger(
            output_dir=os.path.dirname(os.path.abspath(__file__)),
            max_vehicles=max(1, fleet_size),
        )
        filepath = self.trust_weight_logger.start(
            vehicle_id,
            overwrite=trust_recording_overwrite,
        )
        if filepath and self.logger:
            self.logger.logger.info(f"Trust weight recording to {filepath}")
        self._init_runtime_tracking()

    def _get_config_section(self, key: str) -> Dict[str, Any]:
        """Return a config subsection as a plain dict."""
        value = self.config.get(key, {})
        return value if isinstance(value, dict) else {}

    def set_controller_debug_snapshot(
        self, snapshot: Optional[Dict[str, Any]]
    ) -> None:
        """Cache the latest controller command snapshot for trust CSV logging."""
        if not isinstance(snapshot, dict):
            self.controller_debug_snapshot = {}
            return
        self.controller_debug_snapshot = dict(snapshot)

    def _init_prediction_settings(self, vehicle_config: Dict[str, Any]) -> None:
        """Initialize prediction/output configuration."""
        self.prediction_settings = PredictionSettings.from_config(
            self.config,
            vehicle_config,
            trust_threshold=self.trust_config.trust_threshold,
        )
        for name in self.prediction_settings.__dataclass_fields__:
            setattr(self, name, getattr(self.prediction_settings, name))

    def _init_vehicle_model_settings(self, vehicle_config: Dict[str, Any]) -> None:
        """Initialize vehicle-model config used by prediction."""
        self._raw_default_vehicle_config = dict(vehicle_config)
        self.default_vehicle_model = normalize_vehicle_model_config(
            vehicle_config, self.dynamics_prediction_mode
        )
        self.vehicle_model_overrides = load_vehicle_model_overrides(
            self.config.get("vehicle_models", {}),
            self._raw_default_vehicle_config,
            self.dynamics_prediction_mode,
        )
        self.control_timeout_s = as_float(
            self.config.get(
                "control_timeout_s",
                vehicle_config.get("control_timeout_s", 1.0),
            ),
            1.0,
        )
        self.timestamp_alignment_config = self._get_config_section(
            "timestamp_alignment"
        )

        velocity_lag_cfg = dict_section(vehicle_config, "velocity_lag_model")
        self.velocity_lag_enabled = bool(velocity_lag_cfg.get("enabled", False))
        self.velocity_lag_tau = max(float(velocity_lag_cfg.get("tau", 0.301)), 1e-6)
        self.velocity_lag_gain = float(velocity_lag_cfg.get("velocity_gain", 6.598))

        accel_lag_cfg = dict_section(vehicle_config, "accel_lag_model")
        self.accel_lag_enabled = bool(accel_lag_cfg.get("enabled", False))
        self.accel_lag_tau = max(float(accel_lag_cfg.get("tau", 0.318)), 1e-6)
        self.accel_lag_gain = float(accel_lag_cfg.get("input_gain", 1.0))

        # Per-vehicle cached control inputs from V2V. Do not fall back to host
        # control for another target; if target control is absent/stale the
        # prediction model must degrade to constant velocity.
        self._received_control_inputs: Dict[int, Dict[str, float]] = {}

    def _log_prediction_settings(self) -> None:
        """Emit the main prediction-model configuration to the logger."""
        if not self.logger:
            return
        self.logger.logger.info(
            f"Dynamics prediction mode '{self.dynamics_prediction_mode}'"
        )
        self.logger.logger.info(
            f"Velocity lag model {'ENABLED' if self.velocity_lag_enabled else 'DISABLED'}"
            f" (tau={self.velocity_lag_tau}, K={self.velocity_lag_gain})"
        )
        self.logger.logger.info(
            f"Acceleration lag model {'ENABLED' if self.accel_lag_enabled else 'DISABLED'}"
            f" (tau={self.accel_lag_tau}, gain={self.accel_lag_gain})"
        )
        self.logger.logger.info(
            "Output low-pass alpha=%s, attack alpha=%s",
            self.output_low_pass_alpha,
            self.attack_output_low_pass_alpha,
        )
        self.logger.logger.info(
            "Force clean pose anchor=%s, post-rollback anchor=%s",
            self.force_clean_pose_anchor,
            self.post_rollback_anchor_enabled,
        )
        self.logger.logger.info(
            "Relative host-anchor attack blend: position(anchor=%s, estimate=%s), theta(clean=%s, host=%s), velocity(target=%s, host=%s), acceleration(target=%s, host=%s)",
            self.relative_host_anchor_anchor_position_weight,
            self.relative_host_anchor_estimate_position_weight,
            self.relative_host_anchor_clean_theta_weight,
            self.relative_host_anchor_host_theta_weight,
            self.relative_host_anchor_target_velocity_weight,
            self.relative_host_anchor_host_velocity_weight,
            self.relative_host_anchor_target_acceleration_weight,
            self.relative_host_anchor_host_acceleration_weight,
        )
        self.logger.logger.info(
            "Relative host-anchor 2D bearing=%s; direct recovery enabled=%s hold=%s good=%s ramp=%s min_local=%s",
            self.relative_host_anchor_use_bearing,
            self.direct_recovery_enabled,
            self.direct_recovery_hold_steps,
            self.direct_recovery_required_good_steps,
            self.direct_recovery_ramp_steps,
            self.direct_recovery_min_local_trust,
        )
        self.logger.logger.info(
            "Test delays: direct_trust_application_delay_steps=%s, rollback_trigger_delay_steps=%s, rollback_startup_suppress_duration_s=%s",
            self.direct_trust_application_delay_steps,
            self.rollback_trigger_delay_steps,
            self.rollback_startup_suppress_duration_s,
        )

    @staticmethod
    def _make_default_stats() -> Dict[str, int]:
        """Return the default estimator statistics payload."""
        return {
            "trust_updates": 0,
            "weight_updates": 0,
            "attacks_detected": 0,
            "mitigations_applied": 0,
        }

    def _reset_startup_weight_tracking(self) -> None:
        """Reset startup warmup timing and cached warmup weights."""
        self._init_time = time.time()
        self._startup_reference_time_ns = None
        self._startup_weight_result_cache = None
        self._startup_target_weights_cache = {}

    def _reset_v2v_attack_tracking(self) -> None:
        """Reset V2V-attack metadata mirrored into trust logs."""
        if not hasattr(self, "_v2v_attack_tracker"):
            self._v2v_attack_tracker = V2VAttackStatusTracker()
        else:
            self._v2v_attack_tracker.reset()

    def _init_runtime_tracking(self) -> None:
        """Initialize transient runtime caches and tracking state."""
        self._time_reference = None
        self._reset_startup_weight_tracking()
        self._reset_v2v_attack_tracking()

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
                    "timestamp_ns": float(timestamp_ns),
                }
        return super().add_received_local_state(sender_id, state, timestamp_ns)

    def add_received_clean_local_state(
        self, sender_id: int, state, timestamp_ns: int
    ) -> bool:
        """
        Store clean V2V local state for trust-only relative checks.

        This channel must not alter the attacked/control-path estimate updates.
        """
        if sender_id == self.vehicle_id:
            return False

        if isinstance(state, dict):
            state_vec = _state_dict_to_array(state, self.state_dim, logger=self.logger)
        else:
            state_vec = _normalize_state_array(state, self.state_dim, logger=self.logger)

        if state_vec is None:
            return False

        self.received_clean_local_states[int(sender_id)].append(
            (int(timestamp_ns), state_vec.copy())
        )
        if len(self.received_clean_local_states[int(sender_id)]) > 10:
            self.received_clean_local_states[int(sender_id)] = (
                self.received_clean_local_states[int(sender_id)][-10:]
            )
        return True

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

    def _suppress_rollback_triggers_during_startup(
        self, current_time_ns: Optional[int]
    ) -> bool:
        """Whether rollback trigger reasons should be ignored during startup."""
        if current_time_ns is None:
            return False
        duration_s = float(
            max(getattr(self, "rollback_startup_suppress_duration_s", 0.0), 0.0)
        )
        return (
            duration_s > 0.0
            and self._get_startup_elapsed_s(current_time_ns) < duration_s
        )

    @staticmethod
    def _copy_target_weights(weights: Dict[str, Any]) -> Dict[str, Any]:
        """Copy cached target weights so per-step logging cannot mutate them."""
        return {
            "w0": float(weights.get("w0", 0.0)),
            "w_self": float(weights.get("w_self", 1.0)),
            "neighbors": {
                int(neighbor_id): float(weight)
                for neighbor_id, weight in weights.get("neighbors", {}).items()
            },
        }

    def _get_startup_weight_result(self, trust_scores: Dict[int, float]) -> WeightResult:
        """Build the startup summary weights once and keep them stable."""
        if self._startup_weight_result_cache is None:
            neighbor_ids = sorted(
                int(vehicle_id)
                for vehicle_id in trust_scores.keys()
                if int(vehicle_id) != self.vehicle_id
            )
            self._startup_weight_result_cache = self.weight_module.calculate_startup_weights(
                neighbor_ids=neighbor_ids
            )
        return self._startup_weight_result_cache

    def _get_startup_target_weights(
        self,
        target_id: int,
        neighbor_fleet_estimates: Dict[int, Dict],
        direct_state: Optional[np.ndarray],
    ) -> Dict[str, Any]:
        """Cache fixed warmup weights per target for the full startup window."""
        cached = self._startup_target_weights_cache.get(int(target_id))
        if cached is None:
            cached = self.weight_module.calculate_startup_weights_for_target(
                target_id=int(target_id),
                neighbor_fleet_estimates=neighbor_fleet_estimates,
                direct_measurement=direct_state,
            )
            cached = self._copy_target_weights(cached)
            self._startup_target_weights_cache[int(target_id)] = cached
        return self._copy_target_weights(cached)

    def _is_local_channel_untrusted(self, target_id: int, trust_val: float) -> bool:
        """Return true only when low trust is evidence against the direct/local channel."""
        trust_obj = self.trust_model.get_trust_score(int(target_id))
        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
        if trust_obj is None:
            return float(trust_val) < threshold

        local_trust = getattr(trust_obj, "local_trust_sample", None)
        try:
            local_trust_f = float(local_trust)
        except (TypeError, ValueError):
            local_trust_f = float("nan")

        return bool(
            getattr(trust_obj, "flag_local_est_check", False)
            or getattr(trust_obj, "flag_target_attack", False)
            or (np.isfinite(local_trust_f) and local_trust_f < threshold)
        )

    def _get_current_malicious_vehicle_ids(
        self, trust_scores: Dict[int, float]
    ) -> Set[int]:
        """Return external vehicles currently quarantined from fleet-source use."""
        malicious_ids = {
            int(vehicle_id)
            for vehicle_id, trust_val in trust_scores.items()
            if int(vehicle_id) != self.vehicle_id
            and self._is_local_channel_untrusted(int(vehicle_id), float(trust_val))
        }

        rollback = getattr(self, "rollback", None)
        if rollback is not None and getattr(rollback, "enabled", False):
            malicious_ids.update(
                int(vehicle_id)
                for vehicle_id in getattr(rollback, "malicious_vehicles", set())
                if int(vehicle_id) != self.vehicle_id
            )

        return malicious_ids

    @staticmethod
    def _has_active_attack_flags(trust_obj) -> bool:
        """True when direct/local-channel evidence indicates an active attack."""
        if trust_obj is None:
            return False
        return bool(
            getattr(trust_obj, "flag_target_attack", False)
            or getattr(trust_obj, "flag_local_est_check", False)
        )

    def _apply_generalized_trust_attack_flags(self) -> None:
        """OR low generalized trust into each target's attack flag."""
        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))

        for raw_target_id, gtrust_value in self.generalized_trust_vector.items():
            target_id = int(raw_target_id)
            if target_id == self.vehicle_id:
                continue

            trust_obj = self.trust_model.get_trust_score(target_id)
            if trust_obj is None:
                continue

            gtrust = self._safe_float_or_nan(gtrust_value)
            if np.isfinite(gtrust) and gtrust < threshold:
                trust_obj.flag_target_attack = True

    @staticmethod
    def _safe_float_or_nan(value: Any) -> float:
        """Convert scalar-like values to float, returning NaN on failure."""
        try:
            value_f = float(value)
        except (TypeError, ValueError):
            return float("nan")
        return value_f if np.isfinite(value_f) else float("nan")

    def _update_direct_trust_delay_states(
        self, trust_scores: Dict[int, float]
    ) -> None:
        """Track test-only delay of local/direct trust application to w0."""
        delay_steps = int(
            max(getattr(self, "direct_trust_application_delay_steps", 0), 0)
        )
        if delay_steps <= 0:
            self._direct_trust_delay_state.clear()
            return

        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
        active_targets = set()

        for raw_target_id, trust_val in trust_scores.items():
            target_id = int(raw_target_id)
            if target_id == self.vehicle_id:
                continue
            active_targets.add(target_id)

            trust_obj = self.trust_model.get_trust_score(target_id)
            direct_bad = self._is_local_channel_untrusted(
                target_id, float(trust_val)
            )
            state = dict(self._direct_trust_delay_state.get(target_id, {}))

            if not direct_bad:
                local_trust = (
                    self._safe_float_or_nan(
                        getattr(trust_obj, "local_trust_sample", float("nan"))
                    )
                    if trust_obj is not None
                    else float("nan")
                )
                final_trust = self._safe_float_or_nan(trust_val)
                if not np.isfinite(local_trust):
                    local_trust = final_trust
                if not np.isfinite(local_trust):
                    local_trust = 1.0
                state = {
                    "bad_count": 0.0,
                    "active": 0.0,
                    "delay_steps": float(delay_steps),
                    "clean_local_trust": float(max(local_trust, threshold)),
                    "clean_final_trust": float(max(final_trust, threshold))
                    if np.isfinite(final_trust)
                    else float(max(local_trust, threshold)),
                }
            else:
                bad_count = int(state.get("bad_count", 0.0)) + 1
                if "clean_local_trust" not in state:
                    state["clean_local_trust"] = 1.0
                if "clean_final_trust" not in state:
                    state["clean_final_trust"] = 1.0
                state["bad_count"] = float(bad_count)
                state["delay_steps"] = float(delay_steps)
                state["active"] = 1.0 if bad_count <= delay_steps else 0.0

            self._direct_trust_delay_state[target_id] = state

        for target_id in list(self._direct_trust_delay_state.keys()):
            if target_id not in active_targets:
                del self._direct_trust_delay_state[target_id]

    def _is_direct_trust_delay_active(self, target_id: int) -> bool:
        """Whether direct/local trust rejection is being artificially delayed."""
        state = self._direct_trust_delay_state.get(int(target_id), {})
        return bool(float(state.get("active", 0.0)) >= 0.5)

    def _effective_direct_trust_obj(self, target_id: int, trust_obj):
        """Return a proxy trust object while test-only direct-delay is active."""
        if trust_obj is None or not self._is_direct_trust_delay_active(target_id):
            return trust_obj

        state = self._direct_trust_delay_state.get(int(target_id), {})
        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
        local_trust = self._safe_float_or_nan(
            state.get("clean_local_trust", float("nan"))
        )
        final_trust = self._safe_float_or_nan(
            state.get("clean_final_trust", float("nan"))
        )
        if not np.isfinite(local_trust):
            local_trust = 1.0
        if not np.isfinite(final_trust):
            final_trust = local_trust

        delayed = copy(trust_obj)
        delayed.local_trust_sample = float(max(local_trust, threshold))
        delayed.final_score = float(max(final_trust, threshold))
        delayed.flag_local_est_check = False
        delayed.flag_target_attack = False
        return delayed

    def _direct_recovery_scale(self, target_id: int) -> float:
        """Return the current direct-channel recovery gain for a target."""
        if not getattr(self, "direct_recovery_enabled", True):
            return 1.0
        state = self._direct_recovery_state.get(int(target_id))
        if not state:
            return 1.0
        return float(np.clip(state.get("scale", 1.0), 0.0, 1.0))

    def _local_trust_ready_for_direct_recovery(self, trust_obj) -> bool:
        """Whether local-channel trust is strong enough to begin reopening w0."""
        if trust_obj is None:
            return True
        local_trust = getattr(trust_obj, "local_trust_sample", None)
        if local_trust is None:
            return True
        try:
            local_trust_f = float(local_trust)
        except (TypeError, ValueError):
            return False
        return bool(
            np.isfinite(local_trust_f)
            and local_trust_f >= self.direct_recovery_min_local_trust
        )

    def _update_direct_channel_recovery_states(
        self, trust_scores: Dict[int, float]
    ) -> None:
        """Latch local-channel attack recovery and ramp direct-measurement gain."""
        if not getattr(self, "direct_recovery_enabled", True):
            self._direct_recovery_state.clear()
            return

        good_required = int(self.direct_recovery_required_good_steps)
        ramp_steps = int(self.direct_recovery_ramp_steps)

        for raw_target_id in trust_scores.keys():
            target_id = int(raw_target_id)
            if target_id == self.vehicle_id:
                continue

            trust_obj = self.trust_model.get_trust_score(target_id)
            local_bad = self._has_active_attack_flags(trust_obj)
            quarantined = self._is_target_quarantined_by_rollback(target_id)
            local_ready = self._local_trust_ready_for_direct_recovery(trust_obj)
            if self._is_direct_trust_delay_active(target_id):
                local_bad = False
                local_ready = True
            state = self._direct_recovery_state.get(target_id)

            if state is None and not (local_bad or quarantined) and local_ready:
                self._direct_recovery_state[target_id] = {
                    "hold": 0.0,
                    "good": float(good_required),
                    "ramp": float(ramp_steps),
                    "scale": 1.0,
                }
                continue

            if state is None:
                state = {"hold": 0.0, "good": 0.0, "ramp": 0.0, "scale": 1.0}

            if local_bad or quarantined or not local_ready:
                state["hold"] = float(self.direct_recovery_hold_steps)
                state["good"] = 0.0
                state["ramp"] = 0.0
                state["scale"] = 0.0
                self._direct_recovery_state[target_id] = state
                continue

            hold = max(int(state.get("hold", 0.0)), 0)
            if hold > 0:
                state["hold"] = float(hold - 1)
                state["good"] = 0.0
                state["ramp"] = 0.0
                state["scale"] = 0.0
                self._direct_recovery_state[target_id] = state
                continue

            good = min(int(state.get("good", 0.0)) + 1, good_required)
            state["good"] = float(good)
            if good < good_required:
                state["ramp"] = 0.0
                state["scale"] = 0.0
                self._direct_recovery_state[target_id] = state
                continue

            if ramp_steps <= 0:
                state["ramp"] = 0.0
                state["scale"] = 1.0
            else:
                ramp = min(int(state.get("ramp", 0.0)) + 1, ramp_steps)
                state["ramp"] = float(ramp)
                state["scale"] = float(np.clip(ramp / float(ramp_steps), 0.0, 1.0))
            self._direct_recovery_state[target_id] = state

    def _apply_direct_recovery_weight_scale(
        self, target_id: int, weights: Dict[str, object]
    ) -> Dict[str, object]:
        """Scale w0 during direct-channel recovery and move freed gain to self."""
        scale = self._direct_recovery_scale(target_id)
        if scale >= 1.0 - 1e-12:
            return weights

        scaled = {
            "w0": max(0.0, float(weights.get("w0", 0.0))) * scale,
            "neighbors": {
                int(neighbor_id): max(0.0, float(weight))
                for neighbor_id, weight in weights.get("neighbors", {}).items()
            },
            "w_self": max(0.0, float(weights.get("w_self", 0.0))),
        }
        old_w0 = max(0.0, float(weights.get("w0", 0.0)))
        scaled["w_self"] += max(0.0, old_w0 - scaled["w0"])

        used = scaled["w0"] + scaled["w_self"] + sum(scaled["neighbors"].values())
        if abs(1.0 - used) > 1e-12:
            scaled["w_self"] = max(0.0, scaled["w_self"] + (1.0 - used))
        return scaled

    def _is_target_quarantined_by_rollback(self, target_id: int) -> bool:
        """True when rollback evidence says this target's direct channel is bad."""
        rollback = getattr(self, "rollback", None)
        if (
            rollback is None
            or not getattr(rollback, "enabled", False)
            or int(target_id) not in getattr(rollback, "malicious_vehicles", set())
        ):
            return False

        event = getattr(rollback, "last_event", {}) or {}
        active_reasons = event.get("active_trigger_reasons", {}) or {}
        reasons = active_reasons.get(int(target_id), [])
        if not reasons:
            return True

        return any(reason in ("final_trust", "local_est_check") for reason in reasons)

    def _is_direct_measurement_allowed(
        self, target_id: int, trust_scores: Dict[int, float]
    ) -> bool:
        """
        Decide whether the direct owner-target channel may be used.

        This gate is intentionally local-channel driven. A low combined/final
        trust score can persist after a fleet inconsistency or mitigation event,
        but that should not keep `w0` at zero once the target's local trust has
        recovered. Rollback only blocks the direct packet when its trigger
        reason points at the target's direct/local channel.
        """
        if self._is_target_quarantined_by_rollback(target_id):
            return False
        if self._is_direct_trust_delay_active(target_id):
            return True
        if self._direct_recovery_scale(target_id) <= 1e-9:
            return False

        trust_obj = self.trust_model.get_trust_score(int(target_id))
        if trust_obj is None:
            threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
            return float(trust_scores.get(int(target_id), 1.0)) >= threshold

        if bool(getattr(trust_obj, "flag_local_est_check", False)):
            return False

        local_trust = getattr(trust_obj, "local_trust_sample", None)
        if local_trust is None:
            return True

        try:
            local_trust_f = float(local_trust)
        except (TypeError, ValueError):
            return True
        if not np.isfinite(local_trust_f):
            return True

        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
        return local_trust_f >= threshold

    def _get_rollback_trusted_state_entry(
        self, target_id: int
    ) -> Optional[Tuple[np.ndarray, Optional[int]]]:
        """Return a guarded trusted snapshot for rollback seeding."""
        history = self._rollback_trusted_state_history.get(int(target_id))
        if not history:
            return None

        guard_steps = min(self.rollback_trusted_state_guard_steps, len(history) - 1)
        trusted_state, trusted_time_ns = history[-1 - guard_steps]
        return (
            np.asarray(trusted_state, dtype=float).copy(),
            None if trusted_time_ns is None else int(trusted_time_ns),
        )

    @staticmethod
    def _body_relative_from_world_delta(
        dx_world: float, dy_world: float, host_theta: float
    ) -> Tuple[float, float]:
        """Convert a world-frame host-target delta into the host body frame."""
        cos_h = float(np.cos(host_theta))
        sin_h = float(np.sin(host_theta))
        rel_x = cos_h * dx_world + sin_h * dy_world
        rel_y = -sin_h * dx_world + cos_h * dy_world
        return float(rel_x), float(rel_y)

    def _copy_host_anchor_snapshot(
        self, snapshot: Optional[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        """Normalize a replayable host-anchor snapshot."""
        if not isinstance(snapshot, dict):
            return None

        copied: Dict[str, Any] = {}
        for key in (
            "host_x",
            "host_y",
            "host_theta",
            "host_velocity",
            "host_acceleration",
            "distance",
            "sign",
            "relative_velocity",
            "bearing",
            "relative_x",
            "relative_y",
        ):
            if key not in snapshot or snapshot.get(key) is None:
                continue
            try:
                copied[key] = float(snapshot[key])
            except (TypeError, ValueError):
                continue

        if "timestamp_ns" in snapshot and snapshot.get("timestamp_ns") is not None:
            try:
                copied["timestamp_ns"] = int(snapshot["timestamp_ns"])
            except (TypeError, ValueError):
                pass

        source = snapshot.get("source")
        if source is not None:
            copied["source"] = str(source)

        if "distance" not in copied or not np.isfinite(float(copied["distance"])):
            return None

        copied["distance"] = max(float(copied["distance"]), 0.1)
        copied["sign"] = 1.0 if float(copied.get("sign", 1.0)) >= 0.0 else -1.0
        if "bearing" in copied:
            copied["bearing"] = self._wrap_angle(float(copied["bearing"]))
        if "relative_x" not in copied and "bearing" in copied:
            copied["relative_x"] = copied["distance"] * float(
                np.cos(copied["bearing"])
            )
        if "relative_y" not in copied and "bearing" in copied:
            copied["relative_y"] = copied["distance"] * float(
                np.sin(copied["bearing"])
            )
        return copied

    def _resolve_relative_host_anchor_sign(
        self,
        target_id: int,
        host_x: float,
        host_y: float,
        host_theta: float,
        reference_state: Optional[np.ndarray] = None,
        clean_state: Optional[np.ndarray] = None,
    ) -> float:
        """Estimate whether the target lies ahead of or behind the host."""
        heading = np.array([np.cos(host_theta), np.sin(host_theta)], dtype=float)
        tol = 1e-6

        for candidate in (reference_state, clean_state):
            if candidate is None:
                continue
            candidate = np.asarray(candidate, dtype=float)
            if candidate.shape[0] < 2:
                continue
            rel = np.array(
                [float(candidate[0]) - host_x, float(candidate[1]) - host_y], dtype=float
            )
            proj = float(np.dot(rel, heading))
            if abs(proj) > tol:
                return 1.0 if proj >= 0.0 else -1.0

        leader_id = None
        if isinstance(getattr(self, "_time_reference", None), dict):
            leader_id = self._time_reference.get("leader_id")
        try:
            leader_id = int(leader_id) if leader_id is not None else None
        except (TypeError, ValueError):
            leader_id = None

        if leader_id is not None:
            if int(target_id) == leader_id and int(target_id) != self.vehicle_id:
                return 1.0
            if self.vehicle_id == leader_id and int(target_id) != self.vehicle_id:
                return -1.0

        return 1.0 if int(target_id) < self.vehicle_id else -1.0

    def _build_relative_host_anchor_entry(
        self,
        target_id: int,
        current_time_ns: int,
        reference_state: Optional[np.ndarray] = None,
        clean_state: Optional[np.ndarray] = None,
    ) -> Optional[Dict[str, Any]]:
        """Build a 2D host-relative anchor from clean/relative/fleet geometry."""
        if not self.host_state:
            return None

        host_x = float(self.host_state.get("x", 0.0))
        host_y = float(self.host_state.get("y", 0.0))
        host_theta = float(self.host_state.get("theta", 0.0))
        host_velocity = float(self.host_state.get("velocity", 0.0))
        host_acceleration = float(self.host_state.get("acceleration", 0.0))

        distance = float("nan")
        relative_velocity = float("nan")
        bearing = float("nan")
        relative_x = float("nan")
        relative_y = float("nan")
        timestamp_ns = int(current_time_ns)
        source = ""

        if clean_state is None:
            clean_state = self._get_latest_clean_aligned_state(
                target_id=target_id,
                current_time_ns=current_time_ns,
            )

        if clean_state is not None:
            clean_arr = np.asarray(clean_state, dtype=float).reshape(-1)
            if clean_arr.size >= 2:
                dx_world = float(clean_arr[0]) - host_x
                dy_world = float(clean_arr[1]) - host_y
                clean_distance = float(np.hypot(dx_world, dy_world))
                if np.isfinite(clean_distance) and clean_distance > 0.0:
                    distance = clean_distance
                    relative_x, relative_y = self._body_relative_from_world_delta(
                        dx_world=dx_world,
                        dy_world=dy_world,
                        host_theta=host_theta,
                    )
                    bearing = self._wrap_angle(float(np.arctan2(relative_y, relative_x)))
                    timestamp_ns = int(current_time_ns)
                    source = "clean_geometry"

        remembered = getattr(self.trust_model, "recent_relative_measurements", {}).get(
            int(target_id)
        )
        if not (np.isfinite(distance) and distance > 0.0) and isinstance(
            remembered, dict
        ):
            try:
                remembered_distance = float(remembered.get("distance", float("nan")))
            except (TypeError, ValueError):
                remembered_distance = float("nan")
            if np.isfinite(remembered_distance) and remembered_distance > 0.0:
                distance = remembered_distance
                try:
                    relative_velocity = float(
                        remembered.get("relative_velocity", float("nan"))
                    )
                except (TypeError, ValueError):
                    relative_velocity = float("nan")
                try:
                    timestamp_ns = int(
                        remembered.get("timestamp_ns", current_time_ns) or current_time_ns
                    )
                except (TypeError, ValueError):
                    timestamp_ns = int(current_time_ns)
                source = str(remembered.get("source", "relative_measurement_memory"))
                age_s = max((float(current_time_ns) - float(timestamp_ns)) / 1e9, 0.0)
                if (
                    np.isfinite(relative_velocity)
                    and age_s <= float(getattr(self.trust_config, "max_message_age_s", 0.5))
                ):
                    distance += relative_velocity * age_s
                try:
                    remembered_bearing = float(remembered.get("bearing", float("nan")))
                except (TypeError, ValueError):
                    remembered_bearing = float("nan")
                if (
                    np.isfinite(remembered_bearing)
                    and getattr(self, "relative_host_anchor_use_bearing", True)
                ):
                    bearing = self._wrap_angle(remembered_bearing)
                    relative_x = float(distance) * float(np.cos(bearing))
                    relative_y = float(distance) * float(np.sin(bearing))

        if not (np.isfinite(distance) and distance > 0.0):
            target_state = reference_state
            if target_state is None and int(target_id) < self.fleet_states.shape[1]:
                target_state = self.fleet_states[:, int(target_id)]
            if target_state is not None:
                target_state = np.asarray(target_state, dtype=float)
                if target_state.shape[0] >= 2:
                    dx = float(target_state[0]) - host_x
                    dy = float(target_state[1]) - host_y
                    geometric_distance = float(np.hypot(dx, dy))
                    if np.isfinite(geometric_distance) and geometric_distance > 0.0:
                        distance = geometric_distance
                        relative_x, relative_y = self._body_relative_from_world_delta(
                            dx_world=dx,
                            dy_world=dy,
                            host_theta=host_theta,
                        )
                        bearing = self._wrap_angle(
                            float(np.arctan2(relative_y, relative_x))
                        )
                        timestamp_ns = int(current_time_ns)
                        source = "fleet_geometry"

        if not (np.isfinite(distance) and distance > 0.0):
            return None

        sign = self._resolve_relative_host_anchor_sign(
            target_id=target_id,
            host_x=host_x,
            host_y=host_y,
            host_theta=host_theta,
            reference_state=reference_state,
            clean_state=clean_state,
        )
        if np.isfinite(relative_x) and abs(relative_x) > 1e-6:
            sign = 1.0 if relative_x >= 0.0 else -1.0

        snapshot = {
            "host_x": host_x,
            "host_y": host_y,
            "host_theta": host_theta,
            "host_velocity": host_velocity,
            "host_acceleration": host_acceleration,
            "distance": distance,
            "sign": sign,
            "relative_velocity": relative_velocity,
            "timestamp_ns": timestamp_ns,
            "source": source or "relative_host_anchor",
        }
        if np.isfinite(bearing) and getattr(self, "relative_host_anchor_use_bearing", True):
            snapshot["bearing"] = bearing
        if np.isfinite(relative_x) and np.isfinite(relative_y):
            snapshot["relative_x"] = relative_x
            snapshot["relative_y"] = relative_y
        return self._copy_host_anchor_snapshot(snapshot)

    def _get_latest_trusted_relative_anchor_entry(
        self, target_id: int, current_time_ns: Optional[int] = None
    ) -> Optional[Dict[str, Any]]:
        """Return the latest trusted host-relative anchor, optionally time-aligned."""
        history = self._rollback_trusted_relative_anchor_history.get(int(target_id))
        if not history:
            return None

        snapshot = self._copy_host_anchor_snapshot(history[-1])
        if snapshot is None:
            return None

        if current_time_ns is None:
            return snapshot

        timestamp_ns = int(snapshot.get("timestamp_ns", 0) or 0)
        relative_velocity = float(snapshot.get("relative_velocity", float("nan")))
        if timestamp_ns > 0 and np.isfinite(relative_velocity):
            age_s = max((float(current_time_ns) - float(timestamp_ns)) / 1e9, 0.0)
            if age_s <= float(getattr(self.trust_config, "max_message_age_s", 0.5)):
                snapshot["distance"] = max(
                    float(snapshot["distance"]) + relative_velocity * age_s, 0.1
                )
                bearing = float(snapshot.get("bearing", float("nan")))
                if np.isfinite(bearing):
                    snapshot["relative_x"] = float(snapshot["distance"]) * float(
                        np.cos(bearing)
                    )
                    snapshot["relative_y"] = float(snapshot["distance"]) * float(
                        np.sin(bearing)
                    )
        return snapshot

    def _build_relative_host_anchor_snapshot(
        self,
        target_id: int,
        current_time_ns: int,
        reference_state: np.ndarray,
        clean_state: Optional[np.ndarray] = None,
    ) -> Optional[Dict[str, Any]]:
        """Resolve the live host-relative anchor snapshot for prediction/replay."""
        if clean_state is None:
            clean_state = self._get_latest_clean_aligned_state(
                target_id=target_id,
                current_time_ns=current_time_ns,
            )

        snapshot = None
        if clean_state is not None:
            snapshot = self._build_relative_host_anchor_entry(
                target_id=target_id,
                current_time_ns=current_time_ns,
                reference_state=reference_state,
                clean_state=clean_state,
            )
        if snapshot is None:
            snapshot = self._get_latest_trusted_relative_anchor_entry(
                target_id, current_time_ns=current_time_ns
            )
        if snapshot is None:
            snapshot = self._build_relative_host_anchor_entry(
                target_id=target_id,
                current_time_ns=current_time_ns,
                reference_state=reference_state,
                clean_state=clean_state,
            )
        if snapshot is None:
            return None

        if self.host_state:
            snapshot["host_x"] = float(
                self.host_state.get("x", snapshot.get("host_x", 0.0))
            )
            snapshot["host_y"] = float(
                self.host_state.get("y", snapshot.get("host_y", 0.0))
            )
            snapshot["host_theta"] = float(
                self.host_state.get("theta", snapshot.get("host_theta", 0.0))
            )
            snapshot["host_velocity"] = float(
                self.host_state.get("velocity", snapshot.get("host_velocity", 0.0))
            )
            snapshot["host_acceleration"] = float(
                self.host_state.get(
                    "acceleration", snapshot.get("host_acceleration", 0.0)
                )
            )

        snapshot["sign"] = self._resolve_relative_host_anchor_sign(
            target_id=target_id,
            host_x=float(snapshot.get("host_x", 0.0)),
            host_y=float(snapshot.get("host_y", 0.0)),
            host_theta=float(snapshot.get("host_theta", 0.0)),
            reference_state=reference_state,
            clean_state=clean_state,
        )
        return self._copy_host_anchor_snapshot(snapshot)

    def _update_rollback_trusted_state_history(
        self, trust_scores: Dict[int, float], current_time_ns: int
    ) -> None:
        """Cache trusted post-update states and host-relative anchors for replay."""
        if not self.rollback.enabled:
            return

        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
        for vehicle_id, trust_val in trust_scores.items():
            target_id = int(vehicle_id)
            if target_id == self.vehicle_id or float(trust_val) < threshold:
                continue
            if target_id >= self.fleet_states.shape[1]:
                continue

            history = self._rollback_trusted_state_history.get(target_id)
            if history is None or history.maxlen != self.rollback_trusted_state_history_size:
                history = deque(
                    [] if history is None else list(history),
                    maxlen=self.rollback_trusted_state_history_size,
                )
                self._rollback_trusted_state_history[target_id] = history

            history.append(
                (
                    np.asarray(self.fleet_states[:, target_id], dtype=float).copy(),
                    int(current_time_ns),
                )
            )

            anchor_history = self._rollback_trusted_relative_anchor_history.get(target_id)
            if (
                anchor_history is None
                or anchor_history.maxlen != self.rollback_trusted_state_history_size
            ):
                anchor_history = deque(
                    [] if anchor_history is None else list(anchor_history),
                    maxlen=self.rollback_trusted_state_history_size,
                )
                self._rollback_trusted_relative_anchor_history[target_id] = anchor_history

            anchor_entry = self._build_relative_host_anchor_entry(
                target_id=target_id,
                current_time_ns=current_time_ns,
                reference_state=self.fleet_states[:, target_id],
            )
            if anchor_entry is not None:
                anchor_history.append(anchor_entry)

    def _apply_rollback_trigger_delay(
        self, target_id: int, signal: Dict[str, object]
    ) -> Dict[str, object]:
        """Delay rollback trigger reasons for controlled rollback experiments."""
        delay_steps = int(max(getattr(self, "rollback_trigger_delay_steps", 0), 0))
        if delay_steps <= 0:
            self._rollback_trigger_delay_state.pop(int(target_id), None)
            return signal

        reason_keys = (
            "trust_below_threshold",
            "flag_local_est_check",
            "flag_global_est_check",
        )
        reason_active = any(bool(signal.get(key, False)) for key in reason_keys)
        if not reason_active:
            self._rollback_trigger_delay_state[int(target_id)] = 0
            signal["rollback_trigger_delay_count"] = 0
            signal["rollback_trigger_delay_steps"] = delay_steps
            return signal

        count = self._rollback_trigger_delay_state.get(int(target_id), 0) + 1
        self._rollback_trigger_delay_state[int(target_id)] = count
        signal["rollback_trigger_delay_count"] = count
        signal["rollback_trigger_delay_steps"] = delay_steps
        if count <= delay_steps:
            delayed = dict(signal)
            for key in reason_keys:
                delayed[key] = False
            return delayed
        return signal

    def _build_rollback_trigger_signals(
        self,
        trust_scores: Dict[int, float],
        current_time_ns: Optional[int] = None,
    ) -> Dict[int, Dict[str, object]]:
        """Build per-target rollback trigger reasons from trust flags and final trust."""
        threshold = float(np.clip(self.trust_config.trust_threshold, 0.0, 1.0))
        suppress_startup = self._suppress_rollback_triggers_during_startup(
            current_time_ns
        )
        signals: Dict[int, Dict[str, object]] = {}

        for vehicle_id, trust_val in trust_scores.items():
            target_id = int(vehicle_id)
            if target_id == self.vehicle_id:
                continue

            trust_obj = self.trust_model.get_trust_score(target_id)
            raw_signal = {
                "trust_below_threshold": bool(
                    self.rollback_on_final_trust
                    and float(trust_val) < threshold
                    and self._is_local_channel_untrusted(target_id, float(trust_val))
                ),
                "flag_local_est_check": bool(
                    self.rollback_on_local_est_check
                    and trust_obj is not None
                    and getattr(trust_obj, "flag_local_est_check", False)
                ),
                "flag_global_est_check": bool(
                    self.rollback_on_global_est_check
                    and trust_obj is not None
                    and getattr(trust_obj, "flag_global_est_check", False)
                ),
                "final_trust": float(trust_val),
            }
            if suppress_startup:
                raw_signal["startup_trigger_suppressed"] = True
                raw_signal["startup_trigger_suppress_duration_s"] = float(
                    self.rollback_startup_suppress_duration_s
                )
                raw_signal["startup_elapsed_s"] = self._get_startup_elapsed_s(
                    current_time_ns
                )
                self._rollback_trigger_delay_state.pop(target_id, None)
                for key in (
                    "trust_below_threshold",
                    "flag_local_est_check",
                    "flag_global_est_check",
                ):
                    raw_signal[key] = False
                signals[target_id] = raw_signal
                continue

            signals[target_id] = self._apply_rollback_trigger_delay(
                target_id, raw_signal
            )

        return signals

    # ------------------------------------------------------------------
    # V2V attack metadata for trust-log ground truth
    # ------------------------------------------------------------------
    def set_v2v_attack_status(self, status: Optional[Dict[str, Any]]) -> None:
        """
        Store V2V attack metadata supplied by VehicleLogic.

        This is GUI-command ground truth for the trust CSV. The trust model's
        detection flag remains separate as flag_attack_<vehicle_id>.
        """
        self._v2v_attack_tracker.set_status(status)

    def _get_v2v_attack_log_data(self, current_time_ns: int) -> Dict[str, Any]:
        """Build flattened V2V attack metadata for TrustWeightLogger."""
        return self._v2v_attack_tracker.get_log_data(
            current_time_ns=current_time_ns,
            fleet_size=self.fleet_size,
            fallback_clock_s=self._get_log_time_s(current_time_ns),
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
        relative_bearing: Optional[float] = None,
    ) -> bool:
        """Store externally measured host-target relative states (e.g., YOLO/radar)."""
        return self._ext_cache.set(
            target_id, distance, relative_velocity,
            timestamp_ns, source, measurement_confidence, relative_bearing,
        )

    def clear_external_relative_measurement(self, target_id: Optional[int] = None) -> None:
        """Clear cached external relative measurement(s)."""
        self._ext_cache.clear(target_id)

    # ------------------------------------------------------------------
    # Control input helpers
    # ------------------------------------------------------------------
    def _get_vehicle_model_config(self, target_id: int = -1) -> Dict[str, Any]:
        return vehicle_model_for_target(
            default_vehicle_model=self.default_vehicle_model,
            vehicle_model_overrides=self.vehicle_model_overrides,
            target_id=target_id,
            global_prediction_mode=self.dynamics_prediction_mode,
        )

    def _get_target_control(
        self,
        target_id: int,
        host_control: np.ndarray,
        current_time_ns: Optional[int] = None,
    ) -> Optional[np.ndarray]:
        """Return the target vehicle's latest valid control input from V2V cache."""
        cached = self._received_control_inputs.get(target_id)
        if cached is not None:
            if current_time_ns is not None:
                age_s = (
                    float(current_time_ns) - float(cached.get("timestamp_ns", current_time_ns))
                ) / 1e9
                if age_s > max(self.control_timeout_s, 0.0):
                    cached = None
        if cached is not None:
            return np.array([cached["steering"], cached["throttle"]])

        model_cfg = self._get_vehicle_model_config(target_id)
        if bool(model_cfg.get("allow_host_control_fallback", False)):
            return host_control
        return None

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

            # 2.1 Build generalized trust vector O_i(j) when enabled
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

            self._apply_generalized_trust_attack_flags()

            # 2.2 Apply attack mitigation before weight computation/update
            if self.attack_mitigation_enabled:
                self._apply_attack_mitigation(trust_scores, current_time_ns)

            self._update_direct_trust_delay_states(trust_scores)
            self._update_direct_channel_recovery_states(trust_scores)

            # 3. Calculate adaptive weights based on trust/opinion
            weight_source_scores = (
                self.generalized_trust_vector
                if self.weight_config.weight_type == "paper"
                else trust_scores
            )
            if self._use_startup_fixed_weights(current_time_ns):
                weight_result = self._get_startup_weight_result(trust_scores)
            else:
                weight_result = self.weight_module.calculate_weights(
                    weight_source_scores
                )
            self.current_weight_result = weight_result

            # 4. Update estimates for other vehicles
            pre_update_states = self.fleet_states.copy()
            step_targets: Dict[int, Dict] = {}
            target_confidence: Dict[int, float] = {}
            target_prediction_mode: Dict[int, bool] = {}
            consensus_estimates: Dict[int, Dict[str, float]] = {}
            post_prediction_estimates: Dict[int, Dict[str, float]] = {}
            clean_reference_estimates: Dict[int, Dict[str, float]] = {}
            prediction_debugs: Dict[int, Dict[str, float]] = {}

            for target_id in trust_scores.keys():
                if target_id == self.vehicle_id:
                    continue

                current_est = self.fleet_states[:, target_id].copy()

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
                target_ctrl = self._get_target_control(
                    target_id, control, current_time_ns=current_time_ns
                )
                target_trust_obj = self.trust_model.get_trust_score(target_id)
                target_model_cfg = self._get_vehicle_model_config(target_id)
                prediction_mode = normalize_dynamics_prediction_mode(
                    target_model_cfg.get(
                        "dynamics_prediction_mode", self.dynamics_prediction_mode
                    )
                )
                target_attack_or_quarantine = bool(
                    self._has_active_attack_flags(target_trust_obj)
                    or self._is_target_quarantined_by_rollback(target_id)
                )
                force_clean_pose_anchor = bool(
                    self.force_clean_pose_anchor
                    and prediction_mode
                    in ("clean_data", "mixed_clean_data", "relative_host_anchor_mixed")
                    and target_attack_or_quarantine
                )
                attack_relative_host_anchor_active = bool(
                    target_attack_or_quarantine
                    and (
                        prediction_mode == "relative_host_anchor_mixed"
                        or (
                            prediction_mode == "mixed_clean_data"
                            and force_clean_pose_anchor
                        )
                    )
                )
                target_prediction_mode[target_id] = bool(prediction_mode != "none")
                consensus_est = normal_est.copy()
                consensus_estimates[target_id] = self._state_vector_to_log_dict(
                    consensus_est
                )
                clean_reference = self._get_clean_reference_log_state(
                    target_id=target_id,
                    current_time_ns=current_time_ns,
                )
                if clean_reference is not None:
                    clean_reference_estimates[target_id] = clean_reference

                host_anchor_snapshot = None
                if attack_relative_host_anchor_active:
                    host_anchor_snapshot = self._build_relative_host_anchor_snapshot(
                        target_id=target_id,
                        current_time_ns=current_time_ns,
                        reference_state=consensus_est,
                    )

                components["prediction"] = {
                    "dt": float(dt),
                    "control": None
                    if target_ctrl is None
                    else np.asarray(target_ctrl, dtype=float).copy(),
                    "force_clean_pose_anchor": bool(force_clean_pose_anchor),
                    "attack_relative_host_anchor_active": bool(
                        attack_relative_host_anchor_active
                    ),
                    "host_anchor_snapshot": self._copy_host_anchor_snapshot(
                        host_anchor_snapshot
                    ),
                }
                predicted_est = self._apply_state_constraints(
                    self._predict_dynamics(
                        consensus_est,
                        target_ctrl,
                        dt,
                        target_id=target_id,
                        current_time_ns=current_time_ns,
                        force_clean_pose_anchor=force_clean_pose_anchor,
                        attack_relative_host_anchor_active=attack_relative_host_anchor_active,
                        host_anchor_snapshot=host_anchor_snapshot,
                    ),
                    target_id=target_id,
                )
                normal_est = predicted_est
                post_prediction_estimates[target_id] = self._state_vector_to_log_dict(
                    normal_est
                )
                prediction_debugs[target_id] = self._build_prediction_debug(
                    prediction_mode=prediction_mode,
                    consensus_state=consensus_est,
                    predicted_state=normal_est,
                    target_control=target_ctrl,
                    host_control=control,
                    dt=dt,
                    attack_relative_host_anchor_active=attack_relative_host_anchor_active,
                )
                attack_alpha_override = None
                if force_clean_pose_anchor:
                    attack_alpha_override = 1.0
                elif (
                    components["weights"].get("w0", 0.0) <= 1e-9
                    or bool(
                        target_trust_obj is not None
                        and getattr(target_trust_obj, "flag_local_est_check", False)
                    )
                ):
                    attack_alpha_override = self.attack_output_low_pass_alpha

                final_est = self._apply_output_low_pass_filter(
                    previous_state=current_est,
                    new_state=normal_est,
                    target_id=target_id,
                    alpha_override=attack_alpha_override,
                )

                self.fleet_states[:, target_id] = final_est
                step_targets[target_id] = components

            rollback_status = self.rollback.get_status()

            # 5.1 Contamination rollback
            if self.rollback.enabled:
                rollback_trigger_signals = self._build_rollback_trigger_signals(
                    trust_scores,
                    current_time_ns=current_time_ns,
                )
                self.rollback.record(
                    current_time_ns=current_time_ns,
                    pre_update_states=pre_update_states,
                    target_components=step_targets,
                )
                self.fleet_states = self.rollback.check_and_trigger(
                    trust_scores=trust_scores,
                    current_time_ns=current_time_ns,
                    fleet_states=self.fleet_states,
                    trigger_signals=rollback_trigger_signals,
                )
                rollback_status = self.rollback.get_status()
                post_rollback_anchor_count = (
                    self._apply_post_rollback_anchor_correction(
                        trust_scores=trust_scores,
                        current_time_ns=current_time_ns,
                        rollback_status=rollback_status,
                    )
                )
                if post_rollback_anchor_count > 0:
                    rollback_status = dict(rollback_status)
                    rollback_status["post_rollback_anchor_count"] = int(
                        post_rollback_anchor_count
                    )
                self._update_rollback_trusted_state_history(
                    trust_scores=trust_scores,
                    current_time_ns=current_time_ns,
                )

            # 5.2 Log trust/weight/estimation state after rollback has settled
            self._log_update(
                trust_scores,
                weight_result,
                control,
                target_confidence,
                target_prediction_mode,
                current_time_ns=current_time_ns,
                target_components=step_targets,
                rollback_status=rollback_status,
                consensus_estimates=consensus_estimates,
                post_prediction_estimates=post_prediction_estimates,
                clean_reference_estimates=clean_reference_estimates,
                prediction_debugs=prediction_debugs,
            )

            # 6. Cleanup old data
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
                target_data.relative_bearing_from_host = float(
                    external_rel.get("relative_bearing", float("nan"))
                )
                if np.isfinite(target_data.relative_bearing_from_host):
                    target_data.relative_heading = float(
                        target_data.relative_bearing_from_host
                    )
                target_data.relative_measurement_source = str(
                    external_rel.get("source", "external_sensor")
                )
                target_data.relative_measurement_timestamp_ns = int(
                    external_rel.get("timestamp_ns", 0.0)
                )
            else:
                self._attach_clean_v2v_relative_measurement(
                    target_data=target_data,
                    current_time_ns=current_time_ns,
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
                fleet_ts_ns, raw_target_fleet_data = fleet_entry
                target_fleet_data = self._align_fleet_snapshot(
                    raw_target_fleet_data, fleet_ts_ns, current_time_ns
                )

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
            est_dict = self._align_state_dict_to_time(
                fleet_data[vehicle_id],
                snapshot_ts_ns=fleet_ts_ns,
                current_time_ns=current_time_ns,
                target_id=vehicle_id,
            )
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

    def _get_latest_clean_received_state_with_timestamp(
        self, vehicle_id: int, current_time_ns: int
    ) -> Optional[Tuple[int, np.ndarray]]:
        """Return the latest valid clean-channel local state for trust-only use."""
        if vehicle_id not in self.received_clean_local_states:
            return None
        history = self.received_clean_local_states[vehicle_id]
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
                return int(ts_ns), state_vec
        return None

    def _get_latest_clean_aligned_state(
        self, target_id: int, current_time_ns: int
    ) -> Optional[np.ndarray]:
        """Return the latest clean-channel target state aligned to current time."""
        clean_entry = self._get_latest_clean_received_state_with_timestamp(
            int(target_id), current_time_ns
        )
        if clean_entry is None:
            return None
        clean_ts_ns, clean_state_raw = clean_entry
        return self._align_state_array_to_time(
            clean_state_raw,
            snapshot_ts_ns=clean_ts_ns,
            current_time_ns=current_time_ns,
            target_id=target_id,
        )

    def _attach_clean_v2v_relative_measurement(
        self, target_data: VehicleData, current_time_ns: int
    ) -> None:
        """
        Build an independent relative measurement from the clean V2V channel.

        This uses the clean broadcast only for trust validation. The attacked
        local-state path remains the primary control/estimation input.
        """
        clean_entry = self._get_latest_clean_received_state_with_timestamp(
            int(target_data.vehicle_id), current_time_ns
        )
        if clean_entry is None or not self.host_state:
            return

        clean_ts_ns, clean_state = clean_entry
        clean_target = VehicleData(
            vehicle_id=int(target_data.vehicle_id),
            x=float(clean_state[0]),
            y=float(clean_state[1]),
            theta=float(clean_state[2]),
            velocity=float(clean_state[3]),
            acceleration=float(clean_state[4]) if len(clean_state) > 4 else 0.0,
            timestamp_ns=int(clean_ts_ns),
        )
        rel_distance, _ = self.trust_model._resolve_relative_distance(
            self.host_state, clean_target
        )
        rel_velocity = self.trust_model._estimate_radial_relative_velocity(
            self.host_state, clean_target
        )
        rel_bearing = self.trust_model._relative_bearing_from_geometry(
            self.host_state, clean_target
        )

        target_data.distance_from_host = float(rel_distance)
        target_data.relative_velocity_from_host = float(rel_velocity)
        target_data.relative_measurement_confidence = 1.0
        target_data.relative_bearing_from_host = float(rel_bearing)
        target_data.relative_heading = float(rel_bearing)
        target_data.relative_measurement_source = "v2v_clean_local_state"
        target_data.relative_measurement_timestamp_ns = int(clean_ts_ns)

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

    def _cleanup_old_data(self, current_time_ns: int):
        """Extend base cleanup to cover trust-only clean local-state history."""
        super()._cleanup_old_data(current_time_ns)
        try:
            for vehicle_id in list(self.received_clean_local_states.keys()):
                states_list = self.received_clean_local_states[vehicle_id]
                valid_states = [
                    (ts_ns, state)
                    for ts_ns, state in states_list
                    if current_time_ns - ts_ns <= self.max_state_age_ns
                ]
                if valid_states:
                    self.received_clean_local_states[vehicle_id] = valid_states
                else:
                    del self.received_clean_local_states[vehicle_id]
        except Exception as exc:
            if self.logger:
                self.logger.log_error("Clean local-state cleanup error", exc)

    def _timestamp_alignment_enabled(self) -> bool:
        return bool(self.timestamp_alignment_config.get("enabled", True))

    def _max_alignment_s(self) -> float:
        return max(
            float(self.timestamp_alignment_config.get("max_extrapolation_s", 0.5)),
            0.0,
        )

    def _align_state_dict_to_time(
        self,
        state_dict: Dict[str, Any],
        snapshot_ts_ns: int,
        current_time_ns: int,
        target_id: int,
    ) -> Dict[str, Any]:
        """
        Align a received fleet estimate to the host time using constant-velocity
        propagation. This avoids comparing fleet snapshots from different ages
        as if they were simultaneous.
        """
        if not self._timestamp_alignment_enabled() or not isinstance(state_dict, dict):
            return dict(state_dict) if isinstance(state_dict, dict) else {}

        dt = (float(current_time_ns) - float(snapshot_ts_ns)) / 1e9
        if dt <= 0.0 or dt > self._max_alignment_s():
            return dict(state_dict)

        aligned = dict(state_dict)
        theta = as_float(aligned.get("theta"), 0.0)
        velocity = as_float(aligned.get("velocity", aligned.get("v")), 0.0)
        aligned["x"] = as_float(aligned.get("x"), 0.0) + velocity * np.cos(theta) * dt
        aligned["y"] = as_float(aligned.get("y"), 0.0) + velocity * np.sin(theta) * dt
        aligned["timestamp_aligned_dt_s"] = float(dt)

        # Acceleration is often model-defined rather than measured; do not
        # extrapolate it across vehicles here.
        if "acceleration" not in aligned and "a" in aligned:
            aligned["acceleration"] = aligned.get("a")
        return aligned

    def _align_state_array_to_time(
        self,
        state_vec: np.ndarray,
        snapshot_ts_ns: int,
        current_time_ns: int,
        target_id: int,
    ) -> np.ndarray:
        """
        Align a direct local-state broadcast to the host time using the same
        constant-velocity x/y propagation used for fleet snapshots.
        """
        aligned = _normalize_state_array(state_vec, self.state_dim, logger=self.logger)
        if aligned is None:
            return np.zeros(self.state_dim, dtype=float)
        aligned = np.asarray(aligned, dtype=float).copy()

        if not self._timestamp_alignment_enabled():
            return aligned

        dt = (float(current_time_ns) - float(snapshot_ts_ns)) / 1e9
        if dt <= 0.0 or dt > self._max_alignment_s():
            return aligned

        theta = float(aligned[2]) if aligned.shape[0] > 2 else 0.0
        velocity = float(aligned[3]) if aligned.shape[0] > 3 else 0.0
        aligned[0] = float(aligned[0]) + velocity * np.cos(theta) * dt
        aligned[1] = float(aligned[1]) + velocity * np.sin(theta) * dt
        return aligned

    def _align_fleet_snapshot(
        self, fleet_data: Dict[int, Dict], snapshot_ts_ns: int, current_time_ns: int
    ) -> Dict[int, Dict]:
        if not isinstance(fleet_data, dict):
            return {}
        aligned: Dict[int, Dict] = {}
        for raw_vid, state_dict in fleet_data.items():
            try:
                vid = int(raw_vid)
            except (TypeError, ValueError):
                continue
            aligned[vid] = self._align_state_dict_to_time(
                state_dict=state_dict,
                snapshot_ts_ns=snapshot_ts_ns,
                current_time_ns=current_time_ns,
                target_id=vid,
            )
        return aligned

    def _state_vector_to_log_dict(self, state: np.ndarray) -> Dict[str, float]:
        """Convert a state vector into the logger's state dictionary shape."""
        arr = np.asarray(state, dtype=float).reshape(-1)
        if arr.size < self.state_dim:
            arr = np.pad(arr, (0, self.state_dim - arr.size), mode="constant")
        return {
            "x": float(arr[0]),
            "y": float(arr[1]),
            "theta": float(arr[2]) if arr.size > 2 else 0.0,
            "velocity": float(arr[3]) if arr.size > 3 else 0.0,
            "acceleration": float(arr[4]) if arr.size > 4 else 0.0,
        }

    def _get_clean_reference_log_state(
        self, target_id: int, current_time_ns: int
    ) -> Optional[Dict[str, float]]:
        """Return time-aligned clean-channel target state for diagnostics."""
        clean_entry = self._get_latest_clean_received_state_with_timestamp(
            int(target_id), current_time_ns
        )
        if clean_entry is None:
            return None
        clean_ts_ns, clean_state_raw = clean_entry
        clean_state = self._align_state_array_to_time(
            clean_state_raw,
            snapshot_ts_ns=clean_ts_ns,
            current_time_ns=current_time_ns,
            target_id=target_id,
        )
        return self._state_vector_to_log_dict(clean_state)

    def _build_prediction_debug(
        self,
        prediction_mode: str,
        consensus_state: np.ndarray,
        predicted_state: np.ndarray,
        target_control: Optional[np.ndarray],
        host_control: np.ndarray,
        dt: float,
        attack_relative_host_anchor_active: bool,
    ) -> Dict[str, float]:
        """Build compact prediction-step diagnostics for the trust CSV."""
        consensus = np.asarray(consensus_state, dtype=float)
        predicted = np.asarray(predicted_state, dtype=float)
        target_ctrl = (
            None if target_control is None else np.asarray(target_control, dtype=float)
        )
        host_ctrl = np.asarray(host_control, dtype=float)
        source = str(prediction_mode)
        if attack_relative_host_anchor_active:
            source = f"{source}+host_anchor"
        return {
            "source": source,
            "dt": float(dt),
            "dx": float(predicted[0] - consensus[0]),
            "dy": float(predicted[1] - consensus[1]),
            "steering_used": (
                float(target_ctrl[0])
                if target_ctrl is not None and target_ctrl.size > 0
                else float("nan")
            ),
            "host_steering_used": (
                float(host_ctrl[0]) if host_ctrl.size > 0 else float("nan")
            ),
            "theta_in": float(consensus[2]) if consensus.size > 2 else float("nan"),
            "theta_out": float(predicted[2]) if predicted.size > 2 else float("nan"),
            "v_in": float(consensus[3]) if consensus.size > 3 else float("nan"),
            "v_out": float(predicted[3]) if predicted.size > 3 else float("nan"),
            "host_v": float(self.host_state.get("velocity", float("nan")))
            if self.host_state
            else float("nan"),
        }

    def _apply_post_rollback_anchor_correction(
        self,
        trust_scores: Dict[int, float],
        current_time_ns: int,
        rollback_status: Optional[Dict[str, object]] = None,
    ) -> int:
        """Optionally apply one final attack-time anchor after rollback replay.

        Rollback replay already runs the prediction model for each replayed step.
        This hook is a conservative final alignment at the current timestamp; it
        does not propagate forward by another dt.
        """
        if not getattr(self, "post_rollback_anchor_enabled", False):
            return 0
        rollback_status = rollback_status or {}
        if not bool(rollback_status.get("triggered", False)):
            return 0

        applied_count = 0
        for raw_target_id in trust_scores.keys():
            target_id = int(raw_target_id)
            if target_id == self.vehicle_id:
                continue
            self._ensure_fleet_capacity(target_id)

            target_trust_obj = self.trust_model.get_trust_score(target_id)
            target_attack_or_quarantine = bool(
                self._has_active_attack_flags(target_trust_obj)
                or self._is_target_quarantined_by_rollback(target_id)
            )
            if not target_attack_or_quarantine:
                continue

            target_model_cfg = self._get_vehicle_model_config(target_id)
            prediction_mode = normalize_dynamics_prediction_mode(
                target_model_cfg.get(
                    "dynamics_prediction_mode", self.dynamics_prediction_mode
                )
            )
            if prediction_mode not in (
                "clean_data",
                "mixed_clean_data",
                "relative_host_anchor_mixed",
            ):
                continue

            force_clean_pose_anchor = bool(
                self.force_clean_pose_anchor
                and prediction_mode
                in ("clean_data", "mixed_clean_data", "relative_host_anchor_mixed")
            )
            attack_relative_host_anchor_active = bool(
                prediction_mode == "relative_host_anchor_mixed"
                or (
                    prediction_mode == "mixed_clean_data"
                    and force_clean_pose_anchor
                )
            )
            anchored_state = self._anchor_current_state_for_attack(
                state=self.fleet_states[:, target_id],
                target_id=target_id,
                current_time_ns=current_time_ns,
                prediction_mode=prediction_mode,
                force_clean_pose_anchor=force_clean_pose_anchor,
                attack_relative_host_anchor_active=attack_relative_host_anchor_active,
            )
            if anchored_state is None:
                continue

            self.fleet_states[:, target_id] = self._apply_state_constraints(
                anchored_state, target_id=target_id
            )
            applied_count += 1

        return applied_count

    def _anchor_current_state_for_attack(
        self,
        state: np.ndarray,
        target_id: int,
        current_time_ns: int,
        prediction_mode: str,
        force_clean_pose_anchor: bool,
        attack_relative_host_anchor_active: bool,
    ) -> Optional[np.ndarray]:
        """Anchor a current-time state without applying another prediction dt."""
        state = np.asarray(state, dtype=float).copy()
        if state.size < self.state_dim:
            state = np.pad(state, (0, self.state_dim - state.size), mode="constant")

        clean_state = self._get_latest_clean_aligned_state(
            target_id=target_id,
            current_time_ns=current_time_ns,
        )
        if clean_state is not None:
            clean_state = np.asarray(clean_state, dtype=float).copy()

        if prediction_mode == "clean_data":
            if clean_state is None:
                return None
            anchored = state.copy()
            if force_clean_pose_anchor:
                if clean_state.shape[0] > 0:
                    anchored[0] = float(clean_state[0])
                if clean_state.shape[0] > 1:
                    anchored[1] = float(clean_state[1])
            if anchored.shape[0] > 2 and clean_state.shape[0] > 2:
                anchored[2] = self._wrap_angle(float(clean_state[2]))
            if anchored.shape[0] > 3 and clean_state.shape[0] > 3:
                anchored[3] = float(clean_state[3])
            if anchored.shape[0] > 4 and clean_state.shape[0] > 4:
                anchored[4] = float(clean_state[4])
            return anchored

        if prediction_mode in ("mixed_clean_data", "relative_host_anchor_mixed"):
            if not attack_relative_host_anchor_active:
                return None
            return self._anchor_current_state_from_relative_host(
                state=state,
                target_id=target_id,
                current_time_ns=current_time_ns,
                clean_state=clean_state,
                force_clean_pose_anchor=force_clean_pose_anchor,
            )

        return None

    def _anchor_current_state_from_relative_host(
        self,
        state: np.ndarray,
        target_id: int,
        current_time_ns: int,
        clean_state: Optional[np.ndarray],
        force_clean_pose_anchor: bool,
    ) -> Optional[np.ndarray]:
        """Apply relative-host anchoring at the current time without extra dt."""
        anchor_snapshot = self._build_relative_host_anchor_snapshot(
            target_id=target_id,
            current_time_ns=current_time_ns,
            reference_state=state,
            clean_state=clean_state,
        )

        theta = float(state[2]) if state.shape[0] > 2 else 0.0
        velocity = float(state[3]) if state.shape[0] > 3 else 0.0
        acceleration = float(state[4]) if state.shape[0] > 4 else 0.0
        host_theta = (
            float(anchor_snapshot.get("host_theta", theta))
            if anchor_snapshot is not None
            else theta
        )

        if clean_state is not None and clean_state.shape[0] > 2:
            theta = self._blend_angles(
                float(clean_state[2]),
                host_theta,
                primary_weight=self.relative_host_anchor_clean_theta_weight,
                secondary_weight=self.relative_host_anchor_host_theta_weight,
            )
        elif anchor_snapshot is not None:
            theta = host_theta

        if anchor_snapshot is not None:
            host_velocity = float(anchor_snapshot.get("host_velocity", velocity))
            velocity = (
                self.relative_host_anchor_target_velocity_weight * velocity
                + self.relative_host_anchor_host_velocity_weight * host_velocity
            )
            host_acceleration = float(
                anchor_snapshot.get("host_acceleration", acceleration)
            )
            acceleration = (
                self.relative_host_anchor_target_acceleration_weight * acceleration
                + self.relative_host_anchor_host_acceleration_weight * host_acceleration
            )

        anchor_x = float(state[0])
        anchor_y = float(state[1])
        has_position_anchor = False
        if anchor_snapshot is not None:
            host_x = float(anchor_snapshot.get("host_x", anchor_x))
            host_y = float(anchor_snapshot.get("host_y", anchor_y))
            distance = max(float(anchor_snapshot.get("distance", 0.1)), 0.1)
            sign = 1.0 if float(anchor_snapshot.get("sign", 1.0)) >= 0.0 else -1.0
            relative_x = float(anchor_snapshot.get("relative_x", float("nan")))
            relative_y = float(anchor_snapshot.get("relative_y", float("nan")))
            if (
                getattr(self, "relative_host_anchor_use_bearing", True)
                and np.isfinite(relative_x)
                and np.isfinite(relative_y)
            ):
                cos_h = float(np.cos(host_theta))
                sin_h = float(np.sin(host_theta))
                anchor_x = host_x + cos_h * relative_x - sin_h * relative_y
                anchor_y = host_y + sin_h * relative_x + cos_h * relative_y
            else:
                anchor_x = host_x + sign * distance * np.cos(host_theta)
                anchor_y = host_y + sign * distance * np.sin(host_theta)
            has_position_anchor = True
        elif force_clean_pose_anchor and clean_state is not None:
            if clean_state.shape[0] > 0:
                anchor_x = float(clean_state[0])
            if clean_state.shape[0] > 1:
                anchor_y = float(clean_state[1])
            has_position_anchor = True

        if not has_position_anchor and clean_state is None and anchor_snapshot is None:
            return None

        anchored = state.copy()
        if has_position_anchor:
            anchor_w = max(
                float(getattr(self, "relative_host_anchor_anchor_position_weight", 1.0)),
                0.0,
            )
            estimate_w = max(
                float(getattr(self, "relative_host_anchor_estimate_position_weight", 0.0)),
                0.0,
            )
            weight_sum = anchor_w + estimate_w
            if weight_sum > 1e-9:
                anchored[0] = (anchor_w * anchor_x + estimate_w * float(state[0])) / weight_sum
                anchored[1] = (anchor_w * anchor_y + estimate_w * float(state[1])) / weight_sum
            else:
                anchored[0] = anchor_x
                anchored[1] = anchor_y
        if anchored.shape[0] > 2:
            anchored[2] = self._wrap_angle(theta)
        if anchored.shape[0] > 3:
            anchored[3] = velocity
        if anchored.shape[0] > 4:
            anchored[4] = acceleration
        return anchored

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
        use_startup_fixed_weights = self._use_startup_fixed_weights(current_time_ns)
        apply_trust_channel_gating = not (
            use_startup_fixed_weights or self.weight_config.weight_type == "equal"
        )
        current_malicious_ids = (
            self._get_current_malicious_vehicle_ids(trust_scores)
            if apply_trust_channel_gating
            else set()
        )

        components = {
            "direct": {"source": target_id, "weight": 0.0, "state": None},
            "neighbors": {},
            "prediction": {"dt": float(dt), "control": None},
            "weights": {"w0": 0.0, "w_self": 1.0, "neighbors": {}},
        }
        raw_target_trust_obj = self.trust_model.get_trust_score(target_id)
        target_trust_obj = self._effective_direct_trust_obj(
            target_id, raw_target_trust_obj
        )
        allow_direct_channel = (
            True
            if not apply_trust_channel_gating
            else self._is_direct_measurement_allowed(target_id, trust_scores)
        )

        # Get direct measurement from target
        direct_state = None
        if allow_direct_channel:
            direct_entry = self._get_latest_received_state_with_timestamp(
                target_id, current_time_ns
            )
            if direct_entry is not None:
                direct_ts_ns, direct_state_raw = direct_entry
                direct_state = self._align_state_array_to_time(
                    direct_state_raw,
                    snapshot_ts_ns=direct_ts_ns,
                    current_time_ns=current_time_ns,
                    target_id=target_id,
                )

        # Cache available neighbor fleet snapshots containing this target
        neighbor_fleet_estimates: Dict[int, Dict] = {}
        for neighbor_id in self.received_fleet_states.keys():
            if neighbor_id == self.vehicle_id or neighbor_id in current_malicious_ids:
                continue
            fleet_entry = self._get_latest_fleet_data_with_timestamp(
                neighbor_id, current_time_ns
            )
            if fleet_entry is None:
                continue
            fleet_ts_ns, neighbor_fleet_raw = fleet_entry
            neighbor_fleet = self._align_fleet_snapshot(
                neighbor_fleet_raw, fleet_ts_ns, current_time_ns
            )
            if target_id not in neighbor_fleet:
                continue
            neighbor_fleet_estimates[neighbor_id] = neighbor_fleet

        # Calculate weights (paper or trust-based - unified call)
        if use_startup_fixed_weights:
            target_weights = self._get_startup_target_weights(
                target_id=target_id,
                neighbor_fleet_estimates=neighbor_fleet_estimates,
                direct_state=direct_state,
            )
        elif self.weight_config.weight_type == "paper":
            opinion_scores = (
                self.generalized_trust_vector
                if self.generalized_trust_vector
                else trust_scores
            )
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
                target_trust_obj=target_trust_obj,
            )
        target_weights = self._apply_direct_recovery_weight_scale(
            target_id=target_id,
            weights=target_weights,
        )

        # === Flag-Driven w₀ Adaptation ===
        components["weights"] = {
            "w0": float(target_weights.get("w0", 0.0)),
            "w_self": float(target_weights.get("w_self", 0.0)),
            "neighbors": {
                int(neighbor_id): float(weight)
                for neighbor_id, weight in target_weights.get("neighbors", {}).items()
            },
        }

        # === Direct Measurement Correction ===
        if direct_state is not None and target_weights["w0"] > 0:
            direct_delta = target_weights["w0"] * self._state_residual(
                direct_state, current_est
            )
            total_correction += direct_delta
            components["direct"] = {
                "source": target_id,
                "weight": float(target_weights["w0"]),
                "state": np.asarray(direct_state, dtype=float).copy(),
            }

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
            neighbor_delta = w_neighbor * self._state_residual(neigh_est, current_est)
            total_correction += neighbor_delta
            components["neighbors"][neighbor_id] = {
                "weight": float(w_neighbor),
                "state": neigh_est.copy(),
            }

        # === Apply Consensus Correction ===
        # Dynamics propagation f(x̂_corrected, u, dt) is applied in update()
        new_est = self._apply_state_constraints(
            current_est + total_correction, target_id=target_id
        )
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
    def _predict_from_clean_data_motion(
        self,
        state: np.ndarray,
        dt: float,
        target_id: int,
        current_time_ns: Optional[int] = None,
        force_clean_pose_anchor: bool = False,
    ) -> np.ndarray:
        """
        Predict using the clean V2V target broadcast as the motion source.

        This keeps the host's current x/y estimate continuity, but drives the
        motion with clean theta/velocity/acceleration for testing.
        When `force_clean_pose_anchor` is enabled, x/y starts from the aligned
        clean V2V pose instead of the rollback/consensus pose.
        """
        state = np.asarray(state, dtype=float).copy()
        if dt <= 0.0:
            return state

        clean_state = None
        if current_time_ns is not None:
            clean_entry = self._get_latest_clean_received_state_with_timestamp(
                target_id, current_time_ns
            )
            if clean_entry is not None:
                clean_ts_ns, clean_state_raw = clean_entry
                clean_state = self._align_state_array_to_time(
                    clean_state_raw,
                    snapshot_ts_ns=clean_ts_ns,
                    current_time_ns=current_time_ns,
                    target_id=target_id,
                )
        else:
            history = self.received_clean_local_states.get(int(target_id), [])
            if history:
                clean_state = _normalize_state_array(
                    history[-1][1], self.state_dim, logger=self.logger
                )

        if clean_state is None:
            theta = float(state[2]) if state.shape[0] > 2 else 0.0
            velocity = float(state[3]) if state.shape[0] > 3 else 0.0
            acceleration = float(state[4]) if state.shape[0] > 4 else 0.0
        else:
            clean_state = np.asarray(clean_state, dtype=float).copy()
            theta = float(clean_state[2]) if clean_state.shape[0] > 2 else 0.0
            velocity = float(clean_state[3]) if clean_state.shape[0] > 3 else 0.0
            acceleration = (
                float(clean_state[4])
                if clean_state.shape[0] > 4
                else float(state[4]) if state.shape[0] > 4 else 0.0
            )

        predicted = state.copy()
        base_x = float(state[0])
        base_y = float(state[1])
        if force_clean_pose_anchor and clean_state is not None:
            if clean_state.shape[0] > 0:
                base_x = float(clean_state[0])
            if clean_state.shape[0] > 1:
                base_y = float(clean_state[1])
        predicted[0] = base_x + velocity * np.cos(theta) * dt
        predicted[1] = base_y + velocity * np.sin(theta) * dt
        if predicted.shape[0] > 2:
            predicted[2] = self._wrap_angle(theta)
        if predicted.shape[0] > 3:
            predicted[3] = velocity
        if predicted.shape[0] > 4:
            predicted[4] = acceleration
        return predicted

    def _predict_from_mixed_clean_data_motion(
        self,
        state: np.ndarray,
        dt: float,
        target_id: int,
        current_time_ns: Optional[int] = None,
        force_clean_pose_anchor: bool = False,
    ) -> np.ndarray:
        """
        Predict using clean V2V heading with host/estimator speed persistence.

        Theta comes from the clean target broadcast when available, while
        velocity and acceleration stay on the current estimator state.
        When `force_clean_pose_anchor` is enabled, x/y starts from the aligned
        clean V2V pose instead of the rollback/consensus pose.
        """
        state = np.asarray(state, dtype=float).copy()
        if dt <= 0.0:
            return state

        clean_state = None
        if current_time_ns is not None:
            clean_entry = self._get_latest_clean_received_state_with_timestamp(
                target_id, current_time_ns
            )
            if clean_entry is not None:
                clean_ts_ns, clean_state_raw = clean_entry
                clean_state = self._align_state_array_to_time(
                    clean_state_raw,
                    snapshot_ts_ns=clean_ts_ns,
                    current_time_ns=current_time_ns,
                    target_id=target_id,
                )
        else:
            history = self.received_clean_local_states.get(int(target_id), [])
            if history:
                clean_state = _normalize_state_array(
                    history[-1][1], self.state_dim, logger=self.logger
                )

        theta = float(state[2]) if state.shape[0] > 2 else 0.0
        velocity = float(state[3]) if state.shape[0] > 3 else 0.0
        acceleration = float(state[4]) if state.shape[0] > 4 else 0.0

        if clean_state is not None:
            clean_state = np.asarray(clean_state, dtype=float).copy()
            if clean_state.shape[0] > 2:
                theta = float(clean_state[2])

        predicted = state.copy()
        base_x = float(state[0])
        base_y = float(state[1])
        if force_clean_pose_anchor and clean_state is not None:
            if clean_state.shape[0] > 0:
                base_x = float(clean_state[0])
            if clean_state.shape[0] > 1:
                base_y = float(clean_state[1])
        predicted[0] = base_x + velocity * np.cos(theta) * dt
        predicted[1] = base_y + velocity * np.sin(theta) * dt
        if predicted.shape[0] > 2:
            predicted[2] = self._wrap_angle(theta)
        if predicted.shape[0] > 3:
            predicted[3] = velocity
        if predicted.shape[0] > 4:
            predicted[4] = acceleration
        return predicted

    def _predict_from_relative_host_anchor_mixed_motion(
        self,
        state: np.ndarray,
        control: Optional[np.ndarray],
        dt: float,
        target_id: int,
        current_time_ns: Optional[int] = None,
        force_clean_pose_anchor: bool = False,
        attack_relative_host_anchor_active: bool = False,
        host_anchor_snapshot: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        """
        Predict with normal dynamics unless an attack activates host anchoring.

        During attack, one x/y candidate is rebuilt from host pose plus the
        latest trusted relative distance. It is then blended with the normal
        model prediction so the anchor corrects drift without a hard jump.
        Heading blends clean target theta with host theta when both exist, and
        target velocity falls back to host velocity. Outside attack, this mode
        degrades to the normal vehicle model.
        """
        state = np.asarray(state, dtype=float).copy()
        if dt <= 0.0:
            return state
        if not attack_relative_host_anchor_active:
            return self._predict_with_vehicle_model(
                state=state,
                control=control,
                dt=dt,
                target_id=target_id,
            )

        model_predicted = self._predict_with_vehicle_model(
            state=state,
            control=control,
            dt=dt,
            target_id=target_id,
        )

        clean_state = None
        if current_time_ns is not None:
            clean_entry = self._get_latest_clean_received_state_with_timestamp(
                target_id, current_time_ns
            )
            if clean_entry is not None:
                clean_ts_ns, clean_state_raw = clean_entry
                clean_state = self._align_state_array_to_time(
                    clean_state_raw,
                    snapshot_ts_ns=clean_ts_ns,
                    current_time_ns=current_time_ns,
                    target_id=target_id,
                )
        else:
            history = self.received_clean_local_states.get(int(target_id), [])
            if history:
                clean_state = _normalize_state_array(
                    history[-1][1], self.state_dim, logger=self.logger
                )

        anchor_snapshot = self._copy_host_anchor_snapshot(host_anchor_snapshot)
        if anchor_snapshot is None and current_time_ns is not None:
            anchor_snapshot = self._build_relative_host_anchor_snapshot(
                target_id=target_id,
                current_time_ns=current_time_ns,
                reference_state=state,
                clean_state=clean_state,
            )

        theta = float(state[2]) if state.shape[0] > 2 else 0.0
        velocity = float(state[3]) if state.shape[0] > 3 else 0.0
        acceleration = float(state[4]) if state.shape[0] > 4 else 0.0

        host_theta = (
            float(anchor_snapshot.get("host_theta", theta))
            if anchor_snapshot is not None
            else theta
        )
        if clean_state is not None:
            clean_state = np.asarray(clean_state, dtype=float).copy()
            if clean_state.shape[0] > 2:
                clean_theta = float(clean_state[2])
                theta = self._blend_angles(
                    clean_theta,
                    host_theta,
                    primary_weight=self.relative_host_anchor_clean_theta_weight,
                    secondary_weight=self.relative_host_anchor_host_theta_weight,
                )
            else:
                theta = host_theta
        else:
            theta = host_theta

        if anchor_snapshot is not None:
            host_velocity = float(anchor_snapshot.get("host_velocity", velocity))
            velocity = (
                self.relative_host_anchor_target_velocity_weight * velocity
                + self.relative_host_anchor_host_velocity_weight * host_velocity
            )
            host_acceleration = float(
                anchor_snapshot.get("host_acceleration", acceleration)
            )
            acceleration = (
                self.relative_host_anchor_target_acceleration_weight * acceleration
                + self.relative_host_anchor_host_acceleration_weight * host_acceleration
            )

        predicted = state.copy()
        base_x = float(state[0])
        base_y = float(state[1])
        has_position_anchor = False
        if anchor_snapshot is not None:
            host_x = float(anchor_snapshot.get("host_x", base_x))
            host_y = float(anchor_snapshot.get("host_y", base_y))
            distance = max(float(anchor_snapshot.get("distance", 0.1)), 0.1)
            sign = 1.0 if float(anchor_snapshot.get("sign", 1.0)) >= 0.0 else -1.0
            relative_x = float(anchor_snapshot.get("relative_x", float("nan")))
            relative_y = float(anchor_snapshot.get("relative_y", float("nan")))
            if (
                getattr(self, "relative_host_anchor_use_bearing", True)
                and np.isfinite(relative_x)
                and np.isfinite(relative_y)
            ):
                cos_h = float(np.cos(host_theta))
                sin_h = float(np.sin(host_theta))
                base_x = host_x + cos_h * relative_x - sin_h * relative_y
                base_y = host_y + sin_h * relative_x + cos_h * relative_y
            else:
                base_x = host_x + sign * distance * np.cos(host_theta)
                base_y = host_y + sign * distance * np.sin(host_theta)
            has_position_anchor = True
        elif force_clean_pose_anchor and clean_state is not None:
            if clean_state.shape[0] > 0:
                base_x = float(clean_state[0])
            if clean_state.shape[0] > 1:
                base_y = float(clean_state[1])
            has_position_anchor = True

        anchor_predicted_x = base_x + velocity * np.cos(theta) * dt
        anchor_predicted_y = base_y + velocity * np.sin(theta) * dt
        if has_position_anchor and model_predicted.shape[0] > 1:
            anchor_w = max(
                float(
                    getattr(
                        self,
                        "relative_host_anchor_anchor_position_weight",
                        1.0,
                    )
                ),
                0.0,
            )
            estimate_w = max(
                float(
                    getattr(
                        self,
                        "relative_host_anchor_estimate_position_weight",
                        0.0,
                    )
                ),
                0.0,
            )
            weight_sum = anchor_w + estimate_w
            if weight_sum > 1e-9:
                predicted[0] = (
                    anchor_w * anchor_predicted_x
                    + estimate_w * float(model_predicted[0])
                ) / weight_sum
                predicted[1] = (
                    anchor_w * anchor_predicted_y
                    + estimate_w * float(model_predicted[1])
                ) / weight_sum
            else:
                predicted[0] = anchor_predicted_x
                predicted[1] = anchor_predicted_y
        else:
            predicted[0] = anchor_predicted_x
            predicted[1] = anchor_predicted_y
        if predicted.shape[0] > 2:
            predicted[2] = self._wrap_angle(theta)
        if predicted.shape[0] > 3:
            predicted[3] = velocity
        if predicted.shape[0] > 4:
            predicted[4] = acceleration
        return predicted

    def _predict_dynamics(
        self,
        state: np.ndarray,
        control: Optional[np.ndarray],
        dt: float,
        target_id: int = -1,
        current_time_ns: Optional[int] = None,
        force_clean_pose_anchor: bool = False,
        attack_relative_host_anchor_active: bool = False,
        host_anchor_snapshot: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        """Predict next state using bicycle kinematics + configured longitudinal model."""
        model_cfg = self._get_vehicle_model_config(target_id)
        prediction_mode = normalize_dynamics_prediction_mode(
            model_cfg.get("dynamics_prediction_mode", self.dynamics_prediction_mode)
        )
        state = np.asarray(state, dtype=float).copy()
        if dt <= 0.0 or prediction_mode == "none":
            return state
        if prediction_mode == "clean_data":
            return self._predict_from_clean_data_motion(
                state=state,
                dt=dt,
                target_id=target_id,
                current_time_ns=current_time_ns,
                force_clean_pose_anchor=force_clean_pose_anchor,
            )
        if prediction_mode in ("mixed_clean_data", "relative_host_anchor_mixed"):
            return self._predict_from_relative_host_anchor_mixed_motion(
                state=state,
                control=control,
                dt=dt,
                target_id=target_id,
                current_time_ns=current_time_ns,
                force_clean_pose_anchor=force_clean_pose_anchor,
                attack_relative_host_anchor_active=attack_relative_host_anchor_active,
                host_anchor_snapshot=host_anchor_snapshot,
            )
        return self._predict_with_vehicle_model(
            state=state,
            control=control,
            dt=dt,
            target_id=target_id,
        )

    @staticmethod
    def _blend_angles(
        primary_angle: float,
        secondary_angle: float,
        primary_weight: float = 1.0,
        secondary_weight: float = 1.0,
    ) -> float:
        """Blend two headings using a circular mean."""
        sin_sum = primary_weight * np.sin(primary_angle) + secondary_weight * np.sin(
            secondary_angle
        )
        cos_sum = primary_weight * np.cos(primary_angle) + secondary_weight * np.cos(
            secondary_angle
        )
        if abs(sin_sum) <= 1e-9 and abs(cos_sum) <= 1e-9:
            return float(primary_angle)
        return float(np.arctan2(sin_sum, cos_sum))

    def _predict_with_vehicle_model(
        self,
        state: np.ndarray,
        control: Optional[np.ndarray],
        dt: float,
        target_id: int,
    ) -> np.ndarray:
        """Predict next state using the configured vehicle model."""
        model_cfg = self._get_vehicle_model_config(target_id)
        x, y, theta, v = state[:4]
        a = state[4] if len(state) > 4 else 0.0

        if normalize_dynamics_prediction_mode(
            model_cfg.get("dynamics_prediction_mode", self.dynamics_prediction_mode)
        ) == "dead_reckoning":
            state[0] = x + v * np.cos(theta) * dt
            state[1] = y + v * np.sin(theta) * dt
            return state

        has_control = control is not None
        if has_control:
            steering = float(control[0]) if len(control) > 0 else 0.0
            throttle = float(control[1]) if len(control) > 1 else 0.0
        else:
            steering = 0.0
            throttle = 0.0

        steering = float(
            np.clip(
                steering,
                -float(model_cfg.get("max_steering", 0.5)),
                float(model_cfg.get("max_steering", 0.5)),
            )
        )
        L = max(float(model_cfg.get("wheelbase", 0.256)), 1e-6)

        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + (v * np.tan(steering) / L) * dt

        longitudinal_model = str(
            model_cfg.get("longitudinal_model", "constant_velocity")
        ).strip().lower()

        if not has_control:
            v_new = v
            a_new = 0.0
        elif longitudinal_model == "velocity_lag":
            deadband = max(float(model_cfg.get("velocity_lag_deadband", 0.0)), 0.0)
            throttle_eff = float(throttle)
            if deadband > 0.0:
                throttle_eff = float(
                    np.sign(throttle) * max(abs(float(throttle)) - deadband, 0.0)
                )
            v_dot = (
                -(1.0 / float(model_cfg["velocity_lag_tau"])) * v
                + (float(model_cfg["velocity_gain"]) / float(model_cfg["velocity_lag_tau"]))
                * throttle_eff
            )
            v_new = v + v_dot * dt
            a_new = v_dot
        elif longitudinal_model == "velocity_lag_lookup":
            throttle_breakpoints = np.asarray(
                model_cfg.get("velocity_lag_lookup_throttle_breakpoints", []),
                dtype=float,
            ).reshape(-1)
            velocity_breakpoints = np.asarray(
                model_cfg.get("velocity_lag_lookup_velocity_breakpoints", []),
                dtype=float,
            ).reshape(-1)
            if (
                throttle_breakpoints.size >= 2
                and throttle_breakpoints.size == velocity_breakpoints.size
            ):
                v_ss = float(
                    np.interp(float(throttle), throttle_breakpoints, velocity_breakpoints)
                )
            else:
                deadband = max(float(model_cfg.get("velocity_lag_deadband", 0.0)), 0.0)
                throttle_eff = float(throttle)
                if deadband > 0.0:
                    throttle_eff = float(
                        np.sign(throttle) * max(abs(float(throttle)) - deadband, 0.0)
                    )
                v_ss = float(model_cfg["velocity_gain"]) * throttle_eff
            tau_lookup = float(
                model_cfg.get("velocity_lag_lookup_tau", model_cfg["velocity_lag_tau"])
            )
            v_dot = (v_ss - v) / max(tau_lookup, 1e-6)
            v_new = v + v_dot * dt
            a_new = v_dot
        elif longitudinal_model == "velocity_command":
            v_cmd = throttle
            tau = float(model_cfg["velocity_command_tau"])
            v_dot = (v_cmd - v) / tau
            v_new = v + v_dot * dt
            a_new = v_dot
        elif longitudinal_model == "acceleration_lag":
            a_new = a + dt * (
                -(1.0 / float(model_cfg["accel_lag_tau"])) * a
                + (float(model_cfg["accel_lag_gain"]) / float(model_cfg["accel_lag_tau"]))
                * throttle
            )
            v_new = v + a_new * dt
        elif longitudinal_model == "simple_acceleration":
            a_new = throttle
            v_new = v + a_new * dt
        else:
            v_new = v
            a_new = 0.0

        return np.array([x_new, y_new, theta_new, v_new, a_new])

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap an angle to [-pi, pi]."""
        return float(np.arctan2(np.sin(angle), np.cos(angle)))

    def _state_residual(self, measurement: np.ndarray, reference: np.ndarray) -> np.ndarray:
        """Return measurement-reference residual with circular heading error."""
        residual = np.asarray(measurement, dtype=float) - np.asarray(reference, dtype=float)
        if residual.shape[0] > 2:
            residual[2] = self._wrap_angle(float(residual[2]))
        return residual

    def _apply_output_low_pass_filter(
        self,
        previous_state: np.ndarray,
        new_state: np.ndarray,
        target_id: int = -1,
        alpha_override: Optional[float] = None,
    ) -> np.ndarray:
        """Low-pass filter the final target estimate using the previous output."""
        if not getattr(self, "enable_output_low_pass", False):
            return np.asarray(new_state, dtype=float).copy()

        if alpha_override is None:
            alpha = float(
                np.clip(getattr(self, "output_low_pass_alpha", 1.0), 0.0, 1.0)
            )
        else:
            alpha = float(np.clip(alpha_override, 0.0, 1.0))
        if alpha >= 1.0:
            return np.asarray(new_state, dtype=float).copy()

        prev = np.asarray(previous_state, dtype=float)
        new = np.asarray(new_state, dtype=float)
        filtered = (1.0 - alpha) * prev + alpha * new

        if filtered.shape[0] > 2:
            theta_prev = float(prev[2])
            theta_new = float(new[2])
            theta_delta = self._wrap_angle(theta_new - theta_prev)
            filtered[2] = self._wrap_angle(theta_prev + alpha * theta_delta)

        return self._apply_state_constraints(filtered, target_id=target_id)

    def _apply_state_constraints(
        self, state: np.ndarray, target_id: int = -1
    ) -> np.ndarray:
        """Apply physical constraints to state."""
        model_cfg = self._get_vehicle_model_config(target_id)
        constrained = state.copy()
        constrained[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))
        max_v = max(float(model_cfg.get("max_velocity", 2.0)), 1e-6)
        max_a = max(float(model_cfg.get("max_acceleration", 5.0)), 1e-6)
        constrained[3] = np.clip(state[3], -max_v, max_v)
        if len(state) > 4:
            constrained[4] = np.clip(state[4], -max_a, max_a)
        return constrained

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
    def _get_direct_trust_delay_log_data(self) -> Dict[int, Dict[str, float]]:
        """Return direct-trust delay diagnostics for CSV logging."""
        delay_steps = int(
            max(getattr(self, "direct_trust_application_delay_steps", 0), 0)
        )
        log_data: Dict[int, Dict[str, float]] = {}
        for target_id, state in self._direct_trust_delay_state.items():
            log_data[int(target_id)] = {
                "active": float(state.get("active", 0.0)),
                "bad_count": float(state.get("bad_count", 0.0)),
                "delay_steps": float(delay_steps),
                "clean_local_trust": float(state.get("clean_local_trust", np.nan)),
            }
        return log_data

    def _get_rollback_trigger_delay_log_data(self) -> Dict[int, Dict[str, float]]:
        """Return rollback-trigger delay diagnostics for CSV logging."""
        delay_steps = int(max(getattr(self, "rollback_trigger_delay_steps", 0), 0))
        return {
            int(target_id): {
                "count": float(count),
                "delay_steps": float(delay_steps),
            }
            for target_id, count in self._rollback_trigger_delay_state.items()
        }

    def _log_update(
        self,
        trust_scores: Dict[int, float],
        weight_result: WeightResult,
        control: np.ndarray,
        target_confidence: Dict[int, float],
        target_prediction_mode: Dict[int, bool],
        current_time_ns: int,
        target_components: Optional[Dict[int, Dict]] = None,
        rollback_status: Optional[Dict[str, object]] = None,
        consensus_estimates: Optional[Dict[int, Dict[str, float]]] = None,
        post_prediction_estimates: Optional[Dict[int, Dict[str, float]]] = None,
        clean_reference_estimates: Optional[Dict[int, Dict[str, float]]] = None,
        prediction_debugs: Optional[Dict[int, Dict[str, float]]] = None,
    ) -> None:
        """Build and emit per-step trust/weight log data."""
        target_components = target_components or {}
        final_target_weights: Dict[int, Dict] = {}
        for target_id, components in target_components.items():
            weights = components.get("weights", {}) if isinstance(components, dict) else {}
            neighbor_weights = weights.get("neighbors", {}) if isinstance(weights, dict) else {}
            final_target_weights[int(target_id)] = {
                "w0": float(weights.get("w0", 0.0)) if isinstance(weights, dict) else 0.0,
                "w_self": float(weights.get("w_self", 0.0)) if isinstance(weights, dict) else 0.0,
                "neighbors": {
                    int(neighbor_id): float(weight)
                    for neighbor_id, weight in neighbor_weights.items()
                },
            }

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
            "host_throttle": float(control[1]) if len(control) > 1 else float("nan"),
            "controller": dict(self.controller_debug_snapshot),
            "neighbors": {},
            "final_target_weights": final_target_weights,
            "consensus_estimates": consensus_estimates or {},
            "post_prediction_estimates": post_prediction_estimates or {},
            "clean_reference_estimates": clean_reference_estimates or {},
            "prediction_debugs": prediction_debugs or {},
            "direct_trust_delay": self._get_direct_trust_delay_log_data(),
            "rollback_trigger_delay": self._get_rollback_trigger_delay_log_data(),
            "v2v_attack": self._get_v2v_attack_log_data(current_time_ns),
            "rollback": rollback_status or self.rollback.get_status(),
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

    def get_current_weights(self) -> np.ndarray:
        """Get current consensus weights."""
        return self.weight_module.get_weights_array()

    def _restore_fleet_states_from_latest_direct_states(self) -> List[int]:
        """Seed external estimates from latest clean/direct V2V state after attack disable."""
        restored: List[int] = []
        for target_id in range(self.fleet_size):
            target_id = int(target_id)
            if target_id == self.vehicle_id:
                continue

            history = self.received_clean_local_states.get(target_id)
            if not history:
                history = self.received_local_states.get(target_id)
            if not history:
                continue

            try:
                _, state = history[-1]
                arr = np.asarray(state, dtype=float).reshape(-1)
                if arr.size < self.state_dim:
                    arr = np.pad(arr, (0, self.state_dim - arr.size), mode="constant")
                restored_state = self._apply_state_constraints(
                    arr[: self.state_dim], target_id=target_id
                )
                self._ensure_fleet_capacity(target_id)
                self.fleet_states[:, target_id] = restored_state
                restored.append(target_id)
            except Exception as exc:
                if self.logger:
                    self.logger.logger.debug(
                        "Skipping attack-recovery state restore for target %s: %s",
                        target_id,
                        exc,
                    )
        return restored

    def reset_attack_recovery_state(self) -> None:
        """
        Clear trust/rollback memory and re-anchor estimates after attack restore.

        The latest clean V2V state is used when available so stale attack-era
        fleet estimates do not immediately re-trigger global rollback.
        """
        restored_targets = self._restore_fleet_states_from_latest_direct_states()
        self.trust_model.reset()
        self.weight_module.reset()
        self.current_weight_result = None
        self.generalized_trust_vector = {self.vehicle_id: 1.0}
        self.stats = self._make_default_stats()
        self.rollback.reset()
        self._rollback_trusted_state_history.clear()
        self._rollback_trusted_relative_anchor_history.clear()
        self._direct_recovery_state.clear()
        self._direct_trust_delay_state.clear()
        self._rollback_trigger_delay_state.clear()
        self._ext_cache.clear()
        self._reset_startup_weight_tracking()

        if self.logger:
            self.logger.logger.info(
                "Trust/rollback recovery state reset after manual V2V attack restore; restored_targets=%s",
                restored_targets,
            )

    def get_statistics(self) -> Dict[str, object]:
        """Get estimator statistics."""
        data = self.stats.copy()
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
        self.controller_debug_snapshot = {}
        self.received_clean_local_states.clear()
        self._ext_cache.clear()
        self.current_weight_result = None
        self.generalized_trust_vector = {self.vehicle_id: 1.0}
        self.stats = self._make_default_stats()
        self.rollback.reset()
        self._rollback_trusted_state_history.clear()
        self._rollback_trusted_relative_anchor_history.clear()
        self._direct_recovery_state.clear()
        self._direct_trust_delay_state.clear()
        self._rollback_trigger_delay_state.clear()
        self._received_control_inputs.clear()
        self._init_runtime_tracking()

    def __del__(self):
        if hasattr(self, "trust_weight_logger"):
            self.trust_weight_logger.stop()




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
    else:
        raise ValueError(
            f"Unknown trust-based estimator type: {estimator_type}. "
            f"Available: ['trust_consensus', 'trust_kalman']"
        )
