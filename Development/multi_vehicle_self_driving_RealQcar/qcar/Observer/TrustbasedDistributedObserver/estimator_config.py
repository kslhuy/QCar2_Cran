"""
Configuration helpers for the trust-based fleet estimator.

The runtime loader may pass a flattened observer config, while direct tests or
tools may pass the original YAML shape. These helpers support both layouts and
keep fallback defaults out of the estimator implementation.
"""

from dataclasses import dataclass
from typing import Any, Dict, Mapping

import numpy as np


def dict_section(cfg: Mapping[str, Any], key: str) -> Dict[str, Any]:
    """Return a nested dictionary value or an empty dict."""
    if not isinstance(cfg, Mapping):
        return {}
    value = cfg.get(key, {})
    return dict(value) if isinstance(value, Mapping) else {}


def observer_value(config: Mapping[str, Any], key: str, default: Any = None) -> Any:
    """Read an observer setting from flat config first, then `observer:`."""
    if isinstance(config, Mapping) and key in config:
        return config[key]
    observer_cfg = dict_section(config, "observer")
    return observer_cfg.get(key, default)


def as_float(value: Any, default: float) -> float:
    """Convert to float with a stable fallback."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def as_bool(value: Any, default: bool = False) -> bool:
    """Convert common YAML/string boolean forms to bool."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    if value is None:
        return bool(default)
    return bool(value)


def clipped_float(
    value: Any,
    default: float,
    min_value: float = 0.0,
    max_value: float = 1.0,
) -> float:
    """Convert to float and clip into a closed interval."""
    return float(np.clip(as_float(value, default), min_value, max_value))


def nonnegative_int(value: Any, default: int = 0) -> int:
    """Convert to an int clamped at zero."""
    try:
        return max(int(value), 0)
    except (TypeError, ValueError):
        return max(int(default), 0)


def positive_int(value: Any, default: int = 1) -> int:
    """Convert to an int clamped at one."""
    try:
        return max(int(value), 1)
    except (TypeError, ValueError):
        return max(int(default), 1)


def normalize_dynamics_prediction_mode(value: Any) -> str:
    """Normalize aliases used by YAML, scripts, and older experiments."""
    mode = str(value or "model").strip().lower()
    aliases = {
        "model": "model",
        "current": "model",
        "current_model": "model",
        "default": "model",
        "kinematic": "model",
        "clean_data": "clean_data",
        "clean-data": "clean_data",
        "clean": "clean_data",
        "pure_prediction_self": "clean_data",
        "pure-prediction-self": "clean_data",
        "pure_self_prediction": "clean_data",
        "pure-self-prediction": "clean_data",
        "mixed_clean_data": "mixed_clean_data",
        "mixed-clean-data": "mixed_clean_data",
        "mixed_clean": "mixed_clean_data",
        "mixed-clean": "mixed_clean_data",
        "relative_host_anchor_mixed": "relative_host_anchor_mixed",
        "relative-host-anchor-mixed": "relative_host_anchor_mixed",
        "host_anchor_mixed": "relative_host_anchor_mixed",
        "host-anchor-mixed": "relative_host_anchor_mixed",
        "dead_reckoning": "dead_reckoning",
        "dead_reckon": "dead_reckoning",
        "dead-reckoning": "dead_reckoning",
        "dead-reckon": "dead_reckoning",
        "dr": "dead_reckoning",
        "none": "none",
        "no_prediction": "none",
        "no-prediction": "none",
        "disabled": "none",
        "disable": "none",
        "off": "none",
    }
    return aliases.get(mode, "model")


@dataclass(frozen=True)
class ObserverSettings:
    """Resolved observer-level scalar settings."""

    observer_gain: float = 0.1
    consensus_gain: float = 0.2
    attack_mitigation_enabled: bool = True
    turn_steering_threshold: float = 0.1

    @classmethod
    def from_config(
        cls,
        config: Mapping[str, Any],
        trust_config: Mapping[str, Any],
    ) -> "ObserverSettings":
        trust_config = trust_config if isinstance(trust_config, Mapping) else {}
        return cls(
            observer_gain=as_float(observer_value(config, "observer_gain", 0.1), 0.1),
            consensus_gain=as_float(observer_value(config, "consensus_gain", 0.2), 0.2),
            attack_mitigation_enabled=as_bool(
                observer_value(config, "attack_mitigation", True), True
            ),
            turn_steering_threshold=as_float(
                observer_value(
                    config,
                    "turn_steering_threshold",
                    trust_config.get("turn_steering_threshold", 0.1),
                ),
                0.1,
            ),
        )


@dataclass(frozen=True)
class PredictionSettings:
    """Resolved prediction and output-filter settings."""

    enable_output_low_pass: bool = False
    output_low_pass_alpha: float = 1.0
    attack_output_low_pass_alpha: float = 1.0
    force_clean_pose_anchor: bool = True
    relative_host_anchor_clean_theta_weight: float = 0.8
    relative_host_anchor_host_theta_weight: float = 0.2
    relative_host_anchor_target_velocity_weight: float = 0.1
    relative_host_anchor_host_velocity_weight: float = 0.9
    relative_host_anchor_target_acceleration_weight: float = 0.1
    relative_host_anchor_host_acceleration_weight: float = 0.9
    relative_host_anchor_use_bearing: bool = True
    direct_recovery_enabled: bool = True
    direct_recovery_hold_steps: int = 15
    direct_recovery_required_good_steps: int = 10
    direct_recovery_ramp_steps: int = 30
    direct_recovery_min_local_trust: float = 0.5
    dynamics_prediction_mode: str = "model"

    @classmethod
    def from_config(
        cls,
        config: Mapping[str, Any],
        vehicle_config: Mapping[str, Any],
        trust_threshold: float,
    ) -> "PredictionSettings":
        vehicle_config = vehicle_config if isinstance(vehicle_config, Mapping) else {}
        output_alpha = clipped_float(
            observer_value(config, "output_low_pass_alpha", 1.0), 1.0
        )
        raw_mode = observer_value(config, "dynamics_prediction_mode")
        if raw_mode is None:
            raw_mode = observer_value(
                config,
                "prediction_mode",
                vehicle_config.get("dynamics_prediction_mode", "model"),
            )

        return cls(
            enable_output_low_pass=as_bool(
                observer_value(config, "enable_output_low_pass", False), False
            ),
            output_low_pass_alpha=output_alpha,
            attack_output_low_pass_alpha=clipped_float(
                observer_value(config, "attack_output_low_pass_alpha", output_alpha),
                output_alpha,
            ),
            force_clean_pose_anchor=as_bool(
                observer_value(config, "force_clean_pose_anchor", True), True
            ),
            relative_host_anchor_clean_theta_weight=clipped_float(
                observer_value(config, "relative_host_anchor_clean_theta_weight", 0.8),
                0.8,
            ),
            relative_host_anchor_host_theta_weight=clipped_float(
                observer_value(config, "relative_host_anchor_host_theta_weight", 0.2),
                0.2,
            ),
            relative_host_anchor_target_velocity_weight=clipped_float(
                observer_value(
                    config, "relative_host_anchor_target_velocity_weight", 0.1
                ),
                0.1,
            ),
            relative_host_anchor_host_velocity_weight=clipped_float(
                observer_value(config, "relative_host_anchor_host_velocity_weight", 0.9),
                0.9,
            ),
            relative_host_anchor_target_acceleration_weight=clipped_float(
                observer_value(
                    config, "relative_host_anchor_target_acceleration_weight", 0.1
                ),
                0.1,
            ),
            relative_host_anchor_host_acceleration_weight=clipped_float(
                observer_value(
                    config, "relative_host_anchor_host_acceleration_weight", 0.9
                ),
                0.9,
            ),
            relative_host_anchor_use_bearing=as_bool(
                observer_value(config, "relative_host_anchor_use_bearing", True), True
            ),
            direct_recovery_enabled=as_bool(
                observer_value(config, "direct_recovery_enabled", True), True
            ),
            direct_recovery_hold_steps=nonnegative_int(
                observer_value(config, "direct_recovery_hold_steps", 15), 15
            ),
            direct_recovery_required_good_steps=nonnegative_int(
                observer_value(config, "direct_recovery_required_good_steps", 10), 10
            ),
            direct_recovery_ramp_steps=nonnegative_int(
                observer_value(config, "direct_recovery_ramp_steps", 30), 30
            ),
            direct_recovery_min_local_trust=clipped_float(
                observer_value(
                    config, "direct_recovery_min_local_trust", trust_threshold
                ),
                trust_threshold,
            ),
            dynamics_prediction_mode=normalize_dynamics_prediction_mode(raw_mode),
        )


@dataclass(frozen=True)
class RollbackSettings:
    """Resolved contamination rollback settings."""

    enabled: bool = False
    window_size: int = 15
    trusted_state_guard_steps: int = 0
    trusted_state_history_size: int = 15
    on_final_trust: bool = True
    on_local_est_check: bool = True
    on_global_est_check: bool = False

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> "RollbackSettings":
        window_size = positive_int(
            observer_value(config, "rollback_window_size", 15), 15
        )
        guard_steps = nonnegative_int(
            observer_value(config, "rollback_trusted_state_guard_steps", 0), 0
        )
        history_size = positive_int(
            observer_value(
                config, "rollback_trusted_state_history_size", window_size
            ),
            window_size,
        )
        return cls(
            enabled=as_bool(observer_value(config, "rollback_enabled", False), False),
            window_size=window_size,
            trusted_state_guard_steps=guard_steps,
            trusted_state_history_size=max(history_size, guard_steps + 1, 1),
            on_final_trust=as_bool(
                observer_value(config, "rollback_on_final_trust", True), True
            ),
            on_local_est_check=as_bool(
                observer_value(config, "rollback_on_local_est_check", True), True
            ),
            on_global_est_check=as_bool(
                observer_value(config, "rollback_on_global_est_check", False), False
            ),
        )


def normalize_vehicle_model_config(
    cfg: Mapping[str, Any],
    default_prediction_mode: str = "model",
) -> Dict[str, Any]:
    """Normalize a vehicle model dictionary used by prediction."""
    cfg = cfg if isinstance(cfg, Mapping) else {}
    vehicle_type = str(cfg.get("vehicle_type", cfg.get("type", "Qcar")) or "Qcar")
    vehicle_type_norm = "Limo" if vehicle_type.strip().lower() == "limo" else "Qcar"

    velocity_lag = dict_section(cfg, "velocity_lag_model")
    velocity_lag_lookup = dict_section(cfg, "velocity_lag_lookup_model")
    accel_lag = dict_section(cfg, "accel_lag_model")

    longitudinal_model = str(cfg.get("longitudinal_model", "") or "").strip().lower()
    if not longitudinal_model:
        if vehicle_type_norm == "Limo":
            longitudinal_model = "velocity_command"
        elif as_bool(velocity_lag_lookup.get("enabled"), False):
            longitudinal_model = "velocity_lag_lookup"
        elif as_bool(velocity_lag.get("enabled"), False):
            longitudinal_model = "velocity_lag"
        elif as_bool(accel_lag.get("enabled"), False):
            longitudinal_model = "acceleration_lag"
        else:
            longitudinal_model = "constant_velocity"

    return {
        "vehicle_type": vehicle_type_norm,
        "dynamics_prediction_mode": normalize_dynamics_prediction_mode(
            cfg.get(
                "dynamics_prediction_mode",
                cfg.get("prediction_mode", default_prediction_mode),
            )
        ),
        "wheelbase": as_float(cfg.get("wheelbase"), 0.256),
        "max_velocity": as_float(cfg.get("max_velocity"), 2.0),
        "max_acceleration": as_float(cfg.get("max_acceleration"), 5.0),
        "max_steering": as_float(cfg.get("max_steering"), 0.5),
        "longitudinal_model": longitudinal_model,
        "velocity_lag_tau": max(
            as_float(velocity_lag.get("tau"), cfg.get("velocity_lag_tau", 0.301)),
            1e-6,
        ),
        "velocity_gain": as_float(
            velocity_lag.get("velocity_gain"), cfg.get("velocity_gain", 6.598)
        ),
        "velocity_lag_deadband": max(
            as_float(
                velocity_lag.get("throttle_deadband"),
                cfg.get("velocity_lag_deadband", 0.0),
            ),
            0.0,
        ),
        "velocity_lag_lookup_tau": max(
            as_float(
                velocity_lag_lookup.get("tau"),
                cfg.get("velocity_lag_lookup_tau", 0.301),
            ),
            1e-6,
        ),
        "velocity_lag_lookup_throttle_breakpoints": [
            float(v) for v in velocity_lag_lookup.get("throttle_breakpoints", [])
        ],
        "velocity_lag_lookup_velocity_breakpoints": [
            float(v)
            for v in velocity_lag_lookup.get("steady_state_velocity_breakpoints", [])
        ],
        "velocity_command_tau": max(
            as_float(cfg.get("velocity_command_tau"), velocity_lag.get("tau", 0.301)),
            1e-6,
        ),
        "accel_lag_tau": max(
            as_float(accel_lag.get("tau"), cfg.get("accel_lag_tau", 0.318)),
            1e-6,
        ),
        "accel_lag_gain": as_float(
            accel_lag.get("input_gain"), cfg.get("accel_lag_gain", 1.0)
        ),
        "allow_host_control_fallback": as_bool(
            cfg.get("allow_host_control_fallback"), False
        ),
    }


def load_vehicle_model_overrides(
    raw_models: Any,
    default_vehicle_config: Mapping[str, Any],
    default_prediction_mode: str = "model",
) -> Dict[int, Dict[str, Any]]:
    """Load optional per-vehicle model overrides from config."""
    if not isinstance(raw_models, Mapping):
        return {}

    default_vehicle_config = (
        default_vehicle_config if isinstance(default_vehicle_config, Mapping) else {}
    )
    overrides: Dict[int, Dict[str, Any]] = {}
    for raw_id, raw_cfg in raw_models.items():
        try:
            vehicle_id = int(raw_id)
        except (TypeError, ValueError):
            continue
        if not isinstance(raw_cfg, Mapping):
            continue

        merged_raw = dict(default_vehicle_config)
        raw_vehicle_type = raw_cfg.get("vehicle_type", raw_cfg.get("type"))
        default_vehicle_type = default_vehicle_config.get(
            "vehicle_type", default_vehicle_config.get("type", "Qcar")
        )
        type_changed = (
            raw_vehicle_type is not None
            and str(raw_vehicle_type).strip().lower()
            != str(default_vehicle_type).strip().lower()
        )
        if type_changed and "longitudinal_model" not in raw_cfg:
            merged_raw.pop("longitudinal_model", None)

        for key, value in raw_cfg.items():
            if isinstance(value, Mapping) and isinstance(merged_raw.get(key), Mapping):
                nested = dict(merged_raw[key])
                nested.update(value)
                merged_raw[key] = nested
            else:
                merged_raw[key] = value

        normalized = normalize_vehicle_model_config(
            merged_raw, default_prediction_mode
        )
        normalized["_explicit_prediction_mode"] = bool(
            "dynamics_prediction_mode" in raw_cfg or "prediction_mode" in raw_cfg
        )
        overrides[vehicle_id] = normalized
    return overrides


def vehicle_model_for_target(
    default_vehicle_model: Mapping[str, Any],
    vehicle_model_overrides: Mapping[int, Dict[str, Any]],
    target_id: int,
    global_prediction_mode: str,
) -> Dict[str, Any]:
    """Return the effective vehicle model for one target."""
    base_cfg = dict(default_vehicle_model)
    base_cfg["dynamics_prediction_mode"] = global_prediction_mode

    override_cfg = vehicle_model_overrides.get(int(target_id))
    if override_cfg is None:
        return base_cfg

    merged_cfg = dict(base_cfg)
    merged_cfg.update(override_cfg)
    if not bool(override_cfg.get("_explicit_prediction_mode", False)):
        merged_cfg["dynamics_prediction_mode"] = global_prediction_mode
    merged_cfg.pop("_explicit_prediction_mode", None)
    return merged_cfg
