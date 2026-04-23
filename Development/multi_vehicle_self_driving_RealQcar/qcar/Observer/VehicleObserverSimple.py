"""
Vehicle Observer Manager - Coordinates local and fleet state estimation

This is a manager class that:
1. Controls update rates for local and fleet observers
2. Delegates state estimation to specialized estimators (local_state_estimators, fleet_state_estimators)
3. Provides unified interface for accessing state data
4. Manages sensor data reading and caching

Architecture:
- LocalStateEstimator: Handles local vehicle state (EKF, Luenberger, etc.)
- FleetStateEstimator: Handles distributed fleet estimation (Consensus, Distributed Kalman, etc.)
- VehicleObserver: Manager that coordinates both and provides data access
"""

import numpy as np
import threading
import time
import yaml
import os
import copy
from typing import Any, Dict, List, Optional, Tuple
from collections import defaultdict

from Observer.local_state_estimators import (
    LocalEstimatorFactory,
    LocalStateEstimatorBase,
)
from Observer.fleet_state_estimators import (
    FleetEstimatorFactory,
    FleetStateEstimatorBase,
)
from Observer.relative_state_estimators import (
    RelativeEstimatorFactory,
    RelativeStateEstimatorBase,
)
from Observer.estimation_scopes import (
    LOCAL_RECORD_COLUMNS,
    ScopeDataRecorder,
    build_fleet_record_columns,
)
from Observer.Obs_6d_on_track_sysID.vy_kalman_filter import LateralVelocityEKF


class VehicleObserver:
    """
    Vehicle Observer Manager

    Coordinates local and fleet state estimation with pluggable algorithms.
    Provides unified interface for data access to other systems.
    """

    def __init__(
        self,
        vehicle_id: int,
        config=None,
        logger=None,
        local_estimator_type: str = "ekf",
        fleet_estimator_type: str = "consensus",
    ):
        """
        Initialize the Vehicle Observer Manager.

        Args:
            vehicle_id: ID of the host vehicle
            config: Configuration object
            logger: Logger instance
            local_estimator_type: Type of local state estimator ('ekf', 'luenberger', 'dead_reckoning')
            fleet_estimator_type: Type of fleet state estimator ('consensus', 'distributed_kalman')

        Note:
            - Fleet size is initially 1 (just this vehicle)
            - Fleet will be reinitialized when V2V activates via reinitialize_fleet_estimation()
            - State estimators are created by factories and can be swapped at runtime
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = max(
            vehicle_id + 1, 1
        )  # At least large enough for this vehicle
        self.config = config or {}
        self.vehicle_logger = logger

        # Start with types provided by constructor (from external config)
        self.fleet_estimator_type = fleet_estimator_type
        self.local_estimator_type = local_estimator_type

        # Load fleet estimator defaults from config file (acts as global defaults)
        self.fleet_config_defaults = {}
        try:
            config_path = os.path.join(
                os.path.dirname(__file__), "config_fleet_estimators.yaml"
            )
            trust_child_config_path = os.path.join(
                os.path.dirname(__file__),
                "TrustbasedDistributedObserver",
                "config_trust_estimator.yaml",
            )
            with open(config_path, "r") as f:
                loaded = yaml.safe_load(f) or {}

            # Merge trust child config into trust estimator sections.
            try:
                with open(trust_child_config_path, "r") as f:
                    trust_child_loaded = yaml.safe_load(f) or {}
                loaded = self._merge_trust_child_into_fleet_config(
                    loaded, trust_child_loaded
                )
            except Exception as trust_cfg_error:
                if self.vehicle_logger:
                    self.vehicle_logger.log_warning(
                        f"Trust child config not applied: {trust_cfg_error}"
                    )

            if not isinstance(loaded, dict):
                loaded = {}

            self.fleet_config_defaults = loaded.get("fleet", {})
            selected_fleet_type = loaded.get("fleet_estimator_type")
            if selected_fleet_type:
                self.fleet_estimator_type = selected_fleet_type

            # Load fleet recording config
            self.fleet_recording_enabled = loaded.get("enable_recording", False)
            self.fleet_recording_overwrite = loaded.get(
                "recording_overwrite", False
            )
            self.fleet_recording_max_vehicles = loaded.get(
                "max_record_vehicles", 5
            )

            self.vehicle_logger.logger.info(
                f"Loaded fleet estimator config: {self.fleet_estimator_type}"
            )
        except Exception as e:
            if self.vehicle_logger:
                self.vehicle_logger.log_warning(
                    f"Failed to load fleet config file: {e}"
                )

        # Load local estimator defaults from config file (acts as global defaults)
        self.local_config_defaults = {}
        try:
            config_path = os.path.join(
                os.path.dirname(__file__), "config_local_estimators.yaml"
            )
            with open(config_path, "r") as f:
                loaded = yaml.safe_load(f)
                self.local_config_defaults = loaded.get("local", {})
                self.local_estimator_type = loaded.get("local_estimator_type")

                # Load local recording config
                self.local_recording_enabled = loaded.get("enable_recording", False)
                self.local_recording_overwrite = loaded.get(
                    "recording_overwrite", False
                )

                self.vehicle_logger.logger.info(
                    f"Loaded local estimator config: {self.local_estimator_type}"
                )
        except Exception as e:
            if self.vehicle_logger:
                self.vehicle_logger.log_warning(
                    f"Failed to load local config file: {e}"
                )

        # State dimensions: [x, y, theta, v, a] - position, orientation, velocity, acceleration
        self.state_dim = 5

        # Observer configuration (external per-vehicle overrides)
        self.observer_config = self._get_observer_config()
        self.vehicle_geometry_config = self._get_vehicle_geometry_config()

        # Load relative estimator defaults
        self.relative_config_defaults = {}
        self.enable_relative = False
        self.relative_estimator_type = "sa_acc_uio"
        self.relative_recording_enabled = True
        self.relative_recording_overwrite = True
        self.relative_recording_output_dir = None
        try:
            config_path = os.path.join(
                os.path.dirname(__file__), "config_relative_estimators.yaml"
            )
            with open(config_path, "r") as f:
                loaded = yaml.safe_load(f)
                self.relative_config_defaults = loaded.get("relative", {})
                self.relative_estimator_type = loaded.get(
                    "relative_estimator_type", "sa_acc_uio"
                )
                self.enable_relative = self._config_bool(
                    loaded.get("enable_relative", False), False
                )
                self.relative_recording_enabled = self._config_bool(
                    loaded.get("enable_recording", True), True
                )
                self.relative_recording_overwrite = self._config_bool(
                    loaded.get("recording_overwrite", True), True
                )
                self.relative_recording_output_dir = loaded.get(
                    "recording_output_dir", None
                )

                self.vehicle_logger.logger.info(
                    f"Loaded relative estimator config: {self.relative_estimator_type}, Enabled: {self.enable_relative}"
                )
        except Exception as e:
            if self.vehicle_logger:
                self.vehicle_logger.log_warning(
                    f"Failed to load relative config file: {e}"
                )

        # ===== Local State Estimator (pluggable) =====
        self.local_estimator: Optional[LocalStateEstimatorBase] = None
        # Will be initialized later via initialize_local_estimator()

        # ===== Fleet State Estimator (pluggable) =====
        self.fleet_estimator: Optional[FleetStateEstimatorBase] = None
        # Fleet estimator will be created when V2V is activated (not at initialization)
        # This saves resources and ensures clean state when V2V starts
        self.v2v_active = False  # Track if V2V is active
        self._v2v_time_reference: Optional[Dict[str, Any]] = None
        self._v2v_vehicle_manifest: Dict[int, Dict[str, Any]] = {}

        # ===== Relative State Estimator (pluggable) =====
        self.relative_estimator: Optional[RelativeStateEstimatorBase] = None
        # Will be initialized if enabled

        # ===== State Cache (for quick access) =====
        self.local_state = np.zeros(self.state_dim)
        self.position = np.zeros(3)  # [x, y, theta]
        self.velocity = 0.0
        self.gps_valid = False  # GPS validity flag
        self.gps_valid_hold_seconds = 0.25
        self.gps_valid_hold_mode = "adaptive"
        self.gps_valid_hold_scale = 1.15
        self.gps_valid_hold_min_seconds = 0.08
        self.gps_valid_hold_max_seconds = 0.35
        self._last_gps_sample_time = 0.0
        self._gps_sample_period_estimate = 0.0

        # Fleet states (managed by fleet_estimator but cached here)
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))

        # Relative states
        self.relative_state = np.zeros(
            4
        )  # Default size [delta, delta_dot, delta_ddot, f_c]
        self.relative_target_id: Optional[int] = None

        # ===== Sensor Data Cache =====

        self.sensor_data = {
            "motor_tach_raw": 0.0,
            "motor_tach": 0.0,
            "battery_voltage": 0.0,
            "gyro_z": 0.0,
            "accelerometer_raw": np.zeros(3),
            "accelerometer": np.zeros(3),
            "accel_magnitude_raw": 0.0,
            "accel_magnitude": 0.0,
            "timestamp": 0.0,
            "gps_valid": False,
            "gps_fresh": False,
            "gps_has_fix": False,
            "gps_position": np.zeros(3),  # [x, y, theta]
            "gps_age": float("inf"),
            "gps_hold_window": 0.0,
            "relative_measurements_by_target": {},
        }
        # For derivative estimation when only distance is provided.
        self._last_relative_distance_by_target: Dict[int, Tuple[float, float]] = {}

        # ===== Control and Dynamics Cache =====
        # self.last_velocity = 0.0
        self.acceleration_magnitude = 0.0
        self.v_lpf_alpha = 1.0
        self._filtered_motor_tach = 0.0
        self._motor_tach_filter_initialized = False
        self.accel_ema_alpha = 1.0
        self._filtered_accelerometer = np.zeros(3)
        self._accel_filter_initialized = False
        self.control_input = {"steering": 0.0, "throttle": 0.0}
        # Lateral velocity fallback estimate for SysID when 6D observer state is unavailable.
        self._vy_estimate = 0.0
        self._vy_est_last_time = 0.0
        self.vy_ekf = None

        # ===== GPS Reference =====
        self.gps = None  # Will be set during initialize_local_estimator

        # ===== Timing Control =====
        self.local_observer_rate = self.observer_config.get("observer_rate", 100)
        self.fleet_observer_rate = self.observer_config.get("fleet_observer_rate", 50)
        self._last_fleet_observer_time = 0.0

        # ===== Thread Safety =====
        self.lock = threading.RLock()

        # ===== Data Recorders =====
        self.local_recorder = None
        self.fleet_recorder = None
        self._init_recorders()

        self.vehicle_logger.logger.info(
            # f"VehicleObserver initialized: vehicle_id={vehicle_id}, "
            f"Observer config: {self.config.get('observer', {}) if isinstance(self.config, dict) else getattr(self.config, 'observer', {})}, "
            # f"local_estimator={local_estimator_type}, fleet_estimator={fleet_estimator_type}"
        )
        try:
            common_cfg = self.local_config_defaults.get("common", {})
            self.v_lpf_alpha = float(
                np.clip(float(common_cfg.get("v_lpf_alpha", 1.0)), 0.0, 1.0)
            )
            self.accel_ema_alpha = float(
                np.clip(float(common_cfg.get("accel_ema_alpha", 1.0)), 0.0, 1.0)
            )
        except (TypeError, ValueError):
            self.v_lpf_alpha = 1.0
            self.accel_ema_alpha = 1.0
        try:
            gps_hold_cfg = self.observer_config.get(
                "gps_valid_hold_seconds",
                self.local_config_defaults.get("common", {}).get(
                    "gps_valid_hold_seconds", 0.25
                ),
            )
            self.gps_valid_hold_seconds = max(0.0, float(gps_hold_cfg))
        except (TypeError, ValueError):
            self.gps_valid_hold_seconds = 0.25
        try:
            self.gps_valid_hold_mode = str(
                self.observer_config.get(
                    "gps_valid_hold_mode",
                    self.local_config_defaults.get("common", {}).get(
                        "gps_valid_hold_mode", "adaptive"
                    ),
                )
            ).strip().lower() or "adaptive"
        except Exception:
            self.gps_valid_hold_mode = "adaptive"
        try:
            self.gps_valid_hold_scale = max(
                0.5,
                float(
                    self.observer_config.get(
                        "gps_valid_hold_scale",
                        self.local_config_defaults.get("common", {}).get(
                            "gps_valid_hold_scale", 1.15
                        ),
                    )
                ),
            )
        except (TypeError, ValueError):
            self.gps_valid_hold_scale = 1.15
        try:
            self.gps_valid_hold_min_seconds = max(
                0.0,
                float(
                    self.observer_config.get(
                        "gps_valid_hold_min_seconds",
                        self.local_config_defaults.get("common", {}).get(
                            "gps_valid_hold_min_seconds", 0.08
                        ),
                    )
                ),
            )
        except (TypeError, ValueError):
            self.gps_valid_hold_min_seconds = 0.08
        try:
            self.gps_valid_hold_max_seconds = max(
                self.gps_valid_hold_min_seconds,
                float(
                    self.observer_config.get(
                        "gps_valid_hold_max_seconds",
                        self.local_config_defaults.get("common", {}).get(
                            "gps_valid_hold_max_seconds", 0.35
                        ),
                    )
                ),
            )
        except (TypeError, ValueError):
            self.gps_valid_hold_max_seconds = max(
                self.gps_valid_hold_min_seconds, 0.35
            )

    def _get_gps_valid_hold_window(self) -> float:
        """Return the GPS freshness window used to keep the last fix valid."""
        if (
            self.gps_valid_hold_mode == "adaptive"
            and self._gps_sample_period_estimate > 0.0
        ):
            hold = self.gps_valid_hold_scale * self._gps_sample_period_estimate
            return float(
                np.clip(
                    hold,
                    self.gps_valid_hold_min_seconds,
                    self.gps_valid_hold_max_seconds,
                )
            )
        return float(
            np.clip(
                self.gps_valid_hold_seconds,
                self.gps_valid_hold_min_seconds,
                self.gps_valid_hold_max_seconds,
            )
        )

    def _normalize_v2v_time_reference(
        self, time_reference: Optional[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        """Normalize shared V2V time-reference metadata."""
        if not isinstance(time_reference, dict):
            return None

        source = str(time_reference.get("source", "local")).strip() or "local"
        raw_reference_ns = time_reference.get(
            "reference_time_ns", time_reference.get("epoch_time_ns")
        )
        try:
            reference_time_ns = (
                int(raw_reference_ns) if raw_reference_ns is not None else None
            )
        except (TypeError, ValueError):
            reference_time_ns = None

        if reference_time_ns is not None and reference_time_ns < 0:
            reference_time_ns = None

        normalized = {
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

        return normalized

    def set_v2v_time_reference(
        self, time_reference: Optional[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        """Store and propagate the active shared V2V time reference."""
        normalized = self._normalize_v2v_time_reference(time_reference)
        with self.lock:
            self._v2v_time_reference = normalized
            if (
                self.fleet_estimator is not None
                and hasattr(self.fleet_estimator, "set_time_reference")
            ):
                self.fleet_estimator.set_time_reference(normalized)

        if self.vehicle_logger and normalized:
            self.vehicle_logger.logger.info(
                "VehicleObserver: Shared V2V time reference "
                f"source={normalized.get('source')} "
                f"reference_time_ns={normalized.get('reference_time_ns')}"
            )

        return dict(normalized) if normalized else None

    def clear_v2v_time_reference(self) -> None:
        """Clear the active shared V2V time reference."""
        self.set_v2v_time_reference(None)

    def has_v2v_time_reference(self) -> bool:
        """Return True when a shared V2V time reference is active."""
        with self.lock:
            return isinstance(self._v2v_time_reference, dict)

    def get_v2v_time_reference(self) -> Optional[Dict[str, Any]]:
        """Return a copy of the active shared V2V time reference."""
        with self.lock:
            if not isinstance(self._v2v_time_reference, dict):
                return None
            return dict(self._v2v_time_reference)

    def to_v2v_reference_time_ns(self, timestamp_ns: Optional[int] = None) -> int:
        """
        Convert a local wall-clock timestamp into the current V2V time domain.

        Without an active shared reference, this falls back to the raw local
        wall-clock nanosecond timestamp for backward compatibility.
        """
        ts_ns = int(time.time_ns()) if timestamp_ns is None else int(timestamp_ns)
        with self.lock:
            reference_time_ns = None
            if isinstance(self._v2v_time_reference, dict):
                reference_time_ns = self._v2v_time_reference.get("reference_time_ns")

        if reference_time_ns is None:
            return ts_ns
        return max(ts_ns - int(reference_time_ns), 0)

    def _build_v2v_time_payload_locked(self) -> Dict[str, Any]:
        """Return shared V2V time metadata for outgoing broadcast payloads."""
        source = "local"
        reference_vehicle_id: Optional[int] = None
        leader_id: Optional[int] = None
        if isinstance(self._v2v_time_reference, dict):
            source = str(self._v2v_time_reference.get("source", "local"))
            raw_reference_vehicle_id = self._v2v_time_reference.get(
                "reference_vehicle_id"
            )
            raw_leader_id = self._v2v_time_reference.get("leader_id")
            try:
                if raw_reference_vehicle_id is not None:
                    reference_vehicle_id = int(raw_reference_vehicle_id)
            except (TypeError, ValueError):
                reference_vehicle_id = None
            try:
                if raw_leader_id is not None:
                    leader_id = int(raw_leader_id)
            except (TypeError, ValueError):
                leader_id = None

        payload: Dict[str, Any] = {
            "timestamp_ref_ns": self.to_v2v_reference_time_ns(),
            "time_reference_source": source,
        }
        if reference_vehicle_id is not None:
            payload["time_reference_vehicle_id"] = reference_vehicle_id
        elif leader_id is not None:
            payload["time_reference_vehicle_id"] = leader_id
        return payload

    @staticmethod
    def _deep_merge_dict(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
        """Recursively merge override into base and return base."""
        if not isinstance(base, dict) or not isinstance(override, dict):
            return base

        for key, value in override.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                VehicleObserver._deep_merge_dict(base[key], value)
            else:
                base[key] = copy.deepcopy(value)
        return base

    def _merge_trust_child_into_fleet_config(
        self, fleet_cfg: Dict[str, Any], trust_cfg: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Merge TrustbasedDistributedObserver child config into fleet estimator config.

        Parent: Observer/config_fleet_estimators.yaml
        Child: Observer/TrustbasedDistributedObserver/config_trust_estimator.yaml
        """
        merged = copy.deepcopy(fleet_cfg) if isinstance(fleet_cfg, dict) else {}
        trust_cfg = trust_cfg or {}
        if not isinstance(trust_cfg, dict) or not trust_cfg:
            return merged

        fleet_section = merged.setdefault("fleet", {})
        if not isinstance(fleet_section, dict):
            fleet_section = {}
            merged["fleet"] = fleet_section

        # Ensure trust estimator sections exist before merging child parameters.
        trust_consensus = fleet_section.setdefault("trust_consensus", {})
        trust_kalman = fleet_section.setdefault("trust_kalman", {})
        if not isinstance(trust_consensus, dict):
            trust_consensus = {}
            fleet_section["trust_consensus"] = trust_consensus
        if not isinstance(trust_kalman, dict):
            trust_kalman = {}
            fleet_section["trust_kalman"] = trust_kalman

        # Allow an already-nested child layout:
        # trust_cfg["fleet"]["trust_consensus"/"trust_kalman"].
        child_fleet = trust_cfg.get("fleet", {})
        if isinstance(child_fleet, dict):
            child_consensus = child_fleet.get("trust_consensus", {})
            child_kalman = child_fleet.get("trust_kalman", {})
            if isinstance(child_consensus, dict):
                self._deep_merge_dict(trust_consensus, child_consensus)
            if isinstance(child_kalman, dict):
                self._deep_merge_dict(trust_kalman, child_kalman)

        # Backward-compatible child layout (top-level trust/weight/observer).
        child_trust = trust_cfg.get("trust", {})
        if isinstance(child_trust, dict):
            self._deep_merge_dict(trust_consensus.setdefault("trust", {}), child_trust)
            self._deep_merge_dict(trust_kalman.setdefault("trust", {}), child_trust)

        child_weight = trust_cfg.get("weight", {})
        if isinstance(child_weight, dict):
            self._deep_merge_dict(
                trust_consensus.setdefault("weight", {}), child_weight
            )
            self._deep_merge_dict(trust_kalman.setdefault("weight", {}), child_weight)

        child_observer = trust_cfg.get("observer", {})
        if isinstance(child_observer, dict):
            observer_common = {
                key: value for key, value in child_observer.items() if key != "kalman"
            }
            self._deep_merge_dict(trust_consensus, observer_common)
            self._deep_merge_dict(trust_kalman, observer_common)

            kalman_cfg = child_observer.get("kalman", {})
            if isinstance(kalman_cfg, dict):
                kalman_field_map = {
                    "process_noise": "process_noise",
                    "measurement_noise": "measurement_noise",
                    "initial_covariance": "initial_covariance",
                }
                for src_key, dst_key in kalman_field_map.items():
                    if src_key in kalman_cfg:
                        trust_kalman[dst_key] = copy.deepcopy(kalman_cfg[src_key])

        # Keep parent as source of truth for top-level estimator selection.
        # Only fallback to child type if parent omitted it.
        if "fleet_estimator_type" not in merged and "fleet_estimator_type" in trust_cfg:
            merged["fleet_estimator_type"] = trust_cfg["fleet_estimator_type"]

        child_vehicle = trust_cfg.get("vehicle", {})
        if isinstance(child_vehicle, dict):
            self._deep_merge_dict(trust_consensus.setdefault("vehicle", {}), child_vehicle)
            self._deep_merge_dict(trust_kalman.setdefault("vehicle", {}), child_vehicle)

        child_logging = trust_cfg.get("logging", {})
        if isinstance(child_logging, dict):
            self._deep_merge_dict(trust_consensus.setdefault("logging", {}), child_logging)
            self._deep_merge_dict(trust_kalman.setdefault("logging", {}), child_logging)

        for section_name in ("vehicle_models", "timestamp_alignment"):
            child_section = trust_cfg.get(section_name, {})
            if isinstance(child_section, dict):
                self._deep_merge_dict(
                    trust_consensus.setdefault(section_name, {}), child_section
                )
                self._deep_merge_dict(
                    trust_kalman.setdefault(section_name, {}), child_section
                )

        return merged

    def _apply_acceleration_ema(self, accel_raw: np.ndarray) -> np.ndarray:
        """Apply a simple EMA to the cached IMU acceleration."""
        accel = np.asarray(accel_raw, dtype=float).reshape(-1)
        if accel.size < 3:
            accel = np.pad(accel, (0, 3 - accel.size), mode="constant")
        accel = accel[:3]

        if not self._accel_filter_initialized:
            self._filtered_accelerometer = accel.copy()
            self._accel_filter_initialized = True
            return self._filtered_accelerometer.copy()

        alpha = self.accel_ema_alpha
        self._filtered_accelerometer = (
            alpha * accel + (1.0 - alpha) * self._filtered_accelerometer
        )
        return self._filtered_accelerometer.copy()

    def _apply_motor_tach_lpf(self, motor_tach_raw: float) -> float:
        """Apply a simple LPF to the cached motor tach signal."""
        motor_tach = float(motor_tach_raw)
        if not self._motor_tach_filter_initialized:
            self._filtered_motor_tach = motor_tach
            self._motor_tach_filter_initialized = True
            return self._filtered_motor_tach

        alpha = self.v_lpf_alpha
        self._filtered_motor_tach = (
            alpha * motor_tach + (1.0 - alpha) * self._filtered_motor_tach
        )
        return self._filtered_motor_tach

    def _init_recorders(self):
        """Initialize data recorders if enabled in config."""
        try:
            # Initialize Local Recorder
            if self.local_recording_enabled:
                self.local_recorder = ScopeDataRecorder(
                    output_dir="scope_recordings/local"
                )

                # Start recording with vehicle ID prefix
                self.local_recorder.start(
                    columns=LOCAL_RECORD_COLUMNS,
                    name=f"local_V{self.vehicle_id}",
                    overwrite=self.local_recording_overwrite,
                )
                self.vehicle_logger.logger.info(
                    f"Started local data recording for V{self.vehicle_id}"
                )

            # Initialize Fleet Recorder
            if self.fleet_recording_enabled:
                # Fleet recorder needs a fixed column budget for flattened CSV output
                max_vehicles = int(getattr(self, "fleet_recording_max_vehicles", 5))
                self.fleet_recorder = ScopeDataRecorder(
                    output_dir="scope_recordings/fleet", max_vehicles=max_vehicles
                )

                # Start recording with vehicle ID prefix
                self.fleet_recorder.start(
                    columns=build_fleet_record_columns(max_vehicles),
                    name=f"fleet_V{self.vehicle_id}",
                    overwrite=self.fleet_recording_overwrite,
                )
                self.vehicle_logger.logger.info(
                    f"Started fleet data recording for V{self.vehicle_id}"
                )

        except Exception as e:
            self.vehicle_logger.log_error("Failed to initialize recorders", e)

    # ===== Factory Methods for Creating Estimators =====

    @staticmethod
    def _config_bool(value, default: bool = False) -> bool:
        """Parse bool-like YAML/runtime config values."""
        if isinstance(value, bool):
            return value
        if value is None:
            return default
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "yes", "on", "enabled", "enable"}:
                return True
            if normalized in {"0", "false", "no", "off", "disabled", "disable"}:
                return False
        return default

    @staticmethod
    def _normalize_vehicle_type_name(value: Any) -> str:
        """Normalize supported vehicle type names."""
        return "Limo" if str(value or "").strip().lower() == "limo" else "Qcar"

    def _normalize_vehicle_manifest(
        self, vehicle_manifest: Optional[Dict[Any, Any]]
    ) -> Dict[int, Dict[str, Any]]:
        """Normalize V2V activation metadata keyed by vehicle ID."""
        if not isinstance(vehicle_manifest, dict):
            return {}

        normalized: Dict[int, Dict[str, Any]] = {}
        for raw_id, raw_meta in vehicle_manifest.items():
            if not isinstance(raw_meta, dict):
                continue

            raw_vehicle_id = raw_meta.get("vehicle_id", raw_id)
            try:
                vehicle_id = int(raw_vehicle_id)
            except (TypeError, ValueError):
                continue

            vehicle_type = self._normalize_vehicle_type_name(
                raw_meta.get("vehicle_type", raw_meta.get("type", "Qcar"))
            )
            programme_type = str(raw_meta.get("programme_type", "") or "").strip()
            if vehicle_type == "Limo":
                programme_type = "Ros"
            elif programme_type not in {"Py", "Ros"}:
                programme_type = "Py"

            raw_geometry = raw_meta.get(
                "geometry", raw_meta.get("vehicle_geometry", {})
            )
            geometry = {}
            if isinstance(raw_geometry, dict):
                for key in ("wheelbase", "l_r", "l_f", "track"):
                    value = raw_geometry.get(key)
                    if value is None:
                        continue
                    try:
                        geometry[key] = float(value)
                    except (TypeError, ValueError):
                        continue

            normalized[vehicle_id] = {
                "vehicle_type": vehicle_type,
                "programme_type": programme_type,
                "geometry": geometry,
            }

        return normalized

    @staticmethod
    def _find_vehicle_model_key(raw_models: Dict[Any, Any], vehicle_id: int):
        """Return the existing vehicle_models key matching a vehicle ID."""
        for raw_key in raw_models.keys():
            try:
                if int(raw_key) == int(vehicle_id):
                    return raw_key
            except (TypeError, ValueError):
                continue
        return None

    def _merge_v2v_manifest_into_fleet_config(
        self, resolved: Dict[str, Any]
    ) -> None:
        """
        Fill missing per-target model entries from V2V activation metadata.

        The host still uses the default `vehicle` model from config. Static
        `vehicle_models` entries remain authoritative and only receive missing
        metadata from the manifest.
        """
        if not self._v2v_vehicle_manifest:
            return

        raw_models = resolved.get("vehicle_models", {})
        if not isinstance(raw_models, dict):
            raw_models = {}
            resolved["vehicle_models"] = raw_models

        for vehicle_id, meta in self._v2v_vehicle_manifest.items():
            if int(vehicle_id) == int(self.vehicle_id):
                continue

            generated = {
                "vehicle_type": meta.get("vehicle_type", "Qcar"),
                "programme_type": meta.get("programme_type", "Py"),
            }
            geometry = meta.get("geometry", {})
            if isinstance(geometry, dict):
                generated.update(geometry)

            existing_key = self._find_vehicle_model_key(raw_models, vehicle_id)
            if existing_key is None:
                raw_models[int(vehicle_id)] = generated
                continue

            existing = raw_models.get(existing_key)
            if isinstance(existing, dict):
                merged = dict(generated)
                merged.update(existing)
                raw_models[existing_key] = merged

    def _resolve_fleet_estimator_config(self) -> Dict[str, Any]:
        """
        Resolve effective fleet estimator config for the currently selected type.

        Uses YAML defaults when available and falls back to observer-level gains.
        """
        fleet_config = self.fleet_config_defaults.get(self.fleet_estimator_type, {})
        if isinstance(fleet_config, dict) and fleet_config:
            resolved = copy.deepcopy(fleet_config)
            if self.vehicle_geometry_config:
                vehicle_cfg = resolved.setdefault("vehicle", {})
                if not isinstance(vehicle_cfg, dict):
                    vehicle_cfg = {}
                    resolved["vehicle"] = vehicle_cfg
                vehicle_cfg.update(self.vehicle_geometry_config)
            self._merge_v2v_manifest_into_fleet_config(resolved)
            return resolved

        resolved = {
            "consensus_gain": self.observer_config.get("consensus_gain", 0.3),
            "observer_gain": self.observer_config.get("observer_gain", 0.1),
        }
        self._merge_v2v_manifest_into_fleet_config(resolved)
        return resolved

    def _apply_vehicle_geometry_to_local_config(
        self, estimator_params: Dict[str, Any], explicit_override_keys=None
    ) -> None:
        """Inject per-vehicle geometry into local estimators that use a wheelbase."""
        if not isinstance(estimator_params, dict) or not self.vehicle_geometry_config:
            return

        explicit_override_keys = set(explicit_override_keys or ())
        wheelbase = self.vehicle_geometry_config.get("wheelbase")
        try:
            wheelbase = float(wheelbase)
        except (TypeError, ValueError):
            return
        if not np.isfinite(wheelbase) or wheelbase <= 0.0:
            return

        estimator_kind = str(self.local_estimator_type or "").strip().lower()
        if estimator_kind == "ekf":
            if "wheelbase" not in explicit_override_keys:
                estimator_params["wheelbase"] = wheelbase
            if "kin_wheelbase" not in explicit_override_keys:
                estimator_params["kin_wheelbase"] = wheelbase
        elif estimator_kind == "robust_kalman_net":
            if "kin_wheelbase" not in explicit_override_keys:
                estimator_params["kin_wheelbase"] = wheelbase
            comparator_cfg = estimator_params.setdefault("ekf_comparator_config", {})
            if isinstance(comparator_cfg, dict):
                comparator_cfg.setdefault("wheelbase", wheelbase)
                comparator_cfg.setdefault("kin_wheelbase", wheelbase)

    def _create_fleet_estimator(self):
        """Create fleet state estimator using factory"""
        try:
            fleet_config = self._resolve_fleet_estimator_config()
            self.fleet_estimator = FleetEstimatorFactory.create(
                estimator_type=self.fleet_estimator_type,
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                state_dim=self.state_dim,
                config=fleet_config,
                logger=self.vehicle_logger,
            )

            # self.vehicle_logger.logger.info(f"Fleet estimator created: {self.fleet_estimator_type}")
            # Add logging of created estimator type
            self.vehicle_logger.logger.info(
                f"Fleet estimator instance: {type(self.fleet_estimator).__name__} "
                f"(configured type='{self.fleet_estimator_type}')"
            )

        except Exception as e:
            self.vehicle_logger.log_error(
                f"Failed to create fleet estimator: {self.fleet_estimator_type}", e
            )
            # Fallback to consensus
            self.fleet_estimator = FleetEstimatorFactory.create(
                estimator_type="consensus",
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                state_dim=self.state_dim,
                config={"consensus_gain": 0.3},
                logger=self.vehicle_logger,
            )

    def initialize_relative_estimator(self, config_overrides: Dict = None):
        """
        Initialize relative state estimator if enabled.
        """
        if not self.enable_relative:
            return False

        try:
            params = copy.deepcopy(
                self.relative_config_defaults.get(self.relative_estimator_type, {})
            )
            if config_overrides:
                params.update(config_overrides)
            params.setdefault("vehicle_id", self.vehicle_id)
            params.setdefault("enable_recording", self.relative_recording_enabled)
            params.setdefault("recording_overwrite", self.relative_recording_overwrite)
            if self.relative_recording_output_dir is not None:
                params.setdefault(
                    "recording_output_dir", self.relative_recording_output_dir
                )

            self.relative_estimator = RelativeEstimatorFactory.create(
                estimator_type=self.relative_estimator_type,
                config=params,
                logger=self.vehicle_logger,
            )

            for target_id, measurement in self.sensor_data.get(
                "relative_measurements_by_target", {}
            ).items():
                if not hasattr(
                    self.relative_estimator, "set_external_relative_measurement"
                ):
                    break
                self.relative_estimator.set_external_relative_measurement(
                    target_id=int(target_id),
                    distance=float(measurement.get("distance", 0.0)),
                    relative_velocity=measurement.get("relative_velocity"),
                    timestamp_ns=measurement.get("timestamp_ns"),
                    source=str(measurement.get("source", "external_sensor")),
                    measurement_confidence=measurement.get("confidence"),
                )

            self.vehicle_logger.logger.info(
                f"Relative estimator initialized: {self.relative_estimator_type}"
            )
            return True
        except Exception as e:
            self.vehicle_logger.log_error(
                f"Relative estimator initialization failed: {self.relative_estimator_type}",
                e,
            )
            return False

    def initialize_local_estimator(
        self, gps=None, initial_pose=None, estimator_params: Dict = None
    ):
        """
        Initialize local state estimator using factory

        Args:
            gps: GPS instance (for EKF)
            initial_pose: Initial pose [x, y, theta]
            estimator_params: Additional parameters for the estimator

        Returns:
            bool: True if initialization successful
        """
        try:
            explicit_override_keys = set((estimator_params or {}).keys())
            estimator_params = estimator_params or {}

            # Merge with config defaults
            config_defaults = copy.deepcopy(self.local_config_defaults.get("common", {}))
            estimator_defaults = copy.deepcopy(
                self.local_config_defaults.get(self.local_estimator_type, {})
            )
            config_defaults.update(estimator_defaults)
            config_defaults.update(
                estimator_params
            )  # estimator_params override defaults
            estimator_params = config_defaults
            self._apply_vehicle_geometry_to_local_config(
                estimator_params, explicit_override_keys=explicit_override_keys
            )
            try:
                self.v_lpf_alpha = float(
                    np.clip(float(estimator_params.get("v_lpf_alpha", 1.0)), 0.0, 1.0)
                )
                self.accel_ema_alpha = float(
                    np.clip(
                        float(estimator_params.get("accel_ema_alpha", 1.0)),
                        0.0,
                        1.0,
                    )
                )
            except (TypeError, ValueError):
                self.v_lpf_alpha = 1.0
                self.accel_ema_alpha = 1.0

            # # motor_tach is filtered centrally in VehicleObserver, so disable
            # # the EKF's extra LPF by default to avoid double filtering.
            # if self.local_estimator_type == "ekf":
            #     estimator_params["v_lpf_alpha"] = 1.0

            # Store GPS reference at observer level for centralized sensor reading
            self.gps = gps

            self.local_estimator = LocalEstimatorFactory.create(
                estimator_type=self.local_estimator_type,
                initial_pose=initial_pose,
                logger=self.vehicle_logger,
                config=estimator_params,
            )

            self.vehicle_logger.logger.info(
                f"Local estimator initialized: {self.local_estimator_type}"
            )

            if self.enable_relative and self.relative_estimator is None:
                relative_ok = self.initialize_relative_estimator()
                if not relative_ok and self.vehicle_logger:
                    self.vehicle_logger.log_warning(
                        "Relative estimator is enabled but failed to initialize"
                    )

            return True

        except Exception as e:
            self.vehicle_logger.log_error(
                f"Local estimator initialization failed: {self.local_estimator_type}", e
            )
            return False

    # ===== Timing Control =====

    def _should_update_fleet_observer(self, current_time: float) -> bool:
        """Check if fleet observer should update based on its rate (independent of local observer)"""
        if (
            current_time - self._last_fleet_observer_time
            >= 1.0 / self.fleet_observer_rate
        ):
            self._last_fleet_observer_time = current_time
            return True
        return False

    # ===== Configuration =====

    def _get_observer_config(self) -> dict:
        """
        Get observer configuration by merging defaults with external config.

        External config is expected to provide an `observer` block (from YAML/JSON),
        e.g.:
            observer:
              observer_rate: 120
              fleet_observer_rate: 40
              local_estimator_type: ekf
              fleet_estimator_type: distributed_kalman
              observer_gain: [[...]]   # scalar, vector, or matrix
              consensus_gain: 0.2      # scalar, vector, or matrix

        The method is defensive: if no external config is found, it falls back to
        the hardcoded defaults.
        """
        default_config = {
            "observer_rate": 100,
            "fleet_observer_rate": 50,
            "camera_distance_offset": 0.0,
            "yolo_relative_min_confidence": 0.35,
        }

        # Pull observer config block from self.config if present
        observer_cfg = None
        if isinstance(self.config, dict):
            observer_cfg = self.config.get("observer")
        else:
            observer_cfg = getattr(self.config, "observer", None)

        if observer_cfg is None:
            return default_config

        # Convert possible dataclass/object to dict for easy access
        # Guard against malformed observer_cfg to keep loading robust
        try:
            cfg_dict = (
                observer_cfg
                if isinstance(observer_cfg, dict)
                else getattr(observer_cfg, "__dict__", {}) or {}
            )
        except Exception:
            # Fall back to defaults when the external block cannot be parsed
            return default_config

        merged = default_config.copy()
        merged["observer_rate"] = cfg_dict.get("observer_rate", merged["observer_rate"])
        merged["fleet_observer_rate"] = cfg_dict.get(
            "fleet_observer_rate", merged["fleet_observer_rate"]
        )

        if "camera_distance_offset" in cfg_dict:
            merged["camera_distance_offset"] = cfg_dict.get("camera_distance_offset")
        if "yolo_relative_min_confidence" in cfg_dict:
            merged["yolo_relative_min_confidence"] = cfg_dict.get(
                "yolo_relative_min_confidence"
            )

        return merged

    def _get_vehicle_geometry_config(self) -> dict:
        """Extract resolved vehicle geometry from runtime config, if available."""
        geometry_cfg = None
        if isinstance(self.config, dict):
            geometry_cfg = self.config.get("vehicle_geometry")
        else:
            geometry_cfg = getattr(self.config, "vehicle_geometry", None)

        if geometry_cfg is None:
            return {}

        try:
            cfg_dict = (
                geometry_cfg
                if isinstance(geometry_cfg, dict)
                else getattr(geometry_cfg, "__dict__", {}) or {}
            )
        except Exception:
            return {}

        normalized = {}
        for key in ("wheelbase", "l_r", "l_f", "track"):
            value = cfg_dict.get(key)
            if value is not None:
                normalized[key] = float(value)
        return normalized

    def update_sensor_data(self, qcar):
        """
        Update sensor data from QCar hardware AND GPS.
        This centralizes all sensor reading in one place.
        YOLO logic is handled separately in vehicle_logic.py
        """
        try:
            if qcar is not None:
                # Read QCar sensors - handle case where readTask might not exist
                try:
                    qcar.read()
                except AttributeError as read_error:
                    if "readTask" in str(read_error):
                        # QCar object doesn't have readTask, try alternative approach
                        # or skip reading if hardware is not properly initialized
                        self.vehicle_logger.log_warning(
                            "QCar readTask not available, skipping sensor read"
                        )
                        return False
                    else:
                        raise  # Re-raise if it's a different attribute error

                # Update sensor data cache
                with self.lock:
                    motor_tach_raw = float(getattr(qcar, "motorTach", 0.0))
                    motor_tach = self._apply_motor_tach_lpf(motor_tach_raw)
                    accel_raw = (
                        np.asarray(qcar.accelerometer, dtype=float)
                        if hasattr(qcar, "accelerometer")
                        else np.zeros(3)
                    )
                    accel_raw = accel_raw.reshape(-1)
                    if accel_raw.size < 3:
                        accel_raw = np.pad(accel_raw, (0, 3 - accel_raw.size), mode="constant")
                    accel_raw = accel_raw[:3]
                    accel_filtered = self._apply_acceleration_ema(accel_raw)
                    accel_magnitude_raw = float(
                        np.linalg.norm(accel_raw[:2]) if accel_raw.size >= 2 else 0.0
                    )
                    accel_magnitude = float(
                        np.linalg.norm(accel_filtered[:2])
                        if accel_filtered.size >= 2
                        else 0.0
                    )

                    # Read GPS once here (centralized GPS reading)
                    gps_valid = False
                    gps_fresh = False
                    # Initialize with last known position to prevent zero-flickering
                    gps_position = self.sensor_data.get("gps_position", np.zeros(3))
                    sensor_timestamp = time.time()
                    gps_age = float("inf")
                    gps_hold_window = self._get_gps_valid_hold_window()

                    if self.gps is not None:
                        try:
                            if self.gps.readGPS():
                                gps_fresh = True
                                if self._last_gps_sample_time > 0.0:
                                    gps_period = max(
                                        0.0,
                                        sensor_timestamp - self._last_gps_sample_time,
                                    )
                                    if 0.01 <= gps_period <= 2.0:
                                        if self._gps_sample_period_estimate <= 0.0:
                                            self._gps_sample_period_estimate = gps_period
                                        else:
                                            self._gps_sample_period_estimate = (
                                                0.2 * gps_period
                                                + 0.8 * self._gps_sample_period_estimate
                                            )
                                gps_position = np.array(
                                    [
                                        self.gps.position[0],
                                        self.gps.position[1],
                                        self.gps.orientation[2],
                                    ]
                                )
                                self._last_gps_sample_time = sensor_timestamp
                                gps_hold_window = self._get_gps_valid_hold_window()
                        except Exception as gps_error:
                            self.vehicle_logger.log_warning(
                                f"GPS read failed: {gps_error}"
                            )

                    if self._last_gps_sample_time > 0.0:
                        gps_age = max(0.0, sensor_timestamp - self._last_gps_sample_time)
                        gps_valid = gps_age <= gps_hold_window
                    gps_has_fix = self._last_gps_sample_time > 0.0

                    self.sensor_data.update(
                        {
                            "motor_tach_raw": motor_tach_raw,
                            "motor_tach": motor_tach,
                            "battery_voltage": float(
                                getattr(qcar, "batteryVoltage", 0.0)
                            ),
                            "gyro_z": qcar.gyroscope[2],
                            "accelerometer_raw": accel_raw.copy(),
                            "accelerometer": accel_filtered.copy(),
                            "accel_magnitude_raw": accel_magnitude_raw,
                            "accel_magnitude": accel_magnitude,
                            "timestamp": sensor_timestamp,
                            "gps_valid": gps_valid,
                            "gps_fresh": gps_fresh,
                            "gps_has_fix": gps_has_fix,
                            "gps_position": gps_position,
                            "gps_age": gps_age,
                            "gps_hold_window": gps_hold_window,
                        }
                    )

                return True

        except Exception as e:
            self.vehicle_logger.log_error("Sensor data update error", e)
            return False

    def update_observer(
        self, dt: float, last_steering: float = 0.0, throttle: float = 0.0
    ) -> dict:
        """
        Main observer update method that handles both local and fleet observer updates.
        Local observer is called every time (vehicle_logic controls the rate).
        Fleet observer has independent rate control.

        Args:
            dt: Time step
            last_steering: Last steering command
            throttle: Throttle command (for control input tracking)

        Returns:
            dict: Current state information compatible with vehicle_logic
        """
        current_time = time.time()

        try:
            # Update control input cache
            self.control_input = {"steering": last_steering, "throttle": throttle}
            self.acceleration_magnitude = self.sensor_data["accel_magnitude"]

            # Update local observer (always - rate controlled by vehicle_logic)
            state_info = self._update_local_observer(dt, last_steering, throttle)

            # Update fleet observer if it's time (independent rate control)
            if self._should_update_fleet_observer(current_time):
                self._update_fleet_observer_internal(dt)  # Distributed

            # Update relative observer (if enabled and measurements available)
            self._update_relative_observer(dt)

            return state_info

        except Exception as e:
            self.vehicle_logger.log_error("Observer update error", e)
            # Return last known state instead of zeros
            return self._get_last_known_state()

    def _update_local_observer(
        self, dt: float, last_steering: float = 0.0, last_u: float = 0.0
    ) -> dict:
        """
        Update local state estimation using pluggable local estimator.
        This is called every time vehicle_logic calls update_observer().

        Args:
            dt: Time step
            last_steering: Last steering command

        Returns:
            dict: Current state information compatible with vehicle_logic
        """
        try:
            if self.local_estimator is None:
                self.vehicle_logger.log_error(
                    "Local estimator not initialized! Cannot update observer."
                )
                raise RuntimeError(
                    "VehicleObserver: local_estimator is None - observer cannot function"
                )

            # Prepare GPS data dict for estimator. Robust KalmanNet benefits from
            # receiving stale cached GPS position together with freshness/age
            # metadata, while the classical estimators keep the older behavior.
            gps_data = None
            estimator_kind = str(getattr(self, "local_estimator_type", "")).strip().lower()
            if estimator_kind == "robust_kalman_net":
                if self.sensor_data.get("gps_has_fix", False):
                    gps_data = {
                        "x": self.sensor_data["gps_position"][0],
                        "y": self.sensor_data["gps_position"][1],
                        "theta": self.sensor_data["gps_position"][2],
                        "valid": bool(self.sensor_data.get("gps_valid", False)),
                        "position_valid": bool(self.sensor_data.get("gps_fresh", False)),
                        "hold_valid": bool(self.sensor_data.get("gps_valid", False)),
                        "fresh": bool(self.sensor_data.get("gps_fresh", False)),
                        "age_sec": float(self.sensor_data.get("gps_age", float("inf"))),
                        "has_fix": True,
                    }
            elif self.sensor_data.get("gps_valid", False):
                gps_data = {
                    "x": self.sensor_data["gps_position"][0],
                    "y": self.sensor_data["gps_position"][1],
                    "theta": self.sensor_data["gps_position"][2],
                    "valid": True,
                }

            # Update local estimator with sensor data
            success = self.local_estimator.update(
                motor_tach=self.sensor_data["motor_tach"],
                steering=last_steering,
                throttle=last_u,
                dt=dt,
                gyro_z=self.sensor_data["gyro_z"],
                gps_data=gps_data,  # Pass GPS data from centralized sensor reading
                acceleration=self.sensor_data["accelerometer"],
            )

            if not success:
                return self._get_last_known_state()

            # Get current state from estimator (returns numpy array directly)
            state = self.local_estimator.get_state()

            # Update local state cache - handle both 4D and 5 dimension states
            # GPS validity is tracked at observer level based on actual GPS reading
            gps_valid = self.sensor_data.get("gps_valid", False)

            with self.lock:
                if len(state) == 4:
                    # Legacy 4D state: [x, y, theta, v] - add acceleration
                    self.local_state = np.zeros(5)
                    self.local_state[:4] = state.copy()
                    self.local_state[4] = self._extract_accel_x_locked()
                else:
                    # 5D state: [x, y, theta, v, a]
                    self.local_state = state.copy()

                self.position = self.local_state[:3].copy()  # [x, y, theta]
                self.velocity = float(self.local_state[3])
                self.gps_valid = gps_valid  # GPS validity from sensor data

            # Record data if enabled
            if self.local_recorder and self.local_recorder.recording:
                record_data = {
                    "x": float(self.local_state[0]),
                    "y": float(self.local_state[1]),
                    "theta": float(self.local_state[2]),
                    "velocity": float(self.local_state[3]),
                    "acceleration": float(self.local_state[4]),
                    "x_gps": gps_data["x"] if gps_data else 0.0,
                    "y_gps": gps_data["y"] if gps_data else 0.0,
                    "theta_gps": gps_data["theta"] if gps_data else 0.0,
                    "steering": float(last_steering),
                    "throttle": float(last_u),
                    "v_ref": 0.0,  # Placeholder
                    "gps_valid": 1.0 if gps_valid else 0.0,
                }
                self.local_recorder.record(time.time(), record_data)

            return {
                "x": float(state[0]),
                "y": float(state[1]),
                "theta": float(state[2]),
                "velocity": float(state[3]),
                "acceleration": float(self.local_state[4]),
                "gps_valid": gps_valid,
                "position": self.position.copy(),
                "local_state": self.local_state.copy(),
            }

        except Exception as e:
            self.vehicle_logger.log_error("Local observer update error", e)
            return self._get_last_known_state()

    def _update_fleet_observer_internal(self, dt: float):
        """
        Update fleet observer estimates using pluggable fleet estimator.
        This is called based on fleet observer update rate.
        Only runs when V2V is active.

        Args:
            dt: Time step
        """
        try:
            # Only update fleet observer if V2V is active
            if not self.v2v_active:
                return

            # if not self.observer_config["enable_distributed"]: # Only enable, if V2V true
            #     return

            if self.fleet_estimator is None:
                return

            current_time_ns = self.to_v2v_reference_time_ns()

            # Pass actual control inputs (steering, throttle)
            control = np.array([
                self.control_input.get("steering", 0.0),
                self.control_input.get("throttle", 0.0)
            ])

            # Update fleet estimates using pluggable estimator
            current_local = self.local_state.copy()
            self.fleet_states = self.fleet_estimator.update(
                local_state=current_local,
                dt=dt,
                current_time_ns=current_time_ns,  # Pass nanoseconds
                control=control,
            )

            # Verify own state is correctly set in fleet_states
            if self.vehicle_id < self.fleet_size:
                own_fleet_state = self.fleet_states[:, self.vehicle_id]
                if np.allclose(own_fleet_state, 0.0) and not np.allclose(
                    current_local, 0.0
                ):
                    # Own state is zeros but local state is not - this is the bug!
                    self.vehicle_logger.logger.warning(
                        f"VehicleObserver WARNING: Own state in fleet is zeros but local state is not!\n"
                        f"  local_state: x={current_local[0]:.3f}, y={current_local[1]:.3f}, "
                        f"theta={current_local[2]:.3f}, v={current_local[3]:.3f}\n"
                        f"  fleet_states[{self.vehicle_id}]: x={own_fleet_state[0]:.3f}, "
                        f"y={own_fleet_state[1]:.3f}, theta={own_fleet_state[2]:.3f}, v={own_fleet_state[3]:.3f}"
                    )
                    # Force update
                    self.fleet_estimator.fleet_states[:, self.vehicle_id] = (
                        current_local.copy()
                    )
                    self.fleet_states = self.fleet_estimator.get_fleet_states()

            # Record fleet data if enabled
            if self.fleet_recorder and self.fleet_recorder.recording:
                # Prepare record data
                record_data = {
                    "consensus_error": 0.0,  # Placeholder, could be calculated
                    "fleet_states": self.fleet_states,
                }

                # Add trust scores if available
                if hasattr(self.fleet_estimator, "trust_scores"):
                    record_data["trust_scores"] = self.fleet_estimator.trust_scores
                elif hasattr(self.fleet_estimator, "get_trust_scores"):
                    record_data["trust_scores"] = (
                        self.fleet_estimator.get_trust_scores()
                    )

                self.fleet_recorder.record(time.time(), record_data)

        except Exception as e:
            self.vehicle_logger.log_error("Fleet observer update error", e)

    def set_relative_target(self, target_id: Optional[int]) -> None:
        """Set the single target tracked by the relative observer."""
        with self.lock:
            self.relative_target_id = None if target_id is None else int(target_id)

    def get_relative_target(self) -> Optional[int]:
        """Return the currently selected relative-observer target."""
        with self.lock:
            return self.relative_target_id

    def _get_target_vehicle_state(self, target_id: Optional[int]) -> Optional[np.ndarray]:
        """Return the latest cached fleet state for the selected target."""
        if target_id is None or target_id < 0:
            return None
        if target_id >= self.fleet_states.shape[1]:
            return None
        return self.fleet_states[:, target_id].copy()

    def _publish_relative_measurement(
        self,
        target_id: int,
        distance: float,
        relative_velocity: Optional[float],
        timestamp_ns: int,
        source: str,
        measurement_confidence: float,
    ) -> None:
        """Store one target measurement locally and forward it to interested estimators."""
        target_int = int(target_id)
        rel_velocity = (
            float(relative_velocity)
            if relative_velocity is not None and np.isfinite(relative_velocity)
            else float("nan")
        )

        message_timestamp_ns = self.to_v2v_reference_time_ns(timestamp_ns)

        with self.lock:
            self._last_relative_distance_by_target[target_int] = (
                float(distance),
                float(message_timestamp_ns) / 1e9,
            )
            self.sensor_data["relative_measurements_by_target"][target_int] = {
                "distance": float(distance),
                "relative_velocity": rel_velocity,
                "confidence": float(measurement_confidence),
                "timestamp_ns": int(message_timestamp_ns),
                "source": str(source),
            }

        self.set_relative_target(target_int)

        if (
            self.fleet_estimator is not None
            and hasattr(self.fleet_estimator, "set_external_relative_measurement")
        ):
            try:
                self.fleet_estimator.set_external_relative_measurement(
                    target_id=target_int,
                    distance=float(distance),
                    relative_velocity=(
                        rel_velocity if np.isfinite(rel_velocity) else None
                    ),
                    timestamp_ns=int(message_timestamp_ns),
                    source=str(source),
                    measurement_confidence=float(measurement_confidence),
                )
            except TypeError:
                self.fleet_estimator.set_external_relative_measurement(
                    target_id=target_int,
                    distance=float(distance),
                    relative_velocity=(
                        rel_velocity if np.isfinite(rel_velocity) else None
                    ),
                    timestamp_ns=int(message_timestamp_ns),
                    source=str(source),
                )

        if (
            self.relative_estimator is not None
            and hasattr(self.relative_estimator, "set_external_relative_measurement")
        ):
            try:
                self.relative_estimator.set_external_relative_measurement(
                    target_id=target_int,
                    distance=float(distance),
                    relative_velocity=(
                        rel_velocity if np.isfinite(rel_velocity) else None
                    ),
                    timestamp_ns=int(timestamp_ns),
                    source=str(source),
                    measurement_confidence=float(measurement_confidence),
                )
            except TypeError:
                self.relative_estimator.set_external_relative_measurement(
                    target_id=target_int,
                    distance=float(distance),
                    relative_velocity=(
                        rel_velocity if np.isfinite(rel_velocity) else None
                    ),
                    timestamp_ns=int(timestamp_ns),
                    source=str(source),
                )

    def _update_relative_observer(self, dt: float):
        """
        Update relative state observer.
        """
        try:
            if not self.enable_relative or self.relative_estimator is None:
                return

            target_id = self.get_relative_target()
            if target_id is None:
                return

            host_state = self.local_state  # [x, y, theta, v, a]
            target_state = self._get_target_vehicle_state(target_id)
            current_time_ns = time.time_ns()

            # Update
            self.relative_state = self.relative_estimator.update(
                measurement=None,
                dt=dt,
                control_input=self.control_input,
                target_vehicle_state=target_state,
                host_vehicle_state=host_state,
                target_id=target_id,
                current_time_ns=current_time_ns,
            )

        except Exception as e:
            self.vehicle_logger.log_error("Relative observer update error", e)

    def update_relative_measurement(
        self,
        measurement: np.ndarray,
        target_id: Optional[int] = None,
        source: str = "external_sensor",
        measurement_confidence: Optional[float] = None,
        timestamp_ns: Optional[int] = None,
    ):
        """
        Update relative measurement (e.g. from YOLO/radar).

        Args:
            measurement: [delta, delta_dot]
            target_id: Vehicle ID this measurement refers to. Defaults to current relative target.
            source: Source label for logging/debugging
            measurement_confidence: Optional source confidence in [0, 1]
            timestamp_ns: Optional measurement timestamp in nanoseconds
        """
        try:
            arr = np.asarray(measurement, dtype=float).flatten()
            if arr.size == 0:
                return

            rel_distance = float(arr[0])
            if not np.isfinite(rel_distance) or rel_distance <= 0.0:
                return

            target = target_id
            if target is None:
                target = self.get_relative_target()
            if target is None:
                return
            target = int(target)

            now_ns = int(timestamp_ns) if timestamp_ns is not None else int(time.time_ns())
            now_s = now_ns / 1e9
            source_name = str(source)
            source_name_l = source_name.lower()

            # Optional camera/source distance correction for YOLO relative measurements.
            camera_offset = float(self.observer_config.get("camera_distance_offset", 0.0))
            if "yolo" in source_name_l:
                rel_distance += camera_offset

            base_conf = float("nan")
            if measurement_confidence is not None:
                try:
                    base_conf = float(measurement_confidence)
                except Exception:
                    base_conf = float("nan")
            if not np.isfinite(base_conf):
                base_conf = 1.0
            base_conf = float(np.clip(base_conf, 0.0, 1.0))

            rel_velocity = float("nan")
            if arr.size > 1 and np.isfinite(arr[1]):
                rel_velocity = float(arr[1])
            else:
                prev = self._last_relative_distance_by_target.get(target)
                if prev is not None:
                    prev_distance, prev_time_s = prev
                    dt = now_s - prev_time_s
                    if dt > 1e-3:
                        rel_velocity = (rel_distance - prev_distance) / dt

            dynamic_conf = base_conf
            if "yolo" in source_name_l:
                distance_factor = float(
                    np.exp(-max(rel_distance - 6.0, 0.0) / 4.0)
                )
                consistency_factor = 1.0
                prev = self._last_relative_distance_by_target.get(target)
                if prev is not None:
                    prev_distance, prev_time_s = prev
                    dt = now_s - prev_time_s
                    if dt > 1e-3:
                        rel_velocity_from_distance = (rel_distance - prev_distance) / dt
                        if np.isfinite(rel_velocity):
                            consistency_error = abs(
                                rel_velocity - rel_velocity_from_distance
                            )
                            consistency_factor = float(
                                np.exp(-consistency_error / 1.5)
                            )
                        else:
                            consistency_factor = 0.8

                dynamic_conf = float(
                    np.clip(
                        0.60 * base_conf
                        + 0.25 * distance_factor
                        + 0.15 * consistency_factor,
                        0.0,
                        1.0,
                    )
                )
                min_conf = float(
                    self.observer_config.get("yolo_relative_min_confidence", 0.35)
                )
                if dynamic_conf < min_conf:
                    return

            self._publish_relative_measurement(
                target_id=target,
                distance=rel_distance,
                relative_velocity=(
                    rel_velocity if np.isfinite(rel_velocity) else None
                ),
                timestamp_ns=now_ns,
                source=source_name,
                measurement_confidence=dynamic_conf,
            )
        except Exception as e:
            self.vehicle_logger.log_error("Update relative measurement error", e)

    def set_local_estimator(self, estimator: LocalStateEstimatorBase):
        """
        Set or change the local state estimator.
        Allows different types of estimators to be used at runtime.

        Args:
            estimator: LocalStateEstimatorBase instance
        """
        previous_estimator = getattr(self, "local_estimator", None)
        if previous_estimator is not None and previous_estimator is not estimator:
            try:
                if hasattr(previous_estimator, "stop_recording"):
                    previous_estimator.stop_recording()
            except Exception as e:
                if self.vehicle_logger:
                    self.vehicle_logger.log_warning(
                        f"Failed to stop previous local estimator recording: {e}"
                    )

        self.local_estimator = estimator
        self.vehicle_logger.logger.info(
            f"Local estimator set for vehicle {self.vehicle_id}: {type(estimator).__name__}"
        )

    def set_fleet_estimator(self, estimator: FleetStateEstimatorBase):
        """
        Set or change the fleet state estimator.

        Args:
            estimator: FleetStateEstimatorBase instance
        """
        self.fleet_estimator = estimator
        self.vehicle_logger.logger.info(
            f"Fleet estimator set for vehicle {self.vehicle_id}: {type(estimator).__name__}"
        )

    def get_local_estimator(self) -> Optional[LocalStateEstimatorBase]:
        """
        Get the current local state estimator instance.

        Returns:
            LocalStateEstimatorBase instance or None
        """
        return self.local_estimator

    def get_fleet_estimator(self) -> Optional[FleetStateEstimatorBase]:
        """
        Get the current fleet state estimator instance.

        Returns:
            FleetStateEstimatorBase instance or None
        """
        return self.fleet_estimator

    def add_received_local_state(
        self, sender_id: int, state: Dict, timestamp_ns: int
    ) -> bool:
        """
        Add received LOCAL state from another vehicle (from local state broadcasts).
        Delegates to fleet estimator for processing.

        This is called when receiving high-frequency local sensor-based estimates
        from other vehicles (20Hz typical).

        Args:
            sender_id: ID of the vehicle that sent the state
            state: Received 5D state dict with keys: x, y, theta, velocity/v, acceleration (optional)
            timestamp_ns: Timestamp of the state in nanoseconds

        Returns:
            bool: True if state was added successfully
        """
        try:
            if sender_id == self.vehicle_id:
                return False  # Don't store own state

            if self.fleet_estimator is None:
                return False

            # Delegate to fleet estimator - use the proper method name
            return self.fleet_estimator.add_received_local_state(
                sender_id, state, timestamp_ns
            )

        except Exception as e:
            self.vehicle_logger.log_error("Add received local state error", e)
            return False

    def add_received_fleet_state(
        self, sender_id: int, fleet_estimates: Dict, timestamp_ns: int
    ) -> bool:
        """
        Add received fleet state estimates from another vehicle.
        Processes the entire fleet estimates dictionary and extracts individual vehicle states.

        Args:
            sender_id: ID of the vehicle that sent the fleet estimates
            fleet_estimates: Dictionary of fleet states in format:
                {
                    vehicle_id: {
                        'x': float, 'y': float, 'theta': float, 'velocity': float,
                        'acceleration': float,  # Now included!
                        'confidence': float (optional)
                    },
                    ...
                }
            timestamp_ns: Timestamp in nanoseconds

        Returns:
            bool: True if at least one state was added successfully
        """
        try:
            if self.fleet_estimator is None:
                return False

            # # Track if any states were successfully added
            # any_success = False

            # # Extract and add each vehicle's state from fleet estimates
            # for vehicle_id_key, vehicle_state in fleet_estimates.items():
            #     # Convert vehicle_id from string/int to int
            #     try:
            #         vehicle_id_int = int(vehicle_id_key)
            #     except (ValueError, TypeError):
            #         continue  # Skip invalid vehicle IDs

            #     # Skip own vehicle ID (we already have our own state)
            #     if vehicle_id_int == self.vehicle_id:
            #         continue

            #     # Validate vehicle state has required fields
            #     if not isinstance(vehicle_state, dict):
            #         continue

            #     required_fields = ['x', 'y', 'theta', 'velocity']
            #     if not all(field in vehicle_state for field in required_fields):
            #         continue

            # # Extract state components (5D with acceleration)
            # x = vehicle_state.get('x', 0.0)
            # y = vehicle_state.get('y', 0.0)
            # theta = vehicle_state.get('theta', 0.0)
            # velocity = vehicle_state.get('velocity', 0.0)
            # acceleration = vehicle_state.get('acceleration', 0.0)  # New field

            # # Create state vector [x, y, theta, v, a]
            # state_vector = np.array([x, y, theta, velocity, acceleration])

            # # Add to fleet estimator using local_state method (individual vehicle)
            # # Even though this came from a fleet broadcast, we're processing each
            # # vehicle's state individually, so we use add_received_local_state
            # success = self.fleet_estimator.add_received_local_state(
            #     sender_id=vehicle_id_int,
            #     state=state_vector,
            #     timestamp_ns=timestamp_ns
            # )

            # if success:
            #     any_success = True
            #     if self.vehicle_logger:
            #         self.vehicle_logger.logger.debug(
            #             f"VehicleObserver: Added fleet state for vehicle {vehicle_id_int} "
            #             f"(from fleet broadcast by sender {sender_id}) to fleet estimator"
            #         )

            # ALTERNATIVE: Could also pass entire dictionary directly to fleet estimator
            # This would allow fleet estimator to handle correlation between vehicles
            success = self.fleet_estimator.add_received_fleet_state(
                sender_id, fleet_estimates, timestamp_ns
            )

            return success

        except Exception as e:
            self.vehicle_logger.log_error("Add received fleet state error", e)
            return False

    def add_received_neighbor_trust_report(
        self, reporter_id: int, opinions: Dict[int, float]
    ) -> bool:
        """
        Add received neighbor trust opinions (reporter -> target trust).
        """
        try:
            if reporter_id == self.vehicle_id:
                return False
            if self.fleet_estimator is None:
                return False
            if not hasattr(self.fleet_estimator, "add_neighbor_trust_report"):
                return False

            updates = 0
            for target_id, score in opinions.items():
                try:
                    tid = int(target_id)
                    trust_score = float(score)
                    if not np.isfinite(trust_score):
                        continue
                    self.fleet_estimator.add_neighbor_trust_report(
                        reporter_id=reporter_id,
                        target_id=tid,
                        trust_score=float(np.clip(trust_score, 0.0, 1.0)),
                    )
                    updates += 1
                except (TypeError, ValueError):
                    continue

            return updates > 0
        except Exception as e:
            self.vehicle_logger.log_error("Add received neighbor trust report error", e)
            return False

    def get_local_state(self) -> np.ndarray:
        """Get current local state estimate."""
        with self.lock:
            return self.local_state.copy()

    def get_fleet_states(self) -> np.ndarray:
        """Get current fleet state estimates."""
        with self.lock:
            return self.fleet_states.copy()

    def get_vehicle_state(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get state estimate for a specific vehicle."""
        if 0 <= vehicle_id < self.fleet_size:
            with self.lock:
                return self.fleet_states[:, vehicle_id].copy()
        return None

    def get_current_position(self) -> List[float]:
        """Get current position [x, y, theta] compatible with vehicle_logic."""
        with self.lock:
            return [
                float(self.position[0]),
                float(self.position[1]),
                float(self.position[2]),
            ]

    def get_current_velocity(self) -> float:
        """Get current velocity compatible with vehicle_logic."""
        with self.lock:
            return float(self.velocity)

    def is_gps_valid(self) -> bool:
        """Check if GPS is valid."""
        with self.lock:
            return self.gps_valid

    def get_sensor_data(self) -> dict:
        """Get current sensor data."""
        with self.lock:
            return self.sensor_data.copy()

    # Old helper methods removed - fleet estimator handles data management internally

    def _get_last_known_state(self) -> dict:
        """Get last known state when estimation fails - preserves last valid state."""
        with self.lock:
            return {
                "x": float(self.local_state[0]),
                "y": float(self.local_state[1]),
                "theta": float(self.local_state[2]),
                "velocity": float(self.local_state[3]),
                "gps_valid": False,  # Mark as invalid but keep last position
                "position": self.position.copy(),
                "local_state": self.local_state.copy(),
            }

    def get_estimated_state_for_control(self) -> dict:
        """
        Get state information formatted for control systems.
        Compatible with existing vehicle_logic state format.
        """
        with self.lock:
            return {
                "x": float(self.local_state[0]),
                "y": float(self.local_state[1]),
                "theta": float(self.local_state[2]),
                "velocity": float(self.local_state[3]),
                "acceleration": float(self.local_state[4]),
                # 'motor_tach': self.sensor_data['motor_tach'],
                # 'gyro_z': self.sensor_data['gyro_z'],
                "gps_valid": self.gps_valid,
            }

    def _extract_accel_x_locked(self) -> float:
        """Extract longitudinal acceleration from sensor cache."""
        accel = self.sensor_data.get("accelerometer", np.zeros(3))
        try:
            if isinstance(accel, np.ndarray):
                return float(accel[0]) if accel.size > 0 else 0.0
            if isinstance(accel, (list, tuple)):
                return float(accel[0]) if len(accel) > 0 else 0.0
            return 0.0
        except Exception:
            return 0.0

    def _extract_accel_y_locked(self) -> float:
        """Extract lateral acceleration from sensor cache."""
        accel = self.sensor_data.get("accelerometer", np.zeros(3))
        try:
            if isinstance(accel, np.ndarray):
                return float(accel[1]) if accel.size > 1 else 0.0
            if isinstance(accel, (list, tuple)):
                return float(accel[1]) if len(accel) > 1 else 0.0
            return 0.0
        except Exception:
            return 0.0

    def _estimate_lateral_velocity_locked(
        self, vx: float, omega: float, dt_hint: Optional[float] = None
    ) -> float:
        """
        Estimate v_y using an Extended Kalman Filter with IMU relation
        vdot_y ~= a_y - v_x * omega.

        Args:
            vx: Measured/estimated longitudinal speed.
            omega: Measured yaw rate.
            dt_hint: Optional dynamic timestep from caller (seconds). If valid,
                this is preferred over timestamp differencing.
        """
        ts = float(self.sensor_data.get("timestamp", 0.0))
        if ts <= 0.0:
            ts = time.time()

        dt = 0.0
        if dt_hint is not None:
            try:
                dt = float(dt_hint)
            except Exception:
                dt = 0.0

        if self.vy_ekf is None:
            init_dt = dt if dt > 0.0 else 0.01
            if init_dt > 0.2:
                init_dt = 0.2
            self.vy_ekf = LateralVelocityEKF(dt=init_dt)
            self._vy_est_last_time = ts
            return 0.0

        # Prefer caller-provided dynamic timestep from control/observer loop.
        if dt > 0.0:
            if dt > 0.2:
                dt = 0.2
            self._vy_est_last_time = ts
        else:
            # Fallback: derive dt from sensor timestamp progression.
            if self._vy_est_last_time <= 0.0:
                self._vy_est_last_time = ts
                return float(self.vy_ekf.x_state[4])

            dt = ts - self._vy_est_last_time
            self._vy_est_last_time = ts

            if dt <= 0.0:
                return float(self.vy_ekf.x_state[4])

        if dt > 0.2:
            dt = 0.2

        self.vy_ekf.dt = dt

        ay = self._extract_accel_y_locked()
        ax = self._extract_accel_x_locked()
        delta = float(self.control_input.get("steering", 0.0))

        self.vy_ekf.predict(delta=delta, ax=ax, vx_meas=vx)

        gps_valid = self.sensor_data.get("gps_valid", False)
        if gps_valid:
            gps_pos = self.sensor_data.get("gps_position", np.zeros(3))
            gps_data = {
                "x": float(gps_pos[0]),
                "y": float(gps_pos[1]),
                "psi": float(gps_pos[2]),
            }
            vy_est = self.vy_ekf.update(
                omega_meas=omega,
                ay_meas=ay,
                delta=delta,
                ax=ax,
                vx_meas=vx,
                gps_data=gps_data,
            )
        else:
            vy_est = self.vy_ekf.update(
                omega_meas=omega, ay_meas=ay, delta=delta, ax=ax, vx_meas=vx
            )

        return float(vy_est)

    def get_online_sysid_sample(self, dt: Optional[float] = None) -> Optional[np.ndarray]:
        """
        Build one SysID sample [v_x, v_y, omega, delta].

        Uses LateralVelocityEKF (vy_kalman_filter.py) which estimates all states,
        so we don't need to depend on the local_estimator's 6D state.
        """
        with self.lock:
            # 1. Get raw measurements
            vx_meas = (
                float(self.local_state[3])
                if len(self.local_state) > 3
                else float(self.sensor_data.get("motor_tach", 0.0))
            )
            omega_meas = float(self.sensor_data.get("gyro_z", 0.0))
            delta = float(self.control_input.get("steering", 0.0))

            # 2. Update EKF (vy_kalman_filter.py)
            self._estimate_lateral_velocity_locked(vx_meas, omega_meas, dt_hint=dt)

            # 3. Extract filtered states from EKF
            if self.vy_ekf is not None:
                vx_est = float(self.vy_ekf.x_state[3])
                vy_est = float(self.vy_ekf.x_state[4])
                omega_est = float(self.vy_ekf.x_state[5])
            else:
                vx_est = vx_meas
                vy_est = 0.0
                omega_est = omega_meas

            sample = np.array([vx_est, vy_est, omega_est, delta], dtype=np.float32)
            if not np.all(np.isfinite(sample)):
                return None
            return sample

    def get_calibration_sample(self) -> Optional[np.ndarray]:
        """
        Build one calibration sample [v, throttle, steering, yaw_rate, ax, ay, az].

        Used by CalibratingState to record data during active calibration
        sequences (throttle-velocity, steering-curvature, throttle-acceleration).
        """
        with self.lock:
            v = (
                float(self.local_state[3])
                if len(self.local_state) > 3
                else float(self.sensor_data.get("motor_tach", 0.0))
            )
            throttle = float(self.control_input.get("throttle", 0.0))
            steering = float(self.control_input.get("steering", 0.0))
            yaw_rate = float(self.sensor_data.get("gyro_z", 0.0))
            accel = self.sensor_data.get(
                "accelerometer_raw",
                self.sensor_data.get("accelerometer", np.zeros(3)),
            )

            sample = np.array(
                [v, throttle, steering, yaw_rate,
                 float(accel[0]), float(accel[1]), float(accel[2])],
                dtype=np.float32,
            )
            if not np.all(np.isfinite(sample)):
                return None
            return sample

    def get_local_state_for_broadcast(self) -> dict:
        """
        Get local state information for V2V broadcasting.
        High-frequency, local sensor-based estimates.
        Includes acceleration and control inputs for cooperative control.
        """
        with self.lock:
            payload = {
                "vehicle_id": self.vehicle_id,
                "x": float(self.local_state[0]),
                "y": float(self.local_state[1]),
                "theta": float(self.local_state[2]),
                "velocity": float(self.local_state[3]),
                "acceleration": float(self.local_state[4]),
                "control_input": {
                    "steering": float(self.control_input["steering"]),
                    "throttle": float(self.control_input["throttle"]),
                },
                "gps_valid": self.gps_valid,
                "source": "local_sensors",
            }
            payload.update(self._build_v2v_time_payload_locked())
            return payload

    def get_fleet_state_for_broadcast(self) -> dict:
        """
        Get fleet state information for V2V broadcasting.
        Lower-frequency, consensus-based fleet estimates.
        Now includes acceleration: [x, y, theta, v, a]
        """
        with self.lock:
            fleet_data = {}
            for vehicle_id in range(self.fleet_size):
                fs = self.fleet_states[:, vehicle_id]
                if vehicle_id != self.vehicle_id and not np.any(fs):
                    continue
                # Include all tracked vehicles in fleet (zeros or not) for proper fleet estimation
                # The receiver can decide whether to use the data based on confidence/age
                fleet_data[vehicle_id] = {
                    "x": float(self.fleet_states[0, vehicle_id]),
                    "y": float(self.fleet_states[1, vehicle_id]),
                    "theta": float(self.fleet_states[2, vehicle_id]),
                    "velocity": float(self.fleet_states[3, vehicle_id]),
                    "acceleration": float(self.fleet_states[4, vehicle_id])
                    if self.fleet_states.shape[0] > 4
                    else 0.0,
                    "confidence": 1.0
                    if vehicle_id == self.vehicle_id
                    else 0.8,  # Higher confidence for own state
                }

            payload = {
                "sender_id": self.vehicle_id,
                "fleet_states": fleet_data,
                "source": "fleet_consensus",
            }
            payload.update(self._build_v2v_time_payload_locked())
            return payload

    def get_trust_report_for_broadcast(self) -> Optional[Dict[str, Any]]:
        """
        Get trust report for V2V broadcasting.

        Returns None when trust data is unavailable.
        """
        with self.lock:
            if self.fleet_estimator is None:
                return None
            if not hasattr(self.fleet_estimator, "get_all_trust_scores"):
                return None

            try:
                raw_trust_scores = self.fleet_estimator.get_all_trust_scores()
            except Exception:
                return None

            if not isinstance(raw_trust_scores, dict) or not raw_trust_scores:
                return None

            trust_scores: Dict[int, float] = {}
            for target_id, score in raw_trust_scores.items():
                try:
                    tid = int(target_id)
                    trust_score = float(score)
                    if np.isfinite(trust_score):
                        trust_scores[tid] = float(np.clip(trust_score, 0.0, 1.0))
                except (TypeError, ValueError):
                    continue

            if not trust_scores:
                return None

            generalized_vector: Dict[int, float] = {}
            if hasattr(self.fleet_estimator, "get_generalized_trust_vector"):
                try:
                    raw_generalized = self.fleet_estimator.get_generalized_trust_vector()
                    if isinstance(raw_generalized, dict):
                        for target_id, score in raw_generalized.items():
                            try:
                                tid = int(target_id)
                                trust_score = float(score)
                                if np.isfinite(trust_score):
                                    generalized_vector[tid] = float(
                                        np.clip(trust_score, 0.0, 1.0)
                                    )
                            except (TypeError, ValueError):
                                continue
                except Exception:
                    generalized_vector = {}

            payload = {
                "reporter_id": self.vehicle_id,
                "trust_scores": trust_scores,
                "generalized_trust_vector": generalized_vector,
                "source": "trust_estimator",
            }
            payload.update(self._build_v2v_time_payload_locked())
            return payload

    def reinitialize_fleet_estimation(
        self,
        new_fleet_size: int,
        peer_vehicle_ids: List[int],
        time_reference: Optional[Dict[str, Any]] = None,
        vehicle_manifest: Optional[Dict[Any, Any]] = None,
    ):
        """
        Reinitialize fleet estimation when V2V is activated with actual fleet information.
        This should be called when V2V activation provides the real fleet size and peer IDs.

        Args:
            new_fleet_size: Actual number of vehicles in the fleet (including this vehicle)
            peer_vehicle_ids: List of peer vehicle IDs that will be connected
            time_reference: Shared timing metadata for cross-vehicle V2V alignment
            vehicle_manifest: Optional per-vehicle metadata from the ground station
        """
        with self.lock:
            old_fleet_size = self.fleet_size
            self.fleet_size = new_fleet_size
            normalized_time_reference = self._normalize_v2v_time_reference(
                time_reference
            )
            self._v2v_time_reference = normalized_time_reference
            self._v2v_vehicle_manifest = self._normalize_vehicle_manifest(
                vehicle_manifest
            )

            # Mark V2V as active - fleet observer will start updating
            self.v2v_active = True

            # Create fresh fleet estimator with new fleet size (no old data to copy)
            try:
                fleet_config = self._resolve_fleet_estimator_config()

                # Create new fleet estimator with correct size
                self.fleet_estimator = FleetEstimatorFactory.create(
                    estimator_type=self.fleet_estimator_type,
                    vehicle_id=self.vehicle_id,
                    fleet_size=self.fleet_size,
                    state_dim=self.state_dim,
                    config=fleet_config,
                    logger=self.vehicle_logger,
                )
                if hasattr(self.fleet_estimator, "set_time_reference"):
                    self.fleet_estimator.set_time_reference(normalized_time_reference)

                # Initialize only own state in fleet - others will be updated as V2V data arrives
                if self.vehicle_id < self.fleet_size:
                    current_local = self.local_state.copy()
                    self.fleet_estimator.fleet_states[:, self.vehicle_id] = (
                        current_local
                    )
                    # self.vehicle_logger.logger.info(
                    #     f"Distributed Fleet Estimation: Initialized own state in fleet - "
                    #     f"vehicle_{self.vehicle_id}: x={current_local[0]:.3f}, y={current_local[1]:.3f}, "
                    #     f"theta={current_local[2]:.3f}, v={current_local[3]:.3f}"
                    # )

                # Update cached fleet states
                self.fleet_states = self.fleet_estimator.get_fleet_states()

                # Log the complete fleet state after reinit
                self.vehicle_logger.logger.info(
                    f"Distributed Fleet Estimation size: {self.fleet_size}, type: {self.fleet_estimator_type}"
                )
                for vid in range(self.fleet_size):
                    fs = self.fleet_states[:, vid]
                    self.vehicle_logger.logger.info(
                        f"  vehicle_{vid}: x={fs[0]:.3f}, y={fs[1]:.3f}, theta={fs[2]:.3f}, v={fs[3]:.3f}"
                    )

            except Exception as e:
                self.vehicle_logger.log_error(
                    "Fleet estimation reinitialization failed", e
                )

    def reset_fleet_estimation(self):
        """
        Reset fleet estimation state when V2V is disabled.
        Cleans up fleet estimator and resets fleet size to 1 (just this vehicle).
        """
        self.v2v_active = False
        self._v2v_time_reference = None
        self._v2v_vehicle_manifest = {}
        # self.fleet_size = max(self.vehicle_id + 1, 1) # Keep purely local

    def reset_observer(self, initial_pose: Optional[np.ndarray] = None):
        """Reset observer state."""
        with self.lock:
            # Reset local estimator
            if self.local_estimator is not None:
                self.local_estimator.reset(initial_pose)

            # Reset fleet estimator
            if self.fleet_estimator is not None:
                self.fleet_estimator.reset()

            # Reset relative estimator
            if self.relative_estimator is not None:
                self.relative_estimator.reset()
                if hasattr(
                    self.relative_estimator, "clear_external_relative_measurement"
                ):
                    self.relative_estimator.clear_external_relative_measurement()

            # Reset local state cache
            if initial_pose is not None:
                self.local_state[:3] = initial_pose
                self.local_state[3] = 0.0
                self.position = initial_pose.copy()
            else:
                self.local_state = np.zeros(self.state_dim)
                self.position = np.zeros(3)

            self.velocity = 0.0
            self.gps_valid = False
            self._last_gps_sample_time = 0.0
            self.relative_state = np.zeros(4)

            # Reset acceleration and control tracking
            # self.last_velocity = 0.0
            self.acceleration_magnitude = 0.0
            self._filtered_motor_tach = 0.0
            self._motor_tach_filter_initialized = False
            self._filtered_accelerometer = np.zeros(3)
            self._accel_filter_initialized = False
            self.control_input = {"steering": 0.0, "throttle": 0.0}
            self._vy_estimate = 0.0
            self._vy_est_last_time = 0.0
            self.relative_target_id = None
            self._last_relative_distance_by_target.clear()
            self.sensor_data["motor_tach_raw"] = 0.0
            self.sensor_data["motor_tach"] = 0.0
            self.sensor_data["accelerometer_raw"] = np.zeros(3)
            self.sensor_data["accelerometer"] = np.zeros(3)
            self.sensor_data["accel_magnitude_raw"] = 0.0
            self.sensor_data["accel_magnitude"] = 0.0
            self.sensor_data["gps_age"] = float("inf")
            self.sensor_data["relative_measurements_by_target"] = {}
            if hasattr(self, "vy_ekf") and self.vy_ekf is not None:
                self.vy_ekf = LateralVelocityEKF(dt=0.01)

            # Update fleet states from fleet estimator
            if self.fleet_estimator is not None:
                self.fleet_states = self.fleet_estimator.get_fleet_states()



    def stop(self):
        """Stop all observer activities and close recorders."""
        try:
            # Stop local recorder
            if self.local_recorder:
                self.local_recorder.stop()
                self.vehicle_logger.logger.info("Local data recorder stopped")

            # Stop fleet recorder
            if self.fleet_recorder:
                self.fleet_recorder.stop()
                self.vehicle_logger.logger.info("Fleet data recorder stopped")

            # Stop neural observer recording & auto-save model
            if self.local_estimator is not None and hasattr(
                self.local_estimator, "stop_recording"
            ):
                self.local_estimator.stop_recording()

            if self.relative_estimator is not None and hasattr(
                self.relative_estimator, "stop_recording"
            ):
                self.relative_estimator.stop_recording()
                self.vehicle_logger.logger.info("Relative UIO recorder stopped")

        except Exception as e:
            if self.vehicle_logger:
                self.vehicle_logger.log_error("Error stopping observer", e)

    def __del__(self):
        """Cleanup on destruction."""
        self.stop()
