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
from Observer.estimation_scopes import ScopeDataRecorder
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

            # Load fleet plotting config
            self.fleet_plotting_config = {
                "enabled": loaded.get("enable_plotting", False),
                "params": loaded.get("plotting", {}),
            }
            # Load fleet recording config
            self.fleet_recording_enabled = loaded.get("enable_recording", False)
            self.fleet_recording_overwrite = loaded.get(
                "recording_overwrite", False
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

                # Load local plotting config
                self.local_plotting_config = {
                    "enabled": loaded.get("enable_plotting", False),
                    "params": loaded.get("plotting", {}),
                }
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

        # Load relative estimator defaults
        self.relative_config_defaults = {}
        self.enable_relative = False
        self.relative_estimator_type = "sa_acc_uio"
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
                self.enable_relative = loaded.get("enable_relative", False)

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

        # ===== Relative State Estimator (pluggable) =====
        self.relative_estimator: Optional[RelativeStateEstimatorBase] = None
        # Will be initialized if enabled

        # ===== State Cache (for quick access) =====
        self.local_state = np.zeros(self.state_dim)
        self.position = np.zeros(3)  # [x, y, theta]
        self.velocity = 0.0
        self.gps_valid = False  # GPS validity flag

        # Fleet states (managed by fleet_estimator but cached here)
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))

        # Relative states
        self.relative_state = np.zeros(
            4
        )  # Default size [delta, delta_dot, delta_ddot, f_c]

        # ===== Sensor Data Cache =====

        self.sensor_data = {
            "motor_tach": 0.0,
            "gyro_z": 0.0,
            "accelerometer": np.zeros(3),
            "accel_magnitude": 0.0,
            "timestamp": 0.0,
            "gps_valid": False,
            "gps_position": np.zeros(3),  # [x, y, theta]
            "relative_measurement": np.zeros(2),  # [delta, delta_dot] from YOLO etc.
            "relative_measurement_valid": False,
            "relative_measurement_confidence": float("nan"),
            "relative_measurements_by_target": {},
        }
        # For derivative estimation when only distance is provided.
        self._last_relative_distance_by_target: Dict[int, Tuple[float, float]] = {}

        # ===== Control and Dynamics Cache =====
        # self.last_velocity = 0.0
        self.acceleration_magnitude = 0.0
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

        return merged

    def _init_recorders(self):
        """Initialize data recorders if enabled in config."""
        try:
            # Initialize Local Recorder
            if self.local_recording_enabled:
                self.local_recorder = ScopeDataRecorder(
                    output_dir="scope_recordings/local"
                )

                # Define local columns
                local_columns = [
                    "x",
                    "y",
                    "theta",
                    "velocity",
                    "acceleration",
                    "x_gps",
                    "y_gps",
                    "theta_gps",  # GPS reference
                    "steering",
                    "throttle",  # Control inputs
                    "v_ref",  # Reference velocity
                    "gps_valid",  # GPS validity flag
                ]

                # Start recording with vehicle ID prefix
                self.local_recorder.start(
                    columns=local_columns,
                    name=f"local_V{self.vehicle_id}",
                    overwrite=self.local_recording_overwrite,
                )
                self.vehicle_logger.logger.info(
                    f"Started local data recording for V{self.vehicle_id}"
                )

            # Initialize Fleet Recorder
            if self.fleet_recording_enabled:
                # Fleet recorder needs max_vehicles
                max_vehicles = self.fleet_plotting_config["params"].get(
                    "max_vehicles_plot", 5
                )
                self.fleet_recorder = ScopeDataRecorder(
                    output_dir="scope_recordings/fleet", max_vehicles=max_vehicles
                )

                # Define fleet columns (using helper for flattened names)
                fleet_columns = ["consensus_error"]

                # Add flattened state columns: fleet_x_0, fleet_y_0, etc.
                state_names = ["x", "y", "theta", "v", "a"]
                for v_idx in range(max_vehicles):
                    for s_name in state_names:
                        fleet_columns.append(f"fleet_{s_name}_{v_idx}")

                # Add trust score columns
                for v_idx in range(max_vehicles):
                    fleet_columns.append(f"trust_{v_idx}")

                # Start recording with vehicle ID prefix
                self.fleet_recorder.start(
                    columns=fleet_columns,
                    name=f"fleet_V{self.vehicle_id}",
                    overwrite=self.fleet_recording_overwrite,
                )
                self.vehicle_logger.logger.info(
                    f"Started fleet data recording for V{self.vehicle_id}"
                )

        except Exception as e:
            self.vehicle_logger.log_error("Failed to initialize recorders", e)

    # ===== Factory Methods for Creating Estimators =====

    def _resolve_fleet_estimator_config(self) -> Dict[str, Any]:
        """
        Resolve effective fleet estimator config for the currently selected type.

        Uses YAML defaults when available and falls back to observer-level gains.
        """
        fleet_config = self.fleet_config_defaults.get(self.fleet_estimator_type, {})
        if isinstance(fleet_config, dict) and fleet_config:
            return copy.deepcopy(fleet_config)

        return {
            "consensus_gain": self.observer_config.get("consensus_gain", 0.3),
            "observer_gain": self.observer_config.get("observer_gain", 0.1),
        }

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
            params = self.relative_config_defaults.get(self.relative_estimator_type, {})
            if config_overrides:
                params.update(config_overrides)

            self.relative_estimator = RelativeEstimatorFactory.create(
                estimator_type=self.relative_estimator_type,
                config=params,
                logger=self.vehicle_logger,
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
            estimator_params = estimator_params or {}

            # Merge with config defaults
            config_defaults = self.local_config_defaults.get(
                self.local_estimator_type, {}
            )
            config_defaults.update(
                estimator_params
            )  # estimator_params override defaults
            estimator_params = config_defaults

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

        return merged

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
                    # Read accelerometer (x, y, z axes)
                    accel_x = (
                        qcar.accelerometer[0] if hasattr(qcar, "accelerometer") else 0.0
                    )
                    accel_y = (
                        qcar.accelerometer[1] if hasattr(qcar, "accelerometer") else 0.0
                    )
                    # Calculate horizontal acceleration magnitude (norm of x and y)
                    accel_magnitude = float(np.sqrt(accel_x**2 + accel_y**2))

                    # Read GPS once here (centralized GPS reading)
                    gps_valid = False
                    # Initialize with last known position to prevent zero-flickering
                    gps_position = self.sensor_data.get("gps_position", np.zeros(3))

                    if self.gps is not None:
                        try:
                            if self.gps.readGPS():
                                gps_valid = True
                                gps_position = np.array(
                                    [
                                        self.gps.position[0],
                                        self.gps.position[1],
                                        self.gps.orientation[2],
                                    ]
                                )
                        except Exception as gps_error:
                            self.vehicle_logger.log_warning(
                                f"GPS read failed: {gps_error}"
                            )
                            gps_valid = False

                    self.sensor_data.update(
                        {
                            "motor_tach": qcar.motorTach,
                            "gyro_z": qcar.gyroscope[2]
                            if hasattr(qcar, "gyroscope")
                            else 0.0,
                            "accelerometer": qcar.accelerometer
                            if hasattr(qcar, "accelerometer")
                            else np.zeros(3),
                            "accel_magnitude": accel_magnitude,
                            "timestamp": time.time(),
                            "gps_valid": gps_valid,
                            "gps_position": gps_position,
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

            # Prepare GPS data dict for estimator (if GPS is valid)
            gps_data = None
            if self.sensor_data.get("gps_valid", False):
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
                    self.local_state[4] = (
                        self.acceleration_magnitude
                    )  # Add current acceleration
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

            current_time_ns = (
                time.time_ns()
            )  # Use nanoseconds for consistency with V2V timestamps

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

    def _update_relative_observer(self, dt: float):
        """
        Update relative state observer.
        """
        try:
            if not self.enable_relative or self.relative_estimator is None:
                return

            # Check for valid measurement (YOLO/Radar)
            # If not valid, we might skip update or update with prediction only (if supported)
            # The SA_ACC_UIO requires measurement y(k)

            # Need measurement: [delta, delta_dot]
            # y_meas = self.sensor_data.get('relative_measurement')
            # valid = self.sensor_data.get('relative_measurement_valid', False)

            # TEMP: If not valid, just return for now (or implement hold?)
            # if not valid:
            #    return

            # For now, let's assume if it's enabled, we try to use cached data
            # Ideally this comes from vehicle_logic loop feeding update_sensor_data with new Yolo info

            valid = bool(self.sensor_data.get("relative_measurement_valid", False))
            if not valid:
                return

            y_meas = self.sensor_data["relative_measurement"]

            # Preceding vehicle state (from V2V via fleet estimator)
            # Assuming predecessor is id - 1.
            # TODO: logic to determine predecessor ID dynamically
            pre_id = self.vehicle_id - 1
            pre_state = None
            if pre_id >= 0 and self.fleet_estimator:
                # Get from fleet estimator cache
                # fleet_states is [state_dim x fleet_size]
                if pre_id < self.fleet_states.shape[1]:
                    pre_state = self.fleet_states[:, pre_id]

            # Host state
            host_state = self.local_state  # [x, y, theta, v, a]

            # Update
            self.relative_state = self.relative_estimator.update(
                measurement=y_meas,
                dt=dt,
                control_input=self.control_input,
                pre_vehicle_state=pre_state,
                host_vehicle_state=host_state,
            )

        except Exception as e:
            self.vehicle_logger.log_error("Relative observer update error", e)

    def update_relative_measurement(
        self,
        measurement: np.ndarray,
        target_id: Optional[int] = None,
        source: str = "external_sensor",
        measurement_confidence: Optional[float] = None,
    ):
        """
        Update relative measurement (e.g. from YOLO/radar).

        Args:
            measurement: [delta, delta_dot]
            target_id: Vehicle ID this measurement refers to (defaults to predecessor)
            source: Source label for logging/debugging
            measurement_confidence: Optional source confidence in [0, 1]
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
                target = self.vehicle_id - 1 if self.vehicle_id > 0 else None

            now_s = time.time()
            source_name = str(source)
            source_name_l = source_name.lower()

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
            elif target is not None:
                prev = self._last_relative_distance_by_target.get(int(target))
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
                if target is not None:
                    prev = self._last_relative_distance_by_target.get(int(target))
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
                    with self.lock:
                        self.sensor_data["relative_measurement_valid"] = False
                        self.sensor_data["relative_measurement_confidence"] = dynamic_conf
                    return

            y_meas = np.array(
                [rel_distance, rel_velocity if np.isfinite(rel_velocity) else 0.0],
                dtype=float,
            )

            with self.lock:
                self.sensor_data["relative_measurement"] = y_meas
                self.sensor_data["relative_measurement_valid"] = True
                self.sensor_data["relative_measurement_confidence"] = dynamic_conf

                if target is not None:
                    target_int = int(target)
                    self._last_relative_distance_by_target[target_int] = (
                        rel_distance,
                        now_s,
                    )
                    self.sensor_data["relative_measurements_by_target"][target_int] = {
                        "distance": rel_distance,
                        "relative_velocity": rel_velocity,
                        "confidence": dynamic_conf,
                        "timestamp_ns": int(now_s * 1e9),
                        "source": source_name,
                    }

            # Feed trust-based estimators when they support external measurements.
            if (
                target is not None
                and self.fleet_estimator is not None
                and hasattr(self.fleet_estimator, "set_external_relative_measurement")
            ):
                try:
                    self.fleet_estimator.set_external_relative_measurement(
                        target_id=int(target),
                        distance=rel_distance,
                        relative_velocity=(
                            rel_velocity if np.isfinite(rel_velocity) else None
                        ),
                        timestamp_ns=int(now_s * 1e9),
                        source=source_name,
                        measurement_confidence=dynamic_conf,
                    )
                except TypeError:
                    # Backward compatibility with estimators that don't support confidence yet.
                    self.fleet_estimator.set_external_relative_measurement(
                        target_id=int(target),
                        distance=rel_distance,
                        relative_velocity=(
                            rel_velocity if np.isfinite(rel_velocity) else None
                        ),
                        timestamp_ns=int(now_s * 1e9),
                        source=source_name,
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
            accel = self.sensor_data.get("accelerometer", np.zeros(3))

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
            return {
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

    def get_fleet_state_for_broadcast(self) -> dict:
        """
        Get fleet state information for V2V broadcasting.
        Lower-frequency, consensus-based fleet estimates.
        Now includes acceleration: [x, y, theta, v, a]
        """
        with self.lock:
            fleet_data = {}
            for vehicle_id in range(self.fleet_size):
                # Include all vehicles in fleet (zeros or not) for proper fleet estimation
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

            return {
                "sender_id": self.vehicle_id,
                "fleet_states": fleet_data,
                "source": "fleet_consensus",
            }

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

            return {
                "reporter_id": self.vehicle_id,
                "trust_scores": trust_scores,
                "generalized_trust_vector": generalized_vector,
                "source": "trust_estimator",
            }

    def reinitialize_fleet_estimation(
        self, new_fleet_size: int, peer_vehicle_ids: List[int]
    ):
        """
        Reinitialize fleet estimation when V2V is activated with actual fleet information.
        This should be called when V2V activation provides the real fleet size and peer IDs.

        Args:
            new_fleet_size: Actual number of vehicles in the fleet (including this vehicle)
            peer_vehicle_ids: List of peer vehicle IDs that will be connected
        """
        with self.lock:
            old_fleet_size = self.fleet_size
            self.fleet_size = new_fleet_size

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

            # Reset acceleration and control tracking
            # self.last_velocity = 0.0
            self.acceleration_magnitude = 0.0
            self.control_input = {"steering": 0.0, "throttle": 0.0}
            self._vy_estimate = 0.0
            self._vy_est_last_time = 0.0
            self._last_relative_distance_by_target.clear()
            self.sensor_data["relative_measurement"] = np.zeros(2)
            self.sensor_data["relative_measurement_valid"] = False
            self.sensor_data["relative_measurement_confidence"] = float("nan")
            self.sensor_data["relative_measurements_by_target"] = {}
            if hasattr(self, "vy_ekf") and self.vy_ekf is not None:
                self.vy_ekf = LateralVelocityEKF(dt=0.01)

            # Update fleet states from fleet estimator
            if self.fleet_estimator is not None:
                self.fleet_states = self.fleet_estimator.get_fleet_states()

    # ===== Scope Integration for Visualization =====

    # def enable_scopes(self, preset_names: List[str] = None, fps: int = 30,
    #                   time_window: float = 60.0) -> bool:
    #     """
    #     Enable visualization scopes for estimator data.

    #     Args:
    #         preset_names: List of preset names to enable. Options:
    #             - 'local_state': x, y, theta, velocity estimation
    #             - 'local_error': estimation vs GPS error
    #             - 'local_control': steering, throttle, velocity
    #             - 'fleet_position': XY plot of all vehicles
    #             - 'fleet_state': fleet state time series
    #             - 'fleet_consensus': consensus error and trust scores
    #             If None, enables all presets.
    #         fps: Refresh rate (frames per second)
    #         time_window: Time window for plots in seconds

    #     Returns:
    #         True if scopes enabled successfully
    #     """
    #     try:
    #         from Observer.estimation_scopes import (
    #             EstimationScopeManager,
    #             create_scope_manager,
    #             MULTISCOPE_AVAILABLE
    #         )

    #         if not MULTISCOPE_AVAILABLE:
    #             self.vehicle_logger.log_warning("MultiScope not available - scopes disabled")
    #             return False

    #         if preset_names is None:
    #             preset_names = ['local_state', 'local_error', 'local_control',
    #                            'fleet_position', 'fleet_state', 'fleet_consensus']

    #         self.scope_manager = create_scope_manager(
    #             preset_names=preset_names,
    #             fps=fps,
    #             time_window=time_window,
    #             max_vehicles=self.fleet_size
    #         )
    #         self.scope_manager.start()

    #         self.vehicle_logger.logger.info(
    #             f"Estimation scopes enabled: {preset_names}"
    #         )
    #         return True

    #     except ImportError as e:
    #         self.vehicle_logger.log_warning(f"Could not import estimation_scopes: {e}")
    #         return False
    #     except Exception as e:
    #         self.vehicle_logger.log_error("Failed to enable scopes", e)
    #         return False

    def sample_scopes(self, t: float) -> None:
        """
        Sample current estimator data to visualization scopes.
        Call this after update_observer() in the control loop.

        Args:
            t: Current time in seconds (relative to experiment start)
        """
        if not hasattr(self, "scope_manager") or self.scope_manager is None:
            return

        try:
            # Build combined data dict for all presets
            data = {
                # Local state
                "x": float(self.local_state[0]),
                "y": float(self.local_state[1]),
                "theta": float(self.local_state[2]),
                "velocity": float(self.local_state[3]),
                "acceleration": float(self.local_state[4])
                if len(self.local_state) > 4
                else 0.0,
                # GPS reference (if available)
                "x_gps": float(self.sensor_data["gps_position"][0])
                if self.gps_valid
                else float(self.local_state[0]),
                "y_gps": float(self.sensor_data["gps_position"][1])
                if self.gps_valid
                else float(self.local_state[1]),
                "theta_gps": float(self.sensor_data["gps_position"][2])
                if self.gps_valid
                else float(self.local_state[2]),
                # Control inputs
                "steering": float(self.control_input.get("steering", 0.0)),
                "throttle": float(self.control_input.get("throttle", 0.0)),
                "v_ref": 0.0,  # Can be passed from vehicle_logic if needed
                # Fleet states
                "fleet_states": self.fleet_states.copy(),
                "consensus_error": 0.0,  # Can be computed by fleet estimator
                "trust_scores": {},  # Can be populated by trust-based estimators
            }

            self.scope_manager.sample(t, data)

        except Exception as e:
            # Non-blocking - don't interrupt main loop on scope errors
            pass

    def stop_scopes(self) -> None:
        """Stop visualization scopes."""
        if hasattr(self, "scope_manager") and self.scope_manager is not None:
            self.scope_manager.stop()
            self.scope_manager = None
            self.vehicle_logger.logger.info("Estimation scopes stopped")

    def start_scope_recording(self) -> Optional[str]:
        """
        Start recording scope data to file.

        Returns:
            Path to recording file, or None if failed
        """
        if hasattr(self, "scope_manager") and self.scope_manager is not None:
            return self.scope_manager.start_recording()
        return None

    def stop_scope_recording(self) -> None:
        """Stop recording scope data."""
        if hasattr(self, "scope_manager") and self.scope_manager is not None:
            self.scope_manager.stop_recording()

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

        except Exception as e:
            if self.vehicle_logger:
                self.vehicle_logger.log_error("Error stopping observer", e)

    def __del__(self):
        """Cleanup on destruction."""
        self.stop()
