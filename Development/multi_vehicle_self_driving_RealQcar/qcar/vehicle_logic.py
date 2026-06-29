"""
Main Vehicle Controller - Integrates all components
"""

import numpy as np
import time
import os

# import random
from typing import Optional
from threading import Event

try:
    from pal.products.qcar import IS_PHYSICAL_QCAR
except Exception:
    IS_PHYSICAL_QCAR = False
# from hal.products.mats import SDCSRoadMap

from config_main import VehicleMainConfig
from logging_utils import VehicleLogger, PerformanceMonitor
from StateMachine import VehicleState, VehicleStateMachine, Gear
from ground_station_client import GroundStationClient
from safety import WatchdogTimer
from Yolo.YoLo import YOLOManager
from Controller.platoon_controller import PlatoonController, PlatoonConfig
from command_handler import CommandHandler
from V2V.v2v_manager import V2VManager, V2VBroadcastConfig
from Observer.VehicleObserverSimple import VehicleObserver
from Taxi.taxi_manager import TaxiManager

# from Observer.estimation_scopes import EstimationScopeManager, LocalStatePreset, LocalControlPreset, FleetPositionPreset, FleetStatePreset
from Controller.controller_manager import ControllerManager

try:
    from Calibration.On_Track_SysID.src.online_sysid_zmq_client import (
        OnlineSysIDZMQClient,
    )
except Exception:
    OnlineSysIDZMQClient = None

try:
    from Calibration.online_calibration_zmq_client import (
        OnlineCalibrationZMQClient,
    )
except Exception:
    OnlineCalibrationZMQClient = None

try:
    from Observer.KalmaNet.Robust.robust_kalmannet_dataset import (
        RobustKalmanNetDatasetRecorder,
    )
except Exception:
    RobustKalmanNetDatasetRecorder = None

# Note: Controllers (PIDVelocityController, StanleyController) are now imported
# in state machine states, not here


class VehicleLogic:
    """Main vehicle controller class"""

    def __init__(self, config: VehicleMainConfig, kill_event: Event):
        self.config = config
        self.kill_event = kill_event

        # Vehicle identification
        # vehicle_id: Connection/network ID (used for Ground Station communication, file naming, etc.)
        self.vehicle_id = config.network.car_id
        self.vehicle_type = config.vehicle.vehicle_type
        self.programme_type = config.vehicle.programme_type
        self.is_physical_qcar = IS_PHYSICAL_QCAR
        setattr(config.vehicle, "is_physical_qcar", bool(self.is_physical_qcar))


        # self.Is_Limo_Car = config.network.car_id

        # vehicle_position: Position in platoon formation (1=leader, 2=first follower, 3=second follower, etc.)
        # Initially set to vehicle_id, but can be changed by Ground Station platoon formation commands
        self.vehicle_position = config.network.car_id
        # Setup logging
        self.vehicle_logger = VehicleLogger(
            car_id=config.network.car_id,
            log_dir=config.logging.log_dir,
            log_level=config.logging.log_level,
            logging_config=config.logging,
        )

        self.vehicle_logger.logger.info("=" * 60)
        self.vehicle_logger.logger.info(
            f"Vehicle Controller Initialized - Car ID: {config.network.car_id}"
        )
        self.vehicle_logger.logger.info("=" * 60)

        # Performance monitoring
        # Use configured threshold if available, otherwise default to 0.010 (10ms)
        blocking_threshold = getattr(config.safety, "max_loop_time_warning", 0.010)
        self.perf_monitor = PerformanceMonitor(
            self.vehicle_logger, blocking_threshold=blocking_threshold
        )

        # Command handler for centralized command processing
        self.command_handler = CommandHandler(self.vehicle_logger, config)

        # V2V Manager - Complete V2V system (handles communication internally)
        v2v_config = V2VBroadcastConfig(
            local_state_frequency=25.0,  # Hz - High frequency for local states
            fleet_state_frequency=10.0,  # Hz - Lower frequency for fleet states
            trust_report_frequency=2.0,  # Hz - Independent trust opinion exchange
            heartbeat_frequency=1.0,  # Hz - Very low frequency for heartbeats
        )
        base_v2v_manager = V2VManager(
            vehicle_id=config.network.car_id,
            vehicle_logger=self.vehicle_logger,
            config=v2v_config,
            vehicle_observer=None,  # Will be set later when vehicle_observer is created
            base_port=8000,
            status_callback=self._handle_v2v_status_change,
            vehicle_logic=self,  # Pass reference to self for Ground Station reporting
        )

        try:
            from V2V.AttackModule.V2VAttackInjector import V2VAttackInjector
            attack_config_path = os.path.join(os.path.dirname(__file__), "V2V", "AttackModule", "attack_config.yaml")
            self.v2v_manager = V2VAttackInjector(
                v2v_manager=base_v2v_manager,
                attack_config_path=attack_config_path,
                # enabled=config.vehicle.enable_v2v_attack  # Uses fleet_config.yaml
                enabled = False
            )
            self.vehicle_logger.logger.info("V2VAttackInjector successfully wrapped V2VManager.")
        except Exception as e:
            self.vehicle_logger.logger.error(f"Failed to initialize V2VAttackInjector: {e}")
            self.v2v_manager = base_v2v_manager

        # Safety systems
        self.watchdog = WatchdogTimer(
            config.safety.watchdog_timeout, self.vehicle_logger
        )

        # Components (initialized later)
        self.vehicle_logger.logger.info("Creating Ground Station client...")

        self._initialize_network_2_GroundStation()
        self.logger.logger.info("Initializing QCar hardware...")

        self.qcar = None
        self.gps = None

        # YOLO Manager - handles all YOLO-related functionality
        self.yolo_manager = YOLOManager(self.vehicle_logger)

        # Path planning
        self.roadmap = None
        self.waypoint_sequence = None
        self.node_sequence = None

        # Platoon controller
        platoon_config = PlatoonConfig()
        self.platoon_controller = PlatoonController(platoon_config, self.vehicle_logger)
        # Controller Manager - centralized tracking of active controllers
        self.controller_manager = ControllerManager(
            logger=self.vehicle_logger,
            vehicle_type=self.vehicle_type,
            vehicle_geometry=getattr(config, "vehicle_geometry", None),
        )

        pid_params = self.controller_manager.config._get_pid_params()
        self.v_ref = pid_params.get("v_ref", 0.6)
        self.v_ref_actual = 0.0  # Tracks actual computed target speed for scope
        self.controller_manager.set_vehicle_logic(self)  # For waypoint access

        # Taxi Manager
        self.taxi_manager = TaxiManager()

        # Calibration state flag (set by CALIBRATE command)
        self.calibration_requested = False

        # State machine - use simplified version with internal transition logic
        self.state_machine = VehicleStateMachine(self, self.vehicle_logger)

        # Gear state
        self.gear = Gear.DRIVE_1

        # Timing
        self.start_time = time.time()
        self.loop_counter = 0
        self.telemetry_counter = 0

        # Component update rates and timing
        self.controller_rate = (
            config.timing.controller_update_rate if IS_PHYSICAL_QCAR else 100
        )  # 200 for real vehicle, 100 for sim
        self.observer_rate = (
            config.timing.observer_rate if IS_PHYSICAL_QCAR else 100
        )  # 200 for real vehicle, 100 for sim
        self.telemetry_send_rate = getattr(config.timing, "telemetry_send_rate", 10)

        # Timing trackers for different update rates
        self._last_observer_time = 0.0
        self._last_control_time = 0.0
        self._last_telemetry_send = 0.0
        self._last_command_poll_time = 0.0
        self._last_v2v_broadcast_attempt = 0.0

        # V2V status cache
        self._v2v_status_cache = {}
        self._v2v_status_cache_time = 0.0
        self._latest_gui_attack_status = None

        # Periodic status broadcast tracking
        self._last_status_broadcast_time = 0.0
        self._status_broadcast_rate = 1.0  # 1 Hz
        self._command_poll_rate = min(float(self.controller_rate), 50.0)
        self._v2v_broadcast_attempt_rate = min(
            float(self.controller_rate),
            max(20.0, 2.0 * float(v2v_config.local_state_frequency)),
        )

        # Low-churn periodic status cache
        self._periodic_status_static_fields = {}
        self._periodic_status_static_fields_dirty = True

        # Pre-allocate telemetry dict to save memory
        self._telemetry_state = {
            "timestamp": 0.0,
            "time": 0.0,
            "vehicle_type": self.vehicle_type,
            "programme_type": self.programme_type,
            "x": 0.0,
            "y": 0.0,
            "th": 0.0,
            "v": 0.0,
            "u": 0.0,
            "delta": 0.0,
            "state": "UNKNOWN",
            "gps_valid": False,
        }
        self._latest_observer_state = None

        # Initialize Vehicle Observer for local and fleet state estimation
        # VehicleObserver is a manager class that coordinates:
        #   - LocalStateEstimator: Pluggable local state estimation (EKF, Luenberger, etc.)
        #   - FleetStateEstimator: Pluggable fleet estimation (Consensus, Distributed Kalman, etc.)
        # Fleet size starts at 1 and will be expanded when V2V activates
        # Local estimator will be initialized later in INITIALIZING state with GPS data

        self.vehicle_observer = VehicleObserver(
            vehicle_id=config.network.car_id,
            config=config,
            logger=self.vehicle_logger,
        )

        # Connect VehicleObserver to V2VManager
        self.v2v_manager.update_vehicle_observer(self.vehicle_observer)

        # Initialize the event system to connect command_handler to state_machine
        # This allows ground station commands to be properly routed to the current state
        self.state_machine.initialize_event_system()

        # Online SysID (ZMQ mode) is disabled by default.
        # It is created/activated only when commanded at runtime.
        self.online_sysid_zmq = None  # Separated ZMQ mode

        # Online Calibration (ZMQ mode) — passive data collection.
        # Activated at runtime via ENABLE_ONLINE_CALIBRATION command.
        self.online_calibration_zmq = None

        # Robust KalmanNet offline dataset recorder.
        self.robust_kalmannet_dataset = None

        # Opponent detection & tracking (from LiDAR tracker)
        self.opponent_data = []           # List of tracked opponent dicts
        self._last_opponent_update = 0.0  # Timestamp of last opponent data
        self.opponent_timeout = 1.0       # seconds before data is considered stale

    def elapsed_time(self) -> float:
        """Get elapsed time since start"""
        return time.time() - self.start_time

    @property
    def logger(self):
        """Backward compatibility property for accessing the vehicle logger"""
        return self.vehicle_logger

    # ===== Opponent Detection & Tracking =====

    def update_opponent_data(self, raw_data: list):
        """Parse tracked opponent Float32MultiArray into structured dicts.

        Each opponent is encoded as 10 floats:
            [id, x, y, size, vx, vy, is_static, is_visible, distance, confidence]
        """
        FIELDS_PER_OPPONENT = 10
        opponents = []
        if len(raw_data) >= FIELDS_PER_OPPONENT:
            n = len(raw_data) // FIELDS_PER_OPPONENT
            for i in range(n):
                offset = i * FIELDS_PER_OPPONENT
                opponents.append({
                    'id': int(raw_data[offset]),
                    'x': float(raw_data[offset + 1]),
                    'y': float(raw_data[offset + 2]),
                    'size': float(raw_data[offset + 3]),
                    'vx': float(raw_data[offset + 4]),
                    'vy': float(raw_data[offset + 5]),
                    'is_static': raw_data[offset + 6] >= 0.5,
                    'is_visible': raw_data[offset + 7] >= 0.5,
                    'distance': float(raw_data[offset + 8]),
                    'confidence': float(raw_data[offset + 9]),
                })
        self.opponent_data = opponents
        self._last_opponent_update = time.time()

    def get_nearest_opponent(self) -> Optional[dict]:
        """Get the closest dynamic opponent, or None if no opponents detected or data is stale."""
        if time.time() - self._last_opponent_update > self.opponent_timeout:
            return None
        dynamic_opps = [o for o in self.opponent_data if not o['is_static']]
        if not dynamic_opps:
            return None
        return min(dynamic_opps, key=lambda o: o['distance'])

    def get_all_opponents(self) -> list:
        """Get all tracked opponents (both static and dynamic), or empty list if stale."""
        if time.time() - self._last_opponent_update > self.opponent_timeout:
            return []
        return list(self.opponent_data)

    # ===== Attack V2V sim Module  =====

    def _get_v2v_log_time_s(self) -> float:
        """Return the shared V2V-reference time used by trust CSV logs."""
        try:
            if hasattr(self, "vehicle_observer") and self.vehicle_observer is not None:
                if hasattr(self.vehicle_observer, "to_v2v_reference_time_ns"):
                    return max(
                        float(self.vehicle_observer.to_v2v_reference_time_ns()), 0.0
                    ) / 1e9
        except Exception:
            pass

        if hasattr(self, "v2v_manager") and hasattr(self.v2v_manager, "get_elapsed_time"):
            return float(self.v2v_manager.get_elapsed_time())
        return self.elapsed_time()

    def _sync_v2v_attack_status_to_observer(self, attack_status: dict = None):
        """Forward GUI/config V2V attack metadata into the trust-fleet CSV logger."""
        try:
            if not hasattr(self, "vehicle_observer") or self.vehicle_observer is None:
                return

            shared_clock_s = self._get_v2v_log_time_s()
            runtime_status = None
            if hasattr(self, "v2v_manager") and hasattr(
                self.v2v_manager, "get_attack_status"
            ):
                runtime_status = self.v2v_manager.get_attack_status()
            status = attack_status
            if (
                isinstance(status, dict)
                and (
                    status.get("all_scenario_details")
                    or status.get("active_scenario_details")
                )
            ):
                self._latest_gui_attack_status = status.copy()
            if status is None:
                if self._latest_gui_attack_status is not None:
                    status = dict(self._latest_gui_attack_status)
                    status["elapsed_time"] = shared_clock_s
                    status["current_time"] = shared_clock_s
                elif isinstance(runtime_status, dict):
                    status = dict(runtime_status)
                else:
                    status = None

            if isinstance(status, dict) and isinstance(runtime_status, dict):
                runtime_snapshot = runtime_status.get("attack_value_snapshot", {})
                runtime_has_live_data = bool(
                    (
                        isinstance(runtime_snapshot, dict)
                        and runtime_snapshot.get("by_vehicle")
                    )
                    or runtime_status.get("active_scenario_details")
                    or runtime_status.get("all_scenario_details")
                    or runtime_status.get("active_scenarios")
                    or runtime_status.get("total_scenarios")
                )
                merged_runtime_keys = (
                    "attack_value_snapshot",
                    "active_scenario_details",
                    "all_scenario_details",
                    "current_scenario_names",
                    "statistics",
                    "injector_stats",
                    "attack_module_present",
                    "active_scenarios",
                    "total_scenarios",
                )
                for key in merged_runtime_keys:
                    runtime_value = runtime_status.get(key)
                    if runtime_value in (None, "", [], {}):
                        continue
                    status[key] = runtime_value
                for key in ("enabled", "attack_active"):
                    if runtime_has_live_data and key in runtime_status:
                        status[key] = runtime_status[key]

            if (
                isinstance(status, dict)
                and self._latest_gui_attack_status is not None
                and not status.get("attack_active", False)
                and not status.get("all_scenario_details")
                and not status.get("active_scenario_details")
            ):
                merged_status = dict(self._latest_gui_attack_status)
                if "manual_disable_time" in status:
                    merged_status.update(status)
                else:
                    merged_status["elapsed_time"] = shared_clock_s
                    merged_status["current_time"] = shared_clock_s
                status = merged_status
                if "manual_disable_time" in status:
                    self._latest_gui_attack_status = status.copy()

            if isinstance(status, dict) and "manual_disable_time" not in status:
                status = status.copy()
                status["elapsed_time"] = shared_clock_s
                status["current_time"] = shared_clock_s

            if status is None:
                return

            fleet_estimator = None
            if hasattr(self.vehicle_observer, "get_fleet_estimator"):
                fleet_estimator = self.vehicle_observer.get_fleet_estimator()

            if fleet_estimator is None or not hasattr(
                fleet_estimator, "set_v2v_attack_status"
            ):
                return

            fleet_estimator.set_v2v_attack_status(status)
        except Exception as e:
            if getattr(self, "loop_counter", 0) % 100 == 0:
                self.vehicle_logger.log_warning(
                    f"Failed to sync V2V attack status to trust logger: {e}"
                )

    @staticmethod
    def _attack_enum_value(value):
        return getattr(value, "value", value)

    def _build_gui_attack_status(
        self,
        scenario,
        clock_s: float,
        log_start_s: float = None,
        log_end_s: float = None,
        enabled: bool = True,
        active: bool = True,
    ) -> dict:
        """Build trust-log metadata from a GUI-triggered scenario."""
        start_s = float(clock_s if log_start_s is None else log_start_s)
        end_s = (
            float(getattr(scenario, "t_end", float("inf")))
            if log_end_s is None
            else float(log_end_s)
        )
        return {
            "enabled": bool(enabled),
            "attack_active": bool(active),
            "elapsed_time": float(clock_s),
            "all_scenario_details": [
                {
                    "name": getattr(scenario, "scenario_name", ""),
                    "type": self._attack_enum_value(getattr(scenario, "attack_type", "")),
                    "modification": self._attack_enum_value(
                        getattr(scenario, "modification_type", "")
                    ),
                    "data_type": self._attack_enum_value(
                        getattr(scenario, "data_type", "")
                    ),
                    "target_fields": list(getattr(scenario, "target_fields", [])),
                    "t_start": start_s,
                    "t_end": end_s,
                    "attacker_id": int(getattr(scenario, "attacker_id", -1)),
                    "victim_ids": list(getattr(scenario, "victim_ids", [])),
                    "active": bool(active),
                }
            ],
        }

    def enable_attack_module(self):
        """Enable V2V attack injection if available."""
        if hasattr(self.v2v_manager, "enable_attacks"):
            self.v2v_manager.enable_attacks()
            self._sync_v2v_attack_status_to_observer()
            self.vehicle_logger.logger.info("V2V Attack Module ENABLED.")
        else:
            self.vehicle_logger.logger.warning("V2V Attack Module is not available.")

    def trigger_v2v_attack(self, data: dict):
        """Trigger a specific V2V attack from Ground Station"""
        try:
            if not hasattr(self, 'v2v_manager') or not hasattr(self.v2v_manager, 'add_attack_scenario'):
                self.vehicle_logger.logger.warning("V2V Attack Module not available")
                return

            from V2V.AttackModule.AttackScenarios import build_attack_scenario

            attack_type = str(data.get('attack_type', 'Bogus')).strip()
            case_num = int(data.get('case_num', 1))
            data_type = str(data.get('data_type', 'local')).strip().lower()

            attacker_id_raw = data.get('attacker_id', self.vehicle_id)
            if attacker_id_raw in (None, "", "all"):
                requested_attacker_id = -1
            else:
                requested_attacker_id = int(attacker_id_raw)

            # Parse victims properly
            victim_ids = data.get('victim_ids', [])
            if isinstance(victim_ids, str):
                try:
                    import json
                    victim_ids = json.loads(victim_ids)
                except Exception:
                    victim_ids = []

            if not isinstance(victim_ids, list):
                victim_ids = [victim_ids]

            normalized_victim_ids = []
            for victim_id in victim_ids:
                if victim_id in (None, "", "all", -1):
                    normalized_victim_ids = -1
                    break
                normalized_victim_ids.append(int(victim_id))

            if normalized_victim_ids == []:
                normalized_victim_ids = -1

            # Manual trigger semantics: start now and stay active until the user disables it.
            # The injector still uses its local elapsed clock, while logs use the
            # shared V2V-reference clock so CSVs compare across vehicles.
            injector_t_start = (
                self.v2v_manager.get_elapsed_time()
                if hasattr(self.v2v_manager, 'get_elapsed_time')
                else self.elapsed_time()
            )
            log_t_start = self._get_v2v_log_time_s()
            scenario_attacker_id = (
                int(self.vehicle_id)
                if requested_attacker_id == -1
                else requested_attacker_id
            )
            scenario = build_attack_scenario(
                attack_type=attack_type,
                case_number=case_num,
                attacker_id=scenario_attacker_id,
                victim_ids=normalized_victim_ids,
                t_start=injector_t_start,
                t_end=float("inf"),
                data_type=data_type,
            )

            self._sync_v2v_attack_status_to_observer(
                self._build_gui_attack_status(
                    scenario,
                    clock_s=log_t_start,
                    log_start_s=log_t_start,
                )
            )

            # This function can run on every car; only the attacker injects data.
            if requested_attacker_id not in (int(self.vehicle_id), -1):
                return

            if hasattr(self.v2v_manager, 'clear_attack_scenarios'):
                self.v2v_manager.clear_attack_scenarios()
            self.v2v_manager.add_attack_scenario(scenario)
            self.v2v_manager.enable_attacks()
            self._sync_v2v_attack_status_to_observer()

            self.vehicle_logger.logger.info(
                f"Triggered manual {attack_type} attack (Case {case_num}) "
                f"starting at V2V t={log_t_start:.3f}s and running until disabled. "
                f"Victims: {normalized_victim_ids}"
            )
        except Exception as e:
            self.vehicle_logger.log_error(f"Failed to trigger V2V attack", e)

    def disable_v2v_attack(self, data: dict = None):
        """Stop V2V attack injection without resetting observer trust state."""
        try:
            if hasattr(self, 'v2v_manager') and hasattr(self.v2v_manager, 'disable_attacks'):
                disable_time = self._get_v2v_log_time_s()
                status = (
                    dict(self._latest_gui_attack_status)
                    if self._latest_gui_attack_status is not None
                    else (
                        self.v2v_manager.get_attack_status()
                        if hasattr(self.v2v_manager, 'get_attack_status')
                        else {}
                    )
                )
                if isinstance(status, dict):
                    status = status.copy()
                    status["enabled"] = False
                    status["attack_active"] = False
                    status["elapsed_time"] = disable_time
                    status["current_time"] = disable_time
                    status["manual_disable_time"] = disable_time
                    self._sync_v2v_attack_status_to_observer(status)
                self.v2v_manager.disable_attacks()
                if hasattr(self.v2v_manager, 'clear_attack_scenarios'):
                    self.v2v_manager.clear_attack_scenarios()
                self.vehicle_logger.logger.info(
                    "Disabled V2V attack injection via Ground Station; "
                    "observer trust/rollback recovery remains automatic"
                )
        except Exception as e:
            self.vehicle_logger.log_error("Failed to disable V2V attack", e)

    def _get_local_sensor_attack_status(self) -> dict:
        """Get dynamic local sensor attack status from the active local estimator."""
        default_status = {
            "local_sensor_attack_supported": False,
            "local_sensor_attack_enabled": False,
            "local_sensor_attack_active": False,
            "local_sensor_attack_branch_types": "",
            "local_sensor_attack_gps_type": "",
            "local_sensor_attack_remaining_steps": 0,
            "local_sensor_attack_intensity": 0.0,
        }
        vehicle_observer = getattr(self, "vehicle_observer", None)
        if vehicle_observer is None or not hasattr(
            vehicle_observer, "get_local_sensor_attack_status"
        ):
            return default_status

        try:
            status = vehicle_observer.get_local_sensor_attack_status()
        except Exception as e:
            self.vehicle_logger.log_warning(
                f"Failed to get local sensor attack status: {e}"
            )
            return default_status

        if not isinstance(status, dict):
            return default_status

        merged_status = default_status.copy()
        merged_status.update(status)
        return merged_status

    def _get_local_sensor_attack_status_compact(self) -> dict:
        """Return only the low-churn local sensor attack fields intended for UI status."""
        status = self._get_local_sensor_attack_status()
        allowed_keys = (
            "local_sensor_attack_supported",
            "local_sensor_attack_enabled",
            "local_sensor_attack_active",
            "local_sensor_attack_branch_types",
            "local_sensor_attack_gps_type",
            "local_sensor_attack_remaining_steps",
            "local_sensor_attack_intensity",
        )
        return {key: status.get(key) for key in allowed_keys}

    def start_local_sensor_attack(self, config: Optional[dict] = None) -> bool:
        """Enable runtime RKNet local sensor attack injection for this vehicle."""
        vehicle_observer = getattr(self, "vehicle_observer", None)
        if vehicle_observer is None:
            self.vehicle_logger.logger.warning(
                "Local sensor attack ignored: vehicle observer is not initialized"
            )
            return False

        estimator = vehicle_observer.get_local_estimator()
        estimator_type = str(
            getattr(vehicle_observer, "local_estimator_type", "")
        ).strip()
        if estimator_type != "robust_kalman_net" or not hasattr(
            estimator, "start_sensor_attack"
        ):
            self.vehicle_logger.logger.warning(
                "Local sensor attack ignored: active local observer is not robust_kalman_net"
            )
            return False

        attack_config = config if isinstance(config, dict) else {}
        success = bool(estimator.start_sensor_attack(attack_config))
        if success:
            self.vehicle_logger.logger.info(
                f"Local sensor attack enabled for vehicle {self.vehicle_id}"
            )
        return success

    def stop_local_sensor_attack(self) -> bool:
        """Disable runtime RKNet local sensor attack injection for this vehicle."""
        vehicle_observer = getattr(self, "vehicle_observer", None)
        if vehicle_observer is None:
            self.vehicle_logger.logger.warning(
                "Local sensor attack stop ignored: vehicle observer is not initialized"
            )
            return False

        estimator = vehicle_observer.get_local_estimator()
        estimator_type = str(
            getattr(vehicle_observer, "local_estimator_type", "")
        ).strip()
        if estimator_type != "robust_kalman_net" or not hasattr(
            estimator, "stop_sensor_attack"
        ):
            self.vehicle_logger.logger.warning(
                "Local sensor attack stop ignored: active local observer is not robust_kalman_net"
            )
            return False

        success = bool(estimator.stop_sensor_attack())
        if success:
            self.vehicle_logger.logger.info(
                f"Local sensor attack disabled for vehicle {self.vehicle_id}"
            )
        return success

    def disable_attack_module(self):
        """Disable V2V attack injection if available."""
        if hasattr(self.v2v_manager, "disable_attacks"):
            disable_time = self._get_v2v_log_time_s()
            status = (
                dict(self._latest_gui_attack_status)
                if self._latest_gui_attack_status is not None
                else (
                    self.v2v_manager.get_attack_status()
                    if hasattr(self.v2v_manager, "get_attack_status")
                    else {}
                )
            )
            if isinstance(status, dict):
                status = status.copy()
                status["enabled"] = False
                status["attack_active"] = False
                status["elapsed_time"] = disable_time
                status["current_time"] = disable_time
                status["manual_disable_time"] = disable_time
                self._sync_v2v_attack_status_to_observer(status)
            self.v2v_manager.disable_attacks()
            self.vehicle_logger.logger.info(
                "V2V Attack Module DISABLED; observer trust/rollback recovery remains automatic."
            )
        else:
            self.vehicle_logger.logger.warning("V2V Attack Module is not available.")

    def _guess_sysid_racecar_version(self) -> str:
        """
        Pick a model folder for On_Track_SysID.
        Priority: NUC{vehicle_id} if present, otherwise SIM.
        """
        base_dir = os.path.join(
            os.path.dirname(__file__), "Calibration", "On_Track_SysID", "models"
        )
        candidate = f"NUC{self.vehicle_id}"
        if os.path.isdir(os.path.join(base_dir, candidate)):
            return candidate
        if os.path.isdir(os.path.join(base_dir, "SIM")):
            return "SIM"
        return candidate

    def enable_online_sysid_zmq(self, config: dict = None) -> bool:
        """
        Enable separated Online SysID transport over ZMQ.
        Creates sockets only when requested by command.
        """
        cfg = config or {}
        if self.online_sysid_zmq is None:
            if OnlineSysIDZMQClient is None:
                self.vehicle_logger.log_warning(
                    "Online SysID ZMQ client unavailable (import failed)"
                )
                return False

            sample_port = int(cfg.get("sample_port", 18880))
            control_port = int(cfg.get("control_port", 18881))
            status_host = str(cfg.get("status_host", "127.0.0.1"))
            status_port = int(cfg.get("status_port", 18882))
            bind_ip = str(cfg.get("bind_ip", "*"))

            self.online_sysid_zmq = OnlineSysIDZMQClient(
                logger=self.vehicle_logger,
                vehicle_id=self.vehicle_id,
                sample_port=sample_port,
                control_port=control_port,
                status_host=status_host,
                status_port=status_port,
                bind_ip=bind_ip,
            )

        started = self.online_sysid_zmq.start()
        if not started:
            return False

        # Optionally forward runtime training config to worker.
        worker_cfg = cfg.get("worker_config", {})
        if isinstance(worker_cfg, dict) and worker_cfg:
            self.online_sysid_zmq.update_config(worker_cfg)
        return True

    def disable_online_sysid_zmq(self) -> None:
        """Disable and close Online SysID ZMQ transport."""
        if self.online_sysid_zmq is not None:
            try:
                self.online_sysid_zmq.stop()
            except Exception as e:
                self.vehicle_logger.log_error(
                    "Failed to stop Online SysID ZMQ client", e
                )
            self.online_sysid_zmq = None

    def _get_online_sysid_status(self) -> dict:
        """Collect status from the Online SysID ZMQ backend."""
        status = {"enabled": False}

        if self.online_sysid_zmq is not None:
            status["enabled"] = True
            status["mode"] = "zmq"
            status["zmq"] = self.online_sysid_zmq.get_status()

        return status

    # ===== Online Calibration (passive data collection) =====
    def enable_online_calibration_zmq(self, config: dict = None) -> bool:
        """
        Enable passive online calibration transport over ZMQ.
        Creates sockets only when requested by command.
        """
        cfg = config or {}
        if self.online_calibration_zmq is None:
            if OnlineCalibrationZMQClient is None:
                self.vehicle_logger.log_warning(
                    "Online Calibration ZMQ client unavailable (import failed)"
                )
                return False

            sample_port = int(cfg.get("sample_port", 18890))
            control_port = int(cfg.get("control_port", 18891))
            status_host = str(cfg.get("status_host", "127.0.0.1"))
            status_port = int(cfg.get("status_port", 18892))
            bind_ip = str(cfg.get("bind_ip", "*"))

            self.online_calibration_zmq = OnlineCalibrationZMQClient(
                logger=self.vehicle_logger,
                vehicle_id=self.vehicle_id,
                sample_port=sample_port,
                control_port=control_port,
                status_host=status_host,
                status_port=status_port,
                bind_ip=bind_ip,
            )

        started = self.online_calibration_zmq.start_collection()
        if not started:
            return False

        self.vehicle_logger.logger.info(
            "[OnlineCal] Passive calibration data collection ENABLED"
        )
        return True

    def disable_online_calibration_zmq(self) -> None:
        """
        Pause passive calibration collection while keeping ZMQ transport alive.

        The GUI button is labelled "Pause" and users need to run Analyse after
        pausing. Closing the client here would drop the worker connection and
        make analyse/clear/status commands unavailable.
        """
        if self.online_calibration_zmq is not None:
            try:
                self.online_calibration_zmq.stop_collection()
            except Exception as e:
                self.vehicle_logger.log_error(
                    "Failed to pause Online Calibration ZMQ collection", e
                )
            self.vehicle_logger.logger.info(
                "[OnlineCal] Passive calibration data collection PAUSED"
            )

    def close_online_calibration_zmq(self) -> None:
        """Close Online Calibration ZMQ transport."""
        if self.online_calibration_zmq is not None:
            try:
                self.online_calibration_zmq.stop()
            except Exception as e:
                self.vehicle_logger.log_error(
                    "Failed to stop Online Calibration ZMQ client", e
                )
            self.online_calibration_zmq = None
            self.vehicle_logger.logger.info(
                "[OnlineCal] Passive calibration ZMQ transport CLOSED"
            )

    def _get_online_calibration_status(self) -> dict:
        """Collect status from the Online Calibration ZMQ backend."""
        status = {"enabled": False}
        if self.online_calibration_zmq is not None:
            status["enabled"] = True
            status["zmq"] = self.online_calibration_zmq.get_status()
        return status

    # ===== Robust KalmanNet offline dataset collection =====
    def enable_robust_kalmannet_dataset(self, config: dict = None) -> bool:
        """Enable local dataset recording for offline Robust KalmanNet training."""
        cfg = config or {}
        if RobustKalmanNetDatasetRecorder is None:
            self.vehicle_logger.log_warning(
                "Robust KalmanNet dataset recorder unavailable (import failed)"
            )
            return False

        target_type = str(cfg.get("target_estimator_type", "ekf")).strip() or "ekf"
        strict_target = bool(cfg.get("strict_target_type", True))
        current_type = getattr(self.vehicle_observer, "local_estimator_type", "unknown")
        if strict_target and current_type != target_type:
            self.vehicle_logger.log_warning(
                f"[RKNetDataset] Refusing to start: local observer is '{current_type}', expected '{target_type}'"
            )
            return False

        if self.robust_kalmannet_dataset is None:
            self.robust_kalmannet_dataset = RobustKalmanNetDatasetRecorder(
                vehicle_id=self.vehicle_id,
                logger=self.vehicle_logger,
            )

        return self.robust_kalmannet_dataset.start(cfg)

    def disable_robust_kalmannet_dataset(self, save: bool = True) -> Optional[str]:
        """Stop Robust KalmanNet dataset recording and optionally save it."""
        if self.robust_kalmannet_dataset is None:
            return None
        return self.robust_kalmannet_dataset.stop(save=save)

    def clear_robust_kalmannet_dataset(self) -> None:
        """Clear the Robust KalmanNet dataset buffer."""
        if self.robust_kalmannet_dataset is not None:
            self.robust_kalmannet_dataset.clear()

    def _get_robust_kalmannet_dataset_status(self) -> dict:
        """Collect status from the Robust KalmanNet dataset recorder."""
        status = {"enabled": False}
        if self.robust_kalmannet_dataset is not None:
            recorder_status = self.robust_kalmannet_dataset.get_status()
            status["enabled"] = True
            status.update(recorder_status)
        return status

    def _submit_robust_kalmannet_sample(self, timestamp: float) -> None:
        """Submit one synchronized sample to the offline Robust KalmanNet recorder."""
        recorder = self.robust_kalmannet_dataset
        if recorder is None or not recorder.recording:
            return
        if self.vehicle_observer is None:
            return
        if self.vehicle_observer.get_local_estimator() is None:
            return

        try:
            target_state = self.vehicle_observer.get_local_estimator().get_state()
            sensor = self.vehicle_observer.sensor_data
            gps_data = None
            if sensor.get("gps_has_fix", False):
                gps_pos = sensor.get("gps_position", np.zeros(3))
                gps_data = {
                    "x": float(gps_pos[0]),
                    "y": float(gps_pos[1]),
                    "theta": float(gps_pos[2]),
                    "valid": bool(sensor.get("gps_valid", False)),
                    "position_valid": bool(sensor.get("gps_fresh", False)),
                    "hold_valid": bool(sensor.get("gps_valid", False)),
                    "fresh": bool(sensor.get("gps_fresh", False)),
                    "age_sec": float(sensor.get("gps_age", float("inf"))),
                    "has_fix": True,
                }

            recorder.record_sample(
                timestamp=timestamp,
                motor_tach=float(sensor.get("motor_tach", 0.0)),
                steering=float(getattr(self, "_last_steering", 0.0)),
                throttle=float(getattr(self, "_last_u", 0.0)),
                gyro_z=float(sensor.get("gyro_z", 0.0)),
                acceleration=sensor.get("accelerometer", np.zeros(3)),
                gps_data=gps_data,
                target_state=target_state,
                current_estimator_type=str(
                    getattr(self.vehicle_observer, "local_estimator_type", "unknown")
                ),
            )
        except Exception as e:
            self.vehicle_logger.log_error(
                "Failed to submit Robust KalmanNet dataset sample", e
            )

    def run(self):
        """Main control loop"""
        # self.vehicle_logger.logger.info("Starting control loop...")

        # Start the state machine in INITIALIZING state
        # The state machine will handle all initialization through its states

        # CRITICAL: Reset start_time NOW
        self.start_time = time.time()
        self.vehicle_logger.set_start_time(
            self.start_time
        )  # Set reference for relative timestamps
        self.loop_counter = 0
        self.telemetry_counter = 0
        self._latest_observer_state = None

        # Main control loop
        target_dt = 1.0 / self.controller_rate
        last_loop_time = time.time()

        try:
            while not self.kill_event.is_set():
                loop_start = time.time()
                actual_dt = loop_start - last_loop_time
                last_loop_time = loop_start

                # Reset watchdog
                if hasattr(self, "watchdog"):
                    self.watchdog.reset()

                # 1. Sensor Data Reading and GPS Update
                # 2. Observer Update (handles both local and fleet internally)
                if self._should_update_observer(loop_start):
                    self._update_sensor_data(actual_dt)
                    self._observer_update(actual_dt)

                # 3. Control Logic (high frequency)
                if self._should_update_control(loop_start):
                    if not self._control_logic_update(actual_dt):
                        self.vehicle_logger.log_error("Control logic failed")
                        break

                # 4. Communication Tasks (each manages own rate internally)
                self._send_telemetry_to_ground_station()
                self._broadcast_periodic_status()
                self._process_queued_commands()
                self._broadcast_v2v_state()

                # Performance monitoring
                loop_time = time.time() - loop_start
                if hasattr(self, "perf_monitor"):
                    self.perf_monitor.log_loop_time(loop_time)

                # Sleep to maintain loop rate
                sleep_time = target_dt - loop_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

                self.loop_counter += 1

                # Check if experiment time exceeded
                if self.elapsed_time() > self.config.timing.tf:
                    self.vehicle_logger.logger.info("Experiment time limit reached")
                    break

        except KeyboardInterrupt:
            self.vehicle_logger.logger.info("Control loop interrupted by user")
        except Exception as e:
            self.vehicle_logger.log_error("Control loop error", e)
        finally:
            self._shutdown()

            # # Stop scope manager
            # if self.scope_manager:
            #     self.scope_manager.stop()

    # ===== Component Update Rate Control Methods =====
    def _should_update_observer(self, current_time: float) -> bool:
        """Check if local observer should update based on rate"""
        if current_time - self._last_observer_time >= 1.0 / self.observer_rate:
            self._last_observer_time = current_time
            return True
        return False

    def _should_update_control(self, current_time: float) -> bool:
        """Check if control should update based on rate"""
        if current_time - self._last_control_time >= 1.0 / self.controller_rate:
            self._last_control_time = current_time
            return True
        return False

    def _should_poll_commands(self, current_time: float) -> bool:
        """Poll queued commands at a bounded rate to reduce loop overhead."""
        if current_time - self._last_command_poll_time >= 1.0 / self._command_poll_rate:
            self._last_command_poll_time = current_time
            return True
        return False

    def _should_attempt_v2v_broadcast(self, current_time: float) -> bool:
        """Attempt V2V broadcasts near the configured max cadence, not every control tick."""
        if current_time - self._last_v2v_broadcast_attempt >= (
            1.0 / self._v2v_broadcast_attempt_rate
        ):
            self._last_v2v_broadcast_attempt = current_time
            return True
        return False

    def _get_cached_control_state(self) -> dict:
        """Return the latest observer snapshot, falling back to the observer cache."""
        cached_state = self._latest_observer_state
        if cached_state is not None:
            local_state = cached_state.get("local_state")
            acceleration = 0.0
            if local_state is not None and len(local_state) > 4:
                acceleration = float(local_state[4])

            return {
                "x": float(cached_state["x"]),
                "y": float(cached_state["y"]),
                "theta": float(cached_state["theta"]),
                "velocity": float(cached_state["velocity"]),
                "acceleration": acceleration,
                "gps_valid": bool(cached_state.get("gps_valid", False)),
            }

        return self.vehicle_observer.get_estimated_state_for_control()

    # ===== Sensor Data Reading Methods =====
    def _update_sensor_data(self, dt: float):
        """Update sensor data using VehicleObserver - called every loop iteration"""
        try:
            if self.qcar is not None:
                # Use VehicleObserver to update sensor data
                self.vehicle_observer.update_sensor_data(self.qcar)

                # Handle YOLO logic using YOLOManager (only if enabled)
                if self.yolo_manager.yolo_enabled:
                    self.yolo_manager.update(self.loop_counter)
                    if self.vehicle_observer is not None:
                        target_id = int(self.vehicle_id) - 1
                        if target_id >= 0:
                            rel_meas = self.yolo_manager.get_relative_car_measurement()
                            if rel_meas is not None:
                                self.vehicle_observer.update_relative_measurement(
                                    measurement=np.array(
                                        [
                                            rel_meas["distance"],
                                            rel_meas["relative_velocity"],
                                        ],
                                        dtype=float,
                                    ),
                                    target_id=target_id,
                                    source=str(rel_meas["source"]),
                                    measurement_confidence=float(
                                        rel_meas.get("confidence", float("nan"))
                                    ),
                                    timestamp_ns=rel_meas.get("timestamp_ns"),
                                )

        except Exception as e:
            self.vehicle_logger.log_error("Sensor data update error", e)

    # ===== Observer Update Methods =====
    def _observer_update(self, dt: float):
        """Unified observer update - handles both local and fleet observer internally"""
        try:
            # Skip observer update if local estimator not initialized yet
            if (
                self.state_machine.state == VehicleState.INITIALIZING
                or self.vehicle_observer.get_local_estimator() is None
            ):
                return  # Observer not ready yet (still in INITIALIZING state)

            # Get last steering command for EKF
            last_steering = getattr(self, "_last_steering", 0.0)
            last_u = getattr(self, "_last_u", 0.0)

            self._sync_v2v_attack_status_to_observer()

            # Update observer (internally handles local and fleet timing)
            state_info = self.vehicle_observer.update_observer(
                dt, last_steering, last_u
            )
            self._latest_observer_state = state_info

            # Feed Online SysID with latest [v_x, v_y, omega, delta] sample.
            if hasattr(self, "online_sysid_zmq") and self.online_sysid_zmq:
                if self.online_sysid_zmq.is_collecting():
                    sample = self.vehicle_observer.get_online_sysid_sample(dt=dt)
                    if sample is not None:
                        self.online_sysid_zmq.submit_sample(sample)

            # Feed passive calibration with filtered motion/control references.
            if hasattr(self, "online_calibration_zmq") and self.online_calibration_zmq:
                if self.online_calibration_zmq.is_collecting():
                    cal_sample = self.vehicle_observer.get_calibration_sample()
                    if cal_sample is not None:
                        self.online_calibration_zmq.submit_sample(cal_sample)

            self._submit_robust_kalmannet_sample(time.time())

            # Stream scope data to Ground Station (if streaming enabled)
            if (
                hasattr(self, "scope_streamer")
                and self.scope_streamer
                and self.scope_streamer.is_streaming()
            ):
                # Build streaming data (similar to vis_data but can be separate)
                stream_data = state_info.copy()

                # Add GPS reference
                sensor_data = self.vehicle_observer.get_sensor_data()
                if sensor_data.get("gps_valid", False):
                    stream_data["x_gps"] = sensor_data["gps_position"][0]
                    stream_data["y_gps"] = sensor_data["gps_position"][1]
                    stream_data["theta_gps"] = sensor_data["gps_position"][2]

                accel = np.asarray(
                    sensor_data.get("accelerometer", np.zeros(3)), dtype=float
                ).reshape(-1)
                if accel.size < 3:
                    accel = np.pad(accel, (0, 3 - accel.size), mode="constant")
                accel = accel[:3]
                local_state = stream_data.get("local_state")
                if local_state is not None and len(local_state) > 4:
                    stream_data["acceleration"] = float(local_state[4])
                else:
                    stream_data["acceleration"] = float(accel[0])
                stream_data["accel_magnitude"] = float(
                    sensor_data.get("accel_magnitude", np.linalg.norm(accel))
                )

                # Add control signals
                stream_data["v_ref"] = self.v_ref * self.yolo_manager.get_yolo_gain()
                stream_data["v_ref_actual"] = getattr(self, "v_ref_actual", stream_data["v_ref"])
                stream_data["steering"] = last_steering
                stream_data["throttle"] = last_u

                # Add fleet data if V2V is active
                if self.vehicle_observer.v2v_active:
                    stream_data["fleet_states"] = (
                        self.vehicle_observer.get_fleet_states()
                    )
                    stream_data["fleet_size"] = self.vehicle_observer.fleet_size

                # Stream to Ground Station (rate-limited internally)
                self.scope_streamer.stream_sample(self.elapsed_time(), stream_data)

            # Log observer state occasionally

            # # Log observer state occasionally
            # if self.loop_counter % 300 == 0:  # Every 3 seconds at 100Hz (reduced logging)
            #     self.vehicle_logger.logger.debug(
            #         f"Observer: Pos=({state_info['x']:.2f}, {state_info['y']:.2f}, {state_info['theta']:.2f}), "
            #         f"Vel={state_info['velocity']:.2f}, GPS_Valid={state_info['gps_valid']}"
            #     )

            #     # Log fleet observer status
            #     fleet_states = self.vehicle_observer.get_fleet_states()
            #     active_vehicles = np.sum(np.any(fleet_states != 0, axis=0))
            #     self.vehicle_logger.logger.debug(f"Fleet Observer: {active_vehicles} active vehicles in fleet")

        except Exception as e:
            self.vehicle_logger.log_error("Observer update error", e)

    # ===== Control Logic Methods =====
    def _control_logic_update(self, dt: float) -> bool:
        """Control logic update - state machine and vehicle commands"""
        try:
            # Check if components are initialized
            # # If not still need
            # if self.state_machine.state == VehicleState.INITIALIZING or self.qcar is not None :
            #     sensor_data = {}
            #     # Update state machine - this will handle initialization
            #     self.state_machine.update(dt, sensor_data)
            #     return True
            #     # return False
            if self.qcar is None or (
                hasattr(self, "vehicle_observer")
                and self.vehicle_observer.get_local_estimator() is None
            ):
                return self._handle_initialization_control(dt)

            # Get current state from VehicleObserver
            sensor_data = self._get_cached_control_state()
            # Prepare YOLO data
            sensor_data["yolo_data"] = self.yolo_manager.get_yolo_data()

            # Update state machine - it handles all state logic and transitions
            u, delta = self.state_machine.update(dt, sensor_data)

            if u is None or delta is None:
                # self.vehicle_logger.log_error("State machine returned invalid control commands")
                return True  # Skip sending commands

            delta = float(np.clip(delta, -0.48, 0.48))  # Keep within physical limits

            self._last_steering = delta

            # if self.vehicle_type == "Limo":
            #     # Driver expects Ackermann steering angle [rad] on angular.z.
            #     # Keep compatibility with potential normalized outputs.
            #     delta_input = float(delta)
            #     if abs(delta_input) <= 0.55:
            #         delta = delta_input
            #     else:
            #         delta = max(-1.0, min(1.0, delta_input)) * 0.48869
            #     delta = float(np.clip(delta, -0.48869, 0.48869))
            # else:
            #     delta = max(-1.0, min(1.0, delta))


            if self.qcar is not None:
                if self.vehicle_type == "Limo":
                    # For Limo, u is a velocity command in m/s
                    u_clipped = float(np.clip(u, -0.3, 1.2)) # Max ~1.2 m/s
                    self.qcar.write(throttle=u_clipped, steering=delta)
                else:
                    max_throttle = float(getattr(self.gear, "value", 0.1))
                    if abs(u) > max_throttle:
                        u = np.clip(u, -max_throttle, max_throttle)
                    self.qcar.write(throttle=u, steering=delta)

            # Store steering and throttle for next EKF update and telemetry
            self._last_u = u

            return True

        except Exception as e:
            self.vehicle_logger.log_error("Control logic update error", e)
            return False

    def _handle_initialization_control(self, dt: float) -> bool:
        """Handle control during initialization phase"""
        try:
            # During initialization, just update state machine without sensor readings
            if (
                hasattr(self.state_machine, "state")
                and self.state_machine.state == VehicleState.INITIALIZING
            ):
                # Minimal sensor data for initialization
                sensor_data = {
                    "x": 0.0,
                    "y": 0.0,
                    "theta": 0.0,
                    "velocity": 0.0,
                    "motor_tach": 0.0,
                    "gyro_z": 0.0,
                    "yolo_data": self.yolo_manager.get_default_yolo_data(),
                    "gps_valid": False,
                }

                # Update state machine - this will handle initialization
                u, delta = self.state_machine.update(dt, sensor_data)

                # Don't send commands during initialization
                return True
            else:
                self.vehicle_logger.log_error(
                    "Components not initialized but not in INITIALIZING state"
                )
                return False

        except Exception as e:
            self.vehicle_logger.log_error("Initialization control error", e)
            return False

    def set_gear(self, gear: Gear):
        """Set the vehicle gear"""
        self.gear = gear
        self.vehicle_logger.logger.info(f"Gear changed to: {self.gear.name}")

    def get_operational_status(self) -> dict:
        """
        Get grouped operational status including state, gear, and velocity reference.
        This provides a unified view of the car's driving configuration.
        """
        return {
            # 'state': self.state_machine.state.name if hasattr(self.state_machine, 'state') and self.state_machine.state else 'UNKNOWN',
            "gear": self.gear.name,
            # 'velocity_ref': self.v_ref,
            # 'velocity': self.vehicle_observer.get_estimated_state_for_control()['velocity'] if hasattr(self, 'vehicle_observer') and self.vehicle_observer else 0.0
        }

    # ===== Communication Handling Methods =====
    def _send_telemetry_to_ground_station(self):
        """Send telemetry to Ground Station with internal 10Hz rate-limiting"""
        try:
            if not hasattr(self, "vehicle_observer") or self.vehicle_observer is None:
                return

            # Rate-limiting: only send at telemetry_send_rate (10Hz)
            current_time = time.time()
            telemetry_interval = 1.0 / self.telemetry_send_rate
            if current_time - self._last_telemetry_send < telemetry_interval:
                return  # Skip this cycle

            # Build telemetry data
            telemetry = self._build_telemetry_data()

            # Log to file
            if self.config.logging.enable_telemetry_logging:
                self.vehicle_logger.log_telemetry(telemetry)

            # Send to Ground Station
            if self.client_Ground_Station:
                try:
                    is_connected = getattr(
                        self.client_Ground_Station, "is_connected", lambda: True
                    )()
                    if is_connected:
                        self.client_Ground_Station.queue_telemetry(telemetry)
                        self.telemetry_counter += 1
                except Exception as e:
                    # Log errors occasionally to avoid spam
                    if self.loop_counter % 100 == 0:
                        self.vehicle_logger.log_error("Telemetry transmission error", e)

            self._last_telemetry_send = current_time

        except Exception as e:
            self.vehicle_logger.log_error("Telemetry sending error", e)

    def _build_telemetry_data(self) -> dict:
        """Build telemetry data dictionary - pure data collection"""
        # Update pre-allocated telemetry dictionary in-place
        state_info = self._get_cached_control_state()

        self._telemetry_state["timestamp"] = time.time()
        self._telemetry_state["time"] = self.elapsed_time()
        self._telemetry_state["x"] = float(state_info["x"])
        self._telemetry_state["y"] = float(state_info["y"])
        self._telemetry_state["th"] = float(state_info["theta"])
        self._telemetry_state["v"] = float(state_info["velocity"])
        self._telemetry_state["u"] = float(getattr(self, "_last_u", 0.0))
        self._telemetry_state["delta"] = float(getattr(self, "_last_steering", 0.0))
        
        self._telemetry_state["state"] = self.state_machine.state.name if hasattr(self.state_machine, "state") and self.state_machine.state else "UNKNOWN"
        self._telemetry_state["gps_valid"] = bool(state_info.get("gps_valid", False))

        # Include opponent tracking data in telemetry
        self._telemetry_state["opponents"] = self.opponent_data

        return self._telemetry_state

    def _get_v2v_status_cache(self) -> dict:
        """Get V2V status with caching to avoid repeated queries (updated every 1 second)"""
        current_time = time.time()

        # Update cache every 1 second (V2V status doesn't change frequently)
        if current_time - self._v2v_status_cache_time > 1.0:
            try:
                if hasattr(self, "v2v_manager") and self.v2v_manager:
                    is_active = self.v2v_manager.is_active()
                    self._v2v_status_cache = {
                        "v2v_active": is_active,
                        "v2v_peers": len(
                            self.v2v_manager.v2v_communication.peer_vehicles
                        )
                        if is_active
                        else 0,
                        "v2v_protocol": "UDP-Manager" if is_active else "None",
                        "v2v_local_rate": self.v2v_manager.config.local_state_frequency,
                        "v2v_fleet_rate": self.v2v_manager.config.fleet_state_frequency,
                        "v2v_trust_rate": self.v2v_manager.config.trust_report_frequency,
                    }
                else:
                    self._v2v_status_cache = {
                        "v2v_active": False,
                        "v2v_peers": 0,
                        "v2v_protocol": "None",
                        "v2v_local_rate": 0.0,
                        "v2v_fleet_rate": 0.0,
                        "v2v_trust_rate": 0.0,
                    }
                self._v2v_status_cache_time = current_time
            except Exception as e:
                self.vehicle_logger.logger.warning(
                    f"Error updating V2V status cache: {e}"
                )
                # Return safe defaults on error
                self._v2v_status_cache = {
                    "v2v_active": False,
                    "v2v_peers": 0,
                    "v2v_protocol": "None",
                    "v2v_local_rate": 0.0,
                    "v2v_fleet_rate": 0.0,
                    "v2v_trust_rate": 0.0,
                }

        return self._v2v_status_cache.copy()

    def _get_platoon_status(self) -> dict:
        """Get platoon status from platoon_controller for telemetry"""
        try:
            platoon_controller = getattr(self, "platoon_controller", None)
            if platoon_controller is None:
                return {
                    "platoon_enabled": False,
                    "platoon_is_leader": False,
                    "platoon_position": None,
                    "platoon_leader_id": None,
                    "platoon_setup_complete": False,
                }

            return {
                "platoon_enabled": platoon_controller.enabled,
                "platoon_is_leader": platoon_controller.is_leader,
                "platoon_position": getattr(platoon_controller, "my_position", None),
                "platoon_leader_id": platoon_controller.leader_car_id,
                "platoon_setup_complete": getattr(
                    platoon_controller, "setup_complete", False
                ),
            }
        except Exception as e:
            self.vehicle_logger.logger.error(f"Error getting platoon status: {e}")
            return {
                "platoon_enabled": False,
                "platoon_is_leader": False,
                "platoon_position": None,
                "platoon_leader_id": None,
                "platoon_setup_complete": False,
            }

    def invalidate_periodic_status_cache(self) -> None:
        """Mark low-frequency static status fields dirty for rebuild."""
        self._periodic_status_static_fields_dirty = True

    def _is_ground_station_connected(self) -> bool:
        """Return True when telemetry can be queued to the Ground Station."""
        client = getattr(self, "client_Ground_Station", None)
        if client is None:
            return False

        try:
            return bool(getattr(client, "is_connected", lambda: True)())
        except Exception:
            return False

    def _get_observer_type_status(self) -> dict:
        """Get observer selections for low-rate status broadcasts."""
        vehicle_observer = getattr(self, "vehicle_observer", None)
        if vehicle_observer is None:
            return {
                "local_observer_type": "unknown",
                "fleet_observer_type": "unknown",
            }

        return {
            "local_observer_type": getattr(
                vehicle_observer, "local_estimator_type", "unknown"
            ),
            "fleet_observer_type": getattr(
                vehicle_observer, "fleet_estimator_type", "unknown"
            ),
        }

    def _get_controller_type_status(self) -> dict:
        """Get active controller types for low-rate status broadcasts."""
        controller_manager = getattr(self, "controller_manager", None)
        if controller_manager is None:
            return {
                "path_long_ctrl": "unknown",
                "path_lat_ctrl": "unknown",
                "leader_long_ctrl": "unknown",
                "leader_lat_ctrl": "unknown",
            }

        return {
            "path_long_ctrl": controller_manager.get_longitudinal_type("path"),
            "path_lat_ctrl": controller_manager.get_lateral_type("path"),
            "leader_long_ctrl": controller_manager.get_longitudinal_type("leader"),
            "leader_lat_ctrl": controller_manager.get_lateral_type("leader"),
        }

    def _get_cached_periodic_static_status(self) -> dict:
        """Get cached low-churn status fields and rebuild only when invalidated."""
        if (
            self._periodic_status_static_fields_dirty
            or not self._periodic_status_static_fields
        ):
            static_fields = {}
            static_fields.update(self._get_observer_type_status())
            static_fields.update(self._get_controller_type_status())
            self._periodic_status_static_fields = static_fields
            self._periodic_status_static_fields_dirty = False

        return self._periodic_status_static_fields

    def _build_v2v_status_details(self, v2v_status: dict) -> dict:
        """Build the nested V2V payload expected by the Ground Station."""
        is_active = bool(v2v_status.get("v2v_active", False))
        peer_count = int(v2v_status.get("v2v_peers", 0))
        return {
            "status": "connected" if is_active else "disconnected",
            "connected_peers": peer_count,
            "fleet_size": (peer_count + 1) if is_active else 1,
        }

    def _build_periodic_status_message(self, current_time: float) -> dict:
        """Build the periodic low-rate status payload for the Ground Station."""
        v2v_status = self._get_v2v_status_cache()
        status_msg = {
            "type": "v2v_status",
            "timestamp": current_time,
            "car_id": self.vehicle_id,
            "vehicle_type": self.vehicle_type,
            "programme_type": self.programme_type,
            "perception_active": bool(
                getattr(getattr(self, "yolo_manager", None), "yolo_enabled", False)
            ),
            "node_sequence": getattr(self, "node_sequence", None),
            "operational_status": self.get_operational_status(),
            "online_sysid_status": self._get_online_sysid_status(),
            "online_calibration_status": self._get_online_calibration_status(),
            "robust_kalmannet_dataset_status": self._get_robust_kalmannet_dataset_status(),
            "data": self._build_v2v_status_details(v2v_status),
        }
        status_msg.update(v2v_status)
        status_msg.update(self._get_platoon_status())
        status_msg.update(self._get_cached_periodic_static_status())
        status_msg.update(self._get_local_sensor_attack_status_compact())

        try:
            current_handler = self.state_machine.get_current_state_handler()
            if current_handler and hasattr(current_handler, "get_path_visualization_data"):
                path_viz = current_handler.get_path_visualization_data()
                if path_viz:
                    status_msg["path_viz"] = path_viz
        except Exception:
            pass

        return status_msg

    def _broadcast_periodic_status(self):
        """Broadcast periodic status (V2V, Platoon, etc.) at low rate (1Hz)"""
        try:
            current_time = time.time()
            if current_time - self._last_status_broadcast_time < (
                1.0 / self._status_broadcast_rate
            ):
                return

            if not self._is_ground_station_connected():
                return

            status_msg = self._build_periodic_status_message(current_time)
            self.client_Ground_Station.queue_telemetry(status_msg)

            self._last_status_broadcast_time = current_time

        except Exception as e:
            self.vehicle_logger.log_error("Periodic status broadcast error", e)

    def _broadcast_v2v_state(self):
        """Broadcast vehicle state to V2V network - V2VManager handles rate-limiting internally"""
        try:
            current_time = time.time()
            if not self._should_attempt_v2v_broadcast(current_time):
                return

            if not hasattr(self, "v2v_manager") or self.v2v_manager is None:
                return

            # V2VManager.update_broadcast() handles all rate-limiting:
            # - Local state: 20Hz, Fleet state: 5Hz, Heartbeat: 1Hz
            self.v2v_manager.update_broadcast()

            # Periodic logging of V2V activity (every 5 seconds)
            if hasattr(self, "_last_v2v_log_time"):
                if time.time() - self._last_v2v_log_time > 5.0:
                    self._log_v2v_activity()
                    self._last_v2v_log_time = time.time()
            else:
                self._last_v2v_log_time = time.time()

        except Exception as e:
            self.vehicle_logger.log_error("V2V state broadcast error", e)

    def _log_v2v_activity(self):
        """Log V2V communication activity summary"""
        try:
            if hasattr(self, "v2v_manager") and self.v2v_manager.is_active():
                status = self.v2v_manager.get_connection_status()
                stats = status.get("communication_stats", {})

                self.vehicle_logger.logger.debug(
                    f"V2V Activity - Fleet: {status.get('fleet_size', 0)}, "
                    f"Rate: {stats.get('actual_rate_hz', 0.0):.1f}Hz, "
                    f"Sent: {stats.get('messages_sent', 0)}, "
                    f"Recv: {stats.get('messages_received', 0)}"
                )

        except Exception as e:
            self.vehicle_logger.logger.error(f"V2V activity logging error: {e}")

    # V2V messages are now processed automatically via V2VManager
    # No need for manual polling - this eliminates duplicate processing

    def _process_queued_commands(self):
        """Process commands from queue (non-blocking)"""
        if self.client_Ground_Station:
            try:
                current_time = time.time()
                if not self._should_poll_commands(current_time):
                    return

                # Check if client has is_connected method, if not assume connected
                is_connected = getattr(
                    self.client_Ground_Station, "is_connected", lambda: True
                )()
                if is_connected:
                    commands = self.client_Ground_Station.get_latest_commands()
                    if commands:
                        # Use centralized command handler instead of direct processing
                        success = self.command_handler.process_command(commands)
                        if not success:
                            self.vehicle_logger.log_warning("Failed to process command")
            except Exception as e:
                if self.loop_counter % 100 == 0:  # Log error only occasionally
                    self.vehicle_logger.log_error("Command processing error", e)

    def _process_commands(self, commands: dict):
        """Legacy method - now redirects to command handler"""
        self.command_handler.process_command(commands)

    def _initialize_network_2_GroundStation(self) -> bool:
        """Initialize network communication - simplified with 8s timeout handled in client"""
        try:
            self.vehicle_logger.logger.info("Creating Ground Station client...")

            # Create Ground Station client
            self.client_Ground_Station = GroundStationClient(
                config=self.config,
                logger=self.vehicle_logger,
                kill_event=self.kill_event,
            )

            # Initialize network connection (handles 8s timeout internally)
            self.client_Ground_Station.initialize_network()

            # Start network threads (only if connected)
            self.client_Ground_Station.start_threads()

            # Log final connection status
            if self.client_Ground_Station.is_connected():
                self.vehicle_logger.logger.info(
                    "Ground Station communication established"
                )
            else:
                self.vehicle_logger.logger.info(
                    "Continuing without Ground Station connection"
                )

            return True

        except Exception as e:
            self.vehicle_logger.log_error("Ground Station initialization failed", e)
            return False

    def _handle_v2v_status_change(self, event_type: str, peer_id: int):
        """
        Handle immediate V2V status changes from V2VManager
        This is now just a simple forwarder to existing logging
        """
        try:
            if event_type == "peer_connected":
                self.vehicle_logger.logger.info(
                    f"Vehicle {peer_id} connected to V2V network"
                )
            elif event_type == "peer_disconnected":
                self.vehicle_logger.logger.warning(
                    f"Vehicle {peer_id} disconnected from V2V network"
                )
            elif event_type == "v2v_activated":
                peer_data = peer_id if isinstance(peer_id, dict) else {}
                fleet_size = peer_data.get("fleet_size", "unknown")
                self.vehicle_logger.logger.info(
                    f"V2V system activated with fleet size: {fleet_size}"
                )
            elif event_type == "v2v_deactivated":
                self.vehicle_logger.logger.info(f"V2V system deactivated")
            else:
                self.vehicle_logger.logger.debug(f"V2V status change: {event_type}")

        except Exception as e:
            self.vehicle_logger.logger.error(f"V2V status change handling error: {e}")

    def report_v2v_status_to_gs(self, status_data: dict):
        """Report V2V connection status to Ground Station - public method for V2VManager"""
        try:
            if self.client_Ground_Station:
                # Create V2V status report
                v2v_report = {
                    "type": "v2v_status",
                    "car_id": self.config.network.car_id,
                    "data": status_data,
                }

                # Send immediately (high priority)
                self.client_Ground_Station.queue_telemetry(v2v_report)

                self.vehicle_logger.logger.info(
                    f"V2V status reported to GS: {status_data['status']}"
                )
            else:
                self.vehicle_logger.logger.warning(
                    "Cannot report V2V status - no Ground Station connection"
                )

        except Exception as e:
            self.vehicle_logger.logger.error(
                f"Failed to report V2V status to Ground Station: {e}"
            )

    def _shutdown(self):
        """Shutdown all systems"""
        self.vehicle_logger.logger.info("Shutting down...")

        # Use emergency_stop method instead of non-existent SHUTTING_DOWN state
        try:
            self.state_machine.emergency_stop("System shutdown requested")
        except Exception as e:
            self.vehicle_logger.log_error("Error during state machine shutdown", e)

        # Stop vehicle hardware
        if self.qcar:
            try:
                self.qcar.write(throttle=0, steering=0)
                self.vehicle_logger.logger.info(
                    "Vehicle stopped (throttle=0, steering=0)"
                )
            except Exception as e:
                self.vehicle_logger.log_error("Error stopping vehicle", e)

        # if IS_PHYSICAL_QCAR:
        #     # Stop QUARC models controlling hardware (LiDAR, GPS, etc.)
        #     self._stop_quarc_models()

        # Close network handler and stop threads
        if self.client_Ground_Station:
            self.client_Ground_Station.close()

        # Shutdown YOLO server process
        if hasattr(self, "yolo_process") and self.yolo_process:
            try:
                self.vehicle_logger.logger.info("Terminating YOLO server...")
                self.yolo_process.terminate()
                try:
                    self.yolo_process.wait(timeout=5)
                    self.vehicle_logger.logger.info("YOLO server terminated")
                except:
                    self.vehicle_logger.logger.warning("YOLO server force killed")
                    self.yolo_process.kill()
            except Exception as e:
                self.vehicle_logger.logger.error(f"YOLO server shutdown error: {e}")

        # Shutdown YOLO manager
        if hasattr(self, "yolo_manager") and self.yolo_manager.yolo is not None:
            try:
                self.yolo_manager.yolo.terminate()
            except Exception as e:
                self.vehicle_logger.logger.error(f"YOLO receiver shutdown error: {e}")

        # Shutdown V2V Manager (which handles V2V communication internally)
        if hasattr(self, "v2v_manager"):
            try:
                self.v2v_manager.disable_v2v()
            except Exception as e:
                self.vehicle_logger.logger.error(f"V2V Manager shutdown error: {e}")

        # Stop Vehicle Observer (closes data recordings)
        if hasattr(self, "vehicle_observer") and self.vehicle_observer:
            try:
                self.vehicle_observer.stop()
            except Exception as e:
                self.vehicle_logger.logger.error(f"Observer shutdown error: {e}")

        # Stop Online SysID ZMQ transport
        if hasattr(self, "online_sysid_zmq") and self.online_sysid_zmq:
            try:
                self.online_sysid_zmq.stop()
            except Exception as e:
                self.vehicle_logger.logger.error(
                    f"Online SysID ZMQ shutdown error: {e}"
                )

        # Stop Online Calibration ZMQ transport
        if hasattr(self, "online_calibration_zmq") and self.online_calibration_zmq:
            try:
                self.online_calibration_zmq.stop()
            except Exception as e:
                self.vehicle_logger.logger.error(
                    f"Online Calibration ZMQ shutdown error: {e}"
                )

        if hasattr(self, "robust_kalmannet_dataset") and self.robust_kalmannet_dataset:
            try:
                self.robust_kalmannet_dataset.stop(save=True)
            except Exception as e:
                self.vehicle_logger.logger.error(
                    f"Robust KalmanNet dataset shutdown error: {e}"
                )

        # Log final statistics
        self.vehicle_logger.logger.info("=" * 60)
        self.vehicle_logger.logger.info("Final Statistics:")
        self.vehicle_logger.logger.info(f"Total iterations: {self.loop_counter}")
        self.vehicle_logger.logger.info(f"Total time: {self.elapsed_time():.2f}s")

        # Log network statistics
        if self.client_Ground_Station:
            net_stats = self.client_Ground_Station.get_statistics()
            self.vehicle_logger.logger.info(
                f"Network - Telemetry sent: {net_stats['telemetry_sent']}, Commands received: {net_stats['commands_received']}"
            )
            if net_stats["queue_overflows"] > 0:
                self.vehicle_logger.logger.info(
                    f"Network queue overflows: {net_stats['queue_overflows']}"
                )

        perf_stats = self.perf_monitor.get_statistics()
        if "loop_time" in perf_stats:
            self.vehicle_logger.logger.info(
                f"Average loop frequency: {perf_stats['loop_time']['frequency']:.1f} Hz"
            )

        self.vehicle_logger.logger.info("=" * 60)

        # Close logger
        self.vehicle_logger.close()
