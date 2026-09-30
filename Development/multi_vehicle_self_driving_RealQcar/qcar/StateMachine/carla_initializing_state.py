"""
CARLA-specific initialization state.

This state is used by vehicle_main_carla.py. It skips QCar hardware,
QCarGPS, SDCSRoadMap, and calibration setup, then injects CARLA-backed
adapters into VehicleLogic so the normal observer, state machine, telemetry,
and controller paths can run unchanged.
"""

from __future__ import annotations

import time
import traceback
from typing import Any, Dict, Optional, Tuple

import numpy as np

from StateMachine.state_base import StateBase
from StateMachine.vehicle_state import VehicleState, StateTransitionReason

try:
    from command_types import CommandType
except Exception:
    CommandType = None


class CarlaInitializingState(StateBase):
    """Simulator initialization state for the pure-Python CARLA bridge."""

    INITIAL_DELAY = 0.2
    STEP_DELAY = 0.1
    TIMEOUT = 20.0

    def enter(self) -> bool:
        super().enter()
        self.logger.logger.info("=" * 70)
        self.logger.logger.info("Entering CARLA INITIALIZING state")
        self.logger.logger.info("(QCar hardware, GPS hardware, ROS, and SDCS map skipped)")
        self.logger.logger.info("=" * 70)

        self.state_data = {
            "initialization_start": time.time(),
            "components_initialized": False,
            "ready_to_start": False,
            "last_step_time": time.time(),
        }
        return True

    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        throttle, steering = 0.0, 0.0
        current_time = time.time()
        elapsed_time = current_time - self.state_data["initialization_start"]

        if elapsed_time > self.TIMEOUT:
            self.logger.log_warning(
                f"[CARLA INIT] Timeout after {self.TIMEOUT:.1f}s, proceeding to WAITING_FOR_START"
            )
            return (
                throttle,
                steering,
                (
                    VehicleState.WAITING_FOR_START,
                    StateTransitionReason.INITIALIZATION_COMPLETE,
                ),
            )

        if elapsed_time < self.INITIAL_DELAY:
            return throttle, steering, None

        if not self.state_data["components_initialized"]:
            if self._initialize_carla_components():
                self.state_data["components_initialized"] = True
                self.state_data["last_step_time"] = current_time
                self.logger.logger.info("[STEP 1/2] CARLA components initialized")
            else:
                self._log_initialization_progress(elapsed_time)
                return throttle, steering, None

        if (
            self.state_data["components_initialized"]
            and not self.state_data["ready_to_start"]
        ):
            if current_time - self.state_data["last_step_time"] < self.STEP_DELAY:
                return throttle, steering, None

            self.state_data["ready_to_start"] = True
            self.logger.logger.info("[STEP 2/2] CARLA system ready to start")

        if self.state_data["ready_to_start"]:
            self.logger.logger.info("=" * 70)
            self.logger.logger.info(
                f"CARLA initialization complete in {elapsed_time:.1f}s"
            )
            self.logger.logger.info("=" * 70)
            return (
                throttle,
                steering,
                (
                    VehicleState.WAITING_FOR_START,
                    StateTransitionReason.INITIALIZATION_COMPLETE,
                ),
            )

        return throttle, steering, None

    def _initialize_carla_components(self) -> bool:
        steps = [
            ("CARLA adapters", self._inject_carla_adapters, 0.02),
            ("CARLA route", self._initialize_carla_route, 0.02),
            ("Perception defaults", self._initialize_perception_defaults, 0.02),
            ("Observer", self._initialize_observer, 0.02),
            ("Telemetry logging", self._initialize_telemetry, 0.02),
        ]

        try:
            self.logger.logger.info("Starting CARLA component initialization...")
            for idx, (name, init_func, settle_time) in enumerate(steps, 1):
                self.logger.logger.info(f"[{idx}/{len(steps)}] Initializing {name}...")
                if not init_func():
                    self.logger.log_error(f"Failed to initialize {name}")
                    return False
                time.sleep(settle_time)
            return True
        except Exception as exc:
            self.logger.log_error("CARLA component initialization failed", exc)
            traceback.print_exc()
            return False

    def _get_bridge(self):
        bridge = getattr(self.vehicle_logic, "_carla_bridge", None)
        if bridge is None:
            self.logger.log_error("VehicleLogic has no _carla_bridge")
            return None
        return bridge

    def _inject_carla_adapters(self) -> bool:
        bridge = self._get_bridge()
        if bridge is None:
            return False

        if bridge.qcar_adapter is None or bridge.gps_adapter is None:
            self.logger.log_error("CARLA bridge adapters are not ready")
            return False

        self.vehicle_logic.qcar = bridge.qcar_adapter
        self.vehicle_logic.gps = bridge.gps_adapter
        self.vehicle_logic.roadmap = None
        self.vehicle_logic.node_sequence = ["carla_route"]
        self.logger.logger.info("CARLA QCar/GPS adapters injected")
        return True

    def _initialize_carla_route(self) -> bool:
        bridge = self._get_bridge()
        if bridge is None:
            return False

        waypoints = np.asarray(bridge.waypoint_sequence, dtype=float)
        if waypoints.ndim != 2 or waypoints.shape[0] < 2 or waypoints.shape[1] < 2:
            self.logger.log_error(f"Invalid CARLA waypoint sequence: {waypoints.shape}")
            return False

        self.vehicle_logic.waypoint_sequence = waypoints[:2, :]
        self.vehicle_logic.node_sequence = ["carla_route"]

        self.logger.logger.info(
            f"CARLA route loaded with {self.vehicle_logic.waypoint_sequence.shape[1]} waypoints"
        )
        return True

    def _initialize_perception_defaults(self) -> bool:
        yolo_manager = getattr(self.vehicle_logic, "yolo_manager", None)
        if yolo_manager is None:
            return True

        yolo_manager.yolo_enabled = False
        yolo_manager.yolo = None
        yolo_manager.yolo_drive = None

        try:
            from Yolo.YoLo import YOLOData

            yolo_manager._cached_data = YOLOData()
        except Exception:
            pass

        self.logger.logger.info("YOLO disabled for CARLA v1; using default YOLOData")
        return True

    def _initialize_observer(self) -> bool:
        bridge = self._get_bridge()
        if bridge is None:
            return False

        gps = bridge.gps_adapter
        if gps is None:
            self.logger.log_error("CARLA GPS adapter missing")
            return False

        gps.readGPS()
        initial_pose = np.array(
            [
                float(gps.position[0]),
                float(gps.position[1]),
                float(gps.orientation[2]),
            ],
            dtype=float,
        )

        self.logger.logger.info(
            "Initializing observer from CARLA pose: "
            f"x={initial_pose[0]:.2f}, y={initial_pose[1]:.2f}, theta={initial_pose[2]:.3f}"
        )

        success = self.vehicle_logic.vehicle_observer.initialize_local_estimator(
            gps=gps,
            initial_pose=initial_pose,
            estimator_params={
                "use_qcar_ekf": False,
                "disturbance_mode": "none",
            },
        )
        if not success:
            self.logger.log_error("CARLA observer initialization failed")
            return False

        self.vehicle_logic.vehicle_observer._initialized = True
        return True

    def _initialize_telemetry(self) -> bool:
        try:
            logging_cfg = self.config.logging
            data_logging_enabled = any(
                bool(getattr(logging_cfg, attr, False))
                for attr in (
                    "enable_telemetry_logging",
                    "enable_fleet_estimation_logging",
                    "enable_local_estimation_logging",
                    "enable_following_leader_logging",
                    "enable_trust_weight_logging",
                )
            )
            if data_logging_enabled:
                self.vehicle_logic.logger.setup_telemetry_logging(
                    logging_cfg.data_log_dir,
                    enable_telemetry=bool(
                        getattr(logging_cfg, "enable_telemetry_logging", True)
                    ),
                )
            return True
        except Exception as exc:
            self.logger.log_error("CARLA telemetry initialization failed", exc)
            return False

    def _log_initialization_progress(self, elapsed_time: float) -> None:
        if elapsed_time % 2.0 < 0.1:
            self.logger.logger.info(
                f"[CARLA INIT] Waiting for components... ({elapsed_time:.1f}s elapsed)"
            )

    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        if CommandType is not None and command_type == CommandType.EMERGENCY_STOP:
            self.logger.logger.warning("[CARLA INIT] Emergency stop during initialization")
            return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)

        self.logger.logger.info(
            f"[CARLA INIT] Ignoring command during initialization: {command_type}"
        )
        return None

    def exit(self):
        init_time = self.get_time_in_state()
        self.logger.logger.info(f"Exiting CARLA INITIALIZING state after {init_time:.1f}s")
        super().exit()
