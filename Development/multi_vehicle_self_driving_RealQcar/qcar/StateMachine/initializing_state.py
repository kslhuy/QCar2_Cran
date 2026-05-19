"""
Initializing State - Simplified Event-Driven Implementation

Handles system initialization and setup.
Transitions to WAITING_FOR_START when initialization is complete.
"""

import time
import sys
import os
import numpy as np
from typing import Dict, Any, Tuple, Optional

from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason
from Yolo.YoLo import YOLOReceiver, YOLODriveLogic
from pal.products.qcar import QCar, QCarGPS
from hal.products.mats import SDCSRoadMap
from ground_station_client import GroundStationClient


# Add parent directory to sys.path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

try:
    from command_handler import CommandType

    COMMAND_TYPE_AVAILABLE = True
except ImportError as e:
    print(f"ERROR: Cannot import CommandType: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None


class InitializingState(StateBase):
    """Handler for INITIALIZING state with simplified event handling"""

    # Timing constants
    INITIAL_DELAY = 1.0  # Wait before starting initialization
    STEP_DELAY = 0.5  # Delay between initialization steps
    TIMEOUT = 30.0  # Maximum initialization time
    GPS_TIMEOUT = 10.0
    GPS_POLL_INTERVAL = 0.1

    def enter(self) -> bool:
        """Initialize system components"""
        super().enter()
        self.logger.logger.info("Entering INITIALIZING state")

        self.state_data = {
            "initialization_start": time.time(),
            "components_initialized": False,
            "ready_to_start": False,
            "last_step_time": time.time(),
            "current_step_index": 0,
            "current_step_started_at": None,
            "current_step_name": None,
            "step_ready_at": 0.0,
            "init_steps": [
                {
                    "name": "Path planning",
                    "func": self._initialize_path_planning,
                    "settle_time": 0.0,
                },
                {
                    "name": "QCar hardware",
                    "func": self._initialize_qcar_step,
                    "settle_time": 0.0,
                },
                {
                    "name": "Telemetry logging",
                    "func": self._initialize_telemetry,
                    "settle_time": 0.0,
                },
            ],
            "qcar_init_phase": "create_interfaces",
            "qcar_phase_ready_at": 0.0,
            "gps_timeout_at": 0.0,
            "next_gps_poll_at": 0.0,
        }

        return True

    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle initialization process"""
        throttle, steering = 0.0, 0.0
        current_time = time.time()
        elapsed_time = current_time - self.state_data["initialization_start"]

        # Check for timeout
        if elapsed_time > self.TIMEOUT:
            self.logger.log_warning(
                f"[!] Initialization timeout ({self.TIMEOUT}s), proceeding anyway"
            )
            return (
                throttle,
                steering,
                (
                    VehicleState.WAITING_FOR_START,
                    StateTransitionReason.INITIALIZATION_COMPLETE,
                ),
            )

        # Wait for initial system settle time
        if elapsed_time < self.INITIAL_DELAY:
            return throttle, steering, None

        # Initialize components
        if not self.state_data["components_initialized"]:
            init_status = self._advance_initialization(current_time)
            if init_status is False:
                self._log_initialization_progress(elapsed_time)
                return throttle, steering, None
            if not self.state_data["components_initialized"]:
                self._log_initialization_progress(elapsed_time)
                return throttle, steering, None

        # Finalize and transition
        if (
            self.state_data["components_initialized"]
            and not self.state_data["ready_to_start"]
        ):
            if current_time - self.state_data["last_step_time"] < self.STEP_DELAY:
                return throttle, steering, None

            self.state_data["ready_to_start"] = True
            self.logger.logger.info("[STEP 2/2] System ready to start")

        # Transition to WAITING_FOR_START
        if self.state_data["ready_to_start"]:
            self.logger.logger.info(
                f"[SUCCESS] Initialization complete in {elapsed_time:.1f}s"
            )
            return (
                throttle,
                steering,
                (
                    VehicleState.WAITING_FOR_START,
                    StateTransitionReason.INITIALIZATION_COMPLETE,
                ),
            )

        return throttle, steering, None

    def _log_initialization_progress(self, elapsed_time: float):
        """Log initialization progress every 5 seconds"""
        if elapsed_time % 5.0 < 0.1:
            self.logger.logger.info(
                f"[INIT] Component initialization in progress... ({elapsed_time:.1f}s elapsed)"
            )

    def _advance_initialization(self, current_time: float) -> Optional[bool]:
        """Advance one initialization phase without blocking the main loop."""
        steps = self.state_data["init_steps"]
        step_index = self.state_data["current_step_index"]

        if step_index >= len(steps):
            return True

        if step_index == 0 and self.state_data["current_step_started_at"] is None:
            self.logger.logger.info("Starting component initialization...")

        if current_time < self.state_data["step_ready_at"]:
            return None

        step = steps[step_index]
        if self.state_data["current_step_name"] != step["name"]:
            self.state_data["current_step_name"] = step["name"]
            self.state_data["current_step_started_at"] = current_time

        try:
            step_status = step["func"]()
        except Exception as e:
            self.logger.log_error(f"{step['name']} initialization failed", e)
            return False

        if step_status is None:
            return None

        if step_status is False:
            self.logger.log_error(f"{step['name']} initialization failed")
            return False

        completed_at = time.time()
        step_duration = completed_at - self.state_data["current_step_started_at"]
        self.logger.logger.info(
            f"  [{step_index + 1}/{len(steps)}] {step['name']} ready "
            f"({step_duration*1000:.1f}ms)"
        )

        self.state_data["current_step_index"] += 1
        self.state_data["current_step_name"] = None
        self.state_data["current_step_started_at"] = None
        self.state_data["step_ready_at"] = completed_at + float(step["settle_time"])

        if self.state_data["current_step_index"] >= len(steps):
            self.state_data["components_initialized"] = True
            self.state_data["last_step_time"] = completed_at
            self.logger.logger.info(
                f"All {len(steps)} components initialized successfully!"
            )
            self.logger.logger.info("[STEP 1/2] All components initialized")

        return True

    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """Handle commands during initialization - only emergency stop is accepted"""
        if not COMMAND_TYPE_AVAILABLE:
            return super().handle_event(command_type, data)

        if command_type == CommandType.EMERGENCY_STOP:
            self.logger.logger.warning("[!] Emergency stop during initialization")
            return (VehicleState.STOPPED, StateTransitionReason.EMERGENCY_STOP)

        self.logger.logger.info(
            f"Ignoring '{command_type}' command during initialization"
        )
        return None

    def _initialize_telemetry(self) -> bool:
        """Initialize telemetry logging if enabled"""
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
        except Exception as e:
            self.logger.log_error("Telemetry initialization failed", e)
            return False

    def check_initial_position(self) -> bool:
        """Check if vehicle is at start position"""
        try:
            if not (
                hasattr(self, "roadmap")
                and self.roadmap
                and hasattr(self, "node_sequence")
            ):
                return True

            start_node_reached, init_waypoint_seq = self.roadmap.initial_check(
                self.init_pose, self.node_sequence, self.waypoint_sequence
            )

            if not start_node_reached:
                self._log_position_mismatch(init_waypoint_seq)
                return False

            self.vehicle_logger.logger.info("Vehicle is at start position")
            return True

        except Exception as e:
            self.vehicle_logger.log_error("Initial position check failed", e)
            return False

    def _log_position_mismatch(self, init_waypoint_seq):
        """Log detailed information when vehicle is not at start position"""
        target_node = self.node_sequence[0]
        target_pose = self.roadmap.get_node_pose(target_node).squeeze()
        current_dist = np.linalg.norm(self.init_pose[:2] - target_pose[:2])

        self.vehicle_logger.log_warning("=" * 60)
        self.vehicle_logger.log_warning("NOT AT START POSITION")
        self.vehicle_logger.log_warning(
            f"  Current: ({self.init_pose[0]:.2f}, {self.init_pose[1]:.2f}, {self.init_pose[2]:.2f})"
        )
        self.vehicle_logger.log_warning(f"  Target node: {target_node}")
        self.vehicle_logger.log_warning(
            f"  Target: ({target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f})"
        )
        self.vehicle_logger.log_warning(f"  Distance: {current_dist:.2f}m")
        self.vehicle_logger.log_warning("=" * 60)

        # Update waypoint sequence to navigate to start
        if not self.vehicle_logic.is_physical_qcar and init_waypoint_seq is not None:
            init_waypoint_seq = init_waypoint_seq * 0.975

        self.waypoint_sequence = init_waypoint_seq
        if hasattr(self, "steering_controller") and self.steering_controller:
            self.steering_controller.reset(self.waypoint_sequence)

    def exit(self):
        """Clean up initialization state"""
        init_time = self.get_time_in_state()
        self.logger.logger.info(f"[INIT] Exiting - completed in {init_time:.1f}s")
        super().exit()

    # ========== COMPONENT INITIALIZATION METHODS ==========

    def _initialize_path_planning(self) -> bool:
        """Initialize path planning system"""
        if not self.vehicle_logic.controller_manager.config.enable_steering_control:
            return True

        try:
            # Create roadmap
            self.vehicle_logic.roadmap = SDCSRoadMap(
                leftHandTraffic=self.config.path.left_hand_traffic, useSmallMap=False
            )

            # Set node sequence
            self.vehicle_logic.node_sequence = self.config.path.valid_nodes
            self.logger.logger.info(
                f"Node sequence ({len(self.vehicle_logic.node_sequence)} nodes): "
                f"{self.vehicle_logic.node_sequence}"
            )

            # Generate and validate waypoint sequence
            waypoints = self.vehicle_logic.roadmap.generate_path(
                self.vehicle_logic.node_sequence
            )
            if not self._validate_waypoint_sequence(waypoints):
                return False

            if not self.vehicle_logic.is_physical_qcar:
                waypoints = waypoints * 0.975
                self.logger.logger.info("Scaled virtual waypoints by 0.975")
            if self.vehicle_logic.vehicle_type == "Limo":
                waypoints = waypoints * 1.06
                self.logger.logger.info("Scaled Limo waypoints by 1.06")

            self.vehicle_logic.waypoint_sequence = waypoints
            self.logger.logger.info(
                f"Generated path with {waypoints.shape[1]} waypoints"
            )
            return True

        except Exception as e:
            self.logger.log_error("Path planning initialization failed", e)
            return False

    def _validate_waypoint_sequence(self, waypoints) -> bool:
        """Validate generated waypoint sequence"""
        if waypoints is None:
            self.logger.log_error(
                f"Failed to generate path for nodes: {self.vehicle_logic.node_sequence}"
            )
            return False

        if not isinstance(waypoints, np.ndarray):
            self.logger.log_error(f"Invalid waypoint type: {type(waypoints)}")
            return False

        if waypoints.shape[0] < 2 or waypoints.shape[1] < 2:
            self.logger.log_error(f"Invalid waypoint shape: {waypoints.shape}")
            return False

        return True

    def _initialize_network_2_GroundStation(self) -> bool:
        """Initialize network communication with Ground Station"""
        try:
            # Create client
            self.vehicle_logic.client_Ground_Station = GroundStationClient(
                config=self.config,
                logger=self.vehicle_logic.logger,
                kill_event=self.vehicle_logic.kill_event,
            )
            time.sleep(0.2)

            # Initialize network
            if not self.vehicle_logic.client_Ground_Station.initialize_network():
                return False
            time.sleep(0.5)

            # Start threads
            if not self.vehicle_logic.client_Ground_Station.start_threads():
                return False
            time.sleep(0.3)

            self.logger.logger.info("Ground Station communication initialized")
            return True

        except Exception as e:
            self.logger.log_error("Ground Station initialization failed", e)
            return False

    def _initialize_qcar_step(self) -> Optional[bool]:
        """
        Initialize QCar hardware, GPS, and local estimator across multiple loop ticks.

        Returns:
            True when the full step is complete,
            False on failure,
            None while the step is still in progress.
        """
        try:
            phase = self.state_data["qcar_init_phase"]
            current_time = time.time()

            if phase == "create_interfaces":
                if self.vehicle_logic.programme_type == "Ros":
                    self.logger.logger.info(
                        "ROS mode detected - skipping direct QCar initialization"
                    )
                    self.state_data["qcar_init_phase"] = "wait_for_gps"
                    self._start_waiting_for_gps(current_time)
                    return None

                if (
                    not self.vehicle_logic.is_physical_qcar
                    and self.vehicle_logic.vehicle_type == "Qcar"
                ):
                    self.logger.logger.info("QCar Simulation mode detected")
                    from qvl.multi_agent import readRobots

                    self._initialize_simulated_qcar(readRobots)
                    self.state_data["qcar_init_phase"] = "wait_for_gps"
                    self._start_waiting_for_gps(current_time)
                    return None

                self._create_physical_qcar()
                self.state_data["qcar_init_phase"] = "wait_after_qcar"
                self.state_data["qcar_phase_ready_at"] = current_time + 0.3
                return None

            if phase == "wait_after_qcar":
                if current_time < self.state_data["qcar_phase_ready_at"]:
                    return None
                self._create_physical_gps()
                self.state_data["qcar_init_phase"] = "wait_after_gps"
                self.state_data["qcar_phase_ready_at"] = current_time + 0.3
                return None

            if phase == "wait_after_gps":
                if current_time < self.state_data["qcar_phase_ready_at"]:
                    return None
                self.state_data["qcar_init_phase"] = "wait_for_gps"
                self._start_waiting_for_gps(current_time)
                return None

            if phase == "wait_for_gps":
                gps_status = self._poll_gps_once(current_time)
                if gps_status is None:
                    return None
                if gps_status is False:
                    return False
                self.state_data["qcar_init_phase"] = "init_estimator"

            if self.state_data["qcar_init_phase"] == "init_estimator":
                if not self._initialize_state_estimator():
                    return False
                self.logger.logger.info("QCar hardware initialized with state estimation")
                self.state_data["qcar_init_phase"] = "complete"
                return True

            return self.state_data["qcar_init_phase"] == "complete"

        except Exception as e:
            self.logger.log_error("QCar initialization failed", e)
            return False

    def _initialize_simulated_qcar(self, readRobots):
        """Initialize simulated QCar"""

        robotsDir = readRobots()
        name = f"QC2_{self.vehicle_logic.vehicle_id}"
        car_config = robotsDir[name]

        self.vehicle_logic.qcar = QCar(
            readMode=1, frequency=100, hilPort=car_config["hilPort"]
        )

        # Check if calibration was requested
        calibrate_gps = getattr(self.vehicle_logic, "calibration_requested", False)
        if calibrate_gps:
            self.logger.logger.info("📍 GPS calibration requested - calibrating GPS")
            self.vehicle_logic.calibration_requested = False  # Reset flag

        self.vehicle_logic.gps = QCarGPS(
            initialPose=self.config.path.calibration_pose,
            calibrate=calibrate_gps,
            gpsPort=car_config["gpsPort"],
            lidarIdealPort=car_config["lidarIdealPort"],
        )

    def _create_physical_qcar(self):
        """Create the physical QCar interface."""
        self.vehicle_logic.qcar = QCar(
            readMode=1, frequency=self.config.timing.controller_update_rate
        )

    def _create_physical_gps(self):
        """Create the physical GPS interface."""
        calibrate_gps = getattr(
            self.vehicle_logic, "calibration_requested", self.config.path.calibrate
        )
        if getattr(self.vehicle_logic, "calibration_requested", False):
            self.logger.logger.info("GPS calibration requested - calibrating GPS")
            self.vehicle_logic.calibration_requested = False
        self.logger.logger.info(
            f"Calibration pose: {self.config.path.calibration_pose}"
        )

        self.vehicle_logic.gps = QCarGPS(
            initialPose=self.config.path.calibration_pose, calibrate=calibrate_gps
        )

    def _poll_gps_once(self, current_time: float) -> Optional[bool]:
        """Try one GPS read without blocking the control loop."""
        if self.vehicle_logic.gps is None:
            self.logger.log_error("GPS interface is not available")
            return False

        if current_time >= self.state_data["gps_timeout_at"]:
            self.logger.log_error("GPS timeout - no reading received")
            return False

        if current_time < self.state_data["next_gps_poll_at"]:
            return None

        self.state_data["next_gps_poll_at"] = current_time + self.GPS_POLL_INTERVAL
        if not self.vehicle_logic.gps.readGPS():
            return None

        self.init_pose = np.array(
            [
                self.vehicle_logic.gps.position[0],
                self.vehicle_logic.gps.position[1],
                self.vehicle_logic.gps.orientation[2],
            ]
        )
        self.logger.logger.info(
            f"Initial pose: x={self.init_pose[0]:.2f}, "
            f"y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f}"
        )
        return True

    def _start_waiting_for_gps(self, current_time: float):
        """Initialize the non-blocking GPS wait phase."""
        self.logger.logger.info("Waiting for initial GPS reading...")
        self.state_data["gps_timeout_at"] = current_time + self.GPS_TIMEOUT
        self.state_data["next_gps_poll_at"] = current_time

    def _initialize_state_estimator(self) -> bool:
        """Initialize local state estimator through VehicleObserver"""
        try:
            # Use refactored VehicleObserver API
            # Estimator type is already set in VehicleObserver constructor
            # We just need to initialize it with GPS and initial pose
            success = self.vehicle_logic.vehicle_observer.initialize_local_estimator(
                gps=self.vehicle_logic.gps,
                initial_pose=self.init_pose,
                estimator_params={
                    "use_qcar_ekf": self.vehicle_logic.controller_manager.config.enable_steering_control
                },
            )

            if success:
                self.logger.logger.info(
                    f"Local estimator initialized at pose: "
                    f"x={self.init_pose[0]:.2f}, y={self.init_pose[1]:.2f}, theta={self.init_pose[2]:.2f}"
                )
            else:
                self.logger.log_error("Local estimator initialization failed")

            return success

        except Exception as e:
            self.logger.log_error("State estimator initialization failed", e)
            return False
