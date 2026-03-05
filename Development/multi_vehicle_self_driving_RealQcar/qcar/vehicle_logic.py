"""
Main Vehicle Controller - Integrates all components
"""

import numpy as np
import time
import os

# import random
# from typing import Optional
from threading import Event

from pal.products.qcar import IS_PHYSICAL_QCAR
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
        self.is_physical_qcar = IS_PHYSICAL_QCAR

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
        self.v2v_manager = V2VManager(
            vehicle_id=config.network.car_id,
            vehicle_logger=self.vehicle_logger,
            config=v2v_config,
            vehicle_observer=None,  # Will be set later when vehicle_observer is created
            base_port=8000,
            status_callback=self._handle_v2v_status_change,
            vehicle_logic=self,  # Pass reference to self for Ground Station reporting
        )

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
        self.controller_manager = ControllerManager(logger=self.vehicle_logger)

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

        # V2V status cache
        self._v2v_status_cache = {}
        self._v2v_status_cache_time = 0.0

        # Periodic status broadcast tracking
        self._last_status_broadcast_time = 0.0
        self._status_broadcast_rate = 1.0  # 1 Hz

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

    def elapsed_time(self) -> float:
        """Get elapsed time since start"""
        return time.time() - self.start_time

    @property
    def logger(self):
        """Backward compatibility property for accessing the vehicle logger"""
        return self.vehicle_logger

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
                self._send_telemetry_to_ground_station()  # 10Hz internal rate-limiting
                self._broadcast_periodic_status()  # 1Hz periodic status (V2V, Platoon, System)
                self._process_queued_commands()  # No rate limit - process as fast as possible
                self._broadcast_v2v_state()  # V2VManager handles internal rate-limiting

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

            # Update observer (internally handles local and fleet timing)
            state_info = self.vehicle_observer.update_observer(
                dt, last_steering, last_u
            )

            # Feed Online SysID with latest [v_x, v_y, omega, delta] sample.
            sample = self.vehicle_observer.get_online_sysid_sample()
            if sample is not None:
                if hasattr(self, "online_sysid_zmq") and self.online_sysid_zmq:
                    if self.online_sysid_zmq.is_collecting():
                        self.online_sysid_zmq.submit_sample(sample)

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
            sensor_data = self.vehicle_observer.get_estimated_state_for_control()
            # Prepare YOLO data
            sensor_data["yolo_data"] = self.yolo_manager.get_yolo_data()

            # Update state machine - it handles all state logic and transitions
            u, delta = self.state_machine.update(dt, sensor_data)

            if u is None or delta is None:
                # self.vehicle_logger.log_error("State machine returned invalid control commands")
                return True  # Skip sending commands

            # u = 0.075 # Test value
            # delta = 0.0 # Test value

            # print(f"Throttle: {u}, Steering: {delta}")
            # Send commands to vehicle hardware

            self._last_steering = delta

            if self.vehicle_type == "Limo":
                # Driver expects Ackermann steering angle [rad] on angular.z.
                # Keep compatibility with potential normalized outputs.
                delta_input = float(delta)
                if abs(delta_input) <= 0.55:
                    delta = delta_input
                else:
                    delta = max(-1.0, min(1.0, delta_input)) * 0.48869
                delta = float(np.clip(delta, -0.48869, 0.48869))
            else:
                delta = max(-1.0, min(1.0, delta))

            if self.qcar is not None:
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
        # Get current state from VehicleObserver
        state_info = self.vehicle_observer.get_estimated_state_for_control()

        # Get controller data safely (controllers may be in state machine, not vehicle_logic)
        # Check if controllers exist (backward compatibility with FollowingPathState setting them)
        # steering_controller = getattr(self, 'steering_controller', None)
        # waypoint_index = steering_controller.get_waypoint_index() if steering_controller else 0
        # errors = steering_controller.get_errors() if steering_controller else (0.0, 0.0)

        return {
            "timestamp": time.time(),
            "time": self.elapsed_time(),
            "x": float(state_info["x"]),
            "y": float(state_info["y"]),
            "th": float(state_info["theta"]),
            "v": float(state_info["velocity"]),
            "u": float(getattr(self, "_last_u", 0.0)),
            "delta": float(getattr(self, "_last_steering", 0.0)),
            # 'v_ref': float(self.v_ref * self.yolo_manager.get_yolo_gain()),
            # 'yolo_gain': float(self.yolo_manager.get_yolo_gain()),
            # 'waypoint_index': waypoint_index,
            # 'cross_track_error': float(errors[0]),
            # 'heading_error': float(errors[1]),
            "state": self.state_machine.state.name
            if hasattr(self.state_machine, "state") and self.state_machine.state
            else "UNKNOWN",
            "gps_valid": self.vehicle_observer.is_gps_valid()
            if hasattr(self, "vehicle_observer") and self.vehicle_observer
            else False,
            # Observer/Controller types moved to _broadcast_periodic_status (1Hz) to reduce bandwidth
        }

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
            if hasattr(self, "platoon_controller") and self.platoon_controller:
                return {
                    "platoon_enabled": self.platoon_controller.enabled,
                    "platoon_is_leader": self.platoon_controller.is_leader,
                    "platoon_position": getattr(
                        self.platoon_controller, "my_position", None
                    ),
                    "platoon_leader_id": self.platoon_controller.leader_car_id,
                    "platoon_setup_complete": getattr(
                        self.platoon_controller, "setup_complete", False
                    ),
                }
            else:
                return {
                    "platoon_enabled": False,
                    "platoon_is_leader": False,
                    "platoon_position": None,
                    "platoon_leader_id": None,
                    "platoon_setup_complete": False,
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

    def _broadcast_periodic_status(self):
        """Broadcast periodic status (V2V, Platoon, etc.) at low rate (1Hz)"""
        try:
            current_time = time.time()
            if current_time - self._last_status_broadcast_time < (
                1.0 / self._status_broadcast_rate
            ):
                return

            # Get V2V status
            v2v_status = self._get_v2v_status_cache()

            # Get Platoon status
            platoon_status = self._get_platoon_status()

            # # Additional system status if needed
            # system_status = {
            #     'cpu_usage': 0.0, # Placeholder
            #     'memory_usage': 0.0 # Placeholder
            # }

            # Construct periodic status message
            # Note: We send 'v2v_status' type to trigger the specific handler in GS,
            # but we also include top-level fields merged into main state by GS remote_controller

            # Determine V2V details for the handler
            v2v_details = {
                "status": "connected"
                if v2v_status.get("v2v_active")
                else "disconnected",
                "connected_peers": v2v_status.get("v2v_peers", 0),
                "fleet_size": v2v_status.get("v2v_peers", 0) + 1
                if v2v_status.get("v2v_active")
                else 1,
            }

            status_msg = {
                "type": "v2v_status",  # Triggers V2V handler in GS
                "timestamp": current_time,
                "car_id": self.vehicle_id,
                # Top-level fields (will be merged into car state by GS)
                **v2v_status,
                **platoon_status,
                # Observer and Controller types (low-frequency update - only changes on user request)
                "local_observer_type": getattr(
                    self.vehicle_observer, "local_estimator_type", "unknown"
                )
                if hasattr(self, "vehicle_observer")
                else "unknown",
                "fleet_observer_type": getattr(
                    self.vehicle_observer, "fleet_estimator_type", "unknown"
                )
                if hasattr(self, "vehicle_observer")
                else "unknown",
                # Use controller_manager for actual active controller types per state
                "path_long_ctrl": self.controller_manager.get_longitudinal_type("path")
                if hasattr(self, "controller_manager")
                else "unknown",
                "path_lat_ctrl": self.controller_manager.get_lateral_type("path")
                if hasattr(self, "controller_manager")
                else "unknown",
                "leader_long_ctrl": self.controller_manager.get_longitudinal_type(
                    "leader"
                )
                if hasattr(self, "controller_manager")
                else "unknown",
                "leader_lat_ctrl": self.controller_manager.get_lateral_type("leader")
                if hasattr(self, "controller_manager")
                else "unknown",
                # Perception status
                "perception_active": self.yolo_manager.yolo_enabled
                if hasattr(self, "yolo_manager")
                else False,
                # Path information
                "node_sequence": getattr(self, "node_sequence", None),
                "operational_status": self.get_operational_status(),
                "online_sysid_status": self._get_online_sysid_status(),
                # Payload for the handler
                "data": v2v_details,
            }

            # Append path visualization data from current state handler
            try:
                current_handler = self.state_machine.get_current_state_handler()
                if current_handler and hasattr(current_handler, "get_path_visualization_data"):
                    path_viz = current_handler.get_path_visualization_data()
                    if path_viz:
                        status_msg["path_viz"] = path_viz
            except Exception:
                pass  # Non-critical, don't let viz data break status broadcast

            if self.client_Ground_Station:
                is_connected = getattr(
                    self.client_Ground_Station, "is_connected", lambda: True
                )()
                if is_connected:
                    self.client_Ground_Station.queue_telemetry(status_msg)

            self._last_status_broadcast_time = current_time

        except Exception as e:
            self.vehicle_logger.log_error("Periodic status broadcast error", e)

    def _broadcast_v2v_state(self):
        """Broadcast vehicle state to V2V network - V2VManager handles rate-limiting internally"""
        try:
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
