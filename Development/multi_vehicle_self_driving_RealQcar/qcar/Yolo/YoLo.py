import numpy as np
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import cv2  # Added for video streaming
import subprocess
import os
import sys

try:
    import zmq

    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("[YOLO] WARNING: pyzmq not installed. Run: pip install pyzmq")


@dataclass
class YOLOData:
    """
    Immutable container for all YOLO detection data per frame.
    Created fresh each update cycle - no stale data issues.
    """

    # Detection arrays (raw 7-element arrays)
    stop_sign: np.ndarray = field(default_factory=lambda: np.zeros(7))
    traffic_light: np.ndarray = field(default_factory=lambda: np.zeros(7))
    cars: np.ndarray = field(default_factory=lambda: np.zeros(7))
    yield_sign: np.ndarray = field(default_factory=lambda: np.zeros(7))
    person: np.ndarray = field(default_factory=lambda: np.zeros(7))

    # Computed velocity gain from YOLODriveLogic
    yolo_gain: float = 1.0

    # Distance to detected objects
    car_dist: Optional[float] = None
    person_dist: Optional[float] = None

    # Horizontal offsets [-1.0 to 1.0] representing object position relative to camera center
    car_offset: Optional[float] = None
    person_offset: Optional[float] = None

    # Center-box obstacle data (new)
    obstacle: np.ndarray = field(default_factory=lambda: np.zeros(7))
    obstacle_in_path: bool = False  # True if any obstacle in center box
    obstacle_type: float = 0.0  # 0=none, 1=person, 2=cone/other
    obstacle_dist: Optional[float] = None  # Distance to closest center-box obstacle
    obstacle_offset: Optional[float] = None  # Offset of closest center-box obstacle

    # Lane detection data
    lane_confidence: float = 0.0
    lane_steering: float = 0.0
    lane_curvature: float = 0.0
    lane_offset: float = 0.0
    lane_left_detected: bool = False
    lane_right_detected: bool = False

    # Metadata
    is_valid: bool = False
    timestamp: float = 0.0

    def to_dict(self) -> dict:
        """Convert to dictionary for backward compatibility with sensor_data['yolo_data']."""
        return {
            "stop_sign": self.stop_sign,
            "traffic_light": self.traffic_light,
            "cars": self.cars,
            "yield_sign": self.yield_sign,
            "person": self.person,
            "yolo_gain": self.yolo_gain,
            "is_valid": self.is_valid,
            "car_dist": self.car_dist,
            "person_dist": self.person_dist,
            "car_offset": self.car_offset,
            "person_offset": self.person_offset,
            "obstacle_in_path": self.obstacle_in_path,
            "obstacle_type": self.obstacle_type,
            "obstacle_dist": self.obstacle_dist,
            "obstacle_offset": self.obstacle_offset,
            "lane_confidence": self.lane_confidence,
            "lane_steering": self.lane_steering,
            "lane_slope": self.lane_curvature,  # Alias for backward compat
            "lane_intercept": self.lane_offset,  # Alias for backward compat
            "lane_left_detected": self.lane_left_detected,
            "lane_right_detected": self.lane_right_detected,
        }


class YOLOReceiver:
    """
    ZeroMQ-based YOLO data receiver (SUB socket with CONFLATE).

    Advantages over BasicStream:
    - No timeout exceptions — zmq.NOBLOCK returns immediately if no data
    - CONFLATE mode keeps only the latest message — no stale data buildup
    - Automatic reconnection on disconnect (ZMQ handles it internally)
    - Message-framed — each send() = one atomic message, no partial reads
    - No polling loops needed — single recv() call

    Compatible API with the old Quanser BasicStream-based receiver.
    """

    def __init__(self, ip="localhost", nonBlocking=True, port="18666"):
        self.stopSign = np.zeros((7), dtype=np.float64)
        self.trafficlight = np.zeros((7), dtype=np.float64)
        self.cars = np.zeros((7), dtype=np.float64)
        self.yieldSign = np.zeros((7), dtype=np.float64)
        self.person = np.zeros((7), dtype=np.float64)
        self.lane = np.zeros((7), dtype=np.float64)
        self.obstacle = np.zeros((7), dtype=np.float64)

        self._shutting_down = False
        self.connected = False
        self._port = port
        self._ip = ip

        # Expected packet shape (7 rows: stop, traffic, car, yield, person, lane, obstacle)
        self._packet_shape = (7, 7)
        self._packet_bytes = 7 * 7 * 8  # float64 = 8 bytes

        # ZMQ context and socket
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages
        self._socket.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message
        self._socket.setsockopt(zmq.RCVHWM, 1)  # High-water mark = 1
        self._socket.setsockopt(zmq.RCVTIMEO, 0)  # Non-blocking (0 = immediate)
        self._socket.setsockopt(zmq.LINGER, 0)  # Don't hang on close
        self._socket.setsockopt(zmq.RECONNECT_IVL, 200)  # Reconnect every 200ms
        self._socket.setsockopt(
            zmq.RECONNECT_IVL_MAX, 2000
        )  # Max reconnect interval 2s

        self._uri = f"tcp://{ip}:{port}"
        self._socket.connect(self._uri)
        self.connected = True  # ZMQ connect is async — always succeeds
        print(f"[YOLO-ZMQ] Receiver connected to {self._uri} (SUB + CONFLATE)")

        # Don't block waiting for data in init - allow async startup
        # self.status_check('', iterations=30)

    def status_check(self, message, iterations=30):
        """Wait for the publisher to become available."""
        for i in range(iterations):
            try:
                data = self._socket.recv(zmq.NOBLOCK)
                if len(data) == self._packet_bytes:
                    self._unpack(data)
                    if message:
                        print(message)
                    print(f"[YOLO-ZMQ] Publisher confirmed on {self._uri}")
                    return
            except zmq.Again:
                pass
            time.sleep(0.5)
        print(
            f"[YOLO-ZMQ] Warning: No publisher response after {iterations} attempts on {self._uri}"
        )

    def _unpack(self, raw_bytes: bytes):
        """Unpack raw bytes into detection arrays."""
        packet = np.frombuffer(raw_bytes, dtype=np.float64).reshape(self._packet_shape)
        self.stopSign[:] = packet[0, :]
        self.trafficlight[:] = packet[1, :]
        self.cars[:] = packet[2, :]
        self.yieldSign[:] = packet[3, :]
        self.person[:] = packet[4, :]
        self.lane[:] = packet[5, :]
        self.obstacle[:] = packet[6, :]

    def read(self):
        """Read latest detection data. Returns True if new data received."""
        if self._shutting_down:
            return False

        try:
            raw = self._socket.recv(zmq.NOBLOCK)
            if len(raw) == self._packet_bytes:
                self._unpack(raw)
                return True
            else:
                return False
        except zmq.Again:
            # No new message available — this is normal, not an error
            return False
        except zmq.ZMQError as e:
            if not self._shutting_down:
                print(f"[YOLO-ZMQ] Receive error: {e}")
            return False

    def terminate(self):
        """Terminate the YOLO receiver."""
        self._shutting_down = True
        try:
            self._socket.close(linger=0)
        except Exception:
            pass

    def graceful_shutdown(self):
        """Signal shutdown to stop reads before actual termination."""
        self._shutting_down = True

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.terminate()


class YOLOPublisher:
    """
    ZeroMQ-based YOLO data publisher (PUB socket).

    Advantages over BasicStream:
    - Non-blocking send — never blocks, even without subscribers
    - Multiple subscribers supported (PUB/SUB pattern)
    - No connection management needed — ZMQ handles it
    - No timeout exceptions
    - Message-framed — atomic send, no partial writes

    Compatible API with the old Quanser BasicStream-based publisher.
    """

    def __init__(self, ip="localhost", nonBlocking=False, port="18666"):
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 1)  # Keep only latest outgoing message
        self._socket.setsockopt(zmq.LINGER, 0)  # Don't hang on close

        self._uri = f"tcp://*:{port}"
        self._socket.bind(self._uri)
        self.connected = True
        print(f"[YOLO-ZMQ] Publisher bound on {self._uri} (PUB)")

        # Give subscribers time to connect (ZMQ slow-joiner problem)
        time.sleep(0.3)

    def status_check(self, message, iterations=10):
        """No-op for ZMQ — PUB socket doesn't need connection checks."""
        pass

    def send(self, yolodata):
        """Send YOLO detection data as raw bytes.

        Args:
            yolodata: numpy array (6,7) float64 — detection packet

        Returns:
            bool: True if sent successfully
        """
        try:
            raw = np.ascontiguousarray(yolodata, dtype=np.float64).tobytes()
            self._socket.send(raw, zmq.NOBLOCK)
            return True
        except zmq.Again:
            # No subscriber connected — data is dropped (expected for PUB)
            return False
        except zmq.ZMQError as e:
            print(f"[YOLO-ZMQ] Send error: {e}")
            return False

    def terminate(self):
        """Close the publisher socket."""
        try:
            self._socket.close(linger=0)
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.terminate()


class YOLOManager:
    """
    YOLOManager class manages YOLO components and provides high-level interface
    for vehicle_logic.py to reduce YOLO-related code in the main controller.

    Refactored API:
    - update(loop_counter) -> YOLOData: Reads, processes, and returns fresh data each cycle
    - get_data() -> YOLOData: Returns cached YOLOData from last update
    - get_gain() -> float: Convenience getter for velocity gain
    """

    def __init__(self, logger=None):
        self.logger = logger
        self.yolo = None
        self.yolo_drive = None
        self.loop_counter = 0
        self.yolo_enabled = False

        # Cached YOLOData - created fresh each update cycle
        self._cached_data: YOLOData = YOLOData()

    def initialize(
        self, yolo_receiver: "YOLOReceiver", yolo_drive_logic: "YOLODriveLogic"
    ):
        """Initialize YOLO components"""
        if yolo_receiver is None or yolo_drive_logic is None:
            self.yolo_enabled = False  # Disable YOLO if components are missing
        else:
            self.yolo_enabled = (
                True  # Enable YOLO only when both components are provided
            )
        self.yolo = yolo_receiver
        self.yolo_drive = yolo_drive_logic

    def update(self, loop_counter: int = 0) -> YOLOData:
        """
        Update YOLO detection data - reads, processes, and returns fresh YOLOData.
        This is the primary method to call each control loop iteration.

        Returns:
            YOLOData: Fresh instance containing all detection data for this cycle
        """
        self.loop_counter = loop_counter

        # Create new YOLOData instance each cycle (user requested fresh instances)
        data = YOLOData(timestamp=time.time())

        try:
            if self.yolo is None:
                if self.loop_counter % 1000 == 0 and self.logger:
                    self.logger.log_error("YOLO receiver is None")
                self._cached_data = data
                return data

            # Read from YOLO receiver
            new_data = self.yolo.read()

            # # Debug logging
            # if self.loop_counter % 100 == 0 and self.logger:
            #     connection_status = "Connected" if self.yolo._handle.connected else "NOT Connected"
            #     self.logger.logger.info(f"[YOLO] Receiver status: {connection_status}, New data: {new_data}")

            # Copy detection arrays (create new arrays to avoid stale references)
            data.stop_sign = self.yolo.stopSign.copy()
            data.traffic_light = self.yolo.trafficlight.copy()
            data.cars = self.yolo.cars.copy()
            data.yield_sign = self.yolo.yieldSign.copy()
            data.person = self.yolo.person.copy()
            data.obstacle = self.yolo.obstacle.copy()
            data.is_valid = new_data

            # Extract obstacle data from receiver
            obs = self.yolo.obstacle
            if obs[0] > 0:
                data.obstacle_in_path = True
                data.obstacle_type = float(obs[2])
                data.obstacle_dist = float(obs[1]) if obs[1] > 0 else None
                data.obstacle_offset = float(obs[3])

            # Extract lane data from receiver
            lane = self.yolo.lane
            if len(lane) >= 6:
                data.lane_confidence = float(lane[0])
                data.lane_steering = float(lane[1])
                data.lane_curvature = float(lane[2])
                data.lane_offset = float(lane[3])
                data.lane_left_detected = lane[4] > 0.5
                data.lane_right_detected = lane[5] > 0.5

            # Process through YOLODriveLogic to get velocity gain
            if self.yolo_drive is not None:
                try:
                    data.yolo_gain = self.yolo_drive.check_yolo(
                        self.yolo.stopSign,
                        self.yolo.trafficlight,
                        self.yolo.cars,
                        self.yolo.yieldSign,
                        self.yolo.person,
                        self.yolo.obstacle,
                    )
                    # Get computed distances and offsets from drive logic
                    data.car_dist = getattr(self.yolo_drive, "carDist", None)
                    data.person_dist = getattr(self.yolo_drive, "personDist", None)
                    data.car_offset = getattr(self.yolo_drive, "carOffset", None)
                    data.person_offset = getattr(self.yolo_drive, "personOffset", None)
                except Exception as e:
                    if self.loop_counter % 100 == 0 and self.logger:
                        self.logger.log_error("YOLO drive error", e)
                    data.yolo_gain = 1.0

        except Exception as e:
            if self.logger:
                self.logger.log_error("YOLO update error", e)
            data.yolo_gain = 1.0

        self._cached_data = data
        return data

    def get_data(self) -> YOLOData:
        """Get cached YOLOData from last update() call."""
        return self._cached_data

    def get_yolo_data(self) -> dict:
        """
        Get current YOLO detection data as dict.
        DEPRECATED: Use update() or get_data().to_dict() instead.
        Kept for backward compatibility with existing code.
        """
        return self._cached_data.to_dict()

    def get_relative_car_measurement(
        self, source: str = "yolo_car_dist"
    ) -> Optional[Dict[str, Any]]:
        """
        Return a validated host-to-car relative measurement for observers.

        The measurement is extracted from cached YOLO outputs and normalized into
        a single schema so callers do not reimplement validity checks.
        """
        data = self._cached_data

        if not data.is_valid:
            return None

        car_detected = False
        cars_raw = data.cars
        if isinstance(cars_raw, np.ndarray) and cars_raw.size > 0:
            car_detected = float(cars_raw[0]) > 0.0
        elif isinstance(cars_raw, (list, tuple)) and len(cars_raw) > 0:
            car_detected = float(cars_raw[0]) > 0.0

        if not car_detected:
            return None

        if data.car_dist is None:
            return None

        rel_distance = float(data.car_dist)
        if not np.isfinite(rel_distance) or rel_distance <= 0.0:
            return None

        car_count = 1.0
        if isinstance(cars_raw, np.ndarray) and cars_raw.size > 0:
            car_count = float(cars_raw[0])
        elif isinstance(cars_raw, (list, tuple)) and len(cars_raw) > 0:
            car_count = float(cars_raw[0])
        if not np.isfinite(car_count):
            car_count = 1.0

        car_offset = data.car_offset
        if car_offset is None and isinstance(cars_raw, np.ndarray) and cars_raw.size > 2:
            car_offset = float(cars_raw[2])
        elif car_offset is None and isinstance(cars_raw, (list, tuple)) and len(cars_raw) > 2:
            car_offset = float(cars_raw[2])

        offset_abs = abs(float(car_offset)) if car_offset is not None and np.isfinite(car_offset) else 0.5

        count_factor = float(np.clip(car_count / 2.0, 0.0, 1.0))
        distance_factor = float(np.exp(-max(rel_distance - 6.0, 0.0) / 4.0))
        offset_factor = float(np.clip(1.0 - 0.6 * offset_abs, 0.0, 1.0))
        confidence = float(
            np.clip(
                0.5 * count_factor + 0.3 * distance_factor + 0.2 * offset_factor,
                0.0,
                1.0,
            )
        )

        return {
            "distance": rel_distance,
            "relative_velocity": float("nan"),
            "confidence": confidence,
            "source": str(source),
            "timestamp_ns": int(float(data.timestamp) * 1e9),
        }

    def get_default_yolo_data(self) -> dict:
        """Get default YOLO data when YOLO is not available"""
        return YOLOData().to_dict()

    def get_gain(self) -> float:
        """Get velocity gain from last update."""
        return self._cached_data.yolo_gain

    def get_yolo_gain(self) -> float:
        """Get current YOLO velocity gain (alias for backward compat)."""
        return self._cached_data.yolo_gain

    def is_yolo_active(self) -> bool:
        """Check if YOLO components are active"""
        return self.yolo is not None and self.yolo_drive is not None

    # Legacy method - redirects to update()
    def update_yolo_data(self, loop_counter: int = 0) -> float:
        """
        DEPRECATED: Use update() instead.
        Kept for backward compatibility - returns only yolo_gain.
        """
        data = self.update(loop_counter)
        return data.yolo_gain

    def disable(self):
        """Disable YOLO system and clean up resources"""
        if self.logger:
            self.logger.logger.info("[YOLO] Disabling YOLO system...")

        # Signal graceful shutdown first to prevent timeout spam during disable
        if self.yolo is not None:
            try:
                self.yolo.graceful_shutdown()
            except Exception:
                pass  # Ignore errors during shutdown signal

        # Terminate YOLO receiver if it exists
        if self.yolo is not None:
            try:
                self.yolo.terminate()
                if self.logger:
                    self.logger.logger.info("[YOLO] YOLO receiver terminated")
            except Exception as e:
                if self.logger:
                    self.logger.logger.warning(
                        f"[YOLO] Error terminating receiver: {e}"
                    )

        # Clear references
        self.yolo = None
        self.yolo_drive = None
        self.yolo_enabled = False
        self._cached_data = YOLOData()  # Reset to default

        if self.logger:
            self.logger.logger.info("[YOLO] YOLO system disabled")


class YOLODriveLogic:
    """
    YOLODriveLogic class implements the logic for processing YOLO predictions
    and determining the vehicle's velocity gain based on detected objects.

    Arguments:
        stopSignThreshold (float): Distance threshold for stop sign detection.
        trafficThreshold (float): Distance threshold for traffic light detection.
        carThreshold (float): Distance threshold for car detection.
        yieldThreshold (float): Distance threshold for yield sign detection.
        personThreshold (float): Distance threshold for person detection.
        pulseLength (int): Lengh of the pulse generated after detecting an object in number of frames.

    Methods:
        check_yolo(stopSign, trafficLight, QCar, yieldSign, person):
            Processes YOLO predictions and returns the velocity gain.
        stopSignPulse(stopSign):
            Handles stop sign detection logic.
        trafficPulse(trafficLight):
            Handles traffic light detection logic.
        yieldPulse(yieldSign):
            Handles yield sign detection logic.
        carPulse(car):
            Handles car detection logic.
        personPulse(person):
            Handles person detection logic.
    """

    def __init__(
        self,
        stopSignThreshold=0.3,
        trafficThreshold=1.0,
        carThreshold=0.3,
        yieldThreshold=0.3,
        personThreshold=0.6,
        pulseLength=300,
        min_confirm_frames=5,
    ):
        self.counter = 0
        self.counter_yield = 0
        self.counter_traffic = 0
        self.counterStart_traffic = False
        self.counterStart = False
        self.counterStart_yield = False
        self.stopSignTrigger = 0
        self.carTrigger = 0
        self.trafficTrigger = 0
        self.yieldTrigger = 0
        self.personTrigger = 0
        self.vGain_person = 1
        self.vGain_yield = 1
        self.vGain_stop = 1
        self.vGain_car = 1
        self.vGain = 1
        self.stopSignThreshold = stopSignThreshold
        self.trafficThreshold = trafficThreshold
        self.carThreshold = carThreshold
        self.yieldThreshold = yieldThreshold
        self.personThreshold = personThreshold
        self.pulseLength = pulseLength

        self.carDist = 100
        self.stopSignDist = 100
        self.trafficLightDist = 100
        self.yieldDist = 100
        self.personDist = 100
        self.obstacleDist = 100
        
        # Offsets
        self.carOffset = 0.0
        self.personOffset = 0.0
        self.obstacleOffset = 0.0

        self.obstacle_in_path = False
        self.obstacle_type = 0.0  # 0=none, 1=person, 2=cone, 3=car-as-obstacle
        self.vGain_obstacle = 1

        # Overtake mode: when True, cars are treated as obstacles for replanning
        # instead of causing a velocity gain reduction (trailing/stop).
        # Set to False in FOLLOWING_LEADER to always trail the leader.
        self.car_overtake_mode = False

        # Temporal confirmation: require N consecutive frames before triggering
        self.min_confirm_frames = min_confirm_frames
        self._stop_confirm_count = 0
        self._yield_confirm_count = 0
        self._traffic_confirm_count = 0
        # Traffic lights are more stable, require fewer frames
        self._traffic_min_confirm = max(3, min_confirm_frames // 2)

    def check_yolo(
        self, stopSign, trafficLight, QCar, yieldSign, person, obstacle=None
    ):
        """processes the YOLO predictions and returns the velocity gain."""

        self.stopSignPulse(stopSign)
        self.trafficPulse(trafficLight)

        if self.stopSignTrigger == 1 or self.trafficTrigger == 1:
            self.vGain_stop = 0
            # return self.vGain

        self.carPulse(QCar)
        self.personPulse(person)
        # if self.carTrigger ==1 or self.personTrigger == 1:
        #     return self.vGain

        self.yieldPulse(yieldSign)
        if self.yieldTrigger == 1:
            self.vGain_yield = 0.9  # Very slight slowdown for yield
            # return self.vGain

        # Center-box obstacle: person → full stop, cone → gain unaffected (replanned)
        if obstacle is not None:
            self.obstaclePulse(obstacle)

        self.vGain = min(
            [
                self.vGain_yield,
                self.vGain_stop,
                self.vGain_car,
                self.vGain_person,
                self.vGain_obstacle,
            ]
        )
        self.vGain_yield = 1
        self.vGain_stop = 1
        self.vGain_car = 1
        self.vGain_person = 1
        self.vGain_obstacle = 1
        return self.vGain

    def stopSignPulse(self, stopSign):
        """If a stop sign is closer than the threshold AND has been detected for
        min_confirm_frames consecutive frames, a pulse is generated reducing the
        velocity gain to 0 for the duration. After the pulse, detection is paused
        for half the pulse time."""

        stopSignCount = stopSign[0]
        if stopSignCount > 0:
            self.stopSignDist = stopSign[1]
        else:
            self.stopSignDist = 100

        if not self.counterStart:
            # Check distance threshold
            if stopSignCount > 0 and self.stopSignDist < self.stopSignThreshold:
                # Temporal confirmation: increment counter
                self._stop_confirm_count += 1
                if self._stop_confirm_count >= self.min_confirm_frames:
                    # Confirmed! Start the stop pulse
                    self.counterStart = True
                    self.stopSignTrigger = 1
                    self._stop_confirm_count = 0
                else:
                    self.stopSignTrigger = 0
            else:
                # Not detected within threshold — reset confirmation
                self._stop_confirm_count = 0
                self.counterStart = False
                self.stopSignTrigger = 0
        else:
            self.counter += 1
            if self.counter < self.pulseLength:
                self.stopSignTrigger = 1
            elif self.counter < self.pulseLength + int(self.pulseLength / 2):
                self.stopSignTrigger = 0
            else:
                self.counter = 0
                self.counterStart = False
                self.stopSignTrigger = 0

    def trafficPulse(self, trafficLight):
        """If a red traffic light is within the stopping distance window [0.6m, 1.0m],
        trigger a stop after temporal confirmation. Vehicle should stop before
        crossing the roadmap."""

        trafficLightCount = trafficLight[0]

        # Traffic light distance window for stopping
        traffic_min_dist = 0.6  # Don't stop if already too close (past it)
        traffic_max_dist = self.trafficThreshold  # Default 1.0m

        if trafficLightCount > 0:
            self.trafficLightDist = trafficLight[1]
        else:
            self.trafficLightDist = 100
            self.trafficTrigger = 0

        if not self.counterStart_traffic:
            if (
                trafficLightCount > 0
                and self.trafficLightDist <= traffic_max_dist
                and self.trafficLightDist >= traffic_min_dist
            ):
                # Temporal confirmation for traffic lights
                self._traffic_confirm_count += 1
                if self._traffic_confirm_count >= self._traffic_min_confirm:
                    self.trafficTrigger = 1
                    self.counterStart_traffic = True
                    self._traffic_confirm_count = 0
                else:
                    self.trafficTrigger = 0
            else:
                self._traffic_confirm_count = 0
                self.counterStart_traffic = False
                self.trafficTrigger = 0
        else:
            self.counter_traffic += 1
            if self.counter_traffic < int(self.pulseLength / 6):
                self.trafficTrigger = 1
            else:
                self.counter_traffic = 0
                self.counterStart_traffic = False

    def yieldPulse(self, yieldSign):
        """If a yield sign is closer than the threshold AND confirmed for
        min_confirm_frames, reduce speed very slightly (gain = 0.9).
        Lane-side filtering already done server-side."""

        yieldSignCount = yieldSign[0]
        if yieldSignCount > 0:
            self.yieldDist = yieldSign[1]
        else:
            self.yieldDist = 100
            self.yieldTrigger = 0

        if not self.counterStart_yield:
            if yieldSignCount > 0 and self.yieldDist < self.yieldThreshold:
                # Temporal confirmation
                self._yield_confirm_count += 1
                if self._yield_confirm_count >= self.min_confirm_frames:
                    self.counterStart_yield = True
                    self.yieldTrigger = 1
                    self._yield_confirm_count = 0
                else:
                    self.yieldTrigger = 0
            else:
                self._yield_confirm_count = 0
                self.counterStart_yield = False
                self.yieldTrigger = 0
        else:
            self.counter_yield += 1
            if self.counter_yield < self.pulseLength:
                self.yieldTrigger = 1
            else:
                self.counter_yield = 0
                self.counterStart_yield = False

    def carPulse(self, car):
        """If a car is closer than the threshold, the velocity gain will be
        reduced to a value between 0 and 1, depending on the distance to the car.
        The speed will start decresing at the distance of 1.2, and will be 0
        at the distance of self.carThreshold.

        In OVERTAKE mode (car_overtake_mode=True), gain reduction is skipped
        so the local planner can replan around the car instead of stopping.
        Distance tracking (carDist) is always updated for obstacle extraction."""

        carCount = car[0]
        detect_threshold = 1.2
        if carCount > 0:
            self.carDist = car[1]
            self.carOffset = car[2]
        else:
            self.carDist = 100
            self.carOffset = 0.0
            self.carTrigger = 0
        if carCount > 0 and self.carDist < detect_threshold:
            self.carTrigger = 1
            if self.car_overtake_mode:
                # Overtake: skip gain reduction, mark car as obstacle for replanning
                self.obstacle_in_path = True
                self.obstacle_type = 3.0  # car-as-obstacle
                self.obstacleDist = self.carDist
                # vGain_car stays at 1.0 — local planner handles avoidance
            else:
                # Trailing: reduce gain to slow down / stop behind the car
                m = 1 / (detect_threshold - self.carThreshold)
                b = -m * self.carThreshold
                self.vGain_car = np.clip(m * self.carDist + b, 0, 1)
        else:
            self.carTrigger = 0

    def personPulse(self, person):
        """If a person is closer than the threshold, the velocity gain will be
        reduced to a value between 0 and 1, depending on the distance to the car.
        The speed will start decresing at the distance of 1.5, and will be 0
        at the distance of self.personThreshold."""

        personCount = person[0]
        detect_threshold = 1.5
        if personCount > 0:
            self.personDist = person[1]
            self.personOffset = person[2]
        else:
            self.personDist = 100
            self.personOffset = 0.0
            self.personTrigger = 0
        if personCount > 0 and self.personDist < detect_threshold:
            self.personTrigger = 1
            m = 1 / (detect_threshold - self.personThreshold)
            b = -m * self.personThreshold
            self.vGain_person = np.clip(m * self.personDist + b, 0, 1)
        else:
            self.personTrigger = 0

    def obstaclePulse(self, obstacle):
        """Handle center-box obstacle detection.

        obstacle[0] = count of obstacles in center box
        obstacle[1] = distance to closest obstacle
        obstacle[2] = type: 1.0=person, 2.0=cone/other

        Person in center box → full stop (gain = 0).
        Cone in center box   → gain unchanged (path replanning handles avoidance).
        """
        obs_count = obstacle[0]
        if obs_count <= 0:
            self.obstacle_in_path = False
            self.obstacle_type = 0.0
            self.obstacleDist = 100
            self.obstacleOffset = 0.0
            return

        self.obstacle_in_path = True
        self.obstacleDist = float(obstacle[1]) if obstacle[1] > 0 else 100
        self.obstacle_type = float(obstacle[2])
        self.obstacleOffset = float(obstacle[3])

        # Person blocking the path → stop completely
        if self.obstacle_type == 1.0:  # OBSTACLE_PERSON
            obstacle_stop_dist = 1.5  # Start braking within 1.5m
            if self.obstacleDist < obstacle_stop_dist:
                m = 1 / (obstacle_stop_dist - 0.2)
                b = -m * 0.2
                self.vGain_obstacle = np.clip(m * self.obstacleDist + b, 0, 1)
        # Cone → gain stays at 1.0 (replanning handles avoidance)


class YOLOVideoPublisher:
    """
    ZeroMQ-based Video Publisher (PUB socket).
    Sends JPEG-encoded frames over the network.

    NOTE: The publisher always binds to ALL interfaces (tcp://*:PORT) regardless
    of the `ip` argument, so that remote subscribers (e.g. Ground Station PC)
    can connect.  The `ip` parameter is kept for API compatibility but ignored
    for binding purposes.
    """

    def __init__(self, ip="*", port="18766"):
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 2)  # Keep only latest 2 frames
        self._socket.setsockopt(zmq.LINGER, 0)  # Don't hang on close

        # Always bind to ALL interfaces so remote GS can subscribe.
        # (binding to 'localhost' would only be reachable from the same machine)
        self._uri = f"tcp://*:{port}"
        self._socket.bind(self._uri)
        print(
            f"[YOLO-Video] Publisher bound on {self._uri} (all interfaces, port {port})"
        )

    def send(self, frame):
        """
        Compress and send video frame.
        Args:
            frame: encoding-ready image (numpy array, usually BGR)
        """
        try:
            # Compress to JPEG with 80% quality to save bandwidth
            _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            self._socket.send(buffer.tobytes(), zmq.NOBLOCK)
        except zmq.Again:
            pass  # Drop frame if network is congested
        except Exception as e:
            print(f"[YOLO-Video] Send error: {e}")

    def terminate(self):
        try:
            self._socket.close(linger=0)
        except:
            pass


class YOLOVideoReceiver:
    """
    ZeroMQ-based Video Receiver (SUB socket).
    Receives and decodes JPEG frames.
    """

    def __init__(self, ip="localhost", port="18766"):
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.setsockopt(zmq.CONFLATE, 1)  # Always get latest frame
        self._socket.setsockopt(zmq.RCVTIMEO, 0)  # Non-blocking
        self._socket.setsockopt(zmq.LINGER, 0)

        self._uri = f"tcp://{ip}:{port}"
        self._socket.connect(self._uri)
        print(f"[YOLO-Video] Receiver connected to {self._uri}")

    def read(self):
        """
        Read latest frame.
        Returns:
            frame: cv2 image (BGR) or None if no new frame
        """
        try:
            data = self._socket.recv(zmq.NOBLOCK)
            # Decode JPEG buffer
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            return frame
        except zmq.Again:
            return None
        except Exception as e:
            print(f"[YOLO-Video] Read error: {e}")
            return None

    def terminate(self):
        try:
            self._socket.close(linger=0)
        except:
            pass


class YOLOLauncher:
    """
    Helper class to launch YOLO server subprocess and connect receiver.
    Refactored from state_base.py.
    """

    @staticmethod
    def launch_server(
        is_physical: bool, vehicle_id: int, probing: bool, logger=None
    ) -> Optional[subprocess.Popen]:
        """
        Launch the YOLO server subprocess based on vehicle type.

        Args:
            is_physical: True if running on physical QCar
            vehicle_id: ID of the vehicle
            probing: Whether probing is enabled
            logger: Optional logger for status messages (can be None)

        Returns:
            subprocess.Popen object if successful, None otherwise
        """
        try:
            # Get script paths - assume we are in qcar/Yolo/ directory
            current_dir = os.path.dirname(os.path.abspath(__file__))

            if not is_physical:
                # Virtual Vehicle
                yolo_script = os.path.join(current_dir, "yolo_server_virtual.py")
                yolo_port = f"1866{vehicle_id}"

                if not os.path.exists(yolo_script):
                    # Fallback check if running from different context
                    if logger:
                        logger.log_error(
                            f"[PERCEPTION] YOLO script not found: {yolo_script}"
                        )
                    return None

                if logger:
                    logger.logger.info(
                        f"[PERCEPTION] [->] Starting yolo_server_virtual.py..."
                    )
                    logger.logger.info(
                        f"[PERCEPTION] Launching on port {yolo_port} (Probing: {probing})"
                    )

                # Build command
                probing_str = "True" if probing else "False"
                cmd = [
                    "python",
                    yolo_script,
                    "-idx",
                    str(vehicle_id),
                    "-p",
                    probing_str,
                ]
                if probing:
                    cmd.append("-s")  # Show image in cv2 window as backup for virtual

                if logger:
                    logger.logger.info(f"[PERCEPTION] Command: {' '.join(cmd)}")

                # Launch process
                yolo_process = None
                if sys.platform == "win32":
                    yolo_process = subprocess.Popen(
                        cmd, creationflags=subprocess.CREATE_NEW_CONSOLE
                    )
                else:
                    yolo_process = subprocess.Popen(
                        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
                    )

                # Wait for startup
                # Wait briefly to catch immediate crashes (e.g. import errors)
                time.sleep(0.5)
                if yolo_process.poll() is not None:
                    if logger:
                        logger.log_error(
                            f"[PERCEPTION] YOLO server process terminated immediately. Code: {yolo_process.returncode}"
                        )
                    return None

                if logger:
                    logger.logger.info(
                        f"[PERCEPTION] [OK] YOLO server process started (PID: {yolo_process.pid})"
                    )
                return yolo_process

            else:
                # Physical Vehicle
                yolo_script = os.path.join(current_dir, "yolo_server.py")
                probing_flag = "True" if probing else "False"

                if logger:
                    logger.logger.info(f"[PERCEPTION] [->] Starting yolo_server.py...")
                    logger.logger.info(
                        f"[PERCEPTION] Probing: {probing}, Car ID: {vehicle_id}"
                    )

                # Build command
                # video_port = 18760 + vehicle_id  must match multi_probing.py: port = 18760 + car_id
                video_port = 18760 + vehicle_id
                yolo_cmd = [
                    "python",
                    yolo_script,
                    "-p",
                    probing_flag,
                    "-idx",
                    str(vehicle_id),
                    "--video-port",
                    str(video_port),
                ]

                # Log redirection
                # Assuming standard structure: .../qcar/Yolo/ -> .../qcar/logs/
                log_dir = os.path.join(os.path.dirname(current_dir), "logs")
                os.makedirs(log_dir, exist_ok=True)
                log_file = os.path.join(log_dir, f"yolo_{vehicle_id}.log")

                if logger:
                    logger.logger.info(f"[PERCEPTION] Redirecting output to {log_file}")

                # We need to open the file handler and keep it open for the subprocess
                # But subprocess takes file descriptor. We can let the caller handle it?
                # No, we must open it here. Popen will keep it open.
                f_log = open(log_file, "w")
                yolo_process = subprocess.Popen(
                    yolo_cmd, stdout=f_log, stderr=subprocess.STDOUT
                )

                if logger:
                    logger.logger.info(
                        f"[PERCEPTION] [OK] Physical YOLO server started (PID: {yolo_process.pid})"
                    )
                return yolo_process

        except Exception as e:
            if logger:
                logger.log_error("[PERCEPTION] Error launching YOLO server", e)
            return None

    @staticmethod
    def connect_receiver(
        port: str, max_retries: int = 5, retry_delay: float = 3.0, logger=None
    ) -> Optional[YOLOReceiver]:
        """
        Connect a YOLOReceiver to the given port.
        Since ZMQ is async and we removed blocking checks, this returns immediately.
        """
        try:
            if logger:
                logger.logger.info(
                    f"[PERCEPTION] Connecting YOLOReceiver on port {port}..."
                )

            # Use non-blocking mode by default
            receiver = YOLOReceiver(ip="localhost", nonBlocking=True, port=str(port))

            if logger:
                logger.logger.info(
                    f"[PERCEPTION] [OK] YOLOReceiver connected on port {port}"
                )
            return receiver

        except Exception as e:
            if logger:
                logger.log_error(f"[PERCEPTION] Receiver creation failed", e)
            return None
