import logging, time, threading
import numpy as np
from abc import ABC, abstractmethod

from core.Types import SensorData, ControlCommand
from core.Helpers import copy_safe

class BaseVehicleIO(ABC):
    """
    Vehicle I/O with internal buffering.
    'read()' always returns the latest cached data;
    the IO layer decides when to poll hardware based on configured rates.
    'write()' clips commands to safe ranges and delegates to vehicle-specific write.
    """

    def __init__(self, config: dict, logger=None):
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._max_throttle = config["write"]["max_throttle"]
        self._max_steering = config["write"]["max_steering"]
        self._reading_sensor_rate_hz = config["read"]["sensor_rate_hz"]
        self._reading_gps_rate_hz = config["read"]["gps_rate_hz"]
        # Single combined buffer — sensor + GPS share one SensorData
        self._sensor_data_cache = SensorData(
            motor_tach=0.0, gyro_z=0.0, accelerometer=np.zeros(3), sensor_timestamp=0.0,
            gps_valid=False, gps_position=np.zeros(3), gps_timestamp=0.0,
        )
        self._command_cache = ControlCommand(throttle=0.0, steering=0.0, target_velocity=0.0)
        self._cache_lock = threading.Lock()

    # ------------------------------------------------------------------
    # poll sensors and gps and maybe others io device from hardware api.
    # ------------------------------------------------------------------

    @abstractmethod
    def _poll_sensors(self):
        """Write sensor fields DIRECTLY to self._sensor_data_cache.
        Called under _cache_lock — no need to return anything."""
        self._sensor_data_cache.motor_tach = 0.0
        self._sensor_data_cache.gyro_z = 0.0
        self._sensor_data_cache.accelerometer = np.zeros(3)
        self._sensor_data_cache.sensor_timestamp = time.time()

    def read_sensors(self) -> None:
        """Read sensors into cache when the configured rate allows."""
        now = time.time()
        with self._cache_lock:
            if now - self._sensor_data_cache.sensor_timestamp >= 1.0 / self._reading_sensor_rate_hz:
                self._poll_sensors()  # writes directly into cache

    @abstractmethod
    def _poll_gps(self):
        """Write GPS fields DIRECTLY to self._sensor_data_cache.
        Called under _cache_lock."""
        self._sensor_data_cache.gps_valid = False
        self._sensor_data_cache.gps_position = np.zeros(3)
        self._sensor_data_cache.gps_timestamp = time.time()

    def read_gps(self) -> None:
        """Read GPS into cache when the configured rate allows."""
        now = time.time()
        with self._cache_lock:
            if now - self._sensor_data_cache.gps_timestamp >= 1.0 / self._reading_gps_rate_hz:
                self._poll_gps()  # writes directly into cache
    
    # ------------------------------------------------------------------
    # read io part
    # ------------------------------------------------------------------

    def read_to_cache(self) -> None:
        """Convenience method to read both sensors and GPS into cache."""
        self.read_sensors()
        self.read_gps()

    def read(self) -> SensorData:
        """Return a copy of the latest cached sensor data."""
        with self._cache_lock:
            return SensorData(
                motor_tach=self._sensor_data_cache.motor_tach,
                gyro_z=self._sensor_data_cache.gyro_z,
                accelerometer=self._sensor_data_cache.accelerometer.copy(),
                sensor_timestamp=self._sensor_data_cache.sensor_timestamp,
                gps_valid=self._sensor_data_cache.gps_valid,
                gps_position=self._sensor_data_cache.gps_position.copy(),
                gps_timestamp=self._sensor_data_cache.gps_timestamp,
            )
    
    # ------------------------------------------------------------------
    # write control input to vehicle.
    # ------------------------------------------------------------------

    def write(self, command: ControlCommand):
        """Clip then delegate to hardware-specific write."""
        t = self._clip(command.throttle, -self._max_throttle, self._max_throttle)
        s = self._clip(command.steering, -self._max_steering, self._max_steering)
        self._command_cache = ControlCommand(
        throttle=t, steering=s,
        target_velocity=command.target_velocity,
        source=command.source,
        )
        self._hardware_write(t, s)

    def get_last_command(self) -> ControlCommand:
        """Return a copy of the last command sent to the vehicle."""
        with self._cache_lock:
            return ControlCommand(
                throttle=self._command_cache.throttle,
                steering=self._command_cache.steering,
                target_velocity=self._command_cache.target_velocity,
                source="vehicleIO_write_cache",
            )

    @abstractmethod
    def _hardware_write(self, throttle: float, steering: float):
        """Vehicle-specific: QCar uses qcar.write(), Limo publishes to ROS."""
        ...

    @abstractmethod
    def stop(self):
        """Zero command + cleanup."""
        self._hardware_write(0.0, 0.0)

    @staticmethod
    def _default_sensor_data() -> dict:
        return {
            "motor_tach": 0.0,
            "gyro_z": 0.0,
            "accelerometer": np.zeros(3),
            "sensor_timestamp": 0.0,
        }

    @staticmethod
    def _default_gps_data() -> dict:
        return {
            "gps_valid": False,
            "gps_position": np.zeros(3),
            "gps_timestamp": 0.0,
        }

    @staticmethod
    def _clip(value, lo, hi):
        return max(lo, min(hi, value))


class NullVehicleIO(BaseVehicleIO):
    """No-hardware stub. Never polls; always returns safe defaults."""

    def __init__(self, config: dict, logger=None):
        # Bypass rate logic — never poll hardware
        super().__init__(config, logger)
        self._max_throttle = 1.0
        self._max_steering = 1.0

    def _poll_sensors(self):
        """Write safe sensor defaults directly to the shared cache."""
        self._sensor_data_cache.motor_tach = 0.0
        self._sensor_data_cache.gyro_z = 0.0
        self._sensor_data_cache.accelerometer = np.zeros(3)
        self._sensor_data_cache.sensor_timestamp = time.time()

    def _poll_gps(self):
        """Write safe GPS defaults directly to the shared cache."""
        self._sensor_data_cache.gps_valid = False
        self._sensor_data_cache.gps_position = np.zeros(3)
        self._sensor_data_cache.gps_timestamp = time.time()

    def _hardware_write(self, throttle, steering):
        pass

    def stop(self):
        pass
