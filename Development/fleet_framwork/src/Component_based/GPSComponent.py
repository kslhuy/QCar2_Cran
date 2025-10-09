import time
import math
import logging
import numpy as np
from concurrent.futures import ThreadPoolExecutor, TimeoutError
from typing import Optional, Dict, Any


class GPSComponent:
    """
    GPS component for vehicle state estimation and synchronization.
    Handles both virtual QCar (QLabs) and physical QCar GPS data.
    """

    def __init__(self, vehicle_id: int, qcar, qlabs, gps_sync, logger: logging.Logger,
                 use_physical_qcar: bool = False, physical_qcar=None, gps=None,
                 enable_steering_control: bool = True):
        """
        Initialize GPS component.

        Args:
            vehicle_id: Unique vehicle identifier
            qcar: QLabsQCar2 instance
            qlabs: QuanserInteractiveLabs instance
            gps_sync: GPSSync instance for time synchronization
            logger: Logger instance
            use_physical_qcar: Whether to use physical QCar
            physical_qcar: Physical QCar instance (if applicable)
            gps: GPS instance (if applicable)
            enable_steering_control: Whether steering control is enabled
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.qlabs = qlabs
        self.gps_sync = gps_sync
        self.logger = logger

        self.use_physical_qcar = use_physical_qcar
        self.physical_qcar = physical_qcar
        self.gps = gps
        self.enable_steering_control = enable_steering_control

        # GPS data cache
        self.gps_data_cache = {
            'position': None,
            'rotation': None,
            'velocity': 0.0,
            'available': False,
            'timestamp': 0.0,
            'last_update': 0.0
        }

        # Position tracking
        self.current_pos = [0, 0, 0]
        self.current_rot = [0, 0, 0]
        self.velocity = 0.0
        self.prev_pos = None
        self.prev_time = None
        self._prev_time_monotonic = None

        # World transform timeout handling
        self.world_tf_timeout = 0.05  # seconds
        self.world_tf_timeouts = 0
        self.world_tf_errors = 0
        self.world_tf_last_warning = 0.0
        self._last_valid_location = None
        self._last_valid_rotation = None
        self._timeout_warning_issued = False

        # Executor for timeout protection
        self._world_tf_executor = None

    def update_gps_data(self) -> bool:
        """
        Update GPS data from QCar sensors.

        Returns:
            True if update was successful, False otherwise
        """
        try:
            gps_start_time = time.perf_counter()
            current_time = time.time()

            if self.use_physical_qcar and self.physical_qcar is not None:
                success = self._update_physical_qcar_data(current_time)
            else:
                success = self._update_virtual_qcar_data(current_time)

            gps_duration = (time.perf_counter() - gps_start_time) * 1000
            if gps_duration > 2.0:  # Log if GPS update takes >2ms
                self.logger.warning(f"Vehicle {self.vehicle_id}: GPS update took {gps_duration:.4f}ms")

            return success

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: GPS update error: {e}")
            self.gps_data_cache['available'] = False
            return False

    def _update_virtual_qcar_data(self, current_time: float) -> bool:
        """Update using virtual QCar (QLabs) API."""
        # Get current transform from QCar with timeout protection
        result = self._get_world_transform_with_timeout()
        if result is None:
            # Fallback to last known values if available
            if self._last_valid_location is not None:
                location = self._last_valid_location
                rotation = self._last_valid_rotation
            else:
                self.gps_data_cache['available'] = False
                return False
        else:
            try:
                _, location, rotation, _ = result
            except Exception:
                self.logger.error(f"Vehicle {self.vehicle_id}: Unexpected get_world_transform() return format: {result}")
                self.gps_data_cache['available'] = False
                return False
            # Cache successful result
            self._last_valid_location = location
            self._last_valid_rotation = rotation
            self._timeout_warning_issued = False

        if location is not None and rotation is not None:
            # Calculate velocity
            now_mono = time.monotonic()
            if self._prev_time_monotonic is not None and self.prev_pos is not None:
                dt = now_mono - self._prev_time_monotonic
                if dt > 0:
                    dx = location[0] - self.prev_pos[0]
                    dy = location[1] - self.prev_pos[1]
                    self.velocity = math.sqrt(dx*dx + dy*dy) / dt

            # Update current state
            self.current_pos = list(location)
            self.current_rot = list(rotation)
            self.prev_pos = list(location)
            self.prev_time = current_time
            self._prev_time_monotonic = now_mono

            # Update GPS cache
            self.gps_data_cache.update({
                'position': list(location),
                'rotation': list(rotation),
                'velocity': self.velocity,
                'available': True,
                'timestamp': current_time,
                'last_update': current_time
            })
            return True
        else:
            self.gps_data_cache['available'] = False
            return False

    def _get_world_transform_with_timeout(self, timeout: float = 0.1) -> Optional[tuple]:
        """
        Retrieve world transform with a timeout to prevent stalls.

        Args:
            timeout: Maximum seconds to wait for the transform.

        Returns:
            Tuple (id, location, rotation, extra) on success, or None on timeout/error.
        """
        eff_timeout = timeout if timeout is not None else self.world_tf_timeout
        try:
            # Lazy-create executor
            if self._world_tf_executor is None:
                self._world_tf_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"qcar_tf_{self.vehicle_id}")

            future = self._world_tf_executor.submit(self.qcar.get_world_transform)
            return future.result(timeout=eff_timeout)
        except TimeoutError:
            self.world_tf_timeouts += 1
            # Rate-limit warnings (no more than once per 2s)
            now = time.time()
            if now - self.world_tf_last_warning > 2.0:
                self.logger.warning(f"Vehicle {self.vehicle_id}: get_world_transform() timeout after {eff_timeout*1000:.1f}ms (total timeouts={self.world_tf_timeouts})")
                self.world_tf_last_warning = now
            try:
                future.cancel()
            except Exception:
                pass
            return None
        except Exception as e:
            self.world_tf_errors += 1
            self.logger.error(f"Vehicle {self.vehicle_id}: get_world_transform() error ({self.world_tf_errors}): {e}")
            return None

    def _update_physical_qcar_data(self, current_time: float) -> bool:
        """Update using physical QCar API with EKF."""
        if self.physical_qcar is None:
            return False

        try:
            # Read from physical QCar sensors
            self.physical_qcar.read()

            if self.enable_steering_control and self.gps is not None:
                # GPS and EKF update
                if self.gps.readGPS():
                    # GPS data available
                    y_gps = np.array([
                        self.gps.position[0],
                        self.gps.position[1],
                        self.gps.orientation[2]
                    ])
                    self.current_pos = [y_gps[0], y_gps[1], 0.0]
                    self.current_rot = [0.0, 0.0, y_gps[2]]
                    available = True
                else:
                    available = False
            else:
                available = False

            # Update GPS cache
            self.velocity = self.physical_qcar.motorTach
            self.gps_data_cache.update({
                'position': self.current_pos.copy(),
                'rotation': self.current_rot.copy(),
                'velocity': self.velocity,
                'available': available,
                'timestamp': current_time,
                'last_update': current_time
            })
            return available

        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Physical GPS update error: {e}")
            return False

    def get_cached_gps_data(self) -> Dict[str, Any]:
        """Get cached GPS data."""
        return self.gps_data_cache.copy()

    def get_current_state(self) -> Dict[str, Any]:
        """Get current vehicle state from GPS."""
        return {
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity
        }

    def cleanup(self):
        """Clean up GPS component resources."""
        try:
            self._shutdown_world_tf_executor()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: GPS cleanup error: {e}")

    def _shutdown_world_tf_executor(self):
        """Shutdown the world transform executor to avoid thread leakage."""
        try:
            if self._world_tf_executor is not None:
                self._world_tf_executor.shutdown(wait=False, cancel_futures=True)
                self._world_tf_executor = None
        except Exception:
            pass