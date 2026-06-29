import time
import numpy as np

from .BaseIO import BaseVehicleIO


class QCar2IO(BaseVehicleIO):
    def __init__(self, config, vehicle_id=0, logger=None):

        super().__init__(config, logger)
        from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR

        self._is_physical = IS_PHYSICAL_QCAR
        self._QCarGPS = QCarGPS
        self._gps = None

        if IS_PHYSICAL_QCAR:
            self._qcar = QCar(readMode=1, frequency=config["timing"]["loop_rate_hz"],)
            self._gps_kwargs = {"initialPose": [0, 0, 0], "calibrate": False}
        else:
            from qvl.multi_agent import readRobots
            robots = readRobots()
            car_cfg = robots[f"QC2_{vehicle_id}"]
            self._qcar = QCar(readMode=1, frequency=100, hilPort=car_cfg["hilPort"],)
            self._gps_kwargs = {
                "initialPose": [0, 0, 0],
                "calibrate": False,
                "gpsPort": car_cfg["gpsPort"],
                "lidarIdealPort": car_cfg["lidarIdealPort"],
            }

    def _ensure_gps(self):
        if self._gps is None:
            self._gps = self._QCarGPS(**self._gps_kwargs)
        return self._gps

    def _poll_sensors(self):
        """Read QCar sensors, return the standard sensor_data dict."""
        now = time.time()
        try:
            self._qcar.read()
            self._sensor_data_cache.motor_tach = float(getattr(self._qcar, "motorTach", 0.0))
            self._sensor_data_cache.gyro_z = float(self._qcar.gyroscope[2])
            self._sensor_data_cache.accelerometer = np.asarray(self._qcar.accelerometer, dtype=float).reshape(-1)[:3].copy()
            self._sensor_data_cache.sensor_timestamp = now
            return
        except Exception:
            self._logger.warning("Failed to read QCar sensors.")
    
    def _poll_gps(self):
        """Read QCar GPS and return a dict with the standard keys."""
        now = time.time()
        gps_valid = False
        try:
            gps = self._ensure_gps()
            deadline = time.monotonic() + min(0.001, 1.0 / self._reading_gps_rate_hz)
            while time.monotonic() <= deadline:
                if gps.readGPS():
                    gps_valid = True
                    break
                time.sleep(0.001)
        except Exception as e:
            self._logger.warning(f"Failed to read QCar GPS: {e}")
            
        if gps_valid:
            # New GPS data available and updated
            self._sensor_data_cache.gps_valid = True
            self._sensor_data_cache.gps_position = np.array([
                gps.position[0],
                gps.position[1],
                gps.orientation[2],
            ]).copy()
            self._sensor_data_cache.gps_timestamp = now
            return
        elif now - self._sensor_data_cache.gps_timestamp <= 1/self._reading_gps_rate_hz:
            # In time period, return last valid GPS data.
            return
        # Otherwise, return default GPS data.
        self._sensor_data_cache.gps_valid = False

    def _hardware_write(self, throttle: float, steering: float):
        self._qcar.write(throttle=throttle, steering=steering)

    def stop(self):
        self._qcar.write(throttle=0.0, steering=0.0)

    def close(self):
        """Release local client streams without stopping the QLabs RT model."""
        self.stop()
        gps = getattr(self, "_gps", None)
        for stream_name in ("_gps_client", "_lidar_client"):
            stream = getattr(gps, stream_name, None)
            terminate = getattr(stream, "terminate", None)
            if callable(terminate):
                try:
                    terminate()
                except Exception as e:
                    self._logger.debug(f"Failed to terminate GPS stream {stream_name}: {e}")
        lidar = getattr(gps, "lidar", None)
        terminate = getattr(lidar, "terminate", None)
        if callable(terminate):
            try:
                terminate()
            except Exception as e:
                self._logger.debug(f"Failed to terminate QCar lidar client: {e}")
