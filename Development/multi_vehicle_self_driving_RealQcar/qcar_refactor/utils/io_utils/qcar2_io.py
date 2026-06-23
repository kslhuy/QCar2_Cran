from core.vehicle_io import BaseVehicleIO

class QCar2IO(BaseVehicleIO):
    def __init__(self, config):
        super().__init__(config)
        from pal.products.qcar import QCar, QCarGPS
        self._qcar = QCar(readMode=1, frequency=config["timing"]["loop_rate_hz"])
        self._gps = QCarGPS(initialPose=[0,0,0], calibrate=False)

    def read(self) -> dict:
        self._qcar.read()
        # ... read motorTach, gyroscope, accelerometer, GPS ...
        
        return { ... }

    def _hardware_write(self, throttle, steering):
        self._qcar.write(throttle=throttle, steering=steering)

    def stop(self):
        self._qcar.write(throttle=0.0, steering=0.0)