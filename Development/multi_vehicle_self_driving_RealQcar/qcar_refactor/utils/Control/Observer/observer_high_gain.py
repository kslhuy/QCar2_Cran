"""EKF-based observer. Implements BaseObserver.

5D kinematic EKF: state = [x, y, theta, v, a]

Predict:   command-driven — steering → yaw rate, throttle → acceleration
Correct:   sensor fusion — motor_tach (v), gyro (yaw rate), accel_x (a), GPS (x,y,θ)
"""

import time
import numpy as np
from typing import Optional

from .observer_base import ObserverBase
from core.types import VehicleStateEstimate, ControlInput, SensorData


def _wrap(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


# ── Read wheelbase from config ──────────────────────────────────

class ObserverEKF(ObserverBase):
    __slots__ = ('_ekf', '_started', '_last_accel_magnitude', '_wheelbase')

    def __init__(
        self,
        config: dict,
        vehicle_id: int = 0,
        logger=None,
        wheelbase: Optional[float] = None,
    ) -> None:
        effective_config = dict(config)
        if wheelbase is not None:
            effective_config["wheelbase"] = wheelbase
        super().__init__(effective_config, vehicle_id, logger)
        self._ekf = None
        self._started = False
        self._last_accel_magnitude = 0.0
        self._wheelbase = float(self._config.get("wheelbase", 0.3))

    def start(self, initial_pose: Optional[list] = None) -> None:
        if initial_pose is None:
            initial_pose = self._config.get("initial_pose")
        if isinstance(initial_pose, dict):
            initial_pose = [
                initial_pose.get("x", 0.0),
                initial_pose.get("y", 0.0),
                initial_pose.get("theta", 0.0),
            ]
        init = np.array(initial_pose, dtype=float) if initial_pose else None
        self._ekf = _EKF(init, wheelbase=self._wheelbase)
        self._started = True
        self._logger.info("EKFObserver started")

    def update(self, sensor_data: SensorData, dt: float,
               last_command: Optional[ControlInput] = None) -> VehicleStateEstimate:
        if not self._started:
            raise RuntimeError("EKFObserver not started")

        motor_tach = float(sensor_data.motor_tach)
        gyro_z     = float(sensor_data.gyro_z)
        accel      = sensor_data.accelerometer
        accel_xy   = self._horizontal_acceleration(accel)
        self._last_accel_magnitude = accel_xy

        steering = float(last_command.steering) if last_command is not None else None
        throttle = float(last_command.throttle) if last_command is not None else 0.0

        gps = None
        if sensor_data.gps_valid:
            p = sensor_data.gps_position
            gps = {"x": float(p[0]), "y": float(p[1]), "theta": float(p[2])}

        self._ekf.update(motor_tach, gyro_z, dt, accel_xy, gps,
                         steering=steering, throttle=throttle)
        s = self._ekf.get_state()

        return self.assess_estimate(VehicleStateEstimate(
            timestamp=sensor_data.sensor_timestamp,
            x=s[0], y=s[1], theta=s[2], velocity=s[3],
            acceleration=self._last_accel_magnitude,
            gps_valid=sensor_data.gps_valid,
        ))

    def get_latest(self) -> VehicleStateEstimate:
        if self._ekf is None:
            self._logger.warning("EKF not started, returning default state")
            return VehicleStateEstimate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        s = self._ekf.get_state()
        return VehicleStateEstimate(
            time.time(), s[0], s[1], s[2], s[3], self._last_accel_magnitude, False)

    def stop(self) -> None:
        self._started = False

    @staticmethod
    def _horizontal_acceleration(accel) -> float:
        """Return sqrt(ax^2 + ay^2), ignoring gravity on z."""
        if accel is None or len(accel) == 0:
            return 0.0
        accel_array = np.asarray(accel, dtype=float).reshape(-1)
        ax = float(accel_array[0]) if accel_array.size > 0 else 0.0
        ay = float(accel_array[1]) if accel_array.size > 1 else 0.0
        return float(np.hypot(ax, ay))


# ── Internal 5D EKF math ────────────────────────────────────────

class _EKF:
    """
    5D kinematic Extended Kalman Filter: state = [x, y, theta, v, a]

    Predict:   command-driven
                 a ← throttle (trend, resets each step)
                 v ← v + a·dt
                 θ ← θ + v·tan(δ)/L·dt  (bicycle model from steering)
                 Falls back to gyro when steering unavailable.
                 x,y from standard bicycle kinematics.

    Correct:   sensor fusion
                 GPS:     full [x, y, θ, v] correction
                 No GPS:  [motor_tach, gyro_z, accel_x] → [v, yaw_rate, a]
    """

    def __init__(self, initial_pose=None, wheelbase: float = 0.3):
        self.x = np.zeros(5)   # [x, y, theta, v, a]
        if initial_pose is not None:
            self.x[:3] = np.asarray(initial_pose, dtype=float).flatten()[:3]

        self.L = wheelbase

        # Covariance
        self.P = np.diag([0.1, 0.1, 0.1, 0.1, 1.0])  # a uncertain at start

        # Process noise — a gets full reset from command, so higher Q
        self.Q = np.diag([0.01, 0.01, 0.01, 0.05, 0.3])

        # Measurement noise
        self.R_gps = np.diag([0.3, 0.3, 0.1, 0.2])      # x,y,θ,v with GPS
        self.R_imu = np.diag([0.2, 0.1, 0.5])             # v, yaw_rate, a

    def update(self, motor_tach, gyro_z, dt, accel_x=0.0,
               gps=None, steering=None, throttle=0.0):
        # ── 1. Predict ──────────────────────────────────────
        x, y, theta, v, a = self.x
        has_gps = gps is not None

        # Acceleration: throttle gives the trend (command-driven, no memory)
        a_pred = throttle

        # Velocity: integrate acceleration
        v_pred = v + a * dt

        # Heading: bicycle model from steering, fallback to gyro
        if steering is not None:
            theta_pred = _wrap(theta + v * np.tan(steering) / self.L * dt)
            dtheta_dv  = np.tan(steering) / self.L * dt
        else:
            theta_pred = _wrap(theta + gyro_z * dt)
            dtheta_dv  = 0.0

        # Position
        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt

        xp = np.array([x_pred, y_pred, theta_pred, v_pred, a_pred])

        # Jacobian of motion model w.r.t. state (5×5)
        # a_pred = throttle → ∂a_pred/∂a = 0 (no state memory)
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt, 0],
            [0, 1,  v * np.cos(theta) * dt, np.sin(theta) * dt, 0],
            [0, 0,                     1,        dtheta_dv,     0],
            [0, 0,                     0,                1,    dt],
            [0, 0,                     0,                0,     0],
        ])
        Pp = F @ self.P @ F.T + self.Q

        # ── 2. Correct ──────────────────────────────────────
        if has_gps:
            # Full state correction: x, y, θ, v
            # a is unobservable from GPS alone, kept from prediction
            z = np.array([gps["x"], gps["y"], gps["theta"], motor_tach])
            H = np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
            ])
            R = self.R_gps
        else:
            # Sensor fusion: v (tach), yaw rate (gyro), a (accel_x)
            z = np.array([motor_tach, gyro_z, accel_x])
            yaw_rate_factor = np.tan(steering) / self.L if steering is not None else 0.0
            H = np.array([
                [0, 0, 0, 1, 0],
                [0, 0, 0, yaw_rate_factor, 0],
                [0, 0, 0, 0, 1],
            ])
            R = self.R_imu

        # Innovation
        y_res = z - H @ xp
        if has_gps:
            y_res[2] = _wrap(y_res[2])

        # Kalman gain
        S = H @ Pp @ H.T + R
        K = Pp @ H.T @ np.linalg.inv(S)

        # Update
        self.x = xp + K @ y_res
        self.x[2] = _wrap(self.x[2])
        self.P = (np.eye(5) - K @ H) @ Pp

    def get_state(self):
        return self.x.copy()

    def reset(self, initial_pose=None):
        self.x = np.zeros(5)
        if initial_pose is not None:
            self.x[:3] = np.asarray(initial_pose, dtype=float).flatten()[:3]
        self.P = np.diag([0.1, 0.1, 0.1, 0.1, 1.0])
