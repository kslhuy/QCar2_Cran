"""
Local high-gain observer using an internal transformed companion state.

External compatibility is preserved:

    fleet_states[:, j] = [x, y, theta, velocity, acceleration]

Internally, the observer can run on transformed chain coordinates:

    z = [x, x_dot, x_ddot, y, y_dot, y_ddot]

This matches the usual high-gain observer idea better than correcting directly
on the mixed bicycle state. Prediction still uses the QCar bicycle model, then
maps back into the transformed coordinates for high-gain and consensus updates.
"""

import copy
import os
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np
import scipy as sp
import yaml


from ..local_state_estimators import LocalStateEstimatorBase


def _wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _safe_float(value: Any, default: float) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _clip_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    if max_norm <= 0.0:
        return vec
    norm = float(np.linalg.norm(vec))
    if norm > max_norm and norm > 1e-12:
        return vec * (max_norm / norm)
    return vec


class HighGainObserver(LocalStateEstimatorBase):
    def __init__(
        self,
        initial_pose: Optional[np.ndarray] = None,
        config: Optional[Dict] = None,
        logger: Any = None,
    ):
        config = config or {}
        if initial_pose is None:
            initial_pose = np.array([0.0, 0.0, 0.0])
        super().__init__(initial_pose, logger)

        theta = float(config.get("theta", 6.7205))
        LX = np.asarray(config.get("LX", [[1], [2], [1]]), dtype=float)
        LY = np.asarray(config.get("LY", [[1], [2], [1]]), dtype=float)
        N = np.asarray(config.get("N", [[4.72, 9.72], [8.15, 1.14], [4.71, 0.71],
                                        [4.72, 9.72], [8.14, 1.14], [4.71, 0.71]]), dtype=float)

        self.observer_mode = str(config.get("observer_mode", "practical")).lower()
        self.pos_gain = _safe_float(config.get("pos_gain"), 0.35)
        self.pos_to_vel_gain = _safe_float(config.get("pos_to_vel_gain"), 0.12)
        self.pos_to_acc_gain = _safe_float(config.get("pos_to_acc_gain"), 0.03)
        self.vel_gain = _safe_float(config.get("vel_gain"), 0.30)
        self.acc_from_vel_gain = _safe_float(config.get("acc_from_vel_gain"), 0.10)
        self.yaw_correction_gain = _safe_float(config.get("yaw_correction_gain"), 0.08)
        self.gps_heading_gain = _safe_float(config.get("gps_heading_gain"), 0.05)
        self.min_speed_for_heading = _safe_float(config.get("min_speed_for_heading"), 0.5)
        self.max_speed = _safe_float(config.get("max_speed"), 5.0)
        self.max_accel = _safe_float(config.get("max_accel"), 8.0)
        self.max_position_innovation = _safe_float(config.get("max_position_innovation"), 1.0)

        self.initial_z = self._state_to_chain_coordinates(self.state)
        self.z = self.initial_z.copy()
        self.predict_model = int(config.get("predict_model", 1))
        self._last_corrected_gps_timestamp = None
        self.T = np.diag([theta, theta**2, theta**3, theta, theta**2, theta**3])
        self.K = self.T @ sp.linalg.block_diag(LX, LY)
        self.A = np.array([[0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1],
                            [0, 0, 0, 0, 0, 0]])
        self.B = np.array([[0, 0],
                           [0, 0],
                           [1, 0],
                           [0, 0],
                           [0, 0],
                           [0, 1]])
        self.C = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0]])
        self.N = self.T @ N // (theta**3) // 5
        self.gps_dt = 0.0
        self.last_gps_update_time = 0.0

    def update(
        self,
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float = 0.0,
        gps_data: Optional[Dict] = None,
        acceleration: Optional[np.ndarray] = None) -> bool:
        # Update holder, handle various update method
        if self.observer_mode == "classical" or self.observer_mode == "extra_sensor":
            return self._classical_high_gain_update(
                motor_tach, steering, dt, gyro_z, gps_data, acceleration
            )
        else:
            if self.logger:
                self.logger.log_error(f"Invalid observer_mode: {self.observer_mode}")
            return False

    def _classical_high_gain_update(self,
        motor_tach: float,
        steering: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict] = None,
        acceleration: Optional[np.ndarray] = None,
    ) -> bool:
        try:
            v_meas_squared = self.z[1]**2 + self.z[4]**2
            if v_meas_squared < 0.1:
                v_meas_squared = 0.1  # Prevent division by zero or very small numbers
            predict_func = (self.predict_high_gain_1 if self.predict_model == 1 else self.predict_high_gain_2)
            self.z = self._rk4_method(self.z, predict_func, dt, v_meas_squared)
            if self.observer_mode == "extra_sensor":
                z_correc_doc_sensor = self._sensors_correction(motor_tach, steering, gyro_z, acceleration)
                self.z = self.z + dt * z_correc_doc_sensor
            self.state = self._chain_coordinates_to_state(self.z)
            x_pred, y_pred, theta_pred, v_pred = self.state
            # Loop Update with dt
            if gps_data is not None and gps_data.get("valid", False):
                gps_measure = np.array([gps_data.get("x", x_pred),
                                        gps_data.get("y", y_pred),])
                z_correc_doc_gps = self._gps_correction(gps_measure)
                self.gps_dt = time.time() - self.last_gps_update_time
                self.z = self.z + self.gps_dt * z_correc_doc_gps
                self.state = self._chain_coordinates_to_state(self.z)
                self.instability_reset_with_gps(gps_measure)
                self.last_gps_update_time = time.time()
            self.state[2] = _wrap_to_pi(self.state[2])
            self.last_update_time = time.time()
            return True
        except Exception as e:
            if self.logger:
                self.logger.log_error("Local High-Gain Observer update error", e)
            return False

    def _gps_correction(self, gps_measure: np.ndarray) -> np.ndarray:
        estimated_measure = self.C @ self.z
        measurement_error = np.clip(gps_measure - estimated_measure, -self.max_position_innovation, self.max_position_innovation)
        correction = self.K @ measurement_error
        return correction
    
    def _sensors_correction(self,
                            motor_tach: float, 
                            steering: float, 
                            gyro_z: float, 
                            acceleration: Optional[np.ndarray]) -> np.ndarray:
        self.v_est = self.state[3]
        cross_est = self.z[2] * self.z[1] + self.z[4] * self.z[5]
        cross_meas = cross_est
        correction = self.N @ np.array([motor_tach - self.v_est,
                                        cross_meas - cross_est], dtype=float)
        return correction

    
    @staticmethod
    def _state_to_chain_coordinates(state: np.ndarray) -> np.ndarray:
        x, y, theta, v = state
        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        ddx = 0
        ddy = 0
        return np.array([x, dx, ddx, y, dy, ddy])
    
    @staticmethod
    def _chain_coordinates_to_state(z: np.ndarray) -> np.ndarray:
        x, dx, ddx, y, dy, ddy = z
        theta = np.arctan2(dy, dx)
        v = np.sqrt(dx**2 + dy**2)
        return np.array([x, y, theta, v])
    
    @staticmethod
    def _rk4_method(state_variable: np.ndarray,
                        dynamics_func: callable,
                        dt: float,
                        params=None) -> np.ndarray:
        k1 = dynamics_func(state_variable, params)
        k2 = dynamics_func(state_variable + 0.5 * dt * k1, params)
        k3 = dynamics_func(state_variable + 0.5 * dt * k2, params)
        k4 = dynamics_func(state_variable + dt * k3, params)
        return state_variable + (k1 + 2*k2 + 2*k3 + k4)/6 * dt

    @staticmethod
    def _euler_method(state_variable: np.ndarray,
                        dynamics_func: callable,
                        dt: float,
                        params=None) -> np.ndarray:
        return state_variable + dynamics_func(state_variable, params) * dt

    def _predict_bicycle_model(self, ax, gyro_z) -> np.ndarray:
        x, y, theta, v = self.state
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        v_dot = ax
        theta_dot = gyro_z
        return np.array([x_dot, y_dot, theta_dot, v_dot])
  
    @staticmethod
    def predict_high_gain_1(z: np.ndarray, v_meas_squared: float) -> np.ndarray:
        x, dx, ddx, y, dy, ddy = z
        cross = -dy * ddx + dx * ddy
        dddx = -dx * ((cross * ddy) / v_meas_squared)**2
        dddy = -dy * ((cross * ddy) / v_meas_squared)**2
        dx = np.clip(dx, -2, 2)
        dy = np.clip(dy, -2, 2)
        dddx = np.clip(dddx, -1e7, 1e7)
        dddy = np.clip(dddy, -1e7, 1e7)
        return np.array([dx, ddx, dddx, dy, ddy, dddy])

    @staticmethod
    def predict_high_gain_2(z: np.ndarray, v_meas_squared: float) -> np.ndarray:
        x, dx, ddx, y, dy, ddy = z
        cross = -dy * ddx + dx * ddy
        dddx = -3*ddy * cross / v_meas_squared + 2*dx * ( cross/ v_meas_squared )**2
        dddy = 3*ddx * cross / v_meas_squared + 2*dy * ( cross/v_meas_squared )**2
        dx = np.clip(dx, -2, 2)
        dy = np.clip(dy, -2, 2)
        
        dddx = np.clip(dddx, -1e7, 1e7)
        dddy = np.clip(dddy, -1e7, 1e7)
        return np.array([dx, ddx, dddx, dy, ddy, dddy])

    def get_state(self) -> np.ndarray:
        """Get current state estimate as numpy array [x, y, theta, v]."""
        return self.state.copy()
    
    def instability_reset_with_gps(self, gps_measure: np.ndarray) -> bool:
        """Reset observer state if GPS measurement is valid and significantly different from current estimate."""
        current_pos = self.state[:2]
        pos_diff = np.linalg.norm(gps_measure - current_pos)
        if pos_diff > self.max_position_innovation * 2:  # Threshold for instability
            self.reset(initial_pose=np.array([gps_measure[0], gps_measure[1], 0.0]))
            return True
        return False

    def reset(self, initial_pose: Optional[np.ndarray] = None):
        """Reset high-gain observer state."""
        if initial_pose is None:
            initial_pose = np.array([0.0, 0.0, 0.0])
        self.state = np.zeros(self.state_dim)
        self.state[:3] = initial_pose[:3]
        self.initial_z = self._state_to_chain_coordinates(self.state)
        self.z = self.initial_z.copy()
        self.gps_dt = 0.0
        self._last_corrected_gps_timestamp = None
        self.last_update_time = 0.0