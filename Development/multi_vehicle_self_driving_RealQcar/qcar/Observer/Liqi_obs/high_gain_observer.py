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


DEFAULT_STATE_DIM = 5
CHAIN_STATE_DIM = 6

X_INDEX = 0
Y_INDEX = 1
THETA_INDEX = 2
VELOCITY_INDEX = 3
ACCELERATION_INDEX = 4

ZX_INDEX = 0
ZVX_INDEX = 1
ZAX_INDEX = 2
ZY_INDEX = 3
ZVY_INDEX = 4
ZAY_INDEX = 5

def _wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _deep_merge_dict(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge override into base and return base."""
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge_dict(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


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


def _wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


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

        theta = float(config.get("theta", 6.7205))
        LX = np.asarray(config.get("LX", [[11.73], [20.25], [-7.3]]), dtype=float)
        LY = np.asarray(config.get("LY", [[11.73], [20.25], [-7.3]]), dtype=float)
        super().__init__(initial_pose, logger)

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
        self.max_position_innovation = _safe_float(
            config.get("max_position_innovation"), 1.0
        )

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
        self.gps_dt = 0.0

    def update(
        self,
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float = 0.0,
        gps_data: Optional[Dict] = None,
        acceleration: Optional[np.ndarray] = None,
    ) -> bool:
        # Update holder, handle various update method
        try:
            if self.observer_mode == "classical":
                return self._classical_high_gain_update(
                    motor_tach, steering, dt, gyro_z, gps_data, acceleration
                )
            return self._practical_update(
                motor_tach, steering, throttle, dt, gyro_z, gps_data, acceleration
            )
        except Exception as e:
            if self.logger:
                self.logger.log_error("Local High-Gain Observer update failed", e)
            return False
        
    def _classical_high_gain_update(self,
        motor_tach: float,
        steering: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict],
        acceleration: Optional[np.ndarray] = None,
    ) -> bool:
        try:
            # a_accel = acceleration[0] if acceleration is not None else 0.0
            # self.state = self._chain_coordinates_to_state(self.z)
            # state_dot = self._predict_bicycle_model(a_accel, gyro_z)
            # x_pred, y_pred, theta_pred, v_pred = self._euler_method(self.state, state_dot, dt)
            predict_func = (
                self.predict_high_gain_2
                if self.predict_model == 2
                else self.predict_high_gain_1
            )
            self.z = self._rk4_method(self.z, predict_func, dt)
            self.state = self._chain_coordinates_to_state(self.z)
            has_gps = gps_data is not None and gps_data.get("valid", False)
            x_pred, y_pred, theta_pred, v_pred = self.state
            # Loop Update with dt

            if has_gps and self._is_new_gps_sample(gps_data):
                self.gps_dt = dt
                gps_measure = np.array([
                        gps_data.get("x", x_pred),
                        gps_data.get("y", y_pred),
                    ])
                # GPS Loop Update with gps_dt
                z_correc_doc_gps = self._gps_correction(gps_measure)
                self.z = self.z + self.gps_dt * z_correc_doc_gps
                self.state = self._chain_coordinates_to_state(self.z)

            self.state[2] = _wrap_to_pi(self.state[2])
            self.last_update_time = time.time()
            return True
        except Exception as e:
            if self.logger:
                self.logger.log_error("Local High-Gain Observer update error", e)
            return False

    def _practical_update(
        self,
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict],
        acceleration: Optional[np.ndarray] = None,
    ) -> bool:
        try:
            dt = max(float(dt), 1e-3)
            px, vx, ax, py, vy, ay = self.z

            yaw_pred = _wrap_to_pi(float(self.state[2]) + dt * float(gyro_z))
            speed_meas = float(motor_tach) if np.isfinite(motor_tach) else 0.0
            vxy_meas = np.array(
                [speed_meas * np.cos(yaw_pred), speed_meas * np.sin(yaw_pred)]
            )

            use_gps = (
                gps_data is not None
                and gps_data.get("valid", False)
                and self._is_new_gps_sample(gps_data)
            )
            gps_x = gps_data.get("x") if use_gps else None
            gps_y = gps_data.get("y") if use_gps else None

            px, vx, ax = self._axis_observer_update(
                px, vx, ax, gps_x, vxy_meas[0], dt
            )
            py, vy, ay = self._axis_observer_update(
                py, vy, ay, gps_y, vxy_meas[1], dt
            )

            vel_vec = _clip_norm(np.array([vx, vy], dtype=float), self.max_speed)
            acc_vec = _clip_norm(np.array([ax, ay], dtype=float), self.max_accel)
            vx, vy = vel_vec
            ax, ay = acc_vec

            yaw_next = yaw_pred
            speed_est = float(np.linalg.norm(vel_vec))
            if speed_est > self.min_speed_for_heading:
                yaw_vel = float(np.arctan2(vy, vx))
                yaw_next = _wrap_to_pi(
                    yaw_pred
                    + self.yaw_correction_gain * _wrap_to_pi(yaw_vel - yaw_pred)
                )

            if use_gps and gps_data.get("theta") is not None:
                gps_theta = float(gps_data.get("theta"))
                if np.isfinite(gps_theta):
                    yaw_next = _wrap_to_pi(
                        yaw_next
                        + self.gps_heading_gain * _wrap_to_pi(gps_theta - yaw_next)
                    )

            sign_ref = np.sign(speed_meas)
            if abs(sign_ref) < 1e-9:
                sign_ref = np.sign(self.state[3])
            if abs(sign_ref) < 1e-9:
                sign_ref = 1.0

            self.z = np.array([px, vx, ax, py, vy, ay], dtype=float)
            self.state = np.array(
                [px, py, yaw_next, float(sign_ref * speed_est)], dtype=float
            )
            self.last_update_time = time.time()
            return True
        except Exception as e:
            if self.logger:
                self.logger.log_error("Local practical high-gain update error", e)
            return False

    def _axis_observer_update(
        self,
        p: float,
        v: float,
        a: float,
        p_meas: Optional[float],
        v_meas: Optional[float],
        dt: float,
    ) -> Tuple[float, float, float]:
        p_pred = float(p) + dt * float(v) + 0.5 * dt**2 * float(a)
        v_pred = float(v) + dt * float(a)
        a_pred = float(a)

        if v_meas is not None and np.isfinite(v_meas):
            v_innov = float(v_meas) - v_pred
            v_pred = v_pred + self.vel_gain * v_innov
            a_pred = a_pred + self.acc_from_vel_gain * (v_innov / dt)

        if p_meas is None or not np.isfinite(p_meas):
            return p_pred, v_pred, a_pred

        p_innov = float(p_meas) - p_pred
        if self.max_position_innovation > 0.0:
            p_innov = float(
                np.clip(
                    p_innov,
                    -self.max_position_innovation,
                    self.max_position_innovation,
                )
            )

        p_next = p_pred + self.pos_gain * p_innov
        v_next = v_pred + self.pos_to_vel_gain * (p_innov / dt)
        a_next = a_pred + self.pos_to_acc_gain * (2.0 * p_innov / dt**2)
        return p_next, v_next, a_next

    def _is_new_gps_sample(self, gps_data: Optional[Dict]) -> bool:
        if gps_data is None or not gps_data.get("valid", False):
            return False

        timestamp = gps_data.get("timestamp")
        if timestamp is None:
            return True

        try:
            timestamp = float(timestamp)
        except (TypeError, ValueError):
            return True

        if (
            self._last_corrected_gps_timestamp is not None
            and timestamp <= self._last_corrected_gps_timestamp
        ):
            return False

        self._last_corrected_gps_timestamp = timestamp
        return True

    def _gps_correction(self, gps_measure: np.ndarray) -> np.ndarray:
        estimated_measure = self.C @ self.z
        measurement_error = gps_measure - estimated_measure
        correction = self.K @ measurement_error
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
                        dt: float) -> np.ndarray:
        k1 = dynamics_func(state_variable)
        k2 = dynamics_func(state_variable + 0.5 * dt * k1)
        k3 = dynamics_func(state_variable + 0.5 * dt * k2)
        k4 = dynamics_func(state_variable + dt * k3)
        return state_variable + (k1 + 2*k2 + 2*k3 + k4)/6 * dt

    @staticmethod
    def _euler_method(state_variable: np.ndarray,
                        dynamics_func: callable,
                        dt: float) -> np.ndarray:
        return state_variable + dynamics_func(state_variable) * dt

    def _predict_bicycle_model(self, ax, gyro_z) -> np.ndarray:
        x, y, theta, v = self.state
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        v_dot = ax
        theta_dot = gyro_z
        return np.array([x_dot, y_dot, theta_dot, v_dot])
  
    def predict_high_gain_1(self, z: np.ndarray) -> np.ndarray:
        # MATLAB Hichem/Phi.m:
        # phi = [-(cross * ddy) / speed^2; (cross * ddx) / speed^2]
        x, dx, ddx, y, dy, ddy = z
        v_squared = max(float(dx**2 + dy**2), 1e-9)
        cross = -dy * ddx + dx * ddy
        dddx = -(cross * ddy) / v_squared
        dddy = (cross * ddx) / v_squared
        return np.array([dx, ddx, dddx, dy, ddy, dddy])

    def predict_high_gain_2(self, z: np.ndarray) -> np.ndarray:
        # MATLAB method2 keeps the same Phi dynamics; it adds velocity correction
        # separately. The practical mode implements that discrete correction path.
        return self.predict_high_gain_1(z)

    def get_state(self) -> np.ndarray:
        """Get current state estimate as numpy array [x, y, theta, v]."""
        return self.state.copy()

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
    
