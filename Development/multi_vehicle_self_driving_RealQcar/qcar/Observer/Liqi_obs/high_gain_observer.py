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
        initial_pose: np.ndarray,
        config: Dict,
        logger: Any,
    ):
        config = config or {}
        theta = config.get("theta", 6.7205)
        LX = config.get("LX", np.array([[11.73], [20.25], [-7.3]]))
        LY = config.get("LY", np.array([[11.73], [20.25], [-7.3]]))
        super().__init__(initial_pose, logger)

        X0, Y0 = initial_pose[:2] 
        self.initial_z = np.array([X0, 0, 0, Y0, 0, 0])
        self.z = self.initial_z.copy()
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
            return self._high_gain_update(motor_tach, steering, dt, gyro_z, gps_data, acceleration)
        except Exception as e:
            if self.logger:
                self.logger.log_error("Local High-Gain Observer update failed", e)
            return False
        
    def _high_gain_update(self,
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
            self.z = self._rk4_method(self.z, self.predict_high_gain_1, dt)
            self.state = self._chain_coordinates_to_state(self.z)
            has_gps = gps_data is not None and gps_data.get("valid", False)
            x_pred, y_pred, theta_pred, v_pred = self.state
            # Loop Update with dt

            if has_gps:
                self.gps_dt = time.time() - self.last_update_time if self.last_update_time else 0.0
                gps_measure = np.array([
                        gps_data.get("x", x_pred),
                        gps_data.get("y", y_pred),
                        gps_data.get("theta", theta_pred),
                        motor_tach,
                    ])
                # GPS Loop Update with gps_dt
                z_correc_doc_gps = self._gps_correction(gps_measure)
                self.z = self.z + self.gps_dt * z_correc_doc_gps
                self.last_update_time = time.time()
            else:
                pass
            return True
        except Exception as e:
            if self.logger:
                self.logger.log_error("Local High-Gain Observer update error", e)
            return False

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
  
    def predict_high_gain_1(self) -> np.ndarray:
        # Assumption: Constant or 0 slip angle and constant velocity
        x, dx, ddx, y, dy, ddy = self.z
        v = np.sqrt(dx**2 + dy**2)
        dddx = -dx * ( (ddy*dx - ddx*dy) / v**2 )**2
        dddy = -dy * ( (ddy*dx - dy*ddx) / v**2 )**2
        return np.array([dx, ddx, dddx, dy, ddy, dddy])

    def predict_high_gain_2(self) -> np.ndarray:
        # Assumption: Constant or 0 slip angle but constant acceleration
        x, dx, ddx, y, dy, ddy = self.z
        v = np.sqrt(dx**2 + dy**2)
        dddx = -3*ddy * (ddy*dx - ddx*dy) / v**2 + 2*dx * ( (ddy*dx - ddx*dy) / v**2 )**2
        dddy = 3*ddx * (ddy*dx - ddx*dy) / v**2 + 2*dy * ( (ddy*dx - ddx*dy) / v**2 )**2
        return np.array([dx, ddx, dddx, dy, ddy, dddy])
    