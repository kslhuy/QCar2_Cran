"""
Local State Estimators for Vehicle Observer

Provides different local state estimation strategies with a common interface.
Easy to switch between different estimators (EKF, UKF, Luenberger, etc.).
"""

import numpy as np
import time
from abc import ABC, abstractmethod
from typing import Any, Optional, Dict, Tuple

try:
    from hal.content.qcar_functions import QCarEKF
except Exception:
    QCarEKF = None


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class LocalStateEstimatorBase(ABC):
    """Base class for all local state estimators"""

    def __init__(self, initial_pose: Optional[np.ndarray] = None, logger=None):
        """
        Initialize base state estimator

        Args:
            initial_pose: Initial pose [x, y, theta]
            logger: Logger instance
        """
        self.logger = logger
        self.state_dim = 4  # [x, y, theta, v]

        # Initialize state
        self.state = np.zeros(self.state_dim)
        if initial_pose is not None:
            self.state[:3] = initial_pose

        self.last_update_time = 0.0

    @abstractmethod
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
        """
        Update state estimate with sensor data

        Args:
            motor_tach: Motor tachometer reading (velocity)
            steering: Steering angle command
            dt: Time step
            gyro_z: Z-axis gyroscope reading (angular velocity)
            gps_data: Optional GPS data dict with keys: x, y, theta, valid
            acceleration: Optional acceleration data dict with keys: x, y, z, valid
        Returns:
            True if update successful
        """
        pass

    @abstractmethod
    def get_state(self) -> np.ndarray:
        """
        Get current state estimate

        Returns:
            numpy array [x, y, theta, velocity]
        """
        pass

    @abstractmethod
    def reset(self, initial_pose: Optional[np.ndarray] = None):
        """Reset estimator state"""
        pass


class EKFStateEstimator(LocalStateEstimatorBase):
    """
    Extended Kalman Filter (EKF) state estimator
    Uses QCarEKF or custom EKF implementation
    """

    def __init__(
        self,
        initial_pose: Optional[np.ndarray] = None,
        config: Dict = None,
        logger=None,
    ):
        """
        Initialize EKF state estimator

        Args:
            initial_pose: Initial pose [x, y, theta]
            gps: GPS instance (for QCarEKF)
            config: Configuration dict with keys: use_qcar_ekf
            logger: Logger instance
        """
        config = config or {}
        use_qcar_ekf = config.get("use_qcar_ekf", True)

        super().__init__(initial_pose, logger)

        self.use_qcar_ekf = use_qcar_ekf
        self.ekf = None
        self.ekf_initialized = False
        self.last_gain_matrix = None
        self.last_gain_measurement_labels = ()

        wheelbase = config.get("wheelbase", config.get("kin_wheelbase", 0.2))
        try:
            self.wheelbase = float(wheelbase)
        except (TypeError, ValueError):
            self.wheelbase = 0.2
        if not np.isfinite(self.wheelbase) or self.wheelbase <= 0.0:
            self.wheelbase = 0.2

        max_steering_angle = config.get("max_steering_angle", 0.5)
        try:
            self.max_steering_angle = float(max_steering_angle)
        except (TypeError, ValueError):
            self.max_steering_angle = 0.5
        if not np.isfinite(self.max_steering_angle) or self.max_steering_angle <= 0.0:
            self.max_steering_angle = 0.5

        gyro_heading_blend = config.get("gyro_heading_blend", 0.25)
        try:
            gyro_heading_blend = float(gyro_heading_blend)
        except (TypeError, ValueError):
            gyro_heading_blend = 0.25
        self.gyro_heading_blend = float(np.clip(gyro_heading_blend, 0.0, 1.0))

        self.longitudinal_model = str(
            config.get("longitudinal_model", "tachometer")
        ).strip().lower()
        valid_longitudinal_models = {
            "tachometer",
            "imu_acceleration",
            "velocity_lag",
            "velocity_lag_lookup",
            "velocity_command",
            "acceleration_lag",
            "simple_acceleration",
            "coupled_kinematic",
        }
        if self.longitudinal_model not in valid_longitudinal_models:
            self.longitudinal_model = "tachometer"
        self.use_tachometer_update = bool(config.get("use_tachometer_update", True))
        velocity_lag_cfg = config.get("velocity_lag_model", {})
        velocity_lag_lookup_cfg = config.get("velocity_lag_lookup_model", {})
        accel_lag_cfg = config.get("accel_lag_model", {})
        coupled_kinematic_cfg = config.get("coupled_kinematic_model", {})
        if not isinstance(velocity_lag_cfg, dict):
            velocity_lag_cfg = {}
        if not isinstance(velocity_lag_lookup_cfg, dict):
            velocity_lag_lookup_cfg = {}
        if not isinstance(accel_lag_cfg, dict):
            accel_lag_cfg = {}
        if not isinstance(coupled_kinematic_cfg, dict):
            coupled_kinematic_cfg = {}

        self.max_velocity = float(config.get("max_velocity", 2.0))
        self.max_acceleration = float(config.get("max_acceleration", 2.0))
        self.velocity_lag_tau = max(
            float(config.get("velocity_lag_tau", velocity_lag_cfg.get("tau", 0.301))),
            1e-6,
        )
        self.velocity_gain = float(
            config.get("velocity_gain", velocity_lag_cfg.get("velocity_gain", 6.598))
        )
        self.velocity_lag_deadband = max(
            float(
                config.get(
                    "velocity_lag_deadband",
                    velocity_lag_cfg.get("throttle_deadband", 0.0),
                )
            ),
            0.0,
        )
        self.velocity_lag_lookup_tau = max(
            float(
                velocity_lag_lookup_cfg.get(
                    "tau",
                    config.get("velocity_lag_lookup_tau", self.velocity_lag_tau),
                )
            ),
            1e-6,
        )
        self.velocity_lag_lookup_throttle_breakpoints = np.asarray(
            velocity_lag_lookup_cfg.get("throttle_breakpoints", []),
            dtype=float,
        ).reshape(-1)
        self.velocity_lag_lookup_velocity_breakpoints = np.asarray(
            velocity_lag_lookup_cfg.get("steady_state_velocity_breakpoints", []),
            dtype=float,
        ).reshape(-1)
        self.velocity_lag_lookup_enabled = bool(
            velocity_lag_lookup_cfg.get("enabled", True)
        ) and (
            self.velocity_lag_lookup_throttle_breakpoints.size >= 2
            and self.velocity_lag_lookup_throttle_breakpoints.size
            == self.velocity_lag_lookup_velocity_breakpoints.size
        )
        self.velocity_command_tau = max(
            float(config.get("velocity_command_tau", self.velocity_lag_tau)), 1e-6
        )
        self.accel_lag_tau = max(
            float(config.get("accel_lag_tau", accel_lag_cfg.get("tau", 0.318))), 1e-6
        )
        self.accel_lag_gain = float(
            config.get("accel_lag_gain", accel_lag_cfg.get("input_gain", 1.0))
        )
        self.coupled_kinematic_enabled = bool(
            coupled_kinematic_cfg.get("enabled", False)
        )
        self.coupled_kinematic_accel_features = tuple(
            coupled_kinematic_cfg.get(
                "acceleration_features",
                [
                    "bias",
                    "throttle",
                    "abs_throttle",
                    "velocity",
                    "abs_velocity",
                    "steering",
                    "steering_sq",
                    "throttle_abs_steering",
                    "velocity_steering_sq",
                ],
            )
        )
        self.coupled_kinematic_yaw_features = tuple(
            coupled_kinematic_cfg.get(
                "yaw_rate_features",
                [
                    "bias",
                    "steering",
                    "tan_steering",
                    "velocity_steering",
                    "velocity_tan_steering",
                    "throttle_steering",
                ],
            )
        )
        self.coupled_kinematic_accel_coeffs = np.asarray(
            coupled_kinematic_cfg.get("acceleration_coefficients", []),
            dtype=float,
        ).reshape(-1)
        self.coupled_kinematic_yaw_coeffs = np.asarray(
            coupled_kinematic_cfg.get("yaw_rate_coefficients", []),
            dtype=float,
        ).reshape(-1)
        self.coupled_kinematic_max_yaw_rate = max(
            float(
                coupled_kinematic_cfg.get(
                    "limits", {}
                ).get("max_yaw_rate", config.get("max_yaw_rate", 8.0))
            ),
            1e-6,
        )
        if (
            self.coupled_kinematic_accel_coeffs.size
            != len(self.coupled_kinematic_accel_features)
            or self.coupled_kinematic_yaw_coeffs.size
            != len(self.coupled_kinematic_yaw_features)
        ):
            self.coupled_kinematic_enabled = False
        self.longitudinal_accel_state = 0.0
        self.motor_accel_state = 0.0
        accel_fusion_cfg = config.get("acceleration_fusion", {})
        if not isinstance(accel_fusion_cfg, dict):
            accel_fusion_cfg = {}
        self.acceleration_fusion_enabled = bool(
            accel_fusion_cfg.get("enabled", True)
        )
        self.tach_accel_alpha = float(
            np.clip(accel_fusion_cfg.get("tach_derivative_alpha", 0.20), 0.0, 1.0)
        )
        self.accel_output_alpha = float(
            np.clip(accel_fusion_cfg.get("output_alpha", 0.35), 0.0, 1.0)
        )
        self.residual_low_mps = max(
            float(accel_fusion_cfg.get("residual_low_mps", 0.04)), 0.0
        )
        self.residual_high_mps = max(
            float(accel_fusion_cfg.get("residual_high_mps", 0.20)),
            self.residual_low_mps + 1e-6,
        )
        self.model_weight_high = float(
            np.clip(accel_fusion_cfg.get("model_weight_high", 0.90), 0.0, 1.0)
        )
        self.model_weight_low = float(
            np.clip(accel_fusion_cfg.get("model_weight_low", 0.60), 0.0, 1.0)
        )
        self.imu_weight = float(
            np.clip(accel_fusion_cfg.get("imu_weight", 0.0), 0.0, 1.0)
        )
        self.imu_residual_gate_mps2 = max(
            float(accel_fusion_cfg.get("imu_residual_gate_mps2", 1.5)), 0.0
        )
        self.acceleration_estimate = 0.0
        self.tach_accel_lpf = 0.0
        self._prev_motor_tach_for_accel = None
        self.accel_model_weight = self.model_weight_high
        self.velocity_residual = 0.0
        self.acceleration_diagnostics = {
            "a_model": 0.0,
            "a_tach": 0.0,
            "a_imu": 0.0,
            "velocity_residual": 0.0,
            "model_weight": self.accel_model_weight,
        }

        # # Low-pass filter coefficient for velocity (0 < alpha <= 1)
        # # Lower value = smoother but more delay. 1.0 = no filtering.
        # self.v_lpf_alpha = config.get("v_lpf_alpha", 0.15)

        # Fallback EKF matrices (if not using QCarEKF)
        self.P = np.eye(self.state_dim) * 0.1  # State covariance
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1])  # Process noise
        self.R = np.diag([0.3, 0.3, 0.1, 0.2])  # Measurement noise

        # Initialize QCarEKF if available
        if self.use_qcar_ekf:
            self._initialize_qcar_ekf(initial_pose)

    @staticmethod
    def _coerce_gain_matrix(gain: Any) -> Optional[np.ndarray]:
        """Convert best-effort EKF gain objects to a numeric 2D array."""
        if gain is None:
            return None
        if hasattr(gain, "detach"):
            gain = gain.detach()
        if hasattr(gain, "cpu"):
            gain = gain.cpu()
        if hasattr(gain, "numpy"):
            gain = gain.numpy()
        matrix = np.asarray(gain, dtype=float)
        if matrix.size == 0:
            return None
        if matrix.ndim == 1:
            matrix = matrix.reshape(-1, 1)
        elif matrix.ndim != 2:
            return None
        return matrix.copy()

    def _set_last_gain(
        self,
        gain: Optional[np.ndarray],
        measurement_labels: Tuple[str, ...] = (),
    ) -> None:
        matrix = self._coerce_gain_matrix(gain)
        self.last_gain_matrix = matrix
        self.last_gain_measurement_labels = tuple(str(label) for label in measurement_labels)

    def _capture_qcar_gain(
        self, measurement_labels: Tuple[str, ...] = ()
    ) -> None:
        if self.ekf is None:
            self._set_last_gain(None, measurement_labels)
            return

        for attr_name in ("K", "kalman_gain", "last_K", "last_gain", "gain"):
            if hasattr(self.ekf, attr_name):
                self._set_last_gain(getattr(self.ekf, attr_name), measurement_labels)
                return

        self._set_last_gain(None, measurement_labels)

    def _initialize_qcar_ekf(self, initial_pose: Optional[np.ndarray]):
        """Initialize QCarEKF"""
        try:
            if QCarEKF is None:
                raise RuntimeError("QCarEKF is unavailable")

            if initial_pose is None:
                initial_pose = np.array([0.0, 0.0, 0.0])

            self.ekf = QCarEKF(x_0=initial_pose)
            if hasattr(self.ekf, "L"):
                self.ekf.L = self.wheelbase
            self.ekf_initialized = True

            if self.logger:
                self.logger.logger.info(
                    f"QCarEKF initialized successfully (wheelbase={self.wheelbase:.3f} m)"
                )

        except Exception as e:
            if self.logger:
                self.logger.log_error(
                    "QCarEKF initialization failed, using fallback EKF", e
                )
            self.use_qcar_ekf = False
            self.ekf_initialized = False

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
        """Update EKF with sensor data"""
        try:
            if self.use_qcar_ekf and self.ekf_initialized:
                return self._update_qcar_ekf(
                    motor_tach, steering, throttle, dt, gyro_z, gps_data, acceleration
                )
            else:
                return self._update_fallback_ekf(
                    motor_tach, steering, throttle, dt, gyro_z, gps_data, acceleration
                )
        except Exception as e:
            if self.logger:
                self.logger.log_error("EKF update failed", e)
            return False

    def _update_qcar_ekf(
        self,
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict],
        acceleration: Optional[np.ndarray],
    ) -> bool:
        """Update using QCarEKF"""
        try:
            self._set_last_gain(None)
            prev_v = float(self.state[3])
            ax = acceleration[0] if acceleration is not None else 0.0
            v_pred, a_model, _ = self._predict_longitudinal(
                v=prev_v,
                motor_tach=motor_tach,
                throttle=throttle,
                steering=steering,
                ax=ax,
                dt=dt,
            )
            # GPS data is now passed from VehicleObserver (centralized sensor reading)
            y_gps = None
            gps_valid = False

            if gps_data is not None and gps_data.get("valid", False):
                gps_valid = True
                # QCarEKF expects GPS measurement as [x, y, theta]
                y_gps = np.array([gps_data["x"], gps_data["y"], gps_data["theta"]])

            # Update EKF - correct signature: update([motor_tach, steering], dt, y_gps, gyro_z)
            self.ekf.update([motor_tach, steering], dt, y_gps, gyro_z)
            self._capture_qcar_gain(("x", "y", "theta") if gps_valid else ())

            # Get state from EKF (x_hat is a column vector)
            x = self.ekf.x_hat[0, 0]
            y = self.ekf.x_hat[1, 0]
            theta = self.ekf.x_hat[2, 0]

            # Apply Low-Pass Filter to smooth the motor_tach velocity
            # prev_v = self.state[3]
            # velocity = self.v_lpf_alpha * motor_tach + (1.0 - self.v_lpf_alpha) * prev_v

            # Update internal state
            self.state = np.array([x, y, theta, motor_tach])
            a_tach = self._update_tach_acceleration(motor_tach, dt)
            self._fuse_acceleration(
                a_model=a_model,
                a_tach=a_tach,
                a_imu=float(ax),
                velocity_residual=float(motor_tach) - float(v_pred),
            )
            self.last_update_time = time.time()

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("QCarEKF update error", e)
            return False

    def _update_tach_acceleration(self, motor_tach: float, dt: float) -> float:
        """Causal low-pass derivative of motor tachometer velocity."""
        motor_tach = float(motor_tach)
        if dt <= 0.0 or self._prev_motor_tach_for_accel is None:
            self._prev_motor_tach_for_accel = motor_tach
            return float(self.tach_accel_lpf)

        raw_accel = (motor_tach - self._prev_motor_tach_for_accel) / dt
        self._prev_motor_tach_for_accel = motor_tach
        raw_accel = float(
            np.clip(raw_accel, -self.max_acceleration, self.max_acceleration)
        )

        alpha = self.tach_accel_alpha
        self.tach_accel_lpf = (
            alpha * raw_accel + (1.0 - alpha) * self.tach_accel_lpf
        )
        return float(self.tach_accel_lpf)

    def _effective_velocity_lag_throttle(self, throttle: float) -> float:
        """Apply calibrated throttle deadband before velocity-lag prediction."""
        u = float(throttle)
        deadband = float(self.velocity_lag_deadband)
        if deadband <= 0.0:
            return u
        mag = max(abs(u) - deadband, 0.0)
        return float(np.sign(u) * mag)

    def _velocity_lag_lookup_target(self, throttle: float) -> float:
        """Interpolate steady-state velocity from a calibrated throttle map."""
        if not self.velocity_lag_lookup_enabled:
            u_eff = self._effective_velocity_lag_throttle(throttle)
            return float(self.velocity_gain * u_eff)
        return float(
            np.interp(
                float(throttle),
                self.velocity_lag_lookup_throttle_breakpoints,
                self.velocity_lag_lookup_velocity_breakpoints,
            )
        )

    def _model_weight_from_residual(self, residual_abs: float) -> float:
        """Map velocity prediction residual to model trust weight."""
        if residual_abs <= self.residual_low_mps:
            return self.model_weight_high
        if residual_abs >= self.residual_high_mps:
            return self.model_weight_low
        ratio = (residual_abs - self.residual_low_mps) / (
            self.residual_high_mps - self.residual_low_mps
        )
        return float(
            self.model_weight_high
            + ratio * (self.model_weight_low - self.model_weight_high)
        )

    def _fuse_acceleration(
        self,
        a_model: float,
        a_tach: float,
        a_imu: float,
        velocity_residual: float,
    ) -> float:
        """
        Fuse acceleration from calibrated input model and tach derivative.

        IMU acceleration is intentionally low-trust by default. It can add a
        small correction only when it agrees with the model/tach estimate.
        """
        if not self.acceleration_fusion_enabled:
            self.acceleration_estimate = float(
                np.clip(a_model, -self.max_acceleration, self.max_acceleration)
            )
            return self.acceleration_estimate

        model_weight = self._model_weight_from_residual(abs(float(velocity_residual)))
        tach_weight = 1.0 - model_weight
        fused = model_weight * float(a_model) + tach_weight * float(a_tach)

        if self.imu_weight > 0.0 and np.isfinite(a_imu):
            if abs(float(a_imu) - fused) <= self.imu_residual_gate_mps2:
                fused = (
                    (1.0 - self.imu_weight) * fused
                    + self.imu_weight * float(a_imu)
                )

        fused = float(np.clip(fused, -self.max_acceleration, self.max_acceleration))
        alpha = self.accel_output_alpha
        self.acceleration_estimate = float(
            alpha * fused + (1.0 - alpha) * self.acceleration_estimate
        )
        self.accel_model_weight = float(model_weight)
        self.velocity_residual = float(velocity_residual)
        self.acceleration_diagnostics = {
            "a_model": float(a_model),
            "a_tach": float(a_tach),
            "a_imu": float(a_imu) if np.isfinite(a_imu) else float("nan"),
            "velocity_residual": float(velocity_residual),
            "model_weight": float(model_weight),
        }
        return self.acceleration_estimate

    def _coupled_kinematic_feature_value(
        self,
        name: str,
        velocity: float,
        throttle: float,
        steering: float,
    ) -> float:
        delta = float(
            np.clip(steering, -self.max_steering_angle, self.max_steering_angle)
        )
        tan_delta = float(np.tan(np.clip(delta, -0.7, 0.7)))
        feature_map = {
            "bias": 1.0,
            "throttle": float(throttle),
            "abs_throttle": abs(float(throttle)),
            "velocity": float(velocity),
            "abs_velocity": abs(float(velocity)),
            "steering": delta,
            "abs_steering": abs(delta),
            "steering_sq": delta * delta,
            "tan_steering": tan_delta,
            "velocity_steering": float(velocity) * delta,
            "velocity_tan_steering": float(velocity) * tan_delta,
            "throttle_steering": float(throttle) * delta,
            "throttle_abs_steering": float(throttle) * abs(delta),
            "velocity_steering_sq": float(velocity) * delta * delta,
        }
        return float(feature_map.get(str(name), 0.0))

    def _coupled_kinematic_feature_vector(
        self,
        feature_names: Tuple[str, ...],
        velocity: float,
        throttle: float,
        steering: float,
    ) -> np.ndarray:
        return np.asarray(
            [
                self._coupled_kinematic_feature_value(
                    name,
                    velocity=velocity,
                    throttle=throttle,
                    steering=steering,
                )
                for name in feature_names
            ],
            dtype=float,
        )

    def _predict_coupled_kinematic(
        self,
        v: float,
        throttle: float,
        steering: float,
        dt: float,
    ) -> Tuple[float, float, float, float]:
        accel_feat = self._coupled_kinematic_feature_vector(
            self.coupled_kinematic_accel_features,
            velocity=v,
            throttle=throttle,
            steering=steering,
        )
        yaw_feat = self._coupled_kinematic_feature_vector(
            self.coupled_kinematic_yaw_features,
            velocity=v,
            throttle=throttle,
            steering=steering,
        )
        a_pred = float(accel_feat @ self.coupled_kinematic_accel_coeffs)
        yaw_pred = float(yaw_feat @ self.coupled_kinematic_yaw_coeffs)
        a_pred = float(np.clip(a_pred, -self.max_acceleration, self.max_acceleration))
        yaw_pred = float(
            np.clip(
                yaw_pred,
                -self.coupled_kinematic_max_yaw_rate,
                self.coupled_kinematic_max_yaw_rate,
            )
        )
        v_next = float(np.clip(v + dt * a_pred, -self.max_velocity, self.max_velocity))
        v_pose = 0.5 * (float(v) + v_next)
        return v_next, a_pred, float(v_pose), yaw_pred

    def _predict_longitudinal(
        self,
        v: float,
        motor_tach: float,
        throttle: float,
        steering: float,
        ax: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        """
        Predict longitudinal velocity/acceleration.

        Returns:
            (v_pred, a_pred, v_pose), where v_pose is the velocity used by
            the bicycle pose prediction before the tachometer correction step.
        """
        model = self.longitudinal_model
        if dt <= 0.0:
            return float(v), 0.0, float(motor_tach)

        if model == "tachometer":
            a_pred = float(ax)
            v_pred = float(v) + a_pred * dt
            v_pose = float(motor_tach)
        elif model == "imu_acceleration":
            a_pred = float(ax)
            v_pred = float(v) + a_pred * dt
            v_pose = v_pred
        elif model == "velocity_lag":
            u_eff = self._effective_velocity_lag_throttle(throttle)
            a_pred = (
                -(1.0 / self.velocity_lag_tau) * float(v)
                + (self.velocity_gain / self.velocity_lag_tau) * u_eff
            )
            v_pred = float(v) + a_pred * dt
            v_pose = v_pred
        elif model == "velocity_lag_lookup":
            v_target = self._velocity_lag_lookup_target(throttle)
            a_pred = (v_target - float(v)) / self.velocity_lag_lookup_tau
            v_pred = float(v) + a_pred * dt
            v_pose = v_pred
        elif model == "velocity_command":
            a_pred = (float(throttle) - float(v)) / self.velocity_command_tau
            v_pred = float(v) + a_pred * dt
            v_pose = v_pred
        elif model == "acceleration_lag":
            self.longitudinal_accel_state += dt * (
                -(1.0 / self.accel_lag_tau) * self.longitudinal_accel_state
                + (self.accel_lag_gain / self.accel_lag_tau) * float(throttle)
            )
            a_pred = self.longitudinal_accel_state
            v_pred = float(v) + a_pred * dt
            v_pose = v_pred
        elif model == "simple_acceleration":
            a_pred = float(throttle)
            v_pred = float(v) + a_pred * dt
            v_pose = v_pred
        elif model == "coupled_kinematic" and self.coupled_kinematic_enabled:
            v_pred, a_pred, v_pose, _yaw_pred = self._predict_coupled_kinematic(
                v=float(v),
                throttle=float(throttle),
                steering=float(steering),
                dt=dt,
            )
        else:
            a_pred = float(ax)
            v_pred = float(v) + a_pred * dt
            v_pose = float(motor_tach)

        max_v = max(float(self.max_velocity), 1e-6)
        max_a = max(float(self.max_acceleration), 1e-6)
        v_pred = float(np.clip(v_pred, -max_v, max_v))
        v_pose = float(np.clip(v_pose, -max_v, max_v))
        a_pred = float(np.clip(a_pred, -max_a, max_a))
        return v_pred, a_pred, v_pose

    def _update_fallback_ekf(
        self,
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict],
        acceleration: Optional[np.ndarray] = None,
    ) -> bool:
        """Update using 4D bicycle-model sensor fusion EKF implementation."""
        try:
            self._set_last_gain(None)
            # Prediction step
            x, y, theta, v = self.state

            # Extract longitudinal acceleration (ax) from IMU if available, else 0
            # Assuming acceleration is [ax, ay, az]
            ax = acceleration[0] if acceleration is not None else 0.0

            v_pred, a_model, v_pose = self._predict_longitudinal(
                v=v,
                motor_tach=motor_tach,
                throttle=throttle,
                steering=steering,
                ax=ax,
                dt=dt,
            )
            model = self.longitudinal_model
            a_tach = self._update_tach_acceleration(motor_tach, dt)
            self._fuse_acceleration(
                a_model=a_model,
                a_tach=a_tach,
                a_imu=float(ax),
                velocity_residual=float(motor_tach) - float(v_pred),
            )
            steering = float(
                np.clip(steering, -self.max_steering_angle, self.max_steering_angle)
            )
            bicycle_yaw_rate = v_pose * np.tan(steering) / self.wheelbase
            if model == "coupled_kinematic" and self.coupled_kinematic_enabled:
                _v_pred_ck, _a_pred_ck, v_pose, yaw_rate_model = self._predict_coupled_kinematic(
                    v=float(v),
                    throttle=float(throttle),
                    steering=float(steering),
                    dt=dt,
                )
            else:
                yaw_rate_model = bicycle_yaw_rate
            # Blend bicycle model yaw rate with gyro measurement for better responsiveness
            heading_rate = (
                (1.0 - self.gyro_heading_blend) * yaw_rate_model
                + self.gyro_heading_blend * float(gyro_z)
            )

            # QCar-like bicycle prediction for pose; velocity can come from
            # tachometer, IMU acceleration, or a throttle-based model.
            theta_pred = theta + heading_rate * dt
            theta_mid = theta + 0.5 * heading_rate * dt
            x_pred = x + v_pose * np.cos(theta_mid) * dt
            y_pred = y + v_pose * np.sin(theta_mid) * dt

            state_pred = np.array([x_pred, y_pred, theta_pred, v_pred])

            # Jacobian of motion model (F = df/dx)
            F = np.array(
                [
                    [1, 0, -v_pose * np.sin(theta) * dt, 0],
                    [0, 1, v_pose * np.cos(theta) * dt, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )

            # Predict covariance
            P_pred = F @ self.P @ F.T + self.Q

            # Update step: GPS and tachometer corrections are optional so the
            # fallback can run as either sensor fusion or pure model prediction.
            has_gps = gps_data is not None and gps_data.get("valid", False)

            if has_gps and self.use_tachometer_update:
                # Measurement: [x, y, theta, v]
                z = np.array(
                    [
                        gps_data.get("x", x_pred),
                        gps_data.get("y", y_pred),
                        gps_data.get("theta", theta_pred),
                        motor_tach,
                    ]
                )
                H = np.eye(self.state_dim)
                R_meas = self.R
            elif has_gps:
                # Measurement: [x, y, theta]
                z = np.array(
                    [
                        gps_data.get("x", x_pred),
                        gps_data.get("y", y_pred),
                        gps_data.get("theta", theta_pred),
                    ]
                )
                H = np.array(
                    [
                        [1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                    ]
                )
                R_meas = self.R[:3, :3]
            elif self.use_tachometer_update:
                # Measurement: [v] only
                z = np.array([motor_tach])
                H = np.array([[0, 0, 0, 1]])
                R_meas = np.array([[self.R[3, 3]]])  # Only velocity measurement noise
            else:
                self._set_last_gain(None)
                self.state = state_pred
                self.state[2] = wrap_to_pi(self.state[2])
                self.P = P_pred
                self.last_update_time = time.time()
                return True

            y_res = z - H @ state_pred

            # Wrap heading residual if GPS is available (idx 2)
            if has_gps:
                y_res[2] = wrap_to_pi(y_res[2])

            S = H @ P_pred @ H.T + R_meas
            K = P_pred @ H.T @ np.linalg.inv(S)
            if has_gps and self.use_tachometer_update:
                measurement_labels = ("x", "y", "theta", "v")
            elif has_gps:
                measurement_labels = ("x", "y", "theta")
            else:
                measurement_labels = ("v",)
            self._set_last_gain(K, measurement_labels)

            self.state = state_pred + K @ y_res
            # Wrap heading state
            self.state[2] = wrap_to_pi(self.state[2])

            self.P = (np.eye(self.state_dim) - K @ H) @ P_pred

            self.last_update_time = time.time()
            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Fallback EKF update error", e)
            return False

    def get_state(self) -> np.ndarray:
        """Get current state estimate as numpy array [x, y, theta, v, a]."""
        return np.concatenate(
            [self.state.copy(), np.array([self.acceleration_estimate], dtype=float)]
        )

    def get_last_gain_matrix(self) -> Optional[np.ndarray]:
        if self.last_gain_matrix is None:
            return None
        return self.last_gain_matrix.copy()

    def get_last_gain_measurement_labels(self) -> Tuple[str, ...]:
        return tuple(self.last_gain_measurement_labels)

    def reset(self, initial_pose: Optional[np.ndarray] = None):
        """Reset EKF state"""
        if initial_pose is not None:
            self.state[:3] = initial_pose
            self.state[3] = 0.0
        else:
            self.state = np.zeros(self.state_dim)

        self.P = np.eye(self.state_dim) * 0.1
        self.longitudinal_accel_state = 0.0
        self.acceleration_estimate = 0.0
        self.tach_accel_lpf = 0.0
        self._prev_motor_tach_for_accel = None
        self.accel_model_weight = self.model_weight_high
        self.velocity_residual = 0.0
        self.last_gain_matrix = None
        self.last_gain_measurement_labels = ()

        # Reinitialize QCarEKF if used
        if self.use_qcar_ekf:
            self._initialize_qcar_ekf(initial_pose)


class LuenbergerStateEstimator(LocalStateEstimatorBase):
    """
    Luenberger observer for state estimation
    Simple deterministic observer with gain tuning
    """

    def __init__(
        self,
        initial_pose: Optional[np.ndarray] = None,
        config: Dict = None,
        logger=None,
    ):
        """
        Initialize Luenberger observer

        Args:
            initial_pose: Initial pose [x, y, theta]
            config: Configuration dict with keys: observer_gain
            logger: Logger instance
        """
        config = config or {}
        observer_gain = config.get("observer_gain", 0.5)

        super().__init__(initial_pose, logger)

        self.L = np.eye(self.state_dim) * observer_gain  # Observer gain matrix

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
        """Update Luenberger observer"""
        try:
            x, y, theta, v = self.state

            # Prediction (bicycle model)
            x_pred = x + v * np.cos(theta) * dt
            y_pred = y + v * np.sin(theta) * dt
            theta_pred = theta + gyro_z * dt
            v_pred = motor_tach

            state_pred = np.array([x_pred, y_pred, theta_pred, v_pred])

            # Correction (if measurement available)
            if gps_data is not None and gps_data.get("valid", False):
                measurement = np.array(
                    [
                        gps_data.get("x", x_pred),
                        gps_data.get("y", y_pred),
                        gps_data.get("theta", theta_pred),
                        motor_tach,
                    ]
                )

                # Observer update: x_new = x_pred + L * (measurement - x_pred)
                residual = measurement - state_pred
                residual[2] = wrap_to_pi(residual[2])
                self.state = state_pred + self.L @ residual
                self.state[2] = wrap_to_pi(self.state[2])
            else:
                self.state = state_pred

            self.last_update_time = time.time()
            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Luenberger observer update error", e)
            return False

    def get_state(self) -> np.ndarray:
        """Get current state estimate as numpy array [x, y, theta, v]"""
        return self.state.copy()

    def reset(self, initial_pose: Optional[np.ndarray] = None):
        """Reset observer state"""
        if initial_pose is not None:
            self.state[:3] = initial_pose
            self.state[3] = 0.0
        else:
            self.state = np.zeros(self.state_dim)


class LocalEstimatorFactory:
    """Factory to create local state estimators by name"""

    ESTIMATOR_TYPES = {
        "ekf": EKFStateEstimator,
        "luenberger": LuenbergerStateEstimator,
    }

    @staticmethod
    def _lazy_load_neural_estimator():
        """Lazy load neural estimator to avoid import errors if dependencies missing"""
        try:
            # Use importlib to handle directory name starting with number (2LayerObs)
            import importlib

            module = importlib.import_module(
                "Observer.LocalNeuralObs.2LayerObs.neural_state_estimator"
            )
            return module.NeuralLuenbergerEstimator
        except (ImportError, ModuleNotFoundError):
            try:
                # Fallback to root level LocalNeuralObs lazy import
                from Observer.LocalNeuralObs import NeuralLuenbergerEstimator

                return NeuralLuenbergerEstimator
            except ImportError as e:
                raise ImportError(
                    f"Neural estimator requires additional dependencies (torch). "
                    f"Install with: pip install torch. Error: {e}"
                )

    @staticmethod
    def _lazy_load_robust_kalman_net_estimator():
        """Lazy load Robust KalmanNet estimator to avoid hard torch dependency."""
        try:
            import importlib

            module = importlib.import_module("Observer.KalmaNet.Robust.robustKLnet")
            return module.RobustKalmanNetStateEstimator
        except (ImportError, ModuleNotFoundError) as e:
            raise ImportError(
                "Robust KalmanNet estimator requires torch and the "
                "Observer/KalmaNet/Robust package to be importable. "
                f"Error: {e}"
            )

    @staticmethod
    def create(
        estimator_type: str,
        initial_pose: Optional[np.ndarray] = None,
        gps=None,
        logger=None,
        config: Dict = None,
    ) -> LocalStateEstimatorBase:
        """
        Create a local state estimator

        Args:
            estimator_type: One of 'ekf', 'luenberger', 'dead_reckoning',
                'neural_luenberger', 'robust_kalman_net'
            initial_pose: Initial pose [x, y, theta]
            gps: GPS instance (for EKF)
            logger: Logger instance
            config: Configuration parameters for the estimator

        Returns:
            Local state estimator instance
        """
        # Handle neural estimator separately (lazy loading)
        if estimator_type == "neural_luenberger":
            NeuralLuenbergerEstimator = (
                LocalEstimatorFactory._lazy_load_neural_estimator()
            )
            return NeuralLuenbergerEstimator(
                initial_pose=initial_pose, config=config, logger=logger
            )

        if estimator_type == "robust_kalman_net":
            RobustKalmanNetStateEstimator = (
                LocalEstimatorFactory._lazy_load_robust_kalman_net_estimator()
            )
            return RobustKalmanNetStateEstimator(
                initial_pose=initial_pose, config=config, logger=logger
            )

        # Standard estimators
        if estimator_type not in LocalEstimatorFactory.ESTIMATOR_TYPES:
            raise ValueError(
                f"Unknown estimator type: {estimator_type}. "
                f"Available: {list(LocalEstimatorFactory.ESTIMATOR_TYPES.keys()) + ['neural_luenberger', 'robust_kalman_net']}"
            )

        estimator_class = LocalEstimatorFactory.ESTIMATOR_TYPES[estimator_type]
        return estimator_class(initial_pose=initial_pose, config=config, logger=logger)

        # Return the estimator instance
        # estimator_class = LocalEstimatorFactory.ESTIMATOR_TYPES[estimator_type]
        # return estimator_class(initial_pose=initial_pose, config=config, logger=logger)
