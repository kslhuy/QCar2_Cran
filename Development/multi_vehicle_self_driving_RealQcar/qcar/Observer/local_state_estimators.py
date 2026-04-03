"""
Local State Estimators for Vehicle Observer

Provides different local state estimation strategies with a common interface.
Easy to switch between different estimators (EKF, UKF, Luenberger, etc.).
"""

import numpy as np
import time
from abc import ABC, abstractmethod
from typing import Optional, Dict, Tuple
from hal.content.qcar_functions import QCarEKF


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

    def _initialize_qcar_ekf(self, initial_pose: Optional[np.ndarray]):
        """Initialize QCarEKF"""
        try:
            if initial_pose is None:
                initial_pose = np.array([0.0, 0.0, 0.0])

            self.ekf = QCarEKF(x_0=initial_pose)
            self.ekf_initialized = True

            if self.logger:
                self.logger.logger.info("QCarEKF initialized successfully")

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
                return self._update_qcar_ekf(motor_tach, steering, dt, gyro_z, gps_data)
            else:
                return self._update_fallback_ekf(
                    motor_tach, steering, dt, gyro_z, gps_data, acceleration
                )
        except Exception as e:
            if self.logger:
                self.logger.log_error("EKF update failed", e)
            return False

    def _update_qcar_ekf(
        self,
        motor_tach: float,
        steering: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict],
    ) -> bool:
        """Update using QCarEKF"""
        try:
            # GPS data is now passed from VehicleObserver (centralized sensor reading)
            y_gps = None
            gps_valid = False

            if gps_data is not None and gps_data.get("valid", False):
                gps_valid = True
                # QCarEKF expects GPS measurement as [x, y, theta]
                y_gps = np.array([gps_data["x"], gps_data["y"], gps_data["theta"]])

            # Update EKF - correct signature: update([motor_tach, steering], dt, y_gps, gyro_z)
            self.ekf.update([motor_tach, steering], dt, y_gps, gyro_z)

            # Get state from EKF (x_hat is a column vector)
            x = self.ekf.x_hat[0, 0]
            y = self.ekf.x_hat[1, 0]
            theta = self.ekf.x_hat[2, 0]

            # Apply Low-Pass Filter to smooth the motor_tach velocity
            # prev_v = self.state[3]
            # velocity = self.v_lpf_alpha * motor_tach + (1.0 - self.v_lpf_alpha) * prev_v

            # Update internal state
            self.state = np.array([x, y, theta, motor_tach])
            self.last_update_time = time.time()

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("QCarEKF update error", e)
            return False

    def _update_fallback_ekf(
        self,
        motor_tach: float,
        steering: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict],
        acceleration: Optional[np.ndarray] = None,
    ) -> bool:
        """Update using 4D Kinematic Sensor Fusion EKF implementation"""
        try:
            # Prediction step
            x, y, theta, v = self.state

            # Extract longitudinal acceleration (ax) from IMU if available, else 0
            # Assuming acceleration is [ax, ay, az]
            ax = acceleration[0] if acceleration is not None else 0.0

            # Kinematic Prediction using IMU acceleration and gyro
            v_pred = v + ax * dt
            theta_pred = theta + gyro_z * dt
            x_pred = x + v * np.cos(theta) * dt
            y_pred = y + v * np.sin(theta) * dt

            state_pred = np.array([x_pred, y_pred, theta_pred, v_pred])

            # Jacobian of motion model (F = df/dx)
            F = np.array(
                [
                    [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt],
                    [0, 1, v * np.cos(theta) * dt, np.sin(theta) * dt],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )

            # Predict covariance
            P_pred = F @ self.P @ F.T + self.Q

            # Update step
            # We always have tachometer measurement, but GPS is optional
            has_gps = gps_data is not None and gps_data.get("valid", False)

            if has_gps:
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
            else:
                # Measurement: [v] only
                z = np.array([motor_tach])
                H = np.array([[0, 0, 0, 1]])
                R_meas = np.array([[self.R[3, 3]]])  # Only velocity measurement noise

            y_res = z - H @ state_pred

            # Wrap heading residual if GPS is available (idx 2)
            if has_gps:
                y_res[2] = wrap_to_pi(y_res[2])

            S = H @ P_pred @ H.T + R_meas
            K = P_pred @ H.T @ np.linalg.inv(S)

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
        """Get current state estimate as numpy array [x, y, theta, v]"""
        return self.state.copy()

    def reset(self, initial_pose: Optional[np.ndarray] = None):
        """Reset EKF state"""
        if initial_pose is not None:
            self.state[:3] = initial_pose
            self.state[3] = 0.0
        else:
            self.state = np.zeros(self.state_dim)

        self.P = np.eye(self.state_dim) * 0.1

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
