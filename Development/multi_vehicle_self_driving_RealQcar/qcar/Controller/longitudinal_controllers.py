"""
Longitudinal Controllers for Vehicle Following

Provides different longitudinal control strategies with a common interface.
Easy to switch between different controllers.
"""

import math
import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any


def _wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class LongitudinalControllerBase(ABC):
    """Base class for all longitudinal controllers"""

    @abstractmethod
    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        """
        Compute throttle command

        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity'
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (or None)
            dt: Time step

        Returns:
            Throttle command (-1 to 1)
        """
        pass

    @abstractmethod
    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically"""
        pass

    @abstractmethod
    def reset(self):
        """Reset controller state"""
        pass


class PIDVelocityController(LongitudinalControllerBase):
    """
    PID velocity controller with anti-windup
    Compatible with both platoon following and standalone path following
    """

    def __init__(
        self,
        kp=0.1,
        ki=1.0,
        kd=0.01,
        max_throttle=0.3,
        min_throttle=0.0,
        ei_max=1.0,
        v_ref=0.5,
        config=None,
        logger=None,
        **kwargs,
    ):
        """
        Initialize PID velocity controller

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            max_throttle: Maximum throttle output
            min_throttle: Minimum throttle output when v_ref > 0
            ei_max: Integral anti-windup limit
            v_ref: Reference velocity (used if target_velocity not provided)
            config: Optional config object (takes precedence over individual params)
            logger: Logger instance
        """
        self.logger = logger

        # Use config if provided, otherwise use individual parameters
        if config and hasattr(config, "get_longitudinal_params"):
            # ControllerConfig - get params from dictionary
            params = config.get_longitudinal_params("pid")
            self.kp = params.get("kp", kp)
            self.ki = params.get("ki", ki)
            self.kd = params.get("kd", kd)
            self.max_throttle = params.get("max_throttle", max_throttle)
            self.min_throttle = params.get("min_throttle", min_throttle)
            self.ei_max = params.get("ei_max", ei_max)
        else:
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.max_throttle = max_throttle
            self.min_throttle = min_throttle
            self.ei_max = ei_max

        # Simple feedforward gain based on calibration (v_ss = K * u)
        # Average K for QCar is ~6.2 m/s per unit throttle.
        self.ff_gain = 1.0 / 6.2  # u_ff = v_ref * ff_gain

        # print(f"[PIDVelocityController] Initialized with kp={self.kp}, ki={self.ki}, kd={self.kd}, max_throttle={self.max_throttle}")
        self.ei = 0.0  # Integral error
        self.prev_e = None  # Previous error for derivative
        self.last_error = 0.0

    def update(self, v: float, v_ref: float, dt: float) -> float:
        """
        Update speed controller (path following interface)

        Args:
            v: Current velocity
            v_ref: Reference velocity
            dt: Time step

        Returns:
            Throttle command
        """
        # Calculate error
        e = v_ref - v

        # Integral with anti-windup
        self.ei += dt * e
        self.ei = np.clip(self.ei, -self.ei_max, self.ei_max)

        # Calculate derivative of the error
        if self.prev_e is None or dt < 0.001:
            de = 0
        else:
            de = (e - self.prev_e) / dt
        self.prev_e = e

        # Feedforward term based on target velocity
        u_ff = v_ref * self.ff_gain

        # PID control + Feedforward
        u = u_ff + self.kp * e + self.ki * self.ei + self.kd * de

        # Prevent reversing during forward driving
        # A negative PID output means we want to slow down, but pushing
        # 0.0 to a DC motor will immediately brake hard/reverse on 1/10 scale.
        if v_ref > 0.05:
            # We are trying to drive forward, keep a small forward throttle to coast smoothly
            u = np.clip(u, self.min_throttle, self.max_throttle)
        else:
            # We actually want to stop
            if u < 0.0:
                u = 0.0
            else:
                u = np.clip(u, 0.0, self.max_throttle)

        # print(f"[PIDVelocityController] v: {v:.2f}, v_ref: {v_ref:.2f}, e: {e:.2f}, ei: {self.ei:.2f}, de: {de:.2f}, throttle: {u:.2f}")

        self.last_error = e

        return u

    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        """
        Compute PID control based on velocity error (platoon following interface)

        Args:
            follower_state: Dict with keys 'velocity', 'target_velocity'
            leader_state: Not used for PID controller
            dt: Time step

        Returns:
            Throttle command
        """
        current_velocity = follower_state["velocity"]
        target_velocity = follower_state.get("target_velocity", 0.0)

        # Use the update method to maintain state consistency
        return self.update(current_velocity, target_velocity, dt)

    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically"""
        for param, value in params.items():
            if hasattr(self, param):
                setattr(self, param, value)

        if self.logger:
            self.logger.logger.info(f"[PIDVelocityController] Updated params: {params}")

    def reset(self):
        """Reset controller state"""
        self.ei = 0.0
        self.prev_e = None
        self.last_error = 0.0


class CACCLongitudinalController(LongitudinalControllerBase):
    """
    CACC-based longitudinal controller
    Uses spacing error and velocity error to compute acceleration,
    then converts to throttle command
    """

    def __init__(
        self,
        s0=1.5,
        h=0.5,
        K=None,
        acc_to_throttle_gain=0.5,
        max_throttle=0.3,
        alpha_filter=0.3,
        ki_velocity=0.1,
        brake_smoothing=0.5,
        max_acc_rate=2.0,
        spacing_deadband=0.2,
        velocity_deadband=0.05,
        throttle_smoothing=0.7,
        spacing_mode="path_or_projected",
        projection_heading_source="leader",
        blend_heading_deg=20.0,
        min_effective_spacing=0.0,
        config=None,
        logger=None,
    ):
        """
        Initialize CACC longitudinal controller

        Args:
            s0: Minimum spacing (meters)
            h: Time headway (seconds)
            K: Control gains [spacing_gain, velocity_gain]
            acc_to_throttle_gain: Gain to convert acceleration to throttle
            max_throttle: Maximum throttle output
            alpha_filter: Low-pass filter coefficient (0-1)
            ki_velocity: Velocity integral gain for additional stability
            brake_smoothing: Smoothing factor for negative throttle (0-1, higher = smoother)
            max_acc_rate: Maximum acceleration rate of change (m/s^3) to limit jerk
            spacing_deadband: Spacing error deadband to prevent oscillations (meters)
            velocity_deadband: Velocity error deadband to prevent oscillations (m/s)
            throttle_smoothing: Exponential smoothing factor for throttle (0-1, higher = smoother)
            spacing_mode: Gap metric ('euclidean', 'projected', 'blended', 'path', 'path_or_projected')
            projection_heading_source: Heading source for projection ('leader', 'follower', 'average')
            blend_heading_deg: Heading difference where blended mode becomes fully projected
            min_effective_spacing: Minimum spacing used by controller (can be <= 0 for signed spacing)
            config: Optional config object (takes precedence)
            logger: Logger instance
        """
        self.logger = logger

        # Use config if provided
        if config and hasattr(config, "get_longitudinal_params"):
            # ControllerConfig - get params from dictionary
            params = config.get_longitudinal_params("cacc")
            self.s0 = params.get("s0", s0)
            self.h = params.get("h", h)
            self.K = params.get("K", K if K is not None else np.array([[0.2, 0.05]]))
            self.acc_to_throttle_gain = params.get(
                "acc_to_throttle_gain", acc_to_throttle_gain
            )
            self.max_throttle = params.get("max_throttle", max_throttle)
            self.alpha_filter = params.get("alpha_filter", alpha_filter)
            self.ki_velocity = params.get("ki_velocity", ki_velocity)
            self.spacing_mode = params.get("spacing_mode", spacing_mode)
            self.projection_heading_source = params.get(
                "projection_heading_source", projection_heading_source
            )
            self.blend_heading_deg = params.get("blend_heading_deg", blend_heading_deg)
            self.min_effective_spacing = params.get(
                "min_effective_spacing", min_effective_spacing
            )
            # self.spacing_deadband = params.get('spacing_deadband', spacing_deadband)
            # self.velocity_deadband = params.get('velocity_deadband', velocity_deadband)
            # self.throttle_smoothing = params.get('throttle_smoothing', throttle_smoothing)
        else:
            self.s0 = s0
            self.h = h
            self.K = K if K is not None else np.array([[0.2, 0.05]])
            self.acc_to_throttle_gain = acc_to_throttle_gain
            self.max_throttle = max_throttle
            self.alpha_filter = alpha_filter
            self.ki_velocity = ki_velocity
            self.spacing_mode = spacing_mode
            self.projection_heading_source = projection_heading_source
            self.blend_heading_deg = blend_heading_deg
            self.min_effective_spacing = min_effective_spacing
        self.spacing_deadband = spacing_deadband
        self.velocity_deadband = velocity_deadband
        self.throttle_smoothing = throttle_smoothing

        allowed_spacing_modes = {
            "euclidean",
            "projected",
            "blended",
            "path",
            "path_or_projected",
        }
        if self.spacing_mode not in allowed_spacing_modes:
            self.spacing_mode = "path_or_projected"

        if self.projection_heading_source not in {"leader", "follower", "average"}:
            self.projection_heading_source = "leader"

        self.blend_heading_rad = np.deg2rad(max(float(self.blend_heading_deg), 1e-3))

        # State for filtering
        self.prev_acc = 0.0

        # Velocity integral for additional stability (optional)
        self.velocity_integral = 0.0

        # Brake smoothing
        self.brake_smoothing = brake_smoothing
        self.prev_throttle = 0.0

        # Acceleration rate limiter to prevent sudden jumps
        self.max_acc_rate = max_acc_rate  # Maximum change in acceleration per second
        self.prev_acc_desired = 0.0

        # Spacing error integral for steady-state accuracy
        self.spacing_integral = 0.0
        self.ki_spacing = 0.01  # Small integral gain for spacing

    def _select_reference_heading(
        self, follower_theta: float, leader_theta: float
    ) -> float:
        """Select heading used for longitudinal projection."""
        if self.projection_heading_source == "follower":
            return follower_theta
        if self.projection_heading_source == "average":
            return follower_theta + 0.5 * _wrap_to_pi(leader_theta - follower_theta)
        return leader_theta

    def _compute_spacing(
        self,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
    ) -> Dict[str, float]:
        """
        Compute effective spacing using selected metric.

        Returns dict with keys:
            spacing, spacing_euclidean, spacing_projected, lateral_offset, reference_heading
        """
        x = float(follower_state["x"])
        y = float(follower_state["y"])
        follower_theta = float(follower_state.get("theta", 0.0))

        x_j = float(leader_state["x"])
        y_j = float(leader_state["y"])
        leader_theta = float(leader_state.get("theta", 0.0))

        dx = x_j - x
        dy = y_j - y
        spacing_euclidean = float(np.hypot(dx, dy))

        reference_heading = self._select_reference_heading(follower_theta, leader_theta)
        spacing_projected = float(
            dx * np.cos(reference_heading) + dy * np.sin(reference_heading)
        )
        lateral_offset = float(
            -dx * np.sin(reference_heading) + dy * np.cos(reference_heading)
        )

        path_gap = follower_state.get("along_track_gap", None)
        if path_gap is None:
            path_gap = leader_state.get("along_track_gap", None)

        if path_gap is not None and self.spacing_mode in {"path", "path_or_projected"}:
            spacing = float(path_gap)
            source = "path"
        elif self.spacing_mode == "euclidean":
            spacing = spacing_euclidean
            source = "euclidean"
        elif self.spacing_mode == "blended":
            if follower_state.get("is_turning", False):
                blend = 1.0
            else:
                heading_diff = abs(_wrap_to_pi(leader_theta - follower_theta))
                blend = float(np.clip(heading_diff / self.blend_heading_rad, 0.0, 1.0))
            spacing = (1.0 - blend) * spacing_euclidean + blend * spacing_projected
            source = "blended"
        else:
            # 'projected' and fallback for missing 'path' data
            spacing = spacing_projected
            source = "projected"

        spacing = max(float(spacing), float(self.min_effective_spacing))

        return {
            "spacing": spacing,
            "spacing_euclidean": spacing_euclidean,
            "spacing_projected": spacing_projected,
            "lateral_offset": lateral_offset,
            "reference_heading": reference_heading,
            "source": source,
        }

    def _compute_velocity_error(
        self,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
        spacing_info: Dict[str, float],
    ) -> float:
        """Compute relative velocity consistent with spacing metric."""
        follower_v = float(follower_state["velocity"])
        leader_v = float(leader_state["velocity"])

        # For projected/path metrics, compare longitudinal velocity components.
        if spacing_info["source"] in {"path", "projected", "blended"}:
            reference_heading = spacing_info["reference_heading"]
            follower_theta = float(follower_state.get("theta", 0.0))
            leader_theta = float(leader_state.get("theta", 0.0))

            follower_v_long = follower_v * np.cos(follower_theta - reference_heading)
            leader_v_long = leader_v * np.cos(leader_theta - reference_heading)
            return float(leader_v_long - follower_v_long)

        return float(leader_v - follower_v)

    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        """
        Compute throttle using CACC law with enhanced smoothing

        CACC computes desired acceleration based on:
        - Spacing error: (actual_spacing - desired_spacing)
        - Velocity error: (leader_velocity - follower_velocity)

        Enhanced with:
        - Deadband zones to prevent oscillations
        - Progressive control zones (comfort/normal/emergency)
        - Exponential throttle smoothing
        - Velocity-dependent gain scheduling
        """
        if leader_state is None:
            # No leader data - gradually reduce throttle to zero
            target_throttle = 0.0
            smoothed_throttle = (
                self.throttle_smoothing * self.prev_throttle
                + (1 - self.throttle_smoothing) * target_throttle
            )
            self.prev_throttle = smoothed_throttle
            return smoothed_throttle

        # Extract states
        v = follower_state["velocity"]

        # Calculate effective spacing (path/projection-aware)
        spacing_info = self._compute_spacing(follower_state, leader_state)
        spacing = spacing_info["spacing"]

        # Calculate desired spacing (CTH policy: s_d = s0 + h*v)
        spacing_target = self.s0 + self.h * v

        # Calculate errors
        spacing_error = spacing - spacing_target
        velocity_error = self._compute_velocity_error(
            follower_state, leader_state, spacing_info
        )

        # Apply deadband to spacing error to prevent small oscillations
        if abs(spacing_error) < self.spacing_deadband:
            spacing_error = 0.0
        else:
            # Remove deadband offset for smoother transition
            spacing_error = (
                spacing_error - np.sign(spacing_error) * self.spacing_deadband
            )

        # Apply deadband to velocity error
        if abs(velocity_error) < self.velocity_deadband:
            velocity_error = 0.0
        else:
            velocity_error = (
                velocity_error - np.sign(velocity_error) * self.velocity_deadband
            )

        # Update spacing integral (with anti-windup)
        self.spacing_integral += spacing_error * dt
        self.spacing_integral = np.clip(self.spacing_integral, -2.0, 2.0)

        # CACC control law with integral term
        error_vector = np.array([spacing_error, velocity_error])
        acc_desired = (self.K @ error_vector)[
            0
        ] + self.ki_spacing * self.spacing_integral

        # # Velocity-dependent gain scheduling (reduce gains at low speeds for stability)
        # speed_factor = np.clip(v / 0.5, 0.3, 1.0)  # Scale down gains below 0.5 m/s
        # acc_desired *= speed_factor

        # Apply acceleration rate limiter to prevent sudden jumps
        max_acc_change = self.max_acc_rate * dt
        acc_diff = acc_desired - self.prev_acc_desired

        # Limit the change in acceleration
        if abs(acc_diff) > max_acc_change:
            acc_desired = self.prev_acc_desired + np.sign(acc_diff) * max_acc_change

        # Store filtered acceleration for next iteration
        self.prev_acc_desired = acc_desired

        # Convert acceleration to throttle (simplified linear mapping)
        throttle_raw = acc_desired * self.acc_to_throttle_gain

        # Define control zones based on spacing error
        spacing_error_abs = abs(spacing - spacing_target)

        if spacing_error_abs < 0.3:  # Comfort zone - very gentle control
            throttle_raw *= 0.5
        elif spacing_error_abs < 0.8:  # Normal zone - standard control
            throttle_raw *= 0.8
        # else: Emergency zone - full control authority

        # Clamp to limits
        throttle_raw = np.clip(throttle_raw, -self.max_throttle, self.max_throttle)

        # Special handling for braking (negative throttle)
        if throttle_raw < 0:
            # More aggressive smoothing for braking to prevent jerky stops
            smoothing_factor = 0.85
            throttle_raw = (
                smoothing_factor * self.prev_throttle
                + (1 - smoothing_factor) * throttle_raw
            )
            throttle_raw = max(throttle_raw, 0.0)  # No negative throttle output

        # Apply exponential smoothing to final throttle command
        throttle = (
            self.throttle_smoothing * self.prev_throttle
            + (1 - self.throttle_smoothing) * throttle_raw
        )

        # Ensure throttle is non-negative
        throttle = max(throttle, 0.0)

        # Store for next iteration
        self.prev_throttle = throttle

        return throttle

    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically"""
        for param, value in params.items():
            if hasattr(self, param):
                if param == "K":
                    setattr(self, param, np.array(value))
                else:
                    setattr(self, param, value)

        # update derived parameter
        if "blend_heading_deg" in params:
            self.blend_heading_rad = np.deg2rad(
                max(float(self.blend_heading_deg), 1e-3)
            )

        if self.logger:
            self.logger.logger.info(
                f"[CACCLongitudinalController] Updated params: {params}"
            )

    def reset(self):
        """Reset controller state"""
        self.prev_acc = 0.0
        self.velocity_integral = 0.0
        self.prev_throttle = 0.0
        self.prev_acc_desired = 0.0
        self.spacing_integral = 0.0


class IDMControl:
    def __init__(self, alpha=1.0, beta=1.5, v0=1.0, delta=4, T=0.4, s0=7, logger=None):
        """Simple IDM controller initialization."""
        self.alpha = alpha
        self.beta = beta
        self.v0 = v0
        self.delta = delta
        self.T = T
        self.s0 = s0
        self.logger = logger

    def compute_idm_acceleration(self, follower_state, leader_state):
        """Compute IDM acceleration."""
        x, y, theta, v = follower_state
        x_j, y_j, theta_j, v_j = leader_state

        # Calculate spacing and relative velocity
        s = math.hypot(x_j - x, y_j - y)
        delta_v = v - v_j

        # IDM formula
        s_star = (
            self.s0 + self.T * v + (v * delta_v) / (2 * (self.alpha * self.beta) ** 0.5)
        )
        acc = self.alpha * (1 - (v / self.v0) ** self.delta - (s_star / s) ** 2)

        if self.logger:
            self.logger.debug(
                f"IDM: spacing={s:.2f}m, s_star={s_star:.2f}m, acc={acc:.2f}m/s²"
            )

        return acc


class SA_ACCController(LongitudinalControllerBase):
    """
    Safety-Aware Adaptive Cruise Control (SA-ACC) Control Law.

    Implements the control law from SA_ACC_UIO.m
    usync(i) = h*k1*k2*mu(i) - k2*xf(2,i) - (k1+h*k1*k2)*(ksi_hat(3,i)) + ...
               k3/h*(xp(2,i)-xf(2,i)) + k2/h*(psi(1,i)) - k2/h*(Li-li) - (k1+h*k1*k2)*sat(ksi_hat(4,i),20,-20);

    Expects 'ksi_hat_3' and 'ksi_hat_4' in follower_state.
    """

    def __init__(
        self,
        tau=0.4,
        h=0.5,
        k1=-0.8,
        k2=2.5,
        li=5,
        Li=8,
        acc_to_throttle_gain=0.65,
        max_throttle=0.3,
        config=None,
        logger=None,
        **kwargs,
    ):
        """
        Initialize SA-ACC Controller

        Args:
            tau: Driveline time constant
            h: Time headway
            k1, k2: Control gains from SA_ACC logic
            li: Length of vehicle i (or min spacing)
            Li: Desired Spacing constant
            acc_to_throttle_gain: Conversion gain
            max_throttle: Max throttle
        """
        self.logger = logger

        # Use config if provided
        if config and hasattr(config, "get_longitudinal_params"):
            params = config.get_longitudinal_params("sa_acc")
            self.tau = params.get("tau", tau)
            self.h = params.get("h", h)
            self.k1 = params.get("k1", k1)
            self.k2 = params.get("k2", k2)
            self.li = params.get("li", li)
            self.Li = params.get("Li", Li)
            self.acc_to_throttle_gain = params.get(
                "acc_to_throttle_gain", acc_to_throttle_gain
            )
            self.max_throttle = params.get("max_throttle", max_throttle)
        else:
            self.tau = tau
            self.h = h
            self.k1 = k1
            self.k2 = k2
            self.li = li
            self.Li = Li
            self.acc_to_throttle_gain = acc_to_throttle_gain
            self.max_throttle = max_throttle

        # Derived parameter k3
        self.k3 = 1 - self.h * self.k1 * self.k2

    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:

        if leader_state is None:
            return 0.0

        xf_vel = follower_state["velocity"]

        xp_vel = leader_state["velocity"]
        # mu is transmitted acceleration (potentially corrupted).
        # Assume leader_state['acceleration'] contains this.
        mu = leader_state.get("acceleration", 0.0)

        # Measurements / Estimates
        dx = leader_state["x"] - follower_state["x"]
        dy = leader_state["y"] - follower_state["y"]
        spacing = np.hypot(dx, dy)

        # psi_1 = spacing - li
        psi_1 = spacing - self.li

        # Estimates from UIO (passed in follower_state)
        # Default to 0 if not present (reduces to nominal controller?)
        ksi_hat_3 = follower_state.get("ksi_hat_3", 0.0)
        ksi_hat_4 = follower_state.get("ksi_hat_4", 0.0)

        # Control Law
        def sat(val, limit):
            return np.clip(val, -limit, limit)

        term1 = self.h * self.k1 * self.k2 * mu
        term2 = -self.k2 * xf_vel
        term3 = -(self.k1 + self.h * self.k1 * self.k2) * ksi_hat_3
        term4 = (self.k3 / self.h) * (xp_vel - xf_vel)  # psi_2
        term5 = (self.k2 / self.h) * psi_1
        term6 = -(self.k2 / self.h) * (self.Li - self.li)
        term7 = -(self.k1 + self.h * self.k1 * self.k2) * sat(ksi_hat_4, 20)

        usync = term1 + term2 + term3 + term4 + term5 + term6 + term7

        throttle = usync * self.acc_to_throttle_gain
        throttle = np.clip(throttle, -self.max_throttle, self.max_throttle)

        if throttle < 0:
            throttle = max(throttle, 0.0)

        return float(throttle)

    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically"""
        for param, value in params.items():
            if hasattr(self, param):
                setattr(self, param, value)

        # Derived parameter k3
        self.k3 = 1 - self.h * self.k1 * self.k2

        if self.logger:
            self.logger.logger.info(f"[SA_ACCController] Updated params: {params}")

    def reset(self):
        pass


class FixConstantController(LongitudinalControllerBase):
    def __init__(self, throttle: float, config=None, logger=None):
        self.throttle = throttle
        self.logger = logger

    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        return self.throttle

    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically"""
        if "throttle" in params:
            self.throttle = params["throttle"]

        if self.logger:
            self.logger.logger.info(f"[FixConstantController] Updated params: {params}")

    def reset(self):
        pass


class ControllerFactory:
    """Factory to create longitudinal controllers by name"""

    CONTROLLER_TYPES = {
        "pid": PIDVelocityController,
        "cacc": CACCLongitudinalController,
        "sa_acc": SA_ACCController,
        "fix": FixConstantController,
        # MPC will be added dynamically when mpc_wrappers is imported
    }

    @staticmethod
    def create(
        controller_type: str, params: Dict[str, Any] = None, logger=None, config=None
    ):
        """
        Create a longitudinal controller

        Args:
            controller_type: One of 'pid', 'cacc', 'sa_acc', 'fix', 'mpc'
            params: Dictionary of controller-specific parameters
            logger: Logger instance

        Returns:
            Longitudinal controller instance
        """
        params = params or {}

        # Lazy import MPC if requested but not yet registered
        if controller_type == "mpc" and "mpc" not in ControllerFactory.CONTROLLER_TYPES:
            try:
                from .mpc_wrappers import MPCLongitudinalWrapper

                ControllerFactory.CONTROLLER_TYPES["mpc"] = MPCLongitudinalWrapper
            except ImportError:
                raise ValueError(
                    "MPC controller requires casadi. Install with: pip install casadi"
                )

        if controller_type not in ControllerFactory.CONTROLLER_TYPES:
            raise ValueError(
                f"Unknown controller type: {controller_type}. "
                f"Available: {list(ControllerFactory.CONTROLLER_TYPES.keys())}"
            )

        controller_class = ControllerFactory.CONTROLLER_TYPES[controller_type]
        return controller_class(logger=logger, config=config, **params)
