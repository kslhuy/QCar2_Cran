"""
Longitudinal Controllers for Vehicle Following

Provides different longitudinal control strategies with a common interface.
Easy to switch between different controllers.
"""

import math
import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, Tuple


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
        ff_gain=(0.1 / 0.62),
        use_affine_feedforward=False,
        ff_speed_slope=6.63,
        ff_speed_intercept=-0.31,
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
            ff_gain: Feedforward gain converting target speed to base command
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
            self.ff_gain = params.get("ff_gain", ff_gain)
            self.use_affine_feedforward = params.get(
                "use_affine_feedforward", use_affine_feedforward
            )
            self.ff_speed_slope = params.get("ff_speed_slope", ff_speed_slope)
            self.ff_speed_intercept = params.get(
                "ff_speed_intercept", ff_speed_intercept
            )
            self.max_throttle = params.get("max_throttle", max_throttle)
            self.min_throttle = params.get("min_throttle", min_throttle)
            self.ei_max = params.get("ei_max", ei_max)
            self.vehicle_type = kwargs.get("vehicle_type", params.get("vehicle_type", "QCar"))
        else:
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.ff_gain = ff_gain
            self.use_affine_feedforward = use_affine_feedforward
            self.ff_speed_slope = ff_speed_slope
            self.ff_speed_intercept = ff_speed_intercept
            self.max_throttle = max_throttle
            self.min_throttle = min_throttle
            self.ei_max = ei_max
            self.vehicle_type = kwargs.get("vehicle_type", "QCar")

        # Command velocity used for Limo rate limiting
        self.cmd_v = 0.0

        # print(f"[PIDVelocityController] Initialized with kp={self.kp}, ki={self.ki}, kd={self.kd}, max_throttle={self.max_throttle}")
        self.ei = 0.0  # Integral error
        self.prev_e = None  # Previous error for derivative
        self.last_error = 0.0

    def _compute_feedforward(self, v_ref: float) -> float:
        """Convert desired speed into a baseline throttle command."""
        if (
            self.use_affine_feedforward
            and abs(float(self.ff_speed_slope)) > 1e-9
            and v_ref > 0.0
        ):
            return max(
                0.0, (float(v_ref) - float(self.ff_speed_intercept)) / float(self.ff_speed_slope)
            )
        return v_ref * self.ff_gain

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
        if getattr(self, "vehicle_type", "QCar") == "Limo":
            # For Limo, limit the acceleration and return a bounded velocity command
            max_acc = 1.0  # max acceleration m/s^2 for smooth velocity changes
            dv = max_acc * dt
            if v_ref > self.cmd_v:
                self.cmd_v = min(self.cmd_v + dv, v_ref)
            else:
                self.cmd_v = max(self.cmd_v - dv, v_ref)
                
            # If stopping completely, ensure v_cmd drops to 0 smoothly
            if v_ref < 0.05 and self.cmd_v < 0.1:
                self.cmd_v = 0.0
                
            self.cmd_v = np.clip(self.cmd_v, 0.0, 3.0) # Absolute max speed limit for Limo increased to 3.0 to allow Gear scaling
            
            self.last_error = e
            return self.cmd_v
        
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
        u_ff = self._compute_feedforward(v_ref)

        # PID control + Feedforward
        u = u_ff + self.kp * e + self.ki * self.ei + self.kd * de


        # Normal QCar logic
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
        self.cmd_v = 0.0


class QCar2SpeedController(LongitudinalControllerBase):
    """
    Incremental speed controller inspired by qcar2_hardware.cpp.

    The hardware node integrates a PD correction into the outgoing PWM command
    and compensates that correction by battery voltage. This controller keeps
    the same structure while operating on the Python longitudinal interface.
    """

    def __init__(
        self,
        kp=20.0,
        kd=0.1,
        km=0.0047,
        use_affine_feedforward=False,
        ff_speed_slope=6.63,
        ff_speed_intercept=-0.31,
        max_throttle=0.3,
        min_forward_throttle=0.01,
        min_reverse_throttle=0.01,
        nominal_battery_voltage=12.0,
        min_battery_voltage=1.0,
        stop_speed_threshold=1e-3,
        config=None,
        logger=None,
        **kwargs,
    ):
        self.logger = logger

        if config and hasattr(config, "get_longitudinal_params"):
            params = config.get_longitudinal_params("qcar2_speed")
            self.kp = params.get("kp", kp)
            self.kd = params.get("kd", kd)
            self.km = params.get("km", km)
            self.use_affine_feedforward = params.get(
                "use_affine_feedforward", use_affine_feedforward
            )
            self.ff_speed_slope = params.get("ff_speed_slope", ff_speed_slope)
            self.ff_speed_intercept = params.get(
                "ff_speed_intercept", ff_speed_intercept
            )
            self.max_throttle = params.get("max_throttle", max_throttle)
            self.min_forward_throttle = params.get(
                "min_forward_throttle", min_forward_throttle
            )
            self.min_reverse_throttle = params.get(
                "min_reverse_throttle", min_reverse_throttle
            )
            self.nominal_battery_voltage = params.get(
                "nominal_battery_voltage", nominal_battery_voltage
            )
            self.min_battery_voltage = params.get(
                "min_battery_voltage", min_battery_voltage
            )
            self.stop_speed_threshold = params.get(
                "stop_speed_threshold", stop_speed_threshold
            )
        else:
            self.kp = kp
            self.kd = kd
            self.km = km
            self.use_affine_feedforward = use_affine_feedforward
            self.ff_speed_slope = ff_speed_slope
            self.ff_speed_intercept = ff_speed_intercept
            self.max_throttle = max_throttle
            self.min_forward_throttle = min_forward_throttle
            self.min_reverse_throttle = min_reverse_throttle
            self.nominal_battery_voltage = nominal_battery_voltage
            self.min_battery_voltage = min_battery_voltage
            self.stop_speed_threshold = stop_speed_threshold

        self.prior_speed_error = 0.0
        self.motor_speed_cmd = 0.0
        self.last_error = 0.0

    def _compute_feedforward(self, target_speed: float) -> float:
        """Inverse of the steady-state speed map v = a*u + b."""
        if (
            not self.use_affine_feedforward
            or abs(float(self.ff_speed_slope)) <= 1e-9
            or abs(target_speed) <= self.stop_speed_threshold
        ):
            return 0.0

        throttle_ff = (
            float(target_speed) - float(self.ff_speed_intercept)
        ) / float(self.ff_speed_slope)
        return float(np.clip(throttle_ff, -self.max_throttle, self.max_throttle))

    def _effective_battery_voltage(self, follower_state: Dict[str, float]) -> float:
        """Use measured battery voltage when available, otherwise a nominal value."""
        battery_voltage = float(
            follower_state.get("battery_voltage", self.nominal_battery_voltage)
        )
        if battery_voltage < self.min_battery_voltage:
            return float(self.nominal_battery_voltage)
        return battery_voltage

    def _compute_incremental_command(
        self,
        measured_speed: float,
        target_speed: float,
        dt: float,
        battery_voltage: float,
    ) -> float:
        """Replicate the incremental PD logic used in the ROS 2 hardware node."""
        if abs(target_speed) <= self.stop_speed_threshold:
            self.motor_speed_cmd = 0.0
            self.prior_speed_error = 0.0
            self.last_error = 0.0
            return 0.0

        base_command = self._compute_feedforward(target_speed)
        speed_error = target_speed - measured_speed
        self.last_error = speed_error

        if dt <= 1e-6:
            derivative = 0.0
        else:
            derivative = (speed_error - self.prior_speed_error) / dt

        correction = (
            (speed_error * self.kp + derivative * self.kd) * self.km / battery_voltage
        )
        self.motor_speed_cmd = base_command + correction
        self.prior_speed_error = speed_error

        self.motor_speed_cmd = float(
            np.clip(self.motor_speed_cmd, -self.max_throttle, self.max_throttle)
        )

        if (
            0.0 <= self.motor_speed_cmd < self.min_forward_throttle
            and target_speed > 0.0
        ):
            self.motor_speed_cmd += self.min_forward_throttle
        elif (
            -self.min_reverse_throttle < self.motor_speed_cmd < 0.0
            and target_speed < 0.0
        ):
            self.motor_speed_cmd -= self.min_reverse_throttle

        self.motor_speed_cmd = float(
            np.clip(self.motor_speed_cmd, -self.max_throttle, self.max_throttle)
        )
        return self.motor_speed_cmd

    def update(
        self,
        v: float,
        v_ref: float,
        dt: float,
        battery_voltage: Optional[float] = None,
    ) -> float:
        """Path-following helper matching the existing speed-controller API."""
        follower_state = {"velocity": v}
        if battery_voltage is not None:
            follower_state["battery_voltage"] = battery_voltage
        return self.compute_throttle(
            {**follower_state, "target_velocity": v_ref},
            leader_state=None,
            dt=dt,
        )

    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        measured_speed = float(
            follower_state.get("motor_tach", follower_state.get("velocity", 0.0))
        )
        target_speed = float(follower_state.get("target_velocity", 0.0))
        battery_voltage = self._effective_battery_voltage(follower_state)
        return self._compute_incremental_command(
            measured_speed=measured_speed,
            target_speed=target_speed,
            dt=dt,
            battery_voltage=battery_voltage,
        )

    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically."""
        for param, value in params.items():
            if hasattr(self, param):
                setattr(self, param, value)

        if self.logger:
            self.logger.logger.info(f"[QCar2SpeedController] Updated params: {params}")

    def reset(self):
        """Reset controller state."""
        self.prior_speed_error = 0.0
        self.motor_speed_cmd = 0.0
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
        brake_smoothing=0.5,
        max_acc_rate=2.0,
        spacing_deadband=0.2,
        velocity_deadband=0.05,
        throttle_smoothing=0.7,
        spacing_mode="path_or_projected",
        projection_heading_source="leader",
        blend_heading_deg=20.0,
        min_effective_spacing=0.0,
        use_feedforward=False,
        leader_acceleration_weight: float = 0.0,
        limo_max_speed: float = 0.8,
        limo_max_accel: float = 0.4,
        limo_max_decel: float = 0.8,
        limo_leader_speed_margin: float = 0.12,
        limo_gap_closing_gain: float = 0.25,
        limo_close_gap_gain: float = 0.8,
        config=None,
        logger=None,
        **kwargs,
    ):
        """
        Initialize CACC longitudinal controller

        Args:
            s0: Minimum spacing (meters)
            h: Time headway (seconds)
            K: Control gains [spacing_gain, velocity_gain]
            acc_to_throttle_gain: Gain to convert acceleration to throttle
            max_throttle: Maximum throttle output
            brake_smoothing: Smoothing factor for negative throttle (0-1, higher = smoother)
            max_acc_rate: Maximum acceleration rate of change (m/s^3) to limit jerk
            spacing_deadband: Spacing error deadband to prevent oscillations (meters)
            velocity_deadband: Velocity error deadband to prevent oscillations (m/s)
            throttle_smoothing: Exponential smoothing factor for throttle (0-1, higher = smoother)
            spacing_mode: Gap metric ('euclidean', 'projected', 'blended', 'path', 'path_or_projected')
            projection_heading_source: Heading source for projection ('leader', 'follower', 'average')
            blend_heading_deg: Heading difference where blended mode becomes fully projected
            min_effective_spacing: Minimum spacing used by controller (can be <= 0 for signed spacing)
            use_feedforward: Boolean enabling velocity feedforward term (legacy)
            leader_acceleration_weight: Optional weight applied to acceleration
                difference (leader minus follower) and treated as an extra
                velocity-error term. Use this instead of feedforward; set to 0
                for traditional CACC behaviour.
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
            self.spacing_mode = params.get("spacing_mode", spacing_mode)
            self.projection_heading_source = params.get(
                "projection_heading_source", projection_heading_source
            )
            self.blend_heading_deg = params.get("blend_heading_deg", blend_heading_deg)
            self.min_effective_spacing = params.get(
                "min_effective_spacing", min_effective_spacing
            )
            self.spacing_deadband = params.get('spacing_deadband', spacing_deadband)
            self.velocity_deadband = params.get('velocity_deadband', velocity_deadband)
            self.throttle_smoothing = params.get('throttle_smoothing', throttle_smoothing)
            self.brake_smoothing = params.get('brake_smoothing', brake_smoothing)
            self.max_acc_rate = params.get('max_acc_rate', max_acc_rate)
            self.use_feedforward = params.get('use_feedforward', use_feedforward)
            self.leader_acceleration_weight = params.get(
                'leader_acceleration_weight',
                params.get('leader_acceleration_gain', leader_acceleration_weight),
            )
            self.limo_max_speed = params.get("limo_max_speed", limo_max_speed)
            self.limo_max_accel = params.get("limo_max_accel", limo_max_accel)
            self.limo_max_decel = params.get("limo_max_decel", limo_max_decel)
            self.limo_leader_speed_margin = params.get(
                "limo_leader_speed_margin", limo_leader_speed_margin
            )
            self.limo_gap_closing_gain = params.get(
                "limo_gap_closing_gain", limo_gap_closing_gain
            )
            self.limo_close_gap_gain = params.get(
                "limo_close_gap_gain", limo_close_gap_gain
            )
            self.vehicle_type = kwargs.get("vehicle_type", params.get('vehicle_type', 'QCar'))
        else:
            self.s0 = s0
            self.h = h
            self.K = K if K is not None else np.array([[0.2, 0.05]])
            self.acc_to_throttle_gain = acc_to_throttle_gain
            self.max_throttle = max_throttle
            self.spacing_mode = spacing_mode
            self.projection_heading_source = projection_heading_source
            self.blend_heading_deg = blend_heading_deg
            self.min_effective_spacing = min_effective_spacing
            self.spacing_deadband = spacing_deadband
            self.velocity_deadband = velocity_deadband
            self.throttle_smoothing = throttle_smoothing
            self.brake_smoothing = brake_smoothing
            self.max_acc_rate = max_acc_rate
            self.use_feedforward = use_feedforward
            # take argument value when config not used
            self.leader_acceleration_weight = leader_acceleration_weight
            self.limo_max_speed = limo_max_speed
            self.limo_max_accel = limo_max_accel
            self.limo_max_decel = limo_max_decel
            self.limo_leader_speed_margin = limo_leader_speed_margin
            self.limo_gap_closing_gain = limo_gap_closing_gain
            self.limo_close_gap_gain = limo_close_gap_gain
            self.vehicle_type = kwargs.get('vehicle_type', 'QCar')

        # Command velocity used for Limo rate limiting
        self.cmd_v = 0.0

        # Simple feedforward gain based on calibration
        self.ff_gain = 0.1 / 0.62  # u_ff = v_ref * ff_gain


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

        # Brake smoothing and acc rate are already set from config/kwargs
        self.prev_throttle = 0.0

        # Acceleration rate limiter to prevent sudden jumps
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

    def _smooth_to_stop(self) -> float:
        """Gradually reduce throttle to zero when no leader is present."""
        target_throttle = 0.0
        smoothed_throttle = (
            self.throttle_smoothing * self.prev_throttle
            + (1 - self.throttle_smoothing) * target_throttle
        )
        self.prev_throttle = smoothed_throttle
        return smoothed_throttle

    def _apply_deadband(self, value: float, deadband: float) -> float:
        """Apply deadband to a value to prevent micro-oscillations."""
        if abs(value) < deadband:
            return 0.0
        return value - np.sign(value) * deadband

    def _calculate_tracking_errors(self, follower_state: Dict[str, float], leader_state: Dict[str, float]) -> Tuple[float, float]:
        """Calculate spacing and velocity errors."""
        v = follower_state["velocity"]
        
        spacing_info = self._compute_spacing(follower_state, leader_state)
        spacing = spacing_info["spacing"]
        spacing_target = self.s0 + self.h * v
        
        spacing_error = spacing - spacing_target
        velocity_error = self._compute_velocity_error(follower_state, leader_state, spacing_info)

        # incorporate acceleration difference if weight configured
        if self.leader_acceleration_weight:
            leader_acc = leader_state.get("acceleration", 0.0)
            follower_acc = follower_state.get("acceleration", 0.0)
            velocity_error += self.leader_acceleration_weight * (leader_acc - follower_acc)
        
        spacing_error = self._apply_deadband(spacing_error, self.spacing_deadband)
        velocity_error = self._apply_deadband(velocity_error, self.velocity_deadband)
        
        return spacing_error, velocity_error

    def _compute_desired_acceleration(self, spacing_error: float, velocity_error: float, dt: float) -> float:
        """Compute desired acceleration using CACC control law with integral and rate limits."""
        # Update spacing integral (with anti-windup)
        self.spacing_integral += spacing_error * dt
        self.spacing_integral = np.clip(self.spacing_integral, -2.0, 2.0)

        # CACC control law with integral term
        error_vector = np.array([spacing_error, velocity_error])
        acc_desired = (self.K @ error_vector)[0] + self.ki_spacing * self.spacing_integral

        # Apply acceleration rate limiter to prevent sudden jumps
        max_acc_change = self.max_acc_rate * dt
        acc_diff = acc_desired - self.prev_acc_desired

        if abs(acc_diff) > max_acc_change:
            acc_desired = self.prev_acc_desired + np.sign(acc_diff) * max_acc_change

        self.prev_acc_desired = acc_desired
        return acc_desired

    def _compute_limo_speed_cap(
        self,
        spacing_error: float,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
    ) -> float:
        """
        Compute a conservative speed ceiling for the Limo follower.

        Poor localization can make the gap estimate jump. This speed cap keeps
        the follower close to the leader's speed unless the spacing error is
        clearly positive.
        """
        leader_v = max(0.0, float(leader_state.get("velocity", 0.0)))
        recovery_speed = self.limo_gap_closing_gain * max(spacing_error, 0.0)
        speed_cap = leader_v + self.limo_leader_speed_margin + recovery_speed

        if spacing_error < 0.0:
            speed_cap += self.limo_close_gap_gain * spacing_error

        return float(np.clip(speed_cap, 0.0, self.limo_max_speed))

    def _compute_limo_velocity_cmd(
        self,
        acc_desired: float,
        spacing_error: float,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
        dt: float,
    ) -> float:
        """Compute the velocity command for the Limo robot."""
        # Using our own velocity prevents relying on potentially attacked leader velocity data.
        # We integrate the desired acceleration from our current actual velocity.
        v = follower_state["velocity"]
        
        # If cmd_v is vastly different from actual v (e.g., CACC just engaged or we were blocked),
        # snap it to the current velocity so we don't get integral windup or cold starts.
        if abs(self.cmd_v - v) > 0.5:
            self.cmd_v = v

        # Limo needs a much gentler velocity profile than QCar effort control.
        acc_desired = float(
            np.clip(acc_desired, -self.limo_max_decel, self.limo_max_accel)
        )

        # Integrate acceleration to get the new velocity command
        self.cmd_v += acc_desired * dt
        
        # Stop condition: if trying to slow down, and both vehicles are nearly stopped
        if acc_desired < 0 and v < 0.05 and leader_state.get("velocity", 0.0) < 0.05:
            self.cmd_v = 0.0  # Force stop

        speed_cap = self._compute_limo_speed_cap(
            spacing_error, follower_state, leader_state
        )
        self.cmd_v = np.clip(self.cmd_v, 0.0, speed_cap)
        self.prev_throttle = self.cmd_v
        return float(self.cmd_v)

    def _compute_qcar_throttle(self, acc_desired: float, ff_throttle: float, dt: float) -> float:
        """Compute the throttle command for the QCar."""
        throttle_raw = (acc_desired * self.acc_to_throttle_gain) + ff_throttle
        throttle_raw = np.clip(throttle_raw, -self.max_throttle, self.max_throttle)

        # Special handling for braking (negative throttle)
        if throttle_raw < 0:
            # More aggressive smoothing for braking to prevent jerky stops
            throttle_raw = (
                self.brake_smoothing * self.prev_throttle
                + (1 - self.brake_smoothing) * throttle_raw
            )
            throttle_raw = max(throttle_raw, 0.0)  # No negative throttle output

        if self.throttle_smoothing > 0:
            throttle = (
                self.throttle_smoothing * self.prev_throttle
                + (1 - self.throttle_smoothing) * throttle_raw
            )
        else:
            throttle = throttle_raw

        throttle = max(throttle, 0.0)
        self.prev_throttle = throttle
        return float(throttle)

    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        """Clean, pipeline-based CACC execution."""
        
        # 1. Handle missing leader (safe stop)
        if leader_state is None:
            return self._smooth_to_stop()

        # 2. Calculate errors combining spacing and velocity constraints
        spacing_error, velocity_error = self._calculate_tracking_errors(follower_state, leader_state)

        # 3. Compute desired acceleration (CACC Control Law)
        acc_desired = self._compute_desired_acceleration(spacing_error, velocity_error, dt)

        # note: acceleration weight is handled inside _calculate_tracking_errors

        # 4. Generate vehicle-specific hardware commands
        if getattr(self, "vehicle_type", "QCar") == "Limo":
            return self._compute_limo_velocity_cmd(
                acc_desired, spacing_error, follower_state, leader_state, dt
            )
        else:
            # Inject velocity feedforward for QCar effort control
            if getattr(self, "use_feedforward", False):
                ff_throttle = leader_state["velocity"] * self.ff_gain
            else:
                ff_throttle = 0.0
                
            return self._compute_qcar_throttle(acc_desired, ff_throttle, dt)

    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically"""
        for param, value in params.items():
            if hasattr(self, param):
                if param == "K":
                    setattr(self, param, np.array(value))
                elif param in ("leader_acceleration_gain", "leader_acceleration_weight"):
                    # Ensure numeric conversion for control calculations.
                    self.leader_acceleration_weight = float(value)
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
        self.cmd_v = 0.0


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
        "qcar2_speed": QCar2SpeedController,
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
            controller_type: One of 'pid', 'qcar2_speed', 'cacc', 'sa_acc', 'fix', 'mpc'
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
