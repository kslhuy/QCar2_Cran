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
        limo_max_accel=1.0,
        limo_max_decel=1.0,
        limo_max_speed=3.0,
        limo_stop_speed_threshold=0.05,
        limo_stop_command_threshold=0.1,
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
            self.limo_max_accel = params.get("limo_max_accel", limo_max_accel)
            self.limo_max_decel = params.get("limo_max_decel", limo_max_decel)
            self.limo_max_speed = params.get("limo_max_speed", limo_max_speed)
            self.limo_stop_speed_threshold = params.get(
                "limo_stop_speed_threshold", limo_stop_speed_threshold
            )
            self.limo_stop_command_threshold = params.get(
                "limo_stop_command_threshold", limo_stop_command_threshold
            )
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
            self.limo_max_accel = limo_max_accel
            self.limo_max_decel = limo_max_decel
            self.limo_max_speed = limo_max_speed
            self.limo_stop_speed_threshold = limo_stop_speed_threshold
            self.limo_stop_command_threshold = limo_stop_command_threshold

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
        e = v_ref - v

        if getattr(self, "vehicle_type", "QCar") == "Limo":
            # For Limo, limit the acceleration and return a bounded velocity command
            if v_ref > self.cmd_v:
                max_acc = max(float(self.limo_max_accel), 0.0)
                self.cmd_v = min(self.cmd_v + max_acc * dt, v_ref)
            else:
                max_decel = max(float(self.limo_max_decel), 0.0)
                self.cmd_v = max(self.cmd_v - max_decel * dt, v_ref)
                
            # If stopping completely, ensure v_cmd drops to 0 smoothly
            if (
                v_ref < float(self.limo_stop_speed_threshold)
                and self.cmd_v < float(self.limo_stop_command_threshold)
            ):
                self.cmd_v = 0.0
                
            self.cmd_v = np.clip(
                self.cmd_v, 0.0, max(float(self.limo_max_speed), 0.0)
            )
            
            self.last_error = e
            self.prev_e = e
            return self.cmd_v

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
        ff_gain=0.1 / 0.62,
        leader_acceleration_weight: float = 0.0,
        target_velocity_weight: float = 0.0,
        target_velocity_gap_window: float = 0.15,
        target_velocity_turn_scale: float = 0.35,
        close_spacing_deadband: float = 0.03,
        multi_predecessor: Optional[Dict[str, Any]] = None,
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
            ff_gain: Feedforward gain converting leader speed to base throttle
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
            self.ff_gain = float(params.get("ff_gain", ff_gain))
            self.leader_acceleration_weight = params.get(
                'leader_acceleration_weight',
                params.get('leader_acceleration_gain', leader_acceleration_weight),
            )
            self.target_velocity_weight = params.get(
                "target_velocity_weight", target_velocity_weight
            )
            self.target_velocity_gap_window = params.get(
                "target_velocity_gap_window", target_velocity_gap_window
            )
            self.target_velocity_turn_scale = params.get(
                "target_velocity_turn_scale", target_velocity_turn_scale
            )
            self.close_spacing_deadband = params.get(
                "close_spacing_deadband", close_spacing_deadband
            )
            multi_predecessor_cfg = params.get("multi_predecessor", {})
            if isinstance(multi_predecessor, dict):
                multi_predecessor_cfg = {
                    **(multi_predecessor_cfg if isinstance(multi_predecessor_cfg, dict) else {}),
                    **multi_predecessor,
                }
            self.multi_predecessor_config = self._normalize_multi_predecessor_config(
                multi_predecessor_cfg
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
            self.ff_gain = float(ff_gain)
            # take argument value when config not used
            self.leader_acceleration_weight = leader_acceleration_weight
            self.target_velocity_weight = target_velocity_weight
            self.target_velocity_gap_window = target_velocity_gap_window
            self.target_velocity_turn_scale = target_velocity_turn_scale
            self.close_spacing_deadband = close_spacing_deadband
            self.multi_predecessor_config = self._normalize_multi_predecessor_config(
                multi_predecessor
            )
            self.limo_max_speed = limo_max_speed
            self.limo_max_accel = limo_max_accel
            self.limo_max_decel = limo_max_decel
            self.limo_leader_speed_margin = limo_leader_speed_margin
            self.limo_gap_closing_gain = limo_gap_closing_gain
            self.limo_close_gap_gain = limo_close_gap_gain
            self.vehicle_type = kwargs.get('vehicle_type', 'QCar')

        # Command velocity used for Limo rate limiting
        self.cmd_v = 0.0

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
        self.default_max_reverse_throttle = 0.08
        self.default_max_reverse_speed = 0.12
        self.reverse_spacing_deadband = max(float(self.spacing_deadband), 0.05)
        self.reverse_velocity_deadband = max(float(self.velocity_deadband), 0.03)
        self.reverse_throttle_smoothing = max(float(self.brake_smoothing), 0.75)

    @staticmethod
    def _as_bool(value: Any, default: bool = False) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return default if value is None else bool(value)

    @staticmethod
    def _as_float(value: Any, default: float) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return default
        return parsed if np.isfinite(parsed) else default

    @staticmethod
    def _as_int(value: Any, default: int) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _normalize_multi_predecessor_config(
        self, cfg: Optional[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Normalize optional multi-predecessor CACC feedforward settings."""
        if not isinstance(cfg, dict):
            cfg = {}

        order_decay = float(
            np.clip(self._as_float(cfg.get("order_decay", 0.60), 0.60), 0.0, 1.0)
        )
        trust_power = max(self._as_float(cfg.get("trust_power", 1.0), 1.0), 0.0)
        return {
            "enabled": self._as_bool(cfg.get("enabled", False), False),
            "spacing_weight": self._as_float(
                cfg.get("spacing_weight", 0.20), 0.20
            ),
            "max_spacing_contribution": max(
                self._as_float(cfg.get("max_spacing_contribution", 0.35), 0.35),
                0.0,
            ),
            "max_predecessors": max(
                self._as_int(cfg.get("max_predecessors", 3), 3), 0
            ),
            "require_trust": self._as_bool(cfg.get("require_trust", True), True),
            "min_trust": float(
                np.clip(self._as_float(cfg.get("min_trust", 0.50), 0.50), 0.0, 1.0)
            ),
            "missing_trust_weight": float(
                np.clip(
                    self._as_float(cfg.get("missing_trust_weight", 0.0), 0.0),
                    0.0,
                    1.0,
                )
            ),
            "trust_power": trust_power,
            "order_decay": order_decay,
            "distance_decay": max(
                self._as_float(cfg.get("distance_decay", 0.0), 0.0), 0.0
            ),
            "velocity_weight": self._as_float(
                cfg.get("velocity_weight", 0.25), 0.25
            ),
            "acceleration_weight": self._as_float(
                cfg.get("acceleration_weight", 0.25), 0.25
            ),
            "max_velocity_contribution": max(
                self._as_float(cfg.get("max_velocity_contribution", 0.25), 0.25),
                0.0,
            ),
            "max_acceleration_contribution": max(
                self._as_float(
                    cfg.get("max_acceleration_contribution", 0.25), 0.25
                ),
                0.0,
            ),
            "safe_gap_margin": self._as_float(
                cfg.get("safe_gap_margin", 0.05), 0.05
            ),
            "allow_positive_when_close": self._as_bool(
                cfg.get("allow_positive_when_close", False), False
            ),
        }

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

    def _state_longitudinal_velocity(
        self, state: Dict[str, Any], reference_heading: float
    ) -> float:
        velocity = float(state.get("velocity", state.get("v", 0.0)))
        theta = float(state.get("theta", reference_heading))
        return float(velocity * np.cos(theta - reference_heading))

    def _trust_weight_for_predecessor(self, state: Dict[str, Any]) -> float:
        cfg = self.multi_predecessor_config
        trust_value = state.get("leader_trust")
        if trust_value is None:
            trust_value = state.get("generalized_trust", state.get("direct_trust"))

        if trust_value is None:
            return float(cfg["missing_trust_weight"])

        trust = self._as_float(trust_value, float("nan"))
        if not np.isfinite(trust):
            return float(cfg["missing_trust_weight"])

        trust = float(np.clip(trust, 0.0, 1.0))
        if cfg["require_trust"] and trust < cfg["min_trust"]:
            return 0.0
        return float(trust ** cfg["trust_power"])

    def _compute_multi_predecessor_feedforward(
        self,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
        spacing_info: Dict[str, float],
        spacing_error: float,
    ) -> Tuple[float, float, float]:
        """Compute bounded correction terms from trusted vehicles ahead of the host."""
        cfg = self.multi_predecessor_config
        if not cfg.get("enabled", False) or follower_state.get("reverse_follow_active", False):
            return 0.0, 0.0, 0.0

        predecessor_states = follower_state.get("multi_predecessor_states", [])
        if not isinstance(predecessor_states, (list, tuple)):
            return 0.0, 0.0, 0.0

        max_predecessors = int(cfg.get("max_predecessors", 0))
        if max_predecessors <= 0:
            return 0.0, 0.0, 0.0

        reference_heading = float(spacing_info["reference_heading"])
        direct_velocity = self._state_longitudinal_velocity(
            leader_state, reference_heading
        )
        direct_acceleration = float(leader_state.get("acceleration", 0.0))

        weighted_spacing = 0.0
        weighted_velocity = 0.0
        weighted_acceleration = 0.0
        weight_sum = 0.0
        used_count = 0

        for pred_state in predecessor_states:
            if used_count >= max_predecessors:
                break
            if not isinstance(pred_state, dict):
                continue

            trust_weight = self._trust_weight_for_predecessor(pred_state)
            if trust_weight <= 0.0:
                continue

            order_index = max(self._as_int(pred_state.get("order_index", 1), 1), 1)
            order_weight = float(cfg["order_decay"] ** max(order_index - 1, 0))

            distance_weight = 1.0
            distance_decay = float(cfg.get("distance_decay", 0.0))
            if distance_decay > 0.0 and pred_state.get("distance_ahead") is not None:
                distance_ahead = max(
                    self._as_float(pred_state.get("distance_ahead"), 0.0), 0.0
                )
                distance_weight = float(np.exp(-distance_decay * distance_ahead))

            weight = trust_weight * order_weight * distance_weight
            if weight <= 0.0:
                continue

            pred_gap = pred_state.get("distance_ahead")
            if pred_gap is None:
                pred_gap = (
                    (float(pred_state.get("x", 0.0)) - float(follower_state.get("x", 0.0)))
                    * np.cos(reference_heading)
                    + (float(pred_state.get("y", 0.0)) - float(follower_state.get("y", 0.0)))
                    * np.sin(reference_heading)
                )
            pred_gap = max(float(pred_gap), 0.0)
            desired_gap = float(order_index) * (
                self.s0 + self.h * max(float(follower_state.get("velocity", 0.0)), 0.0)
            )
            weighted_spacing += weight * (pred_gap - desired_gap)

            pred_velocity = self._state_longitudinal_velocity(
                pred_state, reference_heading
            )
            pred_acceleration = float(pred_state.get("acceleration", 0.0))
            weighted_velocity += weight * (pred_velocity - direct_velocity)
            weighted_acceleration += weight * (pred_acceleration - direct_acceleration)
            weight_sum += weight
            used_count += 1

        follower_state["multi_predecessor_count"] = used_count
        follower_state["multi_predecessor_weight_sum"] = float(weight_sum)

        if weight_sum <= 1e-9:
            follower_state["multi_predecessor_spacing_term"] = 0.0
            follower_state["multi_predecessor_velocity_term"] = 0.0
            follower_state["multi_predecessor_acceleration_term"] = 0.0
            return 0.0, 0.0, 0.0

        spacing_term = float(cfg["spacing_weight"] * weighted_spacing / weight_sum)
        velocity_term = float(cfg["velocity_weight"] * weighted_velocity / weight_sum)
        acceleration_term = float(
            cfg["acceleration_weight"] * weighted_acceleration / weight_sum
        )

        max_spacing = float(cfg["max_spacing_contribution"])
        max_velocity = float(cfg["max_velocity_contribution"])
        max_acceleration = float(cfg["max_acceleration_contribution"])
        spacing_term = float(np.clip(spacing_term, -max_spacing, max_spacing))
        velocity_term = float(np.clip(velocity_term, -max_velocity, max_velocity))
        acceleration_term = float(
            np.clip(acceleration_term, -max_acceleration, max_acceleration)
        )

        if (
            spacing_error <= float(cfg["safe_gap_margin"])
            and not cfg["allow_positive_when_close"]
        ):
            spacing_term = min(spacing_term, 0.0)
            velocity_term = min(velocity_term, 0.0)
            acceleration_term = min(acceleration_term, 0.0)

        follower_state["multi_predecessor_spacing_term"] = spacing_term
        follower_state["multi_predecessor_velocity_term"] = velocity_term
        follower_state["multi_predecessor_acceleration_term"] = acceleration_term
        return spacing_term, velocity_term, acceleration_term

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

    def _get_reverse_control_config(
        self, follower_state: Optional[Dict[str, float]]
    ) -> Dict[str, float]:
        """Read reverse-follow limits supplied by FOLLOWING_LEADER state."""
        reverse_cfg = {}
        if isinstance(follower_state, dict):
            reverse_cfg = follower_state.get("reverse_follow_config", {}) or {}

        return {
            "max_reverse_throttle": float(
                reverse_cfg.get(
                    "max_reverse_throttle", self.default_max_reverse_throttle
                )
            ),
            "max_reverse_speed": float(
                reverse_cfg.get("max_reverse_speed", self.default_max_reverse_speed)
            ),
            "spacing_deadband": float(
                reverse_cfg.get(
                    "spacing_deadband", self.reverse_spacing_deadband
                )
            ),
            "velocity_deadband": float(
                reverse_cfg.get(
                    "velocity_deadband", self.reverse_velocity_deadband
                )
            ),
            "throttle_smoothing": float(
                reverse_cfg.get(
                    "throttle_smoothing", self.reverse_throttle_smoothing
                )
            ),
        }

    def _calculate_tracking_errors(self, follower_state: Dict[str, float], leader_state: Dict[str, float]) -> Tuple[float, float]:
        """Calculate spacing and velocity errors."""
        v = follower_state["velocity"]
        reverse_active = bool(follower_state.get("reverse_follow_active", False))
        reverse_cfg = self._get_reverse_control_config(follower_state)
        
        spacing_info = self._compute_spacing(follower_state, leader_state)
        spacing = spacing_info["spacing"]
        spacing_target = self.s0 + self.h * (abs(v) if reverse_active else v)
        
        spacing_error = spacing - spacing_target
        velocity_error = self._compute_velocity_error(follower_state, leader_state, spacing_info)

        if self.leader_acceleration_weight and self.vehicle_type != "Limo":
            leader_acc = leader_state.get("acceleration", 0.0)
            follower_acc = follower_state.get("acceleration", 0.0)
            velocity_error += self.leader_acceleration_weight * (leader_acc - follower_acc)

        multi_spacing, multi_velocity, multi_acceleration = self._compute_multi_predecessor_feedforward(
            follower_state, leader_state, spacing_info, spacing_error
        )
        spacing_error += multi_spacing
        velocity_error += multi_velocity + multi_acceleration

        target_velocity = follower_state.get("target_velocity", None)
        if (
            self.target_velocity_weight > 0.0
            and target_velocity is not None
            and not reverse_active
        ):
            free_flow_error = max(float(target_velocity) - float(v), 0.0)
            if free_flow_error > 0.0:
                gap_window = max(float(self.target_velocity_gap_window), 1e-3)
                gap_factor = float(np.clip(spacing_error / gap_window, 0.0, 1.0))
                turn_scale = (
                    float(self.target_velocity_turn_scale)
                    if follower_state.get("is_turning", False)
                    else 1.0
                )
                velocity_error += (
                    self.target_velocity_weight
                    * gap_factor
                    * turn_scale
                    * free_flow_error
                )
        
        if reverse_active:
            spacing_deadband = reverse_cfg["spacing_deadband"]
            velocity_deadband = reverse_cfg["velocity_deadband"]
        else:
            spacing_deadband = self.spacing_deadband
            velocity_deadband = self.velocity_deadband

        if not reverse_active and spacing_error < 0.0:
            spacing_deadband = min(
                float(spacing_deadband),
                max(float(self.close_spacing_deadband), 0.0),
            )

        spacing_error = self._apply_deadband(spacing_error, spacing_deadband)
        velocity_error = self._apply_deadband(velocity_error, velocity_deadband)
        
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

        if spacing_error < 0.0:
            speed_cap = leader_v + self.limo_close_gap_gain * spacing_error
        else:
            recovery_speed = self.limo_gap_closing_gain * spacing_error
            speed_cap = leader_v + self.limo_leader_speed_margin + recovery_speed

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

    def _compute_qcar_throttle(
        self,
        acc_desired: float,
        ff_throttle: float,
        dt: float,
        follower_state: Optional[Dict[str, float]] = None,
    ) -> float:
        """Compute the throttle command for the QCar."""
        reverse_active = bool(
            isinstance(follower_state, dict)
            and follower_state.get("reverse_follow_active", False)
        )
        if reverse_active:
            reverse_cfg = self._get_reverse_control_config(follower_state)
            max_reverse_throttle = max(0.0, reverse_cfg["max_reverse_throttle"])
            max_reverse_speed = max(0.0, reverse_cfg["max_reverse_speed"])
            throttle_raw = (acc_desired * self.acc_to_throttle_gain) + ff_throttle
            throttle_raw = float(np.clip(throttle_raw, -max_reverse_throttle, 0.0))

            current_speed_mag = abs(float(follower_state.get("velocity", 0.0)))
            if current_speed_mag >= max_reverse_speed and throttle_raw < 0.0:
                throttle_raw = 0.0

            if throttle_raw < 0.0:
                prev_reverse = min(float(self.prev_throttle), 0.0)
                smoothing = float(np.clip(reverse_cfg["throttle_smoothing"], 0.0, 0.99))
                throttle = (
                    smoothing * prev_reverse
                    + (1 - smoothing) * throttle_raw
                )
                throttle = float(np.clip(throttle, -max_reverse_throttle, 0.0))
            else:
                throttle = 0.0

            self.prev_throttle = throttle
            return throttle

        throttle_raw = (acc_desired * self.acc_to_throttle_gain) + ff_throttle
        throttle_raw = np.clip(throttle_raw, -self.max_throttle, self.max_throttle)

        # QCar forward-following uses zero effort to slow down; when the CACC
        # asks for deceleration, decay toward zero instead of holding a stale
        # positive throttle. This avoids creeping into the leader at stop.
        throttle_target = max(float(throttle_raw), 0.0)

        if throttle_target < self.prev_throttle:
            brake_alpha = float(np.clip(self.brake_smoothing, 0.0, 0.9))
            throttle_raw = (
                brake_alpha * self.prev_throttle
                + (1 - brake_alpha) * throttle_target
            )
        else:
            throttle_raw = throttle_target

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
                
            return self._compute_qcar_throttle(
                acc_desired, ff_throttle, dt, follower_state=follower_state
            )

    def update_params(self, params: Dict[str, Any]):
        """Update controller parameters dynamically"""
        for param, value in params.items():
            if hasattr(self, param):
                if param == "K":
                    setattr(self, param, np.array(value))
                elif param in ("leader_acceleration_gain", "leader_acceleration_weight"):
                    # Ensure numeric conversion for control calculations.
                    self.leader_acceleration_weight = float(value)
                elif param == "multi_predecessor":
                    self.multi_predecessor_config = (
                        self._normalize_multi_predecessor_config(value)
                    )
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


class SensorAdaptiveCruiseController(LongitudinalControllerBase):
    """
    Local-sensor ACC branch for leader following.

    Uses locally measured gap (for example YOLO `car_dist`) to generate a
    target speed, then relies on the existing PID velocity loop to convert
    that target into the platform-specific throttle or velocity command.
    """

    def __init__(
        self,
        desired_distance=0.35,
        time_headway=0.0,
        distance_gain=1.0,
        min_target_velocity=0.0,
        max_target_velocity=0.8,
        stop_distance=0.20,
        max_distance=2.0,
        config=None,
        logger=None,
        **kwargs,
    ):
        self.logger = logger

        if config and hasattr(config, "get_leader_sensor_acc_config"):
            params = config.get_leader_sensor_acc_config()
            self.desired_distance = float(
                params.get("desired_distance", desired_distance)
            )
            self.time_headway = float(params.get("time_headway", time_headway))
            self.distance_gain = float(params.get("distance_gain", distance_gain))
            self.min_target_velocity = float(
                params.get("min_target_velocity", min_target_velocity)
            )
            self.max_target_velocity = float(
                params.get("max_target_velocity", max_target_velocity)
            )
            self.stop_distance = float(params.get("stop_distance", stop_distance))
            self.max_distance = float(params.get("max_distance", max_distance))
        else:
            self.desired_distance = float(desired_distance)
            self.time_headway = float(time_headway)
            self.distance_gain = float(distance_gain)
            self.min_target_velocity = float(min_target_velocity)
            self.max_target_velocity = float(max_target_velocity)
            self.stop_distance = float(stop_distance)
            self.max_distance = float(max_distance)

        self.vehicle_type = kwargs.get("vehicle_type", "QCar")
        self.speed_controller = PIDVelocityController(
            config=config,
            logger=logger,
            vehicle_type=self.vehicle_type,
        )
        self.last_distance = None
        self.last_distance_error = 0.0
        self.last_target_velocity = 0.0

    def _compute_target_velocity(self, follower_state: Dict[str, float]) -> float:
        measured_distance = follower_state.get(
            "sensor_leader_distance_filtered",
            follower_state.get("sensor_leader_distance"),
        )
        base_velocity = max(
            0.0, float(follower_state.get("target_velocity", self.max_target_velocity))
        )
        current_velocity = float(follower_state.get("velocity", 0.0))

        if measured_distance is None:
            self.last_distance = None
            self.last_distance_error = 0.0
            self.last_target_velocity = 0.0
            return 0.0

        measured_distance = float(measured_distance)
        if (
            not np.isfinite(measured_distance)
            or measured_distance <= 0.0
            or measured_distance > self.max_distance
        ):
            self.last_distance = None
            self.last_distance_error = 0.0
            self.last_target_velocity = 0.0
            return 0.0

        desired_gap = self.desired_distance + self.time_headway * max(current_velocity, 0.0)
        distance_error = measured_distance - desired_gap

        if measured_distance <= self.stop_distance:
            target_velocity = 0.0
        else:
            free_flow_velocity = min(base_velocity, self.max_target_velocity)
            target_velocity = free_flow_velocity + self.distance_gain * distance_error
            target_velocity = float(
                np.clip(
                    target_velocity,
                    self.min_target_velocity,
                    self.max_target_velocity,
                )
            )

        self.last_distance = measured_distance
        self.last_distance_error = float(distance_error)
        self.last_target_velocity = float(target_velocity)
        return self.last_target_velocity

    def compute_throttle(
        self,
        follower_state: Dict[str, float],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        target_velocity = self._compute_target_velocity(follower_state)
        acc_state = dict(follower_state)
        acc_state["target_velocity"] = target_velocity
        return self.speed_controller.compute_throttle(
            acc_state, leader_state=None, dt=dt
        )

    def update_params(self, params: Dict[str, Any]):
        """Update local ACC parameters dynamically."""
        for param, value in params.items():
            if hasattr(self, param):
                setattr(self, param, float(value))

        if self.logger:
            self.logger.logger.info(
                f"[SensorAdaptiveCruiseController] Updated params: {params}"
            )

    def reset(self):
        """Reset controller state."""
        self.last_distance = None
        self.last_distance_error = 0.0
        self.last_target_velocity = 0.0
        self.speed_controller.reset()


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
        "multi_predecessor_cacc": CACCLongitudinalController,
        "sensor_acc": SensorAdaptiveCruiseController,
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
