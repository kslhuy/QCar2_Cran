"""Deterministic kinematic-bicycle IO adapter for headless integration tests."""

from __future__ import annotations

import math

import numpy as np

from core.vehicle_types import ControlInput
from utils.io.io_base import IOBase


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class IOVirtual(IOBase):
    """Virtual vehicle with longitudinal dynamics and kinematic lateral motion.

    The state is ``[x_m, y_m, yaw_rad, velocity_mps]``. Throttle and steering
    are backend-normalized actuator requests with configurable first-order
    actuator lag. ``read_to_cache()`` advances one fixed simulation sample,
    then publishes tachometer, IMU, and intermittent GPS measurements.
    """

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None, **overrides) -> None:
        effective_config = dict(config)
        effective_config.update(overrides)
        super().__init__(effective_config, vehicle_id, logger)

        self.dt = float(self._config.get("dt", 0.02))
        self.wheelbase = float(self._config.get("wheelbase", 0.3))
        self.mass = float(self._config.get("mass", 1.6))
        self.drive_accel_gain = float(self._config.get("drive_accel_gain", 3.2))
        self.throttle_time_constant = max(1e-6, float(self._config.get("throttle_time_constant", 0.25)))
        self.steering_time_constant = max(1e-6, float(self._config.get("steering_time_constant", 0.12)))
        self.rolling_resistance_accel = float(self._config.get("rolling_resistance_accel", 0.025))
        self.viscous_drag_coeff = float(self._config.get("viscous_drag_coeff", 0.10))
        self.air_drag_coeff = float(self._config.get("air_drag_coeff", 0.04))
        self.gps_period_steps = max(1, int(self._config.get("gps_period_steps", 5)))
        self.rng = np.random.default_rng(self._config.get("seed", 7))
        self.motor_tach_noise_std = float(self._config.get("motor_tach_noise_std", 0.01))
        self.gyro_noise_std = float(self._config.get("gyro_noise_std", 0.01))
        self.accel_noise_std = float(self._config.get("accel_noise_std", 0.40))
        self.gps_xy_noise_std = float(self._config.get("gps_xy_noise_std", 0.025))
        self.gps_theta_noise_std = float(self._config.get("gps_theta_noise_std", 0.015))

        self.x = float(self._config.get("initial_x", 0.0))
        self.y = float(self._config.get("initial_y", 0.0))
        self.theta = float(self._config.get("initial_theta", 0.0))
        self.velocity = max(0.0, float(self._config.get("initial_velocity", 0.0)))
        self.acceleration = 0.0
        self.motor_command = 0.0
        self.steering_actual = 0.0
        self.time = 0.0
        self.step_count = 0
        self._last_written = ControlInput(0.0, 0.0, 0.0, "initial")

    def read_to_cache(self) -> None:
        """Advance exactly one deterministic simulation sample and publish it."""
        self._step_dynamics()
        with self._cache_lock:
            self._poll_sensors()
            self._poll_gps()

    def _step_dynamics(self) -> None:
        command = self._last_written
        state = np.array(
            [self.x, self.y, self.theta, self.velocity, self.motor_command, self.steering_actual],
            dtype=float,
        )
        next_state = self._rk4_step(state, float(command.throttle), float(command.steering), self.dt)

        self.x = float(next_state[0])
        self.y = float(next_state[1])
        self.theta = _wrap_to_pi(float(next_state[2]))
        self.velocity = max(0.0, float(next_state[3]))
        self.motor_command = float(next_state[4])
        self.steering_actual = float(next_state[5])
        self.acceleration = self._longitudinal_acceleration(self.velocity, self.motor_command)
        self.time += self.dt
        self.step_count += 1

    def _rk4_step(self, state: np.ndarray, throttle_request: float, steering_request: float, dt: float) -> np.ndarray:
        k1 = self._continuous_dynamics(state, throttle_request, steering_request)
        k2 = self._continuous_dynamics(state + 0.5 * dt * k1, throttle_request, steering_request)
        k3 = self._continuous_dynamics(state + 0.5 * dt * k2, throttle_request, steering_request)
        k4 = self._continuous_dynamics(state + dt * k3, throttle_request, steering_request)
        return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _continuous_dynamics(self, state: np.ndarray, throttle_request: float, steering_request: float) -> np.ndarray:
        x, y, theta, velocity, motor_command, steering_actual = state
        velocity = max(0.0, float(velocity))
        acceleration = self._longitudinal_acceleration(velocity, motor_command)
        yaw_rate = velocity * math.tan(float(steering_actual)) / self.wheelbase
        motor_dot = (throttle_request - float(motor_command)) / self.throttle_time_constant
        steering_dot = (steering_request - float(steering_actual)) / self.steering_time_constant
        return np.array(
            [
                velocity * math.cos(float(theta)),
                velocity * math.sin(float(theta)),
                yaw_rate,
                acceleration,
                motor_dot,
                steering_dot,
            ],
            dtype=float,
        )

    def _longitudinal_acceleration(self, velocity: float, motor_command: float) -> float:
        velocity = max(0.0, float(velocity))
        drive_force = self.mass * self.drive_accel_gain * float(motor_command)
        rolling_force = self.mass * self.rolling_resistance_accel * math.tanh(velocity / 0.03)
        viscous_force = self.mass * self.viscous_drag_coeff * velocity
        air_force = self.mass * self.air_drag_coeff * velocity * abs(velocity)
        return (drive_force - rolling_force - viscous_force - air_force) / self.mass

    def _poll_sensors(self) -> None:
        gyro_z = self.velocity * math.tan(self.steering_actual) / self.wheelbase
        self._sensor_data_cache.motor_tach = float(self.velocity + self.rng.normal(0.0, self.motor_tach_noise_std))
        self._sensor_data_cache.gyro_z = float(gyro_z + self.rng.normal(0.0, self.gyro_noise_std))
        self._sensor_data_cache.accelerometer = np.array(
            [
                self.acceleration + self.rng.normal(0.0, self.accel_noise_std),
                self.rng.normal(0.0, self.accel_noise_std),
                9.81 + self.rng.normal(0.0, self.accel_noise_std),
            ],
            dtype=float,
        )
        self._sensor_data_cache.sensor_timestamp = float(self.time)

    def _poll_gps(self) -> None:
        gps_valid = self.step_count % self.gps_period_steps == 0
        self._sensor_data_cache.gps_valid = gps_valid
        if gps_valid:
            self._sensor_data_cache.gps_position = np.array(
                [
                    self.x + self.rng.normal(0.0, self.gps_xy_noise_std),
                    self.y + self.rng.normal(0.0, self.gps_xy_noise_std),
                    _wrap_to_pi(self.theta + self.rng.normal(0.0, self.gps_theta_noise_std)),
                ],
                dtype=float,
            )
            self._sensor_data_cache.gps_timestamp = float(self.time)

    def _hardware_write(self, throttle: float, steering: float) -> None:
        self._last_written = ControlInput(
            throttle=float(throttle),
            steering=float(steering),
            target_velocity=float(self._command_cache.target_velocity),
            source=self._command_cache.source,
        )

    def true_state(self) -> tuple[float, float, float, float, float]:
        """Return ``x, y, yaw, velocity, acceleration`` without measurement noise."""
        return self.x, self.y, self.theta, self.velocity, self.acceleration

    def actuator_state(self) -> tuple[float, float]:
        """Return the internal throttle and steering actuator states."""
        return self.motor_command, self.steering_actual
