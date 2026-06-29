"""
Integration test for the minimal control loop.

This test combines:
- vehicle IO: a virtual kinematic vehicle using BaseVehicleIO
- observer: EKF
- path planner: StaticWaypointPlanner
- controller: SimplePathController

Run from the qcar_refactor directory:
    python -m unittest test.test_integration_control_loop
"""

import csv
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.Types import ControlCommand, SensorData
from utils.IO.BaseIO import BaseVehicleIO
from utils.Control.Observer.ObserverEKF import EKF
from utils.Control.PathPlanner import StaticWaypointPlanner
from utils.Control.Controller import SimplePathController


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class VirtualKinematicVehicleIO(BaseVehicleIO):
    """
    Deterministic virtual vehicle for integration testing.

    State: [x, y, theta, v]
    Input: ControlCommand(throttle, steering)

    The throttle is treated as a normalized acceleration command for this test.
    """

    def __init__(
        self,
        config: dict,
        dt: float = 0.02,
        wheelbase: float = 0.3,
        mass: float = 1.6,
        drive_accel_gain: float = 3.2,
        throttle_time_constant: float = 0.25,
        steering_time_constant: float = 0.12,
        rolling_resistance_accel: float = 0.025,
        viscous_drag_coeff: float = 0.10,
        air_drag_coeff: float = 0.04,
        gps_period_steps: int = 5,
        seed: int = 7,
        motor_tach_noise_std: float = 0.01,
        gyro_noise_std: float = 0.01,
        accel_noise_std: float = 0.40,
        gps_xy_noise_std: float = 0.025,
        gps_theta_noise_std: float = 0.015,
    ) -> None:
        super().__init__(config)
        self.dt = float(dt)
        self.wheelbase = float(wheelbase)
        self.mass = float(mass)
        self.drive_accel_gain = float(drive_accel_gain)
        self.throttle_time_constant = max(1e-6, float(throttle_time_constant))
        self.steering_time_constant = max(1e-6, float(steering_time_constant))
        self.rolling_resistance_accel = float(rolling_resistance_accel)
        self.viscous_drag_coeff = float(viscous_drag_coeff)
        self.air_drag_coeff = float(air_drag_coeff)
        self.gps_period_steps = int(gps_period_steps)
        self.rng = np.random.default_rng(seed)
        self.motor_tach_noise_std = float(motor_tach_noise_std)
        self.gyro_noise_std = float(gyro_noise_std)
        self.accel_noise_std = float(accel_noise_std)
        self.gps_xy_noise_std = float(gps_xy_noise_std)
        self.gps_theta_noise_std = float(gps_theta_noise_std)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.motor_command = 0.0
        self.steering_actual = 0.0
        self.time = 0.0
        self.step_count = 0
        self._last_written = ControlCommand(0.0, 0.0, 0.0, "initial")

    def read_to_cache(self) -> None:
        self._step_dynamics()
        with self._cache_lock:
            self._poll_sensors()
            self._poll_gps()

    def _step_dynamics(self) -> None:
        command = self._last_written
        throttle_request = float(command.throttle)
        steering_request = float(command.steering)

        state = np.array(
            [
                self.x,
                self.y,
                self.theta,
                self.velocity,
                self.motor_command,
                self.steering_actual,
            ],
            dtype=float,
        )

        next_state = self._rk4_step(
            state,
            throttle_request=throttle_request,
            steering_request=steering_request,
            dt=self.dt,
        )

        self.x = float(next_state[0])
        self.y = float(next_state[1])
        self.theta = _wrap_to_pi(float(next_state[2]))
        self.velocity = max(0.0, float(next_state[3]))
        self.motor_command = float(next_state[4])
        self.steering_actual = float(next_state[5])
        self.acceleration = self._longitudinal_acceleration(
            self.velocity,
            self.motor_command,
        )
        self.time += self.dt
        self.step_count += 1

    def _rk4_step(
        self,
        state: np.ndarray,
        throttle_request: float,
        steering_request: float,
        dt: float,
    ) -> np.ndarray:
        """Integrate the continuous vehicle model over one sample using RK4."""
        k1 = self._continuous_dynamics(state, throttle_request, steering_request)
        k2 = self._continuous_dynamics(
            state + 0.5 * dt * k1,
            throttle_request,
            steering_request,
        )
        k3 = self._continuous_dynamics(
            state + 0.5 * dt * k2,
            throttle_request,
            steering_request,
        )
        k4 = self._continuous_dynamics(
            state + dt * k3,
            throttle_request,
            steering_request,
        )
        return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _continuous_dynamics(
        self,
        state: np.ndarray,
        throttle_request: float,
        steering_request: float,
    ) -> np.ndarray:
        x, y, theta, velocity, motor_command, steering_actual = state
        velocity = max(0.0, float(velocity))

        acceleration = self._longitudinal_acceleration(velocity, motor_command)
        yaw_rate = (
            velocity * math.tan(float(steering_actual)) / self.wheelbase
            if abs(self.wheelbase) > 1e-9
            else 0.0
        )

        motor_dot = (float(throttle_request) - float(motor_command)) / self.throttle_time_constant
        steering_dot = (float(steering_request) - float(steering_actual)) / self.steering_time_constant

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
        """Continuous longitudinal acceleration with smooth resistance terms."""
        velocity = max(0.0, float(velocity))
        drive_force = self.mass * self.drive_accel_gain * float(motor_command)
        rolling_direction = math.tanh(velocity / 0.03)
        rolling_friction_force = self.mass * self.rolling_resistance_accel * rolling_direction
        viscous_drag_force = self.mass * self.viscous_drag_coeff * velocity
        air_drag_force = self.mass * self.air_drag_coeff * velocity * abs(velocity)
        return (
            drive_force
            - rolling_friction_force
            - viscous_drag_force
            - air_drag_force
        ) / self.mass

    def _poll_sensors(self) -> None:
        gyro_z = (
            self.velocity * math.tan(self.steering_actual) / self.wheelbase
            if abs(self.wheelbase) > 1e-9
            else 0.0
        )
        measured_velocity = self.velocity + self.rng.normal(0.0, self.motor_tach_noise_std)
        measured_gyro = gyro_z + self.rng.normal(0.0, self.gyro_noise_std)
        measured_accel_x = self.acceleration + self.rng.normal(0.0, self.accel_noise_std)
        measured_accel_y = self.rng.normal(0.0, self.accel_noise_std)
        measured_accel_z = 9.81 + self.rng.normal(0.0, self.accel_noise_std)

        self._sensor_data_cache.motor_tach = float(measured_velocity)
        self._sensor_data_cache.gyro_z = float(measured_gyro)
        self._sensor_data_cache.accelerometer = np.array(
            [measured_accel_x, measured_accel_y, measured_accel_z],
            dtype=float,
        )
        self._sensor_data_cache.sensor_timestamp = float(self.time)

    def _poll_gps(self) -> None:
        gps_valid = (self.step_count % self.gps_period_steps) == 0
        self._sensor_data_cache.gps_valid = bool(gps_valid)
        if gps_valid:
            noisy_theta = _wrap_to_pi(
                self.theta + self.rng.normal(0.0, self.gps_theta_noise_std)
            )
            self._sensor_data_cache.gps_position = np.array(
                [
                    self.x + self.rng.normal(0.0, self.gps_xy_noise_std),
                    self.y + self.rng.normal(0.0, self.gps_xy_noise_std),
                    noisy_theta,
                ],
                dtype=float,
            )
            self._sensor_data_cache.gps_timestamp = float(self.time)

    def _hardware_write(self, throttle: float, steering: float) -> None:
        self._last_written = ControlCommand(
            throttle=float(throttle),
            steering=float(steering),
            target_velocity=float(self._command_cache.target_velocity),
            source=self._command_cache.source,
        )

    def stop(self) -> None:
        self.write(ControlCommand(0.0, 0.0, 0.0, "virtual_stop"))

    def true_state(self) -> tuple:
        return self.x, self.y, self.theta, self.velocity, self.acceleration

    def actuator_state(self) -> tuple:
        return self.motor_command, self.steering_actual


def _build_racetrack_lap() -> list:
    """Build one oval racetrack lap with two straights and two half-circle turns."""
    straight_length = 3.2
    turn_radius = 0.85
    straight_points = 38
    turn_points = 34

    path = []

    # Bottom straight: start at origin, heading east.
    for x in np.linspace(0.0, straight_length, straight_points, endpoint=False):
        path.append((float(x), 0.0, 0.0))

    # Right half-circle, turning left from east to west.
    right_center = (straight_length, turn_radius)
    for angle in np.linspace(-math.pi / 2.0, math.pi / 2.0, turn_points, endpoint=False):
        x = right_center[0] + turn_radius * math.cos(angle)
        y = right_center[1] + turn_radius * math.sin(angle)
        theta = _wrap_to_pi(angle + math.pi / 2.0)
        path.append((float(x), float(y), theta))

    # Top straight: heading west.
    for x in np.linspace(straight_length, 0.0, straight_points, endpoint=False):
        path.append((float(x), 2.0 * turn_radius, math.pi))

    # Left half-circle, turning left from west to east.
    left_center = (0.0, turn_radius)
    for angle in np.linspace(math.pi / 2.0, 3.0 * math.pi / 2.0, turn_points):
        x = left_center[0] + turn_radius * math.cos(angle)
        y = left_center[1] + turn_radius * math.sin(angle)
        theta = _wrap_to_pi(angle + math.pi / 2.0)
        path.append((float(x), float(y), theta))

    return path


def _build_path(laps: int = 3) -> list:
    """Build a multi-lap racetrack path for closed-loop stability testing."""
    one_lap = _build_racetrack_lap()
    path = []
    for lap_index in range(int(laps)):
        if lap_index == 0:
            path.extend(one_lap)
        else:
            path.extend(one_lap[1:])
    return path


class TestMinimalControlLoopIntegration(unittest.TestCase):
    def test_virtual_vehicle_follows_path_and_saves_plot(self):
        config = {
            "write": {"max_throttle": 0.10, "max_steering": 0.48},
            "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
        }
        dt = 0.02
        vehicle_io = VirtualKinematicVehicleIO(
            config=config,
            dt=dt,
            gps_period_steps=5,
            accel_noise_std=0.40,
            gps_xy_noise_std=0.025,
            gps_theta_noise_std=0.015,
        )
        observer = EKF(wheelbase=vehicle_io.wheelbase)
        planner = StaticWaypointPlanner(
            path_source=_build_path(laps=3),
            target_velocity=0.50,
            lookahead_distance=0.32,
            finish_tolerance=0.20,
            max_search_ahead=28,
        )
        controller = SimplePathController(
            kp_velocity=0.35,
            ki_velocity=0.03,
            kd_velocity=0.0,
            feedforward_gain=0.03,
            steering_gain=1.8,
            max_throttle=0.10,
            min_throttle=-0.10,
            max_steering=0.48,
        )

        observer.start(initial_pose=[0.0, 0.0, 0.0])
        command = ControlCommand(0.0, 0.0, 0.0, "initial")
        rows = []

        max_steps = 4200
        for _ in range(max_steps):
            vehicle_io.write(command)
            vehicle_io.read_to_cache()
            sensor_data = vehicle_io.read()
            estimate = observer.update(sensor_data, dt=dt, last_command=command)
            target = planner.update(estimate)
            command = controller.compute(estimate, target, dt=dt)

            tx, ty, ttheta, tv, ta = vehicle_io.true_state()
            motor_command, steering_actual = vehicle_io.actuator_state()
            rows.append(
                {
                    "time": sensor_data.sensor_timestamp,
                    "true_x": tx,
                    "true_y": ty,
                    "true_theta": ttheta,
                    "true_velocity": tv,
                    "true_acceleration": ta,
                    "est_x": estimate.x,
                    "est_y": estimate.y,
                    "est_theta": estimate.theta,
                    "est_velocity": estimate.velocity,
                    "target_x": target.target_x,
                    "target_y": target.target_y,
                    "target_velocity": target.target_velocity,
                    "following_error_x": target.target_x - tx,
                    "following_error_y": target.target_y - ty,
                    "estimation_error_x": estimate.x - tx,
                    "estimation_error_y": estimate.y - ty,
                    "throttle": command.throttle,
                    "steering": command.steering,
                    "motor_command": motor_command,
                    "steering_actual": steering_actual,
                    "gps_valid": float(sensor_data.gps_valid),
                    "planner_finished": float(target.is_finished),
                }
            )
            if target.is_finished:
                break

        vehicle_io.stop()
        observer.stop()

        self.assertGreater(len(rows), 400)
        self.assertTrue(planner.is_finished(), "virtual vehicle did not finish path")

        final = rows[-1]
        final_error = math.hypot(
            final["true_x"] - planner.waypoints[-1, 0],
            final["true_y"] - planner.waypoints[-1, 1],
        )
        self.assertLess(final_error, 0.45)

        max_estimation_error = max(
            math.hypot(row["true_x"] - row["est_x"], row["true_y"] - row["est_y"])
            for row in rows
        )
        self.assertLess(max_estimation_error, 1.0)

        self._save_artifacts(rows, planner.waypoints)

    def _save_artifacts(self, rows: list, waypoints: np.ndarray) -> None:
        output_dir = os.path.join(os.path.dirname(__file__), "artifacts")
        os.makedirs(output_dir, exist_ok=True)

        csv_path = os.path.join(output_dir, "integration_control_loop.csv")
        with open(csv_path, "w", newline="") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"[CSV] saved {csv_path} ({len(rows)} rows)")

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.skipTest(f"matplotlib unavailable for plot artifact: {exc}")

        t = np.array([row["time"] for row in rows], dtype=float)
        t -= t[0]
        true_xy = np.array([[row["true_x"], row["true_y"]] for row in rows], dtype=float)
        est_xy = np.array([[row["est_x"], row["est_y"]] for row in rows], dtype=float)
        target_xy = np.array([[row["target_x"], row["target_y"]] for row in rows], dtype=float)
        velocity = np.array(
            [[row["true_velocity"], row["est_velocity"], row["target_velocity"]] for row in rows],
            dtype=float,
        )
        command = np.array(
            [
                [
                    row["throttle"],
                    row["steering"],
                    row["motor_command"],
                    row["steering_actual"],
                ]
                for row in rows
            ],
            dtype=float,
        )
        pos_error = np.linalg.norm(true_xy - est_xy, axis=1)

        following_error = np.array(
            [[row["following_error_x"], row["following_error_y"]] for row in rows],
            dtype=float,
        )
        estimation_error = np.array(
            [[row["estimation_error_x"], row["estimation_error_y"]] for row in rows],
            dtype=float,
        )

        fig = plt.figure(figsize=(15, 11), constrained_layout=True)
        grid = fig.add_gridspec(4, 2)
        ax_path = fig.add_subplot(grid[:, 0])
        ax_vel = fig.add_subplot(grid[0, 1])
        ax_cmd = fig.add_subplot(grid[1, 1])
        ax_follow = fig.add_subplot(grid[2, 1])
        ax_err = fig.add_subplot(grid[3, 1])

        self._plot_racetrack(ax_path, waypoints)
        ax_path.plot(true_xy[:, 0], true_xy[:, 1], color="tab:blue", linewidth=1.6, label="virtual vehicle")
        ax_path.plot(est_xy[:, 0], est_xy[:, 1], color="tab:orange", linewidth=1.2, alpha=0.85, label="EKF estimate")
        ax_path.scatter(target_xy[::25, 0], target_xy[::25, 1], s=12, color="tab:green", label="planner targets")
        ax_path.scatter(true_xy[0, 0], true_xy[0, 1], s=36, color="tab:green", marker="o", label="start")
        ax_path.scatter(true_xy[-1, 0], true_xy[-1, 1], s=42, color="tab:red", marker="x", label="finish")
        ax_path.set_title("Racetrack Control Loop")
        ax_path.set_xlabel("x [m]")
        ax_path.set_ylabel("y [m]")
        ax_path.axis("equal")
        ax_path.grid(True, alpha=0.25)
        ax_path.legend(loc="best", fontsize=8)

        ax_vel.plot(t, velocity[:, 0], linewidth=1.2, label="true")
        ax_vel.plot(t, velocity[:, 1], linewidth=1.0, label="estimated")
        ax_vel.plot(t, velocity[:, 2], linewidth=1.0, linestyle="--", label="target")
        ax_vel.set_title("Velocity Tracking")
        ax_vel.set_ylabel("velocity [m/s]")
        ax_vel.grid(True, alpha=0.25)
        ax_vel.legend(loc="best", fontsize=8)

        ax_cmd.step(t, command[:, 0], where="post", linewidth=1.0, label="throttle")
        ax_cmd.step(t, command[:, 1], where="post", linewidth=1.0, label="steering")
        ax_cmd.plot(t, command[:, 2], linewidth=1.0, linestyle="--", label="motor actual")
        ax_cmd.plot(t, command[:, 3], linewidth=1.0, linestyle="--", label="steering actual")
        ax_cmd.set_title("Controller Output")
        ax_cmd.set_ylabel("command")
        ax_cmd.grid(True, alpha=0.25)
        ax_cmd.legend(loc="best", fontsize=8)

        ax_follow.plot(t, following_error[:, 0], linewidth=1.0, label="target_x - true_x")
        ax_follow.plot(t, following_error[:, 1], linewidth=1.0, label="target_y - true_y")
        ax_follow.axhline(0.0, color="0.4", linewidth=0.8)
        ax_follow.set_title("Path Following Error")
        ax_follow.set_ylabel("following error [m]")
        ax_follow.grid(True, alpha=0.25)
        ax_follow.legend(loc="best", fontsize=8)

        ax_err.plot(t, estimation_error[:, 0], linewidth=1.0, label="est_x - true_x")
        ax_err.plot(t, estimation_error[:, 1], linewidth=1.0, label="est_y - true_y")
        ax_err.plot(t, pos_error, linewidth=1.2, color="tab:red", label="position norm")
        ax_err.axhline(0.0, color="0.4", linewidth=0.8)
        ax_err.set_title("EKF Estimation Error")
        ax_err.set_xlabel("time [s]")
        ax_err.set_ylabel("error [m]")
        ax_err.grid(True, alpha=0.25)
        ax_err.legend(loc="best", fontsize=8)

        plot_path = os.path.join(output_dir, "integration_control_loop.png")
        fig.savefig(plot_path, dpi=120)
        plt.close(fig)
        print(f"[PLOT] saved {plot_path}")

    def _plot_racetrack(self, ax, waypoints: np.ndarray) -> None:
        track_half_width = 0.22
        theta = waypoints[:, 2]
        normal = np.column_stack((-np.sin(theta), np.cos(theta)))
        inner = waypoints[:, :2] - track_half_width * normal
        outer = waypoints[:, :2] + track_half_width * normal

        ax.fill(
            np.concatenate((outer[:, 0], inner[::-1, 0])),
            np.concatenate((outer[:, 1], inner[::-1, 1])),
            color="0.92",
            alpha=0.9,
            label="track road",
        )
        ax.plot(outer[:, 0], outer[:, 1], color="0.55", linewidth=0.8)
        ax.plot(inner[:, 0], inner[:, 1], color="0.55", linewidth=0.8)
        ax.plot(waypoints[:, 0], waypoints[:, 1], "k--", linewidth=1.0, label="centerline")


if __name__ == "__main__":
    unittest.main()
