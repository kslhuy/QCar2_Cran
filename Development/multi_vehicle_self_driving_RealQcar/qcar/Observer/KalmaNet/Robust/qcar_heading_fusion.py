from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


def wrap_to_pi(angle: float) -> float:
    return float((float(angle) + np.pi) % (2.0 * np.pi) - np.pi)


def _wrap_observation(observation: float, reference: float) -> float:
    return wrap_to_pi(float(observation) - float(reference)) + float(reference)


@dataclass
class QCarHeadingFusionConfig:
    wheelbase: float = 0.2
    q_kf_theta: float = 1.0e-4
    q_kf_bias: float = 1.0e-3
    r_kf_gps_theta: float = 1.0e-3
    q_ekf_x: float = 1.0e-2
    q_ekf_y: float = 1.0e-2
    q_ekf_theta: float = 1.0e-2
    r_ekf_x: float = 1.0e-2
    r_ekf_y: float = 1.0e-2
    r_ekf_theta: float = 1.0e-3


class QCarHeadingFusion:
    """Local replica of the QCarEKF pose/orientation fusion path.

    This mirrors the Quanser `QCarEKF.update()` sequence exactly for the
    orientation path:
    1. Linear KF on [theta, gyro_bias] driven by gyro_z
    2. Nonlinear pose EKF on [x, y, theta] driven by [v, delta]
    3. GPS theta corrects the heading KF when available
    4. Pose EKF is corrected toward the heading-KF theta every step

    The resulting pose theta matches the heading returned by `QCarEKF.x_hat[2]`.
    """

    def __init__(
        self,
        initial_pose: Optional[Sequence[float]] = None,
        config: Optional[QCarHeadingFusionConfig] = None,
    ) -> None:
        self.config = config or QCarHeadingFusionConfig()
        self.L = max(float(self.config.wheelbase), 1e-6)
        self.A_kf = np.array([[0.0, -1.0], [0.0, 0.0]], dtype=np.float64)
        self.B_kf = np.array([[1.0], [0.0]], dtype=np.float64)
        self.C_kf = np.array([[1.0, 0.0]], dtype=np.float64)
        self.Q_kf = np.diag(
            [float(self.config.q_kf_theta), float(self.config.q_kf_bias)]
        ).astype(np.float64)
        self.R_kf = np.diag([float(self.config.r_kf_gps_theta)]).astype(np.float64)
        self.Q_ekf = np.diag(
            [
                float(self.config.q_ekf_x),
                float(self.config.q_ekf_y),
                float(self.config.q_ekf_theta),
            ]
        ).astype(np.float64)
        self.R_ekf = np.diag(
            [
                float(self.config.r_ekf_x),
                float(self.config.r_ekf_y),
                float(self.config.r_ekf_theta),
            ]
        ).astype(np.float64)
        self.I_kf = np.eye(2, dtype=np.float64)
        self.I_ekf = np.eye(3, dtype=np.float64)
        self.reset(initial_pose=initial_pose)

    def reset(self, initial_pose: Optional[Sequence[float]] = None) -> None:
        pose = np.zeros(3, dtype=np.float64)
        if initial_pose is not None:
            init = np.asarray(initial_pose, dtype=np.float64).reshape(-1)
            pose[: min(init.size, 3)] = init[:3]
        pose[2] = wrap_to_pi(float(pose[2]))
        self.kf_x_hat = np.array([[pose[2]], [0.0]], dtype=np.float64)
        self.kf_P = np.eye(2, dtype=np.float64)
        self.ekf_x_hat = pose.reshape(3, 1).astype(np.float64)
        self.ekf_P = np.eye(3, dtype=np.float64)
        self.x_hat = self.ekf_x_hat

    def _predict_linear_kf(self, u_gyro: float, dt: float) -> None:
        A_d = self.I_kf + self.A_kf * float(dt)
        self.kf_x_hat = A_d @ self.kf_x_hat + float(dt) * self.B_kf * float(u_gyro)
        self.kf_P = A_d @ self.kf_P @ A_d.T + self.Q_kf

    def _correct_linear(self, y, *, heading_kf: bool) -> None:
        y = np.squeeze(np.asarray(y, dtype=object))
        try:
            q = len(y)
        except TypeError:
            q = 1

        if heading_kf:
            state = self.kf_x_hat
            cov = self.kf_P
            C = self.C_kf
            R = self.R_kf
            I = self.I_kf
        else:
            state = self.ekf_x_hat
            cov = self.ekf_P
            C = np.eye(3, dtype=np.float64)
            R = self.R_ekf
            I = self.I_ekf

        if q > 1:
            deletion_list = np.nonzero(y == None)[0]
            y = np.delete(y, deletion_list).astype(np.float64).reshape(
                (q - len(deletion_list), 1)
            )
            C = np.delete(C, deletion_list, axis=0)
            R = np.delete(R, deletion_list, axis=0)
            R = np.delete(R, deletion_list, axis=1)
        else:
            y = np.array([[float(y)]], dtype=np.float64)

        P_times_C_t = cov @ C.T
        S = C @ P_times_C_t + R
        K = P_times_C_t @ np.linalg.inv(S)
        state = state + K @ (y - C @ state)
        cov = (I - K @ C) @ cov

        if heading_kf:
            self.kf_x_hat = state
            self.kf_P = cov
        else:
            self.ekf_x_hat = state
            self.ekf_P = cov

    def _predict_pose_ekf(self, velocity: float, steering: float, dt: float) -> None:
        theta = float(self.ekf_x_hat[2, 0])
        velocity = float(velocity)
        steering = float(steering)
        dt = float(dt)

        F = np.array(
            [
                [1.0, 0.0, -dt * velocity * np.sin(theta)],
                [0.0, 1.0, dt * velocity * np.cos(theta)],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.ekf_P = F @ self.ekf_P @ F.T + self.Q_ekf
        self.ekf_x_hat = self.ekf_x_hat + dt * velocity * np.array(
            [
                [np.cos(theta)],
                [np.sin(theta)],
                [np.tan(steering) / self.L],
            ],
            dtype=np.float64,
        )

    def update(
        self,
        *,
        velocity: Optional[float] = None,
        steering: Optional[float] = None,
        dt: Optional[float] = None,
        gps_pose: Optional[Sequence[float]] = None,
        gyro_z: Optional[float] = None,
    ) -> np.ndarray:
        if dt is not None:
            dt = max(float(dt), 1e-6)
            if gyro_z is not None:
                self._predict_linear_kf(gyro_z, dt)
                self.kf_x_hat[0, 0] = wrap_to_pi(float(self.kf_x_hat[0, 0]))
            if velocity is not None and steering is not None:
                self._predict_pose_ekf(velocity, steering, dt)
                self.ekf_x_hat[2, 0] = wrap_to_pi(float(self.ekf_x_hat[2, 0]))

        if gps_pose is not None:
            gps_pose_arr = np.asarray(gps_pose, dtype=np.float64).reshape(-1)
            y_kf = _wrap_observation(float(gps_pose_arr[2]), float(self.kf_x_hat[0, 0]))
            self._correct_linear(y_kf, heading_kf=True)
            self.kf_x_hat[0, 0] = wrap_to_pi(float(self.kf_x_hat[0, 0]))

            y_ekf = np.array(
                [
                    [float(gps_pose_arr[0])],
                    [float(gps_pose_arr[1])],
                    [float(self.kf_x_hat[0, 0])],
                ],
                dtype=np.float64,
            )
            z_ekf = y_ekf - self.ekf_x_hat
            z_ekf[2, 0] = wrap_to_pi(float(z_ekf[2, 0]))
            y_ekf = z_ekf + self.ekf_x_hat
            self._correct_linear(y_ekf, heading_kf=False)
            self.ekf_x_hat[2, 0] = wrap_to_pi(float(self.ekf_x_hat[2, 0]))
        else:
            y_ekf_theta = _wrap_observation(
                float(self.kf_x_hat[0, 0]), float(self.ekf_x_hat[2, 0])
            )
            self._correct_linear([None, None, y_ekf_theta], heading_kf=False)
            self.ekf_x_hat[2, 0] = wrap_to_pi(float(self.ekf_x_hat[2, 0]))

        self.x_hat = self.ekf_x_hat
        return self.ekf_x_hat.copy()

    def get_pose(self) -> np.ndarray:
        return self.ekf_x_hat[:, 0].copy()

    def get_heading(self) -> float:
        return wrap_to_pi(float(self.ekf_x_hat[2, 0]))
