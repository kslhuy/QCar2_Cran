"""
Extended Kalman filter for the leader 3-state vehicle dynamics.

Dynamics:
    x_dot = f_x(x, u)
    y = C x

Where:
    x in R^3
    y: leader position from V2V
    u: leader control input from V2V
    f_x = [
        x[1],
        x[2],
        -1/eta * x[2] - 1/eta * (c0 + c1 * x[1]) + 1/eta * u
    ]
    C = [1, 0, 0]
"""
from typing import Dict

import numpy as np

from .classical_luenberger_observer import ClassicalLuenbergerObserverEstimator


class ClassicalEKFObserverEstimator(ClassicalLuenbergerObserverEstimator):
    """EKF baseline for leader position-only V2V measurements."""

    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        config = dict(config or {})
        config.setdefault("recorder_prefix", "classical_ekf_observer")
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)

        self.H = self.C.reshape(1, 3)
        self.I3 = np.eye(3)
        self.P = self._load_covariance(
            self.config.get("initial_covariance", self.config.get("P0")),
            default_diag=[1.0, 1.0, 1.0],
            name="initial_covariance",
        )
        self.Q = self._load_covariance(
            self.config.get("process_noise_covariance", self.config.get("Q")),
            default_diag=[1e-4, 1e-3, 1e-2],
            name="process_noise_covariance",
        )
        self.R = self._load_measurement_covariance(
            self.config.get("measurement_noise_covariance", self.config.get("R"))
        )

    def _init_recorder(self) -> None:
        try:
            from .leadering_observer_recorder import LeaderingObserverRecorder

            self.recorder = LeaderingObserverRecorder(
                output_dir=self.debug_output_dir,
                vehicle_id=self.vehicle_id,
                observer_name=self.config.get("recorder_prefix", "classical_ekf_observer"),
            )
            self.recorder.columns.extend(
                [
                    "kalman_gain_x",
                    "kalman_gain_v",
                    "kalman_gain_a",
                    "innovation_covariance",
                    "P_xx",
                    "P_vv",
                    "P_aa",
                ]
            )
            filepath = self.recorder.start()
            if self.logger:
                self.logger.logger.info(f"Classical EKF observer recording started: {filepath}")
        except Exception as exc:
            if self.logger:
                self.logger.log_error("Failed to initialize classical EKF observer recorder", exc)
            self.recorder = None

    def _load_covariance(self, cfg, default_diag, name: str) -> np.ndarray:
        if cfg is None:
            return np.diag(default_diag).astype(float)

        cov = np.array(cfg, dtype=float)
        if cov.shape == (3,):
            return np.diag(cov)
        if cov.shape == (3, 3):
            return cov

        if self.logger:
            self.logger.logger.warning(
                f"ClassicalEKFObserverEstimator: {name} shape {cov.shape} invalid, using default"
            )
        return np.diag(default_diag).astype(float)

    def _load_measurement_covariance(self, cfg) -> np.ndarray:
        if cfg is None:
            variance = max(self.v2v_position_noise_std ** 2, 1e-6)
            return np.array([[variance]], dtype=float)

        cov = np.array(cfg, dtype=float)
        if cov.shape == ():
            return np.array([[float(cov)]], dtype=float)
        if cov.shape == (1,):
            return np.array([[float(cov[0])]], dtype=float)
        if cov.shape == (1, 1):
            return cov

        if self.logger:
            self.logger.logger.warning(
                f"ClassicalEKFObserverEstimator: measurement_noise_covariance shape {cov.shape} invalid, using default"
            )
        variance = max(self.v2v_position_noise_std ** 2, 1e-6)
        return np.array([[variance]], dtype=float)

    def _compute_state_jacobian(self) -> np.ndarray:
        """Continuous-time Jacobian A = df_x / dx."""
        return np.array(
            [
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, -self.c1 / self.eta, -1.0 / self.eta],
            ],
            dtype=float,
        )

    def _record_ekf_debug_sample(self, current_time_ns: int, dt: float, u_scalar: float,
                                 y: float, innovation: float, innovation_covariance: float,
                                 kalman_gain: np.ndarray, local_state, control) -> None:
        true_x, true_v, true_a, true_u = self._get_leader_truth(current_time_ns, local_state, control)
        x_hat, v_hat, a_hat = self.x_hat

        self.debug_data = {
            "time_ns": current_time_ns,
            "dt": dt,
            "hat_tau": 0.0,
            "raw_hat_tau": 0.0,
            "hat_tau_attack_active": 0,
            "hat_tau_attack_bias": 0.0,
            "u_leader": u_scalar,
            "y_zeta": y,
            "v2v_measurement_delay": self.v2v_measurement_delay_s,
            "v2v_measurement_age": self._last_v2v_measurement_age_s,
            "v2v_position_noise": self._last_v2v_position_noise,
            "innovation": innovation,
            "integral_g": 0.0,
            "g_value": 0.0,
            "x_hat": self.x_hat.copy(),
            "zeta_hat_x": x_hat,
            "zeta_hat_v": v_hat,
            "zeta_hat_a": a_hat,
            "true_leader_x": true_x,
            "true_leader_v": true_v,
            "true_leader_a": true_a,
            "true_leader_u": true_u,
            "err_x": x_hat - true_x if np.isfinite(true_x) else np.nan,
            "err_v": v_hat - true_v if np.isfinite(true_v) else np.nan,
            "err_a": a_hat - true_a if np.isfinite(true_a) else np.nan,
            "kalman_gain_x": kalman_gain[0],
            "kalman_gain_v": kalman_gain[1],
            "kalman_gain_a": kalman_gain[2],
            "innovation_covariance": innovation_covariance,
            "P_xx": self.P[0, 0],
            "P_vv": self.P[1, 1],
            "P_aa": self.P[2, 2],
        }

        if self.recorder is not None:
            if self._recording_start_time is None:
                self._recording_start_time = current_time_ns / 1e9
            row = dict(self.debug_data)
            row["time"] = current_time_ns / 1e9 - self._recording_start_time
            row["timestamp"] = current_time_ns / 1e9
            row.pop("time_ns", None)
            row.pop("x_hat", None)
            self.recorder.record(row)
            self._update_count += 1

    def update(self, local_state: np.ndarray, dt: float,
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        try:
            self._ensure_fleet_capacity(self.vehicle_id)

            dt = self._compute_update_dt(current_time_ns, dt)
            u_scalar = self._extract_control_input(current_time_ns, control)
            y = self._extract_y(current_time_ns, local_state)

            x_pred = self.x_hat + dt * self._compute_f_x(self.x_hat, u_scalar)
            A = self._compute_state_jacobian()
            F = self.I3 + dt * A
            P_pred = F @ self.P @ F.T + self.Q

            innovation = y - float(self.H @ x_pred)
            S = self.H @ P_pred @ self.H.T + self.R
            S_scalar = float(S[0, 0])
            if S_scalar <= 0.0:
                S_scalar = 1e-9
            K = (P_pred @ self.H.T / S_scalar).flatten()

            self.x_hat = x_pred + K * innovation
            KH = K.reshape(3, 1) @ self.H
            self.P = (self.I3 - KH) @ P_pred @ (self.I3 - KH).T
            self.P += K.reshape(3, 1) @ self.R @ K.reshape(1, 3)
            self.P = 0.5 * (self.P + self.P.T)

            recorded_innovation = y - float(self.H @ self.x_hat)

            self._ensure_fleet_capacity(self.leader_vehicle_id)
            self.fleet_states[0, self.leader_vehicle_id] = self.x_hat[0]
            if self.state_dim > 3:
                self.fleet_states[3, self.leader_vehicle_id] = self.x_hat[1]
            if self.state_dim > 4:
                self.fleet_states[4, self.leader_vehicle_id] = self.x_hat[2]

            self._record_ekf_debug_sample(
                current_time_ns=current_time_ns,
                dt=dt,
                u_scalar=u_scalar,
                y=y,
                innovation=recorded_innovation,
                innovation_covariance=S_scalar,
                kalman_gain=K,
                local_state=local_state,
                control=control,
            )

            self._cleanup_old_data(current_time_ns)
            return self.fleet_states.copy()

        except Exception as exc:
            if self.logger:
                self.logger.log_error("ClassicalEKFObserverEstimator update error", exc)
            return self.fleet_states.copy()
