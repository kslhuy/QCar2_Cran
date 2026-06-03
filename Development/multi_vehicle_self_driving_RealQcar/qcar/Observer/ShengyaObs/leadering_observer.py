"""
Leadering observer with time-delay integral term.

Dynamics:
    zeta_hat_dot = f_zeta(zeta_hat, u)
                 + B_zeta * integral_{t-hat_tau}^{t} g(zeta_hat(s), u(s)) ds
                 + L (y_zeta - C zeta_hat)
    z_filter_dot = gamma * y(leader position from V2V)

Where:
    zeta_hat= [z_hat; x_hat] in R^4
    y_zeta = z_filter
    u: leader control input (from V2V)
    f_zeta = [gamma * x_hat[0];  x_hat[1]; x_hat[2]; -1/eta * x_hat[2] - 1/eta * (c0 + c1 * x_hat[1]) + 1/eta * u]
    gamma = 1
    g(zeta_hat(s), u(s)) = x_hat[1]
    B_zeta = [-gamma; 0; 0; 0]
    C = [1, 0, 0, 0]
"""
from collections import deque
from typing import Dict, Optional

import numpy as np

from ..fleet_state_estimators import FleetStateEstimatorBase


class LeaderingObserverEstimator(FleetStateEstimatorBase):
    """Leadering observer with delay integral term for a 4-state zeta system."""

    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)

        self.gamma = float(self.config.get("gamma", 1))
        self.eta = float(self.config.get("eta", 0.16))
        self.control_index = int(self.config.get("control_index", 1))
        self.leader_control_index = int(self.config.get("leader_control_index", 1))
        self.leader_vehicle_id = int(self.config.get("leader_vehicle_id", 0))
        self.leader_position_index = int(self.config.get("leader_position_index", 0))
        self.z_measurement_index = int(self.config.get("z_measurement_index", 0))
        self.delay_compensation_enabled = bool(self.config.get("delay_compensation", True))

        self.h_matrix = self._load_row_vector(self.config.get("h_matrix"), default=[1.0, 0.0, 0.0])
        self.dh_dx = self._load_row_vector(self.config.get("dh_dx"), default=self.h_matrix)
        self.c0 = float(self.config.get("c0", 0.007023))
        self.c1 = float(self.config.get("c1", 0.14878))

        self.C = np.array([1.0, 0.0, 0.0, 0.0])
        self.B_zeta = np.array([-self.gamma, 0.0, 0.0, 0.0])

        self.L = np.array([48.66, 23.49, 0.5062, -0.0772])

        self.zeta_hat = np.zeros(4)
        self.z_filter = float(self.config.get("z0", 0.0))
        self._g_history = deque()
        self.g_history_seconds = float(self.config.get("g_history_seconds", 5.0))
        self.max_hat_tau = float(self.config.get("max_hat_tau", 0.147703))
        self.debug_data = {}
        self.debug_recording_enabled = bool(self.config.get("debug_recording", True))
        self.debug_output_dir = self.config.get("debug_output_dir", "observer_recordings")
        self.recorder = None
        self._recording_start_time: Optional[float] = None
        self._update_count = 0
        self.hat_tau_attack_enabled = bool(self.config.get("hat_tau_attack_enabled", False))
        self.hat_tau_attack_start_s = float(self.config.get("hat_tau_attack_start_s", 60.0))
        self.hat_tau_attack_bias_s = float(self.config.get("hat_tau_attack_bias_s", 0.08))
        self._hat_tau_attack_t0_ns: Optional[int] = None
        self._hat_tau_attack_used = False
        self.use_timestamp_dt = bool(self.config.get("use_timestamp_dt", True))
        self.max_update_dt_s = float(self.config.get("max_update_dt_s", 0.05))
        self._last_update_time_ns: Optional[int] = None

        if self.debug_recording_enabled:
            self._init_recorder()

    def _init_recorder(self) -> None:
        try:
            from .leadering_observer_recorder import LeaderingObserverRecorder

            self.recorder = LeaderingObserverRecorder(
                output_dir=self.debug_output_dir,
                vehicle_id=self.vehicle_id,
            )
            filepath = self.recorder.start()
            if self.logger:
                self.logger.logger.info(f"Leadering observer recording started: {filepath}")
        except Exception as exc:
            if self.logger:
                self.logger.log_error("Failed to initialize leadering observer recorder", exc)
            self.recorder = None

    def _load_row_vector(self, vector_cfg, default, expected_shape=(3,)) -> np.ndarray:
        if vector_cfg is None:
            return np.array(default, dtype=float)
        vector = np.array(vector_cfg, dtype=float).flatten()
        if vector.shape != expected_shape:
            if self.logger:
                self.logger.logger.warning(
                    f"LeaderingObserverEstimator: vector shape {vector.shape} invalid, using default"
                )
            return np.array(default, dtype=float)
        return vector

    def _load_observer_gain(self, gain_cfg) -> np.ndarray:
        if gain_cfg is None:
            return np.zeros((4, 1))
        gain = np.array(gain_cfg, dtype=float)
        if gain.shape == (4,):
            return gain.reshape(4, 1)
        if gain.shape == (1, 4):
            return gain.reshape(4, 1)
        if gain.shape != (4, 1):
            if self.logger:
                self.logger.logger.warning(
                    f"LeaderingObserverEstimator: observer_gain shape {gain.shape} invalid, using zeros"
                )
            return np.zeros((4, 1))
        return gain

    def compute_hat_tau(self, current_time_ns: int) -> float:
        """Estimate hat{tau} from the latest leader V2V send timestamp."""
        if not self.delay_compensation_enabled:
            return 0.0

        fallback_hat_tau = float(self.config.get("hat_tau", 0.0))

        history = self.received_local_states.get(self.leader_vehicle_id)
        if not history:
            return fallback_hat_tau

        for send_time_ns, _ in reversed(history):
            age_ns = current_time_ns - int(send_time_ns)
            if age_ns < 0:
                return 0.0
            if age_ns <= self.max_state_age_ns:
                return age_ns * 1e-9

        return fallback_hat_tau

    def _apply_hat_tau_attack(self, current_time_ns: int, hat_tau_s: float) -> tuple:
        """Inject a constant bias into hat_tau once at the configured attack time."""
        if not self.hat_tau_attack_enabled:
            return hat_tau_s, False, 0.0

        if self._hat_tau_attack_t0_ns is None:
            start_delay_ns = int(max(0.0, self.hat_tau_attack_start_s) * 1e9)
            self._hat_tau_attack_t0_ns = int(current_time_ns) + start_delay_ns

        if self._hat_tau_attack_used or current_time_ns < self._hat_tau_attack_t0_ns:
            return hat_tau_s, False, 0.0

        self._hat_tau_attack_used = True
        return hat_tau_s + self.hat_tau_attack_bias_s, True, self.hat_tau_attack_bias_s

    def _compute_update_dt(self, current_time_ns: int, fallback_dt: float) -> float:
        if not self.use_timestamp_dt:
            return float(fallback_dt)

        if self._last_update_time_ns is None:
            self._last_update_time_ns = int(current_time_ns)
            return float(fallback_dt)

        measured_dt = (int(current_time_ns) - self._last_update_time_ns) * 1e-9
        self._last_update_time_ns = int(current_time_ns)
        if measured_dt <= 0.0:
            return float(fallback_dt)
        return max(0.0, min(measured_dt, self.max_update_dt_s))

    def _extract_control_input(self, current_time_ns: int, control: np.ndarray) -> float:
        leader_control = self._get_latest_received_control(self.leader_vehicle_id, current_time_ns)
        if leader_control is not None:
            leader_arr = np.asarray(leader_control, dtype=float).flatten()
            if leader_arr.size == 1:
                return float(leader_arr[0])
            if leader_arr.size > 1:
                if 0 <= self.leader_control_index < leader_arr.size:
                    return float(leader_arr[self.leader_control_index])
                return float(leader_arr[-1])

        if control is None:
            return 0.0
        if np.isscalar(control):
            return float(control)
        control_arr = np.asarray(control, dtype=float).flatten()
        if control_arr.size == 0:
            return 0.0
        if 0 <= self.control_index < control_arr.size:
            return float(control_arr[self.control_index])
        return float(control_arr[-1])

    def _extract_y_zeta(self, current_time_ns: int, local_state) -> float:
        leader_state = self._get_latest_received_state(self.leader_vehicle_id, current_time_ns)
        if leader_state is not None:
            leader_arr = np.asarray(leader_state, dtype=float).flatten()
            if leader_arr.size > self.leader_position_index:
                return float(leader_arr[self.leader_position_index])

        if isinstance(local_state, dict):
            if "z" in local_state:
                return float(local_state["z"])
            if "y_zeta" in local_state:
                return float(local_state["y_zeta"])
            if "y" in local_state:
                return float(local_state["y"])
        if local_state is None:
            return 0.0
        arr = np.asarray(local_state, dtype=float).flatten()
        if arr.size > self.z_measurement_index:
            return float(arr[self.z_measurement_index])
        return float(arr[0]) if arr.size > 0 else 0.0

    def _compute_f_x(self, x_vec: np.ndarray, u_scalar: float) -> np.ndarray:
        x1 = x_vec[0]
        x2 = x_vec[1]
        x3 = x_vec[2]
        x1_dot = x2
        x2_dot = x3
        x3_dot = (-1.0 / self.eta) * x3 - (1.0 / self.eta) * (self.c0 + self.c1 * x2)
        x3_dot += (1.0 / self.eta) * u_scalar
        return np.array([x1_dot, x2_dot, x3_dot], dtype=float)

    def _compute_h(self, x_vec: np.ndarray) -> float:
        return float(self.h_matrix @ x_vec)

    def _compute_g(self, x_vec: np.ndarray, u_scalar: float) -> float:
        """Compute g(zeta_hat, u) = x_hat[1] (second component of x_vec)."""
        return float(x_vec[1])

    def _append_g_history(self, time_s: float, g_value: float) -> None:
        self._g_history.append((time_s, g_value))
        cutoff = time_s - self.g_history_seconds
        while self._g_history and self._g_history[0][0] < cutoff:
            self._g_history.popleft()

    def _integrate_g(self, current_time_s: float, hat_tau_s: float) -> float:
        if hat_tau_s <= 0.0 or not self._g_history:
            return 0.0

        t_start = current_time_s - hat_tau_s
        samples = list(self._g_history)
        if samples[-1][0] < t_start:
            return 0.0

        idx = None
        for i, (t, _) in enumerate(samples):
            if t >= t_start:
                idx = i
                break
        if idx is None:
            return 0.0

        if idx == 0:
            prev_t, prev_g = samples[0]
            if prev_t > t_start:
                prev_t = t_start
        else:
            t0, g0 = samples[idx - 1]
            t1, g1 = samples[idx]
            if t1 > t0:
                ratio = (t_start - t0) / (t1 - t0)
                prev_g = g0 + ratio * (g1 - g0)
                prev_t = t_start
            else:
                prev_t, prev_g = t_start, g1

        total = 0.0
        for t, g in samples[idx:]:
            dt = t - prev_t
            if dt > 0.0:
                total += 0.5 * (g + prev_g) * dt
            prev_t, prev_g = t, g
        return total

    def _compute_f_zeta(self, zeta_hat: np.ndarray, u_scalar: float) -> np.ndarray:
        """Compute f_zeta = [gamma*x_hat[0]; x_hat[1]; x_hat[2]; -1/eta*x_hat[2] - 1/eta*(c0+c1*x_hat[1]) + 1/eta*u]."""
        x_vec = zeta_hat[1:]
        f_x = self._compute_f_x(x_vec, u_scalar)
        return np.array([self.gamma * x_vec[0], f_x[0], f_x[1], f_x[2]], dtype=float)

    def _get_leader_truth(self, current_time_ns: int, local_state, control) -> tuple:
        """Return latest available leader truth as (x, v, a, u). Missing values are NaN."""
        state = None
        leader_control = None
        if self.vehicle_id == self.leader_vehicle_id:
            state = local_state
            leader_control = control
        else:
            state = self._get_latest_received_state(self.leader_vehicle_id, current_time_ns)
            leader_control = self._get_latest_received_control(self.leader_vehicle_id, current_time_ns)

        truth = np.full(4, np.nan, dtype=float)
        if state is not None:
            try:
                arr = np.asarray(state, dtype=float).flatten()
                if arr.size > 0:
                    truth[0] = arr[0]
                if arr.size > 3:
                    truth[1] = arr[3]
                if arr.size > 4:
                    truth[2] = arr[4]
            except Exception:
                pass

        if leader_control is not None:
            try:
                ctrl = np.asarray(leader_control, dtype=float).flatten()
                if ctrl.size == 1:
                    truth[3] = ctrl[0]
                elif ctrl.size > self.leader_control_index:
                    truth[3] = ctrl[self.leader_control_index]
                elif ctrl.size > 0:
                    truth[3] = ctrl[-1]
            except Exception:
                pass

        return tuple(float(v) for v in truth)

    def _record_debug_sample(self, current_time_ns: int, dt: float, hat_tau_s: float,
                             raw_hat_tau_s: float, hat_tau_attack_active: bool,
                             hat_tau_attack_bias_s: float, u_scalar: float,
                             y_zeta: float, innovation: float, integral_g: float,
                             g_value: float, local_state, control) -> None:
        true_x, true_v, true_a, true_u = self._get_leader_truth(current_time_ns, local_state, control)
        x_hat, v_hat, a_hat = self.zeta_hat[1], self.zeta_hat[2], self.zeta_hat[3]

        self.debug_data = {
            "time_ns": current_time_ns,
            "dt": dt,
            "hat_tau": hat_tau_s,
            "raw_hat_tau": raw_hat_tau_s,
            "hat_tau_attack_active": int(hat_tau_attack_active),
            "hat_tau_attack_bias": hat_tau_attack_bias_s,
            "u_leader": u_scalar,
            "y_zeta": y_zeta,
            "z_filter": self.z_filter,
            "innovation": innovation,
            "integral_g": integral_g,
            "g_value": g_value,
            "zeta_hat": self.zeta_hat.copy(),
            "zeta_hat_0": self.zeta_hat[0],
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
        }

        if self.recorder is not None:
            if self._recording_start_time is None:
                self._recording_start_time = current_time_ns / 1e9
            row = dict(self.debug_data)
            row["time"] = current_time_ns / 1e9 - self._recording_start_time
            row.pop("time_ns", None)
            row.pop("zeta_hat", None)
            self.recorder.record(row)
            self._update_count += 1

    def update(self, local_state: np.ndarray, dt: float,
               current_time_ns: int, control: np.ndarray) -> np.ndarray:
        try:
            self._ensure_fleet_capacity(self.vehicle_id)

            dt = self._compute_update_dt(current_time_ns, dt)
            current_time_s = float(current_time_ns) * 1e-9
            u_scalar = self._extract_control_input(current_time_ns, control)
            delayed_y = self._extract_y_zeta(current_time_ns, local_state)
            y_zeta = self.z_filter

            raw_hat_tau_s = self.compute_hat_tau(current_time_ns)
            capped_hat_tau_s = max(0.0, min(raw_hat_tau_s, self.max_hat_tau))
            hat_tau_s, hat_tau_attack_active, hat_tau_attack_bias_s = self._apply_hat_tau_attack(
                current_time_ns,
                capped_hat_tau_s,
            )

            g_value = self._compute_g(self.zeta_hat[1:], u_scalar)
            self._append_g_history(current_time_s, g_value)
            
            integral_g = self._integrate_g(current_time_s, hat_tau_s)

            f_zeta = self._compute_f_zeta(self.zeta_hat, u_scalar)

            innovation = y_zeta - float(self.C @ self.zeta_hat)

            correction = self.L * innovation

            zeta_dot = f_zeta + self.B_zeta * integral_g + correction

            self.z_filter = self.z_filter + dt * self.gamma * delayed_y
            self.zeta_hat = self.zeta_hat + dt * zeta_dot
            recorded_y_zeta = self.z_filter
            recorded_innovation = recorded_y_zeta - float(self.C @ self.zeta_hat)

            self._ensure_fleet_capacity(self.leader_vehicle_id)
            self.fleet_states[0, self.leader_vehicle_id] = self.zeta_hat[1]
            if self.state_dim > 3:
                self.fleet_states[3, self.leader_vehicle_id] = self.zeta_hat[2]
            if self.state_dim > 4:
                self.fleet_states[4, self.leader_vehicle_id] = self.zeta_hat[3]

            self._record_debug_sample(
                current_time_ns=current_time_ns,
                dt=dt,
                hat_tau_s=hat_tau_s,
                raw_hat_tau_s=raw_hat_tau_s,
                hat_tau_attack_active=hat_tau_attack_active,
                hat_tau_attack_bias_s=hat_tau_attack_bias_s,
                u_scalar=u_scalar,
                y_zeta=recorded_y_zeta,
                innovation=recorded_innovation,
                integral_g=integral_g,
                g_value=g_value,
                local_state=local_state,
                control=control,
            )

            self._cleanup_old_data(current_time_ns)
            return self.fleet_states.copy()

        except Exception as exc:
            if self.logger:
                self.logger.log_error("LeaderingObserverEstimator update error", exc)
            return self.fleet_states.copy()

    def get_zeta_hat(self) -> np.ndarray:
        return self.zeta_hat.copy()

    def get_x_hat(self) -> np.ndarray:
        return self.zeta_hat[1:].copy()

    def get_debug_data(self) -> Dict:
        return self.debug_data

    def stop_recording(self):
        if self.recorder is not None:
            stats = self.recorder.stop()
            self.recorder = None
            if self.logger:
                self.logger.logger.info(f"Leadering observer recording stopped: {stats}")
            return stats
        return None

    def __del__(self):
        try:
            self.stop_recording()
        except Exception:
            pass
