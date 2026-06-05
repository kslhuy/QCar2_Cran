"""
Classical Luenberger observer for the leader 3-state vehicle dynamics.

Dynamics:
    x_hat_dot = f_x(x_hat, u) + L (y - y_hat)

Where:
    x_hat in R^3
    y = 1 / (x1 + 1), where x1 is the leader position from V2V
    y_hat = 1 / (x_hat[0] + 1)
    u: leader control input from V2V
    f_x = [
        x_hat[1],
        x_hat[2],
        -1/eta * x_hat[2] - 1/eta * (c0 + c1 * x_hat[1]) + 1/eta * u
    ]
"""
from typing import Dict, Optional

import numpy as np

from ..fleet_state_estimators import FleetStateEstimatorBase
from .leadering_observer import LeaderingObserverEstimator


class ClassicalLuenbergerObserverEstimator(LeaderingObserverEstimator):
    """Classical Luenberger baseline without delay compensation."""

    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        FleetStateEstimatorBase.__init__(self, vehicle_id, fleet_size, state_dim, config, logger)

        self.eta = float(self.config.get("eta", 0.16))
        self.control_index = int(self.config.get("control_index", 1))
        self.leader_control_index = int(self.config.get("leader_control_index", 1))
        self.leader_vehicle_id = int(self.config.get("leader_vehicle_id", 0))
        self.leader_position_index = int(self.config.get("leader_position_index", 0))
        self.z_measurement_index = int(self.config.get("z_measurement_index", 0))
        self.v2v_measurement_delay_s = float(self.config.get("v2v_measurement_delay_s", 0.0))
        self.v2v_position_noise_std = float(self.config.get("v2v_position_noise_std", 0.01))
        noise_seed = self.config.get("measurement_noise_seed")
        self._measurement_rng = np.random.default_rng(noise_seed)
        self._last_v2v_position_noise = 0.0
        self._last_v2v_measurement_age_s = 0.0

        self.c0 = float(self.config.get("c0", 0.007023))
        self.c1 = float(self.config.get("c1", 0.14878))
        self.measurement_denominator_epsilon = float(
            self.config.get("measurement_denominator_epsilon", 1e-6)
        )
        self.singularity_position_margin = float(
            self.config.get("singularity_position_margin", self.measurement_denominator_epsilon)
        )
        self.initialize_position_from_measurement = bool(
            self.config.get("initialize_position_from_measurement", True)
        )
        self._position_initialized_from_measurement = False
        self.C = np.array([1.0, 0.0, 0.0])
        self.L = self._load_observer_gain(
            self.config.get("observer_gain", self.config.get("L"))
        )

        self.x_hat = np.zeros(3)
        self.debug_data = {}
        self.debug_recording_enabled = bool(self.config.get("debug_recording", True))
        self.debug_output_dir = self.config.get("debug_output_dir", "observer_recordings")
        self.recorder = None
        self._recording_start_time: Optional[float] = None
        self._update_count = 0
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
                observer_name=self.config.get("recorder_prefix", "classical_luenberger_observer"),
            )
            filepath = self.recorder.start()
            if self.logger:
                self.logger.logger.info(f"Classical Luenberger observer recording started: {filepath}")
        except Exception as exc:
            if self.logger:
                self.logger.log_error("Failed to initialize classical Luenberger observer recorder", exc)
            self.recorder = None

    def _load_observer_gain(self, gain_cfg) -> np.ndarray:
        if gain_cfg is None:
            return np.array([1.58, 0.0332, -0.0050], dtype=float)
        gain = np.array(gain_cfg, dtype=float).flatten()
        if gain.shape != (3,):
            if self.logger:
                self.logger.logger.warning(
                    f"ClassicalLuenbergerObserverEstimator: observer_gain shape {gain.shape} invalid, using default"
                )
            return np.array([1.58, 0.0332, -0.0050], dtype=float)
        return gain

    def _extract_y(self, current_time_ns: int, local_state) -> float:
        return self._extract_y_zeta(current_time_ns, local_state)

    def _safe_measurement_denominator(self, position: float) -> float:
        denominator = float(position) + 1.0
        if abs(denominator) >= self.measurement_denominator_epsilon:
            return denominator
        if denominator >= 0.0:
            return self.measurement_denominator_epsilon
        return -self.measurement_denominator_epsilon

    def _measurement_output(self, position: float) -> float:
        return 1.0 / self._safe_measurement_denominator(position)

    def _project_away_from_singularity(self) -> None:
        min_position = -1.0 + max(self.singularity_position_margin, 0.0)
        if self.x_hat[0] <= min_position:
            self.x_hat[0] = min_position

    def _initialize_position_if_needed(self, position_measurement: float) -> None:
        if self._position_initialized_from_measurement:
            return
        if not self.initialize_position_from_measurement:
            self._position_initialized_from_measurement = True
            return
        if not np.isfinite(position_measurement):
            return

        self.x_hat[0] = float(position_measurement)
        self._project_away_from_singularity()
        self._position_initialized_from_measurement = True

    def _compute_f_x(self, x_vec: np.ndarray, u_scalar: float) -> np.ndarray:
        x2 = x_vec[1]
        x3 = x_vec[2]
        x1_dot = x2
        x2_dot = x3
        x3_dot = (-1.0 / self.eta) * x3 - (1.0 / self.eta) * (self.c0 + self.c1 * x2)
        x3_dot += (1.0 / self.eta) * u_scalar
        return np.array([x1_dot, x2_dot, x3_dot], dtype=float)

    def _record_debug_sample(self, current_time_ns: int, dt: float, u_scalar: float,
                             y: float, y_position: float, y_hat: float,
                             innovation: float, local_state, control) -> None:
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
            "leader_position_measurement": y_position,
            "y_hat": y_hat,
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
            y_position = self._extract_y(current_time_ns, local_state)
            self._initialize_position_if_needed(y_position)
            y = self._measurement_output(y_position)
            y_hat = self._measurement_output(self.x_hat[0])
            innovation = y - y_hat

            x_hat_dot = self._compute_f_x(self.x_hat, u_scalar) + self.L * innovation
            self.x_hat = self.x_hat + dt * x_hat_dot
            self._project_away_from_singularity()
            recorded_y_hat = self._measurement_output(self.x_hat[0])
            recorded_innovation = y - recorded_y_hat

            self._ensure_fleet_capacity(self.leader_vehicle_id)
            self.fleet_states[0, self.leader_vehicle_id] = self.x_hat[0]
            if self.state_dim > 3:
                self.fleet_states[3, self.leader_vehicle_id] = self.x_hat[1]
            if self.state_dim > 4:
                self.fleet_states[4, self.leader_vehicle_id] = self.x_hat[2]

            self._record_debug_sample(
                current_time_ns=current_time_ns,
                dt=dt,
                u_scalar=u_scalar,
                y=y,
                y_position=y_position,
                y_hat=recorded_y_hat,
                innovation=recorded_innovation,
                local_state=local_state,
                control=control,
            )

            self._cleanup_old_data(current_time_ns)
            return self.fleet_states.copy()

        except Exception as exc:
            if self.logger:
                self.logger.log_error("ClassicalLuenbergerObserverEstimator update error", exc)
            return self.fleet_states.copy()

    def compute_hat_tau(self, current_time_ns: int) -> float:
        return 0.0

    def get_zeta_hat(self) -> np.ndarray:
        return self.x_hat.copy()

    def get_x_hat(self) -> np.ndarray:
        return self.x_hat.copy()

    def get_debug_data(self) -> Dict:
        return self.debug_data

    def stop_recording(self):
        if self.recorder is not None:
            stats = self.recorder.stop()
            self.recorder = None
            if self.logger:
                self.logger.logger.info(f"Classical Luenberger observer recording stopped: {stats}")
            return stats
        return None

    def __del__(self):
        try:
            self.stop_recording()
        except Exception:
            pass


class HighGainLuenbergerObserverEstimator(ClassicalLuenbergerObserverEstimator):
    """High-gain Luenberger observer using per-state gain powers."""

    def __init__(self, vehicle_id: int, fleet_size: int, state_dim: int = 5,
                 config: Dict = None, logger=None):
        config = dict(config or {})
        config.setdefault("recorder_prefix", "high_gain_luenberger_observer")
        super().__init__(vehicle_id, fleet_size, state_dim, config, logger)

        self.base_L = self.L.copy()
        self.high_gain = float(self.config.get("high_gain", self.config.get("ell", 1.0)))
        self.high_gain_exponents = self._load_high_gain_exponents(
            self.config.get("high_gain_exponents", [1.0, 2.0, 3.0])
        )
        self.L = self._compute_high_gain_observer_gain()

    def _load_high_gain_exponents(self, exponent_cfg) -> np.ndarray:
        exponents = np.array(exponent_cfg, dtype=float).flatten()
        if exponents.shape != (3,):
            if self.logger:
                self.logger.logger.warning(
                    "HighGainLuenbergerObserverEstimator: high_gain_exponents "
                    f"shape {exponents.shape} invalid, using [1, 2, 3]"
                )
            return np.array([1.0, 2.0, 3.0], dtype=float)
        return exponents

    def _compute_high_gain_observer_gain(self) -> np.ndarray:
        return self.base_L * np.power(self.high_gain, self.high_gain_exponents)
