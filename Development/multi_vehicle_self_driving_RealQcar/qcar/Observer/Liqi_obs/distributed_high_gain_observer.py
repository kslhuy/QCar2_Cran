"""
Distributed high-gain observer using the QCar bicycle-kinematic fleet state.

This estimator keeps the existing fleet estimator interface and state layout:

    z = y = [x, y, theta, velocity, acceleration]

The prediction model matches the bicycle kinematics used by the trust-based
fleet estimator. The Liqi distributed high-gain structure is applied as a
full-state correction on the same-size z/y vectors, plus a distributed
consensus term from neighbor fleet estimates.
"""

import copy
import os
from typing import Any, Dict, Optional, Tuple

import numpy as np
import yaml

try:
    from Observer.fleet_state_estimators import (
        FleetStateEstimatorBase,
        _normalize_state_array,
        _state_dict_to_array,
    )
except ImportError:  # pragma: no cover - direct package execution fallback
    from ..fleet_state_estimators import (
        FleetStateEstimatorBase,
        _normalize_state_array,
        _state_dict_to_array,
    )


DEFAULT_STATE_DIM = 5
X_INDEX = 0
Y_INDEX = 1
THETA_INDEX = 2
VELOCITY_INDEX = 3
ACCELERATION_INDEX = 4


def _deep_merge_dict(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge override into base and return base."""
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge_dict(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def _safe_float(value: Any, default: float) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _clip_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    if max_norm <= 0.0:
        return vec
    norm = float(np.linalg.norm(vec))
    if norm > max_norm and norm > 1e-12:
        return vec * (max_norm / norm)
    return vec


def _wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def _state_error(measured: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    err = measured - estimate
    if err.size > THETA_INDEX:
        err[THETA_INDEX] = _wrap_angle(measured[THETA_INDEX] - estimate[THETA_INDEX])
    return err


class DistributedHighGainFleetEstimator(FleetStateEstimatorBase):
    """
    Distributed high-gain observer compatible with FleetStateEstimatorBase.

    Per target vehicle, the estimator performs:
    1. bicycle-kinematic prediction on z = [x, y, theta, v, a],
    2. high-gain full-state measurement correction from y of the same size,
    3. full-state consensus correction from neighbor fleet estimates.
    """

    def __init__(
        self,
        vehicle_id: int,
        fleet_size: int,
        state_dim: int = DEFAULT_STATE_DIM,
        config: Dict = None,
        logger=None,
    ):
        merged_config = self._load_effective_config(config or {})
        super().__init__(vehicle_id, fleet_size, state_dim, merged_config, logger)

        observer_cfg = self.config.get("observer", {})
        topology_cfg = self.config.get("topology", {})
        dynamics_cfg = self.config.get("dynamics", {})
        constraints_cfg = self.config.get("constraints", {})
        debug_cfg = self.config.get("debug", {})

        self.z_dim = int(observer_cfg.get("z_dim", self.state_dim))
        if self.z_dim != self.state_dim:
            self._log(
                "warning",
                f"Configured z_dim={self.z_dim} does not match fleet state_dim={self.state_dim}; using state_dim",
            )
            self.z_dim = self.state_dim

        self.theta_config = observer_cfg.get("high_gain_theta", 1.2)
        self.observer_gain_config = observer_cfg.get(
            "observer_gain", [0.5, 0.5, 0.35, 0.6, 0.4]
        )
        self.consensus_gain = max(
            0.0, _safe_float(observer_cfg.get("consensus_gain", 0.4), 0.4)
        )
        self.P_inv = np.linalg.pinv(
            self._as_square_matrix(
                observer_cfg.get("p_matrix"),
                self.z_dim,
                np.eye(self.z_dim),
            )
        )

        self.set_own_state_from_local = bool(
            observer_cfg.get("set_own_state_from_local", True)
        )
        self.direct_state_blend = float(
            np.clip(
                _safe_float(observer_cfg.get("direct_state_blend", 0.0), 0.0),
                0.0,
                1.0,
            )
        )
        self.normalize_consensus_by_weight = bool(
            observer_cfg.get("normalize_consensus_by_weight", True)
        )
        self.max_innovation_norm = _safe_float(
            observer_cfg.get("max_innovation_norm", 10.0), 10.0
        )
        self.max_measurement_term_norm = _safe_float(
            observer_cfg.get("max_measurement_term_norm", 50.0), 50.0
        )
        self.max_consensus_term_norm = _safe_float(
            observer_cfg.get("max_consensus_term_norm", 50.0), 50.0
        )
        self.min_dt = max(0.0, _safe_float(observer_cfg.get("min_dt", 0.001), 0.001))
        self.max_dt = max(self.min_dt, _safe_float(observer_cfg.get("max_dt", 0.2), 0.2))

        self.topology_type = str(topology_cfg.get("type", "fully_connected")).lower()
        self.adjacency_config = topology_cfg.get("adjacency_matrix")
        self.row_vector_r_config = topology_cfg.get("row_vector_r")
        self.adjacency_matrix = self._build_adjacency(self.fleet_size)
        self.row_vector_r = self._resolve_row_vector_r()

        vehicle_cfg = self.config.get("vehicle", {})
        self.wheelbase = _safe_float(
            dynamics_cfg.get("wheelbase", vehicle_cfg.get("wheelbase", 0.256)),
            0.256,
        )
        self.dynamics_model = str(dynamics_cfg.get("model", "bicycle")).lower()
        self.motor_enabled = bool(dynamics_cfg.get("motor_model_enabled", False))
        self.motor_tau = max(1e-3, _safe_float(dynamics_cfg.get("motor_tau", 0.318), 0.318))
        self.k_th = _safe_float(dynamics_cfg.get("k_th", 4.5795), 4.5795)
        self.k_br = _safe_float(dynamics_cfg.get("k_br", 4.5772), 4.5772)
        self.c_v = _safe_float(dynamics_cfg.get("c_v", 5.0), 5.0)
        self.c_fric = _safe_float(dynamics_cfg.get("c_fric", 2.0), 2.0)

        self.max_abs_position = _safe_float(
            constraints_cfg.get("max_abs_position", 10000.0), 10000.0
        )
        self.max_abs_velocity = _safe_float(
            constraints_cfg.get("max_abs_velocity", 5.0), 5.0
        )
        self.max_abs_acceleration = _safe_float(
            constraints_cfg.get("max_abs_acceleration", 10.0), 10.0
        )
        self.max_abs_steering = _safe_float(
            constraints_cfg.get("max_abs_steering", 0.6), 0.6
        )
        self.max_abs_throttle = _safe_float(
            constraints_cfg.get("max_abs_throttle", 1.0), 1.0
        )

        if "max_state_age_ns" in self.config:
            self.max_state_age_ns = int(self.config.get("max_state_age_ns"))

        self._received_control_inputs: Dict[int, np.ndarray] = {}
        self._motor_accel_state: Dict[int, float] = {}
        self.debug_enabled = bool(debug_cfg.get("enabled", False))
        self.debug_data: Dict[str, Any] = {}

        self._log(
            "info",
            "DistributedHighGainFleetEstimator initialized "
            f"(vehicle_id={vehicle_id}, fleet_size={fleet_size}, z_dim={self.z_dim}, "
            f"theta={self.theta_config}, gamma={self.consensus_gain}, "
            f"dynamics={self.dynamics_model})",
        )

    @classmethod
    def _load_effective_config(cls, runtime_config: Dict[str, Any]) -> Dict[str, Any]:
        skill_dir = os.path.dirname(os.path.abspath(__file__))
        observer_dir = os.path.dirname(skill_dir)
        default_path = os.path.join(skill_dir, "config_distributed_high_gain.yaml")

        merged: Dict[str, Any] = cls._load_yaml(default_path)

        runtime_config = (
            copy.deepcopy(runtime_config) if isinstance(runtime_config, dict) else {}
        )
        config_file = runtime_config.get("config_file")
        if config_file:
            config_path = cls._resolve_config_path(config_file, skill_dir, observer_dir)
            if config_path and os.path.abspath(config_path) != os.path.abspath(default_path):
                _deep_merge_dict(merged, cls._load_yaml(config_path))

        runtime_config.pop("config_file", None)
        _deep_merge_dict(merged, runtime_config)
        cls._promote_legacy_keys(merged)
        return merged

    @staticmethod
    def _load_yaml(path: str) -> Dict[str, Any]:
        try:
            with open(path, "r") as stream:
                data = yaml.safe_load(stream) or {}
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            return {}

    @staticmethod
    def _resolve_config_path(path: str, skill_dir: str, observer_dir: str) -> Optional[str]:
        if os.path.isabs(path) and os.path.exists(path):
            return path
        for root in (skill_dir, observer_dir, os.getcwd()):
            candidate = os.path.join(root, path)
            if os.path.exists(candidate):
                return candidate
        return None

    @staticmethod
    def _promote_legacy_keys(config: Dict[str, Any]) -> None:
        observer_cfg = config.setdefault("observer", {})
        for key in (
            "high_gain_theta",
            "observer_gain",
            "consensus_gain",
            "p_matrix",
            "direct_state_blend",
        ):
            if key in config and key not in observer_cfg:
                observer_cfg[key] = config[key]

    def _log(self, level: str, message: str) -> None:
        if self.logger is None:
            return
        target = getattr(self.logger, "logger", self.logger)
        log_fn = getattr(target, level, None)
        if callable(log_fn):
            log_fn(message)

    def _as_square_matrix(
        self, value: Any, dim: int, default: np.ndarray
    ) -> np.ndarray:
        if value is None:
            return default.copy()
        try:
            arr = np.asarray(value, dtype=float)
            if arr.ndim == 0:
                return float(arr) * np.eye(dim)
            if arr.ndim == 1:
                if arr.size == dim:
                    return np.diag(arr)
                if arr.size == dim * dim:
                    return arr.reshape((dim, dim))
            if arr.ndim == 2 and arr.shape == (dim, dim):
                return arr
        except (TypeError, ValueError):
            pass
        self._log("warning", "Invalid p_matrix for distributed high-gain observer; using identity")
        return default.copy()

    def _gain_vector_for_vehicle(self, target_id: int) -> np.ndarray:
        default = np.ones(self.z_dim, dtype=float) * 0.4
        try:
            gain = np.asarray(self.observer_gain_config, dtype=float)
            if gain.ndim == 0:
                return np.full(self.z_dim, float(gain))
            if gain.ndim == 1:
                return self._resize_vector(gain, self.z_dim, default)
            if gain.ndim == 2 and target_id < gain.shape[0]:
                return self._resize_vector(gain[target_id, :], self.z_dim, default)
        except (TypeError, ValueError):
            pass
        return default

    @staticmethod
    def _resize_vector(value: np.ndarray, dim: int, default: np.ndarray) -> np.ndarray:
        arr = np.asarray(value, dtype=float).flatten()
        if arr.size == dim:
            return arr
        if arr.size > dim:
            return arr[:dim]
        if arr.size > 0:
            out = default.copy()
            out[: arr.size] = arr
            return out
        return default.copy()

    def _theta_for_vehicle(self, target_id: int) -> float:
        theta = self.theta_config
        try:
            if isinstance(theta, dict):
                value = theta.get(
                    target_id,
                    theta.get(str(target_id), theta.get("default", 1.2)),
                )
                return max(1.000001, float(value))
            arr = np.asarray(theta, dtype=float).flatten()
            if arr.size == 0:
                return 1.2
            if arr.size == 1:
                return max(1.000001, float(arr[0]))
            index = min(max(target_id, 0), arr.size - 1)
            return max(1.000001, float(arr[index]))
        except (TypeError, ValueError):
            return 1.2

    def _dilation_matrix(self, theta: float) -> np.ndarray:
        powers = np.array([theta ** (i + 1) for i in range(self.z_dim)], dtype=float)
        return np.diag(powers)

    def _consensus_transform(self, theta: float) -> np.ndarray:
        t_mat = self._dilation_matrix(theta)
        t_inv = np.diag(1.0 / np.diag(t_mat))
        return t_mat @ self.P_inv @ t_inv

    def _build_adjacency(self, size: int) -> np.ndarray:
        size = max(1, int(size))
        if self.adjacency_config is not None:
            try:
                raw = np.asarray(self.adjacency_config, dtype=float)
                if raw.ndim == 2 and raw.shape[0] > 0 and raw.shape[1] > 0:
                    adj = np.zeros((size, size), dtype=float)
                    rows = min(size, raw.shape[0])
                    cols = min(size, raw.shape[1])
                    adj[:rows, :cols] = raw[:rows, :cols]
                    np.fill_diagonal(adj, 0.0)
                    return adj
            except (TypeError, ValueError):
                self._log("warning", "Invalid custom adjacency_matrix; using generated topology")

        adj = np.zeros((size, size), dtype=float)
        if size <= 1:
            return adj
        if self.topology_type == "chain":
            for i in range(size):
                if i > 0:
                    adj[i, i - 1] = 1.0
                if i < size - 1:
                    adj[i, i + 1] = 1.0
        elif self.topology_type == "ring":
            for i in range(size):
                adj[i, (i - 1) % size] = 1.0
                adj[i, (i + 1) % size] = 1.0
        else:
            adj[:] = 1.0
            np.fill_diagonal(adj, 0.0)
        return adj

    def _resolve_row_vector_r(self) -> np.ndarray:
        size = self.adjacency_matrix.shape[0]
        if self.row_vector_r_config is not None:
            try:
                r_vec = np.asarray(self.row_vector_r_config, dtype=float).flatten()
                if r_vec.size > 0:
                    padded = np.ones(size, dtype=float)
                    count = min(size, r_vec.size)
                    padded[:count] = np.maximum(r_vec[:count], 1e-9)
                    return padded / np.sum(padded) * size
            except (TypeError, ValueError):
                self._log("warning", "Invalid row_vector_r; computing from Laplacian")

        try:
            degree = np.sum(self.adjacency_matrix, axis=1)
            laplacian = np.diag(degree) - self.adjacency_matrix
            eigvals, eigvecs = np.linalg.eig(laplacian.T)
            idx = int(np.argmin(np.abs(eigvals)))
            r_vec = np.real(eigvecs[:, idx])
            if np.sum(r_vec) < 0.0:
                r_vec = -r_vec
            r_vec = np.maximum(np.abs(r_vec), 1e-9)
            return r_vec / np.sum(r_vec) * size
        except np.linalg.LinAlgError:
            return np.ones(size, dtype=float)

    def _ensure_fleet_capacity(self, min_vehicle_id: int):
        old_size = self.fleet_size
        super()._ensure_fleet_capacity(min_vehicle_id)
        if self.fleet_size != old_size:
            self.adjacency_matrix = self._build_adjacency(self.fleet_size)
            self.row_vector_r = self._resolve_row_vector_r()

    def add_received_local_state(self, sender_id: int, state: Dict, timestamp_ns: int) -> bool:
        if isinstance(state, dict):
            control = state.get("control_input", {})
            if isinstance(control, dict):
                self._received_control_inputs[int(sender_id)] = np.array(
                    [
                        _safe_float(control.get("steering", 0.0), 0.0),
                        _safe_float(control.get("throttle", 0.0), 0.0),
                    ],
                    dtype=float,
                )
        self._ensure_fleet_capacity(int(sender_id))
        return super().add_received_local_state(sender_id, state, timestamp_ns)

    def update(
        self,
        local_state: np.ndarray,
        dt: float,
        current_time_ns: int,
        control: np.ndarray,
    ) -> np.ndarray:
        try:
            self._ensure_fleet_capacity(self.vehicle_id)
            local_vec = _normalize_state_array(local_state, self.state_dim, logger=self.logger)
            if local_vec is None:
                return self.fleet_states.copy()

            dt = float(np.clip(_safe_float(dt, self.min_dt), self.min_dt, self.max_dt))
            host_control = self._normalize_control(control)

            previous_states = self._seed_from_direct_states(
                self.fleet_states.copy(), local_vec, current_time_ns
            )
            if self.set_own_state_from_local and self.vehicle_id < self.fleet_size:
                previous_states[:, self.vehicle_id] = local_vec

            next_states = previous_states.copy()
            debug_rows = []

            for target_id in range(self.fleet_size):
                direct_state = self._direct_state_for_target(
                    target_id, local_vec, current_time_ns
                )

                if target_id == self.vehicle_id and self.set_own_state_from_local:
                    next_states[:, target_id] = local_vec
                    continue

                z_state = previous_states[:, target_id].copy()
                if np.allclose(z_state, 0.0) and direct_state is None:
                    seed = self._neighbor_average_state(target_id, current_time_ns)
                    if seed is not None:
                        z_state = seed

                target_control = self._control_for_target(target_id, host_control)
                z_pred = self._predict_bicycle(
                    z_state, target_control, dt, target_id=target_id
                )
                theta = self._theta_for_vehicle(target_id)
                measurement_term, innovation_norm = self._measurement_term(
                    target_id, z_pred, direct_state, theta
                )
                consensus_term, neighbor_count = self._consensus_term(
                    target_id, z_pred, theta, current_time_ns
                )

                z_next = z_pred + dt * (measurement_term + consensus_term)
                z_next = self._apply_state_constraints(z_next)

                if direct_state is not None and self.direct_state_blend > 0.0:
                    z_next = self._blend_states(z_next, direct_state, self.direct_state_blend)
                    z_next = self._apply_state_constraints(z_next)

                if not np.all(np.isfinite(z_next)):
                    z_next = z_state

                next_states[:, target_id] = z_next

                if self.debug_enabled:
                    debug_rows.append(
                        {
                            "target_id": target_id,
                            "innovation_norm": innovation_norm,
                            "neighbor_count": neighbor_count,
                            "measurement_norm": float(np.linalg.norm(measurement_term)),
                            "consensus_norm": float(np.linalg.norm(consensus_term)),
                        }
                    )

            if self.set_own_state_from_local and self.vehicle_id < self.fleet_size:
                next_states[:, self.vehicle_id] = local_vec

            self.fleet_states = next_states
            self._cleanup_old_data(current_time_ns)

            if self.debug_enabled:
                self.debug_data = {
                    "timestamp_ns": current_time_ns,
                    "dt": dt,
                    "z_dim": self.z_dim,
                    "y_dim": self.z_dim,
                    "rows": debug_rows,
                }

            return self.fleet_states.copy()
        except Exception as exc:
            if self.logger is not None and hasattr(self.logger, "log_error"):
                self.logger.log_error("Distributed high-gain update error", exc)
            else:
                self._log("error", f"Distributed high-gain update error: {exc}")
            return self.fleet_states.copy()

    def _seed_from_direct_states(
        self,
        states: np.ndarray,
        local_vec: np.ndarray,
        current_time_ns: int,
    ) -> np.ndarray:
        seeded = states.copy()
        for target_id in range(self.fleet_size):
            direct = self._direct_state_for_target(target_id, local_vec, current_time_ns)
            if direct is None:
                continue
            if np.allclose(seeded[:, target_id], 0.0) and not np.allclose(direct, 0.0):
                seeded[:, target_id] = direct
        return seeded

    def _direct_state_for_target(
        self,
        target_id: int,
        local_vec: np.ndarray,
        current_time_ns: int,
    ) -> Optional[np.ndarray]:
        if target_id == self.vehicle_id:
            return local_vec.copy()
        return self._get_latest_received_state(target_id, current_time_ns)

    def _measurement_term(
        self,
        target_id: int,
        z_state: np.ndarray,
        y_measured: Optional[np.ndarray],
        theta: float,
    ) -> Tuple[np.ndarray, float]:
        if y_measured is None:
            return np.zeros(self.z_dim), 0.0

        innovation = _state_error(y_measured[: self.z_dim], z_state[: self.z_dim])
        innovation = _clip_norm(innovation, self.max_innovation_norm)
        gain = np.diag(self._gain_vector_for_vehicle(target_id))
        correction = self._dilation_matrix(theta) @ gain @ innovation
        correction = _clip_norm(correction, self.max_measurement_term_norm)
        return correction, float(np.linalg.norm(innovation))

    def _consensus_term(
        self,
        target_id: int,
        z_state: np.ndarray,
        theta: float,
        current_time_ns: int,
    ) -> Tuple[np.ndarray, int]:
        if self.vehicle_id >= self.adjacency_matrix.shape[0]:
            return np.zeros(self.z_dim), 0

        accum = np.zeros(self.z_dim, dtype=float)
        weight_sum = 0.0
        neighbor_count = 0

        row = self.adjacency_matrix[self.vehicle_id]
        for neighbor_id, weight in enumerate(row):
            if neighbor_id == self.vehicle_id or abs(weight) <= 1e-12:
                continue
            neighbor_state = self._neighbor_state(neighbor_id, target_id, current_time_ns)
            if neighbor_state is None:
                continue
            accum += float(weight) * _state_error(neighbor_state[: self.z_dim], z_state[: self.z_dim])
            weight_sum += abs(float(weight))
            neighbor_count += 1

        if neighbor_count == 0:
            return np.zeros(self.z_dim), 0

        if self.normalize_consensus_by_weight and weight_sum > 1e-12:
            accum /= weight_sum

        r_i = 1.0
        if self.vehicle_id < self.row_vector_r.size:
            r_i = float(self.row_vector_r[self.vehicle_id])

        correction = self.consensus_gain * r_i * (self._consensus_transform(theta) @ accum)
        correction = _clip_norm(correction, self.max_consensus_term_norm)
        return correction, neighbor_count

    def _neighbor_state(
        self,
        neighbor_id: int,
        target_id: int,
        current_time_ns: int,
    ) -> Optional[np.ndarray]:
        fleet_dict = self._get_latest_fleet_data(neighbor_id, current_time_ns)
        if not fleet_dict:
            return None

        payload = None
        if target_id in fleet_dict:
            payload = fleet_dict[target_id]
        elif str(target_id) in fleet_dict:
            payload = fleet_dict[str(target_id)]
        if payload is None:
            return None
        return self._payload_to_state_array(payload)

    def _neighbor_average_state(
        self, target_id: int, current_time_ns: int
    ) -> Optional[np.ndarray]:
        samples = []
        for neighbor_id in self.received_fleet_states.keys():
            state = self._neighbor_state(int(neighbor_id), target_id, current_time_ns)
            if state is not None and np.any(np.isfinite(state)):
                samples.append(state)
        if not samples:
            return None
        return np.mean(np.vstack(samples), axis=0)

    def _payload_to_state_array(self, payload: Any) -> Optional[np.ndarray]:
        if isinstance(payload, dict):
            return _state_dict_to_array(payload, self.state_dim, logger=self.logger)
        return _normalize_state_array(payload, self.state_dim, logger=self.logger)

    def _normalize_control(self, control: Any) -> np.ndarray:
        if isinstance(control, dict):
            return np.array(
                [
                    _safe_float(control.get("steering", 0.0), 0.0),
                    _safe_float(control.get("throttle", 0.0), 0.0),
                ],
                dtype=float,
            )
        try:
            arr = np.asarray(control, dtype=float).flatten()
            if arr.size >= 2:
                return arr[:2]
            if arr.size == 1:
                return np.array([0.0, arr[0]], dtype=float)
        except (TypeError, ValueError):
            pass
        return np.zeros(2, dtype=float)

    def _control_for_target(self, target_id: int, host_control: np.ndarray) -> np.ndarray:
        if target_id == self.vehicle_id:
            return host_control
        cached = self._received_control_inputs.get(target_id)
        if cached is not None:
            return cached.copy()
        return host_control.copy()

    def _predict_bicycle(
        self,
        state: np.ndarray,
        control: np.ndarray,
        dt: float,
        target_id: int = -1,
    ) -> np.ndarray:
        x, y_pos, theta, velocity = state[:4]
        acceleration = state[4] if state.size > ACCELERATION_INDEX else 0.0

        steering = float(control[0]) if control.size > 0 else 0.0
        throttle = float(control[1]) if control.size > 1 else 0.0
        steering = float(np.clip(steering, -self.max_abs_steering, self.max_abs_steering))
        throttle = float(np.clip(throttle, -self.max_abs_throttle, self.max_abs_throttle))

        wheelbase = max(abs(self.wheelbase), 1e-6)
        x_new = x + velocity * np.cos(theta) * dt
        y_new = y_pos + velocity * np.sin(theta) * dt
        theta_new = theta + (velocity * np.tan(steering) / wheelbase) * dt

        if self.motor_enabled and dt > 0.0:
            velocity_new, acceleration_new, motor_accel = self._predict_motor_accel(
                throttle=throttle,
                velocity=velocity,
                motor_accel=self._motor_accel_state.get(target_id, 0.0),
                dt=dt,
            )
            self._motor_accel_state[target_id] = motor_accel
        else:
            acceleration_new = acceleration
            velocity_new = velocity + acceleration * dt

        return np.array([x_new, y_new, theta_new, velocity_new, acceleration_new], dtype=float)

    def _predict_motor_accel(
        self,
        throttle: float,
        velocity: float,
        motor_accel: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        u_th = max(throttle, 0.0)
        u_br = max(-throttle, 0.0)
        motor_accel_req = self.k_th * u_th - self.k_br * u_br
        motor_accel_new = motor_accel + (dt / self.motor_tau) * (
            motor_accel_req - motor_accel
        )
        smooth_sign = np.tanh(velocity / 0.02)
        drag_accel = self.c_v * velocity + self.c_fric * smooth_sign
        acceleration_new = motor_accel_new - drag_accel
        velocity_new = velocity + acceleration_new * dt
        return float(velocity_new), float(acceleration_new), float(motor_accel_new)

    def _apply_state_constraints(self, state: np.ndarray) -> np.ndarray:
        constrained = state.copy()
        if constrained.size > X_INDEX:
            constrained[X_INDEX] = np.clip(
                constrained[X_INDEX], -self.max_abs_position, self.max_abs_position
            )
        if constrained.size > Y_INDEX:
            constrained[Y_INDEX] = np.clip(
                constrained[Y_INDEX], -self.max_abs_position, self.max_abs_position
            )
        if constrained.size > THETA_INDEX:
            constrained[THETA_INDEX] = _wrap_angle(constrained[THETA_INDEX])
        if constrained.size > VELOCITY_INDEX:
            constrained[VELOCITY_INDEX] = np.clip(
                constrained[VELOCITY_INDEX],
                -self.max_abs_velocity,
                self.max_abs_velocity,
            )
        if constrained.size > ACCELERATION_INDEX:
            constrained[ACCELERATION_INDEX] = np.clip(
                constrained[ACCELERATION_INDEX],
                -self.max_abs_acceleration,
                self.max_abs_acceleration,
            )
        return constrained

    def _blend_states(
        self, base_state: np.ndarray, measured_state: np.ndarray, alpha: float
    ) -> np.ndarray:
        blended = (1.0 - alpha) * base_state + alpha * measured_state
        if blended.size > THETA_INDEX:
            blended[THETA_INDEX] = base_state[THETA_INDEX] + alpha * _wrap_angle(
                measured_state[THETA_INDEX] - base_state[THETA_INDEX]
            )
        return blended

    def get_debug_data(self) -> Dict[str, Any]:
        return copy.deepcopy(self.debug_data)
