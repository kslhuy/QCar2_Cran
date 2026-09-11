"""Native Trust/Distributed Observer ABI and shadow-mode parity checks."""

from __future__ import annotations

import ctypes
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np


_MAX_TRUST_LEVELS = 8


class _CranTrustConfig(ctypes.Structure):
    _fields_ = [
        ("num_trust_levels", ctypes.c_uint32),
        ("dirichlet_update_rate", ctypes.c_double),
        ("dirichlet_c", ctypes.c_double),
        ("dirichlet_wt_local", ctypes.c_double),
        ("dirichlet_wt_global", ctypes.c_double),
        ("ema_alpha", ctypes.c_double),
        ("trust_threshold", ctypes.c_double),
        ("sudden_change_threshold", ctypes.c_double),
        ("attack_detection_window", ctypes.c_uint32),
        ("dirichlet_method", ctypes.c_uint8),
        ("dirichlet_dual", ctypes.c_uint8),
        ("monitor_sudden_change", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]


class _CranTrustResult(ctypes.Structure):
    _fields_ = [
        ("final_score", ctypes.c_double),
        ("trust_levels", ctypes.c_double * _MAX_TRUST_LEVELS),
        ("trust_level_count", ctypes.c_uint32),
        ("flag_target_attack", ctypes.c_uint8),
        ("flag_global_est_check", ctypes.c_uint8),
        ("flag_local_est_check", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]


class _CranWeightConfig(ctypes.Structure):
    _fields_ = [
        ("w0_fixed", ctypes.c_double),
        ("w_self_base", ctypes.c_double),
        ("w_cap", ctypes.c_double),
        ("trust_threshold", ctypes.c_double),
        ("gamma_self_weight_floor", ctypes.c_double),
        ("flag_w0_target_attack_factor", ctypes.c_double),
        ("flag_w0_global_est_check_factor", ctypes.c_double),
        ("flag_w0_local_est_check_factor", ctypes.c_double),
        ("local_bad_zero_w0_neighbor_total_cap", ctypes.c_double),
        ("kappa", ctypes.c_uint32),
        ("use_gamma_self_weight_adaptation", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class _CranWeightInput(ctypes.Structure):
    _fields_ = [
        ("target_local_trust", ctypes.c_double),
        ("gamma_self", ctypes.c_double),
        ("direct_recovery_scale", ctypes.c_double),
        ("neighbor_count", ctypes.c_uint32),
        ("mode", ctypes.c_uint8),
        ("direct_available", ctypes.c_uint8),
        ("flag_target_attack", ctypes.c_uint8),
        ("flag_global_est_check", ctypes.c_uint8),
        ("flag_local_est_check", ctypes.c_uint8),
        ("has_target_trust", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
    ]


class _CranWeightResult(ctypes.Structure):
    _fields_ = [
        ("direct_weight", ctypes.c_double),
        ("self_weight", ctypes.c_double),
        ("total_neighbor_weight", ctypes.c_double),
    ]


class _CranPredictionConfig(ctypes.Structure):
    _fields_ = [
        ("dt", ctypes.c_double),
        ("steering", ctypes.c_double),
        ("throttle", ctypes.c_double),
        ("wheelbase", ctypes.c_double),
        ("max_velocity", ctypes.c_double),
        ("max_acceleration", ctypes.c_double),
        ("max_steering", ctypes.c_double),
        ("velocity_lag_tau", ctypes.c_double),
        ("velocity_gain", ctypes.c_double),
        ("velocity_lag_deadband", ctypes.c_double),
        ("velocity_lag_lookup_tau", ctypes.c_double),
        ("velocity_command_tau", ctypes.c_double),
        ("accel_lag_tau", ctypes.c_double),
        ("accel_lag_gain", ctypes.c_double),
        ("anchor_position_weight", ctypes.c_double),
        ("estimate_position_weight", ctypes.c_double),
        ("clean_theta_weight", ctypes.c_double),
        ("host_theta_weight", ctypes.c_double),
        ("target_velocity_weight", ctypes.c_double),
        ("host_velocity_weight", ctypes.c_double),
        ("target_acceleration_weight", ctypes.c_double),
        ("host_acceleration_weight", ctypes.c_double),
        ("prediction_mode", ctypes.c_uint8),
        ("longitudinal_model", ctypes.c_uint8),
        ("has_control", ctypes.c_uint8),
        ("force_clean_pose_anchor", ctypes.c_uint8),
        ("attack_anchor_active", ctypes.c_uint8),
        ("has_clean_state", ctypes.c_uint8),
        ("has_anchor", ctypes.c_uint8),
        ("use_anchor_bearing", ctypes.c_uint8),
    ]


class _CranHostAnchor(ctypes.Structure):
    _fields_ = [
        ("host_x", ctypes.c_double),
        ("host_y", ctypes.c_double),
        ("host_theta", ctypes.c_double),
        ("host_velocity", ctypes.c_double),
        ("host_acceleration", ctypes.c_double),
        ("distance", ctypes.c_double),
        ("sign", ctypes.c_double),
        ("relative_x", ctypes.c_double),
        ("relative_y", ctypes.c_double),
    ]


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _native_library_candidates(library_path: Optional[str]) -> Iterable[Path]:
    package_dir = Path(__file__).resolve().parent
    qcar_dir = package_dir.parent
    if library_path:
        supplied = Path(str(library_path)).expanduser()
        if supplied.is_absolute():
            yield supplied
        else:
            yield Path.cwd() / supplied
            yield qcar_dir / supplied
            yield package_dir / supplied

    names = (
        "cran_electronics_core.dll",
        "cran_electronics_core.so",
        "cran_electronics_core.dylib",
    )
    for build_dir in (
        package_dir / "native" / "build",
        package_dir / "native" / "build" / "Release",
        package_dir / "native" / "build" / "Debug",
    ):
        for name in names:
            yield build_dir / name


def discover_native_library(library_path: Optional[str] = None) -> Path:
    """Resolve an explicit path or the standard in-package native build."""
    checked = []
    for candidate in _native_library_candidates(library_path):
        resolved = candidate.resolve()
        if resolved in checked:
            continue
        checked.append(resolved)
        if resolved.is_file():
            return resolved
    checked_text = ", ".join(str(item) for item in checked)
    raise FileNotFoundError(f"CRAN native core was not found; checked: {checked_text}")


class NativeTrustObserverCore:
    """Thin ctypes binding around the platform-neutral C++ algorithm core."""

    def __init__(
        self,
        *,
        library_path: Optional[str],
        trust_config: Mapping[str, Any],
    ) -> None:
        self.library_path = discover_native_library(library_path)
        self._library = ctypes.CDLL(str(self.library_path))
        self._bind_functions()
        config = self._build_trust_config(trust_config)
        self._trust_handle = self._library.cran_trust_create(ctypes.byref(config))
        if not self._trust_handle:
            raise RuntimeError("cran_trust_create returned a null handle")

    def _bind_functions(self) -> None:
        lib = self._library
        double_ptr = ctypes.POINTER(ctypes.c_double)
        lib.cran_trust_create.argtypes = [ctypes.POINTER(_CranTrustConfig)]
        lib.cran_trust_create.restype = ctypes.c_void_p
        lib.cran_trust_destroy.argtypes = [ctypes.c_void_p]
        lib.cran_trust_reset.argtypes = [ctypes.c_void_p]
        lib.cran_trust_local_sample.argtypes = [double_ptr]
        lib.cran_trust_local_sample.restype = ctypes.c_double
        lib.cran_trust_step.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.POINTER(_CranTrustResult),
        ]
        lib.cran_trust_step.restype = ctypes.c_int
        lib.cran_observer_correct.argtypes = [
            double_ptr,
            ctypes.c_size_t,
            double_ptr,
            ctypes.c_double,
            double_ptr,
            double_ptr,
            ctypes.c_size_t,
            ctypes.c_double,
            ctypes.c_double,
            double_ptr,
        ]
        lib.cran_observer_correct.restype = ctypes.c_int
        lib.cran_observer_weights.argtypes = [
            ctypes.POINTER(_CranWeightConfig),
            ctypes.POINTER(_CranWeightInput),
            ctypes.POINTER(ctypes.c_uint32),
            double_ptr,
            double_ptr,
            ctypes.POINTER(_CranWeightResult),
        ]
        lib.cran_observer_weights.restype = ctypes.c_int
        lib.cran_observer_predict.argtypes = [
            double_ptr,
            ctypes.c_size_t,
            double_ptr,
            ctypes.POINTER(_CranPredictionConfig),
            ctypes.POINTER(_CranHostAnchor),
            double_ptr,
            double_ptr,
            ctypes.c_size_t,
            double_ptr,
        ]
        lib.cran_observer_predict.restype = ctypes.c_int

    @staticmethod
    def _build_trust_config(values: Mapping[str, Any]) -> _CranTrustConfig:
        levels = int(values.get("num_trust_levels", 5))
        if not 2 <= levels <= _MAX_TRUST_LEVELS:
            raise ValueError(
                f"num_trust_levels must be in [2, {_MAX_TRUST_LEVELS}]"
            )
        method = str(values.get("dirichlet_method", "ema")).strip().lower()
        dual = str(values.get("dirichlet_type", "Dual")).strip().lower() == "dual"
        return _CranTrustConfig(
            num_trust_levels=levels,
            dirichlet_update_rate=float(values.get("dirichlet_update_rate", 0.1)),
            dirichlet_c=float(values.get("dirichlet_C", 0.2)),
            dirichlet_wt_local=float(values.get("dirichlet_wt_local", 0.4)),
            dirichlet_wt_global=float(values.get("dirichlet_wt_global", 0.5)),
            ema_alpha=float(values.get("ema_alpha", 0.3)),
            trust_threshold=float(values.get("trust_threshold", 0.5)),
            sudden_change_threshold=float(
                values.get("sudden_change_threshold", 0.5)
            ),
            attack_detection_window=max(
                2, int(values.get("attack_detection_window", 10))
            ),
            dirichlet_method=1 if method == "matlab" else 0,
            dirichlet_dual=int(dual),
            monitor_sudden_change=int(
                _as_bool(values.get("monitor_sudden_change", False))
            ),
            reserved=0,
        )

    def close(self) -> None:
        handle = getattr(self, "_trust_handle", None)
        if handle:
            self._library.cran_trust_destroy(handle)
            self._trust_handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def reset(self) -> None:
        self._require_handle()
        self._library.cran_trust_reset(self._trust_handle)

    def local_trust_sample(self, scores: Sequence[float]) -> float:
        if len(scores) != 6:
            raise ValueError("local trust requires exactly six component scores")
        values = (ctypes.c_double * 6)(*[float(value) for value in scores])
        return float(self._library.cran_trust_local_sample(values))

    def trust_step(
        self,
        *,
        target_id: int,
        local_trust_sample: float,
        global_trust_sample: float,
        missing_observation: bool = False,
    ) -> Dict[str, Any]:
        self._require_handle()
        result = _CranTrustResult()
        code = int(
            self._library.cran_trust_step(
                self._trust_handle,
                ctypes.c_uint32(int(target_id)),
                ctypes.c_double(float(local_trust_sample)),
                ctypes.c_double(float(global_trust_sample)),
                ctypes.c_uint8(int(bool(missing_observation))),
                ctypes.byref(result),
            )
        )
        if code != 0:
            raise RuntimeError(f"cran_trust_step failed with code {code}")
        count = int(result.trust_level_count)
        return {
            "final_score": float(result.final_score),
            "trust_levels": np.asarray(result.trust_levels[:count], dtype=float),
            "flag_target_attack": bool(result.flag_target_attack),
            "flag_global_est_check": bool(result.flag_global_est_check),
            "flag_local_est_check": bool(result.flag_local_est_check),
        }

    def observer_correct(
        self,
        *,
        current_state: Sequence[float],
        direct_state: Optional[Sequence[float]],
        direct_weight: float,
        neighbor_states: Sequence[Tuple[int, Sequence[float]]],
        neighbor_weights: Sequence[float],
        max_velocity: float,
        max_acceleration: float,
    ) -> np.ndarray:
        current = np.ascontiguousarray(current_state, dtype=np.float64).reshape(-1)
        state_dim = int(current.size)
        if state_dim <= 0:
            raise ValueError("current_state cannot be empty")
        direct = None
        direct_ptr = None
        if direct_state is not None:
            direct = np.ascontiguousarray(direct_state, dtype=np.float64).reshape(-1)
            if direct.size != state_dim:
                raise ValueError("direct_state dimension does not match current_state")
            direct_ptr = direct.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

        neighbors = np.ascontiguousarray(neighbor_states, dtype=np.float64)
        weights = np.ascontiguousarray(neighbor_weights, dtype=np.float64).reshape(-1)
        if weights.size == 0:
            neighbors = np.empty((0, state_dim), dtype=np.float64)
        else:
            neighbors = neighbors.reshape((-1, state_dim))
        if neighbors.shape[0] != weights.size:
            raise ValueError("neighbor state/weight counts differ")

        output = np.empty(state_dim, dtype=np.float64)
        code = int(
            self._library.cran_observer_correct(
                current.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                ctypes.c_size_t(state_dim),
                direct_ptr,
                ctypes.c_double(float(direct_weight)),
                None
                if weights.size == 0
                else neighbors.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                None
                if weights.size == 0
                else weights.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                ctypes.c_size_t(weights.size),
                ctypes.c_double(float(max_velocity)),
                ctypes.c_double(float(max_acceleration)),
                output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            )
        )
        if code != 0:
            raise RuntimeError(f"cran_observer_correct failed with code {code}")
        return output

    def observer_weights(
        self,
        *,
        mode: str,
        weight_config: Any,
        neighbor_ids: Sequence[int],
        neighbor_trust_scores: Sequence[float],
        direct_available: bool,
        target_trust: Any = None,
        target_local_trust: Optional[float] = None,
        direct_recovery_scale: float = 1.0,
    ) -> Dict[str, Any]:
        """Calculate one target row of adaptive observer weights in C++."""
        if len(neighbor_ids) != len(neighbor_trust_scores):
            raise ValueError("neighbor ID/trust counts differ")

        def cfg(name: str, default: Any) -> Any:
            if isinstance(weight_config, Mapping):
                return weight_config.get(name, default)
            return getattr(weight_config, name, default)

        mode_value = {
            "startup": 0,
            "equal": 1,
            "trust_based": 2,
            "paper": 3,
        }.get(str(mode).strip().lower(), 2)
        if target_local_trust is None:
            target_local_trust = (
                float(getattr(target_trust, "local_trust_sample", 1.0))
                if target_trust is not None
                else 1.0
            )
        native_config = _CranWeightConfig(
            w0_fixed=float(cfg("w0_fixed", 0.3)),
            w_self_base=float(cfg("w_self_base", 0.2)),
            w_cap=float(cfg("w_cap", 0.4)),
            trust_threshold=float(cfg("trust_threshold", 0.5)),
            gamma_self_weight_floor=float(
                cfg("gamma_self_weight_floor", 0.25)
            ),
            flag_w0_target_attack_factor=float(
                cfg("flag_w0_target_attack_factor", 0.25)
            ),
            flag_w0_global_est_check_factor=float(
                cfg("flag_w0_global_est_check_factor", 1.25)
            ),
            flag_w0_local_est_check_factor=float(
                cfg("flag_w0_local_est_check_factor", 0.5)
            ),
            local_bad_zero_w0_neighbor_total_cap=float(
                cfg("local_bad_zero_w0_neighbor_total_cap", 0.01)
            ),
            kappa=max(0, int(cfg("kappa", 5))),
            use_gamma_self_weight_adaptation=int(
                _as_bool(cfg("use_gamma_self_weight_adaptation", True))
            ),
            reserved=(ctypes.c_uint8 * 3)(0, 0, 0),
        )
        native_input = _CranWeightInput(
            target_local_trust=float(target_local_trust),
            gamma_self=float(
                getattr(target_trust, "gamma_self", 1.0)
                if target_trust is not None
                else 1.0
            ),
            direct_recovery_scale=float(direct_recovery_scale),
            neighbor_count=len(neighbor_ids),
            mode=mode_value,
            direct_available=int(bool(direct_available)),
            flag_target_attack=int(
                bool(getattr(target_trust, "flag_target_attack", False))
            ),
            flag_global_est_check=int(
                bool(getattr(target_trust, "flag_global_est_check", False))
            ),
            flag_local_est_check=int(
                bool(getattr(target_trust, "flag_local_est_check", False))
            ),
            has_target_trust=int(target_trust is not None),
            reserved=(ctypes.c_uint8 * 2)(0, 0),
        )
        count = len(neighbor_ids)
        id_values = (
            (ctypes.c_uint32 * count)(*[int(value) for value in neighbor_ids])
            if count
            else None
        )
        trust_values = (
            (ctypes.c_double * count)(
                *[float(value) for value in neighbor_trust_scores]
            )
            if count
            else None
        )
        output_values = (ctypes.c_double * count)() if count else None
        result = _CranWeightResult()
        code = int(
            self._library.cran_observer_weights(
                ctypes.byref(native_config),
                ctypes.byref(native_input),
                id_values,
                trust_values,
                output_values,
                ctypes.byref(result),
            )
        )
        if code != 0:
            raise RuntimeError(f"cran_observer_weights failed with code {code}")
        return {
            "w0": float(result.direct_weight),
            "w_self": float(result.self_weight),
            "neighbors": {
                int(neighbor_ids[index]): float(output_values[index])
                for index in range(count)
            },
        }

    def observer_predict(
        self,
        *,
        corrected_state: Sequence[float],
        clean_state: Optional[Sequence[float]],
        control: Optional[Sequence[float]],
        dt: float,
        prediction_mode: str,
        model_config: Mapping[str, Any],
        prediction_options: Mapping[str, Any],
        force_clean_pose_anchor: bool,
        attack_anchor_active: bool,
        host_anchor: Optional[Mapping[str, Any]],
    ) -> np.ndarray:
        """Propagate a corrected target state with the embedded C++ model."""
        state = np.ascontiguousarray(corrected_state, dtype=np.float64).reshape(-1)
        if state.size < 4:
            raise ValueError("prediction requires at least four state values")
        clean = None
        clean_ptr = None
        if clean_state is not None:
            clean = np.ascontiguousarray(clean_state, dtype=np.float64).reshape(-1)
            if clean.size != state.size:
                raise ValueError("clean_state dimension does not match corrected_state")
            clean_ptr = clean.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

        normalized_mode = str(prediction_mode).strip().lower()
        mode_value = {
            "none": 0,
            "model": 1,
            "dead_reckoning": 2,
            "clean_data": 3,
            "mixed_clean_data": 4,
            "relative_host_anchor_mixed": 4,
        }.get(normalized_mode, 1)
        longitudinal_value = {
            "constant_velocity": 0,
            "velocity_lag": 1,
            "velocity_lag_lookup": 2,
            "velocity_command": 3,
            "acceleration_lag": 4,
            "simple_acceleration": 5,
        }.get(str(model_config.get("longitudinal_model", "constant_velocity")).lower(), 0)
        steering = float(control[0]) if control is not None and len(control) > 0 else 0.0
        throttle = float(control[1]) if control is not None and len(control) > 1 else 0.0

        native_config = _CranPredictionConfig(
            dt=float(dt),
            steering=steering,
            throttle=throttle,
            wheelbase=float(model_config.get("wheelbase", 0.256)),
            max_velocity=float(model_config.get("max_velocity", 2.0)),
            max_acceleration=float(model_config.get("max_acceleration", 5.0)),
            max_steering=float(model_config.get("max_steering", 0.5)),
            velocity_lag_tau=float(model_config.get("velocity_lag_tau", 0.301)),
            velocity_gain=float(model_config.get("velocity_gain", 6.598)),
            velocity_lag_deadband=float(
                model_config.get("velocity_lag_deadband", 0.0)
            ),
            velocity_lag_lookup_tau=float(
                model_config.get("velocity_lag_lookup_tau", 0.301)
            ),
            velocity_command_tau=float(
                model_config.get("velocity_command_tau", 0.301)
            ),
            accel_lag_tau=float(model_config.get("accel_lag_tau", 0.318)),
            accel_lag_gain=float(model_config.get("accel_lag_gain", 1.0)),
            anchor_position_weight=float(
                prediction_options.get("anchor_position_weight", 1.0)
            ),
            estimate_position_weight=float(
                prediction_options.get("estimate_position_weight", 0.0)
            ),
            clean_theta_weight=float(
                prediction_options.get("clean_theta_weight", 0.8)
            ),
            host_theta_weight=float(
                prediction_options.get("host_theta_weight", 0.2)
            ),
            target_velocity_weight=float(
                prediction_options.get("target_velocity_weight", 0.1)
            ),
            host_velocity_weight=float(
                prediction_options.get("host_velocity_weight", 0.9)
            ),
            target_acceleration_weight=float(
                prediction_options.get("target_acceleration_weight", 0.1)
            ),
            host_acceleration_weight=float(
                prediction_options.get("host_acceleration_weight", 0.9)
            ),
            prediction_mode=mode_value,
            longitudinal_model=longitudinal_value,
            has_control=int(control is not None),
            force_clean_pose_anchor=int(bool(force_clean_pose_anchor)),
            attack_anchor_active=int(bool(attack_anchor_active)),
            has_clean_state=int(clean is not None),
            has_anchor=int(host_anchor is not None),
            use_anchor_bearing=int(
                _as_bool(prediction_options.get("use_anchor_bearing", True), True)
            ),
        )

        native_anchor = None
        anchor_ptr = None
        if host_anchor is not None:
            native_anchor = _CranHostAnchor(
                host_x=float(host_anchor.get("host_x", state[0])),
                host_y=float(host_anchor.get("host_y", state[1])),
                host_theta=float(host_anchor.get("host_theta", state[2])),
                host_velocity=float(host_anchor.get("host_velocity", state[3])),
                host_acceleration=float(
                    host_anchor.get("host_acceleration", state[4] if state.size > 4 else 0.0)
                ),
                distance=float(host_anchor.get("distance", 0.1)),
                sign=float(host_anchor.get("sign", 1.0)),
                relative_x=float(host_anchor.get("relative_x", float("nan"))),
                relative_y=float(host_anchor.get("relative_y", float("nan"))),
            )
            anchor_ptr = ctypes.byref(native_anchor)

        throttle_points = np.ascontiguousarray(
            model_config.get("velocity_lag_lookup_throttle_breakpoints", []),
            dtype=np.float64,
        ).reshape(-1)
        velocity_points = np.ascontiguousarray(
            model_config.get("velocity_lag_lookup_velocity_breakpoints", []),
            dtype=np.float64,
        ).reshape(-1)
        lookup_count = (
            int(throttle_points.size)
            if throttle_points.size == velocity_points.size
            else 0
        )
        output = np.empty(state.size, dtype=np.float64)
        code = int(
            self._library.cran_observer_predict(
                state.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                ctypes.c_size_t(state.size),
                clean_ptr,
                ctypes.byref(native_config),
                anchor_ptr,
                None
                if lookup_count == 0
                else throttle_points.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                None
                if lookup_count == 0
                else velocity_points.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                ctypes.c_size_t(lookup_count),
                output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            )
        )
        if code != 0:
            raise RuntimeError(f"cran_observer_predict failed with code {code}")
        return output

    def _require_handle(self) -> None:
        if not getattr(self, "_trust_handle", None):
            raise RuntimeError("Native Trust core is closed")


class TrustObserverShadow:
    """Compare native embedded math and safely gate native authority."""

    def __init__(
        self,
        config: Optional[Mapping[str, Any]],
        trust_config: Mapping[str, Any],
    ) -> None:
        values = dict(config or {})
        self.enabled = _as_bool(values.get("enabled", False))
        configured_mode = str(values.get("mode", "shadow")).strip().lower()
        if configured_mode not in {"shadow", "native_authority"}:
            raise ValueError(
                "embedded_core.mode must be 'shadow' or 'native_authority'"
            )
        # Native authority can never be granted at construction time because no
        # live parity evidence exists yet.  A configured authority request is
        # recorded, but must be explicitly re-issued after the gate is ready.
        self.mode = "shadow"
        self.configured_mode = configured_mode
        self.score_tolerance = max(
            0.0, float(values.get("score_tolerance", 1e-9))
        )
        self.state_tolerance = max(
            0.0, float(values.get("state_tolerance", 1e-9))
        )
        self.weight_tolerance = max(
            0.0, float(values.get("weight_tolerance", 1e-9))
        )
        self.prediction_tolerance = max(
            0.0, float(values.get("prediction_tolerance", 1e-9))
        )
        self.authority_min_comparisons_per_stage = max(
            1, int(values.get("authority_min_comparisons_per_stage", 1000))
        )
        self.core: Optional[NativeTrustObserverCore] = None
        self.available = False
        self.reason = "disabled"
        self.trust_comparisons = 0
        self.weight_comparisons = 0
        self.observer_comparisons = 0
        self.prediction_comparisons = 0
        self.trust_failures = 0
        self.weight_failures = 0
        self.observer_failures = 0
        self.prediction_failures = 0
        self.max_trust_error = 0.0
        self.max_weight_error = 0.0
        self.max_state_error = 0.0
        self.max_prediction_error = 0.0
        self.last_trust_error = 0.0
        self.last_weight_error = 0.0
        self.last_state_error = 0.0
        self.last_prediction_error = 0.0
        self.native_authority_uses = 0
        self.native_trust_uses = 0
        self.native_weight_uses = 0
        self.native_correction_uses = 0
        self.native_prediction_uses = 0
        self.failback_count = 0
        self.last_failback_stage: Optional[str] = None
        self.last_failback_reason: Optional[str] = None
        self.last_mode_change_reason = (
            "startup safety hold"
            if configured_mode == "native_authority"
            else "configured shadow mode"
        )

        if not self.enabled:
            return
        try:
            self.core = NativeTrustObserverCore(
                library_path=values.get("library_path"),
                trust_config=trust_config,
            )
            self.available = True
            self.reason = "ready"
        except Exception as exc:
            self.reason = f"{type(exc).__name__}: {exc}"

    def close(self) -> None:
        if self.core is not None:
            self.core.close()
            self.core = None
        self.available = False

    def set_mode(self, mode: str) -> Dict[str, Any]:
        """Request shadow/native authority without bypassing the parity gate."""
        requested = str(mode).strip().lower()
        if requested not in {"shadow", "native_authority"}:
            raise ValueError(
                "embedded core mode must be 'shadow' or 'native_authority'"
            )
        if requested == "shadow":
            self.mode = "shadow"
            self.last_mode_change_reason = "operator selected shadow mode"
            status = self.get_status()
            status["mode_request_accepted"] = True
            return status

        status = self.get_status()
        if not status["authority_ready"]:
            self.mode = "shadow"
            self.last_mode_change_reason = "native authority rejected by parity gate"
            status = self.get_status()
            status["mode_request_accepted"] = False
            return status

        self.mode = "native_authority"
        self.last_mode_change_reason = "operator enabled gated native authority"
        status = self.get_status()
        status["mode_request_accepted"] = True
        return status

    def should_use_native(self, comparison: Optional[Mapping[str, Any]]) -> bool:
        """Return True only for a passing result while authority remains active."""
        accepted = bool(
            self.mode == "native_authority"
            and comparison is not None
            and comparison.get("passed", False)
            and self.available
        )
        if accepted:
            self.native_authority_uses += 1
            stage = str(comparison.get("stage", ""))
            field = {
                "trust": "native_trust_uses",
                "weight": "native_weight_uses",
                "correction": "native_correction_uses",
                "prediction": "native_prediction_uses",
            }.get(stage)
            if field is not None:
                setattr(self, field, getattr(self, field) + 1)
        return accepted

    def _trip_authority(self, stage: str, reason: str) -> None:
        """Fail safe to Python after an authority-stage mismatch or runtime fault."""
        if self.mode != "native_authority":
            return
        self.mode = "shadow"
        self.failback_count += 1
        self.last_failback_stage = str(stage)
        self.last_failback_reason = str(reason)
        self.last_mode_change_reason = f"automatic failback at {stage}: {reason}"

    def reset(self) -> None:
        if self.core is not None:
            self.core.reset()
        self.reset_parity_evidence()

    def reset_parity_evidence(self) -> None:
        """Clear gate evidence without desynchronizing native Trust memory."""
        self.trust_comparisons = 0
        self.weight_comparisons = 0
        self.observer_comparisons = 0
        self.prediction_comparisons = 0
        self.trust_failures = 0
        self.weight_failures = 0
        self.observer_failures = 0
        self.prediction_failures = 0
        self.max_trust_error = 0.0
        self.max_weight_error = 0.0
        self.max_state_error = 0.0
        self.max_prediction_error = 0.0
        self.last_trust_error = 0.0
        self.last_weight_error = 0.0
        self.last_state_error = 0.0
        self.last_prediction_error = 0.0
        self.native_authority_uses = 0
        self.native_trust_uses = 0
        self.native_weight_uses = 0
        self.native_correction_uses = 0
        self.native_prediction_uses = 0
        self.mode = "shadow"
        self.failback_count = 0
        self.last_failback_stage = None
        self.last_failback_reason = None
        self.last_mode_change_reason = "parity evidence reset; shadow safety hold"

    def compare_trust(
        self, *, target_id: int, python_trust: Any, missing_observation: bool
    ) -> Optional[Dict[str, Any]]:
        if not self.available or self.core is None:
            return None
        try:
            native = self.core.trust_step(
                target_id=target_id,
                local_trust_sample=float(python_trust.local_trust_sample),
                global_trust_sample=float(python_trust.global_trust_sample),
                missing_observation=missing_observation,
            )
            score_error = abs(
                native["final_score"] - float(python_trust.final_score)
            )
            level_error = float(
                np.max(
                    np.abs(
                        native["trust_levels"]
                        - np.asarray(python_trust.trust_levels, dtype=float)
                    )
                )
            )
            flags_match = all(
                native[name] == bool(getattr(python_trust, name))
                for name in (
                    "flag_target_attack",
                    "flag_global_est_check",
                    "flag_local_est_check",
                )
            )
            error = max(score_error, level_error)
        except Exception as exc:
            self._trip_authority("trust", f"runtime {type(exc).__name__}: {exc}")
            self.available = False
            self.reason = f"runtime {type(exc).__name__}: {exc}"
            return None
        self.trust_comparisons += 1
        self.last_trust_error = error
        self.max_trust_error = max(self.max_trust_error, error)
        passed = error <= self.score_tolerance and flags_match
        if not passed:
            self.trust_failures += 1
            self._trip_authority(
                "trust", f"parity error {error:.6g}; flags_match={flags_match}"
            )
        return {
            "stage": "trust",
            "passed": passed,
            "error": error,
            "flags_match": flags_match,
            "native_final_score": native["final_score"],
            "native_result": native,
        }

    def compare_weights(
        self,
        *,
        mode: str,
        weight_config: Any,
        neighbor_candidates: Sequence[Tuple[int, float]],
        direct_available: bool,
        target_trust: Any,
        target_local_trust: Optional[float],
        direct_recovery_scale: float,
        python_weights: Mapping[str, Any],
    ) -> Optional[Dict[str, Any]]:
        if not self.available or self.core is None:
            return None
        try:
            native = self.core.observer_weights(
                mode=mode,
                weight_config=weight_config,
                neighbor_ids=[item[0] for item in neighbor_candidates],
                neighbor_trust_scores=[item[1] for item in neighbor_candidates],
                direct_available=direct_available,
                target_trust=target_trust,
                target_local_trust=target_local_trust,
                direct_recovery_scale=direct_recovery_scale,
            )
            errors = [
                abs(float(native["w0"]) - float(python_weights.get("w0", 0.0))),
                abs(
                    float(native["w_self"])
                    - float(python_weights.get("w_self", 0.0))
                ),
            ]
            native_neighbors = dict(native.get("neighbors", {}))
            python_neighbors = dict(python_weights.get("neighbors", {}))
            for neighbor_id in set(native_neighbors) | set(python_neighbors):
                errors.append(
                    abs(
                        float(native_neighbors.get(neighbor_id, 0.0))
                        - float(python_neighbors.get(neighbor_id, 0.0))
                    )
                )
            error = max(errors) if errors else 0.0
        except Exception as exc:
            self._trip_authority("weight", f"runtime {type(exc).__name__}: {exc}")
            self.available = False
            self.reason = f"runtime {type(exc).__name__}: {exc}"
            return None
        self.weight_comparisons += 1
        self.last_weight_error = error
        self.max_weight_error = max(self.max_weight_error, error)
        passed = error <= self.weight_tolerance
        if not passed:
            self.weight_failures += 1
            self._trip_authority("weight", f"parity error {error:.6g}")
        return {
            "stage": "weight",
            "passed": passed,
            "error": error,
            "native_weights": native,
        }

    def compare_observer(
        self,
        *,
        current_state: Sequence[float],
        direct_state: Optional[Sequence[float]],
        neighbor_states: Sequence[Sequence[float]],
        target_weights: Mapping[str, Any],
        max_velocity: float,
        max_acceleration: float,
        python_corrected_state: Sequence[float],
    ) -> Optional[Dict[str, Any]]:
        if not self.available or self.core is None:
            return None
        try:
            neighbors_by_id = dict(target_weights.get("neighbors", {}))
            neighbor_weights = [
                float(neighbors_by_id[neighbor_id])
                for neighbor_id, _ in neighbor_states
            ]
            states = [state for _, state in neighbor_states]
            native = self.core.observer_correct(
                current_state=current_state,
                direct_state=direct_state,
                direct_weight=float(target_weights.get("w0", 0.0)),
                neighbor_states=states,
                neighbor_weights=neighbor_weights,
                max_velocity=max_velocity,
                max_acceleration=max_acceleration,
            )
            expected = np.asarray(python_corrected_state, dtype=float)
            error = float(np.max(np.abs(native - expected)))
        except Exception as exc:
            self._trip_authority(
                "correction", f"runtime {type(exc).__name__}: {exc}"
            )
            self.available = False
            self.reason = f"runtime {type(exc).__name__}: {exc}"
            return None
        self.observer_comparisons += 1
        self.last_state_error = error
        self.max_state_error = max(self.max_state_error, error)
        passed = error <= self.state_tolerance
        if not passed:
            self.observer_failures += 1
            self._trip_authority("correction", f"parity error {error:.6g}")
        return {
            "stage": "correction",
            "passed": passed,
            "error": error,
            "native_state": native,
        }

    def compare_prediction(
        self,
        *,
        corrected_state: Sequence[float],
        clean_state: Optional[Sequence[float]],
        control: Optional[Sequence[float]],
        dt: float,
        prediction_mode: str,
        model_config: Mapping[str, Any],
        prediction_options: Mapping[str, Any],
        force_clean_pose_anchor: bool,
        attack_anchor_active: bool,
        host_anchor: Optional[Mapping[str, Any]],
        python_predicted_state: Sequence[float],
    ) -> Optional[Dict[str, Any]]:
        if not self.available or self.core is None:
            return None
        try:
            native = self.core.observer_predict(
                corrected_state=corrected_state,
                clean_state=clean_state,
                control=control,
                dt=dt,
                prediction_mode=prediction_mode,
                model_config=model_config,
                prediction_options=prediction_options,
                force_clean_pose_anchor=force_clean_pose_anchor,
                attack_anchor_active=attack_anchor_active,
                host_anchor=host_anchor,
            )
            expected = np.asarray(python_predicted_state, dtype=float)
            error = float(np.max(np.abs(native - expected)))
        except Exception as exc:
            self._trip_authority(
                "prediction", f"runtime {type(exc).__name__}: {exc}"
            )
            self.available = False
            self.reason = f"runtime {type(exc).__name__}: {exc}"
            return None
        self.prediction_comparisons += 1
        self.last_prediction_error = error
        self.max_prediction_error = max(self.max_prediction_error, error)
        passed = error <= self.prediction_tolerance
        if not passed:
            self.prediction_failures += 1
            self._trip_authority("prediction", f"parity error {error:.6g}")
        return {
            "stage": "prediction",
            "passed": passed,
            "error": error,
            "native_state": native,
        }

    def get_status(self) -> Dict[str, Any]:
        stage_comparisons = {
            "trust": self.trust_comparisons,
            "weight": self.weight_comparisons,
            "correction": self.observer_comparisons,
            "prediction": self.prediction_comparisons,
        }
        stage_failures = {
            "trust": self.trust_failures,
            "weight": self.weight_failures,
            "correction": self.observer_failures,
            "prediction": self.prediction_failures,
        }
        comparisons = sum(stage_comparisons.values())
        failures = sum(stage_failures.values())
        parity_pass = bool(
            self.available
            and all(value > 0 for value in stage_comparisons.values())
            and failures == 0
        )
        authority_blockers = []
        if not self.available:
            authority_blockers.append("native core unavailable")
        for stage, count in stage_comparisons.items():
            if count < self.authority_min_comparisons_per_stage:
                authority_blockers.append(
                    f"{stage}: {count}/{self.authority_min_comparisons_per_stage} comparisons"
                )
            if stage_failures[stage] > 0:
                authority_blockers.append(
                    f"{stage}: {stage_failures[stage]} parity failures"
                )
        return {
            "enabled": self.enabled,
            "available": self.available,
            "mode": self.mode,
            "configured_mode": self.configured_mode,
            "authority_active": self.mode == "native_authority",
            "backend": "native_cpp",
            "reason": self.reason,
            "library": None
            if self.core is None
            else str(self.core.library_path),
            "parity_pass": parity_pass,
            "authority_ready": bool(parity_pass and not authority_blockers),
            "authority_blockers": authority_blockers,
            "authority_min_comparisons_per_stage": (
                self.authority_min_comparisons_per_stage
            ),
            "failback_count": self.failback_count,
            "last_failback_stage": self.last_failback_stage,
            "last_failback_reason": self.last_failback_reason,
            "last_mode_change_reason": self.last_mode_change_reason,
            "native_authority_uses": self.native_authority_uses,
            "native_trust_uses": self.native_trust_uses,
            "native_weight_uses": self.native_weight_uses,
            "native_correction_uses": self.native_correction_uses,
            "native_prediction_uses": self.native_prediction_uses,
            "comparisons": comparisons,
            "failures": failures,
            "trust_comparisons": self.trust_comparisons,
            "weight_comparisons": self.weight_comparisons,
            "observer_comparisons": self.observer_comparisons,
            "prediction_comparisons": self.prediction_comparisons,
            "trust_failures": self.trust_failures,
            "weight_failures": self.weight_failures,
            "observer_failures": self.observer_failures,
            "prediction_failures": self.prediction_failures,
            "max_trust_error": self.max_trust_error,
            "max_weight_error": self.max_weight_error,
            "max_state_error": self.max_state_error,
            "max_prediction_error": self.max_prediction_error,
            "last_trust_error": self.last_trust_error,
            "last_weight_error": self.last_weight_error,
            "last_state_error": self.last_state_error,
            "last_prediction_error": self.last_prediction_error,
            "score_tolerance": self.score_tolerance,
            "weight_tolerance": self.weight_tolerance,
            "state_tolerance": self.state_tolerance,
            "prediction_tolerance": self.prediction_tolerance,
        }
