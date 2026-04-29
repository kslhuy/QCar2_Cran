import json
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    from .qcar_heading_fusion import QCarHeadingFusion, QCarHeadingFusionConfig
except ImportError:
    from qcar_heading_fusion import QCarHeadingFusion, QCarHeadingFusionConfig


RAW_KEYS = (
    "ax",
    "ay",
    "wz",
    "delta",
    "vfl",
    "vfr",
    "vrl",
    "vrr",
    "throttle",
    "gps_valid",
    "theta_valid",
    "gps_hold_valid",
    "gps_age_sec",
)
TARGET_DIM = 4
MEAS_DIM = 4
_GPS_AGE_CAP_SEC = 1.0e3
_HEADING_FILTER_Q_PSI = 1.0e-4
_HEADING_FILTER_Q_BIAS = 1.0e-3
_HEADING_FILTER_R_GPS = 1.0e-3
_KIN_WHEELBASE = 0.2
_KIN_VELOCITY_MODEL = "tachometer"
_KIN_VELOCITY_TAU = 0.301
_KIN_VELOCITY_GAIN = 6.598
_KIN_MAX_VELOCITY = 2.0
_KIN_MAX_ACCELERATION = 2.0
_GPS_DROPOUT_XY_MODES = {"freeze", "dead_reckon"}


def _wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _sanitize_gps_age(age_value: Any) -> float:
    try:
        age = float(age_value)
    except (TypeError, ValueError):
        return _GPS_AGE_CAP_SEC
    if not np.isfinite(age):
        return _GPS_AGE_CAP_SEC
    return float(np.clip(age, 0.0, _GPS_AGE_CAP_SEC))


def _safe_dt(curr_ts: float, prev_ts: float) -> float:
    return max(float(curr_ts) - float(prev_ts), 1e-3)


def _dead_reckon_xy(
    x: float,
    y: float,
    heading: float,
    velocity: float,
    dt: float,
) -> Tuple[float, float]:
    dt_val = max(float(dt), 1e-3)
    if not np.isfinite(velocity):
        velocity = 0.0
    heading_val = _wrap_angle(float(heading))
    next_x = float(x) + float(velocity) * float(np.cos(heading_val)) * dt_val
    next_y = float(y) + float(velocity) * float(np.sin(heading_val)) * dt_val
    return float(next_x), float(next_y)


def _normalize_gps_dropout_xy_mode(mode: Any) -> str:
    mode_norm = str(mode or "freeze").strip().lower()
    alias_map = {
        "dead_reckoning": "dead_reckon",
        "dead-reckon": "dead_reckon",
        "dr": "dead_reckon",
    }
    mode_norm = alias_map.get(mode_norm, mode_norm)
    if mode_norm not in _GPS_DROPOUT_XY_MODES:
        raise ValueError(
            f"Unsupported gps_dropout_xy_mode '{mode}'. "
            f"Expected one of: {sorted(_GPS_DROPOUT_XY_MODES)}"
        )
    return mode_norm


def _resolve_heading_kinematic_config(config: Optional[Dict[str, Any]] = None) -> Dict[str, float]:
    cfg = dict(config or {})
    return {
        "wheelbase": max(float(cfg.get("wheelbase", _KIN_WHEELBASE)), 1e-6),
        "velocity_model": str(cfg.get("velocity_model", _KIN_VELOCITY_MODEL)).strip().lower() or _KIN_VELOCITY_MODEL,
        "velocity_tau": max(float(cfg.get("velocity_tau", _KIN_VELOCITY_TAU)), 1e-6),
        "velocity_gain": float(cfg.get("velocity_gain", _KIN_VELOCITY_GAIN)),
        "max_velocity": max(float(cfg.get("max_velocity", _KIN_MAX_VELOCITY)), 1e-6),
        "max_acceleration": max(float(cfg.get("max_acceleration", _KIN_MAX_ACCELERATION)), 1e-6),
    }


def _predict_kinematic_velocity(
    prev_v: float,
    motor_tach: float,
    throttle: float,
    ax: float,
    dt: float,
    cfg: Dict[str, float],
) -> float:
    dt_val = max(float(dt), 1e-6)
    prev_v_val = float(prev_v) if np.isfinite(prev_v) else 0.0
    motor_tach_val = float(motor_tach) if np.isfinite(motor_tach) else prev_v_val
    throttle_val = float(throttle) if np.isfinite(throttle) else 0.0
    ax_val = float(ax) if np.isfinite(ax) else 0.0

    model = str(cfg["velocity_model"]).strip().lower()
    if model == "imu_acceleration":
        a_pred = ax_val
        v_next = prev_v_val + a_pred * dt_val
    elif model == "velocity_lag":
        a_pred = (-prev_v_val + float(cfg["velocity_gain"]) * throttle_val) / float(cfg["velocity_tau"])
        v_next = prev_v_val + a_pred * dt_val
    elif model == "velocity_command":
        a_pred = (throttle_val - prev_v_val) / float(cfg["velocity_tau"])
        v_next = prev_v_val + a_pred * dt_val
    elif model == "simple_acceleration":
        a_pred = throttle_val
        v_next = prev_v_val + a_pred * dt_val
    else:
        max_acc = float(cfg["max_acceleration"])
        a_pred = np.clip((motor_tach_val - prev_v_val) / dt_val, -max_acc, max_acc)
        v_next = motor_tach_val

    max_acc = float(cfg["max_acceleration"])
    max_vel = float(cfg["max_velocity"])
    a_pred = float(np.clip(a_pred, -max_acc, max_acc))
    v_next = float(np.clip(v_next, -max_vel, max_vel))
    return v_next


def _make_qcar_heading_fusion_config(
    kinematic_config: Optional[Dict[str, Any]] = None,
) -> QCarHeadingFusionConfig:
    cfg = _resolve_heading_kinematic_config(kinematic_config)
    return QCarHeadingFusionConfig(
        wheelbase=float(cfg["wheelbase"]),
        q_kf_theta=_HEADING_FILTER_Q_PSI,
        q_kf_bias=_HEADING_FILTER_Q_BIAS,
        r_kf_gps_theta=_HEADING_FILTER_R_GPS,
    )


def _rebuild_heading_measurements_gyro_filter(dataset: Dict[str, Any], timestamps: np.ndarray) -> np.ndarray:
    sample_count = int(timestamps.shape[0])
    if sample_count == 0:
        return np.zeros(0, dtype=np.float32)

    z = np.asarray(dataset["z"], dtype=np.float32)
    gps_valid = np.asarray(_dataset_series(dataset, "gps_valid", timestamps), dtype=np.float32).reshape(-1)
    gps_theta = np.asarray(dataset.get("gps_theta", z[:, 2]), dtype=np.float32).reshape(-1)
    gyro_z = np.asarray(
        dataset.get(
            "gyro_z",
            dataset.get("wz", np.zeros(sample_count, dtype=np.float32)),
        ),
        dtype=np.float32,
    ).reshape(-1)
    x_gt = np.asarray(dataset.get("x_gt", z), dtype=np.float32)

    if gps_valid[0] > 0.5:
        psi0 = float(gps_theta[0])
    elif x_gt.ndim == 2 and x_gt.shape[0] > 0 and x_gt.shape[1] > 2:
        psi0 = float(x_gt[0, 2])
    else:
        psi0 = float(z[0, 2])

    state = np.array([_wrap_angle(psi0), 0.0], dtype=np.float64)
    cov = np.diag([0.5, 0.1]).astype(np.float64)
    heading_meas = np.zeros(sample_count, dtype=np.float32)

    for idx in range(sample_count):
        dt = _safe_dt(timestamps[idx], timestamps[idx - 1]) if idx > 0 else 1e-3
        A = np.array([[1.0, -dt], [0.0, 1.0]], dtype=np.float64)
        B = np.array([dt, 0.0], dtype=np.float64)
        pred = A @ state + B * float(gyro_z[idx])
        pred[0] = _wrap_angle(float(pred[0]))

        Q = np.diag(
            [
                _HEADING_FILTER_Q_PSI * dt,
                _HEADING_FILTER_Q_BIAS * dt,
            ]
        ).astype(np.float64)
        cov_pred = A @ cov @ A.T + Q

        if gps_valid[idx] > 0.5:
            H = np.array([1.0, 0.0], dtype=np.float64)
            innovation = _wrap_angle(float(gps_theta[idx]) - float(pred[0]))
            S = float(H @ cov_pred @ H.T + _HEADING_FILTER_R_GPS)
            if np.isfinite(S) and S > 1e-9:
                K = (cov_pred @ H) / S
                pred = pred + K * innovation
                pred[0] = _wrap_angle(float(pred[0]))
                I = np.eye(2, dtype=np.float64)
                KH = np.outer(K, H)
                cov_pred = (
                    (I - KH) @ cov_pred @ (I - KH).T
                    + np.outer(K, K) * _HEADING_FILTER_R_GPS
                )

        state = pred
        cov = cov_pred
        heading_meas[idx] = np.float32(_wrap_angle(float(state[0])))

    return heading_meas


def _rebuild_heading_measurements_kinematic(
    dataset: Dict[str, Any],
    timestamps: np.ndarray,
    kinematic_config: Optional[Dict[str, Any]] = None,
) -> np.ndarray:
    sample_count = int(timestamps.shape[0])
    if sample_count == 0:
        return np.zeros(0, dtype=np.float32)

    cfg = _resolve_heading_kinematic_config(kinematic_config)
    z = np.asarray(dataset["z"], dtype=np.float32)
    gps_valid = np.asarray(_dataset_series(dataset, "gps_valid", timestamps), dtype=np.float32).reshape(-1)
    gps_theta = np.asarray(dataset.get("gps_theta", z[:, 2]), dtype=np.float32).reshape(-1)
    motor_tach = np.asarray(dataset.get("motor_tach", z[:, 3]), dtype=np.float32).reshape(-1)
    throttle = np.asarray(dataset.get("throttle", np.zeros(sample_count, dtype=np.float32)), dtype=np.float32).reshape(-1)
    accel_x = np.asarray(
        dataset.get("accel_x", dataset.get("ax", np.zeros(sample_count, dtype=np.float32))),
        dtype=np.float32,
    ).reshape(-1)
    delta = np.asarray(
        dataset.get("delta", dataset.get("steering", np.zeros(sample_count, dtype=np.float32))),
        dtype=np.float32,
    ).reshape(-1)
    x_gt = np.asarray(dataset.get("x_gt", z), dtype=np.float32)

    if gps_valid[0] > 0.5:
        psi0 = float(gps_theta[0])
    elif x_gt.ndim == 2 and x_gt.shape[0] > 0 and x_gt.shape[1] > 2:
        psi0 = float(x_gt[0, 2])
    else:
        psi0 = float(z[0, 2])

    if x_gt.ndim == 2 and x_gt.shape[0] > 0 and x_gt.shape[1] > 3:
        prev_v = float(x_gt[0, 3])
    else:
        prev_v = float(z[0, 3])
    if not np.isfinite(prev_v):
        prev_v = float(motor_tach[0]) if sample_count > 0 and np.isfinite(motor_tach[0]) else 0.0

    heading_meas = np.zeros(sample_count, dtype=np.float32)
    prev_heading = _wrap_angle(psi0)

    for idx in range(sample_count):
        dt = _safe_dt(timestamps[idx], timestamps[idx - 1]) if idx > 0 else 1e-3
        v_next = _predict_kinematic_velocity(
            prev_v=prev_v,
            motor_tach=float(motor_tach[idx]),
            throttle=float(throttle[idx]),
            ax=float(accel_x[idx]),
            dt=dt,
            cfg=cfg,
        )
        v_pose = float(motor_tach[idx]) if cfg["velocity_model"] == "tachometer" else v_next
        if not np.isfinite(v_pose):
            v_pose = prev_v
        yaw_rate = float(v_pose) * float(np.tan(float(delta[idx]))) / float(cfg["wheelbase"])

        if gps_valid[idx] > 0.5:
            heading_val = _wrap_angle(float(gps_theta[idx]))
        else:
            heading_val = _wrap_angle(float(prev_heading) + yaw_rate * dt)

        heading_meas[idx] = np.float32(heading_val)
        prev_heading = heading_val
        prev_v = v_next

    return heading_meas


def _rebuild_heading_measurements_qcar_ekf(
    dataset: Dict[str, Any],
    timestamps: np.ndarray,
    kinematic_config: Optional[Dict[str, Any]] = None,
) -> np.ndarray:
    sample_count = int(timestamps.shape[0])
    if sample_count == 0:
        return np.zeros(0, dtype=np.float32)

    z = np.asarray(dataset["z"], dtype=np.float32)
    gps_valid = np.asarray(
        _dataset_series(dataset, "gps_valid", timestamps), dtype=np.float32
    ).reshape(-1)
    gps_x = np.asarray(dataset.get("gps_x", z[:, 0]), dtype=np.float32).reshape(-1)
    gps_y = np.asarray(dataset.get("gps_y", z[:, 1]), dtype=np.float32).reshape(-1)
    gps_theta = np.asarray(dataset.get("gps_theta", z[:, 2]), dtype=np.float32).reshape(-1)
    motor_tach = np.asarray(dataset.get("motor_tach", z[:, 3]), dtype=np.float32).reshape(-1)
    gyro_z = np.asarray(
        dataset.get(
            "gyro_z",
            dataset.get("wz", np.zeros(sample_count, dtype=np.float32)),
        ),
        dtype=np.float32,
    ).reshape(-1)
    delta = np.asarray(
        dataset.get("delta", dataset.get("steering", np.zeros(sample_count, dtype=np.float32))),
        dtype=np.float32,
    ).reshape(-1)
    x_gt = np.asarray(dataset.get("x_gt", z), dtype=np.float32)

    if gps_valid[0] > 0.5:
        initial_pose = np.array([gps_x[0], gps_y[0], gps_theta[0]], dtype=np.float64)
    elif x_gt.ndim == 2 and x_gt.shape[0] > 0 and x_gt.shape[1] > 2:
        initial_pose = np.array([x_gt[0, 0], x_gt[0, 1], x_gt[0, 2]], dtype=np.float64)
    else:
        initial_pose = np.array([z[0, 0], z[0, 1], z[0, 2]], dtype=np.float64)

    fusion = QCarHeadingFusion(
        initial_pose=initial_pose,
        config=_make_qcar_heading_fusion_config(kinematic_config),
    )
    heading_meas = np.zeros(sample_count, dtype=np.float32)

    for idx in range(sample_count):
        dt = _safe_dt(timestamps[idx], timestamps[idx - 1]) if idx > 0 else 1e-3
        gps_pose = None
        if gps_valid[idx] > 0.5:
            gps_pose = np.array(
                [gps_x[idx], gps_y[idx], gps_theta[idx]], dtype=np.float64
            )
        fusion.update(
            velocity=float(motor_tach[idx]),
            steering=float(delta[idx]),
            dt=dt,
            gps_pose=gps_pose,
            gyro_z=float(gyro_z[idx]),
        )
        heading_meas[idx] = np.float32(fusion.get_heading())

    return heading_meas


def _rebuild_heading_measurements(
    dataset: Dict[str, Any],
    timestamps: np.ndarray,
    mode: str = "qcar_ekf",
    kinematic_config: Optional[Dict[str, Any]] = None,
) -> np.ndarray:
    mode_norm = str(mode).strip().lower()
    if mode_norm == "qcar_ekf":
        return _rebuild_heading_measurements_qcar_ekf(
            dataset,
            timestamps,
            kinematic_config=kinematic_config,
        )
    if mode_norm == "kinematic":
        return _rebuild_heading_measurements_kinematic(
            dataset,
            timestamps,
            kinematic_config=kinematic_config,
        )
    if mode_norm != "gyro_filter":
        raise ValueError(
            f"Unsupported heading rebuild mode '{mode}'. Expected 'qcar_ekf', 'gyro_filter', or 'kinematic'."
        )
    return _rebuild_heading_measurements_gyro_filter(dataset, timestamps)


def _default_series(key: str, timestamps: np.ndarray, gps_valid: Optional[np.ndarray] = None) -> np.ndarray:
    length = int(timestamps.shape[0])
    if key == "theta_valid":
        return np.ones(length, dtype=np.float32)
    if key == "gps_hold_valid":
        if gps_valid is None:
            return np.zeros(length, dtype=np.float32)
        return np.asarray(gps_valid, dtype=np.float32).reshape(-1)
    if key == "gps_age_sec":
        if length == 0:
            return np.zeros(0, dtype=np.float32)
        if gps_valid is None:
            return np.full(length, _GPS_AGE_CAP_SEC, dtype=np.float32)
        gps_valid = np.asarray(gps_valid, dtype=np.float32).reshape(-1)
        age = np.full(length, _GPS_AGE_CAP_SEC, dtype=np.float32)
        last_fix_time = None
        for idx, ts in enumerate(timestamps):
            if gps_valid[idx] > 0.5:
                last_fix_time = float(ts)
                age[idx] = 0.0
            elif last_fix_time is not None:
                age[idx] = _sanitize_gps_age(float(ts) - last_fix_time)
        return age
    if key == "gps_has_fix":
        if gps_valid is None:
            return np.zeros(length, dtype=np.float32)
        gps_valid = np.asarray(gps_valid, dtype=np.float32).reshape(-1)
        return np.maximum.accumulate((gps_valid > 0.5).astype(np.float32))
    if key in {"z", "x_gt"}:
        return np.zeros((length, MEAS_DIM), dtype=np.float32)
    return np.zeros(length, dtype=np.float32)


def _dataset_series(
    dataset: Dict[str, Any],
    key: str,
    timestamps: np.ndarray,
    gps_valid: Optional[np.ndarray] = None,
) -> np.ndarray:
    if key in dataset:
        if key == "timestamps":
            return np.asarray(dataset[key], dtype=np.float64)
        return np.asarray(dataset[key], dtype=np.float32)
    return _default_series(key, timestamps=timestamps, gps_valid=gps_valid)


def _as_state_matrix(array: np.ndarray, dim: int, name: str) -> np.ndarray:
    arr = np.asarray(array, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] < dim:
        raise ValueError(f"Expected {name} with shape [N, >={dim}], got {arr.shape}")
    return arr[:, :dim].copy()


def _rebuild_measurements(
    dataset: Dict[str, Any],
    timestamps: np.ndarray,
    heading_rebuild_mode: str = "qcar_ekf",
    heading_kinematic_config: Optional[Dict[str, Any]] = None,
    gps_dropout_xy_mode: str = "freeze",
) -> np.ndarray:
    gps_dropout_xy_mode = _normalize_gps_dropout_xy_mode(gps_dropout_xy_mode)
    z_full = np.asarray(dataset["z"], dtype=np.float32)
    if z_full.ndim != 2 or z_full.shape[1] < MEAS_DIM:
        raise ValueError(f"Expected z with shape [N, >={MEAS_DIM}], got {z_full.shape}")
    z = z_full[:, :MEAS_DIM].copy()

    gps_valid = np.asarray(_dataset_series(dataset, "gps_valid", timestamps), dtype=np.float32).reshape(-1)
    gps_has_fix = np.asarray(
        _dataset_series(dataset, "gps_has_fix", timestamps, gps_valid=gps_valid),
        dtype=np.float32,
    ).reshape(-1)
    gps_x = np.asarray(dataset.get("gps_x", z[:, 0]), dtype=np.float32).reshape(-1)
    gps_y = np.asarray(dataset.get("gps_y", z[:, 1]), dtype=np.float32).reshape(-1)
    motor_tach = np.asarray(
        _dataset_series(dataset, "motor_tach", timestamps),
        dtype=np.float32,
    ).reshape(-1)
    z[:, 2] = _rebuild_heading_measurements(
        dataset,
        timestamps,
        mode=heading_rebuild_mode,
        kinematic_config=heading_kinematic_config,
    )

    last_x = None
    last_y = None
    for idx in range(z.shape[0]):
        if gps_valid[idx] > 0.5:
            last_x = float(gps_x[idx])
            last_y = float(gps_y[idx])
            z[idx, 0] = last_x
            z[idx, 1] = last_y
        elif last_x is not None and last_y is not None:
            if gps_dropout_xy_mode == "dead_reckon":
                prev_ts = float(timestamps[idx - 1]) if idx > 0 else float(timestamps[idx])
                dt_rec = _safe_dt(float(timestamps[idx]), prev_ts)
                last_x, last_y = _dead_reckon_xy(
                    last_x,
                    last_y,
                    heading=float(z[idx, 2]),
                    velocity=float(motor_tach[idx]),
                    dt=dt_rec,
                )
            # Freeze x/y at the last known GPS position instead of
            # dead-reckoning.  The update module already zeroes the x/y
            # innovation when gps_valid=0 (via meas_availability_seq),
            # so the frozen value does not affect the correction during
            # GPS dropout.  When GPS returns, the jump from the frozen
            # position to the fresh GPS reading produces a large dz
            # feature that teaches the model to snap back — matching the
            # covariance-driven behaviour of the classical EKF.
            z[idx, 0] = last_x
            z[idx, 1] = last_y
    return z


class RobustKalmanNetDatasetRecorder:
    """Collect time-aligned samples for offline Robust KalmanNet training."""

    def __init__(self, vehicle_id: int, logger=None, output_dir: Optional[str] = None):
        self.vehicle_id = int(vehicle_id)
        self.logger = logger
        self.output_dir = Path(output_dir) if output_dir else Path(__file__).resolve().parent / "datasets"
        self.recording = False
        self.dataset_name = "robust_kalmannet_dataset"
        self.target_estimator_type = "ekf"
        self.strict_target_type = True
        self.wheel_speed_scale = 1.0
        self.gps_dropout_xy_mode = "freeze"
        self.metadata: Dict[str, Any] = {}
        self.last_saved_path: Optional[str] = None
        self.last_error: Optional[str] = None
        self._buffers: Dict[str, List[Any]] = {}
        self._start_time = 0.0
        self.heading_wheelbase = _KIN_WHEELBASE
        self._heading_fusion: Optional[QCarHeadingFusion] = None

    def _log_info(self, message: str) -> None:
        if self.logger and hasattr(self.logger, "logger"):
            self.logger.logger.info(message)

    def _log_warning(self, message: str) -> None:
        if self.logger and hasattr(self.logger, "log_warning"):
            self.logger.log_warning(message)

    def _log_error(self, message: str, exc: Optional[Exception] = None) -> None:
        self.last_error = str(exc) if exc is not None else message
        if self.logger and hasattr(self.logger, "log_error"):
            self.logger.log_error(message, exc)

    def start(self, config: Optional[Dict[str, Any]] = None) -> bool:
        cfg = dict(config or {})
        self.output_dir = Path(cfg.get("output_dir", self.output_dir))
        self.dataset_name = str(cfg.get("dataset_name", self.dataset_name)).strip() or "robust_kalmannet_dataset"
        self.target_estimator_type = str(cfg.get("target_estimator_type", "ekf")).strip() or "ekf"
        self.strict_target_type = bool(cfg.get("strict_target_type", True))
        self.wheel_speed_scale = float(cfg.get("wheel_speed_scale", 1.0))
        self.gps_dropout_xy_mode = _normalize_gps_dropout_xy_mode(
            cfg.get("gps_dropout_xy_mode", self.gps_dropout_xy_mode)
        )
        self.heading_wheelbase = max(float(cfg.get("wheelbase", _KIN_WHEELBASE)), 1e-6)

        self._buffers = {
            "timestamps": [],
            "motor_tach": [],
            "throttle": [],
            "steering": [],
            "gyro_z": [],
            "accel_x": [],
            "accel_y": [],
            "accel_z": [],
            "gps_valid": [],
            "theta_valid": [],
            "gps_hold_valid": [],
            "gps_age_sec": [],
            "gps_has_fix": [],
            "gps_x": [],
            "gps_y": [],
            "gps_theta": [],
            "z": [],
            "x_gt": [],
        }
        for key in RAW_KEYS:
            self._buffers[key] = []
        self._heading_fusion = None

        self.recording = True
        self._start_time = time.time()
        self.last_error = None
        self.metadata = {
            "vehicle_id": self.vehicle_id,
            "created_at": self._start_time,
            "dataset_name": self.dataset_name,
            "target_estimator_type": self.target_estimator_type,
            "strict_target_type": self.strict_target_type,
            "wheel_speed_scale": self.wheel_speed_scale,
            "gps_dropout_xy_mode": self.gps_dropout_xy_mode,
            "raw_keys": list(RAW_KEYS),
            "state_layout": ["x", "y", "theta", "v"],
            "measurement_layout": ["x", "y", "theta", "v"],
        }
        self._log_info(
            f"[RKNetDataset] Recording started for V{self.vehicle_id} using target '{self.target_estimator_type}'"
        )
        return True

    def record_sample(
        self,
        *,
        timestamp: float,
        motor_tach: float,
        steering: float,
        throttle: float,
        gyro_z: float,
        acceleration: Optional[Sequence[float]],
        gps_data: Optional[Dict[str, Any]],
        target_state: Sequence[float],
        current_estimator_type: str,
    ) -> bool:
        if not self.recording:
            return False

        try:
            estimator_type = str(current_estimator_type or "unknown")
            if self.strict_target_type and estimator_type != self.target_estimator_type:
                return False

            target = np.asarray(target_state, dtype=np.float32).reshape(-1)
            if target.size < 4 or not np.all(np.isfinite(target[:4])):
                return False

            accel = np.asarray(acceleration if acceleration is not None else np.zeros(3), dtype=np.float32).reshape(-1)
            ax = float(accel[0]) if accel.size > 0 else 0.0
            ay = float(accel[1]) if accel.size > 1 else 0.0
            az = float(accel[2]) if accel.size > 2 else 0.0
            w = float(gyro_z)
            wheel_speed = float(motor_tach) * self.wheel_speed_scale

            gps_info = dict(gps_data or {})
            gps_position_valid = bool(
                gps_info.get("position_valid", gps_info.get("fresh", gps_info.get("valid", False)))
            )
            gps_hold_valid = bool(gps_info.get("hold_valid", gps_info.get("valid", gps_position_valid)))
            gps_age_sec = _sanitize_gps_age(gps_info.get("age_sec", 0.0 if gps_position_valid else _GPS_AGE_CAP_SEC))
            gps_has_fix = bool(
                gps_info.get(
                    "has_fix",
                    gps_position_valid or bool(self._buffers["gps_has_fix"] and self._buffers["gps_has_fix"][-1] > 0.5),
                )
            )

            prev_z = self._buffers["z"][-1] if self._buffers["z"] else None
            prev_ts = self._buffers["timestamps"][-1] if self._buffers["timestamps"] else timestamp
            dt_rec = max(float(timestamp) - float(prev_ts), 1e-3)

            if gps_has_fix and gps_data is not None:
                gps_x = float(gps_info.get("x", prev_z[0] if prev_z is not None else target[0]))
                gps_y = float(gps_info.get("y", prev_z[1] if prev_z is not None else target[1]))
                gps_theta = _wrap_angle(float(gps_info.get("theta", prev_z[2] if prev_z is not None else target[2])))
            elif prev_z is not None:
                gps_x = float(prev_z[0])
                gps_y = float(prev_z[1])
                gps_theta = float(prev_z[2])
            else:
                gps_x = float(target[0])
                gps_y = float(target[1])
                gps_theta = _wrap_angle(float(target[2]))

            if self._heading_fusion is None:
                self._heading_fusion = QCarHeadingFusion(
                    initial_pose=np.array([gps_x, gps_y, gps_theta], dtype=np.float64),
                    config=_make_qcar_heading_fusion_config(
                        {"wheelbase": self.heading_wheelbase}
                    ),
                )
            gps_pose = None
            if gps_position_valid:
                gps_pose = np.array([gps_x, gps_y, gps_theta], dtype=np.float64)
            self._heading_fusion.update(
                velocity=float(motor_tach),
                steering=float(steering),
                dt=dt_rec,
                gps_pose=gps_pose,
                gyro_z=w,
            )
            heading_meas = _wrap_angle(float(self._heading_fusion.get_heading()))

            if not gps_position_valid and prev_z is not None and gps_has_fix:
                if self.gps_dropout_xy_mode == "dead_reckon":
                    gps_x, gps_y = _dead_reckon_xy(
                        float(prev_z[0]),
                        float(prev_z[1]),
                        heading=heading_meas,
                        velocity=float(motor_tach),
                        dt=dt_rec,
                    )
                else:
                    gps_x = float(prev_z[0])
                    gps_y = float(prev_z[1])
                # Freeze x/y at the last measurement position instead of
                # dead-reckoning — matches the offline rebuild used for
                # training so the model sees the same distribution online.

            z = np.array(
                [gps_x, gps_y, heading_meas, float(motor_tach)],
                dtype=np.float32,
            )

            x_gt = np.array(
                [
                    float(target[0]),
                    float(target[1]),
                    _wrap_angle(float(target[2])),
                    float(target[3]),
                ],
                dtype=np.float32,
            )

            for key, value in {
                "timestamps": float(timestamp),
                "motor_tach": float(motor_tach),
                "throttle": float(throttle),
                "steering": float(steering),
                "gyro_z": w,
                "accel_x": ax,
                "accel_y": ay,
                "accel_z": az,
                "gps_valid": 1.0 if gps_position_valid else 0.0,
                # Keep heading measurement available through GPS dropout. The
                # heading channel is reconstructed from gyro integration when GPS
                # heading is unavailable, and training should see the same gate
                # semantics as runtime.
                "theta_valid": 1.0,
                "gps_hold_valid": 1.0 if gps_hold_valid else 0.0,
                "gps_age_sec": gps_age_sec,
                "gps_has_fix": 1.0 if gps_has_fix else 0.0,
                "gps_x": gps_x,
                "gps_y": gps_y,
                "gps_theta": gps_theta,
                "ax": ax,
                "ay": ay,
                "wz": w,
                "delta": float(steering),
                "vfl": wheel_speed,
                "vfr": wheel_speed,
                "vrl": wheel_speed,
                "vrr": wheel_speed,
            }.items():
                self._buffers[key].append(value)

            self._buffers["z"].append(z)
            self._buffers["x_gt"].append(x_gt)
            return True
        except Exception as exc:
            self._log_error("[RKNetDataset] Failed to record sample", exc)
            return False

    def stop(self, save: bool = True) -> Optional[str]:
        """Stop recording and optionally save the buffer."""
        already_stopped = not self.recording
        self.recording = False

        if not save:
            if not already_stopped:
                self._log_info(f"[RKNetDataset] Recording stopped for V{self.vehicle_id}")
            return None

        sample_count = len(self._buffers.get("timestamps", []))
        if sample_count > 0:
            return self._save_dataset()

        return self.last_saved_path

    def clear(self) -> None:
        """Clear all buffered samples."""
        self._buffers = {}
        self._heading_fusion = None
        self._log_info(f"[RKNetDataset] Buffer cleared for V{self.vehicle_id}")

    def _save_dataset(self) -> Optional[str]:
        try:
            sample_count = len(self._buffers.get("timestamps", []))
            if sample_count == 0:
                self._log_warning("[RKNetDataset] No samples collected; dataset not saved")
                return None

            self.output_dir.mkdir(parents=True, exist_ok=True)
            timestamp_tag = time.strftime("%Y%m%d_%H%M%S")
            base_name = f"{self.dataset_name}_V{self.vehicle_id}_{timestamp_tag}"
            npz_path = self.output_dir / f"{base_name}.npz"
            json_path = self.output_dir / f"{base_name}.json"

            timestamps = np.asarray(self._buffers["timestamps"], dtype=np.float64)
            dt_mean = float(np.mean(np.diff(timestamps))) if sample_count > 1 else 0.0
            metadata = dict(self.metadata)
            metadata.update(
                {
                    "sample_count": sample_count,
                    "saved_at": time.time(),
                    "duration_sec": float(timestamps[-1] - timestamps[0]) if sample_count > 1 else 0.0,
                    "dt_mean": dt_mean,
                    "npz_path": str(npz_path),
                }
            )

            np.savez_compressed(
                npz_path,
                timestamps=timestamps,
                motor_tach=np.asarray(self._buffers["motor_tach"], dtype=np.float32),
                throttle=np.asarray(self._buffers["throttle"], dtype=np.float32),
                steering=np.asarray(self._buffers["steering"], dtype=np.float32),
                gyro_z=np.asarray(self._buffers["gyro_z"], dtype=np.float32),
                accel_x=np.asarray(self._buffers["accel_x"], dtype=np.float32),
                accel_y=np.asarray(self._buffers["accel_y"], dtype=np.float32),
                accel_z=np.asarray(self._buffers["accel_z"], dtype=np.float32),
                gps_valid=np.asarray(self._buffers["gps_valid"], dtype=np.float32),
                theta_valid=np.asarray(self._buffers["theta_valid"], dtype=np.float32),
                gps_hold_valid=np.asarray(self._buffers["gps_hold_valid"], dtype=np.float32),
                gps_age_sec=np.asarray(self._buffers["gps_age_sec"], dtype=np.float32),
                gps_has_fix=np.asarray(self._buffers["gps_has_fix"], dtype=np.float32),
                gps_x=np.asarray(self._buffers["gps_x"], dtype=np.float32),
                gps_y=np.asarray(self._buffers["gps_y"], dtype=np.float32),
                gps_theta=np.asarray(self._buffers["gps_theta"], dtype=np.float32),
                ax=np.asarray(self._buffers["ax"], dtype=np.float32),
                ay=np.asarray(self._buffers["ay"], dtype=np.float32),
                wz=np.asarray(self._buffers["wz"], dtype=np.float32),
                delta=np.asarray(self._buffers["delta"], dtype=np.float32),
                vfl=np.asarray(self._buffers["vfl"], dtype=np.float32),
                vfr=np.asarray(self._buffers["vfr"], dtype=np.float32),
                vrl=np.asarray(self._buffers["vrl"], dtype=np.float32),
                vrr=np.asarray(self._buffers["vrr"], dtype=np.float32),
                z=np.asarray(self._buffers["z"], dtype=np.float32),
                x_gt=np.asarray(self._buffers["x_gt"], dtype=np.float32),
                metadata_json=json.dumps(metadata),
            )
            json_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

            self.last_saved_path = str(npz_path)
            self._log_info(f"[RKNetDataset] Saved {sample_count} samples to {npz_path}")
            return self.last_saved_path
        except Exception as exc:
            self._log_error("[RKNetDataset] Failed to save dataset", exc)
            return None

    def get_status(self) -> Dict[str, Any]:
        sample_count = len(self._buffers.get("timestamps", []))
        return {
            "recording": self.recording,
            "buffered_samples": sample_count,
            "dataset_name": self.dataset_name,
            "target_estimator_type": self.target_estimator_type,
            "strict_target_type": self.strict_target_type,
            "last_saved_path": self.last_saved_path,
            "last_error": self.last_error,
        }


def load_recorded_dataset(path: str) -> Dict[str, Any]:
    npz_path = Path(path)
    with np.load(npz_path, allow_pickle=False) as data:
        metadata_json = str(data["metadata_json"].tolist()) if "metadata_json" in data else "{}"
        metadata = json.loads(metadata_json)
        payload = {key: data[key] for key in data.files if key != "metadata_json"}
        payload["metadata"] = metadata
        payload["path"] = str(npz_path)
        return payload


def merge_recorded_datasets(paths: Iterable[str]) -> Dict[str, Any]:
    datasets = [load_recorded_dataset(path) for path in paths]
    if not datasets:
        raise ValueError("No dataset paths provided")

    merged: Dict[str, Any] = {
        "metadata": {
            "source_files": [ds["path"] for ds in datasets],
            "segment_lengths": [int(np.asarray(ds["timestamps"]).shape[0]) for ds in datasets],
            "source_segments": [
                {
                    "path": ds["path"],
                    "length": int(np.asarray(ds["timestamps"]).shape[0]),
                    "dt_mean": float(ds.get("metadata", {}).get("dt_mean", 0.02) or 0.02),
                }
                for ds in datasets
            ],
        }
    }
    keys = sorted(
        {
            key
            for ds in datasets
            for key in ds.keys()
            if key not in {"metadata", "path"}
        }
    )
    for key in keys:
        arrays = []
        for ds in datasets:
            timestamps = np.asarray(ds["timestamps"], dtype=np.float64)
            gps_valid = np.asarray(ds["gps_valid"], dtype=np.float32) if "gps_valid" in ds else None
            arr = _dataset_series(ds, key, timestamps=timestamps, gps_valid=gps_valid)
            arrays.append(np.asarray(arr))
        merged[key] = np.concatenate(arrays, axis=0)
    merged["metadata"].update(datasets[0].get("metadata", {}))
    merged["metadata"]["sample_count"] = int(merged["timestamps"].shape[0])
    return merged


def create_sliding_windows(array: np.ndarray, sequence_length: int, stride: int = 1) -> np.ndarray:
    if array.shape[0] < sequence_length:
        raise ValueError(
            f"Need at least {sequence_length} samples but dataset only has {array.shape[0]}"
        )
    windows = [array[i : i + sequence_length] for i in range(0, array.shape[0] - sequence_length + 1, stride)]
    return np.stack(windows, axis=0)


def build_training_windows(
    dataset: Dict[str, Any],
    sequence_length: int,
    stride: int = 1,
    measurement_source: str = "rebuilt",
    heading_rebuild_mode: str = "qcar_ekf",
    heading_kinematic_config: Optional[Dict[str, Any]] = None,
    gps_dropout_xy_mode: str = "freeze",
) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    measurement_source_norm = str(measurement_source).strip().lower()
    if measurement_source_norm not in {"rebuilt", "raw"}:
        raise ValueError(
            f"Unsupported measurement_source '{measurement_source}'. Expected 'rebuilt' or 'raw'."
        )

    metadata = dataset.get("metadata", {}) or {}
    segment_lengths = [int(length) for length in metadata.get("segment_lengths", []) if int(length) > 0]
    source_segments = metadata.get("source_segments", []) or []

    timestamps_all = np.asarray(dataset["timestamps"], dtype=np.float64)
    if not segment_lengths:
        segment_lengths = [int(timestamps_all.shape[0])]

    gps_valid_all = np.asarray(_dataset_series(dataset, "gps_valid", timestamps_all), dtype=np.float32).reshape(-1)
    theta_valid_all = np.ones_like(gps_valid_all, dtype=np.float32)
    gps_hold_valid_all = np.asarray(
        _dataset_series(dataset, "gps_hold_valid", timestamps_all, gps_valid=gps_valid_all),
        dtype=np.float32,
    ).reshape(-1)
    gps_age_all = np.asarray(
        _dataset_series(dataset, "gps_age_sec", timestamps_all, gps_valid=gps_valid_all),
        dtype=np.float32,
    ).reshape(-1)

    raw_series = {
        "gps_valid": gps_valid_all,
        "theta_valid": theta_valid_all,
        "gps_hold_valid": gps_hold_valid_all,
        "gps_age_sec": gps_age_all,
    }
    for key in RAW_KEYS:
        if key in raw_series:
            continue
        raw_series[key] = np.asarray(_dataset_series(dataset, key, timestamps_all), dtype=np.float32).reshape(-1)

    if measurement_source_norm == "raw":
        z_raw = np.asarray(_dataset_series(dataset, "z", timestamps_all), dtype=np.float32)
        if z_raw.ndim != 2 or z_raw.shape[1] < MEAS_DIM:
            raise ValueError(
                f"Expected raw dataset z with shape [N, >={MEAS_DIM}], got {z_raw.shape}"
            )
        z_all = z_raw[:, :MEAS_DIM].copy()
    else:
        z_all = _rebuild_measurements(
            dataset,
            timestamps_all,
            heading_rebuild_mode=heading_rebuild_mode,
            heading_kinematic_config=heading_kinematic_config,
            gps_dropout_xy_mode=gps_dropout_xy_mode,
        )
    x_gt_all = _as_state_matrix(dataset["x_gt"], TARGET_DIM, "x_gt")

    raw_windows = {key: [] for key in RAW_KEYS}
    z_windows: List[np.ndarray] = []
    x_gt_windows: List[np.ndarray] = []
    dt_windows: List[np.ndarray] = []

    start = 0
    for segment_index, segment_length in enumerate(segment_lengths):
        end = start + segment_length
        if segment_length < sequence_length:
            start = end
            continue

        for key in RAW_KEYS:
            segment_raw = raw_series[key][start:end].reshape(-1, 1)
            raw_windows[key].append(create_sliding_windows(segment_raw, sequence_length, stride))

        z_windows.append(create_sliding_windows(z_all[start:end], sequence_length, stride))
        x_gt_windows.append(create_sliding_windows(x_gt_all[start:end], sequence_length, stride))

        timestamps = timestamps_all[start:end]
        dt_array = np.zeros(segment_length, dtype=np.float32)
        if segment_length > 1:
            dt_array[1:] = (timestamps[1:] - timestamps[:-1]).astype(np.float32)
            segment_meta = source_segments[segment_index] if segment_index < len(source_segments) else {}
            mean_dt = float(segment_meta.get("dt_mean", 0.02) or 0.02)
            dt_array[0] = mean_dt
        else:
            dt_array[:] = 0.02
        dt_windows.append(create_sliding_windows(dt_array.reshape(-1, 1), sequence_length, stride))
        start = end

    if not z_windows:
        total_samples = int(timestamps_all.shape[0])
        raise ValueError(
            f"Need at least {sequence_length} samples in one contiguous dataset segment, "
            f"but the longest segment is shorter (total merged samples={total_samples})."
        )

    raw = {
        key: np.concatenate(raw_windows[key], axis=0)
        for key in RAW_KEYS
    }
    z_seq = np.concatenate(z_windows, axis=0)
    x_gt = np.concatenate(x_gt_windows, axis=0)
    x0 = z_seq[:, 0, :].copy()
    dt_seq = np.concatenate(dt_windows, axis=0)

    return raw, z_seq, x_gt, x0, dt_seq


def compute_rmse(pred: np.ndarray, target: np.ndarray) -> Dict[str, Any]:
    if pred.shape != target.shape:
        raise ValueError(f"Shape mismatch: pred {pred.shape} vs target {target.shape}")
    err = pred - target
    err[..., 2] = np.arctan2(np.sin(err[..., 2]), np.cos(err[..., 2]))
    mse = np.mean(err ** 2, axis=tuple(range(err.ndim - 1)))
    rmse = np.sqrt(mse)
    return {
        "rmse": rmse.tolist(),
        "rmse_mean": float(np.mean(rmse)),
    }
