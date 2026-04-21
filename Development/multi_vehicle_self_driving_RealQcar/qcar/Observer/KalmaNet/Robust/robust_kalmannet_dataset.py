import json
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


RAW_KEYS = (
    "ax",
    "ay",
    "wz",
    "delta",
    "vfl",
    "vfr",
    "vrl",
    "vrr",
    "gps_valid",
    "gps_hold_valid",
    "gps_age_sec",
)
TARGET_DIM = 5
MEAS_DIM = 5
_GPS_AGE_CAP_SEC = 1.0e3


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


def _default_series(key: str, timestamps: np.ndarray, gps_valid: Optional[np.ndarray] = None) -> np.ndarray:
    length = int(timestamps.shape[0])
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


def _rebuild_measurements(dataset: Dict[str, Any], timestamps: np.ndarray) -> np.ndarray:
    z = np.asarray(dataset["z"], dtype=np.float32).copy()
    if z.ndim != 2 or z.shape[1] < MEAS_DIM:
        raise ValueError(f"Expected z with shape [N, {MEAS_DIM}], got {z.shape}")

    gps_valid = np.asarray(_dataset_series(dataset, "gps_valid", timestamps), dtype=np.float32).reshape(-1)
    gps_has_fix = np.asarray(
        _dataset_series(dataset, "gps_has_fix", timestamps, gps_valid=gps_valid),
        dtype=np.float32,
    ).reshape(-1)
    gps_x = np.asarray(dataset.get("gps_x", z[:, 0]), dtype=np.float32).reshape(-1)
    gps_y = np.asarray(dataset.get("gps_y", z[:, 1]), dtype=np.float32).reshape(-1)

    last_x = None
    last_y = None
    for idx in range(z.shape[0]):
        if gps_valid[idx] > 0.5:
            last_x = float(gps_x[idx])
            last_y = float(gps_y[idx])
        if gps_has_fix[idx] > 0.5 and last_x is not None and last_y is not None:
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
        self.metadata: Dict[str, Any] = {}
        self.last_saved_path: Optional[str] = None
        self.last_error: Optional[str] = None
        self._buffers: Dict[str, List[Any]] = {}
        self._start_time = 0.0

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
            "raw_keys": list(RAW_KEYS),
            "state_layout": ["x", "y", "theta", "v", "w"],
            "measurement_layout": ["x", "y", "theta", "v", "w"],
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

            if gps_position_valid:
                heading_meas = gps_theta
            elif prev_z is not None:
                heading_meas = _wrap_angle(float(prev_z[2]) + w * dt_rec)
            else:
                heading_meas = _wrap_angle(float(target[2]))

            z = np.array(
                [gps_x, gps_y, heading_meas, float(motor_tach), w],
                dtype=np.float32,
            )

            target_w = float(target[4]) if target.size > 4 else w
            x_gt = np.array(
                [
                    float(target[0]),
                    float(target[1]),
                    _wrap_angle(float(target[2])),
                    float(target[3]),
                    target_w,
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
) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    metadata = dataset.get("metadata", {}) or {}
    segment_lengths = [int(length) for length in metadata.get("segment_lengths", []) if int(length) > 0]
    source_segments = metadata.get("source_segments", []) or []

    timestamps_all = np.asarray(dataset["timestamps"], dtype=np.float64)
    if not segment_lengths:
        segment_lengths = [int(timestamps_all.shape[0])]

    gps_valid_all = np.asarray(_dataset_series(dataset, "gps_valid", timestamps_all), dtype=np.float32).reshape(-1)
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
        "gps_hold_valid": gps_hold_valid_all,
        "gps_age_sec": gps_age_all,
    }
    for key in RAW_KEYS:
        if key in raw_series:
            continue
        raw_series[key] = np.asarray(_dataset_series(dataset, key, timestamps_all), dtype=np.float32).reshape(-1)

    z_all = _rebuild_measurements(dataset, timestamps_all)
    x_gt_all = np.asarray(dataset["x_gt"], dtype=np.float32)

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
