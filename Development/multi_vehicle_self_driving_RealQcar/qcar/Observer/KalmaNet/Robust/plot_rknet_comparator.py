import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

# python plot_rknet_comparator.py --no-show

DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "logs" / "comparator"
DEFAULT_EXPORT_DIR = Path(__file__).resolve().parent / "logs" / "plots"
DEFAULT_RECOVERY_WINDOW_SECONDS = 2.0


def _load_csv_rows(filepath: Path) -> Dict[str, np.ndarray]:
    with filepath.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows: List[dict] = list(reader)

    if not rows:
        raise SystemExit(f"No data rows found in {filepath}")

    numeric_columns = [
        "timestamp",
        "tick",
        "dt",
        "motor_tach",
        "steering",
        "throttle",
        "gyro_z",
        "robust_x",
        "robust_y",
        "robust_theta",
        "robust_v",
        "ekf_x",
        "ekf_y",
        "ekf_theta",
        "ekf_v",
        "ref_x",
        "ref_y",
        "ref_theta",
        "ref_v",
        "delta_x",
        "delta_y",
        "delta_theta",
        "delta_v",
        "position_error_norm",
        "heading_error",
        "velocity_error",
        "robust_ref_dx",
        "robust_ref_dy",
        "robust_ref_dtheta",
        "robust_ref_dv",
        "robust_ref_position_error_norm",
        "robust_ref_heading_error",
        "robust_ref_velocity_error",
        "ekf_ref_dx",
        "ekf_ref_dy",
        "ekf_ref_dtheta",
        "ekf_ref_dv",
        "ekf_ref_position_error_norm",
        "ekf_ref_heading_error",
        "ekf_ref_velocity_error",
        "gps_valid",
        "gps_hold_valid",
        "gps_age_sec",
        "sensor_failure_active",
        "sensor_failure_remaining_steps",
        "sensor_failure_intensity",
        "sensor_failure_imu_intensity",
        "sensor_failure_steer_intensity",
        "sensor_failure_wheel_intensity",
        "sensor_failure_gps_intensity",
        "sensor_failure_gps_xy_intensity",
        "sensor_failure_gps_valid_flip",
        "gps_x",
        "gps_y",
        "gps_theta",
        "accel_x",
        "accel_y",
        "accel_z",
        "pred_mask_mean",
        "pred_mask_min",
        "pred_mask_max",
        "mask_imu_mean",
        "mask_steer_mean",
        "mask_wheel_mean",
        "mask_selected_score",
        "mask_imu_active",
        "mask_steer_active",
        "mask_wheel_active",
        "meas_mask_x",
        "meas_mask_y",
        "meas_mask_psi",
        "meas_mask_v",
        "meas_mask_w",
        "K_norm",
        "K_x_x",
        "K_y_y",
        "K_psi_psi",
        "K_v_v",
        "K_w_w",
        "ekf_K_norm",
        "ekf_K_x_x",
        "ekf_K_y_y",
        "ekf_K_psi_psi",
        "ekf_K_v_v",
        "innov_x",
        "innov_y",
        "innov_psi",
        "innov_v",
        "innov_w",
        "pred_x",
        "pred_y",
    ]
    data: Dict[str, np.ndarray] = {}
    for column in numeric_columns:
        values = []
        for row in rows:
            raw = row.get(column, "")
            try:
                values.append(float(raw))
            except (TypeError, ValueError):
                values.append(np.nan)
        data[column] = np.asarray(values, dtype=np.float64)

    if "innov_psi" in data:
        data["innov_psi"] = np.arctan2(np.sin(data["innov_psi"]), np.cos(data["innov_psi"]))

    data["source"] = np.asarray([row.get("source", "") for row in rows], dtype=object)
    data["mask_selected_branch"] = np.asarray(
        [row.get("mask_selected_branch", "") for row in rows], dtype=object
    )
    data["sensor_failure_branches"] = np.asarray(
        [row.get("sensor_failure_branches", "") for row in rows], dtype=object
    )
    data["sensor_failure_branch_types"] = np.asarray(
        [row.get("sensor_failure_branch_types", "") for row in rows], dtype=object
    )
    data["sensor_failure_gps_type"] = np.asarray(
        [row.get("sensor_failure_gps_type", "") for row in rows], dtype=object
    )
    return data


def _resolve_input_file(filepath: Optional[str]) -> Path:
    if filepath:
        candidate = Path(filepath)
        if not candidate.is_absolute():
            candidate = Path.cwd() / candidate
        return candidate.resolve()

    csv_files = sorted(DEFAULT_LOG_DIR.glob("*.csv"))
    if not csv_files:
        raise SystemExit(f"No comparator CSV files found in {DEFAULT_LOG_DIR}")
    return csv_files[-1]


def _safe_float(value: float) -> Optional[float]:
    if value is None or not np.isfinite(value):
        return None
    return float(value)


def _finite_count(values: np.ndarray) -> int:
    return int(np.isfinite(values).sum())


def _series_stats(values: np.ndarray, *, absolute: bool = False) -> Dict[str, Optional[float]]:
    series = np.abs(values) if absolute else values
    finite = series[np.isfinite(series)]
    if finite.size == 0:
        return {
            "count": 0,
            "mean": None,
            "std": None,
            "min": None,
            "max": None,
            "median": None,
            "rmse": None,
        }

    return {
        "count": int(finite.size),
        "mean": _safe_float(np.mean(finite)),
        "std": _safe_float(np.std(finite)),
        "min": _safe_float(np.min(finite)),
        "max": _safe_float(np.max(finite)),
        "median": _safe_float(np.median(finite)),
        "rmse": _safe_float(np.sqrt(np.mean(np.square(finite)))),
    }


def _series_peak(
    values: np.ndarray,
    time_axis: np.ndarray,
    *,
    absolute: bool = False,
) -> Dict[str, Optional[float]]:
    series = np.abs(values) if absolute else values
    finite_mask = np.isfinite(series) & np.isfinite(time_axis)
    if not np.any(finite_mask):
        return {"time_seconds": None, "value": None}

    finite_series = series[finite_mask]
    finite_time = time_axis[finite_mask]
    peak_index = int(np.argmax(finite_series))
    return {
        "time_seconds": _safe_float(finite_time[peak_index]),
        "value": _safe_float(finite_series[peak_index]),
    }


def _masked_series_stats(
    values: np.ndarray,
    mask: np.ndarray,
    *,
    absolute: bool = False,
) -> Dict[str, Optional[float]]:
    if values.size == 0:
        return _series_stats(values, absolute=absolute)
    mask_bool = np.asarray(mask, dtype=bool)
    if mask_bool.shape != values.shape:
        mask_bool = np.broadcast_to(mask_bool, values.shape)
    masked = np.where(mask_bool, values, np.nan)
    return _series_stats(masked, absolute=absolute)


def _build_phase_masks(
    data: Dict[str, np.ndarray],
    time_axis: np.ndarray,
    recovery_window_seconds: float = DEFAULT_RECOVERY_WINDOW_SECONDS,
) -> Dict[str, np.ndarray]:
    sample_count = int(time_axis.size)
    attack_mask = np.asarray(
        data.get("sensor_failure_active", np.zeros(sample_count, dtype=np.float64)) > 0.5,
        dtype=bool,
    )
    recovery_mask = np.zeros(sample_count, dtype=bool)
    if sample_count == 0:
        return {
            "clean": recovery_mask.copy(),
            "attack": attack_mask,
            "recovery": recovery_mask,
        }

    window_seconds = max(float(recovery_window_seconds), 0.0)
    if window_seconds > 0.0:
        attack_segments = _extract_attack_segments(data, time_axis)
        for segment in attack_segments:
            end_time = segment.get("end_time_seconds")
            if end_time is None:
                continue
            recovery_mask |= (time_axis > float(end_time)) & (
                time_axis <= float(end_time) + window_seconds
            )
    recovery_mask &= ~attack_mask
    clean_mask = ~(attack_mask | recovery_mask)
    return {
        "clean": clean_mask,
        "attack": attack_mask,
        "recovery": recovery_mask,
    }


def _value_distribution(values: np.ndarray) -> Dict[str, int]:
    distribution: Dict[str, int] = {}
    for raw in values:
        key = str(raw).strip() if str(raw).strip() else "unknown"
        distribution[key] = distribution.get(key, 0) + 1
    return dict(sorted(distribution.items()))


def _clean_label(value: Any) -> str:
    label = str(value).strip()
    if not label or label.lower() in {"nan", "none"}:
        return ""
    return label


def _attack_type_label(branch_types: Any, gps_type: Any) -> str:
    labels: List[str] = []
    branch_label = _clean_label(branch_types)
    gps_label = _clean_label(gps_type)
    if branch_label:
        labels.append(branch_label)
    if gps_label:
        labels.append(f"gps:{gps_label}")
    return " + ".join(labels) if labels else "none"


def _extract_attack_segments(
    data: Dict[str, np.ndarray],
    time_axis: np.ndarray,
) -> List[Dict[str, Any]]:
    active = np.asarray(data.get("sensor_failure_active", np.zeros_like(time_axis)) > 0.5)
    if active.size == 0:
        return []

    segments: List[Dict[str, Any]] = []
    start_idx: Optional[int] = None
    for idx, is_active in enumerate(active):
        if is_active and start_idx is None:
            start_idx = idx
        elif not is_active and start_idx is not None:
            segments.append(_build_attack_segment(data, time_axis, start_idx, idx - 1))
            start_idx = None
    if start_idx is not None:
        segments.append(_build_attack_segment(data, time_axis, start_idx, active.size - 1))
    return segments


def _build_attack_segment(
    data: Dict[str, np.ndarray],
    time_axis: np.ndarray,
    start_idx: int,
    end_idx: int,
) -> Dict[str, Any]:
    sl = slice(start_idx, end_idx + 1)
    segment_intensity = np.asarray(data.get("sensor_failure_intensity", np.full_like(time_axis, np.nan))[sl])
    peak_idx_local = 0
    if np.any(np.isfinite(segment_intensity)):
        peak_idx_local = int(np.nanargmax(segment_intensity))
    peak_idx = start_idx + peak_idx_local

    labels = [
        _attack_type_label(branch_type, gps_type)
        for branch_type, gps_type in zip(
            data.get("sensor_failure_branch_types", np.asarray([], dtype=object))[sl],
            data.get("sensor_failure_gps_type", np.asarray([], dtype=object))[sl],
        )
    ]
    label_distribution: Dict[str, int] = {}
    for label in labels:
        clean = _clean_label(label)
        if not clean or clean == "none":
            continue
        label_distribution[clean] = label_distribution.get(clean, 0) + 1
    dominant_label = max(label_distribution, key=label_distribution.get) if label_distribution else "none"

    return {
        "start_index": int(start_idx),
        "end_index": int(end_idx),
        "start_time_seconds": _safe_float(time_axis[start_idx]),
        "end_time_seconds": _safe_float(time_axis[end_idx]),
        "duration_seconds": _safe_float(time_axis[end_idx] - time_axis[start_idx]),
        "peak_time_seconds": _safe_float(time_axis[peak_idx]),
        "peak_intensity": _safe_float(segment_intensity[peak_idx_local]) if segment_intensity.size else None,
        "dominant_type": dominant_label,
        "branch_types": _clean_label(data.get("sensor_failure_branch_types", np.asarray([], dtype=object))[peak_idx]),
        "gps_type": _clean_label(data.get("sensor_failure_gps_type", np.asarray([], dtype=object))[peak_idx]),
        "peak_remaining_steps": _safe_float(
            data.get("sensor_failure_remaining_steps", np.full_like(time_axis, np.nan))[peak_idx]
        ),
    }


def _compute_summary(filepath: Path, data: Dict[str, np.ndarray], time_axis: np.ndarray) -> Dict[str, Any]:
    total_samples = int(len(time_axis))
    recovery_window_seconds = DEFAULT_RECOVERY_WINDOW_SECONDS
    reference_available = bool(np.any(np.isfinite(data.get("ref_x", np.asarray([], dtype=np.float64)))))
    model_used = int(np.count_nonzero(data["source"] == "model"))
    fallback_used = int(np.count_nonzero(data["source"] == "fallback"))
    gps_valid_count = int(np.count_nonzero(data["gps_valid"] > 0.5))
    gps_hold_valid_count = int(np.count_nonzero(data["gps_hold_valid"] > 0.5))
    attack_active_count = int(np.count_nonzero(data.get("sensor_failure_active", np.zeros(total_samples)) > 0.5))
    mask_imu_active_count = int(np.count_nonzero(data["mask_imu_active"] > 0.5))
    mask_steer_active_count = int(np.count_nonzero(data["mask_steer_active"] > 0.5))
    mask_wheel_active_count = int(np.count_nonzero(data["mask_wheel_active"] > 0.5))
    attack_type_labels = np.asarray(
        [
            _attack_type_label(branch_types, gps_type)
            for branch_types, gps_type in zip(
                data.get("sensor_failure_branch_types", np.asarray([], dtype=object)),
                data.get("sensor_failure_gps_type", np.asarray([], dtype=object)),
            )
        ],
        dtype=object,
    )
    attack_segments = _extract_attack_segments(data, time_axis)
    phase_masks = _build_phase_masks(
        data,
        time_axis,
        recovery_window_seconds=recovery_window_seconds,
    )
    gain_errors = {
        "K_norm_error": data["K_norm"] - data["ekf_K_norm"],
        "K_x_x_error": data["K_x_x"] - data["ekf_K_x_x"],
        "K_y_y_error": data["K_y_y"] - data["ekf_K_y_y"],
        "K_psi_psi_error": data["K_psi_psi"] - data["ekf_K_psi_psi"],
        "K_v_v_error": data["K_v_v"] - data["ekf_K_v_v"],
    }

    summary: Dict[str, Any] = {
        "schema_version": 4,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_csv": str(filepath),
        "log_name": filepath.name,
        "sample_count": total_samples,
        "duration_seconds": _safe_float(time_axis[-1]) if total_samples else 0.0,
        "time_start_seconds": _safe_float(time_axis[0]) if total_samples else 0.0,
        "time_end_seconds": _safe_float(time_axis[-1]) if total_samples else 0.0,
        "usage_summary": {
            "model_samples": model_used,
            "fallback_samples": fallback_used,
            "gps_valid_samples": gps_valid_count,
            "gps_hold_valid_samples": gps_hold_valid_count,
            "attack_active_samples": attack_active_count,
            "mask_imu_active_samples": mask_imu_active_count,
            "mask_steer_active_samples": mask_steer_active_count,
            "mask_wheel_active_samples": mask_wheel_active_count,
            "reference_available_samples": int(
                np.count_nonzero(np.isfinite(data.get("ref_x", np.full(total_samples, np.nan))))
            ),
            "model_ratio": _safe_float(model_used / total_samples) if total_samples else None,
            "fallback_ratio": _safe_float(fallback_used / total_samples) if total_samples else None,
            "gps_valid_ratio": _safe_float(gps_valid_count / total_samples) if total_samples else None,
            "gps_hold_valid_ratio": _safe_float(gps_hold_valid_count / total_samples) if total_samples else None,
            "attack_active_ratio": _safe_float(attack_active_count / total_samples) if total_samples else None,
            "mask_imu_active_ratio": _safe_float(mask_imu_active_count / total_samples)
            if total_samples
            else None,
            "mask_steer_active_ratio": _safe_float(mask_steer_active_count / total_samples)
            if total_samples
            else None,
            "mask_wheel_active_ratio": _safe_float(mask_wheel_active_count / total_samples)
            if total_samples
            else None,
        },
        "selected_branch_counts": _value_distribution(data["mask_selected_branch"]),
        "reference_summary": {
            "available": reference_available,
            "mode": "clean_reference_ekf",
        },
        "phase_summary": {
            "recovery_window_seconds": recovery_window_seconds,
            "clean_samples": int(np.count_nonzero(phase_masks["clean"])),
            "attack_samples": int(np.count_nonzero(phase_masks["attack"])),
            "recovery_samples": int(np.count_nonzero(phase_masks["recovery"])),
            "clean_ratio": _safe_float(np.mean(phase_masks["clean"])) if total_samples else None,
            "attack_ratio": _safe_float(np.mean(phase_masks["attack"])) if total_samples else None,
            "recovery_ratio": _safe_float(np.mean(phase_masks["recovery"])) if total_samples else None,
        },
        "attack_summary": {
            "attack_type_counts": _value_distribution(attack_type_labels[attack_type_labels != "none"])
            if attack_type_labels.size
            else {},
            "attack_segment_count": int(len(attack_segments)),
            "segments": attack_segments,
        },
        "phase_metrics": {
            "clean": {
                "robust_ref_position_error_norm": _masked_series_stats(
                    data["robust_ref_position_error_norm"], phase_masks["clean"]
                ),
                "robust_ref_heading_error_abs": _masked_series_stats(
                    data["robust_ref_heading_error"], phase_masks["clean"], absolute=True
                ),
                "robust_ref_velocity_error_abs": _masked_series_stats(
                    data["robust_ref_velocity_error"], phase_masks["clean"], absolute=True
                ),
                "ekf_ref_position_error_norm": _masked_series_stats(
                    data["ekf_ref_position_error_norm"], phase_masks["clean"]
                ),
                "ekf_ref_heading_error_abs": _masked_series_stats(
                    data["ekf_ref_heading_error"], phase_masks["clean"], absolute=True
                ),
                "ekf_ref_velocity_error_abs": _masked_series_stats(
                    data["ekf_ref_velocity_error"], phase_masks["clean"], absolute=True
                ),
            },
            "attack": {
                "robust_ref_position_error_norm": _masked_series_stats(
                    data["robust_ref_position_error_norm"], phase_masks["attack"]
                ),
                "robust_ref_heading_error_abs": _masked_series_stats(
                    data["robust_ref_heading_error"], phase_masks["attack"], absolute=True
                ),
                "robust_ref_velocity_error_abs": _masked_series_stats(
                    data["robust_ref_velocity_error"], phase_masks["attack"], absolute=True
                ),
                "ekf_ref_position_error_norm": _masked_series_stats(
                    data["ekf_ref_position_error_norm"], phase_masks["attack"]
                ),
                "ekf_ref_heading_error_abs": _masked_series_stats(
                    data["ekf_ref_heading_error"], phase_masks["attack"], absolute=True
                ),
                "ekf_ref_velocity_error_abs": _masked_series_stats(
                    data["ekf_ref_velocity_error"], phase_masks["attack"], absolute=True
                ),
            },
            "recovery": {
                "robust_ref_position_error_norm": _masked_series_stats(
                    data["robust_ref_position_error_norm"], phase_masks["recovery"]
                ),
                "robust_ref_heading_error_abs": _masked_series_stats(
                    data["robust_ref_heading_error"], phase_masks["recovery"], absolute=True
                ),
                "robust_ref_velocity_error_abs": _masked_series_stats(
                    data["robust_ref_velocity_error"], phase_masks["recovery"], absolute=True
                ),
                "ekf_ref_position_error_norm": _masked_series_stats(
                    data["ekf_ref_position_error_norm"], phase_masks["recovery"]
                ),
                "ekf_ref_heading_error_abs": _masked_series_stats(
                    data["ekf_ref_heading_error"], phase_masks["recovery"], absolute=True
                ),
                "ekf_ref_velocity_error_abs": _masked_series_stats(
                    data["ekf_ref_velocity_error"], phase_masks["recovery"], absolute=True
                ),
            },
        },
        "series_metrics": {
            "position_error_norm": _series_stats(data["position_error_norm"]),
            "heading_error_signed": _series_stats(data["heading_error"]),
            "heading_error_abs": _series_stats(data["heading_error"], absolute=True),
            "velocity_error_signed": _series_stats(data["velocity_error"]),
            "velocity_error_abs": _series_stats(data["velocity_error"], absolute=True),
            "robust_ref_position_error_norm": _series_stats(data["robust_ref_position_error_norm"]),
            "robust_ref_heading_error_signed": _series_stats(data["robust_ref_heading_error"]),
            "robust_ref_heading_error_abs": _series_stats(data["robust_ref_heading_error"], absolute=True),
            "robust_ref_velocity_error_signed": _series_stats(data["robust_ref_velocity_error"]),
            "robust_ref_velocity_error_abs": _series_stats(data["robust_ref_velocity_error"], absolute=True),
            "ekf_ref_position_error_norm": _series_stats(data["ekf_ref_position_error_norm"]),
            "ekf_ref_heading_error_signed": _series_stats(data["ekf_ref_heading_error"]),
            "ekf_ref_heading_error_abs": _series_stats(data["ekf_ref_heading_error"], absolute=True),
            "ekf_ref_velocity_error_signed": _series_stats(data["ekf_ref_velocity_error"]),
            "ekf_ref_velocity_error_abs": _series_stats(data["ekf_ref_velocity_error"], absolute=True),
            "delta_x": _series_stats(data["delta_x"]),
            "delta_y": _series_stats(data["delta_y"]),
            "delta_theta": _series_stats(data["delta_theta"]),
            "delta_v": _series_stats(data["delta_v"]),
            "robust_ref_dx": _series_stats(data["robust_ref_dx"]),
            "robust_ref_dy": _series_stats(data["robust_ref_dy"]),
            "robust_ref_dtheta": _series_stats(data["robust_ref_dtheta"]),
            "robust_ref_dv": _series_stats(data["robust_ref_dv"]),
            "ekf_ref_dx": _series_stats(data["ekf_ref_dx"]),
            "ekf_ref_dy": _series_stats(data["ekf_ref_dy"]),
            "ekf_ref_dtheta": _series_stats(data["ekf_ref_dtheta"]),
            "ekf_ref_dv": _series_stats(data["ekf_ref_dv"]),
            "innov_x": _series_stats(data["innov_x"]),
            "innov_y": _series_stats(data["innov_y"]),
            "innov_psi": _series_stats(data["innov_psi"]),
            "innov_v": _series_stats(data["innov_v"]),
            "innov_w": _series_stats(data["innov_w"]),
            "K_norm": _series_stats(data["K_norm"]),
            "K_x_x": _series_stats(data["K_x_x"]),
            "K_y_y": _series_stats(data["K_y_y"]),
            "K_psi_psi": _series_stats(data["K_psi_psi"]),
            "K_v_v": _series_stats(data["K_v_v"]),
            "K_w_w": _series_stats(data["K_w_w"]),
            "ekf_K_norm": _series_stats(data["ekf_K_norm"]),
            "ekf_K_x_x": _series_stats(data["ekf_K_x_x"]),
            "ekf_K_y_y": _series_stats(data["ekf_K_y_y"]),
            "ekf_K_psi_psi": _series_stats(data["ekf_K_psi_psi"]),
            "ekf_K_v_v": _series_stats(data["ekf_K_v_v"]),
            "K_norm_error": _series_stats(gain_errors["K_norm_error"]),
            "K_x_x_error": _series_stats(gain_errors["K_x_x_error"]),
            "K_y_y_error": _series_stats(gain_errors["K_y_y_error"]),
            "K_psi_psi_error": _series_stats(gain_errors["K_psi_psi_error"]),
            "K_v_v_error": _series_stats(gain_errors["K_v_v_error"]),
            "steering": _series_stats(data["steering"]),
            "throttle": _series_stats(data["throttle"]),
            "mask_selected_score": _series_stats(data["mask_selected_score"]),
            "pred_mask_mean": _series_stats(data["pred_mask_mean"]),
            "pred_mask_min": _series_stats(data["pred_mask_min"]),
            "pred_mask_max": _series_stats(data["pred_mask_max"]),
            "mask_imu_mean": _series_stats(data["mask_imu_mean"]),
            "mask_steer_mean": _series_stats(data["mask_steer_mean"]),
            "mask_wheel_mean": _series_stats(data["mask_wheel_mean"]),
            "mask_imu_active": _series_stats(data["mask_imu_active"]),
            "mask_steer_active": _series_stats(data["mask_steer_active"]),
            "mask_wheel_active": _series_stats(data["mask_wheel_active"]),
            "meas_mask_x": _series_stats(data["meas_mask_x"]),
            "meas_mask_y": _series_stats(data["meas_mask_y"]),
            "meas_mask_psi": _series_stats(data["meas_mask_psi"]),
            "meas_mask_v": _series_stats(data["meas_mask_v"]),
            "meas_mask_w": _series_stats(data["meas_mask_w"]),
            "gps_age_sec": _series_stats(data["gps_age_sec"]),
            "sensor_failure_intensity": _series_stats(data["sensor_failure_intensity"]),
            "sensor_failure_imu_intensity": _series_stats(data["sensor_failure_imu_intensity"]),
            "sensor_failure_steer_intensity": _series_stats(data["sensor_failure_steer_intensity"]),
            "sensor_failure_wheel_intensity": _series_stats(data["sensor_failure_wheel_intensity"]),
            "sensor_failure_gps_intensity": _series_stats(data["sensor_failure_gps_intensity"]),
        },
        "highlights": {
            "max_position_error_norm": _series_peak(data["position_error_norm"], time_axis),
            "max_abs_heading_error": _series_peak(data["heading_error"], time_axis, absolute=True),
            "max_abs_velocity_error": _series_peak(data["velocity_error"], time_axis, absolute=True),
            "max_robust_ref_position_error_norm": _series_peak(
                data["robust_ref_position_error_norm"], time_axis
            ),
            "max_abs_robust_ref_heading_error": _series_peak(
                data["robust_ref_heading_error"], time_axis, absolute=True
            ),
            "max_abs_robust_ref_velocity_error": _series_peak(
                data["robust_ref_velocity_error"], time_axis, absolute=True
            ),
            "max_ekf_ref_position_error_norm": _series_peak(
                data["ekf_ref_position_error_norm"], time_axis
            ),
            "max_abs_ekf_ref_heading_error": _series_peak(
                data["ekf_ref_heading_error"], time_axis, absolute=True
            ),
            "max_abs_ekf_ref_velocity_error": _series_peak(
                data["ekf_ref_velocity_error"], time_axis, absolute=True
            ),
            "max_abs_K_norm_error": _series_peak(gain_errors["K_norm_error"], time_axis, absolute=True),
            "max_abs_K_x_x_error": _series_peak(gain_errors["K_x_x_error"], time_axis, absolute=True),
            "max_abs_K_y_y_error": _series_peak(gain_errors["K_y_y_error"], time_axis, absolute=True),
            "max_abs_K_psi_psi_error": _series_peak(
                gain_errors["K_psi_psi_error"], time_axis, absolute=True
            ),
            "max_abs_K_v_v_error": _series_peak(gain_errors["K_v_v_error"], time_axis, absolute=True),
            "max_attack_intensity": _series_peak(data["sensor_failure_intensity"], time_axis),
        },
        "trajectory_summary": {
            "robust_start_xy": [
                _safe_float(data["robust_x"][0]) if total_samples else None,
                _safe_float(data["robust_y"][0]) if total_samples else None,
            ],
            "robust_end_xy": [
                _safe_float(data["robust_x"][-1]) if total_samples else None,
                _safe_float(data["robust_y"][-1]) if total_samples else None,
            ],
            "ekf_start_xy": [
                _safe_float(data["ekf_x"][0]) if total_samples else None,
                _safe_float(data["ekf_y"][0]) if total_samples else None,
            ],
            "ekf_end_xy": [
                _safe_float(data["ekf_x"][-1]) if total_samples else None,
                _safe_float(data["ekf_y"][-1]) if total_samples else None,
            ],
            "reference_start_xy": [
                _safe_float(data["ref_x"][0]) if total_samples else None,
                _safe_float(data["ref_y"][0]) if total_samples else None,
            ],
            "reference_end_xy": [
                _safe_float(data["ref_x"][-1]) if total_samples else None,
                _safe_float(data["ref_y"][-1]) if total_samples else None,
            ],
        },
        "data_quality": {
            "finite_position_error_samples": _finite_count(data["position_error_norm"]),
            "finite_heading_error_samples": _finite_count(data["heading_error"]),
            "finite_velocity_error_samples": _finite_count(data["velocity_error"]),
            "finite_reference_x_samples": _finite_count(data["ref_x"]),
            "finite_robust_ref_position_error_samples": _finite_count(
                data["robust_ref_position_error_norm"]
            ),
            "finite_ekf_ref_position_error_samples": _finite_count(
                data["ekf_ref_position_error_norm"]
            ),
            "finite_mask_score_samples": _finite_count(data["mask_selected_score"]),
        },
    }
    return summary


def _add_attack_spans(ax: plt.Axes, attack_segments: List[Dict[str, Any]]) -> None:
    if not attack_segments:
        return
    colors = [
        "tab:red",
        "tab:orange",
        "tab:purple",
        "tab:brown",
        "tab:pink",
        "tab:gray",
    ]
    for idx, segment in enumerate(attack_segments):
        start = segment.get("start_time_seconds")
        end = segment.get("end_time_seconds")
        if start is None or end is None:
            continue
        ax.axvspan(
            float(start),
            float(end),
            color=colors[idx % len(colors)],
            alpha=0.08,
            linewidth=0.0,
        )


def _plot_attack_figure(
    filepath: Path,
    data: Dict[str, np.ndarray],
    time_axis: np.ndarray,
) -> plt.Figure:
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle(f"RKNet Comparator (3/3): Attack Timeline {filepath.name}", fontsize=14)

    attack_segments = _extract_attack_segments(data, time_axis)
    has_attack_metadata = (
        np.any(np.isfinite(data.get("sensor_failure_intensity", np.asarray([], dtype=np.float64))))
        or np.any(data.get("sensor_failure_active", np.asarray([], dtype=np.float64)) > 0.5)
        or any(_clean_label(v) for v in data.get("sensor_failure_branch_types", np.asarray([], dtype=object)))
        or any(_clean_label(v) for v in data.get("sensor_failure_gps_type", np.asarray([], dtype=object)))
    )

    if not has_attack_metadata:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "No attack metadata in this comparator CSV",
                transform=ax.transAxes,
                ha="center",
                va="center",
                fontsize=12,
            )
            ax.set_axis_off()
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        return fig

    ax = axes[0]
    ax.plot(
        time_axis,
        data["sensor_failure_active"],
        label="attack active",
        linewidth=1.2,
        color="black",
    )
    ax.plot(
        time_axis,
        data["sensor_failure_intensity"],
        label="overall intensity",
        linewidth=1.6,
        color="tab:red",
    )
    ax.set_title("Attack Activity and Overall Intensity")
    ax.set_ylabel("active / intensity")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    ax_twin = ax.twinx()
    ax_twin.plot(
        time_axis,
        data["sensor_failure_remaining_steps"],
        label="remaining steps",
        linewidth=1.1,
        color="tab:blue",
        linestyle="--",
        alpha=0.8,
    )
    ax_twin.set_ylabel("remaining steps")
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax_twin.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper left", fontsize=8)
    _add_attack_spans(ax, attack_segments)

    ax = axes[1]
    ax.plot(time_axis, data["sensor_failure_imu_intensity"], label="imu intensity", linewidth=1.3)
    ax.plot(time_axis, data["sensor_failure_steer_intensity"], label="steer intensity", linewidth=1.3)
    ax.plot(time_axis, data["sensor_failure_wheel_intensity"], label="wheel intensity", linewidth=1.3)
    ax.plot(time_axis, data["sensor_failure_gps_xy_intensity"], label="gps xy intensity", linewidth=1.3)
    ax.plot(
        time_axis,
        data["sensor_failure_gps_valid_flip"],
        label="gps valid flip",
        linewidth=1.1,
        linestyle=":",
        color="tab:gray",
    )
    ax.set_title("Per-Channel Attack Intensity")
    ax.set_ylabel("intensity")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=3, loc="upper right")
    _add_attack_spans(ax, attack_segments)

    ax = axes[2]
    type_code_map: Dict[str, float] = {}
    type_codes = np.full(time_axis.shape, np.nan, dtype=np.float64)
    for idx, (branch_types, gps_type, active) in enumerate(
        zip(
            data["sensor_failure_branch_types"],
            data["sensor_failure_gps_type"],
            data["sensor_failure_active"],
        )
    ):
        if not np.isfinite(active) or active <= 0.5:
            continue
        label = _attack_type_label(branch_types, gps_type)
        if label == "none":
            label = "active"
        if label not in type_code_map:
            type_code_map[label] = float(len(type_code_map))
        type_codes[idx] = type_code_map[label]
    ax.plot(time_axis, type_codes, linewidth=1.4, color="tab:purple")
    ax.set_title("Attack Type Timeline")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("type id")
    if type_code_map:
        ordered = sorted(type_code_map.items(), key=lambda item: item[1])
        ax.set_yticks([item[1] for item in ordered])
        ax.set_yticklabels([item[0] for item in ordered], fontsize=8)
    ax.grid(True, alpha=0.3)
    _add_attack_spans(ax, attack_segments)
    for segment in attack_segments:
        label = str(segment.get("dominant_type", "none"))
        start = segment.get("start_time_seconds")
        end = segment.get("end_time_seconds")
        peak = segment.get("peak_intensity")
        if start is None or end is None:
            continue
        center = 0.5 * (float(start) + float(end))
        text = label
        if peak is not None:
            text = f"{label}\npeak={float(peak):.3f}"
        ax.text(
            center,
            0.98,
            text,
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=7,
            bbox={"boxstyle": "round,pad=0.2", "facecolor": "white", "alpha": 0.6, "edgecolor": "none"},
        )

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    return fig


def _export_artifacts(
    filepath: Path,
    figures: Dict[str, plt.Figure],
    summary: Dict[str, Any],
    output_dir: Optional[str],
) -> Dict[str, str]:
    export_dir = Path(output_dir).expanduser() if output_dir else DEFAULT_EXPORT_DIR
    if not export_dir.is_absolute():
        export_dir = (Path.cwd() / export_dir).resolve()
    export_dir.mkdir(parents=True, exist_ok=True)

    base_name = filepath.stem
    json_path = export_dir / f"{base_name}_summary.json"
    artifacts: Dict[str, str] = {}
    for fig_key, figure in figures.items():
        figure_path = export_dir / f"{base_name}_{fig_key}.png"
        figure.savefig(figure_path, dpi=180, bbox_inches="tight")
        artifacts[f"{fig_key}_png"] = str(figure_path)
    artifacts["summary_json"] = str(json_path)
    summary["artifacts"] = dict(artifacts)

    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return artifacts


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot Robust KalmanNet vs EKF comparator logs"
    )
    parser.add_argument(
        "--file",
        help="Comparator CSV file. If omitted, the latest file under logs/comparator is used.",
    )
    parser.add_argument(
        "--output-dir",
        help="Directory where the PNG figures and JSON summary are saved.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save outputs without opening the Matplotlib window.",
    )
    args = parser.parse_args()

    filepath = _resolve_input_file(args.file)
    data = _load_csv_rows(filepath)
    time_axis = data["timestamp"] - data["timestamp"][0]
    reference_available = bool(np.any(np.isfinite(data.get("ref_x", np.asarray([], dtype=np.float64)))))
    attack_segments = _extract_attack_segments(data, time_axis)
    gain_errors = {
        "K_x_x": data["K_x_x"] - data["ekf_K_x_x"],
        "K_y_y": data["K_y_y"] - data["ekf_K_y_y"],
        "K_psi_psi": data["K_psi_psi"] - data["ekf_K_psi_psi"],
        "K_v_v": data["K_v_v"] - data["ekf_K_v_v"],
        "K_norm": data["K_norm"] - data["ekf_K_norm"],
    }

    fig1, axes1 = plt.subplots(4, 2, figsize=(13, 11))
    fig1.suptitle(f"RKNet Comparator (1/2): {filepath.name}", fontsize=14)

    # Figure 1, Row 0: Kalman Gain Diagonals side by side
    ax = axes1[0, 0]
    ax.plot(time_axis, data["K_x_x"], label="RKNet K(x,x)", linewidth=1.2)
    ax.plot(time_axis, data["K_y_y"], label="RKNet K(y,y)", linewidth=1.2)
    ax.plot(time_axis, data["K_psi_psi"], label="RKNet K(psi,psi)", linewidth=1.2)
    ax.plot(time_axis, data["K_v_v"], label="RKNet K(v,v)", linewidth=1.2)
    ax.set_title("RKNet Gain Diagonals")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes1[0, 1]
    ax.plot(time_axis, data["ekf_K_x_x"], label="EKF K(x,x)", linewidth=1.2)
    ax.plot(time_axis, data["ekf_K_y_y"], label="EKF K(y,y)", linewidth=1.2)
    ax.plot(time_axis, data["ekf_K_psi_psi"], label="EKF K(psi,psi)", linewidth=1.2)
    ax.plot(time_axis, data["ekf_K_v_v"], label="EKF K(v,v)", linewidth=1.2)
    ax.set_title("EKF Gain Diagonals")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Figure 1, Row 1: Gain Error and Mask Decision Summary
    ax = axes1[1, 0]
    ax.plot(time_axis, gain_errors["K_x_x"], label="K(x,x) error", linewidth=1.2)
    ax.plot(time_axis, gain_errors["K_y_y"], label="K(y,y) error", linewidth=1.2)
    ax.plot(time_axis, gain_errors["K_psi_psi"], label="K(psi,psi) error", linewidth=1.2)
    ax.plot(time_axis, gain_errors["K_v_v"], label="K(v,v) error", linewidth=1.2)
    ax.plot(time_axis, gain_errors["K_norm"], label="K norm error", linewidth=1.2, linestyle="--")
    ax.axhline(0.0, color="black", linewidth=0.8, linestyle=":", alpha=0.7)
    ax.set_title("Gain Error (RKNet - EKF)")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes1[1, 1]
    branch_to_id = {"imu": 0.0, "steer": 1.0, "wheel": 2.0}
    selected_ids = np.asarray(
        [branch_to_id.get(name, np.nan) for name in data["mask_selected_branch"]],
        dtype=np.float64,
    )
    ax.plot(time_axis, selected_ids, label="selected branch", linewidth=1.3)
    ax.plot(time_axis, data["mask_imu_mean"], label="pred imu mask", alpha=0.5, linestyle=":")
    ax.plot(time_axis, data["mask_wheel_mean"], label="pred wheel mask", alpha=0.5, linestyle=":")
    ax.set_title("Predictor Mask Summary (used only if 'nn' predictor)")
    ax.set_xlabel("time [s]")
    ax.set_yticks([0.0, 1.0, 2.0])
    ax.set_yticklabels(["imu", "steer", "wheel"])
    ax.set_ylim(-0.2, 2.2)
    ax.grid(True, alpha=0.3)
    ax1_twin = ax.twinx()
    ax1_twin.plot(
        time_axis,
        data["mask_selected_score"],
        label="selected score",
        linewidth=1.1,
        color="tab:red",
    )
    ax1_twin.set_ylim(-0.05, 1.05)
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax1_twin.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper right", fontsize=8)

    # Figure 1, Row 2: State Differences and Source/GPS Flags
    ax = axes1[2, 0]
    ax.plot(time_axis, data["delta_x"], label="dx")
    ax.plot(time_axis, data["delta_y"], label="dy")
    ax.plot(time_axis, data["delta_theta"], label="dtheta")
    ax.plot(time_axis, data["delta_v"], label="dv")
    ax.set_title("Signed State Differences (Robust - EKF)")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes1[2, 1]
    model_mask = data["source"] == "model"
    fallback_mask = data["source"] == "fallback"
    ax.plot(time_axis, model_mask.astype(float), label="model used", linewidth=1.2)
    ax.plot(time_axis, fallback_mask.astype(float), label="fallback used", linewidth=1.2)
    ax.plot(time_axis, data["gps_valid"], label="gps fresh", linewidth=1.2)
    ax.plot(time_axis, data["gps_hold_valid"], label="gps hold valid", linewidth=1.2, linestyle=":")
    ax.set_title("Source / GPS Flags")
    ax.set_xlabel("time [s]")
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Figure 1, Row 3: Measurement Update Masks and Innovations
    ax = axes1[3, 0]
    ax.plot(time_axis, data["meas_mask_x"], label="mask x (GPS)", linewidth=1.2)
    ax.plot(time_axis, data["meas_mask_y"], label="mask y (GPS)", linewidth=1.2)
    ax.plot(time_axis, data["meas_mask_psi"], label="mask psi (IMU)", linewidth=1.2)
    ax.plot(time_axis, data["meas_mask_v"], label="mask v (Wheel)", linewidth=1.2)
    ax.set_title("Measurement Update Masks (Attenuates Innovation)")
    ax.set_xlabel("time [s]")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes1[3, 1]
    ax.plot(time_axis, data["innov_x"], label="innov x", linewidth=1.2)
    ax.plot(time_axis, data["innov_y"], label="innov y", linewidth=1.2)
    ax.plot(time_axis, data["innov_psi"], label="innov psi", linewidth=1.2)
    ax.plot(time_axis, data["innov_v"], label="innov v", linewidth=1.2)
    ax.set_title("Innovations (z - H*x_pred)")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    fig1.tight_layout(rect=[0, 0, 1, 0.97])

    # --------------- FIGURE 2 ---------------
    fig2, axes2 = plt.subplots(4, 2, figsize=(12, 10))
    fig2.suptitle(f"RKNet Comparator (2/2): {filepath.name}", fontsize=14)

    # Figure 2, Row 0: Trajectory and Errors
    ax = axes2[0, 0]
    ax.plot(data["robust_x"], data["robust_y"], label="Robust (Update)", linewidth=1.8)
    ax.plot(data["pred_x"], data["pred_y"], label="Robust (Pred)", linewidth=1.2, linestyle=":")
    ax.plot(data["ekf_x"], data["ekf_y"], label="EKF", linewidth=1.4, linestyle="--")
    if reference_available:
        ax.plot(data["ref_x"], data["ref_y"], label="Clean Ref EKF", linewidth=1.4, linestyle="-.", color="black")
    ax.set_title("XY Trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes2[0, 1]
    ax.plot(time_axis, data["position_error_norm"], label="|Robust - EKF| pos", linewidth=1.2, linestyle="--")
    if reference_available:
        ax.plot(
            time_axis,
            data["robust_ref_position_error_norm"],
            label="|Robust - Clean Ref| pos",
            linewidth=1.6,
        )
        ax.plot(
            time_axis,
            data["ekf_ref_position_error_norm"],
            label="|EKF - Clean Ref| pos",
            linewidth=1.2,
            linestyle=":",
        )
    ax.plot(time_axis, np.abs(data["heading_error"]), label="|Robust - EKF| heading", alpha=0.45)
    if reference_available:
        ax.plot(
            time_axis,
            np.abs(data["robust_ref_heading_error"]),
            label="|Robust - Clean Ref| heading",
            alpha=0.75,
        )
        ax.plot(
            time_axis,
            np.abs(data["ekf_ref_heading_error"]),
            label="|EKF - Clean Ref| heading",
            alpha=0.55,
            linestyle=":",
        )
    ax.set_title("Absolute Errors / Clean Reference Deviation")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Error")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    _add_attack_spans(ax, attack_segments)

    # Figure 2, Row 1: Absolute state errors to clean EKF reference
    ax = axes2[1, 0]
    if reference_available:
        ax.plot(time_axis, np.abs(data["robust_ref_dx"]), label="|Robust - Clean Ref| x", linewidth=1.6)
        ax.plot(
            time_axis,
            np.abs(data["ekf_ref_dx"]),
            label="|EKF - Clean Ref| x",
            linewidth=1.3,
            linestyle="--",
        )
    else:
        ax.text(
            0.5,
            0.5,
            "No clean EKF reference available",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
        )
    ax.set_title("Absolute X Error to Clean EKF")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|error_x| [m]")
    ax.grid(True, alpha=0.3)
    if reference_available:
        ax.legend(fontsize=8)
    _add_attack_spans(ax, attack_segments)

    ax = axes2[1, 1]
    if reference_available:
        ax.plot(time_axis, np.abs(data["robust_ref_dy"]), label="|Robust - Clean Ref| y", linewidth=1.6)
        ax.plot(
            time_axis,
            np.abs(data["ekf_ref_dy"]),
            label="|EKF - Clean Ref| y",
            linewidth=1.3,
            linestyle="--",
        )
    else:
        ax.text(
            0.5,
            0.5,
            "No clean EKF reference available",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
        )
    ax.set_title("Absolute Y Error to Clean EKF")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|error_y| [m]")
    ax.grid(True, alpha=0.3)
    if reference_available:
        ax.legend(fontsize=8)
    _add_attack_spans(ax, attack_segments)

    # Figure 2, Row 2: Absolute state errors to clean EKF reference
    ax = axes2[2, 0]
    if reference_available:
        ax.plot(
            time_axis,
            np.abs(data["robust_ref_dtheta"]),
            label="|Robust - Clean Ref| theta",
            linewidth=1.6,
        )
        ax.plot(
            time_axis,
            np.abs(data["ekf_ref_dtheta"]),
            label="|EKF - Clean Ref| theta",
            linewidth=1.3,
            linestyle="--",
        )
    else:
        ax.text(
            0.5,
            0.5,
            "No clean EKF reference available",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
        )
    ax.set_title("Absolute Heading Error to Clean EKF")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|error_theta| [rad]")
    ax.grid(True, alpha=0.3)
    if reference_available:
        ax.legend(fontsize=8)
    _add_attack_spans(ax, attack_segments)

    ax = axes2[2, 1]
    if reference_available:
        ax.plot(time_axis, np.abs(data["robust_ref_dv"]), label="|Robust - Clean Ref| v", linewidth=1.6)
        ax.plot(
            time_axis,
            np.abs(data["ekf_ref_dv"]),
            label="|EKF - Clean Ref| v",
            linewidth=1.3,
            linestyle="--",
        )
    else:
        ax.text(
            0.5,
            0.5,
            "No clean EKF reference available",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
        )
    ax.set_title("Absolute Velocity Error to Clean EKF")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|error_v| [m/s]")
    ax.grid(True, alpha=0.3)
    if reference_available:
        ax.legend(fontsize=8)
    _add_attack_spans(ax, attack_segments)

    # Figure 2, Row 3: Control inputs
    ax = axes2[3, 0]
    ax.plot(time_axis, data["steering"], label="Steering", color="tab:orange")
    ax.set_title("Steering Angle over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("steering [rad]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    _add_attack_spans(ax, attack_segments)

    ax = axes2[3, 1]
    ax.plot(time_axis, data["throttle"], label="Throttle", color="tab:green")
    ax.set_title("Throttle over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("throttle")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    _add_attack_spans(ax, attack_segments)

    fig2.tight_layout(rect=[0, 0, 1, 0.97])
    fig3 = _plot_attack_figure(filepath, data, time_axis)
    summary = _compute_summary(filepath, data, time_axis)
    artifacts = _export_artifacts(
        filepath,
        {
            "figure1": fig1,
            "figure2": fig2,
            "attacks": fig3,
        },
        summary,
        args.output_dir,
    )

    print(f"Saved figure 1: {artifacts['figure1_png']}")
    print(f"Saved figure 2: {artifacts['figure2_png']}")
    print(f"Saved attack figure: {artifacts['attacks_png']}")
    print(f"Saved summary: {artifacts['summary_json']}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig1)
        plt.close(fig2)
        plt.close(fig3)


if __name__ == "__main__":
    main()
