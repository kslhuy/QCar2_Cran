"""
Plot `following_leader_control_vehicle_*.csv` logs.

These CSV files are written by `logging_utils.log_following_leader_control()`
when `fleet_config.yaml` has:

    logging:
      enable_following_leader_logging: true

Examples:
    python plot_following_leader_control.py
    python plot_following_leader_control.py --list
    python plot_following_leader_control.py --file ../data_logs/run_*/following_leader_control_vehicle_1.csv
    python plot_following_leader_control.py --save following_leader_control.png --no-show
    python plot_following_leader_control.py --metrics-csv following_metrics.csv --desired-distance 0.25 --time-headway 0.5
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
from collections import Counter
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def _script_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def _qcar_dir() -> str:
    return os.path.abspath(os.path.join(_script_dir(), ".."))


def _candidate_search_dirs(explicit_dir: Optional[str] = None) -> List[str]:
    if explicit_dir:
        return [os.path.abspath(explicit_dir)]

    qcar_dir = _qcar_dir()
    project_dir = os.path.abspath(os.path.join(qcar_dir, ".."))
    return [
        os.getcwd(),
        os.path.join(qcar_dir, "data_logs"),
        os.path.join(qcar_dir, "logs"),
        qcar_dir,
        project_dir,
    ]


def _find_log_files(explicit_dir: Optional[str] = None) -> List[str]:
    seen = set()
    files: List[str] = []
    for search_dir in _candidate_search_dirs(explicit_dir):
        if not os.path.isdir(search_dir):
            continue
        pattern = os.path.join(search_dir, "**", "following_leader_control_vehicle_*.csv")
        for path in glob.glob(pattern, recursive=True):
            abs_path = os.path.abspath(path)
            if abs_path not in seen:
                seen.add(abs_path)
                files.append(abs_path)
    return sorted(files, key=os.path.getmtime, reverse=True)


def _expand_file_argument(path: str) -> str:
    matches = glob.glob(path)
    if matches:
        return os.path.abspath(sorted(matches, key=os.path.getmtime, reverse=True)[0])
    return os.path.abspath(path)


def _load_csv(filepath: str) -> Tuple[List[str], List[Dict[str, str]]]:
    cleaned_lines: List[str] = []
    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.replace("\x00", "")
            if len(line) < 131072:
                cleaned_lines.append(line)

    reader = csv.DictReader(cleaned_lines)
    columns = reader.fieldnames or []
    rows = list(reader)
    return columns, rows


def _col_to_float(rows: Sequence[Dict[str, str]], col: str) -> np.ndarray:
    values = []
    for row in rows:
        raw = row.get(col, "")
        try:
            value = float(raw)
        except (TypeError, ValueError):
            value = float("nan")
        values.append(value)
    return np.asarray(values, dtype=float)


def _col_to_text(rows: Sequence[Dict[str, str]], col: str) -> List[str]:
    return ["" if row.get(col) is None else str(row.get(col, "")).strip() for row in rows]


def _finite_stats(values: np.ndarray) -> Optional[Tuple[float, float, float]]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.min(finite)), float(np.mean(finite)), float(np.max(finite))


def _format_stats(label: str, values: np.ndarray, unit: str = "") -> str:
    stats = _finite_stats(values)
    if stats is None:
        return f"{label}: no finite data"
    suffix = f" {unit}" if unit else ""
    return f"{label}: min={stats[0]:.4g}, mean={stats[1]:.4g}, max={stats[2]:.4g}{suffix}"


def _finite_values(values: np.ndarray) -> np.ndarray:
    return values[np.isfinite(values)]


def _rmse(values: np.ndarray) -> float:
    finite = _finite_values(values)
    if finite.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(finite * finite)))


def _mean_abs(values: np.ndarray) -> float:
    finite = _finite_values(values)
    if finite.size == 0:
        return float("nan")
    return float(np.mean(np.abs(finite)))


def _max_abs(values: np.ndarray) -> float:
    finite = _finite_values(values)
    if finite.size == 0:
        return float("nan")
    return float(np.max(np.abs(finite)))


def _std(values: np.ndarray) -> float:
    finite = _finite_values(values)
    if finite.size < 2:
        return float("nan")
    return float(np.std(finite))


def _safe_rate(t: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Return d(values)/dt with finite interpolation across short NaN gaps."""
    out = np.full_like(values, np.nan, dtype=float)
    mask = np.isfinite(t) & np.isfinite(values)
    if np.count_nonzero(mask) < 2:
        return out
    t_f = t[mask]
    if np.unique(t_f).size < 2:
        return out
    y_interp = np.interp(t, t_f, values[mask])
    try:
        rate = np.gradient(y_interp, t)
    except Exception:
        return out
    out[np.isfinite(t)] = rate[np.isfinite(t)]
    return out


def _choose_baseline(primary: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    return np.where(np.isfinite(primary), primary, fallback)


def _plot_array(ax, t: np.ndarray, y: np.ndarray, label: str, **kwargs) -> bool:
    if not np.any(np.isfinite(y)):
        return False
    ax.plot(t, y, label=label, **kwargs)
    return True


def _write_metrics_csv(path: str, metrics: Sequence[Tuple[str, float, str, str]]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value", "unit", "note"])
        for name, value, unit, note in metrics:
            writer.writerow([name, value, unit, note])


def _print_metrics_matrix(metrics: Sequence[Tuple[str, float, str, str]]) -> None:
    print("\nPerformance metrics:")
    print(f"{'metric':34s} {'value':>12s} {'unit':8s} note")
    print("-" * 78)
    for name, value, unit, note in metrics:
        value_text = "nan" if not np.isfinite(value) else f"{value:.5g}"
        print(f"{name:34s} {value_text:>12s} {unit:8s} {note}")


def _relative_time(rows: Sequence[Dict[str, str]]) -> np.ndarray:
    t = _col_to_float(rows, "timestamp")
    finite = t[np.isfinite(t)]
    if finite.size:
        t = t - finite[0]
    return t


def _plot_series(ax, t: np.ndarray, rows: Sequence[Dict[str, str]], col: str, label: str, **kwargs) -> bool:
    y = _col_to_float(rows, col)
    if not np.any(np.isfinite(y)):
        return False
    ax.plot(t, y, label=label, **kwargs)
    return True


def _policy_codes(policies: Sequence[str]) -> Tuple[np.ndarray, Dict[str, int]]:
    clean = [policy if policy else "unlogged" for policy in policies]
    unique = sorted(set(clean))
    mapping = {policy: idx for idx, policy in enumerate(unique)}
    values = np.asarray([mapping[policy] for policy in clean], dtype=float)
    return values, mapping


def _build_performance_metrics(
    t: np.ndarray,
    rows: Sequence[Dict[str, str]],
    desired_distance: float,
    time_headway: float,
) -> Tuple[List[Tuple[str, float, str, str]], Dict[str, np.ndarray]]:
    follower_v = _col_to_float(rows, "follower_velocity")
    used_leader_v = _col_to_float(rows, "leader_velocity")
    clean_leader_v = _col_to_float(rows, "leader_clean_velocity")
    base_leader_v = _choose_baseline(clean_leader_v, used_leader_v)

    used_distance = _col_to_float(rows, "distance_to_leader")
    clean_distance = _col_to_float(rows, "leader_clean_distance")
    base_distance = _choose_baseline(clean_distance, used_distance)
    along_track_gap = _col_to_float(rows, "along_track_gap")
    clean_along_track_gap = _col_to_float(rows, "leader_clean_along_track_gap")
    base_along_track_gap = _choose_baseline(clean_along_track_gap, along_track_gap)

    desired_gap = desired_distance + time_headway * np.maximum(follower_v, 0.0)
    spacing_error = base_distance - desired_gap
    along_spacing_error = base_along_track_gap - desired_gap

    throttle = _col_to_float(rows, "throttle_u")
    steering = _col_to_float(rows, "steering_delta")
    raw_throttle = _col_to_float(rows, "raw_throttle_u")
    leader_pos_err = _col_to_float(rows, "leader_pos_err_vs_clean")
    leader_vel_err = _col_to_float(rows, "leader_vel_err_vs_clean")
    leader_heading_err = _col_to_float(rows, "leader_heading_err_vs_clean")
    distance_err_vs_clean = _col_to_float(rows, "distance_err_vs_clean")

    throttle_rate = _safe_rate(t, throttle)
    steering_rate = _safe_rate(t, steering)
    follower_accel = _safe_rate(t, follower_v)
    follower_jerk = _safe_rate(t, follower_accel)
    closing_speed = follower_v - base_leader_v

    leader_std = _std(base_leader_v)
    follower_std = _std(follower_v)
    string_velocity_gain = (
        follower_std / leader_std if np.isfinite(leader_std) and leader_std > 1e-6 else float("nan")
    )

    unsafe_gap_margin = 0.0
    unsafe_mask = np.isfinite(spacing_error) & (spacing_error < -unsafe_gap_margin)
    finite_spacing = np.isfinite(spacing_error)
    unsafe_pct = (
        100.0 * float(np.count_nonzero(unsafe_mask)) / float(np.count_nonzero(finite_spacing))
        if np.count_nonzero(finite_spacing) > 0
        else float("nan")
    )

    metrics = [
        ("gap_rmse_clean_baseline", _rmse(spacing_error), "m", "distance - desired gap"),
        ("gap_mean_abs_error", _mean_abs(spacing_error), "m", "smaller is better"),
        ("gap_std", _std(base_distance), "m", "distance oscillation"),
        ("gap_min", float(np.nanmin(base_distance)) if np.any(np.isfinite(base_distance)) else float("nan"), "m", "minimum clean/used gap"),
        ("unsafe_gap_percent", unsafe_pct, "%", "gap below desired"),
        ("along_track_gap_rmse", _rmse(along_spacing_error), "m", "uses path projection when logged"),
        ("velocity_error_rmse", _rmse(follower_v - base_leader_v), "m/s", "follower - clean leader"),
        ("closing_speed_rms", _rmse(closing_speed), "m/s", "positive means closing"),
        ("string_velocity_gain", string_velocity_gain, "-", "std(follower v) / std(clean leader v)"),
        ("throttle_rate_rms", _rmse(throttle_rate), "1/s", "command smoothness"),
        ("throttle_rate_max_abs", _max_abs(throttle_rate), "1/s", "command spike indicator"),
        ("steering_rate_rms", _rmse(steering_rate), "rad/s", "steering smoothness"),
        ("follower_accel_rms", _rmse(follower_accel), "m/s^2", "velocity derivative"),
        ("follower_jerk_rms", _rmse(follower_jerk), "m/s^3", "acceleration derivative"),
        ("raw_to_final_throttle_rmse", _rmse(raw_throttle - throttle), "-", "smoothing effect"),
        ("leader_pos_err_vs_clean_rmse", _rmse(leader_pos_err), "m", "used leader state vs clean V2V"),
        ("leader_vel_err_vs_clean_rmse", _rmse(leader_vel_err), "m/s", "used leader velocity vs clean V2V"),
        ("leader_heading_err_vs_clean_rmse", _rmse(leader_heading_err), "rad", "used leader heading vs clean V2V"),
        ("distance_err_vs_clean_rmse", _rmse(distance_err_vs_clean), "m", "used distance - clean distance"),
    ]

    series = {
        "follower_v": follower_v,
        "used_leader_v": used_leader_v,
        "clean_leader_v": clean_leader_v,
        "base_leader_v": base_leader_v,
        "used_distance": used_distance,
        "clean_distance": clean_distance,
        "base_distance": base_distance,
        "desired_gap": desired_gap,
        "spacing_error": spacing_error,
        "along_track_gap": along_track_gap,
        "clean_along_track_gap": clean_along_track_gap,
        "base_along_track_gap": base_along_track_gap,
        "along_spacing_error": along_spacing_error,
        "throttle": throttle,
        "steering": steering,
        "raw_throttle": raw_throttle,
        "throttle_rate": throttle_rate,
        "steering_rate": steering_rate,
        "follower_accel": follower_accel,
        "follower_jerk": follower_jerk,
        "closing_speed": closing_speed,
        "leader_pos_err": leader_pos_err,
        "leader_vel_err": leader_vel_err,
        "leader_heading_err": leader_heading_err,
        "distance_err_vs_clean": distance_err_vs_clean,
    }
    return metrics, series


def _style_time_axis(ax, title: str, ylabel: str) -> None:
    ax.set_title(title, fontsize=10, fontweight="bold")
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)


def _print_summary(filepath: str, columns: Sequence[str], rows: Sequence[Dict[str, str]]) -> None:
    print(f"File: {filepath}")
    print(f"Rows: {len(rows)}")
    print(f"Columns: {len(columns)}")

    if not rows:
        return

    t = _relative_time(rows)
    finite_t = t[np.isfinite(t)]
    if finite_t.size:
        print(f"Duration: {finite_t[-1] - finite_t[0]:.3f} s")

    for label, col, unit in [
        ("Leader trust", "leader_trust", ""),
        ("Fusion alpha", "trust_fusion_alpha", ""),
        ("Final throttle", "throttle_u", ""),
        ("Raw throttle", "raw_throttle_u", ""),
        ("CACC throttle", "u_cacc", ""),
        ("Sensor throttle", "u_sensor", ""),
        ("Distance to leader", "distance_to_leader", "m"),
        ("Velocity difference", "velocity_difference", "m/s"),
    ]:
        if col in columns:
            print(_format_stats(label, _col_to_float(rows, col), unit))

    policies = [p for p in _col_to_text(rows, "trust_fusion_policy") if p]
    if policies:
        print("Policy counts:")
        for policy, count in Counter(policies).most_common():
            print(f"  {policy}: {count}")

    sources = [s for s in _col_to_text(rows, "leader_source") if s]
    if sources:
        print("Leader source counts:")
        for source, count in Counter(sources).most_common():
            print(f"  {source}: {count}")

    clean_available = _col_to_float(rows, "leader_clean_available")
    if np.any(np.isfinite(clean_available)):
        pct = 100.0 * np.count_nonzero(clean_available > 0.5) / max(len(clean_available), 1)
        print(f"Clean V2V baseline coverage: {pct:.1f}%")


def plot_following_leader_control(
    filepath: str,
    trust_low: Optional[float] = None,
    trust_high: Optional[float] = None,
    desired_distance: float = 0.25,
    time_headway: float = 0.5,
    metrics_csv: Optional[str] = None,
    save_path: Optional[str] = None,
    show: bool = True,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError("Matplotlib is required to plot this CSV.") from exc

    columns, rows = _load_csv(filepath)
    _print_summary(filepath, columns, rows)
    if not rows:
        raise RuntimeError("CSV has no rows to plot.")

    t = _relative_time(rows)
    policies = _col_to_text(rows, "trust_fusion_policy")
    policy_values, policy_map = _policy_codes(policies)
    metrics, series = _build_performance_metrics(
        t, rows, desired_distance=desired_distance, time_headway=time_headway
    )
    _print_metrics_matrix(metrics)
    if metrics_csv:
        _write_metrics_csv(metrics_csv, metrics)
        print(f"Saved metrics: {os.path.abspath(metrics_csv)}")

    fig, axes = plt.subplots(4, 3, figsize=(18, 12), constrained_layout=True)
    fig.suptitle(os.path.basename(filepath), fontsize=13, fontweight="bold")

    ax = axes[0, 0]
    plotted = [
        _plot_array(ax, t, series["throttle"], "final u", color="tab:blue", lw=1.6),
        _plot_array(ax, t, series["raw_throttle"], "raw u", color="tab:cyan", lw=1.0, alpha=0.8),
        _plot_series(ax, t, rows, "u_cacc", "u_cacc", color="tab:orange", lw=1.1, ls="--"),
        _plot_series(ax, t, rows, "u_sensor", "u_sensor", color="tab:green", lw=1.1, ls=":"),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "Longitudinal Commands", "throttle / command")

    ax = axes[0, 1]
    has_trust = _plot_series(ax, t, rows, "leader_trust", "leader trust", color="tab:purple", lw=1.5)
    has_alpha = _plot_series(ax, t, rows, "trust_fusion_alpha", "fusion alpha", color="tab:red", lw=1.2)
    if trust_low is not None:
        ax.axhline(float(trust_low), color="k", ls=":", lw=0.9, alpha=0.7, label="trust_low")
    if trust_high is not None:
        ax.axhline(float(trust_high), color="gray", ls=":", lw=0.9, alpha=0.7, label="trust_high")
    if has_trust or has_alpha or trust_low is not None or trust_high is not None:
        ax.legend(loc="best", fontsize=8)
    ax.set_ylim(-0.05, 1.05)
    _style_time_axis(ax, "Trust Fusion", "value [0,1]")

    ax = axes[0, 2]
    plotted = [
        _plot_array(ax, t, series["used_distance"], "used distance", color="tab:blue", lw=1.2),
        _plot_array(ax, t, series["clean_distance"], "clean V2V distance", color="tab:green", lw=1.4),
        _plot_array(ax, t, series["base_along_track_gap"], "path gap baseline", color="tab:orange", lw=1.0, ls="--"),
        _plot_array(ax, t, series["desired_gap"], "desired gap", color="k", lw=1.0, ls=":"),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "Gap Tracking", "distance [m]")

    ax = axes[1, 0]
    plotted = [
        _plot_array(ax, t, series["follower_v"], "follower v", color="tab:blue", lw=1.3),
        _plot_array(ax, t, series["used_leader_v"], "used leader v", color="tab:orange", lw=1.0, ls="--"),
        _plot_array(ax, t, series["clean_leader_v"], "clean leader v", color="tab:green", lw=1.3),
        _plot_series(ax, t, rows, "follower_target_velocity", "target v", color="gray", lw=1.0, ls=":"),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "Velocity Tracking", "velocity [m/s]")

    ax = axes[1, 1]
    plotted = [
        _plot_array(ax, t, series["spacing_error"], "gap error", color="tab:blue", lw=1.4),
        _plot_array(ax, t, series["along_spacing_error"], "path gap error", color="tab:orange", lw=1.0, ls="--"),
        _plot_array(ax, t, np.zeros_like(t), "zero", color="k", lw=0.8, ls=":"),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "Spacing Error", "gap - desired [m]")

    ax = axes[1, 2]
    plotted = [
        _plot_array(ax, t, series["closing_speed"], "closing speed", color="tab:red", lw=1.2),
        _plot_array(ax, t, series["follower_accel"], "follower accel", color="tab:blue", lw=1.1, ls="--"),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "String Stability Signals", "m/s or m/s^2")

    ax = axes[2, 0]
    plotted = [
        _plot_array(ax, t, series["leader_pos_err"], "position error", color="tab:blue", lw=1.2),
        _plot_array(ax, t, np.abs(series["leader_vel_err"]), "|velocity error|", color="tab:orange", lw=1.1),
        _plot_array(ax, t, np.abs(series["leader_heading_err"]), "|heading error|", color="tab:green", lw=1.1),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "Used Leader vs Clean V2V", "error")

    ax = axes[2, 1]
    plotted = [
        _plot_array(ax, t, series["throttle_rate"], "du/dt", color="tab:blue", lw=1.1),
        _plot_array(ax, t, series["steering_rate"], "dsteer/dt", color="tab:orange", lw=1.1),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "Command Rates", "rate")

    ax = axes[2, 2]
    plotted = [
        _plot_series(ax, t, rows, "multi_predecessor_velocity_term", "multi-pred v term", color="tab:blue", lw=1.1),
        _plot_series(ax, t, rows, "multi_predecessor_acceleration_term", "multi-pred a term", color="tab:orange", lw=1.1),
        _plot_series(ax, t, rows, "multi_predecessor_weight_sum", "multi-pred weight", color="tab:green", lw=1.1),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    _style_time_axis(ax, "Multi-Predecessor Feedforward", "value")

    ax = axes[3, 0]
    follower_x = _col_to_float(rows, "follower_x")
    follower_y = _col_to_float(rows, "follower_y")
    leader_x = _col_to_float(rows, "leader_x")
    leader_y = _col_to_float(rows, "leader_y")
    clean_x = _col_to_float(rows, "leader_clean_x")
    clean_y = _col_to_float(rows, "leader_clean_y")
    if np.any(np.isfinite(follower_x) & np.isfinite(follower_y)):
        ax.plot(follower_x, follower_y, label="follower", color="tab:blue", lw=1.4)
        ax.scatter(follower_x[0], follower_y[0], color="tab:blue", s=20, marker="o")
    if np.any(np.isfinite(leader_x) & np.isfinite(leader_y)):
        ax.plot(leader_x, leader_y, label="used leader", color="tab:orange", lw=1.1, ls="--")
        ax.scatter(leader_x[0], leader_y[0], color="tab:orange", s=20, marker="o")
    if np.any(np.isfinite(clean_x) & np.isfinite(clean_y)):
        ax.plot(clean_x, clean_y, label="clean leader", color="tab:green", lw=1.4)
        ax.scatter(clean_x[0], clean_y[0], color="tab:green", s=20, marker="o")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("XY Trajectory", fontsize=10, fontweight="bold")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax = axes[3, 1]
    ax.step(t, policy_values, where="post", color="tab:brown", lw=1.4)
    ax.set_yticks(list(policy_map.values()))
    ax.set_yticklabels(list(policy_map.keys()), fontsize=8)
    _style_time_axis(ax, "Fusion Policy", "policy")

    ax = axes[3, 2]
    clean_available = _col_to_float(rows, "leader_clean_available")
    source_values, source_map = _policy_codes(_col_to_text(rows, "leader_source"))
    plotted = [
        _plot_array(ax, t, clean_available, "clean baseline available", color="tab:green", lw=1.1),
        _plot_array(ax, t, source_values, "leader source code", color="tab:purple", lw=1.1, ls="--"),
    ]
    if any(plotted):
        ax.legend(loc="best", fontsize=8)
    if source_map:
        ax.text(
            0.01,
            0.99,
            "\n".join(f"{v}: {k}" for k, v in source_map.items()),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.7, "edgecolor": "none"},
        )
    _style_time_axis(ax, "Baseline Availability / Source", "code")

    if save_path:
        fig.savefig(save_path, dpi=160)
        print(f"Saved figure: {os.path.abspath(save_path)}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def _select_file(args) -> Optional[str]:
    if args.file:
        path = _expand_file_argument(args.file)
        if os.path.isfile(path):
            return path
        raise FileNotFoundError(f"CSV file not found: {path}")

    files = _find_log_files(args.dir)
    if args.list:
        if not files:
            print("No following_leader_control_vehicle_*.csv files found.")
            return None
        for idx, path in enumerate(files):
            mtime = os.path.getmtime(path)
            print(f"[{idx}] {path}  ({mtime:.0f})")
        return None

    if not files:
        return None

    if args.pick is not None:
        if args.pick < 0 or args.pick >= len(files):
            raise IndexError(f"--pick must be in [0, {len(files) - 1}]")
        return files[args.pick]

    return files[0]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot following_leader_control_vehicle_*.csv logs."
    )
    parser.add_argument("--file", help="CSV file to plot. Globs are allowed.")
    parser.add_argument("--dir", help="Directory to search recursively for logs.")
    parser.add_argument("--list", action="store_true", help="List matching CSV files and exit.")
    parser.add_argument("--pick", type=int, help="Pick N from the newest-first file list.")
    parser.add_argument("--save", help="Save the figure to this image path.")
    parser.add_argument("--metrics-csv", help="Optional path to save the performance metric table.")
    parser.add_argument("--no-show", action="store_true", help="Do not open the interactive plot window.")
    parser.add_argument("--trust-low", type=float, default=0.50, help="Draw trust_low threshold.")
    parser.add_argument("--trust-high", type=float, default=0.80, help="Draw trust_high threshold.")
    parser.add_argument("--desired-distance", type=float, default=0.25,
                        help="Base desired following distance used for gap metrics [m].")
    parser.add_argument("--time-headway", type=float, default=0.5,
                        help="Desired gap headway: desired = distance + headway * follower_speed [s].")
    args = parser.parse_args()

    try:
        filepath = _select_file(args)
        if filepath is None:
            if not args.list:
                print("No following_leader_control_vehicle_*.csv files found.")
                print("Enable it in qcar/fleet_config.yaml:")
                print("  logging:")
                print("    enable_following_leader_logging: true")
            return 1 if not args.list else 0

        plot_following_leader_control(
            filepath,
            trust_low=args.trust_low,
            trust_high=args.trust_high,
            desired_distance=args.desired_distance,
            time_headway=args.time_headway,
            metrics_csv=args.metrics_csv,
            save_path=args.save,
            show=not args.no_show,
        )
    except Exception as exc:
        print(f"Error: {exc}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
