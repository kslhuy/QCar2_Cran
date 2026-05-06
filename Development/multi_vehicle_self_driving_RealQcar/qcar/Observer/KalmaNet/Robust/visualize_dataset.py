import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from robust_kalmannet_dataset import _rebuild_measurements

"""
Examples:
python .\visualize_dataset.py
python .\visualize_dataset.py .\datasets\your_dataset.npz
python .\visualize_dataset.py .\datasets\your_dataset.json
"""


STATE_LABELS = ("x", "y", "theta", "v")
STATE_UNITS = ("m", "m", "rad", "m/s")
GPS_INVALID_SHADE_COLOR = "#ffb347"
GYRO_COLOR = "#005f73"
MOTOR_TACH_COLOR = "#d00000"
SUPPORTED_HEADING_REBUILD_MODES = ("qcar_ekf", "gyro_filter", "kinematic")
Z_VARIANT_SPECS = {
    "raw_saved": {"label": "z_raw_saved", "color": "tab:blue", "linestyle": "-", "linewidth": 0.95},
    "rebuilt_qcar_ekf": {"label": "z_rebuilt_qcar_ekf", "color": "tab:cyan", "linestyle": "-", "linewidth": 1.05},
    "rebuilt_gyro_filter": {"label": "z_rebuilt_gyro", "color": "tab:purple", "linestyle": "--", "linewidth": 1.0},
    "rebuilt_kinematic": {"label": "z_rebuilt_kinematic", "color": "tab:green", "linestyle": "-", "linewidth": 1.0},
}


def _wrap_angle_array(angle_array):
    return (angle_array + np.pi) % (2.0 * np.pi) - np.pi


def _decode_metadata(data):
    if "metadata_json" not in data.files:
        return {}

    raw_value = data["metadata_json"]
    meta_str = str(raw_value.tolist()) if hasattr(raw_value, "tolist") else str(raw_value)
    try:
        return json.loads(meta_str)
    except json.JSONDecodeError:
        return {"metadata_json_raw": meta_str}


def _resolve_dataset_path(target_file):
    path = Path(target_file)
    script_dir = Path(__file__).resolve().parent

    candidate_paths = []
    if path.is_absolute():
        candidate_paths.append(path)
    elif path.anchor in ("\\", "/"):
        # Windows paths like "\datasets\file.npz" are drive-rooted. For this tool,
        # interpret them as project-local shorthand instead of "C:\datasets\...".
        candidate_paths.append(script_dir / str(path).lstrip("\\/"))
        candidate_paths.append(Path.cwd() / str(path).lstrip("\\/"))
    else:
        candidate_paths.append(Path.cwd() / path)
        candidate_paths.append(script_dir / path)
        candidate_paths.append(script_dir / "datasets" / path.name)

    path = next((candidate for candidate in candidate_paths if candidate.exists()), candidate_paths[0])

    if path.suffix.lower() == ".json":
        npz_candidate = path.with_suffix(".npz")
        if npz_candidate.exists():
            return npz_candidate
        raise FileNotFoundError(f"Found JSON metadata but missing NPZ payload: {npz_candidate}")

    return path


def _load_array(data, key, default):
    if key not in data.files:
        return np.asarray(default)
    return np.asarray(data[key])


def _load_state_matrix(data, key, sample_count):
    arr = _load_array(
        data,
        key,
        np.zeros((sample_count, len(STATE_LABELS)), dtype=np.float32),
    )
    arr = np.asarray(arr, dtype=np.float32).reshape(sample_count, -1)
    if arr.shape[1] < len(STATE_LABELS):
        padded = np.zeros((sample_count, len(STATE_LABELS)), dtype=np.float32)
        padded[:, : arr.shape[1]] = arr
        return padded
    return arr[:, : len(STATE_LABELS)].copy()


def _normalize_heading_rebuild_modes(modes):
    if modes is None:
        return list(SUPPORTED_HEADING_REBUILD_MODES)

    normalized = []
    seen = set()
    for mode in modes:
        mode_norm = str(mode).strip().lower()
        if not mode_norm:
            continue
        if mode_norm not in SUPPORTED_HEADING_REBUILD_MODES:
            raise ValueError(
                f"Unsupported heading rebuild mode '{mode}'. "
                f"Expected one of: {', '.join(SUPPORTED_HEADING_REBUILD_MODES)}"
            )
        if mode_norm in seen:
            continue
        seen.add(mode_norm)
        normalized.append(mode_norm)
    return normalized or list(SUPPORTED_HEADING_REBUILD_MODES)


def _rebuild_variant_key(mode):
    return f"rebuilt_{str(mode).strip().lower()}"


def _add_invalid_gps_shading(ax, t_rel, gps_valid):
    if t_rel.size == 0 or gps_valid.size == 0:
        return

    invalid = gps_valid < 0.5
    if not np.any(invalid):
        return

    start_idx = None
    for idx, is_invalid in enumerate(invalid):
        if is_invalid and start_idx is None:
            start_idx = idx
        if not is_invalid and start_idx is not None:
            ax.axvspan(t_rel[start_idx], t_rel[idx], color=GPS_INVALID_SHADE_COLOR, alpha=0.12)
            start_idx = None

    if start_idx is not None:
        ax.axvspan(t_rel[start_idx], t_rel[-1], color=GPS_INVALID_SHADE_COLOR, alpha=0.12)


def _state_delta(lhs, rhs):
    lhs_arr = np.asarray(lhs, dtype=np.float32)
    rhs_arr = np.asarray(rhs, dtype=np.float32)
    dim = min(lhs_arr.shape[-1], rhs_arr.shape[-1], len(STATE_LABELS))
    delta = lhs_arr[..., :dim] - rhs_arr[..., :dim]
    if delta.ndim == 2 and delta.shape[1] > 2:
        delta[:, 2] = _wrap_angle_array(delta[:, 2])
    return delta


def _state_metric_summary(delta):
    delta_abs = np.abs(np.asarray(delta, dtype=np.float32))
    rms = np.sqrt(np.mean(np.square(delta_abs), axis=0))
    mean_abs = np.mean(delta_abs, axis=0)
    max_abs = np.max(delta_abs, axis=0)
    return {
        "max_abs": max_abs,
        "rms": rms,
        "mean_abs": mean_abs,
    }


def _format_metric_vector(values):
    return ", ".join(
        f"{STATE_LABELS[idx]}={float(values[idx]):.4f}"
        for idx in range(min(len(values), len(STATE_LABELS)))
    )


def _plot_states(fig, t_rel, x_gt, z_variants, gps_valid):
    dim = min(x_gt.shape[1], len(STATE_LABELS))
    axes = np.atleast_1d(fig.subplots(dim, 1, sharex=True))
    for idx, axis in enumerate(axes):
        axis.plot(t_rel, x_gt[:, idx], label="x_gt", color="tab:red", linewidth=1.2)
        for variant_name, variant_values in z_variants.items():
            spec = Z_VARIANT_SPECS[variant_name]
            axis.plot(
                t_rel,
                variant_values[:, idx],
                label=spec["label"],
                color=spec["color"],
                linewidth=spec["linewidth"],
                linestyle=spec["linestyle"],
                alpha=0.9,
            )
        axis.set_ylabel(f"{STATE_LABELS[idx]} ({STATE_UNITS[idx]})")
        axis.grid(True, alpha=0.3)
        if idx in (0, 2):
            _add_invalid_gps_shading(axis, t_rel, gps_valid)
        if idx == 0:
            axis.legend(loc="upper right")
    axes[0].set_title("State Comparison: Raw and All Rebuild Types")
    axes[-1].set_xlabel("Time (s)")


def _plot_sensor_channels(fig, t_rel, steering, throttle, gps_valid, gps_x, gps_y, gps_theta, accel_x, accel_y, accel_z, gyro_z, motor_tach, vfl, vfr, vrl, vrr):
    axes = fig.subplots(5, 1, sharex=True)

    axes[0].step(t_rel, gps_valid, where="post", color="tab:green", linewidth=1.2)
    axes[0].set_title("GPS Validity")
    axes[0].set_ylabel("valid")
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_rel, gps_x, label="gps_x", linewidth=1.0)
    axes[1].plot(t_rel, gps_y, label="gps_y", linewidth=1.0)
    axes[1].plot(t_rel, gps_theta, label="gps_theta", linewidth=1.0)
    axes[1].set_title("GPS Channels")
    axes[1].set_ylabel("value")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)
    _add_invalid_gps_shading(axes[1], t_rel, gps_valid)

    axes[2].plot(t_rel, accel_x, label="accel_x", linewidth=1.0)
    axes[2].plot(t_rel, accel_y, label="accel_y", linewidth=1.0)
    axes[2].plot(t_rel, accel_z, label="accel_z", linewidth=1.0)
    axes[2].set_title("Accelerometer")
    axes[2].set_ylabel("m/s^2")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t_rel, gyro_z, label="gyro_z", color=GYRO_COLOR, linewidth=1.2)
    axes[3].plot(t_rel, motor_tach, label="motor_tach", color=MOTOR_TACH_COLOR, linewidth=1.2, alpha=0.95)
    axes[3].set_title("Yaw Rate and Motor Tach")
    axes[3].set_ylabel("value")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t_rel, steering, label="steering", linewidth=1.0)
    axes[4].plot(t_rel, throttle, label="throttle", linewidth=1.0)
    axes[4].plot(t_rel, vfl, label="vfl", linewidth=0.9, alpha=0.8)
    axes[4].plot(t_rel, vfr, label="vfr", linewidth=0.9, alpha=0.8)
    axes[4].plot(t_rel, vrl, label="vrl", linewidth=0.9, alpha=0.8)
    axes[4].plot(t_rel, vrr, label="vrr", linewidth=0.9, alpha=0.8)
    axes[4].set_title("Control Inputs and Wheel Speeds")
    axes[4].set_ylabel("command / speed")
    axes[4].set_xlabel("Time (s)")
    axes[4].legend(loc="upper right", ncol=3)
    axes[4].grid(True, alpha=0.3)


def _plot_measurement_error(fig, t_rel, x_gt, z_variants, gps_valid):
    dim = min(x_gt.shape[1], len(STATE_LABELS))
    axes = np.atleast_1d(fig.subplots(dim, 1, sharex=True))

    for idx, axis in enumerate(axes):
        for variant_name, variant_values in z_variants.items():
            spec = Z_VARIANT_SPECS[variant_name]
            error_variant = _state_delta(variant_values, x_gt)
            axis.plot(
                t_rel,
                error_variant[:, idx],
                color=spec["color"],
                linewidth=spec["linewidth"],
                linestyle=spec["linestyle"],
                label=f"{spec['label']} - x_gt",
            )
        axis.axhline(0.0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
        axis.set_ylabel(f"{STATE_LABELS[idx]}")
        axis.grid(True, alpha=0.3)
        if idx in (0, 2):
            _add_invalid_gps_shading(axis, t_rel, gps_valid)
        if idx == 0:
            axis.legend(loc="upper right")

    axes[0].set_title("Measurement Error: Raw and All Rebuild Types")
    axes[-1].set_xlabel("Time (s)")


def _plot_rebuild_delta(fig, t_rel, z_variants, gps_valid):
    z_saved = z_variants["raw_saved"]
    dim = min(z_saved.shape[1], len(STATE_LABELS))
    axes = np.atleast_1d(fig.subplots(dim, 1, sharex=True))

    for idx, axis in enumerate(axes):
        for variant_name, variant_values in z_variants.items():
            if variant_name == "raw_saved":
                continue
            spec = Z_VARIANT_SPECS[variant_name]
            delta = _state_delta(variant_values, z_saved)
            axis.plot(
                t_rel,
                delta[:, idx],
                color=spec["color"],
                linewidth=spec["linewidth"],
                linestyle=spec["linestyle"],
                label=f"{spec['label']} - raw",
            )
        axis.axhline(0.0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
        axis.set_ylabel(f"{STATE_LABELS[idx]}")
        axis.grid(True, alpha=0.3)
        if idx in (0, 2):
            _add_invalid_gps_shading(axis, t_rel, gps_valid)
        if idx == 0:
            axis.legend(loc="upper right")

    axes[0].set_title("Rebuild Delta Against Raw z")
    axes[-1].set_xlabel("Time (s)")


def _plot_trajectory(fig, x_gt, z_variants, gps_valid):
    ax = fig.subplots(1, 1)
    ax.plot(x_gt[:, 0], x_gt[:, 1], label="x_gt trajectory", linestyle="--", color="tab:red", linewidth=1.3)
    for variant_name, variant_values in z_variants.items():
        spec = Z_VARIANT_SPECS[variant_name]
        ax.plot(
            variant_values[:, 0],
            variant_values[:, 1],
            label=f"{spec['label']} trajectory",
            color=spec["color"],
            linewidth=spec["linewidth"],
            linestyle=spec["linestyle"],
            alpha=0.9,
        )

    valid_mask = gps_valid >= 0.5
    invalid_mask = ~valid_mask
    z_saved = z_variants["raw_saved"]
    if np.any(valid_mask):
        ax.scatter(
            z_saved[valid_mask, 0],
            z_saved[valid_mask, 1],
            s=6,
            color="tab:green",
            alpha=0.35,
            label="GPS valid (saved z)",
        )
    if np.any(invalid_mask):
        ax.scatter(
            z_saved[invalid_mask, 0],
            z_saved[invalid_mask, 1],
            s=6,
            color="tab:orange",
            alpha=0.35,
            label="GPS invalid / DR (saved z)",
        )

    ax.set_title("Vehicle Trajectory: Raw and All Rebuild Types")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")


def _print_variant_delta_stats(name, delta):
    changed_mask = np.any(np.abs(delta) > 1.0e-6, axis=1)
    metrics = _state_metric_summary(delta)
    print(f"{name:<18}: {int(np.count_nonzero(changed_mask))}/{delta.shape[0]} samples changed")
    print(
        "  max |delta|      : "
        f"{_format_metric_vector(metrics['max_abs'])}"
    )
    print(
        "  RMS delta        : "
        f"{_format_metric_vector(metrics['rms'])}"
    )
    print(
        "  mean |delta|     : "
        f"{_format_metric_vector(metrics['mean_abs'])}"
    )


def _print_variant_error_stats(name, estimate, x_gt):
    error = _state_delta(estimate, x_gt)
    metrics = _state_metric_summary(error)
    print(f"{name:<18}: error vs x_gt")
    print(
        "  max |error|      : "
        f"{_format_metric_vector(metrics['max_abs'])}"
    )
    print(
        "  RMS error        : "
        f"{_format_metric_vector(metrics['rms'])}"
    )
    print(
        "  mean |error|     : "
        f"{_format_metric_vector(metrics['mean_abs'])}"
    )


def visualize_npz(file_path, heading_kinematic_config=None, heading_rebuild_modes=None):
    resolved_path = _resolve_dataset_path(file_path)
    print(f"Loading dataset from: {resolved_path}\n")

    try:
        data = np.load(resolved_path, allow_pickle=False)
    except Exception as exc:
        print(f"Failed to load file: {exc}")
        return

    print("--- Available Arrays (Keys) ---")
    print(", ".join(data.files))
    print()

    metadata = _decode_metadata(data)
    if metadata:
        print("--- Metadata ---")
        print(json.dumps(metadata, indent=2))
        print()

    timestamps = _load_array(data, "timestamps", [])
    if timestamps.size == 0:
        print("No timestamp data found to plot.")
        return

    sample_count = timestamps.shape[0]
    t_rel = timestamps - timestamps[0]
    dataset_arrays = {key: np.asarray(data[key]) for key in data.files if key != "metadata_json"}

    x_gt = _load_state_matrix(data, "x_gt", sample_count)
    z_saved = _load_state_matrix(data, "z", sample_count)
    selected_rebuild_modes = _normalize_heading_rebuild_modes(heading_rebuild_modes)
    z_variants = {"raw_saved": z_saved}
    for rebuild_mode in selected_rebuild_modes:
        z_variants[_rebuild_variant_key(rebuild_mode)] = _rebuild_measurements(
            dataset_arrays,
            timestamps,
            heading_rebuild_mode=rebuild_mode,
            heading_kinematic_config=heading_kinematic_config,
        ).reshape(sample_count, -1)[:, : len(STATE_LABELS)]
    steering = _load_array(data, "steering", np.zeros(sample_count, dtype=np.float32))
    throttle = _load_array(data, "throttle", np.zeros(sample_count, dtype=np.float32))
    gps_valid = _load_array(data, "gps_valid", np.zeros(sample_count, dtype=np.float32))
    gps_x = _load_array(data, "gps_x", np.zeros(sample_count, dtype=np.float32))
    gps_y = _load_array(data, "gps_y", np.zeros(sample_count, dtype=np.float32))
    gps_theta = _load_array(data, "gps_theta", np.zeros(sample_count, dtype=np.float32))
    accel_x = _load_array(data, "accel_x", np.zeros(sample_count, dtype=np.float32))
    accel_y = _load_array(data, "accel_y", np.zeros(sample_count, dtype=np.float32))
    accel_z = _load_array(data, "accel_z", np.zeros(sample_count, dtype=np.float32))
    gyro_z = _load_array(data, "gyro_z", np.zeros(sample_count, dtype=np.float32))
    motor_tach = _load_array(data, "motor_tach", np.zeros(sample_count, dtype=np.float32))
    vfl = _load_array(data, "vfl", np.zeros(sample_count, dtype=np.float32))
    vfr = _load_array(data, "vfr", np.zeros(sample_count, dtype=np.float32))
    vrl = _load_array(data, "vrl", np.zeros(sample_count, dtype=np.float32))
    vrr = _load_array(data, "vrr", np.zeros(sample_count, dtype=np.float32))

    gps_valid_ratio = float(np.mean(gps_valid >= 0.5))
    print("--- Quick Stats ---")
    print(f"samples      : {sample_count}")
    print(f"duration (s) : {float(t_rel[-1]):.3f}")
    print(f"gps valid    : {gps_valid_ratio * 100.0:.2f}%")
    print(f"rebuild modes: {', '.join(selected_rebuild_modes)}")
    print("delta vs raw:")
    for rebuild_mode in selected_rebuild_modes:
        variant_key = _rebuild_variant_key(rebuild_mode)
        rebuild_delta = _state_delta(z_variants[variant_key], z_saved)
        _print_variant_delta_stats(variant_key, rebuild_delta)
    print("error vs x_gt:")
    _print_variant_error_stats("raw_saved", z_saved, x_gt)
    for rebuild_mode in selected_rebuild_modes:
        variant_key = _rebuild_variant_key(rebuild_mode)
        _print_variant_error_stats(variant_key, z_variants[variant_key], x_gt)
    print()

    fig_traj = plt.figure("Trajectory", figsize=(10, 8), constrained_layout=True)
    _plot_trajectory(fig_traj, x_gt, z_variants, gps_valid)

    fig_states = plt.figure("States", figsize=(12, 12), constrained_layout=True)
    _plot_states(fig_states, t_rel, x_gt, z_variants, gps_valid)

    fig_sensors = plt.figure("Sensors", figsize=(12, 13), constrained_layout=True)
    _plot_sensor_channels(
        fig_sensors,
        t_rel,
        steering,
        throttle,
        gps_valid,
        gps_x,
        gps_y,
        gps_theta,
        accel_x,
        accel_y,
        accel_z,
        gyro_z,
        motor_tach,
        vfl,
        vfr,
        vrl,
        vrr,
    )

    fig_error = plt.figure("Errors", figsize=(12, 12), constrained_layout=True)
    _plot_measurement_error(fig_error, t_rel, x_gt, z_variants, gps_valid)

    fig_rebuild = plt.figure("Rebuild Delta", figsize=(12, 12), constrained_layout=True)
    _plot_rebuild_delta(fig_rebuild, t_rel, z_variants, gps_valid)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Robust KalmanNet recorded datasets.")
    parser.add_argument(
        "dataset",
        nargs="?",
        help="Path to a dataset .npz file or its matching .json metadata file.",
    )
    parser.add_argument(
        "--heading-rebuild-modes",
        "--heading-rebuild-mode",
        nargs="+",
        default=list(SUPPORTED_HEADING_REBUILD_MODES),
        choices=list(SUPPORTED_HEADING_REBUILD_MODES),
        help="One or more rebuilt-theta modes to compare: qcar_ekf, gyro_filter, kinematic.",
    )
    parser.add_argument(
        "--kin-wheelbase",
        type=float,
        default=0.2,
        help="Wheelbase used when --heading-rebuild-mode kinematic.",
    )
    parser.add_argument(
        "--kin-velocity-model",
        default="tachometer",
        choices=["tachometer", "imu_acceleration", "velocity_lag", "velocity_command", "simple_acceleration"],
        help="Velocity model used when --heading-rebuild-mode kinematic.",
    )
    parser.add_argument(
        "--kin-velocity-tau",
        type=float,
        default=0.301,
        help="Velocity time constant for kinematic theta rebuild.",
    )
    parser.add_argument(
        "--kin-velocity-gain",
        type=float,
        default=6.598,
        help="Throttle-to-speed gain for velocity_lag kinematic theta rebuild.",
    )
    parser.add_argument(
        "--kin-max-velocity",
        type=float,
        default=2.0,
        help="Velocity clamp for kinematic theta rebuild.",
    )
    parser.add_argument(
        "--kin-max-acceleration",
        type=float,
        default=2.0,
        help="Acceleration clamp for kinematic theta rebuild.",
    )
    args = parser.parse_args()

    target_dir = Path(__file__).parent / "datasets"
    if args.dataset:
        target_file = args.dataset
    else:
        npz_files = list(target_dir.glob("*.npz"))
        if not npz_files:
            raise SystemExit(f"No .npz files found in {target_dir}")
        target_file = max(npz_files, key=lambda file_path: file_path.stat().st_mtime)

    heading_kinematic_config = {
        "wheelbase": args.kin_wheelbase,
        "velocity_model": args.kin_velocity_model,
        "velocity_tau": args.kin_velocity_tau,
        "velocity_gain": args.kin_velocity_gain,
        "max_velocity": args.kin_max_velocity,
        "max_acceleration": args.kin_max_acceleration,
    }

    visualize_npz(
        target_file,
        heading_kinematic_config=heading_kinematic_config,
        heading_rebuild_modes=args.heading_rebuild_modes,
    )
