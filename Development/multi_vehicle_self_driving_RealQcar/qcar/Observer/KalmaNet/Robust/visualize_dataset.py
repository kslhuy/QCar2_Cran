import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

"""
Examples:
python .\visualize_dataset.py
python .\visualize_dataset.py .\datasets\your_dataset.npz
python .\visualize_dataset.py .\datasets\your_dataset.json
"""


STATE_LABELS = ("x", "y", "theta", "v", "w")
STATE_UNITS = ("m", "m", "rad", "m/s", "rad/s")


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
            ax.axvspan(t_rel[start_idx], t_rel[idx], color="orange", alpha=0.12)
            start_idx = None

    if start_idx is not None:
        ax.axvspan(t_rel[start_idx], t_rel[-1], color="orange", alpha=0.12)


def _plot_states(fig, t_rel, x_gt, z, gps_valid):
    axes = fig.subplots(5, 1, sharex=True)
    for idx, axis in enumerate(axes):
        axis.plot(t_rel, x_gt[:, idx], label="x_gt", color="tab:red", linewidth=1.2)
        axis.plot(t_rel, z[:, idx], label="z", color="tab:blue", linewidth=0.9, alpha=0.85)
        axis.set_ylabel(f"{STATE_LABELS[idx]} ({STATE_UNITS[idx]})")
        axis.grid(True, alpha=0.3)
        _add_invalid_gps_shading(axis, t_rel, gps_valid)
        if idx == 0:
            axis.legend(loc="upper right")
    axes[0].set_title("State Comparison")
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

    axes[3].plot(t_rel, gyro_z, label="gyro_z", color="tab:purple", linewidth=1.0)
    axes[3].plot(t_rel, motor_tach, label="motor_tach", color="tab:brown", linewidth=1.0, alpha=0.9)
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


def _plot_measurement_error(fig, t_rel, x_gt, z, gps_valid):
    axes = fig.subplots(5, 1, sharex=True)
    error = z - x_gt
    error[:, 2] = _wrap_angle_array(error[:, 2])

    for idx, axis in enumerate(axes):
        axis.plot(t_rel, error[:, idx], color="tab:orange", linewidth=1.0)
        axis.axhline(0.0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
        axis.set_ylabel(f"{STATE_LABELS[idx]}")
        axis.grid(True, alpha=0.3)
        _add_invalid_gps_shading(axis, t_rel, gps_valid)

    axes[0].set_title("Measurement Error: z - x_gt")
    axes[-1].set_xlabel("Time (s)")


def _plot_trajectory(fig, x_gt, z, gps_valid):
    ax = fig.subplots(1, 1)
    ax.plot(x_gt[:, 0], x_gt[:, 1], label="x_gt trajectory", linestyle="--", color="tab:red", linewidth=1.3)
    ax.plot(z[:, 0], z[:, 1], label="z trajectory", color="tab:blue", linewidth=1.0, alpha=0.8)

    valid_mask = gps_valid >= 0.5
    invalid_mask = ~valid_mask
    if np.any(valid_mask):
        ax.scatter(z[valid_mask, 0], z[valid_mask, 1], s=6, color="tab:green", alpha=0.35, label="GPS valid")
    if np.any(invalid_mask):
        ax.scatter(z[invalid_mask, 0], z[invalid_mask, 1], s=6, color="tab:orange", alpha=0.35, label="GPS invalid / DR")

    ax.set_title("Vehicle Trajectory")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")


def visualize_npz(file_path):
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

    x_gt = _load_array(data, "x_gt", np.zeros((sample_count, 5), dtype=np.float32)).reshape(sample_count, 5)
    z = _load_array(data, "z", np.zeros((sample_count, 5), dtype=np.float32)).reshape(sample_count, 5)
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
    print()

    fig_traj = plt.figure("Trajectory", figsize=(10, 8), constrained_layout=True)
    _plot_trajectory(fig_traj, x_gt, z, gps_valid)

    fig_states = plt.figure("States", figsize=(12, 12), constrained_layout=True)
    _plot_states(fig_states, t_rel, x_gt, z, gps_valid)

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
    _plot_measurement_error(fig_error, t_rel, x_gt, z, gps_valid)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Robust KalmanNet recorded datasets.")
    parser.add_argument(
        "dataset",
        nargs="?",
        help="Path to a dataset .npz file or its matching .json metadata file.",
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

    visualize_npz(target_file)
