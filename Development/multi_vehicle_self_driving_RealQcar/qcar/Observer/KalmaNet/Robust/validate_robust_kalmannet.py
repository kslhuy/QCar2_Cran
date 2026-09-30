import argparse
import json
from pathlib import Path
import sys
from typing import Dict, List

import numpy as np

try:
    import torch
except ImportError:
    torch = None

from robustKLnet import RSNConfig, RobustStateNet, wrap_angle_scalar
from robust_kalmannet_dataset import compute_rmse, load_recorded_dataset


def build_gps_dict(dataset: Dict[str, np.ndarray], index: int) -> Dict[str, float] | None:
    if float(dataset["gps_valid"][index]) < 0.5:
        return None
    return {
        "x": float(dataset["gps_x"][index]),
        "y": float(dataset["gps_y"][index]),
        "theta": float(dataset["gps_theta"][index]),
        "valid": True,
    }


def run_kinematic_reference(dataset: Dict[str, np.ndarray], wheelbase: float = 0.2) -> np.ndarray:
    predictions: List[np.ndarray] = []
    timestamps = np.asarray(dataset["timestamps"], dtype=np.float64)
    state = np.zeros(4, dtype=np.float64)
    wheelbase = max(float(wheelbase), 1e-6)

    for i in range(len(timestamps)):
        dt = float(timestamps[i] - timestamps[i - 1]) if i > 0 else float(dataset.get("metadata", {}).get("dt_mean", 0.02) or 0.02)
        dt = max(dt, 1e-3)
        gps_data = build_gps_dict(dataset, i)
        motor_tach = float(dataset["motor_tach"][i])
        steering = float(dataset["steering"][i])
        yaw_rate = motor_tach * np.tan(steering) / wheelbase

        theta_prev = float(state[2])
        theta = wrap_angle_scalar(theta_prev + yaw_rate * dt)
        x = float(state[0]) + motor_tach * np.cos(theta_prev) * dt
        y = float(state[1]) + motor_tach * np.sin(theta_prev) * dt

        if gps_data is not None:
            x = float(gps_data["x"])
            y = float(gps_data["y"])
            theta = wrap_angle_scalar(float(gps_data["theta"]))

        state = np.array([x, y, theta, motor_tach], dtype=np.float64)
        predictions.append(state.copy())

    return np.asarray(predictions, dtype=np.float32)


def run_model(dataset: Dict[str, np.ndarray], checkpoint_path: Path, sequence_length: int, device_name: str) -> np.ndarray:
    """One causal rollout; no overlapping-window averaging or target resets."""
    if torch is None:
        raise SystemExit("torch is required for learned-model validation")
    qcar_dir = Path(__file__).resolve().parents[3]
    if str(qcar_dir) not in sys.path:
        sys.path.insert(0, str(qcar_dir))
    from robustKLnet import RobustKalmanNetStateEstimator
    torch.set_num_threads(1)
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    cfg_dict = checkpoint.get("config", {}) if isinstance(checkpoint, dict) else {}
    cfg_dict = dict(cfg_dict, model_path=str(checkpoint_path), device=device_name,
                    sequence_length=1, streaming_inference=True,
                    enable_ekf_comparator=False, enable_clean_reference_comparator=False,
                    publish_clean_reference_output=False, comparator_record_to_file=False)
    # Only the first available measurement initializes the filter, never x_gt.
    initial = np.asarray(dataset["z"][0], dtype=float)[:3]
    estimator = RobustKalmanNetStateEstimator(initial, cfg_dict)
    predictions = []
    timestamps = np.asarray(dataset["timestamps"])
    def value(key, i, default=0.):
        return float(dataset[key][i]) if key in dataset else default
    for i in range(len(timestamps)):
        dt = float(timestamps[i]-timestamps[i-1]) if i else float(cfg_dict.get("dt", .02))
        if dt <= 0 or not np.isfinite(dt):
            raise ValueError("Recorded timestamps must increase within a drive")
        ok = estimator.update(motor_tach=value("motor_tach", i),
                              steering=value("steering", i), throttle=value("throttle", i),
                              dt=dt, gyro_z=value("gyro_z", i),
                              gps_data=build_gps_dict(dataset, i),
                              acceleration=np.array([value("ax", i), value("ay", i), 0.]))
        if not ok:
            raise RuntimeError(f"Estimator failed at sample {i}")
        predictions.append(estimator.get_state()[:4])
    return np.asarray(predictions)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare kinematic reference vs learned Robust KalmanNet offline")
    parser.add_argument("datasets", nargs="+", help="Recorded dataset .npz files")
    parser.add_argument("--checkpoint", help="Checkpoint path for learned model")
    parser.add_argument("--sequence-length", type=int, default=20)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--kin-wheelbase", type=float, default=0.2, help="Wheelbase L used by the QCar bicycle reference model")
    parser.add_argument("--output", default="validation_metrics.json", help="Output metrics filename relative to this script")
    args = parser.parse_args()

    datasets = [load_recorded_dataset(path) for path in args.datasets]
    target = np.concatenate([np.asarray(dataset["x_gt"], dtype=np.float32)[:, :4] for dataset in datasets])
    kinematic_pred = np.concatenate([run_kinematic_reference(dataset, wheelbase=args.kin_wheelbase) for dataset in datasets])
    metrics = {
        "dataset_files": args.datasets,
        "protocol": "causal_streaming_reset_per_file",
        "target_source": [dataset.get("metadata", {}).get("target_estimator_type", "unknown") for dataset in datasets],
        "target_note": "Recorded EKF targets measure imitation error, not physical ground-truth accuracy.",
        "kinematic_reference": compute_rmse(kinematic_pred, target),
    }

    if args.checkpoint:
        checkpoint_path = Path(args.checkpoint)
        if not checkpoint_path.is_absolute():
            checkpoint_path = Path(__file__).resolve().parent / checkpoint_path
        model_pred = np.concatenate([run_model(dataset, checkpoint_path, args.sequence_length, args.device) for dataset in datasets])
        metrics["learned"] = compute_rmse(model_pred, target)
    else:
        model_pred = None

    output_path = Path(__file__).resolve().parent / args.output
    output_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    print(f"Saved metrics to {output_path}")

    predictions_path = output_path.with_suffix(".predictions.npz")
    save_payload = {
        "target": target,
        "kinematic_reference_pred": kinematic_pred,
    }
    if model_pred is not None:
        save_payload["learned_pred"] = model_pred
    np.savez_compressed(predictions_path, **save_payload)
    print(f"Saved predictions to {predictions_path}")


if __name__ == "__main__":
    if "--simulation" in sys.argv:
        from simulation_benchmark import main as simulation_main
        sys.argv.remove("--simulation")
        simulation_main()
    else:
        main()
