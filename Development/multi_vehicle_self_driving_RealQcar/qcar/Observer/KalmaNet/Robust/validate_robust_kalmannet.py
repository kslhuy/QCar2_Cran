import argparse
import json
from pathlib import Path
from typing import Dict, List

import numpy as np

try:
    import torch
except ImportError:
    torch = None

from robustKLnet import RSNConfig, RobustStateNet, wrap_angle_scalar
from robust_kalmannet_dataset import RAW_KEYS, build_training_windows, compute_rmse, merge_recorded_datasets


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
    state = np.zeros(5, dtype=np.float64)
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

        state = np.array([x, y, theta, motor_tach, yaw_rate], dtype=np.float64)
        predictions.append(state.copy())

    return np.asarray(predictions, dtype=np.float32)


def run_model(dataset: Dict[str, np.ndarray], checkpoint_path: Path, sequence_length: int, device_name: str) -> np.ndarray:
    if torch is None:
        raise SystemExit("torch is required for learned-model validation")

    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    cfg_dict = checkpoint.get("config", {}) if isinstance(checkpoint, dict) else {}
    cfg = RSNConfig(**{k: v for k, v in cfg_dict.items() if k in RSNConfig.__dataclass_fields__})
    model = RobustStateNet(cfg)
    state_dict = checkpoint.get("model_state_dict", checkpoint)
    model.load_state_dict(state_dict, strict=False)

    device = torch.device("cuda" if device_name == "auto" and torch.cuda.is_available() else device_name if device_name != "auto" else "cpu")
    model.to(device)
    model.eval()

    raw_windows, z_windows, _, x0_windows, dt_windows = build_training_windows(dataset, sequence_length=sequence_length, stride=1)
    raw_tensors = {key: torch.from_numpy(raw_windows[key]).to(device=device, dtype=torch.float32) for key in RAW_KEYS}
    z_tensor = torch.from_numpy(z_windows).to(device=device, dtype=torch.float32)
    x0_tensor = torch.from_numpy(x0_windows).to(device=device, dtype=torch.float32)
    dt_tensor = torch.from_numpy(dt_windows).to(device=device, dtype=torch.float32)

    with torch.no_grad():
        out = model(raw=raw_tensors, z_seq=z_tensor, x0=x0_tensor, teacher_forcing_state=None, dt_seq=dt_tensor)
    predicted_windows = out["x_upd"].detach().cpu().numpy()

    seq_len = predicted_windows.shape[1]
    total_len = dataset["x_gt"].shape[0]
    fused = np.zeros((total_len, predicted_windows.shape[-1]), dtype=np.float32)
    counts = np.zeros(total_len, dtype=np.float32)
    for window_idx in range(predicted_windows.shape[0]):
        for offset in range(seq_len):
            sample_idx = window_idx + offset
            fused[sample_idx] += predicted_windows[window_idx, offset]
            counts[sample_idx] += 1.0
    counts[counts == 0.0] = 1.0
    fused /= counts[:, None]
    return fused


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare kinematic reference vs learned Robust KalmanNet offline")
    parser.add_argument("datasets", nargs="+", help="Recorded dataset .npz files")
    parser.add_argument("--checkpoint", help="Checkpoint path for learned model")
    parser.add_argument("--sequence-length", type=int, default=20)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--kin-wheelbase", type=float, default=0.2, help="Wheelbase L used by the QCar bicycle reference model")
    parser.add_argument("--output", default="validation_metrics.json", help="Output metrics filename relative to this script")
    args = parser.parse_args()

    dataset = merge_recorded_datasets(args.datasets)
    target = np.asarray(dataset["x_gt"], dtype=np.float32)

    kinematic_pred = run_kinematic_reference(dataset, wheelbase=args.kin_wheelbase)
    metrics = {
        "dataset_files": args.datasets,
        "target_source": dataset.get("metadata", {}).get("target_estimator_type", "unknown"),
        "kinematic_reference": compute_rmse(kinematic_pred, target),
    }

    if args.checkpoint:
        checkpoint_path = Path(args.checkpoint)
        if not checkpoint_path.is_absolute():
            checkpoint_path = Path(__file__).resolve().parent / checkpoint_path
        model_pred = run_model(dataset, checkpoint_path, args.sequence_length, args.device)
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
    main()
