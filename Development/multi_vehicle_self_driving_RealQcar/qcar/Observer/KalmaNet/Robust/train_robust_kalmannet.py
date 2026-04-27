import argparse
import copy
import gc
import json
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

try:
    import torch
    from torch.utils.data import DataLoader, Dataset
except ImportError as exc:
    raise SystemExit("torch is required for training Robust KalmanNet") from exc

from robustKLnet import RSNConfig, RobustStateNet, robuststatenet_loss, weighted_state_mse
from robust_kalmannet_dataset import RAW_KEYS, build_training_windows, merge_recorded_datasets
from sensor_attack_augmentation import AttackConfig, SensorAttackAugmenter


DEFAULT_CONFIG = {
    "datasets": [],
    "output": "models/robust_kalmannet.pt",
    "training": {
        "phase_a_epochs": 20,
        "phase_b_epochs": 20,
        "phase_c_epochs": 20,
        "batch_size": 64,
        "measurement_source": "rebuilt",
        "heading_rebuild_mode": "qcar_ekf",
        "gps_dropout_xy_mode": "freeze",
        "sequence_length": 20,
        "sequence_length_phase_c": 30,
        "stride": 1,
        "val_split": 0.2,
        "val_split_mode": "tail",
        "attacked_validation_enabled": False,
        "attacked_validation_weight": 1.0,
        "attacked_validation_seed": 1007,
        "lr": 1e-4,
        "lr_phase2": None,
        "grad_clip": 1.0,
        "num_workers": 2,
        "pin_memory": True,
        "seed": 7,
        "device": "auto",
        "reverse_split": False,
        "aug_warmup_fraction": 0.5,
        "early_stop_min_delta": 0.0,
        "early_stop_phase_a_patience": 0,
        "early_stop_phase_b_patience": 5,
        "early_stop_phase_c_patience": 5,
    },
    "loss": {
        "lambda_mask": 0.1,
        "lambda_meas_mask": 0.1,
        "lambda_pred": 0.2,
        "lambda_upd": 0.8,
        "lambda_gain": 0.01,
        "state_weights": [1.0, 1.0, 2.0, 1.0, 1.0],
    },
    "augmentation": {
        "enabled": True,
        "attack_prob": 0.5,
        "attack_types": ["bias", "scale", "freeze", "noise", "ramp", "zero_out"],
        "max_branches_attacked": 1,
        "gps_attack_prob": 0.3,
        "gps_attack_types": ["noise", "freeze", "jump", "dropout", "reacquisition"],
    },
    "model": {
        "predictor_mode": "kinematic",
        "kin_wheelbase": 0.2,
        "kin_velocity_model": "tachometer",
        "kin_velocity_tau": 0.301,
        "kin_velocity_gain": 6.598,
        "kin_max_velocity": 2.0,
        "kin_max_acceleration": 2.0,
        "pred_hidden": 32,
        "upd_hidden": 32,
        "gain_hidden": 32,
        "pred_mlp_hidden": 32,
        "mask_hidden": 32,
        "gain_tanh_scale": 2.0,
        "update_mask_init_bias": 2.0,
    },
}

WINDOWS_MULTIPROCESS_DATASET_BYTES_LIMIT = 256 * 1024 * 1024


def _deep_update(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge override into base."""
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_update(base[key], value)
        else:
            base[key] = value
    return base


def load_training_config(config_path: Path) -> Dict[str, Any]:
    """Load YAML training config and merge it with defaults."""
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    if not config_path.exists():
        return config

    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise SystemExit(f"Training config must be a YAML mapping: {config_path}")
    return _deep_update(config, loaded)


def create_parser(config: Dict[str, Any], default_config_path: Path) -> argparse.ArgumentParser:
    """Build parser using YAML-derived defaults while preserving CLI overrides."""
    train_cfg = config.get("training", {})
    loss_cfg = config.get("loss", {})
    aug_cfg = config.get("augmentation", {})
    model_cfg = config.get("model", {})

    parser = argparse.ArgumentParser(
        description="Train Robust KalmanNet offline from recorded datasets"
    )
    parser.add_argument(
        "datasets",
        nargs="*",
        default=list(config.get("datasets", [])),
        help="One or more .npz dataset files recorded from the vehicle",
    )
    parser.add_argument(
        "--config",
        default=str(default_config_path),
        help="YAML config path",
    )
    parser.add_argument(
        "--output",
        default=config.get("output", DEFAULT_CONFIG["output"]),
        help="Checkpoint path relative to this script",
    )
    parser.add_argument(
        "--phase-a-epochs",
        type=int,
        default=int(train_cfg.get("phase_a_epochs", 20)),
        help="Epochs for Phase A (pretrain predictor alone)",
    )
    parser.add_argument(
        "--phase-b-epochs",
        type=int,
        default=int(train_cfg.get("phase_b_epochs", 20)),
        help="Epochs for Phase B (freeze predictor, train update loop)",
    )
    parser.add_argument(
        "--phase-c-epochs",
        type=int,
        default=int(train_cfg.get("phase_c_epochs", 20)),
        help="Epochs for Phase C (fine-tune full model end-to-end)",
    )
    parser.add_argument("--batch-size", type=int, default=int(train_cfg.get("batch_size", 64)))
    parser.add_argument(
        "--measurement-source",
        default=str(train_cfg.get("measurement_source", "rebuilt")),
        choices=["rebuilt", "raw"],
        help="Training z source: rebuilt=offline reconstruction, raw=recorded dataset z as saved",
    )
    parser.add_argument(
        "--heading-rebuild-mode",
        default=str(train_cfg.get("heading_rebuild_mode", "qcar_ekf")),
        choices=["qcar_ekf", "gyro_filter", "kinematic"],
        help="When measurement-source=rebuilt, theta source during GPS loss: qcar_ekf (exact local replica), gyro_filter, or kinematic",
    )
    parser.add_argument(
        "--gps-dropout-xy-mode",
        default=str(train_cfg.get("gps_dropout_xy_mode", "freeze")),
        choices=["freeze", "dead_reckon"],
        help="When measurement-source=rebuilt, how z[x,y] behaves while GPS position is invalid",
    )
    parser.add_argument(
        "--sequence-length", type=int, default=int(train_cfg.get("sequence_length", 20))
    )
    parser.add_argument(
        "--stride", type=int, default=int(train_cfg.get("stride", 1))
    )
    parser.add_argument(
        "--val-split", type=float, default=float(train_cfg.get("val_split", 0.2))
    )
    parser.add_argument(
        "--val-split-mode",
        default=str(
            train_cfg.get(
                "val_split_mode",
                "head" if bool(train_cfg.get("reverse_split", False)) else "tail",
            )
        ),
        choices=["head", "tail", "middle"],
        help="Validation window placement within the chronological dataset",
    )
    parser.add_argument("--lr", type=float, default=float(train_cfg.get("lr", 1e-4)))
    parser.add_argument(
        "--lr-phase2",
        type=float,
        default=train_cfg.get("lr_phase2"),
        help="Learning rate for Phase C (default: lr / 5)",
    )
    parser.add_argument(
        "--device",
        default=str(train_cfg.get("device", "auto")),
        choices=["auto", "cpu", "cuda"],
    )
    parser.add_argument("--seed", type=int, default=int(train_cfg.get("seed", 7)))
    parser.add_argument(
        "--attack-prob",
        type=float,
        default=float(aug_cfg.get("attack_prob", 0.5)),
        help="Probability of attacking a sample",
    )
    parser.add_argument(
        "--attack-types",
        nargs="*",
        default=list(aug_cfg.get("attack_types", DEFAULT_CONFIG["augmentation"]["attack_types"])),
        help="Attack types to use during augmentation",
    )
    parser.add_argument(
        "--gps-attack-prob",
        type=float,
        default=float(aug_cfg.get("gps_attack_prob", DEFAULT_CONFIG["augmentation"]["gps_attack_prob"])),
        help="Independent probability of corrupting GPS x/y measurements",
    )
    parser.add_argument(
        "--gps-attack-types",
        nargs="*",
        default=list(aug_cfg.get("gps_attack_types", DEFAULT_CONFIG["augmentation"]["gps_attack_types"])),
        help="GPS x/y attack types: noise freeze jump dropout reacquisition",
    )
    parser.add_argument(
        "--no-augmentation",
        action="store_true",
        default=not bool(aug_cfg.get("enabled", True)),
        help="Disable attack augmentation (train on clean only)",
    )
    parser.add_argument(
        "--lambda-mask",
        type=float,
        default=float(loss_cfg.get("lambda_mask", 0.1)),
        help="Weight for prediction mask supervision loss",
    )
    parser.add_argument(
        "--lambda-meas-mask",
        type=float,
        default=float(loss_cfg.get("lambda_meas_mask", 0.1)),
        help="Weight for measurement mask supervision loss",
    )
    parser.add_argument(
        "--lambda-pred",
        type=float,
        default=float(loss_cfg.get("lambda_pred", 0.2)),
        help="Phase C prediction loss weight",
    )
    parser.add_argument(
        "--lambda-upd",
        type=float,
        default=float(loss_cfg.get("lambda_upd", 0.8)),
        help="Phase C update loss weight",
    )
    parser.add_argument(
        "--state-weights",
        type=float,
        nargs=5,
        default=list(loss_cfg.get("state_weights", DEFAULT_CONFIG["loss"]["state_weights"])),
        metavar=("WX", "WY", "WTH", "WV", "WA"),
        help="Per-state loss weights [x, y, theta, v, a]",
    )
    parser.add_argument(
        "--lambda-gain",
        type=float,
        default=float(loss_cfg.get("lambda_gain", 0.01)),
        help="Weight for Kalman gain magnitude regularization",
    )
    parser.add_argument(
        "--max-branches-attacked",
        type=int,
        default=int(aug_cfg.get("max_branches_attacked", 1)),
        help="Max branches attacked simultaneously",
    )
    parser.add_argument(
        "--reverse-split",
        action="store_true",
        default=bool(train_cfg.get("reverse_split", False)),
        help="Legacy alias for --val-split-mode=head",
    )
    parser.add_argument(
        "--attacked-validation-enabled",
        action="store_true",
        default=bool(train_cfg.get("attacked_validation_enabled", False)),
        help="Run a second validation pass with synthetic attacks applied",
    )
    parser.add_argument(
        "--no-attacked-validation",
        dest="attacked_validation_enabled",
        action="store_false",
        help="Disable attacked validation pass",
    )
    parser.add_argument(
        "--attacked-validation-weight",
        type=float,
        default=float(train_cfg.get("attacked_validation_weight", 1.0)),
        help="Weight of attacked validation loss in checkpoint selection/early stopping",
    )
    parser.add_argument(
        "--attacked-validation-seed",
        type=int,
        default=int(train_cfg.get("attacked_validation_seed", 1007)),
        help="Fixed RNG seed used to make attacked validation repeatable across epochs",
    )
    parser.add_argument(
        "--aug-warmup-fraction",
        type=float,
        default=float(train_cfg.get("aug_warmup_fraction", 0.5)),
        help="Fraction of Phase B epochs that train on clean data before augmentation ramps in (0.0=no warmup, 1.0=entire phase is clean)",
    )
    parser.add_argument(
        "--predictor-mode",
        default=str(model_cfg.get("predictor_mode", "kinematic")),
        choices=["nn", "kinematic"],
        help="Prediction step: 'nn' (tri-LSTM, learnable, default) | 'kinematic' (QCar bicycle model, no trainable parameters)",
    )
    parser.add_argument(
        "--kin-wheelbase",
        type=float,
        default=float(model_cfg.get("kin_wheelbase", 0.2)),
        help="Wheelbase L used by the analytical kinematic predictor to match QCarEKF",
    )
    parser.add_argument(
        "--kin-velocity-model",
        default=str(model_cfg.get("kin_velocity_model", "tachometer")),
        choices=["tachometer", "imu_acceleration", "velocity_lag", "velocity_command", "simple_acceleration"],
        help="Longitudinal model for kinematic predictor velocity; non-tachometer modes create velocity innovation",
    )
    parser.add_argument(
        "--kin-velocity-tau",
        type=float,
        default=float(model_cfg.get("kin_velocity_tau", 0.301)),
        help="Time constant for velocity_lag/velocity_command kinematic predictor modes",
    )
    parser.add_argument(
        "--kin-velocity-gain",
        type=float,
        default=float(model_cfg.get("kin_velocity_gain", 6.598)),
        help="Throttle-to-speed gain for velocity_lag kinematic predictor mode",
    )
    parser.add_argument(
        "--kin-max-velocity",
        type=float,
        default=float(model_cfg.get("kin_max_velocity", 2.0)),
        help="Velocity clamp for kinematic predictor longitudinal models",
    )
    parser.add_argument(
        "--kin-max-acceleration",
        type=float,
        default=float(model_cfg.get("kin_max_acceleration", 2.0)),
        help="Acceleration clamp for kinematic predictor longitudinal models",
    )
    parser.add_argument(
        "--pred-hidden",
        type=int,
        default=int(model_cfg.get("pred_hidden", 32)),
        help="Hidden size for each predictor branch LSTM",
    )
    parser.add_argument(
        "--upd-hidden",
        type=int,
        default=int(model_cfg.get("upd_hidden", 32)),
        help="Hidden size for the update GRU",
    )
    parser.add_argument(
        "--gain-hidden",
        type=int,
        default=int(model_cfg.get("gain_hidden", 32)),
        help="Hidden size for the Kalman gain MLP",
    )
    parser.add_argument(
        "--pred-mlp-hidden",
        type=int,
        default=int(model_cfg.get("pred_mlp_hidden", 32)),
        help="Hidden size for the motion regressor MLP",
    )
    parser.add_argument(
        "--mask-hidden",
        type=int,
        default=int(model_cfg.get("mask_hidden", 32)),
        help="Hidden size for predictor/update mask MLPs",
    )
    parser.add_argument(
        "--gain-tanh-scale",
        type=float,
        default=float(model_cfg.get("gain_tanh_scale", 2.0)),
        help="Scale applied to tanh-squashed Kalman gains when constrain_gain is enabled",
    )
    parser.add_argument(
        "--update-mask-init-bias",
        type=float,
        default=float(model_cfg.get("update_mask_init_bias", 2.0)),
        help="Initial bias for the updater mask output layer; positive values keep corrections less attenuated at startup",
    )
    parser.add_argument(
        "--grad-clip",
        type=float,
        default=float(train_cfg.get("grad_clip", 1.0)),
        help="Max gradient norm for clipping (0 to disable)",
    )
    parser.add_argument(
        "--num-workers",
        type=int,
        default=int(train_cfg.get("num_workers", 2)),
        help="DataLoader worker count (0 disables background loading)",
    )
    parser.add_argument(
        "--pin-memory",
        dest="pin_memory",
        action="store_true",
        default=bool(train_cfg.get("pin_memory", True)),
        help="Enable pinned host memory for faster CPU-to-GPU transfer",
    )
    parser.add_argument(
        "--no-pin-memory",
        dest="pin_memory",
        action="store_false",
        help="Disable pinned host memory",
    )
    parser.add_argument(
        "--sequence-length-phase-c",
        type=int,
        default=int(train_cfg.get("sequence_length_phase_c", 30)),
        help="Override sequence length for Phase C (default: same as --sequence-length)",
    )
    parser.add_argument(
        "--resume-checkpoint",
        default=None,
        help="Load model state from a saved checkpoint before training",
    )
    parser.add_argument(
        "--resume-mode",
        choices=["continue", "restart"],
        default="continue",
        help="continue=restore optimizer/history and continue from next epoch, restart=load weights only and start a new schedule",
    )
    parser.add_argument(
        "--early-stop-min-delta",
        type=float,
        default=float(train_cfg.get("early_stop_min_delta", 0.0)),
        help="Minimum validation improvement required to reset early-stopping patience",
    )
    parser.add_argument(
        "--early-stop-phase-a-patience",
        type=int,
        default=int(train_cfg.get("early_stop_phase_a_patience", 0)),
        help="Phase A early-stopping patience in epochs (0 disables)",
    )
    parser.add_argument(
        "--early-stop-phase-b-patience",
        type=int,
        default=int(train_cfg.get("early_stop_phase_b_patience", 5)),
        help="Phase B early-stopping patience in epochs (0 disables)",
    )
    parser.add_argument(
        "--early-stop-phase-c-patience",
        type=int,
        default=int(train_cfg.get("early_stop_phase_c_patience", 5)),
        help="Phase C early-stopping patience in epochs (0 disables)",
    )
    return parser


class SlidingWindowDataset(Dataset):
    def __init__(self, raw: Dict[str, np.ndarray], z_seq: np.ndarray, x_gt: np.ndarray, x0: np.ndarray, dt_seq: np.ndarray):
        self.raw = raw
        self.z_seq = z_seq
        self.x_gt = x_gt
        self.x0 = x0
        self.dt_seq = dt_seq

    def __len__(self) -> int:
        return int(self.z_seq.shape[0])

    def __getitem__(self, idx: int):
        raw_item = {key: torch.from_numpy(self.raw[key][idx]) for key in RAW_KEYS}
        return raw_item, torch.from_numpy(self.z_seq[idx]), torch.from_numpy(self.x_gt[idx]), torch.from_numpy(self.x0[idx]), torch.from_numpy(self.dt_seq[idx])


class SubsetWithTransform(Dataset):
    def __init__(self, subset: Dataset):
        self.subset = subset

    def __len__(self):
        return len(self.subset)

    def __getitem__(self, idx):
        return self.subset[idx]


def estimate_dataset_nbytes(dataset: Dataset) -> Optional[int]:
    if isinstance(dataset, SlidingWindowDataset):
        total = sum(array.nbytes for array in dataset.raw.values())
        total += dataset.z_seq.nbytes
        total += dataset.x_gt.nbytes
        total += dataset.x0.nbytes
        total += dataset.dt_seq.nbytes
        return int(total)
    if isinstance(dataset, SubsetWithTransform):
        return estimate_dataset_nbytes(dataset.subset)
    if isinstance(dataset, torch.utils.data.Subset):
        return estimate_dataset_nbytes(dataset.dataset)
    return None


def shutdown_dataloader_workers(loader: Optional[DataLoader]) -> None:
    if loader is None:
        return
    iterator = getattr(loader, "_iterator", None)
    if iterator is not None and hasattr(iterator, "_shutdown_workers"):
        iterator._shutdown_workers()
        loader._iterator = None


def collate_batch(batch):
    raw = {key: torch.stack([item[0][key] for item in batch], dim=0) for key in RAW_KEYS}
    z_seq = torch.stack([item[1] for item in batch], dim=0)
    x_gt = torch.stack([item[2] for item in batch], dim=0)
    x0 = torch.stack([item[3] for item in batch], dim=0)
    dt_seq = torch.stack([item[4] for item in batch], dim=0)
    return raw, z_seq, x_gt, x0, dt_seq


def move_batch_to_device(batch, device: torch.device):
    raw, z_seq, x_gt, x0, dt_seq = batch
    raw = {k: v.to(device=device, dtype=torch.float32) for k, v in raw.items()}
    return raw, z_seq.to(device=device, dtype=torch.float32), x_gt.to(device=device, dtype=torch.float32), x0.to(device=device, dtype=torch.float32), dt_seq.to(device=device, dtype=torch.float32)


def move_optional_tensor_to_device(tensor, device: torch.device):
    if tensor is None:
        return None
    return tensor.to(device=device, dtype=torch.float32)


def make_dataloader(dataset, batch_size: int, shuffle: bool, collate_fn, num_workers: int, pin_memory: bool):
    requested_workers = max(int(num_workers), 0)
    resolved_workers = requested_workers
    dataset_bytes = estimate_dataset_nbytes(dataset)
    if (
        resolved_workers > 0
        and sys.platform.startswith("win")
        and dataset_bytes is not None
        and dataset_bytes >= WINDOWS_MULTIPROCESS_DATASET_BYTES_LIMIT
    ):
        print(
            "  Warning: window dataset is "
            f"{dataset_bytes / (1024.0 ** 2):.1f} MiB on Windows; "
            "forcing num_workers=0 to avoid DataLoader spawn/pickling failures."
        )
        resolved_workers = 0
    kwargs = {
        "batch_size": batch_size,
        "shuffle": shuffle,
        "collate_fn": collate_fn,
        "num_workers": resolved_workers,
        "pin_memory": bool(pin_memory),
    }
    if kwargs["num_workers"] > 0:
        kwargs["persistent_workers"] = True
    loader = DataLoader(dataset, **kwargs)
    loader._requested_num_workers = requested_workers
    loader._resolved_num_workers = resolved_workers
    loader._estimated_dataset_bytes = dataset_bytes
    return loader


def summarize_batch_metrics(batch_metrics: List[Dict[str, float]]) -> Dict[str, float]:
    if not batch_metrics:
        return {}
    numeric_keys = sorted(
        {key for item in batch_metrics for key in item.keys() if key != "has_attack"}
    )
    # Split into clean / attacked batches
    clean = [m for m in batch_metrics if not m.get("has_attack", False)]
    attacked = [m for m in batch_metrics if m.get("has_attack", False)]

    summary: Dict[str, float] = {}
    for key in numeric_keys:
        values = [float(item[key]) for item in batch_metrics if key in item]
        if values:
            summary[key] = float(np.median(values))
            summary[f"{key}_mean"] = float(np.mean(values))
        clean_vals = [float(item[key]) for item in clean if key in item]
        if clean_vals:
            summary[f"{key}_clean"] = float(np.median(clean_vals))
        atk_vals = [float(item[key]) for item in attacked if key in item]
        if atk_vals:
            summary[f"{key}_attacked"] = float(np.median(atk_vals))
    summary["n_clean"] = float(len(clean))
    summary["n_attacked"] = float(len(attacked))
    summary["n_samples"] = float(
        sum(int(item.get("n_samples", 0)) for item in batch_metrics)
    )
    summary["n_attacked_samples"] = float(
        sum(int(item.get("n_attacked_samples", 0)) for item in batch_metrics)
    )
    summary["n_clean_samples"] = float(
        sum(int(item.get("n_clean_samples", 0)) for item in batch_metrics)
    )
    return summary


def build_train_val_subsets(
    dataset: Dataset,
    val_split: float,
    split_mode: str,
) -> Tuple[Dataset, Optional[Dataset]]:
    total_size = int(len(dataset))
    val_size = int(total_size * float(val_split))
    train_size = total_size - val_size
    if train_size <= 0:
        raise SystemExit("Dataset too small after validation split")
    if val_size <= 0:
        return dataset, None

    indices = list(range(total_size))
    mode = str(split_mode).strip().lower()
    if mode == "head":
        val_indices = indices[:val_size]
        train_indices = indices[val_size:]
    elif mode == "tail":
        val_indices = indices[train_size:]
        train_indices = indices[:train_size]
    elif mode == "middle":
        start = max((total_size - val_size) // 2, 0)
        end = min(start + val_size, total_size)
        val_indices = indices[start:end]
        train_indices = indices[:start] + indices[end:]
    else:
        raise SystemExit(
            f"Unsupported val_split_mode '{split_mode}'. Expected one of ['head', 'tail', 'middle']."
        )

    train_subset = torch.utils.data.Subset(dataset, train_indices)
    val_subset = torch.utils.data.Subset(dataset, val_indices)
    return train_subset, val_subset


def evaluate(
    model,
    loader,
    device,
    teacher_forcing: bool,
    state_weights: "torch.Tensor",
    lambda_upd: float,
    lambda_pred: float,
    lambda_gain: float = 0.0,
    lambda_mask: float = 0.0,
    lambda_meas_mask: float = 0.0,
    augmenter: Optional[SensorAttackAugmenter] = None,
    eval_seed: Optional[int] = None,
) -> Tuple[float, Dict[str, float]]:
    model.eval()
    batch_metrics = []
    cpu_rng_state = None
    cuda_rng_state = None
    if augmenter is not None and eval_seed is not None:
        cpu_rng_state = torch.get_rng_state()
        if torch.cuda.is_available():
            cuda_rng_state = torch.cuda.get_rng_state_all()
        torch.manual_seed(int(eval_seed))
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(int(eval_seed))
    with torch.no_grad():
        for batch in loader:
            attack_labels = None
            meas_attack_labels = None
            raw, z_seq, x_gt, x0, dt_seq = batch
            if augmenter is not None:
                raw, z_seq, attack_labels, meas_attack_labels = augmenter.augment_batch(raw, z_seq)
            raw, z_seq, x_gt, x0, dt_seq = move_batch_to_device((raw, z_seq, x_gt, x0, dt_seq), device)
            attack_labels = move_optional_tensor_to_device(attack_labels, device)
            meas_attack_labels = move_optional_tensor_to_device(meas_attack_labels, device)
            out = model(
                raw=raw,
                z_seq=z_seq,
                x0=x0,
                teacher_forcing_state=x_gt if teacher_forcing else None,
                dt_seq=dt_seq,
            )
            loss, _ = robuststatenet_loss(
                out["x_pred"],
                out["x_upd"],
                x_gt,
                lambda_upd=lambda_upd,
                lambda_pred=lambda_pred,
                weights=state_weights,
                pred_mask=out.get("pred_mask"),
                attack_labels=attack_labels,
                lambda_mask=lambda_mask,
                meas_mask=out.get("meas_mask"),
                meas_mask_logits=out.get("meas_mask_logits"),
                meas_attack_labels=meas_attack_labels,
                lambda_meas_mask=lambda_meas_mask,
                K=out.get("K"),
                lambda_gain=lambda_gain,
            )
            loss_upd = weighted_state_mse(out["x_upd"], x_gt, state_weights)
            loss_pred = weighted_state_mse(out["x_pred"], x_gt, state_weights)
            batch_size_current = int(z_seq.shape[0])
            sample_attack_mask = torch.zeros(
                batch_size_current, device=device, dtype=torch.bool
            )
            if attack_labels is not None:
                sample_attack_mask |= attack_labels.reshape(attack_labels.shape[0], -1).sum(dim=1) > 0
            if meas_attack_labels is not None:
                sample_attack_mask |= meas_attack_labels.reshape(meas_attack_labels.shape[0], -1).sum(dim=1) > 0
            attacked_sample_count = int(sample_attack_mask.sum().item())
            clean_sample_count = int(batch_size_current - attacked_sample_count)
            batch_metric = {
                "total": float(loss.item()),
                "state": float(lambda_upd * loss_upd.item() + lambda_pred * loss_pred.item()),
                "loss_upd": float(loss_upd.item()),
                "loss_pred": float(loss_pred.item()),
                "has_attack": bool(attacked_sample_count > 0),
                "n_samples": float(batch_size_current),
                "n_attacked_samples": float(attacked_sample_count),
                "n_clean_samples": float(clean_sample_count),
            }
            if out.get("K") is not None and lambda_gain > 0:
                batch_metric["loss_gain"] = float((out["K"] ** 2).mean().item())
            batch_metrics.append(batch_metric)
    if cpu_rng_state is not None:
        torch.set_rng_state(cpu_rng_state)
    if cuda_rng_state is not None and torch.cuda.is_available():
        torch.cuda.set_rng_state_all(cuda_rng_state)
    summary = summarize_batch_metrics(batch_metrics)
    mean_loss = summary.get("total", float("nan"))
    return mean_loss, summary


def save_checkpoint(
    model,
    optimizer,
    scheduler,
    cfg,
    args,
    training_config,
    merged,
    history,
    best_val: float,
    best_val_selection: float,
    output_path: Path,
    checkpoint_type: str,
    monitor_metric: str,
    monitor_value: float,
    epoch: int,
    phase: int,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "scheduler_state_dict": scheduler.state_dict() if scheduler is not None else None,
            "config": vars(cfg),
            "train_args": vars(args),
            "training_config": training_config,
            "history": history,
            "metadata": merged.get("metadata", {}),
            "best_val_loss": best_val,
            "best_val_selection_loss": best_val_selection,
            "checkpoint_type": checkpoint_type,
            "monitor_metric": monitor_metric,
            "monitor_value": monitor_value,
            "epoch": epoch,
            "phase": phase,
        },
        output_path,
    )


def resolve_checkpoint_path(path_str: Optional[str], script_dir: Path) -> Optional[Path]:
    if not path_str:
        return None
    checkpoint_path = Path(path_str)
    if not checkpoint_path.is_absolute():
        checkpoint_path = (script_dir / checkpoint_path).resolve()
    return checkpoint_path


def get_phase_bounds(args) -> Dict[int, Tuple[int, int]]:
    phase_a_end = int(args.phase_a_epochs)
    phase_b_end = phase_a_end + int(args.phase_b_epochs)
    phase_c_end = phase_b_end + int(args.phase_c_epochs)
    return {
        1: (1, phase_a_end),
        2: (phase_a_end + 1, phase_b_end),
        3: (phase_b_end + 1, phase_c_end),
    }


def get_phase_for_epoch(epoch: int, args) -> Tuple[int, str]:
    if epoch <= int(args.phase_a_epochs):
        return 1, "Phase A"
    if epoch <= int(args.phase_a_epochs) + int(args.phase_b_epochs):
        return 2, "Phase B"
    return 3, "Phase C"


def build_phase_scheduler(optimizer, args, phase: int):
    if phase == 2:
        return torch.optim.lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=max(int(args.phase_b_epochs), 1), eta_min=1e-6
        )
    if phase == 3:
        return torch.optim.lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=max(int(args.phase_c_epochs), 1), eta_min=1e-6
        )
    return None


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument(
        "--config",
        default=str(script_dir / "robust_kalmannet_train_config.yaml"),
    )
    bootstrap_args, _ = bootstrap.parse_known_args()

    config_path = Path(bootstrap_args.config)
    if not config_path.is_absolute():
        config_path = (script_dir / config_path).resolve()

    config = load_training_config(config_path)
    parser = create_parser(config, config_path)
    args = parser.parse_args()

    args.config = str(config_path)
    if args.reverse_split and args.val_split_mode == "tail":
        args.val_split_mode = "head"
    if not args.datasets:
        raise SystemExit(
            "No datasets provided. Set `datasets:` in the YAML config or pass dataset paths on the command line."
        )

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else args.device if args.device != "auto" else "cpu")
    pin_memory = bool(args.pin_memory and device.type == "cuda")
    merged = merge_recorded_datasets(args.datasets)
    heading_kinematic_config = {
        "wheelbase": args.kin_wheelbase,
        "velocity_model": args.kin_velocity_model,
        "velocity_tau": args.kin_velocity_tau,
        "velocity_gain": args.kin_velocity_gain,
        "max_velocity": args.kin_max_velocity,
        "max_acceleration": args.kin_max_acceleration,
    }
    raw, z_seq, x_gt, x0, dt_seq = build_training_windows(
        merged,
        sequence_length=args.sequence_length,
        stride=args.stride,
        measurement_source=args.measurement_source,
        heading_rebuild_mode=args.heading_rebuild_mode,
        heading_kinematic_config=heading_kinematic_config,
        gps_dropout_xy_mode=args.gps_dropout_xy_mode,
    )
    dataset = SlidingWindowDataset(raw, z_seq, x_gt, x0, dt_seq)
    train_subset, val_subset = build_train_val_subsets(
        dataset,
        val_split=args.val_split,
        split_mode=args.val_split_mode,
    )

    if val_subset is not None:
        val_loader = make_dataloader(
            SubsetWithTransform(val_subset),
            batch_size=args.batch_size,
            shuffle=False,
            collate_fn=collate_batch,
            num_workers=args.num_workers,
            pin_memory=pin_memory,
        )
    else:
        val_loader = None

    train_loader = make_dataloader(
        SubsetWithTransform(train_subset),
        batch_size=args.batch_size,
        shuffle=True,
        collate_fn=collate_batch,
        num_workers=args.num_workers,
        pin_memory=pin_memory,
    )

    cfg = RSNConfig(
        dt=float(merged.get("metadata", {}).get("dt_mean", 0.02) or 0.02),
        predictor_mode=args.predictor_mode,
        kin_wheelbase=args.kin_wheelbase,
        kin_velocity_model=args.kin_velocity_model,
        kin_velocity_tau=args.kin_velocity_tau,
        kin_velocity_gain=args.kin_velocity_gain,
        kin_max_velocity=args.kin_max_velocity,
        kin_max_acceleration=args.kin_max_acceleration,
        pred_hidden=args.pred_hidden,
        upd_hidden=args.upd_hidden,
        gain_hidden=args.gain_hidden,
        pred_mlp_hidden=args.pred_mlp_hidden,
        mask_hidden=args.mask_hidden,
        gain_tanh_scale=args.gain_tanh_scale,
        update_mask_init_bias=args.update_mask_init_bias,
    )
    model = RobustStateNet(cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    scheduler = None  # created at each phase start
    state_weights = torch.tensor(args.state_weights, device=device, dtype=torch.float32)
    phase_patience = {
        1: max(int(args.early_stop_phase_a_patience), 0),
        2: max(int(args.early_stop_phase_b_patience), 0),
        3: max(int(args.early_stop_phase_c_patience), 0),
    }
    phase_bad_epochs = {1: 0, 2: 0, 3: 0}
    phase_best_val = {1: float("inf"), 2: float("inf"), 3: float("inf")}
    phase_best_epoch: Dict[int, Optional[int]] = {1: None, 2: None, 3: None}
    phase_best_model_state: Dict[int, Dict[str, Any]] = {}
    phase_best_optimizer_state: Dict[int, Dict[str, Any]] = {}
    phase_best_scheduler_state: Dict[int, Optional[Dict[str, Any]]] = {}
    start_epoch = 1

    print(f"Resolved device : {device}")
    if device.type == "cuda":
        print(f"CUDA available  : {torch.cuda.is_available()}")
        print(f"CUDA device     : {torch.cuda.get_device_name(device.index or 0)}")
    else:
        requested_cuda = args.device == "cuda" or (args.device == "auto" and not torch.cuda.is_available())
        if requested_cuda:
            print("CUDA unavailable in this interpreter; training will run on CPU.")
    resolved_train_workers = getattr(train_loader, "_resolved_num_workers", args.num_workers)
    print(
        f"DataLoader      : num_workers={resolved_train_workers}"
        f" (requested={args.num_workers}), pin_memory={pin_memory}"
    )
    print(f"Measurement z   : {args.measurement_source}")
    if args.measurement_source == "rebuilt":
        print(f"Heading rebuild : {args.heading_rebuild_mode}")
        print(f"GPS dropout x/y : {args.gps_dropout_xy_mode}")
    print(f"Dataset windows : {len(dataset)}, batch_size={args.batch_size}, batches/epoch={len(train_loader)}")
    print(f"Val split mode  : {args.val_split_mode}")

    print(f"\nPredictor mode : {args.predictor_mode}")
    if args.predictor_mode == "kinematic":
        print("  → Kinematic predictor: QCar bicycle model (no learnable parameters in prediction step).")
        print("  → Phase A (Pretrain Predictor) is automatically skipped.")
        print("  → Prediction mask supervision (lambda_mask) is automatically skipped (pred_mask=None).")
        args.phase_a_epochs = 0

    total_epochs = args.phase_a_epochs + args.phase_b_epochs + args.phase_c_epochs
    phase_bounds = get_phase_bounds(args)

    # Initialize attack augmenter
    augmenter = None
    if not args.no_augmentation:
        attack_cfg = AttackConfig.from_dict(config.get("augmentation", {}))
        attack_cfg.attack_prob = args.attack_prob
        attack_cfg.enabled_attacks = args.attack_types
        attack_cfg.max_branches_attacked = args.max_branches_attacked
        attack_cfg.gps_attack_prob = args.gps_attack_prob
        attack_cfg.gps_attack_types = args.gps_attack_types
        augmenter = SensorAttackAugmenter(attack_cfg)
        print(f"Attack augmentation enabled: prob={args.attack_prob}, types={args.attack_types}")
        print(f"GPS x/y augmentation enabled: prob={args.gps_attack_prob}, types={args.gps_attack_types}")
    else:
        print("Attack augmentation DISABLED (training on clean data only)")

    validation_attack_augmenter = None
    if args.attacked_validation_enabled:
        val_attack_cfg = AttackConfig.from_dict(config.get("augmentation", {}))
        val_attack_cfg.attack_prob = args.attack_prob
        val_attack_cfg.enabled_attacks = list(args.attack_types)
        val_attack_cfg.gps_attack_prob = args.gps_attack_prob
        val_attack_cfg.gps_attack_types = list(args.gps_attack_types)
        val_attack_cfg.max_branches_attacked = args.max_branches_attacked
        validation_attack_augmenter = SensorAttackAugmenter(val_attack_cfg)
        print(
            "Attacked validation enabled: "
            f"p_branch={val_attack_cfg.attack_prob}, p_gps={val_attack_cfg.gps_attack_prob}, "
            f"seed={args.attacked_validation_seed}"
        )

    best_val = float("inf")
    best_val_selection = float("inf")
    history = []
    output_path = script_dir / args.output
    robust_output_path = output_path.with_name(f"{output_path.stem}.best_robust{output_path.suffix}")
    phase_c_final_path = output_path.with_name(f"{output_path.stem}.phase_c_final{output_path.suffix}")
    final_path = output_path.with_name(f"{output_path.stem}.final{output_path.suffix}")
    resume_checkpoint_path = resolve_checkpoint_path(args.resume_checkpoint, script_dir)

    if resume_checkpoint_path is not None:
        if not resume_checkpoint_path.exists():
            raise SystemExit(f"Resume checkpoint not found: {resume_checkpoint_path}")
        checkpoint = torch.load(resume_checkpoint_path, map_location=device)
        try:
            model.load_state_dict(checkpoint["model_state_dict"])
        except Exception as exc:
            raise SystemExit(f"Failed to load checkpoint model state from {resume_checkpoint_path}: {exc}") from exc

        print(f"Loaded checkpoint : {resume_checkpoint_path}")
        if args.resume_mode == "continue":
            optimizer_state = checkpoint.get("optimizer_state_dict")
            if optimizer_state is not None:
                try:
                    optimizer.load_state_dict(optimizer_state)
                except Exception as exc:
                    raise SystemExit(
                        f"Failed to load optimizer state from {resume_checkpoint_path}: {exc}"
                    ) from exc
            history = list(checkpoint.get("history", []))
            best_val = float(checkpoint.get("best_val_loss", float("inf")))
            best_val_selection = float(
                checkpoint.get("best_val_selection_loss", float("inf"))
            )
            resumed_epoch = int(checkpoint.get("epoch", 0))
            resumed_phase = int(checkpoint.get("phase", 0))
            start_epoch = resumed_epoch + 1
            if start_epoch > sum(
                [int(args.phase_a_epochs), int(args.phase_b_epochs), int(args.phase_c_epochs)]
            ):
                raise SystemExit(
                    "Checkpoint is already at or beyond the configured total epochs. "
                    "Increase phase epochs or use --resume-mode restart."
                )
            if resumed_phase in phase_best_val:
                phase_entry = next(
                    (
                        item
                        for item in reversed(history)
                        if int(item.get("epoch", -1)) == resumed_epoch and int(item.get("phase", -1)) == resumed_phase
                    ),
                    None,
                )
                if phase_entry is not None:
                    phase_best_val[resumed_phase] = float(
                        phase_entry.get(
                            "val_selection_loss",
                            phase_entry.get("val_loss", float("inf")),
                        )
                    )
                phase_best_epoch[resumed_phase] = resumed_epoch
                phase_best_model_state[resumed_phase] = copy.deepcopy(model.state_dict())
                phase_best_optimizer_state[resumed_phase] = copy.deepcopy(optimizer.state_dict())
                phase_best_scheduler_state[resumed_phase] = copy.deepcopy(
                    checkpoint.get("scheduler_state_dict")
                )

            next_phase, _ = get_phase_for_epoch(start_epoch, args)
            if resumed_phase == next_phase:
                scheduler = build_phase_scheduler(optimizer, args, resumed_phase)
                scheduler_state = checkpoint.get("scheduler_state_dict")
                if scheduler is not None and scheduler_state is not None:
                    try:
                        scheduler.load_state_dict(scheduler_state)
                    except Exception as exc:
                        print(
                            f"Warning: failed to load scheduler state from {resume_checkpoint_path}: {exc}"
                        )
            print(f"Resume mode     : continue from epoch {start_epoch}")
        else:
            print("Resume mode     : restart schedule from loaded weights")

    print(f"Training config : {config_path}")

    def set_module_trainable(module: torch.nn.Module, trainable: bool):
        if hasattr(module, "parameters"):
            for param in module.parameters():
                param.requires_grad = trainable

    print(f"\n{'='*60}")
    print(f"Three-phase training:")
    print(f"  Phase A (Predictor Only) = {args.phase_a_epochs} epochs")
    print(f"  Phase B (Updater Only)   = {args.phase_b_epochs} epochs")
    print(f"  Phase C (End-to-End)     = {args.phase_c_epochs} epochs")
    print(f"{'='*60}\n")
    print(
        f"Early stopping : A={phase_patience[1]}, B={phase_patience[2]}, "
        f"C={phase_patience[3]}, min_delta={args.early_stop_min_delta:.3g}"
    )
    if augmenter is not None and args.phase_b_epochs > 0:
        warmup_ep = int(args.phase_b_epochs * args.aug_warmup_fraction)
        if warmup_ep > 0:
            phase_b_start_epoch = args.phase_a_epochs + 1
            phase_b_attack_epoch = phase_b_start_epoch + warmup_ep
            print(
                "Augmentation note : "
                f"Phase B warmup keeps epochs {phase_b_start_epoch:03d}-"
                f"{phase_b_attack_epoch - 1:03d} clean-only; "
                f"attacked batches start at epoch {phase_b_attack_epoch:03d}."
            )

    epoch = start_epoch
    last_completed_epoch = start_epoch - 1
    last_completed_phase = 0
    final_epoch_for_checkpoint = total_epochs
    final_phase_for_checkpoint = 3 if args.phase_c_epochs > 0 else (2 if args.phase_b_epochs > 0 else 1)

    while epoch <= total_epochs:
        epoch_start = time.perf_counter()
        # Determine phase
        if epoch <= args.phase_a_epochs:
            phase = 1
            phase_label = "Phase A"
            use_teacher_forcing = True
            lambda_pred = 1.0
            lambda_upd = 0.0
            lam_mask = args.lambda_mask
            lam_meas_mask = 0.0
            set_module_trainable(model.predictor, True)
            set_module_trainable(model.updater, False)

        elif epoch <= args.phase_a_epochs + args.phase_b_epochs:
            phase = 2
            phase_label = "Phase B"
            use_teacher_forcing = True
            lambda_pred = 0.0
            lambda_upd = 1.0
            lam_mask = 0.0
            lam_meas_mask = args.lambda_meas_mask
            set_module_trainable(model.predictor, False)
            set_module_trainable(model.updater, True)

            if epoch == args.phase_a_epochs + 1 or (
                scheduler is None and start_epoch == epoch and start_epoch > 1
            ):
                scheduler = build_phase_scheduler(optimizer, args, phase)
                warmup_ep = int(args.phase_b_epochs * args.aug_warmup_fraction)
                print(f"\n{'='*60}")
                print(f"Phase B started: Predictor frozen, Updater training")
                if augmenter is not None and warmup_ep > 0:
                    print(f"  Curriculum: {warmup_ep} clean epochs, then augmentation ramps to target")
                print(f"{'='*60}\n")

        else:
            phase = 3
            phase_label = "Phase C"
            use_teacher_forcing = False
            lambda_pred = args.lambda_pred
            lambda_upd = args.lambda_upd
            lam_mask = args.lambda_mask
            lam_meas_mask = args.lambda_meas_mask
            set_module_trainable(model.predictor, True)
            set_module_trainable(model.updater, True)

            if epoch == args.phase_a_epochs + args.phase_b_epochs + 1 or (
                scheduler is None and start_epoch == epoch and start_epoch > 1
            ):
                lr_phase_e2e = args.lr_phase2 if args.lr_phase2 is not None else args.lr / 5.0
                for pg in optimizer.param_groups:
                    pg["lr"] = lr_phase_e2e

                # FIX-5: Rebuild data loaders with longer sequences for Phase C
                seq_len_c = args.sequence_length_phase_c or args.sequence_length
                if seq_len_c != args.sequence_length:
                    print(f"  Rebuilding data loaders with sequence_length={seq_len_c} for Phase C")
                    shutdown_dataloader_workers(train_loader)
                    shutdown_dataloader_workers(val_loader)
                    train_loader = None
                    val_loader = None
                    gc.collect()
                    raw_c, z_seq_c, x_gt_c, x0_c, dt_seq_c = build_training_windows(
                        merged,
                        sequence_length=seq_len_c,
                        stride=args.stride,
                        measurement_source=args.measurement_source,
                        heading_rebuild_mode=args.heading_rebuild_mode,
                        heading_kinematic_config=heading_kinematic_config,
                        gps_dropout_xy_mode=args.gps_dropout_xy_mode,
                    )
                    dataset_c = SlidingWindowDataset(raw_c, z_seq_c, x_gt_c, x0_c, dt_seq_c)
                    train_subset_c, val_subset_c = build_train_val_subsets(
                        dataset_c,
                        val_split=args.val_split,
                        split_mode=args.val_split_mode,
                    )
                    train_loader = make_dataloader(
                        SubsetWithTransform(train_subset_c),
                        batch_size=args.batch_size,
                        shuffle=True,
                        collate_fn=collate_batch,
                        num_workers=args.num_workers,
                        pin_memory=pin_memory,
                    )
                    if val_subset_c is not None:
                        val_loader = make_dataloader(
                            SubsetWithTransform(val_subset_c),
                            batch_size=args.batch_size,
                            shuffle=False,
                            collate_fn=collate_batch,
                            num_workers=args.num_workers,
                            pin_memory=pin_memory,
                        )
                    else:
                        val_loader = None
                    resolved_c_workers = getattr(train_loader, "_resolved_num_workers", args.num_workers)
                    print(
                        f"  Phase C DataLoader: num_workers={resolved_c_workers}"
                        f" (requested={args.num_workers})"
                    )

                scheduler = build_phase_scheduler(optimizer, args, phase)
                print(f"\n{'='*60}")
                print(f"Phase C started: End-to-end, TF OFF, lr={lr_phase_e2e:.2e}")
                print(f"{'='*60}\n")

        # ── Curriculum: determine effective augmenter for this epoch ──────
        curr_augmenter = None
        curr_attack_prob = 0.0
        curr_gps_attack_prob = 0.0
        if augmenter is not None:
            if phase == 2:  # Phase B: warmup then ramp
                phase_epoch = epoch - args.phase_a_epochs
                warmup_epochs = int(args.phase_b_epochs * args.aug_warmup_fraction)
                if phase_epoch <= warmup_epochs:
                    curr_augmenter = None  # clean warmup
                else:
                    ramp_total = max(args.phase_b_epochs - warmup_epochs, 1)
                    ramp_progress = min((phase_epoch - warmup_epochs) / ramp_total, 1.0)
                    augmenter.config.attack_prob = args.attack_prob * ramp_progress
                    augmenter.config.gps_attack_prob = args.gps_attack_prob * ramp_progress
                    curr_augmenter = augmenter
                    curr_attack_prob = float(augmenter.config.attack_prob)
                    curr_gps_attack_prob = float(augmenter.config.gps_attack_prob)
            else:
                # Phase A or C: full augmentation
                augmenter.config.attack_prob = args.attack_prob
                augmenter.config.gps_attack_prob = args.gps_attack_prob
                curr_augmenter = augmenter
                curr_attack_prob = float(augmenter.config.attack_prob)
                curr_gps_attack_prob = float(augmenter.config.gps_attack_prob)

        model.train()
        train_batch_metrics = []
        train_start = time.perf_counter()
        for batch in train_loader:
            # Apply attack augmentation (curriculum-aware)
            attack_labels = None
            meas_attack_labels = None
            raw_b, z_seq_b, x_gt_b, x0_b, dt_seq_b = batch
            if curr_augmenter is not None:
                raw_b, z_seq_b, attack_labels, meas_attack_labels = curr_augmenter.augment_batch(raw_b, z_seq_b)
            raw_b, z_seq_b, x_gt_b, x0_b, dt_seq_b = move_batch_to_device(
                (raw_b, z_seq_b, x_gt_b, x0_b, dt_seq_b), device
            )
            attack_labels = move_optional_tensor_to_device(attack_labels, device)
            meas_attack_labels = move_optional_tensor_to_device(meas_attack_labels, device)
            batch_size_current = int(z_seq_b.shape[0])
            sample_attack_mask = torch.zeros(
                batch_size_current, device=device, dtype=torch.bool
            )
            if attack_labels is not None:
                sample_attack_mask |= attack_labels.reshape(attack_labels.shape[0], -1).sum(dim=1) > 0
            if meas_attack_labels is not None:
                sample_attack_mask |= meas_attack_labels.reshape(meas_attack_labels.shape[0], -1).sum(dim=1) > 0
            attacked_sample_count = int(sample_attack_mask.sum().item())
            clean_sample_count = int(batch_size_current - attacked_sample_count)

            optimizer.zero_grad()
            out = model(
                raw=raw_b,
                z_seq=z_seq_b,
                x0=x0_b,
                teacher_forcing_state=x_gt_b if use_teacher_forcing else None,
                dt_seq=dt_seq_b,
            )
            loss, logs = robuststatenet_loss(
                out["x_pred"],
                out["x_upd"],
                x_gt_b,
                lambda_upd=lambda_upd,
                lambda_pred=lambda_pred,
                weights=state_weights,
                pred_mask=out.get("pred_mask"),
                attack_labels=attack_labels,
                lambda_mask=lam_mask,
                meas_mask=out.get("meas_mask"),
                meas_mask_logits=out.get("meas_mask_logits"),
                meas_attack_labels=meas_attack_labels,
                lambda_meas_mask=lam_meas_mask,
                K=out.get("K"),
                lambda_gain=args.lambda_gain,
            )
            # Clamp loss to prevent extreme attack batches from corrupting
            # Adam's momentum/variance estimates
            loss_clamped = loss.clamp(max=100.0)
            loss_clamped.backward()
            # FIX-4: gradient clipping for training stability
            if args.grad_clip > 0:
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=args.grad_clip)
            optimizer.step()
            has_attack = bool(
                (attack_labels is not None and attack_labels.sum().item() > 0)
                or (meas_attack_labels is not None and meas_attack_labels.sum().item() > 0)
            )
            batch_metric = {
                "total": float(loss.item()),  # log unclamped for monitoring
                "state": float(lambda_upd * logs["loss_upd"] + lambda_pred * logs["loss_pred"]),
                "loss_upd": float(logs["loss_upd"]),
                "loss_pred": float(logs["loss_pred"]),
                "has_attack": has_attack,
                "n_samples": float(batch_size_current),
                "n_attacked_samples": float(attacked_sample_count),
                "n_clean_samples": float(clean_sample_count),
            }
            for key in (
                "loss_mask",
                "loss_meas_mask",
                "loss_gain",
                "meas_mask_attacked_mean",
                "meas_mask_clean_mean",
            ):
                if key in logs:
                    batch_metric[key] = float(logs[key])
            train_batch_metrics.append(batch_metric)

        train_seconds = time.perf_counter() - train_start

        train_metrics = summarize_batch_metrics(train_batch_metrics)
        train_loss = train_metrics.get("total", float("nan"))
        train_loss_clean = train_metrics.get("total_clean", float("nan"))
        if scheduler is not None:
            scheduler.step()
        val_loss = float("nan")
        val_metrics: Dict[str, float] = {}
        val_attacked_loss = float("nan")
        val_attacked_metrics: Dict[str, float] = {}
        val_selection_loss = float("nan")
        val_seconds = 0.0
        if val_loader is not None:
            val_start = time.perf_counter()
            val_loss, val_metrics = evaluate(
                model,
                val_loader,
                device,
                teacher_forcing=False,
                state_weights=state_weights,
                lambda_upd=lambda_upd,
                lambda_pred=lambda_pred,
                lambda_gain=args.lambda_gain,
                lambda_mask=lam_mask,
                lambda_meas_mask=lam_meas_mask,
            )
            if validation_attack_augmenter is not None:
                val_attacked_loss, val_attacked_metrics = evaluate(
                    model,
                    val_loader,
                    device,
                    teacher_forcing=False,
                    state_weights=state_weights,
                    lambda_upd=lambda_upd,
                    lambda_pred=lambda_pred,
                    lambda_gain=args.lambda_gain,
                    lambda_mask=lam_mask,
                    lambda_meas_mask=lam_meas_mask,
                    augmenter=validation_attack_augmenter,
                    eval_seed=args.attacked_validation_seed,
                )
            val_seconds = time.perf_counter() - val_start
        else:
            val_loss = train_loss
            val_metrics = dict(train_metrics)
        if np.isfinite(val_loss):
            val_selection_loss = val_loss
        if np.isfinite(val_attacked_loss):
            weight = max(float(args.attacked_validation_weight), 0.0)
            denom = 1.0 + weight if np.isfinite(val_selection_loss) else max(weight, 1.0)
            base = val_selection_loss if np.isfinite(val_selection_loss) else 0.0
            val_selection_loss = (base + weight * val_attacked_loss) / denom

        if device.type == "cuda":
            torch.cuda.synchronize(device)
        epoch_seconds = time.perf_counter() - epoch_start
        steps_per_sec = len(train_loader) / epoch_seconds if epoch_seconds > 0 else float("nan")
        n_clean = int(train_metrics.get("n_clean", 0))
        n_attacked = int(train_metrics.get("n_attacked", 0))
        n_clean_samples = int(train_metrics.get("n_clean_samples", 0))
        n_attacked_samples = int(train_metrics.get("n_attacked_samples", 0))
        history.append(
            {
                "epoch": epoch,
                "phase": phase,
                "train_loss": train_loss,
                "train_loss_clean": train_loss_clean,
                "train_loss_mean": train_metrics.get("total_mean", float("nan")),
                "train_state_loss": train_metrics.get("state", float("nan")),
                "val_loss": val_loss,
                "val_state_loss": val_metrics.get("state", float("nan")),
                "val_attacked_loss": val_attacked_loss,
                "val_attacked_state_loss": val_attacked_metrics.get("state", float("nan")),
                "val_selection_loss": val_selection_loss,
                "train_seconds": train_seconds,
                "val_seconds": val_seconds,
                "epoch_seconds": epoch_seconds,
                "steps_per_sec": steps_per_sec,
                "n_clean": n_clean,
                "n_attacked": n_attacked,
                "n_clean_samples": n_clean_samples,
                "n_attacked_samples": n_attacked_samples,
                "effective_attack_prob": float(curr_attack_prob),
                "effective_gps_attack_prob": float(curr_gps_attack_prob),
            }
        )
        last_completed_epoch = epoch
        last_completed_phase = phase
        clean_str = f", clean={train_loss_clean:.6f}" if not np.isnan(train_loss_clean) else ""
        attacked_val_str = (
            f" | val_atk={val_attacked_loss:.6f} "
            f"(state={val_attacked_metrics.get('state', float('nan')):.6f})"
            if np.isfinite(val_attacked_loss)
            else ""
        )
        print(
            f"[{phase_label}] Epoch {epoch:03d} | "
            f"train={train_loss:.6f}{clean_str} (state={train_metrics.get('state', float('nan')):.6f}) | "
            f"val={val_loss:.6f} (state={val_metrics.get('state', float('nan')):.6f}) | "
            f"val_sel={val_selection_loss:.6f}"
            f"{attacked_val_str} | "
            f"time={epoch_seconds:.2f}s (train={train_seconds:.2f}s, val={val_seconds:.2f}s) | "
            f"{steps_per_sec:.2f} steps/s | "
            f"batches clean/atk={n_clean}/{n_attacked} | "
            f"samples clean/atk={n_clean_samples}/{n_attacked_samples} | "
            f"p_branch={curr_attack_prob:.4f}, p_gps={curr_gps_attack_prob:.4f}"
        )
        if device.type == "cuda":
            mem_alloc_mb = torch.cuda.memory_allocated(device) / (1024 ** 2)
            mem_reserved_mb = torch.cuda.memory_reserved(device) / (1024 ** 2)
            print(
                f"  CUDA memory | allocated={mem_alloc_mb:.1f} MiB | reserved={mem_reserved_mb:.1f} MiB"
            )

        phase_improved = val_selection_loss < phase_best_val[phase]
        phase_improved_enough = val_selection_loss < (
            phase_best_val[phase] - float(args.early_stop_min_delta)
        )
        if phase_improved:
            phase_best_val[phase] = val_selection_loss
            phase_best_epoch[phase] = epoch
            phase_best_model_state[phase] = copy.deepcopy(model.state_dict())
            phase_best_optimizer_state[phase] = copy.deepcopy(optimizer.state_dict())
            phase_best_scheduler_state[phase] = (
                copy.deepcopy(scheduler.state_dict()) if scheduler is not None else None
            )
        if phase_patience[phase] > 0 and np.isfinite(val_selection_loss):
            if phase_improved_enough:
                phase_bad_epochs[phase] = 0
            else:
                phase_bad_epochs[phase] += 1

        if val_loss < best_val:
            best_val = val_loss
            save_checkpoint(
                model=model,
                optimizer=optimizer,
                scheduler=scheduler,
                cfg=cfg,
                args=args,
                training_config=config,
                merged=merged,
                history=history,
                best_val=best_val,
                best_val_selection=best_val_selection,
                output_path=output_path,
                checkpoint_type="best_clean",
                monitor_metric="val_loss",
                monitor_value=val_loss,
                epoch=epoch,
                phase=phase,
            )
            print(f"Saved best clean checkpoint to {output_path}")

        if val_selection_loss < best_val_selection:
            best_val_selection = val_selection_loss
            save_checkpoint(
                model=model,
                optimizer=optimizer,
                scheduler=scheduler,
                cfg=cfg,
                args=args,
                training_config=config,
                merged=merged,
                history=history,
                best_val=best_val,
                best_val_selection=best_val_selection,
                output_path=robust_output_path,
                checkpoint_type="best_robust",
                monitor_metric="val_selection_loss",
                monitor_value=val_selection_loss,
                epoch=epoch,
                phase=phase,
            )
            print(f"Saved best robust checkpoint to {robust_output_path}")

        patience = phase_patience[phase]
        if patience > 0 and phase_bad_epochs[phase] >= patience:
            best_epoch = phase_best_epoch.get(phase)
            best_model_state = phase_best_model_state.get(phase)
            best_optimizer_state = phase_best_optimizer_state.get(phase)
            best_scheduler_state = phase_best_scheduler_state.get(phase)
            if best_model_state is not None:
                model.load_state_dict(best_model_state)
            if best_optimizer_state is not None:
                optimizer.load_state_dict(best_optimizer_state)
            if scheduler is not None:
                if best_scheduler_state is None:
                    scheduler = None
                else:
                    scheduler.load_state_dict(best_scheduler_state)

            print(
                f"Early stopping {phase_label} after {patience} stale epochs; "
                f"restoring best {phase_label} weights from epoch {best_epoch}."
            )

            next_epoch = phase_bounds[phase][1] + 1
            if phase == 3 or next_epoch > total_epochs:
                final_epoch_for_checkpoint = int(best_epoch or last_completed_epoch)
                final_phase_for_checkpoint = phase
                break

            scheduler = None
            epoch = next_epoch
            continue

        epoch += 1

    if last_completed_epoch <= 0:
        raise SystemExit("No training epochs were executed.")

    final_checkpoint_path = phase_c_final_path if args.phase_c_epochs > 0 else final_path
    final_checkpoint_type = "final_phase_c" if args.phase_c_epochs > 0 else "final"
    if final_epoch_for_checkpoint == total_epochs:
        final_epoch_for_checkpoint = last_completed_epoch
        final_phase_for_checkpoint = last_completed_phase
    save_checkpoint(
        model=model,
        optimizer=optimizer,
        scheduler=scheduler,
        cfg=cfg,
        args=args,
        training_config=config,
        merged=merged,
        history=history,
        best_val=best_val,
        best_val_selection=best_val_selection,
        output_path=final_checkpoint_path,
        checkpoint_type=final_checkpoint_type,
        monitor_metric="val_selection_loss",
        monitor_value=best_val_selection,
        epoch=final_epoch_for_checkpoint,
        phase=final_phase_for_checkpoint,
    )
    print(f"Saved final checkpoint to {final_checkpoint_path}")

    summary_path = output_path.with_suffix(".train_history.json")
    summary_path.write_text(json.dumps(history, indent=2), encoding="utf-8")
    print(f"Training history saved to {summary_path}")


if __name__ == "__main__":
    main()
