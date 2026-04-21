import argparse
import json
from pathlib import Path
import time
from typing import Any, Dict, List, Tuple

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
        "sequence_length": 20,
        "sequence_length_phase_c": 30,
        "stride": 1,
        "val_split": 0.2,
        "lr": 1e-4,
        "lr_phase2": None,
        "grad_clip": 1.0,
        "num_workers": 2,
        "pin_memory": True,
        "seed": 7,
        "device": "auto",
        "reverse_split": False,
    },
    "loss": {
        "lambda_mask": 0.1,
        "lambda_meas_mask": 0.1,
        "lambda_pred": 0.2,
        "lambda_upd": 0.8,
        "lambda_gain": 0.01,
        "state_weights": [1.0, 1.0, 5.0, 1.0, 1.0],
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
        "predictor_mode": "nn",
        "kin_wheelbase": 0.2,
        "pred_hidden": 32,
        "upd_hidden": 32,
        "gain_hidden": 32,
        "pred_mlp_hidden": 32,
        "mask_hidden": 32,
        "update_mask_init_bias": 2.0,
    },
}


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
        "--sequence-length", type=int, default=int(train_cfg.get("sequence_length", 20))
    )
    parser.add_argument(
        "--stride", type=int, default=int(train_cfg.get("stride", 1))
    )
    parser.add_argument(
        "--val-split", type=float, default=float(train_cfg.get("val_split", 0.2))
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
        help="Use first portion for validation and remaining for training",
    )
    parser.add_argument(
        "--predictor-mode",
        default=str(model_cfg.get("predictor_mode", "nn")),
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
    kwargs = {
        "batch_size": batch_size,
        "shuffle": shuffle,
        "collate_fn": collate_fn,
        "num_workers": max(int(num_workers), 0),
        "pin_memory": bool(pin_memory),
    }
    if kwargs["num_workers"] > 0:
        kwargs["persistent_workers"] = True
    return DataLoader(dataset, **kwargs)


def summarize_batch_metrics(batch_metrics: List[Dict[str, float]]) -> Dict[str, float]:
    if not batch_metrics:
        return {}
    keys = sorted({key for item in batch_metrics for key in item.keys()})
    summary: Dict[str, float] = {}
    for key in keys:
        values = [float(item[key]) for item in batch_metrics if key in item]
        if values:
            summary[key] = float(np.mean(values))
    return summary


def evaluate(
    model,
    loader,
    device,
    teacher_forcing: bool,
    state_weights: "torch.Tensor",
    lambda_upd: float,
    lambda_pred: float,
    lambda_gain: float = 0.0,
) -> Tuple[float, Dict[str, float]]:
    model.eval()
    batch_metrics = []
    with torch.no_grad():
        for batch in loader:
            raw, z_seq, x_gt, x0, dt_seq = move_batch_to_device(batch, device)
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
                K=out.get("K"),
                lambda_gain=lambda_gain,
            )
            loss_upd = weighted_state_mse(out["x_upd"], x_gt, state_weights)
            loss_pred = weighted_state_mse(out["x_pred"], x_gt, state_weights)
            batch_metric = {
                "total": float(loss.item()),
                "state": float(lambda_upd * loss_upd.item() + lambda_pred * loss_pred.item()),
                "loss_upd": float(loss_upd.item()),
                "loss_pred": float(loss_pred.item()),
            }
            if out.get("K") is not None and lambda_gain > 0:
                batch_metric["loss_gain"] = float((out["K"] ** 2).mean().item())
            batch_metrics.append(batch_metric)
    summary = summarize_batch_metrics(batch_metrics)
    mean_loss = summary.get("total", float("nan"))
    return mean_loss, summary


def save_checkpoint(
    model,
    cfg,
    args,
    training_config,
    merged,
    history,
    best_val: float,
    output_path: Path,
    checkpoint_type: str,
    epoch: int,
    phase: int,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "config": vars(cfg),
            "train_args": vars(args),
            "training_config": training_config,
            "history": history,
            "metadata": merged.get("metadata", {}),
            "best_val_loss": best_val,
            "checkpoint_type": checkpoint_type,
            "epoch": epoch,
            "phase": phase,
        },
        output_path,
    )


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
    if not args.datasets:
        raise SystemExit(
            "No datasets provided. Set `datasets:` in the YAML config or pass dataset paths on the command line."
        )

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else args.device if args.device != "auto" else "cpu")
    pin_memory = bool(args.pin_memory and device.type == "cuda")
    merged = merge_recorded_datasets(args.datasets)
    raw, z_seq, x_gt, x0, dt_seq = build_training_windows(merged, sequence_length=args.sequence_length, stride=args.stride)
    dataset = SlidingWindowDataset(raw, z_seq, x_gt, x0, dt_seq)

    val_size = int(len(dataset) * args.val_split)
    train_size = len(dataset) - val_size
    if train_size <= 0:
        raise SystemExit("Dataset too small after validation split")

    if val_size > 0:
        # Prevent data leakage: Use chronological split instead of random split for sliding window time series
        indices = list(range(len(dataset)))
        if args.reverse_split:
            val_subset = torch.utils.data.Subset(dataset, indices[:val_size])
            train_subset = torch.utils.data.Subset(dataset, indices[val_size:])
        else:
            train_subset = torch.utils.data.Subset(dataset, indices[:train_size])
            val_subset = torch.utils.data.Subset(dataset, indices[train_size:])
        val_loader = make_dataloader(
            SubsetWithTransform(val_subset),
            batch_size=args.batch_size,
            shuffle=False,
            collate_fn=collate_batch,
            num_workers=args.num_workers,
            pin_memory=pin_memory,
        )
    else:
        train_subset = dataset
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
        pred_hidden=args.pred_hidden,
        upd_hidden=args.upd_hidden,
        gain_hidden=args.gain_hidden,
        pred_mlp_hidden=args.pred_mlp_hidden,
        mask_hidden=args.mask_hidden,
        update_mask_init_bias=args.update_mask_init_bias,
    )
    model = RobustStateNet(cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    state_weights = torch.tensor(args.state_weights, device=device, dtype=torch.float32)

    print(f"Resolved device : {device}")
    if device.type == "cuda":
        print(f"CUDA available  : {torch.cuda.is_available()}")
        print(f"CUDA device     : {torch.cuda.get_device_name(device.index or 0)}")
    else:
        requested_cuda = args.device == "cuda" or (args.device == "auto" and not torch.cuda.is_available())
        if requested_cuda:
            print("CUDA unavailable in this interpreter; training will run on CPU.")
    print(f"DataLoader      : num_workers={args.num_workers}, pin_memory={pin_memory}")
    print(f"Dataset windows : {len(dataset)}, batch_size={args.batch_size}, batches/epoch={len(train_loader)}")

    print(f"\nPredictor mode : {args.predictor_mode}")
    if args.predictor_mode == "kinematic":
        print("  → Kinematic predictor: QCar bicycle model (no learnable parameters in prediction step).")
        print("  → Phase A (Pretrain Predictor) is automatically skipped.")
        print("  → Prediction mask supervision (lambda_mask) is automatically skipped (pred_mask=None).")
        args.phase_a_epochs = 0

    total_epochs = args.phase_a_epochs + args.phase_b_epochs + args.phase_c_epochs

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

    best_val = float("inf")
    history = []
    output_path = script_dir / args.output
    phase_c_final_path = output_path.with_name(f"{output_path.stem}.phase_c_final{output_path.suffix}")
    final_path = output_path.with_name(f"{output_path.stem}.final{output_path.suffix}")

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

    for epoch in range(1, total_epochs + 1):
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

            if epoch == args.phase_a_epochs + 1:
                print(f"\n{'='*60}")
                print(f"Phase B started: Predictor frozen, Updater training")
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

            if epoch == args.phase_a_epochs + args.phase_b_epochs + 1:
                lr_phase_e2e = args.lr_phase2 if args.lr_phase2 is not None else args.lr / 5.0
                for pg in optimizer.param_groups:
                    pg["lr"] = lr_phase_e2e

                # FIX-5: Rebuild data loaders with longer sequences for Phase C
                seq_len_c = args.sequence_length_phase_c or args.sequence_length
                if seq_len_c != args.sequence_length:
                    print(f"  Rebuilding data loaders with sequence_length={seq_len_c} for Phase C")
                    raw_c, z_seq_c, x_gt_c, x0_c, dt_seq_c = build_training_windows(
                        merged, sequence_length=seq_len_c, stride=args.stride
                    )
                    dataset_c = SlidingWindowDataset(raw_c, z_seq_c, x_gt_c, x0_c, dt_seq_c)
                    val_size_c = int(len(dataset_c) * args.val_split)
                    train_size_c = len(dataset_c) - val_size_c
                    indices_c = list(range(len(dataset_c)))
                    if args.reverse_split:
                        train_subset_c = torch.utils.data.Subset(dataset_c, indices_c[val_size_c:])
                        val_subset_c = torch.utils.data.Subset(dataset_c, indices_c[:val_size_c])
                    else:
                        train_subset_c = torch.utils.data.Subset(dataset_c, indices_c[:train_size_c])
                        val_subset_c = torch.utils.data.Subset(dataset_c, indices_c[train_size_c:])
                    train_loader = make_dataloader(
                        SubsetWithTransform(train_subset_c),
                        batch_size=args.batch_size,
                        shuffle=True,
                        collate_fn=collate_batch,
                        num_workers=args.num_workers,
                        pin_memory=pin_memory,
                    )
                    if val_size_c > 0:
                        val_loader = make_dataloader(
                            SubsetWithTransform(val_subset_c),
                            batch_size=args.batch_size,
                            shuffle=False,
                            collate_fn=collate_batch,
                            num_workers=args.num_workers,
                            pin_memory=pin_memory,
                        )

                print(f"\n{'='*60}")
                print(f"Phase C started: End-to-end, TF OFF, lr={lr_phase_e2e:.2e}")
                print(f"{'='*60}\n")

        model.train()
        train_batch_metrics = []
        train_start = time.perf_counter()
        for batch in train_loader:
            # Apply attack augmentation
            attack_labels = None
            meas_attack_labels = None
            raw_b, z_seq_b, x_gt_b, x0_b, dt_seq_b = batch
            if augmenter is not None:
                raw_b, z_seq_b, attack_labels, meas_attack_labels = augmenter.augment_batch(raw_b, z_seq_b)
            raw_b, z_seq_b, x_gt_b, x0_b, dt_seq_b = move_batch_to_device(
                (raw_b, z_seq_b, x_gt_b, x0_b, dt_seq_b), device
            )
            attack_labels = move_optional_tensor_to_device(attack_labels, device)
            meas_attack_labels = move_optional_tensor_to_device(meas_attack_labels, device)

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
            loss.backward()
            # FIX-4: gradient clipping for training stability
            if args.grad_clip > 0:
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=args.grad_clip)
            optimizer.step()
            batch_metric = {
                "total": float(loss.item()),
                "state": float(lambda_upd * logs["loss_upd"] + lambda_pred * logs["loss_pred"]),
                "loss_upd": float(logs["loss_upd"]),
                "loss_pred": float(logs["loss_pred"]),
            }
            for key in ("loss_mask", "loss_meas_mask", "loss_gain"):
                if key in logs:
                    batch_metric[key] = float(logs[key])
            train_batch_metrics.append(batch_metric)

        train_seconds = time.perf_counter() - train_start

        train_metrics = summarize_batch_metrics(train_batch_metrics)
        train_loss = train_metrics.get("total", float("nan"))
        val_loss = float("nan")
        val_metrics: Dict[str, float] = {}
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
            )
            val_seconds = time.perf_counter() - val_start
        else:
            val_loss = train_loss
            val_metrics = dict(train_metrics)

        if device.type == "cuda":
            torch.cuda.synchronize(device)
        epoch_seconds = time.perf_counter() - epoch_start
        steps_per_sec = len(train_loader) / epoch_seconds if epoch_seconds > 0 else float("nan")
        history.append(
            {
                "epoch": epoch,
                "phase": phase,
                "train_loss": train_loss,
                "train_state_loss": train_metrics.get("state", float("nan")),
                "val_loss": val_loss,
                "val_state_loss": val_metrics.get("state", float("nan")),
                "train_seconds": train_seconds,
                "val_seconds": val_seconds,
                "epoch_seconds": epoch_seconds,
                "steps_per_sec": steps_per_sec,
            }
        )
        print(
            f"[{phase_label}] Epoch {epoch:03d} | "
            f"train={train_loss:.6f} (state={train_metrics.get('state', float('nan')):.6f}) | "
            f"val={val_loss:.6f} (state={val_metrics.get('state', float('nan')):.6f}) | "
            f"time={epoch_seconds:.2f}s (train={train_seconds:.2f}s, val={val_seconds:.2f}s) | "
            f"{steps_per_sec:.2f} steps/s"
        )
        if device.type == "cuda":
            mem_alloc_mb = torch.cuda.memory_allocated(device) / (1024 ** 2)
            mem_reserved_mb = torch.cuda.memory_reserved(device) / (1024 ** 2)
            print(
                f"  CUDA memory | allocated={mem_alloc_mb:.1f} MiB | reserved={mem_reserved_mb:.1f} MiB"
            )

        if val_loss < best_val:
            best_val = val_loss
            save_checkpoint(
                model=model,
                cfg=cfg,
                args=args,
                training_config=config,
                merged=merged,
                history=history,
                best_val=best_val,
                output_path=output_path,
                checkpoint_type="best_overall",
                epoch=epoch,
                phase=phase,
            )
            print(f"Saved best checkpoint to {output_path}")

    final_checkpoint_path = phase_c_final_path if args.phase_c_epochs > 0 else final_path
    final_checkpoint_type = "final_phase_c" if args.phase_c_epochs > 0 else "final"
    final_phase = 3 if args.phase_c_epochs > 0 else (2 if args.phase_b_epochs > 0 else 1)
    save_checkpoint(
        model=model,
        cfg=cfg,
        args=args,
        training_config=config,
        merged=merged,
        history=history,
        best_val=best_val,
        output_path=final_checkpoint_path,
        checkpoint_type=final_checkpoint_type,
        epoch=total_epochs,
        phase=final_phase,
    )
    print(f"Saved final checkpoint to {final_checkpoint_path}")

    summary_path = output_path.with_suffix(".train_history.json")
    summary_path.write_text(json.dumps(history, indent=2), encoding="utf-8")
    print(f"Training history saved to {summary_path}")


if __name__ == "__main__":
    main()
