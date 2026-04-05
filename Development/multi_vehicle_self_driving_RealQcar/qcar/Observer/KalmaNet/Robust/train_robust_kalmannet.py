import argparse
import json
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

try:
    import torch
    from torch.utils.data import DataLoader, Dataset, random_split
except ImportError as exc:
    raise SystemExit("torch is required for training Robust KalmanNet") from exc

from robustKLnet import RSNConfig, RobustStateNet, robuststatenet_loss
from robust_kalmannet_dataset import RAW_KEYS, build_training_windows, merge_recorded_datasets
from sensor_attack_augmentation import AttackConfig, SensorAttackAugmenter


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


def evaluate(model, loader, device, teacher_forcing: bool) -> Tuple[float, Dict[str, float]]:
    model.eval()
    losses = []
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
            state_weights = torch.tensor([1.0, 1.0, 5.0, 1.0, 1.0], device=device)
            loss, _ = robuststatenet_loss(
                out["x_pred"], out["x_upd"], x_gt, weights=state_weights,
            )
            losses.append(float(loss.item()))
    mean_loss = float(np.mean(losses)) if losses else float("nan")
    return mean_loss, {"loss": mean_loss}


def save_checkpoint(
    model,
    cfg,
    args,
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
    parser = argparse.ArgumentParser(description="Train Robust KalmanNet offline from recorded datasets")
    parser.add_argument("datasets", nargs="+", help="One or more .npz dataset files recorded from the vehicle")
    parser.add_argument("--output", default="models/robust_kalmannet.pt", help="Checkpoint path relative to this script")
    parser.add_argument("--phase-a-epochs", type=int, default=20, help="Epochs for Phase A (pretrain predictor alone)")
    parser.add_argument("--phase-b-epochs", type=int, default=20, help="Epochs for Phase B (freeze predictor, train update loop)")
    parser.add_argument("--phase-c-epochs", type=int, default=20, help="Epochs for Phase C (fine-tune full model end-to-end)")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--sequence-length", type=int, default=20)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--lr-phase2", type=float, default=None, help="Learning rate for Phase 2 (default: lr / 5)")
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--attack-prob", type=float, default=0.5, help="Probability of attacking a sample")
    parser.add_argument("--attack-types", nargs="*", default=["bias", "scale", "freeze", "noise", "ramp", "zero_out"],
                        help="Attack types to use during augmentation")
    parser.add_argument("--no-augmentation", action="store_true", help="Disable attack augmentation (train on clean only)")
    parser.add_argument("--lambda-mask", type=float, default=0.1, help="Weight for prediction mask supervision loss")
    parser.add_argument("--lambda-meas-mask", type=float, default=0.1, help="Weight for measurement mask supervision loss")
    parser.add_argument("--lambda-pred", type=float, default=0.2, help="Phase C prediction loss weight")
    parser.add_argument("--lambda-upd", type=float, default=0.8, help="Phase C update loss weight")
    parser.add_argument("--max-branches-attacked", type=int, default=1, help="Max branches attacked simultaneously")
    parser.add_argument("--reverse-split", action="store_true", help="Use first portion for validation and remaining for training")
    parser.add_argument(
        "--predictor-mode",
        default="nn",
        choices=["nn", "kinematic"],
        help="Prediction step: 'nn' (tri-LSTM, learnable, default) | 'kinematic' (unicycle model, no trainable parameters)",
    )
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    total_epochs = args.phase_a_epochs + args.phase_b_epochs + args.phase_c_epochs

    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else args.device if args.device != "auto" else "cpu")
    merged = merge_recorded_datasets(args.datasets)
    raw, z_seq, x_gt, x0, dt_seq = build_training_windows(merged, sequence_length=args.sequence_length, stride=args.stride)
    dataset = SlidingWindowDataset(raw, z_seq, x_gt, x0, dt_seq)

    val_size = int(len(dataset) * args.val_split)
    train_size = len(dataset) - val_size
    if train_size <= 0:
        raise SystemExit("Dataset too small after validation split")

    generator = torch.Generator().manual_seed(args.seed)
    if val_size > 0:
        # Prevent data leakage: Use chronological split instead of random split for sliding window time series
        indices = list(range(len(dataset)))
        if args.reverse_split:
            val_subset = torch.utils.data.Subset(dataset, indices[:val_size])
            train_subset = torch.utils.data.Subset(dataset, indices[val_size:])
        else:
            train_subset = torch.utils.data.Subset(dataset, indices[:train_size])
            val_subset = torch.utils.data.Subset(dataset, indices[train_size:])
        val_loader = DataLoader(SubsetWithTransform(val_subset), batch_size=args.batch_size, shuffle=False, collate_fn=collate_batch)
    else:
        train_subset = dataset
        val_loader = None

    train_loader = DataLoader(SubsetWithTransform(train_subset), batch_size=args.batch_size, shuffle=True, collate_fn=collate_batch)

    cfg = RSNConfig(
        dt=float(merged.get("metadata", {}).get("dt_mean", 0.02) or 0.02),
        predictor_mode=args.predictor_mode,
    )
    model = RobustStateNet(cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    state_weights = torch.tensor([1.0, 1.0, 5.0, 1.0, 1.0], device=device)

    print(f"\nPredictor mode : {args.predictor_mode}")
    if args.predictor_mode == "kinematic":
        print("  → Kinematic predictor: unicycle model (no learnable parameters in prediction step).")
        print("  → Phase A (Pretrain Predictor) is automatically skipped.")
        print("  → Prediction mask supervision (lambda_mask) is automatically skipped (pred_mask=None).")
        args.phase_a_epochs = 0

    # Initialize attack augmenter
    augmenter = None
    if not args.no_augmentation:
        attack_cfg = AttackConfig(
            attack_prob=args.attack_prob,
            enabled_attacks=args.attack_types,
            max_branches_attacked=args.max_branches_attacked,
        )
        augmenter = SensorAttackAugmenter(attack_cfg)
        print(f"Attack augmentation enabled: prob={args.attack_prob}, types={args.attack_types}")
    else:
        print("Attack augmentation DISABLED (training on clean data only)")

    best_val = float("inf")
    history = []
    output_path = Path(__file__).resolve().parent / args.output
    phase_c_final_path = output_path.with_name(f"{output_path.stem}.phase_c_final{output_path.suffix}")
    final_path = output_path.with_name(f"{output_path.stem}.final{output_path.suffix}")

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
                print(f"\n{'='*60}")
                print(f"Phase C started: End-to-end, TF OFF, lr={lr_phase_e2e:.2e}")
                print(f"{'='*60}\n")

        model.train()
        train_losses = []
        for batch in train_loader:
            raw_b, z_seq_b, x_gt_b, x0_b, dt_seq_b = move_batch_to_device(batch, device)

            # Apply attack augmentation
            attack_labels = None
            meas_attack_labels = None
            if augmenter is not None:
                raw_b, z_seq_b, attack_labels, meas_attack_labels = augmenter.augment_batch(raw_b, z_seq_b)

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
                meas_attack_labels=meas_attack_labels,
                lambda_meas_mask=lam_meas_mask,
            )
            loss.backward()
            optimizer.step()
            train_losses.append(float(loss.item()))

        train_loss = float(np.mean(train_losses)) if train_losses else float("nan")
        val_loss = float("nan")
        if val_loader is not None:
            val_loss, _ = evaluate(model, val_loader, device, teacher_forcing=False)
        else:
            val_loss = train_loss

        history.append({"epoch": epoch, "phase": phase, "train_loss": train_loss, "val_loss": val_loss})
        print(f"[{phase_label}] Epoch {epoch:03d} | train={train_loss:.6f} | val={val_loss:.6f}")

        if val_loss < best_val:
            best_val = val_loss
            save_checkpoint(
                model=model,
                cfg=cfg,
                args=args,
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

