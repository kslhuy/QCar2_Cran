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


class SlidingWindowDataset(Dataset):
    def __init__(self, raw: Dict[str, np.ndarray], z_seq: np.ndarray, x_gt: np.ndarray, x0: np.ndarray):
        self.raw = raw
        self.z_seq = z_seq
        self.x_gt = x_gt
        self.x0 = x0

    def __len__(self) -> int:
        return int(self.z_seq.shape[0])

    def __getitem__(self, idx: int):
        raw_item = {key: torch.from_numpy(self.raw[key][idx]) for key in RAW_KEYS}
        return raw_item, torch.from_numpy(self.z_seq[idx]), torch.from_numpy(self.x_gt[idx]), torch.from_numpy(self.x0[idx])


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
    return raw, z_seq, x_gt, x0


def move_batch_to_device(batch, device: torch.device):
    raw, z_seq, x_gt, x0 = batch
    raw = {k: v.to(device=device, dtype=torch.float32) for k, v in raw.items()}
    return raw, z_seq.to(device=device, dtype=torch.float32), x_gt.to(device=device, dtype=torch.float32), x0.to(device=device, dtype=torch.float32)


def evaluate(model, loader, device, teacher_forcing: bool) -> Tuple[float, Dict[str, float]]:
    model.eval()
    losses = []
    with torch.no_grad():
        for batch in loader:
            raw, z_seq, x_gt, x0 = move_batch_to_device(batch, device)
            out = model(
                raw=raw,
                z_seq=z_seq,
                x0=x0,
                teacher_forcing_state=x_gt if teacher_forcing else None,
            )
            state_weights = torch.tensor([1.0, 1.0, 5.0, 1.0, 1.0], device=device)
            loss, _ = robuststatenet_loss(out["x_pred"], out["x_upd"], x_gt, weights=state_weights)
            losses.append(float(loss.item()))
    mean_loss = float(np.mean(losses)) if losses else float("nan")
    return mean_loss, {"loss": mean_loss}


def main() -> None:
    parser = argparse.ArgumentParser(description="Train Robust KalmanNet offline from recorded datasets")
    parser.add_argument("datasets", nargs="+", help="One or more .npz dataset files recorded from the vehicle")
    parser.add_argument("--output", default="models/robust_kalmannet.pt", help="Checkpoint path relative to this script")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--sequence-length", type=int, default=20)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--teacher-forcing", action="store_true", default=True)
    parser.add_argument("--no-teacher-forcing", dest="teacher_forcing", action="store_false")
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else args.device if args.device != "auto" else "cpu")
    merged = merge_recorded_datasets(args.datasets)
    raw, z_seq, x_gt, x0 = build_training_windows(merged, sequence_length=args.sequence_length, stride=args.stride)
    dataset = SlidingWindowDataset(raw, z_seq, x_gt, x0)

    val_size = int(len(dataset) * args.val_split)
    train_size = len(dataset) - val_size
    if train_size <= 0:
        raise SystemExit("Dataset too small after validation split")

    generator = torch.Generator().manual_seed(args.seed)
    if val_size > 0:
        train_subset, val_subset = random_split(dataset, [train_size, val_size], generator=generator)
        val_loader = DataLoader(SubsetWithTransform(val_subset), batch_size=args.batch_size, shuffle=False, collate_fn=collate_batch)
    else:
        train_subset = dataset
        val_loader = None

    train_loader = DataLoader(SubsetWithTransform(train_subset), batch_size=args.batch_size, shuffle=True, collate_fn=collate_batch)

    cfg = RSNConfig(dt=float(merged.get("metadata", {}).get("dt_mean", 0.02) or 0.02))
    model = RobustStateNet(cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    state_weights = torch.tensor([1.0, 1.0, 5.0, 1.0, 1.0], device=device)

    best_val = float("inf")
    history = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_losses = []
        for batch in train_loader:
            raw_b, z_seq_b, x_gt_b, x0_b = move_batch_to_device(batch, device)
            optimizer.zero_grad()
            out = model(
                raw=raw_b,
                z_seq=z_seq_b,
                x0=x0_b,
                teacher_forcing_state=x_gt_b if args.teacher_forcing else None,
            )
            loss, logs = robuststatenet_loss(
                out["x_pred"],
                out["x_upd"],
                x_gt_b,
                lambda_upd=0.8,
                lambda_pred=0.2,
                weights=state_weights,
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

        history.append({"epoch": epoch, "train_loss": train_loss, "val_loss": val_loss})
        print(f"Epoch {epoch:03d} | train={train_loss:.6f} | val={val_loss:.6f}")

        if val_loss < best_val:
            best_val = val_loss
            output_path = Path(__file__).resolve().parent / args.output
            output_path.parent.mkdir(parents=True, exist_ok=True)
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "config": vars(cfg),
                    "train_args": vars(args),
                    "history": history,
                    "metadata": merged.get("metadata", {}),
                    "best_val_loss": best_val,
                },
                output_path,
            )
            print(f"Saved checkpoint to {output_path}")

    summary_path = (Path(__file__).resolve().parent / args.output).with_suffix(".train_history.json")
    summary_path.write_text(json.dumps(history, indent=2), encoding="utf-8")
    print(f"Training history saved to {summary_path}")


if __name__ == "__main__":
    main()
