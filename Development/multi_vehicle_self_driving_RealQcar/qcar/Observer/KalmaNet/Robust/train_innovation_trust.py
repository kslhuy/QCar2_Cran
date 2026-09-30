"""Train sensor trust on causal MockQCar rollouts; export a NumPy runtime MLP.

Drive seeds, not overlapping windows, define train/validation partitions.
Training combines fault supervision and a normalized state-correction loss.
The selected checkpoint is assessed separately by simulation_benchmark.py.
"""
from __future__ import annotations

import argparse
import copy
import json
import hashlib
from pathlib import Path
import numpy as np
import torch
from torch import nn

from innovation_trust import InnovationTrustFilter, wrap
from simulation_benchmark import HERE, SCENARIOS, attacked_samples, generate_drive, model_config


def collect(seeds, duration, checkpoint=None):
    features, targets, residuals, errors, gains = [], [], [], [], []
    for seed in seeds:
        # Robustness to modest calibration error is represented in training.
        drive = generate_drive(seed, duration, mismatch=(seed % 3-1)*.08,
                               gps_hz=[5., 10., 20.][seed % 3])
        for scenario in SCENARIOS:
            samples, _, labels = attacked_samples(drive, scenario, seed+2000)
            estimator = InnovationTrustFilter(drive["initial"], model_config(), checkpoint)
            for i, sample in enumerate(samples):
                estimator.update(**sample)
                clean = drive["samples"][i]
                available = np.array([sample["gps_data"] is not None]*3+[True])
                if not available.any():
                    continue
                corruption = np.zeros(4)
                if sample["gps_data"] is not None and clean["gps_data"] is not None:
                    corruption[:3] = [sample["gps_data"][k]-clean["gps_data"][k]
                                      for k in ("x", "y", "theta")]
                    corruption[2] = wrap(corruption[2])
                corruption[3] = sample["motor_tach"]-clean["motor_tach"]
                # A frozen signal is still useful before it deviates noticeably.
                target = np.exp(-(abs(corruption)/np.array([.07, .07, .05, .06]))**2)
                target[:2] = min(target[:2])
                r = estimator.last_measurement-estimator.last_prediction
                r[2] = wrap(r[2])
                e = estimator.last_prediction-drive["truth"][i]
                e[2] = wrap(e[2])
                scale = np.array([.04, .04, .02, .025])
                features.append(estimator.last_features[available])
                targets.append(target[available])
                residuals.append((r/scale)[available])
                errors.append((e/scale)[available])
                # Include the analytical rejection prior in the correction loss.
                learned = np.maximum(estimator.predict_trust(estimator.last_features), 1e-6)
                gains.append((estimator.last_gain*estimator.last_trust/learned)[available])
        print(f"Collected drive seed={seed}, {len(samples)} ticks x {len(SCENARIOS)} scenarios", flush=True)
    return [np.concatenate(x).astype(np.float32) for x in (features, targets, residuals, errors, gains)]


def save_model(model, path, metadata):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, w1=model[0].weight.detach().cpu().numpy(),
                        b1=model[0].bias.detach().cpu().numpy(),
                        w2=model[2].weight.detach().cpu().numpy(),
                        b2=model[2].bias.detach().cpu().numpy(),
                        metadata=np.array(json.dumps(metadata)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE/"models/innovation_trust_sim.npz")
    parser.add_argument("--train-seeds", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6])
    parser.add_argument("--val-seeds", nargs="+", type=int, default=[21, 22])
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--cache", type=Path, default=HERE/"results/trust_training_data.npz")
    args = parser.parse_args(argv)
    if set(args.train_seeds) & set(args.val_seeds):
        raise ValueError("Training and validation drive seeds must be disjoint")
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    model = nn.Sequential(nn.Linear(InnovationTrustFilter.FEATURE_COUNT, 24), nn.ReLU(), nn.Linear(24, 1))
    metadata = dict(format=InnovationTrustFilter.FORMAT, train_seeds=args.train_seeds,
                    val_seeds=args.val_seeds, training_seed=args.seed, duration=args.duration,
                    model_config=model_config(), measurement_std=[.035, .035, .015, .012],
                    feature_count=InnovationTrustFilter.FEATURE_COUNT,
                    objective="balanced BCE + normalized bounded state correction MSE",
                    train_scenarios=list(SCENARIOS), history=[])
    for iteration in range(args.rounds):
        checkpoint = args.output if iteration else None
        cache = args.cache.with_name(args.cache.stem+f"_round{iteration}.npz")
        source_hash = hashlib.sha256((HERE/"innovation_trust.py").read_bytes()
                                     + (HERE/"simulation_benchmark.py").read_bytes()).hexdigest()
        signature = json.dumps(dict(train=args.train_seeds, val=args.val_seeds,
                                    duration=args.duration, iteration=iteration, source_hash=source_hash))
        # Cache is only reused for the analytic first pass; later passes depend on weights.
        train, val = None, None
        if iteration == 0 and cache.exists():
            with np.load(cache) as data:
                if str(data["signature"]) == signature:
                    train = [data[f"train_{i}"] for i in range(5)]
                    val = [data[f"val_{i}"] for i in range(5)]
                else:
                    print("Regenerating stale training cache", flush=True)
        if train is None:
            train = collect(args.train_seeds, args.duration, checkpoint)
            val = collect(args.val_seeds, args.duration, checkpoint)
            cache.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(cache, signature=np.array(signature),
                                **{f"train_{i}": x for i, x in enumerate(train)},
                                **{f"val_{i}": x for i, x in enumerate(val)})
        train_t = [torch.from_numpy(x) for x in train]
        val_t = [torch.from_numpy(x) for x in val]
        optimizer = torch.optim.AdamW(model.parameters(), lr=.003, weight_decay=1e-4)
        best, best_state, stale = float("inf"), None, 0

        def loss(batch):
            f, target, residual, error, gain = batch
            logits = model(f).squeeze(-1)
            # Missed faults are costly, while clean examples remain the majority.
            weight = 1 + 3*(1-target)
            bce = (torch.nn.functional.binary_cross_entropy_with_logits(logits, target, reduction="none")*weight).mean()
            updated_error = error + torch.sigmoid(logits)*gain*residual
            state = updated_error.square().clamp(max=100).mean()
            return bce + .08*state

        for epoch in range(args.epochs):
            model.train()
            order = rng.permutation(len(train[0]))
            values = []
            for start in range(0, len(order), 4096):
                idx = order[start:start+4096]
                optimizer.zero_grad(set_to_none=True)
                value = loss([x[idx] for x in train_t])
                value.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 2.)
                optimizer.step()
                values.append(float(value.detach()))
            model.eval()
            with torch.no_grad():
                score = float(loss(val_t))
            metadata["history"].append(dict(round=iteration, epoch=epoch, train_loss=float(np.mean(values)), val_loss=score))
            if score < best-1e-5:
                best, best_state, stale = score, copy.deepcopy(model.state_dict()), 0
            else:
                stale += 1
            if epoch % 10 == 0:
                print(f"round={iteration} epoch={epoch:03d} train={np.mean(values):.5f} val={score:.5f}", flush=True)
            if stale >= 15:
                break
        model.load_state_dict(best_state)
        metadata["validation_loss"] = best
        save_model(model, args.output, metadata)
        print(f"Saved {args.output}, validation_loss={best:.6f}", flush=True)
    args.output.with_suffix(".history.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
