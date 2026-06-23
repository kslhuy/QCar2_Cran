"""Smoke test for attack augmentation pipeline."""
import torch
from robustKLnet import RSNConfig, RobustStateNet, robuststatenet_loss, make_dummy_batch
from sensor_attack_augmentation import SensorAttackAugmenter, AttackConfig, mask_supervision_loss

print("=== Test 1: Forward pass returns pred_mask and meas_mask ===")
cfg = RSNConfig()
model = RobustStateNet(cfg)
raw, z_seq, x_gt, x0 = make_dummy_batch(B=4, T=10)
out = model(raw=raw, z_seq=z_seq, x0=x0, teacher_forcing_state=x_gt)
print("x_pred shape:", out["x_pred"].shape)
print("x_upd shape:", out["x_upd"].shape)
print("pred_mask shape:", out["pred_mask"].shape)
print("pred_mask range:", round(out["pred_mask"].min().item(), 4), "-", round(out["pred_mask"].max().item(), 4))
print("meas_mask shape:", out["meas_mask"].shape)
print("meas_mask range:", round(out["meas_mask"].min().item(), 4), "-", round(out["meas_mask"].max().item(), 4))
assert "pred_mask" in out, "pred_mask missing from output"
assert out["pred_mask"].shape == (4, 10, 3 * cfg.pred_hidden), f"Bad mask shape: {out['pred_mask'].shape}"
assert "meas_mask" in out, "meas_mask missing from output"
assert out["meas_mask"].shape == (4, 10, 5), f"Bad meas_mask shape: {out['meas_mask'].shape}"

print()
print("=== Test 2: Attack augmentation ===")
aug = SensorAttackAugmenter(AttackConfig(attack_prob=1.0))
raw_aug, z_aug, labels, meas_labels = aug.augment_batch(raw, z_seq)
print("attack_labels shape:", labels.shape)
print("meas_attack_labels shape:", meas_labels.shape)
print("attacked branches per sample:", labels.sum(dim=0).tolist())
print("attacked meas channels per sample:", meas_labels.sum(dim=0).tolist())
diff = sum((raw_aug[k] - raw[k]).abs().sum().item() for k in raw)
print("raw data changed:", diff > 0)
assert diff > 0, "Augmentation did not modify data"
assert labels.shape == (4, 3), f"Bad labels shape: {labels.shape}"
assert meas_labels.shape == (4, 5), f"Bad meas_labels shape: {meas_labels.shape}"

print()
print("=== Test 3: Loss with pred mask + meas mask supervision ===")
loss, logs = robuststatenet_loss(
    out["x_pred"], out["x_upd"], x_gt,
    pred_mask=out["pred_mask"], attack_labels=labels, lambda_mask=0.1,
    meas_mask=out["meas_mask"], meas_attack_labels=meas_labels, lambda_meas_mask=0.1,
)
print("total loss:", round(loss.item(), 4))
print("logs:", {k: round(v, 4) for k, v in logs.items()})
assert "loss_mask" in logs, "loss_mask missing from logs"
assert "loss_meas_mask" in logs, "loss_meas_mask missing from logs"

print()
print("=== Test 4: Gradients flow through both masks ===")
loss.backward()
mask_grad = model.predictor.mask_net.net[0].weight.grad
print("pred mask_net gradient exists:", mask_grad is not None)
print("pred mask_net gradient norm:", round(mask_grad.norm().item(), 6))
assert mask_grad is not None, "No gradient on pred mask_net"
assert mask_grad.norm().item() > 0, "Zero gradient on pred mask_net"

meas_mask_grad = model.updater.meas_mask_net[0].weight.grad
print("meas_mask_net gradient exists:", meas_mask_grad is not None)
print("meas_mask_net gradient norm:", round(meas_mask_grad.norm().item(), 6))
assert meas_mask_grad is not None, "No gradient on meas_mask_net"
assert meas_mask_grad.norm().item() > 0, "Zero gradient on meas_mask_net"

print()
print("=== Test 5: All attack types produce valid output ===")
for attack_type in ["bias", "scale", "freeze", "noise", "ramp", "zero_out"]:
    single_cfg = AttackConfig(attack_prob=1.0, enabled_attacks=[attack_type])
    single_aug = SensorAttackAugmenter(single_cfg)
    r, z, l, ml = single_aug.augment_batch(raw, z_seq)
    assert all(torch.isfinite(r[k]).all() for k in r), f"{attack_type} produced non-finite raw"
    assert torch.isfinite(z).all(), f"{attack_type} produced non-finite z"
    print(f"  {attack_type}: OK")

print()
print("=== Test 6: No-attack mode (attack_prob=0) leaves data unchanged ===")
no_aug = SensorAttackAugmenter(AttackConfig(attack_prob=0.0))
r_clean, z_clean, l_clean, ml_clean = no_aug.augment_batch(raw, z_seq)
diff_clean = sum((r_clean[k] - raw[k]).abs().sum().item() for k in raw)
assert diff_clean == 0.0, "attack_prob=0 modified data!"
assert l_clean.sum().item() == 0.0, "attack_prob=0 produced attack labels!"
assert ml_clean.sum().item() == 0.0, "attack_prob=0 produced meas attack labels!"
print("  No-attack mode: OK")

print()
print("ALL SMOKE TESTS PASSED")
