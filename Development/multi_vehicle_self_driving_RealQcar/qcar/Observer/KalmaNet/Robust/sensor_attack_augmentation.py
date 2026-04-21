"""
Sensor Attack Augmentation for Robust KalmanNet Training.

Implements configurable adversarial data augmentation that randomly corrupts
sensor branches during training, forcing the masking network to learn
cross-branch consistency checking and automatic attack suppression.

Attack types follow common sensor attack models from autonomous driving
security literature (bias, scaling, freezing, ramp, noise, zero-out).

Reference:
    Dahal et al., "Robust ego vehicle state estimation for Autonomous
    Driving", Robotics and Autonomous Systems, 2023.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    torch = None
    TORCH_AVAILABLE = False


# ============================================================
# BRANCH DEFINITIONS
# ============================================================

# Which raw keys belong to each sensor branch
BRANCH_KEYS: Dict[str, List[str]] = {
    "imu":   ["ax", "ay", "wz"],
    "steer": ["delta"],
    "wheel": ["vfl", "vfr", "vrl", "vrr"],
}

BRANCH_NAMES = list(BRANCH_KEYS.keys())          # ["imu", "steer", "wheel"]
BRANCH_INDEX = {name: i for i, name in enumerate(BRANCH_NAMES)}

# Measurement z = [x, y, psi, v, w]
# Indices 0,1,2 → GNSS-related ; 3 → motor tach ; 4 → gyro
MEAS_GNSS_INDICES = [0, 1, 2]
MEAS_VELOCITY_INDEX = 3
MEAS_YAWRATE_INDEX = 4

GPS_POSITION_ATTACK_TYPES = [
    "noise",
    "freeze",
    "jump",
    "dropout",
    "reacquisition",
]


# ============================================================
# CONFIG
# ============================================================

@dataclass
class AttackConfig:
    """Configuration for sensor attack augmentation."""

    # Probability of attacking *any* sensor in a given sample
    attack_prob: float = 0.5

    # Probability of attacking each branch (given an attack occurs)
    branch_attack_probs: Dict[str, float] = field(default_factory=lambda: {
        "imu": 0.4,
        "steer": 0.3,
        "wheel": 0.3,
    })

    # Probability of also attacking the measurement z
    meas_attack_prob: float = 0.3

    # Independent probability of corrupting GPS x/y measurement channels.
    # These attacks target the update step's position correction behavior.
    gps_attack_prob: float = 0.3
    gps_attack_types: List[str] = field(default_factory=lambda: list(GPS_POSITION_ATTACK_TYPES))

    # Which attack types to use (uniform random selection among enabled)
    enabled_attacks: List[str] = field(default_factory=lambda: [
        "bias", "scale", "freeze", "noise", "ramp", "zero_out",
    ])

    # Severity parameters
    bias_range: Tuple[float, float] = (0.5, 3.0)     # absolute bias magnitude
    scale_range: Tuple[float, float] = (0.1, 5.0)    # multiplicative factor
    noise_std_range: Tuple[float, float] = (0.5, 2.0) # noise standard deviation
    ramp_rate_range: Tuple[float, float] = (0.02, 0.2) # ramp per timestep
    gps_noise_std_range: Tuple[float, float] = (0.03, 0.25) # meters
    gps_jump_range: Tuple[float, float] = (0.20, 1.20)      # meters
    gps_reacq_jump_range: Tuple[float, float] = (0.10, 0.80) # meters
    gps_dropout_min_fraction: float = 0.20
    gps_dropout_max_fraction: float = 0.80

    # Maximum number of branches to attack simultaneously (1 = single-branch)
    max_branches_attacked: int = 1

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "AttackConfig":
        cfg = cls()
        for key, value in d.items():
            if hasattr(cfg, key):
                current = getattr(cfg, key)
                if isinstance(current, tuple):
                    setattr(cfg, key, tuple(value))
                else:
                    setattr(cfg, key, value)
        return cfg


# ============================================================
# INDIVIDUAL ATTACK FUNCTIONS (operate on torch Tensors)
# ============================================================

def _attack_bias(tensor: "torch.Tensor", config: AttackConfig) -> "torch.Tensor":
    """Add constant bias to the entire sequence."""
    lo, hi = config.bias_range
    magnitude = lo + (hi - lo) * torch.rand(1, device=tensor.device).item()
    sign = 1.0 if torch.rand(1).item() > 0.5 else -1.0
    return tensor + sign * magnitude


def _attack_scale(tensor: "torch.Tensor", config: AttackConfig) -> "torch.Tensor":
    """Multiply by a random scaling factor."""
    lo, hi = config.scale_range
    factor = lo + (hi - lo) * torch.rand(1, device=tensor.device).item()
    return tensor * factor


def _attack_freeze(tensor: "torch.Tensor", config: AttackConfig) -> "torch.Tensor":
    """Freeze sensor — repeat the first timestep value for all time."""
    # tensor shape: [B, T, D] or [T, D]
    if tensor.dim() == 3:
        frozen_val = tensor[:, 0:1, :].clone()
        return frozen_val.expand_as(tensor)
    else:
        frozen_val = tensor[0:1, :].clone()
        return frozen_val.expand_as(tensor)


def _attack_noise(tensor: "torch.Tensor", config: AttackConfig) -> "torch.Tensor":
    """Add large Gaussian noise (much larger than natural sensor noise)."""
    lo, hi = config.noise_std_range
    std = lo + (hi - lo) * torch.rand(1, device=tensor.device).item()
    return tensor + torch.randn_like(tensor) * std


def _attack_ramp(tensor: "torch.Tensor", config: AttackConfig) -> "torch.Tensor":
    """Linearly increasing offset over time (ramp attack)."""
    lo, hi = config.ramp_rate_range
    rate = lo + (hi - lo) * torch.rand(1, device=tensor.device).item()
    sign = 1.0 if torch.rand(1).item() > 0.5 else -1.0

    if tensor.dim() == 3:
        T = tensor.shape[1]
        ramp = torch.arange(T, device=tensor.device, dtype=tensor.dtype).view(1, T, 1)
    else:
        T = tensor.shape[0]
        ramp = torch.arange(T, device=tensor.device, dtype=tensor.dtype).view(T, 1)

    return tensor + sign * rate * ramp


def _attack_zero_out(tensor: "torch.Tensor", config: AttackConfig) -> "torch.Tensor":
    """Zero out entire sensor branch."""
    return torch.zeros_like(tensor)


ATTACK_FNS = {
    "bias":     _attack_bias,
    "scale":    _attack_scale,
    "freeze":   _attack_freeze,
    "noise":    _attack_noise,
    "ramp":     _attack_ramp,
    "zero_out": _attack_zero_out,
}


# ============================================================
# AUGMENTER
# ============================================================

class SensorAttackAugmenter:
    """
    Applies random sensor attacks to training batches.

    Usage::

        augmenter = SensorAttackAugmenter(AttackConfig(attack_prob=0.5))
        raw_corrupted, z_corrupted, attack_labels = augmenter.augment_batch(raw, z_seq)

    ``attack_labels`` is a [B, 3] binary tensor where 1 indicates the branch
    (imu=0, steer=1, wheel=2) was attacked. This can be used for optional
    supervised mask loss during training.
    """

    def __init__(self, config: Optional[AttackConfig] = None):
        self.config = config or AttackConfig()
        self._validate_config()

    def _validate_config(self) -> None:
        if not self.config.enabled_attacks:
            raise ValueError("At least one branch attack type must be enabled")
        for name in self.config.enabled_attacks:
            if name not in ATTACK_FNS:
                raise ValueError(
                    f"Unknown attack type '{name}'. "
                    f"Available: {list(ATTACK_FNS.keys())}"
                )
        if not self.config.gps_attack_types:
            raise ValueError("At least one GPS position attack type must be enabled")
        for name in self.config.gps_attack_types:
            if name not in GPS_POSITION_ATTACK_TYPES:
                raise ValueError(
                    f"Unknown GPS attack type '{name}'. "
                    f"Available: {GPS_POSITION_ATTACK_TYPES}"
                )

    def _pick_attack_fn(self):
        """Randomly select one attack type from enabled attacks."""
        idx = int(torch.randint(len(self.config.enabled_attacks), (1,)).item())
        name = self.config.enabled_attacks[idx]
        return name, ATTACK_FNS[name]

    def _pick_gps_attack_name(self) -> str:
        idx = int(torch.randint(len(self.config.gps_attack_types), (1,)).item())
        return str(self.config.gps_attack_types[idx])

    def _should_attack(self) -> bool:
        return torch.rand(1).item() < self.config.attack_prob

    def _pick_branches_to_attack(self) -> List[str]:
        """Select which branches to attack based on config probabilities."""
        candidates = []
        for branch_name, prob in self.config.branch_attack_probs.items():
            if torch.rand(1).item() < prob:
                candidates.append(branch_name)

        if not candidates:
            # At least one branch must be attacked
            idx = int(torch.randint(len(BRANCH_NAMES), (1,)).item())
            candidates = [BRANCH_NAMES[idx]]

        # Limit to max_branches_attacked
        if len(candidates) > self.config.max_branches_attacked:
            perm = torch.randperm(len(candidates))
            candidates = [candidates[i] for i in perm[: self.config.max_branches_attacked]]

        return candidates

    def _random_window(
        self,
        T: int,
        min_fraction: Optional[float] = None,
        max_fraction: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Pick a contiguous time window [start, end) inside one sequence."""
        if T <= 1:
            return 0, max(T, 1)

        lo_frac = self.config.gps_dropout_min_fraction if min_fraction is None else min_fraction
        hi_frac = self.config.gps_dropout_max_fraction if max_fraction is None else max_fraction
        lo_frac = float(np.clip(lo_frac, 0.0, 1.0))
        hi_frac = float(np.clip(max(hi_frac, lo_frac), 0.0, 1.0))
        min_len = max(1, int(round(T * lo_frac)))
        max_len = max(min_len, int(round(T * hi_frac)))
        max_len = min(max_len, T)
        length = min_len
        if max_len > min_len:
            length = int(torch.randint(min_len, max_len + 1, (1,)).item())
        max_start = max(0, T - length)
        start = 0
        if max_start > 0:
            start = int(torch.randint(0, max_start + 1, (1,)).item())
        return start, start + length

    @staticmethod
    def _sample_xy_offset(
        magnitude_range: Tuple[float, float],
        device,
        dtype,
    ) -> "torch.Tensor":
        lo, hi = magnitude_range
        magnitude = float(lo) + (float(hi) - float(lo)) * torch.rand((), device=device, dtype=dtype)
        angle = 2.0 * np.pi * torch.rand((), device=device, dtype=dtype)
        return torch.stack([torch.cos(angle), torch.sin(angle)]) * magnitude

    @staticmethod
    def _fill_raw_window(
        raw_out: Dict[str, "torch.Tensor"],
        key: str,
        batch_index: int,
        start: int,
        end: int,
        value: float,
    ) -> None:
        if key in raw_out and end > start:
            raw_out[key][batch_index, start:end, :].fill_(float(value))

    @staticmethod
    def _set_gps_age_ramp(
        raw_out: Dict[str, "torch.Tensor"],
        batch_index: int,
        start: int,
        end: int,
    ) -> None:
        if "gps_age_sec" not in raw_out or end <= start:
            return
        target = raw_out["gps_age_sec"][batch_index, start:end, :]
        ramp = torch.arange(
            end - start,
            device=target.device,
            dtype=target.dtype,
        ).view(-1, 1)
        target.copy_(ramp * 0.02)

    def _apply_gps_position_attack(
        self,
        raw_out: Dict[str, "torch.Tensor"],
        z_out: "torch.Tensor",
        meas_attack_labels: "torch.Tensor",
        batch_index: int,
    ) -> None:
        """Corrupt GPS x/y measurements and matching validity side channels."""
        T = int(z_out.shape[1])
        if T <= 0:
            return

        attack_name = self._pick_gps_attack_name()
        xy = z_out[batch_index, :, 0:2]
        device = z_out.device
        dtype = z_out.dtype

        if attack_name == "noise":
            lo, hi = self.config.gps_noise_std_range
            std = float(lo) + (float(hi) - float(lo)) * torch.rand((), device=device, dtype=dtype)
            xy.add_(torch.randn_like(xy) * std)
            mark_start, mark_end = 0, T

        elif attack_name == "freeze":
            start, end = self._random_window(T)
            source_idx = max(0, start - 1)
            source_xy = xy[source_idx : source_idx + 1, :].clone()
            xy[start:end, :] = source_xy.expand(end - start, 2)
            mark_start, mark_end = start, end

        elif attack_name == "jump":
            start, end = self._random_window(T)
            offset = self._sample_xy_offset(self.config.gps_jump_range, device, dtype)
            xy[start:end, :].add_(offset.view(1, 2))
            mark_start, mark_end = start, end

        elif attack_name == "dropout":
            start, end = self._random_window(T)
            source_idx = max(0, start - 1)
            source_xy = xy[source_idx : source_idx + 1, :].clone()
            xy[start:end, :] = source_xy.expand(end - start, 2)
            self._fill_raw_window(raw_out, "gps_valid", batch_index, start, end, 0.0)
            self._fill_raw_window(raw_out, "gps_hold_valid", batch_index, start, end, 1.0)
            self._set_gps_age_ramp(raw_out, batch_index, start, end)
            mark_start, mark_end = start, end

        elif attack_name == "reacquisition":
            start, end = self._random_window(T, min_fraction=0.25, max_fraction=0.70)
            reacq_idx = max(start, end - 1)
            source_idx = max(0, start - 1)
            if reacq_idx > start:
                source_xy = xy[source_idx : source_idx + 1, :].clone()
                xy[start:reacq_idx, :] = source_xy.expand(reacq_idx - start, 2)
                self._fill_raw_window(raw_out, "gps_valid", batch_index, start, reacq_idx, 0.0)
                self._fill_raw_window(raw_out, "gps_hold_valid", batch_index, start, reacq_idx, 1.0)
                self._set_gps_age_ramp(raw_out, batch_index, start, reacq_idx)
            offset = self._sample_xy_offset(self.config.gps_reacq_jump_range, device, dtype)
            xy[reacq_idx:end, :].add_(offset.view(1, 2))
            self._fill_raw_window(raw_out, "gps_valid", batch_index, reacq_idx, end, 1.0)
            self._fill_raw_window(raw_out, "gps_hold_valid", batch_index, reacq_idx, end, 1.0)
            self._fill_raw_window(raw_out, "gps_age_sec", batch_index, reacq_idx, end, 0.0)
            mark_start, mark_end = start, end

        else:
            raise ValueError(f"Unsupported GPS attack type: {attack_name}")

        if mark_end > mark_start:
            meas_attack_labels[batch_index, 0:2] = 1.0

    def augment_batch(
        self,
        raw: Dict[str, "torch.Tensor"],
        z_seq: "torch.Tensor",
    ) -> Tuple[Dict[str, "torch.Tensor"], "torch.Tensor", "torch.Tensor", "torch.Tensor"]:
        """
        Apply random sensor attacks to a training batch.

        Args:
            raw: Dict of raw sensor tensors, each [B, T, 1].
                 Keys: ax, ay, wz, delta, vfl, vfr, vrl, vrr
            z_seq: Measurement tensor [B, T, 5] = [x, y, psi, v, w]

        Returns:
            raw_corrupted: Same structure as raw, with attacks applied
            z_corrupted: Measurement tensor with optional attack applied
            attack_labels: [B, 3] binary tensor.
                           Column 0 = IMU attacked, 1 = Steer, 2 = Wheel.
                           Use for optional supervised prediction mask loss.
            meas_attack_labels: [B, 5] binary tensor.
                           Per-channel measurement attack indicator.
                           [x, y, psi, v, w] — 1 if that channel was corrupted.
                           Use for optional supervised measurement mask loss.
        """
        if not TORCH_AVAILABLE:
            raise ImportError("torch is required for sensor attack augmentation")

        B = z_seq.shape[0]
        device = z_seq.device

        # Clone inputs so originals are untouched
        raw_out = {k: v.clone() for k, v in raw.items()}
        z_out = z_seq.clone()

        # attack_labels[b, branch_idx] = 1 if that branch was attacked
        attack_labels = torch.zeros(B, len(BRANCH_NAMES), device=device)
        # meas_attack_labels[b, channel] = 1 if that z channel was corrupted
        meas_attack_labels = torch.zeros(B, 5, device=device)

        for b in range(B):
            branches: List[str] = []

            if self._should_attack():
                branches = self._pick_branches_to_attack()
                attack_name, attack_fn = self._pick_attack_fn()

                for branch_name in branches:
                    branch_idx = BRANCH_INDEX[branch_name]
                    attack_labels[b, branch_idx] = 1.0

                    # Attack all keys belonging to this branch
                    for key in BRANCH_KEYS[branch_name]:
                        raw_out[key][b] = attack_fn(
                            raw_out[key][b].unsqueeze(0), self.config
                        ).squeeze(0)

                # Optionally also attack the measurement z
                if torch.rand(1).item() < self.config.meas_attack_prob:
                    if "imu" in branches:
                        # Attack yaw rate in measurement
                        z_out[b, :, MEAS_YAWRATE_INDEX] = attack_fn(
                            z_out[b, :, MEAS_YAWRATE_INDEX:MEAS_YAWRATE_INDEX + 1].unsqueeze(0),
                            self.config,
                        ).squeeze(0).squeeze(-1)
                        # Mark yaw rate (4) and psi (2) as attacked in measurement
                        meas_attack_labels[b, MEAS_YAWRATE_INDEX] = 1.0
                        meas_attack_labels[b, 2] = 1.0  # psi is derived from IMU
                    if "wheel" in branches:
                        # Attack velocity in measurement
                        z_out[b, :, MEAS_VELOCITY_INDEX] = attack_fn(
                            z_out[b, :, MEAS_VELOCITY_INDEX:MEAS_VELOCITY_INDEX + 1].unsqueeze(0),
                            self.config,
                        ).squeeze(0).squeeze(-1)
                        # Mark velocity (3) as attacked in measurement
                        meas_attack_labels[b, MEAS_VELOCITY_INDEX] = 1.0

            if torch.rand(1).item() < self.config.gps_attack_prob:
                self._apply_gps_position_attack(raw_out, z_out, meas_attack_labels, b)

        return raw_out, z_out, attack_labels, meas_attack_labels


# ============================================================
# MASK SUPERVISION LOSS
# ============================================================

def mask_supervision_loss(
    pred_mask: "torch.Tensor",
    attack_labels: "torch.Tensor",
) -> "torch.Tensor":
    """
    Supervised loss that encourages the prediction mask to suppress
    features from attacked branches.

    The prediction mask has shape [B, T, 3*H] where H is the LSTM
    hidden dim. We need to compare against attack_labels [B, 3].

    Strategy: for each branch, compute the mean mask activation over
    the branch's portion of the feature vector. If the branch was
    attacked (label=1), the target mask value is 0 (suppress). If clean
    (label=0), target is 1 (keep).

    Args:
        pred_mask: [B, T, 3*H] — the mask output from FeatureMask
        attack_labels: [B, 3] — binary labels per branch

    Returns:
        Scalar loss
    """
    if not TORCH_AVAILABLE:
        raise ImportError("torch is required")

    B, T, total_dim = pred_mask.shape
    H = total_dim // 3  # hidden dim per branch

    # Split mask into per-branch mean activations: [B, T, 3]
    branch_masks = []
    for i in range(3):
        branch_slice = pred_mask[:, :, i * H: (i + 1) * H]  # [B, T, H]
        branch_mean = branch_slice.mean(dim=-1)                # [B, T]
        branch_masks.append(branch_mean)
    branch_mask_means = torch.stack(branch_masks, dim=-1)      # [B, T, 3]

    # Target: 1 for clean branches, 0 for attacked branches
    # attack_labels is [B, 3] → expand to [B, T, 3]
    target = (1.0 - attack_labels).unsqueeze(1).expand_as(branch_mask_means)

    # Binary cross-entropy loss
    loss = torch.nn.functional.binary_cross_entropy(
        branch_mask_means.clamp(1e-6, 1.0 - 1e-6),
        target,
        reduction="mean",
    )
    return loss
