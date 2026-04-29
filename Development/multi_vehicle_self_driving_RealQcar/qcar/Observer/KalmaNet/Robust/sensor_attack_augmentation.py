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

# Measurement z = [x, y, psi, v]
# Indices 0,1,2 -> GNSS/heading-related ; 3 -> motor tach
MEAS_GNSS_INDICES = [0, 1, 2]
MEAS_VELOCITY_INDEX = 3
DIRECT_MEASUREMENT_GROUPS: Tuple[Tuple[int, ...], ...] = (
    (2,),
    (3,),
    (2, 3),
)

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

    # Probability of also attacking the measurement z when a raw branch attack
    # already happened in the same sample.
    meas_attack_prob: float = 0.3

    # Independent probability of directly corrupting measurement-update
    # channels [psi, v] without touching raw predictor inputs.
    direct_meas_attack_prob: float = 0.3

    # Independent probability of corrupting GPS x/y measurement channels.
    # These attacks target the update step's position correction behavior.
    gps_attack_prob: float = 0.3
    gps_attack_types: List[str] = field(default_factory=lambda: list(GPS_POSITION_ATTACK_TYPES))
    measurement_focus_only: bool = False

    # Which attack types to use (uniform random selection among enabled)
    enabled_attacks: List[str] = field(default_factory=lambda: [
        "bias", "scale", "freeze", "noise", "ramp", "zero_out",
    ])

    # Severity parameters
    bias_range: Tuple[float, float] = (0.2, 1.0)     # absolute bias magnitude
    scale_range: Tuple[float, float] = (0.1, 2.0)    # multiplicative factor
    noise_std_range: Tuple[float, float] = (0.3, 1.0) # noise standard deviation
    ramp_rate_range: Tuple[float, float] = (0.02, 0.1) # ramp per timestep
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
            meas_attack_labels[batch_index, mark_start:mark_end, 0:2] = 1.0

    def _apply_direct_measurement_attack(
        self,
        z_out: "torch.Tensor",
        meas_attack_labels: "torch.Tensor",
        batch_index: int,
    ) -> None:
        """Directly corrupt measurement channels used by the updater."""
        attack_name, attack_fn = self._pick_attack_fn()
        group_idx = int(torch.randint(len(DIRECT_MEASUREMENT_GROUPS), (1,)).item())
        channels = DIRECT_MEASUREMENT_GROUPS[group_idx]
        attacked = attack_fn(
            z_out[batch_index, :, list(channels)].unsqueeze(0),
            self.config,
        ).squeeze(0)
        z_out[batch_index, :, list(channels)] = attacked
        meas_attack_labels[batch_index, :, list(channels)] = 1.0

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
            z_seq: Measurement tensor [B, T, 4] = [x, y, psi, v]

        Returns:
            raw_corrupted: Same structure as raw, with attacks applied
            z_corrupted: Measurement tensor with optional attack applied
            attack_labels: [B, 3] binary tensor.
                           Column 0 = IMU attacked, 1 = Steer, 2 = Wheel.
                           Use for optional supervised prediction mask loss.
            meas_attack_labels: [B, T, n] binary tensor.
                           Per-timestep, per-channel measurement attack indicator.
                           [x, y, psi, v] — 1 if that channel was corrupted.
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
        T = z_seq.shape[1]
        # meas_attack_labels[b, t, channel] = 1 if z[t, channel] was corrupted
        meas_attack_labels = torch.zeros(
            B,
            T,
            int(z_seq.shape[-1]),
            device=device,
            dtype=z_seq.dtype,
        )

        for b in range(B):
            branches: List[str] = []

            if not self.config.measurement_focus_only and self._should_attack():
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
                        z_out[b, :, 2] = attack_fn(
                            z_out[b, :, 2:3].unsqueeze(0),
                            self.config,
                        ).squeeze(0).squeeze(-1)
                        meas_attack_labels[b, :, 2] = 1.0  # psi is derived from IMU
                    if "wheel" in branches:
                        # Attack velocity in measurement
                        z_out[b, :, MEAS_VELOCITY_INDEX] = attack_fn(
                            z_out[b, :, MEAS_VELOCITY_INDEX:MEAS_VELOCITY_INDEX + 1].unsqueeze(0),
                            self.config,
                        ).squeeze(0).squeeze(-1)
                        # Mark velocity (3) as attacked in measurement
                        meas_attack_labels[b, :, MEAS_VELOCITY_INDEX] = 1.0

            if torch.rand(1).item() < self.config.direct_meas_attack_prob:
                self._apply_direct_measurement_attack(z_out, meas_attack_labels, b)

            if torch.rand(1).item() < self.config.gps_attack_prob:
                self._apply_gps_position_attack(raw_out, z_out, meas_attack_labels, b)

        return raw_out, z_out, attack_labels, meas_attack_labels


# ============================================================
# RUNTIME SENSOR FAILURE SIMULATION
# ============================================================


@dataclass
class RuntimeAttackConfig(AttackConfig):
    """Config for online attack/failure simulation during evaluation."""

    enabled: bool = False
    forced_branches: List[str] = field(default_factory=list)
    force_gps_attack: bool = False
    immediate_attack: bool = False
    min_attack_steps: int = 10
    max_attack_steps: int = 60
    gps_min_attack_steps: Optional[int] = None
    gps_max_attack_steps: Optional[int] = None
    cooldown_steps: int = 0
    gps_cooldown_steps: Optional[int] = None
    seed: Optional[int] = None

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "RuntimeAttackConfig":
        cfg = cls()
        for key, value in d.items():
            if hasattr(cfg, key):
                current = getattr(cfg, key)
                if isinstance(current, tuple):
                    setattr(cfg, key, tuple(value))
                else:
                    setattr(cfg, key, value)
        return cfg


class RuntimeSensorAttackSimulator:
    """
    Stateful online sensor fault simulator.

    This reuses the same attack vocabulary as offline training, but applies
    attacks over multiple observer ticks so runtime evaluation can reproduce
    realistic sensor failures instead of one-step perturbations.
    """

    BRANCH_RUNTIME_KEYS: Dict[str, Tuple[str, ...]] = {
        "imu": ("ax", "ay", "wz"),
        "steer": ("delta",),
        "wheel": ("motor_tach",),
    }

    def __init__(self, config: Optional[RuntimeAttackConfig] = None):
        self.config = config or RuntimeAttackConfig()
        self._rng = np.random.default_rng(self.config.seed)
        self._validate_config()
        self.reset()

    def reset(self) -> None:
        self._branch_events: Dict[str, Dict[str, Any]] = {}
        self._gps_event: Optional[Dict[str, Any]] = None
        self._branch_cooldown_remaining = 0
        self._gps_cooldown_remaining = 0
        self._last_gps_xy = np.zeros(2, dtype=np.float64)
        self._pending_immediate_attack = bool(self.config.immediate_attack)

    def _validate_config(self) -> None:
        if self.config.min_attack_steps <= 0:
            raise ValueError("min_attack_steps must be >= 1")
        if self.config.max_attack_steps < self.config.min_attack_steps:
            raise ValueError("max_attack_steps must be >= min_attack_steps")
        gps_min_steps = self._gps_min_attack_steps()
        gps_max_steps = self._gps_max_attack_steps()
        if gps_min_steps <= 0:
            raise ValueError("gps_min_attack_steps must be >= 1")
        if gps_max_steps < gps_min_steps:
            raise ValueError("gps_max_attack_steps must be >= gps_min_attack_steps")
        if self.config.enabled_attacks:
            for name in self.config.enabled_attacks:
                if name not in ATTACK_FNS:
                    raise ValueError(f"Unknown runtime branch attack type '{name}'")
        if self.config.gps_attack_types:
            for name in self.config.gps_attack_types:
                if name not in GPS_POSITION_ATTACK_TYPES:
                    raise ValueError(f"Unknown runtime GPS attack type '{name}'")
        if self.config.forced_branches:
            invalid_branches = [
                name for name in self.config.forced_branches if name not in BRANCH_NAMES
            ]
            if invalid_branches:
                raise ValueError(
                    f"Unknown forced runtime branches: {invalid_branches}. "
                    f"Available: {BRANCH_NAMES}"
                )

    def _gps_min_attack_steps(self) -> int:
        raw_value = (
            self.config.gps_min_attack_steps
            if self.config.gps_min_attack_steps is not None
            else self.config.min_attack_steps
        )
        return max(1, int(raw_value))

    def _gps_max_attack_steps(self) -> int:
        raw_value = (
            self.config.gps_max_attack_steps
            if self.config.gps_max_attack_steps is not None
            else self.config.max_attack_steps
        )
        return max(self._gps_min_attack_steps(), int(raw_value))

    def _rand(self) -> float:
        return float(self._rng.random())

    def _sample_sign(self) -> float:
        return 1.0 if self._rand() > 0.5 else -1.0

    def _sample_uniform(self, bounds: Tuple[float, float]) -> float:
        lo, hi = float(bounds[0]), float(bounds[1])
        if hi <= lo:
            return lo
        return float(self._rng.uniform(lo, hi))

    def _sample_duration(self, gps: bool = False) -> int:
        if gps:
            lo = self._gps_min_attack_steps()
            hi = self._gps_max_attack_steps()
        else:
            lo = max(1, int(self.config.min_attack_steps))
            hi = max(lo, int(self.config.max_attack_steps))
        if hi <= lo:
            return lo
        return int(self._rng.integers(lo, hi + 1))

    def _pick_attack_name(self) -> str:
        if not self.config.enabled_attacks:
            raise ValueError("No runtime branch attack types are enabled")
        idx = int(self._rng.integers(0, len(self.config.enabled_attacks)))
        return str(self.config.enabled_attacks[idx])

    def _pick_gps_attack_name(self) -> str:
        if not self.config.gps_attack_types:
            raise ValueError("No runtime GPS attack types are enabled")
        idx = int(self._rng.integers(0, len(self.config.gps_attack_types)))
        return str(self.config.gps_attack_types[idx])

    def _pick_branches_to_attack(self) -> List[str]:
        if self.config.forced_branches:
            forced = []
            for branch_name in self.config.forced_branches:
                if branch_name not in forced:
                    forced.append(branch_name)
            if not forced:
                raise ValueError("forced_branches was provided but empty after normalization")
            max_branches = max(1, int(self.config.max_branches_attacked))
            return forced[:max_branches]

        candidates: List[str] = []
        for branch_name in BRANCH_NAMES:
            prob = float(self.config.branch_attack_probs.get(branch_name, 0.0))
            if self._rand() < prob:
                candidates.append(branch_name)

        if not candidates:
            idx = int(self._rng.integers(0, len(BRANCH_NAMES)))
            candidates = [BRANCH_NAMES[idx]]

        if len(candidates) > int(self.config.max_branches_attacked):
            self._rng.shuffle(candidates)
            candidates = candidates[: int(self.config.max_branches_attacked)]
        return candidates

    def _sample_xy_offset(self, magnitude_range: Tuple[float, float]) -> np.ndarray:
        magnitude = self._sample_uniform(magnitude_range)
        angle = float(self._rng.uniform(0.0, 2.0 * np.pi))
        return np.asarray(
            [np.cos(angle) * magnitude, np.sin(angle) * magnitude],
            dtype=np.float64,
        )

    def _build_scalar_attack_state(self, attack_name: str, source_value: float) -> Dict[str, Any]:
        state: Dict[str, Any] = {"name": attack_name}
        if attack_name == "bias":
            state["bias"] = self._sample_sign() * self._sample_uniform(self.config.bias_range)
        elif attack_name == "scale":
            state["factor"] = self._sample_uniform(self.config.scale_range)
        elif attack_name == "freeze":
            state["freeze_value"] = float(source_value)
        elif attack_name == "noise":
            state["std"] = self._sample_uniform(self.config.noise_std_range)
        elif attack_name == "ramp":
            state["rate"] = self._sample_sign() * self._sample_uniform(self.config.ramp_rate_range)
        elif attack_name == "zero_out":
            pass
        else:
            raise ValueError(f"Unsupported runtime branch attack type: {attack_name}")
        return state

    @staticmethod
    def _extract_gps_xy(gps_data: Optional[Dict[str, Any]], fallback: np.ndarray) -> np.ndarray:
        if not isinstance(gps_data, dict):
            return np.asarray(fallback, dtype=np.float64).copy()
        try:
            x_val = float(gps_data.get("x", fallback[0]))
        except (TypeError, ValueError):
            x_val = float(fallback[0])
        try:
            y_val = float(gps_data.get("y", fallback[1]))
        except (TypeError, ValueError):
            y_val = float(fallback[1])
        xy = np.asarray([x_val, y_val], dtype=np.float64)
        if not np.all(np.isfinite(xy)):
            return np.asarray(fallback, dtype=np.float64).copy()
        return xy

    def _start_branch_events(
        self,
        motor_tach: float,
        steering: float,
        gyro_z: float,
        acceleration: np.ndarray,
    ) -> None:
        attack_name = self._pick_attack_name()
        duration = self._sample_duration(gps=False)
        current_values = {
            "imu": {
                "ax": float(acceleration[0]) if acceleration.size > 0 else 0.0,
                "ay": float(acceleration[1]) if acceleration.size > 1 else 0.0,
                "wz": float(gyro_z),
            },
            "steer": {"delta": float(steering)},
            "wheel": {"motor_tach": float(motor_tach)},
        }
        for branch_name in self._pick_branches_to_attack():
            channels = {
                key: self._build_scalar_attack_state(attack_name, value)
                for key, value in current_values[branch_name].items()
            }
            self._branch_events[branch_name] = {
                "branch": branch_name,
                "attack_type": attack_name,
                "remaining_steps": int(duration),
                "elapsed_steps": 0,
                "channels": channels,
            }

    def _start_gps_event(self, gps_data: Optional[Dict[str, Any]]) -> None:
        attack_name = self._pick_gps_attack_name()
        duration = self._sample_duration(gps=True)
        source_xy = self._extract_gps_xy(gps_data, self._last_gps_xy)
        event: Dict[str, Any] = {
            "attack_type": attack_name,
            "remaining_steps": int(max(2, duration) if attack_name == "reacquisition" else duration),
            "elapsed_steps": 0,
            "source_xy": source_xy,
        }
        if attack_name == "noise":
            event["std"] = self._sample_uniform(self.config.gps_noise_std_range)
        elif attack_name == "jump":
            event["offset"] = self._sample_xy_offset(self.config.gps_jump_range)
        elif attack_name == "reacquisition":
            total_steps = int(event["remaining_steps"])
            dropout_steps = max(1, total_steps - 1)
            event["dropout_steps"] = dropout_steps
            event["offset"] = self._sample_xy_offset(self.config.gps_reacq_jump_range)
        self._gps_event = event

    def _apply_scalar_attack(self, value: float, attack_state: Dict[str, Any], elapsed_steps: int) -> float:
        attack_name = str(attack_state.get("name", ""))
        if attack_name == "bias":
            return float(value + float(attack_state.get("bias", 0.0)))
        if attack_name == "scale":
            return float(value * float(attack_state.get("factor", 1.0)))
        if attack_name == "freeze":
            return float(attack_state.get("freeze_value", value))
        if attack_name == "noise":
            std = float(attack_state.get("std", 0.0))
            return float(value + self._rng.normal(0.0, std))
        if attack_name == "ramp":
            return float(value + float(attack_state.get("rate", 0.0)) * float(elapsed_steps))
        if attack_name == "zero_out":
            return 0.0
        return float(value)

    def _apply_gps_event(
        self,
        gps_data: Optional[Dict[str, Any]],
        dt: float,
    ) -> Optional[Dict[str, Any]]:
        if self._gps_event is None:
            return dict(gps_data) if isinstance(gps_data, dict) else gps_data

        gps_out = dict(gps_data or {})
        event = self._gps_event
        attack_name = str(event.get("attack_type", ""))
        elapsed_steps = int(event.get("elapsed_steps", 0))
        source_xy = np.asarray(event.get("source_xy", self._last_gps_xy), dtype=np.float64)
        current_xy = self._extract_gps_xy(gps_out, source_xy)

        xy_out = current_xy.copy()
        position_valid = bool(
            gps_out.get("position_valid", gps_out.get("fresh", gps_out.get("valid", False)))
        )
        fresh = bool(gps_out.get("fresh", position_valid))
        hold_valid = bool(gps_out.get("hold_valid", gps_out.get("valid", position_valid)))
        age_sec = float(max(0.0, gps_out.get("age_sec", 0.0))) if gps_out else 0.0

        if attack_name == "noise":
            std = float(event.get("std", 0.0))
            xy_out = current_xy + self._rng.normal(0.0, std, size=2)
        elif attack_name == "freeze":
            xy_out = source_xy.copy()
        elif attack_name == "jump":
            xy_out = current_xy + np.asarray(event.get("offset", np.zeros(2)), dtype=np.float64)
        elif attack_name == "dropout":
            xy_out = source_xy.copy()
            position_valid = False
            fresh = False
            hold_valid = True
            age_sec = float(max(0.0, elapsed_steps) * max(float(dt), 0.0))
        elif attack_name == "reacquisition":
            dropout_steps = int(event.get("dropout_steps", 1))
            if elapsed_steps < dropout_steps:
                xy_out = source_xy.copy()
                position_valid = False
                fresh = False
                hold_valid = True
                age_sec = float(max(0.0, elapsed_steps) * max(float(dt), 0.0))
            else:
                xy_out = current_xy + np.asarray(event.get("offset", np.zeros(2)), dtype=np.float64)
                position_valid = True
                fresh = True
                hold_valid = True
                age_sec = 0.0
        else:
            raise ValueError(f"Unsupported runtime GPS attack type: {attack_name}")

        gps_out["x"] = float(xy_out[0])
        gps_out["y"] = float(xy_out[1])
        gps_out["position_valid"] = bool(position_valid)
        gps_out["fresh"] = bool(fresh)
        gps_out["valid"] = bool(position_valid)
        gps_out["hold_valid"] = bool(hold_valid)
        gps_out["age_sec"] = float(max(0.0, age_sec))
        return gps_out

    def _advance_events(self) -> None:
        finished_branch = False
        for branch_name in list(self._branch_events.keys()):
            event = self._branch_events[branch_name]
            event["elapsed_steps"] = int(event.get("elapsed_steps", 0)) + 1
            event["remaining_steps"] = int(event.get("remaining_steps", 0)) - 1
            if int(event["remaining_steps"]) <= 0:
                del self._branch_events[branch_name]
                finished_branch = True
        if finished_branch:
            self._branch_cooldown_remaining = max(
                self._branch_cooldown_remaining,
                int(self.config.cooldown_steps),
            )
        elif self._branch_cooldown_remaining > 0 and not self._branch_events:
            self._branch_cooldown_remaining -= 1

        if self._gps_event is not None:
            self._gps_event["elapsed_steps"] = int(self._gps_event.get("elapsed_steps", 0)) + 1
            self._gps_event["remaining_steps"] = int(self._gps_event.get("remaining_steps", 0)) - 1
            if int(self._gps_event["remaining_steps"]) <= 0:
                self._gps_event = None
                gps_cooldown = (
                    self.config.gps_cooldown_steps
                    if self.config.gps_cooldown_steps is not None
                    else self.config.cooldown_steps
                )
                self._gps_cooldown_remaining = max(self._gps_cooldown_remaining, int(gps_cooldown))
        elif self._gps_cooldown_remaining > 0:
            self._gps_cooldown_remaining -= 1

    def _build_metadata(self) -> Dict[str, Any]:
        branch_attacks = [
            {
                "branch": branch_name,
                "attack_type": str(event.get("attack_type", "")),
                "elapsed_steps": int(event.get("elapsed_steps", 0)),
                "remaining_steps": int(event.get("remaining_steps", 0)),
            }
            for branch_name, event in sorted(self._branch_events.items())
        ]
        gps_attack = None
        if self._gps_event is not None:
            gps_attack = {
                "attack_type": str(self._gps_event.get("attack_type", "")),
                "elapsed_steps": int(self._gps_event.get("elapsed_steps", 0)),
                "remaining_steps": int(self._gps_event.get("remaining_steps", 0)),
            }
        return {
            "active": bool(branch_attacks or gps_attack is not None),
            "branch_attacks": branch_attacks,
            "gps_attack": gps_attack,
        }

    @staticmethod
    def _gps_position_valid(gps_data: Optional[Dict[str, Any]]) -> bool:
        return bool(
            isinstance(gps_data, dict)
            and gps_data.get(
                "position_valid",
                gps_data.get("fresh", gps_data.get("valid", False)),
            )
        )

    @staticmethod
    def _gps_xy_from_payload(gps_data: Optional[Dict[str, Any]]) -> np.ndarray:
        if not isinstance(gps_data, dict):
            return np.asarray([np.nan, np.nan], dtype=np.float64)
        try:
            x_val = float(gps_data.get("x", np.nan))
        except (TypeError, ValueError):
            x_val = np.nan
        try:
            y_val = float(gps_data.get("y", np.nan))
        except (TypeError, ValueError):
            y_val = np.nan
        return np.asarray([x_val, y_val], dtype=np.float64)

    def _attach_current_intensity(
        self,
        metadata: Dict[str, Any],
        *,
        orig_motor_tach: float,
        motor_tach_out: float,
        orig_steering: float,
        steering_out: float,
        orig_gyro_z: float,
        gyro_z_out: float,
        orig_accel: np.ndarray,
        accel_out: np.ndarray,
        orig_gps_data: Optional[Dict[str, Any]],
        gps_out: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        metadata_out = dict(metadata)
        branch_intensity = {
            "imu": 0.0,
            "steer": 0.0,
            "wheel": 0.0,
        }

        if any(item.get("branch") == "imu" for item in metadata.get("branch_attacks", [])):
            imu_delta = np.asarray(
                [
                    float(accel_out[0]) - float(orig_accel[0]),
                    float(accel_out[1]) - float(orig_accel[1]),
                    float(gyro_z_out) - float(orig_gyro_z),
                ],
                dtype=np.float64,
            )
            branch_intensity["imu"] = float(np.linalg.norm(imu_delta))

        if any(item.get("branch") == "steer" for item in metadata.get("branch_attacks", [])):
            branch_intensity["steer"] = float(abs(float(steering_out) - float(orig_steering)))

        if any(item.get("branch") == "wheel" for item in metadata.get("branch_attacks", [])):
            branch_intensity["wheel"] = float(abs(float(motor_tach_out) - float(orig_motor_tach)))

        orig_gps_xy = self._gps_xy_from_payload(orig_gps_data)
        attacked_gps_xy = self._gps_xy_from_payload(gps_out)
        gps_delta_xy = attacked_gps_xy - orig_gps_xy
        if not np.all(np.isfinite(gps_delta_xy)):
            gps_pos_intensity = 0.0
        else:
            gps_pos_intensity = float(np.linalg.norm(gps_delta_xy))
        gps_valid_flip = float(
            self._gps_position_valid(orig_gps_data) != self._gps_position_valid(gps_out)
        )
        gps_total_intensity = float(gps_pos_intensity + gps_valid_flip)

        metadata_out["current_intensity"] = {
            "imu": branch_intensity["imu"],
            "steer": branch_intensity["steer"],
            "wheel": branch_intensity["wheel"],
            "gps_xy": gps_pos_intensity,
            "gps_valid_flip": gps_valid_flip,
            "gps_total": gps_total_intensity,
            "overall": float(
                max(
                    branch_intensity["imu"],
                    branch_intensity["steer"],
                    branch_intensity["wheel"],
                    gps_total_intensity,
                )
            ),
        }
        return metadata_out

    def apply(
        self,
        *,
        motor_tach: float,
        steering: float,
        gyro_z: float,
        throttle: float,
        dt: float,
        acceleration: Optional[np.ndarray],
        gps_data: Optional[Dict[str, Any]],
    ) -> Tuple[float, float, float, float, np.ndarray, Optional[Dict[str, Any]], Dict[str, Any]]:
        accel = np.asarray(
            acceleration if acceleration is not None else np.zeros(3, dtype=np.float64),
            dtype=np.float64,
        ).reshape(-1)
        if accel.size < 3:
            accel_padded = np.zeros(3, dtype=np.float64)
            accel_padded[: accel.size] = accel
            accel = accel_padded
        gps_out = dict(gps_data) if isinstance(gps_data, dict) else gps_data

        if isinstance(gps_out, dict):
            gps_xy = self._extract_gps_xy(gps_out, self._last_gps_xy)
            if np.all(np.isfinite(gps_xy)):
                self._last_gps_xy = gps_xy

        orig_motor_tach = float(motor_tach)
        orig_steering = float(steering)
        orig_gyro_z = float(gyro_z)
        orig_accel = accel.astype(np.float64, copy=True)
        orig_gps_data = dict(gps_out) if isinstance(gps_out, dict) else gps_out
        motor_tach_out = float(motor_tach)
        steering_out = float(steering)
        gyro_z_out = float(gyro_z)

        if (
            not self._branch_events
            and not self._pending_immediate_attack
            and self.config.attack_prob > 0.0
            and self._branch_cooldown_remaining <= 0
            and self._rand() < float(self.config.attack_prob)
        ):
            self._start_branch_events(
                motor_tach=motor_tach_out,
                steering=steering_out,
                gyro_z=gyro_z_out,
                acceleration=accel,
            )

        if (
            self._gps_event is None
            and not self._pending_immediate_attack
            and self.config.gps_attack_prob > 0.0
            and self._gps_cooldown_remaining <= 0
            and self._rand() < float(self.config.gps_attack_prob)
        ):
            self._start_gps_event(gps_out)

        if self._pending_immediate_attack:
            if not self._branch_events and (
                self.config.attack_prob > 0.0 or self.config.forced_branches
            ):
                self._start_branch_events(
                    motor_tach=motor_tach_out,
                    steering=steering_out,
                    gyro_z=gyro_z_out,
                    acceleration=accel,
                )
            if self._gps_event is None and (
                self.config.gps_attack_prob > 0.0 or self.config.force_gps_attack
            ):
                self._start_gps_event(gps_out)
            self._pending_immediate_attack = False

        imu_event = self._branch_events.get("imu")
        if imu_event is not None:
            elapsed_steps = int(imu_event.get("elapsed_steps", 0))
            channels = dict(imu_event.get("channels", {}))
            accel[0] = self._apply_scalar_attack(float(accel[0]), channels.get("ax", {}), elapsed_steps)
            accel[1] = self._apply_scalar_attack(float(accel[1]), channels.get("ay", {}), elapsed_steps)
            gyro_z_out = self._apply_scalar_attack(gyro_z_out, channels.get("wz", {}), elapsed_steps)

        steer_event = self._branch_events.get("steer")
        if steer_event is not None:
            elapsed_steps = int(steer_event.get("elapsed_steps", 0))
            channels = dict(steer_event.get("channels", {}))
            steering_out = self._apply_scalar_attack(
                steering_out,
                channels.get("delta", {}),
                elapsed_steps,
            )

        wheel_event = self._branch_events.get("wheel")
        if wheel_event is not None:
            elapsed_steps = int(wheel_event.get("elapsed_steps", 0))
            channels = dict(wheel_event.get("channels", {}))
            motor_tach_out = self._apply_scalar_attack(
                motor_tach_out,
                channels.get("motor_tach", {}),
                elapsed_steps,
            )

        gps_out = self._apply_gps_event(gps_out, dt=float(dt))
        metadata = self._build_metadata()
        metadata = self._attach_current_intensity(
            metadata,
            orig_motor_tach=orig_motor_tach,
            motor_tach_out=motor_tach_out,
            orig_steering=orig_steering,
            steering_out=steering_out,
            orig_gyro_z=orig_gyro_z,
            gyro_z_out=gyro_z_out,
            orig_accel=orig_accel,
            accel_out=accel,
            orig_gps_data=orig_gps_data,
            gps_out=gps_out,
        )
        self._advance_events()

        return (
            float(motor_tach_out),
            float(steering_out),
            float(gyro_z_out),
            float(throttle),
            accel.astype(np.float32, copy=False),
            gps_out,
            metadata,
        )


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
