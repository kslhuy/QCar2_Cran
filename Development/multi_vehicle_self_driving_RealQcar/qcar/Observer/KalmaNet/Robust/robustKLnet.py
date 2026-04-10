from __future__ import annotations

import csv
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
import importlib
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn

    TORCH_AVAILABLE = True
except ImportError:
    torch = None
    TORCH_AVAILABLE = False

    class _TorchModuleFallback:
        def __init__(self, *args, **kwargs):
            pass

    nn = SimpleNamespace(Module=_TorchModuleFallback)


try:
    from Observer.local_state_estimators import LocalStateEstimatorBase
except ImportError:
    class LocalStateEstimatorBase(object):
        def __init__(self, initial_pose=None, logger=None):
            self.logger = logger
            self.state_dim = 4
            self.state = np.zeros(self.state_dim)
            if initial_pose is not None:
                self.state[: min(len(initial_pose), 3)] = initial_pose[:3]
            self.last_update_time = 0.0


# ============================================================
# CONFIG
# ============================================================

@dataclass
class RSNConfig:
    state_dim: int = 5          # [x, y, psi, v, w]
    meas_dim: int = 5           # [x_gnss, y_gnss, psi_imu, v_meas, w_imu]

    imu_in_dim: int = 5         # [v, psi, ax, ay, wz]
    steer_in_dim: int = 3       # [v, psi, delta]
    wheel_in_dim: int = 6       # [v, psi, vfl, vfr, vrl, vrr]

    pred_hidden: int = 32
    upd_hidden: int = 32
    gain_hidden: int = 32

    pred_mlp_hidden: int = 32
    mask_hidden: int = 32

    use_hard_mask: bool = False
    constrain_gain: bool = True
    dt: float = 0.02
    update_mask_init_bias: float = 2.0

    # ── Modular prediction step ──────────────────────────────────────────
    # "nn"        → RobustMotionPredictor (tri-LSTM, learnable, default)
    # "kinematic" → KinematicPredictor   (unicycle model, no parameters)
    predictor_mode: str = "kinematic"
    # LPF alpha for velocity in the kinematic predictor
    kin_v_lpf_alpha: float = 0.0


# ============================================================
# HELPERS
# ============================================================

def _identity_no_grad(func=None):
    if func is None:
        def decorator(inner):
            return inner
        return decorator
    return func


no_grad = torch.no_grad if TORCH_AVAILABLE else _identity_no_grad


@contextmanager
def inference_context():
    """Use the lightest available torch inference context."""
    if TORCH_AVAILABLE:
        context_factory = (
            torch.inference_mode if hasattr(torch, "inference_mode") else torch.no_grad
        )
        with context_factory():
            yield
        return
    yield


def make_H(device: torch.device, dtype=None):
    """
    Measurement model z = H x
    State x = [x, y, psi, v, w]
    Measurement z = [x, y, psi, v, w]
    Here H is identity, as in the paper's measurement vector structure.
    """
    if not TORCH_AVAILABLE:
        raise ImportError("torch is required to build Robust KalmanNet tensors")
    if dtype is None:
        dtype = torch.float32
    return torch.eye(5, device=device, dtype=dtype)



def wrap_angle(angle: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angle), torch.cos(angle))



def wrap_angle_scalar(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def wrap_state_residual(residual: torch.Tensor, angle_index: int = 2) -> torch.Tensor:
    """
    Wrap the angular component of a state/measurement residual vector without
    modifying the input tensor in-place.
    """
    return torch.cat(
        [
            residual[..., :angle_index],
            wrap_angle(residual[..., angle_index : angle_index + 1]),
            residual[..., angle_index + 1 :],
        ],
        dim=-1,
    )



def hard_sigmoid_st(mask_logits: torch.Tensor) -> torch.Tensor:
    """
    Straight-through hard mask.
    Forward: binary mask
    Backward: sigmoid gradient approximation
    """
    probs = torch.sigmoid(mask_logits)
    hard = (probs > 0.5).float()
    return hard.detach() - probs.detach() + probs


# ============================================================
# PREDICTION MODULE
# ============================================================

class SensorLSTM(nn.Module):
    def __init__(self, in_dim: int, hidden_dim: int):
        super().__init__()
        self.norm = nn.LayerNorm(in_dim)
        self.lstm = nn.LSTM(
            input_size=in_dim,
            hidden_size=hidden_dim,
            num_layers=1,
            batch_first=True,
        )

    def forward(self, x: torch.Tensor, hidden=None):
        """
        x: [B, T, in_dim]
        returns:
            y: [B, T, hidden_dim]
            hidden: LSTM hidden
        """
        x_norm = self.norm(x)
        y, hidden = self.lstm(x_norm, hidden)
        return y, hidden


class FeatureMask(nn.Module):
    def __init__(self, feat_dim: int, hidden_dim: int, use_hard_mask: bool = False):
        super().__init__()
        self.use_hard_mask = use_hard_mask
        self.net = nn.Sequential(
            nn.Linear(feat_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, feat_dim),
        )

    def forward(self, feat: torch.Tensor) -> torch.Tensor:
        """
        feat: [B, T, F]
        returns mask same shape
        """
        logits = self.net(feat)
        if self.use_hard_mask:
            return hard_sigmoid_st(logits)
        return torch.sigmoid(logits)


class MotionRegressor(nn.Module):
    def __init__(self, in_dim: int, hidden_dim: int, out_dim: int = 5):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, out_dim),
        )

    def forward(self, feat: torch.Tensor) -> torch.Tensor:
        return self.net(feat)


class RobustMotionPredictor(nn.Module):
    """
    Paper-faithful prediction structure:
    - 3 parallel LSTMs
    - concatenate outputs
    - masking
    - MLP regressor
    - convert local motion to global predicted state
    """

    def __init__(self, cfg: RSNConfig):
        super().__init__()
        self.cfg = cfg

        self.imu_lstm = SensorLSTM(cfg.imu_in_dim, cfg.pred_hidden)
        self.steer_lstm = SensorLSTM(cfg.steer_in_dim, cfg.pred_hidden)
        self.wheel_lstm = SensorLSTM(cfg.wheel_in_dim, cfg.pred_hidden)

        fusion_dim = 3 * cfg.pred_hidden
        self.mask_net = FeatureMask(fusion_dim, cfg.mask_hidden, cfg.use_hard_mask)
        self.regressor = MotionRegressor(fusion_dim, cfg.pred_mlp_hidden, out_dim=5)

    def forward(
        self,
        imu_seq: torch.Tensor,
        steer_seq: torch.Tensor,
        wheel_seq: torch.Tensor,
        prev_state_seq: torch.Tensor,
        hidden_dict: Optional[Dict] = None,
        dt: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict, torch.Tensor]:
        """
        returns:
            x_pred_seq: [B,T,5] predicted state x_{k|k-1}
            motion_seq: [B,T,5] [dxE,dyE,dpsi,v_next,w_next]
            hidden_dict
            mask: [B,T,3*H] the learned feature mask (for supervision)
        """
        if hidden_dict is None:
            hidden_dict = {}

        yI, hI = self.imu_lstm(imu_seq, hidden_dict.get("imu", None))
        yS, hS = self.steer_lstm(steer_seq, hidden_dict.get("steer", None))
        yW, hW = self.wheel_lstm(wheel_seq, hidden_dict.get("wheel", None))

        feat = torch.cat([yI, yS, yW], dim=-1)
        mask = self.mask_net(feat)
        feat_masked = feat * mask

        motion = self.regressor(feat_masked)
        x_pred = self.motion_to_state(prev_state_seq, motion)

        hidden_out = {"imu": hI, "steer": hS, "wheel": hW}
        return x_pred, motion, hidden_out, mask

    @staticmethod
    def motion_to_state(prev_state_seq: torch.Tensor, motion_seq: torch.Tensor) -> torch.Tensor:
        """
        prev_state = [x, y, psi, v, w]
        motion     = [dxE, dyE, dpsi, v_next, w_next]
        """
        x_prev = prev_state_seq[..., 0]
        y_prev = prev_state_seq[..., 1]
        psi_prev = prev_state_seq[..., 2]

        dxE = motion_seq[..., 0]
        dyE = motion_seq[..., 1]
        dpsi = motion_seq[..., 2]
        v_next = motion_seq[..., 3]
        w_next = motion_seq[..., 4]

        dxG = dxE * torch.cos(psi_prev) - dyE * torch.sin(psi_prev)
        dyG = dxE * torch.sin(psi_prev) + dyE * torch.cos(psi_prev)

        x_next = x_prev + dxG
        y_next = y_prev + dyG
        psi_next = wrap_angle(psi_prev + dpsi)

        return torch.stack([x_next, y_next, psi_next, v_next, w_next], dim=-1)


# ============================================================
# KINEMATIC PREDICTOR (parameter-free alternative)
# ============================================================

class KinematicPredictor(nn.Module):
    """
    Parameter-free unicycle-model prediction step.

    Accepts the exact same forward() signature as RobustMotionPredictor so that
    RobustStateNet.forward() requires zero structural changes.

    Kinematic equations (local body frame → global):
        dxE   =  v_prev * dt          (forward motion; no lateral slip)
        dyE   =  0
        dpsi  =  wz * dt             (yaw integration)
        v_next = alpha * motor_tach + (1 - alpha) * v_prev   (LPF)
        w_next = wz

    Signals are extracted from existing branch-input tensors:
        wz         ← imu_seq[..., 4]    layout: [v, psi, ax, ay, wz]
        motor_tach ← wheel_seq[..., 2]  layout: [v, psi, vfl, vfr, vrl, vrr]
    """

    def __init__(self, cfg: RSNConfig):
        super().__init__()
        self.dt = cfg.dt
        self.alpha = float(cfg.kin_v_lpf_alpha)

    def forward(
        self,
        imu_seq: torch.Tensor,       # [B, 1, 5]  [v, psi, ax, ay, wz]
        steer_seq: torch.Tensor,     # [B, 1, 3]  [v, psi, delta]  (unused)
        wheel_seq: torch.Tensor,     # [B, 1, 6]  [v, psi, vfl, vfr, vrl, vrr]
        prev_state_seq: torch.Tensor,  # [B, 1, 5]  x_{k-1|k-1}
        hidden_dict: Optional[Dict] = None,
        dt: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict, None]:
        """
        Returns:
            x_pred   : [B, 1, 5]
            motion   : [B, 1, 5]  [dxE, dyE, dpsi, v_next, w_next]
            {}       : empty hidden dict (stateless)
            None     : no feature mask
        """
        # ── extract signals ──────────────────────────────────────────────
        v_prev     = prev_state_seq[..., 3:4]   # [B, 1, 1]
        wz         = imu_seq[..., 4:5]          # [B, 1, 1]
        motor_tach = wheel_seq[..., 2:3]        # [B, 1, 1]  use vfl as representative

        # ── kinematic model ──────────────────────────────────────────────
        v_next = self.alpha * motor_tach + (1.0 - self.alpha) * v_prev
        w_next = wz
        active_dt = dt if dt is not None else self.dt
        dxE    = v_prev * active_dt
        dyE    = torch.zeros_like(dxE)
        dpsi   = wz * active_dt

        motion = torch.cat([dxE, dyE, dpsi, v_next, w_next], dim=-1)  # [B, 1, 5]
        x_pred = RobustMotionPredictor.motion_to_state(prev_state_seq, motion)

        return x_pred, motion, {}, None


# ============================================================
# UPDATE MODULE
# ============================================================

class LearnedKalmanUpdate(nn.Module):
    """
    Simplified Update Module aligned with the "Minimum Viable Version"
    from the RobustStateNet masking to-do list.
    
    Features:
    obs_diff       = z_k - z_{k-1}          (sz n)
    obs_innov_diff = z_k - Hx_{k|k-1}       (sz n)
    state_diff     = x_{k|k-1} - x_{k-1|k-1} (sz m)
    
    It concatenates these into a feature vector f_upd, generates an update mask
    and multiplies it with the feature vector BEFORE passing to a single GRU.
    """

    def __init__(self, cfg: RSNConfig):
        super().__init__()
        self.cfg = cfg
        self.m = cfg.state_dim
        self.n = cfg.meas_dim
        self.d_h = cfg.upd_hidden
        self.use_hard_mask = cfg.use_hard_mask
        self.constrain_gain = cfg.constrain_gain
        self.update_mask_init_bias = float(cfg.update_mask_init_bias)

        self.feat_dim_upd = self.m + self.n + self.n + 1

        # Update mask network (outputs mask of same size as features)
        self.mask_net = nn.Sequential(
            nn.Linear(self.feat_dim_upd, cfg.mask_hidden),
            nn.ReLU(),
            nn.Linear(cfg.mask_hidden, self.feat_dim_upd),
        )
        final_mask_layer = self.mask_net[-1]
        if isinstance(final_mask_layer, nn.Linear):
            nn.init.constant_(final_mask_layer.bias, self.update_mask_init_bias)

        # Single GRU for gain learning
        self.gru = nn.GRU(
            input_size=self.feat_dim_upd,
            hidden_size=self.d_h,
            batch_first=True
        )

        # Kalman Gain Output MLP
        self.gain_mlp = nn.Sequential(
            nn.Linear(self.d_h, cfg.gain_hidden),
            nn.ReLU(),
            nn.Linear(cfg.gain_hidden, self.m * self.n)
        )

    def forward(
        self,
        x_pred_seq: torch.Tensor,
        x_prev_upd_seq: torch.Tensor,
        x_prev_pred_seq: torch.Tensor,
        x_prev_prev_upd_seq: torch.Tensor,
        z_seq: torch.Tensor,
        z_prev_seq: torch.Tensor,
        gps_valid_seq: Optional[torch.Tensor] = None,
        hidden: Optional[Dict[str, torch.Tensor]] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict[str, torch.Tensor], torch.Tensor, torch.Tensor]:
        
        device = x_pred_seq.device
        B, T, _ = z_seq.shape
        H = make_H(device=device, dtype=x_pred_seq.dtype)
        Hx_pred = torch.matmul(x_pred_seq, H.T)

        # 1. Compute update features
        dx = wrap_state_residual(
            x_pred_seq - x_prev_upd_seq
        )                                        # state diff: x_{k|k-1} - x_{k-1|k-1}
        dz_p = wrap_state_residual(
            z_seq - Hx_pred
        )                                        # innovation: z_k - Hx_{k|k-1}
        dz = wrap_state_residual(
            z_seq - z_prev_seq
        )                                        # meas diff: z_k - z_{k-1}

        if gps_valid_seq is None:
            gps_valid_seq = torch.ones(B, T, 1, device=device, dtype=x_pred_seq.dtype)
        else:
            gps_valid_seq = gps_valid_seq.to(device=device, dtype=x_pred_seq.dtype)

        f_upd = torch.cat([dx, dz_p, dz, gps_valid_seq], dim=-1) # [B, T, m+n+n+1]

        # 2. Generate mask
        mask_logits = self.mask_net(f_upd)
        if self.use_hard_mask:
            m_upd = hard_sigmoid_st(mask_logits)
        else:
            m_upd = torch.sigmoid(mask_logits)

        # 3. Apply mask to features before GRU
        f_upd_masked = f_upd * m_upd

        # We return the middle portion (corresponding to dz_p -> innovation)
        # to serve as 'meas_mask' for external supervision in the training loop.
        # This aligns with the dimensions expected by `meas_attack_labels` [B, n]
        meas_mask_out = m_upd[..., self.m : self.m + self.n]
        # Also return raw logits (pre-sigmoid) for numerically stable BCE loss
        meas_mask_logits_out = mask_logits[..., self.m : self.m + self.n]

        # 4. Run GRU
        if hidden is None:
            hidden = {}
        h_k = hidden.get("gru", torch.zeros(1, B, self.d_h, device=device))
        
        # We can pass the whole sequence through the GRU at once
        out_gru, h_k_next = self.gru(f_upd_masked, h_k)

        # 5. Compute Kalman Gain
        K_flat = self.gain_mlp(out_gru) # [B, T, m*n]
        # FIX-A: Constrain gain to [-1, 1] via tanh to prevent
        # over-correction (K > 1) that causes GPS snap at test time.
        if self.constrain_gain:
            K_flat = torch.tanh(K_flat)
        K_seq = K_flat.view(B, T, self.m, self.n)

        # 6. Apply additive correction
        # x_{k|k} = x_{k|k-1} + K_k @ (z_k - Hx_{k|k-1})
        raw_innovation = wrap_state_residual(z_seq - Hx_pred)
        corr = torch.matmul(K_seq, raw_innovation.unsqueeze(-1)).squeeze(-1)

        x_upd = x_pred_seq + corr

        x_upd = torch.cat([
            x_upd[..., :2],
            wrap_angle(x_upd[..., 2:3]),
            x_upd[..., 3:],
        ], dim=-1)

        hidden_out = {"gru": h_k_next}

        return x_upd, K_seq, hidden_out, meas_mask_out, meas_mask_logits_out



# ============================================================
# FULL ROBUSTSTATENET
# ============================================================

class RobustStateNet(nn.Module):
    def __init__(self, cfg: RSNConfig):
        super().__init__()
        self.cfg = cfg
        # ── modular predictor selection ───────────────────────────────────
        if cfg.predictor_mode == "kinematic":
            self.predictor: nn.Module = KinematicPredictor(cfg)
        else:
            self.predictor = RobustMotionPredictor(cfg)
        self.updater = LearnedKalmanUpdate(cfg)

    def build_branch_inputs(
        self,
        raw: Dict[str, torch.Tensor],
        state_for_input: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        raw keys:
            ax, ay, wz, delta, vfl, vfr, vrl, vrr, gps_valid
        state_for_input = [B,T,5] containing [x,y,psi,v,w]
        Predictor inputs follow paper Eq.(2):
            IMU   = [v, psi, ax, ay, wz]
            Steer = [v, psi, delta]
            Wheel = [v, psi, vfl, vfr, vrl, vrr]
        """
        psi = state_for_input[..., 2:3]
        v = state_for_input[..., 3:4]

        imu = torch.cat([v, psi, raw["ax"], raw["ay"], raw["wz"]], dim=-1)
        steer = torch.cat([v, psi, raw["delta"]], dim=-1)
        wheel = torch.cat(
            [v, psi, raw["vfl"], raw["vfr"], raw["vrl"], raw["vrr"]],
            dim=-1,
        )
        return imu, steer, wheel

    def forward(
        self,
        raw: Dict[str, torch.Tensor],
        z_seq: torch.Tensor,
        x0: torch.Tensor,
        teacher_forcing_state: Optional[torch.Tensor] = None,
        pred_hidden: Optional[Dict] = None,
        upd_hidden=None,
        dt_seq: Optional[torch.Tensor] = None,
    ) -> Dict[str, torch.Tensor]:
        """
        teacher_forcing_state:
            if given, use it to build branch inputs, as done in predictor training in paper
            where GT v and psi are provided during motion-predictor training.
        """
        _, T, _ = z_seq.shape

        x_pred_list = []
        x_upd_list = []
        motion_list = []
        K_list = []
        pred_mask_list = []
        meas_mask_list = []
        meas_mask_logits_list = []

        x_prev_upd = x0
        x_prev_prev_upd = x0
        x_prev_pred = x0
        z_prev = z_seq[:, 0, :]

        pred_hidden_local = pred_hidden
        upd_hidden_local = upd_hidden

        for t in range(T):
            if teacher_forcing_state is not None:
                state_input_t = teacher_forcing_state[:, t, :]
            else:
                state_input_t = x_prev_upd

            state_input_t_seq = state_input_t.unsqueeze(1)
            raw_t = {k: v[:, t : t + 1, :] for k, v in raw.items()}
            imu_t, steer_t, wheel_t = self.build_branch_inputs(raw_t, state_input_t_seq)

            prev_state_seq = x_prev_upd.unsqueeze(1)
            dt_t = dt_seq[:, t : t + 1, :] if dt_seq is not None else None
            x_pred_t, motion_t, pred_hidden_local, mask_t = self.predictor(
                imu_t,
                steer_t,
                wheel_t,
                prev_state_seq,
                pred_hidden_local,
                dt=dt_t,
            )
            z_t = z_seq[:, t : t + 1, :]

            x_prev_upd_seq = x_prev_upd.unsqueeze(1)
            x_prev_pred_seq = x_prev_pred.unsqueeze(1)
            x_prev_prev_upd_seq = x_prev_prev_upd.unsqueeze(1)
            z_prev_seq = z_prev.unsqueeze(1)

            x_upd_t, K_t, upd_hidden_local, meas_mask_t, meas_mask_logits_t = self.updater(
                x_pred_t,
                x_prev_upd_seq,
                x_prev_pred_seq,
                x_prev_prev_upd_seq,
                z_t,
                z_prev_seq,
                raw_t.get("gps_valid"),
                upd_hidden_local,
            )

            x_pred_t = x_pred_t[:, 0, :]
            x_upd_t = x_upd_t[:, 0, :]
            motion_t = motion_t[:, 0, :]
            K_t = K_t[:, 0, :, :]
            # mask_t is None when using KinematicPredictor (no learned mask)
            mask_t = mask_t[:, 0, :] if mask_t is not None else None
            meas_mask_t = meas_mask_t[:, 0, :]
            meas_mask_logits_t = meas_mask_logits_t[:, 0, :]

            x_pred_list.append(x_pred_t)
            x_upd_list.append(x_upd_t)
            motion_list.append(motion_t)
            K_list.append(K_t)
            pred_mask_list.append(mask_t)
            meas_mask_list.append(meas_mask_t)
            meas_mask_logits_list.append(meas_mask_logits_t)

            x_prev_prev_upd = x_prev_upd
            x_prev_pred = x_pred_t
            x_prev_upd = x_upd_t
            z_prev = z_t[:, 0, :]

        # pred_mask is None for kinematic predictor (no learnable mask)
        pred_mask_out = (
            torch.stack(pred_mask_list, dim=1)
            if pred_mask_list and pred_mask_list[0] is not None
            else None
        )
        return {
            "x_pred": torch.stack(x_pred_list, dim=1),
            "x_upd": torch.stack(x_upd_list, dim=1),
            "motion": torch.stack(motion_list, dim=1),
            "K": torch.stack(K_list, dim=1),
            "pred_mask": pred_mask_out,
            "meas_mask": torch.stack(meas_mask_list, dim=1),
            "meas_mask_logits": torch.stack(meas_mask_logits_list, dim=1),
        }


# ============================================================
# ONLINE ESTIMATOR ADAPTER
# ============================================================

class RobustKalmanNetStateEstimator(LocalStateEstimatorBase):
    """
    Adapter that makes RobustStateNet usable inside VehicleObserver.

    The network sample is sequence-based and expects synchronized multi-sensor data.
    The production observer loop only provides one sample per tick, so this adapter:
    1. Keeps a rolling history window.
    2. Builds the network inputs expected by RobustStateNet.
    3. Loads a trained checkpoint when available.
    4. Falls back to a GPS-corrected kinematic observer when the model is not ready.

    Output state follows the current local observer contract:
        [x, y, theta, v]
    The internal model state remains 5D:
        [x, y, theta, v, yaw_rate]
    """

    RAW_KEYS = ("ax", "ay", "wz", "delta", "vfl", "vfr", "vrl", "vrr", "gps_valid")

    def __init__(
        self,
        initial_pose: Optional[np.ndarray] = None,
        config: Optional[Dict[str, Any]] = None,
        logger=None,
    ):
        super().__init__(initial_pose, logger)

        config = config or {}
        self.config = dict(config)

        self.sequence_length = max(1, int(self.config.get("sequence_length", 20)))
        self.min_history = min(
            self.sequence_length,
            max(1, int(self.config.get("min_history", 5))),
        )
        self.velocity_lpf_alpha = float(self.config.get("velocity_lpf_alpha", 0.25))
        self.gps_position_gain = float(self.config.get("gps_position_gain", 0.35))
        self.gps_heading_gain = float(self.config.get("gps_heading_gain", 0.4))
        self.yaw_rate_blend = float(self.config.get("yaw_rate_blend", 0.2))
        self.wheel_speed_scale = float(self.config.get("wheel_speed_scale", 1.0))
        self.heading_filter_enabled = bool(
            self.config.get("heading_filter_enabled", True)
        )
        self.heading_filter_q_psi = max(
            1e-9, float(self.config.get("heading_filter_q_psi", 1e-3))
        )
        self.heading_filter_q_bias = max(
            1e-9, float(self.config.get("heading_filter_q_bias", 1e-4))
        )
        self.heading_filter_r_gps = max(
            1e-9, float(self.config.get("heading_filter_r_gps", 2.5e-3))
        )
        heading_output_gain = float(self.config.get("heading_output_gain", 0.0))
        self.heading_output_gain_gps_valid = float(
            np.clip(
                float(
                    self.config.get(
                        "heading_output_gain_gps_valid", heading_output_gain
                    )
                ),
                0.0,
                1.0,
            )
        )
        self.heading_output_gain_gps_invalid = float(
            np.clip(
                float(
                    self.config.get(
                        "heading_output_gain_gps_invalid", heading_output_gain
                    )
                ),
                0.0,
                1.0,
            )
        )
        self.heading_output_max_correction = max(
            0.0, float(self.config.get("heading_output_max_correction", np.pi))
        )
        self.streaming_inference = bool(self.config.get("streaming_inference", True))
        # FIX-1: When True, skip velocity LPF / GPS re-correction on model output;
        # the learned Kalman gain already handles sensor fusion.
        self.disable_model_postprocess = bool(
            self.config.get("disable_model_postprocess", True)
        )
        self.use_model = bool(self.config.get("use_model", True))
        self.load_pretrained = bool(self.config.get("load_pretrained", False))
        self.allow_untrained_model = bool(self.config.get("allow_untrained_model", False))
        self.use_fallback = bool(self.config.get("use_fallback", True))
        self.enable_ekf_comparator = bool(self.config.get("enable_ekf_comparator", False))
        self.comparator_log_interval = max(
            1, int(self.config.get("comparator_log_interval", 25))
        )
        self.comparator_console_log = bool(
            self.config.get("comparator_console_log", False)
        )
        self.comparator_history_size = max(
            1, int(self.config.get("comparator_history_size", 200))
        )
        self.comparator_record_to_file = bool(
            self.config.get("comparator_record_to_file", True)
        )
        self.comparator_record_interval = max(
            1, int(self.config.get("comparator_record_interval", 1))
        )
        self.comparator_flush_interval = max(
            1, int(self.config.get("comparator_flush_interval", 25))
        )
        self.comparator_output_dir = self._resolve_output_dir(
            self.config.get("comparator_output_dir", "logs/comparator")
        )
        self.comparator_file_prefix = str(
            self.config.get("comparator_file_prefix", "rknet_comparison")
        ).strip() or "rknet_comparison"
        self.comparator_overwrite = bool(
            self.config.get("comparator_overwrite", False)
        )
        self.mask_branch_active_threshold = float(
            self.config.get("mask_branch_active_threshold", 0.5)
        )

        self.device = self._resolve_device(self.config.get("device", "auto"))
        self.model_cfg = RSNConfig(
            pred_hidden=int(self.config.get("pred_hidden", 32)),
            upd_hidden=int(self.config.get("upd_hidden", 32)),
            gain_hidden=int(self.config.get("gain_hidden", 32)),
            pred_mlp_hidden=int(self.config.get("pred_mlp_hidden", 32)),
            mask_hidden=int(self.config.get("mask_hidden", 32)),
            use_hard_mask=bool(self.config.get("use_hard_mask", False)),
            constrain_gain=bool(self.config.get("constrain_gain", True)),
            dt=float(self.config.get("dt", 0.02)),
            update_mask_init_bias=float(self.config.get("update_mask_init_bias", 2.0)),
            predictor_mode=str(self.config.get("predictor_mode", "kinematic")),
            kin_v_lpf_alpha=float(self.config.get("kin_v_lpf_alpha", 0.25)),
        )

        self.model = None
        if TORCH_AVAILABLE:
            self.model = RobustStateNet(self.model_cfg).to(self.device)
            self.model.eval()
            self._log_info(
                f"Robust KalmanNet predictor_mode='{self.model_cfg.predictor_mode}'"
                + (f" (kin_v_lpf_alpha={self.model_cfg.kin_v_lpf_alpha})"
                   if self.model_cfg.predictor_mode == "kinematic" else "")
            )
        self.model_path = self._resolve_model_path(self.config.get("model_path"))
        self.model_ready = False
        self.last_update_used_model = False
        self.last_model_output = None
        self.last_pred_mask = None
        self.last_pred_mask_summary = None
        self.last_meas_mask = None
        self.last_K = None
        self.last_x_pred = None
        self.update_count = 0
        self.ekf_comparator = None
        self.ekf_comparator_ready = False
        self.last_comparator_output = None
        self.last_comparison = None
        self.comparison_history = deque(maxlen=self.comparator_history_size)
        self.comparator_log_file = None
        self.comparator_log_writer = None
        self.comparator_log_path = None
        self.comparator_record_count = 0
        self._stream_pred_hidden = None
        self._stream_upd_hidden = None
        self._stream_prev_upd_state = None
        self._stream_prev_prev_upd_state = None
        self._stream_prev_pred_state = None
        self._stream_prev_measurement = None
        self._stream_initialized = False
        self._dt_mismatch_warned = False

        self.raw_history = {
            key: deque(maxlen=self.sequence_length) for key in self.RAW_KEYS
        }
        self.measurement_history = deque(maxlen=self.sequence_length)
        self.dt_history = deque(maxlen=self.sequence_length)

        self.internal_state = np.zeros(5, dtype=np.float32)
        if initial_pose is not None:
            self.internal_state[0] = float(initial_pose[0])
            self.internal_state[1] = float(initial_pose[1])
            if len(initial_pose) > 2:
                self.internal_state[2] = float(initial_pose[2])
        self.state = self.internal_state[:4].astype(np.float64, copy=True)
        self.heading_filter_state = np.zeros(2, dtype=np.float64)
        self.heading_filter_cov = np.eye(2, dtype=np.float64)
        self.heading_filter_initialized = False
        self.last_filtered_heading = wrap_angle_scalar(float(self.internal_state[2]))
        self._reset_heading_filter(initial_pose)

        self._initialize_model()
        self._initialize_ekf_comparator(initial_pose)

    @staticmethod
    def _resolve_device(device_name: str):
        if not TORCH_AVAILABLE:
            return device_name
        if device_name == "auto":
            return torch.device("cuda" if torch.cuda.is_available() else "cpu")
        return torch.device(device_name)

    @staticmethod
    def _extract_state_dict(checkpoint: Any) -> Dict[str, torch.Tensor]:
        if not TORCH_AVAILABLE:
            raise ImportError("torch is required to load Robust KalmanNet checkpoints")
        if isinstance(checkpoint, dict):
            if "model_state_dict" in checkpoint and isinstance(
                checkpoint["model_state_dict"], dict
            ):
                return checkpoint["model_state_dict"]
            if "state_dict" in checkpoint and isinstance(checkpoint["state_dict"], dict):
                return checkpoint["state_dict"]
            if checkpoint and all(torch.is_tensor(v) for v in checkpoint.values()):
                return checkpoint
        raise ValueError("Unsupported checkpoint format for Robust KalmanNet")

    def _resolve_model_path(self, configured_path: Optional[str]) -> Optional[Path]:
        if not configured_path:
            return None

        candidate = Path(configured_path)
        if candidate.is_absolute():
            return candidate

        module_dir = Path(__file__).resolve().parent
        return (module_dir / candidate).resolve()

    @staticmethod
    def _resolve_output_dir(configured_path: Optional[str]) -> Optional[Path]:
        if not configured_path:
            return None

        candidate = Path(str(configured_path))
        if candidate.is_absolute():
            return candidate

        module_dir = Path(__file__).resolve().parent
        return (module_dir / candidate).resolve()

    def _log_info(self, message: str):
        if self.logger and hasattr(self.logger, "logger"):
            self.logger.logger.info(message)

    def _log_warning(self, message: str):
        if self.logger and hasattr(self.logger, "log_warning"):
            self.logger.log_warning(message)

    def _log_error(self, message: str, exc: Optional[Exception] = None):
        if self.logger and hasattr(self.logger, "log_error"):
            self.logger.log_error(message, exc)

    def _reset_heading_filter(self, initial_pose: Optional[np.ndarray] = None) -> None:
        psi0 = float(self.internal_state[2]) if self.internal_state.size > 2 else 0.0
        if initial_pose is not None and len(initial_pose) > 2:
            psi0 = float(initial_pose[2])
        self.heading_filter_state = np.array(
            [wrap_angle_scalar(psi0), 0.0], dtype=np.float64
        )
        self.heading_filter_cov = np.diag([0.5, 0.1]).astype(np.float64)
        self.heading_filter_initialized = bool(self.heading_filter_enabled)
        self.last_filtered_heading = wrap_angle_scalar(psi0)

    def _ensure_heading_filter_initialized(
        self, gps_data: Optional[Dict[str, Any]] = None
    ) -> None:
        if self.heading_filter_initialized:
            return

        psi0 = float(self.internal_state[2]) if self.internal_state.size > 2 else 0.0
        if gps_data is not None and gps_data.get("valid", False):
            psi0 = float(gps_data.get("theta", psi0))
        self.heading_filter_state = np.array(
            [wrap_angle_scalar(psi0), 0.0], dtype=np.float64
        )
        self.heading_filter_cov = np.diag([0.5, 0.1]).astype(np.float64)
        self.heading_filter_initialized = True
        self.last_filtered_heading = wrap_angle_scalar(psi0)

    def _update_heading_filter(
        self,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]] = None,
    ) -> float:
        if not self.heading_filter_enabled:
            if gps_data is not None and gps_data.get("valid", False):
                self.last_filtered_heading = wrap_angle_scalar(
                    float(gps_data.get("theta", self.internal_state[2]))
                )
            else:
                self.last_filtered_heading = wrap_angle_scalar(
                    float(self.internal_state[2]) + float(gyro_z) * float(dt)
                )
            return self.last_filtered_heading

        self._ensure_heading_filter_initialized(gps_data)

        dt = max(float(dt), 1e-3)
        A = np.array([[1.0, -dt], [0.0, 1.0]], dtype=np.float64)
        B = np.array([dt, 0.0], dtype=np.float64)
        x_pred = A @ self.heading_filter_state + B * float(gyro_z)
        x_pred[0] = wrap_angle_scalar(float(x_pred[0]))

        Q = np.diag(
            [
                self.heading_filter_q_psi * dt,
                self.heading_filter_q_bias * dt,
            ]
        ).astype(np.float64)
        P_pred = A @ self.heading_filter_cov @ A.T + Q

        if gps_data is not None and gps_data.get("valid", False):
            H = np.array([1.0, 0.0], dtype=np.float64)
            innovation = wrap_angle_scalar(
                float(gps_data.get("theta", x_pred[0])) - float(x_pred[0])
            )
            S = float(H @ P_pred @ H.T + self.heading_filter_r_gps)
            if np.isfinite(S) and S > 1e-9:
                K = (P_pred @ H) / S
                x_pred = x_pred + K * innovation
                x_pred[0] = wrap_angle_scalar(float(x_pred[0]))
                I = np.eye(2, dtype=np.float64)
                KH = np.outer(K, H)
                P_pred = (
                    (I - KH) @ P_pred @ (I - KH).T
                    + np.outer(K, K) * self.heading_filter_r_gps
                )

        self.heading_filter_state = x_pred
        self.heading_filter_cov = P_pred
        self.last_filtered_heading = wrap_angle_scalar(float(x_pred[0]))
        return self.last_filtered_heading

    def _apply_heading_filter_output(
        self,
        estimate: np.ndarray,
        gps_data: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        estimate = np.asarray(estimate, dtype=np.float64).copy()
        estimate[2] = wrap_angle_scalar(float(estimate[2]))
        gps_valid = bool(gps_data is not None and gps_data.get("valid", False))
        heading_gain = (
            self.heading_output_gain_gps_valid
            if gps_valid
            else self.heading_output_gain_gps_invalid
        )
        if heading_gain > 0.0:
            heading_residual = wrap_angle_scalar(
                float(self.last_filtered_heading) - float(estimate[2])
            )
            max_correction = float(self.heading_output_max_correction)
            if max_correction > 0.0:
                heading_residual = float(
                    np.clip(heading_residual, -max_correction, max_correction)
                )
            estimate[2] = wrap_angle_scalar(
                float(estimate[2]) + heading_gain * heading_residual
            )
        return estimate

    def _initialize_model(self) -> None:
        if not TORCH_AVAILABLE:
            if self.use_model:
                self._log_warning(
                    "torch is not available; Robust KalmanNet will stay on kinematic fallback"
                )
            return

        if not self.use_model:
            self._log_info("Robust KalmanNet configured in fallback-only mode")
            return

        if self.load_pretrained and self.model_path is not None and self.model_path.exists():
            try:
                checkpoint = torch.load(self.model_path, map_location=self.device)
                state_dict = self._extract_state_dict(checkpoint)
                self.model.load_state_dict(state_dict, strict=False)
                self.model.eval()
                self.model_ready = True
                self._log_info(
                    f"Robust KalmanNet checkpoint loaded from {self.model_path}"
                )
                return
            except Exception as exc:
                self._log_error("Failed to load Robust KalmanNet checkpoint", exc)

        if self.allow_untrained_model:
            self.model_ready = True
            self._log_warning(
                "Robust KalmanNet is running with randomly initialized weights"
            )
            return

        if self.load_pretrained:
            self._log_warning(
                "Robust KalmanNet checkpoint not available; using kinematic fallback"
            )
        else:
            self._log_info(
                "Robust KalmanNet instantiated without pretrained weights; using fallback until a checkpoint is configured"
            )

    def _build_ekf_comparator_config(self) -> Dict[str, Any]:
        comparator_cfg = dict(self.config.get("ekf_comparator_config", {}))
        comparator_cfg.setdefault(
            "use_qcar_ekf",
            bool(self.config.get("comparator_use_qcar_ekf", True)),
        )
        if "comparator_v_lpf_alpha" in self.config:
            comparator_cfg.setdefault(
                "v_lpf_alpha", float(self.config["comparator_v_lpf_alpha"])
            )
        return comparator_cfg

    def _initialize_ekf_comparator(
        self, initial_pose: Optional[np.ndarray] = None
    ) -> None:
        if not self.enable_ekf_comparator:
            return

        try:
            module = importlib.import_module("Observer.local_state_estimators")
            comparator_cls = getattr(module, "EKFStateEstimator")
            comparator_cfg = self._build_ekf_comparator_config()
            self.ekf_comparator = comparator_cls(
                initial_pose=initial_pose,
                config=comparator_cfg,
                logger=self.logger,
            )
            self.ekf_comparator_ready = True
            self._initialize_comparator_log_file()
            self._log_info(
                "[RKNetComparator] EKF comparator enabled"
            )
        except Exception as exc:
            self.ekf_comparator = None
            self.ekf_comparator_ready = False
            self._close_comparator_log_file()
            self._log_error(
                "Failed to initialize EKF comparator for Robust KalmanNet",
                exc,
            )

    def _initialize_comparator_log_file(self) -> None:
        if not self.comparator_record_to_file or self.comparator_output_dir is None:
            return
        if self.comparator_log_file is not None:
            return

        try:
            self.comparator_output_dir.mkdir(parents=True, exist_ok=True)
            if self.comparator_overwrite:
                filepath = self.comparator_output_dir / f"{self.comparator_file_prefix}.csv"
            else:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filepath = self.comparator_output_dir / f"{self.comparator_file_prefix}_{timestamp}.csv"

            self.comparator_log_file = open(filepath, "w", newline="", buffering=8192)
            self.comparator_log_writer = csv.DictWriter(
                self.comparator_log_file,
                fieldnames=[
                    "timestamp",
                    "tick",
                    "source",
                    "dt",
                    "motor_tach",
                    "steering",
                    "throttle",
                    "gyro_z",
                    "gps_valid",
                    "gps_x",
                    "gps_y",
                    "gps_theta",
                    "accel_x",
                    "accel_y",
                    "accel_z",
                    "robust_x",
                    "robust_y",
                    "robust_theta",
                    "robust_v",
                    "ekf_x",
                    "ekf_y",
                    "ekf_theta",
                    "ekf_v",
                    "delta_x",
                    "delta_y",
                    "delta_theta",
                    "delta_v",
                    "position_error_norm",
                    "heading_error",
                    "velocity_error",
                    "pred_mask_mean",
                    "pred_mask_min",
                    "pred_mask_max",
                    "mask_imu_mean",
                    "mask_steer_mean",
                    "mask_wheel_mean",
                    "mask_selected_branch",
                    "mask_selected_score",
                    "mask_imu_active",
                    "mask_steer_active",
                    "mask_wheel_active",
                    "meas_mask_x",
                    "meas_mask_y",
                    "meas_mask_psi",
                    "meas_mask_v",
                    "meas_mask_w",
                    "K_norm",
                    "K_x_x",
                    "K_y_y",
                    "K_psi_psi",
                    "K_v_v",
                    "K_w_w",
                    "innov_x",
                    "innov_y",
                    "innov_psi",
                    "innov_v",
                    "innov_w",
                    "pred_x",
                    "pred_y",
                ],
            )
            self.comparator_log_writer.writeheader()
            self.comparator_log_path = filepath
            self.comparator_record_count = 0
            self._log_info(f"[RKNetComparator] Recording comparisons to {filepath}")
        except Exception as exc:
            self._close_comparator_log_file()
            self._log_error("Failed to open RKNet comparator log file", exc)

    def _close_comparator_log_file(self) -> None:
        if self.comparator_log_file is not None:
            try:
                self.comparator_log_file.flush()
                self.comparator_log_file.close()
            except Exception:
                pass
        self.comparator_log_file = None
        self.comparator_log_writer = None
        self.comparator_record_count = 0

    def _record_comparison_to_file(
        self,
        comparison: Dict[str, Any],
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
        acceleration: Optional[np.ndarray],
    ) -> None:
        if not self.comparator_record_to_file:
            return
        if self.comparator_log_writer is None:
            return
        if self.update_count % self.comparator_record_interval != 0:
            return

        accel = np.asarray(
            acceleration if acceleration is not None else np.zeros(3, dtype=np.float64),
            dtype=np.float64,
        ).reshape(-1)
        gps_valid = bool(gps_data and gps_data.get("valid", True))
        robust_state = comparison["robust_state"]
        ekf_state = comparison["ekf_state"]
        delta_state = comparison["delta_state"]

        try:
            self.comparator_log_writer.writerow(
                {
                    "timestamp": comparison["timestamp"],
                    "tick": comparison["tick"],
                    "source": comparison["robust_source"],
                    "dt": float(dt),
                    "motor_tach": float(motor_tach),
                    "steering": float(steering),
                    "throttle": float(throttle),
                    "gyro_z": float(gyro_z),
                    "gps_valid": int(gps_valid),
                    "gps_x": float(gps_data.get("x", 0.0)) if gps_data else 0.0,
                    "gps_y": float(gps_data.get("y", 0.0)) if gps_data else 0.0,
                    "gps_theta": float(gps_data.get("theta", 0.0)) if gps_data else 0.0,
                    "accel_x": float(accel[0]) if accel.size > 0 else 0.0,
                    "accel_y": float(accel[1]) if accel.size > 1 else 0.0,
                    "accel_z": float(accel[2]) if accel.size > 2 else 0.0,
                    "robust_x": float(robust_state[0]),
                    "robust_y": float(robust_state[1]),
                    "robust_theta": float(robust_state[2]),
                    "robust_v": float(robust_state[3]),
                    "ekf_x": float(ekf_state[0]),
                    "ekf_y": float(ekf_state[1]),
                    "ekf_theta": float(ekf_state[2]),
                    "ekf_v": float(ekf_state[3]),
                    "delta_x": float(delta_state[0]),
                    "delta_y": float(delta_state[1]),
                    "delta_theta": float(delta_state[2]),
                    "delta_v": float(delta_state[3]),
                    "position_error_norm": float(comparison["position_error_norm"]),
                    "heading_error": float(comparison["heading_error"]),
                    "velocity_error": float(comparison["velocity_error"]),
                    "pred_mask_mean": float(comparison.get("pred_mask_mean", np.nan)),
                    "pred_mask_min": float(comparison.get("pred_mask_min", np.nan)),
                    "pred_mask_max": float(comparison.get("pred_mask_max", np.nan)),
                    "mask_imu_mean": float(comparison.get("mask_imu_mean", np.nan)),
                    "mask_steer_mean": float(comparison.get("mask_steer_mean", np.nan)),
                    "mask_wheel_mean": float(comparison.get("mask_wheel_mean", np.nan)),
                    "mask_selected_branch": str(comparison.get("mask_selected_branch", "")),
                    "mask_selected_score": float(comparison.get("mask_selected_score", np.nan)),
                    "mask_imu_active": int(comparison.get("mask_imu_active", 0)),
                    "mask_steer_active": int(comparison.get("mask_steer_active", 0)),
                    "mask_wheel_active": int(comparison.get("mask_wheel_active", 0)),
                    "meas_mask_x": float(comparison.get("meas_mask_x", np.nan)),
                    "meas_mask_y": float(comparison.get("meas_mask_y", np.nan)),
                    "meas_mask_psi": float(comparison.get("meas_mask_psi", np.nan)),
                    "meas_mask_v": float(comparison.get("meas_mask_v", np.nan)),
                    "meas_mask_w": float(comparison.get("meas_mask_w", np.nan)),
                    "K_norm": float(comparison.get("K_norm", np.nan)),
                    "K_x_x": float(comparison.get("K_x_x", np.nan)),
                    "K_y_y": float(comparison.get("K_y_y", np.nan)),
                    "K_psi_psi": float(comparison.get("K_psi_psi", np.nan)),
                    "K_v_v": float(comparison.get("K_v_v", np.nan)),
                    "K_w_w": float(comparison.get("K_w_w", np.nan)),
                    "innov_x": float(comparison.get("innov_x", np.nan)),
                    "innov_y": float(comparison.get("innov_y", np.nan)),
                    "innov_psi": float(comparison.get("innov_psi", np.nan)),
                    "innov_v": float(comparison.get("innov_v", np.nan)),
                    "innov_w": float(comparison.get("innov_w", np.nan)),
                    "pred_x": float(comparison.get("pred_x", np.nan)),
                    "pred_y": float(comparison.get("pred_y", np.nan)),
                }
            )
            self.comparator_record_count += 1
            if self.comparator_record_count % self.comparator_flush_interval == 0:
                self.comparator_log_file.flush()
        except Exception as exc:
            self._log_error("Failed to write RKNet comparator log row", exc)

    def _append_sample(self, raw_sample: Dict[str, float], measurement: np.ndarray, dt: float) -> None:
        for key, value in raw_sample.items():
            self.raw_history[key].append(float(value))
        self.measurement_history.append(np.asarray(measurement, dtype=np.float32))
        self.dt_history.append(float(dt))

    def _clear_stream_state(self) -> None:
        self._stream_pred_hidden = None
        self._stream_upd_hidden = None
        self._stream_prev_upd_state = None
        self._stream_prev_prev_upd_state = None
        self._stream_prev_pred_state = None
        self._stream_prev_measurement = None
        self._stream_initialized = False

    @staticmethod
    def _detach_hidden_state(hidden: Any) -> Any:
        if not TORCH_AVAILABLE or hidden is None:
            return hidden
        if torch.is_tensor(hidden):
            return hidden.detach()
        if isinstance(hidden, tuple):
            return tuple(RobustKalmanNetStateEstimator._detach_hidden_state(v) for v in hidden)
        if isinstance(hidden, list):
            return [RobustKalmanNetStateEstimator._detach_hidden_state(v) for v in hidden]
        if isinstance(hidden, dict):
            return {
                key: RobustKalmanNetStateEstimator._detach_hidden_state(value)
                for key, value in hidden.items()
            }
        return hidden

    def _run_incremental_model_step(
        self,
        raw_sample: Dict[str, float],
        measurement: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        if not TORCH_AVAILABLE or self.model is None:
            raise RuntimeError("Robust KalmanNet model is not available for streaming inference")

        if self._stream_prev_upd_state is None:
            raise RuntimeError("Robust KalmanNet streaming state is not initialized")

        dtype = torch.float32
        measurement_arr = np.asarray(measurement, dtype=np.float32).reshape(1, 1, 5)
        dt_arr = np.asarray([[[float(dt)]]], dtype=np.float32)

        raw_t = {
            key: torch.as_tensor(
                np.asarray([[[float(raw_sample[key])]]], dtype=np.float32),
                device=self.device,
                dtype=dtype,
            )
            for key in self.RAW_KEYS
        }
        z_t = torch.as_tensor(measurement_arr, device=self.device, dtype=dtype)
        dt_t = torch.as_tensor(dt_arr, device=self.device, dtype=dtype)
        prev_state_seq = torch.as_tensor(
            self._stream_prev_upd_state.reshape(1, 1, 5),
            device=self.device,
            dtype=dtype,
        )
        prev_pred_seq = torch.as_tensor(
            self._stream_prev_pred_state.reshape(1, 1, 5),
            device=self.device,
            dtype=dtype,
        )
        prev_prev_upd_seq = torch.as_tensor(
            self._stream_prev_prev_upd_state.reshape(1, 1, 5),
            device=self.device,
            dtype=dtype,
        )
        z_prev_seq = torch.as_tensor(
            self._stream_prev_measurement.reshape(1, 1, 5),
            device=self.device,
            dtype=dtype,
        )

        imu_t, steer_t, wheel_t = self.model.build_branch_inputs(raw_t, prev_state_seq)
        x_pred_t, _, pred_hidden, pred_mask_t = self.model.predictor(
            imu_t,
            steer_t,
            wheel_t,
            prev_state_seq,
            self._stream_pred_hidden,
            dt=dt_t,
        )
        x_upd_t, K_t, upd_hidden, meas_mask_t, _ = self.model.updater(
            x_pred_t,
            prev_state_seq,
            prev_pred_seq,
            prev_prev_upd_seq,
            z_t,
            z_prev_seq,
            raw_t.get("gps_valid"),
            self._stream_upd_hidden,
        )

        x_pred_np = x_pred_t[0, 0].detach().cpu().numpy().astype(np.float64)
        x_upd_np = x_upd_t[0, 0].detach().cpu().numpy().astype(np.float64)
        if not np.all(np.isfinite(x_upd_np)):
            raise ValueError("Robust KalmanNet produced non-finite state")

        self.last_x_pred = x_pred_np
        if pred_mask_t is not None:
            pred_mask_np = pred_mask_t[0, 0].detach().cpu().numpy().astype(np.float64)
            self.last_pred_mask = pred_mask_np.copy()
            self.last_pred_mask_summary = self._summarize_pred_mask(pred_mask_np)
        else:
            self.last_pred_mask = None
            self.last_pred_mask_summary = None

        self.last_meas_mask = meas_mask_t[0, 0].detach().cpu().numpy().astype(np.float64)
        self.last_K = K_t[0, 0].detach().cpu().numpy().astype(np.float64)

        prev_upd_state = self._stream_prev_upd_state.copy()
        self._stream_prev_prev_upd_state = prev_upd_state
        self._stream_prev_pred_state = x_pred_np.astype(np.float32, copy=False)
        self._stream_prev_upd_state = x_upd_np.astype(np.float32, copy=False)
        self._stream_prev_measurement = np.asarray(measurement, dtype=np.float32).copy()
        self._stream_pred_hidden = self._detach_hidden_state(pred_hidden)
        self._stream_upd_hidden = self._detach_hidden_state(upd_hidden)
        self._stream_initialized = True

        x_upd_np[2] = wrap_angle_scalar(float(x_upd_np[2]))
        self.last_model_output = x_upd_np.copy()
        return x_upd_np

    def _prime_stream_from_history(self) -> Optional[np.ndarray]:
        history_len = len(self.measurement_history)
        if history_len < self.min_history:
            return None

        raw_history = {key: list(self.raw_history[key]) for key in self.RAW_KEYS}
        measurement_history = [
            np.asarray(sample, dtype=np.float32).copy() for sample in self.measurement_history
        ]
        dt_history = list(self.dt_history)
        if not dt_history:
            dt_history = [self.model_cfg.dt] * history_len

        self._clear_stream_state()
        initial_state = measurement_history[0].astype(np.float32, copy=True)
        self._stream_prev_upd_state = initial_state.copy()
        self._stream_prev_prev_upd_state = initial_state.copy()
        self._stream_prev_pred_state = initial_state.copy()
        self._stream_prev_measurement = measurement_history[0].copy()

        estimate = None
        for idx in range(history_len):
            raw_sample = {key: float(raw_history[key][idx]) for key in self.RAW_KEYS}
            dt_value = float(dt_history[idx]) if idx < len(dt_history) else float(self.model_cfg.dt)
            estimate = self._run_incremental_model_step(
                raw_sample=raw_sample,
                measurement=measurement_history[idx],
                dt=dt_value,
            )
        return estimate

    def _build_model_inputs(self) -> Tuple[Dict[str, torch.Tensor], torch.Tensor, torch.Tensor, torch.Tensor]:
        if not TORCH_AVAILABLE:
            raise ImportError("torch is required for Robust KalmanNet inference")
        history_len = len(self.measurement_history)
        if history_len == 0:
            raise ValueError("No measurement history available for Robust KalmanNet")

        pad_count = max(0, self.sequence_length - history_len)

        raw_tensors: Dict[str, torch.Tensor] = {}
        for key in self.RAW_KEYS:
            values = list(self.raw_history[key])
            if not values:
                values = [0.0]
            padded = [values[0]] * pad_count + values
            arr = np.asarray(padded[-self.sequence_length :], dtype=np.float32).reshape(1, -1, 1)
            raw_tensors[key] = torch.as_tensor(arr, device=self.device)

        measurements = list(self.measurement_history)
        padded_measurements = [measurements[0]] * pad_count + measurements
        z_arr = np.stack(padded_measurements[-self.sequence_length :], axis=0).astype(np.float32)
        z_seq = torch.as_tensor(z_arr.reshape(1, self.sequence_length, 5), device=self.device)
        x0 = torch.as_tensor(z_arr[0].reshape(1, 5), device=self.device)

        dts = list(self.dt_history)
        if not dts:
            dts = [self.model_cfg.dt]
        padded_dts = [dts[0]] * pad_count + dts
        dt_arr = np.asarray(padded_dts[-self.sequence_length :], dtype=np.float32).reshape(1, self.sequence_length, 1)
        dt_seq = torch.as_tensor(dt_arr, device=self.device)

        return raw_tensors, z_seq, x0, dt_seq

    def _summarize_pred_mask(self, pred_mask: np.ndarray) -> Dict[str, Any]:
        mask = np.asarray(pred_mask, dtype=np.float64).reshape(-1)
        branch_dim = int(self.model_cfg.pred_hidden)
        if branch_dim <= 0 or mask.size < 3 * branch_dim:
            return {
                "pred_mask_mean": float(np.nanmean(mask)) if mask.size else float("nan"),
                "pred_mask_min": float(np.nanmin(mask)) if mask.size else float("nan"),
                "pred_mask_max": float(np.nanmax(mask)) if mask.size else float("nan"),
                "mask_imu_mean": float("nan"),
                "mask_steer_mean": float("nan"),
                "mask_wheel_mean": float("nan"),
                "mask_selected_branch": "unknown",
                "mask_selected_score": float("nan"),
                "mask_imu_active": 0,
                "mask_steer_active": 0,
                "mask_wheel_active": 0,
            }

        imu_mask = mask[0:branch_dim]
        steer_mask = mask[branch_dim : 2 * branch_dim]
        wheel_mask = mask[2 * branch_dim : 3 * branch_dim]
        branch_means = {
            "imu": float(np.mean(imu_mask)),
            "steer": float(np.mean(steer_mask)),
            "wheel": float(np.mean(wheel_mask)),
        }
        selected_branch = max(branch_means, key=branch_means.get)
        threshold = float(self.mask_branch_active_threshold)
        return {
            "pred_mask_mean": float(np.mean(mask)),
            "pred_mask_min": float(np.min(mask)),
            "pred_mask_max": float(np.max(mask)),
            "mask_imu_mean": branch_means["imu"],
            "mask_steer_mean": branch_means["steer"],
            "mask_wheel_mean": branch_means["wheel"],
            "mask_selected_branch": selected_branch,
            "mask_selected_score": float(branch_means[selected_branch]),
            "mask_imu_active": int(branch_means["imu"] >= threshold),
            "mask_steer_active": int(branch_means["steer"] >= threshold),
            "mask_wheel_active": int(branch_means["wheel"] >= threshold),
        }

    def _predict_with_model(self) -> Optional[np.ndarray]:
        if (
            not TORCH_AVAILABLE
            or self.model is None
            or not self.model_ready
            or len(self.measurement_history) < self.min_history
        ):
            return None

        latest_dt = float(self.dt_history[-1]) if self.dt_history else float(self.model_cfg.dt)
        if (
            not self._dt_mismatch_warned
            and self.model_cfg.dt > 0.0
            and abs(latest_dt - self.model_cfg.dt) / self.model_cfg.dt > 0.5
        ):
            self._dt_mismatch_warned = True
            self._log_warning(
                "Robust KalmanNet runtime dt differs significantly from training dt "
                f"(runtime={latest_dt:.4f}s, model={self.model_cfg.dt:.4f}s)"
            )

        if self.streaming_inference:
            with inference_context():
                if not self._stream_initialized:
                    estimate = self._prime_stream_from_history()
                    if estimate is not None:
                        return estimate

                raw_sample = {key: float(self.raw_history[key][-1]) for key in self.RAW_KEYS}
                measurement = np.asarray(self.measurement_history[-1], dtype=np.float32)
                return self._run_incremental_model_step(
                    raw_sample=raw_sample,
                    measurement=measurement,
                    dt=latest_dt,
                )

        raw, z_seq, x0, dt_seq = self._build_model_inputs()
        with inference_context():
            out = self.model(raw=raw, z_seq=z_seq, x0=x0, teacher_forcing_state=None, dt_seq=dt_seq)

        x_upd = out["x_upd"][0, -1].detach().cpu().numpy().astype(np.float64)
        if not np.all(np.isfinite(x_upd)):
            raise ValueError("Robust KalmanNet produced non-finite state")

        self.last_x_pred = out["x_pred"][0, -1].detach().cpu().numpy().astype(np.float64)

        pred_mask = out.get("pred_mask")
        self.last_pred_mask = None
        self.last_pred_mask_summary = None
        if pred_mask is not None:
            pred_mask_np = pred_mask[0, -1].detach().cpu().numpy().astype(np.float64)
            self.last_pred_mask = pred_mask_np.copy()
            self.last_pred_mask_summary = self._summarize_pred_mask(pred_mask_np)

        meas_mask_tensor = out.get("meas_mask")
        if meas_mask_tensor is not None:
            self.last_meas_mask = meas_mask_tensor[0, -1].detach().cpu().numpy().astype(np.float64)
        else:
            self.last_meas_mask = np.full(5, np.nan)

        K_tensor = out.get("K")
        if K_tensor is not None:
            self.last_K = K_tensor[0, -1].detach().cpu().numpy().astype(np.float64)
        else:
            self.last_K = np.full((5, 5), np.nan)

        x_upd[2] = wrap_angle_scalar(float(x_upd[2]))
        self.last_model_output = x_upd.copy()
        return x_upd

    def _measurement_from_inputs(
        self,
        motor_tach: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
        dt: float = 0.02,
    ) -> np.ndarray:
        # FIX-2: When GPS is invalid, use a kinematic dead-reckoning prediction
        # for position channels instead of feeding back internal_state (which
        # creates near-zero innovation and makes the update step useless).
        gps_valid = bool(gps_data is not None and gps_data.get("valid", False))
        self.last_gps_valid = gps_valid
        heading_meas = self._update_heading_filter(dt, gyro_z, gps_data)

        if gps_valid:
            # GPS available → use GPS measurements directly
            measurement = np.array([
                float(gps_data.get("x", self.internal_state[0])),
                float(gps_data.get("y", self.internal_state[1])),
                heading_meas,
                float(motor_tach),
                float(gyro_z),
            ], dtype=np.float32)
        else:
            # GPS unavailable → kinematic dead-reckoning prediction for position
            # so that the innovation (z - Hx_pred) is NOT trivially zero.
            x, y, theta, v, _ = self.internal_state.astype(np.float64)
            theta_ref = heading_meas if self.heading_filter_enabled else theta
            x_dr = x + v * np.cos(theta_ref) * dt
            y_dr = y + v * np.sin(theta_ref) * dt
            measurement = np.array([
                x_dr,
                y_dr,
                heading_meas,
                float(motor_tach),
                float(gyro_z),
            ], dtype=np.float32)

        return measurement

    def _raw_sample_from_inputs(
        self,
        motor_tach: float,
        steering: float,
        gyro_z: float,
        acceleration: Optional[np.ndarray],
        gps_data: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, float]:
        accel = np.asarray(acceleration if acceleration is not None else np.zeros(3), dtype=np.float32)
        wheel_speed = float(motor_tach) * self.wheel_speed_scale
        gps_valid = float(bool(gps_data is not None and gps_data.get("valid", False)))
        return {
            "ax": float(accel[0]) if accel.size > 0 else 0.0,
            "ay": float(accel[1]) if accel.size > 1 else 0.0,
            "wz": float(gyro_z),
            "delta": float(steering),
            "vfl": wheel_speed,
            "vfr": wheel_speed,
            "vrl": wheel_speed,
            "vrr": wheel_speed,
            "gps_valid": gps_valid,
        }

    def _post_process_estimate(
        self,
        estimate: np.ndarray,
        motor_tach: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
    ) -> np.ndarray:
        estimate = np.asarray(estimate, dtype=np.float64).copy()
        estimate[2] = wrap_angle_scalar(float(estimate[2]))
        estimate[3] = (
            self.velocity_lpf_alpha * float(motor_tach)
            + (1.0 - self.velocity_lpf_alpha) * float(estimate[3])
        )
        estimate[4] = (
            self.yaw_rate_blend * float(gyro_z)
            + (1.0 - self.yaw_rate_blend) * float(estimate[4])
        )

        if gps_data is not None and gps_data.get("valid", False):
            estimate[0] = (1.0 - self.gps_position_gain) * estimate[0] + self.gps_position_gain * float(gps_data["x"])
            estimate[1] = (1.0 - self.gps_position_gain) * estimate[1] + self.gps_position_gain * float(gps_data["y"])
            if not self.heading_filter_enabled:
                heading_residual = wrap_angle_scalar(
                    float(gps_data["theta"]) - float(estimate[2])
                )
                estimate[2] = wrap_angle_scalar(
                    float(estimate[2]) + self.gps_heading_gain * heading_residual
                )

        return self._apply_heading_filter_output(estimate, gps_data=gps_data)

    def _fallback_update(
        self,
        motor_tach: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
    ) -> np.ndarray:
        x, y, theta, v, _ = self.internal_state.astype(np.float64)
        v = self.velocity_lpf_alpha * float(motor_tach) + (1.0 - self.velocity_lpf_alpha) * v
        theta_pred = (
            float(self.last_filtered_heading)
            if self.heading_filter_enabled
            else wrap_angle_scalar(theta + float(gyro_z) * dt)
        )
        x_pred = x + v * np.cos(theta_pred) * dt
        y_pred = y + v * np.sin(theta_pred) * dt

        estimate = np.array([x_pred, y_pred, theta_pred, v, float(gyro_z)], dtype=np.float64)
        return self._post_process_estimate(estimate, motor_tach, gyro_z, gps_data)

    def _record_comparison(
        self,
        robust_estimate: np.ndarray,
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
        acceleration: Optional[np.ndarray],
        measurement: Optional[np.ndarray] = None,
    ) -> None:
        if not self.ekf_comparator_ready or self.ekf_comparator is None:
            return

        ok = self.ekf_comparator.update(
            motor_tach=motor_tach,
            steering=steering,
            throttle=throttle,
            dt=dt,
            gyro_z=gyro_z,
            gps_data=gps_data,
            acceleration=acceleration,
        )
        if not ok:
            self._log_warning("[RKNetComparator] EKF comparator update failed")
            return

        ekf_state = np.asarray(self.ekf_comparator.get_state(), dtype=np.float64)
        robust_state = np.asarray(robust_estimate[:4], dtype=np.float64)
        delta_state = robust_state - ekf_state
        delta_state[2] = wrap_angle_scalar(float(delta_state[2]))

        if self.last_update_used_model and self.last_x_pred is not None and measurement is not None:
            innovation = measurement - self.last_x_pred
        else:
            innovation = np.full(5, np.nan)

        comparison = {
            "tick": self.update_count,
            "timestamp": time.time(),
            "robust_source": "model" if self.last_update_used_model else "fallback",
            "robust_state": robust_state.copy(),
            "ekf_state": ekf_state.copy(),
            "delta_state": delta_state.copy(),
            "position_error_norm": float(np.linalg.norm(delta_state[:2])),
            "heading_error": float(delta_state[2]),
            "velocity_error": float(delta_state[3]),
            "meas_mask_x": self.last_meas_mask[0] if self.last_meas_mask is not None else np.nan,
            "meas_mask_y": self.last_meas_mask[1] if self.last_meas_mask is not None else np.nan,
            "meas_mask_psi": self.last_meas_mask[2] if self.last_meas_mask is not None else np.nan,
            "meas_mask_v": self.last_meas_mask[3] if self.last_meas_mask is not None else np.nan,
            "meas_mask_w": self.last_meas_mask[4] if self.last_meas_mask is not None else np.nan,
            "K_norm": float(np.linalg.norm(self.last_K)) if self.last_K is not None else np.nan,
            "K_x_x": self.last_K[0, 0] if self.last_K is not None else np.nan,
            "K_y_y": self.last_K[1, 1] if self.last_K is not None else np.nan,
            "K_psi_psi": self.last_K[2, 2] if self.last_K is not None else np.nan,
            "K_v_v": self.last_K[3, 3] if self.last_K is not None else np.nan,
            "K_w_w": self.last_K[4, 4] if self.last_K is not None else np.nan,
            "innov_x": innovation[0],
            "innov_y": innovation[1],
            "innov_psi": innovation[2],
            "innov_v": innovation[3],
            "innov_w": innovation[4],
            "pred_x": self.last_x_pred[0] if self.last_x_pred is not None else np.nan,
            "pred_y": self.last_x_pred[1] if self.last_x_pred is not None else np.nan,
        }
        if self.last_pred_mask_summary is not None:
            comparison.update(self.last_pred_mask_summary)
        self.last_comparator_output = ekf_state.copy()
        self.last_comparison = comparison
        self.comparison_history.append(comparison)
        self._record_comparison_to_file(
            comparison=comparison,
            motor_tach=motor_tach,
            steering=steering,
            throttle=throttle,
            dt=dt,
            gyro_z=gyro_z,
            gps_data=gps_data,
            acceleration=acceleration,
        )

        if self.comparator_console_log and self.update_count % self.comparator_log_interval == 0:
            robust_state = comparison["robust_state"]
            ekf_state = comparison["ekf_state"]
            delta_state = comparison["delta_state"]
            self._log_info(
                "[RKNetComparator] "
                f"tick={comparison['tick']} "
                f"source={comparison['robust_source']} "
                f"pos={comparison['position_error_norm']:.3f} "
                f"dtheta={comparison['heading_error']:.3f} "
                f"dv={comparison['velocity_error']:.3f} "
                f"robust=[{robust_state[0]:.3f}, {robust_state[1]:.3f}, {robust_state[2]:.3f}, {robust_state[3]:.3f}] "
                f"ekf=[{ekf_state[0]:.3f}, {ekf_state[1]:.3f}, {ekf_state[2]:.3f}, {ekf_state[3]:.3f}] "
                f"delta=[{delta_state[0]:.3f}, {delta_state[1]:.3f}, {delta_state[2]:.3f}, {delta_state[3]:.3f}]"
            )

    @staticmethod
    def _copy_comparison_entry(entry: Dict[str, Any]) -> Dict[str, Any]:
        copied: Dict[str, Any] = {}
        for key, value in entry.items():
            if isinstance(value, np.ndarray):
                copied[key] = value.copy()
            else:
                copied[key] = value
        return copied

    def update(
        self,
        motor_tach: float,
        steering: float,
        throttle: float,
        dt: float,
        gyro_z: float = 0.0,
        gps_data: Optional[Dict] = None,
        acceleration: Optional[np.ndarray] = None,
    ) -> bool:
        try:
            dt = max(float(dt), 1e-3)
            self.update_count += 1
            measurement = self._measurement_from_inputs(motor_tach, gyro_z, gps_data, dt=dt)
            raw_sample = self._raw_sample_from_inputs(
                motor_tach,
                steering,
                gyro_z,
                acceleration,
                gps_data,
            )
            self._append_sample(raw_sample, measurement, dt)

            estimate = None
            self.last_update_used_model = False
            if self.model_ready:
                try:
                    estimate = self._predict_with_model()
                    if estimate is not None:
                        # FIX-1: When disable_model_postprocess is True, the
                        # learned Kalman gain already handles sensor fusion —
                        # only wrap the heading angle.  Full post-processing
                        # (velocity LPF, GPS re-correction) is reserved for
                        # the kinematic fallback path.
                        if self.disable_model_postprocess:
                            estimate = np.asarray(estimate, dtype=np.float64).copy()
                            estimate = self._apply_heading_filter_output(
                                estimate,
                                gps_data=gps_data,
                            )
                        else:
                            estimate = self._post_process_estimate(
                                estimate, motor_tach, gyro_z, gps_data
                            )
                        self.last_update_used_model = True
                except Exception as exc:
                    self._clear_stream_state()
                    self._log_error("Robust KalmanNet inference failed", exc)
                    estimate = None

            if estimate is None:
                if not self.use_fallback:
                    return False
                estimate = self._fallback_update(motor_tach, dt, gyro_z, gps_data)

            self._record_comparison(
                robust_estimate=estimate,
                motor_tach=motor_tach,
                steering=steering,
                throttle=throttle,
                dt=dt,
                gyro_z=gyro_z,
                gps_data=gps_data,
                acceleration=acceleration,
                measurement=measurement,
            )
            self.internal_state = estimate.astype(np.float32)
            self.state = self.internal_state[:4].astype(np.float64, copy=True)
            self.last_update_time = time.time()
            return True

        except Exception as exc:
            self._log_error("Robust KalmanNet estimator update failed", exc)
            return False

    def get_state(self) -> np.ndarray:
        return self.state.copy()

    def get_internal_state(self) -> np.ndarray:
        return self.internal_state.copy()

    def get_comparator_state(self) -> Optional[np.ndarray]:
        if self.last_comparator_output is None:
            return None
        return self.last_comparator_output.copy()

    def get_last_comparison(self) -> Optional[Dict[str, Any]]:
        if self.last_comparison is None:
            return None
        return self._copy_comparison_entry(self.last_comparison)

    def get_comparison_history(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        if limit is None or limit <= 0:
            items = list(self.comparison_history)
        else:
            items = list(self.comparison_history)[-int(limit) :]
        return [self._copy_comparison_entry(item) for item in items]

    def reset(self, initial_pose: Optional[np.ndarray] = None):
        self.raw_history = {
            key: deque(maxlen=self.sequence_length) for key in self.RAW_KEYS
        }
        self.measurement_history = deque(maxlen=self.sequence_length)
        self.dt_history = deque(maxlen=self.sequence_length)
        self._clear_stream_state()
        self.internal_state = np.zeros(5, dtype=np.float32)
        if initial_pose is not None:
            self.internal_state[0] = float(initial_pose[0])
            self.internal_state[1] = float(initial_pose[1])
            if len(initial_pose) > 2:
                self.internal_state[2] = float(initial_pose[2])
        self.state = self.internal_state[:4].astype(np.float64, copy=True)
        self._reset_heading_filter(initial_pose)
        self.last_model_output = None
        self.last_pred_mask = None
        self.last_pred_mask_summary = None
        self.last_meas_mask = None
        self.last_K = None
        self.last_x_pred = None
        self.last_update_used_model = False
        self.update_count = 0
        self.last_comparator_output = None
        self.last_comparison = None
        self.comparison_history.clear()
        if self.comparator_log_file is not None:
            try:
                self.comparator_log_file.flush()
            except Exception:
                pass
        if self.ekf_comparator is not None:
            self.ekf_comparator.reset(initial_pose=initial_pose)

    def stop_recording(self):
        self._close_comparator_log_file()


# ============================================================
# LOSSES
# ============================================================

def weighted_state_mse(
    pred: torch.Tensor,
    target: torch.Tensor,
    weights: Optional[torch.Tensor] = None,
):
    """
    pred,target: [B,T,5]
    weights: [5] or None
    """
    err = pred - target
    # Out-of-place angle wrapping to avoid breaking autograd
    err = torch.cat([
        err[..., :2],
        wrap_angle(err[..., 2:3]),
        err[..., 3:],
    ], dim=-1)
    if weights is not None:
        err = err * weights.view(1, 1, -1)
    return (err ** 2).mean()


def meas_mask_supervision_loss(
    meas_mask_logits: torch.Tensor,
    meas_attack_labels: torch.Tensor,
) -> torch.Tensor:
    """
    Supervised loss encouraging the measurement mask to suppress
    corrupted measurement channels in the innovation.

    Uses BCEWithLogitsLoss (sigmoid + BCE fused) for numerical stability —
    avoids the CUDA assertion 'input_val >= zero && input_val <= one'
    that occurs when NaN/Inf values propagate through a standalone sigmoid.

    Args:
        meas_mask_logits: [B, T, 5] — raw logits (pre-sigmoid) for the
                          per-channel mask on measurements
        meas_attack_labels: [B, 5] — 1 if that measurement channel was attacked

    Returns:
        Scalar loss
    """
    # Target: 1 for clean channels, 0 for attacked channels
    target = (1.0 - meas_attack_labels).unsqueeze(1).expand_as(meas_mask_logits)
    loss = torch.nn.functional.binary_cross_entropy_with_logits(
        meas_mask_logits,
        target,
        reduction="mean",
    )
    return loss


def robuststatenet_loss(
    x_pred: torch.Tensor,
    x_upd: torch.Tensor,
    x_gt: torch.Tensor,
    lambda_upd: float = 0.8,
    lambda_pred: float = 0.2,
    weights: Optional[torch.Tensor] = None,
    pred_mask: Optional[torch.Tensor] = None,
    attack_labels: Optional[torch.Tensor] = None,
    lambda_mask: float = 0.1,
    meas_mask: Optional[torch.Tensor] = None,
    meas_mask_logits: Optional[torch.Tensor] = None,
    meas_attack_labels: Optional[torch.Tensor] = None,
    lambda_meas_mask: float = 0.1,
    K: Optional[torch.Tensor] = None,
    lambda_gain: float = 0.0,
):
    """
    Paper-style combined loss with optional mask supervision:
    L = lambda1 ||y_hat - y_u||^2 + lambda2 ||y_hat - y_p||^2
      + lambda_mask * pred_mask_supervision_loss
      + lambda_meas_mask * meas_mask_supervision_loss
      + lambda_gain * mean(K^2)   (gain regularization)

    When pred_mask and attack_labels are provided, the mask supervision
    term encourages the prediction mask to suppress attacked branches.

    When meas_mask and meas_attack_labels are provided, the measurement
    mask supervision encourages the update mask to suppress corrupted
    measurement channels in the innovation.

    When K is provided and lambda_gain > 0, a gain magnitude penalty
    discourages large Kalman gains to prevent over-correction at test time.
    """
    loss_upd = weighted_state_mse(x_upd, x_gt, weights)
    loss_pred = weighted_state_mse(x_pred, x_gt, weights)
    total = lambda_upd * loss_upd + lambda_pred * loss_pred
    logs = {"loss_upd": loss_upd.item(), "loss_pred": loss_pred.item()}

    if pred_mask is not None and attack_labels is not None and lambda_mask > 0:
        from sensor_attack_augmentation import mask_supervision_loss
        loss_mask = mask_supervision_loss(pred_mask, attack_labels)
        total = total + lambda_mask * loss_mask
        logs["loss_mask"] = loss_mask.item()

    if meas_mask_logits is not None and meas_attack_labels is not None and lambda_meas_mask > 0:
        loss_meas = meas_mask_supervision_loss(meas_mask_logits, meas_attack_labels)
        total = total + lambda_meas_mask * loss_meas
        logs["loss_meas_mask"] = loss_meas.item()

    # FIX-E: Gain regularization — penalize large Kalman gains to encourage
    # conservative corrections and prevent GPS snap at test time.
    if K is not None and lambda_gain > 0:
        loss_gain = (K ** 2).mean()
        total = total + lambda_gain * loss_gain
        logs["loss_gain"] = loss_gain.item()

    return total, logs


# ============================================================
# EXAMPLE DATA FORMAT
# ============================================================

def make_dummy_batch(B=4, T=20, device="cpu"):
    """
    Dummy tensors just to test shapes.
    Replace with real synchronized data.
    """
    raw = {
        "ax": torch.randn(B, T, 1, device=device) * 0.2,
        "ay": torch.randn(B, T, 1, device=device) * 0.1,
        "wz": torch.randn(B, T, 1, device=device) * 0.1,
        "delta": torch.randn(B, T, 1, device=device) * 0.05,
        "vfl": torch.randn(B, T, 1, device=device) * 0.1 + 2.0,
        "vfr": torch.randn(B, T, 1, device=device) * 0.1 + 2.0,
        "vrl": torch.randn(B, T, 1, device=device) * 0.1 + 2.0,
        "vrr": torch.randn(B, T, 1, device=device) * 0.1 + 2.0,
    }

    z_seq = torch.randn(B, T, 5, device=device)
    x_gt = torch.randn(B, T, 5, device=device)
    x0 = x_gt[:, 0, :]

    return raw, z_seq, x_gt, x0


# ============================================================
# TRAIN STEP
# ============================================================

def train_step(model, optimizer, batch, device="cpu"):
    raw, z_seq, x_gt, x0 = batch

    model.train()
    optimizer.zero_grad()

    out = model(raw=raw, z_seq=z_seq, x0=x0, teacher_forcing_state=x_gt)
    state_weights = torch.tensor([1.0, 1.0, 5.0, 1.0, 1.0], device=device)

    loss, logs = robuststatenet_loss(
        out["x_pred"],
        out["x_upd"],
        x_gt,
        lambda_upd=0.8,
        lambda_pred=0.2,
        weights=state_weights,
    )

    loss.backward()
    optimizer.step()

    logs["loss"] = loss.item()
    return logs


# ============================================================
# INFERENCE
# ============================================================

@no_grad()
def run_inference(model, raw, z_seq, x0):
    model.eval()
    return model(raw=raw, z_seq=z_seq, x0=x0, teacher_forcing_state=None)


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    if not TORCH_AVAILABLE:
        raise SystemExit("torch is required to run the Robust KalmanNet demo")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    cfg = RSNConfig(
        pred_hidden=64,
        upd_hidden=64,
        gain_hidden=64,
        pred_mlp_hidden=64,
        mask_hidden=64,
        use_hard_mask=False,
        dt=0.02,
    )

    model = RobustStateNet(cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-5)

    for epoch in range(5):
        batch = make_dummy_batch(B=8, T=30, device=device)
        logs = train_step(model, optimizer, batch, device=device)
        print(f"Epoch {epoch + 1}: {logs}")

    raw, z_seq, x_gt, x0 = make_dummy_batch(B=2, T=30, device=device)
    out = run_inference(model, raw, z_seq, x0)
    print("x_pred shape:", out["x_pred"].shape)
    print("x_upd shape :", out["x_upd"].shape)
    print("K shape     :", out["K"].shape)
