from __future__ import annotations

import csv
from collections import deque
import copy
from contextlib import contextmanager
from dataclasses import dataclass, field
import importlib
from pathlib import Path
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

try:
    from .qcar_heading_fusion import QCarHeadingFusion, QCarHeadingFusionConfig
except ImportError:
    from qcar_heading_fusion import QCarHeadingFusion, QCarHeadingFusionConfig

try:
    from .sensor_attack_augmentation import RuntimeAttackConfig, RuntimeSensorAttackSimulator
except ImportError:
    from sensor_attack_augmentation import RuntimeAttackConfig, RuntimeSensorAttackSimulator


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
    state_dim: int = 4          # [x, y, psi, v]
    meas_dim: int = 4           # [x_gnss, y_gnss, psi_imu, v_meas]

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
    gain_tanh_scale: float = 2.0
    dt: float = 0.02
    update_mask_init_bias: float = 2.0
    apply_meas_mask_to_innovation: bool = True
    normalize_updater_features: bool = True

    # ── Modular prediction step ──────────────────────────────────────────
    # "nn"        → RobustMotionPredictor (tri-LSTM, learnable, default)
    # "kinematic" → KinematicPredictor   (QCar bicycle model, no parameters)
    predictor_mode: str = "kinematic"
    # Wheelbase used by the analytical QCar bicycle predictor.
    kin_wheelbase: float = 0.2
    # Calibration-backed longitudinal model used by the analytical predictor.
    # Legacy kin_velocity_* fields remain as fallback when this is unset.
    longitudinal_model: str = ""
    velocity_lag_model: Dict[str, Any] = field(default_factory=dict)
    velocity_lag_lookup_model: Dict[str, Any] = field(default_factory=dict)
    accel_lag_model: Dict[str, Any] = field(default_factory=dict)
    coupled_kinematic_model: Dict[str, Any] = field(default_factory=dict)
    # Legacy fallback used only when no calibration-backed model is provided.
    kin_velocity_model: str = "tachometer"
    kin_velocity_tau: float = 0.301
    kin_velocity_gain: float = 6.598
    kin_max_velocity: float = 2.0
    kin_max_acceleration: float = 2.0


# ============================================================
# HELPERS
# ============================================================

no_grad = torch.no_grad


@contextmanager
def inference_context():
    """Use the lightest available torch inference context."""
    context_factory = (
        torch.inference_mode if hasattr(torch, "inference_mode") else torch.no_grad
    )
    with context_factory():
        yield


def make_H(
    device: torch.device,
    dtype=None,
    state_dim: int = 4,
    meas_dim: int = 4,
):
    """
    Measurement model z = H x
    State x = [x, y, psi, v]
    Measurement z = [x, y, psi, v]
    """
    if dtype is None:
        dtype = torch.float32
    H = torch.zeros(int(meas_dim), int(state_dim), device=device, dtype=dtype)
    diag_dim = min(int(state_dim), int(meas_dim))
    H[:diag_dim, :diag_dim] = torch.eye(diag_dim, device=device, dtype=dtype)
    return H



def wrap_angle(angle: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angle), torch.cos(angle))



def wrap_angle_scalar(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def dead_reckon_xy(x: float, y: float, heading: float, velocity: float, dt: float) -> Tuple[float, float]:
    dt_val = max(float(dt), 1e-3)
    if not np.isfinite(velocity):
        velocity = 0.0
    heading_val = wrap_angle_scalar(float(heading))
    next_x = float(x) + float(velocity) * float(np.cos(heading_val)) * dt_val
    next_y = float(y) + float(velocity) * float(np.sin(heading_val)) * dt_val
    return float(next_x), float(next_y)


def normalize_gps_dropout_xy_mode(mode: Any) -> str:
    mode_norm = str(mode or "freeze").strip().lower()
    alias_map = {
        "dead_reckoning": "dead_reckon",
        "dead-reckon": "dead_reckon",
        "dr": "dead_reckon",
    }
    mode_norm = alias_map.get(mode_norm, mode_norm)
    if mode_norm not in {"freeze", "dead_reckon"}:
        raise ValueError(
            f"Unsupported gps_dropout_xy_mode '{mode}'. "
            "Expected one of: ['dead_reckon', 'freeze']"
        )
    return mode_norm


def gps_position_is_valid(gps_data: Optional[Dict[str, Any]]) -> bool:
    return bool(
        gps_data is not None
        and gps_data.get(
            "position_valid",
            gps_data.get("fresh", gps_data.get("valid", False)),
        )
    )


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


def safe_l2_normalize(x: torch.Tensor, dim: int = -1) -> torch.Tensor:
    """Numerically stable L2 normalization for updater feature groups."""
    return F.normalize(x, p=2, dim=dim, eps=1e-12)



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
    def __init__(self, in_dim: int, hidden_dim: int, out_dim: int = 4):
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
        self.regressor = MotionRegressor(
            fusion_dim,
            cfg.pred_mlp_hidden,
            out_dim=cfg.state_dim,
        )

    def forward(
        self,
        imu_seq: torch.Tensor,
        steer_seq: torch.Tensor,
        wheel_seq: torch.Tensor,
        prev_state_seq: torch.Tensor,
        hidden_dict: Optional[Dict] = None,
        dt: Optional[torch.Tensor] = None,
        control_seq: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict, torch.Tensor]:
        """
        returns:
            x_pred_seq: [B,T,m] predicted state x_{k|k-1}
            motion_seq: [B,T,m] [dxE,dyE,dpsi,v_next] plus optional w_next
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
        prev_state = [x, y, psi, v] plus optional extra states
        motion     = [dxE, dyE, dpsi, v_next] plus optional extra states
        """
        x_prev = prev_state_seq[..., 0]
        y_prev = prev_state_seq[..., 1]
        psi_prev = prev_state_seq[..., 2]

        dxE = motion_seq[..., 0]
        dyE = motion_seq[..., 1]
        dpsi = motion_seq[..., 2]
        v_next = motion_seq[..., 3]

        dxG = dxE * torch.cos(psi_prev) - dyE * torch.sin(psi_prev)
        dyG = dxE * torch.sin(psi_prev) + dyE * torch.cos(psi_prev)

        x_next = x_prev + dxG
        y_next = y_prev + dyG
        psi_next = wrap_angle(psi_prev + dpsi)

        next_parts = [x_next, y_next, psi_next, v_next]
        if motion_seq.shape[-1] > 4:
            next_parts.extend(
                [motion_seq[..., idx] for idx in range(4, motion_seq.shape[-1])]
            )
        return torch.stack(next_parts, dim=-1)


# ============================================================
# KINEMATIC PREDICTOR (parameter-free alternative)
# ============================================================

class KinematicPredictor(nn.Module):
    """
    Parameter-free QCar bicycle-model prediction step.

    Accepts the exact same forward() signature as RobustMotionPredictor so that
    RobustStateNet.forward() requires zero structural changes.

    Default kinematic equations match QCarEKF.f():
        dxE   =  v_pose * dt
        dyE   =  0
        dpsi  =  v_pose * tan(delta) / L * dt
        w_next = v_pose * tan(delta) / L

    With longitudinal_model="tachometer", v_pose and v_next both copy
    motor_tach. Other longitudinal models predict v_next from previous
    state plus IMU/control inputs, creating velocity innovation in the update.

    Signals are extracted from existing branch-input tensors:
        delta      ← steer_seq[..., 2]  layout: [v, psi, delta]
        motor_tach ← wheel_seq[..., 2]  layout: [v, psi, vfl, vfr, vrl, vrr]
    """

    def __init__(self, cfg: RSNConfig):
        super().__init__()
        self.dt = cfg.dt
        self.wheelbase = max(float(cfg.kin_wheelbase), 1e-6)
        self.velocity_model = self._resolve_longitudinal_model(cfg)
        velocity_lag_cfg = dict(getattr(cfg, "velocity_lag_model", {}) or {})
        velocity_lag_lookup_cfg = dict(getattr(cfg, "velocity_lag_lookup_model", {}) or {})
        accel_lag_cfg = dict(getattr(cfg, "accel_lag_model", {}) or {})
        coupled_kinematic_cfg = dict(getattr(cfg, "coupled_kinematic_model", {}) or {})
        self.velocity_tau = max(
            float(velocity_lag_cfg.get("tau", cfg.kin_velocity_tau)),
            1e-6,
        )
        self.velocity_gain = float(
            velocity_lag_cfg.get("velocity_gain", cfg.kin_velocity_gain)
        )
        self.velocity_lag_deadband = max(
            float(velocity_lag_cfg.get("throttle_deadband", 0.0)),
            0.0,
        )
        self.velocity_lag_lookup_tau = max(
            float(velocity_lag_lookup_cfg.get("tau", self.velocity_tau)),
            1e-6,
        )
        lookup_throttle = np.asarray(
            velocity_lag_lookup_cfg.get("throttle_breakpoints", []),
            dtype=np.float32,
        ).reshape(-1)
        lookup_velocity = np.asarray(
            velocity_lag_lookup_cfg.get("steady_state_velocity_breakpoints", []),
            dtype=np.float32,
        ).reshape(-1)
        self.velocity_lag_lookup_enabled = bool(
            velocity_lag_lookup_cfg.get("enabled", True)
        ) and (
            lookup_throttle.size >= 2
            and lookup_throttle.size == lookup_velocity.size
        )
        if not self.velocity_lag_lookup_enabled:
            lookup_throttle = np.asarray([], dtype=np.float32)
            lookup_velocity = np.asarray([], dtype=np.float32)
        self.register_buffer(
            "velocity_lag_lookup_throttle_breakpoints",
            torch.as_tensor(lookup_throttle, dtype=torch.float32),
            persistent=False,
        )
        self.register_buffer(
            "velocity_lag_lookup_velocity_breakpoints",
            torch.as_tensor(lookup_velocity, dtype=torch.float32),
            persistent=False,
        )
        self.accel_lag_tau = max(
            float(accel_lag_cfg.get("tau", 0.318)),
            1e-6,
        )
        self.accel_lag_gain = float(accel_lag_cfg.get("input_gain", 1.0))
        self.max_velocity = max(float(cfg.kin_max_velocity), 1e-6)
        self.max_acceleration = max(float(cfg.kin_max_acceleration), 1e-6)
        self.coupled_kinematic_accel_features = tuple(
            coupled_kinematic_cfg.get(
                "acceleration_features",
                [
                    "bias",
                    "throttle",
                    "abs_throttle",
                    "velocity",
                    "abs_velocity",
                    "steering",
                    "steering_sq",
                    "throttle_abs_steering",
                    "velocity_steering_sq",
                ],
            )
        )
        self.coupled_kinematic_yaw_features = tuple(
            coupled_kinematic_cfg.get(
                "yaw_rate_features",
                [
                    "bias",
                    "steering",
                    "tan_steering",
                    "velocity_steering",
                    "velocity_tan_steering",
                    "throttle_steering",
                ],
            )
        )
        accel_coeffs = np.asarray(
            coupled_kinematic_cfg.get("acceleration_coefficients", []),
            dtype=np.float32,
        ).reshape(-1)
        yaw_coeffs = np.asarray(
            coupled_kinematic_cfg.get("yaw_rate_coefficients", []),
            dtype=np.float32,
        ).reshape(-1)
        self.coupled_kinematic_enabled = bool(
            coupled_kinematic_cfg.get("enabled", False)
        ) and (
            accel_coeffs.size == len(self.coupled_kinematic_accel_features)
            and yaw_coeffs.size == len(self.coupled_kinematic_yaw_features)
        )
        if not self.coupled_kinematic_enabled:
            accel_coeffs = np.asarray([], dtype=np.float32)
            yaw_coeffs = np.asarray([], dtype=np.float32)
        limits_cfg = coupled_kinematic_cfg.get("limits", {})
        if not isinstance(limits_cfg, dict):
            limits_cfg = {}
        self.coupled_kinematic_max_yaw_rate = max(
            float(limits_cfg.get("max_yaw_rate", 8.0)),
            1e-6,
        )
        self.register_buffer(
            "coupled_kinematic_accel_coeffs",
            torch.as_tensor(accel_coeffs, dtype=torch.float32),
            persistent=False,
        )
        self.register_buffer(
            "coupled_kinematic_yaw_coeffs",
            torch.as_tensor(yaw_coeffs, dtype=torch.float32),
            persistent=False,
        )

    @staticmethod
    def _resolve_longitudinal_model(cfg: RSNConfig) -> str:
        model = str(getattr(cfg, "longitudinal_model", "") or "").strip().lower()
        if not model:
            model = str(getattr(cfg, "kin_velocity_model", "tachometer") or "tachometer").strip().lower()
        alias_map = {
            "accel_lag": "acceleration_lag",
        }
        model = alias_map.get(model, model)
        valid_models = {
            "tachometer",
            "imu_acceleration",
            "velocity_lag",
            "velocity_lag_lookup",
            "velocity_command",
            "acceleration_lag",
            "simple_acceleration",
            "coupled_kinematic",
        }
        if model not in valid_models:
            return "tachometer"
        return model

    def _effective_velocity_lag_throttle(self, throttle: torch.Tensor) -> torch.Tensor:
        deadband = float(self.velocity_lag_deadband)
        if deadband <= 0.0:
            return throttle
        magnitude = torch.clamp(torch.abs(throttle) - deadband, min=0.0)
        return torch.sign(throttle) * magnitude

    def _velocity_lag_lookup_target(self, throttle: torch.Tensor) -> torch.Tensor:
        if not self.velocity_lag_lookup_enabled:
            return self.velocity_gain * self._effective_velocity_lag_throttle(throttle)

        breakpoints = self.velocity_lag_lookup_throttle_breakpoints.to(
            device=throttle.device,
            dtype=throttle.dtype,
        )
        velocities = self.velocity_lag_lookup_velocity_breakpoints.to(
            device=throttle.device,
            dtype=throttle.dtype,
        )
        flat_throttle = throttle.reshape(-1)
        indices = torch.bucketize(flat_throttle, breakpoints)
        indices = torch.clamp(indices, 1, breakpoints.numel() - 1)

        x0 = breakpoints[indices - 1]
        x1 = breakpoints[indices]
        y0 = velocities[indices - 1]
        y1 = velocities[indices]
        ratio = (flat_throttle - x0) / torch.clamp(x1 - x0, min=1e-6)
        ratio = torch.clamp(ratio, 0.0, 1.0)
        interpolated = y0 + ratio * (y1 - y0)
        interpolated = torch.where(
            flat_throttle <= breakpoints[0],
            velocities[0],
            interpolated,
        )
        interpolated = torch.where(
            flat_throttle >= breakpoints[-1],
            velocities[-1],
            interpolated,
        )
        return interpolated.reshape_as(throttle)

    def _coupled_feature_tensor(
        self,
        feature_names: Tuple[str, ...],
        velocity: torch.Tensor,
        throttle: torch.Tensor,
        delta: torch.Tensor,
    ) -> torch.Tensor:
        delta_clip = torch.clamp(delta, -0.7, 0.7)
        tan_delta = torch.tan(delta_clip)
        feature_map = {
            "bias": torch.ones_like(velocity),
            "throttle": throttle,
            "abs_throttle": torch.abs(throttle),
            "velocity": velocity,
            "abs_velocity": torch.abs(velocity),
            "steering": delta,
            "abs_steering": torch.abs(delta),
            "steering_sq": delta * delta,
            "tan_steering": tan_delta,
            "velocity_steering": velocity * delta,
            "velocity_tan_steering": velocity * tan_delta,
            "throttle_steering": throttle * delta,
            "throttle_abs_steering": throttle * torch.abs(delta),
            "velocity_steering_sq": velocity * delta * delta,
        }
        cols = [feature_map.get(name, torch.zeros_like(velocity)) for name in feature_names]
        return torch.cat(cols, dim=-1) if cols else torch.zeros_like(velocity)

    def _predict_coupled_kinematic(
        self,
        prev_v: torch.Tensor,
        throttle: torch.Tensor,
        delta: torch.Tensor,
        dt: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        accel_feat = self._coupled_feature_tensor(
            self.coupled_kinematic_accel_features,
            velocity=prev_v,
            throttle=throttle,
            delta=delta,
        )
        yaw_feat = self._coupled_feature_tensor(
            self.coupled_kinematic_yaw_features,
            velocity=prev_v,
            throttle=throttle,
            delta=delta,
        )
        accel_coeffs = self.coupled_kinematic_accel_coeffs.to(
            device=prev_v.device,
            dtype=prev_v.dtype,
        ).view(1, 1, -1)
        yaw_coeffs = self.coupled_kinematic_yaw_coeffs.to(
            device=prev_v.device,
            dtype=prev_v.dtype,
        ).view(1, 1, -1)
        a_pred = torch.sum(accel_feat * accel_coeffs, dim=-1, keepdim=True)
        yaw_pred = torch.sum(yaw_feat * yaw_coeffs, dim=-1, keepdim=True)
        a_pred = torch.clamp(a_pred, -self.max_acceleration, self.max_acceleration)
        yaw_pred = torch.clamp(
            yaw_pred,
            -self.coupled_kinematic_max_yaw_rate,
            self.coupled_kinematic_max_yaw_rate,
        )
        v_next = prev_v + a_pred * dt
        v_next = torch.clamp(v_next, -self.max_velocity, self.max_velocity)
        return v_next, a_pred, yaw_pred

    def _predict_velocity(
        self,
        prev_state_seq: torch.Tensor,
        imu_seq: torch.Tensor,
        motor_tach: torch.Tensor,
        control_seq: Optional[torch.Tensor],
        delta: Optional[torch.Tensor],
        dt: torch.Tensor,
        hidden_dict: Optional[Dict[str, Any]] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict[str, torch.Tensor]]:
        prev_v = prev_state_seq[..., 3:4]
        ax = imu_seq[..., 2:3]
        throttle = control_seq if control_seq is not None else torch.zeros_like(prev_v)
        delta = delta if delta is not None else torch.zeros_like(prev_v)
        hidden_dict = hidden_dict or {}
        longitudinal_accel_state = hidden_dict.get("longitudinal_accel_state")
        if longitudinal_accel_state is None:
            longitudinal_accel_state = torch.zeros_like(prev_v)
        else:
            longitudinal_accel_state = longitudinal_accel_state.to(
                device=prev_v.device,
                dtype=prev_v.dtype,
            )
        hidden_out: Dict[str, torch.Tensor] = {}

        model = self.velocity_model
        if model == "imu_acceleration":
            a_pred = ax
            v_next = prev_v + a_pred * dt
        elif model == "velocity_lag":
            u_eff = self._effective_velocity_lag_throttle(throttle)
            a_pred = (-prev_v + self.velocity_gain * u_eff) / self.velocity_tau
            v_next = prev_v + a_pred * dt
        elif model == "velocity_lag_lookup":
            v_target = self._velocity_lag_lookup_target(throttle)
            a_pred = (v_target - prev_v) / self.velocity_lag_lookup_tau
            v_next = prev_v + a_pred * dt
        elif model == "velocity_command":
            a_pred = (throttle - prev_v) / self.velocity_tau
            v_next = prev_v + a_pred * dt
        elif model == "acceleration_lag":
            longitudinal_accel_state = longitudinal_accel_state + dt * (
                -(1.0 / self.accel_lag_tau) * longitudinal_accel_state
                + (self.accel_lag_gain / self.accel_lag_tau) * throttle
            )
            a_pred = longitudinal_accel_state
            v_next = prev_v + a_pred * dt
        elif model == "simple_acceleration":
            a_pred = throttle
            v_next = prev_v + a_pred * dt
        elif model == "coupled_kinematic" and self.coupled_kinematic_enabled:
            v_next, a_pred, _yaw_pred = self._predict_coupled_kinematic(
                prev_v=prev_v,
                throttle=throttle,
                delta=delta,
                dt=dt,
            )
        else:
            a_pred = torch.clamp(
                (motor_tach - prev_v) / torch.clamp(dt, min=1e-6),
                -self.max_acceleration,
                self.max_acceleration,
            )
            v_next = motor_tach

        a_pred = torch.clamp(a_pred, -self.max_acceleration, self.max_acceleration)
        v_next = torch.clamp(v_next, -self.max_velocity, self.max_velocity)
        if model == "acceleration_lag":
            hidden_out["longitudinal_accel_state"] = a_pred
        return v_next, a_pred, hidden_out

    def forward(
        self,
        imu_seq: torch.Tensor,       # [B, 1, 5]  [v, psi, ax, ay, wz]
        steer_seq: torch.Tensor,     # [B, 1, 3]  [v, psi, delta]
        wheel_seq: torch.Tensor,     # [B, 1, 6]  [v, psi, vfl, vfr, vrl, vrr]
        prev_state_seq: torch.Tensor,  # [B, 1, 5]  x_{k-1|k-1}
        hidden_dict: Optional[Dict] = None,
        dt: Optional[torch.Tensor] = None,
        control_seq: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict, None]:
        """
        Returns:
            x_pred   : [B, 1, state_dim]
            motion   : [B, 1, state_dim]  [dxE, dyE, dpsi, v_next]
            {}       : empty hidden dict (stateless)
            None     : no feature mask
        """
        # ── extract signals ──────────────────────────────────────────────
        delta      = steer_seq[..., 2:3]        # [B, 1, 1]
        motor_tach = wheel_seq[..., 2:3]        # [B, 1, 1]  use vfl as representative

        # ── kinematic model ──────────────────────────────────────────────
        active_dt = dt if dt is not None else self.dt
        if not torch.is_tensor(active_dt):
            active_dt = torch.as_tensor(
                active_dt,
                device=prev_state_seq.device,
                dtype=prev_state_seq.dtype,
            )

        v_next, _, hidden_out = self._predict_velocity(
            prev_state_seq=prev_state_seq,
            imu_seq=imu_seq,
            motor_tach=motor_tach,
            control_seq=control_seq,
            delta=delta,
            dt=active_dt,
            hidden_dict=hidden_dict,
        )
        if (
            self.velocity_model == "coupled_kinematic"
            and self.coupled_kinematic_enabled
        ):
            throttle = (
                control_seq if control_seq is not None else torch.zeros_like(motor_tach)
            )
            prev_v = prev_state_seq[..., 3:4]
            v_pose = 0.5 * (prev_v + v_next)
            _v_ck, _a_ck, yaw_rate = self._predict_coupled_kinematic(
                prev_v=prev_v,
                throttle=throttle,
                delta=delta,
                dt=active_dt,
            )
        else:
            v_pose = motor_tach if self.velocity_model == "tachometer" else v_next
            yaw_rate = v_pose * torch.tan(delta) / self.wheelbase
        dxE    = v_pose * active_dt
        dyE    = torch.zeros_like(dxE)
        dpsi   = yaw_rate * active_dt
        motion = torch.cat([dxE, dyE, dpsi, v_next], dim=-1)
        x_pred = RobustMotionPredictor.motion_to_state(prev_state_seq, motion)

        return x_pred, motion, hidden_out, None


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
        self.gain_tanh_scale = float(cfg.gain_tanh_scale)
        self.update_mask_init_bias = float(cfg.update_mask_init_bias)
        self.apply_meas_mask_to_innovation = bool(
            getattr(cfg, "apply_meas_mask_to_innovation", True)
        )
        self.normalize_updater_features = bool(
            getattr(cfg, "normalize_updater_features", True)
        )
        # dx, normalized innovation/delta, availability, GPS flags, plus
        # raw innovation magnitudes. The magnitude terms are important for
        # attack response because normalization alone removes anomaly scale.
        self.feat_dim_upd = self.m + self.n + self.n + self.n + 2 + 2 * self.n

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
        theta_valid_seq: Optional[torch.Tensor] = None,
        gps_hold_valid_seq: Optional[torch.Tensor] = None,
        gps_age_seq: Optional[torch.Tensor] = None,
        hidden: Optional[Dict[str, torch.Tensor]] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict[str, torch.Tensor], torch.Tensor, torch.Tensor]:
        
        device = x_pred_seq.device
        B, T, _ = z_seq.shape
        H = make_H(
            device=device,
            dtype=x_pred_seq.dtype,
            state_dim=self.m,
            meas_dim=self.n,
        )
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

        if theta_valid_seq is None:
            theta_valid_seq = gps_valid_seq
        else:
            theta_valid_seq = theta_valid_seq.to(device=device, dtype=x_pred_seq.dtype)

        if gps_hold_valid_seq is None:
            gps_hold_valid_seq = gps_valid_seq
        else:
            gps_hold_valid_seq = gps_hold_valid_seq.to(device=device, dtype=x_pred_seq.dtype)

        if gps_age_seq is None:
            gps_age_seq = torch.zeros(B, T, 1, device=device, dtype=x_pred_seq.dtype)
        else:
            gps_age_seq = gps_age_seq.to(device=device, dtype=x_pred_seq.dtype).clamp_min(0.0)

        meas_availability_seq = torch.cat(
            [
                gps_valid_seq,
                gps_valid_seq,
                theta_valid_seq,
                torch.ones(
                    B,
                    T,
                    max(self.n - 3, 0),
                    device=device,
                    dtype=x_pred_seq.dtype,
                ),
            ],
            dim=-1,
        )
        gps_age_feat = torch.log1p(gps_age_seq.clamp(max=10.0))
        dz_p = dz_p * meas_availability_seq
        dz = dz * meas_availability_seq
        dz_p_mag = torch.log1p(torch.abs(dz_p))
        dz_mag = torch.log1p(torch.abs(dz))

        if self.normalize_updater_features:
            dx = safe_l2_normalize(dx)
            dz_p = safe_l2_normalize(dz_p)
            dz = safe_l2_normalize(dz)

        f_upd = torch.cat(
            [
                dx,
                dz_p,
                dz,
                meas_availability_seq,
                gps_hold_valid_seq,
                gps_age_feat,
                dz_p_mag,
                dz_mag,
            ],
            dim=-1,
        )

        # 2. Generate mask
        mask_logits = self.mask_net(f_upd)
        if self.use_hard_mask:
            m_upd = hard_sigmoid_st(mask_logits)
        else:
            m_upd = torch.sigmoid(mask_logits)

        # 3. Apply mask to features before GRU
        f_upd_masked = f_upd * m_upd

        # Return the learned measurement mask for diagnostics/supervision.
        learned_meas_mask = m_upd[..., self.m : self.m + self.n]
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
        # Keep gain squashing configurable so quick tests can widen the
        # effective range without fully removing the nonlinearity.
        if self.constrain_gain:
            K_flat = self.gain_tanh_scale * torch.tanh(K_flat)
        K_seq = K_flat.view(B, T, self.m, self.n)

        # 6. Apply additive correction
        # x_{k|k} = x_{k|k-1} + K_k @ (z_k - Hx_{k|k-1})
        raw_innovation = wrap_state_residual(z_seq - Hx_pred) * meas_availability_seq
        innovation_for_update = raw_innovation
        if self.apply_meas_mask_to_innovation:
            # Make the learned measurement mask directly suppress attacked
            # channels in the actual correction path instead of only
            # influencing the GRU features used to predict K.
            innovation_for_update = innovation_for_update * learned_meas_mask
        corr = torch.matmul(K_seq, innovation_for_update.unsqueeze(-1)).squeeze(-1)

        x_upd = x_pred_seq + corr

        x_upd = torch.cat([
            x_upd[..., :2],
            wrap_angle(x_upd[..., 2:3]),
            x_upd[..., 3:],
        ], dim=-1)

        hidden_out = {"gru": h_k_next}

        return x_upd, K_seq, hidden_out, learned_meas_mask, meas_mask_logits_out



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
            ax, ay, wz, delta, vfl, vfr, vrl, vrr, throttle,
            gps_valid, gps_hold_valid, gps_age_sec
        state_for_input = [B,T,4] containing [x,y,psi,v]
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
            control_t = raw_t.get("throttle")
            x_pred_t, motion_t, pred_hidden_local, mask_t = self.predictor(
                imu_t,
                steer_t,
                wheel_t,
                prev_state_seq,
                pred_hidden_local,
                dt=dt_t,
                control_seq=control_t,
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
                raw_t.get("theta_valid"),
                raw_t.get("gps_hold_valid"),
                raw_t.get("gps_age_sec"),
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
    3. Loads a trained checkpoint.
    4. Runs streaming or replay inference directly in the observer loop.

    Output state follows the current local observer contract:
        [x, y, theta, v]
    The internal model state is 4D:
        [x, y, theta, v]
    """

    RAW_KEYS = (
        "ax",
        "ay",
        "wz",
        "delta",
        "vfl",
        "vfr",
        "vrl",
        "vrr",
        "throttle",
        "gps_valid",
        "theta_valid",
        "gps_hold_valid",
        "gps_age_sec",
    )
    DEFAULT_BRANCH_ATTACK_TYPES = (
        "bias",
        "scale",
        "freeze",
        "noise",
        "ramp",
        "zero_out",
    )
    DEFAULT_GPS_ATTACK_TYPES = (
        "noise",
        "freeze",
        "jump",
        "dropout",
        "reacquisition",
    )

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
        self.wheel_speed_scale = float(self.config.get("wheel_speed_scale", 1.0))
        self.heading_filter_enabled = bool(
            self.config.get("heading_filter_enabled", True)
        )
        self.heading_filter_q_psi = max(
            1e-9, float(self.config.get("heading_filter_q_psi", 1e-4))
        )
        self.heading_filter_q_bias = max(
            1e-9, float(self.config.get("heading_filter_q_bias", 1e-3))
        )
        self.heading_filter_r_gps = max(
            1e-9, float(self.config.get("heading_filter_r_gps", 1e-3))
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
        self.gps_dropout_xy_mode = normalize_gps_dropout_xy_mode(
            self.config.get("gps_dropout_xy_mode", "freeze")
        )
        self.enable_ekf_comparator = bool(self.config.get("enable_ekf_comparator", False))
        self.enable_clean_reference_comparator = bool(
            self.config.get("enable_clean_reference_comparator", False)
        )
        self.publish_clean_reference_output = bool(
            self.config.get(
                "publish_clean_reference_output",
                self.enable_clean_reference_comparator,
            )
        )
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
        fault_cfg = self.config.get(
            "sensor_failure_simulation",
            self.config.get("fault_simulation", {}),
        )
        if not isinstance(fault_cfg, dict):
            fault_cfg = {}
        self.sensor_failure_simulation_cfg = self._normalize_runtime_attack_config(
            fault_cfg
        )
        self.sensor_failure_simulator: Optional[RuntimeSensorAttackSimulator] = None
        self.last_sensor_failure_metadata: Optional[Dict[str, Any]] = None

        self.device = self._resolve_device(self.config.get("device", "auto"))
        self.model_cfg = RSNConfig(
            pred_hidden=int(self.config.get("pred_hidden", 32)),
            upd_hidden=int(self.config.get("upd_hidden", 32)),
            gain_hidden=int(self.config.get("gain_hidden", 32)),
            pred_mlp_hidden=int(self.config.get("pred_mlp_hidden", 32)),
            mask_hidden=int(self.config.get("mask_hidden", 32)),
            use_hard_mask=bool(self.config.get("use_hard_mask", False)),
            constrain_gain=bool(self.config.get("constrain_gain", True)),
            gain_tanh_scale=float(self.config.get("gain_tanh_scale", 2.0)),
            dt=float(self.config.get("dt", 0.02)),
            update_mask_init_bias=float(self.config.get("update_mask_init_bias", 2.0)),
            apply_meas_mask_to_innovation=bool(
                self.config.get("apply_meas_mask_to_innovation", True)
            ),
            predictor_mode=str(self.config.get("predictor_mode", "kinematic")),
            kin_wheelbase=float(self.config.get("kin_wheelbase", 0.2)),
            longitudinal_model=str(self.config.get("longitudinal_model", "")),
            velocity_lag_model=copy.deepcopy(
                dict(self.config.get("velocity_lag_model", {}) or {})
            ),
            velocity_lag_lookup_model=copy.deepcopy(
                dict(self.config.get("velocity_lag_lookup_model", {}) or {})
            ),
            accel_lag_model=copy.deepcopy(
                dict(self.config.get("accel_lag_model", {}) or {})
            ),
            coupled_kinematic_model=copy.deepcopy(
                dict(self.config.get("coupled_kinematic_model", {}) or {})
            ),
            kin_velocity_model=str(self.config.get("kin_velocity_model", "tachometer")),
            kin_velocity_tau=float(self.config.get("kin_velocity_tau", 0.301)),
            kin_velocity_gain=float(self.config.get("kin_velocity_gain", 6.598)),
            kin_max_velocity=float(self.config.get("kin_max_velocity", self.config.get("max_velocity", 2.0))),
            kin_max_acceleration=float(self.config.get("kin_max_acceleration", self.config.get("max_acceleration", 2.0))),
        )

        self.model = RobustStateNet(self.model_cfg).to(self.device)
        self.model.eval()
        active_longitudinal_model = (
            self.model_cfg.longitudinal_model or self.model_cfg.kin_velocity_model
        )
        self._log_info(
            f"Robust KalmanNet predictor_mode='{self.model_cfg.predictor_mode}'"
            + (f" (kin_wheelbase={self.model_cfg.kin_wheelbase}, "
               f"longitudinal_model={active_longitudinal_model})"
               if self.model_cfg.predictor_mode == "kinematic" else "")
        )
        self._log_info(
            f"Robust KalmanNet gps_dropout_xy_mode='{self.gps_dropout_xy_mode}'"
        )
        if self.publish_clean_reference_output:
            self._log_warning(
                "Robust KalmanNet is configured to publish the clean-reference "
                "comparator output instead of the model output. Disable "
                "'publish_clean_reference_output' for real robustness evaluation."
            )
        self.model_path = self._resolve_model_path(self.config.get("model_path"))
        self.last_model_output = None
        self.last_pred_mask = None
        self.last_pred_mask_summary = None
        self.last_meas_mask = None
        self.last_K = None
        self.last_x_pred = None
        self.update_count = 0
        self.ekf_comparator = None
        self.ekf_comparator_ready = False
        self.clean_reference_comparator = None
        self.clean_reference_comparator_ready = False
        self.last_comparator_output = None
        self.last_reference_output = None
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

        self.internal_state = np.zeros(self.model_cfg.state_dim, dtype=np.float32)
        if initial_pose is not None:
            self.internal_state[0] = float(initial_pose[0])
            self.internal_state[1] = float(initial_pose[1])
            if len(initial_pose) > 2:
                self.internal_state[2] = float(initial_pose[2])
        self.state = self.internal_state[:4].astype(np.float64, copy=True)
        # Track last GPS-valid position for dropout handling in z[x,y].
        self._last_gps_x: Optional[float] = None
        self._last_gps_y: Optional[float] = None
        self.heading_fusion: Optional[QCarHeadingFusion] = None
        self.last_filtered_heading = wrap_angle_scalar(float(self.internal_state[2]))
        self._reset_heading_filter(initial_pose)

        if self.sensor_failure_simulation_cfg.get("enabled", False):
            runtime_attack_cfg = RuntimeAttackConfig.from_dict(
                self.sensor_failure_simulation_cfg
            )
            self.sensor_failure_simulator = RuntimeSensorAttackSimulator(
                runtime_attack_cfg
            )
            self._log_info(
                "Robust KalmanNet sensor failure simulation enabled "
                f"(branch_prob={runtime_attack_cfg.attack_prob}, "
                f"gps_prob={runtime_attack_cfg.gps_attack_prob}, "
                f"duration={runtime_attack_cfg.min_attack_steps}-"
                f"{runtime_attack_cfg.max_attack_steps} steps)"
            )

        self._initialize_model()
        self._initialize_ekf_comparator(initial_pose)

    @staticmethod
    def _resolve_device(device_name: str):
        if device_name == "auto":
            return torch.device("cuda" if torch.cuda.is_available() else "cpu")
        return torch.device(device_name)

    @staticmethod
    def _extract_state_dict(checkpoint: Any) -> Dict[str, torch.Tensor]:
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

    def _warn_if_checkpoint_config_mismatch(self, checkpoint: Any) -> None:
        if not isinstance(checkpoint, dict):
            return

        training_config = checkpoint.get("training_config")
        if not isinstance(training_config, dict):
            return

        training_section = training_config.get("training", {})
        model_section = training_config.get("model", {})
        if not isinstance(training_section, dict):
            training_section = {}
        if not isinstance(model_section, dict):
            model_section = {}

        runtime_longitudinal_model = str(
            self.model_cfg.longitudinal_model or self.model_cfg.kin_velocity_model
        ).strip().lower()
        trained_longitudinal_model = str(
            model_section.get("longitudinal_model", "")
            or model_section.get("kin_velocity_model", "")
        ).strip().lower()
        trained_predictor_mode = str(
            model_section.get("predictor_mode", "")
        ).strip().lower()
        trained_gps_dropout_xy_mode = str(
            training_section.get("gps_dropout_xy_mode", "")
        ).strip().lower()
        trained_sequence_length = int(
            training_section.get("sequence_length", 0) or 0
        )
        trained_sequence_length_phase_c = int(
            training_section.get("sequence_length_phase_c", 0) or 0
        )

        warnings: List[str] = []
        runtime_predictor_mode = str(self.model_cfg.predictor_mode).strip().lower()
        if trained_predictor_mode and trained_predictor_mode != runtime_predictor_mode:
            warnings.append(
                "predictor_mode "
                f"(checkpoint='{trained_predictor_mode}', runtime='{runtime_predictor_mode}')"
            )
        if (
            trained_longitudinal_model
            and trained_longitudinal_model != runtime_longitudinal_model
        ):
            warnings.append(
                "longitudinal model "
                f"(checkpoint='{trained_longitudinal_model}', runtime='{runtime_longitudinal_model}')"
            )
        if (
            trained_gps_dropout_xy_mode
            and trained_gps_dropout_xy_mode != self.gps_dropout_xy_mode
        ):
            warnings.append(
                "gps_dropout_xy_mode "
                f"(checkpoint='{trained_gps_dropout_xy_mode}', runtime='{self.gps_dropout_xy_mode}')"
            )
        if (
            trained_sequence_length > 0
            and self.sequence_length < trained_sequence_length
        ):
            warnings.append(
                "sequence_length "
                f"(checkpoint='{trained_sequence_length}', runtime='{self.sequence_length}')"
            )
        if (
            trained_sequence_length_phase_c > 0
            and self.sequence_length < trained_sequence_length_phase_c
        ):
            warnings.append(
                "sequence_length_phase_c "
                f"(checkpoint='{trained_sequence_length_phase_c}', runtime='{self.sequence_length}')"
            )

        if warnings:
            self._log_warning(
                "Robust KalmanNet runtime config differs from checkpoint training config: "
                + "; ".join(warnings)
            )

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

    @classmethod
    def _normalize_runtime_attack_config(
        cls, config: Optional[Dict[str, Any]]
    ) -> Dict[str, Any]:
        normalized = copy.deepcopy(config) if isinstance(config, dict) else {}
        target_sensor = str(normalized.pop("target_sensor", "") or "").strip().lower()

        attack_types = normalized.pop("attack_types", None)
        if attack_types is not None and "enabled_attacks" not in normalized:
            normalized["enabled_attacks"] = attack_types

        enabled_attacks = normalized.get("enabled_attacks")
        if isinstance(enabled_attacks, str):
            normalized["enabled_attacks"] = [enabled_attacks]
        elif enabled_attacks is None:
            normalized["enabled_attacks"] = list(cls.DEFAULT_BRANCH_ATTACK_TYPES)
        else:
            normalized["enabled_attacks"] = list(enabled_attacks)

        gps_attack_types = normalized.get("gps_attack_types")
        if isinstance(gps_attack_types, str):
            normalized["gps_attack_types"] = [gps_attack_types]
        elif gps_attack_types is None:
            normalized["gps_attack_types"] = list(cls.DEFAULT_GPS_ATTACK_TYPES)
        else:
            normalized["gps_attack_types"] = list(gps_attack_types)

        gps_enabled = normalized.pop("gps_enabled", None)
        if gps_enabled is False:
            normalized["gps_attack_prob"] = 0.0

        if "forced_branches" in normalized:
            forced_branches = normalized.get("forced_branches")
            if isinstance(forced_branches, str):
                normalized["forced_branches"] = [forced_branches]
            else:
                normalized["forced_branches"] = list(forced_branches or [])

        branch_alias_map = {
            "imu": "imu",
            "steering": "steer",
            "steer": "steer",
            "velocity": "wheel",
            "wheel": "wheel",
        }
        if target_sensor == "gps":
            normalized["forced_branches"] = []
            normalized["force_gps_attack"] = True
            normalized["attack_prob"] = 0.0
            normalized["gps_attack_prob"] = 1.0
            normalized["immediate_attack"] = True
        elif target_sensor in branch_alias_map:
            normalized["forced_branches"] = [branch_alias_map[target_sensor]]
            normalized["force_gps_attack"] = False
            normalized["attack_prob"] = 1.0
            normalized["gps_attack_prob"] = 0.0
            normalized["immediate_attack"] = True
            normalized["max_branches_attacked"] = 1
        elif target_sensor in {"", "random"}:
            normalized.setdefault("force_gps_attack", False)
            normalized.setdefault("immediate_attack", True)
        else:
            raise ValueError(
                "Unsupported target_sensor "
                f"'{target_sensor}'. Expected one of: "
                "['random', 'imu', 'steering', 'velocity', 'gps']"
            )

        return normalized

    def start_sensor_attack(self, config: Optional[Dict[str, Any]] = None) -> bool:
        try:
            normalized_config = self._normalize_runtime_attack_config(config)
            normalized_config["enabled"] = True
            runtime_attack_cfg = RuntimeAttackConfig.from_dict(normalized_config)
            self.sensor_failure_simulator = RuntimeSensorAttackSimulator(
                runtime_attack_cfg
            )
            self.sensor_failure_simulation_cfg = copy.deepcopy(normalized_config)
            self.last_sensor_failure_metadata = None
            self._log_info(
                "Robust KalmanNet local sensor attack enabled "
                f"(branch_prob={runtime_attack_cfg.attack_prob}, "
                f"gps_prob={runtime_attack_cfg.gps_attack_prob}, "
                f"duration={runtime_attack_cfg.min_attack_steps}-"
                f"{runtime_attack_cfg.max_attack_steps} steps)"
            )
            return True
        except Exception as exc:
            self._log_error("Failed to start Robust KalmanNet sensor attack", exc)
            return False

    def stop_sensor_attack(self) -> bool:
        self.sensor_failure_simulator = None
        self.last_sensor_failure_metadata = None
        if isinstance(self.sensor_failure_simulation_cfg, dict):
            self.sensor_failure_simulation_cfg["enabled"] = False
        self._log_info("Robust KalmanNet local sensor attack disabled")
        return True

    def get_sensor_attack_status(self) -> Dict[str, Any]:
        metadata = (
            copy.deepcopy(self.last_sensor_failure_metadata)
            if isinstance(self.last_sensor_failure_metadata, dict)
            else {}
        )
        branch_attacks = metadata.get("branch_attacks", []) or []
        gps_attack = metadata.get("gps_attack")
        remaining_steps: List[int] = []
        branch_types: List[str] = []

        for item in branch_attacks:
            branch_name = str(item.get("branch", "")).strip()
            attack_type = str(item.get("attack_type", "")).strip()
            if branch_name and attack_type:
                branch_types.append(f"{branch_name}:{attack_type}")
            try:
                remaining_steps.append(int(item.get("remaining_steps", 0)))
            except (TypeError, ValueError):
                pass

        gps_type = ""
        if isinstance(gps_attack, dict):
            gps_type = str(gps_attack.get("attack_type", "")).strip()
            try:
                remaining_steps.append(int(gps_attack.get("remaining_steps", 0)))
            except (TypeError, ValueError):
                pass

        current_intensity = metadata.get("current_intensity", {}) or {}
        return {
            "local_sensor_attack_supported": True,
            "local_sensor_attack_enabled": bool(
                self.sensor_failure_simulator is not None
            ),
            "local_sensor_attack_active": bool(metadata.get("active", False)),
            "local_sensor_attack_branch_types": "|".join(branch_types),
            "local_sensor_attack_gps_type": gps_type,
            "local_sensor_attack_remaining_steps": max(remaining_steps)
            if remaining_steps
            else 0,
            "local_sensor_attack_intensity": float(
                current_intensity.get("overall", 0.0)
            ),
        }

    def _log_info(self, message: str):
        if self.logger and hasattr(self.logger, "logger"):
            self.logger.logger.info(message)

    def _log_warning(self, message: str):
        if self.logger and hasattr(self.logger, "log_warning"):
            self.logger.log_warning(message)

    def _log_error(self, message: str, exc: Optional[Exception] = None):
        if self.logger and hasattr(self.logger, "log_error"):
            self.logger.log_error(message, exc)

    def _make_heading_fusion(
        self, initial_pose: Optional[np.ndarray] = None
    ) -> QCarHeadingFusion:
        return QCarHeadingFusion(
            initial_pose=initial_pose,
            config=QCarHeadingFusionConfig(
                wheelbase=float(self.model_cfg.kin_wheelbase),
                q_kf_theta=float(self.heading_filter_q_psi),
                q_kf_bias=float(self.heading_filter_q_bias),
                r_kf_gps_theta=float(self.heading_filter_r_gps),
            ),
        )

    def _reset_heading_filter(self, initial_pose: Optional[np.ndarray] = None) -> None:
        pose0 = np.zeros(3, dtype=np.float64)
        if self.internal_state.size > 0:
            pose0[: min(self.internal_state.size, 3)] = self.internal_state[:3]
        if initial_pose is not None:
            init = np.asarray(initial_pose, dtype=np.float64).reshape(-1)
            pose0[: min(init.size, 3)] = init[:3]
        self.heading_fusion = self._make_heading_fusion(pose0)
        self.last_filtered_heading = wrap_angle_scalar(float(pose0[2]))

    def _ensure_heading_filter_initialized(
        self, gps_data: Optional[Dict[str, Any]] = None
    ) -> None:
        if self.heading_fusion is not None:
            return

        pose0 = np.zeros(3, dtype=np.float64)
        pose0[: min(self.internal_state.size, 3)] = self.internal_state[:3]
        if gps_position_is_valid(gps_data):
            pose0[0] = float(gps_data.get("x", pose0[0]))
            pose0[1] = float(gps_data.get("y", pose0[1]))
            pose0[2] = float(gps_data.get("theta", pose0[2]))
        self.heading_fusion = self._make_heading_fusion(pose0)
        self.last_filtered_heading = wrap_angle_scalar(float(pose0[2]))

    def _update_heading_filter(
        self,
        dt: float,
        motor_tach: float,
        steering: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]] = None,
    ) -> float:
        if not self.heading_filter_enabled:
            if gps_position_is_valid(gps_data):
                self.last_filtered_heading = wrap_angle_scalar(
                    float(gps_data.get("theta", self.internal_state[2]))
                )
            else:
                self.last_filtered_heading = wrap_angle_scalar(
                    float(self.internal_state[2]) + float(gyro_z) * float(dt)
                )
            return self.last_filtered_heading

        self._ensure_heading_filter_initialized(gps_data)

        gps_pose = None
        if gps_position_is_valid(gps_data):
            gps_pose = np.array(
                [
                    float(gps_data.get("x", self.internal_state[0])),
                    float(gps_data.get("y", self.internal_state[1])),
                    float(gps_data.get("theta", self.internal_state[2])),
                ],
                dtype=np.float64,
            )
        pose = self.heading_fusion.update(
            velocity=float(motor_tach),
            steering=float(steering),
            dt=max(float(dt), 1e-3),
            gps_pose=gps_pose,
            gyro_z=float(gyro_z),
        )
        self.last_filtered_heading = wrap_angle_scalar(float(pose[2, 0]))
        return self.last_filtered_heading

    def _apply_heading_filter_output(
        self,
        estimate: np.ndarray,
        gps_data: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        estimate = np.asarray(estimate, dtype=np.float64).copy()
        estimate[2] = wrap_angle_scalar(float(estimate[2]))
        gps_valid = gps_position_is_valid(gps_data)
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
        if self.model_path is None:
            raise ValueError(
                "Robust KalmanNet requires 'model_path' in the estimator config"
            )
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"Robust KalmanNet checkpoint not found: {self.model_path}"
            )

        checkpoint = torch.load(self.model_path, map_location=self.device)
        self._warn_if_checkpoint_config_mismatch(checkpoint)
        state_dict = self._extract_state_dict(checkpoint)
        try:
            self.model.load_state_dict(state_dict, strict=False)
        except RuntimeError as exc:
            raise RuntimeError(
                "Robust KalmanNet checkpoint is not compatible with the current "
                "4D [x, y, theta, v] architecture. Retrain the model after "
                "removing the yaw-rate state, or point model_path to a matching "
                "4D checkpoint."
            ) from exc
        self.model.eval()
        self._log_info(
            f"Robust KalmanNet checkpoint loaded from {self.model_path}"
        )

    def _build_ekf_comparator_config(self) -> Dict[str, Any]:
        comparator_cfg = dict(self.config.get("ekf_comparator_config", {}))
        comparator_cfg.setdefault(
            "use_qcar_ekf",
            bool(self.config.get("comparator_use_qcar_ekf", True)),
        )
        for key in (
            "longitudinal_model",
            "use_tachometer_update",
            "velocity_lag_model",
            "velocity_lag_lookup_model",
            "accel_lag_model",
            "coupled_kinematic_model",
            "max_velocity",
            "max_acceleration",
            "velocity_command_tau",
        ):
            if key in self.config:
                comparator_cfg.setdefault(key, copy.deepcopy(self.config.get(key)))
        if "comparator_v_lpf_alpha" in self.config:
            comparator_cfg.setdefault(
                "v_lpf_alpha", float(self.config["comparator_v_lpf_alpha"])
            )
        return comparator_cfg

    def _build_clean_reference_comparator_config(self) -> Dict[str, Any]:
        comparator_cfg = self._build_ekf_comparator_config()
        override_cfg = dict(self.config.get("clean_reference_comparator_config", {}))
        comparator_cfg.update(override_cfg)
        comparator_cfg.setdefault(
            "use_qcar_ekf",
            bool(
                self.config.get(
                    "clean_reference_comparator_use_qcar_ekf",
                    self.config.get("comparator_use_qcar_ekf", True),
                )
            ),
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
        if not self.enable_clean_reference_comparator:
            return

        try:
            module = importlib.import_module("Observer.local_state_estimators")
            comparator_cls = getattr(module, "EKFStateEstimator")
            comparator_cfg = self._build_clean_reference_comparator_config()
            self.clean_reference_comparator = comparator_cls(
                initial_pose=initial_pose,
                config=comparator_cfg,
                logger=self.logger,
            )
            self.clean_reference_comparator_ready = True
            self._log_info(
                "[RKNetComparator] Clean-reference EKF comparator enabled"
            )
        except Exception as exc:
            self.clean_reference_comparator = None
            self.clean_reference_comparator_ready = False
            self._log_error(
                "Failed to initialize clean-reference EKF comparator for Robust KalmanNet",
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
                    "gps_hold_valid",
                    "gps_age_sec",
                    "sensor_failure_active",
                    "sensor_failure_branches",
                    "sensor_failure_branch_types",
                    "sensor_failure_gps_type",
                    "sensor_failure_remaining_steps",
                    "sensor_failure_intensity",
                    "sensor_failure_imu_intensity",
                    "sensor_failure_steer_intensity",
                    "sensor_failure_wheel_intensity",
                    "sensor_failure_gps_intensity",
                    "sensor_failure_gps_xy_intensity",
                    "sensor_failure_gps_valid_flip",
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
                    "ref_x",
                    "ref_y",
                    "ref_theta",
                    "ref_v",
                    "delta_x",
                    "delta_y",
                    "delta_theta",
                    "delta_v",
                    "position_error_norm",
                    "heading_error",
                    "velocity_error",
                    "robust_ref_dx",
                    "robust_ref_dy",
                    "robust_ref_dtheta",
                    "robust_ref_dv",
                    "robust_ref_position_error_norm",
                    "robust_ref_heading_error",
                    "robust_ref_velocity_error",
                    "ekf_ref_dx",
                    "ekf_ref_dy",
                    "ekf_ref_dtheta",
                    "ekf_ref_dv",
                    "ekf_ref_position_error_norm",
                    "ekf_ref_heading_error",
                    "ekf_ref_velocity_error",
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
                    "ekf_K_norm",
                    "ekf_K_x_x",
                    "ekf_K_y_y",
                    "ekf_K_psi_psi",
                    "ekf_K_v_v",
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
        gps_valid = bool(
            gps_data
            and gps_data.get("position_valid", gps_data.get("fresh", gps_data.get("valid", False)))
        )
        gps_hold_valid = bool(gps_data and gps_data.get("hold_valid", gps_data.get("valid", False)))
        gps_age_sec = float(gps_data.get("age_sec", np.nan)) if gps_data else np.nan
        robust_state = comparison["robust_state"]
        ekf_state = comparison["ekf_state"]
        ref_state = comparison.get("reference_state")
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
                    "gps_hold_valid": int(gps_hold_valid),
                    "gps_age_sec": float(gps_age_sec),
                    "sensor_failure_active": int(comparison.get("sensor_failure_active", 0)),
                    "sensor_failure_branches": str(comparison.get("sensor_failure_branches", "")),
                    "sensor_failure_branch_types": str(comparison.get("sensor_failure_branch_types", "")),
                    "sensor_failure_gps_type": str(comparison.get("sensor_failure_gps_type", "")),
                    "sensor_failure_remaining_steps": int(comparison.get("sensor_failure_remaining_steps", 0)),
                    "sensor_failure_intensity": float(comparison.get("sensor_failure_intensity", 0.0)),
                    "sensor_failure_imu_intensity": float(comparison.get("sensor_failure_imu_intensity", 0.0)),
                    "sensor_failure_steer_intensity": float(comparison.get("sensor_failure_steer_intensity", 0.0)),
                    "sensor_failure_wheel_intensity": float(comparison.get("sensor_failure_wheel_intensity", 0.0)),
                    "sensor_failure_gps_intensity": float(comparison.get("sensor_failure_gps_intensity", 0.0)),
                    "sensor_failure_gps_xy_intensity": float(comparison.get("sensor_failure_gps_xy_intensity", 0.0)),
                    "sensor_failure_gps_valid_flip": float(comparison.get("sensor_failure_gps_valid_flip", 0.0)),
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
                    "ref_x": float(ref_state[0]) if ref_state is not None else np.nan,
                    "ref_y": float(ref_state[1]) if ref_state is not None else np.nan,
                    "ref_theta": float(ref_state[2]) if ref_state is not None else np.nan,
                    "ref_v": float(ref_state[3]) if ref_state is not None else np.nan,
                    "delta_x": float(delta_state[0]),
                    "delta_y": float(delta_state[1]),
                    "delta_theta": float(delta_state[2]),
                    "delta_v": float(delta_state[3]),
                    "position_error_norm": float(comparison["position_error_norm"]),
                    "heading_error": float(comparison["heading_error"]),
                    "velocity_error": float(comparison["velocity_error"]),
                    "robust_ref_dx": float(comparison.get("robust_ref_delta_x", np.nan)),
                    "robust_ref_dy": float(comparison.get("robust_ref_delta_y", np.nan)),
                    "robust_ref_dtheta": float(comparison.get("robust_ref_delta_theta", np.nan)),
                    "robust_ref_dv": float(comparison.get("robust_ref_delta_v", np.nan)),
                    "robust_ref_position_error_norm": float(
                        comparison.get("robust_ref_position_error_norm", np.nan)
                    ),
                    "robust_ref_heading_error": float(
                        comparison.get("robust_ref_heading_error", np.nan)
                    ),
                    "robust_ref_velocity_error": float(
                        comparison.get("robust_ref_velocity_error", np.nan)
                    ),
                    "ekf_ref_dx": float(comparison.get("ekf_ref_delta_x", np.nan)),
                    "ekf_ref_dy": float(comparison.get("ekf_ref_delta_y", np.nan)),
                    "ekf_ref_dtheta": float(comparison.get("ekf_ref_delta_theta", np.nan)),
                    "ekf_ref_dv": float(comparison.get("ekf_ref_delta_v", np.nan)),
                    "ekf_ref_position_error_norm": float(
                        comparison.get("ekf_ref_position_error_norm", np.nan)
                    ),
                    "ekf_ref_heading_error": float(
                        comparison.get("ekf_ref_heading_error", np.nan)
                    ),
                    "ekf_ref_velocity_error": float(
                        comparison.get("ekf_ref_velocity_error", np.nan)
                    ),
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
                    "ekf_K_norm": float(comparison.get("ekf_K_norm", np.nan)),
                    "ekf_K_x_x": float(comparison.get("ekf_K_x_x", np.nan)),
                    "ekf_K_y_y": float(comparison.get("ekf_K_y_y", np.nan)),
                    "ekf_K_psi_psi": float(comparison.get("ekf_K_psi_psi", np.nan)),
                    "ekf_K_v_v": float(comparison.get("ekf_K_v_v", np.nan)),
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

    @staticmethod
    def _comparison_state_vector(state: np.ndarray, source_name: str) -> np.ndarray:
        """Return the shared [x, y, theta, v] subset used by comparator logs."""
        vector = np.asarray(state, dtype=np.float64).reshape(-1)
        if vector.size < 4:
            raise ValueError(
                f"{source_name} state must contain at least 4 elements, got shape {vector.shape}"
            )
        return vector[:4].copy()

    @staticmethod
    def _comparison_delta_vector(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
        delta = np.asarray(lhs, dtype=np.float64).reshape(-1)[:4] - np.asarray(
            rhs, dtype=np.float64
        ).reshape(-1)[:4]
        delta[2] = wrap_angle_scalar(float(delta[2]))
        return delta

    @staticmethod
    def _gain_entry_by_label(
        gain_matrix: Optional[np.ndarray],
        measurement_labels: Tuple[str, ...],
        state_index: int,
        measurement_label: str,
    ) -> float:
        if gain_matrix is None:
            return float("nan")
        if state_index < 0 or state_index >= gain_matrix.shape[0]:
            return float("nan")
        try:
            column_index = tuple(measurement_labels).index(measurement_label)
        except ValueError:
            return float("nan")
        if column_index < 0 or column_index >= gain_matrix.shape[1]:
            return float("nan")
        return float(gain_matrix[state_index, column_index])

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
        if hidden is None:
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
        if self.model is None:
            raise RuntimeError("Robust KalmanNet model is not available for streaming inference")

        if self._stream_prev_upd_state is None:
            raise RuntimeError("Robust KalmanNet streaming state is not initialized")

        dtype = torch.float32
        measurement_arr = np.asarray(
            measurement,
            dtype=np.float32,
        ).reshape(1, 1, self.model_cfg.meas_dim)
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
            self._stream_prev_upd_state.reshape(1, 1, self.model_cfg.state_dim),
            device=self.device,
            dtype=dtype,
        )
        prev_pred_seq = torch.as_tensor(
            self._stream_prev_pred_state.reshape(1, 1, self.model_cfg.state_dim),
            device=self.device,
            dtype=dtype,
        )
        prev_prev_upd_seq = torch.as_tensor(
            self._stream_prev_prev_upd_state.reshape(1, 1, self.model_cfg.state_dim),
            device=self.device,
            dtype=dtype,
        )
        z_prev_seq = torch.as_tensor(
            self._stream_prev_measurement.reshape(1, 1, self.model_cfg.meas_dim),
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
            control_seq=raw_t.get("throttle"),
        )
        x_upd_t, K_t, upd_hidden, meas_mask_t, _ = self.model.updater(
            x_pred_t,
            prev_state_seq,
            prev_pred_seq,
            prev_prev_upd_seq,
            z_t,
            z_prev_seq,
            raw_t.get("gps_valid"),
            raw_t.get("theta_valid"),
            raw_t.get("gps_hold_valid"),
            raw_t.get("gps_age_sec"),
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
        if history_len == 0:
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
        z_seq = torch.as_tensor(
            z_arr.reshape(1, self.sequence_length, self.model_cfg.meas_dim),
            device=self.device,
        )
        x0 = torch.as_tensor(
            z_arr[0].reshape(1, self.model_cfg.state_dim),
            device=self.device,
        )

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
        if self.model is None:
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
            self.last_meas_mask = np.full(self.model_cfg.meas_dim, np.nan)

        K_tensor = out.get("K")
        if K_tensor is not None:
            self.last_K = K_tensor[0, -1].detach().cpu().numpy().astype(np.float64)
        else:
            self.last_K = np.full(
                (self.model_cfg.state_dim, self.model_cfg.meas_dim),
                np.nan,
            )

        x_upd[2] = wrap_angle_scalar(float(x_upd[2]))
        self.last_model_output = x_upd.copy()
        return x_upd

    def _measurement_from_inputs(
        self,
        motor_tach: float,
        steering: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
        dt: float = 0.02,
    ) -> np.ndarray:
        gps_position_valid = gps_position_is_valid(gps_data)
        self.last_gps_valid = gps_position_valid
        heading_meas = self._update_heading_filter(
            dt, motor_tach, steering, gyro_z, gps_data
        )

        if gps_position_valid:
            meas_x = float(gps_data.get("x", self.internal_state[0]))
            meas_y = float(gps_data.get("y", self.internal_state[1]))
            # Remember last valid GPS position for freezing during dropout.
            self._last_gps_x = meas_x
            self._last_gps_y = meas_y
        else:
            prev_measurement = (
                np.asarray(self.measurement_history[-1], dtype=np.float32)
                if self.measurement_history
                else None
            )
            if self.gps_dropout_xy_mode == "dead_reckon" and prev_measurement is not None:
                meas_x, meas_y = dead_reckon_xy(
                    float(prev_measurement[0]),
                    float(prev_measurement[1]),
                    heading=heading_meas,
                    velocity=float(motor_tach),
                    dt=dt,
                )
            # GPS unavailable/stale: freeze x/y at the last known GPS position
            # instead of dead-reckoning.  The update module already zeroes the
            # x/y innovation when gps_valid=0 (via meas_availability_seq), so
            # the frozen value does not affect the correction during dropout.
            # When GPS returns, the jump from the frozen position to the fresh
            # GPS reading creates a large dz feature that triggers a proper
            # snap-back correction — matching the training data distribution.
            elif self._last_gps_x is not None and self._last_gps_y is not None:
                meas_x = self._last_gps_x
                meas_y = self._last_gps_y
            else:
                # No GPS fix yet — use current internal state as fallback.
                meas_x = float(self.internal_state[0])
                meas_y = float(self.internal_state[1])

        return np.array(
            [meas_x, meas_y, heading_meas, float(motor_tach)],
            dtype=np.float32,
        )

    def _raw_sample_from_inputs(
        self,
        motor_tach: float,
        steering: float,
        gyro_z: float,
        throttle: float,
        acceleration: Optional[np.ndarray],
        gps_data: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, float]:
        accel = np.asarray(acceleration if acceleration is not None else np.zeros(3), dtype=np.float32)
        wheel_speed = float(motor_tach) * self.wheel_speed_scale
        gps_valid = float(gps_position_is_valid(gps_data))
        # Keep heading measurement available at runtime even when GPS position is
        # stale. z[..., 2] is built from the heading filter, so the updater can
        # fuse kinematic prediction against gyro/GPS heading continuously.
        theta_valid = 1.0
        gps_hold_valid = float(bool(gps_data is not None and gps_data.get("hold_valid", gps_data.get("valid", False))))
        gps_age_sec = float(max(0.0, gps_data.get("age_sec", 0.0))) if gps_data is not None else 0.0
        return {
            "ax": float(accel[0]) if accel.size > 0 else 0.0,
            "ay": float(accel[1]) if accel.size > 1 else 0.0,
            "wz": float(gyro_z),
            "delta": float(steering),
            "vfl": wheel_speed,
            "vfr": wheel_speed,
            "vrl": wheel_speed,
            "vrr": wheel_speed,
            "throttle": float(throttle),
            "gps_valid": gps_valid,
            "theta_valid": theta_valid,
            "gps_hold_valid": gps_hold_valid,
            "gps_age_sec": gps_age_sec,
        }

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
        clean_motor_tach: Optional[float] = None,
        clean_steering: Optional[float] = None,
        clean_throttle: Optional[float] = None,
        clean_gyro_z: Optional[float] = None,
        clean_gps_data: Optional[Dict[str, Any]] = None,
        clean_acceleration: Optional[np.ndarray] = None,
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

        ekf_full_state = np.asarray(self.ekf_comparator.get_state(), dtype=np.float64)
        robust_full_state = np.asarray(robust_estimate, dtype=np.float64)
        ekf_state = self._comparison_state_vector(ekf_full_state, "EKF comparator")
        robust_state = self._comparison_state_vector(
            robust_full_state, "Robust KalmanNet"
        )
        if hasattr(self.ekf_comparator, "get_last_gain_matrix"):
            ekf_gain_matrix = self.ekf_comparator.get_last_gain_matrix()
        else:
            ekf_gain_matrix = None
        if hasattr(self.ekf_comparator, "get_last_gain_measurement_labels"):
            ekf_gain_labels = tuple(self.ekf_comparator.get_last_gain_measurement_labels())
        else:
            ekf_gain_labels = ()
        delta_state = self._comparison_delta_vector(robust_state, ekf_state)

        reference_state = None
        reference_full_state = None
        if (
            self.clean_reference_comparator_ready
            and self.clean_reference_comparator is not None
            and clean_motor_tach is not None
            and clean_steering is not None
            and clean_throttle is not None
            and clean_gyro_z is not None
        ):
            ref_ok = self.clean_reference_comparator.update(
                motor_tach=clean_motor_tach,
                steering=clean_steering,
                throttle=clean_throttle,
                dt=dt,
                gyro_z=clean_gyro_z,
                gps_data=clean_gps_data,
                acceleration=clean_acceleration,
            )
            if not ref_ok:
                self._log_warning("[RKNetComparator] Clean-reference EKF comparator update failed")
            else:
                reference_full_state = np.asarray(
                    self.clean_reference_comparator.get_state(), dtype=np.float64
                )
                reference_state = self._comparison_state_vector(
                    reference_full_state, "Clean-reference EKF comparator"
                )

        if self.last_x_pred is not None and measurement is not None:
            innovation = measurement - self.last_x_pred
            innovation[2] = wrap_angle_scalar(float(innovation[2]))
        else:
            innovation = np.full(self.model_cfg.meas_dim, np.nan)

        if reference_state is not None:
            robust_ref_delta = self._comparison_delta_vector(robust_state, reference_state)
            ekf_ref_delta = self._comparison_delta_vector(ekf_state, reference_state)
        else:
            robust_ref_delta = np.full(4, np.nan, dtype=np.float64)
            ekf_ref_delta = np.full(4, np.nan, dtype=np.float64)

        comparison = {
            "tick": self.update_count,
            "timestamp": time.time(),
            "robust_source": "model",
            "robust_state": robust_state.copy(),
            "ekf_state": ekf_state.copy(),
            "reference_state": reference_state.copy() if reference_state is not None else None,
            "delta_state": delta_state.copy(),
            "position_error_norm": float(np.linalg.norm(delta_state[:2])),
            "heading_error": float(delta_state[2]),
            "velocity_error": float(delta_state[3]),
            "robust_ref_delta_x": float(robust_ref_delta[0]),
            "robust_ref_delta_y": float(robust_ref_delta[1]),
            "robust_ref_delta_theta": float(robust_ref_delta[2]),
            "robust_ref_delta_v": float(robust_ref_delta[3]),
            "robust_ref_position_error_norm": float(np.linalg.norm(robust_ref_delta[:2])),
            "robust_ref_heading_error": float(robust_ref_delta[2]),
            "robust_ref_velocity_error": float(robust_ref_delta[3]),
            "ekf_ref_delta_x": float(ekf_ref_delta[0]),
            "ekf_ref_delta_y": float(ekf_ref_delta[1]),
            "ekf_ref_delta_theta": float(ekf_ref_delta[2]),
            "ekf_ref_delta_v": float(ekf_ref_delta[3]),
            "ekf_ref_position_error_norm": float(np.linalg.norm(ekf_ref_delta[:2])),
            "ekf_ref_heading_error": float(ekf_ref_delta[2]),
            "ekf_ref_velocity_error": float(ekf_ref_delta[3]),
            "meas_mask_x": self.last_meas_mask[0] if self.last_meas_mask is not None else np.nan,
            "meas_mask_y": self.last_meas_mask[1] if self.last_meas_mask is not None else np.nan,
            "meas_mask_psi": self.last_meas_mask[2] if self.last_meas_mask is not None else np.nan,
            "meas_mask_v": self.last_meas_mask[3] if self.last_meas_mask is not None else np.nan,
            "meas_mask_w": self.last_meas_mask[4]
            if self.last_meas_mask is not None and self.last_meas_mask.size > 4
            else np.nan,
            "K_norm": float(np.linalg.norm(self.last_K)) if self.last_K is not None else np.nan,
            "K_x_x": self.last_K[0, 0] if self.last_K is not None else np.nan,
            "K_y_y": self.last_K[1, 1] if self.last_K is not None else np.nan,
            "K_psi_psi": self.last_K[2, 2] if self.last_K is not None else np.nan,
            "K_v_v": self.last_K[3, 3] if self.last_K is not None else np.nan,
            "K_w_w": self.last_K[4, 4]
            if self.last_K is not None and self.last_K.shape[0] > 4 and self.last_K.shape[1] > 4
            else np.nan,
            "ekf_K_norm": float(np.linalg.norm(ekf_gain_matrix))
            if ekf_gain_matrix is not None
            else np.nan,
            "ekf_K_x_x": self._gain_entry_by_label(ekf_gain_matrix, ekf_gain_labels, 0, "x"),
            "ekf_K_y_y": self._gain_entry_by_label(ekf_gain_matrix, ekf_gain_labels, 1, "y"),
            "ekf_K_psi_psi": self._gain_entry_by_label(
                ekf_gain_matrix, ekf_gain_labels, 2, "theta"
            ),
            "ekf_K_v_v": self._gain_entry_by_label(ekf_gain_matrix, ekf_gain_labels, 3, "v"),
            "innov_x": innovation[0],
            "innov_y": innovation[1],
            "innov_psi": innovation[2],
            "innov_v": innovation[3],
            "innov_w": innovation[4] if innovation.size > 4 else np.nan,
            "pred_x": self.last_x_pred[0] if self.last_x_pred is not None else np.nan,
            "pred_y": self.last_x_pred[1] if self.last_x_pred is not None else np.nan,
        }
        comparison.update(self._build_sensor_failure_log_fields())
        if self.last_pred_mask_summary is not None:
            comparison.update(self.last_pred_mask_summary)
        self.last_comparator_output = ekf_full_state.copy()
        self.last_reference_output = (
            reference_full_state.copy() if reference_full_state is not None else None
        )
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
                f"robust_ref_pos={comparison['robust_ref_position_error_norm']:.3f} "
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

    def _build_sensor_failure_log_fields(self) -> Dict[str, Any]:
        metadata = self.last_sensor_failure_metadata or {}
        branch_attacks = metadata.get("branch_attacks", []) or []
        gps_attack = metadata.get("gps_attack")

        branch_names: List[str] = []
        branch_types: List[str] = []
        remaining_steps: List[int] = []
        for item in branch_attacks:
            branch_name = str(item.get("branch", "")).strip()
            attack_type = str(item.get("attack_type", "")).strip()
            if branch_name:
                branch_names.append(branch_name)
            if branch_name and attack_type:
                branch_types.append(f"{branch_name}:{attack_type}")
            try:
                remaining_steps.append(int(item.get("remaining_steps", 0)))
            except (TypeError, ValueError):
                pass

        gps_type = ""
        if isinstance(gps_attack, dict):
            gps_type = str(gps_attack.get("attack_type", "")).strip()
            try:
                remaining_steps.append(int(gps_attack.get("remaining_steps", 0)))
            except (TypeError, ValueError):
                pass
        current_intensity = metadata.get("current_intensity", {}) or {}

        return {
            "sensor_failure_active": int(bool(metadata.get("active", False))),
            "sensor_failure_branches": "|".join(branch_names),
            "sensor_failure_branch_types": "|".join(branch_types),
            "sensor_failure_gps_type": gps_type,
            "sensor_failure_remaining_steps": max(remaining_steps) if remaining_steps else 0,
            "sensor_failure_intensity": float(current_intensity.get("overall", 0.0)),
            "sensor_failure_imu_intensity": float(current_intensity.get("imu", 0.0)),
            "sensor_failure_steer_intensity": float(current_intensity.get("steer", 0.0)),
            "sensor_failure_wheel_intensity": float(current_intensity.get("wheel", 0.0)),
            "sensor_failure_gps_intensity": float(current_intensity.get("gps_total", 0.0)),
            "sensor_failure_gps_xy_intensity": float(current_intensity.get("gps_xy", 0.0)),
            "sensor_failure_gps_valid_flip": float(current_intensity.get("gps_valid_flip", 0.0)),
        }

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
            clean_motor_tach = float(motor_tach)
            clean_steering = float(steering)
            clean_throttle = float(throttle)
            clean_gyro_z = float(gyro_z)
            clean_gps_payload = (
                copy.deepcopy(gps_data) if isinstance(gps_data, dict) else gps_data
            )
            clean_acceleration_payload = (
                np.asarray(acceleration, dtype=np.float64).copy()
                if acceleration is not None
                else None
            )
            gps_payload = copy.deepcopy(gps_data) if isinstance(gps_data, dict) else gps_data
            acceleration_payload = acceleration
            if self.sensor_failure_simulator is not None:
                (
                    motor_tach,
                    steering,
                    gyro_z,
                    throttle,
                    acceleration_payload,
                    gps_payload,
                    self.last_sensor_failure_metadata,
                ) = self.sensor_failure_simulator.apply(
                    motor_tach=float(motor_tach),
                    steering=float(steering),
                    gyro_z=float(gyro_z),
                    throttle=float(throttle),
                    dt=dt,
                    acceleration=acceleration,
                    gps_data=gps_payload,
                )
            else:
                self.last_sensor_failure_metadata = None
            measurement = self._measurement_from_inputs(
                motor_tach,
                steering,
                gyro_z,
                gps_payload,
                dt=dt,
            )
            raw_sample = self._raw_sample_from_inputs(
                motor_tach,
                steering,
                gyro_z,
                throttle,
                acceleration_payload,
                gps_payload,
            )
            self._append_sample(raw_sample, measurement, dt)

            try:
                estimate = self._predict_with_model()
            except Exception:
                self._clear_stream_state()
                raise

            if estimate is None:
                raise RuntimeError("Robust KalmanNet did not produce an estimate")

            estimate = self._apply_heading_filter_output(
                np.asarray(estimate, dtype=np.float64),
                gps_data=gps_payload,
            )

            self._record_comparison(
                robust_estimate=estimate,
                motor_tach=motor_tach,
                steering=steering,
                throttle=throttle,
                dt=dt,
                gyro_z=gyro_z,
                gps_data=gps_payload,
                acceleration=acceleration_payload,
                measurement=measurement,
                clean_motor_tach=clean_motor_tach,
                clean_steering=clean_steering,
                clean_throttle=clean_throttle,
                clean_gyro_z=clean_gyro_z,
                clean_gps_data=clean_gps_payload,
                clean_acceleration=clean_acceleration_payload,
            )
            self.internal_state = estimate.astype(np.float32)
            published_state = self.internal_state[:4].astype(np.float64, copy=True)
            if (
                self.publish_clean_reference_output
                and self.last_reference_output is not None
                and np.asarray(self.last_reference_output).size >= 4
            ):
                published_state = np.asarray(
                    self.last_reference_output[:4], dtype=np.float64
                ).copy()
            self.state = published_state
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
        self.internal_state = np.zeros(self.model_cfg.state_dim, dtype=np.float32)
        if initial_pose is not None:
            self.internal_state[0] = float(initial_pose[0])
            self.internal_state[1] = float(initial_pose[1])
            if len(initial_pose) > 2:
                self.internal_state[2] = float(initial_pose[2])
        self.state = self.internal_state[:4].astype(np.float64, copy=True)
        self._last_gps_x = None
        self._last_gps_y = None
        self._reset_heading_filter(initial_pose)
        self.last_model_output = None
        self.last_pred_mask = None
        self.last_pred_mask_summary = None
        self.last_meas_mask = None
        self.last_K = None
        self.last_x_pred = None
        self.update_count = 0
        self.last_comparator_output = None
        self.last_reference_output = None
        self.last_comparison = None
        self.last_sensor_failure_metadata = None
        self.comparison_history.clear()
        if self.sensor_failure_simulator is not None:
            self.sensor_failure_simulator.reset()
        if self.comparator_log_file is not None:
            try:
                self.comparator_log_file.flush()
            except Exception:
                pass
        if self.ekf_comparator is not None:
            self.ekf_comparator.reset(initial_pose=initial_pose)
        if self.clean_reference_comparator is not None:
            self.clean_reference_comparator.reset(initial_pose=initial_pose)

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
    pred,target: [B,T,m]
    weights: [m] or None
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
    attacked_weight: float = 3.0,
) -> torch.Tensor:
    """
    Supervised loss encouraging the measurement mask to suppress
    corrupted measurement channels in the innovation.

    Uses BCEWithLogitsLoss (sigmoid + BCE fused) for numerical stability —
    avoids the CUDA assertion 'input_val >= zero && input_val <= one'
    that occurs when NaN/Inf values propagate through a standalone sigmoid.

    Args:
        meas_mask_logits: [B, T, n] — raw logits (pre-sigmoid) for the
                          per-channel mask on measurements
        meas_attack_labels: [B, T, n] or legacy [B, n] — 1 if that
                          measurement channel was attacked

    Returns:
        Scalar loss
    """
    if meas_attack_labels.dim() == 2:
        labels = meas_attack_labels.unsqueeze(1).expand_as(meas_mask_logits)
    elif meas_attack_labels.dim() == 3:
        labels = meas_attack_labels.expand_as(meas_mask_logits)
    else:
        raise ValueError(
            f"Expected meas_attack_labels with shape [B, n] or [B, T, n], "
            f"got {tuple(meas_attack_labels.shape)}"
        )

    # Target: 1 for clean channels, 0 for attacked channels. Attacked
    # timesteps are rarer, so weight them higher to avoid all-pass masks.
    target = 1.0 - labels
    element_loss = torch.nn.functional.binary_cross_entropy_with_logits(
        meas_mask_logits,
        target,
        reduction="none",
    )
    attack_weight = max(float(attacked_weight), 1.0)
    weights = torch.where(labels > 0.5, attack_weight, 1.0)
    return (element_loss * weights).sum() / weights.sum().clamp_min(1.0)


def temporal_smoothness_loss(tensor: Optional[torch.Tensor]) -> torch.Tensor:
    """Penalize fast timestep-to-timestep changes for sequence outputs."""
    if tensor is None:
        raise ValueError("temporal_smoothness_loss received None")
    if tensor.dim() < 3 or tensor.shape[1] < 2:
        return tensor.new_zeros(())
    diff = tensor[:, 1:, ...] - tensor[:, :-1, ...]
    return (diff ** 2).mean()


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
    lambda_gain_smooth: float = 0.0,
    lambda_meas_mask_smooth: float = 0.0,
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
        if meas_mask is not None:
            labels = (
                meas_attack_labels.unsqueeze(1).expand_as(meas_mask)
                if meas_attack_labels.dim() == 2
                else meas_attack_labels.expand_as(meas_mask)
            )
            attacked = labels > 0.5
            clean = ~attacked
            if attacked.any():
                logs["meas_mask_attacked_mean"] = meas_mask.detach()[attacked].mean().item()
            if clean.any():
                logs["meas_mask_clean_mean"] = meas_mask.detach()[clean].mean().item()

    # FIX-E: Gain regularization — penalize large Kalman gains to encourage
    # conservative corrections and prevent GPS snap at test time.
    if K is not None and lambda_gain > 0:
        loss_gain = (K ** 2).mean()
        total = total + lambda_gain * loss_gain
        logs["loss_gain"] = loss_gain.item()

    if K is not None and lambda_gain_smooth > 0:
        loss_gain_smooth = temporal_smoothness_loss(K)
        total = total + lambda_gain_smooth * loss_gain_smooth
        logs["loss_gain_smooth"] = loss_gain_smooth.item()

    if meas_mask is not None and lambda_meas_mask_smooth > 0:
        loss_meas_mask_smooth = temporal_smoothness_loss(meas_mask)
        total = total + lambda_meas_mask_smooth * loss_meas_mask_smooth
        logs["loss_meas_mask_smooth"] = loss_meas_mask_smooth.item()

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
        "throttle": torch.randn(B, T, 1, device=device) * 0.05,
        "gps_valid": torch.zeros(B, T, 1, device=device),
        "theta_valid": torch.zeros(B, T, 1, device=device),
        "gps_hold_valid": torch.zeros(B, T, 1, device=device),
        "gps_age_sec": torch.zeros(B, T, 1, device=device),
    }

    z_seq = torch.randn(B, T, 4, device=device)
    x_gt = torch.randn(B, T, 4, device=device)
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
    state_weights = torch.tensor([1.0, 1.0, 2.0, 1.0], device=device)

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
