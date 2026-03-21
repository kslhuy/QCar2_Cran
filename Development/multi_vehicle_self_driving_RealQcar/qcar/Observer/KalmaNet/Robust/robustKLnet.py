from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any, Dict, Optional, Tuple

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

    pred_hidden: int = 64
    upd_hidden: int = 64
    gain_hidden: int = 64

    pred_mlp_hidden: int = 64
    mask_hidden: int = 64

    use_hard_mask: bool = False
    dt: float = 0.02


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
        y, hidden = self.lstm(x, hidden)
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
    ) -> Tuple[torch.Tensor, torch.Tensor, Dict]:
        """
        returns:
            x_pred_seq: [B,T,5] predicted state x_{k|k-1}
            motion_seq: [B,T,5] [dxE,dyE,dpsi,v_next,w_next]
            hidden_dict
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
        return x_pred, motion, hidden_out

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
# UPDATE MODULE
# ============================================================

class UpdateMaskNet(nn.Module):
    def __init__(self, feat_dim: int, hidden_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(feat_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, feat_dim),
            nn.Sigmoid(),
        )

    def forward(self, feat: torch.Tensor) -> torch.Tensor:
        return self.net(feat)


class GainHead(nn.Module):
    def __init__(self, in_dim: int, hidden_dim: int, state_dim: int, meas_dim: int):
        super().__init__()
        self.state_dim = state_dim
        self.meas_dim = meas_dim
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, state_dim * meas_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.net(x)
        return out.view(*out.shape[:-1], self.state_dim, self.meas_dim)


class LearnedKalmanUpdate(nn.Module):
    """
    Features from paper:
    delta_x  = x_{k|k-1} - x_{k-1|k-1}
    delta_zp = z_k - H x_{k|k-1}
    delta_z  = z_k - z_{k-1}
    then masking + GRU + MLP => K
    then x_{k|k} = x_{k|k-1} + K(z_k - Hx_{k|k-1})
    """

    def __init__(self, cfg: RSNConfig):
        super().__init__()
        self.cfg = cfg
        feat_dim = cfg.state_dim + cfg.meas_dim + cfg.meas_dim
        self.mask_net = UpdateMaskNet(feat_dim, cfg.mask_hidden)
        self.gru = nn.GRU(
            input_size=feat_dim,
            hidden_size=cfg.upd_hidden,
            num_layers=1,
            batch_first=True,
        )
        self.gain_head = GainHead(
            cfg.upd_hidden,
            cfg.gain_hidden,
            cfg.state_dim,
            cfg.meas_dim,
        )

    def forward(
        self,
        x_pred_seq: torch.Tensor,
        x_prev_upd_seq: torch.Tensor,
        z_seq: torch.Tensor,
        z_prev_seq: torch.Tensor,
        hidden=None,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        device = x_pred_seq.device
        H = make_H(device=device, dtype=x_pred_seq.dtype)

        Hx_pred = torch.matmul(x_pred_seq, H.T)
        delta_x = x_pred_seq - x_prev_upd_seq
        delta_zp = z_seq - Hx_pred
        delta_z = z_seq - z_prev_seq

        feat = torch.cat([delta_x, delta_zp, delta_z], dim=-1)
        mask = self.mask_net(feat)
        feat_masked = feat * mask

        gru_out, hidden = self.gru(feat_masked, hidden)
        K_seq = self.gain_head(gru_out)

        innovation = (z_seq - Hx_pred).unsqueeze(-1)
        corr = torch.matmul(K_seq, innovation).squeeze(-1)

        x_upd = x_pred_seq + corr
        x_upd[..., 2] = wrap_angle(x_upd[..., 2])

        return x_upd, K_seq, hidden


# ============================================================
# FULL ROBUSTSTATENET
# ============================================================

class RobustStateNet(nn.Module):
    def __init__(self, cfg: RSNConfig):
        super().__init__()
        self.cfg = cfg
        self.predictor = RobustMotionPredictor(cfg)
        self.updater = LearnedKalmanUpdate(cfg)

    def build_branch_inputs(
        self,
        raw: Dict[str, torch.Tensor],
        state_for_input: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        raw keys:
            ax, ay, wz, delta, vfl, vfr, vrl, vrr
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

        x_prev_upd = x0
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
            x_pred_t, motion_t, pred_hidden_local = self.predictor(
                imu_t,
                steer_t,
                wheel_t,
                prev_state_seq,
                pred_hidden_local,
            )
            z_t = z_seq[:, t : t + 1, :]

            x_prev_upd_seq = x_prev_upd.unsqueeze(1)
            z_prev_seq = z_prev.unsqueeze(1)

            x_upd_t, K_t, upd_hidden_local = self.updater(
                x_pred_t,
                x_prev_upd_seq,
                z_t,
                z_prev_seq,
                upd_hidden_local,
            )

            x_pred_t = x_pred_t[:, 0, :]
            x_upd_t = x_upd_t[:, 0, :]
            motion_t = motion_t[:, 0, :]
            K_t = K_t[:, 0, :, :]

            x_pred_list.append(x_pred_t)
            x_upd_list.append(x_upd_t)
            motion_list.append(motion_t)
            K_list.append(K_t)

            x_prev_upd = x_upd_t
            z_prev = z_t[:, 0, :]

        return {
            "x_pred": torch.stack(x_pred_list, dim=1),
            "x_upd": torch.stack(x_upd_list, dim=1),
            "motion": torch.stack(motion_list, dim=1),
            "K": torch.stack(K_list, dim=1),
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

    RAW_KEYS = ("ax", "ay", "wz", "delta", "vfl", "vfr", "vrl", "vrr")

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
        self.use_model = bool(self.config.get("use_model", True))
        self.load_pretrained = bool(self.config.get("load_pretrained", False))
        self.allow_untrained_model = bool(self.config.get("allow_untrained_model", False))
        self.use_fallback = bool(self.config.get("use_fallback", True))

        self.device = self._resolve_device(self.config.get("device", "auto"))
        self.model_cfg = RSNConfig(
            pred_hidden=int(self.config.get("pred_hidden", 64)),
            upd_hidden=int(self.config.get("upd_hidden", 64)),
            gain_hidden=int(self.config.get("gain_hidden", 64)),
            pred_mlp_hidden=int(self.config.get("pred_mlp_hidden", 64)),
            mask_hidden=int(self.config.get("mask_hidden", 64)),
            use_hard_mask=bool(self.config.get("use_hard_mask", False)),
            dt=float(self.config.get("dt", 0.02)),
        )

        self.model = None
        if TORCH_AVAILABLE:
            self.model = RobustStateNet(self.model_cfg).to(self.device)
            self.model.eval()
        self.model_path = self._resolve_model_path(self.config.get("model_path"))
        self.model_ready = False
        self.last_update_used_model = False
        self.last_model_output = None

        self.raw_history = {
            key: deque(maxlen=self.sequence_length) for key in self.RAW_KEYS
        }
        self.measurement_history = deque(maxlen=self.sequence_length)

        self.internal_state = np.zeros(5, dtype=np.float32)
        if initial_pose is not None:
            self.internal_state[0] = float(initial_pose[0])
            self.internal_state[1] = float(initial_pose[1])
            if len(initial_pose) > 2:
                self.internal_state[2] = float(initial_pose[2])
        self.state = self.internal_state[:4].astype(np.float64, copy=True)

        self._initialize_model()

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

    def _log_info(self, message: str):
        if self.logger and hasattr(self.logger, "logger"):
            self.logger.logger.info(message)

    def _log_warning(self, message: str):
        if self.logger and hasattr(self.logger, "log_warning"):
            self.logger.log_warning(message)

    def _log_error(self, message: str, exc: Optional[Exception] = None):
        if self.logger and hasattr(self.logger, "log_error"):
            self.logger.log_error(message, exc)

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

    def _append_sample(self, raw_sample: Dict[str, float], measurement: np.ndarray) -> None:
        for key, value in raw_sample.items():
            self.raw_history[key].append(float(value))
        self.measurement_history.append(np.asarray(measurement, dtype=np.float32))

    def _build_model_inputs(self) -> Tuple[Dict[str, torch.Tensor], torch.Tensor, torch.Tensor]:
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
        return raw_tensors, z_seq, x0

    def _predict_with_model(self) -> Optional[np.ndarray]:
        if (
            not TORCH_AVAILABLE
            or self.model is None
            or not self.model_ready
            or len(self.measurement_history) < self.min_history
        ):
            return None

        raw, z_seq, x0 = self._build_model_inputs()
        with torch.no_grad():
            out = self.model(raw=raw, z_seq=z_seq, x0=x0, teacher_forcing_state=None)

        x_upd = out["x_upd"][0, -1].detach().cpu().numpy().astype(np.float64)
        if not np.all(np.isfinite(x_upd)):
            raise ValueError("Robust KalmanNet produced non-finite state")

        x_upd[2] = wrap_angle_scalar(float(x_upd[2]))
        self.last_model_output = x_upd.copy()
        return x_upd

    def _measurement_from_inputs(
        self,
        motor_tach: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
    ) -> np.ndarray:
        measurement = self.internal_state.copy().astype(np.float32)
        measurement[3] = float(motor_tach)
        measurement[4] = float(gyro_z)

        if gps_data is not None and gps_data.get("valid", False):
            measurement[0] = float(gps_data.get("x", measurement[0]))
            measurement[1] = float(gps_data.get("y", measurement[1]))
            measurement[2] = wrap_angle_scalar(float(gps_data.get("theta", measurement[2])))

        return measurement

    def _raw_sample_from_inputs(
        self,
        motor_tach: float,
        steering: float,
        gyro_z: float,
        acceleration: Optional[np.ndarray],
    ) -> Dict[str, float]:
        accel = np.asarray(acceleration if acceleration is not None else np.zeros(3), dtype=np.float32)
        wheel_speed = float(motor_tach) * self.wheel_speed_scale
        return {
            "ax": float(accel[0]) if accel.size > 0 else 0.0,
            "ay": float(accel[1]) if accel.size > 1 else 0.0,
            "wz": float(gyro_z),
            "delta": float(steering),
            "vfl": wheel_speed,
            "vfr": wheel_speed,
            "vrl": wheel_speed,
            "vrr": wheel_speed,
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
            heading_residual = wrap_angle_scalar(float(gps_data["theta"]) - float(estimate[2]))
            estimate[2] = wrap_angle_scalar(float(estimate[2]) + self.gps_heading_gain * heading_residual)

        return estimate

    def _fallback_update(
        self,
        motor_tach: float,
        dt: float,
        gyro_z: float,
        gps_data: Optional[Dict[str, Any]],
    ) -> np.ndarray:
        x, y, theta, v, _ = self.internal_state.astype(np.float64)
        v = self.velocity_lpf_alpha * float(motor_tach) + (1.0 - self.velocity_lpf_alpha) * v
        theta_pred = wrap_angle_scalar(theta + float(gyro_z) * dt)
        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt

        estimate = np.array([x_pred, y_pred, theta_pred, v, float(gyro_z)], dtype=np.float64)
        return self._post_process_estimate(estimate, motor_tach, gyro_z, gps_data)

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
            measurement = self._measurement_from_inputs(motor_tach, gyro_z, gps_data)
            raw_sample = self._raw_sample_from_inputs(motor_tach, steering, gyro_z, acceleration)
            self._append_sample(raw_sample, measurement)

            estimate = None
            self.last_update_used_model = False
            if self.model_ready:
                try:
                    estimate = self._predict_with_model()
                    if estimate is not None:
                        estimate = self._post_process_estimate(estimate, motor_tach, gyro_z, gps_data)
                        self.last_update_used_model = True
                except Exception as exc:
                    self._log_error("Robust KalmanNet inference failed", exc)
                    estimate = None

            if estimate is None:
                if not self.use_fallback:
                    return False
                estimate = self._fallback_update(motor_tach, dt, gyro_z, gps_data)

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

    def reset(self, initial_pose: Optional[np.ndarray] = None):
        self.raw_history = {
            key: deque(maxlen=self.sequence_length) for key in self.RAW_KEYS
        }
        self.measurement_history = deque(maxlen=self.sequence_length)
        self.internal_state = np.zeros(5, dtype=np.float32)
        if initial_pose is not None:
            self.internal_state[0] = float(initial_pose[0])
            self.internal_state[1] = float(initial_pose[1])
            if len(initial_pose) > 2:
                self.internal_state[2] = float(initial_pose[2])
        self.state = self.internal_state[:4].astype(np.float64, copy=True)
        self.last_model_output = None
        self.last_update_used_model = False


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
    err[..., 2] = wrap_angle(err[..., 2])
    if weights is not None:
        err = err * weights.view(1, 1, -1)
    return (err ** 2).mean()



def robuststatenet_loss(
    x_pred: torch.Tensor,
    x_upd: torch.Tensor,
    x_gt: torch.Tensor,
    lambda_upd: float = 0.8,
    lambda_pred: float = 0.2,
    weights: Optional[torch.Tensor] = None,
):
    """
    Paper-style combined loss:
    L = lambda1 ||y_hat - y_u||^2 + lambda2 ||y_hat - y_p||^2
    """
    loss_upd = weighted_state_mse(x_upd, x_gt, weights)
    loss_pred = weighted_state_mse(x_pred, x_gt, weights)
    total = lambda_upd * loss_upd + lambda_pred * loss_pred
    return total, {"loss_upd": loss_upd.item(), "loss_pred": loss_pred.item()}


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
