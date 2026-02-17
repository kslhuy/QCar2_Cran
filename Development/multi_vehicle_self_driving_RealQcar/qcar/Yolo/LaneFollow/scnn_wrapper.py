import os
import sys
import time
from typing import Dict

import cv2
import numpy as np
import torch

# Add pytorch_auto_drive to sys.path
# This file is in qcar/Yolo/LaneFollow/
# We need to go up 4 levels to get to Development, then into pytorch_auto_drive
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.normpath(os.path.join(CURRENT_DIR, "../../../../"))
PYTORCH_AUTO_DRIVE_DIR = os.path.join(ROOT_DIR, "pytorch_auto_drive")

if PYTORCH_AUTO_DRIVE_DIR not in sys.path:
    sys.path.append(PYTORCH_AUTO_DRIVE_DIR)

try:
    from utils.args import read_config
    from utils.models import MODELS
except ImportError:
    print(f"Error: Could not import modules from {PYTORCH_AUTO_DRIVE_DIR}")
    print("Please ensure pytorch_auto_drive is cloned correctly.")
    raise


class SCNNWrapper:
    def __init__(
        self,
        config_path=None,
        weight_path=None,
        strict_weights=True,
        temporal_alpha=0.6,
        lane_exist_threshold=0.5,
        lane_prob_threshold=None,
        use_half=True,
        use_cuda_benchmark=True,
        morph_open_iterations=0,
        morph_close_iterations=1,
        morph_dilate_iterations=1,
        temporal_binary_threshold=110,
        input_size_override=None,
        disable_lane_head=False,
    ):
        if config_path is None:
            config_path = os.path.join(
                PYTORCH_AUTO_DRIVE_DIR,
                "configs/lane_detection/scnn/resnet18_tusimple.py",
            )

        self.cfg = read_config(config_path)
        self.test_cfg = self.cfg.get("test", {})
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.use_half = bool(use_half and self.device.type == "cuda")
        self.disable_lane_head = bool(disable_lane_head)
        self.model_name = str(
            self.test_cfg.get(
                "exp_name",
                self.cfg.get("train", {}).get(
                    "exp_name", self.cfg.get("model", {}).get("name", "lane_model")
                ),
            )
        )

        if use_cuda_benchmark and self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True

        # We always load a lane checkpoint below, so disable extra preloading from config.
        model_cfg = self.cfg.get("model", {})
        if isinstance(model_cfg, dict):
            if self.disable_lane_head and model_cfg.get("lane_classifier_cfg") is not None:
                model_cfg["lane_classifier_cfg"] = None
                print("[LaneModel] Disabled lane existence head for speed.")

            backbone_cfg = model_cfg.get("backbone_cfg", {})
            if isinstance(backbone_cfg, dict) and backbone_cfg.get("pretrained", False):
                model_cfg["backbone_cfg"]["pretrained"] = False
                print("[LaneModel] Disabled backbone ImageNet preloading.")

            if model_cfg.get("pretrained_weights"):
                model_cfg["pretrained_weights"] = None
                print("[LaneModel] Disabled encoder preloading from pretrained_weights.")

        if input_size_override is not None:
            self.input_size_hw = tuple(int(v) for v in input_size_override)
            if len(self.input_size_hw) != 2:
                raise ValueError("input_size_override must be a 2-tuple (H, W).")
        else:
            self.input_size_hw = tuple(self.test_cfg.get("input_size", (360, 640)))  # (H, W)
        self.input_size_wh = (self.input_size_hw[1], self.input_size_hw[0])  # (W, H)
        print(f"[LaneModel] Inference input size: {self.input_size_hw[0]}x{self.input_size_hw[1]}")
        self.lane_prob_threshold = float(
            self.test_cfg.get("thresh", 0.3)
            if lane_prob_threshold is None
            else lane_prob_threshold
        )
        self.max_lane = int(self.test_cfg.get("max_lane", 5))
        self.lane_exist_threshold = float(lane_exist_threshold)
        self.temporal_alpha = float(np.clip(temporal_alpha, 0.0, 0.99))
        self.temporal_binary_threshold = int(np.clip(temporal_binary_threshold, 1, 254))
        self.morph_open_iterations = int(max(0, morph_open_iterations))
        self.morph_close_iterations = int(max(0, morph_close_iterations))
        self.morph_dilate_iterations = int(max(0, morph_dilate_iterations))

        # Preprocessing params (from test_360.py)
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        self.morph_kernel = np.ones((3, 3), np.uint8)
        self.prev_binary_smoothed = None
        self.metrics: Dict[str, float] = {}
        self.lane_exist_conf = None
        self.fps = 0.0

        # RGB colors (overlay is computed in RGB space)
        self.colors = np.array(
            [
                [0, 0, 0],  # background
                [255, 0, 0],
                [0, 255, 0],
                [0, 0, 255],
                [255, 255, 0],
                [255, 0, 255],
                [0, 255, 255],
            ],
            dtype=np.uint8,
        )

        print(f"Loading lane model from config: {config_path}")
        self.model = MODELS.from_dict(self.cfg["model"])
        self.model.to(self.device)
        self._load_weights(weight_path, strict_weights=strict_weights)
        if self.use_half:
            self.model.half()
            print("[LaneModel] Using FP16 inference on CUDA.")
        self.model.eval()

    @staticmethod
    def _strip_module_prefix(state_dict):
        if not state_dict:
            return state_dict
        sample_key = next(iter(state_dict.keys()))
        if sample_key.startswith("module."):
            return {k.replace("module.", "", 1): v for k, v in state_dict.items()}
        return state_dict

    @staticmethod
    def _apply_legacy_key_compat(state_dict):
        # Some older checkpoints store lane existence head as "aux_head.*"
        # while current configs expect "lane_classifier.*".
        has_aux_head = any(k.startswith("aux_head.") for k in state_dict.keys())
        has_lane_classifier = any(
            k.startswith("lane_classifier.") for k in state_dict.keys()
        )

        if has_aux_head and not has_lane_classifier:
            remapped = {}
            renamed = 0
            for key, value in state_dict.items():
                if key.startswith("aux_head."):
                    key = "lane_classifier." + key[len("aux_head.") :]
                    renamed += 1
                remapped[key] = value
            print(
                f"[LaneModel] Applied legacy key remap: aux_head.* -> lane_classifier.* ({renamed} tensors)"
            )
            return remapped

        return state_dict

    @staticmethod
    def _looks_like_state_dict(obj):
        if not isinstance(obj, dict) or len(obj) == 0:
            return False
        sample_key = next(iter(obj.keys()))
        sample_val = obj[sample_key]
        return isinstance(sample_key, str) and torch.is_tensor(sample_val)

    def _extract_state_dict(self, checkpoint):
        # Most common key names across training codebases
        candidate_keys = ("net", "model", "state_dict", "model_state_dict")

        if self._looks_like_state_dict(checkpoint):
            return checkpoint

        if isinstance(checkpoint, dict):
            for key in candidate_keys:
                if key in checkpoint:
                    candidate = checkpoint[key]
                    if self._looks_like_state_dict(candidate):
                        return candidate
                    if isinstance(candidate, dict):
                        for nested_key in candidate_keys:
                            nested = candidate.get(nested_key)
                            if self._looks_like_state_dict(nested):
                                return nested

        keys = list(checkpoint.keys()) if isinstance(checkpoint, dict) else []
        raise ValueError(
            f"Could not find a valid state_dict in checkpoint. Top-level keys: {keys}"
        )

    def _load_weights(self, weight_path, strict_weights=True):
        if weight_path is None:
            weight_path = os.path.join(
                PYTORCH_AUTO_DRIVE_DIR,
                "checkpoints/resnet18_scnn_tusimple/model.pt",
            )

        if not os.path.exists(weight_path):
            raise FileNotFoundError(
                f"Weight file not found: {weight_path}\n"
                "Download from pytorch-auto-drive MODEL_ZOO.md and update weight_path."
            )

        print(f"Loading weights from {weight_path}")
        checkpoint = torch.load(weight_path, map_location=self.device)
        state_dict = self._extract_state_dict(checkpoint)

        state_dict = self._strip_module_prefix(state_dict)
        state_dict = self._apply_legacy_key_compat(state_dict)
        load_info = self.model.load_state_dict(state_dict, strict=False)
        raw_missing = list(load_info.missing_keys)
        raw_unexpected = list(load_info.unexpected_keys)
        missing = list(raw_missing)
        unexpected = list(raw_unexpected)

        if self.disable_lane_head:
            missing = [k for k in missing if not k.startswith("lane_classifier.")]
            unexpected = [k for k in unexpected if not k.startswith("lane_classifier.")]
            ignored_missing = len(raw_missing) - len(missing)
            ignored_unexpected = len(raw_unexpected) - len(unexpected)
            if ignored_missing or ignored_unexpected:
                print(
                    "[LaneModel] Ignored lane-head key mismatch "
                    f"(missing={ignored_missing}, unexpected={ignored_unexpected})."
                )

        if missing or unexpected:
            print("[LaneModel] Weight mismatch detected during load_state_dict:")
            print(f"  Missing keys: {len(missing)}")
            print(f"  Unexpected keys: {len(unexpected)}")
            if missing:
                print(f"  Example missing keys: {missing[:10]}")
            if unexpected:
                print(f"  Example unexpected keys: {unexpected[:10]}")
            if strict_weights:
                raise RuntimeError(
                    "Lane checkpoint/config mismatch detected. "
                    "Use a matching config + checkpoint pair."
                )
        else:
            print("[LaneModel] Weights loaded with full key match.")

    def pre_process(self, image):
        self.original_image = image.copy()
        img = cv2.resize(image, self.input_size_wh, interpolation=cv2.INTER_LINEAR)
        img = img.astype(np.float32) / 255.0
        img = (img - self.mean) / self.std
        img = img.transpose((2, 0, 1))  # HWC -> CHW
        tensor = torch.from_numpy(img).unsqueeze(0).to(self.device)
        return tensor.half() if self.use_half else tensor.float()

    def _resolve_lane_existence(self, outputs, num_classes):
        lane_count = max(0, num_classes - 1)
        lane_exists = np.ones(lane_count, dtype=bool)
        self.lane_exist_conf = np.ones(lane_count, dtype=np.float32)

        if "lane" not in outputs or outputs["lane"] is None or lane_count == 0:
            return lane_exists

        lane_conf = outputs["lane"].sigmoid().squeeze(0).detach().cpu().numpy().astype(np.float32)
        lane_conf = np.atleast_1d(lane_conf)

        if lane_conf.size != lane_count:
            adjusted = np.ones(lane_count, dtype=np.float32)
            copy_len = min(lane_count, lane_conf.size)
            adjusted[:copy_len] = lane_conf[:copy_len]
            lane_conf = adjusted

        lane_exists = lane_conf > self.lane_exist_threshold

        # Keep top-k lanes if too many survive
        if self.max_lane > 0 and np.sum(lane_exists) > self.max_lane:
            keep_idx = np.argsort(lane_conf)[-self.max_lane :]
            pruned = np.zeros_like(lane_exists, dtype=bool)
            pruned[keep_idx] = True
            lane_exists = pruned

        self.lane_exist_conf = lane_conf
        return lane_exists

    def _build_binary_mask(self, class_map, max_prob_map, lane_exists):
        lane_mask = class_map > 0
        for lane_id, exists in enumerate(lane_exists, start=1):
            if not exists:
                lane_mask[class_map == lane_id] = False

        lane_mask &= max_prob_map >= self.lane_prob_threshold
        binary = lane_mask.astype(np.uint8) * 255

        # Morphological cleanup/tightening
        if self.morph_open_iterations > 0:
            binary = cv2.morphologyEx(
                binary,
                cv2.MORPH_OPEN,
                self.morph_kernel,
                iterations=self.morph_open_iterations,
            )
        if self.morph_close_iterations > 0:
            binary = cv2.morphologyEx(
                binary,
                cv2.MORPH_CLOSE,
                self.morph_kernel,
                iterations=self.morph_close_iterations,
            )
        if self.morph_dilate_iterations > 0:
            binary = cv2.dilate(binary, self.morph_kernel, iterations=self.morph_dilate_iterations)

        # Temporal smoothing for stability
        binary_f = binary.astype(np.float32)
        if self.prev_binary_smoothed is None:
            smoothed = binary_f
        else:
            smoothed = self.temporal_alpha * self.prev_binary_smoothed + (1.0 - self.temporal_alpha) * binary_f
        self.prev_binary_smoothed = smoothed

        return (smoothed >= float(self.temporal_binary_threshold)).astype(np.uint8) * 255

    def _build_instance_mask(self, class_map, binary_mask, lane_exists):
        instance = np.zeros((class_map.shape[0], class_map.shape[1], 3), dtype=np.uint8)
        max_lane_id = min(len(self.colors) - 1, int(class_map.max()))
        for lane_id in range(1, max_lane_id + 1):
            if (lane_id - 1) < lane_exists.size and not lane_exists[lane_id - 1]:
                continue
            lane_pixels = (class_map == lane_id) & (binary_mask > 0)
            instance[lane_pixels] = self.colors[lane_id]
        return instance

    def _update_metrics(self, binary_mask):
        h, w = binary_mask.shape
        lane_pixels = float(np.count_nonzero(binary_mask))
        lane_ratio = lane_pixels / float(h * w)

        band_h = max(1, h // 6)
        top_lane_ratio = float(np.count_nonzero(binary_mask[:band_h])) / float(band_h * w)
        bottom_lane_ratio = float(np.count_nonzero(binary_mask[-band_h:])) / float(band_h * w)

        lane_center_x = float(w / 2.0)
        bottom_nonzero = np.where(binary_mask[-band_h:] > 0)
        if bottom_nonzero[1].size > 0:
            lane_center_x = float(np.mean(bottom_nonzero[1]))

        self.metrics = {
            "lane_ratio": lane_ratio,
            "top_lane_ratio": top_lane_ratio,
            "bottom_lane_ratio": bottom_lane_ratio,
            "lane_center_x": lane_center_x,
        }

    def predict(self, image):
        start_time = time.time()
        input_tensor = self.pre_process(image)

        with torch.inference_mode():
            outputs = self.model(input_tensor)
            logits = outputs["out"]
            if tuple(logits.shape[-2:]) != self.input_size_hw:
                logits = torch.nn.functional.interpolate(
                    logits,
                    size=self.input_size_hw,
                    mode="bilinear",
                    align_corners=True,
                )
            prob_map = logits.softmax(dim=1)
            max_prob, class_idx = torch.max(prob_map, dim=1)

        class_map = class_idx.squeeze(0).cpu().numpy().astype(np.uint8)
        max_prob_map = max_prob.squeeze(0).cpu().numpy().astype(np.float32)
        lane_exists = self._resolve_lane_existence(outputs, num_classes=prob_map.shape[1])

        binary_pred = self._build_binary_mask(class_map, max_prob_map, lane_exists)
        instance_pred = self._build_instance_mask(class_map, binary_pred, lane_exists)
        self._update_metrics(binary_pred)

        # Resize back to original image size
        h_orig, w_orig = image.shape[:2]
        if (h_orig, w_orig) != self.input_size_hw:
            binary_pred = cv2.resize(binary_pred, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
            instance_pred = cv2.resize(instance_pred, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)

        self.binaryPred = binary_pred
        self.instancePred = instance_pred
        self.fps = 1.0 / max(1e-6, (time.time() - start_time))

        return binary_pred, instance_pred

    def get_metrics(self):
        return dict(self.metrics)

    def get_model_name(self):
        return self.model_name

    def set_runtime_params(
        self,
        lane_prob_threshold=None,
        lane_exist_threshold=None,
        temporal_alpha=None,
        temporal_binary_threshold=None,
        morph_open_iterations=None,
        morph_close_iterations=None,
        morph_dilate_iterations=None,
        max_lane=None,
    ):
        if lane_prob_threshold is not None:
            self.lane_prob_threshold = float(np.clip(lane_prob_threshold, 0.0, 1.0))
        if lane_exist_threshold is not None:
            self.lane_exist_threshold = float(np.clip(lane_exist_threshold, 0.0, 1.0))
        if temporal_alpha is not None:
            self.temporal_alpha = float(np.clip(temporal_alpha, 0.0, 0.99))
        if temporal_binary_threshold is not None:
            self.temporal_binary_threshold = int(np.clip(temporal_binary_threshold, 1, 254))
        if morph_open_iterations is not None:
            self.morph_open_iterations = int(max(0, morph_open_iterations))
        if morph_close_iterations is not None:
            self.morph_close_iterations = int(max(0, morph_close_iterations))
        if morph_dilate_iterations is not None:
            self.morph_dilate_iterations = int(max(0, morph_dilate_iterations))
        if max_lane is not None:
            self.max_lane = int(max(0, max_lane))

    def get_runtime_params(self):
        return {
            "lane_prob_threshold": self.lane_prob_threshold,
            "lane_exist_threshold": self.lane_exist_threshold,
            "temporal_alpha": self.temporal_alpha,
            "temporal_binary_threshold": self.temporal_binary_threshold,
            "morph_open_iterations": self.morph_open_iterations,
            "morph_close_iterations": self.morph_close_iterations,
            "morph_dilate_iterations": self.morph_dilate_iterations,
            "max_lane": self.max_lane,
        }

    def reset_temporal_filter(self):
        self.prev_binary_smoothed = None

    def render(self, showFPS=True, showMetrics=False):
        alpha = 0.5
        annotated = cv2.addWeighted(self.original_image, 1.0, self.instancePred, alpha, 0.0)

        if showFPS:
            cv2.putText(
                annotated,
                f"Lane FPS: {self.fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        if showMetrics:
            lane_ratio = self.metrics.get("lane_ratio", 0.0)
            top_ratio = self.metrics.get("top_lane_ratio", 0.0)
            cv2.putText(
                annotated,
                f"LaneRatio: {lane_ratio:.3f} Top: {top_ratio:.3f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

        return annotated
