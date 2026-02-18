"""
Ultra-Fast-Lane-Detection-v2 Wrapper for QCar

Self-contained wrapper that embeds the parsingNet model architecture inline,
so there is NO dependency on the original Ultra-Fast-Lane-Detection-v2 repo.

Supports all model variants:
  - CULane    (res18 / res34)   — 4 lanes, 1600×320 input
  - TuSimple  (res18 / res34)   — 4 lanes, 800×320  input
  - CurveLanes(res18 / res34)   — 10 lanes, 1600×800 input

Reference: https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2
"""

import os
import time
from typing import List, Tuple, Optional, Dict, Any

import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision
import torchvision.transforms as transforms


# ===========================================================================
#  Backbone — standard ResNet feature extractor (returns 3 feature maps)
# ===========================================================================
class _ResNetBackbone(nn.Module):
    def __init__(self, layers: str, pretrained: bool = False):
        super().__init__()
        _builders = {
            "18": torchvision.models.resnet18,
            "34": torchvision.models.resnet34,
            "50": torchvision.models.resnet50,
            "101": torchvision.models.resnet101,
            "152": torchvision.models.resnet152,
        }
        if layers not in _builders:
            raise ValueError(f"Unsupported backbone: {layers}")
        model = _builders[layers](pretrained=pretrained)
        self.conv1 = model.conv1
        self.bn1 = model.bn1
        self.relu = model.relu
        self.maxpool = model.maxpool
        self.layer1 = model.layer1
        self.layer2 = model.layer2
        self.layer3 = model.layer3
        self.layer4 = model.layer4

    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)
        x = self.layer1(x)
        x2 = self.layer2(x)
        x3 = self.layer3(x2)
        x4 = self.layer4(x3)
        return x2, x3, x4


# ===========================================================================
#  ParsingNet for CULane / TuSimple
# ===========================================================================
class _ParsingNetCULane(nn.Module):
    """parsingNet architecture for CULane & TuSimple datasets."""

    def __init__(
        self,
        backbone="18",
        pretrained=False,
        num_grid_row=200,
        num_cls_row=72,
        num_grid_col=100,
        num_cls_col=81,
        num_lane_on_row=4,
        num_lane_on_col=4,
        use_aux=False,
        input_height=320,
        input_width=1600,
        fc_norm=True,
    ):
        super().__init__()
        self.num_grid_row = num_grid_row
        self.num_cls_row = num_cls_row
        self.num_grid_col = num_grid_col
        self.num_cls_col = num_cls_col
        self.num_lane_on_row = num_lane_on_row
        self.num_lane_on_col = num_lane_on_col
        self.use_aux = use_aux

        self.dim1 = num_grid_row * num_cls_row * num_lane_on_row
        self.dim2 = num_grid_col * num_cls_col * num_lane_on_col
        self.dim3 = 2 * num_cls_row * num_lane_on_row
        self.dim4 = 2 * num_cls_col * num_lane_on_col
        self.total_dim = self.dim1 + self.dim2 + self.dim3 + self.dim4

        mlp_mid_dim = 2048
        self.input_dim = (input_height // 32) * (input_width // 32) * 8

        self.model = _ResNetBackbone(backbone, pretrained=pretrained)
        self.cls = nn.Sequential(
            nn.LayerNorm(self.input_dim) if fc_norm else nn.Identity(),
            nn.Linear(self.input_dim, mlp_mid_dim),
            nn.ReLU(),
            nn.Linear(mlp_mid_dim, self.total_dim),
        )
        ch_in = 512 if backbone in ("18", "34") else 2048
        self.pool = nn.Conv2d(ch_in, 8, 1)

    def forward(self, x):
        x2, x3, fea = self.model(x)
        fea = self.pool(fea)
        fea = fea.view(-1, self.input_dim)
        out = self.cls(fea)
        return {
            "loc_row": out[:, : self.dim1].view(
                -1, self.num_grid_row, self.num_cls_row, self.num_lane_on_row
            ),
            "loc_col": out[:, self.dim1 : self.dim1 + self.dim2].view(
                -1, self.num_grid_col, self.num_cls_col, self.num_lane_on_col
            ),
            "exist_row": out[
                :, self.dim1 + self.dim2 : self.dim1 + self.dim2 + self.dim3
            ].view(-1, 2, self.num_cls_row, self.num_lane_on_row),
            "exist_col": out[:, -self.dim4 :].view(
                -1, 2, self.num_cls_col, self.num_lane_on_col
            ),
        }


# ===========================================================================
#  ParsingNet for CurveLanes  (different architecture — per-lane tokens)
# ===========================================================================
class _ParsingNetCurveLanes(nn.Module):
    """parsingNet architecture for CurveLanes dataset (10 lanes, per-lane tokens)."""

    def __init__(
        self,
        backbone="18",
        pretrained=False,
        num_grid_row=200,
        num_cls_row=72,
        num_grid_col=100,
        num_cls_col=41,
        num_lane_on_row=10,
        num_lane_on_col=10,
        use_aux=False,
        input_height=800,
        input_width=1600,
    ):
        super().__init__()
        self.num_grid_row = num_grid_row
        self.num_cls_row = num_cls_row
        self.num_grid_col = num_grid_col
        self.num_cls_col = num_cls_col
        self.num_lane_on_row = num_lane_on_row
        self.num_lane_on_col = num_lane_on_col
        self.use_aux = use_aux

        self.input_height = input_height
        self.input_width = input_width

        self.dim1 = num_grid_row * num_cls_row
        self.dim2 = 2 * num_cls_row
        self.dim3 = num_grid_col * num_cls_col
        self.dim4 = 2 * num_cls_col
        self.total_dim_row = self.dim1 + self.dim2
        self.total_dim_col = self.dim3 + self.dim4

        mlp_mid_dim = 2048
        self.input_dim = (input_height // 32) * (input_width // 32) * 9

        self.model = _ResNetBackbone(backbone, pretrained=pretrained)

        self.cls_distribute = nn.Sequential(
            nn.Conv2d(512, 128, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 20, 3, padding=1),
        )
        self.cls = nn.Sequential(
            nn.LayerNorm(self.input_dim),
            nn.Linear(self.input_dim, mlp_mid_dim),
            nn.ReLU(),
        )
        self.cls_row = nn.Linear(mlp_mid_dim, self.total_dim_row)
        self.cls_col = nn.Linear(mlp_mid_dim, self.total_dim_col)

        ch_in = 512 if backbone in ("18", "34") else 2048
        self.pool = nn.Conv2d(ch_in, 8, 1)

    def forward(self, x):
        x2, x3, fea = self.model(x)
        lane_token = self.cls_distribute(fea).reshape(
            -1, 20, 1, self.input_height // 32, self.input_width // 32
        )
        fea = self.pool(fea).unsqueeze(1).repeat(1, 20, 1, 1, 1)
        fea = torch.cat([fea, lane_token], 2)
        fea = fea.view(-1, self.input_dim)

        out = self.cls(fea).reshape(-1, 20, 2048)
        out_row = self.cls_row(out[:, :10, :]).permute(0, 2, 1)
        out_col = self.cls_col(out[:, 10:, :]).permute(0, 2, 1)

        return {
            "loc_row": out_row[:, : self.dim1, :].view(
                -1, self.num_grid_row, self.num_cls_row, self.num_lane_on_row
            ),
            "loc_col": out_col[:, : self.dim3, :].view(
                -1, self.num_grid_col, self.num_cls_col, self.num_lane_on_col
            ),
            "exist_row": out_row[:, self.dim1 : self.dim1 + self.dim2, :].view(
                -1, 2, self.num_cls_row, self.num_lane_on_row
            ),
            "exist_col": out_col[:, self.dim3 : self.dim3 + self.dim4, :].view(
                -1, 2, self.num_cls_col, self.num_lane_on_col
            ),
        }


# ===========================================================================
#  Configuration Presets
# ===========================================================================

_CONFIGS: Dict[str, Dict[str, Any]] = {
    "culane_res18": dict(
        dataset="CULane",
        backbone="18",
        num_lanes=4,
        num_row=72,
        num_col=81,
        train_width=1600,
        train_height=320,
        num_cell_row=200,
        num_cell_col=100,
        fc_norm=True,
        crop_ratio=0.6,
    ),
    "culane_res34": dict(
        dataset="CULane",
        backbone="34",
        num_lanes=4,
        num_row=72,
        num_col=81,
        train_width=1600,
        train_height=320,
        num_cell_row=200,
        num_cell_col=100,
        fc_norm=True,
        crop_ratio=0.6,
    ),
    "tusimple_res18": dict(
        dataset="Tusimple",
        backbone="18",
        num_lanes=4,
        num_row=56,
        num_col=41,
        train_width=800,
        train_height=320,
        num_cell_row=100,
        num_cell_col=100,
        fc_norm=False,
        crop_ratio=0.8,
    ),
    "tusimple_res34": dict(
        dataset="Tusimple",
        backbone="34",
        num_lanes=4,
        num_row=56,
        num_col=41,
        train_width=800,
        train_height=320,
        num_cell_row=100,
        num_cell_col=100,
        fc_norm=False,
        crop_ratio=0.8,
    ),
    "curvelanes_res18": dict(
        dataset="CurveLanes",
        backbone="18",
        num_lanes=10,
        num_row=72,
        num_col=41,
        train_width=1600,
        train_height=800,
        num_cell_row=200,
        num_cell_col=100,
        fc_norm=True,
        crop_ratio=0.8,
    ),
    "curvelanes_res34": dict(
        dataset="CurveLanes",
        backbone="34",
        num_lanes=10,
        num_row=72,
        num_col=41,
        train_width=1600,
        train_height=800,
        num_cell_row=200,
        num_cell_col=100,
        fc_norm=True,
        crop_ratio=0.8,
    ),
}


def _compute_anchors(dataset: str, num_row: int, num_col: int):
    """Compute row and column anchor arrays (normalized coordinates)."""
    if dataset == "CULane":
        row_anchor = np.linspace(0.42, 1.0, num_row)
        col_anchor = np.linspace(0.0, 1.0, num_col)
    elif dataset == "Tusimple":
        row_anchor = np.linspace(160, 710, num_row) / 720.0
        col_anchor = np.linspace(0.0, 1.0, num_col)
    elif dataset == "CurveLanes":
        row_anchor = np.linspace(0.4, 1.0, num_row)
        col_anchor = np.linspace(0.0, 1.0, num_col)
    else:
        raise ValueError(f"Unknown dataset: {dataset}")
    return row_anchor, col_anchor


# ===========================================================================
#  Post-processing: convert model output → pixel coords
# ===========================================================================


def _pred2coords(
    pred: dict,
    row_anchor: np.ndarray,
    col_anchor: np.ndarray,
    local_width: int = 1,
    original_image_width: int = 640,
    original_image_height: int = 480,
) -> List[List[Tuple[int, int]]]:
    """
    Decode the model prediction dict into a list of lane coordinate lists.
    Each lane is a list of (x, y) pixel coordinates in the original image space.
    """
    batch_size, num_grid_row, num_cls_row, num_lane_row = pred["loc_row"].shape
    batch_size, num_grid_col, num_cls_col, num_lane_col = pred["loc_col"].shape

    max_indices_row = pred["loc_row"].argmax(1).cpu()
    valid_row = pred["exist_row"].argmax(1).cpu()
    max_indices_col = pred["loc_col"].argmax(1).cpu()
    valid_col = pred["exist_col"].argmax(1).cpu()

    pred_loc_row = pred["loc_row"].cpu()
    pred_loc_col = pred["loc_col"].cpu()

    coords: List[List[Tuple[int, int]]] = []

    # Row-anchored lanes (most lanes — left+right of center)
    for i in range(num_lane_row):
        tmp: List[Tuple[int, int]] = []
        if valid_row[0, :, i].sum() > num_cls_row / 2:
            for k in range(num_cls_row):
                if valid_row[0, k, i]:
                    lo = max(0, max_indices_row[0, k, i] - local_width)
                    hi = (
                        min(num_grid_row - 1, max_indices_row[0, k, i] + local_width)
                        + 1
                    )
                    all_ind = torch.arange(lo, hi)
                    out_tmp = (
                        pred_loc_row[0, all_ind, k, i].softmax(0) * all_ind.float()
                    ).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_row - 1) * original_image_width
                    tmp.append(
                        (int(out_tmp), int(row_anchor[k] * original_image_height))
                    )
        if tmp:
            coords.append(tmp)

    # Column-anchored lanes (edge lanes)
    for i in range(num_lane_col):
        tmp = []
        if valid_col[0, :, i].sum() > num_cls_col / 4:
            for k in range(num_cls_col):
                if valid_col[0, k, i]:
                    lo = max(0, max_indices_col[0, k, i] - local_width)
                    hi = (
                        min(num_grid_col - 1, max_indices_col[0, k, i] + local_width)
                        + 1
                    )
                    all_ind = torch.arange(lo, hi)
                    out_tmp = (
                        pred_loc_col[0, all_ind, k, i].softmax(0) * all_ind.float()
                    ).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_col - 1) * original_image_height
                    tmp.append(
                        (int(col_anchor[k] * original_image_width), int(out_tmp))
                    )
        if tmp:
            coords.append(tmp)

    return coords


# ===========================================================================
#  Main Wrapper Class
# ===========================================================================


class UltraFastV2Wrapper:
    """
    Self-contained wrapper for Ultra-Fast-Lane-Detection-v2 inference.

    Usage:
        wrapper = UltraFastV2Wrapper(
            model_path="curvelanes_res18.pth",
            model_type="curvelanes_res18",
        )
        coords = wrapper.predict(bgr_image)   # list of lane coord lists
        vis = wrapper.render(showFPS=True)     # annotated image
    """

    # Lane colors (BGR for OpenCV drawing)
    LANE_COLORS = [
        (0, 255, 0),  # green
        (0, 0, 255),  # red
        (255, 0, 0),  # blue
        (0, 255, 255),  # yellow
        (255, 0, 255),  # magenta
        (255, 255, 0),  # cyan
        (128, 255, 0),  # lime
        (0, 128, 255),  # orange
        (255, 128, 0),  # sky blue
        (128, 0, 255),  # purple
    ]

    def __init__(
        self,
        model_path: str,
        model_type: str = "curvelanes_res18",
        use_half: bool = True,
        use_cuda_benchmark: bool = True,
    ):
        """
        Args:
            model_path: Path to the .pth checkpoint file.
            model_type: One of the preset config names:
                        'culane_res18', 'culane_res34',
                        'tusimple_res18', 'tusimple_res34',
                        'curvelanes_res18', 'curvelanes_res34'
            use_half: Use FP16 inference on CUDA (faster).
            use_cuda_benchmark: Enable cudnn.benchmark.
        """
        if model_type not in _CONFIGS:
            raise ValueError(
                f"Unknown model_type '{model_type}'. "
                f"Choose from: {list(_CONFIGS.keys())}"
            )

        self.cfg = _CONFIGS[model_type]
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.use_half = bool(use_half and self.device.type == "cuda")

        if use_cuda_benchmark and self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True

        # Anchors
        self.row_anchor, self.col_anchor = _compute_anchors(
            self.cfg["dataset"], self.cfg["num_row"], self.cfg["num_col"]
        )

        # Image transform
        # The original repo resizes to (train_height/crop_ratio, train_width),
        # then crops the TOP to get (train_height, train_width).
        # This removes the sky portion of the image.
        self._resize_h = int(self.cfg["train_height"] / self.cfg["crop_ratio"])
        self._train_h = self.cfg["train_height"]
        self._train_w = self.cfg["train_width"]
        self.img_transform = transforms.Compose(
            [
                transforms.ToPILImage(),
                transforms.Resize((self._resize_h, self._train_w)),
                transforms.ToTensor(),
                transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ]
        )

        # Build model
        self.net = self._build_model()
        self._load_weights(model_path)

        if self.use_half:
            self.net.half()
            print("[UltraFastV2] Using FP16 inference on CUDA.")
        self.net.eval()

        # State for rendering
        self.original_image: Optional[np.ndarray] = None
        self.last_coords: List[List[Tuple[int, int]]] = []
        self.fps: float = 0.0

        print(
            f"[UltraFastV2] Loaded {model_type} | "
            f"device={self.device} | "
            f"input={self.cfg['train_width']}x{self.cfg['train_height']} | "
            f"lanes={self.cfg['num_lanes']}"
        )

    # -----------------------------------------------------------------------
    #  Model construction
    # -----------------------------------------------------------------------

    def _build_model(self) -> nn.Module:
        c = self.cfg
        if c["dataset"] == "CurveLanes":
            net = _ParsingNetCurveLanes(
                backbone=c["backbone"],
                pretrained=False,
                num_grid_row=c["num_cell_row"],
                num_cls_row=c["num_row"],
                num_grid_col=c["num_cell_col"],
                num_cls_col=c["num_col"],
                num_lane_on_row=c["num_lanes"],
                num_lane_on_col=c["num_lanes"],
                use_aux=False,
                input_height=c["train_height"],
                input_width=c["train_width"],
            )
        else:
            net = _ParsingNetCULane(
                backbone=c["backbone"],
                pretrained=False,
                num_grid_row=c["num_cell_row"],
                num_cls_row=c["num_row"],
                num_grid_col=c["num_cell_col"],
                num_cls_col=c["num_col"],
                num_lane_on_row=c["num_lanes"],
                num_lane_on_col=c["num_lanes"],
                use_aux=False,
                input_height=c["train_height"],
                input_width=c["train_width"],
                fc_norm=c.get("fc_norm", True),
            )
        return net.to(self.device)

    def _load_weights(self, model_path: str):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")

        print(f"[UltraFastV2] Loading weights from {model_path}")
        checkpoint = torch.load(model_path, map_location="cpu")

        # Extract state_dict from checkpoint wrapper
        if isinstance(checkpoint, dict) and "model" in checkpoint:
            state_dict = checkpoint["model"]
        elif isinstance(checkpoint, dict) and "state_dict" in checkpoint:
            state_dict = checkpoint["state_dict"]
        else:
            state_dict = checkpoint

        # Strip 'module.' prefix from DataParallel training
        compatible = {}
        for k, v in state_dict.items():
            new_k = k[7:] if k.startswith("module.") else k
            compatible[new_k] = v

        info = self.net.load_state_dict(compatible, strict=False)
        if info.missing_keys:
            print(f"[UltraFastV2] Warning — missing keys: {info.missing_keys[:5]}")
        if info.unexpected_keys:
            print(
                f"[UltraFastV2] Warning — unexpected keys: {info.unexpected_keys[:5]}"
            )
        print("[UltraFastV2] Weights loaded successfully.")

    # -----------------------------------------------------------------------
    #  Inference
    # -----------------------------------------------------------------------

    def predict(self, image: np.ndarray) -> List[List[Tuple[int, int]]]:
        """
        Run inference on a BGR image.

        Args:
            image: Input BGR image (H, W, 3) from OpenCV / camera.

        Returns:
            List of lanes, each lane is a list of (x, y) pixel coordinates
            in the original image coordinate space.
        """
        start_time = time.time()
        self.original_image = image.copy()
        h_orig, w_orig = image.shape[:2]

        # Convert BGR → RGB for torchvision transforms
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Apply transforms (resize to resize_h x train_w)
        input_tensor = self.img_transform(rgb)

        # Crop from the top: keep only the bottom train_h rows
        # This removes the sky, matching the original repo's LaneTestDataset
        if input_tensor.shape[1] != self._train_h:
            input_tensor = input_tensor[:, -self._train_h :, :]

        input_tensor = input_tensor.unsqueeze(0).to(self.device)
        if self.use_half:
            input_tensor = input_tensor.half()

        # Forward pass
        with torch.no_grad():
            pred = self.net(input_tensor)

        # Decode coords
        self.last_coords = _pred2coords(
            pred,
            self.row_anchor,
            self.col_anchor,
            local_width=1,
            original_image_width=w_orig,
            original_image_height=h_orig,
        )

        self.fps = 1.0 / max(1e-6, time.time() - start_time)
        return self.last_coords

    # -----------------------------------------------------------------------
    #  Visualization
    # -----------------------------------------------------------------------

    def render(
        self,
        showFPS: bool = True,
        circle_radius: int = 5,
        line_thickness: int = 2,
        draw_lines: bool = True,
    ) -> np.ndarray:
        """
        Draw detected lanes on the original image.

        Returns:
            Annotated BGR image.
        """
        if self.original_image is None:
            raise RuntimeError("Call predict() before render().")

        vis = self.original_image.copy()

        for lane_idx, lane in enumerate(self.last_coords):
            color = self.LANE_COLORS[lane_idx % len(self.LANE_COLORS)]
            # Draw filled circles at each point
            for pt in lane:
                cv2.circle(vis, pt, circle_radius, color, -1)
            # Connect points with lines for better visibility
            if draw_lines and len(lane) > 1:
                for j in range(len(lane) - 1):
                    cv2.line(vis, lane[j], lane[j + 1], color, line_thickness)

        if showFPS:
            cv2.putText(
                vis,
                f"UFLDv2 FPS: {self.fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        return vis

    # -----------------------------------------------------------------------
    #  Utility
    # -----------------------------------------------------------------------

    def get_lane_count(self) -> int:
        """Number of lanes detected in the last prediction."""
        return len(self.last_coords)

    def get_config(self) -> Dict[str, Any]:
        """Return the active configuration dict."""
        return dict(self.cfg)
