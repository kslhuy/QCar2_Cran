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
    exist_threshold: float = 0.5,
    max_lane: int = 0,
) -> Tuple[List[List[Tuple[int, int]]], List[float]]:
    """
    Decode the model prediction dict into a list of lane coordinate lists.

    Returns:
        (coords, confidences) — coords is the lane list, confidences is a
        per-lane float score derived from the exist_row/exist_col heads.
    """
    batch_size, num_grid_row, num_cls_row, num_lane_row = pred["loc_row"].shape
    batch_size, num_grid_col, num_cls_col, num_lane_col = pred["loc_col"].shape

    max_indices_row = pred["loc_row"].argmax(1).cpu()
    valid_row = pred["exist_row"].argmax(1).cpu()
    max_indices_col = pred["loc_col"].argmax(1).cpu()
    valid_col = pred["exist_col"].argmax(1).cpu()

    pred_loc_row = pred["loc_row"].cpu()
    pred_loc_col = pred["loc_col"].cpu()

    # Compute per-lane existence confidence from the exist heads
    exist_row_prob = pred["exist_row"].softmax(1)[0, 1, :, :].cpu()
    exist_col_prob = pred["exist_col"].softmax(1)[0, 1, :, :].cpu()

    coords: List[List[Tuple[int, int]]] = []
    confidences: List[float] = []

    # Row-anchored lanes
    for i in range(num_lane_row):
        lane_exist_conf = float(exist_row_prob[:, i].mean())
        if lane_exist_conf < exist_threshold:
            continue

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
            confidences.append(lane_exist_conf)

    # Column-anchored lanes
    for i in range(num_lane_col):
        lane_exist_conf = float(exist_col_prob[:, i].mean())
        if lane_exist_conf < exist_threshold:
            continue

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
            confidences.append(lane_exist_conf)

    # Max-lane pruning
    if max_lane > 0 and len(coords) > max_lane:
        ranked = sorted(zip(confidences, coords), key=lambda x: x[0], reverse=True)[
            :max_lane
        ]
        confidences = [c for c, _ in ranked]
        coords = [pts for _, pts in ranked]

    return coords, confidences


# ===========================================================================
#  Coordinate-based Center Lane Estimation
# ===========================================================================


def _interpolate_lane_x_at_y(
    lane: List[Tuple[int, int]], target_ys: np.ndarray
) -> np.ndarray:
    """
    Given a lane as (x, y) pairs, interpolate x values at the target y levels.
    Returns NaN for y values outside the lane's y range.
    """
    if len(lane) < 2:
        return np.full(len(target_ys), np.nan)

    # Sort lane points by y (top to bottom)
    pts = sorted(lane, key=lambda p: p[1])
    ys = np.array([p[1] for p in pts], dtype=np.float64)
    xs = np.array([p[0] for p in pts], dtype=np.float64)

    result = np.full(len(target_ys), np.nan)
    y_min, y_max = ys[0], ys[-1]

    for i, ty in enumerate(target_ys):
        if ty < y_min or ty > y_max:
            continue
        result[i] = np.interp(float(ty), ys, xs)

    return result


def estimate_center_from_coords(
    lanes: List[List[Tuple[int, int]]],
    image_width: int,
    image_height: int,
    previous_state: Optional[Dict] = None,
    sample_count: int = 40,
    center_smooth_alpha: float = 0.75,
    camera_y_offset_px: float = 0.0,
) -> Dict[str, Any]:
    """
    Estimate the lane center directly from lane coordinate lists.

    Strategy:
      1. Find the two lanes closest to image center (one left, one right)
      2. Interpolate both at common y-levels
      3. Center = midpoint at each y-level
      4. Fit a polynomial + temporal smoothing on coefficients

    Returns dict with: center_points, left_points, right_points,
    confidence, bottom_center_x, bottom_offset_px, lane_width_px,
    tracker_state.
    """
    previous_state = previous_state or {}
    prev_center_coeff = previous_state.get("center_coeff")
    prev_lane_width = previous_state.get("lane_width_px")
    prev_lost_frames = int(previous_state.get("lost_frames", 0))

    image_center_x = image_width / 2.0

    # Body center in image pixels, corrected for camera lateral offset.
    # camera_y_offset_px > 0 means camera is left of body center,
    # so body center is to the RIGHT in the image.
    body_center_x = image_center_x + camera_y_offset_px

    # Sample y-levels from bottom to ~40% height
    y_top = max(0, int(0.40 * image_height))
    sample_ys = np.linspace(image_height - 1, y_top, sample_count).astype(np.float64)

    empty_result = {
        "left_points": [],
        "right_points": [],
        "center_points": [],
        "confidence": 0.0,
        "bottom_center_x": None,
        "bottom_offset_px": None,
        "lane_width_px": prev_lane_width,
        "left_lane_idx": None,
        "right_lane_idx": None,
        "tracker_state": {
            "center_coeff": prev_center_coeff,
            "lane_width_px": prev_lane_width,
            "lost_frames": prev_lost_frames + 1,
        },
    }

    if not lanes or image_height < 8 or image_width < 8:
        # If we have a previous curve and haven't lost too many frames, coast
        if prev_center_coeff is not None and prev_lost_frames < 8:
            empty_result["confidence"] = max(0.0, 0.3 - 0.04 * prev_lost_frames)
            pts = _eval_poly_points(
                prev_center_coeff, sample_ys, image_height, image_width
            )
            empty_result["center_points"] = pts
            if pts:
                empty_result["bottom_center_x"] = float(pts[0][0])
                empty_result["bottom_offset_px"] = float(pts[0][0] - body_center_x)
        return empty_result

    # --- Step 1: Find bottom-x for each lane to identify left/right ---
    # Use the bottom 30% of available points for each lane to get stable bottom-x
    lane_bottom_xs = []
    for lane in lanes:
        if not lane:
            lane_bottom_xs.append(None)
            continue
        sorted_pts = sorted(lane, key=lambda p: p[1], reverse=True)  # bottom first
        n_bottom = max(1, len(sorted_pts) // 3)
        bottom_x = float(np.mean([p[0] for p in sorted_pts[:n_bottom]]))
        lane_bottom_xs.append(bottom_x)

    # --- Step 2: Pick the best left-right pair straddling the center ---
    left_idx, right_idx = None, None
    best_left_x, best_right_x = -1e9, 1e9

    for i, bx in enumerate(lane_bottom_xs):
        if bx is None:
            continue
        if bx <= image_center_x:
            # Left candidate — want the closest to center (largest x)
            if bx > best_left_x:
                best_left_x = bx
                left_idx = i
        else:
            # Right candidate — want the closest to center (smallest x)
            if bx < best_right_x:
                best_right_x = bx
                right_idx = i

    # Fallback: if only lanes on one side, use the two closest to center
    if left_idx is None and right_idx is None:
        return empty_result

    if left_idx is None or right_idx is None:
        # Only one side — we can still estimate center using lane width prior
        single_idx = left_idx if left_idx is not None else right_idx
        single_xs = _interpolate_lane_x_at_y(lanes[single_idx], sample_ys)

        offset = prev_lane_width / 2.0 if prev_lane_width else image_width * 0.15
        if left_idx is not None:
            center_xs = single_xs + offset
        else:
            center_xs = single_xs - offset

        valid = ~np.isnan(center_xs) & ~np.isnan(single_xs)
        if valid.sum() < 3:
            return empty_result

        center_pts = [
            (int(np.clip(cx, 0, image_width - 1)), int(y))
            for cx, y, v in zip(center_xs, sample_ys, valid)
            if v
        ]
        single_pts = [
            (int(np.clip(sx, 0, image_width - 1)), int(y))
            for sx, y, v in zip(single_xs, sample_ys, valid)
            if v
        ]

        # Fit and smooth
        center_coeff = _fit_poly(center_pts, image_height, degree=2)
        center_coeff = _blend_coeff(
            prev_center_coeff, center_coeff, center_smooth_alpha
        )

        if center_coeff is not None:
            center_pts = _eval_poly_points(
                center_coeff, sample_ys, image_height, image_width
            )
            bottom_x = float(
                np.polyval(center_coeff, (image_height - 1) / max(1, image_height - 1))
            )
            bottom_x = float(np.clip(bottom_x, 0, image_width - 1))
        else:
            bottom_x = None

        return {
            "left_points": single_pts if left_idx is not None else [],
            "right_points": single_pts if right_idx is not None else [],
            "center_points": center_pts,
            "confidence": 0.35 * (valid.sum() / len(sample_ys)),
            "bottom_center_x": bottom_x,
            "bottom_offset_px": None
            if bottom_x is None
            else float(bottom_x - body_center_x),
            "lane_width_px": prev_lane_width,
            "left_lane_idx": left_idx,
            "right_lane_idx": right_idx,
            "tracker_state": {
                "center_coeff": center_coeff,
                "lane_width_px": prev_lane_width,
                "lost_frames": 0,
            },
        }

    # --- Step 3: Interpolate both lanes at common y-levels ---
    left_xs = _interpolate_lane_x_at_y(lanes[left_idx], sample_ys)
    right_xs = _interpolate_lane_x_at_y(lanes[right_idx], sample_ys)

    # Only keep y-levels where BOTH lanes have valid data
    both_valid = ~np.isnan(left_xs) & ~np.isnan(right_xs)
    # Also accept rows where width is reasonable (> 10% of image, < 80%)
    widths = right_xs - left_xs
    reasonable_width = (
        both_valid & (widths > 0.06 * image_width) & (widths < 0.80 * image_width)
    )

    valid_count = int(reasonable_width.sum())
    if valid_count < 3:
        # Not enough paired rows; try single-lane fallback above
        return empty_result

    # --- Step 4: Compute center, lane width, and confidence ---
    center_xs = (left_xs + right_xs) / 2.0
    lane_widths = widths[reasonable_width]
    lane_width_px = float(np.median(lane_widths))

    # Width consistency score
    if len(lane_widths) > 1:
        width_std = float(np.std(lane_widths))
        width_consistency = 1.0 - float(
            np.clip(width_std / max(1.0, lane_width_px), 0.0, 1.0)
        )
    else:
        width_consistency = 0.6

    # Build point lists (only where both are valid)
    left_points = [
        (int(np.clip(lx, 0, image_width - 1)), int(y))
        for lx, y, v in zip(left_xs, sample_ys, reasonable_width)
        if v
    ]
    right_points = [
        (int(np.clip(rx, 0, image_width - 1)), int(y))
        for rx, y, v in zip(right_xs, sample_ys, reasonable_width)
        if v
    ]
    center_points_raw = [
        (int(np.clip(cx, 0, image_width - 1)), int(y))
        for cx, y, v in zip(center_xs, sample_ys, reasonable_width)
        if v
    ]

    # --- Step 5: Polynomial fit + temporal smoothing ---
    center_coeff = _fit_poly(center_points_raw, image_height, degree=2)
    center_coeff = _blend_coeff(prev_center_coeff, center_coeff, center_smooth_alpha)

    # Re-evaluate smooth curve at all sample points
    if center_coeff is not None:
        center_points = _eval_poly_points(
            center_coeff, sample_ys, image_height, image_width
        )
        bottom_center_x = float(
            np.polyval(center_coeff, (image_height - 1) / max(1, image_height - 1))
        )
        bottom_center_x = float(np.clip(bottom_center_x, 0, image_width - 1))
    else:
        center_points = center_points_raw
        bottom_center_x = None

    # --- Step 6: Confidence score ---
    coverage = float(valid_count) / len(sample_ys)
    confidence = (
        0.50 * coverage + 0.30 * width_consistency + 0.20 * min(1.0, valid_count / 15.0)
    )
    confidence = float(np.clip(confidence, 0.0, 1.0))

    # Smooth lane width with previous
    if prev_lane_width is not None:
        lane_width_px = 0.7 * prev_lane_width + 0.3 * lane_width_px

    return {
        "left_points": left_points,
        "right_points": right_points,
        "center_points": center_points,
        "confidence": confidence,
        "bottom_center_x": bottom_center_x,
        "bottom_offset_px": None
        if bottom_center_x is None
        else float(bottom_center_x - body_center_x),
        "lane_width_px": lane_width_px,
        "left_lane_idx": left_idx,
        "right_lane_idx": right_idx,
        "tracker_state": {
            "center_coeff": center_coeff,
            "lane_width_px": lane_width_px,
            "lost_frames": 0,
        },
    }


def _fit_poly(points: List[Tuple[int, int]], image_height: int, degree: int = 2):
    """Fit x = f(y_normalized) polynomial to points."""
    if len(points) < degree + 1:
        return None
    ys = np.array([p[1] for p in points], dtype=np.float64)
    xs = np.array([p[0] for p in points], dtype=np.float64)
    y_norm = ys / max(1.0, float(image_height - 1))
    try:
        coeff = np.polyfit(y_norm, xs, deg=degree)
        return coeff.astype(np.float64)
    except (np.linalg.LinAlgError, ValueError):
        return None


def _blend_coeff(prev, new, alpha: float):
    """EMA blend of polynomial coefficients."""
    if prev is None:
        return new
    if new is None:
        return prev.copy()
    prev = np.asarray(prev, dtype=np.float64)
    new = np.asarray(new, dtype=np.float64)
    max_len = max(prev.size, new.size)
    if prev.size < max_len:
        prev = np.pad(prev, (max_len - prev.size, 0))
    if new.size < max_len:
        new = np.pad(new, (max_len - new.size, 0))
    return alpha * prev + (1.0 - alpha) * new


def _eval_poly_points(
    coeff, sample_ys: np.ndarray, image_height: int, image_width: int
) -> List[Tuple[int, int]]:
    """Evaluate polynomial at sample y-levels, return clipped (x, y) points."""
    if coeff is None:
        return []
    y_norm = sample_ys / max(1.0, float(image_height - 1))
    xs = np.polyval(coeff, y_norm)
    pts = []
    for x, y in zip(xs, sample_ys):
        xi = int(np.clip(round(float(x)), 0, image_width - 1))
        pts.append((xi, int(y)))
    return pts


def draw_center_lane_overlay(
    image: np.ndarray,
    center_result: Dict[str, Any],
    min_confidence: float = 0.0,
):
    """
    Draw the center lane estimation result on an image.
    Left/right boundaries in orange, center in green.
    Only draws if confidence >= min_confidence.
    """
    if center_result.get("confidence", 0.0) < min_confidence:
        return

    def draw_polyline(points, color, thickness):
        if len(points) >= 2:
            pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image, [pts], False, color, thickness, cv2.LINE_AA)
        elif len(points) == 1:
            cv2.circle(image, points[0], max(1, thickness), color, -1, cv2.LINE_AA)

    draw_polyline(center_result["left_points"], (255, 170, 0), 2)
    draw_polyline(center_result["right_points"], (255, 170, 0), 2)
    draw_polyline(center_result["center_points"], (0, 255, 0), 3)

    if center_result["center_points"]:
        # Mark bottom center with a dot
        bottom_point = max(center_result["center_points"], key=lambda p: p[1])
        cv2.circle(image, bottom_point, 6, (0, 255, 0), -1, cv2.LINE_AA)


# ===========================================================================
#  Main Wrapper Class
# ===========================================================================


class UltraFastV2Wrapper:
    """
    Self-contained wrapper for Ultra-Fast-Lane-Detection-v2 inference.

    Usage:
        wrapper = UltraFastV2Wrapper(model_path=..., model_type=...)
        coords = wrapper.predict(bgr_image)   # list of lane coord lists
        vis = wrapper.render(showFPS=True)     # annotated image (clean overlay)
        center = wrapper.estimate_center()     # coordinate-based center lane
    """

    # BGR colors for cv2 drawing (clean, visible)
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
        lane_exist_threshold: float = 0.5,
        max_lane: int = 0,
        center_smooth_alpha: float = 0.75,
        camera_y_offset_px: float = 0.0,
    ):
        """
        Args:
            model_path: Path to the .pth checkpoint file.
            model_type: One of the preset config names.
            use_half: Use FP16 inference on CUDA (faster).
            use_cuda_benchmark: Enable cudnn.benchmark.
            lane_exist_threshold: Confidence threshold for lane existence.
            max_lane: Max lanes to keep (0 = unlimited).
            center_smooth_alpha: EMA alpha for center lane curve smoothing.
            camera_y_offset_px: Pixel offset to compensate for lateral
                camera-to-body-center misalignment.  Positive = camera is
                left of body center (body center is right in image).
        """
        if model_type not in _CONFIGS:
            raise ValueError(
                f"Unknown model_type '{model_type}'. "
                f"Choose from: {list(_CONFIGS.keys())}"
            )

        self.cfg = _CONFIGS[model_type]
        self.model_type_str = model_type
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.use_half = bool(use_half and self.device.type == "cuda")

        if use_cuda_benchmark and self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True

        # Anchors
        self.row_anchor, self.col_anchor = _compute_anchors(
            self.cfg["dataset"], self.cfg["num_row"], self.cfg["num_col"]
        )

        # Image transform
        self._resize_h = int(self.cfg["train_height"] / self.cfg["crop_ratio"])
        self._train_h = self.cfg["train_height"]
        self._train_w = self.cfg["train_width"]
        self.input_size_hw = (self._train_h, self._train_w)
        self.img_transform = transforms.Compose(
            [
                transforms.ToPILImage(),
                transforms.Resize((self._resize_h, self._train_w)),
                transforms.ToTensor(),
                transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ]
        )

        # Runtime-tunable parameters
        self.lane_exist_threshold = float(lane_exist_threshold)
        self.max_lane = int(max(0, max_lane))
        self.center_smooth_alpha = float(np.clip(center_smooth_alpha, 0.0, 0.99))
        self.camera_y_offset_px = float(camera_y_offset_px)

        # Build model
        self.net = self._build_model()
        self._load_weights(model_path)

        if self.use_half:
            self.net.half()
            print("[UltraFastV2] Using FP16 inference on CUDA.")
        self.net.eval()

        # State
        self.original_image: Optional[np.ndarray] = None
        self.last_coords: List[List[Tuple[int, int]]] = []
        self.last_confidences: List[float] = []
        self.center_tracker_state: Dict = {}
        self.last_center_result: Dict[str, Any] = {}
        self.fps: float = 0.0
        self._image_h: int = 0
        self._image_w: int = 0

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

        if isinstance(checkpoint, dict) and "model" in checkpoint:
            state_dict = checkpoint["model"]
        elif isinstance(checkpoint, dict) and "state_dict" in checkpoint:
            state_dict = checkpoint["state_dict"]
        else:
            state_dict = checkpoint

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
            List of lanes, each lane is a list of (x, y) pixel coordinates.
        """
        start_time = time.time()
        self.original_image = image.copy()
        h_orig, w_orig = image.shape[:2]
        self._image_h = h_orig
        self._image_w = w_orig

        # Convert BGR → RGB for torchvision transforms
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Apply transforms
        input_tensor = self.img_transform(rgb)

        # Crop from the top: keep only the bottom train_h rows
        if input_tensor.shape[1] != self._train_h:
            input_tensor = input_tensor[:, -self._train_h :, :]

        input_tensor = input_tensor.unsqueeze(0).to(self.device)
        if self.use_half:
            input_tensor = input_tensor.half()

        # Forward pass
        with torch.no_grad():
            pred = self.net(input_tensor)

        # Decode coords
        self.last_coords, self.last_confidences = _pred2coords(
            pred,
            self.row_anchor,
            self.col_anchor,
            local_width=1,
            original_image_width=w_orig,
            original_image_height=h_orig,
            exist_threshold=self.lane_exist_threshold,
            max_lane=self.max_lane,
        )

        self.fps = 1.0 / max(1e-6, time.time() - start_time)
        return self.last_coords

    # -----------------------------------------------------------------------
    #  Center lane estimation (coordinate-based, no binary mask)
    # -----------------------------------------------------------------------

    def estimate_center(self) -> Dict[str, Any]:
        """
        Estimate the center of the lane from the last predicted coordinates.
        Uses coordinate-based matching (left/right pair identification),
        polynomial fitting, and temporal smoothing.

        Returns:
            Dict with center_points, left_points, right_points,
            confidence, bottom_center_x, bottom_offset_px, lane_width_px.
        """
        result = estimate_center_from_coords(
            self.last_coords,
            self._image_w,
            self._image_h,
            previous_state=self.center_tracker_state,
            center_smooth_alpha=self.center_smooth_alpha,
            camera_y_offset_px=self.camera_y_offset_px,
        )
        self.center_tracker_state = result.get(
            "tracker_state", self.center_tracker_state
        )
        self.last_center_result = result
        return result

    def reset_center_tracker(self):
        """Clear the center lane tracker state."""
        self.center_tracker_state = {}
        self.last_center_result = {}

    # -----------------------------------------------------------------------
    #  Visualization (clean style — colored circles + lines)
    # -----------------------------------------------------------------------

    def render(
        self,
        showFPS: bool = True,
        showMetrics: bool = False,
        circle_radius: int = 5,
        line_thickness: int = 2,
        draw_lines: bool = True,
    ) -> np.ndarray:
        """
        Draw detected lanes on the original image with clean colored overlays.
        """
        if self.original_image is None:
            raise RuntimeError("Call predict() before render().")

        vis = self.original_image.copy()

        for lane_idx, lane in enumerate(self.last_coords):
            color = self.LANE_COLORS[lane_idx % len(self.LANE_COLORS)]
            # Draw filled circles at each point
            for pt in lane:
                cv2.circle(vis, pt, circle_radius, color, -1)
            # Connect points with lines
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

        if showMetrics and self.last_center_result:
            conf = self.last_center_result.get("confidence", 0.0)
            offset = self.last_center_result.get("bottom_offset_px")
            width = self.last_center_result.get("lane_width_px")
            offset_str = f"{offset:+.0f}px" if offset is not None else "n/a"
            width_str = f"{width:.0f}px" if width is not None else "n/a"
            cv2.putText(
                vis,
                f"Center: conf={conf:.2f} offset={offset_str} width={width_str}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

        return vis

    # -----------------------------------------------------------------------
    #  Runtime parameter tuning
    # -----------------------------------------------------------------------

    def set_runtime_params(
        self,
        lane_exist_threshold: Optional[float] = None,
        max_lane: Optional[int] = None,
        center_smooth_alpha: Optional[float] = None,
        camera_y_offset_px: Optional[float] = None,
    ):
        """Adjust runtime-tunable parameters."""
        if lane_exist_threshold is not None:
            self.lane_exist_threshold = float(np.clip(lane_exist_threshold, 0.0, 1.0))
        if max_lane is not None:
            self.max_lane = int(max(0, max_lane))
        if center_smooth_alpha is not None:
            self.center_smooth_alpha = float(np.clip(center_smooth_alpha, 0.0, 0.99))
        if camera_y_offset_px is not None:
            self.camera_y_offset_px = float(camera_y_offset_px)

    def get_runtime_params(self) -> Dict[str, Any]:
        """Return current runtime-tunable parameters."""
        return {
            "lane_exist_threshold": self.lane_exist_threshold,
            "max_lane": self.max_lane,
            "center_smooth_alpha": self.center_smooth_alpha,
            "camera_y_offset_px": self.camera_y_offset_px,
        }

    # -----------------------------------------------------------------------
    #  Utility
    # -----------------------------------------------------------------------

    def get_lane_count(self) -> int:
        """Number of lanes detected in the last prediction."""
        return len(self.last_coords)

    def get_lane_coords(self) -> List[List[Tuple[int, int]]]:
        """Return the raw coordinate lists from the last prediction."""
        return self.last_coords

    def get_model_name(self) -> str:
        """Return the model type/name string."""
        return self.model_type_str

    def get_config(self) -> Dict[str, Any]:
        """Return the active configuration dict."""
        return dict(self.cfg)
