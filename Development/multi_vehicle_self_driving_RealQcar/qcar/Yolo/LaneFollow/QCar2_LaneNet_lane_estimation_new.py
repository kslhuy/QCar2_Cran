import os
import numpy as np
import cv2
import time
from scnn_wrapper import SCNNWrapper
from pal.products.qcar import QCarRealSense


## Timing Parameters and methods
def elapsed_time():
    return time.time() - startTime


def nothing(_):
    pass


def safe_get_trackbar(name, window, default):
    try:
        return cv2.getTrackbarPos(name, window)
    except cv2.error:
        return default


def setup_controls_window(initial, image_height):
    window = "Lane Model Controls"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 420, 600)

    cv2.createTrackbar(
        "LaneProb x100", window, int(initial["lane_prob_threshold"] * 100), 100, nothing
    )
    cv2.createTrackbar(
        "LaneExist x100",
        window,
        int(initial["lane_exist_threshold"] * 100),
        100,
        nothing,
    )
    cv2.createTrackbar(
        "TempAlpha x100", window, int(initial["temporal_alpha"] * 100), 95, nothing
    )
    cv2.createTrackbar(
        "TempBinThr", window, int(initial["temporal_binary_threshold"]), 255, nothing
    )
    cv2.createTrackbar(
        "OpenIter", window, int(initial["morph_open_iterations"]), 5, nothing
    )
    cv2.createTrackbar(
        "CloseIter", window, int(initial["morph_close_iterations"]), 5, nothing
    )
    cv2.createTrackbar(
        "DilateIter", window, int(initial["morph_dilate_iterations"]), 5, nothing
    )

    cv2.createTrackbar("AdaptiveROI", window, int(initial["adaptive_roi"]), 1, nothing)
    cv2.createTrackbar(
        "TopCrop", window, int(initial["top_crop"]), image_height - 1, nothing
    )
    cv2.createTrackbar(
        "BottomCrop", window, int(initial["bottom_crop"]), image_height // 2, nothing
    )
    cv2.createTrackbar(
        "MinTop", window, int(initial["min_top"]), image_height - 1, nothing
    )
    cv2.createTrackbar(
        "MaxTop", window, int(initial["max_top"]), image_height - 1, nothing
    )
    cv2.createTrackbar(
        "ROIAdjPeriod", window, int(initial["roi_adj_period"]), 30, nothing
    )

    return window


def read_live_controls(window, defaults):
    return {
        "lane_prob_threshold": safe_get_trackbar(
            "LaneProb x100", window, int(defaults["lane_prob_threshold"] * 100)
        )
        / 100.0,
        "lane_exist_threshold": safe_get_trackbar(
            "LaneExist x100", window, int(defaults["lane_exist_threshold"] * 100)
        )
        / 100.0,
        "temporal_alpha": safe_get_trackbar(
            "TempAlpha x100", window, int(defaults["temporal_alpha"] * 100)
        )
        / 100.0,
        "temporal_binary_threshold": safe_get_trackbar(
            "TempBinThr", window, int(defaults["temporal_binary_threshold"])
        ),
        "morph_open_iterations": safe_get_trackbar(
            "OpenIter", window, int(defaults["morph_open_iterations"])
        ),
        "morph_close_iterations": safe_get_trackbar(
            "CloseIter", window, int(defaults["morph_close_iterations"])
        ),
        "morph_dilate_iterations": safe_get_trackbar(
            "DilateIter", window, int(defaults["morph_dilate_iterations"])
        ),
        "adaptive_roi": safe_get_trackbar(
            "AdaptiveROI", window, int(defaults["adaptive_roi"])
        )
        > 0,
        "top_crop": safe_get_trackbar("TopCrop", window, int(defaults["top_crop"])),
        "bottom_crop": safe_get_trackbar(
            "BottomCrop", window, int(defaults["bottom_crop"])
        ),
        "min_top": safe_get_trackbar("MinTop", window, int(defaults["min_top"])),
        "max_top": safe_get_trackbar("MaxTop", window, int(defaults["max_top"])),
        "roi_adj_period": max(
            1,
            safe_get_trackbar("ROIAdjPeriod", window, int(defaults["roi_adj_period"])),
        ),
    }


def _extract_row_clusters(row_mask, min_cluster_width=2, max_gap=3):
    lane_x = np.flatnonzero(row_mask > 0)
    if lane_x.size == 0:
        return []

    split_idx = np.where(np.diff(lane_x) > max_gap)[0]
    start_idx = np.concatenate(([0], split_idx + 1))
    end_idx = np.concatenate((split_idx, [lane_x.size - 1]))

    clusters = []
    for s_idx, e_idx in zip(start_idx, end_idx):
        x0 = int(lane_x[s_idx])
        x1 = int(lane_x[e_idx])
        width = x1 - x0 + 1
        if width < min_cluster_width:
            continue
        clusters.append((x0, x1, 0.5 * (x0 + x1), width))

    return clusters


def _fit_curve_x_of_y(points, image_height, max_degree=2):
    if len(points) < 2:
        return None
    ys = np.array([p[1] for p in points], dtype=np.float32)
    xs = np.array([p[0] for p in points], dtype=np.float32)
    y_norm = ys / max(1.0, float(image_height - 1))
    degree = int(min(max_degree, len(points) - 1))
    coeff = np.polyfit(y_norm, xs, deg=degree)
    return coeff.astype(np.float32)


def _eval_curve_x_of_y(coeff, ys, image_height):
    if coeff is None:
        return None
    ys_arr = np.asarray(ys, dtype=np.float32)
    y_norm = ys_arr / max(1.0, float(image_height - 1))
    return np.polyval(coeff, y_norm).astype(np.float32)


def _blend_curve_coeff(prev_coeff, new_coeff, alpha):
    if prev_coeff is None:
        return None if new_coeff is None else new_coeff.copy()
    if new_coeff is None:
        return prev_coeff.copy()

    prev = np.asarray(prev_coeff, dtype=np.float32)
    new = np.asarray(new_coeff, dtype=np.float32)
    max_len = max(prev.size, new.size)

    if prev.size < max_len:
        prev = np.pad(prev, (max_len - prev.size, 0), mode="constant")
    if new.size < max_len:
        new = np.pad(new, (max_len - new.size, 0), mode="constant")

    return (alpha * prev + (1.0 - alpha) * new).astype(np.float32)


def _curve_points_from_coeff(coeff, rows, image_height, image_width):
    if coeff is None:
        return []
    xs = _eval_curve_x_of_y(coeff, rows, image_height)
    if xs is None:
        return []
    pts = []
    for x, y in zip(xs, rows):
        xi = int(np.clip(np.round(float(x)), 0, image_width - 1))
        yi = int(y)
        pts.append((xi, yi))
    return pts


def estimate_center_lane(binary_mask, previous_state=None, sample_count=36):
    previous_state = previous_state or {}

    if binary_mask is None or binary_mask.size == 0:
        return {
            "left_points": [],
            "right_points": [],
            "center_points": [],
            "confidence": 0.0,
            "lane_width_px": previous_state.get("lane_width_px"),
            "sample_rows": 0,
            "paired_rows": 0,
            "center_rows": 0,
            "inferred_rows": 0,
            "bottom_center_x": None,
            "bottom_offset_px": None,
            "tracker_state": dict(previous_state),
        }

    if binary_mask.ndim == 3:
        mask = cv2.cvtColor(binary_mask, cv2.COLOR_BGR2GRAY)
    else:
        mask = binary_mask

    h, w = mask.shape[:2]
    if h < 8 or w < 8:
        return {
            "left_points": [],
            "right_points": [],
            "center_points": [],
            "confidence": 0.0,
            "lane_width_px": previous_state.get("lane_width_px"),
            "sample_rows": 0,
            "paired_rows": 0,
            "center_rows": 0,
            "inferred_rows": 0,
            "bottom_center_x": None,
            "bottom_offset_px": None,
            "tracker_state": dict(previous_state),
        }

    prev_lane_width_px = previous_state.get("lane_width_px")
    prev_center_bottom_x = previous_state.get("center_bottom_x")
    prev_center_curve = previous_state.get("center_curve")
    prev_left_curve = previous_state.get("left_curve")
    prev_right_curve = previous_state.get("right_curve")
    prev_lost_frames = int(previous_state.get("lost_frames", 0))

    y_top = max(0, int(0.35 * h))
    rows = np.linspace(h - 1, y_top, num=max(10, int(sample_count))).astype(np.int32)
    rows = np.unique(rows)[::-1]
    image_center_x = 0.5 * (w - 1)
    center_anchor_x = (
        float(prev_center_bottom_x)
        if prev_center_bottom_x is not None
        else image_center_x
    )

    default_width_px = float(np.clip(0.30 * w, 0.14 * w, 0.52 * w))
    expected_lane_width = (
        default_width_px
        if prev_lane_width_px is None
        else float(np.clip(prev_lane_width_px, 0.12 * w, 0.65 * w))
    )

    min_lane_width_px = max(14.0, 0.06 * w)
    max_lane_width_px = min(0.82 * w, max(min_lane_width_px + 12.0, 2.2 * expected_lane_width))
    if prev_lane_width_px is not None:
        min_lane_width_px = max(min_lane_width_px, 0.55 * expected_lane_width)
        max_lane_width_px = min(max_lane_width_px, 1.55 * expected_lane_width)

    left_points_raw = []
    right_points_raw = []
    center_points_raw = []
    width_samples = []
    paired_rows = 0
    inferred_rows = 0

    for y in rows:
        row_clusters = _extract_row_clusters(mask[int(y), :], min_cluster_width=2, max_gap=3)
        if not row_clusters:
            continue

        centers = np.array(sorted([float(c[2]) for c in row_clusters]), dtype=np.float32)
        if centers.size == 0:
            continue

        if prev_center_curve is not None:
            pred_center_x = float(_eval_curve_x_of_y(prev_center_curve, [y], h)[0])
        else:
            pred_center_x = center_anchor_x
        pred_center_x = float(np.clip(pred_center_x, 0.0, w - 1.0))

        expected_left_x = pred_center_x - 0.5 * expected_lane_width
        expected_right_x = pred_center_x + 0.5 * expected_lane_width

        best_pair = None
        best_cost = 1e9

        for i in range(len(centers) - 1):
            for j in range(i + 1, len(centers)):
                left_x = float(centers[i])
                right_x = float(centers[j])
                lane_width = right_x - left_x
                if lane_width < min_lane_width_px or lane_width > max_lane_width_px:
                    continue

                pair_center_x = 0.5 * (left_x + right_x)
                center_cost = abs(pair_center_x - pred_center_x) / max(20.0, 0.22 * w)
                width_cost = abs(lane_width - expected_lane_width) / max(
                    12.0, expected_lane_width
                )
                cost = 0.68 * center_cost + 0.32 * width_cost
                if cost < best_cost:
                    best_cost = cost
                    best_pair = (left_x, right_x, lane_width)

        pair_measured = False
        left_x = None
        right_x = None

        if best_pair is not None:
            left_x, right_x, lane_width = best_pair
            pair_measured = True
        else:
            left_candidates = centers[centers <= pred_center_x]
            right_candidates = centers[centers >= pred_center_x]

            if left_candidates.size > 0:
                left_x = float(
                    left_candidates[np.argmin(np.abs(left_candidates - expected_left_x))]
                )
            if right_candidates.size > 0:
                right_x = float(
                    right_candidates[np.argmin(np.abs(right_candidates - expected_right_x))]
                )

            if left_x is not None and right_x is not None:
                lane_width = right_x - left_x
                if lane_width < min_lane_width_px or lane_width > max_lane_width_px:
                    if abs(left_x - expected_left_x) <= abs(right_x - expected_right_x):
                        right_x = left_x + expected_lane_width
                    else:
                        left_x = right_x - expected_lane_width
                else:
                    pair_measured = True
            elif left_x is not None:
                right_x = left_x + expected_lane_width
            elif right_x is not None:
                left_x = right_x - expected_lane_width
            else:
                closest_x = float(centers[np.argmin(np.abs(centers - pred_center_x))])
                if closest_x <= pred_center_x:
                    left_x = closest_x
                    right_x = left_x + expected_lane_width
                else:
                    right_x = closest_x
                    left_x = right_x - expected_lane_width

        if left_x is None or right_x is None:
            continue

        left_x = float(np.clip(left_x, 0.0, w - 1.0))
        right_x = float(np.clip(right_x, 0.0, w - 1.0))
        if right_x - left_x < 2.0:
            continue

        center_x = 0.5 * (left_x + right_x)

        left_points_raw.append((int(round(left_x)), int(y)))
        right_points_raw.append((int(round(right_x)), int(y)))
        center_points_raw.append((int(round(center_x)), int(y)))

        if pair_measured:
            paired_rows += 1
            width_samples.append(float(right_x - left_x))
        else:
            inferred_rows += 1

    sample_rows = int(rows.size)
    center_rows = len(center_points_raw)
    pair_support = float(paired_rows) / max(1, sample_rows)
    center_support = float(center_rows) / max(1, sample_rows)

    width_consistency = 0.0
    lane_width_px = None
    if width_samples:
        lane_width_px = float(np.median(width_samples))
        if len(width_samples) == 1:
            width_consistency = 0.7
        else:
            width_std = float(np.std(width_samples))
            width_consistency = 1.0 - float(
                np.clip(width_std / max(1.0, lane_width_px), 0.0, 1.0)
            )
    elif prev_lane_width_px is not None:
        lane_width_px = float(prev_lane_width_px)
        width_consistency = 0.3

    left_curve_new = _fit_curve_x_of_y(left_points_raw, h, max_degree=2)
    right_curve_new = _fit_curve_x_of_y(right_points_raw, h, max_degree=2)
    center_curve_new = _fit_curve_x_of_y(center_points_raw, h, max_degree=2)

    if center_curve_new is not None and prev_center_curve is not None:
        new_bottom = float(_eval_curve_x_of_y(center_curve_new, [h - 1], h)[0])
        prev_bottom = float(_eval_curve_x_of_y(prev_center_curve, [h - 1], h)[0])
        center_shift = abs(new_bottom - prev_bottom)
        # Strong smoothing for jitter, but relax smoothing when lane change is large.
        center_curve_alpha = 0.82 if center_shift < 36.0 else 0.60
        center_curve = _blend_curve_coeff(
            prev_center_curve, center_curve_new, center_curve_alpha
        )
    elif center_curve_new is not None:
        center_curve = center_curve_new
    elif prev_center_curve is not None and prev_lost_frames < 5:
        center_curve = prev_center_curve.copy()
    else:
        center_curve = None

    side_curve_alpha = 0.78
    left_curve = _blend_curve_coeff(prev_left_curve, left_curve_new, side_curve_alpha)
    right_curve = _blend_curve_coeff(prev_right_curve, right_curve_new, side_curve_alpha)

    center_points = _curve_points_from_coeff(center_curve, rows, h, w)
    left_points = _curve_points_from_coeff(left_curve, rows, h, w)
    right_points = _curve_points_from_coeff(right_curve, rows, h, w)

    using_prev_only = False
    if not center_points and prev_center_curve is not None and prev_lost_frames < 5:
        center_points = _curve_points_from_coeff(prev_center_curve, rows, h, w)
        left_points = _curve_points_from_coeff(prev_left_curve, rows, h, w)
        right_points = _curve_points_from_coeff(prev_right_curve, rows, h, w)
        center_curve = prev_center_curve.copy()
        left_curve = None if prev_left_curve is None else prev_left_curve.copy()
        right_curve = None if prev_right_curve is None else prev_right_curve.copy()
        using_prev_only = True

    occupancy_ratio = float(np.count_nonzero(mask)) / float(mask.size)
    occupancy_score = float(np.clip(occupancy_ratio / 0.04, 0.0, 1.0))

    bottom_center_x = None
    if center_curve is not None:
        bottom_center_x = float(_eval_curve_x_of_y(center_curve, [h - 1], h)[0])
    elif center_points:
        bottom_point = max(center_points, key=lambda p: p[1])
        bottom_center_x = float(bottom_point[0])

    if bottom_center_x is not None:
        bottom_center_x = float(np.clip(bottom_center_x, 0.0, w - 1.0))

    tracking_alignment = 0.5
    if bottom_center_x is not None and prev_center_bottom_x is not None:
        tracking_alignment = 1.0 - float(
            np.clip(abs(bottom_center_x - float(prev_center_bottom_x)) / 90.0, 0.0, 1.0)
        )

    inferred_ratio = float(inferred_rows) / max(1.0, float(center_rows))
    confidence = (
        0.45 * pair_support
        + 0.25 * center_support
        + 0.15 * width_consistency
        + 0.10 * tracking_alignment
        + 0.05 * occupancy_score
    )
    confidence *= 1.0 - 0.35 * float(np.clip(inferred_ratio, 0.0, 1.0))
    if using_prev_only:
        confidence *= 0.55
    if paired_rows < 3 and not using_prev_only:
        confidence *= 0.8
    confidence = float(np.clip(confidence, 0.0, 1.0))

    lane_width_px = (
        float(np.clip(lane_width_px, min_lane_width_px, max_lane_width_px))
        if lane_width_px is not None
        else None
    )

    lost_frames = 0 if center_points else (prev_lost_frames + 1)
    tracker_state = {
        "lane_width_px": lane_width_px,
        "center_bottom_x": (
            bottom_center_x if bottom_center_x is not None else prev_center_bottom_x
        ),
        "center_curve": None if center_curve is None else center_curve.copy(),
        "left_curve": None if left_curve is None else left_curve.copy(),
        "right_curve": None if right_curve is None else right_curve.copy(),
        "lost_frames": int(lost_frames),
    }

    return {
        "left_points": left_points,
        "right_points": right_points,
        "center_points": center_points,
        "confidence": confidence,
        "lane_width_px": lane_width_px,
        "sample_rows": sample_rows,
        "paired_rows": int(paired_rows),
        "center_rows": int(center_rows),
        "inferred_rows": int(inferred_rows),
        "bottom_center_x": bottom_center_x,
        "bottom_offset_px": None
        if bottom_center_x is None
        else float(bottom_center_x - image_center_x),
        "tracker_state": tracker_state,
    }


def draw_lane_center_overlay(rgb_image, lane_center_result):
    def draw_polyline(points, color, thickness):
        if len(points) >= 2:
            pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(rgb_image, [pts], False, color, thickness, cv2.LINE_AA)
        elif len(points) == 1:
            cv2.circle(rgb_image, points[0], max(1, thickness), color, -1, cv2.LINE_AA)

    draw_polyline(lane_center_result["left_points"], (255, 170, 0), 2)
    draw_polyline(lane_center_result["right_points"], (255, 170, 0), 2)
    draw_polyline(lane_center_result["center_points"], (0, 255, 0), 3)

    if lane_center_result["center_points"]:
        bottom_point = max(lane_center_result["center_points"], key=lambda p: p[1])
        cv2.circle(rgb_image, bottom_point, 5, (0, 255, 0), -1, cv2.LINE_AA)


sampleRate = 30.0
sampleTime = 1 / sampleRate
simulationTime = 1000.0
print("Sample Time: ", sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Additional parameters
imageWidth = 640
imageHeight = 480
PRETRAINED_MODEL_DIR = (
    r"C:\Users\Quang Huy Nugyen\Documents\Quanser\libraries\resources\pretrained_models"
)
LANE_CONFIG_DIR = r"C:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\QCAR\QCar2_Cran\Development\pytorch_auto_drive\configs\lane_detection"

MODEL_PRESETS = {
    "erfnet_scnn_culane": {
        "config_path": os.path.join(LANE_CONFIG_DIR, "scnn", "erfnet_culane.py"),
        "weight_path": os.path.join(
            PRETRAINED_MODEL_DIR, "erfnet_scnn_culane_20210206.pt"
        ),
    },
    "resnet18_scnn_tusimple": {
        "config_path": os.path.join(LANE_CONFIG_DIR, "scnn", "resnet18_tusimple.py"),
        "weight_path": os.path.join(PRETRAINED_MODEL_DIR, "resnet18_scnn_tusimple.pt"),
    },
    "resnet18_scnn_culane": {
        "config_path": os.path.join(LANE_CONFIG_DIR, "scnn", "resnet18_culane.py"),
        "weight_path": os.path.join(
            PRETRAINED_MODEL_DIR, "resnet18_scnn_culane_20210222.pt"
        ),
    },
    "vgg16_scnn_llamas": {
        "config_path": os.path.join(LANE_CONFIG_DIR, "scnn", "vgg16_llamas.py"),
        "weight_path": os.path.join(
            PRETRAINED_MODEL_DIR, "vgg16_scnn_llamas_20210625.pt"
        ),
    },
    # Non-SCNN options: download checkpoints from MODEL_ZOO and place here.
    "enet_baseline_tusimple": {
        "config_path": os.path.join(LANE_CONFIG_DIR, "baseline", "enet_tusimple.py"),
        "weight_path": os.path.join(PRETRAINED_MODEL_DIR, "enet_baseline_tusimple.pt"),
    },
    "erfnet_baseline_tusimple": {
        "config_path": os.path.join(LANE_CONFIG_DIR, "baseline", "erfnet_tusimple.py"),
        "weight_path": os.path.join(
            PRETRAINED_MODEL_DIR, "erfnet_baseline_tusimple.pt"
        ),
    },
}

# Change this key to compare different model/checkpoint pairs quickly.
selected_model_key = "resnet18_scnn_tusimple"
fallback_model_key = "resnet18_scnn_tusimple"

if selected_model_key not in MODEL_PRESETS:
    raise KeyError(f"Unknown selected_model_key '{selected_model_key}'.")
if fallback_model_key not in MODEL_PRESETS:
    raise KeyError(f"Unknown fallback_model_key '{fallback_model_key}'.")

selected_model = MODEL_PRESETS[selected_model_key]
if not os.path.exists(selected_model["weight_path"]):
    print(
        f"[LaneModel] Missing weights for '{selected_model_key}': {selected_model['weight_path']}"
    )
    print(f"[LaneModel] Falling back to '{fallback_model_key}'.")
    selected_model_key = fallback_model_key
    selected_model = MODEL_PRESETS[selected_model_key]

print(f"[LaneModel] Selected preset: {selected_model_key}")
print(f"[LaneModel] Config: {selected_model['config_path']}")
print(f"[LaneModel] Weights: {selected_model['weight_path']}")

# Speed/accuracy knobs
SPEED_MODE = True
INPUT_SIZE_OVERRIDE = (256, 640) if SPEED_MODE else None  # (H, W)
DISABLE_LANE_HEAD_FOR_SPEED = SPEED_MODE
INFERENCE_EVERY_N = 1  # Set 2 to run the model every 2nd frame for higher loop FPS.
INFERENCE_EVERY_N = max(1, int(INFERENCE_EVERY_N))

print(
    "[LaneModel] Speed settings: "
    f"speed_mode={int(SPEED_MODE)}, "
    f"input_override={INPUT_SIZE_OVERRIDE}, "
    f"disable_lane_head={int(DISABLE_LANE_HEAD_FOR_SPEED)}, "
    f"infer_every_n={INFERENCE_EVERY_N}"
)

# Camera color handling:
# Some SDK builds return BGR even though the buffer name includes "RGB".
CAMERA_BUFFER_ORDER = "BGR"  # "RGB" or "BGR"
if CAMERA_BUFFER_ORDER not in ("RGB", "BGR"):
    raise ValueError("CAMERA_BUFFER_ORDER must be 'RGB' or 'BGR'.")
print(f"[Camera] Buffer color order: {CAMERA_BUFFER_ORDER}")

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Initialize the LaneNet model
# myLaneNet = LaneNet(
#                     # modelPath = 'path/to/model',
#                     imageHeight = imageHeight,
#                     imageWidth = imageWidth,
#                     rowUpperBound = 200
#                     )
myLaneNet = SCNNWrapper(
    config_path=selected_model["config_path"],
    weight_path=selected_model["weight_path"],
    strict_weights=True,
    temporal_alpha=0.45,
    lane_exist_threshold=0.35,
    lane_prob_threshold=0.22,
    use_half=True,
    morph_open_iterations=0,
    morph_close_iterations=1,
    morph_dilate_iterations=1,
    temporal_binary_threshold=100,
    input_size_override=INPUT_SIZE_OVERRIDE,
    disable_lane_head=DISABLE_LANE_HEAD_FOR_SPEED,
)
active_model_name = myLaneNet.get_model_name()
print(f"[LaneModel] Runtime model name: {active_model_name}")

# Initialize the RealSense camera for RGB
# myCamRGB  = Camera3D(mode='RGB', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
myCamRGB = QCarRealSense(
    mode="RGB", frameWidthRGB=imageWidth, frameHeightRGB=imageHeight, video3dPort=18805
)

# Adaptive ROI controls
crop_bottom_px = 60
crop_end = imageHeight - crop_bottom_px
default_crop_start = max(
    90, crop_end - 300
)  # keep ROI higher, around 300px crop height
crop_start = default_crop_start
min_crop_start = 70
max_crop_start = 240
crop_adjust_period = 5
frame_counter = 0
last_loop_fps = 0.0
last_binary_pred = None
last_annotated_crop = None
last_metrics = {}
infer_counter = 0
lane_tracker_state = {}
smoothed_center_confidence = None

# Live controls (trackbars)
runtime_defaults = myLaneNet.get_runtime_params()
runtime_defaults.update(
    {
        "adaptive_roi": 1,
        "top_crop": crop_start,
        "bottom_crop": crop_bottom_px,
        "min_top": min_crop_start,
        "max_top": max_crop_start,
        "roi_adj_period": crop_adjust_period,
    }
)
controls_window = setup_controls_window(runtime_defaults, imageHeight)

try:
    startTime = time.time()
    while elapsed_time() < simulationTime:
        start = time.time()
        myCamRGB.read_RGB()
        raw_frame = myCamRGB.imageBufferRGB
        if CAMERA_BUFFER_ORDER == "BGR":
            raw_rgb = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
        else:
            raw_rgb = raw_frame

        controls = read_live_controls(controls_window, runtime_defaults)

        # Runtime lane-model parameter tuning
        myLaneNet.set_runtime_params(
            lane_prob_threshold=controls["lane_prob_threshold"],
            lane_exist_threshold=controls["lane_exist_threshold"],
            temporal_alpha=controls["temporal_alpha"],
            temporal_binary_threshold=controls["temporal_binary_threshold"],
            morph_open_iterations=controls["morph_open_iterations"],
            morph_close_iterations=controls["morph_close_iterations"],
            morph_dilate_iterations=controls["morph_dilate_iterations"],
        )

        # Runtime crop tuning
        crop_bottom_px = int(np.clip(controls["bottom_crop"], 0, imageHeight - 32))
        crop_end = imageHeight - crop_bottom_px
        min_crop_start = int(np.clip(controls["min_top"], 0, crop_end - 16))
        max_crop_start = int(
            np.clip(controls["max_top"], min_crop_start, crop_end - 16)
        )
        crop_adjust_period = max(1, int(controls["roi_adj_period"]))

        if not controls["adaptive_roi"]:
            crop_start = int(
                np.clip(controls["top_crop"], min_crop_start, max_crop_start)
            )

        # Clamp current crop to valid bounds
        crop_start = int(np.clip(crop_start, min_crop_start, max_crop_start))
        cropped_rgb = raw_rgb[crop_start:crop_end, :, :]

        infer_counter += 1
        crop_shape_hw = cropped_rgb.shape[:2]
        cache_valid = (
            last_binary_pred is not None
            and last_binary_pred.shape[:2] == crop_shape_hw
            and last_annotated_crop is not None
            and last_annotated_crop.shape[:2] == crop_shape_hw
        )
        run_inference = (
            not cache_valid
            or INFERENCE_EVERY_N <= 1
            or (infer_counter % INFERENCE_EVERY_N == 0)
        )

        if run_inference:
            binaryPred, _ = myLaneNet.predict(cropped_rgb)
            metrics = myLaneNet.get_metrics()
            annotated_crop = myLaneNet.render(showFPS=True, showMetrics=True)
            last_binary_pred = binaryPred
            last_annotated_crop = annotated_crop
            last_metrics = dict(metrics)
        else:
            binaryPred = last_binary_pred
            annotated_crop = last_annotated_crop
            metrics = dict(last_metrics)

        lane_center_result = estimate_center_lane(
            binaryPred, previous_state=lane_tracker_state
        )
        lane_tracker_state = lane_center_result.get("tracker_state", lane_tracker_state)

        if smoothed_center_confidence is None:
            smoothed_center_confidence = lane_center_result["confidence"]
        else:
            smoothed_center_confidence = (
                0.7 * smoothed_center_confidence + 0.3 * lane_center_result["confidence"]
            )

        annotated_crop = annotated_crop.copy()
        draw_lane_center_overlay(annotated_crop, lane_center_result)

        # Render lane-model output in the crop, then place it back into full frame.
        annotated_full = raw_rgb.copy()
        annotated_full[crop_start:crop_end, :, :] = annotated_crop
        cv2.rectangle(
            annotated_full,
            (0, crop_start),
            (imageWidth - 1, crop_end - 1),
            (255, 255, 0),
            2,
        )

        # Adjust ROI every few frames using lane occupancy at crop top and bottom.
        frame_counter += 1
        if controls["adaptive_roi"] and frame_counter % crop_adjust_period == 0:
            lane_ratio = metrics.get("lane_ratio", 0.0)
            top_ratio = metrics.get("top_lane_ratio", 0.0)
            bottom_ratio = metrics.get("bottom_lane_ratio", 0.0)

            if top_ratio > 0.012 and crop_start > min_crop_start:
                crop_start -= 4
            elif lane_ratio < 0.002 and crop_start > min_crop_start:
                crop_start -= 2
            elif (
                top_ratio < 0.001
                and bottom_ratio > 0.01
                and crop_start < max_crop_start
            ):
                crop_start += 2

        center_conf_color = (255, 80, 80)
        if smoothed_center_confidence >= 0.65:
            center_conf_color = (80, 255, 80)
        elif smoothed_center_confidence >= 0.35:
            center_conf_color = (255, 220, 80)

        center_offset_px = lane_center_result.get("bottom_offset_px")
        if center_offset_px is None:
            center_offset_text = "n/a"
        else:
            center_offset_text = f"{center_offset_px:+.1f}px"

        cv2.putText(
            annotated_full,
            (
                f"Model: {selected_model_key} "
                f"| In: {myLaneNet.input_size_hw[0]}x{myLaneNet.input_size_hw[1]} "
                f"| N: {INFERENCE_EVERY_N}"
            ),
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            annotated_full,
            f"ROI top: {crop_start}px",
            (10, imageHeight - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            annotated_full,
            f"Lane ratio: {metrics.get('lane_ratio', 0.0):.3f}",
            (10, imageHeight - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            annotated_full,
            f"Bottom crop: {crop_bottom_px}px  Loop FPS: {last_loop_fps:.1f}",
            (10, imageHeight - 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            annotated_full,
            (
                f"Center conf: {smoothed_center_confidence:.2f} "
                f"| rows: {lane_center_result['paired_rows']}/"
                f"{lane_center_result['sample_rows']} "
                f"| inf: {lane_center_result['inferred_rows']} "
                f"| offset: {center_offset_text}"
            ),
            (10, imageHeight - 115),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            center_conf_color,
            2,
        )
        cv2.putText(
            annotated_full,
            (
                f"P:{controls['lane_prob_threshold']:.2f} "
                f"E:{controls['lane_exist_threshold']:.2f} "
                f"A:{controls['temporal_alpha']:.2f} "
                f"D:{controls['morph_dilate_iterations']}"
            ),
            (10, imageHeight - 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )

        # Convert RGB to BGR for OpenCV display
        displayImg = cv2.cvtColor(annotated_full, cv2.COLOR_RGB2BGR)
        binary_display = cv2.cvtColor(binaryPred, cv2.COLOR_GRAY2BGR)
        draw_lane_center_overlay(binary_display, lane_center_result)
        cv2.imshow("Extracted Lane Markings", displayImg)
        cv2.imshow("Lane Binary Mask", binary_display)

        # End timing this iteration
        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        last_loop_fps = 1.0 / max(1e-6, computationTime)
        sleepTime = max(0.0, sampleTime - computationTime)

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000 * sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1

        key = cv2.waitKey(msSleepTime) & 0xFF
        if key == ord("s"):
            print("Saving debug images...")
            cv2.imwrite(
                "debug_input_crop.png", cv2.cvtColor(cropped_rgb, cv2.COLOR_RGB2BGR)
            )
            cv2.imwrite("debug_output.png", displayImg)
            cv2.imwrite("debug_binary.png", binaryPred)
            if CAMERA_BUFFER_ORDER == "BGR":
                cv2.imwrite("debug_raw.png", raw_frame)
            else:
                cv2.imwrite("debug_raw.png", cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR))
        elif key == ord("r"):
            print("Reset temporal filter and ROI.")
            myLaneNet.reset_temporal_filter()
            crop_start = int(
                np.clip(default_crop_start, min_crop_start, max_crop_start)
            )
            last_binary_pred = None
            last_annotated_crop = None
            last_metrics = {}
            lane_tracker_state = {}
            smoothed_center_confidence = None
        elif key == ord("p"):
            params = myLaneNet.get_runtime_params()
            print(
                "[LIVE PARAMS] "
                f"lane_prob={params['lane_prob_threshold']:.2f}, "
                f"lane_exist={params['lane_exist_threshold']:.2f}, "
                f"temp_alpha={params['temporal_alpha']:.2f}, "
                f"temp_bin={params['temporal_binary_threshold']}, "
                f"open/close/dilate="
                f"{params['morph_open_iterations']}/"
                f"{params['morph_close_iterations']}/"
                f"{params['morph_dilate_iterations']}, "
                f"model={selected_model_key}, "
                f"in={myLaneNet.input_size_hw}, infer_n={INFERENCE_EVERY_N}, "
                f"top={crop_start}, bottom={crop_bottom_px}, "
                f"adaptive={int(controls['adaptive_roi'])}"
            )
        elif key == ord("q"):
            print("Quit requested.")
            break

except KeyboardInterrupt:
    print("User interrupted!")

finally:
    cv2.destroyAllWindows()
    myCamRGB.terminate()
