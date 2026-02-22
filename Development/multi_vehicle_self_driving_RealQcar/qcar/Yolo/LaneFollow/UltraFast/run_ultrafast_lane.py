"""
Run Ultra-Fast-Lane-Detection-v2 on QCar RealSense Camera

Features:
  - Clean lane overlay visualization (colored circles + lines)
  - Coordinate-based center lane estimation (no binary mask needed)
  - Adaptive ROI cropping for performance
  - Live OpenCV trackbar controls
  - Rich HUD overlay
  - Keyboard shortcuts: s(save) r(reset) p(params) q(quit)

Usage:
    python run_ultrafast_lane.py
"""

import os
import sys
import time

import cv2
import numpy as np
from pal.products.qcar import QCarRealSense
from ultrafast_wrapper import (
    UltraFastV2Wrapper,
    draw_center_lane_overlay,
)


# ============================================================================
#  Configuration
# ============================================================================

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

MODEL_PATH = (
    r"C:\Users\Quang Huy Nugyen\Documents\Quanser\libraries"
    r"\resources\pretrained_models\curvelanes_res18.pth"
)
MODEL_TYPE = "curvelanes_res18"

SAMPLE_RATE = 30.0
SAMPLE_TIME = 1.0 / SAMPLE_RATE
SIMULATION_TIME = 1000.0

INFERENCE_EVERY_N = 1  # Set 2+ to skip frames
INFERENCE_EVERY_N = max(1, int(INFERENCE_EVERY_N))

# Camera buffer color order: "RGB" or "BGR"
CAMERA_BUFFER_ORDER = "BGR"

# Camera-to-body lateral offset compensation (metres → pixels).
# The RealSense is mounted 0.032 m to the LEFT of the body center (T_CB[1,3]).
# Positive value = camera is left of body ⇒ body center is RIGHT in image.
# Tune this value if the car drifts to one side when centred in the lane.
CAMERA_Y_OFFSET_PX = 10.0  # ≈ 0.032 m, start conservative and tune on track

# Minimum confidence to draw/use center lane estimation
CENTER_MIN_CONFIDENCE = 0.4


# ============================================================================
#  Trackbar helpers
# ============================================================================


def nothing(_):
    pass


def safe_get_trackbar(name, window, default):
    try:
        return cv2.getTrackbarPos(name, window)
    except cv2.error:
        return default


def setup_controls_window(initial, image_height):
    window = "Lane Controls"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 400, 400)

    cv2.createTrackbar(
        "ExistThr x100",
        window,
        int(initial["lane_exist_threshold"] * 100),
        100,
        nothing,
    )
    cv2.createTrackbar("MaxLane", window, int(initial["max_lane"]), 10, nothing)
    cv2.createTrackbar(
        "CenterAlpha x100",
        window,
        int(initial["center_smooth_alpha"] * 100),
        99,
        nothing,
    )
    cv2.createTrackbar("AdaptiveROI", window, 1, 1, nothing)
    cv2.createTrackbar(
        "TopCrop", window, int(initial.get("top_crop", 140)), image_height - 1, nothing
    )
    cv2.createTrackbar(
        "BottomCrop",
        window,
        int(initial.get("bottom_crop", 60)),
        image_height // 2,
        nothing,
    )
    cv2.createTrackbar(
        "MinTop", window, int(initial.get("min_top", 70)), image_height - 1, nothing
    )
    cv2.createTrackbar(
        "MaxTop", window, int(initial.get("max_top", 240)), image_height - 1, nothing
    )
    cv2.createTrackbar(
        "ROIAdjPeriod", window, int(initial.get("roi_adj_period", 5)), 30, nothing
    )
    # Camera offset (pixels), range -50 to +50, mapped to 0-100slider
    # Slider value 50 corresponds to 0 offset.
    cv2.createTrackbar(
        "CamOffset",
        window,
        int(initial.get("camera_y_offset_px", 0) + 50),
        100,
        nothing,
    )

    return window


def read_live_controls(window, defaults):
    return {
        "lane_exist_threshold": safe_get_trackbar(
            "ExistThr x100", window, int(defaults["lane_exist_threshold"] * 100)
        )
        / 100.0,
        "max_lane": safe_get_trackbar("MaxLane", window, int(defaults["max_lane"])),
        "center_smooth_alpha": safe_get_trackbar(
            "CenterAlpha x100", window, int(defaults["center_smooth_alpha"] * 100)
        )
        / 100.0,
        "adaptive_roi": safe_get_trackbar("AdaptiveROI", window, 1) > 0,
        "top_crop": safe_get_trackbar(
            "TopCrop", window, int(defaults.get("top_crop", 140))
        ),
        "bottom_crop": safe_get_trackbar(
            "BottomCrop", window, int(defaults.get("bottom_crop", 60))
        ),
        "min_top": safe_get_trackbar(
            "MinTop", window, int(defaults.get("min_top", 70))
        ),
        "max_top": safe_get_trackbar(
            "MaxTop", window, int(defaults.get("max_top", 240))
        ),
        "roi_adj_period": max(
            1,
            safe_get_trackbar(
                "ROIAdjPeriod", window, int(defaults.get("roi_adj_period", 5))
            ),
        ),
        "camera_y_offset_px": safe_get_trackbar(
            "CamOffset", window, int(defaults.get("camera_y_offset_px", 0) + 50)
        )
        - 50,
    }


# ============================================================================
#  Timing
# ============================================================================


def elapsed_time():
    return time.time() - startTime


# ============================================================================
#  Main
# ============================================================================

if __name__ == "__main__":
    print(f"[UltraFast] Initializing...")
    print(f"  Model:       {MODEL_TYPE}")
    print(f"  Weight:      {MODEL_PATH}")
    print(f"  Camera:      {IMAGE_WIDTH}x{IMAGE_HEIGHT} @ {SAMPLE_RATE} Hz")
    print(f"  InferEvery:  {INFERENCE_EVERY_N}")
    print(f"  BufferOrder: {CAMERA_BUFFER_ORDER}")

    # Initialize model
    model = UltraFastV2Wrapper(
        model_path=MODEL_PATH,
        model_type=MODEL_TYPE,
        lane_exist_threshold=0.35,
        max_lane=2,
        center_smooth_alpha=0.75,
        camera_y_offset_px=CAMERA_Y_OFFSET_PX,
    )
    active_model_name = model.get_model_name()

    # Initialize camera
    myCamRGB = QCarRealSense(
        mode="RGB",
        frameWidthRGB=IMAGE_WIDTH,
        frameHeightRGB=IMAGE_HEIGHT,
        video3dPort=18805,
    )

    # Adaptive ROI state
    crop_bottom_px = 60
    crop_end = IMAGE_HEIGHT - crop_bottom_px
    default_crop_start = max(90, crop_end - 300)
    crop_start = default_crop_start
    min_crop_start = 70
    max_crop_start = 240
    frame_counter = 0
    last_loop_fps = 0.0
    infer_counter = 0
    smoothed_center_confidence = None

    # Cache for frame skipping
    last_annotated_crop = None
    last_center_result = None

    # Trackbar controls
    runtime_defaults = model.get_runtime_params()
    runtime_defaults.update(
        {
            "top_crop": crop_start,
            "bottom_crop": crop_bottom_px,
            "min_top": min_crop_start,
            "max_top": max_crop_start,
            "roi_adj_period": 5,
            "camera_y_offset_px": CAMERA_Y_OFFSET_PX,
        }
    )
    controls_window = setup_controls_window(runtime_defaults, IMAGE_HEIGHT)

    try:
        startTime = time.time()

        while elapsed_time() < SIMULATION_TIME:
            start = time.time()
            myCamRGB.read_RGB()
            raw_frame = myCamRGB.imageBufferRGB

            if CAMERA_BUFFER_ORDER == "BGR":
                raw_rgb = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
            else:
                raw_rgb = raw_frame

            controls = read_live_controls(controls_window, runtime_defaults)

            # Update runtime params
            model.set_runtime_params(
                lane_exist_threshold=controls["lane_exist_threshold"],
                max_lane=controls["max_lane"],
                center_smooth_alpha=controls["center_smooth_alpha"],
                camera_y_offset_px=controls["camera_y_offset_px"],
            )

            # Crop tuning
            crop_bottom_px = int(np.clip(controls["bottom_crop"], 0, IMAGE_HEIGHT - 32))
            crop_end = IMAGE_HEIGHT - crop_bottom_px
            min_crop_start = int(np.clip(controls["min_top"], 0, crop_end - 16))
            max_crop_start = int(
                np.clip(controls["max_top"], min_crop_start, crop_end - 16)
            )

            if not controls["adaptive_roi"]:
                crop_start = int(
                    np.clip(controls["top_crop"], min_crop_start, max_crop_start)
                )

            crop_start = int(np.clip(crop_start, min_crop_start, max_crop_start))
            cropped_rgb = raw_rgb[crop_start:crop_end, :, :]

            # --- Inference (with frame skipping) ---
            infer_counter += 1
            crop_shape_hw = cropped_rgb.shape[:2]
            cache_valid = (
                last_annotated_crop is not None
                and last_annotated_crop.shape[:2] == crop_shape_hw
                and last_center_result is not None
            )
            run_inference = (
                not cache_valid
                or INFERENCE_EVERY_N <= 1
                or (infer_counter % INFERENCE_EVERY_N == 0)
            )

            if run_inference:
                bgr_crop = cv2.cvtColor(cropped_rgb, cv2.COLOR_RGB2BGR)
                coords = model.predict(bgr_crop)
                center_result = model.estimate_center()
                annotated_crop = model.render(showFPS=True, showMetrics=True)
                last_annotated_crop = annotated_crop
                last_center_result = center_result
            else:
                annotated_crop = last_annotated_crop
                center_result = last_center_result

            # Smoothed center confidence
            if smoothed_center_confidence is None:
                smoothed_center_confidence = center_result["confidence"]
            else:
                smoothed_center_confidence = (
                    0.7 * smoothed_center_confidence + 0.3 * center_result["confidence"]
                )

            # Draw center lane overlay
            annotated_crop = annotated_crop.copy()
            draw_center_lane_overlay(
                annotated_crop, center_result, min_confidence=CENTER_MIN_CONFIDENCE
            )

            # Place crop back into full frame
            annotated_full = raw_rgb.copy()
            annotated_full[crop_start:crop_end, :, :] = annotated_crop
            cv2.rectangle(
                annotated_full,
                (0, crop_start),
                (IMAGE_WIDTH - 1, crop_end - 1),
                (255, 255, 0),
                2,
            )

            # --- Adaptive ROI ---
            frame_counter += 1
            if (
                controls["adaptive_roi"]
                and frame_counter % controls["roi_adj_period"] == 0
            ):
                # Use lane coordinate coverage to adjust ROI
                if model.last_coords:
                    all_ys = [pt[1] for lane in model.last_coords for pt in lane]
                    if all_ys:
                        min_y = min(all_ys)
                        crop_h = crop_end - crop_start
                        # If lanes reach very top of crop → expand upward
                        if min_y < 0.15 * crop_h and crop_start > min_crop_start:
                            crop_start -= 4
                        # If lanes only in bottom half → shrink from top
                        elif min_y > 0.5 * crop_h and crop_start < max_crop_start:
                            crop_start += 2
                else:
                    # No lanes detected — expand crop to search wider
                    if crop_start > min_crop_start:
                        crop_start -= 2

            # --- HUD ---
            center_conf_color = (255, 80, 80)
            if smoothed_center_confidence >= 0.65:
                center_conf_color = (80, 255, 80)
            elif smoothed_center_confidence >= 0.35:
                center_conf_color = (255, 220, 80)

            offset_px = center_result.get("bottom_offset_px")
            offset_text = f"{offset_px:+.0f}px" if offset_px is not None else "n/a"
            width_px = center_result.get("lane_width_px")
            width_text = f"{width_px:.0f}px" if width_px is not None else "n/a"

            cv2.putText(
                annotated_full,
                f"Model: {active_model_name}  |  In: {model.input_size_hw[0]}x{model.input_size_hw[1]}  |  Inference_N: {INFERENCE_EVERY_N}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 0),
                2,
            )
            cv2.putText(
                annotated_full,
                f"Lanes: {model.get_lane_count()}  |  ROI: {crop_start}-{crop_end}  |  Loop: {last_loop_fps:.0f} FPS",
                (10, IMAGE_HEIGHT - 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2,
            )
            cv2.putText(
                annotated_full,
                f"Center conf: {smoothed_center_confidence:.2f}  |  offset: {offset_text}  |  width: {width_text}",
                (10, IMAGE_HEIGHT - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                center_conf_color,
                2,
            )
            cv2.putText(
                annotated_full,
                f"ExistThr: {controls['lane_exist_threshold']:.2f}  MaxLane: {controls['max_lane']}  Alpha: {controls['center_smooth_alpha']:.2f}",
                (10, IMAGE_HEIGHT - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 0),
                2,
            )
            cv2.putText(
                annotated_full,
                f"OffsetTune: {controls['camera_y_offset_px']:.0f}px",
                (IMAGE_WIDTH - 160, IMAGE_HEIGHT - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 255),
                2,
            )

            # --- Display ---
            displayImg = cv2.cvtColor(annotated_full, cv2.COLOR_RGB2BGR)
            cv2.imshow("UltraFast Lane Detection v2", displayImg)

            # Timing
            end = time.time()
            computationTime = end - start
            last_loop_fps = 1.0 / max(1e-6, computationTime)
            sleepTime = max(0.0, SAMPLE_TIME - computationTime)
            msSleepTime = max(1, int(1000 * sleepTime))

            # --- Keyboard shortcuts ---
            key = cv2.waitKey(msSleepTime) & 0xFF

            if key == ord("s"):
                print("[UltraFast] Saving debug images...")
                cv2.imwrite(
                    "debug_input_crop.png", cv2.cvtColor(cropped_rgb, cv2.COLOR_RGB2BGR)
                )
                cv2.imwrite("debug_output.png", displayImg)
                if CAMERA_BUFFER_ORDER == "BGR":
                    cv2.imwrite("debug_raw.png", raw_frame)
                else:
                    cv2.imwrite(
                        "debug_raw.png", cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR)
                    )
                print(
                    "[UltraFast] Saved: debug_input_crop.png, debug_output.png, debug_raw.png"
                )

            elif key == ord("r"):
                print("[UltraFast] Reset tracker and ROI.")
                model.reset_center_tracker()
                crop_start = int(
                    np.clip(default_crop_start, min_crop_start, max_crop_start)
                )
                last_annotated_crop = None
                last_center_result = None
                smoothed_center_confidence = None

            elif key == ord("p"):
                params = model.get_runtime_params()
                print(
                    f"[PARAMS] exist_thr={params['lane_exist_threshold']:.2f}, "
                    f"max_lane={params['max_lane']}, "
                    f"center_alpha={params['center_smooth_alpha']:.2f}, "
                    f"model={active_model_name}, "
                    f"ROI={crop_start}-{crop_end}, "
                    f"adaptive={int(controls['adaptive_roi'])}, "
                    f"infer_n={INFERENCE_EVERY_N}"
                )

            elif key == ord("q"):
                print("[UltraFast] Quit requested.")
                break

    except KeyboardInterrupt:
        print("[UltraFast] User interrupted!")

    finally:
        myCamRGB.terminate()
        cv2.destroyAllWindows()
        print("[UltraFast] Cleaned up.")
