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
        cv2.imshow("Extracted Lane Markings", displayImg)
        cv2.imshow("Lane Binary Mask", binaryPred)

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
