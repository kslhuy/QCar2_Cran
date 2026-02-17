## QCar2_lane_following_new.py
# Updated Lane Following Script with Selectable Algorithms
# Modes: BEV (Sliding Window), HSV (Color Threshold), LaneNet (Deep Learning)

import time
import numpy as np
import cv2
import os
import sys

# Ensure library paths
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# QCar Imports
from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.products.qcar import QCarRealSense
from pal.utilities.math import Filter

# Lane Detector Imports
try:
    from lane_detectors import HSVLaneDetector, BEVLaneDetector, LaneNetDetector
    from lane_detection_interface import LaneDetectionResult
except ImportError:
    # Try importing from package if running from root
    try:
        from LaneFollow.lane_detectors import HSVLaneDetector, BEVLaneDetector, LaneNetDetector
        from LaneFollow.lane_detection_interface import LaneDetectionResult
    except ImportError:
        print("Error: Could not import lane detectors. Check your python path.")
        sys.exit(1)

# Optional unified renderer (YOLO + lane overlay). Falls back to cv2 display if unavailable.
try:
    from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy
except ImportError:
    YOLOv8Wrapper_Huy = None

# =====================================================================================
# GLOBAL CONFIG & STATE
# =====================================================================================
imageWidth  = 640
imageHeight = 480

# Control state
throttle = 0.0
steering = 0.0
is_auto = False
current_mode = 0  # 0=BEV, 1=HSV, 2=LaneNet
mode_names = ["BEV (Sliding Window)", "HSV (Color Threshold)", "LaneNet (Deep Learning)"]

# Filters
steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
next(steeringFilter)

# Detectors
detectors = []
active_detector = None
yolo_display = None

# =====================================================================================
# HELPER FUNCTIONS
# =====================================================================================
def nothing(x): pass

def setup_trackbars(params):
    cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Controls", 400, 800)

    # 1. Warping (BEV only)
    cv2.createTrackbar("Warp Top W", "Controls", params.get('warp_w_top', 150), 640, nothing)
    cv2.createTrackbar("Warp Bot W", "Controls", params.get('warp_w_bot', 640), 1000, nothing)
    cv2.createTrackbar("Warp H", "Controls", params.get('warp_h', 180), 480, nothing)
    cv2.createTrackbar("Warp Y Off", "Controls", params.get('warp_y_offset', 0), 200, nothing)

    # 2. Thresholding (HSV/BEV)
    cv2.createTrackbar("S Min", "Controls", params.get('s_min', 50), 255, nothing)
    cv2.createTrackbar("S Max", "Controls", params.get('s_max', 255), 255, nothing) # Not strictly used in new classes but kept for compat
    
    # 3. Control Gains
    cv2.createTrackbar("Steer Gain", "Controls", int(params.get('steer_gain', 1.0)*10), 50, nothing)    
    cv2.createTrackbar("Curve Gain", "Controls", int(params.get('curve_gain', 200.0)), 1000, nothing)
    
    # 4. Calibration
    cv2.createTrackbar("Cam Offset (mm)", "Controls", int(params.get('camera_offset_m', 0.032)*1000), 200, nothing)
    cv2.createTrackbar("Lane W Ref (mm)", "Controls", int(params.get('lane_width_ref_m', 0.5)*1000), 1000, nothing)

def update_bev_detector_params(detector):
    """Update BEV detector parameters from trackbars"""
    detector.warp_w_top = cv2.getTrackbarPos("Warp Top W", "Controls")
    detector.warp_w_bot = cv2.getTrackbarPos("Warp Bot W", "Controls")
    detector.warp_h = cv2.getTrackbarPos("Warp H", "Controls")
    detector.warp_y_offset = cv2.getTrackbarPos("Warp Y Off", "Controls")
    detector.camera_offset_m = cv2.getTrackbarPos("Cam Offset (mm)", "Controls") / 1000.0
    detector.lane_width_ref_m = max(cv2.getTrackbarPos("Lane W Ref (mm)", "Controls") / 1000.0, 0.05)
    
    # Update config dict for gains
    detector.config['steer_gain'] = cv2.getTrackbarPos("Steer Gain", "Controls") / 10.0
    detector.config['curve_gain'] = cv2.getTrackbarPos("Curve Gain", "Controls")
    detector.config['camera_offset_m'] = detector.camera_offset_m
    detector.config['lane_width_ref_m'] = detector.lane_width_ref_m
    
    # Update internal state (recalculate matrix)
    detector._update_transform_matrix()

def save_bev_params(detector):
    detector.save_settings()
    print("BEV Parameters Saved.")

def draw_alpha_panel(img, top_left, bottom_right, color=(20, 20, 20), alpha=0.62):
    overlay = img.copy()
    cv2.rectangle(overlay, top_left, bottom_right, color, -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    cv2.rectangle(img, top_left, bottom_right, (180, 180, 180), 1)


def draw_text_outline(img, text, org, color=(255, 255, 255), scale=0.55, thickness=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def safe_get_trackbar(name, default=0):
    try:
        return cv2.getTrackbarPos(name, "Controls")
    except cv2.error:
        return default


def fmt_value(value, precision=3):
    try:
        if value is None:
            return "n/a"
        v = float(value)
        if np.isnan(v) or np.isinf(v):
            return "n/a"
        return f"{v:.{precision}f}"
    except (TypeError, ValueError):
        return str(value)


def get_tuning_lines(detector):
    lines = [f"Steer Gain: {safe_get_trackbar('Steer Gain', 10) / 10.0:.2f}"]
    if isinstance(detector, BEVLaneDetector):
        lines.extend([
            f"Warp Top/Bot: {safe_get_trackbar('Warp Top W', 150)}/{safe_get_trackbar('Warp Bot W', 640)}",
            f"Warp H/Yoff: {safe_get_trackbar('Warp H', 180)}/{safe_get_trackbar('Warp Y Off', 0)}",
            f"Curve Gain: {safe_get_trackbar('Curve Gain', 200)}",
            f"Cam Offset mm: {safe_get_trackbar('Cam Offset (mm)', 32)}",
            f"Lane Ref mm: {safe_get_trackbar('Lane W Ref (mm)', 500)}",
        ])
    elif isinstance(detector, HSVLaneDetector):
        lines.extend([
            f"S Min/Max: {safe_get_trackbar('S Min', 50)}/{safe_get_trackbar('S Max', 255)}",
            f"HSV lower: {detector.hsv_lower.tolist()}",
            f"HSV upper: {detector.hsv_upper.tolist()}",
            f"Slope Gain: {fmt_value(detector.slope_gain, 2)}",
        ])
    else:
        lines.append("No live tuning trackbars for this mode.")
    return lines


def get_detector_debug_lines(detector, result):
    raw = result.raw_data or {}
    lines = [
        f"Lane Valid: {result.is_valid}",
        f"Confidence: {fmt_value(result.confidence, 2)}",
        f"Left/Right: {result.left_lane_detected}/{result.right_lane_detected}",
    ]
    if isinstance(detector, BEVLaneDetector):
        lines.extend([
            f"L/R conf: {fmt_value(raw.get('left_confidence'), 2)}/{fmt_value(raw.get('right_confidence'), 2)}",
            f"Points L/R: {raw.get('left_points', 0)}/{raw.get('right_points', 0)}",
            f"Lane width px: {fmt_value(raw.get('lane_width_px'), 1)}",
        ])
    elif isinstance(detector, HSVLaneDetector):
        lines.extend([
            f"Lane pos: {raw.get('lane_position', 'unknown')}",
            f"Mask ratio: {fmt_value(raw.get('mask_ratio'), 3)}",
            f"L/R ratio: {fmt_value(raw.get('left_ratio'), 3)}/{fmt_value(raw.get('right_ratio'), 3)}",
            f"Slope/Int: {fmt_value(raw.get('slope'))}/{fmt_value(raw.get('intercept'), 1)}",
        ])
    else:
        lines.append(f"Raw keys: {', '.join(raw.keys()) if raw else 'none'}")
    return lines


def get_centering_guidance(detector, result, suggested_steering):
    """Return direction and steering command to remain near lane center."""
    raw = result.raw_data or {}
    steer_cmd = float(np.clip(suggested_steering, -0.5, 0.5))
    direction = "HOLD"
    reason = "no reliable lane"

    # Prefer lateral offset when available (most interpretable for BEV).
    if result.is_valid and isinstance(detector, BEVLaneDetector):
        offset = float(result.lateral_offset)
        if abs(offset) >= 0.03:
            direction = "LEFT" if offset > 0 else "RIGHT"
            reason = f"offset {offset:+.3f} from center"
        else:
            direction = "HOLD"
            reason = "offset near center"
    elif result.is_valid:
        if abs(steer_cmd) >= 0.03:
            direction = "LEFT" if steer_cmd > 0 else "RIGHT"
            reason = "from steering correction sign"
        else:
            direction = "HOLD"
            reason = "small steering correction"

    return direction, steer_cmd, reason


def draw_hud(
    img,
    result,
    mode_name,
    detector,
    is_auto,
    steering,
    throttle,
    raw_steering,
    suggested_steering,
    fps,
    loop_ms,
):
    h, w = img.shape[:2]
    guidance_dir, guidance_cmd, _ = get_centering_guidance(detector, result, suggested_steering)
    lane_status_color = (80, 255, 80) if result.is_valid else (80, 80, 255)
    mode_color = (0, 220, 255)
    status_color = (80, 255, 80) if is_auto else (90, 170, 255)

    # Main status panel
    draw_alpha_panel(img, (8, 8), (360, 235))
    draw_text_outline(img, f"Mode: {mode_name}", (18, 32), mode_color, 0.65, 2)
    draw_text_outline(img, f"Status: {'AUTO' if is_auto else 'MANUAL'}", (18, 58), status_color, 0.62, 2)
    draw_text_outline(img, f"FPS: {fps:5.1f}   Loop: {loop_ms:5.1f} ms", (18, 82), (255, 255, 255), 0.54, 1)
    draw_text_outline(img, "LANE DETECTED" if result.is_valid else "NO LANE", (18, 108), lane_status_color, 0.58, 2)
    draw_text_outline(img, f"Steer raw/LPF/applied: {raw_steering:+.3f} / {suggested_steering:+.3f} / {steering:+.3f}", (18, 132), (230, 230, 230), 0.48, 1)
    draw_text_outline(img, f"Throttle: {throttle:+.3f}", (18, 154), (230, 230, 230), 0.5, 1)
    draw_text_outline(img, f"Offset: {fmt_value(result.lateral_offset)}  Curvature: {fmt_value(result.curvature)}", (18, 176), (230, 230, 230), 0.5, 1)
    draw_text_outline(img, f"Detected L/R: {result.left_lane_detected}/{result.right_lane_detected}", (18, 198), (230, 230, 230), 0.5, 1)
    draw_text_outline(img, f"Guidance: {guidance_dir} | cmd {guidance_cmd:+.3f}", (18, 220), (160, 230, 255), 0.47, 1)

    # Detector raw panel
    draw_alpha_panel(img, (8, 245), (360, h - 90))
    draw_text_outline(img, "Detector Debug", (18, 268), (255, 255, 120), 0.56, 2)
    y = 292
    for line in get_detector_debug_lines(detector, result):
        draw_text_outline(img, line, (18, y), (220, 220, 220), 0.49, 1)
        y += 20
        if y > h - 100:
            break

    # Live tuning panel
    draw_alpha_panel(img, (w - 310, 8), (w - 8, 200))
    draw_text_outline(img, "Live Tuning", (w - 300, 32), (255, 255, 120), 0.6, 2)
    y = 56
    for line in get_tuning_lines(detector):
        draw_text_outline(img, line, (w - 300, y), (230, 230, 230), 0.5, 1)
        y += 22
        if y > 190:
            break

    # Controls/tips panel
    draw_alpha_panel(img, (w - 310, h - 115), (w - 8, h - 8))
    draw_text_outline(img, "Controls", (w - 300, h - 90), (255, 255, 120), 0.56, 2)
    draw_text_outline(img, "X: Auto  M: Mode  P: Save  Q: Quit", (w - 300, h - 68), (230, 230, 230), 0.5, 1)
    if isinstance(detector, BEVLaneDetector):
        tip = "Tip: Make lane lines parallel in Debug view."
    elif isinstance(detector, HSVLaneDetector):
        tip = "Tip: Tune S Min/Max until lane mask is clean."
    else:
        tip = "Tip: Check confidence and steering stability."
    draw_text_outline(img, tip, (w - 300, h - 44), (160, 230, 255), 0.47, 1)

    # Reference marker for center alignment
    cx = w // 2
    cv2.line(img, (cx, h), (cx, h - 90), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.circle(img, (cx, h - 45), 4, (255, 255, 255), -1, cv2.LINE_AA)


def build_hud_window(
    result,
    mode_name,
    detector,
    is_auto,
    steering,
    throttle,
    raw_steering,
    suggested_steering,
    fps,
    loop_ms,
):
    hud_img = np.zeros((620, 980, 3), dtype=np.uint8)
    draw_hud(
        hud_img,
        result,
        mode_name,
        detector,
        is_auto,
        steering,
        throttle,
        raw_steering,
        suggested_steering,
        fps,
        loop_ms,
    )
    return hud_img


def draw_result_overlay(img, mode_name, is_auto, result):
    """Keep camera view clear; only draw minimal status."""
    h, _ = img.shape[:2]
    draw_alpha_panel(img, (8, 8), (370, 76))
    draw_text_outline(img, f"{mode_name} | {'AUTO' if is_auto else 'MANUAL'}", (18, 34), (255, 255, 255), 0.62, 2)
    lane_color = (80, 255, 80) if result.is_valid else (80, 80, 255)
    draw_text_outline(img, "LANE OK" if result.is_valid else "NO LANE", (18, 60), lane_color, 0.58, 2)
    # Visual center reference
    cx = img.shape[1] // 2
    cv2.line(img, (cx, h), (cx, h - 60), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.circle(img, (cx, h - 30), 4, (255, 255, 255), -1, cv2.LINE_AA)


def init_yolo_display_wrapper():
    """Initialize YOLOv8Wrapper_Huy for unified display rendering if available."""
    if YOLOv8Wrapper_Huy is None:
        print("[Display] YOLOv8Wrapper_Huy not found. Using direct OpenCV display.")
        return None
    try:
        wrapper = YOLOv8Wrapper_Huy(imageWidth=imageWidth, imageHeight=imageHeight)
        print("[Display] Using YOLOv8Wrapper_Huy for result image rendering.")
        return wrapper
    except Exception as e:
        print(f"[Display] Failed to initialize YOLOv8Wrapper_Huy ({e}). Using direct OpenCV display.")
        return None


def build_result_visual(frame, mode_name, is_auto, result, detector, wrapper):
    """Build result view using YOLO wrapper when available, with safe fallback."""
    if wrapper is not None:
        try:
            wrapper.pre_process(frame)
            # Lane-following-only script: no object detection inference here.
            wrapper.processedResults = []
            wrapper.predictions = []
            wrapper.objectsDetected = []
            wrapper.set_lane_detector(detector)
            wrapper.set_lane_result(result)
            display_img = wrapper.post_process_render(showFPS=False, show_lane_overlay=True)
        except Exception as e:
            if not getattr(build_result_visual, "_warned_wrapper_fallback", False):
                print(f"[Display] Wrapper render failed ({e}). Falling back to OpenCV display.")
                build_result_visual._warned_wrapper_fallback = True
            display_img = frame.copy()
    else:
        display_img = frame.copy()

    draw_result_overlay(display_img, mode_name, is_auto, result)
    return display_img


def build_guidance_window(mode_name, detector, result, suggested_steering, applied_steering, fps):
    """Dedicated window that shows the key driving action."""
    guide = np.zeros((260, 900, 3), dtype=np.uint8)
    direction, steer_cmd, reason = get_centering_guidance(detector, result, suggested_steering)
    mag = abs(steer_cmd)
    intensity = int(np.clip((mag / 0.5) * 100.0, 0, 100))

    draw_alpha_panel(guide, (8, 8), (892, 252), color=(15, 15, 15), alpha=0.85)
    draw_text_outline(guide, f"Guidance | {mode_name}", (20, 35), (255, 255, 120), 0.8, 2)
    draw_text_outline(guide, f"Direction to center: {direction}", (20, 78), (230, 230, 230), 0.9, 2)
    draw_text_outline(guide, f"Apply steering command: {steer_cmd:+.3f}  (intensity {intensity}%)", (20, 118), (160, 230, 255), 0.72, 2)
    draw_text_outline(guide, f"Applied steering: {applied_steering:+.3f}  |  FPS: {fps:4.1f}", (20, 151), (220, 220, 220), 0.62, 1)
    draw_text_outline(guide, f"Reason: {reason}", (20, 180), (220, 220, 220), 0.58, 1)

    # Steering bar
    bar_left, bar_right, bar_y = 20, 880, 222
    bar_mid = (bar_left + bar_right) // 2
    cv2.line(guide, (bar_left, bar_y), (bar_right, bar_y), (110, 110, 110), 2, cv2.LINE_AA)
    cv2.line(guide, (bar_mid, bar_y - 12), (bar_mid, bar_y + 12), (240, 240, 240), 2, cv2.LINE_AA)
    steer_px = int((steer_cmd / 0.5) * ((bar_right - bar_left) // 2))
    bar_color = (80, 255, 80) if abs(steer_cmd) < 0.03 else (0, 220, 255)
    cv2.line(guide, (bar_mid, bar_y), (bar_mid + steer_px, bar_y), bar_color, 8, cv2.LINE_AA)
    draw_text_outline(guide, "LEFT", (bar_left, bar_y - 18), (200, 200, 200), 0.5, 1)
    draw_text_outline(guide, "RIGHT", (bar_right - 55, bar_y - 18), (200, 200, 200), 0.5, 1)

    return guide


def build_debug_visual(debug_img, frame, detector, result, fps):
    if debug_img is None:
        debug_img = np.zeros_like(frame)
        draw_text_outline(debug_img, "Detector did not provide a debug image.", (20, 40), (230, 230, 230), 0.6, 2)

    if debug_img.dtype != np.uint8:
        debug_img = cv2.normalize(debug_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    if isinstance(detector, BEVLaneDetector):
        if len(debug_img.shape) == 3:
            debug_gray = cv2.cvtColor(debug_img, cv2.COLOR_BGR2GRAY)
        else:
            debug_gray = debug_img
        debug_gray = cv2.equalizeHist(debug_gray)
        debug_img = cv2.cvtColor(debug_gray, cv2.COLOR_GRAY2BGR)
    elif len(debug_img.shape) == 2:
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)

    vis = debug_img.copy()
    h, w = vis.shape[:2]

    draw_alpha_panel(vis, (8, 8), (w - 8, 108))
    draw_text_outline(vis, f"{detector.__class__.__name__} Debug", (18, 34), (255, 255, 255), 0.65, 2)
    draw_text_outline(vis, f"FPS: {fps:5.1f}  Valid: {result.is_valid}  Conf: {fmt_value(result.confidence, 2)}", (18, 58), (230, 230, 230), 0.52, 1)
    draw_text_outline(vis, f"Steering: {result.steering_correction:+.3f}  Offset: {fmt_value(result.lateral_offset)}  Curv: {fmt_value(result.curvature)}", (18, 82), (230, 230, 230), 0.5, 1)

    return vis

# =====================================================================================
# MAIN PROGRAM
# =====================================================================================

# Initialize Hardware
if not IS_PHYSICAL_QCAR:
    from qvl.multi_agent import readRobots
    robotsDir = readRobots()
    Car1 = robotsDir["QC2_0"]
    myCam = QCarRealSense(mode='RGB', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight, video3dPort=18805)
    myCar = QCar(readMode=1, frequency=100, hilPort=Car1["hilPort"])
else:
    # calibrate = 'y' in input('do you want to recalibrate?(y/n)')
    myCam = QCarRealSense(mode='RGB', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
    myCar = QCar(readMode=1, frequency=100) 

# Initialize Detectors
print("Initializing Detectors...")
detectors.append(BEVLaneDetector())
detectors.append(HSVLaneDetector())
detectors.append(LaneNetDetector())

for d in detectors:
    d.initialize()
    # Try loading settings for BEV
    if isinstance(d, BEVLaneDetector):
        d.load_settings()

active_detector = detectors[current_mode]
yolo_display = init_yolo_display_wrapper()
if yolo_display is not None:
    yolo_display.set_lane_detector(active_detector)

# Initial setup of trackbars (using default or loaded params from BEV detector)
initial_params = {
    'warp_w_top': getattr(detectors[0], 'warp_w_top', 150),
    'warp_w_bot': getattr(detectors[0], 'warp_w_bot', 640),
    'warp_h': getattr(detectors[0], 'warp_h', 180),
    'warp_y_offset': getattr(detectors[0], 'warp_y_offset', 0),
    's_min': int(getattr(detectors[1], 'hsv_lower', np.array([10, 50, 100]))[1]),
    's_max': int(getattr(detectors[1], 'hsv_upper', np.array([45, 255, 255]))[1]),
    'steer_gain': detectors[0].config.get('steer_gain', 1.0),
    'curve_gain': detectors[0].config.get('curve_gain', 200.0),
    'camera_offset_m': getattr(detectors[0], 'camera_offset_m', 0.032),
    'lane_width_ref_m': getattr(detectors[0], 'lane_width_ref_m', 0.5),
}
setup_trackbars(initial_params)

print("\n" + "="*60)
print("Updated Multi-Algorithm Lane Following")
print("  X: Toggle Auto")
print("  M: Switch Algorithm (BEV -> HSV -> LaneNet)")
print("  P: Save Params (BEV only)")
print("  Q: Quit")
print("  Result Window: Camera + minimal status")
print("  HUD Window: Full debug/telemetry values")
print("  Guidance Window: Direction + steering input needed")
print("="*60 + "\n")

try:
    while True:
        start_time = time.time()
        
        # 1. Read Input
        myCam.read_RGB()
        frame = myCam.imageBufferRGB
        if frame is None or frame.size == 0: continue
        
        # Crop 40 pixels from bottom to remove noise/artifacts (Standard QCar practice)
        # However, detectors might handle this internally or expect full frame.
        # The detectors in lane_detectors.py are robust, but let's feed them the full frame 
        # unless they need cropping. HSVLaneDetector handles cropping internally.
        # BEVLaneDetector expects full frame to allow warping correctly.
        
        # 2. Update Params
        if isinstance(active_detector, BEVLaneDetector):
            update_bev_detector_params(active_detector)
        elif isinstance(active_detector, HSVLaneDetector):
            # Update HSV params locally
            s_min = cv2.getTrackbarPos("S Min", "Controls")
            s_max = cv2.getTrackbarPos("S Max", "Controls")
            # Assuming yellow detection mostly relies on S and V
            # Current defaults: lower=[10, 50, 100], upper=[45, 255, 255]
            # We map S Min/Max to the Saturation channel
            active_detector.hsv_lower[1] = s_min
            active_detector.hsv_upper[1] = s_max
            
            # Also update gains
            steer_gain = cv2.getTrackbarPos("Steer Gain", "Controls") / 10.0
            active_detector.slope_gain = steer_gain * 1.5 # Scale appropriately relative to default

        # 3. Detect
        result = active_detector.detect(frame)
        
        # 4. Control Logic
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('x'): 
            is_auto = not is_auto
            print(f"Auto Mode: {is_auto}")
            
        elif key == ord('m'):
            current_mode = (current_mode + 1) % len(detectors)
            active_detector = detectors[current_mode]
            print(f"Switched to: {mode_names[current_mode]}")
            
        elif key == ord('p'):
            if isinstance(active_detector, BEVLaneDetector):
                save_bev_params(active_detector)
            else:
                print("Param saving only available for BEV mode.")
                
        elif key == ord('q'): 
            break
        
        # Calculate Steering
        raw_steering = result.steering_correction if result.is_valid else 0.0
        if result.is_valid:
            # Apply filter
            suggested_steering = float(steeringFilter.send((np.clip(raw_steering, -0.5, 0.5), 0.033)))
        else:
            suggested_steering = 0.0

        if is_auto:
            if result.is_valid:
                steering = suggested_steering
                throttle = 0.05
            else:
                steering = 0.0
                throttle = 0.0
        else:
            # Manual control (optional, or just zero)
            # steering = 0.0 # uncomment to enable manual steering via WASD if implemented
            pass

        # 5. Actuate
        myCar.write(throttle, steering)
        
        # 6. Display
        loop_ms = (time.time() - start_time) * 1000.0
        fps = 1000.0 / max(loop_ms, 1e-3)

        # Show result image (via YOLOv8Wrapper_Huy if available)
        display_img = build_result_visual(
            frame,
            mode_names[current_mode],
            is_auto,
            result,
            active_detector,
            yolo_display,
        )
        cv2.imshow("Result", display_img)

        # Dedicated HUD + guidance windows
        hud_img = build_hud_window(
            result,
            mode_names[current_mode],
            active_detector,
            is_auto,
            steering,
            throttle,
            raw_steering,
            suggested_steering,
            fps,
            loop_ms,
        )
        cv2.imshow("HUD", hud_img)

        guidance_img = build_guidance_window(
            mode_names[current_mode],
            active_detector,
            result,
            suggested_steering,
            steering,
            fps,
        )
        cv2.imshow("Guidance", guidance_img)
        
        # Show debug images if available
        debug_img = active_detector.get_debug_image() if hasattr(active_detector, 'get_debug_image') else None
        cv2.imshow("Debug", build_debug_visual(debug_img, frame, active_detector, result, fps))

except KeyboardInterrupt:
    print("Interrupted")
finally:
    myCam.terminate()
    myCar.terminate()
    cv2.destroyAllWindows()
