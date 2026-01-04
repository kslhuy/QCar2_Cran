## task_lane_following.py
# Robust Lane Following with Bird's Eye View (BEV) & Curved Lane Detection
# Uses Perspective Transform + Sliding Window + Polynomial Fitting
# Keyboard controls: W/S = throttle, A/D = steer, X = auto mode, Q = quit

from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar
from pal.utilities.math import Filter
from hal.utilities.image_processing import ImageProcessing
from qvl.multi_agent import readRobots
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.products.qcar import QCarRealSense

import time
import numpy as np
import cv2
import json
import os
from collections import deque

# =====================================================================================
# BIRD'S EYE VIEW TRANSFORMER
# =====================================================================================
class BirdEyeView:
    def __init__(self, img_width, img_height, debug=False):
        self.img_width = img_width
        self.img_height = img_height
        self.debug = debug
        self.M = None
        self.Minv = None
        self.src_pts = None
        self.dst_pts = None

    def update_transform_matrix(self, params):
        """Update perspective transform matrix based on trackbar params"""
        # Source points (Trapezoid on the road)
        # Top width
        w_top = params.get('warp_w_top', 100)
        # Bottom width
        w_bot = params.get('warp_w_bot', 400)
        # Height of the trapezoid (distance from bottom)
        h_warp = params.get('warp_h', 200)
        # Vertical offset from bottom
        y_bot_offset = params.get('warp_y_offset', 0)
        
        # Center of the image
        cx = self.img_width // 2
        
        # Define the source trapezoid
        # Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        src = np.float32([
            [cx - w_top // 2, self.img_height - h_warp - y_bot_offset], # TL
            [cx + w_top // 2, self.img_height - h_warp - y_bot_offset], # TR
            [cx + w_bot // 2, self.img_height - y_bot_offset],          # BR
            [cx - w_bot // 2, self.img_height - y_bot_offset]           # BL
        ])
        
        self.src_pts = src
        
        # Destination points (Rectangle for Bird's Eye View)
        # We want to map this to a rect that fills most of the frame or a specific lane width
        dst_margin = self.img_width * 0.2
        dst = np.float32([
            [dst_margin, 0],
            [self.img_width - dst_margin, 0],
            [self.img_width - dst_margin, self.img_height],
            [dst_margin, self.img_height]
        ])
        
        self.dst_pts = dst
        
        # Compute the perspective transform matrix and its inverse
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def warp(self, img):
        """Apply perspective transform to image"""
        if self.M is None:
            return img
        return cv2.warpPerspective(img, self.M, (self.img_width, self.img_height), flags=cv2.INTER_LINEAR)

    def unwarp(self, img):
        """Unwarp image back to original perspective"""
        if self.Minv is None:
            return img
        return cv2.warpPerspective(img, self.Minv, (self.img_width, self.img_height), flags=cv2.INTER_LINEAR)
        
    def draw_source_points(self, img):
        """Draw the source polygon on the original image for calibration"""
        if self.src_pts is None:
            return img
        
        pts = self.src_pts.reshape((-1, 1, 2)).astype(np.int32)
        cv2.polylines(img, [pts], True, (0, 0, 255), 2)
        return img

# =====================================================================================
# CURVED LANE DETECTOR (SLIDING WINDOW)
# =====================================================================================
class LaneTracker:
    def __init__(self, nwindows=9, margin=100, minpix=50):
        self.nwindows = nwindows
        self.margin = margin
        self.minpix = minpix
        
        # Polynomial coefficients for recent frames
        self.best_fit_left = None
        self.best_fit_right = None
        
        # Lane finding confidence
        self.detected = False
        
        # History
        self.fit_left_history = deque(maxlen=5)
        self.fit_right_history = deque(maxlen=5)

    def preprocess(self, img, params):
        """
        Combined thresholding: Color + Gradient
        Input: Original RGB Image
        Output: Binary Map
        """
        # 1. HLS Color Threshold (S-channel is good for lanes in shadow)
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        s_channel = hls[:, :, 2]
        l_channel = hls[:, :, 1]
        
        # Thresholds from params
        s_thresh_min = params.get('s_min', 100)
        s_thresh_max = params.get('s_max', 255)
        
        # S-channel binary
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1
        
        # 2. Gradient Filter (Sobel X) - detects vertical lines
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3) # Take the derivative in x
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))
        
        sx_thresh_min = params.get('sx_min', 20)
        sx_thresh_max = params.get('sx_max', 100)
        
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= sx_thresh_min) & (scaled_sobel <= sx_thresh_max)] = 1
        
        # Combine the two binary thresholds
        combined_binary = np.zeros_like(sxbinary)
        # Use OR: if it's high saturation OR it's a strong vertical edge
        combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1
        
        return combined_binary * 255

    def find_lanes_sliding_window(self, binary_warped):
        """
        Find lane pixels using sliding window method
        """
        # Take a histogram of the bottom half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        
        # Create an output image to draw on and visualize the result
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        
        # Find the peak of the left and right halves of the histogram
        midpoint = int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # Set height of windows
        window_height = int(binary_warped.shape[0]//self.nwindows)
        
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []
        
        # Step through the windows one by one
        for window in range(self.nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            
            win_xleft_low = leftx_current - self.margin
            win_xleft_high = leftx_current + self.margin
            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin
            
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
            
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                            (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                            (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > self.minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpix:        
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 
        
        # Fit a second order polynomial to each
        left_fit = None
        right_fit = None
        
        if len(leftx) > 0:
            left_fit = np.polyfit(lefty, leftx, 2)
        if len(rightx) > 0:
            right_fit = np.polyfit(righty, rightx, 2)
            
        return left_fit, right_fit, out_img, left_lane_inds, right_lane_inds

    def smooth_fit(self, new_fit, history):
        if new_fit is None:
            if len(history) > 0:
                return np.mean(history, axis=0) # Return average of history
            return None
        
        history.append(new_fit)
        return np.mean(history, axis=0)

    def calculate_curvature_and_offset(self, left_fit, right_fit, img_height, img_width):
        """
        Calculate curvature and vehicle position offset from center.
        Note: This is in pixel space unless meters_per_pixel is calibrated.
        """
        if left_fit is None and right_fit is None:
            return 0.0, 0.0
        
        # Define y-value where we want radius of curvature (at the bottom of the image)
        y_eval = img_height
        
        # Calculate lane center at bottom of image
        left_x = 0
        right_x = img_width
        
        lane_visible_width = img_width # Default fallback
        lane_center = img_width / 2

        if left_fit is not None:
            left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
        
        if right_fit is not None:
            right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            
        if left_fit is not None and right_fit is not None:
            lane_center = (left_x + right_x) / 2
            lane_visible_width = right_x - left_x
        elif left_fit is not None:
             # Guess right lane if missing (assume standard lane width ~ 400-500px in warped view)
             lane_center = left_x + 250 # rough guess
        elif right_fit is not None:
             lane_center = right_x - 250
             
        # Center offset: Positive = Car is to the right of center (Need to steer Left)
        # Negative = Car is to the left of center (Need to steer Right)
        # Normalized to [-1, 1] relative to half image width (or lane width)
        car_position = img_width / 2
        offset_pixels = car_position - lane_center
        
        # Normalize offset: 1.0 means we are fully 1/2 lane width off
        # Avoid division by zero
        if lane_visible_width < 10: lane_visible_width = 100
        
        norm_offset = offset_pixels / (lane_visible_width / 2.0)
        
        # Calculate Curvature (radius) - Optional for advanced control
        # R = ((1 + (2Ay + B)^2)^1.5) / |2A|
        # We can use the 'A' component (new_fit[0]) directly as a proxy for steering
        # A > 0 : Curve Right, A < 0 : Curve Left
        avg_curvature_proxy = 0
        if left_fit is not None and right_fit is not None:
            avg_curvature_proxy = (left_fit[0] + right_fit[0]) / 2
        elif left_fit is not None:
            avg_curvature_proxy = left_fit[0]
        elif right_fit is not None:
            avg_curvature_proxy = right_fit[0]
            
        return norm_offset, avg_curvature_proxy

    def draw_lane(self, original_img, binary_warped, left_fit, right_fit, M_inv):
        """Draw the projected lane back onto the original image"""
        new_img = np.zeros_like(original_img)
        
        if left_fit is None or right_fit is None:
            return original_img

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        except TypeError:
            # Avoid errors if fit is bad
            return original_img

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        color_warp = np.zeros_like(binary_warped).astype(np.uint8)
        # Stack to make 3 channels
        color_warp = np.dstack((color_warp, color_warp, color_warp)) 

        # Draw the lane with green
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        
        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, M_inv, (original_img.shape[1], original_img.shape[0])) 
        
        # Combine the result with the original image
        result = cv2.addWeighted(original_img, 1, newwarp, 0.3, 0)
        return result

# =====================================================================================
# PARAMETER SAVE/LOAD
# =====================================================================================
def save_parameters(params, filename='lane_params_bev.json'):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, filename)
    try:
        with open(filepath, 'w') as f:
            json.dump(params, f, indent=4)
        print(f"Parameters saved to {filepath}")
    except Exception as e:
        print(f"Error saving parameters: {e}")

def load_parameters(filename='lane_params_bev.json'):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, filename)
    if os.path.exists(filepath):
        try:
            with open(filepath, 'r') as f:
                params = json.load(f)
            print(f"Parameters loaded from {filepath}")
            return params
        except Exception as e:
            print(f"Error loading parameters: {e}")
            return None
    return None

def nothing(x): pass

# =====================================================================================
# MAIN PROGRAM
# =====================================================================================

# Image parameters
imageWidth  = 640
imageHeight = 480

# Initialize Components
bev = BirdEyeView(imageWidth, imageHeight, debug=True)
tracker = LaneTracker()

# Initialize Hardware
if not IS_PHYSICAL_QCAR:
    robotsDir = readRobots()
    Car1 = robotsDir["QC2_0"]
    myCam = QCarRealSense(mode='RGB', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight, video3dPort=18805)
    myCar = QCar(readMode=1, frequency=100, hilPort=Car1["hilPort"])
else:
    calibrate = 'y' in input('do you want to recalibrate?(y/n)')
    myCam = QCarRealSense(mode='RGB', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
    myCar = QCar(readMode=1, frequency=100) 

# Setup Windows & Trackbars
cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Controls", 400, 800)

# Default params
default_params = {
    # Warp params
    'warp_w_top': 150,
    'warp_w_bot': 640,
    'warp_h': 180,
    'warp_y_offset': 0,
    
    # Threshold params
    's_min': 50,
    's_max': 255,
    'sx_min': 20,
    'sx_max': 100,
    
    # Steering gain
    'steer_gain': 1.0,
    'curve_gain': 200.0 # Gain for curvature feedforward
}

# Load params
saved = load_parameters()
if saved: default_params.update(saved)

# Create Trackbars
# 1. Warping
cv2.createTrackbar("Warp Top W", "Controls", default_params['warp_w_top'], 640, nothing)
cv2.createTrackbar("Warp Bot W", "Controls", default_params['warp_w_bot'], 1000, nothing)
cv2.createTrackbar("Warp H", "Controls", default_params['warp_h'], 480, nothing)
cv2.createTrackbar("Warp Y Off", "Controls", default_params['warp_y_offset'], 200, nothing)

# 2. Thresholding
cv2.createTrackbar("S Min", "Controls", default_params['s_min'], 255, nothing)
cv2.createTrackbar("S Max", "Controls", default_params['s_max'], 255, nothing)
cv2.createTrackbar("SX Min", "Controls", default_params['sx_min'], 255, nothing)
cv2.createTrackbar("SX Max", "Controls", default_params['sx_max'], 255, nothing)

# 3. Control
cv2.createTrackbar("Steer Gain", "Controls", int(default_params['steer_gain']*10), 50, nothing)    # 0.0 - 5.0
cv2.createTrackbar("Curve Gain", "Controls", int(default_params['curve_gain']), 1000, nothing)
cv2.createTrackbar("Inv Curve (0/1)", "Controls", 1, 1, nothing) # Default 1 (Inverted)

# Control state
throttle = 0.0
steering = 0.0
is_auto = False
param_save_timer = 0
key = -1

print("\n" + "="*60)
print("Updated BEV Lane Following")
print("  W/S: Manual Throttle")
print("  A/D: Manual Steer")
print("  X: Toggle Auto")
print("  P: Save Params")
print("  Q: Quit")
print("IMPORTANT: Adjust Warp Trackbars until lines are PARALLEL in 'Warped' view!")
print("="*60 + "\n")

steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
next(steeringFilter)

try:
    while True:
        start_time = time.time()
        
        # 1. Read Input
        myCam.read_RGB()
        frame = myCam.imageBufferRGB
        if frame is None or frame.size == 0: continue
        
        # 2. Get Params
        curr_params = {
            'warp_w_top': cv2.getTrackbarPos("Warp Top W", "Controls"),
            'warp_w_bot': cv2.getTrackbarPos("Warp Bot W", "Controls"),
            'warp_h': cv2.getTrackbarPos("Warp H", "Controls"),
            'warp_y_offset': cv2.getTrackbarPos("Warp Y Off", "Controls"),
            's_min': cv2.getTrackbarPos("S Min", "Controls"),
            's_max': cv2.getTrackbarPos("S Max", "Controls"),
            'sx_min': cv2.getTrackbarPos("SX Min", "Controls"),
            'sx_max': cv2.getTrackbarPos("SX Max", "Controls"),
            'steer_gain': cv2.getTrackbarPos("Steer Gain", "Controls") / 10.0,
            'curve_gain': cv2.getTrackbarPos("Curve Gain", "Controls")
        }
        
        # 3. Pipeline
        # Update Warper
        bev.update_transform_matrix(curr_params)
        
        # Preprocess (Thresholding)
        binary = tracker.preprocess(frame, curr_params)
        
        # Warp
        warped = bev.warp(binary)
        
        # Detect Lanes
        left_fit, right_fit, out_img_windows, _, _ = tracker.find_lanes_sliding_window(warped)
        
        # Smooth
        left_fit = tracker.smooth_fit(left_fit, tracker.fit_left_history)
        right_fit = tracker.smooth_fit(right_fit, tracker.fit_right_history)
        
        # Calculate Control Signals
        norm_offset, curvature_proxy = tracker.calculate_curvature_and_offset(left_fit, right_fit, imageHeight, imageWidth)
        
        # Visualization
        result_img = tracker.draw_lane(frame, warped, left_fit, right_fit, bev.Minv)
        # Draw source trapezoid on result for checking calibration
        result_img = bev.draw_source_points(result_img)
        
        # 4. Control Logic
        # 4. Control Logic
        # Handle Key Inputs (Essential for OpenCV window updates)
        if key == ord('x'): 
            is_auto = not is_auto
            print(f"Auto Mode: {is_auto}")
        if key == ord('q'): 
            save_parameters(curr_params)
            break
        if key == ord('p'): save_parameters(curr_params)
        
        # Always calculate the "Auto" steering for display/debug
        # P-Control on Offset + Feedforward on curvature
        # Invert Curvature Term if needed (Usually A > 0 means Left Turn -> Needs Negative Steer)
        invert_curve = cv2.getTrackbarPos("Inv Curve (0/1)", "Controls")
        
        p_term = -norm_offset * curr_params['steer_gain']
        
        if invert_curve == 1:
            ff_term = -curvature_proxy * curr_params['curve_gain']
        else:
            ff_term = curvature_proxy * curr_params['curve_gain']
            
        raw_steering = p_term + ff_term
        
        # Apply filter to suggested steering
        suggested_steering = steeringFilter.send((np.clip(raw_steering, -0.5, 0.5), 0.033))
        
        if is_auto:
            steering = suggested_steering
            throttle = 0.05 # Constant slow speed for testing
        else:
            # Manual Control Logic (Smoothed)
            # Throttle Logic
            if key == ord('w'): 
                throttle = 0.12
            elif key == ord('s'): 
                throttle = -0.12
            else:
                # Decay throttle when no key pressed (feels like coasting)
                throttle *= 0.9 
                if abs(throttle) < 0.01: throttle = 0.0

            # Steering Logic
            # We use a primitive "current_steering" state since CV2 key events are momentary
            if key == ord('a'): 
                steering = 0.3
            elif key == ord('d'): 
                steering = -0.3
            else:
                # Decay steering to center (self-centering)
                steering *= 0.8
                if abs(steering) < 0.01: steering = 0.0

        # 5. Actuate
        # myCar.read_write_std(throttle, steering)
        myCar.write(throttle, steering)
        
        # 6. Display
        # Show Warped View for Calibration
        cv2.imshow("Warped / Sliding Windows", out_img_windows)
        
        # Advanced Visualization of PID Terms
        # Draw a background box
        cv2.rectangle(result_img, (10, 10), (300, 180), (0,0,0), -1)
        
        def draw_bar(img, label, val, max_val, y_pos, color):
            # Bar width 150px. Center at 150.
            center_x = 150
            bar_w_half = 75
            
            # Normalize val (-1 to 1 typically)
            norm = np.clip(val, -max_val, max_val) / max_val
            
            # Draw axis
            cv2.line(img, (center_x - bar_w_half, y_pos), (center_x + bar_w_half, y_pos), (100,100,100), 1)
            cv2.line(img, (center_x, y_pos-5), (center_x, y_pos+5), (255,255,255), 1)
            
            # Draw value bar
            end_x = int(center_x + norm * bar_w_half)
            cv2.line(img, (center_x, y_pos), (end_x, y_pos), color, 4)
            
            # Text
            cv2.putText(img, f"{label}: {val:.3f}", (10, y_pos+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        draw_bar(result_img, "Offset (P)", p_term, 0.5, 30, (0, 255, 255))   # Yellow
        draw_bar(result_img, "Curve (FF)", ff_term, 0.5, 60, (255, 0, 255))   # Magenta
        draw_bar(result_img, "Total", raw_steering, 0.5, 90, (0, 255, 0))     # Green
        
        cv2.putText(result_img, f"Raw Curve: {curvature_proxy:.4f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        cv2.putText(result_img, f"Act Steer: {steering:.2f}", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        
        if is_auto:
             cv2.putText(result_img, "AUTO MODE", (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        else:
             cv2.putText(result_img, "MANUAL", (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,0), 2)

        cv2.imshow("Result", result_img)
        
        # Also show binary mask for debugging thresholds
        cv2.imshow("Binary Mask", binary)
        
        # Update UI interactions at the end to flush draw queue
        key = cv2.waitKey(1) & 0xFF

except KeyboardInterrupt:
    print("Interrupted")
finally:
    myCam.terminate()
    myCar.terminate()
    cv2.destroyAllWindows()
