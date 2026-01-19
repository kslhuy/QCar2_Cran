## task_lane_following.py
# This example combines both the left csi and motor commands to
# allow the QCar to follow a yellow lane. Use the joystick to manually drive the QCar
# to a starting position and enable the line follower by holding the X button on the LogitechF710
# To troubleshoot your camera use the hardware_test_csi_camera_single.py found in the hardware tests

from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar,QCarCameras
from pal.utilities.math import Filter
from pal.utilities.probe import Probe
from qvl.multi_agent import readRobots
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR

# Import HSVLaneDetector for consistent lane detection algorithm
from lane_detectors import HSVLaneDetector, LanePosition

import time
import numpy as np
import cv2
import math
import yaml
import os

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Timing Parameters and methods
sampleRate = 60
sampleTime = 1/sampleRate
print('Sample Time: ', sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Additional parameters
imageWidth  = 1640
imageHeight = 820
cameraID 	= "2@tcpip://localhost:18813"

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#Setting Filter
steeringFilter = Filter().low_pass_first_order_variable(25, 0.1)
next(steeringFilter)
dt = 0.033

if not IS_PHYSICAL_QCAR:
    robotsDir = readRobots()

    # THIS VERSION OF VEHICLE CONTROL NEEDS THE CARS TO BE INITIALIZED USING THE MULTIAGENT CLASS
    Car1 = robotsDir["QC2_1"]
    calibrate=False
else:
    calibrate =  'y' in input('do you want to recalibrate?(y/n)')

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Load lane detection config from YAML file
config_path = os.path.join(os.path.dirname(__file__), 'config_lane_detection.yaml')
try:
    with open(config_path, 'r') as f:
        full_config = yaml.safe_load(f)
    hsv_config = full_config.get('hsv_detector', {})
    print(f"[Config] Loaded lane detection config from: {config_path}")
except Exception as e:
    print(f"[Config] Warning: Could not load config file: {e}")
    print("[Config] Using default parameters")
    hsv_config = {}

# Build detector config from YAML (with defaults as fallback)
lane_detector_config = {
    'crop_ratio_sim': hsv_config.get('crop_ratio_sim', 0.4),
    'crop_ratio_real': hsv_config.get('crop_ratio_real', 0.0),
    'hsv_lower': hsv_config.get('hsv_lower', [10, 50, 100]),
    'hsv_upper': hsv_config.get('hsv_upper', [45, 255, 255]),
    'target_slope': hsv_config.get('target_slope', 0.3419),
    'slope_gain': hsv_config.get('slope_gain', 1.5),
    'intercept_gain': hsv_config.get('intercept_gain', 1/150),
    'intercept_offset': hsv_config.get('intercept_offset', 5.0),
    'lane_detection_threshold': hsv_config.get('lane_detection_threshold', 0.01)
}

## Initialize HSVLaneDetector with config from YAML
laneDetector = HSVLaneDetector(config=lane_detector_config)
laneDetector.initialize()
print(f"[LaneDetector] Initialized HSVLaneDetector (Physical QCar: {IS_PHYSICAL_QCAR})")

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Initialize the CSI cameras
myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)
# myCam = QCarCameras( frameWidth=imageWidth, frameHeight=imageHeight , frameRate=sampleRate, enableFront=True, videoPort=Car1["videoPort"])

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## QCar Initialization
myCar = QCar(readMode=1, frequency=100, hilPort=Car1["hilPort"])

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Keyboard Control State Variables
throttle = 0.0
manual_steering = 0.0
lane_follow_enabled = False

print("\n=== Keyboard Controls ===")
print("HOLD W/S: Throttle forward/backward (continuous)")
print("HOLD A/D: Manual steering left/right (continuous)")
print("X: Toggle lane following / assist mode")
print("+/-: Increase/decrease target_slope (for tuning)")
print("T: Toggle HSV tuning mode")
print("  1-6: Select HSV component (1=H_lo, 2=S_lo, 3=V_lo, 4=H_up, 5=S_up, 6=V_up)")
print("  i/k or [/]: Adjust selected HSV value (increase/decrease)")
print("P: Save current parameters to config file")
print("Spacebar: Emergency stop")
print("ESC or Q: Quit")
print("========================\n")

# Store current tuning parameters (can be modified at runtime)
tuning_params = {
    'target_slope': lane_detector_config['target_slope'],
    'slope_gain': lane_detector_config['slope_gain'],
    'intercept_gain': lane_detector_config['intercept_gain'],
    'intercept_offset': lane_detector_config['intercept_offset'],
    'hsv_lower': list(lane_detector_config['hsv_lower']),  # [H, S, V]
    'hsv_upper': list(lane_detector_config['hsv_upper']),  # [H, S, V]
}

# HSV tuning state
hsv_tuning_mode = False
hsv_selected_component = 0  # 0-5: H_lo, S_lo, V_lo, H_up, S_up, V_up
hsv_component_names = ['H_lower', 'S_lower', 'V_lower', 'H_upper', 'S_upper', 'V_upper']
hsv_step_sizes = [1, 5, 5, 1, 5, 5]  # Different step sizes for H vs S/V

def save_tuning_to_config():
    """Save current tuning parameters to config YAML file."""
    global full_config
    try:
        # Update the hsv_detector section with tuned values
        if 'hsv_detector' not in full_config:
            full_config['hsv_detector'] = {}
        full_config['hsv_detector']['target_slope'] = float(tuning_params['target_slope'])
        full_config['hsv_detector']['slope_gain'] = float(tuning_params['slope_gain'])
        full_config['hsv_detector']['intercept_gain'] = float(tuning_params['intercept_gain'])
        full_config['hsv_detector']['intercept_offset'] = float(tuning_params['intercept_offset'])
        full_config['hsv_detector']['hsv_lower'] = [int(v) for v in tuning_params['hsv_lower']]
        full_config['hsv_detector']['hsv_upper'] = [int(v) for v in tuning_params['hsv_upper']]
        
        with open(config_path, 'w') as f:
            yaml.dump(full_config, f, default_flow_style=False, sort_keys=False)
        print(f"[Config] Saved tuning parameters to: {config_path}")
        print(f"         target_slope: {tuning_params['target_slope']:.4f}")
        print(f"         hsv_lower: {tuning_params['hsv_lower']}")
        print(f"         hsv_upper: {tuning_params['hsv_upper']}")
        return True
    except Exception as e:
        print(f"[Config] Error saving config: {e}")
        return False

def update_hsv_value(direction):
    """Update the selected HSV component."""
    global hsv_selected_component
    step = hsv_step_sizes[hsv_selected_component] * direction
    
    if hsv_selected_component < 3:  # Lower bounds
        idx = hsv_selected_component
        tuning_params['hsv_lower'][idx] = int(np.clip(
            tuning_params['hsv_lower'][idx] + step,
            0, 255 if idx > 0 else 179  # H max is 179 in OpenCV
        ))
        laneDetector.hsv_lower = np.array(tuning_params['hsv_lower'])
    else:  # Upper bounds
        idx = hsv_selected_component - 3
        tuning_params['hsv_upper'][idx] = int(np.clip(
            tuning_params['hsv_upper'][idx] + step,
            0, 255 if idx > 0 else 179
        ))
        laneDetector.hsv_upper = np.array(tuning_params['hsv_upper'])
    
    print(f"[HSV] {hsv_component_names[hsv_selected_component]}: "
          f"Lower={tuning_params['hsv_lower']} Upper={tuning_params['hsv_upper']}")


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Main Loop
try:
	while True:
		start = time.time()
		# Capture RGB Image from CSI
		myCam.read()
		
		# Use HSVLaneDetector for lane detection (consistent algorithm)
		detection_result = laneDetector.detect(myCam.imageData)
		
		# Get lane detection data
		slope = detection_result.raw_data.get('slope', np.nan) if detection_result.raw_data else np.nan
		intercept = detection_result.raw_data.get('intercept', np.nan) if detection_result.raw_data else np.nan
		lane_position = laneDetector.get_lane_position()
		
		# Create binary visualization for overlay (crop bottom 40% for display)
		crop_start = int(imageHeight * 0.6)  # Bottom 40%
		croppedRGB = myCam.imageData[crop_start:, :, :]
		hsvBuf = cv2.cvtColor(croppedRGB, cv2.COLOR_BGR2HSV)
		# Use tuned HSV values for visualization
		binaryImage = cv2.inRange(hsvBuf, 
								  np.array(tuning_params['hsv_lower']), 
								  np.array(tuning_params['hsv_upper']))


		# Overlay detected yellow lane over raw RGB image
		binaryImage=binaryImage/255
		processed = myCam.imageData.copy()
		crop_end = imageHeight  # crop goes to bottom
		# Handle different cropped region size for overlay
		overlay_h = processed[crop_start:crop_end, :, :].shape[0]
		overlay_w = min(binaryImage.shape[1], processed.shape[1])
		binaryImage_resized = binaryImage[:overlay_h, :overlay_w]
		processed[crop_start:crop_end, :overlay_w, 2] = processed[crop_start:crop_end, :overlay_w, 2] + (255-processed[crop_start:crop_end, :overlay_w, 2])*binaryImage_resized
		processed[crop_start:crop_end, :overlay_w, 1] = processed[crop_start:crop_end, :overlay_w, 1]*(1-binaryImage_resized)
		processed[crop_start:crop_end, :overlay_w, 0] = processed[crop_start:crop_end, :overlay_w, 0]*(1-binaryImage_resized)
		
		# Draw detection region rectangle (green box)
		cv2.rectangle(processed, (0, crop_start), (overlay_w, crop_end), (0, 255, 0), 3)
		cv2.putText(processed, "Detection Zone", (10, crop_start - 10), 
					cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

		# Use steering from HSVLaneDetector (consistent algorithm)
		rawSteering = detection_result.steering_correction if detection_result.is_valid else 0.0
		steering = steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), dt))

		# Keyboard input handling - CONTINUOUS (hold-based)
		key = cv2.waitKey(1) & 0xFF
		
		# Continuous throttle control (hold to apply)
		if key == ord('w'):
			throttle = 0.08  # Full forward when holding W
		elif key == ord('s'):
			throttle = -0.08  # Full backward when holding S
		else:
			# Decay throttle when not pressing W/S
			throttle *= 0.8  # Gradual decay
			if abs(throttle) < 0.01:
				throttle = 0.0
		
		# Continuous steering control (hold to apply)
		if key == ord('a'):
			manual_steering = 0.4  # Steer left when holding A
		elif key == ord('d'):
			manual_steering = -0.4  # Steer right when holding D
		else:
			# Decay steering when not pressing A/D
			manual_steering *= 0.5  # Quick decay for responsive steering
			if abs(manual_steering) < 0.02:
				manual_steering = 0.0
		
		# Tuning controls: +/- to adjust target_slope
		if key == ord('+') or key == ord('='):  # = key also works (same key without shift)
			tuning_params['target_slope'] += 0.01
			laneDetector.target_slope = tuning_params['target_slope']
			print(f"[Tuning] target_slope: {tuning_params['target_slope']:.4f}")
		elif key == ord('-') or key == ord('_'):
			tuning_params['target_slope'] -= 0.01
			laneDetector.target_slope = tuning_params['target_slope']
			print(f"[Tuning] target_slope: {tuning_params['target_slope']:.4f}")
		
		# Toggle HSV tuning mode with T key
		if key == ord('t') or key == ord('T'):
			hsv_tuning_mode = not hsv_tuning_mode
			print(f"[HSV Tuning] Mode: {'ENABLED' if hsv_tuning_mode else 'DISABLED'}")
		
		# HSV component selection (1-6 keys)
		if hsv_tuning_mode:
			if key >= ord('1') and key <= ord('6'):
				hsv_selected_component = key - ord('1')
				print(f"[HSV Tuning] Selected: {hsv_component_names[hsv_selected_component]}")
			# i/k or [/] to adjust values (more reliable than arrow keys)
			elif key == ord('i') or key == ord(']') or key == 0 or key == 2490368:  # i, ], or Up arrow
				update_hsv_value(1)
			elif key == ord('k') or key == ord('[') or key == 1 or key == 2621440:  # k, [, or Down arrow
				update_hsv_value(-1)
		
		# Save tuning parameters with P key
		if key == ord('p') or key == ord('P'):
			save_tuning_to_config()
		
		# Toggle lane following
		if key == ord('x'):
			lane_follow_enabled = not lane_follow_enabled
			print(f"Lane following: {'ENABLED' if lane_follow_enabled else 'DISABLED'}")
		
		# Quit
		if key == 27 or key == ord('q'):  # ESC or Q to quit
			print("Quitting...")
			break
		
		# Spacebar to emergency stop
		if key == ord(' '):
			throttle = 0.0
			manual_steering = 0.0

		# Build QCar command - Assist mode with manual influence
		if lane_follow_enabled:
			# Lane following with manual steering assist
			QCarCommand = np.array([0.05 + throttle , 0.0])
			if math.isnan(steering):
				QCarCommand[1] = manual_steering  # Fall back to manual if no lane detected
			else:
				# Blend lane steering with manual input (70% lane, 30% manual)
				QCarCommand[1] =  steering +  manual_steering
			QCarCommand[0] = QCarCommand[0] * np.cos(QCarCommand[1])
		else:
			# Pure manual control
			QCarCommand = np.array([throttle, manual_steering])
		
		# Add telemetry overlay on the processed image
		y_offset = 30
		line_height = 35
		font = cv2.FONT_HERSHEY_SIMPLEX
		font_scale = 0.8
		font_thickness = 2
		
		# Add dark semi-transparent background for right-side text (better visibility)
		overlay = processed.copy()
		cv2.rectangle(overlay, (830, 5), (1620, y_offset + 10*line_height), (0, 0, 0), -1)
		processed = cv2.addWeighted(overlay, 0.6, processed, 0.4, 0)
		
		# Mode indicator with color coding
		mode_color = (0, 255, 0) if lane_follow_enabled else (0, 165, 255)  # Green if enabled, orange if manual
		mode_text = "LANE FOLLOW" if lane_follow_enabled else "MANUAL"
		cv2.putText(processed, f"Mode: {mode_text}", (850, y_offset), 
					font, font_scale, mode_color, font_thickness)
		
		# Steering values - Always show lane detection
		cv2.putText(processed, f"Lane Steering: {steering:.3f}", (850, y_offset + line_height), 
					font, font_scale, (255, 255, 255), font_thickness)
		cv2.putText(processed, f"Manual Input: {manual_steering:.3f}", (850, y_offset + 2*line_height), 
					font, font_scale, (200, 150, 255), font_thickness)
		
		# Command values
		cv2.putText(processed, f"Throttle: {QCarCommand[0]:.3f}", (850, y_offset + 3*line_height), 
					font, font_scale, (100, 200, 255), font_thickness)
		cv2.putText(processed, f"Cmd Steering: {QCarCommand[1]:.3f}", (850, y_offset + 4*line_height), 
					font, font_scale, (100, 200, 255), font_thickness)
		
		# Lane position indicator (NEW)
		lane_pos_text = lane_position.value.upper().replace('_', ' ')
		if lane_position == LanePosition.LEFT_LANE:
			lane_pos_color = (255, 200, 100)  # Light blue for left lane
		elif lane_position == LanePosition.RIGHT_LANE:
			lane_pos_color = (100, 200, 255)  # Orange for right lane
		else:
			lane_pos_color = (128, 128, 128)  # Gray for unknown
		cv2.putText(processed, f"Lane: {lane_pos_text}", (850, y_offset + 5*line_height), 
					font, font_scale, lane_pos_color, font_thickness)
		
		# Lane detection parameters
		cv2.putText(processed, f"Slope: {slope:.3f}", (850, y_offset + 6*line_height), 
					font, font_scale, lane_pos_color, font_thickness)
		cv2.putText(processed, f"Target Slope: {tuning_params['target_slope']:.4f}", (850, y_offset + 7*line_height), 
					font, font_scale, (0, 255, 255), font_thickness)  # Yellow for tunable param
		cv2.putText(processed, f"Intercept: {intercept:.1f}", (850, y_offset + 8*line_height), 
					font, font_scale, lane_pos_color, font_thickness)
		
		# HSV Tuning display (left side of screen)
		hsv_x = 15
		hsv_y = 35
		hsv_line_h = 28
		hsv_font_scale = 0.7
		
		# Add dark background for HSV display (left side)
		overlay_hsv = processed.copy()
		if hsv_tuning_mode:
			cv2.rectangle(overlay_hsv, (5, 10), (280, hsv_y + 9 * hsv_line_h), (0, 0, 0), -1)
		else:
			cv2.rectangle(overlay_hsv, (5, 10), (200, hsv_y + 50), (0, 0, 0), -1)
		processed = cv2.addWeighted(overlay_hsv, 0.6, processed, 0.4, 0)
		
		# HSV mode indicator
		if hsv_tuning_mode:
			cv2.putText(processed, "HSV TUNING [T to exit]", (hsv_x, hsv_y), 
						font, hsv_font_scale, (0, 255, 255), 2)
			
			# Show all HSV values with highlight on selected
			for i, name in enumerate(hsv_component_names):
				if i < 3:
					value = tuning_params['hsv_lower'][i]
				else:
					value = tuning_params['hsv_upper'][i - 3]
				
				# Highlight selected component
				if i == hsv_selected_component:
					color = (0, 255, 255)  # Bright yellow for selected
					prefix = "> "
					thickness = 2
				else:
					color = (255, 255, 255)  # White for others
					prefix = "  "
					thickness = 1
				
				cv2.putText(processed, f"{prefix}{i+1}: {name} = {value}", 
							(hsv_x, hsv_y + (i+1) * hsv_line_h), 
							font, hsv_font_scale, color, thickness)
			
			# Instructions
			cv2.putText(processed, "i/k or [/]: adjust", (hsv_x, hsv_y + 8 * hsv_line_h), 
						font, 0.55, (150, 255, 150), 1)  # Light green
		else:
			# Compact HSV display when not tuning
			hsv_lo = tuning_params['hsv_lower']
			hsv_up = tuning_params['hsv_upper']
			cv2.putText(processed, f"HSV Lo: {hsv_lo}", (hsv_x, hsv_y), 
						font, 0.6, (200, 200, 200), 1)
			cv2.putText(processed, f"HSV Up: {hsv_up}", (hsv_x, hsv_y + 22), 
						font, 0.6, (200, 200, 200), 1)
			cv2.putText(processed, "[T] HSV tune", (hsv_x, hsv_y + 44), 
						font, 0.55, (100, 255, 100), 1)  # Light green hint
		
		# FPS counter
		fps = 1.0 / dt if dt > 0 else 0
		cv2.putText(processed, f"FPS: {fps:.1f}", (850, y_offset + 9*line_height), 
					font, font_scale, (255, 255, 0), font_thickness)
		
		# Display the processed image with all overlays
		cv2.imshow('Detection Overlay', cv2.resize(processed, (820, 410)))
		
		LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
		myCar.read_write_std(QCarCommand[0], QCarCommand[1])
		end = time.time()
		dt = end - start

except KeyboardInterrupt:
	print("User interrupted!")
		
finally:
	# Terminate camera and QCar
	myCam.terminate()
	myCar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
