## task_lane_following.py
# This example combines both the left csi and motor commands to
# allow the QCar to follow a yellow lane. Use the joystick to manually drive the QCar
# to a starting position and enable the line follower by holding the X button on the LogitechF710
# To troubleshoot your camera use the hardware_test_csi_camera_single.py found in the hardware tests

from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar , QCarCameras
from pal.utilities.math import Filter
from hal.utilities.image_processing import ImageProcessing
from qvl.multi_agent import readRobots
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR

import time
import numpy as np
import cv2
import math

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
steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
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
## Initialize the CSI cameras
# myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)
# myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)


myCam = QCarCameras( frameWidth=imageWidth, frameHeight=imageHeight , frameRate=sampleRate, enableFront=True, videoPort=Car1["videoPort"])

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## QCar and Keyboard Control Initialization
myCar = QCar(readMode=1, frequency=100, hilPort=Car1["hilPort"])

# Keyboard control state
throttle_cmd = 0.0
steering_cmd = 0.0
auto_mode_enabled = False  # True = autonomous lane following, False = manual control
target_speed = 0.02  # Default speed for autonomous mode

print("\n=== KEYBOARD CONTROLS ===")
print("MANUAL MODE:")
print("  W/S - Throttle forward/backward")
print("  A/D - Steer left/right")
print("  SPACE - Emergency stop")
print("\nAUTONOMOUS MODE:")
print("  X - Toggle autonomous lane following ON/OFF")
print("  +/- - Increase/decrease autonomous speed")
print("  Q - Quit")
print("========================\n")


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Main Loop
try:
	while True:
		start = time.time()
		# Capture RGB Image from CSI
		myCam.readAll()
		# Crop out a piece of the RGB to improve performance
		croppedRGB = myCam.csiFront.imageData[524:674, 0:820]

		# Convert to HSV for color-based detection
		hsvBuf = cv2.cvtColor(croppedRGB, cv2.COLOR_BGR2HSV)
		
		# Step 1: Detect BLACK road surface (low brightness in HSV V channel)
		# This helps us focus on the road area and ignore white objects outside
		blackRoadMask = ImageProcessing.binary_thresholding(frame=hsvBuf,
													lowerBounds=np.array([0, 0, 0]),        # Any hue, any saturation
													upperBounds=np.array([180, 255, 80]))   # Dark (low value/brightness)

		# Step 2: Detect WHITE lane markers (dashed lines or edge markers)
		# White has low saturation and high value in HSV
		whiteMask = ImageProcessing.binary_thresholding(frame=hsvBuf,
													lowerBounds=np.array([0, 0, 180]),      # Lower threshold to catch more white
													upperBounds=np.array([180, 60, 255]))   # Any hue, low saturation

		# Step 3: Detect YELLOW lane markers (center line separating lanes)
		# Yellow is in the 15-35 hue range in HSV
		yellowMask = ImageProcessing.binary_thresholding(frame=hsvBuf,
													lowerBounds=np.array([15, 60, 120]),    # Yellow hue range, lower thresholds
													upperBounds=np.array([40, 255, 255]))

		# Step 4: Apply morphological operations to clean up noise
		kernel = np.ones((3, 3), np.uint8)
		# Close gaps in dashed lines
		whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_CLOSE, kernel, iterations=2)
		yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernel, iterations=2)
		# Remove small noise
		whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_OPEN, kernel, iterations=1)
		yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel, iterations=1)
		
		# Step 5: Focus on lane markers that are near/on the black road
		# Dilate the black road mask slightly to include markers on edges
		roadArea = cv2.dilate(blackRoadMask, kernel, iterations=5)
		# Keep only white/yellow markers that are within or near the road area
		whiteMask = cv2.bitwise_and(whiteMask, roadArea)
		yellowMask = cv2.bitwise_and(yellowMask, roadArea)

		# Step 6: Combine both white and yellow detections
		binaryImage = cv2.bitwise_or(whiteMask, yellowMask)

		# Overlay detected lane markers over raw RGB image
		# Show white markers in cyan, yellow markers in yellow, and road in dim red
		binaryImage_norm = binaryImage / 255
		whiteMask_norm = whiteMask / 255
		yellowMask_norm = yellowMask / 255
		roadMask_norm = blackRoadMask / 255
		
		processed = myCam.csiFront.imageData.copy()
		# Black road -> Dim red overlay (for debugging)
		processed[524:674, 0:820, 2] = processed[524:674, 0:820, 2] * 0.7 + 30 * roadMask_norm
		# White markers -> Cyan overlay (bright)
		processed[524:674, 0:820, 1] = processed[524:674, 0:820, 1] + (255 - processed[524:674, 0:820, 1]) * whiteMask_norm  # Green
		processed[524:674, 0:820, 2] = processed[524:674, 0:820, 2] + (255 - processed[524:674, 0:820, 2]) * whiteMask_norm  # Blue
		# Yellow markers -> Yellow overlay (bright)
		processed[524:674, 0:820, 2] = processed[524:674, 0:820, 2] + (255 - processed[524:674, 0:820, 2]) * yellowMask_norm  # Red
		processed[524:674, 0:820, 1] = processed[524:674, 0:820, 1] + (255 - processed[524:674, 0:820, 1]) * yellowMask_norm  # Green
		
		# Use the combined binary image for lane following
		binaryImage = binaryImage_norm

		# Find slope and intercept of linear fit from the binary image
		slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binaryImage)

		# Calculate lane confidence based on detected pixels
		total_pixels = binaryImage.shape[0] * binaryImage.shape[1]
		detected_pixels = np.sum(binaryImage > 0)
		lane_confidence = min(100, (detected_pixels / total_pixels) * 100 * 5)  # Scale to 0-100%
		
		# steering from slope and intercept
		rawSteering = 1.5*(slope - 0.3419) + (1/150)*(intercept+5)
		steering = steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), dt))

		# Draw steering indicator on the processed image
		# Draw a line representing steering direction (dynamically positioned)
		crop_width = imageWidth // 2  # Cropped region is half the image width (0:820)
		crop_bottom = 674  # Bottom row of cropped region
		img_center_x = crop_width // 2  # Center of cropped width
		img_bottom_y = crop_bottom - 24  # Slightly above bottom of crop
		steering_indicator_length = 200
		steering_angle_display = -steering * 60  # Convert to degrees for visualization
		end_x = int(img_center_x + steering_indicator_length * np.sin(np.radians(steering_angle_display)))
		end_y = int(img_bottom_y - steering_indicator_length * np.cos(np.radians(steering_angle_display)))
		
		cv2.line(processed, (img_center_x, img_bottom_y), (end_x, end_y), (0, 255, 0), 3)
		cv2.circle(processed, (img_center_x, img_bottom_y), 5, (0, 255, 0), -1)
		
		# Add text overlays for steering and confidence
		font = cv2.FONT_HERSHEY_SIMPLEX
		font_scale = 0.7
		font_thickness = 2
		
		# Steering value (top-left)
		steering_text = f"Steering: {steering:.3f}" if not math.isnan(steering) else "Steering: N/A"
		cv2.putText(processed, steering_text, (10, 30), font, font_scale, (0, 255, 0), font_thickness)
		
		# Lane confidence (top-left, below steering)
		confidence_text = f"Confidence: {lane_confidence:.1f}%"
		confidence_color = (0, 255, 0) if lane_confidence > 30 else (0, 165, 255) if lane_confidence > 15 else (0, 0, 255)
		cv2.putText(processed, confidence_text, (10, 60), font, font_scale, confidence_color, font_thickness)
		
		# Mode indicator (top-right)
		mode_text = "AUTO" if auto_mode_enabled else "MANUAL"
		mode_color = (0, 255, 0) if auto_mode_enabled else (255, 255, 0)
		cv2.putText(processed, mode_text, (670, 30), font, font_scale, mode_color, font_thickness)

		# Display the RGB cropped RGB Image and the resized binary image
		cv2.imshow('Detection Overlay', cv2.resize(processed, (820,410)) )

		# Keyboard control - read key press (1ms wait)
		key = cv2.waitKey(1) & 0xFF
		
		# Manual controls (W/A/S/D)
		if key == ord('w'):
			throttle_cmd = min(throttle_cmd + 0.02, 0.3)
		elif key == ord('s'):
			throttle_cmd = max(throttle_cmd - 0.02, -0.3)
		elif key == ord('a'):
			steering_cmd = max(steering_cmd + 0.05, 0.5)
		elif key == ord('d'):
			steering_cmd = min(steering_cmd - 0.05, -0.5)
		
		# Autonomous mode toggle
		elif key == ord('x'):
			auto_mode_enabled = not auto_mode_enabled
			if auto_mode_enabled:
				print(f"🚗 AUTONOMOUS MODE ON - Speed: {target_speed:.2f}")
			else:
				print("🎮 MANUAL MODE ON")
				throttle_cmd = 0.0
				steering_cmd = 0.0
		
		# Speed adjustment for autonomous mode
		elif key == ord('=') or key == ord('+'):
			target_speed = min(target_speed + 0.02, 0.3)
			print(f"Target speed: {target_speed:.2f}")
		elif key == ord('-') or key == ord('_'):
			target_speed = max(target_speed - 0.02, 0.05)
			print(f"Target speed: {target_speed:.2f}")
		
		# Emergency stop
		elif key == ord(' '):
			throttle_cmd = 0.0
			steering_cmd = 0.0
			auto_mode_enabled = False
			print("⛔ EMERGENCY STOP")
		
		# Quit
		elif key == ord('q'):
			print("Quitting...")
			break
		
		# Determine final commands based on mode
		if auto_mode_enabled:
			# Autonomous lane following
			if not math.isnan(steering):
				final_steering = steering
				# Reduce speed when turning sharply
				speed_factor = np.cos(steering)
				final_throttle = target_speed * max(0.5, speed_factor)
			else:
				# No lane detected - stop
				final_steering = 0.0
				final_throttle = 0.0
			LEDs = np.array([1, 1, 0, 0, 0, 0, 0, 0])  # Front LEDs = autonomous
		else:
			# Manual control
			final_steering = steering_cmd
			final_throttle = throttle_cmd
			LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])  # Rear LEDs = manual
		
		# Write commands to QCar
		myCar.read_write_std(final_throttle, final_steering, None)
		end = time.time()
		dt = end - start

except KeyboardInterrupt:
	print("User interrupted!")
		
finally:
	# Terminate camera and QCar
	myCam.terminate()
	myCar.terminate()
	# gpad.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
